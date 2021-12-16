#!/usr/bin/env python3
import rospy
# --- Messages ---
from apriltag_ros.msg import AprilTagDetectionArray
# --- Functions ---
import numpy as np
# from scipy.linalg import inv
from datetime import datetime
# --- Transforms ---
import tf2_ros
from scipy.spatial.transform import Rotation as R

############ GLOBAL VARIABLES ###################
filepath = None # file where tags will be saved.
DT = 1 # period of timer that gets robot transform T_BO.
tags = {} # dictionary of tag poses, keyed by ID.
# --- Robot characteristics ---
r = 0.033 # wheel radius (m).
w = 0.16 # chassis width (m).
# --- Transforms ---
tf_listener = None
tf_buffer = None
T_CO = None # tf between cam and origin.
# --- TF Frames ---
# we can check for these with 'rosrun tf tf_monitor' while everything is running.
TF_ORIGIN = 'map'
# TF_ROBOT_BASE = 'base_link'
# TF_CAMERA = 'raspicam'       # for hardware
TF_CAMERA = 'camera_rgb_optical_frame'    # for simulation
##########################################


def get_tag_detection(tag_msg):
    """
    Detect an AprilTag's pose relative to the camera.
    Update the list if something was detected.
    """
    # verify there is at least one tag detected.
    if len(tag_msg.detections) == 0:
        return
    
    # do for all detected tags.
    for i in range(len(tag_msg.detections)):
        tag_id = tag_msg.detections[i].id
        tag_pose = tag_msg.detections[i].pose.pose.pose
        # use this to make goal pose in robot base frame.
        t = [tag_pose.position.x,tag_pose.position.y,tag_pose.position.z]
        q = [tag_pose.orientation.w, tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z]
        # make it into an affine matrix.
        r = R.from_quat(q).as_matrix()
        # make affine matrix for transformation.
        T_AC = np.array([[r[0][0],r[0][1],r[0][2],t[0]],
                        [r[1][0],r[1][1],r[1][2],t[1]],
                        [r[2][0],r[2][1],r[2][2],t[2]],
                        [0,0,0,1]])
        # we now have pose of tag in cam frame.
        # print("T_AC\n{}".format(T_AC))

        # calculate global pose of the tag, unless the TF failed to be setup.
        if T_CO is None:
            print("Found tag, but cannot create global transform.")
            return
        # print("T_CO\n{}".format(T_CO))
        T_AO =T_AC@T_CO
        # print("TAO\n{}".format(T_AO))

        # strip out z-axis parts AFTER transforming, to change from SE(3) to SE(2).
        #T_AO = np.delete(T_AO,2,0) # delete 3rd row.
        #T_AO = np.delete(T_AO,2,1) # delete 3rd column.

        # update the dictionary with this tag.
        if tag_id in tags.keys():
            print('UPDATING TAG: ', tag_id)
            # update using learning rate.
            # - use L=0 to throw away old data in favor of new.
            L = 0.9
            tags[tag_id] = np.add(L * tags[tag_id], (1-L) * T_AO)
        else: 
            print('FOUND NEW TAG: ', tag_id)
            # create a new entry for this tag.
            tags[tag_id] = T_AO


def get_transform(TF_TO, TF_FROM):
    global tf_buffer
    """
    Get the expected transform from tf.
    Use translation and quaternion from tf to construct a pose in SE(3).
    """
    try:
        # get most recent relative pose from the tf service.
        pose = tf_buffer.lookup_transform(TF_TO, TF_FROM,rospy.Time(0), rospy.Duration(4))
    except Exception as e:
        # requested transform was not found.
        print("Transform from " + TF_FROM + " to " + TF_TO + " not found.")
        print("Exception: ", e)
        return None
    
    # extract translation and quaternion from tf pose.
    transformT = [pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z]
    transformQ = (
        pose.transform.rotation.x,
        pose.transform.rotation.y,
        pose.transform.rotation.z,
        pose.transform.rotation.w)
    # get equiv rotation matrix from quaternion.
    r = R.from_quat(transformQ).as_matrix()

    # make affine matrix for transformation.
    return np.array([[r[0][0],r[0][1],r[0][2],transformT[0]],
                    [r[1][0],r[1][1],r[1][2],transformT[1]],
                    [r[2][0],r[2][1],r[2][2],transformT[2]],
                    [0,0,0,1]])


def timer_callback(event):
    """
    Update T_CO with newest pose from Cartographer.
    Save tags to file.
    """
    global T_CO
    T_CO = get_transform(TF_TO=TF_CAMERA, TF_FROM=TF_ORIGIN)
    # save tags to file.
    save_tags_to_file(tags)
    

def save_tags_to_file(tags):
    """
    We created a file whose name is the current time when the node is launched.
    All tags and IDs are saved to this file.
    This will recreate the file every timestep, so when the node ends, 
        the file should reflect the most updated set of tag poses.
    """
    if not tags:
        # don't create the file if there aren't any tags detected yet.
        return
    data_for_file = []
    for id in tags.keys():
        print(id, tags[id]) # print to console for debugging.
        data_for_file.append("id: " + str(id))
        for row in tags[id]:
            data_for_file.append(list(row))
        data_for_file.append("---------------------------------------")
    np.savetxt(filepath, data_for_file, fmt="%s", delimiter=",")


def main():
    global tf_listener, tf_buffer, filepath
    rospy.init_node('tag_tracking_node')

    # generate filepath that tags will be written to.
    dt = datetime.now()
    run_id = dt.strftime("%Y-%m-%d-%H-%M-%S")
    filepath = "tags_" + str(run_id) + ".txt"

    # setup TF service.
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(1))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # subscribe to apriltag detections.
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, get_tag_detection, queue_size=1)

    rospy.Timer(rospy.Duration(DT), timer_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
