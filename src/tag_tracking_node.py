#!/usr/bin/env python
import rospy
# --- Messages ---
from apriltag_ros.msg import AprilTagDetectionArray
# --- Functions ---
import numpy as np
from scipy.linalg import inv
from datetime import datetime
import atexit
# --- Transforms ---
import tf
from scipy.spatial.transform import Rotation as R

############ GLOBAL VARIABLES ###################
DT = 1 # period of timer that gets robot transform T_BO.
tags = {} # dictionary of tag poses, keyed by ID.
# --- Robot characteristics ---
r = 0.033 # wheel radius (m)
w = 0.16 # chassis width (m)
X0 = np.array([[1,0,0],[0,1,0],[0,0,1]]) # initial pose
# --- Transforms (homogenous matrices) ---
tf_listener = None # for getting T_CB
T_BO = X0 #None # origin->base
T_CB = None # base->cam
T_AC = None # cam->tag
T_AO = None # origin->tag
# --- TF TOPICS ---
# NOTE we can check for these with 'rosrun tf tf_monitor' while everything is running.
TF_ORIGIN = '' #TODO get from tf_monitor while cartographer is running
TF_ROBOT_BASE = 'base_link' #'/base_footprint'
TF_CAMERA = '/camera_link'
##########################################

def get_tag_detection(tag_msg):
    """
    Detect an AprilTag's pose relative to the camera.
    Update the list if something was detected.
    """
    try:
        tag_id = tag_msg.detections[0].id
        tag_pose = tag_msg.detections[0].pose.pose.pose
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
        # invert to get tf we want. NOTE not sure if this is necessary.
        T_AC = inv(T_AC)
    except:
        # no tag was detected. do nothing.
        return

    # calculate global pose of the tag.
    # NOTE for now, the robot will not be moving, so we can treat cam frame as an origin.
    T_AO = T_AC #* T_CB * T_BO
    # strip out z-axis parts AFTER transforming, to change from SE(3) to SE(2).
    # T_AO = np.delete(T_AO,2,0) # delete 3rd row.
    # T_AO = np.delete(T_AO,2,1) # delete 3rd column.

    # update the dictionary with this tag.
    if tag_id in tags.keys():
        print('UPDATING TAG: ', tag_id)
        # update using learning rate.
        # - use L=0 to throw away old data in favor of new.
        L = 0.9
        tags[tag_id] = np.add(L * tags[tag_id], (1-L) * T_AO)
    else: 
        print('FOUND NEW TAG: ', tag_id)
        # this is a new tag.
        tags[tag_id] = T_AO


def save_tags_on_exit():
    """
    This function should be called when the node exits (via Ctrl-C).
    It should save the 'tags' dict items to a text file.
    We create a folder whose name is the current time.
    Each tag pose is saved as a separate CSV, named as their ID.
    """
    # generate filepath.
    dt = datetime.now()
    run_id = dt.strftime("%Y-%m-%d-%H-%M-%S")
    filepath = "~/" + str(run_id) + "/"
    # save all tags.
    for id in tags.keys():
        np.savetxt(filepath+str(id)+".csv", tags[id], delimiter=",")


def get_transform(TF_FROM, TF_TO):
    """
    Get the expected transform from tf.
    Use translation and quaternion from tf to construct a pose in SE(2).
    """
    try:
        # get relative pose from the tf service. Allow up to 1 second wait.
        (t,q) = tf_listener.lookupTransform(TF_FROM, TF_TO, rospy.Duration(1.0))
    except:
        # requested transform was not found within the 1.0 second Duration.
        print("transform between "+ TF_FROM +"and"+ TF_TO + " not found.")
        return
    # get equiv rotation matrix from quaternion.
    r = R.from_quat(q).as_matrix()
    # make affine matrix for transformation.
    return np.array([[r[0][0],r[0][1],r[0][2],t[0]],
                    [r[1][0],r[1][1],r[1][2],t[1]],
                    [r[2][0],r[2][1],r[2][2],t[2]],
                    [0,0,0,1]])


def timer_callback(event):
    """
    Update T_BO with newest pose from Cartographer.
    """
    global T_BO
    # TODO uncomment this when it's working.
    #T_BO = get_transform(TF_ORIGIN, TF_ROBOT_BASE)
    # print list of tags to console.
    # print(tags)
    for id in tags.keys():
        print(id, tags[id])


def main():
    global tf_listener, T_CB
    rospy.init_node('tag_tracking_node')

    # setup the file writer to be called on exit.
    atexit.register(save_tags_on_exit)

    # get TF from the service.
    tf_listener = tf.TransformListener()
    # set static transforms. TODO uncomment this when it's working.
    #T_CB = get_transform(TF_ROBOT_BASE, TF_CAMERA)

    # subscribe to apriltag detections.
    rospy.Subscriber("/tag_detections",AprilTagDetectionArray,get_tag_detection,queue_size=1)

    rospy.Timer(rospy.Duration(DT), timer_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
