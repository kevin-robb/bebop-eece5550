#!/usr/bin/env python
import rospy
# --- Messages ---
from apriltag_ros.msg import AprilTagDetectionArray
# --- Functions ---
import numpy as np
from scipy.linalg import inv
from time import sleep
# --- Transforms ---
import tf
from scipy.spatial.transform import Rotation as R

############ GLOBAL VARIABLES ###################
tags = {} # dictionary of tag poses, keyed by ID.
# --- Robot characteristics ---
r = 0.033 # wheel radius (m)
w = 0.16 # chassis width (m)
X0 = np.array([[1,0,0],[0,1,0],[0,0,1]]) # initial pose
# --- Transforms (homogenous matrices) ---
tf_listener = None # for getting T_CB
T_BO = None # origin->base
T_CB = None # base->cam
T_AC = None # cam->tag
T_AO = None # origin->tag
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
    
    print("tag detected at T_AC = ", T_AC)
    # calculate global pose of the tag.
    T_AO = T_AC * T_CB * T_BO

    # update the dictionary with this tag.
    if tag_id in tags.keys():
        # update using learning rate.
        L = 0.9
        tags[tag_id] = np.add(L * tags[tag_id], (1-L) * T_AO)
    else: 
        # this is a new tag.
        tags[tag_id] = T_AO


def get_T_CB():
    """
    Get the transform between the camera and base robot frame, 
    and store globally for later use.
    """
    global T_CB
    # make sure the listener has time to initialize
    sleep(2.5)
    # get cam relative to base from the service
    (t,q) = tf_listener.lookupTransform('/base_footprint', '/camera_rgb_optical_frame', rospy.Time(0))
    # get equiv rotation matrix from quaternion
    r = R.from_quat(q).as_matrix()
    # make affine matrix for transformation base->cam
    T_CB = np.array([[r[0][0],r[0][1],r[0][2],t[0]],
                    [r[1][0],r[1][1],r[1][2],t[1]],
                    [r[2][0],r[2][1],r[2][2],t[2]],
                    [0,0,0,1]])


def main():
    global tf_listener
    rospy.init_node('tag_tracking_node')

    # get TF from the service.
    tf_listener = tf.TransformListener()
    # set static transforms.
    get_T_CB()

    # subscribe to apriltag detections.
    rospy.Subscriber("/tag_detections",AprilTagDetectionArray,get_tag_detection,queue_size=1)
    # TODO subscribe to robot's current position from Cartographer. set to T_BO.

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
