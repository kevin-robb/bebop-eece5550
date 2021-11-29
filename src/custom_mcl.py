#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np
from scipy.linalg import logm, inv
from time import sleep
import tf
from scipy.spatial.transform import Rotation as R

control_pub = None
tf_listener = None
# Constants:
r = 0.033 # wheel radius (m)
w = 0.16 # chassis width (m)
dt = 0.1 # period for timer_callback

def timer_callback(event):
    pass

def motion_model(X,U):
    """
    Use current state X and motor commands U to perform forward kinematics for one timestep.
    X = (x, y, yaw)
    U = (dphi_l, d_phi_r)
    """

def form_omega(dx, dt):
    #omega = np.array
    #TODO

def main():
    global control_pub, tf_listener
    rospy.init_node('control_node')
    # get the TF from the service
    tf_listener = tf.TransformListener()

    # publish the command messages
    control_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # subscribers
    rospy.Subscriber("/tag_detections",AprilTagDetectionArray,tag_detect,queue_size=1)
    # make a 10Hz timer
    rospy.Timer(rospy.Duration(dt), timer_callback)
    # pump callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
