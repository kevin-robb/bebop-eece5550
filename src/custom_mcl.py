#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np
from scipy.linalg import logm, inv, expm
from time import sleep
import tf
from scipy.spatial.transform import Rotation as R

control_pub = None
tf_listener = None
# Constants:
r = 0.033 # wheel radius (m)
w = 0.16 # chassis width (m)
dt = 0.1 # period for timer_callback
# Particle Filter parameters
set_size = 100
particle_set = [0 for _ in range(set_size)]
X = np.array([1,0,0],[0,1,0],[0,0,1])
U = np.array([0],[0])

def timer_callback(event):
    pass

def mcl(U, Z, M):
    """
    Given the prior particle set, last motion command, current sensor scan, and map m.
    Output the posterior particle set.
    """
    global particle_set
    for k in range(particle_set):
        # prediction step
        particle_set[k] = motion_model(particle_set[k])
        # compute particle weights
        pass
    
    for _ in range(len(particle_set)):
        # resampling step
        pass

def motion_model(X,U):
    """
    Use current state X and motor commands U to perform forward kinematics for one timestep.
    X in SE(2)
    U = (dx, dtheta)^T
    """
    dx, dtheta = U[0][0], U[1][0]
    # form omega
    omega = np.array([0,-dtheta,dx],[dtheta,0,0],[0,0,0])
    # forward kinematics
    X_next = np.matmul(X, expm(dt * omega))
    return X_next

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
