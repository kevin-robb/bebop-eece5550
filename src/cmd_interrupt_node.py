#!/usr/bin/env python3
import rospy
# --- Messages ---
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# --- Functions ---
import numpy as np
# --- Transforms ---
from scipy.spatial.transform import Rotation as R

############ GLOBAL VARIABLES ###################
# --- Publishers ---
cmd_pub = None
# --- Robot characteristics ---
r = 0.033 # wheel radius (m)
w = 0.16 # chassis width (m)
# --- Sensor data ---
scan = None


def get_scan_data(msg):
    """
    Get laserscan data from LiDAR.
     - 360-index list corresponding to degrees.
    """
    global scan
    scan = msg.ranges


def get_command(msg):
    """
    Receive command sent from motion planner.
    Either forward it along to robot, or replace with our own command.
    """
    if True:
        # forward command to robot.
        cmd_pub.publish(msg)


def main():
    global cmd_pub
    rospy.init_node('tag_tracking_node')

    # create publisher for cmd_vel.
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # subscribe to LiDAR detections.
    rospy.Subscriber('/kobuki/laser/scan', LaserScan, get_scan_data, queue_size=1)
    # subscribe to command sent by motion planner.
    rospy.Subscriber('/cmd_vel_intermediary', Twist, get_command, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
