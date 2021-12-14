#!/usr/bin/env python3
import rospy
# --- Messages ---
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

############ GLOBAL VARIABLES ###################
# --- Publishers ---
cmd_pub = None
# --- Robot characteristics ---
r = 0.033  # wheel radius (m)
w = 0.16  # chassis width (m)
# --- Time parameters ---
start_time = None
end_time   = None
PirouetteDuration = None  # (Rospy.Duration)
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
    global start_time, end_time, PirouetteDuration
    """
    Receive command sent from motion planner.
    Either forward it along to robot, or replace with our own command.
    """
    if rospy.Time.now() >= end_time:
        # replace motion command with a spin.
        msg_new  = Twist()
        msg_new.linear.x = 0
        msg_new.angular.z = 1.25  # [radians] Rotates in place for PirouetteDuration -> 360 degree rotation (seconds)
        cmd_pub.publish(msg_new)
        rospy.sleep(PirouetteDuration)
        start_time = rospy.Time.now()
        end_time = start_time + PirouetteDuration
    else:
        # forward along command from motion planner.
        cmd_pub.publish(msg)


def main():

    global cmd_pub, start_time, end_time, PirouetteDuration

    rospy.init_node('cmd_interrupt_node')

    PirouetteDuration = rospy.Duration(5.0)
    start_time = rospy.Time.now()
    end_time = start_time + PirouetteDuration

    # create publisher for cmd_vel.
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # subscribe to LiDAR detections.
    rospy.Subscriber('/scan', LaserScan, get_scan_data, queue_size=1)
    # subscribe to command sent by motion planner.
    rospy.Subscriber('/cmd_vel_intermediary', Twist, get_command, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
