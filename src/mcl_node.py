#!/usr/bin/env python
import rospy
from mcl_filter import MCL_Filter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
import numpy as np

############ Constants ###################
# --- map file ---
MAP_FILEPATH = './RecordedFiles/Lab3Q1.bag_map.pgm'
# ----- Clock parameters -------
prev_time = None
dt = 0
# ----- Particle Filter parameters -------
filter = None
# ----- Data from subscribers --------------
U = np.array([0], [0])
Z = []
##########################################


def timer_callback(event):
    """
    Subscribe to the '/clock' topic.
    This should keep the particle filter synched with a bag file replay.
    Use the previous time to set dt.
    """
    # handle tracking of dt between timesteps.
    global prev_time, dt
    if prev_time is None:
        prev_time = event.clock
    else:
        dt = event.clock - prev_time
        prev_time = event.clock
    # run the particle filter at this timestep.
    filter.main_mcl_loop(U, Z, dt)

def get_controls(msg):
    """
    Grab the control velocities being sent to the robot from some other node (i.e., teleop).
    Input: msg->twist message
    Return: List containing commanded Linear and Angular velocities
    """
    global U
    U = [msg.linear.x, msg.angular.z]

def get_measurements(msg):
    """
    Get LIDAR Range measurements
    """
    global Z
    Z = msg.ranges

def main():
    global filter
    rospy.init_node('custom_mcl')
    filter = MCL_Filter(map_filepath=MAP_FILEPATH)

    # subscribe to published commands.
    rospy.Subscriber('/cmd_vel', Twist, get_controls, queue_size=1)

    # subscribe to lidar measurements.
    rospy.Subscriber('/scan', LaserScan, get_measurements, queue_size=1)

    # subscribe to the '/clock' topic since it is being used already to replay the bag file.
    rospy.Subscriber('/clock', Clock, timer_callback)
    # rospy.Timer(rospy.Duration(dt), timer_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
