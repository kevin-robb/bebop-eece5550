#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid

# ***************
#   OBSTACLE
M = 75
#   unknown
N = 50
#   free
# ----0-----
#   unknown
# ***************
def callback(cmap: OccupancyGrid):
    data = list(cmap.data)
    for y in range(cmap.info.height):
        for x in range(cmap.info.width):
            i = x + (cmap.info.height - 1 - y) * cmap.info.width
            if data[i] >= M:  
                data[i] = 100
            elif (data[i] >= 0) and (data[i] < N):  # free
                data[i] = 0
            else:  # unknown
                data[i] = -1
    cmap.data = tuple(data)
    pub.publish(cmap)


rospy.init_node('mapc_node', anonymous=True)
sub = rospy.Subscriber('/cmap', OccupancyGrid, callback)
pub = rospy.Publisher('/map', OccupancyGrid, queue_size=20)

rospy.spin()