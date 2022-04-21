#!/usr/bin/env python3
# ROS imports
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#from tf import transformations
#from datetime import datetime

# Util imports
import random
import math
import time

wall_dist = 0.5             # Distance desired from the wall
max_speed = 2.6             # Maximum speed of the robot on meters/seconds
wall_tracked = 0

def read_laser(self, lidar_data):
    """
    Read sensor messagens, and determine distance to each region. 
    Manipulates the values measured by the sensor.
    Callback function for the subscription to the published Laser Scan values.
    """
    
    # Determination of minimum distances in each region
    front = np.array(lidar_data.ranges[0:45] + lidar_data.ranges[314:359])
    fleft = np.array(lidar_data.ranges[46:91])
    left = np.array(lidar_data.ranges[92:137])
    bleft = np.array(lidar_data.ranges[138:183])
    bright = np.array(lidar_data.ranges[184:239])
    right = np.array(lidar_data.ranges[240:285])
    fright = np.array(lidar_data.ranges[286:313])
    regions_ = {
        'front':  min(front.min(), inf),
        'fleft': min(fleft.min(), inf),
        'left':  min(left.min(), inf),
        'bleft':  min(bleft.min(), inf),
        'bright':   min(bright.min(), inf),
        'right':   min(right.min(), inf),
        'fright':   min(fright.min(), inf),
    }

    dist_front = lidar_data.ranges[180]
    msg = Twist()
    if dist_front < wall_dist:
        msg.linear.x = 0
    elif dist_front < wall_dist*2:
        msg.linear.x = 0.5*max_speed
    else:
        msg.linear.x = max_speed
    #print 'Turn Left angular z, linear x %f - %f' % (msg.angular.z, msg.linear.x)
    return msg
