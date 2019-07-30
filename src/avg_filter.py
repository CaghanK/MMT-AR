#!/usr/bin/env python

import rospy
from Classes.Filters import AverageFilter
from geometry_msgs.msg import Vector3

if __name__ == "__main__": 
    avg_filter = AverageFilter(3, 10)
    while not rospy.is_shutdown():
        avg_filter.r.sleep()