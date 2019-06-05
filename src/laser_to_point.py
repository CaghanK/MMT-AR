#!/usr/bin/env python

import rospy
from Classes.AugmentedLaser import AugmentedLaser

if __name__ == "__main__": 
    laser_visualizer = AugmentedLaser()
    while not rospy.is_shutdown():
        laser_visualizer.r.sleep()