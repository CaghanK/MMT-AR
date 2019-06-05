#!/usr/bin/env python

import rospy
from Classes.AugmentedLaser import AugmentedLaser

if __name__ == "__main__": 
    AL = AugmentedLaser()
    while not rospy.is_shutdown():
        AL.update()
        AL.r.sleep()