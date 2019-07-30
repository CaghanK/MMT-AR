#!/usr/bin/env python

import rospy
from Classes.AugmentedRealityTest import MobotAR

if __name__ == "__main__": 
    AR = MobotAR()
    while not rospy.is_shutdown():
        AR.update()
        AR.r.sleep()