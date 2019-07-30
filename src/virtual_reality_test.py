#!/usr/bin/env python

import rospy
from Classes.VirtualRealityTest import MobotVR

if __name__ == "__main__": 
    VR = MobotVR()
    while not rospy.is_shutdown():
        VR.update()
        VR.r.sleep()