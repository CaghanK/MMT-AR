#!/usr/bin/env python

import rospy
from Classes.PoseController import PoseController

if __name__ == "__main__": 
    pose_controller = PoseController()
    while not rospy.is_shutdown():
        pose_controller.r.sleep()
