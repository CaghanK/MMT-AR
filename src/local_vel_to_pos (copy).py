#!/usr/bin/env python

import rospy
from Classes.MobotPose import MobotPose

if __name__ == "__main__": 
    mobot_pose = MobotPose()
    while not rospy.is_shutdown():
        mobot_pose.update()
        mobot_pose.r.sleep()