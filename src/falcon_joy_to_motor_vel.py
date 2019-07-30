#!/usr/bin/env python

import rospy
from Classes.FalconVelocityGenerator import FalconVelocityPublisher

if __name__ == "__main__": 
    vel_generator = FalconVelocityPublisher()
    while not rospy.is_shutdown():
        vel_generator.update()
        vel_generator.r.sleep()

