#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from Queue import Queue
from Classes.Simulation import *

if __name__ == "__main__": 
    sim = Simulation()
    sim.r = rospy.Rate(100)
    while not rospy.is_shutdown():
        sim.r.sleep()