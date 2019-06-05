#!/usr/bin/env python

import rospy
from Classes.ForceGenerator import ForceGenerator

if __name__ == "__main__": 
    force_generator = ForceGenerator()
    while not rospy.is_shutdown():
        force_generator.generate_force()
        force_generator.r.sleep()