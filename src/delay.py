#!/usr/bin/env python

import rospy
from Classes.ROI_delayed import CommunicationDelay

if __name__ == "__main__": 
    delayer = CommunicationDelay()
    while not rospy.is_shutdown():
        delayer.r.sleep()