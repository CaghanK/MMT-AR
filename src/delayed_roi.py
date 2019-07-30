#!/usr/bin/env python

import rospy
from Classes.ROI import DelayedForceGenerator

if __name__ == "__main__": 
    delayed_roi_scanner = DelayedForceGenerator()
    while not rospy.is_shutdown():
        delayed_roi_scanner.r.sleep()