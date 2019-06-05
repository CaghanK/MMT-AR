#!/usr/bin/env python

import rospy
from Classes.ROI import ROI_scanner

if __name__ == "__main__": 
    roi_scanner = ROI_scanner()
    while not rospy.is_shutdown():
        roi_scanner.r.sleep()