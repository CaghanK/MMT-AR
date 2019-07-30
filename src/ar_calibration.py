#!/usr/bin/env python

import rospy
from Classes.AR_Calibration import ARCalibration

if __name__ == "__main__": 
    calib = ARCalibration()
    while not rospy.is_shutdown():
        calib.update()
        calib.r.sleep()