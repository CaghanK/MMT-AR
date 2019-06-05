#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
#from sensor_msgs.msg import Image

# TODO get pose data from joystick/simulink

# Gets proxy pose from simulink
# Publishes proxy pose to gazebo
# Publishes delayed proxy pose as slave pose to gazebo
class ROI_scanner:

    def __init__(self, rate=10):
        rospy.init_node("roi_scanner")
        self.r = rospy.Rate(rate)
        # Proxy pose holder
        self.scan_data = []
        
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback, queue_size=1)
        #self.pub = rospy.Publisher("/falconForce", falconForces, queue_size=1)


    # Subscriber callback
    def callback(self, msg_in):
        angle_min = msg_in.angle_min
        angle_increment = msg_in.angle_increment

        whole_array = np.array(msg_in.ranges)
        print(np.size(whole_array))
        roi_data_index = np.where(whole_array < 0.6)[0]
        
        roi_data_no = np.size(roi_data_index)

        scan_data_with_angle = np.empty([2, roi_data_no])

        scan_data_with_angle[0,:] = np.rad2deg(roi_data_index * angle_increment + angle_min) - np.rad2deg(angle_min)

        scan_data_with_angle[1,:] = whole_array[roi_data_index]
        
        #print(scan_data_with_angle)
        #print(roi_data_no)

    ''''def scan_roi(self):
        for each range in self.scan_data:
            '''


        

    
