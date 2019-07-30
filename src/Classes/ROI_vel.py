#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from Classes.Mobot import inverse_kinematics
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
        self.pub = rospy.Publisher("/mobot/motor_vel", Vector3, queue_size=1)


    # Subscriber callback
    def callback(self, msg_in):
        # Localize the incoming message
        angle_min = msg_in.angle_min
        angle_increment = msg_in.angle_increment

        # Read all distance measurements
        whole_array = np.array(msg_in.ranges)
        # Record the index of values within the specified range
        roi_data_index = np.where(whole_array < 0.8)[0]
        # Find the number of data that satisfies the condition < 0.6m
        roi_data_no = np.size(roi_data_index)

        # Create 2D array to hold range and angle values
        scan_data_with_angle = np.empty([2, roi_data_no])

        # Fill the first row with corresponding angles
        #scan_data_with_angle[0,:] = np.rad2deg(roi_data_index * angle_increment + angle_min) #- np.rad2deg(angle_min)
        scan_data_with_angle[0,:] = roi_data_index * angle_increment + angle_min
        # Fill the second row with distance values
        scan_data_with_angle[1,:] = whole_array[roi_data_index]

        """
        for data_index in range(0, roi_data_no):
            data_pair = np.array([scan_data_with_angle[0,data_index], scan_data_with_angle[1,data_index]])
            print(data_pair)

        print("-------------------")


        for data_index in range(0, roi_data_no):
            
            print(data_pair)

        """

        # ---------- Force Generation ----------

        # Find average coordinate of all points within ROI
        scan_data_xy = np.empty([2, roi_data_no])
        scan_data_xy[0,:] = np.multiply(scan_data_with_angle[1, :], np.cos(scan_data_with_angle[0, :] + np.pi/2))
        scan_data_xy[1,:] = np.multiply(scan_data_with_angle[1, :], np.sin(scan_data_with_angle[0, :] + np.pi/2))

        scan_data_xy_avg = np.array([np.average(scan_data_xy[0, :]), np.average(scan_data_xy[1, :])])

        # Normalize the normal vector
        vec_length = np.sqrt(np.square(scan_data_xy_avg[0]) + np.square(scan_data_xy_avg[1]))
        
        #print(np.sqrt(np.square(scan_data_xy_avg[0]) + np.square(scan_data_xy_avg[0])))
        vec_normalized = np.divide(scan_data_xy_avg, vec_length)
        k = 1
        generated_force = -vec_normalized * (0.6 - vec_length) * k
        generated_force *= 3
        #print(generated_force)

        pwm_demand = inverse_kinematics(generated_force[0], generated_force[1], 0)

        #print(inverse_kinematics(generated_force[0], generated_force[1], 0))

        self.pub.publish(Vector3(pwm_demand[0], pwm_demand[1], pwm_demand[2]))
        

        """
        size = np.size(points[:,0])

        x_total = 0
        y_total = 0
        for index in range(1, size):
            r = sqrt(points[index, 0]**2 + points[index, 1]**2)
            x_total += ((points[index, 0] / r) * self.weighting_func(r))
            y_total += ((points[index, 1] / r) * self.weighting_func(r))
            
        print(x_total, y_total)

        vec_len = sqrt(x_total**2 + y_total**2)
        x_total /= vec_len
        y_total /= vec_len
        print(x_total, y_total)

    def weighting_func(self, value):
        if value < 15:
            weight = 1
        elif value > 60:
            weight= 0
        else:
            weight = ((1 - ((value - 15) / 60))**4)*((4*(value - 15)/(60))+1)
                
        return weight
        """

        #print(scan_data_with_angle)
        #print(scan_data_with_angle)

    ''''def scan_roi(self):
        for each range in self.scan_data:
    '''


        

    
