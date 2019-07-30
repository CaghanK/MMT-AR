#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from Classes.Mobot import inverse_kinematics
from rosfalcon.msg import falconForces
#from sensor_msgs.msg import Image

# TODO get pose data from joystick/simulink

# Gets proxy pose from simulink
# Publishes proxy pose to gazebo
# Publishes delayed proxy pose as slave pose to gazebo
class ROI_scanner:

    def __init__(self, rate=10):
        rospy.init_node("roi_scanner")
        self.r = rospy.Rate(rate)
        self.ROI_radius = 0.6 # radius of force generation in meters
        # Proxy pose holder
        self.scan_data = []
        
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/falconForce", falconForces, queue_size=1)


    # Subscriber callback
    def callback(self, msg_in):
        # Localize the incoming message
        angle_min = msg_in.angle_min
        angle_increment = msg_in.angle_increment

        # Read all distance measurements
        whole_array = np.array(msg_in.ranges)
        # Record the index of values within the specified range
        roi_data_index = np.where(whole_array < self.ROI_radius)[0]
        # Find the number of data that satisfies the condition < 0.6m
        roi_data_no = np.size(roi_data_index)
        #print(roi_data_no)

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
        vec_normalized = np.divide(scan_data_xy_avg, vec_length)

        # Generate force, F = -k*x where x is inversely proportional to the distance between the robot origin and the average of all the points inside ROI
        k = 1
        generated_force = -vec_normalized * (self.ROI_radius - vec_length) * k
        #################################################print(generated_force)
        #print(generated_force)
        k_f = 10

        # Change data type to falconForces to be published
        #force_demand = falconForces(k_f*-generated_force[0], k_f*generated_force[1], 0) # minus x to match direction
        force_demand = falconForces(k_f*generated_force[0], 0.0, -k_f*generated_force[1])
        if roi_data_no == 0:
            force_demand = falconForces(0.0, 0.0, 0.0)

        print(force_demand)
        self.pub.publish(force_demand)

        #print(inverse_kinematics(generated_force[0], generated_force[1], 0))

        #print(scan_data_with_angle)
        #print(scan_data_with_angle)

    ''''def scan_roi(self):
        for each range in self.scan_data:
    '''


        

    
