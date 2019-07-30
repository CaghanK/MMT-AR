#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose2D
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


    # Generates force making use of the delayed scan & pose data and non-delayed proxy pose

class DelayedForceGenerator:
    def __init__(self, rate=7):
        rospy.init_node("delayed_force_generator")
        self.r = rospy.Rate(rate)
        self.ROI_radius = 0.6 # radius of force generation in meters
        # Proxy pose holder
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.scan_data = None
        self.robot_pose = Pose2D()
        self.proxy_pose = Pose2D()
        
        # Gets current proxy pose
        self.sub_proxy = rospy.Subscriber("/proxy/pose_nondelayed", Pose2D, self.callback_proxy, queue_size=1)
        # Gets delayed pose of the robot
        self.sub_robot = rospy.Subscriber("/mobot/pose_delayed", Pose2D, self.callback_robot, queue_size=1)
        # Gets delayed scan data from the robot
        self.sub_scan = rospy.Subscriber("/scan_delayed", LaserScan, self.callback_scan, queue_size=1)
        # Publishes force to falcon
        self.pub_force = rospy.Publisher("/falconForce", falconForces, queue_size=1)

    # Calculates scan points in proxy frame from the delayed scan data and robot pose
    # Convert from polar to cartesian coordinates
    # Move points by (proxy_pose - robot_pose)
    def callback_proxy(self, proxy_in):
        self.proxy_pose = proxy_in           

    def callback_robot(self, robot_in):
        self.robot_pose = robot_in

    def callback_scan(self, scan_in):
        self.scan_data = scan_in.ranges
        self.angle_min = scan_in.angle_min
        self.angle_increment = scan_in.angle_increment
        self.update_force()

    def update_force(self):
        x_diff = -self.robot_pose.y - self.proxy_pose.x
        y_diff = self.robot_pose.x - self.proxy_pose.y
        theta_diff = self.robot_pose.theta - self.proxy_pose.theta

        #print(x_diff, y_diff)

        # Copy distance data to numpy array
        whole_data = np.array(self.scan_data)
        # Get the number of points
        whole_data_size = np.size(whole_data)
        # Create an index array to calculate the angles
        index_array = np.arange(whole_data_size)


        angles = index_array * self.angle_increment + self.angle_min

        whole_data_xy = np.empty([2, whole_data_size])
        whole_data_xy[0,:] = np.multiply(whole_data, np.cos(angles + np.pi/2))
        whole_data_xy[1,:] = np.multiply(whole_data, np.sin(angles + np.pi/2))

        transformed_points = np.empty([2, whole_data_size])
        transformed_points[0,:] = whole_data_xy[0,:] + x_diff
        transformed_points[1,:] = whole_data_xy[1,:] + y_diff
        #transform_vector = np.array([[x_diff], [y_diff]])
        #transformed_points = whole_data_xy + transform_vector
        #print(transform_vector)

        #print(whole_data_size)
        #print(transformed_points_avg)

        points_in_ROI = self.find_points_in_ROI(transformed_points)
        points_in_ROI_avg = np.array([np.average(points_in_ROI[0, :]), np.average(points_in_ROI[1, :])])

        
        # Normalize the normal vector
        vec_length = np.sqrt(np.square(points_in_ROI_avg[0]) + np.square(points_in_ROI_avg[1]))
        vec_normalized = np.divide(points_in_ROI_avg, vec_length)
        #print(vec_normalized)
        
        # Generate force, F = -k*x where x is inversely proportional to the distance between the robot origin and the average of all the points inside ROI
        k = 1
        generated_force = -vec_normalized * (self.ROI_radius - vec_length) * k
        #################################################print(generated_force)
        #print(generated_force)
        k_f = 10
        if np.isnan(generated_force[0]):
            generated_force[0] = 0.0

        if np.isnan(generated_force[1]):
            generated_force[1] = 0.0
        # Change data type to falconForces to be published
        #force_demand = falconForces(k_f*-generated_force[0], k_f*generated_force[1], 0) # minus x to match direction
        force_demand = falconForces(k_f*generated_force[0], 0.0, -k_f*generated_force[1])
        if whole_data_size == 0:
            force_demand = falconForces(0.0, 0.0, 0.0)

        print(force_demand)
        self.pub_force.publish(force_demand)

        # point_now = point_prev + (robot_pose - proxy_pose)
        
    def find_points_in_ROI(self, points):
        square_of_points = points**2
        total_of_squares = square_of_points[0,:] + square_of_points[1,:]

        limit = self.ROI_radius**2
        roi_data_index = np.where(total_of_squares < limit)[0]
        #print(np.size(roi_data_index))
        points_in_ROI = np.empty([2, np.size(roi_data_index)])

        points_in_ROI[0,:] = points[0, roi_data_index]
        points_in_ROI[1,:] = points[1, roi_data_index]

        return points_in_ROI




        

    
