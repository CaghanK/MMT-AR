#!/usr/bin/env python
import rospy
import numpy as np
import Queue
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3
from Classes.Mobot import inverse_kinematics
from rosfalcon.msg import falconForces
#from sensor_msgs.msg import Image

# TODO get pose data from joystick/simulink

# Gets proxy pose from simulink
# Publishes proxy pose to gazebo
# Publishes delayed proxy pose as slave pose to gazebo
class CommunicationDelay:

    def __init__(self, scan_queue_size=7, pose_queue_size=7, proxy_queue_size=30):
        rospy.init_node("comm_delay")
        self.r = rospy.Rate(10)
        self.delay_in_sec = 1

        self.scan_queue = Queue.Queue(maxsize=self.delay_in_sec*scan_queue_size)
        self.pose_queue = Queue.Queue(maxsize=self.delay_in_sec*pose_queue_size)
        self.proxy_queue = Queue.Queue(maxsize=self.delay_in_sec*proxy_queue_size)
        
        # Gets current laser scan data
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.callback_scan, queue_size=1)  
        # Publishes delayed laser scan data
        self.pub_scan = rospy.Publisher("/scan_delayed", LaserScan, queue_size=1)
        """----------"""
        # Gets current robot pose
        self.sub_pose = rospy.Subscriber("/pose2D", Pose2D, self.callback_pose, queue_size=1)
        # Publishes delayed pose data
        self.pub_pose = rospy.Publisher("/mobot/pose_delayed", Pose2D, queue_size=1)
        """----------"""
        # Gets current proxy pose
        self.sub_proxy = rospy.Subscriber("/proxy/pose_nondelayed", Pose2D, self.callback_proxy, queue_size=1)
        # Publishes delayed proxy pose
        self.pub_proxy = rospy.Publisher("/proxy/pose", Pose2D, queue_size=1) # default: _delayed

    # Gets latest scan data, publishes the oldest scan data, saves the latest scan data to the queue
    def callback_scan(self, scan_in):
        if self.scan_queue.full():
            self.pub_scan.publish(self.scan_queue.get())

        self.scan_queue.put(scan_in)

    # Gets latest robot pose, publishes the oldest pose, saves the latest pose to the queue
    def callback_pose(self, pose_in):
        if self.pose_queue.full():
            self.pub_pose.publish(self.pose_queue.get())

        self.pose_queue.put(pose_in)

    # Gets latest proxy pose, publishes the oldest proxy pose, saves the latest proxy pose to the queue
    def callback_proxy(self, pose_in):
        if self.proxy_queue.full():
            self.pub_proxy.publish(self.proxy_queue.get())

        self.proxy_queue.put(pose_in)



        

    
