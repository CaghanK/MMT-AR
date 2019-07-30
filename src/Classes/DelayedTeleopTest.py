#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from transforms3d.euler import euler2mat
from objloader_simple import *
from geometry_msgs.msg import Pose2D
import Queue
#from sensor_msgs.msg import Image

# TODO get pose data from joystick/simulink

# Gets proxy pose from simulink
# Publishes proxy pose to gazebo
# Publishes delayed proxy pose as slave pose to gazebo
class DelayedVideo:

    def __init__(self, rate=30, video_queue_size=30):
        rospy.init_node("augmented_reality")
        self.r = rospy.Rate(rate)
        # Initialize camera
        self.cap = cv2.VideoCapture(1) # video device 0 for laptop cam, 1 for external cam
        self.delay_in_sec = 1
        self.video_queue = Queue.Queue(maxsize=self.delay_in_sec*video_queue_size)

    def update(self):
        # Read new image from camera
        ret, frame = self.cap.read()
        if not ret:
            print ("Unable to capture video")
            return 

        if self.video_queue.full():
            img = self.video_queue.get()
            cv2.imshow('frame', img)
            cv2.waitKey(1)

        self.video_queue.put(frame)