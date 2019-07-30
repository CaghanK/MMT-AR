#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Vector3

class AverageFilter:
    def __init__(self, channel_no=3, size=10):
        rospy.init_node("average_filter")
        self.r = rospy.Rate(100)
        self.size = size
        self.channel_no = channel_no
        # FIFO queue to hold the data
        self.queue = np.zeros((channel_no, size))
        # start intex of the queue, end_index = start_index - 1
        self.head_index = 0
        self.average = np.zeros(channel_no)
        self.sub = rospy.Subscriber("/mobot/motor_vel_measured", Vector3, self.callback_vel, queue_size=1)
        self.pub = rospy.Publisher("/mobot/motor_vel_filtered", Vector3, queue_size=1)
    def callback_vel(self, motor_vel):

        filtered = self.update([motor_vel.x, motor_vel.y, motor_vel.z])
        filtered_V3 = Vector3(filtered[0], filtered[1], filtered[2])
        self.pub.publish(filtered_V3)

    def update(self, new_data):
        for channel in range(self.channel_no):
            # calculate the new average after substracting the oldest and adding the newest element
            self.average[channel] += (new_data[channel] - self.queue[channel, self.head_index]) / self.size
            # replace the oldest element with the newest
            self.queue[channel, self.head_index] = new_data[channel]

        # update the starting index of the dynamic queue
        self.head_index= (self.head_index + 1) % self.size

        return self.average