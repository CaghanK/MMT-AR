#!/usr/bin/env python

import rospy
#import Classes.Mobot.calculate_ang_pos as calc_ang_pos
import numpy as np
import math
from Classes.Mobot import calculate_ang_pos
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Float64

rospy.init_node("calculate_ang_pos")

angPos_wheels = np.array([0, 0, 0], dtype=np.float)

r = rospy.Rate(100)


def callback(encoder_data):
    global angPos_wheels
    msg_out = Float64()

    encoderTicks = np.array([encoder_data.vector.x, encoder_data.vector.y, encoder_data.vector.z]) #TODO change to Vector3 from Vector3Stamped once time is no longer necessary (encoder_data.x, ...)
    angPos_wheels = encoderTicks * (2 * math.pi) / 6533 # rad per sec
    ori_mobot = calculate_ang_pos(angPos_wheels)

    msg_out.data = ori_mobot
    pub_vel.publish(msg_out)


rospy.Subscriber("encoders", Vector3Stamped, callback) # gets encoder counter data
pub_vel = rospy.Publisher("/mobot/ang_pos", Float64, queue_size = 1) # publish velocity data
    

while True:
    while not rospy.is_shutdown():
        r.sleep()

