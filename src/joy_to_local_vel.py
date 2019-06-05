#!/usr/bin/env python

import rospy
from Classes.Mobot import inverse_kinematics
from Classes.Mobot import forward_kinematics
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy

rospy.init_node("joy_to_sim")

msg_vel = Vector3()

r = rospy.Rate(10)


def callback(joy_data):
    MAX_LIN_VEL = 0.5
    MAX_ANG_VEL = 3.1416/2

    joy_x = -joy_data.axes[0] # minus to match the coordinate system
    joy_y = joy_data.axes[1]
    joy_z = joy_data.axes[3]
    
    
    msg_vel.x = map(joy_x, -1.0, 1.0, -MAX_LIN_VEL, MAX_LIN_VEL)
    msg_vel.y = map(joy_y, -1.0, 1.0, -MAX_LIN_VEL, MAX_LIN_VEL)
    msg_vel.z = map(joy_z, -1.0, 1.0, -MAX_ANG_VEL, MAX_ANG_VEL)

    pub_simVel.publish(msg_vel)

def map(input, in_min, in_max, out_min, out_max):
    output = ((input - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min
    return output

rospy.Subscriber("/joy", Joy, callback)
pub_simVel = rospy.Publisher("/sim/mobot_vel", Vector3, queue_size = 1)


while True:
    while not rospy.is_shutdown():
        r.sleep()

