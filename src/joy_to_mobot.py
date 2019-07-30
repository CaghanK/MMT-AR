#!/usr/bin/env python

import rospy
from Classes.Mobot import inverse_kinematics
from Classes.Mobot import forward_kinematics
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy

rospy.init_node("joy_to_mobot")

msg_inv = Vector3()
msg_fwd = Vector3()
r = rospy.Rate(100)

def stop_motors():
    msg_zero = Vector3()
    pub_motorVel.publish(msg_zero)
    pub_motorVel.publish(msg_zero)
    pub_motorVel.publish(msg_zero)
rospy.on_shutdown(stop_motors)


def callback(joy_data):

    joy_x = -joy_data.axes[0] # minus to match the coordinate system
    joy_y = joy_data.axes[1]
    joy_z = -joy_data.axes[3]
    
    vel_array = inverse_kinematics(joy_x, joy_y, joy_z)
    msg_inv.x = 1.5 * vel_array[0]
    msg_inv.y = 1.5 * vel_array[1]
    msg_inv.z = 1.5 * vel_array[2]
    pub_motorVel.publish(msg_inv)


rospy.Subscriber("/joy", Joy, callback)
pub_motorVel = rospy.Publisher("/mobot/motor_vel", Vector3, queue_size = 1) 
pub_mobotVel = rospy.Publisher("/mobot/robot_vel", Vector3, queue_size = 1)

while True:
    while not rospy.is_shutdown():
        """ robot_vel_array = forward_kinematics(msg_inv.x, msg_inv.y, msg_inv.z)
        msg_fwd.x = robot_vel_array[0]
        msg_fwd.y = robot_vel_array[1]
        msg_fwd.z = robot_vel_array[2]
        pub_mobotVel.publish(msg_fwd) """
        r.sleep()


