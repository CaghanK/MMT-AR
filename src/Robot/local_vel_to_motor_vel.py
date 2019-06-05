#!/usr/bin/env python

import rospy
from Classes.Mobot import inverse_kinematics
from geometry_msgs.msg import Vector3

rospy.init_node("joy_to_mobot")

msg_inv = Vector3()
msg_fwd = Vector3()
r = rospy.Rate(100)


def callback(joy_data):

    x = joy_data.x
    y = joy_data.y
    theta = joy_data.z
    vel_array = inverse_kinematics(x, y, theta)
    msg_out = Vector3(vel_array[0], vel_array[1], vel_array[2])
    pub_motorVel.publish(msg_out)


rospy.Subscriber("/mobot/robot_vel", Vector3, callback)
pub_motorVel = rospy.Publisher("/mobot/motor_vel", Vector3, queue_size = 1)

while True:
    while not rospy.is_shutdown():
        r.sleep()

