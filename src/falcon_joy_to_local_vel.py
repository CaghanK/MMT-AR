#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from rosfalcon.msg import falconPos

rospy.init_node("falcon_to_sim")

msg_vel = Vector3()

r = rospy.Rate(10)


def callback(joy_data):
    MAX_LIN_VEL = 0.1 # default:0.5
    MAX_ANG_VEL = 3.1416/4 # default denominator:2

    x_axis = joy_data.axes[0]
    y_axis = -joy_data.axes[2]
    button_left = joy_data.buttons[0]
    button_mid = joy_data.buttons[1]
    button_right = joy_data.buttons[3]
    
    if abs(x_axis) < 5.0:
        x_axis = 0.0

    if abs(y_axis) < 5.0:
        y_axis = 0.0

    ang_vel = 0
    if button_left == 1 and button_right == 0: # Turn right
        ang_vel = MAX_ANG_VEL
    if button_right == 1 and button_left == 0: # Turn left
        ang_vel = -MAX_ANG_VEL

    msg_vel.x = (x_axis / 50) * MAX_LIN_VEL
    msg_vel.y = (y_axis / 50) * MAX_LIN_VEL
    msg_vel.z = ang_vel

    if button_mid == 1:
            msg_vel.x = 0
            msg_vel.y = 0
            msg_vel.z = 0

    pub_simVel.publish(msg_vel)

def map(input, in_min, in_max, out_min, out_max):
    output = ((input - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min
    return output

rospy.Subscriber("/falcon/joystick", Joy, callback)
#rospy.Subscriber("/joy", Joy, callback)
pub_simVel = rospy.Publisher("/proxy/vel", Vector3, queue_size = 1) # default topic: /sim/mobot_vel, /mobot/robot_vel_desired
#pub_simVel = rospy.Publisher("/mobot/robot_vel", Vector3, queue_size = 1)


while True:
    while not rospy.is_shutdown():
        r.sleep()

