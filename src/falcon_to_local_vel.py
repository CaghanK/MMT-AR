#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from rosfalcon.msg import falconPos

rospy.init_node("falcon_to_sim")

msg_vel = Vector3()

r = rospy.Rate(10)


def callback(falcon_data):
    MAX_LIN_VEL = 0.5
    MAX_ANG_VEL = 3.1416/2

    
    """ joy_x = falcon_data.X / 50
    joy_y = -falcon_data.Z / 50
    joy_z = 0 """
    #joy_z = joy_data.axes[3]
    
    msg_vel.x = falcon_data.X / 50
    msg_vel.y = -falcon_data.Z / 50
    msg_vel.z = 0
    
    """ msg_vel.x = map(joy_x, -50, 50, -MAX_LIN_VEL, MAX_LIN_VEL)
    msg_vel.y = map(joy_y, -50, 50, -MAX_LIN_VEL, MAX_LIN_VEL)"""
    #msg_vel.z = map(joy_z, -50, 50, -MAX_ANG_VEL, MAX_ANG_VEL)

    pub_simVel.publish(msg_vel)

def map(input, in_min, in_max, out_min, out_max):
    output = ((input - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min
    return output

rospy.Subscriber("/falconPos", falconPos, callback)
#rospy.Subscriber("/joy", Joy, callback)
pub_simVel = rospy.Publisher("/sim/mobot_vel", Vector3, queue_size = 1)


while True:
    while not rospy.is_shutdown():
        r.sleep()

