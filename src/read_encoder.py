#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist

rospy.init_node("encoder_to_turtle")

msg_out = Twist()

r = rospy.Rate(100)

def callback(encoder_data):
    msg_out.linear.x = encoder_data.r
    msg_out.linear.y = encoder_data.g
    msg_out.angular.z = encoder_data.b
    pub.publish(msg_out)

rospy.Subscriber("encoders", ColorRGBA, callback)
pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 1)

while not rospy.is_shutdown():

    r.sleep()