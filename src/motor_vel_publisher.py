#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
#from sensor_msgs.msg import Joy

rospy.init_node("vel_to_mobot")

def stop_motors():
    msg_zero = Vector3()
    pub_motorVel.publish(msg_zero)
    pub_motorVel.publish(msg_zero)
    pub_motorVel.publish(msg_zero)
rospy.on_shutdown(stop_motors)



r = rospy.Rate(100)

pub_motorVel = rospy.Publisher("/motor_ref", Vector3, queue_size = 1) # default: /mobot/motor_vel

while True:
    while not rospy.is_shutdown():
        msg_out = Vector3()
        msg_out.x = -6
        msg_out.y = -8
        msg_out.z = 3
        pub_motorVel.publish(msg_out)
        r.sleep()

