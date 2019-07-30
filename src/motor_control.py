#!/usr/bin/env python

import rospy
from Classes.MotorController import MotorController
from geometry_msgs.msg import Vector3

if __name__ == "__main__": 
    motor_controller = MotorController()
    while not rospy.is_shutdown():
        motor_controller.update()
        motor_controller.r.sleep()
    motor_controller.pub_pwm.publish(Vector3(0, 0, 0))