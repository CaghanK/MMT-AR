#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped

class PID:
    def __init__(self, kp, kd, ki, output_min=-255, output_max=255):
        rospy.init_node("motor_controller")
        self.sub_ref = rospy.Subscriber("/motor_ref", Vector3, self.callback_ref, queue_size=1)
        self.sub_vel = rospy.Subscriber("/motor_vel", Vector3, self.callback_vel, queue_size=1)
        self.pub_pwm = rospy.Publisher("/mobot/motor_vel", Vector3, queue_size=1)

        self.k_p = kp # proportional constant
        self.k_d = kd # derivative constant
        self.k_i = ki # integral constant
        self.output_min = output_min # output min limit -> 0-255 for pwm controller
        self.output_max = output_max # output max limit -> 0-255 for pwm controller
        self.error = np.array([0, 0, 0]) # reference value - feedback
        self.error_prev = np.array([0, 0, 0]) # error in previous step is stored to use in derivation
        self.time_prev = rospy.get_rostime().to_sec() # time in previous step is stored to use in derivation
        self.integralSum = 0
        self.dt = 0
        self.refValue = np.array([0, 0, 0]) # input of the controller

        
        
    def callback_ref(self, ref_in):
        setpoint = np.array([ref_in.x, ref_in.y, ref_in.z])
        self.setRefValue(setpoint)
    
    def callback_vel(self, vel_in):
        feedback = np.array([vel_in.x, vel_in.y, vel_in.z])
        pwm_out = self.update(feedback)
        msg_out = Vector3(-pwm_out[0], -pwm_out[1], -pwm_out[2])
        self.pub_pwm.publish(msg_out)

    def setRefValue(self, setpoint):
        self.refValue = setpoint

    def setTunings(self, kp, kd, ki):
        self.k_p = kp
        self.k_d = kd
        self.k_i = ki

    def setOutputLimits(self, output_min, output_max):
        self.output_min = output_min
        self.output_max = output_max

    def _saturate(self, value):
        output = np.clip(value, self.output_min, self.output_max)
        return output

    def _update_error(self, feedback):
        self.error = self.refValue - feedback

    def _update_time_and_dt(self):
        now = rospy.get_rostime().to_sec()
        self.dt = now - self.time_prev
        self.time_prev = now

    def proportional(self):
        return self.k_p * self.error

    def derivative(self):
        return self.k_d * ((self.error - self.error_prev) / self.dt)

    def integral(self):
        self.integralSum += (((self.error + self.error_prev) / 2) * self.dt)
        print(self.integralSum)
        return self.k_i * self.integralSum
        
    def update(self, feedback):
        self._update_error(feedback)
        self._update_time_and_dt()
        controlEffort = self.proportional() + self.derivative() + self.integral()
        self.prev_error = self.error
        return self._saturate(controlEffort)


    


