#!/usr/bin/env python

import rospy
import math

class PID:
    def __init__(self, kp, kd, ki, output_min=-255, output_max=255):
        self.k_p = kp # proportional constant
        self.k_d = kd # derivative constant
        self.k_i = ki # integral constant
        self.output_min = output_min # output min limit -> 0-255 for pwm controller
        self.output_max = output_max # output max limit -> 0-255 for pwm controller
        self.error = 0 # reference value - feedback
        self.error_prev = 0 # error in previous step is stored to use in derivation
        self.time_prev = rospy.get_rostime().to_sec() # time in previous step is stored to use in derivation
        self.integralSum = 0
        self.dt = 0
        self.refValue = 0 # input of the controller

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
        value = max(self.output_min, value)
        value = min(value, self.output_max)
        return value

    def _update_error(self, feedback):
        self.prev_error = self.error
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
        return self.k_i * self.integralSum
        
    def update(self, feedback):
        self._update_error(feedback)
        self._update_time_and_dt()
        controlValue = self.proportional() + self.derivative() + self.integral()
        return self._saturate(controlValue)


    


