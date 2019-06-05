#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped

# Gets encoder data from arduino
# Applies modified PD-feedforward control
# Publishes control output as pwm value to arduino
class MotorController:

    def __init__(self, rate=100):
        rospy.init_node("motor_speed_control")
        self.r = rospy.Rate(rate)
        # Mobot pose init (0)
        self.time_prev = rospy.Time(0)
        self.time_now = rospy.Time(0)
        self.encoders_prev = np.array([0, 0, 0])
        self.encoders_now = np.array([0, 0, 0])
        self.error_prev = np.array([0, 0, 0])
        self.ang_vel = np.array([0, 0, 0])
        self.ref_input = Vector3()
        self.dt_secs = 0

        # Control Parameters
        self.k_p = 1
        
        self.sub_ref = rospy.Subscriber("/motor_ref", Vector3, self.callback_ref, queue_size=1)
        self.sub_encoders = rospy.Subscriber("/encoders", Vector3Stamped, self.callback_encoders, queue_size=1)
        self.pub_pwm = rospy.Publisher("/mobot/motor_vel", Vector3, queue_size=1)

    # Subscriber callback
    def callback_ref(self, ref_input):
        self.ref_input = ref_input

    def callback_encoders(self, encoders_stamped):
        self.time_now = encoders_stamped.header.stamp
        self.encoders_now = np.array([encoders_stamped.vector.x, encoders_stamped.vector.y, encoders_stamped.vector.z])
        self.update()

    def udpate(self):
        self.calculate_ang_vel()
        
    def apply_control(self):
        ref_input = self.ref_input
        feedback = self.ang_vel

        control_effort = self.k_p * (self.encoders_now - self.encoders_prev)
    
    def calculate_dt_and_update_prev(self):
        dt_dur = self.time_now - self.time_prev # Unit: ros duration
        self.time_prev = self.time_now
        self.encoders_prev = self.encoders_now
        dt_secs = dt_dur.to_sec()# Unit: seconds
        return dt_secs
    
    def calculate_ang_vel(self):
        ticks_passed = self.encoders_now - self.encoders_prev
        time_passed = self.calculate_dt_and_update_prev()
        self.ang_vel = (ticks_passed / time_passed) * (2 * math.pi)) / 6533



# Calculates trapezoid integral of incoming data
# Takes new data and step size as input
# Holds integral value and previous data
# Outputs new integral value
class Integrator:
    def __init__(self, initial_data):
        self.prev = initial_data
        self.integral = initial_data

    def update(self, data_new, dt):
        self.integral += (data_new + self.prev) * dt / 2
        # Save new data to be used as previous data in the next step
        self.prev = data_new
        return self.integral