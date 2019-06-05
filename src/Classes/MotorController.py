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
        self.time_now = rospy.Time(0)
        self.feedback = np.array([0, 0, 0])
        self.ref_input = np.array([0, 0, 0])
        self.dt_secs = 0

        # Control Parameters
        self.feedfwd_mult = 20
        self.k_p = 35
        
        self.sub_ref = rospy.Subscriber("/motor_ref", Vector3, self.callback_ref, queue_size=1)
        self.sub_feedback = rospy.Subscriber("/motor_vel", Vector3, self.callback_feedback, queue_size=1)
        self.pub_pwm = rospy.Publisher("/mobot/motor_vel", Vector3, queue_size=1)

    # Subscriber callback
    def callback_ref(self, ref_input):
        self.ref_input[0] = ref_input.x
        self.ref_input[1] = ref_input.y
        self.ref_input[2] = ref_input.z

    def callback_feedback(self, msg_in):
        self.feedback[0] = msg_in.x
        self.feedback[1] = msg_in.y
        self.feedback[2] = msg_in.z

    def udpate(self):
        # Copy class variables to local variables to prevent callback interrupt problems
        ref_input = self.ref_input
        feedback = self.feedback

        error = ref_input - feedback
        # P + velocity feedforward control
        control_effort = self.k_p * error + ref_input * self.feedfwd_mult
        # Convert numpy array to Vector3 to be published
        pwm_out = Vector3(control_effort[0], control_effort[1], control_effort[2])

        self.pub_pwm.publish(pwm_out) 
        