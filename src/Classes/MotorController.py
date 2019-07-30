#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from Classes.Mobot import inverse_kinematics_SI

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
        
        self.sub_motor_ref = rospy.Subscriber("/mobot/motor_vel_desired", Vector3, self.callback_motor_vel, queue_size=1)
        self.sub_robot_ref = rospy.Subscriber("/mobot/robot_vel_desired", Vector3, self.callback_robot_vel, queue_size=1)
        self.sub_feedback = rospy.Subscriber("/mobot/motor_vel_filtered", Vector3, self.callback_feedback, queue_size=1)
        self.pub_pwm = rospy.Publisher("/mobot/motor_vel", Vector3, queue_size=1)

    # Subscriber callback
    def callback_motor_vel(self, ref_input):
        self.ref_input[0] = ref_input.x
        self.ref_input[1] = ref_input.y
        self.ref_input[2] = ref_input.z

    def callback_robot_vel(self, robot_vel):
        self.ref_input = inverse_kinematics_SI(robot_vel.x, robot_vel.y, robot_vel.z) # Copy class variables to local variables to prevent callback interrupt problems
        # inverse kinematics etc.
        print(self.ref_input)

    def callback_feedback(self, msg_in):
        self.feedback[0] = msg_in.x
        self.feedback[1] = msg_in.y
        self.feedback[2] = msg_in.z

    def update(self):
        # Copy class variables to local variables to prevent callback interrupt problems
        ref_input = self.ref_input
        feedback = self.feedback

        error = ref_input - feedback
        # P + velocity feedforward control
        control_effort = self.k_p * error + ref_input * self.feedfwd_mult
        # Convert numpy array to Vector3 to be published
        pwm_out = Vector3(control_effort[0], control_effort[1], control_effort[2])

        self.pub_pwm.publish(pwm_out)
        