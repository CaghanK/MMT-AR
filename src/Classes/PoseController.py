#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose

# Gets encoder data from arduino
# Applies modified PD-feedforward control
# Publishes control output as pwm value to arduino
class MotorController:

    def __init__(self, rate=100):
        rospy.init_node("mobot_pose_control")
        self.r = rospy.Rate(rate)
        # Mobot pose init (0)
        self.feedback = np.array([0, 0, 0])
        self.ref_input = np.array([0, 0, 0])

        # Control Parameters
        self.k_p = np.array([2, 2, 1]) # for x,y axes and theta
        
        self.sub_ref = rospy.Subscriber("/sim/mobot_pose", Vector3, self.callback_ref, queue_size=1)
        self.sub_feedback = rospy.Subscriber("/pose", Pose, self.callback_feedback, queue_size=1) # TODO fix topic name
        self.pub_vel = rospy.Publisher("/mobot/robot_vel", Vector3, queue_size=1)

    # Subscriber callback
    def callback_ref(self, ref_input):
        self.ref_input[0] = ref_input.x
        self.ref_input[1] = ref_input.y
        self.ref_input[2] = ref_input.z  # Theta

    def callback_feedback(self, msg_in):
        self.feedback[0] = msg_in.x
        self.feedback[1] = msg_in.y
        # Some calculations to get Theta from quaternion
        self.feedback[2] = msg_in.z

    def udpate(self):
        # Copy class variables to local variables to prevent callback interrupt problems
        ref_input = self.ref_input
        feedback = self.feedback

        error = ref_input - feedback
        # P + velocity feedforward control
        control_effort = np.multiply(self.k_p, error)
        # Convert numpy array to Vector3 to be published
        vel_out = Vector3(control_effort[0], control_effort[1], control_effort[2])

        self.pub_vel.publish(vel_out) 
        