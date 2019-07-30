#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose2D
from Classes.Mobot import inverse_kinematics_SI

# Gets encoder data from arduino
# Applies modified PD-feedforward control
# Publishes control output as pwm value to arduino
class PoseController:

    def __init__(self, rate=100):
        rospy.init_node("mobot_pose_control")
        self.r = rospy.Rate(rate)
        # Mobot pose init (0)
        self.feedback = np.array([0.0, 0.0, 0.0])
        self.ref_input = np.array([0.0, 0.0, 0.0])

        # Control Parameters
        self.k_p = np.array([0.25, 0.25, -0.25]) # for x,y axes and theta
        
        # TODO: change to topic to /proxy/pose_delayed
        self.sub_ref = rospy.Subscriber("/proxy/pose", Pose2D, self.callback_ref, queue_size=1)
        self.sub_feedback = rospy.Subscriber("/pose2D", Pose2D, self.callback_feedback, queue_size=1) # TODO fix topic name
        self.pub_vel = rospy.Publisher("/mobot/motor_vel", Vector3, queue_size=1) # default: /mobot/robot_vel_desired

    # Subscriber callback
    def callback_ref(self, ref_input):
        self.ref_input[0] = ref_input.x
        self.ref_input[1] = ref_input.y
        self.ref_input[2] = ref_input.theta  # Theta
        self.update()

    def callback_feedback(self, msg_in):
        self.feedback[0] = -msg_in.y
        self.feedback[1] = msg_in.x
        # Some calculations to get Theta from quaternion
        self.feedback[2] = msg_in.theta

    def update(self):
        # Copy class variables to local variables to prevent callback interrupt problems
        ref_input = self.ref_input
        feedback = self.feedback

        error = ref_input - feedback
        # P control
        global_velocity_desired = np.multiply(self.k_p, error) # control effort in world coordinates
        # Rotate global linear velocity to local coordinates
        local_lin_velocity_desired = self.rotate2local(global_velocity_desired[0], global_velocity_desired[1], self.feedback[2])

        # Convert numpy array to Vector3 to be published
        local_vel_desired = Vector3(local_lin_velocity_desired[0], local_lin_velocity_desired[1], global_velocity_desired[2]) # since angular velocity is equal in both local and global
        motor_vel_desired = inverse_kinematics_SI(local_lin_velocity_desired[0], local_lin_velocity_desired[1], global_velocity_desired[2])
        print(motor_vel_desired)
        multiplier = 100
        pub_motor_vel = Vector3(multiplier*motor_vel_desired[0], multiplier*motor_vel_desired[1], multiplier*motor_vel_desired[2])
        self.pub_vel.publish(pub_motor_vel) # default: local_vel_desired

    def rotate2local(self, x, y, theta):
        global_vel = np.array([x, y])
        # minus theta: rotation is in opposite direction
        cos_theta = np.cos(-theta)
        sin_theta = np.sin(-theta)
        rot_max = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
        local_vel = np.dot(rot_max, global_vel)
        return local_vel

        
        