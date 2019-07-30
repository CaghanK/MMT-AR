#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
from Classes.Mobot import inverse_kinematics

# Reduces the publish rate of velocity messages to the robot
# Gets position and button data from falcon
# Saves incoming data in 1kHz
# Publishes velocity in 50-100 Hz
class FalconVelocityPublisher:

    def __init__(self, rate=50):
        rospy.init_node("falcon_vel_generator")
        self.r = rospy.Rate(rate)

        self.MAX_LIN_VEL = 0.5
        self.MAX_ANG_VEL = 3.1416/4
        self.falcon_pos = [0.0, 0.0]
        self.falcon_buttons = [0.0, 0.0, 0.0]
        self.msg_vel = Vector3()
        
        self.sub_falcon = rospy.Subscriber("/falcon/joystick", Joy, self.callback_falcon, queue_size=1)
        self.pub_pwm = rospy.Publisher("/mobot/motor_vel", Vector3, queue_size=1)

    # Subscriber callback
    def callback_falcon(self, falcon_joy):

        self.falcon_pos[0] = falcon_joy.axes[0]
        self.falcon_pos[1] = -falcon_joy.axes[2]
        self.falcon_buttons[0] = falcon_joy.buttons[3] # Left button
        self.falcon_buttons[1] = falcon_joy.buttons[2] # Middle button
        self.falcon_buttons[2] = falcon_joy.buttons[0] # Right button

    def update(self):
        # Copy class variables to local variables to prevent callback interrupt problems
        x_axis = self.falcon_pos[0]
        y_axis = self.falcon_pos[1]
        left_button = self.falcon_buttons[0]
        mid_button = self.falcon_buttons[1]
        right_button = self.falcon_buttons[2]

        if abs(x_axis) < 5.0:
            x_axis = 0.0

        if abs(y_axis) < 5.0:
            y_axis = 0.0
    
        joy_x = x_axis / 50
        joy_y = y_axis / 50
        ang_vel = 0
    
        if right_button == 1 and left_button == 0: # Turn right
            ang_vel = self.MAX_ANG_VEL
        if left_button == 1 and right_button == 0: # Turn left
            ang_vel = -self.MAX_ANG_VEL
        joy_z = ang_vel

        vel_array = inverse_kinematics(joy_x, joy_y, joy_z)
        self.msg_vel.x = 1.5 * vel_array[0]
        self.msg_vel.y = 1.5 * vel_array[1]
        self.msg_vel.z = 1.5 * vel_array[2]

        if mid_button == 1:
            self.msg_vel.x = 0
            self.msg_vel.y = 0
            self.msg_vel.z = 0

        self.pub_pwm.publish(self.msg_vel)
        