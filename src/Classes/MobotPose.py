#!/usr/bin/env python
import rospy
import numpy as np
from transforms3d import quaternions as qt
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose2D
#from sensor_msgs.msg import Image

# TODO get pose data from joystick/simulink

# Gets proxy pose from simulink
# Publishes proxy pose to gazebo
# Publishes delayed proxy pose as slave pose to gazebo
class MobotPose:

    def __init__(self, rate=30):
        rospy.init_node("mobot_vel_to_pose")
        self.r = rospy.Rate(rate)
        # Mobot pose init (0)
        self.pose_now = Pose2D(00.0, 0.0, 0.0)
        self.pose_pub = Pose2D()
        self.local_vel = np.array([0, 0])
        self.ang_vel = 0
        self.global_vel = np.array([0.0, 0.0])
        self.time_prev = rospy.Time.now().to_sec()
        self.ang_pos_integrator = Integrator(0)
        self.lin_pos_integrator = Integrator(np.array([0.0, 0.0])) # default: self.global_vel as input
        self.dt = 0.01

        self.sub = rospy.Subscriber("/proxy/vel", Vector3, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/proxy/pose_nondelayed", Pose2D, queue_size=1) # default: /proxy/pose

    # Subscriber callback
    def callback(self, msg_in):
        self.local_vel = np.array([msg_in.x, msg_in.y])
        self.ang_vel = msg_in.z
        #self.update()

    def update(self):
        self.calculate_dt()
        self.update_ang_pos(self.ang_vel)
        self.calculate_global_vel(self.local_vel)
        self.update_pos()
        self.pub.publish(self.pose_pub)

    def calculate_dt(self):
        now = rospy.Time.now().to_sec()
        self.dt = now - self.time_prev
        self.time_prev = now 

    def update_ang_pos(self, ang_vel):
        self.pose_now.theta = self.ang_pos_integrator.update(ang_vel, self.dt)

    # Calculates global velocity from local velocity or does nothing if using global velocity mode
    def calculate_global_vel(self, local_vel):
        # Uncomment following three lines  for local velocity mode
        #theta = self.pose_now.z 
        #rot_max = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        #self.global_vel = np.dot(rot_max, local_vel)

        # Comment for local velocity mode
        self.global_vel = local_vel

    def update_pos(self):
        global_pos = self.lin_pos_integrator.update(self.global_vel, self.dt)
        self.pose_pub.x = global_pos[0]
        self.pose_pub.y = global_pos[1]
        self.pose_pub.theta = self.pose_now.theta
        """
        q = qt.axangle2quat([0, 0, 1], self.pose_now.z, is_normalized=True)
        self.pose_pub.orientation.x = q[1]
        self.pose_pub.orientation.y = q[2]
        self.pose_pub.orientation.z = q[3]
        self.pose_pub.orientation.w = q[0]
        """

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