#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from rosfalcon.msg import falconForces
#from sensor_msgs.msg import Image

# TODO get pose data from joystick/simulink

# Gets proxy pose from simulink
# Publishes proxy pose to gazebo
# Publishes delayed proxy pose as slave pose to gazebo
class ForceGenerator:

    def __init__(self, rate=100):
        rospy.init_node("force_generator")
        self.r = rospy.Rate(rate)
        # Proxy pose holder
        self.proxy_pose = Pose()
        self.force = falconForces()
        self.sub = rospy.Subscriber("/sim/mobot_pose", Pose, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/falconForce", falconForces, queue_size=1)


    # Subscriber callback
    def callback(self, msg_in):
        self.proxy_pose = msg_in

    def generate_force(self):

        if self.proxy_pose.position.y > 13:
            self.force.Z = (-13 + self.proxy_pose.position.y)*2
        else:
            self.force.Z = 0

        self.pub.publish(self.force)




        

    
