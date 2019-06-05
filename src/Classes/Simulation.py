#!/usr/bin/env python
import rospy
import Queue
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

# Gets proxy pose from simulink
# Publishes proxy pose to gazebo
# Publishes delayed proxy pose as slave pose to gazebo
class Simulation:

    def __init__(self, queue_size=250):
        rospy.init_node("simulation_node")
        self.r = rospy.Rate(10)
        self.proxy = ModelState()
        self.slave = ModelState()
        self.proxy.model_name = "mobot_proxy"
        self.slave.model_name = "mobot_simple_1"
        self.proxy.pose.position.z = 0.075
        self.slave.pose.position.z = 0.075

        # Storage for proxy position to be published after a delay
        self.delayQueue = Queue.Queue(maxsize=queue_size)

        # Get calculated proxy pose from simulink
        self.sub = rospy.Subscriber("/sim/mobot_pose", Pose, self.callback, queue_size=1)
        # Publish slave and proxy pose to gazebo
        self.pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        
    # Adds current pose of proxy to the queue
    # Gets delayed pose of proxy as slave pose if the queue is full
    # Slave pose is not published until the queue is full
    def update_queue_and_publish(self):
        # Publish proxy pose first
        self.pub.publish(self.proxy)

        # Prevents publishing slave pose until the queue is filled
        if self.delayQueue.full():
            # Get first-in proxy pose as delayed slave pose from the queue
            self.slave.pose = self.delayQueue.get()
            #self.slave.model_name = "mobot_simple_1"

            # Publish slave pose
            self.pub.publish(self.slave)

        # Put proxy pose to the queue
        self.delayQueue.put(self.proxy.pose)


    # Subscriber callback
    def callback(self, msg_in):
        self.proxy.pose = msg_in
        self.proxy.pose.position.z = 0.075
        self.update_queue_and_publish()
    
