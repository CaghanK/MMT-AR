#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

rospy.init_node("simulink_to_gazebo")

r = rospy.Rate(100)
msg_out = ModelState()


def callback(mobot_pose):
    msg_out.model_name = "mobot_simple_1"
    #msg_out.reference_frame = "world"
    msg_out.pose.position.z = 0.075
    msg_out.pose.orientation.z = 1.0


    msg_out.pose.position.x = mobot_pose.position.x
    msg_out.pose.position.y = mobot_pose.position.y
    msg_out.pose.orientation.x = mobot_pose.orientation.x
    msg_out.pose.orientation.y = mobot_pose.orientation.y
    msg_out.pose.orientation.z = mobot_pose.orientation.z
    msg_out.pose.orientation.w = mobot_pose.orientation.w

    pub_pose.publish(msg_out)

rospy.Subscriber("mobot_pose", Pose, callback)
pub_pose = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size = 1)
    

while True:
    while not rospy.is_shutdown():
        r.sleep()

