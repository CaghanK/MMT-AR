#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import Duration

rospy.init_node("split_encoders")

#msg_out1 = Int64()
#msg_out2 = Int64()
#msg_out3 = Int64()
encoder_data_prev = float()
dTheta = Int64()


time_now = rospy.Time()
time_prev = rospy.Time()
#dt = Duration()

r = rospy.Rate(100)


def callback(encoder_data):
    global time_now
    global time_prev
    global encoder_data_prev
    global dTheta

    #dt_secs = Float64()

    msg_out1 = encoder_data.x
    msg_out2 = encoder_data.y
    msg_out3 = encoder_data.z

    pub1.publish(msg_out1)
    pub2.publish(msg_out2)
    pub3.publish(msg_out3)

    time_now = rospy.Time.now()
    dt_dur = time_now - time_prev
    time_prev = time_now
    dt_secs = dt_dur.to_sec()
    dTheta = msg_out3 - encoder_data_prev
    ang_vel = dTheta / dt_secs
    ang_vel = ang_vel * 360 / 6533
    pub_time.publish(ang_vel)
    encoder_data_prev = msg_out3


rospy.Subscriber("encoders", Vector3, callback)
pub1 = rospy.Publisher("/split_encoders/encoder1", Int64, queue_size = 1)
pub2 = rospy.Publisher("/split_encoders/encoder2", Int64, queue_size = 1)
pub3 = rospy.Publisher("/split_encoders/encoder3", Int64, queue_size = 1)
pub_time = rospy.Publisher("/time_test", Float64 , queue_size = 1)
    

while True:
    while not rospy.is_shutdown():
        r.sleep()

