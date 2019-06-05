#!/usr/bin/env python

import rospy
import numpy
from rospy import Time
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import Duration

import Controller.PID as PID

rospy.init_node("PID_test")
msg_out = Vector3()
set_point = Float64()
set_point.data = 250.0

pid = PID.PID(0.1, 0.0, 0.01, -255, 255)
#dt = Duration()
#dt_secs = Float64()

r = rospy.Rate(100)

def callback(encoder_data):
    
    control_effort = pid.update(encoder_data.data)
    rospy.loginfo("integral =  %f", pid.integralSum)
    msg_out.z = -control_effort
    pub_control.publish(msg_out)

def tuner_callback(tunings):
    pid.setTunings(tunings.x, tunings.y, tunings.z)

def ref_callback(setpoint):
    pid.setRefValue(setpoint.data)


rospy.Subscriber("/state", Float64, callback)
rospy.Subscriber("/setpoint", Float64, ref_callback)
rospy.Subscriber("/PID_constants", Vector3, tuner_callback)
pub_control = rospy.Publisher("/mobot/motor_vel", Vector3, queue_size = 1)
#pub_time = rospy.Publisher("/velocity_test", Float64 , queue_size = 1)
# pub_time = rospy.Publisher("/state", Float64 , queue_size = 1)
# pub_setpoint = rospy.Publisher("/setpoint", Float64 , queue_size = 1)
    

while True:
    while not rospy.is_shutdown():
        r.sleep()

