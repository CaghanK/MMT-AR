#!/usr/bin/env python

import rospy
import numpy as np
import math
from rospy import Time
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import Duration

rospy.init_node("calculate_velocity")

#dTheta = Float64()

time_now = rospy.Time(0)
time_prev = rospy.Time(0)
#dt = Duration()
#dt_secs = Float64()

prev_encoders = np.array([0, 0, 0])

deltaTicks = np.array([0, 0, 0])
dTicks = np.array([0, 0, 0])
ang_vel = np.array([0, 0, 0])

r = rospy.Rate(100)


def callback(encoder_data):
    global time_now
    global time_prev
    global prev_encoders
    global deltaTicks
    global dTicks
    global ang_vel

    msg_out = Vector3()

    '''Calculate dt and save previous time'''
    time_now = encoder_data.header.stamp # rospy.Time()
    dt_dur = time_now - time_prev # rospy.Duration()
    time_prev = time_now  # rospy.Time()
    dt_secs = dt_dur.to_sec() # float

    '''Incoming encoder data as numpy array'''
    curr_encoders = np.array([encoder_data.vector.x, encoder_data.vector.y, encoder_data.vector.z])

    '''Previous method for backup'''
    # dTheta = curr_encoders - prev_encoders # np.array
    # ang_vel = dTheta / dt_secs # np.array
    # ang_vel = ang_vel * 360 / 6533 # np.array

    '''Tick difference / Time difference and convert to radians'''
    deltaTicks = curr_encoders - prev_encoders # numpy.array()
    dTicks = deltaTicks / dt_secs # ticks per sec
    ang_vel = dTicks * (2 * math.pi)
    ang_vel = ang_vel / 6533 # rad per sec
    #ang_vel = dTicks
    '''Fill in the outgoing msg'''
    msg_out.x = ang_vel[0]
    msg_out.y = ang_vel[1]
    msg_out.z = ang_vel[2]

    pub_vel.publish(msg_out)
    prev_encoders = curr_encoders[:] # save curr encoder value to be used as prev value in next cycle


rospy.Subscriber("encoders", Vector3Stamped, callback) # gets encoder counter data
pub_vel = rospy.Publisher("/motor_vel", Vector3 , queue_size = 1) # publish velocity data
    

while True:
    while not rospy.is_shutdown():
        r.sleep()

