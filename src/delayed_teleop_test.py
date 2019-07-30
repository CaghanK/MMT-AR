#!/usr/bin/env python

import rospy
from Classes.DelayedTeleopTest import DelayedVideo

if __name__ == "__main__": 
    video_feed = DelayedVideo()
    while not rospy.is_shutdown():
        video_feed.update()
        video_feed.r.sleep()