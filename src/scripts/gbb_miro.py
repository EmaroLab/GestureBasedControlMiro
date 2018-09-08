#!/usr/bin/env python

################################################################

from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

import math
import numpy
import time
import sys
from miro_constants import miro

from datetime import datetime
## \file gbb_miro.py
## DESCRIPTION OF THE NODE
## The node subscribes to the linear and angular velocity mapped in the imu_data_map node and publish a platform_control message
## A platform_control message contains linear and angular velocities, the lightening pattern of miro's body and other tipes of messages.
##  More in details:
##  Subscribe to the topic /imu_mapping
## read from that topic the value from the smartwatch correctly mapped into miro body velocity
##  Publish on /gbb a platform_control msg containing miro body velocity and a lightening pattern based on smartwatch commands
##  For example: if the command of the smartwatch is a rotation towards right miro will turn right and the right part of its body will lit

class GestureBased():

    def __init__(self):

        self.sub_imu_mapping = rospy.Subscriber('/imu_mapping',Twist,self.callback_gbb,queue_size=1)
        self.pub_platform_control = rospy.Publisher('/gbb', platform_control, queue_size=0)


    def callback_gbb(self,vel_msg):

        q=platform_control()

        self.body_vel=Twist()
        self.body_vel.linear.x = vel_msg.linear.x
        self.body_vel.angular.z = vel_msg.angular.z

        q.body_vel = self.body_vel

        #lightening pattern

        if -5< vel_msg.linear.x < 5 and -0.1 < vel_msg.angular.z<0.1 :
            q.lights_raw = [0,255,255,0,255,255,0,255,255,0,255,255,0,255,255,0,255,255]

        if vel_msg.angular.z > 0 and vel_msg.angular.z > 0.1 :
            q.lights_raw = [0,0,255,0,0,255,0,0,255,0,0,0,0,0,0,0,0,0]

        if vel_msg.angular.z < 0 and vel_msg.angular.z < - 0.1 :
            q.lights_raw = [0,0,0,0,0,0,0,0,0,255,0,255,0,0,255,0,0,255]

        self.pub_platform_control.publish(q)
            

        #rospy.loginfo(sonar_msg.sonar_range)

    def main (self):
        rospy.spin()

if __name__== '__main__':
    rospy.init_node('gbb_miro')
    gesture_based = GestureBased()
    gesture_based.main()