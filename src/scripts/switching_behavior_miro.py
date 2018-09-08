#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image,CompressedImage,Range
from geometry_msgs.msg import Twist

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream


import math
import numpy
import time
import sys
from miro_constants import miro

from datetime import datetime

## DESCRIPTION OF THE NODE
## The node allows to switch from the gesture based behaviour published by the gbb_miro and the obstacle avoidance behavior published by oab_miro
## The switch depends on the presence of an obstacle 

## More in details:
## Subscribe to the topic /platform/sensors
## Read the value from the sonar and check the presence of an obstacle
## Subscribe to the topic /gbb 
## Subscribe to the topic /oab
## Publish the gesture based behavior on miro platform/control
## when an obstacle is encountered Publish the obstacle avoidance behavior on miro platform/control

class SwitchingBehavior():


    def __init__(self):

        #topic root
        #allow to switch from real robot to simulation from launch file
        self.robot_name = rospy.get_param ( '/robot_name', 'rob01')
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root

        #node rate
        self.rate = rospy.get_param('rate',200)
        
        #Switching Condition
        self.danger_threshold = rospy.get_param ('sonar_threshold',0.3)
        self.safe = True
        self.sub_sonar_data = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_switching_condition,queue_size=1)
        
        #Gesture Based Behavior
        self.q_gbb = platform_control()
        self.sub_gbb = rospy.Subscriber('/gbb', platform_control, self.callback_gbb, queue_size=1)

        #Obstacle Avoidance Behavior
        self.q_oab = platform_control()
        self.sub_oab = rospy.Subscriber('/oab', platform_control, self.callback_oab, queue_size=1)

        self.pub_behavior = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=0)


    def callback_switching_condition(self,sonar_data):

        sonar_value = sonar_data.sonar_range.range
        if sonar_value > 0.01 and sonar_value < self.danger_threshold:

            self.safe = False
            print "OBSTACLE DETECTED"

        else:
            self.safe = True
            print "NO OBSTACLE"

    def callback_gbb(self,gbb):

        self.q_gbb = gbb

    def callback_oab( self, oab):

        self.q_oab = oab

    def switching_behavior(self):

        q = platform_control
        r = rospy.Rate(self.rate)

        while not rospy.is_shutdown():

            if self.safe:
                q = self.q_gbb
                print "|GESTURE BASED BEHAVIOR|"

            elif not self.safe:
                q = self.q_oab
                print "|OBSTACLE AVOIDANCE BEHAVIOR|"

            self.pub_behavior.publish(q)

            r.sleep()

if __name__== '__main__':

    rospy.init_node('switching_behavior_miro')
    sb = SwitchingBehavior()
    sb.switching_behavior()
        