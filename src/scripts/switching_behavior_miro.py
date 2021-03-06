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

## \file switching_behavior_miro.py
## \brief The node switching_behavior_miro.py allows to switch from the gesture based behaviour published by the gbb_miro.py and the obstacle avoidance behavior published by oab_miro.py
## @n The switch depends on the presence of an obstacle. 
## @n More in details:
## @n Subscribe to the topic /platform/sensors
## @n Read the value from the sonar and check the presence of an obstacle
## @n Subscribe to the topic /gbb 
## @n Subscribe to the topic /oab
## @n Publish the gesture based behavior on miro platform/control
## @n When an obstacle is encountered Publish the obstacle avoidance behavior on miro platform/control

## The class SwitchingBehavior  allows to switch between the two possible behaviors based on the presence of an obstacle

class SwitchingBehavior():

    ## Constructor
    def __init__(self):

        #topic root
        ## Allow to switch from real robot to simulation from launch file
        self.robot_name = rospy.get_param ( '/robot_name', 'rob01')
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root

        ## Node rate
        self.rate = rospy.get_param('rate',200)
        
        #Switching Condition
        ## Sonar theshold below which an obstacle is encountered
        self.danger_threshold = rospy.get_param ('sonar_threshold',0.3)

        ## Signal if there is an obstacle or not
        self.safe = True
        ## Subscriber to the topic /platform/sensors a message of type platform_sensors that cointains the sonar readings
        self.sub_sonar_data = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_switching_condition,queue_size=1)
        
        ## platform_control object that rapresents the Gesture Based Behavior
        self.q_gbb = platform_control()

        ## Subscriber to the topic /gbb a message of type platform_control that rapresents the Gesture Based Behavior
        self.sub_gbb = rospy.Subscriber('/gbb', platform_control, self.callback_gbb, queue_size=1)

        ## platform_control object that rapresents the Obstacle Avoidance Behavior
        self.q_oab = platform_control()

        ## Subscriber to the topic /oab a message of type platform_control that rapresents the Obstacle Avoidance Behavior
        self.sub_oab = rospy.Subscriber('/oab', platform_control, self.callback_oab, queue_size=1)
        
        ## Publisher to the topic /platform/control on the robot a message of type platform_control that represents the selected behavior
        self.pub_behavior = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=0)

    ## Callback that receives the data from the robot sensors, and uses the information given by the sonar sensor to evaluate the presence of an obstacle
    def callback_switching_condition(self,sonar_data):

        sonar_value = sonar_data.sonar_range.range
        if sonar_value > 0.01 and sonar_value < self.danger_threshold:

            self.safe = False
            print "OBSTACLE DETECTED"

        else:
            self.safe = True
            print "NO OBSTACLE"

    ## Callback that receives the Gesture Based Behavior as a platform_control message
    def callback_gbb(self,gbb):

        self.q_gbb = gbb

    ## Callback that receives the Obstacle Avoidance Behavior as a platform_control message
    def callback_oab( self, oab):

        self.q_oab = oab

    ## Function that based on the value of the variable safe, that represent the presence of the obstacle, publish the behavior selected on Miro
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
        