#!/usr/bin/env python

################################################################

import rospy
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


##@file oab_miro.py
## The node subscribes to the sonar and check the presence of an obstacle
## If the obstacle is present the strategy is to start turning in the direction given by the user through the smartwatch until no obstacle is present anymore
## More in details: 
## Subscribe to the topic /platform/sensors
## Read the value from the sonar and check the presence of an obstacle
## if there is an obstacle implement the obstacle avoidance behaviour 
## The obstacle avoidance behavior:
## Subscribe to the topic /inertial
## save the last acceleration value
## if last acc[1]> 0 save the right
## if last acc[1]<0 save the left
## Then MiRo starts turnin in the direction of the last value saved 
## few degree each time the same obstacle is detected until there is no obstacle anymore

##The class ObstacleAvoidance implements the obstacle Avoidance Behavior

class ObstacleAvoidance():

        
    def __init__(self):

        #topic root
        ## Allow to switch from real robot to simulation from launch file
        self.robot_name = rospy.get_param ( '/robot_name', 'rob01')
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root

        ## Signal if there is an obstacle or not
        self.obstacle = False
        ## Command suggested by the user to start turning towards left or right to avoid the obstacle
        self.last_command=0
        ## Counter that increase the each time the same obstacle is encoutered
        self.counter=0
        ## Startig direction that depends on the last_command given by the user
        self.start_dir=0
        ## Sonar theshold below which an obstacle is encountered
        self.danger_threshold = rospy.get_param ('sonar_threshold',0.3)
        ## Linear and Angular velocities that will be part of the platform_control message
        self.body_vel=Twist()
        
        ## Subscriber to the topic /platform/sensors a message of type platform_sensors that cointains the sonar readings
        self.sub_sonar_data = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_oab,queue_size=1)
        ## Subscriber to the topic /inertial or better imu mapping a message of type IMU or Twist
        self.sub_imu_data = rospy.Subscriber('/inertial',Imu,self.callback_last_command,queue_size=1)
        ## Publisher to the topic /oab a message of type platform_control which corresponds to the Obstacle Avoidance Behavior
        self.pub_platform_control = rospy.Publisher('/oab', platform_control, queue_size=0)

    ## Callback function that receive and save the user's command regarding in which direction start turning to avoid the obstacle
    def callback_last_command(self, sw_data):

        self.last_command = sw_data.linear_acceleration.y
        
    ## Callback that receives the data from the robot sensors, and uses the information given by the sonar sensor to evaluate the presence of an obstacle.
    ## @n If an obstacle is detected then is used the information related to the user's comand to start turning in one direction of few degree.
    ## @n Each time the same obstacle is detected the degrees respect to wich turns increases of a fixed quantity.
    ## @n This angular velocity and a red lightening pattern for Miro's body are used to construct the platform_control message to publish

    
    def callback_oab(self,sonar_data):

        q=platform_control()
        sonar_value = sonar_data.sonar_range.range

        self.obstacle = sonar_value > 0.01 and sonar_value < self.danger_threshold

        if self.obstacle:

            if self.last_command < 0: #dx

                self.start_dir=-0.035
                self.counter=self.counter-0.04

            if self.last_command > 0: #sx
                
                self.start_dir=0.035
                self.counter=self.counter+0.04

            self.body_vel.linear.x=0.0
            self.body_vel.angular.z=self.start_dir+self.counter

            q.body_vel = self.body_vel
            q.lights_raw = [255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0]
            #q.sound_index_P2 = 3
            self.pub_platform_control.publish(q)

        else:
            self.counter=0
        
        #rospy.loginfo(sonar_msg.sonar_range)

    def main (self):
        rospy.spin()

if __name__== '__main__':
    rospy.init_node('oab_miro')
    oab = ObstacleAvoidance()
    oab.main()