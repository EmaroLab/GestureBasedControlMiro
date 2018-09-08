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

##DESCRIPTION OF THE NODE

## The node subscribes to the sonar and check the presence of an obstacle
## If the obstacle is present the strategy is to start turning in the direction given by the user through the smartwatch until no obstacle is present anymore
## More in details: 
## Subscribe to the topic /platform/sensors
## Read the value from the sonar and check the presence of an obstacle
## if there is an obstacle implement the obstacle avoidance behaviour 
## The obstacle avoidance behavior:
## Subscribe to the topic \inertial
## save the last acceleration value
## if last acc[1]> 0 save the right
## if last acc[1]<0 save the left
## Then MiRo starts turnin in the direction of the last value saved 
## few degree each time the same obstacle is detected until there is no obstacle anymore

class ObstacleAvoidance():

        
    def __init__(self):

        #topic root
        #allow to switch from real robot to simulation from launch file
        self.robot_name = rospy.get_param ( '/robot_name', 'rob01')
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root

        self.obstacle = False
        self.last_command=0
        self.counter=0
        self.start_dir=0
        self.danger_threshold = rospy.get_param ('sonar_threshold',0.3)
        
        self.sub_sonar_data = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_oab,queue_size=1)
        self.sub_imu_data = rospy.Subscriber('/inertial',Imu,self.callback_last_command,queue_size=1)
        self.pub_platform_control = rospy.Publisher('/oab', platform_control, queue_size=0)

    
    def callback_last_command(self, sw_data):

        self.last_command = sw_data.linear_acceleration.y
        

    def callback_oab(self,sonar_data):

        q=platform_control()
        sonar_value = sonar_data.sonar_range.range

        self.obstacle = sonar_value > 0.01 and sonar_value < self.danger_threshold

        if self.obstacle:

            #if self.last_command < 0: #dx

            self.start_dir=-0.035
            self.counter=self.counter-0.04

            #if self.last_command > 0: #sx
                
            #self.start_dir=0.035
            #self.counter=self.counter+0.04

            self.body_vel=Twist()
        
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