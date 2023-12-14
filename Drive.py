#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
import tf.transformations


#global Lidar
#Lidar = [[],[],[],[]]
global velocity
velocity = Twist()


def Rear_left(msg):    
    global RL
    RL = msg.range

def Rear_right(msg):
    global RR
    RR = msg.range

def Front_left(msg):
    global FL
    FL = msg.range

def Front_right(msg):
    global FR
    FR = msg.range

def lidar_data(msg):
    global Lidardata
    Lidardata = msg.ranges


#rospy.init_node('publish', anonymous=False)
#sub_lidar = rospy.Subscriber('scan',LaserScan, callback= lidar_data)
sub_rearleft = rospy.Subscriber('range/rl',Range , callback= Rear_left)
sub_rearright = rospy.Subscriber('range/rr',Range, callback= Rear_right)
sub_frontleft = rospy.Subscriber('range/fl',Range, callback= Front_left)
sub_frontright = rospy.Subscriber('range/fr',Range, callback= Front_right)   
FR=0
FL=0
RR=0
RL=0

rospy.init_node('Drive')

pub = rospy.Publisher('cmd_vel', Twist, queue_size= 10)

rate = rospy.Rate(2)

while not rospy.is_shutdown():

    if FL>=0.5 and FR >= 2:
        velocity.linear.x = 0.1

    elif FL < 0.2 and FR < 0.2:
            velocity.angular.z = 0.2
    elif FL < 0.1 and FR < 0.1:
            velocity.angular.z = -0.6


    elif FL < 2: #detecting oject to left - must turn righ
      
            velocity.linear.x = 0.01
            velocity.linear.y = 0.0
            velocity.angular.z = 0.3
          
    elif FR < 2:
        velocity.linear.x = 0.01
        velocity.linear.y = 0.0
        velocity.angular.z = -0.3

    pub.publish(velocity)
rate.sleep()
