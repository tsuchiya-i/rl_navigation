#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import sys, os
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

def callback(data):
    pub_vel =  publish_vel(data)
    pub.publish(pub_vel)

def main():
    rospy.init_node('controller', anonymous=True)
    rospy.Subscriber("/joy" ,Joy ,callback)
    rospy.spin()

def publish_vel(data):
    pub_vel = Twist()

    Lstick_RL = data.axes[0]
    Lstick_FB = data.axes[1]
    pub_vel.linear.x = Lstick_FB
    pub_vel.angular.z = Lstick_RL


    R2 = (-data.axes[5]+1.0)/2.0
    L2 = (data.axes[2]-1.0)/2.0

    if data.buttons[7] == 1:
        pub_vel.linear.x = R2
    if data.buttons[6] == 1:
        pub_vel.linear.x = L2

    if data.buttons[0] == 1:
        pub_vel.linear.x = 0
        pub_vel.angular.z = 0
    if data.buttons[1] == 1:
        pub_vel.linear.x = 1.0
    if data.buttons[3] == 1:
        pub_vel.linear.x = -1.0

    return pub_vel

if __name__ == '__main__':
    main()
