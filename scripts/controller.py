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


class joy_controller():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.forced_stop = False

    def callback(self, data):
        if sum(data.buttons[8:11]) > 0:
            self.forced_stop = not self.forced_stop
        self.publish_vel(data)
    
    def publish_vel(self, data):
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
        self.pub.publish(pub_vel)

    def publish_zero(self):
        pub_vel = Twist()
        pub_vel.angular.z = 0
        pub_vel.linear.x = 0
        self.pub.publish(pub_vel)

    def main(self):
        rospy.init_node('controller', anonymous=True)
        rospy.Subscriber("/joy" ,Joy ,self.callback)
        r = rospy.Rate(20) # 10hz
        while not rospy.is_shutdown():
            if self.forced_stop:
                self.publish_zero()
            r.sleep()
        rospy.spin()

if __name__ == '__main__':
    joycon = joy_controller()
    joycon.main()
