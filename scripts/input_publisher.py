#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, LaserScan
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry

import cv2
import sys, os
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math

class input_publisher():
    def __init__(self):
        self.input_depth_n = rospy.get_param("/input_publisher/input_depth_n", 18)
        self.visual_odom = rospy.get_param("/input_publisher/visual_odom", False)
        self.human_detection = rospy.get_param("/input_publisher/human_detection", False)
        self.camera_info = rospy.get_param("/input_publisher/yolo_camera_info", "/camera/color/camera_info")
        self.max_goal_dist = rospy.get_param("/input_publisher/max_goal_dist", 20)

        if self.human_detection:
            self.input_data = [0.0]*(self.input_depth_n*2+2)
        else:
            self.input_data = [0.0]*(self.input_depth_n+2)

        self.goal_point = np.array([0.0,0.0])
        self.range_max = 10.0
        self.yolo_switch = False

    def callback_scan(self, data):
        ranges = np.array(data.ranges)
        ranges = np.where(np.isnan(ranges),data.range_min,ranges)
        ranges = np.where(ranges>self.range_max, self.range_max, ranges)
        interval = int(len(data.ranges)/(self.input_depth_n-1))
        centor = int(len(data.ranges)/2)
        for i in range(self.input_depth_n):#np.arrange(data.angle_min,data.angle_max,angle_interval):
            if i < self.input_depth_n/2.0:
                self.input_data[i] = ranges[centor + interval*i - 1]/self.range_max
            else:
                self.input_data[i] = ranges[interval * int(i-self.input_depth_n/2.0)]/self.range_max

    def callback_odom(self, data):
        odom_x = data.pose.pose.position.x
        odom_y = data.pose.pose.position.y
        odom_a = data.pose.pose.orientation.z * (math.pi/2)

        goal_point = self.goal_point
        odom_point = np.array([odom_x, odom_y])
        relative = goal_point - odom_point

        t_theta = np.angle(complex(relative[0], relative[1]))
        if t_theta < 0:
            t_theta = math.pi + (math.pi+t_theta)
        theta = t_theta
        if odom_a < math.pi:
            if t_theta > odom_a+math.pi:
                theta=t_theta-2*math.pi
        if odom_a > math.pi:
            if 0 < t_theta < odom_a-math.pi:
                theta=t_theta+2*math.pi
        goal_angle = theta-odom_a

        self.input_data[-2] = np.linalg.norm(relative)/self.max_goal_dist
        self.input_data[-1] = goal_angle/math.pi

    def callback_camera_info(self, data):
        self.img_width = data.width
        if self.human_detection:
            self.yolo_switch = True

    def callback_yolo(self, data):
        if self.yolo_switch and self.human_detection:
            human_data = np.array([self.range_max]*self.input_depth_n)
            for box in data.bounding_boxes:
                box_xmin = float(box.xmin)
                box_xmax = float(box.xmax)
                box_ymin = float(box.ymin)
                box_ymax = float(box.ymax)
                probability = box.probability
                if box.Class == "person":
                    for i in range(self.input_depth_n):
                        if i == 0:
                            if box_xmin < self.img_width-self.img_width/(self.input_depth_n-1)*i-1 < box_xmax:
                                human_data[i] = self.input_data[i]
                        else:
                            if box_xmin < self.img_width-self.img_width/(self.input_depth_n-1)*i < box_xmax:
                                human_data[i] = self.input_data[i]
            self.input_data[self.input_depth_n:self.input_depth_n*2] = human_data

    def callback_click(self, data):
        self.goal_point = np.array([data.pose.position.x,data.pose.position.y])
        print("=====publish_goal=====")
        print(list(self.goal_point))
        print("======================")

    def input_publish(self):
        rospy.init_node('input_publisher', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        rospy.Subscriber(self.camera_info, CameraInfo, self.callback_camera_info)
        if self.visual_odom:
            rospy.Subscriber("/rtabmap/odom",Odometry , self.callback_odom)
        else:
            rospy.Subscriber("/odom",Odometry , self.callback_odom)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback_click)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.callback_yolo)

        publisher = rospy.Publisher('/observe', Float32MultiArray, queue_size=10)
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            array_for_Publish = Float32MultiArray(data = self.input_data)
            publisher.publish(array_for_Publish)
            r.sleep()
        
        rospy.spin()

if __name__ == '__main__':
    publisher = input_publisher()
    publisher.input_publish()

