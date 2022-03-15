#!/usr/bin/env python
import rospy
import roslib.packages
import numpy as np
import os
from tensorflow.keras.models import Sequential, Model
from tensorflow.keras.layers import Dense, Activation, Flatten, Input, Concatenate

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class ddpg_navi():
    def __init__(self):
        self.range_min = rospy.get_param("/depthimage_to_laserscan/range_min", 0.1)
        self.input_depth_n = rospy.get_param("/input_publisher/input_depth_n", 9)
        self.human_detection = rospy.get_param("/input_publisher/human_detection", True)
        self.weight_path = rospy.get_param("/ddpg_navi/weight_path", "sample_weight/ddpg_weights_actor.h5f")
        self.action_n = rospy.get_param("/ddpg_navi/action_n", 2)
        if self.human_detection:
            self.input_n = self.input_depth_n*2+2
        else:
            self.input_n = self.input_depth_n+2

    def callback_observe(self,data):
        self.observe_data = np.array(data.data)
        #print(self.observe_data)

    def newral_network(self,nb_actions=None,input_n=None):
        if nb_actions is None:
            nb_actions = self.action_n
        if input_n is None:
            input_n = self.input_n
        self.actor = Sequential()
        self.actor.add(Flatten(input_shape=(1,) + (input_n,)))
        self.actor.add(Dense(256))
        self.actor.add(Activation('relu'))
        self.actor.add(Dense(128))
        self.actor.add(Activation('relu'))
        self.actor.add(Dense(64))
        self.actor.add(Activation('relu'))
        self.actor.add(Dense(nb_actions))
        self.actor.add(Activation('tanh'))
        print(self.actor.summary())

    def load_weight(self,weight_path=None):
        if weight_path is None:
            weight_path = self.weight_path
        pkg_name = 'rl_navigation'
        weight_path = roslib.packages.get_pkg_dir(pkg_name)+"/weight/"+weight_path
        if os.path.exists(weight_path+".index"):
            self.actor.load_weights(weight_path)
            print("########################\n"
            "find weights-file\n"
            "########################")
        else:
            print("########################\n"
            "not found weights-file\n"
            "########################")

    def navigation(self):
        rospy.init_node('ddpg_navi', anonymous=True)
        rospy.Subscriber("/observe", Float32MultiArray, self.callback_observe)

        self.newral_network(2,20) #output,input
        self.load_weight()

        publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        r = rospy.Rate(10) # 5hz

        while not rospy.is_shutdown():
            action = self.actor.predict(self.observe_data.reshape(1,1,len(self.observe_data)))
            predict_action = action.reshape(2)
            pub_vel = Twist()
            if predict_action[0] > 0.8:
                pub_vel.linear.x = 0.8
            elif predict_action[0] < -0.8:
                pub_vel.linear.x = -0.8
            else:
                pub_vel.linear.x = predict_action[0]
            if predict_action[1] > 0.8:
                pub_vel.angular.z = 0.8
            elif predict_action[1] < -0.8:
                pub_vel.angular.z = -0.8
            else:
                pub_vel.angular.z = predict_action[1]
        
            pub_vel.linear.x = pub_vel.linear.x
            pub_vel.angular.z = pub_vel.angular.z
            
            publisher.publish(pub_vel)

        rospy.spin()

if __name__ == '__main__':
    navigator = ddpg_navi()
    navigator.navigation()

