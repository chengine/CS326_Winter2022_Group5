#!/usr/bin/env python3


import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Localizer():
    
    def __init__(self):

        # ros params
        self.robot_name = rospy.get_param('/robot_name')
        self.robot_type = rospy.get_param('/robot_type')
        self.frame_offset = {
            'x' : rospy.get_param('/frame_offset/x'),
            'y' : rospy.get_param('/frame_offset/y'),
            'theta' : rospy.get_param('/frame_offset/theta')
        }

        # initialize measurements
        self.pose = Pose()
        self.euler = np.zeros((3,))
        self.vel = Twist()
        self.pose2d = np.zeros((3,))

        self.odom_valid = False
        self.mocap_valid = False

    def is_valid(self):

        return self.odom_valid and (self.mocap_valid or self.robot_type == "sim")
        
    def locobot_odom_callback(self, data):

        self.odom_valid = True
        
        if self.robot_type == "sim":

            self.pose = data.pose.pose

            self.euler = R.from_quat([
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w,
            ]).as_euler('ZYX')

            self.pose2d = np.array([self.pose.position.x, self.pose.position.y, self.euler[0]])

        self.vel = data.twist.twist

    def mocap_callback(self, data):

        self.mocap_valid = True
        
        self.pose = data.pose

        self.euler = R.from_quat([
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        ]).as_euler('ZYX')
        
        self.euler[0] += self.frame_offset['theta']
        self.pose.position.x += self.frame_offset['x'] * np.cos(self.euler[0])
        self.pose.position.y += self.frame_offset['y'] * np.sin(self.euler[0])

        self.pose2d = np.array([self.pose.position.x, self.pose.position.y, self.euler[0]])


    def run(self):
        
        rospy.Subscriber("/locobot/mobile_base/odom", Odometry, self.locobot_odom_callback)
        if self.robot_type == "physical":
            rospy.Subscriber("/vrpn_client_node/" + self.robot_name + "/pose", PoseStamped, self.mocap_callback)
