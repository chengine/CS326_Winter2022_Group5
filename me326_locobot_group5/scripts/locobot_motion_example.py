#!/usr/bin/env python3
'''
Written by: Monroe Kennedy, Date: 1/2/2023
Docs: http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html

Example of using moveit for grasping example
'''

import sys

import rospy
import numpy as np
import scipy as sp
from scipy import linalg

import moveit_commander

import geometry_msgs
from geometry_msgs.msg import Pose, PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from perception.detector_classes import BlockDetector
from arm_gripper.arm_classes import MoveLocobotArm
from arm_gripper.cam_orient import OrientCamera

def main():

	rospy.init_node('locobot_motion_example')

	moveit_commander.roscpp_initialize(sys.argv)
	move_arm_obj = MoveLocobotArm(moveit_commander=moveit_commander)
	move_arm_obj.display_moveit_info()
	move_arm_obj.move_arm_down_for_camera()
	# move_arm_obj.move_gripper_down_to_grasp()
	# move_arm_obj.open_gripper()
	# move_arm_obj.close_gripper()

	# Point the camera toward the blocks
	camera_orient_obj = OrientCamera()
	camera_orient_obj.tilt_camera(0.9)

	block_detector = BlockDetector()
	block_detector.publish_block_tf()
	position = block_detector.get_block_locations()

	#Uncomment below to move gripper down for grasping (note the frame is baselink; z=0 is the ground (hitting the ground!))
	# move_arm_obj.open_gripper()
	move_arm_obj.move_gripper_down_to_grasp(position)
	# move_arm_obj.close_gripper()
	# move_arm_obj.open_gripper()

	rospy.spin()


if __name__ == '__main__':
	main()
