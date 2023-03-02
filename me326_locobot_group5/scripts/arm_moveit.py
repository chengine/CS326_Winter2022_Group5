#!/usr/bin/env python3

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
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import String

from arm_gripper.arm_classes import MoveLocobotArm
from arm_gripper.cam_orient import OrientCamera

from me326_locobot_example.srv import BlockDetector, BlockDetectorResponse

def detect_blocks_client(color):
	rospy.wait_for_service('block_detector')
	try:
		detect_blocks = rospy.ServiceProxy('block_detector', BlockDetector)
		resp = detect_blocks(color)

		# TODO: Add a timeout here.
		while len(resp.block_poses.markers) == 0:
			resp = detect_blocks(color)
		return resp
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

if __name__ == "__main__":
	rospy.init_node('moveit_arm')

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

	color = String()
	color.data = 'r'

	block_poses = detect_blocks_client(color)

	first_block_pose = (block_poses.block_poses.markers[0].pose.position.x,
		    block_poses.block_poses.markers[0].pose.position.y, 
		    block_poses.block_poses.markers[0].pose.position.z)
	
	#Uncomment below to move gripper down for grasping (note the frame is baselink; z=0 is the ground (hitting the ground!))
	# move_arm_obj.open_gripper()
	move_arm_obj.move_gripper_down_to_grasp(first_block_pose)
	# move_arm_obj.close_gripper()
	# move_arm_obj.open_gripper()

	rospy.spin()
