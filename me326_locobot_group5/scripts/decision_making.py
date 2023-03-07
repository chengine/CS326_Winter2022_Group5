#!/usr/bin/env python3

import sys
import yaml

import rospy
import numpy as np
import scipy
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

from me326_locobot_group5.srv import BlockDetector, BlockDetectorResponse
from detect_block_node import detect_blocks_client

class PickNPlace():
	def __init__(self, filepath, robot_id, detect_blocks_fn, camera_orient_obj, moveit_arm_obj) -> None:
		self.filepath = filepath
		self.robot_id = robot_id
		self.block_detect_fn = detect_blocks_fn

		self.camera_orient_obj = camera_orient_obj
		self.moveit_arm_obj = move_arm_obj

		self.possible_colors = ['r', 'g', 'b', 'y']

		# TODO: subscriber to odom to get current pose
		# self.current_robo_pos = ...

		odom_topic = 'locobot/odom'
		self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)


		# Initial steps
		self.read_configs()

		# self.initial_scan()
	def read_configs(self):
		with open(self.filepath, 'r') as stream:
			data = yaml.safe_load(stream)

		self.station_loc = np.array(data['station_locations'])

		self.target_config = data['target_config']

		robot_1_colors = data['robot_1_colors']

		robot_2_colors = data['robot_2_colors']

		self.colors = []
		for i in range(4):
			if self.robot_id == 1:
				c = robot_1_colors[i][0]
			else:
				c = robot_2_colors[i][0]
			if c == 1:
				self.colors.append(self.possible_colors[i])

		print(self.colors, self.station_loc)
	def initial_scan(self):
		self.blocks = {
			'r': None,
			'g': None,
			'b': None,
			'y': None
		}

		# Tilt camera to see most of the scene in front
		camera_orient_obj.tilt_camera(0.8)

		# TODO: What to set pan at to center?

		for i in range(12):
			# Update the map of block positions with current image
			self.update_block_map(self.possible_colors)

			# Rotate x degrees
			self.spin(30)

	def update_block_map(self, colors):
		for col in colors:
			color = String()
			color.data = col

			# Returns the poses of all blocks of the color
			block_poses = detect_blocks_client(color)

			blocks_poses = []

			# Consolidates the poses into an array
			for marker in block_poses.block_poses.markers:
				block_pose = [marker.pose.position.x,
					marker.pose.position.y, 
					marker.pose.position.z]
				blocks_poses.append(block_pose)
			blocks_poses = np.array(blocks_poses)

			current_positions = self.blocks[col] 

			if current_positions is not None or len(current_positions) > 0:
				self.blocks[col] = self.update_positions(current_positions, blocks_poses)
			else:
				self.blocks[col] = blocks_poses

	def update_positions(self, data, new_data):
		# Computes the euclidian distance between the data and the new data and 
		# sees which ones are redundant

		distance_matrix = scipy.spatial.distance_matrix(data, new_data)

		# If a new data point has no matches (i.e. the column corresponding to it has no hits), we
		# treat that as a new block
		binary_mask = (distance_matrix < 0.2)

		# Now sum up the mask column wise
		hits = np.sum(binary_mask, axis=0)

		new_data_indices = np.argwhere(hits == 0)

		new_data_to_append = new_data[new_data_indices]

		return np.concatenate([data, new_data_to_append], axis=0)

	def go_to_nearest_block(self, color):
		block_positions = self.blocks[color]
		distance_matrix = scipy.spatial.distance_matrix(block_positions, self.current_robo_pos)
		closest_blck_ind = np.argmin(distance_matrix)

		closest_blck_pos = block_positions[closest_blck_ind]

		# Go to goal that is 0.5 meters away from the target
		dist_away = 0.5
		desired_pos = (1 - dist_away/np.linalg.norm(closest_blck_pos - self.current_robo_pos))*(closest_blck_pos - self.current_robo_pos) + self.current_robo_pos

		# TODO: Go to that desired position

		# TODO: Pick up the block
		# TODO: Get the pose of the block in the base linke frame!
		block_pose = ...
		move_arm_obj.open_gripper()
		move_arm_obj.move_gripper_down_to_grasp(block_pose)
		move_arm_obj.close_gripper()
		move_arm_obj.move_arm_to_home()

	def go_to_station(self):
		desired_pos = self.current_station_pos

		# TODO: Go to that desired position

	def delete_entry(self, data, index):
		return np.delete(data, index, 0)

	def spin(self, angle=360):
		pass

	def odom_callback(self, data):
		return np.array(data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)

if __name__ == "__main__":
	rospy.init_node('decision_making')

	### PARAMS
	robot_id = 1	# Must be either 1 or 2
	filepath = 'resource_gathering.yaml'

	### End of PARAMS

	moveit_commander.roscpp_initialize(sys.argv)

	# Arm object
	# move_arm_obj = MoveLocobotArm(moveit_commander=moveit_commander)
	# move_arm_obj.display_moveit_info()
	# move_arm_obj.move_arm_down_for_camera()

	move_arm_obj = None

	# Camera orientation object
	# camera_orient_obj = OrientCamera()
	camera_orient_obj = None


	PickNPlace(filepath, robot_id, detect_blocks_client, camera_orient_obj, move_arm_obj)
	rospy.spin()
