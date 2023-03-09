#!/usr/bin/env python3

import sys

import rospy
import numpy as np
import scipy

import moveit_commander

from cam_orient import OrientCamera
from me326_locobot_group5.srv import *

from detector import BlockDetectors
from arm_gripper import MoveLocobotArm
from motion_planner import MotionPlanner, MotionPlannerState
from localizer import Localizer
import tf
from geometry_msgs.msg import PointStamped, Pose

SLEEP_DT = 1

class PickNPlace():
	def __init__(self) -> None:

		# ros params
		self.robot_type = rospy.get_param('/robot_type')

		if self.robot_type == "sim":
			self.frame_id = "locobot/odom"
		elif self.robot_type == "physical":
			self.frame_id = "map"

		self.detector = BlockDetectors()

		self.cam_orient_obj = OrientCamera()
		# self.move_to_grasp_service = rospy.ServiceProxy('/arm_gripper/grab', GrabBlock)
		# self.arm_modes_service = rospy.ServiceProxy('/arm_gripper/modes', Modes)

		self.possible_colors = ['r', 'g', 'b', 'y']

		self.robot_id = rospy.get_param('/robot_id')

		self.blocks = {
			'r': None,
			'g': None,
			'b': None,
			'y': None
		}

		self.localizer = Localizer()
		self.planner = MotionPlanner(self.localizer, self.blocks)

		# Initial steps
		self.read_configs()

		# create a tf listener
		self.listener = tf.TransformListener()
	def run(self):
		
		self.localizer.run()
		self.planner.run()
		self.detector.run()

		# self.arm_modes_service("Down")
		self.wait_for_detector()
		self.initial_scan()
		self.go_to_nearest_block_and_grasp(self.colors)

	def read_configs(self):

		self.station_loc = rospy.get_param('/station_locations')

		self.target_config = rospy.get_param('/target_config')

		robot_1_colors = rospy.get_param('/robot_1_colors')

		robot_2_colors = rospy.get_param('/robot_2_colors')

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
		
		# Tilt camera to see most of the scene in front
		self.cam_orient_obj.tilt_camera(0.6)
		rospy.sleep(SLEEP_DT)

		# TODO: What to set pan at to center?

		current_heading = self.localizer.pose2d[2]
		increments = 3 # TODO: increase to 4 or 6

		for i in range(increments):
			# Update the map of block positions with current image
			self.update_block_map(self.possible_colors)

			# Rotate x degrees
			self.spin(current_heading + (2*np.pi)/increments * (i+1))

	def update_block_map(self, colors):

		self.detector.calculate_tf()

		for col in colors:

			# Returns the poses of all blocks of the color
			block_poses = self.detector.block_poses[col]

			blocks_poses = []

			# Consolidates the poses into an array
			for marker in block_poses.markers:
				block_pose = [marker.pose.position.x,
					marker.pose.position.y, 
					marker.pose.position.z]
				blocks_poses.append(block_pose)
			blocks_poses = np.array(blocks_poses)

			current_positions = self.blocks[col] 

			if current_positions is None or len(current_positions) == 0:
				self.blocks[col] = blocks_poses
			else:
				self.blocks[col] = self.update_positions(current_positions, blocks_poses)

		print("block map: ", self.blocks)

	def update_positions(self, data, new_data):
		# Computes the euclidian distance between the data and the new data and 
		# sees which ones are redundant

		if len(new_data) == 0:
			return data
		
		distance_matrix = scipy.spatial.distance_matrix(data, new_data)

		# If a new data point has no matches (i.e. the column corresponding to it has no hits), we
		# treat that as a new block
		binary_mask = (distance_matrix < 0.2)

		# Now sum up the mask column wise
		hits = np.sum(binary_mask, axis=0)

		new_data_indices = np.argwhere(hits == 0)

		new_data_to_append = new_data[new_data_indices]

		if new_data_to_append.size > 0:
			return np.concatenate([data, np.atleast_2d(new_data_to_append)], axis=0)
		else:
			return data

	def go_to_nearest_block_and_grasp(self, colors):
		
		self.planner.go_to_nearest_block(colors)
		self.wait_for_motion()

		# self.update_block_map(self.possible_colors)

		# TODO: Pick up the block
		# TODO: Get the pose of the block in the base link frame!
		# blocks_of_interest = [b for c in colors for b in self.blocks[c]]
		# nearest_block = min(blocks_of_interest, key=lambda b: (self.localizer.pose2d[0]-b[0])**2+(self.localizer.pose2d[1]-b[1])**2)

		# point_3d_geom_msg = PointStamped()
		# point_3d_geom_msg.header.frame_id = self.frame_id
		# point_3d_geom_msg.point.x = nearest_block[0]
		# point_3d_geom_msg.point.y = nearest_block[1]
		# point_3d_geom_msg.point.z = 0.03
		# block_in_base_link_frame = self.listener.transformPoint('locobot/base_link', point_3d_geom_msg)

		# block_pose = Pose()
		# block_pose.position.x = block_in_base_link_frame.point.x
		# block_pose.position.y = block_in_base_link_frame.point.y
		# block_pose.position.z = block_in_base_link_frame.point.z

		# self.move_to_grasp_service(block_pose)

	def go_to_station(self, station_idx):

		self.planner.go_to_station(station_idx)
		self.wait_for_motion()

		# TODO: drop the block

	def spin(self, angle):
		
		self.planner.turn_to_heading(angle)
		self.wait_for_motion()

	def wait_for_detector(self):

		print("waiting for detector...")
		while not self.detector.ready():
			rospy.sleep(SLEEP_DT)
		print("detector ready")

	def wait_for_motion(self):

		while self.planner.state != MotionPlannerState.IDLE:
			rospy.sleep(SLEEP_DT)
	


if __name__ == "__main__":
	rospy.init_node('decision_making')

	picknplace = PickNPlace()
	picknplace.run()

	rospy.spin()
