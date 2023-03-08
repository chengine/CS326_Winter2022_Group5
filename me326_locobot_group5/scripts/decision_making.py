#!/usr/bin/env python3

import sys

import rospy
import numpy as np
import scipy

import moveit_commander

from arm_gripper.arm_classes import MoveLocobotArm
from arm_gripper.cam_orient import OrientCamera

from perception.detector_classes import BlockDetectors
from motion_planner import MotionPlanner, MotionPlannerState
from localizer import Localizer

SLEEP_DT = 1

class PickNPlace():
	def __init__(self, camera_orient_obj, moveit_arm_obj) -> None:

		self.detector = BlockDetectors()

		self.camera_orient_obj = camera_orient_obj
		self.moveit_arm_obj = move_arm_obj

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

	def run(self):
		
		self.localizer.run()
		self.planner.run()
		self.detector.run()

		self.wait_for_detector()
		self.initial_scan()
		self.go_to_nearest_block(self.colors)

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
		camera_orient_obj.tilt_camera(0.8)
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

	def go_to_nearest_block(self, colors):
		
		self.planner.go_to_nearest_block(colors)
		self.wait_for_motion()

		# TODO: Pick up the block
		# TODO: Get the pose of the block in the base linke frame!
		# block_pose = ...
		# move_arm_obj.open_gripper()
		# move_arm_obj.move_gripper_down_to_grasp(block_pose)
		# move_arm_obj.close_gripper()
		# move_arm_obj.move_arm_to_home()

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

	moveit_commander.roscpp_initialize(sys.argv)

	# Arm object
	move_arm_obj = MoveLocobotArm(moveit_commander=moveit_commander)
	move_arm_obj.display_moveit_info()
	move_arm_obj.move_arm_down_for_camera()

	# move_arm_obj = None

	# Camera orientation object
	camera_orient_obj = OrientCamera()
	# camera_orient_obj = None

	picknplace = PickNPlace(camera_orient_obj, move_arm_obj)
	picknplace.run()

	rospy.spin()
