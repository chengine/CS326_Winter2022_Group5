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

from perception.detector_classes import BlockDetectors
from me326_locobot_example.srv import BlockDetector, BlockDetectorResponse

def block_detect_server():
	rospy.init_node('block_detector_service')
	block_detectors = BlockDetectors()

	s = rospy.Service('block_detector', BlockDetector, block_detectors.service_callback)
	print("Ready to detect blocks.")
	rospy.spin()

def detect_blocks_client(color):
	rospy.wait_for_service('block_detector')
	try:
		detect_blocks = rospy.ServiceProxy('block_detector', BlockDetector)
		resp = detect_blocks(color)

		tnow = rospy.Time.now()

		elapsed = 0

		# TODO: Add a timeout here.
		while len(resp.block_poses.markers) == 0 and elapsed < 3.:
			resp = detect_blocks(color)
			elapsed = (rospy.Time.now() - tnow).to_sec()
		return resp
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)
		return -1

if __name__ == "__main__":
	block_detect_server()
