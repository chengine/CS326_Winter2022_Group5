# import cv2
import numpy as np
import open3d as o3d
# import ros_numpy
from perception.clustering import mask_grid

from cv_bridge import CvBridge
import rospy
import tf

from geometry_msgs.msg import Pose, PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from image_geometry import PinholeCameraModel


class BlockDetectors(object):
	"""docstring for BlockDetector"""
	def __init__(self, block_transforms = "/locobot/blocks/tf", block_colors=['r', 'g', 'b', 'y']):
		self.bridge = CvBridge()	
	
		self.block_tf_pub = rospy.Publisher(block_transforms, Pose, queue_size=1)

		self.camera_cube_locator_markers_r = rospy.Publisher("/locobot/blocks/visual_r", MarkerArray, queue_size=1)
		self.camera_cube_locator_markers_g = rospy.Publisher("/locobot/blocks/visual_g", MarkerArray, queue_size=1)
		self.camera_cube_locator_markers_b = rospy.Publisher("/locobot/blocks/visual_b", MarkerArray, queue_size=1)
		self.camera_cube_locator_markers_y = rospy.Publisher("/locobot/blocks/visual_y", MarkerArray, queue_size=1)

		self.block_colors = block_colors

		self.block_poses = {
			'r': MarkerArray(),
			'g': MarkerArray(),
			'b': MarkerArray(),
			'y': MarkerArray()
		}
		# create a tf listener
		self.listener = tf.TransformListener()

		self.camera_model = None
		self.color_img = None
		self.depth_img = None
		self.depth_header = None
	
	def ready(self):

		return self.camera_model is not None and self.color_img is not None and self.depth_img is not None
	
	def info_callback(self, info_msg):
		# create a camera model from the camera info
		self.camera_model = PinholeCameraModel()
		self.camera_model.fromCameraInfo(info_msg)
	
	def color_image_callback(self,color_msg):
		#double check img
		self.color_img = self.bridge.imgmsg_to_cv2(color_msg, "rgb8")

	def depth_image_callback(self,depth_msg):
		# Data is PointCloud2 msg
		self.depth_header = depth_msg.header
		self.depth_img = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width, -1)

	def camera_cube_locator_marker_gen(self, block_positions, color='r', header=None):
		block_markers = MarkerArray()

		marks = []
		for ind, block_xyz in enumerate(block_positions):		
			point_3d_geom_msg = PointStamped()
			point_3d_geom_msg.header = header
			point_3d_geom_msg.point.x = block_xyz[0]
			point_3d_geom_msg.point.y = block_xyz[1]
			point_3d_geom_msg.point.z = block_xyz[2]
			rotated_points = self.listener.transformPoint('locobot/base_footprint', point_3d_geom_msg)

			if abs(rotated_points.point.z) > 0.1 :
				continue

			#this is very simple because we are just putting the point P in the base_link frame (it is static in this frame)
			marker = Marker()
			marker.header.frame_id = "locobot/base_footprint"
			marker.header.stamp = rospy.Time.now()
			marker.id = ind
			marker.type = Marker.SPHERE
			# Set the marker scale
			marker.scale.x = 0.05  # radius of the sphere
			marker.scale.y = 0.05
			marker.scale.z = 0.05

			# Set the marker pose
			marker.pose.position.x = rotated_points.point.x
			marker.pose.position.y = rotated_points.point.y
			marker.pose.position.z = 0.03 #rotated_points.point.z

			# Set the marker color
			marker.color.a = 1.0 #transparency
			if color == 'r':
				marker_color = (1.0, 0.0, 0.0)
			elif color == 'g':
				marker_color = (0.0, 1.0, 0.0)
			elif color == 'b':
				marker_color = (0.0, 0.0, 1.0)
			else:
				marker_color = (1.0, 1.0, 0.0)

			marker.color.r = marker_color[0]
			marker.color.g = marker_color[1]
			marker.color.b = marker_color[2]
			marks.append(marker)

		# Publish the marker
		block_markers.markers = marks
		if color == 'r':
			self.camera_cube_locator_markers_r.publish(block_markers)
			self.block_poses['r'] = block_markers

		if color == 'g':
			self.camera_cube_locator_markers_g.publish(block_markers)
			self.block_poses['g'] = block_markers

		if color == 'b':
			self.camera_cube_locator_markers_b.publish(block_markers)
			self.block_poses['b'] = block_markers

		else:
			self.camera_cube_locator_markers_y.publish(block_markers)
			self.block_poses['y'] = block_markers


	def calculate_tf(self):
		# Data is PointCloud2 msg

		if not self.ready():
			print("detector not ready")
			return

		depth_image = self.depth_img
		height, width = depth_image.shape[0], depth_image.shape[1]
		Y, X = np.meshgrid(np.arange(width), np.arange(height))
		meshgrid = np.stack([Y, X], axis=-1)

		# pc=ros_numpy.numpify(data)
		#pc=ros_numpy.point_cloud2.split_rgb_field(pc)
		#shape = pc.shape + (3, )

		# Coordinates
		# depth_image = np.zeros(self.color_img.shape) 
		# depth_image[..., 0] = pc['x']
		# depth_image[..., 1] = pc['y']
		# depth_image[..., 2] = pc['z']

		# Colors
		rgb = np.zeros(self.color_img.shape)

		rgb[..., 0] = self.color_img[..., 0]
		rgb[..., 1] = self.color_img[..., 1]
		rgb[..., 2] = self.color_img[..., 2]

		# cluster_data = np.concatenate([points, rgb], axis=-1)
		# cluster_data = cluster_data.reshape((-1, 6))
		# cluster_data = cluster_data[~np.isnan(cluster_data).any(axis=-1)]
		# cluster_data = cluster_data.reshape((1, -1, 6))

		# TODO: Seperate based on non-pertinent colors
		for c in self.block_colors:
			masked_img, masked_xyz = mask_grid(rgb.astype(np.uint8), depth_image, meshgrid, self.camera_model, color_mask=c)
			data = np.concatenate([masked_xyz, masked_img], axis=-1)
			data[:, 3:] = data[:, 3:]/255.

			data = data[~np.isnan(data).any(axis=-1)]

			if len(data) > 0:
				pc = o3d.geometry.PointCloud()

				pc.points = o3d.utility.Vector3dVector(data[:, :3])
				pc.colors = o3d.utility.Vector3dVector(data[:, 3:])

				pc_ind = np.array(pc.cluster_dbscan(.1, 5))
				def partition(array):
					return {i: (array == i).nonzero()[0] for i in np.unique(array)}
				
				pc_ind_dict = partition(pc_ind)
				
				block_centers = []
				for key, value in pc_ind_dict.items():
					if key != -1:
						pc_new = pc.select_by_index(value)

						# pc, indexes = pc.remove_statistical_outlier(10, .1)
						# BB = pc_new.get_oriented_bounding_box()

						# bounding_box_center = np.array(BB.center)

						# if bounding_box_center[-1] > 0.5 and BB.volume() < 0.4**2 and BB.volume() > 0.05**2: 	# Prevents finding spurious points
						# 	print(c, BB.volume())

						block_var = np.var(np.array(pc_new.points), axis=0)

						if not np.any(block_var > 0.2):
							center = np.mean(np.array(pc_new.points), axis=0)
							block_centers.append(center)
				block_centers = np.array(block_centers)
				self.camera_cube_locator_marker_gen(block_centers, color=c, header=self.depth_header)
	
	def run(self):

		self.info_sub = rospy.Subscriber('/locobot/camera/color/camera_info', CameraInfo, self.info_callback)
		self.color_image_sub = rospy.Subscriber('/locobot/camera/color/image_raw', Image, self.color_image_callback)
		self.depth_image_sub = rospy.Subscriber("/locobot/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

