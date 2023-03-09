# import cv2
import numpy as np
import open3d as o3d
# import ros_numpy
from utils import mask_grid

import cv2
from cv_bridge import CvBridge
import rospy
import tf

from geometry_msgs.msg import Pose, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
# import ros_numpy

from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as point_cloud2
import tf2_ros
import tf2_py as tf2
from sensor_msgs.msg import PointCloud2, PointField
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

def create_pc_from_points(points, id):
	fields = [PointField('x', 0, PointField.FLOAT32, 1),
			  PointField('y', 4, PointField.FLOAT32, 1),
			  PointField('z', 8, PointField.FLOAT32, 1)]

	header = Header()
	header.frame_id = id
	header.stamp = rospy.Time.now()

	pc2 = point_cloud2.create_cloud(header, fields, points)
	
	return pc2

class BlockDetectors(object):
	"""docstring for BlockDetector"""
	def __init__(self, block_colors=['r', 'g', 'b', 'y']):
		
		# ros params
		self.robot_type = rospy.get_param('/robot_type')

		if self.robot_type == "sim":
			self.frame_id = "locobot/odom"
		elif self.robot_type == "physical":
			self.frame_id = "map"
	
		self.bridge = CvBridge()	
	
		self.camera_cube_locator_markers_r = rospy.Publisher("/locobot/blocks/visual_r", MarkerArray, queue_size=1)
		self.camera_cube_locator_markers_g = rospy.Publisher("/locobot/blocks/visual_g", MarkerArray, queue_size=1)
		self.camera_cube_locator_markers_b = rospy.Publisher("/locobot/blocks/visual_b", MarkerArray, queue_size=1)
		self.camera_cube_locator_markers_y = rospy.Publisher("/locobot/blocks/visual_y", MarkerArray, queue_size=1)
		self.imgs_with_blocks_bb = rospy.Publisher("/locobot/blocks/bounding_box", Image, queue_size=1, latch=True)

		self.filt_img_blocks = rospy.Publisher("/locobot/blocks/filter_img", Image, queue_size=1, latch=True)

		# self.bridge.cv2_to_imgmsg(mask_img, "rgb8")

		self.block_colors = block_colors

		self.block_poses = {
			'r': MarkerArray(),
			'g': MarkerArray(),
			'b': MarkerArray(),
			'y': MarkerArray()
		}
		# create a tf listener
		self.tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tf_buffer)

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
		if self.robot_type == "sim":
			self.depth_img = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width, -1)
		else:
			self.depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
			self.depth_img = self.depth_img / 1000

	def camera_cube_locator_marker_gen(self, block_positions, color='r', header=None):
		block_markers = MarkerArray()

		marks = []
		for ind, block_xyz in enumerate(block_positions):		
			point_3d_geom_msg = PointStamped()
			point_3d_geom_msg.header = self.frame_id
			point_3d_geom_msg.point.x = block_xyz[0]
			point_3d_geom_msg.point.y = block_xyz[1]
			point_3d_geom_msg.point.z = block_xyz[2]
			rotated_points = point_3d_geom_msg # self.listener.transformPoint(self.frame_id, point_3d_geom_msg)

			if abs(rotated_points.point.z) > 0.1 :
				continue

			#this is very simple because we are just putting the point P in the base_link frame (it is static in this frame)
			marker = Marker()
			marker.header.frame_id = self.frame_id
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

		# Colors
		rgb = np.zeros(self.color_img.shape)

		rgb[..., 0] = self.color_img[..., 0]
		rgb[..., 1] = self.color_img[..., 1]
		rgb[..., 2] = self.color_img[..., 2]

		mask_full = np.zeros((height, width), dtype=bool)

		for c in self.block_colors:
			# Mask out the image, leaving only the salient points in the camera frame that correspond to the right block color
			masked_img, masked_xyz, mask = mask_grid(rgb.astype(np.uint8), depth_image, meshgrid, self.camera_model, color_mask=c)
			data = np.concatenate([masked_xyz, masked_img], axis=-1)
			data[:, 3:] = data[:, 3:]/255.

			# Compose masks
			mask_full = np.logical_or(mask_full, np.array(mask))

			# remove any spurious data
			data = data[~np.isnan(data).any(axis=-1)]

			if len(data) == 0:
				continue

			# Transform point cloud from camera frame into world frame
			transform = self.tf_buffer.lookup_transform(self.frame_id, self.depth_header.frame_id, rospy.Time(), rospy.Duration(3.0))
			pcd = create_pc_from_points(data[:, :3], self.depth_header.frame_id)
			pcd_world = do_transform_cloud(pcd, transform)
			pcd_world = np.array(list(point_cloud2.read_points(pcd_world, skip_nans=True, field_names = ("x", "y", "z"))))

			points = pcd_world

			# Rotated colored point cloud
			data[:, :3] = points

			if len(data) > 0:

				# Create the Open3D Point Cloud
				pc = o3d.geometry.PointCloud()

				pc.points = o3d.utility.Vector3dVector(data[:, :3])
				pc.colors = o3d.utility.Vector3dVector(data[:, 3:])

				# Cluster the blocks if more than one block in image
				pc_ind = np.array(pc.cluster_dbscan(.1, 5))
				def partition(array):
					return {i: (array == i).nonzero()[0] for i in np.unique(array)}
				
				pc_ind_dict = partition(pc_ind)
				
				block_centers = []
				for key, value in pc_ind_dict.items():
					# If the cluster is not an outlier
					if key != -1:
						pc_new = pc.select_by_index(value)

						# Interpolate from these points down to the floor to get the rough
						# point cloud of the whole block
						pc_pts = np.array(pc_new.points)

						# Interpolate in the z-direction (i.e. down to the floor) for K points
						pc_pts_interp_z = np.linspace(pc_pts[:, 2], np.zeros(pc_pts[:, 2].shape), 3).transpose(1, 0)
						
						# Create copies of the xy coordinates
						pc_pts_interp_xy = np.stack(3*[pc_pts[:, :2]], axis=-1)

						# Concatenate the two arrays together -> N x 3 x K
						pc_pts_interp_xyz = np.concatenate([pc_pts_interp_xy, pc_pts_interp_z[:, None, :]], axis=1)
						
						# N x K x 3 -> N*K x 3
						pc_pts_inter = pc_pts_interp_xyz.transpose(0, 2, 1).reshape(-1, 3)
						
						# Create another point cloud
						pcd_inter = o3d.geometry.PointCloud()
						pcd_inter.points = o3d.utility.Vector3dVector(pc_pts_inter[:, :3])
						# pc, indexes = pc.remove_statistical_outlier(10, .1)

						# Retrieve oriented bounding box and find the center
						# try: 
						# 	BB = pcd_inter.get_oriented_bounding_box()
						# 	bounding_box_center = np.array(BB.center)

						# 	box_pts = np.array(BB.get_box_points())
						
						# 	if bounding_box_center[-1] < 0.02 and BB.volume() < 0.03**2 and BB.volume() > 0.01**2: 	# Prevents finding spurious points
						# 		center = bounding_box_center
						# 		block_centers.append(center)
						# except:
						# 	continue

						# block_centers.append(np.mean(np.array(pcd_inter.points),axis=0))

						block_var = np.var(np.array(pcd_inter.points), axis=0)

						if not np.any(block_var > 0.05):
							center = np.mean(np.array(pcd_inter.points), axis=0)
							block_centers.append(center)
						
				block_centers = np.array(block_centers)
				self.camera_cube_locator_marker_gen(block_centers, color=c, header=self.depth_header)
	
		# Publish masked img
		filter_img = cv2.bitwise_and(self.color_img, self.color_img, mask=mask_full.astype(np.uint8))
		filter_img_msg = self.bridge.cv2_to_imgmsg(filter_img, "rgb8")
		self.filt_img_blocks.publish(filter_img_msg)


	def run(self):

		self.info_sub = rospy.Subscriber('/locobot/camera/color/camera_info', CameraInfo, self.info_callback)
		self.color_image_sub = rospy.Subscriber('/locobot/camera/color/image_raw', Image, self.color_image_callback)
		self.depth_image_sub = rospy.Subscriber("/locobot/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)
