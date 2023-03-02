import rospy
import numpy as np

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

import geometry_msgs
from geometry_msgs.msg import Pose, PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class MoveLocobotArm(object):
	"""docstring for MoveLocobotArm"""
	def __init__(self,moveit_commander=None):
		self.moveit_commander = moveit_commander
		self.robot = self.moveit_commander.RobotCommander() #this needs to be launched in the namespace of the robot (in this example, this is done in the launch file using 'group')
		self.scene = self.moveit_commander.PlanningSceneInterface()
		self.gripper_group_name = "interbotix_gripper"
		self.gripper_move_group = self.moveit_commander.MoveGroupCommander(self.gripper_group_name)

		self.arm_group_name = "interbotix_arm" #interbotix_arm and interbotix_gripper (can see in Rviz)
		self.arm_move_group = self.moveit_commander.MoveGroupCommander(self.arm_group_name)
		self.display_trajectory_publisher = rospy.Publisher('/locobot/move_group/display_planned_path',
													   moveit_msgs.msg.DisplayTrajectory,
													   queue_size=20)
		# We can get the name of the reference frame for this robot:
		self.planning_frame = self.arm_move_group.get_planning_frame()
		# We can also print the name of the end-effector link for this group:
		self.eef_link = self.arm_move_group.get_end_effector_link()
		self.jnt_names = self.arm_move_group.get_active_joints()
		# We can get a list of all the groups in the robot:
		self.group_names = self.robot.get_group_names()


	def display_moveit_info(self):
		# We can get the name of the reference frame for this robot:
		print("============ Planning frame: %s" % self.planning_frame)
		# We can also print the name of the end-effector link for this group:
		print("============ End effector link: %s" % self.eef_link)
		print("============ Armgroup joint names: %s" % self.jnt_names)
		# We can get a list of all the groups in the robot:
		print("============ Available Planning Groups:", self.robot.get_group_names())
		# Sometimes for debugging it is useful to print the entire state of the
		# robot:
		print("============ Printing robot state")
		print(self.robot.get_current_state())
		print("\n")
		
	def close_gripper(self):
		gripper_goal = self.gripper_move_group.get_current_joint_values()
		print("grippers",gripper_goal)
		gripper_goal[0] = 0.037
		gripper_goal[1] = -0.037
		self.gripper_move_group.go(gripper_goal, wait=True)

	def open_gripper(self):
		gripper_goal = self.gripper_move_group.get_current_joint_values()
		print("grippers",gripper_goal)
		gripper_goal[0] = -0.037
		gripper_goal[1] = 0.037
		self.gripper_move_group.go(gripper_goal, wait=True)

	def move_arm_down_for_camera(self):
		#start here
		joint_goal = self.arm_move_group.get_current_joint_values() 
		#['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
		joint_goal[0] = -0.1115207331248822 #waist
		joint_goal[1] = -0.5313552376357276 #shoulder
		joint_goal[2] = 1.058371284458718 #elbow
		joint_goal[3] = -0.05608022936825474 #forearm_roll
		joint_goal[4] = 0.9302728070281328 #wrist_angle
		joint_goal[5] = -0.14247350829385486 #wrist_rotate

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		self.arm_move_group.go(joint_goal, wait=True)	

	def move_gripper_down_to_grasp(self, position=(0.5, 0., 0.03)):
		pose_goal = geometry_msgs.msg.Pose()

		pose_goal.position.x = position[0]
		pose_goal.position.y = position[1]
		pose_goal.position.z = 0.03

		v = np.matrix([0,1,0]) #pitch about y-axis
		th = 10*np.pi/180. #pitch by 45deg
		#note that no rotation is th= 0 deg

		pose_goal.orientation.x = v.item(0)*np.sin(th/2)
		pose_goal.orientation.y = v.item(1)*np.sin(th/2)
		pose_goal.orientation.z = v.item(2)*np.sin(th/2)
		pose_goal.orientation.w = np.cos(th/2)

		try:
			# self.arm_move_group.set_position_target(position)
			self.arm_move_group.set_pose_target(pose_goal)
			# now we call the planner to compute and execute the plan
			plan = self.arm_move_group.go(wait=True)
			# Calling `stop()` ensures that there is no residual movement
			self.arm_move_group.stop()
			# It is always good to clear your targets after planning with poses.
			# Note: there is no equivalent function for clear_joint_value_targets()
			self.arm_move_group.clear_pose_targets()
		except: 
			print('Not feasible goal')