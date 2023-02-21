#!/usr/bin/env python3
'''
Written by: Monroe Kennedy, Date: 1/2/2023

This script starts at the bottom "if __name__ == '__main__':" which is a function that calls "main():" function.
The main function then instanties a class object (Locobot_example), which takes in a target pose, then listens to the topic 
of the robots odometry (pose), and then calculates the control action to take, then commands the velocity to the robot

This script shows how the robots can go between two points A and B, where A is the initial point (origin), and B is a specified point. 
The robot is first controlled to point B, then is instructed to return to point A.

read more about rospy publishers/subscribers here: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
'''
import rospy
import numpy as np
import scipy.interpolate
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from scipy.spatial.transform import Rotation as R

from ramsete import Ramsete
from pose_controller import PoseController
from trajectory_generator import compute_smoothed_traj, modify_traj_with_limits

CTRL_HZ = 50
CTRL_DT = 1/CTRL_HZ

class LocobotFollowTrajectory(object):
    
    def __init__(self):

        self.robot_name = rospy.get_param('/robot_name')
        self.robot_type = rospy.get_param('/robot_type')
        self.frame_offset = {
            'x' : rospy.get_param('/frame_offset/x'),
            'y' : rospy.get_param('/frame_offset/y'),
            'theta' : rospy.get_param('/frame_offset/theta')
        }

        self.mobile_base_vel_publisher = rospy.Publisher("/locobot/mobile_base/commands/velocity", Twist, queue_size=1) #this is the topic we will publish to in order to move the base

        self.target_pose = Pose()
        self.target_pose_visual = rospy.Publisher("/locobot/mobile_base/target_pose_visual", Marker, queue_size=1)

        # xy waypoints for curved path
        self.path = np.array([
            [0, 0],
            [1, 0],
            [3, 2],
            [5, -1],
        ]) / 2

        # self.path = np.array([
        #     [-0.4, 1.8],
        #     [0, 1.8],
        #     [0.5, 1.9],
        #     [0.6, 1.9]
        # ])

        if self.robot_type == "sim":
            self.frame_id = "locobot/odom"
        elif self.robot_type == "physical":
            self.frame_id = "map"

        self.path_msg = Path(
            header=Header(frame_id=self.frame_id),
            poses=[PoseStamped(pose=Pose(position=Point(x=i[0], y=i[1]))) for i in self.path]
        )
        self.path_visual = rospy.Publisher('/locobot/mobile_base/path_visual', Path, queue_size=1)

        # generate a basic spline trajectory for demonstration

        # control limits
        self.v_max = 0.3 # m/s
        self.w_max = np.pi/4 # rad/s

        # traj gen limits
        self.v_lim = 0.1 # m/s
        self.a_lim = 0.03 # m/s^2
        self.w_lim = np.pi/8 # rad/s

        alpha = 0.5 # smoothing parameter

        traj, traj_ts, = compute_smoothed_traj(self.path, self.v_lim, alpha, CTRL_DT)
        traj_ts, v_goal, w_goal, traj = modify_traj_with_limits(traj, traj_ts, self.v_lim, self.a_lim, self.w_lim, CTRL_DT)

        self.traj = scipy.interpolate.interp1d(traj_ts, traj, axis=0, bounds_error=False, fill_value=(traj[0], traj[-1]))
        self.v_goal = scipy.interpolate.interp1d(traj_ts, v_goal, bounds_error=False, fill_value=0)
        self.w_goal = scipy.interpolate.interp1d(traj_ts, w_goal, bounds_error=False, fill_value=0)
        self.t_final = traj_ts[-1]
        self.x_final = traj[-1, 0:3]
        self.theta_final = traj[-1, 2] + np.pi/2

        self.traj_msg = Path(
            header=Header(frame_id=self.frame_id),
            poses=[PoseStamped(pose=Pose(position=Point(x=(traj_pt := self.traj(t))[0], y=traj_pt[1]))) for t in np.linspace(0, self.t_final, 100)]
        )
        self.traj_visual = rospy.Publisher('/locobot/mobile_base/traj_visual', Path, queue_size=1)

        # set up controllers
        self.ramsete = Ramsete(b=2.0, zeta=0.8)
        self.pose_controller = PoseController(k1=0.4, k2=0.8, k3=0.8)
        self.heading_kP = 0.5
        self.dist_thresh = 0.2
        self.heading_thresh = 0.05

        # initialize measurements
        self.current_pose = Pose()
        self.current_euler = np.zeros(3)
        self.current_vel = Twist()

        self.vel_error = 0
        self.w_error = 0

        self.controller = "ramsete"

        self.last_v = 0
        self.last_w = 0


    def pub_target_point_marker(self):
        #this is putting the marker in the world frame (http://wiki.ros.org/rviz/DisplayTypes/Marker#Points_.28POINTS.3D8.29)
        marker = Marker()
        marker.header.frame_id = self.frame_id #this will be the world frame for the real robot
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.ARROW
        # Set the marker scale
        marker.scale.x = 0.3  # arrow length
        marker.scale.y = 0.1 #arrow width
        marker.scale.z = 0.1 #arrow height

        # Set the marker pose
        marker.pose.position.x = self.target_pose.position.x  # center of the sphere in base_link frame
        marker.pose.position.y = self.target_pose.position.y
        marker.pose.position.z = self.target_pose.position.z
        marker.pose.orientation.x = self.target_pose.orientation.x
        marker.pose.orientation.y = self.target_pose.orientation.y
        marker.pose.orientation.z = self.target_pose.orientation.z
        marker.pose.orientation.w = self.target_pose.orientation.w

        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 0.0 #red
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Publish the marker
        self.target_pose_visual.publish(marker)

    def locobot_odom_callback(self, data):
        
        if self.robot_type == "sim":

            self.current_pose = data.pose.pose

            self.current_euler = R.from_quat([
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w,
            ]).as_euler('ZYX')

        self.current_vel = data.twist.twist

    def mocap_callback(self, data):
        
        self.current_pose = data.pose

        self.current_euler = R.from_quat([
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        ]).as_euler('ZYX')
        
        self.current_euler[0] += self.frame_offset['theta']
        self.current_pose.position.x += self.frame_offset['x'] * np.cos(self.current_euler[0])
        self.current_pose.position.y += self.frame_offset['y'] * np.sin(self.current_euler[0])

    def compute_control(self):

        t = (rospy.Time.now()-self.t_init).to_sec()
        print("time: ", t)

        x_goal = self.traj(t)[0:3]
        v_goal = self.v_goal(t)
        w_goal = self.w_goal(t)

        self.target_pose.position.x, self.target_pose.position.y = x_goal[0:2]
        (
            self.target_pose.orientation.x,
            self.target_pose.orientation.y,
            self.target_pose.orientation.z,
            self.target_pose.orientation.w
        ) = R.from_euler('ZYX', [x_goal[2], 0, 0]).as_quat()

        x = [self.current_pose.position.x, self.current_pose.position.y, self.current_euler[0]]

        self.pub_target_point_marker()

        if self.controller == "ramsete":

            v, w = self.ramsete.calculate(x_goal, v_goal, w_goal, x)
            print('ramsete controller')
            print('x: ', x)
            print('x_goal: ', x_goal)
            print('tracking_error: ', np.linalg.norm(x_goal[0:2]-x[0:2]))

            if np.linalg.norm(self.x_final[0:2]-x[0:2]) < self.dist_thresh or t > self.t_final:
                self.controller = "pose"
            
        elif self.controller == "pose":

            v, w = self.pose_controller.calculate(self.x_final, x)
            print('pose controller')
            print('x: ', x)
            print('x_goal: ', self.x_final)
            print('pose_error: ', np.linalg.norm(self.x_final[0:2]-x[0:2]))

            if np.linalg.norm(self.x_final[0:2]-x[0:2]) < self.heading_thresh:
                self.controller = "heading"
            
        elif self.controller == "heading":

            v, w = (0, self.heading_kP * ((self.theta_final - x[2] + np.pi) % (2*np.pi) - np.pi))
            print('heading controller')
            print('current heading: ', x[2])
            print('goal heading: ', self.theta_final)
            print('heading error: ', self.theta_final-x[2])
            print('pose_error: ', np.linalg.norm(self.x_final[0:2]-x[0:2]))

        # print('v: ', self.current_vel.linear.x)
        # print('w: ', self.current_vel.angular.z)

        print('v_goal: ', v_goal)
        print('w_goal: ', w_goal)

        print('v_cmd: ', v)
        print('w_cmd: ', w)

        # alpha = 0.8

        # self.vel_error = alpha * self.vel_error + (1-alpha)*(v-self.current_vel.linear.x)
        # self.w_error = alpha * self.w_error + (1-alpha)*(w-self.current_vel.angular.z)

        # print('vel_error: ', self.vel_error)
        # print('w_error: ', self.w_error)
        
        control_msg = Twist()
        control_msg.linear.x = np.clip(v, -self.v_max, self.v_max)
        control_msg.angular.z = np.clip(w, -self.w_max, self.w_max)

        # # scale and delay control commands to mimic robot dynamics

        # control_msg.linear.x *= 0.8
        # control_msg.angular.z *= 0.8

        # control_msg.linear.x = 0.99 * self.last_v + (1-0.99) * control_msg.linear.x
        # control_msg.angular.z = 0.5 * self.last_w + (1-0.5) * control_msg.angular.z

        # self.last_v = control_msg.linear.x
        # self.last_w = control_msg.angular.z
        
        # publish control message
        
        self.mobile_base_vel_publisher.publish(control_msg)
        
    def run(self):
        
        rospy.Subscriber("/locobot/mobile_base/odom", Odometry, self.locobot_odom_callback)
        if self.robot_type == "physical":
            rospy.Subscriber("/vrpn_client_node/" + self.robot_name + "/pose", PoseStamped, self.mocap_callback)

        self.t_init = rospy.Time.now()

        rate = rospy.Rate(CTRL_HZ)
        counter = 0

        while not rospy.is_shutdown():
            self.compute_control()

            # publish path at 1Hz
            if counter >= CTRL_HZ:
                counter = 0
            if counter == 0:
                self.path_visual.publish(self.path_msg)
                self.traj_visual.publish(self.traj_msg)
            counter += 1

            rate.sleep()



def main():
    rospy.init_node('locobot_follow_trajectory')
    cls_obj = LocobotFollowTrajectory()
    cls_obj.run()



if __name__ == '__main__':
    main()

