#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from me326_locobot_group5.srv import *
from matplotlib import pyplot as plt

import dijkstra3d

PLAN_HZ = 1
PLAN_DT = 1/PLAN_HZ

class MotionPlanner():
    
    def __init__(self):

        # ros params
        self.robot_name = rospy.get_param('/robot_name')
        self.robot_type = rospy.get_param('/robot_type')
        self.frame_offset = {
            'x' : rospy.get_param('/frame_offset/x'),
            'y' : rospy.get_param('/frame_offset/y'),
            'theta' : rospy.get_param('/frame_offset/theta')
        }
        
        # services
        self.abs_path_service = rospy.ServiceProxy('/traj_follower/abs_path', FollowPath)

        # parameters
        self.x_bounds = [-2, 2]
        self.y_bounds = [-2, 2]
        self.grid_length = 512

        self.grid_x, self.grid_y = np.meshgrid(np.linspace(*self.x_bounds, self.grid_length), np.linspace(*self.y_bounds, self.grid_length))

        # initialize measurements
        self.current_pose = Pose()
        self.current_euler = np.zeros((3,))
        self.current_vel = Twist()
        self.current_x = np.zeros((3,))

        self.blocks = [
            (-1, 0.5, "g"),
            # (0.1, -0.3, "r"),
            # (0.5, 0,5, "b"),
            # (-0.1, -0.1, "y"),
        ]

        self.colors = ["g", "r"]

        self.block_r = 0.03
        self.robot_r = 0.3

    def pose_to_idx(self, x, y):

        return (
            np.round((x-self.x_bounds[0])/(self.x_bounds[1]-self.x_bounds[0]) * (self.grid_length-1)).astype(np.int_).clip(0, self.grid_length-1),
            np.round((y-self.y_bounds[0])/(self.y_bounds[1]-self.y_bounds[0]) * (self.grid_length-1)).astype(np.int_).clip(0, self.grid_length-1)
        )
    
    def idx_to_pose(self, x, y):

        return (
            self.x_bounds[0] + np.array(x, dtype=np.float_) / (self.grid_length-1) * (self.x_bounds[1]-self.x_bounds[0]),
            self.y_bounds[0] + np.array(y, dtype=np.float_) / (self.grid_length-1) * (self.y_bounds[1]-self.y_bounds[0])
        )

    def locobot_odom_callback(self, data):
        
        if self.robot_type == "sim":

            self.current_pose = data.pose.pose

            self.current_euler = R.from_quat([
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w,
            ]).as_euler('ZYX')

            self.current_x = np.array([self.current_pose.position.x, self.current_pose.position.y, self.current_euler[0]])

        self.current_vel = data.twist.twist

    def mocap_callback(self, data):

        print("MOCAP CALLBACK")
        
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

        self.current_x = np.array([self.current_pose.position.x, self.current_pose.position.y, self.current_euler[0]])
    
    def plan_motion(self):

        field = np.ones((self.grid_length, self.grid_length), dtype=bool)

        # for x,y,c in self.blocks:
            
        #     field = field & (((self.grid_x - x)**2 + (self.grid_y-y)**2) >= (self.block_r+self.robot_r)**2)

        nearest_block = min([b for b in self.blocks if b[2] in self.colors], key=lambda b: (self.current_x[0]-b[0])**2+(self.current_x[1]-b[1])**2)

        path = dijkstra3d.dijkstra(field, self.pose_to_idx(*self.current_x[0:2]), self.pose_to_idx(nearest_block[0], nearest_block[1]), compass=True, connectivity=8)

        path_x, path_y = self.idx_to_pose(path[:,0], path[:,1])

        plt.plot(path_x, path_y)
        plt.show()

        print(path_x, path_y)
        
        self.abs_path_service(path_x.astype(np.float32), path_y.astype(np.float32), np.nan)

    def run(self):
        
        rospy.Subscriber("/locobot/mobile_base/odom", Odometry, self.locobot_odom_callback)
        if self.robot_type == "physical":
            rospy.Subscriber("/vrpn_client_node/" + self.robot_name + "/pose", PoseStamped, self.mocap_callback)

        rate = rospy.Rate(PLAN_HZ)

        rospy.wait_for_message("/vrpn_client_node/" + self.robot_name + "/pose", PoseStamped)
        self.plan_motion()
        

def main():
    rospy.init_node('motion_planner')
    cls_obj = MotionPlanner()
    cls_obj.run()


if __name__ == '__main__':
    main()

