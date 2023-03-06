#!/usr/bin/env python3

from enum import Enum

import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from me326_locobot_group5.srv import *
from matplotlib import pyplot as plt

import dijkstra3d

from localizer import Localizer

PLAN_HZ = 1
PLAN_DT = 1/PLAN_HZ

class State(Enum):
    IDLE = 0,
    STARTUP = 1,
    SCAN_FOR_BLOCKS = 2,
    DRIVE_TO_BLOCK = 3,
    GRAB_BLOCK = 4,
    DRIVE_TO_GOAL = 5,
    DROP_BLOCK = 6,

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
        self.localizer = Localizer()

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

    def check_state_transitions(self):

        new_state = self.state

        if self.state == State.IDLE:
            pass
        elif self.state == State.STARTUP:
            if self.localizer.is_valid():
                new_state = State.SCAN_FOR_BLOCKS
        elif self.state == State.SCAN_FOR_BLOCKS:
            if True: # if we have a nearest block
                new_state = State.DRIVE_TO_BLOCK
        elif self.state == State.DRIVE_TO_BLOCK:
            if True: # if we are at the block
                new_state = State.GRAB_BLOCK
        elif self.state == State.GRAB_BLOCK:
            if True: # if we have the block
                new_state = State.DRIVE_TO_GOAL
        elif self.state == State.DRIVE_TO_GOAL:
            if True: # if we are at the goal
                new_state = State.DROP_BLOCK
        elif self.state == State.DROP_BLOCK:
            if True: # if we have dropped the block
                new_state = State.DRIVE_TO_BLOCK

        return new_state
    
    def transition_state(self, new_state):

        if new_state != self.state:
            print("OLD_STATE:", self.state, "-> NEW_STATE:", new_state)
            self.state = new_state
            if self.state == State.IDLE:
                self.x_goal = self.current_x
                self.path_msg = self.empty_path
                self.traj_msg = self.empty_path
                self.controller = "none"
            elif self.state == State.TURN_TO_START:
                self.x_goal = self.x_initial
                self.controller = "heading"
            elif self.state == State.FOLLOW_TRAJ:
                # self.x_goal is updated every cycle in controller
                self.controller = "ramsete"
            elif self.state == State.GO_TO_POSE:
                self.x_goal = self.x_final
                self.controller = "pose"
            elif self.state == State.TURN_TO_FINISH:
                self.x_goal = np.concatenate((self.x_final[0:2], np.atleast_1d(self.theta_final)))
                self.controller = "heading"
            self.last_timestamp = rospy.Time.now()

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
        
        self.localizer.run()

        rate = rospy.Rate(PLAN_HZ)

        while not rospy.is_shutdown():
            self.plan_motion()
            rate.sleep()
        

def main():
    rospy.init_node('motion_planner')
    cls_obj = MotionPlanner()
    cls_obj.run()


if __name__ == '__main__':
    main()

