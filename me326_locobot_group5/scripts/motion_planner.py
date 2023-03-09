#!/usr/bin/env python3

from enum import IntEnum

import numpy as np
from matplotlib import pyplot as plt

import rospy
from std_msgs.msg import Bool

import dijkstra3d

from me326_locobot_group5.srv import *
from utils import wrap_to_pi


class MotionPlannerState(IntEnum):
    IDLE = 0,
    EXECUTING = 1,

class MotionPlanner():
    
    def __init__(self, localizer, blocks):

        # ros params
        self.robot_name = rospy.get_param('/robot_name')
        self.robot_type = rospy.get_param('/robot_type')
        self.frame_offset = {
            'x' : rospy.get_param('/frame_offset/x'),
            'y' : rospy.get_param('/frame_offset/y'),
            'theta' : rospy.get_param('/frame_offset/theta')
        }
        self.robot_id = rospy.get_param('/robot_id')

        self.station_locations = np.array(rospy.get_param('/station_locations'))
        self.target_config = np.array(rospy.get_param('/target_config'))
        self.robot_1_colors = np.array(rospy.get_param('/robot_1_colors')).flatten()
        self.robot_2_colors = np.array(rospy.get_param('/robot_2_colors')).flatten()

        self.possible_colors = ['r', 'g', 'b', 'y']

        self.my_colors = [self.possible_colors[i] for i in (self.robot_1_colors.nonzero()[0] if self.robot_id == 1 else self.robot_2_colors.nonzero()[0])]

        # services
        self.abs_path_service = rospy.ServiceProxy('/traj_follower/abs_path', FollowPath)
        self.turn_heading_service = rospy.ServiceProxy('/traj_follower/turn_heading', TurnHeading)

        # parameters
        self.x_bounds = [-2, 2]
        self.y_bounds = [-2, 2]
        self.grid_length = 512

        self.grid_x, self.grid_y = np.meshgrid(np.linspace(*self.x_bounds, self.grid_length), np.linspace(*self.y_bounds, self.grid_length))

        # initialize measurements
        self.localizer = localizer

        # initialize blocks list
        self.blocks = blocks
        
        self.block_r = 0.03
        self.robot_r = 0.2

        self.robot_stop_buffer = 0.25
        self.robot_avoid_buffer = 0.05

        self.state = MotionPlannerState.IDLE

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


    def turn_to_heading(self, heading):

        heading = wrap_to_pi(heading)

        print("turn_to_heading: ", heading)

        self.turn_heading_service(heading)
        self.state = MotionPlannerState.EXECUTING


    def go_to_nearest_block(self, colors=None):
        
        if colors is None:
            colors = self.my_colors
        
        if isinstance(colors, str):
            colors = [colors,]

        print("go_to_nearest_block: ", colors)

        blocks_of_interest = [b for c in colors for b in self.blocks[c]]

        if not blocks_of_interest:
            print("error: no blocks of specified colors")
            return
        
        nearest_block = min(blocks_of_interest, key=lambda b: (self.localizer.pose2d[0]-b[0])**2+(self.localizer.pose2d[1]-b[1])**2)

        self.plan_motion(nearest_block, self.block_r)


    def go_to_station(self, station_idx):

        print("go_to_station: ", station_idx)

        station_pos = self.station_locations[station_idx]

        self.plan_motion(station_pos[0:2], station_pos[2])


    def plan_motion(self, target_pos, target_r):

        # build occupancy mask, avoiding stations and known blocks

        graph = np.full((self.grid_length, self.grid_length), 0xffffffff, dtype=np.uint32)
        
        for x,y,_ in [b for v in self.blocks.values() for b in v]:

            graph[((self.grid_x - x)**2 + (self.grid_y - y)**2) <= (self.block_r+self.robot_r+self.robot_avoid_buffer)**2] = 0

        for x,y,r in self.station_locations:

            graph[((self.grid_x - x)**2 + (self.grid_y - y)**2) <= (r+self.robot_r+self.robot_avoid_buffer)**2] = 0

        graph[((self.grid_x - target_pos[0])**2 + (self.grid_y - target_pos[1])**2) <= (target_r+self.robot_r+self.robot_stop_buffer)**2] = 0xffffffff

        plt.imshow(graph, origin='lower')
        plt.title('occupancy mask')
        plt.show()

        # equal weights except for disk around target pos, so shortest path can approach from any side

        field = np.ones(graph.shape)
        field[((self.grid_x - target_pos[0])**2 + (self.grid_y - target_pos[1])**2) <= (target_r+self.robot_r+self.robot_stop_buffer)**2] = 0

        # run dijkstra

        path = dijkstra3d.dijkstra(
            field,
            self.pose_to_idx(*self.localizer.pose2d[0:2]),
            self.pose_to_idx(target_pos[0], target_pos[1]),
            voxel_graph=graph.T,
            connectivity=8,
        )

        if len(path) == 0:
            print("error: no feasible path found")
            return

        path_x, path_y = self.idx_to_pose(path[:,0], path[:,1])

        mask = ((path_x - target_pos[0])**2 + (path_y - target_pos[1])**2) >= (self.block_r+self.robot_r+self.robot_stop_buffer)**2 

        path_x, path_y = path_x[mask], path_y[mask]

        diff = target_pos[0:2] - np.array([path_x[-1], path_y[-1]])

        final_heading = np.arctan2(diff[1], diff[0])

        # display path

        plt.plot(path_x, path_y)
        t = np.linspace(0,2*np.pi,100)
        plt.plot(self.localizer.pose2d[0]+self.robot_r*np.cos(t), self.localizer.pose2d[1]+self.robot_r*np.sin(t), color='green')
        for x,y,r in self.station_locations:
            plt.plot(x+r*np.cos(t),y+r*np.sin(t), color='gray', ls='--')
        for c,(x,y,_) in [(k,b) for k,v in self.blocks.items() for b in v]:
            plt.scatter(x,y,color=c,zorder=4)
        plt.gca().axis('equal')
        plt.title('planned path')
        plt.show()

        print(path_x, path_y)
        
        # call path following service

        self.abs_path_service(path_x.astype(np.float32), path_y.astype(np.float32), final_heading)
        self.state = MotionPlannerState.EXECUTING


    def traj_follower_idle_callback(self, data):
        if data.data:
            self.state = MotionPlannerState.IDLE

    def run(self):

        rospy.Subscriber('/traj_follower/idle', Bool, self.traj_follower_idle_callback)



