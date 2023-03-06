#!/usr/bin/env python3

import numpy as np

import rospy
from std_srvs.srv import Empty
from me326_locobot_group5.srv import *
from me326_locobot_group5.msg import *

from localizer import Localizer

CTRL_HZ = 10
CTRL_DT = 1/CTRL_HZ


class BlockSimulator():
    
    def __init__(self):
        
        # publishers
        self.block_publisher = rospy.Publisher("/block_simulator/blocks", BlockArray, queue_size=1)

        # services
        self.grab_block_service = rospy.Service('/block_simulator/grab_block', BlockSrv, self.grab_block)
        self.drop_block_service = rospy.Service('/block_simulator/drop_block', Empty, self.drop_block)

        # initialize localizer
        self.localizer = Localizer()

        # initialize state variables

        robot_radius = 0.3 # meters
        station_radius = 0.3 # meters
        block_radius = 0.03 # meters
        
        colors = ['red','blue','yellow','green']
        resource = np.array([[1,2,3,0],[1,3,0,2],[0,3,2,0]])

        station_locations = np.array([[0,0],[1,0],[-1,0],[0,-1]])
        
        x_bounds = [-2, 2]
        y_bounds = [-2, 2]

        block_multiplier = 2
        num_blocks = block_multiplier * np.sum(resource, axis=0)

        self.blocks_list = []

        for i in range(len(colors)):
            for j in range(num_blocks[i]):
                while True:
                    pos = np.random.uniform(*zip(x_bounds, y_bounds))
                    if all([np.linalg.norm(pos-s) < block_radius+station_radius for s in station_locations]):
                        break
                self.blocks_list.append({'pos': pos, 'color': colors[i]})

        self.current_block = None

        self.fov_horizontal = 60 # degrees
        self.min_range = 0.2 # meters
        self.max_range = 1.0 # meters

        self.drop_dist = robot_radius + 0.1 # meters

    def grab_block(self, req):

        blocks_color = [(i,b) for (i,b) in enumerate(self.blocks_list) if b['color'] == req.block.color]

        if len(blocks_color) > 0:
            query_pos = np.array([req.block.x, req.block.y])
            idx, self.current_block = min(blocks_color, lambda b : np.linalg.norm(b[1]['pos'] - query_pos))
            self.blocks_list.remove(idx)

        return {}

    def drop_block(self):

        if self.current_block is not None:
            
            self.blocks_list.append({
                'pos': self.localizer.pose2d[0:2] + self.drop_dist * np.array([np.cos(self.localizer.pose2d[2]), np.sin(self.localizer.pose2d[2])]),
                'color': self.current_block['color']
            })
            self.current_block = None

        return {}

    def simulate_blocks(self):

        visible_blocks = []

        pose2d = self.localizer.pose2d

        for b in self.blocks_list:

            dist = np.linalg.norm(b['pos'] - pose2d[0:2])
            angle = np.arctan2(b['pos'][1] - pose2d[1], b['pos'][0] - pose2d[0]) - pose2d[2]

            if dist > self.min_range and dist < self.max_range and np.abs(angle) < np.radians(self.fov_horizontal)/2:
                visible_blocks.append(b)

        self.block_publisher.publish(BlockArray([Block(*b['pos'], b['color']) for b in visible_blocks]))
        
    def run(self):
        
        self.localizer.run()

        rate = rospy.Rate(CTRL_HZ)

        while not rospy.is_shutdown():

            self.simulate_blocks()
            rate.sleep()


def main():

    rospy.init_node('block_simulator')
    BlockSimulator().run()


if __name__ == '__main__':
    main()

