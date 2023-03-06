#!/usr/bin/env python3

import math

import rospy
from me326_locobot_group5.srv import *
from me326_locobot_group5.msg import *


CAMERA_FOV_HORIZONTAL = 60 # degrees
CAMERA_MIN_RANGE = 0.2 # meters
CAMERA_MAX_RANGE = 1.0 # meters

class BlockEstimate():

    DIST_THRESHOLD = 0.05
    ALPHA_UPDATE = 0.8
    ALPHA_DECAY = 0.95
    MIN_CONFIDENCE = 0.1

    def __init__(self, x, y, color, confidence):

        self.x = x
        self.y = y
        self.color = color
        self.confidence = confidence

    def in_fov(self, pose2d):

        x,y,theta = pose2d

        dist = math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2)
        angle = math.atan2(self.y - y, self.x - x) - theta

        return dist > CAMERA_MIN_RANGE and dist < CAMERA_MAX_RANGE and abs(angle) < math.radians(CAMERA_FOV_HORIZONTAL)/2    

    def dist(self, pose2d):

        x,y,theta = pose2d

        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2)

    def decay(self):

        self.confidence *= BlockEstimate.ALPHA_DECAY 

    def near(self, other):

        return self.color == other.color and (self.x - other.x) ** 2 + (self.y - other.y) ** 2 < (BlockEstimate.DIST_THRESHOLD / max(BlockEstimate.MIN_CONFIDENCE, min(self.confidence, other.confidence))) ** 2
    
    def update(self, other):

        self.x = BlockEstimate.ALPHA_UPDATE * self.x + (1.0 - BlockEstimate.ALPHA_UPDATE) * other.x
        self.y = BlockEstimate.ALPHA_UPDATE * self.y + (1.0 - BlockEstimate.ALPHA_UPDATE) * other.y
        self.confidence = BlockEstimate.ALPHA_UPDATE * self.confidence + (1.0 - BlockEstimate.ALPHA_UPDATE) * other.confidence


class BlockMapper():
    
    def __init__(self, localizer):

        self.localizer = localizer

        self.robot_type = rospy.get_param('/robot_type')

        self.blocks_list = []

    def blocks_callback(self, data):

        new_blocks = [BlockEstimate(b.x, b.y, b.color, 1.0) for b in data]
        
        for b in self.blocks_list:
            for nb in new_blocks:
                if b.near(nb):
                    b.update(nb)
                    break

    def update_map(self):

        for b in self.blocks_list:
            if b.in_fov(self.localizer.pose2d):
                b.decay()

        self.blocks_list = filter(self.blocks_list, lambda b : b.confidence > BlockEstimate.MIN_CONFIDENCE)

    def get_nearest_block(self, colors=[]):

        if isinstance(colors, str):
            colors = [colors,]
        
        return min(filter(self.blocks_list, lambda b: b.color in colors), lambda b: b.dist(self.localizer.pose2d))
    
    def run(self):

        if self.robot_type == "sim":
            rospy.Subscriber("/block_simulator/blocks", BlockArray, self.blocks_callback)
        elif self.robot_type == "physical":
            pass # TODO : implement