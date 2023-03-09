#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospkg
import os

cubes = {
    'red': [
        [1.5, -0.2],
    ],
    'green': [
        [-1.2, -1.4],
    ],
    'blue': [
        [1.0, -0.4],
        [1.0, 0.5],
    ],
    'yellow': [
        [-0.6, 0.6],
    ],
}

rospy.init_node("spawn_cubes")
rospy.wait_for_service("gazebo/spawn_urdf_model")

spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

rospack = rospkg.RosPack()
package_path = rospack.get_path('me326_locobot_group5')

data = {}

for c in cubes.keys():

    with open(os.path.join(package_path, 'model', f'{c}_cube.urdf'), "r") as f:
        data[c] = f.read()

for c, blocks in cubes.items():

    for i,b in enumerate(blocks):

        name = f'{c}_cube_{i}'
        print(f"spawning item {name}")

        pose = Pose(Point(b[0], b[1], 0.03), Quaternion(0, 0, 0, 1))

        spawn_model(name, data[c], "", pose, "world")