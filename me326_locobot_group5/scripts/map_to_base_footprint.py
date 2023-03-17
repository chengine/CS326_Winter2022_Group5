#!/usr/bin/env python

import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy

# Because of transformations
import tf_conversions

import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped


def mocap_callback(data, frame_offset):

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    euler = R.from_quat([
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w,
    ]).as_euler('ZYX')
    
    theta = euler[0] + frame_offset['theta']
    x = data.pose.position.x + frame_offset['x'] * np.cos(euler[0])
    y = data.pose.position.y + frame_offset['y'] * np.sin(euler[0])

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "locobot/base_footprint"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':

    rospy.init_node('map_to_base_footprint')

    robot_name = rospy.get_param('/robot_name')
    robot_type = rospy.get_param('/robot_type')
    frame_offset = {
        'x' : rospy.get_param('/frame_offset/x'),
        'y' : rospy.get_param('/frame_offset/y'),
        'theta' : rospy.get_param('/frame_offset/theta')
    }

    if robot_type == "physical":
        print("starting map_to_base_footprint")
        rospy.Subscriber("/vrpn_client_node/" + robot_name + "/pose", PoseStamped, lambda data: mocap_callback(data, frame_offset))
    
    rospy.spin()