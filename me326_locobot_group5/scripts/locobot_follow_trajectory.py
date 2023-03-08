#!/usr/bin/env python3

from enum import IntEnum

import numpy as np
import scipy.interpolate
from scipy.spatial.transform import Rotation as R

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Header, Bool
from visualization_msgs.msg import Marker
from me326_locobot_group5.srv import *

from ramsete import Ramsete
from pose_controller import PoseController
from trajectory_generator import compute_smoothed_traj, modify_traj_with_limits
from localizer import Localizer

from utils import wrap_to_pi

CTRL_HZ = 50
CTRL_DT = 1/CTRL_HZ

class TrajectoryFollowerState(IntEnum):
    IDLE = 0,
    TURN_TO_START = 1,
    FOLLOW_TRAJ = 2,
    GO_TO_POSE = 3,
    TURN_TO_FINISH = 4,

class TrajectoryFollower(object):
    
    def __init__(self):

        # ros params
        self.robot_name = rospy.get_param('/robot_name')
        self.robot_type = rospy.get_param('/robot_type')
        self.frame_offset = {
            'x' : rospy.get_param('/frame_offset/x'),
            'y' : rospy.get_param('/frame_offset/y'),
            'theta' : rospy.get_param('/frame_offset/theta')
        }
        
        # publishers and messages

        self.mobile_base_vel_publisher = rospy.Publisher("/locobot/mobile_base/commands/velocity", Twist, queue_size=1)
        self.target_pose_visual = rospy.Publisher("/locobot/mobile_base/target_pose_visual", Marker, queue_size=1)

        self.path_visual = rospy.Publisher('/locobot/mobile_base/path_visual', Path, queue_size=1)
        self.traj_visual = rospy.Publisher('/locobot/mobile_base/traj_visual', Path, queue_size=1)

        self.idle_publisher = rospy.Publisher('/traj_follower/idle', Bool)

        if self.robot_type == "sim":
            self.frame_id = "locobot/odom"
        elif self.robot_type == "physical":
            self.frame_id = "map"

        self.empty_path = Path(
            header=Header(frame_id=self.frame_id),
        )

        self.path_msg = self.empty_path
        self.traj_msg = self.empty_path

        # services
        self.abs_path_service = rospy.Service('/traj_follower/abs_path', FollowPath, self.follow_abs_path)
        self.rel_path_service = rospy.Service('/traj_follower/rel_path', FollowPath, self.follow_rel_path)
        self.turn_heading_service = rospy.Service('/traj_follower/turn_heading', TurnHeading, self.turn_heading)

        # control limits
        self.v_max = 0.3 # m/s
        self.w_max = np.pi/4 # rad/s

        # traj gen limits
        self.v_lim = 0.1 # m/s
        self.a_lim = 0.03 # m/s^2
        self.w_lim = np.pi/8 # rad/s

        # traj gen smoothing parameter
        self.alpha = 0.5 

        # set up controllers
        self.ramsete = Ramsete(b=2.0, zeta=0.8)
        self.pose_controller = PoseController(k1=0.4, k2=0.8, k3=0.8)
        self.heading_kP = 1.0
        self.traj_complete_thresh = 0.2 # m
        self.at_pose_thresh = 0.05 # m
        self.at_heading_thresh = np.radians(2) # deg

        # initialize localizer
        self.localizer = Localizer()

        # initialize state variables
        
        self.last_timestamp = rospy.Time.now()

        self.state = TrajectoryFollowerState.IDLE
        self.controller = "none"

        self.t_final = 0
        self.x_initial = np.zeros((3,))
        self.x_final = np.zeros((3,))
        self.theta_final = 0

        self.x_goal = np.zeros((3,))

    def set_traj(self, path, final_heading=np.nan):

        traj, traj_ts, = compute_smoothed_traj(path, self.v_lim, self.alpha, CTRL_DT)
        traj_ts, v_goal, w_goal, traj = modify_traj_with_limits(traj, traj_ts, self.v_lim, self.a_lim, self.w_lim, CTRL_DT)

        self.traj = scipy.interpolate.interp1d(traj_ts, traj, axis=0, bounds_error=False, fill_value=(traj[0], traj[-1]))
        self.v_goal = scipy.interpolate.interp1d(traj_ts, v_goal, bounds_error=False, fill_value=0)
        self.w_goal = scipy.interpolate.interp1d(traj_ts, w_goal, bounds_error=False, fill_value=0)
        
        self.t_final = traj_ts[-1]
        self.x_initial = traj[0, 0:3]
        self.x_final = traj[-1, 0:3]
        self.theta_final = traj[-1, 2] if np.isnan(final_heading) else final_heading

        self.path_msg = Path(
            header=Header(frame_id=self.frame_id),
            poses=[PoseStamped(pose=Pose(position=Point(x=i[0], y=i[1]))) for i in path]
        )

        self.traj_msg = Path(
            header=Header(frame_id=self.frame_id),
            poses=[PoseStamped(pose=Pose(position=Point(x=(traj_pt := self.traj(t))[0], y=traj_pt[1]))) for t in np.linspace(0, self.t_final, 100)]
        )

        self.transition_state(TrajectoryFollowerState.TURN_TO_START)

    def follow_abs_path(self, req):

        path = np.array([req.x, req.y]).T
        self.set_traj(path, wrap_to_pi(req.final_heading))

        return {}
    
    def follow_rel_path(self, req):

        c, s = np.cos(self.localizer.pose2d[2]), np.sin(self.localizer.pose2d[2])
        R = np.array([[c,-s],[s,c]])

        path = (R @ np.array([req.x, req.y])).T + self.localizer.pose2d[0:2]
        
        self.set_traj(path, self.localizer.pose2d[2] + wrap_to_pi(req.final_heading))

        return {}

    def turn_heading(self, req):

        self.x_final = np.array([*self.localizer.pose2d[0:2], wrap_to_pi(req.heading)])
        self.theta_final = wrap_to_pi(req.heading)
        self.transition_state(TrajectoryFollowerState.TURN_TO_FINISH)

        return {}


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
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (*self.x_goal[0:2], 0)
        (
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ) = R.from_euler('ZYX', [self.x_goal[2], 0, 0]).as_quat()

        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 0.0 #red
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Publish the marker
        self.target_pose_visual.publish(marker)

    def check_state_transitions(self, x, t):

        new_state = self.state

        if self.state == TrajectoryFollowerState.IDLE:
            pass
        elif self.state == TrajectoryFollowerState.TURN_TO_START:
            if abs(self.x_goal[2]-x[2]) < self.at_heading_thresh:
                new_state = TrajectoryFollowerState.FOLLOW_TRAJ
        elif self.state == TrajectoryFollowerState.FOLLOW_TRAJ:
            if np.linalg.norm(self.x_final[0:2]-x[0:2]) < self.traj_complete_thresh or t > self.t_final:
                new_state = TrajectoryFollowerState.GO_TO_POSE
        elif self.state == TrajectoryFollowerState.GO_TO_POSE:
            if np.linalg.norm(self.x_goal[0:2]-x[0:2]) < self.at_pose_thresh:
                new_state = TrajectoryFollowerState.TURN_TO_FINISH
        elif self.state == TrajectoryFollowerState.TURN_TO_FINISH:
            if abs(self.x_goal[2]-x[2]) < self.at_heading_thresh:
                new_state = TrajectoryFollowerState.IDLE

        return new_state
    
    def transition_state(self, new_state):

        if new_state != self.state:
            print("OLD_STATE:", self.state, "-> NEW_STATE:", new_state)
            self.state = new_state
            if self.state == TrajectoryFollowerState.IDLE:
                self.x_goal = self.localizer.pose2d
                self.path_msg = self.empty_path
                self.traj_msg = self.empty_path
                self.controller = "none"
                self.idle_publisher.publish(Bool(True))
            elif self.state == TrajectoryFollowerState.TURN_TO_START:
                self.x_goal = self.x_initial
                self.controller = "heading"
            elif self.state == TrajectoryFollowerState.FOLLOW_TRAJ:
                # self.x_goal is updated every cycle in controller
                self.controller = "ramsete"
            elif self.state == TrajectoryFollowerState.GO_TO_POSE:
                self.x_goal = self.x_final
                self.controller = "pose"
            elif self.state == TrajectoryFollowerState.TURN_TO_FINISH:
                self.x_goal = np.concatenate((self.x_final[0:2], np.atleast_1d(self.theta_final)))
                self.controller = "heading"
            self.last_timestamp = rospy.Time.now()


    def compute_control(self):

        # compute current time (since last state transition)
        t = (rospy.Time.now()-self.last_timestamp).to_sec()
        print("time: ", t)

        # copy localizer pose2d to local variable
        x = self.localizer.pose2d

        # check transition conditions
        new_state = self.check_state_transitions(x, t)

        # transition state if new_state is different
        self.transition_state(new_state)

        print("state: ", self.state)
        print("controller: ", self.controller)

        if self.controller == "ramsete":

            self.x_goal = self.traj(t)[0:3]
            v_goal = self.v_goal(t)
            w_goal = self.w_goal(t)
            v, w = self.ramsete.calculate(self.x_goal, v_goal, w_goal, x)

            print('x: ', x)
            print('x_goal: ', self.x_goal)
            print('tracking_error: ', np.linalg.norm(self.x_goal[0:2]-x[0:2]))
            print('v_goal: ', v_goal)
            print('w_goal: ', w_goal)

        elif self.controller == "pose":

            v, w = self.pose_controller.calculate(self.x_goal, x)

            print('x: ', x)
            print('x_goal: ', self.x_goal)
            print('pose_error: ', np.linalg.norm(self.x_goal[0:2]-x[0:2]))

        elif self.controller == "heading":

            theta_goal = self.x_goal[2]
            theta_error = wrap_to_pi(theta_goal - x[2])
            v, w = (0, self.heading_kP * theta_error)

            print('x: ', x)
            print('x_goal: ', self.x_goal)
            print('pose_error: ', np.linalg.norm(self.x_goal[0:2]-x[0:2]))
            print('current heading: ', x[2])
            print('goal heading: ', theta_goal)
            print('heading error: ', theta_error)

        else:

            v, w = (0, 0)

        print('v_cmd: ', v)
        print('w_cmd: ', w)
        
        # build control message
        control_msg = Twist()
        control_msg.linear.x = np.clip(v, -self.v_max, self.v_max)
        control_msg.angular.z = np.clip(w, -self.w_max, self.w_max)
        
        # publish control message
        self.mobile_base_vel_publisher.publish(control_msg)

        # publish target marker
        self.pub_target_point_marker()
        
    def run(self):
        
        self.localizer.run()

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
    cls_obj = TrajectoryFollower()
    # cls_obj.set_traj(
    #     path = np.array([
    #         [0, 0],
    #         [1, 0],
    #         [3, 2],
    #         [5, -1],
    #     ]) / 2,
    #     final_heading = np.pi/2,
    # )
    # cls_obj.set_traj(
    #     path = np.array([
    #         [-0.4, 1.8],
    #         [0, 1.8],
    #         [0.5, 1.9],
    #         [0.6, 1.9]
    #     ]),
    # )
    cls_obj.run()



if __name__ == '__main__':
    main()

