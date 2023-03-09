#!/bin/bash

gnome-terminal -x roslaunch me326_locobot_group5 xslocobot_moveit_custom.launch robot_model:=locobot_wx250s show_lidar:=true use_camera:=true use_gazebo:=true dof:=6 use_moveit_rviz:=false gui:=true

sleep 10
rosservice call /gazebo/unpause_physics

sleep 5
rosrun me326_locobot_group5 spawn_cubes.py
# gnome-terminal -x roslaunch me326_locobot_group5 gazebo_moveit_example.launch 


