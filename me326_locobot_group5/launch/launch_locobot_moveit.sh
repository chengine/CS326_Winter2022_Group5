#!/bin/bash

gnome-terminal -x roslaunch interbotix_xslocobot_moveit xslocobot_moveit.launch robot_model:=locobot_wx250s show_lidar:=true dof:=6

gnome-terminal -x roslaunch me326_locobot_group5 gazebo_moveit_example.launch 


