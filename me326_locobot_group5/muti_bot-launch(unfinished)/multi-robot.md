the locobot multi robot demo is at /interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_descriptions/launch

one launch file that sets up the environment, enable the robot, moveit

control.launch similar to moveit.launch

each robot has own launch file, responsible for setting parameters right

another launch file for calling the nodes

only one for gazebo and one for rviz

two for description



combined launch file that has robot description @ 
```
/home/azrael2/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_moveit/launch/many_robot_copy3.launch
```

edited rviz file @ 
```
/home/azrael2/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_descriptions/rviz/many_robot_copy1.rviz
```

edited moveit file @
```
/home/azrael2/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_moveit/launch
```

edited gazebo file @
```
/home/azrael2/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_gazebo/launch
```



many_copy3.launch: first define the two robots, then create gazebo empty world, then call moveit_copy1.launch once for each robot, then assign node to each robot according to name space

copy1.rviz: edited to only contain two robots according to nitish

moveit_copy1.launch: edited from og moveit launch file, but without the gazebo spawning. 

gazebo_copy1.launch: edited from og gazebo launch file, but without the empty_world.launch, that part is moved to copy3


EZ commands 4 testing scripts:
roslaunch interbotix_xslocobot_descriptions many_xslocobots.launch
roslaunch interbotix_xslocobot_descriptions many_robot_copy3.launch

roslaunch gazebo_ros empty_world.launch world_name:=za-warudo paused:=true
roslaunch interbotix_xslocobot_gazebo xslocobot_gazebo_copy1.launch robot_model:=locobot_wx250s robot_name:=locobot_1
roslaunch interbotix_xslocobot_moveit xslocobot_moveit_copy1.launch robot_model:=locobot_wx250s robot_name:=locobot_1 show_lidar:=true use_gazebo:=true dof:=6
