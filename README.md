# CS326 Group 5 Collaborative Robotics Project (Winter 2022-23)

## Getting Started and Dependencies

### Building repository

```
cd ~/catkin_ws/src
git clone https://github.com/chengine/CS326_Winter2022_Group5.git
catkin build me326_locobot_group5
source ../devel/setup.bash
```

### To launch simulation

In one window run the following commands
```
roscd me326_locobot_group5
./launch/launch_locobot_gazebo_moveit.sh
```

In a second window run the following commands
```
roscd me326_locobot_group5
roslaunch me326_locobot_group5 gazebo_moveit_example.launch
```

### To launch on hardware
```
roscd me326_locobot_group5
roslaunch me326_locobot_group5 gazebo_moveit_example.launch robot_name:=locobot_1
```

### To open rviz
```
roscd me326_locobot_group5
rviz -d rviz/rviz_traj_env.rviz
```

### To open image viewer
```
rqt_image_view
```

## Method

## File Structure

## Contributors
