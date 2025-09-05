#!/bin/bash

# run script to initialize simulation and exploration modules

# Launch simulation with Turtlebot4 in Gazebo with SLAM, Nav2, RViz and world 'warehouse'
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py slam:=true nav2:=true rviz:=true world:=warehouse &

# Wait for the simulation and Nav2 stack to initialize
sleep 10

# Run frontier detection module (find frontiers)
ros2 run frontier_exploration map_utils --find_frontiers &

# Run exploration module (control)
ros2 run frontier_exploration exploration

