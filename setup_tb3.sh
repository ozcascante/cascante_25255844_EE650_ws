#!/bin/bash

#source setup.bash
source /opt/ros/humble/setup.bash
source ~/cascante_25255844_EE650_ws/install/setup.bash

# Set environment variables
export ROS_DOMAIN_ID=11
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models/
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Improves Gazebo performance
export LIBGL_ALWAYS_SOFTWARE=1
export SVGA_VGPU=0
#export QT_QPA_PLATFORM=xcb

# ocascante custom Aliases to run Gazebo, Rviz and the project
alias startgazebo='ros2 launch turtlebot3_house_navigation random_spawn_house.launch.py'
alias startrviz='ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=./turtlebot3_house_navigation/maps/house_explored.yaml'
alias startpatrol='ros2 launch turtlebot3_house_navigation patrol_example_launch.py use_sim_time:=True'

# Print variables to verify
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
echo "LIBGL_ALWAYS_SOFTWARE: $LIBGL_ALWAYS_SOFTWARE"
echo "SVGA_VGPU: $SVGA_VGPU"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "aliases: startgazebo, startrviz, startpatrol"
echo "Environment loaded."
echo "1. startgazebo, 2. startrviz, 3. startpatrol"

