#!/bin/bash

#source setup.bash
source install/setup.bash

# Set environment variables
export ROS_DOMAIN_ID=11
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models/
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Improves Gazebo performance
export LIBGL_ALWAYS_SOFTWARE=1
export SVGA_VGPU=0

# Print variables to verify
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
echo "LIBGL_ALWAYS_SOFTWARE: $LIBGL_ALWAYS_SOFTWARE"
echo "SVGA_VGPU: $SVGA_VGPU"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"


# Keep the shell open with the variables active
#$SHELL
