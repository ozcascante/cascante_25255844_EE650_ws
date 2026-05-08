# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Setup paths
    pkg_dir = get_package_share_directory('turtlebot3_house_navigation')
    plansys_dir = get_package_share_directory('plansys2_bringup')
    
    # Corrected variable name
    rooms_params_file = os.path.join(pkg_dir, 'params', 'rooms.yaml')

    # 2. PlanSys2 Bringup
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            plansys_dir, 'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': os.path.join(pkg_dir, 'pddl', 'patrol.pddl'),
            'use_sim_time': 'True'
        }.items()
    )

    # 3. Your custom nodes (Note the added use_sim_time)
    common_params = [rooms_params_file, {'use_sim_time': True}]

    move_cmd = Node(
        package='turtlebot3_house_navigation',
        executable='move_action_node',
        name='move_action_node',
        output='screen',
        parameters=common_params)

    patrol_cmd = Node(
        package='turtlebot3_house_navigation',
        executable='patrol_action_node',
        name='patrol_action_node',
        output='screen',
        parameters=common_params)

    controller_cmd = Node(
        package='turtlebot3_house_navigation',
        executable='patrolling_controller_node',
        name='patrolling_controller_node',
        output='screen',
        parameters=common_params)    

    # 4. Create Launch Description
    ld = LaunchDescription()

    # Add actions (Make sure gazebo is launched separately or uncomment the gazebo_cmd logic)
    ld.add_action(plansys2_cmd)
    ld.add_action(move_cmd)
    ld.add_action(patrol_cmd)
    ld.add_action(controller_cmd)

    return ld

