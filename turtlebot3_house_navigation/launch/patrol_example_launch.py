import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('turtlebot3_house_navigation')
    plansys_dir = get_package_share_directory('plansys2_bringup')

    config_params_file = os.path.join(pkg_dir, 'params', 'config.yaml')

    # PlanSys2 Bringup
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(plansys_dir, 'launch', 'plansys2_bringup_launch_monolithic.py')
        ),
        launch_arguments={
            'model_file': os.path.join(pkg_dir, 'pddl', 'patrol.pddl'),
            'use_sim_time': 'True'
        }.items()
    )

    common_params = [config_params_file, {'use_sim_time': True}]

    # Problem generator - runs first to set up random initial state
    problem_generator_cmd = Node(
        package='turtlebot3_house_navigation',
        executable='problem_generator_node',
        name='problem_generator_node',
        output='screen',
        parameters=common_params)

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

    turn_off_light_cmd = Node(
        package='turtlebot3_house_navigation',
        executable='turn_off_light_action_node',
        name='turn_off_light_action_node',
        output='screen',
        parameters=common_params)

    controller_cmd = Node(
        package='turtlebot3_house_navigation',
        executable='patrolling_controller_node',
        name='patrolling_controller_node',
        output='screen',
        parameters=common_params)

    return LaunchDescription([
        plansys2_cmd,
        problem_generator_cmd,
        move_cmd,
        patrol_cmd,
        turn_off_light_cmd,
        controller_cmd
    ])
