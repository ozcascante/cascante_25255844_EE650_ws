import os
import random
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Load rooms.yaml
    pkg_dir = get_package_share_directory('turtlebot3_house_navigation')
    rooms_yaml = os.path.join(pkg_dir, 'params', 'config.yaml')

    with open(rooms_yaml, 'r') as f:
        data = yaml.safe_load(f)
        params = data['/**']['ros__parameters']
        rooms = params['waypoints']
        start_room = random.choice(rooms)
        x, y, yaw = params[start_room]

    # Write start room to a temp file for other nodes to read. Probably we should use a topic but this is faster
    # for now to develop. I will revisit if I have time to use a topic.
    start_room_file = '/tmp/patrol_start_room.txt'
    with open(start_room_file, 'w') as f:
        f.write(start_room)
    print(f"\033[1;94m[random_spawn] Selected Random start room: {start_room} at ({x}, {y})\033[0m")


    # Launch Gazebo + TB3 in the house world
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_house.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'x_pose': str(x),
            'y_pose': str(y),
            'z_pose': '0.01',
            'yaw': str(yaw)
        }.items()
    )

    return LaunchDescription([gazebo_cmd])
