# Copyright 2023 Enjoy Robotics Zrt - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Modifications to this file is to be shared with the code owner.
# Proprietary and confidential
# Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2023

import os

from ament_index_python.packages \
    import get_package_share_directory as package_dir

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Whether or not to use '
                        '/clock from simulation (Gazebo)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    package_dir('nav2_bringup'),
                    'launch',
                    'tb3_simulation_launch.py')),
            launch_arguments={
                'world': os.path.join(
                    package_dir('gazebo_rotating_lidar_plugin'),
                    'worlds', 'waffle.model'),
                'headless': 'False'
            }.items()),

    ])
