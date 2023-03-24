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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    bringup_dir = package_dir('nav2_bringup')

    urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    world = os.path.join(
        package_dir('gazebo_rotating_lidar_plugin'),
        'worlds', 'waffle.model')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Whether or not to use '
                        '/clock from simulation (Gazebo)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'robot_description': robot_description}]),

        ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so', '--verbose', world],
            output='screen'),

    ])
