import os

from ament_index_python.packages import get_package_share_directory as package_dir

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Whether or not to use /clock from simulation (Gazebo)'
    )

    # Launch Gazebo with rotating lidar plugin
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_dir('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': os.path.join(
                package_dir('gazebo_rotating_lidar_plugin'),
                'worlds',
                'waffle.model'
            ),
            'headless': 'False'
        }.items()
    )


    return LaunchDescription([
        use_sim_time_arg,
        gazebo_launch
    ])
