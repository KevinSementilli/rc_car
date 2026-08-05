import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    package_name = 'redcat'
    config_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'localization.yaml'
    )

    launch_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('imu_topic', default_value='unilidar/imu'),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('world_frame', default_value='odom'),
    ]

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'imu0': LaunchConfiguration('imu_topic'),
                'base_link_frame': LaunchConfiguration('base_link_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'world_frame': LaunchConfiguration('world_frame'),
            },
        ],
    )

    return LaunchDescription(launch_arguments + [ekf_node])
