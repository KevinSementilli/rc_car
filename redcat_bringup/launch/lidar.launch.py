import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    initialize_type = LaunchConfiguration('initialize_type')
    work_mode = LaunchConfiguration('work_mode')
    lidar_ip = LaunchConfiguration('lidar_ip')
    local_ip = LaunchConfiguration('local_ip')

    unitree_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('unitree_lidar_ros2'),'launch','launch_L2.py')),
        launch_arguments={
            'initialize_type': initialize_type,
            'work_mode': work_mode,
            'lidar_ip': lidar_ip,
            'local_ip': local_ip,
        }.items(),
    )

    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='unilidar_imu_initial_tf',
        arguments=['0', '0.1026', '0.0915', '0', '0', '0', 'base_link', 'unilidar_imu_initial'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('initialize_type', default_value='2'),
        DeclareLaunchArgument('work_mode', default_value='0'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.0.154'),
        DeclareLaunchArgument('local_ip', default_value='192.168.0.10'),

        lidar_static_tf,
        unitree_lidar_launch,
    ])