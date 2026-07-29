from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument('initialize_type', default_value='2'),
        DeclareLaunchArgument('work_mode', default_value='0'),
        DeclareLaunchArgument('use_system_timestamp', default_value='true'),
        DeclareLaunchArgument('range_min', default_value='0.0'),
        DeclareLaunchArgument('range_max', default_value='100.0'),
        DeclareLaunchArgument('cloud_scan_num', default_value='18'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='4000000'),
        DeclareLaunchArgument('lidar_port', default_value='6101'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.0.154'),
        DeclareLaunchArgument('local_port', default_value='6201'),
        DeclareLaunchArgument('local_ip', default_value='192.168.0.10'),
        DeclareLaunchArgument('cloud_frame', default_value='unilidar_lidar'),
        DeclareLaunchArgument('cloud_topic', default_value='unilidar/cloud'),
        DeclareLaunchArgument('imu_frame', default_value='unilidar_imu'),
        DeclareLaunchArgument('imu_topic', default_value='unilidar/imu'),
    ]

    node1 = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[{
            'initialize_type': ParameterValue(LaunchConfiguration('initialize_type'), value_type=int),
            'work_mode': ParameterValue(LaunchConfiguration('work_mode'), value_type=int),
            'use_system_timestamp': ParameterValue(LaunchConfiguration('use_system_timestamp'), value_type=bool),
            'range_min': ParameterValue(LaunchConfiguration('range_min'), value_type=float),
            'range_max': ParameterValue(LaunchConfiguration('range_max'), value_type=float),
            'cloud_scan_num': ParameterValue(LaunchConfiguration('cloud_scan_num'), value_type=int),
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
            'lidar_port': ParameterValue(LaunchConfiguration('lidar_port'), value_type=int),
            'lidar_ip': LaunchConfiguration('lidar_ip'),
            'local_port': ParameterValue(LaunchConfiguration('local_port'), value_type=int),
            'local_ip': LaunchConfiguration('local_ip'),
            'cloud_frame': LaunchConfiguration('cloud_frame'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'imu_frame': LaunchConfiguration('imu_frame'),
            'imu_topic': LaunchConfiguration('imu_topic'),
        }]
    )

    return LaunchDescription(launch_arguments + [node1])