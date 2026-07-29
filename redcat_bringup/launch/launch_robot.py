import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():

    package_name = 'redcat_bringup'
    package_path = get_package_share_directory(package_name)

    HW_mode = LaunchConfiguration('HW_mode')
    initialize_type = LaunchConfiguration('initialize_type')
    work_mode = LaunchConfiguration('work_mode')
    lidar_ip = LaunchConfiguration('lidar_ip')
    local_ip = LaunchConfiguration('local_ip')

    # launch ros2_control_node to handle controller spawning and loading
    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_path, 'launch' ,'ros2_controllers.launch.py')]), 
            launch_arguments={'HW_mode': HW_mode}.items()
    )

    # launch lidar node
    lidar_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_path, 'launch' ,'lidar.launch.py')]), 
        launch_arguments={
            'initialize_type': initialize_type,
            'work_mode': work_mode,
            'lidar_ip': lidar_ip,
            'local_ip': local_ip,
        }.items(),
    )

    rviz_config_file = os.path.join(package_path, 'config', 'robot.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'HW_mode',
            default_value='mock',
            description='Options : mock, real'),
        DeclareLaunchArgument('initialize_type', default_value='2'),
        DeclareLaunchArgument('work_mode', default_value='0'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.0.154'),
        DeclareLaunchArgument('local_ip', default_value='192.168.0.10'),

        ros2_control,
        lidar_spawner,
        rviz
    ])