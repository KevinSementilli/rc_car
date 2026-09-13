import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory('redcat')

    # isaacsim/ is run by Isaac Sim's own Python, 
    # source tree is the only place it can be found
    package_source_root = Path(os.path.realpath(__file__)).parent.parent

    HW_mode = LaunchConfiguration('HW_mode')

    rviz_config = os.path.join(package_path, 'config', 'rviz', 'isaacsim_cfg.rviz')
    isaacsim_python = '/home/legion5/IsaacSim/python.sh'
    isaacsim_config = str(package_source_root / 'isaacsim' / 'cfg' / 'redcat.yaml')

    # Launch Isaacsim with ros2 bridge system configuration
    isaacsim = ExecuteProcess(
        cmd=[
            isaacsim_python,
            str(package_source_root / 'isaacsim' / 'src' / 'launch_redcat_isaacsim.py'),
            '--config', isaacsim_config,
        ],
        output='screen',
    )

    #  laucnh robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(package_path, 'launch', 'include', 'rsp.launch.py')]),
        launch_arguments={'HW_mode': HW_mode}.items(),
    )

    # launch joystick teleop node
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(package_path, 'launch', 'include', 'joystick.launch.py')]),
    )

    # launch rviz2 with a pre-configured view of the robot and lidar
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'HW_mode',
            default_value='mock',
            description='Options : mock, real'),

        isaacsim,
        rsp,
        joystick,
        rviz,
    ])
