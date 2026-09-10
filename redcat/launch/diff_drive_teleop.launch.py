import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'redcat'
    package_path = get_package_share_directory(package_name)

    HW_mode = LaunchConfiguration('HW_mode')

    # launch robot_state_publisher -- mirrors ros2_controllers.launch.py's own rsp inclusion.
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_path, 'launch', 'rsp.launch.py')]),
            launch_arguments={'HW_mode': HW_mode}.items()
    )

    controller_config = os.path.join(package_path, 'config', 'ros2_controllers_diff_drive.yaml')

    # run the ros2_control_node to handle controller spawning and loading.
    # remap topic to /robot_description, and diff_drive_controller's own /cmd_vel straight onto
    # joystick.launch.py's /cmd_vel_joy output below -- no twist_mux in this standalone pipeline
    # since there's only one command source here (unlike launch_robot.py's full stack).
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        remappings=[
            ("/controller_manager/robot_description", "robot_description"),
            ("/diff_drive_controller/cmd_vel", "/cmd_vel"),
        ],
        output="screen",
    )

    delay_controller_manager = TimerAction(
        period=1.0,
        actions=[controller_manager]
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    # === Spawn joint state broadcaster ===
    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    timed_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_controller_spawner],
        )
    )

    timed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broadcaster_spawner],
        )
    )

    # teleop_twist_joy pipeline -- joy_node + teleop_twist_joy, reusing joystick.launch.py as-is
    # (publishes geometry_msgs/TwistStamped on /cmd_vel_joy, per config/joystick.yaml).
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_path, 'launch', 'joystick.launch.py')])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'HW_mode',
            default_value='mock',
            description='Options : mock, real'),

        rsp,
        delay_controller_manager,
        timed_diff_drive_controller_spawner,
        timed_joint_broad_spawner,
        joystick,
    ])
