# SPDX-License-Identifier: BSD-3-Clause

"""
Standalone script: spawns the redcat RC car, a background scene, a 2D RTX lidar mounted on its
chassis publishing ``sensor_msgs/LaserScan`` directly on ``/scan`` over ROS 2, a ``/cmd_vel``
teleop bridge that drives it (diff-drive kinematics computed in this script, same as the plain
``launch_rtx_lidar_isaacsim.py``), and a ground-truth ``/odom`` + ``odom`` -> ``base_link`` tf2
bridge for slam_toolbox to consume.

This script only does what needs live access to the running Isaac Sim process (spawning the
robot/lidar, driving the wheels, and reading back ground-truth pose for odometry). It does not
launch any other ROS 2 nodes itself -- run these in separate terminals so each piece can be
started, restarted, and debugged independently:

  - ``ros2 launch redcat rsp.launch.py HW_mode:=mock`` -- robot_state_publisher, for the URDF's
    static ``base_link`` -> ``base_footprint`` -> ``chassis`` -> ``unitree_L2`` chain.
  - ``ros2 launch redcat online_async_launch.py use_sim_time:=false`` -- slam_toolbox itself
    (``config/mapper_params_online_async.yaml`` already matches this script's frames/topics:
    ``odom_frame: odom`` / ``base_frame: base_footprint`` / ``scan_topic: /scan``).
  - Whatever publishes ``/cmd_vel``: ``ros2 launch redcat joystick.launch.py``,
    ``ros2 run teleop_twist_keyboard teleop_twist_keyboard``, or similar.
  - ``rviz2`` for visualization.

``use_sim_time:=false`` above because this script publishes no ``/clock`` -- everything here runs
on wall-clock stamps. Driving the sim through a real ros2_control ``diff_drive_controller`` was
considered and dropped: controller_manager and Isaac Sim are separate processes, so it would need
a custom hardware_interface plugin bridging them over ROS 2 topics -- more complexity than this
sim integration needs right now.

Uses the plain ``isaacsim.core.experimental`` API directly (no IsaacLab) -- the lidar creation
and ROS 2 publish sequence below is a literal copy of the standalone
``isaacsim.ros2.bridge/rtx_lidar.py`` example's own 2D-lidar bringup (``Lidar.create()`` ->
``SimulationManager.setup_simulation()`` -> ``LidarSensor()`` -> ``attach_writer(...)``, plus that
same example's ``_read_laser_scan_metadata`` helper), with the lidar mounted on the robot's
chassis instead of floating in place, plus the robot prim itself and the ``/cmd_vel`` diff-drive
teleop bridge added on top.

Run directly through Isaac Sim's own Python, with ROS 2 already sourced (rclpy + the ``ros2`` CLI
must be on PATH):

    /home/legion5/IsaacSim/python.sh \\
        /home/legion5/ros2_ws/src/rc_car/redcat/launch/isaacsim/launch_slam_isaacsim.py

In rviz2: set Fixed Frame to ``map`` and add displays for ``/map`` (Map), ``/scan`` (LaserScan),
and TF.
"""

import argparse
import sys
from pathlib import Path

from isaacsim import SimulationApp

# -- argparse ----------------------------------------------------------------
parser = argparse.ArgumentParser(description="Spawn the redcat RC car with an RTX lidar, driven from /cmd_vel.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument(
    "--lidar_config",
    type=str,
    default="Example_Rotary_2D",
    help=(
        "RTX lidar sensor config name (from isaacsim.sensors.experimental.rtx's"
        " SUPPORTED_LIDAR_CONFIGS) -- NVIDIA's Example_Rotary_2D is a single-plane 2D scan"
        " config, published straight to /scan as sensor_msgs/LaserScan."
    ),
)
parser.add_argument(
    "--environment_usd",
    type=str,
    default="/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    help=(
        "USD path (relative to the Isaac Sim assets root) for the background environment"
        " (walls/obstacles) to spawn, so the lidar has something to scan. Pass an empty string"
        " to skip it and use the bare ground plane."
    ),
)
parser.add_argument(
    "--teleop_topic",
    type=str,
    default="/cmd_vel",
    help=(
        "ROS 2 geometry_msgs/Twist topic to drive the robot from. This script only subscribes --"
        " run joystick.launch.py, teleop_twist_keyboard, or anything else publishing Twist in a"
        " separate terminal to actually drive it."
    ),
)
parser.add_argument(
    "--debug_vis",
    dest="debug_vis",
    action="store_true",
    default=True,
    help="Draw the RTX lidar's point cloud directly in the Isaac Sim viewport (default: on).",
)
parser.add_argument(
    "--no_debug_vis",
    dest="debug_vis",
    action="store_false",
    help="Disable --debug_vis.",
)
args, _ = parser.parse_known_args()

# Example for creating an RTX lidar sensor and publishing PointCloud2 data, mounted on the redcat
# robot -- matches the standalone isaacsim.ros2.bridge/rtx_lidar.py example's own bringup exactly.
simulation_app = SimulationApp({"headless": False})
import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
from isaacsim.storage.native import get_assets_root_path

# Enable the ROS 2 bridge so the RtxLidar*ROS2Publish* writers are registered.
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# Locate Isaac Sim assets folder to load the background environment
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the background environment (walls/obstacles for the lidar to see)
if args.environment_usd:
    stage_utils.add_reference_to_stage(assets_root_path + args.environment_usd, "/background")
    simulation_app.update()

# Relative to this file so the repo works on any machine. Re-export robot_core_diff.xacro's
# URDF through Isaac Sim's URDF Importer to (re)create this path. Three levels up: this file
# lives in redcat/launch/isaacsim/, so parent.parent.parent is the redcat package root.
_REDCAT_USD = Path(__file__).resolve().parent.parent.parent / "urdf" / "isaac_sim" / "redcat.usd"

# Matches config/ros2_controllers_diff_drive.yaml's `diff_drive_controller` exactly, so this
# script's driving kinematics track what the real car's ros2_control setup expects from the
# same /cmd_vel input.
WHEEL_SEPARATION = 0.225
WHEEL_RADIUS = 0.0575
WHEEL_JOINT_NAMES = ["fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"]

# Spawn the redcat robot.
robot_prim_path = "/World/Robot"
stage_utils.add_reference_to_stage(str(_REDCAT_USD), robot_prim_path)
simulation_app.update()


def _read_laser_scan_metadata(prim: object) -> dict[str, float | list[float]]:
    """Read scan configuration from a rotary OmniLidar prim for LaserScan publishing.

    Copied from the standalone ``isaacsim.ros2.bridge/rtx_lidar.py`` example: mirrors the
    metadata extraction performed by ``OgnROS2RtxLidarHelper`` so the LaserScan writer can be
    initialized directly from the prim authored by ``Lidar.create()``.
    """
    rotation_rate = float(prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
    near_range = float(prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
    far_range = float(prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)
    firing_rate = int(prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
    if rotation_rate <= 0 or firing_rate <= 0:
        raise RuntimeError("LaserScan: scanRateBaseHz or patternFiringRateHz is 0 on the lidar prim")
    return {
        "horizontalFov": 360.0,
        "horizontalResolution": 360.0 * rotation_rate / firing_rate,
        "depthRange": [near_range, far_range],
        "rotationRate": rotation_rate,
        "azimuthRange": [-180.0, 180.0],
    }


# Create the 2D rotating RTX Lidar on the robot's chassis. Example_Rotary_2D scans at 10 Hz, so
# tick_rate must be 10 (see isaac_sim_sensors_multitick_lidar_tickrate_must_match_scanrate).
lidar_path = f"{robot_prim_path}/Geometry/base_link/base_footprint/chassis/unitree_L2/RTXLidar"
lidar = Lidar.create(
    path=lidar_path,
    config=args.lidar_config,
    tick_rate=10.0,
    translations=[[0.0, 0.0, 0.0]],
)

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
simulation_app.update()

# LidarSensor wraps the authoring prim, creates the render product, and exposes
# attach_writer() for both registered short names ("draw-point-cloud") and any
# Replicator writer registry name (the ROS 2 publisher).
sensor = LidarSensor(lidar, annotators=[])

# RTX lidar -> ROS 2 LaserScan publisher, straight to /scan (no PointCloud2, no
# pointcloud_to_laserscan conversion step -- slam_toolbox's scan_topic: /scan matches directly).
laser_scan_meta = _read_laser_scan_metadata(prim_utils.get_prim_at_path(lidar.paths[0]))
sensor.attach_writer(
    "RtxLidarROS2PublishLaserScan",
    topicName="scan",
    frameId="unitree_L2",
    **laser_scan_meta,
)

# Visualize the scan in the viewport (RGBA in [0, 1]).
if args.debug_vis:
    sensor.attach_writer(
        "draw-point-cloud",
        color=[0.0, 1.0, 0.5, 1.0],  # bright green
        size=0.05,
    )

simulation_app.update()

app_utils.play()

# Wrap the robot articulation for joint control.
robot = Articulation(robot_prim_path)
left_ids = robot.get_dof_indices(["fl_wheel_joint", "rl_wheel_joint"])
right_ids = robot.get_dof_indices(["fr_wheel_joint", "rr_wheel_joint"])
wheel_ids = robot.get_dof_indices(WHEEL_JOINT_NAMES)

# All four wheels are independently velocity-controlled joints -- zero stiffness + non-zero
# damping is pure velocity control (see Articulation.set_dof_gains's own hint), matching the
# redcat.usd source asset's own drive schema (authored with zero gains, so driving does nothing
# until these are set explicitly).
robot.set_dof_gains(stiffnesses=0.0, dampings=5.0, dof_indices=wheel_ids)
robot.set_dof_max_efforts(8.0, dof_indices=wheel_ids)
robot.set_dof_max_velocities(100.0, dof_indices=wheel_ids)


class _TwistSubscriber:
    """Minimal ROS 2 ``geometry_msgs/Twist`` subscriber, storing ``(linear_x, angular_z)``
    directly.

    Must be constructed after the ``isaacsim.ros2.bridge`` extension has been enabled (which
    itself requires a live ``SimulationApp``), since ``rclpy`` here is the Omniverse-compiled
    build provided by that extension, not the system ``rclpy``.
    """

    def __init__(self, topic: str = "/cmd_vel"):
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node

        self.linear_x = 0.0
        self.angular_z = 0.0

        if not rclpy.ok():
            rclpy.init()

        outer = self

        class _Sub(Node):
            def __init__(self) -> None:
                super().__init__("redcat_teleop_bridge")
                self.create_subscription(Twist, topic, self._on_twist, 10)

            def _on_twist(self, msg: Twist) -> None:
                outer.linear_x = msg.linear.x
                outer.angular_z = msg.angular.z

        self._node = _Sub()

    def spin(self) -> None:
        import rclpy

        rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        self._node.destroy_node()


class _OdomTfPublisher:
    """Publishes the robot Articulation's ground-truth root pose as ``nav_msgs/Odometry`` on
    ``/odom`` and broadcasts the matching ``odom`` -> ``base_link`` tf2 transform -- the only
    odometry input slam_toolbox has to seed scan matching from. ``base_link`` is this URDF's tf
    root (``robot_core_diff.xacro``'s ``base_link_joint`` has ``base_link`` as parent), so this is
    the one dynamic edge needed to connect it to ``odom``; slam_toolbox's own ``base_frame:
    base_footprint`` (a static child of ``base_link`` published by robot_state_publisher) is then
    reached by chaining through it.

    Uses simulation ground truth directly rather than integrating the commanded wheel velocities,
    since the robot spawns at the world origin -- exactly where the ``odom`` frame's origin
    belongs -- so no drift/noise model is needed for this to work.

    Must be constructed after ``isaacsim.ros2.bridge`` is enabled, same as ``_TwistSubscriber``.
    """

    def __init__(self, robot: Articulation, twist_source: "_TwistSubscriber", odom_frame: str = "odom",
                 base_frame: str = "base_link"):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from tf2_ros import TransformBroadcaster

        self._robot = robot
        self._twist_source = twist_source
        self._odom_frame = odom_frame
        self._base_frame = base_frame
        self._Odometry = Odometry
        self._TransformStamped = TransformStamped

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("redcat_odom_tf_bridge")
        self._odom_pub = self._node.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self._node)

    def publish(self) -> None:
        positions, orientations = self._robot.get_world_poses()
        px, py, pz = positions.numpy()[0].tolist()
        qw, qx, qy, qz = orientations.numpy()[0].tolist()  # Isaac Sim quaternions are wxyz.
        stamp = self._node.get_clock().now().to_msg()

        odom_msg = self._Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame
        odom_msg.pose.pose.position.x = px
        odom_msg.pose.pose.position.y = py
        odom_msg.pose.pose.position.z = pz
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = self._twist_source.linear_x
        odom_msg.twist.twist.angular.z = self._twist_source.angular_z
        self._odom_pub.publish(odom_msg)

        tf_msg = self._TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self._odom_frame
        tf_msg.child_frame_id = self._base_frame
        tf_msg.transform.translation.x = px
        tf_msg.transform.translation.y = py
        tf_msg.transform.translation.z = pz
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)

    def close(self) -> None:
        self._node.destroy_node()


def _twist_to_diff_drive(linear_x: float, angular_z: float) -> tuple[float, float]:
    """Convert ``(v, w)`` to ``(left-side, right-side)`` wheel angular velocities, matching this
    package's own ``diff_drive_controller`` (``diff_drive_controller/DiffDriveController``,
    ``config/ros2_controllers_diff_drive.yaml``): each side's linear speed is
    ``v -+ w * wheel_separation / 2``, converted to angular velocity via ``/ wheel_radius``.
    """
    left = (linear_x - angular_z * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
    right = (linear_x + angular_z * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
    return left, right


print(f"[INFO] RTX lidar ({args.lidar_config}) publishing LaserScan on /scan")
print("[INFO] In rviz2: set Fixed Frame to 'map' and add displays for /map (Map), /scan (LaserScan), and TF.")
print("[INFO] This script launches no other ROS 2 nodes -- start robot_state_publisher,")
print("[INFO] slam_toolbox, and a /cmd_vel source separately (see this file's module docstring).")

# ROS 2 teleop bridge -- listens on args.teleop_topic; run a separate /cmd_vel publisher
# (joystick.launch.py, teleop_twist_keyboard, ...) in another terminal to actually drive it.
teleop = _TwistSubscriber(topic=args.teleop_topic)
print(f"[INFO] Teleop bridge listening on: {args.teleop_topic}")

# odom->base_link ground-truth odometry/TF bridge, feeding whatever slam_toolbox instance you
# start separately.
odom_tf = _OdomTfPublisher(robot, teleop)

frame_count = 0
try:
    while simulation_app.is_running():
        teleop.spin()
        left_ang_vel, right_ang_vel = _twist_to_diff_drive(teleop.linear_x, teleop.angular_z)
        robot.set_dof_velocity_targets(left_ang_vel, dof_indices=left_ids)
        robot.set_dof_velocity_targets(right_ang_vel, dof_indices=right_ids)

        odom_tf.publish()

        simulation_app.update()
        frame_count += 1
        if args.test and frame_count >= 10:
            break
except KeyboardInterrupt:
    pass
finally:
    teleop.close()
    odom_tf.close()

# cleanup and shutdown
app_utils.stop()
simulation_app.close()
