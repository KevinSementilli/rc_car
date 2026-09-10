# SPDX-License-Identifier: BSD-3-Clause

"""
Standalone script: spawns the redcat RC car, a background scene, an RTX lidar mounted on its
chassis publishing ``sensor_msgs/PointCloud2`` on ``/point_cloud`` over ROS 2, and a ``/cmd_vel``
teleop bridge that drives it. Always drives from ``/cmd_vel`` (subscribed unconditionally);
``--teleop {joystick,keyboard}`` optionally has the script start that publisher itself.

Uses the plain ``isaacsim.core.experimental`` API directly (no IsaacLab) -- the lidar creation
and ROS 2 publish sequence below is a literal copy of the standalone
``isaacsim.ros2.bridge/rtx_lidar.py`` example's own bringup (``Lidar.create()`` ->
``SimulationManager.setup_simulation()`` -> ``LidarSensor()`` -> ``attach_writer(...)``), with the
lidar mounted on the robot's chassis instead of floating in place, plus the robot prim itself and
the ``/cmd_vel`` diff-drive teleop bridge added on top.

Run directly through Isaac Sim's own Python, with ROS 2 already sourced (rclpy + the ``ros2`` CLI
must be on PATH):

    /home/legion5/IsaacSim/python.sh /home/legion5/ros2_ws/src/rc_car/redcat/launch/play_lidar.py

In rviz2: set Fixed Frame to ``unitree_L2`` directly (no odom/TF is published here, so that's
the only frame that exists) and add a PointCloud2 display on ``/point_cloud``.
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

from isaacsim import SimulationApp

# -- argparse ----------------------------------------------------------------
parser = argparse.ArgumentParser(description="Spawn the redcat RC car with an RTX lidar, driven from /cmd_vel.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument(
    "--lidar_config",
    type=str,
    default="Example_Rotary",
    help=(
        "RTX lidar sensor config name (from isaacsim.sensors.experimental.rtx's"
        " SUPPORTED_LIDAR_CONFIGS) -- NVIDIA's Example_Rotary is a 3D point cloud config."
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
    "--teleop",
    type=str,
    default="none",
    choices=["none", "joystick", "keyboard"],
    help=(
        "Optionally have this script itself start a /cmd_vel publisher: 'joystick' starts"
        " joy_node + teleop_twist_joy (prompting to pick a gamepad if more than one is"
        " connected, unless --joy_device_id is given); 'keyboard' starts teleop_twist_keyboard"
        " in this terminal; 'none' (default) starts nothing -- the script still always listens"
        " on --teleop_topic, so run joystick.launch.py (or anything else publishing Twist) in"
        " another terminal instead if you'd rather not have this script manage that process."
    ),
)
parser.add_argument(
    "--joy_device_id",
    type=int,
    default=None,
    help="joy_node 'device_id' (SDL joystick index) to use for --teleop joystick. Skips the interactive prompt.",
)
parser.add_argument(
    "--teleop_topic",
    type=str,
    default="/cmd_vel",
    help="ROS 2 geometry_msgs/Twist topic to drive the robot from (always subscribed, regardless of --teleop).",
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
# URDF through Isaac Sim's URDF Importer to (re)create this path.
_REDCAT_USD = Path(__file__).resolve().parent.parent / "urdf" / "isaac_sim" / "redcat.usd"

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

# Create the 3D rotating RTX Lidar on the robot's chassis. Example_Rotary scans at 10 Hz, so
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

# RTX lidar -> ROS 2 PointCloud2 publisher.
sensor.attach_writer(
    "RtxLidarROS2PublishPointCloud",
    topicName="point_cloud",
    frameId="unitree_L2",
)

# Visualize the point cloud in the viewport (RGBA in [0, 1]).
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


def _twist_to_diff_drive(linear_x: float, angular_z: float) -> tuple[float, float]:
    """Convert ``(v, w)`` to ``(left-side, right-side)`` wheel angular velocities, matching this
    package's own ``diff_drive_controller`` (``diff_drive_controller/DiffDriveController``,
    ``config/ros2_controllers_diff_drive.yaml``): each side's linear speed is
    ``v -+ w * wheel_separation / 2``, converted to angular velocity via ``/ wheel_radius``.
    """
    left = (linear_x - angular_z * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
    right = (linear_x + angular_z * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
    return left, right


def _redcat_package_share_dir() -> str:
    """Resolve the ``redcat`` ROS 2 package's share directory (holds ``config/joystick.yaml``)
    via ``ros2 pkg prefix`` rather than importing ``ament_index_python`` directly -- this
    script runs under Isaac Sim's own Python interpreter, a different one than the system ROS 2
    install, so ament_index_python isn't guaranteed importable in-process, but the ``ros2`` CLI
    is (it must already be on PATH for isaacsim.ros2.bridge/rclpy to work at all).
    """
    prefix = subprocess.check_output(["ros2", "pkg", "prefix", "redcat"], text=True).strip()
    return os.path.join(prefix, "share", "redcat")


def _list_joystick_devices() -> list[tuple[int, str]]:
    """Parse ``ros2 run joy joy_enumerate_devices``'s table into (device_id, name) pairs --
    ``device_id`` is exactly the SDL joystick index ``joy_node``'s ``device_id`` param expects.
    """
    output = subprocess.check_output(["ros2", "run", "joy", "joy_enumerate_devices"], text=True, timeout=10)
    devices = []
    for line in output.splitlines():
        parts = [p.strip() for p in line.split(":")]
        if len(parts) >= 5 and parts[0].isdigit():
            devices.append((int(parts[0]), parts[-1]))
    return devices


def _select_joystick_device(requested_id: int | None) -> int:
    """Pick which physical gamepad ``joy_node`` should open, prompting interactively when more
    than one is plugged in and ``--joy_device_id`` wasn't given.
    """
    if requested_id is not None:
        return requested_id

    try:
        devices = _list_joystick_devices()
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired) as exc:
        print(f"[WARNING] Could not enumerate joystick devices ({exc}); defaulting to device_id=0.")
        return 0

    if not devices:
        print("[WARNING] No joystick devices detected; defaulting to device_id=0.")
        return 0
    if len(devices) == 1:
        device_id, name = devices[0]
        print(f"[INFO] Using the only detected gamepad: [{device_id}] {name}")
        return device_id

    print("[INFO] Multiple gamepads detected:")
    for device_id, name in devices:
        print(f"  [{device_id}] {name}")
    valid_ids = {device_id for device_id, _ in devices}
    while True:
        choice = input(f"Select a gamepad device_id ({', '.join(str(d) for d in sorted(valid_ids))}): ").strip()
        if choice.isdigit() and int(choice) in valid_ids:
            return int(choice)
        print("[WARNING] Invalid selection, try again.")


def _start_teleop(mode: str, joy_device_id: int | None) -> list[subprocess.Popen]:
    """Launch the requested teleop input pipeline as ROS 2 node subprocesses, publishing
    ``geometry_msgs/Twist`` on ``/cmd_vel`` for ``_TwistSubscriber`` to read.

    For 'joystick', runs ``joy_node`` + ``teleop_twist_joy`` directly with redcat's own
    ``config/joystick.yaml`` rather than via ``ros2 launch redcat joystick.launch.py``, so a
    physical gamepad can be picked interactively first -- and so two of that launch file's
    choices, made for the real car's twist_mux pipeline, don't leak in here: its remap of
    ``/cmd_vel`` to ``/cmd_vel_joy`` (skipped simply by not applying it, since teleop_node is
    invoked directly) and ``publish_stamped_twist: true`` in the yaml itself (overridden back
    to plain ``Twist`` below, to match ``_TwistSubscriber``).
    """
    if mode == "none":
        return []

    if mode == "joystick":
        joy_params = os.path.join(_redcat_package_share_dir(), "config", "joystick.yaml")
        device_id = _select_joystick_device(joy_device_id)
        print(f"[INFO] Starting joy_node (device_id={device_id}) + teleop_twist_joy -> /cmd_vel")
        print("[INFO] Hold the enable button (see config/joystick.yaml's enable_button) while moving the stick.")
        return [
            subprocess.Popen([
                "ros2", "run", "joy", "joy_node",
                "--ros-args", "--params-file", joy_params, "-p", f"device_id:={device_id}",
            ]),
            subprocess.Popen([
                "ros2", "run", "teleop_twist_joy", "teleop_node",
                "--ros-args", "--params-file", joy_params, "-p", "publish_stamped_twist:=false",
            ]),
        ]

    if mode == "keyboard":
        print("[INFO] Starting teleop_twist_keyboard -> /cmd_vel")
        print("[INFO] Keep this terminal focused -- Isaac Sim log lines will interleave with its on-screen keys.")
        # Inherits this process's stdin/stdout so it can read keypresses from and print its key
        # legend to this same terminal -- it uses termios directly, not a rclpy subscription,
        # so it can't be driven any other way.
        return [subprocess.Popen(["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"])]

    raise ValueError(f"Unknown --teleop mode: {mode}")


def _stop_teleop(procs: list[subprocess.Popen]) -> None:
    for proc in procs:
        proc.terminate()
    for proc in procs:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


print(f"[INFO] RTX lidar ({args.lidar_config}) publishing PointCloud2 on /point_cloud")
print("[INFO] In rviz2: set Fixed Frame to 'unitree_L2' and add a PointCloud2 display on /point_cloud.")

# optional: have this script start the requested input pipeline's ROS 2 node(s)
# (joy_node+teleop_twist_joy, or teleop_twist_keyboard) publishing /cmd_vel itself, instead
# of running joystick.launch.py (or anything else) in another terminal.
teleop_procs = _start_teleop(args.teleop, args.joy_device_id)

# ROS 2 teleop bridge -- always listens on args.teleop_topic, regardless of whether this script
# or something else is publishing that topic.
teleop = _TwistSubscriber(topic=args.teleop_topic)
print(f"[INFO] Teleop bridge listening on: {args.teleop_topic}")

frame_count = 0
try:
    while simulation_app.is_running():
        teleop.spin()
        left_ang_vel, right_ang_vel = _twist_to_diff_drive(teleop.linear_x, teleop.angular_z)
        robot.set_dof_velocity_targets(left_ang_vel, dof_indices=left_ids)
        robot.set_dof_velocity_targets(right_ang_vel, dof_indices=right_ids)

        simulation_app.update()
        frame_count += 1
        if args.test and frame_count >= 10:
            break
except KeyboardInterrupt:
    pass
finally:
    teleop.close()
    _stop_teleop(teleop_procs)

# cleanup and shutdown
app_utils.stop()
simulation_app.close()
