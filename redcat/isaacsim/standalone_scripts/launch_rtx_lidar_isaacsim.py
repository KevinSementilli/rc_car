# SPDX-License-Identifier: BSD-3-Clause

"""
Spawns the redcat RC car in Isaac Sim with an RTX lidar and a /cmd_vel-driven diff-drive bridge.

Lidar output follows --lidar_config: names ending in "2D" (e.g. Example_Rotary_2D) publish
sensor_msgs/LaserScan on /scan; everything else publishes sensor_msgs/PointCloud2 on
/point_cloud. Log messages and rviz2 instructions match whichever mode is active.

Elevation (angle from the prim's own +Z axis) defaults to matching the real Unitree L2: 3D
configs sweep from the horizontal plane (90 deg from +Z) up to straight up (0 deg from +Z, i.e.
elevation 0-90 deg above horizontal); 2D configs are pinned exactly to the horizontal plane (90
deg from +Z). Override with --elevation_range_deg.

Run via Isaac Sim's own Python, with ROS 2 sourced:

    /home/legion5/IsaacSim/python.sh /home/legion5/ros2_ws/src/rc_car/redcat/launch/play_lidar.py

rviz2: Fixed Frame = unitree_L2 (no odom/TF is published here); add a PointCloud2 display on
/point_cloud or a LaserScan display on /scan, matching the mode above.
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
        "RTX lidar config from SUPPORTED_LIDAR_CONFIGS. Names ending in '2D' publish LaserScan"
        " on /scan; everything else publishes PointCloud2 on /point_cloud."
    ),
)
parser.add_argument(
    "--elevation_range_deg",
    type=float,
    nargs=2,
    default=None,
    metavar=("MIN_DEG", "MAX_DEG"),
    help=(
        "Override the lidar's per-channel elevation angles (degrees above horizontal; 0 ="
        " horizontal, 90 = straight up along +Z). Defaults to '0 90' for 3D configs (hemisphere"
        " above the mount plane, matching the real Unitree L2) and '0 0' for 2D configs (pinned"
        " to the horizontal plane)."
    ),
)
parser.add_argument(
    "--elevation_step_deg",
    type=float,
    default=None,
    help=(
        "Regenerate the 3D lidar with evenly-spaced elevation planes at this angular spacing"
        " (degrees) across --elevation_range_deg, instead of the config's native channel count"
        " (smaller step = denser vertical sweep). Channel count is derived as"
        " round(range / step) + 1. Ignored for 2D configs, which are always a single plane."
    ),
)
parser.add_argument(
    "--environment_usd",
    type=str,
    default="/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    help="Background USD to load (relative to the assets root). Empty string for a bare ground plane.",
)
parser.add_argument(
    "--teleop",
    type=str,
    default="none",
    choices=["none", "joystick", "keyboard"],
    help=(
        "Optionally start a /cmd_vel publisher ourselves: 'joystick' (joy_node +"
        " teleop_twist_joy), 'keyboard' (teleop_twist_keyboard), or 'none' to drive from an"
        " externally-run publisher instead."
    ),
)
parser.add_argument(
    "--joy_device_id",
    type=int,
    default=None,
    help="joy_node SDL device index. Skips the interactive gamepad picker.",
)
parser.add_argument(
    "--teleop_topic",
    type=str,
    default="/cmd_vel",
    help="geometry_msgs/Twist topic to drive from (always subscribed).",
)
parser.add_argument(
    "--debug_vis",
    dest="debug_vis",
    action="store_true",
    default=True,
    help="Draw the lidar scan in the Isaac Sim viewport (default: on).",
)
args, _ = parser.parse_known_args()

# Sets up an RTX lidar + ROS 2 publisher, matching the standalone rtx_lidar.py example's bringup.
simulation_app = SimulationApp({"headless": False})
import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
from isaacsim.storage.native import get_assets_root_path

# Enable the ROS 2 bridge so the RtxLidar*ROS2Publish* writers are available.
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# Find the Isaac Sim assets root, needed to load the background environment.
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Load a background environment for the lidar to scan.
if args.environment_usd:
    stage_utils.add_reference_to_stage(assets_root_path + args.environment_usd, "/background")
    simulation_app.update()

# Path to the robot USD exported from the URDF (re-run the URDF importer to (re)create it).
_REDCAT_USD = Path(__file__).resolve().parent.parent.parent / "urdf" / "isaac_sim" / "redcat.usd"

# WHEEL_SEPARATION matches config/ros2_controllers_diff_drive.yaml (only the turn-mixing shape
# carries over here -- unlike the real diff_drive_controller, /cmd_vel is treated as a throttle
# fraction of MAX_WHEEL_VELOCITY below, not a physical m/s or rad/s value).
WHEEL_SEPARATION = 0.225
MAX_WHEEL_VELOCITY = 50.0
WHEEL_JOINT_NAMES = ["fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"]

# Spawn the redcat robot.
robot_prim_path = "/World/Robot"
stage_utils.add_reference_to_stage(str(_REDCAT_USD), robot_prim_path)
simulation_app.update()


def _read_laser_scan_metadata(prim: object) -> dict[str, float | list[float]]:
    """Read scan geometry from the lidar prim, needed to initialize the LaserScan writer."""
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


def _remap_lidar_elevation(prim: object, min_deg: float, max_deg: float) -> None:
    """Linearly rescale a rotary lidar's per-channel elevation angles into [min_deg, max_deg]
    (degrees above horizontal). Passing min_deg == max_deg pins every channel to that one
    elevation (used to force a 2D config flat to the horizontal plane).

    Each channel's elevation is baked into the profile as a fixed
    'omni:sensor:Core:emitterState:<channel>:elevationDeg' attribute (scalar or array,
    depending on the profile). A stock profile like Example_Rotary is symmetric about the
    horizon; this rescales whatever range it ships with into the requested one.
    """
    elevation_attrs = [
        attr for attr in prim.GetAttributes()
        if attr.GetName().startswith("omni:sensor:Core:emitterState:") and attr.GetName().endswith(":elevationDeg")
    ]
    if not elevation_attrs:
        carb.log_warn("No emitterState elevationDeg attributes found; --elevation_range_deg ignored.")
        return

    def _as_list(value: object) -> list[float]:
        return list(value) if hasattr(value, "__iter__") else [value]

    all_values = [v for attr in elevation_attrs for v in _as_list(attr.Get())]
    src_min, src_max = min(all_values), max(all_values)
    src_span = (src_max - src_min) or 1.0
    dst_span = max_deg - min_deg

    for attr in elevation_attrs:
        raw = attr.Get()
        remapped = [min_deg + (v - src_min) / src_span * dst_span for v in _as_list(raw)]
        attr.Set(remapped if hasattr(raw, "__iter__") else remapped[0])

    print(f"[INFO] Remapped lidar elevation from [{src_min:.1f}, {src_max:.1f}] deg to"
          f" [{min_deg}, {max_deg}] deg across {len(elevation_attrs)} channel(s).")


_OVERRIDDEN_EMITTER_SUFFIXES = (":elevationDeg", ":azimuthDeg", ":channelId", ":fireTimeNs")


def _resample(values: list, new_len: int) -> list:
    """Nearest-neighbor resample a list to new_len entries, preserving its existing values."""
    if len(values) == new_len:
        return list(values)
    if new_len == 1:
        return [values[0]]
    return [values[round(i * (len(values) - 1) / (new_len - 1))] for i in range(new_len)]


def _set_lidar_elevation_step(prim: object, step_deg: float, min_deg: float, max_deg: float) -> None:
    """Regenerate a rotary lidar's channels at a fixed angular spacing across [min_deg, max_deg]
    (degrees above horizontal) -- controls how densely it sweeps in elevation, expressed as the
    delta angle between adjacent planes rather than a raw channel count.

    Channel count is derived as round(range / step) + 1, then baked into
    'omni:sensor:Core:numberOfChannels' / 'numberOfEmitters'. Every 'emitterState:<channel>:*'
    array must share that same length: elevationDeg/azimuthDeg/channelId/fireTimeNs are
    regenerated fresh (every channel fires at azimuth offset 0 -- the ROTARY scan sweeps azimuth
    via the lidar's own rotation, so that alone covers 360 deg); everything else the profile
    ships (calibration/optics fields like distanceCorrectionM, focalDistM, ...) is nearest-
    neighbor resampled from its existing values instead, since we have no principled new values
    for those.
    """
    elevation_attr = next(
        (attr for attr in prim.GetAttributes()
         if attr.GetName().startswith("omni:sensor:Core:emitterState:") and attr.GetName().endswith(":elevationDeg")),
        None,
    )
    if elevation_attr is None:
        carb.log_warn("No emitterState elevationDeg attribute found; --elevation_step_deg ignored.")
        return
    prefix = elevation_attr.GetName().removesuffix(":elevationDeg")

    orig_elevations = elevation_attr.Get()
    orig_count = len(orig_elevations) if hasattr(orig_elevations, "__iter__") else 1

    num_channels = max(1, round(abs(max_deg - min_deg) / step_deg) + 1)
    elevations = (
        [min_deg] if num_channels == 1
        else [min_deg + i * (max_deg - min_deg) / (num_channels - 1) for i in range(num_channels)]
    )
    prim.GetAttribute("omni:sensor:Core:numberOfChannels").Set(num_channels)
    prim.GetAttribute("omni:sensor:Core:numberOfEmitters").Set(num_channels)
    prim.GetAttribute(f"{prefix}:elevationDeg").Set(elevations)
    prim.GetAttribute(f"{prefix}:azimuthDeg").Set([0.0] * num_channels)
    prim.GetAttribute(f"{prefix}:channelId").Set(list(range(1, num_channels + 1)))
    fire_time_attr = prim.GetAttribute(f"{prefix}:fireTimeNs")
    if fire_time_attr.IsValid():
        fire_time_attr.Set([0] * num_channels)

    # Resample every other per-channel array on this emitter state to the new length, so nothing
    # is left stale at the profile's original channel count.
    for attr in prim.GetAttributes():
        name = attr.GetName()
        if not name.startswith(f"{prefix}:") or name.endswith(_OVERRIDDEN_EMITTER_SUFFIXES):
            continue
        raw = attr.Get()
        if not hasattr(raw, "__iter__") or len(raw) != orig_count or len(raw) == 0:
            continue
        attr.Set(_resample(list(raw), num_channels))

    print(f"[INFO] Set lidar to {num_channels} channel(s) (~{step_deg} deg spacing),"
          f" elevation [{min_deg}, {max_deg}] deg above horizontal.")


# Configs ending in "2D" are single-plane lidars -> LaserScan; everything else -> PointCloud2.
is_2d_lidar = args.lidar_config.strip().lower().endswith("2d")

# tick_rate must match the config's own scan rate (10 Hz for Example_Rotary*).
lidar_path = f"{robot_prim_path}/Geometry/base_link/base_footprint/chassis/unitree_L2/RTXLidar"
lidar = Lidar.create(
    path=lidar_path,
    config=args.lidar_config,
    tick_rate=10.0,
    translations=[[0.0, 0.0, 0.01]],
)

# Default matches the real Unitree L2: 3D sweeps horizon-to-zenith (0-90 deg above horizontal);
# 2D is pinned flat to the horizontal plane (0 deg range, i.e. a single elevation).
if args.elevation_range_deg is not None:
    elevation_range_deg = tuple(args.elevation_range_deg)
elif is_2d_lidar:
    elevation_range_deg = (0.0, 0.0)
else:
    elevation_range_deg = (90.0, 0.0)

_lidar_prim = prim_utils.get_prim_at_path(lidar.paths[0])
if args.elevation_step_deg is not None and not is_2d_lidar:
    _set_lidar_elevation_step(_lidar_prim, args.elevation_step_deg, *elevation_range_deg)
else:
    if args.elevation_step_deg is not None:
        carb.log_warn("--elevation_step_deg is ignored for 2D lidar configs (always 1 plane).")
    _remap_lidar_elevation(_lidar_prim, *elevation_range_deg)

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
simulation_app.update()

# Wraps the lidar prim and creates the render product that writers attach to.
sensor = LidarSensor(lidar, annotators=[])

if is_2d_lidar:
    # 2D lidar -> LaserScan on /scan.
    laser_scan_meta = _read_laser_scan_metadata(prim_utils.get_prim_at_path(lidar.paths[0]))
    sensor.attach_writer(
        "RtxLidarROS2PublishLaserScan",
        topicName="scan",
        frameId="unitree_L2",
        **laser_scan_meta,
    )
else:
    # 3D lidar -> PointCloud2 on /point_cloud.
    sensor.attach_writer(
        "RtxLidarROS2PublishPointCloud",
        topicName="point_cloud",
        frameId="unitree_L2",
    )

# Draw the scan in the viewport (RGBA in [0, 1]).
if args.debug_vis:
    sensor.attach_writer(
        "draw-point-cloud",
        color=[0.0, 1.0, 0.5, 1.0],  # bright green
        size=0.05,
    )

simulation_app.update()

app_utils.play()

# Get wheel joint indices for velocity control.
robot = Articulation(robot_prim_path)
left_ids = robot.get_dof_indices(["fl_wheel_joint", "rl_wheel_joint"])
right_ids = robot.get_dof_indices(["fr_wheel_joint", "rr_wheel_joint"])
wheel_ids = robot.get_dof_indices(WHEEL_JOINT_NAMES)

# Velocity control: zero stiffness + damping only (the USD asset ships with zero gains, so
# driving does nothing until these are set).
robot.set_dof_gains(stiffnesses=0.0, dampings=5.0, dof_indices=wheel_ids)
robot.set_dof_max_efforts(8.0, dof_indices=wheel_ids)
robot.set_dof_max_velocities(MAX_WHEEL_VELOCITY, dof_indices=wheel_ids)


class _TwistSubscriber:
    """Minimal /cmd_vel subscriber, storing (linear_x, angular_z).

    Must be built after isaacsim.ros2.bridge is enabled -- rclpy here is the Omniverse build
    that extension provides, not the system rclpy.
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
    """Convert (v, w) to per-side wheel angular velocity, treating both as throttle fractions
    (nominally -1..1, matching config/joystick.yaml's scale_linear/scale_angular) rather than
    physical m/s or rad/s -- unlike the real diff_drive_controller, which always interprets
    /cmd_vel as physical SI velocity and can't be reconfigured to mean throttle percentage.
    A throttle of +-1.0 on one side alone scales to +-MAX_WHEEL_VELOCITY, matching the same limit
    passed to set_dof_max_velocities() above.
    """
    left = (linear_x - angular_z * WHEEL_SEPARATION / 2.0) * MAX_WHEEL_VELOCITY
    right = (linear_x + angular_z * WHEEL_SEPARATION / 2.0) * MAX_WHEEL_VELOCITY
    return left, right


def _redcat_package_share_dir() -> str:
    """Resolve redcat's share dir via `ros2 pkg prefix` (this interpreter is Isaac Sim's own,
    so ament_index_python isn't guaranteed importable, but the ros2 CLI is).
    """
    prefix = subprocess.check_output(["ros2", "pkg", "prefix", "redcat"], text=True).strip()
    return os.path.join(prefix, "share", "redcat")


def _list_joystick_devices() -> list[tuple[int, str]]:
    """Parse `joy_enumerate_devices`'s table into (device_id, name) pairs."""
    output = subprocess.check_output(["ros2", "run", "joy", "joy_enumerate_devices"], text=True, timeout=10)
    devices = []
    for line in output.splitlines():
        parts = [p.strip() for p in line.split(":")]
        if len(parts) >= 5 and parts[0].isdigit():
            devices.append((int(parts[0]), parts[-1]))
    return devices


def _select_joystick_device(requested_id: int | None) -> int:
    """Pick a gamepad device_id, prompting interactively if more than one is connected."""
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
    """Start the requested teleop pipeline as subprocesses, publishing /cmd_vel."""
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
        # Reads this terminal's stdin directly (termios), not a rclpy subscription.
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


if is_2d_lidar:
    print(f"[INFO] RTX lidar ({args.lidar_config}) publishing LaserScan on /scan")
    print("[INFO] In rviz2: set Fixed Frame to 'unitree_L2' and add a LaserScan display on /scan.")
else:
    print(f"[INFO] RTX lidar ({args.lidar_config}) publishing PointCloud2 on /point_cloud")
    print("[INFO] In rviz2: set Fixed Frame to 'unitree_L2' and add a PointCloud2 display on /point_cloud.")

# Optionally start a /cmd_vel publisher ourselves instead of running one in another terminal.
teleop_procs = _start_teleop(args.teleop, args.joy_device_id)

# Always listen on --teleop_topic, regardless of who's publishing it.
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

# Cleanup and shutdown.
app_utils.stop()
simulation_app.close()
