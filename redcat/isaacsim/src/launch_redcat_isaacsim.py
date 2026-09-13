# SPDX-License-Identifier: BSD-3-Clause

"""
Modular, config-driven counterpart to launch_rtx_lidar_isaacsim.py (kept unmodified as a
single-file fallback): same behavior, but built from generic, reusable library modules --
rtx_lidar_bridge.RtxLidarBridge and diff_drive_bridge.{DiffDriveRobot, TeleopProcessManager,
TwistSubscriber} -- instead of one ~600-line robot-specific script. Those modules know nothing
about "redcat" specifically; every robot-specific value (joint names, mount path, wheel
geometry, ...) comes from a single YAML file instead, in the style of a ROS 2 parameters file:

    isaacsim:
      lidar: {type: Example_Rotary, elevation_range_deg: [90.0, 0.0], ...}
      diff_drive: {wheel_joint_names: [...], wheel_separation: 0.225, ...}
      ...

See cfg/redcat.yaml for the full schema and this robot's actual values. Point --config at a
different YAML to reuse this same script for another robot.

Package layout this script anchors itself to (see _ISAACSIM_ROOT/_PACKAGE_ROOT below): this
file lives in <package>/isaacsim/src/; cfg/ and include/ are its siblings under
<package>/isaacsim/; anything outside isaacsim/ (the URDF-exported robot USD) is resolved from
<package>/ instead, one level up. Porting this to another package/robot just means recreating
that same isaacsim/{src,cfg,include} layout.

Run via Isaac Sim's own Python, with ROS 2 sourced:

    /home/legion5/IsaacSim/python.sh \\
        /home/legion5/ros2_ws/src/rc_car/redcat/isaacsim/src/launch_redcat_isaacsim.py

rviz2: Fixed Frame = "odom" (or "map", once slam_toolbox/nav2 is running); add a RobotModel
display (needs robot_state_publisher running off /joint_states) and a PointCloud2 or LaserScan
display on /point_cloud or /scan, matching the configured lidar type.
"""

import argparse
import sys
from pathlib import Path

from isaacsim import SimulationApp

# This file is <package>/isaacsim/src/launch_redcat_isaacsim.py -- anchor every path off
# _ISAACSIM_ROOT (<package>/isaacsim) rather than counting parent hops, so moving/copying the
# whole isaacsim/ tree to another package keeps working unchanged.
_ISAACSIM_ROOT = Path(__file__).resolve().parent.parent
_PACKAGE_ROOT = _ISAACSIM_ROOT.parent

# include/ holds the generic library modules (rtx_lidar_bridge, diff_drive_bridge, sim_config);
# it's a sibling of src/, not on sys.path by default, so add it explicitly.
sys.path.insert(0, str(_ISAACSIM_ROOT / "include"))
from sim_config import load_config

# -- argparse ----------------------------------------------------------------
# Deliberately minimal: everything robot-specific lives in --config's YAML instead of flags.
parser = argparse.ArgumentParser(description="Spawn a robot in Isaac Sim from a YAML config, driven from /cmd_vel.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode (exit after 10 frames).")
parser.add_argument(
    "--config",
    type=str,
    default=None,
    help="Path to the isaacsim YAML config (default: cfg/redcat.yaml under this isaacsim/ root).",
)
args, _ = parser.parse_known_args()

config_path = Path(args.config) if args.config else _ISAACSIM_ROOT / "cfg" / "redcat.yaml"
cfg = load_config(config_path)

# Sets up an RTX lidar + ROS 2 publisher, matching the standalone rtx_lidar.py example's bringup.
simulation_app = SimulationApp({"headless": False})
import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path

# include/ modules -- only importable now, since they import carb/isaacsim internally.
from rtx_lidar_bridge import RtxLidarBridge
from diff_drive_bridge import DiffDriveRobot, TeleopProcessManager, TwistSubscriber
from joint_state_bridge import JointStateBridge

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
environment_usd = cfg["environment"]["usd"]
if environment_usd:
    stage_utils.add_reference_to_stage(assets_root_path + environment_usd, "/background")
    simulation_app.update()

# Spawn the robot.
robot_prim_path = cfg["robot"]["prim_path"]
robot_usd = _PACKAGE_ROOT / cfg["robot"]["usd_relative_path"]
stage_utils.add_reference_to_stage(str(robot_usd), robot_prim_path)
simulation_app.update()

# Author the lidar prim and apply elevation/channel-count settings, before setup_simulation().
lidar_cfg = cfg["lidar"]
lidar = RtxLidarBridge(
    prim_path=f"{robot_prim_path}/{lidar_cfg['mount_path']}",
    config=lidar_cfg["type"],
    frame_id=lidar_cfg["frame_id"],
    tick_rate=lidar_cfg.get("tick_rate", 10.0),
    translation=tuple(lidar_cfg.get("translation", (0.0, 0.0, 0.0))),
    elevation_range_deg=lidar_cfg.get("elevation_range_deg"),
    elevation_step_deg=lidar_cfg.get("elevation_step_deg"),
    debug_vis=lidar_cfg.get("debug_vis", True),
)
lidar.create()

# Build the /joint_states + TF bridge so robot_state_publisher can generate the robot's TF tree
# and rviz2 can see it move (odom -> base_link, driven off the articulation's own pose; odom is
# left as the tree's root, for slam_toolbox/nav2 to publish map -> odom on top of when running).
# Needs the exact PhysicsArticulationRootAPI prim (robot_prim_path itself is just the reference
# Xform) -- unlike DiffDriveRobot's Articulation wrapper, the physx tensor view this bridge's
# OmniGraph nodes query doesn't resolve a nested articulation root on its own.
jsb_cfg = cfg.get("joint_state_bridge", {})
articulation_root_path = f"{robot_prim_path}/{cfg['robot']['articulation_root_path']}"
joint_state_bridge = JointStateBridge(
    articulation_root_path,
    base_frame_id=jsb_cfg.get("base_frame_id", "base_link"),
    odom_frame_id=jsb_cfg.get("odom_frame_id", "odom"),
    joint_states_topic=jsb_cfg.get("joint_states_topic", "joint_states"),
    odom_topic=jsb_cfg.get("odom_topic", "odom"),
    tf_topic=jsb_cfg.get("tf_topic", "tf"),
    publish_odom=jsb_cfg.get("publish_odom", True),
)
joint_state_bridge.create()

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
simulation_app.update()

# Create the render product and attach the ROS 2 writer, now that the sim is set up.
lidar.attach_writers()
simulation_app.update()

app_utils.play()

# Wrap the robot's wheel joints for throttle-based diff-drive control (see DiffDriveRobot).
dd_cfg = cfg["diff_drive"]
robot = DiffDriveRobot(
    robot_prim_path,
    wheel_joint_names=dd_cfg["wheel_joint_names"],
    left_joint_names=dd_cfg["left_joint_names"],
    right_joint_names=dd_cfg["right_joint_names"],
    wheel_separation=dd_cfg["wheel_separation"],
    max_wheel_velocity=dd_cfg["max_wheel_velocity"],
    damping=dd_cfg.get("damping", 5.0),
    max_effort=dd_cfg.get("max_effort", 8.0),
)

lidar.describe()
joint_state_bridge.describe()

# Optionally start a /cmd_vel publisher ourselves instead of running one in another terminal.
teleop_cfg = cfg["teleop"]
teleop_procs = TeleopProcessManager(package_name=teleop_cfg.get("package_name"))
teleop_procs.start(
    teleop_cfg.get("mode", "none"),
    joy_device_id=teleop_cfg.get("joy_device_id"),
    joystick_params_path=teleop_cfg.get("joystick_params_path"),
)

# Always listen on the configured topic, regardless of who's publishing it.
teleop_topic = teleop_cfg.get("topic", "/cmd_vel")
teleop = TwistSubscriber(topic=teleop_topic)
print(f"[INFO] Teleop bridge listening on: {teleop_topic}")

frame_count = 0
try:
    while simulation_app.is_running():
        teleop.spin()
        robot.drive(teleop.linear_x, teleop.angular_z)

        simulation_app.update()
        frame_count += 1
        if args.test and frame_count >= 10:
            break
except KeyboardInterrupt:
    pass
finally:
    teleop.close()
    teleop_procs.stop()

# Cleanup and shutdown.
app_utils.stop()
simulation_app.close()
