# SPDX-License-Identifier: BSD-3-Clause

"""Generic teleop input handling and diff-drive wheel control for Isaac Sim robots:

- TwistSubscriber: minimal /cmd_vel listener.
- TeleopProcessManager: optionally starts joy_node/teleop_twist_joy or teleop_twist_keyboard.
- DiffDriveMixer / DiffDriveRobot: throttle-based (not physical-unit) diff-drive wheel control.

Not tied to any specific robot -- DiffDriveRobot takes wheel joint names and geometry as
required arguments rather than defaulting to any one robot's values.

Must only be imported after a SimulationApp has been constructed -- its own imports below need
Isaac Sim's runtime to already exist.
"""

import os
import subprocess

from isaacsim.core.experimental.prims import Articulation


class TwistSubscriber:
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
                super().__init__("isaacsim_teleop_bridge")
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


class DiffDriveMixer:
    """Converts (linear_x, angular_z) throttle fractions (nominally -1..1) into per-side wheel
    angular velocity -- unlike the real diff_drive_controller, which always interprets /cmd_vel
    as physical SI velocity and can't be reconfigured to mean throttle percentage. A throttle of
    +-1.0 on one side alone scales to +-max_wheel_velocity.
    """

    def __init__(self, wheel_separation: float, max_wheel_velocity: float):
        self.wheel_separation = wheel_separation
        self.max_wheel_velocity = max_wheel_velocity

    def mix(self, linear_x: float, angular_z: float) -> tuple[float, float]:
        left = (linear_x - angular_z * self.wheel_separation / 2.0) * self.max_wheel_velocity
        right = (linear_x + angular_z * self.wheel_separation / 2.0) * self.max_wheel_velocity
        return left, right


class DiffDriveRobot:
    """Wraps an Articulation's wheel joints for throttle-based diff-drive control.

    wheel_joint_names/left_joint_names/right_joint_names/wheel_separation/max_wheel_velocity are
    all required (no defaults) since they describe one specific robot's geometry -- there's no
    generic fallback that would be meaningful across robots.

    Construct after app_utils.play() -- Articulation requires a playing simulation.
    """

    def __init__(
        self,
        prim_path: str,
        wheel_joint_names: tuple[str, ...],
        left_joint_names: tuple[str, ...],
        right_joint_names: tuple[str, ...],
        wheel_separation: float,
        max_wheel_velocity: float,
        *,
        damping: float = 5.0,
        max_effort: float = 8.0,
    ):
        self.robot = Articulation(prim_path)
        self._left_ids = self.robot.get_dof_indices(list(left_joint_names))
        self._right_ids = self.robot.get_dof_indices(list(right_joint_names))
        wheel_ids = self.robot.get_dof_indices(list(wheel_joint_names))

        # Velocity control: zero stiffness + damping only -- appropriate for a USD asset whose
        # drive schema ships with zero gains (driving does nothing until these are set); adjust
        # damping/max_effort for a different actuator model.
        self.robot.set_dof_gains(stiffnesses=0.0, dampings=damping, dof_indices=wheel_ids)
        self.robot.set_dof_max_efforts(max_effort, dof_indices=wheel_ids)
        self.robot.set_dof_max_velocities(max_wheel_velocity, dof_indices=wheel_ids)

        self.mixer = DiffDriveMixer(wheel_separation, max_wheel_velocity)

    def drive(self, linear_x: float, angular_z: float) -> None:
        """Convert throttle fractions to wheel velocities (see DiffDriveMixer) and apply them."""
        left, right = self.mixer.mix(linear_x, angular_z)
        self.robot.set_dof_velocity_targets(left, dof_indices=self._left_ids)
        self.robot.set_dof_velocity_targets(right, dof_indices=self._right_ids)


def _ros_package_share_dir(package_name: str) -> str:
    """Resolve a ROS 2 package's share dir via `ros2 pkg prefix` rather than importing
    ament_index_python directly -- this interpreter is Isaac Sim's own, so that isn't guaranteed
    importable, but the ros2 CLI is.
    """
    prefix = subprocess.check_output(["ros2", "pkg", "prefix", package_name], text=True).strip()
    return os.path.join(prefix, "share", package_name)


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


class TeleopProcessManager:
    """Optionally starts a /cmd_vel-publishing input pipeline as subprocesses (joystick or
    keyboard), and stops them again on stop().

    joystick mode needs a params YAML (for joy_node + teleop_twist_joy): pass it directly via
    start()'s joystick_params_path, or set package_name here to resolve
    <package share dir>/config/joystick.yaml automatically.
    """

    def __init__(self, package_name: str | None = None):
        self.package_name = package_name
        self._procs: list[subprocess.Popen] = []

    def start(self, mode: str, joy_device_id: int | None = None, joystick_params_path: str | None = None) -> None:
        """Start the requested teleop pipeline. 'none' starts nothing (drive from an externally-
        run publisher instead); this manager still only ever stops what it itself started.
        """
        if mode == "none":
            return

        if mode == "joystick":
            if joystick_params_path is None:
                if self.package_name is None:
                    raise ValueError(
                        "joystick mode needs joystick_params_path or a package_name to resolve it from."
                    )
                joystick_params_path = os.path.join(_ros_package_share_dir(self.package_name), "config",
                                                     "joystick.yaml")
            device_id = _select_joystick_device(joy_device_id)
            print(f"[INFO] Starting joy_node (device_id={device_id}) + teleop_twist_joy -> /cmd_vel")
            print(f"[INFO] Hold the enable button (see {joystick_params_path}'s enable_button) while moving the stick.")
            self._procs = [
                subprocess.Popen([
                    "ros2", "run", "joy", "joy_node",
                    "--ros-args", "--params-file", joystick_params_path, "-p", f"device_id:={device_id}",
                ]),
                subprocess.Popen([
                    "ros2", "run", "teleop_twist_joy", "teleop_node",
                    "--ros-args", "--params-file", joystick_params_path, "-p", "publish_stamped_twist:=false",
                ]),
            ]
            return

        if mode == "keyboard":
            print("[INFO] Starting teleop_twist_keyboard -> /cmd_vel")
            print("[INFO] Keep this terminal focused -- Isaac Sim log lines will interleave with its on-screen keys.")
            # Reads this terminal's stdin directly (termios), not a rclpy subscription.
            self._procs = [subprocess.Popen(["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"])]
            return

        raise ValueError(f"Unknown teleop mode: {mode}")

    def stop(self) -> None:
        for proc in self._procs:
            proc.terminate()
        for proc in self._procs:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
