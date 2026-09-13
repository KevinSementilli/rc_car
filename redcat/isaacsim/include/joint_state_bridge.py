# SPDX-License-Identifier: BSD-3-Clause

"""Joint state + TF bridge between Isaac Sim and ROS 2, so robot_state_publisher (fed by
/joint_states) can build the robot's TF tree and rviz2 can show it driving around.

Built entirely from isaacsim.ros2.bridge / isaacsim.core.nodes OmniGraph nodes -- the same
extension the Isaac Sim UI's "ROS2 Joint States" and "ROS2 Odometry" graph wizards generate --
rather than reimplementing joint/pose publishing by hand over rclpy. See
isaacsim.ros2.ui/isaacsim/ros2/ui/og_utils.py (JointStatePublishGraph / OdometryPublishGraph) and
standalone_examples/api/isaacsim.ros2.bridge/moveit.py for the reference graphs this mirrors.

Publishes two things off one OnPlaybackTick, once the graph is created:
- /joint_states (sensor_msgs/JointState) from the articulation's own joint state -- this is what
  robot_state_publisher needs to compute the robot's internal TF tree (base_link -> wheels, ...).
- odom -> base_link on /tf, via IsaacComputeOdometry -- robot_state_publisher only ever publishes
  the *internal* tree relative to the robot's root link, so without this, rviz2 has nowhere to
  move that root link and the robot appears frozen even while driving. odom is left as the tree's
  root here (no parent published) -- with no nav stack running, rviz2's Fixed Frame is odom
  directly; with slam_toolbox/nav2 running, they publish map -> odom on top of it themselves, per
  this project's REP-105 setup (see config/nav/*.yaml), and Fixed Frame becomes map.

Not tied to any specific robot -- takes the articulation's prim path and frame ids as arguments.
That path must be the exact prim PhysicsArticulationRootAPI is applied to, not just any prim
under it -- the physx tensor articulation view this reads through (IsaacReadJointState /
IsaacComputeOdometry) matches by exact path and won't descend to find it, unlike
isaacsim.core.experimental.prims.Articulation (see diff_drive_bridge.DiffDriveRobot), which
resolves a nested articulation root on its own.

Must only be imported after a SimulationApp has been constructed, with isaacsim.ros2.bridge
already enabled -- its own imports below need both to already exist.

Usage, called once the robot prim exists on stage (order relative to SimulationManager
.setup_simulation()/app_utils.play() doesn't matter -- this only authors OmniGraph prims):

    bridge = JointStateBridge(articulation_root_path, base_frame_id="base_link")
    bridge.create()
    bridge.describe()
"""

import omni.graph.core as og
import usdrt.Sdf


class JointStateBridge:
    """Creates an OmniGraph that publishes /joint_states and the odom->base_link TF link for one
    articulated robot. odom is left as the tree's root -- publish_odom=True is enough with no nav
    stack running; with slam_toolbox/nav2 running, they publish map->odom on top of it themselves.
    """

    def __init__(
        self,
        articulation_root_path: str,
        *,
        base_frame_id: str = "base_link",
        odom_frame_id: str = "odom",
        joint_states_topic: str = "joint_states",
        odom_topic: str = "odom",
        tf_topic: str = "tf",
        publish_odom: bool = True,
        graph_path: str = "/Graph/ROS_JointStateBridge",
    ):
        self.articulation_root_path = articulation_root_path
        self.base_frame_id = base_frame_id
        self.odom_frame_id = odom_frame_id
        self.joint_states_topic = joint_states_topic
        self.odom_topic = odom_topic
        self.tf_topic = tf_topic
        self.publish_odom = publish_odom
        self.graph_path = graph_path

    def create(self) -> None:
        """Author the OmniGraph. Safe to call once; re-running would create a second graph."""
        keys = og.Controller.Keys

        nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ]
        connections = [
            ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
            ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
            ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
            ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
            ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
            ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
            ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
            ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
            ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
        ]
        values = [
            ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(self.articulation_root_path)]),
            ("PublishJointState.inputs:topicName", self.joint_states_topic),
        ]

        if self.publish_odom:
            nodes += [
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("TFOdomToRobot", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ]
            connections += [
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "TFOdomToRobot.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                ("ComputeOdometry.outputs:position", "TFOdomToRobot.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "TFOdomToRobot.inputs:rotation"),
                ("Context.outputs:context", "PublishOdometry.inputs:context"),
                ("Context.outputs:context", "TFOdomToRobot.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "TFOdomToRobot.inputs:timeStamp"),
            ]
            values += [
                ("ComputeOdometry.inputs:chassisPrim", [usdrt.Sdf.Path(self.articulation_root_path)]),
                ("PublishOdometry.inputs:topicName", self.odom_topic),
                ("PublishOdometry.inputs:odomFrameId", self.odom_frame_id),
                ("PublishOdometry.inputs:chassisFrameId", self.base_frame_id),
                ("TFOdomToRobot.inputs:topicName", self.tf_topic),
                ("TFOdomToRobot.inputs:parentFrameId", self.odom_frame_id),
                ("TFOdomToRobot.inputs:childFrameId", self.base_frame_id),
            ]

        og.Controller.edit(
            {"graph_path": self.graph_path, "evaluator_name": "execution"},
            {keys.CREATE_NODES: nodes, keys.CONNECT: connections, keys.SET_VALUES: values},
        )

    def describe(self) -> None:
        """Print the active topics and matching robot_state_publisher / rviz2 setup instructions."""
        print(f"[INFO] Joint state bridge publishing sensor_msgs/JointState on /{self.joint_states_topic}")
        print("[INFO] Launch robot_state_publisher (fed by this topic) to get the robot's internal TF tree.")
        if self.publish_odom:
            print(f"[INFO] Publishing nav_msgs/Odometry on /{self.odom_topic} and "
                  f"{self.odom_frame_id} -> {self.base_frame_id} on /{self.tf_topic}")
            print(f"[INFO] In rviz2: set Fixed Frame to '{self.odom_frame_id}' (or 'map', once "
                  "slam_toolbox/nav2 is running and publishing map -> odom) and add a RobotModel display.")
        else:
            print(f"[INFO] Odometry/TF root publishing disabled -- rviz2's Fixed Frame must be "
                  f"'{self.base_frame_id}' itself, so the robot will not appear to move.")
