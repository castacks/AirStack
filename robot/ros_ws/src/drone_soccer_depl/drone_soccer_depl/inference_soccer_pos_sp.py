"""Run the trained drone-soccer policy through PX4 offboard position control."""

from __future__ import annotations

import math
import os
from pathlib import Path
import re
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
import torch
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from torch import nn


POLICY_RATE_HZ = 10.0
OBSERVATION_SIZE = 20
ACTION_SIZE = 4
NORMALIZATION_EPSILON = 1.0e-2
MAX_YAW_OFFSET_RAD = math.radians(20.0)
DEFAULT_CHECKPOINT_NAME = "model_700.pt"


class SoccerActor(nn.Module):
    """Deterministic actor matching the trained RSL-RL policy architecture."""

    def __init__(self, actor_state: dict[str, torch.Tensor]) -> None:
        """Build and populate the actor from an RSL-RL actor state dictionary.

        Args:
            actor_state: The checkpoint's ``actor_state_dict`` mapping.
        """
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(OBSERVATION_SIZE, 128),
            nn.ELU(),
            nn.Linear(128, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU(),
            nn.Linear(64, ACTION_SIZE),
        )

        mlp_state = {
            name.removeprefix("mlp."): value
            for name, value in actor_state.items()
            if name.startswith("mlp.")
        }
        self.mlp.load_state_dict(mlp_state, strict=True)

        self.register_buffer("observation_mean", actor_state["obs_normalizer._mean"])
        self.register_buffer("observation_std", actor_state["obs_normalizer._std"])

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        """Return the deterministic normalized action for one observation."""
        normalized = (observation - self.observation_mean) / (
            self.observation_std + NORMALIZATION_EPSILON
        )
        return self.mlp(normalized)


def choose_device(requested_device: str) -> torch.device:
    """Resolve ``auto`` to CUDA when available and otherwise use CPU."""
    if requested_device == "auto":
        return torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    device = torch.device(requested_device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError(f"CUDA device requested but unavailable: {requested_device}")
    return device


def load_policy(checkpoint_path: Path, device: torch.device) -> SoccerActor:
    """Load the deterministic actor stored in an RSL-RL checkpoint."""
    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"Policy checkpoint not found: {checkpoint_path}")

    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=True)
    actor_state = checkpoint.get("actor_state_dict")
    if actor_state is None:
        raise KeyError("Checkpoint does not contain 'actor_state_dict'")

    policy = SoccerActor(actor_state).to(device)
    policy.eval()
    return policy


def quaternion_to_rotation_matrix(quaternion: torch.Tensor) -> torch.Tensor:
    """Convert a normalized ``[w, x, y, z]`` quaternion to a rotation matrix."""
    quaternion = quaternion / torch.linalg.vector_norm(quaternion).clamp_min(1.0e-8)
    w, x, y, z = quaternion.unbind()
    return torch.stack(
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ]
    ).reshape(3, 3)


class DroneSoccerOffboard(Node):
    """Publish the offboard heartbeat and run soccer inference in offboard mode."""

    def __init__(self) -> None:
        """Create ROS interfaces, load the policy, and start the control timer."""
        super().__init__("drone_soccer_offboard")

        package_share = Path(get_package_share_directory("drone_soccer_depl"))
        default_checkpoint = package_share / DEFAULT_CHECKPOINT_NAME
        docker_path = "/root/AirStack/robot/ros_ws/src/drone_soccer_depl"
        ckpt_name = "model_999.pt"
        default_checkpoint = os.path.join(docker_path, ckpt_name)
        environment_robot_name = os.environ.get("ROBOT_NAME", "").strip("/")
        environment_name_is_valid = (
            re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", environment_robot_name)
            is not None
            and environment_robot_name != "unknown_robot"
        )
        default_robot_name = (
            environment_robot_name if environment_name_is_valid else "drone_3"
        )
        self.declare_parameter("checkpoint_path", str(default_checkpoint))
        self.declare_parameter("device", "auto")
        self.declare_parameter("robot_name", default_robot_name)
        self.declare_parameter("fmu_prefix", "")
        self.declare_parameter(
            "ball_odometry_topic", "/VolleyBall/mocap_odometry"
        )
        self.declare_parameter("goal_pose_topic", "")
        self.declare_parameter("target_x_ned", 0.0)
        self.declare_parameter("target_y_ned", 0.0)
        self.declare_parameter("next_target_x_ned", 0.0)
        self.declare_parameter("next_target_y_ned", 0.0)
        self.declare_parameter("goal_reach_radius_m", 0.3)
        self.declare_parameter("input_timeout_s", 0.5)
        self.declare_parameter("enable_check", False)

        checkpoint_path = Path(
            self.get_parameter("checkpoint_path").get_parameter_value().string_value
        ).expanduser()
        requested_device = self.get_parameter("device").value
        self.device = choose_device(str(requested_device))
        self.policy = load_policy(checkpoint_path, self.device)

        robot_name = str(self.get_parameter("robot_name").value).strip("/")
        if not robot_name:
            raise ValueError("robot_name must not be empty")
        robot_namespace = f"/{robot_name}"
        configured_fmu_prefix = str(self.get_parameter("fmu_prefix").value)
        self.fmu_prefix = (
            configured_fmu_prefix.rstrip("/")
            if configured_fmu_prefix
            else f"{robot_namespace}/fmu"
        )
        ball_odometry_topic = str(
            self.get_parameter("ball_odometry_topic").value
        )
        configured_goal_topic = str(self.get_parameter("goal_pose_topic").value)
        goal_pose_topic = configured_goal_topic or f"{robot_namespace}/goal/pose"
        goal_points_ned = torch.tensor(
            [
                [
                    float(self.get_parameter("target_x_ned").value),
                    float(self.get_parameter("target_y_ned").value),
                ],
                [
                    float(self.get_parameter("next_target_x_ned").value),
                    float(self.get_parameter("next_target_y_ned").value),
                ],
            ],
            dtype=torch.float32,
            device=self.device,
        )
        # NED (north, east) -> ENU (east, north) for policy observations.
        self.goal_points_enu = goal_points_ned[:, [1, 0]]
        self.goal_index = 0
        self.target_xy_enu = self.goal_points_enu[self.goal_index]
        self.goal_reach_radius_m = float(
            self.get_parameter("goal_reach_radius_m").value
        )
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)

        fmu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        ball_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_mode_publisher = self.create_publisher(
            OffboardControlMode,
            self._fmu_topic("in/offboard_control_mode"),
            fmu_qos,
        )
        self.trajectory_publisher = self.create_publisher(
            TrajectorySetpoint,
            self._fmu_topic("in/trajectory_setpoint"),
            fmu_qos,
        )
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            goal_pose_topic,
            goal_qos,
        )

        self.create_subscription(
            VehicleOdometry,
            self._fmu_topic("out/vehicle_odometry"),
            self._odometry_callback,
            fmu_qos,
        )
        self.create_subscription(
            VehicleStatus,
            self._fmu_topic("out/vehicle_status"),
            self._status_callback,
            fmu_qos,
        )
        self.create_subscription(
            Odometry,
            ball_odometry_topic,
            self._ball_odometry_callback,
            ball_qos,
        )

        self.vehicle_odometry: Optional[VehicleOdometry] = None
        self.vehicle_status: Optional[VehicleStatus] = None
        self.ball_position_enu: Optional[torch.Tensor] = None
        self.ball_velocity_enu: Optional[torch.Tensor] = None
        self.last_odometry_time: Optional[float] = None
        self.last_ball_pose_time: Optional[float] = None
        self.reported_state = ""
        self.goal_reached_reported = False

        self._publish_goal_pose()
        self.create_timer(1.0 / POLICY_RATE_HZ, self._control_callback)
        self.get_logger().info(
            f"Loaded soccer policy from {checkpoint_path} on {self.device}; "
            f"robot={robot_name}; filtered ball state from "
            f"{ball_odometry_topic}; waiting for "
            f"{self.fmu_prefix}/out data and RC-selected offboard mode"
        )

    def _fmu_topic(self, suffix: str) -> str:
        """Return a topic below the configured FMU prefix."""
        return f"{self.fmu_prefix}/{suffix}"

    def _now_seconds(self) -> float:
        """Return the node clock in seconds."""
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _timestamp_microseconds(self) -> int:
        """Return the node clock in microseconds for PX4 messages."""
        return int(self.get_clock().now().nanoseconds / 1000)

    def _odometry_callback(self, message: VehicleOdometry) -> None:
        """Store the newest PX4 odometry sample."""
        self.vehicle_odometry = message
        self.last_odometry_time = self._now_seconds()

    def _status_callback(self, message: VehicleStatus) -> None:
        """Store the newest PX4 vehicle status."""
        self.vehicle_status = message

    def _ball_odometry_callback(self, message: Odometry) -> None:
        """Store the bridge's filtered soccer-ball position and velocity."""
        position = message.pose.pose.position
        velocity = message.twist.twist.linear
        self.ball_position_enu = torch.tensor(
            [position.x, position.y, position.z],
            dtype=torch.float32,
            device=self.device,
        )
        self.ball_velocity_enu = torch.tensor(
            [velocity.x, velocity.y, velocity.z],
            dtype=torch.float32,
            device=self.device,
        )
        self.last_ball_pose_time = self._now_seconds()
        self._report_goal_reached()

    def _report_state(self, state: str) -> None:
        """Log a state transition once instead of printing every control tick."""
        if state == self.reported_state:
            return
        self.reported_state = state
        self.get_logger().info(state)

    def _report_goal_reached(self) -> None:
        """Switch to and publish the next goal after the ball reaches one."""
        if self.ball_position_enu is None:
            return

        distance = torch.linalg.vector_norm(
            self.target_xy_enu - self.ball_position_enu[:2]
        ).item()
        goal_reached = distance < self.goal_reach_radius_m
        if goal_reached and not self.goal_reached_reported:
            self.get_logger().info(
                f"Ball reached the goal at ENU "
                f"({self.target_xy_enu[0].item():.2f}, "
                f"{self.target_xy_enu[1].item():.2f})"
            )
            self.goal_index = (self.goal_index + 1) % len(self.goal_points_enu)
            self.target_xy_enu = self.goal_points_enu[self.goal_index]
            self._publish_goal_pose()
            self.get_logger().info(
                f"Next goal set to ENU "
                f"({self.target_xy_enu[0].item():.2f}, "
                f"{self.target_xy_enu[1].item():.2f})"
            )
        self.goal_reached_reported = goal_reached

    def _publish_goal_pose(self) -> None:
        """Publish the active ENU goal for the Pegasus GUI marker."""
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = float(self.target_xy_enu[0].item())
        message.pose.position.y = float(self.target_xy_enu[1].item())
        message.pose.position.z = 0.0
        message.pose.orientation.w = 1.0
        self.goal_pose_publisher.publish(message)

    def _input_is_fresh(self, timestamp: Optional[float]) -> bool:
        """Return whether a required input has arrived recently enough."""
        if timestamp is None:
            return False
        return self._now_seconds() - timestamp <= self.input_timeout_s

    def _publish_offboard_heartbeat(self) -> None:
        """Publish the position-control heartbeat required by PX4 offboard mode."""
        message = OffboardControlMode()
        message.position = True
        message.velocity = False
        message.acceleration = False
        message.attitude = False
        message.body_rate = False
        message.timestamp = self._timestamp_microseconds()
        self.offboard_mode_publisher.publish(message)

    def _publish_position_setpoint(
        self, north: float, east: float, down: float, yaw: float
    ) -> None:
        """Publish an absolute PX4 NED position and yaw setpoint."""
        message = TrajectorySetpoint()
        message.position = [float(north), float(east), float(down)]
        message.velocity = [math.nan, math.nan, math.nan]
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = float(yaw)
        message.yawspeed = math.nan
        message.timestamp = self._timestamp_microseconds()
        self.trajectory_publisher.publish(message)

    def _create_observation(self) -> torch.Tensor:
        """Build the trained 20-value ENU/FLU soccer observation."""
        if self.vehicle_odometry is None:
            raise RuntimeError("Vehicle odometry is unavailable")
        if self.ball_position_enu is None or self.ball_velocity_enu is None:
            raise RuntimeError("Ball state is unavailable")

        odometry = self.vehicle_odometry
        if odometry.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            raise ValueError(f"Unsupported PX4 pose frame: {odometry.pose_frame}")

        position_ned = torch.tensor(
            odometry.position, dtype=torch.float32, device=self.device
        )
        quaternion_ned_frd = torch.tensor(
            odometry.q, dtype=torch.float32, device=self.device
        )
        rotation_ned_frd = quaternion_to_rotation_matrix(quaternion_ned_frd)

        ned_to_enu = torch.tensor(
            [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
            dtype=torch.float32,
            device=self.device,
        )
        flu_to_frd = torch.diag(
            torch.tensor([1.0, -1.0, -1.0], device=self.device)
        )
        rotation_enu_flu = ned_to_enu @ rotation_ned_frd @ flu_to_frd
        drone_position_enu = ned_to_enu @ position_ned
        drone_yaw_enu = torch.atan2(
            rotation_enu_flu[1, 0], rotation_enu_flu[0, 0]
        )
        yaw_encoding = torch.stack(
            [torch.cos(drone_yaw_enu), torch.sin(drone_yaw_enu)]
        )

        velocity = torch.tensor(
            odometry.velocity, dtype=torch.float32, device=self.device
        )
        if odometry.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED:
            drone_velocity_enu = ned_to_enu @ velocity
        elif odometry.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            drone_velocity_enu = ned_to_enu @ rotation_ned_frd @ velocity
        else:
            raise ValueError(
                f"Unsupported PX4 velocity frame: {odometry.velocity_frame}"
            )

        angular_velocity_frd = torch.tensor(
            odometry.angular_velocity, dtype=torch.float32, device=self.device
        )
        angular_velocity_flu = flu_to_frd @ angular_velocity_frd

        observation = torch.cat(
            [
                drone_position_enu[2:3],
                rotation_enu_flu[:, 2],
                yaw_encoding,
                drone_velocity_enu,
                angular_velocity_flu,
                self.ball_position_enu - drone_position_enu,
                self.target_xy_enu - self.ball_position_enu[:2],
                self.ball_velocity_enu,
            ]
        )
        if observation.numel() != OBSERVATION_SIZE:
            raise RuntimeError(f"Expected 20 observations, got {observation.numel()}")
        return observation.unsqueeze(0)

    def _evaluate_policy_setpoint(self) -> tuple[float, float, float, float]:
        """Run deterministic inference and return a PX4 NED position setpoint."""
        if self.vehicle_odometry is None:
            raise RuntimeError("Vehicle odometry is unavailable")

        observation = self._create_observation()
        with torch.inference_mode():
            normalized_action = self.policy(observation)[0].clamp(-1.0, 1.0)

        # Training maps normalized action to (relative east, relative north,
        # absolute up, relative ENU yaw) with ranges [-1, 1] m, [-1, 1] m,
        # [0.4, 2.0] m, and [-30, 30] degrees.
        east_offset = float(normalized_action[0].item())
        north_offset = float(normalized_action[1].item())
        # height = 1.2 + 0.8 * float(normalized_action[2].item())
        height = 0.85 + 0.45 * float(normalized_action[2].item())
        current_yaw_enu = math.atan2(
            float(observation[0, 5].item()),
            float(observation[0, 4].item()),
        )
        desired_yaw_enu = (
            current_yaw_enu
            + MAX_YAW_OFFSET_RAD * float(normalized_action[3].item())
        )

        # ENU yaw is counter-clockwise from East; PX4 NED yaw is clockwise
        # from North. Wrap the converted setpoint to [-pi, pi].
        desired_yaw_ned = math.pi / 2.0 - desired_yaw_enu
        desired_yaw_ned = math.atan2(
            math.sin(desired_yaw_ned), math.cos(desired_yaw_ned)
        )

        current_north, current_east, _ = self.vehicle_odometry.position
        return (
            float(current_north) + north_offset,
            float(current_east) + east_offset,
            -height,
            desired_yaw_ned,
        )

    def _policy_inputs_are_ready(self) -> bool:
        """Return whether all policy inputs exist and are recent."""
        inputs_are_fresh = all(
            self._input_is_fresh(timestamp)
            for timestamp in (
                self.last_odometry_time,
                self.last_ball_pose_time,
            )
        )
        return inputs_are_fresh and self.ball_velocity_enu is not None

    def _control_callback(self) -> None:
        """Keep offboard available and run the policy only after RC selects it."""
        if bool(self.get_parameter("enable_check").value):
            if not self._policy_inputs_are_ready():
                self._report_state(
                    "Check mode enabled; waiting for fresh odometry and ball state"
                )
                return

            try:
                north, east, down, yaw = self._evaluate_policy_setpoint()
            except (RuntimeError, ValueError) as error:
                self._report_state(f"Policy input error: {error}")
                return

            self._report_state("Check mode enabled; PX4 output disabled")
            self.get_logger().info(
                "Policy result (NED): "
                f"X={north:.3f}, Y={east:.3f}, Z={down:.3f}, "
                f"yaw={yaw:.3f}",
                throttle_duration_sec=0.5,
            )
            return

        self._publish_offboard_heartbeat()

        # if self.vehicle_status is None:
        #     self._report_state("Waiting for PX4 vehicle status")
        #     return
        # if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self._report_state("Publishing heartbeat; waiting for RC-selected offboard mode")
        #     return
        if not self._policy_inputs_are_ready():
            self._report_state("Offboard active; waiting for fresh odometry and ball state")
            return

        try:
            north, east, down, yaw = self._evaluate_policy_setpoint()
            # print(north, east, down)
        except (RuntimeError, ValueError) as error:
            self._report_state(f"Policy input error: {error}")
            return

        self._publish_position_setpoint(north, east, down, yaw)
        self._report_state("Offboard active; running drone-soccer policy at 50 Hz")


def main(args: Optional[list[str]] = None) -> None:
    """Initialize ROS, run the offboard node, and shut down cleanly."""
    rclpy.init(args=args)
    node = DroneSoccerOffboard()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
