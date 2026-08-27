"""Run a trained drone_soccer PPO checkpoint on live mocap + PX4 odometry."""

from __future__ import annotations

from io import BytesIO
from pathlib import Path
from typing import Any
from zipfile import ZipFile

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

from drone_soccer.deploy.observation import (
    OBS_DIM,
    arrays_to_observation,
    quaternion_to_rotation_matrix,
)
from drone_soccer.deploy.waypoint import compute_abs_waypoint

from svg_ground_control.ball_state import BallStateTracker
from svg_ground_control.trajectory_commander_core import clamp_position, vector3


def compute_goal_reference(
    trajectory_type: str,
    elapsed_s: float,
    center_xy: np.ndarray,
    radius_m: float,
    period_s: float,
    phase_rad: float = 0.0,
    rotation_rad: float = 0.0,
) -> np.ndarray:
    """Evaluate the policy's planar ball-goal reference trajectory.

    Before spatial rotation, periodic references start at
    ``center + [radius, 0]`` when ``phase_rad`` is zero. Circles move
    counter-clockwise in the ENU frame; figure eights use a Gerono
    lemniscate.

    Args:
        trajectory_type: ``fixed``, ``circle``, or ``figure8``.
        elapsed_s: Time since the reference trajectory was started.
        center_xy: Fixed goal or circle center in ENU meters.
        radius_m: Circle radius or figure-eight half-width in meters.
        period_s: Period in seconds.
        phase_rad: Initial trajectory phase in radians.
        rotation_rad: Counter-clockwise spatial rotation in ENU radians.

    Returns:
        Goal XY position in ENU meters, shaped ``(2,)``.
    """
    trajectory = trajectory_type.strip().lower()
    center = np.asarray(center_xy, dtype=np.float32).reshape(2)
    if trajectory == 'fixed':
        return center.copy()
    if trajectory not in ('circle', 'figure8'):
        raise ValueError(
            f'goal_trajectory must be fixed, circle, or figure8, got '
            f'{trajectory_type!r}')
    if radius_m < 0.0:
        raise ValueError('goal_radius must be non-negative')
    if period_s <= 0.0:
        raise ValueError(
            'goal_period_s must be positive for a periodic trajectory')

    angle = phase_rad + 2.0 * np.pi * float(elapsed_s) / period_s
    if trajectory == 'circle':
        base_offset = radius_m * np.array(
            [np.cos(angle), np.sin(angle)], dtype=np.float32)
    else:
        # Gerono lemniscate: radius is the half-width and the maximum vertical
        # displacement is radius / 2. Phase zero starts at the rightmost tip.
        base_offset = radius_m * np.array(
            [np.cos(angle), np.sin(angle) * np.cos(angle)],
            dtype=np.float32,
        )
    cos_rotation = np.cos(rotation_rad)
    sin_rotation = np.sin(rotation_rad)
    offset = np.array([
        cos_rotation * base_offset[0] - sin_rotation * base_offset[1],
        sin_rotation * base_offset[0] + cos_rotation * base_offset[1],
    ], dtype=np.float32)
    return center + offset


def enu_waypoint_to_ned(waypoint: np.ndarray) -> np.ndarray:
    """Convert an ENU XYZ position waypoint to PX4's NED coordinates.

    Args:
        waypoint: ENU position in meters, shaped ``(3,)``.

    Returns:
        NED position in meters, shaped ``(3,)``.
    """
    enu = np.asarray(waypoint, dtype=np.float32).reshape(3)
    return np.array([enu[1], enu[0], -enu[2]], dtype=np.float32)


def compute_bounded_waypoint(
    drone_position: np.ndarray,
    policy_action: np.ndarray,
    bounds_min: np.ndarray,
    bounds_max: np.ndarray,
) -> np.ndarray:
    """Convert a relative policy action into a bounded absolute waypoint.

    Args:
        drone_position: Current absolute ENU position in meters.
        policy_action: Policy output ``(dx, dy, z_world)`` in meters.
        bounds_min: Minimum allowed absolute ENU XYZ position.
        bounds_max: Maximum allowed absolute ENU XYZ position.

    Returns:
        Absolute ENU waypoint clamped to the configured flight area.
    """
    waypoint = compute_abs_waypoint(drone_position, policy_action)
    return clamp_position(waypoint, bounds_min, bounds_max)


def load_policy_checkpoint(
    model_path: str,
    action_low: np.ndarray,
    action_high: np.ndarray,
) -> tuple[Any, bool]:
    """Load an SB3 PPO checkpoint, including NumPy 2 to NumPy 1 compatibility.

    Args:
        model_path: Path to the Stable-Baselines3 PPO archive.
        action_low: Lower bounds for the three policy actions.
        action_high: Upper bounds for the three policy actions.

    Returns:
        The object exposing ``predict`` and whether the weight-only
        compatibility loader was required.
    """
    from stable_baselines3 import PPO

    try:
        return PPO.load(model_path, device='cpu'), False
    except ModuleNotFoundError as exc:
        if not exc.name or not exc.name.startswith('numpy._core'):
            raise

    # safe.zip was saved under NumPy 2, whose pickle module names cannot be
    # resolved by AirStack's NumPy 1.26. Rebuild the archive's standard
    # ActorCriticPolicy and load its state dict without unpickling SB3 metadata.
    import torch
    from gymnasium.spaces import Box
    from stable_baselines3.common.policies import ActorCriticPolicy

    observation_space = Box(
        low=-np.inf,
        high=np.inf,
        shape=(OBS_DIM,),
        dtype=np.float32,
    )
    action_space = Box(
        low=np.asarray(action_low, dtype=np.float32),
        high=np.asarray(action_high, dtype=np.float32),
        dtype=np.float32,
    )
    policy = ActorCriticPolicy(
        observation_space,
        action_space,
        lr_schedule=lambda _: 0.0,
    )

    with ZipFile(model_path) as archive:
        policy_bytes = archive.read('policy.pth')
    state_dict = torch.load(
        BytesIO(policy_bytes),
        map_location='cpu',
        weights_only=True,
    )
    policy.load_state_dict(state_dict)
    policy.eval()
    return policy, True


class PolicyCommander(Node):
    def __init__(self) -> None:
        super().__init__('policy_commander')

        self.declare_parameter('drone', 'drone_3')
        self.declare_parameter('ball_name', 'SoccerBall')
        self.declare_parameter('model_path', '')
        self.declare_parameter('target_x', 3.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('goal_trajectory', 'fixed')
        self.declare_parameter('goal_radius', 0.0)
        self.declare_parameter('goal_period_s', 8.0)
        self.declare_parameter('goal_phase_rad', 0.0)
        self.declare_parameter('goal_rotation_rad', 0.0)
        self.declare_parameter('policy_dt', 0.02)
        self.declare_parameter('deterministic', True)
        self.declare_parameter('action_low', [-3.0, -3.0, 0.3])
        self.declare_parameter('action_high', [3.0, 3.0, 2.0])
        self.declare_parameter('bounds_min', [-3.0, -3.0, 0.3])
        self.declare_parameter('bounds_max', [3.0, 3.0, 1.2])
        self.declare_parameter('odom_topic_template',
                               '/{name}/odometry_conversion/odometry')
        self.declare_parameter('ball_odom_topic_template',
                               '/{name}/mocap_odometry')
        self.declare_parameter('ball_pose_topic_template',
                               '/{name}/pose')
        self.declare_parameter(
            'trajectory_setpoint_topic_template',
            '/{name}/fmu/in/trajectory_setpoint',
        )
        self.declare_parameter(
            'offboard_control_mode_topic_template',
            '/{name}/fmu/in/offboard_control_mode',
        )
        self.declare_parameter('state_timeout_s', 0.5)
        self.declare_parameter('velocity_filter_alpha', 0.4)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_topic_prefix', '/policy_commander')

        self.drone = str(self.get_parameter('drone').value)
        ball_name = str(self.get_parameter('ball_name').value)
        model_path = str(self.get_parameter('model_path').value).strip()
        if not model_path:
            raise ValueError('model_path parameter is required (path to SB3 .zip)')
        if not Path(model_path).is_file():
            raise FileNotFoundError(f'model_path not found: {model_path}')

        self.goal_center_xy = np.array([
            float(self.get_parameter('target_x').value),
            float(self.get_parameter('target_y').value),
        ], dtype=np.float32)
        self.goal_trajectory = str(
            self.get_parameter('goal_trajectory').value).strip().lower()
        self.goal_radius = float(self.get_parameter('goal_radius').value)
        self.goal_period_s = float(
            self.get_parameter('goal_period_s').value)
        self.goal_phase_rad = float(
            self.get_parameter('goal_phase_rad').value)
        self.goal_rotation_rad = float(
            self.get_parameter('goal_rotation_rad').value)
        self.target_xy = compute_goal_reference(
            self.goal_trajectory,
            0.0,
            self.goal_center_xy,
            self.goal_radius,
            self.goal_period_s,
            self.goal_phase_rad,
            self.goal_rotation_rad,
        )
        self._goal_reference_start = self.get_clock().now()
        self.action_low = np.asarray(
            self.get_parameter('action_low').value, dtype=np.float32)
        self.action_high = np.asarray(
            self.get_parameter('action_high').value, dtype=np.float32)
        self.bounds_min = vector3(
            self.get_parameter('bounds_min').value, 'bounds_min')
        self.bounds_max = vector3(
            self.get_parameter('bounds_max').value, 'bounds_max')
        self.state_timeout_s = float(self.get_parameter('state_timeout_s').value)
        self.deterministic = bool(self.get_parameter('deterministic').value)
        self.running = False
        self._last_waypoint: np.ndarray | None = None

        self._latest_drone_odom: Odometry | None = None
        self._latest_drone_stamp = None
        self._latest_ball_stamp = None
        self._ball_tracker = BallStateTracker(
            velocity_alpha=float(self.get_parameter('velocity_filter_alpha').value))
        self._ball_from_odom = False

        self.get_logger().info(f'Loading PPO from {model_path}')
        self.policy, used_compat_loader = load_policy_checkpoint(
            model_path,
            self.action_low,
            self.action_high,
        )
        if used_compat_loader:
            self.get_logger().warn(
                'Loaded policy weights with NumPy 2 checkpoint compatibility '
                'mode; SB3 metadata was not deserialized')

        odom_topic = str(self.get_parameter('odom_topic_template').value).format(
            name=self.drone)
        ball_odom_topic = str(
            self.get_parameter('ball_odom_topic_template').value).format(
                name=ball_name)
        ball_pose_topic = str(
            self.get_parameter('ball_pose_topic_template').value).format(
                name=ball_name)
        trajectory_topic = str(
            self.get_parameter('trajectory_setpoint_topic_template').value
        ).format(name=self.drone)
        offboard_mode_topic = str(
            self.get_parameter('offboard_control_mode_topic_template').value
        ).format(name=self.drone)

        self.create_subscription(Odometry, odom_topic, self._drone_odom_cb, 10)
        self.create_subscription(Odometry, ball_odom_topic, self._ball_odom_cb, 10)
        self.create_subscription(
            PoseStamped, ball_pose_topic, self._ball_pose_cb, 10)

        px4_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            trajectory_topic,
            px4_qos,
        )
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            offboard_mode_topic,
            px4_qos,
        )
        self.trajectory_topic = trajectory_topic
        self.offboard_mode_topic = offboard_mode_topic

        prefix = str(self.get_parameter('debug_topic_prefix').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        if self.publish_debug:
            self.obs_pub = self.create_publisher(
                Float32MultiArray, f'{prefix}/obs', 10)
            self.action_pub = self.create_publisher(
                Float32MultiArray, f'{prefix}/action', 10)
            self.waypoint_pub = self.create_publisher(
                Float32MultiArray, f'{prefix}/waypoint', 10)
            self.goal_pub = self.create_publisher(
                Float32MultiArray, f'{prefix}/goal', 10)

        self.create_service(Trigger, '~/start', self._handle_start)
        self.create_service(Trigger, '~/stop', self._handle_stop)

        policy_dt = float(self.get_parameter('policy_dt').value)
        if policy_dt <= 0.0:
            raise ValueError('policy_dt must be positive')
        self.create_timer(policy_dt, self._policy_tick)

        self.get_logger().info(
            f'PolicyCommander ready | drone={self.drone} ball={ball_name} '
            f'odom={odom_topic} ball_odom={ball_odom_topic} '
            f'trajectory_out={trajectory_topic} '
            f'offboard_out={offboard_mode_topic} obs_dim={OBS_DIM} '
            f'goal={self.goal_trajectory} '
            f'center={self.goal_center_xy.tolist()} '
            f'radius={self.goal_radius:.3f}m period={self.goal_period_s:.3f}s '
            f'rotation={self.goal_rotation_rad:.3f}rad')

    def _handle_start(self, _request, response):
        self._goal_reference_start = self.get_clock().now()
        self.target_xy = compute_goal_reference(
            self.goal_trajectory,
            0.0,
            self.goal_center_xy,
            self.goal_radius,
            self.goal_period_s,
            self.goal_phase_rad,
            self.goal_rotation_rad,
        )
        self.running = True
        response.success = True
        response.message = 'policy loop publishing enabled'
        self.get_logger().info(response.message)
        return response

    def _handle_stop(self, _request, response):
        self.running = False
        response.success = True
        response.message = 'policy loop paused (last waypoint held if published)'
        self.get_logger().info(response.message)
        return response

    def _drone_odom_cb(self, msg: Odometry) -> None:
        self._latest_drone_odom = msg
        self._latest_drone_stamp = self.get_clock().now()

    def _ball_odom_cb(self, msg: Odometry) -> None:
        self._ball_from_odom = True
        self._ball_tracker.update_from_odometry(msg)
        self._latest_ball_stamp = self.get_clock().now()

    def _ball_pose_cb(self, msg: PoseStamped) -> None:
        if self._ball_from_odom:
            return
        self._ball_tracker.update_from_pose(msg)
        self._latest_ball_stamp = self.get_clock().now()

    def _inputs_fresh(self) -> bool:
        now = self.get_clock().now()
        if self._latest_drone_stamp is None:
            return False
        age = (now - self._latest_drone_stamp).nanoseconds * 1e-9
        if age > self.state_timeout_s:
            return False
        ball = self._ball_tracker.state
        if ball is None or self._latest_ball_stamp is None:
            return False
        ball_age = (now - self._latest_ball_stamp).nanoseconds * 1e-9
        if ball_age > self.state_timeout_s:
            return False
        return True

    def _update_goal_reference(self) -> None:
        """Update the observation goal from the configured reference clock."""
        elapsed_s = (
            self.get_clock().now() - self._goal_reference_start
        ).nanoseconds * 1e-9
        self.target_xy = compute_goal_reference(
            self.goal_trajectory,
            elapsed_s,
            self.goal_center_xy,
            self.goal_radius,
            self.goal_period_s,
            self.goal_phase_rad,
            self.goal_rotation_rad,
        )

    def _build_observation(self) -> np.ndarray | None:
        odom = self._latest_drone_odom
        ball = self._ball_tracker.state
        if odom is None or ball is None:
            return None

        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        drone_pos = np.array([p.x, p.y, p.z], dtype=np.float32)
        rot = quaternion_to_rotation_matrix(q.w, q.x, q.y, q.z).astype(np.float32)

        t = odom.twist.twist
        drone_vel = np.array([t.linear.x, t.linear.y, t.linear.z], dtype=np.float32)
        drone_omega = np.array(
            [t.angular.x, t.angular.y, t.angular.z], dtype=np.float32)

        return arrays_to_observation(
            drone_pos,
            drone_vel,
            rot,
            drone_omega,
            ball.position,
            ball.velocity,
            self.target_xy,
        )

    def _publish_debug(self, obs: np.ndarray, action: np.ndarray,
                       waypoint: np.ndarray) -> None:
        if not self.publish_debug:
            return
        obs_msg = Float32MultiArray()
        obs_msg.data = [float(x) for x in obs]
        self.obs_pub.publish(obs_msg)
        act_msg = Float32MultiArray()
        act_msg.data = [float(x) for x in action]
        self.action_pub.publish(act_msg)
        wpt_msg = Float32MultiArray()
        wpt_msg.data = [float(x) for x in waypoint]
        self.waypoint_pub.publish(wpt_msg)
        goal_msg = Float32MultiArray()
        goal_msg.data = [float(x) for x in self.target_xy]
        self.goal_pub.publish(goal_msg)

    def _publish_fmu_setpoint(self, waypoint: np.ndarray) -> None:
        """Publish a position setpoint and PX4 Offboard heartbeat directly."""
        timestamp_us = self.get_clock().now().nanoseconds // 1000
        ned_waypoint = enu_waypoint_to_ned(waypoint)

        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp_us
        offboard_mode.position = True
        offboard_mode.velocity = False
        offboard_mode.acceleration = False
        offboard_mode.attitude = False
        offboard_mode.body_rate = False
        offboard_mode.thrust_and_torque = False
        offboard_mode.direct_actuator = False
        self.offboard_mode_pub.publish(offboard_mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp_us
        setpoint.position = [float(value) for value in ned_waypoint]
        setpoint.velocity = [float('nan')] * 3
        setpoint.acceleration = [float('nan')] * 3
        setpoint.jerk = [float('nan')] * 3
        # Preserve the old pose-command behavior: ENU yaw 0 becomes NED +pi/2.
        setpoint.yaw = float(0.0) # temporarily test NED 0 
        setpoint.yawspeed = float('nan')
        self.trajectory_pub.publish(setpoint)

    def _policy_tick(self) -> None:
        if not self.running:
            if self._last_waypoint is not None:
                self._publish_fmu_setpoint(self._last_waypoint)
            return

        if not self._inputs_fresh():
            self.get_logger().warn(
                'Waiting for fresh drone odom and ball state',
                throttle_duration_sec=2.0)
            return

        self._update_goal_reference()
        obs = self._build_observation()
        if obs is None:
            return

        drone_pos_pre = np.array([
            self._latest_drone_odom.pose.pose.position.x,
            self._latest_drone_odom.pose.pose.position.y,
            self._latest_drone_odom.pose.pose.position.z,
        ], dtype=np.float32)

        action, _ = self.policy.predict(obs, deterministic=self.deterministic)
        action = np.clip(
            np.asarray(action, dtype=np.float32),
            self.action_low,
            self.action_high,
        )
        waypoint = compute_bounded_waypoint(
            drone_pos_pre,
            action,
            self.bounds_min,
            self.bounds_max,
        )
        self._last_waypoint = waypoint

        self._publish_fmu_setpoint(waypoint)
        self._publish_debug(obs, action, waypoint)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
