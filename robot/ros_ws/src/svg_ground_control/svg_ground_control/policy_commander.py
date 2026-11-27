"""Run a trained drone_soccer PPO checkpoint on live mocap + PX4 odometry.

Supports standard PPO checkpoints and drone_soccer residual-policy checkpoints
(``policy_type: residual``), whose archive includes the frozen base actor.
"""

from __future__ import annotations

import hashlib
import json
from datetime import datetime
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
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

from drone_soccer.deploy.observation import (
    GOAL_ALIGNED_OBS_DIM,
    OBS_DIM,
    arrays_to_observation,
    batched_arrays_to_goal_aligned_observation,
    cap_observation_goal,
    quaternion_to_rotation_matrix,
)
from drone_soccer.deploy.waypoint import (
    compute_abs_waypoint,
    goal_aligned_actions_to_world,
)

from svg_ground_control.ball_state import BallStateTracker
from svg_ground_control.goal_retarget_bridge import (
    COMMANDER_LOG_FILE,
    DetailLog,
    PolicyManifestWatcher,
    TickRecorder,
    format_update_summary,
)
from svg_ground_control.policy_hot_swap import PolicyHotSwap
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


class RandomGoalSpawner:
    """Maintain one random XY goal inside an axis-aligned ENU rectangle.

    A new goal is sampled when the tracked position (drone or ball, the caller's
    choice) enters the arrival disk, or, with ``timeout_s > 0``, when the goal
    has been held that long. ``last_change`` says why the goal last changed.
    """

    def __init__(
        self,
        minimum_xy: np.ndarray,
        maximum_xy: np.ndarray,
        arrival_radius_m: float,
        seed: int | None = None,
        timeout_s: float = 0.0,
    ) -> None:
        self.minimum_xy = np.asarray(minimum_xy, dtype=np.float32).reshape(2)
        self.maximum_xy = np.asarray(maximum_xy, dtype=np.float32).reshape(2)
        if np.any(self.minimum_xy >= self.maximum_xy):
            raise ValueError(
                'goal_spawn_min_xy must be strictly less than '
                'goal_spawn_max_xy on both axes')
        if arrival_radius_m <= 0.0:
            raise ValueError('goal_arrival_radius must be positive')
        if timeout_s < 0.0:
            raise ValueError('goal_timeout_s must be non-negative')
        self.arrival_radius_m = float(arrival_radius_m)
        self.timeout_s = float(timeout_s)
        self._rng = np.random.default_rng(seed)
        self.last_change: str | None = None
        self._goal_start_s: float | None = None
        self.goal_xy = self._sample()

    def _sample(self) -> np.ndarray:
        return self._rng.uniform(
            self.minimum_xy,
            self.maximum_xy,
        ).astype(np.float32)

    def reset(self) -> np.ndarray:
        """Start a run with a newly sampled goal."""
        self.goal_xy = self._sample()
        self._goal_start_s = None
        return self.goal_xy.copy()

    def update(
        self, position_xy: np.ndarray, now_s: float | None = None,
    ) -> tuple[np.ndarray, bool]:
        """Resample on arrival of ``position_xy`` or when the goal timed out.

        Args:
            position_xy: tracked XY (drone or ball) compared with the goal.
            now_s: current time (s); needed for the timeout. The timeout
                clock of a goal starts at its first update.

        Returns:
            The current goal and whether it changed in this call.
        """
        position = np.asarray(position_xy, dtype=np.float32).reshape(2)
        if now_s is not None and self._goal_start_s is None:
            self._goal_start_s = float(now_s)
        reached = np.linalg.norm(position - self.goal_xy) <= self.arrival_radius_m
        timed_out = (
            self.timeout_s > 0.0 and now_s is not None
            and float(now_s) - self._goal_start_s >= self.timeout_s)
        if reached or timed_out:
            self.last_change = 'reached' if reached else 'timeout'
            self.goal_xy = self._sample()
            self._goal_start_s = None if now_s is None else float(now_s)
        return self.goal_xy.copy(), bool(reached or timed_out)


# policy_iterate of a checkpoint that did not come from the goal-retargeting learner.
EXTERNAL_ITERATE = -1


def _iterate_name(iterate: int) -> str:
    """Console name of a flying policy: its learner iterate, or 'external'."""
    return 'external' if iterate == EXTERNAL_ITERATE else str(iterate)


def _latched_qos() -> QoSProfile:
    """Reliable, transient-local depth-1 QoS: late subscribers get the last message."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


def _format_vector(values: np.ndarray) -> str:
    return '[' + ', '.join(f'{float(value):.3f}' for value in values) + ']'


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
    geofence_buffer_m: float = 0.5,
) -> np.ndarray:
    """Convert a relative policy action into a buffered absolute waypoint.

    Horizontal clamping activates only when the drone is within the configured
    distance of any XY geofence face (or is already outside the fence). This
    leaves the policy's XY waypoint unconstrained while the drone is safely in
    the interior. Altitude is always clamped to preserve the floor and ceiling.

    Args:
        drone_position: Current absolute ENU position in meters.
        policy_action: Policy output ``(dx, dy, z_world)`` in meters.
        bounds_min: Minimum allowed absolute ENU XYZ position.
        bounds_max: Maximum allowed absolute ENU XYZ position.
        geofence_buffer_m: Distance from an XY boundary at which horizontal
            waypoint clamping activates.

    Returns:
        Absolute ENU waypoint with buffered XY and unconditional Z clamping.
    """
    if not np.isfinite(geofence_buffer_m) or geofence_buffer_m < 0.0:
        raise ValueError('geofence_buffer_m must be finite and non-negative')

    drone = np.asarray(drone_position, dtype=np.float32).reshape(3)
    minimum = np.asarray(bounds_min, dtype=np.float32).reshape(3)
    maximum = np.asarray(bounds_max, dtype=np.float32).reshape(3)
    waypoint = compute_abs_waypoint(drone, policy_action)

    distance_to_xy_faces = np.concatenate((
        drone[:2] - minimum[:2],
        maximum[:2] - drone[:2],
    ))
    if np.any(distance_to_xy_faces <= geofence_buffer_m):
        return clamp_position(waypoint, minimum, maximum)

    waypoint[2] = np.clip(waypoint[2], minimum[2], maximum[2])
    return waypoint


POLICY_TYPES = ('standard', 'residual')

# drone_soccer's policy frames (``TrainConfig.policy_frame``), by the size of the
# observation the policy reads: "world" (18 values, actions in world axes) or
# "goal_aligned" (15 values, origin at the goal and x-axis toward the ball; actions
# along those axes). A checkpoint's frame is read from its observation size.
POLICY_FRAMES_BY_OBS_SIZE = {OBS_DIM: 'world', GOAL_ALIGNED_OBS_DIM: 'goal_aligned'}
# Key of the first actor layer in an SB3 ActorCriticPolicy state dict; its weight is
# (hidden, observation size).
_FIRST_ACTOR_LAYER_WEIGHT = 'mlp_extractor.policy_net.0.weight'


def policy_frame_of(policy: Any) -> str:
    """The drone_soccer policy frame a loaded policy acts in.

    Args:
        policy: an SB3 PPO model or ActorCriticPolicy (`load_policy_checkpoint`).

    Returns:
        ``world`` or ``goal_aligned``.
    """
    observation_size = int(policy.observation_space.shape[0])
    if observation_size not in POLICY_FRAMES_BY_OBS_SIZE:
        raise ValueError(
            f'policy reads {observation_size} observation values; expected one of '
            f'{sorted(POLICY_FRAMES_BY_OBS_SIZE)} ({POLICY_FRAMES_BY_OBS_SIZE})')
    return POLICY_FRAMES_BY_OBS_SIZE[observation_size]


def build_policy_observation(
    policy_frame: str,
    state: dict[str, np.ndarray],
    target_xy: np.ndarray,
    max_goal_distance_m: float,
) -> np.ndarray:
    """The observation a policy of ``policy_frame`` reads, as drone_soccer builds it.

    Both frames see the goal capped to ``max_goal_distance_m`` from the ball along its
    true direction; in the goal-aligned frame the capped goal is the frame's origin.

    Args:
        policy_frame: ``world`` or ``goal_aligned``.
        state: the tick's drone and ball state (`PolicyCommander._read_state`).
        target_xy: ``(2,)`` true ENU goal (m).
        max_goal_distance_m: largest ball-to-goal distance shown to the policy (m).

    Returns:
        ``(18,)`` or ``(15,)`` float32 observation.
    """
    rotation = quaternion_to_rotation_matrix(*state['drone_quat_wxyz'])
    if policy_frame == 'world':
        return cap_observation_goal(arrays_to_observation(
            state['drone_position'],
            state['drone_velocity'],
            rotation,
            state['drone_angular_velocity_body'],
            state['ball_position'],
            state['ball_velocity'],
            target_xy,
        ), max_goal_distance_m)
    if policy_frame != 'goal_aligned':
        raise ValueError(f'unknown policy frame {policy_frame!r}')
    return batched_arrays_to_goal_aligned_observation(
        np.reshape(state['drone_position'], (1, 3)),
        np.reshape(state['drone_velocity'], (1, 3)),
        np.reshape(rotation, (1, 3, 3)),
        np.reshape(state['drone_angular_velocity_body'], (1, 3)),
        np.reshape(state['ball_position'], (1, 3)),
        np.reshape(state['ball_velocity'], (1, 3)),
        np.reshape(target_xy, (1, 2)),
        max_goal_distance_m,
    )[0]


def policy_action_to_world(
    policy_frame: str,
    frame_action: np.ndarray,
    ball_xy: np.ndarray,
    target_xy: np.ndarray,
) -> np.ndarray:
    """A policy's (already clipped) action as the world ``(dx, dy, z_world)`` the waypoint takes.

    Args:
        policy_frame: ``world`` or ``goal_aligned``.
        frame_action: ``(3,)`` action in the policy's frame (m).
        ball_xy: ``(2,)`` ball ENU xy the observation was built from (m).
        target_xy: ``(2,)`` true ENU goal (m); the capped goal has the same direction.

    Returns:
        ``(3,)`` float32 world mixed-frame action.
    """
    if policy_frame == 'world':
        return np.asarray(frame_action, dtype=np.float32)
    return goal_aligned_actions_to_world(
        np.reshape(frame_action, (1, 3)),
        np.reshape(ball_xy, (1, 2)),
        np.reshape(target_xy, (1, 2)),
    )[0].astype(np.float32)


def _policy_class_for_type(policy_type: str) -> type:
    """Return the SB3 policy class a ``policy_type`` parameter names.

    Args:
        policy_type: ``standard`` for a plain PPO ``ActorCriticPolicy`` or
            ``residual`` for drone_soccer's ``ResidualActorCriticPolicy``
            (a frozen base actor plus a learned additive action residual).

    Returns:
        The policy class.
    """
    if policy_type == 'standard':
        from stable_baselines3.common.policies import ActorCriticPolicy
        return ActorCriticPolicy
    if policy_type == 'residual':
        # Imported only on demand so standard checkpoints still deploy against
        # drone_soccer checkouts that predate the residual policy.
        from drone_soccer.residual_policy import ResidualActorCriticPolicy
        return ResidualActorCriticPolicy
    raise ValueError(
        f'policy_type must be one of {POLICY_TYPES}, got {policy_type!r}')


def load_policy_checkpoint(
    model_path: str,
    action_low: np.ndarray,
    action_high: np.ndarray,
    policy_type: str = 'standard',
) -> tuple[Any, bool]:
    """Load an SB3 PPO checkpoint, including NumPy 2 to NumPy 1 compatibility.

    Args:
        model_path: Path to the Stable-Baselines3 PPO archive.
        action_low: Lower bounds for the three policy actions.
        action_high: Upper bounds for the three policy actions.
        policy_type: ``standard`` or ``residual``; must match the policy
            class the checkpoint was trained with. A residual checkpoint
            stores its frozen base actor inside the archive, so it is
            self-contained.

    Returns:
        The object exposing ``predict`` and whether the weight-only
        compatibility loader was required.
    """
    from stable_baselines3 import PPO

    policy_class = _policy_class_for_type(policy_type)

    try:
        model = PPO.load(model_path, device='cpu')
    except ModuleNotFoundError as exc:
        if not exc.name or not exc.name.startswith('numpy._core'):
            raise
    else:
        # The archive pickles its own policy class, so PPO.load succeeds for
        # either type. Check it agrees with the configuration (exact type: the
        # residual policy subclasses ActorCriticPolicy).
        if type(model.policy) is not policy_class:
            raise ValueError(
                f'{model_path} contains a {type(model.policy).__name__}, but '
                f'policy_type={policy_type!r} expects '
                f'{policy_class.__name__}')
        return model, False

    # Archives saved under NumPy 2 pickle module names that AirStack's
    # NumPy 1.x cannot resolve. Rebuild the configured policy class and load
    # its state dict without unpickling SB3 metadata. This assumes SB3's
    # default MLP (net_arch [64, 64], Tanh), which drone_soccer trains with;
    # the observation size (policy frame) is read from the first actor layer.
    import torch
    from gymnasium.spaces import Box

    with ZipFile(model_path) as archive:
        policy_bytes = archive.read('policy.pth')
    state_dict = torch.load(
        BytesIO(policy_bytes),
        map_location='cpu',
        weights_only=True,
    )
    if _FIRST_ACTOR_LAYER_WEIGHT not in state_dict:
        raise ValueError(
            f'{model_path} has no {_FIRST_ACTOR_LAYER_WEIGHT}; not an SB3 MLP policy')
    observation_space = Box(
        low=-np.inf,
        high=np.inf,
        shape=(int(state_dict[_FIRST_ACTOR_LAYER_WEIGHT].shape[1]),),
        dtype=np.float32,
    )
    action_space = Box(
        low=np.asarray(action_low, dtype=np.float32),
        high=np.asarray(action_high, dtype=np.float32),
        dtype=np.float32,
    )
    policy = policy_class(
        observation_space,
        action_space,
        lr_schedule=lambda _: 0.0,
    )
    try:
        policy.load_state_dict(state_dict)
    except RuntimeError as exc:
        has_base_actor = any(key.startswith('base_actor.') for key in state_dict)
        raise ValueError(
            f'Could not load {model_path} as policy_type={policy_type!r}; '
            'the checkpoint looks like a '
            f'{"residual" if has_base_actor else "standard"} policy. '
            f'Underlying error: {exc}') from exc
    policy.eval()
    return policy, True


class PolicyCommander(Node):
    def __init__(self) -> None:
        super().__init__('policy_commander')

        self.declare_parameter('drone', 'drone_3')
        self.declare_parameter('ball_name', 'SoccerBall')
        self.declare_parameter('model_path', '')
        # 'standard' (plain PPO) or 'residual' (drone_soccer
        # ResidualActorCriticPolicy: frozen base actor + learned residual).
        self.declare_parameter('policy_type', 'standard')
        self.declare_parameter('target_x', 3.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('goal_trajectory', 'fixed')
        self.declare_parameter('goal_radius', 0.0)
        self.declare_parameter('goal_period_s', 8.0)
        self.declare_parameter('goal_phase_rad', 0.0)
        self.declare_parameter('goal_rotation_rad', 0.0)
        self.declare_parameter('goal_spawn_min_xy', [-1.5, -1.5])
        self.declare_parameter('goal_spawn_max_xy', [1.5, 1.5])
        self.declare_parameter('goal_arrival_radius', 0.35)
        self.declare_parameter('goal_random_seed', -1)
        # Random goals switch when this reaches the arrival disk: "drone"
        # (the original behavior) or "ball" (as in drone_soccer's sim tests).
        self.declare_parameter('goal_arrival_reference', 'drone')
        # Random goals not reached within this time are replaced; 0 disables.
        self.declare_parameter('goal_timeout_s', 0.0)
        # Policies train on goals at most 4 m from the ball; farther goals are
        # shown to the policy pulled in to this distance along their direction.
        self.declare_parameter('max_goal_distance_m', 4.0)
        self.declare_parameter('policy_dt', 0.02)
        self.declare_parameter('deterministic', True)
        self.declare_parameter('action_low', [-3.0, -3.0, 0.3])
        self.declare_parameter('action_high', [3.0, 3.0, 2.0])
        self.declare_parameter('bounds_min', [-3.0, -3.0, 0.3])
        self.declare_parameter('bounds_max', [3.0, 3.0, 1.2])
        self.declare_parameter('geofence_buffer_m', 0.5)
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
        # Accept new checkpoints on <debug_topic_prefix>/load_policy and swap them in
        # between ticks (online policy-memory updates); off keeps the policy fixed.
        self.declare_parameter('enable_policy_hot_swap', False)
        # Online goal-retargeting updates (drone_soccer.goal_retarget): record
        # every flown tick for the host learner and hot-swap the policies it
        # publishes. See README "Online goal-retargeting updates".
        self.declare_parameter('goal_retarget_enabled', False)
        self.declare_parameter('goal_retarget_root', '/bags/goal_retarget')
        # false: record ticks only and keep flying the starting policy.
        self.declare_parameter('goal_retarget_hot_swap', True)

        self.drone = str(self.get_parameter('drone').value)
        ball_name = str(self.get_parameter('ball_name').value)
        model_path = str(self.get_parameter('model_path').value).strip()
        if not model_path:
            raise ValueError('model_path parameter is required (path to SB3 .zip)')
        if not Path(model_path).is_file():
            raise FileNotFoundError(f'model_path not found: {model_path}')
        self.policy_type = str(
            self.get_parameter('policy_type').value).strip().lower()
        if self.policy_type not in POLICY_TYPES:
            raise ValueError(
                f'policy_type must be one of {POLICY_TYPES}, got '
                f'{self.policy_type!r}')

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
        self.goal_arrival_reference = str(
            self.get_parameter('goal_arrival_reference').value).strip().lower()
        if self.goal_arrival_reference not in ('drone', 'ball'):
            raise ValueError(
                'goal_arrival_reference must be drone or ball, got '
                f'{self.goal_arrival_reference!r}')
        self._random_goal_spawner: RandomGoalSpawner | None = None
        if self.goal_trajectory == 'random':
            random_seed = int(self.get_parameter('goal_random_seed').value)
            self._random_goal_spawner = RandomGoalSpawner(
                self.get_parameter('goal_spawn_min_xy').value,
                self.get_parameter('goal_spawn_max_xy').value,
                float(self.get_parameter('goal_arrival_radius').value),
                None if random_seed < 0 else random_seed,
                float(self.get_parameter('goal_timeout_s').value),
            )
            self.target_xy = self._random_goal_spawner.goal_xy.copy()
        else:
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
        self.max_goal_distance_m = float(
            self.get_parameter('max_goal_distance_m').value)
        if self.max_goal_distance_m <= 0.0:
            raise ValueError('max_goal_distance_m must be positive')
        self.action_low = np.asarray(
            self.get_parameter('action_low').value, dtype=np.float32)
        self.action_high = np.asarray(
            self.get_parameter('action_high').value, dtype=np.float32)
        self.bounds_min = vector3(
            self.get_parameter('bounds_min').value, 'bounds_min')
        self.bounds_max = vector3(
            self.get_parameter('bounds_max').value, 'bounds_max')
        self.geofence_buffer_m = float(
            self.get_parameter('geofence_buffer_m').value)
        if (not np.isfinite(self.geofence_buffer_m)
                or self.geofence_buffer_m < 0.0):
            raise ValueError(
                'geofence_buffer_m must be finite and non-negative')
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

        self.get_logger().info(
            f'Loading {self.policy_type} PPO policy from {model_path}')
        self.policy, used_compat_loader = load_policy_checkpoint(
            model_path,
            self.action_low,
            self.action_high,
            self.policy_type,
        )
        if used_compat_loader:
            self.get_logger().warn(
                'Loaded policy weights with NumPy 2 checkpoint compatibility '
                'mode; SB3 metadata was not deserialized')
        # Every policy swapped in later must act in this frame too.
        self.policy_frame = policy_frame_of(self.policy)

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
        # The current goal is operational data, not optional debug data. Keep
        # publishing it even if observation/action telemetry is disabled.
        self.goal_topic = f'{prefix}/goal'
        self.goal_pub = self.create_publisher(
            Float32MultiArray, self.goal_topic, 10)
        if self.publish_debug:
            self.obs_pub = self.create_publisher(
                Float32MultiArray, f'{prefix}/obs', 10)
            self.action_pub = self.create_publisher(
                Float32MultiArray, f'{prefix}/action', 10)
            self.waypoint_pub = self.create_publisher(
                Float32MultiArray, f'{prefix}/waypoint', 10)

        self.create_service(Trigger, '~/start', self._handle_start)
        self.create_service(Trigger, '~/stop', self._handle_stop)

        policy_dt = float(self.get_parameter('policy_dt').value)
        if policy_dt <= 0.0:
            raise ValueError('policy_dt must be positive')
        self.policy_dt = policy_dt
        self._init_policy_updates(model_path)
        self._setup_policy_hot_swap(prefix)
        self._setup_goal_retarget(prefix, model_path, ball_name)
        self.create_timer(policy_dt, self._policy_tick)

        self.get_logger().info(
            f'PolicyCommander ready | drone={self.drone} ball={ball_name} '
            f'odom={odom_topic} ball_odom={ball_odom_topic} '
            f'trajectory_out={trajectory_topic} '
            f'offboard_out={offboard_mode_topic} '
            f'policy_frame={self.policy_frame} '
            f'policy_type={self.policy_type} '
            f'goal={self.goal_trajectory} goal_topic={self.goal_topic} '
            f'center={self.goal_center_xy.tolist()} '
            f'radius={self.goal_radius:.3f}m period={self.goal_period_s:.3f}s '
            f'rotation={self.goal_rotation_rad:.3f}rad '
            f'max_goal_distance={self.max_goal_distance_m:.3f}m '
            f'geofence_buffer={self.geofence_buffer_m:.3f}m '
            f'goal_arrival={self.goal_arrival_reference} '
            f'goal_retarget={self._recorder is not None}')

    def _handle_start(self, _request, response):
        self._goal_reference_start = self.get_clock().now()
        if self._random_goal_spawner is not None:
            self.target_xy = self._random_goal_spawner.reset()
            self.get_logger().info(
                f'New random goal: {self.target_xy.tolist()}')
        else:
            self.target_xy = compute_goal_reference(
                self.goal_trajectory,
                0.0,
                self.goal_center_xy,
                self.goal_radius,
                self.goal_period_s,
                self.goal_phase_rad,
                self.goal_rotation_rad,
            )
        self._publish_goal()
        self.running = True
        response.success = True
        response.message = 'policy loop publishing enabled'
        if self._recorder is not None:
            response.message += (
                f' (goal-retarget: flying iterate {self.policy_iterate})')
        self.get_logger().info(response.message)
        return response

    def _handle_stop(self, _request, response):
        self.running = False
        self._end_recorded_segment('policy stopped')
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
        if self._random_goal_spawner is not None:
            if self.goal_arrival_reference == 'ball':
                ball = self._ball_tracker.state
                if ball is None:
                    return
                position_xy = np.asarray(ball.position, dtype=np.float32)[:2]
            else:
                odom = self._latest_drone_odom
                if odom is None:
                    return
                position = odom.pose.pose.position
                position_xy = np.array([position.x, position.y], dtype=np.float32)
            self.target_xy, changed = self._random_goal_spawner.update(
                position_xy, self.get_clock().now().nanoseconds * 1e-9)
            if changed:
                reason = self._random_goal_spawner.last_change
                self._iterate_stats[reason] += 1
                self.get_logger().info(
                    f'Goal {reason} ({self.goal_arrival_reference}); spawned '
                    f'new random goal: {self.target_xy.tolist()}')
            return
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

    def _read_state(self) -> dict[str, np.ndarray] | None:
        """The drone and ball state the policy acts on this tick."""
        odom = self._latest_drone_odom
        ball = self._ball_tracker.state
        if odom is None or ball is None:
            return None

        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        t = odom.twist.twist
        return {
            'drone_position': np.array([p.x, p.y, p.z], dtype=np.float32),
            'drone_quat_wxyz': np.array([q.w, q.x, q.y, q.z], dtype=np.float64),
            'drone_velocity': np.array(
                [t.linear.x, t.linear.y, t.linear.z], dtype=np.float32),
            'drone_angular_velocity_body': np.array(
                [t.angular.x, t.angular.y, t.angular.z], dtype=np.float32),
            'ball_position': np.asarray(ball.position),
            'ball_velocity': np.asarray(ball.velocity),
        }

    def _observation_from_state(self, state: dict[str, np.ndarray]) -> np.ndarray:
        # The published goal stays the true goal; only the policy's view of
        # it is capped, matching drone_soccer.
        return build_policy_observation(
            self.policy_frame, state, self.target_xy, self.max_goal_distance_m)

    def _publish_goal(self) -> None:
        goal_msg = Float32MultiArray()
        goal_msg.data = [float(x) for x in self.target_xy]
        self.goal_pub.publish(goal_msg)

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
        setpoint.yaw = float(np.pi / 2) # temporarily test NED 0
        setpoint.yawspeed = float('nan')
        self.trajectory_pub.publish(setpoint)

    # --- Policy hot swap (online policy memory and goal retargeting) -------

    def _init_policy_updates(self, model_path: str) -> None:
        """State shared by both policy-update sources; both start disabled."""
        self._hot_swap: PolicyHotSwap | None = None
        self.active_policy_pub = None
        self.policy_update_pub = None
        self._recorder: TickRecorder | None = None
        self._detail_log: DetailLog | None = None
        self._manifest_watcher: PolicyManifestWatcher | None = None
        # Whether iterates the goal-retargeting learner publishes are loaded.
        self._goal_retarget_swaps = False
        self._pending_manifests: dict[str, dict] = {}
        self.policy_iterate = 0
        self.policy_path = model_path
        self._iterate_stats = {'ticks': 0, 'reached': 0, 'timeout': 0}

    def _ensure_hot_swap(self, prefix: str) -> None:
        """Create the background loader and the latched <prefix>/active_policy path topic once."""
        if self._hot_swap is not None:
            return
        self._hot_swap = PolicyHotSwap(self._load_swappable_policy)
        self.active_policy_pub = self.create_publisher(
            String, f'{prefix}/active_policy', _latched_qos())
        self._publish_active_policy_path()

    def _load_swappable_policy(self, path: str) -> Any:
        """Load a checkpoint to swap in; refuse one that acts in another policy frame."""
        policy = load_policy_checkpoint(
            path, self.action_low, self.action_high, self.policy_type)[0]
        frame = policy_frame_of(policy)
        if frame != self.policy_frame:
            raise ValueError(
                f'checkpoint acts in the {frame} policy frame; the flying policy '
                f'acts in the {self.policy_frame} frame')
        return policy

    def _publish_active_policy_path(self) -> None:
        if self.active_policy_pub is None:
            return
        msg = String()
        msg.data = self.policy_path
        self.active_policy_pub.publish(msg)

    def _setup_policy_hot_swap(self, prefix: str) -> None:
        """Accept checkpoint paths on <prefix>/load_policy (online policy memory), if enabled."""
        if not bool(self.get_parameter('enable_policy_hot_swap').value):
            return
        self._ensure_hot_swap(prefix)
        self.create_subscription(
            String, f'{prefix}/load_policy', self._load_policy_cb, 10)
        self.get_logger().info(
            f'Policy hot swap enabled on {prefix}/load_policy')

    def _load_policy_cb(self, msg: String) -> None:
        self.get_logger().info(f'Loading requested policy {msg.data}')
        self._hot_swap.request(msg.data)

    def _policy_action(self, policy: Any, obs: np.ndarray,
                       state: dict[str, np.ndarray]) -> np.ndarray:
        """The world ``(dx, dy, z_world)`` action of ``policy`` on this tick.

        The action is clipped to the action box in the policy's own frame, then
        expressed in world axes, as drone_soccer flies it.
        """
        action, _ = policy.predict(obs, deterministic=self.deterministic)
        frame_action = np.clip(
            np.asarray(action, dtype=np.float32),
            self.action_low,
            self.action_high,
        )
        return policy_action_to_world(
            self.policy_frame, frame_action, state['ball_position'][:2], self.target_xy)

    def _policy_tick(self) -> None:
        self._poll_learner_updates()
        if not self.running:
            self._adopt_loaded_policy(None, None)
            self._publish_goal()
            if self._last_waypoint is not None:
                self._publish_fmu_setpoint(self._last_waypoint)
            return

        if not self._inputs_fresh():
            self._end_recorded_segment('stale drone odom or ball state')
            self._adopt_loaded_policy(None, None)
            self._publish_goal()
            self.get_logger().warn(
                'Waiting for fresh drone odom and ball state',
                throttle_duration_sec=2.0)
            return

        self._update_goal_reference()
        self._publish_goal()
        state = self._read_state()
        if state is None:
            self._end_recorded_segment('missing drone odom or ball state')
            return
        obs = self._observation_from_state(state)
        # Swap between computing the observation and acting on it, so the
        # log can compare both policies' actions on this exact state.
        self._adopt_loaded_policy(obs, state)

        drone_pos_pre = state['drone_position']
        action = self._policy_action(self.policy, obs, state)
        waypoint = compute_bounded_waypoint(
            drone_pos_pre,
            action,
            self.bounds_min,
            self.bounds_max,
            self.geofence_buffer_m,
        )
        self._last_waypoint = waypoint

        self._publish_fmu_setpoint(waypoint)
        self._publish_debug(obs, action, waypoint)
        self._iterate_stats['ticks'] += 1
        self._record_tick(state, action, waypoint)

    # --- Online goal-retargeting updates ----------------------------------

    def _setup_goal_retarget(
            self, prefix: str, model_path: str, ball_name: str) -> None:
        """Open the learner run directory and hot-swap machinery, if enabled."""
        if not bool(self.get_parameter('goal_retarget_enabled').value):
            return

        # Imported here so plain inference does not depend on it. The learner
        # (drone_soccer.goal_retarget.plan_learner) reads these settings.
        from drone_soccer.goal_retarget_config import PLAN_UPDATE
        learner_config = PLAN_UPDATE.forward

        if self.policy_type != 'standard':
            raise ValueError(
                'goal_retarget_enabled needs policy_type standard (the learner '
                f'adapts a plain PPO actor), got {self.policy_type!r}')
        if abs(self.policy_dt - learner_config.policy_dt) > 1e-9:
            raise ValueError(
                f'goal_retarget_enabled needs policy_dt {learner_config.policy_dt}'
                f' (one learner frame per tick), got {self.policy_dt}')
        if not self.deterministic:
            raise ValueError('goal_retarget_enabled needs deterministic: true')
        if abs(self.max_goal_distance_m - learner_config.max_goal_distance_m) > 1e-9:
            raise ValueError(
                'goal_retarget_enabled needs max_goal_distance_m '
                f'{learner_config.max_goal_distance_m}, got '
                f'{self.max_goal_distance_m}')
        if self._random_goal_spawner is not None and (
                self.goal_arrival_reference != 'ball'
                or abs(self._random_goal_spawner.arrival_radius_m
                       - learner_config.reach_radius_m) > 1e-9):
            self.get_logger().warn(
                'Goal-retarget learning was tested with goals that switch when '
                f'the ball is within {learner_config.reach_radius_m} m; this run '
                f'switches on the {self.goal_arrival_reference} within '
                f'{self._random_goal_spawner.arrival_radius_m} m')

        model_sha256 = hashlib.sha256(Path(model_path).read_bytes()).hexdigest()
        session = {
            'created': datetime.now().isoformat(timespec='seconds'),
            'drone': self.drone,
            'ball_name': ball_name,
            'model_path': model_path,
            'model_sha256': model_sha256,
            'policy_type': self.policy_type,
            'policy_frame': self.policy_frame,
            'policy_dt': self.policy_dt,
            'max_goal_distance_m': self.max_goal_distance_m,
            'action_low': self.action_low.tolist(),
            'action_high': self.action_high.tolist(),
            'goal_trajectory': self.goal_trajectory,
            'goal_arrival_reference': self.goal_arrival_reference,
            'goal_timeout_s': float(self.get_parameter('goal_timeout_s').value),
            'frame': 'world ENU',
        }
        root = str(self.get_parameter('goal_retarget_root').value)
        run_name = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{self.drone}"
        self._recorder = TickRecorder(root, run_name, session)
        self._detail_log = DetailLog(self._recorder.run_dir / COMMANDER_LOG_FILE)
        self._manifest_watcher = PolicyManifestWatcher(self._recorder.run_dir)
        self._goal_retarget_swaps = bool(
            self.get_parameter('goal_retarget_hot_swap').value)
        if self._goal_retarget_swaps:
            self._ensure_hot_swap(prefix)
        self.policy_update_topic = f'{prefix}/policy_update'
        self.policy_update_pub = self.create_publisher(
            String, self.policy_update_topic, _latched_qos())
        self._publish_policy_update(None)

        run_dir = str(self._recorder.run_dir)
        host_hint = ''
        if run_dir.startswith('/bags/'):
            host_hint = f' (host: AirStack/robot/bags/{run_dir[len("/bags/"):]})'
        self._detail(
            f'goal-retarget learning: recording ticks to {run_dir}{host_hint}; '
            f'hot swap {"on" if self._goal_retarget_swaps else "OFF"}; policy '
            f'changes are announced on {self.policy_update_topic}; session {session}')
        self.get_logger().info(
            f'Goal-retarget learning ON (hot swap '
            f'{"on" if self._goal_retarget_swaps else "OFF"}): run {run_name}, details in '
            f'{run_dir}/{COMMANDER_LOG_FILE}')

    def _detail(self, message: str) -> None:
        """Write to the run's detailed log only (not the console)."""
        if self._detail_log is not None:
            self._detail_log.write(message)

    def _record_tick(self, state: dict[str, np.ndarray], action: np.ndarray,
                     waypoint: np.ndarray) -> None:
        if self._recorder is None:
            return
        opening = not self._recorder.segment_open
        segment, _ = self._recorder.record(
            self.get_clock().now().nanoseconds * 1e-9,
            self.policy_iterate,
            goal_xy=self.target_xy,
            action=action,
            waypoint=waypoint,
            **state,
        )
        if opening:
            self._detail(
                f'recording segment {segment} (policy iterate {self.policy_iterate})')

    def _end_recorded_segment(self, reason: str) -> None:
        if self._recorder is None:
            return
        segment = self._recorder.segment
        if self._recorder.end_segment():
            self._detail(f'segment {segment} ended ({reason})')

    def _poll_learner_updates(self) -> None:
        """Start loading an iterate the learner just published."""
        if self._manifest_watcher is None:
            return
        manifest = self._manifest_watcher.poll()
        if manifest is None:
            return
        iterate = int(manifest['iterate'])
        summary = format_update_summary(manifest.get('update') or {})
        if not self._goal_retarget_swaps:
            self._detail(
                f'learner published iterate {iterate} ({summary}); hot swap '
                f'is off, still flying iterate {self.policy_iterate}')
            self.get_logger().info(
                f'Learner iterate {iterate} ready; hot swap off, still flying '
                f'iterate {self.policy_iterate}')
            return
        path = manifest['policy_path']
        self._pending_manifests[path] = manifest
        self._detail(f'learner published iterate {iterate}; loading {path}')
        self._hot_swap.request(path)

    def _adopt_loaded_policy(self, obs: np.ndarray | None,
                             state: dict[str, np.ndarray] | None) -> None:
        """Swap in a policy loaded since the last tick and log the change.

        ``obs`` and the ``state`` it was built from are this tick's, or None
        while the policy is paused or its inputs are stale.
        """
        if self._hot_swap is None:
            return
        for error in self._hot_swap.take_errors():
            self._detail(f'policy load failed; still flying iterate '
                         f'{self.policy_iterate}: {error}')
            self.get_logger().error(
                f'Policy load failed, still flying iterate '
                f'{self.policy_iterate}: {error}')
        loaded = self._hot_swap.take_ready()
        if loaded is None:
            return
        manifest = self._pending_manifests.pop(loaded.path, {})
        # A checkpoint requested on load_policy (online policy memory) is no
        # goal-retargeting iterate: it flies as EXTERNAL_ITERATE.
        new_iterate = int(manifest.get('iterate', EXTERNAL_ITERATE))
        for path in [path for path, pending in self._pending_manifests.items()
                     if int(pending['iterate']) < new_iterate]:
            del self._pending_manifests[path]

        old_iterate = self.policy_iterate
        old_policy = self.policy
        stats = self._iterate_stats
        if obs is not None:
            old_action = self._policy_action(old_policy, obs, state)
            new_action = self._policy_action(loaded.policy, obs, state)
        self.policy = loaded.policy
        self.policy_iterate = new_iterate
        self.policy_path = loaded.path
        self._iterate_stats = {'ticks': 0, 'reached': 0, 'timeout': 0}

        flown_s = stats['ticks'] * self.policy_dt
        first_tick = ''
        if self._recorder is not None:
            segment, tick = self._recorder.next_position
            first_tick = f'; first tick: segment {segment} tick {tick}'
        details = [
            f'POLICY UPDATE: now flying iterate {new_iterate} '
            f'({loaded.path}), replacing iterate {old_iterate}{first_tick}'
            + ('' if self.running else ' (policy paused)'),
            f'  learner update {new_iterate}: '
            f"{format_update_summary(manifest.get('update') or {})}",
        ]
        if obs is not None:
            delta = float(np.linalg.norm(new_action - old_action))
            details.append(
                f'  action on this state: iterate {old_iterate} '
                f'{_format_vector(old_action)} -> iterate {new_iterate} '
                f'{_format_vector(new_action)} (|change| {delta:.3f} m)')
        details.append(
            f'  iterate {old_iterate} flew {flown_s:.1f} s:'
            f' {stats["reached"]} goals reached, {stats["timeout"]} timed out')
        self._detail('\n'.join(details))

        change = (f'action change {delta:.2f} m' if obs is not None
                  else 'policy paused')
        self.get_logger().warn(
            f'POLICY UPDATE {_iterate_name(old_iterate)} -> '
            f'{_iterate_name(new_iterate)} | {change} | '
            f'iterate {_iterate_name(old_iterate)}: {flown_s:.0f} s, '
            f'{stats["reached"]} goals')
        if self._recorder is None:
            # Online policy-memory swaps have no detail log; keep the path on screen.
            self.get_logger().info(f'Swapped in policy {loaded.path}')
        self._publish_active_policy_path()
        self._publish_policy_update(manifest.get('update'))

    def _publish_policy_update(self, update: dict | None) -> None:
        """Latched JSON event of the flying goal-retargeting iterate (learning runs only)."""
        if self.policy_update_pub is None:
            return
        segment, tick = self._recorder.next_position
        msg = String()
        msg.data = json.dumps({
            'iterate': self.policy_iterate,
            'path': self.policy_path,
            'ros_time_s': self.get_clock().now().nanoseconds * 1e-9,
            'first_segment': segment,
            'first_tick': tick,
            'update': update,
        })
        self.policy_update_pub.publish(msg)

    def destroy_node(self):
        if getattr(self, '_recorder', None) is not None:
            self._recorder.close()
        if getattr(self, '_detail_log', None) is not None:
            self._detail_log.close()
        return super().destroy_node()


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
