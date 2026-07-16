"""Single-drone DiffAero ground commander — VELOCITY-command policy.

Sibling of ``diffaero_commander.py`` (the attitude+thrust commander). This one
drives a DiffAero policy that was trained/exported with
``dynamics=velocity_pointmass`` (``action_is_velocity=true``): the policy emits a
world-ENU velocity setpoint instead of an attitude+thrust setpoint, and the
flight controller owns attitude. See ``diffaero/diffaero_vel_core.py``.

Interfaces (topic templates come from the config YAML):

    state in:    {state_topic_template}               nav_msgs/Odometry (ENU)
    vel cmd out: {velocity_command_topic_template}    geometry_msgs/TwistStamped
    pos cmd out: {position_command_topic_template}    geometry_msgs/PoseStamped
    services:    {robot_command_service_template}     airstack_msgs/srv/RobotCommand

drone_mode selects the interface AND the velocity frame:
    'sim'  -> /{name}/interface/* (MAVROS/SITL); velocity_command is BODY frame
              (FRAME_BODY_NED), so the world-ENU policy velocity is rotated into
              the yaw-aligned body frame before publishing.
    'real' -> /{name}/fmu/*       (px4_interface/uXRCE-DDS); velocity_command is
              world ENU, so the policy velocity is published unchanged.

Yaw: the policy is trained yaw-aligned with travel direction. The commander runs
a P-controller from the current heading toward the policy's desired heading and
publishes the result as a yaw-rate (TwistStamped angular.z, ENU CCW+).

Lifecycle (std_srvs/Trigger services), identical to the attitude commander:
    ~/takeoff — arm + offboard + ascend to hover_position, then HOLD
    ~/start   — begin the scenario
    ~/hold    — freeze at current position (panic button)
    ~/land    — descend, disarm on touchdown
    ~/reset_fence — clear a latched geofence breach
"""

from enum import Enum
import math

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, Float32, Float32MultiArray
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from airstack_msgs.srv import RobotCommand

from svg_ground_control.scenarios import Bounds, make_scenario
from svg_ground_control.diffaero.diffaero_core import DiffAeroObs
from svg_ground_control.diffaero.diffaero_vel_core import DiffAeroVelPolicy
from svg_ground_control.diffaero.perception_builder import Intrinsics


class FlightState(Enum):
    IDLE = 0
    ARMING = 1
    ASCEND = 2
    FACE_GOAL = 3
    ACTIVE = 4
    LANDING = 5


ARMING_OFFBOARD_S = 1.0
ARMING_ARM_S = 1.5
ARMING_DONE_S = 2.5

TOF_GRID_H = 9
TOF_GRID_W = 16


class DroneHandle:
    """Book-keeping for one drone."""

    def __init__(self, name: str, mode: str):
        self.name = name
        self.mode = mode                  # 'sim' | 'real'
        self.position_offset = np.zeros(3)
        self.takeoff_target = None        # np (3,)
        self.hold_target = None           # np (3,)
        self.hold_orientation = np.array([0., 0., 0., 1.])  # set when FACE_GOAL completes
        self.face_cmd_yaw = None          # slew-limited commanded yaw during FACE_GOAL
        self.state = FlightState.IDLE
        self.position = None              # np (3,) ENU, None until first odometry
        self.velocity = np.zeros(3)
        self.orientation = np.array([0., 0., 0., 1.])  # xyzw quaternion
        self.last_odom_time = None
        self.arming_start = None
        self.arming_steps_done = set()
        self.cmd_pub = None               # velocity publisher (policy + landing)
        self.pos_cmd_pub = None           # position publisher (ascend / hold / fallback)
        self.vel_cmd_enu = None           # latest world-ENU velocity command, for viz (None when pose-commanding)
        self.robot_command_client = None
        self.tof: np.ndarray | None = None        # latest 9×16 pre-encoded perception
        self.tof_raw: np.ndarray | None = None    # latest oriented planar-Z (debug)
        self.last_tof_time = None


class DiffAeroVelocityCommander(Node):

    def __init__(self, **kwargs):
        super().__init__('diffaero_velocity_commander', **kwargs)

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('drone_name', 'drone_1')
        self.declare_parameter('drone_mode', 'sim')  # 'sim' | 'real'

        # Frame of the odometry twist on state_topic_template. 'auto' picks the
        # right one per interface (sim MAVROS = body, real px4_interface =
        # world); override only to work around a nonstandard odometry source.
        self.declare_parameter('odom_twist_frame', 'auto')  # 'auto' | 'body' | 'world'

        self.declare_parameter('scenario', 'hover')
        self.declare_parameter('scenario_speed_mps', 0.6)
        self.declare_parameter('scenario_seed', 7)
        self.declare_parameter('arena_low', [-2.0, -2.0, 0.8])
        self.declare_parameter('arena_high', [2.0, 2.0, 2.0])
        self.declare_parameter('hover_positions', [0.0, 0.0, 1.2])
        self.declare_parameter('goal_position', [2.0, 0.0, 1.2])

        # Single-drone position offset [x, y, z] added to odometry to shift
        # into the world frame. In sim set to the spawn position; with mocap
        # leave at zeros.
        self.declare_parameter('drone_position_offset', [0.0, 0.0, 0.0])

        self.declare_parameter('state_topic_template',
                               '/{name}/odometry_conversion/odometry')
        self.declare_parameter('sim_velocity_command_topic_template',
                               '/{name}/interface/velocity_command')
        self.declare_parameter('sim_position_command_topic_template',
                               '/{name}/interface/pose_command')
        self.declare_parameter('sim_robot_command_service_template',
                               '/{name}/interface/robot_command')
        self.declare_parameter('real_velocity_command_topic_template',
                               '/{name}/fmu/velocity_command')
        self.declare_parameter('real_position_command_topic_template',
                               '/{name}/fmu/pose_command')
        self.declare_parameter('real_robot_command_service_template',
                               '/{name}/fmu/robot_command')

        self.declare_parameter('goal_command_topic_template',
                               '/svg/{name}/goal_command')
        self.declare_parameter('speed_command_topic_template',
                               '/svg/{name}/speed_command')

        self.declare_parameter('tof_topic_template',
                               '/{name}/perception/tof')
        self.declare_parameter('tof_raw_topic_template',
                               '')  # optional; TOF3 debug only
        self.declare_parameter('tof_image_topic_template',
                               '/svg/{name}/tof_image')
        self.declare_parameter('tof_viz_max_dist_m', 5.0)
        self.declare_parameter('tof_viz_scale', 16)  # panel height = 9 * scale px
        self.declare_parameter('tof_viz_side_by_side', False)  # debug: raw | processed
        self.declare_parameter('tof_raw_pre_oriented', True)  # UDP TOF3 is pre-oriented

        self.declare_parameter('checkpoint_path', '')
        # Velocity-policy action limits + target-vel saturation. Leave at -1 to
        # fall back to the values baked into the checkpoint's training config.
        self.declare_parameter('max_vel', 5.0)       # target_vel saturation toward goal
        self.declare_parameter('max_vel_xy', -1.0)   # actor action limit (xy); -1 -> training default
        self.declare_parameter('max_vel_z', -1.0)    # actor action limit (z);  -1 -> training default
        self.declare_parameter('vel_ema_factor', -1.0)  # -1 -> training default
        self.declare_parameter('tof_timeout_s', 0.5)

        self.declare_parameter('fence_enabled', False)
        self.declare_parameter('fence_min', [-1000.0, -1000.0, -1000.0])
        self.declare_parameter('fence_max', [1000.0, 1000.0, 1000.0])

        self.declare_parameter('publish_viz', True)
        self.declare_parameter('viz_frame', 'map')
        # Velocity-command arrow length = vel_cmd (m/s) * this (s). 0.5 => the
        # arrow tip shows where the command would carry the drone in 0.5 s.
        self.declare_parameter('vel_arrow_scale_s', 0.5)

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('state_timeout_s', 0.5)
        self.declare_parameter('hover_kp', 1.0)
        self.declare_parameter('yaw_kp', 1.5)        # yaw-rate P gain (1/s)
        self.declare_parameter('yaw_rate_max', 1.5)  # rad/s clamp on commanded yaw-rate
        self.declare_parameter('arrival_threshold_m', 0.15)
        self.declare_parameter('goal_arrival_threshold_m', 0.4)
        self.declare_parameter('ascend_speed_mps', 0.5)
        self.declare_parameter('land_speed_mps', 0.3)
        self.declare_parameter('land_complete_altitude_m', 0.15)
        self.declare_parameter('face_goal_threshold_rad', 0.05)
        self.declare_parameter('face_goal_yaw_rate_max', 0.8)  # rad/s slew limit while turning to face goal

        # ---- Read parameters --------------------------------------------
        name = str(self.get_parameter('drone_name').value)
        mode = str(self.get_parameter('drone_mode').value)
        if mode not in ('sim', 'real'):
            raise ValueError(f'drone_mode must be sim|real, got "{mode}"')
        self.drone_mode = mode

        # The two interfaces disagree on the odometry twist frame, so resolve it
        # from the mode rather than trusting child_frame_id (see
        # odometry_callback). 'auto': sim -> body, real -> world.
        twist_frame = str(self.get_parameter('odom_twist_frame').value)
        if twist_frame not in ('auto', 'body', 'world'):
            raise ValueError(
                f'odom_twist_frame must be auto|body|world, got "{twist_frame}"')
        if twist_frame == 'auto':
            twist_frame = 'body' if mode == 'sim' else 'world'
        self.odom_twist_frame = twist_frame

        offset = list(self.get_parameter('drone_position_offset').value)
        if len(offset) != 3:
            raise ValueError(
                f'drone_position_offset needs 3 values, got {len(offset)}')

        self.fence_enabled = bool(self.get_parameter('fence_enabled').value)
        self.fence_min = np.array(self.get_parameter('fence_min').value, dtype=float)
        self.fence_max = np.array(self.get_parameter('fence_max').value, dtype=float)
        self.fence_breached = False

        self.state_timeout = float(self.get_parameter('state_timeout_s').value)
        self.hover_kp = float(self.get_parameter('hover_kp').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.yaw_rate_max = float(self.get_parameter('yaw_rate_max').value)
        self.arrival_threshold = float(self.get_parameter('arrival_threshold_m').value)
        self.goal_arrival_threshold = float(
            self.get_parameter('goal_arrival_threshold_m').value)
        self.ascend_speed = float(self.get_parameter('ascend_speed_mps').value)
        self.land_speed = float(self.get_parameter('land_speed_mps').value)
        self.land_complete_alt = float(
            self.get_parameter('land_complete_altitude_m').value)
        self.face_goal_threshold = float(
            self.get_parameter('face_goal_threshold_rad').value)
        self.face_goal_yaw_rate_max = float(
            self.get_parameter('face_goal_yaw_rate_max').value)
        self.control_dt = 1.0 / float(self.get_parameter('control_rate_hz').value)

        self.goal_position = np.array(
            self.get_parameter('goal_position').value, dtype=float)

        # ---- Scenario ---------------------------------------------------
        scenario_name = str(self.get_parameter('scenario').value)
        scenario_kwargs = {}
        if scenario_name in ('hover', 'goal'):
            scenario_kwargs['hover_positions'] = np.array(
                self.get_parameter('hover_positions').value)
            if scenario_name == 'goal':
                scenario_kwargs['initial_goals'] = scenario_kwargs.pop('hover_positions')
        self.scenario = make_scenario(
            scenario_name,
            num_drones=1,
            nominal_speed=float(self.get_parameter('scenario_speed_mps').value),
            bounds=Bounds(
                low=np.array(self.get_parameter('arena_low').value),
                high=np.array(self.get_parameter('arena_high').value)),
            seed=int(self.get_parameter('scenario_seed').value),
            safety_radius=0.0,
            **scenario_kwargs)
        self.scenario_name = scenario_name
        self.mission_active = False

        # ---- Topic templates --------------------------------------------
        state_tmpl = str(self.get_parameter('state_topic_template').value)
        if mode == 'real':
            vel_tmpl = str(self.get_parameter('real_velocity_command_topic_template').value)
            pos_tmpl = str(self.get_parameter('real_position_command_topic_template').value)
            srv_tmpl = str(self.get_parameter('real_robot_command_service_template').value)
        else:
            vel_tmpl = str(self.get_parameter('sim_velocity_command_topic_template').value)
            pos_tmpl = str(self.get_parameter('sim_position_command_topic_template').value)
            srv_tmpl = str(self.get_parameter('sim_robot_command_service_template').value)

        goal_tmpl = str(self.get_parameter('goal_command_topic_template').value)
        speed_tmpl = str(self.get_parameter('speed_command_topic_template').value)
        tof_tmpl = str(self.get_parameter('tof_topic_template').value)
        tof_raw_tmpl = str(self.get_parameter('tof_raw_topic_template').value)
        tof_image_tmpl = str(self.get_parameter('tof_image_topic_template').value)
        self.tof_viz_max_dist = float(self.get_parameter('tof_viz_max_dist_m').value)
        self.tof_viz_scale = max(1, int(self.get_parameter('tof_viz_scale').value))
        self.tof_viz_side_by_side = bool(self.get_parameter('tof_viz_side_by_side').value)
        self.tof_raw_pre_oriented = bool(self.get_parameter('tof_raw_pre_oriented').value)

        self.tof_timeout = float(self.get_parameter('tof_timeout_s').value)

        # ---- Drone wiring -----------------------------------------------
        takeoff_target = self.scenario.initial_positions()[0]
        self.drone = DroneHandle(name, mode)
        self.drone.position_offset = np.array(offset)
        self.drone.takeoff_target = takeoff_target.copy()
        self.drone.hold_target = takeoff_target.copy()
        self.drone.cmd_pub = self.create_publisher(
            TwistStamped, vel_tmpl.format(name=name), 10)
        self.drone.pos_cmd_pub = self.create_publisher(
            PoseStamped, pos_tmpl.format(name=name), 10)
        self.drone.robot_command_client = self.create_client(
            RobotCommand, srv_tmpl.format(name=name))

        self.create_subscription(
            Odometry, state_tmpl.format(name=name),
            self.odometry_callback, 10)
        self.create_subscription(
            Float32MultiArray, tof_tmpl.format(name=name),
            self.tof_callback, 10)

        if scenario_name == 'goal':
            self.create_subscription(
                PoseStamped, goal_tmpl.format(name=name),
                self.goal_callback, 10)
            self.create_subscription(
                Float32, speed_tmpl.format(name=name),
                self.speed_callback, 10)

        # Keep self.drones as a list for methods that iterate (markers, fence).
        self.drones = [self.drone]

        # ---- DiffAero velocity policy -----------------------------------
        checkpoint_path = str(self.get_parameter('checkpoint_path').value)
        self.policy: DiffAeroVelPolicy | None = None
        self.policy_goal: np.ndarray | None = None   # fixed at ~/start; goal = position → hover
        self._policy_last_time = None                 # for gap-detection auto-reset

        def _opt(pname):
            v = float(self.get_parameter(pname).value)
            return v if v > 0 else None

        if checkpoint_path:
            # Dummy intrinsics — PerceptionBuilder is bypassed because the ToF
            # topic sends a pre-encoded 9×16 grid (perception_encoded path).
            dummy_intrinsics = Intrinsics(fx=1.0, fy=1.0, cx=0.5, cy=0.5, H=1, W=1)
            self.policy = DiffAeroVelPolicy(
                intrinsics=dummy_intrinsics,
                checkpoint_path=checkpoint_path,
                vel_ema_factor=_opt('vel_ema_factor'),
                max_vel=float(self.get_parameter('max_vel').value),
                max_vel_xy=_opt('max_vel_xy'),
                max_vel_z=_opt('max_vel_z'),
            )
            self.get_logger().info(f'DiffAero velocity policy loaded from {checkpoint_path}')
            # Warm up the TorchScript/CUDA path now, while we're still in
            # __init__ — the first compute() otherwise costs ~0.5 s of JIT/CUDA
            # init and would stall the odometry callback at policy handoff.
            try:
                warm = DiffAeroObs(
                    position_enu=np.zeros(3), velocity_enu=np.zeros(3),
                    R_enu=np.eye(3), goal_enu=np.zeros(3),
                    perception_encoded=np.zeros((9, 16), dtype=np.float32))
                for _ in range(3):
                    self.policy.compute(warm)
                self.policy.reset()
                self.get_logger().info('DiffAero velocity policy warmed up')
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'policy warm-up failed: {e}')
        else:
            self.get_logger().warn(
                'checkpoint_path not set — ACTIVE phase will use nominal_velocity fallback')

        # ---- Operator services ------------------------------------------
        self.create_service(Trigger, '~/takeoff', self.handle_takeoff)
        self.create_service(Trigger, '~/start', self.handle_start)
        self.create_service(Trigger, '~/hold', self.handle_hold)
        self.create_service(Trigger, '~/land', self.handle_land)
        self.create_service(Trigger, '~/reset_fence', self.handle_reset_fence)

        # ---- Visualization ----------------------------------------------
        self.publish_viz = bool(self.get_parameter('publish_viz').value)
        self.viz_frame = str(self.get_parameter('viz_frame').value)
        self.vel_arrow_scale = float(self.get_parameter('vel_arrow_scale_s').value)
        self.viz_pub = (self.create_publisher(MarkerArray, '/svg/viz/markers', 10)
                        if self.publish_viz else None)
        # ToF debug image — side-by-side raw | processed when both are available.
        self.tof_image_pub = (
            self.create_publisher(Image, tof_image_tmpl.format(name=name), 10)
            if self.publish_viz else None)
        tof_raw_tmpl_resolved = tof_raw_tmpl.format(name=name) if tof_raw_tmpl else ''
        if self.publish_viz and tof_raw_tmpl_resolved:
            self.create_subscription(
                Float32MultiArray, tof_raw_tmpl_resolved,
                self.tof_raw_callback, 10)
            self._tof_raw_rate_start = None
            self._tof_raw_rate_frames = 0
        if self.publish_viz and self.tof_viz_side_by_side and not tof_raw_tmpl_resolved:
            self.get_logger().warn(
                'tof_viz_side_by_side is true but tof_raw_topic_template is empty — '
                'only processed ToF will be shown')

        rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f'DiffAeroVelocityCommander up | {name} | mode={mode} | scenario={scenario_name}'
            + (f' | FENCE {self.fence_min}..{self.fence_max}'
               if self.fence_enabled else ' | fence OFF'))
        if np.any(self.drone.position_offset):
            self.get_logger().info(
                f'position offset (local->world): {self.drone.position_offset}')
        else:
            self.get_logger().warn(
                'drone_position_offset is zero — correct for mocap, but in SIM '
                'set it to the spawn position or geometry will be wrong!')

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def odometry_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        self.drone.position = np.array([p.x, p.y, p.z]) + self.drone.position_offset
        self.drone.orientation = np.array([q.x, q.y, q.z, q.w])
        # The DiffAero policy expects world-ENU velocity. REP-145 says the twist
        # is in child_frame_id (body FLU) and mavros/local_position/odom obeys
        # that, but px4_interface publishes PX4's VELOCITY_FRAME_NED converted to
        # world ENU while still stamping child_frame_id=base_link — so the label
        # is not trustworthy and the frame comes from odom_twist_frame instead.
        # Rotating an already-world velocity would double-apply yaw.
        v_raw = np.array([v.x, v.y, v.z])
        if self.odom_twist_frame == 'body':
            R_flu_to_enu = Rotation.from_quat(self.drone.orientation).as_matrix()
            self.drone.velocity = R_flu_to_enu @ v_raw
        else:
            self.drone.velocity = v_raw
        self.drone.last_odom_time = self.get_clock().now()

    def tof_callback(self, msg: Float32MultiArray):
        dims = [d.size for d in msg.layout.dim]
        h, w = dims if len(dims) == 2 else (TOF_GRID_H, TOF_GRID_W)
        if h != TOF_GRID_H or w != TOF_GRID_W:
            self.get_logger().warning(
                f'Expected ToF grid {TOF_GRID_H}x{TOF_GRID_W}, got {h}x{w}',
                throttle_duration_sec=5.0)
        grid = np.array(msg.data, dtype=np.float32).reshape(h, w)
        self.drone.tof = grid
        self.drone.last_tof_time = self.get_clock().now()
        self._refresh_tof_debug_image()

    def tof_raw_callback(self, msg: Float32MultiArray):
        dims = [d.size for d in msg.layout.dim]
        if len(dims) != 2:
            return
        h, w = dims
        self.drone.tof_raw = np.array(msg.data, dtype=np.float32).reshape(h, w)
        self._refresh_tof_debug_image()
        self._log_tof_raw_rate()

    def _log_tof_raw_rate(self):
        now = self.get_clock().now()
        if self._tof_raw_rate_start is None:
            self._tof_raw_rate_start = now
            self._tof_raw_rate_frames = 0
            return
        self._tof_raw_rate_frames += 1
        elapsed = (now - self._tof_raw_rate_start).nanoseconds * 1e-9
        if elapsed < 1.0:
            return
        hz = self._tof_raw_rate_frames / elapsed
        self.get_logger().info(f'Raw ToF viz publishing at {hz:.1f} Hz')
        self._tof_raw_rate_start = now
        self._tof_raw_rate_frames = 0

    @staticmethod
    def _orient_raw_tof(arr: np.ndarray) -> np.ndarray:
        """Starling TOF is portrait; rotate 90° CCW when stored wide."""
        h, w = arr.shape[:2]
        if w > h:
            return np.rot90(arr, k=1)
        return arr

    @staticmethod
    def _closeness_to_rgb(closeness: np.ndarray,
                          invalid: np.ndarray | None = None) -> np.ndarray:
        """Hot/cold colormap: red = close, blue = far, black = invalid / zero.

        ``closeness`` is in [0, 1]: 0 = far clip / no return, 1 = at the sensor.
        Pixels at exactly 0.0 (or marked invalid) render black.
        """
        t = np.clip(closeness, 0.0, 1.0)
        rgb = np.zeros((*t.shape, 3), dtype=np.float32)

        if invalid is None:
            invalid = t <= 0.0
        else:
            invalid = invalid | (t <= 0.0)

        valid = ~invalid
        u = t[valid]
        rgb[valid, 0] = u          # red   increases with closeness
        rgb[valid, 2] = 1.0 - u    # blue  decreases with closeness

        return (rgb * 255.0).astype(np.uint8)

    @staticmethod
    def _resize_nearest_2d(arr: np.ndarray, out_h: int, out_w: int) -> np.ndarray:
        yi = np.linspace(0, arr.shape[0] - 1, out_h).astype(np.int64)
        xi = np.linspace(0, arr.shape[1] - 1, out_w).astype(np.int64)
        return arr[yi][:, xi]

    def _planar_to_closeness(self, planar_z: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        z = np.asarray(planar_z, dtype=np.float32)
        if not self.tof_raw_pre_oriented:
            z = self._orient_raw_tof(z)
        max_dist = self.tof_viz_max_dist
        valid = np.isfinite(z) & (z > 1e-3)
        closeness = np.zeros_like(z, dtype=np.float32)
        closeness[valid] = 1.0 - np.clip(z[valid], 0.0, max_dist) / max_dist
        return closeness, ~valid

    def _panel_rgb(self, closeness: np.ndarray, invalid: np.ndarray | None,
                   out_h: int) -> np.ndarray:
        aspect = closeness.shape[1] / max(1, closeness.shape[0])
        out_w = max(1, int(round(out_h * aspect)))
        g = self._resize_nearest_2d(closeness, out_h, out_w)
        inv = None
        if invalid is not None:
            inv = self._resize_nearest_2d(invalid.astype(np.float32), out_h, out_w) > 0.5
        return self._closeness_to_rgb(g, invalid=inv)

    def _refresh_tof_debug_image(self):
        """Publish raw | processed side-by-side (hot/cold colormap) for debugging."""
        if self.tof_image_pub is None:
            return

        display_h = TOF_GRID_H * self.tof_viz_scale
        raw_rgb = proc_rgb = None

        if self.drone.tof_raw is not None:
            raw_closeness, raw_invalid = self._planar_to_closeness(self.drone.tof_raw)
            raw_rgb = self._panel_rgb(raw_closeness, raw_invalid, display_h)

        if self.drone.tof is not None:
            proc_rgb = self._panel_rgb(self.drone.tof, None, display_h)

        if self.tof_viz_side_by_side and raw_rgb is not None and proc_rgb is not None:
            gap = np.zeros((display_h, 4, 3), dtype=np.uint8)
            rgb = np.concatenate([raw_rgb, gap, proc_rgb], axis=1)
        elif proc_rgb is not None:
            rgb = proc_rgb
        elif raw_rgb is not None:
            rgb = raw_rgb
        else:
            return

        h, w = rgb.shape[:2]
        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = self.viz_frame
        img.height = h
        img.width = w
        img.encoding = 'rgb8'
        img.is_bigendian = 0
        img.step = w * 3
        img.data = rgb.tobytes()
        self.tof_image_pub.publish(img)

    def goal_callback(self, msg: PoseStamped):
        if hasattr(self.scenario, 'set_goal'):
            p = msg.pose.position
            self.scenario.set_goal(0, np.array([p.x, p.y, p.z]))

    def speed_callback(self, msg: Float32):
        if hasattr(self.scenario, 'set_speed'):
            self.scenario.set_speed(0, msg.data)

    # ------------------------------------------------------------------
    # Operator services
    # ------------------------------------------------------------------

    def handle_takeoff(self, request, response):
        d = self.drone
        if d.state != FlightState.IDLE:
            response.success = False
            response.message = f'{d.name}: not IDLE (state={d.state.name})'
            return response
        if d.position is None:
            response.success = False
            response.message = f'{d.name}: no odometry yet'
            return response
        d.state = FlightState.ARMING
        d.arming_start = self.get_clock().now()
        d.arming_steps_done = set()
        d.hold_target = d.takeoff_target.copy()
        response.success = True
        response.message = f'takeoff: {d.name}'
        return response

    def handle_start(self, request, response):
        if self.fence_breached:
            response.success = False
            response.message = 'geofence breached — call ~/reset_fence first'
            return response
        if self.drone.state != FlightState.ACTIVE:
            response.success = False
            response.message = f'{self.drone.name} not holding yet (state={self.drone.state.name})'
            return response
        self.mission_active = True
        self._policy_last_time = None
        if self.policy is not None:
            self.policy.reset()
            # Fly to the configured goal: target_vel = (goal - pos) drives the
            # policy toward goal_position, then naturally hovers on arrival.
            self.policy_goal = self.goal_position.copy()
            self.get_logger().info(
                f'policy goal set to {self.policy_goal.round(3)}')
        response.success = True
        response.message = f'scenario "{self.scenario_name}" running'
        self.get_logger().info(response.message)
        return response

    def handle_hold(self, request, response):
        self.mission_active = False
        d = self.drone
        if d.position is not None and d.state in (
                FlightState.ASCEND, FlightState.FACE_GOAL, FlightState.ACTIVE):
            d.hold_target = d.position.copy()
            d.hold_orientation = d.orientation.copy()
            d.state = FlightState.ACTIVE
            response.success = True
            response.message = f'holding: {d.name}'
        else:
            response.success = False
            response.message = 'nothing to hold'
        return response

    def handle_land(self, request, response):
        self.mission_active = False
        d = self.drone
        if d.state in (FlightState.ASCEND, FlightState.FACE_GOAL, FlightState.ACTIVE):
            d.state = FlightState.LANDING
            response.success = True
            response.message = f'landing: {d.name}'
        else:
            response.success = False
            response.message = 'no airborne drone to land'
        return response

    def handle_reset_fence(self, request, response):
        d = self.drone
        still_out = (d.position is not None
                     and (np.any(d.position < self.fence_min)
                          or np.any(d.position > self.fence_max)))
        self.fence_breached = False
        response.success = True
        response.message = 'geofence latch cleared' + (
            f' (WARNING {d.name} still outside)' if still_out else '')
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Geofence
    # ------------------------------------------------------------------

    def enforce_fence(self):
        if not self.fence_enabled or self.fence_breached:
            return
        d = self.drone
        if d.position is None or d.state not in (FlightState.FACE_GOAL, FlightState.ACTIVE):
            return
        below = d.position < self.fence_min
        above = d.position > self.fence_max
        if not (below.any() or above.any()):
            return
        self.fence_breached = True
        self.mission_active = False
        axes = 'xyz'
        viol = ', '.join(
            f'{axes[k]}{"<min" if below[k] else ">max"}'
            for k in range(3) if below[k] or above[k])
        d.hold_target = d.position.copy()
        d.hold_orientation = d.orientation.copy()
        d.state = FlightState.ACTIVE
        self.get_logger().error(
            f'GEOFENCE BREACH by {d.name} at '
            f'[{d.position[0]:.2f}, {d.position[1]:.2f}, {d.position[2]:.2f}] '
            f'({viol}) — HOLDING. Call ~/reset_fence to clear.')

    # ------------------------------------------------------------------
    # Robot interface helpers
    # ------------------------------------------------------------------

    def send_robot_command(self, command: int, label: str):
        client = self.drone.robot_command_client
        if not client.service_is_ready():
            self.get_logger().warn(
                f'{self.drone.name}: robot_command service not ready, skipping {label}')
            return
        req = RobotCommand.Request()
        req.command = command
        future = client.call_async(req)

        def report(fut, name=self.drone.name, label=label):
            try:
                ok = fut.result().success
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'{name}: {label} failed: {e}')
                return
            if ok:
                self.get_logger().info(f'{name}: {label} -> success=True')
            else:
                self.get_logger().error(f'{name}: {label} -> success=False')

        future.add_done_callback(report)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def control_loop(self):
        now = self.get_clock().now()
        d = self.drone

        # Advance ARMING state machine.
        if d.state == FlightState.ARMING:
            elapsed = (now - d.arming_start).nanoseconds * 1e-9
            if elapsed >= ARMING_OFFBOARD_S and 'offboard' not in d.arming_steps_done:
                d.arming_steps_done.add('offboard')
                self.send_robot_command(RobotCommand.Request.REQUEST_CONTROL,
                                        'request offboard')
            if elapsed >= ARMING_ARM_S and 'arm' not in d.arming_steps_done:
                d.arming_steps_done.add('arm')
                self.send_robot_command(RobotCommand.Request.ARM, 'arm')
            if elapsed >= ARMING_DONE_S:
                d.state = FlightState.ASCEND
                self.get_logger().info(f'{d.name}: ascending to {d.hold_target}')

        self.enforce_fence()

        if d.position is None:
            return

        fresh = (d.last_odom_time is not None
                 and (now - d.last_odom_time) < Duration(seconds=self.state_timeout))

        # Stale odometry fallback — hold position.
        if not fresh and d.state not in (FlightState.IDLE, FlightState.ARMING):
            self.get_logger().warn(
                f'{d.name}: odometry stale, holding position',
                throttle_duration_sec=1.0)
            self.publish_pose(d.hold_target, d.hold_orientation, now)
            self.publish_markers(now)
            return

        if d.state == FlightState.IDLE:
            pass

        elif d.state == FlightState.ARMING:
            # Stream current position to satisfy offboard mode entry requirement.
            self.publish_pose(d.position, d.orientation, now)

        elif d.state == FlightState.ASCEND:
            error = d.hold_target - d.position
            xy_err = error[:2]
            z_err = float(error[2])
            at_hover_xy = np.linalg.norm(xy_err) < self.arrival_threshold
            at_hover_z = abs(z_err) < self.arrival_threshold
            if at_hover_xy and at_hover_z:
                d.state = FlightState.FACE_GOAL
                d.face_cmd_yaw = self._yaw_from_quat(d.orientation)
                self.get_logger().info(f'{d.name}: holding takeoff position')
                vel = np.zeros(3)
            elif not at_hover_z:
                # Climb straight up first — no horizontal velocity on the ground.
                vel = np.zeros(3)
                vel[2] = self.hover_kp * z_err
                if abs(vel[2]) > self.ascend_speed:
                    vel[2] = math.copysign(self.ascend_speed, vel[2])
            else:
                # At altitude: translate to hover XY while holding Z.
                vel = np.zeros(3)
                vel[:2] = self.hover_kp * xy_err
                speed = np.linalg.norm(vel[:2])
                if speed > self.ascend_speed:
                    vel[:2] *= self.ascend_speed / speed
                vel[2] = np.clip(self.hover_kp * z_err, -self.ascend_speed,
                                 self.ascend_speed)
            self.publish_velocity(vel, now)

        elif d.state == FlightState.FACE_GOAL:
            target_yaw = math.atan2(
                self.goal_position[1] - d.position[1],
                self.goal_position[0] - d.position[0])
            # Slew the commanded yaw toward the target at a bounded rate so the
            # drone rotates smoothly instead of snapping to the final heading.
            if d.face_cmd_yaw is None:
                d.face_cmd_yaw = self._yaw_from_quat(d.orientation)
            max_step = self.face_goal_yaw_rate_max * self.control_dt
            cmd_err = math.remainder(target_yaw - d.face_cmd_yaw, 2 * math.pi)
            d.face_cmd_yaw += float(np.clip(cmd_err, -max_step, max_step))
            cmd_yaw = math.remainder(d.face_cmd_yaw, 2 * math.pi)
            goal_q = np.array([0.0, 0.0,
                                math.sin(cmd_yaw / 2),
                                math.cos(cmd_yaw / 2)])
            self.publish_pose(d.hold_target, goal_q, now)
            current_yaw = self._yaw_from_quat(d.orientation)
            yaw_err = abs(math.remainder(target_yaw - current_yaw, 2 * math.pi))
            if yaw_err < self.face_goal_threshold:
                d.hold_target = d.position.copy()
                d.hold_orientation = goal_q.copy()
                d.state = FlightState.ACTIVE
                self.get_logger().info(
                    f'{d.name}: facing goal (yaw={math.degrees(target_yaw):.1f}°) → ACTIVE')

        elif d.state == FlightState.ACTIVE:
            # Planar policies command no vertical velocity (vz=0), so they reach
            # the goal's XY but never its Z — measure arrival in the horizontal
            # plane only, otherwise the drone gets stuck hovering under/above the
            # goal (running the policy with a noisy near-zero target velocity).
            planar_policy = self.policy is not None and getattr(
                self.policy, 'planar', False)
            reached_goal = False
            if self.mission_active and self.policy is not None:
                offset = self.policy_goal - d.position
                goal_dist = (np.linalg.norm(offset[:2]) if planar_policy
                             else np.linalg.norm(offset))
                reached_goal = goal_dist < self.goal_arrival_threshold
            if reached_goal:
                # The DiffAero policy is a cruise controller, not a position-hold
                # controller — at the goal target_vel→0 but it overshoots and
                # oscillates. Hand off to a stable pose-hold at the goal.
                self.mission_active = False
                if planar_policy:
                    # Hold the goal's XY but keep the current altitude — a planar
                    # policy never controlled Z, so don't jump to the goal's Z.
                    d.hold_target = np.array(
                        [self.policy_goal[0], self.policy_goal[1], d.position[2]])
                else:
                    d.hold_target = self.policy_goal.copy()
                d.hold_orientation = d.orientation.copy()
                self.get_logger().info(
                    f'{d.name}: reached goal {self.policy_goal.round(2)} → HOLD')
                self.publish_pose(d.hold_target, d.hold_orientation, now)
            elif self.mission_active and self.policy is not None:
                # If the policy was interrupted (stale odom, fence, hold) for
                # more than a couple ticks, reset vel_ema so the heading-direction
                # initialization kicks in rather than accumulating stale drift.
                if self._policy_last_time is not None:
                    gap_s = (now - self._policy_last_time).nanoseconds * 1e-9
                    if gap_s > 0.2:
                        self.get_logger().info(
                            f'[policy] interrupted {gap_s:.2f}s — resetting vel_ema')
                        self.policy.reset()
                self._policy_last_time = now
                tof_fresh = (
                    d.last_tof_time is not None
                    and (now - d.last_tof_time) < Duration(seconds=self.tof_timeout)
                )
                obs = DiffAeroObs(
                    position_enu=d.position,
                    velocity_enu=d.velocity,
                    R_enu=Rotation.from_quat(d.orientation).as_matrix(),
                    goal_enu=self.policy_goal,
                    perception_encoded=d.tof if tof_fresh else None,
                )
                cmd = self.policy.compute(obs)
                yaw_rate = self._yaw_rate_to(cmd.desired_yaw_enu, d.orientation)
                self.publish_velocity(cmd.vel_cmd_enu, now, yaw_rate)
                self.get_logger().info(
                    f'[policy] pos={np.round(d.position,2)} vel={np.round(d.velocity,2)} '
                    f'quat={np.round(d.orientation,3)} | '
                    f'goal={np.round(self.policy_goal,2)} tof_fresh={tof_fresh}',
                    throttle_duration_sec=2.0)
                self.get_logger().info(
                    f'[policy] vel_cmd_enu={np.round(cmd.vel_cmd_enu,2)} '
                    f'vel_norm={cmd.vel_norm:.2f} '
                    f'yaw_des={math.degrees(cmd.desired_yaw_enu):.1f}° '
                    f'yaw_rate={yaw_rate:.2f}',
                    throttle_duration_sec=1.0)
            elif self.mission_active:
                self.get_logger().error('No policy loaded!')
                # No policy loaded — nominal velocity fallback.
                nominal = self.scenario.nominal_velocity(d.position.reshape(1, 3))[0]
                self.publish_velocity(nominal, now)
            else:
                # Hold with position command — pose_command is the safe fallback.
                self.publish_pose(d.hold_target, d.hold_orientation, now)

        elif d.state == FlightState.LANDING:
            if d.position[2] <= self.land_complete_alt:
                self.send_robot_command(RobotCommand.Request.DISARM, 'disarm')
                d.state = FlightState.IDLE
                self.get_logger().info(f'{d.name}: landed, disarmed')
            else:
                self.publish_velocity(np.array([0.0, 0.0, -self.land_speed]), now)

        self.publish_markers(now)

    # ------------------------------------------------------------------
    # Yaw helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _yaw_from_quat(q: np.ndarray) -> float:
        """ENU yaw (CCW from +x/East) from an xyzw quaternion."""
        return math.atan2(
            2.0 * (q[3] * q[2] + q[0] * q[1]),
            1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2))

    def _yaw_rate_to(self, desired_yaw_enu: float, q: np.ndarray) -> float:
        """P-controller from current heading toward the policy's desired heading.
        Returns a clamped yaw-rate (rad/s, ENU CCW+)."""
        current_yaw = self._yaw_from_quat(q)
        err = math.remainder(desired_yaw_enu - current_yaw, 2 * math.pi)
        return float(np.clip(self.yaw_kp * err, -self.yaw_rate_max, self.yaw_rate_max))

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def publish_velocity(self, velocity_enu: np.ndarray, now, yaw_rate: float = 0.0):
        """Publish a velocity setpoint.

        The policy emits world-ENU velocity. The sim MAVROS interface interprets
        ``velocity_command`` as a body (yaw-aligned) frame (FRAME_BODY_NED → FLU
        on the ROS side), so in sim we rotate world-ENU → body before publishing.
        The real px4 interface consumes world ENU directly, so it is published
        unchanged. ``yaw_rate`` is frame-agnostic (ENU CCW+) for both interfaces.
        """
        # Stash the world-ENU command for visualization (publish_markers draws
        # the arrow in viz_frame, which is world-ENU like everything else here).
        self.drone.vel_cmd_enu = np.array(velocity_enu, dtype=float)

        if self.drone_mode == 'sim':
            v_out = self._world_to_body(velocity_enu, self.drone.orientation)
        else:
            v_out = velocity_enu

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link' if self.drone_mode == 'sim' else 'map'
        msg.twist.linear.x = float(v_out[0])
        msg.twist.linear.y = float(v_out[1])
        msg.twist.linear.z = float(v_out[2])
        msg.twist.angular.z = float(yaw_rate)
        self.drone.cmd_pub.publish(msg)

    @staticmethod
    def _world_to_body(velocity_enu: np.ndarray, q: np.ndarray) -> np.ndarray:
        """Rotate a world-ENU velocity into the yaw-aligned body FLU frame
        (x-forward, y-left, z-up), matching MAVROS FRAME_BODY_NED on the ROS
        side. Yaw-only so vertical speed maps straight to body z."""
        yaw = DiffAeroVelocityCommander._yaw_from_quat(q)
        c, s = math.cos(yaw), math.sin(yaw)
        # body = Rz(yaw)^T @ world
        vx = c * velocity_enu[0] + s * velocity_enu[1]
        vy = -s * velocity_enu[0] + c * velocity_enu[1]
        return np.array([vx, vy, velocity_enu[2]])

    def publish_pose(self, position: np.ndarray, orientation: np.ndarray, now):
        # Pose-hold means no active velocity command — drop the viz arrow.
        self.drone.vel_cmd_enu = None
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = float(orientation[0])
        msg.pose.orientation.y = float(orientation[1])
        msg.pose.orientation.z = float(orientation[2])
        msg.pose.orientation.w = float(orientation[3])
        self.drone.pos_cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------

    def publish_markers(self, now):
        if self.viz_pub is None:
            return
        d = self.drone
        if d.position is None:
            return
        arr = MarkerArray()
        stamp = now.to_msg()

        r, g, b = (0.9, 0.2, 0.2) if d.mode == 'real' else (0.2, 0.7, 1.0)
        if self.fence_breached:
            r, g, b = (1.0, 0.3, 0.0)

        body = Marker()
        body.header.frame_id = self.viz_frame
        body.header.stamp = stamp
        body.ns = 'body'
        body.id = 0
        body.type = Marker.SPHERE
        body.action = Marker.ADD
        body.pose.position.x = float(d.position[0])
        body.pose.position.y = float(d.position[1])
        body.pose.position.z = float(d.position[2])
        body.pose.orientation.w = 1.0
        body.scale.x = body.scale.y = body.scale.z = 0.3
        body.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        arr.markers.append(body)

        label = Marker()
        label.header.frame_id = self.viz_frame
        label.header.stamp = stamp
        label.ns = 'label'
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(d.position[0])
        label.pose.position.y = float(d.position[1])
        label.pose.position.z = float(d.position[2]) + 0.4
        label.pose.orientation.w = 1.0
        label.scale.z = 0.25
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        label.text = f'{d.name} [{d.mode}|{d.state.name}]'
        arr.markers.append(label)

        # Heading arrow — forward body axis (FLU: X=forward) rotated by quaternion.
        qx, qy, qz, qw = d.orientation
        fx = 1 - 2*(qy*qy + qz*qz)
        fy = 2*(qx*qy + qw*qz)
        fz = 2*(qx*qz - qw*qy)
        arrow = Marker()
        arrow.header.frame_id = self.viz_frame
        arrow.header.stamp = stamp
        arrow.ns = 'heading'
        arrow.id = 3
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points.append(Point(x=float(d.position[0]),
                                   y=float(d.position[1]),
                                   z=float(d.position[2])))
        arrow.points.append(Point(x=float(d.position[0] + 0.5*fx),
                                   y=float(d.position[1] + 0.5*fy),
                                   z=float(d.position[2] + 0.5*fz)))
        arrow.scale.x = 0.05   # shaft diameter
        arrow.scale.y = 0.1    # head diameter
        arrow.scale.z = 0.1    # head length
        arrow.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        arr.markers.append(arrow)

        # Velocity-command arrow (world-ENU). Drawn from the drone toward where
        # the command would carry it in vel_arrow_scale seconds. Cleared (DELETE)
        # whenever we're pose-commanding instead of velocity-commanding.
        vel_arrow = Marker()
        vel_arrow.header.frame_id = self.viz_frame
        vel_arrow.header.stamp = stamp
        vel_arrow.ns = 'vel_cmd'
        vel_arrow.id = 5
        vel_arrow.type = Marker.ARROW
        if d.vel_cmd_enu is None:
            vel_arrow.action = Marker.DELETE
        else:
            vel_arrow.action = Marker.ADD
            tip = d.position + self.vel_arrow_scale * d.vel_cmd_enu
            vel_arrow.points.append(Point(x=float(d.position[0]),
                                          y=float(d.position[1]),
                                          z=float(d.position[2])))
            vel_arrow.points.append(Point(x=float(tip[0]),
                                          y=float(tip[1]),
                                          z=float(tip[2])))
            vel_arrow.scale.x = 0.05   # shaft diameter
            vel_arrow.scale.y = 0.1    # head diameter
            vel_arrow.scale.z = 0.15   # head length
            vel_arrow.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=1.0)
        arr.markers.append(vel_arrow)

        goal_marker = Marker()
        goal_marker.header.frame_id = self.viz_frame
        goal_marker.header.stamp = stamp
        goal_marker.ns = 'goal'
        goal_marker.id = 2
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = float(self.goal_position[0])
        goal_marker.pose.position.y = float(self.goal_position[1])
        goal_marker.pose.position.z = float(self.goal_position[2])
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.25
        goal_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.4, a=0.8)
        arr.markers.append(goal_marker)

        goal_label = Marker()
        goal_label.header.frame_id = self.viz_frame
        goal_label.header.stamp = stamp
        goal_label.ns = 'goal_label'
        goal_label.id = 4
        goal_label.type = Marker.TEXT_VIEW_FACING
        goal_label.action = Marker.ADD
        goal_label.pose.position.x = float(self.goal_position[0])
        goal_label.pose.position.y = float(self.goal_position[1])
        goal_label.pose.position.z = float(self.goal_position[2]) + 0.35
        goal_label.pose.orientation.w = 1.0
        goal_label.scale.z = 0.2
        goal_label.color = ColorRGBA(r=0.0, g=1.0, b=0.4, a=1.0)
        goal_label.text = 'goal'
        arr.markers.append(goal_label)

        if self.fence_enabled:
            arr.markers.append(self._fence_marker(stamp))

        self.viz_pub.publish(arr)

    def _fence_marker(self, stamp):
        lo, hi = self.fence_min, self.fence_max
        corners = [
            (lo[0], lo[1], lo[2]), (hi[0], lo[1], lo[2]),
            (hi[0], hi[1], lo[2]), (lo[0], hi[1], lo[2]),
            (lo[0], lo[1], hi[2]), (hi[0], lo[1], hi[2]),
            (hi[0], hi[1], hi[2]), (lo[0], hi[1], hi[2]),
        ]
        edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7),
                 (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
        m = Marker()
        m.header.frame_id = self.viz_frame
        m.header.stamp = stamp
        m.ns = 'fence'
        m.id = 9000
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03
        m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9) if self.fence_breached \
            else ColorRGBA(r=0.2, g=1.0, b=0.3, a=0.5)
        for a, c in edges:
            for idx in (a, c):
                m.points.append(Point(x=float(corners[idx][0]),
                                      y=float(corners[idx][1]),
                                      z=float(corners[idx][2])))
        return m


def main(args=None):
    rclpy.init(args=args)
    node = DiffAeroVelocityCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
