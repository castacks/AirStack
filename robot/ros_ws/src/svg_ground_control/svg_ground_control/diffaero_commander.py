"""Single-drone DiffAero ground commander.

Commands one drone through the AirStack robot_interface abstraction (works
unchanged over MAVROS in sim and px4_interface/uXRCE-DDS on hardware — only
the topic templates in the config YAML differ):

    state in:    {state_topic_template}               nav_msgs/Odometry (ENU)
    vel cmd out: {velocity_command_topic_template}    geometry_msgs/TwistStamped
    pos cmd out: {position_command_topic_template}    geometry_msgs/PoseStamped
    services:    {robot_command_service_template}     airstack_msgs/srv/RobotCommand

drone_mode selects the interface:
    'sim'  -> /{name}/interface/* (MAVROS/SITL)
    'real' -> /{name}/fmu/*       (px4_interface/uXRCE-DDS)

Geofence: with fence_enabled, if the drone leaves [fence_min, fence_max] the
commander latches a breach — drone freezes and start is blocked until
~/reset_fence.

Lifecycle (std_srvs/Trigger services):
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
from std_msgs.msg import ColorRGBA, Float32, Float32MultiArray
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from airstack_msgs.srv import RobotCommand
from mav_msgs.msg import AttitudeThrust

from svg_ground_control.scenarios import Bounds, make_scenario
from svg_ground_control.diffaero.diffaero_core import DiffAeroObs, DiffAeroPolicy
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


class DroneHandle:
    """Book-keeping for one drone."""

    def __init__(self, name: str, mode: str):
        self.name = name
        self.mode = mode                  # 'sim' | 'real'
        self.position_offset = np.zeros(3)
        self.takeoff_target = None        # np (3,)
        self.hold_target = None           # np (3,)
        self.hold_orientation = np.array([0., 0., 0., 1.])  # set when FACE_GOAL completes
        self.state = FlightState.IDLE
        self.position = None              # np (3,) ENU, None until first odometry
        self.velocity = np.zeros(3)
        self.orientation = np.array([0., 0., 0., 1.])  # xyzw quaternion
        self.last_odom_time = None
        self.arming_start = None
        self.arming_steps_done = set()
        self.cmd_pub = None               # velocity publisher (landing only)
        self.pos_cmd_pub = None           # position publisher (ascend / hold / fallback)
        self.att_cmd_pub = None           # attitude+thrust publisher (policy)
        self.robot_command_client = None
        self.tof: np.ndarray | None = None        # latest 9×16 pre-encoded perception
        self.last_tof_time = None


class DiffAeroCommander(Node):

    def __init__(self, **kwargs):
        super().__init__('diffaero_commander', **kwargs)

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('drone_name', 'drone_1')
        self.declare_parameter('drone_mode', 'sim')  # 'sim' | 'real'

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
        self.declare_parameter('sim_attitude_thrust_topic_template',
                               '/{name}/interface/attitude_thrust_command')
        self.declare_parameter('real_attitude_thrust_topic_template',
                               '/{name}/fmu/attitude_thrust_command')

        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('max_accel', 30.0)
        self.declare_parameter('max_vel', 5.0)
        self.declare_parameter('max_acc_xy', 20.0)
        self.declare_parameter('max_acc_z', 40.0)
        self.declare_parameter('tof_timeout_s', 0.5)

        self.declare_parameter('fence_enabled', False)
        self.declare_parameter('fence_min', [-1000.0, -1000.0, -1000.0])
        self.declare_parameter('fence_max', [1000.0, 1000.0, 1000.0])

        self.declare_parameter('publish_viz', True)
        self.declare_parameter('viz_frame', 'map')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('state_timeout_s', 0.5)
        self.declare_parameter('hover_kp', 1.0)
        self.declare_parameter('arrival_threshold_m', 0.15)
        self.declare_parameter('goal_arrival_threshold_m', 0.4)
        self.declare_parameter('land_speed_mps', 0.3)
        self.declare_parameter('land_complete_altitude_m', 0.15)
        self.declare_parameter('face_goal_threshold_rad', 0.05)

        # ---- Read parameters --------------------------------------------
        name = str(self.get_parameter('drone_name').value)
        mode = str(self.get_parameter('drone_mode').value)
        if mode not in ('sim', 'real'):
            raise ValueError(f'drone_mode must be sim|real, got "{mode}"')

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
        self.arrival_threshold = float(self.get_parameter('arrival_threshold_m').value)
        self.goal_arrival_threshold = float(
            self.get_parameter('goal_arrival_threshold_m').value)
        self.land_speed = float(self.get_parameter('land_speed_mps').value)
        self.land_complete_alt = float(
            self.get_parameter('land_complete_altitude_m').value)
        self.face_goal_threshold = float(
            self.get_parameter('face_goal_threshold_rad').value)

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
        if mode == 'real':
            att_tmpl = str(self.get_parameter('real_attitude_thrust_topic_template').value)
        else:
            att_tmpl = str(self.get_parameter('sim_attitude_thrust_topic_template').value)

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
        self.drone.att_cmd_pub = self.create_publisher(
            AttitudeThrust, att_tmpl.format(name=name), 10)

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

        # ---- DiffAero policy --------------------------------------------
        checkpoint_path = str(self.get_parameter('checkpoint_path').value)
        self.policy: DiffAeroPolicy | None = None
        self.policy_goal: np.ndarray | None = None   # fixed at ~/start; goal = position → hover
        self._policy_last_time = None                 # for gap-detection auto-reset
        if checkpoint_path:
            # Dummy intrinsics — PerceptionBuilder is bypassed because the ToF
            # topic sends a pre-encoded 9×16 grid (perception_encoded path).
            dummy_intrinsics = Intrinsics(fx=1.0, fy=1.0, cx=0.5, cy=0.5, H=1, W=1)
            self.policy = DiffAeroPolicy(
                intrinsics=dummy_intrinsics,
                checkpoint_path=checkpoint_path,
                max_accel=float(self.get_parameter('max_accel').value),
                max_vel=float(self.get_parameter('max_vel').value),
                max_acc_xy=float(self.get_parameter('max_acc_xy').value),
                max_acc_z=float(self.get_parameter('max_acc_z').value),
            )
            self.get_logger().info(f'DiffAero policy loaded from {checkpoint_path}')
            # Warm up the TorchScript/CUDA path now, while we're still in
            # __init__. The first compute() otherwise costs ~0.5 s of JIT/CUDA
            # init, and since we run on a single-threaded executor that stalls
            # the odometry callback at the exact moment of policy handoff —
            # tripping the stale-odom hold and a spurious vel_ema reset.
            try:
                warm = DiffAeroObs(
                    position_enu=np.zeros(3), velocity_enu=np.zeros(3),
                    R_enu=np.eye(3), goal_enu=np.zeros(3))
                for _ in range(3):
                    self.policy.compute(warm)
                self.policy.reset()
                self.get_logger().info('DiffAero policy warmed up')
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
        self.viz_pub = (self.create_publisher(MarkerArray, '/svg/viz/markers', 10)
                        if self.publish_viz else None)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f'DiffAeroCommander up | {name} | mode={mode} | scenario={scenario_name}'
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
        # nav_msgs/Odometry reports twist in child_frame_id (base_link, body FLU)
        # per REP-145 — and mavros/local_position/odom does exactly that. The
        # DiffAero policy expects world-ENU velocity, so rotate body -> world
        # using the orientation (pose.orientation is FLU -> ENU).
        R_flu_to_enu = Rotation.from_quat(self.drone.orientation).as_matrix()
        self.drone.velocity = R_flu_to_enu @ np.array([v.x, v.y, v.z])
        self.drone.last_odom_time = self.get_clock().now()

    def tof_callback(self, msg: Float32MultiArray):
        dims = [d.size for d in msg.layout.dim]
        h, w = dims if len(dims) == 2 else (9, 16)
        self.drone.tof = np.array(msg.data, dtype=np.float32).reshape(h, w)
        self.drone.last_tof_time = self.get_clock().now()

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
            if np.linalg.norm(d.hold_target - d.position) < self.arrival_threshold:
                d.state = FlightState.FACE_GOAL
                self.get_logger().info(f'{d.name}: holding takeoff position')
            self.publish_pose(d.hold_target, d.orientation, now)
        
        elif d.state == FlightState.FACE_GOAL:
            target_yaw = math.atan2(
                self.goal_position[1] - d.position[1],
                self.goal_position[0] - d.position[0])
            goal_q = np.array([0.0, 0.0,
                                math.sin(target_yaw / 2),
                                math.cos(target_yaw / 2)])
            self.publish_pose(d.hold_target, goal_q, now)
            current_yaw = math.atan2(
                2.0 * (d.orientation[3] * d.orientation[2]
                       + d.orientation[0] * d.orientation[1]),
                1.0 - 2.0 * (d.orientation[1] ** 2 + d.orientation[2] ** 2))
            yaw_err = abs(math.remainder(target_yaw - current_yaw, 2 * math.pi))
            if yaw_err < self.face_goal_threshold:
                d.hold_target = d.position.copy()
                d.hold_orientation = goal_q.copy()
                d.state = FlightState.ACTIVE
                self.get_logger().info(
                    f'{d.name}: facing goal (yaw={math.degrees(target_yaw):.1f}°) → ACTIVE')

        elif d.state == FlightState.ACTIVE:
            if (self.mission_active and self.policy is not None
                    and np.linalg.norm(self.policy_goal - d.position)
                    < self.goal_arrival_threshold):
                # The DiffAero policy is a cruise controller, not a position-hold
                # controller — at the goal target_vel→0 but it overshoots and
                # oscillates. Hand off to a stable pose-hold at the goal (the
                # starling deployment did the same at 0.5 m before landing).
                self.mission_active = False
                d.hold_target = self.policy_goal.copy()
                d.hold_orientation = d.orientation.copy()
                self.get_logger().info(
                    f'{d.name}: reached goal {self.policy_goal.round(2)} → HOLD')
                self.publish_pose(d.hold_target, d.hold_orientation, now)
            elif self.mission_active and self.policy is not None:
                # If the policy was interrupted (stale odom, fence, hold) for
                # more than 2 ticks, reset vel_ema so the heading-direction
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
                self.publish_attitude_thrust(cmd.attitude_enu_flu_xyzw, cmd.thrust_norm, now)
                from scipy.spatial.transform import Rotation as _R
                _euler = _R.from_quat(cmd.attitude_enu_flu_xyzw).as_euler('xyz', degrees=True)
                self.get_logger().info(
                    f'[policy] pos={np.round(d.position,2)} vel={np.round(d.velocity,2)} '
                    f'quat={np.round(d.orientation,3)} | '
                    f'goal={np.round(self.policy_goal,2)} tof_fresh={tof_fresh}',
                    throttle_duration_sec=2.0)
                self.get_logger().info(
                    f'[policy] acc_cmd_enu={np.round(cmd.acc_cmd_enu,2)} '
                    f'acc_norm={cmd.acc_norm:.2f} thrust={cmd.thrust_norm:.3f} '
                    f'att_euler_rpy={np.round(_euler,1)}',
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

    def publish_velocity(self, velocity: np.ndarray, now):
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = float(velocity[0])
        msg.twist.linear.y = float(velocity[1])
        msg.twist.linear.z = float(velocity[2])
        self.drone.cmd_pub.publish(msg)

    def publish_pose(self, position: np.ndarray, orientation: np.ndarray, now):
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

    def publish_attitude_thrust(self, xyzw: np.ndarray, thrust_norm: float, now):
        """Publish ENU/FLU attitude + normalized thrust. Interface converts to NED/FRD."""
        msg = AttitudeThrust()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.attitude.x = float(xyzw[0])
        msg.attitude.y = float(xyzw[1])
        msg.attitude.z = float(xyzw[2])
        msg.attitude.w = float(xyzw[3])
        msg.thrust.z = float(thrust_norm)
        self.drone.att_cmd_pub.publish(msg)

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
    node = DiffAeroCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
