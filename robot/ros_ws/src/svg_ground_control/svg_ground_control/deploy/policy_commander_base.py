"""Generic single-drone policy commander.

Owns the flight lifecycle, geofence, and fixed-rate control loop and commands
the drone through the AirStack robot_interface abstraction (MAVROS in sim,
px4_interface on hardware). Subclasses supply a Policy and an ObservationBuilder
through the two make_* hooks; those are the only task-specific pieces.
"""

from __future__ import annotations

from enum import Enum
import math

import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from airstack_msgs.srv import RobotCommand
from mav_msgs.msg import AttitudeThrust

from svg_ground_control.deploy.interfaces import (
    AttitudeSetpoint,
    ObservationBuilder,
    Policy,
    RobotState,
    VelocitySetpoint,
    WaypointSetpoint,
)


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


class PolicyCommanderBase(Node):
    """Flies whatever Policy and ObservationBuilder the subclass provides."""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.declare_extra_parameters()

        self.declare_parameter('drone_name', 'drone_1')
        self.declare_parameter('drone_mode', 'sim')
        self.declare_parameter('trial_log_dir', '')
        self.declare_parameter('policy_name', '')
        self.declare_parameter('drone_position_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('goal_position', [2.0, 0.0, 1.2])
        self.declare_parameter('hover_position', [0.0, 0.0, 1.2])

        self.declare_parameter('state_topic_template',
                               '/{name}/odometry_conversion/odometry')
        self.declare_parameter('sim_velocity_command_topic_template',
                               '/{name}/interface/velocity_command')
        self.declare_parameter('sim_position_command_topic_template',
                               '/{name}/interface/pose_command')
        self.declare_parameter('sim_attitude_thrust_topic_template',
                               '/{name}/interface/attitude_thrust_command')
        self.declare_parameter('sim_robot_command_service_template',
                               '/{name}/interface/robot_command')
        self.declare_parameter('real_velocity_command_topic_template',
                               '/{name}/fmu/velocity_command')
        self.declare_parameter('real_position_command_topic_template',
                               '/{name}/fmu/pose_command')
        self.declare_parameter('real_attitude_thrust_topic_template',
                               '/{name}/fmu/attitude_thrust_command')
        self.declare_parameter('real_robot_command_service_template',
                               '/{name}/fmu/robot_command')

        self.declare_parameter('fence_enabled', False)
        self.declare_parameter('fence_min', [-1000.0, -1000.0, -1000.0])
        self.declare_parameter('fence_max', [1000.0, 1000.0, 1000.0])

        self.declare_parameter('publish_viz', True)
        self.declare_parameter('viz_frame', 'map')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('state_timeout_s', 0.5)
        self.declare_parameter('arrival_threshold_m', 0.15)
        self.declare_parameter('goal_arrival_threshold_m', 0.4)
        self.declare_parameter('land_speed_mps', 0.3)
        self.declare_parameter('land_complete_altitude_m', 0.15)
        self.declare_parameter('face_goal_threshold_rad', 0.05)

        self.name = str(self.get_parameter('drone_name').value)
        self.mode = str(self.get_parameter('drone_mode').value)
        if self.mode not in ('sim', 'real'):
            raise ValueError(f'drone_mode must be sim|real, got "{self.mode}"')

        self.position_offset = np.array(
            self.get_parameter('drone_position_offset').value, dtype=float)
        self.goal_position = np.array(
            self.get_parameter('goal_position').value, dtype=float)
        self.hover_position = np.array(
            self.get_parameter('hover_position').value, dtype=float)

        self.fence_enabled = bool(self.get_parameter('fence_enabled').value)
        self.fence_min = np.array(self.get_parameter('fence_min').value, dtype=float)
        self.fence_max = np.array(self.get_parameter('fence_max').value, dtype=float)
        self.fence_breached = False

        self.state_timeout = float(self.get_parameter('state_timeout_s').value)
        self.arrival_threshold = float(self.get_parameter('arrival_threshold_m').value)
        self.goal_arrival_threshold = float(
            self.get_parameter('goal_arrival_threshold_m').value)
        self.land_speed = float(self.get_parameter('land_speed_mps').value)
        self.land_complete_alt = float(
            self.get_parameter('land_complete_altitude_m').value)
        self.face_goal_threshold = float(
            self.get_parameter('face_goal_threshold_rad').value)

        # Drone runtime state.
        self.state = FlightState.IDLE
        self.position = None
        self.velocity = np.zeros(3)
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.last_odom_time = None
        self.hold_target = self.hover_position.copy()
        self.hold_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.arming_start = None
        self.arming_steps_done = set()
        self.mission_active = False
        self._policy_last_time = None
        self.trial_log = None

        if self.mode == 'real':
            vel_tmpl = str(self.get_parameter('real_velocity_command_topic_template').value)
            pos_tmpl = str(self.get_parameter('real_position_command_topic_template').value)
            att_tmpl = str(self.get_parameter('real_attitude_thrust_topic_template').value)
            srv_tmpl = str(self.get_parameter('real_robot_command_service_template').value)
        else:
            vel_tmpl = str(self.get_parameter('sim_velocity_command_topic_template').value)
            pos_tmpl = str(self.get_parameter('sim_position_command_topic_template').value)
            att_tmpl = str(self.get_parameter('sim_attitude_thrust_topic_template').value)
            srv_tmpl = str(self.get_parameter('sim_robot_command_service_template').value)
        state_tmpl = str(self.get_parameter('state_topic_template').value)

        self.vel_pub = self.create_publisher(
            TwistStamped, vel_tmpl.format(name=self.name), 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, pos_tmpl.format(name=self.name), 10)
        self.att_pub = self.create_publisher(
            AttitudeThrust, att_tmpl.format(name=self.name), 10)
        self.robot_command_client = self.create_client(
            RobotCommand, srv_tmpl.format(name=self.name))
        self.create_subscription(
            Odometry, state_tmpl.format(name=self.name), self.odometry_callback, 10)

        self.observation_builder: ObservationBuilder = self.make_observation_builder()
        self.policy: Policy = self.make_policy()

        self.create_service(Trigger, '~/takeoff', self.handle_takeoff)
        self.create_service(Trigger, '~/start', self.handle_start)
        self.create_service(Trigger, '~/hold', self.handle_hold)
        self.create_service(Trigger, '~/land', self.handle_land)
        self.create_service(Trigger, '~/reset_fence', self.handle_reset_fence)

        self.publish_viz = bool(self.get_parameter('publish_viz').value)
        self.viz_frame = str(self.get_parameter('viz_frame').value)
        self.viz_pub = (self.create_publisher(MarkerArray, '/svg/viz/markers', 10)
                        if self.publish_viz else None)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.on_setup_complete()

        self.get_logger().info(
            f'{node_name} up | {self.name} | mode={self.mode} | '
            + (f'FENCE {self.fence_min}..{self.fence_max}'
               if self.fence_enabled else 'fence OFF'))

    # ------------------------------------------------------------------
    # Subclass hooks
    # ------------------------------------------------------------------

    def declare_extra_parameters(self) -> None:
        """Declare task-specific parameters before the policy is built."""

    def on_setup_complete(self) -> None:
        """Create extra subscriptions or warm up after wiring is in place."""

    def make_policy(self) -> Policy:
        """Return the Policy this commander flies."""
        raise NotImplementedError

    def make_observation_builder(self) -> ObservationBuilder:
        """Return the ObservationBuilder that feeds the policy."""
        raise NotImplementedError

    def get_perception(self):
        """Return the latest perception input for the observation, or None."""
        return None

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def odometry_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        self.position = np.array([p.x, p.y, p.z]) + self.position_offset
        self.orientation = np.array([q.x, q.y, q.z, q.w])
        # Odometry twist is in the body (FLU) frame per REP-145; rotate to ENU
        # world, which is the frame policies expect.
        R_flu_to_enu = Rotation.from_quat(self.orientation).as_matrix()
        self.velocity = R_flu_to_enu @ np.array([v.x, v.y, v.z])
        self.last_odom_time = self.get_clock().now()
        if self.trial_log is not None:
            self.trial_log.write('state', time_s=self.last_odom_time.nanoseconds * 1e-9,
                                 position_enu=self.position.tolist(),
                                 velocity_enu=self.velocity.tolist(),
                                 orientation_xyzw=self.orientation.tolist())

    def current_state(self) -> RobotState:
        return RobotState(
            position_enu=self.position,
            velocity_enu=self.velocity,
            orientation_xyzw=self.orientation,
            goal_enu=self.goal_position,
            perception=self.get_perception(),
        )

    # ------------------------------------------------------------------
    # Operator services
    # ------------------------------------------------------------------

    def handle_takeoff(self, request, response):
        if self.state != FlightState.IDLE:
            response.success = False
            response.message = f'{self.name}: not IDLE (state={self.state.name})'
            return response
        if self.position is None:
            response.success = False
            response.message = f'{self.name}: no odometry yet'
            return response
        self.state = FlightState.ARMING
        self.arming_start = self.get_clock().now()
        self.arming_steps_done = set()
        self.hold_target = self.hover_position.copy()
        response.success = True
        response.message = f'takeoff: {self.name}'
        return response

    def handle_start(self, request, response):
        if self.fence_breached:
            response.success = False
            response.message = 'geofence breached — call ~/reset_fence first'
            return response
        if self.state != FlightState.ACTIVE:
            response.success = False
            response.message = f'{self.name} not holding yet (state={self.state.name})'
            return response
        self._policy_last_time = None
        self.policy.reset()
        self.observation_builder.reset()
        self._begin_trial()
        self.mission_active = True
        response.success = True
        response.message = f'policy running toward {self.goal_position.round(3)}'
        self.get_logger().info(response.message)
        return response

    def handle_hold(self, request, response):
        self.mission_active = False
        self._end_trial('operator_hold')
        if self.position is not None and self.state in (
                FlightState.ASCEND, FlightState.FACE_GOAL, FlightState.ACTIVE):
            self.hold_target = self.position.copy()
            self.state = FlightState.ACTIVE
            response.success = True
            response.message = f'holding: {self.name}'
        else:
            response.success = False
            response.message = 'nothing to hold'
        return response

    def handle_land(self, request, response):
        self.mission_active = False
        self._end_trial('operator_land')
        if self.state in (FlightState.ASCEND, FlightState.FACE_GOAL, FlightState.ACTIVE):
            self.state = FlightState.LANDING
            response.success = True
            response.message = f'landing: {self.name}'
        else:
            response.success = False
            response.message = 'no airborne drone to land'
        return response

    def handle_reset_fence(self, request, response):
        still_out = (self.position is not None
                     and (np.any(self.position < self.fence_min)
                          or np.any(self.position > self.fence_max)))
        self.fence_breached = False
        response.success = True
        response.message = 'geofence latch cleared' + (
            f' (WARNING {self.name} still outside)' if still_out else '')
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Geofence and robot interface
    # ------------------------------------------------------------------

    def enforce_fence(self):
        if not self.fence_enabled or self.fence_breached:
            return
        if self.position is None or self.state not in (
                FlightState.FACE_GOAL, FlightState.ACTIVE):
            return
        if not (np.any(self.position < self.fence_min)
                or np.any(self.position > self.fence_max)):
            return
        self.fence_breached = True
        self.mission_active = False
        self._end_trial('geofence_breach')
        self.hold_target = self.position.copy()
        self.state = FlightState.ACTIVE
        self.get_logger().error(
            f'GEOFENCE BREACH by {self.name} at {self.position.round(2)} — '
            'HOLDING. Call ~/reset_fence to clear.')

    def send_robot_command(self, command: int, label: str):
        client = self.robot_command_client
        if not client.service_is_ready():
            self.get_logger().warn(
                f'{self.name}: robot_command not ready, skipping {label}')
            return
        req = RobotCommand.Request()
        req.command = command
        future = client.call_async(req)

        def report(fut, name=self.name, label=label):
            try:
                ok = fut.result().success
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'{name}: {label} failed: {e}')
                return
            self.get_logger().info(f'{name}: {label} -> success={ok}')

        future.add_done_callback(report)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def control_loop(self):
        now = self.get_clock().now()

        if self.state == FlightState.ARMING:
            elapsed = (now - self.arming_start).nanoseconds * 1e-9
            if elapsed >= ARMING_OFFBOARD_S and 'offboard' not in self.arming_steps_done:
                self.arming_steps_done.add('offboard')
                self.send_robot_command(
                    RobotCommand.Request.REQUEST_CONTROL, 'request offboard')
            if elapsed >= ARMING_ARM_S and 'arm' not in self.arming_steps_done:
                self.arming_steps_done.add('arm')
                self.send_robot_command(RobotCommand.Request.ARM, 'arm')
            if elapsed >= ARMING_DONE_S:
                self.state = FlightState.ASCEND
                self.get_logger().info(f'{self.name}: ascending to {self.hold_target}')

        self.enforce_fence()

        if self.position is None:
            return

        fresh = (self.last_odom_time is not None
                 and (now - self.last_odom_time) < Duration(seconds=self.state_timeout))
        if not fresh and self.state not in (FlightState.IDLE, FlightState.ARMING):
            self._end_trial('stale_odometry')
            self.get_logger().warn(
                f'{self.name}: odometry stale, holding position',
                throttle_duration_sec=1.0)
            self.publish_pose(self.hold_target, self.hold_orientation, now)
            self.publish_markers(now)
            return

        if self.state == FlightState.ARMING:
            self.publish_pose(self.position, self.orientation, now)

        elif self.state == FlightState.ASCEND:
            if np.linalg.norm(self.hold_target - self.position) < self.arrival_threshold:
                self.state = FlightState.FACE_GOAL
                self.get_logger().info(f'{self.name}: holding takeoff position')
            self.publish_pose(self.hold_target, self.orientation, now)

        elif self.state == FlightState.FACE_GOAL:
            target_yaw = math.atan2(
                self.goal_position[1] - self.position[1],
                self.goal_position[0] - self.position[0])
            goal_q = np.array([0.0, 0.0,
                               math.sin(target_yaw / 2), math.cos(target_yaw / 2)])
            self.publish_pose(self.hold_target, goal_q, now)
            current_yaw = math.atan2(
                2.0 * (self.orientation[3] * self.orientation[2]
                       + self.orientation[0] * self.orientation[1]),
                1.0 - 2.0 * (self.orientation[1] ** 2 + self.orientation[2] ** 2))
            if abs(math.remainder(target_yaw - current_yaw, 2 * math.pi)) < self.face_goal_threshold:
                self.hold_target = self.position.copy()
                self.hold_orientation = goal_q.copy()
                self.state = FlightState.ACTIVE
                self.get_logger().info(f'{self.name}: facing goal → ACTIVE')

        elif self.state == FlightState.ACTIVE:
            self.step_active(now)

        elif self.state == FlightState.LANDING:
            if self.position[2] <= self.land_complete_alt:
                self.send_robot_command(RobotCommand.Request.DISARM, 'disarm')
                self.state = FlightState.IDLE
                self.get_logger().info(f'{self.name}: landed, disarmed')
            else:
                self.publish_velocity(np.array([0.0, 0.0, -self.land_speed]), now)

        self.publish_markers(now)

    def step_active(self, now):
        if not self.mission_active:
            self.publish_pose(self.hold_target, self.hold_orientation, now)
            return
        if np.linalg.norm(self.goal_position - self.position) < self.goal_arrival_threshold:
            # Cruise policies do not position-hold; hand off to a stable pose-hold
            # at the goal instead of letting the policy overshoot and oscillate.
            self.mission_active = False
            self._end_trial('goal_reached')
            self.hold_target = self.goal_position.copy()
            self.hold_orientation = self.orientation.copy()
            self.get_logger().info(
                f'{self.name}: reached goal {self.goal_position.round(2)} → HOLD')
            self.publish_pose(self.hold_target, self.hold_orientation, now)
            return
        if self._policy_last_time is not None:
            gap_s = (now - self._policy_last_time).nanoseconds * 1e-9
            if gap_s > 0.2:
                self.policy.reset()
                self.observation_builder.reset()
        self._policy_last_time = now
        setpoint = self.policy.compute(self.observation_builder.build(self.current_state()))
        if self.trial_log is not None:
            self.trial_log.write('policy_command', time_s=now.nanoseconds * 1e-9,
                                 setpoint=self._setpoint_record(setpoint))
        self.publish_setpoint(setpoint, now)

    def destroy_node(self):
        self._end_trial('shutdown')
        return super().destroy_node()

    def _begin_trial(self):
        root = str(self.get_parameter('trial_log_dir').value)
        if not root:
            return
        from .trial_log import TrialLog
        if self.trial_log is not None:
            self._end_trial('restarted')
        policy_name = str(self.get_parameter('policy_name').value) or self.policy.__class__.__name__
        metadata = {
            'policy': policy_name,
            'robot': self.name,
            'mode': self.mode,
            'goal_enu': self.goal_position.tolist(),
            'start_enu': self.position.tolist(),
            'checkpoint_path': str(self.get_parameter('checkpoint_path').value)
                if self.has_parameter('checkpoint_path') else None,
            'frame': 'ENU',
        }
        self.trial_log = TrialLog(root, metadata)
        self.get_logger().info(f'trial log: {self.trial_log.path}')

    def _end_trial(self, reason):
        if self.trial_log is not None:
            self.trial_log.close(reason, time_s=self.get_clock().now().nanoseconds * 1e-9)
            self.trial_log = None

    @staticmethod
    def _setpoint_record(setpoint):
        if isinstance(setpoint, AttitudeSetpoint):
            return {'type': 'attitude_thrust',
                    'attitude_xyzw': setpoint.attitude_enu_flu_xyzw.tolist(),
                    'thrust_norm': float(setpoint.thrust_norm)}
        if isinstance(setpoint, VelocitySetpoint):
            return {'type': 'velocity', 'velocity_enu': setpoint.velocity_enu.tolist()}
        if isinstance(setpoint, WaypointSetpoint):
            return {'type': 'waypoint', 'position_enu': setpoint.position_enu.tolist(),
                    'yaw': setpoint.yaw}
        raise TypeError(f'unknown setpoint type: {type(setpoint).__name__}')

    # ------------------------------------------------------------------
    # Setpoint publishing
    # ------------------------------------------------------------------

    def publish_setpoint(self, setpoint, now):
        if isinstance(setpoint, AttitudeSetpoint):
            self.publish_attitude(setpoint.attitude_enu_flu_xyzw, setpoint.thrust_norm, now)
        elif isinstance(setpoint, VelocitySetpoint):
            self.publish_velocity(setpoint.velocity_enu, now)
        elif isinstance(setpoint, WaypointSetpoint):
            q = self.hold_orientation if setpoint.yaw is None else np.array(
                [0.0, 0.0, math.sin(setpoint.yaw / 2), math.cos(setpoint.yaw / 2)])
            self.publish_pose(setpoint.position_enu, q, now)
        else:
            raise TypeError(f'unknown setpoint type: {type(setpoint).__name__}')

    def publish_velocity(self, velocity, now):
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = float(velocity[0])
        msg.twist.linear.y = float(velocity[1])
        msg.twist.linear.z = float(velocity[2])
        self.vel_pub.publish(msg)

    def publish_pose(self, position, orientation, now):
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
        self.pose_pub.publish(msg)

    def publish_attitude(self, xyzw, thrust_norm, now):
        msg = AttitudeThrust()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.attitude.x = float(xyzw[0])
        msg.attitude.y = float(xyzw[1])
        msg.attitude.z = float(xyzw[2])
        msg.attitude.w = float(xyzw[3])
        msg.thrust.z = float(thrust_norm)
        self.att_pub.publish(msg)

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------

    def publish_markers(self, now):
        if self.viz_pub is None or self.position is None:
            return
        arr = MarkerArray()
        stamp = now.to_msg()
        r, g, b = (0.9, 0.2, 0.2) if self.mode == 'real' else (0.2, 0.7, 1.0)
        if self.fence_breached:
            r, g, b = (1.0, 0.3, 0.0)

        body = Marker()
        body.header.frame_id = self.viz_frame
        body.header.stamp = stamp
        body.ns = 'body'
        body.id = 0
        body.type = Marker.SPHERE
        body.action = Marker.ADD
        body.pose.position.x = float(self.position[0])
        body.pose.position.y = float(self.position[1])
        body.pose.position.z = float(self.position[2])
        body.pose.orientation.w = 1.0
        body.scale.x = body.scale.y = body.scale.z = 0.3
        body.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        arr.markers.append(body)

        goal = Marker()
        goal.header.frame_id = self.viz_frame
        goal.header.stamp = stamp
        goal.ns = 'goal'
        goal.id = 1
        goal.type = Marker.SPHERE
        goal.action = Marker.ADD
        goal.pose.position.x = float(self.goal_position[0])
        goal.pose.position.y = float(self.goal_position[1])
        goal.pose.position.z = float(self.goal_position[2])
        goal.pose.orientation.w = 1.0
        goal.scale.x = goal.scale.y = goal.scale.z = 0.25
        goal.color = ColorRGBA(r=0.0, g=1.0, b=0.4, a=0.8)
        arr.markers.append(goal)

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
