"""Central multi-drone ground commander with a CBF collision safety filter.

One node commands the whole swarm through the AirStack robot_interface
abstraction (works unchanged over MAVROS in sim and px4_interface/uXRCE-DDS
on hardware — only the topic templates in the config YAML differ):

    state in:    {state_topic_template}            nav_msgs/Odometry (ENU)
    command out: {velocity_command_topic_template} geometry_msgs/TwistStamped (ENU)
    services:    {robot_command_service_template}  airstack_msgs/srv/RobotCommand

Nominal commands come from a *scenario* (hover, random_walk, random_goals,
head_on, antipodal, squeeze — see scenarios.py, ported from ~/drone_soccer).
Drones listed in ``teleop_drones`` are operator-driven instead (one teleop
topic per drone); an empty list means every drone follows the scenario.
Drones in ``external_drones`` are tracked for the safety filter but never
commanded (e.g. RC-flown).

Every commanded velocity passes through the velocity-CBF filter
(cbf_filter.filter_velocities, ported from drone_soccer/cbf.py). Teleop
drones are CBF-EXEMPT by design — they play the moving obstacle; the
autonomous drones do the dodging.

Lifecycle (std_srvs/Trigger services):
    ~/takeoff — arm + offboard + ascend everyone to the scenario's initial
                positions, then HOLD there
    ~/start   — begin the scenario (nominal policies go live)
    ~/hold    — pause: every drone holds its current position (panic button)
    ~/land    — descend all commanded drones, disarm on touchdown
"""

from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from airstack_msgs.srv import RobotCommand

from svg_ground_control.cbf_filter import filter_velocities
from svg_ground_control.scenarios import Bounds, make_scenario


class FlightState(Enum):
    IDLE = 0       # on the ground, not commanded
    ARMING = 1     # streaming zero setpoints, requesting offboard + arm
    ASCEND = 2     # climbing to the takeoff target
    ACTIVE = 3     # holding / following the scenario or teleop
    LANDING = 4    # descending; disarm at land_complete_altitude


# Seconds after entering ARMING at which each step fires.
ARMING_OFFBOARD_S = 1.0    # request offboard (REQUEST_CONTROL)
ARMING_ARM_S = 1.5         # arm
ARMING_DONE_S = 2.5        # transition to ASCEND


class DroneHandle:
    """Book-keeping for one drone."""

    def __init__(self, name: str, role: str):
        self.name = name
        self.role = role                  # 'auto' | 'teleop' | 'external'
        self.takeoff_target = None        # np (3,), set from the scenario
        self.hold_target = None           # np (3,), position to hold when not in mission
        self.state = FlightState.IDLE
        self.position = None              # np (3,) ENU, None until first odometry
        self.velocity = np.zeros(3)
        self.last_odom_time = None        # rclpy Time
        self.arming_start = None          # rclpy Time
        self.arming_steps_done = set()
        self.cmd_pub = None
        self.robot_command_client = None
        self.teleop_twist = np.zeros(3)
        self.last_teleop_time = None

    @property
    def commanded(self) -> bool:
        return self.role in ('auto', 'teleop')


class SwarmCommander(Node):

    def __init__(self):
        super().__init__('swarm_commander')

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('drone_names', ['drone_1', 'drone_2', 'drone_3'])
        # Comma-separated names of operator-driven drones (CBF-exempt moving
        # obstacles). Empty string = every drone follows the scenario.
        # (A string, not a list: an empty YAML list has no type and cannot
        # override a string-array parameter default.)
        self.declare_parameter('teleop_drones', '')
        # Comma-separated names tracked for the safety filter but never
        # commanded (e.g. RC-flown).
        self.declare_parameter('external_drones', '')

        # Scenario selection — see scenarios.py. NOTE: for 'squeeze' the
        # drone_names order matters: [holder, holder, intruder].
        self.declare_parameter('scenario', 'hover')
        self.declare_parameter('scenario_speed_mps', 0.6)
        self.declare_parameter('scenario_seed', 7)
        self.declare_parameter('arena_low', [-2.0, -2.0, 0.8])
        self.declare_parameter('arena_high', [2.0, 2.0, 2.0])
        # squeeze scenario geometry (ENU, meters), set both explicitly:
        # the two holder posts as flat [x1,y1,z1, x2,y2,z2] ...
        self.declare_parameter('squeeze_holder_positions',
                               [0.0, -0.69, 1.2, 0.0, 0.69, 1.2])
        # ... and the two waypoints the intruder shuttles between, flat
        # [ax,ay,az, bx,by,bz]; it starts at A and flies toward B first.
        self.declare_parameter('squeeze_intruder_waypoints',
                               [-1.5, 0.0, 1.2, 1.5, 0.0, 1.2])
        # The intruder is the deliberate obstacle: exempt from the CBF so it
        # presses through and the holders alone yield (filtering it makes
        # the filter push it backwards as it approaches the gap).
        self.declare_parameter('squeeze_intruder_cbf_exempt', True)
        # Used by the 'hover' scenario only: flat [x1,y1,z1, ...] per drone.
        self.declare_parameter('hover_positions',
                               [-1.5, 0.0, 1.2, 1.5, 0.0, 1.2, 0.0, -1.5, 1.2])

        self.declare_parameter('state_topic_template',
                               '/{name}/odometry_conversion/odometry')
        self.declare_parameter('velocity_command_topic_template',
                               '/{name}/interface/velocity_command')
        self.declare_parameter('robot_command_service_template',
                               '/{name}/interface/robot_command')
        self.declare_parameter('teleop_topic_template', '/svg/{name}/teleop_command')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('state_timeout_s', 0.5)
        self.declare_parameter('teleop_timeout_s', 0.5)

        # Hold/ascend P-controller
        self.declare_parameter('hover_kp', 1.0)
        self.declare_parameter('arrival_threshold_m', 0.15)

        # Landing
        self.declare_parameter('land_speed_mps', 0.3)
        self.declare_parameter('land_complete_altitude_m', 0.15)

        # CBF safety filter
        self.declare_parameter('cbf_safety_radius_m', 0.55)
        self.declare_parameter('cbf_max_speed_mps', 1.2)
        self.declare_parameter('cbf_alpha', 2.5)
        self.declare_parameter('teleop_max_speed_mps', 1.2)

        def name_list(param: str) -> list:
            raw = str(self.get_parameter(param).value)
            return [n.strip() for n in raw.split(',') if n.strip()]

        names = list(self.get_parameter('drone_names').value)
        teleop_names = name_list('teleop_drones')
        external_names = name_list('external_drones')
        for n in teleop_names + external_names:
            if n not in names:
                raise ValueError(f'"{n}" not in drone_names')

        self.state_timeout = float(self.get_parameter('state_timeout_s').value)
        self.teleop_timeout = float(self.get_parameter('teleop_timeout_s').value)
        self.hover_kp = float(self.get_parameter('hover_kp').value)
        self.arrival_threshold = float(self.get_parameter('arrival_threshold_m').value)
        self.land_speed = float(self.get_parameter('land_speed_mps').value)
        self.land_complete_alt = float(
            self.get_parameter('land_complete_altitude_m').value)
        self.cbf_safety_radius = float(self.get_parameter('cbf_safety_radius_m').value)
        self.cbf_max_speed = float(self.get_parameter('cbf_max_speed_mps').value)
        self.cbf_alpha = float(self.get_parameter('cbf_alpha').value)
        self.teleop_max_speed = float(self.get_parameter('teleop_max_speed_mps').value)

        # ---- Scenario -----------------------------------------------------
        scenario_name = str(self.get_parameter('scenario').value)
        scenario_kwargs = {}
        if scenario_name == 'hover':
            scenario_kwargs['hover_positions'] = np.array(
                self.get_parameter('hover_positions').value)
        elif scenario_name == 'squeeze':
            scenario_kwargs['holder_positions'] = np.array(
                self.get_parameter('squeeze_holder_positions').value)
            scenario_kwargs['intruder_waypoints'] = np.array(
                self.get_parameter('squeeze_intruder_waypoints').value)
            scenario_kwargs['intruder_cbf_exempt'] = bool(
                self.get_parameter('squeeze_intruder_cbf_exempt').value)
        self.scenario = make_scenario(
            scenario_name,
            num_drones=len(names),
            nominal_speed=float(self.get_parameter('scenario_speed_mps').value),
            bounds=Bounds(
                low=np.array(self.get_parameter('arena_low').value),
                high=np.array(self.get_parameter('arena_high').value)),
            safety_radius=self.cbf_safety_radius,
            seed=int(self.get_parameter('scenario_seed').value),
            **scenario_kwargs)
        self.scenario_name = scenario_name
        self.mission_active = False
        if scenario_name == 'squeeze':
            posts = self.scenario.holder_posts
            gap = float(np.linalg.norm(posts[0] - posts[1]))
            self.get_logger().info(
                f'squeeze geometry: posts {gap:.2f} m apart '
                f'(2r keep-out = {2 * self.cbf_safety_radius:.2f} m), '
                f'intruder A={self.scenario.intruder_waypoints[0]} '
                f'B={self.scenario.intruder_waypoints[1]}')

        state_tmpl = str(self.get_parameter('state_topic_template').value)
        cmd_tmpl = str(self.get_parameter('velocity_command_topic_template').value)
        srv_tmpl = str(self.get_parameter('robot_command_service_template').value)
        teleop_tmpl = str(self.get_parameter('teleop_topic_template').value)

        # ---- Per-drone wiring --------------------------------------------
        takeoff_targets = self.scenario.initial_positions()
        self.drones = []
        for i, name in enumerate(names):
            role = ('teleop' if name in teleop_names
                    else 'external' if name in external_names else 'auto')
            drone = DroneHandle(name, role)
            drone.takeoff_target = takeoff_targets[i].copy()
            drone.hold_target = takeoff_targets[i].copy()
            if drone.commanded:
                drone.cmd_pub = self.create_publisher(
                    TwistStamped, cmd_tmpl.format(name=name), 10)
                drone.robot_command_client = self.create_client(
                    RobotCommand, srv_tmpl.format(name=name))
            if role == 'teleop':
                self.create_subscription(
                    TwistStamped, teleop_tmpl.format(name=name),
                    lambda msg, d=drone: self.teleop_callback(d, msg), 10)
            self.create_subscription(
                Odometry, state_tmpl.format(name=name),
                lambda msg, d=drone: self.odometry_callback(d, msg), 10)
            self.drones.append(drone)

        # ---- Operator services ---------------------------------------------
        self.create_service(Trigger, '~/takeoff', self.handle_takeoff)
        self.create_service(Trigger, '~/start', self.handle_start)
        self.create_service(Trigger, '~/hold', self.handle_hold)
        self.create_service(Trigger, '~/land', self.handle_land)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        self._cbf_warn_count = 0

        self.get_logger().info(
            f'SwarmCommander up | scenario={scenario_name} | '
            + ', '.join(f'{d.name}({d.role})' for d in self.drones)
            + f' | CBF r={self.cbf_safety_radius} m, vmax={self.cbf_max_speed} m/s,'
            + f' alpha={self.cbf_alpha}')

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def odometry_callback(self, drone: DroneHandle, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        drone.position = np.array([p.x, p.y, p.z])
        drone.velocity = np.array([v.x, v.y, v.z])
        drone.last_odom_time = self.get_clock().now()

    def teleop_callback(self, drone: DroneHandle, msg: TwistStamped):
        l = msg.twist.linear
        drone.teleop_twist = np.array([l.x, l.y, l.z])
        drone.last_teleop_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # Operator services
    # ------------------------------------------------------------------

    def handle_takeoff(self, request, response):
        now = self.get_clock().now()
        started = []
        for d in self.drones:
            if not d.commanded or d.state != FlightState.IDLE:
                continue
            if d.position is None:
                self.get_logger().warn(
                    f'{d.name}: no odometry yet, refusing takeoff')
                continue
            d.state = FlightState.ARMING
            d.arming_start = now
            d.arming_steps_done = set()
            d.hold_target = d.takeoff_target.copy()
            started.append(d.name)
        response.success = bool(started)
        response.message = ('takeoff: ' + ', '.join(started)) if started \
            else 'no drone eligible for takeoff (missing odometry or not IDLE)'
        return response

    def handle_start(self, request, response):
        not_ready = [d.name for d in self.drones
                     if d.commanded and d.state != FlightState.ACTIVE]
        if not_ready:
            response.success = False
            response.message = 'not all drones holding yet: ' + ', '.join(not_ready)
            return response
        self.mission_active = True
        response.success = True
        response.message = f'scenario "{self.scenario_name}" running'
        self.get_logger().info(response.message)
        return response

    def handle_hold(self, request, response):
        self.mission_active = False
        held = []
        for d in self.drones:
            if d.commanded and d.position is not None \
                    and d.state in (FlightState.ASCEND, FlightState.ACTIVE):
                d.hold_target = d.position.copy()
                d.state = FlightState.ACTIVE
                held.append(d.name)
        response.success = bool(held)
        response.message = 'holding: ' + ', '.join(held) if held else 'nothing to hold'
        return response

    def handle_land(self, request, response):
        self.mission_active = False
        landing = []
        for d in self.drones:
            if d.commanded and d.state in (FlightState.ASCEND, FlightState.ACTIVE):
                d.state = FlightState.LANDING
                landing.append(d.name)
        response.success = bool(landing)
        response.message = ('landing: ' + ', '.join(landing)) if landing \
            else 'no airborne drone to land'
        return response

    # ------------------------------------------------------------------
    # Robot interface helpers
    # ------------------------------------------------------------------

    def send_robot_command(self, drone: DroneHandle, command: int, label: str):
        client = drone.robot_command_client
        if not client.service_is_ready():
            self.get_logger().warn(
                f'{drone.name}: robot_command service not ready, skipping {label}')
            return
        req = RobotCommand.Request()
        req.command = command
        future = client.call_async(req)

        def report(fut, name=drone.name, label=label):
            try:
                ok = fut.result().success
            except Exception as e:  # noqa: BLE001 - log any service failure
                self.get_logger().error(f'{name}: {label} failed: {e}')
                return
            level = self.get_logger().info if ok else self.get_logger().error
            level(f'{name}: {label} -> success={ok}')

        future.add_done_callback(report)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def teleop_command(self, drone: DroneHandle, now) -> np.ndarray:
        stale = (drone.last_teleop_time is None
                 or (now - drone.last_teleop_time)
                 > Duration(seconds=self.teleop_timeout))
        cmd = np.zeros(3) if stale else drone.teleop_twist.copy()
        speed = np.linalg.norm(cmd)
        if speed > self.teleop_max_speed:
            cmd *= self.teleop_max_speed / speed
        return cmd

    def control_loop(self):
        now = self.get_clock().now()

        # Advance ARMING state machines (time-staged, while zeros stream below).
        for d in self.drones:
            if d.state != FlightState.ARMING:
                continue
            elapsed = (now - d.arming_start).nanoseconds * 1e-9
            if elapsed >= ARMING_OFFBOARD_S and 'offboard' not in d.arming_steps_done:
                d.arming_steps_done.add('offboard')
                self.send_robot_command(d, RobotCommand.Request.REQUEST_CONTROL,
                                        'request offboard')
            if elapsed >= ARMING_ARM_S and 'arm' not in d.arming_steps_done:
                d.arming_steps_done.add('arm')
                self.send_robot_command(d, RobotCommand.Request.ARM, 'arm')
            if elapsed >= ARMING_DONE_S:
                d.state = FlightState.ASCEND
                self.get_logger().info(f'{d.name}: ascending to {d.hold_target}')

        # Swarm state: every drone with a known position (any role) feeds the
        # CBF; freshness only gates whether a drone gets commands published.
        tracked = [d for d in self.drones if d.position is not None]
        if not tracked:
            return
        index = {d.name: i for i, d in enumerate(tracked)}
        positions = np.stack([d.position for d in tracked])

        # Scenario nominal velocities — only meaningful (and stateful: goal
        # resampling, wall bounces) once the mission runs and all drones are
        # tracked, so it is stepped exactly then.
        scenario_nominal = None
        if self.mission_active and len(tracked) == len(self.drones):
            all_positions = np.stack([d.position for d in self.drones])
            scenario_nominal = self.scenario.nominal_velocity(all_positions)

        scenario_exempt = (set(self.scenario.cbf_exempt_indices)
                           if self.mission_active else set())

        nominal = np.zeros((len(tracked), 3))
        exempt_rows = set()   # obstacle rows: restored after filtering
        for d in tracked:
            i = index[d.name]
            if d.state in (FlightState.IDLE, FlightState.ARMING):
                nominal[i] = 0.0
            elif d.state == FlightState.LANDING:
                nominal[i] = np.array([0.0, 0.0, -self.land_speed])
            elif d.state == FlightState.ASCEND:
                error = d.hold_target - d.position
                nominal[i] = self.hover_kp * error
                if np.linalg.norm(error) < self.arrival_threshold:
                    d.state = FlightState.ACTIVE
                    self.get_logger().info(f'{d.name}: holding takeoff position')
            elif d.state == FlightState.ACTIVE:
                if d.role == 'teleop' and self.mission_active:
                    nominal[i] = self.teleop_command(d, now)
                    exempt_rows.add(i)
                elif self.mission_active and scenario_nominal is not None:
                    drone_index = self.drones.index(d)
                    nominal[i] = scenario_nominal[drone_index]
                    if drone_index in scenario_exempt:
                        exempt_rows.add(i)
                else:
                    nominal[i] = self.hover_kp * (d.hold_target - d.position)

        # ================= CBF SAFETY FILTER =================
        # Real velocity-CBF (ported from drone_soccer). Exempt rows are the
        # deliberate moving obstacles (teleop drones + scenario-designated
        # ones like the squeeze intruder): their rows are restored to the
        # speed-capped nominal after filtering, so only the other drones
        # dodge — filtering an obstacle would push it back from the
        # conflict instead of letting it force the others to yield.
        result = filter_velocities(
            nominal, positions,
            safety_radius=self.cbf_safety_radius,
            max_speed=self.cbf_max_speed,
            alpha=self.cbf_alpha,
        )
        safe = result.velocities
        for i in exempt_rows:
            cmd = nominal[i].copy()
            speed = np.linalg.norm(cmd)
            if speed > self.cbf_max_speed:
                cmd *= self.cbf_max_speed / speed
            safe[i] = cmd
        if result.used_emergency_stop:
            self.get_logger().warn(
                'CBF emergency push-apart engaged '
                f'(infeasible pairs: {result.num_infeasible})',
                throttle_duration_sec=1.0)
        elif result.corrected.any():
            self._cbf_warn_count += 1
            if self._cbf_warn_count % 20 == 1:  # ~1 Hz at 20 Hz loop
                active = [tracked[i].name
                          for i in np.flatnonzero(result.corrected)
                          if i not in exempt_rows]
                if active:
                    self.get_logger().info(
                        f'CBF active on: {", ".join(active)} '
                        f'(residual {result.residual:.4f})')
        # ======================================================

        # Publish commands; handle landing completion.
        for d in self.drones:
            if not d.commanded or d.state == FlightState.IDLE:
                continue
            fresh = (d.last_odom_time is not None
                     and (now - d.last_odom_time)
                     < Duration(seconds=self.state_timeout))
            if not fresh:
                self.get_logger().warn(
                    f'{d.name}: odometry stale, commanding zero velocity',
                    throttle_duration_sec=1.0)
                self.publish_velocity(d, np.zeros(3), now)
                continue

            if d.state == FlightState.LANDING \
                    and d.position[2] <= self.land_complete_alt:
                self.send_robot_command(d, RobotCommand.Request.DISARM, 'disarm')
                d.state = FlightState.IDLE
                self.get_logger().info(f'{d.name}: landed, disarmed')
                continue

            self.publish_velocity(d, safe[index[d.name]], now)

    def publish_velocity(self, drone: DroneHandle, velocity: np.ndarray, now):
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = float(velocity[0])
        msg.twist.linear.y = float(velocity[1])
        msg.twist.linear.z = float(velocity[2])
        drone.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
