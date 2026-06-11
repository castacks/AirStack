"""Central multi-drone ground commander.

One node commands the whole swarm through the AirStack robot_interface
abstraction (works unchanged over MAVROS in sim and px4_interface/uXRCE-DDS
on hardware — only the topic templates in the config YAML differ):

    state in:    {state_topic_template}            nav_msgs/Odometry (ENU)
    command out: {velocity_command_topic_template} geometry_msgs/TwistStamped (ENU)
    services:    {robot_command_service_template}  airstack_msgs/srv/RobotCommand

Per-drone roles (config):
    hover    — flies to and holds its hover_position; velocity is CBF-filtered
    teleop   — streams operator velocity (the moving obstacle); CBF-exempt
    external — tracked for CBF state only, never commanded (e.g. RC-flown)

Control loop (control_rate_hz):
    nominal velocities (P-controller to hover targets / teleop input)
        -> cbf_filter.filter_velocities()   [PLACEHOLDER until the
                                             drone_soccer filter is dropped in]
        -> TwistStamped per commanded drone

Operator services (std_srvs/Trigger):
    ~/takeoff  — arm + offboard + ascend all hover/teleop drones
    ~/land     — descend all commanded drones, disarm on touchdown
    ~/hold     — overwrite every hover target with the current position
"""

import math
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


class FlightState(Enum):
    IDLE = 0       # on the ground, not commanded
    ARMING = 1     # streaming zero setpoints, requesting offboard + arm
    ASCEND = 2     # climbing to the hover target
    ACTIVE = 3     # holding hover target / following teleop
    LANDING = 4    # descending; disarm at land_complete_altitude


# Seconds after entering ARMING at which each step fires.
ARMING_STREAM_S = 1.0      # stream zero setpoints before requesting offboard
ARMING_OFFBOARD_S = 1.0    # request offboard (REQUEST_CONTROL)
ARMING_ARM_S = 1.5         # arm
ARMING_DONE_S = 2.5        # transition to ASCEND


class DroneHandle:
    """Book-keeping for one drone."""

    def __init__(self, name: str, role: str, hover_target: np.ndarray):
        self.name = name
        self.role = role
        self.hover_target = hover_target
        self.state = FlightState.IDLE
        self.position = None        # np (3,) ENU, None until first odometry
        self.velocity = np.zeros(3)
        self.last_odom_time = None  # rclpy Time
        self.arming_start = None    # rclpy Time
        self.arming_steps_done = set()
        self.cmd_pub = None
        self.robot_command_client = None

    @property
    def commanded(self) -> bool:
        return self.role in ('hover', 'teleop')


class SwarmCommander(Node):

    def __init__(self):
        super().__init__('swarm_commander')

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('drone_names', ['drone_1', 'drone_2', 'drone_3'])
        self.declare_parameter('drone_roles', ['hover', 'hover', 'teleop'])
        # Flat [x1,y1,z1, x2,y2,z2, ...] ENU, one triple per drone_names entry.
        # Hover drones hold this position; the teleop drone ascends to it on
        # takeoff before operator control; external entries are ignored.
        self.declare_parameter('hover_positions',
                               [-1.5, 0.0, 1.2, 1.5, 0.0, 1.2, 0.0, -1.5, 1.2])

        self.declare_parameter('state_topic_template',
                               '/{name}/odometry_conversion/odometry')
        self.declare_parameter('velocity_command_topic_template',
                               '/{name}/interface/velocity_command')
        self.declare_parameter('robot_command_service_template',
                               '/{name}/interface/robot_command')
        self.declare_parameter('teleop_topic', '/svg/teleop_command')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('state_timeout_s', 0.5)
        self.declare_parameter('teleop_timeout_s', 0.5)

        # Hover P-controller
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

        names = list(self.get_parameter('drone_names').value)
        roles = list(self.get_parameter('drone_roles').value)
        hover_flat = list(self.get_parameter('hover_positions').value)

        if len(roles) != len(names):
            raise ValueError(
                f'drone_roles has {len(roles)} entries for {len(names)} drones')
        if len(hover_flat) != 3 * len(names):
            raise ValueError(
                f'hover_positions needs {3 * len(names)} values, got {len(hover_flat)}')
        for role in roles:
            if role not in ('hover', 'teleop', 'external'):
                raise ValueError(f'unknown drone role "{role}"')

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

        state_tmpl = str(self.get_parameter('state_topic_template').value)
        cmd_tmpl = str(self.get_parameter('velocity_command_topic_template').value)
        srv_tmpl = str(self.get_parameter('robot_command_service_template').value)

        # ---- Per-drone wiring --------------------------------------------
        self.drones = []
        for i, (name, role) in enumerate(zip(names, roles)):
            drone = DroneHandle(name, role, np.array(hover_flat[3 * i:3 * i + 3]))
            if drone.commanded:
                drone.cmd_pub = self.create_publisher(
                    TwistStamped, cmd_tmpl.format(name=name), 10)
                drone.robot_command_client = self.create_client(
                    RobotCommand, srv_tmpl.format(name=name))
            self.create_subscription(
                Odometry, state_tmpl.format(name=name),
                lambda msg, d=drone: self.odometry_callback(d, msg), 10)
            self.drones.append(drone)

        # ---- Teleop input -------------------------------------------------
        self.teleop_twist = np.zeros(3)
        self.last_teleop_time = None
        self.create_subscription(
            TwistStamped, str(self.get_parameter('teleop_topic').value),
            self.teleop_callback, 10)

        # ---- Operator services ---------------------------------------------
        self.create_service(Trigger, '~/takeoff', self.handle_takeoff)
        self.create_service(Trigger, '~/land', self.handle_land)
        self.create_service(Trigger, '~/hold', self.handle_hold)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            'SwarmCommander up: '
            + ', '.join(f'{d.name}({d.role})' for d in self.drones)
            + f' | CBF r={self.cbf_safety_radius} m, vmax={self.cbf_max_speed} m/s'
            + ' | NOTE: cbf_filter is a PLACEHOLDER (speed cap only)')

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def odometry_callback(self, drone: DroneHandle, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        drone.position = np.array([p.x, p.y, p.z])
        drone.velocity = np.array([v.x, v.y, v.z])
        drone.last_odom_time = self.get_clock().now()

    def teleop_callback(self, msg: TwistStamped):
        l = msg.twist.linear
        self.teleop_twist = np.array([l.x, l.y, l.z])
        self.last_teleop_time = self.get_clock().now()

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
            started.append(d.name)
        response.success = bool(started)
        response.message = ('takeoff: ' + ', '.join(started)) if started \
            else 'no drone eligible for takeoff (missing odometry or not IDLE)'
        return response

    def handle_land(self, request, response):
        landing = []
        for d in self.drones:
            if d.commanded and d.state in (FlightState.ASCEND, FlightState.ACTIVE):
                d.state = FlightState.LANDING
                landing.append(d.name)
        response.success = bool(landing)
        response.message = ('landing: ' + ', '.join(landing)) if landing \
            else 'no airborne drone to land'
        return response

    def handle_hold(self, request, response):
        held = []
        for d in self.drones:
            if d.commanded and d.position is not None \
                    and d.state in (FlightState.ASCEND, FlightState.ACTIVE):
                d.hover_target = d.position.copy()
                d.role = 'hover'
                d.state = FlightState.ACTIVE
                held.append(d.name)
        response.success = bool(held)
        response.message = 'holding: ' + ', '.join(held) if held else 'nothing to hold'
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
                self.get_logger().info(f'{d.name}: ascending to {d.hover_target}')

        # Swarm state for the CBF: every drone with fresh odometry, any role.
        tracked = [d for d in self.drones
                   if d.position is not None and d.last_odom_time is not None
                   and (now - d.last_odom_time) < Duration(seconds=self.state_timeout)]
        index = {d.name: i for i, d in enumerate(tracked)}

        if not tracked:
            return

        positions = np.stack([d.position for d in tracked])
        nominal = np.zeros((len(tracked), 3))
        teleop_rows = []

        for d in tracked:
            i = index[d.name]
            if d.state in (FlightState.IDLE, FlightState.ARMING):
                nominal[i] = 0.0
            elif d.state == FlightState.LANDING:
                nominal[i] = np.array([0.0, 0.0, -self.land_speed])
            elif d.role == 'teleop' and d.state == FlightState.ACTIVE:
                stale = (self.last_teleop_time is None
                         or (now - self.last_teleop_time)
                         > Duration(seconds=self.teleop_timeout))
                nominal[i] = np.zeros(3) if stale else self.teleop_twist
                teleop_rows.append(i)
            else:  # hover role, ASCEND or ACTIVE
                error = d.hover_target - d.position
                nominal[i] = self.hover_kp * error
                if d.state == FlightState.ASCEND \
                        and np.linalg.norm(error) < self.arrival_threshold:
                    d.state = FlightState.ACTIVE
                    self.get_logger().info(f'{d.name}: holding hover target')

        # ================= CBF SAFETY FILTER =================
        # Placeholder until the drone_soccer filter is dropped into
        # cbf_filter.py — see that file. The teleoperated drone is the
        # moving obstacle: its row is restored to the (speed-capped)
        # operator command after filtering, so only the hovering drones
        # are deflected.
        result = filter_velocities(
            nominal, positions,
            safety_radius=self.cbf_safety_radius,
            max_speed=self.cbf_max_speed,
            alpha=self.cbf_alpha,
        )
        safe = result.velocities
        for i in teleop_rows:
            cmd = nominal[i]
            speed = np.linalg.norm(cmd)
            if speed > self.teleop_max_speed:
                cmd = cmd * (self.teleop_max_speed / speed)
            safe[i] = cmd
        if result.used_emergency_stop:
            self.get_logger().warn('CBF emergency push-apart engaged')
        # ======================================================

        # Publish commands; handle landing completion.
        for d in self.drones:
            if not d.commanded or d.state == FlightState.IDLE:
                continue
            if d.name not in index:
                # Commanded drone with stale state: fail safe to zero velocity.
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
