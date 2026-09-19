"""Gamepad teleop node: /joy in, TwistStamped out on the commander's topic.

Reads sensor_msgs/Joy from the standard `joy` node and the drone's odometry,
runs VelocityMapper, and publishes a world-frame ENU velocity on
/svg/{drone}/teleop_command.

The right stick sets horizontal velocity directly. The left stick sets the
rate of change of a target altitude, so releasing it holds the current height
and the vertical velocity is a correction toward that target. This is why the
node needs odometry, unlike a teleop node that only maps sticks to velocity.

The drone must be listed in the commander's `teleop_drones`, and teleop takes
effect only after /swarm_commander/start.

Which physical device is in use is the `teleop_controller` parameter (see
controllers.py). It supplies the default axis numbers, signs, lock button and
joy topic for that device; the individual axis parameters can still override
them for an odd driver build.
"""

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .pad import PadState
from .velocity import (ALTITUDE_GAIN, CLIMB_RATE_MPS, DEADZONE, MAX_ALTITUDE_M,
                       MAX_CLIMB_SPEED_MPS, MAX_SPEED_MPS, MIN_ALTITUDE_M,
                       YAW_RATE_RAD_S, VelocityMapper)
from .controllers import DEFAULT_CONTROLLER, controller_names, get_controller


class SafeTeleopNode(Node):

    def __init__(self):
        super().__init__('safe_teleop')

        self.declare_parameter('drone', 'drone_1')
        # Input device. Names an entry of controllers.CONTROLLERS; its axis
        # map becomes the default for the axis/sign/button parameters below.
        self.declare_parameter('teleop_controller', DEFAULT_CONTROLLER)
        controller_name = str(self.get_parameter('teleop_controller').value)
        try:
            self.controller = get_controller(controller_name)
        except KeyError as error:
            self.get_logger().fatal(str(error))
            raise SystemExit(
                f"teleop_controller must be one of {controller_names()}")
        profile = self.controller

        self.declare_parameter('joy_topic', profile.joy_topic)
        self.declare_parameter('teleop_topic_template',
                               '/svg/{name}/teleop_command')
        self.declare_parameter('odometry_topic_template',
                               '/{name}/odometry_conversion/odometry')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('joy_timeout_s', 0.5)
        self.declare_parameter('odometry_timeout_s', 0.5)
        # Print the stick reading + published velocity this often (0 = never).
        # teleop.launch.py sets 1 Hz so the pad can be checked before the
        # commander is up.
        self.declare_parameter('print_hz', 0.0)

        self.declare_parameter('max_speed_mps', MAX_SPEED_MPS)
        self.declare_parameter('climb_rate_mps', CLIMB_RATE_MPS)
        self.declare_parameter('altitude_gain', ALTITUDE_GAIN)
        self.declare_parameter('max_climb_speed_mps', MAX_CLIMB_SPEED_MPS)
        self.declare_parameter('min_altitude_m', MIN_ALTITUDE_M)
        self.declare_parameter('max_altitude_m', MAX_ALTITUDE_M)
        self.declare_parameter('deadzone', DEADZONE)

        # Axis map: defaults come from the controller profile. Set one of
        # these explicitly only to correct a driver that maps differently.
        for key, default in profile.mapping_parameters().items():
            self.declare_parameter(key, default)
        self.declare_parameter('yaw_rate_rad_s', YAW_RATE_RAD_S)

        def value(name):
            return self.get_parameter(name).value

        self.mapper = VelocityMapper(
            max_speed=float(value('max_speed_mps')),
            climb_rate=float(value('climb_rate_mps')),
            altitude_gain=float(value('altitude_gain')),
            max_climb_speed=float(value('max_climb_speed_mps')),
            min_altitude=float(value('min_altitude_m')),
            max_altitude=float(value('max_altitude_m')),
            deadzone_width=float(value('deadzone')),
            forward_axis=int(value('forward_axis')),
            left_axis=int(value('left_axis')),
            climb_axis=int(value('climb_axis')),
            lock_button=int(value('lock_button')),
            yaw_axis=int(value('yaw_axis')),
            yaw_sign=float(value('yaw_sign')),
            yaw_rate=float(value('yaw_rate_rad_s')),
            forward_sign=float(value('forward_sign')),
            left_sign=float(value('left_sign')),
            climb_sign=float(value('climb_sign')),
        )

        self.joy_timeout = Duration(seconds=float(value('joy_timeout_s')))
        self.odometry_timeout = Duration(
            seconds=float(value('odometry_timeout_s')))

        self.joy = None
        self.last_joy_time = None
        self.altitude = None
        self.last_odometry_time = None
        self.last_tick = None
        self.was_locked = False

        drone = str(value('drone'))
        self.publisher = self.create_publisher(
            TwistStamped, str(value('teleop_topic_template')).format(name=drone), 10)
        self.create_subscription(Joy, str(value('joy_topic')),
                                 self.joy_callback, 10)
        self.create_subscription(
            Odometry, str(value('odometry_topic_template')).format(name=drone),
            self.odometry_callback, 10)

        rate = float(value('publish_rate_hz'))
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.last_command = None
        print_hz = float(value('print_hz'))
        if print_hz > 0.0:
            self.create_timer(1.0 / print_hz, self.print_status)

        self.get_logger().info(
            f'safe_teleop driving {drone} with {profile.name} '
            f'({profile.description}): publishing {self.publisher.topic_name}, '
            f'reading {value("joy_topic")} and '
            f'{str(value("odometry_topic_template")).format(name=drone)}')

    def joy_callback(self, msg: Joy):
        self.joy = msg
        self.last_joy_time = self.get_clock().now()

    def odometry_callback(self, msg: Odometry):
        self.altitude = msg.pose.pose.position.z
        self.last_odometry_time = self.get_clock().now()

    def _fresh(self, stamp, timeout) -> bool:
        return stamp is not None and (self.get_clock().now() - stamp) < timeout

    def tick(self):
        now = self.get_clock().now()
        dt = 1e-9 * (now - self.last_tick).nanoseconds if self.last_tick else 0.0
        self.last_tick = now

        # No odometry means no altitude to hold against. Publish zero and drop
        # the target so it is re-adopted from wherever the drone actually is
        # when odometry comes back, instead of correcting to a stale height.
        if not self._fresh(self.last_odometry_time, self.odometry_timeout):
            if self.mapper.target_altitude is not None:
                self.get_logger().warn('odometry stale, holding zero velocity',
                                       throttle_duration_sec=2.0)
                self.mapper.target_altitude = None
            self.last_command = None
            self.publish(0.0, 0.0, 0.0)
            return

        joy_fresh = self._fresh(self.last_joy_time, self.joy_timeout)
        if self.joy is None or not joy_fresh:
            self.get_logger().warn('joy stale, holding zero velocity',
                                   throttle_duration_sec=2.0)
        state = self.pad_state(joy_fresh)

        command = self.mapper.update(state, dt, self.altitude)
        self.last_command = command
        if command.held != self.was_locked:
            self.was_locked = command.held
            self.get_logger().info(
                f'left stick {"locked" if command.held else "released"}, '
                f'target altitude {command.target_altitude:.2f} m')
        self.publish(command.vx, command.vy, command.vz, command.yaw_rate)

    def print_status(self):
        """One line: what the pad reads and what is being published.

        Meant for the terminal teleop.launch.py runs in, so the sticks can be
        checked with nothing else up. Without the pad it says so; without
        odometry it shows the sticks but a zero command (the altitude hold
        needs the drone's height).
        """
        joy_ok = self._fresh(self.last_joy_time, self.joy_timeout)
        if not joy_ok:
            self.get_logger().info(
                f'pad: NO /joy (is {self.get_parameter("joy_topic").value} '
                'publishing? pad plugged in?) -> publishing zero velocity')
            return
        m = self.mapper
        sticks = (f'fwd {self.joy.axes[m.forward_axis]:+.2f} '
                  f'left {self.joy.axes[m.left_axis]:+.2f} '
                  f'climb {self.joy.axes[m.climb_axis]:+.2f} '
                  f'yaw {self.joy.axes[m.yaw_axis]:+.2f}'
                  if len(self.joy.axes) > max(m.forward_axis, m.left_axis,
                                               m.climb_axis, m.yaw_axis)
                  else f'{len(self.joy.axes)} axes (fewer than the map needs!)')
        if self.last_command is None:
            self.get_logger().info(
                f'pad: {sticks} | NO odometry -> publishing zero velocity '
                '(sticks work; vz needs the drone height)')
            return
        c = self.last_command
        self.get_logger().info(
            f'pad: {sticks} | cmd vx {c.vx:+.2f} vy {c.vy:+.2f} vz {c.vz:+.2f} '
            f'yaw {c.yaw_rate:+.2f} | alt {c.altitude:.2f} -> {c.target_altitude:.2f} m'
            f'{" | LOCKED" if c.held else ""}')

    def pad_state(self, connected: bool) -> PadState:
        """A Joy message as the PadState the mapper expects."""
        axes = tuple(self.joy.axes) if self.joy else ()
        buttons = tuple(bool(b) for b in self.joy.buttons) if self.joy else ()
        return PadState(axes=axes,
                        raw_axes=tuple(int(a * 32767) for a in axes),
                        buttons=buttons, connected=connected)

    def publish(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SafeTeleopNode()
    except SystemExit as error:
        print(error)
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
