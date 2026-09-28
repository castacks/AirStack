"""Gamepad teleop node: /joy in, TwistStamped out on the commander's topic.

Reads sensor_msgs/Joy from the standard `joy` node, runs VelocityMapper, and
publishes a world-frame ENU *stick velocity* on /svg/{drone}/teleop_command:
right stick = horizontal velocity, left stick = vertical velocity + yaw rate.

Position holding is NOT done here. The swarm commander flies teleop drones in
position mode (position_hold.py): it integrates this velocity into a target
it tracks, seeded from the drone's own position when control is handed over,
so releasing the sticks holds position on all three axes. That is why this
node no longer needs odometry (it only shows the altitude in its status line
when it happens to have it) and no longer keeps an altitude target of its own
— two integrators would fight.

The drone must be listed in the commander's `teleop_drones`, and teleop takes
effect only after /swarm_commander/start.

Which physical device is in use is the `teleop_controller` parameter (see
controllers.py). It supplies the default axis numbers, signs, lock button and
joy topic for that device; the individual axis parameters can still override
them for an odd driver build.

Speed cap — one number, two nodes. `max_speed_mps` (full right stick, here)
and the commander's `teleop_max_speed_mps` (its ceiling on the stick
velocity) must be the same value or the smaller one silently wins. So they
are kept equal at runtime, whichever side is changed:

* the commander's status snapshot (`commander_status_topic`) carries its live
  `teleop_max_speed_mps` in `tuning`; whenever it differs from ours, ours
  follows (`ros2 param set /swarm_commander teleop_max_speed_mps 3.0` or the
  basestation panel's Teleop vmax row change the pad too);
* a runtime set of our `max_speed_mps` (`ros2 param set /safe_teleop
  max_speed_mps 3.0`) is applied live and pushed to the commander's
  `set_parameters` — if the commander is not up yet it adopts nothing, and
  its own value wins once its snapshot arrives.

`sync_max_speed: false` turns both directions off.
"""

import json
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from .pad import PadState
from .velocity import (DEADZONE, MAX_CLIMB_SPEED_MPS, MAX_SPEED_MPS,
                       YAW_RATE_RAD_S, VelocityMapper)
from .controllers import DEFAULT_CONTROLLER, controller_names, get_controller


class SafeTeleopNode(Node):

    def __init__(self, **node_kwargs):
        # node_kwargs: rclpy.node.Node options, e.g. parameter_overrides for
        # tests that construct the node without a launch file.
        super().__init__('safe_teleop', **node_kwargs)

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
        # Print the stick reading + published velocity this often (0 = never).
        # teleop.launch.py sets 1 Hz so the pad can be checked before the
        # commander is up.
        self.declare_parameter('print_hz', 0.0)

        self.declare_parameter('max_speed_mps', MAX_SPEED_MPS)        # full right stick
        self.declare_parameter('max_climb_speed_mps', MAX_CLIMB_SPEED_MPS)  # full left stick
        self.declare_parameter('deadzone', DEADZONE)
        # Keep max_speed_mps equal to the commander's teleop_max_speed_mps
        # (see the module docstring). The status topic and namespace mirror
        # the commander's status_topic parameter and node name.
        self.declare_parameter('sync_max_speed', True)
        self.declare_parameter('commander_status_topic', '/svg/commander_status')
        self.declare_parameter('commander_ns', '/swarm_commander')

        # Axis map: defaults come from the controller profile. Set one of
        # these explicitly only to correct a driver that maps differently.
        for key, default in profile.mapping_parameters().items():
            self.declare_parameter(key, default)
        self.declare_parameter('yaw_rate_rad_s', YAW_RATE_RAD_S)

        def value(name):
            return self.get_parameter(name).value

        self.mapper = VelocityMapper(
            direct_vertical=True,
            max_speed=float(value('max_speed_mps')),
            max_climb_speed=float(value('max_climb_speed_mps')),
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

        # ---- max_speed_mps <-> commander teleop_max_speed_mps ---------------
        self.sync_max_speed = bool(value('sync_max_speed'))
        self.commander_ns = str(value('commander_ns')).rstrip('/')
        # Last teleop_max_speed_mps the commander reported (None = no
        # snapshot yet). A push is skipped when the commander already runs
        # with the value, which is what happens after the panel sets both.
        self.commander_max_speed = None
        # Set while a value from the commander is being adopted, so the
        # parameter callback does not push it straight back.
        self._adopting = False
        self._pending_push = None
        self.commander_params = None
        if self.sync_max_speed:
            self.create_subscription(
                String, str(value('commander_status_topic')),
                self.commander_status_callback, 10)
            self.commander_params = self.create_client(
                SetParameters, f'{self.commander_ns}/set_parameters')
        # Registered last: rclpy runs these for every declare_parameter too.
        self.add_on_set_parameters_callback(self.on_parameter_change)
        if hasattr(self, 'add_post_set_parameters_callback'):   # rclpy >= Iron
            self.add_post_set_parameters_callback(self._apply_parameters)
        else:
            self._apply_in_validate = True

        self.last_command = None
        # Axes that read full scale on the first /joy message. An analog
        # trigger rests there, so a map that points a velocity axis at one
        # commands full speed with nothing touched — the usual symptom of the
        # wrong teleop_controller for the pad. None = not checked yet.
        self.suspect_axes = None
        print_hz = float(value('print_hz'))
        if print_hz > 0.0:
            self.create_timer(1.0 / print_hz, self.print_status)

        self.get_logger().info(
            f'safe_teleop driving {drone} with {profile.name} '
            f'({profile.description}): publishing {self.publisher.topic_name}, '
            f'reading {value("joy_topic")} and '
            f'{str(value("odometry_topic_template")).format(name=drone)}')

    # ------------------------------------------------------------------
    # Runtime parameters
    # ------------------------------------------------------------------

    # Parameters applied live (`ros2 param set /safe_teleop ...`): the
    # stick scaling. Everything else (axes, topics, rate) is wiring read
    # once at startup; a runtime set of it is refused with a reason.
    LIVE_PARAMS = {
        'max_speed_mps': 'max_speed',
        'max_climb_speed_mps': 'max_climb_speed',
        'yaw_rate_rad_s': 'yaw_rate',
        'deadzone': 'deadzone_width',
    }
    POSITIVE_PARAMS = ('max_speed_mps', 'max_climb_speed_mps')
    _apply_in_validate = False

    def on_parameter_change(self, params):
        """Pre-set callback: validate the live parameters, refuse the rest."""
        for p in params:
            if p.name not in self.LIVE_PARAMS:
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} is read once at startup; change the '
                           'config and relaunch teleop.launch.py (live: '
                           + ', '.join(self.LIVE_PARAMS) + ')')
            if p.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} must be a number, got {p.type_.name}')
            value = float(p.value)
            if not math.isfinite(value) or (
                    value <= 0.0 if p.name in self.POSITIVE_PARAMS else value < 0.0):
                bound = '> 0' if p.name in self.POSITIVE_PARAMS else '>= 0'
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} must be finite and {bound}, got {p.value}')
        if self._apply_in_validate:
            self._apply_parameters(params)
        return SetParametersResult(successful=True)

    def _apply_parameters(self, params):
        """Post-set callback: the parameter is stored, apply it to the mapper."""
        for p in params:
            if p.name not in self.LIVE_PARAMS:
                continue
            value = float(p.value)
            attr = self.LIVE_PARAMS[p.name]
            old = getattr(self.mapper, attr)
            setattr(self.mapper, attr, value)
            if p.name != 'max_speed_mps':
                if value != old:
                    self.get_logger().info(f'{p.name} {old:g} -> {value:g} (live)')
                continue
            if self._adopting:
                continue        # came from the commander: nothing to push
            if value != old:
                self.get_logger().info(
                    f'max_speed_mps {old:g} -> {value:g} (live; full right stick)')
            self.push_max_speed(value)

    def push_max_speed(self, value: float):
        """Ask the commander to run with the same stick cap as this node."""
        if not self.sync_max_speed:
            return
        if (self.commander_max_speed is not None
                and abs(self.commander_max_speed - value) < 1e-6):
            return          # it already does (the panel sets both sides)
        if not self.commander_params.service_is_ready():
            self.get_logger().warn(
                f'{self.commander_ns}/set_parameters not available: '
                f'teleop_max_speed_mps stays as the commander has it, and '
                f'max_speed_mps will follow the commander ({value:g} is not '
                'pushed) once its status snapshot arrives')
            return
        request = SetParameters.Request()
        request.parameters = [
            Parameter('teleop_max_speed_mps', Parameter.Type.DOUBLE,
                      float(value)).to_parameter_msg()]
        future = self.commander_params.call_async(request)
        self._pending_push = future

        def report(fut):
            if self._pending_push is fut:
                self._pending_push = None
            try:
                result = fut.result().results[0]
            except Exception as error:   # noqa: BLE001 — service error, log it
                self.get_logger().error(
                    f'{self.commander_ns}/set_parameters failed: {error}')
                return
            if result.successful:
                self.get_logger().info(
                    f'commander teleop_max_speed_mps -> {value:g} (pushed from '
                    'max_speed_mps)')
            else:
                self.get_logger().error(
                    f'commander REJECTED teleop_max_speed_mps={value:g}: '
                    f'{result.reason} — max_speed_mps will follow the '
                    "commander's value instead")

        future.add_done_callback(report)

    def commander_status_callback(self, msg: String):
        """Follow the commander's live teleop_max_speed_mps."""
        try:
            value = json.loads(msg.data).get('tuning', {}).get('teleop_max_speed_mps')
        except (ValueError, AttributeError):
            return
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            return
        self.commander_max_speed = value
        if self._pending_push is not None:
            return          # our own push is in flight; the next snapshot decides
        if abs(self.mapper.max_speed - value) < 1e-6:
            return
        old = self.mapper.max_speed
        self._adopting = True
        try:
            results = self.set_parameters(
                [Parameter('max_speed_mps', Parameter.Type.DOUBLE, value)])
        finally:
            self._adopting = False
        if results[0].successful:
            self.get_logger().info(
                f'max_speed_mps {old:g} -> {value:g} (following the '
                f"commander's teleop_max_speed_mps)")
        else:
            self.get_logger().error(
                f"could not follow the commander's teleop_max_speed_mps="
                f'{value:g}: {results[0].reason}')

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def joy_callback(self, msg: Joy):
        self.joy = msg
        self.last_joy_time = self.get_clock().now()
        if self.suspect_axes is None:
            self.check_axis_map(msg)

    def mapped_axes(self) -> dict:
        """The four velocity axes by the name this node uses for them."""
        m = self.mapper
        return {'forward': m.forward_axis, 'left': m.left_axis,
                'climb': m.climb_axis, 'yaw': m.yaw_axis}

    def check_axis_map(self, msg: Joy):
        """First /joy message: is this map plausible for the pad that sent it?

        Two ways a wrong ``teleop_controller`` shows up here. An axis index the
        pad does not have reads as a constant 0.0 — dead, but harmless. An axis
        that rests at full scale is an analog trigger, and pointing a velocity
        axis at one means full speed commanded with nothing touched, so those
        block the command until they move into a plausible range.
        """
        missing = {n: i for n, i in self.mapped_axes().items()
                   if i >= len(msg.axes)}
        if missing:
            self.get_logger().error(
                f'{self.controller.name} expects axes {sorted(self.mapped_axes().values())} '
                f'but this pad reports only {len(msg.axes)}: '
                f'{", ".join(f"{n} (axis {i})" for n, i in missing.items())} '
                'will never move. Wrong teleop_controller for this pad?')
        self.suspect_axes = {n: i for n, i in self.mapped_axes().items()
                             if i < len(msg.axes) and abs(msg.axes[i]) > 0.9}
        if self.suspect_axes:
            self.get_logger().error(
                'REFUSING TO COMMAND: '
                + ', '.join(f'{n} (axis {i}) rests at {msg.axes[i]:+.2f}'
                            for n, i in self.suspect_axes.items())
                + f'. An axis at full scale untouched is an analog trigger, so '
                f'{self.controller.name} is probably the wrong teleop_controller '
                'for this pad — it would command full speed with nothing held. '
                'Check the pad with `ros2 run svg_ground_control joy_map`, then '
                'pick or add the right profile in safe_teleop/controllers.py.')

    def clear_settled_axes(self, msg: Joy):
        """Drop suspects that have come back into range (a stick held at start)."""
        settled = [n for n, i in self.suspect_axes.items()
                   if i < len(msg.axes) and abs(msg.axes[i]) <= 0.9]
        for name in settled:
            del self.suspect_axes[name]
        if settled and not self.suspect_axes:
            self.get_logger().info(
                f'{", ".join(settled)} back in range, commanding again')

    def odometry_callback(self, msg: Odometry):
        self.altitude = msg.pose.pose.position.z
        self.last_odometry_time = self.get_clock().now()

    def _fresh(self, stamp, timeout) -> bool:
        return stamp is not None and (self.get_clock().now() - stamp) < timeout

    def tick(self):
        now = self.get_clock().now()
        dt = 1e-9 * (now - self.last_tick).nanoseconds if self.last_tick else 0.0
        self.last_tick = now

        joy_fresh = self._fresh(self.last_joy_time, self.joy_timeout)
        if self.joy is None or not joy_fresh:
            self.get_logger().warn('joy stale, holding zero velocity',
                                   throttle_duration_sec=2.0)
        if self.suspect_axes:
            self.clear_settled_axes(self.joy)
        if self.suspect_axes:
            self.get_logger().error(
                'axis map looks wrong, holding zero velocity: '
                + ', '.join(f'{n} (axis {i})'
                            for n, i in self.suspect_axes.items()),
                throttle_duration_sec=5.0)
            self.last_command = None
            self.publish(0.0, 0.0, 0.0)
            return

        state = self.pad_state(joy_fresh)

        # Altitude is display-only here (position hold lives in the commander).
        command = self.mapper.update(
            state, dt, self.altitude if self.altitude is not None else 0.0)
        self.last_command = command
        if command.held != self.was_locked:
            self.was_locked = command.held
            self.get_logger().info(
                f'left stick {"locked" if command.held else "released"} '
                '(vertical + yaw ignored while locked)')
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
        if self.suspect_axes:
            self.get_logger().info(
                f'pad: {sticks} | axis map REFUSED -> publishing zero velocity '
                f'({", ".join(f"{n} on axis {i}" for n, i in self.suspect_axes.items())} '
                f'rests at full scale; wrong teleop_controller?)')
            return
        c = self.last_command
        if c is None:
            self.get_logger().info(f'pad: {sticks} | publishing zero velocity')
            return
        alt = (f' | alt {self.altitude:.2f} m' if self.altitude is not None
               else ' | (no odometry yet)')
        self.get_logger().info(
            f'pad: {sticks} | cmd vx {c.vx:+.2f} vy {c.vy:+.2f} vz {c.vz:+.2f} '
            f'yaw {c.yaw_rate:+.2f}{alt}{" | LOCKED" if c.held else ""}')

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
