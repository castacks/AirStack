"""Native MIGHTY -> PX4 setpoint streamer ("Option A").

Streams ``mavros_msgs/PositionTarget`` on ``mavros/setpoint_raw/local`` straight
from ``mighty_node``'s 100 Hz ``dynus_interfaces/msg/Goal``, bypassing the
``trajectory_override -> trajectory_controller -> pid_controller -> attitude``
chain and its 1 Hz / stride decimation. Modelled on the bridge MIT-ACL actually
fly (``jrached/ros2_px4_stack``'s ``dynus_offboard_node.py``), which packs the
same Goal into a MAVROS setpoint stream at the Goal rate.

DEFAULT OFF. ``MIGHTY_NATIVE_SETPOINTS`` (0/1/true/false) gates both this node's
launch and its ``enabled`` parameter, and gates mighty_bridge's engagement
broadcast. With it unset nothing here runs, nothing subscribes to the new
topics, and pid_controller's ``command_muted`` stays false — the flown path is
bit-identical to today's.

Everything about frames, the Goal->PositionTarget mapping, the yaw convention
and the two-masters/handoff ordering lives in ``native_setpoint.py`` (no ROS
imports, unit tested on the host). This file is the ROS shell around it: it
owns the timer, the subscriptions, the two service clients, and the ordering in
which one tick's instructions are carried out.

TOPIC / SERVICE MAP (relative names; the node runs in ``/<robot>/mighty``)
    goal                   in   dynus_interfaces/Goal          from mighty_node
    native_stream_active   in   std_msgs/Bool  (transient local, 1 Hz heartbeat)
                                                               from mighty_bridge
    odometry               in   nav_msgs/Odometry              (handback hold point)
    pid_command            in   mav_msgs/RollPitchYawrateThrust
                                                (handback confirmation only)
    setpoint_raw_local     out  mavros_msgs/PositionTarget     to MAVROS
    pid_set_parameters     srv  rcl_interfaces/SetParameters   the pid muzzle
    set_trajectory_mode    srv  airstack_msgs/TrajectoryMode   handback ROBOT_POSE
"""

import math
import os
import threading

import rclpy
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from airstack_msgs.srv import TrajectoryMode
from dynus_interfaces.msg import Goal
from mav_msgs.msg import RollPitchYawrateThrust
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool

from mighty_bridge.native_setpoint import (
    STATE_HANDBACK, STATE_IDLE, STREAM_GOAL, STREAM_HOLD, YAW_SOURCES,
    HandoffConfig, HandoffStateMachine, assert_position_target_constants,
    env_flag, env_float, frame_is_native, goal_to_setpoint, goal_vectors,
    hold_setpoint)

#: name of the pid_controller parameter that suppresses its command publishes
MUTE_PARAM = 'command_muted'


class NativeSetpointStreamer(Node):
    def __init__(self):
        super().__init__('mighty_native_setpoint')

        # The env var is the master switch (see the module docstring). It is
        # read as the parameter's DEFAULT rather than passed through the launch
        # file as a <param> value, because ROS 2 XML launch type-infers "0" as
        # an int and would then refuse to bind it to a bool parameter.
        self.declare_parameter(
            'enabled', env_flag(os.environ.get('MIGHTY_NATIVE_SETPOINTS')))
        self.declare_parameter('stream_rate_hz', 50.0)
        self.declare_parameter('world_frame', 'map')
        # 'velocity' reproduces the heading convention the vehicle has flown
        # (see native_setpoint.heading_from_velocity and the MIGHTY spline-yaw
        # finding in bridge_node._traj_cb); 'goal' passes Goal.yaw through.
        self.declare_parameter('yaw_source', 'velocity')
        self.declare_parameter('yaw_min_speed', 0.3)
        self.declare_parameter('send_yaw_rate', False)
        self.declare_parameter('engage_timeout_s', 3.0)
        # How long a disengage is treated as provisional. Default from the env
        # so an empty MIGHTY_NATIVE_GRACE_S cannot become a launch-time type
        # error; the launch file passes the same env, which simply agrees.
        self.declare_parameter(
            'disengage_grace_s',
            env_float(os.environ.get('MIGHTY_NATIVE_GRACE_S'), 4.0))
        # Follow live Goals through the grace instead of holding — removes
        # the per-task-swap decel pulse under raven's goal churn. ENV-ONLY,
        # default ON for this stack: passing it as a launch <param> crashed
        # the node at startup (XML type-infers "1" as int, the parameter is
        # bool — the exact footgun documented for `enabled` above; relearned
        # live 2026-09-02 23:14, exit 1 two seconds in).
        self.declare_parameter(
            'grace_follow_goals',
            env_flag(os.environ.get('MIGHTY_NATIVE_GRACE_FOLLOW'), True))
        self.declare_parameter('goal_stale_s', 0.5)
        self.declare_parameter('hold_max_s', 5.0)
        self.declare_parameter('mute_retry_s', 1.0)
        self.declare_parameter('mute_refresh_s', 0.2)
        self.declare_parameter('handback_confirm_msgs', 3)
        self.declare_parameter('handback_timeout_s', 1.0)
        self.declare_parameter('handback_pin_s', 0.3)
        self.declare_parameter('request_robot_pose_on_handback', True)

        self.enabled = bool(self.get_parameter('enabled').value)
        rate = max(2.0, float(self.get_parameter('stream_rate_hz').value))
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.yaw_source = str(self.get_parameter('yaw_source').value)
        if self.yaw_source not in YAW_SOURCES:
            raise ValueError(f'yaw_source must be one of {YAW_SOURCES}')
        self.yaw_min_speed = float(self.get_parameter('yaw_min_speed').value)
        self.send_yaw_rate = bool(self.get_parameter('send_yaw_rate').value)
        self.request_robot_pose = bool(
            self.get_parameter('request_robot_pose_on_handback').value)

        cfg = HandoffConfig(
            enabled=self.enabled,
            engage_timeout_s=float(self.get_parameter('engage_timeout_s').value),
            disengage_grace_s=float(
                self.get_parameter('disengage_grace_s').value),
            grace_follow_goals=bool(
                self.get_parameter('grace_follow_goals').value),
            goal_stale_s=float(self.get_parameter('goal_stale_s').value),
            hold_max_s=float(self.get_parameter('hold_max_s').value),
            mute_retry_s=float(self.get_parameter('mute_retry_s').value),
            mute_refresh_s=float(self.get_parameter('mute_refresh_s').value),
            handback_confirm_msgs=int(
                self.get_parameter('handback_confirm_msgs').value),
            handback_timeout_s=float(
                self.get_parameter('handback_timeout_s').value),
            handback_pin_s=float(
                self.get_parameter('handback_pin_s').value))
        self._sm = HandoffStateMachine(cfg)
        self._sm_lock = threading.Lock()

        # A MAVROS version bump that renumbered a PositionTarget constant would
        # otherwise turn into a silently wrong type_mask in flight.
        assert_position_target_constants(PositionTarget)

        # state
        self._goal = None
        self._goal_t = None
        self._engaged = False
        self._engaged_t = None
        self._odom = None
        self._last_yaw = 0.0
        self._handback_hold = None
        #: (position, yaw) of the last PositionTarget we actually published —
        #: what every non-handback hold freezes on. See _publish_hold.
        self._last_commanded = None
        self._mute_inflight = None       # value of the request in flight
        self._mute_inflight_t = 0.0
        self._mute_retry_s = cfg.mute_retry_s
        self._mode_inflight = False
        self._data_lock = threading.Lock()

        # Subscriptions and the two service clients are reentrant so a future's
        # done-callback can resolve while a tick is running; the TIMER is
        # mutually exclusive so ticks can never overlap each other. Overlapping
        # ticks would race on the handback hold point and the carried yaw, both
        # of which are deliberately un-locked single-writer state.
        cb = ReentrantCallbackGroup()
        timer_cb = MutuallyExclusiveCallbackGroup()

        # mighty_node publishes `goal` with KeepLast(10)/reliable/volatile
        # (its `critical_qos`); reliable + volatile is what makes a subscriber
        # compatible, depth is ours to choose and we only ever want the newest.
        goal_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                              reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.VOLATILE)
        # Engagement is a latched state, not a stream: transient-local so a
        # streamer that starts (or restarts) mid-route learns the current value
        # immediately instead of waiting for the 1 Hz heartbeat.
        engage_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        odom_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                              reliability=QoSReliabilityPolicy.BEST_EFFORT,
                              durability=QoSDurabilityPolicy.VOLATILE)

        self.create_subscription(Goal, 'goal', self._goal_cb, goal_qos,
                                 callback_group=cb)
        self.create_subscription(Bool, 'native_stream_active',
                                 self._engaged_cb, engage_qos,
                                 callback_group=cb)
        self.create_subscription(Odometry, 'odometry', self._odom_cb, odom_qos,
                                 callback_group=cb)
        if cfg.handback_confirm_msgs > 0:
            self.create_subscription(RollPitchYawrateThrust, 'pid_command',
                                     self._pid_cmd_cb, 1, callback_group=cb)

        # Match mavros_interface's own PositionTarget publisher (depth 1,
        # reliable) — that wiring is known to work against this MAVROS.
        self.setpoint_pub = self.create_publisher(
            PositionTarget, 'setpoint_raw_local', 1)

        self._pid_param_client = self.create_client(
            SetParameters, 'pid_set_parameters', callback_group=cb)
        self._mode_client = self.create_client(
            TrajectoryMode, 'set_trajectory_mode', callback_group=cb)

        self.create_timer(1.0 / rate, self._tick, callback_group=timer_cb)

        self.get_logger().info(
            f'mighty_native_setpoint up: enabled={self.enabled}, '
            f'{rate:.0f} Hz, world_frame={self.world_frame}, '
            f'yaw_source={self.yaw_source}, send_yaw_rate={self.send_yaw_rate}, '
            f'disengage_grace={cfg.disengage_grace_s:.1f} s'
            + ('' if self.enabled else
               ' (INERT — set MIGHTY_NATIVE_SETPOINTS=1 to arm this path)'))

    # ------------------------------------------------------------------
    # clock / callbacks
    # ------------------------------------------------------------------

    def _now(self):
        """Seconds on the NODE's clock — sim time when use_sim_time is on.

        Everything time-based here (goal staleness, the engagement heartbeat,
        the mute deadman) must share one time base with pid_controller's
        ``command_mute_timeout``, which is measured on its own ROS clock.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def _goal_cb(self, msg):
        frame = (msg.header.frame_id or '').strip()
        if not frame_is_native(frame, self.world_frame):
            # Same discipline as bridge_node._poses_in_world_frame: refuse to
            # fly coordinates whose frame we have not accounted for. A Goal in
            # some other frame would be flown as if it were map — silently, and
            # into whatever is at those coordinates.
            self.get_logger().error(
                f'MIGHTY Goal stamped {frame!r}, expected {self.world_frame!r} '
                f'(mighty_node stamps par_.map_frame_id). Refusing to stream '
                f'it; the legacy pid path keeps the vehicle.',
                throttle_duration_sec=5.0)
            return
        with self._data_lock:
            self._goal = msg
            self._goal_t = self._now()

    def _engaged_cb(self, msg):
        with self._data_lock:
            was = self._engaged
            self._engaged = bool(msg.data)
            self._engaged_t = self._now()
        if was != bool(msg.data):
            self.get_logger().info(
                f'mighty_bridge engagement -> {bool(msg.data)}')

    def _odom_cb(self, msg):
        with self._data_lock:
            self._odom = msg

    def _pid_cmd_cb(self, _msg):
        with self._sm_lock:
            self._sm.note_pid_command()

    # ------------------------------------------------------------------
    # the tick
    # ------------------------------------------------------------------

    def _tick(self):
        now = self._now()
        with self._data_lock:
            goal, goal_t = self._goal, self._goal_t
            engaged, engaged_t = self._engaged, self._engaged_t
            odom = self._odom
        with self._sm_lock:
            action = self._sm.tick(
                now,
                engagement_age=None if engaged_t is None else now - engaged_t,
                engaged=engaged,
                goal_age=None if goal_t is None else now - goal_t)
            two_masters = self._sm.pid_commands_while_streaming
        if action.note:
            self.get_logger().info(
                f'native setpoints: {action.state} ({action.note})')
        if two_masters:
            with self._sm_lock:
                self._sm.pid_commands_while_streaming = 0
            self.get_logger().error(
                f'{two_masters} pid attitude command(s) observed while we were '
                f'streaming PositionTargets — the mute did not take and PX4 '
                f'has two masters', throttle_duration_sec=1.0)

        # ORDER OF OPERATIONS, and it is load-bearing (see
        # HandoffStateMachine._handback):
        #   1. publish the setpoint FIRST, so a service call in flight below
        #      never opens a gap in the stream PX4's OFFBOARD watchdog counts;
        #   2. THEN the trajectory-controller ROBOT_POSE request, so the pid's
        #      target is already "hold here" before it is allowed to speak;
        #   3. THEN the mute/un-mute.
        # On ENGAGE the state machine gives stream=None + mute=True in the same
        # tick, so this same order yields mute-then-stream (the mute is
        # confirmed before the first setpoint) — the two orderings do not
        # conflict, they fall out of one rule.
        if action.stream == STREAM_GOAL and goal is not None:
            self._publish_goal(goal)
        elif action.stream == STREAM_HOLD:
            self._publish_hold(action.state, goal, odom)
        if action.state != STATE_HANDBACK:
            self._handback_hold = None

        if action.request_robot_pose:
            if self.request_robot_pose:
                self._request_trajectory_mode(TrajectoryMode.Request.ROBOT_POSE)
            else:
                # Disabled by parameter: ack immediately so the handback does
                # not stall for handback_pin_s waiting on a call never made.
                with self._sm_lock:
                    self._sm.note_robot_pose_result(True)
        if action.request_mute is not None:
            self._request_mute(now, action.request_mute)

    # ------------------------------------------------------------------
    # publishing
    # ------------------------------------------------------------------

    def _publish_goal(self, goal):
        p, v, a, gyaw, gdyaw = goal_vectors(goal)
        fields = goal_to_setpoint(
            p, v, a, gyaw, gdyaw,
            yaw_source=self.yaw_source, prev_yaw=self._last_yaw,
            yaw_min_speed=self.yaw_min_speed,
            send_yaw_rate=self.send_yaw_rate)
        self._last_yaw = fields.yaw
        self._publish(fields)

    def _publish_hold(self, state, goal, odom):
        """Hold: 'stay exactly here'. WHICH 'here' depends on why we are holding.

        * handback -> the vehicle's own position, LATCHED ONCE on entry. Not
          re-read per tick: chasing live odometry is not a hold, it is a free
          drift, and it would also disagree with the trajectory controller's
          ROBOT_POSE target that the pid is about to start tracking.
        * every other hold (stale goals, and the whole disengage GRACE window)
          -> THE LAST SETPOINT WE ACTUALLY SENT. Not the newest Goal: during
          grace mighty_node keeps publishing Goals whether or not a task is
          engaged, so holding "at the latest goal" would silently keep flying
          MIGHTY's plan for a route the task layer has already finished. The
          last commanded point is fixed by construction — re-publishing it
          leaves it unchanged.
        """
        if state == STATE_HANDBACK:
            if self._handback_hold is None:
                point = self._vehicle_hold_point(goal, odom)
                if point[0] is None:
                    # Nothing to hold at yet. Do NOT latch the failure — odom
                    # may still arrive within the handback window, and a
                    # latched None would keep us silent for all of it.
                    return
                self._handback_hold = point
            pos, yaw = self._handback_hold
        elif self._last_commanded is not None:
            pos, yaw = self._last_commanded
        else:
            # Asked to hold before we ever streamed anything.
            pos, yaw = self._vehicle_hold_point(goal, odom)
            if pos is None:
                return
        self._publish(hold_setpoint(pos, yaw, send_yaw_rate=self.send_yaw_rate))

    def _vehicle_hold_point(self, goal, odom):
        """Where the handback hold sits: the vehicle, else the last commanded
        goal, else nothing (``(None, 0.0)``) — the caller retries."""
        if odom is not None:
            p = odom.pose.pose.position
            q = odom.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return ((p.x, p.y, p.z), yaw)
        if goal is not None:
            return ((goal.p.x, goal.p.y, goal.p.z), self._last_yaw)
        self.get_logger().error(
            'handback with neither odometry nor a MIGHTY goal to hold at; '
            'nothing to stream during the overlap — the pid resumes on its own',
            throttle_duration_sec=1.0)
        return (None, 0.0)

    def _publish(self, fields):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        # MAVROS reads coordinate_frame, not header.frame_id, but stamping the
        # frame we believe these numbers are in keeps a bag readable.
        msg.header.frame_id = self.world_frame
        msg.coordinate_frame = fields.coordinate_frame
        msg.type_mask = fields.type_mask
        msg.position.x, msg.position.y, msg.position.z = fields.position
        msg.velocity.x, msg.velocity.y, msg.velocity.z = fields.velocity
        (msg.acceleration_or_force.x, msg.acceleration_or_force.y,
         msg.acceleration_or_force.z) = fields.acceleration
        msg.yaw = fields.yaw
        msg.yaw_rate = fields.yaw_rate
        self.setpoint_pub.publish(msg)
        self._last_commanded = (fields.position, fields.yaw)

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------

    def _request_mute(self, now, value):
        """Set pid_controller's ``command_muted``.

        Non-blocking on purpose: ``wait_for_service`` inside a 50 Hz timer
        would stall the stream. A service that is not up yet reports FAILURE
        straight back to the state machine, which keeps us out of STREAMING —
        no pid, no mute, no native stream, and the vehicle stays on whatever
        path it was already flying.
        """
        # De-duplicate only an IDENTICAL request that is both in flight and
        # still young. Two earlier shapes of this guard were wrong in a way
        # that matters: a plain "one call at a time" flag let a wedged mute
        # REFRESH swallow the un-mute behind it, leaving the pid muted through
        # the whole handback (caught by test_a_dead_bridge_gives_the_vehicle_
        # back); dropping the guard entirely piles up a call per tick at 50 Hz.
        if (self._mute_inflight == value
                and now - self._mute_inflight_t < self._mute_retry_s):
            return
        if not self._pid_param_client.service_is_ready():
            self.get_logger().error(
                f'pid parameter service not available '
                f'({self._pid_param_client.srv_name}); cannot {"" if value else "un"}'
                f'mute the pid, staying on the legacy path',
                throttle_duration_sec=5.0)
            with self._sm_lock:
                self._sm.note_mute_result(now, value, False)
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name=MUTE_PARAM,
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                 bool_value=bool(value)))]
        self._mute_inflight = bool(value)
        self._mute_inflight_t = now
        fut = self._pid_param_client.call_async(req)

        def _done(f, value=value):
            if self._mute_inflight == bool(value):
                self._mute_inflight = None
            try:
                ok = all(r.successful for r in f.result().results)
            except Exception as exc:               # noqa: BLE001
                ok = False
                self.get_logger().error(f'pid {MUTE_PARAM} call failed: {exc}')
            if not ok:
                self.get_logger().error(
                    f'pid {MUTE_PARAM} := {value} REFUSED')
            with self._sm_lock:
                self._sm.note_mute_result(self._now(), value, ok)
        fut.add_done_callback(_done)

    def _request_trajectory_mode(self, mode):
        if self._mode_inflight:
            return
        if not self._mode_client.service_is_ready():
            self.get_logger().warn(
                f'set_trajectory_mode not available ({self._mode_client.srv_name}); '
                f'handing back without pinning the tracking point',
                throttle_duration_sec=5.0)
            with self._sm_lock:
                self._sm.note_robot_pose_result(False)
            return
        req = TrajectoryMode.Request()
        req.mode = mode
        self._mode_inflight = True
        fut = self._mode_client.call_async(req)

        def _done(_f):
            self._mode_inflight = False
            # The ANSWER is what the handback waits on, not its content: it
            # proves the controller ordered our ROBOT_POSE after the TRACK
            # mighty_bridge asks for at the same instant.
            with self._sm_lock:
                self._sm.note_robot_pose_result(True)
        fut.add_done_callback(_done)

    def unmute_on_shutdown(self, timeout_s=0.5):
        """Best-effort un-mute on the way out.

        pid_controller's own watchdog already recovers from us dying
        (``command_mute_timeout``), so this is belt-and-braces for the ORDERLY
        exit — a Ctrl-C should not cost the vehicle half a second of nobody
        commanding it. Deliberately synchronous and short: we are past the
        executor here.
        """
        with self._sm_lock:
            needs_unmute = self._sm.state != STATE_IDLE
        if not needs_unmute or not self._pid_param_client.service_is_ready():
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name=MUTE_PARAM,
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                 bool_value=False))]
        try:
            fut = self._pid_param_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
            self.get_logger().warn('shutting down: pid un-muted')
        except Exception as exc:                   # noqa: BLE001
            self.get_logger().error(f'shutdown un-mute failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = NativeSetpointStreamer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.unmute_on_shutdown()
        except Exception:                          # noqa: BLE001
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:                          # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
