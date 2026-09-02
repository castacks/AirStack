"""Bridge between the MIGHTY planner and AirStack's local-planner seam.

Responsibilities (one node so the module adds a single process):

- odometry (nav_msgs/Odometry) -> dynus_interfaces/State on ``state``,
  rotating the body-frame twist into the world frame (nav_msgs convention
  puts twist in the child frame; MIGHTY expects world-frame velocity).
- MIGHTY's committed ``dynus_interfaces/Trajectory`` -> decimated
  airstack_msgs/TrajectoryXYZVYaw on ``trajectory_override`` (throttled
  receding-horizon replacement; with vehicle-synchronized plan consumption
  every committed trajectory starts at the vehicle, so a clear-and-set
  override avoids the ADD_SEGMENT merge/virtual_time bookkeeping whose
  splice rejections produced silent hover deadlocks).
- NavigateTask action server (``~/navigate_task``): walks the goal path's
  poses as successive ``term_goal`` checkpoints for MIGHTY, mirroring the
  droan_gl task contract (ADD_SEGMENT while navigating, TRACK on exit).
- (opt-in, ``MIGHTY_NATIVE_SETPOINTS``) broadcasts on ``native_stream_active``
  whether the follower / NavigateTask currently owns the vehicle, which is
  what tells ``native_setpoint_node`` when it may take PX4 over directly.
  See README.md, "Two command paths".
"""

import math
import os
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from airstack_msgs.srv import TrajectoryMode
from dynus_interfaces.msg import State, Trajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from task_msgs.action import NavigateTask

from mighty_bridge.native_setpoint import env_flag, heading_from_velocity

# AirStack patch (ORIGIN.md): goals and global plans may arrive stamped in a
# frame other than the robot's map (e.g. 'world', which the map_anchor node
# relates to 'map' by the takeoff offset). droan_gl's global_plan transformed
# them through TF; the bridge used to take the coordinates literally.
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


# heading_from_velocity lives in native_setpoint.py now (imported above,
# body unchanged): both command paths — this one's trajectory_override and the
# native PositionTarget streamer — must point the nose the same way, so there
# is exactly one copy of the rule. Re-exported here so existing importers of
# `mighty_bridge.bridge_node.heading_from_velocity` keep working.


def capped_speed(v, cap):
    """Waypoint speed under the goal's cap; a cap of 0 means none.

    NOT applied to the trajectories the bridge forwards (see _traj_cb):
    MIGHTY anchors every replan on its own committed trajectory, so a vehicle
    paced slower than that trajectory drifts behind the anchor, the start
    guard trips and nothing is forwarded (measured 2026-08-28). Until MIGHTY
    takes a live v_max the cap is recorded and logged only; MIGHTY's own
    v_max is the speed."""
    return min(v, cap) if cap > 0.0 else v


def quat_rotate(qx, qy, qz, qw, vx, vy, vz):
    """Rotate vector v by quaternion q (Hamilton, w last in args)."""
    # t = 2 * cross(q_vec, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    # v' = v + w * t + cross(q_vec, t)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


class MightyBridge(Node):
    def __init__(self):
        super().__init__('mighty_bridge')

        self.declare_parameter('waypoint_tolerance_m', 2.0)
        self.declare_parameter('term_goal_republish_s', 5.0)
        self.declare_parameter('segment_stride', 5)  # 0.05 s spacing: denser segments track corners tighter
        self.declare_parameter('override_period_s', 1.0)  # receding-horizon trajectory replacement rate
        self.declare_parameter('twist_in_body_frame', True)
        self.declare_parameter('world_frame', 'map')
        # global_plan follower (study R5-R7 contract): the route planner
        # publishes a nav_msgs/Path on global_plan; once the vehicle has
        # climbed follow_min_climb_m and settled, the bridge walks that
        # path's poses as term_goal checkpoints exactly like a NavigateTask.
        self.declare_parameter('follow_global_plan', True)
        self.declare_parameter('follow_min_climb_m', 8.0)
        self.declare_parameter('follow_settle_s', 3.0)
        # > mighty's goal_seen_radius (5.0) so the moving carrot never puts
        # the planner into GOAL_SEEN/GOAL_REACHED before the true route end
        self.declare_parameter('follow_lookahead_m', 8.0)
        # Native MIGHTY->PX4 setpoint path (Option A, default OFF): broadcast
        # whether the follower / NavigateTask currently owns the vehicle, so
        # native_setpoint_node knows when it may stream PositionTargets and
        # when it must hand control back to the pid. Purely additive — with
        # the switch off no publisher and no timer are created at all, so this
        # node's graph is exactly what it was.
        # Default comes from the env var rather than a launch <param> because
        # ROS 2 XML launch type-infers "0" as an int, which cannot bind to a
        # bool parameter.
        self.declare_parameter(
            'publish_native_stream_active',
            env_flag(os.environ.get('MIGHTY_NATIVE_SETPOINTS')))
        self.declare_parameter('native_stream_heartbeat_s', 1.0)

        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance_m').value)
        self.republish_s = float(self.get_parameter('term_goal_republish_s').value)
        self.segment_stride = max(1, int(self.get_parameter('segment_stride').value))
        self.override_period = float(self.get_parameter('override_period_s').value)
        self.twist_in_body_frame = bool(self.get_parameter('twist_in_body_frame').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.follow_enabled = bool(self.get_parameter('follow_global_plan').value)
        self.follow_min_climb = float(self.get_parameter('follow_min_climb_m').value)
        self.follow_settle_s = float(self.get_parameter('follow_settle_s').value)
        self.follow_lookahead = float(self.get_parameter('follow_lookahead_m').value)
        self.publish_native_active = bool(
            self.get_parameter('publish_native_stream_active').value)
        self.native_heartbeat_s = float(
            self.get_parameter('native_stream_heartbeat_s').value)
        self._native_active_pub = None

        self._lock = threading.Lock()
        self._odom = None            # latest nav_msgs/Odometry
        self._last_traj_time = 0.0   # wall time of last MIGHTY trajectory
        self._task_active = False
        self._cancel_requested = False
        self._route_active = False   # a NavigateTask or follower route is executing
        # follower state
        self._z0 = None
        self._settled_since = None
        self._airborne = False
        self._follow_plan = None     # list of PoseStamped adopted from global_plan
        self._follow_thread = None
        self._follow_done_plan = None
        self._traj_end = None        # last committed MIGHTY trajectory endpoint
        self._pending_traj = None    # conflated trajectory awaiting the throttle window
        # AirStack patch (ORIGIN.md): the speed cap a NavigateTask goal carries
        # in `max_speed_mps` (0 = none). Applied to every waypoint velocity the
        # bridge hands the trajectory controller, so the controller paces the
        # same MIGHTY path slower; MIGHTY itself keeps planning at its v_max.
        self._speed_cap = 0.0
        self._last_yaw = 0.0         # heading carried between forwarded trajectories
        # The speed MIGHTY is currently planning at: its configured v_max
        # until a cap is pushed into it (below). The start guard scales with
        # it — MIGHTY anchors each replan ~0.6 s ahead on its committed
        # trajectory, so at 7 m/s the honest lead is ~4.5 m.
        self.declare_parameter('mighty_v_max_default', 1.5)
        # OFF by default (2026-08-28): pushing v_max into mighty_node's
        # set_parameters stalled the planner under a 5-robot load (the call
        # never returned, no replan ever ran). MIGHTY flies its configured
        # v_max; the NavigateTask cap is logged only.
        self.declare_parameter('push_vmax_to_mighty', False)
        self._push_vmax_enabled = bool(self.get_parameter('push_vmax_to_mighty').value)
        self.declare_parameter('start_guard_min_m', 4.0)
        self.declare_parameter('start_guard_s', 1.0)
        self._v_max_default = float(self.get_parameter('mighty_v_max_default').value)
        self._v_max_effective = self._v_max_default
        self._guard_min = float(self.get_parameter('start_guard_min_m').value)
        self._guard_s = float(self.get_parameter('start_guard_s').value)
        # MIGHTY's `v_max` is a live parameter (AirStack patch in mighty_node):
        # relative name, so it resolves beside this node in either launch.
        self._vmax_client = self.create_client(SetParameters, 'mighty_node/set_parameters')

        cb = ReentrantCallbackGroup()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        latest_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)

        # odometry in -> state out
        self.state_pub = self.create_publisher(State, 'state', state_qos)
        self.create_subscription(Odometry, 'odometry', self._odom_cb, latest_qos,
                                 callback_group=cb)

        # MIGHTY committed trajectory in -> controller trajectory replacement.
        # With vehicle-synchronized plan consumption every committed trajectory
        # STARTS at the vehicle, so a throttled trajectory_override (clear +
        # set, virtual_time := 0) is a clean receding-horizon handoff — no
        # ADD_SEGMENT merge bookkeeping (whose virtual_time splice-rejection
        # produced silent hover deadlocks in three distinct variants).
        self.segment_pub = self.create_publisher(
            TrajectoryXYZVYaw, 'trajectory_override', 1)
        self.create_subscription(Trajectory, 'mighty_trajectory', self._traj_cb,
                                 reliable_qos, callback_group=cb)

        # term_goal out to MIGHTY
        self.term_goal_pub = self.create_publisher(PoseStamped, 'term_goal', reliable_qos)

        # trajectory controller mode client
        self.mode_client = self.create_client(TrajectoryMode, 'set_trajectory_mode',
                                              callback_group=cb)

        self._action_server = ActionServer(
            self, NavigateTask, '~/navigate_task',
            execute_callback=self._execute_navigate,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            callback_group=cb)

        # conflation flush for the receding-horizon throttle
        self.create_timer(0.2, self._flush_pending_traj, callback_group=cb)

        # Native setpoint path engagement broadcast (see the parameter's
        # comment). TRANSIENT_LOCAL so a streamer that starts, or restarts,
        # mid-route learns the current state at once; the heartbeat is the
        # other half of the contract — the streamer treats "no engagement
        # message for engage_timeout_s" as disengaged, which is what makes a
        # dead bridge give the vehicle back instead of freezing the mute.
        if self.publish_native_active:
            native_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self._native_active_pub = self.create_publisher(
                Bool, 'native_stream_active', native_qos)
            self._publish_native_active(False)
            self.create_timer(self.native_heartbeat_s,
                              self._native_active_heartbeat,
                              callback_group=cb)

        # global_plan follower (see class docstring)
        if self.follow_enabled:
            from nav_msgs.msg import Path
            self.create_subscription(Path, 'global_plan', self._global_plan_cb,
                                     reliable_qos, callback_group=cb)
            self.create_timer(1.0, self._follow_tick, callback_group=cb)

        self.get_logger().info(
            f'mighty_bridge up (waypoint_tolerance={self.waypoint_tolerance} m, '
            f'stride={self.segment_stride}, world_frame={self.world_frame}, '
            f'follow_global_plan={self.follow_enabled}, '
            f'native_setpoint_broadcast={self.publish_native_active})')

    # ------------------------------------------------------------------
    # engagement (native setpoint path)
    # ------------------------------------------------------------------

    def _set_route_active(self, active):
        """THE single locus for the "MIGHTY owns the vehicle" flag.

        `_route_active` gates the trajectory start guard and the yaw carry-over
        in this node; with the native setpoint path armed it additionally tells
        native_setpoint_node when it may stream PositionTargets to PX4. Every
        assignment goes through here so the two can never disagree.

        Must NOT be called while holding `self._lock` — it takes it.
        """
        with self._lock:
            changed = self._route_active != bool(active)
            self._route_active = bool(active)
        if changed and self._native_active_pub is not None:
            self._publish_native_active(bool(active))

    def _publish_native_active(self, active):
        msg = Bool()
        msg.data = bool(active)
        self._native_active_pub.publish(msg)

    def _native_active_heartbeat(self):
        with self._lock:
            active = self._route_active
        self._publish_native_active(active)

    # ------------------------------------------------------------------
    # conversions
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom = msg

        # takeoff-settle detection for the follower (mirrors the mission-glue
        # trigger: climbed follow_min_climb_m and vertical speed ~0 for
        # follow_settle_s)
        if self.follow_enabled and not self._airborne:
            z = msg.pose.pose.position.z
            vz = msg.twist.twist.linear.z
            # AirStack debug: the gate that decides whether the follower ever
            # engages, once per 5 s until it does.
            self.get_logger().info(
                f'follower gate: z0={self._z0} z={z:.2f} climb_needed='
                f'{self.follow_min_climb} vz={vz:.2f} settled_for='
                f'{(time.monotonic() - self._settled_since) if self._settled_since else 0:.1f}s',
                throttle_duration_sec=5.0)
            if self._z0 is None:
                self._z0 = z
            elif z - self._z0 > self.follow_min_climb and abs(vz) < 0.2:
                if self._settled_since is None:
                    self._settled_since = time.monotonic()
                elif time.monotonic() - self._settled_since > self.follow_settle_s:
                    self._airborne = True
                    self.get_logger().info('follower: takeoff settled')
            else:
                self._settled_since = None

        if not self._route_active:
            q = msg.pose.pose.orientation
            with self._lock:
                self._last_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        s = State()
        s.header = msg.header
        s.pos.x = msg.pose.pose.position.x
        s.pos.y = msg.pose.pose.position.y
        s.pos.z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        s.quat = q
        v = msg.twist.twist.linear
        if self.twist_in_body_frame:
            wx, wy, wz = quat_rotate(q.x, q.y, q.z, q.w, v.x, v.y, v.z)
        else:
            wx, wy, wz = v.x, v.y, v.z
        s.vel.x, s.vel.y, s.vel.z = wx, wy, wz
        self.state_pub.publish(s)

    def _traj_cb(self, msg: Trajectory):
        n = len(msg.goals)
        if n == 0:
            return
        now = time.monotonic()
        g_end = msg.goals[-1]
        with self._lock:
            last_fwd = getattr(self, '_last_forward_time', 0.0)
            self._last_traj_time = now
            route_active = self._route_active
            # committed-trajectory end (the follower's catch-up gate compares
            # the vehicle to this)
            self._traj_end = (g_end.p.x, g_end.p.y, g_end.p.z)
        # Never forward a trajectory whose START is far from the vehicle: the
        # override re-anchors the tracking point there and the PID would fly
        # an uncommanded straight line through unswept space (observed:
        # pillar penetration). With vehicle-anchored replanning this is
        # always transient — drop and wait.
        s0 = msg.goals[0]
        d_start = self._distance_to_xyz(s0.p.x, s0.p.y, s0.p.z)
        with self._lock:
            guard = max(self._guard_min, self._guard_s * self._v_max_effective)
        if route_active and d_start is not None and d_start > guard:
            self.get_logger().warn(
                f'dropping trajectory starting {d_start:.1f} m from the vehicle '
                f'(guard {guard:.1f} m at v_max {self._v_max_effective:.1f})')
            return
        # CONFLATING receding-horizon throttle: overriding at every replan
        # (30-50 Hz) would keep re-anchoring the tracking point, but a
        # dropping throttle loses the LAST trajectory of a burst — and MIGHTY
        # stops replanning at GOAL_SEEN once its committed plan reaches the
        # goal, so a dropped final trajectory parked the vehicle 8.6 m short.
        # Buffer the newest; the flush timer publishes it when the period
        # allows.
        if now - last_fwd < self.override_period:
            with self._lock:
                self._pending_traj = msg
            return
        with self._lock:
            self._last_forward_time = now
            self._pending_traj = None

        out = TrajectoryXYZVYaw()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id or self.world_frame

        idxs = list(range(0, n, self.segment_stride))
        if idxs[-1] != n - 1:
            idxs.append(n - 1)
        with self._lock:
            cap = self._speed_cap
            prev_yaw = self._last_yaw
        for i in idxs:
            g = msg.goals[i]
            wp = WaypointXYZVYaw()
            wp.position.x = g.p.x
            wp.position.y = g.p.y
            wp.position.z = g.p.z
            # MIGHTY's own timing, uncapped — see capped_speed() for why.
            wp.velocity = math.sqrt(g.v.x ** 2 + g.v.y ** 2 + g.v.z ** 2)
            # AirStack patch (ORIGIN.md): YAW FROM THE VELOCITY DIRECTION, the
            # convention droan_gl's global_plan used. MIGHTY's committed
            # trajectory carries its spline "yaw" (Sec. III of the DYNUS
            # paper, NOT the body Euler yaw) which was 0 for the whole first
            # flight — the tracking point held heading 0 and the drone flew
            # sideways. Hold the last heading while (nearly) stationary so it
            # does not snap to 0 at the end of a leg.
            prev_yaw = heading_from_velocity(g.v.x, g.v.y, prev_yaw)
            wp.yaw = prev_yaw
            wp.acceleration.x = g.a.x
            wp.acceleration.y = g.a.y
            wp.acceleration.z = g.a.z
            wp.jerk.x = g.j.x
            wp.jerk.y = g.j.y
            wp.jerk.z = g.j.z
            out.waypoints.append(wp)
        with self._lock:
            self._last_yaw = prev_yaw
        self.segment_pub.publish(out)
        w0, w1 = out.waypoints[0], out.waypoints[-1]
        self.get_logger().info(
            f'override -> controller: {len(out.waypoints)} wps, starts '
            f'{d_start if d_start is not None else -1:.1f} m from vehicle at '
            f'({w0.position.x:.1f},{w0.position.y:.1f},{w0.position.z:.1f}) v{w0.velocity:.1f}, '
            f'ends ({w1.position.x:.1f},{w1.position.y:.1f},{w1.position.z:.1f}), cap {cap:.1f}',
            throttle_duration_sec=5.0)

    def _flush_pending_traj(self):
        with self._lock:
            msg = self._pending_traj
            last_fwd = getattr(self, '_last_forward_time', 0.0)
        if msg is None:
            return
        if time.monotonic() - last_fwd < self.override_period:
            return
        with self._lock:
            self._pending_traj = None
        self._traj_cb(msg)

    # ------------------------------------------------------------------
    # global_plan follower (study route contract): the route planner
    # publishes a DENSE nav_msgs/Path on global_plan, re-anchored at the
    # vehicle and republished continuously. Once airborne, the bridge
    # pure-pursuits it: term_goal = the path point a lookahead distance
    # ahead of the vehicle's projection onto the path, sliding to the
    # path end. The moving carrot keeps MIGHTY in TRAVELING (no
    # GOAL_REACHED idles mid-route), so the controller timeline extends
    # continuously and no per-leg mode resets are needed.
    # ------------------------------------------------------------------

    def _poses_in_world_frame(self, poses, what):
        """Return `poses` expressed in world_frame, transforming through TF
        when their header names another frame. None if a transform is
        needed but unavailable — the caller must NOT fly the raw numbers."""
        out = []
        for ps in poses:
            src = (ps.header.frame_id or '').strip()
            if src in ('', self.world_frame):
                out.append(ps)
                continue
            try:
                tf = self._tf_buffer.lookup_transform(
                    self.world_frame, src, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))
            except Exception as exc:
                self.get_logger().error(
                    f'{what}: pose in frame {src!r} but no TF {src} -> '
                    f'{self.world_frame} ({exc}); refusing to fly it as-is')
                return None
            q = PoseStamped()
            q.header.frame_id = self.world_frame
            q.header.stamp = ps.header.stamp
            q.pose = do_transform_pose(ps.pose, tf)
            out.append(q)
        if poses and out and (poses[0].header.frame_id or '') not in ('', self.world_frame):
            a, b = poses[0].pose.position, out[0].pose.position
            self.get_logger().info(
                f'{what}: transformed {len(out)} pose(s) {poses[0].header.frame_id} -> '
                f'{self.world_frame}: ({a.x:.1f},{a.y:.1f},{a.z:.1f}) -> '
                f'({b.x:.1f},{b.y:.1f},{b.z:.1f})', throttle_duration_sec=5.0)
        return out

    def _global_plan_cb(self, msg):
        if not msg.poses:
            return
        poses = self._poses_in_world_frame(list(msg.poses), 'global_plan')
        if poses is None:
            return
        with self._lock:
            first = self._follow_plan is None
            self._follow_plan = poses
        if first:
            self.get_logger().info(
                f'follower: adopted global_plan ({len(poses)} poses)')

    def _follow_tick(self):
        if self._task_active or not self._airborne:
            return
        with self._lock:
            plan = self._follow_plan
        if plan is None:
            return
        final = plan[-1].pose.position
        done = self._follow_done_plan
        if done is not None:
            dx = final.x - done[0]
            dy = final.y - done[1]
            dz = final.z - done[2]
            if (dx * dx + dy * dy + dz * dz) ** 0.5 < 2.0:
                return  # this route was already completed
        if self._follow_thread is not None and self._follow_thread.is_alive():
            return
        self._follow_thread = threading.Thread(
            target=self._follow_route, daemon=True)
        self._follow_thread.start()

    def _carrot(self, poses):
        """Path point ~follow_lookahead_m beyond the vehicle's projection.

        The walk CLAMPS at sharp path-direction reversals (route
        checkpoints on out-and-back legs): a lookahead measured purely
        along the path wraps around hairpins and can land back on the
        vehicle (observed: stable hover deadlock 3.8 m short of a
        checkpoint). Clamping makes the vehicle actually reach the
        corner — which is also what lets the route planner's own
        arrival radius trigger and drop the checkpoint from the path.
        The clamp releases once the vehicle is within 1.5 m of the
        corner (the walk then continues with a fresh direction
        reference, covering the ~1 s of stale path before the planner
        republishes without the reached checkpoint).
        """
        with self._lock:
            odom = self._odom
        if odom is None:
            return None
        p = odom.pose.pose.position
        # nearest pose index (paths from the study planner are anchored at
        # the vehicle, so this is usually 0; static paths also work)
        best_i, best_d = 0, float('inf')
        for i, ps in enumerate(poses):
            q = ps.pose.position
            d = (q.x - p.x) ** 2 + (q.y - p.y) ** 2 + (q.z - p.z) ** 2
            if d < best_d:
                best_d, best_i = d, i

        def seg_dir(a, b):
            v = (b.x - a.x, b.y - a.y, b.z - a.z)
            n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
            return (v[0] / n, v[1] / n, v[2] / n) if n > 1e-6 else None

        acc = 0.0
        ref_dir = None
        prev = poses[best_i].pose.position
        for ps in poses[best_i + 1:]:
            q = ps.pose.position
            d = seg_dir(prev, q)
            step = math.dist((prev.x, prev.y, prev.z), (q.x, q.y, q.z))
            prev = q
            acc += step
            if d is not None:
                if ref_dir is None:
                    ref_dir = d
                else:
                    dot = (ref_dir[0] * d[0] + ref_dir[1] * d[1]
                           + ref_dir[2] * d[2])
                    if dot < -0.17:  # direction reversed > ~100 deg
                        near = math.dist((p.x, p.y, p.z), (q.x, q.y, q.z))
                        if near > 1.5:
                            return ps, False  # clamp at the corner until reached
                        ref_dir = d    # corner reached: release, walk on
                        acc = 0.0
            if acc >= self.follow_lookahead:
                return ps, False
        # the walk exhausted the path: the carrot IS the route end
        return poses[-1], True

    def _follow_route(self):
        self.get_logger().info('follower: engaging (lookahead '
                               f'{self.follow_lookahead} m)')
        # One-time controller timeline reset (duplicates — and therefore
        # does not require — the reference mission glue's mode switch).
        self._set_route_active(True)
        # AirStack patch (ORIGIN.md): the takeoff task leaves the trajectory
        # controller in ROBOT_POSE (tracking point pinned to the vehicle,
        # zero velocity), where an override is stored but never walked —
        # measured 2026-08-28: overrides at 1 Hz, tracking-point velocity
        # 0.0, drone parked. TRACK is the mode that follows the timeline.
        self._set_mode(TrajectoryMode.Request.TRACK)
        self.get_logger().info('follower: trajectory controller -> TRACK')

        last_goal = None
        last_pub = 0.0
        while rclpy.ok():
            if self._task_active:
                self.get_logger().info('follower: yielding to NavigateTask')
                self._set_route_active(False)
                return
            with self._lock:
                plan = self._follow_plan
            if plan is None:
                self._set_route_active(False)
                return
            final = plan[-1].pose.position
            d_final = self._distance_to(final)
            carrot, at_end = self._carrot(plan) or (None, False)
            # Completion requires the carrot walk to have REACHED the path
            # end, not mere spatial proximity to plan[-1]: route legs can
            # pass near the final checkpoint's location mid-route (observed:
            # spurious completion 1.32 m from the final with two checkpoints
            # still unvisited — vehicle parked mid-route).
            if (at_end and d_final is not None
                    and d_final < self.waypoint_tolerance):
                self._publish_term_goal(plan[-1])
                self.get_logger().info(
                    f'follower: route complete ({d_final:.2f} m from end)')
                with self._lock:
                    self._follow_done_plan = (final.x, final.y, final.z)
                    self._follow_plan = None
                self._set_route_active(False)
                return
            # Catch-up gate: if MIGHTY has gone idle (no trajectories) with
            # its committed end still far from the vehicle, withhold new
            # carrots — the controller is still flying the remaining path to
            # that end. Publishing a goal now would make MIGHTY replan from
            # the far end (spatial gap -> uncommanded straight line after the
            # timeline reset). Resume once the vehicle has caught up.
            with self._lock:
                idle = (self._last_traj_time > 0 and
                        time.monotonic() - self._last_traj_time > 2.0)
                traj_end = self._traj_end
            if idle and traj_end is not None:
                d_end = self._distance_to_xyz(*traj_end)
                if d_end is not None and d_end > 2.5:
                    time.sleep(0.3)
                    continue
            if carrot is not None:
                c = carrot.pose.position
                # Defensive: never hand MIGHTY a goal at the vehicle's own
                # position (goal_radius would flip it to GOAL_REACHED and it
                # stops planning — the hover-deadlock failure mode). Skip
                # this tick; the re-anchored path resolves it next second.
                d_c = self._distance_to(c)
                if d_c is not None and d_c < 1.0 and carrot is not plan[-1]:
                    time.sleep(0.3)
                    continue
                now = time.monotonic()
                moved = (last_goal is None or
                         math.dist((c.x, c.y, c.z), last_goal) > 1.0)
                if moved or now - last_pub > self.republish_s:
                    self._publish_term_goal(carrot)
                    last_goal = (c.x, c.y, c.z)
                    last_pub = now
            time.sleep(0.3)

    # ------------------------------------------------------------------
    # NavigateTask
    # ------------------------------------------------------------------

    def _handle_goal(self, goal_request):
        # AirStack patch: an EMPTY-plan goal is a speed-only "activator" (the
        # droan_gl contract the search planners speak): it sets the cap and
        # completes at once while the global_plan follower keeps driving.
        if not goal_request.global_plan.poses:
            return GoalResponse.ACCEPT
        if self._task_active:
            self.get_logger().warn('Rejecting NavigateTask goal: task already active')
            return GoalResponse.REJECT
        self._task_active = True
        return GoalResponse.ACCEPT

    def _apply_speed_cap(self, goal, what):
        """Take `max_speed_mps` off a NavigateTask goal (0 = no cap)."""
        cap = float(getattr(goal, 'max_speed_mps', 0.0) or 0.0)
        with self._lock:
            changed = cap != self._speed_cap
            self._speed_cap = cap
        if changed:
            target = cap if cap > 0.0 else self._v_max_default
            self.get_logger().info(
                f'speed cap {cap:.1f} m/s ({what}) -> MIGHTY v_max {target:.1f} m/s'
                if cap > 0.0 else
                f'speed cap cleared ({what}) -> MIGHTY v_max back to {target:.1f} m/s')
            if self._push_vmax_enabled:
                self._push_vmax(target)
            else:
                self.get_logger().info(
                    f'(push_vmax_to_mighty off: MIGHTY keeps v_max {self._v_max_default:.1f})')
        return cap

    def _push_vmax(self, v):
        """Set MIGHTY's live `v_max` (the speed it PLANS at). The forwarded
        waypoints are never throttled below it (see capped_speed), so the
        vehicle's speed and MIGHTY's replan anchor stay consistent."""
        if not self._vmax_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f'MIGHTY parameter service not available ({self._vmax_client.srv_name}); '
                f'v_max stays {self._v_max_effective:.1f}')
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name='v_max', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))]
        fut = self._vmax_client.call_async(req)

        def _done(f, v=v):
            try:
                ok = all(r.successful for r in f.result().results)
            except Exception as exc:
                ok = False
                self.get_logger().warn(f'set v_max failed: {exc}')
            if ok:
                with self._lock:
                    self._v_max_effective = float(v)
                self.get_logger().info(
                    f'MIGHTY v_max -> {v:.1f} m/s: ok (start guard now '
                    f'{max(self._guard_min, self._guard_s * v):.1f} m)')
            else:
                self.get_logger().warn(f'MIGHTY v_max -> {v:.1f} m/s: REFUSED')
        fut.add_done_callback(_done)

    def _handle_cancel(self, goal_handle):
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    def _set_mode(self, mode):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f'set_trajectory_mode service not available ({self.mode_client.srv_name})')
            return
        req = TrajectoryMode.Request()
        req.mode = mode
        names = {0: 'PAUSE', 1: 'ROBOT_POSE', 2: 'TRACK', 3: 'ADD_SEGMENT', 4: 'REWIND'}
        fut = self.mode_client.call_async(req)

        def _done(f, mode=mode):
            try:
                r = f.result()
                self.get_logger().info(
                    f'trajectory controller mode -> {names.get(mode, mode)}: '
                    f'{"ok" if getattr(r, "success", True) else "REFUSED"}')
            except Exception as exc:
                self.get_logger().warn(f'set_trajectory_mode({names.get(mode, mode)}) failed: {exc}')
        fut.add_done_callback(_done)

    def _distance_to(self, pos):
        with self._lock:
            odom = self._odom
        if odom is None:
            return None
        dx = odom.pose.pose.position.x - pos.x
        dy = odom.pose.pose.position.y - pos.y
        dz = odom.pose.pose.position.z - pos.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _distance_to_xyz(self, x, y, z):
        with self._lock:
            odom = self._odom
        if odom is None:
            return None
        p = odom.pose.pose.position
        return math.dist((p.x, p.y, p.z), (x, y, z))

    def _publish_term_goal(self, pose_stamped):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = pose_stamped.header.frame_id or self.world_frame
        msg.pose = pose_stamped.pose
        self.term_goal_pub.publish(msg)

    def _finish(self, goal_handle, success, message):
        self._set_route_active(False)
        self._set_mode(TrajectoryMode.Request.TRACK)
        result = NavigateTask.Result()
        result.success = success
        result.message = message
        self._task_active = False
        if success:
            goal_handle.succeed()
        elif self._cancel_requested and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _execute_navigate(self, goal_handle):
        goal = goal_handle.request
        if not goal.global_plan.poses:
            cap = self._apply_speed_cap(goal, 'speed-only NavigateTask')
            result = NavigateTask.Result()
            result.success = True
            result.message = f'speed cap set to {cap:.1f} m/s; following global_plan'
            goal_handle.succeed()
            return result
        self._apply_speed_cap(goal, 'NavigateTask with plan')
        self._cancel_requested = False
        poses = self._poses_in_world_frame(list(goal.global_plan.poses), 'NavigateTask')
        if poses is None:
            self._task_active = False
            result = NavigateTask.Result()
            result.success = False
            result.message = 'goal frame not transformable to ' + self.world_frame
            goal_handle.abort()
            return result
        p0 = poses[0].pose.position
        self.get_logger().info(
            f'NavigateTask goal in {self.world_frame}: first pose '
            f'({p0.x:.1f}, {p0.y:.1f}, {p0.z:.1f}) (received in '
            f'{goal.global_plan.poses[0].header.frame_id or goal.global_plan.header.frame_id or "?"!r})')
        final_pos = poses[-1].pose.position
        goal_tol = max(0.1, float(goal.goal_tolerance_m))

        self.get_logger().info(
            f'NavigateTask: {len(poses)} waypoints, goal tolerance {goal_tol:.2f} m')

        self._set_route_active(True)
        self._set_mode(TrajectoryMode.Request.TRACK)   # see _follow_route

        idx = 0
        last_pub = 0.0
        rate_s = 0.2

        while rclpy.ok():
            if self._cancel_requested and goal_handle.is_cancel_requested:
                return self._finish(goal_handle, False, 'Canceled')

            now = time.monotonic()
            # (Re-)issue the current checkpoint: on advance, and periodically in
            # case MIGHTY missed it or has stalled (re-pinning the same goal
            # restarts its replanning timer, which is a safe unstick).
            if now - last_pub > self.republish_s:
                self._publish_term_goal(poses[idx])
                last_pub = now

            dist_final = self._distance_to(final_pos)
            if dist_final is not None:
                fb = NavigateTask.Feedback()
                fb.status = f'navigating wp {idx + 1}/{len(poses)}'
                fb.distance_to_goal = float(dist_final)
                with self._lock:
                    odom = self._odom
                fb.current_position.x = odom.pose.pose.position.x
                fb.current_position.y = odom.pose.pose.position.y
                fb.current_position.z = odom.pose.pose.position.z
                goal_handle.publish_feedback(fb)

                if dist_final < goal_tol and idx == len(poses) - 1:
                    return self._finish(goal_handle, True, 'Goal reached')

                if idx < len(poses) - 1:
                    d = self._distance_to(poses[idx].pose.position)
                    if d is not None and d < self.waypoint_tolerance:
                        idx += 1
                        self._publish_term_goal(poses[idx])
                        last_pub = now
                        self.get_logger().info(
                            f'NavigateTask: advancing to waypoint {idx + 1}/{len(poses)}')

            time.sleep(rate_s)

        return self._finish(goal_handle, False, 'Node shutting down')


def main(args=None):
    rclpy.init(args=args)
    node = MightyBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
