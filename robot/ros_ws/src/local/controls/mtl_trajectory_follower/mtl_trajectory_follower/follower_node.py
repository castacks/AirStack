#!/usr/bin/env python3
"""mtl_trajectory_follower — fly an MTL SearchPlan and point the gimbal.

Topics (relative names; the module launch file maps them to /$ROBOT_NAME/...):

  in   odometry                                  nav_msgs/Odometry (map frame)
  in   search/plan                               mtl_msgs/SearchPlan (transient local)
  in   trajectory_controller/tracking_point_nominal   airstack_msgs/Odometry
  in   gimbal/state                              geometry_msgs/Vector3 (measured, earth frame)
  in   search/abort                              std_msgs/Empty
  in   state_estimate_timed_out                  std_msgs/Bool (drone_safety_monitor)
  out  trajectory_controller/tracking_point      airstack_msgs/Odometry  -> pid_controller
  out  gimbal/cmd_pitch_yaw                      geometry_msgs/Vector3 (x roll, y pitch, z yaw)
  out  search/follower_status                    mtl_msgs/FollowerStatus
  out  search/carrot, search/aim_point           geometry_msgs/PointStamped (viz)
  srv  trajectory_controller/set_trajectory_mode airstack_msgs/srv/TrajectoryMode (client)
  tf   base_link -> camera_gimbal_link -> camera_optical_frame

Tracking-point arbitration: the stack remaps trajectory_controller's own output
to ``tracking_point_nominal``; this node is the only publisher of
``tracking_point``. Idle -> it forwards the nominal point unchanged (takeoff,
land and hover keep working); during a sortie it publishes the carrot. At the
end (or on abort) it asks the trajectory controller to hold (ROBOT_POSE) and
goes back to forwarding.
"""

from __future__ import annotations

import math

import rclpy
import rclpy.executors
import rclpy.time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from airstack_msgs.msg import Odometry as TrackingPoint
from airstack_msgs.srv import TrajectoryMode
from geometry_msgs.msg import PointStamped, TransformStamped, Vector3
from mtl_msgs.msg import FollowerStatus, SearchPlan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from mtl_trajectory_follower import follower_core as fc
from mtl_trajectory_follower.gimbal_math import (OPTICAL_FROM_GIMBAL_QUAT, euler_zyx_to_matrix,
                                                 mat_mul, mat_t, matrix_to_quat, quat_to_matrix)


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def track_from_plan(msg: SearchPlan) -> fc.Track:
    wps = msg.trajectory.waypoints
    return fc.Track(
        x=[w.position.x for w in wps], y=[w.position.y for w in wps], z=[w.position.z for w in wps],
        yaw=[w.yaw for w in wps], speed=[w.velocity for w in wps],
        bx=[p.x for p in msg.boresight], by=[p.y for p in msg.boresight], bz=[p.z for p in msg.boresight],
        arc=list(msg.arc_length_m), t=list(msg.time_s), phi=list(msg.planned_gimbal_phi_rad))


class MtlTrajectoryFollower(Node):
    def __init__(self) -> None:
        super().__init__("mtl_trajectory_follower")
        p = self.declare_parameter
        self.rate_hz = float(p("rate_hz", 20.0).value)
        self.lookahead_turn_radii = float(p("lookahead_turn_radii", 1.2).value)
        self.lookahead_m = float(p("lookahead_m", 0.0).value)
        self.window_lookaheads = float(p("window_lookaheads", 1.5).value)
        self.finish_tolerance_m = float(p("finish_tolerance_m", 3.0).value)
        self.ingress_tolerance_m = float(p("ingress_tolerance_m", 4.0).value)
        self.ingress_alt_tolerance_m = float(p("ingress_alt_tolerance_m", 2.0).value)
        self.gimbal_lead_s = float(p("gimbal_lead_s", 0.2).value)
        self.yaw_lead_s = float(p("yaw_lead_s", 0.5).value)
        self.two_axis_rate_deg_s = float(p("two_axis_rate_deg_s", 120.0).value)
        self.idle_pitch_deg = float(p("idle_gimbal_pitch_deg", 60.0).value)
        self.hold_after_complete_s = float(p("hold_after_complete_s", 3.0).value)
        self.odom_timeout_s = float(p("odometry_timeout_s", 1.0).value)
        self.max_plan_age_s = float(p("max_plan_age_s", 30.0).value)
        self.tf_prefix = str(p("tf_prefix", "").value)
        self.base_frame = str(p("base_frame", "base_link").value)
        self.publish_tf = bool(p("publish_tf", True).value)
        self.mount_offset = [float(v) for v in p("gimbal_mount_offset_m", [0.10, 0.0, -0.08]).value]

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, "odometry", self._on_odom, 10)
        self.create_subscription(SearchPlan, "search/plan", self._on_plan, latched)
        self.create_subscription(TrackingPoint, "trajectory_controller/tracking_point_nominal",
                                 self._on_nominal, 10)
        self.create_subscription(Vector3, "gimbal/state", self._on_gimbal_state, qos_profile_sensor_data)
        self.create_subscription(Empty, "search/abort", lambda _m: self._abort("abort requested"), 10)
        self.create_subscription(Bool, "state_estimate_timed_out", self._on_state_timeout, 10)

        self.tp_pub = self.create_publisher(TrackingPoint, "trajectory_controller/tracking_point", 10)
        self.gimbal_pub = self.create_publisher(Vector3, "gimbal/cmd_pitch_yaw", 10)
        self.status_pub = self.create_publisher(FollowerStatus, "search/follower_status", 10)
        self.carrot_pub = self.create_publisher(PointStamped, "search/carrot", 10)
        self.aim_pub = self.create_publisher(PointStamped, "search/aim_point", 10)
        self.mode_client = self.create_client(TrajectoryMode, "trajectory_controller/set_trajectory_mode")

        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        if self.publish_tf:
            self._static = StaticTransformBroadcaster(self)
            self._send_optical_tf()

        self.odom: Odometry | None = None
        self.odom_time = None
        self.plan: SearchPlan | None = None
        self.follower: fc.TrackFollower | None = None
        self.state = fc.IDLE
        self.plan_id = ""
        self.last_out: fc.FollowerOutput | None = None
        self.active = False
        self.t_start = None
        self.t_complete = None
        self.last_tick = None
        self.gimbal_measured: tuple[float, float, float] | None = None
        self.last_cmd = (0.0, math.radians(self.idle_pitch_deg), 0.0)
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info("mtl_trajectory_follower ready (idle: forwarding the nominal tracking point)")

    # ------------------------------------------------------------------ inputs
    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        self.odom_time = self.get_clock().now()

    def _on_gimbal_state(self, msg: Vector3) -> None:
        self.gimbal_measured = (msg.x, msg.y, msg.z)

    def _on_state_timeout(self, msg: Bool) -> None:
        if msg.data and self.active:
            self._abort("drone_safety_monitor reports state_estimate_timed_out")

    def _on_nominal(self, msg: TrackingPoint) -> None:
        if not self.active:
            self.tp_pub.publish(msg)

    def _on_plan(self, msg: SearchPlan) -> None:
        if not msg.start_mission:
            self.get_logger().info(f"preview plan {msg.plan_id} ({len(msg.boresight)} samples) - not flying it")
            return
        if msg.plan_id == self.plan_id and self.active:
            return
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        if age > self.max_plan_age_s:
            # a latched plan replayed to a restarted follower: never re-fly a stale sortie
            self.get_logger().warn(f"ignoring stale plan {msg.plan_id} ({age:.0f} s old)")
            return
        try:
            track = track_from_plan(msg)
        except ValueError as exc:
            self.get_logger().error(f"rejecting plan {msg.plan_id}: {exc}")
            return
        cfg = fc.FollowerConfig(
            lookahead_m=self.lookahead_m, lookahead_turn_radii=self.lookahead_turn_radii,
            min_turn_radius_m=msg.min_turn_radius_m, window_lookaheads=self.window_lookaheads,
            finish_tolerance_m=self.finish_tolerance_m, ingress_tolerance_m=self.ingress_tolerance_m,
            ingress_alt_tolerance_m=self.ingress_alt_tolerance_m, gimbal_lead_s=self.gimbal_lead_s,
            yaw_lead_s=self.yaw_lead_s, single_axis=msg.single_axis_gimbal, tilt_rad=msg.mount_tilt_rad,
            gimbal_max_rad=msg.gimbal_max_rad or math.radians(80.0),
            gimbal_rate_rad_s=msg.gimbal_rate_rad_s or math.radians(120.0),
            pitch_nudge_max_rad=msg.pitch_nudge_max_rad or math.radians(5.0),
            two_axis_rate_rad_s=math.radians(self.two_axis_rate_deg_s), speed_mps=msg.speed_mps)
        if self.active:
            self.get_logger().warn(f"replacing active sortie {self.plan_id} with {msg.plan_id}")
        self.plan, self.plan_id = msg, msg.plan_id
        self.follower = fc.TrackFollower(track, cfg)
        self.state, self.active = fc.IDLE, True
        self.t_start, self.t_complete, self.last_out = self.get_clock().now(), None, None
        self.get_logger().info(
            f"sortie {msg.plan_id}: {len(track)} samples, {track.total:.0f} m, "
            f"{'single-axis tau=%.0f deg' % math.degrees(cfg.tilt_rad) if cfg.single_axis else '2-axis'}, "
            f"lookahead {cfg.lookahead:.1f} m")

    # ------------------------------------------------------------------ control
    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = 0.0 if self.last_tick is None else (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        odom = self.odom
        have_odom = odom is not None and (now - self.odom_time).nanoseconds * 1e-9 <= self.odom_timeout_s

        if self.active and not have_odom:
            if odom is not None:
                self._abort("odometry went stale")
            return  # nothing sensible to command without a state estimate

        if have_odom:
            pos = (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)
            vehicle_yaw = _yaw_of(odom.pose.pose.orientation)
        if self.active and self.follower is not None:
            out = self.follower.step(pos, vehicle_yaw, max(dt, 0.0))
            self.last_out, self.state = out, out.state
            self._publish_tracking_point(out, now, odom.header.frame_id or "map")
            self._publish_gimbal(out.gimbal)
            self._publish_points(out, now, odom.header.frame_id or "map")
            if out.state == fc.COMPLETE:
                if self.t_complete is None:
                    self.t_complete = now
                    self.get_logger().info(f"sortie {self.plan_id} complete ({out.progress_m:.0f} m)")
                elif (now - self.t_complete).nanoseconds * 1e-9 >= self.hold_after_complete_s:
                    self._hand_back("sortie complete")
        elif have_odom:
            self.last_cmd = (0.0, math.radians(self.idle_pitch_deg), vehicle_yaw)
            self._publish_gimbal(self.last_cmd)
        self._publish_status(now)
        if have_odom:
            self._send_gimbal_tf(now, odom)

    def _abort(self, why: str) -> None:
        if not self.active:
            return
        self.get_logger().warn(f"sortie {self.plan_id} ABORTED: {why}")
        self.state = fc.ABORTED
        self._hand_back(why)

    def _hand_back(self, why: str) -> None:
        """Stop publishing carrots; have the trajectory controller hold where we are."""
        self.active = False
        if self.mode_client.service_is_ready():
            req = TrajectoryMode.Request()
            req.mode = TrajectoryMode.Request.ROBOT_POSE
            self.mode_client.call_async(req)
        else:
            self.get_logger().warn("trajectory_controller/set_trajectory_mode unavailable; "
                                   "the nominal tracking point is forwarded as-is")
        self.get_logger().info(f"handing control back to trajectory_controller ({why})")

    # ------------------------------------------------------------------ outputs
    def _publish_tracking_point(self, out: fc.FollowerOutput, now, frame: str) -> None:
        tp = TrackingPoint()
        tp.header.stamp = now.to_msg()
        tp.header.frame_id = frame
        tp.child_frame_id = frame
        tp.pose.position.x, tp.pose.position.y, tp.pose.position.z = out.carrot
        tp.pose.orientation.z = math.sin(0.5 * out.carrot_yaw)
        tp.pose.orientation.w = math.cos(0.5 * out.carrot_yaw)
        tp.twist.linear.x, tp.twist.linear.y, tp.twist.linear.z = out.carrot_velocity
        self.tp_pub.publish(tp)

    def _publish_gimbal(self, rpy) -> None:
        self.last_cmd = tuple(rpy)
        self.gimbal_pub.publish(Vector3(x=float(rpy[0]), y=float(rpy[1]), z=float(rpy[2])))

    def _publish_points(self, out: fc.FollowerOutput, now, frame: str) -> None:
        for pub, p in ((self.carrot_pub, out.carrot), (self.aim_pub, out.aim)):
            msg = PointStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = frame
            msg.point.x, msg.point.y, msg.point.z = p
            pub.publish(msg)

    def _publish_status(self, now) -> None:
        if not self.plan_id:
            return
        st = FollowerStatus()
        st.header.stamp = now.to_msg()
        st.header.frame_id = "map"
        st.plan_id = self.plan_id
        state = self.state
        if not self.active and state not in (fc.COMPLETE, fc.ABORTED):
            state = fc.IDLE
        st.state = int(state)
        st.state_name = fc.STATE_NAMES[state]
        if self.follower is not None:
            st.total_m = float(self.follower.track.total)
        out = self.last_out
        if out is not None:
            st.progress_m = float(out.progress_m)
            st.remaining_m = float(out.remaining_m)
            st.cross_track_error_m = float(out.cross_track_error_m)
            st.track_index = int(out.track_index)
            st.carrot.x, st.carrot.y, st.carrot.z = out.carrot
            st.aim_point.x, st.aim_point.y, st.aim_point.z = out.aim
            st.gimbal_cmd = Vector3(x=float(out.gimbal[0]), y=float(out.gimbal[1]), z=float(out.gimbal[2]))
        if self.t_start is not None:
            st.elapsed_s = (now - self.t_start).nanoseconds * 1e-9
        self.status_pub.publish(st)

    def _send_optical_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.tf_prefix + "camera_gimbal_link"
        t.child_frame_id = self.tf_prefix + "camera_optical_frame"
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = OPTICAL_FROM_GIMBAL_QUAT
        self._static.sendTransform(t)

    def _send_gimbal_tf(self, now, odom: Odometry) -> None:
        if self.tf is None:
            return
        roll, pitch, yaw = self.gimbal_measured or self.last_cmd
        q = odom.pose.pose.orientation
        r_base = quat_to_matrix(q.x, q.y, q.z, q.w)           # base_link -> map
        r_cam = euler_zyx_to_matrix(roll, pitch, yaw)         # camera -> earth (= map axes)
        rel = matrix_to_quat(mat_mul(mat_t(r_base), r_cam))   # camera -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.tf_prefix + self.base_frame
        t.child_frame_id = self.tf_prefix + "camera_gimbal_link"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = self.mount_offset
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = rel
        self.tf.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MtlTrajectoryFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
