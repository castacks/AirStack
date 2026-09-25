#!/usr/bin/env python3
"""mtl_metrics_logger — record an MTL sortie and score it (Moon et al. 2022).

Runs at ``rate_hz`` (20 Hz). While this robot's follower is INGRESS/SEARCH on a
started plan it records pose, commanded AND measured gimbal angles, the
boresight ground point, footprint, tracking and pointing error, and
accumulates per-target detection probability from the MEASURED gimbal state,
and the residual belief over the whole (normalised) prior: the probability the
target is still where this robot looked and missed it, lower is better.
When the follower reports COMPLETE or ABORTED it writes

    runs/<run_id>/<robot>/telemetry.csv
    runs/<run_id>/<robot>/detection.json
    runs/<run_id>/<robot>/residual_belief.csv
    runs/<run_id>/<robot>/report.html
    runs/<run_id>/ground_truth.json          (copy, for the team analysis)

``scripts/analyze_mtl_run.py --run-dir runs/<run_id>`` then fuses all robots.

Topics (relative; the module launch file maps them under /$ROBOT_NAME/):
  in  odometry, gimbal/state, gimbal/cmd_pitch_yaw, search/plan, search/follower_status
  out search/detection_markers (MarkerArray), search/footprint (PolygonStamped),
      search/metrics (std_msgs/String, JSON, 1 Hz)
"""

from __future__ import annotations

import json
import math
import os
import shutil
from pathlib import Path

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Point32, PolygonStamped, Vector3
from mtl_msgs.msg import FollowerStatus, SearchPlan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from mtl_metrics_logger.analysis import cells_world, targets_world, write_run_outputs
from mtl_metrics_logger.detection import (DetectionModel, TeamScorer, boresight_ground_point, footprint_radius,
                                          prior_from_scenario)

RECORDING = (FollowerStatus.INGRESS, FollowerStatus.SEARCH)
FINAL = (FollowerStatus.COMPLETE, FollowerStatus.ABORTED)


def _as_list(v) -> list:
    """A message array field as a list ([] when absent or not a sequence)."""
    try:
        return list(v)
    except TypeError:
        return []


def _load_json(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


class MtlMetricsLogger(Node):
    def __init__(self) -> None:
        super().__init__("mtl_metrics_logger")
        p = self.declare_parameter
        default_dir = "/root/AirStack/stacks/mtl_search/config"
        self.scenario_file = str(p("scenario_file", f"{default_dir}/scenario.json").value)
        self.ground_truth_file = str(p("ground_truth_file", "").value) or str(
            Path(self.scenario_file).with_name("ground_truth.json"))
        self.belief_png_file = str(p("belief_png_file", "").value) or str(
            Path(self.scenario_file).with_name("belief.png"))
        self.runs_root = str(p("runs_root", "/root/AirStack/runs").value)
        self.agent_name = str(p("agent_name", os.environ.get("ROBOT_NAME", "robot_1")).value)
        self.rate_hz = float(p("rate_hz", 20.0).value)
        self.ground_z_world = float(p("ground_z_world", 0.0).value)
        self.frame_id = str(p("frame_id", "map").value)

        self.scenario = _load_json(self.scenario_file)
        self.ground_truth = _load_json(self.ground_truth_file)
        self.model = DetectionModel.from_scenario(self.scenario["sensor"]["detection"])
        self.fov = math.radians(float(self.scenario["sensor"]["fov_deg"]))
        self.targets_w = targets_world(self.ground_truth)
        self.cells_w, self.cell_mass = cells_world(self.scenario)
        self.prior = prior_from_scenario(self.scenario)  # normalised prior raster (residual belief)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, "odometry", self._on_odom, 10)
        self.create_subscription(Vector3, "gimbal/state", self._on_meas, qos_profile_sensor_data)
        self.create_subscription(Vector3, "gimbal/cmd_pitch_yaw", self._on_cmd, 10)
        self.create_subscription(SearchPlan, "search/plan", self._on_plan, latched)
        self.create_subscription(FollowerStatus, "search/follower_status", self._on_status, 10)
        self.markers_pub = self.create_publisher(MarkerArray, "search/detection_markers", latched)
        self.footprint_pub = self.create_publisher(PolygonStamped, "search/footprint", 10)
        self.metrics_pub = self.create_publisher(String, "search/metrics", 10)

        self.odom = None
        self.meas = None
        self.meas_time = None
        self.cmd = None
        self.plan: SearchPlan | None = None
        self.status: FollowerStatus | None = None
        self.origin = (0.0, 0.0, 0.0)
        self.rows: list[dict] = []
        self.scorer: TeamScorer | None = None
        self.recording_plan = ""
        self.finished_plans: set[str] = set()
        self.t0 = None
        self.last_t = None
        self._metrics_div = 0
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f"mtl_metrics_logger: agent {self.agent_name}, {len(self.targets_w)} ground-truth targets, "
            f"{len(self.cells_w)} cells, "
            f"{'prior ' + str(self.prior.nx) + 'x' + str(self.prior.ny) if self.prior else 'NO prior raster'}"
            f" for the residual belief, runs -> {self.runs_root}")

    # ------------------------------------------------------------ callbacks
    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_meas(self, msg: Vector3) -> None:
        self.meas = (msg.x, msg.y, msg.z)
        self.meas_time = self.get_clock().now()

    def _on_cmd(self, msg: Vector3) -> None:
        self.cmd = (msg.x, msg.y, msg.z)

    def _on_plan(self, msg: SearchPlan) -> None:
        self.plan = msg
        o = msg.map_origin_in_world
        self.origin = (o.x, o.y, o.z)
        self._publish_target_markers()

    def _on_status(self, msg: FollowerStatus) -> None:
        self.status = msg

    # ------------------------------------------------------------ main loop
    def _tick(self) -> None:
        now = self.get_clock().now()
        st, plan, odom = self.status, self.plan, self.odom
        if st is None or plan is None or odom is None or not plan.start_mission or st.plan_id != plan.plan_id:
            return
        if st.plan_id in self.finished_plans:
            return
        if st.state in RECORDING and self.recording_plan != st.plan_id:
            self._begin(st.plan_id, now)
        if self.recording_plan != st.plan_id:
            return
        t = (now - self.t0).nanoseconds * 1e-9
        dt = 0.0 if self.last_t is None else max(t - self.last_t, 0.0)
        self.last_t = t
        row = self._sample(t, st, odom, now)
        self.rows.append(row)
        pitch = row["meas_pitch"] if row["gimbal_measured"] else row["cmd_pitch"]
        yaw = row["meas_yaw"] if row["gimbal_measured"] else row["cmd_yaw"]
        self.scorer.step(t, {self.agent_name: {"pos": (row["x_world"], row["y_world"], row["z_world"]),
                                               "pitch": pitch, "yaw": yaw, "ground_z": self.ground_z_world}}, dt)
        self._publish_footprint(row, now)
        self._metrics_div += 1
        if self._metrics_div % max(int(self.rate_hz), 1) == 0:
            self._publish_target_markers()
            s = self.scorer.summary()
            self.metrics_pub.publish(String(data=json.dumps({
                "plan_id": st.plan_id, "t": round(t, 1), "found": s["targets_detected"],
                "total": s["targets_total"], "mass_covered": s["belief_mass_covered"],
                "residual_mass": None if self.scorer.residual is None
                else round(self.scorer.residual.residual_mass, 6),
                "distance_m": s["total_path_length_m"]})))
        if st.state in FINAL:
            self._finish(st)

    def _begin(self, plan_id: str, now) -> None:
        self.recording_plan = plan_id
        self.rows = []
        self.scorer = TeamScorer(self.targets_w, self.cells_w, self.cell_mass, self.model, self.fov,
                                 prior=self.prior)
        self.t0, self.last_t = now, None
        self.get_logger().info(f"recording sortie {plan_id}")

    def _sample(self, t: float, st: FollowerStatus, odom: Odometry, now) -> dict:
        ox, oy, oz = self.origin
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        v = odom.twist.twist.linear
        xw, yw, zw = p.x + ox, p.y + oy, p.z + oz
        measured = self.meas is not None and self.meas_time is not None and \
            (now - self.meas_time).nanoseconds * 1e-9 < 0.5
        cmd = self.cmd or (float("nan"),) * 3
        meas = self.meas if measured else (None, None, None)
        pitch = meas[1] if measured else cmd[1]
        yaw = meas[2] if measured else cmd[2]
        bore = None if pitch is None or math.isnan(pitch) else \
            boresight_ground_point((xw, yw, zw), pitch, yaw, self.ground_z_world)
        aim_w = (st.aim_point.x + ox, st.aim_point.y + oy)
        return {
            "t": t, "stamp": now.nanoseconds * 1e-9, "agent": self.agent_name, "state": st.state_name,
            "x_map": p.x, "y_map": p.y, "z_map": p.z, "x_world": xw, "y_world": yw, "z_world": zw,
            "n": yw, "e": xw, "d": -zw,
            "yaw": math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
            "vx": v.x, "vy": v.y, "vz": v.z, "speed": math.hypot(v.x, v.y),
            "cmd_roll": cmd[0], "cmd_pitch": cmd[1], "cmd_yaw": cmd[2],
            "meas_roll": meas[0], "meas_pitch": meas[1], "meas_yaw": meas[2], "gimbal_measured": int(measured),
            "bore_x_world": bore[0] if bore else None, "bore_y_world": bore[1] if bore else None,
            "slant_m": bore[2] if bore else None,
            "footprint_r_m": footprint_radius(bore[2], self.fov) if bore else None,
            "progress_m": st.progress_m, "remaining_m": st.remaining_m, "xte_m": st.cross_track_error_m,
            "carrot_x_map": st.carrot.x, "carrot_y_map": st.carrot.y, "carrot_z_map": st.carrot.z,
            "aim_x_world": aim_w[0], "aim_y_world": aim_w[1],
            "pointing_error_m": math.hypot(bore[0] - aim_w[0], bore[1] - aim_w[1]) if bore else None,
        }

    def _finish(self, st: FollowerStatus) -> None:
        plan = self.plan
        self.finished_plans.add(st.plan_id)
        self.recording_plan = ""
        run_id = plan.run_id or "unnamed"
        run_root = Path(self.runs_root) / run_id
        out = run_root / self.agent_name
        ox, oy, _ = self.origin
        planned = [(w.position.x + ox, w.position.y + oy) for w in plan.trajectory.waypoints]
        # the planned looks (track + scheduled boresight, world ENU) -> planned residual belief
        oz = self.origin[2]
        wps, bores, times = (_as_list(plan.trajectory.waypoints), _as_list(plan.boresight),
                             _as_list(plan.time_s))
        n_look = min(len(wps), len(bores), len(times))
        looks = {"t": [float(v) for v in times[:n_look]],
                 "pos": [(w.position.x + ox, w.position.y + oy, w.position.z + oz) for w in wps[:n_look]],
                 "bore": [(b.x + ox, b.y + oy, b.z + oz) for b in bores[:n_look]]} if n_look else None
        try:
            belief = Path(self.belief_png_file).read_bytes() if Path(self.belief_png_file).is_file() else None
            res = write_run_outputs(
                out, scenario=self.scenario, ground_truth=self.ground_truth,
                rows_by_agent={self.agent_name: self.rows},
                planned_by_agent={self.agent_name: {"planned": planned, "home": [ox, oy],
                                                    "serviced_cells": list(plan.serviced_cells),
                                                    "planned_length_m": plan.planned_length_m,
                                                    "looks": looks}},
                title=f"MTL sortie — {self.agent_name}",
                subtitle=f"run {run_id} · plan {st.plan_id} · outcome {st.state_name} · "
                         f"{len(self.rows)} samples at {self.rate_hz:g} Hz",
                belief_png=belief,
                extra={"run_id": run_id, "plan_id": st.plan_id, "outcome": st.state_name,
                       "agent": self.agent_name})
            shutil.copyfile(self.ground_truth_file, run_root / "ground_truth.json")
            if Path(self.belief_png_file).is_file():
                shutil.copyfile(self.belief_png_file, run_root / "belief.png")
            s = res["summary"]
            resid = s.get("residual_belief_mass")
            planned_resid = s.get("planned_residual_belief_mass")
            self.get_logger().info(
                f"sortie {st.plan_id} {st.state_name}: residual belief "
                f"{'n/a' if resid is None else f'{resid:.4f}'}"
                f"{'' if planned_resid is None else f' (planned {planned_resid:.4f})'} = P(target missed by "
                f"this robot), {s['targets_detected']}/{s['targets_total']} targets, "
                f"{100 * s['belief_mass_fraction']:.1f} % of the valid-cell mass reached, "
                f"report -> {out / 'report.html'}")
        except OSError as exc:
            self.get_logger().error(f"could not write run outputs to {out}: {exc} (is runs/ mounted?)")

    # ------------------------------------------------------------ visualisation
    def _publish_footprint(self, row: dict, now) -> None:
        if row["bore_x_world"] is None:
            return
        ox, oy, oz = self.origin
        r = row["footprint_r_m"]
        poly = PolygonStamped()
        poly.header.stamp = now.to_msg()
        poly.header.frame_id = self.frame_id
        for k in range(24):
            a = 2.0 * math.pi * k / 24
            poly.polygon.points.append(Point32(x=float(row["bore_x_world"] - ox + r * math.cos(a)),
                                               y=float(row["bore_y_world"] - oy + r * math.sin(a)),
                                               z=float(self.ground_z_world - oz + 0.1)))
        self.footprint_pub.publish(poly)

    def _publish_target_markers(self) -> None:
        ox, oy, oz = self.origin
        arr = MarkerArray()
        states = self.scorer.targets if self.scorer else None
        for i, (x, y, z) in enumerate(self.targets_w):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "mtl_targets"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x - ox, y - oy, z - oz + 1.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 1.6
            m.scale.z = 2.0
            p = states[i].p_det if states else 0.0
            found = bool(states and states[i].detected)
            m.color.r, m.color.g, m.color.b, m.color.a = (0.05, 0.64, 0.05, 1.0) if found else \
                (0.82, 0.23 + 0.4 * p, 0.23, 0.9)
            arr.markers.append(m)
        self.markers_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MtlMetricsLogger()
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
