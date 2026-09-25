#!/usr/bin/env python3
"""Build ONE rich, team-wide Foxglove file from an MTL run folder.

    pip install -r scripts/requirements-mtl-viz.txt        # once, on the host
    python3 scripts/mtl_foxglove.py --run-dir runs/latest
    # -> runs/<run_id>/foxglove/<run_id>.mcap  +  mtl_layout.json (Foxglove: Layouts -> Import)

Inputs (all optional except the scenario): per robot ``runs/<run>/<robot>/bag/*.mcap``
(recorded by stacks/mtl_search/scripts/mtl_sortie.sh), ``track.json``, ``telemetry.csv``
(fallback when a robot has no bag), and ``scenario.json`` / ``ground_truth.json``.

Why a converted file instead of just opening the robot bags: every robot publishes the
same frame names (``map``, ``base_link``) on its own ``/tf``, each with its own origin
(its spawn point). This tool puts the whole team in ONE world frame
(``world -> robot_N/map -> robot_N/base_link -> robot_N/gimbal -> robot_N/camera_optical``)
and adds what the raw bags don't carry: the scenario (area, prior, valid cells, ground
truth targets coloured by their live detection probability), camera footprints, boresight
rays, flown trails, planned tracks, a camera frustum per drone, JPEG camera streams with
calibration (image panels + images projected in 3D), plottable telemetry and team metrics,
an event log, GPS for the Map panel — and a copy of every recorded raw ROS topic under
``/raw/robot_N/...`` for the Raw Messages / Plot panels.

Topics written (see also mtl_layout.json):
  /tf                          foxglove.FrameTransforms (team TF tree)
  /world/area                  SceneUpdate: boundary, valid cells, homes
  /world/belief                Grid (prior belief; colour-mapped on field "belief" by the layout)
  /world/targets               SceneUpdate: ground truth, colour = P_det, label = time/finder
  /robot_N/model, /frustum     SceneUpdate (frame-locked drone + camera frustum)
  /robot_N/plan                SceneUpdate: planned track + planned boresight ground track
  /robot_N/trail               SceneUpdate: flown track so far
  /robot_N/sensor              SceneUpdate: footprint circle, boresight ray, aim, carrot, label
  /robot_N/camera/image        CompressedImage (JPEG)      /robot_N/camera/calibration
  /robot_N/gps                 LocationFix (Map panel)
  /robot_N/telemetry           JSON: state, speed, altitude, XTE, progress, gimbal cmd/meas, ...
  /team/metrics                JSON: targets found, belief mass covered, distance, P_det per target
  /events                      foxglove.Log: phase changes, takeoff, discoveries
  /raw/robot_N/...             every recorded ROS topic, untouched (CDR)
"""

from __future__ import annotations

import argparse
import bisect
import heapq
import io
import itertools
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "robot/ros_ws/src/behavior/mtl_metrics_logger"))
from mtl_metrics_logger.analysis import cells_world, score_agents, targets_world  # noqa: E402
from mtl_metrics_logger.detection import boresight_ground_point, footprint_radius  # noqa: E402
from mtl_metrics_logger.report import read_telemetry_csv  # noqa: E402

try:
    import numpy as np
    import foxglove
    from foxglove import channels as fch
    from foxglove import messages as fm
    from foxglove.mcap import MCAPCompression, MCAPWriteOptions
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
except ImportError as exc:  # pragma: no cover - dependency hint
    sys.exit(f"mtl_foxglove: missing dependency ({exc.name}). "
             f"Install with: pip install -r {REPO / 'scripts/requirements-mtl-viz.txt'}")

NS = 1_000_000_000
ROBOT_RGB = [(0.165, 0.471, 0.839), (0.922, 0.408, 0.204), (0.106, 0.686, 0.478),
             (0.62, 0.35, 0.85), (0.85, 0.25, 0.55), (0.35, 0.75, 0.85)]
ACTIVE = ("INGRESS", "SEARCH")
OPTICAL_FROM_GIMBAL = (-0.5, 0.5, -0.5, 0.5)   # (x, y, z, w) == mtl_trajectory_follower


# ============================================================================ math
def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_rot(q, v):
    p = q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), q_conj(q))
    return (p[0], p[1], p[2])


def q_norm(q):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def q_euler_zyx(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def hold(times: list[int], values: list, t: int, max_age_ns: int | None = None):
    """Last value at or before t (None if none / too old)."""
    i = bisect.bisect_right(times, t) - 1
    if i < 0:
        return None
    if max_age_ns is not None and t - times[i] > max_age_ns:
        return None
    return values[i]


# ======================================================================= foxglove
def ts(t_ns: int):
    return fm.Timestamp(int(t_ns // NS), int(t_ns % NS))


def col(rgb, a=1.0):
    return fm.Color(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


def p3(x, y, z):
    return fm.Point3(x=float(x), y=float(y), z=float(z))


def pose(x=0.0, y=0.0, z=0.0, q=(0.0, 0.0, 0.0, 1.0)):
    return fm.Pose(position=fm.Vector3(x=float(x), y=float(y), z=float(z)),
                   orientation=fm.Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])))


def line(points, rgb, a=1.0, thickness=0.3, kind=None, scale_invariant=False):
    return fm.LinePrimitive(type=kind or fm.LinePrimitiveLineType.LineStrip, pose=pose(),
                            thickness=float(thickness), scale_invariant=scale_invariant,
                            points=[p3(*p) for p in points], color=col(rgb, a))


def entity(t_ns, frame, eid, frame_locked=False, **prims):
    return fm.SceneEntity(timestamp=ts(t_ns), frame_id=frame, id=eid, frame_locked=frame_locked, **prims)


def pdet_rgb(p, thr):
    """red (0) -> amber (thr/2) -> green (>= thr)."""
    u = max(0.0, min(1.0, p / max(thr, 1e-6)))
    if u < 0.5:
        return (0.82, 0.23 + 0.9 * u, 0.23)
    return (0.82 - 1.5 * (u - 0.5), 0.68 - 0.1 * (u - 0.5), 0.23 - 0.3 * (u - 0.5))


# ========================================================================= inputs
@dataclass
class Robot:
    name: str
    index: int
    rgb: tuple
    home: tuple = (0.0, 0.0, 0.0)          # world position of this robot's map origin
    source: str = "none"
    bag: Path | None = None
    odom_t: list = field(default_factory=list)
    odom: list = field(default_factory=list)       # (pos_map, quat, vel)
    gs_t: list = field(default_factory=list)
    gs: list = field(default_factory=list)         # measured (roll, pitch, yaw)
    gc_t: list = field(default_factory=list)
    gc: list = field(default_factory=list)         # commanded
    st_t: list = field(default_factory=list)
    st: list = field(default_factory=list)         # follower status dicts
    gps_t: list = field(default_factory=list)
    gps: list = field(default_factory=list)
    caminfo: dict | None = None
    plan_path: list = field(default_factory=list)  # map frame
    plan_bore: list = field(default_factory=list)  # map frame
    n_images: int = 0

    def frame(self, f: str) -> str:
        return f"{self.name}/{f}"

    def world(self, p) -> tuple:
        return (p[0] + self.home[0], p[1] + self.home[1], p[2] + self.home[2])


def _bag_files(robot_dir: Path) -> list[Path]:
    return sorted((robot_dir / "bag").glob("*.mcap")) if (robot_dir / "bag").is_dir() else []


def load_bag(r: Robot, files: list[Path]) -> None:
    R = f"/{r.name}"
    want = {f"{R}/odometry_conversion/odometry", f"{R}/gimbal/state", f"{R}/gimbal/cmd_pitch_yaw",
            f"{R}/search/follower_status", f"{R}/gimbal/camera_info", f"{R}/search/plan",
            f"{R}/interface/mavros/global_position/global"}
    plans = []
    for path in files:
        with path.open("rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _schema, ch, msg, m in reader.iter_decoded_messages(topics=sorted(want), log_time_order=True):
                t, topic = int(msg.log_time), ch.topic
                if topic.endswith("/odometry"):
                    p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
                    r.odom_t.append(t)
                    r.odom.append(((p.x, p.y, p.z), q_norm((q.x, q.y, q.z, q.w)), (v.x, v.y, v.z)))
                elif topic.endswith("/gimbal/state"):
                    r.gs_t.append(t)
                    r.gs.append((m.x, m.y, m.z))
                elif topic.endswith("/cmd_pitch_yaw"):
                    r.gc_t.append(t)
                    r.gc.append((m.x, m.y, m.z))
                elif topic.endswith("/follower_status"):
                    r.st_t.append(t)
                    r.st.append({"state": m.state_name, "plan_id": m.plan_id, "progress_m": m.progress_m,
                                 "total_m": m.total_m, "remaining_m": m.remaining_m,
                                 "xte_m": m.cross_track_error_m,
                                 "carrot": (m.carrot.x, m.carrot.y, m.carrot.z),
                                 "aim": (m.aim_point.x, m.aim_point.y, m.aim_point.z)})
                elif topic.endswith("/camera_info") and r.caminfo is None:
                    r.caminfo = {"width": m.width, "height": m.height, "distortion_model": m.distortion_model,
                                 "D": list(m.d), "K": list(m.k), "R": list(m.r), "P": list(m.p)}
                elif topic.endswith("/search/plan"):
                    plans.append(m)
                elif topic.endswith("/global"):
                    r.gps_t.append(t)
                    r.gps.append((m.latitude, m.longitude, m.altitude))
    started = [p for p in plans if p.start_mission] or plans
    if started:
        p = started[-1]
        o = p.map_origin_in_world
        r.home = (o.x, o.y, o.z)
        r.plan_path = [(w.position.x, w.position.y, w.position.z) for w in p.trajectory.waypoints]
        r.plan_bore = [(b.x, b.y, b.z) for b in p.boresight]
    r.source = "bag"


def load_telemetry(r: Robot, rows: list[dict], t_base_ns: int) -> None:
    """Fallback for runs recorded before rosbags: 20 Hz telemetry.csv of the sortie."""
    use_stamp = all(row.get("stamp") is not None for row in rows[:5]) and rows
    for row in rows:
        t = int(((row["stamp"] if use_stamp else row["t"]) or 0.0) * NS) + (0 if use_stamp else t_base_ns)
        pos = (row["x_map"], row["y_map"], row["z_map"])
        r.odom_t.append(t)
        r.odom.append((pos, q_euler_zyx(0.0, 0.0, row.get("yaw") or 0.0),
                       (row.get("vx") or 0.0, row.get("vy") or 0.0, row.get("vz") or 0.0)))
        if row.get("meas_pitch") is not None and row.get("gimbal_measured"):
            r.gs_t.append(t)
            r.gs.append((row.get("meas_roll") or 0.0, row["meas_pitch"], row["meas_yaw"]))
        if row.get("cmd_pitch") is not None:
            r.gc_t.append(t)
            r.gc.append((row.get("cmd_roll") or 0.0, row["cmd_pitch"], row["cmd_yaw"]))
        r.st_t.append(t)
        r.st.append({"state": row.get("state") or "", "plan_id": "", "progress_m": row.get("progress_m") or 0.0,
                     "total_m": (row.get("progress_m") or 0.0) + (row.get("remaining_m") or 0.0),
                     "remaining_m": row.get("remaining_m") or 0.0, "xte_m": row.get("xte_m") or 0.0,
                     "carrot": (row.get("carrot_x_map") or 0.0, row.get("carrot_y_map") or 0.0,
                                row.get("carrot_z_map") or 0.0),
                     "aim": ((row.get("aim_x_world") or 0.0) - r.home[0], (row.get("aim_y_world") or 0.0) - r.home[1],
                             -r.home[2])})
    r.source = "telemetry.csv"


def load_track_json(r: Robot, path: Path) -> None:
    if r.plan_path or not path.is_file():
        return
    tr = json.loads(path.read_text())
    if r.source != "bag" and tr.get("home_enu"):
        r.home = tuple(float(v) for v in tr["home_enu"])
    s = tr.get("samples") or {}
    if isinstance(s, dict) and "x_map" in s:
        r.plan_path = list(zip(s["x_map"], s["y_map"], s["z_map"]))
        if "bx_map" in s:
            r.plan_bore = list(zip(s["bx_map"], s["by_map"], s.get("bz_map", [0.0] * len(s["bx_map"]))))


def iter_raw(r: Robot, skip_topics: set[str]) -> Iterator[tuple[int, str, Any, bytes]]:
    for path in _bag_files_cache.get(r.name, []):
        with path.open("rb") as f:
            reader = make_reader(f)
            for schema, ch, msg in reader.iter_messages(log_time_order=True):
                if ch.topic in skip_topics:
                    continue
                yield int(msg.log_time), ch.topic, schema, msg.data


def iter_images(r: Robot, quality: int, min_dt_ns: int) -> Iterator[tuple[int, Any, Any]]:
    from PIL import Image as PILImage
    topic = f"/{r.name}/gimbal/rgb"
    last = -10 ** 18
    for path in _bag_files_cache.get(r.name, []):
        with path.open("rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _s, _c, msg, m in reader.iter_decoded_messages(topics=[topic], log_time_order=True):
                t = int(msg.log_time)
                if t - last < min_dt_ns:
                    continue
                last = t
                enc = m.encoding.lower()
                ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
                if ch is None:
                    continue
                arr = np.frombuffer(bytes(m.data), dtype=np.uint8)
                step = m.step or m.width * ch
                arr = arr.reshape(m.height, step)[:, : m.width * ch].reshape(m.height, m.width, ch)
                if enc.startswith("bgr"):
                    arr = arr[..., [2, 1, 0]]
                elif ch == 4:
                    arr = arr[..., :3]
                img = PILImage.fromarray(arr if ch != 1 else arr[..., 0])
                buf = io.BytesIO()
                img.save(buf, format="JPEG", quality=quality)
                yield t, buf.getvalue(), (m.width, m.height)


_bag_files_cache: dict[str, list[Path]] = {}


# ========================================================================= build
class Out:
    """Collects (t, topic, message) and writes them time-ordered through foxglove-sdk."""

    JSON_TELEMETRY = {"type": "object", "properties": {k: {"type": t} for k, t in {
        "state": "string", "speed_mps": "number", "altitude_m": "number", "vz_mps": "number",
        "xte_m": "number", "progress_m": "number", "remaining_m": "number", "progress_pct": "number",
        "cmd_roll_deg": "number", "cmd_pitch_deg": "number", "cmd_yaw_deg": "number",
        "meas_roll_deg": "number", "meas_pitch_deg": "number", "meas_yaw_deg": "number",
        "gimbal_tracking_error_deg": "number", "slant_m": "number", "footprint_radius_m": "number",
        "pointing_error_m": "number", "heading_deg": "number", "x_world": "number", "y_world": "number"}.items()}}
    JSON_TEAM = {"type": "object", "properties": {
        "t_s": {"type": "number"}, "targets_found": {"type": "integer"}, "targets_total": {"type": "integer"},
        "belief_mass_covered": {"type": "number"}, "belief_mass_fraction": {"type": "number"},
        "distance_m": {"type": "number"}, "p_det": {"type": "array", "items": {"type": "number"}}}}

    def __init__(self, ctx):
        self.ctx = ctx
        self.items: list[tuple[int, int, str, Any]] = []
        self._seq = itertools.count()
        self._ch: dict[str, Any] = {}

    def add(self, t: int, topic: str, msg: Any) -> None:
        self.items.append((int(t), next(self._seq), topic, msg))

    def channel(self, topic: str, msg: Any):
        ch = self._ch.get(topic)
        if ch is not None:
            return ch
        kind = type(msg).__name__
        table = {"SceneUpdate": fch.SceneUpdateChannel, "FrameTransforms": fch.FrameTransformsChannel,
                 "CompressedImage": fch.CompressedImageChannel, "CameraCalibration": fch.CameraCalibrationChannel,
                 "Grid": fch.GridChannel, "LocationFix": fch.LocationFixChannel, "Log": fch.LogChannel}
        if kind in table:
            ch = table[kind](topic, context=self.ctx)
        elif isinstance(msg, dict):
            schema = self.JSON_TEAM if topic.startswith("/team") else self.JSON_TELEMETRY
            ch = foxglove.Channel(topic, schema=schema, context=self.ctx)
        else:
            raise TypeError(f"no channel for {kind} on {topic}")
        self._ch[topic] = ch
        return ch

    def raw_channel(self, topic: str, schema) -> Any:
        ch = self._ch.get(topic)
        if ch is None:
            sch = foxglove.Schema(name=schema.name, encoding=schema.encoding, data=bytes(schema.data)) \
                if schema is not None else None
            ch = foxglove.Channel(topic, schema=sch, message_encoding="cdr", context=self.ctx)
            self._ch[topic] = ch
        return ch


def build(run_dir: Path, out_path: Path, *, images: bool, jpeg_quality: int, image_hz: float, raw: bool,
          static_period_s: float) -> dict:
    scenario_p = next((p for p in [run_dir / "scenario.json", *sorted(run_dir.glob("robot_*/scenario.json")),
                                   REPO / "stacks/mtl_search/config/scenario.json"] if p.is_file()), None)
    if scenario_p is None:
        raise SystemExit(f"no scenario.json in {run_dir} (or the stack config)")
    scenario = json.loads(scenario_p.read_text())
    gt_p = next((p for p in [run_dir / "ground_truth.json", REPO / "stacks/mtl_search/config/ground_truth.json"]
                 if p.is_file()), None)
    ground_truth = json.loads(gt_p.read_text()) if gt_p else {"targets": []}
    agents = scenario["team"]["agents"]
    homes = {a["name"]: (float(a["home_ned"][1]), float(a["home_ned"][0]), 0.0) for a in agents}
    names = sorted({a["name"] for a in agents} | {p.name for p in run_dir.glob("robot_*") if p.is_dir()})

    robots: list[Robot] = []
    t_base = 1_000 * NS
    for i, name in enumerate(names):
        r = Robot(name, i, ROBOT_RGB[i % len(ROBOT_RGB)], home=homes.get(name, (0.0, 0.0, 0.0)))
        rdir = run_dir / name
        files = _bag_files(rdir)
        _bag_files_cache[name] = files
        if files:
            load_bag(r, files)
        elif (rdir / "telemetry.csv").is_file():
            load_telemetry(r, read_telemetry_csv(rdir / "telemetry.csv"), t_base)
        load_track_json(r, rdir / "track.json")
        if r.odom_t or r.plan_path:
            robots.append(r)
        print(f"[mtl_foxglove] {name}: {r.source}, {len(r.odom_t)} odometry, {len(r.gs_t)} gimbal, "
              f"{len(r.st_t)} status, home ({r.home[0]:.1f}, {r.home[1]:.1f}, {r.home[2]:.1f})")
    if not robots:
        raise SystemExit(f"{run_dir}: no robot data (bags or telemetry.csv)")
    if len({r.source for r in robots if r.odom_t}) > 1:
        print("[mtl_foxglove] WARNING: robots mix bag and telemetry sources; their clocks are not aligned")

    times = [t for r in robots for t in (r.odom_t[:1] + r.odom_t[-1:])]
    T0, T1 = (min(times), max(times)) if times else (t_base, t_base + NS)

    sim_g = scenario.get("airstack", {}).get("sim_gimbal", {})
    mount = tuple(float(v) for v in sim_g.get("mount_offset_m", [0.10, 0.0, -0.08]))
    fov = math.radians(float(scenario["sensor"]["fov_deg"]))
    det = scenario["sensor"]["detection"]
    thr = float(det.get("threshold", 0.9))

    ctx = foxglove.Context()
    out = Out(ctx)

    # ---------------------------------------------------------------- scoring
    rows_by_agent = {}
    for r in robots:
        rows = []
        for k, t in enumerate(r.odom_t[::2] if r.source == "bag" and len(r.odom_t) > 4000 else r.odom_t):
            idx = bisect.bisect_right(r.odom_t, t) - 1
            pos_m, q, vel = r.odom[idx]
            st = hold(r.st_t, r.st, t)
            if r.st_t and (st is None or st["state"] not in ACTIVE):
                continue
            g = hold(r.gs_t, r.gs, t, max_age_ns=NS // 2)
            c = hold(r.gc_t, r.gc, t)
            pw = r.world(pos_m)
            rows.append({"t": t / NS, "x_world": pw[0], "y_world": pw[1], "z_world": pw[2],
                         "meas_pitch": g[1] if g else None, "meas_yaw": g[2] if g else None,
                         "gimbal_measured": 1 if g else 0,
                         "cmd_pitch": c[1] if c else None, "cmd_yaw": c[2] if c else None,
                         "xte_m": st["xte_m"] if st else None})
        if rows:
            rows_by_agent[r.name] = rows
    scorer = None
    t_score0 = 0
    if rows_by_agent:
        scorer, _per, _meas = score_agents(rows_by_agent, scenario, ground_truth)
        t_score0 = int(min(rows[0]["t"] for rows in rows_by_agent.values()) * NS)
    tgt_xy = targets_world(ground_truth)

    # ---------------------------------------------------------------- statics
    area = scenario["mission"]["area"]
    half = float(area["size_m"]) / 2.0
    cn, ce = (float(v) for v in area.get("center_ned", (0.0, 0.0)))
    x0, x1, y0, y1 = ce - half, ce + half, cn - half, cn + half
    cells, masses = cells_world(scenario)
    cell = float(scenario["mapping"]["target_cell_size_m"])
    mmax = max(masses) if masses else 1.0

    grid_msg = None
    bumps = scenario.get("airstack", {}).get("belief", {}).get("bumps", [])
    if bumps:
        res = 4.0
        xs = np.arange(x0 + res / 2, x1, res)
        ys = np.arange(y0 + res / 2, y1, res)
        X, Y = np.meshgrid(xs, ys)
        B = np.zeros_like(X)
        for b in bumps:
            B += float(b["amplitude"]) * np.exp(-0.5 * ((Y - b["n"]) / b["sigma_n"]) ** 2
                                                - 0.5 * ((X - b["e"]) / b["sigma_e"]) ** 2)
        bel = scenario["airstack"]["belief"]
        B = np.minimum(B, float(bel.get("belief_cap", 0.85)))
        if float(bel.get("base_uncertainty", 0.0)) > 0:
            B = np.maximum(B, float(bel["base_uncertainty"]))
        grid_bytes = B.astype("<f4").tobytes()
        grid_args = dict(frame_id="world", pose=pose(x0, y0, 0.02), column_count=len(xs),
                         cell_size=fm.Vector2(x=res, y=res), row_stride=4 * len(xs), cell_stride=4,
                         fields=[fm.PackedElementField(name="belief", offset=0,
                                                       type=fm.PackedElementFieldNumericType.Float32)],
                         data=grid_bytes)
        grid_msg = grid_args

    def static_frame(t):
        tfs = []
        for r in robots:
            tfs.append(fm.FrameTransform(timestamp=ts(t), parent_frame_id="world", child_frame_id=r.frame("map"),
                                         translation=fm.Vector3(x=r.home[0], y=r.home[1], z=r.home[2]),
                                         rotation=fm.Quaternion(x=0, y=0, z=0, w=1)))
            q = OPTICAL_FROM_GIMBAL
            tfs.append(fm.FrameTransform(timestamp=ts(t), parent_frame_id=r.frame("gimbal"),
                                         child_frame_id=r.frame("camera_optical"),
                                         translation=fm.Vector3(x=0, y=0, z=0),
                                         rotation=fm.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
        out.add(t, "/tf", fm.FrameTransforms(transforms=tfs))

        # area: boundary, valid cells (shade = mass), homes
        ents = [entity(t, "world", "boundary", lines=[line([(x0, y0, 0.2), (x1, y0, 0.2), (x1, y1, 0.2), (x0, y1, 0.2)],
                                                          (1.0, 0.85, 0.1), 1.0, 0.8, fm.LinePrimitiveLineType.LineLoop)])]
        cubes = [fm.CubePrimitive(pose=pose(cx, cy, 0.06), size=fm.Vector3(x=cell * 0.92, y=cell * 0.92, z=0.08),
                                  color=col((0.95, 0.95, 0.95), 0.08 + 0.25 * m / mmax)) for (cx, cy), m in zip(cells, masses)]
        ents.append(entity(t, "world", "valid_cells", cubes=cubes))
        for r in robots:
            ents.append(entity(t, "world", f"{r.name}_home",
                               cubes=[fm.CubePrimitive(pose=pose(r.home[0], r.home[1], 0.1),
                                                       size=fm.Vector3(x=2, y=2, z=0.2), color=col(r.rgb, 0.9))],
                               texts=[fm.TextPrimitive(pose=pose(r.home[0], r.home[1], 3.0), billboard=True,
                                                       font_size=14, scale_invariant=True, color=col(r.rgb),
                                                       text=f"{r.name} home")]))
        out.add(t, "/world/area", fm.SceneUpdate(entities=ents))
        if grid_msg:
            out.add(t, "/world/belief", fm.Grid(timestamp=ts(t), **grid_msg))
        for r in robots:
            # planned track + planned boresight ground track (world)
            ents = []
            if r.plan_path:
                ents.append(entity(t, "world", f"{r.name}_plan", lines=[
                    line([r.world(p) for p in r.plan_path[::2]], r.rgb, 0.45, 0.25),
                    line([(p[0] + r.home[0], p[1] + r.home[1], 0.12) for p in r.plan_bore[::2]], r.rgb, 0.35, 0.15)]))
            out.add(t, f"/{r.name}/plan", fm.SceneUpdate(entities=ents))
            # drone model (frame-locked to base_link): X arms, rotors, body, heading arrow
            arm = 0.32
            arms = [(arm, arm, 0.0), (-arm, -arm, 0.0), (arm, -arm, 0.0), (-arm, arm, 0.0)]
            out.add(t, f"/{r.name}/model", fm.SceneUpdate(entities=[entity(
                t, r.frame("base_link"), f"{r.name}_model", frame_locked=True,
                lines=[line(arms, (0.15, 0.15, 0.15), 1.0, 0.05, fm.LinePrimitiveLineType.LineList)],
                cubes=[fm.CubePrimitive(pose=pose(0, 0, 0), size=fm.Vector3(x=0.28, y=0.18, z=0.1), color=col(r.rgb))],
                cylinders=[fm.CylinderPrimitive(pose=pose(*a), size=fm.Vector3(x=0.3, y=0.3, z=0.02),
                                                bottom_scale=1, top_scale=1, color=col(r.rgb, 0.55)) for a in arms],
                arrows=[fm.ArrowPrimitive(pose=pose(0.1, 0, 0), shaft_length=0.5, shaft_diameter=0.05,
                                          head_length=0.15, head_diameter=0.12, color=col(r.rgb))],
                texts=[fm.TextPrimitive(pose=pose(0, 0, 1.2), billboard=True, font_size=13, scale_invariant=True,
                                        color=col(r.rgb), text=r.name)])]))
            # camera frustum (frame-locked to the optical frame: z forward, x right, y down)
            ci = r.caminfo
            w, h = (ci["width"], ci["height"]) if ci else (int(sim_g.get("width", 640)), int(sim_g.get("height", 480)))
            fx = ci["K"][0] if ci and ci["K"][0] > 0 else (w / 2) / math.tan(fov / 2)
            fy = ci["K"][4] if ci and ci["K"][4] > 0 else fx
            d = 8.0
            cx_, cy_ = (w / 2) / fx * d, (h / 2) / fy * d
            c4 = [(-cx_, -cy_, d), (cx_, -cy_, d), (cx_, cy_, d), (-cx_, cy_, d)]
            segs = []
            for a in c4:
                segs += [(0, 0, 0), a]
            for a, b in zip(c4, c4[1:] + c4[:1]):
                segs += [a, b]
            out.add(t, f"/{r.name}/frustum", fm.SceneUpdate(entities=[entity(
                t, r.frame("camera_optical"), f"{r.name}_frustum", frame_locked=True,
                lines=[line(segs, r.rgb, 0.9, 0.04, fm.LinePrimitiveLineType.LineList)])]))

    t = T0
    while t <= T1 + NS:
        static_frame(t)
        t += int(static_period_s * NS)

    # ---------------------------------------------------------------- per robot, timed
    for r in robots:
        if not r.odom_t:
            continue
        last_tf = -10 ** 18
        last_sensor = -10 ** 18
        last_trail = -10 ** 18
        last_tel = -10 ** 18
        last_gps = -10 ** 18
        trail: list[tuple] = []
        prev_state = None
        for k, t in enumerate(r.odom_t):
            pos_m, q, vel = r.odom[k]
            pw = r.world(pos_m)
            if not trail or math.dist(trail[-1], pw) > 0.5:
                trail.append(pw)
            g = hold(r.gs_t, r.gs, t, max_age_ns=NS // 2)
            c = hold(r.gc_t, r.gc, t)
            look = g or c
            st = hold(r.st_t, r.st, t)
            if t - last_tf >= NS // 30:
                last_tf = t
                tfs = [fm.FrameTransform(timestamp=ts(t), parent_frame_id=r.frame("map"),
                                         child_frame_id=r.frame("base_link"),
                                         translation=fm.Vector3(x=pos_m[0], y=pos_m[1], z=pos_m[2]),
                                         rotation=fm.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))]
                qg = q_euler_zyx(*look) if look else q_euler_zyx(0.0, math.radians(60), yaw_of(q))
                qr = q_norm(q_mul(q_conj(q), qg))
                tfs.append(fm.FrameTransform(timestamp=ts(t), parent_frame_id=r.frame("base_link"),
                                             child_frame_id=r.frame("gimbal"),
                                             translation=fm.Vector3(x=mount[0], y=mount[1], z=mount[2]),
                                             rotation=fm.Quaternion(x=qr[0], y=qr[1], z=qr[2], w=qr[3])))
                out.add(t, "/tf", fm.FrameTransforms(transforms=tfs))
            cam = tuple(a + b for a, b in zip(pw, q_rot(q, mount)))
            bore = boresight_ground_point(cam, look[1], look[2], 0.0) if look else None
            speed = math.hypot(vel[0], vel[1])
            state = st["state"] if st else ""
            if t - last_sensor >= NS // 5:
                last_sensor = t
                ents = []
                label = f"{r.name}  {state or 'IDLE'}"
                if st and st["total_m"] > 0:
                    label += f"  {100 * st['progress_m'] / st['total_m']:.0f}%"
                label += f"  {speed:.1f} m/s  {pw[2]:.0f} m"
                prims: dict[str, list] = {"texts": [fm.TextPrimitive(pose=pose(pw[0], pw[1], pw[2] + 3.0), billboard=True,
                                                                    font_size=13, scale_invariant=True,
                                                                    color=col((1, 1, 1)), text=label)],
                                          "lines": [], "spheres": []}
                if bore:
                    gx, gy, slant = bore
                    rad = footprint_radius(slant, fov)
                    circ = [(gx + rad * math.cos(a), gy + rad * math.sin(a), 0.25)
                            for a in (2 * math.pi * i / 48 for i in range(48))]
                    in_range = slant <= float(det.get("beta", 61.0))
                    fp_rgb = r.rgb if in_range else (0.55, 0.55, 0.55)
                    prims["lines"] += [line(circ, fp_rgb, 1.0, 0.25, fm.LinePrimitiveLineType.LineLoop),
                                       line([cam, (gx, gy, 0.25)], fp_rgb, 0.8, 0.08)]
                    prims["texts"].append(fm.TextPrimitive(pose=pose(gx, gy, 1.0), billboard=True, font_size=11,
                                                           scale_invariant=True, color=col(fp_rgb),
                                                           text=f"slant {slant:.0f} m" + ("" if in_range else " (out of range)")))
                if st and state in ACTIVE:
                    aim_w = r.world(st["aim"])
                    car_w = r.world(st["carrot"])
                    prims["spheres"] += [fm.SpherePrimitive(pose=pose(aim_w[0], aim_w[1], 0.4),
                                                            size=fm.Vector3(x=1.2, y=1.2, z=1.2), color=col((1, 1, 1), 0.9)),
                                         fm.SpherePrimitive(pose=pose(*car_w), size=fm.Vector3(x=0.8, y=0.8, z=0.8),
                                                            color=col(r.rgb, 0.9))]
                    prims["lines"].append(line([pw, car_w], r.rgb, 0.6, 0.05))
                ents.append(entity(t, "world", f"{r.name}_sensor", **prims))
                out.add(t, f"/{r.name}/sensor", fm.SceneUpdate(entities=ents))
            if t - last_trail >= NS // 2:
                last_trail = t
                out.add(t, f"/{r.name}/trail", fm.SceneUpdate(entities=[entity(
                    t, "world", f"{r.name}_trail", lines=[line(trail, r.rgb, 1.0, 0.35)] if len(trail) > 1 else [])]))
            if t - last_tel >= NS // 10:
                last_tel = t
                tel = {"state": state, "speed_mps": speed, "altitude_m": pw[2], "vz_mps": vel[2],
                       "heading_deg": math.degrees(yaw_of(q)), "x_world": pw[0], "y_world": pw[1]}
                if st:
                    tel.update({"xte_m": st["xte_m"], "progress_m": st["progress_m"], "remaining_m": st["remaining_m"],
                                "progress_pct": 100.0 * st["progress_m"] / st["total_m"] if st["total_m"] > 0 else 0.0})
                if c:
                    tel.update({"cmd_roll_deg": math.degrees(c[0]), "cmd_pitch_deg": math.degrees(c[1]),
                                "cmd_yaw_deg": math.degrees(c[2])})
                if g:
                    tel.update({"meas_roll_deg": math.degrees(g[0]), "meas_pitch_deg": math.degrees(g[1]),
                                "meas_yaw_deg": math.degrees(g[2])})
                if g and c:
                    dy = (c[2] - g[2] + math.pi) % (2 * math.pi) - math.pi
                    tel["gimbal_tracking_error_deg"] = math.degrees(math.hypot(c[1] - g[1], dy))
                if bore:
                    tel.update({"slant_m": bore[2], "footprint_radius_m": footprint_radius(bore[2], fov)})
                    if st and state in ACTIVE:
                        aw = r.world(st["aim"])
                        tel["pointing_error_m"] = math.hypot(bore[0] - aw[0], bore[1] - aw[1])
                out.add(t, f"/{r.name}/telemetry", tel)
            if state != prev_state:
                if prev_state is not None or state:
                    out.add(t, "/events", fm.Log(timestamp=ts(t), level=fm.LogLevel.Info, name=r.name,
                                                 message=f"{r.name}: {prev_state or 'START'} -> {state or 'IDLE'}"))
                prev_state = state
        for k, t in enumerate(r.gps_t):
            if t - last_gps >= NS // 2:
                last_gps = t
                lat, lon, alt = r.gps[k]
                out.add(t, f"/{r.name}/gps", fm.LocationFix(timestamp=ts(t), frame_id=r.frame("base_link"),
                                                            latitude=lat, longitude=lon, altitude=alt,
                                                            color=col(r.rgb)))

    # ---------------------------------------------------------------- targets + team metrics
    steps = list(zip(scorer.t, scorer.p_det_curve, scorer.detected_count, scorer.mass_curve,
                     scorer.distance_curve)) if scorer else []
    det_time = {tg.index: tg.detection_time_s for tg in scorer.targets} if scorer else {}
    finder = {tg.index: tg.detected_by for tg in scorer.targets} if scorer else {}
    total_mass = scorer.total_mass if scorer else 1.0
    last_emit = -10 ** 18
    prev_detected = 0

    def target_update(t_abs, pdets, t_rel):
        ents = []
        for i, (x, y, _z) in enumerate(tgt_xy):
            p = pdets[i] if pdets else 0.0
            found = det_time.get(i) is not None and t_rel is not None and t_rel >= det_time[i] - 1e-9
            rgb = pdet_rgb(p, thr)
            txt = f"T{i:02d}  P={p:.2f}"
            if found:
                txt += f"  found {det_time[i]:.1f} s ({finder.get(i) or '?'})"
            prims = {"cylinders": [fm.CylinderPrimitive(pose=pose(x, y, 0.6), size=fm.Vector3(x=1.6, y=1.6, z=1.2),
                                                       bottom_scale=1, top_scale=1, color=col(rgb, 0.95))],
                     "texts": [fm.TextPrimitive(pose=pose(x, y, 3.0), billboard=True, font_size=12,
                                                scale_invariant=True, color=col(rgb), text=txt)]}
            if found:
                ring = [(x + 3.0 * math.cos(a), y + 3.0 * math.sin(a), 0.3) for a in (2 * math.pi * k / 32 for k in range(32))]
                prims["lines"] = [line(ring, (0.05, 0.64, 0.05), 1.0, 0.3, fm.LinePrimitiveLineType.LineLoop)]
            ents.append(entity(t_abs, "world", f"target_{i:02d}", **prims))
        out.add(t_abs, "/world/targets", fm.SceneUpdate(entities=ents))

    target_update(T0, [0.0] * len(tgt_xy), None)
    for t_rel, pdets, found_n, mass, dist in steps:
        t_abs = t_score0 + int(t_rel * NS)
        if found_n > prev_detected:
            for tg in scorer.targets:
                if tg.detection_time_s is not None and abs(tg.detection_time_s - t_rel) < 1e-9:
                    out.add(t_abs, "/events", fm.Log(timestamp=ts(t_abs), level=fm.LogLevel.Warning, name="detector",
                                                     message=f"target T{tg.index:02d} FOUND by {tg.detected_by} "
                                                             f"at {t_rel:.1f} s (P_det {tg.p_det:.3f})"))
        if t_abs - last_emit >= NS // 2 or found_n > prev_detected:
            last_emit = t_abs
            target_update(t_abs, pdets, t_rel)
            out.add(t_abs, "/team/metrics", {"t_s": t_rel, "targets_found": found_n, "targets_total": len(tgt_xy),
                                             "belief_mass_covered": mass,
                                             "belief_mass_fraction": mass / total_mass if total_mass else 0.0,
                                             "distance_m": dist, "p_det": list(pdets)})
        prev_detected = found_n
    if steps:
        target_update(max(T1, t_score0 + int(steps[-1][0] * NS)), steps[-1][1], steps[-1][0])

    # ---------------------------------------------------------------- write, merging the bag streams
    out.items.sort(key=lambda it: (it[0], it[1]))
    streams: list[Iterator] = [iter(out.items)]
    seq = itertools.count(10 ** 12)
    min_dt = int(NS / image_hz) if image_hz > 0 else 0
    for r in robots:
        if images and _bag_files_cache.get(r.name):
            def img_stream(r=r):
                ci = r.caminfo
                for t, jpeg, (w, h) in iter_images(r, jpeg_quality, min_dt):
                    r.n_images += 1
                    yield (t, next(seq), f"/{r.name}/camera/image",
                           fm.CompressedImage(timestamp=ts(t), frame_id=r.frame("camera_optical"), data=jpeg, format="jpeg"))
                    if ci:
                        yield (t, next(seq), f"/{r.name}/camera/calibration", fm.CameraCalibration(
                            timestamp=ts(t), frame_id=r.frame("camera_optical"), width=w, height=h,
                            distortion_model=ci["distortion_model"] or "plumb_bob", D=ci["D"], K=ci["K"], R=ci["R"],
                            P=ci["P"]))
            streams.append(img_stream())
        if raw and _bag_files_cache.get(r.name):
            def raw_stream(r=r):
                prefix = f"/{r.name}/"
                for t, topic, schema, data in iter_raw(r, {f"/{r.name}/gimbal/rgb"}):
                    # robot topics keep their name under /raw; shared ones (/tf, /tf_static)
                    # get the robot prefix so the three robots do not collide
                    name = "/raw" + (topic if topic.startswith(prefix) else f"/{r.name}{topic}")
                    yield (t, next(seq), ("RAW", name, schema), data)
            streams.append(raw_stream())

    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = foxglove.open_mcap(str(out_path), allow_overwrite=True, context=ctx,
                                writer_options=MCAPWriteOptions(compression=MCAPCompression.Zstd))
    n = 0
    try:
        for t, _s, topic, msg in heapq.merge(*streams, key=lambda it: (it[0], it[1])):
            if isinstance(topic, tuple):
                _, rtopic, schema = topic
                out.raw_channel(rtopic, schema).log(bytes(msg), log_time=t)
            else:
                out.channel(topic, msg).log(msg, log_time=t)
            n += 1
    finally:
        writer.close()
    summary = scorer.summary() if scorer else {}
    return {"messages": n, "robots": [r.name for r in robots], "images": {r.name: r.n_images for r in robots},
            "t0": T0, "t1": T1, "summary": summary}


# ========================================================================= layout
def write_layout(path: Path, robots: list[str]) -> None:
    """A Foxglove layout (Layouts -> Import from file) wired to the topics above."""
    topics3d = {"/world/area": {"visible": True},
                "/world/belief": {"visible": True, "colorField": "belief", "colorMode": "colormap",
                                  "colorMap": "turbo", "minValue": 0, "maxValue": 0.85},
                "/world/targets": {"visible": True}}
    for r in robots:
        for s in ("model", "frustum", "plan", "trail", "sensor"):
            topics3d[f"/{r}/{s}"] = {"visible": True}
        topics3d[f"/{r}/camera/image"] = {"visible": False}
    config = {
        "3D!team": {"followTf": "world", "followMode": "follow-none", "topics": topics3d,
                    "scene": {"transforms": {"showLabel": False}},
                    "cameraState": {"distance": 420, "perspective": True, "phi": 45, "thetaOffset": 0,
                                    "targetOffset": [0, 0, 0], "target": [0, 0, 0], "targetOrientation": [0, 0, 0, 1],
                                    "fovy": 45, "near": 0.5, "far": 5000}},
        "Plot!targets": {"title": "Team: targets found / belief mass covered", "paths": [
            {"value": "/team/metrics.targets_found", "enabled": True, "timestampMethod": "receiveTime"},
            {"value": "/team/metrics.belief_mass_fraction", "enabled": True, "timestampMethod": "receiveTime"}],
            "showLegend": True},
        "Plot!speed": {"title": "Speed [m/s]", "paths": [
            {"value": f"/{r}/telemetry.speed_mps", "enabled": True, "timestampMethod": "receiveTime", "label": r}
            for r in robots], "showLegend": True},
        "Plot!xte": {"title": "Cross-track error [m]", "paths": [
            {"value": f"/{r}/telemetry.xte_m", "enabled": True, "timestampMethod": "receiveTime", "label": r}
            for r in robots], "showLegend": True},
        "Plot!gimbal": {"title": "Gimbal pitch cmd vs measured [deg]", "paths": [
            p for r in robots for p in (
                {"value": f"/{r}/telemetry.cmd_pitch_deg", "enabled": True, "timestampMethod": "receiveTime",
                 "label": f"{r} cmd"},
                {"value": f"/{r}/telemetry.meas_pitch_deg", "enabled": True, "timestampMethod": "receiveTime",
                 "label": f"{r} meas"})], "showLegend": True},
        "map!gps": {"layer": "map", "topicColors": {}},
        "RosOut!events": {"topicToRender": "/events"},
        "RawMessages!raw": {"topicPath": "/team/metrics"},
    }
    img_ids = []
    for r in robots:
        pid = f"Image!{r}"
        img_ids.append(pid)
        config[pid] = {"imageMode": {"imageTopic": f"/{r}/camera/image", "calibrationTopic": f"/{r}/camera/calibration"},
                       "cameraState": {}, "topics": {}}

    def row(ids, pct=None):
        if len(ids) == 1:
            return ids[0]
        node = ids[0]
        for i, nxt in enumerate(ids[1:], start=1):
            node = {"first": node, "second": nxt, "direction": "row", "splitPercentage": 100 * i / (i + 1)}
        return node

    def col_(a, b, pct):
        return {"first": a, "second": b, "direction": "column", "splitPercentage": pct}

    layout = col_(
        {"first": "3D!team", "second": col_(row(img_ids), "map!gps", 60), "direction": "row", "splitPercentage": 62},
        row(["Plot!targets", "Plot!speed", "Plot!xte", "Plot!gimbal", "RosOut!events"]), 66)
    path.write_text(json.dumps({"configById": config, "globalVariables": {}, "userNodes": {},
                                "playbackConfig": {"speed": 1}, "layout": layout}, indent=1) + "\n")


# ========================================================================== main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--run-dir", type=Path, default=REPO / "runs" / "latest")
    ap.add_argument("--out", type=Path, default=None, help="output .mcap (default: <run>/foxglove/<run>.mcap)")
    ap.add_argument("--no-images", action="store_true", help="skip the camera streams")
    ap.add_argument("--no-raw", action="store_true", help="do not copy the raw ROS topics under /raw/...")
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--image-hz", type=float, default=0.0, help="max camera frame rate per robot (0 = all frames)")
    ap.add_argument("--static-period", type=float, default=1.0,
                    help="re-publish period of static scene layers [s] (lets Foxglove seek anywhere)")
    a = ap.parse_args()
    run_dir = a.run_dir.resolve()
    if not run_dir.is_dir():
        print(f"no such run folder: {run_dir}", file=sys.stderr)
        return 2
    out = a.out or run_dir / "foxglove" / f"{run_dir.name}.mcap"
    res = build(run_dir, out, images=not a.no_images, jpeg_quality=a.jpeg_quality, image_hz=a.image_hz,
                raw=not a.no_raw, static_period_s=a.static_period)
    layout = out.parent / "mtl_layout.json"
    write_layout(layout, res["robots"])
    s = res["summary"]
    print(f"[mtl_foxglove] wrote {out} ({out.stat().st_size / 1e6:.1f} MB, {res['messages']} messages, "
          f"{(res['t1'] - res['t0']) / NS:.0f} s; camera frames {res['images']})")
    if s:
        print(f"[mtl_foxglove] team: {s['targets_detected']}/{s['targets_total']} targets, "
              f"{100 * s['belief_mass_fraction']:.1f} % belief mass covered, {s['total_path_length_m']:.0f} m flown")
    print(f"[mtl_foxglove] open the .mcap in Foxglove and import the layout {layout}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
