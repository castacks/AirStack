#!/usr/bin/env python3
"""Compare a frozen_suburban_8robot benchmark run's DETECTIONS against GROUND TRUTH.

For every `iter_NNN__<env>__<method>` cell in a run directory this prints, per
cell: how many "person" proposals the detector made and how many passed the
confidence gate (summed across the 8 robots, from each robot's `planner.log`
FINAL detector summary), how many target *instances* the planner tracked and
*visited* (from the last `targets:` line per robot — also summed, so the same
person seen by two robots counts twice; this is called out explicitly, never
silently presented as a unique-people count), and — the reason this script
exists — how many ground-truth people were actually within reach of the fleet
at all: `gt_in_covered_area`, a person is counted if any robot's world-frame
trajectory ever came within `--radius` metres of them. That is the honest
denominator: `recall_of_covered = instances / gt_in_covered_area` says how the
detector did on people the fleet could plausibly have seen; `recall_of_all =
instances / gt_people` is the (much harsher, coverage-conflated) number against
every person in the scene, including ones nobody flew near.

Frame handling (the part most likely to go wrong): odometry
(`/robot_N/odometry_conversion/odometry`) is in that robot's own LOCAL map
frame — each robot's frame origin can be hundreds of metres from another's.
`/gcs/robot_N/map_origin` (published once, latched) is the translation that
places it in world/ENU coordinates matching the ground-truth annotations. This
script decodes both directly from the mcap bag and adds them; get this wrong
and you get plausible-looking nonsense (see build-scenes-on-osmo /
frozen-run-baseline-state memory notes for why this class of bug is easy to
ship unnoticed).

conavgpt2_team cells run their planner on `offboard-compute`, whose logs this
mission does NOT collect — there is no planner.log for them. This script does
NOT report 0 for those cells; it reports "no planner log" and falls back to
counting `/robot_N/search/detection_image` messages (published only on a
gate-passing tick) from the bag's mcap statistics index, IF that topic exists
in the bag. As of this writing it often does not (the DDS relay between
offboard-compute and the GCS bag recorder may not carry it) — when absent, the
script says so rather than inventing a number.

No numpy/mcap_ros2/rclpy dependency: this must run against the OSMO pod's bare
`python3`, which has only the `mcap` package installed. Odometry and
map_origin (nav_msgs/Odometry, geometry_msgs/PointStamped) are decoded with a
small hand-rolled CDR reader (see CDRReader below) verified against real bag
bytes. Message COUNTS come from the mcap summary/statistics index (fast, no
decode); only odometry and map_origin messages are ever actually decoded, and
messages are streamed (never a whole bag loaded into memory).

Usage:
  # on the pod (no file needs to land on disk there):
  ssh airstack-osmo 'python3 - --json /tmp/out.json' < detections_vs_gt.py

  # locally, against a results tree fetched from the pod:
  python3 detections_vs_gt.py --results-root /path/to/run_dir \
      --annotations /path/to/raven_nav/annotations --json out.json

Defaults point at the frozen_suburban_8robot 2026-08-31_11-11-42 pod run and
the repo's own annotations directory; override with --results-root /
--annotations for anything else.
"""
import argparse
import glob
import json
import os
import re
import struct
import sys
from collections import defaultdict

# ----------------------------------------------------------------------------
# Defaults
# ----------------------------------------------------------------------------
DEFAULT_RESULTS_ROOT = (
    "/root/AirStack/osmo/results/frozen_suburban_8robot/2026-08-31_11-11-42"
)
DEFAULT_ANNOTATIONS_POD = (
    "/root/AirStack/robot/ros_ws/src/global/planners/raven_nav/annotations"
)


def _default_annotations():
    """Prefer the annotations dir relative to THIS repo checkout (works both
    locally and if this script is ever committed onto the pod at the mirrored
    osmo/analysis/ path); fall back to the known pod path when the script has
    no on-disk location at all (e.g. run via `python3 -` over ssh stdin, which
    is the documented read-only way to execute it against the pod)."""
    try:
        here = os.path.dirname(os.path.abspath(__file__))
    except NameError:
        return DEFAULT_ANNOTATIONS_POD
    cand = os.path.normpath(os.path.join(
        here, "..", "..", "robot", "ros_ws", "src", "global",
        "planners", "raven_nav", "annotations"))
    return cand if os.path.isdir(cand) else DEFAULT_ANNOTATIONS_POD


# ----------------------------------------------------------------------------
# Minimal CDR (ROS 2 default RMW wire format) decoder.
#
# Alignment is relative to the START OF THE PAYLOAD, i.e. the 4-byte CDR
# encapsulation header (e.g. 00 01 00 00 for little-endian, no options) is
# EXCLUDED from the modulo arithmetic. This was verified empirically against
# real bag bytes on the pod: a geometry_msgs/PointStamped with frame_id "map"
# (a 4-byte string field: length-prefix 4 + "map\0") is exactly 44 bytes on
# the wire. Treating the header as part of the alignment base would insert 4
# bytes of padding before the point's doubles and overrun the buffer by
# exactly that much; excluding it lands precisely on the real 44-byte length.
# Getting this backwards is the kind of bug that decodes without raising and
# produces silently-wrong positions -- confirmed against the real encoding
# before trusting it for anything below.
# ----------------------------------------------------------------------------
class CDRReader:
    __slots__ = ("data", "pos", "base")

    def __init__(self, data):
        self.data = data
        self.pos = 0       # relative to the end of the 4-byte header
        self.base = 4       # absolute offset of relative position 0

    def _align(self, n):
        r = self.pos % n
        if r:
            self.pos += n - r

    def i32(self):
        self._align(4)
        v = struct.unpack_from("<i", self.data, self.base + self.pos)[0]
        self.pos += 4
        return v

    def u32(self):
        self._align(4)
        v = struct.unpack_from("<I", self.data, self.base + self.pos)[0]
        self.pos += 4
        return v

    def f64(self):
        self._align(8)
        v = struct.unpack_from("<d", self.data, self.base + self.pos)[0]
        self.pos += 8
        return v

    def string(self):
        self._align(4)
        n = self.u32()
        s = self.data[self.base + self.pos: self.base + self.pos + n - 1]
        self.pos += n
        return s.decode("utf-8", "replace")


def decode_point_stamped_xyz(data):
    """geometry_msgs/msg/PointStamped -> (x, y, z)."""
    c = CDRReader(data)
    c.i32(); c.u32()   # header.stamp.{sec,nanosec} -- unused
    c.string()          # header.frame_id -- unused
    return c.f64(), c.f64(), c.f64()


def decode_odometry_xyz(data):
    """nav_msgs/msg/Odometry -> pose.pose.position (x, y, z). Deliberately
    stops right after position: orientation, both covariance matrices, and
    twist are never parsed because nothing here needs them."""
    c = CDRReader(data)
    c.i32(); c.u32()   # header.stamp
    c.string()          # header.frame_id
    c.string()          # child_frame_id
    return c.f64(), c.f64(), c.f64()


# ----------------------------------------------------------------------------
# planner.log parsing
# ----------------------------------------------------------------------------
FINAL_SUMMARY_RE = re.compile(
    r'\[robot_(\d+)\] detector summary \(FINAL\): (\d+) ticks '
    r'\| "person" proposed on (\d+) \| passed gate ([\d.]+) on (\d+) '
    r'\| run max ([\d.]+)'
)
TARGETS_RE = re.compile(
    r'targets: object_pcd (\d+) pts \| found_goal (True|False) '
    r'\| instances (\d+) \| visited (\d+)'
)


def parse_planner_log(path):
    """Return {'final': {...} or None, 'targets': {...} or None} by scanning
    the whole log (small text file, a few thousand lines); 'final' is the
    (last, should-be-only) FINAL detector summary line, 'targets' is the LAST
    `targets:` line in the file (the run's final tally for that robot)."""
    final = None
    last_targets = None
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = FINAL_SUMMARY_RE.search(line)
            if m:
                final = {
                    "ticks": int(m.group(2)),
                    "proposed": int(m.group(3)),
                    "gate_threshold": float(m.group(4)),
                    "passed_gate": int(m.group(5)),
                    "run_max": float(m.group(6)),
                }
                continue
            m2 = TARGETS_RE.search(line)
            if m2:
                last_targets = {
                    "object_pcd_pts": int(m2.group(1)),
                    "found_goal": m2.group(2) == "True",
                    "instances": int(m2.group(3)),
                    "visited": int(m2.group(4)),
                }
    return {"final": final, "targets": last_targets}


ROBOT_DESKTOP_RE = re.compile(r"^airstack-robot-desktop-(\d+)$")


def find_robot_ids_from_logs(iter_dir):
    logs_dir = os.path.join(iter_dir, "logs")
    ids = []
    if os.path.isdir(logs_dir):
        for name in os.listdir(logs_dir):
            m = ROBOT_DESKTOP_RE.match(name)
            if m:
                ids.append(int(m.group(1)))
    return sorted(ids)


# ----------------------------------------------------------------------------
# mcap bag reading
# ----------------------------------------------------------------------------
ODOM_RE = re.compile(r"^/robot_(\d+)/odometry_conversion/odometry$")
ORIGIN_RE = re.compile(r"^/gcs/robot_(\d+)/map_origin$")
DETIMG_RE = re.compile(r"^/robot_(\d+)/search/detection_image$")


def find_bag(iter_dir):
    cands = sorted(glob.glob(os.path.join(iter_dir, "bags", "gcs", "*.mcap")))
    return cands[0] if cands else None


def load_trajectories(bag_path, radius, use_3d=False, stride=1):
    """Stream the bag ONCE. Returns:
      world_traj      : {robot_id: [(x,y[,z]), ...]}  world-frame points
      origins         : {robot_id: (x,y,z)}            decoded map_origin
      odom_robots     : sorted list of robot ids with an odometry topic
      missing_origin  : robot ids with odometry but no map_origin in the bag
      detimg_totals   : {robot_id: msg_count} or None if the topic is absent
                        from this bag entirely (fast: statistics index only,
                        no decoding of the images themselves)
    Only odometry and map_origin payloads are ever decoded; everything else is
    read from the mcap summary/statistics index (message counts) or skipped.
    """
    from mcap.reader import make_reader

    with open(bag_path, "rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        if summary is None:
            raise RuntimeError("bag has no summary/statistics index (cannot "
                                "read message counts or seek efficiently)")
        topics = {ch.topic: cid for cid, ch in summary.channels.items()}

        odom_topic_of = {}
        for t in topics:
            m = ODOM_RE.match(t)
            if m:
                odom_topic_of[int(m.group(1))] = t
        origin_topic_of = {}
        for t in topics:
            m = ORIGIN_RE.match(t)
            if m:
                origin_topic_of[int(m.group(1))] = t
        detimg_topic_of = {}
        for t in topics:
            m = DETIMG_RE.match(t)
            if m:
                detimg_topic_of[int(m.group(1))] = t

        # map_origin: one latched message per robot -- decode the first.
        origins = {}
        for rid, t in origin_topic_of.items():
            for _schema, _channel, msg in reader.iter_messages(topics=[t]):
                origins[rid] = decode_point_stamped_xyz(msg.data)
                break

        detimg_totals = None
        if detimg_topic_of:
            stats = summary.statistics
            detimg_totals = {
                rid: (stats.channel_message_counts.get(topics[t], 0)
                      if stats else None)
                for rid, t in detimg_topic_of.items()
            }

        odom_robots = sorted(odom_topic_of)
        missing_origin = sorted(r for r in odom_robots if r not in origins)

        topic_to_robot = {t: r for r, t in odom_topic_of.items()}
        want_topics = [t for r, t in odom_topic_of.items() if r in origins]
        world_traj = defaultdict(list)
        counters = defaultdict(int)
        for _schema, channel, msg in reader.iter_messages(topics=want_topics):
            rid = topic_to_robot[channel.topic]
            counters[rid] += 1
            if stride > 1 and (counters[rid] % stride):
                continue
            x, y, z = decode_odometry_xyz(msg.data)
            ox, oy, oz = origins[rid]
            if use_3d:
                world_traj[rid].append((x + ox, y + oy, z + oz))
            else:
                world_traj[rid].append((x + ox, y + oy))

    return dict(world_traj), origins, odom_robots, missing_origin, detimg_totals


def count_covered(gt_people, all_points, radius, use_3d):
    """Number of GT people with >=1 trajectory point within `radius` metres
    (2D horizontal by default, matching osmo/analyze_visited.py's convention;
    pass use_3d for full 3D distance). Early-exits per GT point."""
    r2 = radius * radius
    covered = 0
    for g in gt_people:
        gx, gy, gz = g
        hit = False
        for p in all_points:
            if use_3d:
                px, py, pz = p
                d2 = (px - gx) ** 2 + (py - gy) ** 2 + (pz - gz) ** 2
            else:
                px, py = p
                d2 = (px - gx) ** 2 + (py - gy) ** 2
            if d2 <= r2:
                hit = True
                break
        covered += hit
    return covered


# ----------------------------------------------------------------------------
# cell discovery / scene resolution
# ----------------------------------------------------------------------------
FAILED_ATTEMPT_RE = re.compile(r"_failed_attempt_\d+$")
ITER_DIR_RE = re.compile(r"^iter_(\d+)__(.+)$")


def resolve_run_dir(root):
    """If `root` itself holds iter_* dirs, use it. Otherwise, if it holds
    timestamped run subdirectories (e.g. a mission results dir with several
    dated runs), descend into the lexically-last one that has iter_* dirs."""
    def has_iters(d):
        try:
            return any(re.match(r"^iter_\d+__", n) for n in os.listdir(d))
        except OSError:
            return False

    if has_iters(root):
        return root
    try:
        subdirs = sorted(
            os.path.join(root, n) for n in os.listdir(root)
            if os.path.isdir(os.path.join(root, n)))
    except OSError:
        return root
    for d in reversed(subdirs):
        if has_iters(d):
            return d
    return root


def find_cells(run_dir):
    """[(iter_num, dir_name, dir_path, env, method), ...] sorted by iter_num.
    Skips *_failed_attempt_N dirs (retries superseded by the final attempt)."""
    cells = []
    for name in sorted(os.listdir(run_dir)):
        path = os.path.join(run_dir, name)
        if not os.path.isdir(path) or FAILED_ATTEMPT_RE.search(name):
            continue
        m = ITER_DIR_RE.match(name)
        if not m:
            continue
        rest = m.group(2)
        parts = rest.split("__")
        if len(parts) == 2:
            env, method = parts
        else:
            env, method = "__".join(parts[:-1]), parts[-1]
        cells.append((int(m.group(1)), name, path, env, method))
    return sorted(cells, key=lambda c: c[0])


def build_annotation_index(annotations_dir):
    """{lowercased scene stem -> real stem}, excluding *_region / *_obstacles
    sidecar files."""
    idx = {}
    for fn in os.listdir(annotations_dir):
        if not fn.endswith(".json"):
            continue
        stem = fn[:-5]
        if stem.endswith("_region") or stem.endswith("_obstacles"):
            continue
        idx[stem.lower()] = stem
    return idx


def scene_key_from_env(env, method):
    suffix = "_" + method
    return env[: -len(suffix)] if env.endswith(suffix) else env


_gt_cache = {}


def load_gt_people(annotations_dir, scene_stem):
    if scene_stem in _gt_cache:
        return _gt_cache[scene_stem]
    with open(os.path.join(annotations_dir, scene_stem + ".json")) as f:
        data = json.load(f)
    people = [tuple(o["bbox_world"]["center_xyz_m"]) for o in data
              if str(o.get("class", "")).lower() == "person"]
    _gt_cache[scene_stem] = people
    return people


# ----------------------------------------------------------------------------
# per-cell processing
# ----------------------------------------------------------------------------
def process_cell(iter_num, name, iter_dir, env, method, annotations_dir,
                  annot_idx, radius, use_3d, stride):
    r = {
        "iter": iter_num, "dir": name, "env": env, "method": method,
        "caveats": [],
    }

    status = "unknown"
    it_json = os.path.join(iter_dir, "iteration.json")
    if os.path.isfile(it_json):
        try:
            with open(it_json) as f:
                status = json.load(f).get("status", "unknown")
        except (OSError, json.JSONDecodeError):
            pass
    r["status"] = status

    scene_key = scene_key_from_env(env, method)
    scene_stem = annot_idx.get(scene_key.lower())
    if scene_stem is None:
        r["error"] = f"could not resolve scene from env={env!r} (tried {scene_key!r})"
        return r
    r["scene"] = scene_stem
    gt = load_gt_people(annotations_dir, scene_stem)
    r["gt_people"] = len(gt)

    bag_path = find_bag(iter_dir)
    if bag_path is None:
        r["error"] = "no bag found under bags/gcs/ (cell not yet complete?)"
        return r

    # --- planner.log (absent entirely for team cells -> "no planner log") ---
    robot_ids = find_robot_ids_from_logs(iter_dir)
    per_final, per_targets = {}, {}
    any_log = False
    for rid in robot_ids:
        p = os.path.join(iter_dir, "logs", f"airstack-robot-desktop-{rid}",
                          "planner.log")
        if not os.path.isfile(p):
            continue
        any_log = True
        parsed = parse_planner_log(p)
        if parsed["final"]:
            per_final[rid] = parsed["final"]
        if parsed["targets"]:
            per_targets[rid] = parsed["targets"]

    if not any_log:
        r["proposed"] = r["passed_gate"] = None
        r["instances"] = r["visited"] = None
        r["caveats"].append(
            "no planner.log on ANY robot for this cell (its planner runs on "
            "offboard-compute, whose logs this mission does not collect) -- "
            "proposed/passed_gate/instances/visited are UNKNOWN, not zero")
    else:
        missing_f = [x for x in robot_ids if x not in per_final]
        if missing_f:
            r["caveats"].append(
                f"robots {missing_f} had no FINAL detector-summary line in "
                f"planner.log (crashed/truncated?) -- proposed/passed_gate "
                f"below only sum robots {sorted(per_final)}")
        missing_t = [x for x in robot_ids if x not in per_targets]
        if missing_t:
            r["caveats"].append(
                f"robots {missing_t} had no targets: line -- instances/visited "
                f"below only sum robots {sorted(per_targets)}")
        r["proposed"] = sum(v["proposed"] for v in per_final.values())
        r["passed_gate"] = sum(v["passed_gate"] for v in per_final.values())
        r["instances"] = sum(v["instances"] for v in per_targets.values())
        r["visited"] = sum(v["visited"] for v in per_targets.values())

    # --- bag: trajectories (all methods) + detection_image (team fallback) ---
    try:
        traj, origins, odom_robots, missing_origin, detimg = load_trajectories(
            bag_path, radius, use_3d=use_3d, stride=stride)
    except Exception as e:
        r["error"] = f"failed to read bag {bag_path}: {e}"
        return r

    if len(odom_robots) != 8:
        r["caveats"].append(
            f"only {len(odom_robots)} robots have an odometry topic in this "
            f"bag (expected 8): {odom_robots}")
    if missing_origin:
        r["caveats"].append(
            f"robots {missing_origin} have odometry but NO map_origin message "
            f"in the bag -- excluded from gt_in_covered_area (cannot place "
            f"their trajectory in the world frame)")

    r["odom_robots"] = odom_robots
    r["covered_robots"] = sorted(traj.keys())

    if not any_log:
        if detimg is not None:
            r["detection_image_msgs"] = sum(v or 0 for v in detimg.values())
            r["caveats"].append(
                "no planner.log -> detections shown as "
                "/robot_N/search/detection_image message COUNT (one per "
                "gate-passing tick across the whole run); NOT the same unit "
                "as passed_gate/instances from a planner.log and not directly "
                "comparable to the other rows")
        else:
            r["detection_image_msgs"] = None
            r["caveats"].append(
                "no planner.log AND no */search/detection_image topic in this "
                "bag -- no detection data could be recovered for this cell at "
                "all (gt_in_covered_area below is still computed from odometry)")

    all_points = [p for pts in traj.values() for p in pts]
    r["gt_in_covered_area"] = count_covered(gt, all_points, radius, use_3d)
    r["radius_m"] = radius

    if r.get("instances") is not None:
        cov = r["gt_in_covered_area"]
        r["recall_of_covered"] = (r["instances"] / cov) if cov else None
        r["recall_of_all"] = (r["instances"] / len(gt)) if gt else None
        if cov and r["instances"] / cov > 1.0 + 1e-9:
            r["caveats"].append(
                "recall_of_covered > 1.0 -- instances is summed across 8 "
                "robots and double-counts the same person seen by more than "
                "one robot; treat as an upper bound, not a unique-people count")
    else:
        r["recall_of_covered"] = r["recall_of_all"] = None

    return r


# ----------------------------------------------------------------------------
# reporting
# ----------------------------------------------------------------------------
def _fmt(v, spec=None):
    if v is None:
        return "-"
    if spec:
        return format(v, spec)
    return str(v)


def _fmt_pct(v):
    return "-" if v is None else f"{v * 100:.0f}%"


def print_report(cells, radius):
    hdr = (f"{'cell':40s} {'status':8s} {'gt':>4s} {'cov':>5s} "
           f"{'prop':>6s} {'gate':>5s} {'inst*':>6s} {'vis*':>5s} "
           f"{'rec/cov':>8s} {'rec/all':>8s}")
    print(f"per-cell results (coverage radius = {radius:g} m; "
          f"'inst*'/'vis*' = instances/visited SUMMED across 8 robots, "
          f"DOUBLE-COUNTS a person seen or visited by more than one robot)\n")
    print(hdr)
    print("-" * len(hdr))
    by_method = defaultdict(lambda: defaultdict(float))
    by_method_n = defaultdict(int)
    for c in cells:
        label = f"{c.get('scene', c['env']):24s} {c['method']:15s}"
        if "error" in c:
            print(f"{label} {'ERROR':8s}  {c['error']}")
            continue
        row = (f"{label} {c['status']:8s} {_fmt(c['gt_people']):>4s} "
               f"{_fmt(c['gt_in_covered_area']):>5s} "
               f"{_fmt(c['proposed']):>6s} {_fmt(c['passed_gate']):>5s} "
               f"{_fmt(c['instances']):>6s} {_fmt(c['visited']):>5s} "
               f"{_fmt_pct(c['recall_of_covered']):>8s} "
               f"{_fmt_pct(c['recall_of_all']):>8s}")
        print(row)
        m = c["method"]
        by_method_n[m] += 1
        for k in ("gt_people", "gt_in_covered_area", "proposed",
                  "passed_gate", "instances", "visited"):
            if c.get(k) is not None:
                by_method[m][k] += c[k]

    print("\ntotals by method (summed across scenes; recall recomputed from "
          "the sums, not averaged per-cell)\n")
    thdr = (f"{'method':16s} {'cells':>5s} {'gt':>5s} {'cov':>5s} "
            f"{'prop':>7s} {'gate':>6s} {'inst*':>7s} {'vis':>5s} "
            f"{'rec/cov':>8s} {'rec/all':>8s}")
    print(thdr)
    print("-" * len(thdr))
    for m in sorted(by_method):
        s = by_method[m]
        inst = s.get("instances")
        cov = s.get("gt_in_covered_area")
        gtp = s.get("gt_people")
        rec_cov = (inst / cov) if inst is not None and cov else None
        rec_all = (inst / gtp) if inst is not None and gtp else None
        print(f"{m:16s} {by_method_n[m]:5d} "
              f"{_fmt(int(s.get('gt_people', 0)) if 'gt_people' in s else None):>5s} "
              f"{_fmt(int(s.get('gt_in_covered_area', 0)) if 'gt_in_covered_area' in s else None):>5s} "
              f"{_fmt(int(s.get('proposed', 0)) if 'proposed' in s else None):>7s} "
              f"{_fmt(int(s.get('passed_gate', 0)) if 'passed_gate' in s else None):>6s} "
              f"{_fmt(int(s.get('instances', 0)) if 'instances' in s else None):>7s} "
              f"{_fmt(int(s.get('visited', 0)) if 'visited' in s else None):>5s} "
              f"{_fmt_pct(rec_cov):>8s} {_fmt_pct(rec_all):>8s}")

    print("\ncaveats\n" + "-" * 7)
    any_caveat = False
    for c in cells:
        if c.get("caveats"):
            any_caveat = True
            print(f"[{c['dir']}]")
            for cv in c["caveats"]:
                print(f"  - {cv}")
    if not any_caveat:
        print("(none)")


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Compare benchmark detections against ground truth.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--results-root", default=DEFAULT_RESULTS_ROOT,
                     help="a run dir full of iter_NNN__<env>__<method> cells, "
                          "or a parent of several timestamped run dirs "
                          f"(default: {DEFAULT_RESULTS_ROOT})")
    ap.add_argument("--annotations", default=None,
                     help="raven_nav annotations dir (default: resolved "
                          "relative to this script's repo, else the pod path)")
    ap.add_argument("--radius", type=float, default=50.0,
                     help="coverage radius in meters (default: 50)")
    ap.add_argument("--3d", dest="use_3d", action="store_true",
                     help="use full 3D distance for coverage (default: "
                          "horizontal XY, matching osmo/analyze_visited.py)")
    ap.add_argument("--traj-stride", type=int, default=1,
                     help="use every Nth odometry message per robot "
                          "(default: 1, i.e. all of them -- a full "
                          "8-robot/~20k-msg bag decodes in ~10s, no need to "
                          "subsample by default)")
    ap.add_argument("--json", default=None,
                     help="also write full machine-readable results here")
    args = ap.parse_args(argv)

    annotations_dir = args.annotations or _default_annotations()
    if not os.path.isdir(annotations_dir):
        sys.exit(f"annotations dir not found: {annotations_dir}")
    run_dir = resolve_run_dir(args.results_root)
    if not os.path.isdir(run_dir):
        sys.exit(f"results root not found: {args.results_root}")

    annot_idx = build_annotation_index(annotations_dir)
    cells_raw = find_cells(run_dir)
    if not cells_raw:
        sys.exit(f"no iter_NNN__<env>__<method> cells found under {run_dir}")

    print(f"results root: {run_dir}")
    print(f"annotations:  {annotations_dir}")
    print(f"cells found:  {len(cells_raw)} "
          f"(_failed_attempt_N retries already excluded)\n")

    results = []
    for iter_num, name, path, env, method in cells_raw:
        r = process_cell(iter_num, name, path, env, method, annotations_dir,
                          annot_idx, args.radius, args.use_3d,
                          args.traj_stride)
        results.append(r)

    print_report(results, args.radius)

    if args.json:
        out = {
            "results_root": run_dir,
            "annotations_dir": annotations_dir,
            "radius_m": args.radius,
            "use_3d": args.use_3d,
            "cells": results,
        }
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
