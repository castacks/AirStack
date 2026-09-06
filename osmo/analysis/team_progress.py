#!/usr/bin/env python3
"""Compute detector-confirmed team progress and path efficiency from bags.

Progress is ground-truth associated: a victim is detected when its world XY
location lies inside a 12 m circle around a planner `search_target`. Such a
target exists only after a person detection cleared the 0.65 confidence gate,
was projected through depth, and accumulated enough evidence to cluster. One
liberal target circle can credit multiple ground-truth people.

Metrics:
  progress       unique detector-confirmed victims / all victims
  progress_auc   integral(progress(t), 0..600 s) / 600 s
  actual_path_m  sum of every robot's XY path during its timed search window
  ideal_*_m      open multi-depot Euclidean route through victim centres
  ppl            progress * ideal_detected_m / max(actual_path_m, ideal_detected_m)

For methods with robot search sectors, a victim is assigned to the polygon
that contains it and only that robot may serve it in the oracle.  CoNavGPT2 is
solved as a shared multi-depot problem.  Routes are open: no return to the
launch point is required.  Empty robot sectors therefore have ideal length 0.

Usage:
  python3 osmo/analysis/team_progress.py --run /path/to/run [--run ...] \
      --json osmo/results/team_progress.json --markdown /tmp/results.md

Later --run roots replace earlier roots for the same scene/method.  Only plain
iter_* directories whose iteration.json says "passed" are eligible; failed
attempts are always ignored.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import glob
import io
import json
import math
import os
import re
from collections import defaultdict
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from mcap.exceptions import EndOfFile
from mcap.records import Chunk as ChunkRecord, Message as MessageRecord
from mcap.stream_reader import StreamReader, breakup_chunk
from mcap_ros2.decoder import DecoderFactory


ANNOT_TOPIC = "/gcs/annotations/bboxes"
VISIT_RADIUS_M = 12.0
BUDGET_S = 600.0
METRIC_VERSION = 4
# Four disasters x two locales x three levels x two scored fleet sizes
# (4-drone and 8-drone), per benchmark-disaster-dataset/SKILL.md.
TOTAL_RUNS_PER_BASELINE = 48
DEFAULT_DATASET_ROOT = Path("/home/krrishjain/SEI-COA/final_disaster_dataset")
START_RE = re.compile(r"sim budget \d+(?:\.\d+)? s starts NOW, at sim t=([\d.]+) s")
ROBOT_RE = re.compile(r"airstack-robot-desktop-(\d+)")
SCENE_RE = re.compile(r"(fire|hurricane|tornado|earthquake)(urban|suburban)l(\d+)", re.I)


def stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def passed_iterations(root: str):
    for p in sorted(Path(root).glob("iter_*")):
        if not p.is_dir() or "failed_attempt" in p.name:
            continue
        status = p / "iteration.json"
        if not status.exists():
            continue
        try:
            if json.loads(status.read_text()).get("status") == "passed":
                yield p
        except (OSError, json.JSONDecodeError):
            continue


def identity(iter_dir: Path):
    parts = iter_dir.name.split("__")
    if len(parts) < 3:
        raise ValueError(f"cannot parse iteration name: {iter_dir.name}")
    env, method = parts[-2], parts[-1]
    m = SCENE_RE.search(env)
    if not m:
        raise ValueError(f"cannot parse scene from {env}")
    disaster, locale, level = m.groups()
    scene = f"{disaster.title()} / {locale.title()} L{level}"
    pretty_method = {
        "frontier": "Frontier", "lawnmower": "Lawnmower",
        "vlfm": "VLFM", "conavgpt2_team": "CoNavGPT2",
        "nearest": "Nearest",
    }.get(method, method)
    return scene, pretty_method, method


def pose_group(person):
    pose = str(person.get("pose") or "").lower()
    attitude = str(person.get("attitude") or "").lower()
    if attitude == "upright" or any(k in pose for k in ("idle", "walk", "wave", "stand")):
        return "upright"
    if pose.startswith("lying_") or attitude in ("face_up", "face_down", "side"):
        return "lying"
    if "sit" in pose or "seated" in pose:
        return "seated"
    if "crouch" in pose:
        return "crouched"
    return "unknown"


def load_gt_attributes(scene, gt, dataset_root=DEFAULT_DATASET_ROOT):
    """Match bag GT markers to the frozen scene's richer GT_people records."""
    m = re.match(r"(\w+) / (\w+) L(\d+)", scene)
    if not m:
        return {}
    disaster, locale, level = m.groups()
    path = dataset_root / disaster / locale / f"level_{level}" / "1" / "GT_people.json"
    if not path.exists():
        return {}
    people = json.loads(path.read_text()).get("people", [])
    attrs = {}
    used = set()
    for target, (gx, gy) in gt.items():
        candidates = [(math.hypot(float(p["x"]) - gx, float(p["y"]) - gy), i, p)
                      for i, p in enumerate(people) if i not in used]
        if not candidates:
            continue
        dist, i, person = min(candidates)
        if dist > 2.0:
            continue
        used.add(i)
        environment = person.get("where") or person.get("scenario") or "unknown"
        attrs[target] = {
            "pose": pose_group(person),
            "visibility": str(person.get("visibility") or "unknown"),
            "occlusion": str(person.get("occlusion") or "unknown"),
            "environment": str(environment),
        }
    return attrs


def find_bag(iter_dir: Path) -> Path:
    bags = sorted(iter_dir.glob("bags/gcs/*.mcap"))
    if not bags:
        raise FileNotFoundError(f"no GCS bag under {iter_dir}")
    return bags[0]


def parse_starts(iter_dir: Path):
    per_robot, shared = {}, None
    for log in iter_dir.glob("logs/*/planner.log"):
        text = log.read_text(errors="replace")
        m = START_RE.search(text)
        if not m:
            continue
        value = float(m.group(1))
        rm = ROBOT_RE.search(str(log.parent))
        if rm:
            per_robot[int(rm.group(1))] = value
        elif log.parent.name == "offboard-compute":
            shared = value
    return per_robot, shared


def topic_map(summary):
    return {ch.topic: cid for cid, ch in summary.channels.items()}


def chunk_messages(f, chunk_index, channel_ids):
    """Decode one compressed chunk once and yield selected raw messages."""
    f.seek(chunk_index.chunk_start_offset)
    stream = StreamReader(io.BytesIO(f.read(chunk_index.chunk_length)),
                          skip_magic=True, emit_chunks=True)
    try:
        for record in stream.records:
            if isinstance(record, ChunkRecord):
                for inner in breakup_chunk(record):
                    if (isinstance(inner, MessageRecord)
                            and inner.channel_id in channel_ids):
                        yield inner
                return
    except EndOfFile:
        return


def credit_target_circle(gt, first_hits, center, radius, rel_t, robot):
    """Credit every as-yet-unseen GT person inside one detector target circle."""
    cx, cy = center
    r2 = radius * radius
    for target, (gx, gy) in gt.items():
        if (cx - gx) ** 2 + (cy - gy) ** 2 <= r2:
            hit = (max(0.0, rel_t), robot, cx, cy)
            if target not in first_hits or hit[0] < first_hits[target][0]:
                first_hits[target] = hit


def load_annotation_gt(path):
    """Read the launcher's canonical PEOPLE-only annotation JSON."""
    entries = json.loads(Path(path).read_text())
    gt = {}
    for index, entry in enumerate(entries):
        if entry.get("class") != "person":
            continue
        center = entry.get("bbox_world", {}).get("center_xyz_m", [])
        if len(center) >= 2:
            gt[index] = (float(center[0]), float(center[1]))
    if not gt:
        raise RuntimeError(f"no person records in GT annotation {path}")
    return gt


def annotation_name(scene):
    """Map `Earthquake / Urban L1` to the launcher's annotation basename."""
    match = re.fullmatch(r"(\w+) / (\w+) L(\d+)", scene)
    if not match:
        raise ValueError(f"cannot form annotation name from scene {scene!r}")
    disaster, locale, level = match.groups()
    return f"{disaster.title()}{locale.title()}L{level}V1.json"


def load_static_and_plan_starts(bag: Path, fallback_gt=None):
    """Load static GT, sector polygons, and first global-plan stamps."""
    factory = DecoderFactory()
    gt, sectors, plan_starts, plan_xy, origins = {}, {}, {}, {}, {}
    with bag.open("rb") as f:
        reader = make_reader(f, decoder_factories=[factory])
        summary = reader.get_summary()
        if summary is None:
            raise RuntimeError(f"bag has no index: {bag}")
        topics = topic_map(summary)
        expected_plans = sum(
            1 for cid, ch in summary.channels.items()
            if re.match(r"/robot_\d+/global_plan$", ch.topic)
            and summary.statistics.channel_message_counts.get(cid, 0) > 0)
        wanted = [ANNOT_TOPIC]
        wanted += [t for t in topics if re.match(r"/gcs/robot_\d+/search/markers$", t)]
        wanted += [t for t in topics if re.match(r"/robot_\d+/global_plan$", t)]
        wanted += [t for t in topics if re.match(r"/gcs/robot_\d+/map_origin$", t)]
        for _schema, channel, message, decoded in reader.iter_decoded_messages(topics=wanted):
            topic = channel.topic
            if topic == ANNOT_TOPIC and not gt:
                for mk in decoded.markers:
                    if mk.type == 1 and mk.ns == "person_fill":
                        p = mk.pose.position
                        gt[int(mk.id)] = (float(p.x), float(p.y))
            sm = re.match(r"/gcs/robot_(\d+)/search/markers$", topic)
            if sm:
                robot = int(sm.group(1))
                if robot not in sectors:
                    for mk in decoded.markers:
                        if mk.ns == "search_area" and len(mk.points) >= 3:
                            sectors[robot] = [(float(p.x), float(p.y)) for p in mk.points]
                            break
            pm = re.match(r"/robot_(\d+)/global_plan$", topic)
            if pm:
                robot = int(pm.group(1))
                plan_starts.setdefault(robot, stamp_s(decoded.header.stamp))
                if robot not in plan_xy and decoded.poses:
                    p = decoded.poses[0].pose.position
                    plan_xy[robot] = (float(p.x), float(p.y))
            om = re.match(r"/gcs/robot_(\d+)/map_origin$", topic)
            if om:
                robot = int(om.group(1))
                p = decoded.point
                origins.setdefault(robot, (float(p.x), float(p.y)))
            # Static topics and sector markers precede or accompany the first
            # global plans. Some accepted runs have inactive marker channels
            # (zero messages), so waiting for eight sector polygons scans the
            # entire multi-gigabyte bag and can never succeed. Eight first
            # plans are the reliable end-of-preamble sentinel.
            if (gt and expected_plans and len(plan_starts) >= expected_plans
                    and len(origins) >= expected_plans):
                break
    if not gt and fallback_gt:
        gt = load_annotation_gt(fallback_gt)
    if not gt:
        raise RuntimeError(f"no person_fill GT markers in {bag}")
    return gt, sectors, plan_starts, plan_xy, origins


def geometry_from_markdown(path):
    """Recover previously validated path lengths from a per-run result table.

    Detector rescoring does not change where robots flew. Reusing these values
    avoids decoding the image-heavy bag's complete odometry stream again. The
    markdown stores kilometres to 0.01 km, so the maximum path error is 5 m.
    """
    if not path:
        return {}
    out = {}
    for line in Path(path).read_text().splitlines():
        if not line.startswith("|") or "`" not in line:
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 9 or not cells[7].endswith(" km"):
            continue
        out[(cells[0], cells[1])] = float(cells[7][:-3]) * 1000.0
    return out


def load_search_evidence(bag: Path, gt, starts_t, circle_radii=(VISIT_RADIUS_M,),
                         budget=BUDGET_S, chunk_stride=50):
    """Read detector target circles and accumulated flight trails.

    `/gcs/robot_N/search/markers` is already transformed into the same global
    ENU frame as the GT markers.  A `search_target` marker exists only after a
    goal-class detection cleared sem_threshold, was projected through depth,
    accumulated in object_pcd, and formed a DBSCAN target instance.  The
    benchmark credits every GT person inside the planner's 12 m target circle.

    Target instances and trails persist in subsequent marker messages, so
    sampling every tenth compressed chunk retains final state and gives about
    one-second timing resolution while avoiding the image payloads in ~90% of
    the bag chunks.
    """
    factory = DecoderFactory()
    radii = tuple(sorted(set(float(r) for r in circle_radii)))
    hits_by_radius = {r: {} for r in radii}
    trails = {}
    starts_xy = {}
    target_circle_messages = 0
    with bag.open("rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        if summary is None:
            raise RuntimeError(f"bag has no index: {bag}")
        marker_channels = {
            cid: ch for cid, ch in summary.channels.items()
            if re.match(r"/gcs/robot_\d+/search/markers$", ch.topic)
        }
        if not marker_channels:
            raise RuntimeError(f"no GCS search marker channels in {bag}")
        channel_ids = set(marker_channels)
        decoders = {
            cid: factory.decoder_for(ch.message_encoding,
                                     summary.schemas[ch.schema_id])
            for cid, ch in marker_channels.items()
        }
        robot_of = {
            cid: int(ch.topic.split("/")[2].split("_")[1])
            for cid, ch in marker_channels.items()
        }
        chunks = sorted(
            (ci for ci in summary.chunk_indexes
             if channel_ids.intersection(ci.message_index_offsets)),
            key=lambda ci: ci.message_start_time)
        selected = set(range(0, len(chunks), max(1, chunk_stride)))
        # Always include each channel's last chunk so the longest accumulated
        # trail is retained even when it falls between stride samples.
        for cid in channel_ids:
            indices = [i for i, ci in enumerate(chunks)
                       if cid in ci.message_index_offsets]
            if indices:
                selected.add(indices[-1])

        for i in sorted(selected):
            for raw in chunk_messages(f, chunks[i], channel_ids):
                robot = robot_of[raw.channel_id]
                decoded = decoders[raw.channel_id](raw.data)
                for mk in decoded.markers:
                    if mk.action != 0:  # Marker.ADD/MODIFY only
                        continue
                    ts = stamp_s(mk.header.stamp)
                    rel_t = ts - starts_t.get(robot, ts)
                    if mk.ns == "search_robot" and rel_t >= 0 and robot not in starts_xy:
                        starts_xy[robot] = (float(mk.pose.position.x),
                                            float(mk.pose.position.y))
                    elif mk.ns == "search_trail" and len(mk.points) > len(trails.get(robot, ())):
                        trails[robot] = [(float(p.x), float(p.y)) for p in mk.points]
                    elif mk.ns == "search_target" and 0.0 <= rel_t <= budget:
                        target_circle_messages += 1
                        cx, cy = float(mk.pose.position.x), float(mk.pose.position.y)
                        for radius in radii:
                            credit_target_circle(gt, hits_by_radius[radius],
                                                 (cx, cy), radius, rel_t, robot)
    return hits_by_radius, trails, starts_xy, target_circle_messages


def load_world_trajectories(bag: Path):
    """Decode header-stamped odometry and translate robot-local maps to ENU."""
    factory = DecoderFactory()
    origins = {}
    with bag.open("rb") as f:
        reader = make_reader(f, decoder_factories=[factory])
        summary = reader.get_summary()
        topics = topic_map(summary)
        origin_topics = [t for t in topics if re.match(r"/gcs/robot_\d+/map_origin$", t)]
        for _s, channel, _m, decoded in reader.iter_decoded_messages(topics=origin_topics):
            robot = int(channel.topic.split("/")[2].split("_")[1])
            p = decoded.point
            origins.setdefault(robot, (float(p.x), float(p.y), float(p.z)))
            if len(origins) == len(origin_topics):
                break

    raw = defaultdict(list)
    with bag.open("rb") as f:
        reader = make_reader(f, decoder_factories=[factory])
        summary = reader.get_summary()
        topics = topic_map(summary)
        odom_topics = [t for t in topics if re.match(
            r"/robot_\d+/odometry_conversion/odometry$", t)]
        for _s, channel, _m, decoded in reader.iter_decoded_messages(topics=odom_topics):
            robot = int(channel.topic.split("/")[1].split("_")[1])
            if robot not in origins:
                continue
            o = origins[robot]
            p = decoded.pose.pose.position
            raw[robot].append((stamp_s(decoded.header.stamp),
                               float(p.x) + o[0], float(p.y) + o[1]))
    return raw, origins


def resample_window(points, start, budget=BUDGET_S, step=1.0):
    """Linearly sample XY at fixed sim-time intervals, suppressing odom jitter."""
    if len(points) < 2:
        return np.empty((0, 3), dtype=float)
    arr = np.asarray(points, dtype=float)
    arr = arr[np.argsort(arr[:, 0])]
    # Drop duplicate timestamps (keep last).
    _, rev_idx = np.unique(arr[::-1, 0], return_index=True)
    arr = arr[np.sort(len(arr) - 1 - rev_idx)]
    end = start + budget
    lo = max(start, arr[0, 0])
    hi = min(end, arr[-1, 0])
    if hi <= lo:
        return np.empty((0, 3), dtype=float)
    times = np.arange(lo, hi + 1e-6, step)
    if times[-1] < hi:
        times = np.append(times, hi)
    x = np.interp(times, arr[:, 0], arr[:, 1])
    y = np.interp(times, arr[:, 0], arr[:, 2])
    return np.column_stack((times - start, x, y))


def point_in_polygon(point, polygon):
    """Boundary-inclusive ray-cast test."""
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if abs(cross) < 1e-8 and min(x1, x2) - 1e-8 <= x <= max(x1, x2) + 1e-8 \
                and min(y1, y2) - 1e-8 <= y <= max(y1, y2) + 1e-8:
            return True
        if (y1 > y) != (y2 > y):
            xin = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < xin:
                inside = not inside
    return inside


def assign_targets(gt, sectors, robots, shared):
    if shared or not sectors:
        return {target: None for target in gt}
    centroids = {r: np.mean(np.asarray(poly), axis=0) for r, poly in sectors.items()}
    assignment = {}
    for target, xy in gt.items():
        candidates = [r for r, poly in sectors.items() if point_in_polygon(xy, poly)]
        if not candidates:
            # Numerical seams and slightly cropped marker polygons should not
            # silently discard GT; assign to the nearest recorded sector.
            candidates = list(centroids)
        assignment[target] = min(candidates,
            key=lambda r: (float(np.linalg.norm(np.asarray(xy) - centroids[r])), r))
    return assignment


def route_length(starts, gt, targets, assignment=None, time_limit_s=2):
    """Open multi-depot VRP length. Returns (metres, per-robot metres)."""
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2

    robots = sorted(starts)
    targets = list(targets)
    if not robots or not targets:
        return 0.0, {r: 0.0 for r in robots}
    locations = [starts[r] for r in robots] + [gt[t] for t in targets]
    end_offset = len(locations)
    locations += [(0.0, 0.0)] * len(robots)  # zero-cost dummy open ends
    starts_i = list(range(len(robots)))
    ends_i = list(range(end_offset, end_offset + len(robots)))
    manager = pywrapcp.RoutingIndexManager(len(locations), len(robots), starts_i, ends_i)
    routing = pywrapcp.RoutingModel(manager)

    def distance(from_index, to_index):
        a, b = manager.IndexToNode(from_index), manager.IndexToNode(to_index)
        if b >= end_offset:
            return 0
        return int(round(math.dist(locations[a], locations[b]) * 1000.0))

    transit = routing.RegisterTransitCallback(distance)
    routing.SetArcCostEvaluatorOfAllVehicles(transit)
    if assignment:
        robot_vehicle = {r: i for i, r in enumerate(robots)}
        for j, target in enumerate(targets):
            assigned = assignment.get(target)
            if assigned in robot_vehicle:
                # SetAllowedVehiclesForIndex has a broken absl::Span binding
                # in some OR-Tools 9.15 wheels; the one-vehicle case is exactly
                # represented by fixing VehicleVar directly.
                routing.VehicleVar(manager.NodeToIndex(len(robots) + j)).SetValue(
                    robot_vehicle[assigned])

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(max(1, int(time_limit_s)))
    solution = routing.SolveWithParameters(params)
    if solution is None:
        raise RuntimeError("VRP solver found no solution")
    per_robot = {}
    for vehicle, robot in enumerate(robots):
        index, length = routing.Start(vehicle), 0
        while not routing.IsEnd(index):
            nxt = solution.Value(routing.NextVar(index))
            length += routing.GetArcCostForVehicle(index, nxt, vehicle)
            index = nxt
        per_robot[robot] = length / 1000.0
    return sum(per_robot.values()), per_robot


def analyze_iteration(iter_dir: Path, radius=VISIT_RADIUS_M, budget=BUDGET_S,
                      cached_actual_path_m=None, chunk_stride=50,
                      gt_annotations_dir=None):
    scene, method, raw_method = identity(iter_dir)
    shared = raw_method == "conavgpt2_team"
    bag = find_bag(iter_dir)
    fallback_gt = None
    if gt_annotations_dir:
        candidate = Path(gt_annotations_dir) / annotation_name(scene)
        if candidate.exists():
            fallback_gt = candidate
    gt, sectors, plan_starts, plan_xy, origins = load_static_and_plan_starts(
        bag, fallback_gt)
    raw_traj = None
    if cached_actual_path_m is None:
        raw_traj, _origins = load_world_trajectories(bag)
    log_starts, shared_start = parse_starts(iter_dir)
    robots = sorted(raw_traj if raw_traj is not None else plan_starts)
    starts_t = {}
    for robot in robots:
        starts_t[robot] = (shared_start if shared_start is not None else
                           log_starts.get(robot, plan_starts.get(robot)))
    missing = [r for r, t in starts_t.items() if t is None]
    if missing:
        raise RuntimeError(f"no timed-search start for robots {missing}")
    sensitivity_radii = (radius, radius + 5.0, radius + 10.0)
    hits_by_radius, _trails, _marker_starts, target_circle_messages = load_search_evidence(
        bag, gt, starts_t, sensitivity_radii, budget, chunk_stride)
    hits = hits_by_radius[radius]
    gt_attrs = load_gt_attributes(scene, gt)
    breakdowns = {}
    for axis in ("pose", "visibility", "occlusion", "environment"):
        counts = defaultdict(lambda: {"total": 0, "detected": 0})
        for target in gt:
            category = gt_attrs.get(target, {}).get(axis, "unknown")
            counts[category]["total"] += 1
            counts[category]["detected"] += int(target in hits)
        breakdowns[axis] = dict(counts)
    if raw_traj is not None:
        trajectories = {r: resample_window(raw_traj[r], starts_t[r], budget)
                        for r in robots}
        starts_xy = {r: tuple(trajectories[r][0, 1:3]) for r in robots
                     if len(trajectories[r])}
        actual = sum(float(np.linalg.norm(np.diff(tr[:, 1:3], axis=0), axis=1).sum())
                     for tr in trajectories.values() if len(tr) > 1)
    else:
        missing_xy = [r for r in robots if r not in plan_xy]
        if missing_xy:
            raise RuntimeError(f"no first-plan XY for robots {missing_xy}")
        # Cached geometry mode is used only for fixed planners. Their first
        # plan is robot-local, so translate its first pose with map_origin.
        # CoNavGPT2 always takes the full odometry path because its shared
        # plan's first pose is not a reliable robot start depot.
        starts_xy = {
            r: (
                plan_xy[r][0] + origins.get(r, (0.0, 0.0))[0],
                plan_xy[r][1] + origins.get(r, (0.0, 0.0))[1])
            for r in robots
        }
        actual = float(cached_actual_path_m)
    assignment = assign_targets(gt, sectors, robots, shared)
    detections = {target: hit[0] for target, hit in hits.items()}
    detectors = {target: hit[1] for target, hit in hits.items()}
    ideal_all, ideal_by_robot = route_length(
        starts_xy, gt, gt.keys(), None if shared else assignment)
    # The found-set oracle is allowed to optimally reassign targets for shared
    # CoNavGPT2; fixed-sector methods retain their recorded ownership.
    ideal_detected, _ = route_length(
        starts_xy, gt, detections.keys(), None if shared else assignment)
    progress = len(detections) / len(gt) if gt else 0.0
    progress_auc = (sum(max(0.0, budget - t) for t in detections.values()) /
                    (budget * len(gt))) if gt else 0.0
    ppl = (progress * ideal_detected / max(actual, ideal_detected)
           if progress and ideal_detected else 0.0)
    return {
        "metric_version": METRIC_VERSION,
        "scene": scene, "method": method, "method_key": raw_method,
        "run_folder": f"{iter_dir.parent.parent.name}/{iter_dir.parent.name}/{iter_dir.name}",
        "iter_dir": str(iter_dir), "bag": str(bag),
        "gt_victims": len(gt), "detected_victims": len(detections),
        "progress": progress, "progress_auc": progress_auc,
        "actual_path_m": actual, "ideal_all_path_m": ideal_all,
        "ideal_detected_path_m": ideal_detected, "ppl": ppl,
        "target_circle_radius_m": radius, "budget_s": budget,
        "target_circle_messages_sampled": target_circle_messages,
        "radius_sensitivity": {
            str(int(r) if r.is_integer() else r): {
                "detected_victims": len(hits_by_radius[r]),
                "progress": len(hits_by_radius[r]) / len(gt) if gt else 0.0,
                "progress_auc": (sum(max(0.0, budget - h[0])
                                     for h in hits_by_radius[r].values()) /
                                 (budget * len(gt))) if gt else 0.0,
            } for r in sensitivity_radii
        },
        "target_breakdowns": breakdowns,
        "start_source": "offboard planner log" if shared_start is not None else
                        ("per-robot planner logs" if len(log_starts) == len(robots)
                         else "first global plan fallback"),
        "robots": len(robots), "empty_oracle_routes": sum(v < 1e-6 for v in ideal_by_robot.values()),
        "detected_by_robot": {str(r): sum(v == r for v in detectors.values()) for r in robots},
    }


def averages(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[row["method"]].append(row)
    result = []
    for method, vals in sorted(groups.items()):
        result.append({
            "method": method, "runs": len(vals),
            **{k: float(np.mean([v[k] for v in vals])) for k in
               ("progress", "progress_auc", "actual_path_m", "ideal_all_path_m", "ppl")},
        })
    return result


def markdown(rows, avgs):
    out = [
        "## Actual results (detector-confirmed team progress and PPL)", "",
        "A GT victim counts as detected when its world-frame XY location falls inside a **12 m circle around a planner `search_target`** during the 600-s search. A target circle exists only after a `person` detection clears the shared 0.65 confidence gate, is depth-projected, and forms a clustered target instance. One liberal circle can credit multiple GT people; drone proximity alone never counts. Time-integrated progress is normalized area under the cumulative detector-confirmed progress curve; marker chunks were sampled at about 20-s intervals (final persistent target state is always read, so final detection counts are exact). Paths are 1 Hz, world-frame XY odometry. Ideal lengths are OR-Tools oracle estimates for open Euclidean multi-depot routes through victim centres; fixed-sector methods preserve recorded robot ownership, while CoNavGPT2 permits joint assignment. Ground debris does not obstruct an aerial XY geodesic, and no return to launch is required. PPL uses the ideal route through detected GT victims: `progress × ideal_detected / max(actual, ideal_detected)`.", "",
        "### Per completed run", "",
        "| Scene | Method | Run folder | GT | Detected | Detector-confirmed progress | Time-integrated progress | Actual team path | Ideal all-target path | PPL |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for r in sorted(rows, key=lambda x: (x["scene"], x["method"])):
        out.append(f'| {r["scene"]} | {r["method"]} | `{r["run_folder"]}` | '
                   f'{r["gt_victims"]} | {r["detected_victims"]} | {r["progress"]:.3f} | '
                   f'{r["progress_auc"]:.3f} | {r["actual_path_m"]/1000:.2f} km | '
                   f'{r["ideal_all_path_m"]/1000:.2f} km | {r["ppl"]:.4f} |')
    out += ["", "### Average by baseline", "",
            "| Method | Completed / total runs | Avg progress | Avg time-integrated progress | Avg actual team path | Avg ideal all-target path | Avg PPL |",
            "|---|---:|---:|---:|---:|---:|---:|"]
    for a in avgs:
        out.append(f'| {a["method"]} | {a["runs"]}/{TOTAL_RUNS_PER_BASELINE} | {a["progress"]:.3f} | '
                   f'{a["progress_auc"]:.3f} | {a["actual_path_m"]/1000:.2f} km | '
                   f'{a["ideal_all_path_m"]/1000:.2f} km | {a["ppl"]:.4f} |')

    radius_keys = sorted(
        {key for row in rows for key in row.get("radius_sensitivity", {})},
        key=float)
    if radius_keys:
        out += ["", "### Target-circle radius sensitivity", "",
                "These rows change only the GT-to-target-circle association radius; the detector gate and target circles are unchanged.", ""]
        base_key = radius_keys[0]
        columns = [f"{float(base_key):g} m progress"]
        for key in radius_keys[1:]:
            columns += [f"{float(key):g} m progress", "Gain vs base"]
        header = "| Method | Runs | " + " | ".join(columns) + " |"
        out += [header, "|---|---:|" + "---:|" * len(columns)]
        by_method = defaultdict(list)
        for row in rows:
            by_method[row["method"]].append(row)
        for method, vals in sorted(by_method.items()):
            means = {}
            for key in radius_keys:
                samples = [v["radius_sensitivity"][key]["progress"] for v in vals
                           if key in v.get("radius_sensitivity", {})]
                means[key] = float(np.mean(samples)) if samples else None
            cells = [f"{means[base_key]:.3f}" if means[base_key] is not None else "—"]
            for key in radius_keys[1:]:
                if means[key] is None or means[base_key] is None:
                    cells += ["—", "—"]
                else:
                    cells += [f"{means[key]:.3f}",
                              f"{means[key] - means[base_key]:+.3f}"]
            out.append(f"| {method} | {len(vals)} | " + " | ".join(cells) + " |")

        zero_rows = [r for r in rows
                     if r["radius_sensitivity"][base_key]["detected_victims"] == 0]
        out += ["", f"#### Runs with zero detections at {float(base_key):g} m", "",
                "| Scene | Method | " + " | ".join(
                    f"{float(key):g} m detected / GT" for key in radius_keys) + " |",
                "|---|---|" + "---:|" * len(radius_keys)]
        for row in sorted(zero_rows, key=lambda x: (x["scene"], x["method"])):
            cells = [f'{row["radius_sensitivity"][key]["detected_victims"]}/{row["gt_victims"]}'
                     for key in radius_keys]
            out.append(f'| {row["scene"]} | {row["method"]} | ' + " | ".join(cells) + " |")

    if any(row.get("target_breakdowns") for row in rows):
        out += ["", "Breakdown opportunities count each GT victim once per completed run/method; the same frozen-scene victim is therefore one opportunity for each baseline that searched that scene."]

    for axis, title in (("pose", "Pose"), ("visibility", "Visibility"),
                        ("occlusion", "Occlusion"), ("environment", "Environment")):
        counts = defaultdict(lambda: {"total": 0, "detected": 0})
        for row in rows:
            prefix = row["scene"].split(" / ", 1)[0] + " / " if axis == "environment" else ""
            for category, values in row.get("target_breakdowns", {}).get(axis, {}).items():
                key = prefix + category
                counts[key]["total"] += values["total"]
                counts[key]["detected"] += values["detected"]
        if counts:
            out += ["", f"### Detection breakdown by {title.lower()}", "",
                    f"| {title} | Detected / opportunities | Detection rate |",
                    "|---|---:|---:|"]
            for category, values in sorted(counts.items()):
                rate = values["detected"] / values["total"] if values["total"] else 0.0
                out.append(f'| {category} | {values["detected"]}/{values["total"]} | {rate:.3f} |')
    return "\n".join(out) + "\n"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", action="append", required=True)
    p.add_argument("--json")
    p.add_argument("--markdown")
    p.add_argument("--visit-radius", type=float, default=VISIT_RADIUS_M)
    p.add_argument("--budget", type=float, default=BUDGET_S)
    p.add_argument("--workers", type=int, default=1,
                   help="bags to decode concurrently (useful on network storage)")
    p.add_argument("--resume", action="store_true",
                   help="reuse matching rows already present in --json")
    p.add_argument("--refresh-method",
                   help="with --resume, recompute rows whose pretty method matches")
    p.add_argument("--geometry-markdown",
                   help="reuse unchanged actual-path lengths from an earlier result table")
    p.add_argument("--chunk-stride", type=int, default=50,
                   help="sample every Nth marker-bearing chunk (final state is always read)")
    p.add_argument("--gt-annotations-dir",
                   help="fallback directory of canonical <Scene>V1.json files when a bag omitted GT markers")
    args = p.parse_args()
    chosen = {}
    for root in args.run:
        for iteration in passed_iterations(root):
            try:
                scene, method, _ = identity(iteration)
            except ValueError:
                continue
            chosen[(scene, method)] = iteration
    rows = []
    if args.resume and args.json and Path(args.json).exists():
        try:
            old_rows = json.loads(Path(args.json).read_text()).get("rows", [])
            wanted_paths = {str(p) for p in chosen.values()}
            rows = [r for r in old_rows if r.get("iter_dir") in wanted_paths
                    and r.get("metric_version") == METRIC_VERSION
                    and r.get("method") != args.refresh_method]
        except (OSError, json.JSONDecodeError):
            rows = []
    def save_partial():
        avgs_now = averages(rows)
        payload = {"schema_version": 1, "rows": rows, "averages": avgs_now,
                   "failed_analyses": len(chosen) - len(rows),
                   "analysis_complete": len(rows) == len(chosen)}
        if args.json:
            Path(args.json).write_text(json.dumps(payload, indent=2) + "\n")
        if args.markdown:
            Path(args.markdown).write_text(markdown(rows, avgs_now))

    completed_paths = {r["iter_dir"] for r in rows}
    pending = [iteration for _key, iteration in sorted(chosen.items())
               if str(iteration) not in completed_paths]

    cached_geometry = geometry_from_markdown(args.geometry_markdown)

    def run_one(iteration):
        scene, method, raw = identity(iteration)
        cached_path = (None if raw == "conavgpt2_team"
                       else cached_geometry.get((scene, method)))
        return analyze_iteration(iteration, args.visit_radius, args.budget,
                                 cached_path, args.chunk_stride,
                                 args.gt_annotations_dir)

    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = {}
        for iteration in pending:
            print(f"[start] {iteration.name}", flush=True)
            futures[pool.submit(run_one, iteration)] = iteration
        for future in concurrent.futures.as_completed(futures):
            iteration = futures[future]
            print(f"[done {len(rows)+1}/{len(chosen)}] {iteration.name}", flush=True)
            try:
                rows.append(future.result())
                rows.sort(key=lambda r: (r["scene"], r["method"]))
                save_partial()
            except Exception as exc:
                print(f"ERROR {iteration}: {exc}", flush=True)
    avgs = averages(rows)
    payload = {"schema_version": 1, "rows": rows, "averages": avgs,
               "failed_analyses": len(chosen) - len(rows),
               "analysis_complete": len(rows) == len(chosen)}
    if args.json:
        Path(args.json).write_text(json.dumps(payload, indent=2) + "\n")
    md = markdown(rows, avgs)
    if args.markdown:
        Path(args.markdown).write_text(md)
    else:
        print(md)
    return 1 if len(rows) != len(chosen) else 0


if __name__ == "__main__":
    raise SystemExit(main())
