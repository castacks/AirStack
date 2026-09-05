#!/usr/bin/env python3
"""Compute team search progress and path-efficiency metrics from completed bags.

The analysis is deliberately ground-truth based.  A victim is physically
visited when a robot's world-frame XY trajectory comes within 12 m (the search
planner's target_visit_radius_m).  Detector proposals are not used.

Metrics:
  progress       unique visited victims / all victims
  progress_auc   integral(progress(t), 0..600 s) / 600 s
  actual_path_m  sum of every robot's XY path during its timed search window
  ideal_*_m      open multi-depot Euclidean route through victim centres
  ppl            progress * ideal_found_m / max(actual_path_m, ideal_found_m)

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
import json
import math
import os
import re
from collections import defaultdict
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


ANNOT_TOPIC = "/gcs/annotations/bboxes"
VISIT_RADIUS_M = 12.0
BUDGET_S = 600.0
METRIC_VERSION = 2
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


def load_static_and_plan_starts(bag: Path):
    """Load static GT, sector polygons, and first global-plan stamps."""
    factory = DecoderFactory()
    gt, sectors, plan_starts = {}, {}, {}
    with bag.open("rb") as f:
        reader = make_reader(f, decoder_factories=[factory])
        summary = reader.get_summary()
        if summary is None:
            raise RuntimeError(f"bag has no index: {bag}")
        topics = topic_map(summary)
        wanted = [ANNOT_TOPIC]
        wanted += [t for t in topics if re.match(r"/gcs/robot_\d+/search/markers$", t)]
        wanted += [t for t in topics if re.match(r"/robot_\d+/global_plan$", t)]
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
            # Static topics and the first plans occur near bag start.  Stop as
            # soon as all eight plans and sectors are known; shared methods do
            # not publish sector polygons, so eight plans alone is sufficient.
            if gt and len(plan_starts) >= 8 and (len(sectors) >= 8 or not any(
                    "/search/markers" in t for t in wanted)):
                break
    if not gt:
        raise RuntimeError(f"no person_fill GT markers in {bag}")
    return gt, sectors, plan_starts


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


def visit_times(gt, assignment, trajectories, radius=VISIT_RADIUS_M, shared=False):
    visits = {}
    visitor = {}
    r2 = radius * radius
    for target, xy in gt.items():
        eligible = trajectories if shared else [assignment[target]]
        for robot in eligible:
            tr = trajectories.get(robot)
            if tr is None or len(tr) == 0:
                continue
            d2 = (tr[:, 1] - xy[0]) ** 2 + (tr[:, 2] - xy[1]) ** 2
            hits = np.flatnonzero(d2 <= r2)
            if len(hits):
                t = float(tr[hits[0], 0])
                if target not in visits or t < visits[target]:
                    visits[target], visitor[target] = t, robot
    return visits, visitor


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


def analyze_iteration(iter_dir: Path, radius=VISIT_RADIUS_M, budget=BUDGET_S):
    scene, method, raw_method = identity(iter_dir)
    shared = raw_method == "conavgpt2_team"
    bag = find_bag(iter_dir)
    gt, sectors, plan_starts = load_static_and_plan_starts(bag)
    raw_traj, _origins = load_world_trajectories(bag)
    log_starts, shared_start = parse_starts(iter_dir)
    robots = sorted(raw_traj)
    starts_t = {}
    for robot in robots:
        starts_t[robot] = (shared_start if shared_start is not None else
                           log_starts.get(robot, plan_starts.get(robot)))
    missing = [r for r, t in starts_t.items() if t is None]
    if missing:
        raise RuntimeError(f"no timed-search start for robots {missing}")
    trajectories = {r: resample_window(raw_traj[r], starts_t[r], budget)
                    for r in robots}
    starts_xy = {r: tuple(trajectories[r][0, 1:3]) for r in robots
                 if len(trajectories[r])}
    assignment = assign_targets(gt, sectors, robots, shared)
    # Team progress credits a physical visit by any drone, including a rare
    # cross-sector visit.  Sector ownership constrains only the ideal route.
    visits, visitors = visit_times(gt, assignment, trajectories, radius, shared=True)
    actual = sum(float(np.linalg.norm(np.diff(tr[:, 1:3], axis=0), axis=1).sum())
                 for tr in trajectories.values() if len(tr) > 1)
    ideal_all, ideal_by_robot = route_length(
        starts_xy, gt, gt.keys(), None if shared else assignment)
    # The found-set oracle is allowed to optimally reassign targets for shared
    # CoNavGPT2; fixed-sector methods retain their recorded ownership.
    ideal_found, _ = route_length(
        starts_xy, gt, visits.keys(), None if shared else assignment)
    progress = len(visits) / len(gt) if gt else 0.0
    progress_auc = (sum(max(0.0, budget - t) for t in visits.values()) /
                    (budget * len(gt))) if gt else 0.0
    ppl = progress * ideal_found / max(actual, ideal_found) if progress and ideal_found else 0.0
    return {
        "metric_version": METRIC_VERSION,
        "scene": scene, "method": method, "method_key": raw_method,
        "run_folder": f"{iter_dir.parent.parent.name}/{iter_dir.parent.name}/{iter_dir.name}",
        "iter_dir": str(iter_dir), "bag": str(bag),
        "gt_victims": len(gt), "visited_victims": len(visits),
        "progress": progress, "progress_auc": progress_auc,
        "actual_path_m": actual, "ideal_all_path_m": ideal_all,
        "ideal_found_path_m": ideal_found, "ppl": ppl,
        "visit_radius_m": radius, "budget_s": budget,
        "start_source": "offboard planner log" if shared_start is not None else
                        ("per-robot planner logs" if len(log_starts) == len(robots)
                         else "first global plan fallback"),
        "robots": len(robots), "empty_oracle_routes": sum(v < 1e-6 for v in ideal_by_robot.values()),
        "visited_by_robot": {str(r): sum(v == r for v in visitors.values()) for r in robots},
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
        "## Actual results (ground-truth team progress and PPL)", "",
        "A physical visit means any drone came within **12 m horizontally** of a GT victim during its own 600-s timed search window. Time-integrated progress is normalized area under the cumulative team-progress curve (higher is better; an immediate complete search is 1.0). Paths are 1 Hz, world-frame XY odometry. The ideal lengths are OR-Tools oracle estimates for open Euclidean multi-depot routes through victim centres; fixed-sector methods preserve recorded robot ownership, while CoNavGPT2 permits joint assignment. Ground debris does not obstruct an aerial XY geodesic, and no return to launch is required. PPL uses the ideal route through the victims actually reached: `progress × ideal_found / max(actual, ideal_found)`.", "",
        "### Per completed run", "",
        "| Scene | Method | Run folder | GT | Visited | Progress | Time-integrated progress | Actual team path | Ideal all-target path | PPL |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for r in sorted(rows, key=lambda x: (x["scene"], x["method"])):
        out.append(f'| {r["scene"]} | {r["method"]} | `{r["run_folder"]}` | '
                   f'{r["gt_victims"]} | {r["visited_victims"]} | {r["progress"]:.3f} | '
                   f'{r["progress_auc"]:.3f} | {r["actual_path_m"]/1000:.2f} km | '
                   f'{r["ideal_all_path_m"]/1000:.2f} km | {r["ppl"]:.4f} |')
    out += ["", "### Average by baseline", "",
            "| Method | Runs | Avg progress | Avg time-integrated progress | Avg actual team path | Avg ideal all-target path | Avg PPL |",
            "|---|---:|---:|---:|---:|---:|---:|"]
    for a in avgs:
        out.append(f'| {a["method"]} | {a["runs"]} | {a["progress"]:.3f} | '
                   f'{a["progress_auc"]:.3f} | {a["actual_path_m"]/1000:.2f} km | '
                   f'{a["ideal_all_path_m"]/1000:.2f} km | {a["ppl"]:.4f} |')
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
                    and r.get("metric_version") == METRIC_VERSION]
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

    def run_one(iteration):
        return analyze_iteration(iteration, args.visit_radius, args.budget)

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
