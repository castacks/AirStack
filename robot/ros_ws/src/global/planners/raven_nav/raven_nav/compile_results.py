#!/usr/bin/env python3
"""Compile per-robot raven result dumps into one fleet-wide result + metrics.

Each raven_nav node writes ``<results_dir>/<robot>.json`` (own AABBs + per-target
event log + path length, all in **global ENU**). This script reads every such
file, merges the AABBs across robots with the *same* dedup logic raven uses at
runtime (``discoveries.merge_confirmed_targets`` — NMS keep-representative over
xy-projected overlap/gap, so stacked z-fragments of one structure collapse and no
box balloons by union), so boxes that refer to the same physical house collapse
into one. It then aggregates discovery/confirmation/
visit timestamps and multi-robot path-cost metrics.

Status matters: a merged AABB is ``visited`` if *any* contributing robot marked it
visited (``_fold_into_rep`` keeps 'visited' sticky), else ``observing``. We report two
detection sets downstream:
  - **either**  — observing ∪ visited (all detections; the useful set for the
    frontier baseline, where nothing is ever visited).
  - **visited** — the subset the fleet actually closed out.

Outputs (written next to the inputs):
  - ``compiled_results.json`` — deduped fleet AABBs (+ per-house first
    discovery/confirmation/visit time) and the raw per-robot summaries.
  - ``metrics.json``          — path-length / cost + timing metrics.

Usage:
    python3 -m raven_nav.compile_results --results-dir /root/.cache/raven_results \
        [--expect 3] [--wait-timeout 30] [--reason time_limit]
    ros2 run raven_nav compile_results --results-dir ...
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import time
from typing import List, Optional

import numpy as np

from raven_nav.discoveries import (
    ConfirmedTarget,
    merge_confirmed_targets,
)

# Same radius used in raven's event log to decide two centroids are the same
# physical instance (for attributing event timestamps to a merged AABB).
EVENT_MATCH_DIST_M = 8.0


def _load_robot_files(results_dir: str, expect: int,
                      wait_timeout: float) -> List[dict]:
    """Load all ``<robot>.json`` files, optionally waiting for ``expect`` of
    them to appear (each robot's finalize calls this concurrently)."""
    deadline = time.time() + max(0.0, wait_timeout)
    pattern = os.path.join(results_dir, '*.json')
    skip = {'compiled_results.json', 'metrics.json',
            'groundtruth_comparison.json'}
    while True:
        files = [f for f in sorted(glob.glob(pattern))
                 if os.path.basename(f) not in skip]
        loaded = []
        for f in files:
            try:
                with open(f) as fh:
                    loaded.append(json.load(fh))
            except (OSError, json.JSONDecodeError):
                continue   # mid-write; retry on next pass
        if expect <= 0 or len(loaded) >= expect or time.time() >= deadline:
            return loaded
        time.sleep(1.0)


def _targets_to_confirmed(robots: List[dict]) -> List[ConfirmedTarget]:
    """Flatten every robot's AABBs into ConfirmedTargets (global ENU)."""
    out: List[ConfirmedTarget] = []
    for rb in robots:
        for t in rb.get('confirmed_targets_enu', []):
            out.append(ConfirmedTarget(
                label=str(t.get('label', '')),
                center=np.asarray(t.get('center_enu', [0, 0, 0]), dtype=float),
                size=np.asarray(t.get('size', [0, 0, 0]), dtype=float),
                status=str(t.get('status', 'observing')),
                confidence=float(t.get('confidence', 0.0)),
                ts=0.0,
            ))
    return out


def _all_events(robots: List[dict]) -> List[dict]:
    out = []
    for rb in robots:
        for ev in rb.get('target_events', []):
            out.append(ev)
    return out


def _min_opt(a: Optional[float], b: Optional[float]) -> Optional[float]:
    vals = [x for x in (a, b) if x is not None]
    return min(vals) if vals else None


def _attach_events(merged: List[ConfirmedTarget],
                   events: List[dict]) -> List[dict]:
    """For each merged fleet AABB, gather the earliest discovery/confirm/visit
    time across all robot events whose centroid falls within match radius."""
    houses = []
    for ct in merged:
        c = np.asarray(ct.center, dtype=float)
        disc = conf = vis = None
        n_contrib = 0
        for ev in events:
            if ev.get('label') != ct.label:
                continue
            p = np.asarray(ev.get('pos_enu', [1e9, 1e9, 1e9]), dtype=float)
            if float(np.linalg.norm(p - c)) <= EVENT_MATCH_DIST_M:
                n_contrib += 1
                disc = _min_opt(disc, ev.get('first_discovered_ts'))
                conf = _min_opt(conf, ev.get('first_confirmed_ts'))
                vis = _min_opt(vis, ev.get('first_visited_ts'))
        houses.append({
            'label': ct.label,
            'center_enu': c.tolist(),
            'size': np.asarray(ct.size, dtype=float).tolist(),
            'status': ct.status,            # 'visited' if any robot visited it
            'confidence': float(ct.confidence),
            'first_discovered_ts': disc,
            'first_confirmed_ts': conf,
            'first_visited_ts': vis,
            'num_event_contributors': n_contrib,
        })
    return houses


def _path_metrics(robots: List[dict]) -> dict:
    """Per-robot + fleet path-length / cost metrics for multi-robot eval."""
    per_robot = {}
    lengths = []
    durations = []
    for rb in robots:
        name = rb.get('robot', '?')
        L = float(rb.get('path_length_m', 0.0) or 0.0)
        D = rb.get('mission_duration_s')
        per_robot[name] = {
            'path_length_m': L,
            'mission_duration_s': D,
            'completion_reason': rb.get('completion_reason'),
            'coverage_fraction': rb.get('coverage_fraction'),
            'avg_speed_m_s': (L / D) if (D and D > 0) else None,
        }
        lengths.append(L)
        if D is not None:
            durations.append(float(D))
    n = len(lengths)
    total = float(sum(lengths))
    arr = np.asarray(lengths, dtype=float) if lengths else np.zeros(0)
    return {
        'num_robots': n,
        'per_robot': per_robot,
        # Total distance flown by the whole fleet — the headline cost metric.
        'total_path_length_m': total,
        'mean_path_length_m': float(arr.mean()) if n else 0.0,
        'std_path_length_m': float(arr.std()) if n else 0.0,
        'min_path_length_m': float(arr.min()) if n else 0.0,
        # Max single-robot path ≈ makespan proxy; imbalance = max/mean.
        'max_path_length_m': float(arr.max()) if n else 0.0,
        'path_imbalance': (float(arr.max() / arr.mean())
                           if n and arr.mean() > 0 else None),
        'makespan_s': max(durations) if durations else None,
        'sum_duration_s': float(sum(durations)) if durations else None,
    }


def compile_results(results_dir: str, expect: int = 0,
                    wait_timeout: float = 0.0,
                    reason: Optional[str] = None) -> dict:
    robots = _load_robot_files(results_dir, expect, wait_timeout)
    merged = merge_confirmed_targets(_targets_to_confirmed(robots))
    events = _all_events(robots)
    houses = _attach_events(merged, events)

    n_either = len(houses)
    n_visited = sum(1 for h in houses if str(h['status']).lower() == 'visited')
    n_observing = n_either - n_visited

    # Earliest-time helpers relative to the earliest mission start across robots.
    starts = [rb.get('mission_start_ts') for rb in robots
              if rb.get('mission_start_ts') is not None]
    t0 = min(starts) if starts else None

    def _rel(ts):
        return (ts - t0) if (ts is not None and t0 is not None) else None

    for h in houses:
        h['first_discovered_rel_s'] = _rel(h['first_discovered_ts'])
        h['first_confirmed_rel_s'] = _rel(h['first_confirmed_ts'])
        h['first_visited_rel_s'] = _rel(h['first_visited_ts'])

    compiled = {
        'generated_ts': time.time(),
        'mission_reason': reason or 'unspecified',
        'num_robots': len(robots),
        'robots': [rb.get('robot') for rb in robots],
        'counts': {
            'either': n_either,        # observing ∪ visited
            'observing': n_observing,
            'visited': n_visited,
        },
        'houses': houses,
        'per_robot_summary': [{
            'robot': rb.get('robot'),
            'completion_reason': rb.get('completion_reason'),
            'coverage_fraction': rb.get('coverage_fraction'),
            'path_length_m': rb.get('path_length_m'),
            'mission_duration_s': rb.get('mission_duration_s'),
            'num_own_aabbs': len(rb.get('confirmed_targets_enu', [])),
            'num_events': len(rb.get('target_events', [])),
        } for rb in robots],
    }

    metrics = {
        'generated_ts': time.time(),
        'mission_reason': reason or 'unspecified',
        'detections': {
            'either': n_either,
            'observing_only': n_observing,
            'visited': n_visited,
        },
        'path': _path_metrics(robots),
    }

    os.makedirs(results_dir, exist_ok=True)
    with open(os.path.join(results_dir, 'compiled_results.json'), 'w') as f:
        json.dump(compiled, f, indent=2)
    with open(os.path.join(results_dir, 'metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=2)
    return compiled


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--results-dir', required=True,
                    help='Dir holding per-robot <robot>.json dumps.')
    ap.add_argument('--expect', type=int, default=0,
                    help='Wait for this many robot files before compiling '
                         '(0 = compile whatever is present now).')
    ap.add_argument('--wait-timeout', type=float, default=30.0,
                    help='Max seconds to wait for --expect files.')
    ap.add_argument('--reason', default=None,
                    help="Mission end reason tag, e.g. 'time_limit'/'coverage'.")
    args = ap.parse_args(argv)

    compiled = compile_results(
        args.results_dir, expect=args.expect,
        wait_timeout=args.wait_timeout, reason=args.reason)

    c = compiled['counts']
    print(f"[compile] {compiled['num_robots']} robot(s) → "
          f"{c['either']} house(s) [either], "
          f"{c['visited']} visited, {c['observing']} observing-only")
    print(f"[compile] wrote compiled_results.json + metrics.json to "
          f"{args.results_dir}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
