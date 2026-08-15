#!/usr/bin/env python3
"""Compare a hardware bag against a sim bag of the same waypoint mission."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path

from bag_io import (
    closest_approach,
    downsample,
    load_bag,
    path_length,
    polyline_cross_track,
    resample_by_arclength,
    samples_to_xyz,
)


def _rmse(errs: list[float]) -> float:
    if not errs:
        return 0.0
    return math.sqrt(sum(e * e for e in errs) / len(errs))


def _track_series(bag, n: int = 200) -> list[dict]:
    t0 = bag.start_ns
    return [
        {
            "t_rel_s": round((s.t_ns - t0) / 1e9, 3),
            "x": round(s.x, 4),
            "y": round(s.y, 4),
            "z": round(s.z, 4),
        }
        for s in downsample(bag.track, n)
    ]


def compare(hw_dir: Path, sim_dir: Path, mission: dict | None) -> dict:
    hw = load_bag(hw_dir)
    sim = load_bag(sim_dir)
    hw_xyz = samples_to_xyz(hw.track)
    sim_xyz = samples_to_xyz(sim.track)

    waypoints = []
    if mission and mission.get("waypoints"):
        waypoints = [(w["x"], w["y"], w["z"]) for w in mission["waypoints"]]
    elif hw.goals:
        waypoints = [(g.x, g.y, g.z) for g in hw.goals]

    # Spatial path comparison (arc-length aligned so hover duration does not dominate).
    n = 200
    hw_rs = resample_by_arclength(hw.track, n)
    sim_rs = resample_by_arclength(sim.track, n)
    pair_err = [
        math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
        for a, b in zip(hw_rs, sim_rs)
    ]
    xy_err = [
        math.hypot(a.x - b.x, a.y - b.y) for a, b in zip(hw_rs, sim_rs)
    ]
    z_err = [abs(a.z - b.z) for a, b in zip(hw_rs, sim_rs)]

    takeoff_alt = 1.0
    if mission:
        takeoff_alt = float(mission.get("takeoff_altitude_m", 1.0))
    elif waypoints:
        takeoff_alt = waypoints[0][2]

    hover_poly = [(0.0, 0.0, 0.0), (0.0, 0.0, takeoff_alt)] + waypoints
    if waypoints:
        hover_poly.append((waypoints[-1][0], waypoints[-1][1], 0.0))

    hw_ct = polyline_cross_track(hw_xyz, hover_poly) if hover_poly else []
    sim_ct = polyline_cross_track(sim_xyz, hover_poly) if hover_poly else []

    wp_rows = []
    for i, wp in enumerate(waypoints, 1):
        hw_d, hw_s = closest_approach(hw.track, wp)
        sim_d, sim_s = closest_approach(sim.track, wp)
        wp_rows.append(
            {
                "index": i,
                "xyz": [round(c, 4) for c in wp],
                "hw_err_m": round(hw_d, 4),
                "sim_err_m": round(sim_d, 4),
                "hw_t_rel_s": round((hw_s.t_ns - hw.start_ns) / 1e9, 3) if hw_s else None,
                "sim_t_rel_s": round((sim_s.t_ns - sim.start_ns) / 1e9, 3) if sim_s else None,
            }
        )

    def _speed_stats(samples):
        speeds = [
            math.sqrt(s.vx * s.vx + s.vy * s.vy + s.vz * s.vz)
            for s in samples
            if (s.vx or s.vy or s.vz)
        ]
        if not speeds:
            return None
        return {
            "max_m_s": round(max(speeds), 3),
            "mean_m_s": round(statistics.mean(speeds), 3),
        }

    report = {
        "hardware": {
            "bag": str(hw_dir),
            "duration_s": round(hw.duration_s, 3),
            "path_length_m": round(path_length(hw_xyz), 3),
            "start_xyz": [round(v, 4) for v in hw_xyz[0]] if hw_xyz else None,
            "end_xyz": [round(v, 4) for v in hw_xyz[-1]] if hw_xyz else None,
            "z_max_m": round(max(p[2] for p in hw_xyz), 3) if hw_xyz else None,
            "cross_track_mean_m": round(statistics.mean(hw_ct), 4) if hw_ct else None,
            "cross_track_max_m": round(max(hw_ct), 4) if hw_ct else None,
            "speed": _speed_stats(hw.odom),
            "pose_topic": hw.pose_topic or hw.odom_topic,
        },
        "simulation": {
            "bag": str(sim_dir),
            "duration_s": round(sim.duration_s, 3),
            "path_length_m": round(path_length(sim_xyz), 3),
            "start_xyz": [round(v, 4) for v in sim_xyz[0]] if sim_xyz else None,
            "end_xyz": [round(v, 4) for v in sim_xyz[-1]] if sim_xyz else None,
            "z_max_m": round(max(p[2] for p in sim_xyz), 3) if sim_xyz else None,
            "cross_track_mean_m": round(statistics.mean(sim_ct), 4) if sim_ct else None,
            "cross_track_max_m": round(max(sim_ct), 4) if sim_ct else None,
            "speed": _speed_stats(sim.odom),
            "pose_topic": sim.pose_topic or sim.odom_topic,
        },
        "comparison": {
            "path_rmse_m": round(_rmse(pair_err), 4),
            "path_max_m": round(max(pair_err), 4) if pair_err else None,
            "xy_rmse_m": round(_rmse(xy_err), 4),
            "z_rmse_m": round(_rmse(z_err), 4),
            "duration_delta_s": round(sim.duration_s - hw.duration_s, 3),
            "path_length_delta_m": round(path_length(sim_xyz) - path_length(hw_xyz), 3),
            "waypoints": wp_rows,
        },
        "hw_path": _track_series(hw, 180),
        "sim_path": _track_series(sim, 180),
        "waypoints": [{"x": x, "y": y, "z": z} for x, y, z in waypoints],
    }
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hw", type=Path, required=True)
    parser.add_argument("--sim", type=Path, required=True)
    parser.add_argument("--mission", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    args = parser.parse_args()
    mission = json.loads(args.mission.read_text()) if args.mission else None
    report = compare(args.hw, args.sim, mission)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    c = report["comparison"]
    print(f"wrote {args.output}")
    print(f"path RMSE {c['path_rmse_m']} m  max {c['path_max_m']} m")
    print(f"XY RMSE {c['xy_rmse_m']} m  Z RMSE {c['z_rmse_m']} m")
    print(
        f"duration hw {report['hardware']['duration_s']}s  "
        f"sim {report['simulation']['duration_s']}s"
    )
    for wp in c["waypoints"]:
        print(
            f"  wp{wp['index']} {wp['xyz']}: "
            f"hw_err={wp['hw_err_m']}m  sim_err={wp['sim_err_m']}m"
        )


if __name__ == "__main__":
    main()
