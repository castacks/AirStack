#!/usr/bin/env python3
"""Extract a replayable mission spec from a hardware rosbag."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from bag_io import (
    closest_approach,
    downsample,
    fmt_ts,
    load_bag,
    path_length,
    samples_to_xyz,
)


def extract(bag_dir: Path) -> dict:
    bag = load_bag(bag_dir)
    track = bag.track
    t0 = bag.start_ns
    xyz = samples_to_xyz(track)

    takeoff_vz = 1.0
    land_vz = 0.8
    if bag.odom:
        vzs = [s.vz for s in bag.odom]
        if vzs:
            takeoff_vz = max(0.3, min(2.0, max(vzs)))
            land_vz = max(0.3, min(2.0, abs(min(vzs))))

    hover_z = 1.0
    if track:
        zs = [s.z for s in track]
        hover_z = max(zs)

    waypoints = []
    for g in bag.goals:
        d, closest = closest_approach(track, (g.x, g.y, g.z))
        waypoints.append(
            {
                "x": round(g.x, 4),
                "y": round(g.y, 4),
                "z": round(g.z, 4),
                "yaw_deg": round(math.degrees(g.yaw), 2),
                "frame": g.frame or "map",
                "t_rel_s": round((g.t_ns - t0) / 1e9, 3),
                "hw_closest_err_m": round(d, 4),
                "hw_closest_t_rel_s": round((closest.t_ns - t0) / 1e9, 3) if closest else None,
            }
        )

    path_ds = downsample(track, 250)
    return {
        "source_bag": str(bag_dir),
        "start": fmt_ts(bag.start_ns),
        "end": fmt_ts(bag.end_ns),
        "duration_s": round(bag.duration_s, 3),
        "topics": bag.topics,
        "pose_topic": bag.pose_topic,
        "odom_topic": bag.odom_topic,
        "goal_topic": bag.goal_topic,
        "takeoff_altitude_m": round(max(g["z"] for g in waypoints), 3) if waypoints else round(hover_z, 3),
        "takeoff_velocity_m_s": 1.0,
        "land_velocity_m_s": round(land_vz, 3),
        "goal_tolerance_m": 0.3,
        "hw_observed_takeoff_vz_m_s": round(takeoff_vz, 3),
        "path_length_m": round(path_length(xyz), 3),
        "start_xyz": [round(v, 4) for v in xyz[0]] if xyz else None,
        "end_xyz": [round(v, 4) for v in xyz[-1]] if xyz else None,
        "waypoints": waypoints,
        "reference_path": [
            {
                "t_rel_s": round((s.t_ns - t0) / 1e9, 3),
                "x": round(s.x, 4),
                "y": round(s.y, 4),
                "z": round(s.z, 4),
            }
            for s in path_ds
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory")
    parser.add_argument("-o", "--output", type=Path, required=True)
    args = parser.parse_args()
    mission = extract(args.bag)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(mission, indent=2) + "\n")
    print(f"wrote {args.output}")
    print(f"duration {mission['duration_s']}s  path {mission['path_length_m']}m")
    print(f"waypoints: {len(mission['waypoints'])}")
    for i, wp in enumerate(mission["waypoints"], 1):
        print(
            f"  {i}: ({wp['x']}, {wp['y']}, {wp['z']})  "
            f"t+{wp['t_rel_s']}s  hw_err={wp['hw_closest_err_m']}m"
        )


if __name__ == "__main__":
    main()
