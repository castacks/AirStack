#!/usr/bin/env python3
"""Standalone ordered-waypoint track checker.

Judges whether an odometry track visited an ordered list of waypoints, each
within a distance tolerance and a per-waypoint time budget. Used by
``tests/system/test_waypoint_flight.py``, but deliberately dependency-free
(stdlib only) and runnable outside the AirStack harness: success is defined
purely on the odometry track, not on any AirStack-specific interface, so the
same checker can judge waypoint flight on any ROS 2 system that can dump
odometry to CSV::

    ros2 topic echo --csv /robot_1/interface/mavros/local_position/odom > odom.csv
    python3 waypoint_checker.py --odom-csv odom.csv \
        --waypoints "10,0,10; 10,10,10; 0,10,10" --tolerance 1.5 --budget 120

Exit code 0 iff every waypoint was reached in order within tolerance and
budget; a JSON verdict is printed to stdout either way.

Input CSV format: the flattened ``ros2 topic echo --csv`` output of
``nav_msgs/Odometry`` (header stamp in columns 0-1, position x/y/z in columns
4-6). Lines that do not parse (ros2 banners, partial writes) are skipped.
"""

import argparse
import json
import math
import sys

# Column indices in `ros2 topic echo --csv` output for nav_msgs/Odometry
# (all primitives flattened in declaration order).
_COL_STAMP_SEC = 0
_COL_STAMP_NSEC = 1
_COL_POS_X = 4
_COL_POS_Y = 5
_COL_POS_Z = 6
# A full Odometry row has 4 header/frame fields + 7 pose + 36 cov + 6 twist
# + 36 cov = 89 columns; require at least position columns to be present.
_MIN_COLUMNS = 7


def parse_odom_csv(path):
    """Parse a ros2 ``--csv`` odometry dump into [(t, x, y, z), ...] rows."""
    rows = []
    with open(path) as fh:
        for line in fh:
            parts = line.strip().split(",")
            if len(parts) < _MIN_COLUMNS:
                continue
            try:
                t = float(parts[_COL_STAMP_SEC]) + float(parts[_COL_STAMP_NSEC]) * 1e-9
                x = float(parts[_COL_POS_X])
                y = float(parts[_COL_POS_Y])
                z = float(parts[_COL_POS_Z])
            except ValueError:
                continue
            rows.append((t, x, y, z))
    return rows


def parse_waypoints(spec):
    """Parse ``"x,y,z; x,y,z; ..."`` into [(x, y, z), ...]."""
    waypoints = []
    for chunk in spec.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [p.strip() for p in chunk.split(",")]
        if len(parts) != 3:
            raise ValueError(f"waypoint {chunk!r} is not 'x,y,z'")
        waypoints.append(tuple(float(p) for p in parts))
    if not waypoints:
        raise ValueError("no waypoints given")
    return waypoints


def check_track(rows, waypoints, tolerance_m, budget_s_per_waypoint):
    """Check that the track visits every waypoint in order.

    A waypoint counts as reached at the first sample (searching forward from
    the previous waypoint's arrival) within ``tolerance_m`` of it, provided
    that sample's time is within ``budget_s_per_waypoint`` of the previous
    arrival (or of track start, for the first waypoint). Times are whatever
    clock stamped the odometry (sim time in AirStack runs).

    Returns a verdict dict; ``verdict["success"]`` is the pass/fail judgment.
    """
    verdict = {
        "success": False,
        "num_odom_samples": len(rows),
        "tolerance_m": tolerance_m,
        "budget_s_per_waypoint": budget_s_per_waypoint,
        "waypoints": [],
    }
    if not rows:
        verdict["error"] = "no odometry samples"
        return verdict

    start_idx = 0
    prev_arrival_t = rows[0][0]
    all_reached = True

    for wi, (wx, wy, wz) in enumerate(waypoints):
        # arrival = FIRST sample within tolerance (preserves ordering
        # semantics); closest_approach = true minimum over the whole
        # remaining track, so the report reflects how near the drone
        # actually got, not just the tolerance boundary crossing.
        arrival_idx = None
        closest = math.inf
        for i in range(start_idx, len(rows)):
            t, x, y, z = rows[i]
            d = math.sqrt((x - wx) ** 2 + (y - wy) ** 2 + (z - wz) ** 2)
            if d < closest:
                closest = d
            if arrival_idx is None and d <= tolerance_m:
                arrival_idx = i

        entry = {
            "index": wi,
            "target": [wx, wy, wz],
            "closest_approach_m": round(closest, 3),
            "reached": arrival_idx is not None,
        }
        if arrival_idx is not None:
            arrival_t = rows[arrival_idx][0]
            elapsed = arrival_t - prev_arrival_t
            entry["elapsed_from_prev_s"] = round(elapsed, 3)
            entry["within_budget"] = elapsed <= budget_s_per_waypoint
            if not entry["within_budget"]:
                all_reached = False
            start_idx = arrival_idx
            prev_arrival_t = arrival_t
        else:
            all_reached = False
            # Keep evaluating later waypoints from the same index so the
            # verdict reports closest approaches for all of them.
        verdict["waypoints"].append(entry)

    verdict["total_time_s"] = round(rows[-1][0] - rows[0][0], 3)
    verdict["success"] = all_reached
    return verdict


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--odom-csv", required=True,
                    help="ros2 topic echo --csv dump of nav_msgs/Odometry")
    ap.add_argument("--waypoints", required=True,
                    help="Ordered waypoints 'x,y,z; x,y,z; ...' in the odometry frame")
    ap.add_argument("--tolerance", type=float, default=1.5,
                    help="Pass distance to each waypoint in meters (default 1.5)")
    ap.add_argument("--budget", type=float, default=120.0,
                    help="Time budget per waypoint in seconds of odometry "
                         "clock (default 120)")
    args = ap.parse_args(argv)

    rows = parse_odom_csv(args.odom_csv)
    waypoints = parse_waypoints(args.waypoints)
    verdict = check_track(rows, waypoints, args.tolerance, args.budget)
    print(json.dumps(verdict, indent=2))
    return 0 if verdict["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
