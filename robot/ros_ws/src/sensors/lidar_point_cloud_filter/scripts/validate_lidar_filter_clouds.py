#!/usr/bin/env python3
# Copyright 2026 AirLab CMU
# SPDX-License-Identifier: Apache-2.0
"""One-shot ROS 2 check for liveliness: filtered LiDAR cloud vs raw (Isaac / Pegasus).

Run inside the robot container with workspace sourced and ROS_DOMAIN_ID set::

  python3 /root/AirStack/robot/ros_ws/src/sensors/lidar_point_cloud_filter/scripts/validate_lidar_filter_clouds.py --robot-num 1

Checks (after receiving a non-empty filtered cloud):
  * Filtered ``.../point_cloud``: all coordinates finite; **minimum range** must be at
    least ``near_range_m`` from the running filter node (minus tolerance). The
    tolerance is ``max(0.05 m, 5% of near_range_m)`` — the ``0.05`` is **slack
    around the configured near range**, not a standalone “no points within 5 cm”
    rule. At least one return beyond 2 m so long-range points are not stripped.
  * Raw ``.../point_cloud_raw`` (optional): if present and contains near-field
    returns below ``near_range_m``, the filtered cloud must still respect the
    near-range floor (filter removes self-hits / clutter; NaNs skipped when reading).

Exit 0 on success, 1 on validation failure, 2 on ROS/runtime errors.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def _read_near_range_m(robot_num: int) -> float:
    """Query ``near_range_m`` from the running filter node; default 0.75."""
    rid = os.environ.get('ROS_DOMAIN_ID', '1')
    inner = (
        'set -e; source /opt/ros/jazzy/setup.bash; '
        'source /root/AirStack/robot/ros_ws/install/setup.bash; '
        f'export ROS_DOMAIN_ID={rid}; '
        f'ros2 param get /robot_{robot_num}/sensors/lidar_point_cloud_filter near_range_m'
    )
    try:
        proc = subprocess.run(
            ['bash', '-c', inner],
            capture_output=True,
            text=True,
            timeout=20,
        )
    except (subprocess.TimeoutExpired, OSError):
        return 0.75
    if proc.returncode != 0:
        return 0.75
    m = re.search(r'[\d.]+', proc.stdout)
    try:
        return float(m.group(0)) if m else 0.75
    except ValueError:
        return 0.75


def _ranges_xyz(msg: PointCloud2) -> np.ndarray | None:
    names = {f.name for f in msg.fields}
    if not {'x', 'y', 'z'}.issubset(names):
        return None
    pts = list(
        point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    )
    if not pts:
        return np.array([], dtype=np.float64)
    arr = np.array([(float(p[0]), float(p[1]), float(p[2])) for p in pts], dtype=np.float64)
    if not np.isfinite(arr).all():
        return None
    return np.linalg.norm(arr, axis=1)


def _wait_for_cloud(
    node: Node, topic: str, timeout_s: float, reliable: bool
) -> PointCloud2 | None:
    qos = QoSProfile(
        depth=10,
        reliability=(
            QoSReliabilityPolicy.RELIABLE
            if reliable
            else QoSReliabilityPolicy.BEST_EFFORT
        ),
    )
    holder: list[PointCloud2] = []

    def _cb(msg: PointCloud2) -> None:
        if not holder:
            holder.append(msg)

    sub = node.create_subscription(PointCloud2, topic, _cb, qos)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not holder:
        rclpy.spin_once(node, timeout_sec=0.5)
    sub.destroy()
    return holder[0] if holder else None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--robot-num', type=int, required=True)
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument(
        '--qos-best-effort',
        action='store_true',
        help='Subscribe with BEST_EFFORT (default: RELIABLE to match filter config)',
    )
    args = ap.parse_args()
    n = args.robot_num
    filt_topic = f'/robot_{n}/sensors/ouster/point_cloud'
    raw_topic = f'/robot_{n}/sensors/ouster/point_cloud_raw'
    reliable = not args.qos_best_effort

    near_range_m = _read_near_range_m(n)
    tol = max(0.05, near_range_m * 0.05)

    node = None
    try:
        rclpy.init()
        node = Node('validate_lidar_filter_clouds')
        fmsg = _wait_for_cloud(node, filt_topic, args.timeout, reliable)
        if fmsg is None:
            print(f'ERROR: no message on {filt_topic} within {args.timeout}s', file=sys.stderr)
            return 1

        fr = _ranges_xyz(fmsg)
        if fr is None:
            print('ERROR: filtered cloud has non-finite coordinates or missing xyz', file=sys.stderr)
            return 1
        if fr.size == 0:
            print('ERROR: filtered cloud is empty', file=sys.stderr)
            return 1

        mn_f = float(fr.min())
        if mn_f < near_range_m - tol:
            print(
                f'ERROR: filtered min range {mn_f:.4f}m < near_range_m ({near_range_m}) - tol {tol:.4f}',
                file=sys.stderr,
            )
            return 1

        if float(fr.max()) < 2.0:
            print(
                f'ERROR: expected long-range returns; filtered max range {float(fr.max()):.4f}m',
                file=sys.stderr,
            )
            return 1

        rmsg = _wait_for_cloud(node, raw_topic, min(30.0, args.timeout), reliable)
        if rmsg is not None:
            rr = _ranges_xyz(rmsg)
            if rr is not None and rr.size > 0:
                mn_r = float(rr.min())
                if mn_r < near_range_m - tol:
                    print(
                        f'INFO: raw min range {mn_r:.4f}m (< near_range_m); '
                        f'filtered min {mn_f:.4f}m (must stay >= near_range_m)',
                    )
                if mn_r < near_range_m - tol and mn_f < near_range_m - tol:
                    print(
                        'ERROR: raw has near-field clutter but filtered min still below near_range_m',
                        file=sys.stderr,
                    )
                    return 1

        print(
            f'OK robot_{n}: near_range_m={near_range_m:.3f} filtered '
            f'points={fr.size} min_r={mn_f:.3f}m max_r={float(fr.max()):.3f}m',
        )
        return 0
    except Exception as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
