#!/usr/bin/env python3
"""FP pressure of the CURRENT published semantic cloud, in one number set.

Subscribes ONE message of /robot_N/rayfronts/msg_serv/voxels_sim/all (the
softmax scores raven thresholds) and reports, for sim_0 (the target):

  * how many voxels exceed 0.5 / 0.6 / 0.8,
  * their z-range and a coarse xy-cell histogram of the top offenders —
    i.e. WHERE the map would sprout person boxes right now.

Run inside a robot container:  python3 fp_pressure.py [robot=1] [timeout=30]
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def main():
    robot = sys.argv[1] if len(sys.argv) > 1 else "1"
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    rclpy.init()
    node = Node("fp_pressure_probe")
    got = {}

    def cb(msg):
        got["msg"] = msg

    node.create_subscription(
        PointCloud2, f"/robot_{robot}/rayfronts/msg_serv/voxels_sim/all", cb,
        QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1))
    t0 = time.time()
    while "msg" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
    if "msg" not in got:
        print(f"FP-PRESSURE: no voxels_sim/all message within {timeout:.0f}s "
              "(the mapper republishes every querying.period frames — wait a "
              "cycle and retry)")
        return 1
    msg = got["msg"]
    names = [f.name for f in msg.fields]
    sims = sorted(n for n in names if n.startswith("sim_"))
    if not sims:
        print("FP-PRESSURE: cloud has no sim_ fields")
        return 1
    fields = ("x", "y", "z", sims[0])
    pts = list(pc2.read_points(msg, field_names=fields, skip_nans=True))
    a = np.array([list(p) for p in pts], dtype=np.float32)
    if a.size == 0:
        print("FP-PRESSURE: empty cloud")
        return 1
    # cloud is the robot's LOCAL frame FLU already (msg_serv republishes
    # per-robot); x,y,z here are map-local metres.
    s = a[:, 3]
    n = len(s)
    out = [f"FP-PRESSURE n={n} sim0_mean={s.mean():.3f}"]
    for thr in (0.5, 0.6, 0.8):
        m = s > thr
        c = int(m.sum())
        if c:
            z = a[m, 2]
            out.append(f">{thr}: {c} vox ({100.0*c/n:.1f}%) z[{z.min():.1f},"
                       f"{z.max():.1f}]")
        else:
            out.append(f">{thr}: 0")
    m = s > 0.6
    if m.any():
        cells = {}
        for x, y in a[m, :2].round(-1)[:, :2]:
            cells[(x, y)] = cells.get((x, y), 0) + 1
        top = sorted(cells.items(), key=lambda kv: -kv[1])[:5]
        out.append("hot cells(10m): " + " ".join(
            f"({int(x)},{int(y)})x{c}" for (x, y), c in top))
    print("  ".join(out))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
