#!/usr/bin/env python3
"""Analyze drone1_20260812_030151 rosbag and print a structured report."""

from __future__ import annotations

import math
from collections import defaultdict
from datetime import datetime, timezone

from rosbags.highlevel import AnyReader
from pathlib import Path

BAG = Path("/home/pranavkumara/Desktop/AirStack/tmp_rosbag/drone1_20260812_030151")


def quat_to_yaw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def fmt_ts(ns: int) -> str:
    return datetime.fromtimestamp(ns / 1e9, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f UTC")


def stats(xs):
    if not xs:
        return None
    return {
        "n": len(xs),
        "min": min(xs),
        "max": max(xs),
        "mean": sum(xs) / len(xs),
    }


def main() -> None:
    with AnyReader([BAG]) as reader:
        print("=" * 72)
        print("BAG OVERVIEW")
        print("=" * 72)
        print(f"path:     {BAG}")
        print(f"duration: {reader.duration / 1e9:.3f} s")
        print(f"start:    {fmt_ts(reader.start_time)}")
        print(f"end:      {fmt_ts(reader.end_time)}")
        print(f"messages: {reader.message_count}")
        print()
        print("TOPICS")
        print("-" * 72)
        for c in reader.connections:
            hz = c.msgcount / (reader.duration / 1e9) if reader.duration else 0
            print(f"  {c.topic:50s}  {c.msgtype:40s}  n={c.msgcount:6d}  ~{hz:6.1f} Hz")

        pose_xyz = []
        pose_yaw = []
        pose_t = []
        odom_xyz = []
        odom_v = []
        odom_t = []
        goals = []
        vel_cmds = []
        px4_xyz = []
        px4_t = []
        first_last = {}
        gaps = defaultdict(list)
        last_t = {}

        for conn, timestamp, raw in reader.messages():
            msg = reader.deserialize(raw, conn.msgtype)
            topic = conn.topic
            if topic in last_t:
                dt = (timestamp - last_t[topic]) / 1e9
                gaps[topic].append(dt)
            last_t[topic] = timestamp
            if topic not in first_last:
                first_last[topic] = [timestamp, timestamp]
            else:
                first_last[topic][1] = timestamp

            if topic == "/drone_1/pose":
                p = msg.pose.position
                o = msg.pose.orientation
                pose_xyz.append((p.x, p.y, p.z))
                pose_yaw.append(quat_to_yaw(o.x, o.y, o.z, o.w))
                pose_t.append(timestamp)
            elif topic == "/drone_1/odometry_conversion/odometry":
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                odom_xyz.append((p.x, p.y, p.z))
                odom_v.append((v.x, v.y, v.z))
                odom_t.append(timestamp)
            elif topic == "/svg/drone_1/goal_command":
                p = msg.pose.position
                o = msg.pose.orientation
                goals.append(
                    {
                        "t": timestamp,
                        "frame": msg.header.frame_id,
                        "xyz": (p.x, p.y, p.z),
                        "yaw_deg": math.degrees(quat_to_yaw(o.x, o.y, o.z, o.w)),
                    }
                )
            elif topic == "/drone_1/fmu/velocity_command":
                lin = msg.twist.linear
                ang = msg.twist.angular
                vel_cmds.append(
                    {
                        "t": timestamp,
                        "frame": msg.header.frame_id,
                        "lin": (lin.x, lin.y, lin.z),
                        "ang": (ang.x, ang.y, ang.z),
                    }
                )
            elif topic == "/drone_1/fmu/out/vehicle_odometry":
                # px4_msgs/VehicleOdometry: position[3], velocity[3]
                pos = getattr(msg, "position", None)
                if pos is not None and len(pos) >= 3:
                    px4_xyz.append((float(pos[0]), float(pos[1]), float(pos[2])))
                    px4_t.append(timestamp)

        print()
        print("=" * 72)
        print("PER-TOPIC TIME SPAN / GAPS")
        print("=" * 72)
        for topic, (t0, t1) in first_last.items():
            dts = gaps[topic]
            max_gap = max(dts) if dts else 0
            print(
                f"  {topic}\n"
                f"    first {fmt_ts(t0)}  last {fmt_ts(t1)}  span {(t1-t0)/1e9:.3f}s"
                f"  max_gap {max_gap:.3f}s"
            )

        print()
        print("=" * 72)
        print("GOAL COMMANDS  /svg/drone_1/goal_command")
        print("=" * 72)
        t0 = reader.start_time
        for i, g in enumerate(goals, 1):
            rel = (g["t"] - t0) / 1e9
            x, y, z = g["xyz"]
            print(
                f"  #{i}  t+{rel:7.2f}s  frame={g['frame']!r}  "
                f"xyz=({x:7.3f}, {y:7.3f}, {z:7.3f})  yaw={g['yaw_deg']:6.1f} deg"
            )
        if not goals:
            print("  (none)")

        def path_len(pts):
            d = 0.0
            for a, b in zip(pts, pts[1:]):
                d += math.dist(a, b)
            return d

        print()
        print("=" * 72)
        print("POSE  /drone_1/pose")
        print("=" * 72)
        if pose_xyz:
            xs, ys, zs = zip(*pose_xyz)
            print(f"  samples: {len(pose_xyz)}")
            print(f"  x:  min={min(xs):.3f}  max={max(xs):.3f}  mean={sum(xs)/len(xs):.3f}")
            print(f"  y:  min={min(ys):.3f}  max={max(ys):.3f}  mean={sum(ys)/len(ys):.3f}")
            print(f"  z:  min={min(zs):.3f}  max={max(zs):.3f}  mean={sum(zs)/len(zs):.3f}")
            print(f"  start xyz: ({pose_xyz[0][0]:.3f}, {pose_xyz[0][1]:.3f}, {pose_xyz[0][2]:.3f})")
            print(f"  end   xyz: ({pose_xyz[-1][0]:.3f}, {pose_xyz[-1][1]:.3f}, {pose_xyz[-1][2]:.3f})")
            print(f"  path length: {path_len(pose_xyz):.2f} m")
            print(f"  yaw deg: min={math.degrees(min(pose_yaw)):.1f}  max={math.degrees(max(pose_yaw)):.1f}")
            # downsample path for readability
            step = max(1, len(pose_xyz) // 12)
            print("  downsampled path (t, x, y, z):")
            for i in range(0, len(pose_xyz), step):
                rel = (pose_t[i] - t0) / 1e9
                x, y, z = pose_xyz[i]
                print(f"    t+{rel:7.2f}s  ({x:7.3f}, {y:7.3f}, {z:7.3f})")

        print()
        print("=" * 72)
        print("ODOM  /drone_1/odometry_conversion/odometry")
        print("=" * 72)
        if odom_xyz:
            xs, ys, zs = zip(*odom_xyz)
            vxs, vys, vzs = zip(*odom_v)
            speeds = [math.sqrt(vx * vx + vy * vy + vz * vz) for vx, vy, vz in odom_v]
            print(f"  samples: {len(odom_xyz)}")
            print(f"  x:  min={min(xs):.3f}  max={max(xs):.3f}")
            print(f"  y:  min={min(ys):.3f}  max={max(ys):.3f}")
            print(f"  z:  min={min(zs):.3f}  max={max(zs):.3f}")
            print(f"  start xyz: ({odom_xyz[0][0]:.3f}, {odom_xyz[0][1]:.3f}, {odom_xyz[0][2]:.3f})")
            print(f"  end   xyz: ({odom_xyz[-1][0]:.3f}, {odom_xyz[-1][1]:.3f}, {odom_xyz[-1][2]:.3f})")
            print(f"  path length: {path_len(odom_xyz):.2f} m")
            print(
                f"  speed m/s: min={min(speeds):.3f}  max={max(speeds):.3f}  "
                f"mean={sum(speeds)/len(speeds):.3f}"
            )
            print(f"  vz m/s:    min={min(vzs):.3f}  max={max(vzs):.3f}")

        print()
        print("=" * 72)
        print("VELOCITY COMMANDS  /drone_1/fmu/velocity_command")
        print("=" * 72)
        if vel_cmds:
            lins = [c["lin"] for c in vel_cmds]
            angs = [c["ang"] for c in vel_cmds]
            speeds = [math.sqrt(x * x + y * y + z * z) for x, y, z in lins]
            print(f"  samples: {len(vel_cmds)}  frame={vel_cmds[0]['frame']!r}")
            print(
                f"  |v| m/s: min={min(speeds):.3f}  max={max(speeds):.3f}  "
                f"mean={sum(speeds)/len(speeds):.3f}"
            )
            for axis, idx in (("vx", 0), ("vy", 1), ("vz", 2)):
                vals = [c[idx] for c in lins]
                print(f"  {axis}: min={min(vals):.3f}  max={max(vals):.3f}  mean={sum(vals)/len(vals):.3f}")
            yaw_rates = [c[2] for c in angs]
            print(
                f"  yaw_rate: min={min(yaw_rates):.3f}  max={max(yaw_rates):.3f}  "
                f"mean={sum(yaw_rates)/len(yaw_rates):.3f}"
            )
            # find when commands become non-zero
            first_nz = next((c for c in vel_cmds if math.dist(c["lin"], (0, 0, 0)) > 1e-3), None)
            last_nz = next((c for c in reversed(vel_cmds) if math.dist(c["lin"], (0, 0, 0)) > 1e-3), None)
            if first_nz:
                print(f"  first nonzero cmd t+{(first_nz['t']-t0)/1e9:.2f}s  lin={first_nz['lin']}")
            if last_nz:
                print(f"  last  nonzero cmd t+{(last_nz['t']-t0)/1e9:.2f}s  lin={last_nz['lin']}")
            # sample a few commands around goals
            print("  samples around bag:")
            idxs = [0, len(vel_cmds) // 4, len(vel_cmds) // 2, 3 * len(vel_cmds) // 4, -1]
            for i in idxs:
                c = vel_cmds[i]
                rel = (c["t"] - t0) / 1e9
                print(f"    t+{rel:7.2f}s  lin={c['lin']}  ang={c['ang']}")

        print()
        print("=" * 72)
        print("PX4 VEHICLE ODOM  /drone_1/fmu/out/vehicle_odometry")
        print("=" * 72)
        if px4_xyz:
            xs, ys, zs = zip(*px4_xyz)
            print(f"  samples: {len(px4_xyz)}")
            print(f"  x:  min={min(xs):.3f}  max={max(xs):.3f}")
            print(f"  y:  min={min(ys):.3f}  max={max(ys):.3f}")
            print(f"  z:  min={min(zs):.3f}  max={max(zs):.3f}")
            print(f"  start xyz: ({px4_xyz[0][0]:.3f}, {px4_xyz[0][1]:.3f}, {px4_xyz[0][2]:.3f})")
            print(f"  end   xyz: ({px4_xyz[-1][0]:.3f}, {px4_xyz[-1][1]:.3f}, {px4_xyz[-1][2]:.3f})")
            print(f"  path length: {path_len(px4_xyz):.2f} m")

        # Compare pose vs odom at overlapping times (nearest)
        print()
        print("=" * 72)
        print("POSE vs ODOM POSITION DELTA (nearest-time, every ~2s)")
        print("=" * 72)
        if pose_xyz and odom_xyz:
            j = 0
            for i, t in enumerate(pose_t):
                rel = (t - t0) / 1e9
                if i != 0 and (rel % 2) > 0.05 and i != len(pose_t) - 1:
                    continue
                while j + 1 < len(odom_t) and abs(odom_t[j + 1] - t) < abs(odom_t[j] - t):
                    j += 1
                dx = pose_xyz[i][0] - odom_xyz[j][0]
                dy = pose_xyz[i][1] - odom_xyz[j][1]
                dz = pose_xyz[i][2] - odom_xyz[j][2]
                print(
                    f"  t+{rel:7.2f}s  dxyz=({dx:7.3f}, {dy:7.3f}, {dz:7.3f})  "
                    f"|d|={math.sqrt(dx*dx+dy*dy+dz*dz):.3f}"
                )


if __name__ == "__main__":
    main()
