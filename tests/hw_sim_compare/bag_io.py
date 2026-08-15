"""Shared rosbag2/MCAP helpers for hardware-vs-sim comparison."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

from rosbags.highlevel import AnyReader


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def fmt_ts(ns: int) -> str:
    return datetime.fromtimestamp(ns / 1e9, tz=timezone.utc).strftime(
        "%Y-%m-%d %H:%M:%S.%f UTC"
    )


def path_length(pts: list[tuple[float, float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def downsample(xs: list, n: int) -> list:
    if len(xs) <= n:
        return list(xs)
    if n <= 1:
        return [xs[0]]
    out = []
    for i in range(n):
        idx = round(i * (len(xs) - 1) / (n - 1))
        out.append(xs[idx])
    return out


@dataclass
class PoseSample:
    t_ns: int
    x: float
    y: float
    z: float
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


@dataclass
class GoalSample:
    t_ns: int
    x: float
    y: float
    z: float
    yaw: float
    frame: str


@dataclass
class BagTracks:
    path: Path
    duration_s: float
    start_ns: int
    end_ns: int
    topics: dict[str, int]
    pose: list[PoseSample] = field(default_factory=list)
    odom: list[PoseSample] = field(default_factory=list)
    goals: list[GoalSample] = field(default_factory=list)
    pose_topic: str = ""
    odom_topic: str = ""
    goal_topic: str = ""

    @property
    def track(self) -> list[PoseSample]:
        """Preferred world-frame position track (mocap pose, else converted odom)."""
        return self.pose or self.odom


def _is_pose_topic(name: str) -> bool:
    return name.endswith("/pose") and "goal" not in name and "command" not in name


def _is_odom_topic(name: str) -> bool:
    return name.endswith("/odometry_conversion/odometry") or name.endswith(
        "/odometry"
    ) and "vehicle_odometry" not in name and "mavros" not in name


def _is_goal_topic(name: str) -> bool:
    return name.endswith("/goal_command") or name.endswith("/global_plan")


def load_bag(bag_dir: Path) -> BagTracks:
    bag_dir = Path(bag_dir)
    with AnyReader([bag_dir]) as reader:
        topics = {c.topic: c.msgcount for c in reader.connections}
        pose_topic = next((c.topic for c in reader.connections if _is_pose_topic(c.topic)), "")
        odom_topic = next((c.topic for c in reader.connections if _is_odom_topic(c.topic)), "")
        goal_topic = next((c.topic for c in reader.connections if _is_goal_topic(c.topic)), "")

        tracks = BagTracks(
            path=bag_dir,
            duration_s=reader.duration / 1e9,
            start_ns=reader.start_time,
            end_ns=reader.end_time,
            topics=topics,
            pose_topic=pose_topic,
            odom_topic=odom_topic,
            goal_topic=goal_topic,
        )

        for conn, timestamp, raw in reader.messages():
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == pose_topic:
                p = msg.pose.position
                o = msg.pose.orientation
                tracks.pose.append(
                    PoseSample(
                        timestamp, p.x, p.y, p.z,
                        quat_to_yaw(o.x, o.y, o.z, o.w),
                    )
                )
            elif conn.topic == odom_topic:
                p = msg.pose.pose.position
                o = msg.pose.pose.orientation
                v = msg.twist.twist.linear
                tracks.odom.append(
                    PoseSample(
                        timestamp, p.x, p.y, p.z,
                        quat_to_yaw(o.x, o.y, o.z, o.w),
                        v.x, v.y, v.z,
                    )
                )
            elif conn.topic == goal_topic and "PoseStamped" in conn.msgtype:
                p = msg.pose.position
                o = msg.pose.orientation
                tracks.goals.append(
                    GoalSample(
                        timestamp, p.x, p.y, p.z,
                        quat_to_yaw(o.x, o.y, o.z, o.w),
                        msg.header.frame_id,
                    )
                )
        return tracks


def samples_to_xyz(samples: list[PoseSample]) -> list[tuple[float, float, float]]:
    return [(s.x, s.y, s.z) for s in samples]


def closest_approach(
    track: list[PoseSample], goal: tuple[float, float, float]
) -> tuple[float, PoseSample | None]:
    best_d = float("inf")
    best = None
    for s in track:
        d = math.dist((s.x, s.y, s.z), goal)
        if d < best_d:
            best_d = d
            best = s
    return best_d, best


def polyline_cross_track(
    pts: list[tuple[float, float, float]],
    poly: list[tuple[float, float, float]],
) -> list[float]:
    """3D distance from each point to the closest point on the polyline."""
    if len(poly) < 2:
        return [math.dist(p, poly[0]) if poly else 0.0 for p in pts]
    errs = []
    for p in pts:
        best = float("inf")
        for a, b in zip(poly, poly[1:]):
            ax, ay, az = a
            bx, by, bz = b
            vx, vy, vz = bx - ax, by - ay, bz - az
            wx, wy, wz = p[0] - ax, p[1] - ay, p[2] - az
            denom = vx * vx + vy * vy + vz * vz
            t = 0.0 if denom == 0 else max(0.0, min(1.0, (wx * vx + wy * vy + wz * vz) / denom))
            proj = (ax + t * vx, ay + t * vy, az + t * vz)
            best = min(best, math.dist(p, proj))
        errs.append(best)
    return errs


def resample_by_arclength(
    samples: list[PoseSample], n: int = 200
) -> list[PoseSample]:
    if len(samples) <= 1 or n <= 1:
        return list(samples)
    xyz = samples_to_xyz(samples)
    segs = [math.dist(a, b) for a, b in zip(xyz, xyz[1:])]
    total = sum(segs)
    if total <= 1e-6:
        return downsample(samples, n)
    cum = [0.0]
    for s in segs:
        cum.append(cum[-1] + s)
    out: list[PoseSample] = []
    j = 0
    for i in range(n):
        target = total * i / (n - 1)
        while j + 1 < len(cum) and cum[j + 1] < target:
            j += 1
        if j + 1 >= len(samples):
            out.append(samples[-1])
            continue
        span = cum[j + 1] - cum[j]
        t = 0.0 if span <= 1e-9 else (target - cum[j]) / span
        a, b = samples[j], samples[j + 1]
        out.append(
            PoseSample(
                int(a.t_ns + t * (b.t_ns - a.t_ns)),
                a.x + t * (b.x - a.x),
                a.y + t * (b.y - a.y),
                a.z + t * (b.z - a.z),
                a.yaw + t * (b.yaw - a.yaw),
            )
        )
    return out
