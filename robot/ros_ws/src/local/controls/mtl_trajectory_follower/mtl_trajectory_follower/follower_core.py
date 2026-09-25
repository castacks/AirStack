"""Arc-length carrot pursuit + scheduled gimbal pointing (pure Python, no ROS).

Port of the drone-planning-testbed's ``SearchPlanner`` execution logic
(``testbed/planners/search_base.py``) onto AirStack's cascaded controller.

Why carrot-following indexed by FLOWN ARC LENGTH rather than by time: a
single-axis gimbal schedule is defined against the planned track, not the clock.
Progress is measured by projecting the vehicle onto the track (forward window
only, so adjacent serpentine lanes never capture it), the setpoint is the point
``L = 1.2 R_min`` further along, and the gimbal is aimed from the vehicle's
ACTUAL position at the ground point the plan wanted looked at *at that arc
position*. So a speed mismatch stretches the sweep in time but not in space,
and tracking error does not become pointing error.

States: INGRESS (fly to the track start at mission altitude) -> SEARCH ->
COMPLETE (hold at the end). ABORTED is decided by the node.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Sequence

from mtl_trajectory_follower.gimbal_math import (single_axis_command, slew_limit, two_axis_command,
                                                 wrap_pi)

IDLE, INGRESS, SEARCH, COMPLETE, ABORTED = 0, 1, 2, 3, 4
STATE_NAMES = {IDLE: "IDLE", INGRESS: "INGRESS", SEARCH: "SEARCH", COMPLETE: "COMPLETE",
               ABORTED: "ABORTED"}

__all__ = ["Track", "FollowerConfig", "FollowerOutput", "TrackFollower", "STATE_NAMES",
           "IDLE", "INGRESS", "SEARCH", "COMPLETE", "ABORTED"]


@dataclass
class Track:
    """One agent's planned sortie in the robot's map frame (all lists length N)."""

    x: list[float]
    y: list[float]
    z: list[float]
    yaw: list[float]
    speed: list[float]
    bx: list[float]
    by: list[float]
    bz: list[float]
    arc: list[float] = field(default_factory=list)
    t: list[float] = field(default_factory=list)
    phi: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        n = len(self.x)
        if n < 2:
            raise ValueError(f"a track needs at least 2 samples (got {n})")
        for name in ("y", "z", "yaw", "speed", "bx", "by", "bz"):
            if len(getattr(self, name)) != n:
                raise ValueError(f"track field {name!r} has {len(getattr(self, name))} samples, expected {n}")
        if not self.arc:
            self.arc = [0.0]
            for k in range(1, n):
                self.arc.append(self.arc[-1] + math.hypot(self.x[k] - self.x[k - 1], self.y[k] - self.y[k - 1]))
        if len(self.arc) != n:
            raise ValueError("arc length array does not match the track")

    def __len__(self) -> int:
        return len(self.x)

    @property
    def total(self) -> float:
        return self.arc[-1]

    @property
    def step_m(self) -> float:
        steps = sorted(b - a for a, b in zip(self.arc, self.arc[1:]) if b > a)
        return steps[len(steps) // 2] if steps else 1.0

    # interpolation at an arc length -------------------------------------- #
    def _locate(self, s: float) -> tuple[int, float]:
        s = min(max(s, 0.0), self.total)
        k = bisect.bisect_right(self.arc, s) - 1
        k = min(max(k, 0), len(self) - 2)
        seg = self.arc[k + 1] - self.arc[k]
        w = 0.0 if seg <= 0 else (s - self.arc[k]) / seg
        return k, min(max(w, 0.0), 1.0)

    def position_at(self, s: float) -> tuple[float, float, float]:
        k, w = self._locate(s)
        return (self.x[k] + w * (self.x[k + 1] - self.x[k]),
                self.y[k] + w * (self.y[k + 1] - self.y[k]),
                self.z[k] + w * (self.z[k + 1] - self.z[k]))

    def boresight_at(self, s: float) -> tuple[float, float, float]:
        k, w = self._locate(s)
        return (self.bx[k] + w * (self.bx[k + 1] - self.bx[k]),
                self.by[k] + w * (self.by[k + 1] - self.by[k]),
                self.bz[k] + w * (self.bz[k + 1] - self.bz[k]))

    def yaw_at(self, s: float) -> float:
        k, w = self._locate(s)
        return wrap_pi(self.yaw[k] + w * wrap_pi(self.yaw[k + 1] - self.yaw[k]))

    def speed_at(self, s: float) -> float:
        k, w = self._locate(s)
        return self.speed[k] + w * (self.speed[k + 1] - self.speed[k])

    def tangent_at(self, s: float) -> tuple[float, float, float]:
        k, _ = self._locate(s)
        dx, dy, dz = self.x[k + 1] - self.x[k], self.y[k + 1] - self.y[k], self.z[k + 1] - self.z[k]
        n = math.sqrt(dx * dx + dy * dy + dz * dz)
        if n < 1e-9:
            yaw = self.yaw[k]
            return (math.cos(yaw), math.sin(yaw), 0.0)
        return (dx / n, dy / n, dz / n)


@dataclass
class FollowerConfig:
    lookahead_m: float = 0.0            # 0 -> lookahead_turn_radii * R_min
    lookahead_turn_radii: float = 1.2
    min_turn_radius_m: float = 12.0
    window_lookaheads: float = 1.5      # forward projection window, in lookaheads: the vehicle
                                        # chases a carrot L ahead, so progress cannot jump further;
                                        # a wider window lets a close serpentine lane capture it
    min_window_samples: int = 20
    finish_tolerance_m: float = 3.0
    ingress_tolerance_m: float = 4.0
    ingress_alt_tolerance_m: float = 2.0
    gimbal_lead_s: float = 0.2          # aim ahead to cover command latency
    yaw_lead_s: float = 0.5             # heading reference ahead of the vehicle
    single_axis: bool = True
    tilt_rad: float = math.radians(30.0)
    gimbal_max_rad: float = math.radians(80.0)
    gimbal_rate_rad_s: float = math.radians(120.0)
    pitch_nudge_max_rad: float = math.radians(5.0)
    two_axis_rate_rad_s: float = math.radians(120.0)
    speed_mps: float = 6.0

    @property
    def lookahead(self) -> float:
        return self.lookahead_m if self.lookahead_m > 0 else self.lookahead_turn_radii * self.min_turn_radius_m


@dataclass
class FollowerOutput:
    state: int
    carrot: tuple[float, float, float]
    carrot_yaw: float
    carrot_velocity: tuple[float, float, float]
    aim: tuple[float, float, float]
    gimbal: tuple[float, float, float]      # (roll, pitch, yaw) earth frame [rad]
    progress_m: float
    remaining_m: float
    cross_track_error_m: float
    track_index: int
    gimbal_diag: dict

    @property
    def state_name(self) -> str:
        return STATE_NAMES[self.state]


class TrackFollower:
    """Stateful follower for one sortie; ``step()`` at the control rate."""

    def __init__(self, track: Track, cfg: FollowerConfig) -> None:
        self.track = track
        self.cfg = cfg
        self.state = IDLE
        self.idx = 0
        self.progress = 0.0
        self._gimbal: tuple[float, float, float] | None = None
        step = max(track.step_m, 1e-3)
        self.window = max(int(round(cfg.window_lookaheads * cfg.lookahead / step)), cfg.min_window_samples)

    # ------------------------------------------------------------------ #
    def start(self, pos: Sequence[float]) -> int:
        """Pick INGRESS or SEARCH from where the vehicle is when the plan arrives."""
        self.idx, self.progress, _ = self._project(pos, 0, self.window)
        self.state = SEARCH if self._at_start(pos) else INGRESS
        if self.state == INGRESS:
            self.idx, self.progress = 0, 0.0
        return self.state

    def _at_start(self, pos: Sequence[float]) -> bool:
        tr = self.track
        return (math.hypot(pos[0] - tr.x[0], pos[1] - tr.y[0]) <= self.cfg.ingress_tolerance_m
                and abs(pos[2] - tr.z[0]) <= self.cfg.ingress_alt_tolerance_m)

    def _project(self, pos: Sequence[float], lo: int, hi: int) -> tuple[int, float, float]:
        """Closest point on segments [lo, hi) -> (segment index, arc length, distance)."""
        tr = self.track
        lo = max(lo, 0)
        hi = max(min(hi, len(tr) - 1), lo + 1)
        best = (lo, tr.arc[lo], float("inf"))
        px, py = pos[0], pos[1]
        for k in range(lo, hi):
            ax, ay = tr.x[k], tr.y[k]
            dx, dy = tr.x[k + 1] - ax, tr.y[k + 1] - ay
            seg2 = dx * dx + dy * dy
            w = 0.0 if seg2 <= 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg2))
            cx, cy = ax + w * dx, ay + w * dy
            d = math.hypot(px - cx, py - cy)
            if d < best[2] - 1e-9:
                best = (k, tr.arc[k] + w * (tr.arc[k + 1] - tr.arc[k]), d)
        return best

    # ------------------------------------------------------------------ #
    def step(self, pos: Sequence[float], vehicle_yaw: float, dt: float) -> FollowerOutput:
        tr, cfg = self.track, self.cfg
        L = cfg.lookahead
        xte = 0.0
        diag: dict = {}
        if self.state == IDLE:
            self.start(pos)

        if self.state == INGRESS:
            if self._at_start(pos):
                self.state = SEARCH
            else:
                sx, sy, sz = tr.x[0], tr.y[0], tr.z[0]
                dx, dy = sx - pos[0], sy - pos[1]
                dist = math.hypot(dx, dy)
                if dist > L:
                    carrot = (pos[0] + L * dx / dist, pos[1] + L * dy / dist, sz)
                    heading = math.atan2(dy, dx)
                else:
                    carrot = (sx, sy, sz)
                    heading = tr.yaw[0] if dist < 2.0 * cfg.ingress_tolerance_m else math.atan2(dy, dx)
                v = cfg.speed_mps
                vel = (v * dx / dist, v * dy / dist, 0.0) if dist > 1e-6 else (0.0, 0.0, 0.0)
                aim = tr.boresight_at(0.0)
                gimbal = self._gimbal_command(pos, aim, vehicle_yaw, dt, diag)
                xte = math.hypot(dx, dy)
                return FollowerOutput(INGRESS, carrot, heading, vel, aim, gimbal, 0.0, tr.total,
                                      xte, 0, diag)

        if self.state == SEARCH:
            k, s, xte = self._project(pos, self.idx, self.idx + self.window)
            if s >= self.progress:  # forward only
                self.idx, self.progress = k, s
            if tr.total - self.progress <= cfg.finish_tolerance_m:
                self.state = COMPLETE

        if self.state == COMPLETE:
            end = (tr.x[-1], tr.y[-1], tr.z[-1])
            aim = (tr.bx[-1], tr.by[-1], tr.bz[-1])
            gimbal = self._gimbal_command(pos, aim, vehicle_yaw, dt, diag)
            xte = math.hypot(pos[0] - end[0], pos[1] - end[1])
            return FollowerOutput(COMPLETE, end, tr.yaw[-1], (0.0, 0.0, 0.0), aim, gimbal,
                                  tr.total, 0.0, xte, len(tr) - 1, diag)

        # SEARCH: carrot L ahead of the projected progress
        s_car = min(self.progress + L, tr.total)
        carrot = tr.position_at(s_car)
        v = tr.speed_at(self.progress) or cfg.speed_mps
        tx, ty, tz = tr.tangent_at(s_car)
        vel = (v * tx, v * ty, v * tz)
        heading = tr.yaw_at(self.progress + cfg.yaw_lead_s * v)
        aim = tr.boresight_at(self.progress + cfg.gimbal_lead_s * v)
        gimbal = self._gimbal_command(pos, aim, vehicle_yaw, dt, diag)
        return FollowerOutput(SEARCH, carrot, heading, vel, aim, gimbal, self.progress,
                              tr.total - self.progress, xte, self.idx, diag)

    # ------------------------------------------------------------------ #
    def _gimbal_command(self, pos, aim, vehicle_yaw, dt, diag) -> tuple[float, float, float]:
        cfg = self.cfg
        if cfg.single_axis:
            cmd, d = single_axis_command(pos, aim, vehicle_yaw, cfg.tilt_rad, cfg.gimbal_max_rad,
                                         cfg.pitch_nudge_max_rad)
            diag.update(d)
        else:
            cmd = two_axis_command(pos, aim, yaw_hint=vehicle_yaw)
        if self._gimbal is None or dt <= 0.0:
            self._gimbal = cmd
        else:
            rate = cfg.gimbal_rate_rad_s if cfg.single_axis else cfg.two_axis_rate_rad_s
            self._gimbal = slew_limit(self._gimbal, cmd, rate * dt)
        return self._gimbal
