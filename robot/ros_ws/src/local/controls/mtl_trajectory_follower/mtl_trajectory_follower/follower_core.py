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

Single-axis gimbal laws (``FollowerConfig.gimbal_law``, from the plan):

``open_loop``  replay the planned cross-track angle (``roll + phi``) at the
               current arc position, look angle fixed at the mount tilt: the
               gimbal just swings left and right as planned
               (:func:`~mtl_trajectory_follower.gimbal_math.single_axis_open_loop`).
               The MTL stack's default (``mission.follower.gimbal_law``).
``aim_point``  aim from the vehicle's actual position at the planned ground
               point, cross-track angle plus a +/-5 deg pitch nudge
               (:func:`~mtl_trajectory_follower.gimbal_math.single_axis_command`).
               The follower's original law; used when the plan names none
               (e.g. the TIGRIS planner).

The 2-DOF mount always aims at the planned ground point.

Quadrotor plans (``SearchPlan.platform == "quadrotor"``: curve planner + ``quad.enabled``)
are flown by :class:`QuadTrackFollower` instead: the camera is BODY-FIXED, so where it
looks is set by the airframe attitude, and the plan is a 6-DOF reference in TIME
(x, y, z, roll, pitch, yaw, with velocity and acceleration). It is tracked by a time
index that advances with the clock (slowed by a lag governor when the vehicle falls
behind), and every tick returns the reference attitude as a feed-forward for the
attitude controller (``FollowerOutput.attitude_ref``).
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Sequence

from mtl_trajectory_follower.gimbal_math import (GIMBAL_LAWS, single_axis_command, single_axis_open_loop,
                                                 slew_limit, two_axis_command, wrap_pi)

IDLE, INGRESS, SEARCH, COMPLETE, ABORTED = 0, 1, 2, 3, 4
STATE_NAMES = {IDLE: "IDLE", INGRESS: "INGRESS", SEARCH: "SEARCH", COMPLETE: "COMPLETE",
               ABORTED: "ABORTED"}

__all__ = ["Track", "FollowerConfig", "FollowerOutput", "TrackFollower", "QuadFollowerConfig",
           "QuadTrackFollower", "STATE_NAMES", "IDLE", "INGRESS", "SEARCH", "COMPLETE", "ABORTED"]


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
    phi: list[float] = field(default_factory=list)     # planned 1-DOF gimbal angle (+ right)
    roll: list[float] = field(default_factory=list)    # planned airframe roll (crossAngle = roll + phi)
    # 6-DOF (quadrotor) references; empty for a gimbal plan
    pitch: list[float] = field(default_factory=list)   # planned airframe pitch, + nose UP (planner convention)
    vx: list[float] = field(default_factory=list)      # planned velocity [map]
    vy: list[float] = field(default_factory=list)
    ax: list[float] = field(default_factory=list)      # planned acceleration [map]
    ay: list[float] = field(default_factory=list)
    yaw_rate: list[float] = field(default_factory=list)

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
        for name in ("phi", "roll", "pitch", "vx", "vy", "ax", "ay", "yaw_rate"):
            if getattr(self, name) and len(getattr(self, name)) != n:
                raise ValueError(f"track field {name!r} has {len(getattr(self, name))} samples, expected {n}")
        if self.t and len(self.t) != n:
            raise ValueError(f"track field 't' has {len(self.t)} samples, expected {n}")

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

    def cross_angle_at(self, s: float) -> float:
        """Planned level-frame cross-track angle ``roll + phi`` (+ right) at arc length ``s``."""
        k, w = self._locate(s)
        phi = self.phi[k] + w * (self.phi[k + 1] - self.phi[k]) if self.phi else 0.0
        roll = self.roll[k] + w * (self.roll[k + 1] - self.roll[k]) if self.roll else 0.0
        return phi + roll

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


    # interpolation at a TIME (6-DOF quadrotor references) ------------------- #
    @property
    def duration(self) -> float:
        return self.t[-1] - self.t[0] if self.t else 0.0

    def _locate_t(self, tq: float) -> tuple[int, float]:
        if not self.t:
            raise ValueError("this track has no timeline")
        tq = min(max(tq, self.t[0]), self.t[-1])
        k = bisect.bisect_right(self.t, tq) - 1
        k = min(max(k, 0), len(self) - 2)
        seg = self.t[k + 1] - self.t[k]
        w = 0.0 if seg <= 0 else (tq - self.t[k]) / seg
        return k, min(max(w, 0.0), 1.0)

    @staticmethod
    def _lerp(v: list[float], k: int, w: float) -> float:
        return v[k] + w * (v[k + 1] - v[k]) if v else 0.0

    def reference_at_time(self, tq: float) -> dict:
        """The 6-DOF reference at time ``tq`` (seconds on the plan's timeline)."""
        k, w = self._locate_t(tq)
        L = self._lerp
        yaw = wrap_pi(self.yaw[k] + w * wrap_pi(self.yaw[k + 1] - self.yaw[k]))
        if self.vx:
            vel = (L(self.vx, k, w), L(self.vy, k, w), 0.0)
        else:  # no planned velocity: finite difference of the positions
            dt = max(self.t[k + 1] - self.t[k], 1e-6)
            vel = ((self.x[k + 1] - self.x[k]) / dt, (self.y[k + 1] - self.y[k]) / dt,
                   (self.z[k + 1] - self.z[k]) / dt)
        return {
            "pos": (L(self.x, k, w), L(self.y, k, w), L(self.z, k, w)),
            "vel": vel,
            "acc": (L(self.ax, k, w), L(self.ay, k, w), 0.0),
            "roll": L(self.roll, k, w), "pitch_up": L(self.pitch, k, w), "yaw": yaw,
            "yaw_rate": L(self.yaw_rate, k, w),
            "bore": (L(self.bx, k, w), L(self.by, k, w), L(self.bz, k, w)),
            "arc": L(self.arc, k, w), "index": k,
        }


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
    gimbal_law: str = "aim_point"       # single-axis law: "open_loop" | "aim_point" (see module doc)

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
    # quadrotor (6-DOF) mode only: the reference attitude (roll, pitch, yaw) in the
    # ROS ZYX convention of the map frame (FLU body: pitch > 0 = nose DOWN), the
    # attitude feed-forward for the controller; None = no feed-forward this tick
    attitude_ref: tuple | None = None
    t_ref: float | None = None

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
        if cfg.gimbal_law not in GIMBAL_LAWS:
            raise ValueError(f"gimbal_law {cfg.gimbal_law!r} is not one of {', '.join(GIMBAL_LAWS)}")
        step = max(track.step_m, 1e-3)
        self.window = max(int(round(cfg.window_lookaheads * cfg.lookahead / step)), cfg.min_window_samples)

    # ------------------------------------------------------------------ #
    def replace_track(self, track: Track) -> None:
        """Swap in a REVISION of the same sortie (a receding-horizon planner such as
        tigris_search_planner re-publishes the whole track with an unchanged prefix).

        The arc-length progress carries over (the prefix is identical, so the
        index is still valid); a sortie that had reached the old end resumes
        SEARCH when the revision extends it.
        """
        self.track = track
        step = max(track.step_m, 1e-3)
        self.window = max(int(round(self.cfg.window_lookaheads * self.cfg.lookahead / step)),
                          self.cfg.min_window_samples)
        self.progress = min(self.progress, track.total)
        self.idx = min(max(self.idx, 0), len(track) - 2)
        if self.state == COMPLETE and track.total - self.progress > self.cfg.finish_tolerance_m:
            self.state = SEARCH

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
                gimbal = self._gimbal_command(pos, aim, vehicle_yaw, dt, diag, 0.0)
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
            gimbal = self._gimbal_command(pos, aim, vehicle_yaw, dt, diag, tr.total)
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
        s_aim = self.progress + cfg.gimbal_lead_s * v
        aim = tr.boresight_at(s_aim)
        gimbal = self._gimbal_command(pos, aim, vehicle_yaw, dt, diag, s_aim)
        return FollowerOutput(SEARCH, carrot, heading, vel, aim, gimbal, self.progress,
                              tr.total - self.progress, xte, self.idx, diag)

    # ------------------------------------------------------------------ #
    def _gimbal_command(self, pos, aim, vehicle_yaw, dt, diag, s_aim) -> tuple[float, float, float]:
        cfg = self.cfg
        if cfg.single_axis and cfg.gimbal_law == "open_loop":
            cmd, d = single_axis_open_loop(pos, aim, vehicle_yaw, cfg.tilt_rad,
                                           self.track.cross_angle_at(s_aim), cfg.gimbal_max_rad)
            diag.update(d)
        elif cfg.single_axis:
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


# =============================================================================
#  Quadrotor with a body-fixed camera: 6-DOF time-indexed reference tracking
# =============================================================================
@dataclass
class QuadFollowerConfig:
    min_turn_radius_m: float = 12.0     # INGRESS carrot = lookahead_turn_radii * R (as the gimbal follower)
    lookahead_turn_radii: float = 1.2
    ingress_tolerance_m: float = 4.0
    ingress_alt_tolerance_m: float = 2.0
    ingress_yaw_tolerance_rad: float = math.radians(20.0)   # face the first sample's heading before t = 0
    finish_tolerance_m: float = 3.0
    finish_timeout_s: float = 10.0      # after the reference ends: COMPLETE anyway after this long
    lag_soft_m: float = 5.0             # time-index governor: full rate while the vehicle lags < this ...
    lag_hard_m: float = 20.0            # ... and the reference stops while it lags >= this
    # Roll / pitch and yaw reference ahead of the position reference, to cancel the attitude
    # loops' lag. Measured in Isaac / PX4 (s0136 quad run, 2026-11-11): roll / pitch reach the
    # feed-forward ~0.25 s late, yaw ~0.45 s late (PX4's yaw loop is the slow one).
    attitude_lead_s: float = 0.25
    yaw_lead_s: float = 0.45
    speed_mps: float = 6.0              # INGRESS speed
    tilt_rad: float = math.radians(32.25)  # the body-fixed camera's tilt from body -z toward +x

    @property
    def lookahead(self) -> float:
        return self.lookahead_turn_radii * self.min_turn_radius_m


def body_fixed_camera_euler(roll: float, pitch_down: float, yaw: float, tilt: float) -> tuple[float, float, float]:
    """Earth-frame (roll, pitch, yaw) of a camera bolted to an airframe at ZYX attitude
    (roll, pitch_down, yaw) [ROS FLU: pitch > 0 nose down], boresight tilted ``tilt``
    from body -z toward body +x (i.e. pitched down 90 deg - tilt from the nose); the
    gimbal/cmd_pitch_yaw / gimbal/state convention of gimbal_math."""
    from mtl_trajectory_follower.gimbal_math import euler_zyx_to_matrix, mat_mul, matrix_to_euler_zyx
    r_body = euler_zyx_to_matrix(roll, pitch_down, yaw)
    r_mount = euler_zyx_to_matrix(0.0, 0.5 * math.pi - tilt, 0.0)
    return matrix_to_euler_zyx(mat_mul(r_body, r_mount), yaw_hint=yaw)


class QuadTrackFollower:
    """Fly a quadrotor plan: INGRESS (to the first sample, at its altitude and heading),
    then SEARCH by TIME: the reference time ``t_ref`` advances with the clock while the
    vehicle keeps up (lag governor), the tracking point is the reference position and
    velocity at ``t_ref`` and its heading the reference yaw, and ``attitude_ref`` carries
    the reference roll / pitch / yaw for the attitude feed-forward. COMPLETE holds the end.
    Same ``step()`` contract as :class:`TrackFollower`."""

    def __init__(self, track: Track, cfg: QuadFollowerConfig) -> None:
        if not track.t:
            raise ValueError("a quadrotor track needs its timeline (time_s)")
        if any(b <= a for a, b in zip(track.t, track.t[1:])):
            raise ValueError("a quadrotor track's timeline must be strictly increasing")
        self.track = track
        self.cfg = cfg
        self.state = IDLE
        self.t_ref = track.t[0]
        self.t_end_reached = None   # wall seconds spent since t_ref hit the end
        self.idx = 0
        self.progress = 0.0

    def replace_track(self, track: Track) -> None:  # the curve planner never revises a plan
        self.track = track

    def _at_start(self, pos, vehicle_yaw) -> bool:
        tr, c = self.track, self.cfg
        return (math.hypot(pos[0] - tr.x[0], pos[1] - tr.y[0]) <= c.ingress_tolerance_m
                and abs(pos[2] - tr.z[0]) <= c.ingress_alt_tolerance_m
                and abs(wrap_pi(vehicle_yaw - tr.yaw[0])) <= c.ingress_yaw_tolerance_rad)

    def start(self, pos, vehicle_yaw: float = 0.0) -> int:
        self.state = SEARCH if self._at_start(pos, vehicle_yaw) else INGRESS
        self.t_ref = self.track.t[0]
        return self.state

    def _lag(self, ref: dict, pos) -> float:
        """How far the vehicle is BEHIND the reference along the reference velocity
        (the distance to it when the reference is slow)."""
        dx, dy = ref["pos"][0] - pos[0], ref["pos"][1] - pos[1]
        vx, vy = ref["vel"][0], ref["vel"][1]
        sp = math.hypot(vx, vy)
        return (dx * vx + dy * vy) / sp if sp > 0.5 else math.hypot(dx, dy)

    def _attitude(self, ref: dict) -> tuple[float, float, float]:
        # planner: pitch + nose up  ->  ROS FLU ZYX: pitch + nose down
        return (ref["roll"], -ref["pitch_up"], ref["yaw"])

    def _out(self, state, ref, carrot, heading, vel, pos, att, xte) -> FollowerOutput:
        tr = self.track
        cam = body_fixed_camera_euler(*(att or (0.0, 0.0, heading)), self.cfg.tilt_rad)
        return FollowerOutput(state, carrot, heading, vel, ref["bore"], cam, self.progress,
                              max(tr.total - self.progress, 0.0), xte, ref["index"],
                              {"t_ref": self.t_ref, "lag_m": self._lag(ref, pos)}, att, self.t_ref)

    def step(self, pos, vehicle_yaw: float, dt: float) -> FollowerOutput:
        tr, c = self.track, self.cfg
        if self.state == IDLE:
            self.start(pos, vehicle_yaw)
        if self.state == INGRESS:
            if self._at_start(pos, vehicle_yaw):
                self.state = SEARCH
                self.t_ref = tr.t[0]
            else:
                ref = tr.reference_at_time(tr.t[0])
                sx, sy, sz = tr.x[0], tr.y[0], tr.z[0]
                dx, dy = sx - pos[0], sy - pos[1]
                dist = math.hypot(dx, dy)
                L = c.lookahead
                if dist > L:
                    carrot = (pos[0] + L * dx / dist, pos[1] + L * dy / dist, sz)
                    heading = math.atan2(dy, dx)
                else:
                    carrot = (sx, sy, sz)
                    heading = tr.yaw[0] if dist < 2.0 * c.ingress_tolerance_m else math.atan2(dy, dx)
                vel = (c.speed_mps * dx / dist, c.speed_mps * dy / dist, 0.0) if dist > L else (0.0, 0.0, 0.0)
                self.progress = 0.0
                return self._out(INGRESS, ref, carrot, heading, vel, pos, None, dist)

        if self.state == SEARCH:
            ref = tr.reference_at_time(self.t_ref)
            lag = self._lag(ref, pos)
            span = max(c.lag_hard_m - c.lag_soft_m, 1e-6)
            rate = min(max((c.lag_hard_m - lag) / span, 0.0), 1.0)
            self.t_ref = min(self.t_ref + rate * max(dt, 0.0), tr.t[-1])
            ref = tr.reference_at_time(self.t_ref)
            self.progress = max(self.progress, ref["arc"])
            if self.t_ref >= tr.t[-1]:
                self.t_end_reached = (self.t_end_reached or 0.0) + max(dt, 0.0)
                end_err = math.hypot(pos[0] - tr.x[-1], pos[1] - tr.y[-1])
                if end_err <= c.finish_tolerance_m or self.t_end_reached >= c.finish_timeout_s:
                    self.state = COMPLETE
            if self.state == SEARCH:
                att_ref = tr.reference_at_time(self.t_ref + c.attitude_lead_s)
                yaw_ref = tr.reference_at_time(self.t_ref + c.yaw_lead_s)
                att = (att_ref["roll"], -att_ref["pitch_up"], yaw_ref["yaw"])
                xte = math.hypot(pos[0] - ref["pos"][0], pos[1] - ref["pos"][1])
                return self._out(SEARCH, ref, ref["pos"], att[2], ref["vel"], pos, att, xte)

        # COMPLETE: hold the end, level, no feed-forward
        ref = tr.reference_at_time(tr.t[-1])
        self.progress = tr.total
        end = (tr.x[-1], tr.y[-1], tr.z[-1])
        xte = math.hypot(pos[0] - end[0], pos[1] - end[1])
        return self._out(COMPLETE, ref, end, tr.yaw[-1], (0.0, 0.0, 0.0), pos, None, xte)
