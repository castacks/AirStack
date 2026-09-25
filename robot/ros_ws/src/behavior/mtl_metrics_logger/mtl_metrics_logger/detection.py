"""Detection scoring for MTL search flights (pure Python, no ROS, no numpy).

Scored from the FLOWN pose and the MEASURED gimbal attitude, never from the
plan — a plan can claim a cell and the aircraft can still arrive late with the
gimbal slewing.

Sensor model: Moon et al. (2022), the one the MATLAB pipeline and
``cpp_planner/src/eval/detection.cpp`` use:

    P(z | x) = 1 / (a + exp(b (r - c)))    for 3-D range r <= beta   (else p_out_of_range)

Gates, per agent and step: the target must lie inside the camera's circular
ground footprint of radius ``d_slant * tan(FOV/2)`` about the point the
boresight actually hits the ground (it grows as the gimbal swings out), and
``r <= beta``. Evidence accumulates as a joint miss probability

    P_miss(t) = P_miss(t-1) * (1 - P)^(dt / dt_ref)        P_det = 1 - P_miss

The exponent makes the score independent of the logging rate: the sigmoid is
a per-LOOK probability calibrated at the planner's ``dt_ref = 0.1 s``, so a
20 Hz logger must not count twice as many looks as a 10 Hz one. With
``dt == dt_ref`` this is exactly the reference update. A target is declared
FOUND the first step ``P_det >= threshold`` (0.9); the agent whose look pushed
it over is the responsible agent.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Iterable, Mapping, Sequence

__all__ = [
    "DetectionModel",
    "boresight_ground_point",
    "footprint_radius",
    "TargetState",
    "TeamScorer",
    "resample_hold",
]


@dataclass(frozen=True)
class DetectionModel:
    a: float = 1.10
    b: float = 0.10
    c: float = 61.0
    beta: float = 61.0
    p_out_of_range: float = 1.0e-6
    threshold: float = 0.90
    dt_ref_s: float = 0.1

    @classmethod
    def from_scenario(cls, det: Mapping) -> "DetectionModel":
        return cls(a=float(det.get("a", 1.10)), b=float(det.get("b", 0.10)), c=float(det.get("c", 61.0)),
                   beta=float(det.get("beta", 61.0)), p_out_of_range=float(det.get("p_out_of_range", 1e-6)),
                   threshold=float(det.get("threshold", 0.9)), dt_ref_s=float(det.get("dt_ref_s", 0.1)))

    def probability(self, r: float) -> float:
        if r > self.beta:
            return self.p_out_of_range
        return 1.0 / (self.a + math.exp(self.b * (r - self.c)))


def boresight_ground_point(pos: Sequence[float], pitch: float, yaw: float, ground_z: float = 0.0):
    """Where the camera axis meets the plane ``z = ground_z`` -> ``(gx, gy, slant)``.

    ``pitch > 0`` looks down (earth-frame convention of gimbal/state). Returns
    ``None`` when the boresight is at/above the horizon or the vehicle is not
    above the plane — such samples are excluded, never projected to infinity.
    """
    cp = math.cos(pitch)
    bx, by, bz = math.cos(yaw) * cp, math.sin(yaw) * cp, -math.sin(pitch)
    height = pos[2] - ground_z
    if bz >= -1e-6 or height <= 0.0:
        return None
    s = height / -bz
    return (pos[0] + s * bx, pos[1] + s * by, s)


def footprint_radius(slant: float, fov_rad: float) -> float:
    return slant * math.tan(fov_rad / 2.0)


@dataclass
class TargetState:
    index: int
    x: float
    y: float
    z: float = 0.0
    p_miss: float = 1.0
    first_seen_s: float | None = None
    detection_time_s: float | None = None
    detected_by: str | None = None
    observers: set = field(default_factory=set)
    observations: int = 0
    min_slant_m: float = math.inf

    @property
    def p_det(self) -> float:
        return 1.0 - self.p_miss

    @property
    def detected(self) -> bool:
        return self.detection_time_s is not None


class TeamScorer:
    """Incremental team-wide scorer.

    Call :meth:`step` once per timeline step with every agent's sample for that
    step; the miss product is fused across agents inside the step, and the
    threshold is evaluated after all agents have looked (two agents looking at
    once are credited together, as in the reference implementation).
    """

    def __init__(self, targets: Iterable[Sequence[float]], cells: Iterable[Sequence[float]],
                 cell_mass: Iterable[float], model: DetectionModel, fov_rad: float) -> None:
        self.model = model
        self.fov = float(fov_rad)
        self.tan_half = math.tan(self.fov / 2.0)
        self.targets = [TargetState(i, float(t[0]), float(t[1]), float(t[2]) if len(t) > 2 else 0.0)
                        for i, t in enumerate(targets)]
        self.cells = [(float(c[0]), float(c[1])) for c in cells]
        self.cell_mass = [float(m) for m in cell_mass]
        self.cell_covered = [False] * len(self.cells)
        self.covered_mass = 0.0
        self.distance_m: dict[str, float] = {}
        self._last_pos: dict[str, tuple[float, float]] = {}
        # curves
        self.t: list[float] = []
        self.detected_count: list[int] = []
        self.mass_curve: list[float] = []
        self.distance_curve: list[float] = []
        self.p_det_curve: list[list[float]] = []

    @property
    def total_mass(self) -> float:
        return sum(self.cell_mass)

    def step(self, t: float, samples: Mapping[str, Mapping], dt: float, *, record: bool = True) -> None:
        """``samples[agent] = {"pos": (x,y,z), "pitch": p, "yaw": y, "ground_z": z0}``.

        ``pitch``/``yaw`` are the MEASURED gimbal angles (earth frame); pass
        ``None`` for either to mark the look invalid (no camera state).
        """
        m = self.model
        weight = max(dt, 0.0) / m.dt_ref_s if m.dt_ref_s > 0 else 1.0
        for agent, s in samples.items():
            pos = s["pos"]
            last = self._last_pos.get(agent)
            if last is not None:
                self.distance_m[agent] = self.distance_m.get(agent, 0.0) + math.hypot(pos[0] - last[0], pos[1] - last[1])
            else:
                self.distance_m.setdefault(agent, 0.0)
            self._last_pos[agent] = (pos[0], pos[1])
            pitch, yaw = s.get("pitch"), s.get("yaw")
            if pitch is None or yaw is None or weight <= 0.0 or math.isnan(pitch) or math.isnan(yaw):
                continue
            gp = boresight_ground_point(pos, pitch, yaw, s.get("ground_z", 0.0))
            if gp is None:
                continue
            gx, gy, slant = gp
            if slant > m.beta:
                # the look point itself is beyond sensor range: nothing inside can be seen
                continue
            radius = slant * self.tan_half
            r2 = radius * radius
            for tg in self.targets:
                dx, dy = tg.x - gx, tg.y - gy
                if dx * dx + dy * dy > r2:
                    continue
                rng = math.sqrt((tg.x - pos[0]) ** 2 + (tg.y - pos[1]) ** 2 + (tg.z - pos[2]) ** 2)
                p = m.probability(rng)
                tg.p_miss *= (1.0 - p) ** weight
                tg.observations += 1
                tg.observers.add(agent)
                tg.min_slant_m = min(tg.min_slant_m, rng)
                if tg.first_seen_s is None:
                    tg.first_seen_s = t
                if not tg.detected and tg.p_det >= m.threshold and tg.detected_by is None:
                    tg.detected_by = agent  # the look that pushed it over (first agent in the step)
            for i, (cx, cy) in enumerate(self.cells):
                if not self.cell_covered[i]:
                    dx, dy = cx - gx, cy - gy
                    if dx * dx + dy * dy <= r2:
                        self.cell_covered[i] = True
                        self.covered_mass += self.cell_mass[i]
        for tg in self.targets:
            if not tg.detected and tg.p_det >= m.threshold:
                tg.detection_time_s = t
        if record:
            self.t.append(t)
            self.detected_count.append(sum(1 for tg in self.targets if tg.detected))
            self.mass_curve.append(self.covered_mass)
            self.distance_curve.append(sum(self.distance_m.values()))
            self.p_det_curve.append([tg.p_det for tg in self.targets])

    # ------------------------------------------------------------------ #
    def summary(self) -> dict:
        found = [tg for tg in self.targets if tg.detected]
        times = [tg.detection_time_s for tg in found]
        dist = sum(self.distance_m.values())
        total = self.total_mass
        return {
            "targets_total": len(self.targets),
            "targets_detected": len(found),
            "targets_missed": len(self.targets) - len(found),
            "detection_threshold": self.model.threshold,
            "mean_time_to_discovery_s": (sum(times) / len(times)) if times else None,
            "total_path_length_m": round(dist, 2),
            "cells_total": len(self.cells),
            "cells_covered": sum(self.cell_covered),
            "belief_mass_total": round(total, 3),
            "belief_mass_covered": round(self.covered_mass, 3),
            "belief_mass_fraction": round(self.covered_mass / total, 5) if total > 0 else 0.0,
            "belief_mass_per_km": round(self.covered_mass / (dist / 1000.0), 2) if dist > 0 else 0.0,
            "distance_by_agent_m": {k: round(v, 2) for k, v in sorted(self.distance_m.items())},
        }

    def target_table(self) -> list[dict]:
        return [{
            "index": tg.index, "x": round(tg.x, 3), "y": round(tg.y, 3),
            "detection_prob": round(tg.p_det, 6),
            "detected": tg.detected,
            "detection_time_s": tg.detection_time_s,
            "first_seen_s": tg.first_seen_s,
            "responsible_agent": tg.detected_by if tg.detected else None,
            "observed_by": sorted(tg.observers),
            "observations": tg.observations,
            "min_range_m": None if math.isinf(tg.min_slant_m) else round(tg.min_slant_m, 3),
        } for tg in self.targets]


def resample_hold(times: Sequence[float], values: Sequence, timeline: Sequence[float]) -> list:
    """Hold-last-value resampling (a measured gimbal angle is a state held since it was
    measured — interpolating across a slew would invent pointings the camera never had)."""
    out = []
    n = len(times)
    for t in timeline:
        k = bisect.bisect_right(times, t) - 1
        out.append(values[min(max(k, 0), n - 1)] if n else None)
    return out
