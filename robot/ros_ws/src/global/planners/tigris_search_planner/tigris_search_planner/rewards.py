"""The two TIGRIS reward models in stdlib Python (a mirror of src/belief.cpp).

Used by ``scripts/analyze_tigris_run.py`` to score ANY run (TIGRIS or MTL) with
both rewards, from the flown telemetry and from the planned track:

* **original** - TIGRIS (Moon et al. 2023): per-cell Bernoulli presence belief
  (the scenario's un-normalised bump raster, floored at ``initial_confidence``),
  Bayes update with ``tpr(r)`` = the scenario sigmoid for ``r <= beta`` (0.5
  beyond) and ``fpr = 1 - tpr``, the TIGRIS branch (``p > 0.5`` -> assumed
  detection, else a miss), reward = entropy drop x ``Rs`` (belief rose) or ``Rf``
  (fell). A cell is updated once per *pass* at the best range any look of the
  pass had; the analysis cuts the flown path into passes of ``edge_m`` metres
  (default: the planner's ``extend_dist_m``), the planner uses its tree edges.
* **matched** - the searched belief mass ``sum(prior) - sum(residual)`` with
  ``residual = prior * prod (1 - P(r))^(dt / dt_ref)``, the logger's metric on
  the planning grid.

Both use the body-fixed camera footprint the logger scores: the ground disc of
radius ``slant * tan(fov / 2)`` about the boresight ground point, skipped when
the slant range exceeds ``beta``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Mapping, Sequence

__all__ = ["Detection", "Grid", "Look", "look_from_gimbal", "look_from_pose", "look_from_points", "RewardParams",
           "RewardCurve", "score_looks", "raw_prior"]


@dataclass(frozen=True)
class Detection:
    a: float = 1.10
    b: float = 0.10
    c: float = 61.0
    beta: float = 61.0
    p_out: float = 1e-6
    dt_ref: float = 0.1

    @classmethod
    def from_scenario(cls, sc: Mapping) -> "Detection":
        d = sc["sensor"]["detection"]
        return cls(float(d.get("a", 1.1)), float(d.get("b", 0.1)), float(d.get("c", 61.0)),
                   float(d.get("beta", 61.0)), float(d.get("p_out_of_range", 1e-6)), float(d.get("dt_ref_s", 0.1)))

    def prob(self, r: float) -> float:
        return self.p_out if r > self.beta else 1.0 / (self.a + math.exp(self.b * (r - self.c)))


@dataclass(frozen=True)
class RewardParams:
    use_entropy: bool = True
    rs: float = 2.0
    rf: float = 1.0
    initial_confidence: float = 0.01
    tpr_beyond_beta: float = 0.5


@dataclass
class Look:
    px: float
    py: float
    pz: float
    gx: float
    gy: float
    radius: float
    weight: float


def look_from_pose(x: float, y: float, h: float, yaw: float, fov: float, tilt: float, det: Detection,
                   weight: float, phi: float = 0.0) -> Look | None:
    """Single-axis mount at cross-track angle ``phi`` (+ right; 0 = body-fixed camera), as
    belief.cpp lookFromPose: ground point h tan(tilt) / cos(phi) ahead, h tan(phi) right."""
    c = math.cos(tilt) * math.cos(phi)
    if h <= 0.0 or c <= 1e-6:
        return None
    slant = h / c
    if slant > det.beta:
        return None
    ahead = h * math.tan(tilt) / math.cos(phi)
    right = h * math.tan(phi)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return Look(x, y, h, x + ahead * cy + right * sy, y + ahead * sy - right * cy, slant * math.tan(fov / 2.0),
                weight)


def sweep_phi(t: float, rate: float, amplitude: float) -> float:
    """Cross-track angle of the gimbal sweep at track time ``t`` (belief.cpp Camera::phiAt):
    a triangle wave of slope +-rate between -amplitude and +amplitude, phi(0) = 0 moving right."""
    if rate <= 0.0 or amplitude <= 0.0:
        return 0.0
    u = math.fmod(rate * t, 4.0 * amplitude)
    if u < 0.0:
        u += 4.0 * amplitude
    if u < amplitude:
        return u
    if u < 3.0 * amplitude:
        return 2.0 * amplitude - u
    return u - 4.0 * amplitude


def look_from_gimbal(x: float, y: float, h: float, pitch: float, yaw: float, fov: float, det: Detection,
                     weight: float) -> Look | None:
    if pitch is None or yaw is None or math.isnan(pitch) or math.isnan(yaw):
        return None
    bz = -math.sin(pitch)
    if bz >= -1e-6 or h <= 0.0:
        return None
    s = h / -bz
    if s > det.beta:
        return None
    cp = math.cos(pitch)
    return Look(x, y, h, x + s * math.cos(yaw) * cp, y + s * math.sin(yaw) * cp, s * math.tan(fov / 2.0), weight)


def look_from_points(p: Sequence[float], g: Sequence[float], fov: float, det: Detection,
                     weight: float) -> Look | None:
    """Look from camera position ``p`` at a planned boresight ground point ``g`` (world ENU),
    as ``mtl_metrics_logger.detection.planned_residual`` scores a planned track."""
    if p is None or g is None:
        return None
    slant = math.sqrt((p[0] - g[0]) ** 2 + (p[1] - g[1]) ** 2 + (p[2] - g[2]) ** 2)
    if slant > det.beta or p[2] - g[2] <= 0.0:
        return None
    return Look(p[0], p[1], p[2] - g[2], g[0], g[1], slant * math.tan(fov / 2.0), weight)


def _axis(lo: float, hi: float, step: float) -> list:
    count = int(math.floor((hi - lo) / step + 1e-9)) + 1
    return [lo + k * step for k in range(count)]


def raw_prior(sc: Mapping, progress=None):
    """(xs, ys, raw, norm): the scenario bump raster, capped + floored (raw) and normalised.
    ``progress(stage, done, total)`` (optional) is called once per bump."""
    area = sc["mission"]["area"]
    bel = (sc.get("airstack") or {}).get("belief") or {}
    bumps = bel.get("bumps") or []
    if not bumps:
        raise ValueError("scenario carries no airstack.belief.bumps")
    half = float(area["size_m"]) / 2.0
    cn, ce = (float(v) for v in area.get("center_ned", (0.0, 0.0)))
    res = float(area.get("belief_res_m", 2.0))
    ys, xs = _axis(cn - half, cn + half, res), _axis(ce - half, ce + half, res)
    cap, floor = float(bel.get("belief_cap", 0.85)), float(bel.get("base_uncertainty", 0.0))
    rows = [[0.0] * len(xs) for _ in ys]
    for ib, b in enumerate(bumps):
        if progress is not None:
            progress("reward grid", ib, len(bumps) + 1)
        gn = [math.exp(-0.5 * ((y - float(b["n"])) / float(b["sigma_n"])) ** 2) for y in ys]
        ge = [math.exp(-0.5 * ((x - float(b["e"])) / float(b["sigma_e"])) ** 2) for x in xs]
        amp = float(b.get("amplitude", 0.4))
        for i, gi in enumerate(gn):
            if gi < 1e-12:
                continue
            row = rows[i]
            for j, gj in enumerate(ge):
                row[j] += amp * gi * gj
    raw = []
    for row in rows:
        for v in row:
            v = min(v, cap)
            raw.append(max(v, floor) if floor > 0.0 else v)
    tot = math.fsum(raw)
    return xs, ys, raw, [v / tot for v in raw]


@dataclass
class Grid:
    """Planning grid over the search area (world ENU), as ``PlanningGrid`` in belief.cpp."""

    x_min: float
    y_min: float
    res: float
    nx: int
    ny: int
    mass: list
    presence0: list

    @classmethod
    def from_scenario(cls, sc: Mapping, res: float = 4.0, initial_confidence: float = 0.01,
                      progress=None) -> "Grid":
        area = sc["mission"]["area"]
        size = float(area["size_m"])
        cn, ce = (float(v) for v in area.get("center_ned", (0.0, 0.0)))
        x_min, y_min = ce - size / 2.0, cn - size / 2.0
        nx = max(1, int(math.floor(size / res + 1e-9)))
        ny = nx
        xs, ys, raw, norm = raw_prior(sc, progress=progress)
        mass = [0.0] * (nx * ny)
        pres = [initial_confidence] * (nx * ny)  # max of the prior point values in each cell (TIGRIS setup())
        cols = [min(nx - 1, max(0, int(math.floor((x - x_min) / res)))) for x in xs]
        for i, y in enumerate(ys):
            ci = min(ny - 1, max(0, int(math.floor((y - y_min) / res))))
            base = i * len(xs)
            for j, cj in enumerate(cols):
                k = ci * nx + cj
                mass[k] += norm[base + j]
                if raw[base + j] > pres[k]:
                    pres[k] = raw[base + j]
        if progress is not None:
            progress("reward grid", 1, 1)
        return cls(x_min, y_min, res, nx, ny, mass, pres)

    def disc(self, gx: float, gy: float, r: float):
        res, x0, y0 = self.res, self.x_min, self.y_min
        i0 = max(0, int(math.ceil((gy - r - y0) / res - 0.5 - 1e-9)))
        i1 = min(self.ny - 1, int(math.floor((gy + r - y0) / res - 0.5 + 1e-9)))
        r2 = r * r
        for i in range(i0, i1 + 1):
            y = y0 + (i + 0.5) * res
            dy = y - gy
            rem = r2 - dy * dy
            if rem < 0.0:
                continue
            half = math.sqrt(rem)
            j0 = max(0, int(math.ceil((gx - half - x0) / res - 0.5 - 1e-9)))
            j1 = min(self.nx - 1, int(math.floor((gx + half - x0) / res - 0.5 + 1e-9)))
            for j in range(j0, j1 + 1):
                x = x0 + (j + 0.5) * res
                if (x - gx) ** 2 + dy * dy <= r2:
                    yield i * self.nx + j, x, y


def _entropy(p: float) -> float:
    if p <= 0.0 or p >= 1.0:
        return 0.0
    return -p * math.log2(p) - (1.0 - p) * math.log2(1.0 - p)


def original_update(p: float, r: float, det: Detection, rp: RewardParams) -> tuple[float, float]:
    """(reward, new belief) of one TIGRIS cell update at range r."""
    tpr = det.prob(r) if r <= det.beta else rp.tpr_beyond_beta
    fpr = 1.0 - tpr
    if p > 0.5:
        den = tpr * p + fpr * (1.0 - p)
        q = tpr * p / den if den > 0 else p
    else:
        den = (1.0 - tpr) * p + (1.0 - fpr) * (1.0 - p)
        q = (1.0 - tpr) * p / den if den > 0 else p
    w = rp.rs if q - p > 0 else rp.rf
    return ((_entropy(p) - _entropy(q)) if rp.use_entropy else abs(q - p)) * w, q


@dataclass
class RewardCurve:
    t: list = field(default_factory=list)
    original: list = field(default_factory=list)
    matched: list = field(default_factory=list)

    def as_dict(self, ndigits: int = 6) -> dict:
        return {"t_s": [round(v, 3) for v in self.t], "original": [round(v, ndigits) for v in self.original],
                "matched": [round(v, ndigits) for v in self.matched]}


def score_looks(grid: Grid, det: Detection, rp: RewardParams, samples: Iterable[tuple], edge_m: float,
                progress=None, stage: str = "tigris rewards") -> RewardCurve:
    """Cumulative rewards along a look sequence (one agent or a fused team).

    ``samples``: ``(t, arc, look)`` or ``(t, arc, look, agent)`` in time order; ``arc`` is
    that agent's flown (or planned) distance and ``look`` may be None (no valid look).
    MATCHED is updated per look. ORIGINAL is updated once per pass of ``edge_m`` metres of
    an agent's arc, each touched cell at its best range of the pass, so the ORIGINAL curve
    steps up at the end of every pass. Agents keep separate passes but share the belief.
    ``progress(stage, done, total)`` (optional) is called along the samples.
    """
    residual = list(grid.mass)
    presence = list(grid.presence0)
    log_out = math.log1p(-min(max(det.p_out, 0.0), 1.0 - 1e-15))
    cur = RewardCurve()
    totals = {"original": 0.0, "matched": 0.0}
    pass_start: dict = {}
    rmin: dict = {}

    def close_pass(agent):
        cells = rmin.get(agent)
        if not cells:
            return
        for k, r in cells.items():
            rew, q = original_update(presence[k], r, det, rp)
            presence[k] = q
            totals["original"] += rew
        cells.clear()

    samples = list(samples)
    every = max(1, len(samples) // 500)
    for i_s, sample in enumerate(samples):
        if progress is not None and i_s % every == 0:
            progress(stage, i_s, len(samples))
        t, arc, look = sample[0], sample[1], sample[2]
        agent = sample[3] if len(sample) > 3 else ""
        if agent not in pass_start:
            pass_start[agent] = arc
            rmin[agent] = {}
        if arc - pass_start[agent] >= edge_m:
            close_pass(agent)
            pass_start[agent] = arc
        if look is not None and look.weight > 0.0 and look.radius > 0.0:
            h2 = look.pz * look.pz
            f_out = math.exp(look.weight * log_out)
            cells = rmin[agent]
            for k, x, y in grid.disc(look.gx, look.gy, look.radius):
                r = math.sqrt((x - look.px) ** 2 + (y - look.py) ** 2 + h2)
                old = residual[k]
                if old > 0.0:
                    if r > det.beta:
                        f = f_out
                    else:
                        q = 1.0 - 1.0 / (det.a + math.exp(det.b * (r - det.c)))
                        f = q ** look.weight if q > 0.0 else 0.0
                    residual[k] = old * f
                    totals["matched"] += old - old * f
                prev = cells.get(k)
                if prev is None or r < prev:
                    cells[k] = r
        cur.t.append(t)
        cur.original.append(totals["original"])
        cur.matched.append(totals["matched"])
    for agent in list(rmin):
        close_pass(agent)
    if progress is not None:
        progress(stage, len(samples), len(samples))
    if cur.t:
        cur.original[-1] = totals["original"]
    return cur
