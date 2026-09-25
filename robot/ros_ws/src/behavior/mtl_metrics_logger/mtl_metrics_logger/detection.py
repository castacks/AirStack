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

Residual belief (the planner-comparison metric)
-----------------------------------------------
The prior is a probability mass function over the scenario raster (it sums to
1). Every pixel ``x`` of it gets the same Bayes update as a target standing
there, i.e. a string of misses, with EXACTLY the gates, sigmoid and rate
normalisation above (ground at ``z = ground_z``)::

    residual(x)  = prior(x) * prod_looks (1 - P(z | x))^(dt / dt_ref)
                 = P(target at x AND every look missed it)
    residualMass = sum_x residual(x) = P(the search missed the target)

LOWER IS BETTER: 1 = nothing looked at, -> 0 = everything seen well. It is the
Python counterpart of ``mtl::eval::computeResidualBelief`` (vendored
cpp_planner, ``CHANGES_belief_mass_and_residual.md`` section 3); at a target
standing on a pixel centre ``residual / prior == P_miss`` of that target. The
one deliberate difference to the C++ reference is the ``dt / dt_ref`` exponent
(the C++ counts every trajectory step as one look): with ``dt == dt_ref`` (the
planner's 0.1 s) the two agree, and the flown score does not depend on the
logging rate.
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
    "PriorGrid",
    "prior_from_scenario",
    "ResidualBelief",
    "planned_residual",
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


# --------------------------------------------------------------------------- #
# prior raster + residual belief
# --------------------------------------------------------------------------- #
@dataclass
class PriorGrid:
    """The normalised prior in WORLD ENU (x = East = e, y = North = n).

    ``values[i * nx + j]`` is the probability that the target is in the pixel
    centred at ``(xs[j], ys[i])``; ``xs``/``ys`` are ascending and uniform with
    step ``res`` [m]. ``values`` sums to 1.
    """

    xs: list
    ys: list
    res: float
    values: list

    @property
    def nx(self) -> int:
        return len(self.xs)

    @property
    def ny(self) -> int:
        return len(self.ys)

    @property
    def total(self) -> float:
        return math.fsum(self.values)

    def value_at(self, x: float, y: float) -> float:
        j = min(max(int(round((x - self.xs[0]) / self.res)), 0), self.nx - 1)
        i = min(max(int(round((y - self.ys[0]) / self.res)), 0), self.ny - 1)
        return self.values[i * self.nx + j]


def _axis(lo: float, hi: float, step: float) -> list:
    count = int(math.floor((hi - lo) / step + 1e-9)) + 1
    return [lo + k * step for k in range(count)]


def prior_from_scenario(scenario: Mapping) -> PriorGrid | None:
    """Rebuild the scenario's prior raster (normalised) from ``airstack.belief``.

    Mirrors ``mtl_search_planner.scenario.generate_belief`` term for term (same
    axes, bump sum, cap, floor, then ``/ sum``) from the bump list the generator
    records, so this package needs no dependency on the planner. Works for
    scenarios written before the prior was normalised too (it normalises here).
    Returns ``None`` when the scenario carries no bumps.
    """
    area = scenario["mission"]["area"]
    bel = (scenario.get("airstack") or {}).get("belief") or {}
    bumps = bel.get("bumps") or []
    if not bumps:
        return None
    half = float(area["size_m"]) / 2.0
    cn, ce = (float(v) for v in area.get("center_ned", (0.0, 0.0)))
    res = float(area.get("belief_res_m", 2.0))
    n_axis = _axis(cn - half, cn + half, res)
    e_axis = _axis(ce - half, ce + half, res)
    cap = float(bel.get("belief_cap", 0.85))
    floor = float(bel.get("base_uncertainty", 0.0))
    rows = [[0.0] * len(e_axis) for _ in n_axis]
    for b in bumps:
        bn, be = float(b["n"]), float(b["e"])
        sn, se = float(b["sigma_n"]), float(b["sigma_e"])
        amp = float(b.get("amplitude", 0.4))
        gn = [math.exp(-0.5 * ((n - bn) / sn) ** 2) for n in n_axis]
        ge = [math.exp(-0.5 * ((e - be) / se) ** 2) for e in e_axis]
        for i, gi in enumerate(gn):
            if gi < 1e-12:
                continue
            row = rows[i]
            a = amp * gi
            for j, gj in enumerate(ge):
                row[j] += a * gj
    flat = []
    for row in rows:
        for v in row:
            v = min(v, cap)
            if floor > 0.0:
                v = max(v, floor)
            flat.append(v)
    total = math.fsum(flat)
    if not total > 0.0:
        return None
    inv = 1.0 / total
    return PriorGrid(xs=e_axis, ys=n_axis, res=res, values=[v * inv for v in flat])


class ResidualBelief:
    """Per-pixel miss product over the whole prior (see the module docstring).

    Feed it looks with :meth:`look` (the :class:`TeamScorer` does, from the same
    gated samples it scores the targets with). ``residual_mass`` is kept
    incrementally for the time curve; :meth:`exact_mass` re-sums the map.
    """

    def __init__(self, prior: PriorGrid, model: DetectionModel) -> None:
        self.prior = prior
        self.model = model
        self.residual = list(prior.values)
        self.prior_mass = prior.total
        self.residual_mass = self.prior_mass
        self.looks = 0
        self._log_out = math.log1p(-min(max(model.p_out_of_range, 0.0), 1.0 - 1e-15))

    def look(self, pos: Sequence[float], gx: float, gy: float, radius: float, weight: float,
             ground_z: float = 0.0) -> None:
        """One look: footprint of ``radius`` about the boresight ground point ``(gx, gy)``."""
        if not (weight > 0.0 and radius > 0.0):
            return
        pr, m = self.prior, self.model
        xs, ys, res, nx = pr.xs, pr.ys, pr.res, pr.nx
        x0, y0 = xs[0], ys[0]
        i0 = max(0, int(math.ceil((gy - radius - y0) / res - 1e-9)))
        i1 = min(pr.ny - 1, int(math.floor((gy + radius - y0) / res + 1e-9)))
        if i0 > i1:
            return
        px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
        h2 = (pz - ground_z) ** 2
        r2 = radius * radius
        a, b, c, beta = m.a, m.b, m.c, m.beta
        f_out = math.exp(weight * self._log_out)
        R = self.residual
        exp, sqrt = math.exp, math.sqrt
        dm = 0.0
        for i in range(i0, i1 + 1):
            y = ys[i]
            dy = y - gy
            rem = r2 - dy * dy
            if rem < 0.0:
                continue
            half = sqrt(rem)
            j0 = max(0, int(math.ceil((gx - half - x0) / res - 1e-9)))
            j1 = min(nx - 1, int(math.floor((gx + half - x0) / res + 1e-9)))
            if j0 > j1:
                continue
            dy2 = dy * dy
            yp2 = (y - py) ** 2 + h2
            base = i * nx
            for j in range(j0, j1 + 1):
                x = xs[j]
                dx = x - gx
                if dx * dx + dy2 > r2:  # same inclusive disc test as the targets
                    continue
                k = base + j
                old = R[k]
                if old == 0.0:
                    continue
                d3 = sqrt((x - px) ** 2 + yp2)
                if d3 > beta:
                    f = f_out
                else:
                    q = 1.0 - 1.0 / (a + exp(b * (d3 - c)))
                    f = q ** weight if q > 0.0 else 0.0
                new = old * f
                R[k] = new
                dm += old - new
        self.residual_mass -= dm
        self.looks += 1

    def exact_mass(self) -> float:
        return math.fsum(self.residual)

    def block_means(self, block: int, *, prior: bool = False) -> list[float]:
        """Mean pixel value per ``block x block`` block, row-major from the south-west
        (``ceil(ny/block)`` rows of ``ceil(nx/block)``); ``prior=True`` for the prior."""
        pr = self.prior
        block = max(int(block), 1)
        src = pr.values if prior else self.residual
        nbx = -(-pr.nx // block)
        nby = -(-pr.ny // block)
        acc = [0.0] * (nbx * nby)
        cnt = [0] * (nbx * nby)
        for i in range(pr.ny):
            bi = (i // block) * nbx
            base = i * pr.nx
            for j in range(pr.nx):
                k = bi + j // block
                acc[k] += src[base + j]
                cnt[k] += 1
        return [a / c for a, c in zip(acc, cnt)]

    def blocks(self, block: int) -> list[dict]:
        """``block x block`` pixel SUMS (ragged edge blocks sum what they have):
        ``[{"x", "y", "prior", "residual", "n", "i0", "j0"}]``, row-major from the south-west,
        with x/y the mean pixel centre [world ENU], ``n`` the pixel count, ``i0``/``j0`` the
        first raster row/column of the block.
        The prior and residual columns sum to ``prior_mass`` and the residual mass."""
        pr = self.prior
        block = max(int(block), 1)
        out = []
        for bi in range(0, pr.ny, block):
            rows = range(bi, min(bi + block, pr.ny))
            y = sum(pr.ys[i] for i in rows) / len(rows)
            for bj in range(0, pr.nx, block):
                cols = range(bj, min(bj + block, pr.nx))
                x = sum(pr.xs[j] for j in cols) / len(cols)
                p = r = 0.0
                for i in rows:
                    base = i * pr.nx
                    for j in cols:
                        p += pr.values[base + j]
                        r += self.residual[base + j]
                out.append({"x": x, "y": y, "prior": p, "residual": r,
                            "n": len(rows) * len(cols), "i0": bi, "j0": bj})
        return out

    def summary(self) -> dict:
        res = self.exact_mass()
        return {"prior_belief_mass": round(self.prior_mass, 6),
                "residual_belief_mass": round(res, 6),
                "searched_belief_mass": round(self.prior_mass - res, 6),
                "searched_belief_fraction": round((self.prior_mass - res) / self.prior_mass, 5)
                if self.prior_mass > 0 else 0.0,
                "residual_looks": self.looks}


def planned_residual(prior: PriorGrid, model: DetectionModel, fov_rad: float,
                     tracks: Iterable[Mapping[str, Sequence]]) -> ResidualBelief:
    """Residual belief of a PLAN: every planned sample is one look at its scheduled
    boresight point, weighted ``dt / dt_ref`` like the flown score.

    ``tracks``: per agent ``{"t": [...], "pos": [(x, y, z)...], "bore": [(x, y, z)...]}``
    in world ENU. The footprint radius follows the slant range to the boresight
    point, as in ``mtl::eval::computeResidualBelief``.
    """
    rb = ResidualBelief(prior, model)
    tan_half = math.tan(float(fov_rad) / 2.0)
    for tr in tracks:
        t, pos, bore = tr.get("t") or [], tr.get("pos") or [], tr.get("bore") or []
        n = min(len(t), len(pos), len(bore))
        if n == 0:
            continue
        dts = [t[k + 1] - t[k] for k in range(n - 1)]
        dt_nom = sorted(dts)[len(dts) // 2] if dts else model.dt_ref_s
        for k in range(n):
            dt = dts[k] if k < n - 1 else dt_nom  # the last sample looks for one period too
            p, g = pos[k], bore[k]
            if p is None or g is None:
                continue
            slant = math.sqrt((p[0] - g[0]) ** 2 + (p[1] - g[1]) ** 2 + (p[2] - g[2]) ** 2)
            if slant > model.beta:
                continue  # as TeamScorer: the look point itself is out of range
            w = max(dt, 0.0) / model.dt_ref_s if model.dt_ref_s > 0 else 1.0
            rb.look(p, g[0], g[1], slant * tan_half, w, g[2])
    return rb


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
                 cell_mass: Iterable[float], model: DetectionModel, fov_rad: float,
                 prior: PriorGrid | None = None, residual_snapshot_s: float | None = None,
                 residual_snapshot_block: int = 2) -> None:
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
        # residual belief over the whole prior (None when the scenario has no prior raster)
        self.residual = ResidualBelief(prior, model) if prior is not None else None
        self.residual_curve: list[float | None] = []
        # optional time series of the residual map (block means), e.g. for the Foxglove export
        self.residual_snapshot_s = residual_snapshot_s
        self.residual_snapshot_block = int(residual_snapshot_block)
        self.residual_snapshots: list[tuple[float, list[float]]] = []

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
            if self.residual is not None:
                self.residual.look(pos, gx, gy, radius, weight, s.get("ground_z", 0.0))
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
            self.residual_curve.append(self.residual.residual_mass if self.residual is not None else None)
            if self.residual is not None and self.residual_snapshot_s:
                last = self.residual_snapshots[-1][0] if self.residual_snapshots else -math.inf
                if t - last >= self.residual_snapshot_s - 1e-9:
                    self.residual_snapshots.append((t, self.residual.block_means(self.residual_snapshot_block)))

    # ------------------------------------------------------------------ #
    def summary(self) -> dict:
        found = [tg for tg in self.targets if tg.detected]
        times = [tg.detection_time_s for tg in found]
        dist = sum(self.distance_m.values())
        total = self.total_mass
        rb = self.residual.summary() if self.residual is not None else {
            "prior_belief_mass": None, "residual_belief_mass": None, "searched_belief_mass": None,
            "searched_belief_fraction": None, "residual_looks": 0}
        searched = rb["searched_belief_mass"]
        return {
            # headline planner score: P(target missed by the search), lower is better
            **rb,
            "searched_belief_per_km": round(searched / (dist / 1000.0), 6) if searched is not None and dist > 0
            else None,
            "targets_total": len(self.targets),
            "targets_detected": len(found),
            "targets_missed": len(self.targets) - len(found),
            "detection_threshold": self.model.threshold,
            "mean_time_to_discovery_s": (sum(times) / len(times)) if times else None,
            "total_path_length_m": round(dist, 2),
            "cells_total": len(self.cells),
            "cells_covered": sum(self.cell_covered),
            # cell coverage: a valid cell counts (its whole mass) once its centre fell in a footprint.
            # Masses are probabilities (the prior sums to 1), so these are P(target in ...) too.
            "belief_mass_total": round(total, 6),
            "belief_mass_covered": round(self.covered_mass, 6),
            "belief_mass_fraction": round(self.covered_mass / total, 5) if total > 0 else 0.0,
            "belief_mass_per_km": round(self.covered_mass / (dist / 1000.0), 6) if dist > 0 else 0.0,
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
