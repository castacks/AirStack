"""MTL search scenario: synthesis, I/O and frame conversions (stdlib only).

This module is the single Python definition of an MTL search *scenario* and is
imported from three places that share no ROS workspace:

* the robot container (``mtl_metrics_logger`` scores against the scenario),
* the Isaac Sim container (``search_mission_scene.py`` textures the ground and
  places the ground-truth targets from it),
* the host (``scripts/mtl_generate_scenario.py`` writes it,
  ``scripts/analyze_mtl_run.py`` re-scores runs against it).

It therefore depends on nothing but the standard library: no numpy, no rclpy.

The files it produces (next to each other, e.g. ``stacks/mtl_search/config/``):

``scenario.json``      planner-facing. A strict superset of ``mtl.scenario/1``
                       (the format ``cpp_planner/apps/mtl_plan_json.cpp`` reads),
                       so the stock ``mtl_plan`` tool can plan it too. Carries the
                       mission geometry, platform, sensor, team and the valid
                       cells the HOST extracted. Never contains targets.
``ground_truth.json``  the hidden targets. Read by the scene and the scorer only.
``belief.png``         the prior rendered as a heat map, draped on the ground.

Frames
------
``mission NED``  n North, e East, d Down [m], origin = Isaac world origin. The
                 scenario and ground truth are authored in it.
``world ENU``    Isaac Sim / AirStack world: x = e, y = n, z = -d.
``map``          one robot's odometry frame (PX4/MAVROS local origin = where
                 that robot spawned): p_map = p_worldENU - home_worldENU.
``mtl``          the planner's internal x East / y North over [0, mapSize];
                 handled only inside the C++ adapter (mirrors mtl_plan_json.cpp).
"""

from __future__ import annotations

import copy
import json
import math
import random
import struct
import zlib
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

__all__ = [
    "SCENARIO_SCHEMA",
    "GROUND_TRUTH_SCHEMA",
    "BeliefGrid",
    "ned_to_enu",
    "enu_to_ned",
    "world_to_map",
    "map_to_world",
    "generate_belief",
    "sample_targets",
    "extract_valid_cells",
    "build_scenario",
    "write_scenario_bundle",
    "load_json",
    "load_scenario",
    "load_ground_truth",
    "agent_entry",
    "agent_home_enu",
    "detection_probability",
    "check_fleet_consistency",
    "belief_rgb_rows",
    "write_png",
]

SCENARIO_SCHEMA = "mtl.scenario/1"
GROUND_TRUTH_SCHEMA = "mtl.ground_truth/1"


# --------------------------------------------------------------------------- #
# frames
# --------------------------------------------------------------------------- #
def ned_to_enu(n: float, e: float, d: float = 0.0) -> tuple[float, float, float]:
    """Mission NED -> world ENU."""
    return (float(e), float(n), -float(d))


def enu_to_ned(x: float, y: float, z: float = 0.0) -> tuple[float, float, float]:
    """World ENU -> mission NED."""
    return (float(y), float(x), -float(z))


def world_to_map(p_world: Sequence[float], home_world: Sequence[float]) -> tuple[float, float, float]:
    """World ENU -> a robot's map (odometry) frame, whose origin is its home."""
    return tuple(float(a) - float(b) for a, b in zip(p_world, home_world))  # type: ignore[return-value]


def map_to_world(p_map: Sequence[float], home_world: Sequence[float]) -> tuple[float, float, float]:
    return tuple(float(a) + float(b) for a, b in zip(p_map, home_world))  # type: ignore[return-value]


# --------------------------------------------------------------------------- #
# belief grid
# --------------------------------------------------------------------------- #
class BeliefGrid:
    """Prior belief raster over the square search area, indexed ``values[i_n][j_e]``."""

    def __init__(self, values: list[list[float]], n_axis: list[float], e_axis: list[float],
                 res_m: float) -> None:
        self.values = values
        self.n_axis = n_axis
        self.e_axis = e_axis
        self.res_m = float(res_m)

    @property
    def shape(self) -> tuple[int, int]:
        return (len(self.n_axis), len(self.e_axis))

    @property
    def pixel_area_m2(self) -> float:
        return self.res_m * self.res_m

    @property
    def total_mass(self) -> float:
        return sum(sum(row) for row in self.values) * self.pixel_area_m2

    @property
    def peak(self) -> float:
        return max(max(row) for row in self.values)

    def value_at(self, n: float, e: float) -> float:
        i = int(round((n - self.n_axis[0]) / self.res_m))
        j = int(round((e - self.e_axis[0]) / self.res_m))
        i = min(max(i, 0), len(self.n_axis) - 1)
        j = min(max(j, 0), len(self.e_axis) - 1)
        return self.values[i][j]


def _axis(lo: float, hi: float, step: float) -> list[float]:
    count = int(math.floor((hi - lo) / step + 1e-9)) + 1
    return [lo + k * step for k in range(count)]


def area_bounds(area: Mapping[str, Any]) -> tuple[float, float, float, float]:
    """(n_min, n_max, e_min, e_max) of the square search area."""
    half = float(area["size_m"]) / 2.0
    cn, ce = (float(v) for v in area.get("center_ned", (0.0, 0.0)))
    return (cn - half, cn + half, ce - half, ce + half)


def generate_belief(area: Mapping[str, Any], spec: Mapping[str, Any], seed: int
                    ) -> tuple[BeliefGrid, list[dict[str, float]]]:
    """Sum of axis-aligned Gaussian bumps, capped then floored. Deterministic in ``seed``.

    Port of ``testbed.mission.generate_belief`` (and of the MATLAB
    ``generateBeliefMap``), with ``random.Random`` instead of numpy so the
    result is reproducible on any Python without third-party packages.
    Returns the grid and the bump list (so the scene could rebuild it).
    """
    n_min, n_max, e_min, e_max = area_bounds(area)
    res = float(area.get("belief_res_m", 2.0))
    n_axis = _axis(n_min, n_max, res)
    e_axis = _axis(e_min, e_max, res)

    rng = random.Random(int(seed))
    size = float(area["size_m"])
    margin = float(spec.get("edge_margin_frac", 0.12)) * size
    sig_lo = float(spec.get("sigma_min_m", 20.0))
    sig_hi = float(spec.get("sigma_max_m", 45.0))
    peak = float(spec.get("max_prior_peak", 0.4))
    cap = float(spec.get("belief_cap", 0.85))
    floor = float(spec.get("base_uncertainty", 0.0))

    values = [[0.0] * len(e_axis) for _ in n_axis]
    bumps: list[dict[str, float]] = []
    for _ in range(int(spec.get("num_centroids", 6))):
        cn = rng.uniform(n_min + margin, n_max - margin)
        ce = rng.uniform(e_min + margin, e_max - margin)
        sn = rng.uniform(sig_lo, sig_hi)
        se = rng.uniform(sig_lo, sig_hi)
        bumps.append({"n": cn, "e": ce, "sigma_n": sn, "sigma_e": se, "amplitude": peak})
        gn = [math.exp(-0.5 * ((n - cn) / sn) ** 2) for n in n_axis]
        ge = [math.exp(-0.5 * ((e - ce) / se) ** 2) for e in e_axis]
        for i, gi in enumerate(gn):
            if gi < 1e-12:
                continue
            row = values[i]
            a = peak * gi
            for j, gj in enumerate(ge):
                row[j] += a * gj

    for row in values:
        for j, v in enumerate(row):
            v = min(v, cap)
            if floor > 0.0:
                v = max(v, floor)
            row[j] = v
    return BeliefGrid(values, n_axis, e_axis, res), bumps


def sample_targets(grid: BeliefGrid, spec: Mapping[str, Any], seed: int) -> list[dict[str, float]]:
    """Inverse-CDF sample of ground-truth targets from the prior (min-separation rejection)."""
    count = int(spec.get("count", 0))
    if count <= 0:
        return []
    min_sep = float(spec.get("min_separation_m", 0.0))
    rng = random.Random(int(seed))

    flat: list[float] = [v for row in grid.values for v in row]
    total = sum(flat)
    if total <= 0.0:
        raise ValueError("prior belief is everywhere zero; cannot sample targets")
    cdf: list[float] = []
    acc = 0.0
    for v in flat:
        acc += v
        cdf.append(acc / total)

    n_cols = len(grid.e_axis)
    picked: list[dict[str, float]] = []
    for _ in range(max(200 * count, 2000)):
        if len(picked) >= count:
            break
        u = rng.random()
        lo, hi = 0, len(cdf) - 1
        while lo < hi:  # bisect_left
            mid = (lo + hi) // 2
            if cdf[mid] < u:
                lo = mid + 1
            else:
                hi = mid
        i, j = divmod(lo, n_cols)
        n = grid.n_axis[i] + rng.uniform(-0.5, 0.5) * grid.res_m
        e = grid.e_axis[j] + rng.uniform(-0.5, 0.5) * grid.res_m
        if any(math.hypot(n - t["n"], e - t["e"]) < min_sep for t in picked):
            continue
        picked.append({"index": len(picked), "n": round(n, 3), "e": round(e, 3),
                       "belief": round(grid.values[i][j], 6)})
    if len(picked) < count:
        raise ValueError(
            f"could only place {len(picked)} of {count} targets at min_separation_m={min_sep}; "
            "lower it or widen the prior")
    return picked


def extract_valid_cells(grid: BeliefGrid, cell_size_m: float, mean_thresh: float
                        ) -> tuple[list[list[float]], list[float], float]:
    """Dice the prior into blocks; keep those whose MEAN beats the threshold.

    Kept blocks carry their AGGREGATE mass ``sum(pixels) * pixel_area`` (mean
    decides whether to go, mass decides what it is worth). Returns
    ``(centers_ned, masses, effective_cell_size_m)``.
    """
    block = max(int(round(cell_size_m / grid.res_m)), 1)
    n_blocks = len(grid.n_axis) // block
    e_blocks = len(grid.e_axis) // block
    if n_blocks == 0 or e_blocks == 0:
        raise ValueError("target_cell_size_m is larger than the search area")
    centers: list[list[float]] = []
    masses: list[float] = []
    for bi in range(n_blocks):
        rows = grid.values[bi * block:(bi + 1) * block]
        n_mean = sum(grid.n_axis[bi * block:(bi + 1) * block]) / block
        for bj in range(e_blocks):
            s = 0.0
            for row in rows:
                s += sum(row[bj * block:(bj + 1) * block])
            mean = s / (block * block)
            if mean <= mean_thresh:
                continue
            e_mean = sum(grid.e_axis[bj * block:(bj + 1) * block]) / block
            centers.append([round(n_mean, 4), round(e_mean, 4)])
            masses.append(round(s * grid.pixel_area_m2, 6))
    if not centers:
        raise ValueError("no valid cells: mean_information_thresh is above every block's mean")
    return centers, masses, block * grid.res_m


# --------------------------------------------------------------------------- #
# the scenario bundle
# --------------------------------------------------------------------------- #
def _finite_or_none(v: Any) -> float | None:
    if v is None:
        return None
    f = float(v)
    return f if math.isfinite(f) else None


def build_scenario(mission: Mapping[str, Any], agents: Sequence[Mapping[str, Any]],
                   *, provenance: Mapping[str, Any] | None = None
                   ) -> tuple[dict[str, Any], dict[str, Any], BeliefGrid]:
    """Mission dict (``mission.yaml``'s ``mission:`` block) + agents -> bundle.

    ``agents``: ``[{"name": "robot_1", "home_ned": [n, e]}, ...]`` in robot
    order (robot N = domain N). ``start_ned`` defaults to ``home_ned``: a
    multirotor takes off where it spawned and flies from there.
    """
    m = copy.deepcopy(dict(mission))
    seed = int(m.get("seed", 21))
    area = m["area"]
    grid, bumps = generate_belief(area, m.get("belief", {}), seed)
    targets = sample_targets(grid, m.get("targets", {}), seed + 1)
    mp = m.get("mapping", {})
    centers, masses, cell_size = extract_valid_cells(
        grid, float(mp.get("target_cell_size_m", 20.0)),
        float(mp.get("mean_information_thresh", 0.08)))

    air = m.get("aircraft", {})
    sensor = m.get("sensor", {})
    team = m.get("team", {})
    det = sensor.get("detection", {})

    # Vertical deconfliction: agent i cruises at altitude_m + i * altitude_separation_m.
    # The planner solves the team problem at altitude_m (the offsets are far
    # below its sensor-geometry sensitivity); the offset is applied to each
    # agent's flown track by the ROS adapter and to its takeoff altitude by
    # mtl_sortie.sh. 0 (or absent) = every agent at altitude_m.
    alt_sep = float(team.get("altitude_separation_m", 0.0) or 0.0)
    team_agents = []
    for i, a in enumerate(agents):
        home = [float(a["home_ned"][0]), float(a["home_ned"][1])]
        start = [float(v) for v in a.get("start_ned", home)]
        entry: dict[str, Any] = {"name": str(a["name"]), "start_ned": start, "home_ned": home}
        if alt_sep:
            entry["altitude_offset_m"] = round(i * alt_sep, 6)
        team_agents.append(entry)
    if not team_agents:
        raise ValueError("scenario needs at least one agent")

    scenario: dict[str, Any] = {
        "schema": SCENARIO_SCHEMA,
        "mission": {
            "name": m.get("name", "mtl_search"),
            "seed": seed,
            "area": {
                "size_m": float(area["size_m"]),
                "center_ned": [float(v) for v in area.get("center_ned", (0.0, 0.0))],
                "belief_res_m": float(area.get("belief_res_m", 2.0)),
            },
        },
        "aircraft": {
            "altitude_m": float(air.get("altitude_m", 30.0)),
            "speed_mps": float(air.get("speed_mps", 6.0)),
            "min_turn_radius_m": float(air.get("min_turn_radius_m", 12.0)),
            "dt": float(air.get("dt", 0.1)),
            "dubins_step_m": float(air.get("dubins_step_m", 0.5)),
        },
        "mapping": {
            "target_cell_size_m": float(cell_size),
            "mean_information_thresh": float(mp.get("mean_information_thresh", 0.08)),
            "max_cluster_radius_m": float(mp.get("max_cluster_radius_m", 45.0)),
            "kmeans_replicates": int(mp.get("kmeans_replicates", 3)),
            "kmeans_max_iter": int(mp.get("kmeans_max_iter", 200)),
        },
        "sensor": {
            "fov_deg": float(sensor.get("fov_deg", 60.0)),
            "single_axis_gimbal": bool(sensor.get("single_axis_gimbal", True)),
            "tilt_deg": float(sensor.get("tilt_deg", 30.0)),
            "max_slant_range_m": float(sensor.get("max_slant_range_m", 90.0)),
            "detection": {
                "a": float(det.get("a", 1.10)),
                "b": float(det.get("b", 0.10)),
                "c": float(det.get("c", 61.0)),
                "beta": float(det.get("beta", 61.0)),
                "p_out_of_range": float(det.get("p_out_of_range", 1.0e-6)),
                "threshold": float(det.get("threshold", 0.9)),
                "dt_ref_s": float(det.get("dt_ref_s", air.get("dt", 0.1))),
            },
        },
        "gimbal": dict(m.get("gimbal", {})),
        "team": {
            "max_flight_time_s": _finite_or_none(team.get("max_flight_time_s")),
            "max_flight_distance_m": _finite_or_none(team.get("max_flight_distance_m")),
            "altitude_separation_m": alt_sep,
            "agents": team_agents,
        },
        "cells": {
            "centers": centers,
            "mass": masses,
            "total_map_mass": round(grid.total_mass, 6),
        },
        "solver": dict(m.get("solver", {})),
        "verbose": False,
        "verify_geometry": True,
        "verify_geometry_verbose": False,
        # --- AirStack extension block (ignored by mtl_plan) ----------------
        "airstack": {
            "frames": {
                "scenario": "mission NED (n, e, d) [m], origin = Isaac world origin",
                "world": "ENU: x = e, y = n, z = -d",
                "map": "per-robot odometry frame; origin = that robot's home (spawn)",
            },
            "belief": {
                "bumps": [{k: round(v, 6) for k, v in b.items()} for b in bumps],
                "belief_cap": float(m.get("belief", {}).get("belief_cap", 0.85)),
                "base_uncertainty": float(m.get("belief", {}).get("base_uncertainty", 0.0)),
                "peak": round(grid.peak, 6),
                "texture": "belief.png",
            },
            "flight": dict(m.get("flight", {})),
            "sim_gimbal": dict(m.get("sim_gimbal", {})),
            "render": dict(m.get("render", {})),
            "provenance": dict(provenance or {}),
        },
    }
    ground_truth = {
        "schema": GROUND_TRUTH_SCHEMA,
        "mission": scenario["mission"]["name"],
        "seed": seed,
        "frame": "mission NED; z = 0 on the ground plane",
        "note": "GROUND TRUTH ONLY - never handed to a planner.",
        "render": dict(m.get("targets", {}).get("render", {})),
        "targets": targets,
    }
    return scenario, ground_truth, grid


def write_scenario_bundle(out_dir: str | Path, scenario: Mapping[str, Any],
                          ground_truth: Mapping[str, Any], grid: BeliefGrid,
                          texture_px: int | None = None) -> dict[str, Path]:
    """Write ``scenario.json``, ``ground_truth.json`` and ``belief.png``."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = {
        "scenario": out / "scenario.json",
        "ground_truth": out / "ground_truth.json",
        "belief_png": out / "belief.png",
    }
    paths["scenario"].write_text(json.dumps(scenario, indent=1, sort_keys=False) + "\n",
                                 encoding="utf-8")
    paths["ground_truth"].write_text(json.dumps(ground_truth, indent=1) + "\n", encoding="utf-8")
    render = scenario.get("airstack", {}).get("render", {})
    cells = scenario["cells"]["centers"] if render.get("show_valid_cells", False) else []
    rows = belief_rgb_rows(grid, texture_px or int(render.get("texture_px", 512)),
                           cells=cells, cell_size=scenario["mapping"]["target_cell_size_m"])
    write_png(paths["belief_png"], rows)
    return paths


# --------------------------------------------------------------------------- #
# loading
# --------------------------------------------------------------------------- #
def load_json(path: str | Path) -> dict[str, Any]:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def load_scenario(path: str | Path) -> dict[str, Any]:
    sc = load_json(path)
    if sc.get("schema") != SCENARIO_SCHEMA:
        raise ValueError(f"{path}: schema is {sc.get('schema')!r}, expected {SCENARIO_SCHEMA!r}")
    if not sc.get("team", {}).get("agents"):
        raise ValueError(f"{path}: scenario has no team.agents")
    return sc


def load_ground_truth(path: str | Path) -> dict[str, Any]:
    gt = load_json(path)
    if gt.get("schema") != GROUND_TRUTH_SCHEMA:
        raise ValueError(f"{path}: schema is {gt.get('schema')!r}, expected {GROUND_TRUTH_SCHEMA!r}")
    return gt


def agent_entry(scenario: Mapping[str, Any], name: str) -> tuple[int, dict[str, Any]]:
    """(index, entry) of the team agent called ``name`` (e.g. ``robot_2``)."""
    agents = scenario["team"]["agents"]
    for i, a in enumerate(agents):
        if a["name"] == name:
            return i, dict(a)
    raise KeyError(f"agent {name!r} is not in the scenario team "
                   f"({', '.join(a['name'] for a in agents)})")


def agent_home_enu(scenario: Mapping[str, Any], name: str, z: float = 0.0) -> tuple[float, float, float]:
    _, a = agent_entry(scenario, name)
    n, e = a["home_ned"]
    return (float(e), float(n), float(z))


def detection_probability(r: float, det: Mapping[str, Any]) -> float:
    """Moon et al. (2022): ``1 / (a + exp(b (r - c)))`` for ``r <= beta``."""
    if r > float(det["beta"]):
        return float(det.get("p_out_of_range", 0.0))
    return 1.0 / (float(det["a"]) + math.exp(float(det["b"]) * (r - float(det["c"]))))


def check_fleet_consistency(scenario: Mapping[str, Any], spawns_enu: Mapping[str, Sequence[float]],
                            tol_m: float = 0.5) -> list[str]:
    """Named mismatches between the scenario's agent homes and fleet spawns."""
    problems = []
    names = [a["name"] for a in scenario["team"]["agents"]]
    for name, spawn in spawns_enu.items():
        if name not in names:
            problems.append(f"fleet robot {name} is not a scenario agent ({', '.join(names)})")
            continue
        hx, hy, _ = agent_home_enu(scenario, name)
        d = math.hypot(float(spawn[0]) - hx, float(spawn[1]) - hy)
        if d > tol_m:
            problems.append(f"{name}: fleet spawn ({spawn[0]:g}, {spawn[1]:g}) is {d:.1f} m from the "
                            f"scenario home ({hx:g}, {hy:g}) - regenerate the scenario")
    for name in names:
        if name not in spawns_enu:
            problems.append(f"scenario agent {name} has no fleet spawn")
    return problems


# --------------------------------------------------------------------------- #
# texture
# --------------------------------------------------------------------------- #
_INFERNO = [  # (t, r, g, b) - a compact inferno-like ramp
    (0.00, 0.001, 0.000, 0.014),
    (0.15, 0.106, 0.047, 0.259),
    (0.30, 0.337, 0.063, 0.431),
    (0.45, 0.576, 0.149, 0.404),
    (0.60, 0.788, 0.259, 0.278),
    (0.75, 0.941, 0.463, 0.141),
    (0.90, 0.980, 0.733, 0.188),
    (1.00, 0.988, 0.998, 0.645),
]


def _ramp(t: float, stops=_INFERNO) -> tuple[float, float, float]:
    t = min(max(t, 0.0), 1.0)
    for (t0, r0, g0, b0), (t1, r1, g1, b1) in zip(stops, stops[1:]):
        if t <= t1:
            w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            return (r0 + w * (r1 - r0), g0 + w * (g1 - g0), b0 + w * (b1 - b0))
    return stops[-1][1:]


def belief_rgb_rows(grid: BeliefGrid, px: int, *, ground_rgb=(0.30, 0.38, 0.23),
                    cells: Sequence[Sequence[float]] = (), cell_size: float = 0.0,
                    cell_rgb=(0.20, 0.85, 1.0), cell_alpha=0.35) -> list[bytes]:
    """RGB rows for the ground texture. Row 0 = NORTH edge, column 0 = WEST edge.

    Low belief blends into a grass colour so the plane reads as terrain; the
    optional cell outlines mark the blocks the planner aims at.
    """
    px = max(16, int(px))
    n_min, n_max = grid.n_axis[0], grid.n_axis[-1]
    e_min, e_max = grid.e_axis[0], grid.e_axis[-1]
    peak = grid.peak or 1.0
    outline = [False] * (px * px)
    if cells and cell_size > 0:
        half = cell_size / 2.0
        for cn, ce in cells:
            for n0, e0, n1, e1 in (
                (cn - half, ce - half, cn - half, ce + half),
                (cn + half, ce - half, cn + half, ce + half),
                (cn - half, ce - half, cn + half, ce - half),
                (cn - half, ce + half, cn + half, ce + half),
            ):
                steps = max(2, int(px * max(abs(n1 - n0), abs(e1 - e0)) / (n_max - n_min)) + 1)
                for k in range(steps + 1):
                    w = k / steps
                    n = n0 + w * (n1 - n0)
                    e = e0 + w * (e1 - e0)
                    r = int((n_max - n) / (n_max - n_min) * (px - 1))
                    c = int((e - e_min) / (e_max - e_min) * (px - 1))
                    if 0 <= r < px and 0 <= c < px:
                        outline[r * px + c] = True
    rows: list[bytes] = []
    for r in range(px):
        n = n_max - (r + 0.5) / px * (n_max - n_min)
        buf = bytearray()
        for c in range(px):
            e = e_min + (c + 0.5) / px * (e_max - e_min)
            t = grid.value_at(n, e) / peak
            hr, hg, hb = _ramp(t)
            w = min(1.0, t * 4.0)  # heat fades in over the lowest quarter
            rgb = [ground_rgb[0] * (1 - w) + hr * w, ground_rgb[1] * (1 - w) + hg * w,
                   ground_rgb[2] * (1 - w) + hb * w]
            if outline[r * px + c]:
                rgb = [rgb[k] * (1 - cell_alpha) + cell_rgb[k] * cell_alpha for k in range(3)]
            buf.extend(int(max(0.0, min(1.0, v)) * 255 + 0.5) for v in rgb)
        rows.append(bytes(buf))
    return rows


def write_png(path: str | Path, rgb_rows: Iterable[bytes]) -> None:
    """Minimal 8-bit RGB PNG encoder (zlib + CRC), no third-party imports."""
    rows = list(rgb_rows)
    height = len(rows)
    width = len(rows[0]) // 3 if rows else 0
    raw = b"".join(b"\x00" + row for row in rows)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 9))
           + chunk(b"IEND", b""))
    Path(path).write_bytes(png)
