#!/usr/bin/env python3
"""Generate a benchmark scenario set (mtl.scenario/1 files) from a spec.

    python3 scripts/planner_benchmark/gen_scenarios.py --spec scripts/planner_benchmark/specs/wide.json \\
        --out bench/wide

Each scenario is a copy of a template bundle (default: stacks/mtl_search/config/scenario.json,
so aircraft, sensor, detection model, mapping and solver settings are the ones you fly) with
the PRIOR, the CELLS, the BUDGET and optionally the ALTITUDE and HOME replaced. The prior is a
bump mixture (see priors.py), which every planner and scorer in the stack rebuilds the same way.

Writes ``<out>/scenarios/<id>.json`` and ``<out>/index.json`` (one record per scenario: the
factors that produced it and descriptors of the prior, used by make_report.py to explain where
each planner wins). Deterministic for a given spec and seed. Needs numpy.
"""

from __future__ import annotations

import argparse
import copy
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
from priors import FAMILIES, make_prior, rasterize, sample_family_params  # noqa: E402

DEFAULT_TEMPLATE = REPO / "stacks/mtl_search/config/scenario.json"


def extract_cells(na, ea, V, cell_m, min_mass):
    """Whole blocks of `cell_m`, kept when their mass beats `min_mass` (scenario.py's rule)."""
    res = na[1] - na[0]
    b = max(1, int(round(cell_m / res)))
    nb, eb = na.size // b, ea.size // b
    blk = V[:nb * b, :eb * b].reshape(nb, b, eb, b).sum(axis=(1, 3))
    cn = na[:nb * b].reshape(nb, b).mean(1)
    ce = ea[:eb * b].reshape(eb, b).mean(1)
    keep = np.argwhere(blk > min_mass)
    centers = [[round(float(cn[i]), 4), round(float(ce[j]), 4)] for i, j in keep]
    masses = [round(float(blk[i, j]), 12) for i, j in keep]
    return centers, masses, blk


def descriptors(na, ea, V, blk, home, budget_m, cell_m):
    """Numbers that describe the prior, for the report's "where does each planner win" analysis."""
    p = V.ravel()
    nz = p[p > 0]
    entropy = float(-(nz * np.log(nz)).sum())
    eff_area_km2 = float(math.exp(entropy) * (na[1] - na[0]) ** 2 / 1e6)
    N, E = np.meshgrid(na, ea, indexing="ij")
    d = np.hypot(N - home[0], E - home[1])
    mean_dist = float((V * d).sum())
    within_half_budget = float(V[d <= budget_m / 2].sum())
    cn, ce = float((V * N).sum()), float((V * E).sum())
    spread = float(math.sqrt((V * ((N - cn) ** 2 + (E - ce) ** 2)).sum()))
    q = np.sort(blk.ravel())[::-1]
    c = np.cumsum(q)
    cells_for_50 = int(np.searchsorted(c, 0.5 * c[-1]) + 1)
    cells_for_90 = int(np.searchsorted(c, 0.9 * c[-1]) + 1)
    # peaks: 8-connected components of the block grid above 25 % of its max
    mask = blk > 0.25 * blk.max()
    seen = np.zeros_like(mask, bool)
    peaks = 0
    for i, j in np.argwhere(mask):
        if seen[i, j]:
            continue
        peaks += 1
        stack = [(i, j)]
        seen[i, j] = True
        while stack:
            a, b = stack.pop()
            for da, db in itertools.product((-1, 0, 1), repeat=2):
                x, y = a + da, b + db
                if 0 <= x < mask.shape[0] and 0 <= y < mask.shape[1] and mask[x, y] and not seen[x, y]:
                    seen[x, y] = True
                    stack.append((x, y))
    gini_sorted = np.sort(blk.ravel())
    n = gini_sorted.size
    gini = float((2 * np.arange(1, n + 1) - n - 1).dot(gini_sorted) / (n * gini_sorted.sum()))
    return {"entropy": round(entropy, 4), "effective_area_km2": round(eff_area_km2, 4),
            "mean_dist_from_home_m": round(mean_dist, 1), "mass_within_half_budget": round(within_half_budget, 4),
            "spread_m": round(spread, 1), "cells_for_50pct": cells_for_50, "cells_for_90pct": cells_for_90,
            "peaks_25pct": peaks, "gini": round(gini, 4),
            "budget_over_spread": round(budget_m / max(spread, 1.0), 3)}


def _py(o):
    """numpy scalars / arrays -> plain JSON types."""
    if hasattr(o, "item") and getattr(o, "ndim", 0) == 0:
        return o.item()
    if hasattr(o, "tolist"):
        return o.tolist()
    if isinstance(o, tuple):
        return list(o)
    raise TypeError(f"not JSON serializable: {type(o)}")


def pick(rng, v):
    """A spec value: a scalar, a list to choose from, or {"range": [lo, hi]}."""
    if isinstance(v, dict) and "range" in v:
        lo, hi = v["range"]
        return float(rng.uniform(lo, hi))
    if isinstance(v, list):
        return v[int(rng.integers(len(v)))]
    return v


def build(spec: dict, template: dict, out: Path, res: float) -> list[dict]:
    rng = np.random.default_rng(int(spec.get("seed", 0)))
    size = float(template["mission"]["area"]["size_m"])
    cn0, ce0 = template["mission"]["area"].get("center_ned", [0.0, 0.0])
    area = (cn0 - size / 2, cn0 + size / 2, ce0 - size / 2, ce0 + size / 2)
    fams = spec.get("families", {k: 1.0 for k in FAMILIES})
    names = list(fams)
    w = np.array([float(fams[k]) for k in names])
    w = w / w.sum()
    homes = spec.get("homes", {"near_center": [-170.0, -170.0]})
    count = int(spec.get("count", 100))
    cell_m = float(template["mapping"]["target_cell_size_m"])
    min_mass = float(spec.get("minimum_belief_mass", template["mapping"]["minimum_belief_mass"]))
    cap = float(spec.get("belief_cap", 0.85))
    (out / "scenarios").mkdir(parents=True, exist_ok=True)
    template = copy.deepcopy(template)
    for dotted, v in (spec.get("overrides") or {}).items():  # e.g. "sensor.detection.beta": 900
        d = template
        ks = dotted.split(".")
        for kk in ks[:-1]:
            d = d.setdefault(kk, {})
        d[ks[-1]] = v
    index = []
    for k in range(count):
        fam = names[int(rng.choice(len(names), p=w))]
        params = dict(sample_family_params(fam, rng))
        params.update((spec.get("family_overrides") or {}).get(fam, {}))
        home_name = list(homes)[int(rng.integers(len(homes)))]
        home = [float(v) for v in homes[home_name]]
        budget_s = float(pick(rng, spec.get("budget_s", [1000])))
        alt = float(pick(rng, spec.get("altitude_m", [template["aircraft"]["altitude_m"]])))
        seed = int(rng.integers(1 << 31))
        prng = np.random.default_rng(seed)
        bumps, floor, used = make_prior(fam, prng, area, home=home, **params)
        na, ea, V = rasterize(bumps, floor, area, res=res, cap=cap)
        centers, masses, blk = extract_cells(na, ea, V, cell_m, min_mass)
        if not centers:
            continue
        sc = copy.deepcopy(template)
        sid = f"s{k:04d}_{fam}"
        sc["mission"]["name"] = sid
        sc["mission"]["seed"] = seed
        sc["aircraft"]["altitude_m"] = alt
        sc["team"]["max_flight_time_s"] = budget_s
        sc["team"]["max_flight_distance_m"] = None
        team_size = int(pick(rng, spec.get("team_size", [1])))
        # a team launches from one site, 12 m apart (like the fleet file's spawns)
        sc["team"]["agents"] = [{"name": f"robot_{i + 1}", "start_ned": [home[0], home[1] + 12.0 * i],
                                 "home_ned": [home[0], home[1] + 12.0 * i], "altitude_offset_m": 0.0}
                                for i in range(team_size)]
        sc["cells"] = {"centers": centers, "mass": masses, "total_map_mass": 1.0}
        sc.setdefault("airstack", {})["belief"] = {
            "bumps": [{kk: round(vv, 6) for kk, vv in b.items()} for b in bumps], "belief_cap": cap,
            "base_uncertainty": round(floor, 9), "normalised": True, "peak": float(V.max()), "texture": "belief.png"}
        sc.pop("info_aware", None)
        sc["airstack"].pop("gimbal_actuation", None)
        (out / "scenarios" / f"{sid}.json").write_text(json.dumps(sc, default=_py))
        budget_m = budget_s * float(sc["aircraft"]["speed_mps"])
        rec = {"id": sid, "family": fam, "params": {kk: (list(vv) if isinstance(vv, tuple) else vv)
                                                   for kk, vv in used.items() if kk != "home"},
               "home": home_name, "budget_s": budget_s, "budget_m": budget_m, "altitude_m": alt,
               "team_size": team_size,
               "n_bumps": len(bumps), "floor": floor, "n_cells": len(centers), "cells_mass": round(sum(masses), 4),
               **descriptors(na, ea, V, blk, home, budget_m, cell_m)}
        index.append(rec)
    (out / "index.json").write_text(json.dumps({"spec": spec, "res_m": res, "scenarios": index}, indent=1,
                                              default=_py))
    return index


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--spec", type=Path, required=True, help="JSON spec (see specs/*.json)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    ap.add_argument("--res", type=float, default=5.0, help="raster resolution for the cells [m] (default 5)")
    args = ap.parse_args(argv)
    spec = json.loads(args.spec.read_text())
    template = json.loads(args.template.read_text())
    idx = build(spec, template, args.out, args.res)
    fams = {}
    for r in idx:
        fams[r["family"]] = fams.get(r["family"], 0) + 1
    print(f"[gen_scenarios] {len(idx)} scenarios -> {args.out}/scenarios ({', '.join(f'{k} {v}' for k, v in sorted(fams.items()))})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
