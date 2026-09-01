#!/usr/bin/env python3
"""hurricane_house_cut_search.py — re-cut `hurricane._HOUSE_CUTS` against the
REAL house population, not a synthetic sample.

WHY THIS EXISTS. `hurricane.house_level_for_intensity`'s own docstring
claimed level 3 lands at 25% structural, calibrated against 800 SYNTHETIC
houses (uniform intensity draws re-weighted by `CODE_ERAS`' population
shares, in isolation). The rendered level-3 plate did not agree:
`GT_hurricane.json` (`~/hurricane_previews/V2_L3/`) recorded 21
`roof_collapsed` + 14 `partial_collapse` + 1 `leveled` of 94 houses = 38%
structural. The synthetic sample never saw the field's own noise/streak
covariance or the actual (x, y) positions the layout drew, so it could not
catch this.

Every house already reachable in a rendered scene carries its own
`intensity` and `vulnerability` in its GT record (the launcher writes both
per house). This tool loads that record directly and:

  1. TALLIES the current (or any candidate) `_HOUSE_CUTS` against the real
     population, resampling `jitter` many times per house so the reported
     fractions are a smooth expectation rather than one noisy draw — the
     same maths `house_level_for_intensity` runs per house, just batched.
  2. SEARCHES new boundaries by weighted least-squares against five named
     level-3 targets at once (pristine+shingles, cover_lost,
     deck_panels_lost, roof_stripped, structural), because tuning any ONE
     of them in isolation (as the first 2026-08-31 re-cut did, capping only
     structural) just pushes the pile-up into the rung below it — see
     `hurricane.py`'s `_HOUSE_CUTS` comment for the two-round history.

USAGE (read-only report, no pxr needed — `hurricane.py` has no pxr import):

    python3 scene_gen/tools/hurricane_house_cut_search.py --report
    python3 scene_gen/tools/hurricane_house_cut_search.py --search

`test_hurricane_house_cuts.py` imports `tally` and `load_gt` directly rather
than shelling out to this file, so the two never drift.
"""
import argparse
import json
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _SCENE_GEN)

from disaster import hurricane as hu  # noqa: E402

STRUCTURAL = ("roof_collapsed", "partial_collapse", "leveled")

DEFAULT_GT = {
    "L2": os.path.expanduser("~/hurricane_previews/V2_L2/GT_hurricane.json"),
    "L3": os.path.expanduser("~/hurricane_previews/V2_L3/GT_hurricane.json"),
}


def load_gt(path):
    """`[(intensity, vulnerability), ...]` for every house in a GT file.

    Both fields are recorded per house by the launcher already — this does
    NOT re-derive them from `(x, y)` and a fresh field/vuln draw, so it is
    exact regardless of which RNG stream produced the original scene.
    """
    with open(path) as fh:
        doc = json.load(fh)
    return [(float(h["intensity"]), float(h["vulnerability"]))
            for h in doc["houses"]]


def tally(pairs, cuts, jitter=0.06, trials=300, seed=12345):
    """`{level: fraction}` for `cuts` over `pairs`, resampling `jitter`
    `trials` times per house — the same blend `house_level_for_intensity`
    computes, batched for a smooth expectation instead of one noisy draw.
    """
    rng = random.Random(seed)
    counts = {lv: 0 for lv in hu.HOUSE_LEVELS}
    total = 0
    for it, vuln in pairs:
        eff0 = it / hu._resistance(vuln)
        for _ in range(trials):
            lv = hu._ladder(cuts, eff0 + rng.uniform(-jitter, jitter))
            counts[lv] += 1
            total += 1
    return {k: v / total for k, v in counts.items()}


def struct_frac(dist):
    return sum(dist.get(k, 0.0) for k in STRUCTURAL)


def named_metrics(dist):
    """`(pristine+shingles, cover_lost, deck_panels_lost, roof_stripped,
    structural)` — the five acceptance numbers the search targets."""
    return (dist.get("pristine", 0.0) + dist.get("shingles_lost", 0.0),
            dist.get("cover_lost", 0.0),
            dist.get("deck_panels_lost", 0.0),
            dist.get("roof_stripped", 0.0),
            struct_frac(dist))


def _report(cuts=None, trials=800):
    cuts = cuts or hu._HOUSE_CUTS
    for name, path in DEFAULT_GT.items():
        if not os.path.exists(path):
            print("{0}: no GT file at {1}".format(name, path))
            continue
        pairs = load_gt(path)
        dist = tally(pairs, cuts, trials=trials)
        m = named_metrics(dist)
        print("{0} ({1} houses):".format(name, len(pairs)))
        for k in sorted(dist, key=lambda z: -dist[z]):
            print("  {0:18s} {1:6.2%}".format(k, dist[k]))
        print("  -> pristine+shingles {0:.1%}  cover {1:.1%}  deck {2:.1%}  "
              "roof_stripped {3:.1%}  structural {4:.1%}".format(*m))


def _search(target=(0.15, 0.30, 0.25, 0.18, 0.12), weight_structural=4.0,
            trials=300):
    import numpy as np
    from scipy.optimize import minimize

    pairs3 = load_gt(DEFAULT_GT["L3"])
    eff0 = np.array([it / hu._resistance(v) for it, v in pairs3])
    rng = np.random.default_rng(777)
    jitter = rng.uniform(-0.06, 0.06, size=(trials, len(eff0)))
    target = np.array(target)
    weights = np.array([1.0, 1.0, 1.0, 1.0, weight_structural])

    def frac_of(bounds):
        eff = eff0[None, :] + jitter
        idx = np.searchsorted(np.asarray(bounds), eff, side="right")
        counts = np.bincount(idx.ravel(), minlength=8)
        return counts / idx.size

    def metrics_of(f):
        return np.array([f[0] + f[1], f[2], f[3], f[4], f[5] + f[6] + f[7]])

    def loss(x):
        bounds = np.cumsum(np.exp(x))
        return float(np.sum(weights * (metrics_of(frac_of(bounds)) - target) ** 2))

    x0 = np.log(np.diff(np.concatenate([[0.0], [0.30, 0.44, 0.58, 0.68, 0.80,
                                                 0.90, 1.05]])))
    res = minimize(loss, x0, method="Nelder-Mead",
                   options={"maxiter": 4000, "xatol": 1e-4, "fatol": 1e-8})
    bounds = np.cumsum(np.exp(res.x))
    names = ("pristine", "shingles_lost", "cover_lost", "deck_panels_lost",
             "roof_stripped", "roof_collapsed", "partial_collapse")
    print("loss", res.fun)
    for n, b in zip(names, bounds):
        print("  {0:18s} {1:.3f}".format(n, b))
    return bounds


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", action="store_true",
                    help="tally the CURRENT hurricane._HOUSE_CUTS")
    ap.add_argument("--search", action="store_true",
                    help="fit new boundaries against the level-3 targets")
    args = ap.parse_args()
    if args.search:
        _search()
    else:
        _report()


if __name__ == "__main__":
    main()
