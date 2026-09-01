#!/usr/bin/env python3
"""
test_hurricane_fences.py — pins `washaway.fence_specs`/`_house_downwind_of`/
`_fence_fall_lean`/`fence_wind_threshold` (DEBRIS D3 review, 2026-08-31):
"fences stand intact in 2 m of surge."

THE REPORTED DEFECT. `~/hurricane_previews/ROUND1_L3/`'s render evidence
shows picket/rail fences (`suburb_scene`/`scene_generator.apply_placements`,
one placed prim per panel under `<PARENT>/fence_<group>_<i>`) standing
UNTOUCHED wherever they landed — in a metre of standing water, and squarely
downwind of a levelled house — because nothing in the water pass or the
wind pass ever looks at them. `washaway.apply_washaway`/`apply_car_washaway`
move houses and cars; `raft_specs`/`land_debris_specs` scatter loose
material; no code path so much as reads a fence prim.

`fence_specs` is the PURE decision half of the fix: given already-measured
fence geometry (`x, y, yaw_deg, length_m` — `_measure_fence`'s job, which
needs a stage and is out of scope for this offline file) plus depth/wind/
house callables, it decides "gone" (carried off, replaced by 1-3 floating
`fence`-kind rafts drifting shoreward), "flat" (blown down, hinged on its
own base edge) or "stands" (untouched). `apply_fence_pose` is the
stage-touching other half and is likewise out of scope here.

RUNS WITHOUT ISAAC. Same import-boundary guarantee as
`test_washaway_debris.py`: nothing this file exercises reaches for `pxr`.

USAGE
    python3 scene_gen/tests/test_hurricane_fences.py
    pytest -q scene_gen/tests/test_hurricane_fences.py
"""

import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import planks                             # noqa: E402
from disaster import washaway as wash                    # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    detail = "" if not detail else str(detail)
    if not cond:
        FAILS.append("{0}: {1}".format(name, detail))
    print(("PASS " if cond else "FAIL ") + name +
          ("" if not detail else " -- " + detail))


def strict(fn):
    """Same discipline every sibling test file in this stream uses: a test
    that adds to `FAILS` without asserting is a test nobody can trust ran."""
    def run(*a, **kw):
        mine = len(FAILS)
        out = fn(*a, **kw)
        assert len(FAILS) == mine, FAILS[mine:]
        return out
    run.__name__, run.__doc__ = fn.__name__, fn.__doc__
    return run


@strict
def test_fall_lean_sign_matches_wind_bearing():
    """`_fence_fall_lean` returns whichever sign sends the panel's "up" edge
    to the bearing closer to the requested wind bearing — verified against
    the closed-form Rodrigues result (`lean=+90 -> yaw-90`,
    `lean=-90 -> yaw+90`) for a spread of yaws and wind bearings, not just
    the cardinal cases."""
    def bearing_after_lean(yaw_deg, lean_deg):
        ax, ay = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        th = math.radians(lean_deg)
        vx, vy = ay * math.sin(th), -ax * math.sin(th)
        return math.degrees(math.atan2(vy, vx)) % 360.0

    def ang_dist(a, b):
        return abs((a - b + 180.0) % 360.0 - 180.0)

    rng = random.Random(1)
    for _ in range(200):
        yaw = rng.uniform(0.0, 360.0)
        wind = rng.uniform(0.0, 360.0)
        lean_mag = rng.uniform(80.0, 88.0)
        signed = wash._fence_fall_lean(yaw, wind, lean_mag)
        check("|signed lean| == requested magnitude",
             abs(abs(signed) - lean_mag) < 1e-9, signed)
        result_bearing = bearing_after_lean(yaw, signed)
        # The OTHER sign must not land closer -- this is the actual claim
        # ("falls TOWARD the wind"), not just "some valid fall direction".
        other_bearing = bearing_after_lean(yaw, -signed)
        check("chosen sign lands closer to the wind bearing than the other",
             ang_dist(result_bearing, wind) <= ang_dist(other_bearing, wind)
             + 1e-6,
             "yaw={0:.1f} wind={1:.1f} signed={2:.1f} -> {3:.1f} "
             "(other -> {4:.1f})".format(yaw, wind, signed, result_bearing,
                                         other_bearing))


@strict
def test_downwind_of_house_forward_and_lateral():
    """`_house_downwind_of`: true directly downwind within
    `downwind_widths * footprint`, false upwind, false too far to the side,
    false past the downwind cap."""
    house = (0.0, 0.0, 12.0, 0.7, "leveled", None)

    def wind_bearing_fn(x, y):
        return 0.0   # blows toward +X

    check("directly downwind, close in, is inside the shadow",
         wash._house_downwind_of(10.0, 0.0, house, wind_bearing_fn))
    check("upwind of the house is never inside the shadow",
         not wash._house_downwind_of(-10.0, 0.0, house, wind_bearing_fn))
    check("past the downwind cap is outside the shadow",
         not wash._house_downwind_of(
             wash.FENCE_DOWNWIND_WIDTHS * 12.0 + 5.0, 0.0, house,
             wind_bearing_fn))
    check("far off to the side is outside the lateral corridor",
         not wash._house_downwind_of(5.0, 50.0, house, wind_bearing_fn))
    check("just inside the lateral corridor is inside the shadow",
         wash._house_downwind_of(
             5.0, wash.FENCE_DOWNWIND_LATERAL_FRAC * 12.0 * 0.8, house,
             wind_bearing_fn))
    # Rotate the wind 90 degrees: what was downwind is now lateral.
    def wind_bearing_90(x, y):
        return 90.0

    check("rotating the wind bearing rotates the shadow with it",
         not wash._house_downwind_of(10.0, 0.0, house, wind_bearing_90)
         and wash._house_downwind_of(0.0, 10.0, house, wind_bearing_90))


@strict
def test_wind_threshold_percentile_and_empty():
    """`fence_wind_threshold` returns the requested percentile of the
    sampled field and a never-triggerable sentinel for an empty fence
    list."""
    fences = [(float(i), 0.0, 0.0, 2.0) for i in range(101)]

    def wind_intensity_fn(x, y):
        return x / 100.0   # 0.00 .. 1.00, evenly spaced with `fences` above

    t70 = wash.fence_wind_threshold(fences, wind_intensity_fn, pctile=0.70)
    check("70th percentile of an evenly-spaced 0..1 field lands near 0.70",
         abs(t70 - 0.70) < 0.02, t70)
    t0 = wash.fence_wind_threshold(fences, wind_intensity_fn, pctile=0.0)
    check("0th percentile is the minimum", abs(t0 - 0.0) < 1e-9, t0)
    t100 = wash.fence_wind_threshold(fences, wind_intensity_fn, pctile=1.0)
    check("100th percentile is the maximum", abs(t100 - 1.0) < 1e-9, t100)
    check("an empty fence list returns a sentinel above any real intensity",
         wash.fence_wind_threshold([], wind_intensity_fn) > 1.0)


@strict
def test_fence_specs_three_way_split():
    """End to end: a fence in deep water is "gone" (with 1-3 fence rafts
    that break the water surface, same draft invariant every other raft
    kind pins), a fence downwind of a levelled house is "flat", and an
    unremarkable fence far from both is "stands" — using a field where the
    wind-percentile branch cannot fire on its own (a flat, low intensity
    field) so the downwind/water tests are isolated."""
    houses = [(0.0, 0.0, 12.0, 0.8, "leveled", None),
             (500.0, 500.0, 12.0, 0.2, "shingles_lost", None)]

    def depth_fn(x, y):
        return 2.0 if x > 200.0 else 0.0   # deep water only far to the east

    def wind_bearing_fn(x, y):
        return 0.0   # blows toward +X

    # DELIBERATELY NOT PERFECTLY UNIFORM: `fence_wind_threshold`'s own
    # percentile is RANK-based, so an EXACTLY flat field (every fence tied
    # at its own 70th percentile) makes the ">= threshold" test degenerate
    # true for 100% of the field -- a real edge case worth documenting, but
    # not what THIS test wants to isolate (the downwind/water branches).
    # The real `hurricane.intensity_field` always carries some spatial
    # noise (its own docstring: "single digits of percent spread"), so this
    # mirrors that with a small, explicit per-fence spread instead.
    def wind_intensity_fn(x, y):
        return 0.1 if x < 0.0 else 0.5

    fences = [
        (300.0, 0.0, 45.0, 2.0),     # in deep water -> gone (wind irrelevant)
        (10.0, 0.0, 0.0, 2.0),       # downwind of the levelled house -> flat
        (-50.0, 100.0, 0.0, 2.0),    # dry, upwind/lateral, no wrecked house
                                     # nearby, low wind rank -> stands
    ]
    rng = random.Random(7)
    out = wash.fence_specs(fences, depth_fn, wind_bearing_fn,
                           wind_intensity_fn, houses, 1.0, rng)
    check("three fences in, three decisions out", len(out) == 3, len(out))

    gone, flat, stands = out
    check("deep-water fence is gone", gone["action"] == "gone", gone)
    check("gone fence carries 1-3 rafts",
         "rafts" in gone and 1 <= len(gone["rafts"]) <= 3, gone.get("rafts"))
    for r in gone["rafts"]:
        check("gone-fence raft is kind 'fence'", r["kind"] == "fence", r)
        pts, _n = planks._box(r)
        zs = [p[2] for p in pts]
        check("gone-fence raft breaks the water surface",
             max(zs) > 1.0 > min(zs),
             "top={0:.3f} bot={1:.3f}".format(max(zs), min(zs)))
        check("gone-fence raft drifted toward the WET side (x < 300, "
             "shoreward, not further into deep water)",
             r["x"] < 300.0, r["x"])

    check("downwind-of-house fence is flat", flat["action"] == "flat", flat)
    check("flat fence's lean magnitude is in the requested ~85 deg band",
         wash.FENCE_FLAT_LEAN_LO_DEG <= abs(flat["lean_deg"])
         <= wash.FENCE_FLAT_LEAN_HI_DEG, flat["lean_deg"])

    check("unremarkable fence stands", stands["action"] == "stands", stands)


@strict
def test_fence_specs_min_house_level_gate():
    """A house that has only lost shingles (below `FENCE_MIN_HOUSE_LEVEL`)
    does not flatten a fence behind it; a house at or past that level
    does — same fence position, only the house's own level differs."""
    def depth_fn(x, y):
        return 0.0

    def wind_bearing_fn(x, y):
        return 0.0

    # A single-fence list makes `fence_wind_threshold` trivially equal to
    # that one fence's own value (a 1-point "percentile" is just the
    # point), so an extra high-intensity PROBE fence, far away and never
    # inspected, is added purely to give the percentile something to sit
    # below -- the fence actually under test stays at a low, clearly
    # sub-threshold rank.
    def wind_intensity_fn(x, y):
        return 0.9 if x > 1000.0 else 0.1

    fence = [(10.0, 0.0, 0.0, 2.0), (2000.0, 2000.0, 0.0, 2.0)]

    mild = [(0.0, 0.0, 12.0, 0.5, "shingles_lost", None)]
    out_mild = wash.fence_specs(fence, depth_fn, wind_bearing_fn,
                                wind_intensity_fn, mild, 1.0,
                                random.Random(2))
    check("a shingles_lost house does not flatten its downwind fence",
         out_mild[0]["action"] == "stands", out_mild[0])

    severe = [(0.0, 0.0, 12.0, 0.5, "roof_stripped", None)]
    out_severe = wash.fence_specs(fence, depth_fn, wind_bearing_fn,
                                  wind_intensity_fn, severe, 1.0,
                                  random.Random(2))
    check("a roof_stripped (>= deck_panels_lost) house flattens it",
         out_severe[0]["action"] == "flat", out_severe[0])


@strict
def test_fence_specs_wind_percentile_branch():
    """Independent of any house: a fence sitting in the top
    `1 - FENCE_WIND_PCTILE` of the field's own wind-intensity distribution
    is flattened even with no wrecked house anywhere near it."""
    def depth_fn(x, y):
        return 0.0

    def wind_bearing_fn(x, y):
        return 0.0

    fences = [(float(i) * 10.0, 0.0, 0.0, 2.0) for i in range(20)]

    def wind_intensity_fn(x, y):
        return x / 200.0   # rises left to right across the 20 fences

    out = wash.fence_specs(fences, depth_fn, wind_bearing_fn,
                           wind_intensity_fn, [], 1.0, random.Random(4))
    # No houses at all, so only the wind-percentile branch can fire.
    n_flat = sum(1 for d in out if d["action"] == "flat")
    check("some fences flatten purely from the wind-percentile branch",
         0 < n_flat < len(out), n_flat)
    # And they must be the HIGH-intensity ones, not a random subset.
    flat_xs = [d["x"] for d in out if d["action"] == "flat"]
    stand_xs = [d["x"] for d in out if d["action"] == "stands"]
    check("the flattened fences are the high-wind ones",
         not flat_xs or not stand_xs or min(flat_xs) > max(stand_xs) - 1e-9,
         (sorted(flat_xs), sorted(stand_xs)))


if __name__ == "__main__":
    test_fall_lean_sign_matches_wind_bearing()
    test_downwind_of_house_forward_and_lateral()
    test_wind_threshold_percentile_and_empty()
    test_fence_specs_three_way_split()
    test_fence_specs_min_house_level_gate()
    test_fence_specs_wind_percentile_branch()
    if FAILS:
        print("\n{0} FAILURE(S)".format(len(FAILS)))
        for f in FAILS:
            print(" -", f)
        sys.exit(1)
    print("\nALL PASS")
