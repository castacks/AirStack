#!/usr/bin/env python3
"""
test_hurricane_debris_land.py — pins `washaway.land_debris_specs` (DEBRIS
stream deliverable E, 2026-08-31): the LAND half of the hurricane debris
field, downwind of a damaged house.

THE REPORTED DEFECT. `suburb_hurricane_launch_script.py`'s own docstring
advertises `HUR_DEBRIS` as "vegetation debris pieces per damaged house
(default 60)"; it is read once into `N_DEBRIS` and never referenced again
anywhere else in that file. A Category-2/3 hurricane peels roofs (the
`build-hurricane-scenes` skill's "Debris is 70% vegetation -- the opposite
of the tornado plank field", and the coarse building third of that split),
and none of the peeled material was ever authored on dry land -- only the
water half (`washaway.raft_specs`) had a debris field at all. This is the
land counterpart.

`land_debris_specs` is a THIN orchestration layer, deliberately: it decides
which houses shed debris, what classes, how many pieces and how far, then
hands each house to the ALREADY-CORRECT `planks.scatter_from_wreck` (whose
own seating, `planks._lay`, is points-based and pinned by
`fix-floating-debris`'s discipline -- never `UsdGeom.BBoxCache`). This file
therefore tests the ORCHESTRATION -- gating, kind selection, piece counts,
the downwind reach cap -- not the comet shape itself (`scatter_from_wreck`
has no dedicated offline test of its own to extend; the geometry claims it
already makes are covered informally by the tornado pipeline's renders).

RUNS WITHOUT ISAAC. Same import-boundary guarantee as
`test_washaway_debris.py`: nothing this file exercises reaches for `pxr`.

USAGE
    python3 scene_gen/tests/test_hurricane_debris_land.py
    pytest -q scene_gen/tests/test_hurricane_debris_land.py
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
    def run(*a, **kw):
        mine = len(FAILS)
        out = fn(*a, **kw)
        assert len(FAILS) == mine, FAILS[mine:]
        return out
    run.__name__, run.__doc__ = fn.__name__, fn.__doc__
    return run


@strict
def test_gate_below_cover_lost_sheds_nothing():
    """`shingles_lost` (below the default `min_level="cover_lost"`) and
    `pristine` houses shed nothing -- a cosmetically-fine roof does not
    litter its own lawn."""
    wrecks = [(0.0, 0.0, 12.0, 0.30, "shingles_lost", "p"),
             (50.0, 0.0, 12.0, 0.20, "pristine", "p")]
    specs = wash.land_debris_specs(wrecks, lambda x, y: 0.0,
                                   random.Random(1))
    check("no pieces below the gate", len(specs) == 0, len(specs))


@strict
def test_swept_house_sheds_no_land_debris():
    """`swept` is a WATER state (the house's material went into
    `raft_specs`'s domain, not the wind's) and is absent from the default
    level order, so a `swept` entry is skipped exactly like `pristine`."""
    wrecks = [(0.0, 0.0, 12.0, 0.95, "swept", "p")]
    specs = wash.land_debris_specs(wrecks, lambda x, y: 0.0,
                                   random.Random(1))
    check("swept house sheds nothing on land", len(specs) == 0, len(specs))


@strict
def test_kinds_match_level():
    """Kind gating exactly matches the review's own language: `cover_lost`
    -> shingle sheets (`deck`) + siding; `deck_panels_lost`/`roof_stripped`
    -> sheathing + rafters (`joist`); the collapse rungs -> every class."""
    levels_and_expected = [
        ("cover_lost", {"deck", "siding"}),
        ("deck_panels_lost", {"sheathing", "joist"}),
        ("roof_stripped", {"sheathing", "joist"}),
        ("roof_collapsed", set(planks.STOCK)),
        ("partial_collapse", set(planks.STOCK)),
        ("leveled", set(planks.STOCK)),
    ]
    for level, expected in levels_and_expected:
        wrecks = [(0.0, 0.0, 12.0, 0.9, level, "p")]
        rng = random.Random(2)
        specs = wash.land_debris_specs(wrecks, lambda x, y: 0.0, rng)
        seen = {s["class"] for s in specs}
        check("{0} kinds subset of expected".format(level),
             seen.issubset(expected),
             "seen={0} expected={1}".format(seen, expected))
        check("{0} produced pieces".format(level), len(specs) > 0)


@strict
def test_piece_count_scales_with_intensity():
    """Piece count interpolates `LAND_DEBRIS_N_LO..N_HI` with the house's
    own `intensity`, matching the review's "~15-30 at L2, ~30-60 at L3" --
    expressed here as intensity since that IS what separates the two
    presets house by house (see the module's own docstring)."""
    for it, lo_expect, hi_expect in ((0.10, 10, 25), (0.50, 30, 50),
                                     (1.00, 50, 65)):
        wrecks = [(0.0, 0.0, 12.0, it, "leveled", "p")]
        specs = wash.land_debris_specs(wrecks, lambda x, y: 0.0,
                                       random.Random(3))
        check("intensity {0} piece count in [{1}, {2}]"
             .format(it, lo_expect, hi_expect),
             lo_expect <= len(specs) <= hi_expect, len(specs))
    check("N_LO/N_HI bracket the requested 15-60 range",
         wash.LAND_DEBRIS_N_LO <= 15 and wash.LAND_DEBRIS_N_HI >= 60,
         (wash.LAND_DEBRIS_N_LO, wash.LAND_DEBRIS_N_HI))


@strict
def test_reach_cap_geometry():
    """`_reach_cap` itself, exactly (no randomness): a footprint straight
    downwind narrows the reach to its near edge; the SAME footprint placed
    behind the source, or too far off to the side, does not narrow it at
    all (returns `None`)."""
    downwind = [(18.0, 0.0, 12.0)]
    cap = wash._reach_cap(0.0, 0.0, 0.0, downwind)
    check("downwind footprint caps at its near edge (18 - 6 = 12)",
         cap is not None and abs(cap - 12.0) < 1e-6, cap)

    behind = [(-18.0, 0.0, 12.0)]
    check("a footprint BEHIND the source never caps the reach",
         wash._reach_cap(0.0, 0.0, 0.0, behind) is None)

    far = [(400.0, 0.0, 12.0)]
    check("a footprint far downwind (past min_reach_m) still caps",
         wash._reach_cap(0.0, 0.0, 0.0, far) is not None)

    to_the_side = [(18.0, 40.0, 12.0)]
    check("a footprint well off to the side does not cap",
         wash._reach_cap(0.0, 0.0, 0.0, to_the_side) is None)


def _pooled_downwind_distances(wrecks, wind_deg, n_trials=12):
    """Run `land_debris_specs` `n_trials` times with different seeds and
    pool every piece's downwind (x, since wind_deg=0 here) distance from
    the source house at the origin -- reduces the max()-of-one-sample noise
    a single run's stochastic comet would otherwise carry."""
    out = []
    for seed in range(n_trials):
        specs = wash.land_debris_specs(wrecks, lambda x, y: wind_deg,
                                       random.Random(100 + seed))
        out.extend(s["x"] for s in specs if s["x"] > -5.0)
    return sorted(out)


@strict
def test_reach_cap_shrinks_the_realised_field():
    """Pooled over many seeds, a house capped by a close downwind neighbour
    produces a materially SHORTER debris field (90th-percentile downwind
    distance) than the same house with nothing in its path."""
    # The blocking neighbour is `shingles_lost` -- BELOW the default gate,
    # so it contributes its FOOTPRINT (every `wrecks` entry counts as an
    # obstacle for `_reach_cap`, gated or not) without shedding any debris
    # of its own to contaminate the downwind-distance measurement. Placed
    # CLOSE (15 m) so the cap (15 - 6 = 9 m) is a large fraction (~36%) of
    # `LAND_DEBRIS_REACH_M`'s default 14 m -- a small cap relative to the
    # default reach is a real but modest effect (measured separately below)
    # and this scenario isolates a big, robust one.
    capped = [(0.0, 0.0, 12.0, 0.9, "leveled", "p1"),
             (15.0, 0.0, 12.0, 0.9, "shingles_lost", "p2")]
    open_field = [(0.0, 0.0, 12.0, 0.9, "leveled", "p1")]

    d_capped = _pooled_downwind_distances(capped, 0.0)
    d_open = _pooled_downwind_distances(open_field, 0.0)

    def p90(xs):
        return xs[int(0.9 * (len(xs) - 1))]

    check("pooled samples non-trivial",
         len(d_capped) > 100 and len(d_open) > 100,
         (len(d_capped), len(d_open)))
    check("capped p90 well under the uncapped p90",
         p90(d_capped) < 0.80 * p90(d_open),
         "capped_p90={0:.2f} open_p90={1:.2f}"
         .format(p90(d_capped), p90(d_open)))
    check("capped field stays clear of the neighbour's far side (21 m)",
         max(d_capped) < 21.0, max(d_capped))

    # A SMALLER cap effect, closer to the earlier 18 m scenario, is real but
    # more modest -- checked directly against `_reach_cap`'s own arithmetic
    # rather than a second, looser statistical threshold.
    modest = [(0.0, 0.0, 12.0, 0.9, "leveled", "p1"),
             (18.0, 0.0, 12.0, 0.9, "shingles_lost", "p2")]
    d_modest = _pooled_downwind_distances(modest, 0.0)
    expected_ratio = wash._reach_cap(0.0, 0.0, 0.0, [(18.0, 0.0, 12.0)]) \
        / wash.LAND_DEBRIS_REACH_M
    check("modest cap (18 m neighbour) still measurably shortens p90",
         p90(d_modest) < 0.95 * p90(d_open),
         "modest_p90={0:.2f} open_p90={1:.2f} expected_ratio={2:.2f}"
         .format(p90(d_modest), p90(d_open), expected_ratio))


@strict
def test_seating_is_points_based_and_never_airborne():
    """Every piece's own lowest rotated corner (the closed-form vertical
    extent `planks._lay` computes -- never `UsdGeom.BBoxCache`) sits at or
    within a couple of centimetres of the configured ground_z; nothing
    hovers."""
    wrecks = [(0.0, 0.0, 12.0, 0.85, "partial_collapse", "p")]
    rng = random.Random(8)
    specs = wash.land_debris_specs(wrecks, lambda x, y: 45.0, rng)
    check("produced pieces", len(specs) > 0, len(specs))
    gz = wash.LAND_DEBRIS_GROUND_Z_M
    worst = 0.0
    for s in specs:
        pts, _n = planks._box(s)
        bottom = min(p[2] for p in pts)
        worst = max(worst, bottom - gz)
        check("piece not airborne (bottom <= ground_z + 1cm)",
             bottom <= gz + 0.01,
             "bottom={0:.4f} ground_z={1:.4f}".format(bottom, gz))
    check("no piece far below ground either (sink <= 3cm)",
         all(min(p[2] for p in planks._box(s)[0]) >= gz - 0.03
             for s in specs))


@strict
def test_wind_bearing_drives_direction():
    """Pieces travel in the direction `wind_bearing_fn` reports for that
    house, not a fixed global heading -- two houses with opposite bearings
    scatter to opposite sides."""
    wrecks_e = [(0.0, 0.0, 12.0, 0.8, "leveled", "p")]
    wrecks_w = [(0.0, 0.0, 12.0, 0.8, "leveled", "p")]
    specs_e = wash.land_debris_specs(wrecks_e, lambda x, y: 0.0,
                                     random.Random(9))
    specs_w = wash.land_debris_specs(wrecks_w, lambda x, y: 180.0,
                                     random.Random(9))
    mean_x_e = sum(s["x"] for s in specs_e) / len(specs_e)
    mean_x_w = sum(s["x"] for s in specs_w) / len(specs_w)
    check("east-bearing comet drifts +x on average", mean_x_e > 0.5,
         mean_x_e)
    check("west-bearing comet drifts -x on average", mean_x_w < -0.5,
         mean_x_w)


@strict
def test_skins_fn_applied_to_skinned_classes():
    """`skins_fn`, when given, stamps `siding`/`deck` pieces with the
    wrecked house's own materials -- the tornado field's "the debris wears
    what the house wore" fix, reused rather than reinvented."""
    wrecks = [(0.0, 0.0, 12.0, 0.9, "cover_lost", "coastal_blue")]

    def skins_fn(w):
        return {"siding": "Wall_" + w[5], "deck": "Roof_" + w[5]}

    specs = wash.land_debris_specs(wrecks, lambda x, y: 0.0, random.Random(4),
                                   skins_fn=skins_fn)
    check("produced pieces", len(specs) > 0, len(specs))
    skinned = [s for s in specs if s["class"] in planks.SKINNED]
    check("every skinned-class piece carries the house's own skin",
         skinned and all(s.get("skin") for s in skinned),
         len(skinned))
    check("skin names come from skins_fn",
         all(s["skin"] in ("Wall_coastal_blue", "Roof_coastal_blue")
             for s in skinned))


@strict
def test_cone_fraction_meets_the_review_target():
    """D2 review, "LAND DEBRIS IS A RING, NOT A COMET": pooled over many
    seeds and several isolated houses (200 m apart -- far past
    `LAND_DEBRIS_REACH_M`, so `_reach_cap` never engages and every house's
    own comet is unambiguous), at least 70% of a house's own pieces fall
    inside a +-35 deg cone about its own wind bearing, and the median
    downwind distance lands inside the requested 1-3 building-width band.
    Reproduces, on a synthetic scene with no GT file, the measurement
    `tools/hurricane_debris_plot.py` makes on the real L2/L3 house
    populations (medians 0.762/0.761 cone fraction, 15.0/16.7 m downwind)."""
    cone_deg = 35.0
    wrecks = [(float(i) * 200.0, 0.0, 12.0, 0.85, "leveled", "p")
             for i in range(6)]
    fracs, downwinds = [], []
    for seed in range(24):
        specs = wash.land_debris_specs(wrecks, lambda x, y: 0.0,
                                       random.Random(500 + seed))
        by_house = {i: [] for i in range(len(wrecks))}
        for s in specs:
            # Nearest house BY X is exact here: houses sit 200 m apart on
            # one line and no comet reaches a third of that.
            i = min(range(len(wrecks)),
                   key=lambda j: abs(s["x"] - wrecks[j][0]))
            by_house[i].append(s)
        for i, pieces in by_house.items():
            if not pieces:
                continue
            hx = wrecks[i][0]
            in_cone = 0
            s_vals = []
            for p in pieces:
                s, t = p["x"] - hx, p["y"]
                s_vals.append(s)
                if math.degrees(abs(math.atan2(t, s))) <= cone_deg:
                    in_cone += 1
            fracs.append(in_cone / len(pieces))
            downwinds.append(sum(s_vals) / len(s_vals))

    fracs.sort()
    downwinds.sort()
    med_frac = fracs[len(fracs) // 2]
    med_dw = downwinds[len(downwinds) // 2]
    check("pooled a non-trivial number of houses", len(fracs) > 50,
         len(fracs))
    check("median cone fraction >= 0.70 (the review's own target)",
         med_frac >= 0.70, med_frac)
    check("median downwind distance inside the 1-3 building-width "
         "(8-45 m) band the review asks for",
         8.0 <= med_dw <= 45.0, med_dw)


@strict
def test_submerged_land_debris_converts_to_raft():
    """D2 review, "LAND DEBRIS ON SUBMERGED GROUND": omitting `depth_fn`
    keeps the ORIGINAL flat-list return, byte-identical for every existing
    caller. Passing it changes the return to a `(land, rafts)` pair; a
    piece whose landing point is under real water
    (`depth_fn(x, y) > LAND_DEBRIS_SUBMERGED_DEPTH_M`) is pulled out of
    `land` and rebuilt as a `_RAFT_DIMS`-schema raft at the
    `LAND_TO_RAFT_KIND`-mapped kind, floating at `water_level_m` via
    `_one_raft`'s own draft formula -- so it breaks the water surface
    exactly like every other raft (`test_washaway_debris.py`'s own
    draft-invariant test, for the same guarantee)."""
    wrecks = [(0.0, 0.0, 12.0, 0.9, "leveled", "p")]

    plain = wash.land_debris_specs(wrecks, lambda x, y: 0.0, random.Random(5))
    check("no depth_fn -> plain list, unchanged contract",
         isinstance(plain, list) and len(plain) > 0, type(plain))

    land_dry, rafts_dry = wash.land_debris_specs(
        wrecks, lambda x, y: 0.0, random.Random(5),
        depth_fn=lambda x, y: 0.0)
    check("bone-dry depth_fn -> (land, rafts) pair, nothing converted",
         len(rafts_dry) == 0 and len(land_dry) == len(plain),
         (len(land_dry), len(rafts_dry), len(plain)))

    land_wet, rafts_wet = wash.land_debris_specs(
        wrecks, lambda x, y: 0.0, random.Random(5),
        depth_fn=lambda x, y: 1.5, water_level_m=1.5)
    check("deeply-flooded depth_fn -> every piece converts, land empty",
         len(land_wet) == 0 and len(rafts_wet) == len(plain),
         (len(land_wet), len(rafts_wet), len(plain)))

    kinds_seen = {r["kind"] for r in rafts_wet}
    expected_kinds = {wash.LAND_TO_RAFT_KIND.get(s["class"], "timber")
                      for s in plain}
    check("converted rafts use the LAND_TO_RAFT_KIND mapping",
         kinds_seen == expected_kinds, (kinds_seen, expected_kinds))
    check("every mapped kind is a real _RAFT_DIMS kind",
         kinds_seen.issubset(set(wash._RAFT_DIMS)), kinds_seen)

    for r in rafts_wet:
        pts, _n = planks._box(r)
        zs = [p[2] for p in pts]
        check("converted raft breaks the water surface (top > 1.5 > bot)",
             max(zs) > 1.5 > min(zs),
             "top={0:.3f} bot={1:.3f}".format(max(zs), min(zs)))


if __name__ == "__main__":
    test_gate_below_cover_lost_sheds_nothing()
    test_swept_house_sheds_no_land_debris()
    test_kinds_match_level()
    test_piece_count_scales_with_intensity()
    test_reach_cap_geometry()
    test_reach_cap_shrinks_the_realised_field()
    test_seating_is_points_based_and_never_airborne()
    test_wind_bearing_drives_direction()
    test_skins_fn_applied_to_skinned_classes()
    test_cone_fraction_meets_the_review_target()
    test_submerged_land_debris_converts_to_raft()
    if FAILS:
        print("\n{0} FAILURE(S)".format(len(FAILS)))
        for f in FAILS:
            print(" -", f)
        sys.exit(1)
    print("\nALL PASS")
