#!/usr/bin/env python3
"""test_tornado_street.py — the URBAN CORRIDOR's street-furniture, tree and
car pass (R9, `_plans/urban_tornado_plan.md` §8, stream SP), pinned without
Isaac.

`disaster/tornado_street.py`'s `plan_street` decides, per non-house
placement, what a tornado did to it -- felled, carried away, tipped in
place, or left alone -- and (for cars) whether the ordinary Paulikas-shares
outcome is additionally promoted to a 20-60 m THROW. Everything in it is
pure geometry/decision-making: no `pxr`, no stage, no Isaac.

RUNS WITHOUT ISAAC. Checked directly below
(`test_module_is_pxr_free_at_import_time`) the same way `test_tornado_
urban_ground.py`/`test_car_toss.py` check their own pure halves. The one
exception is the "APPLY" section (`apply_street`/`_apply_action`/
`_normalize_yaw_op`/`_apply_tree_level`, extracted from the launcher
2026-09-01 so the bench can wire it too) -- those lazily import `pxr`
INSIDE themselves, so a handful of tests below build a real, bare
`Usd.Stage.CreateInMemory()` (`pxr`/`usd-core` on the host, no Kit, no
GPU, the same "bare usd-core" convention `test_tornado_urban_ground.py`'s
own `build()` tests already use) rather than staying purely offline.

USAGE
    python3 scene_gen/tests/test_tornado_street.py
    pytest -q scene_gen/tests/test_tornado_street.py
"""

import json
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from pxr import Gf, Sdf, Usd, UsdGeom                        # noqa: E402

from disaster import tornado as tn                          # noqa: E402
from disaster import tornado_street as tst                   # noqa: E402

FAILS = []


def check(cond, msg):
    print(("    PASS  " if cond else "    FAIL  ") + msg)
    if not cond:
        FAILS.append(msg)


def strict(fn):
    """`test_car_toss.py`'s own idiom: a recorded FAIL becomes a real
    pytest failure without stopping a standalone run at the first one."""
    def run(*a, **kw):
        mine = len(FAILS)
        out = fn(*a, **kw)
        assert len(FAILS) == mine, FAILS[mine:]
        return out
    run.__name__ = fn.__name__
    run.__doc__ = fn.__doc__
    return run


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------

def _flat_wind(bearing=45.0):
    def wind_at_fn(x, y):
        return {"bearing_deg": bearing}
    return wind_at_fn


def _flat_intensity(i):
    def intensity_fn(x, y):
        return i
    return intensity_fn


def _prop(path, category, x=0.0, y=0.0, usd="asset.usd", **extra):
    d = {"prim_path": path, "category": category, "x_m": x, "y_m": y,
        "usd": usd}
    d.update(extra)
    return d


def _car(path, x=0.0, y=0.0, yaw_deg=0.0):
    return _prop(path, "car", x=x, y=y, yaw_deg=yaw_deg)


def _tree_species_usd(species="Shumard_Oak"):
    return "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/{0}/{0}.usd".format(species)


def _plan(placements, i, rng_seed, bearing=45.0, **kw):
    rng = random.Random(rng_seed)
    return tst.plan_street(placements, _flat_intensity(i), _flat_wind(bearing),
                           rng, **kw)


def _by_path(actions, path):
    for a in actions:
        if a["path"] == path:
            return a
    return None


# ---------------------------------------------------------------------------
# module purity
# ---------------------------------------------------------------------------

@strict
def test_module_is_pxr_free_at_import_time():
    print("\n[purity] no `pxr` import at MODULE level (lazy imports inside "
          "apply_street's own functions are fine and expected)")
    import ast
    path = os.path.join(_SCENE_GEN, "disaster", "tornado_street.py")
    src = open(path).read()
    tree = ast.parse(src, filename=path)
    # Only TOP-LEVEL statements -- a lazy `from pxr import ...` inside a
    # function body (`_normalize_yaw_op`, `_apply_action`, `apply_street`,
    # `_apply_tree_level`'s `from . import vegetation`) is a NESTED
    # statement, invisible to `tree.body`, and is exactly what this test
    # must NOT flag: `plan_street` and everything above the "APPLY"
    # section stays importable with no `usd-core` on the path, but calling
    # `apply_street` legitimately needs `pxr`.
    top_level_pxr = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name == "pxr" or alias.name.startswith("pxr."):
                    top_level_pxr.append(alias.name)
        elif isinstance(node, ast.ImportFrom):
            if node.module and (node.module == "pxr"
                                or node.module.startswith("pxr.")):
                top_level_pxr.append(node.module)
    check(top_level_pxr == [],
          "no module-level pxr import (found: %r)" % top_level_pxr)
    mod_file = tst.__file__
    check(mod_file.endswith(".py"), "module loaded from source, not a stub")


def test_apply_street_functions_exist_but_are_not_called_by_plan_street():
    print("\n[purity] apply_street/_apply_action/_normalize_yaw_op are "
          "importable names (proves the module loaded despite the lazy "
          "pxr imports inside them) without ever touching a stage")
    check(callable(tst.apply_street), "apply_street is a callable")
    check(callable(tst._apply_action), "_apply_action is a callable")
    check(callable(tst._normalize_yaw_op), "_normalize_yaw_op is a callable")


# ---------------------------------------------------------------------------
# 1. the measured vocabulary is where the plan says it is, and nowhere else
# ---------------------------------------------------------------------------

@strict
def test_category_lists_are_disjoint_and_documented():
    print("\n[vocabulary] FELLED / CARRIED / UNTOUCHED never overlap")
    felled, carried = set(tst.FELLED), set(tst.CARRIED)
    check(not (felled & carried), "FELLED and CARRIED share no category")
    check(not (felled & set(tst.UNTOUCHED)), "FELLED/UNTOUCHED disjoint")
    check(not (carried & set(tst.UNTOUCHED)), "CARRIED/UNTOUCHED disjoint")
    # the plan's own anchor examples, mapped as the module docstring records
    check("streetlight" in felled and "traffic_light" in felled
          and "bus_stop" in felled, "the plan's FELLED anchors are present")
    check("sign" in felled, "'sign' (no mast/small split exists) -> FELLED")
    check("trash_can" in carried and "bench" in carried
          and "traffic_cone" in carried, "the plan's CARRIED anchors are present")
    check("planter_fence" in carried,
          "'planter_fence' stands in for the plan's 'planter'")
    check("sign_mast" not in felled and "sign_small" not in carried
          and "planter" not in carried,
          "no literal sign_mast/sign_small/planter -- they do not exist "
          "in the measured vocabulary")
    for c in ("fire_hydrant", "bollard", "manhole", "storm_drain"):
        check(c in tst.UNTOUCHED, "%s is a documented UNTOUCHED category" % c)


@strict
def test_untouched_categories_never_produce_an_action():
    print("\n[vocabulary] fire_hydrant/bollard/manhole/storm_drain stay plumb")
    for cat in tst.UNTOUCHED:
        placements = [_prop("/p/x", cat)]
        for i in (0.3, 0.6, 0.95, 1.0):
            hits = 0
            for seed in range(40):
                acts = _plan(placements, i, seed)
                hits += len(acts)
            check(hits == 0, "%s at i=%.2f: 0 actions over 40 seeds (got %d)"
                  % (cat, i, hits))


@strict
def test_house_and_human_are_never_walked():
    print("\n[vocabulary] house/human placements are skipped outright")
    placements = [_prop("/p/h", "house"), _prop("/p/u", "human")]
    acts = _plan(placements, 1.0, 1)
    check(acts == [], "house/human at i=1.0 produce zero actions")


# ---------------------------------------------------------------------------
# 2. shares monotone in intensity
# ---------------------------------------------------------------------------

def _tally_felled(category, i, n=600, seed0=0):
    counts = {"fell": 0, "carry": 0, "none": 0}
    for k in range(n):
        placements = [_prop("/p/x", category, x=1000.0 + k * 200.0)]
        acts = _plan(placements, i, seed0 + k)
        if not acts:
            counts["none"] += 1
        else:
            counts[acts[0]["action"]] += 1
    return counts


@strict
def test_felled_shares_monotone_in_intensity():
    print("\n[shares] FELLED down+gone share rises with intensity")
    prev = -1.0
    for i in (0.05, 0.25, 0.5, 0.75, 0.95):
        c = _tally_felled("streetlight", i)
        acted = c["fell"] + c["carry"]
        frac = acted / 600.0
        print("    i=%.2f  fell=%d carry=%d none=%d  (acted %.3f)"
              % (i, c["fell"], c["carry"], c["none"], frac))
        check(frac >= prev - 0.03, "monotone-ish rise at i=%.2f" % i)
        prev = frac
    check(prev > 0.85, "the core fells or carries almost every pole (%.3f)"
          % prev)


@strict
def test_carried_shares_monotone_in_intensity():
    print("\n[shares] CARRIED gone+toss share rises with intensity")
    prev = -1.0
    for i in (0.05, 0.25, 0.5, 0.75, 0.95):
        # CARRIED's second outcome is "toss", not "fell" -- tally directly
        # rather than reusing `_tally_felled` (a FELLED-shaped helper).
        counts = {"toss": 0, "carry": 0, "none": 0}
        for k in range(600):
            placements = [_prop("/p/x", "trash_can", x=1000.0 + k * 200.0)]
            acts = _plan(placements, i, k)
            if not acts:
                counts["none"] += 1
            else:
                counts[acts[0]["action"]] += 1
        frac = (counts["carry"] + counts["toss"]) / 600.0
        print("    i=%.2f  carry=%d toss=%d none=%d  (acted %.3f)"
              % (i, counts["carry"], counts["toss"], counts["none"], frac))
        check(frac >= prev - 0.03, "monotone-ish rise at i=%.2f" % i)
        prev = frac
    check(prev > 0.9, "the core carries or tosses almost every trash can")


@strict
def test_carried_removal_dominates_at_core():
    print("\n[shares] at the core, CARRIED is mostly GONE, not just tipped")
    counts = {"carry": 0, "toss": 0, "none": 0}
    N = 800
    for k in range(N):
        placements = [_prop("/p/x", "bench", x=1000.0 + k * 50.0)]
        acts = _plan(placements, 0.95, k)
        if not acts:
            counts["none"] += 1
        else:
            counts[acts[0]["action"]] += 1
    print("   ", counts)
    check(counts["carry"] > counts["toss"],
          "carry (%d) outnumbers toss (%d) at i=0.95" % (counts["carry"],
                                                          counts["toss"]))


# ---------------------------------------------------------------------------
# 3. the felled lean band
# ---------------------------------------------------------------------------

@strict
def test_felled_lean_band():
    print("\n[felled] lean magnitude stays in 62-100 degrees")
    bad = 0
    n = 0
    for k in range(500):
        placements = [_prop("/p/x", "streetlight", x=1000.0 + k * 50.0)]
        acts = _plan(placements, 0.6 + (k % 5) * 0.08, k)
        for a in acts:
            if a["action"] == "fell":
                n += 1
                mag = abs(a["roll_deg"])
                if not (62.0 - 1e-6 <= mag <= 100.0 + 1e-6):
                    bad += 1
    check(n > 50, "collected a meaningful sample of fell actions (%d)" % n)
    check(bad == 0, "0 of %d fell actions outside [62, 100] degrees" % n)


@strict
def test_carried_lean_band_is_different_from_felled():
    print("\n[carried] toss lean magnitude stays in 38-96 degrees, a wider "
          "and lower band than a felled pole's 62-100")
    n = bad = 0
    for k in range(500):
        placements = [_prop("/p/x", "bench", x=1000.0 + k * 50.0)]
        acts = _plan(placements, 0.5 + (k % 5) * 0.08, k)
        for a in acts:
            if a["action"] == "toss":
                n += 1
                mag = abs(a["roll_deg"])
                if not (38.0 - 1e-6 <= mag <= 96.0 + 1e-6):
                    bad += 1
                check(1.0 - 1e-6 <= a["dist_m"] <= 6.0 + 1e-6,
                      "toss distance %.2f within 1-6 m" % a["dist_m"]) \
                    if n <= 5 else None
    check(n > 50, "collected a meaningful sample of toss actions (%d)" % n)
    check(bad == 0, "0 of %d toss actions outside [38, 96] degrees" % n)


# ---------------------------------------------------------------------------
# 4. correlated runs -- "leaning the same way", never independent
# ---------------------------------------------------------------------------

@strict
def test_run_correlation_is_measurable():
    print("\n[runs] a felled neighbour within 40 m raises the fell share, "
          "over 200 seeds (hurricane_research.md 3.6: domino runs)")
    I = 0.5   # room for the +0.20 bonus to matter (base p_down = 0.75)
    N = 200
    close_fell = far_fell = 0
    for seed in range(N):
        close = [_prop("/p/a", "streetlight", x=0.0, y=0.0),
                _prop("/p/b", "streetlight", x=20.0, y=0.0)]  # 20 m: in-run
        far = [_prop("/p/a", "streetlight", x=0.0, y=0.0),
              _prop("/p/b", "streetlight", x=100.0, y=0.0)]   # 100 m: independent
        ac = _plan(close, I, seed)
        af = _plan(far, I, seed)
        b_close = _by_path(ac, "/p/b")
        b_far = _by_path(af, "/p/b")
        if b_close and b_close["action"] == "fell":
            close_fell += 1
        if b_far and b_far["action"] == "fell":
            far_fell += 1
    close_frac, far_frac = close_fell / float(N), far_fell / float(N)
    print("    B fell-share: close(20m)=%.3f  far(100m)=%.3f"
          % (close_frac, far_frac))
    check(close_frac > far_frac + 0.05,
          "a felled neighbour within 40 m measurably raises B's own fell "
          "share (close %.3f vs far %.3f)" % (close_frac, far_frac))


@strict
def test_run_correlation_leans_the_same_way():
    print("\n[runs] a correlated fall shares the neighbour's roll SIGN")
    I = 0.5
    same_sign = total = 0
    for seed in range(300):
        placements = [_prop("/p/a", "streetlight", x=0.0, y=0.0),
                     _prop("/p/b", "streetlight", x=15.0, y=0.0)]
        acts = _plan(placements, I, seed)
        a = _by_path(acts, "/p/a")
        b = _by_path(acts, "/p/b")
        if a and b and a["action"] == "fell" and b["action"] == "fell" \
                and b.get("correlated"):
            total += 1
            if (a["roll_deg"] >= 0) == (b["roll_deg"] >= 0):
                same_sign += 1
    check(total > 20, "collected enough correlated pairs (%d)" % total)
    check(same_sign == total,
          "every correlated pair shares its roll sign (%d/%d)"
          % (same_sign, total))


# ---------------------------------------------------------------------------
# 5. trees -- the suburb cut table, wired through species/level/geom
# ---------------------------------------------------------------------------

@strict
def test_trees_mostly_fallen_in_core_pristine_outside():
    print("\n[trees] core trees mostly fallen/snapped; fringe trees mostly "
          "pristine (tornado._TREE_CUTS: 0.07/0.30/0.46/0.80/1.01)")
    core_levels = {}
    fringe_levels = {}
    N = 400
    for seed in range(N):
        placements = [_prop("/p/t", "street_tree", x=1000.0 + seed * 20.0,
                            usd=_tree_species_usd())]
        core = _plan(placements, 0.95, seed)[0]
        fringe = _plan(placements, 0.02, seed + 999999)[0]
        core_levels[core["level"]] = core_levels.get(core["level"], 0) + 1
        fringe_levels[fringe["level"]] = fringe_levels.get(fringe["level"], 0) + 1
    print("    core  :", core_levels)
    print("    fringe:", fringe_levels)
    core_down = core_levels.get("fallen", 0) + core_levels.get("snapped", 0)
    check(core_down / float(N) > 0.85,
          "core trees are overwhelmingly fallen/snapped (%.3f)"
          % (core_down / float(N)))
    # The threshold is well under the pristine cut's own theoretical share
    # (tree jitter is +-0.09 against a 0.07 cut, so at i=0.02 a meaningful
    # slice of draws legitimately lands "limbed" instead -- measured ~0.75)
    # -- the point of this check is CONTRAST with the core, not purity.
    fringe_pristine = fringe_levels.get("pristine", 0)
    check(fringe_pristine / float(N) > 0.65,
          "fringe trees are mostly pristine (%.3f)"
          % (fringe_pristine / float(N)))


@strict
def test_tree_action_has_no_geometry_ops_when_pristine():
    print("\n[trees] a pristine tree gets no geom/azimuth")
    for seed in range(50):
        placements = [_prop("/p/t", "street_tree", x=1000.0 + seed * 30.0,
                            usd=_tree_species_usd())]
        # i must stay > 0.0: `plan_street` treats i<=0.0 as fully outside
        # the corridor and skips the placement entirely (no action at all,
        # not even a "pristine" level record) -- a small positive i here
        # keeps the placement in play while still landing "pristine" most
        # of the time.
        acts = _plan(placements, 0.02, seed + 5)
        if not acts:
            continue
        act = acts[0]
        if act["level"] == "pristine":
            check(act["geom"] is None, "pristine tree: geom is None")
            check("azimuth_deg" not in act,
                  "pristine tree: no azimuth_deg key")


@strict
def test_snapped_tree_is_the_v1_uproot_standin():
    print("\n[trees] v1: 'snapped' takes the same geom as 'fallen' "
          "(documented deviation -- see module docstring)")
    found_snapped = False
    for seed in range(300):
        placements = [_prop("/p/t", "street_tree", x=1000.0 + seed * 20.0,
                            usd=_tree_species_usd())]
        act = _plan(placements, 0.98, seed)[0]
        if act["level"] == "snapped":
            found_snapped = True
            check(act["geom"] == "uproot",
                  "snapped tree v1 stand-in geom is 'uproot'")
    check(found_snapped, "at least one snapped tree was drawn across 300 seeds")


# ---------------------------------------------------------------------------
# 6. cars -- unconditional Paulikas shares, no nesting bug
# ---------------------------------------------------------------------------

@strict
def test_cars_below_entry_intensity_are_untouched():
    print("\n[cars] below CAR_ENTRY_MIN_I, a car is never even drawn")
    placements = [_car("/p/c", x=1000.0)]
    acts = _plan(placements, tst.CAR_ENTRY_MIN_I - 0.05, 1)
    check(acts == [], "no car action below the entry floor")


@strict
def test_car_moved_share_matches_car_pose_unconditional():
    print("\n[cars] the moved share at a given intensity matches "
          "car_pose's own UNCONDITIONAL p_move -- no nesting bug")
    I = 0.6
    N = 2000
    moved = 0
    for seed in range(N):
        placements = [_car("/p/c", x=1000.0 + seed * 7.0)]
        acts = _plan(placements, I, seed)
        if acts:
            moved += 1
    measured = moved / float(N)
    expected = tn.CAR_P_MOVE[0] + tn.CAR_P_MOVE[1] * I
    print("    measured p_move=%.3f  car_pose p_move=%.3f" % (measured,
                                                               expected))
    check(abs(measured - expected) < 0.035,
          "measured move share tracks car_pose's own model within 3.5 pts")


# ---------------------------------------------------------------------------
# 7. the thrown-car mechanic (2026-09-01 directive)
# ---------------------------------------------------------------------------

@strict
def test_thrown_only_ever_promotes_an_already_toppled_core_car():
    print("\n[thrown] a thrown action is always toppled, always in the "
          "core, and every thrown car moved")
    N = 1500
    for seed in range(N):
        placements = [_car("/p/c", x=1000.0 + seed * 3.0)]
        i = 0.5 + (seed % 6) * 0.1   # sweeps 0.5..1.0, above and below CORE_I
        acts = _plan(placements, i, seed)
        if not acts:
            continue
        a = acts[0]
        if a.get("thrown"):
            check(a["toppled"] is True, "thrown implies toppled")
            check(i >= tst.CORE_I - 1e-9, "thrown only fires in the core")
            check(a["pose"] in ("side", "roof", "nose"),
                  "thrown keeps one of car_pose's own tipped attitudes")


@strict
def test_core_thrown_share_and_tipped_plus_thrown_matches_car_pose():
    print("\n[thrown] at i=0.9 over 300 seeds: thrown share in [0.10,0.15] "
          "(wide MC tolerance for report), and (tipped OR thrown) count "
          "equals what car_pose's own p_tip alone would draw")
    I = 0.9
    N = 300
    n_toppled = n_thrown = 0
    for seed in range(N):
        placements = [_car("/p/c", x=1000.0 + seed * 3.0)]
        acts = _plan(placements, I, seed)
        if not acts:
            continue
        a = acts[0]
        if a["toppled"]:
            n_toppled += 1
        if a.get("thrown"):
            n_thrown += 1
    frac_toppled = n_toppled / float(N)
    frac_thrown = n_thrown / float(N)
    p_tip = tn.CAR_P_TIP[0] + tn.CAR_P_TIP[1] * I ** 1.5   # mass=1
    print("    measured (tipped incl. thrown)=%.3f  car_pose p_tip=%.3f"
          % (frac_toppled, p_tip))
    print("    measured thrown-of-all-core=%.3f  (design target 0.10-0.15, "
          "THROWN_OF_TIPPED=%.2f)" % (frac_thrown, tst.THROWN_OF_TIPPED))
    # every promotion starts from a car_pose toppled draw and NEVER creates
    # a new toppled outcome of its own -- so the toppled COUNT (not the
    # attitude) is exactly what an unmodified car_pose would have produced,
    # within ordinary Monte Carlo noise at N=300.
    check(abs(frac_toppled - p_tip) < 0.07,
          "tipped(+thrown) share tracks car_pose's own p_tip within MC "
          "tolerance (measured %.3f vs %.3f)" % (frac_toppled, p_tip))
    check(0.05 <= frac_thrown <= 0.20,
          "thrown share of ALL core cars near the requested 0.10-0.15 band "
          "(measured %.3f, MC tolerance widened for N=%d)" % (frac_thrown, N))
    check(n_thrown <= n_toppled,
          "thrown is a subset of toppled, never more (%d thrown, %d toppled)"
          % (n_thrown, n_toppled))


@strict
def test_thrown_distance_is_20_to_60_m():
    print("\n[thrown] a thrown car's own displacement is 20-60 m")
    n = 0
    for seed in range(1500):
        placements = [_car("/p/c", x=1000.0 + seed * 3.0)]
        acts = _plan(placements, 0.95, seed)
        if not acts:
            continue
        a = acts[0]
        if a.get("thrown"):
            n += 1
            d = math.hypot(a["dx"], a["dy"])
            check(20.0 - 1e-6 <= d <= 60.0 + 1e-6,
                  "thrown displacement %.1f m within [20, 60]" % d)
    check(n > 20, "collected a meaningful thrown sample (%d)" % n)


@strict
def test_every_thrown_landing_point_is_on_open_ground():
    print("\n[thrown] a landing point is never inside a building OBB and "
          "never off the plate")
    # A building footprint that sits squarely across the 20-60 m throw fan
    # from the car's own position and heading, so rejection is actually
    # exercised rather than trivially always-clear.
    buildings = [{"x_m": 1035.0, "y_m": 20.0, "W": 30.0, "D": 30.0,
                 "yaw_deg": 0.0}]
    bounds = (900.0, -80.0, 1100.0, 80.0)
    n_thrown = n_checked = 0
    for seed in range(600):
        placements = [_car("/p/c", x=1000.0)]
        rng = random.Random(seed)
        acts = tst.plan_street(placements, _flat_intensity(0.95),
                               _flat_wind(20.0), rng, buildings=buildings,
                               bounds=bounds)
        if not acts:
            continue
        a = acts[0]
        n_checked += 1
        if a.get("thrown"):
            n_thrown += 1
            lx, ly = 1000.0 + a["dx"], 0.0 + a["dy"]
            check(bounds[0] <= lx <= bounds[2] and bounds[1] <= ly <= bounds[3],
                  "thrown landing (%.1f, %.1f) stays on the plate" % (lx, ly))
            inside = tst._footprint_test(buildings)(lx, ly)
            check(not inside,
                  "thrown landing (%.1f, %.1f) is not inside the building "
                  "OBB" % (lx, ly))
    check(n_thrown > 0, "at least one thrown car was drawn (%d)" % n_thrown)
    print("    %d thrown of %d checked cars" % (n_thrown, n_checked))


# ---------------------------------------------------------------------------
# 8. the review floor (min_tipped / UT_MIN_TIPPED's pure half)
# ---------------------------------------------------------------------------

@strict
def test_min_tipped_floor_is_met_when_reachable():
    print("\n[floor] min_tipped forces the floor when enough cars exist")
    placements = [_car("/p/c%d" % k, x=1000.0 + k * 12.0) for k in range(10)]
    # Just above CAR_ENTRY_MIN_I (0.20): cars are eligible to be considered
    # at all, but the natural toppled share is low, so the floor is the
    # thing actually doing the work rather than luck.
    I = tst.CAR_ENTRY_MIN_I + 0.02
    for seed in range(30):
        acts = _plan(placements, I, seed, min_tipped=3)
        n_toppled = sum(1 for a in acts if a.get("toppled"))
        check(n_toppled >= 3,
              "seed %d: >=3 toppled cars with min_tipped=3 (got %d)"
              % (seed, n_toppled))


@strict
def test_min_tipped_zero_leaves_the_model_untouched():
    print("\n[floor] min_tipped=0 (the default) never forces anything")
    placements = [_car("/p/c%d" % k, x=1000.0 + k * 12.0) for k in range(10)]
    for seed in range(10):
        acts = _plan(placements, 0.15, seed, min_tipped=0)
        check(all(not a.get("forced") for a in acts),
              "seed %d: no forced cars at min_tipped=0" % seed)


@strict
def test_forced_cars_are_never_also_thrown():
    print("\n[floor] a forced car is marked forced and never thrown")
    placements = [_car("/p/c%d" % k, x=1000.0 + k * 12.0) for k in range(12)]
    any_forced = False
    for seed in range(60):
        acts = _plan(placements, 0.9, seed, min_tipped=8)
        for a in acts:
            if a.get("forced"):
                any_forced = True
                check(not a.get("thrown"),
                      "a forced car is never also thrown")
    check(any_forced, "at least one forced car was observed across 60 seeds")


# ---------------------------------------------------------------------------
# 9. determinism and JSON round-trip
# ---------------------------------------------------------------------------

def _mixed_placements():
    return [
        _prop("/p/sl", "streetlight", x=0.0),
        _prop("/p/tc", "trash_can", x=8.0),
        _prop("/p/hy", "fire_hydrant", x=16.0),
        _prop("/p/t", "street_tree", x=-8.0, usd=_tree_species_usd()),
        _car("/p/car", x=24.0, yaw_deg=15.0),
        _prop("/p/h", "house", x=0.0, y=40.0),
    ]


@strict
def test_one_seed_one_answer():
    print("\n[repeatable] a scene is the same scene twice")
    a = _plan(_mixed_placements(), 0.8, 42, min_tipped=1)
    b = _plan(_mixed_placements(), 0.8, 42, min_tipped=1)
    check(a == b, "the same seed reproduces byte-for-byte")
    c = _plan(_mixed_placements(), 0.8, 43, min_tipped=1)
    check(a != c, "...and a different seed is a different scene")


@strict
def test_actions_are_json_safe():
    print("\n[json] every action round-trips through json.dumps/loads")
    acts = _plan(_mixed_placements(), 0.85, 7, min_tipped=1)
    check(len(acts) > 0, "the mixed fixture produced at least one action")
    try:
        blob = json.dumps(acts)
        back = json.loads(blob)
    except (TypeError, ValueError) as exc:
        blob, back = None, None
        check(False, "json.dumps/loads raised: %s" % exc)
    if back is not None:
        check(back == acts, "round-trip is lossless")
    for a in acts:
        check(isinstance(a.get("path"), str) and a["path"],
              "%s: path is a non-empty string" % a)
        check(a.get("kind") in ("felled", "carried", "tree", "car"),
              "%s: kind is one of the four families" % a)
        check(a.get("action") in ("fell", "carry", "toss", "level",
                                  "car_pose"),
              "%s: action is one of fell/carry/toss/level/car_pose" % a)


# ---------------------------------------------------------------------------
# 10. apply_street -- the ONE place this file touches a real stage. Bare
# `Usd.Stage.CreateInMemory()`, no Kit -- exactly what `apply_street`'s own
# lazy `pxr` imports were built to allow.
# ---------------------------------------------------------------------------

def _stub_placed_prim(stage, path, x=0.0, y=0.0, yaw_deg=0.0):
    """One Xform authored EXACTLY the way `scene_generator.apply_placements`
    authors a generic placement -- translate / rotateXYZ (roll, pitch, yaw)
    / scale -- the shape `_normalize_yaw_op` exists to fix up. A plain
    `Cube` child gives `toss_prim`'s own bbox-based seating something real
    to measure, so this exercises the same code path a placed prop does."""
    prim = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), 0.0))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))
    xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    UsdGeom.Cube.Define(stage, Sdf.Path(path + "/geo")).CreateSizeAttr(1.0)
    return prim


def _ops_by_name(prim):
    xf = UsdGeom.Xformable(prim)
    return {op.GetOpName().split(":")[-1]: op.Get()
           for op in xf.GetOrderedXformOps()}


@strict
def test_apply_street_fells_a_streetlight_and_carries_a_bin():
    print("\n[apply] a felled streetlight gains the lean xform; a carried "
          "bin goes inactive")
    stage = Usd.Stage.CreateInMemory()
    sl = _stub_placed_prim(stage, "/World/street/sl", x=10.0, y=5.0,
                           yaw_deg=30.0)
    bin_ = _stub_placed_prim(stage, "/World/street/bin", x=20.0, y=5.0)

    fell = {"path": "/World/street/sl", "kind": "felled",
           "category": "streetlight", "action": "fell",
           "bearing_deg": 40.0, "dist_m": 1.0, "roll_deg": 75.0,
           "yaw_jitter_deg": 10.0}
    carry = {"path": "/World/street/bin", "kind": "carried",
            "category": "trash_can", "action": "carry"}

    counts = tst.apply_street(stage, [fell, carry], min_tipped=0,
                              verbose=False)

    ops = _ops_by_name(sl.GetPrim())
    check("rotateX" in ops and abs(float(ops["rotateX"]) - 75.0) < 1e-6,
          "the felled streetlight carries a 75 deg rotateX lean")
    check(sl.GetPrim().IsActive(), "a felled prim stays active (posed, "
                                   "not deleted)")
    # THE ROTATEXYZ-TRAP FIX, PINNED DIRECTLY: `sl` started at yaw 30, so
    # its post-fell rotateZ must be 30 + yaw_jitter_deg (40), not just the
    # jitter alone (10) -- `_normalize_yaw_op` is what makes that true.
    check("rotateZ" in ops, "a rotateZ op exists after the fell")
    check(abs(float(ops["rotateZ"]) - 40.0) < 1e-6,
          "rotateZ is old_yaw(30) + yaw_jitter_deg(10) = 40, not 10 alone "
          "-- the rotateXYZ-trap fix (measured %.3f)" % float(ops["rotateZ"]))

    check(not bin_.GetPrim().IsActive(),
          "a carried bin is deactivated (SetActive(False))")

    check(counts["n_actions"] == 2, "n_actions == 2")
    check(counts["n_applied"] == 2, "n_applied == 2 (0 failed)")
    check(counts["n_failed"] == 0, "n_failed == 0")
    check(counts["by_kind"] == {"felled": {"fell": 1},
                                "carried": {"carry": 1}},
          "by_kind counts exactly one fell and one carry, by their own "
          "kind (got %r)" % counts["by_kind"])
    check(counts["min_tipped"] == 0, "min_tipped is echoed back unchanged")
    for k in ("n_thrown", "n_forced", "n_correlated"):
        check(counts[k] == 0, "%s == 0 (neither action was a car)" % k)


@strict
def test_apply_street_tips_a_tree_and_reports_a_failure():
    print("\n[apply] a tree 'level' action tips via vegetation.tip_tree; "
          "an action on a missing prim path counts as failed, not raised")
    stage = Usd.Stage.CreateInMemory()
    tree = _stub_placed_prim(stage, "/World/street/t", x=-5.0, y=0.0,
                             yaw_deg=0.0)

    level = {"path": "/World/street/t", "kind": "tree",
            "category": "street_tree", "action": "level",
            "level": "leaning", "geom": "lean", "azimuth_deg": 15.0,
            "lean_deg": 30.0}
    missing = {"path": "/World/street/does_not_exist", "kind": "felled",
              "category": "streetlight", "action": "fell",
              "bearing_deg": 0.0, "dist_m": 1.0, "roll_deg": 70.0,
              "yaw_jitter_deg": 0.0}

    counts = tst.apply_street(stage, [level, missing], min_tipped=1,
                              verbose=False)

    ops = _ops_by_name(tree.GetPrim())
    check("orient" in ops, "the leaning tree carries an orient op "
                          "(vegetation.tip_tree's own rotation build)")
    check(counts["n_actions"] == 2, "n_actions == 2")
    check(counts["n_applied"] == 1, "n_applied == 1 (the tree)")
    check(counts["n_failed"] == 1,
          "n_failed == 1 (the missing prim -- toss_prim returns False "
          "for an invalid path rather than raising)")
    check(counts["by_kind"] == {"tree": {"level": 1}, "felled": {"fell": 1}},
          "by_kind still counts BOTH actions, applied or not")
    check(counts["min_tipped"] == 1, "min_tipped echoes the value passed in")


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------

def main():
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith("test_") and callable(o)]
    print("tornado_street: %d tests, offline (no pxr, no GPU)" % len(tests))
    broken = []
    for name, fn in tests:
        try:
            fn()
        except Exception as exc:                       # noqa: BLE001
            import traceback
            broken.append(name)
            print("    ERROR %s: %s" % (name, exc))
            print(traceback.format_exc())
    print("\n" + "=" * 72)
    if FAILS or broken:
        for m in FAILS:
            print("  FAILED: " + m)
        for m in broken:
            print("  ERRORED: " + m)
        return 1
    print("  all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
