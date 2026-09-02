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
# 11. ROUND 4 -- the min_moved floor, points-based seating, the axis-up
# residual and the laid-down tree. Every number here comes from
# `tools/tornado_street_seat_probe.py` / `tools/tornado_tree_fall_probe.py`
# run against the real bench C1 cell in the container; these tests pin the
# behaviour offline.
# ---------------------------------------------------------------------------

def _wedge_prim(stage, path, x=0.0, y=0.0, yaw_deg=0.0, roll_deg=0.0,
                scale=1.0):
    """A placement whose BOUNDING BOX LIES about it -- the whole point.

    A 4 x 0.2 x 0.2 m spar with a single stub sticking out sideways at the
    TOP: the bbox is 4 x 2 x 0.2 m but there is no geometry at all in the
    (y = -1, z = 0) corner, which is exactly the corner `tornado.toss_prim`
    lifts a rolled prop by. Modelled on the real streetlight
    (`SM_lightpost_light_post_b.usd`, 0.55 x 2.19 x 6.01 m, the 2.19 being
    a lamp arm that exists only at the top), which `toss_prim` floated
    0.788 m off the road.
    """
    prim = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), 0.0))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(float(roll_deg), 0.0, float(yaw_deg)))
    xf.AddScaleOp().Set(Gf.Vec3f(float(scale), float(scale), float(scale)))
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path + "/geo"))
    pts = []
    # the spar, along +z
    for (px, py, pz) in ((-0.1, -0.1, 0.0), (0.1, -0.1, 0.0),
                        (0.1, 0.1, 0.0), (-0.1, 0.1, 0.0),
                        (-0.1, -0.1, 4.0), (0.1, -0.1, 4.0),
                        (0.1, 0.1, 4.0), (-0.1, 0.1, 4.0)):
        pts.append(Gf.Vec3f(px, py, pz))
    # the arm, only at the top, only on -y
    for (px, py, pz) in ((-0.05, -1.0, 3.8), (0.05, -1.0, 3.8),
                        (0.05, -1.0, 4.0), (-0.05, -1.0, 4.0)):
        pts.append(Gf.Vec3f(px, py, pz))
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr([4] * 3)
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    return prim


def _world_min_z(stage, path):
    return tst._points_min_z(stage, path)


@strict
def test_min_moved_floor_leaves_no_pristine_car_and_never_tips_one():
    print("\n[cars] min_moved: the SECOND review floor -- no core car is "
          "left parked, and it never adds a tipped or thrown car")
    cars = [_car("/p/c%d" % k, x=k * 12.0) for k in range(3)]
    n_seeds = 0
    for seed in range(120):
        base = _plan(cars, 0.85, seed, min_tipped=0, min_moved=0)
        floored = _plan(cars, 0.85, seed, min_tipped=0, min_moved=3)
        n_seeds += 1
        check(len(floored) >= len(base),
              "seed %d: the floor never removes an action" % seed)
        if len(floored) < 3:
            continue
        check(len(floored) == 3,
              "seed %d: min_moved=3 leaves no car untouched (%d acted on)"
              % (seed, len(floored)))
        base_tipped = sum(1 for a in base if a.get("toppled"))
        add_tipped = sum(1 for a in floored
                        if a.get("toppled") and a.get("floor") == "moved")
        check(add_tipped == 0,
              "seed %d: a min_moved promotion is never a TIP (%d)"
              % (seed, add_tipped))
        check(sum(1 for a in floored if a.get("thrown"))
              == sum(1 for a in base if a.get("thrown")),
              "seed %d: the thrown count is unchanged by the floor" % seed)
        check(base_tipped <= sum(1 for a in floored if a.get("toppled")),
              "seed %d: the floor never un-tips a car" % seed)
    check(n_seeds == 120, "walked 120 seeds")


@strict
def test_min_moved_records_are_marked_and_min_zero_is_a_no_op():
    print("\n[cars] a min_moved car is stamped forced/floor='moved'; "
          "min_moved=0 changes nothing")
    cars = [_car("/p/c%d" % k, x=k * 12.0) for k in range(3)]
    for seed in range(40):
        a0 = _plan(cars, 0.85, seed, min_tipped=0, min_moved=0)
        a1 = _plan(cars, 0.85, seed, min_tipped=0, min_moved=0)
        check(a0 == a1, "seed %d: min_moved=0 is deterministic" % seed)
    acts = _plan(cars, 0.85, 3, min_tipped=0, min_moved=3)
    forced = [a for a in acts if a.get("floor") == "moved"]
    for a in forced:
        check(a["forced"] is True, "a min_moved car is forced=True")
        check(a["thrown"] is False, "a min_moved car is never thrown")
        check(a["pose"] == "shoved",
              "a min_moved car takes car_pose's shoved branch (got %r)"
              % a["pose"])
    check(json.loads(json.dumps(acts)) == acts,
          "min_moved actions stay JSON round-trippable")


@strict
def test_points_seating_beats_the_bbox_that_floated_the_lamp():
    print("\n[seat] a felled prop is seated by MESH POINTS, so a prop "
          "whose bbox has an empty low corner does not float")
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    _wedge_prim(stage, "/World/street/lamp", x=3.0, y=0.0, yaw_deg=30.0)
    z_pre = _world_min_z(stage, "/World/street/lamp")
    check(abs(z_pre) < 1e-6, "the upright spar starts on grade (%.4f)" % z_pre)

    fell = {"path": "/World/street/lamp", "kind": "felled",
           "category": "streetlight", "action": "fell",
           "bearing_deg": 40.0, "dist_m": 1.0, "roll_deg": 82.0,
           "yaw_jitter_deg": 10.0}
    counts = tst.apply_street(stage, [fell], verbose=False)
    z_post = _world_min_z(stage, "/World/street/lamp")
    check(abs(z_post) <= tst.SEAT_TOL_M,
          "the felled spar's lowest VERTEX is on grade (%.4f m)" % z_post)
    check(counts["n_off_grade"] == 0,
          "apply_street reports 0 props off grade")
    check(counts["n_seated"] == 1, "apply_street reports 1 prop seated")
    check(abs(counts["max_gap_m"]) <= tst.SEAT_TOL_M,
          "the reported worst gap is inside tolerance (%.4f)"
          % counts["max_gap_m"])
    # And the bbox-based answer really is different -- if it were not,
    # this test would pass for the wrong reason.
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bb = bc.ComputeWorldBound(
        stage.GetPrimAtPath(Sdf.Path("/World/street/lamp"))
    ).ComputeAlignedRange()
    check(float(bb.GetMin()[2]) < -0.2,
          "the bbox of the SAME posed prim still reads well below grade "
          "(%.3f) -- i.e. the two data really do disagree"
          % float(bb.GetMin()[2]))


@strict
def test_a_prop_that_stays_put_keeps_its_kerb_height_and_a_thrown_one_does_not():
    print("\n[seat] the datum depends on whether the prop went anywhere: "
          "a kerb-standing prop keeps its kerb, a thrown one lands on "
          "grade")
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    # Both start 0.18 m up -- `scene_generator` places urban street props
    # on `sidewalk_top` (0.015 m + the slab's own thickness), never z=0.
    for name in ("stay", "flung"):
        p = _wedge_prim(stage, "/World/street/" + name)
        for op in UsdGeom.Xformable(p).GetOrderedXformOps():
            if op.GetOpName().split(":")[-1] == "translate":
                op.Set(Gf.Vec3d(0.0, 0.0, 0.18))
    check(abs(_world_min_z(stage, "/World/street/stay") - 0.18) < 1e-6,
          "both start on a 0.18 m kerb")

    stay = {"path": "/World/street/stay", "kind": "felled",
           "category": "streetlight", "action": "fell",
           "bearing_deg": 0.0, "dist_m": 0.9, "roll_deg": 80.0,
           "yaw_jitter_deg": 0.0}
    flung = {"path": "/World/street/flung", "kind": "carried",
            "category": "trash_can", "action": "toss",
            "bearing_deg": 0.0, "dist_m": 30.0, "roll_deg": 80.0,
            "yaw_jitter_deg": 0.0, "pitch_deg": 0.0}
    tst.apply_street(stage, [stay, flung], verbose=False)

    z_stay = _world_min_z(stage, "/World/street/stay")
    z_flung = _world_min_z(stage, "/World/street/flung")
    check(abs(z_stay - 0.18) <= tst.SEAT_TOL_M,
          "a prop that moved %.1f m (< SEAT_STAY_PUT_M) keeps its kerb "
          "height (%.4f, want 0.18)" % (stay["dist_m"], z_stay))
    check(abs(z_flung) <= tst.SEAT_TOL_M,
          "a prop thrown 30 m lands on grade, not 0.18 m above it "
          "(%.4f)" % z_flung)


@strict
def test_axis_up_placement_keeps_its_correction_and_rolls_about_the_long_axis():
    print("\n[seat] a Y-up placement (rotateXYZ roll=90, the car pool's "
          "own convention) keeps that 90 and still rolls about its LENGTH")
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    path = "/World/street/car"
    prim = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.7))
    # Y-up asset placed the way `AssetPools.roll_of`/`yaw_of` place one:
    # roll 90 stands it up, and the art `yaw-offset` of 90 is folded into
    # the yaw so the LENGTH ends up along the placement bearing (0 here).
    xf.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, 90.0))
    xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path + "/geo"))
    # local X = width 2, local Y = height 1.4 (up), local Z = length 4.6
    pts = []
    for sx_ in (-1.0, 1.0):
        for sy_ in (-0.7, 0.7):
            for sz_ in (-2.3, 2.3):
                pts.append(Gf.Vec3f(sx_, sy_, sz_))
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 3, 2])
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)

    act = {"path": path, "kind": "car", "category": "car",
          "action": "car_pose", "pose": "side", "dx": 0.0, "dy": 0.0,
          "d_m": 0.0, "roll_deg": 90.0, "pitch_deg": 0.0,
          "yaw_delta_deg": 0.0, "toppled": True, "thrown": False,
          "forced": False, "floor": None, "intensity": 0.9}
    tst.apply_street(stage, [act], verbose=False)

    names = [op.GetOpName().split(":")[-1]
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps()]
    check("axisUp" in names,
          "the placement's axis-up correction survives as its own suffixed "
          "op (ops: %r)" % names)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = bc.ComputeWorldBound(prim.GetPrim()).ComputeAlignedRange()
    sz = r.GetSize()
    # ON ITS SIDE: the 4.6 m LENGTH stays horizontal and the 2.0 m WIDTH
    # becomes the height. Rolling about the width instead (the bug) would
    # stand the 4.6 m length up and report a ~4.6 m tall car.
    check(abs(float(sz[2]) - 2.0) < 0.05,
          "a car on its side is its own WIDTH tall (%.2f m, want 2.0)"
          % float(sz[2]))
    check(max(float(sz[0]), float(sz[1])) > 4.0,
          "its LENGTH is still horizontal (%.2f m)"
          % max(float(sz[0]), float(sz[1])))
    check(abs(float(r.GetMin()[2])) <= tst.SEAT_TOL_M,
          "and it rests ON the plate, not in it (%.4f m)"
          % float(r.GetMin()[2]))


@strict
def test_uprooted_tree_is_laid_down_and_seated_on_its_butt():
    print("\n[trees] an uprooted tree keeps the lean it was DRAWN (no "
          "seat-band bisection back to lean_min_deg) and its butt lands "
          "one root plate off the ground")
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    path = "/World/street/tree"
    _wedge_prim(stage, path, x=2.0, y=1.0, yaw_deg=140.0)
    act = {"path": path, "kind": "tree", "category": "street_tree",
          "action": "level", "level": "fallen", "geom": "uproot",
          "azimuth_deg": 20.0, "lean_deg": 84.0, "lean_min_deg": 46.0,
          "species": "Shumard_Oak", "intensity": 0.9}
    tst.apply_street(stage, [act], verbose=False)

    ops = {op.GetOpName().split(":")[-1]: op.Get()
          for op in UsdGeom.Xformable(
              stage.GetPrimAtPath(Sdf.Path(path))).GetOrderedXformOps()}
    check("orient" in ops, "the tree carries tip_tree's orient op")
    t = ops["translate"]
    check(abs(float(t[2]) - tst.TREE_ROOT_LIFT_M) < 1e-4,
          "the butt (tip_tree's own pivot) sits at TREE_ROOT_LIFT_M "
          "(%.4f, want %.4f)" % (float(t[2]), tst.TREE_ROOT_LIFT_M))
    # It really was laid DOWN: the quaternion's rotation angle is the lean.
    q = ops["orient"]
    ang = 2.0 * math.degrees(math.acos(min(1.0, abs(float(q.real)))))
    check(abs(ang - 84.0) < 1.0,
          "the applied lean is the 84 deg that was planned, not a "
          "bisected-down 46 (%.1f)" % ang)
    plate = stage.GetPrimAtPath(Sdf.Path(path + "_rootplate"))
    check(bool(plate) and plate.IsValid(),
          "a vegetation.root_plate sibling is authored at the butt")


@strict
def test_tip_azimuth_cancels_the_placement_yaw():
    print("\n[trees] the fall azimuth handed to tip_tree is corrected for "
          "the placement yaw its op order applies on top")
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    path = "/World/street/t2"
    _wedge_prim(stage, path, x=0.0, y=0.0, yaw_deg=270.0)
    tst._normalize_yaw_op(stage, path)
    got = tst._tip_azimuth(stage, path, 63.0)
    check(abs(got - (63.0 - 270.0)) < 1e-6,
          "azimuth 63 under a 270 deg placement yaw is handed over as "
          "-207 (got %.3f)" % got)
    check(abs(tst._tip_azimuth(stage, "/World/street/nope", 63.0) - 63.0)
          < 1e-6, "a missing prim degrades to the raw azimuth")


@strict
def test_uproot_lean_band_lays_the_tree_down():
    print("\n[trees] TREE_UPROOT_RANGE_DEG is a LAID-DOWN band, and "
          "lean_min_deg is no longer a floor anything bisects to")
    lo, hi = tst.TREE_UPROOT_RANGE_DEG
    check(lo >= 70.0, "the shallowest uproot lean is >= 70 deg (%.1f) -- "
                      "measured off/crown_r 0.94 at 70, 0.73 at 46" % lo)
    check(hi <= 92.0, "the deepest is <= 92 deg (%.1f)" % hi)
    leans = []
    for seed in range(300):
        placements = [_prop("/p/t", "street_tree", x=1000.0 + seed * 20.0,
                            usd=_tree_species_usd())]
        act = _plan(placements, 0.97, seed)[0]
        if act.get("geom") == "uproot":
            leans.append(act["lean_deg"])
    check(leans, "at least one uproot was drawn")
    check(min(leans) >= lo - 1e-6 and max(leans) <= hi + 1e-6,
          "every drawn uproot lean is inside the band (%.1f..%.1f)"
          % (min(leans), max(leans)))


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
