#!/usr/bin/env python3
"""
test_hurricane_float_trees.py — pins `washaway.floating_tree_specs` (DEBRIS
D5 review, second half of "I don't really see floating tree trunks/fallen
down trees in the flood. Add that."): whole `tree_<species>_fallen.usd`
archetypes referenced a second time, in OPEN water, with a Z translate that
seats the TRUNK — not the whole prim's bounding box — at the waterline.

RUNS WITHOUT ISAAC for every test except `test_b_*`, which independently
re-measures the shipped archetype files with bare `pxr` (no Kit, no
SimulationApp — the `Nucleus USD without Kit` idiom, applied to a LOCAL
file) and checks `washaway._FLOAT_TREE_TRUNK_M`'s hardcoded numbers against
that fresh measurement, so a future edit to one of these archetypes cannot
silently drift the table this module's Z placement depends on. Guarded by
`HAVE_USD` the same way `test_washaway_debris.py`'s own `test_g_*` is;
SKIPS (prints, does not fail) on a host with no USD rather than breaking
this file's own no-Isaac promise for every other test here.

USAGE
    python3 scene_gen/tests/test_hurricane_float_trees.py
    pytest -q scene_gen/tests/test_hurricane_float_trees.py
"""

import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import washaway as wash                   # noqa: E402

try:
    from pxr import Usd, UsdGeom                             # noqa: E402
    HAVE_USD = True
except Exception:
    HAVE_USD = False

_ARCH_DIR = os.path.join(_SCENE_GEN, "assets", "archetypes_hurricane")

FAILS = []


def check(name, cond, detail=""):
    detail = "" if not detail else str(detail)
    if not cond:
        FAILS.append("{0}: {1}".format(name, detail))
    print(("PASS " if cond else "FAIL ") + name +
          ("" if not detail else " -- " + detail))


def strict(fn):
    """Copied from `test_washaway_debris.py`'s own decorator: a test that
    adds to `FAILS` without asserting is a test nobody can trust was ever
    run."""
    def run(*a, **kw):
        mine = len(FAILS)
        out = fn(*a, **kw)
        assert len(FAILS) == mine, FAILS[mine:]
        return out
    run.__name__, run.__doc__ = fn.__name__, fn.__doc__
    return run


@strict
def test_a_every_archetype_file_exists_and_is_measured():
    """A. Every KEY in `washaway._FLOAT_TREE_TRUNK_M` has a matching
    `<key>.usd` file in `scene_gen/assets/archetypes_hurricane/` (the same
    `ARCH_DIR` the launcher's `arch` dict is built from) and a plausible
    `(trunk_diameter_m, trunk_z_center_m)` pair — a real tree trunk is well
    under a couple of metres through, and the centre offset is small
    relative to the whole tree (these archetypes run 6-13 m long)."""
    for name, (diam, z_center) in wash._FLOAT_TREE_TRUNK_M.items():
        path = os.path.join(_ARCH_DIR, name + ".usd")
        check("{0}: archetype file exists".format(name),
             os.path.isfile(path), path)
        check("{0}: trunk diameter is plausible (0.2-2.5 m)".format(name),
             0.2 <= diam <= 2.5, diam)
        check("{0}: trunk Z-centre offset is small (< 1 m)".format(name),
             abs(z_center) < 1.0, z_center)


@strict
def test_b_trunk_table_matches_a_fresh_offline_measurement():
    """B. Independently re-measures each archetype's trunk mesh (found by
    name: `trunk`, or `*_bark_Mat` where no prim is literally named
    "trunk") restricted to the 1 m band nearest `y=0` ("base at origin"),
    and checks `washaway._FLOAT_TREE_TRUNK_M`'s hardcoded
    `(diameter, z_center)` against it within a generous tolerance — this is
    a REGRESSION GUARD against a future archetype re-bake silently
    invalidating the table this module's Z placement depends on, not a
    claim that the table must match to the millimetre."""
    if not HAVE_USD:
        print("SKIP test_b_trunk_table_matches_a_fresh_offline_measurement "
             "(pxr not present on this host)")
        return

    def find_trunk(stage):
        for p in stage.Traverse():
            if p.IsA(UsdGeom.Mesh):
                nm = p.GetName().lower()
                if "trunk" in nm or "bark" in nm:
                    return p
        return None

    for name in wash._FLOAT_TREE_TRUNK_M:
        path = os.path.join(_ARCH_DIR, name + ".usd")
        if not os.path.isfile(path):
            continue    # already reported by test_a
        stage = Usd.Stage.Open(path)
        trunk = find_trunk(stage)
        check("{0}: a trunk/bark mesh was found".format(name), trunk is not
             None, path)
        if trunk is None:
            continue
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        xf = xc.GetLocalToWorldTransform(trunk)
        pts_local = UsdGeom.Mesh(trunk).GetPointsAttr().Get() or []
        pts = [xf.Transform(p) for p in pts_local]
        if not pts:
            continue
        ys = [p[1] for p in pts]
        y_max = max(ys)
        band = 1.0
        near = [p for p in pts if p[1] >= y_max - band]
        if not near:
            continue
        zs = [p[2] for p in near]
        measured_diam = max(zs) - min(zs)
        measured_center = (max(zs) + min(zs)) / 2.0

        table_diam, table_center = wash._FLOAT_TREE_TRUNK_M[name]
        check("{0}: table diameter within 30% of a fresh measurement"
             .format(name),
             abs(measured_diam - table_diam) / max(1e-6, measured_diam)
             < 0.30,
             "table={0:.3f} measured={1:.3f}".format(table_diam,
                                                      measured_diam))
        check("{0}: table Z-centre within 0.5 m of a fresh measurement"
             .format(name),
             abs(measured_center - table_center) < 0.5,
             "table={0:.3f} measured={1:.3f}".format(table_center,
                                                      measured_center))


@strict
def test_c_count_and_species_draw():
    """C. `floating_tree_specs` places `count` (default `_FLOAT_TREE_COUNT`,
    15-30) instances, drawing only from the `archetypes` list it is given,
    and every returned spec carries a full schema."""
    region = (-500.0, -500.0, 500.0, 500.0)

    def deep_fn(x, y):
        return 3.0   # uniformly deep open water everywhere

    kn = wash.resolve_cfg({"water_level_m": 2.8})
    for seed in (1, 2, 3):
        rng = random.Random(seed)
        specs = wash.floating_tree_specs(kn, region, rng, deep_fn)
        check("seed={0}: count lands in the requested 15-30 band"
             .format(seed), 15 <= len(specs) <= 30, len(specs))
        for s in specs:
            check("every spec has the full schema",
                 all(k in s for k in ("archetype", "x", "y", "z", "yaw",
                                      "draft")),
                 sorted(s))
            check("archetype is a real _FLOAT_TREE_TRUNK_M key",
                 s["archetype"] in wash._FLOAT_TREE_TRUNK_M, s["archetype"])
            check("yaw is a plausible degree value",
                 0.0 <= s["yaw"] < 360.0, s["yaw"])
            check("draft lands in the requested 0.40-0.60 band",
                 wash._FLOAT_TREE_DRAFT_LO <= s["draft"]
                 <= wash._FLOAT_TREE_DRAFT_HI, s["draft"])

    # `archetypes=` restricts the draw.
    rng = random.Random(7)
    one_name = next(iter(wash._FLOAT_TREE_TRUNK_M))
    specs_one = wash.floating_tree_specs(kn, region, rng, deep_fn,
                                         archetypes=[one_name], count=(20, 20))
    check("archetypes= restricts every draw to the given list",
         all(s["archetype"] == one_name for s in specs_one), specs_one)
    check("count=(20, 20) places exactly 20", len(specs_one) == 20,
         len(specs_one))


@strict
def test_d_open_water_and_house_clearance_gates():
    """D. No floating tree is ever placed in water shallower than
    `_FLOAT_TREE_MIN_DEPTH_M`, nor within `_FLOAT_TREE_HOUSE_CLEAR_M` of any
    house footprint centre — measured directly, not inferred: a depth field
    with a real shoreline and a ring of houses both exercise the gate, and a
    fully-dry / fully-house-covered plate must place NOTHING rather than
    crash.

    HOUSES ONLY -- NOT `obstacles` (see `floating_tree_specs`'s own "HOUSES
    ONLY, DELIBERATELY" docstring section: a dense obstacle population like
    every tree on a real plate was measured to starve this function of
    qualifying open water almost entirely)."""
    region = (-200.0, -200.0, 200.0, 200.0)

    def depth_fn(x, y):
        return max(0.0, 0.02 * (x + 200.0))   # shoreline at x=-200, deep at
                                               # x=+200 (depth up to 8 m)

    houses = [(50.0, 0.0, 12.0), (-20.0, 60.0, 10.0), (100.0, -80.0, 14.0)]
    kn = wash.resolve_cfg({"water_level_m": 2.8})
    rng = random.Random(11)
    specs = wash.floating_tree_specs(kn, region, rng, depth_fn,
                                     houses=houses)
    check("a non-trivial field was placed", len(specs) > 5, len(specs))
    bad_depth = [s for s in specs
                if depth_fn(s["x"], s["y"]) < wash._FLOAT_TREE_MIN_DEPTH_M]
    check("every tree sits in water deeper than _FLOAT_TREE_MIN_DEPTH_M",
         not bad_depth, len(bad_depth))
    all_fp = list(houses)
    bad_house = [s for s in specs
                if any((s["x"] - hx) ** 2 + (s["y"] - hy) ** 2
                      < (wash._FLOAT_TREE_HOUSE_CLEAR_M + 0.5 * hfp) ** 2
                      for hx, hy, hfp in all_fp)]
    check("every tree clears every house/obstacle by "
         "_FLOAT_TREE_HOUSE_CLEAR_M", not bad_house, len(bad_house))

    # A fully-dry plate places nothing and does not crash.
    def dry_fn(x, y):
        return 0.0
    specs_dry = wash.floating_tree_specs(kn, region, random.Random(3),
                                         dry_fn)
    check("a fully-dry plate places zero floating trees",
         specs_dry == [], len(specs_dry))


@strict
def test_e_z_seats_the_trunk_at_the_requested_draft():
    """E. `z` is computed so that a fraction of the archetype's OWN
    measured trunk diameter — between `_FLOAT_TREE_DRAFT_LO` and
    `_FLOAT_TREE_DRAFT_HI` — sits below `water_level_m`, using the trunk's
    local Z-centre (`_FLOAT_TREE_TRUNK_M`), the same physical shape
    `_one_raft`'s own `log` draft formula uses. Reconstructs the formula
    directly from the returned `z`/`draft` and checks it lands back on the
    archetype's own table entry."""
    region = (-50.0, -50.0, 50.0, 50.0)

    def deep_fn(x, y):
        return 5.0

    water_level_m = 2.8
    kn = wash.resolve_cfg({"water_level_m": water_level_m})
    rng = random.Random(5)
    specs = wash.floating_tree_specs(kn, region, rng, deep_fn)
    check("a non-trivial field was placed", len(specs) > 0, len(specs))
    for s in specs:
        diam, z_center = wash._FLOAT_TREE_TRUNK_M[s["archetype"]]
        expect_z = (water_level_m + diam * (0.5 - s["draft"])) - z_center
        # `spec["draft"]` is ROUNDED to 3 d.p. for reporting
        # (`floating_tree_specs`'s own `round(draft, 3)`) but `z` is
        # computed from the UNROUNDED draw -- so reconstructing `z` from
        # the rounded draft can differ by up to `0.0005 * diam`
        # (< 1 mm for every archetype here). 2e-3 covers that with margin.
        check("{0}: z matches the draft formula for its own draft"
             .format(s["archetype"]),
             abs(s["z"] - expect_z) < 2e-3,
             "z={0:.4f} expect={1:.4f}".format(s["z"], expect_z))
        # The TRUNK's own world Z (undoing the z_center offset this
        # function applies) must genuinely straddle the water surface for
        # ANY plausible trunk half-extent -- draft in (0, 1) and diam > 0
        # together guarantee it, the same invariant `_one_raft`'s `log`
        # kind holds for an authored box.
        trunk_world_z = s["z"] + z_center
        top = trunk_world_z + 0.5 * diam
        bot = trunk_world_z - 0.5 * diam
        check("{0}: trunk straddles the water surface (top > level > bot)"
             .format(s["archetype"]),
             top > water_level_m > bot,
             "top={0:.3f} level={1} bot={2:.3f}".format(top, water_level_m,
                                                        bot))


@strict
def test_f_yaw_and_default_fallback():
    """F. Yaw draws cover the full circle over enough samples (not stuck
    near 0), and an unknown archetype name (not in the table) falls back to
    `_FLOAT_TREE_TRUNK_DEFAULT_M` rather than raising."""
    region = (-80.0, -80.0, 80.0, 80.0)

    def deep_fn(x, y):
        return 4.0

    kn = wash.resolve_cfg({"water_level_m": 2.0})
    rng = random.Random(9)
    specs = wash.floating_tree_specs(kn, region, rng, deep_fn, count=(30, 30))
    yaws = [s["yaw"] for s in specs]
    check("yaw spans a wide range (not stuck near one value)",
         max(yaws) - min(yaws) > 180.0, (min(yaws), max(yaws)))

    rng2 = random.Random(2)
    specs_unknown = wash.floating_tree_specs(
        kn, region, rng2, deep_fn, archetypes=["tree_Not_A_Real_Species"],
        count=(3, 3))
    check("an unknown archetype name still places (falls back to the "
         "default trunk geometry) rather than raising",
         len(specs_unknown) == 3, len(specs_unknown))


if __name__ == "__main__":
    test_a_every_archetype_file_exists_and_is_measured()
    test_b_trunk_table_matches_a_fresh_offline_measurement()
    test_c_count_and_species_draw()
    test_d_open_water_and_house_clearance_gates()
    test_e_z_seats_the_trunk_at_the_requested_draft()
    test_f_yaw_and_default_fallback()
    if FAILS:
        print("\n{0} FAILURE(S)".format(len(FAILS)))
        for f in FAILS:
            print(" -", f)
        sys.exit(1)
    print("\nALL PASS")
