#!/usr/bin/env python3
"""test_quake_twins.py — does a SAME_ART original (an MCE merged building)
get a size-matched kit TWIN instead of `urban_building.py`'s own generic
styles, and does the swap leave the placement's pose alone?

    python3 scene_gen/tests/test_quake_twins.py
    pytest -q scene_gen/tests/test_quake_twins.py

WHY THIS EXISTS
---------------
Round 5 (user, 2026-08-30, looking at the first M7.8 500 m earthquake
scene): "Looks like you're assembling the building yourself and as a result
a lot of the buildings look wrong. Once again look at urban fire, the
buildings look correct there." The fire bench the user likes shows the REAL
`ModernCityEnvironment` merged originals beside size-matched kit TWINS
(`disaster/kit_substitute.route`); `urban_quake_v3.yaml` puts exactly those
three MCE buildings in the earthquake pool, and `disaster/quake.
decide_building` is the pure per-building decision that turns "this
building's grade is DG3" into "reference `bld_commercial_mid_DG3.usd`
instead" (or, when `kit_substitute.route` refuses, leaves the original
completely alone).

`decide_building` is PURE (no pxr, no stage) by construction — see its
docstring — precisely so it can be unit-tested here and reused, unmodified,
by `tools/layout_dry_run.py`'s offline city-scale tally. The one thing this
file needs `pxr` for at all is proving that a reference swap on an authored
placement prim (`ClearReferences` + `AddReference`, exactly what `assemble`'s
same_art branch does) never touches the prim's translate/rotate/scale ops —
`usd-core` (no Kit) is enough for that.

WHAT THIS CANNOT SEE: whether the twin's own geometry actually looks right
next to the kept original in a render, whether `_mono_pass`'s fallback lean
reads correctly on a KEPT same_art building. That needs Isaac Sim — not run
here.
"""

import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import quake as q               # noqa: E402
from disaster import kit_substitute as ks      # noqa: E402


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
_MB01_USD = ("Muyang/ModernCityEnvironment/Collected_Building01/"
            "SM_MERGED_BP_MBuilding01.usd")
_MB01_DIMS = (28.5, 18.5, 29.0)      # measured — see kit_substitute's docstring
_MB02_USD = ("Muyang/ModernCityEnvironment/Collected_Building02/"
            "SM_MERGED_BP_MBuilding02.usd")
_MB02_DIMS = (91.1, 96.1, 68.7)


def _manifest(*rows):
    """{(style, level): record} from (style, level, usd) tuples — a synthetic
    stand-in for `quake.load_manifest`'s real archetypes.json read."""
    out = {}
    for style, level, usd in rows:
        out[(style, level)] = {"style": style, "level": level, "usd": usd,
                               "W": 1.0, "D": 1.0, "H": 1.0}
    return out


def test_module_exposes_the_pure_decision_function():
    for name in ("decide_building", "_same_art_material", "_pool_entries"):
        assert hasattr(q, name), name


# ---------------------------------------------------------------------------
# decide_building — the pure per-building contract
# ---------------------------------------------------------------------------
def test_dg0_keeps_the_original_untouched():
    """A pristine (DG0) same_art building is never even asked about a twin —
    `route()` is not called at all (proved by dimensions no style could ever
    match, exactly like `kit_substitute.check()`'s own "already kit" case)."""
    rng = random.Random(0)
    out = q.decide_building(_MB01_USD, "DG0", 999.0, 999.0, 999.0, "urm",
                            {}, rng, x=10.0, y=-4.0, yaw_deg=90.0)
    assert out == {"x": 10.0, "y": -4.0, "yaw_deg": 90.0,
                  "action": "keep", "grade": "DG0"}


def test_dg3_becomes_the_named_kit_twin_with_pose_preserved():
    """MBuilding01 at DG3 -> `commercial_mid` DG3 — the exact named match
    `kit_substitute.check()` asserts for this asset's real measured
    dimensions — and the placement's x/y/yaw ride through unchanged (the
    swap only ever touches the reference, never the transform)."""
    manifest = _manifest(
        ("commercial_mid", "DG3", "/arch/bld_commercial_mid_DG3.usd"))
    rng = random.Random(1)
    W, D, H = _MB01_DIMS
    out = q.decide_building(_MB01_USD, "DG3", W, D, H, "urm", manifest, rng,
                            x=123.4, y=-56.7, yaw_deg=270.0)
    assert out["action"] == "twin"
    assert out["style"] == "commercial_mid"
    assert out["usd"] == "/arch/bld_commercial_mid_DG3.usd"
    assert out["grade"] == "DG3"
    assert not out["stepped"]
    assert (out["x"], out["y"], out["yaw_deg"]) == (123.4, -56.7, 270.0)


def test_refused_route_keeps_the_original_and_records_a_reason():
    """A 302 m MCE tower has no kit style within `MAX_H_RATIO`/
    `MAX_AREA_RATIO` — `kit_substitute.check()`'s own "never fall back to
    slicing" assertion — so it must come back `keep`, at the DRAWN grade
    (not DG0: the building really is damaged, there is just nothing honest
    to swap it for), with a human-readable reason attached."""
    rng = random.Random(2)
    out = q.decide_building(_MB02_USD, "DG4", 60.0, 140.0, 302.0, "rc",
                            {}, rng, x=0.0, y=0.0, yaw_deg=0.0)
    assert out["action"] == "keep"
    assert out["grade"] == "DG4"
    assert out.get("reason")
    assert "usd" not in out or out.get("usd") is None


def test_already_kit_style_swaps_to_its_own_style():
    """`route()` is general enough to also answer for a building that is
    ALREADY a kit archetype (`pack_of() == 'kit'`, `_kit_style_of` reads the
    style straight off the filename) — proved here by handing it dimensions
    no style could ever match by size, exactly as `kit_substitute.check()`
    proves the same thing for `route()` directly."""
    manifest = _manifest(("office", "DG4", "/arch/bld_office_DG4.usd"))
    rng = random.Random(3)
    out = q.decide_building("omniverse://host/archetype/bld_office_DG0.usd",
                            "DG4", 999.0, 999.0, 999.0, "urm", manifest, rng,
                            x=1.0, y=2.0, yaw_deg=45.0)
    assert out["action"] == "twin"
    assert out["style"] == "office"
    assert out["usd"] == "/arch/bld_office_DG4.usd"
    assert (out["x"], out["y"], out["yaw_deg"]) == (1.0, 2.0, 45.0)


def test_fallback_steps_down_to_the_nearest_baked_grade():
    """MBuilding01 drawn at DG4, but the manifest only has `commercial_mid`
    baked at DG3 (and DG1) — the SAME "step down until something is baked"
    fallback `assemble`'s already-kit branch uses, so a missing DG4 becomes
    a DG3 rather than a pristine building in the shaking core."""
    manifest = _manifest(
        ("commercial_mid", "DG1", "/arch/bld_commercial_mid_DG1.usd"),
        ("commercial_mid", "DG3", "/arch/bld_commercial_mid_DG3.usd"))
    rng = random.Random(4)
    W, D, H = _MB01_DIMS
    out = q.decide_building(_MB01_USD, "DG4", W, D, H, "urm", manifest, rng,
                            x=7.0, y=8.0, yaw_deg=180.0)
    assert out["action"] == "twin"
    assert out["grade"] == "DG3"
    assert out["usd"] == "/arch/bld_commercial_mid_DG3.usd"
    assert out["stepped"] is True


def test_fallback_with_nothing_baked_at_any_grade_keeps_the_original():
    """The named style has NO baked archetype at all (an empty manifest) —
    `decide_building` must not crash or invent a usd; it keeps the original
    with a reason, same as a route refusal."""
    rng = random.Random(5)
    W, D, H = _MB01_DIMS
    out = q.decide_building(_MB01_USD, "DG2", W, D, H, "urm", {}, rng,
                            x=0.0, y=0.0, yaw_deg=0.0)
    assert out["action"] == "keep"
    assert out["grade"] == "DG2"
    assert out.get("reason")


# ---------------------------------------------------------------------------
# _same_art_material — the asset-set `material:` tag lookup
# ---------------------------------------------------------------------------
def test_same_art_material_matches_by_suffix():
    config = {"usds": {"buildings": {"tower": [
        {"usd": _MB01_USD, "scale": 1.0, "material": "urm"},
        {"usd": _MB02_USD, "scale": 0.01, "material": "rc"},
        "some/bare/string/entry.usd",
    ]}}}
    # the placement's usd is asset_root + the pool entry's bare path —
    # ENDS WITH the pool entry, never equals it
    placed = "omniverse://host/Library/" + _MB01_USD
    assert q._same_art_material(placed, config) == "urm"
    placed2 = "omniverse://host/Library/" + _MB02_USD
    assert q._same_art_material(placed2, config) == "rc"


def test_same_art_material_defaults_when_untagged():
    config = {"usds": {"buildings": {"tower": ["bare/string.usd"]}}}
    assert q._same_art_material("anything/at/all.usd", config) == \
        q._SAME_ART_DEFAULT_TYPE


# ---------------------------------------------------------------------------
# the reference swap on a real (in-memory) stage keeps the transform
# ---------------------------------------------------------------------------
def test_reference_swap_keeps_translate_and_yaw_unchanged():
    """`assemble`'s same_art twin branch does exactly this sequence —
    `ClearReferences()` / `AddReference()` / `Load()` — and NOTHING else to
    the prim. `ClearReferences` clears the REFERENCE arc, not the xform op
    order (`scene_generator._set_xform_ops`'s own docstring makes the same
    distinction for `ClearXformOpOrder`), so the placement's translate /
    rotateXYZ / scale ops must come back byte-identical after the swap —
    this is the entire reason a twin lands at "same x, same y, same yaw"
    without `assemble` doing anything extra to arrange it."""
    from pxr import Gf, Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/World/gen/house_0")
    prim.GetReferences().AddReference("does/not/exist_original.usd")

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    t_op = xform.AddTranslateOp()
    t_op.Set(Gf.Vec3d(123.4, -56.7, 0.0))
    r_op = xform.AddRotateXYZOp()
    r_op.Set(Gf.Vec3f(0.0, 0.0, 270.0))
    s_op = xform.AddScaleOp()
    s_op.Set(Gf.Vec3f(1.0, 1.0, 1.0))

    before_t, before_r, before_s = t_op.Get(), r_op.Get(), s_op.Get()
    before_order = xform.GetOrderedXformOps()

    # THE SWAP — same three calls `assemble`'s twin branch makes.
    refs = prim.GetReferences()
    refs.ClearReferences()
    refs.AddReference("does/not/exist_twin_DG3.usd")
    prim.Load()

    assert t_op.Get() == before_t
    assert r_op.Get() == before_r
    assert s_op.Get() == before_s
    assert [op.GetOpName() for op in xform.GetOrderedXformOps()] == \
        [op.GetOpName() for op in before_order]


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("ok   " + name)
            except Exception as exc:
                fails += 1
                print("FAIL " + name + ": " + str(exc))
    raise SystemExit(1 if fails else 0)
