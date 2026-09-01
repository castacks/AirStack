#!/usr/bin/env python3
"""test_roof_plant_kit.py — does the FOUNDATION family (`r_settlement` /
`r_tilt_severe` / `r_overturn`) land its rooftop plant somewhere real,
offline, against a synthetic stage?

    python3 scene_gen/tests/test_roof_plant_kit.py
    pytest -q scene_gen/tests/test_roof_plant_kit.py

WHY THIS EXISTS
---------------
`scene_gen/tools/roof_plant_seat_probe.py` (points-based, against real baked
archetypes) found the kit roof plant (`tank_dark` / `plant_metal`) floating
at every damaged grade, worst at `OV` and `TILT` (measured over the 162-file
archetype library, both the "after re-bake" pod log and an independent local
re-run of the same 162 files: DG0 and DG1 are CLEAN — 0 of 51 and 0 of 73
clusters fail, gap <=0.03 m — so there is no dress-time mis-seat and no
probe false positive at those grades; the count climbs from DG2 onward and
`OV`/`TILT` alone account for ~60% of the 218-226 floaters).

Investigation (see the `_settle_foundation_roof_plant` module comment in
`disaster/quake_flow.py`) found the roof plant IS carried by the SAME rigid
transform as the rest of the building for all three foundation recipes
(`wreck_building` appends `dress_roof`'s paths into `ctx["fit"]["all"]`
BEFORE any recipe runs, and `_everything(ctx)` — what every foundation
recipe transforms — reads `ctx["fit"]["all"]` straight off; confirmed by
measurement: `bld_apartment_OV.usd`'s un-merged AC units carry a local "up"
axis of roughly (-0.01, -0.98, 0.20), tipped with the fallen shell, not
(0, 0, 1)). So this is NOT the "left upright while the shell rotates under
it" failure. What IS still broken: after the ride, `_b_settle_roof_plant`
hands the prop to the row's SHARED PhysX settle with a small idle tip and no
geometric fallback, and a body on a deck that is now tilted or toppled
either needs more of the shared step budget than it gets (measured: `tilt_
severe` gaps of 2-6 m, frozen partway down the slope) or never had a real
target at all (an `overturn`'s roof no longer faces up anywhere) and drifts
toward the bake-time ground plane, freezing 0.7-1.35 m above true grade with
no support the per-building archetype export can show.

This file pins `quake_flow._settle_foundation_roof_plant`, the geometric
resolver that replaces that hand-off for `tilt_severe` and `overturn`
(NOT `r_settlement`, which already measures ~0 floaters and is left alone):
a support probe under the prop's OWN, already-transformed footprint
(`quake_collapse._deck_support_z`, the identical geometry `_sweep_roof_
props` already uses for the qc_* crush recipes), with a `_a_bury_props`
drop-to-grade fallback when the deck no longer faces up at all. Pure `pxr`,
no Kit, no Isaac Sim, no settle/PhysX dependency — same shape as
`test_quake_collapse.py`'s `test_deck_support_z_*` / `test_sweep_roof_
props_*`, which this is modelled on.

WHAT IT CANNOT SEE: whether a REAL bake's shared per-row PhysX settle still
disagrees with this placement for some OTHER prop this file does not
construct, and whether the corrected archetypes still read right at 40 m.
That needs a rebake and a render.
"""
import math
import os
import random
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import quake_collapse as qc        # noqa: E402
from disaster import quake_flow as qf            # noqa: E402


def _empty_stage(root="/World/b0"):
    from pxr import Sdf, Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, Sdf.Path(root))
    return stage, root


def _stage_box(stage, path, cx, cy, cz, sx, sy, sz, tilt_deg=0.0):
    """A box mesh, optionally rigidly tilted about the X axis BEFORE being
    placed at (cx, cy, cz) — baked into the points (not an xformOp), the
    same convention `test_quake_collapse.py::_stage_box` uses for an
    upright box, extended here with a tilt so a single helper can stand in
    for both a level deck (`tilt_deg=0`) and a deck that rode a `tilt_
    severe` / `overturn` rigid transform. A rigid rotation cannot flip a
    face's winding, so the top face's world normal is (0, -sin(tilt),
    cos(tilt)) — `nz = cos(tilt_deg)`, which is what lets a test pick a
    tilt on either side of `quake_collapse.ROOF_PROP_UP_THRESHOLD` (0.72,
    i.e. ~44 deg) on purpose."""
    from pxr import Gf, Sdf, UsdGeom
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    th = math.radians(tilt_deg)
    c, s = math.cos(th), math.sin(th)

    def _rot(dx, dy, dz):
        return (dx, dy * c - dz * s, dy * s + dz * c)

    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    pts = []
    for dz in (-hz, hz):
        for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            rx, ry, rz = _rot(dx, dy, dz)
            pts.append(Gf.Vec3f(cx + rx, cy + ry, cz + rz))
    faces, counts = [], []
    for f in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5),
              (2, 3, 7, 6), (3, 0, 4, 7)):
        faces.extend(f)
        counts.append(4)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    return mesh.GetPrim()


def _rig_ctx(stage, plant, fixed=None, seed=11, parent="/World/b0"):
    return {"stage": stage, "parent": parent, "rng": random.Random(seed),
           "roof_plant": list(plant), "roof_fixed": list(fixed or []),
           "loose": [], "static_extra": [], "notes": []}


def _world_z_range(stage, path):
    from pxr import Usd, UsdGeom
    pr = stage.GetPrimAtPath(path)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(pr).ComputeAlignedRange()
    return r.GetMin()[2], r.GetMax()[2]


def _rotation_angle_deg(stage, path):
    from pxr import Gf, Usd, UsdGeom
    pr = stage.GetPrimAtPath(path)
    xf = UsdGeom.XformCache()
    tr = Gf.Transform(xf.GetLocalToWorldTransform(pr))
    return tr.GetRotation().GetAngle()


def test_settle_foundation_roof_plant_seats_on_a_tilted_deck():
    """`tilt_severe`'s own range (10-30 deg) stays within `ROOF_PROP_UP_
    THRESHOLD` (nz = cos(10..30) = 0.98..0.87, both > 0.72): the deck the
    whole-body transform just carried the tank onto is still a valid
    resting surface, so this must find it and seat there — NOT hand the
    tank to a settle that has no better idea. THE BUG THIS PINS: without
    this fix the tank would be left wherever `_b_settle_roof_plant`'s idle
    tip put it before a (never-run, in this test) PhysX settle, which is
    exactly the "frozen 2-6 m short" failure measured on real `TILT`
    archetypes."""
    stage, root = _empty_stage()
    tilt = 20.0
    deck = root + "/roof_deck"
    # a REALISTIC kit roof tile, not a single giant slab: the real archetype
    # roof is many ~4-5 m tiles (`bld_apartment_roof_7_*`), and `_deck_
    # support_z`'s own per-prim AABB prune (a cheap bbox check BEFORE the
    # per-triangle pass) rejects a candidate whose OWN bbox top already
    # clears `z_ceiling + margin` — true of a wide tilted panel even where
    # the LOCAL patch under the query is fine. A small tile is both what is
    # actually authored and what keeps this test measuring the up-facing
    # threshold, not an artifact of an oversized fixture.
    _stage_box(stage, deck, 0.0, 0.0, 10.0, 4.0, 3.0, 0.3, tilt_deg=tilt)
    tank = root + "/tank_b0_1"
    # the tank rode the SAME `M` as the deck (that is the part already
    # confirmed working) -- so its base sits close to the deck top at its
    # own (post-transform) footprint, same as `dress_roof`'s original 0.02 m
    # gap. Centred at z=11.33 with a 2.0 m height, its BASE (what `_settle_
    # foundation_roof_plant` measures, not its centre) is 10.33 -- ~0.02 m
    # above the tilted deck's own measured support (10.312, see below).
    tank_cz = 11.33
    _stage_box(stage, tank, 0.0, 0.0, tank_cz, 1.6, 1.6, 2.0)

    # ground truth, measured on the UNTOUCHED stage before the call mutates
    # it: the exact support this construction gives, and the tank's real
    # base (not its authored centre).
    base_before, _ = _world_z_range(stage, tank)
    expected = qc._deck_support_z(stage, root, cx=0.0, cy=0.0, half_w=0.92,
                                  half_d=0.92, z_ceiling=base_before,
                                  exclude={tank})
    assert expected is not None

    m = {"z0": 0.0}
    ctx = _rig_ctx(stage, [tank])
    n_seated, n_dropped = qf._settle_foundation_roof_plant(ctx, m)

    assert (n_seated, n_dropped) == (1, 0)
    assert tank not in ctx["roof_plant"], "not removed from roof_plant"
    assert tank not in ctx["loose"], "must not be a rigid body"
    assert tank in ctx["static_extra"]

    pr = stage.GetPrimAtPath(tank)
    assert pr.IsValid() and pr.IsActive(), "must not be deactivated"

    lo_z, _hi_z = _world_z_range(stage, tank)
    # the translate alone lands the base EXACTLY on `expected`; the idle
    # tip that follows (up to `B_ROOF_PLANT_TIP_DEG` about the box's own
    # pivot) can then swing a different corner to the bottom, moving the
    # bbox min by up to roughly half the box's own diagonal times
    # sin(tip) -- ~0.14 m for this 1.6 x 1.6 x 2.0 box at 7 deg -- so the
    # gate here is a band, not an exact match (`test_sweep_roof_props_
    # small_drop_...` makes the same allowance for the same reason).
    assert abs(lo_z - expected) < 0.2, (lo_z, expected)
    # landed close to the tilted deck (~10 m up), not left ~1 m up where it
    # started nor dropped through it to grade
    assert 9.0 < lo_z < 11.0, (lo_z, expected)

    angle = _rotation_angle_deg(stage, tank)
    assert angle <= qf.B_ROOF_PLANT_TIP_DEG + 1e-6, \
        "a prop that found its deck should get the small idle tip, not a bury roll"


def test_settle_foundation_roof_plant_drops_to_grade_past_the_up_threshold():
    """`overturn`'s range (60-90 deg) puts `nz = cos(60..90) = 0.50..0.0`,
    below `ROOF_PROP_UP_THRESHOLD` (0.72) -- the roof the tank rode onto no
    longer faces up at all, so there is no "resting on the deck" any more.
    THE BUG THIS PINS: `_b_settle_roof_plant`'s generic path still hands
    this to the settle regardless, which is exactly how a real `OV`
    archetype measured props scattered 0.7-1.35 m above grade with no
    support at all in the exported file. This must fall back to
    `_a_bury_props`'s "no surviving deck" path instead: static, tipped,
    landed near grade."""
    stage, root = _empty_stage()
    tilt = 80.0     # nz = cos(80) ~ 0.174 -- well past the threshold
    deck = root + "/toppled_roof"
    _stage_box(stage, deck, 0.0, 0.0, 10.0, 4.0, 3.0, 0.3, tilt_deg=tilt)
    tank = root + "/tank_b0_1"
    _stage_box(stage, tank, 0.0, 0.0, 12.5, 1.6, 1.6, 2.0)

    m = {"z0": 0.0}
    ctx = _rig_ctx(stage, [tank])
    n_seated, n_dropped = qf._settle_foundation_roof_plant(ctx, m)

    assert (n_seated, n_dropped) == (0, 1)
    assert tank not in ctx["roof_plant"], "not removed from roof_plant"
    assert tank not in ctx["loose"], "must not be a rigid body (static bury, not settle)"
    assert tank in ctx["static_extra"]

    pr = stage.GetPrimAtPath(tank)
    assert pr.IsValid() and pr.IsActive(), "keep=1.0 -- never silently deactivated"

    lo_z, _hi_z = _world_z_range(stage, tank)
    # dropped to grade, not left at its post-transform height (~11.7 m up)
    assert lo_z < 3.0, ("did not drop toward grade", lo_z)
    assert lo_z > m["z0"] - 2.5, ("dropped through the ground", lo_z)

    angle = _rotation_angle_deg(stage, tank)
    assert angle >= 10.0, "a bury-dressed prop should get a real tip, not stand upright"


def test_settle_foundation_roof_plant_props_do_not_report_each_other_as_support():
    """Two roof-plant props side by side with NOTHING real under either of
    them: `exclude_paths` must cover every path still being resolved this
    call, not just the one currently being placed -- otherwise the first
    prop resolved could read the SECOND, not-yet-moved prop's own geometry
    as "the deck," a false floor exactly like `roof_plant_seat_probe.py`'s
    own note about a tank's legs and barrel (a different mesh each) having
    to stay associated so one never reads as the other's support."""
    stage, root = _empty_stage()
    tank1 = root + "/tank_b0_1"
    tank2 = root + "/tank_b0_2"
    _stage_box(stage, tank1, 0.0, 0.0, 10.0, 1.6, 1.6, 2.0)
    _stage_box(stage, tank2, 2.0, 0.0, 10.0, 1.6, 1.6, 2.0)

    m = {"z0": 0.0}
    ctx = _rig_ctx(stage, [tank1, tank2])
    n_seated, n_dropped = qf._settle_foundation_roof_plant(ctx, m)

    assert n_seated == 0 and n_dropped == 2, \
        "with nothing real under either tank, both must fall back to grade"
    for t in (tank1, tank2):
        assert t not in ctx["roof_plant"]
        lo_z, _ = _world_z_range(stage, t)
        assert lo_z < 5.0, (t, lo_z)


def test_settle_foundation_roof_plant_empty_is_a_noop():
    stage, root = _empty_stage()
    ctx = _rig_ctx(stage, [])
    assert qf._settle_foundation_roof_plant(ctx, {"z0": 0.0}) == (0, 0)
    assert ctx["static_extra"] == [] and ctx["loose"] == []


if __name__ == "__main__":
    import inspect as _inspect
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        marks = getattr(fn, "pytestmark", [])
        if marks:
            continue
        if _inspect.signature(fn).parameters:
            continue
        fn()
        print("ok  " + name)
