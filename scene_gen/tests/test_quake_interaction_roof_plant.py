#!/usr/bin/env python3
"""test_quake_interaction_roof_plant.py — does the LIVE `lean_on`/
`collapse_onto` building-pair interaction (`quake._d_interactions` /
`quake._d_live_lean`) stop handing its rooftop plant (tanks, AC units) to a
shared physics settle it can starve, offline, against a synthetic stage?

    python3 scene_gen/tests/test_quake_interaction_roof_plant.py
    pytest -q scene_gen/tests/test_quake_interaction_roof_plant.py

WHY THIS EXISTS
---------------
`eq500_v4`'s scene log (`~/scorch_previews/eq500_v4/eq500_v4.log`, lines
~1978-1991): the live building-pair path dropped 8 roof-plant props into one
settle shared with 155 rigid bodies (the pair's own fracture debris). That
settle ran to its hard-coded 1800-step cap with 1 body still moving --
`tank_ix1_2`, a water tank frozen mid-flight and exported floating over a
destroyed building.

Investigation (see `quake._d_settle_roof_plant_now`'s own docstring) found
the same shape of bug `test_roof_plant_kit.py` already pinned for the
FOUNDATION family (`r_tilt_severe`/`r_overturn`): the whole-body rigid
transform already carries the roof plant onto whatever is left of the roof
BEFORE `quake_flow._b_settle_roof_plant`'s generic end-of-recipe pass hands
it to `ctx["loose"]`, so the settle is never actually needed -- a geometric
resolver (`quake_collapse._deck_support_z` to seat it, `quake_flow.
_a_bury_props` to drop it to grade when the deck no longer faces up) can
answer deterministically, same as `quake_flow._settle_foundation_roof_plant`
already does for `tilt_severe`/`overturn`. `quake._d_settle_roof_plant_now`
reuses that exact function (now taking an optional `label` so the two
call sites' notes read differently) instead of duplicating its logic, and
is called from `quake._d_live_lean` right after `qf.wreck_building` returns
-- BEFORE `_d_interactions` ever collects `res["loose"]` into the shared
settle's `loose_all`.

This file pins that new call (`quake._d_settle_roof_plant_now`): a dropped
tank ends up EITHER seated on real support OR buried at grade, is pulled out
of `loose`/`velocity` without disturbing any OTHER (unrelated) loose body in
the same `res`, and is never left active-and-loose for the shared settle to
gamble on. It also pins the two OTHER halves of the fix at the
`_d_interactions` settle call site itself (`converge=True`, `rest_v2=True`,
and a `fire_bake.deactivate_airborne` sweep afterward as the net for
whatever fracture debris still doesn't converge) by source inspection --
the same `inspect.getsource` + substring idiom
`test_quake_flow_rubble_routing.py` already uses to pin a call site's exact
kwargs, chosen here because building a real interacting building PAIR
end-to-end (kit modules, fracture, a real shared PhysX settle) needs Isaac
Sim, not a bare `pxr` stage.

WHAT IT CANNOT SEE: whether a REAL scene's shared settle, with real fracture
debris alongside the (now correctly excluded) roof plant, converges any
better in wall-clock terms, and whether the corrected `eq500_v4` re-run
actually shows `tank_ix1_2` seated instead of floating. That needs a
re-render.
"""
import inspect
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import quake as q                  # noqa: E402
from disaster import quake_collapse as qc        # noqa: E402
from disaster import quake_flow as qf            # noqa: E402


# ---------------------------------------------------------------------------
# helpers -- same shapes `test_roof_plant_kit.py` already validated, extended
# with an unrelated "other loose body" so a test here can prove the fix
# touches ONLY the roof plant, never the pair's own fracture debris.
# ---------------------------------------------------------------------------
def _empty_stage(root="/World/b0"):
    from pxr import Sdf, Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, Sdf.Path(root))
    return stage, root


def _stage_box(stage, path, cx, cy, cz, sx, sy, sz, tilt_deg=0.0):
    """A box mesh, optionally rigidly tilted about the X axis BEFORE being
    placed at (cx, cy, cz) -- baked into the points, same convention
    `test_roof_plant_kit.py::_stage_box` uses. A rigid rotation cannot flip
    a face's winding, so the top face's world normal is
    (0, -sin(tilt), cos(tilt)) -- `nz = cos(tilt_deg)`, letting a test pick
    a tilt on either side of `quake_collapse.ROOF_PROP_UP_THRESHOLD`
    (0.72, ~44 deg) on purpose."""
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


def _world_z_range(stage, path):
    from pxr import Usd, UsdGeom
    pr = stage.GetPrimAtPath(path)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(pr).ComputeAlignedRange()
    return r.GetMin()[2], r.GetMax()[2]


def _rig_res(stage, plant, other_loose=(), fixed=None, seed=11,
             parent="/World/b0", mass_z0=0.0, mass="main"):
    """A `res` dict shaped exactly like what `qf.wreck_building` returns to
    `quake._d_live_lean` -- the same keys `_d_settle_roof_plant_now` reads
    and mutates (`loose`, `velocity`, `roof_plant`, `roof_fixed`,
    `roof_plant_mass`, `info.masses`, `static_extra`, `notes`). Every loose
    path (plant AND the unrelated debris) starts with a velocity entry, the
    same as a real `_b_settle_roof_plant`/recipe pass would leave."""
    loose = list(plant) + list(other_loose)
    return {"stage": stage, "parent": parent, "rng": random.Random(seed),
            "roof_plant": list(plant), "roof_fixed": list(fixed or []),
            "roof_plant_mass": mass, "loose": loose, "static_extra": [],
            "notes": [], "velocity": {p: (1.0, 0.0, -1.0) for p in loose},
            "info": {"masses": {mass: {"z0": mass_z0}}}}


# ---------------------------------------------------------------------------
# `quake._d_settle_roof_plant_now`
# ---------------------------------------------------------------------------
def test_d_settle_roof_plant_now_seats_a_dropped_tank_and_leaves_debris_loose():
    """THE BUG THIS PINS (eq500_v4, `tank_ix1_2`): the live interaction path
    used to hand EVERY roof-plant path to the shared settle's `loose_all`
    alongside the pair's own fracture debris. This must pull ONLY the tank
    back out (leaving the unrelated debris fragment exactly where it was)
    and seat it on the tilted deck it already rode onto -- never rely on
    the shared settle finding it in time."""
    stage, root = _empty_stage()
    tilt = 20.0
    deck = root + "/roof_deck"
    _stage_box(stage, deck, 0.0, 0.0, 10.0, 4.0, 3.0, 0.3, tilt_deg=tilt)
    tank = root + "/tank_ix1_2"
    tank_cz = 11.33
    _stage_box(stage, tank, 0.0, 0.0, tank_cz, 1.6, 1.6, 2.0)
    debris = root + "/brk_ix1_bld_commercial_mid_ground_1_1/frag_018"
    _stage_box(stage, debris, 5.0, 5.0, 1.0, 0.4, 0.4, 0.4)

    base_before, _ = _world_z_range(stage, tank)
    expected = qc._deck_support_z(stage, root, cx=0.0, cy=0.0, half_w=0.92,
                                  half_d=0.92, z_ceiling=base_before,
                                  exclude={tank})
    assert expected is not None

    res = _rig_res(stage, [tank], other_loose=[debris])
    n_seated, n_dropped = q._d_settle_roof_plant_now(res)

    assert (n_seated, n_dropped) == (1, 0)
    assert tank not in res["loose"], "must be pulled out of the shared settle"
    assert debris in res["loose"], "unrelated fracture debris must be untouched"
    assert tank not in res["roof_plant"], "not removed from roof_plant"
    assert tank not in res["velocity"], "stale velocity entry must be dropped"
    assert debris in res["velocity"], "unrelated velocity entry must survive"
    assert tank in res["static_extra"]
    assert any("roof_plant(interaction)" in n for n in res["notes"]), res["notes"]

    pr = stage.GetPrimAtPath(tank)
    assert pr.IsValid() and pr.IsActive(), "must not be deactivated"
    lo_z, _hi_z = _world_z_range(stage, tank)
    # same band `test_roof_plant_kit.py` allows for the translate + idle tip
    assert abs(lo_z - expected) < 0.2, (lo_z, expected)


def test_d_settle_roof_plant_now_drops_to_grade_when_no_deck_survives():
    """The building went past `ROOF_PROP_UP_THRESHOLD` (a `collapse_onto`'s
    roof no longer faces up at all): the tank must fall back to
    `_a_bury_props`'s deterministic grade-drop, static and tipped -- NEVER
    handed to the settle with no real target, which is exactly how
    `tank_ix1_2` froze mid-flight."""
    stage, root = _empty_stage()
    tilt = 80.0     # nz = cos(80) ~ 0.174, well past the 0.72 threshold
    deck = root + "/toppled_roof"
    _stage_box(stage, deck, 0.0, 0.0, 10.0, 4.0, 3.0, 0.3, tilt_deg=tilt)
    tank = root + "/tank_ix2_1"
    _stage_box(stage, tank, 0.0, 0.0, 12.5, 1.6, 1.6, 2.0)

    res = _rig_res(stage, [tank])
    n_seated, n_dropped = q._d_settle_roof_plant_now(res)

    assert (n_seated, n_dropped) == (0, 1)
    assert tank not in res["loose"], "must not ship active-and-loose"
    assert tank not in res["velocity"]
    assert tank in res["static_extra"]

    pr = stage.GetPrimAtPath(tank)
    assert pr.IsValid() and pr.IsActive(), "keep=1.0 -- never silently deactivated"
    lo_z, _hi_z = _world_z_range(stage, tank)
    assert lo_z < 3.0, ("did not drop toward grade", lo_z)
    assert lo_z > -2.5, ("dropped through the ground", lo_z)


def test_d_settle_roof_plant_now_is_a_noop_without_roof_plant():
    """A building on the interaction path that dressed no roof plant at all
    (too short for `dress_roof`'s 3-storey gate) must leave `loose` and
    `velocity` completely alone -- no crash, no spurious note."""
    stage, root = _empty_stage()
    debris = root + "/frag_ix3_004"
    _stage_box(stage, debris, 0.0, 0.0, 0.5, 0.3, 0.3, 0.3)
    res = _rig_res(stage, [], other_loose=[debris])

    assert q._d_settle_roof_plant_now(res) == (0, 0)
    assert res["loose"] == [debris]
    assert debris in res["velocity"]
    assert res["static_extra"] == []
    assert res["notes"] == []


def test_d_settle_roof_plant_now_ignores_roof_fixed_the_same_way():
    """`roof_fixed` (vents, bulkheads -- fixtures dressed alongside the
    tanks/AC units) rides the same `_settle_foundation_roof_plant` path as
    `roof_plant`; `_d_settle_roof_plant_now` must pull it out of `loose`
    too, not just the `roof_plant` list."""
    stage, root = _empty_stage()
    deck = root + "/roof_deck"
    _stage_box(stage, deck, 0.0, 0.0, 10.0, 4.0, 3.0, 0.3, tilt_deg=0.0)
    vent = root + "/vent_ix4_1"
    _stage_box(stage, vent, 0.0, 0.0, 10.32, 0.6, 0.6, 0.6)

    res = _rig_res(stage, [], fixed=[vent])
    res["loose"] = [vent]
    res["velocity"] = {vent: (0.0, 0.0, -1.0)}

    n_seated, n_dropped = q._d_settle_roof_plant_now(res)
    assert (n_seated, n_dropped) == (1, 0)
    assert vent not in res["loose"]
    assert vent not in res["velocity"]
    assert vent not in res["roof_fixed"]
    assert vent in res["static_extra"]


# ---------------------------------------------------------------------------
# the `_d_interactions` settle call site itself
# ---------------------------------------------------------------------------
def test_d_interactions_settle_call_converges_and_sweeps_for_airborne_debris():
    """Pins the other two-thirds of the fix at the exact call site the
    `eq500_v4` log traced this to -- `_d_interactions`'s single
    `settle.run(...)` for the shared pair settle:

      * `converge=True` and `rest_v2=True`, so `settle_steps` (1800,
        unchanged) is a TARGET the throw phase may run past, measured with
        the points-based / net-travel rest check, instead of a hard cap
        baked against regardless of what is still in flight.
      * a `fire_bake.deactivate_airborne` sweep afterward, scoped to this
        call's own `quake_interact` holder -- the bake pipeline's own
        2026-08-31 precedent for "judge every candidate against the real
        geometry left standing and switch off whatever has none" -- as the
        net for any fracture fragment that still does not converge.
      * `strict` is NOT set at this call site -- a bad interaction settle
        must keep warning, never raise, matching the task's instruction not
        to turn this one on.

    Building a real interacting PAIR end-to-end (kit modules, fracture, a
    real shared PhysX settle) needs Isaac Sim, which is why this is a
    source pin rather than a run of `_d_interactions` itself -- the same
    `inspect.getsource` idiom `test_quake_flow_rubble_routing.py` already
    uses to pin a call site's exact kwargs."""
    src = inspect.getsource(q._d_interactions)
    assert "settle.run(" in src
    assert "converge=True" in src
    assert "rest_v2=True" in src
    assert "deactivate_airborne" in src
    assert "strict=True" not in src
    assert "strict=SETTLE_STRICT" not in src


# ---------------------------------------------------------------------------
# the shared resolver's new `label` kwarg must not disturb its existing
# callers (`r_tilt_severe`/`r_overturn`, and `test_roof_plant_kit.py`)
# ---------------------------------------------------------------------------
def test_settle_foundation_roof_plant_label_defaults_to_foundation():
    stage, root = _empty_stage()
    tank = root + "/tank_b0_1"
    _stage_box(stage, tank, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0)
    ctx = {"stage": stage, "parent": root, "rng": random.Random(1),
           "roof_plant": [tank], "roof_fixed": [], "loose": [],
           "static_extra": [], "notes": []}

    qf._settle_foundation_roof_plant(ctx, {"z0": 0.0})

    assert any("roof_plant(foundation)" in n for n in ctx["notes"]), ctx["notes"]


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
