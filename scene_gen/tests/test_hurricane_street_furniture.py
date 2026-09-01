#!/usr/bin/env python3
"""test_hurricane_street_furniture.py — pins `disaster.street_furniture`
(new module, this stream, 2026-09-01): "I don't see fallen over [signs],
stop signs, etc. Those should also be thrown away/uprooted" (user, on the
live 500 m hurricane plate).

THE REPORTED DEFECT. The scene's own tally places 45 `sign`, 37
`streetlight`, 23 `trash_can` and 19 `fire_hydrant` prims
(`suburb_scene.build_frontage`), and NOTHING before this file read any of
them — not `washaway.py`, not `hurricane.py`, not the launcher. They stood
pristine in a Cat-3 wind field.

WHAT THIS FILE VERIFIES, OFFLINE, BARE `pxr` (usd-core is importable from
host `python3` — this file never touches Isaac Sim / `SimulationApp`,
same guarantee `test_hurricane_fences.py` and `test_hurricane_fence_axis.py`
already carry):

  1. THE PURE DECISION FUNCTION, `street_furniture_specs`, needs no `pxr`
     at all and is tested first, in isolation, against synthetic
     depth/wind callables:

       (a) `fire_hydrant` is the DELIBERATE NEGATIVE CASE — it must never
           move, at any intensity up to the theoretical maximum, proving
           this pass DISCRIMINATES rather than flattening everything.
       (b) `trash_can` is the opposite extreme — "essentially always
           displaced" at realistic hurricane intensities.
       (c) `streetlight` "mostly survives" even at a real Level-3 gust.
       (d) `sign` shows a genuine MIX of outcomes at Level 2, not
           saturated to one action.
       (e) every category's own "how much of it is untouched" fraction
           moves the RIGHT WAY (down) as `wind_intensity_fn` rises — the
           "scale the probabilities with local wind intensity" requirement,
           checked as an actual monotonic trend, not asserted by
           construction.
       (f) surge DEPTH alone (zero wind) removes the LIGHT categories
           (`sign`, `trash_can`) past their own threshold and leaves the
           heavy ones (`streetlight`, `fire_hydrant`) untouched — "a bin
           floats" is a cause distinct from wind, and it does not apply to
           an anchored pole or a bolted hydrant.
       (g) an unrecognised category always "stands" — refusing to guess,
           `washaway.raft_kind_weights(None)`'s own convention.

  2. THE STAGE-TOUCHING APPLY FUNCTION, `apply_street_furniture_pose`,
     against a synthetic prim built to carry the EXACT geometry trap
     `washaway.apply_fence_pose` already paid for twice (see that
     function's own docstring, and this file's `_build_item_prim`): the
     prim's authored `translate` is the true ground PLUS a nonzero
     anchor offset, reproducing `apply_placements` folding an
     anchor->centroid offset into the translation. A correct seat must
     read the item's own PRE-pose lowest world point (points-based,
     `bake.world_point_bounds` — never `UsdGeom.BBoxCache`), not `t[2]`.

       (a) a "flat" item ends up with a SMALL Z extent (lying down) and a
           footprint that reaches toward the requested fall azimuth, not
           away from it, at more than one azimuth (proof the hinge
           construction's sign is right, not a coincidence of one
           bearing).
       (b) a "leaning" item's Z extent stays close to its full standing
           height — proof "leaning" is genuinely partial, not a second
           spelling of "flat".
       (c) the flattened item's LOWEST WORLD POINT lands at the PRE-pose
           measured ground, not at the raw `translate.z`.
       (d) THE CONTROL, in `__main__` only: a local re-implementation that
           seats from `t[2]` directly (the anti-pattern this module's own
           docstring warns against) is run against the SAME synthetic prim
           and is EXPECTED TO FAIL check (c) — a test whose control does
           not fail is not proof this test pins anything.
       (e) "gone" deactivates the prim (`prim.IsActive()` false) and
           reports it changed something; "stands" changes nothing.

USAGE
    python3 scene_gen/tests/test_hurricane_street_furniture.py
    pytest -q scene_gen/tests/test_hurricane_street_furniture.py
"""

import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import street_furniture as sf                # noqa: E402

try:
    from pxr import Gf, Usd, UsdGeom                        # noqa: E402
    from disaster import bake                                # noqa: E402
    HAVE_USD = True
except Exception:                                            # pragma: no cover
    HAVE_USD = False

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


# ---------------------------------------------------------------------------
# helpers shared by the pure-function tests
# ---------------------------------------------------------------------------

def _flat_intensity(i):
    def wind_intensity_fn(x, y):
        return i
    return wind_intensity_fn


def _zero_depth(x, y):
    return 0.0


def _flat_bearing(deg):
    def wind_bearing_fn(x, y):
        return deg
    return wind_bearing_fn


def _run(category, i, n=4000, seed=7, depth_fn=_zero_depth):
    """`n` synthetic items of one `category`, all at the same field
    intensity `i` (spatial position does not matter to any callable used
    here — only the per-item `rng` draw does), decided once. Returns the
    list of decision dicts."""
    items = [(float(k), 0.0, category) for k in range(n)]
    rng = random.Random(seed)
    return sf.street_furniture_specs(items, depth_fn, _flat_bearing(0.0),
                                     _flat_intensity(i), rng)


def _moved_fraction(decisions):
    n = len(decisions)
    if n == 0:
        return 0.0
    return sum(1 for d in decisions if d["action"] != "stands") / float(n)


# ---------------------------------------------------------------------------
# 1(a)-1(d): per-class probability behaviour
# ---------------------------------------------------------------------------

@strict
def test_fire_hydrant_never_moves_even_at_maximum_intensity():
    """THE DELIBERATE NEGATIVE CASE. `wind_intensity_fn` clamped to its own
    maximum (1.0) at every sampled point, across many independent draws --
    if the pass ever discriminates incorrectly, this is where it would
    show up first."""
    out = _run("fire_hydrant", 1.0, n=8000)
    n_moved = sum(1 for d in out if d["action"] != "stands")
    check("0 of 8000 hydrants move at i=1.0 (the theoretical maximum)",
         n_moved == 0, n_moved)


def _wind_bearing_and_depth_never_called_hydrant():
    """Depth/wind past every real category's own threshold, still 0 moves:
    the hydrant ladder's onset is unreachable BY CONSTRUCTION (see
    `street_furniture._CUTS["fire_hydrant"]`'s own docstring), not merely
    unlikely."""
    def deep(x, y):
        return 5.0   # far past any other category's own depth_gone_m
    out = _run("fire_hydrant", 1.0, n=2000, depth_fn=deep)
    n_moved = sum(1 for d in out if d["action"] != "stands")
    check("hydrants ignore depth entirely (no `_DEPTH_GONE_M` entry)",
         n_moved == 0, n_moved)


@strict
def test_fire_hydrant_ignores_depth_too():
    _wind_bearing_and_depth_never_called_hydrant()


@strict
def test_trash_can_essentially_always_displaced():
    """"essentially always displaced in a Cat-3" -- at both shipped
    hurricane levels (`suburb_hurricane_500_l2.yaml` i=0.55,
    `..._l3.yaml` i=0.70) fewer than 5% of bins should still be standing,
    and it should get WORSE, not better, at the higher level."""
    l2 = _run("trash_can", 0.55, n=6000)
    l3 = _run("trash_can", 0.70, n=6000)
    stands_l2 = sum(1 for d in l2 if d["action"] == "stands") / len(l2)
    stands_l3 = sum(1 for d in l3 if d["action"] == "stands") / len(l3)
    check("fewer than 5% of bins stand at L2 (i=0.55)", stands_l2 < 0.05,
         stands_l2)
    check("fewer than 5% of bins stand at L3 (i=0.70)", stands_l3 < 0.05,
         stands_l3)
    check("L3 leaves at least as few bins standing as L2 (worse storm, "
         "not fewer displaced)", stands_l3 <= stands_l2 + 1e-9,
         (stands_l2, stands_l3))
    # Both "flat" (relocated, still modelled) and "gone" (removed) must
    # actually appear -- a bin population that is 100% one or the other
    # would mean this module only implements half the design brief
    # ("mostly gone from their placed position ... or removed entirely").
    kinds = {d["action"] for d in l2} | {d["action"] for d in l3}
    check("both 'flat' and 'gone' occur among displaced bins",
         "flat" in kinds and "gone" in kinds, kinds)


@strict
def test_streetlight_mostly_survives():
    """"Mostly SURVIVES; a minority lean or fail at the base" -- at BOTH
    shipped levels a clear majority must still stand, and "flat" (full
    base failure) must stay rare relative to "leaning" at a real L3 (it is
    reserved for the extreme tail of the per-item resistance/jitter
    draw -- see `_CUTS["streetlight"]`'s own docstring)."""
    l2 = _run("streetlight", 0.55, n=6000)
    l3 = _run("streetlight", 0.70, n=6000)
    stands_l2 = sum(1 for d in l2 if d["action"] == "stands") / len(l2)
    stands_l3 = sum(1 for d in l3 if d["action"] == "stands") / len(l3)
    flat_l3 = sum(1 for d in l3 if d["action"] == "flat") / len(l3)
    leaning_l3 = sum(1 for d in l3 if d["action"] == "leaning") / len(l3)
    check("streetlights essentially untouched at L2 (>= 95% stand)",
         stands_l2 >= 0.95, stands_l2)
    check("a clear majority of streetlights still stand at L3 (> 50%)",
         stands_l3 > 0.50, stands_l3)
    check("'flat' (base failure) stays a minority of 'leaning' at L3",
         flat_l3 < leaning_l3, (flat_l3, leaning_l3))
    check("no streetlight is ever 'gone' (a failed pole lies where it "
         "fell, it is not carried off)",
         all(d["action"] != "gone" for d in l2 + l3), None)


@strict
def test_sign_shows_a_genuine_mix_at_l2():
    """The lightest, most commonly lost item on the plate: at a real L2
    gust it should show a MIX of outcomes, not be saturated to one
    action -- both a majority damaged (this is "the most commonly lost"
    category) and a real spread across at least 3 of the 4 possible
    actions."""
    out = _run("sign", 0.55, n=8000)
    from collections import Counter
    c = Counter(d["action"] for d in out)
    n = len(out)
    check("signs show at least 3 distinct actions at L2",
         len(c) >= 3, dict(c))
    stands_frac = c.get("stands", 0) / n
    check("a majority of signs are damaged in some way at L2 (< 50% stand)",
         stands_frac < 0.50, stands_frac)
    # `gone` DROPPED FROM THIS LIST, 2026-09-01, when the sign ladder's
    # `flat` ceiling was raised 56 -> 78 m/s. At the reference plate's real
    # intensity band the old cut sent 62-84% of signs to `gone`, i.e. most
    # of the plate's 45 signs simply vanished — defensible physics, wrong
    # for this dataset: the review asked for signs "thrown away/uprooted" so
    # that they can be SEEN, and a deleted sign is invisible to a detector
    # while an uprooted one lying on the verge is a findable object. `gone`
    # is still reachable (8% at i=0.77) but is no longer expected at L2, so
    # asserting it here would pin the behaviour that was deliberately
    # removed. `leaning` and `flat` still carry the "genuine mix" claim.
    for action in ("leaning", "flat"):
        check("action '{0}' has real (>1%) representation at L2"
             .format(action), c.get(action, 0) / n > 0.01,
             c.get(action, 0) / n)
    check("`gone` is still REACHABLE for a sign at severe wind, just not "
         "at L2 (it is the rung the 78 m/s ceiling pushed up, not deleted)",
         any(d["action"] == "gone" for d in _run("sign", 0.85, n=4000)))


# ---------------------------------------------------------------------------
# 1(e): scaling with intensity
# ---------------------------------------------------------------------------

@strict
def test_damage_fraction_rises_with_intensity_for_every_moving_category():
    """"Scale the probabilities with local wind intensity" -- checked as an
    ACTUAL MONOTONIC TREND across a spread of intensities, not merely
    asserted. `fire_hydrant` is excluded on purpose: it has nothing to
    scale (see `test_fire_hydrant_never_moves_even_at_maximum_intensity`),
    and a flat 0% curve is not a rising one."""
    levels = (0.10, 0.30, 0.50, 0.70, 0.90)
    for category in ("sign", "trash_can", "streetlight"):
        fracs = [_moved_fraction(_run(category, i, n=5000, seed=11))
                 for i in levels]
        # Non-decreasing across the whole sweep (allow a hair of statistical
        # noise between adjacent equal-ish points; 5000 draws keeps that
        # small) AND a real rise from the lowest to the highest level.
        non_decreasing = all(fracs[k + 1] >= fracs[k] - 0.02
                             for k in range(len(fracs) - 1))
        check("[{0}] damaged fraction is non-decreasing across rising "
             "intensity {1}".format(category, levels),
             non_decreasing, fracs)
        check("[{0}] damaged fraction at the highest sampled intensity is "
             "clearly above the lowest".format(category),
             fracs[-1] - fracs[0] > 0.20, fracs)


# ---------------------------------------------------------------------------
# 1(f): depth as a separate cause, light items only
# ---------------------------------------------------------------------------

@strict
def test_depth_alone_removes_only_the_light_categories():
    """Zero wind everywhere; deep standing water (1.0 m, past both
    `sign`'s 0.6 m and `trash_can`'s 0.3 m own thresholds) is the ONLY
    thing that could move anything. "A bin floats" is the design brief's
    own phrase for exactly this."""
    def deep(x, y):
        return 1.0

    def zero_wind(x, y):
        return 0.0
    items = [(0.0, 0.0, cat) for cat in
             ("sign", "trash_can", "streetlight", "fire_hydrant")]
    rng = random.Random(3)
    out = sf.street_furniture_specs(items, deep, _flat_bearing(0.0),
                                    zero_wind, rng)
    by_cat = {d["category"]: d["action"] for d in out}
    check("a sign in deep water with zero wind is 'gone'",
         by_cat["sign"] == "gone", by_cat["sign"])
    check("a trash can in deep water with zero wind is 'gone'",
         by_cat["trash_can"] == "gone", by_cat["trash_can"])
    check("a streetlight in deep water with zero wind still stands "
         "(no depth cause applies to it)",
         by_cat["streetlight"] == "stands", by_cat["streetlight"])
    check("a fire hydrant in deep water with zero wind still stands",
         by_cat["fire_hydrant"] == "stands", by_cat["fire_hydrant"])

    # And below each light item's own threshold, depth alone must NOT act
    # -- proves the check is a genuine THRESHOLD, not "any water at all".
    def shallow(x, y):
        return 0.05
    out_shallow = sf.street_furniture_specs(items, shallow,
                                            _flat_bearing(0.0), zero_wind,
                                            rng)
    by_cat_shallow = {d["category"]: d["action"] for d in out_shallow}
    check("a sign in 5 cm of water with zero wind still stands",
         by_cat_shallow["sign"] == "stands", by_cat_shallow["sign"])
    check("a trash can in 5 cm of water with zero wind still stands",
         by_cat_shallow["trash_can"] == "stands",
         by_cat_shallow["trash_can"])


# ---------------------------------------------------------------------------
# 1(g): unknown category
# ---------------------------------------------------------------------------

@strict
def test_unknown_category_always_stands():
    """Refusing to guess is the same convention `washaway.
    raft_kind_weights(None)` / `fence_wind_threshold([], ...)` use
    elsewhere: a category this file has no ladder for is left alone,
    however extreme the inputs."""
    # `mailbox` USED TO BE THE EXAMPLE HERE and stopped being valid the
    # moment a mailbox ladder was added (2026-09-01, "same for mail boxes").
    # Picking a real-but-unmodelled category is the point of the test, so it
    # now uses one that is genuinely not in `_CUTS` and is unlikely ever to
    # be: a category this module has no ladder for must be left ALONE,
    # however hostile the inputs.
    items = [(0.0, 0.0, "park_bench")] * 200
    assert "park_bench" not in sf._CUTS, (
        "park_bench gained a ladder; pick another unmodelled category or "
        "this test silently stops testing anything")

    def hostile_depth(x, y):
        return 99.0

    def hostile_wind(x, y):
        return 1.0
    rng = random.Random(9)
    out = sf.street_furniture_specs(items, hostile_depth, _flat_bearing(0.0),
                                    hostile_wind, rng)
    check("an unrecognised category always 'stands', even under the most "
         "extreme depth/wind inputs this function accepts",
         all(d["action"] == "stands" for d in out),
         {d["action"] for d in out})


# ---------------------------------------------------------------------------
# 2: the stage-touching apply function
# ---------------------------------------------------------------------------

# A generic vertical post shape -- thin in X/Y, tall in Z -- close enough to
# every real category (sign, streetlight, trash can, hydrant are all
# roughly post/cylinder-shaped when reduced to "does it end up flat or
# still standing") that one synthetic shape covers all of them; nothing
# below tests a category-specific mesh silhouette.
THICK_M = 0.15
HEIGHT_M = 4.0
# The KEY GEOMETRY TRAP: the mesh's own local origin sits ANCHOR_OFFSET_M
# above its physical base (mimicking `apply_placements` folding a
# bbox-center anchor offset into the authored translate -- see
# `washaway.apply_fence_pose`'s own docstring for the real bug this
# reproduces). So the mesh's local Z range is
# `[-ANCHOR_OFFSET_M, HEIGHT_M - ANCHOR_OFFSET_M]`, NOT `[0, HEIGHT_M]`.
ANCHOR_OFFSET_M = 0.9
TRUE_GROUND_Z = 12.5   # an arbitrary, deliberately-not-zero world ground


def _build_item_prim(stage, prim_path, x_m=0.0, y_m=0.0, yaw_deg=0.0,
                     ssf=1.0):
    """Reproduces what `apply_placements` puts on stage for one
    street-furniture item: an Xform holder with
    translate -> rotateXYZ(0, 0, yaw) -> scale, translate.z authored as
    `TRUE_GROUND_Z + ANCHOR_OFFSET_M` (the ground PLUS the anchor offset,
    exactly the trap `apply_street_furniture_pose` must not fall into), a
    child Mesh box whose local Z range does NOT start at the origin."""
    UsdGeom.Xform.Define(stage, prim_path)
    prim = stage.GetPrimAtPath(prim_path)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x_m) * ssf, float(y_m) * ssf,
                                     (TRUE_GROUND_Z + ANCHOR_OFFSET_M) * ssf))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))
    xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))

    mesh = UsdGeom.Mesh.Define(stage, prim_path + "/geo")
    h = THICK_M / 2.0
    z0, z1 = -ANCHOR_OFFSET_M, HEIGHT_M - ANCHOR_OFFSET_M
    pts = [(-h, -h, z0), (h, -h, z0), (h, h, z0), (-h, h, z0),
           (-h, -h, z1), (h, -h, z1), (h, h, z1), (-h, h, z1)]
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([4] * 6)
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 5, 4,
                                      2, 3, 7, 6, 0, 3, 7, 4, 1, 2, 6, 5])
    lo = [min(q[k] for q in pts) for k in range(3)]
    hi = [max(q[k] for q in pts) for k in range(3)]
    mesh.CreateExtentAttr([Gf.Vec3f(*lo), Gf.Vec3f(*hi)])
    return prim


def _world_bounds(prim, ssf=1.0):
    """Merged world AABB over every Mesh descendant, POINTS-BASED via
    `bake.world_point_bounds` -- never `UsdGeom.BBoxCache`, same discipline
    `test_hurricane_fence_axis.py`'s own `_merged_world_bounds` documents."""
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    lo = [None, None, None]
    hi = [None, None, None]
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        b = bake.world_point_bounds(p, xc)
        if b is None:
            continue
        (bx0, by0, bz0), (bx1, by1, bz1) = b
        for k, (v0, v1) in enumerate(((bx0, bx1), (by0, by1), (bz0, bz1))):
            lo[k] = v0 if lo[k] is None else min(lo[k], v0)
            hi[k] = v1 if hi[k] is None else max(hi[k], v1)
    if lo[0] is None:
        return None
    return (tuple(v / ssf for v in lo), tuple(v / ssf for v in hi))


def _broken_apply_seat_from_translate(stage, prim_path, decision, ssf=1.0):
    """THE CONTROL. Identical to `apply_street_furniture_pose` except the
    ground reference is the raw `t[2]` instead of the item's OWN
    pre-pose measured lowest point -- the exact anti-pattern
    `apply_street_furniture_pose`'s own docstring (point 1) warns against.
    Used ONLY by the `__main__` control block, never by a pytest test, to
    prove this file's seating check actually discriminates."""
    from pxr import Gf, Usd, UsdGeom

    from disaster import bake as _bake

    prim = stage.GetPrimAtPath(prim_path)
    action = decision.get("action")
    if action not in ("flat", "leaning"):
        return False
    xf = UsdGeom.Xformable(prim)
    vals = {}
    for op in xf.GetOrderedXformOps():
        vals[op.GetOpName().split(":")[-1]] = op.Get()
    t = vals.get("translate") or Gf.Vec3d(0.0, 0.0, 0.0)
    sc = vals.get("scale")
    rot = vals.get("rotateXYZ")
    ground_z_stage = float(t[2])          # <-- THE BUG, on purpose.

    import math
    az = math.radians(float(decision.get("azimuth_deg", 0.0)))
    axis = Gf.Vec3d(-math.sin(az), math.cos(az), 0.0)
    lean_rot = Gf.Rotation(axis, float(decision.get("lean_deg", 85.0)))
    place = rot if rot is not None else Gf.Vec3f(0.0, 0.0, 0.0)
    place_rot = (Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), float(place[2]))
                 * Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), float(place[1]))
                 * Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), float(place[0])))
    combined = place_rot * lean_rot
    q = combined.GetQuat()
    im = q.GetImaginary()

    def _apply(tz):
        xf.SetXformOpOrder([])
        xf.AddTranslateOp().Set(Gf.Vec3d(float(t[0]), float(t[1]), tz))
        xf.AddOrientOp().Set(Gf.Quatf(float(q.GetReal()), float(im[0]),
                                      float(im[1]), float(im[2])))
        if sc is not None:
            xf.AddScaleOp().Set(Gf.Vec3f(float(sc[0]), float(sc[1]),
                                         float(sc[2])))

    _apply(float(t[2]))
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    lowest = None
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        b = _bake.world_point_bounds(p, xc)
        if b is None:
            continue
        lowest = b[0][2] if lowest is None else min(lowest, b[0][2])
    if lowest is not None:
        _apply(float(t[2]) + (ground_z_stage - lowest))
    return True


@strict
def test_apply_flat_lands_small_z_and_seats_at_true_ground():
    """(2a)+(2c): a "flat" item ends up lying down (small Z extent) and
    seated at `TRUE_GROUND_Z` -- NOT at the raw authored translate.z
    (`TRUE_GROUND_Z + ANCHOR_OFFSET_M`, which is what a seat-from-translate
    bug would produce -- see `test_apply_flat_seating_uses_measured_ground_
    not_translate` for the direct comparison)."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    for azimuth in (0.0, 90.0, 200.0):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        prim_path = "/World/item_test"
        prim = _build_item_prim(stage, prim_path)

        decision = {"action": "flat", "azimuth_deg": azimuth,
                   "lean_deg": 88.0, "slide_m": 0.0}
        changed = sf.apply_street_furniture_pose(stage, prim_path, decision,
                                                  ssf=1.0)
        check("[az={0}] apply reports it changed the prim".format(azimuth),
             changed is True, changed)

        bounds = _world_bounds(prim)
        check("[az={0}] flattened item has measurable geometry"
             .format(azimuth), bounds is not None, bounds)
        if bounds is None:
            continue
        (lox, loy, loz), (hix, hiy, hiz) = bounds
        z_span = hiz - loz
        check("[az={0}] flattened item's Z extent is small (lying down, "
             "not standing at ~{1} m)".format(azimuth, HEIGHT_M),
             z_span < 1.0, z_span)
        check("[az={0}] flattened item's LOWEST point sits at the TRUE "
             "ground ({1} m), not at translate.z ({2} m)"
             .format(azimuth, TRUE_GROUND_Z,
                    TRUE_GROUND_Z + ANCHOR_OFFSET_M),
             abs(loz - TRUE_GROUND_Z) < 0.02, loz)


@strict
def test_apply_flat_falls_toward_the_requested_azimuth():
    """The hinge sign is right, not a coincidence of one bearing: at
    azimuth 0 the footprint must extend toward +X (away from the base at
    x=0), and at azimuth 180 toward -X -- checked both ways so a sign
    error would be caught rather than passing by symmetry."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    for azimuth, expect_positive_x in ((0.0, True), (180.0, False)):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        prim_path = "/World/item_test"
        prim = _build_item_prim(stage, prim_path, x_m=0.0, y_m=0.0)
        decision = {"action": "flat", "azimuth_deg": azimuth,
                   "lean_deg": 88.0, "slide_m": 0.0}
        sf.apply_street_furniture_pose(stage, prim_path, decision, ssf=1.0)
        bounds = _world_bounds(prim)
        check("[az={0}] measurable geometry".format(azimuth),
             bounds is not None, bounds)
        if bounds is None:
            continue
        (lox, _loy, _loz), (hix, _hiy, _hiz) = bounds
        reach_pos = hix - 0.0     # how far the footprint extends past x=0
        reach_neg = 0.0 - lox
        if expect_positive_x:
            check("[az=0] footprint reaches farther in +X than -X "
                 "(falls TOWARD azimuth 0)", reach_pos > reach_neg,
                 (reach_pos, reach_neg))
        else:
            check("[az=180] footprint reaches farther in -X than +X "
                 "(falls TOWARD azimuth 180)", reach_neg > reach_pos,
                 (reach_pos, reach_neg))


@strict
def test_apply_leaning_stays_close_to_full_height():
    """(2b): "leaning" is genuinely partial -- at a small lean angle the
    item's Z extent must stay close to its full standing height
    (`HEIGHT_M`), clearly distinct from "flat"'s small Z extent."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    prim_path = "/World/item_test"
    prim = _build_item_prim(stage, prim_path)
    decision = {"action": "leaning", "azimuth_deg": 30.0, "lean_deg": 12.0,
               "slide_m": 0.0}
    sf.apply_street_furniture_pose(stage, prim_path, decision, ssf=1.0)
    bounds = _world_bounds(prim)
    check("leaning item has measurable geometry", bounds is not None, bounds)
    if bounds is None:
        return
    (_lox, _loy, loz), (_hix, _hiy, hiz) = bounds
    z_span = hiz - loz
    check("a 12-degree lean keeps most of the item's height vertical "
         "(Z extent > 3.5 m of its {0} m standing height)".format(HEIGHT_M),
         z_span > 3.5, z_span)
    check("a 12-degree lean is still measurably seated at the true ground",
         abs(loz - TRUE_GROUND_Z) < 0.05, loz)


@strict
def test_apply_gone_deactivates_and_stands_is_a_noop():
    """(2e): "gone" deactivates the prim and reports a change; "stands"
    touches nothing and reports no change."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    prim_path = "/World/item_test"
    prim = _build_item_prim(stage, prim_path)
    changed = sf.apply_street_furniture_pose(stage, prim_path,
                                             {"action": "gone"}, ssf=1.0)
    check("'gone' reports a change", changed is True, changed)
    check("'gone' deactivates the prim", prim.IsActive() is False,
         prim.IsActive())

    stage2 = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage2, "/World")
    prim2 = _build_item_prim(stage2, prim_path)
    pre_bounds = _world_bounds(prim2)
    changed2 = sf.apply_street_furniture_pose(stage2, prim_path,
                                              {"action": "stands"}, ssf=1.0)
    post_bounds = _world_bounds(prim2)
    check("'stands' reports no change", changed2 is False, changed2)
    check("'stands' leaves the geometry exactly where it was",
         pre_bounds == post_bounds, (pre_bounds, post_bounds))


@strict
def test_measure_street_item_reads_translate_and_carries_category():
    """`measure_street_item` returns `(x, y, category)` from the prim's own
    translate, scaled by `ssf`, with `category` passed through verbatim --
    it must NOT try to parse `category` out of the prim's name (see its own
    docstring on why `trash_can` specifically breaks that)."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    ssf = 2.5
    prim_path = "/World/trash_can_3_88"
    prim = _build_item_prim(stage, prim_path, x_m=12.0, y_m=-7.5, ssf=ssf)
    x, y, category = sf.measure_street_item(stage, prim, "trash_can",
                                             ssf=ssf)
    check("measured x matches the authored (unscaled) position",
         abs(x - 12.0) < 1e-6, x)
    check("measured y matches the authored (unscaled) position",
         abs(y - (-7.5)) < 1e-6, y)
    check("category is carried through verbatim, not parsed from the name",
         category == "trash_can", category)


if __name__ == "__main__":
    if not HAVE_USD:
        print("pxr is not importable on this interpreter -- cannot run "
             "this file's stage-touching checks (the pure-function checks "
             "still ran above). usd-core should be pip-installed on the "
             "host per this repo's own convention.")

    ok = True
    try:
        test_fire_hydrant_never_moves_even_at_maximum_intensity()
        test_fire_hydrant_ignores_depth_too()
        test_trash_can_essentially_always_displaced()
        test_streetlight_mostly_survives()
        test_sign_shows_a_genuine_mix_at_l2()
        test_damage_fraction_rises_with_intensity_for_every_moving_category()
        test_depth_alone_removes_only_the_light_categories()
        test_unknown_category_always_stands()
        test_apply_flat_lands_small_z_and_seats_at_true_ground()
        test_apply_flat_falls_toward_the_requested_azimuth()
        test_apply_leaning_stays_close_to_full_height()
        test_apply_gone_deactivates_and_stands_is_a_noop()
        test_measure_street_item_reads_translate_and_carries_category()
    except AssertionError as e:
        ok = False
        print("\nTEST FAILURE(S):")
        for f in (e.args[0] if e.args else []):
            print("  -", f)

    if HAVE_USD:
        print("\n" + "=" * 78)
        print("CONTROL: seating from raw translate.z instead of the "
             "measured pre-pose ground (the anti-pattern "
             "`apply_street_furniture_pose`'s own docstring warns "
             "against). A test whose control does not fail is not proof "
             "-- this check is EXPECTED TO FAIL.")
        print("=" * 78)
        before = len(FAILS)
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        prim_path = "/World/item_test"
        prim = _build_item_prim(stage, prim_path)
        decision = {"action": "flat", "azimuth_deg": 0.0, "lean_deg": 88.0,
                   "slide_m": 0.0}
        _broken_apply_seat_from_translate(stage, prim_path, decision,
                                          ssf=1.0)
        bounds = _world_bounds(prim)
        if bounds is None:
            check("CONTROL: broken apply produced measurable geometry",
                 False, bounds)
        else:
            (_lox, _loy, loz), (_hix, _hiy, _hiz) = bounds
            check("CONTROL: seat-from-translate lands at the TRUE ground "
                 "(expected to be WRONG -- it should land "
                 "{0:.2f} m too high, at translate.z)"
                 .format(ANCHOR_OFFSET_M),
                 abs(loz - TRUE_GROUND_Z) < 0.02, loz)
        control_fails = FAILS[before:]
        print("\nControl produced {0} failure(s) (expected: > 0):"
             .format(len(control_fails)))
        for f in control_fails:
            print("  CONTROL-FAIL -", f)
        if not control_fails:
            print("\nCONTROL DID NOT FAIL -- this test does not actually "
                 "pin the seating bug. Treat this as a defect in the TEST.")
            ok = False
        else:
            print("\nControl failed as expected: this test would have "
                 "caught a seat-from-translate regression.")

    if not ok:
        sys.exit(1)
    print("\nALL PASS" + (" (and the control correctly failed)"
                          if HAVE_USD else ""))
