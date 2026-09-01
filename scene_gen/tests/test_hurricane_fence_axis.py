#!/usr/bin/env python3
"""test_hurricane_fence_axis.py — pins the fence long-axis bug in
`washaway.measure_fence`/`apply_fence_pose` (fixed 2026-09-01 on the user's
own report against the live 500 m hurricane plate).

THE REPORTED DEFECT (verbatim): "A lot of fences look like they're on their
short side (which is wrong) and are standing up straight."

ROOT CAUSE. `apply_placements` authors every placed prim as
translate -> rotateXYZ(roll, pitch, yaw) -> scale. A fence module's AUTHORED
yaw is not the fence's own run bearing — it is the run bearing PLUS whatever
`yaw-offset` the asset declares in `config/asset_sets/suburban.yaml`, the
correction that turns art modelled along its own local axis onto the
boundary line. The privacy panel `objaverse://1cec32aea61e4959bb1ba9a79ada7cd5`
("Wood fence", 60% of fenced lots) is NATIVE 0.076-0.08 m (local X,
thickness) x 3.51 m (local Y, length) x 1.83 m (Z, height) with
`yaw-offset: 90.0`, so its authored `rotateXYZ[2]` points ACROSS the panel,
not along it.

`measure_fence` used to read `yaw = rotateXYZ[2]` and project the panel's
world points onto `(cos yaw, sin yaw)` to get `length_m` — for this asset
that axis is the THICKNESS direction, so it returned 0.08 m as the panel's
"length". `apply_fence_pose` then used that SAME wrong yaw as the HINGE AXIS
for the "blown flat" rotation (`lean_rot` about
`(cos yaw, sin yaw, 0)`, `lean_deg` ~90). Hinging a 0.08 x 3.51 x 1.83 m
panel about the axis that runs ACROSS it (rather than along it) rotates the
3.51 m length axis into VERTICAL and lays the 0.08 m thickness axis down
flat — exactly "on its short side" AND "standing up straight" at once. One
wrong axis, both symptoms, and the derivation is checked mechanically by
this file's own `_check_apply_flat` (see its docstring for the full
before/after geometry).

THE FIX (already applied to `washaway.py`; this file may NOT edit it).
`measure_fence` now measures the panel's extent along BOTH `yaw` and
`yaw + 90` and keeps the LONGER as the panel's true long axis, self-
correcting for whatever `yaw-offset` (if any) the asset declares.
`apply_fence_pose` uses that corrected yaw ONLY as the hinge axis, and
rebuilds the placement rotation by reading the prim's OWN `rotateXYZ` back
in FULL (roll, pitch, yaw) rather than re-deriving just a Z rotation from
`decision["yaw"]` — the old code silently dropped any authored roll/pitch.

WHAT THIS FILE VERIFIES, OFFLINE, WITH BARE `pxr` (no Isaac Sim, no
`SimulationApp` — `usd-core` is importable from host `python3`):

  1. A synthetic fence prim reproducing the REAL asset shape (a
     0.08 x 3.51 x 1.83 m box, long axis on local +Y, placed with
     `rotateXYZ(0, 0, run_bearing + 90)`) at several run bearings.
  2. `measure_fence` returns `length_m` in `[3.51, 3.59]` m (the LONG
     dimension, never the ~0.08 m thickness), at every bearing — and a yaw
     that matches the true run bearing modulo 180 degrees (an axis, not a
     directed vector, has two equally valid bearings 180 degrees apart).
     THE `[3.51, 3.59]` BAND, NOT A TIGHT `~3.51`, IS DELIBERATE — see
     "FINDING 2" below; this is measured, derived and proven bounded, not
     a loose guess.
  3. That yaw is independently re-measured (via `bake.world_point_bounds`,
     never `UsdGeom.BBoxCache` — see that function's own bug catalogue) to
     confirm the world extent along it really is the larger of the two
     candidates, not merely trusted algebra.
  4. THE DECISIVE CHECK: `apply_fence_pose` is actually run with a "flat"
     decision, and the RESULTING WORLD GEOMETRY is measured, points-based.
     A correctly flattened panel ends up with a SMALL Z extent (~0.08 m,
     the thickness) and a plan footprint of ~3.51 m along the fence run by
     ~1.83 m across (the fallen height). The bug's signature — Z extent
     blown up to ~3.51 m, footprint collapsed to ~0.08 x ~1.83 — must NOT
     appear. SEE "FINDING 3" BELOW: against the fix as shipped, this check
     only actually passes at ONE of this file's four test bearings.
  5. Roll survives: a fence authored with a non-zero `rotateXYZ` roll
     flattens to DIFFERENTLY-SHAPED world geometry than an otherwise
     identical roll-free fence — proof `apply_fence_pose` reads roll back
     off the prim rather than discarding it (the old code's second bug).
  6. A CONTROL, in `__main__` only (never as a pytest test): `measure_fence`
     is monkeypatched to the OLD, uncorrected-yaw behaviour, and (2)/(4)'s
     checks are re-run against it. Every one of them is EXPECTED TO FAIL —
     a test whose control does not fail is not proof this test pins
     anything.

FINDING 1 — `measure_fence`'S AXIS SELECTION IS CORRECT. Proven, not just
observed: writing out the AABB-corner proxy's own formula shows
`along - across = (LENGTH_M - THICKNESS_M) * (1 - |sin(2*theta)|)`, where
`theta` is the panel's offset from world-axis alignment — `>= 0` for every
`theta`, hitting exactly `0` only at the single knife-edge `theta = 45`
degrees (a measure-zero case for a real run bearing). The reported bug
(wrong hinge axis) is fixed correctly and robustly at every bearing.

FINDING 2 — SUBSEQUENTLY FIXED, 2026-09-01, AFTER THIS FILE WAS WRITTEN.
`measure_fence` now projects the TRANSFORMED MESH POINTS rather than the
four plan corners of their AABB, so `length_m` reads the true 3.510 m at
every bearing and the `[3.51, 3.59]` band below is historical. The analysis
that follows is kept because it is what identified the error.
`measure_fence`'S `length_m` VALUE HAD A BOUNDED, MINOR INACCURACY AT
OBLIQUE BEARINGS. `_extent(a_deg)` projects the panel's WORLD-AXIS-
ALIGNED AABB CORNERS (`bake.world_point_bounds`'s own `bx0/bx1/by0/by1`)
onto a candidate bearing, not the mesh's TRUE rotated corners. Those agree
only when the panel's own axes are exactly aligned with world X/Y
(bearings 0/90 in this file's own sweep, which is why THOSE two measure
exactly 3.510000 / 0.080000); at any other bearing the AABB-of-corners
over-states both candidate extents, but PROVABLY bounded: `length_m` lands
in `[LENGTH_M, LENGTH_M + THICKNESS_M]` = `[3.51, 3.59]`, and the losing
"across" candidate lands in `[THICKNESS_M, LENGTH_M + THICKNESS_M]` =
`[0.08, 3.59]` — at the worst bearing tested here (37 degrees) `across`
reads 3.454 m, nowhere near the true 0.08 m. This never flips the axis
CHOICE (Finding 1's proof covers that) — it only means `length_m` itself
can read up to the panel's own thickness too high at odd bearings, worth
knowing if any caller ever uses it for something more precise than "is
this the long axis" (nothing in `washaway.py` currently does — `fence_
specs` only threads it, unread, into the decision dict).

FINDING 3 — SUBSEQUENTLY FIXED, 2026-09-01, AFTER THIS FILE WAS WRITTEN.
The composition is now `place_rot * lean_rot` and the false "RIGHT operand
first" claim has been removed from `apply_fence_pose`'s own docstring;
measured points-based, the flattened panel is 3.51 m along the run by
1.83 m across with a 0.080 m Z extent at every bearing in this file's
sweep. The analysis that follows is kept because it is what found the bug,
and its control still reproduces the pre-fix failure.
`apply_fence_pose` COMPOSED ITS ROTATION IN THE WRONG ORDER, A SEPARATE AND
MORE SERIOUS BUG THAN THE ONE THIS FILE WAS WRITTEN FOR. The function
builds `combined = lean_rot * place_rot` on the strength of its own
docstring's claim that "`Gf.Rotation`'s `*` performs the RIGHT operand
first" (intent: place the panel in world orientation, THEN tip it about
the world-frame hinge). Measured directly in this repo's own `pxr` —
`(Gf.Rotation(+Z,90) * Gf.Rotation(+X,90)).TransformDir((1,0,0))` lands at
`(0,0,1)`, which is only reachable by applying the LEFT operand (the Z
rotation) FIRST — `Gf.Rotation`'s `*` actually composes LEFT-operand-
first, the opposite of the docstring's claim. So the code as written
applies `lean_rot` FIRST (to the panel's still-LOCAL, not-yet-placed
geometry, using an axis built from WORLD coordinates — a frame mismatch)
and `place_rot` SECOND, backwards from the physical intent.

This is not theoretical: sweeping this file's own four test bearings
through the REAL, unmodified `apply_fence_pose` gives —

    bearing   Xspan   Yspan   Zspan   (expect Zspan == 0.08 always)
       0.0    1.830   0.080   3.510   <- WRONG: this is the ORIGINAL bug's
                                          exact signature (length vertical)
      37.0    2.596   2.359   2.851   <- WRONG: smeared across all 3 axes,
                                          not flat in any recognisable plane
      90.0    1.830   3.510   0.080   <- happens to be correct, BY NUMERIC
                                          COINCIDENCE at this one bearing
     205.0    2.368   2.402   3.215   <- WRONG, same as 37 degrees

and hand-composing the SWAPPED order (`place_rot * lean_rot`) reproduces
the physically-correct "thickness axis always ends up purely vertical"
result at EVERY one of those bearings, confirming the fix's actual defect
is the operand order, not anything else in the surrounding logic (the
hinge-axis VALUE it is fed, per Finding 1, is correct).

THIS TEST DOES NOT PAPER OVER FINDING 3: `test_apply_fence_pose_flat_
lands_on_thickness_not_length` is HONESTLY EXPECTED TO FAIL against the
current `washaway.py` at every bearing except 90 — that failure is this
file correctly doing its job of pinning the fix's actual behaviour, not a
test-authoring defect. (Contrast with the CONTROL in `__main__`, which is
SUPPOSED to fail and says so explicitly at the end of its own output; this
pytest test is NOT supposed to fail, and its failure here IS the finding.)

USAGE
    python3 scene_gen/tests/test_hurricane_fence_axis.py
    pytest -q scene_gen/tests/test_hurricane_fence_axis.py
"""

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import washaway as wash                    # noqa: E402

try:
    from pxr import Gf, Usd, UsdGeom                      # noqa: E402
    from disaster import bake                              # noqa: E402
    HAVE_USD = True
except Exception:                                          # pragma: no cover
    HAVE_USD = False

FAILS = []


def check(name, cond, detail=""):
    detail = "" if not detail else str(detail)
    if not cond:
        FAILS.append("{0}: {1}".format(name, detail))
    print(("PASS " if cond else "FAIL ") + name +
          ("" if not detail else " -- " + detail))


def strict(fn):
    """Same discipline `test_hurricane_fences.py` uses: a test that adds to
    `FAILS` without asserting is a test nobody can trust ran."""
    def run(*a, **kw):
        mine = len(FAILS)
        out = fn(*a, **kw)
        assert len(FAILS) == mine, FAILS[mine:]
        return out
    run.__name__, run.__doc__ = fn.__name__, fn.__doc__
    return run


# ---------------------------------------------------------------------------
# geometry constants matching the REAL objaverse asset named in the bug
# report (`config/asset_sets/suburban.yaml`, `lot_fences` entry
# `objaverse://1cec32aea61e4959bb1ba9a79ada7cd5`).
# ---------------------------------------------------------------------------
THICKNESS_M = 0.08     # local X — the SHORT axis the bug used to measure
LENGTH_M = 3.51        # local Y — the LONG axis, `yaw-offset: 90.0` maps it
HEIGHT_M = 1.83        # local Z (up), unaffected by yaw
YAW_OFFSET_DEG = 90.0
BEARINGS = (0.0, 37.0, 90.0, 205.0)   # cardinal, odd, right-angle, reflex


def _make_box_mesh(stage, mesh_path, thickness_m, length_m, height_m):
    """A box `Mesh` shaped exactly like the real fence panel: local X =
    thickness, Y = length, Z = height (base at local z=0, matching an
    asset whose pivot sits at ground level). Only EXTENTS (spans) matter to
    every assertion in this file, so X/Y are centred on the local origin;
    only Z needs to start at 0 for the "seated at ground" half of
    `apply_fence_pose` to have something sane to do (not itself asserted
    here — only the ROTATED extents are)."""
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    hx, hy = thickness_m / 2.0, length_m / 2.0
    z0, z1 = 0.0, height_m
    pts = [(-hx, -hy, z0), (hx, -hy, z0), (hx, hy, z0), (-hx, hy, z0),
           (-hx, -hy, z1), (hx, -hy, z1), (hx, hy, z1), (-hx, hy, z1)]
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([4] * 6)
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 5, 4,
                                      2, 3, 7, 6, 0, 3, 7, 4, 1, 2, 6, 5])
    lo = [min(q[k] for q in pts) for k in range(3)]
    hi = [max(q[k] for q in pts) for k in range(3)]
    mesh.CreateExtentAttr([Gf.Vec3f(*lo), Gf.Vec3f(*hi)])
    return mesh.GetPrim()


def _build_fence_prim(stage, prim_path, run_bearing_deg, roll_deg=0.0,
                      pitch_deg=0.0, x_m=0.0, y_m=0.0,
                      thickness_m=THICKNESS_M, length_m=LENGTH_M,
                      height_m=HEIGHT_M, yaw_offset_deg=YAW_OFFSET_DEG,
                      ssf=1.0):
    """Reproduces exactly what `apply_placements` puts on stage for one
    fence module: an Xform holder with
    translate -> rotateXYZ(roll, pitch, run_bearing + yaw_offset) -> scale,
    with a child Mesh box shaped like the real asset. Returns
    `(prim, authored_yaw_deg)`.
    """
    UsdGeom.Xform.Define(stage, prim_path)
    prim = stage.GetPrimAtPath(prim_path)
    xf = UsdGeom.Xformable(prim)
    authored_yaw = (float(run_bearing_deg) + float(yaw_offset_deg)) % 360.0
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x_m) * ssf, float(y_m) * ssf, 0.0))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(float(roll_deg), float(pitch_deg),
                                     float(authored_yaw)))
    xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    _make_box_mesh(stage, prim_path + "/panel", thickness_m * ssf,
                   length_m * ssf, height_m * ssf)
    return prim, authored_yaw


def _ang_mod180_diff(a_deg, b_deg):
    """Smallest difference between two bearings treated as UNDIRECTED lines
    (an axis, not a vector, is unchanged by a 180-degree flip)."""
    d = (float(a_deg) - float(b_deg)) % 180.0
    return min(d, 180.0 - d)


def _merged_world_bounds(prim, ssf=1.0):
    """Merged world AABB `((lox,loy,loz), (hix,hiy,hiz))` over every Mesh
    descendant of `prim`, POINTS-BASED via `bake.world_point_bounds` —
    never `UsdGeom.BBoxCache`, which the `fix-floating-debris` skill and
    `bake.world_point_bounds`'s own docstring document as inflating hugely
    (in both directions) for a thin sheet lying at an angle inside its own
    local extent box, which is exactly this shape."""
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


def _extent_along(prim, angle_deg, ssf=1.0):
    """World-space extent of `prim`'s mesh geometry projected onto the
    bearing `angle_deg`, measured from the TRANSFORMED MESH POINTS.

    POINTS, NOT THE AABB CORNERS this helper first used. Projecting the four
    plan corners of an axis-aligned bounding box onto an arbitrary bearing
    over-reads for any panel not aligned with world X/Y, because the AABB of
    a diagonal rectangle is strictly larger than the rectangle. Measured on
    the correctly-flattened 3.51 x 1.83 panel at a 205 deg run bearing, the
    AABB-corner form reported along = 4.91 m and across = 4.52 m where the
    true answers are 3.51 and 1.83 — so this helper failed a CORRECT
    `apply_fence_pose` and the failure was the measurement's, not the
    code's. The repo's `fix-floating-debris` skill documents exactly this
    bbox-on-a-thin-rotated-sheet blind spot; a test written to catch a
    geometry bug must not reintroduce it in its own oracle."""
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    ca = math.cos(math.radians(angle_deg))
    sa = math.sin(math.radians(angle_deg))
    lo = hi = None
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
        if not pts:
            continue
        m = xc.GetLocalToWorldTransform(p)
        for v in pts:
            w = m.Transform(Gf.Vec3d(float(v[0]), float(v[1]), float(v[2])))
            q = (w[0] / ssf) * ca + (w[1] / ssf) * sa
            lo = q if lo is None else min(lo, q)
            hi = q if hi is None else max(hi, q)
    return 0.0 if lo is None else (hi - lo)


def _old_measure_fence(stage, prim, ssf=1.0):
    """Reproduces the PRE-FIX `measure_fence` verbatim, for the CONTROL
    only: `yaw = rotateXYZ[2]` taken as-is (no long-axis correction), and
    `length_m` is the extent projected onto THAT axis alone. This is not a
    guess at what the old code did — it is transcribed from the description
    in the current `measure_fence`'s own docstring/inline comments (the fix
    commit's before/after), which this test file may read but the module
    itself may not be edited to reproduce."""
    xf = UsdGeom.Xformable(prim)
    vals = {}
    for op in xf.GetOrderedXformOps():
        vals[op.GetOpName().split(":")[-1]] = op.Get()
    t = vals.get("translate") or Gf.Vec3d(0.0, 0.0, 0.0)
    rot = vals.get("rotateXYZ")
    yaw = float(rot[2]) if rot is not None else 0.0
    x, y = float(t[0]) / ssf, float(t[1]) / ssf
    length = _extent_along(prim, yaw, ssf=ssf)
    return x, y, yaw, length


# ---------------------------------------------------------------------------
# shared check bodies — used both by the pytest tests (wrapped in `strict`,
# expected to PASS) and by the `__main__` CONTROL (run unwrapped against a
# monkeypatched `measure_fence`, expected to FAIL).
# ---------------------------------------------------------------------------

def _check_measure_fence(run_bearing_deg, tag=None):
    """Checks (2) and (3) from the module docstring against whatever
    function currently answers to `wash.measure_fence` — the real, fixed
    one under pytest; the monkeypatched pre-fix one under the CONTROL."""
    tag = tag or "measure bearing={0:.1f}".format(run_bearing_deg)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    prim_path = "/World/fence_test"
    prim, authored_yaw = _build_fence_prim(stage, prim_path, run_bearing_deg)

    x, y, yaw, length = wash.measure_fence(stage, prim, ssf=1.0)

    # BOUNDS, NOT A TIGHT ~3.51 EQUALITY -- see the module docstring's
    # "FINDING 2": `measure_fence`'s AABB-corner proxy measures
    # the TRUE length exactly only at axis-aligned bearings; at an oblique
    # bearing it over-reads by a PROVEN-BOUNDED amount, up to the panel's
    # own thickness. The lower bound (never LESS than the true length) and
    # the upper bound (never more than length+thickness) are both real
    # claims this test pins, not slack for a sloppy test.
    check("[{0}] measured length is never LESS than the true {1} m length"
         .format(tag, LENGTH_M), length >= LENGTH_M - 1e-3, length)
    check("[{0}] measured length is bounded by the proven worst case "
         "(length + thickness = {1:.2f} m)"
         .format(tag, LENGTH_M + THICKNESS_M),
         length <= LENGTH_M + THICKNESS_M + 1e-3, length)
    check("[{0}] measured length is clearly not the {1} m thickness"
         .format(tag, THICKNESS_M), length > 1.0, length)
    check("[{0}] returned yaw matches the true run bearing mod 180 deg"
         .format(tag),
         _ang_mod180_diff(yaw, run_bearing_deg) < 0.01,
         "yaw={0:.3f} run_bearing={1:.3f}".format(yaw, run_bearing_deg))

    # Independent re-measurement: the world extent along the RETURNED yaw
    # really is the larger of the two axis candidates, not merely trusted --
    # this is the invariant that is actually PROVEN to never flip (see the
    # module docstring), so it is checked as a strict inequality, not a
    # tolerance band.
    along = _extent_along(prim, yaw)
    across = _extent_along(prim, yaw + 90.0)
    check("[{0}] independently re-measured extent along the returned yaw "
         "IS the larger of the two candidates".format(tag),
         along > across, (along, across))
    check("[{0}] independently re-measured extent along returned yaw "
         "matches measure_fence's own length_m (no transcription error in "
         "this file's own re-derivation)".format(tag),
         abs(along - length) < 1e-6, (along, length))
    check("[{0}] independently re-measured extent across (yaw+90) is "
         "bounded (thickness .. length+thickness), the same proxy "
         "envelope as `along`".format(tag),
         THICKNESS_M - 1e-3 <= across <= LENGTH_M + THICKNESS_M + 1e-3,
         across)


def _check_apply_flat(run_bearing_deg, roll_deg=0.0, tag=None):
    """Check (4), THE DECISIVE ONE. Measures, then flattens, one fence and
    inspects the RESULTING WORLD GEOMETRY, points-based
    (`_merged_world_bounds`/`_extent_along`, never `UsdGeom.BBoxCache`).

    THE MECHANICAL DERIVATION THIS PINS (roll=pitch=0 case): before the
    fix, the hinge axis (`decision["yaw"]`) sat along the panel's THICKNESS
    direction (perpendicular to the real run). Rotating a box 90 degrees
    about the axis perpendicular to its own long dimension swaps that long
    dimension with whichever axis IS perpendicular to the hinge and
    initially vertical — i.e. the 3.51 m length axis rotates from
    horizontal into VERTICAL, and the 1.83 m height axis rotates down to
    horizontal, ALONG the run. Result: Z extent ~3.51 m (the panel stands
    up), footprint ~0.08 m (run) x ~1.83 m (across) — "on its short side
    ... standing up straight", verbatim.

    After the fix, the hinge runs along the TRUE 3.51 m length axis, so
    rotating 90 degrees about it swaps HEIGHT (1.83 m, vertical) with
    THICKNESS (0.08 m, horizontal, across the run) and leaves the length
    axis alone. Result: Z extent ~0.08 m, footprint ~3.51 m (run) x
    ~1.83 m (across) — lying down, correctly, on its own thickness.
    """
    tag = tag or "flat bearing={0:.1f} roll={1:.1f}".format(
        run_bearing_deg, roll_deg)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    prim_path = "/World/fence_test"
    prim, authored_yaw = _build_fence_prim(stage, prim_path, run_bearing_deg,
                                           roll_deg=roll_deg)

    x, y, yaw, length = wash.measure_fence(stage, prim, ssf=1.0)
    decision = {"action": "flat", "yaw": yaw, "lean_deg": 90.0}
    changed = wash.apply_fence_pose(stage, prim_path, decision, ssf=1.0)

    check("[{0}] apply_fence_pose reports it changed the prim".format(tag),
         changed is True, changed)

    bounds = _merged_world_bounds(prim, ssf=1.0)
    check("[{0}] flattened panel has measurable world geometry".format(tag),
         bounds is not None, bounds)
    if bounds is None:
        return

    (_lox, _loy, loz), (_hix, _hiy, hiz) = bounds
    z_span = hiz - loz
    # Ground truth is `run_bearing_deg` itself, NOT `yaw` — independent of
    # whatever `measure_fence` (real or monkeypatched) claims the axis is.
    along = _extent_along(prim, run_bearing_deg)
    across = _extent_along(prim, run_bearing_deg + 90.0)

    check("[{0}] flattened panel's Z extent is the {1} m THICKNESS, not "
         "the 1.83 m height".format(tag, THICKNESS_M),
         abs(z_span - THICKNESS_M) < 0.02, z_span)
    check("[{0}] flattened panel's Z extent has NOT become the {1} m "
         "LENGTH (the 'standing on its short side' bug)"
         .format(tag, LENGTH_M), z_span < 1.0, z_span)
    check("[{0}] flattened panel's plan footprint runs ~{1} m ALONG the "
         "fence line".format(tag, LENGTH_M),
         abs(along - LENGTH_M) < 0.02, along)
    check("[{0}] flattened panel's plan footprint is ~{1} m ACROSS the "
         "fence line (the fallen height)".format(tag, HEIGHT_M),
         abs(across - HEIGHT_M) < 0.02, across)


# ---------------------------------------------------------------------------
# pytest-visible tests — all against the REAL, fixed washaway.py
# ---------------------------------------------------------------------------

@strict
def test_measure_fence_returns_long_axis_at_every_bearing():
    """(2)+(3): `measure_fence` reports the LONG (3.51 m) dimension as
    `length_m`, and its yaw axis, independently re-measured, really is the
    longer of the two candidates — at a cardinal, an odd, a right-angle and
    a reflex run bearing."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    for b in BEARINGS:
        _check_measure_fence(b)


@strict
def test_apply_fence_pose_flat_lands_on_thickness_not_length():
    """(4), THE DECISIVE TEST: a fence blown "flat" ends up lying on its
    0.08 m thickness with a 3.51 x 1.83 m footprint, at every bearing —
    never standing up on its 3.51 m length with a 0.08 m footprint."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    for b in BEARINGS:
        _check_apply_flat(b)


@strict
def test_apply_fence_pose_preserves_authored_roll():
    """(6): a fence authored with a non-zero `rotateXYZ` roll must
    flatten to MEASURABLY DIFFERENT world geometry than an otherwise
    identical roll-free fence. The old code rebuilt the placement rotation
    from `decision["yaw"]` alone (a pure Z rotation) and folded only the Z
    translate, silently discarding any authored roll/pitch — under that
    behaviour the two cases below would be geometrically IDENTICAL."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    bearing = 40.0

    stage_a = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage_a, "/World")
    prim_a, _ = _build_fence_prim(stage_a, "/World/fence_a", bearing,
                                  roll_deg=0.0)
    _x, _y, yaw_a, _len = wash.measure_fence(stage_a, prim_a, ssf=1.0)
    wash.apply_fence_pose(stage_a, "/World/fence_a",
                          {"action": "flat", "yaw": yaw_a, "lean_deg": 90.0},
                          ssf=1.0)
    bounds_a = _merged_world_bounds(prim_a, ssf=1.0)

    stage_b = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage_b, "/World")
    prim_b, _ = _build_fence_prim(stage_b, "/World/fence_b", bearing,
                                  roll_deg=5.0)
    _xb, _yb, yaw_b, _lenb = wash.measure_fence(stage_b, prim_b, ssf=1.0)
    wash.apply_fence_pose(stage_b, "/World/fence_b",
                          {"action": "flat", "yaw": yaw_b, "lean_deg": 90.0},
                          ssf=1.0)
    bounds_b = _merged_world_bounds(prim_b, ssf=1.0)

    check("apply_fence_pose ran on both fences",
         bounds_a is not None and bounds_b is not None,
         (bounds_a, bounds_b))
    if bounds_a is None or bounds_b is None:
        return

    flat_a = bounds_a[0] + bounds_a[1]
    flat_b = bounds_b[0] + bounds_b[1]
    max_diff = max(abs(a - b) for a, b in zip(flat_a, flat_b))
    check("a 5-degree authored roll changes the flattened world geometry "
         "(roll survives apply_fence_pose, not discarded)",
         max_diff > 1e-3,
         "max coordinate delta={0:.6f} (bounds_a={1}, bounds_b={2})"
         .format(max_diff, bounds_a, bounds_b))


if __name__ == "__main__":
    if not HAVE_USD:
        print("pxr is not importable on this interpreter -- cannot run "
             "this file's checks at all (usd-core should be pip-installed "
             "on the host per this repo's own convention).")
        sys.exit(1)

    ok = True
    try:
        test_measure_fence_returns_long_axis_at_every_bearing()
        test_apply_fence_pose_flat_lands_on_thickness_not_length()
        test_apply_fence_pose_preserves_authored_roll()
    except AssertionError as e:
        ok = False
        print("\nPIN TEST FAILURE(S) against the current washaway.py "
             "(measure_fence's axis fix is correct -- see FINDING 3 in "
             "this file's own module docstring if the failures below are "
             "all from the 'flat' test; that is a REAL, SEPARATE bug in "
             "apply_fence_pose's rotation-composition order, reported, "
             "not fixed by this file):")
        for f in (e.args[0] if e.args else []):
            print("  -", f)

    print("\n" + "=" * 78)
    print("CONTROL: monkeypatching wash.measure_fence to the PRE-FIX "
         "(uncorrected-yaw) behaviour.")
    print("A test whose control does not fail is not proof -- every check "
         "below is EXPECTED TO FAIL.")
    print("=" * 78)
    before = len(FAILS)
    orig_measure_fence = wash.measure_fence
    wash.measure_fence = _old_measure_fence
    try:
        for b in BEARINGS:
            tag = "CONTROL bearing={0:.1f}".format(b)
            _check_measure_fence(b, tag=tag)
            _check_apply_flat(b, tag=tag)
    finally:
        wash.measure_fence = orig_measure_fence
    control_fails = FAILS[before:]

    print("\nControl produced {0} failure(s) out of the checks it ran "
         "(expected: > 0, ideally most of them):".format(len(control_fails)))
    for f in control_fails:
        print("  CONTROL-FAIL -", f)

    if not control_fails:
        print("\nCONTROL DID NOT FAIL -- this test does not actually pin "
             "the bug. Treat this as a defect in the TEST, not evidence "
             "the fix is unnecessary.")
        ok = False
    else:
        print("\nControl failed as expected: this test would have caught "
             "the pre-fix bug.")

    if not ok:
        sys.exit(1)
    print("\nALL PASS (fixed washaway.py) -- and the control correctly "
         "failed against the pre-fix behaviour.")
