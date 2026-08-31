#!/usr/bin/env python3
"""test_fire_bake_wall_backing.py — the BACKING TEST that replaced the
blanket `_WALL_ATTACHED` exemption in `disaster.fire_bake._judge_candidates`
(2026-08-30, fire_dtc3 bench, building b5: `spall`/`spallhalo` stamps hanging
10-38 m in the air where the wall behind them was torn away by collapse, but
kept ACTIVE anyway because every wall-attached family was exempted from the
lean/support check with no check that a wall was actually still there).

    pytest -q scene_gen/tests/test_fire_bake_wall_backing.py

    # in the container, where pxr + vtk are real:
    docker exec isaac-sim bash -c \
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
       /isaac-sim/AirStack/scene_gen/tests/test_fire_bake_wall_backing.py"

Needs `pxr` (for the stage) and `vtk` (`_judge_candidates`'s locator) —
neither is on the host, so every test skips (prints SKIP, returns) when
`HAVE_USD` is False, same convention as `test_fire_bake.py`.

WHAT IS BUILT. A flat "spall" stamp is a fan/strip mesh whose every point
sits the SAME distance along the SAME face normal (`urban_fire._stamp_pt`,
`quake_flow._b_face_pt`) — exactly coplanar. This test does not need the
real authoring code to prove that: a bare 4-point quad in a single plane
has the identical property, so `_flat_normal`'s SVD fit sees the same shape
a real stamp gives it. Every case below places one such quad, floating high
in the air with nothing under it (so the plain downward "seated" check
never accidentally supplies contact and the backing test is what decides
its fate), and a second quad standing in for "the wall" at a controlled
distance straight behind it — or no second quad at all.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)

from disaster import fire_bake as fb                          # noqa: E402

try:
    from pxr import Gf, Sdf, Usd, UsdGeom                     # noqa: E402
    HAVE_USD = True
except Exception:                                              # pragma: no cover
    HAVE_USD = False


def _stage():
    """A fresh in-memory stage with `fb.BAKE_ROOT`/g0 ready for meshes."""
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path(fb.BAKE_ROOT))
    cell = fb.BAKE_ROOT + "/g0"
    UsdGeom.Xform.Define(stage, Sdf.Path(cell))
    return stage, cell


def _quad(stage, path, x, y0, y1, z0, z1):
    """A flat 4-point mesh at fixed `x`, spanning `[y0, y1] x [z0, z1]` --
    coplanar in the world YZ plane by construction, same as a real stamp
    authored on an X-facing wall."""
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    pts = [(x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1)]
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    return mesh


# a stamp this small, this high, with nothing below it -- gap (~10 m) is
# always far past `gap_m` (1.0), so every case turns on backing alone.
_STAMP_HALF = 0.15
_STAMP_X = 5.0
_STAMP_Z0, _STAMP_Z1 = 9.85, 10.15


def _stamp(stage, cell, tag, y_center):
    path = "{0}/spall_{1}_1".format(cell, tag)
    _quad(stage, path, _STAMP_X, y_center - _STAMP_HALF, y_center + _STAMP_HALF,
         _STAMP_Z0, _STAMP_Z1)
    return path


def _wall(stage, cell, tag, y_center, dx):
    """A big wall-standin quad `dx` metres behind the stamp at `y_center`
    (i.e. at `_STAMP_X - dx`) -- named so it does NOT match any
    `_CANDIDATE_PREFIXES` entry, exactly like a real structural wall mesh."""
    path = "{0}/wall_test_{1}".format(cell, tag)
    _quad(stage, path, _STAMP_X - dx, y_center - 2.0, y_center + 2.0, 8.0, 12.0)
    return path


def _judged_by_path(info):
    return {j["path"]: j for j in info["judged"]}


def test_wall_backed_at_5cm_is_kept():
    """A wall 0.05 m behind the stamp (well inside `_BACKING_MAX_M`) --
    real construction proud offsets are 3-3.6 cm -- must be found and the
    stamp KEPT."""
    if not HAVE_USD:
        print("  wall_backed_5cm      SKIP (no pxr — run under usd_python.sh)")
        return
    stage, cell = _stage()
    spall = _stamp(stage, cell, "a", 0.0)
    _wall(stage, cell, "a", 0.0, dx=0.05)

    info = fb._judge_candidates(stage, fb.BAKE_ROOT, gap_m=1.0, verbose=False)
    j = _judged_by_path(info)[spall]
    assert j["contact"] is True, j
    assert j["deactivate"] is False, j
    assert j["backing_m"] is not None and abs(j["backing_m"] - 0.05) < 1e-3, j
    print("  wall_backed_5cm      OK  (backing_m={0:.4f}, kept)".format(
        j["backing_m"]))


def test_no_wall_is_deactivated():
    """No wall at all behind the stamp -- the collapse took it -- must be
    DEACTIVATED. This is the exact `spall_g5_70` shape from the bug
    report: `hit=ground`, `contact=False`, now also `backing_m=None`."""
    if not HAVE_USD:
        print("  no_wall              SKIP (no pxr — run under usd_python.sh)")
        return
    stage, cell = _stage()
    spall = _stamp(stage, cell, "b", 20.0)
    # no _wall() call -- nothing behind it

    info = fb._judge_candidates(stage, fb.BAKE_ROOT, gap_m=1.0, verbose=False)
    j = _judged_by_path(info)[spall]
    assert j["contact"] is False, j
    assert j["deactivate"] is True, j
    assert j["backing_m"] is None, j
    assert j["support_path"] is None and j["support_z"] == 0.0, j   # "ground"
    print("  no_wall              OK  (deactivated, backing_m=None)")


def test_wall_1m_back_is_too_far_and_deactivated():
    """A wall that IS present, but 1.0 m back -- far past `_BACKING_MAX_M`
    (0.35 m) -- does not count as backing. This is the case the old
    blanket exemption could not distinguish from "kept": no distance check
    at all, only "was anything (non-decal) found by the short lean rays".
    """
    if not HAVE_USD:
        print("  wall_1m_away         SKIP (no pxr — run under usd_python.sh)")
        return
    stage, cell = _stage()
    spall = _stamp(stage, cell, "c", 40.0)
    _wall(stage, cell, "c", 40.0, dx=1.0)

    info = fb._judge_candidates(stage, fb.BAKE_ROOT, gap_m=1.0, verbose=False)
    j = _judged_by_path(info)[spall]
    assert j["contact"] is False, j
    assert j["deactivate"] is True, j
    assert j["backing_m"] is None, j
    print("  wall_1m_away         OK  (deactivated, wall out of reach)")


def test_bar_family_keeps_the_old_lean_exemption():
    """`sbar`/`rebar` are round bars, not flat decals (see
    `_WALL_BAR_FAMILIES`'s comment in `fire_bake.py` for why the backing
    test does not apply to them) -- they must still go through the OLD
    generic lean check unchanged: a real neighbour within lean reach keeps
    them, and `backing_m` is never even computed for them (it stays
    `None` because they never enter the decal branch at all)."""
    if not HAVE_USD:
        print("  bar_family_lean      SKIP (no pxr — run under usd_python.sh)")
        return
    stage, cell = _stage()
    # a tiny bar (`_cyl`-shaped in real life; a small quad here is enough --
    # the lean check only looks at the group's bounding box) high in the
    # air, with a real neighbour close enough for the SHORT lean-check
    # reach (`half_extent + _LEAN_MARGIN_M`, a few tens of cm) to catch.
    bar_path = "{0}/sbar_z_1".format(cell)
    _quad(stage, bar_path, 60.0, -0.05, 0.05, 9.95, 10.05)
    neighbour = "{0}/wall_test_z".format(cell)
    _quad(stage, neighbour, 60.05, -1.0, 1.0, 8.0, 12.0)

    info = fb._judge_candidates(stage, fb.BAKE_ROOT, gap_m=1.0, verbose=False)
    j = _judged_by_path(info)[bar_path]
    assert j["contact"] is True, j
    assert j["deactivate"] is False, j
    assert j["backing_m"] is None, j            # bars never run the new test
    print("  bar_family_lean      OK  (sbar/rebar unaffected, still kept "
          "by the lean check)")


def test_invisible_wall_does_not_back():
    """An INVISIBLE wall 5 cm behind a stamp must NOT count as backing.

    `_judge_candidates`' own mesh-load loop already drops invisible meshes
    from the whole triangle soup before the locator is even built ("INVISIBLE
    GEOMETRY IS NOT SUPPORT" -- the merged `<cell>/src` subtree a live GAC
    bake keeps composed-but-hidden until export). This pins that the NEW
    backing test inherits that exclusion for free, the same way the old
    downward/lean rays already did -- a real regression here would mean a
    live bake could find "backing" from source geometry that is about to be
    dropped, exactly the kind of live-vs-exported mismatch this fix is
    guarding against (2026-08-31, `fire_dtc3` bench)."""
    if not HAVE_USD:
        print("  invisible_wall       SKIP (no pxr — run under usd_python.sh)")
        return
    from pxr import UsdGeom
    stage, cell = _stage()
    spall = _stamp(stage, cell, "inv", 80.0)
    wall_path = _wall(stage, cell, "inv", 80.0, dx=0.05)
    UsdGeom.Imageable(stage.GetPrimAtPath(wall_path)).MakeInvisible()

    info = fb._judge_candidates(stage, fb.BAKE_ROOT, gap_m=1.0, verbose=False)
    j = _judged_by_path(info)[spall]
    assert j["contact"] is False, j
    assert j["deactivate"] is True, j
    assert j["backing_m"] is None, j
    assert j["backing_path"] is None, j
    print("  invisible_wall       OK  (invisible wall never counted as "
          "backing, deactivated)")


def test_batch_does_not_prop_up_its_own_condemned():
    """THE ACTUAL BAKE-TIME BUG (2026-08-31): `deactivate_airborne` used to
    judge every candidate from ONE snapshot and flip the losers off in a
    single batch, so a stamp whose only "seat" was ANOTHER member of that
    same batch survived if that other member had not been flipped off YET
    at the moment it was checked.

    Reproduces the measured shape exactly: an `sbar` (round bar, no backing
    test of its own -- see `_WALL_BAR_FAMILIES`) floats with nothing under
    or beside it 5 cm under a `spall` stamp that ALSO has nothing behind it.
    In one `_judge_candidates` snapshot the `spall`'s plain downward ray
    calls the `sbar` a seat (`contact=True` from the flush-seat check alone
    -- before the backing branch ever runs) while the `sbar` is
    independently judged unsupported. A single-pass deactivate would turn
    the `sbar` off and leave the `spall` active with a "seat" that no
    longer exists. `deactivate_airborne` must re-judge after each pass and
    catch it on the next one -- this is what
    `tools/airborne_replay_probe.py` proved end-to-end on the real bake
    (`gac_SM_Building_26_F4_s162.usd`: pass 1 caught `sbar_g5_20` among 76,
    pass 2 caught the 14 stamps that had been resting on members of that
    same batch, pass 3 found nothing new)."""
    if not HAVE_USD:
        print("  batch_fixed_point    SKIP (no pxr — run under usd_python.sh)")
        return
    from pxr import Sdf
    stage, cell = _stage()
    # the bar: nothing under or beside it -- an isolated floater
    bar_path = "{0}/sbar_batch_1".format(cell)
    _quad(stage, bar_path, 5.0, -0.05, 0.05, 9.60, 9.80)
    # the stamp: bottom_z 9.85, so the bar's top at 9.80 is 5 cm below it --
    # well inside `_SEAT_TOL_M` (0.15 m), found by the PLAIN DOWNWARD ray,
    # before the stamp's own backing test would ever get a look
    spall = _stamp(stage, cell, "batch", 0.0)

    # sanity: confirm the single-snapshot judge really does show the shape
    # this test is about, before checking `deactivate_airborne` fixes it
    info = fb._judge_candidates(stage, fb.BAKE_ROOT, gap_m=1.0, verbose=False)
    j = _judged_by_path(info)
    assert j[bar_path]["deactivate"] is True, j[bar_path]
    assert j[spall]["deactivate"] is False, j[spall]     # the bug, pre-fix
    assert j[spall]["contact"] is True and j[spall]["backing_m"] is None, \
        j[spall]                          # kept via flush-seat, not backing

    n = fb.deactivate_airborne(stage, fb.BAKE_ROOT, gap_m=1.0, verbose=False)
    assert n == 2, "expected both the bar and the stamp gone, got {0}".format(n)
    bar_active = stage.GetPrimAtPath(Sdf.Path(bar_path)).IsActive()
    spall_active = stage.GetPrimAtPath(Sdf.Path(spall)).IsActive()
    assert bar_active is False, "sbar should be off after pass 1"
    assert spall_active is False, (
        "spall should be off after pass 2 -- its only 'seat' was the sbar, "
        "which pass 1 already condemned")
    print("  batch_fixed_point    OK  (bar off pass 1, stamp off pass 2, "
          "{0} total deactivated)".format(n))


TESTS = [test_wall_backed_at_5cm_is_kept, test_no_wall_is_deactivated,
         test_wall_1m_back_is_too_far_and_deactivated,
         test_bar_family_keeps_the_old_lean_exemption,
         test_invisible_wall_does_not_back,
         test_batch_does_not_prop_up_its_own_condemned]


def main():
    print("test_fire_bake_wall_backing  (pxr {0})".format(
        "present" if HAVE_USD else "ABSENT — all tests skipped"))
    failed = 0
    for t in TESTS:
        try:
            t()
        except AssertionError as exc:
            failed += 1
            print("  {0:<19} FAIL  {1}".format(t.__name__, exc))
        except Exception as exc:
            failed += 1
            import traceback
            traceback.print_exc()
            print("  {0:<19} ERROR {1}".format(t.__name__, exc))
    print("\n{0}/{1} passed".format(len(TESTS) - failed, len(TESTS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
