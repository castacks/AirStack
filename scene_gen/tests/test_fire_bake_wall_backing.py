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


TESTS = [test_wall_backed_at_5cm_is_kept, test_no_wall_is_deactivated,
         test_wall_1m_back_is_too_far_and_deactivated,
         test_bar_family_keeps_the_old_lean_exemption]


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
