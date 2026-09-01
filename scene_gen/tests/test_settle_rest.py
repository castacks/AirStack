#!/usr/bin/env python3
"""test_settle_rest.py — `disaster.settle`'s rest and grade measurements.

    python3 scene_gen/tests/test_settle_rest.py      # host: pxr, no Kit
    pytest -q scene_gen/tests/test_settle_rest.py

WHY THIS FILE EXISTS
--------------------
The `city_smoke43` fire-city smoke bake (2026-08-31) reported two faults on
its GAC cell — 80 bodies "STILL MOVING at bake time" and 131 "BELOW GRADE"
— and both of them were the MEASUREMENT, not the pile:

  * `_z_min` asked `UsdGeom.BBoxCache` where a body's bottom was. That is the
    AABB of the LOCAL extent box's transformed corners, so for a shard lying
    diagonally in its own box it reads far below the lowest real vertex.
    Measured over the two smoke cells' settled bodies, `points_z_min -
    bbox_z_min` was p99 1.03 m (gac) / 0.56 m (kit) — and the "worst body
    below grade" those runs reported was -1.03 m / -0.59 m. The same
    numbers. `_lift_below_grade` then hoisted 131 / 59 bodies by that
    spurious amount, which MANUFACTURES float.
  * the final still-moving check stepped 20 substeps (0.33 s) and flagged
    anything that travelled more than 4 mm — 12 mm/s. Bodies creeping in a
    heap are not "frozen mid-flight", and `fire_bake_launch_script.py`
    DELETES what that check flags.

`.agents/skills/fix-floating-debris` says it in one line: **if a look defect
survives a measurement, suspect the measurement.** These tests pin the
measurements, on the host, with no Isaac Sim — the only way to be sure a
seven-minute bake is testing the fix rather than the weather.

`carb` is stubbed: `settle` imports it at module scope for logging only.
"""
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))

if "carb" not in sys.modules:                      # Kit-only logging shim
    _carb = types.ModuleType("carb")
    _carb.log_warn = lambda *a, **k: None
    _carb.log_error = lambda *a, **k: None
    sys.modules["carb"] = _carb

from disaster import settle                                    # noqa: E402

try:
    from pxr import Gf, Usd, UsdGeom                            # noqa: E402
    HAVE_USD = True
except Exception:                                               # pragma: no cover
    HAVE_USD = False


# ---------------------------------------------------------------------------
# 1. the decomposition budget scales with the piece
# ---------------------------------------------------------------------------
def test_decomp_budget_scales_with_the_piece():
    """One fixed budget cannot serve a 0.8 m chip and a 25 m core shell.

    Measured on the smoke cell, still-moving rate by body diagonal: 6-10%
    below 8 m, 29-71% above it, and every one of those movers was wedged in
    the collapse heap. `_scaled_decomp` must be the IDENTITY below
    `BIG_PIECE_M` (so nothing about a normal fragment changes), must grow
    with size, must stay inside PhysX's own 64-vertex hull ceiling, and must
    be capped so the cooker can never be handed an unbounded job.
    """
    base = settle.DECOMP_LIMITS
    small = settle._scaled_decomp(0.9)
    assert small == base, small
    assert settle._scaled_decomp(settle.BIG_PIECE_M - 0.01) == base

    big = settle._scaled_decomp(24.7)              # SM_Building_19's core shell
    assert big["max_hulls"] > base["max_hulls"], big
    assert big["voxel_resolution"] > base["voxel_resolution"], big
    assert big["vertex_limit"] <= 64, big          # PhysX's own ceiling
    mid = settle._scaled_decomp(12.0)
    assert base["max_hulls"] <= mid["max_hulls"] <= big["max_hulls"], (mid, big)

    huge = settle._scaled_decomp(400.0)            # capped at 4x, not 500x
    assert huge["max_hulls"] == settle._scaled_decomp(
        4.0 * settle.BIG_PIECE_M)["max_hulls"], huge
    assert huge["max_hulls"] <= 4 * base["max_hulls"], huge
    # a caller's explicit override still wins as the BASE it scales from
    ov = settle._scaled_decomp(24.7, {"max_hulls": 2})
    assert ov["max_hulls"] < big["max_hulls"], ov
    print("  decomp budget       OK  (identity <{0:.0f} m, 4x cap, <=64 verts)"
          .format(settle.BIG_PIECE_M))


# ---------------------------------------------------------------------------
# 2. rest is net travel, not a per-chunk delta
# ---------------------------------------------------------------------------
class _FakePile(object):
    """A pile where `n_creep` bodies JITTER in place and `n_travel` TRAVEL.

    The jitterers move `jitter` metres every chunk, alternating direction, so
    their per-chunk delta is large and their NET travel over any even number
    of chunks is zero — which is exactly the population the v1 test called
    "still moving" and the bake then deleted.
    """

    def __init__(self, n_creep=8, n_travel=2, jitter=0.02, speed=0.5):
        self.n_creep, self.n_travel = n_creep, n_travel
        self.jitter, self.speed = jitter, speed
        self.k = 0
        self.frozen = set()

    def paths(self):
        return (["creep_{0}".format(i) for i in range(self.n_creep)]
                + ["go_{0}".format(i) for i in range(self.n_travel)])

    def positions(self):
        out = {}
        sign = 1.0 if (self.k % 2) else -1.0
        for i in range(self.n_creep):
            p = "creep_{0}".format(i)
            out[p] = (0.0, 0.0, 0.0 if p in self.frozen
                      else sign * self.jitter)
        for i in range(self.n_travel):
            p = "go_{0}".format(i)
            out[p] = (0.0, 0.0, 0.0 if p in self.frozen
                      else -self.speed * self.k)
        return out


def _run_phase(pile, chunk=200, cap=3000, **kw):
    """Drive `_settle_phase` against `pile`, with `_step`/`_positions` faked."""
    real_step, real_pos, real_freeze = (settle._step, settle._positions,
                                        settle._freeze_creepers)

    def _step(n, dt=1.0 / 60.0, fabric=False):
        pile.k += 1
        return "fake"

    def _freeze(bodies, paths):
        """Stand-in for the real one: returns only what it ACTUALLY froze."""
        got = [str(q) for q in paths if str(q) not in pile.frozen]
        pile.frozen.update(got)
        return got

    settle._step = _step
    settle._positions = lambda bodies: pile.positions()
    settle._freeze_creepers = _freeze
    try:
        return settle._settle_phase(pile.paths(), chunk, cap, **kw)
    finally:
        (settle._step, settle._positions,
         settle._freeze_creepers) = real_step, real_pos, real_freeze


def test_a_jittering_body_is_at_rest():
    """v1 counts a body rocking 2 cm either way as moving; v2 does not.

    Same pile, same steps. The only difference is which question is asked:
    "did it move since last chunk" (jitter says yes, forever) or "did it get
    anywhere" (jitter says no).
    """
    v1 = _run_phase(_FakePile(n_creep=8, n_travel=2), stall_chunks=3)
    used, moving, _drv, at_rest, reason, frozen = v1
    assert not at_rest and reason == "stalled", (at_rest, reason)
    assert moving == 10, moving          # every body "moving", jitterers too
    assert frozen == [], frozen

    v2 = _run_phase(_FakePile(n_creep=8, n_travel=2), stall_chunks=3,
                    creep_tol=settle.CREEP_TOL_M, creep_window=2,
                    freeze_rounds=0)
    used2, moving2, _d2, rest2, reason2, frozen2 = v2
    assert moving2 == 2, moving2         # only the two that are going somewhere
    assert reason2 == "stalled", reason2  # and they never stop, so: stalled
    print("  jitter is not motion OK  (v1 10 moving / v2 2 moving, same pile)")


def test_a_stall_freezes_the_jitterers_and_carries_on():
    """A stall is a cue, not a verdict.

    v1's answer to a stall was to stop and tell the operator more steps would
    not help — true, and useless, because the pile still had to be baked and
    everything still moving was deleted downstream. v2 freezes the bodies
    that have proved they are going nowhere (they keep their pose, so they
    still hold up whatever is on them) and carries on for the rest.
    """
    pile = _FakePile(n_creep=8, n_travel=2)
    used, moving, _d, at_rest, reason, frozen = _run_phase(
        pile, stall_chunks=3, creep_tol=settle.CREEP_TOL_M, creep_window=2,
        freeze_rounds=settle.CREEP_FREEZE_ROUNDS)
    assert len(frozen) == 8, frozen        # exactly the jitterers
    assert all(f.startswith("creep_") for f in frozen), frozen
    assert pile.frozen == set(frozen), pile.frozen
    # NO DOUBLE COUNTING. A stall can fire several times and each time the
    # whole not-travelling set is offered again; reporting the offer instead
    # of the action printed "1631 FROZEN" on a 561-body pile.
    assert len(frozen) == len(set(frozen)), "frozen list has duplicates"
    assert len(frozen) <= len(pile.paths()), (len(frozen), len(pile.paths()))
    # and the travellers are still counted as moving, not swept up with them
    assert moving == 2, moving
    print("  stall freezes creep  OK  ({0} frozen, {1} still travelling)"
          .format(len(frozen), moving))


def test_everything_at_rest_ends_the_phase():
    """When nothing is travelling the phase must say `rest`, not run to cap."""
    pile = _FakePile(n_creep=10, n_travel=0)
    used, moving, _d, at_rest, reason, frozen = _run_phase(
        pile, chunk=200, cap=3000, stall_chunks=3,
        creep_tol=settle.CREEP_TOL_M, creep_window=2,
        freeze_rounds=settle.CREEP_FREEZE_ROUNDS)
    assert at_rest and reason == "rest", (at_rest, reason)
    assert moving == 0, moving
    assert used < 3000, used
    assert frozen == [], frozen            # nothing had to be frozen
    print("  all-jitter is rest   OK  ({0} steps, reason={1})".format(used, reason))


def test_v1_path_is_untouched():
    """`creep_tol=0` and `freeze_rounds=0` must reproduce v1 exactly.

    The MCE kit look is frozen against the v1 path, so this is the guard
    that says the new code cannot reach a `kit:` bake.
    """
    pile = _FakePile(n_creep=6, n_travel=1)
    a = _run_phase(pile, stall_chunks=3)
    b = _run_phase(_FakePile(n_creep=6, n_travel=1), stall_chunks=3,
                   creep_tol=0.0, freeze_rounds=0)
    assert a[:5] == b[:5], (a[:5], b[:5])
    assert a[5] == [] and b[5] == []
    print("  v1 path unchanged    OK  (creep_tol=0 reproduces, nothing frozen)")


# ---------------------------------------------------------------------------
# 3. grade, measured off points instead of a box
# ---------------------------------------------------------------------------
def _diagonal_sheet_stage(seat_z=0.05, tilt_deg=45.0, half=(2.0, 1.0, 0.05)):
    """A THIN SHEET LYING DIAGONALLY INSIDE ITS OWN LOCAL BOX, seated flat.

    This is the shape the whole defect is about, and a plain box will not
    reproduce it: for a box mesh the extent box IS the geometry, so
    `BBoxCache` and the points agree and nothing is learned. A Voronoi shard
    is not a box — it is a sliver at an angle to its local axes — so its
    extent box is far larger than it is, and rotating that box and
    re-axis-aligning it (which is all `ComputeWorldBound` does) reports a
    bottom metres below the lowest real vertex.

    Built by baking a `tilt_deg` rotation about Y INTO the points (so the
    local extent box is loose around the sheet) and then taking it back out
    in the prim's own transform (so the sheet ends up flat and level). The
    seat is solved from the points, not guessed, so the sheet's true lowest
    vertex sits exactly `seat_z` above grade whatever the numbers are.
    """
    import math

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    mesh = UsdGeom.Mesh.Define(stage, "/World/shard")
    hx, hy, hz = half
    c, sn = math.cos(math.radians(tilt_deg)), math.sin(math.radians(tilt_deg))
    raw = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
           (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    pts = [(x * c + z * sn, y, -x * sn + z * c) for (x, y, z) in raw]
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([4] * 6)
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 5, 4,
                                      2, 3, 7, 6, 0, 3, 7, 4, 1, 2, 6, 5])
    lo = [min(q[k] for q in pts) for k in range(3)]
    hi = [max(q[k] for q in pts) for k in range(3)]
    mesh.CreateExtentAttr([Gf.Vec3f(*lo), Gf.Vec3f(*hi)])
    x = UsdGeom.Xformable(mesh)
    tr = x.AddTranslateOp()
    tr.Set(Gf.Vec3d(0.0, 0.0, 0.0))
    x.AddRotateYOp().Set(-float(tilt_deg))     # undo the bake: sheet is flat
    prim = mesh.GetPrim()
    z0 = settle._points_z_min(prim)            # where it actually is
    tr.Set(Gf.Vec3d(0.0, 0.0, float(seat_z) - z0))
    return stage, prim


def test_bbox_reads_a_seated_slab_as_below_grade():
    """The measurement that produced "131 body(s) finished BELOW GRADE".

    A 4 x 3 x 0.1 m slab tilted 35 degrees and seated at z = 0.30 m has every
    vertex above grade. `UsdGeom.BBoxCache` says its bottom is well under the
    world, because it rotates the extent BOX and re-axis-aligns it — and
    `_lift_below_grade` believes it and hoists the slab by that much.
    """
    if not HAVE_USD:
        print("  bbox vs points      SKIP (no pxr)")
        return
    stage, prim = _diagonal_sheet_stage(seat_z=0.05)
    bc = settle._bbox_cache()
    z_box = settle._z_min(bc, prim)
    z_pts = settle._points_z_min(prim)
    assert z_pts is not None
    assert z_pts > 0.0, z_pts                  # really seated above grade
    assert z_box < -0.10, z_box                # the box says otherwise
    assert (z_pts - z_box) > 0.4, (z_pts, z_box)
    # and the delta is real geometry, not a rounding artefact: it is the
    # AABB of the rotated extent box, i.e. the shard's own diagonal
    # the audit follows the measurement it is given
    n_box, worst_box, _ex = settle._below_grade(stage, [prim], 0.0)
    n_pts, worst_pts, _ex2 = settle._below_grade(stage, [prim], 0.0,
                                                 points=True)
    assert n_box == 1 and n_pts == 0, (n_box, n_pts)
    print("  bbox vs points      OK  (box {0:+.2f} m vs points {1:+.2f} m on a "
          "seated shard)".format(z_box, z_pts))


def test_points_lift_leaves_a_seated_slab_alone():
    """...and the repair follows it too. The box-based lift MOVES a slab that
    was already on the ground; the points-based one does not touch it, and
    still rescues a slab that really is buried."""
    if not HAVE_USD:
        print("  points lift         SKIP (no pxr)")
        return
    stage, prim = _diagonal_sheet_stage(seat_z=0.05)
    z_before = settle._points_z_min(prim)
    n, failed = settle._lift_below_grade(stage, [prim], 0.0, points=True)
    assert (n, failed) == (0, 0), (n, failed)
    assert abs(settle._points_z_min(prim) - z_before) < 1e-9

    n2, failed2 = settle._lift_below_grade(stage, [prim], 0.0, points=False)
    assert n2 == 1 and failed2 == 0, (n2, failed2)
    lifted = settle._points_z_min(prim) - z_before
    assert lifted > 0.4, lifted                # float, manufactured by a repair

    # a slab that really IS buried is still rescued, exactly onto grade
    stage2, sunk = _diagonal_sheet_stage(seat_z=-1.20)
    assert settle._points_z_min(sunk) < 0.0
    n3, f3 = settle._lift_below_grade(stage2, [sunk], 0.0, points=True)
    assert (n3, f3) == (1, 0), (n3, f3)
    assert abs(settle._points_z_min(sunk)) < 1e-6, settle._points_z_min(sunk)
    print("  points lift         OK  (seated shard untouched; box lift floats "
          "it {0:.2f} m; buried shard still rescued)".format(lifted))


TESTS = [test_decomp_budget_scales_with_the_piece,
         test_a_jittering_body_is_at_rest,
         test_a_stall_freezes_the_jitterers_and_carries_on,
         test_everything_at_rest_ends_the_phase,
         test_v1_path_is_untouched,
         test_bbox_reads_a_seated_slab_as_below_grade,
         test_points_lift_leaves_a_seated_slab_alone]


def main():
    print("test_settle_rest  (pxr {0})".format(
        "present" if HAVE_USD else "ABSENT — USD tests skipped"))
    failed = 0
    for t in TESTS:
        try:
            t()
        except AssertionError as exc:
            failed += 1
            print("  {0:<19} FAIL  {1}".format(t.__name__, exc))
        except Exception as exc:                   # pragma: no cover
            failed += 1
            import traceback
            traceback.print_exc()
            print("  {0:<19} ERROR {1}".format(t.__name__, exc))
    print("\n{0}/{1} passed".format(len(TESTS) - failed, len(TESTS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
