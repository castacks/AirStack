#!/usr/bin/env python3
"""test_bake_reseat.py — the geometry that puts baked debris back on the ground.

Reported 2026-08-27 on the assembled tornado plate:

    "There is some floating logs (tree debris), etc. Why? place them close to
     z=0, shouldn't be that hard."

MEASURED FIRST, on the shipped library, with a bare-`pxr` probe (no Isaac, no
SimulationApp — see the run-isaac-sim-launcher skill's standalone `pxr`
section). Per mesh: world bbox, and "is there anything overlapping me in plan
whose top lands inside my vertical span". The result named the bug on its own:

    archetype                       meshes   air    air%   z_min
    tree_Shumard_Oak_snapped            43    17   39.5%   -0.98
    tree_Shumard_Oak_limbed             17     6   35.3%   -0.98
    tree_Douglas_Fir_{limbed,snapped}   61,47   0    0.0%   -0.21,-0.52
    tree_{Beech,Aspen,Apple}_*          ...     0    0.0%   -0.00
    every `leaning` / `fallen`          ...     0    0.0%

and inside the two bad files THREE INDEPENDENT STICKS SAT AT EXACTLY z = 0.53
with clear air under them. That is not a solver artefact — a solver does not
leave three unrelated bodies at the same height to the centimetre. It is a
single uniform offset, and `bake.export_object`'s own docstring already records
the same signature from the opposite cause ("every seated stick in
`tree_American_Beech_torched` sat at exactly 0.567 m").

THE MECHANISM. `wood_debris` does not simulate its small pieces at all: it
SEATS them, `piece.bounds[0][2] = ground_z - 0.001`, and hands them back as
static geometry. `export_object(drop_to_ground=True)` then takes the world
min-Z of the FIRST root — the tree — as `rz` and subtracts it from every mesh
in the file. When the tree's own geometry dips below grade (a root flare under
the turf, a low branch past the trunk's base) `rz` is NEGATIVE, and subtracting
it LIFTS the whole file — every analytically-seated stick with it, by exactly
`|rz|`.

The distribution confirms it end to end. Anything that had sunk below `rz` was
rescued individually to exactly `rz`, so it lands at 0.00 — which is the large
cluster of logs at exactly 0.00 in the probe. Anything that had been correctly
seated was ABOVE `rz`, so it kept its relative height and came out floating by
`|rz|`. And `_NO_DROP` (`leaning`, `fallen`) never applies the drop, which is
why those files are clean at every species. The float appears exactly, and
only, where the drop was applied.

So the fix is a separation, not a tolerance: the drop moves the OBJECT, the
debris keeps the absolute Z it was authored at, and `_reseat_roots` — pure
geometry, no solver — is what corrects a piece the solver actually misplaced.

RUNS WITHOUT ISAAC. `_reseat_roots` takes a bbox cache and asks it for exactly
one thing, `ComputeWorldBound(prim).ComputeAlignedRange()`, so a dozen lines of
stand-in stand in for Kit. That is deliberate: it is the one piece of the bake
whose correctness is not obvious (the support test was wrong twice before v3),
and it is the only piece that can be checked without a GPU.

    python3 scene_gen/tests/test_bake_reseat.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))

from disaster import bake                                       # noqa: E402


# --------------------------------------------------------------------------
# the stand-in for UsdGeom.BBoxCache / Usd.Prim
# --------------------------------------------------------------------------
class _Range(object):
    def __init__(self, box):
        self._b = box

    def IsEmpty(self):
        return self._b is None

    def GetMin(self):
        return self._b[0:3]

    def GetMax(self):
        return self._b[3:6]


class _Bound(object):
    def __init__(self, box):
        self._b = box

    def ComputeAlignedRange(self):
        return _Range(self._b)


class _Path(object):
    def __init__(self, s):
        self.pathString = s


class _Prim(object):
    """A root: a path and an axis-aligned box (x0,y0,z0,x1,y1,z1)."""

    def __init__(self, path, box):
        self._p = _Path(path)
        self.box = box

    def GetPath(self):
        return self._p


class _Cache(object):
    def ComputeWorldBound(self, prim):
        return _Bound(prim.box)


def seat(roots, grade=0.0, **kw):
    return bake._reseat_roots(_Cache(), roots, grade, **kw)


def box(x0, y0, z0, x1, y1, z1):
    return (x0, y0, z0, x1, y1, z1)


GROUND = 0.0


class TestSunkAndAirborne(unittest.TestCase):
    """The two corrections, in isolation."""

    def test_a_piece_below_grade_is_raised_onto_it(self):
        # The half-space and CCD are the braces; this is the belt. A fragment
        # the solver pushed through the harness quad at ~20 m/s comes back.
        r = _Prim("/log", box(0, 0, -1.40, 1, 1, -1.10))
        dz = seat([r])
        self.assertAlmostEqual(dz["/log"], 1.40, places=6)

    def test_a_piece_frozen_in_the_air_is_dropped_onto_grade(self):
        r = _Prim("/log", box(0, 0, 0.90, 1, 1, 1.10))
        dz = seat([r])
        self.assertAlmostEqual(dz["/log"], -0.90, places=6)

    def test_a_piece_already_resting_is_not_touched(self):
        # A settled fragment has a few centimetres of daylight under its
        # axis-aligned box simply because the box is bigger than the shape.
        r = _Prim("/log", box(0, 0, 0.04, 1, 1, 0.30))
        self.assertEqual(seat([r]), {})

    def test_a_piece_lands_on_what_is_under_it_not_on_the_lawn(self):
        # Bottom-up placement: the pile is what holds the next piece up.
        pile = _Prim("/pile", box(0, 0, 0.0, 4, 4, 0.80))
        top = _Prim("/top", box(1, 1, 1.60, 3, 3, 1.80))
        dz = seat([pile, top])
        self.assertNotIn("/pile", dz)
        self.assertAlmostEqual(dz["/top"], -(1.60 - 0.80), places=6)


class TestSupportTestV3(unittest.TestCase):
    """The part that was wrong twice. Both wrong answers are pinned."""

    def test_a_wall_stub_does_not_support_what_floats_beside_its_top(self):
        # v2's failure: a stub spanning 0..2.5 m "supported" everything inside
        # its plan footprint at every height. `house_ranch_leveled` went 17.0%
        # airborne to 15.1% and no further.
        stub = _Prim("/stub", box(0, 0, 0.0, 1, 1, 2.50))
        frag = _Prim("/frag", box(0, 0, 1.20, 1, 1, 1.35))
        dz = seat([stub, frag])
        self.assertIn("/frag", dz)
        self.assertAlmostEqual(dz["/frag"], -1.20, places=6)

    def test_walls_do_support_the_roof_they_carry(self):
        # v1's failure: "anything lying entirely below us" dropped every intact
        # gable roof to the lawn, because a roof's box includes its overhanging
        # soffit and the walls' tops are above the roof's own min-z.
        w1 = _Prim("/w1", box(0.0, 0.0, 0.0, 0.3, 6.0, 2.60))
        w2 = _Prim("/w2", box(5.7, 0.0, 0.0, 6.0, 6.0, 2.60))
        roof = _Prim("/roof", box(-0.4, -0.4, 2.40, 6.4, 6.4, 4.20))
        self.assertEqual(seat([w1, w2, roof]), {})

    def test_a_corner_clip_is_not_a_seat(self):
        # The overlap has to be at least `min_ovl` of the SMALLER of the two
        # plan areas, or a passing box counts as a floor.
        post = _Prim("/post", box(0.0, 0.0, 0.0, 4.0, 4.0, 1.00))
        sheet = _Prim("/sheet", box(3.95, 3.95, 1.80, 6.0, 6.0, 1.90))
        dz = seat([post, sheet])
        self.assertIn("/sheet", dz)


class TestFreeze(unittest.TestCase):
    """A windthrown tree's pose is AUTHORED. Reseating it undoes the scene."""

    def test_a_fallen_tree_is_not_stood_back_up(self):
        # `tip_tree` bisects the lean down until the crown is just INTO the
        # turf (`seat_band = (-1.1, -0.15)`), so a fallen tree's box min-z is
        # legitimately below grade. Without `freeze` it reads as "sank through
        # the floor" and every fallen trunk in the corridor gets lifted onto
        # the lawn — which is the one thing that makes it read as windthrow.
        tree = _Prim("/tree", box(-1, -4, -0.42, 14, 4, 2.10))
        dz = seat([tree], freeze=["/tree"])
        self.assertEqual(dz, {})
        self.assertAlmostEqual(seat([tree])["/tree"], 0.42, places=6)

    def test_a_frozen_root_still_holds_a_log_up(self):
        # Frozen means "never moved", not "not there": a log can come to rest
        # ACROSS a fallen trunk and must not be dropped through it.
        trunk = _Prim("/tree", box(0, 0, -0.30, 10, 2, 1.20))
        log = _Prim("/log", box(2, 0, 1.15, 4, 2, 1.45))
        dz = seat([trunk, log], freeze=["/tree"])
        self.assertEqual(dz, {})


class TestPreShift(unittest.TestCase):
    """Measure a root where it is GOING, not where the solver left it."""

    def test_a_dropped_trunk_supports_at_its_dropped_height(self):
        # The tree is about to be dropped 0.50 m by `drop_to_ground`, so its
        # top goes 1.50 -> 1.00. A log lying across it comes down onto the
        # trunk WHERE THE TRUNK ENDS UP.
        trunk = _Prim("/tree", box(0, 0, 0.50, 10, 2, 1.50))
        log = _Prim("/log", box(2, 0, 1.45, 4, 2, 1.75))
        dz = seat([trunk, log], freeze=["/tree"], pre={"/tree": -0.50})
        self.assertAlmostEqual(1.45 + dz["/log"], 1.00, places=6)

    def test_without_the_pre_shift_the_log_is_left_hanging(self):
        # The same case with no `pre`: the log is tested against a trunk that
        # has since moved, is judged supported by the top it USED to have, and
        # is left 0.45 m above the trunk it is supposed to be lying on.
        trunk = _Prim("/tree", box(0, 0, 0.50, 10, 2, 1.50))
        log = _Prim("/log", box(2, 0, 1.45, 4, 2, 1.75))
        self.assertEqual(seat([trunk, log], freeze=["/tree"]), {})


class TestSeatPlan(unittest.TestCase):
    """`_seat_plan` — the two corrections COMPOSED, which is where the bug was.

    A root finishes at `z_src - rz + dz`, so every assertion below is that
    expression. Getting either half right on its own was never the problem.
    """

    def plan(self, roots, **kw):
        return bake._seat_plan(_Cache(), roots, recenter=(11.0, 22.0, GROUND),
                               **kw)

    def final_z(self, root, rz, lift):
        return root.box[2] - rz + lift.get(root.GetPath().pathString, 0.0)

    def test_the_shumard_oak_case_end_to_end(self):
        # The measured file: a tree whose own base is 0.531 m under grade, a
        # stick `wood_debris` seated at ground_z - 0.001, and a log the solver
        # tunnelled through the harness quad to -1.90.
        tree = _Prim("/tree", box(-4, -4, -0.531, 4, 4, 9.0))
        stick = _Prim("/stick", box(2.7, 3.5, -0.001, 2.8, 4.1, 0.139))
        sunk = _Prim("/sunk", box(12, 1, -1.90, 14, 2, -1.60))
        rz, lift = self.plan([tree, stick, sunk], drop_to_ground=True,
                             reseat=True, reseat_first=False)
        self.assertAlmostEqual(rz, -0.531, places=6)
        # the object lands on grade, as `drop_to_ground` promises
        self.assertAlmostEqual(self.final_z(tree, rz, lift), GROUND, places=6)
        # THE STICK STAYS WHERE IT WAS ANALYTICALLY PLACED — not 0.530 up
        self.assertAlmostEqual(self.final_z(stick, rz, lift), -0.001,
                               places=6)
        # and the tunnelled log comes back onto the ground
        self.assertAlmostEqual(self.final_z(sunk, rz, lift), GROUND, places=6)

    def test_a_tree_whose_base_is_at_its_origin_is_unaffected(self):
        # The four clean species. Nothing to compensate, nothing to reseat.
        tree = _Prim("/tree", box(-4, -4, 0.0, 4, 4, 9.0))
        stick = _Prim("/stick", box(2.7, 3.5, -0.001, 2.8, 4.1, 0.139))
        rz, lift = self.plan([tree, stick], drop_to_ground=True, reseat=True,
                             reseat_first=False)
        self.assertAlmostEqual(rz, GROUND, places=6)
        self.assertAlmostEqual(self.final_z(stick, rz, lift), -0.001,
                               places=6)

    def test_a_fallen_tree_keeps_its_press_into_the_turf(self):
        # `_NO_DROP`: no drop at all, and the reseat must not stand it up.
        tree = _Prim("/tree", box(-1, -4, -0.42, 14, 4, 2.1))
        log = _Prim("/log", box(6, 5, 0.85, 7, 6, 1.05))
        rz, lift = self.plan([tree, log], drop_to_ground=False, reseat=True,
                             reseat_first=False)
        self.assertAlmostEqual(rz, GROUND, places=6)
        self.assertAlmostEqual(self.final_z(tree, rz, lift), -0.42, places=6)
        # the log beside it, frozen in the air, still comes down
        self.assertAlmostEqual(self.final_z(log, rz, lift), GROUND, places=6)

    def test_the_root_ball_is_not_lifted_off_the_trunk_it_covers(self):
        # THE REGRESSION `reseat_freeze` EXISTS FOR. `wind_tree` authors the
        # ball centred on `lift = r_plate * 0.5` so it STRADDLES the tipped
        # trunk's open base, which puts its underside at `-r_plate / 2` on
        # purpose. That reads as "sank through the floor", and raising it
        # separates it from the base it exists to cover — while the trunk,
        # frozen, stays put. `wind_tree` publishes both as `res["anchored"]`.
        r_plate = 0.72
        tree = _Prim("/tree", box(-1, -4, -0.36, 14, 4, 2.1))
        ball = _Prim("/ball", box(-0.7, -0.7, -0.5 * r_plate,
                                  0.7, 0.7, 0.5 * r_plate))
        log = _Prim("/log", box(6, 5, 0.85, 7, 6, 1.05))
        rz, lift = self.plan([tree, ball, log], reseat=True,
                             reseat_first=False,
                             reseat_freeze=["/tree", "/ball"])
        self.assertAlmostEqual(self.final_z(ball, rz, lift), -0.5 * r_plate,
                               places=6)
        # ...and without it, the ball is raised clear of the trunk end
        _, bad = self.plan([tree, ball, log], reseat=True, reseat_first=False)
        self.assertAlmostEqual(bad["/ball"], 0.5 * r_plate, places=6)

    def test_the_house_path_reseats_every_root_including_the_first(self):
        # `reseat_first=True`, no drop: a wrecked building was authored on the
        # harness ground and every module is equal.
        mod = _Prim("/mod", box(0, 0, -0.90, 6, 6, 2.10))
        frag = _Prim("/frag", box(1, 1, 3.40, 2, 2, 3.55))
        rz, lift = self.plan([mod, frag], reseat=True)
        self.assertAlmostEqual(self.final_z(mod, rz, lift), GROUND, places=6)
        self.assertAlmostEqual(self.final_z(frag, rz, lift), 2.10 + 0.90,
                               places=6)

    def test_no_flags_means_no_corrections(self):
        tree = _Prim("/tree", box(-4, -4, -0.531, 4, 4, 9.0))
        rz, lift = self.plan([tree])
        self.assertEqual(lift, {})
        self.assertAlmostEqual(rz, GROUND, places=6)


class TestTheShumardOakArithmetic(unittest.TestCase):
    """The measured bug, replayed as pure arithmetic.

    This is `export_object`'s seating block reduced to what it does to a Z:
    the drop takes `rz` from the first root and subtracts it from everything;
    the fix compensates it back out for every root that is not the object.
    """

    RZ = -0.531        # the Shumard's own min-z at bake time
    SEATED = -0.001    # where `wood_debris` puts a seated stick

    def test_the_old_behaviour_reproduces_the_measured_0_53(self):
        # every mesh -> z - rz
        self.assertAlmostEqual(self.SEATED - self.RZ, 0.530, places=3)

    def test_a_sunk_log_landed_at_exactly_zero_under_the_old_rule(self):
        # ...which is the cluster of logs at exactly 0.00 in the probe, and
        # the reason the file looked half-right.
        zr = -1.90                                   # tunnelled through
        lift = self.RZ - zr                          # the old per-root rescue
        self.assertAlmostEqual(zr - self.RZ + lift, 0.0, places=6)

    def test_the_compensation_returns_a_seated_stick_to_the_ground(self):
        # new rule: debris also carries `+(rz - grade)`, so the datum shift
        # cancels and the stick keeps the Z it was authored at.
        comp = self.RZ - GROUND
        self.assertAlmostEqual(self.SEATED - self.RZ + comp, self.SEATED,
                               places=6)

    def test_the_object_itself_still_lands_on_grade(self):
        # The first root gets no compensation, so the drop still does its job.
        self.assertAlmostEqual(self.RZ - self.RZ, GROUND, places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
