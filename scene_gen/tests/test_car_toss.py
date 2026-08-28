#!/usr/bin/env python3
"""
test_car_toss.py — what a tornado does to a parked car, pinned without Isaac.

`disaster/tornado.py`'s `car_pose` decides, for one parked vehicle, whether it
moves at all, how far, in what attitude it comes to rest, which way it ends up
pointing and what stopped it. `car_blockers` is the obstacle set it stops
against. Everything in both is pure geometry, which is the whole reason they
are separate from `toss_prim` — the authoring half needs a stage, the DECISION
half does not, and the decision half is the part that can be wrong in a way no
render makes obvious.

Nine claims are worth pinning, and every one of them has already been got
wrong once somewhere in this pipeline:

  1. THE LADDER IS THE SURVEY. Move and tip shares track Paulikas, Schmidlin &
     Marshall 2016 (959 vehicles, 12 tornadoes) at the intensity asked for.
  2. THE TIP SHARE IS UNCONDITIONAL. `p_tip` is a share of ALL cars, not of the
     moved ones — the nesting bug that produced 5 moved and 0 tipped on the
     first real run — and every tipped car is also a moved one.
  3. TWO THIRDS OF THE MOVED CARS ARE MERELY SHOVED. A corridor where every
     car is upside down is as wrong as one where none is.
  4. A SHOVED CAR GOES A CAR LENGTH OR TWO AND A TIPPED ONE GOES FURTHER.
     Wheels-down travel is friction-limited; the survey's long displacements
     belong to the vehicles that left the ground.
  5. THE RESTING HEADING IS SET BY THE OUTCOME. A rolled car ends up with its
     long axis ACROSS the way it travelled (it rolled about that axis); a
     shoved one slews about its own parked heading and does not spin.
  6. A CAR COMES TO REST AGAINST SOMETHING. The march stops at the first
     blocker, names it, and leaves the car outside it rather than inside.
  7. THE `toppled` FLAG IS A PUBLISHED CONTRACT. It gated where
     `tornado_people._in_vehicle` seated occupants — UPRIGHT CARS ONLY —
     until that scenario was cut on 2026-08-27 with the rest of the figures
     the tornado had not hit. The flag itself is still the record's contract
     and is still what any downstream pass reads, so a shoved car — including
     one jammed nose-up against a wall — must never come back toppled.
  8. MASS TELLS. The same wind that rolls a saloon does not roll a 9.5 m bus.
  9. ONE SEED, ONE ANSWER.

RUNS WITHOUT ISAAC. `tornado` imports `pxr` only inside `toss_prim`; nothing
here imports `pxr`, opens a stage or needs a GPU.

USAGE
    python3 scene_gen/tests/test_car_toss.py
    pytest -s scene_gen/tests/test_car_toss.py
"""

import math
import os
import random
import statistics as stats
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import tornado as tn                   # noqa: E402

# The 100 m preset's own numbers, so the samples below are the scene's.
THROW_DEG = 38.0 + 20.0        # heading + curl_deg
THROW_M = 16.0
N = 4000                       # draws per intensity — enough for a 2% band

FAILS = []


def check(cond, msg):
    print(("    PASS  " if cond else "    FAIL  ") + msg)
    if not cond:
        FAILS.append(msg)


def sample(intensity, n=N, seed=7, **kw):
    """*n* independent cars at one intensity, as a list of `car_pose` dicts."""
    rng = random.Random(seed)
    kw.setdefault("long_axis_deg", 0.0)
    return [tn.car_pose(intensity, rng, THROW_DEG, THROW_M, **kw)
            for _ in range(n)]


def _acute(a, b):
    """Angle between two BEARINGS treated as axes, folded into [0, 90]. A car's
    long axis is defined mod 180 — nose forward and nose back are the same
    axis — so a raw bearing difference is the wrong measure of "across"."""
    d = abs(tn._wrap180(a - b))
    return 90.0 - abs(d - 90.0)


def strict(fn):
    """Make a recorded FAIL an actual pytest failure.

    `check` records rather than raises, because a run is far more useful when
    it prints the whole table than when it stops at the first bad row — that
    is the whole point of the script entry point below. Under pytest that
    would make every test pass unconditionally, so each one re-reads the tally
    it started with and asserts nothing was added to it. Same idiom as
    `test_fence_rules.py`, wrapped in a decorator so it cannot be left off a
    test somebody adds later.
    """
    def run(*a, **kw):
        mine = len(FAILS)
        out = fn(*a, **kw)
        assert len(FAILS) == mine, FAILS[mine:]
        return out
    run.__name__ = fn.__name__
    run.__doc__ = fn.__doc__
    return run



# ---------------------------------------------------------------------------
# 1-2. the ladder, and the nesting that ruined it once
# ---------------------------------------------------------------------------

@strict
def test_rates_track_the_survey():
    print("\n[rates] the Paulikas ladder, and it is unconditional")
    # EF3-EF4 in the survey: 63% displaced, 15% rolled or lofted. The field
    # value that stands for it here is the one the ladder is written against.
    for it, want_move, want_tip in ((0.5, 0.41, 0.156), (0.9, 0.658, 0.306)):
        got = sample(it)
        moved = [p for p in got if p["moved"]]
        tipped = [p for p in got if p["toppled"]]
        f_move, f_tip = len(moved) / float(N), len(tipped) / float(N)
        check(abs(f_move - want_move) < 0.025,
              "intensity %.1f: %.3f of cars moved (survey ladder %.3f)"
              % (it, f_move, want_move))
        check(abs(f_tip - want_tip) < 0.025,
              "intensity %.1f: %.3f of ALL cars tipped (ladder %.3f) — the "
              "share is unconditional, not %.3f of the movers"
              % (it, f_tip, want_tip, want_tip))

    # THE NESTING BUG, stated as an inequality. Two independent coins would
    # give p_move * p_tip = 0.20 at it = 0.9, which is a third of the truth.
    got = sample(0.9)
    f_tip = sum(p["toppled"] for p in got) / float(N)
    check(f_tip > 0.658 * 0.306 * 1.5,
          "the tip rate is not p_move x p_tip (%.3f, nested would be %.3f)"
          % (f_tip, 0.658 * 0.306))
    check(all(p["moved"] for p in got if p["toppled"]),
          "every tipped car is also a displaced one")
    check(all(not p["toppled"] for p in got if not p["moved"]),
          "...and a car that did not move did not topple")


@strict
def test_calm_ground_is_untouched():
    print("\n[floor] below the floor nothing happens")
    for it in (0.0, 0.05, tn.CAR_MIN_INTENSITY):
        got = sample(it, n=400)
        check(not any(p["moved"] for p in got),
              "intensity %.2f: no car moved (floor is %.2f)"
              % (it, tn.CAR_MIN_INTENSITY))
        check(all(p["pose"] == "parked" and p["d_m"] == 0.0 for p in got),
              "intensity %.2f: ...and every one is still `parked` at zero "
              "displacement" % it)


# ---------------------------------------------------------------------------
# 3-4. the pose mix and the two distance regimes
# ---------------------------------------------------------------------------

@strict
def test_most_moved_cars_are_only_shoved():
    print("\n[mix] two thirds shoved, and the tipped mix behind it")
    moved = [p for p in sample(0.9) if p["moved"]]
    share = {k: sum(p["pose"] == k for p in moved) / float(len(moved))
             for k in ("shoved", "side", "roof", "nose")}
    check(0.45 < share["shoved"] < 0.65,
          "%.2f of moved cars are merely shoved — not a corridor of upside-"
          "down cars" % share["shoved"])
    tip = [p for p in moved if p["pose"] != "shoved"]
    on_side = sum(p["pose"] == "side" for p in tip) / float(len(tip))
    # A NOSED-IN CAR HAS TO HAVE HIT SOMETHING, so on an EMPTY street the mix
    # collapses into `side` and nothing stands on its nose. That is the point
    # of the demotion (2026-08-27): `nose` was being drawn from the tip mix
    # and then thrown down an open carriageway, and the render was a van
    # standing vertically on its bumper in the middle of the road with
    # nothing in front of it. The mix is asserted against a BLOCKED street
    # below, which is the only place the pose is physical.
    on_nose = sum(p["pose"] == "nose" for p in tip) / float(len(tip))
    check(abs(on_side - (tn.CAR_TIP_MIX[0] + tn.CAR_TIP_MIX[2])) < 0.06,
          "%.2f of tipped cars are on their SIDE on an OPEN street (mix says "
          "%.2f once the un-arrested nose cars are demoted into it)"
          % (on_side, tn.CAR_TIP_MIX[0] + tn.CAR_TIP_MIX[2]))
    check(on_nose == 0.0,
          "%.2f stand on the NOSE with nothing to hit — must be zero"
          % on_nose)


@strict
def test_shoved_and_thrown_are_different_distances():
    print("\n[reach] friction-limited against airborne")
    moved = [p for p in sample(0.9) if p["moved"]]
    sh = [p["d_m"] for p in moved if p["pose"] == "shoved"]
    tp = [p["d_m"] for p in moved if p["pose"] != "shoved"]
    check(max(sh) < 8.0,
          "a shoved car travels at most %.1f m — a car length or two, not a "
          "flight" % max(sh))
    check(stats.median(tp) > stats.median(sh) * 1.5,
          "a tipped car goes further: median %.1f m against %.1f m"
          % (stats.median(tp), stats.median(sh)))
    # The reach has to scale with the field or the corridor has no gradient.
    far = [p["d_m"] for p in sample(0.35) if p["moved"] and p["pose"] != "shoved"]
    check(stats.median(far) < stats.median(tp),
          "...and the throw follows the intensity gradient: %.1f m at 0.35 "
          "against %.1f m at 0.9" % (stats.median(far), stats.median(tp)))


@strict
def test_travel_stays_in_its_cone():
    print("\n[bearing] the debris heading, with the right spread per pose")
    moved = [p for p in sample(0.9) if p["moved"]]
    # `nose` needs a street with something in it — see the demotion note in
    # the mix test — so it is sampled against a wall rather than open road.
    blocked = lambda bx, by: "wall" if bx > 6.0 else None      # noqa: E731
    nosed = [p for p in sample(0.9, blocked=blocked) if p["moved"]]
    for pose in ("shoved", "side", "nose"):
        got = [p for p in (nosed if pose == "nose" else moved)
               if p["pose"] == pose]
        if not got:
            check(False, "%s: no cars of this pose to measure" % pose)
            continue
        worst = max(abs(tn._wrap180(math.degrees(math.atan2(p["dy"], p["dx"]))
                                    - THROW_DEG))
                    for p in got if p["d_m"] > 0.05)
        check(worst <= tn.CAR_SPREAD_DEG[pose] + 0.5,
              "%-6s travels within %.0f deg of the throw heading (cone %.0f)"
              % (pose, worst, tn.CAR_SPREAD_DEG[pose]))
    check(tn.CAR_SPREAD_DEG["shoved"] < tn.CAR_SPREAD_DEG["side"],
          "a shoved car scatters less than a lofted one — it never left the "
          "flow it was standing in")


# ---------------------------------------------------------------------------
# 5. the resting heading
# ---------------------------------------------------------------------------

@strict
def test_a_rolled_car_lies_across_its_travel():
    print("\n[yaw] the resting heading is set by the outcome")
    axis = 17.0                       # an arbitrary parked heading
    moved = [p for p in sample(0.9, long_axis_deg=axis) if p["moved"]]

    rolled = [p for p in moved if p["pose"] in ("side", "roof")]
    across = [_acute(axis + p["yaw_delta_deg"],
                     math.degrees(math.atan2(p["dy"], p["dx"])))
              for p in rolled if p["d_m"] > 0.05]
    check(stats.median(across) > 65.0,
          "a rolled car rests with its long axis ACROSS the way it went "
          "(median %.0f deg from it; an undirected yaw would give ~45)"
          % stats.median(across))

    shoved = [p for p in moved if p["pose"] == "shoved"]
    check(max(abs(p["yaw_delta_deg"]) for p in shoved) <= 38.0,
          "a shoved car SLEWS at most 38 deg about its own heading — four "
          "tyres resist spinning better than they resist sliding")

    # A WALL RIGHT ACROSS THE STREET, so `nose` survives the demotion and its
    # heading can be asserted at all. Without a blocker there are no nosed
    # cars by construction — see the mix test.
    blocked = lambda bx, by: "wall" if bx > 6.0 else None      # noqa: E731
    nosed = [p for p in sample(0.9, long_axis_deg=axis, blocked=blocked)
             if p["moved"] and p["pose"] == "nose"]
    check(bool(nosed), "a blocked street produces nosed-in cars (%d)"
          % len(nosed))
    along = [_acute(axis + p["yaw_delta_deg"],
                    math.degrees(math.atan2(p["dy"], p["dx"])))
             for p in nosed if p["d_m"] > 0.05]
    check(stats.median(along) < 30.0,
          "a nosed-in car points at what it hit (median %.0f deg off its own "
          "travel)" % stats.median(along))

    # The degraded path still has to produce a car, not an exception.
    blind = [p for p in sample(0.9, long_axis_deg=None) if p["moved"]]
    check(len(blind) > 0 and all(abs(p["yaw_delta_deg"]) <= 80.0
                                 for p in blind),
          "with no art offset the yaw degrades to an undirected wobble "
          "rather than to a confidently wrong bearing")


# ---------------------------------------------------------------------------
# 6. coming to rest against something
# ---------------------------------------------------------------------------

@strict
def test_blockers_arrest_the_travel():
    print("\n[jam] a moved car stops against something")
    # A wall of house squarely down-track, 8 m out.
    ta = math.radians(THROW_DEG)
    hx, hy = math.cos(ta) * 8.0, math.sin(ta) * 8.0
    blocked = tn.car_blockers(standing=[(hx, hy, 9.0)])
    check(blocked(hx, hy) == "house" and blocked(hx + 20.0, hy) is None,
          "the blocker index answers `house` on the house and None on the "
          "open lot beside it")
    check(tn.car_blockers(trees=[(0.0, 0.0)])(0.4, 0.0) == "tree",
          "...and a tree bole is a blocker at its trunk radius")
    check(tn.car_blockers(cars=[(0.0, 0.0)])(2.0, 0.0) == "car",
          "...and a car stops against another car, which is the pile a real "
          "track leaves in a gutter")

    moved = [p for p in sample(0.9, blocked=blocked) if p["moved"]]
    jam = [p for p in moved if p["arrested_by"]]
    check(len(jam) > 0 and all(p["arrested_by"] == "house" for p in jam),
          "%d of %d moved cars jammed, and every one names what stopped it"
          % (len(jam), len(moved)))
    # OUTSIDE the wall, not inside it. The probe is the car's NOSE, so the
    # test has to reconstruct the nose — and along the car's OWN travel
    # bearing, not the mean throw heading: the cone is +-52 deg wide for a
    # rolled car, and measuring the nose along the mean bearing puts it up to
    # 1.3 m off the truth and fails a model that is correct.
    half = tn.CAR_REF_LEN_M * 0.5
    depth = []
    for p in jam:
        a = math.atan2(p["dy"], p["dx"]) if p["d_m"] > 1e-9 else ta
        depth.append(4.5 - math.hypot(p["dx"] + math.cos(a) * half - hx,
                                      p["dy"] + math.sin(a) * half - hy))
    # A march step is 0.5 m and the car is backed off 0.35 of it, so the nose
    # may sit up to 0.15 m proud of the nominal disc. Anything past that is a
    # car in somebody's living room.
    check(max(depth) <= tn.CAR_MARCH_M - tn.CAR_JAM_GAP_M + 1e-6,
          "no jammed car is authored inside the wall that stopped it "
          "(deepest nose %.2f m past the face, one march step allows %.2f)"
          % (max(depth), tn.CAR_MARCH_M - tn.CAR_JAM_GAP_M))
    free = [p["d_m"] for p in sample(0.9) if p["moved"]]
    check(stats.median([p["d_m"] for p in jam])
          < stats.median(free) + 1e-9,
          "a jammed car stops short of where the bare throw vector would have "
          "put it (%.1f m against %.1f m)"
          % (stats.median([p["d_m"] for p in jam]), stats.median(free)))


# ---------------------------------------------------------------------------
# 7. the contract tornado_people relies on
# ---------------------------------------------------------------------------

@strict
def test_upright_cars_stay_upright_for_the_people_pass():
    print("\n[contract] `in_vehicle` seats occupants in UPRIGHT cars only")
    ta = math.radians(THROW_DEG)
    blocked = tn.car_blockers(standing=[(math.cos(ta) * 6.0,
                                         math.sin(ta) * 6.0, 9.0)])
    moved = [p for p in sample(0.95, blocked=blocked) if p["moved"]]
    shoved = [p for p in moved if p["pose"] == "shoved"]
    check(shoved and not any(p["toppled"] for p in shoved),
          "%d shoved cars and not one flagged `toppled` — including the ones "
          "jammed nose-up against a wall" % len(shoved))
    check(max(abs(p["pitch_deg"]) for p in shoved) <= tn.CAR_JAM_PITCH_DEG[1],
          "the jam lean tops out at %.0f deg, under the 30 deg line the "
          "launcher's `toppled` flag is drawn at" % tn.CAR_JAM_PITCH_DEG[1])
    jammed = [p for p in shoved if p["arrested_by"]]
    check(any(abs(p["pitch_deg"]) > 1.0 for p in jammed),
          "...and a car shoved hard into a wall does ride up it rather than "
          "stopping dead level")
    check(all(p["toppled"] for p in moved
              if p["pose"] in ("side", "roof", "nose")),
          "every side / roof / nose car IS flagged toppled, so the occupant "
          "planner never seats a figure in one")


# ---------------------------------------------------------------------------
# 8-9. mass, and determinism
# ---------------------------------------------------------------------------

@strict
def test_a_bus_is_harder_to_move_than_a_saloon():
    print("\n[mass] length is the proxy, and it is meant to stop a bus")
    car = sample(0.9, length_m=tn.CAR_REF_LEN_M)
    bus = sample(0.9, length_m=9.48)          # the pool's citybus, measured
    f_car = sum(p["moved"] for p in car) / float(N)
    f_bus = sum(p["moved"] for p in bus) / float(N)
    t_car = sum(p["toppled"] for p in car) / float(N)
    t_bus = sum(p["toppled"] for p in bus) / float(N)
    check(f_bus < f_car * 0.65,
          "a 9.5 m bus moves at %.2f against a saloon's %.2f" % (f_bus, f_car))
    check(t_bus < t_car * 0.65,
          "...and tips at %.2f against %.2f" % (t_bus, t_car))
    van = sample(0.9, length_m=4.84)          # the pool's delivery van
    f_van = sum(p["moved"] for p in van) / float(N)
    check(abs(f_van - f_car) < 0.05,
          "the residential pool (4.60-4.84 m) is within 5% of the reference "
          "— this exists to stop a bus cartwheeling, not to re-rank saloons")


@strict
def test_one_seed_one_answer():
    print("\n[repeatable] a scene is the same scene twice")
    a = sample(0.8, n=300, seed=99)
    b = sample(0.8, n=300, seed=99)
    check(a == b, "300 cars reproduce exactly on the same seed")
    c = sample(0.8, n=300, seed=100)
    check(a != c, "...and a different seed is a different scene")


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------

def main():
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith("test_") and callable(o)]
    print("car toss: %d tests, offline (no pxr, no GPU)" % len(tests))
    broken = []
    for name, fn in tests:
        try:
            fn()
        except Exception as exc:                       # noqa: BLE001
            import traceback
            broken.append(name)
            print("    ERROR %s: %s" % (name, exc))
            print(traceback.format_exc())
    print("\n" + "=" * 72)
    if FAILS or broken:
        for m in FAILS:
            print("  FAILED: " + m)
        for m in broken:
            print("  ERRORED: " + m)
        return 1
    print("  all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
