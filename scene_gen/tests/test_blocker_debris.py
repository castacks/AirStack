#!/usr/bin/env python3
"""The road-blockage debris does not float — checked on VERTICES, not on specs.

WHY THIS FILE EXISTS. "The debris is floating" has now been reported three
times against the same 30 lines, and each time the check that was run said it
was fine:

  1. **Wrong datum.** `_blocker_debris` lifted everything to a hard-coded
     0.10 m with the comment "the carriageway is at ~0.10" — true on the
     1600 m plat and false everywhere else, because `apply_ground`'s z ladder
     scales with plate SPAN. On the 1 km wildfire preset the asphalt is at
     0.015 m and every piece stood 8.5 cm up.
  2. **Wrong support.** The fix for (1) seated the third of limbs that "rest on
     the trunks" at their CENTRE on a log crown — so a 3 m stick had BOTH ends
     0.8 m in the air, and because every propped piece was seated the same way
     they lined up in a band at crown height. Measured on the authored scene:
     15 of 38 pieces a blockage, ends at 0.62-1.01 m. A spec-level check passed
     it, because per-piece it *was* touching something.
  3. **Wrong geometry.** `log_mesh` jitters every ring vertex radially by
     `rough` (+-17% of the radius), so the lowest VERTEX is not at `z - r`: it
     is anywhere in `z - r(1 +- rough)`, and only lands at the bottom at all
     when `sides` puts a vertex there. On a 0.48 m trunk section that is 8 cm,
     and at this scene's 24-degree sun 8 cm of height is 18 cm of detached
     shadow — which is exactly what the eye reads as floating.

Fault (3) is invisible to any test that asserts on `{p0, p1, r0, r1}`, which
is what the first two rounds of checking did. So this file builds the REAL
vertices through `vegetation.log_points` — the pure half of `log_mesh`, split
out for this — and measures those.

RUNS WITHOUT ISAAC AND WITHOUT PXR. `log_points` is arithmetic; `people.py`
imports `pxr` only inside the functions that need a stage.

USAGE
    python3 scene_gen/tests/test_blocker_debris.py
"""

import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import people as ppl                    # noqa: E402
from disaster import vegetation as veg                # noqa: E402

FAILS = []

#: Every plate size this pipeline builds, as the GROUND height each produces.
#: `_Z_GRASS` is 0.02 and `ground_z_scale` clamps span/1600 into [0.08, 1.0];
#: the 1 km wildfire preset pins 0.15, giving 0.003 m. Grass, not asphalt: the
#: litter is mostly on the verge, and the ladder puts grass below the road. A
#: seating rule that is only right at one of these is the bug this file opens
#: with.
ROAD_Z = (0.02 * 0.08, 0.003, 0.02 * 0.625, 0.02 * 1.0)

SEEDS = range(16)


def check(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAILS.append(msg)


# ---------------------------------------------------------------------------
# the measurement
# ---------------------------------------------------------------------------

def pieces(seed, road_z, index=0, half_w=5.5):
    """The real planner output for one blockage."""
    return ppl._blocker_debris(0.0, 0.0, 90.0, half_w,
                               random.Random(seed), road_z=road_z,
                               index=index)


def verts(spec, index, j):
    """The vertices the AUTHOR will write for this piece.

    Same side count and same seed `scene_api._place_debris` uses, which is the
    whole point of `vegetation.piece_seed` existing.
    """
    return veg.log_points(spec["p0"], spec["p1"], spec["r0"], spec["r1"], 1.0,
                          sides=spec["sides"],
                          rng=random.Random(veg.piece_seed(spec["kind"],
                                                           index, j)),
                          rough=ppl._DEBRIS_ROUGH)


def lowest(spec, index, j):
    return min(q[2] for q in verts(spec, index, j))


def _end_heights(spec, index, j):
    """Lowest vertex within each END third of the piece, along its own axis."""
    ax, ay, az = spec["p0"]
    bx, by, bz = spec["p1"]
    dx, dy, dz = bx - ax, by - ay, bz - az
    L2 = dx * dx + dy * dy + dz * dz or 1.0
    lo_a, lo_b = 1e9, 1e9
    for q in verts(spec, index, j):
        t = ((q[0] - ax) * dx + (q[1] - ay) * dy + (q[2] - az) * dz) / L2
        if t <= 0.34:
            lo_a = min(lo_a, q[2])
        elif t >= 0.66:
            lo_b = min(lo_b, q[2])
    return lo_a, lo_b


def _supports(spec, others, index):
    """Height of the highest OTHER piece's crown under a point, or None."""
    tops = []
    for j, o in enumerate(others):
        if o is spec:
            continue
        tops.append((o, j))
    return tops


# ---------------------------------------------------------------------------

def test_01_nothing_hangs_over_the_carriageway():
    """No piece has BOTH of its ends clear of the ground.

    The load-bearing claim, and the one round 2 failed. One end up is a
    lean-to; two ends up is a floater, whatever it is nominally resting on.
    """
    print("\n[1] no piece has both ends off the ground, at any plate size")
    worst = 0.0
    n = 0
    for road_z in ROAD_Z:
        bad = 0
        for seed in SEEDS:
            sp = pieces(seed, road_z)
            for j, p in enumerate(sp):
                n += 1
                a, b = _end_heights(p, 0, j)
                clear = min(a, b) - road_z
                if clear > 0.05:
                    bad += 1
                    worst = max(worst, clear)
        check(bad == 0,
              "ground_z %.3f m: %d piece(s) with both ends >5 cm clear" %
              (road_z, bad))
    print("      (%d pieces measured; worst both-ends clearance %.3f m)"
          % (n, worst))


def test_02_every_piece_touches_the_ground_or_a_log():
    """Its lowest vertex is at or under the carriageway, or it is a lean-to.

    A lean-to is allowed to have its low end on the road and its high end on a
    trunk section; nothing else may be above the road at all.
    """
    print("\n[2] every piece is seated: lowest vertex at or below the road")
    for road_z in ROAD_Z:
        hi = []
        for seed in SEEDS:
            sp = pieces(seed, road_z)
            for j, p in enumerate(sp):
                lo = lowest(p, 0, j)
                if lo > road_z:
                    hi.append(round(lo - road_z, 4))
        check(not hi, "ground_z %.3f m: %d piece(s) whose lowest vertex is "
                      "above the ground%s" %
              (road_z, len(hi), (" (worst %.3f m)" % max(hi)) if hi else ""))


def test_03_and_nothing_is_buried():
    """The other failure mode. A piece sunk half its own girth into the tarmac
    is as wrong as one hovering over it, and a seating pass that only ever
    pushes DOWN is exactly how you get there."""
    print("\n[3] and nothing is buried more than its own girth")
    for road_z in ROAD_Z:
        deep = []
        for seed in SEEDS:
            sp = pieces(seed, road_z)
            for j, p in enumerate(sp):
                lo = lowest(p, 0, j)
                r = max(p["r0"], p["r1"])
                if lo < road_z - 2.0 * r:
                    deep.append(round(road_z - lo, 3))
        check(not deep, "road_z %.3f m: %d piece(s) sunk past a full "
                        "diameter" % (road_z, len(deep)))


def test_04_the_seating_uses_the_authored_vertices():
    """The regression guard for fault (3).

    Seat on `z - r` instead of on the vertices and this fails: the radial
    jitter alone puts the lowest vertex up to `rough * r` above the nominal
    bottom, and nothing in the spec says so.
    """
    print("\n[4] seating measures vertices, not radii")
    road_z = 0.015
    gap = []
    for seed in SEEDS:
        for j, p in enumerate(pieces(seed, road_z)):
            nominal = min(p["p0"][2] - p["r0"], p["p1"][2] - p["r1"])
            gap.append(lowest(p, 0, j) - nominal)
    check(max(abs(g) for g in gap) > 0.005,
          "the vertex bottom and the nominal bottom genuinely differ "
          "(max %.3f m) — a spec-level test cannot see this" %
          max(abs(g) for g in gap))
    # and the seated result is right despite that
    for seed in SEEDS:
        for j, p in enumerate(pieces(seed, road_z)):
            if lowest(p, 0, j) > road_z:
                check(False, "seed %d piece %d floats" % (seed, j))
                return
    check(True, "and every piece is still seated once it is accounted for")


def test_05_the_author_builds_the_piece_that_was_measured():
    """`piece_seed` is shared, and it is deterministic across processes.

    `scene_api._tube` seeded the jitter with `abs(hash(prim_path))`. Python
    randomises `hash()` of a str per process unless `PYTHONHASHSEED` is pinned,
    so the log the planner seated was not the log the author built — and the
    difference is exactly the quantity fault (3) is about.
    """
    print("\n[5] planner and author agree on the mesh")
    check(veg.piece_seed("log", 2, 5) == veg.piece_seed("log", 2, 5),
          "piece_seed is stable within a process")
    check(veg.piece_seed("log", 2, 5) != veg.piece_seed("limb", 2, 5),
          "and separates the two kinds")
    check(veg.piece_seed("log", 2, 5) != veg.piece_seed("log", 3, 5),
          "and separates two blockages")
    body = "\n".join(l for l in _tube_src().splitlines()
                      if not l.strip().startswith("#"))
    body = body.split('"""')[0] + body.split('"""')[-1]
    check("hash(" not in body,
          "scene_api._tube no longer seeds off hash(prim_path)")
    check("seed=veg.piece_seed(" in _place_src(),
          "and _place_debris passes the shared seed")
    check('sides=d.get("sides"' in _place_src(),
          "and the side count comes from the spec, not from a local default")


def _api_src():
    p = os.path.join(_SCENE_GEN, "scene_api.py")
    return open(p).read()


def _tube_src():
    s = _api_src()
    i = s.index("def _tube(")
    return s[i:s.index("\ndef ", i + 5)]


def _place_src():
    s = _api_src()
    i = s.index("def _place_debris(")
    return s[i:s.index("\ndef ", i + 5)]


def test_06_the_datum_follows_the_plate():
    """Fault (1). `_road_z` is `_Z_ASPHALT * z_scale`, never a constant."""
    print("\n[6] the carriageway height is derived, not hard-coded")
    src = open(os.path.join(_SCENE_GEN, "disaster", "people.py")).read()
    i = src.index("def _road_z(")
    body = src[i:src.index("\ndef ", i + 5)]
    check("_Z_GRASS_M" in body,
          "_road_z uses the GRASS datum, not the road — most of the litter "
          "is on the verge")
    # AND THE COPY MUST NOT DRIFT. `people.py` may not import `suburb_scene`
    # (it pulls in pxr and breaks the host-side plan), so the constant is
    # duplicated — which is only safe if something checks it.
    import re as _re
    src2 = open(os.path.join(_SCENE_GEN, "suburb_scene.py")).read()
    m = _re.search(r"^_Z_GRASS\s*=\s*([0-9.]+)", src2, _re.M)
    check(m is not None and abs(float(m.group(1)) - ppl._Z_GRASS_M) < 1e-12,
          "people._Z_GRASS_M (%.3f) still matches suburb_scene._Z_GRASS (%s)"
          % (ppl._Z_GRASS_M, m.group(1) if m else "?"))
    check("z_scale" in body, "and scales it by the plate's z_scale")
    check("lift = 0.10" not in src, "the 0.10 m constant is gone")

    class _P:
        ctx = {"z_scale": 0.15}
    check(abs(ppl._road_z(_P()) - 0.003) < 1e-9,
          "z_scale 0.15 -> ground at 0.003 m (the 1 km wildfire preset)")

    class _Q:
        ctx = {}
    check(abs(ppl._road_z(_Q()) - 0.02) < 1e-9,
          "and an absent scale falls back to the 1600 m plat's 0.02 m")


def test_07_deterministic():
    print("\n[7] one seed, one field")
    a = pieces(4, 0.015)
    b = pieces(4, 0.015)
    check(all(x["p0"] == y["p0"] and x["p1"] == y["p1"]
              for x, y in zip(a, b)) and len(a) == len(b),
          "two runs of the same seed produce the identical field")


def test_08_the_field_is_still_a_blockage():
    """Guard against 'fix the float by deleting the debris'."""
    print("\n[8] and it is still a road blockage")
    sp = pieces(3, 0.015)
    logs = [p for p in sp if p["kind"] == "log"]
    limbs = [p for p in sp if p["kind"] == "limb"]
    check(len(logs) == 4, "4 trunk sections across the lanes")
    check(len(limbs) == 34, "34 limbs between and on them")
    prop = 0
    for j, p in enumerate(sp):
        a, b = _end_heights(p, 0, j)
        if max(a, b) - 0.015 > 0.15:
            prop += 1
    check(prop >= 4,
          "%d piece(s) lean with one end up — a flat mat is not a blockage "
          "either" % prop)


def main():
    for fn in (test_01_nothing_hangs_over_the_carriageway,
               test_02_every_piece_touches_the_ground_or_a_log,
               test_03_and_nothing_is_buried,
               test_04_the_seating_uses_the_authored_vertices,
               test_05_the_author_builds_the_piece_that_was_measured,
               test_06_the_datum_follows_the_plate,
               test_07_deterministic,
               test_08_the_field_is_still_a_blockage):
        fn()
    print("\n" + "=" * 70)
    if FAILS:
        print("FAILED %d check(s)" % len(FAILS))
        for m in FAILS:
            print("  - " + m)
        return 1
    print("all passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
