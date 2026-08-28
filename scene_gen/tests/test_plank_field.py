#!/usr/bin/env python3
"""
test_plank_field.py — the tornado debris field stays ON THE PLATE.

`planks.scatter_from_wreck` throws a comet tail `reach_m` downtrack of every
wrecked house and knows nothing about the region it is being thrown across.
That is right for the MODEL — a tornado does not stop at a property line and
it does not stop at ours — and wrong for the PICTURE: `suburb_scene.apply_ground`
lays its base sheet over exactly `region` and nothing beyond it, so a board past
the boundary hangs over the void. On the 100 x 100 m plate's first render that
showed as pale rectangles floating in the sky past the downtrack corner, which
is the most conspicuous thing an aerial frame can contain.

`planks.clip_to_region` is the fix and this pins it. Three claims:

  1. IT MEASURES THE WHOLE BOARD, not its centre. A 4 m joist centred a metre
     inside the boundary still puts a metre of itself over the edge.
  2. IT DROPS RATHER THAN CLAMPS. Clamping piles the tail up in a line along
     the boundary, which is a worse artefact than the one being fixed.
  3. IT COSTS ALMOST NOTHING. The share it removes is small by construction —
     if it is not, the scatter is throwing debris off the plate wholesale and
     the reach, not the clip, is what wants looking at.

RUNS WITHOUT ISAAC. `disaster/planks.py` reaches for `pxr` only inside
`materials` and `build`; `scatter_*`, `_box` and `clip_to_region` are pure
Python, which is why they are separate functions.

USAGE
    python3 scene_gen/tests/test_plank_field.py
    pytest -s scene_gen/tests/test_plank_field.py
"""

import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import numpy as np                                  # noqa: E402
from disaster import planks                         # noqa: E402
from disaster import tornado as tn                  # noqa: E402

FAILS = []


def check(cond, msg):
    print(("    PASS  " if cond else "    FAIL  ") + msg)
    if not cond:
        FAILS.append(msg)


# The 100 m close-up, whose corner is where the floating boards appeared.
REGION = (-50.0, -50.0, 50.0, 50.0)
CFG = dict(tn.DEFAULTS, width_m=46.0, core_frac=0.22, throw_m=16.0,
           origin_m=[-18.0, -26.0], heading_deg=38.0, peak=0.89,
           wobble_m=3.2, wobble_period_m=160.0, along_period_m=95.0,
           edge_noise_m=5.0)


def _field(seed=5):
    return tn.intensity_field(CFG, REGION, np.random.default_rng(seed + 23))


def _wrecks(inten, n=6, seed=1):
    """Six wrecked houses inside the corridor, the way the launcher has them."""
    rng = random.Random(seed)
    out = []
    while len(out) < n:
        x, y = rng.uniform(-50.0, 50.0), rng.uniform(-50.0, 50.0)
        if inten(x, y) > 0.5:
            out.append((x, y, 12.0, inten(x, y)))
    return out


def _scatter(seed=5):
    inten = _field(seed)
    rng = random.Random(seed + 77)
    specs = []
    for (x, y, fp, it) in _wrecks(inten, seed=seed):
        specs += planks.scatter_from_wreck(x, y, fp, it, 58.0, 16.0, rng,
                                           n_pieces=140)
    house = len(specs)
    specs += planks.scatter_over_region(REGION, inten, 58.0, rng,
                                        per_100m2=4.5, cell_m=10.0)
    return specs, house


def _outside(spec, region=REGION):
    pts, _n = planks._box(spec)
    return (min(q[0] for q in pts) < region[0]
            or max(q[0] for q in pts) > region[2]
            or min(q[1] for q in pts) < region[1]
            or max(q[1] for q in pts) > region[3])


def test_the_defect_is_real():
    """Unclipped, boards DO leave the plate — or the clip is pinning nothing."""
    print("\n[1] the unclipped scatter really does overhang the plate")
    specs, _h = _scatter()
    off = [s for s in specs if _outside(s)]
    over = max((max(abs(s["x"]) - 50.0, abs(s["y"]) - 50.0) for s in off),
               default=0.0)
    print("      %d board(s), %d overhanging, worst centre overshoot %.1f m"
          % (len(specs), len(off), over))
    check(len(off) > 0,
          "the 100 m plate throws boards off its edge (%d of %d)"
          % (len(off), len(specs)))
    # ...and only a few, or the reach is what wants fixing, not the clip.
    check(len(off) < 0.10 * len(specs),
          "it is a tail, not the field (%.1f%%)"
          % (100.0 * len(off) / len(specs)))


def test_clip_removes_exactly_those():
    """Everything kept is inside; everything dropped was outside."""
    print("\n[2] the clip keeps the inside and drops the outside")
    specs, _h = _scatter()
    kept, dropped = planks.clip_to_region(specs, REGION, verbose=False)
    check(dropped == len(specs) - len(kept),
          "the count it reports is the count it removed (%d)" % dropped)
    check(not any(_outside(s) for s in kept),
          "no kept board has a corner off the plate")
    check(dropped == sum(1 for s in specs if _outside(s)),
          "it removed every overhanging board and nothing else")


def test_it_measures_the_whole_board():
    """A board whose CENTRE is inside but whose end is not must go.

    The whole reason `clip_to_region` calls `_box` rather than testing `x`/`y`.
    A 4 m joist laid along +X with its centre 1 m inside the east boundary
    reaches a metre past it.
    """
    print("\n[3] the test is on the geometry, not on the centre")
    spec = {"x": 49.0, "y": 0.0, "z": 0.05, "l": 4.0, "w": 0.24, "t": 0.02,
            "yaw": 0.0, "pitch": 0.0, "roll": 0.0, "class": "joist"}
    inside_centre = REGION[0] <= spec["x"] <= REGION[2]
    kept, dropped = planks.clip_to_region([spec], REGION, verbose=False)
    print("      centre at x=%.1f is inside; the board reaches x=%.1f"
          % (spec["x"], spec["x"] + spec["l"] / 2.0))
    check(inside_centre, "its centre is legally on the plate")
    check(dropped == 1 and not kept,
          "...and it is dropped anyway, because its end is not")

    # Turned 90 degrees the same board fits, which is the other half of the
    # claim: this is a geometric test and not a margin on the centre.
    spec2 = dict(spec, yaw=90.0)
    kept2, dropped2 = planks.clip_to_region([spec2], REGION, verbose=False)
    check(dropped2 == 0 and len(kept2) == 1,
          "the same board turned along the boundary is kept")


def test_it_is_stable_and_cheap():
    """Deterministic, and it does not eat the field on the big plate either."""
    print("\n[4] stable across seeds, and harmless on the 500 m plate")
    rows = []
    for sd in (5, 6, 7, 8):
        specs, _h = _scatter(sd)
        _k, d = planks.clip_to_region(specs, REGION, verbose=False)
        rows.append((sd, len(specs), d, 100.0 * d / len(specs)))
    for (sd, n, d, pc) in rows:
        print("      seed %d: %4d board(s), %2d dropped (%.1f%%)"
              % (sd, n, d, pc))
    check(all(pc < 10.0 for (_s, _n, _d, pc) in rows),
          "the share stays a tail on every seed (worst %.1f%%)"
          % max(pc for (_s, _n, _d, pc) in rows))

    # The 500 m plate: the same track is a smaller fraction of it, so the
    # clip should be near-inert there. If it ever is not, the reach has grown.
    big = (-250.0, -250.0, 250.0, 250.0)
    cfg = dict(tn.DEFAULTS, width_m=155.0, core_frac=0.22, throw_m=34.0)
    inten = tn.intensity_field(cfg, big, np.random.default_rng(33))
    rng = random.Random(77)
    specs = []
    for _k in range(20):
        while True:
            x, y = rng.uniform(-250, 250), rng.uniform(-250, 250)
            if inten(x, y) > 0.5:
                break
        specs += planks.scatter_from_wreck(x, y, 12.0, inten(x, y), 58.0, 34.0,
                                           rng, n_pieces=140)
    _k2, d2 = planks.clip_to_region(specs, big, verbose=False)
    print("      500 m plate: %d board(s), %d dropped (%.2f%%)"
          % (len(specs), d2, 100.0 * d2 / len(specs)))
    check(d2 < 0.02 * len(specs),
          "near-inert on the plate it was not written for (%.2f%%)"
          % (100.0 * d2 / len(specs)))


def main():
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith("test_") and callable(o)]
    print("plank field: %d tests, offline (no pxr, no GPU)" % len(tests))
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
        for m in FAILS + broken:
            print("  FAILED: " + m)
        return 1
    print("  all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
