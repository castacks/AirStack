"""fence_check.py — assertions on the lot fences. Exits non-zero on failure.

    python3 tools/fence_check.py                    # seeds 1,3,5,7
    python3 tools/fence_check.py --seeds 3 --verbose

Runs the same host-side scene `tools/fence_png.py` draws — same `build`, same
`classify`, same thresholds — so a failure here can always be looked at:

    python3 tools/fence_check.py --seeds 5
    python3 tools/fence_png.py --seed 5 --out _plans/f5.png

THE INVARIANTS, and why each is one.

  no module on the carriageway
      Tested POSITIVELY against the street centrelines, not against the block
      polygon: the polygon is nominally the kerb but bulges over it at
      `offset_polygon`'s mitre limit, and a lot line is a straight chord across
      a face that curves. Threshold is the kerb itself (margin 0) — the pass
      keeps `_FENCE_ROAD_MARGIN_M` = 0.5 m of daylight, so this asserts the
      guarantee with room to spare rather than restating the constant.

  no module inside a cul-de-sac turnaround
      The paved disc, not the disc plus its `bulb_margin_m` front yard: the
      margin is a siting preference, the paving is a fact.

  no module crossing another
      Hairline cores, `_FenceGrid`'s own test. Two runs meeting at a lot corner
      interpenetrate by half a panel thickness and must pass; two runs crossing
      must not. See `_FenceGrid` for why the ribbon separates them.

  no two fences down one boundary
      Near-parallel, within `_FenceGrid._DOUBLE_M`, overlapping along the line.
      Invisible to the crossing test and most of what reads as "the fences
      overlap" — 148 modules on seed 3 before this pass existed.

  one fence asset per house
      A lot with a picket down one side and a railing across the front reads as
      a repair. `_fence_pick` chooses once per HOUSE, over every tag.

  no gate or doorway asset
      `_fence_run` repeats ONE module, so a gate in the module is a gate every
      few metres and the air around it is a hole every few metres. The named
      uid is the one that was in `lot_fences` and caused exactly that.

  run continuity
      Consecutive modules along a collinear fence LINE must meet within 25 cm.
      Grouped by line rather than by run because `_fence_run` tiles a single run
      exactly by construction — the question worth asking is whether a lot's run
      meets its neighbour's on the same bearing. Asserted as a rate, not as
      zero: a run legitimately stops at a wall, at a bulb, and at a kerb.
"""

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from fence_png import (build, modules, classify, house_assets,   # noqa: E402
                       continuity)

# Assets that are NOT plain repeating panels. `bd8afe2f` — Objaverse "Wooden
# Fence" — is a lone post, a gap, a picket panel, a gap and a solid GATE LEAF
# inside one 5.95 m bbox; it shipped in `lot_fences` and a tiled run read as
# panel/gate/panel/gate with a metre of nothing between each. Anything matching
# these substrings is banned from lot fencing outright.
GATE_ASSETS = ("bd8afe2fa6c24d238a761bc4751ebb03", "_Door", "_Doorway",
               "_Gate", "_Hole")

# A run stops at a wall, at a turnaround and at the kerb, so some breaks in a
# collinear line are correct. 3% is comfortably above the 0.8% the pass measures
# and comfortably below the 1.8% it measured before the rework.
MAX_BREAK_FRAC = 0.03


def check(seed, config_name="suburb_net", verbose=False):
    """`(stats, failures)` for one seed."""
    scene = build(seed, config_name)
    mods = modules(scene)
    bad = classify(scene, mods)
    assets = house_assets(scene)
    cont = continuity(scene, mods)
    gates = [i for i, m in enumerate(mods)
             if any(g in m["usd"] for g in GATE_ASSETS)]
    multi = sum(n for k, n in assets.items() if k > 1)

    stats = {"seed": seed, "modules": len(mods),
             "on_road": len(bad["on_road"]), "in_bulb": len(bad["in_bulb"]),
             "crossing": len(bad["crossing"]), "doubled": len(bad["doubled"]),
             "houses_fenced": sum(assets.values()),
             "houses_multi_asset": multi, "gate_modules": len(gates),
             "break_pairs": cont["pairs"], "breaks": cont["breaks"],
             "break_frac": round(cont["break_frac"], 4)}

    fails = []
    for key, msg in (("on_road", "fence modules standing on a carriageway"),
                     ("in_bulb", "fence modules inside a turnaround"),
                     ("crossing", "fence modules crossing another fence"),
                     ("doubled", "fence modules doubled on one boundary")):
        if stats[key]:
            fails.append(f"{stats[key]} {msg}")
            if verbose:
                for i in sorted(bad[key])[:8]:
                    fails.append(f"      at ({mods[i]['x']:.1f}, "
                                 f"{mods[i]['y']:.1f})")
    if multi:
        fails.append(f"{multi} houses showing more than one fence asset")
    if gates:
        fails.append(f"{len(gates)} modules using a gate/doorway asset "
                     f"({mods[gates[0]]['usd'].rsplit('/', 1)[-1]})")
    if cont["break_frac"] > MAX_BREAK_FRAC:
        fails.append(f"{cont['breaks']}/{cont['pairs']} consecutive modules "
                     f"more than 0.25 m apart ({cont['break_frac']:.1%} > "
                     f"{MAX_BREAK_FRAC:.0%})")
    if not mods:
        fails.append("no fence modules placed at all")
    return stats, fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="1,3,5,7")
    ap.add_argument("--config", default="suburb_net")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    bad = 0
    for seed in (int(s) for s in args.seeds.split(",")):
        stats, fails = check(seed, args.config, args.verbose)
        print(f"[fence_check] seed {seed}: " + "  ".join(
            f"{k}={v}" for k, v in stats.items() if k != "seed"))
        for f in fails:
            print(f"  FAIL  {f}")
        bad += len(fails)
    print(f"[fence_check] {'FAILED' if bad else 'OK'} — {bad} failure(s)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
