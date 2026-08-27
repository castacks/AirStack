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

  no run standing in the neighbour's garden
      REPORTED AS A RATE, not asserted at zero, and it is a question about the
      PLAT rather than about the modules — so it is measured on
      `suburb_parcel`'s own `fence_segs` and not on what `_fence_run` tiled
      along them. A lot is a rectangle hung off one frontage, so two lots hung
      off two frontages of the same block overlap in the corner between them
      and one lot's run then stands inside the other's lot. That is not a
      crossing (the runs need never touch) and not a doubling (they are not
      parallel), so the three tests above are all blind to it, and it is most
      of what "the fences overlap" turns out to mean on a zoom: measured over
      seeds 3/5/8, 627 of 2,858 runs and 11.8 km of fence before
      `junction_skew_clear_deg` existed, 509 and 9.4 km after — and, of
      those, the ones across a SKEWED corner went 119 to 8.
      TWO CAPS, because only one of them is a guarantee. Across ALL corners it
      is a loose ceiling: a corner near square is the pre-existing corner
      behaviour `suburb_parcel` documents and has no packing pass to fix, and
      most of the rest is two lots on one face where the street curves. Across
      a SKEWED corner it is near zero by construction — those lots are refused
      outright — and that cap is what fails if the rule is turned off or
      breaks.

  run continuity
      Consecutive modules along a collinear fence LINE must meet within 25 cm.
      Grouped by line rather than by run because `_fence_run` tiles a single run
      exactly by construction — the question worth asking is whether a lot's run
      meets its neighbour's on the same bearing. Asserted as a rate, not as
      zero: a run legitimately stops at a wall, at a bulb, and at a kerb.

  no fence that encloses nothing  (`fence_fragment`)
      A lot carrying fence modules whose back yard those modules do not close.
      This is the defect the eye reads as "the fences are broken" — 76 of the
      155 fenced lots on seed 3 before the enclosure rework, each of them a
      three-sided U you walk into at either side yard. `build_placements` now
      strips a lot that cannot complete its perimeter, iterating to a fixed
      point because deleting one lot's fence can un-close the neighbour whose
      side line that fence was (4 sweeps on seed 3: 144, 29, 7, 2 lots).

      STATED OVER EVERY LOT WITH A MODULE, WITH NO EXCEPTION, which it could
      not be until the same pass also dropped the gardenless-fenced-lot
      carve-out. A lot with under 4 m of garden behind its back wall has no
      yard that can ever close; while those kept their fence this had to except
      them by name, and an exception written into a test is the defect hiding
      inside the test that states it. They are stripped too now — 55 of them on
      seed 3 — so the rule is the whole rule.

  no seating in an open back yard  (`seating_unenclosed`)
      A garden bench or table set standing in a yard that neither a fence nor a
      treeline encloses. 294 of the 358 patio props on seed 3 stood in one
      before this work; the target is zero and this gates on zero.

      JUDGED BY `suburb_scene._yard_enclosed`, THE SHIPPED PREDICATE, imported
      rather than restated for the reason the whole enclosure rework exists:
      four passes had re-derived "the rear yard" from three different sources
      and no two agreed. Fences are `h["fence_drawn"]` (what went down, not
      what the plat proposed) and trees are the YARD PASS's canopies only —
      `suburb_yardplan` screens a yard with the trees it planted itself and
      indexes nothing else, so a checker counting the parcel pass's verge trees
      as well would be a looser gate than the one being checked.

      ATTRIBUTED BY THE LOT'S OWN LOCAL FRAME, NOT BY NEAREST HOUSE CENTRE.
      A seating group is anchored `patio_side_off_frac` (0.55) of a half width
      off the back door, so on a narrow lot the nearest house centre to it is
      frequently the NEIGHBOUR's: measured on seed 3, nearest-centre reports 1
      prop in an open yard that is in fact sitting in a fenced one, and during
      the seating work that mis-attribution produced two such false positives.
      `fence_png._LotIndex` asks each lot's own `frontage`/`u`/`n`/`lot_width`/
      `lot_depth` instead — the five numbers the lot was ISSUED on, and the same
      five `suburb_scene._lot_lines` strikes the enclosure edges from.
"""

import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from fence_png import (build, modules, classify, house_assets,   # noqa: E402
                       continuity, houses, canopies, seating,
                       seating_unenclosed, fence_fragment, enclosure_state)
# `fence_png` puts scene_gen on the path when it imports; this is the same
# `_corner_deg` the plat gates on, imported rather than restated so the test
# and the rule can never disagree about what "off square" means.
from detail import suburb_parcel as sp                           # noqa: E402

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

# Share of fence RUNS allowed to stand inside a lot that is not their own, at
# ALL corners. A ceiling rather than a guarantee — see the docstring: most of
# what is left is two lots on one face where the street curves, and the corners
# near square that `suburb_parcel` documents and does not pack. Measured after
# `junction_skew_clear_deg`: 16.5, 17.8, 19.1, 22.8% on seeds 3, 5, 8, 1.
MAX_TRESPASS_FRAC = 0.30
# ...and the share allowed at a corner more than `junction_skew_clear_deg` off
# square, which IS a guarantee: those lots are refused outright, so no fence of
# theirs exists to stand anywhere. Not zero, because a cul-de-sac WEDGE lot is
# outside that rule (the wedge machinery owns it) and its radial frontage
# normal can read as a skewed corner against a stem lot's. Measured over seeds
# 3/5/8: 119 of 2,858 runs (4.16%) before, 8 of 2,874 (0.28%) after.
MAX_SKEW_TRESPASS_FRAC = 0.01
# Less than this much of a run inside a neighbour's lot is a lot line landing a
# hair inside another, not a fence in a garden. Half a lot width would be
# absurd; half a metre would be noise.
TRESPASS_MIN_M = 1.0


def _seg_in_quad(a, b, quad):
    """Length of the segment *a*-*b* lying inside a convex quad.

    Half-plane clipping in the segment's own parameter, which is exact for a
    convex polygon and needs nothing from `suburb_parcel` — the same arithmetic
    `_clip_seg` does against a building, asked for a length instead of a cut.
    """
    poly = list(quad)
    ar = 0.0
    for i in range(len(poly)):
        j = (i + 1) % len(poly)
        ar += poly[i][0] * poly[j][1] - poly[j][0] * poly[i][1]
    if ar < 0.0:
        poly = list(reversed(poly))
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for i in range(len(poly)):
        p, q = poly[i], poly[(i + 1) % len(poly)]
        nx, ny = q[1] - p[1], -(q[0] - p[0])        # outward for a CCW ring
        den = nx * dx + ny * dy
        num = nx * (a[0] - p[0]) + ny * (a[1] - p[1])
        if abs(den) < 1e-12:
            if num > 0.0:
                return 0.0
            continue
        t = -num / den
        if den > 0.0:
            t1 = min(t1, t)
        else:
            t0 = max(t0, t)
        if t0 > t1:
            return 0.0
    return max(0.0, t1 - t0) * math.hypot(dx, dy)


def trespass(scene, cell=25.0):
    """`(runs, total_m, skew_runs, worst)` — fence runs in somebody else's lot.

    Lots are hashed by cell so this is not 2,000 runs against 2,000 lots. Only
    a run more than `TRESPASS_MIN_M` inside a foreign lot counts.

    ``skew_runs`` is the subset whose two lots front a corner more than
    `junction_skew_clear_deg` OFF SQUARE — the population `suburb_parcel`
    refuses to plat, and so the one this can hold to near zero. The band
    excludes a "corner" under 20 degrees (two lots back to back across a
    shallow block, which is a block-depth question) and over 160 (two lots on
    one face where the street curves).
    """
    skew_deg = float((scene.get("cfg", {}).get("suburb_parcel") or {}).get(
        "junction_skew_clear_deg", sp.DEFAULTS["junction_skew_clear_deg"]))
    # `fence_png.houses`, not a second copy of the comprehension — the name is
    # now imported at module scope and a local list under the same name would
    # shadow it for the rest of this function.
    hs = houses(scene)
    lots, keys = [], []
    for h in hs:
        q = h.get("lot_corners")
        lots.append(q)
        if not q:
            keys.append(((0.0, 0.0), -1.0))
            continue
        cx = sum(v[0] for v in q) / len(q)
        cy = sum(v[1] for v in q) / len(q)
        keys.append(((cx, cy),
                     max(math.hypot(v[0] - cx, v[1] - cy) for v in q)))
    grid = {}
    for i, (c, r) in enumerate(keys):
        if r < 0.0:
            continue
        rr = int(r // cell) + 1
        gx, gy = int(c[0] // cell), int(c[1] // cell)
        for ax in range(gx - rr, gx + rr + 1):
            for ay in range(gy - rr, gy + rr + 1):
                grid.setdefault((ax, ay), []).append(i)

    n, total, skew, worst = 0, 0.0, 0, []
    for hi, h in enumerate(hs):
        for seg in (h.get("fence_segs") or ()):
            a, b = seg[0], seg[1]
            m = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
            r = math.hypot(b[0] - a[0], b[1] - a[1]) / 2.0
            rr = int(r // cell) + 2
            gx, gy = int(m[0] // cell), int(m[1] // cell)
            cand = set()
            for ax in range(gx - rr, gx + rr + 1):
                for ay in range(gy - rr, gy + rr + 1):
                    cand.update(grid.get((ax, ay), ()))
            best, who = 0.0, None
            for li in cand:
                if li == hi or not lots[li]:
                    continue
                c, lr = keys[li]
                if math.hypot(c[0] - m[0], c[1] - m[1]) > lr + r:
                    continue
                ln = _seg_in_quad(a, b, lots[li])
                if ln > best:
                    best, who = ln, li
            if best > TRESPASS_MIN_M:
                n += 1
                total += best
                worst.append((best, m))
                if who is not None and h.get("n") and hs[who].get("n"):
                    th = sp._corner_deg(h["n"], hs[who]["n"])
                    if 20.0 <= th <= 160.0 and abs(th - 90.0) > skew_deg:
                        skew += 1
    worst.sort(reverse=True)
    return n, total, skew, worst[:8]


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
    tres_n, tres_m, tres_skew, tres_worst = trespass(scene)
    runs = sum(len(h.get("fence_segs") or ())
               for p in scene["parcels"] for h in p["houses"])
    hs = houses(scene)
    trees = canopies(scene)
    state = enclosure_state(scene, trees)
    seat_bad = seating_unenclosed(scene, trees, state)
    frag = fence_fragment(scene)

    stats = {"seed": seed, "modules": len(mods),
             "on_road": len(bad["on_road"]), "in_bulb": len(bad["in_bulb"]),
             "crossing": len(bad["crossing"]), "doubled": len(bad["doubled"]),
             "on_drive": len(bad["on_drive"]),
             # LOTS THAT ACTUALLY KEEP A FENCE. This used to be
             # `sum(assets.values())` — the number of houses that CHOSE an
             # asset in `_fence_pick` — and it read 311 on seed 3 where 121
             # lots have a module on the ground. The gap is the all-or-nothing
             # sweep: a lot picks its asset before its perimeter is laid, and
             # 182 of them are stripped afterwards for a yard they could not
             # close. Reporting the pick made the fence look three times as
             # common as it is, which is the one thing a rate calibration is
             # read off. The pick count is still here as `fence_picked`
             # because it is the denominator `houses_multi_asset` is a count
             # out of.
             "lots_fenced": sum(1 for h in hs if h.get("fence_drawn")),
             "fence_picked": sum(assets.values()),
             "houses_multi_asset": multi, "gate_modules": len(gates),
             # THE ENCLOSURE, which is what the fence is for. `yards_open` is
             # not a failure — a back garden with no fence and no treeline is
             # correct and common — it is the denominator that says whether
             # `seating_unenclosed` reading 0 means the gate works or means
             # every garden in the suburb happens to be enclosed.
             "yards_fenced": sum(1 for v in state.values() if v == "fenced"),
             "yards_screened": sum(1 for v in state.values()
                                   if v == "screened"),
             "yards_open": sum(1 for v in state.values() if v == "open"),
             "fence_fragment": len(frag),
             "seating": len(seating(scene)),
             "seating_unenclosed": len(seat_bad),
             # The PLAT-level defect, on the runs rather than on the modules.
             "runs": runs, "trespass_runs": tres_n,
             "trespass_m": round(tres_m, 1),
             "trespass_frac": round(tres_n / runs, 4) if runs else 0.0,
             "trespass_skew_runs": tres_skew,
             "trespass_skew_frac": (round(tres_skew / runs, 4) if runs
                                    else 0.0),
             "break_pairs": cont["pairs"], "breaks": cont["breaks"],
             "break_frac": round(cont["break_frac"], 4)}

    fails = []
    for key, msg in (("on_road", "fence modules standing on a carriageway"),
                     ("in_bulb", "fence modules inside a turnaround"),
                     ("crossing", "fence modules crossing another fence"),
                     ("doubled", "fence modules doubled on one boundary"),
                     # A FENCE ACROSS ITS OWN DRIVEWAY. Gated, not merely
                     # reported, because the fix is structural and cheap to
                     # lose: `suburb_parcel` derives ONE `drive_off` that the
                     # front opening, the side and rear cuts and the paved
                     # ribbon all read. Anything that re-introduces a second
                     # opinion about where the drive is shows up here first.
                     ("on_drive", "fence modules standing across a driveway")):
        if stats[key]:
            fails.append(f"{stats[key]} {msg}")
            if verbose:
                for i in sorted(bad[key])[:8]:
                    fails.append(f"      at ({mods[i]['x']:.1f}, "
                                 f"{mods[i]['y']:.1f})")
    # THE TWO ENCLOSURE INVARIANTS, gated rather than reported for the same
    # reason `on_drive` is: both are guarantees a single edit can silently
    # take away. `fence_fragment` goes the moment anything stops iterating the
    # all-or-nothing sweep to a fixed point (one sweep alone leaves 38 fenced
    # lots open on seed 3); `seating_unenclosed` goes the moment the seating
    # gate is moved back ahead of the planting slot, because a yard cannot be
    # screened by trees that have not been planted yet.
    if frag:
        fails.append(f"{len(frag)} lots carry fence modules that do not "
                     f"enclose their back yard")
        if verbose:
            for h in frag[:8]:
                fails.append(f"      lot at ({h['c'][0]:.1f}, "
                             f"{h['c'][1]:.1f}), "
                             f"{len(h['fence_drawn'])} spans")
    if seat_bad:
        fails.append(f"{len(seat_bad)} seating props standing in a back yard "
                     f"that is neither fenced nor tree-screened")
        if verbose:
            for (pt, usd) in seat_bad[:8]:
                fails.append(f"      {usd.rsplit('/', 1)[-1]} at "
                             f"({pt[0]:.1f}, {pt[1]:.1f})")
    if multi:
        fails.append(f"{multi} houses showing more than one fence asset")
    if gates:
        fails.append(f"{len(gates)} modules using a gate/doorway asset "
                     f"({mods[gates[0]]['usd'].rsplit('/', 1)[-1]})")
    if stats["trespass_skew_frac"] > MAX_SKEW_TRESPASS_FRAC:
        fails.append(f"{tres_skew}/{runs} fence runs standing inside a lot "
                     f"across a SKEWED corner "
                     f"({stats['trespass_skew_frac']:.1%} > "
                     f"{MAX_SKEW_TRESPASS_FRAC:.0%}) — "
                     f"`junction_skew_clear_deg` should have refused those lots")
    if stats["trespass_frac"] > MAX_TRESPASS_FRAC:
        fails.append(f"{tres_n}/{runs} fence runs standing inside a lot that "
                     f"is not their own ({stats['trespass_frac']:.1%} > "
                     f"{MAX_TRESPASS_FRAC:.0%}, {tres_m:.0f} m of fence)")
        if verbose:
            for (ln, m) in tres_worst:
                fails.append(f"      {ln:.1f} m at ({m[0]:.1f}, {m[1]:.1f})")
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
