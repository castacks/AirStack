#!/usr/bin/env python3
"""Which library assets can a scene's districts ACTUALLY place — and which can't?

    python3 scene_gen/tools/asset_fit_audit.py --config downtown_gac

WHY THIS EXISTS
---------------
"A lot of the building assets are not being used" has two completely different
causes and they need opposite fixes:

  * STATISTICAL — the asset fits plenty of blocks and merely lost the draw.
    Fix by tuning the height profile or the repeat penalties.
  * MECHANICAL — the asset cannot be placed on any block its typology is ever
    given, whatever the dice say. No amount of tuning helps, and it is INVISIBLE
    in a placement histogram: a model with zero placements looks the same
    either way.

A big asset is the usual mechanical casualty and the failure is quiet. Two ways
it happens here, both measured rather than hypothetical:

  1. The block is too small. `_pack_free` only ever considers a candidate whose
     footprint fits the free rectangle in one of its four yaws.
  2. THE PERIMETER BAND, which is the subtle one. `_perimeter_rects` carves a
     frontage band `perimeter_depth_m` deep and leaves the middle as courtyard,
     so on a block deep enough to be carved, the deepest thing that can be
     placed is the BAND, not the block. An asset deeper than the band is
     excluded from every block big enough to have one — that is, from exactly
     the blocks that look like they should hold it.

So this walks the real thing: the same `_pool_entries` the generator uses (so
the yaw-offset sx/sy swap is applied identically), the same `_perimeter_rects`,
the same terrace band, against the blocks the layout ACTUALLY produced for that
typology rather than against the configured targets, which are only a request.

READ THE `fit%` COLUMN, NOT THE `placed` COLUMN. `placed` is one seed's dice;
`fit%` is the share of that typology's real blocks on which the asset could be
placed at all, and it is the number that separates the two causes.
"""

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.dirname(_HERE))

import plan_png as P                                            # noqa: E402
from detail import districts as D                               # noqa: E402


def _fits(sx, sy, w, h, tol=0.01):
    """Either yaw. `_pack_free` tries 0/90/180/270, so both orientations."""
    return ((sx <= w + tol and sy <= h + tol)
            or (sy <= w + tol and sx <= h + tol))


def audit(cfg, layout, res, verbose=False):
    dcfg = cfg.get("districts") or {}
    typs = dcfg.get("typologies") or {}
    typ_of = layout.get("_typology_of") or {}
    inset = D.block_inset(cfg, res)
    cache = {}

    rows = []
    for tname in sorted(typs):
        typ = typs[tname]
        pool = D._pools_for(cfg, res, typ.get("pools") or [tname], cache)
        if not pool:
            rows.append((tname, "(POOL EMPTY)", 0, 0, 0, 0.0, "POOL EMPTY"))
            continue
        blocks = [b for b, t in typ_of.items() if t == tname]
        rects = [(b[0] + inset, b[1] + inset, b[2] - inset, b[3] - inset)
                 for b in blocks]
        terrace = str(typ.get("morphology", "pack")) == "terrace"
        band = float(typ.get("perimeter_depth_m", 0.0))
        min_side = min(min(e[3]["sx"], e[3]["sy"]) for e in pool)
        # The terrace band is derived from the pool's DEEPEST member, exactly
        # as `_lay_terrace` does — one deep member moves it for everyone.
        depth0 = max(e[3]["sx"] for e in pool) if terrace else 0.0
        alley = float(typ.get("alley_m", 6.0))
        alley_max = float(typ.get("alley_max_m", 0.0))

        for e in pool:
            usd = os.path.basename(e[0])
            sx, sy, sz = e[3]["sx"], e[3]["sy"], e[3]["sz"]
            n_ok = 0
            for r in rects:
                w, h = r[2] - r[0], r[3] - r[1]
                if terrace:
                    lo = 2.0 * depth0 + alley
                    hi = 2.0 * depth0 + (alley_max if alley_max > alley
                                         else alley * 2.5)
                    short = min(w, h)
                    # the block must take a pair, and this house must be no
                    # deeper than the strip the pair is cut at
                    if lo <= short <= hi and sx <= depth0 + 0.01:
                        n_ok += 1
                    continue
                ok = False
                for sub in D._perimeter_rects(r, band, min_side):
                    if _fits(sx, sy, sub[2] - sub[0], sub[3] - sub[1]):
                        ok = True
                        break
                n_ok += 1 if ok else 0
            frac = (n_ok / len(rects)) if rects else 0.0
            note = ""
            if not rects:
                note = "no block zoned"
            elif n_ok == 0:
                note = "IMPOSSIBLE"
                if band > 0.0 and min(sx, sy) > band:
                    note = "IMPOSSIBLE (deeper than perimeter band %.0f m)" % band
                elif terrace and sx > depth0 + 0.01:
                    note = "IMPOSSIBLE (deeper than terrace strip %.1f m)" % depth0
            elif frac < 0.20:
                note = "unlikely"
            rows.append((tname, usd, sz, sx, sy, frac, note))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="downtown_gac")
    ap.add_argument("--only-problems", action="store_true")
    ap.add_argument("--seeds", type=int, default=0,
                    help="run this many seeds and report a PLACEMENT "
                         "PROBABILITY per asset instead of one seed's count")
    a = ap.parse_args()

    # ONE SEED IS ONE ROLL OF THE DICE. An asset with zero placements in a
    # single run may be impossible, unlikely, or simply unlucky, and the three
    # need opposite fixes — so sweep seeds and report P(placed), which
    # separates "never generated" from "generated one run in three".
    if a.seeds > 0:
        import statistics
        per_seed = []
        cfg = layout = res = None
        for k in range(a.seeds):
            cfg, layout, placements, res = P.build(a.config, seed=1000 + k)
            c = {}
            for p in placements:
                if p.get("category") == "house":
                    u = os.path.basename(str(p.get("usd", "")))
                    c[u] = c.get(u, 0) + 1
            per_seed.append(c)
            print("  seed %d: %d models placed" % (1000 + k, len(c)),
                  file=sys.stderr)
        rows = audit(cfg, layout, res)
        seen = set()
        out = []
        for tname, usd, sz, sx, sy, frac, note in rows:
            if usd in seen:
                continue
            seen.add(usd)
            hits = [c.get(usd, 0) for c in per_seed]
            p_placed = sum(1 for h in hits if h) / float(len(hits))
            out.append((p_placed, statistics.mean(hits), max(hits),
                        tname, usd, sz, sx, sy, frac))
        out.sort()
        print("\n%-30s %7s %7s %7s %6s %8s %7s %5s  %s"
              % ("asset", "H", "sx", "sy", "fit%", "P(placed)", "mean", "max",
                 "verdict"))
        print("-" * 104)
        bad = 0
        for pp, mn, mx, tname, usd, sz, sx, sy, frac in out:
            v = ("NEVER GENERATED" if pp == 0.0 else
                 "rare" if pp < 0.34 else
                 "uncommon" if pp < 0.67 else "")
            if v:
                bad += 1
            if a.only_problems and not v:
                continue
            print("%-30s %7.1f %7.1f %7.1f %5.0f%% %7.0f%%  %7.2f %5d  %s"
                  % (usd[:30], sz, sx, sy, 100 * frac, 100 * pp, mn, mx, v))
        print("-" * 104)
        print("%d of %d asset(s) flagged over %d seeds"
              % (bad, len(out), a.seeds))
        return

    cfg, layout, placements, res = P.build(a.config)
    counts = {}
    for p in placements:
        if p.get("category") == "house":
            u = os.path.basename(str(p.get("usd", "")))
            counts[u] = counts.get(u, 0) + 1

    rows = audit(cfg, layout, res)
    print("\n%-14s %-30s %7s %7s %7s %6s %7s  %s"
          % ("typology", "asset", "H", "sx", "sy", "fit%", "placed", "verdict"))
    print("-" * 118)
    bad = 0
    for tname, usd, sz, sx, sy, frac, note in rows:
        # "no block zoned" is a ZONING choice, not a fit failure — the probe
        # preset deliberately zones two typologies and leaves the rest unbuilt.
        # Reporting it as a problem buries the real ones.
        if a.only_problems and (not note or note == "no block zoned"):
            continue
        if note.startswith("IMPOSSIBLE") or note == "unlikely":
            bad += 1
        print("%-14s %-30s %7.1f %7.1f %7.1f %5.0f%% %7d  %s"
              % (tname, usd[:30], sz, sx, sy, 100.0 * frac,
                 counts.get(usd, 0), note))
    print("-" * 118)
    print("%d asset/typology pair(s) flagged" % bad)


if __name__ == "__main__":
    main()
