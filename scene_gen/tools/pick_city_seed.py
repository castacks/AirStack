#!/usr/bin/env python3
"""pick_city_seed.py — sweep seeds, score each city, rank them.

A 1 km plate has far fewer district nuclei than a 1.5 km one
(`area_m2_per_nucleus: 30000` ⇒ ~33 nuclei at 1 km against ~75 at 1.5 km), and
nucleus 0 is forced into the core. With that few draws the district mix is
genuinely high-variance: MEASURED across eight seeds at 1 km, highrise+tower
came out anywhere from **17.1 % to 48.1 %** of block area. Seed choice is not a
detail on this plate — it is the single largest lever on whether a cell reads
as a balanced downtown or as a wall of towers.

So pick the seed deliberately, against the same numbers the review sheets and
the pytest gates use, rather than taking whatever the preset happens to carry.

WHAT IS SCORED
--------------
Each seed is built offline (CPU, no Kit, no GPU — `plan_png.build`) and scored
on five terms. Every term is a DISTANCE FROM A TARGET BAND, normalised, so no
term can dominate by unit choice and the total reads as "how far from ideal":

    tall_share      highrise+tower as a share of block area   target 0.20-0.35
    repeat_share    buildings in a same-model pair within R   target <= 0.15
    copies_model    buildings / distinct models               target <= 6.0
    top_share       the single most-used model's share        target <= 0.08
    empty_blocks    non-park blocks with no buildings         target 0

`empty_blocks` is a HARD gate rather than a weighted term — a city with an
empty block is not a better city for scoring well elsewhere, so any seed with
one is ranked last regardless. Same for a seed that fails to build at all.

The weights are deliberately blunt (`WEIGHTS`) and are not tuned: the point of
this tool is to make the trade visible and reproducible, not to compress five
numbers into one authoritative verdict. Read the table, not just the winner —
a seed two places down with a much better `tall_share` is often the one you
actually want, and the table is printed in full for exactly that reason.

USAGE
-----
    # sweep 24 seeds on the shared full-pool preset
    python3 scene_gen/tools/pick_city_seed.py \
        --config downtown_gac --region 1000 --seeds 0-23 \
        --out scene_gen/_plans/seed_pick_1km.json

    # a fire preset needs the layout-only override, same as plan_png
    python3 scene_gen/tools/pick_city_seed.py \
        --config downtown_fire_1500 --fire-layout --region 1000 --seeds 0-23
"""
import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_HERE, _SCENE_GEN):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import layout_review_png as review  # noqa: E402

#: (low, high) target bands. A value inside the band scores 0.
TARGETS = {
    "tall_share": (0.20, 0.35),
    "repeat_share": (0.0, 0.15),
    "copies_model": (0.0, 6.0),
    "top_share": (0.0, 0.08),
}

#: Normalisers — roughly "how much of this term is one whole unit of badness".
#: Chosen so a term at its normaliser contributes ~1.0 to the total.
SCALE = {
    "tall_share": 0.15,
    "repeat_share": 0.15,
    "copies_model": 4.0,
    "top_share": 0.06,
}

WEIGHTS = {
    "tall_share": 2.0,      # the complaint that started this
    "repeat_share": 2.0,    # the other complaint
    "copies_model": 1.0,
    "top_share": 1.0,
}


def _miss(value, band):
    lo, hi = band
    if value < lo:
        return lo - value
    if value > hi:
        return value - hi
    return 0.0


def score(metrics):
    """Weighted, normalised distance from the target bands. Lower is better."""
    total = 0.0
    parts = {}
    for key, band in TARGETS.items():
        miss = _miss(float(metrics[key]), band)
        term = WEIGHTS[key] * (miss / SCALE[key])
        parts[key] = round(term, 3)
        total += term
    return total, parts


def measure(config, seed, region, fire_layout, repeat_radius):
    cfg, layout, placements, res = review.build(
        config, seed=seed, region=region, fire_layout=fire_layout)
    houses = [p for p in placements if review._house(p)]
    dv = review.diversity_stats(houses, res, repeat_radius)
    blocks = review.block_typologies(layout)
    st = review.typology_stats(blocks)

    # Empty non-park blocks. A building counts for the block its CENTRE is in,
    # which is the same rule `city_layout_audit` and the review sheets use.
    occupied = set()
    for p in houses:
        for i, (rect, _name) in enumerate(blocks):
            if rect[0] <= p["x_m"] <= rect[2] and rect[1] <= p["y_m"] <= rect[3]:
                occupied.add(i)
                break
    empty = [i for i, (_r, name) in enumerate(blocks)
             if i not in occupied and name != "park"]

    return {
        "seed": seed,
        "blocks": len(blocks),
        "houses": len(houses),
        "models": dv["models"],
        "copies_model": dv["copies_per_model"],
        "top_share": dv["top_share"],
        "top_model": dv["top_model"][0],
        "repeat_pairs": dv["repeat_pairs"],
        "repeat_share": dv["repeat_share"],
        "tall_share": st["tall_share"],
        "empty_blocks": len(empty),
    }


def parse_seeds(text):
    out = []
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            out.extend(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="downtown_gac")
    ap.add_argument("--region", type=float, default=1000.0)
    ap.add_argument("--seeds", default="0-15",
                    help="comma list and/or a-b ranges, e.g. '0-23' or '2,4,7'")
    ap.add_argument("--fire-layout", action="store_true")
    ap.add_argument("--repeat-radius", type=float, default=60.0)
    ap.add_argument("--out", default=None, help="write the full table as JSON")
    ap.add_argument("--top", type=int, default=5,
                    help="how many winners to call out at the end")
    a = ap.parse_args()

    rows = []
    for seed in parse_seeds(a.seeds):
        try:
            m = measure(a.config, seed, a.region, a.fire_layout,
                        a.repeat_radius)
        except Exception as exc:                       # noqa: BLE001
            print("[seed %d] BUILD FAILED: %s" % (seed, exc), file=sys.stderr)
            rows.append({"seed": seed, "failed": str(exc)})
            continue
        m["score"], m["score_parts"] = score(m)
        rows.append(m)
        print("[seed %3d] tall %.1f%%  rep %.1f%%  models %d  cpm %.1f  "
              "top %.1f%%  empty %d  -> score %.2f"
              % (seed, 100 * m["tall_share"], 100 * m["repeat_share"],
                 m["models"], m["copies_model"], 100 * m["top_share"],
                 m["empty_blocks"], m["score"]), flush=True)

    ok = [r for r in rows if "failed" not in r]
    # Hard gate first, then the weighted score.
    ok.sort(key=lambda r: (r["empty_blocks"] > 0, r["score"]))

    print("\n%5s %6s %7s %7s %6s %7s %7s %6s %7s"
          % ("seed", "blocks", "houses", "models", "cpm", "tall%", "rep%",
             "empty", "score"))
    for r in ok:
        print("%5d %6d %7d %7d %6.1f %7.1f %7.1f %6d %7.2f"
              % (r["seed"], r["blocks"], r["houses"], r["models"],
                 r["copies_model"], 100 * r["tall_share"],
                 100 * r["repeat_share"], r["empty_blocks"], r["score"]))

    if ok:
        print("\nBEST %d:" % min(a.top, len(ok)))
        for r in ok[:a.top]:
            worst = max(r["score_parts"].items(), key=lambda kv: kv[1])
            print("  seed %-4d score %.2f   (worst term: %s %.2f)   "
                  "tall %.1f%%  rep %.1f%%  models %d"
                  % (r["seed"], r["score"], worst[0], worst[1],
                     100 * r["tall_share"], 100 * r["repeat_share"],
                     r["models"]))

    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)) or ".",
                    exist_ok=True)
        with open(a.out, "w") as fh:
            json.dump({"config": a.config, "region_m": a.region,
                       "repeat_radius_m": a.repeat_radius,
                       "targets": TARGETS, "weights": WEIGHTS,
                       "rows": rows}, fh, indent=1)
        print("\n[seed-pick] %s" % a.out)
    return ok


if __name__ == "__main__":
    main()
