#!/usr/bin/env python3
"""match_assets.py — find library assets that could stand in for a layout feature.

    python3 scene_gen/tools/match_assets.py \
        --index scene_gen/_plans/sei_coa_tagged.tsv \
        --plan disaster_city --top 6
    python3 scene_gen/tools/match_assets.py --index ... \
        --tags building --size 21x17 --top 10          # one ad-hoc query

Every query returns a RANKED LIST, never a single answer. Nothing here can see
the assets — it is matching a name-derived tag set against a measured bounding
box — so the last step is always a person (or a render pass) looking at the top
few. `--json` writes the candidates out for exactly that.

THE SCORE, and what each term is defending against:

  tags        required tags must all be present, excluded ones must be absent.
              This is a filter, not a score: an asset tagged `material` is not
              a worse building, it is not a building.
  fit         how well the asset's footprint fits the target, ALLOWING A
              QUARTER TURN — a 21 x 17 m target and a 17 x 21 m asset are the
              same asset rotated, and scoring them differently would rank the
              right model below the wrong one. Ratio-based, not absolute, so a
              2 m error on a 40 m warehouse does not outweigh a 2 m error on a
              3 m kiosk.
  height      same, when the target says one. Weighted lower than plan: a
              building is chosen for its footprint and scaled or swapped for
              storeys, and half the library has no measured height anyway.
  penalties   `payload` (a sub-file, not placeable on its own), `instanced`
              (a reference wrapper), `stage` (a whole scene) and missing
              extents each push a candidate down rather than out — they are
              usually wrong and occasionally the only thing available.
"""

import argparse
import json
import math
import os
import sys

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

PENALTY = {"payload": 0.45, "instanced": 0.15, "stage": 0.6, "material": 0.9}


def load_assets(path):
    out = []
    with open(path) as f:
        next(f)
        for line in f:
            c = line.rstrip("\n").split("\t")
            if len(c) < 11:
                continue
            tags = set(filter(None, c[5].split(";")))
            try:
                dims = (float(c[7]), float(c[8]), float(c[9])) if c[7] else None
            except ValueError:
                dims = None
            out.append({"path": c[0], "name": c[1], "ext": c[2],
                        "size_bytes": int(c[3]), "pack": c[4], "tags": tags,
                        "dims": dims})
    return out


def fit_score(dims, want_lw, want_h):
    """1.0 for a perfect fit, falling off with the log-ratio of each side.

    The asset is allowed a quarter turn — `(l, w)` and `(w, l)` are the same
    model — and the better of the two orientations is taken.
    """
    if not dims:
        return 0.0
    al, aw = max(dims[0], dims[1]), min(dims[0], dims[1])
    if al <= 1e-6 or aw <= 1e-6:
        return 0.0
    best = 0.0
    for tl, tw in ((max(want_lw), min(want_lw)),):
        if tl <= 1e-6 or tw <= 1e-6:
            continue
        e = abs(math.log(al / tl)) + abs(math.log(aw / tw))
        best = max(best, math.exp(-e))
    if want_h and dims[2] > 1e-6:
        eh = abs(math.log(dims[2] / want_h))
        best = 0.75 * best + 0.25 * math.exp(-eh)
    return best


def query(assets, need=(), avoid=(), size=None, height=None, top=8,
          pack=None):
    need, avoid = set(need), set(avoid)
    out = []
    for a in assets:
        if need and not need <= a["tags"]:
            continue
        if avoid & a["tags"]:
            continue
        if pack and a["pack"] != pack:
            continue
        s = fit_score(a["dims"], size, height) if size else (0.5 if a["dims"] else 0.2)
        for t, p in PENALTY.items():
            if t in a["tags"]:
                s *= (1.0 - p)
        if not a["dims"]:
            s *= 0.35
        out.append((s, a))
    out.sort(key=lambda r: -r[0])
    return out[:top]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--index", required=True)
    ap.add_argument("--plan", default="")
    ap.add_argument("--tags", default="")
    ap.add_argument("--avoid", default="payload,material,stage")
    ap.add_argument("--size", default="", help="LxW in metres, e.g. 21x17")
    ap.add_argument("--height", type=float, default=0.0)
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--json", default="")
    args = ap.parse_args()

    assets = load_assets(args.index)
    print(f"[match] {len(assets):,} assets in the library")
    avoid = [t for t in args.avoid.split(",") if t]

    queries = []
    if args.plan:
        from layout import site_plan as spl
        spec = spl.load_spec(args.plan)
        # KIND -> the tags a stand-in for it must carry. Deliberately loose:
        # the point is to surface candidates, and a query that returns nothing
        # teaches less than one that returns six imperfect things.
        # AND WHAT EACH MUST NOT BE. Without this the top match for three
        # different buildings was `Shumard_Oak.usd`, for a mast it was
        # `Fountain_Grass_Tall`, and for a pipe rig a boulder — every one of
        # them the right SIZE and carrying the required tag, because the tag
        # vocabulary picks up a `building` from a directory name and the fit
        # score cannot tell a 20 m tree from a 20 m house.
        KIND_AVOID = {
            "building": ["tree", "plant", "grass", "rock", "terrain", "debris",
                         "rubble_pile", "vehicle", "person", "road", "sidewalk"],
            "tower": ["tree", "plant", "grass", "rock", "terrain", "vehicle"],
            "vehicle": ["building", "tree", "plant", "grass", "terrain"],
            "tank": ["tree", "plant", "grass", "building", "terrain", "rock"],
            "vessel": ["tree", "plant", "grass", "building", "terrain", "rock"],
            "pipe_rig": ["tree", "plant", "grass", "building", "terrain", "rock"],
            "container": ["tree", "plant", "grass", "terrain", "rock"],
            "mast": ["tree", "plant", "grass", "building", "terrain", "rock"],
            "rubble": ["building", "tree", "plant", "vehicle", "person"],
            "wreck": ["building", "tree", "plant", "person"],
        }
        KIND_TAGS = {"building": ["building"], "tower": ["building"],
                     "vehicle": ["vehicle"], "rubble": ["rubble_pile"],
                     "debris": ["debris"], "container": ["prop"],
                     "tank": ["prop"], "vessel": ["prop"], "pipe_rig": ["prop"],
                     "mast": ["streetside"], "wreck": ["debris"],
                     "tree": ["tree"], "fence": ["fence"], "sign": ["sign"],
                     "pit": ["terrain"]}
        for i, f in enumerate(spec.get("features") or []):
            k = f.get("kind")
            sz = f.get("size_m")
            if not sz:
                poly = f.get("poly") or []
                if poly:
                    xs = [p[0] for p in poly]
                    ys = [p[1] for p in poly]
                    sz = [max(xs) - min(xs), max(ys) - min(ys)]
            queries.append({"id": f"{k}_{i:02d}", "kind": k,
                            "need": KIND_TAGS.get(k, []),
                            "avoid": KIND_AVOID.get(k, []),
                            "size": sz, "height": f.get("height_m", 0.0),
                            "note": f.get("note", "")})
    else:
        queries.append({"id": "adhoc", "kind": "?",
                        "need": [t for t in args.tags.split(",") if t],
                        "size": [float(v) for v in args.size.split("x")]
                                 if args.size else None,
                        "height": args.height, "note": ""})

    dump = {}
    for q in queries:
        res = query(assets, q["need"], list(avoid) + list(q.get("avoid", [])),
                    q["size"], q["height"], args.top)
        print(f"\n=== {q['id']}  need={q['need']}  "
              f"size={q['size']}  h={q['height']}\n    {q['note'][:78]}")
        dump[q["id"]] = []
        for s, a in res:
            d = a["dims"]
            dim = f"{d[0]:.1f}x{d[1]:.1f}x{d[2]:.1f}" if d else "unmeasured"
            print(f"   {s:5.3f}  {dim:>20}  {a['pack']}/{a['name'][:44]}")
            dump[q["id"]].append({"score": round(s, 4), "path": a["path"],
                                  "name": a["name"], "pack": a["pack"],
                                  "dims": d, "tags": sorted(a["tags"])})
    if args.json:
        with open(args.json, "w") as f:
            json.dump(dump, f, indent=1)
        print(f"\n[match] candidates -> {args.json}")


if __name__ == "__main__":
    main()
