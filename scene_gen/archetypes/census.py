"""census — which asset types a scene ACTUALLY places, written down.

The archetype library is priced per (type, level), so the first question any
bake has to answer is "which types does the scene I care about place?". The
answer is not a property of the asset pack — a 1 km urban_v3 layout draws ~40
of the pack's 71 intact buildings, and which 40 depends on the seed, the
region, and the measured footprints.

WHY THIS IS NOT `plan.used_by_scene`
------------------------------------
`used_by_scene` answers the same question by running the layout in-process,
and it carries a warning that matters here: packing keys off MEASURED
footprints, and a plain `python3` cannot open a Nucleus asset, so it falls
back to `fallback_sizes`, packs differently, and names different buildings.
Measured on `urban_quake_tiny`: 5 structure types on the host against 2 under
Kit.

So the authoritative census is the one taken from a REAL scene build, under
Kit, with Nucleus reachable — which is exactly what the launcher already does
before it photographs anything. `record()` turns that run's placement list
into a file; `read()` hands it back to the baker on the host, where it is now
a measurement rather than an estimate.

WHAT IT IS FOR
--------------
Two things, both in the library:

1. **Bake order.** `bake_cli --census <file>` bakes the used types first, so a
   bake that is stopped early has finished the assets a scene needs.
2. **A durable `used_by` mark on the manifest**, so a library that outlives
   this session still says which of its archetypes the urban_v3 layout
   depends on and which are speculative stock.
"""

from __future__ import annotations

import datetime
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from archetypes import library as lib                          # noqa: E402

#: Placement categories that hold a structure. `house` is what every building
#: pool lands in, whatever typology it was drawn from.
STRUCTURE_CATEGORIES = ("house",)

#: ...and vegetation. `street_tree` is the kerbside pool, `tree` the rest.
VEGETATION_CATEGORIES = ("tree", "street_tree", "park_tree")


def _pack_index(config: dict) -> dict:
    """slug -> what the PACK says about that asset.

    The placement only carries a resolved USD path. Everything else worth
    recording — which pool it came from, what it is made of, whether it is
    authored ruin art — is in the pack, and is looked up by slug because the
    placement's path has been joined to `asset_root` while the pack's has not.
    """
    import scene_generator as sg

    out = {}
    bld = sg._building_section(config) or {}
    for pool, entries in (bld.items() if isinstance(bld, dict) else ()):
        for e in sg._flatten_pool(entries):
            usd = e.get("usd") if isinstance(e, dict) else e
            if not usd:
                continue
            tags = list(e.get("tags") or ()) if isinstance(e, dict) else []
            out.setdefault(lib.type_slug(usd), {
                "usd": usd,
                "pool": pool,
                "material": (e.get("material") if isinstance(e, dict) else "")
                            or "",
                "tags": tags,
                "kind": "structure",
            })
    for pool in ("trees", "street_trees", "park_trees"):
        for e in sg._flatten_pool((config.get("usds") or {}).get(pool)):
            usd = e.get("usd") if isinstance(e, dict) else e
            if not usd:
                continue
            out.setdefault(lib.type_slug(usd), {
                "usd": usd, "pool": pool, "material": "", "tags": [],
                "kind": "vegetation",
            })
    return out


def _twin_distances(pts: list) -> list:
    """For each point, the distance to the NEAREST OTHER point in the list.

    This is what "is the same building standing within sight" means as a
    number. The city-wide histogram cannot answer it — a perfectly balanced
    count still reads as copy-paste when the two copies face each other across
    one street — so a census that only counted types would miss exactly the
    defect a viewer notices first.

    O(n^2), and deliberately: the largest count in a 1 km urban layout is ~25,
    so a spatial index would cost more to read than it saves.
    """
    out = []
    for i, (x, y) in enumerate(pts):
        best = None
        for j, (u, v) in enumerate(pts):
            if i == j:
                continue
            d = math.hypot(x - u, y - v)
            if best is None or d < best:
                best = d
        if best is not None:
            out.append(best)
    return out


def _diversity(config: dict, counts: dict, index: dict) -> dict:
    """How much of the library this layout reached, and how close the twins are.

    `near_50m` / `near_100m` are the headline: the share of BUILDINGS standing
    within that distance of another copy of themselves. A drone flying a
    corridor sees a few hundred metres of it, so those are the two bands where
    a repeat reads as a repeat rather than as a city having a style.
    """
    struct = [r for r in counts.values() if r.get("kind") == "structure"]
    n_placed = sum(r["count"] for r in struct)
    available = sum(1 for m in index.values() if m.get("kind") == "structure"
                    and "damaged" not in m.get("tags", ())
                    and "destroyed" not in m.get("tags", ()))
    near50 = sum(1 for r in struct for d in r.get("_twins", ()) if d < 50.0)
    near100 = sum(1 for r in struct for d in r.get("_twins", ()) if d < 100.0)
    top = max((r["count"] for r in struct), default=0)
    return {
        "types_used": len(struct),
        "types_available": available,
        "buildings": n_placed,
        "top_model_share": round(top / n_placed, 4) if n_placed else 0.0,
        # Share of buildings with a copy of themselves this close.
        "near_50m": round(near50 / n_placed, 4) if n_placed else 0.0,
        "near_100m": round(near100 / n_placed, 4) if n_placed else 0.0,
        "closest_twin_m": round(min((d for r in struct
                                     for d in r.get("_twins", ())),
                                    default=0.0), 1),
        # The knobs this was measured under, so two censuses can be compared
        # without going back to the config that produced them.
        # UNDER `layout`, not `detail`: districts decides WHERE buildings go,
        # which is a layout decision even though the module lives in `detail/`.
        "knobs": {k: ((config.get("layout") or {}).get("districts") or {})
                     .get(k)
                  for k in ("repeat_penalty", "repeat_radius_m",
                            "repeat_local_penalty", "repeat_local_falloff",
                            "pack_area_band")},
    }


def record(config: dict, placements: list, config_name: str = "") -> dict:
    """The census document for one built scene.

    *placements* is the list `build_scene` / `generate_scene_on_stage`
    returned. Counts are per TYPE, because that is the unit Stage A bakes —
    twelve copies of one brownstone are one archetype, not twelve.
    """
    index = _pack_index(config)
    layout = config.get("layout") or {}
    counts, unknown = {}, {}

    for p in placements:
        cat = str(p.get("category") or "")
        if cat in STRUCTURE_CATEGORIES:
            kind = "structure"
        elif cat in VEGETATION_CATEGORIES:
            kind = "vegetation"
        else:
            continue
        usd = str(p.get("usd") or "")
        if not usd:
            continue
        slug = lib.type_slug(usd)
        rec = counts.get(slug)
        if rec is None:
            meta = index.get(slug)
            if meta is None:
                # A placement whose asset is not in any pool this module knows
                # how to read. Recorded rather than dropped: a census that
                # silently omits assets is worse than one that admits it.
                unknown[slug] = usd
                meta = {"usd": usd, "pool": "", "material": "", "tags": [],
                        "kind": kind}
            rec = counts[slug] = {"type": slug, "count": 0, "_pts": [], **meta}
        rec["count"] += 1
        try:
            rec["_pts"].append((float(p.get("x_m")), float(p.get("y_m"))))
        except (TypeError, ValueError):
            pass

    # WHERE the copies are, not just how many. See `_twin_distances`.
    for rec in counts.values():
        twins = _twin_distances(rec.pop("_pts", []))
        rec["_twins"] = twins
        if twins:
            rec["nearest_twin_m"] = round(min(twins), 1)
            rec["median_twin_m"] = round(sorted(twins)[len(twins) // 2], 1)
    diversity = _diversity(config, counts, index)
    for rec in counts.values():
        rec.pop("_twins", None)

    ordered = sorted(counts.values(), key=lambda r: (-r["count"], r["type"]))
    return {
        "diversity": diversity,
        "version": 1,
        "config": config_name or str(config.get("_name") or ""),
        "asset_pack": str(config.get("asset_pack") or ""),
        "seed": config.get("seed"),
        "region_m": list(layout.get("region_m") or ()),
        "recorded_at": datetime.datetime.now().astimezone().isoformat(
            timespec="seconds"),
        "placements": len(placements),
        "unknown": unknown,
        "assets": ordered,
    }


def write(path: str, doc: dict) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as fh:
        json.dump(doc, fh, indent=2, sort_keys=True)
    return path


def read(path: str) -> dict:
    if not path or not os.path.isfile(path):
        return {}
    with open(path) as fh:
        return json.load(fh) or {}


def used_types(doc: dict, kind: str = "") -> list:
    """The type slugs the census saw, most-placed first.

    Most-placed first IS the bake order: the asset standing on forty corners
    is the one whose missing archetype is most visible, so it should be baked
    before the one that appears twice.
    """
    return [a["type"] for a in (doc or {}).get("assets", ())
            if not kind or a.get("kind") == kind]


def summarise(doc: dict) -> str:
    assets = (doc or {}).get("assets") or []
    st = [a for a in assets if a.get("kind") == "structure"]
    vg = [a for a in assets if a.get("kind") == "vegetation"]
    reg = "x".join(str(int(v)) for v in (doc.get("region_m") or ())) or "?"
    lines = [
        f"census — {doc.get('config', '?')} @ seed {doc.get('seed')} "
        f"({reg} m, pack {doc.get('asset_pack', '?')})",
        f"  {len(st)} building types over "
        f"{sum(a['count'] for a in st)} buildings",
        f"  {len(vg)} vegetation types over "
        f"{sum(a['count'] for a in vg)} plants",
    ]
    d = (doc or {}).get("diversity") or {}
    if d:
        lines.append(
            f"  diversity: {d.get('types_used')}/{d.get('types_available')} "
            f"models, top {100 * d.get('top_model_share', 0):.1f}%, "
            f"twin within 50 m {100 * d.get('near_50m', 0):.0f}% / "
            f"100 m {100 * d.get('near_100m', 0):.0f}%, "
            f"closest {d.get('closest_twin_m')} m")
    if doc.get("unknown"):
        lines.append(f"  {len(doc['unknown'])} type(s) not found in any pool")
    return "\n".join(lines)


def main():
    import argparse

    ap = argparse.ArgumentParser(
        description="Read a census written by a scene run under Kit.")
    ap.add_argument("path")
    ap.add_argument("--list", action="store_true",
                    help="every type and its count, bake order first")
    ap.add_argument("--kind", default="", choices=["", "structure",
                                                   "vegetation"])
    args = ap.parse_args()

    doc = read(args.path)
    if not doc:
        raise SystemExit(f"no census at {args.path}")
    print(summarise(doc))
    if args.list:
        for a in doc.get("assets", ()):
            if args.kind and a.get("kind") != args.kind:
                continue
            print(f"  {a['count']:4d}  {a['type']:<40} "
                  f"{a.get('pool', ''):<10} {a.get('material', '')}")


if __name__ == "__main__":
    main()
