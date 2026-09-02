#!/usr/bin/env python3
"""fire_corridor_manifest.py — a fire manifest from a WIND-DRIVEN CORRIDOR.

A drop-in alternative to `fire_city_union.py`'s multi-ignition selection. It
emits the SAME manifest shape (`records` carrying `usd`/`x`/`y`/`x_orig`/
`y_orig`/`kind`/`asset`/`style`/`typology`/`W`/`D`/`H`/`cell`/`i`/`level`/
`age_s`/`t_ignite_s`/...), so `fire_city_bake.sh`, the assembly launcher and
`fire_people` consume it unchanged.

What differs is WHICH buildings burn and WHY — see `disaster/fire_corridor.py`
for the model. In one line: one fire, one front, everything inside the swathe
burns, and the level is the compartment-fire clock rather than a post-hoc
count shuffle. That is what makes the burn contiguous, and it is why this
tool does NOT filter the corridor down to what currently has a bake — the
whole point is that the bake list is a consequence of the corridor.

    python3 scene_gen/tools/fire_corridor_manifest.py \\
        --dump scene_gen/_plans/fc_dump_1km_l1.json \\
        --target-frac 0.10 --epoch-s 6336 \\
        --out scene_gen/_plans/fire_corr_l1.json \\
        --bake-list scene_gen/_plans/bake_l1.json

`--bake-list` writes the lazy-bake worklist for the pod: every distinct asset
inside the corridor that does not already route to a bake, with its instance
count and bake kind, so the pod job is exactly scoped to this cell.
"""
import argparse
import collections
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_HERE, _SCENE_GEN):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fire_city_dry_run as fdr                      # noqa: E402
from disaster import fire_corridor as fc             # noqa: E402
from disaster import urban_fire_city as ufc          # noqa: E402


def _size_of(placements):
    by_usd = {}
    for p in placements:
        u = p.get("usd")
        if u and p.get("W") is not None:
            by_usd[u] = (float(p["W"]), float(p["D"]), float(p["H"]))
    return lambda u: by_usd.get(u)


def _n_storeys(H):
    return max(1, int(round(float(H) / 3.2)))


def build(dump_path, *, target_frac, epoch_s, heading_deg=None,
          half_width_m=fc.DEFAULT_HALF_WIDTH_M,
          spread_deg=fc.DEFAULT_SPREAD_DEG, n_collapse=0, n_f6=0, seed=0):
    config, layout, placements, dump_seed, preset, sha = \
        fdr.load_placements_dump(dump_path)
    # THE FIRE KEYS ARE NOT ON THE COMPILED CONFIG. `compile_spec` drops the
    # raw top-level `heading_deg`/`duration_s`/`start_offset_frac` -- they are
    # the preset's own spec, not compiled disaster parameters -- so reading
    # them off `config` silently yields the DEFAULT. That is a quiet failure:
    # every level ran at 45 deg while the presets said 70 / 160 / 285, and it
    # only showed up because all three review sheets came out with the same
    # diagonal. `fire_city_dry_run._raw_fire_spec` re-reads the YAML, which is
    # the same route the shipped pipeline already takes.
    raw = fdr._raw_fire_spec(preset)
    if heading_deg is None:
        heading_deg = float(raw["heading_deg"])
    # The plate size comes off the LAYOUT (`(x0, y0, x1, y1)`), which the dump
    # always carries; `config` has neither `region_m` nor `region` after
    # compilation.
    reg = layout.get("region") or (-500.0, -500.0, 500.0, 500.0)
    region = [abs(float(reg[2]) - float(reg[0])),
              abs(float(reg[3]) - float(reg[1]))]

    houses_idx = [i for i, p in enumerate(placements)
                  if p.get("category") == "house"]
    size_of = _size_of(placements)

    # Classify every house once: burnable now / bakeable / permanently not.
    base = {}
    permanent_bad = set()
    for i in houses_idx:
        rec = dict(placements[i])
        rec.setdefault("category", "house")
        rec.setdefault("prim_path", rec.get("cell"))
        ok, val = ufc.burnable(layout, rec, size_of)
        if ok:
            base[i] = val
        else:
            if fc.unbakeable(val):
                permanent_bad.add(i)
            base[i] = None

    target_n = int(round(float(target_frac) * len(houses_idx)))
    # WIDEN WHEN THE PLATE RUNS OUT OF LENGTH. A corridor prefers to grow
    # along the wind -- that is what makes a bigger level read as "the fire
    # ran further" -- but the run available depends on the heading: a
    # diagonal crosses a 1 km plate in 1414 m, an axis-aligned one in 1000 m.
    # So an off-diagonal heading at a high `target_frac` simply cannot reach
    # the count by lengthening, and the corridor has to fan out instead.
    # Without this the tool refused any heading that was not the 45 deg
    # diagonal as soon as coverage passed ~25 %.
    origin = None
    width = float(half_width_m)
    for _attempt in range(6):
        origin, length_m, members, n_bad = fc.place_corridor(
            placements, houses_idx, region, heading_deg=heading_deg,
            target_n=target_n, half_width_m=width, spread_deg=spread_deg,
            permanent_bad=permanent_bad)
        if origin is not None:
            break
        width *= 1.30
    if origin is None:
        raise SystemExit(
            "no corridor holds %d houses on a %s m plate at %.0f deg even at "
            "half-width %.0f m -- lower --target-frac"
            % (target_n, region, heading_deg, width))
    half_width_m = width

    levels, v = fc.assign_levels(members, epoch_s=epoch_s, length_m=length_m)
    levels = fc.apply_collapse(levels, members, n_collapse=n_collapse,
                               n_f6=n_f6, seed=seed)

    records = []
    skipped_permanent = []
    for i, p, s, _c in members:
        if i not in levels:
            continue
        if i in permanent_bad:
            skipped_permanent.append(i)
            continue
        lvl, t_ig, age = levels[i]
        rec = base.get(i)
        needs_bake = rec is None
        if rec is None:
            # In the corridor and bakeable, but no bake yet. It still belongs
            # in the manifest -- that is what the bake list is for -- so a
            # minimal record is synthesised from the placement.
            W, D, H = size_of(p.get("usd")) or (None, None, None)
            rec = {"usd": p.get("usd"), "x": float(p["x_m"]),
                   "y": float(p["y_m"]),
                   "x_orig": float(p.get("x_m_orig", p["x_m"])),
                   "y_orig": float(p.get("y_m_orig", p["y_m"])),
                   "yaw_deg": float(p.get("yaw_deg", 0.0)),
                   "z": float(p.get("z_m", 0.0)),
                   "kind": None, "asset": None, "style": None,
                   "typology": ufc.typology_at(layout, p["x_m"], p["y_m"]),
                   "W": W, "D": D, "H": H, "cell": p.get("cell")}
        out = dict(rec)
        out.update({
            "i": i, "level": lvl,
            "t_ignite_s": round(float(t_ig), 1),
            "age_s": round(float(age), 1),
            "how": "corridor", "via": None, "origin": 0, "seed": int(seed),
            "origin_frac": round(s / length_m, 4) if length_m else 0.0,
            "n_storeys": _n_storeys(out.get("H") or 10.0),
            "needs_bake": needs_bake,
        })
        records.append(out)

    records.sort(key=lambda r: -r["age_s"])
    doc = {
        "schema": "fire_city_manifest.corridor.v1",
        "preset": preset, "seed": int(dump_seed),
        "n": len(records), "n_achieved": len(records),
        "epoch_s": float(epoch_s),
        "corridor": {
            "heading_deg": float(heading_deg),
            "origin": [round(origin[0], 3), round(origin[1], 3)],
            "length_m": round(length_m, 2),
            "half_width_m": float(half_width_m),
            "spread_deg": float(spread_deg),
            "front_speed_mps": round(v, 4),
            "target_frac": float(target_frac),
            "n_houses": len(houses_idx),
            "n_permanently_unbakeable_inside": len(skipped_permanent),
        },
        "records": records,
        "placements_dump": {"path": os.path.abspath(dump_path), "sha256": sha},
        "note": ("wind-driven corridor, %.0f m long x %.0f m half-width at "
                 "%.0f deg; front %.3f m/s; %d of %d houses (%.1f%%); "
                 "%d permanently unbakeable buildings left intact inside"
                 % (length_m, half_width_m, heading_deg, v, len(records),
                    len(houses_idx), 100.0 * len(records) / len(houses_idx),
                    len(skipped_permanent))),
    }
    return doc, layout, placements


def bake_list(doc):
    """The lazy-bake worklist: what must be baked for THIS cell."""
    need = collections.Counter()
    kinds = {}
    for r in doc["records"]:
        if not r.get("needs_bake"):
            continue
        b = os.path.basename(str(r.get("usd") or ""))
        need[b] += 1
        kinds[b] = r.get("kind")
    return {
        "schema": "fire_bake_worklist.v1",
        "preset": doc.get("preset"), "seed": doc.get("seed"),
        "n_assets": len(need),
        "n_instances": sum(need.values()),
        "assets": [{"usd": k, "instances": v, "kind": kinds.get(k)}
                   for k, v in need.most_common()],
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dump", required=True)
    ap.add_argument("--target-frac", type=float, default=0.20)
    ap.add_argument("--epoch-s", type=float, required=True)
    ap.add_argument("--heading-deg", type=float, default=None)
    ap.add_argument("--half-width-m", type=float,
                    default=fc.DEFAULT_HALF_WIDTH_M)
    ap.add_argument("--spread-deg", type=float, default=fc.DEFAULT_SPREAD_DEG)
    ap.add_argument("--collapse", type=int, default=0)
    ap.add_argument("--f6", type=int, default=0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--bake-list", default=None)
    a = ap.parse_args()

    doc, _layout, _pl = build(
        a.dump, target_frac=a.target_frac, epoch_s=a.epoch_s,
        heading_deg=a.heading_deg, half_width_m=a.half_width_m,
        spread_deg=a.spread_deg, n_collapse=a.collapse, n_f6=a.f6,
        seed=a.seed)

    os.makedirs(os.path.dirname(os.path.abspath(a.out)) or ".", exist_ok=True)
    with open(a.out, "w") as fh:
        json.dump(doc, fh, indent=1)
    hist = collections.Counter(r["level"] for r in doc["records"])
    c = doc["corridor"]
    print("[corridor] %s" % a.out)
    print("  %d of %d houses (%.1f%%)  |  %.0f m x %.0f m half-width @ %.0f deg"
          % (len(doc["records"]), c["n_houses"],
             100.0 * len(doc["records"]) / c["n_houses"],
             c["length_m"], c["half_width_m"], c["heading_deg"]))
    print("  levels %s" % dict(sorted(hist.items())))
    print("  front %.3f m/s, %d permanently unbakeable left intact inside"
          % (c["front_speed_mps"], c["n_permanently_unbakeable_inside"]))

    if a.bake_list:
        wl = bake_list(doc)
        with open(a.bake_list, "w") as fh:
            json.dump(wl, fh, indent=1)
        print("[bake-list] %s — %d assets, %d instances need a bake"
              % (a.bake_list, wl["n_assets"], wl["n_instances"]))
    return doc


if __name__ == "__main__":
    main()
