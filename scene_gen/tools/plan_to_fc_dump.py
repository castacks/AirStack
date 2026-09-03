#!/usr/bin/env python3
"""plan_to_fc_dump.py — a `fire_city_placements_dump.v1` from an OFFLINE layout.

The dump that every downstream fire tool consumes (`fire_city_dry_run.
load_placements_dump`, `fire_city_union`, `fc_dump_crop`, `fire_people.
derive_layout`) is normally written by `urban_fire_city_launch_script.
dump_city_placements` — INSIDE Kit, on a pod, after a real build. That makes
the whole manifest chain unavailable to anyone without a GPU, which is
backwards: choosing which buildings burn is pure arithmetic over a layout and
needs no renderer at all.

This writes the same schema from `plan_png.build`, which runs the real
subdivider and the real `districts` pass on CPU with `measure_usds=False`. So
the chain

    layout  ->  dump  ->  fire_city_union --profile  ->  manifest  ->  review PNG

runs end to end on a laptop, and a pod is only spent once the manifest is
already known to be good.

WHAT IS AND IS NOT FAITHFUL
---------------------------
Faithful: block rects and their typologies, every house's model, position, yaw
and footprint, and the INDEX `i` each house holds in the full placement list.
`load_placements_dump` rebuilds `placements` at `n_placements_total` length and
`urban_fire_city_launch_script.resolve_cell` matches a manifest record back to
a live prim by that index, so it has to mean the same thing here as it does in
Kit — hence `i` is the enumerate index over the COMPLETE placement list
(props, cars and trees included), not over the houses alone.

Not faithful: `W`/`D`/`H` come from the asset-set comment scrape and the
measured `_plans/*.json`, not from opening the USD. They agree with a Kit
measurement to within a few cm for every asset that carries a measurement, and
`plan_png`'s `res.guessed` names any that do not — those are reported by
``--warn-guessed`` and are the only numbers here a pod would change.

`cell` is synthesised as ``/World/stage/generated/house_<model#>_<i>``, the
same shape `apply_placements` writes, so a record reads the same in both
worlds. It is not guaranteed to equal the prim name a Kit build would choose
for the same slot; anything matching on `cell` alone (rather than on `i` +
usd + distance, which is `resolve_cell`'s real route) must not rely on it.

USAGE
-----
    python3 scene_gen/tools/plan_to_fc_dump.py \
        --config downtown_fire_1500 --fire-layout --seed 4 --region 1000 \
        --out scene_gen/_plans/fc_dump_1km_s4.json
"""
import argparse
import collections
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_HERE, _SCENE_GEN):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import plan_png  # noqa: E402

SCHEMA = "fire_city_placements_dump.v1"


def _yaw_swapped(fp, yaw_deg):
    """`(W, D)` in WORLD axes for a footprint turned by *yaw_deg*.

    The dump's `W`/`D` are what `urban_fire_city.burnable` and
    `kit_substitute.route` size a building from, and `dump_city_placements`
    writes them post-rotation. Every downtown yaw is axis-aligned, so this is
    a swap on the odd quarter turns and identity otherwise; a yaw that is
    neither is left alone rather than approximated, which would quietly
    mis-size the one asset it applied to.
    """
    w, d = float(fp["sx"]), float(fp["sy"])
    y = float(yaw_deg) % 180.0
    if 45.0 <= y < 135.0:
        return d, w
    return w, d


def build_doc(cfg, layout, placements, res, preset, seed):
    typ_of = layout.get("_typology_of") or {}
    blocks = []
    for b in layout.get("blocks", []):
        rect = tuple(float(v) for v in b)
        name = typ_of.get(rect)
        if name is None:
            try:
                name = typ_of.get(b)
            except TypeError:
                name = None
        blocks.append({"rect": [round(v, 4) for v in rect], "name": name})

    model_no = {}
    out = []
    for i, p in enumerate(placements):
        if p.get("category") not in ("house", "building"):
            continue
        usd = str(p.get("usd", ""))
        base = os.path.basename(usd)
        n = model_no.setdefault(base, len(model_no))
        fp = res.get(usd, "house", scale=p.get("scale", 1.0),
                     axis_up=p.get("axis_up", "Z"))
        w, d = _yaw_swapped(fp, p.get("yaw_deg", 0.0))
        out.append({
            "i": i,
            "cell": "/World/stage/generated/house_%d_%d" % (n, i),
            "usd": usd,
            "x_m": round(float(p["x_m"]), 6),
            "y_m": round(float(p["y_m"]), 6),
            "z_m": round(float(p.get("z_m", 0.0)), 6),
            "yaw_deg": round(float(p.get("yaw_deg", 0.0)), 4),
            "scale": float(p.get("scale", cfg.get("asset_scale", 1.0)) or 1.0),
            "category": "house",
            "axis_up": str(p.get("axis_up", "Z")).upper(),
            "W": round(float(w), 6),
            "D": round(float(d), 6),
            "H": round(float(fp["sz"]), 6),
        })

    x0, y0, x1, y1 = layout["region"]
    return {
        "schema": SCHEMA,
        "dimensions_space": "world_xy",
        "preset": preset,
        "seed": int(seed),
        "region_m": [round(x1 - x0, 3), round(y1 - y0, 3)],
        "n_placements_total": len(placements),
        "placements": out,
        "typology": {"blocks": blocks},
        "_source": "plan_to_fc_dump.py (offline, measure_usds=False)",
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="downtown_fire_1500")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--region", type=float, default=None)
    ap.add_argument("--fire-layout", action="store_true")
    ap.add_argument("--out", required=True)
    ap.add_argument("--warn-guessed", action="store_true",
                    help="list every asset whose footprint was NOT measured")
    a = ap.parse_args()

    overrides = {}
    if a.fire_layout:
        overrides["disaster-type"] = "none"
    if a.region:
        overrides["region_m"] = [float(a.region), float(a.region)]
    cfg, layout, placements, res = plan_png.build(
        a.config, seed=a.seed, spec_overrides=overrides or None)

    seed = a.seed if a.seed is not None else cfg.get("seed", 0)
    doc = build_doc(cfg, layout, placements, res, a.config, seed)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)) or ".", exist_ok=True)
    with open(a.out, "w") as fh:
        json.dump(doc, fh, indent=1)
    print("[fc-dump] %s — %d houses of %d placements, %d blocks, %.0f m"
          % (a.out, len(doc["placements"]), doc["n_placements_total"],
             len(doc["typology"]["blocks"]), doc["region_m"][0]))
    if a.warn_guessed:
        guessed = sorted(getattr(res, "guessed", []) or [])
        houses = {os.path.basename(p["usd"]) for p in doc["placements"]}
        hit = [g for g in guessed if os.path.basename(g) in houses]
        print("[fc-dump] unmeasured assets in play: %d (%s)"
              % (len(hit), ", ".join(os.path.basename(g) for g in hit[:8])
                 or "none"))
    return doc


if __name__ == "__main__":
    main()
