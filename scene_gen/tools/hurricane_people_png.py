#!/usr/bin/env python3
"""hurricane_people_png.py -- the offline 2D verification gate for the
HURRICANE PEOPLE PASS (`disaster/hurricane_people.py`), separate from
`hurricane_layout_png.py`'s own material-prediction tool (that file's core is
untouched; this is a fresh, narrower script per this stream's own brief).

WHAT THIS REPLAYS. The same house-level assignment loop
`suburb_hurricane_launch_script.py` and `hurricane_layout_png.py` both run
(`hurricane.draw_vulnerability` -> `surge.house_water_state` -> (if wet)
`washaway.house_surge_state` -> `surge`'s own `swept` override, else
`hurricane.tornado_level_for_intensity`), on a REAL in-memory stage built by
`suburb_scene.generate_suburb_on_stage` -- so the house count, level tally and
positions match what a real Isaac build at this seed would place. What it does
NOT replay: cars, fences, land debris, rafts, ponds -- none of it is read by
the people pass, and skipping it keeps this tool an order of magnitude faster
than the full material-prediction one.

THE PEOPLE PASS ITSELF is `disaster.hurricane_people.plan_people`, called
exactly as the launcher's own "7c) THE PEOPLE" block calls it, against a
STUB resolver/asset-pool pair (a fixed 1.80 m / 0.35 m-deep character, no
Nucleus lookup at all) -- this is a 2D dry run of POSITIONS and CLASSES, not
a material or rig-fidelity check, and the stub keeps it runnable with no
network access whatsoever, per this task's "no Isaac runs" rule.

Usage:
    python3 scene_gen/tools/hurricane_people_png.py \\
        [--preset suburb_hurricane_500_l3,suburb_hurricane_500_l2] \\
        [--seed 11] [--out-dir ~/hurricane_previews/offline/people]
"""
import argparse
import collections
import math
import os
import random
import sys
import time

os.environ.setdefault("PXR_USDC_EMIT_DEPRECATION_WARNINGS", "0")

import numpy as np                                             # noqa: E402
from PIL import Image, ImageDraw                                # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_SCENE_GEN, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from pxr import Usd                                             # noqa: E402
import compile_disaster as cd                                   # noqa: E402
import suburb_scene as ss                                       # noqa: E402
from disaster import hurricane as hu                            # noqa: E402
from disaster import surge as sgw                                # noqa: E402
from disaster import washaway as wash                            # noqa: E402
from disaster import hurricane_people as hp                      # noqa: E402
from detail import modular_house as mh                           # noqa: E402

PARENT = "/World/gen"

HUMANS = ["rp_carla_rigged_001_ue4.usd", "rp_eric_rigged_001_ue4.usd",
         "rp_sophia_rigged_002_ue4.usd", "rp_nathan_rigged_003_ue4.usd",
         "rp_manuel_rigged_001_ue4.usd", "rp_dennis_rigged_001_ue4.usd"]


class _Pools:
    def scale_of(self, usd): return 0.01
    def axis_of(self, usd): return "z"
    def yaw_of(self, usd): return 90.0
    def roll_of(self, usd): return 0.0


class _Resolver:
    """A 1.80 m character, 0.35 m deep -- no Nucleus lookup at all. This
    tool checks POSITIONS and CLASSES, not rig fidelity; see the module
    docstring."""
    def get(self, usd, cat, **kw):
        return {"sx": 1.25, "sy": 0.35, "sz": 1.80, "base": 0.0,
                "cx": 0.0, "cy": 0.0, "cz": 0.0}


LEVEL_COLOR = {
    "pristine": (150, 190, 130), "roof_stripped": (200, 190, 90),
    "roof_collapsed": (200, 140, 70), "partial_collapse": (190, 100, 60),
    "leveled": (150, 60, 50), "swept": (110, 40, 90),
}


def _build_houses(config, seed, houses_ss, inten, scfg, depth):
    """The launcher's own house-level loop, verbatim order/RNG, minus the
    stage-authoring side effects this tool has no use for."""
    fp_by_style = {e["style"]: max(e["w"], e["d"])
                  for e in ss.modular_catalogue(config)}
    drng = random.Random(seed + 5)
    house_recs, wrecks = [], []
    htally = collections.Counter()
    n_swept = 0
    for i, h in enumerate(houses_ss):
        it = float(inten(h["x"], h["y"]))
        era, vuln = hu.draw_vulnerability(drng)
        wst = sgw.house_water_state(scfg, h["x"], h["y"], drng)
        _sl = None
        if float(wst.get("depth", 0.0)) > 0.0:
            try:
                _sl = wash.house_surge_state(float(wst["depth"]), vuln, drng)
            except Exception:
                pass
        if wst.get("swept") or _sl == "swept":
            level = "swept"
            n_swept += 1
        else:
            level = hu.tornado_level_for_intensity(it, drng, vuln=vuln)
        htally[level] += 1
        rec = {"prim_path": "/World/gen/inst/h_%d" % i, "style": h["style"],
              "level": level, "x": float(h["x"]), "y": float(h["y"]),
              "yaw_deg": float(h["yaw"]), "intensity": it,
              "water_depth_m": float(wst.get("depth", 0.0))}
        house_recs.append(rec)
        if level != "pristine":
            wrecks.append((h["x"], h["y"], fp_by_style.get(h["style"], 12.0),
                          it, level, None))
    return house_recs, wrecks, fp_by_style, dict(htally), n_swept


def _prime_layout(config):
    stage = Usd.Stage.CreateInMemory()
    binfo = {}
    ss.generate_suburb_on_stage(stage, config, parent_path=PARENT,
                               scene_scale_factor=1.0, info_out=binfo,
                               assembly=True)
    region = tuple(binfo.get("region") or (-250.0, -250.0, 250.0, 250.0))
    return (region, binfo.get("house_instances", []),
           binfo.get("tree_instances", []))


def run_preset(preset, seed, out_dir):
    t0 = time.time()
    config = cd.load_scene_config(preset)
    region, houses_ss, trees_ss = _prime_layout(config)
    span = max(region[2] - region[0], region[3] - region[1])

    hcfg = hu.resolve_cfg(config)
    inten = hu.intensity_field(hcfg, region, np.random.default_rng(seed + 23))

    _hsub = ((config.get("disaster") or {}).get("hurricane") or {})
    scfg = sgw.resolve_cfg({k: v for k, v in _hsub.items()
                            if k in sgw.DEFAULTS})
    scfg.update(sgw.knobs_from_env(span))
    for k, v in _hsub.items():
        if k in sgw.DEFAULTS:
            scfg[k] = v
    depth = sgw.depth_at(scfg, region, np.random.default_rng(seed + 41))
    water_level = sgw.water_level(scfg)

    house_recs, wrecks, fp_by_style, htally, n_swept = _build_houses(
        config, seed, houses_ss, inten, scfg, depth)
    print("[{0}] {1:.0f}s layout: {2} house(s), {3} tree(s); levels {4}; "
          "{5} swept; water_level={6:.2f}m shore_bearing={7:.0f}deg".format(
              preset, time.time() - t0, len(house_recs), len(trees_ss),
              htally, n_swept, water_level, scfg["shore_bearing_deg"]))

    ctx = {
        "region": region, "wrecks": wrecks, "houses": house_recs,
        "fp_by_style": fp_by_style, "depth_at": depth,
        "water_level": water_level,
        "shore_bearing_deg": float(scfg["shore_bearing_deg"]),
        "intensity_at": inten, "humans": list(HUMANS),
        "resolver": _Resolver(), "asset_pools": _Pools(),
        "plank_specs": [], "deck_points": [],
    }
    cfg = hp.resolve_cfg(config)
    rng = random.Random(seed + 191)
    t1 = time.time()
    humans, debris, records = hp.plan_people(cfg, ctx, rng)
    summ = hp.summarise(records)
    print("[{0}] {1:.1f}s people: {2}".format(
        preset, time.time() - t1, summ))

    _draw(preset, seed, out_dir, region, house_recs, trees_ss, depth,
         water_level, records)
    return records, summ


def _sample_depth_grid(region, depth_at, n=180):
    x0, y0, x1, y1 = region
    xs = np.linspace(x0, x1, n)
    ys = np.linspace(y0, y1, n)
    g = np.zeros((n, n), dtype=np.float32)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            g[j, i] = depth_at(x, y)
    return g


def _draw(preset, seed, out_dir, region, house_recs, trees_ss, depth_at,
         water_level, records):
    x0, y0, x1, y1 = region
    span_m = max(x1 - x0, y1 - y0)
    px_per_m = 1600.0 / span_m
    w = int((x1 - x0) * px_per_m)
    h = int((y1 - y0) * px_per_m)

    def to_px(x, y):
        return ((x - x0) * px_per_m, (y1 - y) * px_per_m)   # north-up

    im = Image.new("RGB", (w, h), (235, 235, 228))
    dr = ImageDraw.Draw(im, "RGBA")

    # -- water depth shading, coarse grid -----------------------------------
    grid = _sample_depth_grid(region, depth_at, n=140)
    gh, gw = grid.shape
    cw, ch = w / float(gw), h / float(gh)
    depth_cap = 2.5
    for j in range(gh):
        for i in range(gw):
            d = float(grid[j, i])
            if d <= hp.DRY_DEPTH_M:
                continue
            a = int(60 + 150 * min(1.0, d / depth_cap))
            # world y increases with j (linspace y0->y1); screen row is
            # (gh-1-j) for north-up.
            row = gh - 1 - j
            dr.rectangle([i * cw, row * ch, (i + 1) * cw, (row + 1) * ch],
                        fill=(40, 90, 160, a))

    # -- houses --------------------------------------------------------------
    for hh in house_recs:
        cx, cy = to_px(hh["x"], hh["y"])
        col = LEVEL_COLOR.get(hh["level"], (120, 120, 120))
        r = 5.0
        dr.rectangle([cx - r, cy - r, cx + r, cy + r], fill=col,
                    outline=(30, 30, 30))

    # -- trees (faint, context only) -----------------------------------------
    for t in trees_ss:
        cx, cy = to_px(t["x"], t["y"])
        dr.ellipse([cx - 1.5, cy - 1.5, cx + 1.5, cy + 1.5],
                  fill=(60, 130, 60, 120))

    # -- people ---------------------------------------------------------------
    marker = {"dry_wreck": (200, 20, 20), "water": (0, 190, 220),
             "roof": (255, 150, 0)}
    for r in records:
        cx, cy = to_px(r["x"], r["y"])
        col = marker.get(r["domain"], (0, 0, 0))
        rad = 5.5
        if r["domain"] == "roof":
            dr.polygon([(cx, cy - rad), (cx - rad, cy + rad),
                       (cx + rad, cy + rad)], fill=col, outline=(0, 0, 0))
            # FACING TICK — a short line in `r["yaw"]`'s own world direction
            # (this file's convention, matching `hurricane_people.
            # _facing_yaw_for_dir`: forward = (sin(yaw), -cos(yaw))), so a
            # reviewer can see at a glance that every roof figure faces
            # outward/downhill rather than along the ridge or into the roof.
            yaw_rad = math.radians(r.get("yaw", 0.0))
            fwd_x, fwd_y = math.sin(yaw_rad), -math.cos(yaw_rad)
            tx, ty = to_px(r["x"] + 1.6 * fwd_x, r["y"] + 1.6 * fwd_y)
            dr.line([(cx, cy), (tx, ty)], fill=(0, 0, 0), width=2)
        elif r["domain"] == "water":
            dr.ellipse([cx - rad, cy - rad, cx + rad, cy + rad], fill=col,
                     outline=(0, 60, 80))
        else:
            dr.line([(cx - rad, cy - rad), (cx + rad, cy + rad)],
                   fill=col, width=2)
            dr.line([(cx - rad, cy + rad), (cx + rad, cy - rad)],
                   fill=col, width=2)

    # -- legend ----------------------------------------------------------------
    ly = 10
    dr.rectangle([8, 8, 430, 190], fill=(255, 255, 255, 210))
    dr.text((14, ly), "%s (seed %d)" % (preset, seed), fill=(0, 0, 0))
    ly += 16
    dr.text((14, ly), "water depth shading: darker = deeper (cap %.1f m)"
           % depth_cap, fill=(0, 0, 0))
    ly += 16
    for lvl, col in LEVEL_COLOR.items():
        dr.rectangle([14, ly, 24, ly + 10], fill=col, outline=(0, 0, 0))
        dr.text((30, ly - 2), lvl, fill=(0, 0, 0))
        ly += 14
    ly += 4
    dr.line([(14, ly), (24, ly + 10)], fill=marker["dry_wreck"], width=2)
    dr.line([(14, ly + 10), (24, ly)], fill=marker["dry_wreck"], width=2)
    dr.text((30, ly - 2), "dry-land casualty (tornado_people, verbatim)",
           fill=(0, 0, 0))
    ly += 16
    dr.ellipse([14, ly, 24, ly + 10], fill=marker["water"])
    dr.text((30, ly - 2), "in the water, chest-deep", fill=(0, 0, 0))
    ly += 16
    dr.polygon([(19, ly), (14, ly + 10), (24, ly + 10)], fill=marker["roof"])
    dr.text((30, ly - 2),
           "on a roof (pristine only), seated on the slope; tick = facing",
           fill=(0, 0, 0))

    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "%s_people.png" % preset)
    im.save(out_path)
    print("[{0}] -> {1}".format(preset, out_path))
    return out_path


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--preset",
                   default="suburb_hurricane_500_l3,suburb_hurricane_500_l2")
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--out-dir",
                   default=os.path.expanduser(
                       "~/hurricane_previews/offline/people"))
    args = ap.parse_args(argv)
    for preset in [p.strip() for p in args.preset.split(",") if p.strip()]:
        run_preset(preset, args.seed, args.out_dir)


if __name__ == "__main__":
    main()
