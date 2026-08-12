"""plan_png.py — top-down plan of the layout, without Isaac Sim.

    python3 tools/plan_png.py --config urban_v2 --out /tmp/plan.png

WHY THIS WORKS WITHOUT USD
--------------------------
`build_city` is pure geometry: 1,600 lines that never touch a USD API. It needs
`pxr` only because `scene_generator` imports it at module scope, and it needs a
*resolver* only to ask each asset how big it is. Stub the first and answer the
second from the measurements already recorded in the asset-set comments, and the
whole layout pipeline — subdivision, packing, districts, parks — runs on the
host in a second.

That turns "is the layout right?" from a container launch into a PNG, which is
the loop worth having while the layout is still moving.

WHAT IT IS NOT
--------------
A render. It shows WHERE things are, not what they look like: no materials, no
heights, no props. Footprints come from the comments, so an asset whose comment
is stale or missing falls back to `fallback_sizes` and is drawn hatched — those
are the ones not to trust.
"""

import argparse
import os
import re
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _SCENE_GEN)

# scene_generator imports pxr at module scope; nothing we call touches it.
for _m in ("pxr", "pxr.Gf", "pxr.Sdf", "pxr.Usd", "pxr.UsdGeom",
           "pxr.UsdShade", "pxr.UsdSkel", "pxr.Vt", "pxr.UsdPhysics"):
    sys.modules.setdefault(_m, types.ModuleType(_m))
for _n in ("Gf", "Sdf", "Usd", "UsdGeom", "UsdShade", "UsdSkel", "Vt",
           "UsdPhysics"):
    setattr(sys.modules["pxr"], _n, types.ModuleType(_n))

import yaml                                                    # noqa: E402

# "# 21.1 x  6.8 x 14.2 m" — the format every measured comment already uses.
_SIZE = re.compile(r"#\s*([\d.]+)\s*x\s*([\d.]+)\s*x\s*([\d.]+)\s*m")
_USD = re.compile(r'["\']([^"\']+\.usda?)["\']')


def measured_sizes(paths):
    """`{usd basename: (sx, sy, sz)}` scraped from the asset-set comments.

    The comment may sit on the entry's own line or the line after it, so both
    are considered. Basename rather than full path: the same asset is written
    with different prefixes in different pools.
    """
    out, pending = {}, None
    for path in paths:
        if not os.path.exists(path):
            continue
        for line in open(path):
            usd = _USD.search(line)
            size = _SIZE.search(line)
            if usd and size:
                out[os.path.basename(usd.group(1))] = tuple(
                    float(size.group(i)) for i in (1, 2, 3))
                pending = None
            elif usd:
                pending = os.path.basename(usd.group(1))
            elif size and pending:
                out[pending] = tuple(float(size.group(i)) for i in (1, 2, 3))
                pending = None
    return out


class StubResolver:
    """Answers footprint queries from the measured table, or the config's
    `fallback_sizes`. Records which assets fell back so the plan can flag them."""

    def __init__(self, sizes, fallbacks):
        self.sizes = sizes
        self.fallbacks = fallbacks or {}
        self.guessed = set()

    def get(self, usd, category, scale=1.0, axis_up="Z", **_kw):
        name = os.path.basename(str(usd))
        wh = self.sizes.get(name)
        if wh is None:
            fb = self.fallbacks.get(category) or [4.0, 4.0, 4.0]
            wh = (float(fb[0]), float(fb[1]),
                  float(fb[2]) if len(fb) > 2 else 4.0)
            self.guessed.add(name or category)
        return {"sx": wh[0], "sy": wh[1], "sz": wh[2], "base": 0.0,
                "cx": 0.0, "cy": 0.0}


def build(config_name):
    import random
    from compile_disaster import resolve_config_path, compile_spec, DEFAULT_BASE
    import scene_generator as sg
    from layout import city_layout
    from detail import districts

    path = resolve_config_path(config_name)
    cfg = compile_spec(yaml.safe_load(open(path)),
                       yaml.safe_load(open(DEFAULT_BASE)))
    cfg = sg.resolve_asset_set(cfg, path)
    cfg["measure_usds"] = False

    sets = os.path.join(_SCENE_GEN, "config", "asset_sets")
    sizes = measured_sizes([os.path.join(sets, f)
                            for f in os.listdir(sets) if f.endswith(".yaml")])
    res = StubResolver(sizes, cfg.get("fallback_sizes"))
    rng = random.Random(int(cfg.get("seed", 0)) + 7717)

    with city_layout.patched(cfg):
        placements, layout = sg.build_city(cfg, res)
    da, rings = districts.assign(cfg, layout)
    if rings:
        districts.remap_buildings(cfg, layout, placements, res, rng, da)
    return cfg, layout, placements, res


_COLOUR = {"rowhouse": "#b5651d", "midrise": "#7f8fa6",
           "tower": "#4b5d73", "park": "#5f8d4e"}


def draw(cfg, layout, placements, res, out_path, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    x0, y0, x1, y1 = layout["region"]
    fig, ax = plt.subplots(figsize=(13, 13))
    ax.set_facecolor("#2b2b2b")                       # asphalt shows through
    typ_of = layout.get("_typology_of") or {}

    for b in layout.get("blocks", []):
        t = typ_of.get(tuple(b)) or typ_of.get(b)
        ax.add_patch(Rectangle((b[0], b[1]), b[2] - b[0], b[3] - b[1],
                               facecolor="#3f4a3a" if t == "park" else "#4a4a44",
                               edgecolor="none", zorder=1))
    for c in layout.get("road_corridors", []):
        ax.add_patch(Rectangle((c["x0"], c["y0"]), c["x1"] - c["x0"],
                               c["y1"] - c["y0"],
                               facecolor="#1f1f1f",
                               edgecolor="#00d0ff" if c.get("internal") else "none",
                               lw=1.2, zorder=2))

    # Park circulation, drawn before the buildings so nothing hides it. In the
    # sim the walks disappear under the canopy, which is exactly why they are
    # hard to judge there and easy to judge here.
    park_layers = (("trail", "#c9bfa6", 1.0, 4),
                   ("park_feature", "#5aa9e6", 1.6, 6),
                   ("play_structure", "#e8a33d", 1.4, 6),
                   ("bench", "#8d6e4a", 0.9, 5),
                   ("fence", "#6b7a5a", 0.5, 4))
    n_park = {}
    for cat, colour, pad, z in park_layers:
        for p in placements:
            if p.get("category") != cat:
                continue
            fp = res.get(p.get("usd", ""), cat)
            w = max(fp["sx"] * pad, 1.2)
            h = max(fp["sy"] * pad, 1.2)
            ax.add_patch(Rectangle((p["x_m"] - w / 2, p["y_m"] - h / 2), w, h,
                                   facecolor=colour, edgecolor="none", zorder=z))
            n_park[cat] = n_park.get(cat, 0) + 1

    counts = {}
    for p in placements:
        if p.get("category") not in ("house", "building"):
            continue
        fp = res.get(p.get("usd", ""), "house")
        yaw = float(p.get("yaw_deg", 0.0)) % 180.0
        w, h = (fp["sy"], fp["sx"]) if 45 <= yaw < 135 else (fp["sx"], fp["sy"])
        t = typ_of.get(_block_of(layout, p)) or "midrise"
        # Colour by what was BUILT, not by what the zone asked for. A block
        # whose terrace layout refused it still carries the rowhouse zone, and
        # coluring off that drew ordinary packed mid-rise in brownstone orange —
        # which reads as a terrace block with far more than two rows.
        if t == "rowhouse" and "Brownstone" not in os.path.basename(
                str(p.get("usd", ""))):
            t = "midrise"
        counts[t] = counts.get(t, 0) + 1
        ax.add_patch(Rectangle((p["x_m"] - w / 2, p["y_m"] - h / 2), w, h,
                               facecolor=_COLOUR.get(t, "#7f8fa6"),
                               edgecolor="#111", lw=0.3, zorder=3))

    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    legend = "   ".join(f"{k} {v}" for k, v in sorted(counts.items()))
    park = "  ".join(f"{k} {v}" for k, v in sorted(n_park.items()))
    sub = f"{len(layout.get('blocks', []))} blocks   " \
          f"{len(layout.get('road_corridors', []))} corridors   {legend}"
    if park:
        sub += f"\npark: {park}"
    if res.guessed:
        sub += f"   [{len(res.guessed)} assets un-measured]"
    ax.set_title(f"{title}\n{sub}", color="#ddd", fontsize=10)
    fig.patch.set_facecolor("#1b1b1b")
    fig.savefig(out_path, dpi=110, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"[plan] {out_path}")
    print(f"[plan] {sub}")


def _block_of(layout, p):
    x, y = p["x_m"], p["y_m"]
    for b in layout.get("blocks", []):
        if b[0] <= x <= b[2] and b[1] <= y <= b[3]:
            return tuple(b)
    return None


def dump_json(cfg, layout, placements, res, out_path):
    """The same scene as machine-readable geometry.

    A PNG says something is wrong; this says WHAT. Every block with its measured
    size and typology, every corridor, every building with its footprint — so a
    bad layout can be diagnosed by querying it instead of by squinting at
    pixels, which is how several of these bugs got misdiagnosed.
    """
    import json
    typ_of = layout.get("_typology_of") or {}
    blocks = []
    for b in layout.get("blocks", []):
        t = typ_of.get(tuple(b)) or typ_of.get(b)
        inner = [p for p in placements
                 if p.get("category") in ("house", "building")
                 and b[0] <= p["x_m"] <= b[2] and b[1] <= p["y_m"] <= b[3]]
        area = (b[2] - b[0]) * (b[3] - b[1])
        built = 0.0
        for p in inner:
            fp = res.get(p.get("usd", ""), "house")
            built += fp["sx"] * fp["sy"]
        blocks.append({
            "rect": [round(v, 2) for v in b],
            "w": round(b[2] - b[0], 2), "h": round(b[3] - b[1], 2),
            "short": round(min(b[2] - b[0], b[3] - b[1]), 2),
            "long": round(max(b[2] - b[0], b[3] - b[1]), 2),
            "typology": t, "buildings": len(inner),
            "built_frac": round(built / area, 3) if area else 0.0,
        })
    doc = {
        "config": cfg.get("_name", ""),
        "region": [round(v, 2) for v in layout["region"]],
        "blocks": blocks,
        "corridors": [{"rect": [round(float(c[k]), 2)
                                for k in ("x0", "y0", "x1", "y1")],
                       "dir": c.get("dir"), "n_lanes": c.get("n_lanes"),
                       "internal": bool(c.get("internal"))}
                      for c in layout.get("road_corridors", [])],
        "buildings": [{"usd": os.path.basename(str(p.get("usd", ""))),
                       "x": round(p["x_m"], 2), "y": round(p["y_m"], 2),
                       "yaw": round(float(p.get("yaw_deg", 0.0)), 1)}
                      for p in placements
                      if p.get("category") in ("house", "building")],
        "unmeasured": sorted(res.guessed),
    }
    with open(out_path, "w") as fh:
        json.dump(doc, fh, indent=1)
    print(f"[plan] {out_path}")
    return doc


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", action="store_true",
                    help="also write <out>.json — geometry, not pixels")
    ap.add_argument("--config", default="urban_v2")
    # Defaults into the repo, not /tmp: a plan is meant to be looked at, and a
    # scratch path nobody can find is the same as not writing one. Gitignored.
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    if not a.out:
        d = os.path.join(_SCENE_GEN, "_plans")
        os.makedirs(d, exist_ok=True)
        a.out = os.path.join(d, f"{a.config}.png")
    cfg, layout, placements, res = build(a.config)
    draw(cfg, layout, placements, res, a.out, title=a.config)
    if a.json:
        dump_json(cfg, layout, placements, res,
                  os.path.splitext(a.out)[0] + ".json")


if __name__ == "__main__":
    main()
