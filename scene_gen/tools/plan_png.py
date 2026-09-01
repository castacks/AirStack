"""plan_png.py — top-down plan of the layout, without Isaac Sim.

    python3 tools/plan_png.py --config downtown --out /tmp/plan.png

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
import json
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

# "# 21.1 x  6.8 x 14.2 m" was the first format every measured comment used,
# and the ORIGINAL regex anchored the three numbers immediately after the
# `#`. `urban_gac.yaml` broke that anchor on purpose — its comments put the
# measured place/front/blank prose FIRST and the size last: "# 1-sided mid,
# front E, blank N,W,S, 29x28x55 m". Anchoring only on "a `#` appears
# somewhere earlier on the line", not on the size immediately following it,
# reads both orderings; `.*?` is non-greedy so it still finds the FIRST size
# pattern after the `#`, not some later one. Tolerates both the compact
# rounded form GAC/downtowncity comments use ("29x28x55 m") and the spaced
# decimal form the original brownstone comments use ("21.1 x  6.8 x 14.2 m").
_SIZE = re.compile(r"#.*?([\d.]+)\s*x\s*([\d.]+)\s*x\s*([\d.]+)\s*m\b")
# `.usdc` too: every asset in the 2026-08-26 building drop is a `.usdc`, and
# without it they scraped no size, fell back to `fallback_sizes.house` and drew
# as 100 identical 30 x 20 m boxes — a plan that looks like a packing bug and
# is not one.
_USD = re.compile(r'["\']([^"\']+\.usd[ac]?)["\']')


def measured_sizes(paths):
    """`{usd basename: (sx, sy, sz)}` scraped from the asset-set comments.

    The comment may sit on the entry's own line or the line after it, so both
    are considered. Basename rather than full path: the same asset is written
    with different prefixes in different pools.

    THE FALLBACK SOURCE, not the primary one — see `measured_json`. A comment
    is prose a human wrote for a human; nothing enforces that its numbers
    stay in sync with the asset, or that the regex above keeps matching
    whatever style the next asset-set author picks. Kept only for libraries
    nobody has run a `gac_faces.py`-style measurement pass over yet.
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


def measured_json(paths, default_ext=".usd"):
    """`{usd basename: (sx, sy, sz)}` from `gac_faces.py`-style measurement
    files: a JSON list of ``{"name", "usd"?, "W", "D", "H", ...}`` records,
    one per building, written by actually opening the USD and measuring its
    bbox — `tools/gac_faces.py`, `tools/gac_props_measure.py`'s building
    counterpart. AUTHORITATIVE over `measured_sizes`'s comment-scrape: a
    comment is free text that happens to describe the asset, this is the
    measurement the comment was transcribed FROM.

    Keyed by the record's own ``usd`` field when present (its basename is
    exact — `dtc_faces.json` and `gac_faces.json` both carry it); falls back
    to ``name + default_ext`` when it is not (`gac_buildings.json` has no
    `usd` field, and every GreatAmericanCity asset is `.usd`, never `.usdc`).
    W/D/H map to sx/sy/sz — the same axis order `measured_sizes`'s comments
    already use, because those comments were generated FROM these numbers
    (`tools/faces_to_yaml.py`).
    """
    out = {}
    for path in paths:
        if not os.path.exists(path):
            continue
        try:
            rows = json.load(open(path))
        except (OSError, ValueError):
            continue
        for row in rows or ():
            w, d, h = row.get("W"), row.get("D"), row.get("H")
            if w is None or d is None or h is None:
                continue
            usd = row.get("usd")
            base = (os.path.basename(str(usd)) if usd
                    else (str(row.get("name", "")) + default_ext))
            if base and base != default_ext:
                out[base] = (float(w), float(d), float(h))
    return out


class StubResolver:
    """Answers footprint queries from the measured JSON table (authoritative),
    the comment-scrape table (fallback), or the config's `fallback_sizes`
    (last resort). Records which assets fell back — split by BUILDING vs prop
    category, because a plan built from fallback boxes for the buildings it
    exists to validate is worse than no plan, and a bare count buried that."""

    def __init__(self, sizes, fallbacks, json_sizes=None):
        self.sizes = sizes
        self.json_sizes = json_sizes or {}
        self.fallbacks = fallbacks or {}
        self.guessed = set()                # union, for callers that don't care
        self.guessed_buildings = set()
        self.guessed_props = set()

    def get(self, usd, category, scale=1.0, axis_up="Z", **_kw):
        name = os.path.basename(str(usd))
        wh = self.json_sizes.get(name) or self.sizes.get(name)
        if wh is None:
            fb = self.fallbacks.get(category) or [4.0, 4.0, 4.0]
            wh = (float(fb[0]), float(fb[1]),
                  float(fb[2]) if len(fb) > 2 else 4.0)
            tag = name or category
            self.guessed.add(tag)
            (self.guessed_buildings if category in ("house", "building")
             else self.guessed_props).add(tag)
        return {"sx": wh[0], "sy": wh[1], "sz": wh[2], "base": 0.0,
                "cx": 0.0, "cy": 0.0}


def build(config_name, seed=None, spec_overrides=None):
    """*seed*, when given, overrides the preset's own — so a caller can
    run the same scene many times and get a DISTRIBUTION rather than one
    roll of the dice. See `tools/asset_fit_audit.py --seeds`.

    *spec_overrides* is applied to the RAW loaded spec dict before
    `compile_spec`, exactly the mechanism `tools/layout_dry_run.py` /
    `tools/fire_city_dry_run.py` already use for `region_m`/`asset-set`/
    `disaster-type` (see `compile_disaster.load_scene_config`'s own
    `spec_overrides` parameter). NEEDED for any `disaster-type: fire` preset
    (`downtown_fire_500`/`downtown_fire_1500`): `"fire"` is not a compiled
    disaster type (see those presets' own header comment), so `compile_spec`
    raises on one unless the caller overrides it to `"none"` for layout
    purposes the same way `fire_city_dry_run.build_layout` does — this
    function has no other way to plan such a preset."""
    import random
    from compile_disaster import resolve_config_path, compile_spec, DEFAULT_BASE
    import scene_generator as sg
    from layout import city_layout
    from detail import districts

    path = resolve_config_path(config_name)
    spec = yaml.safe_load(open(path))
    if spec_overrides:
        spec = dict(spec)
        spec.update(spec_overrides)
    cfg = compile_spec(spec, yaml.safe_load(open(DEFAULT_BASE)))
    cfg = sg.resolve_asset_set(cfg, path)
    cfg["measure_usds"] = False
    if seed is not None:
        cfg["seed"] = int(seed)

    sets = os.path.join(_SCENE_GEN, "config", "asset_sets")
    sizes = measured_sizes([os.path.join(sets, f)
                            for f in os.listdir(sets) if f.endswith(".yaml")])
    # AUTHORITATIVE over the comment-scrape: real per-asset measurements from
    # actually opening the USD, keyed by exact basename where the file gives
    # one. `gac_buildings.json` has no `usd` field (GAC is always `.usd`);
    # `dtc_faces.json`/`gac_faces.json` do, so they win on any name collision
    # via dict update order below.
    plans = os.path.join(_SCENE_GEN, "_plans")
    json_sizes = {}
    json_sizes.update(measured_json([os.path.join(plans, "gac_buildings.json")],
                                    default_ext=".usd"))
    json_sizes.update(measured_json([os.path.join(plans, "gac_faces.json")]))
    json_sizes.update(measured_json([os.path.join(plans, "dtc_faces.json")]))
    res = StubResolver(sizes, cfg.get("fallback_sizes"), json_sizes)
    rng = random.Random(int(cfg.get("seed", 0)) + 7717)

    with city_layout.patched(cfg):
        placements, layout = sg.build_city(cfg, res)
    da, rings = districts.assign(cfg, layout)
    if rings:
        districts.remap_buildings(cfg, layout, placements, res, rng, da)
    if res.guessed_buildings and seed is None:
        print(f"[plan] {len(res.guessed_buildings)} BUILDING asset(s) "
              f"un-measured, drawn as fallback boxes: "
              f"{', '.join(sorted(res.guessed_buildings))}")
    if res.guessed_props and seed is None:
        print(f"[plan] {len(res.guessed_props)} prop asset(s) un-measured: "
              f"{', '.join(sorted(res.guessed_props))}")
    return cfg, layout, placements, res


_COLOUR = {"rowhouse": "#b5651d", "midrise": "#7f8fa6",
           "tower": "#4b5d73", "park": "#5f8d4e"}


def _model_colour(usd):
    """A stable colour per MODEL, so the plan can be read for repetition.

    Typology colouring answers "is the zoning right"; it cannot answer "is the
    same building standing three times on one street", which is the other half
    of whether a city looks generated. Here two footprints of the same colour
    are the same asset, so a cluster of one colour IS the defect — no legend
    needed and no counting.

    Hue from a hash of the basename, spread with the golden ratio so adjacent
    hash values land far apart on the wheel; saturation and value jittered off
    the same hash so two models that collide in hue still separate.
    """
    import colorsys
    h = 0
    for ch in os.path.basename(str(usd)):
        h = (h * 131 + ord(ch)) & 0xFFFFFFFF
    hue = ((h * 0.6180339887) % 1.0)
    sat = 0.45 + 0.40 * (((h >> 8) & 0xFF) / 255.0)
    val = 0.55 + 0.40 * (((h >> 16) & 0xFF) / 255.0)
    r, g, b = colorsys.hsv_to_rgb(hue, sat, val)
    return "#%02x%02x%02x" % (int(r * 255), int(g * 255), int(b * 255))


def draw(cfg, layout, placements, res, out_path, title="", by_model=False,
        crop_window=None):
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

    counts, models = {}, {}
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
        models[os.path.basename(str(p.get("usd", "")))] = 1
        face = (_model_colour(p.get("usd", "")) if by_model
                else _COLOUR.get(t, "#7f8fa6"))
        ax.add_patch(Rectangle((p["x_m"] - w / 2, p["y_m"] - h / 2), w, h,
                               facecolor=face,
                               edgecolor="#111", lw=0.3, zorder=3))

    if crop_window is not None:
        # `tools/baseline_layouts.py`'s own draw: the 1 km crop window this
        # LEVEL will actually solve/export on, over the full 1.5 km plate --
        # crimson, on top of everything, so it reads at a glance which
        # districts the window keeps vs cuts. Drawn on the UNCROPPED map;
        # crop_window.crop_layout's own output is what a caller draws
        # separately for "after".
        cwx0, cwy0, cwx1, cwy1 = crop_window
        ax.add_patch(Rectangle((cwx0, cwy0), cwx1 - cwx0, cwy1 - cwy0,
                               facecolor="none", edgecolor="#ff2d55",
                               lw=2.4, ls="--", zorder=10))

    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    legend = "   ".join(f"{k} {v}" for k, v in sorted(counts.items()))
    park = "  ".join(f"{k} {v}" for k, v in sorted(n_park.items()))
    sub = f"{len(layout.get('blocks', []))} blocks   " \
          f"{len(layout.get('road_corridors', []))} corridors   {legend}   " \
          f"{len(models)} distinct models"
    if by_model:
        sub += "  (one colour per model — a cluster of one colour is a repeat)"
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


def audit(cfg, layout, placements, res):
    """Print, without drawing anything: per-typology block/building counts,
    the per-model histogram (WITH each model's height — a count alone cannot
    show a height-distribution problem, only a repetition one), the unused
    model list, and the blank-wall-to-street audit.

    The last of those is the acceptance test for the whole facing feature
    (`_pool_entries`'s `blank0`, `_pack_free`'s street filter, `_lay_terrace`'s
    per-strip filter and `_order_run`): for every PLACED building that
    carries a `blank:` tag, recompute its WORLD blank sides straight from its
    final `yaw_deg` (no need to go through `blank0` here — that intermediate
    exists only because `_pack_free`/`_lay_terrace` don't know `yaw-offset`
    at the point they choose a placement yaw; the placed building's `yaw_deg`
    already has it folded in) and test them against its own block's edges
    with `districts._street_sides`, the exact function `_pack_free` used to
    decide whether to place it there in the first place. Zero violations
    means the placement pass and the audit agree; anything else is a real
    defect, not a measurement artefact.
    """
    from detail import districts

    typ_of = layout.get("_typology_of") or {}
    blocks = layout.get("blocks", [])
    blocks_by_typ, buildings_by_typ = {}, {}
    for b in blocks:
        t = typ_of.get(tuple(b)) or typ_of.get(b) or "(unzoned)"
        blocks_by_typ[t] = blocks_by_typ.get(t, 0) + 1
    counts, heights = {}, {}
    for p in placements:
        if p.get("category") not in ("house", "building"):
            continue
        t = typ_of.get(_block_of(layout, p)) or "(unzoned)"
        buildings_by_typ[t] = buildings_by_typ.get(t, 0) + 1
        name = os.path.basename(str(p.get("usd", "")))
        counts[name] = counts.get(name, 0) + 1
        if name not in heights:
            heights[name] = res.get(p.get("usd", ""), "house")["sz"]

    print("[audit] blocks by typology:  " +
          "  ".join(f"{k}={v}" for k, v in sorted(blocks_by_typ.items())))
    print("[audit] buildings by typology:  " +
          "  ".join(f"{k}={v}" for k, v in sorted(buildings_by_typ.items())))

    n_total = sum(counts.values())
    top = sorted(counts.items(), key=lambda kv: -kv[1])[:15]
    print(f"[audit] model histogram (top {len(top)} of {len(counts)} "
          f"distinct, {n_total} buildings):")
    for name, n in top:
        pct = 100.0 * n / n_total if n_total else 0.0
        print(f"[audit]   {name:28s} {n:4d} ({pct:5.1f}%)   "
              f"H {heights.get(name, 0.0):6.1f}")

    # `usd -> meta` rebuilt from config through the SAME pool machinery the
    # real pipeline used — `_pool_entries` is pure (config, resolver) -> pool,
    # so calling it again after the fact is exactly as valid as the call the
    # pipeline itself made, and it is the only place `blank:`/`place` tags
    # get parsed out of the YAML.
    usds_cfg = (cfg.get("usds") or {}).get("buildings") or {}
    meta_by_usd = {}
    for key in usds_cfg:
        for e in districts._pool_entries(cfg, res, key):
            meta_by_usd[os.path.basename(e[0])] = e[5]

    # `models_unused` is scoped to the TYPOLOGY pools (`districts.typologies.
    # *.pools`, non-terrace), matching exactly what `[districts] models_unused`
    # already reports in the real pipeline log. The wider `usds.buildings.*`
    # set also carries `damaged`/`destroyed` stock that `build_city` places
    # directly and `districts` never touches — counting those as "unused"
    # would be true but meaningless, since they were never candidates here.
    dcfg = cfg.get("districts") or {}
    typologies = dcfg.get("typologies") or {}
    pool_names = set()
    for name, t in typologies.items():
        if str(t.get("morphology", "pack")) == "terrace":
            continue                    # ROW HOUSES NEVER REACH THE SKYLINE
        for key in (t.get("pools") or [name]):
            for e in districts._pool_entries(cfg, res, key):
                pool_names.add(os.path.basename(e[0]))
    unused = sorted(pool_names - set(counts))
    if unused:
        print(f"[audit] models_unused ({len(unused)} of {len(pool_names)}): "
              f"{', '.join(unused)}")

    violations = _blank_wall_violations(cfg, layout, placements, res,
                                        meta_by_usd)
    print(f"[audit] blank-wall-to-street violations: {len(violations)}")
    for name, x, y, yaw, bad in violations[:20]:
        print(f"[audit]   {name} at ({x:.1f}, {y:.1f}) yaw={yaw:.0f}  "
              f"blank side(s) {','.join(bad)} face the street")
    if len(violations) > 20:
        print(f"[audit]   ... and {len(violations) - 20} more")


def _blank_wall_violations(cfg, layout, placements, res, meta_by_usd=None):
    """The list the `--audit` blank-wall count is built from —
    ``[(usd_basename, x, y, yaw_deg, bad_sides)]`` for every PLACED building
    carrying a `blank:` tag whose world-frame blank sides overlap its own
    block's street-facing sides. Pulled out of `audit()` so `audit_selftest`
    can run the identical measurement against a differently-built placement
    list — see there for why that matters.
    """
    from detail import districts
    if meta_by_usd is None:
        meta_by_usd = {}
        for key in (cfg.get("usds") or {}).get("buildings") or {}:
            for e in districts._pool_entries(cfg, res, key):
                meta_by_usd[os.path.basename(e[0])] = e[5]
    inset = districts.block_inset(cfg, res)
    violations = []
    for p in placements:
        if p.get("category") not in ("house", "building"):
            continue
        meta = meta_by_usd.get(os.path.basename(str(p.get("usd", ""))))
        if not meta or not meta.get("blank"):
            continue
        blk = _block_of(layout, p)
        if blk is None:
            continue
        rect = (blk[0] + inset, blk[1] + inset, blk[2] - inset, blk[3] - inset)
        x0, y0, x1, y1 = districts._rect_of(p, res)
        sides = districts._street_sides(rect, x0, y0, x1 - x0, y1 - y0)
        yaw = float(p.get("yaw_deg", 0.0))
        bad = districts._rot_sides(meta["blank"], yaw) & sides
        if bad:
            violations.append((os.path.basename(str(p.get("usd", ""))),
                              p["x_m"], p["y_m"], yaw, sorted(bad)))
    return violations


def audit_selftest(config_name, seed=None, spec_overrides=None):
    """Prove the blank-wall audit is not vacuous.

    `_pack_free`'s facing filter and the audit's own violation count share
    the same two functions (`_street_sides`, `_rot_sides`) by design — that
    is what makes "the placement pass and the audit agree" a meaningful
    claim in `audit()`'s docstring. It also means a run with the real
    metadata in place will show 0 violations EVEN IF the audit's measurement
    were silently broken, because the filter already refused every
    candidate that would have violated it — corrupting a tag to be
    maximally wrong (as the initial manual check for this feature did)
    proves the FILTER works, not that the AUDIT would have caught it if the
    filter hadn't.

    So this rebuilds the SAME scene with `districts._street_sides`
    monkey-patched to report NO street sides ever — `_pack_free`'s filter
    then has nothing to refuse, so a `blank:`-tagged building can land with
    its blank side on the street exactly as it would if the filter did not
    exist — and audits THAT placement list with the real `_street_sides`
    restored. A working audit MUST report violations here; if it is 0 too,
    "0 violations" in the real run is unfalsifiable and cannot be trusted as
    this feature's acceptance gate.
    """
    from detail import districts
    real_street_sides = districts._street_sides
    districts._street_sides = lambda *a, **kw: frozenset()
    try:
        cfg, layout, placements, res = build(config_name, seed=seed,
                                             spec_overrides=spec_overrides)
    finally:
        districts._street_sides = real_street_sides   # restore before auditing
    violations = _blank_wall_violations(cfg, layout, placements, res)
    print(f"[audit] SELF-TEST ({config_name}): facing filter neutered "
          f"during placement (nothing was ever refused), then audited with "
          f"the REAL _street_sides — violations: {len(violations)} "
          f"(this must be > 0, or the audit above is untestable)")
    for name, x, y, yaw, bad in violations[:10]:
        print(f"[audit]   SELF-TEST   {name} at ({x:.1f}, {y:.1f}) "
              f"yaw={yaw:.0f}  blank side(s) {','.join(bad)} face the street")
    if len(violations) > 10:
        print(f"[audit]   SELF-TEST   ... and {len(violations) - 10} more")
    return violations


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", action="store_true",
                    help="also write <out>.json — geometry, not pixels")
    ap.add_argument("--config", default="downtown")
    ap.add_argument("--seed", type=int, default=None,
                    help="override the preset's own committed seed")
    ap.add_argument("--fire-layout", action="store_true",
                    help="this preset is `disaster-type: fire` (not a "
                         "compiled disaster type, e.g. downtown_fire_500/"
                         "downtown_fire_1500) -- override it to 'none' for "
                         "layout purposes, the same way "
                         "fire_city_dry_run.build_layout does")
    # Defaults into the repo, not /tmp: a plan is meant to be looked at, and a
    # scratch path nobody can find is the same as not writing one. Gitignored.
    ap.add_argument("--out", default="")
    ap.add_argument("--by-model", action="store_true",
                    help="colour each building by MODEL instead of by "
                         "typology — reads for repetition, not for zoning")
    ap.add_argument("--audit", action="store_true",
                    help="print typology/model/facing diagnostics and draw "
                         "nothing — the acceptance test for the facing "
                         "system, see plan_png.audit()")
    ap.add_argument("--audit-selftest", action="store_true",
                    help="prove the blank-wall audit is not vacuous: rebuild "
                         "with the facing filter neutered and confirm the "
                         "audit then reports violations > 0 — see "
                         "plan_png.audit_selftest(). Implies --audit.")
    a = ap.parse_args()
    if not a.out:
        d = os.path.join(_SCENE_GEN, "_plans")
        os.makedirs(d, exist_ok=True)
        a.out = os.path.join(d, f"{a.config}.png")
    overrides = {"disaster-type": "none"} if a.fire_layout else None
    cfg, layout, placements, res = build(a.config, seed=a.seed,
                                         spec_overrides=overrides)
    if a.audit or a.audit_selftest:
        audit(cfg, layout, placements, res)
        if a.audit_selftest:
            audit_selftest(a.config, seed=a.seed, spec_overrides=overrides)
        return
    draw(cfg, layout, placements, res, a.out, title=a.config,
         by_model=a.by_model)
    if a.json:
        dump_json(cfg, layout, placements, res,
                  os.path.splitext(a.out)[0] + ".json")


if __name__ == "__main__":
    main()
