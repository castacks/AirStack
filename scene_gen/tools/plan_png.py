"""plan_png.py — top-down plan of a generated scene, without Isaac Sim.

    python3 tools/plan_png.py --config earthquake

Writes two maps beside each other:

    <config>_layout.png   the PRISTINE scene — stages 1+2, before any damage
    <config>_damage.png   the same scene after stage 3, shaded by what the
                          disaster did to it, over the damage field

THE SAME PIPELINE THE SIM RUNS
------------------------------
This calls `compile_disaster.load_scene_config` and `generate_scene.build_scene`
— the exact two entry points `scene_launch_script.py` uses under `airstack up`.
It does not reimplement compilation or re-run the stages by hand, because a
preview that drifts from the thing it previews is worse than no preview: this
tool used to call `sg.build_city` directly and so never saw `city_detail` or the
disaster stage at all, and silently kept showing a pristine city for a preset
whose whole point was damage.

    airstack up   ->  load_scene_config  ->  build_scene  ->  apply_placements
                                                              mesh_damage
    plan_png      ->  load_scene_config  ->  build_scene  ->  matplotlib

The split is at exactly the point where the scene stops being numbers and
becomes geometry. Everything left of it is shared; everything right of it is
what this tool exists to avoid paying for.

THE ONE DEVIATION, AND WHY
--------------------------
The resolver. `sg._make_resolver` measures real USD bounding boxes, which needs
every asset resolvable — Nucleus included. `StubResolver` answers from the
sizes already recorded in the asset-set comments instead, so the whole thing
runs on the host in about a second. That is the only substitution, and
un-measured assets are counted in the subtitle so you know which footprints not
to trust.

WHAT IT IS NOT
--------------
A render. It shows WHERE things are, not what they look like: no materials, no
heights, no mesh damage. `_mesh_damage` is an INTENSITY, and the failure mode a
building will suffer is not chosen until `mesh_damage.apply_to_stage` deforms
it — so the damage map can show how hard a building was hit and by which
mechanism, but not how it ends up looking. For that, use
`tools/damage_gallery.py`.

Needs `usd-core` on the interpreter (system `python3`), the same as
`preset_report.py` and `tests/` — `scene_generator` imports `pxr` at module
scope. Nothing here calls a USD API.
"""

import argparse
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _SCENE_GEN)

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
    `fallback_sizes`. Records which assets fell back so the plan can flag them.

    THIS IS AN APPROXIMATION, AND IT MOVES THE LAYOUT
    -------------------------------------------------
    Footprint is what block packing keys off, so a wrong size does not blur the
    plan — it changes it. Measured against the sim on the same config:

        suburb     plan 1947 houses   sim 1853   no shared position
        downtown   plan  266 houses   sim  346

    because the sim measures every USD's real bounds while this reads the
    `# 21.1 x 6.8 x 14.2 m` comments and falls back to `fallback_sizes` for
    anything without one — which on the suburban set is *every* house, since
    those entries carry name comments but not size comments.

    Making this measure the local assets is the obvious fix and is NOT done
    here: a first attempt at it moved suburb to 2049 rather than to 1853,
    i.e. further away, so the divergence is not the comment table alone. Until
    that is understood, the honest tool is the one that says how much it
    guessed — `[N assets un-measured]` in the subtitle — and defers to
    `generate_scene.write_run_plan`, which renders from inside the sim off the
    real resolver and is correct by construction.
    """

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


def build(config_name, stop_after="disaster"):
    """Compile and run the generator exactly as the launch script does.

    `stop_after="detail"` yields the pristine scene (stages 1+2); the default
    runs the disaster stage too. Both come from the same `build_scene` call the
    sim makes, so the two maps cannot disagree with each other or with it.
    """
    from compile_disaster import load_scene_config, resolve_config_path
    import generate_scene

    path = resolve_config_path(config_name)
    cfg = load_scene_config(path)          # <- what scene_launch_script calls
    cfg["measure_usds"] = False            # StubResolver answers instead

    sets = os.path.join(_SCENE_GEN, "config", "asset_sets")
    sizes = measured_sizes([os.path.join(sets, f)
                            for f in os.listdir(sets) if f.endswith(".yaml")])
    res = StubResolver(sizes, cfg.get("fallback_sizes"))

    placements, layout, _counts = generate_scene.build_scene(
        cfg, res, stop_after=stop_after)
    cfg.setdefault("_name", config_name)
    return cfg, layout, placements, res


_COLOUR = {"rowhouse": "#b5651d", "midrise": "#7f8fa6",
           "tower": "#4b5d73", "park": "#5f8d4e"}

# How a building ended up, and how it got there. The distinction that matters
# is MECHANISM, not severity: a ruin swap is authored art dropped on the
# footprint, mesh damage is the procedural pipeline deforming the real model.
# They look nothing alike in the sim and are tuned by different knobs, so the
# map has to tell them apart.
_FATE_COLOUR = {
    "intact":    "#5c6b52",   # untouched by the disaster
    "swapped":   "#c2452d",   # a ruin asset was swapped onto the footprint
    "mesh":      "#e8a33d",   # deformed in place by mesh_damage
}
_DEBRIS_COLOUR = {"debris": "#9a8f7a", "debris_pile": "#6f5f47"}


def classify(placements, pristine_usd):
    """Per house: how the disaster stage left it.

    `_mesh_damage` is the marker `apply_to_buildings` leaves for the pass that
    runs after composition; a changed `usd` means a ruin was swapped in. A
    building can only be one or the other — that is the fate branch — so this
    is a partition, not a heuristic.
    """
    out = []
    for i, p in enumerate(placements):
        if p.get("category") not in ("house", "building"):
            continue
        inten = float(p.get("_mesh_damage") or 0.0)
        if inten > 0.0:
            fate = "mesh"
        elif pristine_usd.get(i) not in (None, p.get("usd")):
            fate = "swapped"
        else:
            fate = "intact"
        out.append((p, fate, inten))
    return out


def field_of(cfg, layout):
    """The damage field as a callable, or None where the type has none.

    The same `make_damage_field` every disaster knob is scaled by, so the
    background of the damage map is literally what drove the fates drawn on
    top of it rather than an artist's impression of them.
    """
    import scene_generator as sg
    dis = sg._stage(cfg, "disaster")
    region = layout.get("region")
    if not dis or not region:
        return None
    f = sg.make_damage_field(dis.get("field"), tuple(region))
    return f if f.hi > 0.0 else None


def draw(cfg, layout, placements, res, out_path, title="",
         damage=None, field=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

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
            fp = footprint(res, p, cat)
            w = max(fp["sx"] * pad, 1.2)
            h = max(fp["sy"] * pad, 1.2)
            ax.add_patch(Rectangle((p["x_m"] - w / 2, p["y_m"] - h / 2), w, h,
                                   facecolor=colour, edgecolor="none", zorder=z))
            n_park[cat] = n_park.get(cat, 0) + 1

    # The damage field, above the block fill and below everything the disaster
    # produced. It has to sit here rather than at the bottom: blocks and road
    # corridors are painted opaque, so a backdrop under them is invisible —
    # which is how the first version of this map came out with no field at all.
    if field is not None:
        import numpy as np
        n = 128
        grid = np.array([[field(x, y) for x in np.linspace(x0, x1, n)]
                         for y in np.linspace(y0, y1, n)])
        # One hue, opacity carrying the value — NOT a colormap. Every diverging
        # or perceptual map peaks in brightness somewhere in its middle, so an
        # `inferno` field drew the mild corners brighter than the full-strength
        # core and read exactly backwards. With a fixed colour and alpha = k,
        # untouched ground is simply untinted and there is nothing to misread.
        rgba = np.zeros(grid.shape + (4,))
        rgba[..., 0], rgba[..., 1], rgba[..., 2] = 0.85, 0.20, 0.10
        rgba[..., 3] = np.clip(grid, 0.0, 1.0) * 0.55
        ax.imshow(rgba, extent=(x0, x1, y0, y1), origin="lower",
                  interpolation="bilinear", zorder=2.5)

    # Debris under the buildings: it is ground cover, and drawn on top it
    # would hide the very fates it is evidence for.
    n_debris = {}
    if damage is not None:
        for cat, colour in _DEBRIS_COLOUR.items():
            for p in placements:
                if p.get("category") != cat:
                    continue
                r = 1.6 if cat == "debris_pile" else 0.9
                ax.add_patch(Circle((p["x_m"], p["y_m"]), r, facecolor=colour,
                                    edgecolor="none", alpha=0.75, zorder=3))
                n_debris[cat] = n_debris.get(cat, 0) + 1

    fate_of = {id(p): (f, i) for p, f, i in (damage or [])}
    counts = {}
    n_fate = {}
    for p in placements:
        if p.get("category") not in ("house", "building"):
            continue
        fp = footprint(res, p, "house")
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
        face, edge, lw, alpha = _COLOUR.get(t, "#7f8fa6"), "#111", 0.3, 1.0
        if damage is not None:
            fate, inten = fate_of.get(id(p), ("intact", 0.0))
            n_fate[fate] = n_fate.get(fate, 0) + 1
            face = _FATE_COLOUR[fate]
            if fate == "mesh":
                # Intensity as opacity, so the gradient the field applies is
                # readable per building and not only as a backdrop: a building
                # at the epicentre is solid, one at the edge is faint.
                edge, lw, alpha = "#f4e3c0", 0.5, 0.30 + 0.70 * inten
        ax.add_patch(Rectangle((p["x_m"] - w / 2, p["y_m"] - h / 2), w, h,
                               facecolor=face, edgecolor=edge, lw=lw,
                               alpha=alpha, zorder=4))

    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    legend = "   ".join(f"{k} {v}" for k, v in sorted(counts.items()))
    park = "  ".join(f"{k} {v}" for k, v in sorted(n_park.items()))
    sub = f"{len(layout.get('blocks', []))} blocks   " \
          f"{len(layout.get('road_corridors', []))} corridors   {legend}"
    if damage is not None:
        sub += ("\nfate: " + "  ".join(f"{k} {v}" for k, v in sorted(n_fate.items()))
                + "   debris: "
                + "  ".join(f"{k} {v}" for k, v in sorted(n_debris.items())))
    if park:
        sub += f"\npark: {park}"
    # `guessed` is the StubResolver's own bookkeeping. The real
    # `SizeResolver` measures every asset it can open and has no such
    # attribute — and when THAT is the resolver there is nothing to warn
    # about, because nothing was approximated.
    guessed = getattr(res, "guessed", ())
    if guessed:
        sub += f"   [{len(guessed)} assets un-measured]"
    ax.set_title(f"{title}\n{sub}", color="#ddd", fontsize=10)
    fig.patch.set_facecolor("#1b1b1b")
    fig.savefig(out_path, dpi=110, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"[plan] {out_path}")
    print(f"[plan] {sub}")


def footprint(res, p, category=None):
    """The metric footprint of one placement.

    `scene_generator.placement_footprint` is the rule; this is the local name
    for it. Bugs from restating it are documented there — and note that
    `StubResolver` ignores `scale` (correctly: its comment-scraped sizes are
    already metric), so this tool cannot catch a regression in it. The sim-side
    plan and `disaster_stage` can, and both use the same function.
    """
    import scene_generator as sg
    return sg.placement_footprint(res, p, category)


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
            fp = footprint(res, p, "house")
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
        "unmeasured": sorted(getattr(res, "guessed", ())),
    }
    with open(out_path, "w") as fh:
        json.dump(doc, fh, indent=1)
    print(f"[plan] {out_path}")
    return doc


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--json", action="store_true",
                    help="also write <out>.json — geometry, not pixels")
    ap.add_argument("--config", default="earthquake")
    # Defaults into the repo, not /tmp: a plan is meant to be looked at, and a
    # scratch path nobody can find is the same as not writing one. Gitignored.
    ap.add_argument("--out-dir", default="")
    ap.add_argument("--only", choices=("layout", "damage"),
                    help="write just one of the two maps")
    a = ap.parse_args()

    out_dir = a.out_dir or os.path.join(_SCENE_GEN, "_plans")
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.join(out_dir, os.path.basename(str(a.config)))

    # Two runs of the same `build_scene`, differing only in where they stop.
    # `build_scene` reseeds from the config, so the pristine map is the exact
    # scene the damage map then wrecks — not an approximation of it.
    if a.only != "damage":
        cfg, layout, pristine, res = build(a.config, stop_after="detail")
        draw(cfg, layout, pristine, res, stem + "_layout.png",
             title=f"{a.config} — layout (pristine)")
        if a.json:
            dump_json(cfg, layout, pristine, res, stem + "_layout.json")

    if a.only != "layout":
        cfg, layout, pristine, res = build(a.config, stop_after="detail")
        # Which USD each house had BEFORE the disaster stage, by index, so a
        # ruin swap is detectable afterwards.
        pristine_usd = {i: p.get("usd") for i, p in enumerate(pristine)
                        if p.get("category") in ("house", "building")}
        cfg, layout, placements, res = build(a.config, stop_after="disaster")
        dmg = classify(placements, pristine_usd)
        draw(cfg, layout, placements, res, stem + "_damage.png",
             title=f"{a.config} — damage (severity "
                   f"{_stage_get(cfg, 'severity', '?')})",
             damage=dmg, field=field_of(cfg, layout))


def _stage_get(cfg, key, default=None):
    import scene_generator as sg
    return sg._stage(cfg, "disaster").get(key, default)


if __name__ == "__main__":
    main()
