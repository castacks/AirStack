#!/usr/bin/env python3
"""layout_review_png.py — the 2D REVIEW SHEETS for a city layout, before any
Isaac build.

`plan_png.py` already draws a plan and already colours by model. This tool is
the REVIEW pass on top of it: the two questions a human actually asks of a
generated downtown before agreeing to spend a pod on it are

    1. "is the zoning right"      -- which district is where, and how big
    2. "does it look copy-pasted" -- is the same model standing next to itself

and neither is answerable from `plan_png`'s output alone. The typology view has
no LABELS, so a reviewer has to match colours against a legend block by block;
the model view has no notion of DISTANCE, so a reader has to spot two same-hue
rectangles by eye across a 1 km plate. Both are exactly the kind of judgement a
render is bad at and arithmetic is good at, so this tool measures them and draws
the answer rather than leaving it to be squinted at.

Everything here runs OFFLINE — CPU only, no Kit, no GPU, no Nucleus. It goes
through `plan_png.build`, which sets `measure_usds=False` and takes footprints
from the checked-in `config/asset_sets/*.yaml` comment scrape plus the measured
`_plans/*.json` files. That is the whole point: a 1 km layout can be reviewed,
rejected and re-rolled on a laptop, and only a layout that has already passed
this costs a pod.

THE THREE SHEETS
----------------
``districts``  Blocks filled and LABELLED with their typology, roads drawn,
               buildings outlined so density reads. Answers question 1.

``diversity``  Buildings filled by MODEL, with every same-model pair closer
               than ``--repeat-radius`` ringed and joined by a line. A cluster
               of one colour was already visible in `plan_png --by-model`; what
               is new is that the offending PAIRS are marked, so "some repeats
               are fine" can be judged against "these particular ones are 12 m
               apart". Answers question 2.

``damage``     The same plate greyed back, with the damaged buildings from a
               fire/quake/tornado manifest filled by damage level and the
               ignition points marked. Answers "is the damage where I wanted
               it, and did it land on a variety of buildings".

Each sheet carries its own measured stats in the title, so the PNG is
self-describing when it turns up in a review folder a week later.

USAGE
-----
    # undamaged city, both sheets, generated at 1 km directly
    python3 scene_gen/tools/layout_review_png.py \
        --config downtown_fire_1500 --fire-layout --seed 4 --region 1000 \
        --out-dir ~/layout_review --tag undamaged_1km_s4

    # with a damage overlay from a fire manifest
    python3 scene_gen/tools/layout_review_png.py \
        --config downtown_fire_1500 --fire-layout --seed 4 --region 1000 \
        --manifest scene_gen/_plans/fire_l1.json \
        --out-dir ~/layout_review --tag fire_L1

``--region`` overrides the preset's own ``region_m`` through the same
`spec_overrides` channel `plan_png`/`fire_city_dry_run` already use, so a
1.5 km preset can be reviewed at the 1 km it will actually ship at without
editing the preset. ``--fire-layout`` is required for any `disaster-type: fire`
preset for the reason `plan_png.build`'s docstring gives — "fire" is not a
compiled disaster type and `compile_spec` raises on it.
"""
import argparse
import collections
import itertools
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_HERE, _SCENE_GEN):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import plan_png  # noqa: E402

#: Typology fills. Deliberately NOT `plan_png`'s palette: these are picked so
#: the ladder reads as a ladder (low = warm/pale, tall = cold/dark), which is
#: what makes an over-towered plate obvious at a glance.
TYPOLOGY_COLOUR = {
    "rowhouse": "#c8a165",
    "lowrise": "#d9b48a",
    "brick_midrise": "#a8705a",
    "midrise": "#6f8fa8",
    "tower": "#41618a",
    "highrise": "#2b3d63",
    "park": "#4a6b45",
}
_UNKNOWN_TYPOLOGY = "#5a5a5a"

#: Damage-level fills, shared across disasters. Fire uses F0-F5c, quake and
#: tornado use their own ladders; anything unrecognised falls to the last
#: colour so an unknown level still SHOWS rather than silently vanishing.
DAMAGE_COLOUR = {
    "F0": "#ffe9a8", "F1": "#ffcf6b", "F2": "#ff9f40",
    "F3": "#f4692c", "F4": "#d33a1f", "F5": "#8f1d12", "F5c": "#5c0f0a",
}
_UNKNOWN_DAMAGE = "#ff00ff"


def _house(p):
    return p.get("category") in ("house", "building")


def _footprint(res, p):
    fp = res.get(p.get("usd", ""), "house")
    w, d = float(fp["sx"]), float(fp["sy"])
    # `res` reports the model's own footprint at scale 1; the packer's yaw is
    # axis-aligned in every downtown preset, so a 90/270 turn swaps the axes.
    # Anything else is left unswapped rather than guessed at -- a diagonal
    # building would need a real OBB and there are none in these scenes.
    yaw = float(p.get("yaw_deg", 0.0)) % 180.0
    if 45.0 <= yaw < 135.0:
        w, d = d, w
    return w, d


def block_typologies(layout):
    """`[(rect, typology_name), ...]`, tolerating both key shapes.

    `layout["_typology_of"]` is keyed by the block TUPLE, but `layout["blocks"]`
    holds lists after a JSON round-trip, so a plain lookup silently misses on a
    reloaded layout and every block comes back `None`. Both are tried.
    """
    typ_of = layout.get("_typology_of") or {}
    out = []
    for b in layout.get("blocks", []):
        rect = tuple(float(v) for v in b)
        name = typ_of.get(rect)
        if name is None:
            try:
                name = typ_of.get(b)
            except TypeError:
                name = None
        out.append((rect, name))
    return out


def repeat_pairs(houses, res, radius_m):
    """Every pair of same-model buildings whose CENTRES are within *radius_m*.

    Centre distance, not footprint gap: two 40 m department stores 45 m apart
    centre-to-centre are touching, and the thing a viewer reacts to is "that
    building again, right there", which tracks centre spacing closely enough
    at the scale that matters. Returns `[(dist, i, j), ...]` sorted nearest
    first, so the worst offenders draw last and on top.
    """
    by_model = collections.defaultdict(list)
    for i, p in enumerate(houses):
        by_model[os.path.basename(str(p.get("usd", "")))].append(i)
    r2 = float(radius_m) ** 2
    out = []
    for _model, idxs in by_model.items():
        for a, b in itertools.combinations(idxs, 2):
            pa, pb = houses[a], houses[b]
            d2 = (pa["x_m"] - pb["x_m"]) ** 2 + (pa["y_m"] - pb["y_m"]) ** 2
            if d2 <= r2:
                out.append((math.sqrt(d2), a, b))
    out.sort(reverse=True)
    return out


def diversity_stats(houses, res, radius_m):
    models = collections.Counter(
        os.path.basename(str(p.get("usd", ""))) for p in houses)
    pairs = repeat_pairs(houses, res, radius_m)
    involved = set()
    for _d, a, b in pairs:
        involved.add(a)
        involved.add(b)
    n = max(1, len(houses))
    return {
        "houses": len(houses),
        "models": len(models),
        "copies_per_model": len(houses) / max(1, len(models)),
        "top_model": models.most_common(1)[0] if models else ("-", 0),
        "top_share": (models.most_common(1)[0][1] / n) if models else 0.0,
        "repeat_pairs": len(pairs),
        "repeat_involved": len(involved),
        "repeat_share": len(involved) / n,
        "histogram": models,
        "pairs": pairs,
    }


def typology_stats(blocks):
    area = collections.Counter()
    count = collections.Counter()
    for rect, name in blocks:
        key = name or "unzoned"
        area[key] += (rect[2] - rect[0]) * (rect[3] - rect[1])
        count[key] += 1
    total = sum(area.values()) or 1.0
    tall = area["highrise"] + area["tower"]
    return {"area": area, "count": count, "total": total,
            "tall_share": tall / total}


def _new_axes(layout, size=15.0):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    x0, y0, x1, y1 = layout["region"]
    fig, ax = plt.subplots(figsize=(size, size))
    ax.set_facecolor("#242424")
    fig.patch.set_facecolor("#141414")
    ax.set_xlim(x0 - 20, x1 + 20)
    ax.set_ylim(y0 - 20, y1 + 20)
    ax.set_aspect("equal")
    ax.tick_params(colors="#888888", labelsize=8)
    for s in ax.spines.values():
        s.set_color("#444444")
    return fig, ax


def _draw_roads(ax, layout, z=2):
    from matplotlib.patches import Rectangle
    for c in layout.get("road_corridors", []):
        ax.add_patch(Rectangle(
            (c["x0"], c["y0"]), c["x1"] - c["x0"], c["y1"] - c["y0"],
            facecolor="#161616", edgecolor="none", zorder=z))


def _title(ax, text, sub=""):
    """Headline plus a monospace stats block, as ONE title.

    Drawing the stats as a separate `ax.text` above the axes collides with the
    title whenever the stats run to more than one line — which they always do —
    so both live in the title string and matplotlib does the stacking.
    """
    ax.set_title(text, color="#f0f0f0", fontsize=15, pad=8, loc="left")
    if sub:
        ax.text(0.0, 1.012, sub, transform=ax.transAxes, color="#b0b0b0",
                fontsize=9.5, va="bottom", ha="left", family="monospace",
                linespacing=1.5)
        # Push the headline above the stats block by the height the stats
        # actually occupy, in title-pad points: ~13 pt per line at 9.5 pt with
        # 1.5 line spacing, plus a little air.
        ax.set_title(text, color="#f0f0f0", fontsize=15, loc="left",
                     pad=14 + 13 * (sub.count("\n") + 1))


def _save(fig, out_path):
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".",
                exist_ok=True)
    fig.savefig(out_path, dpi=110, facecolor=fig.get_facecolor(),
                bbox_inches="tight")
    import matplotlib.pyplot as plt
    plt.close(fig)
    print("[review] %s" % out_path)
    return out_path


def draw_districts(cfg, layout, placements, res, out_path, title=""):
    """Sheet 1 — zoning, with every block LABELLED."""
    from matplotlib.patches import Rectangle
    blocks = block_typologies(layout)
    houses = [p for p in placements if _house(p)]
    st = typology_stats(blocks)
    fig, ax = _new_axes(layout)

    for i, (rect, name) in enumerate(blocks):
        colour = TYPOLOGY_COLOUR.get(name, _UNKNOWN_TYPOLOGY)
        w, h = rect[2] - rect[0], rect[3] - rect[1]
        ax.add_patch(Rectangle((rect[0], rect[1]), w, h, facecolor=colour,
                               edgecolor="#101010", lw=0.8, alpha=0.92,
                               zorder=1))
    _draw_roads(ax, layout)

    # Buildings as outlines only: the block fill is carrying the zoning, so a
    # solid building would fight it. An outline still shows density and the
    # street wall, which is what tells you a block is full or half empty.
    for p in houses:
        w, d = _footprint(res, p)
        ax.add_patch(Rectangle((p["x_m"] - w / 2, p["y_m"] - d / 2), w, d,
                               facecolor="none", edgecolor="#12121280",
                               lw=0.45, zorder=3))

    # Labels last so nothing covers them. A block too small for its own name
    # gets the short form; below that it gets nothing rather than a smear.
    for i, (rect, name) in enumerate(blocks):
        w, h = rect[2] - rect[0], rect[3] - rect[1]
        cx, cy = (rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0
        n_in = sum(1 for p in houses
                   if rect[0] <= p["x_m"] <= rect[2]
                   and rect[1] <= p["y_m"] <= rect[3])
        label = "%s\n#%d  %.0fx%.0f m  %d bld" % (name or "unzoned", i, w, h,
                                                  n_in)
        short = "%s #%d" % ((name or "?")[:8], i)
        if min(w, h) >= 55:
            txt, fs = label, 7.5
        elif min(w, h) >= 30:
            txt, fs = short, 6.5
        else:
            continue
        ax.text(cx, cy, txt, ha="center", va="center", fontsize=fs,
                color="#0d0d0d", zorder=6, linespacing=1.25,
                family="monospace",
                bbox=dict(boxstyle="round,pad=0.18", fc="#ffffffbb",
                          ec="none"))

    legend = "  ".join(
        "%s %d(%.0f%%)" % (n, st["count"][n], 100 * st["area"][n] / st["total"])
        for n, _ in st["area"].most_common())
    _title(ax, title or "districts",
           "blocks %d | buildings %d | highrise+tower %.1f%% of block area\n%s"
           % (len(blocks), len(houses), 100 * st["tall_share"], legend))
    return _save(fig, out_path)


def draw_diversity(cfg, layout, placements, res, out_path, title="",
                   radius_m=60.0, top_n=18):
    """Sheet 2 — model diversity, with close same-model pairs called out."""
    from matplotlib.patches import Rectangle, Circle
    blocks = block_typologies(layout)
    houses = [p for p in placements if _house(p)]
    dv = diversity_stats(houses, res, radius_m)
    fig, ax = _new_axes(layout)

    for rect, name in blocks:
        ax.add_patch(Rectangle((rect[0], rect[1]), rect[2] - rect[0],
                               rect[3] - rect[1], facecolor="#333333",
                               edgecolor="none", zorder=1))
    _draw_roads(ax, layout)

    for p in houses:
        w, d = _footprint(res, p)
        ax.add_patch(Rectangle(
            (p["x_m"] - w / 2, p["y_m"] - d / 2), w, d,
            facecolor=plan_png._model_colour(p.get("usd", "")),
            edgecolor="#00000055", lw=0.3, zorder=3))

    # THE OFFENDING PAIRS. Drawn as a line between the two centres plus a ring
    # on each, so a repeat reads even where the two footprints are small. The
    # nearest pairs draw last (see `repeat_pairs`' sort) and so sit on top.
    for dist, a, b in dv["pairs"]:
        pa, pb = houses[a], houses[b]
        ax.plot([pa["x_m"], pb["x_m"]], [pa["y_m"], pb["y_m"]],
                color="#ff2d55", lw=1.1, alpha=0.85, zorder=7)
        for p in (pa, pb):
            w, d = _footprint(res, p)
            ax.add_patch(Circle((p["x_m"], p["y_m"]), max(w, d) * 0.62,
                                facecolor="none", edgecolor="#ff2d55",
                                lw=1.0, alpha=0.85, zorder=7))

    top = "  ".join("%s x%d" % (m[:22], c)
                    for m, c in dv["histogram"].most_common(6))
    _title(ax, title or "building diversity",
           ("buildings %d | distinct models %d | copies/model %.1f | "
            "top model %s = %.1f%%\n"
            "same-model pairs within %.0f m: %d  (%d buildings, %.1f%% of the "
            "city)   red = one such pair\n%s")
           % (dv["houses"], dv["models"], dv["copies_per_model"],
              dv["top_model"][0][:26], 100 * dv["top_share"], radius_m,
              dv["repeat_pairs"], dv["repeat_involved"],
              100 * dv["repeat_share"], top))

    # A real legend for the top models -- the colour hash is stable but not
    # memorable, and "which model is the pale blue one" is the first question
    # a reviewer asks once they see a cluster.
    from matplotlib.patches import Patch
    handles = [Patch(facecolor=plan_png._model_colour(m), edgecolor="#00000055",
                     label="%s  x%d" % (m[:30], c))
               for m, c in dv["histogram"].most_common(top_n)]
    leg = ax.legend(handles=handles, loc="upper left",
                    bbox_to_anchor=(1.01, 1.0), fontsize=7.5,
                    facecolor="#1c1c1c", edgecolor="#444444", labelcolor="#dddddd",
                    title="models (top %d of %d)" % (min(top_n, dv["models"]),
                                                     dv["models"]))
    leg.get_title().set_color("#dddddd")
    leg.get_title().set_fontsize(8)
    return _save(fig, out_path)


def draw_block_shapes(cfg, layout, placements, res, out_path, title=""):
    """Sheet 4 — the BLOCK SHAPE GALLERY, and the map coloured by block size.

    The districts sheet answers "which district is where"; it cannot answer
    "are these blocks all the same shape", because the blocks are scattered
    across a 1 km plate and a reader cannot hold three rectangles 400 m apart
    in their head and compare them. That question is what actually decides
    whether a district reads as copy-pasted: identical CONTAINERS holding a
    similar count of similar-grain buildings look the same from above however
    varied the models inside are.

    So this draws every block's outline again, to scale, stacked by typology
    with a common baseline — the comparison a reviewer would otherwise have
    to do by eye across the map — and repeats the map with block fill keyed
    to LONG-SIDE LENGTH rather than typology. A district whose blocks are all
    one size comes out as one flat colour and one repeated rectangle.

    MEASURED on `downtown_urban_1000` seed 8, which is what prompted this:
    tower blocks span 235-245 m on their long side (CV 0.02) and highrise
    149-235 m (CV 0.14), against rowhouse's 97-245 m (CV 0.45). The tall
    typologies pile up against `layout.max_block_m` and come out uniform.
    """
    import statistics
    from matplotlib.patches import Rectangle
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    blocks = [(r, n) for r, n in block_typologies(layout)
              if n and n != "park"]
    by_typ = collections.defaultdict(list)
    for rect, name in blocks:
        by_typ[name].append(rect)
    order = sorted(by_typ, key=lambda n: -len(by_typ[n]))

    fig = plt.figure(figsize=(17, 9))
    fig.patch.set_facecolor("#141414")
    gsL = fig.add_axes([0.045, 0.06, 0.44, 0.84])
    gsR = fig.add_axes([0.545, 0.06, 0.42, 0.84])

    # ---- left: the gallery, one row per typology, blocks drawn to scale ----
    gsL.set_facecolor("#1c1c1c")
    y = 0.0
    row_gap = 30.0
    max_w = 0.0
    for name in order:
        rects = sorted(by_typ[name], key=lambda r: -(r[2] - r[0]))
        h_row = max((r[3] - r[1]) for r in rects)
        x = 0.0
        for r in rects:
            w, h = r[2] - r[0], r[3] - r[1]
            gsL.add_patch(Rectangle((x, y), w, h,
                                    facecolor=TYPOLOGY_COLOUR.get(name,
                                                                  _UNKNOWN_TYPOLOGY),
                                    edgecolor="#0d0d0d", lw=1.0, alpha=0.95))
            gsL.text(x + w / 2, y + h / 2, "%.0f×%.0f" % (w, h),
                     ha="center", va="center", fontsize=6.5,
                     color="#101010", family="monospace")
            x += w + 12.0
        longs = [max(r[2] - r[0], r[3] - r[1]) for r in rects]
        cv = (statistics.pstdev(longs) / statistics.mean(longs)) if len(longs) > 1 else 0.0
        gsL.text(-14.0, y + h_row / 2,
                 "%s\nn=%d  CV %.2f" % (name, len(rects), cv),
                 ha="right", va="center", fontsize=8.5, color="#e0e0e0",
                 family="monospace", linespacing=1.4)
        max_w = max(max_w, x)
        y += h_row + row_gap
    gsL.set_xlim(-190, max_w + 20)
    gsL.set_ylim(-20, y)
    gsL.set_aspect("equal")
    gsL.axis("off")
    gsL.set_title("every block, to scale, by district\n"
                  "a row of identical rectangles IS the defect",
                  color="#f0f0f0", fontsize=12, loc="left", pad=12)

    # ---- right: the map, coloured by long side ----
    x0, y0, x1, y1 = layout["region"]
    gsR.set_facecolor("#242424")
    longs_all = [max(r[2] - r[0], r[3] - r[1]) for r, _n in blocks]
    lo, hi = min(longs_all), max(longs_all)
    cmap = plt.get_cmap("viridis")
    for rect, _name in blocks:
        L = max(rect[2] - rect[0], rect[3] - rect[1])
        t = (L - lo) / (hi - lo) if hi > lo else 0.5
        gsR.add_patch(Rectangle((rect[0], rect[1]), rect[2] - rect[0],
                                rect[3] - rect[1], facecolor=cmap(t),
                                edgecolor="#101010", lw=0.8))
        gsR.text((rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2,
                 "%.0f" % L, ha="center", va="center", fontsize=6.5,
                 color="#ffffff" if t < 0.6 else "#101010",
                 family="monospace")
    gsR.set_xlim(x0 - 20, x1 + 20)
    gsR.set_ylim(y0 - 20, y1 + 20)
    gsR.set_aspect("equal")
    gsR.tick_params(colors="#888888", labelsize=8)
    for s in gsR.spines.values():
        s.set_color("#444444")
    sm = plt.cm.ScalarMappable(cmap=cmap,
                               norm=plt.Normalize(vmin=lo, vmax=hi))
    cb = fig.colorbar(sm, ax=gsR, fraction=0.035, pad=0.02)
    cb.set_label("block long side (m)", color="#dddddd", fontsize=9)
    cb.ax.tick_params(colors="#bbbbbb", labelsize=8)
    gsR.set_title("same map, coloured by block long side\n"
                  "one flat colour in a district = one block size",
                  color="#f0f0f0", fontsize=12, loc="left", pad=12)

    fig.suptitle(title or "block shapes", color="#f0f0f0", fontsize=15,
                 x=0.045, ha="left", y=0.985)
    return _save(fig, out_path)


def load_manifest(path):
    """A fire/quake/tornado damage manifest -> `[{x, y, level, usd}, ...]`.

    Accepts the two shapes that actually exist in `_plans/`: the union tool's
    `{"records": [...]}` and a bare list. Each record is read for `x`/`y` (or
    `x_orig`/`y_orig` -- see `urban_fire_city.burnable`'s note on the cropped
    frame; the ORIGINAL coordinate is the one that matches an uncropped
    layout) and a level under `level`, `fire_level` or `damage_level`.
    """
    with open(path) as fh:
        doc = json.load(fh)
    records = doc.get("records") if isinstance(doc, dict) else doc
    out = []
    for r in records or []:
        if not isinstance(r, dict):
            continue
        x = r.get("x_orig", r.get("x"))
        y = r.get("y_orig", r.get("y"))
        if x is None or y is None:
            continue
        lvl = (r.get("level") or r.get("fire_level")
               or r.get("damage_level") or "?")
        out.append({"x": float(x), "y": float(y), "level": str(lvl),
                    "usd": os.path.basename(str(r.get("usd", ""))),
                    "W": r.get("W"), "D": r.get("D")})
    return out


def draw_damage(cfg, layout, placements, res, damaged, out_path, title="",
                origins=None, match_m=6.0):
    """Sheet 3 — where the damage landed, and on what."""
    from matplotlib.patches import Rectangle, Circle
    blocks = block_typologies(layout)
    houses = [p for p in placements if _house(p)]
    fig, ax = _new_axes(layout)

    for rect, name in blocks:
        ax.add_patch(Rectangle((rect[0], rect[1]), rect[2] - rect[0],
                               rect[3] - rect[1], facecolor="#3a3a3a",
                               edgecolor="none", zorder=1))
    _draw_roads(ax, layout)

    # Undamaged stock, greyed right back so the damage is the only thing with
    # colour on the sheet.
    for p in houses:
        w, d = _footprint(res, p)
        ax.add_patch(Rectangle((p["x_m"] - w / 2, p["y_m"] - d / 2), w, d,
                               facecolor="#585858", edgecolor="#00000044",
                               lw=0.3, zorder=3))

    # Each manifest record is matched back to its layout building by nearest
    # centre, so the drawn footprint is the REAL one rather than the record's
    # possibly-absent W/D. A record with no building within `match_m` is drawn
    # at its own coordinate with a dashed ring -- that mismatch is itself a
    # finding (it is exactly what a frame shift looks like) and must not be
    # silently dropped.
    levels = collections.Counter()
    unmatched = 0
    for rec in damaged:
        best, bd = None, 1e18
        for p in houses:
            d2 = (p["x_m"] - rec["x"]) ** 2 + (p["y_m"] - rec["y"]) ** 2
            if d2 < bd:
                bd, best = d2, p
        colour = DAMAGE_COLOUR.get(rec["level"], _UNKNOWN_DAMAGE)
        levels[rec["level"]] += 1
        if best is not None and math.sqrt(bd) <= match_m:
            w, d = _footprint(res, best)
            ax.add_patch(Rectangle((best["x_m"] - w / 2, best["y_m"] - d / 2),
                                   w, d, facecolor=colour, edgecolor="#000000",
                                   lw=0.6, zorder=6))
        else:
            unmatched += 1
            ax.add_patch(Circle((rec["x"], rec["y"]), 14.0, facecolor="none",
                                edgecolor=colour, lw=1.6, ls="--", zorder=6))

    for o in origins or []:
        ax.plot([o[0]], [o[1]], marker="*", markersize=20, color="#ffe14d",
                markeredgecolor="#000000", markeredgewidth=0.7, zorder=9)

    ladder = "  ".join("%s %d" % (k, levels[k])
                       for k in sorted(levels, key=lambda s: (len(s), s)))
    warn = ("   !! %d record(s) matched NO building within %.0f m — frame "
            "mismatch" % (unmatched, match_m)) if unmatched else ""
    _title(ax, title or "damage",
           "damaged %d of %d buildings (%.1f%%)   %s%s"
           % (len(damaged), len(houses),
              100 * len(damaged) / max(1, len(houses)), ladder, warn))

    from matplotlib.patches import Patch
    handles = [Patch(facecolor=DAMAGE_COLOUR.get(k, _UNKNOWN_DAMAGE),
                     edgecolor="#000000", label="%s  x%d" % (k, levels[k]))
               for k in sorted(levels, key=lambda s: (len(s), s))]
    if handles:
        leg = ax.legend(handles=handles, loc="upper left",
                        bbox_to_anchor=(1.01, 1.0), fontsize=8,
                        facecolor="#1c1c1c", edgecolor="#444444",
                        labelcolor="#dddddd", title="damage level")
        leg.get_title().set_color("#dddddd")
    return _save(fig, out_path)


def build(config, seed=None, region=None, fire_layout=False):
    overrides = {}
    if fire_layout:
        overrides["disaster-type"] = "none"
    if region:
        overrides["region_m"] = [float(region), float(region)]
    return plan_png.build(config, seed=seed,
                          spec_overrides=overrides or None)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="downtown_fire_1500")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--region", type=float, default=None,
                    help="override the preset's region_m (metres, square)")
    ap.add_argument("--fire-layout", action="store_true",
                    help="required for any `disaster-type: fire` preset")
    ap.add_argument("--manifest", default=None,
                    help="damage manifest JSON -> also draw the damage sheet")
    ap.add_argument("--origins", default=None,
                    help="'x,y;x,y' ignition points to star on the damage sheet")
    ap.add_argument("--repeat-radius", type=float, default=60.0)
    ap.add_argument("--out-dir", default=os.path.join(_SCENE_GEN, "_plans",
                                                      "layout_review"))
    ap.add_argument("--tag", default=None,
                    help="filename stem; defaults to <config>_s<seed>_<region>")
    ap.add_argument("--shapes", action="store_true",
                    help="also draw the BLOCK SHAPE sheet -- every block to "
                         "scale by district, plus the map keyed to block size")
    ap.add_argument("--stats-json", action="store_true",
                    help="also write <tag>_stats.json")
    a = ap.parse_args()

    cfg, layout, placements, res = build(a.config, seed=a.seed,
                                         region=a.region,
                                         fire_layout=a.fire_layout)
    x0, y0, x1, y1 = layout["region"]
    tag = a.tag or "%s_s%s_%.0fm" % (a.config, a.seed if a.seed is not None
                                     else "def", x1 - x0)
    os.makedirs(a.out_dir, exist_ok=True)
    head = "%s  seed %s  %.0f x %.0f m" % (
        a.config, a.seed if a.seed is not None else cfg.get("seed"),
        x1 - x0, y1 - y0)

    outs = []
    outs.append(draw_districts(cfg, layout, placements, res,
                               os.path.join(a.out_dir, tag + "_districts.png"),
                               title="DISTRICTS — " + head))
    outs.append(draw_diversity(cfg, layout, placements, res,
                               os.path.join(a.out_dir, tag + "_diversity.png"),
                               title="BUILDING DIVERSITY — " + head,
                               radius_m=a.repeat_radius))
    if a.shapes:
        outs.append(draw_block_shapes(
            cfg, layout, placements, res,
            os.path.join(a.out_dir, tag + "_blockshapes.png"),
            title="BLOCK SHAPES — " + head))
    if a.manifest:
        origins = []
        if a.origins:
            for part in a.origins.split(";"):
                if part.strip():
                    xs, ys = part.split(",")
                    origins.append((float(xs), float(ys)))
        outs.append(draw_damage(cfg, layout, placements, res,
                                load_manifest(a.manifest),
                                os.path.join(a.out_dir, tag + "_damage.png"),
                                title="DAMAGE — " + head, origins=origins))

    if a.stats_json:
        houses = [p for p in placements if _house(p)]
        dv = diversity_stats(houses, res, a.repeat_radius)
        st = typology_stats(block_typologies(layout))
        doc = {
            "config": a.config, "seed": a.seed, "region_m": [x1 - x0, y1 - y0],
            "blocks": len(layout.get("blocks", [])), "houses": len(houses),
            "models": dv["models"],
            "copies_per_model": round(dv["copies_per_model"], 2),
            "top_model": dv["top_model"][0],
            "top_model_share": round(dv["top_share"], 4),
            "repeat_pairs": dv["repeat_pairs"],
            "repeat_share": round(dv["repeat_share"], 4),
            "repeat_radius_m": a.repeat_radius,
            "tall_share_of_block_area": round(st["tall_share"], 4),
            "blocks_by_typology": dict(st["count"]),
        }
        path = os.path.join(a.out_dir, tag + "_stats.json")
        with open(path, "w") as fh:
            json.dump(doc, fh, indent=1)
        print("[review] %s" % path)
    return outs


if __name__ == "__main__":
    main()
