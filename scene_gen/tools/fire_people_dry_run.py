#!/usr/bin/env python3
"""
fire_people_dry_run.py — the 2-D GATE for `disaster/fire_people.py`.

HOST-SIDE. No Isaac, no docker, no Nucleus, no `pxr`. It runs the real
planner on the real inputs (the FC city placements dump + the fire manifest,
plus the bake sidecars when they exist), renders a top-down PNG to
`scene_gen/_plans/fire_people_<seed>.png`, prints a placement census, and
prints the rule checks.

WHY THIS EXISTS AT ALL, and it is the same argument
`place-people-in-scenes` makes: **RUN THE 2-D DRY RUN BEFORE PAYING FOR AN
ISAAC BUILD.** `fire_people.py` touches no stage, so the whole composition —
ground classification, standoff geometry, the fire-side cones, the window and
roof solves, the burial band — runs host-side in seconds. Every failure in a
people pass is silent: the placement succeeds, the figure is counted, the
ground truth is written, and nothing is visible. A picture and a census are
what catch that before ten minutes of Kit does not.

WHAT THIS GATE CAN AND CANNOT DECIDE
------------------------------------
It decides the x/y classes completely: who is inside a footprint, who is in a
debris apron, whether the standoff and the upwind rule actually produced an
asymmetric crowd, whether groups are groups, which surface everyone is on.

It CANNOT decide the three-dimensional ones. A figure leaning out of a window
0.17 m past the facade, or lying with 38 % of itself under a windrow, is a
render question and the tornado skill is unambiguous that the ONLY way to
answer it is a close bench shot. Those records carry `needs_bench` and this
tool reports how many — it never clears them. The PNG plots them (with their
z and storey annotated) so their PLAN position can be judged, which is a
different and smaller claim.

USAGE

    cd scene_gen && python3 tools/fire_people_dry_run.py \\
        --dump _plans/fc_dump_500.json \\
        --manifest _plans/fire_city_500b.json \\
        --seed 7

    # ...and, once a bake has run:
    #   --sidecar-dir <dir with the bakes' .json sidecars>

`--from-preset <preset>` builds the REAL city host-side instead of reading a
dump: it calls `fire_city_dry_run.build_layout`, which runs the actual
`generate_scene.generate_scene_on_stage` pipeline in memory with the same
localised-URL / patched-`SizeResolver` trick that tool documents, and turns
the placements it produced into the same dump shape. It also hands the
planner the REAL `city_layout` — actual BSP blocks, actual road corridors,
actual `sidewalk_rects` — instead of the derivation, so both ground paths get
exercised.

    uv run --python 3.13 --with usd-core --with numpy --with pyyaml \
        --with matplotlib python tools/fire_people_dry_run.py \
        --from-preset downtown_fire_500 --manifest _plans/fire_city_4.json

**THIS IS THE HOST-SIDE BUILD, AND IT IS NOT THE CITY KIT BUILDS.** That is
the whole reason `FC_DUMP` exists (see `fire_city_dry_run.py`'s own account of
the 2026-08-30 manifest/city mismatch): the host-side resolver substitutes
cached GAC/DTC footprints for assets it cannot reach, so the packer can make
different spacing decisions than Kit's. Use it for a REAL-GEOMETRY gate while
the dump is being produced; re-run on `--dump` before anything is authored.

`--synth` builds a SYNTHETIC dump and manifest instead — a 500 m plate, a
4 x 4 block grid with real street widths, sixteen burning buildings on the
manifest's own level histogram. It exists so the planner can be exercised
and unit-tested before the city launcher has written a dump, and it prints a
banner on every run saying so. **A synthetic run is not evidence about the
real city**: its road network is a regular grid and the real one is a BSP
subdivision with a road hierarchy.
"""
import argparse
import json
import math
import os
import random
import sys

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_TOOLS_DIR)
if _SCENE_GEN_DIR not in sys.path:
    sys.path.insert(0, _SCENE_GEN_DIR)

from disaster import fire_people as fp        # noqa: E402


# ---------------------------------------------------------------------------
# The wind, off the raw preset
# ---------------------------------------------------------------------------
def preset_heading_deg(preset, default=45.0):
    """`heading_deg` off the RAW preset yaml.

    Read the same way `fire_city_dry_run.py` and
    `urban_fire_city_launch_script.preset_fire_block` read it — directly,
    never through `compile_spec`, because `compile_disaster.DISASTERS` has no
    `"fire"` entry and compiling a `disaster-type: fire` preset RAISES.

    Falls back to a line-scan when `pyyaml` is not installed, so this tool
    keeps its "no dependencies but matplotlib" property.
    """
    if not preset:
        return default, "default (no preset name)"
    path = os.path.join(_SCENE_GEN_DIR, "config", "presets",
                        "{0}.yaml".format(preset))
    if not os.path.isfile(path):
        return default, "default ({0} not found)".format(path)
    try:
        import yaml
        with open(path) as fh:
            raw = yaml.safe_load(fh) or {}
        if raw.get("heading_deg") is not None:
            return float(raw["heading_deg"]), os.path.basename(path)
    except Exception:
        pass
    try:
        with open(path) as fh:
            for line in fh:
                s = line.strip()
                if s.startswith("heading_deg:"):
                    return (float(s.split(":", 1)[1].split("#")[0].strip()),
                            os.path.basename(path) + " (line scan)")
    except OSError:
        pass
    return default, "default (no heading_deg in {0})".format(
        os.path.basename(path))


# ---------------------------------------------------------------------------
# The REAL city, built host-side — see the module docstring for the caveat
# ---------------------------------------------------------------------------
def preset_inputs(preset, seed=None):
    """`(dump, layout, note)` from the REAL layout pipeline.

    Reuses `fire_city_dry_run`'s `build_layout` (and its `_measure_wdh` /
    `_gac_dtc_cache`) verbatim rather than reimplementing the localised-URL
    and patched-resolver machinery, so this mode cannot drift from the tool
    that produces the manifest it is paired with.

    The dump it returns is in the real `fire_city_placements_dump.v1` shape
    with every house's ORIGINAL index into the full placement list preserved,
    for the same reason `dump_city_placements` preserves it.
    """
    import fire_city_dry_run as fcd
    from disaster import gac_fire as gf

    config, layout, placements, resolver = fcd.build_layout(preset, seed=seed)
    cache = fcd._gac_dtc_cache()
    houses = []
    for i, p in enumerate(placements):
        if p.get("category") != "house":
            continue
        W, D, H = fcd._measure_wdh(p.get("usd"), p, resolver, cache, gf)
        houses.append({
            "i": i, "cell": p.get("prim_path"), "usd": p.get("usd"),
            "x_m": float(p.get("x_m", 0.0)), "y_m": float(p.get("y_m", 0.0)),
            "z_m": float(p.get("z_m", 0.0)),
            "yaw_deg": float(p.get("yaw_deg", 0.0)),
            "scale": float(p.get("scale", 1.0)), "category": "house",
            "axis_up": p.get("axis_up", "Z"),
            "W": float(W), "D": float(D), "H": float(H)})
    rm = (config.get("layout", {}) or {}).get("region_m") or [500.0, 500.0]
    dump = {"schema": "fire_city_placements_dump.v1", "preset": preset,
            "seed": int(config.get("seed", seed or 0)),
            "region_m": [float(v) for v in rm],
            "n_placements_total": len(placements), "placements": houses,
            "typology": {"blocks": [
                {"rect": [float(v) for v in rect], "name": name}
                for rect, name in (layout.get("_typology_of") or {}).items()]}}
    note = ("REAL host-side layout ({0} house placements of {1}, {2} blocks, "
            "{3} road corridors) — NOT the Kit build; re-run on FC_DUMP "
            "before authoring".format(len(houses), len(placements),
                                      len(layout.get("blocks") or []),
                                      len(layout.get("road_corridors") or [])))
    return dump, layout, note


# ---------------------------------------------------------------------------
# The synthetic fixture — a development stand-in, never evidence
# ---------------------------------------------------------------------------
def synth_inputs(seed=4, region_m=500.0, blocks_per_side=4, street_m=26.0):
    """`(dump, manifest)` in the real schemas.

    A regular `blocks_per_side` grid with `street_m` corridors between the
    blocks, four buildings per block, and a burning set drawn along a
    diagonal so the spread reads as a wave. The level histogram is
    `fire_city_4.json`'s: one F5c origin, then F4 / F3 / F1.

    Deliberately regular, and that is the limitation: the real city is a BSP
    subdivision with a road hierarchy and blocks of several sizes, so this
    fixture can prove the planner's ARITHMETIC and cannot prove anything
    about how the crowd sits in the real street network.
    """
    rng = random.Random(seed)
    half = region_m / 2.0
    pitch = region_m / blocks_per_side
    blk = pitch - street_m

    blocks, placements = [], []
    i = 0
    for bx in range(blocks_per_side):
        for by in range(blocks_per_side):
            cx = -half + pitch * (bx + 0.5)
            cy = -half + pitch * (by + 0.5)
            rect = [cx - blk / 2.0, cy - blk / 2.0,
                    cx + blk / 2.0, cy + blk / 2.0]
            typ = "midrise" if (bx + by) % 3 else "brick_midrise"
            if bx in (0, blocks_per_side - 1) and by in (0,
                                                         blocks_per_side - 1):
                typ = "tower"
            blocks.append({"rect": rect, "name": typ})
            for qx in (-1, 1):
                for qy in (-1, 1):
                    W = rng.uniform(24.0, 36.0)
                    D = rng.uniform(22.0, 40.0)
                    H = rng.uniform(18.0, 46.0)
                    x = cx + qx * (blk / 4.0)
                    y = cy + qy * (blk / 4.0)
                    i += 3        # leave gaps: real dumps skip non-houses
                    placements.append({
                        "i": i, "cell": "/World/stage/generated/house_%d" % i,
                        "usd": "synthetic/Building_%d.usd" % (i % 12),
                        "x_m": x, "y_m": y, "z_m": 0.0,
                        "yaw_deg": float(rng.choice([0, 90, 180, 270])),
                        "scale": 1.0, "category": "house", "axis_up": "Z",
                        "W": W, "D": D, "H": H})
    dump = {"schema": "fire_city_placements_dump.v1",
            "preset": "downtown_fire_500", "seed": seed,
            "region_m": [region_m, region_m],
            "n_placements_total": i + 3, "placements": placements,
            "typology": {"blocks": blocks}}

    # The burning set: the buildings nearest a SW->NE diagonal, in blocks the
    # no-fire district rule would allow (never `tower`).
    def typ_at(x, y):
        for b in blocks:
            r = b["rect"]
            if r[0] <= x <= r[2] and r[1] <= y <= r[3]:
                return b["name"]
        return None

    cands = [p for p in placements
             if typ_at(p["x_m"], p["y_m"]) in ("midrise", "brick_midrise")]
    cands.sort(key=lambda p: abs(p["y_m"] - p["x_m"]))
    chosen = cands[:16]
    levels = (["F5c"] + ["F4"] * 6 + ["F3"] * 7 + ["F1"] * 2)
    recs = []
    for k, p in enumerate(chosen):
        n_st = max(2, int(round(p["H"] / 3.3)))
        lvl = levels[k]
        lo, hi = fp.BAND[lvl]
        origin = rng.randrange(0, max(1, n_st - 1))
        sides = tuple(rng.sample(["N", "E", "S", "W"],
                                 1 if lvl in ("F1", "F2") else 2))
        recs.append({
            "usd": p["usd"], "x": p["x_m"], "y": p["y_m"],
            "yaw_deg": p["yaw_deg"], "z": 0.0, "kind": "synthetic",
            "asset": os.path.basename(p["usd"]).split(".")[0],
            "style": None, "typology": typ_at(p["x_m"], p["y_m"]),
            "W": p["W"], "D": p["D"], "H": p["H"], "cell": p["cell"],
            "i": p["i"], "level": lvl, "origin": origin, "sides": list(sides),
            "t_ignite_s": 600.0 * k, "age_s": 10080.0 - 600.0 * k,
            "via": None, "how": "radiation", "seed": 100 + k,
            "btype": "urm" if p["H"] <= 25.0 else "rc",
            "entry_side": sides[0], "origin_frac": 0.45,
            "n_storeys": n_st})
    manifest = {"seed": seed, "preset": "downtown_fire_500", "n": 16,
                "n_achieved": 16, "origin": recs[0]["i"], "epoch_s": 10080.0,
                "records": recs, "refused": []}
    return dump, manifest


# ---------------------------------------------------------------------------
# The picture
# ---------------------------------------------------------------------------
_LEVEL_COLOUR = {"F0": "#9aa0a6", "F1": "#ffd166", "F2": "#ffb347",
                 "F3": "#ff8c42", "F4": "#e8452b", "F5": "#a3231a",
                 "F5c": "#6d1b14", "F6": "#3f1210"}
_CLASS_STYLE = {
    "evacuee":        ("o", "#1b7f5a", 26, "evacuee (sidewalk/street, upwind)"),
    "onlooker":       ("o", "#2f6fb0", 18, "onlooker (further out)"),
    "at_car":         ("s", "#7b4fb5", 22, "at a kerb parking bay"),
    "window":         ("^", "#f2c14e", 40, "at a window, above the fire"),
    "roof":           ("*", "#00b3b3", 78, "roof refuge (intact deck)"),
    "casualty_apron": ("X", "#c1121f", 46, "prone at the rubble apron edge"),
    "roof_debris":    ("P", "#8d0801", 46, "under roof-deck debris"),
}


def render_png(plan, path, title):
    """The top-down gate picture. Returns the path, or None if matplotlib is
    not installed (the census and the rule checks still run — the picture is
    the nicest half of this tool, not the load-bearing half)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle, Polygon
    except ImportError as exc:
        print("[fire_people] no matplotlib ({0}); skipping the PNG".format(exc))
        return None

    sol = plan.solver
    x0, y0, x1, y1 = sol.region
    fig, ax = plt.subplots(figsize=(13.0, 13.0), dpi=110)
    ax.set_facecolor("#20242b")

    # --- ground classes -------------------------------------------------
    # NORMALISED THROUGH `layout_rects`: a real `city_layout`'s
    # `road_corridors` are dicts and its blocks are tuples, so indexing
    # `r[0]` draws the derived layout correctly and the real one not at all.
    for r in fp.layout_rects(sol.layout, "road_corridors"):
        ax.add_patch(Rectangle((r[0], r[1]), r[2] - r[0], r[3] - r[1],
                               facecolor="#33383f", edgecolor="none",
                               zorder=1))
    for r in fp.layout_rects(sol.layout, "paved_blocks"):
        ax.add_patch(Rectangle((r[0], r[1]), r[2] - r[0], r[3] - r[1],
                               facecolor="#3c414a", edgecolor="none",
                               zorder=1))
    for r in fp.layout_rects(sol.layout, "sidewalk_rects"):
        ax.add_patch(Rectangle((r[0], r[1]), r[2] - r[0], r[3] - r[1],
                               facecolor="#4a5058", edgecolor="none",
                               zorder=2))
    for r in fp.layout_rects(sol.layout, "blocks"):
        ax.add_patch(Rectangle((r[0], r[1]), r[2] - r[0], r[3] - r[1],
                               facecolor="none", edgecolor="#5a6068",
                               lw=0.6, zorder=3))

    # --- every building footprint ---------------------------------------
    def corners(cx, cy, W, D, yaw):
        return [(cx + p[0], cy + p[1]) for p in
                (fp._rot(sx * W / 2.0, sy * D / 2.0, yaw)
                 for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)))]

    for (cx, cy, W, D, yaw) in sol.footprints:
        ax.add_patch(Polygon(corners(cx, cy, W, D, yaw), closed=True,
                             facecolor="#2b3038", edgecolor="#6f7680",
                             lw=0.5, zorder=4))

    # --- the burning set, and its fire sides ----------------------------
    for b in sol.buildings:
        col = _LEVEL_COLOUR.get(b.level, "#ff8c42")
        ax.add_patch(Polygon(corners(b.x, b.y, b.W, b.D, b.yaw), closed=True,
                             facecolor=col, alpha=0.42, edgecolor=col,
                             lw=1.1, zorder=5))
        # standoff ring, dashed
        ax.add_patch(plt.Circle((b.x, b.y), b.radius + b.standoff,
                                fill=False, ls=(0, (4, 4)), lw=0.7,
                                edgecolor=col, alpha=0.55, zorder=5))
        for s in b.sides:
            cxf, cyf, half = fp.face_center(b.rec, s)
            nx, ny = fp.side_normal_world(s, b.yaw)
            tx, ty = -ny, nx
            ax.plot([cxf - tx * half, cxf + tx * half],
                    [cyf - ty * half, cyf + ty * half],
                    color=col, lw=3.4, solid_capstyle="butt", zorder=6)
        ax.annotate(b.level, (b.x, b.y), color="#f4f4f4", fontsize=6.4,
                    ha="center", va="center", zorder=7)

    # --- the wind -------------------------------------------------------
    dwn, up = fp.wind_vectors(plan.meta["heading_deg"])
    L = (x1 - x0) * 0.12
    ax.annotate("", xy=(x0 + L * 2.4 + dwn[0] * L, y1 - L * 0.9 + dwn[1] * L),
                xytext=(x0 + L * 2.4, y1 - L * 0.9),
                arrowprops=dict(arrowstyle="-|>", color="#e8eaed", lw=2.0),
                zorder=9)
    ax.text(x0 + L * 2.4, y1 - L * 1.5,
            "wind -> {0:.0f} deg (crowd upwind)".format(
                plan.meta["heading_deg"]),
            color="#e8eaed", fontsize=8, zorder=9)

    # --- the people -----------------------------------------------------
    seen = set()
    for r in plan.records:
        m, c, s, lab = _CLASS_STYLE[r["cls"]]
        ax.scatter([r["x"]], [r["y"]], marker=m, s=s, c=c,
                   edgecolors="#101317", linewidths=0.4, zorder=10,
                   label=(lab if r["cls"] not in seen else None))
        seen.add(r["cls"])
        # THE THREE-DIMENSIONAL CLASSES ARE ANNOTATED WITH THEIR z, because
        # a plan view cannot show it and a reviewer must not read a window
        # figure as somebody standing in the street.
        if r["cls"] == "window":
            ax.annotate("s{0} z{1:.0f}".format(r["storey"], r["z"]),
                        (r["x"], r["y"]), textcoords="offset points",
                        xytext=(4, 3), color="#f2c14e", fontsize=5.0,
                        zorder=11)
        elif r["cls"] == "roof":
            ax.annotate("z{0:.0f}".format(r["z"]), (r["x"], r["y"]),
                        textcoords="offset points", xytext=(4, 3),
                        color="#00b3b3", fontsize=5.0, zorder=11)
        elif r["cls"] in ("casualty_apron", "roof_debris"):
            ax.annotate("{0:.0f}%".format(100.0 * r.get("covered_frac", 0.0)),
                        (r["x"], r["y"]), textcoords="offset points",
                        xytext=(4, 3), color="#ffb3b3", fontsize=5.0,
                        zorder=11)

    ax.set_xlim(x0 - 6, x1 + 6)
    ax.set_ylim(y0 - 6, y1 + 6)
    ax.set_aspect("equal")
    ax.set_title(title, color="#f4f4f4", fontsize=10)
    ax.tick_params(colors="#9aa0a6", labelsize=7)
    for sp in ax.spines.values():
        sp.set_color("#5a6068")
    leg = ax.legend(loc="lower left", fontsize=6.6, framealpha=0.85,
                    facecolor="#20242b", edgecolor="#5a6068", markerscale=1.2)
    for t in leg.get_texts():
        t.set_color("#e8eaed")
    fig.patch.set_facecolor("#15181d")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.savefig(path, bbox_inches="tight", facecolor=fig.get_facecolor())
    import matplotlib.pyplot as _plt
    _plt.close(fig)
    return path


# ---------------------------------------------------------------------------
# The census
# ---------------------------------------------------------------------------
def print_census(plan):
    cen = fp.summarise(plan)
    m = plan.meta
    print("")
    print("=" * 74)
    print("PLACEMENT CENSUS  —  preset {0}, manifest seed {1}, people seed "
          "{2}".format(m.get("preset"), m.get("manifest_seed"), m["seed"]))
    print("=" * 74)
    print("  region            {0} m   burning buildings {1} of {2} "
          "placements".format(m.get("region_m"), m["n_burning"],
                              m["n_placements"]))
    print("  wind (toward)     {0:.0f} deg   epoch {1} s".format(
        m["heading_deg"], m.get("epoch_s")))
    print("  ground layout     {0}".format(m["layout_source"]))
    print("  bake sidecars     {0}".format(m["sidecars"]))
    print("  requested {0}   placed {1}   locations (groups) {2}".format(
        m["total_requested"], cen["total"], cen["locations"]))
    if m.get("shortfall_before_reflow"):
        print("  reflow            {0} unspent by a starved 3-D class, handed "
              "to the street classes".format(m["shortfall_before_reflow"]))
    print("")
    print("  class                 n    share   budget   needs_bench  "
          "aerially visible")
    tot = max(1, cen["total"])
    for c in fp.CLASSES:
        n = cen["by_class"].get(c, 0)
        nb = sum(1 for r in plan.records
                 if r["cls"] == c and r.get("needs_bench"))
        nv = sum(1 for r in plan.records
                 if r["cls"] == c and r.get("aerial_visible"))
        deg = "  DEGRADED" if c in plan.degraded else ""
        print("  {0:<18} {1:>4}   {2:>5.1%}   {3:>6}   {4:>11}   "
              "{5:>6}/{6}{7}".format(c, n, n / tot, m["budget"].get(c, 0),
                                     nb, nv, n, deg))
    print("")
    print("  group sizes        {0}   (window and roof_debris are "
          "singletons BY DESIGN: one figure per opening / per slab)".format(
              {k: cen["group_sizes"][k] for k in sorted(cen["group_sizes"])}))
    print("  poses              {0}   (a posed static takes NO pose — it "
          "has no skeleton)".format(cen["poses"]))
    print("  surfaces           {0}".format(cen["surfaces"]))
    print("  window variants    {0}".format(cen["window_variant"]))
    print("  window openings    {0}".format(cen["openings_source"]))
    print("  roof deck source   {0}".format(cen["deck_source"]))
    print("  occlusion patterns {0}".format(cen["occlusion"]))
    print("  distance to the burning wall (m)   {0}".format(cen["d_wall_m"]))
    print("  collapse-zone fraction (1.5H)      {0}".format(
        cen["collapse_zone_frac"]))
    print("  characters used    {0} of {1}   ({2} rigged placements)".format(
        cen["characters"], len(fp.RIGGED_HUMANS) + len(fp.POSED_HUMANS),
        cen["rigged"]))
    print("  needs_bench TOTAL  {0}   (nothing in a 2-D run can clear these)"
          .format(cen["needs_bench"]))
    if plan.degraded:
        print("  degraded classes   {0}   (no eligible building; share given "
              "back to evacuee/onlooker)".format(sorted(plan.degraded)))
    if plan.dropped:
        print("  dropped by the aerial-visibility filter: {0}".format(
            dict(sorted(plan.dropped.items(), key=lambda kv: -kv[1]))))
    if plan.refused:
        top = dict(sorted(plan.refused.items(),
                          key=lambda kv: -kv[1])[:10])
        print("  refusal tally (top 10): {0}".format(top))
    return cen


def print_converter(plan, n_examples=3):
    """Run `fire_people.to_placements` and show what the launcher will get.

    THE CONVERTER IS PART OF THE GATE. A record that is geometrically perfect
    and cannot be authored is worth nothing, and the failure is silent in
    exactly the way the rest of this pipeline is — so the conversion runs on
    every dry run, its skips are printed by reason, and three real dicts are
    shown so a reader can check the contract by eye rather than by trust.
    """
    ps, skipped = fp.to_placements(plan)
    n_skip = sum(len(v) for v in skipped.values())
    print("")
    print("  APPLY_PLACEMENTS CONVERSION  (fire_people.to_placements)")
    print("   {0} record(s) -> {1} placement(s), {2} skipped".format(
        len(plan.records), len(ps), n_skip))
    print("   NOTE: this is the host path (nominal stature/depth). The "
          "launcher should pass")
    print("         ctx= so `people._human_placement` measures each rig "
          "instead.")
    if skipped:
        print("   skipped by reason: {0}".format(
            {k: len(v) for k, v in sorted(skipped.items())}))
    # One example per shape that matters: a stander, a seated figure, a prone
    # burial, a window figure, a roof figure — whichever of those exist.
    want = [("a street stander", lambda r: r["cls"] in fp.STREET_CLASSES
             and not r.get("prone") and r.get("pose") in ("idle", "walk")),
            ("a prone burial", lambda r: bool(r.get("prone"))),
            ("a window sill-sitter", lambda r: r["cls"] == "window"
             and r.get("variant") == "sill_sit"),
            ("a roof figure", lambda r: r["cls"] == "roof"),
            ("a kerb sitter", lambda r: r.get("pose") == "sit_edge"
             and r["cls"] in fp.STREET_CLASSES)]
    by_id = {}
    ps_tagged, _ = fp.to_placements(plan, tag_ids=True)
    for p in ps_tagged:
        by_id[p["fire_people_id"]] = p
    shown = 0
    for label, pred in want:
        if shown >= n_examples:
            break
        for r in plan.records:
            if not pred(r) or r["id"] not in by_id:
                continue
            p = dict(by_id[r["id"]])
            p.pop("fire_people_id", None)
            print("")
            print("   -- {0}  (record {1}, cls {2}, z_mode {3}, support z "
                  "{4})".format(label, r["id"], r["cls"], r["z_mode"], r["z"]))
            print("      " + json.dumps(
                {k: (round(v, 4) if isinstance(v, float) else v)
                 for k, v in p.items()}))
            shown += 1
            break
    return ps, skipped


def print_checks(plan):
    print("")
    print("  RULE CHECKS")
    ok_all = True
    for name, ok, detail in fp.check_rules(plan):
        ok_all = ok_all and ok
        print("   {0:<34} {1}   {2}".format(
            name, "PASS" if ok else "FAIL",
            "{0}/{1} checked{2}".format(
                detail["n_violations"], detail["n_checked"],
                ("   " + detail["note"]) if detail["note"] else "")))
        if not ok:
            print("       violations: {0}".format(detail["violations"]))
    return ok_all


# ---------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--dump", default=os.path.join(
        _SCENE_GEN_DIR, "_plans", "fc_dump_500.json"))
    ap.add_argument("--manifest", default=os.path.join(
        _SCENE_GEN_DIR, "_plans", "fire_city_500b.json"))
    ap.add_argument("--sidecar-dir", default=None)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--total", type=int, default=None)
    ap.add_argument("--heading-deg", type=float, default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--json", default=None,
                    help="write the ground truth here as well")
    ap.add_argument("--synth", action="store_true",
                    help="synthetic dump/manifest — development only")
    ap.add_argument("--from-preset", default=None,
                    help="build the REAL city host-side instead of reading a "
                         "dump (needs usd-core; see the module docstring)")
    ap.add_argument("--layout-seed", type=int, default=None)
    args = ap.parse_args(argv)

    layout = None
    if args.from_preset:
        if not os.path.isfile(args.manifest):
            ap.error("--from-preset still needs a --manifest (the fire solve "
                     "for the same preset/seed); none at " + args.manifest)
        manifest = fp.load_manifest(args.manifest)
        dump, layout, note = preset_inputs(args.from_preset,
                                           seed=args.layout_seed)
        print("[fire_people] {0}".format(note))
    elif args.synth:
        print("!" * 74)
        print("!! SYNTHETIC INPUTS — a regular block grid, not the real BSP "
              "city.")
        print("!! This exercises the planner's arithmetic and is NOT evidence "
              "about the")
        print("!! real scene. Re-run with --dump/--manifest before quoting "
              "anything.")
        print("!" * 74)
        dump, manifest = synth_inputs(seed=args.seed)
        layout = None
    else:
        for p, what in ((args.dump, "dump"), (args.manifest, "manifest")):
            if not os.path.isfile(p):
                ap.error("no {0} at {1} (use --synth for a development "
                         "run)".format(what, p))
        dump = fp.load_dump(args.dump)
        manifest = fp.load_manifest(args.manifest)

    sidecars = fp.load_sidecars(args.sidecar_dir)
    heading, hsrc = (args.heading_deg, "--heading-deg") if \
        args.heading_deg is not None else \
        preset_heading_deg(manifest.get("preset") or dump.get("preset"))
    print("[fire_people] wind heading {0:.1f} deg from {1}".format(
        heading, hsrc))

    cfg = {"total": args.total} if args.total else None
    plan = fp.plan_people(dump, manifest, seed=args.seed, cfg=cfg,
                          layout=layout, sidecars=sidecars,
                          heading_deg=heading)

    out = args.out or os.path.join(_SCENE_GEN_DIR, "_plans",
                                   "fire_people_{0}.png".format(args.seed))
    title = ("fire_people — {0} seed {1} / people seed {2} — {3} figures at "
             "{4} locations{5}".format(
                 plan.meta.get("preset"), plan.meta.get("manifest_seed"),
                 args.seed, len(plan.records),
                 len({(r["cls"], r["group"]) for r in plan.records}),
                 "  [SYNTHETIC]" if args.synth else ""))
    png = render_png(plan, out, title)

    print_census(plan)
    print_converter(plan)
    ok = print_checks(plan)
    if png:
        print("")
        print("  PNG  -> {0}".format(png))
    if args.json:
        fp.write_records(args.json, plan)
        print("  JSON -> {0}".format(args.json))
    print("")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
