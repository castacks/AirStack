#!/usr/bin/env python3
"""
block_fill_probe.py — HOST-SIDE (no Isaac Sim, no Nucleus) measurement of how
much of each city BLOCK actually gets built on, broken down by typology, plus
a top-down matplotlib PNG of blocks / buildings / plaza props.

Answers the question "there is a lot of empty space in the blocks" with a
number instead of a screenshot: for every non-park block it reports

    fill = (sum of building footprint area clipped to the block interior)
           / (block interior area)

where `block interior` is the block rect shrunk by `districts.block_inset`
(the verge + sidewalk strip `build_city` holds content off the edge), i.e.
the ground a building is actually allowed to stand on.

It also re-runs `districts.free_rects` over each block's own obstacle set and
reports the LEFTOVER open rectangles — the same decomposition `infill_blocks`
walks — so an under-filled block can be told apart from one that is simply
full of small buildings.

Reuses `tools/fire_city_dry_run.build_layout`, so the layout, footprints and
RNG stream are the REAL ones (same localised building URLs, same
GAC/downtowncity measured footprint cache).

    python3 scene_gen/tools/block_fill_probe.py --preset downtown_fire_500 --seed 4
    python3 scene_gen/tools/block_fill_probe.py --png scene_gen/_plans/layout_fill_check.png
    python3 scene_gen/tools/block_fill_probe.py --json /tmp/fill.json --quiet
"""
import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.dirname(_HERE)
for _p in (_SG, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------
def _union_len(spans, lo, hi):
    """Length of the union of *spans*, clipped to ``[lo, hi]``."""
    cur, total = lo, 0.0
    for a, b in sorted((max(a, lo), min(b, hi)) for a, b in spans):
        if b <= cur:
            continue
        total += b - max(a, cur)
        cur = max(cur, b)
    return total


def street_wall(rect, rects, depth: float = 6.0):
    """``(covered_m, total_m)`` of the block's own frontage line.

    THE MEASURE THAT MATTERS FOR "the block looks empty". A block's four
    inset edges are its street walls; a metre of that line with no building
    within *depth* of it is a metre of bare block interior seen from the
    pavement, which is what reads as "the sidewalk is enormous here". Open
    ground in the MIDDLE of a block is a courtyard or a service alley and is
    not the same defect, so area alone cannot tell the two apart — a full-depth
    alley between two buildings touches two edges and would count as frontage
    void under any area test, while costing only its own width of street wall.
    """
    x0, y0, x1, y1 = rect
    total = covered = 0.0
    for lo, hi, sel, proj in (
            (x0, x1, lambda r: r[1] - y0 <= depth, lambda r: (r[0], r[2])),
            (x0, x1, lambda r: y1 - r[3] <= depth, lambda r: (r[0], r[2])),
            (y0, y1, lambda r: r[0] - x0 <= depth, lambda r: (r[1], r[3])),
            (y0, y1, lambda r: x1 - r[2] <= depth, lambda r: (r[1], r[3]))):
        total += hi - lo
        covered += _union_len([proj(r) for r in rects if sel(r)], lo, hi)
    return covered, total


def _clip_area(r, b) -> float:
    """Area of rect *r* clipped to rect *b* (both `(x0, y0, x1, y1)`)."""
    w = min(r[2], b[2]) - max(r[0], b[0])
    h = min(r[3], b[3]) - max(r[1], b[1])
    return w * h if w > 0 and h > 0 else 0.0


def _overlap(a, b, tol: float = 1e-6) -> float:
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w > tol and h > tol:
        return w * h
    return 0.0


# --------------------------------------------------------------------------
# the measurement
# --------------------------------------------------------------------------
def measure(config, layout, placements, resolver):
    """Return ``{"blocks": [...], "by_typology": {...}, "totals": {...}}``."""
    from detail import districts

    inset = districts.block_inset(config, resolver)
    blocks = list(layout.get("blocks") or [])
    typ_of = dict(layout.get("_typology_of") or {})
    parks = set(districts.park_blocks(layout, placements))

    houses = [p for p in placements if p.get("category") == "house"]
    hrects = [districts._rect_of(p, resolver) for p in houses]

    # Plaza / arrangement props, reported separately: they are what a block
    # interior is SUPPOSED to hold once no building fits there.
    plaza_cats = {"street_tree", "bench", "planter", "park_feature",
                  "cafe_set", "bike_rack", "bollard", "trash_can",
                  "streetlight", "dumpster", "bus_stop", "fire_hydrant",
                  "mailbox", "parking_meter", "phone_booth", "sign"}
    props = [p for p in placements if p.get("category") in plaza_cats]
    # The OPENINGS the composed arrangements were laid in — published by
    # `city_detail._place_plazas` so the figure can show that every cluster
    # sits in a gap between buildings and not on a sidewalk.
    arrangements = [list(map(float, a))
                    for a in (layout.get("_arrangement_rects") or ())]

    rows = []
    for blk in blocks:
        rect = (blk[0] + inset, blk[1] + inset, blk[2] - inset, blk[3] - inset)
        bw, bh = rect[2] - rect[0], rect[3] - rect[1]
        if bw <= 0 or bh <= 0:
            continue
        area = bw * bh
        name = "park" if blk in parks else typ_of.get(blk)
        built, n, local = 0.0, 0, []
        for p, r in zip(houses, hrects):
            a = _clip_area(r, rect)
            if a <= 0.0:
                continue
            built += a
            local.append(r)
            cx, cy = float(p["x_m"]), float(p["y_m"])
            if rect[0] <= cx <= rect[2] and rect[1] <= cy <= rect[3]:
                n += 1
        # leftover open ground, as the infill pass itself decomposes it
        frees = districts.free_rects(rect, local, 8.0)
        free_area = sum((f[2] - f[0]) * (f[3] - f[1]) for f in frees)
        # WHERE the open ground is, which matters more than how much of it
        # there is. Open ground on the block's own edge is a STREET
        # FRONTAGE with no building on it — the "the sidewalk is enormous
        # here" artefact. The same area in the middle of the block is a
        # courtyard or a service alley, which is what a dense block is
        # supposed to have. A rect counts as frontage when one of its sides
        # lies on the block's inset (sidewalk) line.
        tol = 0.5
        front = [f for f in frees
                 if (f[0] - rect[0] < tol or f[1] - rect[1] < tol
                     or rect[2] - f[2] < tol or rect[3] - f[3] < tol)]
        front_area = sum((f[2] - f[0]) * (f[3] - f[1]) for f in front)
        wall_cov, wall_tot = street_wall(rect, local)
        biggest = max(((f[2] - f[0]) * (f[3] - f[1]) for f in frees),
                      default=0.0)
        nprops = sum(1 for p in props
                     if rect[0] <= float(p["x_m"]) <= rect[2]
                     and rect[1] <= float(p["y_m"]) <= rect[3])
        rows.append({
            "block": [float(v) for v in blk],
            "arrangements": [a for a in arrangements
                             if rect[0] - 1.0 <= (a[0] + a[2]) / 2.0
                             <= rect[2] + 1.0
                             and rect[1] - 1.0 <= (a[1] + a[3]) / 2.0
                             <= rect[3] + 1.0],
            "rect": [float(v) for v in rect],
            "typology": name,
            "w_m": bw, "h_m": bh,
            "area_m2": area,
            "built_m2": built,
            "fill": built / area if area else 0.0,
            "n_buildings": n,
            "free_m2": free_area,
            "frontage_free_m2": front_area,
            "wall_covered_m": wall_cov,
            "wall_total_m": wall_tot,
            "free_rects": len(frees),
            "free": [[round(float(v), 2) for v in f] for f in frees],
            "biggest_free_m2": biggest,
            "n_props": nprops,
        })

    by = {}
    for r in rows:
        d = by.setdefault(r["typology"], {"n_blocks": 0, "area_m2": 0.0,
                                          "built_m2": 0.0, "free_m2": 0.0,
                                          "frontage_free_m2": 0.0,
                                          "wall_covered_m": 0.0,
                                          "wall_total_m": 0.0,
                                          "n_buildings": 0, "n_props": 0,
                                          "biggest_free_m2": 0.0})
        d["n_blocks"] += 1
        d["area_m2"] += r["area_m2"]
        d["built_m2"] += r["built_m2"]
        d["free_m2"] += r["free_m2"]
        d["frontage_free_m2"] += r["frontage_free_m2"]
        d["wall_covered_m"] += r["wall_covered_m"]
        d["wall_total_m"] += r["wall_total_m"]
        d["n_buildings"] += r["n_buildings"]
        d["n_props"] += r["n_props"]
        d["biggest_free_m2"] = max(d["biggest_free_m2"], r["biggest_free_m2"])
    for d in by.values():
        d["fill"] = d["built_m2"] / d["area_m2"] if d["area_m2"] else 0.0
        d["wall"] = (d["wall_covered_m"] / d["wall_total_m"]
                     if d["wall_total_m"] else 0.0)

    # footprint overlap audit — two buildings may never intersect
    worst, n_over = 0.0, 0
    for i in range(len(hrects)):
        for j in range(i + 1, len(hrects)):
            o = _overlap(hrects[i], hrects[j], tol=0.05)
            if o > 0.0:
                n_over += 1
                worst = max(worst, o)

    # ---- THE COMPOSED-SEATING AUDIT ------------------------------------
    # Every bench standing in a BLOCK INTERIOR (as opposed to on the kerb
    # ring, where a lone bench is exactly right) has to belong to a
    # composition: the plaza's ring, or the allée's back-to-back pair. A lone
    # bench on open ground is the "scattered rather than composed" defect the
    # suburb rule and this file's plaza section both exist to prevent, and it
    # is invisible in any count of how many benches were placed.
    #
    # Measured off the PLACED positions, not the formula that produced them,
    # so a half-placed pair (one bench refused by the occupancy grid) shows
    # up here rather than being averaged away.
    # NO LONE FURNITURE (user, 2026-08-31): a bench, a café set or a planter
    # may only exist as part of a composed group — a bench PAIR back to back,
    # a plaza RING, a café RUN, a planter LINE. Every one of those puts a
    # same-category companion within a few metres, so the audit is simply
    # "does this prop have one", measured off the placed positions rather
    # than off the rule that produced them: a group whose second member was
    # refused by the occupancy grid shows up here as a lone prop, which is
    # precisely the defect.
    group_span = 8.0
    lone, lone_by_cat = [], {}
    for cat in ("bench", "cafe_set", "planter"):
        here = [p for p in placements if p.get("category") == cat]
        for a in here:
            ax, ay = float(a["x_m"]), float(a["y_m"])
            if not any(b is not a
                       and (float(b["x_m"]) - ax) ** 2
                       + (float(b["y_m"]) - ay) ** 2 < group_span ** 2
                       for b in here):
                lone.append((cat, round(ax, 1), round(ay, 1)))
                lone_by_cat[cat] = lone_by_cat.get(cat, 0) + 1
    n_composed = sum(1 for p in placements
                     if p.get("category") in ("bench", "cafe_set", "planter"))

    # ---- TREE-Z CENSUS ---------------------------------------------------
    # "this tree is floating on the sidewalk" (user, 2026-08-31). A prop's
    # placement z has to put its lowest geometry ON whatever surface is under
    # it, and `apply_ground_planes` lays four of them: asphalt 0.00, block
    # grass 0.01, block paving 0.02, sidewalk ring `sidewalk_top_m`. The gap
    # reported here is `z + z0 - surface`, i.e. how far the asset's own bottom
    # sits above the ground it stands on — 0 is seated, positive is floating,
    # negative is a root ball correctly buried.
    from detail import city_detail as _cd
    surf = _cd._surface_field(layout, float(layout.get("sidewalk_top_m", 0.0)))
    z_gaps, worst_float = {}, []
    for p in placements:
        cat = p.get("category")
        if cat not in ("street_tree", "bench", "planter", "cafe_set",
                       "trash_can", "bollard", "streetlight", "dumpster",
                       "bus_stop", "fire_hydrant"):
            continue
        fp = resolver.get(p.get("usd", ""), cat, scale=p.get("scale"),
                          axis_up=p.get("axis_up", "Z"))
        # `base` is -z0, so the asset's own bottom is at z - base.
        gap = float(p.get("z_m", 0.0)) - float(fp["base"]) - surf(
            float(p["x_m"]), float(p["y_m"]))
        lo, hi = z_gaps.get(cat, (gap, gap))
        z_gaps[cat] = (min(lo, gap), max(hi, gap))
        if gap > 0.02:
            worst_float.append((cat, round(gap, 3),
                                round(float(p["x_m"]), 1),
                                round(float(p["y_m"]), 1)))
    worst_float.sort(key=lambda t: -t[1])

    # ---- PROP GROUND-FOOTPRINT OVERLAP ----------------------------------
    # The ground each prop stands on, not its bbox: a tree's CROWN overhangs
    # a bench by design (`city_detail._occ_extent`'s `_CANOPY` mode), so the
    # trunk is what may not collide.
    # Same convention `tools/plaza_check.py` grades against: the ORIENTED
    # footprint at the placement yaw, SAT-tested, with a `street_tree` reduced
    # to its trunk (`city_detail._occ_extent`'s `_CANOPY` mode) because a crown
    # OVER a bench is shade and is the point. An axis-aligned test is not
    # equivalent — a 5 m log bench at 40 degrees to the grid measures 4 m
    # across in x and hides a real collision.
    from detail import parks as _parks

    def _obox(p):
        cat = p.get("category")
        fp = resolver.get(p.get("usd", ""), cat, scale=p.get("scale"),
                          axis_up=p.get("axis_up", "Z"))
        sx, sy = fp["sx"], fp["sy"]
        if cat == "street_tree":
            sx = sy = max(0.2, min(sx, sy) * 0.25)
        return _parks._box(float(p["x_m"]), float(p["y_m"]),
                           float(p.get("yaw_deg", 0.0)), sx, sy)

    prop_boxes = [_obox(p) for p in props]
    prop_over, prop_pairs = 0, []
    for i in range(len(prop_boxes)):
        for j in range(i + 1, len(prop_boxes)):
            # CO-LOCATED PARTS OF ONE ASSEMBLY ARE NOT AN OVERLAP. A fountain
            # is a basin plus three water discs sharing one origin and one
            # ground reservation by design — the same exception
            # `tools/plaza_check.py` documents.
            if (abs(props[i]["x_m"] - props[j]["x_m"]) < 0.01
                    and abs(props[i]["y_m"] - props[j]["y_m"]) < 0.01):
                continue
            if _parks._box_hit(prop_boxes[i], prop_boxes[j]):
                prop_over += 1
                if len(prop_pairs) < 8:
                    prop_pairs.append(
                        f"{props[i]['category']}@"
                        f"({props[i]['x_m']:.1f},{props[i]['y_m']:.1f})"
                        f" x {props[j]['category']}@"
                        f"({props[j]['x_m']:.1f},{props[j]['y_m']:.1f})")

    tot_area = sum(r["area_m2"] for r in rows if r["typology"] != "park")
    tot_built = sum(r["built_m2"] for r in rows if r["typology"] != "park")
    return {
        "blocks": rows,
        "by_typology": by,
        "totals": {
            "n_blocks": len(rows),
            "n_buildings": len(houses),
            "area_m2": tot_area,
            "built_m2": tot_built,
            "fill": tot_built / tot_area if tot_area else 0.0,
            "free_m2": sum(r["free_m2"] for r in rows if r["typology"] != "park"),
            "frontage_free_m2": sum(r["frontage_free_m2"] for r in rows
                                    if r["typology"] != "park"),
            "wall": (sum(r["wall_covered_m"] for r in rows
                         if r["typology"] != "park")
                     / max(1e-9, sum(r["wall_total_m"] for r in rows
                                     if r["typology"] != "park"))),
            "overlaps": n_over,
            "worst_overlap_m2": worst,
            "n_props": len(props),
            "composed_props": n_composed,
            "lone_props": len(lone),
            "lone_by_cat": lone_by_cat,
            "lone_at": lone[:12],
            "z_gap_by_cat": {k: [round(v[0], 3), round(v[1], 3)]
                             for k, v in sorted(z_gaps.items())},
            "n_floating": len(worst_float),
            "worst_float": worst_float[:8],
            "prop_overlaps": prop_over,
            "prop_overlap_pairs": prop_pairs,
        },
    }


def report(m) -> str:
    out = []
    out.append("")
    out.append("typology        blocks   area m2   built m2   fill    bldgs  "
               "free m2  street wall  props")
    out.append("-" * 106)
    for name in sorted(m["by_typology"], key=lambda k: str(k)):
        d = m["by_typology"][name]
        out.append(f"{str(name):14s} {d['n_blocks']:6d} {d['area_m2']:9,.0f} "
                   f"{d['built_m2']:10,.0f}  {d['fill']:5.1%} {d['n_buildings']:7d} "
                   f"{d['free_m2']:8,.0f} {d['wall']:11.1%} {d['n_props']:6d}")
    t = m["totals"]
    out.append("-" * 106)
    out.append(f"{'TOTAL(non-park)':14s} {t['n_blocks']:6d} {t['area_m2']:9,.0f} "
               f"{t['built_m2']:10,.0f}  {t['fill']:5.1%} {t['n_buildings']:7d} "
               f"{t['free_m2']:8,.0f} {t['wall']:11.1%}")
    out.append(f"overlapping footprint pairs: {t['overlaps']} "
               f"(worst {t['worst_overlap_m2']:.2f} m2); "
               f"prop ground-footprint overlaps: {t['prop_overlaps']}"
               + (f" {t['prop_overlap_pairs']}" if t['prop_overlaps'] else ""))
    out.append(f"prop seating on the ground (z + z0 - surface, m): "
               f"{t['n_floating']} floating > 2 cm"
               + (f" {t['worst_float']}" if t['n_floating'] else "")
               + "\n  " + "  ".join(f"{k}[{v[0]:+.3f},{v[1]:+.3f}]"
                                    for k, v in t['z_gap_by_cat'].items()))
    out.append(f"no lone furniture: {t['composed_props']} bench/cafe_set/"
               f"planter placements, {t['lone_props']} with no same-category "
               f"companion within 8 m"
               + (f" {t['lone_by_cat']} -> {t['lone_at']}"
                  if t['lone_props'] else ""))
    out.append("")
    out.append("per block (worst fill first):")
    for r in sorted(m["blocks"], key=lambda r: r["fill"]):
        out.append(f"  {str(r['typology']):14s} {r['w_m']:6.1f} x {r['h_m']:6.1f} m  "
                   f"fill {r['fill']:5.1%}  n={r['n_buildings']:2d}  "
                   f"free {r['free_m2']:7,.0f} m2 in {r['free_rects']:2d} rect(s) "
                   f"wall {r['wall_covered_m'] / max(1e-9, r['wall_total_m']):5.1%}  "
                   f"biggest {r['biggest_free_m2']:7,.0f}  props={r['n_props']:3d}")
    return "\n".join(out)


# --------------------------------------------------------------------------
# picture
# --------------------------------------------------------------------------
_PROP_STYLE = {
    "street_tree": ("#2e7d32", 14, "^"),
    "bench":       ("#8d6e63", 10, "s"),
    "planter":     ("#6d4c41", 12, "h"),
    "park_feature": ("#0277bd", 22, "*"),
    "cafe_set":    ("#ad1457", 12, "D"),
    "bike_rack":   ("#455a64", 8, "|"),
    "bollard":     ("#546e7a", 5, "."),
    "trash_can":   ("#37474f", 6, "o"),
}


_COLOURS = {"rowhouse": "#c8a165", "lowrise": "#9ccc9c",
            "midrise": "#8fbcd4", "brick_midrise": "#d49a8f",
            "tower": "#b9a5d6", "highrise": "#e8d36a",
            "park": "#4c7a4c", None: "#777777"}


def _draw_props(ax, placements, x0, y0, x1, y1, scale=1.0, seen=None):
    seen = set() if seen is None else seen
    for p in placements:
        cat = p.get("category")
        st = _PROP_STYLE.get(cat)
        if not st:
            continue
        px, py = float(p["x_m"]), float(p["y_m"])
        if not (x0 <= px <= x1 and y0 <= py <= y1):
            continue
        col, size, mark = st
        ax.scatter([px], [py], s=size * scale, c=col, marker=mark, zorder=7,
                   linewidths=0.4, edgecolors="none",
                   label=cat if cat not in seen else None)
        seen.add(cat)
    return seen


def _draw_detail(ax, row, m, placements, resolver, title):
    """One block at a legible scale — this is where the plaza/court
    ARRANGEMENT is judged, since at region scale a tree is one pixel."""
    from matplotlib.patches import Rectangle
    from detail import districts
    import math as _m

    bx0, by0, bx1, by1 = row["block"]
    ax.set_facecolor("#3a3a3a")
    ax.add_patch(Rectangle((bx0, by0), bx1 - bx0, by1 - by0,
                           facecolor=_COLOURS.get(row["typology"], "#777"),
                           alpha=0.30, zorder=1))
    for f in row.get("free") or ():
        ax.add_patch(Rectangle((f[0], f[1]), f[2] - f[0], f[3] - f[1],
                               facecolor="#ff2d2d", edgecolor="#ff2d2d",
                               lw=0.6, alpha=0.22, hatch="//", zorder=4))
    for a in row.get("arrangements") or ():
        ax.add_patch(Rectangle((a[0], a[1]), a[2] - a[0], a[3] - a[1],
                               facecolor="none", edgecolor="#19e0c0", lw=2.0,
                               zorder=6))
    for p in placements:
        if p.get("category") != "house":
            continue
        rr = districts._rect_of(p, resolver)
        if rr[2] < bx0 or rr[0] > bx1 or rr[3] < by0 or rr[1] > by1:
            continue
        ax.add_patch(Rectangle((rr[0], rr[1]), rr[2] - rr[0], rr[3] - rr[1],
                               facecolor="#efefef", edgecolor="#101010",
                               lw=0.6, zorder=5))
    _draw_props(ax, placements, bx0, by0, bx1, by1, scale=7.0)
    # a bench's seat normal is its yaw + 90 (parks.py's ring convention) —
    # drawn so a back-to-back PAIR and an inward-facing RING are visible as
    # such rather than as two dots.
    for p in placements:
        if p.get("category") != "bench":
            continue
        px, py = float(p["x_m"]), float(p["y_m"])
        if not (bx0 <= px <= bx1 and by0 <= py <= by1):
            continue
        th = _m.radians(float(p.get("yaw_deg", 0.0)) + 90.0)
        ax.plot([px, px + 2.6 * _m.cos(th)], [py, py + 2.6 * _m.sin(th)],
                c="#ffe600", lw=1.1, zorder=8)
    ax.set_xlim(bx0 - 4, bx1 + 4)
    ax.set_ylim(by0 - 4, by1 + 4)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=9)
    ax.legend(loc="upper right", fontsize=6, framealpha=0.85, ncol=2)


def _table(m, before=None) -> str:
    head = ("typology        blk   area m2  built m2   fill   bldgs   free m2"
            "  wall")
    lines = [head, "-" * len(head)]
    for name in sorted(m["by_typology"], key=lambda k: str(k)):
        d = m["by_typology"][name]
        b = (before or {}).get("by_typology", {}).get(name)
        lines.append(f"{str(name):14s} {d['n_blocks']:3d} {d['area_m2']:9,.0f} "
                     f"{d['built_m2']:9,.0f} {d['fill']:6.1%} "
                     f"{d['n_buildings']:5d} {d['free_m2']:9,.0f} "
                     f"{d['wall']:5.1%}")
        if b:
            lines.append(f"{'  was':14s} {b['n_blocks']:3d} {b['area_m2']:9,.0f} "
                         f"{b['built_m2']:9,.0f} {b['fill']:6.1%} "
                         f"{b['n_buildings']:5d} {b['free_m2']:9,.0f} "
                         f"{b['wall']:5.1%}")
    t = m["totals"]
    lines.append("-" * len(head))
    lines.append(f"{'TOTAL':14s} {t['n_blocks']:3d} {t['area_m2']:9,.0f} "
                 f"{t['built_m2']:9,.0f} {t['fill']:6.1%} "
                 f"{t['n_buildings']:5d} {t['free_m2']:9,.0f} {t['wall']:5.1%}")
    if before:
        bt = before["totals"]
        lines.append(f"{'  was':14s} {bt['n_blocks']:3d} {bt['area_m2']:9,.0f} "
                     f"{bt['built_m2']:9,.0f} {bt['fill']:6.1%} "
                     f"{bt['n_buildings']:5d} {bt['free_m2']:9,.0f} "
                     f"{bt['wall']:5.1%}")
    lines.append("")
    lines.append(f"building footprint overlaps {t['overlaps']}   "
                 f"prop ground overlaps {t['prop_overlaps']}")
    lines.append(f"props floating above their surface: {t['n_floating']}")
    lines.append(f"bench/cafe_set/planter {t['composed_props']}, none lone: "
                 f"{t['lone_props'] == 0}"
                 + (f"  {t['lone_by_cat']}" if t['lone_props'] else ""))
    lines.append("")
    lines.append("fill   = building footprint / block interior")
    lines.append("free   = open ground >= 8 m across inside the block")
    lines.append("cyan   = the OPENING between buildings an arrangement was")
    lines.append("         laid in; red hatch = still-open ground")
    lines.append("wall   = share of the block frontage LINE with a building")
    lines.append("         within 6 m of it -- the measure of 'the block")
    lines.append("         looks empty' seen from the street")
    return "\n".join(lines)


def draw(m, config, layout, placements, resolver, path: str, title: str,
         before=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from detail import districts

    x0, y0, x1, y1 = layout.get("region", (0, 0, 0, 0))
    fig = plt.figure(figsize=(23, 18), dpi=110)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.30, 1.0],
                          height_ratios=[1.0, 1.0, 0.85])
    ax = fig.add_subplot(gs[:, 0])
    ax.set_facecolor("#3a3a3a")           # asphalt

    colours = _COLOURS

    for r in m["blocks"]:
        for a in r.get("arrangements") or ():
            ax.add_patch(Rectangle((a[0], a[1]), a[2] - a[0], a[3] - a[1],
                                   facecolor="none", edgecolor="#19e0c0",
                                   lw=1.6, ls="-", zorder=6))
        for f in r.get("free") or ():
            ax.add_patch(Rectangle((f[0], f[1]), f[2] - f[0], f[3] - f[1],
                                   facecolor="#ff2d2d", edgecolor="#ff2d2d",
                                   lw=0.6, alpha=0.28, hatch="//", zorder=4))
        bx0, by0, bx1, by1 = r["block"]
        c = colours.get(r["typology"], "#777777")
        ax.add_patch(Rectangle((bx0, by0), bx1 - bx0, by1 - by0,
                               facecolor=c, edgecolor="none", alpha=0.30,
                               zorder=1))
        rx0, ry0, rx1, ry1 = r["rect"]
        ax.add_patch(Rectangle((rx0, ry0), rx1 - rx0, ry1 - ry0,
                               facecolor="none", edgecolor=c, lw=0.8,
                               ls=":", zorder=2))
        ax.text((bx0 + bx1) / 2.0, (by0 + by1) / 2.0,
                f"{r['typology']}\n{r['fill']:.0%}  n={r['n_buildings']}",
                ha="center", va="center", fontsize=7, color="#ffffff",
                zorder=9, alpha=0.85)

    for p in placements:
        if p.get("category") != "house":
            continue
        rr = districts._rect_of(p, resolver)
        ax.add_patch(Rectangle((rr[0], rr[1]), rr[2] - rr[0], rr[3] - rr[1],
                               facecolor="#efefef", edgecolor="#101010",
                               lw=0.5, zorder=5))

    _draw_props(ax, placements, x0, y0, x1, y1)

    ax.set_xlim(x0 - 10, x1 + 10)
    ax.set_ylim(y0 - 10, y1 + 10)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=11)
    ax.legend(loc="upper right", fontsize=7, framealpha=0.85, ncol=2)

    # DETAIL: the block with the most plaza furniture (a skyscraper block, by
    # construction — that is where the plazas are), and the non-plaza block
    # with the most, which is where the planted courts are.
    order = sorted(m["blocks"], key=lambda r: -r["n_props"])
    sky = next((r for r in order if r["typology"] in ("highrise", "tower")),
               order[0])
    other = next((r for r in order
                  if r["typology"] not in ("highrise", "tower", "park")),
                 next((r for r in order if r is not sky), order[-1]))
    _draw_detail(fig.add_subplot(gs[0, 1]), sky, m, placements, resolver,
                 f"SKYSCRAPER BLOCK — {sky['typology']} "
                 f"{sky['w_m']:.0f}x{sky['h_m']:.0f} m: fountain, bench ring "
                 f"(yellow = seat normal), tree ring, allee "
                 f"({sky['n_props']} props)")
    _draw_detail(fig.add_subplot(gs[1, 1]), other, m, placements, resolver,
                 f"OTHER TYPOLOGY — {other['typology']} "
                 f"{other['w_m']:.0f}x{other['h_m']:.0f} m: planted courts in "
                 f"the block interior ({other['n_props']} props)")
    axt = fig.add_subplot(gs[2, 1])
    axt.axis("off")
    axt.text(0.0, 1.0, _table(m, before), family="monospace", fontsize=8.5,
             va="top", ha="left", transform=axt.transAxes)

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--preset", default="downtown_fire_500")
    ap.add_argument("--seed", type=int, default=4)
    ap.add_argument("--png", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--compare", default=None,
                    help="a JSON written by an earlier run (e.g. --legacy); "
                         "its numbers are shown as `was` rows under each of "
                         "this run's, which is the before/after table")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--legacy", action="store_true",
                    help="turn OFF all three 2026-08-31 under-fill fixes "
                         "(districts.pack_justify, districts.infill."
                         "rank_fallback, and the `_built_typology_of` "
                         "hand-off that lets infill see a terrace-refused "
                         "block) so the same tool measures the packing this "
                         "preset had before them")
    a = ap.parse_args()

    import fire_city_dry_run as fdr
    if a.legacy:
        import compile_disaster
        _orig = compile_disaster.load_scene_config

        def _legacy_cfg(*args, **kw):
            cfg = _orig(*args, **kw)
            d = cfg.setdefault("districts", {})
            d["pack_justify"] = False
            d.setdefault("infill", {})["rank_fallback"] = 0
            return cfg

        compile_disaster.load_scene_config = _legacy_cfg

        # ...and hide the built-typology hand-off, so a terrace-refused block
        # goes back to being skipped by infill as "back yards" the way it was.
        from detail import districts as _dd
        _orig_infill = _dd.infill_blocks

        def _legacy_infill(config, layout, placements, resolver, rng, zone_at):
            layout.pop("_built_typology_of", None)
            return _orig_infill(config, layout, placements, resolver, rng,
                                zone_at)

        _dd.infill_blocks = _legacy_infill
    config, layout, placements, resolver = fdr.build_layout(a.preset,
                                                            seed=a.seed)
    m = measure(config, layout, placements, resolver)
    if not a.quiet:
        print(report(m))
    if a.json:
        with open(a.json, "w") as fh:
            json.dump(m, fh, indent=1)
        print(f"[fill] wrote {a.json}")
    if a.png:
        t = (f"{a.preset} seed {a.seed} — block fill "
             f"{m['totals']['fill']:.1%} over {m['totals']['n_blocks']} blocks, "
             f"{m['totals']['n_buildings']} buildings")
        before = None
        if a.compare:
            with open(a.compare) as fh:
                before = json.load(fh)
        out = draw(m, config, layout, placements, resolver, a.png, t, before)
        print(f"[fill] wrote {out}")


if __name__ == "__main__":
    main()
