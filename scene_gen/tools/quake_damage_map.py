#!/usr/bin/env python3
"""Draw the exact building/grade/failure-face plan from a quake city run.

The Isaac launcher writes ``quake_buildings.json`` beside its captures.  This
tool turns that machine-readable record into the review image that must be
checked *before* judging oblique renders: footprint colour is the assigned
grade, thick red edges are structural failure faces, and dashed amber edges
are directions which only received surface/parapet debris.

Example::

    python3 tools/quake_damage_map.py \
      --records ~/docker/isaac-sim/logs/eq250/quake_buildings.json \
      --manifest assets/archetype/archetypes.json \
      --manifest assets/gac_quake/gac_quake.json \
      --region 250 --out ~/docker/isaac-sim/logs/eq250/damage_plan.png
"""

import argparse
import json
import math
import os


GRADE_COLOUR = {
    "DG0": "#d9e6f2",
    "DG1": "#b8d98b",
    "DG2": "#f3d56b",
    "DG3": "#f39a4a",
    "DG4": "#dc5a4f",
    "DG5": "#7c2638",
    "SETTLE": "#a78bd4",
    "TILT": "#815ac0",
    "OV": "#3e2464",
}


def _rows(path):
    doc = json.load(open(path))
    if isinstance(doc, list):
        return doc
    for key in ("archetypes", "records", "entries"):
        if isinstance(doc.get(key), list):
            return doc[key]
    return []


def _base_grade(label):
    label = str(label or "DG0")
    if label.startswith("AEC_"):
        label = label[4:]
    return label.split("+", 1)[0]


def _manifest_index(paths):
    out = {}
    for path in paths:
        for row in _rows(path):
            name = row.get("style") or row.get("name")
            grade = row.get("level") or row.get("grade")
            if name and grade:
                out[(str(name), _base_grade(grade))] = row
    return out


def _rotate(x, y, yaw_deg):
    a = math.radians(float(yaw_deg))
    return (x * math.cos(a) - y * math.sin(a),
            x * math.sin(a) + y * math.cos(a))


def _world_point(rec, lx, ly):
    dx, dy = _rotate(lx, ly, rec.get("yaw_deg", 0.0))
    return float(rec["x"]) + dx, float(rec["y"]) + dy


def _footprint(rec):
    w, d = float(rec.get("W", 20.0)), float(rec.get("D", 20.0))
    return [_world_point(rec, x, y) for x, y in
            ((-w / 2, -d / 2), (w / 2, -d / 2),
             (w / 2, d / 2), (-w / 2, d / 2))]


def _side_segment(rec, side):
    w, d = float(rec.get("W", 20.0)), float(rec.get("D", 20.0))
    local = {
        "S": ((-w / 2, -d / 2), (w / 2, -d / 2), (0.0, -1.0)),
        "N": ((w / 2, d / 2), (-w / 2, d / 2), (0.0, 1.0)),
        "E": ((w / 2, -d / 2), (w / 2, d / 2), (1.0, 0.0)),
        "W": ((-w / 2, d / 2), (-w / 2, -d / 2), (-1.0, 0.0)),
    }[side]
    a, b, n = local
    aw, bw = _world_point(rec, *a), _world_point(rec, *b)
    nw = _rotate(*n, rec.get("yaw_deg", 0.0))
    return aw, bw, nw


def _world_side(rec, side):
    """Nearest world cardinal for one building-local face normal."""
    _a, _b, n = _side_segment(rec, side)
    if abs(n[0]) >= abs(n[1]):
        return "E" if n[0] >= 0.0 else "W"
    return "N" if n[1] >= 0.0 else "S"


def _type_of(rec, row):
    return str((row or {}).get("type") or (row or {}).get("btype") or
               ("monolith" if rec.get("mono") else "unknown"))


def _meaning(grade, btype):
    if grade == "DG0":
        return "intact"
    if grade == "DG1":
        return "minor cracking / local non-structural loss"
    if grade == "DG2":
        return ("masonry cracking + local wall loss" if btype == "urm" else
                "moderate glazing/infill and local structural damage")
    if grade == "DG3":
        return ("partial corner/elevation masonry collapse" if btype == "urm" else
                "partial facade/infill failure")
    if grade == "DG4":
        return ("heavy partial masonry collapse" if btype == "urm" else
                "soft/mid-storey or heavy frame damage")
    if grade == "DG5":
        return ("total masonry collapse with any braced stub retained"
                if btype == "urm" else "pancake / major structural collapse")
    if grade == "SETTLE":
        return "foundation settlement"
    if grade == "TILT":
        return "foundation tilt (shell largely retained)"
    if grade == "OV":
        return "overturn onto clear ground"
    return "assigned damage"


def _people_rows(path):
    if not path:
        return []
    doc = json.load(open(path))
    return doc.get("people", []) if isinstance(doc, dict) else doc


def draw(records, manifests, out_path, region_m=None, title=None,
         people=None, map_only=False):
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Polygon, Patch

    idx = _manifest_index(manifests)
    joined = []
    for i, rec in enumerate(records, 1):
        grade = _base_grade(rec.get("grade"))
        row = idx.get((str(rec.get("style", "")), grade))
        fall = tuple(rec.get("fall_sides") or
                     (row or {}).get("fall_sides") or ())
        # New manifests record the collapse planner's structural faces
        # separately. An old manifest has no such field, so preserve its
        # historical all-red rendering rather than silently hiding data.
        failure = tuple(rec.get("failure_sides") or
                        (row or {}).get("failure_sides", fall) or ())
        debris_only = tuple(side for side in fall if side not in failure)
        display_i = int(rec.get("_review_index", i))
        joined.append((display_i, rec, grade, row, failure, debris_only))

    if region_m is not None:
        half = float(region_m) / 2.0
        xlim, ylim = (-half, half), (-half, half)
    else:
        pts = [p for _i, r, _g, _row, _failure, _debris in joined
               for p in _footprint(r)]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        pad = 10.0
        xlim, ylim = (min(xs) - pad, max(xs) + pad), (min(ys) - pad, max(ys) + pad)

    # A 500 m scene can contain 100+ buildings; its inventory is machine-
    # readable in the JSON, while squeezing it beside the plan makes both
    # halves illegible. ``map_only`` spends the full canvas on spatial review.
    fig = plt.figure(figsize=((13, 13) if map_only else (18, 11)),
                     facecolor="#15191e")
    if map_only:
        ax, table = fig.add_subplot(1, 1, 1), None
    else:
        gs = fig.add_gridspec(1, 2, width_ratios=(1.55, 1.0), wspace=0.04)
        ax, table = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])
    ax.set_facecolor("#252a31")
    ax.axhline(0, color="#3a424c", lw=0.5)
    ax.axvline(0, color="#3a424c", lw=0.5)

    for i, rec, grade, row, failure, debris_only in joined:
        face = GRADE_COLOUR.get(grade, "#82909d")
        poly = Polygon(_footprint(rec), closed=True, facecolor=face,
                       edgecolor="#e8edf2", lw=0.8, alpha=0.92, zorder=3)
        ax.add_patch(poly)
        ax.text(float(rec["x"]), float(rec["y"]), str(i), ha="center",
                va="center", fontsize=8, weight="bold", color="#101317",
                zorder=6)
        for side in failure:
            if side not in "SENW":
                continue
            a, b, n = _side_segment(rec, side)
            ax.plot((a[0], b[0]), (a[1], b[1]), color="#ff334f",
                    lw=3.1, solid_capstyle="round", zorder=7)
            mx, my = 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])
            arrow = min(7.0, max(3.0, 0.18 * float(rec.get("H", 12.0))))
            ax.arrow(mx, my, n[0] * arrow, n[1] * arrow,
                     width=0.28, head_width=1.8, head_length=2.1,
                     length_includes_head=True, color="#ff334f", zorder=8)
        for side in debris_only:
            if side not in "SENW":
                continue
            a, b, n = _side_segment(rec, side)
            ax.plot((a[0], b[0]), (a[1], b[1]), color="#ffad42",
                    lw=2.6, linestyle=(0, (4, 2)),
                    solid_capstyle="round", zorder=7)
            mx, my = 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])
            arrow = min(5.0, max(2.5, 0.14 * float(rec.get("H", 12.0))))
            ax.arrow(mx, my, n[0] * arrow, n[1] * arrow,
                     width=0.20, head_width=1.4, head_length=1.7,
                     length_includes_head=True, color="#ffad42", zorder=8)

    people = list(people or ())
    casualty_states = {"prone_casualty", "interior_casualty",
                       "rubble_casualty"}
    standing = [p for p in people if p.get("active", True)
                and p.get("state") not in casualty_states]
    interior = [p for p in people if p.get("active", True)
                and p.get("state") == "interior_casualty"]
    rubble = [p for p in people if p.get("active", True)
              and p.get("state") == "rubble_casualty"]
    legacy = [p for p in people if p.get("active", True)
              and p.get("state") == "prone_casualty"]
    casualties = interior + rubble + legacy
    if standing:
        ax.scatter([p["x"] for p in standing], [p["y"] for p in standing],
                   s=18, marker="o", facecolor="#4ee6ff", edgecolor="#10242a",
                   linewidth=0.5, zorder=10)
    if interior:
        ax.scatter([p["x"] for p in interior], [p["y"] for p in interior],
                   s=32, marker="D", facecolor="#ff70d8", edgecolor="#4c153f",
                   linewidth=0.6, zorder=11)
    if rubble or legacy:
        _ground = rubble + legacy
        ax.scatter([p["x"] for p in _ground], [p["y"] for p in _ground],
                   s=35, marker="x", color="#ff4dd2", linewidth=1.4, zorder=11)
    if casualties:
        # IDs make the plan directly traceable to quake_people.json / prims.
        for p in casualties:
            ax.annotate(str(p.get("id", "")).replace("eqp_", "P"),
                        (float(p["x"]), float(p["y"])), xytext=(3, 3),
                        textcoords="offset points", fontsize=5.2,
                        color="#ffb7ed", zorder=12)

    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_xlabel("world X (m)", color="#cbd2da")
    ax.set_ylabel("world Y (m)", color="#cbd2da")
    ax.tick_params(colors="#aeb7c2")
    for spine in ax.spines.values():
        spine.set_color("#59636e")
    ax.annotate("N / +Y", xy=(xlim[0] + 9, ylim[1] - 6),
                xytext=(xlim[0] + 9, ylim[1] - 22), color="white",
                ha="center", arrowprops=dict(color="white", width=1,
                                               headwidth=7))
    ax.set_title(title or "Earthquake damage assignment",
                 color="white", fontsize=15, pad=12)

    legend = [Patch(facecolor=c, edgecolor="#eee", label=g)
              for g, c in GRADE_COLOUR.items()]
    legend.append(Line2D([0], [0], color="#ff334f", lw=3,
                         label="structural facade/frame failure"))
    legend.append(Line2D([0], [0], color="#ffad42", lw=2.6,
                         linestyle=(0, (4, 2)),
                         label="debris direction only (e.g. parapet)"))
    if standing:
        legend.append(Line2D([0], [0], marker="o", color="none",
                             markerfacecolor="#4ee6ff", markersize=6,
                             label="standing / walking person"))
    if interior:
        legend.append(Line2D([0], [0], marker="D", color="none",
                             markerfacecolor="#ff70d8", markeredgecolor="#4c153f",
                             markersize=6, label="casualty inside partial collapse"))
    if rubble or legacy:
        legend.append(Line2D([0], [0], marker="x", color="#ff4dd2",
                             markersize=7, linestyle="none",
                             label="casualty partially covered by rubble"))
    if map_only:
        ax.legend(handles=legend, loc="upper center", bbox_to_anchor=(0.5, -0.07),
                  ncol=3, fontsize=8, facecolor="#20252b",
                  labelcolor="white", framealpha=0.95)
    else:
        ax.legend(handles=legend, loc="lower left", fontsize=8,
                  facecolor="#20252b", labelcolor="white", framealpha=0.95)

    if table is not None:
        table.set_facecolor("#15191e")
        table.axis("off")
        lines = ["BUILDING DAMAGE INVENTORY", ""]
        for i, rec, grade, row, failure, debris_only in joined:
            btype = _type_of(rec, row)
            modifier = str(rec.get("grade", ""))
            faces = ",".join(_world_side(rec, q) for q in failure) \
                if failure else "—"
            debris = ",".join(_world_side(rec, q) for q in debris_only) \
                if debris_only else "—"
            local_faces = ",".join(failure) if failure else "—"
            local_debris = ",".join(debris_only) if debris_only else "—"
            lines.append("{0:02d}  {1}  {2}".format(
                i, modifier, rec.get("style", "?")))
            lines.append("     {0}; structural faces (world): {1}".format(
                _meaning(grade, btype), faces))
            if failure and faces != local_faces:
                lines.append("     source-archetype local faces: " + local_faces)
            if debris_only:
                lines.append("     surface/parapet debris only (world): " + debris)
                if debris != local_debris:
                    lines.append("     source-archetype local debris: " + local_debris)
            if rec.get("interaction"):
                lines.append("     interaction: " + str(rec["interaction"]))
        lines += ["", "Solid red = structural shell/frame failure.",
                  "Dashed amber = debris direction without facade failure.",
                  "No marked edge means no directional collapse face",
                  "(typically foundation motion or a generic monolith action).",
                  "", "People: {0} standing/walking, {1} inside partial "
                  "collapse, {2} rubble-covered casualty."
                  .format(len(standing), len(interior), len(rubble) + len(legacy))]
        table.text(0.01, 0.99, "\n".join(lines), va="top", ha="left",
                   family="monospace", fontsize=8.3, color="#e7ebef",
                   linespacing=1.24)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)
    return joined


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--records", required=True)
    ap.add_argument("--manifest", action="append", default=[])
    ap.add_argument("--out", required=True)
    ap.add_argument("--region", type=float, default=None)
    ap.add_argument("--title", default=None)
    ap.add_argument("--people", default=None,
                    help="quake_people.json written by the Isaac launcher")
    ap.add_argument("--map-only", action="store_true",
                    help="use the full canvas for the spatial map; omit inventory")
    focus = ap.add_mutually_exclusive_group()
    focus.add_argument("--focus-index", type=int, default=None,
                       help="draw one 1-based record, retaining its city-map number")
    focus.add_argument("--focus-prim", default=None,
                       help="draw the record whose prim path exactly matches")
    args = ap.parse_args()
    records = json.load(open(args.records))
    if args.focus_index is not None:
        if args.focus_index < 1 or args.focus_index > len(records):
            ap.error("--focus-index is outside the records list")
        rec = dict(records[args.focus_index - 1])
        rec["_review_index"] = args.focus_index
        records = [rec]
    elif args.focus_prim is not None:
        matches = [(i, rec) for i, rec in enumerate(records, 1)
                   if str(rec.get("prim")) == args.focus_prim]
        if len(matches) != 1:
            ap.error("--focus-prim matched {0} records".format(len(matches)))
        i, raw = matches[0]
        rec = dict(raw)
        rec["_review_index"] = i
        records = [rec]
    joined = draw(records, args.manifest, args.out, args.region, args.title,
                  people=_people_rows(args.people), map_only=args.map_only)
    known = sum(bool(row) for _i, _r, _g, row, _failure, _debris in joined)
    print("[quake_map] {0}: {1} building(s), {2} manifest-matched".format(
        args.out, len(joined), known))


if __name__ == "__main__":
    main()
