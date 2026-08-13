"""park_png.py — detailed top-down plan of a `suburb_park` layout.

    python3 tools/park_png.py --seed 3 --out _plans/park.png

Draws the park as SURFACES with their real materials and real line markings,
not as labelled boxes: the basketball compound is a painted slab inside a
chain-link run, the playground is sand, the pitch is mown grass with its
penalty areas on it. That is the point of the drawing — a box labelled
"basketball" tells you nothing about whether four courts actually fit, and the
markings are the check that they do.
"""

import argparse
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

import suburb_park as pk                                       # noqa: E402

# Surfaces, picked to read as materials rather than as a legend.
COL = {
    "grass":       "#4c7a3a",
    "grass_pitch": "#43702f",     # mown, slightly deeper than rough lawn
    "lawn":        "#568741",
    "sand":        "#d9c39a",     # playground
    "asphalt":     "#3f4a55",     # basketball slab
    "tennis":      "#2f6b52",     # surround
    "tennis_in":   "#2e5f86",     # inside the lines
    "path":        "#c4b295",
    "line":        "#f2f2ee",
    "fence":       "#9aa3ab",
    "picnic":      "#6b7f4e",
    "bike":        "#8f6f52",   # sealed bike circuit, darker than footpath
}

PROP = {                 # colour, radius in metres, z-order
    "hoop":            ("#e8712f", 1.1, 9),
    "picnic_table":    ("#8a5a2b", 1.4, 9),
    "gazebo":          ("#a8763f", 3.2, 9),
    "fountain":        ("#7fb2e5", 3.0, 9),
    "park_sign":       ("#d8c257", 1.6, 10),
    "swing_set":       ("#c8503f", 3.4, 9),
    "play_structure":  ("#c8503f", 4.0, 9),
    "seesaw":          ("#d9793f", 1.8, 9),
    "tree":            ("#2f5a2a", 3.4, 8),
    "soccer_goal":     ("#efefef", 2.0, 9),
    # path furniture — placed against the finished route, so it reads as
    # belonging to the path rather than scattered on the grass
    "bench":           ("#b5883f", 1.1, 10),
    "trash_can":       ("#2b3a33", 0.8, 10),
}


def _poly(ax, pts, **kw):
    from matplotlib.patches import Polygon
    ax.add_patch(Polygon(pts, closed=True, **kw))


def _rect_pts(r):
    x0, y0, x1, y1 = r
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def draw(park, out_path, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    x0, y0, x1, y1 = park["region"]
    W, H = x1 - x0, y1 - y0
    fig, ax = plt.subplots(figsize=(16, 16 * H / W))
    ppm = fig.dpi * fig.get_size_inches()[0] / W        # points per metre

    # 0) park ground
    _poly(ax, _rect_pts(park["region"]), facecolor=COL["grass"],
          edgecolor="none", zorder=0)

    def markings(lines, cx, cy, lw=0.12, col=None, z=6):
        for ln in lines:
            ax.plot([cx + p[0] for p in ln], [cy + p[1] for p in ln],
                    color=col or COL["line"], lw=max(0.6, lw * ppm),
                    solid_capstyle="round", zorder=z)

    def xf(pts, cx, cy, yaw):
        """Local facility coords -> world, through the facility's rotation."""
        a = math.radians(yaw)
        ux, uy = math.cos(a), math.sin(a)
        return [(cx + ux * x - uy * y, cy + uy * x + ux * y) for (x, y) in pts]

    def markings(lines, cx, cy, yaw, lw=0.12, col=None, z=6):
        for ln in lines:
            w = xf(ln, cx, cy, yaw)
            ax.plot([q[0] for q in w], [q[1] for q in w],
                    color=col or COL["line"], lw=max(0.6, lw * ppm),
                    solid_capstyle="round", zorder=z)

    # 1) facility surfaces + their markings, all rotated with the facility
    for z in park["zones"]:
        k = z["kind"]
        cx, cy = z["centre"]
        yaw = z["yaw"]
        if k == "soccer":
            _poly(ax, z["corners"], facecolor=COL["grass_pitch"],
                  edgecolor="none", zorder=2)
            markings(pk.soccer_markings(), cx, cy, yaw, lw=0.12)
        elif k == "basketball_compound":
            _poly(ax, z["corners"], facecolor=COL["asphalt"],
                  edgecolor="none", zorder=2)
            for court in z["courts"]:
                markings(pk.basketball_markings(), court["centre"][0],
                         court["centre"][1], court["yaw"], lw=0.09)
        elif k == "tennis_block":
            _poly(ax, z["corners"], facecolor=COL["tennis"],
                  edgecolor="none", zorder=2)
            cl, cwid = pk.TENNIS["court"]
            for court in z["courts"]:
                ccx, ccy = court["centre"]
                _poly(ax, xf([(-cl / 2, -cwid / 2), (cl / 2, -cwid / 2),
                              (cl / 2, cwid / 2), (-cl / 2, cwid / 2)],
                             ccx, ccy, court["yaw"]),
                      facecolor=COL["tennis_in"], edgecolor="none", zorder=3)
                markings(pk.tennis_markings(), ccx, ccy, court["yaw"], lw=0.08)
        elif k == "playground":
            _poly(ax, z["corners"], facecolor=COL["sand"], edgecolor="none",
                  zorder=2)
        elif k == "picnic":
            _poly(ax, z["corners"], facecolor=COL["picnic"], edgecolor="none",
                  zorder=2)

    # 2) paths — ONE shared-use way per route, drawn as its two sides.
    # Not two networks: a separate bike circuit had to weave past the pedestrian
    # one and tangled wherever they met. The divider is drawn from the same
    # centreline, so the two sides cannot drift apart or cross.
    for p in park["paths"]:
        pts = p["pts"]
        w = float(p.get("width_m", 2.6))
        share = float(p.get("bike_share", 0.0))
        ax.plot([q[0] for q in pts], [q[1] for q in pts], color=COL["path"],
                lw=max(1.0, w * ppm), solid_capstyle="round",
                solid_joinstyle="round", zorder=5)
        if share > 0.0:
            bw = w * share
            # Cycle side sits against one edge; offset to the centre of it.
            side = pk.offset_polyline(pts, (w - bw) / 2.0)
            ax.plot([q[0] for q in side], [q[1] for q in side],
                    color=COL["bike"], lw=max(0.8, bw * ppm),
                    solid_capstyle="round", solid_joinstyle="round", zorder=6)
            div = pk.offset_polyline(pts, w / 2.0 - bw)
            ax.plot([q[0] for q in div], [q[1] for q in div], color="#efe7d6",
                    lw=max(0.4, 0.16 * ppm), solid_capstyle="butt", zorder=7)

    # 3) fences — a mark per real panel, so the run reads as panels not a line
    for f in park["fences"]:
        cx, cy = f["c"]
        a = math.radians(f["yaw"])
        hx, hy = math.cos(a) * 1.2, math.sin(a) * 1.2
        ax.plot([cx - hx, cx + hx], [cy - hy, cy + hy], color=COL["fence"],
                lw=max(1.4, 0.5 * ppm), solid_capstyle="butt", zorder=8)
        # a post at each panel end, which is what makes a run read as fence
        ax.plot([cx - hx], [cy - hy], marker="o", ms=1.6,
                color="#6f787f", zorder=8)

    # 4) props. The goal gets its true 7.32 m mouth and net depth rather than
    # a round marker, because at this scale a circle would misreport its size
    # by a factor of three and the whole point of the drawing is that the
    # dimensions are checkable.
    gw = pk.SOCCER["goal_w"] / 2.0
    for p in park["props"]:
        if p["kind"] == "soccer_goal":
            cx, cy = p["c"]
            a = math.radians(p["yaw"])
            ux, uy = math.cos(a), math.sin(a)        # into the pitch
            vx, vy = -uy, ux                         # along the goal line
            d = 2.0                                  # net depth
            _poly(ax, [(cx + vx * gw, cy + vy * gw),
                       (cx - vx * gw, cy - vy * gw),
                       (cx - vx * gw - ux * d, cy - vy * gw - uy * d),
                       (cx + vx * gw - ux * d, cy + vy * gw - uy * d)],
                  facecolor="#e9edf0", edgecolor="#b9c2c8",
                  lw=0.6, alpha=0.9, zorder=9)
            continue
        col, r, z = PROP.get(p["kind"], ("#ff00ff", 1.0, 9))
        if p["kind"] == "tree":
            # Deterministic jitter off the position, so a copse reads as a
            # canopy of mixed crowns rather than a field of identical dots.
            h = (int(abs(p["c"][0]) * 7.3) ^ int(abs(p["c"][1]) * 11.7)) % 100
            r *= 0.62 + h / 100.0 * 0.85
            col = ("#2f5a2a", "#356331", "#294f26")[h % 3]
        ax.add_patch(Circle(p["c"], r, facecolor=col, edgecolor="none",
                            alpha=0.95, zorder=z))

    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.axis("off")
    if title:
        ax.set_title(title, color="#e8e8e8", fontsize=13)
    fig.patch.set_facecolor("#1b1b1b")
    fig.savefig(out_path, dpi=120, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--width", type=float, default=None)
    ap.add_argument("--height", type=float, default=None)
    ap.add_argument("--out", default="_plans/park.png")
    args = ap.parse_args()

    cfg = {}
    if args.width and args.height:
        cfg["region_m"] = [args.width, args.height]
    rng = random.Random(args.seed)
    park = pk.plan(rng, cfg)
    s = pk.stats(park)
    W = park["region"][2] - park["region"][0]
    H = park["region"][3] - park["region"][1]
    print(f"[park] {W:.0f} x {H:.0f} m ({W * H / 10000:.1f} ha)  seed {args.seed}")
    print(f"[park] zones: {s['zones']}")
    print(f"[park] props: {s['props']}")
    print(f"[park] {s['fence_panels']} fence panels, {s['paths']} paths")
    # THE CHECK THAT MATTERS. A plan that looks fine and routes a path over the
    # tennis block is not fine, and at this scale the crossing is a few pixels.
    bad = pk.check(park)
    print(f"[park] path-facility intersections: {bad}"
          + ("  <-- BROKEN" if bad else "  OK"))

    out = args.out
    if not os.path.isabs(out):
        out = os.path.join(os.path.dirname(_HERE), out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    zc = s["zones"]
    draw(park, out, title=(
        f"suburban park — {W:.0f} x {H:.0f} m ({W * H / 10000:.1f} ha)   "
        f"{zc.get('basketball_compound', 0) and pk.DEFAULTS['n_basketball']} "
        f"basketball · {zc.get('tennis_block', 0) and pk.DEFAULTS['n_tennis']} "
        f"tennis · {zc.get('soccer', 0)} pitch · "
        f"{zc.get('picnic', 0)} picnic · {s['props'].get('tree', 0)} trees"))
    print(f"[park] wrote {out}")


if __name__ == "__main__":
    main()
