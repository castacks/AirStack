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

    # 1) facility surfaces + their markings
    for z in park["zones"]:
        k, r = z["kind"], z["rect"]
        if k == "lawn":
            _poly(ax, _rect_pts(r), facecolor=COL["lawn"], edgecolor="none",
                  zorder=1)
        elif k == "soccer":
            _poly(ax, _rect_pts(r), facecolor=COL["grass_pitch"],
                  edgecolor="none", zorder=2)
            cx, cy = z["centre"]
            markings(pk.soccer_markings(), cx, cy, lw=0.12)
            # goals at both ends
            L = pk.SOCCER["pitch"][0] / 2.0
            gw = pk.SOCCER["goal_w"] / 2.0
            for s in (-1.0, 1.0):
                ax.plot([cx + s * L, cx + s * L], [cy - gw, cy + gw],
                        color="#ffffff", lw=max(1.2, 0.5 * ppm), zorder=7)
        elif k == "basketball_compound":
            _poly(ax, _rect_pts(r), facecolor=COL["asphalt"],
                  edgecolor="none", zorder=2)
            for court in z["courts"]:
                cx, cy = court["centre"]
                markings(pk.basketball_markings(), cx, cy, lw=0.09)
        elif k == "tennis_block":
            _poly(ax, _rect_pts(r), facecolor=COL["tennis"],
                  edgecolor="none", zorder=2)
            for court in z["courts"]:
                cx, cy = court["centre"]
                cl, cwid = pk.TENNIS["court"]
                _poly(ax, _rect_pts((cx - cl / 2, cy - cwid / 2,
                                     cx + cl / 2, cy + cwid / 2)),
                      facecolor=COL["tennis_in"], edgecolor="none", zorder=3)
                markings(pk.tennis_markings(), cx, cy, lw=0.08)
        elif k == "playground":
            _poly(ax, _rect_pts(r), facecolor=COL["sand"], edgecolor="none",
                  zorder=2)
        elif k == "picnic":
            _poly(ax, _rect_pts(r), facecolor=COL["picnic"], edgecolor="none",
                  zorder=2)

    # 2) paths, under the props and over the surfaces
    for p in park["paths"]:
        pts = p["pts"]
        w = 3.0 if p["kind"] == "loop" else (2.2 if p["kind"] == "spine" else 2.0)
        ax.plot([q[0] for q in pts], [q[1] for q in pts], color=COL["path"],
                lw=max(1.0, w * ppm), solid_capstyle="round",
                solid_joinstyle="round", zorder=5)

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

    # 4) props
    for p in park["props"]:
        col, r, z = PROP.get(p["kind"], ("#ff00ff", 1.0, 9))
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
