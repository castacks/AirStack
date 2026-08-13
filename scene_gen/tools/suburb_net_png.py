"""suburb_net_png.py — top-down plan of a `suburb_net` layout.

    python3 tools/suburb_net_png.py --seed 3 --out /tmp/suburb.png

Draws the network the way it will be built: each street as a stroked polyline
at its true kerb-to-kerb width, blocks as the polygon faces between them. No
USD, no assets — this is the iteration loop for judging whether the streets
merge sensibly, which is a question about the plan and not about materials.

Roads are drawn with round joins and butt caps so a junction shows exactly the
geometry the graph has: if two streets do not actually meet, the picture shows
the gap rather than papering over it with a patch.
"""

import argparse
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

import suburb_net as sn                                        # noqa: E402
import suburb_parcel as sp                                     # noqa: E402

CLASS_COLOR = {
    "arterial":   "#5a5a5a",
    "collector":  "#565656",
    "local":      "#525252",
    "cul_de_sac": "#4e4e4e",
}


def draw(net, blks, info, out_path, title="", show_nodes=True, parcels=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Circle

    x0, y0, x1, y1 = info["region"]
    fig, ax = plt.subplots(figsize=(13, 13 * (y1 - y0) / (x1 - x0)))
    ax.set_facecolor("#2f2f2d")

    # Blocks first: the land between the streets.
    for b in blks:
        ax.add_patch(Polygon(b["poly"], closed=True, facecolor="#47513c",
                             edgecolor="#3a412f", lw=0.6, zorder=1))

    # Streets, widest class first so narrow ones read on top at a junction.
    # "boundary" is the crop edge, not pavement -- never drawn.
    order = ["arterial", "collector", "local", "cul_de_sac"]
    for cls in order:
        for e in net.edges.values():
            if e.road_class != cls:
                continue
            xs = [p[0] for p in e.pts]
            ys = [p[1] for p in e.pts]
            # linewidth in points = metres * (points per metre)
            ppm = fig.dpi * fig.get_size_inches()[0] / (x1 - x0)
            ax.plot(xs, ys, color=CLASS_COLOR[cls], lw=e.width_m * ppm,
                    solid_capstyle="butt", solid_joinstyle="round", zorder=2)

    # Cul-de-sac turnarounds, drawn at the code radius.
    for e in net.edges.values():
        if e.street_type != "lollipop":
            continue
        tip = e.pts[-1]
        ax.add_patch(Circle(tip, sn.DEFAULTS["bulb_radius_m"],
                            facecolor=CLASS_COLOR["cul_de_sac"],
                            edgecolor="none", zorder=3))

    # Detail, drawn above the roads: drives, then houses, then canopy. Order
    # matters — a tree overhanging a roof is what the plan should show, since
    # that is what the camera will see.
    if parcels:
        ppm = fig.dpi * fig.get_size_inches()[0] / (x1 - x0)
        for p in parcels:
            for d in p["drives"]:
                ax.plot([d["a"][0], d["b"][0]], [d["a"][1], d["b"][1]],
                        color="#8d8d86", lw=d["w"] * ppm, solid_capstyle="butt",
                        zorder=5)
            for h in p["houses"]:
                ax.add_patch(Polygon(h["corners"], closed=True,
                                     facecolor="#b9c4d6", edgecolor="#5d6675",
                                     lw=0.5, zorder=6))
            for t in p["trees"]:
                col = {"street": "#4f7a3f", "front": "#5c8a46",
                       "back": "#3f6b34"}[t["kind"]]
                ax.add_patch(Circle(t["c"], t["r"], facecolor=col,
                                    edgecolor="none", alpha=0.88, zorder=7))

    if show_nodes:
        for n in net.nodes.values():
            if n.road_degree(net) <= 1:
                continue
            col = "#d08a3a" if n.road_degree(net) >= 4 else "#7fb2e5"
            ax.plot([n.p[0]], [n.p[1]], marker="o", ms=2.6, color=col,
                    zorder=8, lw=0)

    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.axis("off")
    if title:
        ax.set_title(title, color="#dddddd", fontsize=11)
    fig.patch.set_facecolor("#1e1e1e")
    fig.savefig(out_path, dpi=110, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--width", type=float, default=1600.0)
    ap.add_argument("--height", type=float, default=1200.0)
    ap.add_argument("--out", default="_plans/suburb_net.png")
    ap.add_argument("--no-nodes", action="store_true")
    ap.add_argument("--no-detail", action="store_true",
                    help="streets and blocks only, no houses/trees")
    ap.add_argument("--set", action="append", default=[],
                    metavar="KEY=VALUE", help="override a DEFAULTS knob")
    args = ap.parse_args()

    cfg = {}
    for kv in args.set:
        k, _, v = kv.partition("=")
        try:
            cfg[k] = float(v) if "." in v or v.isdigit() else eval(v)
        except Exception:
            cfg[k] = v

    rng = random.Random(args.seed)
    net, blks, info = sn.generate(args.width, args.height, rng, cfg)
    s = sn.stats(net, blks, info["region"])
    print(f"[suburb_net] seed {args.seed}  {args.width:.0f} x {args.height:.0f} m")
    print(f"  collectors {info['collectors']}  connectors {info['connectors']}"
          f"  loops {info['loops']}  lollipops {info['lollipops']}")
    print(sn.format_stats(s))

    parcels = None
    if not args.no_detail:
        parcels = sp.parcel_blocks(blks, rng)
        ps = sp.stats(parcels)
        print("  %d houses, %d trees across %d/%d blocks (%.1f houses/block)"
              % (ps["houses"], ps["trees"], ps["blocks_built"], ps["blocks"],
                 ps["houses_per_built_block"]))

    out = args.out
    if not os.path.isabs(out):
        out = os.path.join(os.path.dirname(_HERE), out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    title = (f"suburb_net seed {args.seed} — {s['blocks']} blocks, "
             f"{s['dead_end_pct']:.0f}% dead ends, "
             f"{s['three_way_pct']:.0f}% three-way, "
             f"{s['km_per_km2']:.1f} km/km²")
    if parcels:
        ps = sp.stats(parcels)
        title += f"\n{ps['houses']} houses, {ps['trees']} trees"
    draw(net, blks, info, out, title=title, show_nodes=not args.no_nodes,
         parcels=parcels)
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
