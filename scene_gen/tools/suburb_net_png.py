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

    # Blocks first: the land between the streets. A block flagged
    # `undeveloped` was deliberately never subdivided — woodland, a drainage
    # reserve, a phase nobody built — so it is drawn as rough open ground with
    # a dashed edge and gets no houses. Distinct on purpose: a sparse plan and
    # a plan whose subdivision pass simply gave up look identical otherwise,
    # and only one of those is a bug. An undeveloped parcel that TOUCHED THE
    # PARK is not in this list at all: it merged into the park's own face, so it
    # is drawn below with the open space rather than hatched next to it.
    for b in blks:
        if b.get("undeveloped"):
            ax.add_patch(Polygon(b["poly"], closed=True, facecolor="#3d4a33",
                                 edgecolor="#6b7a52", lw=0.9, ls=(0, (5, 3)),
                                 hatch="//", zorder=1))
        else:
            ax.add_patch(Polygon(b["poly"], closed=True, facecolor="#47513c",
                                 edgecolor="#3a412f", lw=0.6, zorder=1))

    # The park reserve, under the streets it was laid before. The pale band is
    # the padding no street may enter; the bright green is the OPEN SPACE, and
    # it is drawn as ONE SHAPE — `park["poly"]`, the reserve plus every
    # undeveloped parcel that touched it, with no frame street between them.
    # That is the whole point of the merge, so the plan has to show it as one
    # thing: two shapes butted together would look exactly like the thing this
    # replaced. If a carriageway is drawn over the green or the pale band, the
    # reserve was not respected and the picture says so rather than hiding it.
    park = info.get("park")
    if park:
        kx0, ky0, kx1, ky1 = park["reserve"]
        ax.add_patch(Polygon([(kx0, ky0), (kx1, ky0), (kx1, ky1), (kx0, ky1)],
                             closed=True, facecolor="#3c5a34",
                             edgecolor="#6f9c5e", lw=0.8, ls=(0, (4, 3)),
                             zorder=1.2))
        px0, py0, px1, py1 = park["rect"]
        open_space = park.get("poly") or [(px0, py0), (px1, py0),
                                          (px1, py1), (px0, py1)]
        ax.add_patch(Polygon(open_space, closed=True, facecolor="#2e7d32",
                             edgecolor="#8fd694", lw=1.4, zorder=1.3))
        # The label sits on the RESERVE and reports the reserve, because that
        # rectangle is still what the park's own content is built into; the
        # merged land is open ground the park runs out into.
        note = "PARK\n%.0f x %.0f m" % (px1 - px0, py1 - py0)
        if park.get("merged"):
            note += "\n+%d open parcels" % park["merged"]
        ax.text(0.5 * (px0 + px1), 0.5 * (py0 + py1), note,
                color="#cfe9cf", fontsize=8, ha="center", va="center",
                zorder=1.35)
        # Each entrance: the junction on the frame street, a line in to the
        # reserve edge, and the gate point on the park boundary it aligns to.
        for en in park["entrances"]:
            p, g, sp = en["p"], en["gate"], en["street_p"]
            ax.plot([sp[0], g[0]], [sp[1], g[1]], color="#ffd166", lw=1.3,
                    zorder=9, solid_capstyle="round")
            ax.plot([p[0]], [p[1]], marker="o", ms=4.5, color="#ffd166",
                    zorder=9, lw=0)
            ax.plot([g[0]], [g[1]], marker="s", ms=4.0, color="#fff3c4",
                    markeredgecolor="#7a5c00", markeredgewidth=0.5, zorder=9,
                    lw=0)

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
    park = info.get("park")
    if park:
        print("  park %.0f x %.0f m at (%.0f, %.0f), %.0f m pad — "
              "%d entrances on %d sides (%s), %d approach streets"
              % (park["size"][0], park["size"][1], park["center"][0],
                 park["center"][1], park["pad_m"], len(park["entrances"]),
                 len(park["sides"]), "".join(park["sides"]),
                 park["approaches"]))
        # The published extent, which is only the reserve when nothing merged.
        poly = park.get("poly")
        if poly:
            print("  open space %.1f ha over %d parcels merged in (reserve is "
                  "%.1f ha of it)"
                  % (abs(sn.polygon_area(poly)) / 10000.0, park["merged"],
                     park["size"][0] * park["size"][1] / 10000.0))
    print(sn.format_stats(s))
    # BLOCK SHAPE, next to the morphology it trades against. `rectangularity`
    # is area over the block's own minimum-area bounding rectangle: 1.0 for a
    # rectangle at any angle, 0.5 for any triangle. The share under 0.62 is the
    # number the shape term in `_best_cut` exists to hold down — it ran at
    # 47.5% before that term and about 23% after, so a run that drifts back up
    # says the scoring stopped biting rather than that the seed was unlucky.
    rects = [sn.rectangularity(b["poly"]) for b in blks]
    if rects:
        print("  block shape  %5.2f mean rectangularity, %.0f%% under 0.62 "
              "(triangular)"
              % (sum(rects) / len(rects),
                 100.0 * sum(1 for r in rects if r < 0.62) / len(rects)))

    parcels = None
    if not args.no_detail:
        # Undeveloped blocks are open land, so they are not handed to the
        # parcelling pass at all — that is what the flag is for.
        parcels = sp.parcel_blocks([b for b in blks
                                    if not b.get("undeveloped")], rng)
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
             f"{s['km_per_km2']:.1f} km/km²"
             f"\n{s['undeveloped_pct']:.0f}% undeveloped (hatched), "
             f"{100.0 * sum(1 for r in rects if r < 0.62) / max(len(rects), 1):.0f}%"
             f" of blocks triangular")
    if park:
        ha = (abs(sn.polygon_area(park["poly"])) / 10000.0 if park.get("poly")
              else park["size"][0] * park["size"][1] / 10000.0)
        title += (f"\npark {park['size'][0]:.0f} x {park['size'][1]:.0f} m "
                  f"+ {park['merged']} merged parcels = {ha:.1f} ha open, "
                  f"{len(park['entrances'])} entrances on "
                  f"{len(park['sides'])} sides")
    if parcels:
        ps = sp.stats(parcels)
        title += f"\n{ps['houses']} houses, {ps['trees']} trees"
    draw(net, blks, info, out, title=title, show_nodes=not args.no_nodes,
         parcels=parcels)
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
