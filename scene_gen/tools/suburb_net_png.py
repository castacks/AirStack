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
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from layout import suburb_net as sn                                        # noqa: E402
from detail import suburb_parcel as sp                                     # noqa: E402
from detail import modular_house as mh                                     # noqa: E402


def _kit_catalogue():
    """The modular kit as `(styles, sizes)`, the way `suburb_scene` builds it.

    THE PREVIEW HAS TO PLAT AGAINST THE SAME CATALOGUE AS THE SCENE, and until
    now it did not: `suburb_scene.generate_suburb_on_stage` derives
    `house_sizes` from `modular_catalogue` and hands it to `parcel_blocks`,
    while this tool passed only the preset's `suburb_parcel` block — which
    names no footprints — so every house here was sized from the NOMINAL
    `house_w_m`/`house_d_m` fallback (9-13 x 8-11 m) rather than from the kit's
    real 10-20 m. The tool's own comment already says the preview must parcel
    on the same terms as the scene; this closes the last place it did not.
    It is also a hard requirement for row housing, which resolves a mix of
    STYLE NAMES to the `size_index` the scene will place.

    Duplicated here rather than imported because `suburb_scene` imports `pxr`
    at module scope and this tool deliberately does not — `modular_house`
    imports it lazily, so the arithmetic is reachable without USD.
    """
    styles, sizes = [], []
    for style in mh.ORDER:
        cs = mh.footprint(mh.STYLES[style])
        xs = [i for i, _ in cs]
        ys = [j for _, j in cs]
        styles.append(style)
        sizes.append(((max(xs) - min(xs) + 1) * mh.CELL_M,
                      (max(ys) - min(ys) + 1) * mh.CELL_M))
    return styles, sizes


def _front_openings_fn(styles):
    """The same `front_openings` seam `suburb_scene` installs.

    `row_housing` aims each unit's footway at the style's REAL front door with
    it, and the plat breaks its front fence on it. Without it the walk lands on
    the middle of the elevation, which on the kit houses is a window.
    """
    def fn(size_index, house_w):
        spec = mh.STYLES[styles[size_index % len(styles)]]
        door_x, garage_x, _ = mh.front_anchors(spec)
        gaps = [(door_x, 1.2 / 2.0 + 0.6, "door")]
        if garage_x is not None:
            gaps.append((garage_x, 3.2 / 2.0 + 0.6, "drive"))
        return gaps
    return fn


def _preset_parcel_cfg():
    """The shipped preset's `suburb_parcel` block, or {} if it cannot be read.

    Read from the preset rather than duplicated here: a second copy of
    `house_gap_m` in a preview tool is a copy that goes stale the first time
    the real one is tuned.
    """
    import os as _os
    try:
        import yaml
    except ImportError:
        return {}
    for rel in ("config/presets/suburb_net.yaml",):
        path = _os.path.join(_os.path.dirname(_HERE), rel)
        if not _os.path.exists(path):
            continue
        try:
            raw = yaml.safe_load(open(path)) or {}
        except Exception:
            return {}
        node = raw
        for key in ("suburb_parcel",):
            if key in node:
                return dict(node[key] or {})
        for v in raw.values():
            if isinstance(v, dict) and isinstance(v.get("suburb_parcel"), dict):
                return dict(v["suburb_parcel"])
    return {}

def _plan_cars(seed, net, parcels):
    """`suburb_scene.build_cars` run for the plan, as drawable boxes.

    NEEDS `pxr` — `suburb_scene` imports it at module scope, which is why the
    rest of this tool does not touch it. `pip install usd-core` is enough; so
    is Isaac's own python. The flag is opt-in for exactly that reason.

    A PLAN PREVIEW, NOT THE BUILD. Two differences, both worth knowing before
    judging a picture:
      * `measure_usds` is off here, so footprints come from `fallback_sizes`
        and every car is the same 2.15 x 5.44 m box. The real run measures.
      * the modular kit's per-lot `plan` is attached inside
        `generate_suburb_on_stage`, so drives here are the PARCEL runs
        (`d["a"] -> d["b"]`), which is the same line to within the garage
        offset, and `existing` is empty — nothing is standing yet, so the
        keep-outs against trees, hydrants and street furniture do not bite.
        Junction, bulb, apron and structural keep-outs all do.
    """
    import sys as _sys
    _sys.path.insert(0, os.path.dirname(_HERE))
    try:
        import suburb_scene as ss
        import compile_disaster as cd
        import scene_generator as _sg
    except ImportError as e:
        print(f"  --cars needs the USD python bindings ({e}); "
              f"try `pip install usd-core`")
        return []
    cfg = cd.load_scene_config("suburb_net")
    cfg["measure_usds"] = False
    resolver = _sg._make_resolver(cfg)
    pools = ss.AssetPools(cfg)
    cars = ss.build_cars(cfg, resolver, net, parcels,
                         random.Random(seed + 7717), pools,
                         existing=(), info=None)
    out = []
    for q in cars:
        usd = q["usd"]
        fp = resolver.get(usd, "car", scale=pools.scale_of(usd),
                          axis_up=pools.axis_of(usd))
        yo = pools.yaw_of(usd)
        c, sn_ = abs(math.cos(math.radians(yo))), abs(math.sin(math.radians(yo)))
        ln = c * float(fp["sx"]) + sn_ * float(fp["sy"])
        wd = sn_ * float(fp["sx"]) + c * float(fp["sy"])
        if ln < wd:
            ln, wd = wd, ln
        head = math.radians(float(q["yaw_deg"]) - yo)
        x, y = float(q["x_m"]), float(q["y_m"])
        out.append({
            "x": x, "y": y, "head": head, "len": ln,
            "role": q.get("role", "kerb"),
            "corners": ss._obb_corners(x, y, math.cos(head), math.sin(head),
                                       ln / 2.0, wd / 2.0),
        })
    return out


CLASS_COLOR = {
    "arterial":   "#5a5a5a",
    "collector":  "#565656",
    "local":      "#525252",
    "cul_de_sac": "#4e4e4e",
}


def draw(net, blks, info, out_path, title="", show_nodes=True, parcels=None,
         window=None, lots=False, cars=None):
    """Plan of the whole crop, or of *window* — ``(x, y, radius)`` in metres.

    A cul-de-sac head is about 35 m across on a 1600 m crop, i.e. two percent
    of the picture: at that scale a house on the paving and a house beside it
    are the same three pixels. `window` re-frames the same draw on a square of
    ground so the turnaround can actually be judged, and everything keeps its
    TRUE WIDTH because the metres-to-points factor is taken from the window
    rather than from the region.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Circle

    x0, y0, x1, y1 = info["region"]
    if window:
        wx, wy, wr = float(window[0]), float(window[1]), float(window[2])
        x0, y0, x1, y1 = wx - wr, wy - wr, wx + wr, wy + wr
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
            # linewidth in points = metres * (points per metre). A POINT IS
            # 1/72 INCH, not one pixel: multiplying by `fig.dpi` drew every
            # carriageway and drive dpi/72 = 1.39x wider than it is, which on a
            # 9.1 m cul-de-sac is a metre and a half of asphalt that is not
            # there — invisible at crop scale and the whole question at a
            # turnaround, where the judgement is whether a drive apron meets
            # the kerb or overruns it.
            ppm = 72.0 * fig.get_size_inches()[0] / (x1 - x0)
            ax.plot(xs, ys, color=CLASS_COLOR[cls], lw=e.width_m * ppm,
                    solid_capstyle="butt", solid_joinstyle="round", zorder=2)

    # Cul-de-sac turnarounds, drawn at the code radius — over a disc of GROUND
    # out to the lot line. The block boundary at a bulb is the turnaround arc
    # plus `_arc_cap_bulbs`' verge, so the block fill stops 3 m short of the
    # kerb and the strip between them was coming out as bare axes background: a
    # black ring round every head, which reads as a defect and is not one. It
    # is the verge, and the verge is land.
    for e in net.edges.values():
        if e.street_type != "lollipop":
            continue
        tip = e.pts[-1]
        ax.add_patch(Circle(tip, sn.DEFAULTS["bulb_radius_m"] + 3.0,
                            facecolor="#47513c", edgecolor="none", zorder=1.1))
        ax.add_patch(Circle(tip, sn.DEFAULTS["bulb_radius_m"],
                            facecolor=CLASS_COLOR["cul_de_sac"],
                            edgecolor="none", zorder=3))

    # Detail, drawn above the roads: drives, then houses, then canopy. Order
    # matters — a tree overhanging a roof is what the plan should show, since
    # that is what the camera will see.
    if parcels:
        ppm = 72.0 * fig.get_size_inches()[0] / (x1 - x0)   # points per metre
        for p in parcels:
            # LOT LINES AND FENCES, off by default. At crop scale they are
            # noise; zoomed in they are the only way to see whether the side
            # lines round a bulb converge on its centre or run parallel past
            # it, which is the whole question a wedge lot answers.
            if lots:
                for h in p["houses"]:
                    lc = h.get("lot_corners")
                    if lc:
                        ax.add_patch(Polygon(lc, closed=True, fill=False,
                                             edgecolor="#c8b48a", lw=0.7,
                                             ls=(0, (4, 2)), zorder=4.5))
                    for seg in (h.get("fence_segs") or ()):
                        ax.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]],
                                color=("#e0603a" if seg[2] == "low"
                                       else "#7a5230"),
                                lw=1.6, zorder=6.5, solid_capstyle="round")
            for d in p["drives"]:
                ax.plot([d["a"][0], d["b"][0]], [d["a"][1], d["b"][1]],
                        color="#8d8d86", lw=d["w"] * ppm, solid_capstyle="butt",
                        zorder=5)
            # ROW-HOME CLUSTERS, drawn distinctly from the platted fabric —
            # the whole point of the overlay is that a court reads as SHARED
            # ground and a detached drive does not, so they must not come out
            # the same grey. Communal green first, then the court and its one
            # access drive, then the bays and the footway to every door.
            for cl in (p.get("clusters") or ()):
                for g in cl["greens"]:
                    ax.add_patch(Polygon(g, closed=True, facecolor="#3f6b34",
                                         edgecolor="none", alpha=0.85,
                                         zorder=4.2))
                for key, col in (("court", "#2a2d33"), ("drive", "#2a2d33")):
                    ax.add_patch(Polygon(cl[key], closed=True, facecolor=col,
                                         edgecolor="#8fa0b8", lw=0.8,
                                         zorder=5.2))
                for bay in cl["parking"]["bays"]:
                    bx, by = bay["centre"]
                    a = math.radians(bay["yaw_deg"])
                    ca, sa = math.cos(a), math.sin(a)
                    # 2.7 x 5.5 m, the bay's long axis ALONG its own heading.
                    pts = [(bx + ca * dx - sa * dy, by + sa * dx + ca * dy)
                           for (dx, dy) in ((-2.75, -1.35), (2.75, -1.35),
                                            (2.75, 1.35), (-2.75, 1.35))]
                    ax.add_patch(Polygon(pts, closed=True, fill=False,
                                         edgecolor="#d8dde4", lw=0.45,
                                         zorder=5.4))
                for (a, b) in cl["walks"]:
                    ax.plot([a[0], b[0]], [a[1], b[1]], color="#9c968c",
                            lw=cl["walk_w_m"] * ppm, solid_capstyle="butt",
                            zorder=5.3)
            for h in p["houses"]:
                # A ROW UNIT IS A DIFFERENT COLOUR ON PURPOSE. Judging the
                # morphology means seeing at a glance which houses are attached
                # and which are not; at crop scale a 12.9 m unit and a 15 m
                # detached house are the same handful of pixels otherwise.
                ax.add_patch(Polygon(h["corners"], closed=True,
                                     facecolor=("#e0b678" if h.get("row")
                                                else "#b9c4d6"),
                                     edgecolor=("#7a5a2c" if h.get("row")
                                                else "#5d6675"),
                                     lw=0.5, zorder=6))
            for t in p["trees"]:
                col = {"street": "#4f7a3f", "front": "#5c8a46",
                       "back": "#3f6b34"}[t["kind"]]
                ax.add_patch(Circle(t["c"], t["r"], facecolor=col,
                                    edgecolor="none", alpha=0.88, zorder=7))

    # PARKED CARS, above everything else, each drawn as its own measured box
    # with a tick out of its NOSE. Facing is the thing this overlay exists to
    # judge: a driveway car should point at its house and every car on one
    # kerb should point the same way as its neighbours.
    for c in (cars or ()):
        ax.add_patch(Polygon(c["corners"], closed=True,
                             facecolor=("#ffd166" if c["role"] == "driveway"
                                        else "#59c2ff"),
                             edgecolor="#141414", lw=0.4, zorder=9))
        hx, hy = math.cos(c["head"]), math.sin(c["head"])
        # MAGENTA, not red: `--lots` draws the low front fence in #e0603a and a
        # red tick beside one was read as a 46 m fence run in the first render.
        # Nothing else in this plan is magenta.
        ax.plot([c["x"], c["x"] + hx * (c["len"] / 2.0 + 1.3)],
                [c["y"], c["y"] + hy * (c["len"] / 2.0 + 1.3)],
                color="#ff2fd0", lw=1.4, zorder=10, solid_capstyle="butt")

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
    # `tight` measures every artist, including the park label and any street
    # that runs off the crop — harmless full-frame, but on a 140 m window it
    # grows the canvas to hold a text label a kilometre away and the ground
    # ends up a strip down one side. A window has already said what to show.
    fig.savefig(out_path, dpi=110,
                bbox_inches=(None if window else "tight"),
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
    ap.add_argument("--zoom", metavar="X,Y,R",
                    help="crop to a square of half-side R metres about (X, Y)")
    ap.add_argument("--zoom-bulb", type=int, metavar="N",
                    help="crop about the Nth cul-de-sac tip (0-based), so a "
                         "turnaround can be judged without hunting for its "
                         "coordinates first")
    ap.add_argument("--zoom-r", type=float, default=70.0,
                    help="half-side for --zoom-bulb (default 70 m)")
    ap.add_argument("--zoom-row", type=int, metavar="N",
                    help="crop about the Nth row-home court (0-based), so a "
                         "cluster can be judged without hunting for its "
                         "coordinates first")
    ap.add_argument("--lots", action="store_true",
                    help="draw lot rectangles and fence runs as well")
    ap.add_argument("--cars", action="store_true",
                    help="overlay parked cars, each at its measured size with "
                         "a tick out of its nose (needs the USD python "
                         "bindings: pip install usd-core)")
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
        # THE PREVIEW MUST PARCEL ON THE SAME TERMS AS THE SCENE, or it is not
        # a preview of anything. This called `parcel_blocks` with no config at
        # all, so it silently used the module DEFAULTS while
        # `suburb_scene.generate_suburb_on_stage` used the preset — and once the
        # scene started passing cul-de-sac keep-outs and a wider side yard, the
        # plan was drawing houses the build would never place. Both now derive
        # the keep-out discs from the same place: the bulbs `apply_ground` paves.
        pcfg = dict(_preset_parcel_cfg())
        # `--set` reaches the PARCEL pass as well as the net, so a knob like
        # `row_share` can be swept from the command line the way the net knobs
        # already can. Harmless in the other direction: `parcel_blocks` reads
        # only the keys it knows, so a net knob landing here is ignored.
        pcfg.update(cfg)
        bulb_r = float(sn.DEFAULTS["bulb_radius_m"])
        pcfg.setdefault("keepout_discs",
                        [(e.pts[-1], bulb_r + float(pcfg.get("bulb_margin_m", 3.0)))
                         for e in net.edges.values()
                         if e.street_type == "lollipop"])
        if pcfg.get("modular_houses") and pcfg.get("house_sizes") is None:
            _styles, _sizes = _kit_catalogue()
            pcfg["house_sizes"] = _sizes
            pcfg["house_styles"] = _styles
            pcfg["front_openings"] = _front_openings_fn(_styles)
        # Undeveloped blocks are open land, so they are not handed to the
        # parcelling pass at all — that is what the flag is for.
        parcels = sp.parcel_blocks([b for b in blks
                                    if not b.get("undeveloped")], rng, pcfg)
        ps = sp.stats(parcels)
        print("  %d houses, %d trees across %d/%d blocks (%.1f houses/block)"
              % (ps["houses"], ps["trees"], ps["blocks_built"], ps["blocks"],
                 ps["houses_per_built_block"]))
        # WHAT THE PLAT REFUSED, and why. `skew_yield` is the only one of these
        # that is a deliberate thinning rather than a failure to fit something
        # — lots held out of a block corner well off square, where the two
        # frontages' rectangles would have stood in each other and one lot's
        # fence run would have crossed the other's back garden. It is printed
        # beside the others so a run whose houses/block has dropped can be read
        # against it. See `suburb_parcel.junction_skew_clear_deg`.
        print("  refused: %d for house size, %d on a turnaround, %d in a "
              "cul-de-sac wedge, %d at a skewed block corner"
              % (ps["size_rejected"], ps["keepout_rejected"],
                 ps["wedge_yield"], ps.get("skew_yield", 0)))
        if ps.get("row_courts") or ps.get("row_rejected"):
            print("  row homes: %d unit(s) in %d court(s) on %d block(s), "
                  "%d bay(s); %d block(s) drew a row district and could not "
                  "fit one"
                  % (ps["row_units"], ps["row_courts"], ps["row_blocks"],
                     ps["row_bays"], ps["row_rejected"]))
            print("    mixes %s" % (dict(sorted(ps["row_mixes"].items())),))
            print("    styles %s" % (dict(sorted(ps["row_styles"].items())),))
            print("    palettes %s"
                  % (dict(sorted(ps["row_palettes"].items())),))

    cars = None
    if args.cars:
        if parcels is None:
            print("  --cars needs the detail pass; drop --no-detail")
        else:
            cars = _plan_cars(args.seed, net, parcels)
            n_dr = sum(1 for c in cars if c["role"] == "driveway")
            print("  %d cars: %d on driveways, %d at the kerb"
                  % (len(cars), n_dr, len(cars) - n_dr))

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
        if ps.get("row_courts"):
            title += (f"; {ps['row_units']} row homes (orange) in "
                      f"{ps['row_courts']} court(s) on {ps['row_blocks']} "
                      f"block(s), mixes "
                      f"{'/'.join(sorted(ps['row_mixes']))}")
    window = None
    if args.zoom:
        window = [float(v) for v in args.zoom.split(",")]
    elif args.zoom_row is not None:
        cls = [cl for p in (parcels or ()) for cl in (p.get("clusters") or ())]
        if not cls:
            print("  no row-home courts to zoom to")
        else:
            cls.sort(key=lambda cl: cl["parking"]["centre"])
            cl = cls[max(0, min(len(cls) - 1, args.zoom_row))]
            t = cl["parking"]["centre"]
            window = [t[0], t[1], args.zoom_r]
            print("  zoom on court %d/%d at (%.0f, %.0f) — %s mix, %s "
                  "palettes, %d units, %d bays"
                  % (args.zoom_row, len(cls), t[0], t[1], cl["mix"],
                     cl["palette_set"], cl["units"],
                     len(cl["parking"]["bays"])))
    elif args.zoom_bulb is not None:
        tips = [e.pts[-1] for e in net.edges.values()
                if e.street_type == "lollipop"]
        if not tips:
            print("  no cul-de-sacs to zoom to")
        else:
            tips.sort()
            t = tips[max(0, min(len(tips) - 1, args.zoom_bulb))]
            window = [t[0], t[1], args.zoom_r]
            print("  zoom on bulb %d/%d at (%.0f, %.0f)"
                  % (args.zoom_bulb, len(tips), t[0], t[1]))
    if cars:
        n_dr = sum(1 for c in cars if c["role"] == "driveway")
        title += (f"\n{len(cars)} cars: {n_dr} driveway (yellow), "
                  f"{len(cars) - n_dr} kerb (blue); magenta tick = nose")
    draw(net, blks, info, out, title=title, show_nodes=not args.no_nodes,
         parcels=parcels, window=window, lots=args.lots, cars=cars)
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
