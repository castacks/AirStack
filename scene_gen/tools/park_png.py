"""park_png.py — detailed top-down plan of a `suburb_park` layout.

    python3 tools/park_png.py --seed 3 --out _plans/park.png
    python3 tools/park_png.py --seed 3 --net --out _plans/park_net.png

Draws the park as SURFACES with their real materials and real line markings,
not as labelled boxes: the basketball compound is a painted slab inside a
chain-link run, the playground is sand, the pitch is mown grass with its
penalty areas on it. That is the point of the drawing — a box labelled
"basketball" tells you nothing about whether four courts actually fit, and the
markings are the check that they do.

`--net` LAYS THE STREETS FIRST. The refuge parking lot is the one facility
sited from outside the park — it goes on the entrance nearest the courts, and
its apron has to reach that entrance's kerb — so a drawing made with invented
entrances cannot show whether it is right. With `--net` this runs
`suburb_net.generate` exactly as `suburb_scene.generate_suburb_on_stage` does
(same seed offset, same rng, park-local translation of `info["park"]
["entrances"]`), so the plan is the one the scene will build. It costs a few
seconds of street generation; without it the module's own synthetic south
entrance is used and the rest of the park is unchanged.
"""

import argparse
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from detail import suburb_park as pk                                       # noqa: E402
from layout import suburb_net as sn                                        # noqa: E402

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
    # The refuge lot. Deliberately NOT the court asphalt: the two are the same
    # material in the scene, and on the plan the whole question is whether the
    # lot has landed on a court, which two identical greys would hide.
    "parking":     "#33393f",
    "apron":       "#3d444b",   # the drive across the verge, a shade lighter
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
    "bike_rack":       ("#5c7d99", 1.0, 9),
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
    # THE APRON LIVES OUTSIDE THE PARK. It crosses the verge to the frame
    # street's kerb, `park_pad_m` beyond the rect, so a frame set to the rect
    # clips off the half of it that answers the question the drawing is for —
    # does the lot actually meet the street. Widened HERE, before the figure,
    # so `ppm` (and every line width derived from it) is still to scale.
    lot = pk.parking_info(park)
    if lot and lot["apron"]:
        px = [q[0] for q in lot["apron"]] + [x0, x1]
        py = [q[1] for q in lot["apron"]] + [y0, y1]
        x0, x1 = min(px) - 4.0, max(px) + 4.0
        y0, y1 = min(py) - 4.0, max(py) + 4.0
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
        elif k == "parking":
            # Apron UNDER the slab, so where they meet reads as one surface
            # rather than as two rectangles with a seam.
            if lot and lot["apron"]:
                _poly(ax, lot["apron"], facecolor=COL["apron"],
                      edgecolor="none", zorder=1)
            _poly(ax, z["corners"], facecolor=COL["parking"],
                  edgecolor="none", zorder=2)
            markings(z.get("lines", []), cx, cy, yaw, lw=0.10)
            # A tick per bay, pointing the way a nosed-in car faces. This is
            # the check that the SCHEDULE the survivors/cars pass consumes
            # agrees with the paint — the lines come from the layout and the
            # bays from `parking_info`, and if the two disagreed it would show
            # here as ticks straddling a stall line.
            for b in (lot or {}).get("bays", ()):
                a = math.radians(b["yaw_deg"])
                bx, by = b["centre"]
                ax.plot([bx - math.cos(a) * 1.6, bx + math.cos(a) * 1.6],
                        [by - math.sin(a) * 1.6, by + math.sin(a) * 1.6],
                        color="#7f9bb5", lw=max(0.5, 0.10 * ppm),
                        solid_capstyle="butt", zorder=7)

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

    if lot and lot["apron"]:
        # Where the apron meets the kerb — the point a car turns in at.
        ax.plot([lot["entrance"][0]], [lot["entrance"][1]], marker="v",
                ms=7, color="#e8d24a", zorder=11)
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


def _net_entrances(seed, region_m):
    """The real park reserve and its entrances, in PARK-LOCAL coords.

    Mirrors `suburb_scene.generate_suburb_on_stage`'s park block exactly — same
    `random.Random(seed + 7717)`, same `sn.generate`, same recentring on the
    reserve — so `--net` draws the park the scene will actually build rather
    than one planned against invented entrances.
    """
    rng = random.Random(seed + 7717)
    _net, _blks, info = sn.generate(float(region_m[0]), float(region_m[1]),
                                    rng, {})
    pinfo = info.get("park")
    if not pinfo:
        return None, None, None
    px0, py0, px1, py1 = pinfo["rect"]
    dx, dy = (px0 + px1) / 2.0, (py0 + py1) / 2.0
    ents = [{"gate": (e["gate"][0] - dx, e["gate"][1] - dy),
             "p": (e["p"][0] - dx, e["p"][1] - dy),
             "dir": e["dir"], "side": e["side"]} for e in pinfo["entrances"]]
    return [px1 - px0, py1 - py0], ents, rng


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--width", type=float, default=None)
    ap.add_argument("--height", type=float, default=None)
    ap.add_argument("--out", default="_plans/park.png")
    ap.add_argument("--net", action="store_true",
                    help="lay the streets first and use the real entrances")
    ap.add_argument("--net-region", type=float, nargs=2,
                    default=[1600.0, 1200.0], metavar=("W", "H"))
    args = ap.parse_args()

    cfg = {}
    rng = random.Random(args.seed)
    if args.net:
        size, ents, rng = _net_entrances(args.seed, args.net_region)
        if ents is None:
            raise SystemExit("[park] suburb_net reserved no park")
        cfg["region_m"] = size
        cfg["entrances"] = ents
        print(f"[park] streets laid: {len(ents)} entrances, sides "
              f"{sorted({e['side'] for e in ents})}")
    if args.width and args.height:
        cfg["region_m"] = [args.width, args.height]
    park = pk.plan(rng, cfg)
    s = pk.stats(park)
    W = park["region"][2] - park["region"][0]
    H = park["region"][3] - park["region"][1]
    print(f"[park] {W:.0f} x {H:.0f} m ({W * H / 10000:.1f} ha)  seed {args.seed}")
    print(f"[park] zones: {s['zones']}")
    print(f"[park] props: {s['props']}")
    print(f"[park] {s['fence_panels']} fence panels, {s['paths']} paths")
    # THE CHECKS THAT MATTER. A plan that looks fine and routes a path over the
    # tennis block is not fine, and at this scale the crossing is a few pixels.
    bad = pk.check(park)
    print(f"[park] path-facility intersections: {bad}"
          + ("  <-- BROKEN" if bad else "  OK"))
    # The stricter one. Crossing a court is the outright bug; running along the
    # fence with no grass between is the one that makes the drawing look wrong,
    # and the drawing is the only place it shows.
    graze = pk.check_pad(park)
    pads = sorted({round(z.get("pad_m", 0.0), 1) for z in park["zones"]})
    print(f"[park] padding breaches (band {pads} m off the path EDGE): {graze}"
          + ("  <-- BROKEN" if graze else "  OK"))
    # AND THE SAME QUESTION FOR THE TREES, which the two above do not ask: they
    # measure PATHS. A trunk keep-out passes at 4 m and still hangs a 12.71 m
    # Black_Oak crown 8 m over the baseline, so the figure printed is the CROWN
    # edge — trunk distance less the widest radius the pool can supply — and it
    # is the only one here that has to be positive rather than zero.
    tb, tcrown = pk.check_tree_clear(park)
    ntree = s["props"].get("tree", 0)
    print(f"[park] tree keep-out breaches: {tb}"
          + ("  <-- BROKEN" if tb else "  OK")
          + f"; {ntree} trees, nearest CROWN EDGE to a court "
          + ("n/a" if tcrown is None else f"{tcrown:+.2f} m")
          + ("" if tcrown is None
             else ("  OK" if tcrown > 0 else "  <-- BROKEN")))
    # THE REFUGE LOT. `check`/`check_pad`/`check_reach` measure paths against
    # facilities and cover the lot for free, because it is a zone like any
    # other; what they do NOT measure is facility-on-facility overlap, which is
    # the one thing the lot can get wrong that the courts cannot — it is sited
    # against a street rather than by `_place`.
    bays, over, off, alen = pk.check_parking(park)
    lot = pk.parking_info(park)
    if lot:
        print(f"[park] refuge lot: {lot['w']:.0f} x {lot['d']:.0f} m, "
              f"{lot['rows']} rows x {lot['bays_per_row']} = {bays} bays, "
              f"yaw {lot['yaw_deg']:.0f} deg, apron {alen:.1f} m to the kerb "
              f"at ({lot['entrance'][0]:.0f}, {lot['entrance'][1]:.0f})")
        print(f"[park] lot overlapping other facilities: {over}"
              + ("  <-- BROKEN" if over else "  OK")
              + f"; bays off the slab: {off}"
              + ("  <-- BROKEN" if off else "  OK"))
    else:
        print("[park] refuge lot: NONE  <-- BROKEN unless `parking` is off")
    # And the check that the padding did not simply strand everything: a band
    # nothing may enter is a facility nobody can walk to.
    got, tot = pk.check_reach(park)
    print(f"[park] facilities with a path on their boundary: {got}/{tot}"
          + ("  OK" if got == tot else "  <-- BROKEN"))
    # THE TWO THAT SAY THE PATHS ARE A NETWORK. Everything above can pass on a
    # heap of disconnected polylines — and did: before the paths were built as
    # a graph, seed 3 scored 0 crossings, 0 breaches and 8/8 reachable with its
    # twenty paths in twenty separate components and thirteen ends stopping
    # dead in the grass. On the plan those are the short stubs coming out of a
    # court and connecting to nothing.
    orph = pk.check_orphans(park)
    print(f"[park] path ends leading nowhere: {orph}"
          + ("  <-- BROKEN" if orph else "  OK"))
    comp = pk.check_connected(park)
    print(f"[park] connected components of the path network: {comp}"
          + ("  OK" if comp == 1 else "  <-- BROKEN"))
    # And the one the de-duplication pass used to be for. It is not zero on
    # every seed: where the tour's return leg runs beside its outbound one the
    # spine still doubles itself, which is a tour problem, not a graph one.
    print(f"[park] doubled paving (segment pairs off a junction):"
          f" {pk.check_doubled(park)}")
    print(f"[park] paths crossing themselves: {pk.check_self_crossings(park)}")

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
