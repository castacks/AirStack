#!/usr/bin/env python3
"""trace_site_plan.py — turn an aerial photo into a `layout/site_plan` spec.

    AirStack/.venv/bin/python scene_gen/tools/trace_site_plan.py \
        --image photos/disaster-city.png --mpp 0.1534 \
        --name disaster_city --out scene_gen/config/site_plans/disaster_city.yaml

Run it ONCE. The YAML it writes is the source of truth from then on — see the
module docstring of `layout/site_plan.py` for why the scene must not be a
function of a segmentation threshold.

THE PIPELINE, and what each stage is defending against:

    pavement mask     asphalt is the only surface in an aerial that is both
                      desaturated AND blue-shifted; concrete pads, bare earth
                      and dry grass all sit on the warm side of neutral, so
                      LAB b* separates them where brightness alone does not.

    blobs removed     a parking lot is pavement but is not a street. Anything
                      that survives an opening with a `--blob-r` disk is wider
                      than any road on the site, so it is taken out before
                      skeletonising — otherwise the lot contributes a bush of
                      false streets. Junction widenings are narrower than the
                      disk and survive, which is what keeps four-ways joined.

    skeleton -> graph standard 8-connected thinning, then node pixels (degree
                      != 2) clustered so a fat junction is ONE node.

    pruned            two passes. Short spurs go first; then leaves are pulled
                      iteratively until only the 2-connected core remains,
                      EXCEPT at the crop edge, where a degree-1 node is a road
                      leaving the photo and must be kept. Without the second
                      pass every rubble field and tree shadow that touched the
                      asphalt left a street stub poking into a block.

    widths            2x the median distance-transform value along each
                      centreline: the width of the pavement the line runs down,
                      measured rather than assumed.

    roles             the recovered faces are classified by what covers them,
                      so a block knows it is trees or concrete or a lot. The
                      guess is written with its evidence (`fractions:`) so a
                      wrong one is obvious and correctable by hand.
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml
from skimage.morphology import remove_small_objects, skeletonize

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from layout import site_plan as spl                                    # noqa: E402
from layout import suburb_net as sn                                    # noqa: E402
from tools.site_plan_png import draw as draw_plan                      # noqa: E402


# ---------------------------------------------------------------------------
# surface classification
# ---------------------------------------------------------------------------

def surface_classes(bgr):
    """Per-pixel surface label. Returns a dict of boolean masks.

    Thresholds were read off sampled patches of the reference aerial and are
    deliberately coarse: they decide what a BLOCK is mostly made of, a question
    that survives being wrong about individual pixels.
    """
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    B = lab[..., 2].astype(np.int16) - 128          # + warm, - cool
    H = hsv[..., 0].astype(np.int16) * 2            # OpenCV packs hue in 0-179
    S = hsv[..., 1].astype(np.int16)
    V = hsv[..., 2].astype(np.int16)

    # Vegetation on this aerial runs 60 deg (dry grass) to 112 deg (canopy);
    # asphalt sits at 225-234 deg, blue, which is why the upper bound matters —
    # at `H < 180` the road itself reads as green and the mask empties out.
    green = (H > 30) & (H < 165) & (S > 28)
    out = {}
    out["asphalt"] = (B < -1) & (S < 55) & (V > 70) & (V < 205) & ~green
    out["roof"] = (V > 232) & (S < 30)
    out["pad"] = (V > 190) & (S < 70) & (B >= -1) & ~out["roof"]
    out["tree"] = green & (V < 145)
    out["grass"] = green & (V >= 145) & ~out["pad"]
    known = np.zeros_like(out["asphalt"])
    for m in out.values():
        known |= m
    out["bare"] = ~known
    return out


def pavement_mask(bgr, min_area=3000):
    m = surface_classes(bgr)["asphalt"].astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, _disk(7))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, _disk(5))
    n, lb, st, _ = cv2.connectedComponentsWithStats(m, 8)
    keep = np.zeros_like(m)
    for i in range(1, n):
        if st[i, cv2.CC_STAT_AREA] >= min_area:
            keep[lb == i] = 1
    return keep


def _disk(r):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))


# ---------------------------------------------------------------------------
# skeleton -> graph
# ---------------------------------------------------------------------------

_NB = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def skeleton_graph(skel):
    """``(nodes, edges)`` from a thinned mask. Nodes are (x, y) float pixels."""
    h, w = skel.shape

    def nbrs(y, x):
        return [(y + dy, x + dx) for dy, dx in _NB
                if 0 <= y + dy < h and 0 <= x + dx < w and skel[y + dy, x + dx]]

    deg = np.zeros(skel.shape, np.uint8)
    ys, xs = np.nonzero(skel)
    for y, x in zip(ys, xs):
        deg[y, x] = len(nbrs(y, x))
    # A junction is several pixels of degree 3+ side by side; label them
    # together so the graph gains one node there, not four.
    nlab, lab = cv2.connectedComponents(
        (skel & (deg != 2)).astype(np.uint8), 8)
    nodes = {}
    for i in range(1, nlab):
        pts = np.argwhere(lab == i)
        nodes[i] = (float(pts[:, 1].mean()), float(pts[:, 0].mean()))

    edges, seen = [], set()
    for i in range(1, nlab):
        for (sy, sx) in map(tuple, np.argwhere(lab == i)):
            for (ny, nx) in nbrs(sy, sx):
                if lab[ny, nx] == i or (sy, sx, ny, nx) in seen:
                    continue
                path, prev, (py, px) = [(sx, sy), (nx, ny)], (sy, sx), (ny, nx)
                while lab[py, px] == 0:
                    nxt = [p for p in nbrs(py, px) if p != prev]
                    if not nxt:
                        break
                    step = next((p for p in nxt if lab[p] != 0), nxt[0])
                    prev, (py, px) = (py, px), step
                    path.append((px, py))
                    if lab[py, px] != 0:
                        break
                if lab[py, px] == 0:
                    continue
                seen.add((sy, sx, ny, nx))
                seen.add((py, px, prev[0], prev[1]))
                edges.append({"a": i, "b": int(lab[py, px]), "px": path})
    # SNAP EVERY END ONTO ITS JUNCTION'S CENTROID. A path starts and ends at
    # whichever PIXEL of the junction cluster it happened to leave from, and a
    # fat four-way is 20 px across — so four streets that meet in the photo
    # arrive at four points metres apart, the graph never joins them, and the
    # faces come back as one region-sized polygon with the streets dangling
    # inside it. One assignment is the whole fix.
    for e in edges:
        e["px"][0] = nodes[e["a"]]
        e["px"][-1] = nodes[e["b"]]
    return nodes, edges


def _plain(v):
    """numpy scalars out, python scalars in — `yaml.safe_dump` refuses the rest.

    Every number here has passed through OpenCV or numpy at some point, and a
    np.float32 that survives to the dump fails the whole write at the end of a
    minute of segmentation.
    """
    if isinstance(v, dict):
        return {k: _plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_plain(x) for x in v]
    if isinstance(v, np.generic):
        return v.item()
    return v


def _plen(p):
    return sum(math.dist(p[i], p[i + 1]) for i in range(len(p) - 1))


def prune(nodes, edges, shape, spur_px, border_px):
    """Spurs first, then leaves, keeping the ones that run off the photo."""
    h, w = shape

    def at_border(nid):
        x, y = nodes[nid]
        return (x < border_px or y < border_px
                or x > w - border_px or y > h - border_px)

    def degrees(es):
        d = {}
        for e in es:
            d[e["a"]] = d.get(e["a"], 0) + 1
            d[e["b"]] = d.get(e["b"], 0) + 1
        return d

    for _ in range(200):
        d = degrees(edges)
        drop = {i for i, e in enumerate(edges)
                if _plen(e["px"]) < spur_px
                and ((d.get(e["a"], 0) == 1 and not at_border(e["a"]))
                     or (d.get(e["b"], 0) == 1 and not at_border(e["b"])))}
        if not drop:
            break
        edges = [e for i, e in enumerate(edges) if i not in drop]
    # Now the 2-connected core. A leaf that is not at the crop edge is a
    # segmentation artefact however long it is: real streets on this kind of
    # site either close a loop or leave the photo.
    for _ in range(200):
        d = degrees(edges)
        drop = {i for i, e in enumerate(edges)
                if (d.get(e["a"], 0) == 1 and not at_border(e["a"]))
                or (d.get(e["b"], 0) == 1 and not at_border(e["b"]))}
        if not drop:
            break
        edges = [e for i, e in enumerate(edges) if i not in drop]
    used = {e["a"] for e in edges} | {e["b"] for e in edges}
    return {k: v for k, v in nodes.items() if k in used}, edges


def rdp(pts, eps):
    """Ramer-Douglas-Peucker, iterative so a long centreline cannot recurse
    past the interpreter's stack limit."""
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        ax, ay = pts[i]
        bx, by = pts[j]
        dx, dy = bx - ax, by - ay
        n = math.hypot(dx, dy) or 1e-9
        far, fd = None, eps
        for k in range(i + 1, j):
            px, py = pts[k]
            d = abs(dy * px - dx * py + bx * ay - by * ax) / n
            if d > fd:
                far, fd = k, d
        if far is not None:
            keep[far] = True
            stack.append((i, far))
            stack.append((far, j))
    return [p for p, k in zip(pts, keep) if k]


def measure_width(path, dist):
    """Pavement width along a centreline: 2x the median distance transform.

    Median, not mean: the ends of every centreline sit inside a junction blob
    where the transform reports the blob's radius, and a handful of those would
    drag a mean well over the width of the street itself.
    """
    v = [dist[int(round(y)), int(round(x))] for (x, y) in path
         if 0 <= int(round(y)) < dist.shape[0] and 0 <= int(round(x)) < dist.shape[1]]
    if not v:
        return 0.0
    v.sort()
    return 2.0 * v[len(v) // 2]


# ---------------------------------------------------------------------------
# blocks
# ---------------------------------------------------------------------------

def classify_blocks(blocks, classes, to_px, shape):
    """Label each block by the surface that covers most of it."""
    h, w = shape
    out = []
    for b in blocks:
        poly = np.array([to_px(p) for p in b["poly"]], np.int32)
        m = np.zeros((h, w), np.uint8)
        cv2.fillPoly(m, [poly], 1)
        # Pull in off the kerb: the inset polygon still grazes the asphalt, and
        # a thin ring of road around every block skews small ones to "parking".
        m = cv2.erode(m, _disk(6))
        n = int(m.sum())
        if n < 200:
            frac = {}
        else:
            frac = {k: round(float((v & (m > 0)).sum()) / n, 3)
                    for k, v in classes.items()}
        role = _role_of(frac)
        cx, cy = b["centroid"]
        out.append({"at": [round(cx, 1), round(cy, 1)], "role": role,
                    "area_m2": int(b["area"]), "fractions": frac})
    return out


def _role_of(frac):
    """Surface fractions -> a role name the scene passes can act on."""
    if not frac:
        return "grass"
    g = frac.get
    if g("asphalt", 0) > 0.45:
        return "parking"
    # 0.25, not a third: these lots are winter scrub, bare ground showing
    # through open canopy, and at 0.35 two obviously wooded blocks came back
    # `staging` on tree fractions of 0.28 and 0.35.
    if g("tree", 0) > 0.25:
        return "wooded"
    if g("pad", 0) + g("roof", 0) > 0.40:
        return "pad"
    if g("grass", 0) > 0.35:
        return "grass"
    if g("bare", 0) + g("asphalt", 0) > 0.40:
        return "staging"
    return "grass"


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--image", required=True)
    ap.add_argument("--mpp", type=float, required=True,
                    help="metres per pixel; see the spec's `source.calibration`")
    ap.add_argument("--name", default="site")
    ap.add_argument("--out", required=True, help="site-plan YAML to write")
    ap.add_argument("--preview", default="", help="overlay PNG (default: <out>.png)")
    ap.add_argument("--blob-r", type=int, default=70,
                    help="px: pavement wider than 2x this is a lot, not a road")
    ap.add_argument("--spur-px", type=int, default=90)
    ap.add_argument("--min-width-m", type=float, default=3.5,
                    help="pavement narrower than one lane is not a street")
    ap.add_argument("--rim-px", type=int, default=40,
                    help="px: width of the perimeter drive put back around a lot")
    ap.add_argument("--border-px", type=int, default=30,
                    help="px: a leaf this close to the edge is a road leaving")
    ap.add_argument("--rdp-px", type=float, default=3.0)
    ap.add_argument("--min-block-m2", type=float, default=150.0,
                    help="a recovered face under this is a sliver, not a block")
    ap.add_argument("--image-ref", default="",
                    help="how the spec should NAME the aerial (default: its "
                         "file name — the spec may not point outside the repo)")
    args = ap.parse_args()

    bgr = cv2.imread(args.image)
    if bgr is None:
        raise SystemExit(f"cannot read {args.image}")
    h, w = bgr.shape[:2]
    classes = surface_classes(bgr)
    pav = pavement_mask(bgr)

    lots = cv2.morphologyEx(pav, cv2.MORPH_OPEN, _disk(args.blob_r))
    # A LOT IS A BLOCK WHOSE PERIMETER DRIVE IS A ROAD, and both are the same
    # asphalt, so no threshold separates them. Deleting the lot outright also
    # deleted the drive around it, and with the drive gone nothing bounded that
    # third of the site: the faces there merged into their neighbours and a
    # 3,500 m² parking lot came back as no block at all. So the lot's RIM goes
    # back into the road mask. Its medial axis lands `rim_px`/2 inside the lot
    # edge, which is where a perimeter drive's centreline actually runs.
    rim = ((lots > 0) & (cv2.erode(lots, _disk(args.rim_px)) == 0))
    blobs = cv2.dilate(lots, _disk(3))
    roads = (((pav > 0) & (blobs == 0)) | rim).astype(np.uint8)
    roads = cv2.morphologyEx(roads, cv2.MORPH_CLOSE, _disk(8))
    roads = remove_small_objects(roads.astype(bool), 4000).astype(np.uint8)
    dist = cv2.distanceTransform(roads, cv2.DIST_L2, 5)

    nodes, edges = skeleton_graph(skeletonize(roads.astype(bool)))
    for e in edges:
        e["w_m"] = float(measure_width(e["px"], dist)) * args.mpp
    # WIDTH IS THE FILTER THAT SPUR LENGTH CANNOT BE. The tracks worn across a
    # rubble field read as asphalt and they close loops, so they survive both
    # pruning passes however obviously they are not streets — but they are 2-3 m
    # across, and no road on the site is. Dropping them before the pruning is
    # what leaves the pruning a clean grid to work on.
    narrow = [e for e in edges if e["w_m"] < args.min_width_m]
    edges = [e for e in edges if e["w_m"] >= args.min_width_m]
    nodes, edges = prune(nodes, edges, (h, w), args.spur_px, args.border_px)
    print(f"[trace] {len(nodes)} nodes, {len(edges)} streets, "
          f"{sum(_plen(e['px']) for e in edges) * args.mpp:.0f} m of road "
          f"({len(narrow)} runs dropped under {args.min_width_m} m)")

    # pixels (+y down, origin top-left) -> metres (+y north, origin centred)
    def to_m(p):
        return (round((p[0] - w / 2.0) * args.mpp, 2),
                round((h / 2.0 - p[1]) * args.mpp, 2))

    def to_px(p):
        return (int(round(p[0] / args.mpp + w / 2.0)),
                int(round(h / 2.0 - p[1] / args.mpp)))

    roads_out = []
    for e in edges:
        pts = rdp(e["px"], args.rdp_px)
        wm = e["w_m"]
        roads_out.append({"class": spl.class_for_width(wm),
                          "width_m": round(wm, 1),
                          "pts": [list(to_m(p)) for p in pts]})
    roads_out.sort(key=lambda r: -sum(1 for _ in r["pts"]))

    spec = {"name": args.name,
            "source": {"image": args.image_ref or os.path.basename(args.image),
                       "meters_per_pixel": args.mpp,
                       "image_px": [w, h],
                       "traced_by": "scene_gen/tools/trace_site_plan.py"},
            "region_m": [round(w * args.mpp, 1), round(h * args.mpp, 1)],
            "min_block_m2": args.min_block_m2,
            "roads": roads_out}

    # Build it through the real layout module, so the roles are attached to the
    # faces the SCENE will see rather than to a second, parallel recovery.
    net, blocks, info = spl.generate(*spec["region_m"], None,
                                     {"_spec": spec})
    spec["blocks"] = classify_blocks(blocks, classes, to_px, (h, w))
    for b, tag in zip(blocks, spec["blocks"]):
        b["role"] = tag["role"]
        # So the stats line below reports the open land honestly. `generate`
        # derives this from the role, but the roles did not exist when it ran.
        b["undeveloped"] = tag["role"] in spl.OPEN_ROLES

    with open(args.out, "w") as fh:
        fh.write(f"# Traced from {os.path.relpath(args.image)} by "
                 f"{os.path.basename(__file__)}. EDIT THIS FILE, not the tracer:\n"
                 f"# see layout/site_plan.py for why the trace is run once.\n")
        yaml.safe_dump(_plain(spec), fh, sort_keys=False, default_flow_style=None,
                       width=100)
    print(f"[trace] {len(blocks)} blocks -> {args.out}")
    print(sn.format_stats(sn.stats(net, blocks, info["region"])))
    # NOT beside the spec by default. The preview is a render OF THE AERIAL with
    # the plan on top, so it carries the same third-party imagery the spec
    # deliberately does not commit — and `config/` is no place for a 3 MB PNG.
    png = args.preview or os.path.join(
        os.path.dirname(_HERE), "_plans",
        os.path.splitext(os.path.basename(args.out))[0] + "_trace.png")
    os.makedirs(os.path.dirname(png) or ".", exist_ok=True)
    spec["_path"] = args.out
    print(f"[trace] preview -> {draw_plan(spec, net, blocks, png, base=bgr)}")


if __name__ == "__main__":
    main()
