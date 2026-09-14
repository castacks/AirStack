#!/usr/bin/env python3
"""site_plan_png.py — draw a committed site plan, over its own aerial.

    AirStack/.venv/bin/python scene_gen/tools/site_plan_png.py \
        --plan disaster_city --out scene_gen/_plans/disaster_city.png

This is the loop for EDITING the spec: `trace_site_plan.py` runs the
segmentation once, and from then on a street is moved or a role corrected by
hand in the YAML — at which point the only question is whether the plan still
matches the photo, which is what this answers. It reads nothing but the spec,
so it shows exactly the geometry the scene will be built from.

Roles are drawn as a tint over the aerial, so a mislabelled block is obvious:
the block that reads as trees in the photo but is tinted `staging` is the one
to go and fix.
"""

import argparse
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from layout import site_plan as spl                                    # noqa: E402
from layout import suburb_net as sn                                    # noqa: E402

ROLE_BGR = {"parking": (200, 90, 40), "wooded": (40, 140, 40),
            "pad": (110, 190, 230), "grass": (60, 200, 120),
            "staging": (60, 160, 220), "rubble": (40, 60, 190),
            "collapsed": (30, 30, 160), "water": (190, 130, 40),
            "_untagged": (140, 140, 140)}

# Features are drawn as OUTLINES over the zone tint, never filled: the zone is
# the ground and the feature is what stands on it, and filling both makes the
# second one look like a hole in the first. Buildings get the heaviest stroke
# because they are the thing a reader is looking for.
FEATURE_BGR = {"building": (255, 255, 255), "rubble": (60, 60, 255),
               "wreck": (0, 140, 255), "pit": (90, 200, 255),
               "vehicle": (255, 230, 0), "container": (255, 160, 0),
               "mast": (255, 0, 220), "debris": (120, 120, 255),
             "tower": (200, 0, 255), "tank": (0, 200, 255),
             "vessel": (0, 235, 235), "pipe_rig": (120, 220, 255),
             "fence": (170, 255, 170), "sign": (255, 255, 120),
             "tree": (60, 180, 60)}
FEATURE_W = {"building": 3, "rubble": 3}


def projector(spec):
    """``(to_px, (w, h))`` — metres (+y north, centred) back to image pixels."""
    mpp = float(spec.get("source", {}).get("meters_per_pixel", 0.15))
    px = spec.get("source", {}).get("image_px")
    if px:
        w, h = int(px[0]), int(px[1])
    else:
        rw, rh = spl.region_of(spec)
        w, h = int(round(rw / mpp)), int(round(rh / mpp))

    def to_px(p):
        return (int(round(p[0] / mpp + w / 2.0)),
                int(round(h / 2.0 - p[1] / mpp)))

    return to_px, (w, h), mpp


def find_aerial(spec):
    """Locate the source aerial, or None.

    THE IMAGE IS NOT IN THE REPO and the spec must not point outside it —
    `coasei/CLAUDE.md`: no relative path may escape a repo root, and a
    third-party aerial is not ours to commit anyway. So the spec records only a
    FILE NAME and it is searched for, in the order a person would look:
    where they are standing, beside the spec, then `$SITE_PLAN_AERIALS`.
    Missing, the plan still draws — on a blank sheet, which is honest about
    what can no longer be checked.
    """
    img = (spec.get("source") or {}).get("image")
    if not img:
        return None
    roots = [os.curdir, os.path.dirname(spec.get("_path", ".")),
             os.environ.get("SITE_PLAN_AERIALS", "")]
    for root in roots:
        if not root:
            continue
        path = img if os.path.isabs(img) else os.path.join(root, img)
        if os.path.isfile(path):
            return path
    return None


def base_image(spec):
    """The aerial the plan was traced from, or a blank canvas."""
    _to_px, (w, h), _mpp = projector(spec)
    path = find_aerial(spec)
    if path:
        bgr = cv2.imread(path)
        if bgr is not None:
            return cv2.resize(bgr, (w, h)) if bgr.shape[:2] != (h, w) else bgr
    print(f"[site_plan_png] aerial "
          f"{(spec.get('source') or {}).get('image')!r} not found "
          f"(set SITE_PLAN_AERIALS); drawing on blank")
    return np.full((h, w, 3), 235, np.uint8)


ROW_BAND_M = 25.0   # vertical grouping for reading-order numbering


def reading_order(items, key):
    """Indices of *items* in reading order: top row left-to-right, then down.

    A NUMBER HAS TO SURVIVE A RE-TRACE. The obvious numbering is the order the
    blocks come out of the graph traversal, and it is useless to refer to:
    it depends on which half-edge the walk happened to start from, so adding
    one street renumbers the site and "block 7" means something else than it
    did yesterday. Sorting by position instead means a number moves only if the
    thing itself moves. Rows are banded because two blocks whose centroids
    differ by 2 m are on the same row to a reader and would otherwise be
    ordered by that 2 m.
    """
    def sort_key(i):
        x, y = key(items[i])
        return (-round(y / ROW_BAND_M), x)
    return sorted(range(len(items)), key=sort_key)


def _badge(img, txt, at, radius, fill):
    """A numbered disc. Drawn rather than written because the number has to be
    findable at a glance over an aerial that is every brightness at once, and
    an outlined glyph is not."""
    cx, cy = int(at[0]), int(at[1])
    cv2.circle(img, (cx, cy), radius + 1, (0, 0, 0), -1, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), radius, fill, -1, cv2.LINE_AA)
    sc = 0.44 if radius < 13 else 0.55
    # INK CHOSEN AGAINST THE DISC, not fixed white. The feature badges take
    # their fill from the kind's legend colour, and `building` is white — a
    # white numeral on it is invisible, which is how 14 of the 27 features
    # ended up unlabelled on the first pass.
    lum = 0.114 * fill[0] + 0.587 * fill[1] + 0.299 * fill[2]
    ink = (20, 20, 20) if lum > 140 else (255, 255, 255)
    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, sc, 2)
    cv2.putText(img, txt, (cx - tw // 2, cy + th // 2),
                cv2.FONT_HERSHEY_SIMPLEX, sc, ink, 2, cv2.LINE_AA)


def draw(spec, net, blocks, out_path, labels=True, base=None, features=None):
    """*base* is the already-loaded aerial, for a caller (the tracer) that has
    it in hand — searching for it again by file name would only fail."""
    to_px, _wh, mpp = projector(spec)
    vis = base_image(spec) if base is None else base.copy()
    tint = vis.copy()
    for b in blocks:
        poly = np.array([to_px(p) for p in b["poly"]], np.int32)
        cv2.fillPoly(tint, [poly], ROLE_BGR.get(b.get("role"), (140, 140, 140)))
    vis = cv2.addWeighted(tint, 0.35, vis, 0.65, 0)
    for e in net.edges.values():
        if e.road_class == "boundary":
            continue
        px = np.array([to_px(p) for p in e.pts], np.int32)
        # Stroked at its real kerb-to-kerb width, then a thin centreline: the
        # carriageway is what has to line up with the photo, and a 1 px line
        # through the middle of a 40 px road proves nothing about its width.
        cv2.polylines(vis, [px], False, (255, 255, 255),
                      max(1, int(round(e.width_m / mpp))), cv2.LINE_AA)
        cv2.polylines(vis, [px], False, (0, 200, 255), 2, cv2.LINE_AA)
    for n in net.nodes.values():
        if n.road_degree(net) >= 3:
            cv2.circle(vis, to_px(n.p), 5, (255, 0, 255), -1)
    for f in features or ():
        poly = np.array([to_px(q) for q in f["poly"]], np.int32)
        col = FEATURE_BGR.get(f["kind"], (200, 200, 200))
        cv2.polylines(vis, [poly], True, (0, 0, 0), FEATURE_W.get(f["kind"], 2) + 3,
                      cv2.LINE_AA)
        cv2.polylines(vis, [poly], True, col, FEATURE_W.get(f["kind"], 2), cv2.LINE_AA)
    if labels:
        # EVERY REGION CARRIES A NUMBER, so it can be named in a sentence.
        # `B7` is a block (a zone with a role), `F12` a feature (a thing
        # standing on one); both are numbered in reading order and listed in
        # the sidecar index this writes beside the image.
        for n, i in enumerate(reading_order(blocks, lambda b: b["centroid"]), 1):
            b = blocks[i]
            at = to_px(b["centroid"])
            _badge(vis, str(n), at, 13, (40, 40, 40))
            _text(vis, str(b.get("role", "?")), (at[0], at[1] + 28), 0.5,
                  (255, 255, 255))
        for n, i in enumerate(reading_order(features or [],
                                            lambda f: f["at"]), 1):
            f = (features or [])[i]
            col = FEATURE_BGR.get(f["kind"], (200, 200, 200))
            top = min(to_px(q)[1] for q in f["poly"])
            at = (to_px(f["at"])[0], top - 11)
            _badge(vis, str(n), at, 10, col)
            # BUILDINGS GO UNNAMED, still. There are 14 of them and the word
            # adds nothing a white rectangle has not already said — repeated
            # over a dense block it collides with the zone label. The badge is
            # what makes them referrable; the legend carries the count.
            if f["kind"] != "building":
                _text(vis, f["kind"], (at[0], top - 24), 0.45, col)
    _legend(vis, blocks, features)
    cv2.imwrite(out_path, vis)
    _write_index(out_path, blocks, features)
    return out_path


def _write_index(out_path, blocks, features):
    """`<image>_index.tsv` — what each badge on the image refers to.

    The image is for the eye and this is for everything else: it gives the
    number, the role or kind, the area or footprint, and the centroid in site
    metres, so a request naming `B7` can be resolved back to a manifest entry
    without re-deriving the numbering by hand.
    """
    path = os.path.splitext(out_path)[0] + "_index.tsv"
    rows = ["id\tkind_or_role\tarea_m2\tx_m\ty_m\tnote"]
    for n, i in enumerate(reading_order(blocks, lambda b: b["centroid"]), 1):
        b = blocks[i]
        cx, cy = b["centroid"]
        rows.append(f"B{n}\t{b.get('role','?')}\t{b.get('area', 0):.0f}"
                    f"\t{cx:.1f}\t{cy:.1f}\t")
    for n, i in enumerate(reading_order(features or [], lambda f: f["at"]), 1):
        f = (features or [])[i]
        xs = [q[0] for q in f["poly"]]
        ys = [q[1] for q in f["poly"]]
        a = (max(xs) - min(xs)) * (max(ys) - min(ys))
        rows.append(f"F{n}\t{f['kind']}\t{a:.0f}\t{f['at'][0]:.1f}"
                    f"\t{f['at'][1]:.1f}\t{str(f.get('note','')).replace(chr(9), ' ')}")
    with open(path, "w") as fh:
        fh.write("\n".join(rows) + "\n")
    return path


def _text(img, txt, at, scale, colour):
    """Outlined label — the aerial underneath is every brightness at once."""
    at = (int(at[0] - 4.6 * scale * len(txt)), int(at[1]))
    for c, t in (((0, 0, 0), int(3 + 3 * scale)), (colour, 1)):
        cv2.putText(img, txt, at, cv2.FONT_HERSHEY_SIMPLEX, scale, c, t, cv2.LINE_AA)


def _legend(img, blocks, features):
    """Key, bottom-left. Counts come from what was drawn, not from a constant."""
    rows = [("ZONES", None)]
    seen = {}
    for b in blocks:
        seen[b["role"]] = seen.get(b["role"], 0) + 1
    rows += [(f"{k}  x{v}", ROLE_BGR.get(k, (140, 140, 140)))
             for k, v in sorted(seen.items(), key=lambda kv: -kv[1])]
    if features:
        rows.append(("FEATURES", None))
        fs = {}
        for f in features:
            fs[f["kind"]] = fs.get(f["kind"], 0) + 1
        rows += [(f"{k}  x{v}", FEATURE_BGR.get(k, (200, 200, 200)))
                 for k, v in sorted(fs.items(), key=lambda kv: -kv[1])]
    pad, lh = 14, 26
    w, h = 250, pad * 2 + lh * len(rows)
    y0 = img.shape[0] - h - 20
    box = img[y0:y0 + h, 20:20 + w]
    img[y0:y0 + h, 20:20 + w] = (0.30 * box + 0.70 * np.zeros_like(box)).astype(np.uint8)
    for i, (label, col) in enumerate(rows):
        y = y0 + pad + lh * i + 18
        if col is None:
            cv2.putText(img, label, (34, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 2, cv2.LINE_AA)
        else:
            cv2.rectangle(img, (36, y - 12), (54, y - 1), col, -1)
            cv2.putText(img, label, (64, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (235, 235, 235), 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--plan", required=True, help="site-plan name or YAML path")
    ap.add_argument("--out", required=True)
    ap.add_argument("--no-labels", action="store_true")
    ap.add_argument("--blank", action="store_true",
                    help="draw the plan alone, without the aerial under it")
    args = ap.parse_args()

    spec = spl.load_spec(args.plan)
    net, blocks, info = spl.generate(*spl.region_of(spec), None, {"_spec": spec})
    print(f"[site_plan_png] {spec['name']}: {info['roads']} streets, "
          f"{len(blocks)} blocks, {info['roles_tagged']} roles matched, "
          f"{len(info['features'])} features "
          f"({info['features_homeless']} not on a block)")
    print(sn.format_stats(sn.stats(net, blocks, info["region"])))
    roles = {}
    for b in blocks:
        roles.setdefault(b["role"], []).append(b["area"])
    for r, a in sorted(roles.items(), key=lambda kv: -sum(kv[1])):
        print(f"  {r:12s} {len(a):2d} blocks  {sum(a):7,.0f} m²")
    base = None
    if args.blank:
        _to_px, (w, h), _mpp = projector(spec)
        base = np.full((h, w, 3), 245, np.uint8)
    print(f"[site_plan_png] -> "
          f"{draw(spec, net, blocks, args.out, not args.no_labels, base, info['features'])}")


if __name__ == "__main__":
    main()
