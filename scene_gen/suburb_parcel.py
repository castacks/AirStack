"""
suburb_parcel.py — lots, houses, drives and planting on a `suburb_net` block.

WHAT THIS IS FOR
----------------
`suburb_net` produces a street graph and the block polygons between the
streets. That is enough to judge the ROADS but not the FABRIC: a plan of bare
blocks cannot show whether the blocks are actually usable — whether a lot of
sane width fits along the frontage, whether the deep middle of a block is
stranded, whether cul-de-sac bulbs get the ring of houses that justifies them.
This pass answers those, and it is what makes the preview legible as a place.

THE ORGANISING IDEA: LOTS ARE HUNG OFF FRONTAGE, NOT CUT OUT OF AREA
--------------------------------------------------------------------
A suburban parcel is defined by the street it faces. So the block boundary is
walked as ARCLENGTH and a lot is issued every `lot_width_m`, each one taking
its orientation from the frontage tangent at that station:

    house yaw     = frontage tangent      (parallel to the street, always)
    house centre  = station + inward normal * (setback + depth/2)

which is why houses follow a curving street round its curve instead of staying
axis-aligned to the world. On the old rect-based generator every house was at
0/90/180/270 degrees because the block edge was always axis-aligned; here the
yaw is whatever the street is doing at that point, and that alone is most of
the difference in how the plan reads.

The deep interior of a block is deliberately left empty. That is not a gap in
the algorithm — it is the back yards, and on a real suburban block the middle
is exactly that: nothing but lawn, fence lines and trees.

OVERLAP IS TESTED AS ORIENTED BOXES
-----------------------------------
Once houses rotate with the street, an axis-aligned overlap test is wrong: two
houses on the inside of a curve can be well clear as AABBs and still intersect,
and on the outside of a curve the reverse. :func:`_obb_overlap` is a separating
axis test on the four box normals, which is exact for two rectangles.

WHAT IT DOES NOT DO
-------------------
No USD. Footprints are nominal rectangles, not measured asset bounding boxes —
`scene_generator.SizeResolver` is what does that, and wiring these lots to the
asset library is the next step, not this one. Treat the houses here as showing
WHERE a house goes and which way it faces, not which asset lands there.
"""

import math

from suburb_net import (_add, _sub, _mul, _dot, _unit, _perp, _dist,
                        polyline_length, point_at, tangent_at,
                        point_in_polygon, polygon_area)


def _ring_cum(ring):
    out = [0.0]
    for i in range(len(ring) - 1):
        out.append(out[-1] + _dist(ring[i], ring[i + 1]))
    return out


def _side_at(cum, s):
    """Index of the polygon side containing arclength *s*."""
    for i in range(len(cum) - 1):
        if s <= cum[i + 1]:
            return i
    return max(0, len(cum) - 2)

# ---------------------------------------------------------------------------
# house archetypes
# ---------------------------------------------------------------------------
# A lot is not a house — it is a house AND what came with it. A post-war plat is
# built by one developer in phases, so a run of lots shares a package: the same
# builder puts up the same house with the same garage and the same fence down a
# whole street, and the next phase does something slightly different. Drawing an
# archetype independently per lot destroys exactly that, and is what makes a
# generated suburb read as a shuffled catalogue rather than as a subdivision.
#
# So archetypes are assigned in RUNS along each block's frontage, not per lot.
ARCHETYPES = {
    # name          weight  garage   fence   lot scale
    "plain":       {"w": 0.30, "garage": 0.0, "fence": 0.0, "scale": 1.00},
    "fenced":      {"w": 0.22, "garage": 0.0, "fence": 1.0, "scale": 1.00},
    "garage":      {"w": 0.24, "garage": 1.0, "fence": 0.0, "scale": 1.08},
    "full":        {"w": 0.18, "garage": 1.0, "fence": 1.0, "scale": 1.18},
    # The big ones: wider lot, deeper setback, everything on it.
    "large":       {"w": 0.06, "garage": 1.0, "fence": 1.0, "scale": 1.42},
}


def _draw_archetype(rng):
    names = list(ARCHETYPES)
    tot = sum(ARCHETYPES[n]["w"] for n in names)
    r = rng.random() * tot
    for n in names:
        r -= ARCHETYPES[n]["w"]
        if r <= 0.0:
            return n
    return names[-1]


DEFAULTS = {
    # How many consecutive lots share an archetype. A builder puts up a phase,
    # not one house; 4-9 lots is a plat phase on a typical block face.
    "archetype_run": [4, 9],
    # Frontage per dwelling. US suburban lots run 50-80 ft wide; 20 m is a
    # 66 ft lot, the commonest post-war width.
    "lot_width_m": [17.0, 26.0],
    # Front setback. Zoning ordinances put this at 20-30 ft almost everywhere.
    "setback_m": [6.5, 11.0],
    # Detached house footprint: 30-50 ft across the frontage by 26-40 ft deep.
    "house_w_m": [9.0, 15.0],
    "house_d_m": [8.0, 12.5],
    # A lot needs this much depth behind the setback or it is not a lot.
    "min_lot_depth_m": 21.0,
    "house_gap_m": 4.0,          # side-yard between neighbours
    "driveway_w_m": 3.2,
    "garage_share": 0.55,        # lots whose drive widens to a pad
    # Planting. Front-yard and back-yard trees plus a street tree rhythm.
    # Off by default: suburb_yardplan owns yard planting. Set true only when
    # using this module WITHOUT yardplan.
    "own_yard_trees": False,
    "back_trees_per_100m2": 0.055,
    "front_tree_chance": 0.0,
    "street_tree_spacing_m": [16.0, 30.0],
    "tree_r_m": [2.2, 4.6],
    "tree_clear_m": 2.0,         # keep canopy off the house footprint
}


def _rng_pair(v, fallback):
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    if isinstance(v, (int, float)):
        return float(v), float(v)
    return fallback


def _corners(cx, cy, w, d, ux, uy):
    """Four corners of a box centred at (cx,cy), *w* along (ux,uy), *d* across."""
    vx, vy = -uy, ux
    hw, hd = w / 2.0, d / 2.0
    return [(cx + ux * hw + vx * hd, cy + uy * hw + vy * hd),
            (cx - ux * hw + vx * hd, cy - uy * hw + vy * hd),
            (cx - ux * hw - vx * hd, cy - uy * hw - vy * hd),
            (cx + ux * hw - vx * hd, cy + uy * hw - vy * hd)]


def _obb_overlap(a, b, pad=0.0):
    """Separating-axis test between two oriented boxes given as corner lists.

    Exact for rectangles: two convex polygons are disjoint iff some edge normal
    of one separates them, and a rectangle has only two distinct normals.
    """
    for poly in (a, b):
        for i in range(2):                      # two distinct normals per box
            ex = poly[(i + 1) % 4][0] - poly[i][0]
            ey = poly[(i + 1) % 4][1] - poly[i][1]
            n = _unit((-ey, ex))
            amin = min(_dot(n, p) for p in a)
            amax = max(_dot(n, p) for p in a)
            bmin = min(_dot(n, p) for p in b)
            bmax = max(_dot(n, p) for p in b)
            if amax + pad < bmin or bmax + pad < amin:
                return False
    return True


def _inward(poly, p, t):
    """Unit normal at *p* on the boundary pointing into the polygon."""
    n = _perp(t)
    if not point_in_polygon(poly, _add(p, _mul(n, 0.75))):
        n = _mul(n, -1.0)
    return n


def parcel_blocks(blocks, rng, cfg=None):
    """Lots, houses, drives and trees for every block.

    Returns ``[{"block": poly, "houses": [...], "drives": [...],
    "trees": [...]}, ...]`` where each house is
    ``{"c": (x, y), "w": w, "d": d, "u": (ux, uy), "corners": [...]}``.
    """
    c = dict(DEFAULTS)
    c.update(cfg or {})
    lw = _rng_pair(c["lot_width_m"], (17.0, 26.0))
    sb = _rng_pair(c["setback_m"], (6.5, 11.0))
    hw = _rng_pair(c["house_w_m"], (9.0, 15.0))
    hd = _rng_pair(c["house_d_m"], (8.0, 12.5))
    tr = _rng_pair(c["tree_r_m"], (2.2, 4.6))
    sts = _rng_pair(c["street_tree_spacing_m"], (16.0, 30.0))
    min_depth = float(c["min_lot_depth_m"])
    gap = float(c["house_gap_m"])
    dw = float(c["driveway_w_m"])

    out = []
    for blk in blocks:
        if isinstance(blk, dict):
            poly = blk["poly"]
            faces_street = blk.get("frontage")
        else:
            poly, faces_street = blk, None
        ring = list(poly) + [poly[0]]
        cum = _ring_cum(ring)

        def is_frontage(s):
            """Is the block side at arclength *s* a street?

            The crop boundary closes the block but is not pavement, so nothing
            fronts it. Everything else does.
            """
            if not faces_street:
                return True
            i = _side_at(cum, s)
            return bool(faces_street[i % len(faces_street)])

        perim = polyline_length(ring)
        if perim < 40.0:
            out.append({"block": poly, "houses": [], "drives": [], "trees": []})
            continue

        houses, drives, trees = [], [], []
        s = rng.uniform(0.0, lw[0])
        guard = 0
        run_lo, run_hi = _rng_pair(c.get("archetype_run", [4, 9]), (4.0, 9.0))
        arch = _draw_archetype(rng)
        run_left = int(rng.uniform(run_lo, run_hi + 0.999))
        while s < perim and guard < 4000:
            guard += 1
            width = rng.uniform(*lw)
            st = s
            s += width
            mid = st + width / 2.0
            if not is_frontage(mid):
                continue
            p = point_at(ring, mid)
            t = tangent_at(ring, mid)
            n = _inward(poly, p, t)

            # A lot needs real depth behind the setback. Probing the inward ray
            # is what keeps houses off the thin necks of an awkward block
            # instead of letting them hang over the far kerb.
            depth_ok = point_in_polygon(poly, _add(p, _mul(n, min_depth)))
            if not depth_ok:
                continue

            # Advance the archetype run. Consecutive lots share a package, so
            # similar houses end up next to each other rather than shuffled.
            if run_left <= 0:
                arch = _draw_archetype(rng)
                run_left = int(rng.uniform(run_lo, run_hi + 0.999))
            spec = ARCHETYPES[arch]
            run_left -= 1

            setback = rng.uniform(*sb)
            h_w = min(rng.uniform(*hw) * spec["scale"], width - gap)
            h_d = rng.uniform(*hd) * spec["scale"]
            if h_w < 7.0:
                continue
            cx, cy = _add(p, _mul(n, setback + h_d / 2.0))
            u = _unit(t)
            corners = _corners(cx, cy, h_w, h_d, u[0], u[1])
            if any(not point_in_polygon(poly, q) for q in corners):
                continue
            if any(_obb_overlap(corners, h["corners"], pad=gap * 0.5)
                   for h in houses):
                continue

            houses.append({"c": (cx, cy), "w": h_w, "d": h_d, "u": u,
                           # What this lot gets BUILT with, not just the house:
                           # suburb_scene composes the package from these.
                           "archetype": arch,
                           "has_garage": spec["garage"] > 0.0,
                           "has_fence": spec["fence"] > 0.0,
                           "lot_width": width,
                           # The INWARD normal, kept because it cannot be
                           # recovered later: perp(u) is perpendicular to u by
                           # definition, so no test against u can tell which of
                           # the two normals points away from the kerb.
                           "n": n, "frontage": p,
                           "corners": corners,
                           "yaw_deg": math.degrees(math.atan2(u[1], u[0]))})
            # Drive runs from the kerb to the front face, offset to one side of
            # the house so it lands beside the door rather than through it.
            side = 1.0 if rng.random() < 0.5 else -1.0
            off = _mul(_perp(u), 0.0)
            a0 = _add(p, _mul(u, side * h_w * 0.30))
            a1 = _add(a0, _mul(n, setback + 0.5))
            drives.append({"a": a0, "b": a1, "w": dw,
                           "pad": rng.random() < float(c["garage_share"])})
            # NO FRONT-YARD TREE HERE. `suburb_yardplan` owns yard planting and
            # places a specimen tree per lot against a POINT BUDGET; emitting one
            # here too planted every yard twice. Measured on the shipped preset:
            # 693 front + 858 back trees from this pass on top of yardplan's,
            # 1,551 duplicates, and this pass has no budget at all. This module
            # keeps only the STREET rhythm along the verge, which is the one
            # thing yardplan does not do.

        # Street trees: a rhythm along the kerb, independent of the lots, which
        # is how a verge is actually planted.
        u_s = rng.uniform(0.0, sts[1])
        while u_s < perim:
            if not is_frontage(u_s):
                u_s += rng.uniform(*sts)
                continue
            p = point_at(ring, u_s)
            t = tangent_at(ring, u_s)
            n = _inward(poly, p, t)
            q = _add(p, _mul(n, 2.6))
            if point_in_polygon(poly, q):
                r = rng.uniform(*tr)
                if not any(_dist(q, h["c"]) < max(h["w"], h["d"]) / 2.0 + r
                           for h in houses):
                    trees.append({"c": q, "r": r, "kind": "street"})
            u_s += rng.uniform(*sts)

        # NO BACK-YARD SCATTER HERE either -- same reason: yardplan plants the
        # rear yard, budgeted. Left as a no-op rather than deleted so the
        # `back_trees_per_100m2` knob keeps working for callers that use this
        # module without yardplan.
        area = abs(polygon_area(poly))
        want = int(area / 100.0 * float(c["back_trees_per_100m2"])
                   ) if c.get("own_yard_trees") else 0
        xs = [q[0] for q in poly]
        ys = [q[1] for q in poly]
        clear = float(c["tree_clear_m"])
        for _ in range(want * 12):
            if want <= 0:
                break
            q = (rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys)))
            if not point_in_polygon(poly, q):
                continue
            r = rng.uniform(*tr)
            if any(_obb_overlap(_corners(q[0], q[1], r * 2, r * 2, 1.0, 0.0),
                                h["corners"], pad=clear) for h in houses):
                continue
            if any(_dist(q, t["c"]) < r + t["r"] for t in trees):
                continue
            trees.append({"c": q, "r": r, "kind": "back"})
            want -= 1

        out.append({"block": poly, "houses": houses, "drives": drives,
                    "trees": trees})
    return out


def stats(parcels):
    n_h = sum(len(p["houses"]) for p in parcels)
    n_t = sum(len(p["trees"]) for p in parcels)
    per = [len(p["houses"]) for p in parcels if p["houses"]]
    return {"houses": n_h, "trees": n_t,
            "blocks_built": len(per), "blocks": len(parcels),
            "houses_per_built_block": (sum(per) / len(per)) if per else 0.0}
