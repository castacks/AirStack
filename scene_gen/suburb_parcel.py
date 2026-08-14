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

The deep interior of a block is deliberately left empty of HOUSES. That is not
a gap in the algorithm — it is the back yards, and on a real suburban block the
middle is exactly that: nothing but lawn, fence lines and trees. Each lot does
carry its rectangle (`lot_corners`, `lot_depth`) so the fence lines that bound
those back yards can be drawn, and so a consumer can tell a 21 m yard from a
38 m one.

DENSITY IS A PROPERTY OF THE BLOCK, NOT OF THE LOT
--------------------------------------------------
Drawing lot width from one range everywhere makes every street in the map the
same street, statistically. Real tracts are not like that, and the reason is
legal rather than aesthetic — see the comment on :data:`DENSITY`.

OVERLAP IS TESTED AS ORIENTED BOXES
-----------------------------------
Once houses rotate with the street, an axis-aligned overlap test is wrong: two
houses on the inside of a curve can be well clear as AABBs and still intersect,
and on the outside of a curve the reverse. :func:`_obb_overlap` is a separating
axis test on the four box normals, which is exact for two rectangles.

GROUND THE BLOCK POLYGON DOES NOT KNOW ABOUT
--------------------------------------------
Being inside the block polygon is not the same as being off the pavement. A
cul-de-sac does not end at a point: `suburb_net` puts a paved TURNAROUND DISC
at the tip of every lollipop edge (14.64 m radius, IFC Fig D103.1's 96 ft
driving diameter) and `suburb_scene.apply_ground` lays asphalt over it, but the
block polygon is built from the street faces and carries no record of it. A lot
issued off the frontage beside a bulb is therefore platted on the road, and
passes every containment test in here while doing it. `cfg["keepout_discs"]` is
how the caller — which has the net, and so has the real per-edge radius — says
where that pavement is; house, garage, fence and tree are all refused or cut
back out of it.

WHAT IT DOES NOT DO
-------------------
No USD, and no asset library: this module still cannot look a house up. What it
will now do is USE a measurement someone else took — `cfg["house_sizes"]` takes
the caller's measured footprints and sites those instead of nominal rectangles,
stamping the entry it used on each house as `size_index` so the caller can place
that exact asset. Absent the key the footprints are nominal rectangles drawn
from `house_w_m`/`house_d_m`, and then every overlap test below is only as true
as that guess — which is the honest reading of a house that clears the test here
and overlaps its neighbour on screen.
"""

import math

from suburb_net import (_add, _sub, _mul, _dot, _unit, _perp, _dist,
                        polyline_length, point_at, tangent_at,
                        point_in_polygon, polygon_area, seg_seg_dist)


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


# ---------------------------------------------------------------------------
# density classes
# ---------------------------------------------------------------------------
# Same argument as ARCHETYPES one level up. A zoning map does not assign a
# minimum lot width to a HOUSE, it assigns one to a DISTRICT, and the district
# boundary is drawn along streets and platted block lines. US single-family
# districts commonly bottom out around a 40-50 ft minimum width and top out at
# 100 ft or more in a large-lot/estate district, with the front setback moving
# with it (20-25 ft in the tight districts, 40-50 ft in the estate ones). So
# every lot on a block face is about as wide as its neighbours, and the change
# happens at the block, which is exactly what "the houses can be close in some
# areas and far apart in others" describes.
#
# Drawn ONCE PER BLOCK. Drawing per lot would put an estate lot between two
# starter lots, which no plat has ever done.
#
#   "lot"     multiplies the sampled frontage width
#   "setback" multiplies the sampled front setback
#   "depth"   multiplies the sampled lot depth — gently. Depth is set by how
#             the surveyor tiered the block, so it moves far less than width
#             does across districts, but the estate tract does get the deeper
#             back yard as well as the wider one.
#
# The HOUSE is deliberately not scaled. A wider lot in a low-density district
# buys side yard and back yard, not a bigger building — the same builder's
# 1,600 sq ft plan goes up on the 40 ft lot and on the 100 ft lot. House size
# is the archetype's job (ARCHETYPES[...]["scale"]), and mixing the two knobs
# would cancel the effect the width is there to produce.
DENSITY = {
    # name           weight   lot width   setback       depth
    "tight":       {"w": 0.28, "lot": 0.78, "setback": 0.80, "depth": 0.85},
    "normal":      {"w": 0.44, "lot": 1.00, "setback": 1.00, "depth": 1.00},
    "loose":       {"w": 0.22, "lot": 1.45, "setback": 1.25, "depth": 1.15},
    "estate":      {"w": 0.06, "lot": 2.20, "setback": 1.60, "depth": 1.35},
}


def _draw_density(rng, mix=None):
    """Pick a density class, honouring a caller-supplied ``{name: weight}``."""
    names = list(DENSITY)
    wts = [float((mix or {}).get(n, DENSITY[n]["w"])) for n in names]
    tot = sum(w for w in wts if w > 0.0)
    if tot <= 0.0:
        return "normal"
    r = rng.random() * tot
    for n, w in zip(names, wts):
        if w <= 0.0:
            continue
        r -= w
        if r <= 0.0:
            return n
    return names[-1]


DEFAULTS = {
    # How many consecutive lots share an archetype. A builder puts up a phase,
    # not one house; 4-9 lots is a plat phase on a typical block face.
    "archetype_run": [4, 9],
    # Per-block density mix, as {class: weight}. None = the weights authored on
    # DENSITY. A preset that wants a uniform tract passes {"normal": 1.0}.
    "density_mix": None,
    # Frontage per dwelling. US suburban lots run 50-80 ft wide; 20 m is a
    # 66 ft lot, the commonest post-war width.
    "lot_width_m": [17.0, 26.0],
    # Front setback. Zoning ordinances put this at 20-30 ft almost everywhere.
    "setback_m": [6.5, 11.0],
    # Detached house footprint: 30-50 ft across the frontage by 26-40 ft deep.
    # NOMINAL — only used when `house_sizes` is absent. See it below.
    "house_w_m": [9.0, 15.0],
    "house_d_m": [8.0, 12.5],
    # MEASURED FOOTPRINTS, one `(w, d)` per house asset the caller may place, in
    # metres, `w` ALONG THE FRONTAGE and `d` inward — already resolved through
    # whatever yaw offset the renderer applies (`suburb_scene` turns the house
    # by `house_yaw_offset_deg`, so the measured axis that ends up across the
    # front is the one to pass as `w`). This module only knows the street frame
    # and cannot do that resolution itself.
    #
    # WHY: every overlap test below is only as true as the box it is handed, and
    # the nominal box is a guess about art nobody measured. The shipped pool
    # mixes bungalows pinned to a 12 m longest plan dimension with the
    # RetroNeighborhood SM_House_* that `suburban.yaml` flags in as many words
    # as unverified, while this module sites boxes anywhere from 7.0 to 21.3 m
    # across. A lot that reserved 9 m and receives a 12 m asset overlaps its
    # neighbour on screen having passed every test in here, and no arithmetic
    # fix reaches that, because the arithmetic was right about the wrong box.
    #
    # MEASURED IN, INDEX OUT. The entry chosen is stamped on the house as
    # `size_index`; it is only a fix if the caller places THAT asset, otherwise
    # the guess has merely moved one module along.
    "house_sizes": None,
    # PAVEMENT THE BLOCK POLYGON CANNOT SEE, as `((x, y), r)` discs in world
    # metres. Cul-de-sac turnarounds are what this is for; the caller holds the
    # net and so knows the real radius of each one, which is why none is assumed
    # here. Pass the radius you want KEPT CLEAR rather than the paved radius if
    # you want a front yard as well: a house tangent to the kerb is still a
    # house on the road.
    "keepout_discs": (),
    # A lot needs this much depth behind the setback or it is not a lot.
    "min_lot_depth_m": 21.0,
    # Platted lot depth, 85-125 ft, the band almost every US plat sits in. It
    # is SAMPLED and then trimmed to what the block can give, not simply taken
    # as deep as the block allows: these blocks are 100-200 m through, so
    # "deepest that fits" would put every single lot on the cap and every back
    # yard the same size, which is the opposite of the point.
    "lot_depth_m": [26.0, 38.0],
    # Hard ceiling, however deep the block is. A 60 m lot is not a suburban
    # lot, it is acreage; past ~130 ft the plat would have run a rear alley or
    # a second tier of lots instead. The block interior beyond it stays open.
    "max_lot_depth_m": 40.0,
    "house_gap_m": 4.0,          # side-yard between neighbours
    "driveway_w_m": 3.2,
    "garage_share": 0.55,        # lots whose drive widens to a pad
    # Detached double garage: 20 x 21 ft, the standard two-car box. Held beside
    # the house rather than merged into it because the house asset is a single
    # mesh and cannot grow a wing.
    "garage_w_m": 6.0,
    "garage_d_m": 6.5,
    "garage_gap_m": 0.8,         # breezeway between house and garage
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


def _norm_discs(v):
    """`[((x, y), r), ...]` from whatever the caller passed, dropping junk."""
    out = []
    for item in (v or ()):
        try:
            c, r = item[0], float(item[1])
        except (TypeError, IndexError, ValueError):
            continue
        if r > 0.0:
            out.append(((float(c[0]), float(c[1])), r))
    return out


def _hits_keepout(corners, discs):
    """Does an oriented box reach into any keep-out disc?

    Box-versus-circle rather than centre-versus-circle: a 12 m house whose
    CENTRE clears a 14.64 m bulb by a metre still has half its frontage on the
    turnaround, and the turnaround is the thing you see it standing on. Inside
    (the disc swallowing the box whole, which happens to a garage) or within *r*
    of any wall both count; the bbox reject in front is what keeps this off the
    hot path, since a block sees at most one or two bulbs and usually none.
    """
    if not discs:
        return False
    xs = [q[0] for q in corners]
    ys = [q[1] for q in corners]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    for (c, r) in discs:
        if (c[0] < x0 - r or c[0] > x1 + r
                or c[1] < y0 - r or c[1] > y1 + r):
            continue
        if point_in_polygon(corners, c):
            return True
        for i in range(4):
            if seg_seg_dist(corners[i], corners[(i + 1) % 4], c, c) < r:
                return True
    return False


def _clip_seg_disc(a, b, c, r):
    """Trim a fence run back out of a paved turnaround.

    Same contract and same reasoning as :func:`_clip_seg` one level down — a
    fence stops at the thing it cannot stand in — with a circle in place of a
    building. Solving |a + d*t - c| = r for the two crossings is exact, where
    sampling the run would let a short chord through. The middle case gives up
    for the same reason it does there: a bulb biting the centre of a lot line
    would leave two stubs, and a fence in two pieces with a road through it is
    worse than no fence.
    """
    d = _unit(_sub(b, a))
    ln = _dist(a, b)
    f = _sub(a, c)
    hb = _dot(f, d)                       # half-b of the quadratic in t
    det = hb * hb - (_dot(f, f) - r * r)
    if det <= 0.0:
        return a, b                       # the line misses the circle outright
    sq = math.sqrt(det)
    t0, t1 = -hb - sq, -hb + sq
    if t1 <= 0.0 or t0 >= ln:
        return a, b                       # ...or the crossings are off the run
    if t0 <= 0.0 and t1 >= ln:
        return None                       # the whole run is on the pavement
    if t0 <= 0.0:
        a = _add(a, _mul(d, t1))
    elif t1 >= ln:
        b = _add(a, _mul(d, t0))
    else:
        return None
    return (a, b) if _dist(a, b) >= 2.0 else None


def _norm_sizes(v):
    """`[(w, d), ...]` from the caller's measured table, dropping junk.

    Order is PRESERVED, because the index into it is the contract: it is what
    tells the caller which asset was sited. Sorting here would silently rebind
    every house to a different model.
    """
    out = []
    for e in (v or ()):
        try:
            w, d = float(e[0]), float(e[1])
        except (TypeError, IndexError, ValueError):
            continue
        if w > 0.0 and d > 0.0:
            out.append((w, d))
    return out


def _pick_size(sizes, order, rng, max_w, scale):
    """Choose a measured footprint for a lot of this frontage, or ``None``.

    Two things the nominal path gets for free and this one has to earn.

    FIT. `h_w = min(sampled, width - gap)` cannot be applied to a measurement —
    clamping a measured asset is just the nominal guess again, and a 12 m model
    scaled down to 9 m is a model with 9 m doors. So the fit is a FILTER on the
    catalogue instead: the builder puts the plan that fits on the lot, which is
    what a real plat does and is why narrow lots carry narrow houses. Only when
    NOTHING in the catalogue fits does the lot go unbuilt. The criterion is the
    same half-gap :func:`parcel_blocks` gives the overlap test, deliberately —
    a stricter one here would refuse a house the overlap test would have let
    stand, and pay for it in gaps down the street.

    SIZE CLASS. `ARCHETYPES[...]["scale"]` cannot resize a measured asset for
    the same reason, so it selects rather than scales: rank the fitting entries
    by footprint AREA and bias the draw up that ranking. Scale is linear and
    area goes as its square, hence the exponent — 1.00 leaves the draw uniform
    over everything that fits, 1.42 ("large") pulls its mean to about the 74th
    percentile of it. Biasing rather than slicing matters: a hard band would cut
    the top of the catalogue out of the 52% of lots at scale 1.00 and those
    models would never be built.
    """
    fit = [i for i in order if sizes[i][0] <= max_w]
    if not fit:
        return None
    q = rng.random() ** (1.0 / (scale * scale))
    return fit[min(len(fit) - 1, int(q * len(fit)))]


def _probe_depth(poly, p, n, u, width, lo, hi, step=1.0):
    """Trim a wanted depth *hi* to what the block will actually give.

    The same idea as the `min_lot_depth_m` gate, walked outward instead of
    tested once: step inward along *n* and keep the deepest station whose rear
    corners are all still inside the block. Stopping at the FIRST failure
    rather than continuing matters — a re-entrant block can put polygon
    interior back under the ray on the far side of a neck, and a lot is not
    allowed to jump the gap and take its back yard out of the next street's
    frontage. Returns *hi* exactly when nothing gets in the way, and never
    less than *lo*: a lot that only just cleared the depth gate above still
    gets its minimum rectangle rather than a sliver.
    """
    hw = width / 2.0
    fl = _add(p, _mul(u, -hw))
    fr = _add(p, _mul(u, hw))
    best, d = lo, lo
    while d < hi - 1e-9:
        d = min(d + step, hi)
        if (point_in_polygon(poly, _add(p, _mul(n, d)))
                and point_in_polygon(poly, _add(fl, _mul(n, d)))
                and point_in_polygon(poly, _add(fr, _mul(n, d)))):
            best = d
        else:
            break
    return best


def _seg_key(p0, p1, q=0.05):
    """Quantised identity for a boundary, so two lots that share it fence once.

    Unordered: nothing says the two neighbours walk the shared line from the
    same end.
    """
    a = (round(p0[0] / q), round(p0[1] / q))
    b = (round(p1[0] / q), round(p1[1] / q))
    return (a, b) if a <= b else (b, a)


def _line_dupe(segs, a, b, tol=0.60, cos_tol=0.92, min_ov=0.5):
    """Is this boundary already fenced, by the lot on the other side of it?

    :func:`_seg_key` catches only the exact case, and the exact case is the
    rare one. Two lots share a boundary but do not describe it identically:
    they agree on the front anchor only to within the chord error of a curving
    block face, and they disagree outright on the far end, because each one
    platted its own lot depth. Back-to-back lots on a thin block share a REAR
    line and meet it head-on, from opposite directions and offset along it.
    Keyed on endpoints, none of that matches, and the lot line gets two fences
    laid down it — a 26 m one and a 34 m one, overlapping for 26 m. Which is
    the doubled-geometry bug, just in fencing.

    So match the LINE: near-parallel (either direction), overlapping along it
    by more than *min_ov*, and no more than *tol* apart WHERE THEY OVERLAP —
    measured at the middle of the shared stretch rather than at an endpoint,
    because two boundaries that share a corner and splay apart by a few degrees
    are two boundaries, while two that run 20 m side by side 10 cm apart are
    one. Neighbours whose fences merely MEET end to end project to a
    zero-length overlap and are correctly left alone, and a block corner is
    safe because the two faces there are tens of degrees apart.
    """
    d = _unit(_sub(b, a))
    ln = _dist(a, b)
    for entry in segs:
        q0, q1 = entry[0], entry[1]
        e = _unit(_sub(q1, q0))
        if abs(_dot(d, e)) < cos_tol:
            continue
        t0, t1 = _dot(_sub(q0, a), d), _dot(_sub(q1, a), d)
        lo, hi = max(min(t0, t1), 0.0), min(max(t0, t1), ln)
        if hi - lo <= min_ov:
            continue
        mid = _add(a, _mul(d, (lo + hi) * 0.5))
        k = max(0.0, min(_dist(q0, q1), _dot(_sub(mid, q0), e)))
        if _dist(mid, _add(q0, _mul(e, k))) <= tol:
            return True
    return False


def _seg_box(a, b, t=0.04):
    """A segment as a hairline box, so :func:`_obb_overlap` can test it."""
    d = _unit(_sub(b, a))
    m = _mul(_add(a, b), 0.5)
    return _corners(m[0], m[1], _dist(a, b), t, d[0], d[1])


def _clip_seg(a, b, box, pad=0.10):
    """Trim a fence run back to where a building stands on it.

    A fence butts into a wall, it does not pass through it, and two buildings
    stand on these lines. A garage on the side line is the boundary structure
    for its own length — half the reason they were built on the line — so the
    run is cut at it rather than deleted, and the back of the side yard stays
    fenced, which is what you see from the street. The other case is a
    NEIGHBOUR's house: lots are hung off frontage independently, so two lots
    issued from perpendicular faces of the same block genuinely overlap deep in
    the corner, and one of them will have platted its back yard across the
    other's living room. Cutting the fence at the wall is the honest reading of
    that — the alternative is a full lot-versus-lot packing pass, which is a
    different module.

    Returns ``None`` when nothing useful is left, or when the obstruction is in
    the middle of the run and cutting would leave two stubs.
    """
    if not _obb_overlap(_seg_box(a, b), box, pad=-pad):
        return a, b
    d = _unit(_sub(b, a))
    ln = _dist(a, b)
    ts = [_dot(_sub(q, a), d) for q in box]
    t0, t1 = min(ts) - pad, max(ts) + pad
    if t0 <= 0.0 and t1 < ln:
        a = _add(a, _mul(d, t1))
    elif t1 >= ln and t0 > 0.0:
        b = _add(a, _mul(d, t0))
    else:
        return None
    return (a, b) if _dist(a, b) >= 2.0 else None


def _clip_standing(lines, box):
    """Re-cut fences already standing where a new garage has just landed.

    Lots are issued along the frontage in order, so the neighbour fenced the
    shared line before this lot existed and could not have known a garage was
    coming. The line stays REGISTERED in *lines* either way, so no third lot
    lays another one down it.
    """
    for e in lines:
        if e[2] is None:
            continue
        cut = _clip_seg(e[0], e[1], box)
        if cut is not None and cut[0] is e[0] and cut[1] is e[1]:
            continue                                   # untouched
        i = e[2].index(e[3])
        if cut is None:
            e[2].pop(i)
            e[2] = None
            continue
        e[0], e[1] = cut
        e[3] = (cut[0], cut[1], e[3][2])
        e[2][i] = e[3]


def _inward(poly, p, t):
    """Unit normal at *p* on the boundary pointing into the polygon."""
    n = _perp(t)
    if not point_in_polygon(poly, _add(p, _mul(n, 0.75))):
        n = _mul(n, -1.0)
    return n


def parcel_blocks(blocks, rng, cfg=None):
    """Lots, houses, drives and trees for every block.

    Returns ``[{"block": poly, "density": name, "houses": [...],
    "drives": [...], "trees": [...], "garages_rejected": n,
    "keepout_rejected": n, "size_rejected": n}, ...]`` where each house is
    ``{"c": (x, y), "w": w, "d": d, "u": (ux, uy), "corners": [...]}``
    plus the lot it stands on: ``lot_corners`` (front_left, front_right,
    rear_right, rear_left), ``lot_depth``, ``fence_segs``
    (``[(p0, p1, "privacy"|"low"), ...]``, deduplicated across the block) and
    ``garage`` (``None`` or a box in the same shape as the house). With
    ``cfg["house_sizes"]`` supplied, ``size_index`` says which measured entry
    the footprint came from and the caller is expected to place that asset.
    """
    c = dict(DEFAULTS)
    c.update(cfg or {})
    lw = _rng_pair(c["lot_width_m"], (17.0, 26.0))
    sb = _rng_pair(c["setback_m"], (6.5, 11.0))
    hw = _rng_pair(c["house_w_m"], (9.0, 15.0))
    hd = _rng_pair(c["house_d_m"], (8.0, 12.5))
    tr = _rng_pair(c["tree_r_m"], (2.2, 4.6))
    sts = _rng_pair(c["street_tree_spacing_m"], (16.0, 30.0))
    ld = _rng_pair(c["lot_depth_m"], (26.0, 38.0))
    min_depth = float(c["min_lot_depth_m"])
    max_depth = max(min_depth, float(c["max_lot_depth_m"]))
    gap = float(c["house_gap_m"])
    dw = float(c["driveway_w_m"])
    g_w = float(c["garage_w_m"])
    g_d = float(c["garage_d_m"])
    g_sep = float(c["garage_gap_m"])
    dens_mix = c.get("density_mix")
    sizes = _norm_sizes(c.get("house_sizes"))
    # Ranked by area once, not per lot: the catalogue is a couple of dozen
    # entries and every lot on the map walks it.
    size_order = sorted(range(len(sizes)),
                        key=lambda i: sizes[i][0] * sizes[i][1])
    discs = _norm_discs(c.get("keepout_discs"))

    out = []
    for blk in blocks:
        if isinstance(blk, dict):
            poly = blk["poly"]
            faces_street = blk.get("frontage")
        else:
            poly, faces_street = blk, None
        ring = list(poly) + [poly[0]]
        cum = _ring_cum(ring)
        # The bulbs that can possibly reach this block. Every candidate on the
        # block is tested against this list, so it is worth cutting 26 discs
        # down to the nought-or-one that are actually near before doing it.
        pxs = [q[0] for q in poly]
        pys = [q[1] for q in poly]
        blk_discs = [(dc, dr) for (dc, dr) in discs
                     if (min(pxs) - dr <= dc[0] <= max(pxs) + dr
                         and min(pys) - dr <= dc[1] <= max(pys) + dr)]

        def is_frontage(s):
            """Is the block side at arclength *s* a street?

            The crop boundary closes the block but is not pavement, so nothing
            fronts it. Everything else does.
            """
            if not faces_street:
                return True
            i = _side_at(cum, s)
            return bool(faces_street[i % len(faces_street)])

        # The district this block sits in. One draw, before any lot exists.
        dens = _draw_density(rng, dens_mix)
        d_lot = float(DENSITY[dens]["lot"])
        d_set = float(DENSITY[dens]["setback"])
        d_dep = float(DENSITY[dens].get("depth", 1.0))

        perim = polyline_length(ring)
        if perim < 40.0:
            out.append({"block": poly, "density": dens, "houses": [],
                        "drives": [], "trees": [], "garages_rejected": 0,
                        "keepout_rejected": 0, "size_rejected": 0})
            continue

        houses, drives, trees = [], [], []
        garages = []                 # boxes only, for the overlap tests
        fenced = set()               # boundary identities already fenced
        fence_lines = []             # ...and the same as lines, for near-misses
        n_reject = 0
        n_keepout = 0                # lots lost to a turnaround
        n_nofit = 0                  # ...and to a frontage nothing measured fits
        s = rng.uniform(0.0, lw[0])
        guard = 0
        run_lo, run_hi = _rng_pair(c.get("archetype_run", [4, 9]), (4.0, 9.0))
        arch = _draw_archetype(rng)
        run_left = int(rng.uniform(run_lo, run_hi + 0.999))
        while s < perim and guard < 4000:
            guard += 1
            width = rng.uniform(*lw) * d_lot
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

            # Width and setback move with the district; the HOUSE does not.
            # The extra frontage an estate lot buys is side yard, and the extra
            # depth behind the house is the back yard the user is asking for.
            setback = rng.uniform(*sb) * d_set
            if sizes:
                # The catalogue is the authority on how big a house is. See
                # `house_sizes` in DEFAULTS and :func:`_pick_size`.
                size_i = _pick_size(sizes, size_order, rng,
                                    width - gap * 0.5, float(spec["scale"]))
                if size_i is None:
                    n_nofit += 1
                    continue
                h_w, h_d = sizes[size_i]
            else:
                size_i = None
                h_w = min(rng.uniform(*hw) * spec["scale"], width - gap)
                h_d = rng.uniform(*hd) * spec["scale"]
                if h_w < 7.0:
                    continue
            cx, cy = _add(p, _mul(n, setback + h_d / 2.0))
            u = _unit(t)
            corners = _corners(cx, cy, h_w, h_d, u[0], u[1])
            if any(not point_in_polygon(poly, q) for q in corners):
                continue
            # Inside the block is not off the road — see the module docstring.
            # The lot goes entirely, rather than being nudged along the
            # frontage: the station is where the street said to put it, and a
            # house shuffled off its own frontage to dodge the turnaround is
            # the next screenshot's defect.
            if _hits_keepout(corners, blk_discs):
                n_keepout += 1
                continue
            if any(_obb_overlap(corners, h["corners"], pad=gap * 0.5)
                   for h in houses):
                continue
            # Garages are tested here too. They are placed after their house,
            # so without this a later house on the next lot would happily land
            # on the previous lot's garage — it is not in `houses`.
            if any(_obb_overlap(corners, g["corners"], pad=0.5)
                   for g in garages):
                continue

            # This house may stand on a line an earlier lot already fenced —
            # see :func:`_clip_seg` on why lots overlap at all.
            _clip_standing(fence_lines, corners)

            # --- the lot rectangle -------------------------------------------
            # Frontage width along the kerb, `lot_depth` inward. Left/right is
            # taken against the frontage tangent: left = -u, right = +u.
            # ...but never shallower than the house on it, plus a rear yard.
            # A deep setback and a large archetype together can put the back
            # wall 35 m in, and a rear lot line in front of the house it
            # belongs to would fence the building in half.
            want_depth = min(max_depth,
                             max(min_depth, setback + h_d + 4.0,
                                 rng.uniform(*ld) * d_dep))
            lot_depth = _probe_depth(poly, p, n, u, width,
                                     min_depth, want_depth)
            # Half-width is capped by the CHORD back to the two stations this
            # lot actually runs between. On a straight face that is exactly
            # `width / 2`; on a curving one the chord is shorter than the arc,
            # and taking the arc would push the corner past the neighbouring
            # lot's — a 34 m estate frontage on a curve overshoots by ~4 m,
            # which is four metres of two lots' front fences in the same place.
            # Surveyors plat curved frontage as chord bearings for this reason.
            half = min(width / 2.0,
                       _dist(p, point_at(ring, st)),
                       _dist(p, point_at(ring, s)))
            fl = _add(p, _mul(u, -half))
            fr = _add(p, _mul(u, half))
            rl = _add(fl, _mul(n, lot_depth))
            rr = _add(fr, _mul(n, lot_depth))
            lot_corners = [fl, fr, rr, rl]

            # --- which side the car lives on ---------------------------------
            # Drawn before the garage, because the drive and the garage have to
            # agree: a drive that runs up the left of the house to a garage on
            # the right is the single most obvious tell in a generated suburb.
            side = 1.0 if rng.random() < 0.5 else -1.0

            garage = None
            if spec["garage"] > 0.0:
                g_off = side * (h_w / 2.0 + g_sep + g_w / 2.0)
                gc = _add(p, _add(_mul(u, g_off),
                                  _mul(n, setback + g_d / 2.0)))
                g_corners = _corners(gc[0], gc[1], g_w, g_d, u[0], u[1])
                # ON or OVER the side lot line is allowed, and is the normal
                # case: a 6 m garage plus a 12 m house does not fit inside a
                # 20 m frontage, which is why detached garages all over
                # pre-zoning plats sit hard on the line. Requiring containment
                # instead costs the fenced archetypes their garage on every lot
                # narrower than about 26 m, i.e. nearly all of them, and "full"
                # then renders identically to "fenced". What is NOT allowed is
                # a garage inside a building, so the two structural tests are
                # exactly the ones the house passes; the FENCE is what gives
                # way, cut at the garage wall by `_clip_seg` below.
                # The bulb is tested here for the same reason the house tests
                # it, and it bites harder: a garage sits at the setback line
                # with no front yard in front of it, so a lot whose house
                # cleared the turnaround can still have parked its garage on it.
                ok = (all(point_in_polygon(poly, q) for q in g_corners)
                      and not _hits_keepout(g_corners, blk_discs))
                if ok:
                    ok = not (any(_obb_overlap(g_corners, h["corners"])
                                  for h in houses)
                              or any(_obb_overlap(g_corners, g["corners"])
                                     for g in garages))
                if ok:
                    garage = {"c": gc, "w": g_w, "d": g_d, "u": u,
                              "corners": g_corners}
                    garages.append(garage)
                    _clip_standing(fence_lines, g_corners)
                else:
                    # No garage beats a garage in the neighbour's living room.
                    n_reject += 1

            # --- fences -------------------------------------------------------
            # Sides and rear are the tall close-board run; only the two full
            # packages also close the front, and that one has to be the LOW
            # asset — every US code that caps fence height caps the front yard
            # at 3-4 ft against 6-8 ft elsewhere, so a privacy panel along the
            # kerb reads as a compound, not a suburb. The tags are the pool
            # tags in the asset set; they are not decorative.
            fence_segs = []
            if spec["fence"] > 0.0:
                cand = [(fl, rl, "privacy"), (fr, rr, "privacy"),
                        (rl, rr, "privacy")]
                blockers = [corners] + [h["corners"] for h in houses] \
                    + [g["corners"] for g in garages]
                if arch in ("full", "large"):
                    # Front run, broken for the drive. A fence across your own
                    # driveway is worse than no fence.
                    g_half = (g_w if garage is not None else dw) / 2.0 + 0.6
                    d_off = (g_off if garage is not None
                             else side * h_w * 0.30)
                    for x0, x1 in ((-half, d_off - g_half),
                                   (d_off + g_half, half)):
                        if x1 - x0 < 1.5:
                            continue
                        cand.append((_add(p, _mul(u, x0)),
                                     _add(p, _mul(u, x1)), "low"))
                for a, b, tag in cand:
                    k = _seg_key(a, b)
                    if k in fenced or _line_dupe(fence_lines, a, b):
                        continue
                    fenced.add(k)
                    cut = (a, b)
                    # Pavement before buildings, because it is the cut that can
                    # take the whole run: a lot beside a bulb has its two side
                    # lines running out of the turnaround, and there is no point
                    # asking which wall a fence stops at when the front half of
                    # it is on the road.
                    for (dc, dr) in blk_discs:
                        cut = _clip_seg_disc(cut[0], cut[1], dc, dr)
                        if cut is None:
                            break
                    if cut is not None:
                        for box in blockers:     # includes this lot's own
                            cut = _clip_seg(cut[0], cut[1], box)
                            if cut is None:
                                break
                    if cut is None:
                        # A building stands on this boundary for its whole
                        # length. Still registered, so the neighbour does not
                        # come back and fence it either.
                        fence_lines.append([a, b, None, None])
                        continue
                    seg = (cut[0], cut[1], tag)
                    fence_lines.append([cut[0], cut[1], fence_segs, seg])
                    fence_segs.append(seg)

            houses.append({"c": (cx, cy), "w": h_w, "d": h_d, "u": u,
                           # What this lot gets BUILT with, not just the house:
                           # suburb_scene composes the package from these.
                           "archetype": arch,
                           # WHICH measured footprint was sited, as an index
                           # into `cfg["house_sizes"]` (None when the nominal
                           # sampling was used). The caller must place the asset
                           # at this index and no other — the box tested for
                           # overlap here IS that asset, and drawing a different
                           # one puts the module back to guessing.
                           "size_index": size_i,
                           "has_garage": spec["garage"] > 0.0,
                           "has_fence": spec["fence"] > 0.0,
                           "lot_width": width,
                           # The lot itself, so the package can be BUILT and
                           # not merely recorded: the rectangle, how deep the
                           # back yard actually runs on this block, the fence
                           # lines already deduplicated against the neighbour,
                           # and the garage box (None when it did not fit).
                           "lot_corners": lot_corners,
                           "lot_depth": lot_depth,
                           "fence_segs": fence_segs,
                           "garage": garage,
                           "density": dens,
                           # The INWARD normal, kept because it cannot be
                           # recovered later: perp(u) is perpendicular to u by
                           # definition, so no test against u can tell which of
                           # the two normals points away from the kerb.
                           "n": n, "frontage": p,
                           "corners": corners,
                           "yaw_deg": math.degrees(math.atan2(u[1], u[0]))})
            # Drive runs from the kerb to the front face, offset to one side of
            # the house so it lands beside the door rather than through it —
            # and when this lot got a garage, to the garage door instead, which
            # is where a drive on a real lot goes. Same `side` either way.
            if garage is not None:
                a0 = _add(p, _mul(u, g_off))
                a1 = _add(a0, _mul(n, setback + 0.2))
            else:
                a0 = _add(p, _mul(u, side * h_w * 0.30))
                a1 = _add(a0, _mul(n, setback + 0.5))
            drives.append({"a": a0, "b": a1, "w": dw,
                           # A garage apron IS the pad; without one it is the
                           # `garage_share` coin flip as before.
                           "pad": (garage is not None
                                   or rng.random() < float(c["garage_share"]))})
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
                # A verge tree goes 2.6 m in from the block boundary, which
                # around a turnaround is 2.6 m into the asphalt: the boundary
                # the offset is measured from is not the edge of the bulb.
                on_bulb = any(_dist(q, dc) < dr + r for (dc, dr) in blk_discs)
                if not on_bulb and not any(
                        _dist(q, h["c"]) < max(h["w"], h["d"]) / 2.0 + r
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
            if any(_dist(q, dc) < dr + r for (dc, dr) in blk_discs):
                continue
            if any(_obb_overlap(_corners(q[0], q[1], r * 2, r * 2, 1.0, 0.0),
                                h["corners"], pad=clear) for h in houses):
                continue
            if any(_dist(q, t["c"]) < r + t["r"] for t in trees):
                continue
            trees.append({"c": q, "r": r, "kind": "back"})
            want -= 1

        out.append({"block": poly, "density": dens, "houses": houses,
                    "drives": drives, "trees": trees,
                    "garages_rejected": n_reject,
                    # Both new rejection reasons are reported rather than
                    # swallowed: a thinned-out street is its own defect, and
                    # these are the two numbers that say whether a keep-out or a
                    # catalogue with nothing narrow in it caused one.
                    "keepout_rejected": n_keepout,
                    "size_rejected": n_nofit})
    return out


def stats(parcels):
    n_h = sum(len(p["houses"]) for p in parcels)
    n_t = sum(len(p["trees"]) for p in parcels)
    per = [len(p["houses"]) for p in parcels if p["houses"]]
    hs = [h for p in parcels for h in p["houses"]]
    dens = {}
    for p in parcels:
        k = p.get("density")
        dens[k] = dens.get(k, 0) + 1
    arch = {}
    for h in hs:
        k = h.get("archetype")
        arch[k] = arch.get(k, 0) + 1
    depths = [h["lot_depth"] for h in hs if "lot_depth" in h]
    return {"houses": n_h, "trees": n_t,
            "blocks_built": len(per), "blocks": len(parcels),
            "houses_per_built_block": (sum(per) / len(per)) if per else 0.0,
            "garages": sum(1 for h in hs if h.get("garage")),
            "garages_rejected": sum(int(p.get("garages_rejected", 0))
                                    for p in parcels),
            "keepout_rejected": sum(int(p.get("keepout_rejected", 0))
                                    for p in parcels),
            "size_rejected": sum(int(p.get("size_rejected", 0))
                                 for p in parcels),
            "fenced_houses": sum(1 for h in hs if h.get("fence_segs")),
            "fence_segs": sum(len(h.get("fence_segs") or ()) for h in hs),
            "density_blocks": dens, "archetypes": arch,
            "mean_lot_depth": (sum(depths) / len(depths)) if depths else 0.0}
