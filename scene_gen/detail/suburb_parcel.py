"""
suburb_parcel.py — lots, houses, drives and planting on a `suburb_net` block.

WHAT THIS IS FOR
----------------
`suburb_net` produces a street graph and the block polygons between the
streets. That is enough to judge the ROADS but not the FABRIC: whether a lot of
sane width fits along the frontage, whether the deep middle of a block is
stranded, whether cul-de-sac bulbs get the ring of houses that justifies them.

THE ORGANISING IDEA: LOTS ARE HUNG OFF FRONTAGE, NOT CUT OUT OF AREA
--------------------------------------------------------------------
A suburban parcel is defined by the street it faces. So the block boundary is
walked as ARCLENGTH and a lot is issued every `lot_width_m`, each one taking
its orientation from the frontage tangent at that station:

    house yaw     = frontage tangent      (parallel to the street, always)
    house centre  = station + inward normal * (setback + depth/2)

which is why houses follow a curving street round its curve instead of staying
axis-aligned to the world, as they did on the old rect-based generator where
the block edge was always axis-aligned.

The deep interior of a block is deliberately left empty of HOUSES. That is not
a gap in the algorithm — it is the back yards, and on a real suburban block the
middle is exactly that: lawn, fence lines and trees. Each lot carries its
rectangle (`lot_corners`, `lot_depth`) so those fences can be drawn, and so a
consumer can tell a 21 m yard from a 38 m one.

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
driving diameter), but the block polygon is built from the street faces and
carries no record of it, so a lot issued beside a bulb is platted on the road
and passes every containment test in here while doing it. `cfg["keepout_discs"]`
is how the caller — which has the net, and so the real per-edge radius — says
where that pavement is; house, garage, fence and tree are all refused or cut
back out of it.

WHAT IT DOES NOT DO
-------------------
No USD, and no asset library: this module cannot look a house up. What it can
do is USE a measurement someone else took — `cfg["house_sizes"]` sites the
caller's measured footprints instead of nominal rectangles and stamps the entry
used on each house as `size_index`, so the caller can place that exact asset.
Absent the key the footprints are nominal rectangles from
`house_w_m`/`house_d_m`, and every overlap test below is then only as true as
that guess — which is the honest reading of a house that clears the test here
and overlaps its neighbour on screen.
"""

import math

from layout.suburb_net import (_add, _sub, _mul, _dot, _unit, _perp, _dist,
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
# built by one developer in phases, so a run of lots shares a package: same
# house, same garage, same fence down a whole street, and the next phase does
# something slightly different. Drawn per lot instead, a generated suburb reads
# as a shuffled catalogue rather than as a subdivision — so archetypes are
# assigned in RUNS along each block's frontage.
# A package couples HOW BIG the house is to WHAT IS ON THE PLOT, which is how
# a real street reads: the small houses are the bare ones and the big ones carry
# everything. The range is wide on purpose — measured over a full plat it puts
# `plain` at a 144 m2 mean and `large` at 314 m2, which is most of the
# catalogue's 100-300 m2 span. Narrower and the correlation is invisible,
# because the district `size` bias multiplies in on top and washes it out. `plain` used to sit at scale 1.00, so "no fence, no garage" landed
# on mid-sized houses and the correlation was invisible; it is now genuinely the
# small end. `pool` is the top of that same ladder.
ARCHETYPES = {
    # name          weight  garage   fence  pool   lot scale
    "plain":       {"w": 0.30, "garage": 0.0, "fence": 0.0, "pool": 0.0,
                    "scale": 0.55},   # small, bare plot
    "fenced":      {"w": 0.22, "garage": 0.0, "fence": 1.0, "pool": 0.0,
                    "scale": 0.80},
    "garage":      {"w": 0.24, "garage": 1.0, "fence": 0.0, "pool": 0.0,
                    "scale": 1.05},
    "full":        {"w": 0.18, "garage": 1.0, "fence": 1.0, "pool": 0.0,
                    "scale": 1.35},
    # The big ones: wider lot, deeper setback, everything on it — pool included.
    "large":       {"w": 0.06, "garage": 1.0, "fence": 1.0, "pool": 1.0,
                    "scale": 1.90},
}


# WHICH PACKAGE A DISTRICT BUILDS. The archetype was drawn independently of
# the density class, so `large` (garage, fence, pool, biggest house) landed on a
# 1346 m2 lot against `plain`'s 1139 — the two were uncorrelated and the estate
# districts got roomier lots carrying the same houses.
#
# The split is measured. `tools/osm_lot_stats.py` over five ordinary suburbs and
# five large-lot towns: at a >=39 m nearest-house-spacing threshold (2x the
# ordinary median of 19.5 m) 4.3% of ordinary-suburb dwellings qualify as
# estate-sized, against 63% in the estate towns. So the authored weights are
# right for a MIXED tract and badly wrong for a district that is meant to BE
# low-density.
ARCH_BY_DENSITY = {
    "tight":  {"plain": 0.45, "fenced": 0.30, "garage": 0.15,
               "full": 0.09, "large": 0.01},
    "normal": None,          # the weights authored on ARCHETYPES
    "loose":  {"plain": 0.15, "fenced": 0.18, "garage": 0.25,
               "full": 0.30, "large": 0.12},
    "estate": {"plain": 0.05, "fenced": 0.07, "garage": 0.10,
               "full": 0.18, "large": 0.60},   # 0.60 ~ the measured 63%
}


def _draw_archetype(rng, density=None):
    names = list(ARCHETYPES)
    over = ARCH_BY_DENSITY.get(density) if density else None
    wts = [float(over[n]) if over else float(ARCHETYPES[n]["w"]) for n in names]
    tot = sum(wts)
    if tot <= 0.0:
        return "normal" if "normal" in ARCHETYPES else names[0]
    r = rng.random() * tot
    for n, w in zip(names, wts):
        r -= w
        if r <= 0.0:
            return n
    return names[-1]


# ---------------------------------------------------------------------------
# density classes
# ---------------------------------------------------------------------------
# Same argument as ARCHETYPES one level up. A zoning map assigns a minimum lot
# width to a DISTRICT, not to a house, and the district boundary runs along
# streets and platted block lines. US single-family districts bottom out around
# a 40-50 ft minimum width and top out at 100 ft or more in a large-lot/estate
# district, with the front setback moving with it (20-25 ft tight, 40-50 ft
# estate). So the change happens AT THE BLOCK, and this is drawn ONCE PER BLOCK:
# per lot would put an estate lot between two starter lots.
#
#   "lot"     multiplies the sampled frontage width
#   "setback" multiplies the sampled front setback
#   "depth"   multiplies the sampled lot depth — gently. Depth is set by how the
#             surveyor tiered the block, so it moves far less than width does
#             across districts.
#
# The HOUSE is deliberately not scaled. A wider lot in a low-density district
# buys side yard and back yard, not a bigger building — the same builder's
# 1,600 sq ft plan goes up on the 40 ft lot and on the 100 ft lot. House size is
# the archetype's job (ARCHETYPES[...]["scale"]); driving both from width would
# cancel the effect width is there to produce.
# `size` biases WHICH house is drawn from the catalogue, on top of what merely
# FITS. Without it a district changed only its lot width and setback, so a
# roomy estate block got the same mean footprint as a tight one — the lots grew
# but the houses on them did not, which is not what a bigger-lot district looks
# like. It feeds `_pick_size`'s exponent, so it selects up the area ranking
# rather than scaling a measured asset (which would give a 9 m model 9 m doors).
DENSITY = {
    # name           weight   lot width   setback       depth        house size
    "tight":       {"w": 0.28, "lot": 0.78, "setback": 0.80, "depth": 0.85,
                    "size": 0.72},
    "normal":      {"w": 0.44, "lot": 1.00, "setback": 1.00, "depth": 1.00,
                    "size": 1.00},
    "loose":       {"w": 0.22, "lot": 1.45, "setback": 1.25, "depth": 1.15,
                    "size": 1.30},
    # MEASURED, not guessed — `tools/osm_lot_stats.py`, five ordinary suburbs
    # against five large-lot towns, 5.76 km2 each. Nearest-house spacing is the
    # anchor because it reproduces a known plat: Levittown measures 18.1 m
    # against its platted 60 ft (18.29 m), 1% out. Estate/suburb spacing is
    # 49.4 / 19.5 = 2.53x, footprint area 2.02x, setback-to-centreline 1.86x.
    #   lot     2.20 -> 2.4   (measured 2.53, shaded down)
    #   setback 1.60 -> 2.4   (measured 2.53 — the old value was ~60% short)
    #   depth   1.35 -> 1.8   (measured 1.86, and that is a LOWER bound: the
    #                          land figure includes streets, which scale with
    #                          frontage rather than with area)
    #   size    1.75 -> 1.95  (footprint 2.02x)
    "estate":      {"w": 0.06, "lot": 2.40, "setback": 2.40, "depth": 1.80,
                    "size": 1.95},
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


def _density_field(blocks, rng, cfg):
    """A map of density DISTRICTS over the whole tract, not a per-block draw.

    WHY THIS EXISTS. Density was one independent draw per block, so a tight
    block could sit between two estate blocks on the same street. That reads as
    noise rather than as variety — the same lesson the rect-layout module
    learned and recorded, which the graph layout never inherited. Real tracts
    are not like that, and the reason is legal rather than aesthetic: plats are
    recorded separately, so lot width and setback change AT the line between
    two subdivisions and nowhere else.

    So: seeds on a jittered grid `neighbourhood_m` apart, nearest seed wins.
    A district is then a contiguous multi-block patch with a ragged boundary
    you can walk to, which is what a subdivision boundary actually looks like.

    Returns ``(x, y) -> class name``. `neighbourhood_m: 0` restores the old
    per-block draw, so a preset that wants the noise can still ask for it.
    """
    patch = float(cfg.get("neighbourhood_m", 0.0) or 0.0)
    mix = cfg.get("density_mix")
    if patch <= 0.0:
        return lambda _p: _draw_density(rng, mix)

    pts = []
    for blk in blocks:
        poly = blk["poly"] if isinstance(blk, dict) else blk
        pts.extend(poly)
    if not pts:
        return lambda _p: _draw_density(rng, mix)
    x0 = min(q[0] for q in pts); x1 = max(q[0] for q in pts)
    y0 = min(q[1] for q in pts); y1 = max(q[1] for q in pts)

    seeds = []
    nx = max(1, int(round((x1 - x0) / patch)))
    ny = max(1, int(round((y1 - y0) / patch)))
    for i in range(nx + 1):
        for j in range(ny + 1):
            # Jittered by up to half a cell: a pure grid gives straight
            # district edges that read as a chessboard from the air.
            cx = x0 + (i + rng.uniform(-0.35, 0.35)) * (x1 - x0) / max(nx, 1)
            cy = y0 + (j + rng.uniform(-0.35, 0.35)) * (y1 - y0) / max(ny, 1)
            seeds.append((cx, cy, _draw_density(rng, mix)))

    def at(p):
        best, bd = seeds[0], float("inf")
        for sd in seeds:
            d = (sd[0] - p[0]) ** 2 + (sd[1] - p[1]) ** 2
            if d < bd:
                bd, best = d, sd
        return best[2]
    return at


DEFAULTS = {
    # DISTRICT SIZE. How far apart the density seeds sit, so how big a patch of
    # one lot size is. ~300 m is a few blocks — a plat phase. 0 disables the
    # map and returns to an independent draw per block.
    "neighbourhood_m": 300.0,
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
    # RetroNeighborhood SM_House_* that `suburban.yaml` flags as unverified,
    # while this module sites boxes from 7.0 to 21.3 m across. A lot that
    # reserved 9 m and receives a 12 m asset overlaps its neighbour on screen
    # having passed every test in here; no arithmetic fix reaches that, because
    # the arithmetic was right about the wrong box.
    #
    # MEASURED IN, INDEX OUT. The entry chosen is stamped on the house as
    # `size_index`; it is only a fix if the caller places THAT asset.
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
    # Rear space a pool plot asks for: 2.5 m walk + a 4 m-deep
    # pool + 2.5 m walk, with a little margin.
    "pool_rear_m": 9.5,
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
    # Detached double garage: 20 x 21 ft, the standard two-car box. NOTHING IS
    # BUILT IN THIS BOX any more — `suburb_scene.house_catalogue` folds the
    # garage wing into the measured house footprint, so what renders comes from
    # `size_index`. The box survives as RESERVED GROUND: it blocks later houses,
    # cuts fences at its wall, and sets where the drive and the front-fence gap
    # go. Sizing it is therefore still a geometry decision, not decoration.
    "garage_w_m": 6.0,
    "garage_d_m": 6.5,
    "garage_gap_m": 0.8,         # breezeway between house and garage
    # HOW FAR BACK OFF THE KERB THE FENCE PERIMETER STARTS, and it is not zero.
    # `suburb_net.blocks_from_faces` insets every block face by exactly half
    # that side's carriageway, so the block boundary IS the kerb line — and
    # `suburb_scene.build_frontage` then lays its sidewalk ring 0.8 m and its
    # lamps/hydrants/bins 1.6 m INSIDE that same boundary (`furnishing_inset_m`
    # 1.6, halved for the paving). A fence struck on the raw front lot line
    # therefore stands in the gutter or on the pavement: measured on seed 3,
    # 965 of 1,803 front-fence modules (53%) were inside a carriageway by
    # `_RoadIndex`. 2.5 m clears the 1.6 m furnishing line by most of a metre,
    # which is also where a real plat puts the fence — behind the utility
    # strip, not on the kerb. The SIDE runs start from the same inset line, so
    # the perimeter still closes at its two front corners.
    "fence_front_inset_m": 2.5,
    # Planting. `suburb_yardplan` owns yard planting; this module keeps only the
    # street rhythm along the verge. `back_trees_per_100m2` is read only under
    # `own_yard_trees`, and NOTHING reads `front_tree_chance` — both are kept
    # for callers that use this module without yardplan.
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
        for i in range(2):
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

    Same contract as :func:`_clip_seg` one level down — a fence stops at the
    thing it cannot stand in — with a circle in place of a building. Solving
    |a + d*t - c| = r for the two crossings is exact, where sampling the run
    would let a short chord through. The middle case gives up for the same
    reason it does there: a bulb biting the centre of a lot line would leave two
    stubs, and a fence in two pieces with a road through it is worse than none.
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
    a 12 m model scaled down to 9 m is a model with 9 m doors. So the fit is a
    FILTER on the catalogue instead, which is what a real plat does and is why
    narrow lots carry narrow houses; only when NOTHING fits does the lot go
    unbuilt. The criterion is the same half-gap :func:`parcel_blocks` gives the
    overlap test, deliberately — a stricter one here would refuse a house the
    overlap test would have let stand, and pay for it in gaps down the street.

    SIZE CLASS. `ARCHETYPES[...]["scale"]` cannot resize a measured asset for
    the same reason, so it selects rather than scales: rank the fitting entries
    by footprint AREA and bias the draw up that ranking. Scale is linear and
    area goes as its square, hence the exponent — 1.00 leaves the draw uniform
    over everything that fits, 1.42 ("large") pulls its mean to about the 74th
    percentile of it. Biasing rather than slicing matters: a hard band would cut
    the top of the catalogue out of the 52% of lots at scale 1.00, and those
    models would never be built.
    """
    fit = [i for i in order if sizes[i][0] <= max_w]
    if not fit:
        return None
    q = rng.random() ** (1.0 / (scale * scale))
    return fit[min(len(fit) - 1, int(q * len(fit)))]


def _probe_depth(poly, p, n, u, width, lo, hi, step=1.0):
    """Trim a wanted depth *hi* to what the block will actually give.

    The `min_lot_depth_m` gate walked outward instead of tested once: step
    inward along *n* and keep the deepest station whose rear corners are all
    still inside the block. Stopping at the FIRST failure matters — a re-entrant
    block can put polygon interior back under the ray on the far side of a neck,
    and a lot may not jump the gap and take its back yard out of the next
    street's frontage. Returns *hi* when nothing gets in the way, and never less
    than *lo*, so a lot that only just cleared the depth gate still gets its
    minimum rectangle rather than a sliver.
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


def _line_dupe(segs, a, b, tol=1.20, cos_tol=0.92, min_ov=0.5):
    """The standing fence line this boundary duplicates, or ``None``.

    :func:`_seg_key` catches only the exact case, and the exact case is the rare
    one: two lots agree on the front anchor only to within the chord error of a
    curving block face and disagree outright on the far end, because each platted
    its own lot depth. Keyed on endpoints, the shared line gets two fences laid
    down it — a 26 m one and a 34 m one, overlapping for 26 m.

    So match the LINE: near-parallel (either direction), overlapping along it by
    more than *min_ov*, and no more than *tol* apart WHERE THEY OVERLAP —
    measured at the middle of the shared stretch, not at an endpoint, because
    two boundaries that share a corner and splay apart by a few degrees are two
    boundaries while two that run 20 m side by side 10 cm apart are one. Fences
    that merely MEET end to end project to a zero-length overlap and are left
    alone; a block corner is safe because its two faces are tens of degrees apart.

    *tol* IS 1.2 m, NOT 0.6. It is the "same boundary" threshold, and the thing
    it is really measuring is how far two neighbours' platted side lines can
    drift apart — each is struck from its own frontage station, so on a curving
    face they diverge by the chord error over the lot depth. At 0.6 m a
    measurable tail of pairs fell just outside and BOTH got fenced, which is two
    fences a metre apart down one side yard. Nothing legitimate lives in the
    0.6-1.2 m band: the narrowest real gap between two distinct boundaries is a
    side yard, and `house_gap_m` is 6 m.

    Returns the matching ``fence_lines`` entry (so the caller can extend it in
    place) rather than a bool, because dropping the duplicate outright is what
    left the deeper lot's yard half open — see the merge in `parcel_blocks`.
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
            return entry
    return None


def _line_union(q0, q1, a, b):
    """The shortest segment along q0->q1 that covers both it and a->b.

    Projected onto the STANDING line's own direction, not remitred to some
    average: the fence that is already there is the one that stays, and all this
    does is lengthen it. Two neighbours' side lines are parallel to within
    `_line_dupe`'s tolerance by the time this is reached, so the projection
    loses nothing worth keeping.
    """
    d = _unit(_sub(q1, q0))
    ts = (0.0, _dist(q0, q1), _dot(_sub(a, q0), d), _dot(_sub(b, q0), d))
    return _add(q0, _mul(d, min(ts))), _add(q0, _mul(d, max(ts)))


def _relay(entry, cut, segs, tag):
    """Move a registered boundary onto *cut*, still as exactly ONE run.

    The registry entry carries a back-pointer into the list that owns its
    segment (``entry[2]``) and the tuple sitting in it (``entry[3]``) — the same
    handle :func:`_clip_standing` re-cuts through, so extending a neighbour's
    fence and cutting one at a new garage go through the same two lines of
    book-keeping. ``entry[2] is None`` means the line was registered but never
    fenced (a building stood on all of it when the neighbour platted); this lot
    may take it over, which is strictly better than leaving the boundary bare.
    """
    if entry[2] is None:
        entry[0], entry[1] = cut
        entry[2] = segs
        entry[3] = (cut[0], cut[1], tag)
        segs.append(entry[3])
        return
    i = entry[2].index(entry[3])
    entry[0], entry[1] = cut
    entry[3] = (cut[0], cut[1], entry[3][2])
    entry[2][i] = entry[3]


def _clip_seg_cross(a, b, q0, q1, pad=0.35):
    """Stop a fence run where it CROSSES one that is already standing.

    A fence butts into another fence; it does not pass through it. Lots hung off
    two faces of the same block genuinely overlap in the corner (see
    :func:`_clip_seg`), and where they do, one lot's rear line runs straight
    through its neighbour's side line — measured on seed 3, 57 module pairs
    interpenetrating by more than 0.25 m, up to 2.4 m, all of them at block
    corners or on the inside of a curve. Cutting the newcomer at the crossing is
    the honest reading of "that ground is already fenced".

    Only a PROPER interior crossing counts. A lot's own three runs meet at its
    two rear corners and the front runs meet the side runs at the front corners;
    those touch at an endpoint, and *pad* keeps them out of it — a T-join within
    35 cm of either end is a join, not a crossing.

    Returns the longer surviving piece, or ``None`` if neither is worth keeping.
    """
    d = _sub(b, a)
    e = _sub(q1, q0)
    den = d[0] * e[1] - d[1] * e[0]
    if abs(den) < 1e-9:                     # parallel: `_line_dupe`'s business
        return a, b
    w = _sub(q0, a)
    t = (w[0] * e[1] - w[1] * e[0]) / den   # along a->b, normalised
    s = (w[0] * d[1] - w[1] * d[0]) / den   # along q0->q1, normalised
    ln, qn = _dist(a, b), _dist(q0, q1)
    if not (pad < t * ln < ln - pad) or not (pad < s * qn < qn - pad):
        return a, b
    hit = _add(a, _mul(d, t))
    if t * ln >= ln - t * ln:
        return (a, hit) if t * ln >= 2.0 else None
    return (hit, b) if ln - t * ln >= 2.0 else None


def _seg_box(a, b, t=0.04):
    """A segment as a hairline box, so :func:`_obb_overlap` can test it."""
    d = _unit(_sub(b, a))
    m = _mul(_add(a, b), 0.5)
    return _corners(m[0], m[1], _dist(a, b), t, d[0], d[1])


def _clip_seg(a, b, box, pad=0.10):
    """Trim a fence run back to where a building stands on it.

    A fence butts into a wall, it does not pass through it, and two boxes stand
    on these lines. A garage on the side line is the boundary structure for its
    own length — half the reason they were built on the line — so the run is cut
    at it rather than deleted and the back of the side yard stays fenced. The
    other case is a NEIGHBOUR's house: lots are hung off frontage independently,
    so two lots issued from perpendicular faces of the same block genuinely
    overlap deep in the corner and one has platted its back yard across the
    other's living room. Cutting at the wall is the honest reading of that; the
    alternative is a full lot-versus-lot packing pass, which is another module.

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


def _front_runs(lo, hi, gaps, min_len=1.5):
    """`[lo, hi]` less every `(centre, half_width)` opening in *gaps*.

    The front fence has to break wherever something crosses the front lot line
    — the drive, the walk — and those are the house's openings, not the plat's
    guess at them. Overlapping gaps merge; a stub shorter than *min_len* is
    dropped rather than emitted as a two-panel orphan.
    """
    runs = []
    x = lo
    for a, b in sorted((c - h, c + h) for c, h in gaps):
        if b <= lo or a >= hi:
            continue
        if a - x >= min_len:
            runs.append((x, a))
        x = max(x, b)
    if hi - x >= min_len:
        runs.append((x, hi))
    return runs


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
    ``garage`` (``None`` or a box in the same shape as the house — RESERVED
    GROUND, not an asset request; see `garage_w_m`). With ``cfg["house_sizes"]``
    supplied, ``size_index`` says which measured entry the footprint came from
    and the caller is expected to place that asset.
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
    POOL_REAR_M = float(c.get("pool_rear_m", 9.5))
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
    # WHERE THIS HOUSE'S FRONT OPENINGS ARE, asked rather than guessed.
    # `fn(size_index, house_w) -> [(offset_along_u, half_width, kind), ...]`,
    # kind being "door" or "drive". The caller knows which asset it will put on
    # this lot and where that asset's front step and garage door are; this
    # module cannot. Without it the front fence broke at `house_w * 0.30` on a
    # coin-flip side, which on the kit houses was the wrong side of the house
    # 55% of the time — a fence straight across the drive.
    #
    # A "drive" opening also means THE ART HAS ITS OWN GARAGE, so no detached
    # box is reserved for it. Reserving one anyway is a phantom building: it ate
    # a side yard, clipped the fence around itself and aimed the drive at a
    # structure that was never placed — 239 of them on seed 3.
    #
    # None keeps the old behaviour for sets that cannot answer.
    openings_fn = c.get("front_openings")

    dens_at = _density_field(blocks, rng, c)

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

        # The district this block sits in — read off the density MAP at the
        # block's centre, so neighbouring blocks agree and the tract has
        # recognisable sub-areas instead of per-block noise.
        bcx = sum(pxs) / len(pxs)
        bcy = sum(pys) / len(pys)
        dens = dens_at((bcx, bcy))
        d_lot = float(DENSITY[dens]["lot"])
        d_set = float(DENSITY[dens]["setback"])
        d_dep = float(DENSITY[dens].get("depth", 1.0))
        d_size = float(DENSITY[dens].get("size", 1.0))
        # The depth CAP is a policy number, so it scales with the district too.
        # Leaving it at the tract-wide 40 m was silently un-doing the estate
        # multipliers: a 2.4x setback plus a 20 m house needs 50 m of lot
        # before any rear yard exists, so every estate lot hit the cap and got
        # a NEGATIVE back garden. That is why estates had 49% large houses and
        # not one pool. `_probe_depth` still bounds this by real geometry.
        blk_max_depth = max(min_depth, max_depth * d_dep)

        perim = polyline_length(ring)
        if perim < 40.0:
            out.append({"block": poly, "density": dens, "houses": [],
                        "drives": [], "trees": [], "garages_rejected": 0,
                        "keepout_rejected": 0, "size_rejected": 0})
            continue

        houses, drives, trees = [], [], []
        garages = []                 # reserved ground; nothing is built in it
        fenced = set()               # boundary identities already fenced
        fence_lines = []             # ...and the same as lines, for near-misses
        n_reject = 0
        n_keepout = 0                # lots lost to a turnaround
        n_nofit = 0                  # ...and to a frontage nothing measured fits
        s = rng.uniform(0.0, lw[0])
        guard = 0
        run_lo, run_hi = _rng_pair(c.get("archetype_run", [4, 9]), (4.0, 9.0))
        arch = _draw_archetype(rng, dens)
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
                arch = _draw_archetype(rng, dens)
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
                                    width - gap * 0.5,
                                    float(spec["scale"]) * d_size)
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
            # Frontage width along the kerb, `lot_depth` inward, left/right
            # taken against the frontage tangent: left = -u, right = +u. Never
            # shallower than the house plus a rear yard: a deep setback and a
            # large archetype together put the back wall 35 m in, and a rear lot
            # line in front of its own house would fence the building in half.
            # REAR ALLOWANCE. 4 m is enough for a yard; a pool needs room for
            # itself plus a walkable margin at both ends, so a plot whose
            # package wants one ASKS FOR THE DEPTH UP FRONT. Requesting it
            # afterwards is how every pool ended up not fitting: the lots were
            # granted 4 m of rear and the pool needed 9.
            want_pool = spec.get("pool", 0.0) > 0.0
            rear_need = POOL_REAR_M if want_pool else 4.0
            want_depth = min(blk_max_depth,
                             max(min_depth, setback + h_d + rear_need,
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

            # Does the ART already have a garage? Asked before the box is
            # reserved, because the answer decides whether to reserve one.
            art_gaps = (openings_fn(size_i, h_w)
                        if (openings_fn is not None and size_i is not None)
                        else None)
            art_garage = any(len(g) > 2 and g[2] == "drive"
                             for g in (art_gaps or ()))

            garage = None
            if spec["garage"] > 0.0 and not art_garage:
                g_off = side * (h_w / 2.0 + g_sep + g_w / 2.0)
                gc = _add(p, _add(_mul(u, g_off),
                                  _mul(n, setback + g_d / 2.0)))
                g_corners = _corners(gc[0], gc[1], g_w, g_d, u[0], u[1])
                # ON or OVER the side lot line is allowed, and is the normal
                # case: a 6 m garage plus a 12 m house does not fit inside a
                # 20 m frontage, which is why detached garages all over
                # pre-zoning plats sit hard on the line. Requiring containment
                # instead refuses the box on every lot narrower than about 26 m,
                # i.e. nearly all of them. What is NOT allowed is a box inside a
                # building, so the two structural tests are exactly the ones the
                # house passes; the FENCE gives way instead, cut at the garage
                # wall by `_clip_seg` below.
                # The bulb bites harder here than on the house: a garage sits at
                # the setback line with no front yard in front of it, so a lot
                # whose house cleared the turnaround can still park on it.
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

            # WHERE THE PAVING CROSSES THE FRONT LOT LINE, as (offset along u,
            # half width) — one answer, used by the fence below, published on
            # the house record for the yard pass, and matching the runs
            # `suburb_scene` actually draws. Everything that has to stay off the
            # drive and the walk now reads the same numbers instead of each
            # re-deriving them from a guess.
            front_gaps = [(g[0], g[1]) for g in (art_gaps or ())]
            if not front_gaps:
                front_gaps = [((g_off if garage is not None
                                else side * h_w * 0.30),
                               (g_w if garage is not None else dw) / 2.0 + 0.6)]

            # --- fences -------------------------------------------------------
            # ONE PERIMETER, PULLED BACK OFF THE KERB. `fence_front_inset_m`
            # says why the front corners are not `fl`/`fr`: the block boundary
            # is the kerb line and the pavement is laid inside it, so a fence on
            # the raw front lot line stands in the road. The SIDE runs start
            # from the same inset points, which is what keeps the four runs a
            # closed rectangle rather than three sides and a floating stub.
            #
            # Sides and rear are the tall run; only the two full packages also
            # close the front, and that one carries the "low" tag — every US
            # code that caps fence height caps the front yard at 3-4 ft against
            # 6-8 ft elsewhere. The tag is a HEIGHT LIMIT for the renderer to
            # honour, not a second asset pool: `suburb_scene` draws one asset
            # per house and simply skips a "low" run its asset is too tall for.
            fence_segs = []
            if spec["fence"] > 0.0:
                f_in = float(c["fence_front_inset_m"])
                pf = _add(p, _mul(n, f_in))
                ffl = _add(fl, _mul(n, f_in))
                ffr = _add(fr, _mul(n, f_in))
                cand = [(ffl, rl, "privacy"), (ffr, rr, "privacy"),
                        (rl, rr, "privacy")]
                blockers = [corners] + [h["corners"] for h in houses] \
                    + [g["corners"] for g in garages]
                if arch in ("full", "large"):
                    # Front run, broken wherever something crosses it. A fence
                    # across your own driveway is worse than no fence.
                    for x0, x1 in _front_runs(-half, half, front_gaps):
                        cand.append((_add(pf, _mul(u, x0)),
                                     _add(pf, _mul(u, x1)), "low"))

                def _cut_run(a, b):
                    """Trim a boundary out of the pavement, the walls and any
                    fence already standing across it — in that order.

                    Pavement first, because it is the cut that can take the
                    whole run: a lot beside a bulb has its two side lines
                    running out of the turnaround, and there is no point asking
                    which wall a fence stops at when the front half of it is on
                    the road. Fences last, because a run already shortened by a
                    wall may no longer cross anything.
                    """
                    cut = (a, b)
                    for (dc, dr) in blk_discs:
                        cut = _clip_seg_disc(cut[0], cut[1], dc, dr)
                        if cut is None:
                            return None
                    for box in blockers:         # includes this lot's own
                        cut = _clip_seg(cut[0], cut[1], box)
                        if cut is None:
                            return None
                    for entry in fence_lines:
                        if entry[2] is None:
                            continue             # registered, nothing standing
                        cut = _clip_seg_cross(cut[0], cut[1],
                                              entry[0], entry[1])
                        if cut is None:
                            return None
                    return cut

                for a, b, tag in cand:
                    k = _seg_key(a, b)
                    if k in fenced:
                        continue
                    fenced.add(k)
                    dupe = _line_dupe(fence_lines, a, b)
                    if dupe is not None:
                        # THE NEIGHBOUR ALREADY FENCED THIS BOUNDARY — but only
                        # to ITS OWN platted depth. Two lots hung off the same
                        # face take 26 m and 34 m of side yard, so dropping the
                        # duplicate outright left 8 m of the deeper lot open and
                        # its rear run starting in mid-air: measured on seed 3,
                        # 1,033 of 1,412 run endpoints (73%) had no other
                        # endpoint within 0.35 m of them. So EXTEND the standing
                        # run over both instead. The boundary is still fenced
                        # ONCE, by one lot, and the deeper lot's rear run now
                        # has something to meet.
                        grown = _cut_run(*_line_union(dupe[0], dupe[1], a, b))
                        # Never shorter than what is already there: the re-cut
                        # sees walls the neighbour's run predates, and losing a
                        # standing fence to gain a longer one is not a trade.
                        if (grown is not None
                                and _dist(*grown) >= _dist(dupe[0], dupe[1])):
                            _relay(dupe, grown, fence_segs, tag)
                        continue
                    cut = _cut_run(a, b)
                    if cut is None:
                        # A building or another fence stands on this boundary
                        # for its whole length. Still registered, so the
                        # neighbour does not come back and fence it either.
                        fence_lines.append([a, b, None, None])
                        continue
                    seg = (cut[0], cut[1], tag)
                    fence_lines.append([cut[0], cut[1], fence_segs, seg])
                    fence_segs.append(seg)

            houses.append({"c": (cx, cy), "w": h_w, "d": h_d, "u": u,
                           # Which package this lot was drawn as. Read by
                           # `stats`; `has_garage`/`has_fence` further down are
                           # recorded for callers and read by nothing in-tree.
                           "archetype": arch,
                           # WHICH measured footprint was sited, as an index
                           # into `cfg["house_sizes"]` (None when the nominal
                           # sampling was used). The caller must place the asset
                           # at this index and no other — the box tested for
                           # overlap here IS that asset, and drawing a different
                           # one puts the module back to guessing.
                           "size_index": size_i,
                           "has_garage": spec["garage"] > 0.0,
                           # In the ART, not as a separate box beside the house.
                           "art_garage": art_garage,
                           "has_fence": spec["fence"] > 0.0,
                           # ...and only if the BLOCK actually granted it.
                           # `_probe_depth` can come back short on a shallow or
                           # awkward block, and a pool half in the next street
                           # is worse than no pool.
                           "has_pool": (spec.get("pool", 0.0) > 0.0
                                        and lot_depth - setback - h_d
                                        >= POOL_REAR_M - 0.5),
                           "lot_width": width,
                           # The lot itself, so the yard can be planted and not
                           # merely recorded: the rectangle, how deep the back
                           # yard actually runs on this block, the fence lines
                           # already deduplicated against the neighbour, and the
                           # garage box (None when it did not fit).
                           "lot_corners": lot_corners,
                           "lot_depth": lot_depth,
                           "front_gaps": front_gaps,
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
            # the house so it lands beside the door rather than through it — and
            # when this lot reserved a garage box, up to that instead, which is
            # where a drive on a real lot goes. Same `side` either way.
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
            # already places a specimen tree per lot; emitting one here too
            # planted every yard twice — measured on the shipped preset, 693
            # front + 858 back trees from this pass on top of yardplan's, 1,551
            # duplicates. This module keeps only the STREET rhythm along the
            # verge, which is the one thing yardplan does not do.

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

        # NO BACK-YARD SCATTER HERE either, same reason: yardplan plants the
        # rear yard. Left as a no-op rather than deleted so the
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
                    # Reported rather than swallowed: a thinned-out street is
                    # its own defect, and these are the two numbers that say
                    # whether a keep-out or a catalogue with nothing narrow
                    # enough in it caused one.
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
