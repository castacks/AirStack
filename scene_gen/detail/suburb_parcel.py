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

A CUL-DE-SAC HEAD IS PLATTED RADIALLY, NOT WALKED
-------------------------------------------------
Keeping things off the paving is only half of it. `suburb_net` splices the
turnaround ARC into the block boundary so the head has frontage at all, and
that frontage turns 330 degrees in about 100 m — which the arclength rule
above handles by giving every station a rectangle on its own tangent, and
neighbouring stations on such an arc are 40 to 80 degrees apart. A real plat
does the obvious thing instead and hangs WEDGE LOTS off the arc: side lines
along the two bounding radii, so they converge on the turnaround centre and are
shared exactly between neighbours; the house on a chord facing that centre; the
drive aimed at the paving; the front fence following the arc rather than
cutting the chord across it. `blk["bulbs"]` is how the head announces itself
and `_lot_jobs` plats every head BEFORE the streets that reach them, because
whichever lot is issued first keeps the ground they both want. See the block
comment above :func:`_bulb_runs`.

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
from detail import row_housing as rh


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
#
# THE FENCE WEIGHTS ARE 1.85x WHAT THE PLAT USED TO DRAW, AND THE NUMBER IS
# MEASURED, NOT CHOSEN. `suburb_scene` now takes a lot's fence away outright
# when it cannot close the back yard (`_FENCE_ON_GARDENLESS_LOT` and the sweep
# beside it), so the share PLATTED and the share you can SEE stopped being the
# same number. On seed 3 the old weights platted 152 fenced lots and 121 of
# those had a back yard to close; after the sweep only 70 kept their fence. The
# ask was to put the visible count back on that 121, so:
#
#     k = 1.00 -> 70 closed     k = 1.60 -> 108     k = 1.85 -> 121
#     k = 1.30 -> 80            k = 1.80 -> 115     k = 1.90 -> 129
#
# 1.85 is where it lands, and the response is chunky rather than smooth (1.75
# comes back 105) because archetypes are assigned in RUNS along a frontage —
# one draw moves a whole terrace, so the count steps in fives and tens. It is
# also mildly SUPERLINEAR, which is the interesting part: a lot's side boundary
# is usually its neighbour's fence, so fencing more lots does not merely add
# fenced lots, it closes yards that were open on one side.
#
# HOW THE WEIGHT MOVES, and it is not a scale-and-renormalise. `plain` and
# `fenced` are the same package with and without a fence, and so are `garage`
# and `full`; the increase is shifted WITHIN those two pairs, split in
# proportion to the headroom each has. That keeps P(garage) and P(pool)
# EXACTLY where they were — scaling the three fenced entries and renormalising
# would have quietly raised the garage rate with them, and the garage rate is
# somebody else's calibration.
#
# ...AND IT IS CAPPED AT 0.95 PER DISTRICT. `loose` starts at 0.60 fenced and
# `estate` at 0.85, so 1.85x asks both for more than one, and letting them
# saturate empties `plain` and `garage` out of those districts entirely — the
# small-bare-plot end of the size ladder these packages exist to encode simply
# stops existing above `normal` density. The ceiling costs nothing measurable
# (seed 3 comes back 121 either way) and leaves one lot in twenty unfenced,
# which is what a low-density district actually looks like.
#
# What it costs elsewhere, measured on seed 3: 589 houses -> 578, mean lot
# 31.0 x 29.6 m -> 30.8 x 29.8 m. The archetypes carry a `scale` as well as a
# fence, so moving weight up the ladder does move the lots — by 1%.
ARCHETYPES = {
    # name          weight  garage   fence  pool   lot scale
    "plain":       {"w": 0.083, "garage": 0.0, "fence": 0.0, "pool": 0.0,
                    "scale": 0.55},   # small, bare plot
    "fenced":      {"w": 0.437, "garage": 0.0, "fence": 1.0, "pool": 0.0,
                    "scale": 0.80},
    "garage":      {"w": 0.066, "garage": 1.0, "fence": 0.0, "pool": 0.0,
                    "scale": 1.05},
    "full":        {"w": 0.354, "garage": 1.0, "fence": 1.0, "pool": 0.0,
                    "scale": 1.35},
    # The big ones: wider lot, deeper setback, everything on it — pool included.
    "large":       {"w": 0.060, "garage": 1.0, "fence": 1.0, "pool": 1.0,
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
#
# EVERY ROW HAS BEEN THROUGH THE SAME 1.85x SHIFT as `ARCHETYPES` above — see
# the note there for where the number comes from and why the weight moves
# within the plain/fenced and garage/full pairs rather than across the table.
# The `large` share is untouched in all four, because it is the one entry set
# from a measurement of its own (the 63% below) rather than from a fence rate.
# The fenced share each row now carries: tight 0.74, normal 0.85, loose 0.95,
# estate 0.95 — the last two at the ceiling.
ARCH_BY_DENSITY = {
    "tight":  {"plain": 0.195, "fenced": 0.555, "garage": 0.065,
               "full": 0.175, "large": 0.010},
    "normal": None,          # the weights authored on ARCHETYPES
    "loose":  {"plain": 0.019, "fenced": 0.311, "garage": 0.031,
               "full": 0.519, "large": 0.120},
    "estate": {"plain": 0.017, "fenced": 0.103, "garage": 0.033,
               "full": 0.247, "large": 0.600},   # 0.60 ~ the measured 63%
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


# The class name a ROW-HOME DISTRICT carries. Not a member of `DENSITY`, and
# deliberately not: every entry there is a set of MULTIPLIERS on a lot width, a
# setback and a lot depth, and a row-home cluster has none of those things —
# it has a pitch, a court and a party gap. Making it a fifth density class
# would have meant inventing four numbers that mean nothing, and every
# `DENSITY[dens]` lookup in this file would have silently used them.
ROW_CLASS = "row"


def _draw_seed_class(rng, mix, row_share, row_cfg):
    """One district seed's class, and what a row district additionally needs.

    Returns ``(name, extra)`` where *extra* is None for the ordinary density
    classes and ``{"mix": ..., "palette_set": ...}`` for a row-home district.

    THE MIX IS DRAWN HERE, ON THE SEED, AND NOT ON THE BLOCK. That is the whole
    of requirement "different row-home areas use different house types": a
    district is a contiguous patch of blocks, so drawing the mix once per seed
    makes every block in one patch part of ONE development, and the next patch
    a different builder's. Drawn per block instead, two adjacent row blocks
    would be a terrace beside a cottage court, which reads as the catalogue
    being dealt out rather than as two developments.

    NOTHING IS DRAWN WHEN `row_share` IS ZERO, and the short-circuit is
    load-bearing rather than tidy: an unconditional `rng.random()` here would
    shift the whole downstream sequence, so every seed's plat would change the
    moment this feature was merged even with the feature switched off.
    """
    if row_share > 0.0 and rng.random() < row_share:
        return ROW_CLASS, {"mix": rh.draw_mix(rng, row_cfg),
                           "palette_set": rh.draw_palette_set(rng, row_cfg)}
    return _draw_density(rng, mix), None


def _density_field(blocks, rng, cfg):
    """A map of DISTRICTS over the whole tract, not a per-block draw.

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

    A district may also be a ROW-HOME DEVELOPMENT rather than a lot size —
    `row_share` of the seeds draw `ROW_CLASS` instead, and carry the row mix
    and colour set the whole patch is built from. See :func:`_draw_seed_class`.

    Returns ``(x, y) -> (class name, extra)``, *extra* being None except on a
    row district. `neighbourhood_m: 0` restores the old per-block draw, so a
    preset that wants the noise can still ask for it.
    """
    patch = float(cfg.get("neighbourhood_m", 0.0) or 0.0)
    mix = cfg.get("density_mix")
    share = max(0.0, min(1.0, float(cfg.get("row_share", 0.0) or 0.0)))
    row_cfg = dict(cfg.get("row_housing") or {})
    if patch <= 0.0:
        return lambda _p: _draw_seed_class(rng, mix, share, row_cfg)

    pts = []
    for blk in blocks:
        poly = blk["poly"] if isinstance(blk, dict) else blk
        pts.extend(poly)
    if not pts:
        return lambda _p: _draw_seed_class(rng, mix, share, row_cfg)
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
            seeds.append((cx, cy) + _draw_seed_class(rng, mix, share, row_cfg))

    def at(p):
        best, bd = seeds[0], float("inf")
        for sd in seeds:
            d = (sd[0] - p[0]) ** 2 + (sd[1] - p[1]) ** 2
            if d < bd:
                bd, best = d, sd
        return best[2], best[3]
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
    # ROW-HOME DEVELOPMENTS. The share of district SEEDS (not of blocks) that
    # build attached rows around a shared parking court instead of detached
    # lots — see `detail/row_housing.py` for what one is and why the dataset
    # wants it. It lives here rather than in that module's own DEFAULTS because
    # it is a question about the DISTRICT MAP, which is this file's: it is
    # drawn by `_density_field` alongside `density_mix` and `neighbourhood_m`,
    # and a reader tuning the mix of fabrics needs the three of them together.
    #
    # 0.0 by default, so no preset gains a row-home area by accident and the
    # `rng` sequence of an existing seed is untouched (see `_draw_seed_class`
    # on why that short-circuit matters). At `neighbourhood_m: 400` a
    # 1600 x 1200 m crop carries ~20 seeds and ~45 developed blocks, so 0.12
    # is two or three row districts covering a handful of blocks — a minority
    # fabric, which is what it is in a real suburb.
    "row_share": 0.0,
    # ...and the cluster GEOMETRY, passed straight through to
    # `row_housing.plan_cluster`, whose own DEFAULTS document every knob with
    # the measurement behind it. ONE key rather than a dozen flat ones on
    # purpose: the party gap, the bay module and the court setback are that
    # module's business and this dict would otherwise become the authority on
    # numbers it does not own — the same mistake `house_sizes` exists to stop
    # this module making about footprints.
    "row_housing": None,
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
    # HOW FAR OFF SQUARE A BLOCK CORNER MAY BE BEFORE ITS LOTS ARE REFUSED.
    #
    # `suburb_net` meets its streets at up to `junction_skew_deg` (38 degrees)
    # off the normal on purpose — real plats meet at 70-110 degrees, not
    # exactly 90 — so the block corners this pass walks past run anywhere from
    # 52 to 128 degrees. That is right for the STREETS and wrong for what gets
    # platted at them: a lot is a RECTANGLE HUNG OFF ONE FRONTAGE, so where two
    # frontages meet, both corner lots claim the ground between them. At a
    # square corner the shared ground is a modest corner square and reads as
    # the ordinary suburban corner it is. Off square it is a whole lot's worth,
    # and what shows on screen is the neighbour's FENCE RUN across your back
    # garden: measured over seeds 3/5/8 before this knob existed, 627 of 2,858
    # fence runs (22%) stood inside a lot that was not their own — 11.8 km of
    # fence, up to 65 m in a single run. That is not a crossing and not a
    # doubling, so `suburb_scene._FenceGrid` and `tools/fence_check` were both
    # blind to it while reporting zero.
    #
    # THE GATE IS DEPARTURE FROM SQUARE, IN EITHER DIRECTION. A 125-degree
    # corner is as skewed as a 55-degree one, and measured over seeds
    # 3/5/8, its lots overlap MORE — the 67 overlapping pairs at 100-130
    # degrees average 246 m2 against the 87 pairs at 20-55 degrees' 152.
    # Both ends, one number.
    #
    # A lot is refused when its rectangle comes within `junction_skew_clear_m`
    # of one already platted across a boundary corner further off square than
    # this. Nothing is refused for merely being near a skewed junction: a lot
    # whose rectangle clears its neighbour's is a lot whose fences clear them
    # too. The refusals are counted as `skew_yield`.
    #
    # MEASURED, sweeping the angle over seeds 3/5/8 on the scene
    # `tools/fence_png.py` builds — houses, then the overlapping corner-lot
    # pairs and the square metres of lot-on-lot overlap left at corners:
    #
    #   this knob   corners kept   houses           pairs   corner overlap
    #   90 (off)    everything     1853              335    68.0k m2
    #   35          55-125         1794   (-3.2%)    259    52.6k
    #   30          60-120         1757   (-5.2%)    214    36.1k
    #   25          65-115         1710   (-7.7%)    172    33.9k
    #   20          70-110         1687   (-9.0%)    150    29.1k
    #
    # 30 IS THE KNEE and is the value here. Going 35 -> 30 buys 24% of the
    # overlap for 2% of the houses; 30 -> 25 buys 3% more for another 2.5% of
    # the houses, and it keeps costing after that. 70-110 is the band the
    # `junction_skew_deg` comment names as what real plats do and would be the
    # principled choice, but it is 9% of the plat for 6 points more overlap
    # than 30 removes, and a street thinned by one house in eleven is its own
    # defect. Set to 90.0 to turn the rule off and get the old plat back.
    #
    # WHAT 30 ACTUALLY GUARANTEES, which is narrower than the table above and
    # is the point: overlap between two lots at a corner MORE than 30 degrees
    # off square goes from 130 pairs / 26.3k m2 to 12 pairs / 209 m2, and the
    # fence runs standing inside a lot across such a corner go from 119 to 8
    # (4.2% of all runs to 0.3%). `tools/fence_check` asserts the second as
    # `trespass_skew_frac`. The rest of the table moves because refusing a lot
    # shifts every `rng` draw after it, not because the rule reached corners
    # near square — it does not, on purpose.
    "junction_skew_clear_deg": 30.0,
    # ...and how much daylight those two lots must keep. This is the *pad*
    # handed to :func:`_convex_overlap`, so 0.0 would mean only "they must not
    # actually overlap" and a positive value asks for a gap as well. The lot
    # lines are where the fences are struck and two fences a foot apart read as
    # one fence drawn twice — `suburb_scene._FenceGrid._DOUBLE_M` calls
    # anything inside 3 m doubled and drops the second — so a lot line that
    # merely grazes its neighbour's still costs a fence. 2.0 m is well under
    # the narrowest side yard `house_gap_m` (6 m) could ever produce, so it
    # cannot refuse a lot that had real room.
    "junction_skew_clear_m": 2.0,
    # Rear space a pool plot ASKS FOR. It used to be the 9.5 m the pool itself
    # occupies (2.5 m walk + a 4 m pool + 2.5 m walk) and that is the wrong
    # question: granted exactly what it occupies, the lot came out fence-pool-
    # fence with no garden round the water, and the trunk of any tree the yard
    # pass could still fit stood within a couple of metres of the coping. 20 m
    # buys the walk, the pool, a rear walk and ~8 m of lawn — the difference
    # between a pool in a back garden and a pool that IS the back garden.
    "pool_rear_m": 20.0,
    # ...and what a pool actually NEEDS, which is a different number and has to
    # stay one. The plat cannot always grant 20 m — a shallow or awkward block
    # is bounded by `_probe_depth` and by the block polygon — and refusing a
    # pool on every lot that came back at 14 would delete most of the 6%
    # package rather than give it a bigger garden. `has_pool` tests against
    # THIS, `modular_house.pool_at` re-clamps the water into whatever rear was
    # actually granted, and the lawn is what varies in between. Matches
    # `POOL_WALK_M + 4 + POOL_REAR_WALK_M` over there.
    "pool_rear_min_m": 9.5,
    # No TREE within this of a pool's water rectangle, measured from the rect
    # EDGE. Not a crown-clearance number: the wildfire pass replaces every tree
    # with a baked burnt archetype that scatters wood debris over a 7.5-10.5 m
    # radius about the trunk (`disaster/vegetation.py` `_DEBRIS`, worst case
    # 10.5 m at `fallen`/`stump`), and that debris is authored as loose
    # geometry with no idea the water is there. At the old generic 2.0 m
    # clearance the rubble landed IN the pool. 11.0 m clears the worst case
    # with half a metre in hand. SHRUBS AND PROPS ARE NOT AFFECTED — they carry
    # no debris, and a planter at the poolside is the point of a poolside.
    # Read by `suburb_yardplan.plan`, `suburb_scene.build_open_planting` and
    # the final sweep in `suburb_scene.generate_suburb_on_stage`; it lives here
    # because this is where the other planting knobs live.
    "pool_tree_clear_m": 11.0,
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


def _convex_overlap(a, b, pad=0.0):
    """Separating-axis test between two CONVEX polygons of any vertex count.

    :func:`_obb_overlap` takes only two normals per shape because a rectangle
    has only two; a wedge lot is a trapezoid and its four edges point four
    ways, so the rectangle version reports a separation that is not there. Same
    contract otherwise, *pad* included — a negative *pad* asks whether they
    overlap by more than that much, which is what "a lot may clip its
    neighbour's corner, but not stand in its garden" needs.
    """
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            ex = poly[(i + 1) % n][0] - poly[i][0]
            ey = poly[(i + 1) % n][1] - poly[i][1]
            if abs(ex) < 1e-12 and abs(ey) < 1e-12:
                continue
            nv = _unit((-ey, ex))
            amin = min(_dot(nv, p) for p in a)
            amax = max(_dot(nv, p) for p in a)
            bmin = min(_dot(nv, p) for p in b)
            bmax = max(_dot(nv, p) for p in b)
            if amax + pad < bmin or bmax + pad < amin:
                return False
    return True


# WHAT COUNTS AS A CORNER OF THE BLOCK BOUNDARY, AND HOW FAR EITHER SIDE OF
# ONE THE SKEW RULE LOOKS.
#
# `junction_skew_clear_deg` is a test on the angle between two lots' frontages,
# and that angle ALONE cannot say what it is looking at. Four different things
# land in the same range:
#
#   two lots on ONE face                 normals parallel     "corner" 180
#   two lots either side of a JUNCTION   normals 90 apart     "corner"  90
#   two lots on a CURVING face, 44 m     normals up to 84     "corner"  96  (!)
#   two lots BACK TO BACK across a       normals opposite     "corner"   0
#     block too shallow for two tiers
#
# The third is the one that bites: `min_radius_m` for a local street is 30 m
# and a loose lot is 44 m wide, so two CONSECUTIVE lots on a bend can have
# their normals 84 degrees apart with no junction anywhere near them, and to a
# test that only sees normals they look exactly like a right-angle corner.
# Refusing them cost 31% of the houses on seeds 3/5/8 — measured, and the
# reason this is a two-part test.
#
# So the pair is qualified on the BOUNDARY first: the two stations must have a
# real corner of the ring between them. A corner is a vertex that turns more
# than `_SKEW_CORNER_TURN_DEG` in one step. `suburb_net` samples a street every
# ~6 m and holds it to a 30 m radius, so a curve turns at most ~11.5 degrees
# per vertex by construction and 20-25 in the measured worst case (the mitres
# `offset_polygon` leaves at a junction blob); a street meeting another turns
# 52-128 at a single vertex. 30 degrees sits in the gap. Vertices closer
# together than `_SKEW_CORNER_MERGE_M` and turning the same way are summed
# first, because a junction is sometimes mitred into two or three vertices a
# metre apart and each carries only a share of the turn.
#
# ...and there has to be AT LEAST ONE of them between the two stations, walked
# the short way. That is the test that takes the same-face case away, and it is
# the only part of this that is clean. Measured over seeds 3/5/8, every
# overlapping lot pair cross-tabulated by corners-between against the corner
# angle, as pairs/m2:
#
#   corners       theta 0-20   20-40    40-70   70-110  110-130  130-165   165+
#     0                  -        -        -        -        -    4/412  194/3602
#     1                  -        -    6/2196 43/10824  30/7423  7/1806   1/106
#     2                  -    8/799  29/4598 69/14221  11/3168    1/288       -
#     3+           11/2251  17/2671 86/16066  20/3679        -        -       -
#
# Read the first row: with NO corner between them, a pair is at 130-180 degrees
# and averages 19 m2 — two lots on one face, overlapping by a sliver where the
# street curves. That is not a corner and must not be refused.
#
# The COUNT beyond one is not clean and is not used. A block ring carries about
# eight detected corners against three to six faces — mitre spikes off
# `offset_polygon`, and tight bends that turn 30 degrees at one vertex — so
# genuine adjacent-face pairs turn up under 2 and 3 as readily as under 1.
#
# TRIED AND REJECTED: an arclength reach instead of a corner count, on the
# reasoning that the two lots either side of a corner are a lot width apart.
# They are not. On a wedge-shaped block two long faces converge at a sharp apex
# and the lots that end up inside each other are 100-260 m apart along the
# boundary and nowhere near the apex — measured on seed 3, a 90 m reach saw
# only 54 of the 108 overlapping corner pairs and 12.2k of their 19.2k m2, and
# the ones it missed were the biggest (a 1,019 m2 pair at 57 degrees, 149 m of
# boundary apart). Counting corners has no length in it at all, and the overlap
# test is its own distance filter: two lots 300 m apart do not overlap.
#
# ...and finally on the ANGLE BAND, which is a guard rather than the rule. Two
# lots BACK TO BACK across a block too shallow for two tiers have their normals
# opposite, which reads as a 0-degree corner and is nothing of the kind — 11
# pairs over the three seeds, bottom-left of the table. Their rear yards really
# do interpenetrate, and this deliberately does not touch it: that is a shallow
# BLOCK, not a skewed junction, and refusing one of them would delete a whole
# street face rather than a corner. Left for whoever fixes the block-depth
# question. The upper limit is the same guard against a corner detected inside
# what is really one face.
_SKEW_CORNER_TURN_DEG = 30.0
_SKEW_CORNER_MERGE_M = 3.5
_SKEW_THETA_MIN = 20.0
_SKEW_THETA_MAX = 160.0


def _turn_corners(poly, cum, runs):
    """Arclengths at which the block boundary turns a real corner.

    Turnaround vertices are excluded outright. A cul-de-sac head IS a corner at
    each end of its arc, but the ground either side of that corner is already
    governed by the wedge machinery — `_lot_jobs` plats the head first and
    `wedge_yield` refuses the stem lot that stands in it — and letting the skew
    rule refuse it as well would double-count the same yield under two names.
    """
    n = len(poly)
    if n < 3:
        return ()
    skip = set()
    for (_spec, i0, i1) in (runs or ()):
        i = i0
        for _ in range(n):
            skip.add(i)
            if i == i1:
                break
            i = (i + 1) % n
        skip.add((i0 - 1) % n)
        skip.add((i1 + 1) % n)
    turn = []
    for i in range(n):
        d0 = _sub(poly[i], poly[i - 1])
        d1 = _sub(poly[(i + 1) % n], poly[i])
        if _dot(d0, d0) < 1e-18 or _dot(d1, d1) < 1e-18:
            turn.append(0.0)
            continue
        d0, d1 = _unit(d0), _unit(d1)
        turn.append(math.degrees(math.atan2(d0[0] * d1[1] - d0[1] * d1[0],
                                            _dot(d0, d1))))
    out, used = [], [False] * n
    for i in range(n):
        if used[i] or i in skip or abs(turn[i]) < 1.0:
            continue
        used[i] = True
        grp, tot, j = [i], turn[i], i
        for _ in range(n):
            k = (j + 1) % n
            if (used[k] or k in skip or abs(turn[k]) < 1.0
                    or turn[k] * turn[i] <= 0.0
                    or _dist(poly[j], poly[k]) > _SKEW_CORNER_MERGE_M):
                break
            used[k] = True
            grp.append(k)
            tot += turn[k]
            j = k
        if abs(tot) >= _SKEW_CORNER_TURN_DEG:
            out.append(cum[max(grp, key=lambda q: abs(turn[q]))])
    return tuple(sorted(out))


def _across_corner(a, b, perim, corners):
    """Is there a corner of the boundary between stations *a* and *b*?

    Walked the SHORT way round. See the block comment above: a pair with none
    is two lots on ONE face and is not this rule's business, whatever angle
    the face has curved through between them.
    """
    if not corners or perim <= 0.0:
        return False
    lo, hi = (a, b) if a <= b else (b, a)
    if hi - lo <= perim - (hi - lo):
        return any(lo < cs < hi for cs in corners)
    return any(cs > hi or cs < lo for cs in corners)


def _corner_deg(n_a, n_b):
    """The angle the two frontages meet at, given their INWARD normals.

    180 for two lots on one face, 90 for a square corner, 52 for the sharpest
    corner `junction_skew_deg` can make and 128 for the bluntest. Reported off
    the normals rather than off the tangents because a tangent is only defined
    up to its sign — `u` is the frontage direction of the walk, so two faces
    walked in opposite senses would compare as their own supplement.
    """
    d = max(-1.0, min(1.0, _dot(_unit(n_a), _unit(n_b))))
    return 180.0 - math.degrees(math.acos(d))


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


def _drive_box(d):
    """A drive record ``{a, b, w}`` as its oriented paving rectangle.

    The SAME rectangle `tools/fence_png` draws and `apply_ground` lays, so "the
    fence stands on the drive" is one claim tested once rather than two modules
    each deriving the asphalt from the run.
    """
    a, b, w = d["a"], d["b"], float(d.get("w", 3.0))
    ln = _dist(a, b)
    if ln < 1e-6:
        return _corners(a[0], a[1], w, w, 1.0, 0.0)
    u_ = _unit(_sub(b, a))
    m = _mul(_add(a, b), 0.5)
    return _corners(m[0], m[1], ln, w, u_[0], u_[1])


def _clip_seg_paving(a, b, box, pad=0.10):
    """Open a fence run where the DRIVEWAY crosses it.

    A fence does not stand across the drive of the house it belongs to. The
    front run has always known that — `_front_runs` punches the opening out of
    it — but a drive crosses the front line at `drive_off`, and on a narrow lot
    that offset is at or past the side line: the drive then runs up the side
    yard and straight through the SIDE fence, which no cut ever looked at.
    Measured on seed 3, 32 modules across 15 drives, 13 of them on a side or
    rear run and the rest on a front run whose opening had been cut in the wrong
    place. Both halves of that are what this closes.

    Same contract as :func:`_clip_seg` — trim to the obstruction, do not pass
    through it — with two differences that come from what a drive is:

      * IT NEVER DELETES A RUN FOR BEING CROSSED IN THE MIDDLE. `_clip_seg`
        refuses a mid-run obstruction because a building in the middle of a
        boundary means the boundary was platted through a house. A drive in the
        middle of a run is ordinary, and the answer there is the longer
        surviving side: a fence that stops at the drive reads correctly, a
        boundary left bare does not.
      * A 1.5 m FLOOR, not 2.0: the piece that survives is the long one by
        construction, and the short stub between a drive and a lot corner is
        still a fence somebody built.

    The apron is the drive's own oriented box, so the opening is as wide as the
    paving and no wider; the 0.6 m breathing room either side is already in
    `drive_half` where the front opening is struck.
    """
    if not _obb_overlap(_seg_box(a, b), box, pad=-pad):
        return a, b
    d = _unit(_sub(b, a))
    ln = _dist(a, b)
    ts = [_dot(_sub(q, a), d) for q in box]
    t0, t1 = min(ts) - pad, max(ts) + pad
    if t0 <= 0.0 and t1 >= ln:
        return None                       # the drive covers the whole run
    head = None if t0 <= 0.0 else (a, _add(a, _mul(d, t0)))
    tail = None if t1 >= ln else (_add(a, _mul(d, t1)), b)
    if head is None:
        return tail if _dist(*tail) >= 1.5 else None
    if tail is None:
        return head if _dist(*head) >= 1.5 else None
    keep = head if _dist(*head) >= _dist(*tail) else tail
    return keep if _dist(*keep) >= 1.5 else None


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


def _extend_to_meet(a, b, lines, blockers, reach=3.0, lateral=0.75):
    """Push a run's ends the last metre or two onto a fence already standing.

    THE GAP BETWEEN TWO NEIGHBOURS' BACK YARDS, which is the one defect in this
    pass a flythrough shows and no invariant caught. Every lot strikes its rear
    corners as ``boundary_station + n * lot_depth`` using ITS OWN inward normal
    and ITS OWN probed depth. Two lots share the side station and agree on
    nothing else: `lot_depth` is probed per lot against the block polygon and
    `n` is the normal at each lot's own midpoint, so on a curving face the two
    rear lines land 1-4 m apart. The side fence between them is deduped to one
    run and `_line_union`'d out to the DEEPER of the two, and the shallower
    lot's rear run then stops in mid-air a metre and a half short of it.
    Measured on seed 3: 22 rear ends within 1.5 m of a neighbour's fence and
    not touching it, plus 18 more within 6 m.

    A surveyor's answer would be to make the two lots agree on the rear line,
    but they legitimately do not — the block is a different depth at each
    station, which is exactly what `_probe_depth` is for. What a BUILDER does is
    run the fence the extra metre until it meets the one already there, and that
    is what this does: cast each end along the run's OWN direction and stop at
    the first standing fence within *reach*.

    Two ways a fence can be met, and both are needed:

      crossing      the standing run is at an angle — a rear line meeting the
                    side line it should have landed on. Ordinary ray/segment
                    intersection.
      collinear     the standing run is the NEIGHBOUR'S REAR FENCE, parallel
                    and a hair off line, so the ray never crosses it. Its
                    nearest endpoint is projected onto the ray instead, and
                    accepted if it lies within *lateral* of it — 0.75 m, which
                    is wider than any module is thick and far under the 6 m
                    side yard that separates two genuinely different lines.

    *reach* is 3.0 m: `_FenceGrid._DOUBLE_M`, the distance below which two
    parallel fences are already held to be one fence drawn twice. Nothing that
    close is a second boundary, so nothing that close can be wrongly joined.

    An extension that would push the run into a wall is refused — the run was
    already cut at the buildings, and gaining a metre of fence is not worth
    putting it through a garage.
    """
    a, b = tuple(a), tuple(b)
    d = _unit(_sub(b, a))
    standing = [(e[0], e[1]) for e in lines if e[2] is not None]
    if not standing:
        return a, b
    out = [a, b]
    for k, (origin, step) in enumerate(((a, _mul(d, -1.0)), (b, d))):
        best = None
        for (q0, q1) in standing:
            e = _unit(_sub(q1, q0))
            den = step[0] * e[1] - step[1] * e[0]
            w = _sub(q0, origin)
            if abs(den) < 1e-6:                       # parallel: project instead
                for q in (q0, q1):
                    t = _dot(_sub(q, origin), step)
                    if not (1e-3 < t <= reach):
                        continue
                    if (_dist(_add(origin, _mul(step, t)), q) <= lateral
                            and (best is None or t < best)):
                        best = t
                continue
            t = (w[0] * e[1] - w[1] * e[0]) / den     # along the ray, in metres
            u_ = (w[0] * step[1] - w[1] * step[0]) / den
            if not (1e-3 < t <= reach):
                continue
            if not (-lateral <= u_ <= _dist(q0, q1) + lateral):
                continue
            if best is None or t < best:
                best = t
        if best is None:
            continue
        grown = _add(origin, _mul(step, best))
        # A wall OR a drive. Gaining a metre of fence is not worth putting it
        # through a garage, and it is certainly not worth putting it back
        # across the driveway `_cut_run` just opened.
        if any(_obb_overlap(_seg_box(out[1 - k], grown), box, pad=-0.10)
               for box in blockers):
            continue
        out[k] = grown
    return out[0], out[1]


# HOW LONG A TIP PAST THE LAST JOIN IS STILL A STUB, and it is measured, not
# judged by eye. See `_trim_stub_tips`. On seed 3 at the shipped weights, 2 m
# trims 30 runs and 34.9 m (median 1.11 m, worst 1.88 m). 4 m trims 66 runs and
# 136.3 m instead — and the extra 36 are not stubs, they are side lines whose
# next corner post is simply further along, so the suburb comes back with 12
# MORE dangling fence ends (56 against 44) and not one additional overshoot
# removed. 2 m takes everything this pass can take.
_STUB_TIP_M = 2.0


def _trim_stub_tips(lines, max_tip=_STUB_TIP_M, tol=0.35, min_len=2.0,
                    cos_tol=0.985):
    """Cut every standing run back to the last fence that meets it.

    THE OTHER HALF OF A T-JOIN, AND NOTHING CUTS IT TODAY. `_clip_seg_cross`
    only ever trims the NEWCOMER, and it deliberately declines to fire at all
    when the meeting is within `pad` of an end — "a T-join within 35 cm of
    either end is a join, not a crossing", which is right for the newcomer and
    leaves the standing run's tip hanging past the join. `_line_union` then
    makes it worse on purpose: a shared side line is grown out to the DEEPER of
    the two lots, so the shallower lot's rear run lands in the middle of it and
    the tip beyond is fence enclosing nobody's garden.

    Measured on seed 3 before this existed: 23 of the 146 dangling fence ends
    sat 0.48-3.61 m (median 1.60 m, 36 m of fence in total) past a point where
    another run's end touched their own line. That is the classic builder's
    tell in reverse — a real fence stops AT the corner post.

    WHAT IT COSTS, STATED. At the fence rate this module now ships it removes
    30 runs' worth of overshoot and the suburb comes back with FOUR MORE
    dangling ends, not fewer (44 against 40). That is not the trim failing: a
    shortened boundary is re-tiled from scratch by `_fence_run` — every module
    in it moves, and the `_FenceGrid` clash trim downstream then lands
    differently — so any change of this size moves the dangling count by a few
    either way. The overshoot it removes is a fact about the plat; the four is
    tiler noise, and the honest reading is that this fixes a real defect and is
    neutral on the metric that happens to be easiest to count.

    A POST-PASS OVER THE BLOCK, NOT A CUT INSIDE `_cut_run`, and that ordering
    is the whole reason it is correct. Inside the lot loop "the last thing that
    meets this run" is a moving target: the deeper neighbour's rear run has not
    been platted yet, so trimming there would cut away exactly the length that
    is about to be needed. Here every lot on the block is issued and the answer
    is a fact. `fence_lines` is per block and so is this.

    *max_tip* is what keeps a legitimately longer side run: past a T-join is
    not the same as past the LAST T-join, and a run carrying on 8 m to reach a
    deeper lot's rear corner has that corner's own join further along it, so
    the tip measured here is small by construction whenever the run is right.

    *cos_tol* excludes a run that is COLLINEAR with this one. A neighbour's run
    ending in the middle of this line is not a corner post, it is the same
    boundary fenced twice — `_line_dupe`'s business, and cutting here would
    turn one long fence into a shorter one for no gain.
    """
    n = 0
    live = [e for e in lines if e[2] is not None]
    for entry in live:
        a, b = entry[0], entry[1]
        ln = _dist(a, b)
        if ln < min_len:
            continue
        d = _unit(_sub(b, a))
        ts = []
        for other in live:
            if other is entry:
                continue
            oe = _unit(_sub(other[0], other[1]))
            if abs(_dot(d, oe)) >= cos_tol:
                continue
            for q in (other[0], other[1]):
                t = _dot(_sub(q, a), d)
                if not (0.0 <= t <= ln):
                    continue
                if _dist(q, _add(a, _mul(d, t))) <= tol:
                    ts.append(t)
        if not ts:
            continue
        hi, lo = max(ts), min(ts)
        cut_a, cut_b = a, b
        if tol < ln - hi <= max_tip and hi >= min_len:
            cut_b = _add(a, _mul(d, hi))
        if tol < lo <= max_tip and _dot(_sub(cut_b, a), d) - lo >= min_len:
            cut_a = _add(a, _mul(d, lo))
        if cut_a is a and cut_b is b:
            continue
        _relay(entry, (cut_a, cut_b), None, None)
        n += 1
    return n


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


def _clip_standing(lines, box, clip=_clip_seg):
    """Re-cut fences already standing where a new garage — or DRIVE — landed.

    Lots are issued along the frontage in order, so the neighbour fenced the
    shared line before this lot existed and could not have known a garage was
    coming. The line stays REGISTERED in *lines* either way, so no third lot
    lays another one down it.

    THE DRIVE NEEDS THE SAME TREATMENT, and not having it is why the driveway
    cut did not close all the way. `_cut_run` opens a fence for every apron
    ALREADY on the block, which is the right rule for the lot laying the fence
    and no rule at all for the lot that comes next: A fences the shared side
    line, B is issued, B's drive runs up that line, and nothing goes back. That
    left 1-4 modules a seed across a drive — always the neighbour's, never the
    lot's own. Passing `clip=_clip_seg_paving` re-cuts them here, at the moment
    the drive is struck, exactly as a late garage re-cuts them.
    """
    for e in lines:
        if e[2] is None:
            continue
        cut = clip(e[0], e[1], box)
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


# ---------------------------------------------------------------------------
# cul-de-sac heads: frontage that is an ARC, and the WEDGE LOTS hung off it
# ---------------------------------------------------------------------------
# A TURNAROUND IS NOT A STREET, AND A LOT ON ONE IS NOT A RECTANGLE.
#
# `suburb_net._arc_cap_bulbs` puts the 17.64 m turnaround arc into the block
# boundary, which is what gives the head of a cul-de-sac any frontage at all.
# Walking that arc with the ordinary arclength rule then plats a RECTANGLE per
# station — tangent frontage, parallel side lines, inward normal at the
# station's own midpoint — and on a boundary that turns 330 degrees in 100 m
# that is wrong in every way at once:
#
#   * the side lines of two neighbours are parallel to two DIFFERENT normals,
#     33 degrees apart on a five-lot head, so consecutive lot rectangles cross
#     each other near the kerb and leave a widening wedge of nobody's land at
#     the back. Both defects are drawn: overlapping front fences, and a rear
#     fence ending in mid-air.
#   * the front lot line is a chord of the arc, so it CUTS THE PAVING — 2.8 m
#     of it on a five-lot head, more on a four — and `_clip_seg_disc` then has
#     to take the front fence out again, which is why bulbs came out fenced on
#     three sides.
#   * the deep half of the rectangle points wherever the station's normal
#     pointed, not away from the turnaround, so a lot at the throat plats its
#     back yard down the stem carriageway.
#
# A real plat does the obvious thing instead: the side lines are RADII of the
# turnaround, so they converge on its centre, are shared exactly between
# neighbours, and the lot is a pie wedge — narrow at the kerb (18-25 m of arc)
# and wider at the rear, which is also why the house fits even though the
# frontage looks short. Everything downstream follows from the frame:
#
#     n  = the radius at the lot's mid-angle, pointing AWAY from the bulb
#     u  = the tangent there, so the house yaw faces the turnaround centre
#     side lines   along the two bounding radii
#     front line   the arc itself, never a chord across it
#     drive        radial, from the house front to the PAVED radius
#
# The count is chosen, not stumbled into: the arclength walk would issue as
# many lots as the sampled width happened to allow, and the shipped preset
# samples 30-44 m — one and a half lots round a whole turnaround. So the head
# is platted as N EQUAL WEDGES with N picked from the frontage the lots
# actually get, which is measured at the house rather than at the kerb.


def _bulb_runs(poly, bulbs, tol=0.6, min_run=4):
    """Which boundary vertices are a turnaround kerb, as ``(spec, i0, i1)``.

    `suburb_net.blocks_from_faces` publishes one entry per bulb it spliced into
    this block (``blk["bulbs"]``), so the centre and the radius are known
    exactly and the run is just the vertices sitting on that circle. It is
    RE-FOUND here rather than carried as indices because two bulbs on one block
    splice one after the other, and the second splice renumbers the first.

    ``i1`` may be less than ``i0`` — the run wraps the ring's start, which is
    an artefact of where the face traversal happened to begin. `_rotate_ring`
    is what takes that case away.
    """
    n = len(poly)
    out = []
    for spec in (bulbs or ()):
        try:
            c = (float(spec["c"][0]), float(spec["c"][1]))
            r = float(spec["r"])
        except (TypeError, KeyError, IndexError, ValueError):
            continue
        on = [abs(_dist(q, c) - r) < tol for q in poly]
        if not any(on) or all(on):
            continue
        start = next((i for i in range(n) if on[i] and not on[(i - 1) % n]),
                     None)
        if start is None:
            continue
        run = [start]
        while on[(run[-1] + 1) % n] and len(run) < n:
            run.append((run[-1] + 1) % n)
        if len(run) < min_run:
            continue
        out.append((spec, run[0], run[-1]))
    return out


def _rotate_ring(poly, frontage, k):
    """Re-start the boundary at vertex *k*, frontage flags with it.

    The arclength walk is linear from 0 to the perimeter, so an arc that
    straddles the ring's start would be platted as two half-turnarounds with a
    seam down the middle. Where the ring starts carries no meaning — the face
    traversal picked it — so it is simply moved.
    """
    n = len(poly)
    k %= max(1, n)
    if k == 0:
        return poly, frontage
    out = list(poly[k:]) + list(poly[:k])
    if frontage is not None and len(frontage) == n:
        frontage = list(frontage[k:]) + list(frontage[:k])
    return out, frontage


def _arc_record(poly, cum, spec, i0, i1, discs, want_w, setback,
                min_kerb_m=11.0, n_lo=4, n_hi=7, min_sweep=1.2):
    """One turnaround as a platting job: the span, the sweep and HOW MANY lots.

    *want_w* is the frontage this block's district would have given a lot on a
    straight street, and it is compared against the arc AT THE HOUSE — radius
    plus setback — not at the kerb. That is the whole reason a cul-de-sac lot
    can be 20 m at the kerb and still take a 14 m house: the wedge is half as
    wide again by the time it reaches the front wall. Comparing at the kerb
    instead gives three lots round a head that comfortably holds five.

    ``r`` is the radius the lots are struck from, which is the boundary arc
    unless the caller's keep-out disc is bigger — a preset may ask for more
    front yard than the 3 m verge `_arc_cap_bulbs` uses, and lots must start
    outside whatever it asked for or every house on the head is refused.

    Returns ``None`` when this is not a turnaround worth platting radially: a
    sliver of arc left over at a block corner is better walked as ordinary
    frontage than cut into three wedges of nothing.
    """
    c = (float(spec["c"][0]), float(spec["c"][1]))
    r = float(spec["r"])
    for (dc, dr) in (discs or ()):
        if _dist(dc, c) < 1.0:
            r = max(r, dr)
    n = len(poly)
    ang = [math.atan2(q[1] - c[1], q[0] - c[0]) for q in poly]
    sweep, i = 0.0, i0
    while i != i1:
        j = (i + 1) % n
        d = ang[j] - ang[i]
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        sweep += d
        i = j
    if abs(sweep) < min_sweep:
        return None
    s0, s1 = cum[i0], cum[i1]
    if s1 - s0 < 20.0:
        return None
    # How many lots. Measured where the houses stand, then pulled back until
    # every lot still has a plattable frontage at the kerb. A FULL TURNAROUND
    # gets the count a real one has whatever the district asks for — 4 to 7
    # round the head, which is what the 96 ft bulb was dimensioned to serve.
    # An estate district would otherwise ask for two and a half 89 m frontages
    # and get three lots the size of the block, and that is not what a
    # large-lot cul-de-sac looks like: the wedge is how a big lot gets a
    # modest frontage, not a reason to plat fewer of them.
    lots = int(round((r + setback) * abs(sweep) / max(want_w, 1.0)))
    full = abs(sweep) >= 4.5
    lots = max(n_lo if full else 1, min(n_hi, lots))
    while lots > (n_lo if full else 1) and r * abs(sweep) / lots < min_kerb_m:
        lots -= 1
    return {"c": c, "r": r, "rp": float(spec.get("r_pave", r)),
            "s0": s0, "s1": s1, "a0": ang[i0], "sweep": sweep, "n": lots}


def _arc_at(arcs, s, eps=1e-6):
    """The turnaround whose frontage covers arclength *s*, or ``None``."""
    for a in arcs:
        if a["s0"] - eps <= s < a["s1"] - eps:
            return a
    return None


def _next_arc_s(arcs, s):
    """Arclength at which the next turnaround's frontage begins, or ``None``.

    What it is for: a lot issued just before a bulb must not be allowed to run
    its frontage into the arc, because the arc is already spoken for by the
    wedges. Stopping the straight lot short is the transition — no overlap, and
    the two platting rules meet at one station rather than fighting over a span.
    """
    nxt = None
    for a in arcs:
        if a["s0"] > s and (nxt is None or a["s0"] < nxt):
            nxt = a["s0"]
    return nxt


def _wedge_at(arc, s):
    """The wedge of *arc* that starts at arclength *s*.

    ``a_l``/``a_r`` are the LEFT and RIGHT bounding radii in the walk's own
    sense of left and right (-u and +u), whichever way round the arc the ring
    happens to run, so the lot corners come out in the order every other lot on
    the block uses.
    """
    span = arc["s1"] - arc["s0"]
    if span <= 1e-6:
        return None
    k = int((s - arc["s0"]) / span * arc["n"] + 1e-6)
    k = max(0, min(arc["n"] - 1, k))
    a_l = arc["a0"] + arc["sweep"] * (float(k) / arc["n"])
    a_r = arc["a0"] + arc["sweep"] * (float(k + 1) / arc["n"])
    return {"arc": arc, "k": k,
            "a_l": a_l, "a_r": a_r, "a_m": 0.5 * (a_l + a_r),
            "half_ang": abs(arc["sweep"]) / (2.0 * arc["n"]),
            "sgn": 1.0 if arc["sweep"] >= 0.0 else -1.0,
            "s_end": arc["s0"] + span * (float(k + 1) / arc["n"])}


def _ray(c, r, a):
    return (c[0] + r * math.cos(a), c[1] + r * math.sin(a))


def _lot_jobs(perim, arcs, rng, lw, d_lot, start, min_split_m=12.0, cap=4000):
    """Every lot this block will be offered, as ``(station, width, wedge)``.

    THE HEADS COME FIRST, and that ordering is the point of the function.
    Lots are issued in list order and each one is refused if it overlaps
    something already standing, so whatever is platted first wins the contested
    ground. Walking the ring in arclength order gives that ground to whichever
    lot the traversal happened to reach first — which, on a block with a
    cul-de-sac notched into it, is a stem lot: the stem walls run past the
    throat, a 30-44 m frontage there puts a big house within 30 m of the tip,
    and the first two wedges of the head were then refused for overlapping it.
    Measured on the scene `tools/fence_png.py` builds over seeds 3, 5 and 8, 58
    turnarounds: platted in ring order the heads took 107 houses between them,
    three of them none at all; platted first, 230 and none bald.

    A cul-de-sac head is the feature the eye goes to and the stem is ordinary
    street, so the head is platted first and the stem takes what is left. The
    walk then skips the arcs it already covered, and a straight lot that would
    run into one is cut short at its start — or dropped, if what is left of it
    is not a lot. Either way the two rules meet at one station.
    """
    jobs = []
    for a in arcs:
        span = a["s1"] - a["s0"]
        for k in range(a["n"]):
            s_k = a["s0"] + span * (float(k) / a["n"])
            w = _wedge_at(a, s_k)
            if w is not None:
                jobs.append((s_k, span / a["n"], w))
    s = start
    while s < perim and len(jobs) < cap:
        here = _arc_at(arcs, s)
        if here is not None:
            s = here["s1"]
            continue
        width = rng.uniform(*lw) * d_lot
        nxt = _next_arc_s(arcs, s)
        if nxt is not None and s + width > nxt:
            width = nxt - s
            if width < min_split_m:
                s = nxt
                continue
        jobs.append((s, width, None))
        s += width
    return jobs


def _probe_wedge(poly, c, r, a_l, a_r, lo, hi, step=1.0):
    """How deep this wedge may run before it leaves the block.

    Same contract as :func:`_probe_depth`, tested on the two bounding RADII and
    the mid-ray rather than on a rectangle's corners — with one difference that
    matters at the throat, where the arc meets the stem walls: it returns
    ``None`` when the lot cannot even reach *lo*. The rectangular probe is
    allowed to hand back its minimum regardless, because a shallow lot on a
    street is merely shallow; a wedge at the throat that is granted its minimum
    anyway has platted its back yard down the cul-de-sac's own carriageway.
    """
    a_m = 0.5 * (a_l + a_r)

    def ok(d):
        for a in (a_l, a_m, a_r):
            if not point_in_polygon(poly, _ray(c, r + d, a)):
                return False
        return True

    if not ok(lo):
        return None
    best, d = lo, lo
    while d < hi - 1e-9:
        d = min(d + step, hi)
        if ok(d):
            best = d
        else:
            break
    return best


def parcel_blocks(blocks, rng, cfg=None):
    """Lots, houses, drives and trees for every block.

    Returns ``[{"block": poly, "density": name, "houses": [...],
    "drives": [...], "trees": [...], "garages_rejected": n,
    "keepout_rejected": n, "size_rejected": n, "wedge_lots": n,
    "wedge_yield": n, "skew_yield": n}, ...]`` where each house is
    ``{"c": (x, y), "w": w, "d": d, "u": (ux, uy), "corners": [...]}``
    plus the lot it stands on: ``lot_corners`` (front_left, front_right,
    rear_right, rear_left), ``lot_depth``, ``fence_segs``
    (``[(p0, p1, "privacy"|"low"), ...]``, deduplicated across the block) and
    ``garage`` (``None`` or a box in the same shape as the house — RESERVED
    GROUND, not an asset request; see `garage_w_m`). With ``cfg["house_sizes"]``
    supplied, ``size_index`` says which measured entry the footprint came from
    and the caller is expected to place that asset.

    A block whose DISTRICT drew `ROW_CLASS` is not platted into lots at all:
    `row_housing.plan_cluster` sites one or two courts of attached row homes
    on it and the block reports ``clusters`` (the court, its bay schedule in
    `suburb_park.parking_info`'s shape, the single access drive, the footway to
    every door and the communal green) with an EMPTY ``drives`` list. Its house
    records are this same dict with ``garage: None``, no ``fence_segs``,
    ``has_pool: False`` and three extra keys — ``row``, ``cluster`` and
    ``palette``. Ordinary blocks carry ``clusters: ()``. See `row_share`.

    A lot on a cul-de-sac head is a WEDGE and not a rectangle — ``lot_corners``
    is then a trapezoid whose side lines are radii of the turnaround, and its
    front edge is the CHORD of an arc the fences follow rather than a straight
    lot line. ``wedge_lots`` counts them; ``wedge_yield`` counts the street lots
    refused for standing in one. See the block comment above
    :func:`_bulb_runs`, and `suburb_net.blocks_from_faces` for ``blk["bulbs"]``.

    ``skew_yield`` counts the street lots refused for standing in ANOTHER
    STREET LOT at a block corner more than ``junction_skew_clear_deg`` off
    square. It is the only one of these counts that is a deliberate thinning
    rather than a failure to fit something: `suburb_net` meets its streets at
    up to 38 degrees off the normal on purpose, and two rectangles hung off two
    frontages that meet at 52 degrees run through each other. Corners near
    square are left alone. See `junction_skew_clear_deg` in DEFAULTS.
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
    POOL_REAR_M = float(c.get("pool_rear_m", 20.0))
    # What the pool needs, as against what its lot asks for. See the two
    # entries in DEFAULTS: the ask buys the garden, the minimum is what
    # decides whether there is a pool at all.
    POOL_REAR_MIN_M = min(POOL_REAR_M, float(c.get("pool_rear_min_m", 9.5)))
    max_depth = max(min_depth, float(c["max_lot_depth_m"]))
    # HOW FAR OFF SQUARE A CORNER MAY BE, and how much daylight its two lots
    # must keep. Clamped at 90 because that is the whole range there is —
    # a corner cannot be more than 90 degrees off square — so 90 reads as "off",
    # and the test below short-circuits on it rather than measuring every pair
    # to conclude nothing. See both entries in DEFAULTS.
    skew_deg = min(90.0, max(0.0, float(c.get("junction_skew_clear_deg",
                                              30.0))))
    skew_pad = float(c.get("junction_skew_clear_m", 2.0))
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
            bulb_specs = blk.get("bulbs") or ()
        else:
            poly, faces_street = blk, None
            bulb_specs = ()
        # THE TURNAROUNDS ON THIS BLOCK, before anything is measured off the
        # ring. Where the ring starts is arbitrary, and an arc that straddles
        # that start would be walked as two pieces; re-starting the ring past
        # the last one costs nothing and takes the case away entirely. See
        # `_rotate_ring`.
        runs = _bulb_runs(poly, bulb_specs)
        if any(i1 < i0 for (_sp, i0, i1) in runs):
            _, _, i_end = max(runs, key=lambda t: t[2])
            poly, faces_street = _rotate_ring(poly, faces_street, i_end + 1)
            runs = _bulb_runs(poly, bulb_specs)
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
        dens, dens_extra = dens_at((bcx, bcy))
        # A ROW-HOME DISTRICT HAS NO DENSITY MULTIPLIERS, so it borrows
        # `normal`'s for the handful of things that still run on a cluster
        # block — the cul-de-sac wedge sizing above and the depth cap below.
        # Nothing on the cluster path reads them; they exist so a block that
        # tries to be a cluster and fails can plat as an ordinary one without a
        # second lookup.
        _dm = DENSITY.get(dens) or DENSITY["normal"]
        d_lot = float(_dm["lot"])
        d_set = float(_dm["setback"])
        d_dep = float(_dm.get("depth", 1.0))
        d_size = float(_dm.get("size", 1.0))
        # The depth CAP is a policy number, so it scales with the district too.
        # Leaving it at the tract-wide 40 m was silently un-doing the estate
        # multipliers: a 2.4x setback plus a 20 m house needs 50 m of lot
        # before any rear yard exists, so every estate lot hit the cap and got
        # a NEGATIVE back garden. That is why estates had 49% large houses and
        # not one pool. `_probe_depth` still bounds this by real geometry.
        blk_max_depth = max(min_depth, max_depth * d_dep)

        # ...and the turnarounds as platting jobs, which needs the district:
        # how many wedges a head takes is a question about how wide a lot this
        # district builds. See `_arc_record`.
        want_w = 0.5 * (lw[0] + lw[1]) * d_lot
        arcs = []
        for (bspec, i0, i1) in runs:
            rec = _arc_record(poly, cum, bspec, i0, i1, blk_discs, want_w,
                              0.5 * (sb[0] + sb[1]) * d_set)
            if rec is not None:
                arcs.append(rec)

        perim = polyline_length(ring)
        # WHERE THIS BOUNDARY TURNS A CORNER, once, before any lot is offered.
        # The skew rule needs it for every candidate pair and it is a property
        # of the ring, not of the lot. Skipped entirely when the rule is off.
        skew_corners = (_turn_corners(poly, cum, runs) if skew_deg < 90.0
                        else ())
        if perim < 40.0:
            # Too small for anything, a cluster included — so a row district
            # that lands here is recorded as a refusal and the block reports
            # the ordinary class, rather than showing up in the stats as a
            # row-home block with no row homes on it.
            _row_tiny = 1 if dens == ROW_CLASS else 0
            dens = "normal" if _row_tiny else dens
            out.append({"block": poly, "density": dens, "houses": [],
                        "drives": [], "trees": [], "clusters": (),
                        "garages_rejected": 0,
                        "keepout_rejected": 0, "size_rejected": 0,
                        "wedge_yield": 0, "skew_yield": 0, "wedge_lots": 0,
                        "row_rejected": _row_tiny})
            continue

        houses, drives, trees = [], [], []
        garages = []                 # reserved ground; nothing is built in it
        fenced = set()               # boundary identities already fenced
        fence_lines = []             # ...and the same as lines, for near-misses
        n_reject = 0
        n_keepout = 0                # lots lost to a turnaround
        n_nofit = 0                  # ...and to a frontage nothing measured fits
        n_wedge_yield = 0            # ...and to a cul-de-sac head's wedge lots
        n_skew_yield = 0             # ...and to a block corner well off square
        n_row_reject = 0             # ...and a row district that would not fit
        wedge_lots = []              # the wedge quads already platted here
        # Street lots already platted here, as `(station, inward normal, quad)`.
        # Kept beside `houses` rather than read back off it because the skew
        # test needs the boundary STATION, which the house record does not
        # carry, and because a wedge lot must not be in it — a wedge's normal is
        # a turnaround radius, not a frontage normal, so comparing the two would
        # report a corner where there is only a cul-de-sac.
        street_lots = []
        clusters = ()

        # -- A ROW-HOME BLOCK IS NOT PLATTED INTO LOTS AT ALL -----------------
        # The whole block goes to `row_housing`, which sites one or two courts
        # with their rows and hands back house records in this module's own
        # shape. A block is NOT half cluster and half street lots: a
        # development boundary in real platting runs along a street or a
        # recorded plat line, not down the middle of a block face, and mixing
        # the two would also mean the lot walk had to test every station
        # against a court it cannot see.
        #
        # WHEN IT WILL NOT FIT — too small a block, a catalogue with none of
        # the mix's styles in it, a turnaround eating the only frontage — the
        # block falls back to ORDINARY platting at `normal` density and says so
        # in `row_rejected`. `normal` rather than a fresh draw because the
        # district's zoning question was never asked: the seed said "row home",
        # not "tight" or "estate", and inventing an answer would also consume
        # an `rng` draw that the successful case does not.
        if dens == ROW_CLASS:
            rcfg = dict(c.get("row_housing") or {})
            rcfg.update(mix=(dens_extra or {}).get("mix"),
                        palette_set=(dens_extra or {}).get("palette_set"),
                        house_sizes=c.get("house_sizes"),
                        house_styles=c.get("house_styles"),
                        front_openings=openings_fn,
                        keepout_discs=blk_discs)
            plan = rh.plan_cluster(poly, faces_street, rng, rcfg)
            if plan is None:
                n_row_reject = 1
                dens = "normal"
            else:
                clusters = plan["clusters"]
                houses = list(plan["houses"])

        run_lo, run_hi = _rng_pair(c.get("archetype_run", [4, 9]), (4.0, 9.0))
        arch = _draw_archetype(rng, dens)
        run_left = int(rng.uniform(run_lo, run_hi + 0.999))
        # EVERY LOT THIS BLOCK WILL BE OFFERED, IN THE ORDER IT IS OFFERED —
        # turnaround heads before the streets that lead to them. See
        # :func:`_lot_jobs` for why the order is not simply arclength. A block
        # that took a cluster is offered NOTHING: its ground is spoken for.
        for (st, width, wedge) in (() if clusters else
                                   _lot_jobs(perim, arcs, rng, lw, d_lot,
                                             rng.uniform(0.0, lw[0]))):
            arc = wedge["arc"] if wedge is not None else None
            s = st + width
            mid = st + width / 2.0
            if not is_frontage(mid):
                continue
            if wedge is not None:
                # The wedge's own frame, struck from the turnaround centre
                # rather than read off the boundary: `n` is the radius at the
                # lot's mid-angle pointing away from the bulb, `u` the tangent
                # there — so the house yaw faces the turnaround, which is the
                # one thing every house round a cul-de-sac has in common.
                a_c, a_r0 = arc["c"], arc["r"]
                p = _ray(a_c, a_r0, wedge["a_m"])
                n = (math.cos(wedge["a_m"]), math.sin(wedge["a_m"]))
                u = _mul(_perp(n), wedge["sgn"])
                t = u
            else:
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
            # HOW MUCH FRONTAGE THE HOUSE ACTUALLY GETS. On a street that is
            # the platted width; on a wedge the side lines diverge, so the
            # building line is wider than the kerb line by the ratio of their
            # radii — a 20 m frontage on a 17.6 m turnaround gives 29 m at a
            # 8 m setback, which is the difference between a cul-de-sac that
            # holds five houses and one that holds three. Measured as the CHORD
            # between the two side lines at the setback radius, so it is a real
            # distance across the lot and not an arclength that overstates it.
            if wedge is not None:
                build_w = 2.0 * (arc["r"] + setback) * math.sin(
                    min(wedge["half_ang"], math.pi / 2.0))
            else:
                build_w = width
            if sizes:
                # The catalogue is the authority on how big a house is. See
                # `house_sizes` in DEFAULTS and :func:`_pick_size`.
                size_i = _pick_size(sizes, size_order, rng,
                                    build_w - gap * 0.5,
                                    float(spec["scale"]) * d_size)
                if size_i is None:
                    n_nofit += 1
                    continue
                h_w, h_d = sizes[size_i]
            else:
                size_i = None
                h_w = min(rng.uniform(*hw) * spec["scale"], build_w - gap)
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
            # ...AND THE CAP HAS TO MOVE WITH THE ASK. `blk_max_depth` is the
            # tract cap scaled by the district's own `depth` multiplier, which
            # on a tight block is BELOW the 20 m rear a pool now asks for — so
            # the request was granted by `max(...)` and clipped straight back
            # off by `min(...)`, and the lot came back too shallow for the
            # thing it was platted for. A pool lot is allowed the tract-wide
            # `max_lot_depth_m` whatever district it sits in: it is the 6%
            # package, and being the deep lot on the street is what it is for.
            # `_probe_depth` still bounds this by the real block geometry, so
            # nothing here can push a lot out over the far kerb.
            depth_cap = max(blk_max_depth, max_depth) if want_pool \
                else blk_max_depth
            want_depth = min(depth_cap,
                             max(min_depth, setback + h_d + rear_need,
                                 rng.uniform(*ld) * d_dep))
            if wedge is not None:
                # THE WEDGE. Side lines are the two bounding RADII, so the pair
                # a neighbour shares is the same line struck from the same
                # centre — no crossing near the kerb, no orphan strip at the
                # back, and no arithmetic keeping two independent lots in step.
                # `_probe_wedge` refuses outright rather than granting a
                # minimum: at the throat, where the arc meets the stem walls,
                # the minimum runs down the cul-de-sac's own carriageway.
                lot_depth = _probe_wedge(poly, a_c, a_r0,
                                         wedge["a_l"], wedge["a_r"],
                                         min_depth, want_depth)
                if lot_depth is None:
                    continue
                fl = _ray(a_c, a_r0, wedge["a_l"])
                fr = _ray(a_c, a_r0, wedge["a_r"])
                rl = _ray(a_c, a_r0 + lot_depth, wedge["a_l"])
                rr = _ray(a_c, a_r0 + lot_depth, wedge["a_r"])
                # The front lot line is the ARC between `fl` and `fr`, and the
                # quad records its chord — which dips inside the kerb circle by
                # r(1 - cos(half angle)), up to 4 m on a four-lot head. Nothing
                # is built off that chord: the fences below follow the arc, and
                # every pass that plats inside `lot_corners` is handed the same
                # keep-out disc this one is.
                half = (a_r0 + float(c["fence_front_inset_m"])) \
                    * wedge["half_ang"]
            else:
                lot_depth = _probe_depth(poly, p, n, u, width,
                                         min_depth, want_depth)
                # Half-width is capped by the CHORD back to the two stations
                # this lot actually runs between. On a straight face that is
                # exactly `width / 2`; on a curving one the chord is shorter
                # than the arc, and taking the arc would push the corner past
                # the neighbouring lot's — a 34 m estate frontage on a curve
                # overshoots by ~4 m, which is four metres of two lots' front
                # fences in the same place. Surveyors plat curved frontage as
                # chord bearings for this reason.
                half = min(width / 2.0,
                           _dist(p, point_at(ring, st)),
                           _dist(p, point_at(ring, s)))
                fl = _add(p, _mul(u, -half))
                fr = _add(p, _mul(u, half))
                rl = _add(fl, _mul(n, lot_depth))
                rr = _add(fr, _mul(n, lot_depth))
            lot_corners = [fl, fr, rr, rl]

            # THE STREET GIVES WAY TO THE HEAD. The throat is a block corner
            # like any other, and lots hung off the two faces meeting at a
            # corner overlap there — see :func:`_clip_seg`, which exists to
            # cope with exactly that. It is worse at a turnaround: the wedge's
            # end side line is a radius pointing back down the stem, so the
            # ground it claims is the ground the last stem lot wants, and the
            # two lots' side fences then run through each other for 20 m.
            # Since the head is platted first anyway (`_lot_jobs`), the rule is
            # simply that a lot may clip a wedge's corner but not stand in its
            # garden. Between two street lots the same rule is applied ONLY at
            # a corner well off square — see the skew test below; at a corner
            # near square it is the pre-existing corner behaviour this module
            # documents and does not have the packing pass to fix.
            #
            # WEDGES ARE TESTED TOO, and they have to be — `cul_de_sac_gap_
            # factor` lets two stubs end 48 m apart, which is closer than two
            # 40 m-deep heads can both be platted, so the far side of one head
            # lands in the far side of the other. Two wedges of the SAME head
            # share their side line to the bit, so the tolerance passes them
            # without a special case.
            if wedge_lots and any(_convex_overlap(lot_corners, q, pad=-2.0)
                                  for q in wedge_lots):
                n_wedge_yield += 1
                continue

            # --- A SKEWED CORNER IS NOT PLATTED THROUGH -----------------------
            # The one place two STREET lots are held apart. See
            # `junction_skew_clear_deg` in DEFAULTS for the measurement and for
            # why the gate is departure from square rather than the angle
            # itself. Three conditions, and all three are needed:
            #
            #   the two frontages must actually MEET AT A CORNER — tested on
            #     the boundary walk, because two lots on ONE face that curves
            #     have their normals up to 84 degrees apart and are
            #     indistinguishable from a corner pair by angle. The
            #     `_SKEW_THETA_MIN/MAX` band rides along with this test as a
            #     second guard: see `_turn_corners`' block comment;
            #   that corner must be more than `junction_skew_clear_deg` off
            #     square, in either direction;
            #   and the two lot quads must be within `junction_skew_clear_m` of
            #     each other. Nothing is refused for merely being NEAR a skewed
            #     corner: a lot whose rectangle clears its neighbour's is a lot
            #     whose fences clear them too, however sharp the junction it
            #     happens to sit at, and refusing it would delete houses to fix
            #     a defect that is not there.
            #
            # Whichever lot reached the corner first keeps it, which is the same
            # rule the wedge test above uses and is what a real plat does: the
            # corner belongs to the lot that FRONTS one street, and the other
            # street's run stops clear of its side yard.
            #
            # TRIED AND REJECTED: a plain keep-out — refuse any lot whose
            # frontage is within R metres of a skewed corner. It does not
            # target the offending lot, because the lot that ends up inside its
            # neighbour is often not the one nearest the corner: lots here are
            # 30-44 m wide and on a wedge-shaped block the pair can be 100 m
            # of boundary apart. Measured over seeds 3/5/8, against the 130
            # overlapping pairs / 26.3k m2 at corners more than 30 degrees off
            # square:
            #
            #   R      houses lost     of that overlap it would remove
            #   10 m    19  (1.0%)      15 pairs,  3.5k m2  (13%)
            #   20 m    77  (4.2%)      28 pairs,  7.4k m2  (28%)
            #   30 m   147  (7.9%)      44 pairs, 10.1k m2  (38%)
            #   40 m   216 (11.7%)      50 pairs, 10.3k m2  (39%)
            #
            # It plateaus at 39% however wide it gets. The quad test below
            # costs 5.2% of the houses and removes 99.2%.
            if wedge is None and skew_deg < 90.0 and street_lots:
                hit = False
                for (s_o, n_o, q_o) in street_lots:
                    if not _across_corner(mid, s_o, perim, skew_corners):
                        continue
                    theta = _corner_deg(n, n_o)
                    if not (_SKEW_THETA_MIN <= theta <= _SKEW_THETA_MAX):
                        continue
                    if abs(theta - 90.0) <= skew_deg:
                        continue
                    if _convex_overlap(lot_corners, q_o, pad=skew_pad):
                        hit = True
                        break
                if hit:
                    n_skew_yield += 1
                    continue

            if wedge is not None:
                wedge_lots.append(lot_corners)
            else:
                street_lots.append((mid, n, lot_corners))

            # This house may stand on a line an earlier lot already fenced —
            # see :func:`_clip_seg` on why lots overlap at all.
            _clip_standing(fence_lines, corners)

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
            # THE BUG THIS REPLACES. There were two drives and they disagreed.
            # `art_gaps` is the KIT's answer — where this style's front door and
            # garage door cross the front line — and it is what `front_gaps`
            # took. The drive that actually gets PAVED is `plan_lot`'s when the
            # kit stamped a garage and this pass's `drives[]` entry otherwise,
            # and that entry was struck at `g_off` or at `side * h_w * 0.30`
            # from a coin flip. So:
            #
            #   art has a door but NO garage  ->  `art_gaps` is non-empty (the
            #       door), the `if not front_gaps` fallback never fires, and the
            #       PLAT's drive gets no opening at all. That is every
            #       garage-less kit style — cottage, two_storey, wide_house,
            #       terrace — and on seed 3 it was 19 front-fence modules
            #       standing across their own driveway.
            #   art HAS a garage  ->  the drive is paved at the kit's `garage_x`
            #       while this pass recorded one at `side * h_w * 0.30`, so
            #       every consumer of `drives[]` aimed at the wrong strip.
            #
            # Hence `drive_off` / `drive_half`: the kit's garage opening when
            # the kit has one, this pass's own otherwise. `drives[]` below is
            # struck from it, so the plat and the kit now name the same ribbon,
            # and the fence gives way to that one ribbon everywhere.
            art_drive = next((g for g in (art_gaps or ())
                              if len(g) > 2 and g[2] == "drive"), None)
            if art_drive is not None:
                drive_off, drive_half = float(art_drive[0]), float(art_drive[1])
            else:
                drive_off = g_off if garage is not None else side * h_w * 0.30
                drive_half = (g_w if garage is not None else dw) / 2.0 + 0.6
            # The front opening is the union of the kit's crossings and the
            # plat's drive — the second only when the kit did not supply one,
            # or the same drive is cut out twice.
            front_gaps = [(g[0], g[1]) for g in (art_gaps or ())]
            if art_drive is None:
                front_gaps.append((drive_off, drive_half))

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
            # NO FENCE ON A CUL-DE-SAC LOT. A wedge lot is a pie slice off the
            # turnaround arc: its side lines converge toward the bulb centre,
            # so a perimeter fence reads as a wedge of panels narrowing to a
            # point at the kerb. It is the one place the fence does not look
            # like a fence, so these lots get none.
            wants_fence = spec["fence"] > 0.0 and wedge is None
            if wants_fence:
                f_in = float(c["fence_front_inset_m"])
                if wedge is not None:
                    # SAME PERIMETER, STRUCK RADIALLY. The two side runs start
                    # `f_in` out along their OWN radius, not along the lot's
                    # mid-normal — offsetting both corners by one normal turns
                    # the shared side line into two lines a metre apart, which
                    # is the doubled-fence defect the plan colours cyan.
                    ffl = _ray(a_c, a_r0 + f_in, wedge["a_l"])
                    ffr = _ray(a_c, a_r0 + f_in, wedge["a_r"])
                    fence_rl, fence_rr = rl, rr
                else:
                    # SIDE FENCES MEET THE NEIGHBOUR'S. `half` above is
                    # chord-clamped so the HOUSE lot never overruns its
                    # neighbour; on a curving face that leaves the lot corner
                    # short of the true boundary, and two neighbours' fences
                    # stop short of each other with a strip of nobody's land
                    # between them. The BOUNDARY is the station the two lots
                    # SHARE, so anchoring the fence there makes them meet
                    # exactly instead of nearly. The house keeps the clamped
                    # corners — only the fence is widened.
                    _b0, _b1 = point_at(ring, st), point_at(ring, s)
                    # Which station is on the -u side; do not assume ordering.
                    if _dot(_sub(_b0, p), u) > _dot(_sub(_b1, p), u):
                        _b0, _b1 = _b1, _b0
                    ffl = _add(_b0, _mul(n, f_in))
                    ffr = _add(_b1, _mul(n, f_in))
                    fence_rl = _add(_b0, _mul(n, lot_depth))
                    fence_rr = _add(_b1, _mul(n, lot_depth))
                pf = _add(p, _mul(n, f_in))
                cand = [(ffl, fence_rl, "privacy"), (ffr, fence_rr, "privacy"),
                        (fence_rl, fence_rr, "privacy")]
                blockers = [corners] + [h["corners"] for h in houses] \
                    + [g["corners"] for g in garages]
                if arch in ("full", "large"):
                    # Front run, broken wherever something crosses it. A fence
                    # across your own driveway is worse than no fence.
                    if wedge is not None:
                        # AND ON A WEDGE IT FOLLOWS THE ARC. The chord between
                        # the two front corners cuts the turnaround — 2.8 m of
                        # it on a five-lot head — so a straight front fence
                        # there is a fence in the road, and `_clip_seg_disc`
                        # would delete most of it rather than bend it. Each
                        # surviving run is emitted as a few chords along the
                        # kerb radius instead, which is what a fence built to
                        # follow a curved lot line looks like anyway.
                        rf = a_r0 + f_in
                        # The gaps are offsets in the HOUSE's frame, measured
                        # across the building line; the fence is an arc two
                        # radii further in, where the same drive crosses a
                        # shorter span. Scaling by the radius ratio is what
                        # keeps the opening over the drive instead of a metre
                        # and a half round the curve from it.
                        k_r = rf / max(a_r0 + setback, 1e-6)
                        arc_gaps = [(o * k_r, hwd * k_r)
                                    for (o, hwd) in front_gaps]
                        for x0, x1 in _front_runs(-half, half, arc_gaps):
                            k = max(1, int(abs(x1 - x0) / rf / 0.32) + 1)
                            pts = [_ray(a_c, rf,
                                        wedge["a_m"] + wedge["sgn"]
                                        * (x0 + (x1 - x0) * i / k) / rf)
                                   for i in range(k + 1)]
                            for i in range(k):
                                cand.append((pts[i], pts[i + 1], "low"))
                    else:
                        for x0, x1 in _front_runs(-half, half, front_gaps):
                            cand.append((_add(pf, _mul(u, x0)),
                                         _add(pf, _mul(u, x1)), "low"))

                # EVERY DRIVE THAT CROSSES THIS LOT, as an oriented apron: this
                # lot's own, and those of the lots already issued on this block,
                # because two lots overlap in a block corner and the neighbour's
                # drive is as real a piece of asphalt as your own.
                _d0 = _add(p, _mul(u, drive_off))
                aprons = [_drive_box(d) for d in drives[-8:]]
                aprons.append(_drive_box({"a": _d0,
                                          "b": _add(_d0, _mul(n, setback + 0.6)),
                                          "w": 2.0 * drive_half}))

                def _cut_run(a, b):
                    """Trim a boundary out of the pavement, the walls, the
                    DRIVEWAY and any fence already standing across it — in that
                    order.

                    Pavement first, because it is the cut that can take the
                    whole run: a lot beside a bulb has its two side lines
                    running out of the turnaround, and there is no point asking
                    which wall a fence stops at when the front half of it is on
                    the road. The drive comes after the walls, because a run
                    already stopped at a garage wall may no longer reach the
                    apron in front of it. Fences last, because a run already
                    shortened by a wall may no longer cross anything.
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
                    for box in aprons:
                        cut = _clip_seg_paving(cut[0], cut[1], box)
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
                    if cut is not None:
                        cut = _extend_to_meet(cut[0], cut[1], fence_lines,
                                              blockers)
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
                           "has_fence": wants_fence,
                           # Cul-de-sac lot. Recorded because the fence
                           # rule keys off it and nothing downstream
                           # could otherwise tell a wedge lot apart.
                           "wedge_lot": wedge is not None,
                           # THE KERB, WHEN IT IS AN ARC. `frontage` is the LOT
                           # LINE and `u` the TANGENT there, so a kerb end slid
                           # along `u` sits on the tangent at the lot-line
                           # radius, outside the paving by the verge
                           # `_arc_cap_bulbs` left plus the tangent bulge.
                           # `modular_house.plan_lot` cannot recover the
                           # turnaround from the record, so it is published:
                           # (centre, PAVED radius), None on a straight lot.
                           "kerb_arc": ((a_c, float(arc["rp"]))
                                        if wedge is not None else None),
                           # ...and only if the BLOCK actually granted enough
                           # for one. `_probe_depth` can come back short on a
                           # shallow or awkward block, and a pool half in the
                           # next street is worse than no pool. Tested against
                           # the MINIMUM, not against `pool_rear_m`: the ask is
                           # what buys the garden, and holding the admission
                           # test to it would refuse most of the package the
                           # moment the ask went up.
                           "has_pool": (spec.get("pool", 0.0) > 0.0
                                        and lot_depth - setback - h_d
                                        >= POOL_REAR_MIN_M),
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
            # ...at `drive_off` — the ONE offset the front-fence opening and the
            # fence cuts above were struck from, so the paving and the gap in
            # the fence are the same ribbon by construction rather than by two
            # guesses happening to agree.
            a0 = _add(p, _mul(u, drive_off))
            a1 = _add(a0, _mul(n, setback + (0.2 if garage is not None else 0.5)))
            if wedge is not None:
                # AIM IT AT THE TURNAROUND. Offsetting along the tangent puts
                # the apron on the tangent LINE, which leaves the kerb behind
                # as the arc curves away — and the kerb here is not the lot
                # line either: `_arc_cap_bulbs` leaves a verge, so the asphalt
                # is `r_pave` and the lot starts three metres further out. So
                # the run is struck from the house end back to the paved
                # radius, and every drive on the head points at its centre.
                a0 = _ray(a_c, arc["rp"], math.atan2(a1[1] - a_c[1],
                                                     a1[0] - a_c[0]))
            # ...and the fences the earlier lots already stood on this line
            # get out of its way, the same way they do for a late garage.
            _clip_standing(fence_lines, _drive_box({"a": a0, "b": a1,
                                                    "w": 2.0 * drive_half}),
                           clip=_clip_seg_paving)
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
        #
        # ON A CLUSTER BLOCK THE KERB IS STILL A KERB and still gets its verge
        # trees — the one thing a row-home development shares with the street
        # it fronts. What it must not get is a tree in the access drive: that
        # drive crosses the verge exactly where these are planted, and it is
        # the only route in, so a tree standing in it is not a cosmetic defect.
        # Same test the house loop uses, against the cluster's paving axes.
        row_paving = [(a, b, hw) for cl in clusters for (a, b, hw)
                      in cl["paving"]]

        def _on_cluster_paving(q, r):
            for (a, b, hw) in row_paving:
                if seg_seg_dist(a, b, q, q) < hw + r:
                    return True
            return False

        u_s = rng.uniform(0.0, sts[1])
        while u_s < perim:
            if not is_frontage(u_s):
                u_s += rng.uniform(*sts)
                continue
            p = point_at(ring, u_s)
            t = tangent_at(ring, u_s)
            n = _inward(poly, p, t)
            r = rng.uniform(*tr)
            # A verge tree goes 2.6 m in from the block boundary. AROUND A
            # TURNAROUND THAT IS NOT ENOUGH: the keep-out disc is the paving
            # plus a front-yard margin and the boundary arc sits on it, so a
            # tree 2.6 m in was inside the disc by its own crown radius and the
            # `on_bulb` test below threw every one of them away — a bald ring
            # of houses with no planting at all in front of them. Stepping out
            # by the crown as well puts the trunk where a real cul-de-sac's
            # verge trees stand, just inside the front lot line.
            step = 2.6 + (r if _arc_at(arcs, u_s) is not None else 0.0)
            q = _add(p, _mul(n, step))
            if point_in_polygon(poly, q):
                on_bulb = any(_dist(q, dc) < dr + r for (dc, dr) in blk_discs)
                if (not on_bulb and not _on_cluster_paving(q, r)
                        and not any(
                            _dist(q, h["c"]) < max(h["w"], h["d"]) / 2.0 + r
                            for h in houses)):
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

        # EVERY LOT ON THIS BLOCK IS NOW ISSUED, which is the only point at
        # which "what meets this run" is a fact rather than a guess about a
        # neighbour not platted yet. See `_trim_stub_tips`.
        _trim_stub_tips(fence_lines)

        out.append({"block": poly, "density": dens, "houses": houses,
                    "drives": drives, "trees": trees,
                    # THE SHARED GEOMETRY A ROW-HOME BLOCK HAS AND A PLATTED
                    # ONE DOES NOT: the court (with its bay schedule in
                    # `suburb_park.parking_info`'s schema), the single access
                    # drive, the footway to every door and the communal green.
                    # Empty tuple on every ordinary block, so a consumer can
                    # test it without a default. `drives` stays EMPTY on a
                    # cluster block, which is what keeps the driveway passes —
                    # `apply_ground`, `paving_keepout`, `build_cars` — off it:
                    # each walks `p["drives"]` and indexes `p["houses"]` by the
                    # same ordinal, so a partial list would mis-pair them.
                    "clusters": clusters,
                    "garages_rejected": n_reject,
                    # Reported rather than swallowed: a thinned-out street is
                    # its own defect, and these are the numbers that say
                    # whether a keep-out, a catalogue with nothing narrow
                    # enough in it, or a cul-de-sac head that was platted
                    # first caused one.
                    "keepout_rejected": n_keepout,
                    "size_rejected": n_nofit,
                    "wedge_yield": n_wedge_yield,
                    # ...and to a block corner more than
                    # `junction_skew_clear_deg` off square, where two lots
                    # hung off the two frontages would have stood in each
                    # other. This is the ONE count that is a deliberate
                    # thinning rather than a failure — see the DEFAULTS entry —
                    # so a run whose houses/block has dropped can be read
                    # against it instead of being blamed on the catalogue.
                    "skew_yield": n_skew_yield,
                    # How many lots on this block are cul-de-sac wedges. The
                    # one number that says whether the turnarounds got platted
                    # at all, which no other count distinguishes.
                    "wedge_lots": len(wedge_lots),
                    # 1 when this block drew a row-home district and could not
                    # fit a court in it, so fell back to ordinary platting.
                    # Reported rather than swallowed for the same reason the
                    # counts above are: a row_share that produces no clusters
                    # is either a share set too low or a block set too small,
                    # and only this number tells them apart.
                    "row_rejected": n_row_reject})
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
    # ROW HOMES, counted separately from everything above. A cluster unit IS a
    # house and is in `n_h`, but the numbers that say whether the feature ran —
    # how many developments, how many courts, how many bays, which mixes and
    # which styles — are recoverable from nowhere else.
    cls = [cl for p in parcels for cl in (p.get("clusters") or ())]
    row = rh.stats(cls)
    return {"houses": n_h, "trees": n_t,
            "row_blocks": sum(1 for p in parcels
                              if p.get("clusters")),
            "row_courts": row["courts"], "row_units": row["units"],
            "row_bays": row["bays"], "row_mixes": row["mixes"],
            "row_styles": row["styles"], "row_palettes": row["palettes"],
            "row_rejected": sum(int(p.get("row_rejected", 0))
                                for p in parcels),
            "blocks_built": len(per), "blocks": len(parcels),
            "houses_per_built_block": (sum(per) / len(per)) if per else 0.0,
            "garages": sum(1 for h in hs if h.get("garage")),
            "garages_rejected": sum(int(p.get("garages_rejected", 0))
                                    for p in parcels),
            "keepout_rejected": sum(int(p.get("keepout_rejected", 0))
                                    for p in parcels),
            "size_rejected": sum(int(p.get("size_rejected", 0))
                                 for p in parcels),
            "wedge_yield": sum(int(p.get("wedge_yield", 0))
                               for p in parcels),
            # Lots refused for standing in a neighbour's at a block corner more
            # than `junction_skew_clear_deg` off square. The only one of these
            # counts that is a deliberate thinning rather than a failure.
            "skew_yield": sum(int(p.get("skew_yield", 0)) for p in parcels),
            "wedge_lots": sum(int(p.get("wedge_lots", 0)) for p in parcels),
            "fenced_houses": sum(1 for h in hs if h.get("fence_segs")),
            "fence_segs": sum(len(h.get("fence_segs") or ()) for h in hs),
            "density_blocks": dens, "archetypes": arch,
            "mean_lot_depth": (sum(depths) / len(depths)) if depths else 0.0}
