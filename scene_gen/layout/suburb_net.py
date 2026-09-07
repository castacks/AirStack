"""
suburb_net.py — a suburban street network built as a GRAPH, from scratch.

WHY THIS EXISTS, AND WHAT IT REPLACES
-------------------------------------
The previous suburb generators were subdivision-based: a region was cut into
axis-aligned rectangles and every "road" was a rect. A later pass tried to make
those roads *look* curved by widening the rect into an envelope and painting
grass over the parts the carriageway did not use. That approach cannot produce a
natural junction, and the reason is structural rather than a tuning problem:

    the corridor never knew which way the road was pointing.

A junction was therefore a rectangular patch of asphalt held open between two
pieces of geometry that had no relationship to each other — a straight side
street ending at an envelope edge, and a curved carriageway wandering somewhere
inside that envelope. The patch had to span the gap between them, and since the
envelope is ~41 m across while the road inside it is ~10.6 m, the result was a
rectangular tongue of asphalt of essentially random length sticking out at every
crossing. That is the "intersections are weird" symptom, and no amount of
padding fixes it, because the two roads never agreed on where they met.

So this module inverts the construction. THE JUNCTION IS THE PRIMITIVE:

    A street is BORN on a host street's centreline, at a known arclength, with
    its start tangent set to the host's NORMAL at that point.

and the street body is a cubic Hermite curve that interpolates *specified
tangents at both ends*. A loop leaves its host perpendicular and arrives back
perpendicular because that is what it was solved for — not because a clearance
test happened to allow it. Merging is correct before anything is drawn.

REPRESENTATION
--------------
A planar graph with real geometry, which is how OSM stores a street network and
how CityEngine (Parish & Müller 2001) and Vanegas (2012) generate one:

    Node   a junction or a dead end:  (x, y), incident half-edges
    Edge   a street segment:          polyline centreline, class, width, type
    Face   a block:                   polygon, recovered by planar traversal

Blocks are POLYGONS recovered as the faces of the graph, not leaves of a
recursion. That is what lets the street network be shaped freely: whatever the
streets do, the faces between them are still an exact tiling of the land.

MORPHOLOGY: "LOOPS AND LOLLIPOPS"
---------------------------------
Southworth & Ben-Joseph's typology of American residential fabric runs
gridiron -> fragmented parallel -> warped parallel -> loops and lollipops ->
lollipops on a stick, roughly by decade. Post-war suburbia is the fourth.

THE PIPELINE, IN THE ORDER IT RUNS
----------------------------------
    0  crop boundary      NOT a road: four zero-width edges that close the
                          planar graph so faces exist. The region is a crop of a
                          larger suburb, so streets run off the edge
    0b park reserve       the rectangle the park will occupy, plus a padding
                          band, plus the frame street around it. RESERVED
                          BEFORE ANY STREET IS LAID — see below
    1  collectors         curvilinear routes crossing the interior, as a
                          harmonic offset from a straight baseline so curvature
                          is bounded and both ends leave the crop square
    1b park approaches    short streets from the frame outward to the first
                          street they meet, one per side: the park's entrances,
                          and the frame's connection to the rest of the graph
    2  loops              a street leaving a host and returning to it
    3  face subdivision   split the largest remaining face until every block is
                          near target size, aiming each cut ACROSS the parcel
                          and leaving a share of parcels UNDEVELOPED — THE
                          LAYER THAT FILLS THE LAND
    4  lollipops          cul-de-sacs, grown until the dead-end share is met.
                          NOT into undeveloped land: a dead end exists to serve
                          lots and there are none there
    5  merge open land    the frame street comes DOWN where the park abuts an
                          undeveloped parcel, so the two read as one open space
                          — which is what makes the park a polygon

(Loops really do run before subdivision, for the reason given at that call
site: a loop needs 200-520 m of continuous frontage and subdivision does not,
so running subdivision first consumed every long frontage and left zero loops.)

BLOCK SHAPE: CUT ACROSS THE PARCEL, NOT ACROSS A CORNER
-------------------------------------------------------
A cut from one side of a four-sided face to the OPPOSITE side leaves two
quadrilaterals. A cut to an ADJACENT side slices the corner off and leaves a
triangle. Nothing used to prefer the first: the far end was sited at 34-66% of
the way round the PERIMETER, which is only a proxy for "the opposite side" and
a poor one when the sides are unequal, and cut length plus area balance plus
grain say nothing about shape at all — a corner slice is often the shortest and
best-balanced candidate on offer. Measured over twelve seeds, 47.5% of blocks
came out with `rectangularity` under 0.62, i.e. nearly half the fabric was
triangular. Residential plats on OSM are not.

Three changes, all inside :func:`_best_cut`, and they need each other:

  * A SIDE IS GEOMETRIC, NOT AN EDGE (:func:`_face_corners`). `hes`/`parts`
    give one entry per graph edge, which is neither necessary nor sufficient:
    a curving street contributes dozens of vertices and, once junctions have
    split it, several edges, all of them one side; one edge bending round a
    corner is two. A corner is where the turning is CONCENTRATED — a boundary
    vertex whose exterior angle clears `face_corner_deg` — and the sides are
    the arcs between corners. The threshold separates the two cases by
    construction: a 30 m radius sampled every 6 m turns 11.5 degrees per
    vertex however far it bends in total, a street meeting a host turns 52-128
    degrees at one vertex.
  * CANDIDATES ARE AIMED. Both ends are drawn on non-adjacent sides, at
    complementary fractions along them, so the cut runs parallel to the two
    sides it does not touch.
  * THE SCORE CARRIES A SHAPE TERM AND A FLOOR. `block_shape_weight`
    penalises the worse half's `rectangularity` — 1.0 for a rectangle at any
    angle or aspect, exactly 0.5 for any triangle — and `block_side_penalty`
    charges for each missing corner of separation between the ends. Candidates
    that clear `block_shape_floor` AND run genuinely across are kept in a
    separate bucket and win outright, so a corner slice is only made when
    nothing across the parcel is legal at all.

Result over the same twelve seeds when that work landed: 23.3% under 0.62, mean
rectangularity 0.632 -> 0.697, and 23.0% on the current fabric — the shape term
is what holds it, and the passes added since have not moved it.
Blocks are not all rectangles, and should not be — the residue is
mostly parcels with a cul-de-sac stub notched into them and the genuinely
three-sided faces left over where three streets meet.

SPARSENESS: NOT EVERY FACE IS DEVELOPED
---------------------------------------
Real suburbs have undeveloped parcels — woodland, a drainage reserve, a phase
that was never built, land not taken up yet — and a generator that tiles the
region wall to wall reads as a masterplan rather than a place. `undeveloped_share`
of block-sized faces are left UNSUBDIVIDED, biased toward the crop edge and away
from the collectors, which is where such land actually survives on an aerial
(see :func:`_undeveloped_weight`). They are still emitted as blocks, so the
ground is drawn, but flagged `undeveloped` for the passes that build houses.

TWO THINGS FOLLOW FROM "NOBODY BUILT ON IT", and both are about roads:

  * NO CUL-DE-SAC GOES INTO IT (:func:`_dead_ends_in_open_land`). A dead end
    exists to give lots frontage and to turn a fire appliance round at the end
    of it; land with no lots on it needs neither, so a stub poking into one is
    a road that serves nothing. A road CROSSING that land — to reach the fabric
    beyond it, or to run off the crop edge — is a different thing entirely and
    stays allowed, because that road is doing the one job a road does. The test
    is on the stem and the bulb, not on the street.
  * WHERE IT TOUCHES THE PARK, THE FRAME BETWEEN THEM COMES DOWN
    (:func:`_merge_open_land`). See below.

THE PARK IS RESERVED, NOT FOUND
-------------------------------
A neighbourhood park is 420 x 300 m — 12.6 ha — against a median block of about
2.8 ha. There is no block big enough to drop one into, and there never will be,
because the subdivision pass exists precisely to stop faces that size existing.
So the rectangle is chosen first and the network is generated around it, which
is the same move the park module makes internally when it routes its paths
around its courts rather than fitting courts into the gaps the paths left.

Every phase asks ONE predicate, :func:`reserve_blocks` — collectors, both face
cutters, loops, cul-de-sacs. Four separate patches would have been four chances
to get it wrong; one predicate means a phase that forgets to ask is a visibly
missing line. :class:`Reserve` holds the two rectangles that matter: the park,
and the park grown by the padding band that no centreline may enter.

A frame street rings the reserve, and it is not decoration. A planar face
traversal cannot see holes, so an unframed reserve sits inside some face and
every block cut from that face is wrong. With the frame, the face containing
the park is exactly the frame's interior and is simply not emitted as a block —
and the park gets what a real park has, frontage with houses facing it.

THE PARK IS NOT A RECTANGLE, THOUGH THE RESERVE IS. Every reason the frame is
laid is a reason about DEVELOPED land: houses to front it, an address to reach,
a way in from the streets. Where the neighbour is an undeveloped parcel none of
those exist, and a carriageway there is a line ruled between two pieces of open
ground that are otherwise the same ground. So it is not laid — the last pass
takes those segments back out (:func:`_merge_open_land`), the two faces become
one, and the park's published extent, ``info["park"]["poly"]``, is the reserve
plus whatever merged into it: a polygon, and usually a fat L or T rather than a
box. Measured over twelve seeds it absorbs 1.8 parcels and runs 12.6 ha (the
reserve alone) to 29.5 ha.

``rect`` KEEPS ITS MEANING and is still a rectangle: it is the region
`suburb_park` plans its own content into, and that module wants a box. The two
answer different questions — where the park's contents go, and how far the open
space runs — so both are published and `poly` contains `rect` exactly.

Step 2 does the real work and step 3 rarely fires; that is honest rather than
ideal, and it happens because a face split and a loop compete for the same
frontage. Growing streets opportunistically off other streets was tried first
and abandoned: it only thickens fabric that already exists, so it produced
ladder rungs between whichever collectors happened to run parallel and left the
rest of the section empty. Open ground has nothing to grow FROM. Subdividing
faces attacks it the other way round and always works on the biggest remaining
parcel, so coverage is guaranteed.

TARGETS, AND WHERE THIS LANDS AGAINST THEM
------------------------------------------
Measured suburbs (OSM, five US subdivisions) and the literature, against the
mean of twelve seeds at 1600 x 1200 m with the defaults below, park included:

    measure               target           measured   spread over 12 seeds
    dead-end nodes        12-35%           14.5%      12.5 - 17.2
    three-way junctions   69-87%           82.4%      73.0 - 89.3
    street density        7.4-12.4 km/km²  10.6       9.9 - 11.3
    block area (median)   15k-34k m²       28.0k      25.6k - 30.5k

    and three the shape, sparseness and merge passes are tuned on or report,
    not OSM targets:
    blocks under 0.62      —               23.0%      14.3 - 38.0
    undeveloped parcels   `undeveloped_share` 10.6%    5.2 - 16.3
    open space (park+merged) —             21.7 ha    12.6 - 29.7

(`undeveloped parcels` counts the ones still standing as their own BLOCK. It
reads lower than `undeveloped_share` asks because 1.8 parcels a seed merge into
the park and stop being blocks at all; `info["undeveloped_polys"]` has every
parcel the subdivision pass left whole, merged or not.)

All four means sit in band. WHAT SPARSENESS COST, against the same twelve seeds
with `undeveloped_share` at 0 (12.5% / 82.6% / 11.0 / 26.0k / 22.5% under 0.62):
street density falls 0.21 km/km² and median block area rises 2.7k, which is the
whole of the trade and both directions are the obvious ones — a parcel nobody
cut is a street that was never built and a block that stayed whole. The dead-end
share RISES 1.7 points, because the cul-de-sac pass grows until its target share
is met and there are fewer junctions to dilute it. Block shape is untouched
(23.3% against 22.5%): the two passes are independent, and the shape figure is
the shape term's alone.

WHAT THE PARK COST, measured against the same
twelve seeds with `park_enable` off (14.1% / 81.4% / 11.7 / 28.1k): density
falls 0.8 km/km², because 15.6 ha of keep-out is land the subdivision pass no
longer runs streets through and the frame does not cost as much as it saved.
The three-way share rises 4.4 points and is the tightest of the four — every
junction the park adds is a T, the four approaches at both ends and every
subdivision cut that lands on the frame, while four-ways come from collectors
crossing things and the park takes a 640 x 520 m bite out of the corridors a
collector can use. Individual seeds do run over 87% where they did not before;
`park_entrances` and `four_way_chance` are the two knobs that move it.

KNOWN, MEASURED, NOT FIXED. Over 24 seeds with and without the park, counting
non-adjacent carriageways closer than half their widths plus 4 m (ends trimmed
by `skip_ends_m`, crossings exempt within 30 m, same street exempt): 8 without
the park, 17 with. Streets leaving one node under 30 degrees apart: 0 without,
5 with. Both come from the SAME pre-existing hole rather than from anything the
reserve does — `_best_cut` fork-checks its far end unconditionally but its near
end only when the cut was deliberately snapped to a junction, so a cut that
lands within `split_edge`'s 1 m tolerance of an existing node by accident gets
no fork check at all. The park makes it show up more often because the frame
adds roughly ten junctions on a curving host. The fix belongs in `_best_cut`,
not here. STILL NOT FIXED, and re-measured on the twelve-seed sweep the shape
and sparseness work was checked against: 3 overlapping pairs and 2 sub-30-degree
forks, identical before and after that work — every one of them a subdivision
cut meeting a COLLECTOR, which is the shape the description above predicts.
Barrington-Leigh & Millard-Ball (PNAS 2015)
independently give 26.6% dead ends for US intersections built 1993-97 and 19.1%
for 2008-12, metro extremes 19-42%.

WHAT MADE THE LAST TWO FIT was removing the arterial ring, not tuning. While the
region was framed by a border road, that racetrack contributed about 2.4 km/km²
of pavement serving lots on one side only, which pinned density near 13.2 and
forced `block_area_target_m2` up to ~65k to compensate — leaving deep stranded
cores in the middle of every block. Treating the region as a CROP of a larger
suburb instead, with streets simply running off the edge, freed that budget and
let the target come back down to 40k, where blocks are both fully built and the
right size. Density and block area do still trade against each other — for
compact blocks of area A the through-street length is about 2000/sqrt(A) km/km²
— so :func:`stats` reports all four and the trade stays visible.

GEOMETRIC STANDARDS
-------------------
* Cul-de-sac turnaround is a fire-apparatus dimension. IFC Appendix D, Table
  D103.4: a dead end over 150 ft needs a turnaround; for 151-500 ft that is a
  96 ft DIAMETER cul-de-sac — a 14.6 m radius to the driving surface. Length is
  capped at 500-750 ft by most ordinances. The paving is only half of what a
  turnaround is: the LOTS round one are wedges on the arc rather than
  rectangles on a tangent, which is `suburb_parcel`'s business and the reason
  :func:`_arc_cap_bulbs` publishes each bulb on the block it belongs to.
* Local street 10.7 m kerb to kerb (SUDAS Table 5C-1.01 residential + parking),
  collector 11.6 m, arterial 14 m.
* Residential centreline curve radius bottoms out near 30 m in real
  subdivisions; `min_radius_m` rejects anything sharper so streets stay
  drivable rather than kinked.

WHAT THIS MODULE DOES NOT DO
----------------------------
It produces a network and its blocks. It does not write USD and does not parcel
blocks into lots — those are separate passes over this output. Run
`tools/suburb_net_png.py` to see a plan; that is the iteration loop.
"""

import math

_TOL = 1e-9


# ---------------------------------------------------------------------------
# vector / polyline geometry  (pure Python: no shapely, no numpy dependency)
# ---------------------------------------------------------------------------

def _add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _mul(a, s):
    return (a[0] * s, a[1] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _norm(a):
    return math.hypot(a[0], a[1])


def _unit(a):
    n = _norm(a)
    return (a[0] / n, a[1] / n) if n > _TOL else (1.0, 0.0)


def _perp(a):
    """Left normal. Consistently left-handed so 'which side' stays meaningful."""
    return (-a[1], a[0])


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def polyline_length(pts):
    return sum(_dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def _cumulative(pts):
    out = [0.0]
    for i in range(len(pts) - 1):
        out.append(out[-1] + _dist(pts[i], pts[i + 1]))
    return out


def point_at(pts, s):
    """Point at arclength *s* along the polyline, clamped to its ends."""
    cum = _cumulative(pts)
    total = cum[-1]
    if s <= 0.0:
        return pts[0]
    if s >= total:
        return pts[-1]
    for i in range(len(pts) - 1):
        if cum[i + 1] >= s:
            seg = cum[i + 1] - cum[i]
            f = (s - cum[i]) / seg if seg > _TOL else 0.0
            return _add(pts[i], _mul(_sub(pts[i + 1], pts[i]), f))
    return pts[-1]


def tangent_at(pts, s):
    """Unit tangent at arclength *s*, pointing from pts[0] toward pts[-1]."""
    cum = _cumulative(pts)
    total = cum[-1]
    s = max(0.0, min(total, s))
    for i in range(len(pts) - 1):
        if cum[i + 1] >= s - _TOL:
            d = _sub(pts[i + 1], pts[i])
            if _norm(d) > _TOL:
                return _unit(d)
    return _unit(_sub(pts[-1], pts[0]))


def split_polyline(pts, s):
    """Split into ``(head, tail)`` at arclength *s*. Both share the cut point."""
    cum = _cumulative(pts)
    total = cum[-1]
    if s <= _TOL:
        return [pts[0]], list(pts)
    if s >= total - _TOL:
        return list(pts), [pts[-1]]
    p = point_at(pts, s)
    head, tail = [], []
    placed = False
    for i in range(len(pts)):
        if not placed and cum[i] >= s - _TOL:
            placed = True
        (tail if placed else head).append(pts[i])
    head.append(p)
    tail.insert(0, p)
    return head, tail


def resample(pts, step):
    """Even-arclength resample, endpoints preserved. Keeps curvature tests fair."""
    total = polyline_length(pts)
    if total < _TOL:
        return list(pts)
    n = max(2, int(math.ceil(total / max(step, 1e-3))))
    return [point_at(pts, total * k / n) for k in range(n + 1)]


def hermite(p0, t0, p1, t1, tension=0.45, samples=24):
    """Cubic Hermite from *p0* to *p1* with UNIT end tangents *t0*, *t1*.

    This is the whole reason junctions come out right. A street is solved as a
    boundary-value problem — leave the host perpendicular, arrive at the next
    host perpendicular — instead of being drawn and then checked. *tension*
    scales the tangent magnitudes by the chord length; 0.45 gives a full,
    suburban-looking sweep without the S-kink that large values produce.
    """
    chord = _dist(p0, p1)
    m0, m1 = _mul(t0, chord * tension), _mul(t1, chord * tension)
    out = []
    for k in range(samples + 1):
        u = k / samples
        u2, u3 = u * u, u * u * u
        h00 = 2 * u3 - 3 * u2 + 1
        h10 = u3 - 2 * u2 + u
        h01 = -2 * u3 + 3 * u2
        h11 = u3 - u2
        out.append((h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0],
                    h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1]))
    return out


def min_radius(pts):
    """Smallest circumradius over consecutive triples — the curvature check.

    Rejecting on radius rather than on turn angle is what keeps the test
    independent of how densely the curve happens to be sampled.
    """
    best = float("inf")
    for i in range(1, len(pts) - 1):
        a, b, c = pts[i - 1], pts[i], pts[i + 1]
        ab, bc, ca = _dist(a, b), _dist(b, c), _dist(c, a)
        area2 = abs(_cross(_sub(b, a), _sub(c, a)))
        if area2 < 1e-9 or ab * bc * ca < 1e-9:
            continue
        best = min(best, ab * bc * ca / (2.0 * area2))
    return best


def seg_seg_dist(a0, a1, b0, b1):
    """Distance between two segments; 0 when they intersect."""
    d1, d2 = _sub(a1, a0), _sub(b1, b0)
    r = _sub(a0, b0)
    a, e, f = _dot(d1, d1), _dot(d2, d2), _dot(d2, r)
    if a < _TOL and e < _TOL:
        return _dist(a0, b0)
    if a < _TOL:
        s, t = 0.0, max(0.0, min(1.0, f / e))
    else:
        c = _dot(d1, r)
        if e < _TOL:
            t, s = 0.0, max(0.0, min(1.0, -c / a))
        else:
            b = _dot(d1, d2)
            denom = a * e - b * b
            s = max(0.0, min(1.0, (b * f - c * e) / denom)) if denom > _TOL else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, max(0.0, min(1.0, (b - c) / a))
    p = _add(a0, _mul(d1, s))
    q = _add(b0, _mul(d2, t))
    return _dist(p, q)


def bbox(pts):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys))


def bbox_gap(a, b):
    """Distance between two AABBs; 0 when they overlap. A cheap lower bound on
    the distance between anything inside them."""
    dx = max(b[0] - a[2], a[0] - b[2], 0.0)
    dy = max(b[1] - a[3], a[1] - b[3], 0.0)
    return math.hypot(dx, dy)


def polyline_dist(pa, pb, limit=None):
    """Minimum distance between two polylines.

    *limit* is an early-out: callers only ever ask "is this closer than the
    minimum gap", so once something below *limit* is found the exact value does
    not matter. With a bounding-box pre-test this turns the inner loop from
    every-segment-against-every-segment into a handful of comparisons, which is
    what makes subdividing a few hundred faces tractable.
    """
    ba, bb = bbox(pa), bbox(pb)
    if limit is not None and bbox_gap(ba, bb) >= limit:
        return float("inf")
    best = float("inf")
    for i in range(len(pa) - 1):
        sa = (min(pa[i][0], pa[i + 1][0]), min(pa[i][1], pa[i + 1][1]),
              max(pa[i][0], pa[i + 1][0]), max(pa[i][1], pa[i + 1][1]))
        if limit is not None and bbox_gap(sa, bb) >= limit:
            continue
        for j in range(len(pb) - 1):
            best = min(best, seg_seg_dist(pa[i], pa[i + 1], pb[j], pb[j + 1]))
            if best < _TOL:
                return 0.0
            if limit is not None and best < limit:
                return best
    return best


def polygon_area(poly):
    """Signed area. Positive is counter-clockwise."""
    s = 0.0
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        s += _cross(a, b)
    return s / 2.0


def polygon_centroid(poly):
    a = polygon_area(poly)
    if abs(a) < _TOL:
        n = len(poly) or 1
        return (sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n)
    cx = cy = 0.0
    for i in range(len(poly)):
        p, q = poly[i], poly[(i + 1) % len(poly)]
        cr = _cross(p, q)
        cx += (p[0] + q[0]) * cr
        cy += (p[1] + q[1]) * cr
    return (cx / (6.0 * a), cy / (6.0 * a))


def convex_hull(pts):
    """Monotone-chain hull, CCW, no collinear points."""
    p = sorted(set((round(a, 6), round(b, 6)) for (a, b) in pts))
    if len(p) < 3:
        return list(p)

    def half(seq):
        out = []
        for q in seq:
            while len(out) >= 2 and _cross(_sub(out[-1], out[-2]),
                                           _sub(q, out[-1])) <= 0.0:
                out.pop()
            out.append(q)
        return out

    lo, up = half(p), half(list(reversed(p)))
    return lo[:-1] + up[:-1]


def min_area_rect(poly):
    """Area of the smallest-area rectangle enclosing *poly*, by rotating calipers.

    The minimum-area rectangle of a convex hull always has a side flush with a
    hull edge (Freeman & Shapira 1975), so trying every hull edge as the
    rectangle's axis is exact rather than a search.
    """
    h = convex_hull(poly)
    if len(h) < 3:
        return 0.0
    best = None
    n = len(h)
    for i in range(n):
        d = _sub(h[(i + 1) % n], h[i])
        L = _norm(d)
        if L < 1e-9:
            continue
        u = (d[0] / L, d[1] / L)
        v = _perp(u)
        us = [_dot(_sub(q, h[i]), u) for q in h]
        vs = [_dot(_sub(q, h[i]), v) for q in h]
        a = (max(us) - min(us)) * (max(vs) - min(vs))
        if best is None or a < best:
            best = a
    return best or 0.0


def rectangularity(poly):
    """Area over the area of the polygon's own minimum-area bounding rectangle.

    THE SHAPE NUMBER THIS MODULE IS TUNED ON, because it is the one that
    separates the two block shapes a plat actually cares about while ignoring
    the one it does not: ORIENTATION. A rectangle scores 1.0 whichever way it
    is turned; a triangle scores exactly 0.5 whatever its proportions, because
    a triangle is always half of some enclosing rectangle; a trapezoid with one
    side clipped lands between the two. Convexity or a perimeter-based
    compactness ratio would both call a long thin rectangle bad, and a long
    thin rectangle is a perfectly ordinary suburban block.

    Measured on the shipped generator before the shape term existed, 47.5% of
    blocks scored under 0.62 — nearly half the fabric was a corner sliced off a
    face, which is not what residential platting produces.
    """
    a = abs(polygon_area(poly))
    r = min_area_rect(poly)
    return (a / r) if r > 1e-9 else 0.0


def offset_polygon(poly, insets):
    """Shrink a CCW polygon, edge *i* moving inward by ``insets[i]``.

    Per-edge rather than uniform because the inset is half the width of the
    ROAD on that edge, and a block can face an arterial on one side and a
    cul-de-sac on another. Each edge line is moved inward and consecutive lines
    are intersected; near-parallel neighbours fall back to the offset point so a
    straight run never blows up to infinity.
    """
    n = len(poly)
    if n < 3:
        return []
    lines = []
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        d = _unit(_sub(b, a))
        inward = _perp(d)                     # CCW polygon: left is interior
        off = _mul(inward, insets[i % len(insets)])
        lines.append((_add(a, off), _add(b, off)))
    out = []
    for i in range(n):
        p0, p1 = lines[(i - 1) % n]
        q0, q1 = lines[i]
        d1, d2 = _sub(p1, p0), _sub(q1, q0)
        den = _cross(d1, d2)
        # MITRE LIMIT. Two offset lines meeting at a shallow angle intersect
        # far away — the sharper the corner, the further. A bare
        # near-parallel guard is not enough because `den` stays comfortably
        # above any epsilon while the intersection still runs off to infinity:
        # measured, one block came out spanning 30,876 m, from x = -31,284 to
        # x = -407, and rendered as a grass sheet stretching past the horizon.
        # Beyond the limit the corner is cut square instead, which is what a
        # mitre limit does in every stroking implementation.
        limit = max(insets[i % len(insets)], 1.0) * 6.0
        if abs(den) > 1e-9:
            t = _cross(_sub(q0, p0), d2) / den
            v = _add(p0, _mul(d1, t))
            if _dist(v, poly[i]) <= limit:
                out.append(v)
                continue
        out.append(q0)
    return out


# ---------------------------------------------------------------------------
# road classes
# ---------------------------------------------------------------------------

# Widths are kerb to kerb. Sources in the module docstring.
CLASSES = {
    "arterial":  {"width_m": 14.0, "lanes": 4, "min_radius_m": 120.0},
    "collector": {"width_m": 11.6, "lanes": 2, "min_radius_m": 60.0},
    "local":     {"width_m": 10.7, "lanes": 2, "min_radius_m": 30.0},
    "cul_de_sac": {"width_m": 9.1, "lanes": 2, "min_radius_m": 25.0},
    # The crop edge. Zero width because it is not pavement: it closes the planar
    # graph so faces exist, and nothing insets from it or draws it.
    "boundary":  {"width_m": 0.0, "lanes": 0, "min_radius_m": 1e9},
}


class Node:
    __slots__ = ("id", "p", "edges")

    def __init__(self, nid, p):
        self.id = nid
        self.p = p
        self.edges = []          # edge ids incident to this node

    @property
    def degree(self):
        return len(self.edges)

    def road_degree(self, net):
        """Incident edges that are actual pavement, ignoring the crop edge."""
        return sum(1 for eid in self.edges
                   if net.edges[eid].road_class != "boundary")


class Edge:
    __slots__ = ("id", "a", "b", "pts", "road_class", "street_type", "width_m",
                 "street_id")

    def __init__(self, eid, a, b, pts, road_class, street_type, width_m,
                 street_id=0):
        self.id = eid
        self.a = a               # node id at pts[0]
        self.b = b               # node id at pts[-1]
        self.pts = pts
        self.road_class = road_class
        self.street_type = street_type
        self.width_m = width_m
        # A NAMED STREET, not a graph edge. Every junction placed on a street
        # splits the edge under it, so after a dozen cul-de-sacs a collector is
        # a dozen edges — and a loop needing 300 m of frontage would never find
        # a host again. Streets that share this id are one continuous road and
        # are addressed as one polyline (:func:`street_chain`), which is also
        # how OSM models a way that has been split at its junctions.
        self.street_id = street_id

    @property
    def length(self):
        return polyline_length(self.pts)

    @property
    def half_w(self):
        return self.width_m / 2.0


class Network:
    """Planar street graph. Nodes are junctions and dead ends; edges carry
    polyline geometry."""

    def __init__(self):
        self.nodes = {}
        self.edges = {}
        self._nid = 0
        self._eid = 0
        self._sid = 0

    def new_street_id(self):
        self._sid += 1
        return self._sid

    # -- construction -----------------------------------------------------
    def add_node(self, p):
        self._nid += 1
        self.nodes[self._nid] = Node(self._nid, p)
        return self._nid

    def node_at(self, p, tol=0.5):
        """Reuse a node within *tol*, so streets that meet share a junction
        rather than stacking two nodes a centimetre apart."""
        for n in self.nodes.values():
            if _dist(n.p, p) <= tol:
                return n.id
        return None

    def add_edge(self, pts, road_class, street_type, a=None, b=None,
                 width_m=None, street_id=None):
        if a is None:
            a = self.node_at(pts[0]) or self.add_node(pts[0])
        if b is None:
            b = self.node_at(pts[-1]) or self.add_node(pts[-1])
        # Snap geometry to the nodes so the graph and the drawing agree exactly.
        pts = list(pts)
        pts[0], pts[-1] = self.nodes[a].p, self.nodes[b].p
        self._eid += 1
        w = width_m if width_m is not None else CLASSES[road_class]["width_m"]
        sid = self.new_street_id() if street_id is None else street_id
        e = Edge(self._eid, a, b, pts, road_class, street_type, w, sid)
        self.edges[e.id] = e
        self.nodes[a].edges.append(e.id)
        self.nodes[b].edges.append(e.id)
        return e

    def remove_edge(self, eid):
        e = self.edges.pop(eid)
        for nid in (e.a, e.b):
            if eid in self.nodes[nid].edges:
                self.nodes[nid].edges.remove(eid)
        return e

    def split_edge(self, edge, s):
        """Split *edge* at arclength *s*, returning the new middle node id.

        This is what makes a T-junction a real graph junction rather than two
        streets that merely touch: the host is genuinely cut, and the node it
        gains has degree 3.
        """
        total = edge.length
        if s <= 1.0:
            return edge.a
        if s >= total - 1.0:
            return edge.b
        head, tail = split_polyline(edge.pts, s)
        a, b = edge.a, edge.b
        cls, st, w, sid = (edge.road_class, edge.street_type, edge.width_m,
                           edge.street_id)
        self.remove_edge(edge.id)
        mid = self.add_node(head[-1])
        self.add_edge(head, cls, st, a=a, b=mid, width_m=w, street_id=sid)
        self.add_edge(tail, cls, st, a=mid, b=b, width_m=w, street_id=sid)
        return mid

    # -- queries ----------------------------------------------------------
    def clearance(self, pts, ignore=(), skip_ends_m=0.0, limit=None):
        """Smallest distance from *pts* to any edge not in *ignore*.

        *skip_ends_m* trims that much off each end of the candidate before
        measuring, because the ends are AT their junctions by construction and
        would otherwise report zero clearance against the host they just joined.
        """
        probe = pts
        if skip_ends_m > 0.0:
            total = polyline_length(pts)
            if total <= 2.0 * skip_ends_m + 1.0:
                return float("inf")
            _h, probe = split_polyline(pts, skip_ends_m)
            probe, _t = split_polyline(probe, polyline_length(probe) - skip_ends_m)
        best = float("inf")
        pb = bbox(probe)
        for e in self.edges.values():
            if e.id in ignore:
                continue
            if limit is not None and bbox_gap(pb, bbox(e.pts)) >= limit:
                continue
            best = min(best, polyline_dist(probe, e.pts, limit=limit))
            if best <= _TOL:
                return 0.0
            if limit is not None and best < limit:
                return best
        return best

    def clearance_allowing_crossings(self, pts, ignore=(), limit=None,
                                     junction_r=30.0, own_hw=5.8):
        """Clearance, exempting only the NEIGHBOURHOOD OF A CROSSING.

        Two roads that cross are not too close together at the crossing — that
        is a four-way, and measuring raw distance there would score it as zero
        clearance and reject it, which is why only one collector ever landed.

        But exempting the whole crossed EDGE, as this did originally, is far too
        coarse: once two collectors crossed anywhere, each was free to run
        alongside the other everywhere else, and nothing objected. Measured, that
        is where the remaining overlapping carriageways came from — a concrete
        case had two collectors 6.7 m apart, needing 11.6 m, for an 8 m run,
        sharing no node, simply because they happened to cross further along.

        So the exemption is now a RADIUS around each actual crossing point.
        Away from its crossings the pair is measured like any other, which is
        the behaviour the name always implied.
        """
        best = float("inf")
        for e in self.edges.values():
            if e.id in ignore:
                continue
            xs = _crossing_points(pts, e.pts)
            if not xs:
                best = min(best, polyline_dist(pts, e.pts, limit=limit))
            else:
                # A CROSSED pair is held to a different standard: not the full
                # street spacing, which two roads that cross can never satisfy
                # near the crossing (requiring it dropped collectors from 3 to
                # 1), but simply NOT OVERLAPPING. They may run close; they may
                # not be laid on top of each other.
                need = own_hw + e.half_w + 4.0
                probe = [q for q in pts
                         if all(_dist(q, x) > junction_r for x in xs)]
                for i in range(len(probe) - 1):
                    if _dist(probe[i], probe[i + 1]) > 3.0 * junction_r:
                        continue
                    d = polyline_dist([probe[i], probe[i + 1]], e.pts,
                                      limit=need)
                    if d < need:
                        return 0.0          # overlapping away from a crossing
            if best <= _TOL:
                return 0.0
            if limit is not None and best < limit:
                return best
        return best

    def edges_of_class(self, *classes):
        return [e for e in self.edges.values() if e.road_class in classes]

    def street_ids(self, *street_types):
        out = []
        for e in self.edges.values():
            if street_types and e.street_type not in street_types:
                continue
            if e.street_id not in out:
                out.append(e.street_id)
        return out

    def street_chain(self, sid):
        """All edges with street id *sid*, ordered head to tail.

        Returns ``(pts, parts)`` where *parts* is ``[(edge, flipped), ...]`` in
        the same order, so an arclength along *pts* can be mapped back to the
        edge that must actually be split.
        """
        edges = [e for e in self.edges.values() if e.street_id == sid]
        if not edges:
            return [], []
        # Node -> incident edges within this street only.
        inc = {}
        for e in edges:
            inc.setdefault(e.a, []).append(e)
            inc.setdefault(e.b, []).append(e)
        ends = [n for n, es in inc.items() if len(es) == 1]
        start = ends[0] if ends else edges[0].a      # ring: start anywhere
        chain, seen, node = [], set(), start
        while True:
            nxt = None
            for e in inc.get(node, []):
                if e.id not in seen:
                    nxt = e
                    break
            if nxt is None:
                break
            seen.add(nxt.id)
            flipped = (nxt.b == node)
            chain.append((nxt, flipped))
            node = nxt.a if flipped else nxt.b
        pts = []
        for (e, flipped) in chain:
            seg = list(reversed(e.pts)) if flipped else e.pts
            pts.extend(seg if not pts else seg[1:])
        return pts, chain

    def locate_on_street(self, sid, s):
        """Map arclength *s* along street *sid* to ``(edge, local_s, pt, tan)``."""
        pts, chain = self.street_chain(sid)
        if not pts:
            return None
        acc = 0.0
        for (e, flipped) in chain:
            L = e.length
            if s <= acc + L or (e, flipped) is chain[-1]:
                local = s - acc
                local = max(0.0, min(L, local))
                if flipped:
                    local = L - local
                p = point_at(e.pts, local)
                t = tangent_at(e.pts, local)
                if flipped:
                    t = _mul(t, -1.0)
                return e, local, p, t
            acc += L
        return None


# ---------------------------------------------------------------------------
# planar face extraction — blocks are the faces of the graph
# ---------------------------------------------------------------------------

def faces(net):
    """Minimal cycles of the planar graph, as CCW polygons with their edges.

    Standard half-edge traversal: at each node the incident half-edges are
    sorted by heading, and walking a face means always taking the NEXT
    half-edge clockwise from the reverse of the one just traversed. Every
    interior face comes out counter-clockwise and the single outer face comes
    out clockwise, which is how it is told apart.

    Returns ``[{"poly": [...], "edges": [eid, ...]}, ...]`` — interior faces
    only. Dangling edges (a cul-de-sac stem) are traversed in both directions
    within the face that contains them, which is correct: a dead end does not
    divide the land, it pokes into it.
    """
    # Half-edge = (edge_id, from_node). Heading is the tangent leaving from_node.
    def heading(eid, frm):
        e = net.edges[eid]
        pts = e.pts if e.a == frm else list(reversed(e.pts))
        for k in range(1, len(pts)):
            d = _sub(pts[k], pts[0])
            if _norm(d) > 1e-6:
                return math.atan2(d[1], d[0])
        return 0.0

    order = {}
    for nid, node in net.nodes.items():
        hes = [(eid, nid) for eid in node.edges]
        hes.sort(key=lambda h: heading(h[0], h[1]))
        order[nid] = hes

    def next_he(eid, frm):
        """After arriving at `to` along (eid, frm), the next half-edge of the
        face is the one just clockwise from the reverse half-edge."""
        e = net.edges[eid]
        to = e.b if e.a == frm else e.a
        ring = order[to]
        rev = (eid, to)
        i = ring.index(rev)
        return ring[(i - 1) % len(ring)]

    seen = set()
    out = []
    for nid, node in net.nodes.items():
        for eid in node.edges:
            start = (eid, nid)
            if start in seen:
                continue
            cycle, he = [], start
            guard = 0
            while he not in seen and guard < 20000:
                seen.add(he)
                cycle.append(he)
                he = next_he(*he)
                guard += 1
            if not cycle or he != start:
                continue
            poly, eids = [], []
            for (ce, cf) in cycle:
                e = net.edges[ce]
                pts = e.pts if e.a == cf else list(reversed(e.pts))
                poly.extend(pts[:-1])
                eids.append(ce)
            if len(poly) < 3:
                continue
            if polygon_area(poly) <= 0.0:      # clockwise -> the outer face
                continue
            # `hes` keeps each edge's DIRECTION of travel around the face. The
            # face is counter-clockwise, so the interior is always to the left
            # of that direction — which is what lets a street inserted across
            # the face know which way "inward" is without a containment test.
            out.append({"poly": poly, "edges": eids, "hes": list(cycle)})
    return out


def point_in_polygon(poly, p):
    """Crossing-number test."""
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if (a[1] > y) != (b[1] > y):
            xin = a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if x < xin:
                inside = not inside
    return inside


def _arc_cap_bulbs(poly, frontage, net, eids, verge=3.0, bulbs_out=None):
    """Replace each cul-de-sac tip in a block boundary with the turnaround ARC.

    WHY. A cul-de-sac stem is a DANGLING edge, so `faces()` walks out along it
    and back, and the inset turns that into a ~9 m slit whose end cap is a
    diagonal a couple of metres from the tip. The 14.64 m turnaround disc never
    enters the boundary at all: measured, 84.9% of every paved bulb lies INSIDE
    the block polygon as nominally buildable land, and the nearest boundary
    vertex sits 4.5 m from the tip instead of ~14.6 m.

    The consequence is not that houses stand on the paving — the keep-out disc
    stops that — it is that there is no ARC to hang lots off, so the head of
    every cul-de-sac comes out bald: 19% of bulbs had no house within 45 m, and
    of those that did, 30% faced AWAY from the turnaround because they were
    platted against the straight stem wall instead.

    Putting the arc into the boundary makes the turnaround real frontage that
    the arclength walk can plat, and demotes the keep-out disc back to a
    belt-and-braces check. *verge* is the strip between kerb and lot line.

    AND IT IS NOT ENOUGH ON ITS OWN. Frontage that curves through 330 degrees
    is not the same kind of frontage as a street: the lots on it are WEDGES
    whose side lines converge on the turnaround centre, not rectangles hung off
    a tangent, and the pass that plats them has to be told which arc is which.
    So each bulb spliced in is appended to *bulbs_out* as
    ``{"c": tip, "r": lot-line radius, "r_pave": kerb radius}`` and travels on
    the block; `suburb_parcel` finds the run of boundary vertices at ``r`` from
    ``c`` and plats that span radially instead of walking it as arclength. The
    two radii are both wanted downstream and differ by exactly *verge* — the
    lot line is where the wedges start, the kerb is where a driveway apron has
    to reach to touch asphalt.
    """
    R = DEFAULTS["bulb_radius_m"] + verge
    seen = set()
    for eid in eids:
        e = net.edges.get(eid)
        if e is None or e.street_type != "lollipop" or len(e.pts) < 2:
            continue
        # ONCE PER EDGE, NOT ONCE PER HALF-EDGE. A cul-de-sac stem is dangling,
        # so the face traversal walks out along it and back and `eids` names it
        # TWICE. The second visit found the arc it had just spliced in still
        # "near" the tip — the vertices are at exactly R and land a float ulp
        # inside it — and spliced a second copy over the first, leaving a
        # boundary that ran round the turnaround, back, and round again. The
        # polygon still closed and still had the right area, so nothing caught
        # it; what it broke was the ANGLE walk, which needs the arc monotone.
        if eid in seen:
            continue
        seen.add(eid)
        tip = e.pts[-1]
        near = [i for i, q in enumerate(poly) if _dist(q, tip) < R]
        if not near or len(near) >= len(poly) - 2:
            continue
        # The run must be contiguous around the ring; rotate so it is.
        n = len(poly)
        start = next((i for i in near if (i - 1) % n not in near), None)
        if start is None:
            continue
        run = [start]
        while (run[-1] + 1) % n in near and len(run) < n:
            run.append((run[-1] + 1) % n)
        prev_i, next_i = (run[0] - 1) % n, (run[-1] + 1) % n
        a0 = math.atan2(poly[prev_i][1] - tip[1], poly[prev_i][0] - tip[0])
        a1 = math.atan2(poly[next_i][1] - tip[1], poly[next_i][0] - tip[0])
        # Sweep the way that passes through the OUTWARD stem direction, which
        # is the side the turnaround actually bulges to.
        out_ang = math.atan2(tip[1] - e.pts[-2][1], tip[0] - e.pts[-2][0])
        def _norm(a):
            while a <= -math.pi:
                a += 2 * math.pi
            while a > math.pi:
                a -= 2 * math.pi
            return a
        ccw = _norm(out_ang - a0) % (2 * math.pi) <= _norm(a1 - a0) % (2 * math.pi)
        sweep = (_norm(a1 - a0) % (2 * math.pi)) if ccw else \
                -((_norm(a0 - a1)) % (2 * math.pi))
        steps = max(6, int(abs(sweep) / 0.30))
        arc = [(tip[0] + R * math.cos(a0 + sweep * k / steps),
                tip[1] + R * math.sin(a0 + sweep * k / steps))
               for k in range(1, steps)]
        keep = [(poly[i], frontage[i]) for i in range(n) if i not in set(run)]
        # Rebuild with the arc spliced in where the run was. Arc vertices ARE
        # frontage — that is the whole point.
        at = keep.index((poly[prev_i], frontage[prev_i]))
        merged = keep[:at + 1] + [(q, True) for q in arc] + keep[at + 1:]
        poly = [q for q, _f in merged]
        frontage = [f for _q, f in merged]
        if bulbs_out is not None:
            bulbs_out.append({"c": (float(tip[0]), float(tip[1])), "r": R,
                              "r_pave": DEFAULTS["bulb_radius_m"]})
    return poly, frontage


def blocks_from_faces(net, face_list, min_area=400.0, reserve=None,
                      undeveloped_pts=()):
    """Faces inset by the half-width of the road bounding each side.

    The face polygon runs down street CENTRELINES, so the buildable parcel is
    the face pulled in by half a carriageway on every side. Doing it per edge
    rather than uniformly is what lets a block that faces an arterial on one
    side and a cul-de-sac on another sit correctly against both.

    A face that encloses the reserve is NOT a block. It is the land inside the
    park frame, which was set aside before the first street was laid — emitting
    it would hand the parcelling pass 12.6 ha of park to build houses on.

    A face the subdivision pass deliberately left whole IS still a block —
    `undeveloped_pts` carries one interior point per such face, and the block it
    lands in is flagged ``undeveloped``. It has to be emitted, or the ground
    under it is never drawn and the parcel reads as a hole in the world; the
    flag is what tells the passes that build houses to leave it as open land.
    Every block carries the key, so a consumer can test it without a default.

    WITH ONE EXCEPTION, and it is the reserve rule above rather than a second
    rule: a parcel that :func:`_merge_open_land` folded into the park is part of
    the park's face now, so the face it is in covers the reserve and no block
    comes out of it at all. Its ground is drawn by whatever draws the park —
    ``info["park"]["poly"]`` is the extent, and it includes that parcel.

    ``bulbs`` NAMES THE ARCS. :func:`_arc_cap_bulbs` puts each turnaround into
    the boundary as a run of vertices, and from the polygon alone there is no
    way back to which circle they came off — the parcelling pass needs the
    centre and the radius to strike a wedge lot's side lines, and the paved
    radius to bring a driveway apron to the asphalt. So each spliced bulb is
    listed as ``{"c": tip, "r": lot-line radius, "r_pave": kerb radius}``. Every
    block carries the key, empty on the ones with no cul-de-sac in them.
    """
    out = []
    for f in face_list:
        poly, eids = f["poly"], f["edges"]
        if reserve is not None and reserve.covers(poly):
            continue
        undeveloped = any(point_in_polygon(poly, q) for q in undeveloped_pts)
        # One inset per polygon vertex, taken from the edge that vertex starts.
        # `frontage` marks, for the same vertex, whether that side is actual
        # pavement — the crop boundary is not, and a lot cannot face it. Without
        # this the parcelling pass hangs a row of houses along the region edge
        # fronting nothing at all.
        insets, frontage = [], []
        for eid in eids:
            e = net.edges[eid]
            n_pts = len(e.pts) - 1
            is_road = e.road_class != "boundary"
            insets.extend([e.half_w] * n_pts)
            frontage.extend([is_road] * n_pts)
        if len(insets) != len(poly):
            pad = len(poly) - len(insets)
            insets = (insets + [CLASSES["local"]["width_m"] / 2.0] * max(pad, 0))[:len(poly)]
            frontage = (frontage + [True] * max(pad, 0))[:len(poly)]
        shrunk = offset_polygon(poly, insets)
        if len(shrunk) < 3:
            continue
        # Give every cul-de-sac head its turnaround arc, or the parcel pass has
        # no frontage to plat there. See `_arc_cap_bulbs`.
        bulbs = []
        shrunk, frontage = _arc_cap_bulbs(shrunk, frontage, net, eids,
                                          bulbs_out=bulbs)
        a = polygon_area(shrunk)
        if a < min_area:
            continue
        # `bulbs` is the arc's provenance, not a second copy of the geometry:
        # the vertices are already in `poly`, and this says which of them are a
        # turnaround and where its centre is. Every block carries the key so a
        # consumer can test it without a default, the same contract
        # `undeveloped` keeps.
        out.append({"poly": shrunk, "area": a, "edges": eids,
                    "frontage": frontage, "undeveloped": undeveloped,
                    "bulbs": bulbs,
                    "centroid": polygon_centroid(shrunk)})
    return out


# ---------------------------------------------------------------------------
# the reserve — land the network is laid AROUND rather than through
# ---------------------------------------------------------------------------

def _pt_in_rect(p, rect, eps=0.0):
    x0, y0, x1, y1 = rect
    return (x0 + eps < p[0] < x1 - eps) and (y0 + eps < p[1] < y1 - eps)


def _seg_hits_rect(a, b, rect):
    """True when segment *a*-*b* passes through the INTERIOR of *rect*.

    Liang-Barsky clip to get the span of the segment that lies in the box, then
    a strictness test on the MIDPOINT of that span. The midpoint test is what
    separates entering from touching: a street that runs along the reserve's
    edge, or clips its corner, has a degenerate or boundary-only span and is
    allowed — which is exactly the case an entrance has to be, since an entrance
    must reach the reserve without going into it.
    """
    x0, y0, x1, y1 = rect
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for (p, q) in ((-dx, a[0] - x0), (dx, x1 - a[0]),
                   (-dy, a[1] - y0), (dy, y1 - a[1])):
        if abs(p) < 1e-12:
            if q < 0.0:
                return False                  # parallel and outside this slab
        else:
            r = q / p
            if p < 0.0:
                if r > t1:
                    return False
                t0 = max(t0, r)
            else:
                if r < t0:
                    return False
                t1 = min(t1, r)
    if t1 <= t0:
        return False
    tm = 0.5 * (t0 + t1)
    return _pt_in_rect((a[0] + dx * tm, a[1] + dy * tm), rect, eps=1e-6)


class Reserve:
    """Land set aside BEFORE any street is laid, and the streets laid around it.

    WHY THE PARK IS RESERVED FIRST. A neighbourhood park is 420 x 300 m — 12.6
    ha — while the median block this generator produces is about 2.8 ha. A park
    is therefore not a block, and it cannot be found by looking for a block big
    enough to hold one: no such block exists, and forcing one would mean leaving
    a 12.6 ha face undivided, which the subdivision pass exists to prevent. The
    same problem, at the same scale, was solved inside the park itself by
    routing its paths around the courts rather than placing courts in the gaps
    the paths left. This does that one level up: choose the rectangle, then lay
    the network around it.

    Two rectangles, and the difference matters:

        rect   the park proper, `size` metres, what the park module builds into
        keep   `rect` grown by `pad` — the STREET KEEP-OUT. No centreline may
               enter it, so the park gets a verge rather than a kerb pressed
               against its fence, and the park's own boundary planting has room

    Streets may TOUCH `keep`; that is what an entrance is, and why the
    containment test is strict about the interior rather than about contact.
    """

    __slots__ = ("rect", "pad", "keep", "size", "center")

    def __init__(self, rect, pad=0.0):
        x0, y0, x1, y1 = rect
        self.rect = (x0, y0, x1, y1)
        self.pad = float(pad)
        self.keep = (x0 - self.pad, y0 - self.pad, x1 + self.pad, y1 + self.pad)
        self.size = (x1 - x0, y1 - y0)
        self.center = (0.5 * (x0 + x1), 0.5 * (y0 + y1))

    # -- THE predicate ----------------------------------------------------
    def blocks(self, pts, clear=0.0):
        """True when centreline *pts* may not exist: it enters the keep-out.

        *clear* grows the keep-out for the things that occupy area rather than a
        line — a cul-de-sac bulb is a 14.6 m disc at the tip of its stem, and
        the stem staying out is not enough.
        """
        x0, y0, x1, y1 = self.keep
        r = (x0 - clear, y0 - clear, x1 + clear, y1 + clear)
        if len(pts) < 2:
            return bool(pts) and _pt_in_rect(pts[0], r)
        for i in range(len(pts) - 1):
            if _seg_hits_rect(pts[i], pts[i + 1], r):
                return True
        return False

    def covers(self, poly):
        """True when polygon *poly* encloses the reserved land.

        Because no centreline enters the keep-out, a face either contains the
        whole of it or misses it entirely — so one containment test settles it,
        and the corners are only belt and braces against a degenerate polygon.
        """
        if point_in_polygon(poly, self.center):
            return True
        # The PARK's corners, not the keep-out's. The frame street rounds the
        # keep-out's corners off, so its own fillet passes between the corner
        # and the park — testing the keep-out corner reported the face OUTSIDE
        # the frame as covering the reserve, and that face and everything cut
        # from it were being thrown away (block count 58 -> 34).
        x0, y0, x1, y1 = self.rect
        return any(point_in_polygon(poly, c)
                   for c in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)))

    # -- entrances --------------------------------------------------------
    def gate(self, p):
        """Where a street at *p* meets the reserve.

        Returns ``(entrance, gate, inward_normal, side)``: the point on the
        KEEP-OUT boundary nearest *p*, the matching point on the park's own
        boundary a padding band further in — which is where the park's gate
        goes — the inward normal, and which of the four sides it is on.
        """
        x0, y0, x1, y1 = self.keep
        cx, cy = self.center
        hx = max((x1 - x0) / 2.0, _TOL)
        hy = max((y1 - y0) / 2.0, _TOL)
        ux, uy = (p[0] - cx) / hx, (p[1] - cy) / hy
        if abs(ux) >= abs(uy):
            side = "E" if ux >= 0.0 else "W"
            q = (x1 if ux >= 0.0 else x0, max(y0, min(y1, p[1])))
            n = (-1.0, 0.0) if ux >= 0.0 else (1.0, 0.0)
        else:
            side = "N" if uy >= 0.0 else "S"
            q = (max(x0, min(x1, p[0])), y1 if uy >= 0.0 else y0)
            n = (0.0, -1.0) if uy >= 0.0 else (0.0, 1.0)
        g = _add(q, _mul(n, self.pad))
        gx0, gy0, gx1, gy1 = self.rect
        g = (max(gx0, min(gx1, g[0])), max(gy0, min(gy1, g[1])))
        return q, g, n, side


def reserve_blocks(reserve, pts, clear=0.0):
    """THE containment test, and the only one — every street-laying phase asks
    this and nothing else.

    Collectors, face subdivision and its fallback, loops and cul-de-sacs all
    propose a centreline and then run it past a list of reasons it might not be
    allowed. Adding a fifth reason in four places would have meant four chances
    to get it wrong and four places to change if the reserve ever stops being a
    rectangle; routing them all through one predicate means a phase that forgets
    to ask is visibly missing a line rather than quietly subtly different. It is
    None-safe so the whole feature switches off with `park_enable`.
    """
    return reserve is not None and reserve.blocks(pts, clear)


# ---------------------------------------------------------------------------
# generation
# ---------------------------------------------------------------------------

DEFAULTS = {
    # -- collectors ------------------------------------------------------
    "collectors": 3,               # curvilinear routes across the interior
    "collector_waypoints": [3, 5],
    "collector_wander": 0.28,      # fraction of region half-span
    # -- local streets ---------------------------------------------------
    # Recursive face subdivision: the layer that fills the land and sets grain.
    "subdivide_iters": 400,
    "block_area_target_m2": 40000.0,   # split any face bigger than this
    "block_area_min_m2": 7000.0,       # refuse a cut leaving less than this
    # A face this many times over target is GROSSLY oversized and gets the
    # relaxed fallback cut (:func:`_fallback_cut`) rather than being abandoned.
    # Deliberately not 1.0: a face a little over target that refuses every well
    # formed candidate is a normal, tolerable block, and cutting all of those
    # too pushed street density from 11.0 to 13.3 km/km² — outside the measured
    # band — and buried the cul-de-sacs, whose share fell from 17.9% to 5%.
    # Only the parcels that read as unsubdivided land are forced.
    "block_area_hard_max_factor": 1.5,
    # -- block shape -----------------------------------------------------
    # How hard a cut is pushed to leave two QUAD-ISH halves instead of slicing
    # a corner off the face. `block_shape_weight` multiplies (1 - the worse
    # half's `rectangularity`), so a triangular half (0.5) costs 1.3x against a
    # rectangular one (~1.0) costing nothing; `block_side_penalty` is charged
    # per missing corner of separation between the two ends, which catches the
    # same failure topologically before the areas go bad. Zero disables both
    # and reproduces the pre-shape fabric, which measured 47.5% of blocks under
    # 0.62 rectangularity — nearly half of them triangles.
    "block_shape_weight": 2.6,
    "block_side_penalty": 1.0,
    # Below this `rectangularity` the halves are not blocks, they are wedges,
    # and a candidate that leaves one is only used when NOTHING across the
    # parcel is legal. Measured against the final blocks rather than these
    # halves: a block is inset by half a carriageway on every side, which costs
    # a further 0.03-0.09, so a floor of 0.66 here is the 0.62 the finished
    # fabric is scored on.
    "block_shape_floor": 0.66,
    # Exterior angle at which the face boundary counts as turning a CORNER
    # rather than curving. Above the ~11.5 degrees per vertex a 30 m radius
    # street can reach when sampled every 6 m, below the 52 degrees a street
    # leaving a host at full `junction_skew_deg` makes. See `_face_corners`.
    "face_corner_deg": 40.0,
    # Share of candidates aimed at a NON-ADJACENT side rather than drawn from
    # the 34-66%-of-perimeter window. The window is only a proxy for "the
    # opposite side" and a poor one when the sides are unequal.
    "cut_across_share": 0.8,
    # -- sparseness ------------------------------------------------------
    # Share of block-sized faces left UNSUBDIVIDED and unbuilt: woodland, a
    # drainage reserve, a phase never built, land not taken up yet. They are
    # still emitted as blocks (so the ground is drawn) with `undeveloped` set.
    # Set to 0 for the wall-to-wall fabric this generator produced before.
    "undeveloped_share": 0.15,
    # Only a face already down to this multiple of target may be left: the loop
    # always holds the biggest face, so declining without a cap strands
    # whatever is huge at that moment.
    "undeveloped_max_factor": 1.5,
    # Where undeveloped land survives: within this of the crop edge, and beyond
    # this of the nearest collector. See `_undeveloped_weight`.
    "undeveloped_edge_m": 300.0,
    "undeveloped_collector_m": 260.0,
    "loop_attempts": 420,
    # DOUBLED WHEN CUL-DE-SACS STOPPED GOING INTO UNDEVELOPED LAND. This pass
    # is attempt-limited, not target-limited: it runs until `dead_end_target`
    # is met and never gets there (the measured share is ~14% against a 24%
    # target), so every candidate class it may no longer use comes straight off
    # the count. Refusing to dead-end in open land cost 5.4 cul-de-sacs a seed
    # and took the dead-end share to 11.2%, under the 12% floor of the measured
    # band; at 1400 it is back to 14.4% and 19.5 cul-de-sacs, i.e. the fabric
    # the band was fit to, with the stubs replatted onto land that has houses
    # on it. The attempts are cheap — 6 s across twelve seeds.
    # CUL-DE-SACS ARE BACK ON. They were off, and the recorded reason was that
    # the paved bulb is a disc the block polygon does not know about, so lots
    # were sited on it, fences ran out into it and driveways fanned round it;
    # `_arc_cap_bulbs` splicing the arc into the boundary fixed most of that,
    # and "most" on the feature that draws the eye was not good enough.
    #
    # WHAT WAS STILL WRONG WAS NOT THE ARC, IT WAS THE LOTS ON IT. Frontage
    # that turns 330 degrees in 100 m was being walked by the same rule as a
    # street, so each station got a RECTANGLE on its own tangent: consecutive
    # lots crossed each other near the kerb, the front lot line was a chord
    # that cut the paving, and a lot at the throat platted its back yard down
    # the stem. Three things fixed it, and they needed each other:
    #
    #   * `suburb_parcel` plats a head as N WEDGES with radial side lines
    #     (`_wedge_at`, `_probe_wedge`) — shared exactly between neighbours,
    #     house yawed to the centre, drive aimed at the paved radius, front
    #     fence following the arc rather than cutting across it.
    #   * heads are platted BEFORE the streets that reach them (`_lot_jobs`),
    #     and a street lot that would stand in a wedge's garden gives way.
    #     Platted in ring order the stem lots reach the throat first, and a
    #     stem lot is 30-44 m of frontage against a wedge's 25: the head then
    #     averaged two houses where it now takes four.
    #   * `_arc_cap_bulbs` no longer splices the same bulb twice. A dangling
    #     stem appears in the face cycle in both directions, and the second
    #     visit laid a second arc over the first — a boundary that still closed
    #     and still had the right area, so nothing caught it, but that ran
    #     round the turnaround and back.
    #
    # MEASURED ON THE SCENE `tools/fence_png.py` BUILDS — the real catalogue and
    # the real placements, not the nominal-footprint preview — over seeds 3, 5
    # and 8, 58 turnarounds:
    #
    #                        rectangular    wedges     no cul-de-sacs
    #   houses on the rings      107          230            —
    #   heads with none            3            0            —
    #   lot pairs overlapping    704          456          493
    #   houses on the tract     1708         1627         1531
    #
    # Nothing built, fenced, planted or paved stands on a bulb (0 of each,
    # tested against the 14.64 m PAVED radius, not the keep-out); every house
    # on a ring faces the turnaround centre to within a rounding error;
    # `tools/fence_check.py` is clean on its own seed set. The 81 houses the
    # wedges cost against rectangular platting are the stem lots that gave way
    # at the throats, and they buy the 248-pair fall in overlaps — a plat with
    # cul-de-sacs now overlaps LESS than one without them, on 6% more houses.
    # Dead-end share 13.4-15.4% against the 12-35% band.
    # Set false to go back down the ladder to a plat with no dead ends, which
    # is a real morphology — the gridiron and warped-parallel eras both have
    # one. Every downstream pass keys off the lollipop edges, so they simply
    # find none.
    "cul_de_sacs": True,
    "lollipop_attempts": 1400,
    "loop_share": 0.55,            # of successful locals, how many are loops
    "loop_depth_m": [90.0, 230.0],    # how far a loop bulges from its host
    "loop_min_depth_m": 70.0,         # below this the loop is not worth a block
    "loop_span_m": [200.0, 520.0],    # along-host distance between its two ends
    "lollipop_len_m": [55.0, 170.0],  # <= 750 ft (IFC cap on a dead end)
    "throat_m": 18.0,              # straight run leaving a junction
    # -- spacing ---------------------------------------------------------
    "min_gap_m": 88.0,             # centreline to centreline between streets
    "junction_spacing_m": 34.0,    # between any two junctions
    # Real plats meet at 70-110 degrees, not exactly 90. Allowing the swing both
    # matches them and lets the curve between two junctions stay drivable.
    "junction_skew_deg": 38.0,
    # Share of face splits that land on an existing junction, making it a
    # four-way. Suburbia is T-dominated but not T-exclusive.
    "four_way_chance": 0.35,
    # A new street must leave a junction at least this far from every arm
    # already there. Real plats do not fork at 13 degrees.
    "min_fork_deg": 38.0,
    # Cul-de-sacs are grown until this share of nodes are dead ends, which is
    # the measured signature of the fabric rather than a fixed count.
    "dead_end_target": 0.24,
    "cul_de_sac_gap_factor": 0.55,   # stub clearance, as a share of min_gap
    # Plat grain: how strongly streets prefer to run parallel to their
    # neighbours. 0 disables and gives an organic, un-suburban network.
    "grain_weight": 2.6,
    "grain_swirl_deg": 34.0,
    "grain_period_m": 620.0,
    "edge_clear_m": 26.0,          # keep off the region boundary
    # -- the park reserve ------------------------------------------------
    # A neighbourhood park is 12.6 ha against a 2.8 ha median block, so it is
    # RESERVED BEFORE THE FIRST STREET IS LAID and the network is generated
    # around it. See :class:`Reserve`.
    "park_enable": True,
    "park_size_m": [420.0, 300.0],   # matches suburb_park's own region
    "park_pad_m": 20.0,              # verge: streets stay out of this too
    # How far inside the crop the keep-out must sit, on top of `edge_clear_m`.
    # Not cosmetic: the reserve needs land on ALL FOUR sides for its frame
    # street to have blocks to face, and for entrances to exist on more than
    # the two sides that happen to point inward.
    "park_margin_m": 110.0,
    # The frame street around the reserve, offset from the keep-out by half its
    # own carriageway plus a little. WHY THERE IS A FRAME AT ALL: without one
    # the reserve is a hole inside some face, and a planar face traversal knows
    # nothing about holes — the block covering it would overlap the park, and
    # the subdivision pass would keep picking that face and failing to cut it.
    # A street on all four sides makes the face containing the park exactly the
    # frame's interior, which is then simply not emitted as a block. It is also
    # what a real park has: frontage, with houses facing it across the street.
    "park_ring_offset_m": 8.0,
    "park_ring_fillet_m": 32.0,      # corner radius; > the 30 m local minimum
    # Approach streets run from the frame outward to the first street they
    # meet: one per side, plus this many extra on random sides. Never fewer
    # than four, because a park with one way in is wrong — and because they are
    # also what connects the frame to the rest of the graph. The park usually
    # ends up with about twice this many entrances, since subdivision cuts land
    # on the frame too and each of those is a way in as well.
    "park_entrances": 5,
    "park_entry_min_m": 45.0,        # shorter than this is not a street
    # -- turnaround ------------------------------------------------------
    "bulb_radius_m": 14.64,        # IFC Fig D103.1: 96 ft driving diameter
    # -- curvature -------------------------------------------------------
    "sample_m": 6.0,
}


def _rng_pair(v, fallback):
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    if isinstance(v, (int, float)):
        return float(v), float(v)
    return fallback


def generate(width_m, height_m, rng, cfg=None):
    """Build a suburban network in a *width_m* x *height_m* region, centred.

    Returns ``(net, blocks, info)``.
    """
    c = dict(DEFAULTS)
    c.update(cfg or {})
    hw, hh = width_m / 2.0, height_m / 2.0
    region = (-hw, -hh, hw, hh)
    net = Network()

    min_gap = float(c["min_gap_m"])
    throat = float(c["throat_m"])
    sample = float(c["sample_m"])
    edge_clear = float(c["edge_clear_m"])
    jspace = float(c["junction_spacing_m"])

    # ---- level 0: the region boundary ---------------------------------
    # NOT A ROAD. The region is a CROP of a larger suburb, so ringing it with an
    # arterial drew a rectangular racetrack round the scene and made every
    # street stop at it — which reads as an island rather than a piece of a
    # town. These four edges exist only to close the planar graph so faces can
    # be extracted; they carry zero width, are never drawn, and blocks are not
    # inset from them. Streets simply run off the edge, which is what a crop of
    # a real suburb looks like.
    aw = 0.0
    ax0, ay0 = -hw, -hh
    ax1, ay1 = hw, hh
    corners = [(ax0, ay0), (ax1, ay0), (ax1, ay1), (ax0, ay1)]
    cids = [net.add_node(p) for p in corners]
    for i in range(4):
        net.add_edge([corners[i], corners[(i + 1) % 4]], "boundary", "boundary",
                     a=cids[i], b=cids[(i + 1) % 4])

    # ---- level 0.5: RESERVE THE PARK, AND FRAME IT --------------------
    # Chosen before any pavement exists, because it is far too big to fit into
    # anything the network would leave behind. Everything after this point
    # consults `reserve_blocks` and lays itself around the rectangle.
    #
    # THE FRAME GOES DOWN WITH THE RESERVE, BEFORE THE COLLECTORS. It was
    # originally laid after them, on the reasoning that its approach streets
    # need something to connect to — but the frame is laid unconditionally,
    # since the reserve cannot move, so anything already on that ground ends up
    # underneath it. Measured: a collector grazing the reserve's south-east
    # corner ended up 1.2 m from the frame's fillet, two carriageways on top of
    # each other, and the block inset off that collector clipped 3.7 m into the
    # keep-out. Laying the frame first makes it just another street the
    # collectors have to clear, which is the rule that already exists.
    reserve = _place_reserve(rng, region, c, edge_clear)
    ring_sid = None
    if reserve is not None:
        ring_sid = _lay_reserve_ring(net, reserve,
                                     float(c["park_ring_offset_m"]),
                                     float(c["park_ring_fillet_m"]))

    # ---- level 1: collectors ------------------------------------------
    # Curvilinear routes crossing the interior from one crop edge to the one
    # OPPOSITE it, running off the frame at both ends. Real collectors wander,
    # and they are the first thing that stops a suburb reading as a grid.
    #
    # WHY THE ROUTE IS A HARMONIC OFFSET AND NOT JITTERED WAYPOINTS. The first
    # attempt drew independent random offsets at 3-5 waypoints and interpolated;
    # measured over 36 trials that gave a median minimum radius of 17.5 m
    # against the 60 m a collector needs, because consecutive waypoints could
    # swing full amplitude in opposite directions. Summing a couple of raised
    # cosines instead bounds curvature by the harmonic that produced it, and
    # every term has ZERO SLOPE at f=0 and f=1 — so the route leaves and meets
    # its arterial exactly perpendicular, which is the whole junction contract.
    # The lateral shift between the two ends rides on a smoothstep for the same
    # reason: a linear term would tilt the approach.
    wander = float(c["collector_wander"])
    n_harm = int(c.get("collector_harmonics", 2))
    made_col = 0
    boundary_ids = {e.id for e in net.edges.values()
                    if e.road_class == "boundary"}
    # A route that would run through the reserve is thrown away whole rather
    # than bent around it — a collector is a straight-ish crossing of the
    # section by definition, and a version of it that detours round a 460 m
    # obstacle is a different street.
    #
    # RETRIES GO UP A LOT when a reserve exists, and this is worth more than it
    # looks. The park plus its frame plus the 88 m a street must keep from that
    # frame takes a 640 x 520 m bite out of the corridors a collector can use,
    # and at the original 25 tries the count fell from 2.4 to 1.9 per seed.
    # Collectors are where four-way junctions come from — they cross everything
    # in their path — so losing half of one pushed the three-way share from
    # 81.4% to 86.7% against a ceiling of 87%. The tries are cheap (a resample
    # and a curvature test reject most of them before any clearance work).
    col_tries = 25 if reserve is None else 140
    for _ in range(int(c["collectors"]) * col_tries):
        if made_col >= int(c["collectors"]):
            break
        vertical = rng.random() < 0.5
        span_lo, span_hi = ((ax0, ax1) if vertical else (ay0, ay1))
        run_lo, run_hi = ((ay0, ay1) if vertical else (ax0, ax1))
        pad = edge_clear + min_gap
        if span_hi - span_lo < 2.0 * pad:
            continue
        u0 = rng.uniform(span_lo + pad, span_hi - pad)
        u1 = rng.uniform(span_lo + pad, span_hi - pad)

        amp_cap = wander * (span_hi - span_lo)
        amps = [rng.uniform(-1.0, 1.0) * amp_cap / (k + 1)
                for k in range(n_harm)]

        n_s = max(24, int((run_hi - run_lo) / sample))
        pts = []
        for i in range(n_s + 1):
            f = i / n_s
            smooth = f * f * (3.0 - 2.0 * f)          # zero slope at both ends
            u = u0 + (u1 - u0) * smooth
            for k, a_k in enumerate(amps):
                u += a_k * (1.0 - math.cos(2.0 * math.pi * (k + 1) * f)) / 2.0
            u = max(span_lo + edge_clear, min(span_hi - edge_clear, u))
            v = run_lo + (run_hi - run_lo) * f
            pts.append((u, v) if vertical else (v, u))

        # BEFORE the resample, which is the expensive step here — it is
        # quadratic in the sample count and the reserve rejects a large share of
        # the corridors on its own. Resampling only moves points ALONG the
        # polyline, so a route that clears the rectangle now still clears it
        # after, and `_connect_route` checks again before anything is committed.
        if reserve_blocks(reserve, pts):
            continue
        pts = resample(pts, sample)

        if min_radius(pts) < CLASSES["collector"]["min_radius_m"]:
            continue
        if reserve_blocks(reserve, pts):
            continue
        # Ignore the crop boundary: the route starts and ends ON it.
        if net.clearance_allowing_crossings(
                pts, ignore=boundary_ids, limit=min_gap,
                own_hw=CLASSES["collector"]["width_m"] / 2.0) < min_gap:
            continue
        # Crossings are allowed to touch, but not at a glancing angle.
        if any(crossing_angle(pts, o.pts) < float(c["min_fork_deg"])
               for o in net.edges.values() if o.road_class != "boundary"):
            continue
        if _connect_route(net, pts, "collector", "collector", min_gap,
                          reserve=reserve):
            made_col += 1

    # ---- level 2: locals — loops and lollipops ------------------------
    loop_share = float(c["loop_share"])
    depth = _rng_pair(c["loop_depth_m"], (110.0, 240.0))
    span = _rng_pair(c["loop_span_m"], (150.0, 420.0))
    llen = _rng_pair(c["lollipop_len_m"], (70.0, 190.0))
    bulb_r = float(c["bulb_radius_m"])

    junctions = []       # points where something already joined a host

    def spaced_ok(p):
        return all(_dist(p, q) >= jspace for q in junctions)

    # ---- level 1.5: the streets that reach the park -------------------
    # AFTER the collectors and BEFORE anything else. After, because an approach
    # runs outward from the frame until it meets something, and the collectors
    # are what there is to meet. Before, because until one of them lands the
    # frame is a separate component of the graph: the reserve is then a HOLE
    # inside a face, a planar face traversal knows nothing about holes, and
    # every block cut from that face would be wrong.
    n_entry = 0
    if reserve is not None:
        n_entry = _link_reserve(net, reserve, rng, c, throat, min_gap, sample,
                                region, junctions, spaced_ok, ring_sid)

    # LOOPS FIRST, THEN LOLLIPOPS, and the order is not cosmetic. A loop needs
    # 150-420 m of continuous frontage on its host; a cul-de-sac needs a point.
    # Platting the cheap thing first fills the collector with junctions and the
    # expensive thing never fits — which is exactly what happened when the two
    # were interleaved (15 lollipops, 0 loops). Real subdivisions are platted in
    # this order for the same reason.
    # One plat orientation per suburb, with a slow swirl across it.
    grain_base = rng.uniform(0.0, math.pi)
    grain_swirl = math.radians(float(c["grain_swirl_deg"]))

    # LOOPS BEFORE SUBDIVISION. A loop needs 200-520 m of continuous frontage on
    # its host and a clear bulge of open land to swing into; face subdivision
    # needs neither. Running subdivision first therefore consumed every long
    # frontage and left loops with nowhere to attach — measured, 0 loops on
    # every seed, so half the "loops and lollipops" morphology was missing and
    # the fabric was really just connectors-and-cul-de-sacs. Platting them off
    # the collectors first, while the interior is still open, is also the order
    # a real subdivision is laid out in: the loop streets go in with the
    # collector, and the rest of the parcel is cut up around them.
    n_loops = 0
    for _ in range(int(c["loop_attempts"])):
        sids = net.street_ids("collector", "loop")
        if not sids:
            break
        if _grow_loop(net, sids[rng.randrange(len(sids))], rng, depth, span,
                      throat, min_gap, sample, region, edge_clear, spaced_ok,
                      junctions, min_depth=float(c["loop_min_depth_m"]),
                      reserve=reserve):
            n_loops += 1

    n_conn, undeveloped = _subdivide_faces(
        net, rng, c, throat, min_gap, sample, region, edge_clear, junctions,
        spaced_ok, grain_base=grain_base, grain_swirl=grain_swirl,
        reserve=reserve)
    und_pts = [u["p"] for u in undeveloped]
    und_polys = [u["poly"] for u in undeveloped]

    n_lolli = 0
    want_dead = float(c["dead_end_target"])
    for _ in range(int(c["lollipop_attempts"]) if c.get("cul_de_sacs") else 0):
        if dead_end_share(net, region) >= want_dead:
            break
        sids = net.street_ids("collector", "connector", "loop")
        if not sids:
            break
        if _grow_lollipop(net, sids[rng.randrange(len(sids))], rng, llen,
                          throat, min_gap, sample, region, edge_clear, bulb_r,
                          spaced_ok, junctions,
                          cul_gap_factor=float(c["cul_de_sac_gap_factor"]),
                          reserve=reserve, undeveloped=und_polys):
            n_lolli += 1

    # ---- level 5: the park absorbs the open land it touches -----------
    # LAST, because it is the only pass that takes something away, and what it
    # takes away depends on where the undeveloped parcels ended up — which is
    # not known until subdivision has run, and which cul-de-sacs poke where is
    # not known until the pass above has. See :func:`_merge_open_land`.
    merged_polys = []
    if reserve is not None:
        _dropped, merged_polys = _merge_open_land(net, reserve, ring_sid,
                                                  und_pts)

    face_list = faces(net)
    blks = blocks_from_faces(net, face_list, reserve=reserve,
                             undeveloped_pts=und_pts)
    info = {"collectors": made_col, "connectors": n_conn, "loops": n_loops,
            "lollipops": n_lolli, "region": region, "park": None,
            "reserve": None,
            # Every parcel subdivision left whole, as its CENTRELINE polygon —
            # including the ones that then merged into the park, which are no
            # longer blocks and so cannot be recovered from `blks`. This is the
            # extent of the open land, which is what a pass that wants to plant
            # or flood it needs; `undeveloped` below counts the ones that are
            # still blocks in their own right.
            "undeveloped_polys": und_polys,
            "undeveloped": sum(1 for b in blks if b["undeveloped"])}
    if reserve is not None:
        # READ OFF THE FINISHED GRAPH, not from the approach streets that were
        # laid for it. Face subdivision sites its cuts on face perimeters, and
        # the frame is on the perimeter of every face around the park, so it
        # collects junctions of its own as the fabric fills in. Those are
        # entrances too, and counting only the ones this module placed on
        # purpose would under-report the park's real frontage.
        ents = _reserve_entrances(net, reserve, ring_sid)
        info["park"] = {
            "rect": reserve.rect,          # the park itself, 420 x 300 m
            "size": reserve.size,
            "center": reserve.center,
            "pad_m": reserve.pad,
            "reserve": reserve.keep,       # park + padding: the street keep-out
            # THE PARK'S REAL EXTENT, and NOT a rectangle once open land has
            # merged into it: `rect` still says where the park's own content is
            # built, because it is the region `suburb_park` plans into and that
            # module wants a box; `poly` says how far the open space actually
            # runs. Equal to `rect` (bar corner rounding) when nothing merged.
            "poly": _park_polygon(net, reserve, ring_sid,
                                  float(c["park_ring_offset_m"]),
                                  float(c["park_ring_fillet_m"])),
            "merged": len(merged_polys),   # parcels absorbed
            "merged_polys": merged_polys,
            "ring_street_id": ring_sid,
            "approaches": n_entry,
            "entrances": ents,
            "sides": sorted({e["side"] for e in ents}),
        }
        info["reserve"] = reserve.keep
    return net, blks, info


def _connect_route(net, pts, road_class, street_type, min_gap, reserve=None):
    """Add a route, splitting every existing edge it crosses and splitting
    itself at the same points.

    This is what keeps the graph PLANAR. A collector that runs over another
    collector without a node there would leave a face that is not a block and a
    junction that is not a junction; downstream, the two roads would simply
    overlap. Crossings become real 4-way nodes here, once, at construction.

    Returns True when the route was added. The reserve is checked HERE as well
    as by the caller, because this is the one door every route goes through and
    a route that reaches it is about to become permanent geometry.
    """
    if reserve_blocks(reserve, pts):
        return False
    # Find intersections of `pts` with existing edges.
    cuts = []       # (arclength on pts, edge, arclength on that edge, point)
    cum = _cumulative(pts)
    for e in list(net.edges.values()):
        ecum = _cumulative(e.pts)
        for i in range(len(pts) - 1):
            for j in range(len(e.pts) - 1):
                hit = _seg_intersect(pts[i], pts[i + 1], e.pts[j], e.pts[j + 1])
                if hit is None:
                    continue
                p, ta, tb = hit
                sa = cum[i] + ta * _dist(pts[i], pts[i + 1])
                sb = ecum[j] + tb * _dist(e.pts[j], e.pts[j + 1])
                cuts.append((sa, e, sb, p))
    cuts.sort(key=lambda z: z[0])

    # Split hosts first; collect the node ids the route must pass through.
    waypoints = []
    for (sa, e, sb, p) in cuts:
        if e.id not in net.edges:              # already split by an earlier cut
            e = _edge_containing(net, p)
            if e is None:
                continue
            sb = _arclength_of(e.pts, p)
        nid = net.split_edge(e, sb)
        waypoints.append((sa, nid))

    # Then cut the route itself at those arclengths and add piece by piece.
    # ONE street id for every piece: the route is a single named street that
    # happens to have junctions on it, so `street_chain` can address it whole.
    # Letting each piece mint its own id fragmented a collector into three
    # short streets and left loops with nowhere long enough to attach.
    sid = net.new_street_id()
    prev_n = None
    remaining = list(pts)
    consumed = 0.0
    for (sa, nid) in waypoints:
        local = sa - consumed
        if local <= 1.0 or local >= polyline_length(remaining) - 1.0:
            continue
        head, remaining = split_polyline(remaining, local)
        net.add_edge(head, road_class, street_type, a=prev_n, b=nid,
                     street_id=sid)
        prev_n, consumed = nid, sa
    if polyline_length(remaining) > 1.0:
        net.add_edge(remaining, road_class, street_type, a=prev_n,
                     street_id=sid)
    return True


def _place_reserve(rng, region, c, edge_clear):
    """Choose the park rectangle, before there is anything to place it against.

    Uniform inside the box of centres that keeps the KEEP-OUT `park_margin_m`
    clear of the crop edge, so there is land for a street on all four sides.
    The margin is relaxed in two steps and then abandoned rather than failing
    hard, because a caller may hand this a region barely bigger than the park
    and a suburb with no park is a better answer than an exception.
    """
    if not c.get("park_enable", True):
        return None
    size = c.get("park_size_m") or [420.0, 300.0]
    pw, ph = float(size[0]), float(size[1])
    pad = float(c.get("park_pad_m", 20.0))
    margin = float(c.get("park_margin_m", 110.0))
    x0, y0, x1, y1 = region
    khw, khh = pw / 2.0 + pad, ph / 2.0 + pad
    for m in (margin, margin * 0.5, 0.0):
        lo_x, hi_x = x0 + edge_clear + m + khw, x1 - edge_clear - m - khw
        lo_y, hi_y = y0 + edge_clear + m + khh, y1 - edge_clear - m - khh
        if hi_x <= lo_x or hi_y <= lo_y:
            continue
        cx, cy = rng.uniform(lo_x, hi_x), rng.uniform(lo_y, hi_y)
        return Reserve((cx - pw / 2.0, cy - ph / 2.0,
                        cx + pw / 2.0, cy + ph / 2.0), pad)
    return None


def _arc_pts(c, r, a0, a1, n=6):
    return [(c[0] + r * math.cos(a0 + (a1 - a0) * k / n),
             c[1] + r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]


def _ring_sides(keep, off, fillet):
    """The four sides of the frame street, CCW from the bottom-left corner.

    Each side runs from the middle of one corner fillet to the middle of the
    next, so the four nodes of the ring sit ON the corners and a whole straight
    frontage is one edge — which is what lets face subdivision site a cut
    anywhere along it, and what makes the ring read as four streets meeting at
    four corners rather than as a polygon.
    """
    x0, y0, x1, y1 = keep
    r = max(0.0, min(fillet, (x1 - x0) / 2.0 - 1.0, (y1 - y0) / 2.0 - 1.0))
    # A ROUNDED CORNER CUTS THE CORNER OFF, and the corner it cuts off is the
    # reserve's. The midpoint of a fillet of radius *r* sits r*(1 - 1/sqrt2)
    # inside the rectangle's corner on each axis, so at the 8 m offset first
    # tried, a 32 m fillet put 1.4 m of carriageway centreline inside the
    # keep-out on all four corners. The offset is therefore never less than
    # that, which is the one place the frame's geometry has to know about the
    # thing it is framing.
    off = max(off, r * (1.0 - 1.0 / math.sqrt(2.0)) + 1.0)
    x0 -= off
    y0 -= off
    x1 += off
    y1 += off
    q = math.pi / 2.0
    if r < 1.0:                                    # degenerate: square corners
        bl, br, tr, tl = (x0, y0), (x1, y0), (x1, y1), (x0, y1)
        return [[bl, br], [br, tr], [tr, tl], [tl, bl]], (x0, y0, x1, y1), 0.0
    cbl, cbr = (x0 + r, y0 + r), (x1 - r, y0 + r)
    ctr, ctl = (x1 - r, y1 - r), (x0 + r, y1 - r)
    sides = [
        _arc_pts(cbl, r, 5 * q / 2.0, 3 * q) + _arc_pts(cbr, r, 3 * q, 7 * q / 2.0),
        _arc_pts(cbr, r, 7 * q / 2.0, 4 * q) + _arc_pts(ctr, r, 0.0, q / 2.0),
        _arc_pts(ctr, r, q / 2.0, q) + _arc_pts(ctl, r, q, 3 * q / 2.0),
        _arc_pts(ctl, r, 3 * q / 2.0, 2 * q) + _arc_pts(cbl, r, 2 * q, 5 * q / 2.0),
    ]
    return sides, (x0, y0, x1, y1), r


def _lay_reserve_ring(net, reserve, off, fillet):
    """The frame street around the reserve. Returns its street id.

    Four edges sharing one street id, so `street_chain` addresses the frame as
    one road the way it addresses a collector that has been split at its
    junctions. The corners are filleted at more than the 30 m local minimum
    radius: a park frontage is a street people drive, not a boundary line.
    """
    sides, _rect, _r = _ring_sides(reserve.keep, off, fillet)
    sid = net.new_street_id()
    first = net.add_node(sides[0][0])
    prev = first
    for i, pts in enumerate(sides):
        b = first if i == len(sides) - 1 else None
        e = net.add_edge(pts, "local", "park_ring", a=prev, b=b, street_id=sid)
        prev = e.b
    return sid


def _ray_hit(net, p, d, ignore, max_d):
    """First edge a ray from *p* along unit *d* meets: ``(dist, point, edge)``.

    Aiming at the FIRST thing hit is not an optimisation, it is what keeps the
    graph planar: a connector that skipped a nearer street to reach a better one
    would cross it without a node there.
    """
    q = _add(p, _mul(d, max_d))
    best = None
    for e in net.edges.values():
        if e.id in ignore:
            continue
        for j in range(len(e.pts) - 1):
            hit = _seg_intersect(p, q, e.pts[j], e.pts[j + 1])
            if hit is None:
                continue
            dist = hit[1] * max_d
            if dist < 1.0:
                continue
            if best is None or dist < best[0]:
                best = (dist, hit[0], e)
    return best


def _link_reserve(net, reserve, rng, c, throat, min_gap, sample, region,
                  junctions, spaced_ok, ring_sid, skew_deg=24.0):
    """Approach streets from the frame outward, one per side. Returns the count.

    THE PARK NEEDS SEVERAL WAYS IN, ON DIFFERENT SIDES. One entrance turns a
    12.6 ha park into an appendix of whichever street happens to touch it;
    every real neighbourhood park is entered from the streets around it.

    Each approach leaves the frame on the outward normal — the same junction
    contract every other street here is built to — and runs until it meets the
    FIRST existing street, which it joins as a proper split. That doubles as the
    graph repair the frame needs: until one of these lands, the ring is a
    separate component and the face containing the park is a hole nothing knows
    about.

    Constraints relax in tiers, junction spacing first and street spacing after,
    bottoming out at not-overlapping. A park with no way in at all is a worse
    outcome than a park approached down a street that sits closer to its
    neighbour than the plat would like, and the bottom tier is what guarantees
    the frame is connected — but it never buys that with two carriageways laid
    on top of each other.
    """
    max_d = _dist((region[0], region[1]), (region[2], region[3]))
    min_len = float(c.get("park_entry_min_m", 45.0))
    fillet = float(c["park_ring_fillet_m"])
    off = float(c["park_ring_offset_m"])
    _sides, rect, r = _ring_sides(reserve.keep, off, fillet)
    rx0, ry0, rx1, ry1 = rect
    # side -> (outward normal, point-on-frame as a function of t, t range)
    inset = r + 20.0
    lay = {
        "S": ((0.0, -1.0), lambda t: (t, ry0), (rx0 + inset, rx1 - inset)),
        "E": ((1.0, 0.0), lambda t: (rx1, t), (ry0 + inset, ry1 - inset)),
        "N": ((0.0, 1.0), lambda t: (t, ry1), (rx0 + inset, rx1 - inset)),
        "W": ((-1.0, 0.0), lambda t: (rx0, t), (ry0 + inset, ry1 - inset)),
    }
    order = ["S", "E", "N", "W"]
    rng.shuffle(order)
    want = max(4, int(c.get("park_entrances", 5)))
    while len(order) < want:
        order.append(order[rng.randrange(4)])

    made = 0
    for side in order:
        n_out, at, (t_lo, t_hi) = lay[side]
        if t_hi <= t_lo:
            continue
        if _link_one(net, reserve, rng, n_out, at, t_lo, t_hi, ring_sid,
                     throat, min_gap, sample, region, junctions, spaced_ok,
                     max_d, min_len, skew_deg, float(c["min_fork_deg"])):
            made += 1
    return made


def _link_one(net, reserve, rng, n_out, at, t_lo, t_hi, ring_sid, throat,
              min_gap, sample, region, junctions, spaced_ok, max_d, min_len,
              skew_deg, min_fork, tries=10):
    """One approach street off the frame. See :func:`_link_reserve`."""
    x0, y0, x1, y1 = region
    ring_ids = {e.id for e in net.edges.values() if e.street_id == ring_sid}
    # The last tier drops to NOT OVERLAPPING rather than to nothing. Asking for
    # no clearance at all would have let an approach be laid on top of a street
    # it runs beside, which is the one failure this generator spent two rounds
    # removing; a carriageway and a half apart is as far as the guarantee that
    # the frame gets connected is allowed to push.
    no_overlap = CLASSES["local"]["width_m"] + 6.0
    for gap in (min_gap, min_gap * 0.72, min_gap * 0.5, no_overlap):
        want_spacing = gap >= min_gap * 0.72
        for _try in range(tries):
            t = rng.uniform(t_lo, t_hi)
            p0 = at(t)
            if want_spacing and not spaced_ok(p0):
                continue
            ang = math.radians(rng.uniform(-skew_deg, skew_deg))
            ca, sa = math.cos(ang), math.sin(ang)
            aim = (n_out[0] * ca - n_out[1] * sa, n_out[0] * sa + n_out[1] * ca)
            hit = _ray_hit(net, p0, aim, ring_ids, max_d)
            if hit is None or hit[0] < min_len:
                continue
            _L, p1, host = hit
            if want_spacing and not spaced_ok(p1):
                continue
            t1 = tangent_at(host.pts, _arclength_of(host.pts, p1))
            n1 = _perp(t1)
            if _dot(n1, _sub(p0, p1)) < 0.0:
                n1 = _mul(n1, -1.0)
            chord = _unit(_sub(p1, p0))
            d0 = _rotate_toward(n_out, chord, skew_deg)
            d1 = _rotate_toward(n1, _mul(chord, -1.0), skew_deg)
            a = _add(p0, _mul(d0, throat))
            b = _add(p1, _mul(d1, throat))
            curved = resample([p0] + hermite(a, d0, b, _mul(d1, -1.0),
                                             tension=0.42, samples=20) + [p1],
                              sample)
            shapes = []
            if min_radius(curved) >= CLASSES["local"]["min_radius_m"]:
                shapes.append(curved)
            shapes.append(resample([p0, p1], sample))     # straight: no curvature
            ignore = set(ring_ids) | {host.id}
            for pnt in (p0, p1):
                nid = net.node_at(pnt, tol=1.5)
                if nid is not None:
                    ignore |= set(net.nodes[nid].edges)
            for pts in shapes:
                if reserve_blocks(reserve, pts):
                    _reject('park approach reserve')
                    continue
                if any(not (x0 - 0.5 <= q[0] <= x1 + 0.5
                            and y0 - 0.5 <= q[1] <= y1 + 0.5) for q in pts):
                    _reject('park approach leaves region')
                    continue
                if net.clearance(pts, ignore=ignore, skip_ends_m=throat * 0.6,
                                 limit=gap) < gap:
                    _reject('park approach clearance')
                    continue
                # PLANARITY IS NOT NEGOTIABLE even in the last tier: the whole
                # point of aiming at the first hit is that the street reaches it
                # without crossing anything, and the curved shape can bow off
                # the ray far enough to break that.
                if any(_routes_cross(pts, e.pts) for e in net.edges.values()
                       if e.id not in ignore):
                    _reject('park approach crosses')
                    continue
                nid1 = net.node_at(p1, tol=1.5)
                if nid1 is not None and not _fork_ok(net, nid1,
                                                     _sub(pts[-2], pts[-1]),
                                                     min_fork):
                    _reject('park approach shallow fork')
                    continue
                if _commit_cut(net, pts, p0, p1, junctions):
                    return True
    return False


def _park_face(net, reserve):
    """The face the reserve sits in, as ``(index, face)`` — None if there is none.

    One containment test on the park's centre, for the reason `covers` gives:
    no centreline enters the keep-out, so exactly one face holds it. After
    :func:`_merge_open_land` that face is bigger than the frame's interior,
    which is the point.
    """
    for i, f in enumerate(faces(net)):
        if point_in_polygon(f["poly"], reserve.center):
            return i, f
    return None, None


def _merge_open_land(net, reserve, ring_sid, undeveloped_pts):
    """Take the frame street DOWN where the park abuts land nobody built on.

    THE FRAME EXISTS TO GIVE FRONTAGE. Every reason it is laid — houses facing
    the park across a street, entrances where those streets meet it, a face
    traversal that can see the reserve at all — is a reason about DEVELOPED
    land. Where the neighbour is an undeveloped parcel there are no houses to
    front, no address to reach, and nothing on the far side that the street
    connects to; a carriageway there is a line drawn between two pieces of open
    ground that are otherwise the same ground. So it is not laid: park and
    parcel read as one continuous open space, which is what they are.

    Mechanically this MERGES TWO FACES, and that is the whole of the change to
    everything downstream. The park's face stops being the frame's interior and
    becomes the frame's interior plus the parcels that touch it, which
    `blocks_from_faces` already declines to emit as a block (it covers the
    reserve) and :func:`_park_polygon` publishes as the park's real extent —
    no longer a rectangle.

    Returns ``(removed_edge_ids, merged_polys)``.

    ONE CONTIGUOUS RUN PER PARCEL. A parcel that touched the frame in two
    separate runs would, if both came down, leave whatever sits between them
    ringed by open space — an island, i.e. a hole, and a planar face traversal
    cannot represent one: the merged face would come out with the island's land
    counted as park. Such a parcel is left framed rather than published wrong.
    It does not arise in the twelve-seed sweep (a parcel reaches two runs only
    by wrapping around a dead end poking out of the frame, and
    :func:`_dead_ends_in_open_land` is why there are none).
    """
    if ring_sid is None or not undeveloped_pts:
        return [], []
    fl = faces(net)
    park_i = None
    for i, f in enumerate(fl):
        if point_in_polygon(f["poly"], reserve.center):
            park_i = i
            break
    if park_i is None:
        return [], []
    und = {}
    for i, f in enumerate(fl):
        if i != park_i and any(point_in_polygon(f["poly"], q)
                               for q in undeveloped_pts):
            und[i] = f
    if not und:
        return [], []
    by_edge = {}
    for i, f in enumerate(fl):
        for eid in f["edges"]:
            by_edge.setdefault(eid, set()).add(i)
    # The park face's edge list is in cyclic order around the frame, so the
    # neighbour across each one, in that order, is what the run test needs.
    ring = list(fl[park_i]["edges"])
    nbr = []
    for eid in ring:
        e = net.edges[eid]
        others = [i for i in by_edge.get(eid, ()) if i != park_i]
        nbr.append(others[0] if (e.street_id == ring_sid and len(others) == 1
                                 and others[0] in und) else None)
    drop, merged = [], []
    n = len(nbr)
    for i in sorted(set(v for v in nbr if v is not None)):
        at = [k for k in range(n) if nbr[k] == i]
        runs = sum(1 for k in at if nbr[(k - 1) % n] != i)
        if runs != 1:
            continue
        drop.extend(ring[k] for k in at)
        merged.append(und[i]["poly"])
    for eid in drop:
        net.remove_edge(eid)
    if drop:
        drop.extend(_prune_open_space_stubs(net, reserve))
    # A frame corner whose two edges both came down is left holding nothing.
    # Drop it: a node with no edges is not a junction and not a dead end, and
    # every count over the graph would have to special-case it.
    for nid in [k for k, node in net.nodes.items() if not node.edges]:
        del net.nodes[nid]
    return drop, merged


def _prune_open_space_stubs(net, reserve):
    """Streets left dangling INSIDE the merged open space, taken back out.

    WHAT STRANDS THEM. Two undeveloped parcels either side of the same approach
    street both touch the frame, so both merge — and the street between them,
    which used to run from the fabric to the frame, now ends in the middle of
    one continuous piece of open ground. It fronts nothing on either side (that
    is what undeveloped means) and reaches nothing at its end (the frame it
    reached is gone). That is the same road-that-serves-nothing this whole pass
    is about, arrived at from the other direction, so it goes the same way.

    It also has to go for a mechanical reason. A dangling edge does not divide
    land, so the face traversal walks it out and back INSIDE one face; offset
    per edge, the two sides of that slit move APART rather than together and the
    published polygon folds over itself along the carriageway. Measured over
    twelve seeds before this: 7 seeds with road centrelines inside the park
    polygon and up to 8 self-intersections in it.

    Appearing TWICE in the face's edge cycle is exactly the definition of that
    street, so no geometry test is needed. It cascades, because taking one out
    can leave the one behind it dangling in turn.
    """
    dropped = []
    for _ in range(12):
        _i, f = _park_face(net, reserve)
        if f is None:
            break
        seen = {}
        for eid in f["edges"]:
            seen[eid] = seen.get(eid, 0) + 1
        stubs = [eid for eid, k in seen.items() if k > 1]
        if not stubs:
            break
        for eid in stubs:
            net.remove_edge(eid)
        dropped.extend(stubs)
    return dropped


def _park_polygon(net, reserve, ring_sid, off, fillet):
    """The park's real extent: the reserve plus whatever open land merged in.

    Built from the face the reserve sits in, pulled in per edge by what that
    edge IS — which is the same construction `blocks_from_faces` uses on a
    block, with one addition:

        frame street   pulled in to the PARK BOUNDARY, i.e. by the padding band
                       plus the frame's own offset. `rect` keeps its meaning
                       exactly: where the frame still stands, this polygon runs
                       along it
        other street   half a carriageway, like any block
        crop edge      nothing: it is not pavement, so open land runs to it

    So the polygon equals `rect` (bar 1.6 m of corner rounding off the frame's
    fillets) when nothing merged, and grows an arm over every parcel that did.

    THE CORNER WHERE THE FRAME STOPS NEEDS A CLAMP, and it is the one place
    this differs from a block. Every corner of a block joins two edges inset by
    about the same 5 m; here a frame edge inset by 30 m meets a street inset by
    5 m, and where those two meet at anything but a right angle their offset
    lines intersect a long way out — past `offset_polygon`'s mitre limit, which
    then cuts the corner square from a point that is also outside. Measured, it
    put a 30-50 m spike of "park" across the carriageway at every such corner,
    which is exactly the thing this module refuses to draw. A corner that falls
    OUTSIDE THE FACE cannot be right whatever produced it — an inset polygon
    lives inside the thing it was inset from — so it is replaced by the plain
    perpendicular offset of that vertex, which is where a right-angle corner
    would have put it anyway.
    """
    _i, f = _park_face(net, reserve)
    if f is None:
        return None
    _sides, rrect, _r = _ring_sides(reserve.keep, off, fillet)
    ring_inset = reserve.rect[0] - rrect[0]        # pad + effective offset
    insets = []
    for eid in f["edges"]:
        e = net.edges[eid]
        if e.street_id == ring_sid:
            d = ring_inset
        elif e.road_class == "boundary":
            d = 0.0
        else:
            d = e.half_w
        insets.extend([d] * (len(e.pts) - 1))
    poly = f["poly"]
    if len(insets) != len(poly):
        insets = (insets + [CLASSES["local"]["width_m"] / 2.0]
                  * max(len(poly) - len(insets), 0))[:len(poly)]
    out = offset_polygon(poly, insets)
    if len(out) != len(poly):
        return out if len(out) >= 3 else None
    n = len(poly)
    for i, v in enumerate(out):
        if point_in_polygon(poly, v):
            continue
        a, b = poly[i], poly[(i + 1) % n]
        d = _sub(b, a)
        if _norm(d) < _TOL:
            continue
        out[i] = _add(a, _mul(_perp(_unit(d)), insets[i]))
    return out if len(out) >= 3 else None


def _reserve_entrances(net, reserve, ring_sid, min_sep=25.0):
    """Where the neighbourhood reaches the park, read off the finished graph.

    An entrance is a junction ON THE FRAME where a street that is not the frame
    arrives — approaches placed on purpose and subdivision cuts that landed
    there alike. Each is reported as the point on the reserve boundary opposite
    it and the point a padding band further in on the park's own boundary, so
    the park can put its gate where the street already leads.
    """
    if ring_sid is None:
        return []
    ring_ids = {e.id for e in net.edges.values() if e.street_id == ring_sid}
    nids = set()
    for eid in ring_ids:
        e = net.edges[eid]
        nids.add(e.a)
        nids.add(e.b)
    out = []
    for nid in sorted(nids):
        node = net.nodes.get(nid)
        if node is None:
            continue
        if not any(net.edges[i].street_id != ring_sid
                   and net.edges[i].road_class != "boundary"
                   for i in node.edges):
            continue
        q, g, n, side = reserve.gate(node.p)
        if any(_dist(q, o["p"]) < min_sep for o in out):
            continue
        out.append({"p": q, "gate": g, "dir": n, "side": side, "node": nid,
                    "street_p": node.p})
    return out


def _crossing_points(pa, pb):
    """Every proper intersection point between two polylines."""
    out = []
    for i in range(len(pa) - 1):
        for j in range(len(pb) - 1):
            hit = _seg_intersect(pa[i], pa[i + 1], pb[j], pb[j + 1])
            if hit is not None:
                out.append(hit[0])
    return out


def _routes_cross(pa, pb):
    """True when two polylines properly intersect at least once."""
    for i in range(len(pa) - 1):
        for j in range(len(pb) - 1):
            if _seg_intersect(pa[i], pa[i + 1], pb[j], pb[j + 1]) is not None:
                return True
    return False


def crossing_angle(pa, pb):
    """Smallest angle (deg) at which two polylines cross, or 180 if they do not.

    A crossing is allowed to have zero clearance -- that is what makes a
    four-way -- but nothing was checking the ANGLE of it. Two collectors
    crossing at 25 degrees run nearly parallel through the junction and their
    carriageways overlap for tens of metres either side, which reads as one
    wide wavy road rather than as a crossroads. Measured: every shallow fork
    left after the snap and fallback rules were tightened was a
    collector/collector pair, all 12 of them.
    """
    best = 180.0
    for i in range(len(pa) - 1):
        da = _sub(pa[i + 1], pa[i])
        if _norm(da) < 1e-9:
            continue
        for j in range(len(pb) - 1):
            if _seg_intersect(pa[i], pa[i + 1], pb[j], pb[j + 1]) is None:
                continue
            db = _sub(pb[j + 1], pb[j])
            if _norm(db) < 1e-9:
                continue
            c = abs(_dot(_unit(da), _unit(db)))
            best = min(best, math.degrees(math.acos(max(-1.0, min(1.0, c)))))
    return best


def _seg_intersect(a0, a1, b0, b1):
    """Proper intersection point of two segments, or None."""
    d1, d2 = _sub(a1, a0), _sub(b1, b0)
    den = _cross(d1, d2)
    if abs(den) < 1e-12:
        return None
    r = _sub(b0, a0)
    t = _cross(r, d2) / den
    u = _cross(r, d1) / den
    if -1e-9 <= t <= 1 + 1e-9 and -1e-9 <= u <= 1 + 1e-9:
        return _add(a0, _mul(d1, t)), t, u
    return None


def _edge_containing(net, p, tol=0.75):
    for e in net.edges.values():
        for i in range(len(e.pts) - 1):
            if seg_seg_dist(p, p, e.pts[i], e.pts[i + 1]) <= tol:
                return e
    return None


def _arclength_of(pts, p):
    cum = _cumulative(pts)
    best, bs = float("inf"), 0.0
    for i in range(len(pts) - 1):
        d = seg_seg_dist(p, p, pts[i], pts[i + 1])
        if d < best:
            best = d
            seg = _sub(pts[i + 1], pts[i])
            L = _norm(seg)
            if L > _TOL:
                t = max(0.0, min(1.0, _dot(_sub(p, pts[i]), seg) / (L * L)))
            else:
                t = 0.0
            bs = cum[i] + t * L
    return bs


def _face_perimeter(net, face):
    """``(total, parts)`` where each part is ``(edge, flipped, s_start, length)``.

    Parameterises the face boundary by arclength so a split can be sited at
    "40% of the way round" and mapped back to the edge that must be cut.
    """
    parts, total = [], 0.0
    for (eid, frm) in face["hes"]:
        e = net.edges.get(eid)
        if e is None:
            return 0.0, []
        flipped = (e.a != frm)
        L = e.length
        parts.append((e, flipped, total, L))
        total += L
    return total, parts


def _at_perimeter(parts, u):
    """Map perimeter position *u* to ``(edge, edge_local_s, point, inward_n)``."""
    for (e, flipped, s0, L) in parts:
        if u <= s0 + L or (e, flipped, s0, L) is parts[-1]:
            local_or = max(0.0, min(L, u - s0))          # along travel direction
            local = (L - local_or) if flipped else local_or
            p = point_at(e.pts, local)
            t = tangent_at(e.pts, local)
            if flipped:
                t = _mul(t, -1.0)
            return e, local, p, _perp(t)                 # CCW face: left is in
    return None


def _face_corners(parts, corner_deg=40.0, merge_m=14.0):
    """Perimeter positions where the face boundary turns a CORNER.

    WHY A "SIDE" IS NOT AN EDGE. A cut from one side of a quadrilateral to the
    OPPOSITE side leaves two quadrilaterals; a cut to an ADJACENT side slices a
    corner off and leaves a triangle. To prefer the first the face has to know
    what its sides are — and `hes`/`parts` cannot answer that, because they give
    one entry per GRAPH EDGE. A curving street resampled every 6 m contributes
    dozens of polyline vertices and, after a few junctions have split it, several
    graph edges, all of which are one side of the block. Conversely one graph
    edge can wrap two sides of a face when it bends round a corner.

    So a side is defined GEOMETRICALLY, by where the turning is CONCENTRATED:
    a corner is a boundary vertex whose exterior angle is at least
    *corner_deg*, and the sides are the arcs between consecutive corners. That
    separates the two cases correctly by construction — a residential curve
    bottoms out at a 30 m radius and is sampled at 6 m, so it turns at most
    ~11.5 degrees per vertex however far it bends in total and stays ONE side,
    while a street meeting a host turns 52-128 degrees at a single vertex
    (`junction_skew_deg` caps the departure at 38 degrees off the normal) and
    always reads as a corner. Runs of sharp vertices within *merge_m* collapse
    to one corner, so a tight fillet is not counted as several.
    """
    samp = []
    for (e, flipped, s0, L) in parts:
        pts = list(reversed(e.pts)) if flipped else list(e.pts)
        acc = 0.0
        samp.append((s0, pts[0]))
        for k in range(1, len(pts) - 1):        # last point == next part's first
            acc += _dist(pts[k - 1], pts[k])
            samp.append((s0 + acc, pts[k]))
    n = len(samp)
    if n < 3:
        return []
    sharp = []
    for i in range(n):
        p_prev, p, p_next = samp[i - 1][1], samp[i][1], samp[(i + 1) % n][1]
        a, b = _sub(p, p_prev), _sub(p_next, p)
        if _norm(a) < 1e-6 or _norm(b) < 1e-6:
            continue
        turn = abs(math.degrees(math.atan2(_cross(a, b), _dot(a, b))))
        if turn >= corner_deg:
            sharp.append((samp[i][0], turn))
    if not sharp:
        return []
    # Collapse runs: keep the sharpest vertex of each cluster.
    out, run = [], [sharp[0]]
    for item in sharp[1:]:
        if item[0] - run[-1][0] <= merge_m:
            run.append(item)
        else:
            out.append(max(run, key=lambda t: t[1])[0])
            run = [item]
    out.append(max(run, key=lambda t: t[1])[0])
    return out


def _corners_between(corners, u_from, u_to, total, eps=3.0):
    """How many corners lie strictly inside the perimeter arc *u_from*->*u_to*.

    The separation of a cut's two ends, in corners. Zero means both ends are on
    the SAME side (a bite out of one edge); one means ADJACENT sides, which is
    the corner-slicing cut that makes a triangle; two or more means the ends
    face each other across the parcel, which is the cut a plat actually makes.

    Counted rather than indexed because a cut deliberately snapped to an
    existing junction lands exactly ON a corner, where a side index is
    ambiguous and an off-by-one silently reclassifies the candidate.
    """
    span = (u_to - u_from) % total
    if span <= 2.0 * eps:
        return 0
    return sum(1 for c in corners
               if eps < ((c - u_from) % total) < span - eps)


# Why proposed streets were refused, by reason. Read by the plan tool when a
# run comes out sparse; tuning the knobs blind is how the sliver-area bug
# survived two rounds of "just raise the attempt count".
REJECTS = {}


def _reject(why):
    REJECTS[why] = REJECTS.get(why, 0) + 1
    return False


def grain_dir(x, y, base_rad, swirl_rad, period_m):
    """Local plat direction at ``(x, y)`` — the way streets here want to run.

    WHY A SUBURB NEEDS A GRAIN. Choosing each cut only by length and balance
    gives every street its own heading, and the result reads as an organic
    network — an old town centre or a road system grown by accident — with
    triangular blocks and no repeating rhythm. Real post-war suburbia does not
    look like that, because it is platted subdivision by subdivision: within one
    plat the streets run roughly PARALLEL at a consistent spacing, curving
    together, which is what gives suburbia its combed look from the air.

    The field is smooth and low-frequency, so the grain drifts across the
    section the way adjoining plats do rather than switching abruptly. Streets
    are scored on how well they follow it, not forced onto it, so the fabric
    stays irregular where the land makes it irregular.
    """
    a = base_rad + swirl_rad * math.sin(x / period_m) * math.cos(y / period_m)
    return (math.cos(a), math.sin(a))


def _rotate_toward(v, target, max_deg):
    """*v* rotated toward *target* by at most *max_deg*."""
    ang = math.atan2(_cross(v, target), _dot(v, target))
    lim = math.radians(max_deg)
    ang = max(-lim, min(lim, ang))
    ca, sa = math.cos(ang), math.sin(ang)
    return (v[0] * ca - v[1] * sa, v[0] * sa + v[1] * ca)


def _perimeter_arc(parts, total, u_from, u_to, step=10.0):
    """Points along the face boundary walking forward from *u_from* to *u_to*.

    Wraps past the seam. Used to rebuild the two halves a proposed street would
    cut the face into, so their areas can be checked before anything is added.
    """
    span = (u_to - u_from) % total
    n = max(2, int(span / step))
    out = []
    for k in range(n + 1):
        u = (u_from + span * k / n) % total
        a = _at_perimeter(parts, u)
        if a is not None:
            out.append(a[2])
    return out


def _split_face(net, face, rng, throat, min_gap, sample, min_block, region,
                edge_clear, junctions, spaced_ok, skew_deg=38.0, tries=90,
                four_way_chance=0.0, grain_w=0.0, grain_base=0.0,
                grain_swirl=0.0, grain_period=600.0, allow_fallback=False,
                min_fork=38.0, reserve=None, shape_w=0.0, side_pen=0.0,
                corner_deg=40.0, across_share=0.8, shape_floor=0.0):
    """Insert one street across *face*, dividing it in two.

    RECURSIVE FACE SUBDIVISION IS WHAT FILLS THE LAND. Growing streets
    opportunistically off other streets — cast a ray, connect what it hits —
    only ever thickens fabric that already exists: it produced ladder rungs
    between the two collectors that happened to run parallel and left the rest
    of the section empty, because open ground has nothing to grow FROM. Splitting
    faces attacks the opposite way round, always subdividing the biggest
    remaining parcel, so coverage is guaranteed and block area converges on its
    target instead of depending on where the collectors happened to land.

    The street is still born the same way every other street here is: on a host
    centreline, leaving perpendicular, arriving perpendicular at the far side.

    `tries` IS 90, NOT THE 26 IT WAS, and the extra candidates are what the
    shape term spends. A cut is only kept if it leaves two well-shaped halves,
    so the sampler now has to FIND one rather than take the first legal thing —
    and the same face that yielded a workable cut in a handful of tries yields
    a workable well-shaped cut in far more. Measured over six seeds: at 34
    tries 31.8% of blocks came out triangular and 16 faces per seed had to fall
    back on a corner slice; at 90 the same seeds gave 23% and about 10. The
    cost is roughly 7 to 13 seconds a seed, which is nothing against a run that
    ends in USD.
    """
    total, parts = _face_perimeter(net, face)
    if total < 4.0 * throat or not parts:
        return False
    poly = face["poly"]

    # Perimeter positions of the face's existing junctions. Landing a split on
    # one of these turns a three-way into a FOUR-WAY, and without it every split
    # makes a T: measured three-way share sat at 96-98% against the 69-87% a
    # real suburb shows. Suburbia is T-dominated, not T-exclusive.
    node_us, acc = [], 0.0
    for (e, flipped, s0, L) in parts:
        nid = e.b if flipped else e.a
        if net.nodes[nid].road_degree(net) == 3:
            node_us.append(s0)
        acc = s0 + L

    # Decided ONCE for the whole split, not per candidate. Rolling it per try
    # let the best-of-N scoring quietly discard the four-way candidates — they
    # competed against non-snapped ones on cut length and almost never won, so
    # the four-way share stayed at noise however high the chance was set.
    want_snap = bool(node_us) and rng.random() < four_way_chance
    # Try the four-way first and fall back to a free cut, rather than failing
    # the face outright: a snapped split that finds no valid candidate used to
    # abandon the parcel altogether, which both lost the block and capped how
    # many four-ways could ever land.
    best = None
    for phase_snap in ((True, False) if want_snap else (False,)):
        best = _best_cut(net, face, rng, parts, total, poly, node_us,
                         phase_snap, tries, throat, min_gap, sample, min_block,
                         skew_deg, spaced_ok, min_fork=min_fork, grain_w=grain_w,
                         grain_base=grain_base, grain_swirl=grain_swirl,
                         grain_period=grain_period, reserve=reserve,
                         shape_w=shape_w, side_pen=side_pen,
                         corner_deg=corner_deg, across_share=across_share,
                         shape_floor=shape_floor)
        if best is not None:
            break
    if best is None and allow_fallback:
        best = _fallback_cut(net, face, rng, parts, total, poly, throat,
                             min_gap, sample, min_block, skew_deg, spaced_ok,
                             min_fork=min_fork, reserve=reserve,
                             shape_w=shape_w)
    if best is None:
        return False
    _score, pts, p0, p1 = best
    return _commit_cut(net, pts, p0, p1, junctions)


def _fallback_cut(net, face, rng, parts, total, poly, throat, min_gap, sample,
                  min_block, skew_deg, spaced_ok, tries=90, min_fork=38.0,
                  reserve=None, shape_w=0.0):
    """Last-resort cut for a face :func:`_best_cut` cannot divide.

    WHY A FACE GETS STUCK, MEASURED RATHER THAN GUESSED. `_best_cut` sites the
    far end of a cut PARAMETRICALLY, at 34-66% of the way round the perimeter,
    which is a good proxy for "the opposite side" only while the face is roughly
    convex. It stops being one as soon as a face is large and convoluted: on the
    worst measured face (462,837 m², 5,983 m of perimeter around an 1143 x 746 m
    L, 861 boundary vertices) half a perimeter walks you AROUND a corner instead
    of ACROSS the parcel, so the two inward normals did not face each other and
    the Hermite had to absorb the whole turn — 481 of 600 sampled candidates
    failed the 30 m radius check, 64 more failed clearance, and none survived.
    The face was marked stuck and shipped at 426,855 m², fourteen times the
    median block, rendering as one huge grass sheet.

    So this pass changes the two things that were actually wrong:

    * THE PAIRING IS GEOMETRIC, NOT PARAMETRIC. The far end is drawn from the
      whole perimeter and kept only when the chord genuinely crosses the parcel
      — within ~70 degrees of the inward normal at BOTH ends. That is the test
      the 34-66% window was standing in for.
    * BOTH SHAPES ARE TRIED, CURVE FIRST. Curvature was the single biggest
      rejection and a straight street has none, so the chord is tried whenever
      the curve is refused — including when the curve is refused for bulging out
      of the face or crowding a neighbour, which the straight cut between the
      same two points often does not do. Trying the curve and giving up on the
      pair was measured to throw away most of the workable cuts on the second
      stuck face: an exhaustive 12 m grid over its perimeter found 5,358 straight
      pairs at 63 m clearance or better, and the sampler was rejecting them.

    Constraints are then relaxed in tiers, junction spacing before street
    spacing, so a face is only ever cut tighter than `min_gap` when nothing
    roomier exists. On the same 113,720 m² face NO pair anywhere on the
    perimeter reaches the full 88 m gap — the parcel is a band, and that is why
    it was stuck — but thousands clear 63 m, which is where tier two lands.
    """
    tiers = ((1.0, True), (0.72, True), (0.72, False), (0.5, False))
    for (gap_factor, want_spacing) in tiers:
        gap = min_gap * gap_factor
        best = None
        for _try in range(tries):
            u0 = rng.uniform(0.0, total)
            u1 = (u0 + total * rng.uniform(0.08, 0.92)) % total
            a0 = _at_perimeter(parts, u0)
            a1 = _at_perimeter(parts, u1)
            if a0 is None or a1 is None:
                continue
            e0, _l0, p0, n0 = a0
            e1, _l1, p1, n1 = a1
            if _dist(p0, p1) < 2.0 * throat + 40.0:
                _reject('fallback ends too close')
                continue
            if want_spacing and not (spaced_ok(p0) and spaced_ok(p1)):
                _reject('fallback junction spacing')
                continue
            # Both ends must look INTO the parcel along the chord, or the cut
            # merely grazes the boundary and encloses nothing worth a block.
            chord = _unit(_sub(p1, p0))
            # Derived from min_fork, not a bare constant. At the original 0.35
            # a chord could sit 69.5 deg off the inward normal, i.e. leave its
            # host only 20.5 deg off-line -- which is a shallow fork, and the
            # fallback was quietly producing them faster than the snap check
            # removed them. Requiring the chord within (90 - min_fork) of the
            # normal makes the two rules agree.
            _fork_cos = math.cos(math.radians(max(0.0, 90.0 - min_fork)))
            if (_dot(chord, n0) < _fork_cos
                    or _dot(_mul(chord, -1.0), n1) < _fork_cos):
                _reject('fallback grazing')
                continue
            d0 = _rotate_toward(n0, chord, skew_deg)
            d1 = _rotate_toward(n1, _mul(chord, -1.0), skew_deg)
            a = _add(p0, _mul(d0, throat))
            b = _add(p1, _mul(d1, throat))
            curved = resample([p0] + hermite(a, d0, b, _mul(d1, -1.0),
                                             tension=0.42, samples=22) + [p1],
                              sample)
            shapes = [curved] if min_radius(curved) >= \
                CLASSES["local"]["min_radius_m"] else []
            shapes.append(resample([p0, p1], sample))   # straight: no curvature
            ignore = {e0.id, e1.id}
            for pnt in (p0, p1):
                nid = net.node_at(pnt, tol=1.5)
                if nid is not None:
                    ignore |= set(net.nodes[nid].edges)
            for pts in shapes:
                if any(not point_in_polygon(poly, q) for q in pts[2:-2]):
                    _reject('fallback leaves face')
                    continue
                if reserve_blocks(reserve, pts):
                    _reject('fallback reserve')
                    continue
                if net.clearance(pts, ignore=ignore, skip_ends_m=throat * 0.6,
                                 limit=gap) < gap:
                    _reject('fallback clearance')
                    continue
                side_a = _perimeter_arc(parts, total, u0, u1) + list(reversed(pts))
                side_b = _perimeter_arc(parts, total, u1, u0) + list(pts)
                area_a = abs(polygon_area(side_a))
                area_b = abs(polygon_area(side_b))
                if min(area_a, area_b) < min_block:
                    _reject('fallback sliver')
                    continue
                # Same objective as the main pass: the shortest cut that still
                # halves the parcel. No grain term — a face that reached here
                # has already refused every well-behaved candidate, and
                # preferring the plat direction over simply getting the land
                # divided is what left it undivided in the first place.
                score = polyline_length(pts) * (1.0 + 0.6 * abs(area_a - area_b)
                                                / max(area_a + area_b, 1.0))
                # Shape IS kept here, unlike grain. Grain is a preference about
                # which way the plat runs and a stuck face has already refused
                # every well-behaved candidate; shape is about whether what
                # comes out is a block at all, and it only reorders candidates
                # that already passed, so it cannot leave the face undivided.
                if shape_w > 0.0:
                    shape = min(rectangularity(side_a), rectangularity(side_b))
                    score *= (1.0 + shape_w * max(0.0, 1.0 - shape))
                if best is None or score < best[0]:
                    best = (score, pts, p0, p1)
                break
        if best is not None:
            return best
    return None


def _best_cut(net, face, rng, parts, total, poly, node_us, want_snap, tries,
              throat, min_gap, sample, min_block, skew_deg, spaced_ok,
              grain_w=0.0, grain_base=0.0, grain_swirl=0.0, grain_period=600.0,
              min_fork=38.0, reserve=None, shape_w=0.0, side_pen=0.0,
              corner_deg=40.0, across_share=0.8, shape_floor=0.0):
    """Lowest-scoring valid street across the face, or None."""
    # The driveable region: the face inset per edge by that edge's half width
    # plus half the width of the street being inserted.
    own_hw = CLASSES["local"]["width_m"] / 2.0
    insets = []
    for (e, _flip, _s0, _L) in parts:
        n_pts = max(1, len(e.pts) - 1)
        insets.extend([e.half_w + own_hw] * n_pts)
    if len(insets) != len(poly):
        insets = (insets + [own_hw * 2.0] * len(poly))[:len(poly)]
    drive_poly = offset_polygon(poly, insets)
    if len(drive_poly) < 3 or abs(polygon_area(drive_poly)) < 1.0:
        drive_poly = None

    # THE FACE'S SIDES, ONCE. Everything about block shape is decided against
    # these: candidates are aimed ACROSS the parcel rather than round it, and
    # the ones that still slice a corner are penalised in the score.
    corners = _face_corners(parts, corner_deg=corner_deg)
    sides = []
    for k in range(len(corners)):
        span = (corners[(k + 1) % len(corners)] - corners[k]) % total
        if span > 1.0:
            sides.append((corners[k], span))
    n_sides = len(sides)

    def _pick(weights):
        """Index chosen in proportion to *weights* — sides by their length."""
        tot = sum(weights)
        if tot <= 0.0:
            return rng.randrange(len(weights))
        r, acc = rng.uniform(0.0, tot), 0.0
        for k, w in enumerate(weights):
            acc += w
            if r <= acc:
                return k
        return len(weights) - 1

    def _in_side(k, f=None):
        """A position along side *k*, held off its two corners.

        *f* is the fraction of the way along the usable part of the side; None
        draws it uniformly.
        """
        lo, span = sides[k]
        m = min(0.3 * span, 22.0)
        if span < 2.0 * m + 1.0:
            return (lo + 0.5 * span) % total
        if f is None:
            return (lo + rng.uniform(m, span - m)) % total
        f = min(1.0, max(0.0, f))
        return (lo + m + f * (span - 2.0 * m)) % total

    def _side_of(u):
        for k, (lo, span) in enumerate(sides):
            if ((u - lo) % total) < span:
                return k
        return 0

    def _frac_in(k, u):
        """Where along side *k* position *u* sits, as `_in_side`'s fraction."""
        lo, span = sides[k]
        m = min(0.3 * span, 22.0)
        usable = span - 2.0 * m
        if usable <= 1.0:
            return 0.5
        return min(1.0, max(0.0, (((u - lo) % total) - m) / usable))

    def _far_side(i):
        """A side NON-ADJACENT to side *i*, weighted by length, or None."""
        far = [k for k in range(n_sides)
               if (k - i) % n_sides not in (0, 1, n_sides - 1)]
        if not far:
            return None
        return far[_pick([sides[k][1] for k in far])]

    def _opposite_f(f):
        """The far end's fraction, for a cut that runs ACROSS rather than skew.

        Sides run in opposite senses round a CCW boundary, so the point facing
        fraction *f* on one side is fraction ``1 - f`` on the side across from
        it — a cut between the two runs roughly parallel to the two sides it
        does NOT touch, which is the cut that leaves two rectangles. Jittered,
        not pinned: plats are not drafted on a grid, and the score is what
        finally chooses between the candidates this generates.
        """
        return 1.0 - f + rng.uniform(-0.22, 0.22)

    # A four-way snap lands the cut ON an existing junction, and a junction that
    # is itself a CORNER of this face can only ever slice that corner off. Most
    # are not — a T whose third arm points away leaves the boundary running
    # straight through it — so the corner ones are simply dropped rather than
    # the snap being given up on.
    if want_snap and node_us and corners:
        clear = [u for u in node_us
                 if min(min((u - c) % total, (c - u) % total)
                        for c in corners) > 14.0]
        if clear:
            node_us = clear

    best = [None, None]              # [aimed across, everything else]
    for _try in range(tries):
        u0 = u1 = None
        snapped = False
        if want_snap and node_us:
            u0 = node_us[rng.randrange(len(node_us))]
            snapped = True
            if n_sides >= 4 and rng.random() < across_share:
                i = _side_of(u0)
                j = _far_side(i)
                if j is not None:
                    u1 = _in_side(j, _opposite_f(_frac_in(i, u0)))
        elif n_sides >= 4 and rng.random() < across_share:
            # AIM THE CANDIDATE AT A NON-ADJACENT SIDE. Drawing the far end at
            # 34-66% of the way round the PERIMETER is only a proxy for "the
            # opposite side", and it is a bad one whenever the sides are
            # unequal: on a face with one long frontage and three short ones,
            # most of that window lands back on the long side or on a
            # neighbour, so the candidate pool was mostly corner-slicing cuts
            # and scoring could only pick the least bad of them. Sides are
            # weighted by length so a long frontage is still where most cuts
            # start, which is also true of real plats.
            i = _pick([s[1] for s in sides])
            j = _far_side(i)
            if j is not None:
                f = rng.random()
                u0 = _in_side(i, f)
                u1 = _in_side(j, _opposite_f(f))
        if u0 is None:
            u0 = rng.uniform(0.0, total)
        if u1 is None:
            u1 = (u0 + total * rng.uniform(0.34, 0.66)) % total
        a0 = _at_perimeter(parts, u0)
        a1 = _at_perimeter(parts, u1)
        if a0 is None or a1 is None:
            continue
        e0, l0, p0, n0 = a0
        e1, l1, p1, n1 = a1
        if _dist(p0, p1) < 2.0 * throat + 40.0:
            _reject('ends too close')
            continue
        # A DELIBERATE four-way snap sits exactly on an existing junction, so
        # the spacing rule must not be applied to it — it is zero from itself by
        # construction, which silently rejected every four-way candidate and
        # pinned the three-way share at 97%.
        if not ((snapped or spaced_ok(p0)) and spaced_ok(p1)):
            _reject('junction spacing')
            continue

        # DEPART NEAR-PERPENDICULAR, NOT EXACTLY PERPENDICULAR. Forcing a right
        # angle at both ends made the curve between them absorb the whole
        # direction change, and 171 of 489 candidates failed the 30 m radius
        # check because of it. Letting the departure swing up to `skew_deg`
        # toward the chord is also what real plats do — subdivision streets meet
        # at 70-110 degrees routinely — so this buys realism and feasibility at
        # once. The throat runs along the SAME direction, so there is no kink
        # where the straight stub meets the curve.
        chord = _unit(_sub(p1, p0))
        d0 = _rotate_toward(n0, chord, skew_deg)
        d1 = _rotate_toward(n1, _mul(chord, -1.0), skew_deg)
        a = _add(p0, _mul(d0, throat))
        b = _add(p1, _mul(d1, throat))
        pts = [p0] + hermite(a, d0, b, _mul(d1, -1.0), tension=0.42,
                             samples=22) + [p1]
        pts = resample(pts, sample)

        if min_radius(pts) < CLASSES["local"]["min_radius_m"]:
            _reject('curvature')
            continue
        # The reserve sits WITHIN a face until the frame street closes round it,
        # and a face that touches the park is otherwise a perfectly ordinary
        # parcel to cut — so containment against the face is not enough and the
        # keep-out has to be asked separately.
        if reserve_blocks(reserve, pts):
            _reject('reserve')
            continue
        # AGAINST THE INSET REGION, NOT THE RAW FACE. This is the structural
        # reason overlapping roads were ever proposed. A face's boundary runs
        # down its neighbours' CENTRELINES, so "inside the face" still allows a
        # new centreline to sit half a carriageway inside the road that bounds
        # it -- two streets laid on top of each other, entirely legally by this
        # test. Measured: 10 non-adjacent carriageway overlaps over 8 seeds.
        #
        # The region a new street may occupy is the face pulled in by half the
        # BOUNDARY road on each side plus half of its own width. Testing that
        # rejects the overlap at the point it is proposed, instead of leaving a
        # clearance rule to notice afterwards -- and it needs no exemptions,
        # because the endpoints are excluded by construction rather than by a
        # special case.
        # This ALSO subsumes the region check: every face lies inside the
        # arterial frame, so a street inside a face is inside the region. The
        # explicit region test was not merely redundant but wrong — a junction
        # sits ON the frame, 7 m from the boundary, so an `edge_clear` of 26 m
        # rejected every street that touched the arterial (100 candidates).
        if drive_poly and any(not point_in_polygon(drive_poly, q)
                              for q in pts[3:-3]):
            _reject('leaves face')
            continue
        if not drive_poly and any(not point_in_polygon(poly, q)
                                  for q in pts[2:-2]):
            _reject('leaves face')
            continue
        # Streets that meet at a junction are SUPPOSED to touch there, so every
        # edge incident to an endpoint is exempt — not just the one the endpoint
        # was located on. Exempting only the located edge meant a cut landing on
        # a degree-3 node read zero clearance against the other two arms and was
        # always rejected, which is why no four-way ever formed however the
        # chance was set (0 of 23 snapped attempts survived).
        ignore = {e0.id, e1.id}
        for pnt in (p0, p1):
            nid = net.node_at(pnt, tol=1.5)
            if nid is not None:
                ignore |= set(net.nodes[nid].edges)

        if net.clearance(pts, ignore=ignore, skip_ends_m=throat * 0.6,
                         limit=min_gap) < min_gap:
            _reject('clearance')
            continue
        # Refuse a cut that would leave a sliver on either side. Each half is
        # the face boundary from one junction round to the other, closed by the
        # new street — NOT the street polyline on its own, which encloses no
        # area at all and rejected every candidate when it was used by mistake.
        side_a = _perimeter_arc(parts, total, u0, u1) + list(reversed(pts))
        side_b = _perimeter_arc(parts, total, u1, u0) + list(pts)
        area_a, area_b = abs(polygon_area(side_a)), abs(polygon_area(side_b))
        if min(area_a, area_b) < min_block:
            _reject('sliver')
            continue

        # A cut that lands on an existing junction must not fork off an arm
        # already there at a shallow angle.
        # UNCONDITIONAL, not `if snapped`. The gate was pointless and harmful:
        # node_at() already returns None when the endpoint is mid-edge, so the
        # check is a natural no-op for a fresh cut. Gating it on the DELIBERATE
        # snap meant a cut that landed on an existing node BY ACCIDENT -- within
        # split_edge's 1 m tolerance, which happens whenever a host already
        # carries a junction near the chosen arclength -- was never fork-checked
        # at its near end. Measured over 24 seeds: 5 forks at 16-17 degrees and
        # 9 extra overlapping carriageways, all of them from this one gate.
        nid = net.node_at(p0, tol=1.5)
        if nid is not None and not _fork_ok(net, nid, _sub(pts[1], pts[0]),
                                            min_fork):
            _reject('shallow fork')
            continue
        nid1 = net.node_at(p1, tol=1.5)
        if nid1 is not None and not _fork_ok(net, nid1,
                                             _sub(pts[-2], pts[-1]), min_fork):
            _reject('shallow fork')
            continue

        # KEEP THE BEST CANDIDATE, NOT THE FIRST. Taking whatever passed first
        # produced long meandering cuts and therefore thin, high-perimeter
        # blocks — which is why street density ran 14-16 km/km² while block area
        # was already in band. The shortest cut that still balances the two
        # halves gives compact blocks, and compactness is exactly the ratio of
        # street length to land served.
        # Length and balance give compact blocks; the grain term is what makes
        # neighbouring streets run parallel instead of each going its own way.
        score = polyline_length(pts) * (1.0 + 0.6 * abs(area_a - area_b)
                                        / max(area_a + area_b, 1.0))
        if grain_w > 0.0:
            mid = point_at(pts, polyline_length(pts) / 2.0)
            g = grain_dir(mid[0], mid[1], grain_base, grain_swirl, grain_period)
            align = abs(_dot(_unit(_sub(p1, p0)), g))     # 1 = along the grain
            score *= (1.0 + grain_w * (1.0 - align))
        # BLOCK SHAPE. Length, balance and grain between them say nothing about
        # what SHAPE the two halves come out, and a cut from one side of a face
        # to the side next to it is often the shortest and best-balanced
        # candidate on offer — it is also the one that slices a corner off and
        # leaves a triangle. Measured on the generator before this term: 47.5%
        # of blocks scored under 0.62 on `rectangularity`, i.e. nearly half the
        # fabric was triangular, which is not what a residential plat looks
        # like on OSM anywhere.
        #
        # Two independent penalties, because they catch different failures. The
        # rectangularity of the WORSE half is the direct measure — 1.0 for a
        # rectangle whatever its aspect or orientation, exactly 0.5 for any
        # triangle — and it is the half that matters, since a cut that leaves
        # one good block and one wedge has still made a wedge. The corner
        # SEPARATION is topological and fires before the geometry goes bad:
        # ends on the same side (sep 0) or on adjacent sides (sep 1) cannot
        # produce two quadrilaterals however the areas happen to fall.
        good = True
        if shape_w > 0.0 or side_pen > 0.0:
            shape = min(rectangularity(side_a), rectangularity(side_b))
            sep = 2
            if corners:
                sep = min(_corners_between(corners, u0, u1, total),
                          _corners_between(corners, u1, u0, total))
            score *= (1.0 + shape_w * max(0.0, 1.0 - shape))
            if sep < 2:
                score *= (1.0 + side_pen * (2 - sep))
            # A FLOOR, NOT ONLY A WEIGHT. Scoring alone still takes the best of
            # a bad pool: on a face where every sampled candidate slices a
            # corner, the multiplier is a constant and the winner is whichever
            # corner-slice happened to be shortest. Splitting the pool in two
            # and preferring the well-shaped bucket means a triangular cut is
            # made only when nothing across the parcel is legal at all — which
            # does happen, and is why the other bucket is kept rather than the
            # candidate being rejected outright.
            good = (sep >= 2 and shape >= shape_floor)
        slot = 0 if good else 1
        if best[slot] is None or score < best[slot][0]:
            best[slot] = (score, pts, p0, p1)
    if best[0] is None and best[1] is not None:
        _reject('shape: only a corner slice was legal')
    return best[0] or best[1]


def _fork_ok(net, node_id, direction, min_deg):
    """True when *direction* leaves *node_id* far enough from every existing arm.

    WHY THIS IS NEEDED. Clearance exempts edges incident to a junction, because
    streets meeting at a junction are supposed to touch — without that exemption
    no four-way could ever form. But the exemption is unbounded, so nothing
    stopped a new street leaving an existing node at 13 degrees from an arm
    already there and running alongside it. Measured on the shipped preset: 6
    pairs of DIFFERENT streets forking under 30 degrees, their carriageways
    overlapping for 20-44 m past the node — which reads as one wide wavy road,
    or as two roads laid on top of each other.

    A real plat does not fork at 13 degrees. Junction geometry, not clearance,
    is the right place to rule it out.
    """
    node = net.nodes.get(node_id)
    if node is None:
        return True
    for eid in node.edges:
        e = net.edges.get(eid)
        if e is None or e.road_class == "boundary":
            continue
        pts = e.pts if e.a == node_id else list(reversed(e.pts))
        arm = None
        for k in range(1, len(pts)):
            d = _sub(pts[k], pts[0])
            if _norm(d) > 1e-6:
                arm = _unit(d)
                break
        if arm is None:
            continue
        cosang = max(-1.0, min(1.0, _dot(_unit(direction), arm)))
        if math.degrees(math.acos(cosang)) < min_deg:
            return False
    return True


def _commit_cut(net, pts, p0, p1, junctions):
    """Split both hosts and add the street between the two new nodes."""
    # Split the far host first, then re-find the near one by point: the two
    # may be the SAME edge, in which case cutting it renumbers everything.
    t1 = _edge_containing(net, p1)
    if t1 is None:
        return False
    n_b = net.split_edge(t1, _arclength_of(t1.pts, p1))
    t0 = _edge_containing(net, p0)
    if t0 is None:
        return False
    n_a = net.split_edge(t0, _arclength_of(t0.pts, p0))
    if n_a == n_b:
        return False
    net.add_edge(pts, "local", "connector", a=n_a, b=n_b)
    junctions.extend([p0, p1])
    return True


def _undeveloped_weight(poly, region, cols, edge_m, col_m):
    """0-1: how likely this parcel is to be land nobody has built on yet.

    WHERE UNDEVELOPED LAND ACTUALLY SURVIVES, which is not uniformly at random.
    On OSM, the parcels still carrying woodland, a drainage reserve or bare
    field in an otherwise built-out subdivision are the ones at the OUTER EDGE
    of the platted area — the phase that was never built, or the strip left
    against whatever the subdivision backs onto — and the ones FURTHEST FROM
    THE COLLECTOR, because frontage on the road everybody drives down is the
    first land to be taken up and the last to be left over. Land wrapped in
    houses on all four sides, next to the through route, is built.

    So the two terms are distance to the crop edge and distance to the nearest
    collector, each saturating at its own scale, multiplied rather than added:
    a parcel has to be BOTH peripheral AND off the main road to score high,
    which is the combination the aerials actually show.
    """
    c = polygon_centroid(poly)
    x0, y0, x1, y1 = region
    d_edge = min(c[0] - x0, x1 - c[0], c[1] - y0, y1 - c[1])
    edge_t = 1.0 - min(1.0, max(0.0, d_edge) / max(edge_m, 1.0))
    if cols:
        d_col = min(polyline_dist([c, c], p) for p in cols)
        col_t = min(1.0, d_col / max(col_m, 1.0))
    else:
        col_t = 1.0
    return (0.25 + 0.75 * edge_t) * (0.25 + 0.75 * col_t)


# What `undeveloped_share` has to be multiplied by so the share of blocks that
# come out undeveloped MATCHES it. Two corrections, both measured rather than
# guessed: a face is offered exactly once, on the iteration where it is the
# biggest thing left and has already come down to block size, and only about
# two fifths of the finished parcels ever pass through that window (the rest
# are the second half of a cut and drop below target without being offered);
# and `_undeveloped_weight` averages about 0.32 across a region, since it is
# deliberately near zero in the built-out middle. `stats` reports the share
# that actually came out, so this staying honest is checkable.
_UNDEVELOPED_GAIN = 7.0


def _subdivide_faces(net, rng, cfg, throat, min_gap, sample, region,
                     edge_clear, junctions, spaced_ok, grain_base=0.0,
                     grain_swirl=0.0, reserve=None):
    """Keep splitting the largest face until every block is near target size.

    Returns ``(cuts_made, undeveloped)``, a list of ``{"p", "poly"}`` — one
    entry per face deliberately LEFT ALONE, see `undeveloped_share`. The point
    is what marks the parcel (see below); the polygon is kept alongside it
    because two later passes need the parcel's EXTENT rather than a point in it:
    cul-de-sacs must not dead-end inside one, and one that touches the park is
    merged into it.
    """
    target = float(cfg["block_area_target_m2"])
    min_block = float(cfg["block_area_min_m2"])
    hard_max = target * float(cfg.get("block_area_hard_max_factor", 1.5))
    # SPARSENESS. A suburb is not a wall-to-wall tiling of blocks: some parcels
    # are woodland, a drainage reserve, a phase that was never built, or simply
    # land not taken up yet. Leaving a share of faces uncut is what puts that
    # back — and it has to happen HERE, inside the loop that would otherwise
    # keep picking the largest face, rather than as a post-pass, because a face
    # only stays whole if nothing ever cuts it.
    und_share = float(cfg.get("undeveloped_share", 0.0))
    und_cap = target * float(cfg.get("undeveloped_max_factor", 1.25))
    und_edge = float(cfg.get("undeveloped_edge_m", 300.0))
    und_col = float(cfg.get("undeveloped_collector_m", 260.0))
    cols = [e.pts for e in net.edges.values() if e.road_class == "collector"]
    left = []                # {"p", "poly"} per face left undeveloped
    made, stuck = 0, set()
    for _ in range(int(cfg["subdivide_iters"])):
        face_list = faces(net)
        # The face inside the park frame is the reserve, not undivided land. It
        # is the biggest face on the board and would be picked first on every
        # iteration, so it is dropped here rather than left to fail its way into
        # `stuck` — and dropped rather than cut, which is the whole point.
        big = [f for f in face_list
               if abs(polygon_area(f["poly"])) > target
               and tuple(sorted(f["edges"])) not in stuck
               and not any(point_in_polygon(f["poly"], u["p"]) for u in left)
               and not (reserve is not None and reserve.covers(f["poly"]))]
        if not big:
            break
        big.sort(key=lambda f: -abs(polygon_area(f["poly"])))
        f = big[0]
        # A face past `hard_max` is not a block, it is undivided land: the worst
        # measured one shipped at fourteen times the median and rendered as a
        # single grass sheet. Those are allowed the relaxed fallback cut;
        # everything else still either takes a well formed street or is left
        # alone, which is what keeps the fabric's measured morphology.
        area = abs(polygon_area(f["poly"]))
        # A parcel is only left undeveloped while it is still BLOCK-SIZED. The
        # loop always holds the biggest face on the board, so declining to cut
        # without this cap would strand whatever happened to be huge at that
        # moment — which is the same failure `block_area_hard_max_factor`
        # exists to stop, and would blow the "no face over 3x the median"
        # invariant on its own.
        if und_share > 0.0 and area <= und_cap:
            w = _undeveloped_weight(f["poly"], region, cols, und_edge, und_col)
            if rng.random() < und_share * _UNDEVELOPED_GAIN * w:
                # Marked by an interior POINT, not by its edge ids: a cut in the
                # neighbouring face lands on this face's boundary and splits
                # those edges, so an id-based key would silently expire while
                # the parcel itself never moved. The POLYGON is safe to keep for
                # the same reason the point is: nothing ever cuts this face
                # again (the loop skips any face holding a `left` point), so the
                # only thing later passes can do to it is add vertices along its
                # boundary, which does not move the boundary.
                left.append({"p": polygon_centroid(f["poly"]),
                             "poly": list(f["poly"])})
                continue
        if _split_face(net, f, rng, throat, min_gap, sample, min_block, region,
                       edge_clear, junctions, spaced_ok,
                       skew_deg=float(cfg["junction_skew_deg"]),
                       four_way_chance=float(cfg["four_way_chance"]),
                       min_fork=float(cfg["min_fork_deg"]),
                       grain_w=float(cfg["grain_weight"]),
                       grain_base=grain_base, grain_swirl=grain_swirl,
                       grain_period=float(cfg["grain_period_m"]),
                       shape_w=float(cfg.get("block_shape_weight", 0.0)),
                       side_pen=float(cfg.get("block_side_penalty", 0.0)),
                       corner_deg=float(cfg.get("face_corner_deg", 40.0)),
                       across_share=float(cfg.get("cut_across_share", 0.8)),
                       shape_floor=float(cfg.get("block_shape_floor", 0.0)),
                       allow_fallback=area > hard_max, reserve=reserve):
            made += 1
        else:
            stuck.add(tuple(sorted(f["edges"])))
    return made, left


def _dead_ends_in_open_land(polys, pts, tip, bulb_r):
    """True when this dead end would come to rest on land nobody built on.

    A CUL-DE-SAC EXISTS TO SERVE LOTS. Its whole justification is frontage: the
    stem gives four to eight houses an address and the bulb turns the fire
    appliance round at the end of it. An undeveloped parcel has no houses on it
    by definition, so a stub poking into one serves nothing, ends nowhere, and
    reads as a road the plat forgot to finish — which is exactly what it is.

    A road CROSSING that land is a different thing and stays allowed: a street
    that runs through the parcel to reach the fabric on the other side, or to
    run off the crop edge, is doing the one job a road does. So the test is on
    the stem and the turnaround only, and it is asked in `_grow_lollipop`, which
    is the only pass that builds a dead end on purpose.

    The bulb is a 14.6 m disc, not a point, so it is tested as one — the same
    reason `reserve_blocks` is asked with `bulb_r` a few lines below.
    """
    if not polys:
        return False
    for poly in polys:
        x0, y0, x1, y1 = bbox(poly)
        if (tip[0] < x0 - bulb_r or tip[0] > x1 + bulb_r
                or tip[1] < y0 - bulb_r or tip[1] > y1 + bulb_r):
            # the bulb is clear of this parcel; the stem still might not be
            if all(q[0] < x0 or q[0] > x1 or q[1] < y0 or q[1] > y1
                   for q in pts):
                continue
        if point_in_polygon(poly, tip):
            return True
        ring = list(poly) + [poly[0]]
        if polyline_dist([tip, tip], ring, limit=bulb_r) < bulb_r:
            return True
        # The stem's first point sits ON its host, which is often this parcel's
        # own boundary, so a containment test there is a coin toss — start at
        # the second sample, where the street has actually committed to a side.
        if any(point_in_polygon(poly, q) for q in pts[1:]):
            return True
    return False


def _grow_lollipop(net, sid, rng, llen, throat, min_gap, sample, region,
                   edge_clear, bulb_r, spaced_ok, junctions,
                   cul_gap_factor=0.72, reserve=None, undeveloped=()):
    """A dead-end street off street *sid*, ending in a turnaround."""
    pts_h, _chain = net.street_chain(sid)
    if not pts_h:
        return False
    L = polyline_length(pts_h)
    if L < 2.0 * throat + 20.0:
        return False
    s = rng.uniform(throat, L - throat)
    loc = net.locate_on_street(sid, s)
    if loc is None:
        return False
    host, local_s, p0, tan = loc
    n0 = _perp(tan)
    if rng.random() < 0.5:
        n0 = _mul(n0, -1.0)
    if not spaced_ok(p0):
        return False
    length = rng.uniform(*llen)

    # Straight throat, then a gentle bend so it does not read as a spur.
    a = _add(p0, _mul(n0, throat))
    bend = rng.uniform(-0.55, 0.55)
    t1 = _unit((n0[0] * math.cos(bend) - n0[1] * math.sin(bend),
                n0[0] * math.sin(bend) + n0[1] * math.cos(bend)))
    tip = _add(a, _mul(t1, max(20.0, length - throat)))
    pts = [p0] + hermite(a, n0, tip, t1, tension=0.4, samples=16)
    pts = resample(pts, sample)

    if min_radius(pts) < CLASSES["cul_de_sac"]["min_radius_m"]:
        return False
    # The stem staying out of the reserve is not enough — the turnaround is a
    # 14.6 m disc, so it is asked for with that much clearance.
    if reserve_blocks(reserve, pts) or reserve_blocks(reserve, [tip], bulb_r):
        return False
    # NOT INTO UNDEVELOPED LAND. See :func:`_dead_ends_in_open_land`.
    if _dead_ends_in_open_land(undeveloped, pts, tip, bulb_r):
        return False
    # The bulb needs its own room, so clear a bulb radius around the tip too.
    x0, y0, x1, y1 = region
    if not (x0 + edge_clear + bulb_r <= tip[0] <= x1 - edge_clear - bulb_r
            and y0 + edge_clear + bulb_r <= tip[1] <= y1 - edge_clear - bulb_r):
        return False
    host_ids = {e.id for e in net.edges.values() if e.street_id == sid}
    stub_gap = min_gap * cul_gap_factor
    if net.clearance(pts, ignore=host_ids, skip_ends_m=throat * 0.6,
                     limit=stub_gap) < stub_gap:
        return False
    if net.clearance([tip, tip], ignore=host_ids,
                     limit=stub_gap + bulb_r) < stub_gap + bulb_r:
        return False

    mid = net.split_edge(host, local_s)
    net.add_edge(pts, "cul_de_sac", "lollipop", a=mid)
    junctions.append(p0)
    return True


def _grow_loop(net, sid, rng, depth, span, throat, min_gap, sample, region,
               edge_clear, spaced_ok, junctions, min_depth=70.0, reserve=None):
    """A street leaving street *sid* and returning to it, enclosing a block.

    Both ends are solved as boundary conditions — leave perpendicular, arrive
    perpendicular — which is exactly what the old envelope approach could not
    express and is why its junctions never looked right.
    """
    pts_h, _chain = net.street_chain(sid)
    if not pts_h:
        return False
    L = polyline_length(pts_h)
    d = rng.uniform(*span)
    if L < d + 2.0 * throat + 20.0:
        return False
    s0 = rng.uniform(throat, L - d - throat)
    s1 = s0 + d
    loc0 = net.locate_on_street(sid, s0)
    loc1 = net.locate_on_street(sid, s1)
    if loc0 is None or loc1 is None:
        return False
    side = 1 if rng.random() < 0.5 else -1
    p0, n0 = loc0[2], _mul(_perp(loc0[3]), side)
    p1, n1 = loc1[2], _mul(_perp(loc1[3]), side)
    if not (spaced_ok(p0) and spaced_ok(p1)):
        return False

    dep = rng.uniform(*depth)
    # THE SHOULDER NEEDS ROOM TO TURN. Leaving the host perpendicular and then
    # running parallel to it is a 90 degree turn, and a 90 degree turn of radius
    # R needs about R of travel in BOTH axes. Fixing the apex at a fraction of
    # the span instead let the loop bulge 240 m out while advancing only 42 m
    # along, compressing the shoulder to a ~15 m radius — which is why all 419
    # loop attempts failed the curvature check and the fabric came out as a
    # pure dendritic tree. So the turn run is derived from the depth, and the
    # depth is capped by the span that actually exists to turn through it.
    turn = dep * 0.95
    if d < 2.0 * turn + 50.0:
        dep = (d - 50.0) / (2.0 * 0.95)
        if dep < float(min_depth):
            return False
        turn = dep * 0.95
    a = _add(p0, _mul(n0, throat))
    b = _add(p1, _mul(n1, throat))
    # Two apex controls give the flat-topped sweep a real loop street has,
    # rather than a single circular bow.
    mid_t = _unit(_sub(p1, p0))
    apex0 = _add(_add(p0, _mul(mid_t, turn)), _mul(n0, dep))
    apex1 = _add(_add(p0, _mul(mid_t, d - turn)), _mul(n1, dep))
    pts = [p0]
    pts.extend(hermite(a, n0, apex0, mid_t, tension=0.5, samples=14)[1:])
    pts.extend(hermite(apex0, mid_t, apex1, mid_t, tension=0.35, samples=10)[1:])
    pts.extend(hermite(apex1, mid_t, b, _mul(n1, -1.0), tension=0.5,
                       samples=14)[1:])
    pts.append(p1)
    pts = resample(pts, sample)

    if min_radius(pts) < CLASSES["local"]["min_radius_m"]:
        return False
    if reserve_blocks(reserve, pts):
        return False
    x0, y0, x1, y1 = region
    if any(not (x0 + edge_clear <= q[0] <= x1 - edge_clear
                and y0 + edge_clear <= q[1] <= y1 - edge_clear) for q in pts):
        return False
    host_ids = {e.id for e in net.edges.values() if e.street_id == sid}
    if net.clearance(pts, ignore=host_ids, skip_ends_m=throat * 0.6, limit=min_gap) < min_gap:
        return False

    # Split at the FAR end first. Splitting at s0 replaces the edge that s1 was
    # located on, so the second lookup has to be redone against the new graph —
    # doing the far end first means only ONE lookup is ever invalidated, and
    # `locate_on_street` re-resolves it from the chain rather than an edge id.
    n_b = net.split_edge(loc1[0], loc1[1])
    loc0b = net.locate_on_street(sid, s0)
    if loc0b is None or _dist(loc0b[2], p0) > 2.0:
        return False
    n_a = net.split_edge(loc0b[0], loc0b[1])
    net.add_edge(pts, "local", "loop", a=n_a, b=n_b)
    junctions.extend([p0, p1])
    return True


# ---------------------------------------------------------------------------
# measurement — judged against OSM, not by eye
# ---------------------------------------------------------------------------

def dead_end_share(net, region):
    """Fraction of street nodes that are genuine dead ends.

    ONE definition, used both by :func:`stats` and by the cul-de-sac loop that
    grows toward a target. A node where a street runs off the crop edge has one
    incident street and is not a cul-de-sac — the street continues outside the
    region. Counting those made the growth loop believe the target was already
    met before a single cul-de-sac had been placed, and it exited immediately.
    """
    x0, y0, x1, y1 = region

    def at_crop(n):
        return (abs(n.p[0] - x0) < 1.0 or abs(n.p[0] - x1) < 1.0
                or abs(n.p[1] - y0) < 1.0 or abs(n.p[1] - y1) < 1.0)

    real = [n for n in net.nodes.values() if n.road_degree(net) > 0]
    if not real:
        return 0.0
    dead = sum(1 for n in real if n.road_degree(net) == 1 and not at_crop(n))
    return dead / len(real)


def stats(net, blks, region):
    """Morphology numbers with their measured suburban targets alongside."""
    x0, y0, x1, y1 = region
    km2 = ((x1 - x0) / 1000.0) * ((y1 - y0) / 1000.0)
    # A node where a street runs off the crop is NOT a dead end -- the street
    # continues outside the region. Counting those as cul-de-sacs would inflate
    # the signature stat of the whole fabric.
    on_edge = set()
    for e in net.edges.values():
        if e.road_class == "boundary":
            on_edge.add(e.a)
            on_edge.add(e.b)
    bx0, by0, bx1, by1 = region

    def _at_crop(n):
        return (abs(n.p[0] - bx0) < 1.0 or abs(n.p[0] - bx1) < 1.0
                or abs(n.p[1] - by0) < 1.0 or abs(n.p[1] - by1) < 1.0)

    real = [n for n in net.nodes.values() if n.road_degree(net) > 0]
    deg = [n.road_degree(net) for n in real]
    dead = int(round(dead_end_share(net, region) * max(len(real), 1)))
    junc = [d for d in deg if d >= 3]
    three = sum(1 for d in junc if d == 3)
    length_km = sum(e.length for e in net.edges.values()
                    if e.road_class != "boundary") / 1000.0
    areas = sorted(b["area"] for b in blks)

    def med(v):
        return v[len(v) // 2] if v else 0.0

    return {
        "nodes": len(net.nodes),
        "edges": len(net.edges),
        "blocks": len(blks),
        "dead_end_pct": 100.0 * dead / max(len(deg), 1),
        "dead_end_target": "12-35%",
        "three_way_pct": 100.0 * three / max(len(junc), 1),
        "three_way_target": "69-87%",
        "street_km": length_km,
        "km_per_km2": length_km / max(km2, 1e-9),
        "km_per_km2_target": "7.4-12.4",
        "block_area_median": med(areas),
        "block_area_target": "15k-34k m²",
        # Not a target, a report: `undeveloped_share` asks for a share and the
        # bias decides which parcels get it, so what actually came out is worth
        # printing next to the four measures it moves.
        "undeveloped_pct": 100.0 * sum(1 for b in blks if b.get("undeveloped"))
        / max(len(blks), 1),
    }


def format_stats(s):
    return (
        f"  nodes {s['nodes']}  edges {s['edges']}  blocks {s['blocks']}\n"
        f"  dead ends    {s['dead_end_pct']:5.1f}%   target {s['dead_end_target']}\n"
        f"  three-way    {s['three_way_pct']:5.1f}%   target {s['three_way_target']}\n"
        f"  street dens. {s['km_per_km2']:5.1f} km/km²  target "
        f"{s['km_per_km2_target']}\n"
        f"  block median {s['block_area_median']:,.0f} m²  target "
        f"{s['block_area_target']}\n"
        f"  undeveloped  {s['undeveloped_pct']:5.1f}%   left as open land"
    )
