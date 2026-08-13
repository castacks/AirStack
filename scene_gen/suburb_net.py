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
    1  collectors         curvilinear routes crossing the interior, as a
                          harmonic offset from a straight baseline so curvature
                          is bounded and both ends leave the crop square
    2  face subdivision   split the largest remaining face until every block is
                          near target size — THE LAYER THAT FILLS THE LAND
    3  loops              a street leaving a host and returning to it
    4  lollipops          cul-de-sacs, grown until the dead-end share is met

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
mean of eight seeds at 1600 x 1200 m with the defaults below:

    measure               target           measured   spread over 8 seeds
    dead-end nodes        12-35%           18.3%      13.5 - 21.4
    three-way junctions   69-87%           76.6%      70.1 - 81.6
    street density        7.4-12.4 km/km²  11.0       10.7 - 11.3
    block area (median)   15k-34k m²       31.9k      27.9k - 34.7k

All four sit in band on every seed. Barrington-Leigh & Millard-Ball (PNAS 2015)
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
  capped at 500-750 ft by most ordinances.
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

    def clearance_allowing_crossings(self, pts, ignore=(), limit=None):
        """Clearance, but an edge the route PROPERLY CROSSES is not a conflict.

        Two collectors that cross are not too close together — they make a
        four-way junction, which `_connect_route` builds. Measuring raw distance
        would score that crossing as zero clearance and reject it, which is why
        only one collector ever landed. Near-parallel running is still rejected,
        because those edges are not crossed.
        """
        best = float("inf")
        for e in self.edges.values():
            if e.id in ignore:
                continue
            if _routes_cross(pts, e.pts):
                continue
            best = min(best, polyline_dist(pts, e.pts, limit=limit))
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


def blocks_from_faces(net, face_list, min_area=400.0):
    """Faces inset by the half-width of the road bounding each side.

    The face polygon runs down street CENTRELINES, so the buildable parcel is
    the face pulled in by half a carriageway on every side. Doing it per edge
    rather than uniformly is what lets a block that faces an arterial on one
    side and a cul-de-sac on another sit correctly against both.
    """
    out = []
    for f in face_list:
        poly, eids = f["poly"], f["edges"]
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
        a = polygon_area(shrunk)
        if a < min_area:
            continue
        out.append({"poly": shrunk, "area": a, "edges": eids,
                    "frontage": frontage,
                    "centroid": polygon_centroid(shrunk)})
    return out


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
    "loop_attempts": 420,
    "lollipop_attempts": 700,
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
    for _ in range(int(c["collectors"]) * 25):
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
        pts = resample(pts, sample)

        if min_radius(pts) < CLASSES["collector"]["min_radius_m"]:
            continue
        # Ignore the crop boundary: the route starts and ends ON it.
        if net.clearance_allowing_crossings(pts, ignore=boundary_ids,
                                            limit=min_gap) < min_gap:
            continue
        _connect_route(net, pts, "collector", "collector", min_gap)
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
                      junctions, min_depth=float(c["loop_min_depth_m"])):
            n_loops += 1

    n_conn = _subdivide_faces(net, rng, c, throat, min_gap, sample, region,
                              edge_clear, junctions, spaced_ok,
                              grain_base=grain_base, grain_swirl=grain_swirl)

    n_lolli = 0
    want_dead = float(c["dead_end_target"])
    for _ in range(int(c["lollipop_attempts"])):
        if dead_end_share(net, region) >= want_dead:
            break
        sids = net.street_ids("collector", "connector", "loop")
        if not sids:
            break
        if _grow_lollipop(net, sids[rng.randrange(len(sids))], rng, llen,
                          throat, min_gap, sample, region, edge_clear, bulb_r,
                          spaced_ok, junctions,
                          cul_gap_factor=float(c["cul_de_sac_gap_factor"])):
            n_lolli += 1

    face_list = faces(net)
    blks = blocks_from_faces(net, face_list)
    info = {"collectors": made_col, "connectors": n_conn, "loops": n_loops,
            "lollipops": n_lolli, "region": region}
    return net, blks, info


def _connect_route(net, pts, road_class, street_type, min_gap):
    """Add a route, splitting every existing edge it crosses and splitting
    itself at the same points.

    This is what keeps the graph PLANAR. A collector that runs over another
    collector without a node there would leave a face that is not a block and a
    junction that is not a junction; downstream, the two roads would simply
    overlap. Crossings become real 4-way nodes here, once, at construction.
    """
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


def _routes_cross(pa, pb):
    """True when two polylines properly intersect at least once."""
    for i in range(len(pa) - 1):
        for j in range(len(pb) - 1):
            if _seg_intersect(pa[i], pa[i + 1], pb[j], pb[j + 1]) is not None:
                return True
    return False


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
                edge_clear, junctions, spaced_ok, skew_deg=38.0, tries=26,
                four_way_chance=0.0, grain_w=0.0, grain_base=0.0,
                grain_swirl=0.0, grain_period=600.0, allow_fallback=False):
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
                         skew_deg, spaced_ok, grain_w=grain_w,
                         grain_base=grain_base, grain_swirl=grain_swirl,
                         grain_period=grain_period)
        if best is not None:
            break
    if best is None and allow_fallback:
        best = _fallback_cut(net, face, rng, parts, total, poly, throat,
                             min_gap, sample, min_block, skew_deg, spaced_ok)
    if best is None:
        return False
    _score, pts, p0, p1 = best
    return _commit_cut(net, pts, p0, p1, junctions)


def _fallback_cut(net, face, rng, parts, total, poly, throat, min_gap, sample,
                  min_block, skew_deg, spaced_ok, tries=90):
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
            if (_dot(chord, n0) < 0.35
                    or _dot(_mul(chord, -1.0), n1) < 0.35):
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
                if best is None or score < best[0]:
                    best = (score, pts, p0, p1)
                break
        if best is not None:
            return best
    return None


def _best_cut(net, face, rng, parts, total, poly, node_us, want_snap, tries,
              throat, min_gap, sample, min_block, skew_deg, spaced_ok,
              grain_w=0.0, grain_base=0.0, grain_swirl=0.0, grain_period=600.0):
    """Lowest-scoring valid street across the face, or None."""
    best = None
    for _try in range(tries):
        u0 = rng.uniform(0.0, total)
        snapped = False
        if want_snap and node_us:
            u0 = node_us[rng.randrange(len(node_us))]
            snapped = True
        u1 = u0 + total * rng.uniform(0.34, 0.66)
        if u1 >= total:
            u1 -= total
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
        # Must stay inside the face it is dividing, or it crosses a street.
        # This ALSO subsumes the region check: every face lies inside the
        # arterial frame, so a street inside a face is inside the region. The
        # explicit region test was not merely redundant but wrong — a junction
        # sits ON the frame, 7 m from the boundary, so an `edge_clear` of 26 m
        # rejected every street that touched the arterial (100 candidates).
        if any(not point_in_polygon(poly, q) for q in pts[2:-2]):
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
        if best is None or score < best[0]:
            best = (score, pts, p0, p1)
    return best


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


def _subdivide_faces(net, rng, cfg, throat, min_gap, sample, region,
                     edge_clear, junctions, spaced_ok, grain_base=0.0,
                     grain_swirl=0.0):
    """Keep splitting the largest face until every block is near target size."""
    target = float(cfg["block_area_target_m2"])
    min_block = float(cfg["block_area_min_m2"])
    hard_max = target * float(cfg.get("block_area_hard_max_factor", 1.5))
    made, stuck = 0, set()
    for _ in range(int(cfg["subdivide_iters"])):
        face_list = faces(net)
        big = [f for f in face_list
               if abs(polygon_area(f["poly"])) > target
               and tuple(sorted(f["edges"])) not in stuck]
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
        if _split_face(net, f, rng, throat, min_gap, sample, min_block, region,
                       edge_clear, junctions, spaced_ok,
                       skew_deg=float(cfg["junction_skew_deg"]),
                       four_way_chance=float(cfg["four_way_chance"]),
                       grain_w=float(cfg["grain_weight"]),
                       grain_base=grain_base, grain_swirl=grain_swirl,
                       grain_period=float(cfg["grain_period_m"]),
                       allow_fallback=area > hard_max):
            made += 1
        else:
            stuck.add(tuple(sorted(f["edges"])))
    return made


def _grow_lollipop(net, sid, rng, llen, throat, min_gap, sample, region,
                   edge_clear, bulb_r, spaced_ok, junctions,
                   cul_gap_factor=0.72):
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
               edge_clear, spaced_ok, junctions, min_depth=70.0):
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
    }


def format_stats(s):
    return (
        f"  nodes {s['nodes']}  edges {s['edges']}  blocks {s['blocks']}\n"
        f"  dead ends    {s['dead_end_pct']:5.1f}%   target {s['dead_end_target']}\n"
        f"  three-way    {s['three_way_pct']:5.1f}%   target {s['three_way_target']}\n"
        f"  street dens. {s['km_per_km2']:5.1f} km/km²  target "
        f"{s['km_per_km2_target']}\n"
        f"  block median {s['block_area_median']:,.0f} m²  target "
        f"{s['block_area_target']}"
    )
