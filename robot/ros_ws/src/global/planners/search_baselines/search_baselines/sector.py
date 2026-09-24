"""Sector geometry for the search baselines: grow a region, split it into
equal-area sectors, test containment.

WHY THIS DOES NOT IMPORT `raven_nav`. The static-sector arms are defined by
every robot computing the SAME partition from the SAME inputs and taking slice
`my_id`, exchanging no messages after t=0. That property only survives if the
partition is a pure function of (polygon, n, index) — no shared state, no
node handles, and no dependency that another arm might reconfigure underneath
it. `raven_nav` carries a point-in-polygon routine of its own and this module
reimplements it rather than importing it, because a package-local copy whose
only import is numpy is the cheapest way to guarantee that purity. The
duplication is the point, not an oversight.

Determinism is therefore load-bearing, and it constrains the algorithms below
in ways that look overcautious in isolation:

  * no randomness, no time-dependent branching, no iteration over a set/dict;
  * bisection runs to a FIXED iteration count against a FIXED tolerance, so
    two robots on two machines land on bit-identical cut coordinates;
  * cuts are axis-aligned, which lets a shared sector edge be resolved by a
    half-open containment rule (below) instead of by a distance tolerance. A
    tolerance would be a per-machine coin flip on exactly the points that
    matter — the ones on the seam between two drones.

FRAME. Map-frame metres, (x, y). A (M,3) polygon is accepted and z is dropped;
altitude is somebody else's constraint.

CONTAINMENT CONVENTION. `points_in_polygon` uses an even-odd ray cast whose
tie-breaking makes the region half-open: the LOWER and LEFT boundaries belong
to the polygon, the UPPER and RIGHT ones do not. That is not an accident of
the arithmetic, it is the reason the partition tiles exactly. Two adjacent
strips share a cut line; under this rule a point exactly on it is inside the
upper/right strip and outside the lower/left one, so it lands in exactly one
sector with no epsilon anywhere. The price is that a point on the outermost
top/right edge of the parent region is reported OUTSIDE the region — a drone
sitting precisely there is judged out of bounds and gets nudged back in, which
is the safe direction to be wrong in.

DEGENERATE INPUT. Nothing here raises on a malformed polygon and nothing fails
silently either. A polygon with fewer than three distinct vertices is treated
as UNBOUNDED: containment returns True everywhere, and partitioning hands every
index the input unchanged and logs a warning. The alternative reading — "no
valid geofence, so nothing is allowed" — would freeze a drone in place the
moment a parameter was mistyped, which is a worse failure than flying an
unconstrained mission.
"""

import logging
import math

import numpy as np

_LOG = logging.getLogger(__name__)

# A miter offset blows up as the interior angle closes: the corner point runs
# to infinity for a 0-degree spike. Cap the displacement at this multiple of
# the margin (a limit of 4 starts biting below a ~29-degree interior angle).
# Past the cap the corner is under-padded rather than launched into orbit.
_MITER_LIMIT = 4.0

# 1 + cos(theta) between the two edge normals. Near zero means the edges double
# back on each other (a zero-width spur), where the bisector is meaningless.
_MITER_MIN_DENOM = 1e-6

# Bisection budget for the equal-area cuts. 64 halvings drive the bracket to
# double precision over any plausible map extent, so the fixed count — not the
# tolerance — is what actually terminates the loop on every machine alike.
_BISECT_ITERS = 64

# Stop early once a slice is this close to its target share. The API contract
# allows 2%; bisection is cheap enough that there is no reason to spend it.
_AREA_RTOL = 0.002

_EPS = 1e-12


# ---------------------------------------------------------------------------
# primitives
# ---------------------------------------------------------------------------

def _as_poly(poly_xy):
    """Normalise anything polygon-shaped to a clean (M,2) float64 vertex ring.

    Geofences arrive from YAML, from ROS messages and from other planners, and
    each source has its own idea of the format: (M,3) with an altitude column,
    a closing vertex repeated to make the ring explicit, a vertex duplicated by
    an operator clicking twice. All three break the padding maths (a
    zero-length edge has no normal) while being harmless to area and
    containment, so they are normalised away once here instead of being
    special-cased in four places.
    """
    if poly_xy is None:
        return np.zeros((0, 2), dtype=np.float64)
    p = np.asarray(poly_xy, dtype=np.float64)
    if p.ndim != 2 or p.shape[0] == 0 or p.shape[1] < 2:
        return np.zeros((0, 2), dtype=np.float64)
    p = np.ascontiguousarray(p[:, :2])
    if p.shape[0] >= 2:
        # Drop every vertex coincident with its predecessor. Rolling by one
        # makes p[0]'s predecessor p[-1], so an explicit closing vertex is
        # collapsed by the same test that removes interior duplicates.
        step = np.linalg.norm(p - np.roll(p, 1, axis=0), axis=1)
        p = p[step > _EPS]
    return p


def _signed_area(p):
    """Shoelace with the sign kept: positive is counter-clockwise. `p` must
    already be normalised. The sign is what tells `pad_polygon` which side of
    an edge is outward, so it cannot be thrown away as early as the public
    `polygon_area` does."""
    if p.shape[0] < 3:
        return 0.0
    x, y = p[:, 0], p[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    return float(np.dot(x, y1) - np.dot(x1, y)) * 0.5


def polygon_area(poly_xy):
    """Absolute shoelace area in square metres. Degenerate rings return 0.0.

    Correct on a self-intersecting ring only in the specific sense the clipper
    needs: a Sutherland-Hodgman result whose two halves are joined by a
    zero-width bridge along the cut line sums to the true area, because the
    bridge encloses nothing. It is NOT a general signed-area-of-a-weird-loop
    guarantee, and no caller here needs one.
    """
    return abs(_signed_area(_as_poly(poly_xy)))


def polygon_centroid(poly_xy):
    """Area-weighted centroid, as (2,) float64.

    The vertex mean is the tempting one-liner and it is wrong: it weights a
    cluster of closely-spaced vertices on one edge as heavily as the whole
    opposite side, so a region drawn with more detail on its north face
    reports a centroid dragged north. The shoelace centroid is weighted by
    area and is invariant to how finely the boundary happens to be sampled.
    Zero-area rings have no area to weight by, so those — and those only —
    fall back to the vertex mean rather than dividing by zero.
    """
    p = _as_poly(poly_xy)
    if p.shape[0] == 0:
        return np.zeros(2, dtype=np.float64)
    if p.shape[0] < 3:
        return p.mean(axis=0)
    x, y = p[:, 0], p[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cross = x * y1 - x1 * y
    a2 = float(cross.sum())
    if abs(a2) < _EPS:
        return p.mean(axis=0)
    return np.array([float(np.dot(x + x1, cross)),
                     float(np.dot(y + y1, cross))]) / (3.0 * a2)


def points_in_polygon(pts_xy, poly_xy):
    """Vectorised even-odd ray cast. pts_xy: (N,2) or (N,3); poly_xy: (M,2).

    Returns (N,) bool. Concave polygons are handled — even-odd is a crossing
    parity test and never assumed convexity. A ring containing the zero-width
    bridges the clipper produces is handled too, since the bridge's two
    collinear edges either both count or both do not.

    Boundary convention (see the module docstring): the ray is cast along +x
    and a crossing counts only where the point lies STRICTLY left of it, so
    lower and left edges are inside and upper and right edges are outside. The
    region is half-open, adjacent sectors never both claim a point on their
    shared seam, and no epsilon is involved in deciding it.

    Empty `pts_xy` returns an empty bool array. A polygon with fewer than
    three distinct vertices returns all True, meaning UNBOUNDED — note that
    this is the opposite of the all-False that `raven_nav`'s copy returns, and
    deliberately so: here a missing polygon means "no constraint configured",
    not "no space is legal", and returning all-False would strand a drone.

    Loops over edges and vectorises over points rather than broadcasting to an
    (N,M) array: a geofence has a handful of vertices while N is thousands of
    frontier candidates, so the loop is short and the memory stays O(N).
    """
    pts = np.asarray(pts_xy, dtype=np.float64)
    if pts.size == 0:
        return np.zeros(0, dtype=bool)
    if pts.ndim == 1:
        pts = pts.reshape(1, -1)
    poly = _as_poly(poly_xy)
    if poly.shape[0] < 3:
        return np.ones(pts.shape[0], dtype=bool)

    px, py = pts[:, 0], pts[:, 1]
    inside = np.zeros(pts.shape[0], dtype=bool)
    x1, y1 = poly[:, 0], poly[:, 1]
    x2, y2 = np.roll(x1, -1), np.roll(y1, -1)
    for i in range(poly.shape[0]):
        # Half-open in y as well as x: `>` on both endpoints means a point at
        # exactly a vertex's height is counted by one of the two edges meeting
        # there, never both and never neither.
        straddles = (y1[i] > py) != (y2[i] > py)
        if not straddles.any():
            continue
        # straddles is all-False whenever y1 == y2, so this division is only
        # reached with a non-zero denominator; no epsilon fudge is needed and
        # none is added, because one would shift genuine crossings.
        x_cross = (x2[i] - x1[i]) * (py - y1[i]) / (y2[i] - y1[i]) + x1[i]
        inside ^= straddles & (px < x_cross)
    return inside


# ---------------------------------------------------------------------------
# clipping
# ---------------------------------------------------------------------------

def clip_polygon_halfplane(poly_xy, axis_idx, value, keep='below'):
    """Sutherland-Hodgman clip against an axis-aligned half-plane.

    axis_idx: 0 for x, 1 for y. keep='below' keeps coord <= value, 'above'
    keeps coord >= value. Returns (M,2); an empty (0,2) array when nothing
    survives, which every caller must read as "this sector has no area".

    Both sides use a non-strict comparison, so a vertex sitting exactly on the
    plane is inside for BOTH halves. That is what makes the two halves share
    bit-identical edge coordinates, which is in turn what lets the half-open
    containment rule assign a seam point to exactly one of them. The
    intersection coordinate is additionally SNAPPED to `value` rather than
    left as cur + t*(nxt-cur): the interpolation is off by an ulp or two, and
    an ulp of disagreement between two sectors' shared edge is exactly the gap
    a point can fall through.

    Clipping a concave polygon that the plane cuts into disconnected pieces
    yields a single ring whose pieces are joined by zero-width bridges lying
    on the plane. That is left as-is rather than split into components,
    because both operations that consume the result — shoelace area and
    even-odd containment — are already correct on such a ring, and a component
    split would mean sectors that are lists of polygons everywhere downstream.
    """
    p = _as_poly(poly_xy)
    if p.shape[0] < 3:
        return np.zeros((0, 2), dtype=np.float64)
    ai = int(axis_idx)
    if ai not in (0, 1):
        raise ValueError("axis_idx must be 0 (x) or 1 (y), got %r" % (axis_idx,))
    value = float(value)
    if keep == 'below':
        inside = p[:, ai] <= value
    elif keep == 'above':
        inside = p[:, ai] >= value
    else:
        raise ValueError("keep must be 'below' or 'above', got %r" % (keep,))

    out = []
    m = p.shape[0]
    for i in range(m):
        j = (i + 1) % m
        cur, nxt = p[i], p[j]
        ci, cj = inside[i], inside[j]
        if ci != cj:
            d = nxt[ai] - cur[ai]
            # ci != cj forces the endpoints onto opposite sides, so d != 0;
            # the guard is here so a future caller cannot turn it into a nan.
            t = 0.0 if d == 0.0 else (value - cur[ai]) / d
            hit = cur + t * (nxt - cur)
            hit[ai] = value
        if cj:
            if not ci:
                out.append(hit)
            out.append(nxt)
        elif ci:
            out.append(hit)

    if len(out) < 3:
        return np.zeros((0, 2), dtype=np.float64)
    # Vertices exactly on the plane get emitted twice by the branch above.
    # Zero-length edges are harmless to area and containment but they break
    # the normal computation in pad_polygon, so drop them at the source.
    ring = [out[0]]
    for q in out[1:]:
        if q[0] != ring[-1][0] or q[1] != ring[-1][1]:
            ring.append(q)
    if len(ring) > 1 and ring[0][0] == ring[-1][0] and ring[0][1] == ring[-1][1]:
        ring.pop()
    if len(ring) < 3:
        return np.zeros((0, 2), dtype=np.float64)
    return np.asarray(ring, dtype=np.float64)


# ---------------------------------------------------------------------------
# padding
# ---------------------------------------------------------------------------

def _unit_normals(edges, orient):
    """Outward unit normal per edge, for a ring of the given orientation.

    perp((dx,dy)) = (dy,-dx) points outward for a counter-clockwise ring;
    `orient` (+1 CCW / -1 CW) flips it for the other winding so callers never
    have to pre-normalise the input's handedness.
    """
    n = np.stack([edges[:, 1], -edges[:, 0]], axis=1) * orient
    length = np.linalg.norm(n, axis=1)
    length[length < _EPS] = 1.0
    return n / length[:, None]


def _miter_offset(p, margin):
    """Push every vertex out along its angle-bisector by `margin`, or None if
    the ring is too degenerate for the bisector to mean anything."""
    s = _signed_area(p)
    if abs(s) < _EPS:
        return None
    orient = 1.0 if s > 0.0 else -1.0

    prev_edge = p - np.roll(p, 1, axis=0)      # edge arriving at each vertex
    next_edge = np.roll(p, -1, axis=0) - p     # edge leaving each vertex
    na = _unit_normals(prev_edge, orient)
    nb = _unit_normals(next_edge, orient)

    # The point at distance `margin` from BOTH offset edge lines is
    # v + margin * (na + nb) / (1 + na.nb). It reduces to v + margin*n on a
    # straight run and to the true corner on a square, and — unlike scaling
    # about the centroid — it does not care about the vertex's distance from
    # anything, so the metric margin is uniform on an elongated region.
    denom = 1.0 + np.sum(na * nb, axis=1)
    bisector = na + nb
    off = np.empty_like(p)
    sane = denom > _MITER_MIN_DENOM
    off[sane] = margin * bisector[sane] / denom[sane][:, None]
    # A near-180-degree turn is a zero-width spur with no meaningful bisector;
    # pushing along the outgoing normal keeps the vertex outside the original.
    off[~sane] = margin * nb[~sane]

    length = np.linalg.norm(off, axis=1)
    runaway = length > _MITER_LIMIT * margin
    if runaway.any():
        off[runaway] *= (_MITER_LIMIT * margin / length[runaway])[:, None]
    return p + off


def _orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_cross(p1, q1, p2, q2):
    """True only for a PROPER crossing (interiors meet transversally).

    Touching endpoints and collinear overlaps are excluded on purpose: the
    clipper legitimately produces both, and flagging them would send every
    padded L-shape down the conservative fallback path for no reason.
    """
    d1 = _orient(p1, q1, p2)
    d2 = _orient(p1, q1, q2)
    d3 = _orient(p2, q2, p1)
    d4 = _orient(p2, q2, q1)
    return (d1 * d2 < 0.0) and (d3 * d4 < 0.0)


def _collinear_overlap(p1, q1, p2, q2, tol):
    """True when two segments lie on the same line and share positive length.

    Worth its own test because an over-wide offset does NOT always announce
    itself as a proper crossing. Fold a slot narrower than 2*margin and its
    two walls swap sides; the walls themselves no longer intersect, and the
    inversion surfaces only as the two edges at the slot mouth running back
    over each other along the same line. Without this the ring passes for
    simple while enclosing a phantom hole.
    """
    d = q1 - p1
    n = float(np.linalg.norm(d))
    if n < _EPS:
        return False
    u = d / n
    # Perpendicular offsets of the other segment's endpoints from this line.
    if (abs(u[0] * (p2[1] - p1[1]) - u[1] * (p2[0] - p1[0])) > tol or
            abs(u[0] * (q2[1] - p1[1]) - u[1] * (q2[0] - p1[0])) > tol):
        return False
    t0 = float(u[0] * (p2[0] - p1[0]) + u[1] * (p2[1] - p1[1]))
    t1 = float(u[0] * (q2[0] - p1[0]) + u[1] * (q2[1] - p1[1]))
    return min(max(t0, t1), n) - max(min(t0, t1), 0.0) > tol


def _is_simple(p):
    """O(M^2) test over non-adjacent edge pairs: no proper crossings, and no
    collinear overlaps either.

    Quadratic is fine and a sweep line is not worth writing: a geofence is a
    handful of vertices, and this runs once per padding call, not per tick.
    """
    m = p.shape[0]
    if m < 4:
        return True
    # Tolerances scale with the region so the test means the same thing on a
    # 50 m courtyard and a 5 km wildfire perimeter.
    scale = float(np.max(p.max(axis=0) - p.min(axis=0)))
    tol = max(scale, 1.0) * 1e-9
    nxt = np.roll(p, -1, axis=0)
    for i in range(m):
        for j in range(i + 1, m):
            if j == (i + 1) % m or i == (j + 1) % m:
                continue
            if _segments_cross(p[i], nxt[i], p[j], nxt[j]):
                return False
            if _collinear_overlap(p[i], nxt[i], p[j], nxt[j], tol):
                return False
    return True


def _convex_hull(p):
    """Andrew's monotone chain, counter-clockwise. Returns the input unchanged
    if the points are collinear (fewer than three hull vertices)."""
    if p.shape[0] < 3:
        return p.copy()
    q = p[np.lexsort((p[:, 1], p[:, 0]))]

    def _half(pts):
        chain = []
        for pt in pts:
            while len(chain) >= 2 and _orient(chain[-2], chain[-1], pt) <= 0.0:
                chain.pop()
            chain.append(pt)
        return chain

    lower = _half(q)
    upper = _half(q[::-1])
    hull = np.asarray(lower[:-1] + upper[:-1], dtype=np.float64)
    return hull if hull.shape[0] >= 3 else p.copy()


def _padded_bbox(p, margin):
    lo = p.min(axis=0) - margin
    hi = p.max(axis=0) + margin
    return np.array([[lo[0], lo[1]], [hi[0], lo[1]],
                     [hi[0], hi[1]], [lo[0], hi[1]]], dtype=np.float64)


def _pad_is_sane(original, candidate, margin):
    """The properties a padded region must have to be usable: a simple ring,
    containing everything the caller drew, no smaller than the original, and
    no larger than the original's bounding box grown by the same margin.

    That last bound is the one that catches a self-overlapping offset. The
    true Minkowski offset is provably inside the padded bounding box, so its
    area cannot exceed it; a ring that does exceed it is counting some patch
    of ground twice, which means it folded over itself somewhere. It costs one
    shoelace and it is independent of whether the fold happens to present as a
    crossing or as a collinear overlap.

    Checked rather than assumed, because the miter offset is only guaranteed
    to deliver any of this on convex input.
    """
    if candidate is None or candidate.shape[0] < 3:
        return False
    if not np.all(np.isfinite(candidate)):
        return False
    area = polygon_area(candidate)
    if area < polygon_area(original) - 1e-9:
        return False
    box = _padded_bbox(original, margin)
    if area > polygon_area(box) * (1.0 + 1e-9) + 1e-9:
        return False
    if not points_in_polygon(original, candidate).all():
        return False
    return _is_simple(candidate)


def pad_polygon(poly_xy, margin_m):
    """Grow a polygon outward by roughly `margin_m` metres on all sides.

    WHY NOT SCALE ABOUT THE CENTROID. Scaling is the obvious three-line
    version and it is wrong in metres. A 1200x100 m river corridor scaled to
    give 10 m of clearance across its short axis gains 120 m along its long
    one; ask for clearance the other way round and the corridor's ends barely
    move. The requested quantity is a distance, and scaling delivers a ratio.

    WHY NOT A FULL MINKOWSKI OFFSET. The exact answer — offset every edge,
    round or miter the convex corners, then remove the self-intersecting loops
    a concave corner generates — needs a robust polygon boolean library. For
    "slightly more than the region on all sides" on a hand-drawn geofence that
    is a large dependency and a large surface for numerical failure.

    WHAT THIS DOES. Each vertex is pushed along its angle-bisector by the
    amount that puts it `margin_m` from both of its edge lines. On convex
    corners that is the exact Minkowski result; on straight runs it is exactly
    a perpendicular push; on reflex corners it correctly closes a notch in
    from both sides. The metric margin is therefore uniform, independent of
    the region's aspect ratio.

    THE LIMITATION ACCEPTED. On a strongly non-convex polygon, a margin larger
    than a local feature — a slot narrower than 2*margin_m, a spike shorter
    than it — makes the offset walls cross each other, which the exact
    algorithm resolves by deleting the inverted loop and this one cannot. So
    the result is VALIDATED, not trusted: it must be a simple ring (no proper
    crossings AND no collinear overlaps, since a folded slot shows up as the
    latter), it must contain every original vertex, and its area must sit
    between the original's and that of the original's bounding box grown by
    the same margin — a fold double-counts ground and breaches the upper
    bound. Failing any of those, the fallback is the padded convex hull
    (convex input always offsets cleanly), and if the points are collinear so
    even the hull is degenerate, the padded bounding box. Both fallbacks are
    supersets of the correct answer, so the failure mode is "searched more
    ground than strictly necessary", never "flew outside a region it thought
    was inside". Falling back logs a warning; it is never silent.

    Fewer than three distinct vertices, or margin_m <= 0, returns the
    normalised input unchanged — there is nothing to grow and nothing to
    grow it from.
    """
    p = _as_poly(poly_xy)
    margin = float(margin_m)
    if p.shape[0] < 3 or margin <= 0.0:
        return p.copy()

    candidate = _miter_offset(p, margin)
    if _pad_is_sane(p, candidate, margin):
        return candidate

    hull = _convex_hull(p)
    if hull.shape[0] >= 3:
        candidate = _miter_offset(hull, margin)
        if _pad_is_sane(p, candidate, margin):
            _LOG.warning(
                "pad_polygon: %.2f m offset self-intersects on a %d-vertex "
                "concave region (margin exceeds a local feature size); using "
                "the padded convex hull, which over-covers by %.1f m^2",
                margin, p.shape[0], polygon_area(candidate) - polygon_area(p))
            return candidate

    box = _padded_bbox(p, margin)
    _LOG.warning(
        "pad_polygon: neither the %.2f m offset nor the padded hull is usable "
        "for this %d-vertex region; falling back to the padded bounding box",
        margin, p.shape[0])
    return box


# ---------------------------------------------------------------------------
# partitioning
# ---------------------------------------------------------------------------

def _pick_axis(p, axis):
    """Resolve axis='auto'|'x'|'y' to a coordinate index.

    'auto' cuts along the LONGER extent, which is what makes strips short and
    fat: six 200x100 m sectors beat one 1200x100 m sector because a lawnmower
    over the long thin one spends its life on transit legs, and because a
    frontier planner inside it keeps being pulled the length of the region.

    The choice is between x and y only — the true PCA principal axis is not
    used even though it would suit a diagonal region better. A rotated cut
    puts the seam between two sectors at an arbitrary angle, where the
    half-open containment rule no longer resolves a point on the seam by
    inspection, and the tiling guarantee that lets a robot trust its own
    boundary with zero communication is worth more than tighter sectors on
    diagonal regions.

    Ties go to x, deterministically: a square region must partition the same
    way on every robot.
    """
    if axis == 'x':
        return 0
    if axis == 'y':
        return 1
    if axis != 'auto':
        raise ValueError("axis must be 'auto', 'x' or 'y', got %r" % (axis,))
    w = float(p[:, 0].max() - p[:, 0].min())
    h = float(p[:, 1].max() - p[:, 1].min())
    return 0 if w >= h else 1


def _area_cuts(p, axis_idx, fracs):
    """Coordinates where the area below the cut hits each requested fraction.

    Bisection, not an analytic sweep. The area below a moving line is
    monotone and continuous but its derivative jumps at every vertex, so
    solving it exactly means an event-sorted sweep with per-segment quadratics
    — much more code, and it earns nothing here because each bisection step is
    one clip plus one shoelace over a tiny ring.

    Bisecting on AREA rather than splitting the extent evenly is the whole
    point: on anything that is not a rectangle, equal width is not equal area,
    and the drone assigned the pinched end of a region would finish long
    before the one assigned the fat end.
    """
    lo = float(p[:, axis_idx].min())
    hi = float(p[:, axis_idx].max())
    total = polygon_area(p)
    cuts = []
    for f in fracs:
        target = total * float(f)
        tol = _AREA_RTOL * total / max(len(fracs) + 1, 1)
        a, b = lo, hi
        for _ in range(_BISECT_ITERS):
            mid = 0.5 * (a + b)
            below = polygon_area(clip_polygon_halfplane(p, axis_idx, mid, 'below'))
            if abs(below - target) <= tol:
                a = b = mid
                break
            if below < target:
                a = mid
            else:
                b = mid
        cuts.append(0.5 * (a + b))
    return cuts


def _slice_by_fracs(p, axis_idx, fracs):
    """Cut `p` at the given cumulative area fractions, low coordinate first."""
    cuts = _area_cuts(p, axis_idx, fracs)
    pieces = []
    for k in range(len(cuts) + 1):
        piece = p
        if k > 0:
            piece = clip_polygon_halfplane(piece, axis_idx, cuts[k - 1], 'above')
        if k < len(cuts):
            piece = clip_polygon_halfplane(piece, axis_idx, cuts[k], 'below')
        if piece.shape[0] < 3:
            _LOG.warning(
                "partition: slice %d of %d came back empty on axis %d — the "
                "region is probably disconnected or thinner than the cut "
                "spacing; that index gets no area",
                k, len(cuts) + 1, axis_idx)
        pieces.append(np.asarray(piece, dtype=np.float64))
    return pieces


def _grid_rows(p, n):
    """Row count that makes tiles as close to square as the aspect allows.

    A tile is about (W/cols) x (H/rows); setting that ratio to 1 with
    rows*cols = n gives rows = sqrt(n*H/W). Rounding it is enough — the
    remaining choice is only ever between two adjacent integers, and the
    aspect penalty between them is small compared to being off by a factor.
    """
    w = float(p[:, 0].max() - p[:, 0].min())
    h = float(p[:, 1].max() - p[:, 1].min())
    if w <= _EPS or h <= _EPS:
        return 1
    return max(1, min(n, int(round(math.sqrt(n * h / w)))))


def _grid(p, n):
    """Rows of near-equal-area tiles, row-major from the bottom.

    Rows carry UNEQUAL tile counts when n does not factor nicely — n=5 becomes
    a 3-tile row and a 2-tile row, not a 1x5 line — and each row band is given
    area proportional to its tile count, so every tile still comes out at
    area/n. Restricting to exact factor pairs would silently degrade every
    prime fleet size back to strips.
    """
    rows = _grid_rows(p, n)
    base, extra = divmod(n, rows)
    per_row = [base + (1 if i < extra else 0) for i in range(rows)]

    running = np.cumsum(per_row[:-1]) / float(n)
    bands = _slice_by_fracs(p, 1, list(running)) if rows > 1 else [p]

    tiles = []
    for band, count in zip(bands, per_row):
        if count <= 1 or band.shape[0] < 3:
            tiles.append(band)
            continue
        fracs = [k / float(count) for k in range(1, count)]
        tiles.extend(_slice_by_fracs(band, 0, fracs))
    return tiles


def _principal_frame(p):
    """Rotation matrix R whose FIRST row is the polygon's major axis (unit),
    second row the minor: `(p - c) @ R.T` puts the polygon in its own frame.

    PCA of the vertex set. That is exact for the evenly sampled front ellipse
    the wildfire scenes hand in and for a rectangle's four corners, and only
    approximate for a polygon whose vertices are unevenly spaced — which is
    the sampling bias `lawnmower._axis_direction` avoids with rotating
    calipers; here a few degrees of tilt only costs a little slack in the
    rectangles, so the cheaper estimate is taken. The sign is fixed so the
    major axis points into +x (or +y on a tie), which keeps the band order
    the same on every robot.
    """
    q = p - p.mean(axis=0)
    cov = q.T @ q
    w, v = np.linalg.eigh(cov)
    major = v[:, int(np.argmax(w))]
    if major[0] < 0 or (major[0] == 0 and major[1] < 0):
        major = -major
    minor = np.array([-major[1], major[0]])
    return np.stack([major, minor])


def _rect_bands(p, axis_idx, n, frame=None):
    """n equal-WIDTH bands across the polygon's extent along `axis_idx`, each
    sector the axis-aligned BOUNDING RECTANGLE of (band ∩ polygon).

    With `frame` (a 2x2 rotation, see `_principal_frame`) the same is done in
    the polygon's OWN frame and the rectangles are rotated back: bands along
    the region's major axis, each rectangle only as wide across as the region
    is there. For a diagonal footprint that is the difference between hugging
    the damage and covering the diagonal of the map — measured on the 1 km
    wildfire burn (+50 m pad, 5 robots): axis-aligned rectangles are 196 m
    wide by 626-888 m long (the middle one spans the diagonal), oriented ones
    245 m along the burn by 500-620 m across it, 123-152k m^2 each, union 70%
    of the plat against 74% axis-aligned and 92% for the old whole-plat strips.

    For a region that is not a rectangle — the fire ellipse on the wildfire
    plats — equal-area strips of the region itself are slanted slivers, and
    a drone confined to one spends its time on the seam. A rectangle per band
    that reaches only as far as the damage does in that band is the shape a
    search actually wants: axis-aligned, so lawnmower lanes and the half-open
    seam rule hold; short where the region is short, so the fleet is not sent
    over ground the disaster never touched; and roughly, not exactly, equal —
    the ends of a diagonal ellipse get less, which is what is there.

    Adjacent rectangles share their seam coordinate exactly (the same
    arithmetic on both sides), so a point on the seam belongs to exactly one.
    A band that misses the polygon entirely (only possible with a concave
    region) gets the band's slice of the polygon's bounding box instead, so
    an index never comes back empty.
    """
    centre = p.mean(axis=0)
    if frame is not None:
        p = (p - centre) @ frame.T
    lo = float(p[:, axis_idx].min())
    hi = float(p[:, axis_idx].max())
    other = 1 - axis_idx
    o_lo = float(p[:, other].min())
    o_hi = float(p[:, other].max())
    cuts = [lo + (hi - lo) * k / float(n) for k in range(n + 1)]
    out = []
    for k in range(n):
        a, b = cuts[k], cuts[k + 1]
        piece = clip_polygon_halfplane(p, axis_idx, a, keep='above')
        if piece.shape[0]:
            piece = clip_polygon_halfplane(piece, axis_idx, b, keep='below')
        if piece.shape[0] >= 3 and polygon_area(piece) > _EPS:
            p_lo = float(piece[:, other].min())
            p_hi = float(piece[:, other].max())
        else:
            p_lo, p_hi = o_lo, o_hi
        rect = np.zeros((4, 2), dtype=float)
        rect[:, axis_idx] = [a, b, b, a]
        rect[:, other] = [p_lo, p_lo, p_hi, p_hi]
        if frame is not None:
            rect = rect @ frame + centre
        out.append(rect)
    return out


def partition(poly_xy, n, mode='strips', axis='auto'):
    """Split a polygon into `n` sectors of approximately equal AREA.

    Returns a list of `n` (M,2) arrays in index order, tiling the input with
    no overlap: every point strictly inside the input is inside exactly one
    sector, guaranteed by axis-aligned cuts plus the half-open containment
    rule rather than by any tolerance.

    mode='strips' cuts parallel bands along one axis. axis='auto' takes the
    longer extent so strips come out short and fat (see `_pick_axis`);
    'x'/'y' force it. This is the shape the roster asks for, and it is also
    the shape the lawnmower generator wants, since a strip of a rectangle is a
    rectangle and `generate_lawnmower` takes length/width directly.

    mode='rect' cuts n equal-WIDTH bands along the axis and hands each robot
    the bounding RECTANGLE of its band's share of the region — see
    `_rect_bands`: the shape for a non-rectangular damage footprint, tight to
    the damage in each band and only roughly equal in area. axis='principal'
    (rect only) cuts along the region's own major axis and returns ROTATED
    rectangles, which is what hugs a diagonal footprint.

    mode='grid' lays out near-square tiles, choosing the row count from n and
    the region's aspect ratio. `axis` is ignored in this mode — the aspect
    ratio already determines the layout, and honouring `axis` too would mean
    two knobs fighting over the same decision. Square tiles minimise the
    perimeter a drone can be pulled to and suit an isotropic search; strips
    suit a boustrophedon sweep. Neither dominates, which is why both exist.

    Equal area, not equal width: for anything non-rectangular those differ,
    and equal width would hand one drone a pinched corner and another the
    bulk of the region. Cuts are bisected until each slice is within
    _AREA_RTOL of its share.

    n <= 1 returns [poly] unchanged. A polygon with fewer than three distinct
    vertices, or zero area, cannot be split: every index gets the input
    unchanged and a warning is logged. Combined with `points_in_polygon`
    treating such a polygon as unbounded, that degrades to "no sectoring, no
    constraint" instead of to a fleet with nowhere legal to fly.
    """
    p = _as_poly(poly_xy)
    n = int(n)
    if n <= 1:
        return [p.copy()]

    total = polygon_area(p)
    if p.shape[0] < 3 or total <= _EPS:
        _LOG.warning(
            "partition: cannot split a degenerate region (%d distinct "
            "vertices, area %.6g m^2) into %d sectors; every index gets the "
            "whole input, which containment then treats as unbounded",
            p.shape[0], total, n)
        return [p.copy() for _ in range(n)]

    if mode == 'strips':
        axis_idx = _pick_axis(p, axis)
        return _slice_by_fracs(p, axis_idx, [k / float(n) for k in range(1, n)])
    if mode == 'rect':
        if str(axis).lower() == 'principal':
            return _rect_bands(p, 0, n, frame=_principal_frame(p))
        return _rect_bands(p, _pick_axis(p, axis), n)
    if mode == 'grid':
        return _grid(p, n)
    raise ValueError("mode must be 'strips', 'rect' or 'grid', got %r" % (mode,))


def sector_for(poly_xy, n, index, mode='strips', axis='auto', margin_m=0.0):
    """Pad, partition, and hand back sector `index` (0-based).

    The one call a planner node needs: same inputs on every robot, different
    `index`, no messages exchanged.

    `index` is CLAMPED into [0, n-1] and the clamp is logged. A robot id that
    exceeds the configured fleet size is a launch-file mistake, and the two
    honest responses are to crash the node or to over-subscribe one sector.
    Mid-mission, a second drone sharing the last sector costs some duplicated
    coverage; a node that will not start costs the whole run.

    n is likewise floored at 1, so `n=0` degrades to "one sector, the whole
    padded region" rather than dividing by zero.
    """
    n = max(1, int(n))
    padded = pad_polygon(poly_xy, margin_m) if margin_m > 0 else _as_poly(poly_xy)
    sectors = partition(padded, n, mode=mode, axis=axis)
    i = int(index)
    if i < 0 or i >= len(sectors):
        clamped = min(max(i, 0), len(sectors) - 1)
        _LOG.warning(
            "sector_for: index %d is outside [0, %d] for a %d-way partition; "
            "clamping to %d, so that sector is flown by more than one robot",
            i, len(sectors) - 1, len(sectors), clamped)
        i = clamped
    return sectors[i]


# ---------------------------------------------------------------------------
# self-test: python3 sector.py
# ---------------------------------------------------------------------------

def _min_edge_distance(pts, poly):
    """Smallest distance from any of `pts` to any edge of `poly`. Test-only:
    the parity test cannot distinguish "inside" from "on the boundary"."""
    pts = np.asarray(pts, dtype=np.float64)
    poly = _as_poly(poly)
    a = poly
    b = np.roll(poly, -1, axis=0)
    best = np.inf
    for p in pts:
        ab = b - a
        t = np.clip(np.sum((p - a) * ab, axis=1) /
                    np.maximum(np.sum(ab * ab, axis=1), _EPS), 0.0, 1.0)
        d = np.linalg.norm(a + t[:, None] * ab - p, axis=1)
        best = min(best, float(d.min()))
    return best


def _selftest():
    rng = np.random.default_rng(0)

    RECT = np.array([[0., 0.], [300., 0.], [300., 120.], [0., 120.]])
    LSHAPE = np.array([[0., 0.], [100., 0.], [100., 50.],
                       [50., 50.], [50., 100.], [0., 100.]])
    SQUARE = np.array([[0., 0.], [100., 0.], [100., 100.], [0., 100.]])
    LONG = np.array([[0., 0.], [400., 0.], [400., 50.], [0., 50.]])
    SLOTTED = np.array([[0., 0.], [100., 0.], [100., 100.], [60., 100.],
                        [60., 20.], [40., 20.], [40., 100.], [0., 100.]])

    def sample_inside(poly, want, pad=0.0):
        lo = poly.min(axis=0) - pad
        hi = poly.max(axis=0) + pad
        got = []
        while sum(len(g) for g in got) < want:
            c = rng.uniform(lo, hi, size=(4 * want, 2))
            got.append(c[points_in_polygon(c, poly)])
        return np.vstack(got)[:want]

    # -- areas and shoelace -------------------------------------------------
    assert abs(polygon_area(RECT) - 300. * 120.) < 1e-9
    assert abs(polygon_area(LSHAPE) - 7500.) < 1e-9
    assert abs(polygon_area(LSHAPE[::-1]) - 7500.) < 1e-9, "winding must not matter"
    assert polygon_area(np.array([[0., 0.], [1., 1.]])) == 0.0
    assert np.allclose(polygon_centroid(SQUARE), [50., 50.])
    # A vertex-mean centroid would be dragged by the extra vertices on one
    # edge; the shoelace one must not move at all.
    dense = np.array([[0., 0.], [25., 0.], [50., 0.], [75., 0.],
                      [100., 0.], [100., 100.], [0., 100.]])
    assert np.allclose(polygon_centroid(dense), [50., 50.]), "centroid is area-weighted"
    print("ok  area / centroid")

    # -- containment conventions -------------------------------------------
    assert points_in_polygon(np.zeros((0, 2)), SQUARE).shape == (0,)
    assert points_in_polygon([[5., 5.]], np.array([[0., 0.], [1., 1.]])).all(), \
        "degenerate polygon must read as unbounded"
    assert points_in_polygon([[5., 5.]], None).all()
    assert not points_in_polygon([[75., 75.]], LSHAPE)[0], "notch is outside"
    assert points_in_polygon([[25., 75.]], LSHAPE)[0]
    edge = points_in_polygon([[0., 50.], [100., 50.], [50., 0.], [50., 100.]], SQUARE)
    assert list(edge) == [True, False, True, False], \
        "half-open: left/bottom inside, right/top outside"
    # Concave, with the ray crossing four edges: the slot spans x 40..60 above
    # y=20, so a point in it is outside while one under it is inside.
    assert points_in_polygon([[50., 10.]], SLOTTED)[0]
    assert not points_in_polygon([[50., 75.]], SLOTTED)[0]
    assert points_in_polygon([[30., 75.]], SLOTTED)[0]
    print("ok  containment (empty / degenerate / concave / on-edge)")

    # -- clipping -----------------------------------------------------------
    half = clip_polygon_halfplane(SQUARE, 0, 40., 'below')
    assert abs(polygon_area(half) - 4000.) < 1e-9
    assert clip_polygon_halfplane(SQUARE, 0, -10., 'below').shape == (0, 2), \
        "an empty clip must return (0,2), not garbage"
    assert abs(polygon_area(clip_polygon_halfplane(LSHAPE, 0, 50., 'above'))
               - 2500.) < 1e-9, "clip through a reflex vertex"
    print("ok  half-plane clip")

    # -- equal area ---------------------------------------------------------
    for name, poly in (('rect', RECT), ('L', LSHAPE), ('slotted', SLOTTED)):
        for mode in ('strips', 'grid'):
            for n in (1, 2, 3, 5):
                secs = partition(poly, n, mode=mode)
                total = polygon_area(poly)
                areas = np.array([polygon_area(s) for s in secs])
                assert len(secs) == n, (name, mode, n)
                assert abs(areas.sum() - total) < 1e-6 * total, \
                    ("areas must sum to the whole", name, mode, n, areas.sum(), total)
                worst = np.max(np.abs(areas - total / n)) / (total / n)
                assert worst <= 0.05, (name, mode, n, worst, areas)
    assert np.allclose(partition(LSHAPE, 1)[0], _as_poly(LSHAPE)), "n=1 is identity"
    print("ok  equal area  (rect / L / slotted x strips,grid x n=1,2,3,5, <=5%)")

    # -- tiling and containment --------------------------------------------
    for name, poly in (('rect', RECT), ('L', LSHAPE), ('slotted', SLOTTED)):
        for mode in ('strips', 'grid'):
            for n in (2, 3, 5):
                secs = partition(poly, n, mode=mode)
                pts = sample_inside(poly, 5000)
                hits = np.zeros(pts.shape[0], dtype=int)
                for s in secs:
                    hits += points_in_polygon(pts, s).astype(int)
                assert (hits == 1).all(), \
                    (name, mode, n, "multi/zero-claimed", int((hits != 1).sum()),
                     pts[hits != 1][:3])
                # Outside the parent must be outside every sector.
                lo, hi = poly.min(axis=0) - 40., poly.max(axis=0) + 40.
                cand = rng.uniform(lo, hi, size=(6000, 2))
                out = cand[~points_in_polygon(cand, poly)]
                leak = np.zeros(out.shape[0], dtype=int)
                for s in secs:
                    leak += points_in_polygon(out, s).astype(int)
                assert (leak == 0).all(), (name, mode, n, "sector escapes parent",
                                           int((leak != 0).sum()))
    print("ok  tiling: 5000 pts land in exactly one sector; outside lands in none")

    # -- axis auto ----------------------------------------------------------
    secs = partition(LONG, 4, mode='strips', axis='auto')
    for s in secs:
        w = s[:, 0].max() - s[:, 0].min()
        h = s[:, 1].max() - s[:, 1].min()
        assert abs(w - 100.) < 1e-6 and abs(h - 50.) < 1e-6, \
            ("auto must cut along x on a 400x50 region", w, h)
    forced = partition(LONG, 4, mode='strips', axis='y')
    fw = forced[0][:, 0].max() - forced[0][:, 0].min()
    assert abs(fw - 400.) < 1e-6, "axis='y' must cut the other way"
    tall = partition(np.array([[0., 0.], [50., 0.], [50., 400.], [0., 400.]]),
                     4, mode='strips', axis='auto')
    assert abs((tall[0][:, 1].max() - tall[0][:, 1].min()) - 100.) < 1e-6, \
        "auto must follow the long axis when it is y"
    print("ok  axis='auto' cuts along the long axis (x on 400x50, y on 50x400)")

    # -- rect: equal-width bands, bbox of the region in each --------------
    ang = np.linspace(0., 2. * np.pi, 72, endpoint=False)
    ELL = np.stack([250. * np.cos(ang), 120. * np.sin(ang)], axis=1)
    c, sn = np.cos(np.pi / 4.), np.sin(np.pi / 4.)
    ELL = ELL @ np.array([[c, sn], [-sn, c]])        # a 45-degree ellipse
    rects = partition(ELL, 5, mode='rect', axis='auto')
    assert len(rects) == 5 and all(r.shape == (4, 2) for r in rects)
    xs = sorted(float(r[:, 0].min()) for r in rects)
    assert abs((xs[1] - xs[0]) - (ELL[:, 0].max() - ELL[:, 0].min()) / 5.) < 1e-9, "equal width"
    for k in range(4):
        assert rects[k][:, 0].max() == rects[k + 1][:, 0].min(), "seams shared exactly"
    inside = sample_inside(ELL, 3000)
    owners = np.zeros(len(inside), dtype=int)
    for r in rects:
        owners += points_in_polygon(inside, r).astype(int)
    assert (owners == 1).all(), "every point of the region in exactly one rectangle"
    for r in rects:                                   # each rect hugs its band's share
        band = clip_polygon_halfplane(clip_polygon_halfplane(ELL, 0, r[:, 0].min(), 'above'),
                                      0, r[:, 0].max(), 'below')
        assert abs(r[:, 1].min() - band[:, 1].min()) < 1e-9
        assert abs(r[:, 1].max() - band[:, 1].max()) < 1e-9
    areas = [polygon_area(r) for r in rects]
    assert sum(areas) < polygon_area(_padded_bbox(ELL, 0.)) - 1., "tighter than the bbox"
    print("ok  rect: 45-deg ellipse -> 5 equal-width rectangles, areas %s m^2, "
          "union %.0f%% of the bbox, tiling exact"
          % ([int(a) for a in areas],
             100. * sum(areas) / polygon_area(_padded_bbox(ELL, 0.))))
    orects = partition(ELL, 5, mode='rect', axis='principal')
    owners = np.zeros(len(inside), dtype=int)
    for r in orects:
        owners += points_in_polygon(inside, r).astype(int)
    assert (owners == 1).all(), "oriented rectangles tile the region exactly"
    oareas = [polygon_area(r) for r in orects]
    assert sum(oareas) < sum(areas), "oriented rectangles are tighter on a diagonal region"
    for r in orects:                                   # never wider than the minor axis
        R = _principal_frame(ELL)
        q = (r - ELL.mean(axis=0)) @ R.T
        assert q[:, 1].max() - q[:, 1].min() <= 2. * 120. + 1e-6
    print("ok  rect/principal: same ellipse -> rotated rectangles, areas %s m^2, "
          "union %.0f%% of the axis-aligned union"
          % ([int(a) for a in oareas], 100. * sum(oareas) / sum(areas)))

    # -- padding ------------------------------------------------------------
    pad = pad_polygon(SQUARE, 10.)
    assert abs(polygon_area(pad) - 120. * 120.) < 1e-6, polygon_area(pad)
    assert points_in_polygon(SQUARE, pad).all(), "originals must be inside"
    # "Strictly" inside, not merely inside: the half-open rule would report a
    # vertex sitting exactly on a left/bottom edge as inside, so check the
    # distance to the padded boundary instead of the parity test.
    assert _min_edge_distance(SQUARE, pad) > 1e-6, _min_edge_distance(SQUARE, pad)
    # Elongated: scaling about the centroid would fail this badly.
    padded_long = pad_polygon(LONG, 10.)
    assert abs((padded_long[:, 0].max() - padded_long[:, 0].min()) - 420.) < 1e-6
    assert abs((padded_long[:, 1].max() - padded_long[:, 1].min()) - 70.) < 1e-6
    print("ok  pad: 100x100 +10m -> %.1f m^2; 400x50 +10m -> %.0fx%.0f (uniform metres)"
          % (polygon_area(pad),
             padded_long[:, 0].max() - padded_long[:, 0].min(),
             padded_long[:, 1].max() - padded_long[:, 1].min()))

    # Concave that still offsets cleanly: the notch must close in, not out.
    padL = pad_polygon(LSHAPE, 5.)
    assert _is_simple(padL) and polygon_area(padL) > polygon_area(LSHAPE)
    assert points_in_polygon(LSHAPE, padL).all()
    assert not points_in_polygon([[75., 75.]], padL)[0], \
        "padding must not swallow the notch"
    # A margin wider than the slot forces the documented fallback.
    padS = pad_polygon(SLOTTED, 15.)
    assert _is_simple(padS), "fallback must still be a simple ring"
    assert polygon_area(SLOTTED) <= polygon_area(padS) <= 130. * 130. + 1e-9
    assert points_in_polygon(SLOTTED, padS).all()
    # The 20 m slot is narrower than 2*margin, so a true offset closes it.
    # The raw miter instead folds the walls past each other and leaves a
    # phantom hole there; the fallback must cover it.
    assert points_in_polygon([[50., 60.]], padS)[0], \
        "a slot narrower than 2*margin must be padded shut, not inverted"
    assert not _pad_is_sane(SLOTTED, _miter_offset(_as_poly(SLOTTED), 15.), 15.), \
        "the folded offset must be rejected, not shipped"
    assert pad_polygon(SQUARE, 0.).shape == (4, 2), "margin 0 is a no-op"
    assert pad_polygon(np.array([[0., 0.], [1., 1.]]), 5.).shape[0] < 3
    print("ok  pad: concave stays simple; over-wide margin falls back (logged above)")

    # -- grid vs strips squareness -----------------------------------------
    def worst_aspect(secs):
        a = []
        for s in secs:
            w = s[:, 0].max() - s[:, 0].min()
            h = s[:, 1].max() - s[:, 1].min()
            a.append(max(w, h) / max(min(w, h), _EPS))
        return max(a)

    g = worst_aspect(partition(SQUARE, 4, mode='grid'))
    st = worst_aspect(partition(SQUARE, 4, mode='strips'))
    assert g < st, (g, st)
    assert abs(g - 1.0) < 1e-6, g
    print("ok  grid n=4 on a square: worst tile aspect %.2f vs %.2f for strips" % (g, st))

    # -- input forms --------------------------------------------------------
    # Same region expressed four ways: as given, reversed winding, with an
    # explicit closing vertex, and with an altitude column. All must partition
    # identically, because a geofence reaches this module from YAML, from ROS
    # messages and from other planners, and no caller normalises first.
    variants = {
        'ccw': LSHAPE,
        'cw': LSHAPE[::-1],
        'closed': np.vstack([LSHAPE, LSHAPE[:1]]),
        'xyz': np.hstack([LSHAPE, np.full((6, 1), 12.5)]),
        'dup': np.vstack([LSHAPE[:1], LSHAPE]),
    }
    for name, v in variants.items():
        assert abs(polygon_area(v) - 7500.) < 1e-9, name
        areas = sorted(polygon_area(x) for x in partition(v, 3))
        assert all(abs(a - 2500.) < 1e-6 for a in areas), (name, areas)
        assert points_in_polygon([[25., 75.], [75., 75.]], v).tolist() == [True, False], name
    # Larger fleets still tile.
    for n in (7, 8, 12):
        for mode in ('strips', 'grid'):
            secs = partition(LSHAPE, n, mode=mode)
            pts = sample_inside(LSHAPE, 3000)
            hits = np.zeros(pts.shape[0], dtype=int)
            for sec in secs:
                hits += points_in_polygon(pts, sec).astype(int)
            assert (hits == 1).all(), (mode, n, int((hits != 1).sum()))
            ar = np.array([polygon_area(x) for x in secs])
            assert np.max(np.abs(ar - 7500. / n)) / (7500. / n) <= 0.05, (mode, n, ar)
    print("ok  input forms (winding / closed ring / xyz / dup vertex) and n=7,8,12")

    # -- sector_for ---------------------------------------------------------
    whole = sector_for(SQUARE, 1, 0)
    assert abs(polygon_area(whole) - 10000.) < 1e-9
    a = sector_for(SQUARE, 3, 2)
    assert np.allclose(a, sector_for(SQUARE, 3, 99)), "index clamps high"
    assert np.allclose(sector_for(SQUARE, 3, 0), sector_for(SQUARE, 3, -5)), \
        "index clamps low"
    padded_sector = sector_for(SQUARE, 2, 0, margin_m=10.)
    assert abs(polygon_area(padded_sector) - 120. * 120. / 2.) < 1e-6
    assert abs(polygon_area(sector_for(SQUARE, 0, 0)) - 10000.) < 1e-9, "n=0 floors to 1"
    print("ok  sector_for: pad+partition+clamp")

    print("\nall sector.py self-tests passed")


if __name__ == '__main__':
    logging.basicConfig(level=logging.WARNING,
                        format='[%(levelname)s] %(name)s: %(message)s')
    _selftest()
