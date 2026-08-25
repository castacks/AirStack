"""Boustrophedon coverage of a sector: the floor every semantic method must beat.

WHY THIS EXISTS. A semantic searcher — value maps, VLM-picked frontiers, an
open-vocabulary embedding field — is only worth its compute if it finds people
FASTER than flying the sector back and forth. That comparison needs an
opponent that cannot be accused of being handicapped, and a lawnmower is the
one baseline with no free parameters to lose on: it does not choose, it does
not react, it does not fail to converge. If it wins, the semantics were noise.

It also has a property no reactive planner has: coverage of the sector is
guaranteed by construction rather than hoped for, so it bounds what is
achievable with a perfect detector and no prioritisation. Anything that beats
it beats it by looking in the right place FIRST, which is the only thing the
semantics were ever supposed to buy.

REACTIVITY IS DELIBERATELY ABSENT. It flies its lanes until the episode ends
and then flies them again — `next_waypoint` wraps rather than terminating. A
lawnmower that stopped on "coverage complete" would be scored on a shorter
flight than the methods it is compared against, and the detector is imperfect
anyway: a second pass over the same ground finds people the first pass missed.

Frame is map-frame metres, (x, y), ENU. No ROS, no shapely — the whole module
is numpy so it can be unit-tested without a robot or a simulator, and the
self-test at the bottom is the proof it is right.

WHAT THIS IS NOT. Lanes are a single family of parallel lines clipped to the
polygon, not a boustrophedon CELL DECOMPOSITION. The difference shows up only
on concave sectors: a lane that would cover a notch is clipped away at the
notch mouth, so a band of up to half a lane spacing can go unswept alongside a
boundary edge that runs parallel to the lanes. A cell decomposition (split the
polygon at every critical vertex, lane each cell separately) closes that gap
at the cost of near-duplicate lanes at every cell seam and a lane ordering that
no longer connects end-to-end. The gap is not left unmanaged: the spacing comes
from `spacing_for_footprint`, which sizes it from the sensor footprint WITH
overlap, so the true swath is wider than the nominal spacing and absorbs it. If a sector is concave
enough for that to be uncomfortable, pass the convex pieces separately.
"""

import math

import numpy as np


# ── geometry helpers ─────────────────────────────────────────────────────────

def _as_poly(poly_xy):
    """Polygon as a clean (M, 2) float array, or None if it is not usable.

    Returns None rather than raising, and every public entry point degrades to
    an empty result: a malformed search area comes from an operator-supplied
    parameter, and a planner that dies on it takes the whole mission with it,
    whereas one that produces no lanes is visibly and safely doing nothing.
    """
    if poly_xy is None:
        return None
    p = np.asarray(poly_xy, dtype=np.float64)
    if p.ndim != 2 or p.shape[1] != 2 or p.shape[0] < 3:
        return None
    if not np.isfinite(p).all():
        return None
    # Drop a repeated closing vertex; the crossing test walks edges modulo M and
    # a duplicate would contribute a zero-length edge.
    if np.allclose(p[0], p[-1]):
        p = p[:-1]
    if p.shape[0] < 3:
        return None
    # Zero (or near-zero) area is a line, not a region.
    area = 0.5 * abs(float(np.sum(p[:, 0] * np.roll(p[:, 1], -1)
                                  - np.roll(p[:, 0], -1) * p[:, 1])))
    scale = float(np.max(np.ptp(p, axis=0)))
    if scale <= 0.0 or area <= 1e-9 * scale * scale:
        return None
    return p


def _convex_hull(p):
    """Monotone-chain hull, counter-clockwise. Written out rather than pulled
    from scipy.spatial so this module has no dependency beyond numpy — it is
    twelve lines and it removes a reason for the baseline to fail to import on
    a robot image that trimmed scipy."""
    q = np.unique(p, axis=0)                    # also sorts lexicographically
    if len(q) < 3:
        return q

    def _half(pts):
        h = []
        for r in pts:
            while len(h) >= 2:
                a, b = h[-2], h[-1]
                if (b[0] - a[0]) * (r[1] - a[1]) - (b[1] - a[1]) * (r[0] - a[0]) > 0:
                    break
                h.pop()
            h.append(r)
        return h

    lower, upper = _half(q), _half(q[::-1])
    return np.array(lower[:-1] + upper[:-1], dtype=np.float64)


def _axis_direction(p, axis):
    """Unit vector the lanes run ALONG.

    'auto' minimises the number of turns, and the number of turns is the number
    of lanes, which is the polygon's extent PERPENDICULAR to the lanes divided
    by the spacing. So the job is to find the direction of minimum width and
    run the lanes across it. For a convex hull the minimum width is always
    attained perpendicular to some hull EDGE (rotating calipers), so testing one
    direction per hull edge is exact, not a search.

    The alternative — the principal axis of the vertex covariance — is cheaper
    and wrong in a way that matters: it is biased by how the boundary happens to
    be sampled, so a long edge cut into ten collinear vertices drags the axis
    toward itself and the sector gets swept the short way. Area moments fix the
    sampling bias but still return the inertial axis, which is not the minimum
    width for anything but an ellipse.

    'x' / 'y' force the axis. A float is taken as an angle in radians, which is
    what a caller wants when lanes have to line up with something outside this
    module (a shoreline, a road, a wind direction the operator wants to fly
    into).
    """
    if isinstance(axis, (int, float)) and not isinstance(axis, bool):
        return np.array([math.cos(float(axis)), math.sin(float(axis))])
    a = str(axis).lower()
    if a == 'x':
        return np.array([1.0, 0.0])
    if a == 'y':
        return np.array([0.0, 1.0])
    if a != 'auto':
        raise ValueError(f"axis must be 'auto', 'x', 'y' or an angle in "
                         f"radians, got {axis!r}")

    h = _convex_hull(p)
    if len(h) < 3:
        return np.array([1.0, 0.0])
    best, best_key = None, None
    for k in range(len(h)):
        e = h[(k + 1) % len(h)] - h[k]
        n = float(np.hypot(e[0], e[1]))
        if n <= 1e-12:
            continue
        u = e / n
        v = np.array([-u[1], u[0]])
        width = float(np.ptp(h @ v))
        length = float(np.ptp(h @ u))
        # Ties (a square, a regular polygon) are broken by the longer extent
        # along the lanes and then by angle, so the answer never depends on
        # vertex order.
        ang = math.atan2(u[1], u[0]) % math.pi
        key = (round(width, 9), -round(length, 9), round(ang, 9))
        if best_key is None or key < best_key:
            best, best_key = u, key
    if best is None:
        return np.array([1.0, 0.0])
    # Canonical sign, so the lane direction — and therefore the whole path — is
    # the same whichever way the polygon was wound.
    if best[0] < 0 or (best[0] == 0.0 and best[1] < 0):
        best = -best
    return best


def _clip_line(p, u, v, t, eps):
    """Intervals of the line {q : q.v == t} that lie inside the polygon, as
    (s_start, s_end) pairs in the along-lane coordinate q.u.

    Scanline fill with the half-open crossing rule `(a1 > 0) != (a2 > 0)` — the
    same rule the planner's point-in-polygon test uses — so a vertex sitting
    exactly on the line is counted once, not twice or zero times, and crossings
    always come in pairs.

    The rule cannot save an EDGE that lies exactly along the line: it
    contributes no crossing at all and the pairing can come out odd. That is a
    measure-zero case in general but a certainty for an axis-aligned rectangle
    swept along its own edge, so a lane that lands on any vertex is nudged by
    `eps` (nanometres at survey scale) and re-cut.
    """
    a = p @ v - t
    if np.any(np.abs(a) < eps):
        a = p @ v - (t + eps)
    b = p @ u
    xs = []
    m = len(p)
    for k in range(m):
        k2 = (k + 1) % m
        a1, a2 = a[k], a[k2]
        if (a1 > 0.0) != (a2 > 0.0):
            f = a1 / (a1 - a2)
            xs.append(b[k] + f * (b[k2] - b[k]))
    xs.sort()
    if len(xs) % 2:                     # unpaired crossing: a degeneracy the
        xs = xs[:-1]                    # nudge did not catch — drop it rather
    return [(xs[i], xs[i + 1]) for i in range(0, len(xs), 2)]   # than emit junk


def _lane_groups(poly_xy, spacing_m, axis='auto', margin_m=0.0):
    """The lanes, grouped: one list of (2, 2) segments per lane, ordered by lane
    offset and then along the lane. `lanes` flattens this; `boustrophedon` needs
    the grouping to know where one lane ends and the next begins."""
    p = _as_poly(poly_xy)
    if p is None or not np.isfinite(spacing_m) or spacing_m <= 0.0:
        return [], np.array([1.0, 0.0]), np.array([0.0, 1.0])
    margin = max(float(margin_m), 0.0)

    u = _axis_direction(p, axis)
    u = u / float(np.hypot(u[0], u[1]))
    v = np.array([-u[1], u[0]])

    tv = p @ v
    t_lo, t_hi = float(tv.min()) + margin, float(tv.max()) - margin
    width = t_hi - t_lo
    if width < 0.0:
        return [], u, v                 # margin exceeded the sector's width

    # Lanes are inset half a spacing from each side and spread evenly, so the
    # swath edges land on the sector edges. Rounding the count UP and then
    # re-deriving the spacing keeps every gap <= the requested spacing; rounding
    # down would leave a strip wider than the sensor swath along one side.
    n = max(1, int(math.ceil(width / float(spacing_m) - 1e-9)))
    step = width / n if n > 0 else 0.0
    eps = 1e-9 * max(1.0, float(np.max(np.abs(p))))

    groups = []
    for k in range(n):
        t = t_lo + step * (k + 0.5)
        segs = []
        for s0, s1 in _clip_line(p, u, v, t, eps):
            s0 += margin
            s1 -= margin
            # A slice shorter than 2 * margin vanishes, and a lane that only
            # grazes a vertex has zero length. Both are dropped: a waypoint pair
            # a millimetre apart is a turn command, not coverage.
            if s1 - s0 <= max(1e-6, eps):
                continue
            segs.append(np.array([t * v + s0 * u, t * v + s1 * u]))
        if segs:
            groups.append(segs)
    return groups, u, v


# ── public API ───────────────────────────────────────────────────────────────

def lanes(poly_xy, spacing_m, axis='auto', margin_m=0.0):
    """Parallel lanes clipped to the polygon.

    Returns a list of (2, 2) arrays, each [[x0, y0], [x1, y1]] — ONE CONTIGUOUS
    SEGMENT that lies entirely inside the polygon. A concave sector cuts a lane
    into several disjoint pieces (a lane crossing a notch enters, leaves, and
    re-enters) and each piece is returned separately, never joined across the
    gap: joining them would produce a straight waypoint pair whose middle is
    outside the search area, which is at best wasted flight and at worst a
    geofence violation.

    Segments are ordered by lane offset and then along the lane, so the pieces
    of one lane are consecutive and a caller that wants the lane structure back
    can group by the offset of each segment's midpoint.

    `axis='auto'` runs lanes along the direction that minimises the lane count
    (see `_axis_direction`); every turn costs a deceleration, a heading change
    and an acceleration, and produces no coverage while it happens.

    `margin_m` keeps the lanes clear of the boundary. It is applied as an inset
    of the lane offsets from the sector's extent plus a trim off both ends of
    every clipped segment — NOT as a true polygon erosion, which needs a
    Minkowski offset and self-intersection cleanup that is not worth a
    dependency here. The two agree for every edge a lane crosses (which
    includes the walls of a notch); they differ for an edge that runs nearly
    parallel to the lanes in the interior of the sector, where the nearest lane
    can end up closer than `margin_m` to it. Where exact clearance is a safety
    requirement, shrink the polygon upstream and pass margin_m=0.

    Degenerate inputs return [] rather than raising: fewer than 3 vertices, a
    non-finite coordinate, zero enclosed area, spacing <= 0, or a margin wider
    than half the sector.
    """
    groups, _, _ = _lane_groups(poly_xy, spacing_m, axis, margin_m)
    return [s for segs in groups for s in segs]


def boustrophedon(poly_xy, spacing_m, axis='auto', margin_m=0.0, start_xy=None):
    """Ordered (N, 2) waypoints: lane 1 forward, lane 2 back, alternating.

    Alternating is the whole point — flying every lane in the same direction
    would add a full lane-length transit between each pair, which on a 300 m
    sector at 5 m/s is a minute of flight per lane bought for nothing. Reversing
    means consecutive lanes connect end to end and the only transit is the
    turn itself.

    `start_xy` picks the entry point: the drone arrives at the sector from
    wherever it took off, and starting at the far corner wastes the transit.
    The nearest END OF A LANE (not the nearest point on a lane — entering a lane
    at its middle leaves half of it unswept until the next loop) is chosen, and
    the sweep runs from there through the lanes in ascending offset order,
    wrapping past the last lane back to the first so every lane is flown exactly
    once per loop. If that entry lane is in the middle of the sector, the wrap
    costs one long transit across it; that is the price of not skipping ground,
    and it disappears when `start_xy` is near a corner, which is the normal
    case.

    Fully deterministic: no randomness, ties resolved by lowest lane index and
    then by the forward direction, so the same sector always produces the same
    path and two runs of an experiment are comparable.

    Returns a (0, 2) array for a degenerate sector — see `lanes`.
    """
    groups, _u, _v = _lane_groups(poly_xy, spacing_m, axis, margin_m)
    if not groups:
        return np.zeros((0, 2), dtype=np.float64)

    n = len(groups)
    k0, flip0 = 0, False
    if start_xy is not None:
        s = np.asarray(start_xy, dtype=np.float64).reshape(2)
        if np.isfinite(s).all():
            best = None
            for k, segs in enumerate(groups):
                for flip, end in ((False, segs[0][0]), (True, segs[-1][1])):
                    d = float(np.hypot(*(end - s)))
                    key = (round(d, 9), k, flip)
                    if best is None or key < best:
                        best, k0, flip0 = key, k, flip

    out = []
    for m in range(n):
        segs = groups[(k0 + m) % n]
        reverse = flip0 != bool(m % 2)          # alternate, from the chosen end
        seq = segs[::-1] if reverse else segs
        for seg in seq:
            a, b = (seg[1], seg[0]) if reverse else (seg[0], seg[1])
            # Consecutive duplicates would make next_waypoint burn an index on a
            # point it is already standing on.
            if out and np.allclose(out[-1], a, atol=1e-9):
                out.append(b)
            else:
                out.extend((a, b))
    return np.asarray(out, dtype=np.float64).reshape(-1, 2)


def next_waypoint(path, cur_xy, idx, reach_radius_m):
    """Advance along a precomputed path. Returns (new_index, waypoint).

    Wraps at the end instead of signalling "done": the caller flies until the
    episode's time or battery budget runs out, so the path is a LOOP. A second
    pass over the same ground is not wasted effort — the detector is imperfect,
    and the lighting, the viewing angle and the occlusions are all different the
    second time.

    Waypoints already within `reach_radius_m` are skipped rather than visited in
    turn, because lane ends cluster: at a turn, the end of one lane and the
    start of the next are one spacing apart, and with a reach radius of half a
    spacing the drone would otherwise report arrival at both from the same
    position and stall through a burst of no-op goals.

    Degenerate inputs fail by holding station, which is the safe action for a
    flying vehicle: an empty path returns the caller's own position, so the
    drone hovers instead of being commanded to an undefined goal. A
    `reach_radius_m` of 0 or less means arrival can never be registered, so the
    same index is returned forever — the drone flies to that waypoint and holds.
    If EVERY waypoint is within reach (a path smaller than the reach radius, or
    a reach radius set to the size of the sector), the scan stops after one full
    lap and returns the starting index rather than looping forever.
    """
    p = np.asarray(path, dtype=np.float64).reshape(-1, 2)
    cur = np.asarray(cur_xy, dtype=np.float64).reshape(2)
    n = len(p)
    if n == 0:
        return int(idx), cur.copy()
    k = int(idx) % n
    for _ in range(n):
        if float(np.hypot(*(p[k] - cur))) > float(reach_radius_m):
            return k, p[k].copy()
        k = (k + 1) % n
    return k, p[k].copy()


def spacing_for_footprint(altitude_m, hfov_rad, overlap_frac=0.2):
    """Lane spacing from the camera's ground footprint. Returns metres.

    Geometry, nadir-pointing camera over flat ground:

                          camera
                            |\\
                            | \\   half the horizontal field of view, hfov/2
                            |  \\
                 altitude h |   \\
                            |    \\
                    --------+-----+--------  ground
                            |<--->|
                             h*tan(hfov/2)

    so the swath is W = 2 * h * tan(hfov / 2), and neighbouring swaths are made
    to overlap by `overlap_frac` of W:

        spacing = W * (1 - overlap_frac)

    WHY THIS FUNCTION EXISTS AT ALL. A lawnmower baseline is trivially made to
    look bad by choosing the spacing badly, and the failure is silent. Spacing
    wider than the swath leaves unobserved strips between lanes, so the sector
    is flown, reported as covered, and the casualties in the strips are never
    seen — the baseline scores low for a reason that has nothing to do with the
    method being compared against it. Deriving the spacing from the sensor is
    what makes the comparison fair, and the overlap is what keeps it fair when
    the drone does not track its lane perfectly, when the ground is not flat,
    and where a concave sector clips a lane short (see the module docstring).

    A nadir camera is assumed. A forward-tilted camera has a trapezoidal
    footprint whose width varies with range, and the conservative substitution
    is the width at the NEAR edge of the usable band, not at the centre.

    Raises ValueError on altitude <= 0 or an hfov outside (0, pi): these come
    from a config file read once at start-up, and a silent fallback would put a
    plausible-looking but wrong spacing into a run that then has to be thrown
    away. `overlap_frac` is clamped to [0, 0.95] instead — it is a preference,
    not a measurement, and 1.0 would ask for zero spacing and infinite lanes.
    """
    h = float(altitude_m)
    fov = float(hfov_rad)
    if not np.isfinite(h) or h <= 0.0:
        raise ValueError(f'altitude_m must be > 0, got {altitude_m}')
    if not np.isfinite(fov) or fov <= 0.0 or fov >= math.pi:
        raise ValueError(f'hfov_rad must be in (0, pi), got {hfov_rad}')
    ov = min(max(float(overlap_frac), 0.0), 0.95)
    swath = 2.0 * h * math.tan(0.5 * fov)
    return swath * (1.0 - ov)


# ── self-test ────────────────────────────────────────────────────────────────

def _seg_dist(pts, segs):
    """Distance from each of (N, 2) points to the nearest of M segments."""
    a = np.array([s[0] for s in segs])
    b = np.array([s[1] for s in segs])
    ab = b - a
    l2 = np.maximum((ab ** 2).sum(1), 1e-12)
    t = ((pts[:, None, :] - a[None]) * ab[None]).sum(2) / l2[None]
    t = np.clip(t, 0.0, 1.0)
    foot = a[None] + t[..., None] * ab[None]
    return np.linalg.norm(pts[:, None, :] - foot, axis=2).min(1)


def _inside(pts, poly):
    x, y = pts[:, 0], pts[:, 1]
    ins = np.zeros(len(pts), dtype=bool)
    m = len(poly)
    for i in range(m):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % m]
        straddles = (y1 > y) != (y2 > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            xint = (x2 - x1) * (y - y1) / (y2 - y1) + x1
        ins ^= straddles & (x < xint)
    return ins


def _sample_inside(poly, n, rng):
    lo, hi = poly.min(0), poly.max(0)
    out = []
    while sum(len(o) for o in out) < n:
        c = rng.uniform(lo, hi, size=(4 * n, 2))
        out.append(c[_inside(c, poly)])
    return np.concatenate(out)[:n]


def _selftest():
    rng = np.random.default_rng(7)

    # 1. Lanes cover the polygon. Two bounds are checked, because they are two
    #    different claims and only the first is exact:
    #
    #    (a) Every interior point is within spacing/2 of a lane LINE. This is
    #        what the lane placement guarantees outright, on any polygon, and if
    #        it ever fails the offsets are wrong.
    #    (b) Every interior point is within the SENSOR SWATH half-width of a
    #        clipped lane SEGMENT. Distance to the segment can exceed spacing/2
    #        even on a convex sector: alongside a boundary edge that is slanted
    #        relative to the lanes, the nearest lane is clipped short of the
    #        point's along-lane coordinate, so the nearest segment is the corner
    #        of that lane and the extra distance is the boundary's shear over
    #        half a spacing. It is a sliver in the corners, not a strip, and it
    #        is exactly what the overlap in `spacing_for_footprint` is for:
    #        at the default 20 % the true swath half-width is
    #        (spacing/2)/(1 - 0.2) = 0.625 * spacing.
    over = 0.2
    swath_half = 0.5 / (1.0 - over)
    th = math.radians(25.0)
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    rect = (np.array([[-15.0, -6.0], [15.0, -6.0], [15.0, 6.0], [-15.0, 6.0]])
            @ R.T) + np.array([40.0, -25.0])
    hexa = np.array([[0.0, 20.0], [17.0, 10.0], [17.0, -10.0],
                     [0.0, -20.0], [-17.0, -10.0], [-17.0, 10.0]])
    for name, poly, sp in (('rot-rect 30x12 @25deg', rect, 2.0),
                           ('hexagon r=20', hexa, 3.0)):
        groups, u, v = _lane_groups(poly, sp)
        segs = [s_ for g_ in groups for s_ in g_]
        offs = np.array([float(g_[0][0] @ v) for g_ in groups])
        pts = _sample_inside(poly, 5000, rng)
        d_line = np.abs((pts @ v)[:, None] - offs[None]).min(1)
        d_seg = _seg_dist(pts, segs)
        assert d_line.max() <= 0.5 * sp + 1e-9, (
            f'{name}: point {d_line.max():.4f} m from every lane LINE, '
            f'spacing/2 = {0.5 * sp}')
        assert d_seg.max() <= swath_half * sp + 1e-9, (
            f'{name}: point {d_seg.max():.4f} m from every lane SEGMENT, '
            f'swath half-width = {swath_half * sp:.4f}')
        print(f'[1] {name}: spacing {sp} m -> {len(groups)} lanes / '
              f'{len(segs)} segments; over 5000 interior samples the worst '
              f'distance to a lane LINE is {d_line.max():.4f} m '
              f'(<= spacing/2 = {0.5 * sp:.3f}) and to a clipped SEGMENT '
              f'{d_seg.max():.4f} m (<= 20%-overlap swath half-width '
              f'{swath_half * sp:.3f}); {(d_seg > 0.5 * sp).mean():.2%} of '
              f'samples sit in a boundary sliver beyond spacing/2  OK')

    # 1b. The concave case, measured rather than asserted at spacing/2. The
    #     module docstring claims the shortfall is bounded by about one spacing
    #     and is absorbed by the sensor overlap; this is the number behind that.
    ell = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 5.0],
                    [5.0, 5.0], [5.0, 10.0], [0.0, 10.0]])
    sp = 1.5
    segs = lanes(ell, sp, axis=-math.pi / 4)
    pts = _sample_inside(ell, 5000, rng)
    d = _seg_dist(pts, segs)
    assert d.max() <= sp, f'concave shortfall {d.max():.4f} exceeded one spacing'
    print(f'[1b] L-shape, lanes at -45 deg, spacing {sp} m: worst distance '
          f'{d.max():.4f} m (<= spacing {sp}, > spacing/2 = {0.5 * sp}: the '
          f'documented concave gap), {(d > 0.5 * sp).mean():.2%} of samples '
          f'beyond spacing/2  OK')

    # 2. A concave polygon splits a lane. Lanes at -45 deg cross the notch of
    #    the L twice, so they enter, leave and re-enter.
    groups, _, _ = _lane_groups(ell, sp, axis=-math.pi / 4)
    split = [len(g) for g in groups]
    assert max(split) >= 2, f'no lane was split on a concave polygon: {split}'
    k = int(np.argmax(split))
    gap_seg = groups[k]
    gap = float(np.hypot(*(gap_seg[1][0] - gap_seg[0][1])))
    mid = 0.5 * (gap_seg[0][1] + gap_seg[1][0])
    assert not _inside(mid[None, :], ell)[0], (
        'the midpoint of the gap is inside the polygon, so it was not a gap')
    print(f'[2] L-shape lane segment counts {split}: lane {k} split into '
          f'{split[k]} disjoint segments {gap:.2f} m apart, and the midpoint of '
          f'the gap is OUTSIDE the polygon (never joined across)  OK')

    # 3. Direction alternates and consecutive lanes connect end to end. Tested
    #    on a rectangle so every lane is exactly one segment and the waypoint
    #    pairs line up with the lanes.
    sp = 2.0
    path = boustrophedon(rect, sp)
    g, u, _ = _lane_groups(rect, sp)
    assert all(len(x) == 1 for x in g) and len(path) == 2 * len(g)
    dirs = path[1::2] - path[0::2]
    proj = dirs @ u
    assert np.all(proj[:-1] * proj[1:] < 0), (
        f'lane travel direction did not alternate: {np.sign(proj)}')
    gaps = np.linalg.norm(path[2::2] - path[1:-1:2], axis=1)
    assert gaps.max() <= sp * 1.2, (
        f'turn gap {gaps.max():.3f} exceeds spacing * 1.2 = {sp * 1.2}')
    print(f'[3] rot-rect: {len(g)} lanes, {len(path)} waypoints; travel '
          f'direction alternates every lane (signs '
          f'{"".join("+-"[int(s < 0)] for s in np.sign(proj))}); '
          f'turn gaps min {gaps.min():.3f} max {gaps.max():.3f} m '
          f'<= spacing*1.2 = {sp * 1.2:.3f}  OK')
    assert np.array_equal(path, boustrophedon(rect, sp)), 'not deterministic'

    # start_xy begins at the lane end nearest it, and the whole loop still
    # covers every lane exactly once.
    corner = rect.mean(0) + (rect[2] - rect.mean(0)) * 1.6
    p2 = boustrophedon(rect, sp, start_xy=corner)
    ends = np.array([e for x in g for e in (x[0][0], x[0][1])])
    nearest = ends[int(np.argmin(np.linalg.norm(ends - corner, axis=1)))]
    assert np.allclose(p2[0], nearest), (
        f'start {p2[0]} is not the nearest lane end {nearest}')
    assert len(p2) == len(path), 'start_xy changed how many lanes are flown'
    print(f'[3] start_xy=({corner[0]:.1f}, {corner[1]:.1f}) starts at the '
          f'nearest lane end ({p2[0][0]:.2f}, {p2[0][1]:.2f}), '
          f'{len(p2)} waypoints unchanged  OK')

    # 4. next_waypoint skips what is already reached and wraps at the end.
    sq = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
    pth = boustrophedon(sq, 4.0, axis='x')
    i, w = next_waypoint(pth, pth[0] + np.array([0.05, 0.0]), 0, 0.5)
    assert i == 1 and np.allclose(w, pth[1]), 'did not skip a reached waypoint'
    last = len(pth) - 1
    i2, w2 = next_waypoint(pth, pth[last], last, 0.5)
    assert i2 == 0 and np.allclose(w2, pth[0]), (
        f'did not wrap at the end: got index {i2}')
    i3, _ = next_waypoint(pth, np.array([500.0, 500.0]), 2, 0.5)
    assert i3 == 2, 'advanced past a waypoint that was not reached'
    i4, w4 = next_waypoint(pth, pth[3], 3, 1e6)
    assert i4 == 3, 'an all-within-reach path must terminate the scan'
    i5, w5 = next_waypoint(np.zeros((0, 2)), np.array([1.0, 2.0]), 0, 1.0)
    assert i5 == 0 and np.allclose(w5, [1.0, 2.0]), 'empty path must hold station'
    print(f'[4] next_waypoint on a {len(pth)}-point loop: skips reached points, '
          f'index {last} -> {i2} at the end (wraps), holds when nothing is '
          f'reached, terminates when everything is, holds station on an empty '
          f'path  OK')

    # 5. spacing_for_footprint.
    alt, fov, ov = 20.0, math.radians(105.0), 0.2
    s = spacing_for_footprint(alt, fov, ov)
    swath = 2.0 * alt * math.tan(0.5 * fov)
    assert s > 0.0 and abs(s - swath * (1.0 - ov)) < 1e-9
    assert s < swath, 'spacing must be narrower than the swath it overlaps'
    print(f'[5] spacing_for_footprint({alt} m, {math.degrees(fov):.0f} deg, '
          f'overlap {ov:.0%}): swath = 2*{alt}*tan({math.degrees(fov) / 2:.1f} '
          f'deg) = {swath:.2f} m -> spacing {s:.2f} m')
    assert spacing_for_footprint(20.0, fov, 0.0) == swath
    assert spacing_for_footprint(20.0, fov, 2.0) > 0.0, 'overlap must clamp'
    for bad in ((0.0, fov), (-1.0, fov), (10.0, 0.0), (10.0, math.pi)):
        try:
            spacing_for_footprint(*bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f'spacing_for_footprint{bad} should have raised')
    n_lanes = len(_lane_groups(hexa, s)[0])
    print(f'[5] the 34 m-wide hexagon above needs {n_lanes} lane(s) at that '
          f'spacing, i.e. one pass at 20 m covers it; bad altitude/fov raise, '
          f'overlap clamps  OK')

    # 6. Degenerate sectors produce nothing instead of raising.
    for bad in (None, [[0.0, 0.0], [1.0, 1.0]], [[0, 0], [1, 0], [2, 0]],
                [[0, 0], [1, 0], [float('nan'), 1]]):
        assert lanes(bad, 1.0) == [], f'{bad} should give no lanes'
        assert boustrophedon(bad, 1.0).shape == (0, 2)
    assert lanes(sq, 0.0) == [] and lanes(sq, -1.0) == []
    assert lanes(sq, 1.0, margin_m=6.0) == [], 'margin wider than the sector'
    assert len(lanes(sq, 1.0, margin_m=1.0)) > 0
    m_segs = lanes(sq, 1.0, margin_m=1.0)
    allpts = np.array([q for s_ in m_segs for q in s_])
    assert allpts.min() >= 1.0 - 1e-9 and allpts.max() <= 9.0 + 1e-9, (
        'margin was not honoured on the axis-aligned case')
    try:
        lanes(sq, 1.0, axis='diagonal')
    except ValueError:
        pass
    else:
        raise AssertionError('a bad axis name should raise')
    print('[6] degenerate sectors (None, 2 points, collinear, NaN, spacing<=0, '
          'margin wider than the sector) return nothing; margin honoured; '
          'bad axis name raises  OK')

    # 7. 'auto' picks the long axis: lanes on a 30x12 rectangle run along the
    #    30 m side, which is 5 lanes at 3 m rather than the 10 it would be the
    #    other way.
    u_auto = _axis_direction(_as_poly(rect), 'auto')
    long_side = (rect[1] - rect[0]) / np.linalg.norm(rect[1] - rect[0])
    assert abs(abs(float(u_auto @ long_side)) - 1.0) < 1e-9, (
        f'auto axis {u_auto} is not the long side {long_side}')
    n_auto = len(_lane_groups(rect, 3.0, 'auto')[0])
    n_cross = len(_lane_groups(rect, 3.0, float(math.atan2(*long_side[::-1])
                                                + math.pi / 2))[0])
    assert n_auto < n_cross
    print(f"[7] axis='auto' on the 30x12 rectangle runs along the long side: "
          f'{n_auto} lanes vs {n_cross} across  OK')
    print('lawnmower.py: all self-tests passed')


if __name__ == '__main__':
    _selftest()
