"""
suburb_park.py — layout for a large suburban park.

WHAT THIS OWNS
--------------
The park is not a block with trees on it. It is a set of FACILITIES at real
dimensions, a path network that reaches each one, and the leftover green
between them — and the facilities are what fix the sizes, not the other way
round. A basketball court is 28.65 x 15.24 m whatever the park wants, so the
layout packs to those figures and lets the open lawn take the remainder.

Sourced assets cover the objects; the SURFACES are generated, because a court
is a painted rectangle and no asset library has the one that fits your park:

    from assets   hoop, fence panel, gazebo, fountain, park sign, picnic
                  table, swing set / play structure / seesaw, tennis court
    generated     court slabs and their line markings, soccer pitch and its
                  markings, playground sand, path network, fence runs

DIMENSIONS, WHICH ARE NOT NEGOTIABLE
------------------------------------
    basketball   28.65 x 15.24 m court (94 x 50 ft), hoop 1.575 m in from the
                 baseline, 3-point arc 6.75 m, key 4.88 x 5.79 m
    tennis       23.77 x 10.97 m doubles court, 8.23 m singles width, service
                 line 6.40 m from the net; fenced enclosure 36.58 x 18.29 m
    soccer       park pitch, inside the 90-120 x 45-90 m the Laws allow:
                 100 x 64 m with a 9.15 m centre circle, 16.5 m penalty area
                 and 5.5 m goal area

The tennis COURT is an asset here rather than generated, because a full court
was available; its markings therefore come with it and this module only places
the enclosure and its fence.

LAYOUT IS COMPOSED, NOT PACKED
------------------------------
Each facility names the position it wants as a fraction of the park, and only
slides if something is already there. A shelf packer was tried first and is
what a park emphatically is not: it filled the southern strip and left a
single 200 x 130 m slab of nothing above it, with the picnic grounds stranded
in the corner furthest from the courts they serve.

The composition: active sport along the south edge, where it can be fenced and
lit without facing houses; passive green to the north; playground and picnic
BETWEEN them and hard against the courts, because that is where parents stand.

The lawn is not a zone. It is whatever grass no facility covers, which is why
trees are rejection-sampled against the facility rects instead of being
confined to a lawn rectangle.
"""

import math

import suburb_net as sn

# ---------------------------------------------------------------------------
# regulation dimensions (metres)
# ---------------------------------------------------------------------------

BASKETBALL = {
    "court": (28.65, 15.24),      # 94 x 50 ft
    "pad": 2.0,                   # run-off outside the lines
    "hoop_inset": 1.575,          # centre of ring from the baseline
    "key": (4.88, 5.79),          # width x length of the lane
    "circle_r": 1.80,
    "three_r": 6.75,
    "three_straight": 0.90,       # arc's straight section from the sideline
}

TENNIS = {
    "court": (23.77, 10.97),      # doubles
    "singles_w": 8.23,
    "service": 6.40,              # service line from the net
    "enclosure": (36.58, 18.29),
}

SOCCER = {
    "pitch": (100.0, 64.0),
    "circle_r": 9.15,
    "penalty": (16.5, 40.32),     # depth x width
    "goal_area": (5.5, 18.32),
    "penalty_spot": 11.0,
    "goal_w": 7.32,
}

DEFAULTS = {
    "region_m": [420.0, 300.0],
    "edge_buffer_m": 12.0,        # tree belt inside the park boundary
    # DERIVED, not chosen. A shared path threading between two facilities needs
    # both their padding bands plus its own width: 2 x 8 + 4.6 = 20.6 m at the
    # fenced compounds. At the old flat 9.0 the corridors were 9.2-13.9 m and
    # the path physically could not fit, so the router was being asked to solve
    # an impossible problem and left 17 padding breaches that no amount of
    # detour tuning could remove. Widening the gap at PLACEMENT time makes the
    # corridors passable by construction; see facility_gap() below.
    "facility_gap_m": 0.0,      # 0 = derive from the padding and path width
    # Between COURTS inside one compound -- they share a slab and a fence, so
    # this is a couple of metres of sideline run-off, not a path corridor.
    "court_gap_m": 2.5,
    # The pitches sit side by side but must READ as two. At the bare facility
    # gap they abut into one 138 m slab of green with two sets of markings on
    # it, which is not a pair of pitches, it is a drawing error. A real ground
    # leaves a walkable margin between pitches and puts a path down it.
    "pitch_gap_m": 30.0,
    "n_basketball": 4,
    "n_tennis": 3,
    "n_soccer": 2,
    "playground_m": [46.0, 34.0],
    "picnic_areas": 3,
    "picnic_m": [30.0, 22.0],
    "picnic_tables": [5, 9],
    "gazebos": [2, 4],
    "fountains": [2, 3],
    "path_w_m": 3.0,
    "loop_inset_m": 7.0,
    "fence_panel_m": 2.4,         # the sourced chain-link section's run
    "path_clear_m": 5.0,     # a path keeps this far off a set piece
    # CLEARANCE BAND ROUND A FACILITY. Not crossing a court was never the whole
    # requirement — a path that merely grazes the fence is still wrong. A real
    # park leaves grass between the two, because nobody wants to walk with a
    # shoulder on the chain-link and because the fence has to stand somewhere.
    #
    # This band is measured from the path's EDGE, not its centreline. The
    # shared-use way is 4.6 m wide and the cycle side is on the OUTSIDE of it,
    # so a centreline figure quietly under-measures the real clearance by half
    # the width and puts the bikes in the margin the band was meant to keep.
    "facility_pad_m": 6.0,
    # PER KIND, and the extra on a fenced compound is not decoration. The
    # chain-link run stands ON the boundary line, its gate swings outward, and
    # the bike racks are parked 4.5 m off the same face (see
    # `bike_racks_per_court`, placed against the compound below). 6 m would run
    # the path straight through the racks; 8 m clears them with a metre and a
    # half of grass left over. An open picnic ground or a pitch has nothing
    # standing on its edge, so the base band is enough there.
    "facility_pad_fenced_m": 8.0,
    # Shared-use path: one way, divided. The spine carries both modes; a spur
    # into a single facility does not need a cycle side.
    "path_w_shared_m": 4.6,   # 2.6 m walking + 2.0 m cycle side
    "path_w_spur_m": 2.6,
    "bike_share": 0.44,
    "bike_racks_per_court": 4,       # share of the shared path given to the cycle side
    "copses": 9,
    "copse_r_m": [26.0, 55.0],
    "lawn_tree_per_1000m2": 1.1,
    "bench_spacing_m": 34.0,      # along a path, alternating sides
    "trash_spacing_m": 78.0,
    "furniture_offset_m": 2.9,    # bench/bin set back from the path centreline
}


def _rng_pair(v, fallback):
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    if isinstance(v, (int, float)):
        return float(v), float(v)
    return fallback


# ---------------------------------------------------------------------------
# court markings — line lists in LOCAL coords, centred on the facility
# ---------------------------------------------------------------------------

def _arc(cx, cy, r, a0, a1, n=28):
    return [(cx + r * math.cos(a0 + (a1 - a0) * k / n),
             cy + r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]


def basketball_markings():
    """Full-court lines, origin at centre, long axis +x."""
    L, W = BASKETBALL["court"]
    hl, hw = L / 2.0, W / 2.0
    kw, kl = BASKETBALL["key"]
    r3, cr = BASKETBALL["three_r"], BASKETBALL["circle_r"]
    inset = BASKETBALL["hoop_inset"]
    out = [
        [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw), (-hl, -hw)],   # outline
        [(0, -hw), (0, hw)],                                        # halfway
        _arc(0, 0, cr, 0, 2 * math.pi),                             # centre
    ]
    for s in (-1.0, 1.0):
        base = s * hl
        hoop = base - s * inset
        # key + free-throw circle
        out.append([(base, -kw / 2), (base - s * kl, -kw / 2),
                    (base - s * kl, kw / 2), (base, kw / 2)])
        out.append(_arc(base - s * kl, 0, cr, 0, 2 * math.pi))
        # three-point line: straights from the baseline, joined by the arc
        y = hw - BASKETBALL["three_straight"]
        dx = math.sqrt(max(0.0, r3 ** 2 - y ** 2))
        tip = hoop - s * dx                 # where each straight meets the arc
        out.append([(base, -y), (tip, -y)])
        out.append([(base, y), (tip, y)])
        # The arc opens TOWARD centre court, so it is centred on the direction
        # -s in x (pi at the +x end, 0 at the -x end) and spans +-half either
        # side of that. Written per-end with a conditional it went the other
        # way and swept through the baseline instead, putting a 5 m bulge of
        # three-point line outside the court at each end.
        mid = math.atan2(0.0, -s)
        half = math.atan2(y, dx)
        out.append(_arc(hoop, 0, r3, mid - half, mid + half))
    return out


def tennis_markings():
    L, W = TENNIS["court"]
    hl, hw = L / 2.0, W / 2.0
    sh = TENNIS["singles_w"] / 2.0
    sv = TENNIS["service"]
    return [
        [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw), (-hl, -hw)],
        [(-hl, -sh), (hl, -sh)], [(-hl, sh), (hl, sh)],   # singles sidelines
        [(0, -hw), (0, hw)],                              # net
        [(-sv, -sh), (-sv, sh)], [(sv, -sh), (sv, sh)],   # service lines
        [(-sv, 0), (sv, 0)],                              # centre service line
    ]


def soccer_markings():
    L, W = SOCCER["pitch"]
    hl, hw = L / 2.0, W / 2.0
    pd, pw = SOCCER["penalty"]
    gd, gw = SOCCER["goal_area"]
    out = [
        [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw), (-hl, -hw)],
        [(0, -hw), (0, hw)],
        _arc(0, 0, SOCCER["circle_r"], 0, 2 * math.pi),
    ]
    for s in (-1.0, 1.0):
        base = s * hl
        out.append([(base, -pw / 2), (base - s * pd, -pw / 2),
                    (base - s * pd, pw / 2), (base, pw / 2)])
        out.append([(base, -gw / 2), (base - s * gd, -gw / 2),
                    (base - s * gd, gw / 2), (base, gw / 2)])
    return out


# ---------------------------------------------------------------------------
# shelf packing
# ---------------------------------------------------------------------------

def _obb(cx, cy, w, h, yaw_deg):
    """Corner list of an oriented box, CCW."""
    a = math.radians(yaw_deg)
    ux, uy = math.cos(a), math.sin(a)
    vx, vy = -uy, ux
    hw, hh = w / 2.0, h / 2.0
    return [(cx + ux * hw + vx * hh, cy + uy * hw + vy * hh),
            (cx - ux * hw + vx * hh, cy - uy * hw + vy * hh),
            (cx - ux * hw - vx * hh, cy - uy * hw - vy * hh),
            (cx + ux * hw - vx * hh, cy + uy * hw - vy * hh)]


def _sat_overlap(a, b, pad=0.0):
    """Separating-axis test between two oriented boxes."""
    for poly in (a, b):
        for i in range(2):
            ex = poly[(i + 1) % 4][0] - poly[i][0]
            ey = poly[(i + 1) % 4][1] - poly[i][1]
            n = sn._unit((-ey, ex))
            amin = min(sn._dot(n, p) for p in a)
            amax = max(sn._dot(n, p) for p in a)
            bmin = min(sn._dot(n, p) for p in b)
            bmax = max(sn._dot(n, p) for p in b)
            if amax + pad < bmin or bmax + pad < amin:
                return False
    return True


def _inside(corners, inner):
    return all(inner[0] <= x <= inner[2] and inner[1] <= y <= inner[3]
               for (x, y) in corners)


def _place(inner, placed, gap, w, h, fx, fy, yaw, rng):
    """Oriented placement at (fx, fy), spiralling outward until it fits.

    FACILITIES ARE ROTATED, and that is what stops the park reading as a grid.
    Axis-aligned rects gave a plan that was unmistakably generated: every pitch,
    court and picnic ground parallel to every other and to the boundary, which
    no real park is. Rotation means overlap has to be a separating-axis test on
    oriented boxes rather than an interval compare — an AABB around a rotated
    pitch is up to twice its true area and would refuse placements that are
    perfectly fine.
    """
    x0, y0, x1, y1 = inner
    cx0 = x0 + (x1 - x0) * fx
    cy0 = y0 + (y1 - y0) * fy
    step = max(5.0, gap)
    for ring in range(0, 46):
        for ang in range(0, 360, 24 if ring else 360):
            d = ring * step
            cx = cx0 + d * math.cos(math.radians(ang))
            cy = cy0 + d * math.sin(math.radians(ang))
            for dy in ((0.0,) if ring < 6 else (0.0, -12.0, 12.0)):
                c = _obb(cx, cy, w, h, yaw + dy)
                if not _inside(c, inner):
                    continue
                if any(_sat_overlap(c, q, gap) for q in placed):
                    continue
                return (cx, cy), yaw + dy, c
    return None, None, None


def _nearest_on_obb(q, corners):
    """Closest point on an oriented box's boundary, and whether q is inside."""
    inside = True
    best, bd = corners[0], 1e18
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        d = sn._sub(b, a)
        L2 = sn._dot(d, d) or 1.0
        t = max(0.0, min(1.0, sn._dot(sn._sub(q, a), d) / L2))
        pt = sn._add(a, sn._mul(d, t))
        dd = sn._dist(q, pt)
        if dd < bd:
            bd, best = dd, pt
        # CCW box: outside if the point is right of any edge
        if sn._cross(d, sn._sub(q, a)) < 0.0:
            inside = False
    return best, inside, bd


def _clear_list(obstacles, clear):
    """Broadcast *clear* over *obstacles* — a scalar, or one figure per box.

    THE CLEARANCE IS PER OBSTACLE. It has to be: the band a path keeps is a
    property of the facility (a fenced compound owns more of its surroundings
    than a picnic lawn does) and of the path's own width, so one number for the
    whole scene either pads everything to the widest case — which shoves the
    spine into the trees — or pads nothing enough. Every router entry point
    takes either form so the callers that do not care can keep passing a float.
    """
    if isinstance(clear, (int, float)):
        return [float(clear)] * len(obstacles)
    out = [float(x) for x in clear]
    if len(out) != len(obstacles):
        raise ValueError("clearance list must be parallel to the obstacles")
    return out


def avoid(pts, obstacles, clear, exempt=(), iters=4, ends=True):
    """Push individual path POINTS out of every facility they land in.

    THIS IS A PRE-PASS, NOT THE ROUTING. On its own it is exactly as wrong as
    it sounds and was the original bug: two consecutive sample points can both
    sit outside a court while the straight segment between them goes clean over
    the net, so a path whose every point passes this test still crossed the
    tennis block. Segment-level work is `route()` below; this only exists to
    lift a WAYPOINT that was dropped inside a facility, so the router starts
    from endpoints that are legal.

    *exempt* is the facility a spur belongs to — its own path starts ON that
    boundary and must not be pushed off it. *ends* is false when the first and
    last point are anchors (a gate, a facility entrance) that must not move.
    """
    out = list(pts)
    cls = _clear_list(obstacles, clear)
    lo, hi = (0, len(out)) if ends else (1, max(1, len(out) - 1))
    for _ in range(iters):
        moved = False
        for i in range(lo, hi):
            q = out[i]
            for ob, cl in zip(obstacles, cls):
                if any(ob is e for e in exempt):
                    continue
                near, inside, d = _nearest_on_obb(q, ob)
                if inside or d < cl:
                    n = sn._sub(q, near)
                    if sn._norm(n) < 1e-6:
                        # q sits on the boundary: away from the centre is out
                        cx = sum(p[0] for p in ob) / 4.0
                        cy = sum(p[1] for p in ob) / 4.0
                        n = sn._sub(q, (cx, cy))
                        if sn._norm(n) < 1e-6:
                            n = (1.0, 0.0)
                        n = sn._unit(n)
                    else:
                        # AND THIS IS THE SIGN THAT WAS WRONG. For a point
                        # INSIDE the box, `q - near` points from the boundary
                        # inward, so pushing along it drove the point deeper in
                        # and it never came out — which is why the router kept
                        # finding endpoints pinned inside a facility and gave up
                        # on the detour. Outward is the other way.
                        n = sn._unit(n)
                        if inside:
                            n = sn._mul(n, -1.0)
                    out[i] = sn._add(near, sn._mul(n, cl))
                    q = out[i]
                    moved = True
        if not moved:
            break
    return out


# ---------------------------------------------------------------------------
# segment-level routing — the path has to MISS the facility, not merely have
# its sample points outside one
# ---------------------------------------------------------------------------

_CORNER_M = 1.4        # slack beyond `clear` on the corners a detour rounds


def _box_frame(cor):
    """Centre, unit axes and half-extents of the oriented box *cor*."""
    ctr = (sum(p[0] for p in cor) / 4.0, sum(p[1] for p in cor) / 4.0)
    u = sn._unit(sn._sub(cor[0], cor[1]))
    v = sn._unit(sn._sub(cor[0], cor[3]))
    return (ctr, u, v,
            sn._dist(cor[0], cor[1]) / 2.0, sn._dist(cor[0], cor[3]) / 2.0)


def _span(a, b, frame, pad):
    """As `seg_box_span`, on an already-computed frame — the inner loop."""
    ctr, u, v, hw, hh = frame
    hw += pad
    hh += pad
    ra, rb = sn._sub(a, ctr), sn._sub(b, ctr)
    la = (sn._dot(ra, u), sn._dot(ra, v))
    lb = (sn._dot(rb, u), sn._dot(rb, v))
    t0, t1 = 0.0, 1.0
    for k, lim in ((0, hw), (1, hh)):
        p, d = la[k], lb[k] - la[k]
        if abs(d) < 1e-12:
            if p < -lim or p > lim:
                return None
        else:
            ka, kb = (-lim - p) / d, (lim - p) / d
            if ka > kb:
                ka, kb = kb, ka
            t0 = max(t0, ka)
            t1 = min(t1, kb)
            if t0 > t1:
                return None
    return (t0, t1)


def seg_box_span(a, b, cor, pad=0.0):
    """Parametric span of segment a-b inside *cor* grown by *pad*, else None.

    A facility is an ORIENTED box, so an axis-aligned interval test is both
    wrong and far too fat — the AABB of a rotated 100 x 64 m pitch is nearly
    twice its area and would report crossings that do not happen. In the box's
    OWN frame it is axis aligned, so clip the segment against its two slabs
    there and the answer is exact.
    """
    return _span(a, b, _box_frame(cor), pad)


def _hull(pts):
    """CCW convex hull, collinear points KEPT so a and b survive on it."""
    pts = sorted(set(pts))
    if len(pts) < 3:
        return list(pts)

    def half(ps):
        h = []
        for p in ps:
            while len(h) >= 2 and sn._cross(sn._sub(h[-1], h[-2]),
                                            sn._sub(p, h[-2])) < 0.0:
                h.pop()
            h.append(p)
        return h

    lo = half(pts)
    up = half(list(reversed(pts)))
    return lo[:-1] + up[:-1]


def _bypass(a, b, frame, pad, both=False):
    """Corner chain taking a-b round a box, the shorter of the two ways.

    With *both*, hand back BOTH ways, shorter first, and let the caller choose.
    It has to be able to: once facilities carry a padding band, two of them
    standing 12 m apart have no gap left between their bands, and the shorter
    way round one of them is straight down that dead corridor. The caller
    lifts each candidate's corners and takes the way whose corners survive.

    ROUND THE HULL OF {a, b, box}, not round the box's own corners. Splitting
    the inflated corners into those left and right of a-b and walking one side
    LOOKS like the bypass and is not one: the first corner on the chosen side
    can sit behind the box as seen from *a*, so the opening leg goes straight
    back through the thing being avoided. That was why routing thrashed — every
    detour re-collided with its own obstacle, spliced again, and the point
    count ran into the thousands before the budget stopped it. The hull
    contains the box, so walking its boundary cannot re-enter, and either way
    round is legal; the shorter one is the one a person would take.
    """
    ctr, u, v, hw, hh = frame
    if sn._dist(a, b) < 1e-9:
        return []
    # PAD LADDER. An endpoint that is outside the facility but inside the
    # INFLATED box is not on the hull, so the full-clearance detour has nowhere
    # to start and the old code gave up — leaving the segment running straight
    # through the pitch, which is the one outcome that is not allowed. Falling
    # back to a tighter detour hugs the facility instead of clearing it by the
    # full margin, and hugging is enormously better than crossing.
    h = None
    for k in (1.0, 0.6, 0.3, 0.1):
        p = pad * k if k > 0.1 else 0.15
        inf = []
        for sx, sy in ((1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)):
            ex, ey = sx * (hw + p), sy * (hh + p)
            inf.append((ctr[0] + u[0] * ex + v[0] * ey,
                        ctr[1] + u[1] * ex + v[1] * ey))
        cand = _hull([a, b] + inf)
        if a in cand and b in cand:
            h = cand
            break
    if h is None:
        return [] if not both else []
    n = len(h)
    ia, ib = h.index(a), h.index(b)
    opts = []
    for stepd in (1, -1):
        ch, k = [], (ia + stepd) % n
        while k != ib and len(ch) < n:
            ch.append(h[k])
            k = (k + stepd) % n
        seq = [a] + ch + [b]
        L = sum(sn._dist(seq[i], seq[i + 1]) for i in range(len(seq) - 1))
        opts.append((L, ch))
    opts.sort(key=lambda t: t[0])
    if both:
        return [ch for (_L, ch) in opts]
    return opts[0][1] or []


def _lift(p, recs, margins, iters=6):
    """Shove a point out of every box it is inside, by the shortest way.

    In the box's own frame the escape is just whichever slab it is nearest to
    leaving, which is both cheaper and better behaved than projecting onto the
    nearest boundary point — the projection has no sign of its own and pushed
    interior points further in.

    *margins* is one figure per rec, because the boxes no longer share a
    clearance.
    """
    margins = _clear_list(recs, margins)
    for _ in range(iters):
        moved = False
        for (ctr, u, v, hw, hh), margin in zip(recs, margins):
            r = sn._sub(p, ctr)
            lx, ly = sn._dot(r, u), sn._dot(r, v)
            ex, ey = (hw + margin) - abs(lx), (hh + margin) - abs(ly)
            if ex <= 0.0 or ey <= 0.0:
                continue
            if ex < ey:
                lx = (hw + margin) if lx >= 0.0 else -(hw + margin)
            else:
                ly = (hh + margin) if ly >= 0.0 else -(hh + margin)
            p = (ctr[0] + u[0] * lx + v[0] * ly,
                 ctr[1] + u[1] * lx + v[1] * ly)
            moved = True
        if not moved:
            break
    return p


def _in_box(p, frame, margin):
    """Is *p* inside box *frame* grown by *margin*? The router's own predicate."""
    ctr, u, v, hw, hh = frame
    r = sn._sub(p, ctr)
    return abs(sn._dot(r, u)) < hw + margin and abs(sn._dot(r, v)) < hh + margin


def _first_hit(a, b, recs, clears):
    """The obstacle a-b runs into SOONEST, so detours resolve in travel order.

    Returns the ``(frame, clearance)`` pair, because the detour has to be taken
    round the box at THAT box's own margin, not at some scene-wide one.
    """
    hit, ht = None, 2.0
    for frame, cl in zip(recs, clears):
        sp = _span(a, b, frame, cl)
        if sp is None or sp[1] - sp[0] < 1e-9:
            continue
        if sp[0] < ht:
            ht, hit = sp[0], (frame, cl)
    return hit


def _route_seg(a, b, recs, clears, budget):
    """One segment, detoured round everything it meets.

    ITERATIVE, NOT RECURSIVE. Recursing on each of the five sub-segments a
    bypass produces fans out as 5^depth and hung the generator outright — the
    work is quadratic in the obstacles at worst, but the recursion made it
    exponential in nothing but its own bookkeeping. Splicing corner chains
    into a single chain and rescanning from the splice does the same job in a
    bounded number of passes.
    """
    clears = _clear_list(recs, clears)
    # A spliced corner has to come out of any OTHER box it lands in far enough
    # to be on that box's bypass hull, or the next detour computed from it
    # falls down the pad ladder in `_bypass` and ends up hugging the facility.
    # Half the clearance (which is what this used) is by construction inside
    # the inflated box, so it guaranteed the fallback every time.
    lifts = [cl + _CORNER_M + 0.5 for cl in clears]
    bands = [cl + _CORNER_M - 0.1 for cl in clears]
    chain = [a, b]
    i, stall = 0, 0
    while i < len(chain) - 1 and budget[0] > 0:
        hit = _first_hit(chain[i], chain[i + 1], recs, clears)
        if hit is None:
            i += 1
            stall = 0
            continue
        frame, cl = hit
        # EITHER WAY ROUND, WHICHEVER SURVIVES. The shorter way is what a
        # person would take and is still tried first, but with a padding band
        # on every facility the short way round a pitch is frequently the
        # corridor between it and the picnic ground beside it — 12 m of grass
        # with 16 m of band wanted in it. The corners land in the dead zone,
        # `_lift` bounces them between the two boundaries without ever
        # satisfying either, and the segment ends up parked in the gap, which
        # is precisely the graze the band was added to stop. Testing the long
        # way round as well costs one extra lift and settles it.
        opts = _bypass(chain[i], chain[i + 1], frame, cl + _CORNER_M, both=True)
        ins = []
        for opt in opts:
            if not opt:
                continue
            cand = [_lift(q, recs, lifts) for q in opt]
            if not any(_in_box(q, f, m)
                       for q in cand for f, m in zip(recs, bands)):
                ins = cand
                break
        if not ins:
            for opt in opts:
                if opt:
                    ins = opt
                    break
        # A corner taken round ONE facility can land inside the NEXT one, and
        # nothing else ever revisits it: the pre-pass only saw the original
        # waypoints, and the detour that would fix it needs an endpoint that is
        # not already inside a box. Lifting each spliced corner as it is
        # created is what stops a bike route pausing for two segments in the
        # middle of a pitch.
        ins = [_lift(q, recs, lifts) for q in ins]
        budget[0] -= 1

        stall += 1
        # A bypass that cannot help — the endpoint is pinned inside the box —
        # would otherwise splice for ever at the same index.
        #
        # THE STALL ALLOWANCE IS NOT A FREE CONSTANT. Padding puts facilities
        # closer together than twice the band in places, and a segment across
        # such a corridor ping-pongs: round the pitch, back into the picnic
        # ground, round that, back at the pitch. It does converge — each
        # bypass is a hull walk, so it strictly goes round — but not in the six
        # tries this allowed, and giving up left the segment in the corridor,
        # which is precisely the case the band was added to catch.
        if not ins or stall > 18 or len(chain) > 400:
            i += 1
            stall = 0
            continue
        chain[i + 1:i + 1] = ins
    return chain


def route(pts, obstacles, clear, exempt=(), ends=False, budget=400):
    """Route a polyline so that NO SEGMENT of it comes within *clear* of an
    obstacle — the guarantee `avoid()` cannot make.

    Points first get lifted out of anything they are sitting inside, because a
    detour computed from an endpoint that is itself inside the obstacle has
    nowhere legal to start. Then every segment is tested against every
    obstacle and taken round the nearest one it meets.
    """
    if len(pts) < 2:
        return list(pts)
    cls = _clear_list(obstacles, clear)
    keep = [i for i, o in enumerate(obstacles)
            if not any(o is e for e in exempt)]
    obs = [obstacles[i] for i in keep]
    cls = [cls[i] for i in keep]
    # Push clear of the BYPASS inflation, not just of `clear`: a point lifted
    # to exactly `clear` still sits inside the box the detour corners are taken
    # from, so it would not appear on that hull and the bypass would give up.
    seed = avoid(pts, obs, [x + _CORNER_M + 1.0 for x in cls],
                 iters=8, ends=ends)
    recs = [_box_frame(o) for o in obs]
    # DROP THE WAYPOINTS THE DETOUR CANNOT START FROM. `avoid` measures
    # Euclidean distance to the box; `_bypass` and `_span` measure against the
    # box INFLATED by the pad, and near a corner those are emphatically not the
    # same thing — a point shoved `clear` metres off a corner along the
    # diagonal is still inside the inflated rectangle. It therefore does not
    # land on the detour hull, `_bypass` walks down its pad ladder until it
    # finds one that does contain it, and the "detour" it returns hugs the
    # facility instead of clearing it. That is the whole of the padding
    # failure: the segments came out BESIDE the courts, not on them, which is
    # exactly the defect the band exists to stop.
    #
    # Dropping rather than shoving, because a waypoint is a suggestion (a tour
    # stop, a sample off the smoothing curve) and the facility is not. Shoving
    # them out was tried and is worse than it sounds: consecutive samples
    # escape by different faces, the curve turns into a zigzag, and the spine
    # came back with four thousand points in it. Take them out instead and the
    # segment spans the gap in one piece, which `_route_seg` then takes round
    # the outside — the way a person walking would.
    #
    # Endpoints are never dropped: they are what the path is FOR.
    band = [x + _CORNER_M + 0.1 for x in cls]
    if len(seed) > 2:
        kept = [seed[0]]
        for q in seed[1:-1]:
            if not any(_in_box(q, f, m) for f, m in zip(recs, band)):
                kept.append(q)
        kept.append(seed[-1])
        seed = kept
    # Detours are charged against a budget so a pathological seed cannot hang
    # the generator, but the figure has to be PER SEGMENT, not per route. The
    # coarse tour is seventeen waypoints and each of its legs crosses the whole
    # park past eight facilities, so a flat 400 was down to its last twenty
    # before the tour was half laid — and every leg after that came out
    # unrouted, which is where most of the padding failures were coming from.
    # `stall` and the chain cap bound what one segment can spend, so scaling by
    # the segment count only ever buys the work the router was going to be
    # allowed to do anyway.
    bud = [max(budget, 64 * len(seed))]
    out = [seed[0]]
    for i in range(len(seed) - 1):
        out.extend(_route_seg(seed[i], seed[i + 1], recs, cls, bud)[1:])
    out = _dedupe(out)
    # REPAIR PASS. `clear` is a comfort margin and missing it is untidy; being
    # inside the facility at all is the actual defect, so anything still
    # penetrating gets one more detour at a clearance it cannot argue with.
    # Segments that are already fine cost one slab clip each, so this is nearly
    # free — it only does work where there is something left to fix.
    bud2 = [max(budget, 64 * len(out))]
    rep = [out[0]]
    fix = [0.6] * len(recs)
    for i in range(len(out) - 1):
        rep.extend(_route_seg(out[i], out[i + 1], recs, fix, bud2)[1:])
    return _dedupe(rep)


def deloop(pts, max_passes=60, max_loop_m=float("inf")):
    """Cut out any loop where a polyline crosses ITSELF.

    The router splices detour waypoints in as it walks a route round obstacles,
    and nothing checked whether the spliced result folded back over its own
    earlier section. It does: measured, 23 self-crossings on seed 3 and 345,382
    on seed 7. On the plan that reads as a snarl of paths on top of each other,
    which is what it is -- one path, crossing itself.

    Where segment i and a later segment j intersect, everything between them is
    a loop that leads nowhere, so it is replaced by the intersection point. This
    is the standard de-loop; doing it AFTER routing rather than trying to stop
    the router creating them keeps the router simple, and a loop is trivial to
    detect once the polyline exists.
    """
    out = list(pts)
    for _ in range(max_passes):
        cut = None
        n = len(out)
        for x in range(n - 1):
            for y in range(x + 2, n - 1):
                hit = sn._seg_intersect(out[x], out[x + 1], out[y], out[y + 1])
                if hit is None:
                    continue
                # UNBOUNDED BY DEFAULT. Bounding this to "small" loops was
                # tried on the theory that a long self-crossing is the route
                # legitimately going round something; measured, it is not --
                # at a 45 m bound seed 42 still carried 42,461 self-crossings,
                # so the pathological loops are the LARGE ones. Orphaned spurs
                # were 8-9 either way, i.e. not caused by de-looping.
                span = sum(sn._dist(out[k], out[k + 1])
                           for k in range(x, min(y + 1, n - 1)))
                if span > max_loop_m:
                    continue
                cut = (x, y, hit[0])
                break
            if cut:
                break
        if not cut:
            break
        x, y, q = cut
        out = out[:x + 1] + [q] + out[y + 1:]
    return out


def _dedupe(pts, tol=0.05):
    out = [pts[0]]
    for q in pts[1:]:
        if sn._dist(q, out[-1]) > tol:
            out.append(q)
    if len(out) < 2:
        out = list(pts[:2])
    return out


def lay(way, obstacles, clear, samples=8, exempt=(), ends=False):
    """Waypoints -> a smooth path that is legal.

    Routed first with slack, so the smoothing that follows has room to bulge
    into; then routed again at the true clearance to repair anything the curve
    cut. Smoothing a legal polyline is not itself legal — a Catmull-Rom
    through a detour's corners bows outward on the turns and inward at the
    joins, and it was the inward bow that put the spine back on the pitch.
    """
    cls = _clear_list(obstacles, clear)
    coarse = route(way, obstacles, [x + 3.0 for x in cls],
                   exempt=exempt, ends=ends)
    if len(coarse) < 2:
        return coarse
    smooth = _curve(coarse, samples=samples)
    return route(smooth, obstacles, cls, exempt=exempt, ends=ends)


def _offset(pts, d):
    """Polyline shifted *d* to its left — a cycle track alongside a footpath."""
    out = []
    for i, p in enumerate(pts):
        a = pts[max(0, i - 1)]
        b = pts[min(len(pts) - 1, i + 1)]
        t = sn._unit(sn._sub(b, a))
        out.append((p[0] - t[1] * d, p[1] + t[0] * d))
    return out


def _decimate(pts, step):
    out = pts[::step]
    if out[-1] is not pts[-1]:
        out.append(pts[-1])
    return out


def _along(pts, spacing, start):
    """Walk a polyline, yielding (point, unit tangent) every *spacing* metres."""
    out, s, nxt = [], 0.0, start
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        L = sn._dist(a, b)
        if L < 1e-9:
            continue
        t = sn._unit(sn._sub(b, a))
        while nxt <= s + L:
            f = (nxt - s) / L
            out.append(((a[0] + (b[0] - a[0]) * f,
                         a[1] + (b[1] - a[1]) * f), t))
            nxt += spacing
        s += L
    return out


def offset_polyline(pts, d):
    """*pts* shifted sideways by *d*, mitred at interior vertices.

    Needed because a shared-use path is drawn as its two sides, not as one
    stroke: the cycle side has to follow the same curve as the walking side at
    a constant offset, and a per-segment normal would leave a notch at every
    bend. Same mitre reasoning as the road ribbons.
    """
    n = len(pts)
    if n < 2:
        return list(pts)
    out = []
    for i in range(n):
        if i == 0:
            m = sn._perp(sn._unit(sn._sub(pts[1], pts[0])))
            k = 1.0
        elif i == n - 1:
            m = sn._perp(sn._unit(sn._sub(pts[-1], pts[-2])))
            k = 1.0
        else:
            d0 = sn._unit(sn._sub(pts[i], pts[i - 1]))
            d1 = sn._unit(sn._sub(pts[i + 1], pts[i]))
            m = sn._unit(sn._add(sn._perp(d0), sn._perp(d1)))
            k = 1.0 / max(0.4, abs(sn._dot(m, sn._perp(d0))))
        out.append(sn._add(pts[i], sn._mul(m, d * k)))
    return out


def _curve(waypoints, samples=16):
    """A smooth path through *waypoints*, Catmull-Rom style.

    Straight segments between facilities is what made the path network read as
    plumbing. Real park paths curve, and they curve because they were desire
    lines before they were paving.
    """
    if len(waypoints) < 2:
        return list(waypoints)
    pts = [waypoints[0]]
    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        prev = waypoints[i - 1] if i > 0 else a
        nxt = waypoints[i + 2] if i + 2 < len(waypoints) else b
        ta = sn._unit(sn._sub(b, prev))
        tb = sn._unit(sn._sub(nxt, a))
        pts.extend(sn.hermite(a, ta, b, tb, tension=0.38,
                              samples=samples)[1:])
    return pts


def _centre(r):
    return ((r[0] + r[2]) / 2.0, (r[1] + r[3]) / 2.0)


def _fence_run(rect, panel):
    """Panel centres and yaws around *rect*, one run per side."""
    x0, y0, x1, y1 = rect
    out = []
    for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1),
                             (x1, y1, x0, y1), (x0, y1, x0, y0)):
        L = math.hypot(bx - ax, by - ay)
        n = max(1, int(round(L / panel)))
        yaw = math.degrees(math.atan2(by - ay, bx - ax))
        for k in range(n):
            f = (k + 0.5) / n
            out.append({"c": (ax + (bx - ax) * f, ay + (by - ay) * f),
                        "yaw": yaw})
    return out


def facility_gap(c):
    """Spacing between facilities, wide enough for a padded path to pass."""
    g = float(c.get("facility_gap_m", 0.0) or 0.0)
    if g > 0.0:
        return g
    pad = max(float(c["facility_pad_m"]),
              float(c.get("facility_pad_fenced_m", c["facility_pad_m"])))
    return 2.0 * pad + float(c["path_w_shared_m"]) + 2.0


def facility_pad(z, c):
    """Grass a path must leave between its EDGE and facility *z*, in metres.

    Split by kind rather than one figure for the park, and the reason is what
    stands on the boundary. A fenced compound is not just its slab: the
    chain-link run occupies the line itself, the gate swings outward off it,
    and the bike racks are parked 4.5 m off the same face — so the base band
    would put the shared path through the racks and leave the fence nowhere to
    stand. A pitch, a picnic ground or a playground has a soft edge with
    nothing on it, and there the band only has to read as grass.
    """
    if z.get("fenced"):
        return float(c.get("facility_pad_fenced_m", c["facility_pad_m"]))
    return float(c["facility_pad_m"])


# ---------------------------------------------------------------------------
# the path network — a GRAPH, not a heap of polylines
# ---------------------------------------------------------------------------

# How close to an edge's end a join has to be before it is that end rather than
# a new node one metre short of it, and how close a path has to arrive before
# the connector between the two is a rounding error rather than a path.
_JOIN_M = 1.0
_WELD_M = 2.0


def _finish(pts):
    """The geometry an edge is stored with: even samples, no self-crossing.

    Resampled to 3 m FIRST, because `_decimate` takes an integer stride rather
    than a distance and so thins a dense stretch and a sparse one by the same
    factor; even arclength spacing makes the de-loop's segment tests comparable
    and bounds the point count by path LENGTH. Done as the edge enters the
    graph, so the stored geometry is the drawn geometry — a junction is then
    sited on the line that actually gets paved.
    """
    if len(pts) < 2:
        return list(pts)
    if sn.polyline_length(pts) > 6.0:
        pts = sn.resample(pts, 3.0)
    return deloop(pts)


class PathNet:
    """The park's paths as a graph: nodes are junctions, gates and facility
    entrances; edges carry the polyline geometry between them.

    THIS IS THE FIX FOR THE ORPHANED SPUR, and it is structural rather than a
    tidy-up. Paths used to be built as a list of INDEPENDENT POLYLINES: a spur
    was routed from a court to a point the code had computed on the trunk, and
    nothing anywhere recorded that the point was a junction. The pass that
    afterwards clipped doubled paving was therefore free to delete the exact
    stretch of trunk a spur had joined onto — and did. The spur survived, still
    touching its court, now leading into the grass. No repair pass fixes that,
    because the fact it needed (this point is a junction) was never written
    down; and a repair pass is the wrong shape of answer anyway, since it
    reconnects what should never have come apart.

    Here a path that meets another SPLITS it, exactly as `suburb_net.Network`
    does for streets and for the same reason, so the meeting is a node of
    degree three. Two properties then hold BY CONSTRUCTION rather than by
    inspection afterwards:

      * every edge ends at a node, so a path can only stop at a junction, a
        facility entrance, a set piece or a gate — there is nowhere else for an
        end to be;
      * every edge is joined to the network in the same breath as it is
        created, so the network is one connected component.

    None of `suburb_net`'s street_id / road_class machinery is here: a park
    path is not a named road, nothing downstream addresses a route as one
    continuous way, and a footpath has no class.
    """

    def __init__(self):
        self.nodes = {}          # id -> point
        self.edges = {}          # id -> path record, incl. `a`, `b`, `pts`
        self.adj = {}            # node id -> set of edge ids
        self._nid = 0
        self._eid = 0

    # -- construction -----------------------------------------------------
    def add_node(self, p):
        self._nid += 1
        self.nodes[self._nid] = (float(p[0]), float(p[1]))
        self.adj[self._nid] = set()
        return self._nid

    def node_at(self, p, tol=0.75):
        """Reuse a node within *tol*, so paths that end together share one
        junction rather than stacking two nodes a centimetre apart."""
        best, bd = None, tol
        for nid, q in self.nodes.items():
            d = sn._dist(p, q)
            if d <= bd:
                best, bd = nid, d
        return best

    def add_edge(self, pts, kind, width_m, a=None, b=None, **extra):
        pts = list(pts)
        if len(pts) < 2:
            return None
        pts = _dedupe(pts)
        if len(pts) < 2:
            return None
        if a is None:
            a = self.node_at(pts[0]) or self.add_node(pts[0])
        if b is None:
            b = self.node_at(pts[-1]) or self.add_node(pts[-1])
        # Snap the geometry onto its nodes. Graph and drawing then agree
        # EXACTLY, which is what makes "these two paths share an end" a
        # testable property instead of a matter of tolerance.
        pts[0], pts[-1] = self.nodes[a], self.nodes[b]
        if a == b and sn.polyline_length(pts) < 2.0:
            return None
        self._eid += 1
        e = dict(extra)
        e.update({"id": self._eid, "a": a, "b": b, "pts": pts, "kind": kind,
                  "width_m": width_m})
        self.edges[self._eid] = e
        self.adj[a].add(self._eid)
        self.adj[b].add(self._eid)
        return e

    def remove_edge(self, eid):
        e = self.edges.pop(eid)
        for nid in (e["a"], e["b"]):
            self.adj.get(nid, set()).discard(eid)
        return e

    def split(self, e, s):
        """Split *e* at arclength *s*, returning the node id at the cut.

        This is what makes a T-junction a real junction rather than two paths
        that merely touch: the host is genuinely cut, and the node it gains has
        degree three.
        """
        total = sn.polyline_length(e["pts"])
        if s <= _JOIN_M:
            return e["a"]
        if s >= total - _JOIN_M:
            return e["b"]
        head, tail = sn.split_polyline(e["pts"], s)
        self.remove_edge(e["id"])
        mid = self.add_node(head[-1])
        # `serves` is the padding EXEMPTION, and only the stretch inside the
        # facility's own band has earned it — the head, since a serving stub is
        # built from the boundary outward.
        for (piece, na, nb, serves) in ((head, e["a"], mid, e.get("serves")),
                                        (tail, mid, e["b"], None)):
            self._eid += 1
            f = dict(e)
            f.update({"id": self._eid, "a": na, "b": nb, "pts": piece,
                      "serves": serves})
            self.edges[self._eid] = f
            self.adj[na].add(self._eid)
            self.adj[nb].add(self._eid)
        return mid

    def weld(self, a, b):
        """Merge node *a* into node *b*, dragging its edges' ends with it.

        For a path that arrives at the network essentially on top of an
        existing node: a 30 cm connector is not a path, it is a rounding error
        with an id, and the old code's answer — leave the gap unbridged — is
        precisely how a spur ended two metres short of the trunk it was
        supposed to be joining.
        """
        if a == b or a not in self.nodes:
            return b
        pb = self.nodes[b]
        for eid in list(self.adj[a]):
            e = self.edges[eid]
            if e["a"] == a:
                e["a"] = b
                e["pts"][0] = pb
            if e["b"] == a:
                e["b"] = b
                e["pts"][-1] = pb
            self.adj[b].add(eid)
        del self.adj[a]
        del self.nodes[a]
        for eid in list(self.adj[b]):
            e = self.edges[eid]
            if e["a"] == e["b"] and sn.polyline_length(e["pts"]) < 2.0:
                self.remove_edge(eid)
        return b

    # -- queries ----------------------------------------------------------
    def candidates(self, p, ignore=(), skip_serving=True, step=8.0):
        """Points on the network *p* could join to, nearest first, as
        ``(distance, edge, arclength, point)``.

        A list rather than a single answer because the nearest join is not
        always the join you can walk to: put a fenced compound between the two
        and the router wraps the connection round it and back, which is a 54 m
        gap paved as a 208 m path. The caller picks; see `join_point`.

        SAMPLED ALONG each edge, not one point per edge. Offering only each
        edge's nearest point makes the answer depend on where that edge happens
        to have been split by earlier work — and when a facility stands between
        *p* and that one point, the perfectly walkable alternative twenty
        metres further along THE SAME EDGE is never offered at all. That was
        the concrete case above.

        Serving stubs are skipped: they are the one path allowed inside a
        facility's padding band, so cutting a junction into one would drag a
        path that holds no such exemption in there with it.
        """
        out = []
        for e in self.edges.values():
            if e["id"] in ignore:
                continue
            if skip_serving and e.get("serves") is not None:
                continue
            pts, s, near = e["pts"], 0.0, None
            for i in range(len(pts) - 1):
                d = sn._sub(pts[i + 1], pts[i])
                L = sn._norm(d)
                t = 0.0
                if L > 1e-9:
                    t = max(0.0, min(1.0, sn._dot(sn._sub(p, pts[i]), d)
                                     / (L * L)))
                q = sn._add(pts[i], sn._mul(d, t))
                dd = sn._dist(p, q)
                if near is None or dd < near[0]:
                    near = (dd, e, s + t * L, q)
                s += L
            if near is None:
                continue
            out.append(near)
            total = s
            k = 0.0
            while k <= total:
                if abs(k - near[2]) > step * 0.5:
                    q = sn.point_at(pts, k)
                    out.append((sn._dist(p, q), e, k, q))
                k += step
        out.sort(key=lambda z: z[0])
        return out

    def nearest(self, p, ignore=(), skip_serving=True):
        """Nearest point on the network to *p*, or None if there is none."""
        c = self.candidates(p, ignore=ignore, skip_serving=skip_serving)
        return c[0] if c else None

    def split_at(self, p, tol=1.0):
        """Split whatever edge passes through *p*, returning the node there.

        Sited by POINT rather than by (edge, arclength) so it stays correct
        after an earlier split has replaced the edge the caller was holding —
        the same hazard `suburb_net._connect_route` handles with
        `_edge_containing`.
        """
        hit = self.nearest(p)
        if hit is None or hit[0] > tol:
            return None
        return self.split(hit[1], hit[2])

    def clearance(self, pts, ignore=(), skip_ends_m=0.0):
        """Smallest distance from *pts* to any edge not in *ignore*.

        What a candidate route is tested with BEFORE it is laid, which is how
        `suburb_net` keeps two streets off the same ground and how this keeps
        two paths off it. Testing beforehand is the whole difference between
        not paving the same strip twice and paving it twice and cutting one of
        them up afterwards — and it was the cutting up that severed spurs from
        the network.

        *skip_ends_m* trims that much off each end of the candidate before
        measuring, because a route that is about to be joined to the network at
        both ends is ON the network there by construction and would otherwise
        report zero clearance against the very edge it is joining.
        """
        probe = list(pts)
        if skip_ends_m > 0.0:
            total = sn.polyline_length(probe)
            if total <= 2.0 * skip_ends_m + 1.0:
                return float("inf")
            _h, probe = sn.split_polyline(probe, skip_ends_m)
            probe, _t = sn.split_polyline(
                probe, sn.polyline_length(probe) - skip_ends_m)
        best = float("inf")
        for e in self.edges.values():
            if e["id"] in ignore:
                continue
            best = min(best, sn.polyline_dist(probe, e["pts"]))
        return best

    def paths(self):
        """The edges as the plain path records the rest of the file expects."""
        return [self.edges[k] for k in sorted(self.edges)]


# ---------------------------------------------------------------------------
# the layout
# ---------------------------------------------------------------------------

def plan(rng, cfg=None):
    """Lay the park out. Returns a dict of zones, paths, fences and props.

    THE ORDER IS THE DESIGN, and it is the order the thing is actually built
    in:

        (a) FACILITIES, plus the set pieces that are destinations rather than
            furniture — fountains, gazebos, the sign. Their dimensions are
            fixed, so they claim ground first and everything else works round
            them.
        (b) PATHS, routed between what is already standing and joining each
            facility square on to its boundary.
        (c) TREES, into whatever green is left, clear of facilities AND paths.
        (d) PATH FURNITURE — benches and bins. These belong to the path, so
            they cannot be positioned until the route is final.

    Interleaving them was the bug. Furniture was placed against paths that had
    not been routed yet and ended up in the grass beside nothing; trees took
    ground the paths then had to be pushed through.
    """
    c = dict(DEFAULTS)
    c.update(cfg or {})
    W, H = float(c["region_m"][0]), float(c["region_m"][1])
    hw, hh = W / 2.0, H / 2.0
    region = (-hw, -hh, hw, hh)
    buf = float(c["edge_buffer_m"])
    gap = facility_gap(c)
    clear = float(c["path_clear_m"])

    inner = (-hw + buf, -hh + buf, hw - buf, hh - buf)
    zones, placed = [], []
    # One base orientation for the park, with each facility swung off it. A
    # park is not aligned to north; it is aligned to its own site.
    base_yaw = rng.uniform(-22.0, 22.0)
    gates = [(0.0, -hh + buf * 0.5),
             (hw - buf * 0.5, rng.uniform(-hh * 0.2, hh * 0.4))]

    def add(kind, w, h, fx, fy, spread=16.0, **extra):
        yaw = base_yaw + rng.uniform(-spread, spread)
        c, yaw, corners = _place(inner, placed, gap, w, h, fx, fy, yaw, rng)
        if c is None:
            return None
        placed.append(corners)
        z = {"kind": kind, "centre": c, "w": w, "h": h, "yaw": yaw,
             "corners": corners}
        z.update(extra)
        zones.append(z)
        return z

    def local(z, lx, ly):
        """Local facility coords -> world."""
        a = math.radians(z["yaw"])
        ux, uy = math.cos(a), math.sin(a)
        return (z["centre"][0] + ux * lx - uy * ly,
                z["centre"][1] + uy * lx + ux * ly)

    props = []

    def prop(kind, x, y, yaw=0.0):
        props.append({"kind": kind, "c": (x, y), "yaw": yaw})

    bl, bw = BASKETBALL["court"]
    bpad = BASKETBALL["pad"]

    # PITCHES GO TOGETHER, sharing one orientation and one gap. Scattering them
    # to opposite ends of the park was wrong twice over: clubs lay pitches as a
    # block so one set of goals, parking and drainage serves both, and two
    # pitches at different angles read as an accident rather than as a ground.
    pl, pwd = SOCCER["pitch"]
    n_sc = int(c["n_soccer"])
    pitch_gap = max(float(c["pitch_gap_m"]), gap + 0.5)
    if n_sc:
        first = add("soccer", pl, pwd, 0.24, 0.32, spread=12.0)
        if first:
            a = math.radians(first["yaw"])
            ux, uy = math.cos(a), math.sin(a)
            vx, vy = -uy, ux                      # across the pitch
            # A REAL GAP, not the bare `facility_gap_m`. Flush pitches in the
            # same green with the same markings read as one pitch drawn twice;
            # `pitch_gap_m` opens a corridor wide enough to walk, and a path
            # goes down it below, which is what separates them on the ground.
            for i in range(1, n_sc):
                # Try BOTH sides: the first pitch may already sit near an edge,
                # and offsetting blindly to one side put the second pitch
                # outside the park, which silently dropped it.
                #
                # And CLOSE UP IF IT MUST. Widening the gap to separate the two
                # pitches also widened the pair, and at some orientations the
                # block stopped fitting the park at all — which dropped the
                # second pitch entirely. Two pitches near enough to touch still
                # read as two once the corridor path is between them; one pitch
                # reads as a bug.
                done = False
                for g in (pitch_gap, pitch_gap * 0.72, pitch_gap * 0.5,
                          gap + 0.5):
                    step = pwd + g
                    for sgn in (1.0, -1.0):
                        # A STAGGER ALONG THE TOUCHLINE is allowed, and is the
                        # difference between two pitches and one: squared up,
                        # the pair can miss the park boundary by a metre and
                        # the second pitch is dropped outright. Grounds stagger
                        # adjacent pitches anyway.
                        for slide in (0.0, 12.0, -12.0, 24.0, -24.0):
                            cx = (first["centre"][0] + vx * step * i * sgn
                                  + ux * slide)
                            cy = (first["centre"][1] + vy * step * i * sgn
                                  + uy * slide)
                            corners = _obb(cx, cy, pl, pwd, first["yaw"])
                            if _inside(corners, inner) and not any(
                                    _sat_overlap(corners, q, gap)
                                    for q in placed):
                                placed.append(corners)
                                zones.append({"kind": "soccer",
                                              "centre": (cx, cy),
                                              "w": pl, "h": pwd,
                                              "yaw": first["yaw"],
                                              "corners": corners})
                                done = True
                                break
                        if done:
                            break
                    if done:
                        break

    n_bb = int(c["n_basketball"])
    if n_bb:
        cols = 2 if n_bb > 1 else 1
        rows = int(math.ceil(n_bb / cols))
        # COURTS INSIDE ONE COMPOUND SHARE A SLAB. `gap` is the spacing
        # BETWEEN facilities and is derived from the padding a path needs to
        # pass (~20.6 m); using it here too spread the four courts apart as if
        # each were its own facility, when the whole point of a compound is that
        # they sit together inside one fence. A real park paints four courts on
        # one slab with a couple of metres between the sidelines.
        cgap = float(c["court_gap_m"])
        cw = cols * (bl + 2 * bpad) + (cols - 1) * cgap
        ch = rows * (bw + 2 * bpad) + (rows - 1) * cgap
        z = add("basketball_compound", cw, ch, 0.66, 0.22, courts=[],
                fenced=True)
        if z:
            for i in range(n_bb):
                r, q = divmod(i, cols)
                lx = -cw / 2 + bpad + bl / 2 + q * (bl + 2 * bpad + cgap)
                ly = -ch / 2 + bpad + bw / 2 + r * (bw + 2 * bpad + cgap)
                z["courts"].append({"centre": local(z, lx, ly),
                                    "yaw": z["yaw"]})

    tw, th = TENNIS["enclosure"]
    n_tn = int(c["n_tennis"])
    if n_tn:
        # NOT `H`: that is the park height, and rebinding it here silently
        # shrank the region for everything downstream.
        tblk = n_tn * th + (n_tn - 1) * 2.0
        z = add("tennis_block", tw, tblk, 0.90, 0.30, courts=[], fenced=True)
        if z:
            for i in range(n_tn):
                ly = -tblk / 2 + th / 2 + i * (th + 2.0)
                z["courts"].append({"centre": local(z, 0.0, ly),
                                    "yaw": z["yaw"]})

    pw, ph = _rng_pair(c["playground_m"], (46.0, 34.0))
    add("playground", pw, ph, 0.62, 0.50, surface="sand")

    picnic, spots = [], [(0.45, 0.50), (0.84, 0.58), (0.24, 0.50),
                         (0.70, 0.86)]
    for i in range(int(c["picnic_areas"])):
        mw, mh = _rng_pair(c["picnic_m"], (30.0, 22.0))
        fx, fy = spots[i % len(spots)]
        z = add("picnic", mw, mh, fx, fy, spread=26.0, tables=[])
        if z:
            picnic.append(z)

    # -- facility furniture: hoops, goals, tables, play kit ------------------
    # These belong to the facility, not to the path, so they go in with it.
    for z in zones:
        if z["kind"] == "basketball_compound":
            for court in z["courts"]:
                a = math.radians(court["yaw"])
                ux, uy = math.cos(a), math.sin(a)
                off = bl / 2.0 - BASKETBALL["hoop_inset"]
                for sgn in (-1.0, 1.0):
                    prop("hoop", court["centre"][0] + ux * sgn * off,
                         court["centre"][1] + uy * sgn * off,
                         court["yaw"] + (180.0 if sgn > 0 else 0.0))
        elif z["kind"] == "soccer":
            a = math.radians(z["yaw"])
            ux, uy = math.cos(a), math.sin(a)
            L = SOCCER["pitch"][0] / 2.0
            for sgn in (-1.0, 1.0):
                prop("soccer_goal", z["centre"][0] + ux * sgn * L,
                     z["centre"][1] + uy * sgn * L,
                     z["yaw"] + (180.0 if sgn > 0 else 0.0))

    tlo, thi = _rng_pair(c["picnic_tables"], (5.0, 9.0))
    for z in picnic:
        n = int(rng.uniform(tlo, thi + 0.999))
        cols = max(1, int(math.ceil(math.sqrt(n))))
        rows = int(math.ceil(n / cols))
        for i in range(n):
            r, q = divmod(i, cols)
            lx = (-z["w"] / 2 + z["w"] * (q + 0.5) / cols)
            ly = (-z["h"] / 2 + z["h"] * (r + 0.5) / rows)
            wx, wy = local(z, lx, ly)
            prop("picnic_table", wx, wy, z["yaw"] + rng.uniform(-12, 12))
            z["tables"].append((wx, wy))

    for z in zones:
        if z["kind"] != "playground":
            continue
        for i, kind in enumerate(("swing_set", "play_structure", "seesaw",
                                  "seesaw")):
            lx = -z["w"] / 2 + z["w"] * (i + 0.5) / 4.0
            ly = z["h"] * (0.15 if i % 2 else -0.15)
            wx, wy = local(z, lx, ly)
            prop(kind, wx, wy, rng.uniform(0, 360))

    # -- set pieces: fountains, gazebos, the sign ---------------------------
    # DESTINATIONS, not path furniture. A fountain is a thing you walk TO, so
    # it claims its ground with the facilities and the path network is routed
    # to reach it. Placing it after the paths meant siting it wherever was
    # left and then dragging a spur out to it, which is why those spurs used
    # to dangle off the spine pointing at nothing.
    ix0, iy0, ix1, iy1 = inner
    feature_pts, features = [], []

    def open_spot(rad, tries=400):
        for _ in range(tries):
            q = (rng.uniform(ix0 + rad, ix1 - rad),
                 rng.uniform(iy0 + rad, iy1 - rad))
            box = _obb(q[0], q[1], rad * 2, rad * 2, 0.0)
            if any(_sat_overlap(box, r, 2.0) for r in placed):
                continue
            if any(sn._dist(q, o[1]) < rad * 2.2 for o in feature_pts):
                continue
            return q
        return None

    def set_piece(kind, rad, box_r):
        q = open_spot(rad)
        if not q:
            return
        box = _obb(q[0], q[1], box_r * 2, box_r * 2, base_yaw)
        feature_pts.append((kind, q, box))
        features.append(box)
        prop(kind, q[0], q[1], rng.uniform(0, 360))

    n_f = int(rng.uniform(*_rng_pair(c["fountains"], (2.0, 3.0))) + 0.5)
    n_g = int(rng.uniform(*_rng_pair(c["gazebos"], (2.0, 4.0))) + 0.5)
    for _ in range(n_f):
        set_piece("fountain", 15.0, 5.0)
    for _ in range(n_g):
        set_piece("gazebo", 12.0, 4.5)
    prop("park_sign", gates[0][0], gates[0][1] + 3.0, 90.0)

    # Bike parking where people arrive to play: against the fenced compounds,
    # on the side the path reaches them from, not scattered round the park.
    for z in zones:
        if z["kind"] not in ("basketball_compound", "tennis_block"):
            continue
        # Toward the park interior, which is the side every path reaches these
        # compounds from. Computed from the facility alone so this does not
        # depend on the path network existing yet.
        n = sn._unit(sn._sub((0.0, 0.0), z["centre"]))
        if sn._norm(n) < 1e-6:
            n = (0.0, 1.0)
        # Boundary point on the interior-facing side, in the facility's own
        # frame -- `entrance()` is defined with the path network, which does not
        # exist yet at this point in the build.
        a = math.radians(z["yaw"])
        ux, uy = math.cos(a), math.sin(a)
        lx = max(-z["w"] / 2, min(z["w"] / 2, n[0] * ux + n[1] * uy)
                 * z["w"] / 2)
        ly = max(-z["h"] / 2, min(z["h"] / 2, -n[0] * uy + n[1] * ux)
                 * z["h"] / 2)
        e = local(z, lx, ly)
        t = sn._perp(n)
        base = sn._add(e, sn._mul(n, 4.5))
        for k in range(int(c["bike_racks_per_court"])):
            off = (k - (float(c["bike_racks_per_court"]) - 1) / 2.0) * 2.2
            q = sn._add(base, sn._mul(t, off))
            prop("bike_rack", q[0], q[1],
                 math.degrees(math.atan2(t[1], t[0])))

    # =======================================================================
    # (b) PATHS — everything above is now an obstacle
    # =======================================================================
    # The spine wanders, NOT a rectangle round the edge. The perimeter loop was
    # the same mistake as ringing the suburb with an arterial: it drew a
    # racetrack round the scene and made every facility hang off it like a
    # stub. A park is entered at a couple of points and threaded by a route
    # that wanders between the things worth reaching.
    # Built from `zones` rather than from `placed` so the pads below stay
    # index-aligned with the facilities they belong to; the corner lists are
    # the same objects either way, which is what the identity-based `exempt`
    # relies on.
    obstacles = [z["corners"] for z in zones] + features
    # THE PADDING BAND, one figure per obstacle. Recorded on the zone as well,
    # so the checks below measure what the plan actually promised rather than
    # re-deriving it from a config they were not given. Set pieces keep the old
    # `path_clear_m`: a fountain is a destination the path is meant to arrive
    # at, not a facility with an edge to respect.
    for z in zones:
        z["pad_m"] = facility_pad(z, c)
    pads = [z["pad_m"] for z in zones] + [clear] * len(features)

    def clears(width_m):
        """Centreline clearances for a way *width_m* wide.

        The band is measured from the path's EDGE and the router works in
        centreline distances, so the two differ by half the width — and it is
        the wide shared-use spine, whose cycle side runs along the OUTSIDE of
        the way, where that half matters most.
        """
        return [p + width_m / 2.0 for p in pads]

    # NO SEPARATE BIKE NETWORK. A second circuit that had to weave past the
    # first produced exactly the tangle it sounds like wherever the two met:
    # two independent routes crossing at shallow angles, each dodging the
    # other's obstacles. A park path is ONE wider way carrying both modes,
    # divided down its length -- which is also how shared-use paths are
    # actually built. `width_m` and `bike_share` on each edge say how wide it is
    # and how much of it is the cycle side; the divider is drawn from those, so
    # there is only ever one network to route and one set of junctions.
    w_spine = float(c["path_w_shared_m"])
    w_spur = float(c["path_w_spur_m"])
    share = float(c["bike_share"])
    net = PathNet()

    # Waypoints sit OFF each facility, on the side facing the park centre.
    stops = []
    for z in zones:
        cx, cy = z["centre"]
        n = sn._unit(sn._sub((0.0, 0.0), (cx, cy)))
        stops.append((cx + n[0] * (max(z["w"], z["h"]) / 2.0 + 9.0),
                      cy + n[1] * (max(z["w"], z["h"]) / 2.0 + 9.0)))
    for (_k, q, _b) in feature_pts:
        stops.append(q)

    # NEAREST-NEIGHBOUR TOUR from one gate to the other. Sorting the stops by
    # x+y read as an ordering but is not a route: it doubled back across the
    # middle of the park and the spine tied itself in a knot around the picnic
    # grounds. A greedy tour is not optimal and does not need to be — it just
    # has to not cross itself.
    #
    # NOT 2-OPTED. Improving the tour was tried, on straight-line cost and then
    # on a cost that charges for the facilities a leg would have to walk round.
    # Both are better tours and neither is a better park: the router pays for a
    # shorter leg with a longer detour, and measured over seeds 1/3/7/42 the
    # doubled paving went from 205 m to 355 m (straight-line: worse still).
    # Greedy leaves one long return leg that can run beside its outbound one;
    # that is the residual `check_doubled` reports, and it is the cheaper bill.
    way, rest, cur = [gates[0]], list(stops), gates[0]
    while rest:
        nxt = min(rest, key=lambda q: sn._dist(cur, q))
        rest.remove(nxt)
        way.append(nxt)
        cur = nxt
    way.append(gates[1])
    # THE ROOT OF THE NETWORK, and the only edge that joins nothing — it is
    # what everything else joins to. Gate to gate, so both its ends are park
    # entrances rather than stubs.
    net.add_edge(_finish(lay(way, obstacles, clears(w_spine), samples=10)),
                 "spine", w_spine, bike_share=share)

    def walkable(a, b, width_m, exempt=()):
        """Is the straight line from *a* to *b* clear of every padding band?"""
        for ob, cl in zip(obstacles, clears(width_m)):
            if any(ob is x for x in exempt):
                continue
            sp = seg_box_span(a, b, ob, cl - 0.1)
            if sp is not None and sp[1] - sp[0] > 1e-6:
                return False
        return True

    def legal(pts, width_m, exempt=()):
        """Does a ROUTED path keep every band it is not exempt from?

        `check_pad`'s predicate, asked before the path is committed instead of
        after the park is built. The router is a heuristic on a budget and
        where the gap it is asked to thread is tight it comes back with a leg
        through a band anyway; the answer to that is not to widen the band or
        to shave the result, it is to JOIN SOMEWHERE ELSE.
        """
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            L = sn._dist(a, b)
            if L < 1e-9:
                continue
            for ob, cl in zip(obstacles, clears(width_m)):
                if any(ob is x for x in exempt):
                    continue
                sp = seg_box_span(a, b, ob, cl - 0.05)
                if sp is not None and (sp[1] - sp[0]) * L > 0.05:
                    return False
        return True

    def picks(p, width_m, ignore=(), exempt=(), n=3):
        """The junctions *p* could sensibly join to, best first.

        Nearest as the crow flies is the wrong answer when a facility stands in
        between — the router then wraps the connection round the compound and
        back, and a 54 m gap is paved as a 208 m path running alongside the
        spine for a third of its length. So a candidate must have a clear
        straight run to be offered at all, nearest first. The search is capped,
        because a "clear" junction on the far side of the park is not a
        junction anybody would walk to; if nothing inside the cap is clear, the
        nearest is used and the router does what it can.

        More than one is returned so the caller can route to each and take one
        that comes out legal; they are spread out, since two candidates ten
        metres apart on the same edge are the same junction and will fail the
        same way.
        """
        cands = net.candidates(p, ignore=ignore)
        if not cands:
            return []
        cap = cands[0][0] * 3.0 + 20.0
        out = []
        for hit in cands:
            if hit[0] > cap:
                break
            if not walkable(p, hit[3], width_m, exempt=exempt):
                continue
            if any(sn._dist(hit[3], g[3]) < 10.0 for g in out):
                continue
            out.append(hit)
            if len(out) >= n:
                break
        return out or [cands[0]]

    def join_point(p, width_m, ignore=(), exempt=()):
        """Where *p* should meet the network, without routing to it yet."""
        got = picks(p, width_m, ignore=ignore, exempt=exempt, n=1)
        return got[0] if got else None

    def join(p, width_m, ignore=(), exempt=(), tries=4):
        """Where *p* joins the network, the path that gets it there, and
        whether that path came out legal.

        Legality is reported rather than enforced because the two callers want
        different things from a failure: a facility must be connected whatever
        the router manages, while a fountain that cannot be reached without
        walking through a pitch's run-off simply does not get a path of its
        own.
        """
        best = (None, None, False)
        for hit in picks(p, width_m, ignore=ignore, exempt=exempt, n=tries):
            pts = _finish(lay([p, hit[3]], obstacles, clears(width_m),
                              samples=6, exempt=exempt))
            if legal(pts, width_m, exempt=exempt):
                return hit, pts, True
            if best[0] is None:
                best = (hit, pts, False)
        return best

    def tie(nid, width_m, exempt=()):
        """Join node *nid* into the rest of the network — split, then connect.

        THE SINGLE DOOR every path goes through, which is why there is no way
        to end up with one that joins nothing. The target is a point ON AN
        EXISTING EDGE and that edge is CUT there, so the connection is a
        junction and not two paths that happen to touch. Where the node has
        arrived within a couple of metres of the network already, it is welded
        onto the junction instead of being bridged by a stub of path shorter
        than it is wide.

        Edges already at *nid* are ignored, or the nearest point on the network
        would be the path we are trying to connect, at a distance of zero.
        """
        p = net.nodes[nid]
        near = net.nearest(p, ignore=net.adj[nid])
        if near is None:
            return None
        if near[0] <= _WELD_M:
            return net.weld(nid, net.split(near[1], near[2]))
        # Connectivity is not optional, so an illegal best-effort route is
        # still laid here: a facility with no path to it is the worse failure,
        # and `check_pad` is left to say so out loud rather than being quietly
        # satisfied by a facility nobody can reach.
        hit, pts, _ok = join(p, width_m, ignore=net.adj[nid], exempt=exempt)
        if hit is None:
            return None
        mid = net.split(hit[1], hit[2])
        if sn._dist(p, net.nodes[mid]) <= _WELD_M:
            return net.weld(nid, mid)
        net.add_edge(pts, "spur", width_m, a=nid, b=mid)
        return mid

    # THE PATH BETWEEN THE PITCHES. A widened gap alone still leaves two green
    # rectangles touching a strip of the same green; running the main path down
    # the corridor is what makes the eye read two grounds instead of one.
    #
    # A ROUTE BETWEEN TWO JUNCTIONS, not a stripe with two dead ends bridged
    # afterwards. Laying it as a free-standing polyline and then tying each end
    # to the nearest path is how it doubled the spine: the nearest path to a
    # dead end pointing out of the corridor is the spine at the corridor's FAR
    # end, so the connector was routed 60 m back down the corridor it had just
    # been laid beside. Siting both ends on the network first and routing
    # between them cannot do that.
    pitches = [z for z in zones if z["kind"] == "soccer"]
    if len(pitches) >= 2:
        a = math.radians(pitches[0]["yaw"])
        ux, uy = math.cos(a), math.sin(a)
        mx = sum(z["centre"][0] for z in pitches[:2]) / 2.0
        my = sum(z["centre"][1] for z in pitches[:2]) / 2.0
        arm = pl / 2.0 + 12.0
        ea = (mx - ux * arm, my - uy * arm)
        eb = (mx + ux * arm, my + uy * arm)
        ha = join_point(ea, w_spine)
        hb = join_point(eb, w_spine)
        if ha is not None and hb is not None:
            corr = _finish(lay([ha[3], ea, (mx, my), eb, hb[3]], obstacles,
                               clears(w_spine), samples=6))
            # AND ONLY IF THE GAP IS NOT PAVED ALREADY. The tour threads
            # between the things worth reaching and the pitches are two of
            # them, so it quite often runs down this corridor by itself — in
            # which case a second way down it is not a corridor, it is the seam
            # `_dedup` used to be here to cut out. Measured on the ROUTED line,
            # not the straight one, since it is the routed line that gets
            # paved; the ends are trimmed because they are ON the network by
            # construction. `suburb_net` rejects a candidate street on
            # clearance in the same way and for the same reason.
            if (net.clearance(corr, skip_ends_m=10.0) > w_spine
                    and legal(corr, w_spine)):
                na, nb = net.split_at(ha[3]), net.split_at(hb[3])
                if na is not None and nb is not None and na != nb:
                    net.add_edge(corr, "spine", w_spine, a=na, b=nb,
                                 bike_share=share)

    def entrance(z, toward):
        """Boundary point nearest *toward*, and the outward normal of its FACE.

        SQUARE ON. A court is a rectangle, so the path that serves it arrives
        perpendicular to the edge it meets. Using the centre-to-point direction
        as the normal (which is what this did) walks the approach in at
        whatever diagonal the centreline happens to make, and past a corner
        that is visibly not a gate.
        """
        a = math.radians(z["yaw"])
        ux, uy = math.cos(a), math.sin(a)
        d = sn._sub(toward, z["centre"])
        lx0 = d[0] * ux + d[1] * uy
        ly0 = -d[0] * uy + d[1] * ux
        hwz, hhz = z["w"] / 2.0, z["h"] / 2.0
        lx = max(-hwz, min(hwz, lx0))
        ly = max(-hhz, min(hhz, ly0))
        # whichever face it overshot furthest is the face it belongs to
        if abs(lx0) - hwz >= abs(ly0) - hhz:
            s = 1.0 if lx0 >= 0.0 else -1.0
            lx = s * hwz
            n = (ux * s, uy * s)
        else:
            s = 1.0 if ly0 >= 0.0 else -1.0
            ly = s * hhz
            n = (-uy * s, ux * s)
        return local(z, lx, ly), n

    # -- spurs: join each facility to the trunk, arriving on its boundary ----
    # THE ONE PATH ALLOWED THROUGH THE BAND. Padding every facility and then
    # obeying it everywhere would leave nothing able to reach an entrance, so
    # the band has exactly one exception and this is it: the stub that serves
    # the facility crosses its own band, and only its own — `exempt` already
    # existed for precisely this and is what carries it. Everything downstream
    # of the throat is bound like any other path.
    def faces(z):
        """The four face centres of *z* and their outward normals."""
        a = math.radians(z["yaw"])
        ux, uy = math.cos(a), math.sin(a)
        hwz, hhz = z["w"] / 2.0, z["h"] / 2.0
        return [(local(z, hwz, 0.0), (ux, uy)),
                (local(z, -hwz, 0.0), (-ux, -uy)),
                (local(z, 0.0, hhz), (-uy, ux)),
                (local(z, 0.0, -hhz), (uy, -ux))]

    def stub_ok(e, throat, z):
        """Does the straight stub clear every OTHER facility's band?

        The same question `join_point` asks of a candidate junction, which is
        why it is the same predicate: its own facility is exempt because the
        stub starts on that boundary by design.
        """
        return walkable(e, throat, w_spur, exempt=(z["corners"],))

    def reach_m(z):
        # The throat is the first point OUTSIDE the band, so the leg that runs
        # on from it needs no exception at all. Half the spur's width again,
        # because the band is measured off the path edge, plus a metre so the
        # junction itself is not sitting on the line.
        return z["pad_m"] + w_spur / 2.0 + 1.0

    for zi, z in enumerate(zones):
        hit = net.nearest(z["centre"])
        sp = hit[3] if hit is not None else gates[0]
        e, n = entrance(z, sp)
        throat = sn._add(e, sn._mul(n, reach_m(z)))
        # THE GATE GOES WHERE THERE IS ROOM FOR ONE. The face nearest the trunk
        # is the right answer nine times in ten, but two facilities stand as
        # little as 9 m apart and the band is wider than that — so a stub out of
        # the facing side walks straight into the neighbour's padding, which no
        # exemption covers because it serves the other one. Fall back to
        # whichever remaining face is clear, nearest the trunk first; a park
        # puts its gate on the side you can actually walk up to.
        #
        # Choosing the face by whether the LEG ON FROM IT comes out legal, not
        # merely whether the stub has room, was tried as well: over seeds 1-12
        # it changed not one park and cost a third of the run time, because the
        # face nearest the network is the one that gets used almost every time.
        if not stub_ok(e, throat, z):
            for (fe, fn) in sorted(faces(z), key=lambda q: sn._dist(q[0], sp)):
                ft = sn._add(fe, sn._mul(fn, reach_m(z)))
                if stub_ok(fe, ft, z):
                    e, n, throat = fe, fn, ft
                    break
        # The stub runs straight out of the face. Its own facility is exempt —
        # it starts ON that boundary by design — but not its neighbours, which
        # a blind stub between two picnic grounds would walk into.
        #
        # ALWAYS EMITTED, even when the trunk already runs past the entrance.
        # Skipping it there was how facilities ended up with no path touching
        # them at all: the trunk keeps the full band off the boundary, so
        # "close enough" is still a fence away from the gate.
        #
        # The entrance gets a node of its OWN rather than being welded to
        # whatever happens to lie within tolerance: `check_reach` measures this
        # end against the boundary at half a metre, and a node reused from
        # elsewhere would drag it off the facility it is the entrance to.
        stub = net.add_edge(_finish(route([e, throat], obstacles,
                                          clears(w_spur),
                                          exempt=(z["corners"],))),
                            "spur", w_spur, a=net.add_node(e), bike_share=0.0,
                            serves=zi)
        if stub is not None:
            # And the throat joins the network like anything else. There is no
            # branch here for "the trunk is already close enough" — that branch
            # is what left a spur two metres short of the path it was meant to
            # meet, touching its court and connected to nothing.
            tie(stub["b"], w_spur)

    # Set pieces are DESTINATIONS: a fountain with no path to it is as wrong as
    # a court with none. The spur is exempt from the fountain's own clearance
    # box for the same reason a facility stub is exempt from its facility's.
    for (_k, q, box) in feature_pts:
        near = net.nearest(q)
        if near is None or near[0] <= 5.0:
            continue          # the network already passes close enough to it
        hit, pts, ok = join(q, w_spur, exempt=(box,))
        if hit is None or not ok:
            # No way to it that keeps the bands: the set piece keeps its ground
            # and does without a spur, exactly as the pitch corridor does when
            # the gap is already paved. Better an unserved fountain than a path
            # laid through a pitch's run-off.
            continue
        mid = net.split(hit[1], hit[2])
        net.add_edge(pts, "spur", w_spur, a=net.add_node(q), b=mid,
                     bike_share=0.0)

    # NO DE-DUPLICATION PASS. There used to be one here, and deleting it is
    # the point of the graph. Paths were routed against the FACILITIES but not
    # against each other, so the tour spine and the pitch corridor could run
    # over the same ground; the pass clipped a later path wherever it doubled
    # an earlier one and dropped what was left if it came to under three
    # points. It knew nothing about connectivity, so what it deleted was
    # sometimes precisely the stretch a spur had joined onto — which is how a
    # court ended up with a short path coming out of it and going nowhere.
    #
    # A path that MEETS another now joins it at a node instead of being drawn
    # alongside it and cleaned up afterwards, so there is no doubled paving to
    # remove: `check_doubled` measures what is left of it.
    paths = net.paths()

    # -- fences -------------------------------------------------------------
    panel = float(c["fence_panel_m"])
    fences = []
    for z in zones:
        if not z.get("fenced"):
            continue
        cor = z["corners"]
        for i in range(4):
            ax, ay = cor[i]
            bx, by = cor[(i + 1) % 4]
            L = math.hypot(bx - ax, by - ay)
            n = max(1, int(round(L / panel)))
            yaw = math.degrees(math.atan2(by - ay, bx - ax))
            for k in range(n):
                f = (k + 0.5) / n
                fences.append({"c": (ax + (bx - ax) * f, ay + (by - ay) * f),
                               "yaw": yaw})

    # =======================================================================
    # (c) TREES — into what is left, clear of facilities AND of the paths
    # =======================================================================
    # NOT a belt on the boundary rectangle. Trees are massed in COPSES whose
    # density falls off from a centre, so the wood has a soft edge and reads as
    # continuing past the park rather than stopping at a line. That is what
    # blends the park into whatever surrounds it.
    #
    # The path test is on a GRID of every path sample rather than every third
    # one: sampling the polyline sparsely let trees land in the middle of a
    # path wherever the curve happened to be sampled coarsely.
    cell = 12.0
    pgrid = {}
    for pa in paths:
        pts = pa["pts"]
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            L = sn._dist(a, b)
            steps = max(1, int(L / 3.0))
            for k in range(steps + 1):
                t = k / float(steps)
                q = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
                key = (int(math.floor(q[0] / cell)), int(math.floor(q[1] / cell)))
                pgrid.setdefault(key, []).append(q)

    def near_path(q, d):
        gx = int(math.floor(q[0] / cell))
        gy = int(math.floor(q[1] / cell))
        rr = int(math.ceil(d / cell))
        for i in range(gx - rr, gx + rr + 1):
            for j in range(gy - rr, gy + rr + 1):
                for p in pgrid.get((i, j), ()):
                    if sn._dist(q, p) < d:
                        return True
        return False

    def free(q, size, path_d=5.0):
        box = _obb(q[0], q[1], size, size, 0.0)
        if any(_sat_overlap(box, r, 1.0) for r in obstacles):
            return False
        return not near_path(q, path_d)

    n_copse = int(c["copses"])
    for _ in range(n_copse):
        # Copses hug the edges, where a park thickens into its surroundings.
        edge = rng.randrange(4)
        if edge == 0:
            cxy = (rng.uniform(-hw, hw), rng.uniform(-hh, -hh + 60))
        elif edge == 1:
            cxy = (rng.uniform(-hw, hw), rng.uniform(hh - 60, hh))
        elif edge == 2:
            cxy = (rng.uniform(-hw, -hw + 60), rng.uniform(-hh, hh))
        else:
            cxy = (rng.uniform(hw - 60, hw), rng.uniform(-hh, hh))
        rad = rng.uniform(*_rng_pair(c["copse_r_m"], (26.0, 55.0)))
        for _ in range(int(rad * rad * 0.02)):
            # r^0.6 biases inward, so density thins toward the edge instead of
            # ending on a hard circle.
            a = rng.uniform(0, 2 * math.pi)
            d = rad * (rng.random() ** 0.6)
            q = (cxy[0] + d * math.cos(a), cxy[1] + d * math.sin(a))
            if not (-hw <= q[0] <= hw and -hh <= q[1] <= hh):
                continue
            if free(q, 6.0):
                prop("tree", q[0], q[1], rng.uniform(0, 360))

    # A thin scatter over the remaining green, so the open lawn is not bald.
    want = int((ix1 - ix0) * (iy1 - iy0) / 1000.0
               * float(c["lawn_tree_per_1000m2"]))
    tries = 0
    while want > 0 and tries < want * 80:
        tries += 1
        q = (rng.uniform(ix0, ix1), rng.uniform(iy0, iy1))
        if free(q, 8.0):
            prop("tree", q[0], q[1], rng.uniform(0, 360))
            want -= 1

    # =======================================================================
    # (d) PATH FURNITURE — benches and bins, which BELONG TO THE PATH
    # =======================================================================
    # Last, because they are the one thing here whose position is defined by
    # the route rather than by the ground. Placed before the paths were final
    # they sat in the grass beside nothing, facing nowhere. A bench faces the
    # path it is set back from; the bins go on the same line, a good deal
    # sparser, which is the ratio you actually see.
    bspace = float(c["bench_spacing_m"])
    tspace = float(c["trash_spacing_m"])
    foff = float(c["furniture_offset_m"])
    furn = []

    def furn_ok(p):
        box = _obb(p[0], p[1], 2.4, 2.4, 0.0)
        if any(_sat_overlap(box, r, 0.5) for r in obstacles):
            return False
        return not any(sn._dist(p, f) < 7.0 for f in furn)

    for pa in paths:
        if pa["kind"] not in ("spine", "spur"):
            continue
        pts = pa["pts"]
        if len(pts) < 2:
            continue
        side = 1.0
        for (q, t) in _along(pts, bspace, bspace * 0.55):
            side = -side          # benches alternate sides down a path
            p = (q[0] - t[1] * foff * side, q[1] + t[0] * foff * side)
            if not furn_ok(p):
                continue
            furn.append(p)
            # yaw faces back across the path, which is where the view is
            prop("bench", p[0], p[1],
                 math.degrees(math.atan2(t[1], t[0])) - 90.0 * side)
        for (q, t) in _along(pts, tspace, tspace * 0.3):
            p = (q[0] - t[1] * foff, q[1] + t[0] * foff)
            if not furn_ok(p):
                continue
            furn.append(p)
            prop("trash_can", p[0], p[1],
                 math.degrees(math.atan2(t[1], t[0])))

    return {"region": region, "zones": zones, "paths": paths,
            "fences": fences, "props": props}


def check(park, eps=0.25):
    """How many (path segment, facility) pairs actually cross. Must be zero.

    Counts PENETRATION, not contact: a spur is supposed to finish exactly on
    the boundary it serves, so a segment that merely touches the box is
    correct and a segment carrying *eps* metres or more of its length through
    the interior is the bug. That distinction is the reason this measures the
    clipped span rather than asking whether the endpoints are inside.
    """
    n = 0
    for pa in park["paths"]:
        pts = pa["pts"]
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            L = sn._dist(a, b)
            if L < 1e-9:
                continue
            for z in park["zones"]:
                sp = seg_box_span(a, b, z["corners"], 0.0)
                if sp is None:
                    continue
                if (sp[1] - sp[0]) * L > eps:
                    n += 1
    return n


def check_pad(park, eps=0.05):
    """Path segments inside the padding of a facility they do not serve.

    The companion to `check()` and the stricter of the two: crossing a court is
    the outright bug, but grazing the fence is the one that makes the drawing
    look wrong, and until this counted it nothing measured it.

    THREE THINGS THIS GETS RIGHT.

    (a) The band is measured from the path's EDGE. `width_m` is on every path
        and half of it is added to the facility's own pad, because the cycle
        side of a shared-use way runs along the OUTSIDE — a centreline test
        would pass a spine whose bikes are 2.3 m into the grass.
    (b) The serving spur is exempt, and only for the facility it serves. That
        is the whole point of the band having an exception: a facility with a
        band nothing may enter is a facility nobody can reach.
    (c) The predicate is the router's own — the segment clipped against the box
        INFLATED by the band, not distance to a rounded offset region. Testing
        something stricter than the router enforces would report failures the
        router was never asked to prevent.
    """
    n = 0
    for pa in park["paths"]:
        pts = pa["pts"]
        half = float(pa.get("width_m", 2.6)) / 2.0
        serves = pa.get("serves")
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            L = sn._dist(a, b)
            if L < 1e-9:
                continue
            for zi, z in enumerate(park["zones"]):
                if serves is not None and serves == zi:
                    continue
                pad = float(z.get("pad_m", 0.0)) + half
                sp = seg_box_span(a, b, z["corners"], max(0.0, pad - eps))
                if sp is None:
                    continue
                if (sp[1] - sp[0]) * L > eps:
                    n += 1
    return n


def check_reach(park, tol=0.5):
    """(facilities with a path endpoint on their boundary, facilities).

    The padding's failure mode is not a path in the wrong place, it is a
    facility with no path at all — inflate the obstacles and everything is
    beautifully clear of everything and nothing can be walked to. So the band
    is only correct if this comes back with the two numbers equal.
    """
    zones = park["zones"]
    hit = 0
    for z in zones:
        for pa in park["paths"]:
            pts = pa["pts"]
            if len(pts) < 1:
                continue
            if any(_nearest_on_obb(q, z["corners"])[2] <= tol
                   for q in (pts[0], pts[-1])):
                hit += 1
                break
    return hit, len(zones)


def _ends(park):
    """Every path's two ends, as ``(path index, point)``."""
    out = []
    for i, pa in enumerate(park["paths"]):
        pts = pa["pts"]
        if len(pts) < 2:
            continue
        out.append((i, pts[0]))
        out.append((i, pts[-1]))
    return out


_TERMINUS_PROPS = ("fountain", "gazebo", "park_sign")


def _joined(park, p, tol):
    """Is *p* a junction — i.e. does another path END there too?"""
    n = 0
    for (_i, q) in _ends(park):
        if sn._dist(p, q) <= tol:
            n += 1
    return n >= 2


def _at_gate(park, p, gate_m):
    """Is *p* a park gate — i.e. on the boundary, where a path leaves?

    Measured against the park's own rectangle rather than against an exported
    list of gate positions, so it survives the park being translated into its
    reserve (`suburb_scene._shift_park`) and cannot be fooled by stale
    coordinates.
    """
    x0, y0, x1, y1 = park["region"]
    return min(abs(p[0] - x0), abs(p[0] - x1),
               abs(p[1] - y0), abs(p[1] - y1)) <= gate_m


def check_orphans(park, tol=1.0, fac_m=12.0, feat_m=10.0, gate_m=8.0):
    """Path ends that lead nowhere. Must be zero.

    A path end is legitimate in exactly three cases, and they are the three
    places a real park path stops:

      * it is a NETWORK NODE — another path ends at the same point, so this is
        a junction and the walk continues;
      * it is a FACILITY ENTRANCE, i.e. on (or at the gate of) a court, pitch,
        playground or picnic ground;
      * it is a PARK GATE, or a set piece that is itself the destination — a
        fountain, a gazebo, the park sign.

    Anything else is a stub dying in the grass, which is the defect this file
    was rebuilt around. Note the junction test is GEOMETRIC — two ends at the
    same point — not a lookup in the node table the generator happens to have
    exported, so it still measures something when the generator is wrong.
    """
    n = 0
    for (_i, p) in _ends(park):
        if _joined(park, p, tol):
            continue
        if any(_nearest_on_obb(p, z["corners"])[2] <= fac_m
               for z in park["zones"]):
            continue
        if any(sn._dist(p, pr["c"]) <= feat_m for pr in park["props"]
               if pr["kind"] in _TERMINUS_PROPS):
            continue
        if _at_gate(park, p, gate_m):
            continue
        n += 1
    return n


def check_connected(park, tol=1.0):
    """Number of connected components in the path network. Must be 1.

    Paths are joined when they SHARE AN END. Crossing, or running alongside,
    does not count: a network built as a heap of independent polylines can look
    joined on the plan and still be six components, and a junction that is not
    a node is exactly the thing that lets a later edit sever a route without
    anything noticing.
    """
    idx = [i for i, pa in enumerate(park["paths"]) if len(pa["pts"]) >= 2]
    if not idx:
        return 0
    parent = {i: i for i in idx}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    ends = _ends(park)
    for x in range(len(ends)):
        for y in range(x + 1, len(ends)):
            if ends[x][0] == ends[y][0]:
                continue
            if sn._dist(ends[x][1], ends[y][1]) <= tol:
                union(ends[x][0], ends[y][0])
    return len({find(i) for i in idx})


def check_self_crossings(park):
    """Paths that cross THEMSELVES, segment pair by segment pair. Must be zero."""
    n = 0
    for pa in park["paths"]:
        pts = pa["pts"]
        for x in range(len(pts) - 1):
            for y in range(x + 2, len(pts) - 1):
                if sn._seg_intersect(pts[x], pts[x + 1],
                                     pts[y], pts[y + 1]) is not None:
                    n += 1
    return n


def check_doubled(park, slack=0.6):
    """Segment pairs from DIFFERENT paths whose paving overlaps.

    The measure `_dedup` used to exist for. Junction neighbourhoods are exempt:
    two paths that meet at a node necessarily overlap where they meet, and that
    is a junction, not a seam.
    """
    n = 0
    paths = [pa for pa in park["paths"] if len(pa["pts"]) >= 2]
    nodes = [p for (_i, p) in _ends(park)]
    for i in range(len(paths)):
        wa = float(paths[i].get("width_m", 2.6)) / 2.0
        for j in range(i + 1, len(paths)):
            wb = float(paths[j].get("width_m", 2.6)) / 2.0
            lim = wa + wb - slack
            pa, pb = paths[i]["pts"], paths[j]["pts"]
            for x in range(len(pa) - 1):
                for y in range(len(pb) - 1):
                    if sn.seg_seg_dist(pa[x], pa[x + 1],
                                       pb[y], pb[y + 1]) >= lim:
                        continue
                    mid = ((pa[x][0] + pa[x + 1][0] + pb[y][0] + pb[y + 1][0])
                           / 4.0,
                           (pa[x][1] + pa[x + 1][1] + pb[y][1] + pb[y + 1][1])
                           / 4.0)
                    if any(sn._dist(mid, q) <= 6.0 for q in nodes):
                        continue
                    n += 1
    return n


def stats(park):
    from collections import Counter
    z = Counter(x["kind"] for x in park["zones"])
    p = Counter(x["kind"] for x in park["props"])
    return {"zones": dict(z), "props": dict(p),
            "fence_panels": len(park["fences"]),
            "paths": len(park["paths"])}
