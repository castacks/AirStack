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
    "facility_gap_m": 9.0,
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
    "path_clear_m": 5.0,     # a path keeps this far off any facility
    # Shared-use path: one way, divided. The spine carries both modes; a spur
    # into a single facility does not need a cycle side.
    "path_w_shared_m": 4.6,   # 2.6 m walking + 2.0 m cycle side
    "path_w_spur_m": 2.6,
    "bike_share": 0.44,       # share of the shared path given to the cycle side
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
        out.append([(base, -y), (hoop - s * dx, -y)])
        out.append([(base, y), (hoop - s * dx, y)])
        a = math.atan2(y, -s * dx)
        out.append(_arc(hoop, 0, r3, -a if s > 0 else a,
                        a if s > 0 else 2 * math.pi - a))
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
    lo, hi = (0, len(out)) if ends else (1, max(1, len(out) - 1))
    for _ in range(iters):
        moved = False
        for i in range(lo, hi):
            q = out[i]
            for ob in obstacles:
                if any(ob is e for e in exempt):
                    continue
                near, inside, d = _nearest_on_obb(q, ob)
                if inside or d < clear:
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
                    out[i] = sn._add(near, sn._mul(n, clear))
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


def _bypass(a, b, frame, pad):
    """Corner chain taking a-b round a box, the shorter of the two ways.

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
        return []          # an endpoint is pinned inside: nothing legal to do
    n = len(h)
    ia, ib = h.index(a), h.index(b)
    best, blen = None, None
    for stepd in (1, -1):
        ch, k = [], (ia + stepd) % n
        while k != ib and len(ch) < n:
            ch.append(h[k])
            k = (k + stepd) % n
        seq = [a] + ch + [b]
        L = sum(sn._dist(seq[i], seq[i + 1]) for i in range(len(seq) - 1))
        if blen is None or L < blen:
            best, blen = ch, L
    return best or []


def _lift(p, recs, margin, iters=6):
    """Shove a point out of every box it is inside, by the shortest way.

    In the box's own frame the escape is just whichever slab it is nearest to
    leaving, which is both cheaper and better behaved than projecting onto the
    nearest boundary point — the projection has no sign of its own and pushed
    interior points further in.
    """
    for _ in range(iters):
        moved = False
        for (ctr, u, v, hw, hh) in recs:
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


def _first_hit(a, b, recs, clear):
    """The obstacle a-b runs into SOONEST, so detours resolve in travel order."""
    hit, ht = None, 2.0
    for frame in recs:
        sp = _span(a, b, frame, clear)
        if sp is None or sp[1] - sp[0] < 1e-9:
            continue
        if sp[0] < ht:
            ht, hit = sp[0], frame
    return hit


def _route_seg(a, b, recs, clear, budget):
    """One segment, detoured round everything it meets.

    ITERATIVE, NOT RECURSIVE. Recursing on each of the five sub-segments a
    bypass produces fans out as 5^depth and hung the generator outright — the
    work is quadratic in the obstacles at worst, but the recursion made it
    exponential in nothing but its own bookkeeping. Splicing corner chains
    into a single chain and rescanning from the splice does the same job in a
    bounded number of passes.
    """
    chain = [a, b]
    i, stall = 0, 0
    while i < len(chain) - 1 and budget[0] > 0:
        hit = _first_hit(chain[i], chain[i + 1], recs, clear)
        if hit is None:
            i += 1
            stall = 0
            continue
        ins = _bypass(chain[i], chain[i + 1], hit, clear + _CORNER_M)
        # A corner taken round ONE facility can land inside the NEXT one, and
        # nothing else ever revisits it: the pre-pass only saw the original
        # waypoints, and the detour that would fix it needs an endpoint that is
        # not already inside a box. Lifting each spliced corner as it is
        # created is what stops a bike route pausing for two segments in the
        # middle of a pitch.
        ins = [_lift(q, recs, clear * 0.5) for q in ins]
        budget[0] -= 1
        stall += 1
        # A bypass that cannot help — the endpoint is pinned inside the box —
        # would otherwise splice for ever at the same index.
        if not ins or stall > 6 or len(chain) > 240:
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
    obs = [o for o in obstacles if not any(o is e for e in exempt)]
    # Push clear of the BYPASS inflation, not just of `clear`: a point lifted
    # to exactly `clear` still sits inside the box the detour corners are taken
    # from, so it would not appear on that hull and the bypass would give up.
    seed = avoid(pts, obs, clear + _CORNER_M + 1.0, iters=8, ends=ends)
    recs = [_box_frame(o) for o in obs]
    bud = [budget]
    out = [seed[0]]
    for i in range(len(seed) - 1):
        out.extend(_route_seg(seed[i], seed[i + 1], recs, clear, bud)[1:])
    out = _dedupe(out)
    # REPAIR PASS. `clear` is a comfort margin and missing it is untidy; being
    # inside the facility at all is the actual defect, so anything still
    # penetrating gets one more detour at a clearance it cannot argue with.
    # Segments that are already fine cost one slab clip each, so this is nearly
    # free — it only does work where there is something left to fix.
    bud2 = [budget]
    rep = [out[0]]
    for i in range(len(out) - 1):
        rep.extend(_route_seg(out[i], out[i + 1], recs, 0.6, bud2)[1:])
    return _dedupe(rep)


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
    coarse = route(way, obstacles, clear + 3.0, exempt=exempt, ends=ends)
    if len(coarse) < 2:
        return coarse
    smooth = _curve(coarse, samples=samples)
    return route(smooth, obstacles, clear, exempt=exempt, ends=ends)


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
    gap = float(c["facility_gap_m"])
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
        cw = cols * (bl + 2 * bpad) + (cols - 1) * gap
        ch = rows * (bw + 2 * bpad) + (rows - 1) * gap
        z = add("basketball_compound", cw, ch, 0.66, 0.22, courts=[],
                fenced=True)
        if z:
            for i in range(n_bb):
                r, q = divmod(i, cols)
                lx = -cw / 2 + bpad + bl / 2 + q * (bl + 2 * bpad + gap)
                ly = -ch / 2 + bpad + bw / 2 + r * (bw + 2 * bpad + gap)
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

    # =======================================================================
    # (b) PATHS — everything above is now an obstacle
    # =======================================================================
    # The spine wanders, NOT a rectangle round the edge. The perimeter loop was
    # the same mistake as ringing the suburb with an arterial: it drew a
    # racetrack round the scene and made every facility hang off it like a
    # stub. A park is entered at a couple of points and threaded by a route
    # that wanders between the things worth reaching.
    obstacles = placed + features
    paths = []

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
    way, rest, cur = [gates[0]], list(stops), gates[0]
    while rest:
        nxt = min(rest, key=lambda q: sn._dist(cur, q))
        rest.remove(nxt)
        way.append(nxt)
        cur = nxt
    way.append(gates[1])
    spine = lay(way, obstacles, clear, samples=10)
    paths.append({"pts": spine, "kind": "spine"})
    trunk = [spine]

    # THE PATH BETWEEN THE PITCHES. A widened gap alone still leaves two green
    # rectangles touching a strip of the same green; running the main path down
    # the corridor is what makes the eye read two grounds instead of one.
    pitches = [z for z in zones if z["kind"] == "soccer"]
    if len(pitches) >= 2:
        a = math.radians(pitches[0]["yaw"])
        ux, uy = math.cos(a), math.sin(a)
        mx = sum(z["centre"][0] for z in pitches[:2]) / 2.0
        my = sum(z["centre"][1] for z in pitches[:2]) / 2.0
        arm = pl / 2.0 + 12.0
        corr = lay([(mx - ux * arm, my - uy * arm), (mx, my),
                    (mx + ux * arm, my + uy * arm)],
                   obstacles, clear, samples=6, ends=True)
        paths.append({"pts": corr, "kind": "spine"})
        trunk.append(corr)

    def nearest_on_trunk(p):
        best, bp = 1e18, gates[0]
        for pts in trunk:
            for pt in pts:
                d = sn._dist(p, pt)
                if d < best:
                    best, bp = d, pt
        return bp

    # Tie the pitch corridor back into the spine at both ends, so it is part of
    # the network rather than a stripe lying across the grass.
    for pts in trunk[1:]:
        for end in (pts[0], pts[-1]):
            best, bp = 1e18, None
            for pt in spine:
                d = sn._dist(end, pt)
                if d < best:
                    best, bp = d, pt
            if bp and best > clear + 2.0:
                paths.append({"pts": route([end, bp], obstacles, clear,
                                           ends=False), "kind": "spur"})

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
    for z in zones:
        sp = nearest_on_trunk(z["centre"])
        e, n = entrance(z, sp)
        if sn._dist(e, sp) <= clear + 8.0:
            continue
        throat = sn._add(e, sn._mul(n, clear + 7.0))
        # The stub runs straight out of the face. Its own facility is exempt —
        # it starts ON that boundary by design — but not its neighbours, which
        # a blind 12 m stub between two picnic grounds would walk into.
        paths.append({"pts": route([e, throat], obstacles, clear,
                                   exempt=(z["corners"],)), "kind": "spur"})
        if sn._dist(throat, sp) > 2.0:
            paths.append({"pts": lay([throat, sp], obstacles, clear,
                                     samples=6), "kind": "spur"})

    for (_k, q, box) in feature_pts:
        sp = nearest_on_trunk(q)
        if sn._dist(q, sp) <= 5.0:
            continue
        paths.append({"pts": lay([q, sp], obstacles, clear, samples=6,
                                 exempt=(box,)), "kind": "spur"})

    # NO SEPARATE BIKE NETWORK. A second circuit that had to weave past the
    # first produced exactly the tangle it sounds like wherever the two met:
    # two independent routes crossing at shallow angles, each dodging the
    # other's obstacles. A park path is ONE wider way carrying both modes,
    # divided down its length -- which is also how shared-use paths are
    # actually built. `width_m` and `bike_share` below say how wide each path
    # is and how much of it is the cycle side; the divider is drawn from those,
    # so there is only ever one network to route and one set of junctions.
    for pa in paths:
        w = float(c["path_w_shared_m"] if pa["kind"] == "spine"
                  else c["path_w_spur_m"])
        pa["width_m"] = w
        pa["bike_share"] = float(c["bike_share"]) if pa["kind"] == "spine" else 0.0

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


def stats(park):
    from collections import Counter
    z = Counter(x["kind"] for x in park["zones"])
    p = Counter(x["kind"] for x in park["props"])
    return {"zones": dict(z), "props": dict(p),
            "fence_panels": len(park["fences"]),
            "paths": len(park["paths"])}
