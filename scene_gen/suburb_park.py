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
    "bike_loop": True,
    "bike_inset_m": 26.0,
    "bike_w_m": 2.4,
    "copses": 9,
    "copse_r_m": [26.0, 55.0],
    "lawn_tree_per_1000m2": 1.1,
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
    """Lay the park out. Returns a dict of zones, paths, fences and props."""
    c = dict(DEFAULTS)
    c.update(cfg or {})
    W, H = float(c["region_m"][0]), float(c["region_m"][1])
    hw, hh = W / 2.0, H / 2.0
    region = (-hw, -hh, hw, hh)
    buf = float(c["edge_buffer_m"])
    gap = float(c["facility_gap_m"])

    inner = (-hw + buf, -hh + buf, hw - buf, hh - buf)
    zones, placed = [], []
    # One base orientation for the park, with each facility swung off it. A
    # park is not aligned to north; it is aligned to its own site.
    base_yaw = rng.uniform(-22.0, 22.0)

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

    bl, bw = BASKETBALL["court"]
    bpad = BASKETBALL["pad"]

    # PITCHES GO TOGETHER, sharing one orientation and one gap. Scattering them
    # to opposite ends of the park was wrong twice over: clubs lay pitches as a
    # block so one set of goals, parking and drainage serves both, and two
    # pitches at different angles read as an accident rather than as a ground.
    pl, pwd = SOCCER["pitch"]
    n_sc = int(c["n_soccer"])
    if n_sc:
        first = add("soccer", pl, pwd, 0.24, 0.30, spread=12.0)
        if first:
            a = math.radians(first["yaw"])
            ux, uy = math.cos(a), math.sin(a)
            vx, vy = -uy, ux                      # across the pitch
            # +0.5 m, because the pair sits at EXACTLY `gap` and the
            # separating-axis test uses a strict `<`: at equality it reports an
            # overlap and the second pitch was silently dropped every run.
            step = pwd + gap + 0.5
            for i in range(1, n_sc):
                # Try BOTH sides: the first pitch may already sit near an edge,
                # and offsetting blindly to one side put the second pitch
                # outside the park, which silently dropped it.
                for sgn in (1.0, -1.0):
                    cx = first["centre"][0] + vx * step * i * sgn
                    cy = first["centre"][1] + vy * step * i * sgn
                    corners = _obb(cx, cy, pl, pwd, first["yaw"])
                    if _inside(corners, inner) and not any(
                            _sat_overlap(corners, q, gap) for q in placed):
                        placed.append(corners)
                        zones.append({"kind": "soccer", "centre": (cx, cy),
                                      "w": pl, "h": pwd, "yaw": first["yaw"],
                                      "corners": corners})
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
        H = n_tn * th + (n_tn - 1) * 2.0
        z = add("tennis_block", tw, H, 0.90, 0.30, courts=[], fenced=True)
        if z:
            for i in range(n_tn):
                ly = -H / 2 + th / 2 + i * (th + 2.0)
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

    # -- paths: a wandering spine, NOT a rectangle round the edge ----------
    # The perimeter loop was the same mistake as ringing the suburb with an
    # arterial: it drew a racetrack round the scene and made every facility
    # hang off it like a stub. A park is entered at a couple of points and
    # threaded by a route that wanders between the things worth reaching.
    paths = []
    gates = [(0.0, -hh + buf * 0.5), (hw - buf * 0.5, rng.uniform(-hh * 0.2,
                                                                 hh * 0.4))]
    # Waypoints sit OFF each facility, on the side facing the park centre.
    stops = []
    for z in zones:
        cx, cy = z["centre"]
        n = sn._unit(sn._sub((0.0, 0.0), (cx, cy)))
        stops.append((cx + n[0] * (max(z["w"], z["h"]) / 2.0 + 9.0),
                      cy + n[1] * (max(z["w"], z["h"]) / 2.0 + 9.0)))

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
    paths.append({"pts": _curve(way), "kind": "spine"})

    def nearest_on_spine(p):
        best, bp = 1e18, way[0]
        for pt in paths[0]["pts"]:
            d = sn._dist(p, pt)
            if d < best:
                best, bp = d, pt
        return bp

    def entrance(z, toward):
        """Boundary point of the oriented facility nearest *toward*."""
        a = math.radians(z["yaw"])
        ux, uy = math.cos(a), math.sin(a)
        d = sn._sub(toward, z["centre"])
        lx = max(-z["w"] / 2, min(z["w"] / 2, d[0] * ux + d[1] * uy))
        ly = max(-z["h"] / 2, min(z["h"] / 2, -d[0] * uy + d[1] * ux))
        return local(z, lx, ly)

    for z in zones:
        sp = nearest_on_spine(z["centre"])
        e = entrance(z, sp)
        if sn._dist(e, sp) <= 3.0:
            continue
        # STRAIGHT, and square on to the facility. A court is a rectangle, so
        # the path that serves it arrives perpendicular to its edge and runs
        # straight — a curve sweeping past the corner of a fenced compound is
        # not how anyone reaches a gate. The spine may wander; the approach
        # does not.
        n = sn._unit(sn._sub(e, z["centre"]))
        throat = sn._add(e, sn._mul(n, 8.0))
        paths.append({"pts": [e, throat], "kind": "spur"})
        if sn._dist(throat, sp) > 2.0:
            paths.append({"pts": [throat, sp], "kind": "spur"})

    # -- bike lane -----------------------------------------------------------
    # A separate circuit rather than a widened footpath: a park bike route is
    # signed and continuous, and it is the thing that connects the park to the
    # streets outside it, so it runs near the boundary and passes both gates.
    if bool(c.get("bike_loop", True)):
        bi = float(c["bike_inset_m"])
        bx0, by0 = -hw + bi, -hh + bi
        bx1, by1 = hw - bi, hh - bi
        ring = []
        n_seg = 22
        for k in range(n_seg + 1):
            f = k / n_seg
            # A rounded circuit, not a rectangle: it is jittered off a
            # superellipse so it reads as a route rather than a boundary.
            th = 2.0 * math.pi * f
            ex = math.copysign(abs(math.cos(th)) ** 0.62, math.cos(th))
            ey = math.copysign(abs(math.sin(th)) ** 0.62, math.sin(th))
            wob = 1.0 + 0.06 * math.sin(th * 3.0 + base_yaw)
            ring.append(((bx0 + bx1) / 2 + ex * (bx1 - bx0) / 2 * wob,
                         (by0 + by1) / 2 + ey * (by1 - by0) / 2 * wob))
        paths.append({"pts": _curve(ring, samples=8), "kind": "bike"})

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

    # -- props ---------------------------------------------------------------
    props = []

    def prop(kind, x, y, yaw=0.0):
        props.append({"kind": kind, "c": (x, y), "yaw": yaw})

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

    ix0, iy0, ix1, iy1 = inner
    feature_pts = []

    def open_spot(clear, tries=400):
        for _ in range(tries):
            q = (rng.uniform(ix0 + clear, ix1 - clear),
                 rng.uniform(iy0 + clear, iy1 - clear))
            box = _obb(q[0], q[1], clear * 2, clear * 2, 0.0)
            if any(_sat_overlap(box, r, 2.0) for r in placed):
                continue
            if any(sn._dist(q, o) < clear * 2.2 for o in feature_pts):
                continue
            return q
        return None

    n_f = int(rng.uniform(*_rng_pair(c["fountains"], (2.0, 3.0))) + 0.5)
    n_g = int(rng.uniform(*_rng_pair(c["gazebos"], (2.0, 4.0))) + 0.5)
    for _ in range(n_f):
        q = open_spot(15.0)
        if q:
            feature_pts.append(q)
            prop("fountain", q[0], q[1], rng.uniform(0, 360))
    for _ in range(n_g):
        q = open_spot(12.0)
        if q:
            feature_pts.append(q)
            prop("gazebo", q[0], q[1], rng.uniform(0, 360))
    for q in feature_pts:
        sp = nearest_on_spine(q)
        if sn._dist(q, sp) > 4.0:
            paths.append({"pts": _curve([q, sp]), "kind": "spur"})

    prop("park_sign", gates[0][0], gates[0][1] + 3.0, 90.0)

    # -- woodland ------------------------------------------------------------
    # NOT a belt on the boundary rectangle. Trees are massed in COPSES whose
    # density falls off from a centre, so the wood has a soft edge and reads as
    # continuing past the park rather than stopping at a line. That is what
    # blends the park into whatever surrounds it.
    def free(q, clear):
        box = _obb(q[0], q[1], clear, clear, 0.0)
        if any(_sat_overlap(box, r, 1.0) for r in placed):
            return False
        for pa in paths:
            for pt in pa["pts"][::3]:
                if sn._dist(q, pt) < 4.5:
                    return False
        return True

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

    return {"region": region, "zones": zones, "paths": paths,
            "fences": fences, "props": props}


def stats(park):
    from collections import Counter
    z = Counter(x["kind"] for x in park["zones"])
    p = Counter(x["kind"] for x in park["props"])
    return {"zones": dict(z), "props": dict(p),
            "fence_panels": len(park["fences"]),
            "paths": len(park["paths"])}
