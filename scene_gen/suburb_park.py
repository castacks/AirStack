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
    "region_m": [360.0, 260.0],
    "edge_buffer_m": 12.0,        # tree belt inside the park boundary
    "facility_gap_m": 9.0,
    "n_basketball": 4,
    "n_tennis": 3,
    "n_soccer": 1,
    "playground_m": [46.0, 34.0],
    "picnic_areas": 3,
    "picnic_m": [30.0, 22.0],
    "picnic_tables": [5, 9],
    "gazebos": [2, 4],
    "fountains": [2, 3],
    "path_w_m": 3.0,
    "loop_inset_m": 7.0,
    "fence_panel_m": 2.4,         # the sourced chain-link section's run
    "tree_belt_spacing_m": [9.0, 16.0],
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

def _fits(r, placed, inner, gap):
    """Inside *inner* and *gap* clear of everything already placed."""
    if not (inner[0] <= r[0] and r[2] <= inner[2]
            and inner[1] <= r[1] and r[3] <= inner[3]):
        return False
    for q in placed:
        if (r[0] < q[2] + gap and q[0] < r[2] + gap
                and r[1] < q[3] + gap and q[1] < r[3] + gap):
            return False
    return True


def _at(inner, fx, fy, w, h):
    """Rect of *w* x *h* centred at fractional position (fx, fy) of *inner*."""
    x0, y0, x1, y1 = inner
    cx = x0 + (x1 - x0) * fx
    cy = y0 + (y1 - y0) * fy
    return (cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)


def _slide(inner, placed, gap, w, h, fx, fy):
    """Place at (fx, fy), sliding outward in a spiral until it fits.

    A park is COMPOSED, not packed. Shelf-packing filled the bottom of the
    park and left one 200 x 130 m slab of nothing above it, with the picnic
    grounds stranded at the far corner from the courts they belong to. Naming
    each facility's intended position and only searching locally keeps the
    composition while still guaranteeing no overlap.
    """
    r = _at(inner, fx, fy, w, h)
    if _fits(r, placed, inner, gap):
        return r
    step = max(4.0, gap)
    for ring in range(1, 40):
        for ang in range(0, 360, 30):
            a = math.radians(ang)
            d = ring * step
            q = (r[0] + d * math.cos(a), r[1] + d * math.sin(a),
                 r[2] + d * math.cos(a), r[3] + d * math.sin(a))
            if _fits(q, placed, inner, gap):
                return q
    return None


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

    def add(kind, w, h, fx, fy, **extra):
        r = _slide(inner, placed, gap, w, h, fx, fy)
        if r is None:
            return None
        placed.append(r)
        z = {"kind": kind, "rect": r, "centre": _centre(r)}
        z.update(extra)
        zones.append(z)
        return z

    # THE COMPOSITION. Active sport along the south edge where it can be fenced
    # and floodlit without facing houses; passive green to the north; the
    # playground and picnic grounds BETWEEN them and hard against the courts,
    # because that is where parents stand.
    bl, bw = BASKETBALL["court"]
    bpad = BASKETBALL["pad"]

    for _ in range(int(c["n_soccer"])):
        add("soccer", SOCCER["pitch"][0], SOCCER["pitch"][1], 0.22, 0.24)

    n_bb = int(c["n_basketball"])
    if n_bb:
        cols = 2 if n_bb > 1 else 1
        rows = int(math.ceil(n_bb / cols))
        cw = cols * (bl + 2 * bpad) + (cols - 1) * gap
        ch = rows * (bw + 2 * bpad) + (rows - 1) * gap
        z = add("basketball_compound", cw, ch, 0.63, 0.20,
                courts=[], fenced=True)
        if z:
            x0, y0, _x1, _y1 = z["rect"]
            for i in range(n_bb):
                r, q = divmod(i, cols)
                cx = x0 + bpad + bl / 2.0 + q * (bl + 2 * bpad + gap)
                cy = y0 + bpad + bw / 2.0 + r * (bw + 2 * bpad + gap)
                z["courts"].append({"centre": (cx, cy), "yaw": 0.0})

    tw, th = TENNIS["enclosure"]
    n_tn = int(c["n_tennis"])
    if n_tn:
        z = add("tennis_block", tw, n_tn * th + (n_tn - 1) * 2.0, 0.90, 0.28,
                courts=[], fenced=True)
        if z:
            x0, y0, _x1, _y1 = z["rect"]
            for i in range(n_tn):
                z["courts"].append({
                    "centre": (x0 + tw / 2.0,
                               y0 + th / 2.0 + i * (th + 2.0)), "yaw": 0.0})

    pw, ph = _rng_pair(c["playground_m"], (46.0, 34.0))
    add("playground", pw, ph, 0.63, 0.52, surface="sand")

    # Picnic grounds: one beside the playground, one by the courts, the rest
    # out on the green. Adjacency is the whole point -- a picnic ground on the
    # far side of the park from the courts is one nobody uses.
    picnic, spots = [], [(0.45, 0.52), (0.86, 0.55), (0.20, 0.62),
                         (0.72, 0.78)]
    for i in range(int(c["picnic_areas"])):
        mw, mh = _rng_pair(c["picnic_m"], (30.0, 22.0))
        fx, fy = spots[i % len(spots)]
        z = add("picnic", mw, mh, fx, fy, tables=[])
        if z:
            picnic.append(z)

    # -- paths: a perimeter loop plus a spur to every facility --------------
    li = float(c["loop_inset_m"])
    lx0, ly0 = -hw + li, -hh + li
    lx1, ly1 = hw - li, hh - li
    loop = [(lx0, ly0), (lx1, ly0), (lx1, ly1), (lx0, ly1), (lx0, ly0)]
    paths = [{"pts": loop, "kind": "loop"}]

    def nearest_on_loop(p):
        best, bp = 1e18, loop[0]
        for i in range(len(loop) - 1):
            a, b = loop[i], loop[i + 1]
            d = sn._sub(b, a)
            L2 = sn._dot(d, d) or 1.0
            t = max(0.0, min(1.0, sn._dot(sn._sub(p, a), d) / L2))
            q = sn._add(a, sn._mul(d, t))
            dd = sn._dist(p, q)
            if dd < best:
                best, bp = dd, q
        return bp

    def entrance(rect, toward):
        """Point on the rect's boundary nearest *toward*.

        Spurs used to run to the facility CENTRE, which drew the path straight
        across the pitch and through the middle of the basketball compound. A
        path arrives AT a facility; it does not cross it.
        """
        x0, y0, x1, y1 = rect
        return (min(max(toward[0], x0), x1), min(max(toward[1], y0), y1))

    for z in zones:
        c0 = _centre(z["rect"])
        lp = nearest_on_loop(c0)
        e = entrance(z["rect"], lp)
        if sn._dist(e, lp) > 1.0:
            paths.append({"pts": [e, lp], "kind": "spur"})

    # An internal spine so the park is walkable between facilities rather than
    # only out to the loop and back: it threads the playground and the picnic
    # grounds, which is the route people actually take.
    spine = [z for z in zones if z["kind"] in ("playground", "picnic")]
    spine.sort(key=lambda z: z["centre"][0])
    for a, b in zip(spine, spine[1:]):
        pa = entrance(a["rect"], b["centre"])
        pb = entrance(b["rect"], a["centre"])
        paths.append({"pts": [pa, pb], "kind": "spine"})

    # -- fences around the enclosures --------------------------------------
    panel = float(c["fence_panel_m"])
    fences = []
    for z in zones:
        if z.get("fenced"):
            fences.extend(_fence_run(z["rect"], panel))

    # -- props --------------------------------------------------------------
    props = []

    def prop(kind, x, y, yaw=0.0):
        props.append({"kind": kind, "c": (x, y), "yaw": yaw})

    # Hoops: one per court, at each baseline, facing in.
    for z in zones:
        if z["kind"] != "basketball_compound":
            continue
        for court in z["courts"]:
            cx, cy = court["centre"]
            for s in (-1.0, 1.0):
                prop("hoop", cx + s * (bl / 2.0 - BASKETBALL["hoop_inset"]),
                     cy, 180.0 if s > 0 else 0.0)

    # Picnic tables, gridded inside their area rather than scattered: a picnic
    # ground is laid out, not strewn.
    tlo, thi = _rng_pair(c["picnic_tables"], (5.0, 9.0))
    for z in picnic:
        x0, y0, x1, y1 = z["rect"]
        n = int(rng.uniform(tlo, thi + 0.999))
        cols = max(1, int(math.ceil(math.sqrt(n))))
        rows = int(math.ceil(n / cols))
        for i in range(n):
            r, q = divmod(i, cols)
            x = x0 + (x1 - x0) * (q + 0.5) / cols
            y = y0 + (y1 - y0) * (r + 0.5) / rows
            prop("picnic_table", x, y, rng.uniform(-12, 12))
            z["tables"].append((x, y))

    # Playground pieces, spread across the sand.
    for z in zones:
        if z["kind"] != "playground":
            continue
        x0, y0, x1, y1 = z["rect"]
        for i, kind in enumerate(("swing_set", "play_structure", "seesaw",
                                  "seesaw")):
            f = (i + 0.5) / 4.0
            prop(kind, x0 + (x1 - x0) * f,
                 y0 + (y1 - y0) * (0.35 if i % 2 else 0.65),
                 rng.uniform(0, 360))

    # Fountains and gazebos go in the OPEN GREEN, not on the perimeter loop.
    # On the loop they landed inside the tree belt and were swallowed by it,
    # and a park's set pieces belong where the open space is anyway: a fountain
    # is a destination, so it wants lawn around it, not a hedge behind it.
    ix0, iy0, ix1, iy1 = inner

    def open_spot(clear, tries=300):
        for _ in range(tries):
            q = (rng.uniform(ix0 + clear, ix1 - clear),
                 rng.uniform(iy0 + clear, iy1 - clear))
            if any(r[0] - clear < q[0] < r[2] + clear
                   and r[1] - clear < q[1] < r[3] + clear for r in placed):
                continue
            if any(sn._dist(q, o) < clear * 2.0 for o in feature_pts):
                continue
            return q
        return None

    feature_pts = []
    n_f = int(rng.uniform(*_rng_pair(c["fountains"], (2.0, 3.0))) + 0.5)
    n_g = int(rng.uniform(*_rng_pair(c["gazebos"], (2.0, 4.0))) + 0.5)
    for _ in range(n_f):
        q = open_spot(16.0)
        if q:
            feature_pts.append(q)
            prop("fountain", q[0], q[1], rng.uniform(0, 360))
    for _ in range(n_g):
        q = open_spot(13.0)
        if q:
            feature_pts.append(q)
            prop("gazebo", q[0], q[1], rng.uniform(0, 360))

    # Each set piece earns a spur off the loop, so it is reachable rather than
    # marooned in the middle of the grass.
    for q in feature_pts:
        paths.append({"pts": [q, nearest_on_loop(q)], "kind": "spine"})

    # Park sign at the entrance: the middle of the south edge.
    prop("park_sign", 0.0, -hh + buf * 0.4, 90.0)

    # Tree belt round the boundary, plus scatter on the lawn.
    slo, shi = _rng_pair(c["tree_belt_spacing_m"], (9.0, 16.0))
    ring = [(-hw + 4, -hh + 4), (hw - 4, -hh + 4), (hw - 4, hh - 4),
            (-hw + 4, hh - 4), (-hw + 4, -hh + 4)]
    s = 0.0
    total = sn.polyline_length(ring)
    while s < total:
        p = sn.point_at(ring, s)
        # Jittered inward: a belt planted on the exact rectangle reads as a
        # fence of trees rather than as planting.
        t = sn.tangent_at(ring, s)
        n = sn._perp(t)
        d = rng.uniform(-1.5, 7.0)
        prop("tree", p[0] + n[0] * d + rng.uniform(-1.5, 1.5),
             p[1] + n[1] * d + rng.uniform(-1.5, 1.5), rng.uniform(0, 360))
        s += rng.uniform(slo, shi)
    # Scattered over whatever grass is left, by rejection against the
    # facilities and the path loop -- there is no lawn RECT to sample inside,
    # because the lawn is the negative space.
    ix0, iy0, ix1, iy1 = inner
    want = int((ix1 - ix0) * (iy1 - iy0) / 1000.0
               * float(c["lawn_tree_per_1000m2"]))
    tries = 0
    while want > 0 and tries < want * 60:
        tries += 1
        q = (rng.uniform(ix0, ix1), rng.uniform(iy0, iy1))
        if any(r[0] - 4 < q[0] < r[2] + 4 and r[1] - 4 < q[1] < r[3] + 4
               for r in placed):
            continue
        if min(abs(q[0] - lx0), abs(q[0] - lx1),
               abs(q[1] - ly0), abs(q[1] - ly1)) < 5.0:
            continue                      # keep the loop path clear
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
