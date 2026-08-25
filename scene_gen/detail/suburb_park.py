"""
suburb_park.py — layout for a large suburban park.

WHAT THIS OWNS
--------------
The park is not a block with trees on it. It is a set of FACILITIES at real
dimensions, a path network that reaches each one, and the leftover green
between them — and the facilities are what fix the sizes, not the other way
round. A basketball court is 28.65 x 15.24 m whatever the park wants, so the
layout is built to those figures and lets the open lawn take the remainder.

Sourced assets cover the objects; the SURFACES are generated, because a court
is a painted rectangle and no asset library has the one that fits your park:

    from assets   hoop, fence panel, gazebo, fountain, park sign, picnic
                  table, swing set / play structure / seesaw, tennis court
    generated     court slabs and their line markings, soccer pitch and its
                  markings, playground sand, path network, fence runs,
                  the refuge parking lot — slab, bay lines and apron

DIMENSIONS, WHICH ARE NOT NEGOTIABLE
------------------------------------
    basketball   28.65 x 15.24 m court (94 x 50 ft), hoop 1.575 m in from the
                 baseline, 3-point arc 6.75 m, key 4.88 x 5.79 m
    tennis       23.77 x 10.97 m doubles court, 8.23 m singles width, service
                 line 6.40 m from the net; fenced enclosure 36.58 x 18.29 m
    soccer       park pitch, inside the 90-120 x 45-90 m the Laws allow:
                 100 x 64 m with a 9.15 m centre circle, 16.5 m penalty area
                 and 5.5 m goal area
    parking      2.7 x 5.5 m bay (9 x 18 ft) served by a 7.0 m aisle — the
                 width a 90-degree stall needs to back out into — so a
                 double-loaded module is 5.5 + 7.0 + 5.5 = 18.0 m deep

THE REFUGE LOT IS A FACILITY, NOT A CAR PARK BOLTED ON
------------------------------------------------------
The park carries one large parking lot, and it is here because of what the
scene is FOR: in the wildfire evacuations this dataset is modelled on, the
"temporary refuge areas" people were directed to were almost all parking
lots — a hectare of asphalt with nothing on it to burn, next to open ground,
reachable by road. A later pass puts groups of survivors and a few cars on
it, so what it has to publish is not a slab but a SCHEDULE OF BAYS.

That fixes three things about where it goes:

    against the frame street   a lot you cannot drive into is not a refuge.
        It sits `parking_setback_m` inside the park boundary on the side of
        a REAL entrance — `plan` is handed `suburb_net`'s `park_entrances`
        and picks the one nearest the courts — and an apron at least 7 m wide
        crosses the verge from the kerb to the lot's street face.
    on the sports side         because that is where the entrances and the
        hard surface already are, and because a lot dropped in the middle of
        the greensward is the one thing that would read as generated.
    as an oriented rectangle   so it claims ground with the courts, paths
        route round it, trees are rejection-sampled off it, and the picnic
        and playground pads keep their clearance. It is in `zones`, so every
        pass that already respects a facility respects this one for free.

The layout is the ordinary double-loaded one: rows of bays along the
frontage, a 7 m aisle between each back-to-back pair, and an end lane at
each end joining one aisle to the next — that ring is the internal loop, and
it is what the apron feeds. At the default 64 x 40 m that is 2 modules x 2
rows x 18 bays = 72, i.e. 35.6 m2 a bay including circulation, which is what
a surface lot actually costs.

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

PLANTING IS A COMPOSITION, NOT A COUNT
--------------------------------------
Density is stated TWICE: a margin rate for the belt inside the
boundary and for the strips between one facility and the next, and a fraction
of it for the open greensward in between. A park at one uniform rate is either
an orchard or a mown field, and this one was the mown field — 11 stems/ha,
against the 40-120/ha a municipal park's planted ground carries and the 85/ha
the undeveloped-land pass in this same scene uses. See `lawn_tree_per_1000m2`,
`tree_open_frac` and `tree_margin_m`.

THE FOUNTAINS ARE PLAZAS, NOT POINTS
------------------------------------
Ported from the urban park (`parks.py`, "THE ATTRACTION AS A COMPOSED
CENTREPIECE"): a ring path outside the water, benches standing on it facing
in, and no walk running at the fountain — the ring is what you arrive on and
what carries you round. The clearing round it comes free, because trees are
rejection-sampled 5 m off every path and the ring is a path.
"""

import math

from layout import suburb_net as sn

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

# The refuge lot, at the same standing as the courts above: these are the
# figures a surface car park is actually striped to, not preferences.
#
#   bay     2.7 x 5.5 m is the 9 x 18 ft standard stall. Narrower (2.4 m) is a
#           compact bay and wrong for a lot people will be told to shelter on.
#   aisle   7.0 m is the two-way aisle a 90-degree stall needs to back out
#           into; below about 6.7 m the manoeuvre takes two bites.
#   loop    the end lane joining one aisle to the next. Same 7.0 m, because it
#           carries the same traffic and a lane that narrows at the turn is
#           where a lot jams — which is the one failure mode a refuge may not
#           have.
#   edge    kerb strip between the outermost bay's back line and the lot
#           boundary, so a wheel stop has somewhere to be.
PARKING = {
    "bay": (2.7, 5.5),
    "aisle": 7.0,
    "loop": 7.0,
    "edge": 1.0,
}

DEFAULTS = {
    "region_m": [420.0, 300.0],
    "edge_buffer_m": 12.0,        # tree belt inside the park boundary
    # DERIVED, not chosen. A shared path threading between two facilities needs
    # both their padding bands plus its own width: 2 x 8 + 4.6 = 20.6 m at the
    # fenced compounds. The old flat 9.0 left 9.2-13.9 m corridors the path
    # could not physically fit down, and 17 padding breaches with them.
    # Widening at PLACEMENT time makes them passable by construction; see
    # facility_gap().
    "facility_gap_m": 0.0,      # 0 = derive from the padding and path width
    # Between COURTS inside one compound -- they share a slab and a fence, so
    # this is a couple of metres of sideline run-off, not a path corridor.
    "court_gap_m": 2.5,
    # The pitches sit side by side but must READ as two. At the bare facility
    # gap they abut into one 138 m slab of green with two sets of markings on
    # it. A real ground leaves a walkable margin between pitches, with a path
    # down it.
    "pitch_gap_m": 30.0,
    "n_basketball": 4,
    "n_tennis": 3,
    "n_soccer": 2,
    "playground_m": [46.0, 34.0],
    # ---- refuge parking lot ------------------------------------------------
    # SIZED BY THE BAY COUNT, not by taste. The depth is what fixes the rows:
    # 40 m holds exactly two 18 m double-loaded modules with a metre of kerb
    # strip either side, and 58 m would hold three but is a car park with a
    # park round it rather than the other way about. The frontage is then what
    # fixes the bays per row — 64 m less the two 7 m end lanes is a 50 m strip,
    # 18 bays at 2.7 m — so the lot lands at 2 x 2 x 18 = 72 bays on 2,560 m2.
    # A groups-of-survivors pass wants 70-90 marked bays to work with; 60 m
    # frontage gives 68 and 68 is under it, which is the whole reason this is
    # 64 and not the round number.
    "parking": True,
    "parking_m": [64.0, 40.0],
    "parking_bay_m": [2.7, 5.5],
    "parking_aisle_m": 7.0,
    "parking_loop_m": 7.0,
    "parking_edge_m": 1.0,
    # Grass between the park boundary and the lot's street face. Not zero: the
    # boundary is where the park's own planting belt and its fence line would
    # go, and a slab flush to it reads as the street having been widened.
    "parking_setback_m": 3.0,
    # THE APRON, and 7 m is a floor rather than a figure. Two cars must pass in
    # it — one arriving at a refuge while another leaves is the case it exists
    # for — so it is the aisle width, plus a metre because a bellmouth flares.
    "parking_apron_w_m": 8.0,
    # ...and how long it is allowed to get. Nominally the drive is the verge
    # plus the setback (20 + 3 = 23 m); anything much past that means the lot
    # has slid a long way off its entrance and is being served by a diagonal,
    # so the search moves to the next entrance instead. See `_place_parking`.
    "parking_apron_max_m": 45.0,
    # How far outside the park boundary the kerb is, i.e. how long the apron
    # has to be. Overridden by the real entrance when `entrances` is supplied;
    # this is only the standalone fallback, and it matches `suburb_net`'s
    # `park_pad_m` (the verge no street centreline may enter).
    "parking_kerb_m": 20.0,
    # The lot may sit outside `edge_buffer_m` — that band is the tree belt, and
    # a lot AGAINST the street is the point. It keeps this much off the park
    # rect instead, which is only what stops the slab bleeding over the edge.
    "parking_rect_inset_m": 3.0,
    # See `facility_pad`: a kerb, not a fence, so the band is a verge.
    "parking_pad_m": 3.0,
    "picnic_areas": 3,
    "picnic_m": [30.0, 22.0],
    # A 30 x 22 m picnic ground is 660 m2; six tables on it is one per 110 m2,
    # i.e. grass with some tables on it. A municipal picnic ground runs one
    # table per 55-70 m2 — two families in earshot, benches not touching — so
    # the same ground carries 9-12.
    "picnic_tables": [7, 12],
    "gazebos": [2, 4],
    "fountains": [2, 3],
    "fence_panel_m": 2.4,         # the sourced chain-link section's run
    "path_clear_m": 5.0,     # a path keeps this far off a set piece
    # CLEARANCE BAND ROUND A FACILITY. Not crossing a court was never the whole
    # requirement — a path that grazes the fence is still wrong, and the fence
    # has to stand somewhere. Measured from the path's EDGE, not its
    # centreline: the shared-use way is 4.6 m wide with the cycle side on the
    # OUTSIDE, so a centreline figure under-measures clearance by half the
    # width and puts the bikes in the margin the band exists to keep.
    "facility_pad_m": 6.0,
    # PER KIND, and the extra on a fenced compound is not decoration: the
    # chain-link stands ON the boundary line, its gate swings outward, and the
    # bike racks are parked 4.5 m off the same face (`bike_racks_per_court`,
    # placed against the compound below). 6 m would run the path straight
    # through the racks; 8 m clears them with 1.5 m of grass over. A picnic
    # ground or a pitch has nothing standing on its edge, so the base band
    # does.
    "facility_pad_fenced_m": 8.0,
    # Shared-use path: one way, divided. The spine carries both modes; a spur
    # into a single facility does not need a cycle side.
    "path_w_shared_m": 4.6,   # 2.6 m walking + 2.0 m cycle side
    "path_w_spur_m": 2.6,
    "bike_share": 0.44,           # share of the way given to the cycle side
    "bike_racks_per_court": 4,
    # ---- planting ----------------------------------------------------------
    # HOW MANY TREES A PARK HAS, the one figure here that was simply wrong. The
    # lawn scatter ran at 1.1 per 1000 m2 = 11 stems/ha; over 12.6 ha that is
    # 273 trees including the copses, 22/ha — a mown field with a fringe. For
    # scale: the undeveloped-land pass in this same scene runs 85/ha and reads
    # as light woodland, and a municipal park's PLANTED ground (the belt inside
    # the boundary, the strips between one facility and the next) runs
    # 40-120/ha with the open lawn left open between.
    #
    # So the density is stated twice: `lawn_tree_per_1000m2` is the MARGIN rate
    # and `tree_open_frac` what survives of it out on the greensward, which is
    # what stops a park-density scatter turning into an orchard.
    # `tree_margin_m` is how far in from a boundary or off a facility the
    # margin reaches — a crown here is ~7 m across, so four rows of it, the
    # belt you actually see round a sports ground.
    "copses": 10,
    "copse_r_m": [26.0, 55.0],
    # A copse is a WOOD, so it is planted at the woodland rate rather than the
    # park rate — against the 85/ha the undeveloped-land pass uses, thinned by
    # the r^0.6 bias toward its centre and by whatever it lands on. The old
    # hardcoded 0.02 * r^2 was 64/ha of ATTEMPTS, i.e. under 50/ha placed.
    "copse_per_1000m2": 8.5,
    # NOT ALL ON THE EDGE. Copses hugging the boundary exclusively is right for
    # the screen belt and is exactly why the MIDDLE of the park came out bald:
    # open lawn at an 11/ha scatter between the pitches and the courts. A third
    # go inland, smaller, into the gaps between facilities — a stand, not a
    # wood.
    "copse_inland_share": 0.34,
    "lawn_tree_per_1000m2": 3.8,
    "tree_open_frac": 0.26,
    "tree_margin_m": 28.0,
    # Crowns are ~7 m across on the plan; two stems closer than this read as
    # one blob. Not a hard forest rule — real stands do touch — just enough to
    # keep a Poisson scatter from clumping into rosettes at park density.
    "tree_spacing_m": 4.6,
    # ---- playground --------------------------------------------------------
    # One piece of kit per this much sand. A 46 x 34 m playground is 1,564 m2
    # and lands at nine pieces, what a neighbourhood playground that size
    # actually carries; the old pass put FOUR on it in a single row.
    "play_area_per_piece_m2": 170.0,
    # ...of which at most this many may be the two expensive assets. Per
    # `park.yaml` the swing set and the Kids Place are 75.9k and 79.5k faces,
    # and `play_structure` is not in `suburb_scene`'s instanced categories, so
    # the fifth one is another 79.5k faces and not a transform. The seesaw is
    # 498 faces and carries the rest.
    "play_expensive_cap": 4,
    "play_bench_pitch_m": 26.0,   # round the inside of the fence, facing in
    # ---- fountain plaza ----------------------------------------------------
    # Radius of the ring path round a fountain, ported from the urban park (see
    # `parks.py`, "THE ATTRACTION AS A COMPOSED CENTREPIECE"). The fountain art
    # is 4-5 m across, so 8 m leaves ~5.5 m of standing ground between water
    # and paving; the benches sit `furniture_offset_m` outside the ring, the
    # same set-back a bench keeps from any other path here.
    "fountain_ring_r_m": 8.0,
    "fountain_bench_pitch_m": 9.0,
    # ---- path furniture ----------------------------------------------------
    # 34 m between benches is a TRAIL figure. On a park path the spacing you
    # see is 25-30 m — the next bench always visible from the one you are on,
    # which is the rest-every-30 m rule of thumb accessible-route guidance
    # uses.
    "bench_spacing_m": 27.0,      # along a path, alternating sides
    # Left near its old figure DELIBERATELY. The bins were not sparse before,
    # they were being REFUSED — `furn_ok` held everything 7 m off everything
    # and the benches went down first, so seed 1 kept only 13 of the ~22 the
    # spacing asks for. With the bin's own radius fixed (see `furn_ok`), 78
    # gave 27 bins to 50 benches, a bin at every other seat and one more than a
    # park needs; 75 lands at roughly 1 : 2.3, the ratio you count walking
    # round a real one.
    "trash_spacing_m": 75.0,
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


def _merge_spans(spans, tol=1e-6):
    """Union of 1-D intervals that touch or overlap."""
    out = []
    for (a, b) in sorted(spans):
        if out and a <= out[-1][1] + tol:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return [(a, b) for (a, b) in out]


def parking_layout(w, d, bay=None, aisle=None, loop=None, edge=None):
    """Bays and paint for a *w* x *d* surface lot, in LOCAL coords.

    LOCAL +X RUNS ALONG THE STREET FRONTAGE and local +y into the park, so the
    apron always meets the -y face. Everything here is generated for the same
    reason the court slabs are: a lot is a striped rectangle at a fixed stall
    size and no asset library has the one that fits a given park.

    THE LAYOUT IS THE ORDINARY DOUBLE-LOADED ONE, and it is worth saying why it
    is not the obvious "fill the rectangle with rows":

        rows along the FRONTAGE, not across it. Across it, each aisle would
        dead-end on the street face and on the far boundary, and a lot whose
        aisles have no through route is a lot that gridlocks the moment two
        cars meet — which for a refuge is the failure that matters.
        an END LANE at each end, `loop` wide. That is the internal loop: aisle
        -> end lane -> next aisle -> other end lane, a closed ring, and the
        apron feeds straight into one end lane rather than into the back of a
        row of bays.
        rows BACK TO BACK across a module boundary, so two rows share one back
        line — which is both how a lot is striped and half the line meshes.

    Returns ``{"bays", "lines", "lanes", "n_bay", "rows", "module_m"}`` — bays
    as ``(lx, ly, face)`` with *face* the local-y direction a nosed-in car
    points, lines as local polylines in the same form the court markings use,
    lanes as the two end-lane centre offsets in local x. None if the rectangle
    is too small to stripe.
    """
    bw, bd = (bay or PARKING["bay"])
    bw, bd = float(bw), float(bd)
    aisle = PARKING["aisle"] if aisle is None else float(aisle)
    loop = PARKING["loop"] if loop is None else float(loop)
    edge = PARKING["edge"] if edge is None else float(edge)

    module = 2.0 * bd + aisle
    usable = d - 2.0 * edge
    n_mod = int(usable // module) if module > 0 else 0
    single = False
    if n_mod < 1:
        # Too shallow for a double-loaded module: one row against one aisle.
        # Not a fallback anybody should hit at the shipped size — it is here so
        # a knob set to something silly degrades instead of dividing by zero.
        module = bd + aisle
        if usable < module:
            return None
        n_mod, single = 1, True

    block = n_mod * module
    y0 = -block / 2.0
    strip = w - 2.0 * loop
    n_bay = int(strip // bw) if bw > 0 else 0
    if n_bay < 1:
        return None
    used = n_bay * bw
    x0 = -used / 2.0

    rows = []
    for m in range(n_mod):
        base = y0 + m * module
        # back line on `base`, mouth `bd` further in: a car nosed in points -y
        rows.append((base, base + bd, -1.0))
        if not single:
            top = base + module
            rows.append((top, top - bd, 1.0))

    bays, backs, div = [], {}, {}
    for (yb, yf, face) in rows:
        mid = (yb + yf) / 2.0
        for i in range(n_bay):
            bays.append((x0 + (i + 0.5) * bw, mid, face))
        for i in range(n_bay + 1):
            div.setdefault(round(x0 + i * bw, 4), []).append(
                (min(yb, yf), max(yb, yf)))
        # Keyed by y, so the two rows of a back-to-back pair collapse to the
        # one line they are actually striped with.
        backs[round(yb, 4)] = (x0, x0 + used)

    lines = [[(a, y), (b, y)] for (y, (a, b)) in sorted(backs.items())]
    for x, spans in sorted(div.items()):
        for (a, b) in _merge_spans(spans):
            lines.append([(x, a), (x, b)])

    return {"bays": bays, "lines": lines, "n_bay": n_bay, "rows": len(rows),
            "module_m": module,
            "lanes": [(-w / 2.0 + x0) / 2.0, (w / 2.0 + x0 + used) / 2.0]}


# ---------------------------------------------------------------------------
# oriented-box placement — a facility asks for a spot and slides off it only
# when something is already there (the shelf packer this replaced is in the
# module docstring, under "LAYOUT IS COMPOSED, NOT PACKED")
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


def to_world(z, p):
    """A point in facility *z*'s own frame, in park coords."""
    a = math.radians(z["yaw"])
    ux, uy = math.cos(a), math.sin(a)
    return (z["centre"][0] + ux * p[0] - uy * p[1],
            z["centre"][1] + uy * p[0] + ux * p[1])


def to_local(z, p):
    """Inverse of :func:`to_world`.

    EVERYTHING THE PARKING LOT PUBLISHES IS STORED LOCAL, and that is not a
    style choice: `suburb_scene._shift_park` translates a park built at the
    origin into its reserve by moving `centre`, `corners`, path points, fences
    and props, and it knows nothing about bays or aprons. Local offsets ride
    along with `centre` for free, so the lot cannot come apart in the shift —
    which is exactly the class of bug the court markings avoid by being local
    line lists rather than world polylines.
    """
    a = math.radians(z["yaw"])
    ux, uy = math.cos(a), math.sin(a)
    dx, dy = p[0] - z["centre"][0], p[1] - z["centre"][1]
    return (dx * ux + dy * uy, -dx * uy + dy * ux)


def _place_parking(rect, placed, gap, w, d, gate, inward, kerb_pt, setback,
                   lanes, apron_w, apron_max=45.0, keep_out=(), keep_m=10.0,
                   slide_m=120.0, push_m=45.0, step=3.0):
    """Site the refuge lot against the frontage at *gate*.

    NOT `_place`. The other facilities spiral outward from a fraction of the
    park and are free to swing off the base yaw, because nothing outside the
    park cares which way a picnic ground faces. This one is pinned by the
    STREET: its yaw is the frontage's, its street face stays `setback` in from
    the park boundary, and the apron still has to reach the kerb after it
    moves. So the only freedoms are a slide ALONG the frontage and a push
    inland, and they are not equal freedoms.

    THE CANDIDATES ARE RANKED, NOT SCANNED IN ORDER, because the first legal
    position is very often the wrong one and it took two goes to see why:

        push    metres inland past the setback. Weighted hardest — the lot is
                supposed to be ON the street, and a lot 45 m in with a drive
                hanging off it is a lot in the trees.
        skew    how far the apron leans, i.e. the offset between the gate and
                the end lane the mouth sits in. The mouth goes in an END lane
                (that is where circulation is; the middle of the street face is
                the back of a row of bays), so a lot centred on its gate has a
                28 m lean over a 23 m run — a drive crossing the verge at 50
                degrees. Sliding the LOT by a lane offset squares it up and
                costs nothing: an entrance thirty metres along the same
                frontage is still that entrance.
        slide   distance from the gate, weighted lightly, as the tie-break
                that keeps the lot next to the entrance it was chosen for.

    *keep_out* are points the slab may not swallow — the park's own gates,
    which anchor the ends of the spine and cannot be pushed out of a facility
    the way an intermediate waypoint can. Miss this and the path network is
    routed from a node inside the car park, which `check` reports as a spine
    crossing a facility and no amount of re-routing can fix.

    Returns ``(centre, yaw_deg, corners, apron_corners, mouth)`` or five Nones.
    """
    inward = sn._unit(inward)
    if sn._norm(inward) < 1e-9:
        return (None,) * 5
    # Local +x along the frontage, local +y inward — see `parking_layout`.
    u = (inward[1], -inward[0])
    yaw = math.degrees(math.atan2(u[1], u[0]))
    lanes = list(lanes) or [0.0]

    cands = []
    n_s = int(slide_m / step)
    for j in range(int(push_m / step) + 1):
        push = j * step
        for i in range(-n_s, n_s + 1):
            s = i * step
            # The gate sits at local x = -s, the lot having slid by +s.
            lx = min(lanes, key=lambda v: abs(v + s))
            cands.append((push + 0.9 * abs(lx + s) + 0.10 * abs(s),
                          push, s, lx))
    cands.sort(key=lambda t: t[0])

    for (_cost, push, s, lx) in cands:
        out = setback + push + d / 2.0
        cx = gate[0] + u[0] * s + inward[0] * out
        cy = gate[1] + u[1] * s + inward[1] * out
        cor = _obb(cx, cy, w, d, yaw)
        if not _inside(cor, rect):
            continue
        swallowed = False
        for q in keep_out:
            _near, ins, dd = _nearest_on_obb(q, cor)
            if ins or dd < keep_m:
                swallowed = True
                break
        if swallowed:
            continue
        if any(_sat_overlap(cor, q, gap) for q in placed):
            continue
        mouth = (cx + u[0] * lx - inward[0] * (d / 2.0),
                 cy + u[1] * lx - inward[1] * (d / 2.0))
        if sn._dot(sn._sub(mouth, kerb_pt), inward) < 1.0:
            continue              # kerb is already past the lot: no apron
        # AND THE DRIVE HAS TO BE A DRIVE. Nominally it is the verge plus the
        # setback, ~23 m; the cap is what stops a lot that had to slide a long
        # way sideways being served by a diagonal across the park's frontage.
        # Rejecting here rather than shortening it sends the search on to the
        # next entrance, which is the right answer.
        if sn._dist(mouth, kerb_pt) > apron_max:
            continue
        h = apron_w / 2.0
        apron = [(kerb_pt[0] - u[0] * h, kerb_pt[1] - u[1] * h),
                 (kerb_pt[0] + u[0] * h, kerb_pt[1] + u[1] * h),
                 (mouth[0] + u[0] * h, mouth[1] + u[1] * h),
                 (mouth[0] - u[0] * h, mouth[1] - u[1] * h)]
        # The apron crosses the verge and the setback, where nothing should be
        # standing — but a picnic ground slid to the boundary would be, and a
        # drive laid over one is worse than a lot sited a lane along.
        if any(_sat_overlap(apron, q, 0.5) for q in placed):
            continue
        return (cx, cy), yaw, cor, apron, mouth
    return (None,) * 5


def _entrance_frame(e, rect, kerb_m):
    """``(gate, inward unit normal, kerb point)`` for one park entrance.

    *e* is a `suburb_net` `park_entrances` record IN PARK-LOCAL COORDS — `gate`
    on the park's own boundary, `p` out on the reserve boundary where the
    street is, `dir` the inward normal. Only `gate` (or `p`) is required; the
    rest is derived off the boundary the point is nearest, so a caller with a
    bare (x, y) still gets a usable frame.
    """
    g = tuple(e.get("gate") or e.get("p") or (0.0, 0.0))
    n = e.get("dir")
    x0, y0, x1, y1 = rect
    if not n:
        side = e.get("side")
        if side is None:
            d = {"W": g[0] - x0, "E": x1 - g[0],
                 "S": g[1] - y0, "N": y1 - g[1]}
            side = min(d, key=d.get)
        n = {"W": (1.0, 0.0), "E": (-1.0, 0.0),
             "S": (0.0, 1.0), "N": (0.0, -1.0)}.get(side, (0.0, 1.0))
    n = sn._unit((float(n[0]), float(n[1])))
    # The gate is on the park boundary; clamp it there, because an entrance
    # read off a finished street graph can be a few centimetres off.
    g = (max(x0, min(x1, g[0])), max(y0, min(y1, g[1])))
    kerb = e.get("p")
    if kerb is None:
        kerb = (g[0] - n[0] * kerb_m, g[1] - n[1] * kerb_m)
    return g, n, (float(kerb[0]), float(kerb[1]))


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
                        # THE SIGN MATTERS. For a point INSIDE the box,
                        # `q - near` points from the boundary INWARD, so
                        # pushing along it drives the point deeper in and it
                        # never escapes. Outward is the other way.
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
    the inflated corners left and right of a-b and walking one side LOOKS like
    the bypass and is not: the first corner on the chosen side can sit behind
    the box as seen from *a*, so the opening leg goes straight back through the
    obstacle and the detour re-collides with it for ever. The hull contains the
    box, so walking its boundary cannot re-enter; either way round is legal and
    the shorter is the one a person would take.
    """
    ctr, u, v, hw, hh = frame
    if sn._dist(a, b) < 1e-9:
        return []
    # PAD LADDER. An endpoint outside the facility but inside the INFLATED box
    # is not on the hull, so the full-clearance detour has nowhere to start and
    # the old code gave up — leaving the segment running through the pitch. A
    # tighter detour only hugs the facility, and hugging beats crossing.
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
        return []
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
    bypass produces fans out as 5^depth and hung the generator outright, when
    the work is only quadratic in the obstacles. Splicing corner chains into a
    single chain and rescanning from the splice does the same job in a bounded
    number of passes.
    """
    clears = _clear_list(recs, clears)
    # A spliced corner has to come out of any OTHER box it lands in far enough
    # to be on that box's bypass hull, or the next detour computed from it
    # falls down `_bypass`'s pad ladder and hugs the facility. Half the
    # clearance — what this used — is inside the inflated box by construction,
    # so it guaranteed that fallback every time.
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
        # person would take and is still tried first, but with a band on every
        # facility it is frequently the corridor between a pitch and the picnic
        # ground beside it — 12 m of grass with 16 m of band wanted in it. The
        # corners land in that dead zone and `_lift` bounces them between the
        # two boundaries without satisfying either, parking the segment in the
        # gap. Testing the long way round as well costs one extra lift and
        # settles it.
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
        # A corner taken round ONE facility can land inside the NEXT, and
        # nothing else revisits it — the pre-pass only saw the original
        # waypoints. Lifting each spliced corner as it is created is what stops
        # a route pausing for two segments in the middle of a pitch.
        ins = [_lift(q, recs, lifts) for q in ins]
        budget[0] -= 1

        stall += 1
        # A bypass that cannot help — the endpoint is pinned inside the box —
        # would otherwise splice for ever at the same index.
        #
        # THE STALL ALLOWANCE IS NOT A FREE CONSTANT. Where padding puts two
        # facilities closer together than twice the band, a segment across the
        # corridor ping-pongs: round the pitch, back into the picnic ground,
        # round that, back at the pitch. It converges — each bypass is a hull
        # walk, so it strictly goes round — but not in the six tries this used
        # to allow, and giving up left the segment in the corridor.
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
    # box INFLATED by the pad, and near a corner those are not the same thing —
    # a point shoved `clear` metres off a corner along the diagonal is still
    # inside the inflated rectangle. It therefore misses the detour hull,
    # `_bypass` walks down its pad ladder, and the "detour" it returns hugs the
    # facility instead of clearing it: segments BESIDE the courts, which is the
    # defect the band exists to stop.
    #
    # Dropping rather than shoving, because a waypoint is a suggestion (a tour
    # stop, a sample off the smoothing curve) and the facility is not. Shoving
    # was tried: consecutive samples escape by different faces, the curve turns
    # into a zigzag, and the spine came back with four thousand points in it.
    # Dropped instead, the segment spans the gap in one piece and `_route_seg`
    # takes it round the outside — the way a person walking would.
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
    # the generator, but the figure has to be PER SEGMENT, not per route: the
    # coarse tour is seventeen waypoints and each leg crosses the whole park
    # past eight facilities, so a flat 400 was down to its last twenty before
    # the tour was half laid, and every leg after that came out unrouted.
    # `stall` and the chain cap bound what one segment can spend, so scaling by
    # the segment count only buys work the router was going to be allowed.
    bud = [max(budget, 64 * len(seed))]
    out = [seed[0]]
    for i in range(len(seed) - 1):
        out.extend(_route_seg(seed[i], seed[i + 1], recs, cls, bud)[1:])
    out = _dedupe(out)
    # REPAIR PASS. `clear` is a comfort margin and missing it is untidy; being
    # inside the facility at all is the actual defect, so anything still
    # penetrating gets one more detour at a clearance it cannot argue with.
    # Segments already fine cost one slab clip each, so this is nearly free.
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
    a loop leading nowhere, so it is replaced by the intersection point. Doing
    this AFTER routing rather than stopping the router creating them keeps the
    router simple, and a loop is trivial to detect once the polyline exists.
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
                # legitimately going round something. Measured, it is not: at a
                # 45 m bound seed 42 still carried 42,461 self-crossings, so
                # the pathological loops are the LARGE ones.
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

    Split by kind rather than one figure for the park, because of what stands
    on the boundary: a fenced compound has the chain-link run on the line
    itself, a gate swinging outward off it, and bike racks 4.5 m off the same
    face, so the base band would put the shared path through the racks. A
    pitch, a picnic ground or a playground has a soft edge, and there the band
    only has to read as grass.
    """
    if z.get("fenced"):
        return float(c.get("facility_pad_fenced_m", c["facility_pad_m"]))
    if z["kind"] == "parking":
        # A CAR PARK'S EDGE IS A KERB, and a footway along a kerb is where a
        # footway goes — there is no fence to graze and nothing standing in
        # the margin, so the full band is 6 m of grass nobody asked for. It
        # also costs: the lot is pinned to the frontage and cannot slide off
        # the spine the way a picnic ground can, so the wide band came back as
        # padding breaches on the tour rather than as clearance.
        return float(c.get("parking_pad_m", 3.0))
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

    Resampled to 3 m FIRST, not thinned by an integer stride: a stride thins a
    dense stretch and a sparse one by the same factor, whereas even arclength
    spacing makes the de-loop's segment tests comparable and bounds the point
    count by path LENGTH. Done as the edge enters the graph, so the stored
    geometry is the drawn geometry — a junction is then sited on the line that
    actually gets paved.
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
    was routed from a court to a point on the trunk, and nothing recorded that
    the point was a junction. The pass that afterwards clipped doubled paving
    was therefore free to delete the exact stretch of trunk a spur had joined
    onto — and did, leaving the spur touching its court and leading into the
    grass. No repair pass fixes that: the fact it needed (this point is a
    junction) was never written down.

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
        to have been split by earlier work, and hides the perfectly walkable
        alternative twenty metres further along THE SAME EDGE — the case above.

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
        two paths off it. Testing beforehand is the difference between not
        paving a strip twice and paving it twice then cutting one up afterwards
        — and it was the cutting up that severed spurs from the network.

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
            them. The refuge parking lot goes in here too, ahead of the soft
            facilities, because it is the only one sited from OUTSIDE the park:
            `cfg["entrances"]` is `suburb_net`'s `park_entrances` translated
            into park-local coords, and the lot takes the one nearest the
            courts. Left out, it falls back to a synthetic south entrance so
            the module still stands alone.
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

    # FURNITURE THAT BELONGS TO A COMPOSITION rather than to a path — the bin
    # at a picnic ground, the benches inside the playground fence, the ones on
    # a fountain ring. Collected so the path-furniture pass in (d) can seed its
    # spacing test with them; it otherwise knows only its own placements and
    # would drop a second bench on top of one.
    composed = []

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
            # A REAL GAP, not the bare `facility_gap_m`: flush pitches in the
            # same green with the same markings read as one pitch drawn twice.
            # `pitch_gap_m` opens a corridor wide enough to walk, and a path
            # goes down it below, which is what separates them on the ground.
            for i in range(1, n_sc):
                # Try BOTH sides: the first pitch may already sit near an edge,
                # and offsetting blindly to one side put the second outside the
                # park, silently dropping it.
                #
                # And CLOSE UP IF IT MUST — a wider gap also widens the pair,
                # and at some orientations the block stopped fitting the park
                # at all. Two pitches near enough to touch still read as two
                # once the corridor path is between them; one pitch reads as a
                # bug.
                done = False
                for g in (pitch_gap, pitch_gap * 0.72, pitch_gap * 0.5,
                          gap + 0.5):
                    step = pwd + g
                    for sgn in (1.0, -1.0):
                        # A STAGGER ALONG THE TOUCHLINE is allowed: squared up,
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
        # COURTS INSIDE ONE COMPOUND SHARE A SLAB. `gap` is the spacing BETWEEN
        # facilities, derived from the padding a path needs to pass (~20.6 m);
        # using it here too spread the four courts as if each were its own
        # facility. A real park paints four on one slab, a couple of metres
        # between the sidelines.
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

    # -- the refuge parking lot ---------------------------------------------
    # PLACED WITH THE COURTS, not after the soft facilities, because it is the
    # one thing here whose position is dictated from OUTSIDE the park: it has
    # to be on a street. Going in before the playground and the picnic grounds
    # means those slide off it rather than it slide off them, which is the
    # right precedence — a picnic ground can be anywhere and a refuge lot
    # cannot. It draws no rng, so adding it moves nothing downstream that its
    # own footprint does not.
    apron_box = None
    if bool(c.get("parking", True)):
        lw, ld = _rng_pair(c["parking_m"], (64.0, 40.0))
        bay = _rng_pair(c["parking_bay_m"], PARKING["bay"])
        lot_lay = parking_layout(lw, ld, bay,
                                 float(c["parking_aisle_m"]),
                                 float(c["parking_loop_m"]),
                                 float(c["parking_edge_m"]))
        if lot_lay is not None:
            # WHICH ENTRANCE. The sports end of the park is where the hard
            # surface, the floodlighting and the people arriving by car already
            # are, so the lot goes on the entrance nearest the courts — and
            # falls back to the pitches, then to the south boundary, so a park
            # configured with no courts at all still gets a lot on a street.
            crt = [z for z in zones
                   if z["kind"] in ("basketball_compound", "tennis_block")]
            if not crt:
                crt = [z for z in zones if z["kind"] == "soccer"]
            if crt:
                anchor = (sum(z["centre"][0] for z in crt) / len(crt),
                          sum(z["centre"][1] for z in crt) / len(crt))
            else:
                anchor = (0.0, -hh)
            kerb_m = float(c["parking_kerb_m"])
            ents = list(c.get("entrances") or [])
            if ents:
                # NEAREST THE COURTS FIRST, then the next nearest. One
                # entrance is not enough to try: the one closest to the courts
                # can be the one whose frontage the tennis block is already
                # standing on, and a lot forced onto it ends up inland with a
                # long diagonal drive. The neighbouring entrance is thirty
                # metres of walk further and a better car park.
                frames = sorted((_entrance_frame(e, region, kerb_m)
                                 for e in ents),
                                key=lambda f: sn._dist(f[0], anchor))
            else:
                # Standalone (no street network): synthesise the south
                # entrance `suburb_net` would have put nearest the courts.
                gx = max(-hw + lw / 2.0, min(hw - lw / 2.0, anchor[0]))
                frames = [((gx, -hh), (0.0, 1.0), (gx, -hh - kerb_m))]
            ins = float(c["parking_rect_inset_m"])
            lot_rect = (-hw + ins, -hh + ins, hw - ins, hh - ins)
            # THE LADDER. Three things can be given up and they are not worth
            # the same, so the loop nesting IS the priority order:
            #
            #   (1) CLEARANCE, innermost, spent first. The full facility gap is
            #       22.6 m; the rungs go down to `facility_pad_m`. A refuge lot
            #       6 m off a court fence still reads as a car park beside a
            #       court, which is what real parks look like.
            #   (2) THE ENTRANCE, next, so an entrance nearer the courts gets
            #       the WHOLE clearance ladder before the next gate is looked
            #       at. Measured over 13 seeds: entrance-outermost puts 11 of
            #       them on the south frontage 61-88 m from the courts;
            #       clearance-outermost sends four of those to the far side of
            #       the park because a distant gate happened to clear 22.6 m
            #       while the near one wanted 16.
            #   (3) THE DRIVE, outermost, given up last. The first pass will
            #       not take a lot more than 9 m inland or an apron more than
            #       8 m longer than the verge it crosses; without that cap seed
            #       3 took its nearest gate anyway and sat 15 m inland with a
            #       38 m lean on the drive — a car park in the trees.
            setb = float(c["parking_setback_m"])
            amax = float(c["parking_apron_max_m"])
            lc = None
            rungs = (gap, gap * 0.72, gap * 0.5, float(c["facility_pad_m"]))
            for tight in (True, False):
                for (gate, nrm, kerb_pt) in frames:
                    for g in rungs:
                        # The drive's nominal length is the verge this entrance
                        # actually has, not a constant: the reserve pad is one
                        # figure but a gate on a corner fillet is further out.
                        nom = sn._dist(gate, kerb_pt) + setb
                        lc, lyaw, lcor, apron, mouth = _place_parking(
                            lot_rect, placed, g, lw, ld, gate, nrm, kerb_pt,
                            setb, lot_lay["lanes"],
                            float(c["parking_apron_w_m"]),
                            min(nom + 8.0, amax) if tight else amax,
                            keep_out=gates,
                            push_m=9.0 if tight else 45.0)
                        if lc is not None:
                            break
                    if lc is not None:
                        break
                if lc is not None:
                    break
            if lc is not None:
                placed.append(lcor)
                z = {"kind": "parking", "centre": lc, "w": lw, "h": ld,
                     "yaw": lyaw, "corners": lcor,
                     "lines": lot_lay["lines"],
                     "n_bay_row": lot_lay["n_bay"], "n_rows": lot_lay["rows"]}
                # Local, so `_shift_park` carries them for free (`to_local`).
                z["bays_local"] = [(bx, by) for (bx, by, _f) in
                                   lot_lay["bays"]]
                z["bay_face"] = [f for (_x, _y, f) in lot_lay["bays"]]
                z["apron_local"] = [to_local(z, q) for q in apron]
                z["mouth_local"] = to_local(z, mouth)
                z["entrance_local"] = to_local(z, kerb_pt)
                zones.append(z)
                apron_box = apron

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
        # AND A BIN, at a corner of the ground: a dozen tables generate
        # rubbish. The path pass will not supply it — it hangs bins off the
        # route at 75 m and the route keeps 6 m off this ground's edge.
        wx, wy = local(z, z["w"] / 2.0 - 2.0, -z["h"] / 2.0 + 2.0)
        composed.append((wx, wy))
        prop("trash_can", wx, wy, z["yaw"])

    # -- the playground ------------------------------------------------------
    # FOUR PIECES IN A ROW IS NOT A PLAYGROUND. The old pass put a swing, a
    # structure and two seesaws at even x across the sand: on 1,564 m2 that is
    # one piece per 390 m2, and the render read as a beach with some equipment
    # left on it. The real figure is nearer one per 150-200 m2, laid as a
    # SPREAD rather than a line because the kit has to be walkable between.
    #
    # SEPARATION IS A SAFETY FIGURE, not taste. CPSC's Public Playground Safety
    # Handbook puts a minimum 6 ft (1.83 m) between the use zones of adjacent
    # equipment, and swings want more fore and aft; the radii below are the USE
    # ZONE, not the frame, which is why the swing carries 3.4 m for a 4.5 m
    # asset.
    #
    # THE MIX IS DRIVEN BY THE FACE COUNTS, see `park.yaml`. The swing set and
    # the Kids Place are 75.9k and 79.5k faces and `play_structure` is NOT one
    # of the categories `suburb_scene` instances, so each extra is paid in full
    # and they stay capped (`play_expensive_cap`, 4). The seesaw is 498 faces —
    # 152x cheaper — and fills the rest: nine pieces cost 313k against the old
    # four's 156k.
    play_r = {"swing_set": 3.4, "play_structure": 4.2, "seesaw": 1.8}
    for z in zones:
        if z["kind"] != "playground":
            continue
        hwz, hhz = z["w"] / 2.0, z["h"] / 2.0
        n_kit = max(4, int(round(z["w"] * z["h"]
                                 / float(c["play_area_per_piece_m2"]))))
        big = ["play_structure", "swing_set", "swing_set", "play_structure"]
        kit_list = big[:max(1, min(int(c["play_expensive_cap"]), n_kit // 2))]
        kit_list += ["seesaw"] * max(0, n_kit - len(kit_list))
        # (lx, ly, use-zone radius) of everything already standing on the sand.
        kit = []

        # `edge` is a band round the inside of the fence the kit stays out of.
        # Not slack: a CPSC use zone may not overlap the enclosure, and the
        # same band is where the benches go — measured, at 1.5 m only three of
        # the six benches the perimeter asks for survived the collision test.
        def sand_spot(r, tries=500, edge=3.0, keep_middle=0.0):
            """A free (lx, ly) with room for a *r* metre use zone."""
            ax, ay = hwz - r - edge, hhz - r - edge
            if ax <= 0.0 or ay <= 0.0:
                return None
            for _ in range(tries):
                lx, ly = rng.uniform(-ax, ax), rng.uniform(-ay, ay)
                if (abs(lx) < hwz * keep_middle
                        and abs(ly) < hhz * keep_middle):
                    continue
                if any(math.hypot(lx - qx, ly - qy) < r + qr + 1.83
                       for (qx, qy, qr) in kit):
                    continue
                kit.append((lx, ly, r))
                return (lx, ly)
            return None

        for kind in kit_list:
            spot = sand_spot(play_r[kind])
            if spot is None:
                continue
            wx, wy = local(z, *spot)
            # Yawed off the playground's own axis, not freely: a swing's fall
            # zone is fore and aft, so kit square to the space is what leaves
            # the walking room. uniform(0, 360) put swings across the gaps.
            prop(kind, wx, wy, z["yaw"] + rng.uniform(-25.0, 25.0))
        # A SHADE SHELTER AND SOMEWHERE TO PUT A BAG — the sun cover every
        # municipal playground has, and the tables the party sits at. Held out
        # of the middle third so the sand people actually play on stays clear.
        for kind, r in (("gazebo", 3.4), ("picnic_table", 1.6),
                        ("picnic_table", 1.6)):
            spot = sand_spot(r, tries=300, edge=1.0, keep_middle=0.45)
            if spot is None:
                continue
            wx, wy = local(z, *spot)
            prop(kind, wx, wy, z["yaw"] + rng.uniform(-25.0, 25.0))
        # BENCHES ROUND THE INSIDE EDGE, FACING IN: an adult sits on the rail
        # and watches the whole box, so the seat faces the equipment. Same
        # convention as the path benches below — the yaw IS the direction the
        # bench looks — so this is `atan2` back at the middle of the sand.
        seat = 2.2
        cor = [(-hwz + seat, -hhz + seat), (hwz - seat, -hhz + seat),
               (hwz - seat, hhz - seat), (-hwz + seat, hhz - seat)]
        segs = [(cor[i], cor[(i + 1) % 4]) for i in range(4)]
        per = sum(sn._dist(a, b) for (a, b) in segs)
        n_b = max(3, int(round(per / float(c["play_bench_pitch_m"]))))
        for i in range(n_b):
            s = per * (i + 0.35) / n_b
            lx = ly = 0.0
            for (a, b) in segs:
                L = sn._dist(a, b)
                if s > L:
                    s -= L
                    continue
                t = s / L
                lx, ly = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
                break
            if any(math.hypot(lx - qx, ly - qy) < qr + 1.9
                   for (qx, qy, qr) in kit):
                continue
            kit.append((lx, ly, 1.1))
            wx, wy = local(z, lx, ly)
            composed.append((wx, wy))
            prop("bench", wx, wy,
                 z["yaw"] + math.degrees(math.atan2(-ly, -lx)))
        for sgn in (1.0, -1.0):
            wx, wy = local(z, sgn * (hwz - 2.4), -sgn * (hhz - 2.4))
            composed.append((wx, wy))
            prop("trash_can", wx, wy, z["yaw"])

    # -- set pieces: fountains, gazebos, the sign ---------------------------
    # DESTINATIONS, not path furniture. A fountain is a thing you walk TO, so
    # it claims its ground with the facilities and the network is routed to
    # reach it. Placing it after the paths meant siting it wherever was left
    # and then dragging a spur out, which is why those spurs dangled off the
    # spine.
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
        # Toward the park interior, the side every path reaches these compounds
        # from. From the facility alone: the path network does not exist yet.
        n = sn._unit(sn._sub((0.0, 0.0), z["centre"]))
        if sn._norm(n) < 1e-6:
            n = (0.0, 1.0)
        # Boundary point on the interior-facing side, in the facility's own
        # frame -- `entrance()` is defined below, with the path network.
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
    # The spine wanders, NOT a rectangle round the edge. A perimeter loop was
    # the same mistake as ringing the suburb with an arterial: a racetrack with
    # every facility hanging off it as a stub. A park is entered at a couple of
    # points and threaded by a route between the things worth reaching.
    #
    # Built from `zones` rather than from `placed` so the pads below stay
    # index-aligned with their facilities; the corner lists are the same
    # objects either way, which is what the identity-based `exempt` relies on.
    # THE APRON IS AN OBSTACLE, not a zone. It is a driveway across the verge,
    # so nothing may be planted on it and no path may run down it — but it is
    # not a facility either: it owns no padding band of its own and nothing
    # should route a spur to it. `features` is exactly that category (the
    # fountain and gazebo keep-outs), so it goes there, which also keeps
    # `obstacles` index-aligned with `zones` for the pads below.
    if apron_box is not None:
        features.append(apron_box)
    obstacles = [z["corners"] for z in zones] + features
    # THE PADDING BAND, one figure per obstacle. Recorded on the zone too, so
    # the checks below measure what the plan promised rather than re-deriving
    # it from a config they were not given. Set pieces keep `path_clear_m`: a
    # fountain is a destination to arrive at, not an edge to respect.
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

    # NO SEPARATE BIKE NETWORK. A second circuit weaving past the first was the
    # tangle it sounds like wherever the two met: independent routes crossing
    # at shallow angles, each dodging the other's obstacles. A park path is ONE
    # wider way carrying both modes, divided down its length -- which is how
    # shared-use paths are actually built. `width_m` and `bike_share` on each
    # edge give the divider, so there is one network and one set of junctions.
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
    # x+y reads as an ordering but is not a route: it doubled back across the
    # park and knotted the spine round the picnic grounds. Greedy is not
    # optimal and does not need to be — it just has to not cross itself.
    #
    # NOT 2-OPTED. Improving the tour was tried, on straight-line cost and on a
    # cost that charges for the facilities a leg must walk round. Both are
    # better tours and neither is a better park: the router pays for a shorter
    # leg with a longer detour, and over seeds 1/3/7/42 the doubled paving went
    # from 205 m to 355 m (straight-line: worse still). Greedy leaves one long
    # return leg that can run beside its outbound one — the residual
    # `check_doubled` reports, and the cheaper bill.
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
        between — the router wraps the connection round the compound and back,
        paving a 54 m gap as a 208 m path that runs alongside the spine for a
        third of its length. So a candidate must have a clear straight run to
        be offered at all, nearest first. The search is capped: a "clear"
        junction on the far side of the park is not one anybody would walk to,
        and if nothing inside the cap is clear the nearest is used.

        More than one is returned so the caller can route to each and take one
        that comes out legal. They are spread out, since two candidates ten
        metres apart on the same edge are the same junction and fail alike.
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

        THE SINGLE DOOR every path goes through, which is why none can end up
        joining nothing. The target is a point ON AN EXISTING EDGE and that
        edge is CUT there, so the connection is a junction, not two paths that
        happen to touch. A node already within a couple of metres of the
        network is welded onto the junction rather than bridged by a stub of
        path shorter than it is wide.

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
        # and `check_pad` is left to say so out loud.
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
    # afterwards. Laying it free-standing and then tying each end to the
    # nearest path doubled the spine: the nearest path to a dead end pointing
    # out of the corridor is the spine at the corridor's FAR end, so the
    # connector ran 60 m back down the corridor it had just been laid beside.
    # Siting both ends on the network first cannot do that.
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
            # them, so it quite often runs down this corridor by itself, and a
            # second way down it would only be a seam. Measured on the ROUTED
            # line, since that is what gets paved; the ends are trimmed because
            # they are ON the network by construction. `suburb_net` rejects a
            # candidate street on clearance in the same way and for the same
            # reason.
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
        whatever diagonal the centreline makes, past a corner that is not a
        gate.
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
    # THE ONE PATH ALLOWED THROUGH THE BAND. Padding every facility and obeying
    # it everywhere would leave nothing able to reach an entrance, so there is
    # exactly one exception: the stub serving a facility crosses its own band
    # and only its own, carried by `exempt`. Everything downstream of the
    # throat is bound like any other path.
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
        # is right nine times in ten, but two facilities stand as little as 9 m
        # apart and the band is wider than that, so a stub out of the facing
        # side walks into the neighbour's padding — which no exemption covers,
        # since it serves the other one. Fall back to whichever remaining face
        # is clear, nearest the trunk first.
        #
        # Choosing the face by whether the LEG ON FROM IT comes out legal, not
        # merely whether the stub has room, was tried too: over seeds 1-12 it
        # changed not one park and cost a third of the run time.
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
        # ALWAYS EMITTED, even when the trunk already runs past the entrance:
        # skipping it there was how facilities ended up with no path touching
        # them at all, since the trunk keeps the full band off the boundary and
        # "close enough" is still a fence away from the gate.
        #
        # The entrance gets a node of its OWN, not a weld to whatever lies
        # within tolerance: `check_reach` measures this end against the
        # boundary at half a metre, and a reused node would drag it off the
        # facility.
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

    # -- fountain plazas: a ring round the water, benches on it facing in ----
    # PORTED FROM THE URBAN PARK, not invented a second time. `parks.py`'s
    # module docstring ("THE ATTRACTION AS A COMPOSED CENTREPIECE") states the
    # pattern: an attraction is a CIRCLE, not a point; a concentric promenade
    # sits outside its keep-out; walks land ON the ring, so it carries you
    # round the water rather than at it; benches stand just outside it facing
    # inward; and the planting is held out of a wider clearing, so what the
    # walks lead to is visible instead of buried in canopy.
    #
    # WHAT PORTS AND WHAT DOES NOT.
    #
    #   `_ring`, `_clip_outside`  parks.py draws free polylines and cleans up
    #     afterwards, so it has to CUT its walks on the ring. Here the paths
    #     are a graph and nothing may be cut after the fact (see `PathNet`),
    #     and the clipping is unnecessary anyway: the fountain's keep-out box
    #     is already an obstacle carrying `path_clear_m`, so no other path
    #     comes within 12.3 m of the water BY CONSTRUCTION. The ring is built
    #     inside that dead zone, where there is nothing to clip against.
    #   ring benches            port straight over, with one difference: in
    #     parks.py the seat normal is the placement yaw + 90, reusing the
    #     along-path convention, whereas here a bench's yaw IS the direction it
    #     looks (see the path furniture in (d)), so inward is `th + 180`.
    #   the clearing            comes free. Trees are rejection-sampled against
    #     the finished path grid at 5 m and the ring is a path, so the annulus
    #     either side of it is already unplantable.
    #
    # A RING IS A CYCLE AND THE INVARIANTS ARE THE HARD PART, so it is built as
    # TWO ARCS between two nodes rather than as one closed edge. A closed edge
    # has both ends at the same node, which `add_edge` refuses outright and
    # `check_orphans` could not read as a junction anyway; two arcs give every
    # end a second path ending on it, so the ring is one component with no
    # loose end and reaches the rest of the network through the ordinary `join`
    # -> `split` -> `add_edge` sequence every other path uses.
    #
    # AND IT IS ONLY LAID IF IT COMES OUT LEGAL, down a radius ladder, exactly
    # as the pitch corridor is only laid if the gap is not paved already.
    # `open_spot` sites a fountain 17 m clear of the nearest facility box and a
    # fenced compound wants 8 m plus half the path width off its edge, so an
    # 8 m ring can finish 0.3 m short of the band; shrink it before giving up,
    # and if no radius fits, the fountain keeps the plain spur it always had.
    # A ring that breaches the padding is worse than no ring.
    ring_r0 = float(c["fountain_ring_r_m"])
    ring_pitch = float(c["fountain_bench_pitch_m"])
    foff_m = float(c["furniture_offset_m"])

    def arc_pts(q, r, a0, a1):
        n = max(6, int(abs(a1 - a0) * r / 1.5))
        return [(q[0] + r * math.cos(a0 + (a1 - a0) * k / n),
                 q[1] + r * math.sin(a0 + (a1 - a0) * k / n))
                for k in range(n + 1)]

    def plaza(q, box):
        """Ring path and inward benches round *q*. True if one was laid."""
        for r in (ring_r0, ring_r0 - 1.0, ring_r0 - 2.0):
            if r < 4.5:
                break
            if not legal(arc_pts(q, r, 0.0, 2.0 * math.pi), w_spur,
                         exempt=(box,)):
                continue
            near = net.nearest(q)
            if near is None:
                return False
            # The walk arrives from wherever the network already is, so that is
            # where the ring is entered and where it is cut into two arcs.
            th = math.atan2(near[3][1] - q[1], near[3][0] - q[0])
            gate = (q[0] + r * math.cos(th), q[1] + r * math.sin(th))
            # Sited and routed BEFORE anything is committed: an approach that
            # cannot be walked legally means no plaza, and rolling a half-built
            # ring back out of the graph is exactly the kind of after-the-fact
            # surgery `PathNet` exists to make unnecessary.
            hit, pts, ok = join(gate, w_spur, exempt=(box,))
            if hit is None or not ok:
                continue
            na = net.add_node(gate)
            # `serves` marks a path no junction may be cut into. On a serving
            # stub that is because the stub owns a padding exemption; here it
            # is because the ring sits inside the fountain's keep-out, so a
            # path joining it would have to cross the water. -1 is not a zone
            # index, so it grants no exemption and `check_pad` still holds the
            # ring to every facility band.
            e1 = net.add_edge(arc_pts(q, r, th, th + math.pi), "ring", w_spur,
                              a=na, bike_share=0.0, serves=-1)
            if e1 is None:
                return False
            e2 = net.add_edge(arc_pts(q, r, th, th - math.pi), "ring", w_spur,
                              a=na, b=e1["b"], bike_share=0.0, serves=-1)
            if e2 is None:
                return False
            mid = net.split(hit[1], hit[2])
            if sn._dist(gate, net.nodes[mid]) <= _WELD_M:
                net.weld(na, mid)
            else:
                net.add_edge(pts, "spur", w_spur, a=na, b=mid, bike_share=0.0)
            # Benches OUTSIDE the ring, at the same set-back a bench keeps from
            # any other path here, facing the water. Two bins at the quarters,
            # as in parks.py — a plaza people sit at is a plaza people eat at.
            rb = r + foff_m
            nb = max(4, int(2.0 * math.pi * rb / max(2.0, ring_pitch)))
            phase = rng.uniform(0.0, 360.0)
            for i in range(nb):
                a = math.radians(phase + i * 360.0 / nb)
                p = (q[0] + rb * math.cos(a), q[1] + rb * math.sin(a))
                composed.append(p)
                prop("bench", p[0], p[1], math.degrees(a) + 180.0)
            for i in range(2):
                a = math.radians(phase + 45.0 + i * 180.0)
                p = (q[0] + rb * math.cos(a), q[1] + rb * math.sin(a))
                composed.append(p)
                prop("trash_can", p[0], p[1], math.degrees(a) + 180.0)
            return True
        return False

    # Set pieces are DESTINATIONS: a fountain with no path to it is as wrong as
    # a court with none. The spur is exempt from the fountain's own clearance
    # box for the same reason a facility stub is exempt from its facility's.
    for (_k, q, box) in feature_pts:
        near = net.nearest(q)
        if near is None or near[0] <= 5.0:
            continue          # the network already passes close enough to it
        if _k == "fountain" and plaza(q, box):
            continue      # the ring is its own arrival; no spur to water
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

    # NO DE-DUPLICATION PASS. There used to be one here, and deleting it is the
    # point of the graph. Paths were routed against the FACILITIES but not
    # against each other, so the spine and the pitch corridor could run over
    # the same ground; the pass clipped a later path wherever it doubled an
    # earlier one. It knew nothing about connectivity, so what it deleted was
    # sometimes precisely the stretch a spur had joined onto — which is how a
    # court ended up with a short path coming out of it and going nowhere.
    #
    # A path that MEETS another now joins it at a node instead of being drawn
    # alongside it and cleaned up afterwards; `check_doubled` measures what is
    # left of the doubling.
    paths = net.paths()

    # -- fences -------------------------------------------------------------
    panel = float(c["fence_panel_m"])
    fences = []

    def gate_at(z):
        """Where a path arrives on *z*'s boundary, i.e. where the gate goes."""
        for pa in paths:
            for p in (pa["pts"][0], pa["pts"][-1]):
                near, _in, d = _nearest_on_obb(p, z["corners"])
                if d <= 1.0:
                    return near
        return None

    for z in zones:
        # THE PLAYGROUND IS FENCED TOO, and not for decoration: a play area is
        # enclosed so a two-year-old cannot walk out of it onto a pitch. Done
        # HERE rather than by putting `fenced` on the zone, because that flag
        # also selects `facility_pad_fenced_m` (8 m instead of 6) and would
        # move every path near the playground — the fence is a fact about the
        # boundary, not a claim on the room the paths must leave. The compounds
        # keep the flag: their band really is wider, the bike racks stand in
        # it.
        if not (z.get("fenced") or z["kind"] == "playground"):
            continue
        cor = z["corners"]
        # AND A GAP WHERE THE SERVING PATH ARRIVES. Without it the stub that
        # `check_reach` measures ends against a panel — a fence with a path
        # walking into it. Only the playground gets one; the compounds' runs
        # predate this.
        gate = gate_at(z) if z["kind"] == "playground" else None
        for i in range(4):
            ax, ay = cor[i]
            bx, by = cor[(i + 1) % 4]
            L = math.hypot(bx - ax, by - ay)
            n = max(1, int(round(L / panel)))
            yaw = math.degrees(math.atan2(by - ay, bx - ax))
            for k in range(n):
                f = (k + 0.5) / n
                p = (ax + (bx - ax) * f, ay + (by - ay) * f)
                if gate is not None and sn._dist(p, gate) < panel:
                    continue
                fences.append({"c": p, "yaw": yaw})

    # =======================================================================
    # (c) TREES — into what is left, clear of facilities AND of the paths
    # =======================================================================
    # NOT a belt on the boundary rectangle. Trees are massed in COPSES whose
    # density falls off from a centre, so the wood has a soft edge and reads as
    # continuing past the park rather than stopping at a line.
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

    # STEMS DO NOT STACK. There was no tree-to-tree test at all, survivable at
    # 11/ha and not at park density: a Poisson scatter at 55/ha puts pairs a
    # metre apart, and two crowns a metre apart are one blob with two trunks
    # under it. Same grid trick as the path test above, so the question stays
    # O(1) per candidate instead of O(trees) — at 700 trees the naive version
    # is a quarter of a million distance tests per pass.
    tsep = float(c["tree_spacing_m"])
    tgrid = {}

    def plant(q, spacing):
        gx, gy = int(math.floor(q[0] / cell)), int(math.floor(q[1] / cell))
        rr = int(math.ceil(spacing / cell))
        for i in range(gx - rr, gx + rr + 1):
            for j in range(gy - rr, gy + rr + 1):
                for p in tgrid.get((i, j), ()):
                    if sn._dist(q, p) < spacing:
                        return False
        tgrid.setdefault((gx, gy), []).append(q)
        prop("tree", q[0], q[1], rng.uniform(0, 360))
        return True

    # WHERE THE PLANTING GOES, the half of "more trees" that decides whether
    # the result is a park or an orchard. A park is planted at its MARGINS —
    # the belt inside the boundary, the strips between one facility and the
    # next — and left open in the middle, because the open lawn is what the
    # planting frames. A flat scatter at margin density fills the greensward
    # and the pitch surrounds evenly, which is a plantation.
    #
    # So a candidate is accepted with a probability falling from 1 at a margin
    # to `tree_open_frac` out in the open. The COUNT is unaffected — the loop
    # still runs until it has placed `want` — so the weighting moves trees
    # about rather than quietly changing the density knob's meaning.
    #
    # `d` is measured to the facility BOX, not to its padding band, on purpose:
    # what makes a strip between two facilities read as planted is trees right
    # up against the fence line, and `free()` already holds them the 4 m off it
    # that stops a crown sitting on the court.
    marg = float(c["tree_margin_m"])
    openf = float(c["tree_open_frac"])

    def plant_w(q):
        d = min(q[0] + hw, hw - q[0], q[1] + hh, hh - q[1])
        for ob in obstacles:
            if d <= 0.0:
                break
            d = min(d, _nearest_on_obb(q, ob)[2])
        if d >= marg:
            return openf
        return openf + (1.0 - openf) * (1.0 - d / marg) ** 1.5

    n_copse = int(c["copses"])
    cdens = float(c["copse_per_1000m2"])
    inland = float(c["copse_inland_share"])
    for _ in range(n_copse):
        rad = rng.uniform(*_rng_pair(c["copse_r_m"], (26.0, 55.0)))
        if rng.random() < inland:
            # INLAND STANDS, smaller than the edge woods. The edge copses are a
            # screen belt and right to be big; the middle of the park wants the
            # other thing, a clump you walk round between the courts and the
            # picnic ground. Sited by the same `free` test, with more room
            # asked for, so a stand does not start life half on a pitch.
            cxy = None
            for _try in range(80):
                p = (rng.uniform(ix0, ix1), rng.uniform(iy0, iy1))
                if free(p, 16.0, path_d=10.0):
                    cxy = p
                    break
            if cxy is None:
                continue
            rad *= 0.55
        else:
            # Edge copses hug the boundary, where a park thickens into
            # whatever surrounds it.
            edge = rng.randrange(4)
            if edge == 0:
                cxy = (rng.uniform(-hw, hw), rng.uniform(-hh, -hh + 60))
            elif edge == 1:
                cxy = (rng.uniform(-hw, hw), rng.uniform(hh - 60, hh))
            elif edge == 2:
                cxy = (rng.uniform(-hw, -hw + 60), rng.uniform(-hh, hh))
            else:
                cxy = (rng.uniform(hw - 60, hw), rng.uniform(-hh, hh))
        for _ in range(int(math.pi * rad * rad / 1000.0 * cdens)):
            # r^0.6 biases inward, so density thins toward the edge instead of
            # ending on a hard circle.
            a = rng.uniform(0, 2 * math.pi)
            d = rad * (rng.random() ** 0.6)
            q = (cxy[0] + d * math.cos(a), cxy[1] + d * math.sin(a))
            if not (-hw <= q[0] <= hw and -hh <= q[1] <= hh):
                continue
            if free(q, 6.0):
                # Tighter inside a wood than out on the lawn: a copse is meant
                # to close over, so crowns touching is the point of it.
                plant(q, tsep * 0.78)

    # The scatter over the remaining green — margins dense, open lawn thin.
    want = int((ix1 - ix0) * (iy1 - iy0) / 1000.0
               * float(c["lawn_tree_per_1000m2"]))
    # 80 tries per tree was enough for a scatter that accepted nearly
    # everything. This one refuses on three counts (facility, path, weight) and
    # the last of them refuses 70% of the open lawn by design, so the budget
    # has to cover the refusals or the density knob silently tops out.
    tries = 0
    while want > 0 and tries < want * 220:
        tries += 1
        q = (rng.uniform(ix0, ix1), rng.uniform(iy0, iy1))
        if not free(q, 6.0):
            continue
        if rng.random() > plant_w(q):
            continue
        if plant(q, tsep):
            want -= 1

    # =======================================================================
    # (d) PATH FURNITURE — benches and bins, which BELONG TO THE PATH
    # =======================================================================
    # Last, because they are the one thing here whose position is defined by
    # the route rather than by the ground: placed before the paths were final
    # they sat in the grass beside nothing, facing nowhere. A bench faces the
    # path it is set back from; the bins go on the same line, a good deal
    # sparser.
    bspace = float(c["bench_spacing_m"])
    tspace = float(c["trash_spacing_m"])
    foff = float(c["furniture_offset_m"])
    # SEEDED WITH THE COMPOSED FURNITURE. The playground's benches and the ones
    # on a fountain ring are already on the ground by the time this runs, and
    # the spacing test only knew about what this loop itself had placed — so a
    # spur passing a plaza dropped a second bench two metres from one already
    # facing the water.
    furn = list(composed)

    def furn_ok(p, d=7.0):
        """Room for furniture at *p*, keeping *d* off its neighbours.

        THE RADIUS IS PER KIND, and a flat 7 m was actively wrong for the bin.
        Two benches 3 m apart are one bench drawn twice; a BIN 3 m from a bench
        is where bins actually are, because somebody is sitting down. With one
        figure for both, whichever pass ran first took the ground: benches
        first left 13 bins on seed 1, bins first left 35 benches.
        """
        box = _obb(p[0], p[1], 2.4, 2.4, 0.0)
        if any(_sat_overlap(box, r, 0.5) for r in obstacles):
            return False
        return not any(sn._dist(p, f) < d for f in furn)

    for pa in paths:
        # "ring" is excluded on purpose, for `parks.py`'s reason: a ring's
        # furniture is COMPOSED — an even pitch of benches facing the water —
        # so the walk-and-drop pass would lay a second row of seats round the
        # same circle, alternating sides and facing outward.
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
            # 2.5 m, not 7: a bin belongs BESIDE a bench (see `furn_ok`), and
            # it only has to miss the seat it is standing next to.
            if not furn_ok(p, 2.5):
                continue
            furn.append(p)
            prop("trash_can", p[0], p[1],
                 math.degrees(math.atan2(t[1], t[0])))

    # =======================================================================
    # (e) ONE BENCH AND ONE BIN FOR THE WHOLE PARK
    # =======================================================================
    # A park is furnished by ONE PROCUREMENT ORDER: the borough buys a hundred
    # of the same bin and bolts them down, so every bin matches every other bin
    # and every bench every other bench. Drawing per PROP out of a pool of
    # eight bins gives a park in which no two bins are alike, which reads as a
    # junk yard rather than as furniture — the same defect reported on the
    # street furniture (bins, hydrants, streetlights), same cause and same fix.
    #
    # THE POOL IS NOT NARROWED, only the draw. The variety in `park.yaml` and
    # `shared.yaml` is what makes a different seed a differently furnished
    # park, which is the point of holding eight bins; taking the choice once
    # per PARK instead of once per PROP keeps that and loses only the jumble.
    # Picnic tables are in the matched set for the same reason — a picnic
    # ground is a batch of identical tables — while trees, play kit and the set
    # pieces are deliberately not: a wood is not procured.
    #
    # DRAWN LAST, and not by accident of where the code sits: every rng draw
    # shifts every draw after it, so making this choice at the top of `plan`
    # would mean adding a bin to a picnic ground moves a soccer pitch. Nothing
    # reads the rng after this point, so here it is free.
    #
    # An opaque number rather than an index, so it stays valid whatever length
    # the pool turns out to be; the consumer takes it modulo the pool size (see
    # `suburb_scene._park_placements`).
    matched = {k: rng.randrange(1000003)
               for k in ("bench", "trash_can", "picnic_table")}
    for p in props:
        if p["kind"] in matched:
            p["variant"] = matched[p["kind"]]

    return {"region": region, "zones": zones, "paths": paths,
            "fences": fences, "props": props}


def parking_zone(park):
    """The refuge lot's zone, or None if the park has no parking."""
    for z in park["zones"]:
        if z["kind"] == "parking":
            return z
    return None


def parking_info(park):
    """The refuge lot in WORLD coords — what the survivors/cars pass reads.

    THE ONE PLACE THE LOCAL STORAGE IS UNDONE. Everything about the lot is kept
    in the facility's own frame so `suburb_scene._shift_park` cannot leave the
    bays behind when it moves the slab (see `to_local`); this resolves it once,
    against whatever `centre` the park ended up at, so no consumer has to know
    the convention. Call it AFTER the shift.

    Schema::

        {"corners": [(x, y) x4],      # the slab, CCW, world
         "centre":  (x, y),
         "yaw_deg": float,            # +x runs along the street frontage
         "w": float, "d": float,      # frontage x depth, metres
         "bays":    [{"centre": (x, y), "yaw_deg": float}, ...],
         "apron":   [(x, y) x4],      # kerb -> lot, CCW, world
         "entrance": (x, y),          # where the apron meets the street
         "mouth":   (x, y),           # where it meets the lot
         "rows": int, "bays_per_row": int}

    A bay's ``yaw_deg`` is the heading a car nosed into it points — away from
    the aisle it reversed out of, i.e. the lot's yaw +-90 depending on which
    side of the aisle the bay is on.
    """
    z = parking_zone(park)
    if z is None:
        return None
    faces = z.get("bay_face") or [1.0] * len(z.get("bays_local", ()))
    return {
        "corners": [tuple(q) for q in z["corners"]],
        "centre": tuple(z["centre"]),
        "yaw_deg": float(z["yaw"]),
        "w": float(z["w"]), "d": float(z["h"]),
        "bays": [{"centre": to_world(z, p),
                  "yaw_deg": float(z["yaw"]) + (90.0 if f > 0 else -90.0)}
                 for (p, f) in zip(z["bays_local"], faces)],
        "apron": [to_world(z, p) for p in z.get("apron_local", ())],
        "entrance": to_world(z, z["entrance_local"]),
        "mouth": to_world(z, z["mouth_local"]),
        "rows": int(z.get("n_rows", 0)),
        "bays_per_row": int(z.get("n_bay_row", 0)),
    }


def check_parking(park, gap=0.0):
    """The refuge lot's own self-test.

    ``(bays, overlaps, off_slab, apron_len_m)`` — the bay count, how many other
    facilities the slab runs into, how many bays fall outside their own slab,
    and how long the apron is. The first three must be (>0, 0, 0) and the last
    must be positive, or the lot is not a lot anybody can drive onto.

    Facility non-overlap is not covered by `check`/`check_pad`/`check_reach`:
    those measure PATHS against facilities, and `_place` is what keeps the
    facilities off each other. The lot is the first thing here sited by an
    outside constraint rather than by `_place`, so it is the first that can get
    that wrong, and it says so out loud.
    """
    z = parking_zone(park)
    if z is None:
        return 0, 0, 0, 0.0
    over = sum(1 for q in park["zones"]
               if q is not z and _sat_overlap(z["corners"], q["corners"], gap))
    off = 0
    hw, hh = z["w"] / 2.0, z["h"] / 2.0
    for p in z.get("bays_local", ()):
        if abs(p[0]) > hw + 1e-6 or abs(p[1]) > hh + 1e-6:
            off += 1
    a = z.get("apron_local")
    alen = sn._dist(to_world(z, a[0]), to_world(z, a[3])) if a else 0.0
    return len(z.get("bays_local", ())), over, off, alen


def check(park, eps=0.25):
    """How many (path segment, facility) pairs actually cross. Must be zero.

    Counts PENETRATION, not contact: a spur is supposed to finish exactly on
    the boundary it serves, so a segment that merely touches the box is correct
    and one carrying *eps* metres or more of its length through the interior is
    the bug. Hence the clipped span rather than an endpoint-inside test.
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

    Anything else is a stub dying in the grass, the defect this file was
    rebuilt around. The junction test is GEOMETRIC — two ends at the same
    point — not a lookup in an exported node table, so it still measures
    something when the generator is wrong.
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

    The measure the deleted de-duplication pass used to exist for (see the note
    in `plan`). Junction neighbourhoods are exempt: two paths that meet at a
    node necessarily overlap where they meet, and that is a junction, not a
    seam.
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
    lot = parking_zone(park)
    return {"zones": dict(z), "props": dict(p),
            "fence_panels": len(park["fences"]),
            "parking_bays": len(lot.get("bays_local", ())) if lot else 0,
            "paths": len(park["paths"])}
