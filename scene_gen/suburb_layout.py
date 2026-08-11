"""
suburb_layout.py — hierarchical street layout for a post-war suburb.

Additive, and wired the same way as `city_layout.py`: `make_subdivider` returns a
drop-in replacement for `scene_generator._subdivide_region_metric` with the same
signature and the same ``(blocks, road_corridors)`` return shape, and
:class:`patched` swaps it in around one `build_city` call and restores it after.
Nothing tracked is edited.

WHY A SECOND SUBDIVIDER AND NOT A KNOB ON THE FIRST
---------------------------------------------------
`city_layout` and the built-in are both BSP: a cell is cut by a road that runs
edge to edge, and the two halves recurse. Every road therefore spans its cell,
so **a BSP cannot express a dead end at all** — and a dead end is the single
defining feature of the fabric being asked for. Cul-de-sacs are not a decoration
that can be added afterwards either: they change what the block structure *is*,
because the land behind a bulb is one deep parcel rather than a street-bounded
block.

So this module abandons recursive splitting. It COMPOSES corridors top-down as
an explicit hierarchy, and then recovers the blocks as the complement of the
streets (:func:`_extract_blocks`). Once corridors are placed rather than derived
from splits, a street that stops in the middle of a cell costs nothing.

WHAT THE GUIDANCE SAYS  (numbers that became defaults, sources in the preset)
----------------------------------------------------------------------------
* Functional hierarchy — arterial / collector / local — is FHWA's. Its
  *Highway Functional Classification* Table III-1 puts SUBURBAN arterial spacing
  at 1–2 miles, the classic section-line grid, which is why the region border
  ring is the arterial network and nothing inside it needs to be one. FHWA gives
  no numeric COLLECTOR spacing at all, only a 3/4-mile trip-length split between
  major and minor, so collector spacing here is measured off OSM rather than
  taken from a standard.
* Local street width is the biggest single difference from downtown, and every
  source makes the driver the PARKING configuration rather than the class. VDOT
  Table B(1)-1: 24 ft with no parking, 29 ft with parking both sides, at 25 mph.
  Iowa SUDAS quantises two-lane roadways to 26/31/34/37 ft. Napa's local
  standard is 36 ft. So a local here is 35 ft against a downtown avenue's 46 ft,
  and its kerb parking is a third of its width.
* Cul-de-sac turnarounds are a fire-apparatus dimension, not a styling choice.
  IFC Appendix D, D103.4: a dead end over 150 ft needs a turnaround, and Table
  D103.4 requires, for 151–500 ft, a 20 ft width plus one of a 120 ft hammerhead,
  a 60 ft "Y", or a 96 ft DIAMETER cul-de-sac — a 48 ft (14.6 m) radius to the
  driving surface, not to the ROW. SUDAS 5C-2(P) adopts 45–48 ft. Length is
  capped at 500–600 ft by many ordinances, 750 ft by IFC, and 1,000 ft or 200
  trips/day (about 20 lots) by ITE/ULI/ASCE.
* Measured suburbs (OSM, five US subdivisions, 2.56 km² each): 12–35% of nodes
  are dead ends against 6–8% in pre-war grids; 69–87% of junctions are three-way
  against 38% in a pre-war grid; street density 7.4–12.4 km/km² against 17.6–23.4
  downtown; blocks 15k–34k m². Barrington-Leigh & Millard-Ball (PNAS 2015,
  SI Table S2) independently give 26.6% dead ends for US intersections built
  1993–97 and 19.1% for 2008–12, metro extremes 19–42%. Those are the targets.

THE LIMIT, STATED PLAINLY: NO CURVES
------------------------------------
Every downstream consumer — block packing, `city_detail`'s frontage walk,
`road_markings`, `apply_ground_planes` — takes blocks and corridors as
axis-aligned rects. A curved or diagonal street is not reachable without editing
tracked code. Real suburban streets curve; these do not.

The realism therefore has to come from TOPOLOGY, and it is worth being precise
about what that buys, because it is most of the difference:

    hierarchy      three road classes at three widths, not one width everywhere
    dead ends      cul-de-sacs with a real turnaround, which no BSP can make
    T-junctions    streets that stop on a cross street instead of crossing it
    loops          ring streets around an interior superblock
    grain          block size varies by neighbourhood cell, not globally

and what it does not: the streets are straight, the bulb is a square rather than
a circle, and the plan reads as rectilinear from above. The square bulb is the
least dishonest of those — IFC Figure D103.1 lists a square turnaround and a
hammerhead as approved alternatives to the circular bulb, so a rectilinear
turnaround is a real form, not a compromise shape.

CORRIDOR CONTRACT
-----------------
Identical to `city_layout`'s, so every consumer works unchanged::

    x0,y0,x1,y1  the MAXIMUM extent of the road, kerb strips included
    dir          "ns" | "ew"
    n_lanes      travelled lanes
    carriage     [c0, c1]  cross-axis span of the travelled way, centred
    park_w       reserved kerb strip per side (0 where there is no parking)
    parking      "none" | "lo" | "hi" | "both"
    zone         neighbourhood cell name, for `city_detail`'s stop_zones
    road_class   "arterial" | "collector" | "local"

plus one key of this module's own, ignored by everything else and used by the
verification pass:

    street_type  "border" | "collector" | "rail" | "rung" | "loop" | "stem"
                 | "spine" | "cul_de_sac" | "bulb" | "hammerhead"
"""

_TOL = 1e-6

# Published for `districts.park_blocks`, exactly as `city_layout.PARK_RESERVES`
# is. One city is built per process, so a module global is honest here.
PARK_RESERVES: list = []

# Filled by the last subdivision. Nothing downstream reads it; the host-side
# verification pass does, so it does not have to re-derive topology it already
# knows.
LAST_STATS: dict = {}


# Widths per functional class, all overridable from `suburb_layout.classes`.
# lanes x lane_w_m is the travelled way; parking_m is the kerb strip per side,
# so the corridor rect is lanes*lane_w_m + 2*parking_m wide.
_DEFAULT_CLASSES = {
    # Section-line arterial, 4 x 11.5 ft = 46 ft. ITE/CNU RP-036A ch.9 allows
    # 10-12 ft on arterials. No kerb parking (SUDAS 5C-2(O) forbids it there).
    "arterial":   {"lanes": 4, "lane_w_m": 3.5, "parking_m": 0.0,
                   "parking_policy": {"none": 1.0}},
    # 2 x 11 ft + 2 x 8 ft = 38 ft, which is Larimer County's Loveland Minor
    # Collector roadway exactly; 11 ft is RP-036A's collector lane.
    "collector":  {"lanes": 2, "lane_w_m": 3.35, "parking_m": 2.44,
                   "parking_policy": {"none": 0.30, "one_side": 0.45,
                                      "both": 0.25}},
    # 2 x 10.5 ft (SUDAS Table 5C-1.01 local residential) + 2 x 7 ft (RP-036A
    # residential parking) = 35 ft. SUDAS footnote 10 quantises two-lane
    # roadways to 26/31/34/37 ft, so this sits on its 34 ft step.
    "local":      {"lanes": 2, "lane_w_m": 3.20, "parking_m": 2.13,
                   "parking_policy": {"one_side": 0.30, "both": 0.70}},
    # 20 ft exactly: IFC Table D103.4's required width for a 151-500 ft dead
    # end, and RP-036A's note that fire districts want 20 ft clear. No kerb
    # parking — the whole width is the fire lane.
    "cul_de_sac": {"lanes": 2, "lane_w_m": 3.05, "parking_m": 0.0,
                   "parking_policy": {"none": 1.0}},
}

_PUBLISHED_CLASS = {"arterial": "arterial", "collector": "collector",
                    "local": "local", "cul_de_sac": "local"}


def _rng_range(v, fallback):
    """Accept ``[lo, hi]`` or a scalar for a range-valued knob."""
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    if isinstance(v, (int, float)):
        return float(v), float(v)
    return fallback


def _weighted(weights: dict, rng, fallback: str):
    """Draw a key from a ``{name: weight}`` mapping."""
    items = [(k, float(v)) for k, v in (weights or {}).items() if float(v) > 0.0]
    total = sum(w for _k, w in items)
    if total <= 0.0:
        return fallback
    r = rng.random() * total
    for k, w in items:
        r -= w
        if r <= 0.0:
            return k
    return items[-1][0]


def _rect(c):
    return (c["x0"], c["y0"], c["x1"], c["y1"])


def _sep(a, b) -> float:
    """Clearance between two rects: 0 when they touch or overlap.

    The MAX of the per-axis separations, not the diagonal distance. Two rects
    offset 5 m in x and 200 m in y are 200 m apart in the sense that matters —
    there is a whole block between them — whereas 5 m in x and 5 m in y is a
    sliver at the corner and has to be rejected. Taking the max says both.
    """
    sx = max(b[0] - a[2], a[0] - b[2], 0.0)
    sy = max(b[1] - a[3], a[1] - b[3], 0.0)
    return max(sx, sy)


def _crosses(a: dict, b: dict) -> bool:
    """True when perpendicular corridors *a* and *b* form a clean junction.

    A clean junction is each road's WIDTH lying wholly inside the other's RUN —
    a 4-way, or a T where one road's run ends inside the other. Anything else is
    two roads clipping each other's corner, which downstream would draw as a
    kinked intersection with a stub of asphalt hanging off it.
    """
    ns, ew = (a, b) if a["dir"] == "ns" else (b, a)
    return (ns["x0"] >= ew["x0"] - _TOL and ns["x1"] <= ew["x1"] + _TOL
            and ew["y0"] >= ns["y0"] - _TOL and ew["y1"] <= ns["y1"] + _TOL)


def _uniq(vals, tol=1e-6):
    out = []
    for v in sorted(vals):
        if not out or v - out[-1] > tol:
            out.append(v)
    return out


def _bands(lo: float, hi: float, t_lo: float, t_hi: float, min_gap: float,
           rng, jitter: float):
    """Cut ``[lo, hi]`` into bands each roughly ``t_lo..t_hi`` wide.

    Returns the interior boundary positions (not lo/hi). Used for collector
    spacing and for the local street grid inside a cell, so both express the
    same idea: a target block dimension, jittered, never below *min_gap*.
    """
    span = hi - lo
    target = (t_lo + t_hi) / 2.0
    n = int(round(span / target)) - 1
    while n > 0 and span / (n + 1) < min_gap:
        n -= 1
    if n <= 0:
        return []
    step = span / (n + 1)
    slack = min(step - min_gap, step * 0.35) * max(0.0, min(1.0, jitter))
    return [lo + step * (k + 1) + (rng.random() * 2.0 - 1.0) * slack / 2.0
            for k in range(n)]


def _extract_blocks(region, corridors, reserved, min_dim: float):
    """Blocks as the exact complement of the corridors inside *region*.

    Every corridor edge becomes a grid line, cells covered by a corridor are
    struck out, and what is left is merged into maximal rectangles. This is what
    replaces the BSP's "a block is a leaf of the recursion", and it is why the
    streets can be composed freely: whatever shape the street network ends up,
    the leftover land is still tiled exactly.

    It has to be exact rather than approximate. `apply_ground_planes` lays ONE
    asphalt mesh over the whole region and then a grass mesh per block, so any
    land no block covers is rendered as road.

    *reserved* rects (park superblocks) are emitted whole and their cells
    pre-claimed, so a park is never chopped into several blocks.
    """
    rx0, ry0, rx1, ry1 = region
    xs = _uniq([rx0, rx1] + [v for c in corridors for v in (c["x0"], c["x1"])]
               + [v for r in reserved for v in (r[0], r[2])])
    ys = _uniq([ry0, ry1] + [v for c in corridors for v in (c["y0"], c["y1"])]
               + [v for r in reserved for v in (r[1], r[3])])
    xs = [v for v in xs if rx0 - _TOL <= v <= rx1 + _TOL]
    ys = [v for v in ys if ry0 - _TOL <= v <= ry1 + _TOL]
    nx, ny = len(xs) - 1, len(ys) - 1
    if nx <= 0 or ny <= 0:
        return []

    rects = [_rect(c) for c in corridors]
    free = [[True] * nx for _ in range(ny)]
    for j in range(ny):
        cy = (ys[j] + ys[j + 1]) / 2.0
        for i in range(nx):
            cx = (xs[i] + xs[i + 1]) / 2.0
            for (a0, b0, a1, b1) in rects:
                if a0 < cx < a1 and b0 < cy < b1:
                    free[j][i] = False
                    break

    taken = [[False] * nx for _ in range(ny)]
    reserved_blocks = []
    for r in reserved:
        for j in range(ny):
            if not (r[1] - _TOL <= ys[j] and ys[j + 1] <= r[3] + _TOL):
                continue
            for i in range(nx):
                if r[0] - _TOL <= xs[i] and xs[i + 1] <= r[2] + _TOL:
                    taken[j][i] = True
        reserved_blocks.append(tuple(r))

    def open_at(i, j):
        return free[j][i] and not taken[j][i]

    def maximal(i, j):
        """Largest-area rectangle anchored at (i, j).

        Not "grow one axis to exhaustion then the other" — that greedy runs the
        first row out past the block it is really in, and every subsequent row
        then has to come off as its own thin strip. Widening one column at a
        time and taking the best area over all widths is what keeps a 200x150 m
        parcel one block instead of six ribbons.
        """
        best = (0.0, i, j)
        jmax = ny - 1
        for i1 in range(i, nx):
            if not open_at(i1, j):
                break
            k = j
            while k + 1 <= jmax and open_at(i1, k + 1):
                k += 1
            jmax = min(jmax, k)
            a = (xs[i1 + 1] - xs[i]) * (ys[jmax + 1] - ys[j])
            if a > best[0]:
                best = (a, i1, jmax)
        imax = nx - 1
        for j1 in range(j, ny):
            if not open_at(i, j1):
                break
            k = i
            while k + 1 <= imax and open_at(k + 1, j1):
                k += 1
            imax = min(imax, k)
            a = (xs[imax + 1] - xs[i]) * (ys[j1 + 1] - ys[j])
            if a > best[0]:
                best = (a, imax, j1)
        return best[1], best[2]

    blocks = []
    for j in range(ny):
        for i in range(nx):
            if not open_at(i, j):
                continue
            i1, j1 = maximal(i, j)
            for jj in range(j, j1 + 1):
                for ii in range(i, i1 + 1):
                    taken[jj][ii] = True
            blocks.append((xs[i], ys[j], xs[i1 + 1], ys[j1 + 1]))

    # Stair-step cleanup. Two blocks sharing a whole edge are one parcel, and
    # the union of two street-free rects is street-free, so merging them can
    # never put a block over a road.
    changed = True
    while changed:
        changed = False
        for a in range(len(blocks)):
            for b in range(a + 1, len(blocks)):
                p, q = blocks[a], blocks[b]
                same_x = abs(p[0] - q[0]) < _TOL and abs(p[2] - q[2]) < _TOL
                same_y = abs(p[1] - q[1]) < _TOL and abs(p[3] - q[3]) < _TOL
                if not ((same_x and (abs(p[3] - q[1]) < _TOL
                                     or abs(q[3] - p[1]) < _TOL))
                        or (same_y and (abs(p[2] - q[0]) < _TOL
                                        or abs(q[2] - p[0]) < _TOL))):
                    continue
                blocks[a] = (min(p[0], q[0]), min(p[1], q[1]),
                             max(p[2], q[2]), max(p[3], q[3]))
                blocks.pop(b)
                changed = True
                break
            if changed:
                break

    blocks = _absorb_thin(blocks, min_dim, rects)
    LAST_STATS["sliver_blocks"] = sum(
        1 for b in blocks if min(b[2] - b[0], b[3] - b[1]) < min_dim)
    return reserved_blocks + blocks


def _absorb_thin(blocks, min_dim, corridor_rects):
    """Fold ribbons narrower than *min_dim* into the block they abut.

    Tiling a notched parcel with rectangles produces ribbons — the land beside a
    cul-de-sac shaft but behind its wider turnaround genuinely is a 12 m strip,
    and no rectangular decomposition can avoid emitting it. Most of them,
    though, are not real: they are the leftover after the greedy took the big
    rectangle out of an L, and the neighbour they abut is one parcel with them.

    The merge is exact rather than approximate. The neighbour is cut along the
    ribbon's own span and only the overlapping slice is grown, so the pieces
    still partition exactly the same land: no gaps for `apply_ground_planes` to
    render as asphalt, no overlaps, and — since both inputs are street-free —
    nothing lands on a road. A cut that would itself leave a ribbon is refused.
    """
    def absorb(t, b):
        """b re-cut so it swallows t, or None if that would make things worse."""
        for ax in (0, 1):
            lo_t, hi_t = (t[ax], t[ax + 2])
            lo_b, hi_b = (b[ax], b[ax + 2])
            cross_t = (t[1 - ax], t[3 - ax])
            cross_b = (b[1 - ax], b[3 - ax])
            if not (abs(cross_t[1] - cross_b[0]) < _TOL
                    or abs(cross_b[1] - cross_t[0]) < _TOL):
                continue                      # not abutting on this axis
            if not (lo_b - _TOL <= lo_t and hi_t <= hi_b + _TOL):
                continue                      # ribbon overhangs the neighbour
            if (0.0 < lo_t - lo_b < min_dim) or (0.0 < hi_b - hi_t < min_dim):
                continue                      # the cut would leave a new ribbon
            merged_cross = (min(cross_t[0], cross_b[0]),
                            max(cross_t[1], cross_b[1]))
            out = []
            for a0, a1, c in ((lo_b, lo_t, cross_b), (hi_t, hi_b, cross_b),
                              (lo_t, hi_t, merged_cross)):
                if a1 - a0 <= _TOL:
                    continue
                out.append((a0, c[0], a1, c[1]) if ax == 0
                           else (c[0], a0, c[1], a1))
            # Cutting a block can strand the middle slice with no kerb of its
            # own — every side of it another block — and a block with no street
            # frontage has no driveway, no walk and no way to reach a house.
            if any(all(_sep(p, r) > _TOL for r in corridor_rects) for p in out):
                return None
            return out
        return None

    out = list(blocks)
    for _pass in range(3 * len(blocks) + 8):
        thin = sorted((k for k in range(len(out))
                       if min(out[k][2] - out[k][0],
                              out[k][3] - out[k][1]) < min_dim),
                      key=lambda k: ((out[k][2] - out[k][0])
                                     * (out[k][3] - out[k][1])))
        if not thin:
            break
        done = False
        for k in thin:
            for j in range(len(out)):
                if j == k:
                    continue
                new = absorb(out[k], out[j])
                if new is None:
                    continue
                out = ([out[i] for i in range(len(out)) if i not in (j, k)]
                       + new)
                done = True
                break
            if done:
                break
        if not done:
            break
    return out


def make_subdivider(layout_cfg: dict, config: dict = None):
    """Return a replacement for `scene_generator._subdivide_region_metric`.

    *layout_cfg* is ``config["suburb_layout"]``. *config* is accepted for
    signature parity with `city_layout.make_subdivider` and is not otherwise
    needed: a suburb has no district rings, so nothing has to be looked up.
    """
    cfg = layout_cfg or {}
    classes = cfg.get("classes") or {}

    def cls_cfg(name):
        d = dict(_DEFAULT_CLASSES[name])
        d.update(classes.get(name) or {})
        return d

    def cls_half_w(name):
        c = cls_cfg(name)
        return (int(c["lanes"]) * float(c["lane_w_m"])
                + 2.0 * float(c["parking_m"])) / 2.0

    col_lo, col_hi = _rng_range(cfg.get("collector_spacing_m"), (380.0, 620.0))
    bs_lo, bs_hi = _rng_range(cfg.get("block_short_m"), (72.0, 118.0))
    bl_lo, bl_hi = _rng_range(cfg.get("block_long_m"), (165.0, 265.0))
    patterns = cfg.get("cell_patterns") or {"ladder": 0.40, "loop": 0.35,
                                            "cul_cluster": 0.25}
    through_chance = float(cfg.get("rung_through_chance", 0.30))

    cul = cfg.get("cul_de_sac") or {}
    cul_lo, cul_hi = _rng_range(cul.get("per_km2"), (10.0, 18.0))
    len_lo, len_hi = _rng_range(cul.get("length_m"), (46.0, 140.0))
    # IFC Figure D103.1: 48 ft radius (96 ft driving-surface diameter), and a
    # 120 ft hammerhead reaching 60 ft either side of the stem centreline. Both
    # round UP off the foot figure — they are code minima, not targets.
    bulb_r = float(cul.get("bulb_radius_m", 14.64))
    hammer_share = float(cul.get("hammerhead_share", 0.25))
    hammer_arm = float(cul.get("hammerhead_arm_m", 15.25))

    pk = cfg.get("parks") or {}
    park_on = bool(pk.get("enabled", True))
    pk_lo, pk_hi = _rng_range(pk.get("count"), (1.0, 2.0))
    pk_side_lo, pk_side_hi = _rng_range(pk.get("side_m"), (150.0, 250.0))

    def subdivide(w_m, h_m, min_block_m, max_block_m, jitter, rng, roads_cfg):
        # Two lot depths back to back is the smallest honest gap between two
        # parallel streets; anything less is a sliver that no lot can front.
        # US suburban lot depth is 100-120 ft, so 62 m is a shallow-lot suburb.
        min_gap = float(cfg.get("min_gap_m", 62.0))
        # A cul-de-sac may back onto the next street across ONE lot depth: the
        # bulb's lots wrap it, the other street's lots back onto them.
        stub_gap = float(cfg.get("stub_gap_m", 34.0))
        jitter = float(cfg.get("jitter", jitter))

        corridors: list = []
        reserves: list = []
        half_w, half_h = w_m / 2.0, h_m / 2.0
        region = (-half_w, -half_h, half_w, half_h)

        def street(sp, r0, r1, dirn, cls, kind, zone=None, half=None,
                   carriage_w=None):
            """Build (do not yet register) one corridor centred on *sp*.

            *sp* is the centreline on the cross axis, ``[r0, r1]`` the run. The
            rect is the FULL width including kerb strips, because that is what
            `apply_ground_planes` paints and what the frontage walk measures
            against; the travelled way rides along in ``carriage``.
            """
            cc = cls_cfg(cls)
            lanes = int(cc["lanes"])
            car_w = float(carriage_w if carriage_w is not None
                          else lanes * float(cc["lane_w_m"]))
            hw = float(half if half is not None
                       else car_w / 2.0 + float(cc["parking_m"]))
            pw = float(cc["parking_m"])
            policy = _weighted(cc.get("parking_policy"), rng, "both")
            if policy == "one_side":
                policy = "lo" if rng.random() < 0.5 else "hi"
            if pw <= 0.0:
                policy = "none"
            c = {"n_lanes": lanes, "park_w": pw, "parking": policy,
                 "zone": zone, "road_class": _PUBLISHED_CLASS[cls],
                 "street_type": kind, "dir": dirn,
                 "carriage": [sp - car_w / 2.0, sp + car_w / 2.0]}
            if dirn == "ns":
                c.update(x0=sp - hw, y0=r0, x1=sp + hw, y1=r1)
            else:
                c.update(x0=r0, y0=sp - hw, x1=r1, y1=sp + hw)
            return c

        def fits(c, gap, hosts=(), no_touch=(), only_hosts=False):
            """True when *c* can be added: inside the region, and against every
            existing corridor either a clean junction or *gap* metres clear.

            *no_touch* names street types *c* may not junction with at all,
            whatever the geometry. That is what stops one cul-de-sac branching
            off another: the geometry of a T into a dead-end shaft is perfectly
            valid, but the result is a dead end reachable through a dead end,
            which no subdivision ordinance permits and which would break the
            "exactly one street" invariant the turnaround is sized for.
            """
            if not (region[0] - _TOL <= c["x0"] and c["x1"] <= region[2] + _TOL
                    and region[1] - _TOL <= c["y0"]
                    and c["y1"] <= region[3] + _TOL):
                return False
            for r in reserves:
                if _sep(_rect(c), r) < min(gap, stub_gap) - _TOL:
                    return False
            hid = {id(h) for h in hosts}
            for o in corridors:
                s = _sep(_rect(c), _rect(o))
                if s > _TOL:
                    if s < gap - _TOL:
                        return False
                    continue
                if id(o) in hid:            # deliberate junction or abutment
                    continue
                if only_hosts or o["street_type"] in no_touch:
                    return False
                if o["dir"] == c["dir"]:    # two parallel roads on top of each other
                    return False
                if not _crosses(c, o):
                    return False
            return True

        def add(c):
            corridors.append(c)
            return c

        # ---- level 0: the section-line arterial frame -----------------------
        # Border ring, exactly as the built-in lays it, but at arterial width.
        # The suburb is one section of the historic 1-mile arterial grid, so the
        # frame is the arterial network and nothing inside it needs to be one.
        brd_hw = cls_half_w("arterial")
        brd = [
            street(-half_h + brd_hw, -half_w, half_w, "ew", "arterial",
                   "border", zone="edge"),
            street(half_h - brd_hw, -half_w, half_w, "ew", "arterial",
                   "border", zone="edge"),
            street(-half_w + brd_hw, -half_h + 2 * brd_hw, half_h - 2 * brd_hw,
                   "ns", "arterial", "border", zone="edge"),
            street(half_w - brd_hw, -half_h + 2 * brd_hw, half_h - 2 * brd_hw,
                   "ns", "arterial", "border", zone="edge"),
        ]
        for c in brd:
            add(c)
        in0, in1 = -half_w + 2 * brd_hw, half_w - 2 * brd_hw
        in2, in3 = -half_h + 2 * brd_hw, half_h - 2 * brd_hw

        # ---- level 1: collectors -------------------------------------------
        # Collectors are the only interior roads that cross the whole suburb.
        # Everything below them is a local street that serves lots and stops.
        col_hw = cls_half_w("collector")
        xs_c = _bands(in0, in1, col_lo, col_hi, 2 * min_gap, rng, jitter)
        ys_c = _bands(in2, in3, col_lo, col_hi, 2 * min_gap, rng, jitter)
        col_ns, col_ew = [], []
        for sp in xs_c:
            c = street(sp, -half_h + brd_hw, half_h - brd_hw, "ns", "collector",
                       "collector", zone="collector")
            if fits(c, min_gap, hosts=brd):
                col_ns.append(add(c))
        for sp in ys_c:
            c = street(sp, -half_w + brd_hw, half_w - brd_hw, "ew", "collector",
                       "collector", zone="collector")
            if fits(c, min_gap, hosts=brd + col_ns):
                col_ew.append(add(c))

        # Neighbourhood cells: the free rects between collectors, each with the
        # four corridors that bound it. A local street that terminates on a cell
        # boundary must run to that corridor's FAR edge, or the junction is a
        # corner-clip rather than a T (see _crosses).
        x_edges = [(brd[2], in0)] + [(c, c["x1"]) for c in col_ns]
        x_stops = [(c, c["x0"]) for c in col_ns] + [(brd[3], in1)]
        y_edges = [(brd[0], in2)] + [(c, c["y1"]) for c in col_ew]
        y_stops = [(c, c["y0"]) for c in col_ew] + [(brd[1], in3)]

        cells = []
        for i, ((cw, x0), (ce, x1)) in enumerate(zip(x_edges, x_stops)):
            for j, ((cs, y0), (cn, y1)) in enumerate(zip(y_edges, y_stops)):
                cells.append({"rect": (x0, y0, x1, y1), "name": f"cell{i}{j}",
                              "w": cw, "e": ce, "s": cs, "n": cn})

        # ---- park superblocks ----------------------------------------------
        # A park is a loop cell whose interior is never built on: the ring road
        # around it becomes the park's four frontages, which is what a real
        # neighbourhood park has, and `parks.py` then composes the interior.
        # Sized in ABSOLUTE metres rather than as a multiple of the block target
        # the way the urban scene does it, because the ask here was explicitly
        # for bigger parks than the urban ones and a suburb's block target is
        # the wrong yardstick for a park that serves the whole neighbourhood.
        want_parks = rng.randint(int(pk_lo), int(pk_hi)) if park_on else 0
        park_cells = set()
        if want_parks:
            room = sorted(
                (k for k, c in enumerate(cells)
                 if min(c["rect"][2] - c["rect"][0],
                        c["rect"][3] - c["rect"][1]) > pk_side_lo + 2 * min_gap),
                key=lambda k: -((cells[k]["rect"][2] - cells[k]["rect"][0])
                                * (cells[k]["rect"][3] - cells[k]["rect"][1])))
            for k in room[:want_parks]:
                park_cells.add(k)

        local_hw = cls_half_w("local")

        def loop(cell, park=False):
            """Ring road inset from the cell boundary, plus stems out to it.

            Interior of the ring is one superblock — a park when *park*, a deep
            back-lot cluster otherwise. The ring corners are where the ns and ew
            legs overlap, which downstream reads as a junction, so the loop is a
            connected circuit and not four dangling streets.
            """
            x0, y0, x1, y1 = cell["rect"]
            w, h = x1 - x0, y1 - y0
            if park:
                side = rng.uniform(pk_side_lo, pk_side_hi)
                d = max(min_gap, min((w - side) / 2.0, (h - side) / 2.0))
            else:
                d = rng.uniform(bs_lo, bs_hi)
            # Ring has to leave a full gap outside AND inside, or the interior
            # superblock is a sliver and the frontage strip cannot hold a lot.
            if (min(w, h) - 2.0 * (d + local_hw)) < min_gap or d < min_gap:
                return False
            legs = []
            for sp, dirn in ((x0 + d, "ns"), (x1 - d, "ns"),
                             (y0 + d, "ew"), (y1 - d, "ew")):
                r0, r1 = ((y0 + d - local_hw, y1 - d + local_hw) if dirn == "ns"
                          else (x0 + d - local_hw, x1 - d + local_hw))
                legs.append(street(sp, r0, r1, dirn, "local", "loop",
                                   zone=cell["name"]))
            hosts = [cell[k] for k in "wesn"]
            for k, leg in enumerate(legs):
                if not fits(leg, min_gap, hosts=hosts + legs[:k]):
                    return False
            undo = len(corridors)
            for leg in legs:
                add(leg)

            # Stems. A ring with no stem is an island — four streets reachable
            # from nothing, which is what left the network in three components
            # before this was enforced. So the ring is only kept if at least one
            # stem lands, and rolled back whole otherwise.
            #
            # The stem is parallel to two of the ring's own legs and its run
            # overlaps them at the ring corner, so it needs a full min_gap of
            # clearance from them: an inset of min_gap/2 puts it a lot-and-a-bit
            # from the ring corner and every stem is rejected.
            n_stems = 0
            for side in rng.sample(["w", "e", "s", "n"], 4):
                if n_stems >= 2:
                    break
                host = cell[side]
                ew = side in ("w", "e")
                a, b = ((y0, y1) if ew else (x0, x1))
                lo = a + d + 2.0 * local_hw + min_gap
                hi = b - d - 2.0 * local_hw - min_gap
                if hi <= lo:
                    continue
                sp = rng.uniform(lo, hi)
                if ew:
                    r0, r1 = ((host["x0"], x0 + d + local_hw) if side == "w"
                              else (x1 - d - local_hw, host["x1"]))
                else:
                    r0, r1 = ((host["y0"], y0 + d + local_hw) if side == "s"
                              else (y1 - d - local_hw, host["y1"]))
                if r1 - r0 < 2.0 * local_hw:
                    continue
                stem = street(sp, r0, r1, "ew" if ew else "ns", "local", "stem",
                              zone=cell["name"])
                if fits(stem, min_gap, hosts=[host] + legs):
                    add(stem)
                    n_stems += 1
            if not n_stems:
                del corridors[undo:]
                return False
            if park:
                reserves.append((x0 + d + local_hw, y0 + d + local_hw,
                                 x1 - d - local_hw, y1 - d - local_hw))
            return True

        def ladder(cell):
            """Rails along the cell's long axis, rungs across it.

            Rungs mostly stop on a rail instead of running through to the far
            collector — that is where the three-way junctions come from, and a
            measured suburb is 69-87% three-way against 38% for a pre-war grid.
            """
            x0, y0, x1, y1 = cell["rect"]
            long_x = (x1 - x0) >= (y1 - y0)
            rails, rungs = [], []
            if long_x:
                for sp in _bands(y0, y1, bs_lo, bs_hi, min_gap, rng, jitter):
                    c = street(sp, cell["w"]["x0"], cell["e"]["x1"], "ew",
                               "local", "rail", zone=cell["name"])
                    if fits(c, min_gap, hosts=[cell["w"], cell["e"]] + rails):
                        rails.append(add(c))
                cuts = _bands(x0, x1, bl_lo, bl_hi, min_gap, rng, jitter)
            else:
                for sp in _bands(x0, x1, bs_lo, bs_hi, min_gap, rng, jitter):
                    c = street(sp, cell["s"]["y0"], cell["n"]["y1"], "ns",
                               "local", "rail", zone=cell["name"])
                    if fits(c, min_gap, hosts=[cell["s"], cell["n"]] + rails):
                        rails.append(add(c))
                cuts = _bands(y0, y1, bl_lo, bl_hi, min_gap, rng, jitter)

            for sp in cuts:
                through = rng.random() < through_chance or not rails
                if long_x:
                    lo, hi = cell["s"]["y0"], cell["n"]["y1"]
                    if not through:
                        rail = rails[rng.randrange(len(rails))]
                        lo, hi = ((lo, rail["y1"]) if rng.random() < 0.5
                                  else (rail["y0"], hi))
                    c = street(sp, lo, hi, "ns", "local", "rung",
                               zone=cell["name"])
                else:
                    lo, hi = cell["w"]["x0"], cell["e"]["x1"]
                    if not through:
                        rail = rails[rng.randrange(len(rails))]
                        lo, hi = ((lo, rail["x1"]) if rng.random() < 0.5
                                  else (rail["x0"], hi))
                    c = street(sp, lo, hi, "ew", "local", "rung",
                               zone=cell["name"])
                hosts = [cell[k] for k in "wesn"] + rails + rungs
                if fits(c, min_gap, hosts=hosts):
                    rungs.append(add(c))
            return bool(rails or rungs)

        def spine(cell):
            """One local street crossing the cell, and nothing else — the frame
            a cul-de-sac cluster hangs off. Deliberately sparse: the through
            street count is what keeps street density down at the measured
            7.4-12.4 km/km² rather than a downtown's 17-23."""
            x0, y0, x1, y1 = cell["rect"]
            long_x = (x1 - x0) >= (y1 - y0)
            if long_x:
                sp = y0 + (y1 - y0) * rng.uniform(0.35, 0.65)
                c = street(sp, cell["w"]["x0"], cell["e"]["x1"], "ew", "local",
                           "spine", zone=cell["name"])
            else:
                sp = x0 + (x1 - x0) * rng.uniform(0.35, 0.65)
                c = street(sp, cell["s"]["y0"], cell["n"]["y1"], "ns", "local",
                           "spine", zone=cell["name"])
            if fits(c, min_gap, hosts=[cell[k] for k in "wesn"]):
                add(c)
                return True
            return False

        for k, cell in enumerate(cells):
            if k in park_cells:
                if loop(cell, park=True):
                    continue
            kind = _weighted(patterns, rng, "ladder")
            ok = {"loop": lambda: loop(cell), "ladder": lambda: ladder(cell),
                  "cul_cluster": lambda: spine(cell)}[kind]()
            if not ok and kind != "ladder":
                ladder(cell)

        # ---- level 3: cul-de-sacs ------------------------------------------
        # A dead end is legal only with a turnaround: IFC D103.4 requires one on
        # any dead end over 150 ft, and Figure D103.1 dimensions the forms. Both
        # forms here are from that figure — the bulb (square, ~96 ft across) and
        # the hammerhead — which is also why a rectilinear turnaround is not a
        # compromise: the code sanctions exactly these shapes.
        cul_hw = cls_half_w("cul_de_sac")

        def try_cul_de_sac():
            # Access management: a cul-de-sac hangs off a local street. Coming
            # straight off a collector is possible but rare, because every
            # driveway-equivalent access onto a collector is a conflict point
            # the hierarchy exists to avoid.
            local_hosts = [c for c in corridors
                           if c["street_type"] in ("rail", "rung", "loop",
                                                   "spine", "stem")]
            col_hosts = [c for c in corridors
                         if c["street_type"] == "collector"]
            hostable = local_hosts
            if col_hosts and (not local_hosts or rng.random() < 0.10):
                hostable = col_hosts
            if not hostable:
                return False
            host = hostable[rng.randrange(len(hostable))]
            ns_host = host["dir"] == "ns"
            # The shaft leaves the host at right angles, so its direction is the
            # host's opposite, and it spans the host's full width to make a
            # clean T rather than clipping the kerb.
            dirn = "ew" if ns_host else "ns"
            run_lo, run_hi = ((host["y0"], host["y1"]) if ns_host
                              else (host["x0"], host["x1"]))
            if run_hi - run_lo < 4.0 * cul_hw:
                return False
            sp = rng.uniform(run_lo + 2.0 * cul_hw, run_hi - 2.0 * cul_hw)
            length = rng.uniform(len_lo, len_hi)
            hammer = rng.random() < hammer_share
            base_lo, base_hi = ((host["x0"], host["x1"]) if ns_host
                                else (host["y0"], host["y1"]))
            out = 1.0 if rng.random() < 0.5 else -1.0
            root = base_hi if out > 0 else base_lo
            tip = root + out * length
            shaft_r = (root, tip) if out > 0 else (tip, root)

            if hammer:
                shaft = street(sp, shaft_r[0], shaft_r[1], dirn, "cul_de_sac",
                               "cul_de_sac", zone=host.get("zone"))
                arm_sp = tip - out * cul_hw
                arm = street(arm_sp, sp - hammer_arm - cul_hw,
                             sp + hammer_arm + cul_hw,
                             "ns" if dirn == "ew" else "ew", "cul_de_sac",
                             "hammerhead", zone=host.get("zone"))
            else:
                # Bulb abuts the shaft end rather than overlapping it, so two
                # same-direction rects never sit on top of each other.
                neck = tip - out * 2.0 * bulb_r
                shaft = street(sp, min(root, neck), max(root, neck), dirn,
                               "cul_de_sac", "cul_de_sac", zone=host.get("zone"))
                if abs(neck - root) < 2.0 * cul_hw:
                    return False
                arm = street(sp, min(neck, tip), max(neck, tip), dirn,
                             "cul_de_sac", "bulb", zone=host.get("zone"),
                             half=bulb_r, carriage_w=2.0 * bulb_r)
            # only_hosts, not merely "no other dead end": a shaft that cleanly
            # T's into a SECOND through street is geometrically fine but is then
            # a connector, not a dead end, and the turnaround on its far end is
            # nonsense. The one legal junction is the one it left.
            if not fits(shaft, stub_gap, hosts=[host], only_hosts=True):
                return False
            add(shaft)
            if not fits(arm, stub_gap, hosts=[shaft], only_hosts=True):
                del corridors[-1:]
                return False
            add(arm)
            return True

        km2 = (w_m / 1000.0) * (h_m / 1000.0)
        want_cul = int(round(rng.uniform(cul_lo, cul_hi) * km2))
        made = 0
        for _ in range(want_cul * 40):
            if made >= want_cul:
                break
            if try_cul_de_sac():
                made += 1

        blocks = _extract_blocks(region, corridors, reserves, min_block_m)
        PARK_RESERVES[:] = [b for b in blocks if b in {tuple(r) for r in reserves}]

        # districts.park_blocks() reads city_layout.PARK_RESERVES, and that is
        # the only channel parks.py has for "this block is a park". Writing the
        # global is a runtime handshake with a module this work may not edit —
        # not an edit to it. Guarded because city_layout is the urban scene's
        # and need not be importable for a suburb run.
        try:
            import city_layout
            city_layout.PARK_RESERVES[:] = list(PARK_RESERVES)
        except ImportError:
            pass

        length_km = sum((c["x1"] - c["x0"]) if c["dir"] == "ew"
                        else (c["y1"] - c["y0"]) for c in corridors) / 1000.0
        kinds = {}
        for c in corridors:
            kinds[c["street_type"]] = kinds.get(c["street_type"], 0) + 1
        LAST_STATS.update(
            blocks=len(blocks), corridors=len(corridors), parks=len(reserves),
            cul_de_sacs=made, cul_de_sac_target=want_cul, kinds=kinds,
            street_km=length_km, street_km_per_km2=length_km / max(km2, 1e-9),
            cells=len(cells))
        print(f"[suburb_layout] {len(blocks)} blocks, {len(corridors)} corridors, "
              f"{made}/{want_cul} cul-de-sacs, {len(reserves)} park superblock(s), "
              f"{len(cells)} cells\n"
              f"[suburb_layout]   {length_km:.1f} km of street "
              f"({length_km / max(km2, 1e-9):.1f} km/km2)   "
              + "  ".join(f"{k}={v}" for k, v in sorted(kinds.items())))
        return blocks, corridors

    return subdivide


class patched:
    """Context manager swapping the generator's subdivider for the run.

    Same contract as `city_layout.patched`, and mutually exclusive with it in
    practice: a config enables one or the other. Monkey-patching rather than a
    parameter because `build_city` calls `_subdivide_region_metric` internally
    and this work may not edit that file.
    """

    def __init__(self, config: dict):
        self._full = config
        self._cfg = (config.get("suburb_layout") or {})
        self._enabled = bool(self._cfg.get("enabled"))
        self._orig = None

    def __enter__(self):
        if not self._enabled:
            return False
        import scene_generator as sg
        self._orig = sg._subdivide_region_metric
        sg._subdivide_region_metric = make_subdivider(self._cfg, self._full)
        print("[suburb_layout] hierarchical suburb subdivision: "
              f"collector spacing {self._cfg.get('collector_spacing_m')} m, "
              f"blocks {self._cfg.get('block_short_m')} x "
              f"{self._cfg.get('block_long_m')} m")
        return True

    def __exit__(self, *exc):
        if self._orig is not None:
            import scene_generator as sg
            sg._subdivide_region_metric = self._orig
        return False
