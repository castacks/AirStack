"""
parks.py — lay out the park superblocks as one designed place each.

WHAT build_city DOES WITH A PARK BLOCK
--------------------------------------
`packing.park_block_chance` picks whole blocks and skips packing them, then a
later pass gives each ONE Catmull-Rom trail entering and leaving at opposite
edge midpoints, benches / lamps / bins walked along that single trail at fixed
spacings on a random side, optionally one playground, and rocks, plants and
trees rejection-sampled over the whole block — trees with `even=True`, i.e.
best-candidate sampling for a deliberately *uniform* spread.

WHY THAT READS AS TWO HALF-PARKS
--------------------------------
A park block is a BSP leaf, and the BSP puts a road wherever it splits. Two
park blocks that happen to land side by side therefore come out as two
independently-composed halves with a street down the middle — and because each
half is internally symmetric (gates at edge midpoints, a hub in the middle, a
rounded-rectangle promenade) the pair reads as one design mirrored about the
road. `city_layout` now carves a park SUPERBLOCK out of the region before it
subdivides, so a park is one leaf with streets on its sides and no road through
it, and this module composes that one rect. Nothing here merges anything.

The scene config should set `packing.park_block_chance: 0` (and min/max_parks
0) so `build_city` makes no parks of its own; the reserved superblock is not in
its `park_set`, so it also PAVES it — the concrete tiles are stripped below.

CIRCULATION
-----------
Olmsted's rules, as far as an axis-aligned generator can carry them:
circulation is separated by kind (a promenade around the edge, quieter walks
through the middle); walks are curvilinear and meet at obtuse angles; entrances
sit at corners and pull desire lines diagonally across; a broad greensward is
protected in the middle and planting is massed at the boundary to screen the
city out. Walks also have to LEAD somewhere, so each interior junction is an
attraction — a fountain, a playground, a seating plaza — and the planting goes
in the cells between walks rather than scattered over the whole block.

THE ATTRACTION AS A COMPOSED CENTREPIECE
----------------------------------------
An attraction is a circle, not a point. Around its keep-out sits a concentric
promenade; every walk in the park is CLIPPED to the outside of that promenade,
so a desire line approaches the fountain, lands on the ring, and the ring
carries you around it — no walk crosses the water. Benches stand just outside
the ring facing inward (the seat normal is the placement yaw + 90°, which is
what the along-path bench convention below encodes), and the planting is held
out of a clearing wider still, so the thing the walks lead to is actually
visible instead of buried in canopy. The composition is per-kind only in what
stands at the centre: fountain, play structures, or an empty seating circle.

EVERY CATEGORY RESERVES ITS FOOTPRINT
-------------------------------------
One `_Occ` per park, shared by paving, furniture, planting, rocks, railing and
people, holding each prop's MEASURED footprint (see `_Occ` for why it reserves
an oriented rectangle where `city_detail._Occupancy` reserves an AABB). Nothing
is placed except through `place()`, so a category cannot opt out of it.
"""

import math

from scene_generator import (_catmull_rom_points, _in_exclusion, _in_rect,
                             _normalize_usd_list, _walk_polyline)

# Ground heights build_city places on: apply_ground_planes puts the block grass
# plane at 0.01, and path tiles go a hair above it.
_GRASS_TOP = 0.01
_TRAIL_Z = 0.015

# Categories this module owns inside a park. `concrete` is here because the
# reserved superblock is not in build_city's park_set, so its paving pass tiles
# the whole interior at z=0.02 — on top of the grass at z=0.01. The categories
# only this module emits are listed too, so a re-run replaces its own work
# instead of doubling it.
_OWNED = ("trail", "concrete", "bench", "streetlight", "trash_can", "tree",
          "plant", "rock", "human", "play_structure", "house", "park_feature",
          "fence")

# Paving is 2 cm proud of the grass and people stand on it, so neither is a
# collision; everything else in a park is a solid object at eye height.
_FLAT = ("trail", "concrete")


def _pair(v, fallback):
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    if isinstance(v, (int, float)):
        return float(v), float(v)
    return fallback


class _Lib:
    """The `usds` pools a park needs, with per-asset scale / axis / yaw / tags."""

    def __init__(self, config: dict):
        self._default = float(config.get("asset_scale", 1.0))
        self._root = str(config.get("asset_root", "") or "")
        self._usds = config.get("usds") or {}
        self._sc, self._au, self._yo, self._tags = {}, {}, {}, {}

    def pool(self, key, tag=None, without=None):
        raw = ((self._usds.get("tiles") or {}).get("trail") if key == "trail"
               else self._usds.get(key))
        return self.normalize(raw, tag, without)

    def normalize(self, raw, tag=None, without=None):
        paths, sc, au, yo, tags = _normalize_usd_list(raw, self._default,
                                                     self._root)
        self._sc.update(sc)
        self._au.update(au)
        self._yo.update(yo)
        self._tags.update(tags)
        if tag is not None:
            paths = [p for p in paths if tag in self._tags.get(p, ())]
        if without is not None:
            paths = [p for p in paths if without not in self._tags.get(p, ())]
        return paths

    def scale(self, u):
        return self._sc.get(u, self._default)

    def axis_up(self, u):
        return self._au.get(u, "Z")

    def yaw_offset(self, u):
        return self._yo.get(u, 0.0)


# ---------------------------------------------------------------------------
# occupancy: one shared footprint reservation for every category
# ---------------------------------------------------------------------------

def _box(x, y, yaw_deg, sx, sy):
    """An oriented footprint: centre, half-extents, and its rotation."""
    a = math.radians(yaw_deg)
    return (x, y, max(1e-3, sx) / 2.0, max(1e-3, sy) / 2.0,
            math.cos(a), math.sin(a))


def _box_hit(a, b):
    """Separating-axis test between two oriented rectangles."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    for r, s in ((a, b), (b, a)):
        for ux, uy, h in ((r[4], r[5], r[2]), (-r[5], r[4], r[3])):
            reach = (abs(s[2] * (s[4] * ux + s[5] * uy))
                     + abs(s[3] * (-s[5] * ux + s[4] * uy)))
            if abs(dx * ux + dy * uy) > h + reach:
                return False
    return True


def _box_extent(b):
    return (abs(b[2] * b[4]) + abs(b[3] * b[5]),
            abs(b[2] * b[5]) + abs(b[3] * b[4]))


class _Occ:
    """Footprint reservation over one shared bucket grid, per park.

    Modelled on `city_detail._Occupancy` — reserve the prop's measured
    footprint with a pad, never a point per prop — but the rectangle is
    ORIENTED. On a kerb every prop is square to the street and an AABB is
    exact; a park yaws everything to a curving walk, and the AABB of a 4 m path
    tile turned 45° is 5.8 m across, which refuses the bench beside it and
    still misses the corner it does cover. The grid is only a bucketing scheme
    so the test stays local.

    The pad is applied to the CANDIDATE only, so the guaranteed gap between any
    two props is exactly `pad_m` rather than twice it.
    """

    def __init__(self, cell_m: float = 6.0, pad_m: float = 0.2):
        self.cell = max(1.0, float(cell_m))
        self.pad = max(0.0, float(pad_m))
        self._cells = {}

    def _keys(self, box):
        ex, ey = _box_extent(box)
        c = self.cell
        return [(gx, gy)
                for gx in range(int(math.floor((box[0] - ex) / c)),
                                int(math.floor((box[0] + ex) / c)) + 1)
                for gy in range(int(math.floor((box[1] - ey) / c)),
                                int(math.floor((box[1] + ey) / c)) + 1)]

    def free(self, box, ignore=()) -> bool:
        pad = (box[0], box[1], box[2] + self.pad, box[3] + self.pad,
               box[4], box[5])
        for k in self._keys(pad):
            for cat, other in self._cells.get(k, ()):
                if cat in ignore:
                    continue
                if _box_hit(pad, other):
                    return False
        return True

    def take(self, box, cat) -> None:
        for k in self._keys(box):
            self._cells.setdefault(k, []).append((cat, box))

    def reserve(self, box, cat, ignore=()) -> bool:
        """Claim *box*; False (and nothing claimed) if it hits a reservation."""
        if not self.free(box, ignore):
            return False
        self.take(box, cat)
        return True


class _Field:
    """Bucketed point set with a bounded nearest-point query.

    The planting tests every candidate against the whole walk network; at a
    few thousand path samples that is the hot loop, and bucketing turns it into
    a nine-cell lookup.
    """

    def __init__(self, pts, cell: float):
        self.cell = max(1.0, float(cell))
        self.g = {}
        for x, y in pts:
            self.g.setdefault((int(math.floor(x / self.cell)),
                               int(math.floor(y / self.cell))), []).append((x, y))

    def min_d2(self, x, y):
        """Squared distance to the nearest point, or inf beyond one cell."""
        gx, gy = int(math.floor(x / self.cell)), int(math.floor(y / self.cell))
        best = float("inf")
        for ax in range(gx - 1, gx + 2):
            for ay in range(gy - 1, gy + 2):
                for px, py in self.g.get((ax, ay), ()):
                    best = min(best, (x - px) ** 2 + (y - py) ** 2)
        return best


# ---------------------------------------------------------------------------
# circulation geometry
# ---------------------------------------------------------------------------

def _irregular_loop(rect, rng, wobble=0.16, n=14):
    """A closed curvilinear promenade inside *rect*.

    Deliberately not a rounded rectangle: a constant-radius outline is exactly
    what makes two parks read as mirror images of one another.
    """
    x0, y0, x1, y1 = rect
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    # Budget for the wobble INSIDE the rect. On axis k_ is 1 and f reaches
    # 1 + wobble, so using the half-extent directly swung the loop 16% past the
    # rect — MEASURED, a 36.5 m stretch of promenade fell outside the park and
    # every tile on it was dropped, because `place` bounds-checks before it
    # honours `force`. That is what the gap in the path was.
    rx = (x1 - x0) / 2.0 / (1.0 + wobble)
    ry = (y1 - y0) / 2.0 / (1.0 + wobble)
    way = []
    for k in range(n):
        a = 2.0 * math.pi * k / n
        ca, sa = math.cos(a), math.sin(a)
        # Square off toward the corners so the walk uses the whole park rather
        # than inscribing an ellipse in it.
        k_ = max(abs(ca), abs(sa)) ** 0.35
        f = 1.0 + rng.uniform(-wobble, wobble)
        way.append((cx + rx * f * ca / k_, cy + ry * f * sa / k_))
    way.append(way[0])
    # Catmull-Rom overshoots outside its control points, so clamp as well —
    # the budget above makes the clamp a no-op almost everywhere rather than
    # the thing shaping the curve.
    return [(min(max(px, x0), x1), min(max(py, y0), y1))
            for px, py in _catmull_rom_points(way, samples_per_seg=6)]


def _curved(a, b, jitter, rng):
    """Gently curved walk from *a* to *b* — a desire line, not a ruled line."""
    ax, ay = a
    bx, by = b
    nx, ny = -(by - ay), (bx - ax)
    n = math.hypot(nx, ny) or 1.0
    nx, ny = nx / n, ny / n
    way = [a]
    for t in (0.3, 0.62):
        s = rng.uniform(-jitter, jitter)
        way.append((ax + (bx - ax) * t + nx * s, ay + (by - ay) * t + ny * s))
    way.append(b)
    return _catmull_rom_points(way, samples_per_seg=8)


def _ring(cx, cy, r, n=44):
    """The concentric promenade around an attraction, as a closed polyline."""
    pts = [(cx + r * math.cos(2.0 * math.pi * k / n),
            cy + r * math.sin(2.0 * math.pi * k / n)) for k in range(n)]
    pts.append(pts[0])
    return pts


def _seg_inside(a, b, circles):
    """Merged parameter intervals of segment a->b lying inside any circle."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    aa = dx * dx + dy * dy
    if aa < 1e-12:
        return []
    hits = []
    for cx, cy, r in circles:
        fx, fy = a[0] - cx, a[1] - cy
        bb = 2.0 * (fx * dx + fy * dy)
        cc = fx * fx + fy * fy - r * r
        disc = bb * bb - 4.0 * aa * cc
        if disc <= 0.0:
            continue
        s = math.sqrt(disc)
        t0 = max(0.0, (-bb - s) / (2.0 * aa))
        t1 = min(1.0, (-bb + s) / (2.0 * aa))
        if t1 > t0:
            hits.append([t0, t1])
    hits.sort()
    merged = []
    for t0, t1 in hits:
        if merged and t0 <= merged[-1][1] + 1e-9:
            merged[-1][1] = max(merged[-1][1], t1)
        else:
            merged.append([t0, t1])
    return merged


def _clip_outside(poly, circles, min_len=8.0):
    """The runs of *poly* that lie outside every circle, cut on the boundary.

    This is what makes a walk APPROACH an attraction rather than run through
    it: the desire line is drawn to the attraction's centre and then clipped to
    the outside of its promenade, so it ends exactly on the ring and the ring
    carries the walk around. Cutting on the circle rather than dropping whole
    paths is what stops the network fragmenting when a link passes a third
    attraction on its way.
    """
    runs, cur = [], []
    for a, b in zip(poly, poly[1:]):
        outs, cursor = [], 0.0
        for t0, t1 in _seg_inside(a, b, circles):
            if t0 > cursor:
                outs.append((cursor, t0))
            cursor = max(cursor, t1)
        if cursor < 1.0:
            outs.append((cursor, 1.0))
        if (not outs or outs[0][0] > 1e-9) and cur:
            runs.append(cur)
            cur = []
        for t0, t1 in outs:
            if not cur:
                cur = [(a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0)]
            cur.append((a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1))
            if t1 < 1.0 - 1e-9:
                runs.append(cur)
                cur = []
    if cur:
        runs.append(cur)
    return [r for r in runs if _poly_len(r) >= min_len]


def _poly_len(poly):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(poly, poly[1:]))


def _walk_paths(paths, step, phase=0.0):
    """`_walk_polyline` over a whole NETWORK, carrying arc length across paths.

    Restarting the phase on every path is what bunched the furniture: a walk
    shorter than one pitch got nothing at all, a long one got a full row, and
    every path put its first prop at the same fraction of its own length. One
    running arc length over the network gives one global pitch.
    """
    if step <= 1e-9:
        return
    carry = (phase % 1.0) * step
    for path in paths:
        for x, y, yaw in _walk_polyline(path, step, phase=carry / step):
            yield x, y, yaw
        carry = (carry - _poly_len(path)) % step


def _gates(rect, rng, n_lo, n_hi):
    """Entrances: the four corners plus jittered mid-edge points.

    Corners first — a corner entrance pulls a desire line diagonally across the
    park, which is what stops the walk network from being a cross.
    """
    x0, y0, x1, y1 = rect
    span_x, span_y = x1 - x0, y1 - y0
    cand = [(x0, y0), (x1, y0), (x0, y1), (x1, y1)]
    cand += [(x0 + span_x * (0.5 + rng.uniform(-0.22, 0.22)), y0),
             (x0 + span_x * (0.5 + rng.uniform(-0.22, 0.22)), y1),
             (x0, y0 + span_y * (0.5 + rng.uniform(-0.22, 0.22))),
             (x1, y0 + span_y * (0.5 + rng.uniform(-0.22, 0.22)))]
    rng.shuffle(cand)
    want = max(3, min(len(cand), rng.randint(int(n_lo), int(n_hi))))
    return cand[:want]


def _cells(rect, cell, rng):
    """The park diced into cells of about *cell* metres, in random order.

    One planting candidate per cell is uniform BY CONSTRUCTION. Rejection
    sampling with a retry cap is not: uniform darts clump, the clumps eat the
    separation budget, and the cap runs out before the gaps are found — which
    is exactly the dense-here-empty-there planting this replaces.
    """
    x0, y0, x1, y1 = rect
    nx = max(1, int(round((x1 - x0) / cell)))
    ny = max(1, int(round((y1 - y0) / cell)))
    cw, ch = (x1 - x0) / nx, (y1 - y0) / ny
    out = [(x0 + i * cw, y0 + j * ch, cw, ch)
           for i in range(nx) for j in range(ny)]
    rng.shuffle(out)
    return out


# ---------------------------------------------------------------------------
# build
# ---------------------------------------------------------------------------

def build(config: dict, layout: dict, placements: list, resolver, rng) -> int:
    """Lay out every park superblock. Returns the number of parks built."""
    import districts

    cfg = config.get("parks") or {}
    lay = cfg.get("layout") or {}
    acfg = cfg.get("attraction") or {}
    lib = _Lib(config)

    trail_usds = lib.pool("trail")
    if not trail_usds:
        print("[parks] no tiles.trail asset — park pass skipped")
        return 0
    bench_usds = lib.pool("benches", tag="park") or lib.pool("benches")
    # Falling back to the whole pool: no streetlight in the urban_v2 library
    # carries a `park` tag, so the tagged lookup alone left parks unlit.
    light_usds = lib.pool("streetlights", tag="park") or lib.pool("streetlights")
    trash_usds = lib.pool("trash_cans", tag="park") or lib.pool("trash_cans")
    tree_usds = lib.pool("trees", without="stump")
    stump_usds = lib.pool("trees", tag="stump")
    plant_usds = lib.pool("plants")
    rock_usds = lib.pool("rocks")
    play_usds = lib.pool("play_structures")
    human_usds = lib.pool("humans")
    fence_usds = lib.pool("park_fences")
    features = (config.get("usds") or {}).get("park_features") or []

    # Composed playground sets. Each part carries an offset in the ANCHOR's own
    # frame plus a yaw relative to the group, so the arrangement is preserved
    # wherever the group lands and whichever way it faces. Normalizing each part
    # through `lib` keeps its own scale/axis/yaw-offset handling.
    groups = []
    for grp in ((config.get("usds") or {}).get("play_groups") or []):
        parts = []
        for part in (grp.get("parts") or []):
            paths = lib.normalize([part])
            if not paths:
                continue
            off = part.get("offset_m") or [0.0, 0.0]
            parts.append({"usd": paths[0],
                          "offset": (float(off[0]), float(off[1])),
                          "yaw": float(part.get("yaw_deg", 0.0))})
        if parts:
            groups.append(parts)

    exclusions = config.get("exclusions") or []
    tile_overlap = float(config.get("layout", {}).get("tile_overlap", 1.02))
    inset_m = districts.block_inset(config, resolver)

    tu = trail_usds[0]
    t_scale = lib.scale(tu) * tile_overlap
    tfp = resolver.get(tu, "trail", scale=t_scale)
    t_len, t_w = tfp["sx"], tfp["sy"]

    # Densities, not spacings: the walk network is several times the length of
    # build_city's single trail, so a fixed pitch applied to all of it gives a
    # bench every few metres. Central Park runs ~9,000 benches over 3.41 km2 —
    # one per ~380 m2 — which is the ceiling these sit under. The Downtown_West
    # source map is denser still at park scale (32 benches, 44 light posts).
    den = {"bench": float(cfg.get("bench_density_per_1000m2", 2.5)),
           "streetlight": float(cfg.get("lamp_density_per_1000m2", 1.6)),
           "trash_can": float(cfg.get("trash_can_density_per_1000m2", 0.8))}
    pitch = {"bench": float(cfg.get("bench_spacing_m", 14.0)),
             "streetlight": float(cfg.get("streetlight_spacing_m", 22.0)),
             "trash_can": float(cfg.get("trash_can_spacing_m", 30.0))}
    furn_off = float(cfg.get("furniture_offset_m", 1.2))
    path_clear = float(lay.get("path_clear_m", 1.0))
    lawn_frac = float(cfg.get("lawn_area_frac", 0.10))
    tree_sep = float(cfg.get("tree_min_separation_m", 4.0))
    plant_sep = float(cfg.get("plant_min_separation_m", 2.0))
    rock_sep = float(cfg.get("rock_min_separation_m", 2.5))
    tree_den = float(cfg.get("tree_density_per_100m2", 1.2))
    plant_den = float(cfg.get("plant_density_per_100m2", 0.8))
    # A crown is above head height: it may overhang a walk, a bench or another
    # crown, and massed canopy is what a grove IS. What may not overlap is the
    # trunk and its pit, which is the share of the crown reserved on the ground.
    canopy_frac = float(cfg.get("canopy_footprint_frac", 0.45))
    max_canopy = float(cfg.get("tree_max_canopy_m", 14.0))
    grove_area = float(cfg.get("grove_area_m2", 1600.0))
    lawn_plant_m = float(cfg.get("lawn_plant_max_m", 1.2))
    fence_gap = float(cfg.get("fence_module_gap_m", 0.0))
    pg_chance = float(cfg.get("playground_chance", 0.6))
    pg_count = _pair(cfg.get("playground_structures"), (2.0, 3.0))
    pg_spacing = float(cfg.get("playground_spacing_m", 5.0))

    ring_gap = float(acfg.get("ring_gap_m", 1.0))
    clearing_m = float(acfg.get("clearing_m", 9.0))
    ring_bench_pitch = float(acfg.get("bench_spacing_m", 9.0))
    plaza_r = float(acfg.get("plaza_radius_m", 4.0))
    occ_cell = float(cfg.get("occupancy_cell_m", 6.0))
    occ_pad = float(cfg.get("occupancy_pad_m", 0.2))

    parks = districts.park_blocks(layout, placements)
    if not parks:
        print("[parks] no park blocks")
        return 0

    def fp_of(usd, cat, scale=None):
        return resolver.get(usd, cat, scale=lib.scale(usd) if scale is None
                            else scale, axis_up=lib.axis_up(usd))

    def radius_of(usd, cat):
        fp = fp_of(usd, cat)
        return math.hypot(fp["sx"], fp["sy"]) / 2.0

    # The park pool spans 0.6 m saplings to 27 m specimens. At one separation
    # for all of them the big ones stand inside each other, which is most of
    # what reads as collision up close; the cap keeps the pool to what a walk
    # can be planted with, and the crown-fraction reservation spaces the rest
    # in proportion to their size.
    tree_usds = [u for u in tree_usds
                 if max(fp_of(u, "tree")["sx"], fp_of(u, "tree")["sy"])
                 <= max_canopy] or tree_usds

    def add(usd, x, y, z, yaw, category, scale=None):
        placements.append({
            "usd": usd, "x_m": x, "y_m": y, "z_m": z,
            "yaw_deg": yaw + lib.yaw_offset(usd),
            "roll_deg": 90.0 if lib.axis_up(usd) == "Y" else 0.0,
            "pitch_deg": 0.0,
            "scale": lib.scale(usd) if scale is None else scale,
            "category": category, "axis_up": lib.axis_up(usd)})

    # Everything build_city put inside a park comes out — including its paving.
    n_before = len(placements)
    placements[:] = [p for p in placements
                     if not (p.get("category") in _OWNED
                             and any(_in_rect(float(p["x_m"]), float(p["y_m"]),
                                              b) for b in parks))]
    cleared = n_before - len(placements)

    tally = {}
    n_refused = n_walks = 0
    walk_len = 0.0
    for blk in parks:
        rect = (blk[0] + inset_m, blk[1] + inset_m,
                blk[2] - inset_m, blk[3] - inset_m)
        x0, y0, x1, y1 = rect
        w, h = x1 - x0, y1 - y0
        if w < 12.0 or h < 12.0:
            continue
        area = w * h
        occ = _Occ(occ_cell, occ_pad)

        def blocked(px, py):
            return exclusions and _in_exclusion(px, py, exclusions)

        def place(usd, cat, x, y, yaw, scale=None, keep=None, ignore=(),
                  force=False, z0=_GRASS_TOP):
            """The ONLY way anything enters a park: measure, reserve, place.

            *keep* overrides the measured footprint with a square of that side
            — planting reserves the ground its trunk and pit need rather than
            the crown that hangs over it. It is claimed at the placement yaw
            like every other footprint, so what is reserved is exactly what an
            overlap test measures.
            """
            nonlocal n_refused
            if not _in_rect(x, y, rect) or blocked(x, y):
                return None
            fp = fp_of(usd, cat, scale)
            box = (_box(x, y, yaw + lib.yaw_offset(usd), keep, keep) if keep
                   else _box(x, y, yaw + lib.yaw_offset(usd), fp["sx"], fp["sy"]))
            if force:
                occ.take(box, cat)
            elif not occ.reserve(box, cat, ignore):
                n_refused += 1
                return None
            add(usd, x, y, z0 + fp["base"], yaw, cat, scale)
            tally[cat] = tally.get(cat, 0) + 1
            return fp

        # ---- attractions first: a walk has to lead somewhere, and how far it
        # has to stop short of one is what the whole network is clipped to.
        kinds = ["fountain" if features else "plaza", "playground", "plaza"]
        rng.shuffle(kinds)
        n_lo, n_hi = _pair(lay.get("attractions"), (2.0, 3.0))
        n_att = max(1, min(4, rng.randint(int(n_lo), int(n_hi))))

        att = []
        for k in range(n_att):
            kind = kinds[k % len(kinds)]
            if kind == "fountain" and features:
                feat = features[rng.randrange(len(features))]
                parts = lib.normalize(feat.get("parts") or [])
                keep_r = max([float(feat.get("radius_m", 6.0))]
                             + [radius_of(u, "park_feature") for u in parts])
            elif kind == "playground" and play_usds and rng.random() < pg_chance:
                keep_r = pg_spacing + max(radius_of(u, "play_structure")
                                          for u in play_usds)
                parts = None
            else:
                kind, parts, keep_r = "plaza", None, plaza_r
            ring_r = keep_r + t_w / 2.0 + ring_gap
            spec = {"kind": kind, "parts": parts, "keep_r": keep_r,
                    "ring_r": ring_r, "clear_r": ring_r + clearing_m}
            # Room for the whole composition inside the park, and clear of the
            # other attractions' clearings.
            m = spec["clear_r"] + 4.0
            if 2.0 * m >= min(w, h):
                continue
            for _try in range(60):
                hx = rng.uniform(x0 + m, x1 - m)
                hy = rng.uniform(y0 + m, y1 - m)
                if any(math.hypot(hx - a["x"], hy - a["y"])
                       < a["clear_r"] + spec["clear_r"] + 4.0 for a in att):
                    continue
                if blocked(hx, hy):
                    continue
                spec["x"], spec["y"] = hx, hy
                att.append(spec)
                break

        rings = [(a["x"], a["y"], a["ring_r"]) for a in att]
        # Exclusion zones are keep-outs, and `place` drops anything inside one
        # BEFORE it honours `force` — so a walk that simply ran into one lost
        # its tiles and left a hole. MEASURED: gaps of 16.0, 9.0 and 3.5 m where
        # three walks crossed the 10 m spawn clearing at the origin. Give each
        # one that lands in the park a ring, exactly as an attraction gets one:
        # the walks then clip to its outside and a paved circle carries them
        # around, so the clearing reads as a feature instead of a break.
        excl_rings = []
        for ex in (exclusions or ()):
            ec = ex.get("center") if isinstance(ex, dict) else None
            if not ec:
                continue
            er = float(ex.get("radius", 0.0)) + t_w / 2.0 + ring_gap
            if er <= 0.0 or not _in_rect(ec[0], ec[1], rect):
                continue
            excl_rings.append((float(ec[0]), float(ec[1]), er))
        rings = rings + excl_rings

        # ---- the walk network, clipped to the outside of every ring
        gates = _gates(rect, rng, *_pair(lay.get("entrances"), (4.0, 6.0)))
        jitter = float(lay.get("path_curve_m", 5.0))
        drawn = []
        for g in gates:
            if att:
                a = min(att, key=lambda p: (p["x"] - g[0]) ** 2
                        + (p["y"] - g[1]) ** 2)
                drawn.append(_curved(g, (a["x"], a["y"]), jitter, rng))
        for a, b in zip(att, att[1:]):
            drawn.append(_curved((a["x"], a["y"]), (b["x"], b["y"]), jitter, rng))
        if rng.random() < float(lay.get("loop_chance", 0.8)) and min(w, h) >= \
                float(lay.get("loop_min_side_m", 45.0)):
            li = float(lay.get("loop_inset_m", 8.0))
            drawn.append(_irregular_loop((x0 + li, y0 + li, x1 - li, y1 - li),
                                         rng))
        paths = []
        for poly in drawn:
            paths.extend(_clip_outside(poly, rings, min_len=t_len * 1.5))
        # The rings are paving and circulation like any other walk, but their
        # furniture is COMPOSED — benches at an even pitch facing the water —
        # so they are held out of the walk-and-drop pass below, which would
        # otherwise drop a second row of seats on them facing outward.
        ring_paths = ([_ring(a["x"], a["y"], a["ring_r"]) for a in att]
                      + [_ring(cx, cy, r) for cx, cy, r in excl_rings])
        walked = _Field([pt for p in paths + ring_paths for pt in p],
                        max(8.0, path_clear + t_w))
        n_walks += len(paths) + len(ring_paths)
        walk_len += sum(_poly_len(p) for p in paths + ring_paths)

        # ---- path tiles. The Downtown_West walk tiles are authored at kerb
        # height, so the tile's own base has to be honoured or they float.
        # Paving is claimed unconditionally and before anything else: walks are
        # the one thing in a park that may not be moved aside, and consecutive
        # tiles are meant to overlap into a continuous ribbon.
        for path in paths + ring_paths:
            for tx, ty, tyaw in _walk_polyline(path, t_len * 0.8):
                place(tu, "trail", tx, ty, tyaw, scale=t_scale, force=True,
                      z0=_TRAIL_Z)

        # ---- railing along the park edge, broken at every entrance. Ahead of
        # the planting: a railing with tree-shaped holes in it is not a railing.
        if fence_usds:
            fu = fence_usds[0]
            mod = max(1.0, fp_of(fu, "fence")["sy"]) + fence_gap
            # The runs along y stop half a module short at both ends so the
            # corner is mitred by the run along x instead of the two crossing
            # through each other.
            cut = mod * 0.6
            for (ex0, ey0, ex1, ey1, yaw) in (
                    (x0, y0, x1, y0, 0.0), (x0, y1, x1, y1, 0.0),
                    (x0, y0 + cut, x0, y1 - cut, 90.0),
                    (x1, y0 + cut, x1, y1 - cut, 90.0)):
                run = math.hypot(ex1 - ex0, ey1 - ey0)
                n = int(run // mod)
                for k in range(n):
                    t = (k + 0.5) / max(1, n)
                    px_ = ex0 + (ex1 - ex0) * t
                    py_ = ey0 + (ey1 - ey0) * t
                    if min((px_ - g[0]) ** 2 + (py_ - g[1]) ** 2
                           for g in gates) < 36.0:
                        continue                      # leave the gate open
                    # Modules butt end to end, so a railing does not hold the
                    # occupancy pad against its own neighbours — with the pad
                    # applied every second module was refused and the railing
                    # came out as a dashed line.
                    place(fu, "fence", px_, py_, yaw, ignore=("fence",))

        # ---- what stands at the centre of each attraction
        for a in att:
            ax, ay = a["x"], a["y"]
            if a["kind"] == "fountain":
                # The stack shares one origin by construction — basin plus
                # three water discs — so it reserves once, as the keep-out.
                occ.take(_box(ax, ay, 0.0, 2.0 * a["keep_r"],
                              2.0 * a["keep_r"]), "park_feature")
                spin = rng.uniform(0.0, 360.0)
                for u in a["parts"]:
                    place(u, "park_feature", ax, ay, spin, force=True)
            elif a["kind"] == "playground":
                # A composed group wins over the scatter: its parts hold a
                # measured relationship to each other, so it is placed rigidly —
                # one group yaw, offsets rotated into it. Scattering them on a
                # circle at independent angles is what put the slide facing away
                # from the swings.
                if groups:
                    grp = groups[rng.randrange(len(groups))]
                    spin = rng.uniform(0.0, 360.0)
                    cs, sn = (math.cos(math.radians(spin)),
                              math.sin(math.radians(spin)))
                    for part in grp:
                        ox, oy = part["offset"]
                        place(part["usd"], "play_structure",
                              ax + cs * ox - sn * oy,
                              ay + sn * ox + cs * oy,
                              spin + part["yaw"], force=True)
                else:
                    n_st = rng.randint(int(pg_count[0]), int(pg_count[1]))
                    picks = list(play_usds)
                    rng.shuffle(picks)
                    base = rng.uniform(0.0, 360.0)
                    for i in range(n_st):
                        ang = base + i * 360.0 / n_st
                        r_ = math.radians(ang)
                        place(picks[i % len(picks)], "play_structure",
                              ax + math.cos(r_) * pg_spacing,
                              ay + math.sin(r_) * pg_spacing,
                              ang + 180.0 + rng.uniform(-15.0, 15.0))
            elif rock_usds:                      # seating circle round a group
                for i in range(rng.randint(2, 3)):
                    ang = rng.uniform(0.0, 2.0 * math.pi)
                    r_ = rng.uniform(0.0, max(0.5, a["keep_r"] - 2.0))
                    place(rng.choice(rock_usds), "rock",
                          ax + math.cos(ang) * r_, ay + math.sin(ang) * r_,
                          rng.uniform(0.0, 360.0))

            # Benches on the ring, facing the water. Travelling anticlockwise
            # the tangent is theta+90 and the outward normal is the right hand
            # side, which is the `side < 0` case of the along-path convention
            # below — so the yaw IS the tangent and the seat normal (yaw + 90)
            # points at the centre.
            r_b = a["ring_r"] + t_w / 2.0 + furn_off
            n_b = max(3, int(2.0 * math.pi * r_b / max(2.0, ring_bench_pitch)))
            phase = rng.uniform(0.0, 360.0)
            for i in range(n_b if bench_usds else 0):
                th = phase + i * 360.0 / n_b
                r_ = math.radians(th)
                place(rng.choice(bench_usds), "bench",
                      ax + math.cos(r_) * r_b, ay + math.sin(r_) * r_b,
                      th + 90.0)
            if trash_usds:
                for i in range(2):
                    th = math.radians(phase + 45.0 + i * 180.0)
                    place(rng.choice(trash_usds), "trash_can",
                          ax + math.cos(th) * r_b, ay + math.sin(th) * r_b,
                          math.degrees(th) + 180.0)
            if light_usds:
                for i in range(4):
                    th = math.radians(phase + 22.5 + i * 90.0)
                    place(rng.choice(light_usds), "streetlight",
                          ax + math.cos(th) * (r_b + 1.0),
                          ay + math.sin(th) * (r_b + 1.0),
                          math.degrees(th) + 180.0)

        # ---- the greensward: one protected open lawn, placed where neither
        # the walks nor the attractions are. Keeping a middle open is the
        # single most Olmsted move a park layout makes — but it is a clearing,
        # not a void, so it is a tenth of the park rather than a third.
        lawn_r = math.sqrt(max(1.0, area * lawn_frac) / math.pi)
        mrg = min(lawn_r, w / 2.0 - 6.0), min(lawn_r, h / 2.0 - 6.0)
        # Its own coarse field: `walked` resolves distance only to the walk
        # keep-out, and every candidate beyond that would score the same.
        far = _Field([pt for p in paths + ring_paths for pt in p], 40.0)
        best = None
        for _ in range(300):
            lx = rng.uniform(x0 + mrg[0], x1 - mrg[0])
            ly = rng.uniform(y0 + mrg[1], y1 - mrg[1])
            d = min([far.min_d2(lx, ly)]
                    + [(lx - a["x"]) ** 2 + (ly - a["y"]) ** 2 for a in att])
            if best is None or d > best[0]:
                best = (d, lx, ly)
        lawn = (best[1], best[2])

        def clear_of_paths(px, py, m):
            return walked.min_d2(px, py) > (m + t_w / 2.0) ** 2

        def open_ground(px, py, r=0.0):
            """Off the walks, out of the lawn, out of every tree clearing."""
            if not clear_of_paths(px, py, path_clear):
                return False
            if math.hypot(px - lawn[0], py - lawn[1]) < lawn_r + r:
                return False
            return not any(math.hypot(px - a["x"], py - a["y"])
                           < a["clear_r"] + r for a in att)

        # ---- seating and lighting along the walks, facing them
        for pool, cat in ((bench_usds, "bench"), (light_usds, "streetlight"),
                          (trash_usds, "trash_can")):
            if not pool:
                continue
            want = max(1.0, area / 1000.0 * den[cat])
            step = max(pitch[cat], sum(_poly_len(p) for p in paths) / want)
            ph = {"bench": 0.5, "streetlight": 0.25, "trash_can": 0.75}[cat]
            for i, (fx, fy, fyaw) in enumerate(_walk_paths(paths, step, ph)):
                # Runs of three on one side, then three on the other: a side
                # drawn per prop reads as a zig-zag, a run reads as laid out.
                # A refused slot gets the other side and then a slide along the
                # walk, as city_detail does at a kerb — most refusals are a
                # phase problem (a junction, the prop before it) rather than a
                # capacity one, and dropping them thins the walk unevenly.
                done = False
                for slide in (0.0, 2.0, -2.0):
                    for flip in (0, 1):
                        side = (1.0 if ((i // 3 + flip) % 2 == 0) else -1.0)
                        rad = math.radians(fyaw)
                        nx_, ny_ = -math.sin(rad) * side, math.cos(rad) * side
                        d = t_w / 2.0 + furn_off
                        px_ = fx + nx_ * d + math.cos(rad) * slide
                        py_ = fy + ny_ * d + math.sin(rad) * slide
                        # Along the path, not across it. Yawing to the path
                        # NORMAL turns a 3.3 m bench broadside to a 3 m walk,
                        # which is what put seats through the paving. The long
                        # axis follows the path and the half-turn is what points
                        # the seat back at it.
                        if cat == "bench":
                            yaw_ = fyaw if side < 0.0 else fyaw + 180.0
                        else:
                            yaw_ = math.degrees(math.atan2(-ny_, -nx_))
                        if place(rng.choice(pool), cat, px_, py_, yaw_):
                            done = True
                            break
                    if done:
                        break

        # ---- planting. One candidate per grid cell, jittered inside it: even
        # coverage by construction, and the jitter is what keeps it off a grid.
        # Species come from grove seeds rather than a per-tree lottery, so the
        # park reads as massed planting instead of an arboretum sample.
        groves = []
        for _ in range(max(1, int(area / max(200.0, grove_area)))):
            groves.append((rng.uniform(x0, x1), rng.uniform(y0, y1),
                           rng.choice(tree_usds) if tree_usds else None,
                           rng.choice(plant_usds) if plant_usds else None))

        def grove_at(px, py):
            return min(groves, key=lambda g: (px - g[0]) ** 2 + (py - g[1]) ** 2)

        planted = []
        if tree_usds:
            cell = math.sqrt(100.0 / max(0.05, tree_den))
            for cx_, cy_, cw, ch in _cells(rect, cell, rng):
                for _try in range(4):
                    px = cx_ + rng.uniform(0.06, 0.94) * cw
                    py = cy_ + rng.uniform(0.06, 0.94) * ch
                    u = grove_at(px, py)[2]
                    crown = max(fp_of(u, "tree")["sx"], fp_of(u, "tree")["sy"])
                    if not open_ground(px, py, crown * 0.25):
                        continue
                    # The reservation is the trunk and its pit, floored at the
                    # separation a young tree needs; crowns interlock above it.
                    if place(u, "tree", px, py, rng.uniform(0.0, 360.0),
                             keep=max(tree_sep, crown * canopy_frac)):
                        planted.append((px, py))
                        break
        if plant_usds:
            # The greensward is mown grass and wildflower, not bare ground: the
            # small end of the understorey pool is exactly that, and a meadow
            # with nothing on it is the single largest hole in a park's cover.
            # Bushes stay out — they would read as a thicket, not a lawn.
            meadow = [u for u in plant_usds
                      if max(fp_of(u, "plant")["sx"],
                             fp_of(u, "plant")["sy"]) <= lawn_plant_m]
            cell = math.sqrt(100.0 / max(0.05, plant_den))
            for cx_, cy_, cw, ch in _cells(rect, cell, rng):
                for _try in range(4):
                    px = cx_ + rng.uniform(0.06, 0.94) * cw
                    py = cy_ + rng.uniform(0.06, 0.94) * ch
                    # Understorey is allowed up to the edge of a clearing: it
                    # is knee-high and does not hide the thing in the middle.
                    if not clear_of_paths(px, py, path_clear):
                        continue
                    on_lawn = math.hypot(px - lawn[0], py - lawn[1]) < lawn_r
                    if on_lawn and not meadow:
                        continue
                    if any(math.hypot(px - a["x"], py - a["y"]) < a["ring_r"]
                           + 2.0 for a in att):
                        continue
                    u = rng.choice(meadow) if on_lawn else grove_at(px, py)[3]
                    fp = fp_of(u, "plant")
                    if place(u, "plant", px, py, rng.uniform(0.0, 360.0),
                             keep=max(plant_sep, fp["sx"], fp["sy"])):
                        break
        if stump_usds and planted and rng.random() < 0.4:
            p = planted[rng.randrange(len(planted))]
            sx_, sy_ = p[0] + rng.uniform(-4, 4), p[1] + rng.uniform(-4, 4)
            # A stump is a tree for clearing purposes too, or the one thing
            # exempt from the rule ends up standing in the fountain's clearing.
            if open_ground(sx_, sy_):
                place(rng.choice(stump_usds), "tree", sx_, sy_,
                      rng.uniform(0.0, 360.0))

        rocks: list = []
        rk_lo, rk_hi = _pair(cfg.get("rocks_per_park"), (3.0, 8.0))
        for _ in range(rng.randint(int(rk_lo), int(rk_hi)) * 3):
            if not (rock_usds and planted) or len(rocks) >= rk_hi:
                break
            p = planted[rng.randrange(len(planted))]
            px_, py_ = p[0] + rng.uniform(-6, 6), p[1] + rng.uniform(-6, 6)
            if (min([(px_ - q[0]) ** 2 + (py_ - q[1]) ** 2
                     for q in rocks] or [1e9]) < rock_sep ** 2
                    or not clear_of_paths(px_, py_, 1.0)):
                continue
            if place(rng.choice(rock_usds), "rock", px_, py_,
                     rng.uniform(0.0, 360.0)):
                rocks.append((px_, py_))

        # ---- people, on the walks and out on the greensward. They stand ON
        # the paving, so paving is the one reservation they ignore.
        if human_usds:
            hstep = float(config.get("humans", {}).get("trail_spacing_m", 30.0))
            for hx, hy, hyaw in _walk_paths(paths + ring_paths,
                                            hstep or 1e9, 0.15):
                place(rng.choice(human_usds), "human", hx, hy, hyaw,
                      ignore=_FLAT)
            for _ in range(rng.randint(3, 7)):
                a_ = rng.uniform(0.0, 2.0 * math.pi)
                rr = lawn_r * math.sqrt(rng.random())
                place(rng.choice(human_usds), "human",
                      lawn[0] + math.cos(a_) * rr, lawn[1] + math.sin(a_) * rr,
                      rng.uniform(0.0, 360.0), ignore=_FLAT)

    detail = "  ".join(f"{k}={v}" for k, v in sorted(tally.items()))
    print(f"[parks] built {len(parks)} park(s): cleared {cleared}, "
          f"{n_walks} walks / {walk_len:,.0f} m, "
          f"{n_refused} refused on footprint overlap\n"
          f"[parks]   {detail}")
    return len(parks)
