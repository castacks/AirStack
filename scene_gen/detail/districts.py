"""
districts.py — zoning: what gets built where, and how tall.

Additive. `build_city` is never modified; its placement list is rewritten.

Exports
-------
* `assign()`        — radial ring lookup, used by `city_detail` to thin
                      street furniture toward the edge.
* `zone_field()`    — ``zone_at(x, y) -> typology dict``. The zoning map.
                      `city_layout` may call this to size blocks per zone
                      (see BLOCK SIZE, below).
* `remap_buildings()` — the pipeline hook: rezones every block, then runs
                      infill and the park pass.
* `free_rects()`, `park_blocks()`, `block_inset()` — helpers `parks.py`
                      shares.

WHY POST-PROCESS RATHER THAN CONFIGURE
--------------------------------------
The building pools and the packing that fits them into lots both live inside
`build_city`, and this work is under a no-edits constraint. Packing has already
solved the geometry by the time it returns, so its output can be rewritten:
footprints are known, blocks are known, and a placement is just a dict.

ZONING, NOT A HEIGHT GRADIENT
-----------------------------
A radial height gradient alone produces a city where every block is the same
KIND of place and only the storey count changes. Real cities zone by *type* —
a district of party-wall row houses, a district of mid-rise apartment blocks, a
downtown of towers — and the boundaries are patchy rather than concentric.

So a **typology** (`districts.typologies.*`) owns a building pool, a height
profile and a preferred block size, and the radial rings only supply the
*mixture* of typologies likely at that distance from the centre. The zoning map
itself is a jittered-grid Voronoi over the region: contiguous multi-block
patches, ragged boundaries, and a patch of one type can land inside a ring
dominated by another. `zoning.bleed` additionally re-rolls a fraction of blocks
straight from the ring mix, which is what puts a mid-rise on a row-house street
the way a real corner apartment building sits in a terrace district.

MORPHOLOGY
----------
Typologies differ in how their blocks are built, not only in what fills them:

* `terrace` — party-wall rows along the block's long faces, laid END TO END
  WITH NO GAP, full face length, back yards/alley left in the middle. This is
  the whole point of a row house: one dropped into a gap on its own reads as a
  detached house with a missing neighbour, which is exactly wrong. A terrace
  asset is therefore never in `buildings.intact` (where `build_city` would pick
  it uniformly and scatter it) — it lives in its own pool and is only ever
  placed by this pass, in a run.
* `pack` — guillotine packing from the typology's pool, as downtown does.

BLOCK SIZE
----------
Measured, the library's footprints span 15.6 x 24.3 m to 91.1 x 96.1 m. Six of
the ten stock buildings are 50-81 m wide, against a ~46 m usable interior on
the blocks the subdivider was producing — so those blocks could not be built on
at all and stayed bare pavement. Block size has to follow the typology: a
row-house block wants ~48 m (two 21 m terraces back to back plus an alley), a
tower block wants ~100 m.

The subdivider lives in `city_layout.py`, which this work may not edit, so
`zone_field()` is exposed for it to consult. Until that lands, every typology
choice is validated against the block it actually got and downgraded to one
whose pool fits — the scene degrades to "smaller buildings than intended"
rather than to "empty block".
"""

import math
import random

from scene_generator import _in_exclusion, _normalize_usd_list, _stage

# Footprint slack when testing whether a building still fits its lot. Small and
# negative-biased on purpose: fractionally larger risks clipping a neighbour.
_FIT_TOL_M = 0.05

# Geometry below an asset's own origin (areaways, basement window wells, the
# sunken front court of a brownstone) is authored to sit BELOW grade. The
# generator's `z = fp["base"]` rule lifts the building until that clears the
# ground, which floats the stoop. Anything shallower than this is sunk back to
# grade; deeper, and the origin probably isn't the ground floor.
_BELOW_GRADE_MAX_M = 1.2


def assign(config: dict, layout: dict):
    """Return ``(district_at, districts)``.

    ``district_at(x, y)`` gives the ring dict for a world position, or ``None``
    when zoning is off. Rings are concentric fractions of the region's
    half-diagonal — they no longer decide what gets built, only the mixture of
    typologies and how much street furniture `city_detail` thins out.
    """
    cfg = _stage(config, "layout").get("districts") or {}
    rings = cfg.get("rings") or []
    if not cfg.get("enabled") or not rings:
        return (lambda x, y: None), []

    x0, y0, x1, y1 = layout.get("region", (0.0, 0.0, 0.0, 0.0))
    w, h = abs(x1 - x0), abs(y1 - y0)
    if min(w, h) < float(cfg.get("min_region_m", 0.0)):
        print(f"[districts] region {w:.0f}x{h:.0f} m is below "
              f"min_region_m={cfg.get('min_region_m')} — single district")
        return (lambda x, y: None), []

    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    half_diag = math.hypot(w, h) / 2.0 or 1.0
    ordered = sorted(rings, key=lambda r: float(r.get("to_frac", 1.0)))

    def district_at(x, y):
        f = math.hypot(x - cx, y - cy) / half_diag
        for r in ordered:
            if f <= float(r.get("to_frac", 1.0)):
                return r
        return ordered[-1]

    names = ", ".join(str(r.get("name", "?")) for r in ordered)
    print(f"[districts] {len(ordered)} rings over {w:.0f}x{h:.0f} m: {names}")
    return district_at, ordered


def block_lookup(district_at):
    """Adapt `district_at` to the block-rect callback `city_detail` expects."""
    def district_of(block):
        x0, y0, x1, y1 = block
        return district_at((x0 + x1) / 2.0, (y0 + y1) / 2.0)
    return district_of


# ---------------------------------------------------------------------------
# the zoning map
# ---------------------------------------------------------------------------

def _region_of(config: dict, region=None):
    if region is not None:
        return tuple(region)
    r = (config.get("layout") or {}).get("region_m", [400.0, 400.0])
    w, h = float(r[0]), float(r[1])
    return (-w / 2.0, -h / 2.0, w / 2.0, h / 2.0)


def _pick(mix: dict, rnd: random.Random):
    total = sum(max(0.0, float(v)) for v in mix.values()) or 1.0
    r = rnd.random() * total
    for k, v in mix.items():
        r -= max(0.0, float(v))
        if r <= 0.0:
            return k
    return next(iter(mix))


class _AreaMap:
    """Land use as a few irregular contiguous areas grown from nuclei.

    Concentric rings are Burgess (1925) — the textbook caricature. Real cities
    look closer to Harris & Ullman's multiple-nuclei model (1945), and that is
    what OSM landuse polygons show: a handful of irregular contiguous patches
    around one dominant CBD, not bands. So typology is decided by growing
    regions outward from a small number of nuclei on a coarse lattice, and the
    radial rings survive only as the PRIOR on where those nuclei land — the
    downtown still tends to sit centrally without the city reading as banded.

    Growth is Dijkstra from every nucleus at once with jittered edge costs,
    which is what makes a boundary wander instead of coming out as the straight
    bisector a plain Voronoi would give.
    """

    def __init__(self, config: dict, region=None):
        cfg = _stage(config, "layout").get("districts") or {}
        zcfg = cfg.get("zoning") or {}
        self.typologies = cfg.get("typologies") or {}
        self.rings = sorted(cfg.get("rings") or [],
                            key=lambda r: float(r.get("to_frac", 1.0)))
        self.bleed = float(zcfg.get("bleed", 0.12))
        self.blend_m = float(zcfg.get("blend_m", 150.0))
        cell = max(30.0, float(zcfg.get("cell_m", 120.0)))

        x0, y0, x1, y1 = _region_of(config, region)
        self.x0, self.y0, self.cell = x0, y0, cell
        self.cx, self.cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        self.half_diag = math.hypot(x1 - x0, y1 - y0) / 2.0 or 1.0
        self.nx = max(1, int(math.ceil((x1 - x0) / cell)))
        self.ny = max(1, int(math.ceil((y1 - y0) / cell)))

        # Own RNG: the map must not shift because some other pass happened to
        # draw a different number of randoms before it.
        rnd = random.Random(int(config.get("seed", 0)) * 7919 + 104729)
        self.grid = self._grow(zcfg, rnd)
        self._ladder(zcfg)

    # -- lattice helpers
    def _ij(self, x, y):
        return (min(self.nx - 1, max(0, int((x - self.x0) / self.cell))),
                min(self.ny - 1, max(0, int((y - self.y0) / self.cell))))

    def _xy(self, i, j):
        return (self.x0 + (i + 0.5) * self.cell,
                self.y0 + (j + 0.5) * self.cell)

    def ring_of(self, x, y):
        f = math.hypot(x - self.cx, y - self.cy) / self.half_diag
        for r in self.rings:
            if f <= float(r.get("to_frac", 1.0)):
                return r
        return self.rings[-1] if self.rings else {}

    def _grow(self, zcfg, rnd):
        """Place nuclei, then flood the lattice outward from all of them."""
        import heapq

        # One nucleus per `nucleus_per_m2` of region, split between typologies
        # by the ring mix at each candidate site — so a tower nucleus is much
        # likelier to be drawn near the centre and a row-house one at the edge.
        area = self.nx * self.ny * self.cell * self.cell
        n_nuc = max(2, int(round(area / max(1.0, float(
            zcfg.get("area_m2_per_nucleus", 120_000.0))))))
        heap, grid = [], {}
        for k in range(n_nuc):
            for _try in range(24):
                # Multiple-nuclei still has ONE dominant CBD. Left to chance the
                # innermost ring is a small target and most seeds land outside
                # it, so a city comes out with no downtown at all — nucleus 0 is
                # therefore forced into the core ring.
                if k == 0 and zcfg.get("downtown", True):
                    i = int(self.nx * rnd.uniform(0.38, 0.62))
                    j = int(self.ny * rnd.uniform(0.38, 0.62))
                else:
                    i = rnd.randrange(self.nx)
                    j = rnd.randrange(self.ny)
                i = min(self.nx - 1, max(0, i))
                j = min(self.ny - 1, max(0, j))
                if (i, j) in grid:
                    continue
                x, y = self._xy(i, j)
                mix = (self.ring_of(x, y) or {}).get("mix") or {}
                name = _pick(mix, rnd) if mix else None
                if name is None:
                    continue
                grid[(i, j)] = name
                heapq.heappush(heap, (0.0, k, i, j, name))
                break

        while heap:
            d, k, i, j, name = heapq.heappop(heap)
            if grid.get((i, j)) not in (None, name) and d > 0.0:
                continue
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ni, nj = i + di, j + dj
                if not (0 <= ni < self.nx and 0 <= nj < self.ny):
                    continue
                if (ni, nj) in grid:
                    continue
                grid[(ni, nj)] = name
                heapq.heappush(heap, (d + rnd.uniform(0.6, 1.6), k, ni, nj,
                                      name))
        return grid

    def _ladder(self, zcfg):
        """Forbid a rank jump greater than one between neighbouring areas.

        Transitional zoning, which is how real codes stop a tower from abutting
        a two-storey terrace: where the jump is too big the intermediate
        typology is inserted instead. Also what keeps block sizes from having
        to blend across an impossible gap.
        """
        if not zcfg.get("adjacency_ladder", True):
            return
        rank = {n: float(t.get("rank", 0.0))
                for n, t in self.typologies.items()}
        by_rank = sorted(rank, key=lambda n: rank[n])
        for _sweep in range(3):
            changed = False
            for (i, j), name in list(self.grid.items()):
                for di, dj in ((1, 0), (0, 1)):
                    other = self.grid.get((i + di, j + dj))
                    if other is None or other == name:
                        continue
                    a, b = rank.get(name, 0.0), rank.get(other, 0.0)
                    if abs(a - b) <= 1.0 + 1e-9:
                        continue
                    mid = min(by_rank,
                              key=lambda n: abs(rank[n] - (a + b) / 2.0))
                    self.grid[(i + di, j + dj) if b > a else (i, j)] = mid
                    changed = True
            if not changed:
                break

    # -- lookups
    def name_at(self, x, y):
        return self.grid.get(self._ij(x, y))

    def bleed_name(self, x, y, rng):
        """A typology drawn straight from the ring mix, ignoring the area.

        Clean zoning is the giveaway; real cities leak. This is what puts a
        mid-rise apartment block in the middle of a terrace street.
        """
        mix = (self.ring_of(x, y) or {}).get("mix") or {}
        return _pick(mix, rng) if mix else None

    def targets_at(self, x, y):
        """Blended block-size targets ``(short_lo, short_hi, long_lo, long_hi)``.

        Sampled over a small cross rather than at the point alone, so block size
        GRADES across an area boundary over `blend_m` instead of snapping from
        a 40 m residential block to a 110 m downtown one in a single street.
        """
        # BLEND ONLY WITHIN A TYPOLOGY. Averaging a cell with neighbours of a
        # DIFFERENT typology does not grade the seam, it destroys the target:
        # measured, row-house blocks configured at 53-58 m came out 77-92 m
        # because most row-house cells are boundary cells and every one was
        # being pulled toward the 80-115 m mid-rise and 70-110 m tower figures
        # around it. A terrace pair needs 52.7 m, so those blocks could only
        # ever be 40% built. A hard seam in block size is a far smaller problem
        # than a typology that cannot build what it is for.
        here_name = self.name_at(x, y)
        acc, n = [0.0, 0.0, 0.0, 0.0], 0
        r = self.blend_m
        for dx, dy in ((0.0, 0.0), (r, 0.0), (-r, 0.0), (0.0, r), (0.0, -r)):
            name = self.name_at(x + dx, y + dy)
            if name != here_name:
                continue
            t = self.typologies.get(name or "") or {}
            s = t.get("block_short_m")
            lg = t.get("block_long_m")
            if not s or not lg:
                continue
            acc[0] += float(s[0])
            acc[1] += float(s[1])
            acc[2] += float(lg[0])
            acc[3] += float(lg[1])
            n += 1
        if not n:
            return None
        out = tuple(v / n for v in acc)
        # A typology may run its blocks ACROSS the city's grain. `city_layout`
        # maps (short, long) onto (x, y) by one global `long_axis`, so every
        # block in the scene is elongated the same way — which is most of why a
        # terrace district reads as more of the same. Swapping the pair here
        # turns that typology's blocks a quarter turn, and because the terrace
        # rows follow the block's own long face they turn with it.
        here = self.typologies.get(self.name_at(x, y) or "") or {}
        if _grain_flip(here.get("cross_grain"), x, y):
            out = (out[2], out[3], out[0], out[1])
        return out

    def __call__(self, x, y):
        """Typology dict at a position, with its ``name`` filled in."""
        name = self.name_at(x, y)
        if name is None:
            return None
        t = dict(self.typologies.get(name) or {})
        t["name"] = name
        return t


def _grain_flip(cross_grain, x: float, y: float) -> bool:
    """Whether the block grain turns a quarter at this position.

    ``true`` turns the whole typology, which makes every terrace block in the
    scene run the same way — the same monotony the flag exists to break, just
    rotated. A NUMBER instead turns it by section: the plane is cut into squares
    of that many metres and alternate ones flip, so one row-house district runs
    with the city grain and the next runs across it.

    Sections are coarse on purpose. The grain has to hold across a whole strip
    or its blocks come out inconsistent, so the size should comfortably exceed
    one district; a section boundary falling inside a strip is the one case to
    watch. Position, not `rng` — the subdivider samples this many times per
    cell and a random answer would differ between samples of the same block.
    """
    if not cross_grain:
        return False
    if cross_grain is True:
        return True
    try:
        step = float(cross_grain)
    except (TypeError, ValueError):
        return True
    if step <= 0.0:
        return True
    ix = int(math.floor(x / step))
    iy = int(math.floor(y / step))
    return ((ix * 73856093) ^ (iy * 19349663)) % 2 == 0


def zone_field(config: dict, region=None):
    """``zone_at(x, y) -> typology dict | None``.

    Public so `city_layout` can size each block to the area it falls in; see
    BLOCK SIZE in the module docstring. Returns a None-yielding callable when
    typology zoning is not configured, so callers need no feature test.
    """
    cfg = _stage(config, "layout").get("districts") or {}
    if not (cfg.get("enabled") and cfg.get("typologies") and cfg.get("rings")):
        return lambda x, y: None
    return _AreaMap(config, region)


# ---------------------------------------------------------------------------
# building library
# ---------------------------------------------------------------------------

def _pool_entries(config: dict, resolver, key: str):
    """``[(usd, scale, axis_up, footprint)]`` for one ``usds.buildings`` pool."""
    usds = config.get("usds") or {}
    bld = usds.get("buildings") or usds.get("houses") or {}
    raw = bld.get(key) or []
    default_scale = float(config.get("asset_scale", 1.0))
    asset_root = str(config.get("asset_root", "") or "")
    paths, sc_ovr, au_ovr, _yaw, _tags = _normalize_usd_list(
        raw, default_scale, asset_root)

    out, seen = [], set()
    for p in paths:
        if p in seen:
            continue
        seen.add(p)
        sc = sc_ovr.get(p, default_scale)
        au = au_ovr.get(p, "Z")
        out.append((p, sc, au, resolver.get(p, "house", scale=sc, axis_up=au)))
    return out


def _pools_for(config, resolver, names, cache):
    """Merge the named ``usds.buildings`` pools, tallest first."""
    out, seen = [], set()
    for n in names or ():
        if n not in cache:
            cache[n] = _pool_entries(config, resolver, n)
        for e in cache[n]:
            if e[0] not in seen:
                seen.add(e[0])
                out.append(e)
    out.sort(key=lambda e: -e[3]["sz"])
    return out


def _place_z(fp):
    """Ground height for a footprint — see `_BELOW_GRADE_MAX_M`."""
    return 0.0 if 0.0 < fp["base"] <= _BELOW_GRADE_MAX_M else fp["base"]


def _new_placement(entry, x, y, yaw, category="house"):
    usd, sc, au, fp = entry
    return {"usd": usd, "x_m": x, "y_m": y, "z_m": _place_z(fp),
            "yaw_deg": yaw, "roll_deg": 90.0 if au == "Y" else 0.0,
            "pitch_deg": 0.0, "scale": sc, "category": category,
            "axis_up": au}


# ---------------------------------------------------------------------------
# height field
# ---------------------------------------------------------------------------

class _Skyline:
    """Samples a building height, per typology.

    Two effects compose. The typology supplies a log-normal target —
    right-skewed, so most draws are low and a few are tall. `neighbour_weight`
    then pulls the draw toward the mean of nearby buildings already decided, so
    height varies gradually along a street instead of independently per lot.
    """

    def __init__(self, cfg: dict, rng):
        self.rng = rng
        self.w = float(cfg.get("neighbour_weight", 0.5))
        self.r2 = float(cfg.get("neighbour_radius_m", 45.0)) ** 2
        # Wider than the typology's own sigma on purpose: with a coarse library
        # an exact match is usually unavailable, and always taking the nearest
        # re-quantises the distribution onto a handful of heights.
        self.pick_sigma = float(cfg.get("pick_sigma", 0.30))
        # 0 restores pure height matching; 1 makes an asset's odds inversely
        # proportional to how many times it is already standing in the city.
        self.repeat = float(cfg.get("repeat_penalty", 0.6))
        self.used: dict = {}            # usd -> times placed
        self.placed: list = []          # (x, y, height_m)

    def target(self, typ: dict, x: float, y: float) -> float:
        med = float((typ or {}).get("height_median_m", 0.0))
        if med <= 0.0:
            return 0.0
        t = med * math.exp(self.rng.gauss(0.0, float(typ.get("height_sigma", 0.4))))
        cap = typ.get("height_max_m")
        if cap:
            t = min(t, float(cap))
        if self.w > 0.0:
            near = [h for (px, py, h) in self.placed
                    if (px - x) ** 2 + (py - y) ** 2 <= self.r2]
            if near:
                mean = sum(near) / len(near)
                t = math.exp((1.0 - self.w) * math.log(max(t, 0.5))
                             + self.w * math.log(max(mean, 0.5)))
        return t

    def choose(self, candidates, target: float):
        if not candidates:
            return None
        if target <= 0.0:
            return self._pick(candidates, [1.0] * len(candidates))
        lt = math.log(target)
        w = [math.exp(-((math.log(max(e[3]["sz"], 0.5)) - lt) ** 2)
                      / (2.0 * self.pick_sigma ** 2)) for e in candidates]
        if sum(w) <= 1e-12:
            return min(candidates,
                       key=lambda e: abs(math.log(max(e[3]["sz"], 0.5)) - lt))
        return self._pick(candidates, w)

    def _pick(self, candidates, w):
        """Weighted draw, damped by how often each asset has already been used.

        Height match alone concentrates the city on a handful of models: an
        asset is eligible whenever it FITS, so small footprints win far more
        slots than large ones and one model ends up a fifth of every building in
        the scene. Dividing by (1 + uses)^repeat_penalty leaves the height
        match in charge of WHICH band gets picked while spreading the draw
        across the models inside that band.
        """
        if self.repeat > 0.0:
            w = [wi / ((1.0 + self.used.get(e[0], 0)) ** self.repeat)
                 for e, wi in zip(candidates, w)]
        total = sum(w)
        if total <= 1e-12:
            return candidates[self.rng.randrange(len(candidates))]
        r = self.rng.random() * total
        for e, wi in zip(candidates, w):
            r -= wi
            if r <= 0.0:
                self.used[e[0]] = self.used.get(e[0], 0) + 1
                return e
        self.used[candidates[-1][0]] = self.used.get(candidates[-1][0], 0) + 1
        return candidates[-1]

    def record(self, x, y, height):
        self.placed.append((x, y, height))


# ---------------------------------------------------------------------------
# free-space decomposition
# ---------------------------------------------------------------------------

def free_rects(rect, obstacles, min_side: float = 1.0):
    """Decompose *rect* minus *obstacles* into axis-aligned free rectangles.

    Sweeps the obstacle x-edges into columns, takes each column's free
    y-intervals, then merges columns sharing an interval — so a strip of open
    ground comes back as one wide rectangle rather than a row of slivers.
    """
    x0, y0, x1, y1 = rect
    if x1 - x0 < min_side or y1 - y0 < min_side:
        return []
    obs = [o for o in obstacles
           if o[2] > x0 and o[0] < x1 and o[3] > y0 and o[1] < y1]

    xs = sorted({x0, x1} | {min(max(v, x0), x1)
                            for o in obs for v in (o[0], o[2])})
    cells: list = []
    for cx0, cx1 in zip(xs, xs[1:]):
        if cx1 - cx0 < 1e-6:
            continue
        blocked = sorted((o[1], o[3]) for o in obs
                         if o[0] < cx1 - 1e-6 and o[2] > cx0 + 1e-6)
        cursor, spans = y0, []
        for by0, by1 in blocked:
            if by0 > cursor:
                spans.append((cursor, min(by0, y1)))
            cursor = max(cursor, by1)
            if cursor >= y1:
                break
        if cursor < y1:
            spans.append((cursor, y1))
        cells += [[cx0, a, cx1, b] for a, b in spans if b - a > 1e-6]

    merged: list = []
    for c in cells:
        for m in merged:
            if (abs(m[2] - c[0]) < 1e-6 and abs(m[1] - c[1]) < 1e-6
                    and abs(m[3] - c[3]) < 1e-6):
                m[2] = c[2]
                break
        else:
            merged.append(c)
    return [tuple(m) for m in merged
            if m[2] - m[0] >= min_side and m[3] - m[1] >= min_side]


def _rect_of(p, resolver, category=None, margin=0.0):
    cat = category or p.get("category")
    fp = resolver.get(p["usd"], cat, scale=p.get("scale"),
                      axis_up=p.get("axis_up", "Z"))
    sx, sy = fp["sx"], fp["sy"]
    if abs((float(p.get("yaw_deg", 0.0)) % 180.0) - 90.0) < 45.0:
        sx, sy = sy, sx
    x, y = float(p["x_m"]), float(p["y_m"])
    return (x - sx / 2.0 - margin, y - sy / 2.0 - margin,
            x + sx / 2.0 + margin, y + sy / 2.0 + margin)


def _placement_rects(placements, resolver, categories, margin: float = 0.0):
    return [_rect_of(p, resolver, margin=margin) for p in placements
            if p.get("category") in categories]


def park_blocks(layout: dict, placements: list):
    """The blocks that are parks.

    Preferred source is `city_layout.PARK_RESERVES` — superblocks the
    subdivider carved out whole, with streets on their sides and no road
    through them. Falling back to trail-tile detection covers a run where the
    subdivider is the built-in one and `build_city` chose the parks itself.

    "Has no buildings" is deliberately NOT the test: a block where nothing in
    the library fit is also empty, and that block needs building on, not a
    footpath through it.
    """
    blocks = layout.get("blocks", [])
    try:
        from layout import city_layout
        reserved = [b for b in city_layout.PARK_RESERVES if b in blocks]
    except ImportError:
        reserved = []
    if reserved:
        return reserved
    trails = [(float(p["x_m"]), float(p["y_m"])) for p in placements
              if p.get("category") == "trail"]
    return [b for b in blocks
            if any(b[0] <= x <= b[2] and b[1] <= y <= b[3]
                   for x, y in trails)]


def block_inset(config: dict, resolver):
    """How far `build_city` holds block content off the block edge: a verge
    plus a sidewalk one tile wide. Recomputed here because the generator keeps
    the inset local to its packing loop."""
    tiles = (config.get("usds") or {}).get("tiles") or {}
    raw = tiles.get("sidewalk") or tiles.get("brick") or []
    default = float(config.get("asset_scale", 1.0))
    paths, sc_ovr, _au, _yaw, _tags = _normalize_usd_list(
        raw, default, str(config.get("asset_root", "") or ""))
    sw = 0.0
    if paths:
        fp = resolver.get(paths[0], "sidewalk", scale=sc_ovr.get(paths[0], default))
        sw = max(fp["sx"], fp["sy"])
    return float(_stage(config, "layout").get("frontage", {}).get("verge_m", 0.0)) + sw


# ---------------------------------------------------------------------------
# morphology: terrace
# ---------------------------------------------------------------------------

def _tile_run(length: float, pool, rng, jitter=0.35):
    """Choose row assets whose lengths sum as close to *length* as possible.

    Longest-first with a random pick among the top few, so consecutive block
    faces don't all come out as the same sequence. *pool* entries are measured
    with their long axis on y.
    """
    by_len = sorted(pool, key=lambda e: -e[3]["sy"])
    if not by_len:
        return []
    shortest = by_len[-1][3]["sy"]
    out, rem = [], length
    while rem >= shortest - 1e-6:
        fits = [e for e in by_len if e[3]["sy"] <= rem + 1e-6]
        if not fits:
            break
        k = 1 if len(fits) == 1 or rng.random() > jitter else min(len(fits), 2)
        e = fits[rng.randrange(k)]
        out.append(e)
        rem -= e[3]["sy"]
    return out


def _perimeter_rects(rect, depth: float, min_side: float):
    """The block's frontage BAND, as up to four rects, centre left out.

    Guillotine-packing the whole block interior lets a building sit anywhere in
    it, so a block often ends up built at the back with a deep apron of bare
    paving at the street. From the pavement that apron reads as an enormously
    wide sidewalk — it is not sidewalk at all, it is unbuilt block interior.

    A dense urban block is built to its edges with the slack in the MIDDLE,
    which is what this produces: pack the band, leave the courtyard. Returns the
    whole rect when it is too shallow to have a distinct middle.
    """
    x0, y0, x1, y1 = rect
    w, h = x1 - x0, y1 - y0
    if depth <= 0.0 or w <= 2.0 * depth + min_side or h <= 2.0 * depth + min_side:
        return [rect]
    return [(x0, y0, x1, y0 + depth),               # south frontage
            (x0, y1 - depth, x1, y1),               # north
            (x0, y0 + depth, x0 + depth, y1 - depth),   # west, between them
            (x1 - depth, y0 + depth, x1, y1 - depth)]   # east


def _split_along(blk, spans, axis):
    """The block cut into the pieces BETWEEN *spans*, spans left as gaps."""
    x0, y0, x1, y1 = blk
    lo, hi = (y0, y1) if axis == "x" else (x0, x1)
    cuts = sorted(spans)
    out, edge = [], lo
    for a, b in cuts + [(hi, hi)]:
        if a - edge > 1.0:
            out.append((x0, edge, x1, a) if axis == "x"
                       else (edge, y0, a, y1))
        edge = b
    return out


def _add_internal_street(layout, rect, span, axis, road_w, typ):
    """Register the terrace superblock's own street as a real corridor.

    `apply_ground_planes` lays asphalt and lane markings from
    ``layout["road_corridors"]``, and it runs AFTER this pass, so appending
    here is enough to have the street actually built rather than drawn as a
    gap in the paving. It is a residential street by construction: two lanes,
    parking both sides, which is what a row-house street is.
    """
    lo, hi = span
    x0, y0, x1, y1 = rect
    park_w = float(typ.get("street_park_w_m", 2.4))
    car_lo, car_hi = lo + park_w, hi - park_w
    c = {"n_lanes": 2, "carriage": [car_lo, car_hi], "park_w": park_w,
         "parking": "both", "zone": "edge", "road_class": "local",
         "internal": True}
    if axis == "x":                       # street runs along X, spans in Y
        c.update({"x0": x0, "x1": x1, "y0": lo, "y1": hi, "dir": "ew"})
    else:
        c.update({"x0": lo, "x1": hi, "y0": y0, "y1": y1, "dir": "ns"})
    layout.setdefault("road_corridors", []).append(c)


def _terrace_quad(rect, depth: float, alley_m: float, road_w: float):
    """Terrace rows on their OWN finer grid, or ``None``.

        row | alley | row  ||street||  row | alley | row  ||street||  ...

    As many back-to-back pairs as the block will take, each pair separated from
    the next by a street of its own. A terrace pair on a deep block leaves its
    back row fronting nothing, and filling the leftover with unrelated buildings
    is what makes brownstones look like they are sharing someone else's block.
    Real row-house districts answer this by being cut FINER than the rest of the
    city — smaller blocks, more streets — so the superblock brings its own.

    Returns ``(strips, [road_span, ...], axis)``. Each strip carries its own
    facing, so every pair fronts outward to whatever bounds it.
    """
    x0, y0, x1, y1 = rect
    w, h = x1 - x0, y1 - y0
    along_x = w >= h
    short = h if along_x else w
    pair = 2.0 * depth + alley_m
    # Most pairs that fit, ending on a pair rather than a street.
    n = int((short + road_w) // (pair + road_w))
    if n < 2:
        return None                       # a single pair is not a superblock
    need = n * pair + (n - 1) * road_w
    base = (y0 if along_x else x0) + (short - need) / 2.0

    strips, roads = [], []
    for i in range(n):
        p0 = base + i * (pair + road_w)
        outer, inner = p0, p0 + depth + alley_m
        if along_x:
            strips.append(((x0, outer, x1, outer + depth), "x", True))
            strips.append(((x0, inner, x1, inner + depth), "x", False))
        else:
            strips.append(((outer, y0, outer + depth, y1), "y", True))
            strips.append(((inner, y0, inner + depth, y1), "y", False))
        if i < n - 1:
            r0 = p0 + pair
            roads.append((r0, r0 + road_w))
    return strips, roads, ("x" if along_x else "y")


def _next_typology(typologies: dict, name: str):
    """The next typology up the intensity ladder, for a refused block."""
    here = (typologies.get(name) or {}).get("rank", 0)
    up = sorted(((t.get("rank", 0), n) for n, t in typologies.items()
                 if t.get("rank", 0) > here
                 and str(t.get("morphology", "pack")) != "terrace"))
    return up[0][1] if up else None


def _terrace_strips(rect, depth: float, alley_m: float, alley_max_m: float = 0.0):
    """Frontage strips for a perimeter terrace block, or NOTHING.

    A terrace block is two rows back to back with a service alley between them
    and nothing else. Both other outcomes this used to produce are wrong:

    * ONE row down the middle of a narrow block. It leaves the block's own
      frontage unbuilt on both sides, which reads as an absurdly wide pavement.
    * Two rows on the FACES of a block far deeper than two terraces need, which
      leaves a courtyard-sized void between their backs rather than an alley.

    So the block short side has to land in a band — 2*depth + alley_m at the
    tight end, 2*depth + alley_max_m at the loose end — and a block outside it
    is refused. The caller falls back to another morphology, which is correct:
    a block that cannot take a terrace pair is not a terrace block.
    """
    x0, y0, x1, y1 = rect
    w, h = x1 - x0, y1 - y0
    lo = 2.0 * depth + alley_m
    hi = 2.0 * depth + (alley_max_m if alley_max_m > alley_m else alley_m * 2.5)
    short = min(w, h)
    if short < lo:
        return []                               # cannot fit a pair at all
    if short > hi:
        # Deeper than a terrace pair needs. This used to keep both rows against
        # ONE face and hand the remainder back to the packer, which is why the
        # pavement came out 2.00 m on one side of the block and 25.30 m on the
        # other — MEASURED, on the 84-92 m blocks. Neither way out is any good:
        # rows on opposite faces leave a courtyard where an alley belongs, and
        # rows on one face leave that lopsided strip. So refuse, exactly as the
        # too-narrow case does, and let the block build as the next typology up.
        # A block that cannot take a terrace pair is not a terrace block.
        return []
    if w >= h:                                  # faces run along X
        return [((x0, y0, x1, y0 + depth), "x", True),
                ((x0, y1 - depth, x1, y1), "x", False)]
    return [((x0, y0, x0 + depth, y1), "y", True),
            ((x1 - depth, y0, x1, y1), "y", False)]


def _face_runs(face_len: float, pool, rng, med: float, sigma: float,
               gap_lo: float, gap_hi: float):
    """Break a block face into terrace runs separated by gaps.

    MEASURED (Philadelphia, Boston Back Bay, Baltimore; contiguous party-wall
    runs at a 1.0 m wall gap): a run is 6 houses / 43-48 m of frontage at the
    median and 17-29 houses / 105-130 m at p90 — NOT a whole block face. Filling
    the face end to end makes a 120 m unbroken wall, which is why runs are
    sampled log-normally instead. 93-97% of row-house stock sits in runs of >=3,
    so single houses only ever appear closing out a run.
    """
    shortest = min(e[3]["sy"] for e in pool)
    out, cursor = [], 0.0
    while face_len - cursor >= shortest:
        want = min(face_len - cursor, med * math.exp(rng.gauss(0.0, sigma)))
        chosen = _tile_run(want, pool, rng)
        if not chosen:
            break
        used = sum(e[3]["sy"] for e in chosen)
        out.append((cursor, chosen))
        cursor += used + rng.uniform(gap_lo, gap_hi)
    return out


def _lay_terrace(rect, pool, rng, facing_deg: float, alley_m: float,
                 run_median_m: float = 45.0, run_sigma: float = 0.70,
                 run_gap_m=(3.0, 9.0), alley_max_m: float = 0.0,
                 strips=None):
    """Place party-wall rows around *rect*. Returns ``[(entry, x, y, yaw)]``.

    Rows inside a run butt end to end with no gap — a terrace shares walls, and
    the 2.5 m packing gap between them is exactly what makes a row read as
    detached houses. Between runs there IS a gap: a side alley or a vacant lot.
    """
    depth = min(e[3]["sx"] for e in pool)
    out = []
    for (sx0, sy0, sx1, sy1), axis, near_lo in (
            strips if strips is not None
            else _terrace_strips(rect, depth, alley_m, alley_max_m)):
        face_len = (sx1 - sx0) if axis == "x" else (sy1 - sy0)
        runs = _face_runs(face_len, pool, rng, run_median_m, run_sigma,
                          run_gap_m[0], run_gap_m[1])
        if not runs:
            continue
        # Centre the whole sequence on the face so the end gaps match.
        span = runs[-1][0] + sum(e[3]["sy"] for e in runs[-1][1])
        shift = (face_len - span) / 2.0
        for start, chosen in runs:
            cursor = start + shift
            for e in chosen:
                ln, dp = e[3]["sy"], e[3]["sx"]
                if axis == "x":
                    # Depth may be less than the strip; sit the row on the
                    # street edge, the face nearer the block boundary.
                    cx = sx0 + cursor + ln / 2.0
                    cy = sy0 + dp / 2.0 if near_lo else sy1 - dp / 2.0
                    yaw = 90.0 + (facing_deg if near_lo
                                  else 180.0 + facing_deg)
                else:
                    cy = sy0 + cursor + ln / 2.0
                    cx = sx0 + dp / 2.0 if near_lo else sx1 - dp / 2.0
                    yaw = facing_deg if near_lo else 180.0 + facing_deg
                out.append((e, cx, cy, yaw))
                cursor += ln
    return out


# ---------------------------------------------------------------------------
# morphology: pack
# ---------------------------------------------------------------------------

def _pack_free(rect, pool, gap: float, min_side: float, rng, sky, typ,
               area_band: float = 0.55):
    """Guillotine-pack *pool* into *rect*; returns ``[(entry, cx, cy, yaw)]``.

    Candidates are tried largest-footprint-first so a gap closes with one big
    building rather than several small ones, and the orientation putting the
    long axis along the gap's long axis is preferred. Which of the near-largest
    lands there is the skyline's choice, not the packer's.

    *area_band* is how far below the largest fitting footprint an asset may be
    and still compete. At 0.85 the band is often a single asset — the largest
    thing that fits wins every equivalent gap in the city, which is why two
    models took 66% of all buildings and why the skyline's repeat penalty had
    nothing to act on. Widening it puts real choice in the band; the packer
    still prefers big, it just no longer decides alone.
    """
    out, stack = [], [tuple(rect)]
    while stack:
        x0, y0, x1, y1 = stack.pop()
        w, h = x1 - x0, y1 - y0
        if w < min_side or h < min_side:
            continue
        fits = []
        for e in pool:
            sx, sy = e[3]["sx"], e[3]["sy"]
            if sx <= w and sy <= h:
                fits.append((e, sx, sy, 0.0))
            if sy <= w and sx <= h and abs(sx - sy) > 1e-6:
                fits.append((e, sy, sx, 90.0))
        if not fits:
            continue
        best = max(f[1] * f[2] for f in fits)
        top = [f for f in fits if f[1] * f[2] >= best * area_band]
        along = w >= h
        oriented = [f for f in top if (f[1] >= f[2]) == along] or top
        chosen = sky.choose([f[0] for f in oriented], sky.target(typ, x0, y0))
        e, bw, bh, yaw = next(f for f in oriented if f[0] is chosen)

        cx, cy = x0 + bw / 2.0, y0 + bh / 2.0
        out.append((e, cx, cy, yaw))
        sky.record(cx, cy, e[3]["sz"])

        right_w, top_h = x1 - (x0 + bw) - gap, y1 - (y0 + bh) - gap
        if right_w >= top_h:
            if right_w > 0:
                stack.append((x0 + bw + gap, y0, x1, y1))
            if top_h > 0:
                stack.append((x0, y0 + bh + gap, x0 + bw, y1))
        else:
            if top_h > 0:
                stack.append((x0, y0 + bh + gap, x1, y1))
            if right_w > 0:
                stack.append((x0 + bw + gap, y0, x1, y0 + bh))
    return out


# ---------------------------------------------------------------------------
# the pass
# ---------------------------------------------------------------------------

def _fits_block(pool, rect):
    w, h = rect[2] - rect[0], rect[3] - rect[1]
    return any(min(e[3]["sx"], e[3]["sy"]) <= min(w, h) + _FIT_TOL_M
               and max(e[3]["sx"], e[3]["sy"]) <= max(w, h) + _FIT_TOL_M
               for e in pool)


def rezone_blocks(config: dict, layout: dict, placements: list, resolver, rng,
                  zone_at) -> dict:
    """Rebuild every non-park block to its zone's typology.

    Rebuilding rather than swapping in place because morphology, not just
    height, is what distinguishes a row-house block from a tower block — and a
    swap can only ever replace a building with a smaller one, so it can never
    turn a mid-rise lot into a tower lot.

    Disaster art survives untouched: ruins from the damaged/destroyed pools and
    their debris stay exactly where `build_city` put them and are treated as
    obstacles, so re-zoning cannot quietly repair a bombed city.
    """
    cfg = _stage(config, "layout").get("districts") or {}
    typologies = cfg.get("typologies") or {}
    if not typologies:
        return {}

    cache: dict = {}
    intact = {e[0] for e in _pools_for(config, resolver, ["intact"], cache)}
    order = sorted(typologies,
                   key=lambda n: -float(typologies[n].get("rank", 0.0)))
    pool_of = {n: _pools_for(config, resolver,
                             typologies[n].get("pools") or [n], cache)
               for n in typologies}

    # Never demolish what cannot be rebuilt. This pass drops every intact
    # building and re-packs the block from its typology pools, so an asset set
    # that does not define those pools loses its buildings outright — and
    # silently, because ruins are not in the intact set and survive, leaving a
    # city of nothing but rubble. That is what happened when the `downtown`
    # locale started enabling districts (Phase 1) for presets still on the v1
    # `urban` set: houses went 0 at severity 0 and *rose* with severity.
    if not any(pool_of.values()):
        print("[districts] no typology pools resolve for this asset set — "
              "leaving buildings as packed")
        return {}

    inset = block_inset(config, resolver)
    gap = float(_stage(config, "layout").get("packing", {}).get("building_gap_m", 2.5))
    bleed = float((cfg.get("zoning") or {}).get("bleed", 0.12))
    exclusions = config.get("exclusions") or []
    sky = _Skyline(cfg, rng)
    parks = set(park_blocks(layout, placements))

    # Ruins and their debris are immovable; everything else already standing
    # inside a block is an obstacle too. Street furniture is absent — city_detail
    # runs after this pass.
    keep_cats = {"play_structure", "trail", "tree", "bus_stop", "traffic_light",
                 "debris_pile", "debris"}
    survivors = [p for p in placements
                 if p.get("category") != "house" or p.get("usd") not in intact]

    # Terrace superblocks are deliberately rare: they are the distinctive part
    # of the city, and each one eats a large block and adds a street.
    max_super = int((cfg.get("typologies", {}).get("rowhouse") or {})
                    .get("max_superblocks", 2))
    n_super = 0
    splits = []          # (original block, [sub-blocks], typology)

    doomed = set()
    for i, p in enumerate(placements):
        if p.get("category") != "house" or p.get("usd") not in intact:
            continue
        x, y = float(p["x_m"]), float(p["y_m"])
        for b in layout.get("blocks", []):
            if b not in parks and b[0] <= x <= b[2] and b[1] <= y <= b[3]:
                doomed.add(i)
                break
    if doomed:
        placements[:] = [p for i, p in enumerate(placements) if i not in doomed]

    obstacles = [_rect_of(p, resolver, margin=gap)
                 for p in placements
                 if p.get("category") in keep_cats or p.get("category") == "house"]

    typ_of, counts, added = {}, {}, 0
    paved: list = []                 # terrace-block interiors to un-pave
    for blk in layout.get("blocks", []):
        if blk in parks:
            typ_of[blk] = "park"
            counts["park"] = counts.get("park", 0) + 1
            continue
        rect = (blk[0] + inset, blk[1] + inset, blk[2] - inset, blk[3] - inset)
        if rect[2] - rect[0] < 4.0 or rect[3] - rect[1] < 4.0:
            continue

        bcx, bcy = (blk[0] + blk[2]) / 2.0, (blk[1] + blk[3]) / 2.0
        here = zone_at(bcx, bcy) or {}
        # `remap_buildings` passes `zone_field`, which yields a TYPOLOGY, so
        # `.get("name")` is the typology name. The `mix` branch is for a caller
        # that hands in `assign`'s RING instead — a ring name is not a key in
        # pool_of and would silently fall through to "first typology that fits".
        name = (_pick(here["mix"], rng) if here.get("mix")
                else here.get("name"))
        if rng.random() < bleed and hasattr(zone_at, "bleed_name"):
            name = zone_at.bleed_name(bcx, bcy, rng) or name
        # A typology whose pool cannot fit this block would leave it empty;
        # step down through the ranks until something can be built.
        for cand in [name] + [n for n in order if n != name]:
            if cand in pool_of and _fits_block(pool_of[cand], rect):
                name = cand
                break
        else:
            continue
        typ = dict(typologies.get(name) or {})
        typ["name"] = name
        typ_of[blk] = name
        counts[name] = counts.get(name, 0) + 1

        local = [o for o in obstacles
                 if o[2] > rect[0] and o[0] < rect[2]
                 and o[3] > rect[1] and o[1] < rect[3]]
        pool = pool_of[name]

        if str(typ.get("morphology", "pack")) == "terrace" and not local:
            gaps = typ.get("run_gap_m") or (3.0, 9.0)
            alley = float(typ.get("alley_m", 6.0))
            depth0 = min(e[3]["sx"] for e in pool)
            road_w = float(typ.get("street_w_m", 11.4))
            quad = None
            # A terrace SUPERBLOCK: four rows around an internal street, the
            # row-house district cut on its own finer grid. Budgeted, because
            # the whole point is that these are a distinctive few rather than
            # the default — and because each one consumes a large block.
            if n_super < max_super:
                quad = _terrace_quad(rect, depth0, alley, road_w)
            if quad:
                strips, road_spans, road_axis = quad
                laid = _lay_terrace(rect, pool, rng,
                                    float(typ.get("facing_offset_deg", 0.0)),
                                    alley,
                                    float(typ.get("run_median_m", 45.0)),
                                    float(typ.get("run_sigma", 0.70)),
                                    (float(gaps[0]), float(gaps[1])),
                                    float(typ.get("alley_max_m", 0.0)),
                                    strips=strips)
                if laid:
                    n_super += 1
                    for span in road_spans:
                        _add_internal_street(layout, rect, span, road_axis,
                                             road_w, typ)
                    # SPLIT THE BLOCK along the new streets. apply_ground_planes
                    # draws one ground mesh PER BLOCK at z=0.01, over the
                    # asphalt base at z=0 — so a corridor inside an unchanged
                    # block rect is simply buried under that block's own grass.
                    # Splitting leaves the street as a gap between blocks, which
                    # is exactly how every other road in the scene shows
                    # through.
                    splits.append((blk, _split_along(blk, road_spans,
                                                     road_axis), name))
                    counts["terrace_superblock"] = \
                        counts.get("terrace_superblock", 0) + 1
                    counts["terrace_rows"] = counts.get("terrace_rows", 0) \
                        + len(strips)
            else:
                laid = _lay_terrace(rect, pool, rng,
                                    float(typ.get("facing_offset_deg", 0.0)),
                                    alley,
                                    float(typ.get("run_median_m", 45.0)),
                                    float(typ.get("run_sigma", 0.70)),
                                    (float(gaps[0]), float(gaps[1])),
                                    float(typ.get("alley_max_m", 0.0)))
            # The interior stays PAVED. A row-house block in NYC or Boston has
            # a paved service alley behind it, not lawn — exposing the grass
            # plane here is what made these blocks read as suburban houses with
            # back gardens.
            if not laid:
                # The block is outside the terrace band, so it is not a terrace
                # block. Build it as the next typology up rather than leaving a
                # hole — refusing here is the whole point, but refusing AND
                # leaving it empty would just trade one artefact for another.
                alt = _next_typology(typologies, name)
                if alt and pool_of.get(alt):
                    name, typ, pool = alt, dict(typologies[alt]), pool_of[alt]
                    typ["name"] = alt
                    counts[alt] = counts.get(alt, 0) + 1
                    counts[  # the refused terrace no longer counts as one
                        "rowhouse_refused"] = counts.get("rowhouse_refused", 0) + 1
                    min_side = min(min(e[3]["sx"], e[3]["sy"]) for e in pool)
                    for fr in free_rects(rect, local, min_side):
                        laid += _pack_free(fr, pool, gap, min_side, rng, sky,
                                           typ)
            elif quad:
                # A SUPERBLOCK IS ENTIRELY ITS OWN. Four rows and their street
                # are the whole composition, so nothing else is built here —
                # that is the point of giving the row-house district its own
                # grid. Packing the leftover with mid-rise is exactly the
                # "brownstones sharing a block with other buildings" the
                # superblock exists to stop.
                pass
            else:
                # NOTHING BEHIND THE PAIR. Two rows is the whole composition on
                # a plain terrace block; building the leftover out with mid-rise
                # reads as a third row of brownstones and is the same
                # sharing-a-block artefact the superblock exists to avoid. The
                # space stays open — rear gardens, which is what is actually
                # behind a terrace.
                pass
        else:
            laid = []
            min_side = min(min(e[3]["sx"], e[3]["sy"]) for e in pool)
            band = float(typ.get("perimeter_depth_m", 0.0))
            for outer in _perimeter_rects(rect, band, min_side):
                for fr in free_rects(outer, local, min_side):
                    laid += _pack_free(fr, pool, gap, min_side, rng, sky, typ)

        for entry, cx, cy, yaw in laid:
            if exclusions and _in_exclusion(cx, cy, exclusions):
                continue
            placements.append(_new_placement(entry, cx, cy, yaw))
            obstacles.append(_rect_of(placements[-1], resolver, margin=gap))
            sky.record(cx, cy, entry[3]["sz"])
            added += 1

    n_paving = 0
    if paved:
        keep = []
        for p in placements:
            if p.get("category") == "concrete" and any(
                    r[0] <= p["x_m"] <= r[2] and r[1] <= p["y_m"] <= r[3]
                    for r in paved):
                n_paving += 1
                continue
            keep.append(p)
        placements[:] = keep

    # Rewrite the block list so the superblocks' internal streets read as gaps.
    # Done after the loop rather than during it because the loop iterates
    # layout["blocks"].
    if splits:
        blocks = list(layout.get("blocks") or [])
        for original, pieces, tname in splits:
            if original in blocks:
                i = blocks.index(original)
                blocks[i:i + 1] = pieces
            typ_of.pop(original, None)
            for p in pieces:
                typ_of[p] = tname
        layout["blocks"] = blocks
        print(f"[districts] {len(splits)} terrace superblock(s) split into "
              f"{sum(len(p) for _o, p, _t in splits)} blocks around their "
              f"own streets")

    layout["_typology_of"] = typ_of
    detail = "  ".join(f"{k}={v}" for k, v in sorted(counts.items()))
    print(f"[districts] rezoned {len(typ_of)} blocks -> {added} buildings "
          f"({len(doomed)} replaced, {n_paving} paving tiles removed from "
          f"terrace yards)   {detail}")
    # Per-typology accounting, because "I don't see any X" has cost several
    # rounds of guessing. A typology can come out at zero for four different
    # reasons and they are indistinguishable in the viewport: no block was
    # zoned for it, its pool did not resolve, its pool did not fit the block it
    # got, or the block fell outside the terrace band and was refused. Say
    # which.
    for name in sorted(typologies):
        pool = pool_of.get(name) or []
        blocks_here = sum(1 for v in typ_of.values() if v == name)
        built = counts.get(name, 0)
        note = ""
        if not pool:
            note = "  <- POOL EMPTY (asset paths unresolved?)"
        elif blocks_here == 0:
            note = "  <- no block zoned for it"
        elif built == 0:
            note = "  <- zoned but nothing built"
        print(f"[districts]   {name:9s} pool={len(pool):2d} "
              f"blocks={blocks_here:2d} built={built:2d}{note}")
    if counts.get("rowhouse_refused"):
        print(f"[districts]   terrace refused on "
              f"{counts['rowhouse_refused']} block(s) outside the alley band "
              f"— rebuilt as the next typology up")
    return typ_of


def infill_blocks(config: dict, layout: dict, placements: list, resolver,
                  rng, zone_at) -> int:
    """Build on whatever block area is still bare pavement.

    `build_city`'s packer abandons a leftover the moment no library building
    fits it, and with `packing.placeholders.enabled` false those leftovers keep
    the block's concrete tiles and nothing else — which is the "the sidewalk is
    enormous here" effect: it is not sidewalk, it is unbuilt paved interior.
    """
    cfg = (_stage(config, "layout").get("districts") or {}).get("infill") or {}
    if not cfg.get("enabled", True):
        return 0
    dcfg = _stage(config, "layout").get("districts") or {}
    cache: dict = {}
    # NEVER infill from a terrace pool. A row house only works in a party-wall
    # run laid along a block face; dropped into a leftover gap by the packer it
    # lands on its own, spaced by its own 21 m DEPTH, so a column of them reads
    # as houses stacked front-to-back. Terrace typologies are placed by
    # `_lay_terrace` and by nothing else.
    typs = dcfg.get("typologies") or {}
    names = [n for n, t in typs.items()
             if str(t.get("morphology", "pack")) != "terrace"]
    pool = _pools_for(config, resolver,
                      (cfg.get("pools") or names or ["intact"]), cache)
    if not pool:
        return 0

    smallest = min(min(e[3]["sx"], e[3]["sy"]) for e in pool)
    min_gap = float(cfg.get("min_gap_m", 0.0)) or smallest
    gap = float(cfg.get("gap_m", _stage(config, "layout").get("packing", {})
                        .get("building_gap_m", 2.5)))
    margin = float(cfg.get("clearance_m", gap))
    inset = block_inset(config, resolver)
    exclusions = config.get("exclusions") or []

    obstacles = _placement_rects(
        placements, resolver,
        {"house", "play_structure", "trail", "tree", "bus_stop",
         "traffic_light", "debris_pile"}, margin)
    sky = _Skyline(dcfg, rng)
    for p in placements:
        if p.get("category") == "house":
            fp = resolver.get(p["usd"], "house", scale=p.get("scale"),
                              axis_up=p.get("axis_up", "Z"))
            sky.record(float(p["x_m"]), float(p["y_m"]), fp["sz"])

    parks = set(park_blocks(layout, placements))
    typ_of = layout.get("_typology_of") or {}
    added = gaps = 0
    gap_area = 0.0
    for blk in layout.get("blocks", []):
        if blk in parks:
            continue
        # A terrace block's middle is its back yards and service alley, not a
        # gap to be filled.
        tname = typ_of.get(blk)
        tcfg = (dcfg.get("typologies") or {}).get(tname) or {}
        if str(tcfg.get("morphology", "pack")) == "terrace":
            continue
        rect = (blk[0] + inset, blk[1] + inset, blk[2] - inset, blk[3] - inset)
        typ = dict(tcfg)
        typ["name"] = tname
        for fr in free_rects(rect, obstacles, min_gap):
            gaps += 1
            gap_area += (fr[2] - fr[0]) * (fr[3] - fr[1])
            for entry, cx, cy, yaw in _pack_free(fr, pool, gap, min_gap, rng,
                                                 sky, typ):
                if exclusions and _in_exclusion(cx, cy, exclusions):
                    continue
                placements.append(_new_placement(entry, cx, cy, yaw))
                obstacles.append(_rect_of(placements[-1], resolver,
                                          margin=margin))
                added += 1

    print(f"[districts] infill: {gaps} unbuilt gaps ({gap_area:,.0f} m2) "
          f"-> {added} buildings")
    return added


def remap_buildings(config: dict, layout: dict, placements: list, resolver,
                    rng, district_at) -> int:
    """Pipeline hook: rezone, infill, then rebuild the parks.

    WHY THIS DRIVES THE OTHER PASSES: `generate_city_v2` owns the pipeline and
    is off-limits to this work, and this is the only hook it offers that
    receives the placement list. `layout` carries a done-flag so a caller that
    grows explicit calls later gets a no-op rather than a doubled scene.
    """
    cfg = _stage(config, "layout").get("districts") or {}
    if not cfg.get("enabled") or layout.get("_districts_done"):
        return 0
    layout["_districts_done"] = True

    zone_at = zone_field(config, layout.get("region"))
    typ_of = rezone_blocks(config, layout, placements, resolver, rng, zone_at)
    infill_blocks(config, layout, placements, resolver, rng, zone_at)
    try:
        from detail import parks
    except ImportError:
        return len(typ_of)
    parks.build(config, layout, placements, resolver, rng)
    return len(typ_of)
