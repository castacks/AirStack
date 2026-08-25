"""
suburb_yardplan.py — yard planting for lots that are not axis-aligned.

WHY `suburb_yards` COULD NOT SIMPLY BE CALLED
---------------------------------------------
`suburb_yards` is a good pass and this module reuses the hard half of it. What
it cannot do is this layout's geometry. Its yard records are world AABBs —
`x0, y0, x1, y1 = rect` — and its `_house_side` picks the wall a foundation row
follows by comparing AXIS-ALIGNED outward normals. On a curved street the lots
turn with the street:

    lots within 5 deg of axis-aligned      9%   of 1,523
    AABB area / true footprint             median 1.74x, p90 2.00x, max 2.16x
    overlapping house AABB pairs           132   (true oriented boxes: 0)

So handing it AABBs would inflate every yard by about three quarters, spill
planting across the kerb and into next door, produce 132 overlapping yards, and
put the foundation row against a wall the house does not have — which is
precisely the failure `_house_side` was written to avoid.

WHAT IS REUSED, AND WHAT IS REPLACED
------------------------------------
Reused verbatim, because neither touches geometry and both encode work that
would be silly to redo:

    _Lib      species pools carrying a MEASURED point cost and a sampling
              weight, so an expensive handsome species can be carried as an
              occasional accent instead of being dropped or bankrupting the run
    _Budget   two purses, trees and everything else, allocated per unit area.
              A ceiling, not what decides the planting: with instancing on it
              barely binds here (see :func:`plan`). It was written for the
              urban scene, which OOM-killed at 89.1M points.

Replaced: all of the geometry.

THE LOCAL FRAME IS THE WHOLE IDEA
---------------------------------
Every lot already knows its own frame — `suburb_parcel` solved it when it
placed the house: a frontage point, the tangent ``u`` along the street, and the
inward normal ``n``. So planting is composed in ``(along, deep)`` coordinates
and transformed to the world at the end:

    world = house_centre + u * along + n * deep

which means the composition is written once, in a frame where the house front
is simply ``deep < 0``, and it follows the street round its curve for free. No
case analysis, no nearest-wall test, nothing to get wrong when the lot rotates.

    deep
      ^   . . . T . . T . .   rear boundary row ON THE REAL LOT LINE, and
      |   .                .   canopy massed over the back garden
      |   .  T   T     T   .
      |   . T [ HOUSE ] T  .   side yards: the gap between neighbours
      |   .   ###row###    .   foundation row along the true front wall
      |   .      T         .   specimen tree, offset to one side
      0 --+------o---------+-- kerb        o = mailbox
          |<---- lot ----->|
                along

EVERY DIMENSION IS READ OFF THE LOT, NOT ASSUMED
------------------------------------------------
The first version of this pass had three constants in it — a 16 m rear yard,
a boundary row half the HOUSE wide, and nothing at all down the sides — and
all three are wrong now that `suburb_parcel` plats a real lot:

    lot_depth               21-40 m, per lot        (16 m was over the fence
                                                     on the shallow half)
    side yard, per side     ~2 m tight ... ~12 m estate
    trees inside a building 17-26 a seed, from the rear tree thrown a flat
                            5-12 m past the back wall into the lot behind

So the lot rectangle (`lot_corners`, `lot_depth`), the garage box and the
fence runs are all read off the house record, and every tree is tested
against every building in the scene rather than against its own.
"""

import math

from detail.suburb_yards import _Lib, _Budget, _tagged          # geometry-free, reused as-is
from layout import suburb_net as sn

DEFAULTS = {
    "point_budget": 30_000_000,
    "tree_budget_frac": 0.62,
    # Front yard: composed, not scattered.
    "foundation_row": [3, 7],        # shrubs along the house front wall
    "foundation_gap_m": 1.7,
    "foundation_off_m": 1.1,         # stand-off from the wall
    "specimen_tree_chance": 0.55,
    "mailbox_chance": 0.7,
    # Rear yard: massed along the boundary, canopy over the middle.
    "rear_boundary_step_m": 4.5,
    # Was a 0.75 coin flip for ONE tree, which is a quarter of the back gardens
    # in the suburb with no canopy at all and none of the rest with more than a
    # single trunk. A back garden is where a suburb's canopy actually is.
    "rear_tree_chance": 0.92,
    # Sampled, then clamped by what the yard can hold (below), so a 6 m deep
    # rear yard still gets one and a 16 m deep one gets four.
    "rear_trees": [2, 4],
    # Square metres of rear yard per tree, i.e. a ~6.5 m planting grid. Wider
    # than `tree_min_separation_m` on purpose: separation is the floor the
    # canopies impose, this is the spacing a garden is actually planted at, and
    # driving the count off area is what keeps a tight lot from being a thicket.
    "rear_tree_area_m2": 42.0,
    # Fallback only, for a house record carrying no `lot_depth`. It is WRONG as
    # a constant — lots plat 21-40 m deep, so 16 m past the back wall is over
    # the rear boundary on the shallow half of them, which is how the pass this
    # replaces put 17-26 trees a seed inside somebody else's house.
    "rear_depth_fallback_m": 16.0,
    # -- side yards: the gap between neighbouring houses, previously EMPTY ----
    # The gap is `suburb_parcel.house_gap_m` (4.0 m) only on the tightest lots;
    # frontage is scaled per block by the density class (tight 0.78 to estate
    # 2.20) while the house is not, so the measured side yard runs from ~2 m to
    # ~12 m a side. Every number here is therefore a THRESHOLD on the width the
    # lot actually has, never an assumed width.
    "side_tree_min_w_m": 2.8,        # clear strip a small canopy needs
    "side_shrub_min_w_m": 1.2,       # ...below which only ground cover fits
    "side_tree_chance": 0.85,
    "side_trees": [1, 2],            # 2 only where the strip is long enough
    "side_shrubs": [1, 3],
    # See the note where `shrubs` is bound: off by default.
    "yard_shrubs_enabled": False,
    "side_long_strip_m": 7.5,        # strip length that earns the second tree
    # Planting is held off the lot line by this much. The side and rear fences
    # stand ON that line (`suburb_parcel` plats them from `lot_corners`), so
    # one inset keeps trees out of the fence, off the survey line and out of
    # the neighbour's garage, which is allowed to sit hard on it.
    "lot_inset_m": 0.9,
    "clear_house_m": 0.6,
    "clear_drive_m": 0.4,
    "clear_fence_m": 0.7,            # to a fence RUN this lot owns
    "tree_min_separation_m": 3.0,
    "darts": 12,                     # tries per tree before the spot is given up
    # -- SPECIES BY DISTANCE FROM THE WALL -----------------------------------
    # `yard_trees` spans 3.02 m (Douglas_Fir) to 25.42 m (Black_Oak) of MEASURED
    # crown, and the draw used to ignore that entirely: a 25 m oak was as likely
    # two metres off a side wall as in the middle of the garden, which is the
    # single most obvious thing wrong with the planting from the air.
    #
    # So the weighted draw is biased by where in its own yard the tree stands.
    # The ramp is a GARDEN's, not a suburb's -- `suburb_scene` grades open
    # ground over 12-40 m, but nothing in a yard is 40 m from its own house:
    "tree_size_by_distance": True,
    # ...at or inside 3 m of the wall a tree takes the small end. 3 m is the
    # foundation strip plus a stand-off, i.e. the closest anything is planted.
    "tree_size_near_m": 3.0,
    # ...and 18 m out it takes the big end unreserved. Measured on the shipped
    # preset: the deepest rear-yard planting station sits ~17 m past the back
    # wall, so 18 m is "the far corner of a large garden" and the Black_Oak is
    # reachable there and nowhere else.
    "tree_size_far_m": 18.0,
    # How hard the bias bites: weight is multiplied by (1 - |rank - t|) ** this.
    # 3.0 leaves the far end of the pool ~0.8% of its weight at t=0, i.e. rare
    # rather than banned -- an old tree beside a house is a real thing, a whole
    # street of them is not. THE COST WEIGHTS SURVIVE, deliberately: this
    # multiplies them instead of replacing them, so the expensive accents stay
    # as rare as `yard_trees` priced them.
    "tree_size_sharp": 3.0,
}


class _Solids:
    """Every house and garage in the suburb as an oriented box, on a hash grid.

    A tree inside a garage is as wrong as one inside a house, and the building a
    stray tree lands in is usually NOT the one whose yard it was planted for.
    Measured on the pass this replaces, seeds 1-3: 17/20/26 trees inside a house
    and 2/3/2 inside a garage per seed, every one from the rear tree thrown a
    flat 5-12 m past the back wall into the lot behind.

    So the test is against every solid in the scene rather than this lot's own
    footprint, and at ~1,100 lots against ~4,500 candidate points it has to be a
    grid: the scan version is ~10M oriented-box tests a seed.

    Boxes are inserted into every cell their bounding disc touches INFLATED by
    `pad_max`, so a query need only look in the point's own cell — which is why
    `clear` clamps the pad it is given to `pad_max` rather than trusting it.
    """

    CELL = 32.0

    def __init__(self, houses, pad_max=2.0, rings=()):
        self.pad_max = float(pad_max)
        self.g = {}
        # RINGS ARE SOLIDS TOO. A swimming pool is a thing you cannot plant in
        # for exactly the reason a house is, so it goes through this index and
        # not through a mechanism of its own — same oriented box, same grid,
        # same `clear()` margin. `modular_house.pool_at` returns the water
        # rectangle as four corners in ring order, which is all this needs.
        for ring in (rings or ()):
            if not ring or len(ring) < 4:
                continue
            ax, ay = float(ring[0][0]), float(ring[0][1])
            bx, by = float(ring[1][0]), float(ring[1][1])
            cx2, cy2 = float(ring[2][0]), float(ring[2][1])
            w = math.hypot(bx - ax, by - ay)
            d = math.hypot(cx2 - bx, cy2 - by)
            if w < 1e-6 or d < 1e-6:
                continue
            u = ((bx - ax) / w, (by - ay) / w)
            c = (0.25 * sum(float(q[0]) for q in ring[:4]),
                 0.25 * sum(float(q[1]) for q in ring[:4]))
            self._add(c, u, w, d)
        for h in houses:
            self._add(h.get("c"), h.get("u"), h.get("w"), h.get("d"))
            # `.get`, because `garage` is None on the archetypes that have
            # none and the key may not be there at all on an older record.
            g = h.get("garage")
            if isinstance(g, dict):
                self._add(g.get("c"), g.get("u") or h.get("u"),
                          g.get("w"), g.get("d"))

    def _add(self, c, u, w, d):
        if not c or not u or not w or not d:
            return
        hw, hd = float(w) / 2.0, float(d) / 2.0
        r = math.hypot(hw, hd) + self.pad_max
        box = (float(c[0]), float(c[1]), float(u[0]), float(u[1]), hw, hd)
        for gx in range(int(math.floor((c[0] - r) / self.CELL)),
                        int(math.floor((c[0] + r) / self.CELL)) + 1):
            for gy in range(int(math.floor((c[1] - r) / self.CELL)),
                            int(math.floor((c[1] + r) / self.CELL)) + 1):
                self.g.setdefault((gx, gy), []).append(box)

    def clear(self, x, y, pad=0.0):
        """Is (x, y) outside every building, by at least *pad*?"""
        pad = min(float(pad), self.pad_max)
        for bx, by, ux, uy, hw, hd in self.g.get(
                (int(math.floor(x / self.CELL)),
                 int(math.floor(y / self.CELL))), ()):
            dx, dy = x - bx, y - by
            if (abs(dx * ux + dy * uy) <= hw + pad
                    and abs(-dx * uy + dy * ux) <= hd + pad):
                return False
        return True


class _Spacing:
    """The trees already standing, on a grid keyed at the separation distance.

    A grid rather than the linear scan it replaces because tripling the canopy
    made the scan quadratic in a number that matters: ~4,500 trees against
    ~15,000 candidate points is ~34M distance tests a seed, where a cell lookup
    is nine buckets. The rule is enforced across EVERYTHING this pass plants —
    front, side and rear, and across lot boundaries, which is what stops two
    neighbours' side yards planting the same two metres twice.
    """

    def __init__(self, sep):
        self.sep = float(sep)
        self.cell = max(0.5, float(sep))
        self.g = {}
        self.n = 0

    def ok(self, x, y):
        cx, cy = int(math.floor(x / self.cell)), int(math.floor(y / self.cell))
        s2 = self.sep * self.sep
        for a in range(cx - 1, cx + 2):
            for b in range(cy - 1, cy + 2):
                for px, py in self.g.get((a, b), ()):
                    if (x - px) ** 2 + (y - py) ** 2 < s2:
                        return False
        return True

    def add(self, x, y):
        self.g.setdefault((int(math.floor(x / self.cell)),
                           int(math.floor(y / self.cell))), []).append((x, y))
        self.n += 1

    def __len__(self):
        return self.n


def _seg_dist(px, py, a, b):
    """Distance from a point to the segment *a*-*b* (a fence run)."""
    ax, ay = a[0], a[1]
    bx, by = b[0], b[1]
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 <= 1e-9 else max(0.0, min(1.0, ((px - ax) * vx
                                                 + (py - ay) * vy) / L2))
    return math.hypot(px - (ax + vx * t), py - (ay + vy * t))


def _rng_pair(v, fallback):
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    if isinstance(v, (int, float)):
        return float(v), float(v)
    return fallback


def _canopy_ranks(pool, resolver):
    """Each entry's place in the pool by MEASURED crown width, 0.0 to 1.0.

    Returns a list aligned with *pool*, or None when the measurements do not
    separate it -- under `measure_usds: false`, or with the assets unreachable,
    every entry falls back to the one `fallback_sizes.tree` and a "ranking" of
    identical numbers is a ranking by dict order. Half a metre of spread is
    nothing against the 22 m this library really covers, so below that the
    caller is told to go back to the plain weighted draw.

    `max(sx, sy)` is the key: crown WIDTH, because what decides whether a tree
    belongs beside a wall is how far it reaches out, not how tall it is.
    """
    if resolver is None or len(pool) < 2:
        return None
    sizes = []
    for e in pool:
        fp = resolver.get(e["usd"], "tree", scale=e.get("scale", 1.0),
                          axis_up=e.get("axis_up", "Z"))
        sizes.append(max(float(fp.get("sx", 0.0) or 0.0),
                         float(fp.get("sy", 0.0) or 0.0)))
    lo, hi = min(sizes), max(sizes)
    if hi - lo <= 0.5:
        return None
    order = sorted(range(len(pool)), key=lambda i: (sizes[i], i))
    ranks = [0.0] * len(pool)
    for place, i in enumerate(order):
        ranks[i] = place / float(len(pool) - 1)
    return ranks


def _size_t(dist_m, near_m, far_m):
    """Where a tree *dist_m* from its own house wall sits on the size ramp.

    0 at or inside `near_m` -- small end of the pool; 1 at or beyond `far_m`.
    A ramp and not a threshold because the thing being modelled is a gradient:
    what a householder plants against a wall, what stands in the middle of the
    garden, and everything between.
    """
    if far_m <= near_m:
        return 1.0 if dist_m >= far_m else 0.0
    return min(1.0, max(0.0, (float(dist_m) - near_m) / (far_m - near_m)))


def _pick(pool, rng, ranks=None, t=None, sharp=3.0):
    """Weighted draw. Species is chosen BEFORE price is checked, deliberately:
    drawing only from what is affordable silently deletes every expensive
    species, which is the failure `suburb_yards` documents.

    With *ranks* and *t*, the pool's own weights are MULTIPLIED by a size
    affinity -- ``(1 - |rank - t|) ** sharp`` -- rather than replaced by one.
    That is what keeps the cost weighting intact: `yard_trees` prices its
    handsome species at a twenty-fifth of the cheap ones and this must not
    quietly undo that while it is choosing a size.
    """
    if not pool:
        return None
    w = [max(0.0, float(e.get("weight", 1.0))) for e in pool]
    if ranks is not None and t is not None:
        tt = min(1.0, max(0.0, float(t)))
        w = [wi * (1.0 - abs(r - tt)) ** sharp for wi, r in zip(w, ranks)]
    total = sum(w)
    if total <= 0.0:
        return pool[rng.randrange(len(pool))]
    r = rng.random() * total
    for e, wi in zip(pool, w):
        r -= wi
        if r <= 0.0:
            return e
    return pool[-1]


def _clear_of(off, gaps, limit, rng=None, pad=1.5):
    """A front-yard offset near +/-*off* that no paved run occupies.

    `front_gaps` is where the drive and the walk cross the front lot line, as
    (offset along the frontage, half width). On this kit both sit on the same
    side of every style, so a coin-flip side put the specimen tree's trunk
    within 0.1 m of the drive edge on three styles of eight.

    Done by SUBTRACTING the padded runs from the lot frontage and taking the
    nearest surviving point, which is exact. Nudging the offset away from
    whichever run it hit is not: on a lot with two runs, stepping clear of the
    drive walks straight into the walk, and 113 of 5,000 random lots did.
    """
    free, x = [], -limit
    for a, b in sorted((c - hw - pad, c + hw + pad) for c, hw in gaps):
        if b <= -limit or a >= limit:
            continue
        if a > x:
            free.append((x, a))
        x = max(x, b)
    if limit > x:
        free.append((x, limit))
    if not free:
        return None

    def nearest(t):
        lo, hi = min(free, key=lambda r: 0.0 if r[0] <= t <= r[1]
                     else min(abs(t - r[0]), abs(t - r[1])))
        return min(max(t, lo), hi)

    signs = [-1.0, 1.0]
    if rng is not None and rng.random() < 0.5:
        signs.reverse()          # ties would otherwise put every tree left
    best = None
    for sgn in signs:
        a = nearest(sgn * off)
        cost = abs(a - sgn * off)
        if best is None or cost < best[0]:
            best = (cost, a)
    return best[1]


def plan(config, parcels, rng, resolver=None, keepout_discs=None,
         keepout_rings=None):
    """Plant every lot in *parcels*. Returns ``(placements, stats)``.

    *parcels* is `suburb_parcel.parcel_blocks` output, so each house carries its
    own frame: centre ``c``, along-street unit ``u``, footprint ``w`` x ``d``.
    """
    cfg = dict(DEFAULTS)
    cfg.update(config.get("suburb_yardplan") or config.get("suburb_yards") or {})

    lib = _Lib(config)
    # BUSHES ARE OFF by default. The foundation row, the side-yard fill and the
    # rear boundary row each drop a handful of shrubs per lot, and at suburb
    # scale that reads as sparse litter rather than as planting — the plants are
    # too few and too far apart to compose. Trees carry the greenery instead.
    # Set `yard_shrubs_enabled: true` to bring them back.
    shrubs = (lib.pool("yard_shrubs") or lib.pool("plants")) \
        if cfg.get("yard_shrubs_enabled", False) else []
    trees = lib.pool("yard_trees") or lib.pool("trees")
    # SPLIT BY TAG. `yard_props` holds mailboxes, a bin, patio furniture and a
    # shelter, and the kerb slot below drew from ALL of them — so two thirds of
    # the time the "mailbox at the kerb" was a table-and-chairs, a garden bench
    # or a 2.3 m transit shelter, standing in the street. Each now goes where it
    # belongs.
    _pp = lib.pool("yard_props")
    mailboxes = _tagged(_pp, "mailbox")
    patio_props = [e for e in _pp if "patio" in e["tags"] or "shed" in e["tags"]]

    houses = [h for p in parcels for h in p["houses"]]
    if not houses:
        return [], {"lots": 0}
    # Budget is allocated per unit area, so a big lot gets proportionally more
    # and the spend is uniform in points per square metre across the suburb.
    # SHUFFLED, and that is load-bearing rather than tidy. A purse carries its
    # under-spend forward until a yard can afford a 48.7k-point tree, so in
    # build order the trees all land in the first streets laid and the rest of
    # the suburb comes out bare. Visiting lots in random order spreads them.
    rng.shuffle(houses)
    lot_area = [max(60.0, h["w"] * (h["d"] + 26.0)) for h in houses]
    # `instanced=True` because `suburb_scene` instances the tree/plant
    # categories. Priced per placement instead, the purse refuses canopy long
    # before the scene is anywhere near its real cost.
    #
    # AND THAT IS WHY THE DENSITY BELOW IS A GEOMETRY DECISION, NOT A BUDGET
    # ONE. Swept on seed 1 (1,173 lots), the tree count barely moves with the
    # ceiling: 4,888 trees at 15M, 4,900 at 30M, 4,914 at 300M — 0.3% over a
    # twentyfold sweep, the canopy purse spending 2.86M of its 18.6M share.
    # What refuses a tree is a wall, a fence, a garage or a neighbour's canopy.
    budget = _Budget(float(cfg["point_budget"]), sum(lot_area),
                     float(cfg["tree_budget_frac"]),
                     bool(cfg.get("instanced", True)))

    row_lo, row_hi = _rng_pair(cfg["foundation_row"], (3.0, 7.0))
    gap = float(cfg["foundation_gap_m"])
    stand = float(cfg["foundation_off_m"])
    tree_sep = float(cfg["tree_min_separation_m"])
    clear_house = float(cfg["clear_house_m"])
    clear_drive = float(cfg["clear_drive_m"])
    clear_fence = float(cfg.get("clear_fence_m", 0.7))
    inset = float(cfg.get("lot_inset_m", 0.9))
    rear_fallback = float(cfg.get("rear_depth_fallback_m", 16.0))
    rear_lo, rear_hi = _rng_pair(cfg.get("rear_trees", [2, 4]), (2.0, 4.0))
    rear_area = max(4.0, float(cfg.get("rear_tree_area_m2", 42.0)))
    side_lo, side_hi = _rng_pair(cfg.get("side_trees", [1, 2]), (1.0, 2.0))
    side_sh_lo, side_sh_hi = _rng_pair(cfg.get("side_shrubs", [1, 3]), (1.0, 3.0))
    side_tree_w = float(cfg.get("side_tree_min_w_m", 2.8))
    side_shrub_w = float(cfg.get("side_shrub_min_w_m", 1.2))
    side_long = float(cfg.get("side_long_strip_m", 7.5))
    darts = max(1, int(cfg.get("darts", 12)))
    # SPECIES BY DISTANCE FROM THE WALL. Measured once for the whole suburb --
    # the pool is six entries and the resolver caches, but the ranking is a
    # property of the pool and not of the lot.
    t_ranks = (_canopy_ranks(trees, resolver)
               if cfg.get("tree_size_by_distance", True) else None)
    t_near = float(cfg.get("tree_size_near_m", 3.0))
    t_far = float(cfg.get("tree_size_far_m", 18.0))
    t_sharp = float(cfg.get("tree_size_sharp", 3.0))

    # Every building in the suburb, before anything is planted — one pass over
    # the same `houses` list, so the neighbours are covered too.
    solids = _Solids(houses, pad_max=max(2.0, clear_house + 1.0),
                     rings=keepout_rings)

    out = []
    placed_trees = _Spacing(tree_sep)
    tally = {}
    n_side_tree = n_side_shrub = n_rear_tree = n_front_tree = 0
    n_keepout = 0
    # Normalised once: `((x, y), r)` in world metres, same shape
    # `suburb_parcel` takes.
    _discs = [((float(c[0]), float(c[1])), float(r))
              for (c, r) in (keepout_discs or ())]
    n_side_narrow = n_side_blocked = 0

    def emit(entry, x, y, yaw, category, purse):
        """Charge the purse, then place. Returns False if unaffordable.

        KEEP-OUTS ARE CHECKED FIRST, and before the purse is charged: a prop
        refused for standing on a cul-de-sac turnaround must not also spend the
        budget that would have planted a legal one somewhere else.
        """
        if entry is None:
            return False
        # This pass plats inside `lot_corners`, and a lot's corners can overhang
        # a turnaround — the parcel pass keeps HOUSES off the paving but says
        # nothing about the yard around them, so sheds, patio sets and trees
        # were being planted on the asphalt. Same disc list `parcel_blocks`
        # uses, so the two passes agree about where the pavement is.
        if _discs:
            nonlocal n_keepout
            fpk = None
            if resolver is not None:
                fpk = resolver.get(entry["usd"], category,
                                   scale=entry.get("scale", 1.0),
                                   axis_up=entry.get("axis_up", "Z"))
            pad = 0.5 * max(float((fpk or {}).get("sx", 0.0) or 0.0),
                            float((fpk or {}).get("sy", 0.0) or 0.0))
            for (c, r) in _discs:
                if (x - c[0]) ** 2 + (y - c[1]) ** 2 <= (r + pad) ** 2:
                    n_keepout += 1
                    return False
        pts = float(entry.get("points", 0.0) or 0.0)
        if not purse.can(pts, entry.get("usd")):
            return False
        purse.charge(pts, entry.get("usd"))
        fp = None
        if resolver is not None:
            fp = resolver.get(entry["usd"], category,
                              scale=entry.get("scale", 1.0),
                              axis_up=entry.get("axis_up", "Z"))
        out.append({
            "usd": entry["usd"], "x_m": x, "y_m": y,
            "z_m": (fp or {}).get("base", 0.0),
            # `_Lib.pool` stores the per-asset offset under "yaw", not
            # "yaw_offset" — reading the wrong key silently discarded
            # EVERY yard prop's orientation correction.
            "yaw_deg": yaw + float(entry.get("yaw", 0.0)),
            "roll_deg": 90.0 if entry.get("axis_up") == "Y" else 0.0,
            "pitch_deg": 0.0, "scale": entry.get("scale", 1.0),
            "category": category, "axis_up": entry.get("axis_up", "Z"),
        })
        tally[category] = tally.get(category, 0) + 1
        return True

    for h, area in zip(houses, lot_area):
        budget.open(area)
        cx, cy = h["c"]
        u = h["u"]                      # along the street
        # The INWARD normal, taken from the lot rather than recomputed. perp(u)
        # is perpendicular to u by construction, so no dot product against u can
        # say which of the two normals faces away from the kerb -- the first
        # version of this tried exactly that and the test was identically zero,
        # leaving `deep` pointing at the street on about half the lots.
        n = h.get("n") or sn._perp(u)

        def world(along, deep):
            return (cx + u[0] * along + n[0] * deep,
                    cy + u[1] * along + n[1] * deep)

        half_w, half_d = h["w"] / 2.0, h["d"] / 2.0
        yaw_deg = h["yaw_deg"]

        # -- the LOT, in the same frame as the house -----------------------
        # `frontage` is the kerb point the lot was struck from, so the house
        # centre stands `setback + d/2` inside it: that distance is where the
        # kerb is in `deep`, and `lot_depth` from the same point is where the
        # REAR BOUNDARY is. Read defensively because another pass owns these
        # keys — and the fallbacks are the only reason the constants survive.
        front_off = (sn._dist(h["c"], h["frontage"]) if h.get("frontage")
                     else half_d + 8.0)
        rear_line = (float(h["lot_depth"]) - front_off if h.get("lot_depth")
                     else half_d + rear_fallback)
        # Half-frontage from the platted corners, which is a CHORD; lot_width
        # is the arc it was cut from and overshoots the corner on a curve by
        # metres (`suburb_parcel` measures ~4 m on a 34 m estate frontage).
        lc = h.get("lot_corners")
        half_lot = (sn._dist(lc[0], lc[1]) / 2.0
                    if lc and len(lc) >= 2 else
                    float(h.get("lot_width", h["w"] + 4.0)) / 2.0)
        half_lot = max(half_lot, half_w + 0.4)
        fences = h.get("fence_segs") or ()

        # The garage box, in the same (along, deep) frame. It shares the house's
        # `u`, so the two boxes are axis-aligned to each other and the test is a
        # pair of interval checks rather than an SAT. Nothing is built in it, but
        # it is the ground `suburb_parcel` reserved and the side the drive runs
        # up — which is how this pass knows which side yard is the car's without
        # being handed the drive.
        gar = h.get("garage")
        g_along = g_deep = g_hd = 0.0
        if isinstance(gar, dict) and gar.get("c"):
            dx, dy = gar["c"][0] - cx, gar["c"][1] - cy
            g_along = dx * u[0] + dy * u[1]
            g_deep = dx * n[0] + dy * n[1]
            g_hd = float(gar.get("d", 6.5)) / 2.0
        else:
            gar = None

        def free(x, y, pad):
            """Nothing built here, and no fence of this lot's through it."""
            if not solids.clear(x, y, pad):
                return False
            for seg in fences:
                if len(seg) >= 2 and _seg_dist(x, y, seg[0], seg[1]) < clear_fence:
                    return False
            return True

        def plant_tree(along, deep, pad):
            """Emit a tree at a local-frame point if everything allows it.

            SIZE COMES FREE HERE. `along`/`deep` are already in the house's own
            frame, so the distance from the wall is two subtractions against the
            half-footprint -- no index, no neighbour search, and it is the right
            house by construction: the tree belongs to this lot.
            """
            x, y = world(along, deep)
            if not placed_trees.ok(x, y) or not free(x, y, pad):
                return False
            wall = math.hypot(max(abs(along) - half_w, 0.0),
                              max(abs(deep) - half_d, 0.0))
            if emit(_pick(trees, rng, ranks=t_ranks,
                          t=_size_t(wall, t_near, t_far), sharp=t_sharp),
                    x, y, rng.uniform(0, 360), "tree", budget.tree):
                placed_trees.add(x, y)
                return True
            return False

        # -- front: foundation row along the TRUE front wall ---------------
        n_row = int(rng.uniform(row_lo, row_hi + 0.999))
        span = min(h["w"] - 0.8, max(0.0, (n_row - 1) * gap))
        for k in range(n_row):
            a = (-span / 2.0 + k * gap) if n_row > 1 else 0.0
            x, y = world(a, -half_d - stand)
            emit(_pick(shrubs, rng), x, y, rng.uniform(0, 360), "plant",
                 budget.other)

        # -- front: one specimen tree, offset to a side, never centred -----
        if trees and rng.random() < float(cfg["specimen_tree_chance"]):
            a = _clear_of(half_w * 0.75 + 1.6, h.get("front_gaps") or (),
                          half_lot - inset, rng)
            if a is not None and plant_tree(a, -half_d - 5.0, clear_house):
                n_front_tree += 1

        # -- front: mailbox at the kerb, facing the street -----------------
        # THE OFFSET IS CLAMPED. A flat 8.5 m in front of the front wall is
        # further than most of the platted setback (6.5-11 m), so on the short
        # half of that range the prop landed past the kerb and in the
        # carriageway. `front_off` is the real distance to the lot line.
        if mailboxes and rng.random() < float(cfg["mailbox_chance"]):
            # `front_off` is centre-to-frontage, so the wall-to-kerb
            # distance is that minus half the house depth.
            off = min(8.5, max(1.0, front_off - half_d - 1.0))
            x, y = world(half_w * 0.9, -half_d - off)
            # Face the STREET. `yaw_deg` is the frontage tangent, so the street
            # is at yaw_deg - 90; `+ 90` pointed every one of these into the
            # block. Same convention `build_frontage` uses for its kerb props.
            emit(_pick(mailboxes, rng), x, y, yaw_deg - 90.0, "plant",
                 budget.other)

        # -- SIDE YARDS: the gap between the houses, which had nothing in it --
        # The width is DERIVED, never assumed. `house_gap_m` is a 4.0 m minimum,
        # not the gap: `suburb_parcel` scales the frontage per block by the
        # density class (tight 0.78 through estate 2.20) and does not scale the
        # house, so the strip measures ~2 m a side on a tight lot and ~12 m on an
        # estate one. Below `side_tree_min_w_m` a canopy will not fit and the
        # strip gets ground cover instead of a tree shrunk out of shape to hold
        # it; below `side_shrub_min_w_m` it gets nothing, which is what a 2 m
        # alley between two walls looks like.
        for sgn in (-1.0, 1.0):
            a_in = half_w + clear_house
            a_out = half_lot - inset
            # `deep` runs from the front wall to the back wall, which is the
            # side yard proper — behind it is rear yard and is planted below.
            # Starting at the wall line and not at the kerb is also what keeps
            # this off the drive: with no garage the drive runs at 0.3 * w,
            # inside the house's own span, and stops at the front face.
            d_lo = -half_d + clear_drive
            d_hi = min(half_d, rear_line - inset)
            if gar is not None and sgn * g_along > 0.0:
                # The car's side. The reserved box takes this strip and the
                # drive runs up to its front, so what is plantable is the piece
                # BEHIND it — the breezeway between the box and the back fence.
                # Cut in DEEP only, not in width: a detached double sits at
                # half_w + 0.8 + 3.0 out, past the lot line on any lot under
                # ~26 m of frontage, so also excluding its along-span closed the
                # whole side yard on every garage lot (measured: 1,142 of ~2,346
                # strips refused for width on seed 1).
                d_lo = max(d_lo, g_deep + g_hd + clear_house)
            width = a_out - a_in
            length = d_hi - d_lo
            if width < side_shrub_w or length < 1.0:
                n_side_narrow += 1
                continue
            if trees and width >= side_tree_w \
                    and rng.random() < float(cfg["side_tree_chance"]):
                want = int(rng.uniform(side_lo, side_hi + 0.999)) \
                    if length >= side_long else int(side_lo)
                for i in range(max(0, want)):
                    hit = False
                    for _d in range(darts):
                        # Across the strip: biased to its middle, because a
                        # tree in a 3 m gap has nowhere else to be, and the
                        # inset already holds it off the line either way.
                        a = sgn * (a_in + width * rng.uniform(0.35, 0.65))
                        d = d_lo + length * ((i + rng.uniform(0.2, 0.8))
                                             / max(1, want))
                        if plant_tree(a, min(d, d_hi), clear_house):
                            n_side_tree += 1
                            hit = True
                            break
                    if not hit:
                        n_side_blocked += 1
            if shrubs:
                for _ in range(int(rng.uniform(side_sh_lo, side_sh_hi + 0.999))):
                    a = sgn * (a_in + width * rng.uniform(0.15, 0.85))
                    d = d_lo + length * rng.uniform(0.0, 1.0)
                    x, y = world(a, d)
                    if free(x, y, 0.25) and emit(_pick(shrubs, rng), x, y,
                                                 rng.uniform(0, 360), "plant",
                                                 budget.other):
                        n_side_shrub += 1

        # -- rear: massed along the boundary, canopy over the middle -------
        # The boundary row now follows the REAL rear lot line and spans the
        # REAL lot width. Both were constants (16 m back, +-half the HOUSE
        # width) and both were wrong: lots plat 21-40 m deep, so a flat 16 m
        # threw the row over the back fence on the shallow half of them, and
        # +-half_w is the house, not the lot, so the row stopped short of both
        # side boundaries on every lot wider than its building.
        step = max(1.0, float(cfg.get("rear_boundary_step_m", 4.5)))
        depth = rear_line - inset
        if shrubs and depth > half_d + clear_house:
            a = -half_lot + inset
            while a <= half_lot - inset:
                x, y = world(a, depth)
                if free(x, y, 0.25):
                    emit(_pick(shrubs, rng), x, y, rng.uniform(0, 360),
                         "plant", budget.other)
                a += step

        # -- rear: the patio set and the shed, which is where they belong ---
        # These were only ever reachable through the kerb slot above, which is
        # how a garden table and a shelter ended up on the road.
        if patio_props and depth > half_d + clear_house + 2.0:
            px = rng.uniform(-half_lot * 0.5, half_lot * 0.5)
            py = half_d + (depth - half_d) * rng.uniform(0.35, 0.65)
            x, y = world(px, py)
            if free(x, y, 1.4):
                emit(_pick(patio_props, rng), x, y,
                     yaw_deg + rng.choice([0.0, 180.0]), "plant", budget.other)

        # Canopy over the back garden. The count comes off the AREA the lot
        # actually has behind the house — clamped by `rear_trees` at both ends —
        # so the deep estate garden gets its four and the shallow tight one
        # still gets its one. The 0.75 coin flip for ONE tree that this replaces
        # left a quarter of the gardens with no canopy and the rest with a
        # single trunk.
        b_lo = half_d + clear_house
        b_hi = rear_line - inset
        if trees and b_hi - b_lo >= 1.0 \
                and rng.random() < float(cfg["rear_tree_chance"]):
            usable = (b_hi - b_lo) * max(0.0, 2.0 * (half_lot - inset))
            want = int(max(rear_lo, min(rear_hi, round(usable / rear_area))))
            for i in range(want):
                for _d in range(darts):
                    a = rng.uniform(-half_lot + inset, half_lot - inset)
                    d = rng.uniform(b_lo, b_hi)
                    if plant_tree(a, d, clear_house):
                        n_rear_tree += 1
                        break

    stats = {"lots": len(houses), "placed": len(out), "tally": tally,
             "points": budget.spent, "budget": budget.total,
             "refused": budget.refused,
             # The canopy purse on its own, because it is the one that used to
             # bind. Instanced it no longer does: ~5,000 trees cost 2.9M of an
             # 18.6M share, and a refusal is a lot early in the shuffle whose
             # cap has not accumulated yet.
             "tree_points": budget.tree.spent, "tree_budget": budget.tree.total,
             "tree_refused": budget.tree.refused,
             "front_trees": n_front_tree, "side_trees": n_side_tree,
             "side_shrubs": n_side_shrub, "rear_trees": n_rear_tree,
             # Two different refusals, kept apart: a strip too narrow for a
             # canopy at all, against one that had the room but no spot left
             # once the buildings, the fences and the neighbours' trees were
             # taken out. The first is geometry, the second is crowding.
             "side_no_room": n_side_narrow, "side_no_spot": n_side_blocked,
             # Props refused for standing on a cul-de-sac turnaround. Worth
             # reporting rather than swallowing: a non-zero number here with a
             # zero disc list would mean the caller forgot to pass them.
             "keepout": n_keepout}
    return out, stats


def report(stats):
    t = "  ".join(f"{k}={v}" for k, v in sorted(stats["tally"].items()))
    pct = 100.0 * stats["points"] / max(1.0, stats["budget"])
    lots = max(1, stats["lots"])
    canopy = stats["tally"].get("tree", 0)
    print(f"[yardplan] {stats['placed']} placed across {stats['lots']} lots\n"
          f"[yardplan]   {t}\n"
          f"[yardplan]   {canopy / lots:.2f} trees per lot "
          f"({stats.get('front_trees', 0)} front / "
          f"{stats.get('side_trees', 0)} side / "
          f"{stats.get('rear_trees', 0)} rear); "
          f"{stats.get('keepout', 0)} off turnarounds, "
          f"{stats.get('side_no_room', 0)} side strips too narrow, "
          f"{stats.get('side_no_spot', 0)} with no spot left\n"
          f"[yardplan]   {stats['points']:,} of {int(stats['budget']):,} points "
          f"({pct:.0f}%), {stats['refused']} refused on budget "
          f"({stats.get('tree_points', 0):,} of "
          f"{int(stats.get('tree_budget', 0)):,} of it canopy)")
