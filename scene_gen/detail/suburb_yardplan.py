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
    # -- A BACK YARD IS ENCLOSED BY A FENCE OR BY A TREELINE ------------------
    # ...or it is open ground. Only the first half of that was reachable. On
    # seed 3 of the shipped preset, before this row existed, of the 363 lots
    # that have a real back garden (>= `patio_min_rear_yard_m` behind the back
    # wall):
    #
    #     fenced closed   82        screened by trees    5
    #     part-screened   57        wide open          219
    #
    # judging a yard enclosed when all three of its edges are at least 60%
    # within reach of a fence module or of a canopy, sampled every metre —
    # which is `suburb_scene._yard_enclosed`, and is now what asks.
    #
    # Five. Not because trees are scarce — this pass plants 2,637 of them —
    # but because none of them is planted AS A BOUNDARY. The rear canopy slot
    # throws 2-4 darts at the middle of the back garden, which is a copse and
    # never an edge, and the one row that does walk the real rear lot line
    # (`rear_boundary_step_m`, below) plants SHRUBS, which are off by default
    # and would not screen anything if they were on.
    #
    # So this is that row, planted with trees, along the three edges that
    # actually enclose a back yard: both side lines from the BUILDING LINE
    # back, and the rear line. Those three are exactly what a fence encloses
    # when it closes — `suburb_scene._trim_to_building_line` cuts a side run
    # to the front of the house for the same reason — so a screened yard and
    # a fenced one come out the same shape and one predicate downstream can
    # read either.
    #
    # Share of the ELIGIBLE lots: no fence of their own, a back garden deep
    # enough to be worth screening, and a side line that is not inside their
    # own building. On seed 3 that population is 285 of 479 lots, and this
    # pass is deliberately a MINORITY of it. A suburb where every unfenced
    # back garden is walled in trees is as wrong as one where none are, and it
    # is worse for what comes after: an enclosed yard is what qualifies a lot
    # for SEATING, so this number is also how much garden furniture the suburb
    # grows. At 0.40 the roll takes 105 lots and 120 back yards come out
    # screened by `suburb_scene._yard_enclosed` — the extra fifteen are
    # neighbours picking up a shared side line off somebody else's row, which
    # is what a boundary planting does. Against 72 closed by fence, that is
    # rather more than one screened garden per fenced one, and it still leaves
    # 165 of the 407 wide open and 50 part-screened, which is the point. (Those
    # four counts move by a handful run to run as the fence pass changes what
    # it manages to close; the RATIO is the stable part and is what this knob
    # sets.)
    #
    # It is also the whole cost of this slot: a screen is ~10 trunks, so the
    # roll is ~1,000 trees on top of the ~2,600 the rest of this pass plants.
    # Doubling the share doubles that, and the canopy is the only thing in the
    # yard pass with a real point cost.
    "screen_chance": 0.40,
    # SPACING IS DERIVED FROM THE TREE ACTUALLY DRAWN, not from this knob.
    # `yard_trees` spans 3.02 m of crown (Douglas_Fir) to 25.42 m (Black_Oak)
    # and the draw is weighted, so the measured canopy RADIUS in this scene is
    # p10 1.51 m / median 1.51 m / p90 5.24 m — a factor of three and a half.
    # Any single number is therefore wrong twice over: at 4.5 m the oaks stand
    # in each other and the firs leave a 1.5 m hole between every pair, which
    # is a row of trees and not a screen. So the walk advances by the crown it
    # just planted plus the crown it is about to plant (see `_crown_radii`),
    # which closes canopy-to-canopy whatever the draw hands it.
    #
    # This knob is what is left when that cannot be done: a run with no
    # resolver, or `measure_usds: false` with the assets unreachable, where
    # every entry answers with the one `fallback_sizes.tree` and a "derived"
    # step is a fiction. It is also the advance used to step PAST a refused
    # station, where there is no drawn crown to measure. 4.5 m because that is
    # what the shrub boundary row above is spaced at and two boundary rows in
    # one module should not disagree about what a boundary row is, and because
    # it clears `tree_min_separation_m` (3.0) — a fallback step at or under
    # the separation floor is a row the spacing grid deletes every other tree
    # from.
    "screen_step_m": 4.5,
    # How far inside the boundary the trunks stand. Held off the line for the
    # reasons `lot_inset_m` (0.9) gives — the survey line, and the neighbour's
    # garage, which is allowed to sit hard on it — and no further, because a
    # screen that stands back from the line stops screening: the strip you
    # left is the gap you can see through from next door, and the two
    # neighbours' rows on a shared boundary then sit far enough apart that
    # `tree_min_separation_m` no longer merges them into the one treeline a
    # shared line should be. 1.0 m is that inset plus a decimetre for a trunk,
    # which is thicker than the shrub the general inset was written for.
    "screen_inset_m": 1.0,
    # -- THE PATIO SET NEEDS A BACK GARDEN TO STAND IN ------------------------
    # How much open ground there has to be BEHIND THE BACK WALL, inside the lot
    # inset, before the lot is offered patio furniture at all. The test used to
    # be a bare `+ 2.0` on top of `clear_house_m`, i.e. 2.6 m — narrower than
    # the table-and-chairs set itself (2.43 m measured), so the furniture was
    # admitted to gardens that could not hold it and ended up half inside the
    # house or over the rear fence. Worse, `lot_depth` is platted per block and
    # the house is sized from the kit independently of it, so a measured 7.7% of
    # lots on the shipped preset (seed 1) have their rear lot line BEHIND their
    # own back wall — those houses have no back garden at all, and a table and
    # chairs on one stood in the neighbour's garden or in the next street.
    #
    # 4.0 m is a garden rather than a fit: the widest prop plus its stand-off
    # off the wall (`clear_house_m` 0.6 + 2.62 = 3.22 m) would technically fit
    # at 3.3, but a dining set filling a strip wall-to-fence, with no lawn left
    # either side of it, reads as a mistake from the air rather than a garden.
    # The FIT is enforced separately and from the MEASURED footprint of the prop
    # actually drawn, so this knob only has to say how much garden is enough.
    "patio_min_rear_yard_m": 4.0,
    # FLOOR under the measured extent of a patio prop, not merely a fallback.
    # It has to be a floor because an unmeasured run is not detectable from the
    # answer: with `measure_usds: false` the resolver returns the generic
    # `fallback_sizes.plant` (1.0 m) for every prop in the pool, which is a
    # perfectly plausible number and a third of the truth, so a plain fallback
    # would silently admit every tight garden. 2.62 m is the widest thing the
    # shipped pool holds (ParkBench01); the next widest is the table set at
    # 2.43, so on a measured run this floor costs 0.19 m of clearance and on an
    # unmeasured one it is what keeps the furniture out of the strip gardens.
    "patio_extent_fallback_m": 2.62,
    # -- ...AND IT HAS TO BE AN ARRANGEMENT, NOT A PROP -----------------------
    # What stood here was one item drawn from `{ParkBench01, Kalmar_TblChr}`,
    # dropped at `rng.uniform` across the whole rear band, turned by
    # `rng.choice([0.0, 180.0])`. Three separate randomnesses and no
    # composition: a bench alone in the middle of a lawn, at an angle to
    # everything, is what the scene actually contained 358 times.
    #
    # A patio is a place, not an object. It sits ON the ground behind the back
    # door, offset to one side of the wall rather than centred on it, with the
    # table set squared to the house and the seat in a fixed relation to the
    # table. So the group is composed in the lot's own (along, deep) frame,
    # anchored on the back wall, and placed as ONE footprint under ONE yaw.
    #
    # Walking room between the table set and the bench, edge to edge. It is
    # what makes the two read as a pair rather than as two props that happen to
    # be near each other: under ~0.5 m they touch and the bench looks shoved
    # against the chairs, over ~1.5 m the gap is wider than the bench is deep
    # and the pair falls apart into two objects again. It is also what the
    # group costs in garden depth — `clear_house` + table + this + bench is
    # 4.7 m on the measured pool, against `patio_min_rear_yard_m` of 4.0, so
    # every centimetre here is a shallow garden that gets the table alone.
    "patio_bench_gap_m": 0.9,
    # Where along the back wall the patio sits, as a fraction of the HALF-WIDTH
    # of the house — so it scales with the building instead of being a metre
    # count that is off-centre on a cottage and against the wall of a villa.
    # 0.0 centres the group on the back door, which is the one place a patio
    # never is; 1.0 puts it off the corner of the house entirely, with nothing
    # to sit against. 0.55 is a little past halfway along the wall, which is
    # where a slab gets laid: clear of the door, still under the eaves.
    "patio_side_off_frac": 0.55,
    # The bin by the back door. `yard_props` has carried a `bin`-tagged
    # TrashCan (0.51 m, 630 points) since the pool was written and NOTHING in
    # the generator has ever placed it. Not every household leaves it out at
    # the back — some keep it down the side or in the garage, neither of which
    # this pass models — so it is a coin weighted toward yes rather than a
    # fixture. At 1.0 every single back wall in the suburb has the same bin
    # against it, which reads as a repeated asset; at 0.0 the pool entry goes
    # back to being dead weight.
    "patio_bin_chance": 0.6,
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


# -- WHAT COUNTS AS AN ENCLOSED BACK YARD, MIRRORED FROM `suburb_scene` -------
# `suburb_scene` owns this question. It publishes `_YARD_SCREEN_COVER` (0.60),
# `_YARD_SCREEN_SLACK_M` (1.5) and `_YARD_SAMPLE_M` (1.0), writes the fence
# half of the answer onto every house record as `h["enclosure"]`, and strikes
# the three edges the answer is measured over as `h["rear_edges"]`. THOSE
# CONSTANTS ARE THE SOURCE OF TRUTH; these three are copies.
#
# Copied and not imported because the import would be a cycle: `suburb_scene`
# imports this module at module scope, so this module cannot import it back.
# The alternative — a fourth pass inventing its own bar for "screened" — is
# precisely the disease the enclosure work was written to cure, so the numbers
# are duplicated and labelled instead of re-derived. If they move over there,
# move them here.
_SCREEN_COVER = 0.60             # suburb_scene._YARD_SCREEN_COVER
_SCREEN_SLACK_M = 1.5            # suburb_scene._YARD_SCREEN_SLACK_M
_EDGE_SAMPLE_M = 1.0             # suburb_scene._YARD_SAMPLE_M


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


class _Fences:
    """EVERY fence module in the suburb, on a hash grid. Not just this lot's.

    `free()` used to ask `h["fence_segs"]` — this lot's own platted perimeter —
    and that was wrong in both directions at once:

        it missed the NEIGHBOUR'S fence.   A side boundary is shared and
            `suburb_parcel._relay` gives it to whichever lot was issued first,
            so the panels down this garden's left-hand side are usually the
            other lot's record. The screen row makes that bite: it deliberately
            plants a metre inside the boundary, which is exactly where the
            neighbour's fence stands.
        it invented fence THAT WAS NEVER BUILT. `fence_segs` is the plat's
            proposal and every cut happens downstream in `build_placements`
            without being written back, so a lot whose fence was trimmed away
            module by module still vetoed planting along a line with nothing
            on it.

    Both are the same fix: index what was actually DRAWN, over every house,
    once. `fence_drawn` when the record carries it, the plat when it does not —
    the same fallback and the same reason as the screen row's `fenced` test.

    Inserted by INFLATED BOUNDING BOX and queried in the point's own cell, the
    pattern `_Solids` uses: a span is up to 40 m long and a cell is 16, so a
    handful of cells hold it and a false positive costs one segment distance.
    """

    CELL = 16.0

    def __init__(self, houses, reach):
        self.reach = float(reach)
        self.g = {}
        self.n = 0
        for h in houses:
            spans = h.get("fence_drawn")
            if spans is None:
                spans = h.get("fence_segs")
            for s in (spans or ()):
                if len(s) >= 2 and s[0] and s[1]:
                    self._add(s[0], s[1])

    def _add(self, a, b):
        r = self.reach
        for gx in range(int(math.floor((min(a[0], b[0]) - r) / self.CELL)),
                        int(math.floor((max(a[0], b[0]) + r) / self.CELL)) + 1):
            for gy in range(int(math.floor((min(a[1], b[1]) - r) / self.CELL)),
                            int(math.floor((max(a[1], b[1]) + r) / self.CELL)) + 1):
                self.g.setdefault((gx, gy), []).append((a, b))
        self.n += 1

    def clear(self, x, y, pad):
        """Is (x, y) further than *pad* from every fence module in the scene?"""
        for (a, b) in self.g.get((int(math.floor(x / self.CELL)),
                                  int(math.floor(y / self.CELL))), ()):
            if _seg_dist(x, y, a, b) < pad:
                return False
        return True


class _Canopies:
    """The trees THIS PASS planted, as discs already grown by the screen slack.

    Only used to answer "is this edge screened", which is why it stores the
    inflated radius squared and nothing else: the question is never "how far to
    the nearest tree", it is "does anything reach this sample".

    A grid and not `suburb_scene._edge_cover`'s bounding-box prefilter, because
    the two are asked in opposite shapes. That one is handed the whole suburb
    per edge and rejects it per source; this one is built ONCE for ~3,500 trees
    and then asked ~30 samples x 3 edges x ~400 lots, where a per-call scan of
    the tree list is the ~10^8 tests that must not happen.
    """

    CELL = 8.0

    def __init__(self, slack=_SCREEN_SLACK_M):
        self.slack = float(slack)
        self.g = {}
        self.n = 0

    def add(self, x, y, r):
        rr = float(r) + self.slack
        for gx in range(int(math.floor((x - rr) / self.CELL)),
                        int(math.floor((x + rr) / self.CELL)) + 1):
            for gy in range(int(math.floor((y - rr) / self.CELL)),
                            int(math.floor((y + rr) / self.CELL)) + 1):
                self.g.setdefault((gx, gy), []).append((x, y, rr * rr))
        self.n += 1

    def covers(self, x, y):
        for (px, py, r2) in self.g.get((int(math.floor(x / self.CELL)),
                                        int(math.floor(y / self.CELL))), ()):
            if (x - px) ** 2 + (y - py) ** 2 <= r2:
                return True
        return False

    def __len__(self):
        return self.n


def _screen_cover(edges, canopies):
    """The WEAKEST of *edges*' screen covers, in ``[0, 1]``.

    A minimum and never a mean, for `suburb_scene._yard_enclosed`'s reason: two
    sides planted and the rear left open is not two thirds of an enclosure, it
    is a yard you walk into from the back. Sampled at `_EDGE_SAMPLE_M` and
    counted the same way that function counts, so "screened" means the same
    thing on both sides of the module boundary — the answer is only ever
    compared against `_SCREEN_COVER`, and a metre of resolution on a 10-40 m
    edge already decides that more finely than the decision needs.
    """
    if not edges:
        return 0.0
    worst = 1.0
    for (p0, p1) in edges:
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        ln = math.hypot(dx, dy)
        if ln < 1e-6:
            return 0.0
        n = max(2, int(math.ceil(ln / _EDGE_SAMPLE_M)) + 1)
        hit = 0
        for i in range(n):
            t = i / float(n - 1)
            if canopies.covers(p0[0] + dx * t, p0[1] + dy * t):
                hit += 1
        worst = min(worst, hit / float(n))
        if worst <= 0.0:
            break
    return worst


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


def _crown_radii(pool, resolver):
    """``{usd: half the MEASURED crown width}``, or ``{}`` when unmeasurable.

    The screen row's spacing is this and nothing else: it walks a boundary
    advancing by the crown it just planted plus the crown it is about to, so
    the canopies close whatever the weighted draw hands it. A fixed step
    cannot, because the pool is not one size — Douglas_Fir is 3.02 m across
    and Black_Oak 25.42 m, and both are common draws at the far end of a lot.

    Same key as `_canopy_ranks` and for the same reason: ``max(sx, sy)`` is
    crown WIDTH, which is what decides whether two neighbouring trees touch.
    Empty rather than a guess when there is no resolver — the caller then
    falls back to `screen_step_m`, which is honest about being a constant,
    where a fabricated radius would not be.
    """
    if resolver is None:
        return {}
    out = {}
    for e in pool:
        fp = resolver.get(e["usd"], "tree", scale=e.get("scale", 1.0),
                          axis_up=e.get("axis_up", "Z"))
        out[e["usd"]] = 0.5 * max(float(fp.get("sx", 0.0) or 0.0),
                                  float(fp.get("sy", 0.0) or 0.0))
    return out


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
    # SPLIT BY TAG. `yard_props` holds mailboxes, a bin, patio furniture and (it
    # used to) a shelter, and the kerb slot below drew from ALL of them — so two
    # thirds of the time the "mailbox at the kerb" was a table-and-chairs, a
    # garden bench or a 2.3 m transit shelter, standing in the street. Each now
    # goes where it belongs.
    #
    # `shed` is still read because the slot is still here; the shipped suburban
    # pool no longer STOCKS one (see the note in `asset_sets/suburban.yaml` —
    # the AEC MobilaShelter read as the bus stop it was authored as), so on that
    # pool this list is the two patio entries and nothing else. An empty tag
    # must stay empty: `_tagged` falls back to the whole pool when nothing
    # carries the tag, which is right for the mailbox slot and would put a bin
    # in the far corner of every garden here.
    _pp = lib.pool("yard_props")
    mailboxes = _tagged(_pp, "mailbox")
    patio_props = [e for e in _pp if "patio" in e["tags"] or "shed" in e["tags"]]
    # THE BIN, BY THE SAME LITERAL TEST AND NOT BY `_tagged`. `_tagged` falls
    # back to the WHOLE pool when nothing carries the tag, which is right for
    # the mailbox slot and would stand a table-and-chairs against the back wall
    # here. The suburban pool does carry one `bin` entry; a pool that does not
    # must place nothing, not something else.
    bin_props = [e for e in _pp if "bin" in e["tags"]]
    # MEASURED FOOTPRINT PER PROP, RESOLVED ONTO THE LOT'S OWN AXES.
    #
    # This used to be one number per prop, `max(sx, sy)`, and the reason was
    # sound at the time: the +-180 flip the old slot rolled per prop meant
    # nothing here could say which face would turn to the house, so a bench
    # 2.62 m one way and 0.79 m the other had to be refused any garden that
    # only held it end-on. The group has ONE yaw and it is the house's, so the
    # question is now answerable: `sx` runs along the frontage and `sy` runs
    # deep, unless the entry's own `yaw` correction turns the asset a quarter
    # turn, in which case they swap. Keeping the square would have cost the
    # composition its shape — the bench is 0.79 m deep and treating it as
    # 2.62 m deep adds 1.8 m to the depth every group needs, which is a large
    # share of the back gardens in the suburb.
    #
    # Category "plant" because that is what `emit` charges these as.
    # `patio_extent_fallback_m` is still a FLOOR, but it is now applied to the
    # GROUP's box rather than to each piece — see the note on it in DEFAULTS:
    # its job is to catch an unmeasured run, where every prop answers with the
    # one `fallback_sizes.plant` and a group would otherwise measure a metre
    # across and be admitted to gardens that cannot hold it.
    patio_fallback = float(cfg.get("patio_extent_fallback_m", 2.62))
    patio_min_yard = float(cfg.get("patio_min_rear_yard_m", 4.0))
    bench_gap = max(0.0, float(cfg.get("patio_bench_gap_m", 0.9)))
    patio_side_frac = float(cfg.get("patio_side_off_frac", 0.55))
    bin_chance = float(cfg.get("patio_bin_chance", 0.6))
    prop_axes = {}
    for _e in list(patio_props) + list(bin_props):
        _fp = (resolver.get(_e["usd"], "plant", scale=_e.get("scale", 1.0),
                            axis_up=_e.get("axis_up", "Z"))
               if resolver is not None else None)
        _sx = float((_fp or {}).get("sx", 0.0) or 0.0)
        _sy = float((_fp or {}).get("sy", 0.0) or 0.0)
        # A quarter-turn correction swaps the axes; anything else is a nudge
        # and leaves them where they are. Modulo 180 because a half turn maps
        # a rectangle onto itself.
        _yo = abs(float(_e.get("yaw", 0.0))) % 180.0
        _ea, _ed = ((_sy, _sx) if 45.0 <= _yo < 135.0 else (_sx, _sy))
        # NOTHING IS ZERO METRES ACROSS. With no resolver at all the answer is
        # 0.0 on both axes, and a zero here is a prop that clears every wall,
        # fits every garden and needs no room — the composition would collapse
        # onto a point. The smallest thing the pool stocks measures 0.51 m, so
        # half a metre is the floor below which an answer is a missing
        # measurement rather than a small object. The real protection against
        # an unmeasured run is `patio_extent_fallback_m` on the GROUP box.
        prop_axes[_e["usd"]] = (max(0.5, _ea), max(0.5, _ed))
    # THE CENTREPIECE AND THE SEAT, PICKED ONCE FOR THE WHOLE SUBURB rather
    # than drawn per lot, because the composition is a relation between two
    # SPECIFIC things and a random pair has no relation to compose. The table
    # set is whichever patio entry is squarest — `Kalmar_TblChr` is 2.43 x 2.42
    # and already a table WITH chairs, i.e. an arrangement in one asset — and
    # the seat is whichever is longest and thinnest, which is `ParkBench01` at
    # 2.62 x 0.79. Chosen by measurement and not by name so a pool that swaps
    # either asset still composes; a pool with only one patio entry makes it
    # the centrepiece and places no bench.
    def _oblong(e):
        a, d = prop_axes.get(e["usd"], (0.0, 0.0))
        return (max(a, d) / max(min(a, d), 1e-6))
    _sorted_patio = sorted(patio_props, key=_oblong)
    tbl_e = _sorted_patio[0] if _sorted_patio else None
    bench_e = _sorted_patio[-1] if len(_sorted_patio) > 1 else None

    # ROW HOMES ARE NOT PLANTED HERE, and the reason is the first line of this
    # module's docstring: every dimension is read off THE LOT, and an attached
    # row home does not have one. `suburb_parcel` gives a cluster unit a
    # `lot_corners` that is its footprint plus the footway strip and two metres
    # of bin store, because something downstream has to reserve that ground —
    # but there is no front yard to compose a foundation row across (the
    # footway to the court runs through it), no side yard between two units
    # 2.5 m apart, and no back garden at all. The ground behind and between the
    # rows is COMMUNAL: `row_housing` publishes it as the cluster's greens, and
    # `suburb_scene.build_open_planting` plants it as block interior, which is
    # what it is.
    #
    # They are still in `solids` below — a tree from a NEIGHBOURING lot must
    # not be planted inside one.
    all_houses = [h for p in parcels for h in p["houses"]]
    houses = [h for h in all_houses if not h.get("row")]
    n_row_skipped = len(all_houses) - len(houses)
    if not houses:
        return [], {"lots": 0, "row_skipped": n_row_skipped}
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
    # THE SCREEN ROW. Radii measured once for the whole suburb, exactly as the
    # ranking above is: both are properties of the pool, not of the lot.
    screen_chance = float(cfg.get("screen_chance", 0.40))
    screen_step = max(1.0, float(cfg.get("screen_step_m", 4.5)))
    screen_inset = float(cfg.get("screen_inset_m", 1.0))
    crown_r = _crown_radii(trees, resolver)

    # Every building in the suburb, before anything is planted — one pass over
    # the same `houses` list, so the neighbours are covered too.
    # EVERY house in the suburb, row homes included — see above: they are
    # excluded from being PLANTED, not from being planted AROUND.
    solids = _Solids(all_houses, pad_max=max(2.0, clear_house + 1.0),
                     rings=keepout_rings)
    # ...and every fence in the suburb, for the same reason and at the same
    # time: see `_Fences` on why asking one lot's own `fence_segs` was wrong in
    # both directions. Built from `all_houses` so a row home's fence counts too.
    fence_ix = _Fences(all_houses, clear_fence)
    # THE POOLS AGAIN, AT A MUCH BIGGER RADIUS AND FOR TREES ONLY.
    # `keepout_rings` already goes into `solids` above, which stops anything
    # being planted IN the water — a 2.6 m margin, the generic one a house
    # gets. That is the right number for a shrub and the wrong one for a tree,
    # because a tree in this dataset does not stay a tree: the wildfire pass
    # swaps every one for a baked burnt archetype carrying wood debris
    # scattered 7.5-10.5 m about its trunk (`disaster/vegetation.py`
    # `_DEBRIS`), so a trunk 2 m from the coping puts broken timber in the
    # water. Same index, same oriented boxes, same `clear()` — a second one at
    # the debris radius, consulted only by `plant_tree`.
    #
    # The knob lives under `suburb_parcel` with the other planting clearances
    # (`open_tree_clear_m`, `tree_clear_m`), because `build_open_planting` and
    # the final sweep in `suburb_scene` enforce the same distance and three
    # copies of it in three sections is three chances to disagree.
    pool_clear = float((config.get("suburb_parcel") or {})
                       .get("pool_tree_clear_m", 11.0))
    pool_water = (_Solids((), pad_max=max(pool_clear, 0.1),
                          rings=keepout_rings)
                  if (pool_clear > 0.0 and keepout_rings) else None)

    out = []
    placed_trees = _Spacing(tree_sep)
    # EVERY TREE THIS PASS PLANTS, AS A DISC, for the seating gate. Kept as a
    # flat list and indexed after the planting loop rather than as it grows:
    # the whole point of the second pass is that a lot's screen is often partly
    # its NEIGHBOUR's row, and an index consulted while it is still being
    # filled answers about a half-planted suburb.
    canopies = []
    tally = {}
    n_side_tree = n_side_shrub = n_rear_tree = n_front_tree = 0
    n_keepout = 0
    # Normalised once: `((x, y), r)` in world metres, same shape
    # `suburb_parcel` takes.
    _discs = [((float(c[0]), float(c[1])), float(r))
              for (c, r) in (keepout_discs or ())]
    n_side_narrow = n_side_blocked = n_pool_tree = 0
    # The patio slot, split four ways so a drop in the furniture count says
    # WHICH question refused it: no back garden at all, a garden too small for
    # the prop drawn, or a garden with something already standing in the spot.
    n_patio = n_patio_nogarden = n_patio_nofit = n_patio_blocked = 0
    # The seating gate, split by WHY. `patio_open` is the headline and the
    # whole reason this phase exists: it was 294 of 358 groups standing in a
    # garden open to the block behind it, and it has to read 0. `seat_fenced`
    # and `seat_screened` are the two ways a yard qualifies, reported apart
    # because they are different mechanisms and either one silently failing
    # would otherwise hide inside the total.
    n_patio_open = n_patio_noedges = n_seat_fenced = n_seat_screened = 0
    n_patio_pieces = n_patio_solo = n_bin = 0
    # What the whole-suburb fence index changed, both ways. `fence_neighbour`
    # is the bug it was built for: a station refused by a fence that is not
    # this lot's, which the old per-lot test waved through. `fence_ghost` is
    # the other direction — a station the plat's untrimmed proposal used to
    # veto with nothing actually standing there.
    n_fence_nb = n_fence_ghost = 0
    # Lots carrying a back garden, stashed by the planting loop for the seating
    # pass. Frames are STASHED AND NOT RE-DERIVED: four passes re-deriving "the
    # rear yard" three different ways is the defect this work exists to remove,
    # and a fifth derivation inside this same module would be the worst of
    # them.
    seat_jobs = []
    # The screen row, split the same way. `screen_eligible` is the population
    # the chance rolls against — unfenced lots with a back garden — so a
    # screen count that looks thin says whether the ROLL or the GROUND refused
    # it, and `screen_blocked` says how much of a rolled row the walls, the
    # neighbours' canopies and the purse ate.
    n_screen_lot = n_screen_elig = n_screen_tree = n_screen_blocked = 0

    def emit(entry, x, y, yaw, category, purse, prop_kind=None):
        """Charge the purse, then place. Returns False if unaffordable.

        `prop_kind` NAMES WHAT THE THING ACTUALLY IS, and it exists because
        `category` cannot. Everything this pass places is charged to the
        `plant` budget — that is what the purse is denominated in — so a
        mailbox, a wheelie bin and a patio table all ship with
        `category: "plant"`, and a downstream pass looking for street furniture
        finds a shrub. The tornado corridor pass is exactly that reader: it
        blows fences, bins, mailboxes and signs out of the track and left every
        mailbox on the plate standing perfectly upright in a levelled block,
        because none of them said "mailbox" anywhere. This does, without
        moving a single prop between budgets.

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
            "prop_kind": prop_kind,
        })
        tally[category] = tally.get(category, 0) + 1
        return True

    def free_at(x, y, pad, own=()):
        """Nothing built here, and no fence ANYWHERE IN THE SUBURB through it.

        *own* is this lot's platted perimeter and is used for NOTHING but the
        two counters: it is what the old per-lot test asked, so comparing the
        two answers is what says how much the whole-suburb index moved. The
        decision is `fence_ix`'s alone.
        """
        nonlocal n_fence_nb, n_fence_ghost
        if not solids.clear(x, y, pad):
            return False
        hit_own = any(len(sg) >= 2
                      and _seg_dist(x, y, sg[0], sg[1]) < clear_fence
                      for sg in own)
        if not fence_ix.clear(x, y, clear_fence):
            if not hit_own:
                n_fence_nb += 1
            return False
        if hit_own:
            n_fence_ghost += 1
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
            """This lot's view of `free_at`. See it, and `_Fences`."""
            return free_at(x, y, pad, fences)

        def plant_tree(along, deep, pad, entry=None):
            """Emit a tree at a local-frame point if everything allows it.

            SIZE COMES FREE HERE. `along`/`deep` are already in the house's own
            frame, so the distance from the wall is two subtractions against the
            half-footprint -- no index, no neighbour search, and it is the right
            house by construction: the tree belongs to this lot.

            Returns THE ENTRY PLANTED, or None. It used to return a bool and
            every caller still reads it as one, which is why this is safe; the
            screen row needs to know which species landed, because its next
            station is one crown further on and the crowns differ by a factor
            of eight. *entry* is the other half of that: a caller that has
            already drawn a species — because it had to, to know how far to
            step — hands it in rather than having a second one drawn here.
            """
            x, y = world(along, deep)
            if not placed_trees.ok(x, y) or not free(x, y, pad):
                return None
            # A TREE ONLY. The station is not re-tried anywhere else: a lot
            # with a pool in it has less plantable rear yard than one without,
            # which is the correct answer and is why the count drops slightly
            # on pool lots and nowhere else.
            if pool_water is not None and not pool_water.clear(x, y, pool_clear):
                nonlocal n_pool_tree
                n_pool_tree += 1
                return None
            if entry is None:
                wall = math.hypot(max(abs(along) - half_w, 0.0),
                                  max(abs(deep) - half_d, 0.0))
                entry = _pick(trees, rng, ranks=t_ranks,
                              t=_size_t(wall, t_near, t_far), sharp=t_sharp)
            if emit(entry, x, y, rng.uniform(0, 360), "tree", budget.tree):
                placed_trees.add(x, y)
                # THE RADIUS THE SCREEN WALK ASSUMED, recorded with the trunk.
                # The seating gate has to ask the same question the screen row
                # answered — does this canopy reach the boundary — so it must
                # use the same crown, not a second estimate of it.
                canopies.append((x, y, max(crown_r.get(entry["usd"],
                                                       screen_step / 2.0),
                                           tree_sep / 2.0)))
                return entry
            return None

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
                 budget.other, prop_kind="mailbox")

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
        # EVERYTHING THAT PLANTS RUNS BEFORE ANYTHING THAT FURNISHES, and the
        # ordering is load-bearing rather than tidy. The patio slot used to sit
        # between the boundary row and the canopy, which put the one decision
        # that has to know whether the garden is enclosed AHEAD of the passes
        # that enclose it — a treeline planted afterwards cannot qualify a seat
        # that is already standing. It has since gone further than a reorder:
        # seating is a SECOND PASS over the whole suburb, after this loop, for
        # the reason set out where it lives. Nothing in this loop furnishes
        # anything any more, and the ordering above survives so that the canopy
        # and the screen still take their ground before the seating pass asks
        # `free()` for a patio.
        #
        # The boundary row now follows the REAL rear lot line and spans the
        # REAL lot width. Both were constants (16 m back, +-half the HOUSE
        # width) and both were wrong: lots plat 21-40 m deep, so a flat 16 m
        # threw the row over the back fence on the shallow half of them, and
        # +-half_w is the house, not the lot, so the row stopped short of both
        # side boundaries on every lot wider than its building.
        step = max(1.0, float(cfg.get("rear_boundary_step_m", 4.5)))
        depth = rear_line - inset
        rear_yard = depth - half_d          # behind the back wall, inside inset
        if shrubs and depth > half_d + clear_house:
            a = -half_lot + inset
            while a <= half_lot - inset:
                x, y = world(a, depth)
                if free(x, y, 0.25):
                    emit(_pick(shrubs, rng), x, y, rng.uniform(0, 360),
                         "plant", budget.other)
                a += step

        # -- rear: THE SCREEN — a treeline on the lots with no fence --------
        # The row above, planted with trees instead of with shrubs that are
        # switched off, and carried round the two side lines as well: see
        # `screen_chance` in DEFAULTS for why it is those three edges and not
        # the back garden's own three.
        #
        # WHETHER THE LOT IS FENCED IS ASKED OF `fence_drawn`, NOT `fence_segs`.
        # `fence_segs` is the perimeter the PLAT proposed, and every cut that
        # shortens or deletes it happens downstream inside `build_placements`
        # — `_trim_to_building_line`, `_trim_offroad`, `_fence_run` returning
        # nothing, the `_FenceGrid` clash trim — with none of them written back
        # onto the record. So a lot whose fence was taken away module by module
        # still carries a full rectangle in `fence_segs`, and screening off
        # that refuses a treeline to gardens with nothing at all standing round
        # them. `fence_drawn` is what actually survived. Read it when it is
        # there and fall back to the plat when it is not — an older record, or
        # a caller planning before any placement exists — where over-reporting
        # the fence is at least the safe direction: it screens too few gardens
        # rather than enclosing one twice.
        _drawn = h.get("fence_drawn")
        fenced = bool(fences if _drawn is None else _drawn)
        # A side line needs somewhere to stand. `half_lot` is floored at
        # `half_w + 0.4` for lots whose platted frontage is narrower than their
        # own building, and on those the side line runs THROUGH the house: every
        # station would be refused by `free()` and the roll would be spent on a
        # lot that could never have been screened. Asked here rather than
        # inside the walk so `screen_eligible` means what it says.
        side_room = (half_lot - screen_inset) - (half_w + clear_house)
        if trees and not fenced and rear_yard >= patio_min_yard \
                and side_room > 0.0:
            n_screen_elig += 1
            if rng.random() < screen_chance:
                n_screen_lot += 1
                a_e = half_lot - screen_inset
                d_e = rear_line - screen_inset
                runs = []
                for sgn in (-1.0, 1.0):
                    d0 = -half_d                       # the building line
                    if gar is not None and sgn * g_along > 0.0:
                        # THE CAR'S SIDE. `suburb_parcel` reserves the box and
                        # the drive runs up to the front of it; nothing in this
                        # pass is told where that paving is, and the side-yard
                        # slot above dodges it the same way — by starting
                        # behind the box. So this side of the screen begins at
                        # the back of the garage. On the 59 of 479 lots (seed
                        # 3) that reserve one, that edge then falls short of
                        # the building line and the yard reads part-screened
                        # rather than screened, which is the truth about it:
                        # you can walk up the drive straight into the garden.
                        d0 = max(d0, g_deep + g_hd + clear_house)
                    runs.append((sgn * a_e, d0, sgn * a_e, d_e))
                # ...and the rear line joins their far ends. The corners are
                # shared deliberately and left to `tree_min_separation_m` to
                # arbitrate: two runs meet there wanting the same three metres
                # of ground and exactly one of them should get it.
                runs.append((-a_e, d_e, a_e, d_e))
                for (a0, d0, a1, d1) in runs:
                    L = math.hypot(a1 - a0, d1 - d0)
                    if L < 1.0:
                        continue
                    ua, ud = (a1 - a0) / L, (d1 - d0) / L
                    s = 0.0                 # how far the canopy line reaches
                    while s < L:
                        # THE SPECIES IS DRAWN BEFORE THE STATION IS FIXED, and
                        # that inversion is the whole mechanism: `s` is where
                        # the last crown ended, so the next trunk belongs one
                        # of ITS OWN radii further on, and the radius is a
                        # property of the draw. Picking a station first and
                        # then a tree gives back the fixed step this exists to
                        # avoid. `t` is taken at the frontier rather than at
                        # the station it settles on — a radius apart, against
                        # an 18 m ramp, which is inside the noise.
                        wall = math.hypot(max(abs(a0 + ua * s) - half_w, 0.0),
                                          max(abs(d0 + ud * s) - half_d, 0.0))
                        e = _pick(trees, rng, ranks=t_ranks,
                                  t=_size_t(wall, t_near, t_far), sharp=t_sharp)
                        # Half the fallback step when the pool was never
                        # measured, and never under half the separation floor:
                        # below that the walk lays a row `_Spacing` deletes
                        # every other tree from and then reports as blocked.
                        r = max(crown_r.get((e or {}).get("usd"),
                                            screen_step / 2.0), tree_sep / 2.0)
                        sc = min(s + r, L)
                        if plant_tree(a0 + ua * sc, d0 + ud * sc,
                                      clear_house, entry=e) is not None:
                            n_screen_tree += 1
                            s = sc + r
                        else:
                            # No crown was planted, so there is no crown to
                            # measure a step from: walk on by the constant and
                            # try again. The ground may open up two metres
                            # further along — a neighbour's canopy is a disc,
                            # not a wall.
                            n_screen_blocked += 1
                            s += screen_step

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

        # -- rear: the patio set, which needs a back garden to stand in -----
        # These were only ever reachable through the kerb slot above, which is
        # how a garden table and a shelter ended up on the road.
        #
        # A HOUSE WITH NO BACK GARDEN GETS NO PATIO FURNITURE. The old test was
        # `depth > half_d + clear_house + 2.0` — 2.6 m of ground behind the back
        # wall, less than the 2.43 m the table set is across — and it admitted
        # 427 of 479 lots on the shipped preset (seed 1), 82 of them with under
        # 5 m of garden and 37 with a platted rear lot line BEHIND their own
        # back wall. `lot_depth` is sampled per block and the house is sized
        # from the kit independently of it, so those lots are not a bug in the
        # plat; they are houses that genuinely have no back garden, and a dining
        # set on one stood in the neighbour's plot or in the next street.
        #
        # So the gate is now two separate questions, and both are asked against
        # the REAL prop rather than a constant:
        #   is there a garden at all      rear_yard >= patio_min_rear_yard_m
        #   does this prop fit in it      wall clearance + its measured extent,
        #                                 deep AND across the frontage
        # and the group is then anchored in the band where the FOOTPRINT clears
        # both the back wall and the rear lot line — the old code placed a
        # centre point and tested it with a flat 1.4 m pad, which is why a
        # 2.6 m prop could still straddle the fence in a garden that passed.
        #
        # NOTHING IS PLACED HERE ANY MORE. The lot's frame is stashed and the
        # seating runs in a second pass over the whole suburb — see the block
        # after this loop for why it has to.
        if patio_props and rear_yard >= patio_min_yard:
            seat_jobs.append({
                "h": h, "c": (cx, cy), "u": u, "n": n, "fences": fences,
                "half_w": half_w, "half_d": half_d, "half_lot": half_lot,
                "yaw": yaw_deg, "depth": depth,
                # Which side the drive is, or None. The patio goes on the OTHER
                # one: the back door is not the garage door.
                "gar_along": (g_along if gar is not None else None)})
        elif patio_props:
            n_patio_nogarden += 1

    # ---- SEATING: A COMPOSED GROUP, AND ONLY IN AN ENCLOSED BACK YARD ------
    # The rule, in the user's words: only trees in unfenced back yards; seating
    # if the back is somewhat fenced BY TREES; nothing for completely open
    # backs; and the seating has to be an arrangement, not random.
    #
    # WHY A SECOND PASS. A lot's screen is very often not a lot's own trees. A
    # side boundary is shared, and the screen row plants a metre inside it, so
    # the row that closes this garden's left-hand side belongs to the neighbour
    # — who, in a single loop, has not been planted yet when this lot is asked
    # whether it is enclosed. `build_placements` hit exactly this asking the
    # same question about fences and answered it the same way, and measured the
    # cost at one lot in 85 — small, because it walks lots in RING order and a
    # neighbour is nearly always issued next.
    #
    # THIS PASS DOES NOT WALK IN RING ORDER. It shuffles, deliberately (see the
    # note on `rng.shuffle` above: unshuffled, the purse spends the whole canopy
    # on the first streets laid), so the lot that plants the row down this
    # garden's side is at a uniformly random point in the order. Replaying the
    # emission order against an in-loop gate: it would have refused 12 of the
    # 118 screened lots on seed 3, 4 of 72 on seed 1 and 8 of 136 on seed 5 —
    # one in ten, not one in eighty-five. And every one of them is a bench
    # deleted because of the order the shuffle happened to produce, which is a
    # defect that cannot be reproduced from the seed and reads in a render as a
    # garden that is mysteriously bare. Here every tree in the suburb is
    # standing and the answer is a fact about the ground.
    #
    # THE GATE IS READ OFF THE RECORD, NOT RE-DERIVED. `h["enclosure"]` is the
    # fence half, computed in `build_placements` from the modules that actually
    # went down; `h["rear_edges"]` is the three boundaries the answer is
    # measured over, struck from the numbers the lot was ISSUED on. Both are
    # `suburb_scene`'s, and a fifth pass in this repo inventing its own rear
    # yard is the exact disease being cured. So a lot whose record does not
    # publish the edges gets NO SEATING rather than a locally invented
    # rectangle — `patio_no_edges` says loudly when that happens.
    #
    # ...with one exception that is not a fallback: a yard whose FENCE already
    # closed it needs no edges to prove it, because `build_placements` already
    # measured that over those same edges. Only the tree half needs them here.
    screen_ix = _Canopies(_SCREEN_SLACK_M)
    for (_cx, _cy, _cr) in canopies:
        screen_ix.add(_cx, _cy, _cr)
    tbl_a, tbl_d = prop_axes.get((tbl_e or {}).get("usd"), (0.0, 0.0))
    bch_a, bch_d = prop_axes.get((bench_e or {}).get("usd"), (0.0, 0.0))
    for job in seat_jobs:
        h = job["h"]
        # -- the gate -------------------------------------------------------
        if bool((h.get("enclosure") or {}).get("closed")):
            n_seat_fenced += 1
        else:
            edges = h.get("rear_edges") or ()
            if not edges:
                n_patio_noedges += 1
                continue
            if _screen_cover(edges, screen_ix) < _SCREEN_COVER:
                n_patio_open += 1
                continue
            n_seat_screened += 1

        cx, cy = job["c"]
        u, n = job["u"], job["n"]
        half_w, half_d = job["half_w"], job["half_d"]
        half_lot, depth, yaw_deg = job["half_lot"], job["depth"], job["yaw"]
        fences = job["fences"]

        def world(along, deep):
            return (cx + u[0] * along + n[0] * deep,
                    cy + u[1] * along + n[1] * deep)

        # -- the composition ------------------------------------------------
        # ONE YAW FOR THE WHOLE GROUP, and it is the house's. `yaw_deg` is the
        # frontage tangent — the mailbox slot above establishes the convention
        # — so at `yaw_deg` a prop's long axis runs parallel to the back wall,
        # which is how furniture is set out on a patio and is the entire
        # difference from `rng.choice([0.0, 180.0])` per prop. The bench takes
        # the half turn, so the two are mirror images about the group's centre
        # and read as a set from above.
        #
        # HONEST ABOUT WHAT IS NOT KNOWN HERE: which way a bench's seat looks
        # is a property of the asset's own facing axis, and nothing in the pool
        # records it — `entry["yaw"]` is the only correction available and both
        # patio entries carry none. Squaring both to the same wall makes the
        # pair read correctly either way; settling which of the two looks at
        # the house needs the asset on a stage.
        #
        # Offsets are measured from the TABLE's centre, and the table is
        # anchored hard against the back wall band, because that is where a
        # patio slab is laid. The bench is set BEYOND it, out toward the
        # garden: wall, table, seat, lawn.
        grp = [(tbl_e, 0.0, tbl_a, tbl_d, yaw_deg)]
        if bench_e is not None:
            grp.append((bench_e, tbl_d / 2.0 + bench_gap + bch_d / 2.0,
                        bch_a, bch_d, yaw_deg + 180.0))
        # -- does the GROUP fit, as one footprint ---------------------------
        # THE BOX, NOT THE PIECES. The old test asked whether one prop's extent
        # cleared the wall and the lot line; a pair set out deep needs the span
        # from the front of the table to the back of the bench, and testing the
        # bench alone is how a composed group ends up with its far half over
        # the rear boundary.
        #
        # A SHALLOW GARDEN GETS THE TABLE ON ITS OWN rather than nothing. The
        # pair needs `clear_house` + 2.42 + `patio_bench_gap_m` + 0.79 = 4.7 m
        # behind the back wall on the measured pool, against a
        # `patio_min_rear_yard_m` of 4.0 — so a band of real gardens can hold a
        # table set and not a group. It is a narrow band and it is not
        # theoretical: 5 of the 180 groups on seed 3 land in it, and 0 lots
        # fail to fit even the table. `Kalmar_TblChr` is a table WITH chairs,
        # which is already an arrangement, and it is still anchored and squared
        # rather than dropped at random: that is a smaller garden, not a
        # regression to the old behaviour.
        stations = None
        for members in ([grp, grp[:1]] if len(grp) > 1 else [grp]):
            # THE FALLBACK IS A FLOOR ON THE BOX, ON ALL THREE SIDES. On an
            # unmeasured run every piece answers with the same generic size and
            # the group would measure a metre square: it would be admitted to
            # gardens that cannot hold it, and — because the anchor is struck
            # off the box's own near face — stood half inside the back wall.
            ga = max(patio_fallback, max(m[2] for m in members))
            gd_lo = min(-patio_fallback / 2.0,
                        min(m[1] - m[3] / 2.0 for m in members))
            gd_hi = max(gd_lo + patio_fallback,
                        max(m[1] + m[3] / 2.0 for m in members))
            d_anchor = half_d + clear_house - gd_lo
            a_lim = half_lot - inset - ga / 2.0
            if d_anchor + gd_hi > depth or a_lim <= 0.0:
                continue
            # WHERE ALONG THE WALL. Off to one side, and to the side the car is
            # NOT on: `suburb_parcel` reserves the garage box and runs the drive
            # up to it, so the back door and the slab are on the other half of
            # the wall. With no garage there is no reason to prefer a side and
            # it is a coin flip, which is the one randomness a patio is allowed.
            gs = job["gar_along"]
            side = ((-1.0 if gs > 0.0 else 1.0) if gs is not None
                    else (1.0 if rng.random() < 0.5 else -1.0))
            a_anchor = min(max(side * half_w * patio_side_frac,
                               -a_lim), a_lim)
            stations = [(e, world(a_anchor, d_anchor + do), yw, ea, ed)
                        for (e, do, ea, ed, yw) in members]
            break
        if stations is None:
            n_patio_nofit += 1
            continue
        # EVERY PIECE CLEARS THE GROUND, not just the anchor. A group that
        # passes as a box can still have its bench inside the neighbour's fence
        # or under a screen tree planted an hour ago in pass one.
        if not all(free_at(st[1][0], st[1][1],
                           max(patio_fallback, st[3], st[4]) / 2.0, fences)
                   for st in stations):
            n_patio_blocked += 1
            continue
        # THE CENTREPIECE DECIDES. If the purse or a turnaround refuses the
        # table there is no group to hang a bench off, and a bench standing
        # alone at the far offset of a group that was never placed is worse
        # than the empty lawn it replaces.
        e0, (x0, y0), yw0, _a0, _d0 = stations[0]
        if not emit(e0, x0, y0, yw0, "plant", budget.other,
                    prop_kind="patio"):
            n_patio_blocked += 1
            continue
        n_patio += 1
        n_patio_pieces += 1
        for (e, (x, y), yw, _ea, _ed) in stations[1:]:
            if emit(e, x, y, yw, "plant", budget.other, prop_kind="patio"):
                n_patio_pieces += 1
        if len(stations) < 2:
            n_patio_solo += 1
        # -- and the bin by the back door -----------------------------------
        # Against the wall, on the house side of the patio — between the slab
        # and the back door rather than out in the garden, which is the one
        # place a wheelie bin never stands. It takes no part in the group's fit
        # test: at 0.51 m it is the smallest thing this pass places, it lives
        # inside the wall band the table has already cleared, and folding it
        # into the box would push the whole group a half metre further out for
        # an object that is allowed to be refused on its own.
        if bin_props and rng.random() < bin_chance:
            be = _pick(bin_props, rng)
            b_a, b_d = prop_axes.get((be or {}).get("usd"), (0.6, 0.6))
            b_lim = half_lot - inset - b_a / 2.0
            bx, by = world(min(max(a_anchor - side * (ga / 2.0 + 0.5
                                                      + b_a / 2.0),
                                   -b_lim), b_lim),
                           half_d + clear_house + b_d / 2.0)
            if b_lim > 0.0 \
                    and free_at(bx, by, max(b_a, b_d) / 2.0, fences) \
                    and emit(be, bx, by, yaw_deg, "plant", budget.other,
                             prop_kind="bin"):
                n_bin += 1
                n_patio_pieces += 1

    stats = {"lots": len(houses), "placed": len(out), "tally": tally,
             # Row homes seen and deliberately left alone. Reported so a run
             # whose yard count drops says WHY rather than looking thinned.
             "row_skipped": n_row_skipped,
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
             "keepout": n_keepout,
             # ...and tree stations refused for standing inside the burnt
             # archetype's debris radius of a pool. Same argument: zero here on
             # a preset that has pools means the rings never arrived.
             "pool_trees": n_pool_tree,
             # The patio slot. `patio_no_garden` is the headline: lots whose
             # rear lot line leaves less than `patio_min_rear_yard_m` behind the
             # back wall, i.e. houses that do not have a back garden to furnish.
             "patio": n_patio, "patio_no_garden": n_patio_nogarden,
             "patio_no_fit": n_patio_nofit, "patio_blocked": n_patio_blocked,
             # THE ONE NUMBER THIS PHASE IS JUDGED ON. Lots refused seating
             # for a garden that is neither fenced nor screened — not a failure
             # count but a WORKLOAD count, because it is exactly what the pass
             # used to furnish anyway. 294 of the 358 props on seed 3 stood in
             # an open back yard before any of the enclosure work; run the old
             # slot against today's fences and screens and it is still 169 of
             # 358, and this counter reads 183. A zero here alongside a zero
             # `patio` means the gate is stuck shut, not that every back garden
             # in the suburb is enclosed.
             "patio_open": n_patio_open,
             # ...and lots the gate could not judge because the record carries
             # no `rear_edges`. Should be 0 against a `suburb_scene` that
             # publishes them; anything else means this module and that one
             # have drifted apart and every unfenced garden in the suburb is
             # silently going without seating.
             "patio_no_edges": n_patio_noedges,
             "seat_fenced": n_seat_fenced, "seat_screened": n_seat_screened,
             # Props, not groups: a group is 2 or 3 of them. `patio_solo` is
             # the shallow gardens that took the table set alone.
             "patio_pieces": n_patio_pieces, "patio_solo": n_patio_solo,
             "bins": n_bin,
             # What the whole-suburb fence index moved, both ways — see
             # `_Fences`. `fence_neighbour` is the bug it fixes.
             "fence_neighbour": n_fence_nb, "fence_ghost": n_fence_ghost,
             # The screen row. `screen_eligible` is the denominator
             # `screen_chance` rolls against — unfenced lots with a back
             # garden and a side line clear of their own building — so a thin
             # screen count says whether the ROLL or the GROUND refused it.
             # `screen_blocked` is stations `free()`, `_Spacing` or the purse
             # turned down: a high number against a low `screen_trees` means
             # the boundary is already full, not that the walk is broken.
             "screen_lots": n_screen_lot, "screen_eligible": n_screen_elig,
             "screen_trees": n_screen_tree, "screen_blocked": n_screen_blocked}
    return out, stats


def report(stats):
    t = "  ".join(f"{k}={v}" for k, v in sorted(stats["tally"].items()))
    pct = 100.0 * stats["points"] / max(1.0, stats["budget"])
    lots = max(1, stats["lots"])
    canopy = stats["tally"].get("tree", 0)
    row = stats.get("row_skipped", 0)
    print(f"[yardplan] {stats['placed']} placed across {stats['lots']} lots"
          + (f" ({row} row homes skipped: shared ground, no private yard)"
             if row else "") + "\n"
          f"[yardplan]   {t}\n"
          f"[yardplan]   {canopy / lots:.2f} trees per lot "
          f"({stats.get('front_trees', 0)} front / "
          f"{stats.get('side_trees', 0)} side / "
          f"{stats.get('rear_trees', 0)} rear); "
          f"{stats.get('keepout', 0)} off turnarounds, "
          f"{stats.get('side_no_room', 0)} side strips too narrow, "
          f"{stats.get('side_no_spot', 0)} with no spot left, "
          f"{stats.get('pool_trees', 0)} inside a pool's debris radius\n"
          f"[yardplan]   {stats.get('screen_lots', 0)} boundary screens of "
          f"{stats.get('screen_eligible', 0)} unfenced lots with a back garden"
          f" ({stats.get('screen_trees', 0)} trunks, "
          f"{stats.get('screen_blocked', 0)} stations refused)\n"
          f"[yardplan]   {stats.get('patio', 0)} seating groups "
          f"({stats.get('patio_pieces', 0)} props: "
          f"{stats.get('seat_fenced', 0)} in a fenced back yard, "
          f"{stats.get('seat_screened', 0)} in a tree-screened one, "
          f"{stats.get('patio_solo', 0)} too shallow for the bench, "
          f"{stats.get('bins', 0)} bins)\n"
          f"[yardplan]   refused: {stats.get('patio_open', 0)} back yards "
          f"neither fenced nor screened, "
          f"{stats.get('patio_no_garden', 0)} with no back garden at all, "
          f"{stats.get('patio_no_fit', 0)} too small for the group, "
          f"{stats.get('patio_blocked', 0)} with the spot taken"
          + (f", {stats['patio_no_edges']} with no rear_edges on the record"
             if stats.get('patio_no_edges') else "") + "\n"
          f"[yardplan]   fences: {stats.get('fence_neighbour', 0)} stations "
          f"refused by a NEIGHBOUR's fence, {stats.get('fence_ghost', 0)} "
          f"freed from a platted one that was never built\n"
          f"[yardplan]   {stats['points']:,} of {int(stats['budget']):,} points "
          f"({pct:.0f}%), {stats['refused']} refused on budget "
          f"({stats.get('tree_points', 0):,} of "
          f"{int(stats.get('tree_budget', 0)):,} of it canopy)")
