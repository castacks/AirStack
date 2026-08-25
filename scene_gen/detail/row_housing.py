"""
row_housing.py — attached rows around a shared parking court, as a plat pass.

WHAT THIS IS, AND WHY THE PLAT NEEDS IT
---------------------------------------
`suburb_parcel` plats DETACHED suburbia: one dwelling per lot, a driveway per
door, side yards on both flanks and a back garden. That is one American
residential morphology and it is not the only one. The other common one —
garden apartments / attached townhouses — packs the dwellings onto a fraction
of the ground and SHARES everything a detached lot keeps private: no back
garden, no garage, no driveway per door, and one parking court in the middle
that every unit walks to.

It matters for this dataset rather than being an aesthetic choice, and the
reasons are the ones `cluster_housing_launch_script.py` was built to show:

  * Attached rows have NO defensible space between units. A detached plat
    loses houses in a scatter; a row loses the whole run, because the gap that
    would have stopped it is 2.5 m of party wall instead of 12 m of lawn.
  * The court concentrates the vehicles. In a detached plat the cars are one
    per driveway; here they are all in one place — the obvious refuge (bare
    asphalt, nothing on it to burn) and the obvious trap if the single access
    drive is blocked.
  * Everyone's route out is the same route. ONE drive serves the cluster.

THE REFERENCE, AND WHAT IS TAKEN FROM IT
----------------------------------------
`simulation/isaac-sim/launch_scripts/cluster_housing_launch_script.py` is a
hand-composed bench of exactly this morphology whose proportions were approved
on sight. Everything dimensional here comes off it:

    12.9 x 22.6 m `terrace` units, 2.5 m between party walls, 5 to a row
    two facing rows across one double-loaded court
    2.7 x 5.5 m bays, 7.0 m aisle          (`suburb_park.PARKING`)
    7.5 m from the court edge to the front wall
    ONE 8 m access drive
    a footway from the court to every door
    a shallow communal green behind each row
    NO garage, driveway, back garden or pool per unit

The bench also recorded the constraint that decides the party gap: this kit
models every unit as a FREE-STANDING SHELL with its own end walls, so units
cannot butt into a true shared wall without interpenetrating. 2.5 m is the
tightest that stays clean and still reads as a terrace.

WHAT IS DIFFERENT HERE: THE COURT RUNS INTO THE BLOCK, NOT ALONG THE STREET
---------------------------------------------------------------------------
The bench put its single drive at the WEST END of a court whose long axis ran
along the street. On a real block that end is another street's frontage or the
middle of the block, so the one thing the morphology turns on — a single drive
off a single street — would have had nowhere to come from.

So the court is turned ninety degrees: its aisle runs INTO the block on the
block's inward normal, the drive is simply that aisle continued out to the
kerb, and the two rows flank it left and right. From the street you see the
two rows' end walls with the court mouth between them, which is what a garden
apartment court actually presents to the road. Everything else — bay module,
party gap, setback, footways — is unchanged.

WHY A ROW UNIT IS STILL A `suburb_parcel` HOUSE RECORD
------------------------------------------------------
Every consumer downstream of the plat reads that record: `build_placements`
sites the asset off `size_index`, `modular_house.plan_lot` aims the walk off
`c`/`u`/`n`/`frontage`, `suburb_yardplan` composes in the (`u`, `n`) frame,
`build_cars` keeps cars off `corners`, `disaster.people` measures a front yard
as `c - n * (d/2 + k)`. A row unit that carried a shape of its own would need
every one of those to grow a special case. So it does not: `plan_cluster`
emits the SAME dict, with `garage: None`, no `fence_segs`, `has_pool: False`
and an empty `drives` list on the block, plus a `row` flag for the two passes
that genuinely must behave differently (the yard pass, which composes a
private garden a row unit does not have, and the ground pass, which draws the
court).

THE FRAME, AND WHY `u` IS DERIVED FROM `n` AND NOT THE OTHER WAY ROUND
----------------------------------------------------------------------
`modular_house.build_building` is placed at `yaw = degrees(atan2(u))` and
faces its front at local -Y, and local +Y at that yaw is `perp(u)` — the LEFT
normal. `plan_lot` meanwhile composes off the record's `n`. The two agree only
when `n == perp(u)`, which on a block face is true because block polygons are
CCW (`suburb_net.offset_polygon`: "CCW polygon: left is interior") and
`_inward` therefore always returns the left normal.

A row unit does not front a block face, so nothing enforces that for free.
Here the OUTWARD direction is what the layout knows — away from the court —
so `u` is derived from it as ``u = (n.y, -n.x)``, which makes ``perp(u) == n``
by construction. Getting this backwards mirrors every door and porch in the
row about the party wall, and the render is the only place it shows.
"""

import math

from layout.suburb_net import (_add, _mul, _unit, _dist, point_in_polygon,
                               polyline_length, point_at, tangent_at)
from detail.suburb_park import PARKING


# ---------------------------------------------------------------------------
# what the art really measures
# ---------------------------------------------------------------------------
# MEASURED FOOTPRINTS of the baked pristine archetypes
# (`scene_gen/assets/archetypes/house_<style>_pristine.usd`), W across the
# frontage x D into the lot, in metres.
#
# THESE ARE NOT `suburb_scene.modular_catalogue`'s NUMBERS, and the difference
# is the whole reason the table exists. That catalogue reports the kit's CELL
# FOOTPRINT — `modular_house.footprint()` times `CELL_M` — which is the wall
# grid: a cottage is 2x2 cells, so it reports 10.0 x 10.0. The asset that
# lands on the lot carries eaves, a porch and a bay on top of that grid and
# measures 11.1 x 12.8. On a detached lot the 1-3 m difference disappears into
# a 6 m side yard and nobody sees it. In a row at a 2.5 m party gap it is most
# of the gap, so the pitch has to be struck off what the asset really is or
# the units interpenetrate on screen having passed every test in the plat.
#
# The record still stamps `size_index` into the CATALOGUE, because that is what
# `build_placements` places; only the box this module reserves is measured.
MEASURED = {
    "cottage":    (11.1, 12.8),
    "two_storey": (11.1, 12.8),
    "wide_house": (16.1, 13.8),
    "ranch":      (21.1, 12.8),
    "terrace":    (12.9, 22.6),
    "villa":      (17.2, 12.6),
    "l_family":   (21.1, 22.8),
    "l_bungalow": (21.1, 22.8),
}


# ---------------------------------------------------------------------------
# which styles can be a row home, and which cannot
# ---------------------------------------------------------------------------
# A ROW MIX IS A DEVELOPMENT, NOT A SHUFFLE. Each cluster draws exactly one of
# these and every unit in it comes from that mix, so two row-home areas on the
# same map read as two different developments by two different builders rather
# than as one catalogue dealt twice. That is the same argument
# `suburb_parcel.ARCHETYPES` makes about runs along a block face, one scale up.
#
# WITHIN a mix the styles are chosen so the row still reads as one terrace:
# comparable depth (the FRONT wall is what lines up; see `plan_cluster`) and
# no style whose art carries a garage door.
#
# REJECTED, and why — recorded because three of the eight styles look admissible
# on width alone and are not:
#
#   villa       17.2 x 12.6. Width suits a row and depth suits a row. Its art
#               carries an INTEGRAL GARAGE DOOR on the front elevation
#               (`STYLES["villa"]["garage"] = 1`), and a row unit has no drive
#               — the door would open onto 7.5 m of lawn between it and the
#               court. Admitting it and drawing a private apron across the
#               footway was tried on paper and abandoned: an apron per unit is
#               a driveway per unit, which is the morphology this file exists
#               to be the opposite of.
#   ranch       21.1 x 12.8. 21 m of frontage per dwelling is a detached plan;
#               five of them make a 118 m row, longer than the frontage of most
#               blocks this generator produces. Also a garage in the art.
#   l_family    21.1 x 22.8, and `l_bungalow` the same. L-plans. The wing turns
#   l_bungalow  ninety degrees OUT of the row, so at a 2.5 m party gap it
#               stands in front of its neighbour's front wall and blocks that
#               unit's own footway. Also 21 m frontage; also a garage.
#
# Left admissible: cottage, two_storey, wide_house, terrace — all `garage: 0`.
MIXES = {
    # THE BENCH'S OWN MORPHOLOGY. 12.9 x 22.6 m two-storey bars, the only shape
    # this kit gets a continuous flat roof out of, with a shallower gabled
    # two-storey box breaking the run. The two differ by 9.8 m of depth and the
    # step lands at the BACK, facing the communal green — which is where a real
    # terrace is ragged (back extensions, different build phases) and never at
    # the front, because the front is the thing that has to line up.
    "terrace":  {"terrace": 0.78, "two_storey": 0.22},
    # SMALL UNITS. Identical 11.1 x 12.8 footprints, so the row is flush front
    # AND back; what varies is one storey against two, a picket fence against a
    # low stone wall, and the palette. This is the mix that proves the
    # variation is not coming from the footprint.
    "cottage":  {"cottage": 0.55, "two_storey": 0.45},
    # WIDER UNITS with narrow infill: 16.1 m against 11.1 m on a common pitch,
    # so the gap between neighbours visibly varies down the row and the wider
    # units read as the bigger flats they are. Depths are 13.8 and 12.8, a 1 m
    # step at the back.
    "garden":   {"wide_house": 0.60, "two_storey": 0.40},
}

# How often each mix is the one a row district draws. Flat by design: there is
# no measurement saying which of the three is commoner, and inventing a bias
# would be inventing a fact.
MIX_WEIGHTS = {"terrace": 1.0, "cottage": 1.0, "garden": 1.0}


# THE DEVELOPMENT'S COLOUR RANGE. `modular_house.apply_palette` rebinds the
# wall, gable and roof subsets of an already-built house, so a per-unit colour
# costs NO extra geometry and no extra bake — the wildfire pass's own note
# ("collapse geometry is independent of wall colour, so palette and scorch stay
# runtime binds") is the same fact from the other side.
#
# A SET PER CLUSTER, A COLOUR PER UNIT, AND NEIGHBOURS FORCED TO DIFFER. Drawn
# freely, a three-colour set puts the same colour on adjacent units about a
# third of the time, and two identical units side by side is exactly the thing
# that makes a row read as copy-paste — it is far more visible than two
# identical units four doors apart. The set is what keeps the run coherent:
# a development is painted from one range, not from the whole catalogue.
#
# `concrete` is deliberately absent: it is the commercial concrete with a worn
# asphalt roof, and it is not a colour anybody paints housing.
PALETTE_SETS = {
    "brick":  ("brick_red", "brick_buff", "stucco"),
    "timber": ("wood_white", "wood_dark", "siding_cream"),
    "render": ("stucco", "concrete_pale", "siding_cream"),
}
PALETTE_SET_WEIGHTS = {"brick": 1.0, "timber": 1.0, "render": 1.0}


DEFAULTS = {
    # -- the row ------------------------------------------------------------
    # Between party walls. Real attached townhouses share a wall outright; this
    # kit's units are free-standing shells with their own end walls, so butting
    # them together interpenetrates the geometry. 2.5 m is the closest that
    # stays clean and still reads as a terrace rather than as detached — the
    # bench's finding, and the number it was approved at.
    "unit_gap_m": 2.5,
    # Units per row. The bench used 5; 3 is where a "row" stops being one. The
    # planner takes the LARGEST that fits the block and falls back down this
    # range rather than shrinking the units.
    "units_per_row": [3, 5],
    # Court edge to front wall: a footway, a planted strip and a doorstep.
    # Under about 6 m the units read as fronting directly onto parked cars,
    # which is what a badly-designed one does. 7.5 is the bench's.
    "front_setback_m": 7.5,
    # WITHIN-ROW VARIATION, part one. Every unit's front wall on the identical
    # line is the other way a row reads as copy-paste; a metre of slop is what
    # a real terrace built in phases has. Added to `front_setback_m`, so it
    # only ever moves a unit AWAY from the court — the setback is a minimum.
    "setback_jitter_m": 1.2,
    # -- the court ----------------------------------------------------------
    # Bay and aisle come from `suburb_park.PARKING` rather than being repeated
    # here, because they are the regulation figures and not a property of
    # either scene. 2.7 x 5.5 m bays, 7.0 m aisle.
    "court_pad_m": 3.0,          # kerb strip at each end of the bay run
    # HOW MANY BAYS, and this is a SUPPLY number rather than a geometric one.
    # The bench painted bays down the whole length of both kerbs — 50 of them
    # for 10 units — because the court's length is set by the row and the row
    # is long. Five spaces per dwelling is a shopping centre, not housing: US
    # multifamily zoning almost universally asks 1.5-2.0 per unit, and a
    # garden-apartment court built to five would be mostly empty asphalt in
    # every frame. So the slab still runs the full length of the rows (the
    # aisle has to reach every footway) and only the middle stretch of each
    # kerb is marked out, which is also what one looks like from the air.
    # The geometric maximum still caps it, so a short court simply gets fewer.
    "bays_per_dwelling": 1.75,
    # Block boundary to the court's near end. The block boundary IS the kerb
    # line (`suburb_net.blocks_from_faces` insets every face by half its
    # carriageway) and `suburb_scene.build_frontage` lays its sidewalk 0.8 m
    # and its street furniture 1.6 m inside that — so the drive has to cross
    # about 2.5 m of public verge before it is on the cluster's own ground.
    # 6.0 m gives the apron a real length instead of dropping the court onto
    # the footway.
    "court_front_m": 6.0,
    # THE ONE WAY IN, and the whole point of the morphology. 8.0 m is the
    # bench's; it is also two 3.5 m lanes plus a margin, i.e. wide enough for
    # two cars to pass, which is what makes a single drive legal at all.
    "drive_w_m": 8.0,
    # -- the shared ground --------------------------------------------------
    # Communal green behind each row, where a detached plat would have put
    # private gardens. Deliberately shallow: the ground a garden apartment
    # gives back is amenity strip, not garden. Kept BELOW
    # `suburb_parcel.pool_rear_min_m` (9.5) on purpose, so `modular_house.
    # pool_at` refuses a pool on geometry as well as on the `has_pool` flag.
    "green_m": 6.0,
    "head_green_m": 5.0,         # ...and at the closed end of the court
    # The footway from the court kerb to a front door. Wider than the 1.2 m
    # `suburb_scene.WALK_W_M` front walk because this one is the shared route
    # to a block of front doors rather than one household's path.
    "walk_w_m": 1.6,
    # Ground reserved BEHIND a unit's back wall, inside its `lot_corners`. Two
    # metres is a bin store and a back step. Everything past it is the communal
    # green, and leaving that OUT of `lot_corners` is deliberate — see
    # `plan_cluster`, it is what lets `build_open_planting` put trees on it.
    "unit_rear_clear_m": 2.0,
    # -- how many, and where ------------------------------------------------
    # A big block can carry two developments side by side, each with its own
    # court and its own single drive. Two courts sharing one drive is NOT what
    # this does: the drive is the risk, and merging them would halve it.
    "max_courts_per_block": 2,
    "court_gap_m": 16.0,         # between two clusters on one block
    # How finely the frontage is walked looking for an anchor. The cluster is
    # symmetric about its own drive, so the station IS the only way it can
    # slide along a face, and a coarse walk simply misses fits. MEASURED over
    # seeds 3/5/8 x three mixes, 411 (block, mix) pairs:
    #     6 m   287 fit (70%)   62.8 s
    #     4 m   305 fit (74%)   67.5 s
    #     3 m   309 fit (75%)   72.9 s
    # 4 m is the knee — four points of yield for seven percent of the search.
    "station_step_m": 4.0,
    # Boundary sampling interval of the containment test. See
    # :func:`_rect_inside` — the test walks the development's own
    # boundary, not a grid over its interior, and 2.5 m is under the
    # narrowest feature a `suburb_net` block has.
    "fit_step_m": 2.5,
    # -- what the caller supplies -------------------------------------------
    # Aligned with `suburb_parcel.house_sizes`: the style name of each measured
    # catalogue entry, so a mix named in styles can be resolved to the
    # `size_index` `build_placements` will place. Without it there is no way to
    # ask for a `terrace` and `plan_cluster` refuses the block.
    "house_styles": None,
    "house_sizes": None,
    # `fn(size_index, house_w) -> [(offset_along_u, half_width, kind), ...]`,
    # the same seam `suburb_parcel` uses. It is what aims the footway at the
    # real front door instead of at the middle of the elevation.
    "front_openings": None,
    "keepout_discs": (),
    # Which mix and which colour set this cluster builds. Set per DISTRICT by
    # `suburb_parcel._density_field`, not per block — that is what makes two
    # row-home AREAS differ rather than two adjacent blocks.
    "mix": None,
    "palette_set": None,
}


def _rng_pair(v, fallback):
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    if isinstance(v, (int, float)):
        return float(v), float(v)
    return fallback


def _pick_weighted(rng, weights):
    """A key from ``{name: weight}``, or None when nothing has weight."""
    names = [k for k, w in (weights or {}).items() if float(w) > 0.0]
    if not names:
        return None
    tot = sum(float(weights[k]) for k in names)
    r = rng.random() * tot
    for k in names:
        r -= float(weights[k])
        if r <= 0.0:
            return k
    return names[-1]


def draw_mix(rng, cfg=None):
    """The mix name a row DISTRICT builds. Drawn once per district seed."""
    c = dict(DEFAULTS)
    c.update(cfg or {})
    return _pick_weighted(rng, c.get("mix_weights") or MIX_WEIGHTS)


def draw_palette_set(rng, cfg=None):
    """The colour set a row DISTRICT is painted from. Once per district seed."""
    c = dict(DEFAULTS)
    c.update(cfg or {})
    return _pick_weighted(rng, c.get("palette_set_weights")
                          or PALETTE_SET_WEIGHTS)


def _corners(cx, cy, w, d, ux, uy):
    """Four corners of a box centred at (cx, cy), *w* along (ux, uy).

    Same convention and same vertex order as `suburb_parcel._corners`, and it
    has to be: the overlap tests, the car keep-out and the yard pass all take
    the two lists as interchangeable.
    """
    vx, vy = -uy, ux
    hw, hd = w / 2.0, d / 2.0
    return [(cx + ux * hw + vx * hd, cy + uy * hw + vy * hd),
            (cx - ux * hw + vx * hd, cy - uy * hw + vy * hd),
            (cx - ux * hw - vx * hd, cy - uy * hw - vy * hd),
            (cx + ux * hw - vx * hd, cy + uy * hw - vy * hd)]


def _hits_disc(pts, discs):
    """Does any of *pts* land inside a keep-out disc?"""
    for (c, r) in (discs or ()):
        r2 = float(r) * float(r)
        for q in pts:
            if (q[0] - c[0]) ** 2 + (q[1] - c[1]) ** 2 <= r2:
                return True
    return False


def _inward(poly, p, t):
    """Unit normal at *p* on the boundary pointing into the polygon."""
    n = (-t[1], t[0])
    if not point_in_polygon(poly, _add(p, _mul(n, 0.75))):
        n = (-n[0], -n[1])
    return n


def _frame_pts(p, u, n, a, b):
    return (p[0] + u[0] * a + n[0] * b, p[1] + u[1] * a + n[1] * b)


def _rect_inside(poly, p, u, n, a0, a1, b0, b1, discs, step):
    """Is the (a, b) rectangle wholly inside *poly* and off every keep-out disc?

    THE BOUNDARY, NOT A GRID, and that is exact rather than an approximation.
    A block polygon is SIMPLE — one closed ring, no islands — so it cannot bite
    into a rectangle without crossing that rectangle's own boundary twice. If
    every point of the boundary is inside the polygon, the interior is too.
    The first version sampled a 4 m grid over the whole rectangle instead: on a
    92 x 88 m development that is 550 points against the boundary's 180, three
    n_units to try and a hundred frontage stations to try them at, and one
    seed's parcelling went from seconds to minutes.

    Four corners plus the centre are tested FIRST as a cheap gate, because the
    overwhelming majority of stations fail on a corner and never deserve the
    dense walk.

    *step* is the boundary sampling interval. 2.5 m is well under the smallest
    feature a `suburb_net` block has — the narrowest neck this generator emits
    is a cul-de-sac stem at a whole carriageway width — so nothing can poke in
    between two samples.
    """
    corners = ((a0, b0), (a1, b0), (a1, b1), (a0, b1))
    gate = [_frame_pts(p, u, n, a, b) for (a, b) in corners]
    gate.append(_frame_pts(p, u, n, (a0 + a1) / 2.0, (b0 + b1) / 2.0))
    for q in gate:
        if not point_in_polygon(poly, q):
            return None
    pts = list(gate)
    for i in range(4):
        aa, bb = corners[i]
        ca, cb = corners[(i + 1) % 4]
        L = math.hypot(ca - aa, cb - bb)
        k = max(1, int(L / max(0.5, step)))
        for j in range(1, k):
            t = j / float(k)
            q = _frame_pts(p, u, n, aa + (ca - aa) * t, bb + (cb - bb) * t)
            if not point_in_polygon(poly, q):
                return None
            pts.append(q)
    if _hits_disc(pts, discs):
        return None
    return pts


def _mix_styles(mix_name, cfg):
    mixes = cfg.get("mixes") or MIXES
    return dict(mixes.get(mix_name) or {})


def _resolve(styles, cfg):
    """``{style: (size_index, w, d)}`` for the styles this catalogue can place.

    A mix naming a style the catalogue does not carry loses that style rather
    than the cluster: an asset set with no `terrace` should still build the
    cottage rows. A mix that loses ALL of its styles returns empty and the
    caller refuses the block, which is the honest answer.
    """
    names = list(cfg.get("house_styles") or ())
    sizes = list(cfg.get("house_sizes") or ())
    measured = dict(MEASURED)
    measured.update(cfg.get("measured") or {})
    out = {}
    for s in styles:
        try:
            i = names.index(s)
        except ValueError:
            continue
        if s in measured:
            w, d = measured[s]
        elif i < len(sizes):
            w, d = float(sizes[i][0]), float(sizes[i][1])
        else:
            continue
        out[s] = (i, float(w), float(d))
    return out


def _draw_style(rng, weights, allowed):
    """A style from the mix, restricted to what the catalogue can place."""
    w = {k: v for k, v in weights.items() if k in allowed}
    return _pick_weighted(rng, w)


def _draw_palette(rng, palettes, previous):
    """A colour from the set, never the one next door.

    See `PALETTE_SETS`: adjacent identical colour is what makes a row read as
    copy-paste, and it is far more visible than a repeat four doors along.
    A one-colour set degrades to that colour rather than looping forever.
    """
    opts = [q for q in palettes if q != previous] or list(palettes)
    return opts[rng.randrange(len(opts))] if opts else None


def _one_cluster(poly, p, u, n, rng, cfg, resolved, weights, palettes,
                 discs, taken, index):
    """Fit and compose ONE court with its two rows at this frontage anchor.

    Returns the cluster record, or None when nothing in the units-per-row range
    fits here. The fit is tried LARGEST FIRST and the units are never shrunk to
    make one work: they are measured art, and shrinking a measured asset to fit
    a reservation is the exact mistake `suburb_parcel.house_sizes` exists to
    stop the rest of this pipeline making.
    """
    gap = float(cfg["unit_gap_m"])
    setback = float(cfg["front_setback_m"])
    jit = max(0.0, float(cfg["setback_jitter_m"]))
    pad = float(cfg["court_pad_m"])
    b_front = float(cfg["court_front_m"])
    drive_w = float(cfg["drive_w_m"])
    green = float(cfg["green_m"])
    head = float(cfg["head_green_m"])
    walk_w = float(cfg["walk_w_m"])
    rear_clear = float(cfg["unit_rear_clear_m"])
    step = float(cfg["fit_step_m"])
    openings = cfg.get("front_openings")
    bay_w, bay_d = float(PARKING["bay"][0]), float(PARKING["bay"][1])
    aisle = float(PARKING["aisle"])

    # THE PITCH IS THE WIDEST STYLE IN THE MIX, and it is what everything is
    # RESERVED against: the fit test, the court's length and the bay run. It
    # has to be the widest, because which styles a row actually draws is not
    # known until after the block has been committed to, and a reservation
    # struck off the average is a reservation the row can overrun.
    #
    # WITHIN the reservation the units are PACKED at a constant party gap and
    # the packed run centred, rather than each unit sitting in the middle of
    # its own pitch. That was the first version and it is wrong for this
    # morphology: on the `garden` mix an 11.1 m unit in an 18.6 m pitch gets
    # 7.5 m of daylight either side, so half the row reads as DETACHED — which
    # is the one thing an attached row must not do. Packed, every party gap is
    # exactly `unit_gap_m` whatever was drawn, and a row of narrow units is
    # simply a shorter row with more communal green at its ends.
    w_max = max(v[1] for v in resolved.values())
    d_max = max(v[2] for v in resolved.values())
    pitch = w_max + gap
    court_w = 2.0 * bay_d + aisle
    # THE CORE IS WHAT MUST FIT; THE GREEN IS WHAT IS LEFT OVER. Half-width of
    # the built development off the court centreline: the court, the
    # footway/setback at its jittered maximum, and the deepest unit in the mix.
    #
    # The first version required the green as well, and it cost most of the
    # blocks: 6 m behind each row plus 5 m at the head plus the setback jitter
    # inflates the reservation by 14 m across and 5 m deep, which on a
    # `terrace` cluster is 92.6 x 88 m against the core's 78.2 x 83. Measured
    # over seeds 1/3/5/7/8/12 at `row_share: 0.15`, requiring it refused rather
    # more blocks than it built. A development squeezed onto a tight block gets
    # a two-metre amenity strip instead of a six-metre one — which is what
    # happens on real tight sites — so the green is GROWN into whatever the
    # block still has after the core is in.
    core_half = court_w / 2.0 + setback + jit + d_max

    def _fits(hw, dep):
        return _rect_inside(poly, p, u, n, -hw, hw, 0.0, dep,
                            discs, step) is not None

    n_lo, n_hi = _rng_pair(cfg.get("units_per_row", [3, 5]), (3.0, 5.0))
    n_units = court_len = b0 = b1 = depth = env = half_w = None
    for cand in range(int(n_hi), max(1, int(n_lo)) - 1, -1):
        c_len = cand * pitch
        lo, hi = b_front, b_front + c_len
        # The drive runs out THROUGH the block boundary to meet the kerb, so
        # containment is tested from b = 0 and only the drive polygon is
        # allowed past it — the carriageway is drawn over that overlap.
        if not _fits(core_half, hi):
            continue
        # Amenity ground, greedily, coarse to nothing. Four tries each rather
        # than a bisection: the whole range is 6 m and a metre either way is
        # not a distinction anybody can see.
        g_keep = 0.0
        for frac in (1.0, 0.66, 0.33):
            if _fits(core_half + green * frac, hi):
                g_keep = green * frac
                break
        h_keep = 0.0
        for frac in (1.0, 0.66, 0.33):
            if _fits(core_half + g_keep, hi + head * frac):
                h_keep = head * frac
                break
        hw = core_half + g_keep
        dep = hi + h_keep
        box = _corners(p[0] + n[0] * dep / 2.0, p[1] + n[1] * dep / 2.0,
                       2.0 * hw, dep, u[0], u[1])
        if any(_convex_overlap(box, q, pad=-0.5) for q in taken):
            continue
        n_units, court_len, b0, b1 = cand, c_len, lo, hi
        depth, env, half_w = dep, box, hw
        green, head = g_keep, h_keep
        break
    if n_units is None:
        return None

    def W(a, b):
        return _frame_pts(p, u, n, a, b)

    houses, walks = [], []
    for row, sgn in ((0, -1.0), (1, 1.0)):
        # THE UNIT'S OWN FRAME. `n_unit` points AWAY from the court, so the
        # front (local -Y at the placement yaw) looks at it; `u_unit` is then
        # forced by ``perp(u_unit) == n_unit`` — see the module docstring for
        # what goes wrong when it is not.
        n_unit = (sgn * u[0], sgn * u[1])
        u_unit = (n_unit[1], -n_unit[0])
        # `u_unit` is +n on the row at -u and -n on the row at +u, so a door
        # offset along the unit's own frontage maps to +-that along the court.
        ub = 1.0 if row == 0 else -1.0
        # DRAW THE WHOLE ROW FIRST, so the packed run can be centred: where a
        # unit stands depends on how wide its neighbours turned out.
        drawn = []
        for _k in range(n_units):
            st = _draw_style(rng, weights, resolved)
            if st is not None:
                drawn.append(st)
        run = sum(resolved[st][1] for st in drawn) + gap * max(
            0, len(drawn) - 1)
        b_at = b0 + (court_len - run) / 2.0
        prev_pal = None
        for style in drawn:
            size_i, h_w, h_d = resolved[style]
            # WITHIN-ROW VARIATION, in the three places it is free:
            #   the STYLE, drawn per unit from this cluster's mix;
            #   the SETBACK, jittered BACK off the common building line, so the
            #     line stays a minimum and no unit ever creeps at the court;
            #   the PALETTE, from this cluster's set with next door excluded.
            back = rng.uniform(0.0, jit)
            pal = _draw_palette(rng, palettes, prev_pal)
            prev_pal = pal
            a_wall = sgn * (court_w / 2.0 + setback + back)
            a_c = a_wall + sgn * h_d / 2.0
            b_c = b_at + h_w / 2.0
            b_at += h_w + gap
            c = W(a_c, b_c)
            fr = W(sgn * court_w / 2.0, b_c)
            corners = _corners(c[0], c[1], h_w, h_d, u_unit[0], u_unit[1])
            # The lot is the ground this unit answers for and no more: the
            # footway strip in front, its own footprint, and `rear_clear`
            # behind it. THE COMMUNAL GREEN IS DELIBERATELY OUTSIDE IT —
            # `suburb_scene.build_open_planting` keeps out of every
            # `lot_corners` and plants the block interior, so leaving the green
            # out is what puts trees on it. Inside, it would be bare mown grass
            # with a fence-line's worth of nothing on it.
            lot_d = setback + back + h_d + rear_clear
            # Half a party gap either side, so two neighbours' lots abut on the
            # centreline of the gap between them and neither claims the other's
            # wall. `pitch` would overlap them by whatever the units are
            # narrower than the widest style.
            half_lot = (h_w + gap) / 2.0
            fl = _add(fr, _mul(u_unit, -half_lot))
            frr = _add(fr, _mul(u_unit, half_lot))
            rl = _add(fl, _mul(n_unit, lot_d))
            rr = _add(frr, _mul(n_unit, lot_d))
            gaps = (openings(size_i, h_w)
                    if (openings is not None and size_i is not None) else None)
            front_gaps = [(g[0], g[1]) for g in (gaps or ())] or [(0.0, 1.0)]
            door_x = 0.0
            for g in (gaps or ()):
                if len(g) > 2 and g[2] == "door":
                    door_x = float(g[0])
                    break
            houses.append({
                "c": c, "w": h_w, "d": h_d, "u": u_unit,
                "corners": corners,
                "archetype": "row",
                "size_index": size_i,
                # WHAT A ROW UNIT DOES NOT HAVE, said explicitly rather than
                # left to a missing key. Every one of these is read somewhere
                # downstream, and together they are the whole of the "no
                # garage, no drive, no pool, no private fence" contract.
                "has_garage": False,
                "art_garage": any(len(g) > 2 and g[2] == "drive"
                                  for g in (gaps or ())),
                "has_fence": False,
                "has_pool": False,
                "garage": None,
                "fence_segs": [],
                "lot_width": h_w + gap,
                "lot_corners": [fl, frr, rr, rl],
                "lot_depth": lot_d,
                "front_gaps": front_gaps,
                "density": "row",
                "n": n_unit, "frontage": fr,
                "yaw_deg": math.degrees(math.atan2(u_unit[1], u_unit[0])),
                # THE FOUR KEYS THAT ARE NEW. `row` is the flag a pass tests
                # when it genuinely has to behave differently; `cluster` says
                # which court this unit belongs to; `style` records what was
                # drawn without anybody having to walk the catalogue back; and
                # `palette` is the per-unit colour `build_placements` hands to
                # `modular_house.apply_palette`.
                "row": True,
                "cluster": int(index),
                "style": style,
                "palette": pal,
            })
            walks.append((W(sgn * court_w / 2.0, b_c + ub * door_x),
                          W(a_wall, b_c + ub * door_x)))
    if not houses:
        return None

    # -- the court, in the schema `suburb_park.parking_info` publishes -------
    # Same keys, same meanings, so `disaster.people`'s `parking_refuge` and the
    # car pass can take a cluster court and the park's refuge lot through the
    # identical code. `yaw_deg` is the LOT's own +x, which here is the aisle
    # running into the block (`n`), and a bay's heading is that +-90 — exactly
    # what `parking_info` documents.
    n_fit = int(max(0.0, court_len - 2.0 * pad) // bay_w)
    n_want = int(round(len(houses) * float(cfg["bays_per_dwelling"]) / 2.0))
    n_bay = max(0, min(n_fit, n_want))
    used = n_bay * bay_w
    b_bay0 = b0 + (court_len - used) / 2.0
    bays = []
    for sgn in (-1.0, 1.0):
        a_bay = sgn * (court_w / 2.0 - bay_d / 2.0)
        for k in range(n_bay):
            bays.append({"centre": W(a_bay, b_bay0 + (k + 0.5) * bay_w),
                         "yaw_deg": math.degrees(
                             math.atan2(sgn * u[1], sgn * u[0]))})
    court = [W(-court_w / 2.0, b0), W(court_w / 2.0, b0),
             W(court_w / 2.0, b1), W(-court_w / 2.0, b1)]
    # The apron runs 1.5 m PAST the block boundary. That boundary is the kerb
    # line, so an apron stopping on it would leave a hairline of grass between
    # the drive and the road; the carriageway sits a rung higher in the ground
    # z ladder, so the overlap is covered rather than z-fighting.
    drive = [W(-drive_w / 2.0, -1.5), W(drive_w / 2.0, -1.5),
             W(drive_w / 2.0, b0), W(-drive_w / 2.0, b0)]
    greens = []
    for sgn in (-1.0, 1.0):
        g_in = sgn * (court_w / 2.0 + setback + jit + d_max)
        g_out = g_in + sgn * green
        greens.append([W(min(g_in, g_out), b0), W(max(g_in, g_out), b0),
                       W(max(g_in, g_out), b1), W(min(g_in, g_out), b1)])
    greens.append([W(-half_w, b1), W(half_w, b1),
                   W(half_w, b1 + head), W(-half_w, b1 + head)])

    return {
        "index": int(index),
        "mix": cfg.get("mix"),
        "palette_set": cfg.get("palette_set"),
        "units": len(houses),
        "units_per_row": n_units,
        "rows": 2,
        "pitch_m": pitch,
        # The development's own extent in its (u, n) frame, so a consumer can
        # size something against a cluster without re-deriving it from
        # `envelope`: half-width off the court centreline, and depth from the
        # block boundary to the back of the head green.
        "half_width_m": half_w,
        "depth_m": depth,
        "origin": tuple(p), "u": tuple(u), "n": tuple(n),
        "envelope": env,
        "court": court,
        "drive": drive,
        "walks": walks,
        "walk_w_m": walk_w,
        "greens": greens,
        "houses": houses,
        # THE PAVING, as (a, b, half-width) axes for a ribbon sampler. Court and
        # drive are both rectangles, so each is one straight run with a
        # half-width — which is the shape `suburb_scene.paving_keepout` already
        # speaks, so nothing downstream needs a polygon rasteriser to keep a
        # tree off the court.
        "paving": ([(W(0.0, b0), W(0.0, b1), court_w / 2.0),
                    (W(0.0, -1.5), W(0.0, b0), drive_w / 2.0)]
                   + [(a, b, walk_w / 2.0) for (a, b) in walks]),
        "parking": {
            "corners": court,
            "centre": W(0.0, (b0 + b1) / 2.0),
            "yaw_deg": math.degrees(math.atan2(n[1], n[0])),
            "w": court_len, "d": court_w,
            "bays": bays,
            "apron": drive,
            "entrance": W(0.0, 0.0),
            "mouth": W(0.0, b0),
            "rows": 2, "bays_per_row": n_bay,
        },
    }


def _convex_overlap(a, b, pad=0.0):
    """Separating-axis overlap for two convex polygons, `suburb_parcel`'s.

    Copied rather than imported to keep the dependency one-way: `suburb_parcel`
    imports THIS module, so importing it back would be a cycle at load time.
    """
    for poly in (a, b):
        m = len(poly)
        for i in range(m):
            ex = poly[(i + 1) % m][0] - poly[i][0]
            ey = poly[(i + 1) % m][1] - poly[i][1]
            if abs(ex) < 1e-12 and abs(ey) < 1e-12:
                continue
            nv = _unit((-ey, ex))
            amin = min(nv[0] * q[0] + nv[1] * q[1] for q in a)
            amax = max(nv[0] * q[0] + nv[1] * q[1] for q in a)
            bmin = min(nv[0] * q[0] + nv[1] * q[1] for q in b)
            bmax = max(nv[0] * q[0] + nv[1] * q[1] for q in b)
            if amax + pad < bmin or bmax + pad < amin:
                return False
    return True


def plan_cluster(poly, frontage, rng, cfg=None):
    """One or more row-home clusters on a block, or None if none fits.

    *poly* is the block polygon and *frontage* the per-vertex street flags
    `suburb_net.blocks_from_faces` emits (side ``i`` runs from vertex ``i``);
    ``None`` means every side is a street.

    Returns::

        {"houses":  [ ...same records `parcel_blocks` emits... ],
         "clusters": [ {court, drive, walks, greens, parking, paving, ...} ],
         "units": n, "courts": n}

    THE SEARCH. Frontage stations every `station_step_m`, each one tried as the
    drive's mouth with the block's inward normal as the aisle. At each station
    the largest `units_per_row` that fits wins — the units are never shrunk,
    because they are measured art and shrinking one is what put a 12 m asset on
    a 9 m lot everywhere else in this pipeline. A block that takes one cluster
    is walked on past it for a second, up to `max_courts_per_block`, each with
    its own drive; two courts sharing one drive is not what this builds,
    because the single drive IS the risk the morphology is here to model.
    """
    c = dict(DEFAULTS)
    c.update(cfg or {})
    if len(poly) < 3:
        return None
    mix_name = c.get("mix") or draw_mix(rng, c)
    weights = _mix_styles(mix_name, c)
    if not weights:
        return None
    resolved = _resolve(weights, c)
    if not resolved:
        # The catalogue carries none of this mix's styles. Refusing the block
        # is the honest answer: substituting whatever IS there would build a
        # development out of the one style nobody chose.
        return None
    pset = c.get("palette_set") or draw_palette_set(rng, c)
    palettes = list((c.get("palette_sets") or PALETTE_SETS).get(pset) or ())
    discs = [((float(q[0][0]), float(q[0][1])), float(q[1]))
             for q in (c.get("keepout_discs") or ())]

    ring = list(poly) + [poly[0]]
    perim = polyline_length(ring)
    cum = [0.0]
    for i in range(len(ring) - 1):
        cum.append(cum[-1] + _dist(ring[i], ring[i + 1]))

    def is_street(s):
        if not frontage:
            return True
        for i in range(len(cum) - 1):
            if s <= cum[i + 1]:
                return bool(frontage[i % len(frontage)])
        return bool(frontage[-1])

    out, taken = [], []
    step = max(1.0, float(c["station_step_m"]))
    max_courts = max(1, int(c["max_courts_per_block"]))
    s = step * 0.5
    while s < perim and len(out) < max_courts:
        if not is_street(s):
            s += step
            continue
        pt = point_at(ring, s)
        t = tangent_at(ring, s)
        u = _unit(t)
        nv = _inward(poly, pt, t)
        cl = _one_cluster(poly, pt, u, nv, rng, c, resolved, weights,
                          palettes, discs, taken, len(out))
        if cl is None:
            s += step
            continue
        out.append(cl)
        taken.append(cl["envelope"])
        # Past this cluster's own width plus a gap. The envelope test above is
        # what actually enforces separation — this only stops the walk
        # re-testing forty stations inside a court it just built.
        s += max(step, float(c["court_gap_m"]))
    if not out:
        return None
    return {"houses": [h for cl in out for h in cl["houses"]],
            "clusters": out, "units": sum(cl["units"] for cl in out),
            "courts": len(out)}


def stats(clusters):
    """Reporting numbers for a list of cluster records."""
    if not clusters:
        return {"courts": 0, "units": 0, "bays": 0, "mixes": {}, "styles": {},
                "palettes": {}}
    mixes, styles, pals = {}, {}, {}
    n_bay = 0
    for cl in clusters:
        mixes[cl.get("mix")] = mixes.get(cl.get("mix"), 0) + 1
        n_bay += len(cl["parking"]["bays"])
        for h in cl["houses"]:
            styles[h["style"]] = styles.get(h["style"], 0) + 1
            pals[h["palette"]] = pals.get(h["palette"], 0) + 1
    return {"courts": len(clusters),
            "units": sum(cl["units"] for cl in clusters),
            "bays": n_bay, "mixes": mixes, "styles": styles, "palettes": pals}
