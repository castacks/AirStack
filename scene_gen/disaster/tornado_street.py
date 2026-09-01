"""tornado_street -- R9, stream SP: THE STREET PASS.

`_plans/urban_tornado_plan.md` Sec 8 (ROUND 3): "The corridor's own props
stood untouched through two launches (streetlights plumb, signals up,
awnings on, trees green, cars parked) -- the strongest 'nothing happened
here' signal left." `plan_street` (below) is the PURE decision half -- no
`pxr` import anywhere ABOVE the "APPLY" section, so this module is
importable and testable with nothing but `usd-core`-free Python; every
test in `tests/test_tornado_street.py` except the one apply-side stage
test never needs `pxr` on the path at all.

`apply_street` (the "APPLY" section, added 2026-09-01 -- see that
section's own docstring for why it was EXTRACTED from the launcher rather
than left there) is the impure half: it lazily imports `pxr`/
`vegetation` INSIDE its own functions, never at module top level, so
importing `tornado_street` itself still never requires `usd-core` -- only
CALLING `apply_street` (or the private helpers under it) does. The city
launcher (`urban_tornado_city_launch_script.TornadoCityApp.street_pass`)
is now a thin wrapper: it resolves env vars into `plan_street`'s
arguments, calls `plan_street` then `apply_street`, and does launcher-only
bookkeeping (VRAM capture) with what comes back -- exactly the split
`tornado.car_pose`/`toss_prim` already use in this pipeline.

THE MODEL THIS PORTS FROM. `simulation/isaac-sim/launch_scripts/
suburb_tornado_launch_script.py`'s own corridor pass (its section "5)
STREET FURNITURE, FENCES AND CARS IN THE CORRIDOR") and `tornado.car_pose`
are the two things being ported, not re-invented -- the FELLED/CARRIED
two-outcome-band draw (nested thresholds off ONE random number, per item),
the felled lean band (62-100 deg, flatter than a carried item's 38-96),
and the unconditional-Paulikas-shares car model are all identical in
mechanism to the suburb file. What changes for downtown:

  * the category VOCABULARY (measured below, not suburb's `prop_kind`
    yard-furniture list -- downtown has no yards);
  * the wind bearing used per item is `wind_at`'s LOCAL bearing at that
    item's own (x, y), not the suburb's single scene-wide `throw_deg`
    (`heading_deg + curl_deg`) -- the downtown corridor's own `tcfg` already
    carries `curvature_deg_per_km`, so a long corridor's wind direction
    genuinely varies along its length and a single global heading would be
    wrong for a curved track;
  * FELLED poles fall in CORRELATED RUNS (`hurricane_research.md` Sec 3.6:
    "domino-like failure... poles go in runs along the line" -- render as
    correlated sequences, "all leaning the same way", never independent);
  * CARS get a CORE "thrown" outcome layered on top of the ordinary
    unconditional-shares model (the 2026-09-01 mid-round directive -- see
    "THE THROWN CAR MECHANIC" below).

THE MEASURED VOCABULARY (grepped `category` in `detail/city_detail.py` and
`scene_gen/scene_generator.py`'s prop emission, 2026-09-01). `city_detail.
_CATEGORY` maps its pool names onto these placement categories:

    streetlight, bench, trash_can, fire_hydrant, bus_stop, street_tree,
    planter_fence, mailbox, cafe_set, bollard, bike_lane_delineator,
    parking_meter, bike_rack, utility_pole, dumpster, phone_booth,
    traffic_cone, billboard, manhole, newspaper_box, storm_drain, sign

`sign` is EVERY posted-signage pool folded into one category --
`signs_stop`, `signs_street_name` and `signs_parking` all collapse to
`"sign"` in `_CATEGORY` -- so THERE IS NO `sign_mast` / `sign_small` SPLIT
ANYWHERE IN THE EMITTED DATA. `traffic_light` and `car` are stamped inline
elsewhere in `city_detail.py` (`_place_traffic_lights`, `_place_parked_
cars`), outside `_CATEGORY`. `human` and `house` also appear as categories
on a full placements list but are not street stock -- see "WHAT THIS FILE
DOES NOT TOUCH" below. Unlike the suburb pipeline, `category` is the
TRUTHFUL field here: `prop_kind` (the suburb yardplan's own workaround for
its `plant`-budget category lie) does not exist anywhere in `city_detail.py`
or `scene_generator.py` -- grepped, zero hits outside `suburb_yardplan.py`
itself. This pass reads `category` directly.

AWNINGS ARE NOT A PLACEMENT CATEGORY AT ALL, and there is nothing under that
name for a street walk to act on. Ground-floor storefront awnings are baked
into the GAC/kit building facade texture/geometry (`gac_slice.
GLASS_TEX_NOT = ("awning",)`; `tools/_gac_props.py`'s "facade/street" hint
list carries `"awning"` as a per-piece material hint, not a placement).
Rooftop light fixtures/awnings are swept by the T1 roof ladder
(`tornado_urban.t_roof_props_sweep`'s own docstring: "Rooftop plant (and,
by the same flag, light fixtures/awnings at the lighter levels)"), which is
stream RF's file, not this one. The plan's R9 bullet ("awnings gone") is
therefore ALREADY covered upstream of this pass, by a different module --
recorded here so nobody re-implements it a second time looking for an
`"awning"` category that will never exist.

THE PRESET'S OWN ACTIVE SUBSET. `downtown_tornado_bench_500.yaml` (this
launcher's default `SCENE_CONFIG`) leaves several pools EMPTY on cost --
most notably `planter_fences: []` (`config/asset_sets/urban.yaml`'s own
comment: a tree guard bundles a full street tree inside its payload and at
`guard_chance 0.45` that was 65.8M points and OOM-killed the scene) -- so
the plan's own CARRIED example "planter" places ZERO instances in the scene
this launcher actually builds. `billboards` and `newspaper_boxes` are
likewise unconfigured in this preset. The mapping below still names them
(a future preset may enable them, and the pool being present costs nothing
if it is empty) but the MEASURED, ACTIVE vocabulary for
`downtown_tornado_bench_500` is: streetlight, traffic_light, sign (x3
pools), bus_stop, bench, trash_can, fire_hydrant, mailbox, cafe_set,
bollard, bike_lane_delineator, parking_meter, bike_rack, utility_pole,
dumpster, phone_booth, traffic_cone, manhole, storm_drain, street_tree, car.

MAPPING THE PLAN'S ILLUSTRATIVE LISTS ONTO THE MEASURED VOCABULARY. The R9
brief names FELLED as "(streetlight, traffic_light, sign_mast, bus_stop)"
and CARRIED as "(trash_can, bench, traffic_cone, sign_small, planter)".
Read literally that is five and four items respectively and would leave
the rest of the measured vocabulary (mailbox, cafe_set, dumpster,
phone_booth, newspaper_box, parking_meter, bike_lane_delineator, utility_
pole, bike_rack, billboard) untouched -- which is exactly the bug the
suburb model exists to FIX ("every mailbox on the plate standing perfectly
upright in a levelled block" was the review complaint that motivated
CARRIED in the first place). So the two lists below are EXTENDED to cover
the measured vocabulary, using the plan's five named items as anchors and
`_ORIENT`'s own physical grouping (`city_detail._ORIENT`: `streetlight` and
`utility_pole` share the `"pole"` orientation mode, distinct from `bench`/
`mailbox`'s `"face"` and `trash_can`'s `"free"`) as the classifying
principle -- bolted-and-tall goes FELLED, light-and-freestanding goes
CARRIED, exactly the suburb rule ("what a track does to a thing depends on
what the thing is"). Every mapping decision, and every category
deliberately left untouched, is recorded in the `FELLED`/`CARRIED`/
`_UNTOUCHED` comments below.

`sign` -> FELLED wholesale (no mast/small split exists to divide it on;
all three source pools -- `signs_stop`, `signs_street_name`, `signs_
parking` -- describe pole/post-mounted signage in the real world, so every
sign takes the mast share rather than being split arbitrarily).
`planter_fence` stands in for the plan's "planter" (there is no bare
`"planter"` category; the fence variant is the only one that exists, and it
currently places zero instances in this preset per the note above).

WHAT THIS FILE DOES NOT TOUCH, and why:
  * `house` -- buildings; the T0-T4 ladder (`tornado_urban`/`tornado_kit`)
    and the roof pass (`tornado_roof`, stream RF) own building damage.
  * `human` -- explicitly out of scope this round; R10 (stream V) is the
    urban victim-placement research this pass must not pre-empt.
  * `fire_hydrant`, `bollard` -- low, squat, no sail; cast iron / concrete,
    anchored below grade. Real hydrants and bollards routinely survive a
    tornado standing -- `_ORIENT` files both under `"face"`/`"free"`, never
    `"pole"`, which is the same physical read this pass uses to leave them
    alone rather than a coin flip that would sometimes carry a hydrant away.
  * `manhole`, `storm_drain` -- already `wind_flow.IMMOVABLE`; ground-level
    covers a track cannot lift.
  * `street_tree` -- its own TREE branch (below), not FELLED/CARRIED.
  * `car` -- its own CAR branch (below).

THE RUN CORRELATION (FELLED only). `hurricane_research.md` Sec 3.6:
"Render pole failures as correlated sequences along a street, all leaning
the same way -- never as independent random events" (StEER/Michael:
"domino-like failure of tens to hundreds of distribution poles"). Modelled
as a SEQUENTIAL walk (deterministic given the input placements' own order
and one `rng` stream): every FELLED item within `RUN_RADIUS_M` (40 m) of an
already-felled item in this same walk gets `RUN_BONUS` (+0.20) added to its
own `p_down` before the draw, AND -- "leaning the same way" is a heading
claim, not just a proximity one -- its fall bearing is pulled toward the
neighbour's own drawn bearing with a NARROWER jitter (`RUN_BEARING_JITTER`,
+-10 deg, against the independent draw's +-`FALL_BEARING_JITTER`, +-25 deg)
and its roll TAKES THE SAME SIGN (falls to the same side) rather than an
independent coin flip. The lean MAGNITUDE and the yaw jitter (which way the
fixture arm ends up facing) stay independently drawn, so a run reads as
"the same event" without reading as photocopied.

THE THROWN CAR MECHANIC (2026-09-01 mid-round directive, on top of the
original brief's "port the suburb car block"). The user's own framing:
"do cars get overturned/thrown? Cause you can do that." Requirement: in the
CORE (`i >= CORE_I`, 0.80) the outcome mix must visibly include FLIPPED
(on side/roof) and ROLLED cars displaced along the LOCAL wind bearing --
already what `car_pose` gives, once it is handed the per-point local
bearing instead of a global one -- AND a small share (0.10-0.15 of ALL core
cars) THROWN 20-60 m, landing along the bearing, never inside a building
OBB.

"THROWN" IS A RELABELLING OF A SLICE OF THE TIPPED POPULATION, NOT A
SEPARATE INDEPENDENT DRAW LAYERED ON TOP OF IT -- this is the load-bearing
design choice and it is what keeps item (3) of the directive true
("tipped+thrown share matches car_pose's own model within tolerance").
`car_pose` is called FIRST, UNCHANGED, with its own unconditional Paulikas
shares (`CAR_P_MOVE`/`CAR_P_TIP`) -- this is also what avoids reopening the
exact nesting bug the tornado skill's own "Vehicles" section records
(Paulikas' displacement rates are UNCONDITIONAL shares of ALL vehicles;
testing a second probability only on cars that already passed the first
one multiplies the two and silently collapses the tail -- "5 of 25 cars
moved and NONE tipped"). Only for a car in the CORE whose `car_pose` draw
already landed on a TIPPED pose (side/roof/nose -- `pose["toppled"]`) does
a SECOND, independent coin (`THROWN_OF_TIPPED`, 0.40) decide whether that
same car is promoted from "tipped in place" to "tipped and thrown 20-60 m".
`THROWN_OF_TIPPED = 0.40` is picked so that, given `car_pose`'s own
`p_tip(i=0.9, mass=1) ~= 0.05 + 0.30 * 0.9**1.5 ~= 0.306`, the resulting
UNCONDITIONAL share of thrown cars among ALL core cars at i=0.9 lands at
`0.40 * 0.306 ~= 0.122` -- inside the requested 0.10-0.15 band (measured in
`tests/test_tornado_street.py::test_core_thrown_share_in_band`, which is
also where "tipped+thrown share matches car_pose's own model" is checked:
the (tipped OR thrown) count is by construction the SAME count `car_pose`'s
own `p_tip` alone would have produced, since thrown never fires without a
prior tip; the promotion never touches an un-tipped or un-moved car and
never adds a NEW tipped event of its own).

THE LANDING POINT is drawn along the SAME local wind bearing `car_pose`
already used (+-`THROW_BEARING_JITTER`, 8 deg) at a fresh distance in
`THROW_RANGE_M` (20-60 m) -- NOT the small `car_pose`-native tip distance,
which is what makes this visibly different from an ordinary rolled car.
Validated against `buildings` (a `_footprint_test`-style OBB reject, built
once from the SAME `x_m`/`y_m`/`W`/`D`/`yaw_deg` shaped list `tornado_
urban_ground.scatter_corridor`'s own footprint rejection already reads) and
`bounds` (the plate's own region box) -- "clamped to streets/plazas" is
implemented as "never inside a building footprint and never off the plate",
since everything that is not a building footprint in this generator's own
layout IS street, sidewalk or plaza. Up to `THROW_TRIES` (6) redraws of
bearing jitter and distance before giving up and leaving the car at its
ORDINARY (untossed-further) `car_pose` outcome -- a car is never placed
inside a wall to satisfy a quota. The attitude (roll/pitch -- side, roof or
nose) is NOT redrawn on promotion: it stays whatever `car_pose` itself
picked, so a thrown car's silhouette is still one of `car_pose`'s three
documented tipped poses, just moved much further.

`UT_MIN_TIPPED` (the launcher's own env knob, default 2 -- see the launcher
docstring) is the suburb's `TOR_MIN_TIPPED` review-floor pattern, ported
here as `min_tipped=`: if fewer than `min_tipped` cars ended up toppled
after the ordinary walk, the highest-intensity untouched candidates are
forced over via `tornado.car_pose(force="tip")` -- which "skips the DRAW
only, never the pose model" (that function's own docstring) -- until the
floor is met or candidates run out. Forced cars are marked `"forced": True`
in their action record and are NEVER also promoted to "thrown": the floor
guarantees a VISIBLE tipped car for review, the thrown mechanic is a
separate, purely probabilistic enhancement, and conflating the two would
make `min_tipped` silently change the measured thrown share.

PURE. No `pxr` import anywhere in this module -- `tornado.py` and
`tornado_urban_ground._footprint_test` (imported below) are both
`pxr`-free at module level, by their own docstrings, for exactly this
reason. Every action dict is JSON-safe: plain `float`/`int`/`str`/`bool`/
`None`, no numpy scalars, no tuple keys.
"""

import math
import os

from . import tornado as trn
from .tornado_urban_ground import _footprint_test

# ---------------------------------------------------------------------------
# the category lists -- see the module docstring for the full mapping
# rationale and the measured vocabulary each name is drawn from.
# ---------------------------------------------------------------------------

FELLED = (
    "streetlight",      # plan's own example; bolted, tall, a sail on a pole
    "traffic_light",     # plan's own example; same physical class
    "sign",               # plan's "sign_mast" -- no mast/small split exists,
                          # so every posted sign (stop/street-name/parking)
                          # takes the mast share
    "bus_stop",           # plan's own example; a tall glazed shelter
    "utility_pole",       # `_ORIENT["utility_pole"] == "pole"`, same class
                          # as streetlight
    "bike_rack",          # bolted; the suburb's OWN FELLED list already
                          # carries `bike_rack` verbatim
    "billboard",          # tall panel-on-a-post; unconfigured in the
                          # default preset (harmless if it never fires)
)

CARRIED = (
    "trash_can",          # plan's own example
    "bench",              # plan's own example
    "traffic_cone",       # plan's own example
    "planter_fence",      # stands in for the plan's "planter" (no bare
                          # `"planter"` category exists); places zero
                          # instances in the default preset (see docstring)
    "mailbox",            # the suburb review's headline example of what
                          # must NOT stand plumb through a corridor
    "cafe_set",           # light table/chair group; suburb's own `cafe_set`
    "dumpster",           # large but on castors; suburb's `bin` counterpart
    "phone_booth",        # a light glazed shell, not founded like a hydrant
    "newspaper_box",       # small freestanding box; unconfigured by default
    "parking_meter",       # thin post, short -- not "founded and tall"
    "bike_lane_delineator",  # a flexible post MEANT to give
)

BLOWN = FELLED + CARRIED

#: Categories deliberately left standing -- see the module docstring's
#: "WHAT THIS FILE DOES NOT TOUCH" for the physical argument for each.
UNTOUCHED = ("fire_hydrant", "bollard", "manhole", "storm_drain")

#: Categories this pass skips outright -- `street_tree` and `car` are NOT
#: here even though they are not FELLED/CARRIED either: they have their
#: own dedicated branches in `plan_street` below (checked before the
#: FELLED/CARRIED dispatch), so listing them here would shadow those
#: branches and silently drop every tree and car action.
_SKIP = ("house", "human")

# ---------------------------------------------------------------------------
# FELLED / CARRIED shares -- `suburb_tornado_launch_script.py`'s own
# formulas, ported verbatim (same nested-threshold, gone-then-down,
# nothing-else-changed structure `tornado.car_pose` uses for the same
# reason: testing a lean only on props that already survived a removal
# roll multiplies the two probabilities and leaves the tail standing).
# ---------------------------------------------------------------------------

#: p_down = a + b * i  (FELLED primary outcome), capped; +RUN_BONUS when a
#: felled neighbour is within RUN_RADIUS_M, capped higher in that case so
#: the bonus can matter near the top of the range.
FELL_P_DOWN = (0.55, 0.40)
FELL_P_DOWN_CAP = 0.92
FELL_P_DOWN_CAP_RUN = 0.95
#: p_gone = FELL_GONE_FRAC * i * (1 - p_down)  (FELLED secondary outcome --
#: it takes the very core to shear a standard off its base and carry it).
FELL_GONE_FRAC = 0.55
#: A felled pole hinges at its base and lies FLATTER than a carried item.
FELL_LEAN_RANGE = (62.0, 100.0)
#: Small downtrack drag as the base goes over -- the base stays roughly
#: anchored, unlike a genuinely carried item.
FELL_DRAG_RANGE_M = (0.2, 1.4)

#: p_gone = a + b * i  (CARRIED primary outcome), capped.
CARRY_P_GONE = (0.18, 0.86)
CARRY_P_GONE_CAP = 0.90
#: p_down = CARRY_DOWN_FRAC * (1 - p_gone)  (CARRIED secondary outcome --
#: a survivor of the removal roll, still down and moved rather than plumb).
CARRY_DOWN_FRAC = 0.85
#: Knocked over, wider range than a hinged pole -- there is no single
#: hinge point on a bench or a trash can.
CARRY_LEAN_RANGE = (38.0, 96.0)
#: "Survivors tossed 1-6 m downwind" -- the R9 brief's own number, larger
#: than the suburb model's 0.2-1.4 m in-place drag: a carried-class item
#: that is not fully gone genuinely travels, it does not just topple.
CARRY_TOSS_RANGE_M = (1.0, 6.0)

#: The domino run: how close counts as "the same event", and how much it
#: raises the neighbour's own p_down.
RUN_RADIUS_M = 40.0
RUN_BONUS = 0.20
#: Bearing jitter: wide for an independent fall, narrow for a correlated
#: one ("all leaning the same way").
FALL_BEARING_JITTER_DEG = 25.0
RUN_BEARING_JITTER_DEG = 10.0
#: Facing (yaw) after the fall stays independently jittered either way --
#: a run agrees on WHICH WAY IT FELL, not on which way a fixture arm ended
#: up pointing.
YAW_JITTER_RANGE_DEG = (-40.0, 40.0)
CARRY_PITCH_RANGE_DEG = (-18.0, 18.0)

# ---------------------------------------------------------------------------
# trees
# ---------------------------------------------------------------------------

#: Wide fall-bearing jitter for a tree, matching the suburb/skill's own
#: convention ("wide +-38 deg jitter... a stand all pointing within five
#: degrees reads as a logging operation, not as windthrow") -- distinct
#: from a pole's narrower +-25.
TREE_FALL_JITTER_DEG = 38.0
#: `vegetation.wind_tree`'s own per-geom lean ranges (`"lean"`/`"uproot"`
#: branches, `disaster/vegetation.py` around line 2872) -- drawn HERE,
#: pure, rather than re-drawn by the launcher with a second rng stream,
#: so the action record alone determines the applied lean and the
#: determinism/JSON-round-trip tests cover it. `lean_min_deg` (`"uproot"`
#: only -- `"lean"` has no seat band to clamp against) is also
#: `wind_tree`'s own constant, handed through so the launcher's
#: `vegetation.tip_tree` call needs no extra knowledge of the split.
TREE_LEAN_RANGE_DEG = (19.0, 38.0)          # geom == "lean"
#: ROUND 4: 74-82 -> 76-88, AND `_apply_tree_level` no longer hands
#: `tip_tree` a `seat_band`, which is the change that actually matters.
#: MEASURED (`tools/tornado_tree_fall_probe.py`, 2026-09-01), Shumard_Oak
#: (10.9 m, crown radius 5.23 m), the crown-centre offset from the stump as
#: a fraction of the crown radius -- the "does it read fallen from 60-90 m"
#: number, since once the offset exceeds the radius the canopy no longer
#: sits over its own base:
#:
#:      lean   30    46    62    70    77    82    88
#:      off/r  0.51  0.73  0.88  0.94  0.97  0.99  0.99
#:
#: and the shipped `seat_band=(-1.1, -0.15)` bisection SETTLED ON 46.0 for
#: every one of the three species measured (it can never seat a crown that
#: wide, so it bottoms out on `lean_min_deg` and returns) -- i.e. every
#: "fallen" street tree in the round-3 bench was authored as a 46-degree
#: LEAN with its crown 4-9.6 m under the plate. That is exactly the
#: reviewed defect ("a fallen tree whose canopy reads as an intact standing
#: tree from above"). The band is dropped rather than retuned because the
#: quantity it bisects on is the WHOLE-TREE lowest point, which for a
#: leafed crown falls monotonically with lean -- seating it is only
#: possible by NOT laying the tree down.
TREE_UPROOT_RANGE_DEG = (76.0, 88.0)        # geom == "uproot"
#: Retained ONLY so the action record's shape is unchanged (it is still
#: written into every uproot action, and `tests/test_tornado_street.py`
#: round-trips the record). `_apply_tree_level` no longer passes it to
#: `tip_tree`, because there is no bisection left to floor.
TREE_UPROOT_LEAN_MIN_DEG = 46.0
#: How high a windthrown tree's BUTT ends up once the root plate levers out
#: of the ground -- the one number `_apply_tree_level` seats an uprooted
#: tree on (see its docstring: the pivot IS the butt, so this is an exact
#: placement, not a measurement).
TREE_ROOT_LIFT_M = 0.26
#: The soil-and-root wheel authored at the butt (`vegetation.root_plate`),
#: sized off the tree's own measured height and clamped to the 0.9-3.0 m
#: band real windthrow plates occupy.
TREE_PLATE_R_FRAC = 0.16
TREE_PLATE_R_RANGE_M = (0.9, 3.0)
#: A street tree wider than this cannot be windthrown legibly by a rigid
#: rotation about its base -- see `_apply_tree_level`'s own note and the
#: launcher spec in the round-4 report. Only ever WARNED about, never
#: silently "fixed": the fix belongs in the asset pool.
TREE_STREET_MAX_CROWN_R_M = 8.0

# ---------------------------------------------------------------------------
# cars
# ---------------------------------------------------------------------------

#: `car_pose` is engaged at all only once a car is genuinely in the track.
CAR_ENTRY_MIN_I = 0.20
#: The "thrown" mechanic only promotes CORE cars -- the 2026-09-01 directive.
CORE_I = 0.80
#: Share of an ALREADY-TIPPED core car that gets promoted to "thrown" --
#: see the module docstring's derivation of the resulting unconditional
#: share of ALL core cars (~0.12 at i=0.9, mass=1).
THROWN_OF_TIPPED = 0.40
THROW_RANGE_M = (20.0, 60.0)
THROW_BEARING_JITTER_DEG = 8.0
THROW_TRIES = 6

#: The SECOND review floor (round 4). `min_tipped` guarantees a car on its
#: side/roof; it says nothing about the cars that did not tip, and at
#: `CAR_P_MOVE = (0.10, 0.62)` an i=0.85 core car stays PARKED with
#: probability 1 - (0.10 + 0.62*0.85) = 0.373. On the three-car bench strip
#: that is a 37% chance per car of the exact defect the round-4 photo shows
#: -- "a red car in the corridor perfectly upright and pristine" (measured:
#: at UTB_SEED 7 the Nissan drew 0.71 > p_move and was never touched).
#:
#: `min_moved` is the same review-floor pattern as `min_tipped`, one rung
#: down: after the ordinary walk AND the tip floor, the highest-intensity
#: cars that are still untouched are forced through `car_pose(force="move")`
#: -- which "skips the DRAW only, never the pose model" -- until at least
#: `min_moved` cars have SOME outcome (tipped or shoved). It never converts
#: a shoved car into a tipped one (that is `min_tipped`'s job and it runs
#: first), so it cannot move the measured tipped/thrown shares: a forced
#: car is stamped `"forced": True` and `"floor": "moved"`, and
#: `test_core_thrown_share_in_band` keeps measuring the model with both
#: floors at 0, which is how every share in this file is quoted.
MOVED_FLOOR_MIN_I = 0.55


def _species_from_usd(usd):
    """`".../Vegetation/Shumard_Oak/Shumard_Oak.usd" -> "Shumard_Oak"` --
    the street-tree pool's own naming convention (`asset_sets/urban.yaml`),
    the same one `suburb_tornado_launch_script.TREE_SPECIES`'s keys use."""
    if not usd:
        return None
    stem = os.path.splitext(os.path.basename(str(usd)))[0]
    return stem or None


def _in_bounds(x, y, bounds):
    if not bounds:
        return True
    x0, y0, x1, y1 = bounds
    return x0 <= x <= x1 and y0 <= y <= y1


def _signed(rng):
    return 1.0 if rng.random() < 0.5 else -1.0


def _felled_action(path, category, x, y, i, bearing, rng, run_positions):
    """One FELLED-category item. Returns an action dict or `None`
    (untouched). Mutates nothing; the caller appends the returned position
    to `run_positions` itself when the outcome is `"fell"`."""
    bonus = 0.0
    nearest_bearing = None
    nearest_sign = None
    best_d = None
    for (px, py, pbearing, psign) in run_positions:
        d = math.hypot(x - px, y - py)
        if best_d is None or d < best_d:
            best_d, nearest_bearing, nearest_sign = d, pbearing, psign
    correlated = best_d is not None and best_d <= RUN_RADIUS_M
    if correlated:
        bonus = RUN_BONUS

    cap = FELL_P_DOWN_CAP_RUN if correlated else FELL_P_DOWN_CAP
    p_down = min(cap, FELL_P_DOWN[0] + FELL_P_DOWN[1] * i + bonus)
    p_gone = FELL_GONE_FRAC * i * (1.0 - p_down)

    r = rng.random()
    if r < p_gone:
        return {"path": path, "kind": "felled", "category": category,
                "action": "carry", "correlated": bool(correlated),
                "run_neighbor_m": (round(best_d, 3) if correlated else None)}
    if r < p_gone + p_down:
        if correlated:
            az = nearest_bearing + rng.uniform(-RUN_BEARING_JITTER_DEG,
                                               RUN_BEARING_JITTER_DEG)
            sign = nearest_sign
        else:
            az = bearing + rng.uniform(-FALL_BEARING_JITTER_DEG,
                                       FALL_BEARING_JITTER_DEG)
            sign = _signed(rng)
        lean = rng.uniform(*FELL_LEAN_RANGE) * sign
        dist = rng.uniform(*FELL_DRAG_RANGE_M) * (0.4 + 0.6 * i)
        yaw_jitter = rng.uniform(*YAW_JITTER_RANGE_DEG)
        return {"path": path, "kind": "felled", "category": category,
                "action": "fell", "bearing_deg": float(az % 360.0),
                "dist_m": float(dist), "roll_deg": float(lean),
                "yaw_jitter_deg": float(yaw_jitter),
                "correlated": bool(correlated),
                "run_neighbor_m": (round(best_d, 3) if correlated else None),
                "_roll_sign": sign}
    return None


def _carried_action(path, category, x, y, i, bearing, rng):
    p_gone = min(CARRY_P_GONE_CAP, CARRY_P_GONE[0] + CARRY_P_GONE[1] * i)
    p_down = CARRY_DOWN_FRAC * (1.0 - p_gone)
    r = rng.random()
    if r < p_gone:
        return {"path": path, "kind": "carried", "category": category,
                "action": "carry"}
    if r < p_gone + p_down:
        az = bearing + rng.uniform(-FALL_BEARING_JITTER_DEG,
                                   FALL_BEARING_JITTER_DEG)
        dist = rng.uniform(*CARRY_TOSS_RANGE_M)
        lean = rng.uniform(*CARRY_LEAN_RANGE) * _signed(rng)
        yaw_jitter = rng.uniform(*YAW_JITTER_RANGE_DEG)
        pitch = rng.uniform(*CARRY_PITCH_RANGE_DEG)
        return {"path": path, "kind": "carried", "category": category,
                "action": "toss", "bearing_deg": float(az % 360.0),
                "dist_m": float(dist), "roll_deg": float(lean),
                "yaw_jitter_deg": float(yaw_jitter),
                "pitch_deg": float(pitch)}
    return None


def _tree_action_for(path, category, x, y, i, bearing, rng, usd):
    species = _species_from_usd(usd)
    level = trn.tree_level_for_intensity(i, rng, species=species)
    rec = {"path": path, "kind": "tree", "category": category,
          "action": "level", "level": level, "species": species,
          "intensity": round(float(i), 5)}
    if level == "leaning":
        rec["geom"] = "lean"
    elif level in ("fallen", "snapped"):
        # v1: `snapped` gets the SAME tip/uproot treatment as `fallen`
        # rather than `vegetation.snap`'s true fracture-and-delete spar --
        # a live per-tree VTK cut needs a coordinated debris budget with
        # stream DB's ground ledger (its own removed material would
        # otherwise double up with `tornado_urban_ground`'s corridor
        # field), which is out of this pass's ownership. See the module
        # docstring / notes file "TREES" section for the follow-up.
        rec["geom"] = "uproot"
    else:
        rec["geom"] = None
    if rec["geom"] is not None:
        az = bearing + rng.uniform(-TREE_FALL_JITTER_DEG, TREE_FALL_JITTER_DEG)
        rec["azimuth_deg"] = float(az % 360.0)
        if rec["geom"] == "lean":
            rec["lean_deg"] = float(rng.uniform(*TREE_LEAN_RANGE_DEG))
        else:   # "uproot"
            rec["lean_deg"] = float(rng.uniform(*TREE_UPROOT_RANGE_DEG))
            rec["lean_min_deg"] = TREE_UPROOT_LEAN_MIN_DEG
    return rec


def _find_landing(x, y, bearing, dist_range, rng, jitter, in_building,
                  bounds, tries):
    for _ in range(int(tries)):
        az = bearing + rng.uniform(-jitter, jitter)
        d = rng.uniform(*dist_range)
        lx = x + math.cos(math.radians(az)) * d
        ly = y + math.sin(math.radians(az)) * d
        if not _in_bounds(lx, ly, bounds):
            continue
        if in_building is not None and in_building(lx, ly):
            continue
        return float(az % 360.0), float(d)
    return None


def _car_action(path, x, y, i, bearing, rng, *, throw_m, long_axis_deg,
                length_m, in_building, bounds):
    pose = trn.car_pose(i, rng, bearing, throw_m, x=x, y=y,
                        long_axis_deg=long_axis_deg, length_m=length_m,
                        blocked=None)
    if not pose["moved"]:
        return None, False
    thrown = False
    if i >= CORE_I and pose["toppled"] and rng.random() < THROWN_OF_TIPPED:
        landing = _find_landing(x, y, bearing, THROW_RANGE_M, rng,
                                THROW_BEARING_JITTER_DEG, in_building,
                                bounds, THROW_TRIES)
        if landing is not None:
            az, d = landing
            pose = dict(pose)
            pose["dx"] = math.cos(math.radians(az)) * d
            pose["dy"] = math.sin(math.radians(az)) * d
            pose["d_m"] = d
            thrown = True
    rec = {"path": path, "kind": "car", "category": "car",
          "action": "car_pose", "pose": pose["pose"],
          "dx": float(pose["dx"]), "dy": float(pose["dy"]),
          "d_m": float(pose["d_m"]), "roll_deg": float(pose["roll_deg"]),
          "pitch_deg": float(pose["pitch_deg"]),
          "yaw_delta_deg": float(pose["yaw_delta_deg"]),
          "toppled": bool(pose["toppled"]), "thrown": bool(thrown),
          "forced": False, "floor": None,
          "intensity": round(float(i), 5)}
    return rec, bool(pose["toppled"])


def plan_street(placements, intensity_fn, wind_at_fn, rng, *,
                buildings=None, bounds=None, throw_m=24.0, min_tipped=0,
                min_moved=0, car_length_fn=None, car_heading_fn=None):
    """Walk every NON-house, non-human placement and decide what the wind
    did to it. Pure: no `pxr`, no stage. Returns `[action, ...]`, JSON-safe.

    `placements` -- the full placements list (`prim_path`/`category`/
    `x_m`/`y_m`/`usd`/`yaw_deg` per item -- the exact shape `apply_
    placements` leaves on every entry, `self.placements` in the launcher).

    `intensity_fn(x, y) -> float` / `wind_at_fn(x, y) -> {"bearing_deg",
    ...}` -- callables, so this function never has to know `tcfg`'s shape;
    the launcher wires `lambda x, y: tn.wind_at(tcfg, x, y)` etc.

    `buildings` -- optional list of dicts with `x_m`/`y_m`/`W`/`D`/
    `yaw_deg` (the SAME shape `tornado_urban_ground._footprint_test`
    already reads, and `self.dump_doc["placements"]` already is) used ONLY
    to keep a THROWN car's landing point out of a building footprint.
    `bounds` -- optional `(x0, y0, x1, y1)`, the plate's own region --
    "clamped to streets/plazas" for a thrown landing.

    `throw_m` -- `tcfg["throw_m"]`, the scene's own debris reach, handed
    straight to `car_pose` for its ordinary (non-thrown) distance.

    `min_tipped` -- the review floor (`UT_MIN_TIPPED`'s pure half); 0 (the
    default) leaves the model exactly as measured, never set on a scene
    that is being measured.

    `min_moved` -- the SECOND review floor (`UT_MIN_MOVED`'s pure half, new
    in round 4; see `MOVED_FLOOR_MIN_I`). Counts tipped AND shoved cars,
    runs AFTER `min_tipped`, and only ever promotes a still-PARKED car to
    `car_pose(force="move")` -- so it can never change the tipped or thrown
    share. 0 (the default) leaves the model exactly as measured.

    `car_length_fn(placement) -> length_m` / `car_heading_fn(placement) ->
    long_axis_deg` -- optional; default to `tornado.CAR_REF_LEN_M` (mass=1)
    and the placement's own `yaw_deg` respectively, so this runs with no
    stage/resolver at all. The launcher wires a resolver-measured length
    for a more accurate mass proxy where one is cheaply available.
    """
    car_length_fn = car_length_fn or (lambda p: trn.CAR_REF_LEN_M)
    car_heading_fn = car_heading_fn or (lambda p: float(p.get("yaw_deg", 0.0)))
    in_building = _footprint_test(buildings) if buildings else None

    actions = []
    run_positions = []   # felled poles this walk has already put down
    car_cands = []        # (i, path, x, y, bearing, placement) for the
                          # ordinary pass AND the min_tipped top-up
    n_toppled = 0
    n_moved = 0

    for p in placements or ():
        path = p.get("prim_path")
        if not path:
            continue
        cat = str(p.get("category") or "")
        if cat in _SKIP:
            continue
        x, y = float(p.get("x_m", 0.0)), float(p.get("y_m", 0.0))
        i = max(0.0, min(1.0, float(intensity_fn(x, y))))
        if i <= 0.0:
            continue
        wind = wind_at_fn(x, y) or {}
        bearing = float(wind.get("bearing_deg", 0.0))

        if cat == "street_tree":
            actions.append(_tree_action_for(path, cat, x, y, i, bearing,
                                            rng, p.get("usd")))
            continue

        if cat == "car":
            if i < CAR_ENTRY_MIN_I:
                continue
            car_cands.append((i, path, x, y, bearing, p))
            continue

        if cat in FELLED:
            act = _felled_action(path, cat, x, y, i, bearing, rng,
                                 run_positions)
            if act is not None:
                actions.append({k: v for k, v in act.items()
                                if not k.startswith("_")})
                if act["action"] == "fell":
                    run_positions.append((x, y, act["bearing_deg"],
                                          act["_roll_sign"]))
        elif cat in CARRIED:
            act = _carried_action(path, cat, x, y, i, bearing, rng)
            if act is not None:
                actions.append(act)
        # else: UNTOUCHED or an unmapped category -- left standing.

    # ---- cars: ordinary pass, in PLACEMENT order (determinism follows
    # the input list, same as the felled/carried walk above) -------------
    done = set()
    for (i, path, x, y, bearing, p) in car_cands:
        rec, toppled = _car_action(
            path, x, y, i, bearing, rng, throw_m=throw_m,
            long_axis_deg=car_heading_fn(p), length_m=car_length_fn(p),
            in_building=in_building, bounds=bounds)
        if rec is not None:
            actions.append(rec)
            done.add(path)
            n_moved += 1
            if toppled:
                n_toppled += 1

    # ---- cars: the review floor (`UT_MIN_TIPPED`'s pure half) ---------
    if min_tipped > 0 and n_toppled < int(min_tipped):
        for (i, path, x, y, bearing, p) in sorted(car_cands,
                                                   key=lambda c: -c[0]):
            if n_toppled >= int(min_tipped):
                break
            if path in done:
                continue
            pose = trn.car_pose(max(i, 0.75), rng, bearing, throw_m, x=x,
                                y=y, long_axis_deg=car_heading_fn(p),
                                length_m=car_length_fn(p), blocked=None,
                                force="tip")
            if not pose["toppled"]:
                continue
            actions.append({
                "path": path, "kind": "car", "category": "car",
                "action": "car_pose", "pose": pose["pose"],
                "dx": float(pose["dx"]), "dy": float(pose["dy"]),
                "d_m": float(pose["d_m"]), "roll_deg": float(pose["roll_deg"]),
                "pitch_deg": float(pose["pitch_deg"]),
                "yaw_delta_deg": float(pose["yaw_delta_deg"]),
                "toppled": True, "thrown": False, "forced": True,
                "floor": "tipped", "intensity": round(float(i), 5)})
            done.add(path)
            n_toppled += 1
            n_moved += 1

    # ---- cars: the SECOND review floor (`UT_MIN_MOVED`'s pure half) ----
    # Runs LAST and only ever touches a car no earlier pass acted on, so
    # neither the ordinary Paulikas walk nor the tipped floor can be
    # disturbed by it -- see `MOVED_FLOOR_MIN_I`.
    if min_moved > 0 and n_moved < int(min_moved):
        for (i, path, x, y, bearing, p) in sorted(car_cands,
                                                   key=lambda c: -c[0]):
            if n_moved >= int(min_moved):
                break
            if path in done:
                continue
            pose = trn.car_pose(max(i, MOVED_FLOOR_MIN_I), rng, bearing,
                                throw_m, x=x, y=y,
                                long_axis_deg=car_heading_fn(p),
                                length_m=car_length_fn(p), blocked=None,
                                force="move")
            if not pose["moved"]:
                continue
            actions.append({
                "path": path, "kind": "car", "category": "car",
                "action": "car_pose", "pose": pose["pose"],
                "dx": float(pose["dx"]), "dy": float(pose["dy"]),
                "d_m": float(pose["d_m"]), "roll_deg": float(pose["roll_deg"]),
                "pitch_deg": float(pose["pitch_deg"]),
                "yaw_delta_deg": float(pose["yaw_delta_deg"]),
                "toppled": bool(pose["toppled"]), "thrown": False,
                "forced": True, "floor": "moved",
                "intensity": round(float(i), 5)})
            done.add(path)
            n_moved += 1
            if pose["toppled"]:
                n_toppled += 1

    return actions


# ---------------------------------------------------------------------------
# APPLY -- extracted from the launcher, 2026-09-01 (stream SP, follow-up).
#
# `plan_street` above decides; everything below APPLIES those decisions
# onto a live stage. This used to live entirely inside
# `urban_tornado_city_launch_script.py` (`_normalize_yaw_op`/`_apply_tree_
# level`/`_apply_street_action`/the loop in `TornadoCityApp.street_pass`).
# A LAUNCHER CANNOT BE IMPORTED (`SimulationApp` is constructed at import
# in every one of this repo's launcher files -- see that file's own
# docstring for why a second one segfaults), so stream B's bench could not
# wire `plan_street` at its own street cell: the application half was
# reachable from exactly one file, and the bench correctly declined to
# guess an untested contract rather than duplicate it blind. Moved here so
# ANY caller with a stage and an action list -- the city launcher, the
# bench, a future probe -- gets the SAME apply behaviour from one call.
# The city launcher's own `street_pass()` is now a thin wrapper around
# `apply_street` (env resolution + the callables `plan_street` wants +
# launcher-only bookkeeping like VRAM capture); no application LOGIC
# lives there any more.
#
# LAZY `pxr`/`vegetation` IMPORTS, INSIDE THE FUNCTIONS THAT NEED THEM --
# the module docstring's own claim ("importable and testable with nothing
# but `usd-core`-free Python") stays true for `plan_street` and everything
# above this line; only CALLING something in this section touches `pxr`.
# `vegetation` itself imports no `pxr` at module level either (checked
# directly, `hashlib`/`math`/`os`/`re` only) so pulling it in here adds no
# further dependency than `pxr` already does.
# ---------------------------------------------------------------------------

#: How far off grade a posed prop is allowed to end up before the seat
#: report calls it out. 2 cm is the round-4 brief's own tolerance.
SEAT_TOL_M = 0.02


def _wrap180(deg):
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return d + 360.0 if d <= -180.0 else d


def _long_axis_bearing(stage, path, roll0, pitch0, yaw0):
    """World bearing of the prop's own LONG axis under the placement
    rotation `rotateXYZ(roll0, pitch0, yaw0)`, in degrees, or `None`.

    "Long axis" = the LOCAL bbox axis with the largest extent that the
    placement rotation leaves HORIZONTAL. For a car that is the length
    (4.2-8.4 m against a 1.8-2.9 m width), and it is what `car_pose`'s
    roll is defined about ("a ROLLED car rolls about its LONG AXIS"). The
    vertical axis is excluded outright -- a saloon is 1.2-1.4 m tall, but
    an upended van is not, and rolling a prop about its own height axis is
    a yaw, not a roll.

    The bbox is measured on the UNROTATED referenced geometry (the prim's
    own ops are excluded by `ComputeUntransformedBound`), so this is the
    one bbox read in this module that a rotation cannot lie about: a box's
    EXTENTS along its own axes are exact, it is only its CORNERS that stop
    meaning anything once you turn it.
    """
    from pxr import Sdf, Usd, UsdGeom

    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return None
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = bc.ComputeUntransformedBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    size = r.GetSize()
    ca, sa = (math.cos(math.radians(roll0)), math.sin(math.radians(roll0)))
    cb, sb = (math.cos(math.radians(pitch0)), math.sin(math.radians(pitch0)))
    cc, sc_ = (math.cos(math.radians(yaw0)), math.sin(math.radians(yaw0)))
    # The three COLUMNS of Rz . Ry . Rx -- the world directions the local
    # x/y/z axes end up pointing.
    cols = (
        (cc * cb, sc_ * cb, -sb),
        (cc * sb * sa - sc_ * ca, sc_ * sb * sa + cc * ca, cb * sa),
        (cc * sb * ca + sc_ * sa, sc_ * sb * ca - cc * sa, cb * ca),
    )
    best = None
    for k in range(3):
        d = cols[k]
        if abs(d[2]) > 0.7:               # this local axis is the UP axis
            continue
        ext = float(size[k])
        if best is None or ext > best[0]:
            best = (ext, math.degrees(math.atan2(d[1], d[0])) % 360.0)
    return None if best is None else best[1]


def _points_min_z(stage, path):
    """World-space minimum z of the geometry under *path*, FROM MESH
    POINTS -- in stage units. `None` if nothing measurable is there.

    A BOUNDING BOX LIES ABOUT A ROTATED PRIM, and that is not a theory
    here, it is the measured cause of two of the four defects this round's
    D5 ledger names (`.agents/skills/fix-floating-debris`: "airborne wood
    audited as clean" for exactly this reason).

    `tornado.toss_prim` seats a prop by taking its UNTRANSFORMED bbox,
    rotating the eight CORNERS of that box by the roll/pitch about to be
    authored, and lifting by the lowest one. A box corner is a point where
    there is usually no geometry at all, so the rotated corner dives below
    the rotated MESH and the prop is lifted clear of the ground by the
    difference. MEASURED on the round-3 bench (`tools/
    tornado_street_seat_probe.py`, UTB_SEED 7, cell C1): the streetlight's
    own bbox is 0.55 x 2.19 x 6.01 m -- the 2.19 m is the LAMP ARM, which
    exists only at the top of the pole -- so at a 74.5 deg fell the corner
    at (y = +1.093, z = -0.012) is empty air, `toss_prim` lifted the pole
    1.044 m, and its lowest real vertex came to rest 0.788 m ABOVE the
    road. That is the "lamp pole lying flat but FLOATING above the ground
    (visible shadow gap)" in `C1_obl.png`, exactly.

    So: points, every time, and a bbox ONLY as the last resort for a prim
    that carries no points at all (an analytic `UsdGeom.Cube`, a prop whose
    every gprim is inside a `PointInstancer`).

    POINTINSTANCER SUBTREES ARE PRUNED AND BOUNDED, NOT WALKED. A
    prototype's own points sit wherever the prototype was authored -- near
    its own origin -- and have nothing to do with where its instances are,
    so walking into `/Prototypes` would return a confidently wrong number.
    `ComputeWorldBound` on the instancer itself is correct (it accounts for
    the per-instance transforms) and is the honest floor for that subtree.
    """
    from pxr import Sdf, Usd, UsdGeom

    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return None
    try:
        import numpy as _np
    except Exception:                                          # noqa: BLE001
        _np = None

    # A FRESH CACHE PER CALL, for `vegetation.tip_tree`'s own reason: a
    # cache reused across a transform change hands back the pose from
    # before the change, and this function is called on both sides of one.
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    best = None
    instancers = []
    it = iter(Usd.PrimRange(prim, Usd.TraverseInstanceProxies(
        Usd.PrimIsActive & Usd.PrimIsDefined)))
    for p in it:
        if p.IsA(UsdGeom.PointInstancer):
            instancers.append(p)
            it.PruneChildren()
            continue
        if not p.IsA(UsdGeom.PointBased):
            continue
        attr = UsdGeom.PointBased(p).GetPointsAttr()
        pts = attr.Get() if attr else None
        if not pts:
            continue
        m = xc.GetLocalToWorldTransform(p)
        c0, c1, c2, c3 = m[0][2], m[1][2], m[2][2], m[3][2]
        if _np is not None:
            arr = _np.asarray(pts, dtype=float)
            z = arr[:, 0] * c0 + arr[:, 1] * c1 + arr[:, 2] * c2 + c3
            zmin = float(z.min())
        else:
            zmin = min(float(v[0]) * c0 + float(v[1]) * c1
                       + float(v[2]) * c2 + c3 for v in pts)
        best = zmin if best is None else min(best, zmin)

    for p in instancers:
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_,
                                UsdGeom.Tokens.render])
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        if not r.IsEmpty():
            z = float(r.GetMin()[2])
            best = z if best is None else min(best, z)

    if best is None:
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_,
                                UsdGeom.Tokens.render])
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if not r.IsEmpty():
            best = float(r.GetMin()[2])
    return best


def _parent_z_scale(stage, prim):
    """How much world z one unit of the prim's own `translate` z buys --
    the holder chain's z scale. `place_holder` gives every bench/city cell
    an `ssf` scale, so this is never 1 by assumption."""
    from pxr import Usd, UsdGeom

    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    m = xc.GetParentToWorldTransform(prim)
    s = float(m[2][2])
    return s if abs(s) > 1e-9 else 1.0


def _seat_prim(stage, path, z_pre, ground_z=0.0):
    """Slide *path* straight down (or up) its own local z until its lowest
    MESH POINT is back where it was before the pose -- and never below
    *ground_z*. Returns `(gap_m, seated)`.

    THE DATUM IS `ground_z` WHEN ONE IS GIVEN (the default 0.0 -- the
    plate every scene in this pipeline builds its roads on), and the
    prop's OWN pre-pose lowest point when it is `None`.

    Preserving the pre-pose contact is the more conservative rule and it
    was the first cut, but it is wrong for the two things this pass
    actually does. A prop can arrive at this pass ALREADY off grade --
    `scene_generator.apply_placements` seats on `SizeResolver`'s `base`,
    measured with `useExtentsHint=True`, so it misses geometry outside a
    stale authored `extent` (MEASURED, cell C1: the Shumard_Oak street
    tree arrives 0.982 m below the plate, the GMC motorhome 0.139 m above
    it) -- and preserving that faithfully is a correct implementation of
    the wrong thing. And a THROWN car lands 20-60 m from where it was
    parked, so "the height of the bay it was parked in" is not its ground
    any more. Pass `None` only for a scene whose street props genuinely
    stand on something raised that this pass must not flatten.
    """
    from pxr import Gf, Sdf, UsdGeom

    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return None, False
    z_post = _points_min_z(stage, path)
    if z_post is None or z_pre is None:
        return None, False
    target = float(z_pre) if ground_z is None else float(ground_z)
    dz = target - z_post
    if abs(dz) < 1e-9:
        return 0.0, True
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpName().split(":")[-1] != "translate":
            continue
        t = op.Get() or Gf.Vec3d(0.0, 0.0, 0.0)
        op.Set(Gf.Vec3d(float(t[0]), float(t[1]),
                        float(t[2]) + dz / _parent_z_scale(stage, prim)))
        z_chk = _points_min_z(stage, path)
        return (None if z_chk is None else float(z_chk - target)), True
    return None, False


def _pose_prim(stage, path, dx, dy, roll_deg, yaw_jitter_deg, pitch_deg=0.0,
               ground_z=0.0):
    """Move, roll, pitch and RE-SEAT one placed prop. Returns
    `(ok, gap_m)`; `gap_m` is the seated prop's residual distance off its
    own datum, in metres of stage z (0 is perfect).

    THE ROUND-4 REPLACEMENT FOR `tornado.toss_prim`, and it exists for two
    measured reasons, neither of which can be fixed inside `toss_prim`
    itself (that file is shared with the frozen suburb preset, whose
    outputs are snapshot-tested, and it is not this stream's to edit).

    1. IT DESTROYS THE AXIS-UP CORRECTION, WHICH IS WHY THE BUS IS BURIED.
       `scene_generator.apply_placements` authors every placement as
       translate + rotateXYZ(roll, pitch, yaw) + scale, and for a Y-UP
       asset the `roll` is the +90 that stands the asset upright
       (`AssetPools.roll_of` returns 90 whenever `axis_of(usd) == "Y"`).
       `toss_prim` reads `vals.get("rotateZ")` BY NAME, finds nothing,
       clears the op order and rebuilds it as translate + rotateZ +
       rotateY + rotateX + scale -- so that +90 is silently dropped along
       with the placement yaw. MEASURED (`tools/
       tornado_street_seat_probe.py`, UTB_SEED 7): every car in the pool is
       Y-up (`130.usdz`, `FREE_GMC_Motorhome...`, `Nissan_Fairlady...`,
       `axis_up: Y`, `roll_deg: 90`), and the thrown motorhome -- the
       "yellow bus" of the review photo -- came out with its lowest vertex
       2.527 m BELOW the plate, the second forced car 1.738 m below. The
       `_normalize_yaw_op` guard ("DELIBERATELY LEFT ALONE if rotateXYZ
       carries a non-trivial roll or pitch") correctly declines to flatten
       that roll, and then hands the prim to `toss_prim` anyway, which
       flattens it.

       THE FIX IS THE RESIDUAL. Write the pose as

           T . Rz(yawU + dyaw) . Ry(pitch) . Rx(roll) . [Rz(yaw0 - yawU)
                                                         . Ry(p0) . Rx(r0)]

       where `yawU` is the WORLD BEARING OF THE PROP'S OWN LONG AXIS under
       the placement rotation. The bracketed residual is exactly a
       `rotateXYZ` of `(r0, p0, yaw0 - yawU)`, so it is authored as one
       suffixed op (`xformOp:rotateXYZ:axisUp`).

       `yawU` cannot just be `yaw0`, and that is the subtle half. An asset
       pack's art frame rarely points its long axis along +X, so
       `AssetPools.yaw_of` carries a per-asset `yaw-offset` that is folded
       INTO `yaw0` -- every car in the urban pool has `yaw_off = 90`.
       Stripping the whole of `yaw0` leaves the car's length along Y, and
       `Rx(roll)` then rolls it end-over-end instead of onto its side
       (MEASURED before this correction: 130.usdz "on its roof" came out
       3.0 m tall against the 2.0 m its width-axis roll should give, i.e.
       pitched, not rolled). So `yawU` is MEASURED from the geometry
       instead: the local bbox axis with the largest extent that the
       placement rotation leaves HORIZONTAL is the long axis, and its
       world bearing is `yawU`.

       ONLY WHEN THERE IS A BASE ROLL/PITCH TO RECOVER. With
       `r0 == p0 == 0` -- every Z-up prop, which is every streetlight,
       signal, bin and tree on the strip -- `yawU` is pinned to `yaw0` and
       the residual is the identity, so the authored op list is
       byte-for-byte what `toss_prim` used to write and nothing about a
       Z-up prop's pose changes. The measurement only ever runs for an
       asset that carries an axis-up correction.

    2. IT SEATS ON A BOUNDING BOX -- see `_points_min_z` for the 0.788 m
       floating streetlight that comes out of that. Seating here is
       points-based and happens AFTER the rotation is authored, which is
       the whole of the fix-floating-debris lesson: rotate first, then
       measure the real geometry, then translate.
    """
    from pxr import Gf, Sdf, UsdGeom

    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return False, None
    xf = UsdGeom.Xformable(prim)
    vals = {}
    for op in xf.GetOrderedXformOps():
        vals[op.GetOpName().split(":")[-1]] = op.Get()
    t = vals.get("translate") or Gf.Vec3d(0.0, 0.0, 0.0)
    sc = vals.get("scale")
    rxyz = vals.get("rotateXYZ")
    if rxyz is not None:
        roll0, pitch0, yaw0 = float(rxyz[0]), float(rxyz[1]), float(rxyz[2])
    else:
        roll0 = float(vals.get("rotateX") or 0.0)
        pitch0 = float(vals.get("rotateY") or 0.0)
        yaw0 = float(vals.get("rotateZ") or 0.0)

    if abs(roll0) > 0.01 or abs(pitch0) > 0.01:
        yaw_u = _long_axis_bearing(stage, path, roll0, pitch0, yaw0)
        if yaw_u is None:
            yaw_u = yaw0
    else:
        yaw_u = yaw0

    z_pre = _points_min_z(stage, path)

    xf.SetXformOpOrder([])
    xf.AddTranslateOp().Set(Gf.Vec3d(float(t[0]) + dx, float(t[1]) + dy,
                                     float(t[2])))
    xf.AddRotateZOp().Set(yaw_u + float(yaw_jitter_deg))
    # ORDER IS Rz . Ry . Rx . (residual), matching `toss_prim`'s own
    # comment: roll is applied first about the prop's long axis, and a
    # pitch on top of that stands a car already on its side onto its nose.
    if abs(float(pitch_deg)) > 0.01:
        xf.AddRotateYOp().Set(float(pitch_deg))
    if abs(float(roll_deg)) > 0.01:
        xf.AddRotateXOp().Set(float(roll_deg))
    res = (roll0, pitch0, _wrap180(yaw0 - yaw_u))
    if any(abs(v) > 0.01 for v in res):
        xf.AddRotateXYZOp(UsdGeom.XformOp.PrecisionFloat, "axisUp").Set(
            Gf.Vec3f(*[float(v) for v in res]))
    if sc is not None:
        xf.AddScaleOp().Set(sc)

    gap, _ok = _seat_prim(stage, path, z_pre, ground_z=ground_z)
    return True, gap


def _normalize_yaw_op(stage, path):
    """THE ROTATEXYZ TRAP, and it is NOT tree-specific -- this was first
    caught while wiring `vegetation.tip_tree` for street trees but it
    turned out to also silently corrupt every OTHER prop this pass touches
    through `tornado.toss_prim` (felled poles, tossed furniture, cars),
    which is why this helper is general and is called before every
    `toss_prim`/`tip_tree` invocation in `_apply_action`, not just the
    tree branch.

    `scene_generator.apply_placements` authors `translate` / `rotateXYZ`
    (a (roll, pitch, yaw) triple) / `scale` for EVERY generic placement it
    writes -- streetlight, bench, car, street tree, all of it -- never the
    bare `rotateZ` float `vegetation.tip_tree`'s own `_apply()` AND
    `tornado.toss_prim` both look for BY NAME (`"rotateZ" in vals` /
    `vals.get("rotateZ")`; `toss_prim`'s own docstring: "a prim placed by
    `apply_placements` carries translate + rotateZ + maybe scale"). That
    docstring is true of the SUBURB pipeline's own `_ref` helper (which
    does author a bare `rotateZ`) and false of the CITY generator's
    `apply_placements` (measured directly against its source AND against
    live `toss_prim` output, 2026-09-01 -- `_set_xform_ops` calls
    `AddRotateXYZOp()` unconditionally, and every asset in `asset_sets/
    urban.yaml` is "centimetre-authored and Z-up... no axis-up" per that
    file's own line-15 comment, so roll/pitch are 0 for every category
    this pass acts on, not just trees).

    MEASURED CONSEQUENCE, bare `usd-core`, no Kit: a prim placed at yaw 30
    deg, then `toss_prim(..., yaw_jitter_deg=15)` WITHOUT this fix, comes
    out at `rotateZ = 15`, not `45` -- `old_yaw = float(vals.get("rotateZ")
    or 0.0)` silently reads 0 because the op is named `rotateXYZ`. This is
    NOT the low-severity "canopy facing" cosmetic the tree case alone would
    be: for a CAR, `car_pose`'s own `yaw_delta_deg` is DEFINED as a delta
    off the car's existing heading (`_wrap180(target - long_axis_deg)`), so
    losing `old_yaw` puts every tossed/felled/car prop at an absolute
    heading equal to its OWN jitter/delta alone -- never assume it is
    harmless without this fix. `tests/test_tornado_street.py`'s own
    apply-side stage test pins the fix directly (a placement at a
    non-zero starting yaw keeps that yaw as the base for its post-fell
    facing).

    THE FIX IS HERE, NOT IN `tornado.py`/`vegetation.py`: both are shared
    with the suburb pipeline (where the assumption is correct, since its
    own `_ref` helper really does author a bare `rotateZ`) and neither is
    this stream's file to edit. Rewrite the prim's own op order --
    translate + rotateZ(yaw) + scale -- BEFORE handing it to `toss_prim`/
    `tip_tree`, so their by-name reads find what they expect. A no-op if
    the prim already carries a bare `rotateZ` (nothing to rewrite) or has
    no `rotateXYZ` at all; DELIBERATELY LEFT ALONE (returns without
    touching anything) if `rotateXYZ` carries a non-trivial roll or pitch
    -- not the flat case this fix targets on the urban asset set, and
    collapsing it to `rotateZ` would silently discard real geometry rather
    than recover a yaw.
    """
    from pxr import Gf, Sdf, UsdGeom

    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    names = [op.GetOpName().split(":")[-1] for op in ops]
    if "rotateXYZ" not in names or "rotateZ" in names:
        return
    vals = {}
    for op in ops:
        vals[op.GetOpName().split(":")[-1]] = op.Get()
    rxyz = vals.get("rotateXYZ")
    if rxyz is None:
        return
    roll, pitch, yaw = float(rxyz[0]), float(rxyz[1]), float(rxyz[2])
    if abs(roll) > 0.5 or abs(pitch) > 0.5:
        return
    t = vals.get("translate") or Gf.Vec3d(0.0, 0.0, 0.0)
    sc = vals.get("scale")
    xf.SetXformOpOrder([])
    xf.AddTranslateOp().Set(t)
    xf.AddRotateZOp().Set(yaw)
    if sc is not None:
        xf.AddScaleOp().Set(sc)


def _tip_azimuth(stage, path, azimuth_deg):
    """The azimuth to hand `vegetation.tip_tree` so the tree falls toward
    the WORLD bearing *azimuth_deg*.

    `tip_tree` rebuilds the prim as `translate -> rotateZ -> orient ->
    scale`, and builds its tip axis from `azimuth_deg` in the frame
    UNDERNEATH that `rotateZ` -- so the placement's own yaw is applied on
    top of the fall direction and the tree comes down at
    `azimuth_deg + rotateZ`, not at `azimuth_deg`. MEASURED on the C1
    strip: the three street trees carry placement yaws of 270.7, 32.4 and
    269.2 degrees (`apply_placements` draws a random yaw for every prop),
    so a corridor whose wind blows at bearing 63 put one tree down at 334
    and another at 96 -- windthrow scattered to the compass, which is the
    exact failure `TREE_FALL_JITTER_DEG`'s own comment says must not
    happen ("a stand all pointing within five degrees reads as a logging
    operation" -- and one pointing every which way reads as no event at
    all).

    Subtracting the yaw here is a CALLER-SIDE correction, deliberately:
    `vegetation.tip_tree` is shared with the suburb pipeline, whose own
    `_ref` helper authors the same bare `rotateZ` and whose outputs are
    snapshot-tested, so the op order cannot be changed from under it.
    """
    from pxr import Sdf, UsdGeom

    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return float(azimuth_deg)
    yaw = 0.0
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpName().split(":")[-1] == "rotateZ":
            yaw = float(op.Get() or 0.0)
            break
    return float(azimuth_deg) - yaw


#: `"uproot"`'s root-plate sizing, `vegetation.wind_tree`'s own formula
#: (`r_plate = clamp(trunk_r * 1.5, 0.28, 0.85)`, `lift = r_plate * 0.5`)
#: run against a FIXED representative trunk radius rather than a per-tree
#: measurement -- see `_apply_tree_level`'s own docstring for why.
_TREE_DEFAULT_TRUNK_R_M = 0.32


def _apply_tree_level(stage, a, ssf=1.0, ground_z=0.0, root_plates=True,
                      root_material_path=""):
    """`a["action"] == "level"` (a `street_tree` record). `geom is None`
    (pristine/limbed) is a v1 no-op -- see the module docstring's "TREES"
    coverage for why `limbed` gets no live crown-thinning this round.
    `"lean"` is root-sprung (no seat band, no lift -- still standing on
    its own roots). `"uproot"` covers BOTH `fallen` and v1's `snapped`
    stand-in.

    NO `vegetation.survey()` CALL HERE, ON PURPOSE -- and this is worth
    recording because the obvious-looking implementation (measure the
    tree's own trunk radius, size the root-plate lift off it, the way
    `wind_tree`'s own `"uproot"` branch does) is actively WRONG on the
    urban preset this was built against. TWO independent reasons, both
    measured 2026-09-01:

      1. `trunk_r` is not something `survey()` computes -- it is published
         as a SIDE EFFECT of `wood_debris()` measuring the bole with
         `fracture.prim_to_mesh` (`vegetation.py` around line 1779,
         "PUBLISHED, because anything else built at the foot of this tree
         has to be sized against the TREE"). This pass deliberately does
         not call `wood_debris` (the full fracture-based debris pipeline
         this stream's brief scoped OUT of v1), so `info.get("trunk_r")`
         would be `None` every single time either way.
      2. A CITY street tree commonly composes with `IsInstance() == True`
         (`instance_placements: true` presets), and `survey()`'s own first
         check is `if root.IsInstance(): print [veg] ... IS INSTANCEABLE
         ... and return an EMPTY dict` -- `Usd.PrimRange` cannot descend an
         instance, by USD's own rule, so this is not a bug in `survey`, it
         is the function doing exactly what it documents. Calling it here
         would print that warning for every tipped tree and still hand
         back nothing useful.

    So every uprooted tree gets the SAME representative root-plate lift
    (`_TREE_DEFAULT_TRUNK_R_M`, `wind_tree`'s own fallback constant) rather
    than a per-species one -- a real, minor loss of variety, acceptable
    because this pass never authors the root-BALL MESH itself (`root_
    ball`/`root_plate` -- also out of scope), so the only thing the lift
    affects is how high `tip_tree` raises the trunk's own base before
    seating. `tip_tree`, unlike `survey`, works fine on an instanced prim
    -- it only edits the prim's OWN `xformOpOrder` attribute, never
    anything "inside" the referenced subtree (the same reason `toss_prim`
    already works on an instanced streetlight/car).

    Returns True if a tip was attempted (not a guarantee the seat
    bisection landed inside its band -- `tip_tree` itself falls back to
    `lean_min_deg` and stays partly buried rather than fail)."""
    from . import vegetation as veg

    geom = a.get("geom")
    if geom is None:
        return True
    _normalize_yaw_op(stage, a["path"])
    if geom == "lean":
        veg.tip_tree(stage, a["path"], a["lean_deg"],
                    azimuth_deg=_tip_azimuth(stage, a["path"],
                                             a["azimuth_deg"]),
                    lift_m=0.0, seat_band=None)
        return True

    # ---- "uproot": LAY IT DOWN, then seat it on its BUTT ---------------
    #
    # ROUND 4. The shipped call passed `seat_band=(-1.1, -0.15)` with
    # `lean_min_deg=46`, and on every species in the urban pool the
    # bisection bottomed out and returned 46.0 -- a tree standing at 46
    # degrees off vertical with its crown 4.0-9.6 m under the plate, which
    # from 60-90 m reads as an intact standing tree (the round-4 photo,
    # `C1_top.png`). MEASURED, `tools/tornado_tree_fall_probe.py`:
    #
    #     Shumard_Oak  (10.9 m, crown r 5.23)  lean 46 -> min_z -3.97
    #     Black_Oak    (19.7 m, crown r 12.71) lean 46 -> min_z -9.49
    #     both settle on lean_min_deg for bands down to (-4.0, -0.15)
    #
    # The band cannot be retuned into working, because the quantity it
    # bisects on -- the WHOLE TREE's lowest point -- falls MONOTONICALLY
    # with lean for a leafed crown (Shumard: -0.53 at lean 0, -3.07 at 30,
    # -4.13 at 46, -5.24 at 82). "Seat the crown" and "lay the tree down"
    # are the same trade in opposite directions, and a fallen tree that
    # does not read fallen is worth nothing.
    #
    # So the tree is laid down at the lean that was drawn and seated on the
    # ONE datum that is physically the ground contact: the BUTT, which is
    # `tip_tree`'s own pivot (its docstring: "THE PIVOT IS THE LOCAL
    # ORIGIN, which on every tree in this library is the base of the
    # trunk"). Putting the pivot at `ground_z + TREE_ROOT_LIFT_M` puts the
    # butt exactly a root plate's height off the ground and runs the trunk
    # down to grade along its length -- no bbox, no points, no bisection,
    # one matrix read. The crown then presses INTO the plate, which is
    # what a windthrown crown in leaf does and what the round-4 brief
    # explicitly allows ("slight ground intersection of the canopy is
    # fine"); the part of it that matters is the part above grade, and
    # that is now a flattened disc of foliage lying a crown-radius away
    # from its own stump instead of a ball centred on it.
    from pxr import Gf, Sdf, Usd, UsdGeom

    path = a["path"]
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    crown_r = None
    if prim and prim.IsValid():
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_,
                                UsdGeom.Tokens.render])
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if not r.IsEmpty():
            crown_r = 0.5 * max(float(r.GetSize()[0]), float(r.GetSize()[1]))
            height = float(r.GetSize()[2])
        else:
            height = 0.0
    else:
        height = 0.0

    veg.tip_tree(stage, path, a["lean_deg"],
                 azimuth_deg=_tip_azimuth(stage, path, a["azimuth_deg"]),
                 lift_m=0.0, seat_band=None)

    # Seat the BUTT. `tip_tree` rebuilds translate/orient/scale, so the
    # pivot's world z is just the prim's own local-to-world translation.
    ok = False
    if prim and prim.IsValid():
        xf = UsdGeom.Xformable(prim)
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        m = xc.GetLocalToWorldTransform(prim)
        z_butt = float(m[3][2])
        gz = 0.0 if ground_z is None else float(ground_z)
        dz = (gz + TREE_ROOT_LIFT_M * float(ssf)) - z_butt
        for op in xf.GetOrderedXformOps():
            if op.GetOpName().split(":")[-1] != "translate":
                continue
            t = op.Get() or Gf.Vec3d(0.0, 0.0, 0.0)
            op.Set(Gf.Vec3d(float(t[0]), float(t[1]),
                            float(t[2]) + dz / _parent_z_scale(stage, prim)))
            ok = True
            break

    # THE ROOT PLATE -- `vegetation.root_plate`, the "signature feature"
    # its own docstring names: "a tree that was cut leaves a stump, a tree
    # that blew over leaves a two-to-four-metre wheel of earth standing on
    # edge with the trunk running away from it". Authored as a SIBLING of
    # the tree (never a child: the tree prim is about to be rotated and a
    # child would rotate with it), at the tree's own translate.
    if root_plates and ok and prim and prim.IsValid():
        try:
            import random as _random
            import zlib as _zlib
            xf = UsdGeom.Xformable(prim)
            tv = None
            for op in xf.GetOrderedXformOps():
                if op.GetOpName().split(":")[-1] == "translate":
                    tv = op.Get()
                    break
            if tv is not None:
                r_pl = max(TREE_PLATE_R_RANGE_M[0],
                           min(TREE_PLATE_R_RANGE_M[1],
                               TREE_PLATE_R_FRAC * max(4.0, height)))
                parent, name = path.rsplit("/", 1)
                seed = _zlib.crc32(path.encode("utf-8")) & 0x7FFFFFFF
                veg.root_plate(
                    stage, "{0}/{1}_rootplate".format(parent, name),
                    float(tv[0]) / ssf, float(tv[1]) / ssf, r_pl,
                    float(a["azimuth_deg"]) + 180.0, _random.Random(seed),
                    mat_prim_path=root_material_path, ssf=ssf)
        except Exception as exc:                               # noqa: BLE001
            print("[tornado_street] root_plate skipped for {0}: {1}".format(
                path, exc))

    if crown_r is not None and crown_r > TREE_STREET_MAX_CROWN_R_M:
        print("[tornado_street] WARNING {0}: crown radius {1:.1f} m is a "
              "PARK specimen, not a kerb tree -- a rigid windthrow about "
              "its base cannot read fallen at this size (crown offset "
              "never clears its own radius). Fix the POOL: "
              "`asset_sets/urban.yaml` already separates `street_trees` "
              "(Shumard_Oak 11.4 m, Douglas_Fir 6.0 m) from `trees` "
              "(Black_Oak 19.7 m / 25.4 m crown -- \"a park specimen, not "
              "a kerb tree\", its own comment).".format(path, crown_r))
    return True


def _apply_action(stage, a, ssf, ground_z=0.0, root_plates=True,
                  root_material_path=""):
    """Dispatch one `plan_street` action onto the stage. Returns
    `(ok, gap_m)` -- `ok` is True if applied (or correctly a no-op --
    `carry` on a prim that is already gone is not a failure), `gap_m` is
    the posed prop's measured distance off its seating datum in stage z
    (`None` when nothing was seated). `ssf` converts the action's own
    METRE distances to stage units (`apply_street` derives it from the
    stage's own `metersPerUnit`, the same convention `place_holder`'s
    `* ssf` on every translate already uses in the launcher).

    `ground_z` is the seat FLOOR in world stage units -- see `_seat_prim`.
    `None` disables it and preserves each prop's own pre-pose contact."""
    from pxr import Sdf

    action = a.get("action")
    path = a.get("path")
    if action == "carry":
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if not prim or not prim.IsValid():
            return False, None
        return bool(prim.SetActive(False)), None
    if action in ("fell", "toss"):
        dx = math.cos(math.radians(a["bearing_deg"])) * a["dist_m"] * ssf
        dy = math.sin(math.radians(a["bearing_deg"])) * a["dist_m"] * ssf
        return _pose_prim(stage, path, dx, dy, a["roll_deg"],
                          a["yaw_jitter_deg"],
                          pitch_deg=a.get("pitch_deg", 0.0),
                          ground_z=ground_z)
    if action == "level":
        return _apply_tree_level(stage, a, ssf=ssf, ground_z=ground_z,
                                 root_plates=root_plates,
                                 root_material_path=root_material_path), None
    if action == "car_pose":
        return _pose_prim(stage, path, a["dx"] * ssf, a["dy"] * ssf,
                          a["roll_deg"], a["yaw_delta_deg"],
                          pitch_deg=a.get("pitch_deg", 0.0),
                          ground_z=ground_z)
    return False, None


def apply_street(stage, actions, *, min_tipped=0, min_moved=0,
                 ground_z=0.0, root_plates=True, root_material_path="",
                 verbose=True):
    """Apply a `plan_street()` action list onto a live stage.

    Returns a counts dict:

        {"by_kind": {"felled": {"fell": N, "carry": N}, "carried": {...},
                     "tree": {"level": N}, "car": {"car_pose": N}},
         "n_actions": N, "n_applied": N, "n_failed": N,
         "n_thrown": N, "n_forced": N, "n_correlated": N,
         "min_tipped": int(min_tipped), "min_moved": int(min_moved),
         "n_seated": N, "n_off_grade": N, "max_gap_m": float,
         "worst_gap_path": str | None}

    `by_kind` sub-dicts only carry the `action` names actually seen for
    that `kind` -- an empty/absent kind means nothing of that family was
    ever acted on for this call, not an error.

    `min_tipped` is NEVER used to decide anything here -- `plan_street`
    already resolved the review floor into `"forced": True` actions before
    this function ever sees them (see that function's own `min_tipped=`
    kwarg). It is accepted here purely so the printed report (`verbose`)
    and the returned dict can say what floor the caller asked `plan_street`
    for, without the caller having to thread it through twice by hand.

    `ssf` (stage units per metre) is read directly off the stage's own
    `metersPerUnit` (`UsdGeom.GetStageMetersPerUnit`, the same one-line
    formula `scene_prep.get_stage_meters_per_unit` wraps in the launcher
    utils) rather than taken as a parameter -- every action's own
    distances (`dist_m`, `dx`, `dy`, ...) are real metres regardless of
    the stage's own unit convention, and a caller that already has a stage
    has this for free; NOT a parameter is one fewer thing the bench has to
    get right to match this function's own convention.

    `ground_z` -- the seat FLOOR, in world stage units (0.0, the plate, by
    default). Every posed prop is re-seated onto its OWN pre-pose lowest
    mesh point, never below `ground_z`; pass `None` to disable the floor
    and preserve the pre-pose contact exactly (right for a scene whose
    street props stand on something raised). See `_pose_prim` and
    `_points_min_z` for what this replaces and why.

    `root_plates` -- author `vegetation.root_plate`'s wheel of soil at the
    butt of every UPROOTED tree (True by default; the signature that
    separates a windthrown tree from a felled one at any distance).
    `root_material_path` is bound to it when given -- pass a soil/dirt
    `UsdShade.Material` path from the cell's own material scope; with ""
    the plate falls back to its own dark-earth `displayColor`.

    `n_seated`/`n_off_grade`/`max_gap_m` REPORT THE SEATING, measured from
    mesh points after the pose is authored. `n_off_grade` counts props
    further than `SEAT_TOL_M` (2 cm) from their datum -- on a correct run
    it is 0, and it is the number the round-4 D5 defect ("a bus half-sunk
    into the plate, a lamp pole floating") would have shown as 9 of 11.

    `verbose=True` (the default) prints a `[tornado_street]`-prefixed
    per-action-family table plus the thrown/forced/correlated counts --
    the SAME information `TornadoCityApp.street_pass` used to print under
    its own `[ut]` prefix, now printed here so every caller gets it for
    free; the launcher no longer prints this table itself; and also prints
    one line per action that raised (never silently swallowed). `False`
    silences all of it -- the returned dict is authoritative either way.
    """
    from pxr import UsdGeom

    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    ssf = 1.0 / mpu if mpu and mpu > 0 else 1.0

    gz = ground_z if ground_z is None else float(ground_z) * ssf

    by_kind = {}
    n_applied = n_failed = n_seated = n_off = 0
    max_gap, worst = 0.0, None
    for a in actions or ():
        kind, action = a.get("kind"), a.get("action")
        row = by_kind.setdefault(kind, {})
        row[action] = row.get(action, 0) + 1
        gap = None
        try:
            ok, gap = _apply_action(stage, a, ssf, ground_z=gz,
                                    root_plates=root_plates,
                                    root_material_path=root_material_path)
        except Exception as exc:                                # noqa: BLE001
            ok = False
            if verbose:
                print("[tornado_street] FAILED {0} {1} on {2}: {3}".format(
                    kind, action, a.get("path"), exc))
        if ok:
            n_applied += 1
        else:
            n_failed += 1
        if gap is not None:
            n_seated += 1
            gap_m = float(gap) / ssf
            if abs(gap_m) > SEAT_TOL_M:
                n_off += 1
            if abs(gap_m) > abs(max_gap):
                max_gap, worst = gap_m, a.get("path")

    all_actions = actions or ()
    out = {
        "by_kind": by_kind,
        "n_actions": len(all_actions),
        "n_applied": n_applied,
        "n_failed": n_failed,
        "n_thrown": sum(1 for a in all_actions if a.get("thrown")),
        "n_forced": sum(1 for a in all_actions if a.get("forced")),
        "n_correlated": sum(1 for a in all_actions if a.get("correlated")),
        "min_tipped": int(min_tipped),
        "min_moved": int(min_moved),
        "n_seated": n_seated,
        "n_off_grade": n_off,
        "max_gap_m": round(float(max_gap), 4),
        "worst_gap_path": worst,
    }

    if verbose:
        print("[tornado_street] {0} action(s) ({1} applied, {2} failed)"
              .format(out["n_actions"], out["n_applied"], out["n_failed"]))
        for kind in ("felled", "carried", "tree", "car"):
            row = by_kind.get(kind, {})
            print("    {0:<8} {1}".format(
                kind, ", ".join("{0}={1}".format(k, v)
                               for k, v in sorted(row.items())) or "-"))
        print("    car: {0} thrown, {1} forced (min_tipped={2}, "
              "min_moved={3})  felled: {4} in a correlated run".format(
                  out["n_thrown"], out["n_forced"], out["min_tipped"],
                  out["min_moved"], out["n_correlated"]))
        print("    seat: {0} prop(s) points-seated, {1} off grade by more "
              "than {2:.0f} cm; worst {3:+.3f} m{4}".format(
                  out["n_seated"], out["n_off_grade"], SEAT_TOL_M * 100.0,
                  out["max_gap_m"],
                  "" if not worst else " (" + str(worst) + ")"))

    return out
