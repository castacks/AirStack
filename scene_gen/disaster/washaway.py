"""washaway stage — what storm surge does to a house, and to a car, that
`surge.py`'s water does not: floats them, scours a yard, or leaves nothing
but a slab.

THE GAP THIS FILLS
-------------------
`surge.py` authors the water — an inundation sheet, ponds, wrack windrows,
washover fans — and it is explicit that this is all it does (its own module
docstring: "the water sibling of `wind_flow.py`"). `hurricane_flow.py`
authors wind damage on a house, and it is equally explicit about what it
will NOT do: `wreck_building` *raises* on `level="swept"`, naming this module
by its package (`disaster.surge`) as the one that owns any bare-slab state.
So there is a house-shaped hole between the two: nothing in the repo puts
water-caused, non-wind damage ON a house. A scene with `surge.py`'s water
drawn under an ordinary, square, wind-only-damaged house is exactly the
complaint that motivated this file — the water is present but did nothing.

A SECOND, IDENTICAL HOLE EXISTED FOR VEHICLES, and it was worse: this
module's own docstring used to scope itself to houses ONLY, even though
`hurricane.py` states TWICE that cars are moved by water rather than wind
and names `disaster.surge` (this file's actual owner in practice) as the
module responsible. Nothing anywhere floated, shifted or even wet a car —
`_plans/hurricane_coverage_audit.md`'s rank-#1 finding. `car_surge_state`/
`car_shift_spec`/`apply_car_washaway`, below, are the vehicle counterpart
of `house_surge_state`/`shift_spec`/`apply_washaway` — see "THE VEHICLE
LADDER" for why they are a NEW ladder with its own thresholds rather than
the house one reused at a smaller scale.

THE LADDER, AND THE PHYSICS BEHIND IT
----------------------------------------
Wind loads a building from above and from one face. Surge does something
structurally different: it loads the walls laterally at the waterline and,
past a threshold, it FLOATS the whole building off its foundation — the
mechanism a wind ladder has no rung for at all. The five states below are
the recognised post-storm survey progression (StEER/FEMA MAT field reports
after Katrina, Ike and Michael):

    wet        water inside, nothing moved. A mud line on the walls. The
               overwhelmingly common outcome — most surge-flooded houses are
               this and nothing more.
    scoured    the ground around and under the house stripped to bare mud —
               yard, driveway, landscaping gone — while the house itself is
               still square on its foundation.
    shifted    THE SIGNATURE IMAGE OF SURGE. Buoyant uplift on a slab-on-
               grade wood frame overcame its own dead load and anchorage;
               the house floated, travelled downstream of the flow, and
               settled again, usually rotated and off-level. Reads instantly
               from the air — a house sitting askew beside its own bare
               slab — and it is what the user asked this module to produce.
    collapsed  the frame came apart in the water. A heap, not a building.
    swept      nothing left but the bare slab (and, in a fuller pass than
               this one attempts, plumbing stubs). Roueche et al.'s finding
               on 3,016 hurricane homes is the reason this state is rare and
               gated hard: *every* home in that sample that suffered complete
               destruction was surge-associated, never wind alone, and even
               within the surge footprint complete destruction was 3.1% of
               homes — so `swept` needs both the deepest water on the plate
               AND a weak construction draw, not merely "the top of a
               ladder" (see `house_surge_state`).

`house_surge_state` is a depth+vulnerability ladder in the same shape
`hurricane.house_level_for_intensity` already uses for wind (`effective =
raw / resistance(vuln)`) — reused here because it is the right idea
independently of wind: a badly-anchored 1970s slab house floats free in
LESS water than a properly tied-down, elevated one, for the same reason a
weak building fails at a lower wind speed. Every constant that blend is
built from is marked TUNED, NOT SOURCED where it is one, because — unlike
`hurricane.py`'s wind onsets, which come from Marshall's and HAZUS's gust
tables — there is no published depth-to-displacement curve for a
residential slab-on-grade house. What IS anchored: the calibration target
itself (0.3-0.8 m almost all `wet`, `shifted` starting around 1.5-2 m,
`swept` gated on `surge.DEFAULTS["swept_depth_m"]`'s own 2.5 m, itself
anchored to Kennedy et al.'s Bolivar/Ike destroyed-zone still-water range).

THE TWO THINGS THAT MAKE IT READ FROM THE AIR
-------------------------------------------------
1. `shift_spec` displaces DOWN-FLOW and turns the house — never a small
   nudge (the user's own line: "a house nudged 0.5 m looks like a bug").
   `apply_washaway` reuses `tornado.toss_prim` for the actual move, exactly
   as directed: that function MEASURES the prop's own bounding box to seat
   it after rotation rather than guessing a lift, which is precisely what a
   whole-house rigid transform needs and precisely why this module writes
   no seating maths of its own. `flow_bearing_deg` is a single scene-wide
   number (see "WIRING THIS TO surge.py" below) so every shifted house on
   one plate agrees with the same current instead of scattering in random
   directions, which would read as noise rather than as water.
2. `raft_specs`/`build_rafts` put debris ON the water so the flood surface
   is not an empty mirror: a `vegetation` mat class plus timber, sheet goods
   and wall sections collected against the upstream (seaward) face of
   whatever is still standing, plus a light background scatter across open
   water. Geometry reuses `planks._box`/`planks._FACES` — the exact routine
   the tornado debris field already uses — merged one mesh per debris
   class, the same few-hundred-boards-into-one-mesh trade `planks.build`
   makes and for the same reason: a raft field is per-PLACE, not
   per-archetype, so it has to stay cheap however many houses it collects
   against. `vegetation` was ADDED, not always present — see "THE RAFT
   FIELD WAS ALL BUILDING MATERIAL" below, rank #2 in the coverage audit.

THE RAFT FIELD WAS ALL BUILDING MATERIAL
---------------------------------------------
`_RAFT_KIND_WEIGHTS` used to be `timber`/`sheet`/`wall` only — every raft on
every plate was construction debris, even though this codebase's OWN
citation for its *ground* debris mix (Lee County's Ian collection, 734,136
yd³ vegetative against 285,282 yd³ C&D — `hurricane.VEG_SHARE`/`BLD_SHARE`,
a 72/28 split) says vegetation is the majority by more than 2:1. Water-borne
vegetation mats — dark, organic, torn-off canopy and understory tangled by
the current — are the strongest class separator against a muddy or
reflective water surface (`_plans/hurricane_coverage_audit.md` S2 row 2), and
there were none.

Fixed two ways, because one alone leaves a gap: `_RAFT_KIND_WEIGHTS` itself
now carries a `vegetation` entry sized at that same 72/28 split, so a bare
`raft_specs(cfg, region, rng, houses, depth_fn)` — the only call shape any
caller used before this fix, and still what a caller with no per-scene
intensity in hand should use — puts vegetation in the field AT ALL. But that
constant is a STATIC fallback, and the task was explicit: "thread
`hurricane.debris_mix` in, don't hardcode a second copy of the same number."
`raft_kind_weights(veg_share)` is that thread: a caller who already computed
`mix = hurricane.debris_mix(i, rng)` for the ground debris passes
`mix["leaf_litter"] + mix["limbs_fronds"]` in, and `raft_specs`'s optional
`kind_weights` argument uses THAT number instead of the fallback constant —
one source of truth, the fallback only covers the caller who has no
`debris_mix` result to hand across. This module still never imports
`hurricane` (see the `pxr`/import section below) — the number crosses as a
plain float, the same "wire a number, not a config dict" contract
`water_level_m`/`flow_bearing_deg` already use.

WHAT THIS MODULE DELIBERATELY DOES NOT DO
----------------------------------------------
`collapsed` is authored as one heavily tilted, lightly displaced rigid
transform on the house's existing prim (`collapse_spec` + `toss_prim`
again) — NOT a real fracture. A true "frame came apart" collapse belongs to
`fracture.py`/`settle.py`, the way `wind_flow`'s `roof_collapsed` and above
already use them, and reaching for that machinery from a brand-new,
single-file module would mean depending on per-house fragment/material
context (`items`, `tag`, `nrng`, `planks_mats` — see
`hurricane_flow.wreck_building`'s own kwargs) that this module has no way to
assemble on its own. Flagged here, and again in the verification report, as
the acknowledged simplification: a rigid, badly-listing house standing in
for a real pile of broken framing.

No fluid simulation, no physics body, nothing animates — same contract
`surge.py` states for the water, for the same reason: everything here is
authored once as static geometry and a rigid transform.

THE VEHICLE LADDER — CARS ARE NOT SMALL HOUSES
---------------------------------------------------
`car_surge_state`/`car_shift_spec`/`apply_car_washaway` mirror
`house_surge_state`/`shift_spec`/`apply_washaway`'s SHAPE — a depth ladder,
a pure-geometry pose spec, a stage-touching apply function — but NONE of
the house ladder's numbers. Two reasons, both load-bearing:

1. A CAR IS FAR LIGHTER RELATIVE TO ITS DISPLACED VOLUME THAN A HOUSE. NOAA/
   NWS's own "Turn Around Don't Drown" depth table — the vehicle equivalent
   of the calibration exercise `RESIST_LO`/`RESIST_HI` do for houses — puts
   the onsets at roughly 15/30/60 cm (floorpan wetted / many cars float /
   most vehicles including SUVs carried away), not the house ladder's
   0.75-2.8 m. `CAR_EFF_SWAMP_M`/`CAR_EFF_FLOAT_M` sit an order of magnitude
   below `EFF_SCOUR_M`/`EFF_SHIFT_M` for exactly this reason — see "CARS ARE
   NOT HOUSES" above `_CAR_CUTS` for the citation.
2. A CAR HAS NO CODE ERA. `house_surge_state` divides depth by
   `_surge_resistance(vuln)`, where `vuln` comes from
   `hurricane.draw_vulnerability` — a construction-era draw a vehicle does
   not have and this module will not invent one for. What a car DOES have
   is mass and ground clearance, and the only proxy for those this module
   can measure is the vehicle's own LENGTH, exactly the argument
   `tornado.car_pose` already makes for its (unrelated, wind-driven) model.
   `car_surge_state`'s optional `length_m` and the private
   `_car_resistance` it feeds do that job; `RESIST_LO`/`RESIST_HI`/`_CUTS`
   above are UNTOUCHED and never read by any car function.

A FLOATED CAR ENDS UP SOMEWHERE, and where is the characteristic image: not
adrift in open water, but against the downstream side of whatever stopped
it — a house, a fence line, a tree, another car. That is `car_shift_spec`'s
optional `blocked` argument: pass `tornado.car_blockers(...)`'s predicate
(the SAME one a hurricane launch script would build for its road-blockage
pass — see the audit's rank #3) plus the car's current `x`/`y`, and the
function marches the drift in `CAR_MARCH_STEP_M` steps exactly the way
`tornado.car_pose` marches a wind-thrown car, arresting it at the first hit
and adding a jam pitch (sometimes a steeper "mounted partly onto it" pitch —
`CAR_MOUNT_P` — for the "not always flush against it" case). Reusing
`car_blockers`'/`car_pose`'s MARCHING MECHANISM is not the same as reusing
`car_pose`'s wind-lofting PROBABILITIES (`CAR_P_MOVE`/`CAR_P_TIP`) — those
stay exactly where `hurricane.py`'s own docstring says they belong: nowhere
near a hurricane car, which is not wind-thrown.

`piled` (from `car_surge_state`) is a PREDICTION that the march will find
something, made from depth alone before any blocker geometry is
consulted — it cannot be anything else, since `car_surge_state` takes no
position. If the caller then calls `car_shift_spec` with a real `blocked`
predicate and the march finds nothing, the car still gets its ordinary
floated drift and settle tilt, just without the extra jam pitch — a
"piled" car that did not, in fact, find a pile is not visually wrong the
way `tornado.car_pose`'s undemoted `nose` pose was (a car standing on its
bumper with nothing in front of it): an unobstructed floated car sitting a
little off level is a perfectly ordinary outcome, so this module does not
bother renaming the state the way that function renames `nose` to `side`.

`pxr` IS IMPORTED ONLY INSIDE THE STAGE-TOUCHING ENTRY POINTS
--------------------------------------------------------------------
`apply_washaway`, `build_rafts`, and the private helpers reachable only
from them (`_measure_house`, `_merge_boxes`, `_slab_material`) — plus, for
the vehicle pass, `apply_car_washaway` AND `_measure_car`, the second one
DELIBERATELY an entry point in its own right rather than a helper
`apply_car_washaway` alone reaches: a caller needs a car's length BEFORE
`car_surge_state` decides whether it floats at all, which is before
`apply_car_washaway` would ever run — see `_measure_car`'s own docstring.
Every other function — the whole ladder, `shift_spec`, `collapse_spec`,
`car_surge_state`, `car_shift_spec`, `raft_specs`, `raft_kind_weights`,
`resolve_cfg`, `knobs_from_env`, `summarise` — runs on a bare `python3`
with `pxr` absent, the `scour_relief.py` split this codebase uses
everywhere else. `surge`, `tornado`, `planks` and `vegetation` are imported
at module scope for their pure-Python helpers (`surge._as_random`,
`surge._bounds`, `surge._grad_deg`, `surge._draw`, `tornado._ladder`,
`tornado.toss_prim`, `planks._box`, `planks._FACES`, `planks.wood_material`,
`planks.skin_material`; `vegetation.bark_material`, ADDED for the floating
`log` raft's real bark texture — see `build_rafts`'s own docstring) — none
of those four modules import `pxr` at module scope either (verified by
reading them), so this chain stays importable with no USD on the path.
`hurricane` is deliberately NOT imported here at all — see "THE RAFT FIELD
WAS ALL BUILDING MATERIAL" above for why its number crosses as a plain
float instead.

WIRING THIS TO surge.py — TWO NUMBERS CROSS THE MODULE BOUNDARY
---------------------------------------------------------------------
The bug catalogue in `.agents/skills/build-hurricane-scenes/SKILL.md`
records that `surge.resolve_cfg` silently no-ops when handed the wrong
shape of dict — so this module never calls anything in `surge` with ITS OWN
`cfg`, and never asks `surge` to resolve a dict that came from here. The two
numbers a real scene needs to hand across explicitly are:

    flow_bearing_deg   -> pass `surge_cfg["shore_bearing_deg"]` as
                          `shift_spec`'s second argument. Both mean "the
                          compass-free bearing water moves ONSHORE", and
                          `surge.py`'s own docstring establishes the sign
                          convention this module reuses unchanged.
    water_level_m      -> set `DEFAULTS["water_level_m"]` (or the
                          `WASHAWAY_WATER_LEVEL_M` env var) from
                          `surge.water_level(surge_cfg)` before calling
                          `raft_specs` — rafts float AT the water surface
                          `surge.build_inundation` actually draws its L1
                          quad at, not at `surge._SURFACE_Z_M` (which is a
                          few centimetres above GRADE, meant for pond/wrack
                          geometry that sits near real ground, not for
                          anything floating on deep water).

Everything else this module needs from `surge` (`depth_fn`, per-house
`depth_m`, `houses`) is passed as a plain argument, never pulled through a
shared config dict.
"""

import math
import os

from . import planks
from . import surge
from . import tornado
from . import vegetation

# ---------------------------------------------------------------------------
# the damage ladder — plain module constants, not a `cfg` dict, because
# `house_surge_state`/`shift_spec`/`collapse_spec` take no `cfg` argument at
# all (matching `tornado.house_level_for_intensity`/`car_pose`, which tune
# the same way: named constants + optional keyword overrides, not a dict).
# ---------------------------------------------------------------------------

LEVELS = ("wet", "scoured", "shifted", "collapsed", "swept")

# Resistance multiplier applied to depth before the ladder is read:
# `eff = depth_m / resistance(vuln)` — the exact shape
# `hurricane._resistance` uses for wind, transposed to buoyant uplift. A
# badly-anchored slab-on-grade house (vuln=1) floats free in the LEAST real
# depth; a well-tied-down or elevated one (vuln=0) needs roughly twice as
# much. TUNED, NOT SOURCED: no study reports a 0..1 vulnerability-to-
# buoyant-uplift curve the way HUD's Charley survey does for wind, so this
# is shaped to be qualitatively consistent with the anchorage argument
# rather than fit to a dataset.
# WIDENED AT THE BOTTOM 2026-08-30, after the first render of the level-3
# plate put ONE house in `shifted` out of 43 flooded ones — a flooded
# neighbourhood in which nothing had moved.
#
# The top of this range was fine; the bottom was not. At `RESIST_LO = 0.80`
# the WEAKEST house on the plate still needed 2.0 x 0.80 = 1.6 m of standing
# water before it floated off its slab, and the deepest water any house on a
# 500 m plate actually stands in is about 1.9 m — so flotation was reachable
# only in the last few centimetres of the field, by almost nobody.
#
# 1.6 m is too high for that class regardless. An unanchored light wood frame
# or a pre-HUD manufactured home floats once the water is roughly floor-to-
# waist inside it — the dead load is small and the enclosed volume is large —
# which is the ~1.0-1.2 m range the post-Katrina and post-Ike surveys keep
# reporting for homes found off their foundations. 0.55 puts that class at
# 2.0 x 0.55 = 1.1 m, inside it.
#
# The TOP is unchanged on purpose, so this widens the spread rather than
# sliding it: a fortified/elevated house still needs 2.0 x 1.55 = 3.1 m, more
# than this plate ever offers, and correctly never moves. That widening IS
# the Marshall finding this pipeline already leans on elsewhere — "flaws in
# construction or poor attachment/anchoring can better explain sharp
# gradations in damage rather than a rapid change in wind speed" — applied to
# water: two neighbours in the same depth, one gone and one merely soaked, is
# the norm rather than the exception.
RESIST_LO = 0.55   # vuln = 1, weakest anchorage: floats at ~1.1 m
RESIST_HI = 1.55   # vuln = 0, best anchorage / elevation: needs ~3.1 m

# Effective-depth (metres, AFTER the resistance divide) onsets for the
# ladder below `swept`. Picked so that, blended across a realistic code-era
# population (`hurricane.CODE_ERAS`'s weighted mean vuln is ~0.51, giving a
# typical resistance ~1.17; the weakest pre-1976 stock bottoms out around
# 0.80), the RAW depth at which each state starts appearing lands on the
# calibration target in the module docstring: `scoured` picks up soon after
# 0.8 m (weak-house floor ~0.6 m, typical-house floor ~0.9 m); `shifted`
# starts at the WEAKEST houses around 1.5 m and is the typical-house onset
# around 2.3 m, giving ~14% of all flooded houses "shifted" on the level-3
# verification plate (`surge_m=2.8`) — a clear minority, not a majority; see
# the verification report for the measured histogram. The FIRST version of
# this ladder (`EFF_SHIFT_M=1.5`) measured 30% shifted on that same plate —
# visibly too many, "the neighbourhood looks bulldozed" territory — which is
# why the cut sits here and not at the more obvious "matches the docstring's
# range at the population mean" value. TUNED, NOT SOURCED for the exact cut
# values; the SHAPE (monotonic in effective depth) is the same ladder idiom
# `tornado._HOUSE_CUTS` uses.
EFF_SCOUR_M = 0.75
EFF_SHIFT_M = 2.0
EFF_COLLAPSE_M = 2.8

# One draw's worth of per-house noise in the same effective-depth units,
# so the ladder's boundaries are not contour lines on the flood field —
# `tornado._ladder`'s own argument for why every ladder in this codebase
# jitters. TUNED, NOT SOURCED.
LADDER_JITTER_M = 0.18

_CUTS = ((EFF_SCOUR_M, "wet"),
         (EFF_SHIFT_M, "scoured"),
         (EFF_COLLAPSE_M, "shifted"),
         (float("inf"), "collapsed"))

# `swept` is carved OUT of the `collapsed` bucket, gated on BOTH conditions
# independently rather than on one large ratio — see the module docstring's
# citation. The depth gate matches `surge.DEFAULTS["swept_depth_m"]` (2.5 m,
# itself anchored to Kennedy et al.'s Bolivar/Ike still-water destroyed-zone
# range) verbatim, as a cross-check rather than a re-derivation; a caller
# whose `surge_cfg` overrides that default should pass the same number in
# via `house_surge_state`'s `swept_depth_m` keyword so the two stay in sync.
SWEPT_DEPTH_M = 2.5
# Probability of `swept` (vs staying `collapsed`) once the depth gate is
# open, linear in `vuln` — a fortified house that reaches the deepest water
# on the plate is still ruined, but its debris is less likely to have gone
# anywhere; a pre-1976 slab is more likely to have nothing left at all.
# TUNED, NOT SOURCED.
SWEPT_P_LO = 0.12   # vuln = 0
SWEPT_P_HI = 0.65   # vuln = 1


def _surge_resistance(vuln):
    v = max(0.0, min(1.0, float(vuln)))
    return RESIST_HI + (RESIST_LO - RESIST_HI) * v


def house_surge_state(depth_m, vuln, rng, *, swept_depth_m=SWEPT_DEPTH_M,
                      jitter_m=LADDER_JITTER_M):
    """Per-house surge damage level, or `None` if the house is dry.

    `depth_m` is a single scalar — the caller's own
    `surge.depth_at(cfg, region, rng)(x, y)` or
    `surge.house_water_state(cfg, x, y, rng)["depth"]`; this function does
    not know about `surge.py`'s field at all, only the number. `vuln` is
    0..1 from `hurricane.draw_vulnerability` (0 = fortified/elevated, 1 =
    weakest pre-code slab) — the SAME per-house draw a caller already made
    for the wind ladder, reused here rather than re-rolled, because a house
    that skimped on hurricane strapping did not gain a properly engineered
    foundation tie-down in the same breath.

    Returns `None` for a dry house (`depth_m <= 0`) — there is no surge
    state for water that never arrived; `hurricane_flow` owns the wind
    ladder for that house instead.
    """
    rng = surge._as_random(rng)
    d = max(0.0, float(depth_m))
    if d <= 1e-6:
        return None
    eff = d / _surge_resistance(vuln)
    level = tornado._ladder(_CUTS, eff, rng, jitter_m)
    if level == "collapsed" and d >= float(swept_depth_m):
        v = max(0.0, min(1.0, float(vuln)))
        p_swept = SWEPT_P_LO + (SWEPT_P_HI - SWEPT_P_LO) * v
        if rng.random() < p_swept:
            return "swept"
    return level


# ---------------------------------------------------------------------------
# shift_spec / collapse_spec — pure geometry, no `pxr`, no `cfg`. Mirrors
# `tornado.car_pose`'s contract: a dict of numbers `apply_washaway` (or
# `tornado.toss_prim` directly) consumes, with every random draw already
# resolved so the stage-touching side draws no randomness of its own.
# ---------------------------------------------------------------------------

SHIFT_DIST_MIN_M = 2.5     # clears a house's own footprint -- see docstring
SHIFT_DIST_MAX_M = 18.0    # TUNED, NOT SOURCED: "found in the next yard, not
                            # the next county"
SHIFT_DIST_REF_DEPTH_M = 1.5   # matches `EFF_SHIFT_M`'s rough real-depth
                                # onset; only used to shape "how far PAST
                                # onset is this water", not to gate whether a
                                # shift happens at all (`house_surge_state`
                                # already decided that)
SHIFT_DIST_SATURATE_M = 2.5    # past this much extra depth, distance maxes
                                # out. TUNED, NOT SOURCED.
SHIFT_YAW_DEG = (12.0, 55.0)       # the turn that reads from the air
SHIFT_PITCH_MAX_DEG = 6.0          # small askew settle, not a topple
SHIFT_ROLL_MAX_DEG = 6.0
SHIFT_BEARING_JITTER_DEG = 12.0    # keeps a street's shifted houses roughly
                                    # agreeing on a current, not identical

# STRANDED AGAINST CONTACT, NOT THROUGH IT -- ADDED (DEBRIS D6 review,
# 2026-09-01: "there's also 1 house that looks like it's flying/brown away
# onto another house. That's a bit much and unrealistic"). Before this fix
# `shift_spec` had no idea any OTHER house existed: it drew a bearing and a
# distance and handed back a delta, and if that delta happened to land one
# floated house's footprint on top of a standing neighbour's, nothing here
# or in `apply_washaway` ever noticed. A floated house DOES travel — that is
# the whole point of this ladder rung — but it travels until it hits
# something, exactly the way `tornado.car_pose`'s thrown car does and
# `car_shift_spec` (this module's own vehicle counterpart) already models;
# see this function's own `blocked` argument below for the fix, which reuses
# `car_blockers`'s MARCHING MECHANISM the same way `car_shift_spec` already
# does, not a new one.
SHIFT_MARCH_STEP_M = 0.25       # FINER than `car_shift_spec`'s 0.5 m, ON
                                # PURPOSE: the march can overshoot into a
                                # blocker's disc by up to one whole step
                                # before it is detected, and the guaranteed
                                # final clearance is `SHIFT_JAM_GAP_M -
                                # SHIFT_MARCH_STEP_M` in the worst case (the
                                # collision is caught in the very first
                                # instant of a step) -- 0.75 - 0.25 = 0.50 m,
                                # the BOTTOM of the user's own "0.5-1 m gap"
                                # band. A 0.75 m step (matching the gap
                                # itself) would guarantee only a ~0 m
                                # clearance in that same worst case, i.e. no
                                # guarantee at all.
SHIFT_JAM_GAP_M = 0.75          # "stop at CONTACT with a 0.5-1 m gap" -- the
                                # user's own stated compromise between
                                # "flying" and "grounded/stranded"; the
                                # middle of that band. See `SHIFT_MARCH_
                                # STEP_M`'s own comment for why the step is
                                # sized relative to this, not the other way
                                # around.


def shift_spec(depth_m, flow_bearing_deg, rng, *, dist_min_m=SHIFT_DIST_MIN_M,
               dist_max_m=SHIFT_DIST_MAX_M, x=0.0, y=0.0, own_fp_m=0.0,
               blocked=None):
    """Where a floated house comes to rest, and how it sits when it gets
    there — `{dx, dy, dyaw, pitch, roll, d_m, arrested_by}`, world metres
    and degrees.

    THE HOUSE MOVES DOWN-FLOW. `flow_bearing_deg` is the surge's own inland
    direction (`surge_cfg["shore_bearing_deg"]` — see this module's "WIRING
    THIS TO surge.py"), so every house a caller displaces on one plate
    drifts the same way: a street of shifted houses all turned to agree with
    one current, not a scatter of random headings that would read as noise
    rather than water. A small per-house bearing jitter keeps them from
    looking suspiciously parallel.

    DISTANCE GROWS WITH HOW FAR PAST THE ONSET DEPTH THE WATER IS, not with
    depth itself: a house one centimetre past the point where it floats free
    should not travel as far as one sitting in the plate's deepest water.
    TUNED, NOT SOURCED — unlike `tornado.car_pose`, which at least has
    Paulikas' vehicle-displacement survey to shape against, no field study
    gives a depth-to-drift-distance curve for a residential slab-on-grade
    house. `dist_min_m` clears the house's own footprint on purpose — the
    user's own line was "a house nudged 0.5 m looks like a bug".

    ROTATION AND A SMALL ASKEW SETTLE, ALWAYS. A floated house that lands
    dead level and square looks like it was picked up and put down by a
    crane, not one that drifted onto whatever was already on the ground —
    the user's own "it came to rest on debris, not on a prepared pad".
    `dyaw` is the large, obvious turn; `pitch`/`roll` stay a few degrees
    because the house is still basically intact at this level
    (`collapse_spec` is the heap, not this one).

    `x`, `y`, `own_fp_m` and `blocked`, ALL OPTIONAL: omit them (the
    default) and this is exactly the old bare depth-driven drift with no
    collision awareness at all — every EXISTING caller is byte-identical.
    Supply the house's own current world position (`x`, `y` — the caller's
    own `h["x"], h["y"]`), its own footprint radius proxy (`own_fp_m` —
    e.g. `fp_by_style.get(h["style"], 12.0)`, the same lookup the
    "# DEBRIS RAFTS"/"# LAND DEBRIS" launcher blocks already use) and a
    `tornado.car_blockers(standing=...)`-shaped predicate built over every
    OTHER house on the plate (the caller excludes this house's own entry —
    the same "a car does not block itself" responsibility `car_shift_spec`'s
    own docstring already places on its caller), and the drift is walked in
    `SHIFT_MARCH_STEP_M` steps exactly the way a floated car's is: arrested
    at the first hit, with `dist` pulled back to `SHIFT_JAM_GAP_M` short of
    it. `arrested_by` in the returned dict is `None` when nothing was hit
    (the old, unconditional behaviour) or the blocker's tag when the march
    found one — the same field `car_shift_spec` already returns, for the
    same reason: a caller that wants to know can.
    """
    rng = surge._as_random(rng)
    d = max(0.0, float(depth_m))
    past = max(0.0, d - SHIFT_DIST_REF_DEPTH_M)
    frac = min(1.0, past / SHIFT_DIST_SATURATE_M)
    dist = dist_min_m + (dist_max_m - dist_min_m) * (frac ** 0.8)
    dist *= rng.uniform(0.75, 1.25)
    bearing = float(flow_bearing_deg) + rng.gauss(0.0, SHIFT_BEARING_JITTER_DEG)
    rad = math.radians(bearing)

    arrested = None
    if blocked is not None and dist > 0.0:
        half = max(1.0, float(own_fp_m)) * 0.5
        s = 0.0
        while s < dist:
            s = min(dist, s + SHIFT_MARCH_STEP_M)
            tag = blocked(float(x) + math.cos(rad) * (s + half),
                         float(y) + math.sin(rad) * (s + half))
            if tag is not None:
                arrested = tag
                dist = max(0.0, s - SHIFT_JAM_GAP_M)
                break

    return {
        "dx": math.cos(rad) * dist,
        "dy": math.sin(rad) * dist,
        "dyaw": rng.choice((-1.0, 1.0)) * rng.uniform(*SHIFT_YAW_DEG),
        "pitch": rng.uniform(-SHIFT_PITCH_MAX_DEG, SHIFT_PITCH_MAX_DEG),
        "roll": rng.uniform(-SHIFT_ROLL_MAX_DEG, SHIFT_ROLL_MAX_DEG),
        "d_m": round(dist, 3),
        "arrested_by": arrested,
    }


COLLAPSE_DIST_MIN_M = 0.5      # a heap does not travel -- see docstring
COLLAPSE_DIST_MAX_M = 3.0
COLLAPSE_DIST_SATURATE_M = 2.0
COLLAPSE_YAW_DEG = (0.0, 20.0)
# RETUNED, DEBRIS D6 review 2026-09-01 (user: "flying/[thrown] away onto
# another house. That's a bit much and unrealistic" -- plus this review's
# own "cap any pitch/roll so no wall leaves the water plane by more than
# ~1.5 m" directive). The ORIGINAL (18.0, 42.0) was picked purely for "heavy
# list, most of the read" with no height check at all against it.
# `planks._lay`'s own true-vertical-half-extent idea (the same closed form
# `_one_raft` reuses for a raft's draft: half_extent grows with
# `sin(pitch)` times the body's own half-length), applied to
# `COLLAPSE_NOMINAL_HALF_FP_M` (see that constant's own docstring for why a
# NOMINAL figure is what is available here, not a real per-house
# measurement), gives a corner rise of roughly `6.0 * sin(pitch)` for pitch
# ALONE (roll tilts a different pair of corners, so the two are not simply
# additive at any one corner): 42 degrees rises ~4.0 m -- comfortably past
# the 1.5 m target and the actual "flying" read reported — while 14 degrees
# rises ~1.45 m, just inside it. Still a visibly heavier list than
# `SHIFT_PITCH_MAX_DEG`'s mild 6-degree settle, so a collapsed house still
# reads as a heap next to a merely-shifted one, just not a launching one.
COLLAPSE_PITCH_DEG = (10.0, 14.0)
COLLAPSE_ROLL_DEG = (10.0, 14.0)
# Nominal house half-footprint, metres -- used ONLY to derive
# `COLLAPSE_PITCH_DEG`/`COLLAPSE_ROLL_DEG`'s own height check above. This
# spec is drawn (in every real caller) BEFORE the house prim is even
# referenced, let alone measured (`_measure_house` runs inside
# `apply_washaway`, which receives this function's OUTPUT), so a real
# per-house footprint is not available at the point this constant is
# needed; `fp_by_style`'s own 10-20 m range midpoint, halved, is the
# closest a caller-agnostic constant gets.
COLLAPSE_NOMINAL_HALF_FP_M = 6.0

# STRANDED AGAINST CONTACT -- see `shift_spec`'s own "STRANDED AGAINST
# CONTACT, NOT THROUGH IT" section for the defect this fixes; a heap does
# not travel far, but even a 0.5-3 m settle can, on a tight lot, still land
# on a neighbour's own footprint, and the fix is identical. Step sized
# FINER than the gap for the same reason `SHIFT_MARCH_STEP_M` is -- see that
# constant's own comment; 0.75 - 0.25 = 0.50 m guaranteed worst-case
# clearance.
COLLAPSE_MARCH_STEP_M = 0.25
COLLAPSE_JAM_GAP_M = 0.75


def collapse_spec(depth_m, rng, *, x=0.0, y=0.0, own_fp_m=0.0, blocked=None):
    """Pose for a house whose frame came apart in the water — `{dx, dy,
    dyaw, pitch, roll, arrested_by}`, the same shape `shift_spec` returns so
    `apply_washaway` drives both through `tornado.toss_prim` identically.

    UNLIKE `shift_spec`, THE HOUSE DOES NOT TRAVEL FAR: it broke up roughly
    where it stood rather than floating off intact, so displacement is a
    small settle (well under a house-length) in a RANDOM direction (there is
    no down-flow drift to a pile that did not float), and the read comes
    almost entirely from a heavy, near-random list rather than a directed
    turn. TUNED, NOT SOURCED throughout.

    THIS IS A RIGID TRANSFORM, NOT A FRACTURE — see the module docstring's
    "WHAT THIS MODULE DELIBERATELY DOES NOT DO". A real collapse belongs to
    `fracture.py`/`settle.py`; this is the acknowledged stand-in.

    `x`, `y`, `own_fp_m` and `blocked` — the SAME optional, backward-
    compatible marching contract `shift_spec` now takes (see that
    function's own docstring for the full explanation): omitted, this is
    the old unconditional random settle; supplied, the settle is arrested
    at the first neighbour it would otherwise land inside of.
    """
    rng = surge._as_random(rng)
    d = max(0.0, float(depth_m))
    frac = min(1.0, d / COLLAPSE_DIST_SATURATE_M)
    dist = COLLAPSE_DIST_MIN_M + (COLLAPSE_DIST_MAX_M -
                                  COLLAPSE_DIST_MIN_M) * frac
    ang = math.radians(rng.uniform(0.0, 360.0))

    arrested = None
    if blocked is not None and dist > 0.0:
        half = max(1.0, float(own_fp_m)) * 0.5
        s = 0.0
        while s < dist:
            s = min(dist, s + COLLAPSE_MARCH_STEP_M)
            tag = blocked(float(x) + math.cos(ang) * (s + half),
                         float(y) + math.sin(ang) * (s + half))
            if tag is not None:
                arrested = tag
                dist = max(0.0, s - COLLAPSE_JAM_GAP_M)
                break

    return {
        "dx": math.cos(ang) * dist,
        "dy": math.sin(ang) * dist,
        "dyaw": rng.choice((-1.0, 1.0)) * rng.uniform(*COLLAPSE_YAW_DEG),
        "pitch": rng.choice((-1.0, 1.0)) * rng.uniform(*COLLAPSE_PITCH_DEG),
        "roll": rng.choice((-1.0, 1.0)) * rng.uniform(*COLLAPSE_ROLL_DEG),
        "arrested_by": arrested,
    }


# ---------------------------------------------------------------------------
# VEHICLES — the car counterpart of the house ladder above. Ranked #1 in
# `_plans/hurricane_coverage_audit.md`: `hurricane.py` states twice that
# cars are moved by water and names this module as the owner; nothing here
# ever touched one until this section. See the module docstring's "THE
# VEHICLE LADDER — CARS ARE NOT SMALL HOUSES" for why every constant below
# is new rather than a scaled-down `RESIST_LO`/`RESIST_HI`/`_CUTS`/`EFF_*` —
# those stay exactly as they are and no car function reads them.
# ---------------------------------------------------------------------------

# The residential pool's own reference length — the same number
# `tornado.CAR_REF_LEN_M` uses, for the same population (Paulikas' survey is
# overwhelmingly light passenger vehicles and this suburb's pool runs
# 4.60-4.84 m for an ordinary car). A SEPARATE constant, not an import of
# that one: this ladder's resistance curve is buoyancy, not wind lofting,
# and the two need to be free to diverge even though they start from the
# same reference vehicle.
CAR_REF_LEN_M = 4.6

# A longer/heavier vehicle resists buoyant uplift for the same structural
# reason `tornado.car_pose`'s `mass = CAR_REF_LEN_M / length_m` resists wind
# lofting — frontal/plan area and mass both scale with length for
# geometrically similar vehicles. The ratio runs the OPPOSITE way from that
# formula on purpose: `car_pose` expresses "harder to move" as a smaller
# force multiplier, this expresses it as MORE DEPTH NEEDED, and depth is
# divided by this factor (`_car_resistance`), so the factor has to grow
# with length rather than shrink. TUNED, NOT SOURCED for the exact slope;
# the depth calibration below is quoted for ordinary passenger cars and the
# suburban pool runs a 9.5 m transit bus, so "a longer vehicle needs more
# real depth to float" is the qualitatively correct direction even without
# a fitted curve.
CAR_RESIST_LO = 0.60   # a compact well under the reference length
CAR_RESIST_HI = 2.00   # a van, pickup, or the pool's own 9.5 m bus

# CARS ARE NOT HOUSES. EFFECTIVE-DEPTH ONSETS (metres, AFTER the length-
# resistance divide), ANCHORED rather than TUNED — unlike the house
# ladder's own calibration-target exercise, these come straight off
# NOAA/NWS's public "Turn Around Don't Drown" depth table, which is
# literally a depth-to-vehicle-response chart: ~6 in (0.15 m) of water
# reaches most passenger-car floorpans and can stall or float-and-steer a
# car out of control; ~12 in (0.30 m) of moving water floats and carries
# away MANY cars; ~24 in (0.60 m) carries away MOST vehicles, SUVs and
# pickups included. A car is far lighter relative to its displaced volume
# than a house, and these onsets sit an order of magnitude below the house
# ladder's `EFF_SCOUR_M`/`EFF_SHIFT_M` (0.75 m / 2.0 m) for exactly that
# reason — a house needs metres, a car needs inches.
CAR_EFF_SWAMP_M = 0.15
CAR_EFF_FLOAT_M = 0.30

# Per-car jitter, same purpose as `LADDER_JITTER_M` — stops the ladder
# reading as contour lines — scaled down from that constant in rough
# proportion to how much smaller this ladder's own thresholds are (0.18 m
# against a 0.75 m onset there carries over as roughly 0.05 m against a
# 0.30 m onset here). TUNED, NOT SOURCED.
CAR_LADDER_JITTER_M = 0.05

_CAR_CUTS = ((CAR_EFF_SWAMP_M, "dry"),
            (CAR_EFF_FLOAT_M, "swamped"),
            (float("inf"), "floated"))

# `piled` is carved OUT of `floated`, mirroring how `swept` is carved out of
# `collapsed` above — but gated on how far PAST the float onset the water
# is, not on a construction-era `vuln` a car does not have. A car that only
# just floats free bobs in place or drifts a short way into open water or a
# lawn; one sitting in water well past the NWS "carries away most vehicles"
# line (0.60 m — `CAR_EFF_FLOAT_M + CAR_PILE_SATURATE_M` reaches exactly
# that) has travelled far enough, in strong enough current, that finding a
# wall, a fence, a tree or another car in its path is the LIKELY outcome,
# not the exception. This is a PREDICTION made from depth alone, before any
# blocker geometry exists — see `car_shift_spec` for where that prediction
# is actually tested against one. TUNED, NOT SOURCED for the probabilities;
# anchored at the top end to the same 0.60 m NWS figure the ladder above
# already cites.
CAR_PILE_SATURATE_M = 0.30
CAR_PILE_P_LO = 0.15
CAR_PILE_P_HI = 0.80


def _car_resistance(length_m, ref_len_m=CAR_REF_LEN_M):
    L = max(1.0, float(length_m))
    ref = max(1.0, float(ref_len_m))
    return max(CAR_RESIST_LO, min(CAR_RESIST_HI, L / ref))


def car_surge_state(depth_m, rng, *, length_m=CAR_REF_LEN_M,
                    jitter_m=CAR_LADDER_JITTER_M):
    """Per-car surge damage state: `'dry'`, `'swamped'`, `'floated'` or
    `'piled'`. Always a real string — unlike `house_surge_state`, there is
    no `None` sentinel here, because "no water reached this car" is a state
    worth naming rather than a case every caller must remember to test for
    separately.

    `depth_m` is the caller's own `surge.depth_at(...)(x, y)` — the same
    single scalar `house_surge_state` takes. `length_m` is the ONE piece of
    per-vehicle variance this function draws on, and it is deliberately NOT
    `vuln`: a car has no construction era or anchorage code
    (`hurricane.draw_vulnerability` is a per-HOUSE draw), so reusing
    `_surge_resistance` would be inventing a code era for an object that
    has none. What a car DOES have is mass and ground clearance, and length
    is the only proxy for those this module can measure the same way
    `_measure_house` measures a footprint — see `apply_car_washaway`'s own
    `_measure_car`, or pass a length already known from the asset pool
    (`suburb_scene.car_dims`, the way the tornado launch script already
    does for `tornado.car_pose`). Omit it and every car is treated as the
    reference saloon.
    """
    rng = surge._as_random(rng)
    d = max(0.0, float(depth_m))
    if d <= 1e-6:
        return "dry"
    eff = d / _car_resistance(length_m)
    level = tornado._ladder(_CAR_CUTS, eff, rng, jitter_m)
    if level == "floated":
        past = max(0.0, eff - CAR_EFF_FLOAT_M)
        frac = min(1.0, past / CAR_PILE_SATURATE_M)
        p_pile = CAR_PILE_P_LO + (CAR_PILE_P_HI - CAR_PILE_P_LO) * frac
        if rng.random() < p_pile:
            return "piled"
    return level


# ---------------------------------------------------------------------------
# car_shift_spec — pure geometry, no `pxr`, mirroring `shift_spec`'s
# contract (depth + bearing + rng in, a `toss_prim`-shaped pose dict out).
# ---------------------------------------------------------------------------

CAR_SHIFT_DIST_MIN_M = 1.5     # clears the car's own footprint -- the same
                                # "never a small nudge" argument `shift_spec`
                                # makes for a house, at a car's scale
CAR_SHIFT_DIST_MAX_M = 22.0    # TUNED, NOT SOURCED: a car in fast water
                                # travels much further than a house relative
                                # to its own length -- Harvey accounts of
                                # cars swept into bayous are tens of metres,
                                # not "the next yard"
CAR_SHIFT_DIST_REF_DEPTH_M = CAR_EFF_FLOAT_M   # distance grows past THIS
                                                # depth, not from zero --
                                                # same "how far past onset"
                                                # shape `shift_spec` uses
CAR_SHIFT_DIST_SATURATE_M = 0.9
CAR_SHIFT_YAW_DEG = (15.0, 70.0)    # a car spins more freely than a house
                                    # once afloat -- far less mass, far less
                                    # anchorage to resist the turn
CAR_BEARING_JITTER_DEG = 16.0

# A FLOATED CAR IS OFTEN LEFT AT AN ANGLE, AND IS NOT ALWAYS FLUSH AGAINST
# WHATEVER STOPPED IT -- the user's own line. Every floated car gets a
# baseline settle tilt (it bobbed to rest on uneven ground or debris, not a
# level pad); one that is ARRESTED by a blocker gets a bigger, asymmetric
# lift instead of the baseline -- either a jam pitch (stopped flush against
# the obstruction, nose lifted) or, `CAR_MOUNT_P` of the time, a steeper
# mount pitch AND some roll (climbed part way up onto it -- a low fence
# rail, another car's hood or roof). TUNED, NOT SOURCED throughout.
CAR_SETTLE_PITCH_DEG = (3.0, 14.0)
CAR_SETTLE_ROLL_DEG = (3.0, 14.0)
CAR_JAM_PITCH_DEG = (10.0, 26.0)
CAR_MOUNT_P = 0.30
CAR_MOUNT_PITCH_DEG = (22.0, 36.0)
CAR_MOUNT_ROLL_DEG = (8.0, 22.0)

# How the drift is MARCHED against a `blocked` predicate, same idiom
# `tornado.car_pose` uses for its own march (`CAR_MARCH_M`/`CAR_JAM_GAP_M`
# there) -- copied as separate constants rather than imported, the same
# "two lines is not worth a cross-module dependency" call `surge._draw`'s
# own docstring makes about `scour_relief._draw`.
CAR_MARCH_STEP_M = 0.5
CAR_JAM_GAP_M = 0.4


def car_shift_spec(depth_m, flow_bearing_deg, rng, *,
                   dist_min_m=CAR_SHIFT_DIST_MIN_M,
                   dist_max_m=CAR_SHIFT_DIST_MAX_M,
                   x=0.0, y=0.0, length_m=CAR_REF_LEN_M, blocked=None):
    """Where a floated car ends up, and how it sits — `{dx, dy, dyaw, pitch,
    roll, d_m, arrested_by}`, the exact shape `apply_car_washaway` hands to
    `tornado.toss_prim`. Mirrors `shift_spec`'s contract: same depth-driven,
    down-flow drift, same saturate-past-onset distance shape, same
    per-object bearing jitter so a street of floated cars agrees with one
    current instead of scattering — calibrated for a car, not a house; see
    the VEHICLE section above for why the constants differ, and never read
    `EFF_SHIFT_M`/`SHIFT_DIST_*_M` into this function.

    `x`, `y` and `blocked` are OPTIONAL and, together, REUSE
    `tornado.car_blockers`'s marching mechanism rather than reinventing it:
    omit them and this is a bare depth-driven drift with no marching at
    all, exactly `shift_spec`'s own contract. Supply the car's current
    world position (the caller already has it, from the same placement
    list `car_blockers` was built against) and a `car_blockers(...)`
    predicate, and the drift is walked in `CAR_MARCH_STEP_M` steps exactly
    the way `tornado.car_pose` steps a wind-thrown car, so a floated car
    comes to rest against the same house walls, tree boles and other cars a
    tornado-thrown one would. This reuses `car_pose`'s MARCHING MECHANISM
    only, never its wind-lofting probabilities — see the module docstring's
    "THE VEHICLE LADDER" for why those stay out of this function entirely.
    `arrested_by` is `None` when nothing was hit (open water or a lawn) or
    the blocker's tag string when the march found one.
    """
    rng = surge._as_random(rng)
    d = max(0.0, float(depth_m))
    past = max(0.0, d - CAR_SHIFT_DIST_REF_DEPTH_M)
    frac = min(1.0, past / CAR_SHIFT_DIST_SATURATE_M)
    dist = dist_min_m + (dist_max_m - dist_min_m) * (frac ** 0.8)
    dist *= rng.uniform(0.75, 1.25)
    bearing = float(flow_bearing_deg) + rng.gauss(0.0, CAR_BEARING_JITTER_DEG)
    rad = math.radians(bearing)

    arrested = None
    if blocked is not None and dist > 0.0:
        half = max(1.0, float(length_m)) * 0.5
        s = 0.0
        while s < dist:
            s = min(dist, s + CAR_MARCH_STEP_M)
            tag = blocked(float(x) + math.cos(rad) * (s + half),
                         float(y) + math.sin(rad) * (s + half))
            if tag is not None:
                arrested = tag
                dist = max(0.0, s - CAR_JAM_GAP_M)
                break

    pitch = rng.choice((-1.0, 1.0)) * rng.uniform(*CAR_SETTLE_PITCH_DEG)
    roll = rng.choice((-1.0, 1.0)) * rng.uniform(*CAR_SETTLE_ROLL_DEG)
    if arrested is not None:
        if rng.random() < CAR_MOUNT_P:
            pitch = rng.choice((-1.0, 1.0)) * rng.uniform(*CAR_MOUNT_PITCH_DEG)
            roll = rng.choice((-1.0, 1.0)) * rng.uniform(*CAR_MOUNT_ROLL_DEG)
        else:
            pitch = rng.choice((-1.0, 1.0)) * rng.uniform(*CAR_JAM_PITCH_DEG)

    return {
        "dx": math.cos(rad) * dist,
        "dy": math.sin(rad) * dist,
        "dyaw": rng.choice((-1.0, 1.0)) * rng.uniform(*CAR_SHIFT_YAW_DEG),
        "pitch": pitch,
        "roll": roll,
        "d_m": round(dist, 3),
        "arrested_by": arrested,
    }


# ---------------------------------------------------------------------------
# raft_specs / build_rafts — the field half DOES take a `cfg` (like every
# other scatter function in `surge.py`/`planks.py`), because it has enough
# knobs (density, standoff, kind mix, dimensions) to earn one.
# ---------------------------------------------------------------------------

DEFAULTS = {
    "seed": 0,

    # Mirrors `surge.DEFAULTS["surge_m"]`'s own default (2.0 m, floored the
    # same way `surge.water_level` floors it) so a bare `resolve_cfg({})`
    # bench produces a sane float height with no wiring at all. A real scene
    # MUST override this from `surge.water_level(surge_cfg)` — see the
    # module docstring's "WIRING THIS TO surge.py".
    "water_level_m": 2.0,

    # -- background raft scatter across open water -- TUNED, NOT SOURCED;
    # scaled off `surge.DEFAULTS`'s own per-100m2 idiom for L2 ponding --
    "raft_cell_m": 25.0,
    # RAISED HARD, 2026-08-31, after a review of the 500 m L2 plate: "have
    # debris floating in it, wood planks etc from houses, rather than just
    # empty water near houses".
    #
    # The old rates put 47 boxes on a 500 m plate with 136 houses and 37% of
    # it under water — 3 timber, 3 sheet, 4 wall, 37 vegetation. At 25 m
    # cells, 0.03 per 100 m2 is 0.19 mats per CELL, i.e. four fifths of the
    # flooded cells held nothing at all, and the water read as a clean sheet.
    # A real surge-flooded street is a debris field: everything that floats
    # comes off the houses and collects.
    #
    # 0.28 puts ~1.8 pieces in a wet 25 m cell (~260 across the plate's wet
    # third), which scatters open water without carpeting it. The clusters
    # below carry the rest and are where the read actually comes from.
    #
    # RAISED AGAIN, 2026-08-31 (user: "the debris in the flooded area needs
    # to increase a lot in number" — measured against the shipped
    # `FINAL_L3_brown` render, 3,234 water rafts on a 500 m, 55%-flooded L3
    # plate; `deep_water_obl.png` still reads as open water with scattered
    # confetti, not a debris field). 5x: this is the single biggest lever on
    # OPEN-WATER count (it also scales the waterline band by the same
    # factor, `raft_waterline_boost` unchanged) — see `raft_per_house`/
    # `_RAFT_TANGLE` for the matching cluster-side raise and
    # `_drift_line_specs` for the new mid-water structure this budget also
    # has to fill. TUNED, NOT SOURCED for the exact multiple; searched
    # jointly with the other two knobs below against the "~3x, 12-15k
    # pieces at L3" target (`tools/hurricane_debris_plot.py`'s own offline
    # count).
    "raft_bg_per_100m2": 1.4,
    # 0.5 m kept debris off the whole shallow margin — but the waterline is
    # exactly where flood debris strands and is most visible, and it is what
    # the review was looking at. A 0.25 m board floats in 0.25 m of water.
    "raft_min_depth_m": 0.25,

    # -- THE STRAND LINE, ADDED 2026-08-31 -- flood debris collects at the
    # WATERLINE (the review's own language: "locally sourced, short-
    # travelled ... against the first obstruction"), not spread evenly
    # across open water. `raft_waterline_band_m` is how far past
    # `raft_min_depth_m` still counts as "the waterline" (a 15 cm-deep
    # margin); any background cell whose centre depth falls inside
    # `[raft_min_depth_m, raft_min_depth_m + raft_waterline_band_m]` draws
    # at `raft_bg_per_100m2 * raft_waterline_boost` instead of the plain
    # background rate. TUNED, NOT SOURCED; `raft_waterline_boost` sits
    # inside the requested 3-5x band.
    "raft_waterline_band_m": 0.15,
    "raft_waterline_boost": 1.6,   # was 4.0 — with the strand line this DREW the shoreline; user 2026-08-31: debris must not collect exactly along the border

    # -- clustered against standing obstacles -- TUNED, NOT SOURCED --
    # 1.4 per house, times a depth ramp, is roughly one board per flooded
    # house. Raised with the background above; see `_RAFT_TANGLE` for the
    # second half of the change (each draw is a small tangle, not one plank).
    #
    # RAISED AGAIN, 2026-08-31, alongside `raft_bg_per_100m2` -- 2.4x the
    # DRAW RATE, paired with `_RAFT_TANGLE`'s own 2x average-tangle-size
    # raise below, so the pile against a wrecked house grows by roughly
    # 2.4 x 2 = 4.8x overall ("more tangles, larger tangle sizes near
    # obstructions"), the biggest single multiplier in this pass because
    # "against a house" is where a real flood's debris actually collects.
    "raft_per_house": 12.0,
    "raft_saturate_depth_m": 2.0,
    "raft_standoff_m": (0.3, 2.0),
    "raft_bearing_jitter_deg": 20.0,

    # -- TREES/CARS, ADDED 2026-08-31 -- "clusters on the up-flow side of
    # standing houses/trees/cars", `raft_specs`'s new `obstacles=` argument.
    # A tree or a car is a far smaller snag than a house, so it collects
    # noticeably less against it -- a fraction of `raft_per_house`, not the
    # same rate. TUNED, NOT SOURCED.
    #
    # Kept at the SAME ratio to `raft_per_house` (0.3) through the
    # 2026-08-31 raise, so a tree/car catch still scales with the house
    # catch instead of falling further behind it.
    "raft_per_obstacle": 3.6,
}


def resolve_cfg(config):
    """`DEFAULTS` with `config` layered over it. Idempotent, same contract
    as `surge.resolve_cfg`/`ground.resolve_cfg`."""
    kn = dict(DEFAULTS)
    kn.update(config or {})
    return kn


def knobs_from_env(span_m):
    """`DEFAULTS` overridden by `WASHAWAY_*` environment variables, the
    `SURGE_*`/`GROUND_*` convention. `span_m` scales the one knob whose
    sensible default depends on plate size (a raft field on a 100 m bench
    should not use the same background cell count as a 1 km cell)."""
    span_m = float(span_m)

    def _f(name, default):
        raw = os.environ.get("WASHAWAY_" + name)
        return float(default) if raw is None or raw.strip() == "" else float(raw)

    return dict(
        seed=int(_f("SEED", DEFAULTS["seed"])),
        water_level_m=_f("WATER_LEVEL_M", DEFAULTS["water_level_m"]),
        raft_cell_m=_f("RAFT_CELL_M", max(8.0, min(DEFAULTS["raft_cell_m"],
                                                    0.05 * span_m))),
        raft_bg_per_100m2=_f("RAFT_BG_PER_100M2",
                             DEFAULTS["raft_bg_per_100m2"]),
        raft_min_depth_m=_f("RAFT_MIN_DEPTH_M", DEFAULTS["raft_min_depth_m"]),
        raft_waterline_band_m=_f("RAFT_WATERLINE_BAND_M",
                                 DEFAULTS["raft_waterline_band_m"]),
        raft_waterline_boost=_f("RAFT_WATERLINE_BOOST",
                                DEFAULTS["raft_waterline_boost"]),
        raft_per_house=_f("RAFT_PER_HOUSE", DEFAULTS["raft_per_house"]),
        raft_per_obstacle=_f("RAFT_PER_OBSTACLE",
                             DEFAULTS["raft_per_obstacle"]),
        raft_saturate_depth_m=_f("RAFT_SATURATE_DEPTH_M",
                                 DEFAULTS["raft_saturate_depth_m"]),
    )


# Debris in `planks._box`'s spec schema (l/w/t + x/y/z + yaw/pitch/roll).
# Five classes — timber, sheet goods, whole wall sections, vegetation mats
# and whole log/bole sections — sized as broken pieces rather than a size
# range drawn from nothing, the same argument `planks.STOCK` makes. All but
# `log` are flat "mats"; see that entry for why it is not.
# (len_range_m, width_range_m, thickness_range_m)
_RAFT_DIMS = {
    # A WHOLE TREE TRUNK IN THE WATER. The review asked for "trees, logs
    # fallen off" floating alongside the plank debris, and nothing in this
    # table was shaped like one: the longest class was a 3.5 m board 0.3 m
    # wide, so a "vegetation" raft could only ever read as brush. A bole
    # section is the single most recognisable thing in flood footage --
    # metres long, round, and thick enough to sit proud of the surface
    # instead of lying flush like a mat.
    #
    # WIDENED AGAIN, 2026-08-31 (DEBRIS D5 review: "I don't really see
    # floating tree trunks/fallen down trees in the flood"). 2.5-8 m at
    # 0.25-0.55 m through was still a garden tree; a mature street/yard tree
    # taken whole by a hurricane surge runs longer and thicker than that, and
    # a bigger bole is also what makes the class read as a TRUNK rather than
    # a stout limb next to the rest of the raft field's boards. 3.5-11 m of
    # trunk at 0.30-0.70 m through. `_one_raft` forces the thickness to equal
    # the width for this class (a bole is round in section, not a plank),
    # and gives it the full tilt range -- a floating log rolls. TUNED, NOT
    # SOURCED for the exact numbers; "mature street tree, whole" is the
    # target the range was picked against.
    "log":    ((3.5, 11.0), (0.30, 0.70), (0.30, 0.70)),
    "timber": ((0.8, 3.5), (0.10, 0.30), (0.03, 0.08)),
    "sheet":  ((1.0, 2.4), (0.60, 1.20), (0.02, 0.05)),
    "wall":   ((1.5, 3.5), (1.00, 2.40), (0.08, 0.16)),
    # COARSE HURRICANE STOCK, ADDED 2026-08-31 (DEBRIS stream review: "have
    # debris floating in it ... not just [uniform] planks"). A hurricane
    # PEELS rather than shreds (`build-hurricane-scenes` skill's "Debris is
    # 70% vegetation" section): the 28% that is building material is coarse
    # — whole sheathing panels, torn fence sections, long siding strips and
    # roof sections — not the tornado's stud-and-joist confetti. `timber`/
    # `sheet`/`wall` above stay as the generic fallback shapes; these four
    # are the named, real-world sections a raft field should actually show.
    #
    # A 4x8 (1.22 x 2.44 m) OSB/plywood sheathing panel, the single most
    # recognisable flat object in flood-debris photography. Jittered a few
    # cm either way -- a torn panel is not a fresh sheet off the rack.
    "panel":   ((1.15, 1.32), (2.30, 2.55), (0.010, 0.015)),
    # A whole fence PANEL (post-to-post bay), pickets and rails still
    # attached -- thickness stands in for the rail/picket depth, not a
    # single board's thickness.
    "fence":   ((1.75, 1.85), (2.10, 2.45), (0.045, 0.090)),
    # A long strip of exterior cladding torn off in one run rather than
    # broken into a `timber`-sized piece -- distinct from `planks.SKINNED`
    # `siding` (which carries a specific wrecked house's own material): this
    # is the generic, un-skinned raft-field fallback.
    "siding_strip": ((3.0, 3.6), (0.15, 0.25), (0.008, 0.014)),
    # A torn roof section: sheathing plus its covering, big and irregular
    # enough that "2-4 m, roughly rectangular" is the honest shape a box can
    # give it. Split into two SEPARATE kinds below (`roof_top`/`roof_under`)
    # rather than one "roof" kind with a per-piece variant, because a piece
    # floats shingle-up about as often as it floats sheathing-up and the two
    # faces are a different colour -- see `_RAFT_TINT`. Using two literal
    # kind strings means `build_rafts`'s existing per-`kind` grouping needs
    # no change at all to produce the two-tone mesh.
    "roof_top":   ((2.0, 4.0), (1.5, 3.0), (0.03, 0.06)),
    "roof_under": ((2.0, 4.0), (1.5, 3.0), (0.03, 0.06)),
    # VEGETATION IS SHAPED DIFFERENTLY FROM SAWN MATERIAL. A mat of
    # torn-off canopy, palm fronds and understory brush tangled together by
    # the current is broad and roughly as wide as it is long — nothing like
    # a plank's 3-9:1 length:width ratio — and it has real vertical bulk
    # from the branches and root balls caught in it, not a flat sheet's few
    # centimetres. `l`/`w` ranges overlap almost entirely on purpose, so
    # `planks._box` draws a squarish, irregular footprint most of the time
    # rather than a plank shape wearing a vegetation tint. TUNED, NOT
    # SOURCED for the exact numbers; the SHAPE choice (broad and thick, not
    # long and thin) is the load-bearing part.
    "vegetation": ((1.2, 4.5), (0.9, 3.2), (0.12, 0.35)),
}
# TUNED, NOT SOURCED for the building-side 5:3:2 split (unchanged from the
# original three-kind table). The 72/28 VEGETATION/BUILDING ratio itself is
# NOT tuned — it is Lee County's own post-Ian debris-collection tonnage
# (734,136 yd³ vegetative vs 285,282 yd³ C&D), the same citation
# `hurricane.VEG_SHARE`/`BLD_SHARE` already use for the ground debris mix
# (`hurricane.debris_mix`'s own docstring: "the 72/28 vegetation/building
# split is held FIXED across intensity"). This dict is the STATIC FALLBACK a
# bare `raft_specs(cfg, region, rng, houses, depth_fn)` call gets — the only
# call shape every caller before this fix used, and still what a caller with
# no per-scene `debris_mix` result in hand should pass. A caller that DOES
# have one should not read this constant at all: see `raft_kind_weights`,
# which threads that live number through instead of re-deriving it here a
# second time (the module docstring's "THE RAFT FIELD WAS ALL BUILDING
# MATERIAL").
# `log` takes `_LOG_SHARE_OF_VEG` of whatever the organic share is -- a
# fallen bole IS vegetation debris, so it comes out of that budget rather
# than diluting the building classes the review specifically asked to see
# more of.
_LOG_SHARE_OF_VEG = 0.22
# THE BUILDING-SIDE COMPOSITION, factored out to its own dict, 2026-08-31,
# so `_RAFT_KIND_WEIGHTS` (the static fallback) and `raft_kind_weights`
# (the live-`debris_mix`-driven version) build the SAME building mix instead
# of the two drifting apart the way `timber`/`sheet`/`wall`'s old inline
# 0.5/0.3/0.2 would have the moment a caller edited one and not the other.
# Sums to 1.0; re-skewed to make room for the coarse hurricane classes added
# alongside the original three generic shapes (see `_RAFT_DIMS`'s own
# docstring) -- `panel`/`fence`/`siding_strip` are what a photograph of
# hurricane debris actually shows, so they now carry more of the budget than
# the generic `sheet`/`wall` fallback shapes do. `roof_top`/`roof_under`
# split the "torn roof section" allocation close to evenly (a piece lands
# shingle-up about as often as sheathing-up), which is also why each is
# HALF of the `_ROOF_SHARE` below rather than a nested weight.
_ROOF_SHARE = 0.14
_BUILD_MIX = {
    "timber": 0.20, "sheet": 0.12, "wall": 0.08,
    "panel": 0.16, "fence": 0.14, "siding_strip": 0.16,
    "roof_top": _ROOF_SHARE * 0.55, "roof_under": _ROOF_SHARE * 0.45,
}
assert abs(sum(_BUILD_MIX.values()) - 1.0) < 1e-9, _BUILD_MIX
_RAFT_KIND_WEIGHTS = {"vegetation": 0.72 * (1.0 - _LOG_SHARE_OF_VEG),
                      "log": 0.72 * _LOG_SHARE_OF_VEG}
_RAFT_KIND_WEIGHTS.update(
    {k: 0.28 * v for k, v in _BUILD_MIX.items()})
# The classes that are organic rather than sawn. `_house_mix` caps these
# TOGETHER: what piles against a wrecked house is building material, and a
# log is no more "from the house" than a canopy mat is.
_ORGANIC_KINDS = ("vegetation", "log")
# WET DARKENING, ADDED 2026-08-31. Every raft now has a real draft (see
# `_one_raft`'s "DRAFT AND ATTITUDE" section) -- it is genuinely IN the
# water, not resting on top of it -- so the visible portion of a building-
# material piece is soaked, not dry lumber. `_RAFT_TINT`'s building-class
# entries above are DRY reference colours; `build_rafts` multiplies each of
# them by `_WET_DARKEN` before it goes on `diffuse_tint`. Applies to every
# kind EXCEPT `_ORGANIC_KINDS`: `vegetation`'s tint is already the dark,
# desaturated "stripped waterlogged canopy" end of the palette and `log`'s
# is already wet bark (`_RAFT_TINT["log"]`'s own docstring) -- darkening
# either again would push them toward black with no reference to check it
# against. TUNED, NOT SOURCED for the exact factor: LBNL-48334's wet/dry
# concrete ratio (0.36/0.59 = 0.61) is the nearest measured analogue this
# codebase already cites (`build-hurricane-scenes` skill's "Vegetation does
# not darken" section) for a wet, saturated building material, and 0.60
# sits inside the requested 0.55-0.65 band around it.
_WET_DARKEN = 0.60
# Reuses `planks.wood_material`'s texture (the same "pale, bare-framing"
# argument `planks.py` makes — there is no dedicated foliage-mat asset in
# the repo) but tinted per class so a raft field is not one uniform colour —
# the same trick `planks._TINT` plays.
#
# THESE NUMBERS WERE NEVER TAKING EFFECT. `planks.wood_material` puts `tint`
# on `diffuse_color_constant`, and that input's own comment in `planks.py`
# claims it "MULTIPLIES the map in OmniPBR". It does not: `OmniPBR.mdl`
# (`kit/mdl/core/Base`) is explicit that `diffuse_color_constant` is the
# albedo `diffuse_texture` REPLACES, and `diffuse_tint` is what is
# "multiplied over the final albedo colour" — the same finding this
# codebase already made independently at least twice more (`surge.
# _dry_material`'s docstring; `quake_flow.py`'s "THE TINT HAS TO GO ON
# `diffuse_tint`", which names this exact function's comment as the wrong
# one to trust). The practical effect: every raft, whatever `kind`, was
# rendering as plain untinted `Ash_Planks` — timber, sheet, wall and
# vegetation all the same pale board colour — which is the "one uniform
# colour" failure this dict's own docstring says it exists to prevent.
# Fixed in `build_rafts`, which patches the shader `wood_material` returns
# rather than editing that function (outside this module).
_RAFT_TINT = {
    # RE-TUNED, DEBRIS D6 review 2026-09-01: "still looks too much like the
    # same green debris everywhere but the houses don't have any green."
    # `timber` was the one kind this module's own D3/D5 saturation passes
    # never touched — (1.00, 0.97, 0.90), a 10% channel spread, dead level
    # with `test_e_tint_targets_are_saturated_not_neutral`'s own >10% floor
    # (i.e. it would FAIL that check if it were even in the list the test
    # covers, which it is not — an oversight this review closes). A near-
    # white, nearly-achromatic albedo is exactly the case this codebase has
    # independently rediscovered three times now (`_RAFT_TINT`'s own "PALE
    # GREEN-GREY CARDBOARD" section below, `surge._dry_material`'s docstring,
    # the D3 review that saturated `panel`/`fence`/`siding_strip`): it has
    # almost no colour of its own, so whatever the water/sky ambient throws
    # at it dominates the rendered pixel, and the shipped renders' raft
    # field (measured directly off `FINAL8_L3_brown/raft_field_obl.png`, a
    # broad flat piece sized like this kind: sRGB ~(120, 135, 125), G > B >
    # R) is the result. Retargeted to a real pale-SAWN-wood colour with a
    # visible warm cast (spread (0.62-0.42)/0.62 = 32%) — still reads as
    # "fresh-cut lumber", just no longer a blank canvas for ambient tint.
    "timber": (0.62, 0.55, 0.42),
    # PALE YELLOW PLYWOOD, ADDED (DEBRIS D3 review, 2026-08-31): `sheet` is
    # the generic thin flat mat and `panel` (below) is the named 4x8 OSB
    # sheet; splitting their COLOUR as well as their kind is what turns two
    # near-identical pale rectangles into "plywood" and "OSB" at a glance —
    # see that review's own reference values (plywood ~ (0.55, 0.42, 0.25)
    # linear absolute albedo, DRY).
    "sheet":  (0.55, 0.42, 0.25),
    # SIDING-CREAM FACE, ADDED same review: a `wall` piece is mostly the
    # house's painted/sided OUTER face (the review's own reference:
    # white/cream siding ~ (0.75, 0.72, 0.66) linear) even though the real
    # object also shows bare studs on its back — one tint per kind (per this
    # module's own "keep one material per kind" rule) has to pick the more
    # common visible face, and from the air that is the siding, not the
    # framing.
    #
    # NUDGED FURTHER, DEBRIS D6 review 2026-09-01: 0.75/0.72/0.66 was a real
    # improvement over the original 0.88/0.86/0.82 but still only a 12%
    # channel spread — right at `test_e_tint_targets_are_saturated_not_
    # neutral`'s own >10% floor, the thinnest margin of any kind that test
    # covers, and `wall`'s own 1.5-3.5 x 1.0-2.4 m footprint is the size
    # class the shipped `FINAL8_L3_brown/raft_field_obl.png` teal-cardboard
    # piece actually measures as. Pushed to a 25% spread — still a pale
    # cream/white siding colour, just no longer thin enough for the ambient
    # water/sky tint to read through it.
    "wall":   (0.72, 0.64, 0.54),
    # THE COARSE HURRICANE CLASSES, ADDED 2026-08-31 alongside `_RAFT_DIMS`'s
    # own new entries, RE-TUNED the same day (D3 review) against real
    # flood-debris references instead of "how pale is dry lumber": every one
    # of the four below used to sit within 6-10% of NEUTRAL WHITE/GREY
    # (`wall` 0.88/0.86/0.82, `panel` 0.86/0.80/0.66, `siding_strip`
    # 0.90/0.88/0.84) — a mat with almost no colour of its own is exactly
    # what a stray ambient/GI tint (the muddy water beside it, the sky
    # above it) reads AS, which is the "pale green-grey cardboard" defect
    # the review reported (`~/hurricane_previews/ROUND1_L3/raft_field_obl.
    # png`: measured RGB ~(205, 208, 190), i.e. G >= R > B — no wood is that
    # colour). Giving each kind its own SATURATED, real-material colour is
    # the fix available in this module: it cannot control the sky or the
    # water, but it can stop handing them a near-white canvas to tint.
    # OSB ORANGE-BROWN. `panel`'s own docstring already says "OSB/plywood";
    # this is the OSB half of that pair now that `sheet` (above) carries the
    # plywood one, and it is the review's own OSB reference value verbatim.
    "panel":        (0.42, 0.27, 0.14),
    # WEATHERED GREY-BROWN, the review's own fence reference.
    "fence":        (0.30, 0.26, 0.20),
    # WHITE/CREAM, close to `wall`'s siding face (same material, torn off in
    # a strip instead of a whole section) but not identical to it — kept a
    # shade brighter/cleaner, since a single torn strip has less of the
    # house's grime and shadow than a whole wall panel with its stud shadow
    # lines.
    #
    # NUDGED ALONGSIDE `wall`, DEBRIS D6 review 2026-09-01, same "still only
    # a ~13% spread" reasoning — kept a shade brighter than `wall`'s own
    # retune, preserving the "less grime, less shadow" relationship between
    # the two.
    "siding_strip": (0.76, 0.68, 0.58),
    # TWO-TONE ROOF SECTION. `roof_top` is the weathered asphalt-shingle
    # face. `roof_under` is the bare sheathing/felt underside, pale like
    # `panel`'s OSB but not as saturated (felt/underlayment is greyer than
    # raw OSB). Being two literal `_RAFT_DIMS`/`_RAFT_KIND_WEIGHTS` KINDS
    # rather than one kind with a per-piece variant is what lets
    # `build_rafts`'s existing per-`kind` grouping produce the two-tone mesh
    # with no change to that function.
    #
    # RE-TUNED AGAIN, 2026-08-31 (DEBRIS D5 review: "some of the debris have
    # the burnt texture, we don't want that"). The PREVIOUS value here,
    # (0.06, 0.06, 0.06), was chosen to be "genuinely near-black shingle" and
    # it overshot: measured directly off the shipped
    # `FINAL3_L3_brown/raft_field_obl.png` (PIL/numpy, pixels with luma < 90
    # clustered and averaged), the dark debris in that render reads sRGB
    # ~(62, 64, 72)/255 — luma ~26%, i.e. AT or BELOW char/soot territory —
    # and no burn/scorch/soot texture is bound anywhere on this path to
    # explain it (grepped `planks.py`/`washaway.py`/`vegetation.py`'s own
    # debris-facing code: the only "char" hits are `planks.py`'s "NO CHAR,
    # ANYWHERE" section header and unrelated words containing the substring
    # — "Charley", "character" — so this was a VALUE problem, not a bound
    # texture). Real weathered asphalt shingle is dark but not soot-black;
    # (0.13, 0.13, 0.14) is the requested value, still visibly the darkest
    # kind in the field (`test_e_tint_targets_are_saturated_not_neutral`'s
    # own "roof_top reads as near-black shingle" check, band widened to
    # match) but clearing `_RAFT_MIN_DRY_SRGB_LUMA` (see that constant, just
    # below this table) instead of sitting under it.
    "roof_top":     (0.13, 0.13, 0.14),
    "roof_under":   (0.50, 0.40, 0.27),
    # DARK AND ORGANIC ON PURPOSE — the opposite end of the palette from the
    # pale bare-framing tints above. A mat of stripped, waterlogged canopy
    # reads near-black-green against bright floodwater, which is exactly the
    # class separator the coverage audit calls out (S2 row 2: "a strong,
    # high-texture-frequency albedo against a muddy or reflective water
    # surface"). Same "no dedicated asset, so tint what is already bound"
    # trade `_slab_material` makes for bare concrete, applied to
    # `planks.wood_material`'s texture instead of an untextured OmniPBR.
    # DARK sodden foliage. The first-shipped value resolved to pale sage,
    # which made the whole open-water raft field read MINT under the storm
    # dome — bench-verified 2026-08-31 (tools/raft_material_bench.py).
    #
    # DARKENED FURTHER, DEBRIS D6 review 2026-09-01: (0.105, 0.115, 0.060)
    # still has G fractionally over R (a ~10% relative gap) — technically
    # exempt from the building-kind "no G > R" guard (this IS organic
    # matter), but with the whole raft field under review for reading green,
    # a value that is itself the ONE kind allowed to have G on top is worth
    # pulling further from the boundary rather than defending it. Every
    # channel scaled down toward the task's own "if it reads MINTY, darken
    # further" fallback; still G-leaning by design (real stripped canopy is
    # olive, not brown) but at a luma low enough that the hue is barely
    # perceptible rather than a visible sage cast.
    "vegetation": (0.082, 0.066, 0.040),  # SODDEN BROWN, R>G. The olive exemption died 2026-09-01 -- user, live in the GUI: "still looks like too much green debris". Dead vegetation in muddy floodwater is brown.
    # WET BARK, NOT SAWN TIMBER. A bole that has been in the water is the
    # darkest thing in the debris field -- soaked bark is well under half the
    # albedo of the pale `Ash_Planks` map every raft class is bound to, and
    # leaving `log` out of this dict would have given it the untinted board
    # colour (1.0 through `_RAFT_TEX_MEAN_LINEAR`), i.e. a bright cream
    # cylinder. NOT tinted against `_RAFT_TEX_MEAN_LINEAR` any more, see
    # `build_rafts`'s own "BARK, NOT ASH_PLANKS" section — `log` now binds
    # `vegetation.bark_material` (real bark, ADDED for the DEBRIS D5 review's
    # "I don't really see floating tree trunks" note) and is normalised
    # against that texture's OWN measured mean instead
    # (`_LOG_BARK_TEX_MEAN_LINEAR`).
    #
    # CHECKED, NOT NUDGED (D5 review: "must read brown, not black"):
    # R:G:B = 1.00:0.78:0.55 is a clear warm brown, not a neutral, and its
    # `_srgb_luma` (0.322, computed below) already clears
    # `_RAFT_MIN_DRY_SRGB_LUMA` with room to spare — the real bark TEXTURE
    # (as opposed to this flat tint alone) does the rest of the "reads as
    # wood, not char" work.
    "log": (0.105, 0.082, 0.058),
}


def _srgb_luma(rgb_linear):
    """Perceptual luma (Rec. 709 weights), computed on the sRGB-ENCODED
    channels — "sRGB luma" the way it is normally meant when eyeballing a
    rendered PNG's own stored (already gamma-encoded) pixels, not the
    physically-linear relative luminance a light-transport calculation
    would want. Used only to hold `_RAFT_TINT`'s own DRY reference values to
    `_RAFT_MIN_DRY_SRGB_LUMA`, just below."""
    def _enc(c):
        c = max(0.0, min(1.0, float(c)))
        return 12.92 * c if c <= 0.0031308 else 1.055 * c ** (1.0 / 2.4) - 0.055
    r, g, b = (_enc(c) for c in rgb_linear)
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


# NO DEBRIS KIND MAY RENDER AS CHAR — ADDED (DEBRIS D5 review: "some of the
# debris have the burnt texture, we don't want that"). `roof_top`'s own
# docstring above records the render evidence (measured ~26% sRGB luma
# against this floor's 28%) and the fix; this assertion is what stops a
# FUTURE edit from quietly reintroducing the same defect on any kind, not
# just the one that shipped it. Runs once at import time, on the small,
# fixed `_RAFT_TINT` table — negligible cost, and a table this size is
# exactly where "assert the invariant where the data lives" is cheap enough
# to always be on rather than gated behind a test file. `_ORGANIC_KINDS`
# (`vegetation`, `log`) are DELIBERATELY held to the SAME floor as
# everything else here: both already clear it (checked in their own
# `_RAFT_TINT` comments above) and "dark and organic on purpose" is not a
# license to go char-dark, it is a different hue at a similar VALUE.
_RAFT_MIN_DRY_SRGB_LUMA = 0.28
for _k, _v in _RAFT_TINT.items():
    assert _srgb_luma(_v) >= _RAFT_MIN_DRY_SRGB_LUMA, (
        "_RAFT_TINT[{0!r}] = {1} reads at {2:.1%} sRGB luma, under the "
        "{3:.0%} char-avoidance floor".format(
            _k, _v, _srgb_luma(_v), _RAFT_MIN_DRY_SRGB_LUMA))
del _k, _v
# MEASURED off `planks.WOOD_BASE`, full 2048x2048 resolution, sRGB->linear —
# the same method `surge.py`'s `_MUD_TEX_MEAN_LINEAR`/
# `_WASHOVER_TEX_MEAN_LINEAR` use, so a reader comparing files sees the same
# convention. Ash_Planks is a genuinely pale map, nearly 7x Soil_Mud's — using
# a mud constant here by analogy would have been exactly the "close enough to
# share" mistake `_WASHOVER_TEX_MEAN_LINEAR` documents.
#
# A SCALAR HERE WAS THE SECOND HALF OF THE "PALE GREEN-GREY CARDBOARD" BUG
# (D3 review, 2026-08-31; see `_RAFT_TINT`'s docstring for the render
# evidence and the retuned targets). This constant used to be ONE NUMBER,
# 0.351 — the average of the texture's three per-channel means collapsed
# into a single scalar — and `build_rafts` divided EVERY target channel by
# that same scalar:
#
#     t = tuple(c / _RAFT_TEX_MEAN_LINEAR for c in tint)
#
# which is only correct if the texture is NEUTRAL (R, G and B share one
# mean). Ash_Planks is not: measured per channel it is
#
#     R 0.4754   G 0.3345   B 0.2417   (linear; scalar average 0.3507)
#
# — R is very nearly TWICE B, an ordinary warm pale wood. Dividing every
# target channel by the shared scalar instead of by its OWN channel's mean
# means the rendered result silently DRIFTS from every kind's own tint by
# `texture_mean[c] / scalar_mean`: +35% on R (0.475/0.351), roughly correct
# on G (0.334/0.351), and -31% on B (0.242/0.351) — i.e. every raft rendered
# WARMER and LESS BLUE than its own authored `_RAFT_TINT` entry said it
# should, regardless of kind. That drift alone pushes warm, not green, so it
# is not the whole story (the `_RAFT_TINT` values themselves were also too
# pale/neutral to resist ambient tinting — see that dict's docstring) — but
# it is real, it is provable from the shipped texture file, and it made
# every kind's ABSOLUTE albedo wrong regardless of which way the hue moved.
#
# Fixed by making this a PER-CHANNEL tuple and dividing element-wise
# (`build_rafts`): `diffuse_tint[c] = tint[c] / _RAFT_TEX_MEAN_LINEAR[c]`
# guarantees the AVERAGE rendered albedo equals the authored target exactly,
# for any texture bias, because `texture_mean[c] * (target[c] /
# texture_mean[c]) == target[c]` by construction — the fix does not depend
# on knowing why the texture is warm, only on no longer assuming it isn't.
_RAFT_TEX_MEAN_LINEAR = (0.4754, 0.3345, 0.2417)
# `_RAFT_TILT_DEG`/`_RAFT_BOB_M` (a flat +-4 deg tilt and a +-3 cm Z jitter)
# are RETIRED 2026-08-31: superseded by the per-kind `_RAFT_TILT_RANGE_DEG`
# and the draft-fraction jitter below `_one_raft`, which vary the same idea
# by kind and can no longer sink a thin sheet under the water plane the way
# an absolute Z jitter could.
# A DEBRIS MAT IS A TANGLE, NOT A BOARD. Every cluster draw used to place one
# box, so a "raft field" was evenly spaced single planks -- confetti, at any
# density. Flood debris arrives as interlocked rafts: a few pieces at mixed
# angles within arm's reach of each other, with clear water between the
# mats. Drawing 1-3 pieces inside a 0.7 m radius per cluster point buys that
# read for nothing (the pieces still merge into the same per-kind mesh).
#
# RAISED, 2026-08-31, alongside `raft_per_house`/`raft_per_obstacle`
# ("more tangles, larger tangle sizes near obstructions"): 2-6 pieces
# (average 4.0, was 2.0 -- a 2x multiplier on top of the draw-rate raise)
# inside a slightly wider 0.75 m radius, so a bigger tangle still reads as
# separate interlocked pieces rather than one box's worth of overlapping
# geometry at the old 0.7 m. TUNED, NOT SOURCED.
_RAFT_TANGLE = (2, 6)
_RAFT_TANGLE_R_M = 0.75
# WHAT IS FLOATING AGAINST A HOUSE CAME OFF A HOUSE. The kind mix is one
# global number, so a plank was as likely to appear in mid-field as against a
# wall and a canopy mat as likely to pile on a porch as to drift. Near a
# standing building the mix has to skew to building material -- that is the
# review note ("wood planks etc FROM HOUSES") and it is also just what the
# flow does: the house is the source. Vegetation is capped rather than
# removed (there are still trees in the yard), and the building classes keep
# `raft_kind_weights`' own 5:3:2 ratio.
_RAFT_HOUSE_VEG_CAP = 0.20


def _house_mix(weights):
    """`weights` re-skewed toward building material for a house cluster.

    Caps `vegetation` at `_RAFT_HOUSE_VEG_CAP` and gives what it gave up to
    timber/sheet/wall in their existing proportion, so a caller's live
    `raft_kind_weights(veg_share)` still steers the mix -- it just cannot
    make the debris against a wrecked house be mostly leaves.
    """
    w = dict(weights or _RAFT_KIND_WEIGHTS)
    org = sum(float(w.get(k, 0.0)) for k in _ORGANIC_KINDS)
    if org <= _RAFT_HOUSE_VEG_CAP or org <= 0.0:
        return w
    bld = {k: float(v) for k, v in w.items() if k not in _ORGANIC_KINDS}
    tot = sum(bld.values())
    if tot <= 0.0:
        return w
    freed = org - _RAFT_HOUSE_VEG_CAP
    out = {k: v + freed * (v / tot) for k, v in bld.items()}
    # squeeze the organic classes proportionally into what is left
    for k in _ORGANIC_KINDS:
        out[k] = float(w.get(k, 0.0)) * (_RAFT_HOUSE_VEG_CAP / org)
    return out


def raft_kind_weights(veg_share=None):
    """Kind-weight dict for `_weighted_kind`/`raft_specs`, built from a
    single vegetation SHARE (0..1) rather than a second copy of
    `hurricane.debris_mix`'s own 72/28 split.

    `veg_share` is the caller's own `mix["leaf_litter"] + mix["limbs_fronds"]`
    from `hurricane.debris_mix(i, rng)` — see the module docstring's "THE
    RAFT FIELD WAS ALL BUILDING MATERIAL" for why a number crosses here
    instead of a re-derivation: this module never imports `hurricane`, it is
    handed the one float it needs, the same contract `water_level_m`/
    `flow_bearing_deg` already use for numbers that cross from `surge.py`.

    `None` (the default) returns `_RAFT_KIND_WEIGHTS` UNCHANGED — the static
    72/28 fallback, not zero vegetation — so a caller with no `debris_mix`
    result still gets a scene with vegetation rafts in it, just not one
    tracking a specific scene's own live intensity-derived mix.

    THE BUILDING SHARE IS SCALED, NOT RENORMALISED, from `_BUILD_MIX` --
    the SAME dict `_RAFT_KIND_WEIGHTS` itself is built from (factored out
    2026-08-31 so this function and that constant cannot drift apart the
    way the old inline 0.5/0.3/0.2 could have). Multiplying every entry by
    `(1 - veg_share)` keeps `_BUILD_MIX`'s internal ratio and lets
    `vegetation`/`log` take the rest.
    """
    if veg_share is None:
        return dict(_RAFT_KIND_WEIGHTS)
    v = max(0.0, min(1.0, float(veg_share)))
    bld = 1.0 - v
    out = {"vegetation": v * (1.0 - _LOG_SHARE_OF_VEG),
          "log": v * _LOG_SHARE_OF_VEG}
    out.update({k: bld * m for k, m in _BUILD_MIX.items()})
    return out


def _weighted_kind(rng, weights=None):
    w = weights or _RAFT_KIND_WEIGHTS
    r = rng.random() * sum(w.values())
    acc = 0.0
    for name, wgt in w.items():
        acc += wgt
        if r <= acc:
            return name
    return "timber" if "timber" in w else next(iter(w))


# ---------------------------------------------------------------------------
# DRAFT AND ATTITUDE, ADDED 2026-08-31 -- the "floating debris" review found
# every raft lying dead flat, dead centred on the water plane (or, for
# `log`, lifted so 75% of its diameter rode ABOVE it -- see the arithmetic
# this replaces), with at most +-4 degrees of tilt: a paper cut-out resting
# on TOP of an opaque surface rather than a piece actually IN the water.
#
# Fixed by computing, per piece, the TRUE vertical half-extent of the
# rotated box -- the same closed-form `planks._lay` uses for ground seating,
# sum of the three local half-extents' absolute projections onto world Z --
# and placing the centre so a KIND-DEPENDENT FRACTION of that extent sits
# below `water_level_m`:
#
#     z_centre = water_level_m + half_extent * (1 - 2 * draft_frac)
#
# `draft_frac` is how much of the piece rides submerged. Flat building
# material floats HIGH on its own broad area (`_RAFT_DRAFT_DEFAULT = 0.85`
# puts most of the thickness under water and a thin proud lip above it --
# the "waterlogged board" look); a round log floats LOW (0.50 -- half its
# diameter, matching this module's own pre-existing "rides with roughly
# half its diameter above the surface" docstring, which the old
# `+ 0.25 * w` arithmetic never actually delivered); a torn roof section
# rides at 0.60 (real trapped air under the deck); a vegetation mat at 0.70
# (branches and root balls add buoyant bulk but the leaf mass sits low).
# TUNED, NOT SOURCED -- there is no published hurricane-debris draft table
# -- but every fraction is bounded well inside (0, 1) so a piece ALWAYS
# breaks the surface: `top_z > water_z > bottom_z` for every spec is the
# hard invariant `tests/test_washaway_debris.py` pins, per kind.
_RAFT_DRAFT_DEFAULT = 0.85
_RAFT_DRAFT_FRAC = {
    "log": 0.50,
    "roof_top": 0.60, "roof_under": 0.60,
    "vegetation": 0.70,
}
# Jitter on the FRACTION, not on Z directly -- an absolute Z jitter can push
# a thin piece's draft past 0 or 1 and either sink it under the opaque quad
# or lift it clear of the water, exactly the failure mode the old
# `_RAFT_BOB_M` risked on a 2 cm-thick sheet. Clamped well inside (0, 1).
_RAFT_DRAFT_JITTER = 0.06
_RAFT_DRAFT_MIN, _RAFT_DRAFT_MAX = 0.12, 0.90

# ROLL/PITCH RANGE, DEGREES -- "a random roll/pitch (3-12 deg), logs and
# limbs more". Building material gets the smaller, generic range; round
# boles and vegetation mats get more (a log ALSO keeps its own free 0-360
# degree ROLL about its long axis -- see `_one_raft` -- since a round
# section's shape and draft do not depend on which way round it has spun).
_RAFT_TILT_RANGE_DEG = {
    "log": (8.0, 24.0),
    "vegetation": (6.0, 16.0),
    "roof_top": (4.0, 12.0), "roof_under": (4.0, 12.0),
}
_RAFT_TILT_RANGE_DEFAULT = (3.0, 12.0)

# TANGLE LEAN -- "adjacent pieces in a tangle overlap/lean on each other".
# The Nth (N > 0) piece drawn inside one `_RAFT_TANGLE` cluster is lifted
# toward the surface by up to `_RAFT_LEAN_FRAC` of its OWN submerged depth,
# as if resting partly on the piece(s) already drawn there. Capped at 0.55
# (not 1.0) so the "always breaks the surface" invariant above holds for a
# leaned piece too -- at least `1 - _RAFT_LEAN_FRAC` of its intended draft
# stays below `water_level_m`.
_RAFT_LEAN_FRAC = 0.55

# HOUSE-MATCHED SKIN, ADDED (DEBRIS D5 review: "the debris placed in the
# water all look like they have the same texture. match them to the houses
# they are near"). `planks.py`'s own doctrine is the model: `siding`/`deck`
# are the two classes that carry a house's colour (`planks.SKINNED`) and
# everything else is bare timber, "the inside of a wall wherever it came
# from". This raft field's own two building-material shapes that visually
# stand in for those same two things are `wall`/`siding_strip` (the outside
# of a wall, torn off whole or in a strip) and `roof_top` (the shingle FACE
# of a torn roof section -- `roof_under` is the bare sheathing/felt
# underside and deliberately stays OUT of this map, same as `planks.STOCK`'s
# own `joist`/`stud`/`sheathing`/`board` staying bare). Maps a skinnable
# raft `kind` to the KEY a `mh.palette_skins(...)`-shaped
# `{"siding": <name>, "deck": <name>}` dict uses for it -- this module never
# imports `detail.modular_house` (the same import-boundary rule the module
# docstring states for `hurricane`), so it only ever sees that dict's own
# two key names, never the material names or the palette machinery behind
# them.
_RAFT_SKIN_CLASS = {"wall": "siding", "siding_strip": "siding",
                    "roof_top": "deck"}


def _one_raft(x, y, rng, kn, weights=None, tangle_idx=0, skin=None):
    """One floating debris piece's spec (`planks._box`'s schema plus
    `kind`) -- position, size, draft and attitude, all resolved here so
    `build_rafts` is pure geometry authoring.

    `tangle_idx` is this piece's 0-based position within one
    `_RAFT_TANGLE` cluster draw: 0 for the background scatter and for the
    first piece of every cluster, >0 for a piece drawn to overlap/lean on
    the one(s) before it -- see `_RAFT_LEAN_FRAC`.

    `skin`, ADDED for the "match them to the houses they are near" fix: an
    OPTIONAL `{"siding": <name>, "deck": <name>}` dict (`_RAFT_SKIN_CLASS`'s
    own key names) -- the caller's own `mh.palette_skins(palette)`, crossed
    in as a plain dict the same way every other cross-module value in this
    file crosses (numbers, callables, and now this). If the drawn `kind` is
    one of `_RAFT_SKIN_CLASS`'s keys AND `skin` supplies a name for the
    matching slot, the returned spec carries `"skin"` = that name, the exact
    field `planks.scatter_from_wreck(skins=...)` already sets on a `siding`/
    `deck` land piece — `build_rafts` groups on `(kind, skin)` and binds the
    named material for any group that has one (see that function's own
    docstring). Every OTHER kind, and every call that omits `skin`, is
    unaffected -- background scatter, obstacle clusters, the strand line and
    drift lines all pass `skin=None` (the default) and stay generic, which
    is correct: open water and a tree/car catch are not "from a house".
    """
    kind = _weighted_kind(rng, weights)
    lo_l, hi_l = _RAFT_DIMS[kind][0]
    lo_w, hi_w = _RAFT_DIMS[kind][1]
    lo_t, hi_t = _RAFT_DIMS[kind][2]
    l = rng.uniform(lo_l, hi_l)
    w = rng.uniform(lo_w, hi_w)
    # ROUND IN SECTION: drawing `t` independently of `w` would give a
    # flattened bole rather than a round one.
    t = w if kind == "log" else rng.uniform(lo_t, hi_t)

    # MAGNITUDE drawn from `[tlo, thi]` DIRECTLY, sign separately -- NOT
    # `rng.uniform(-1, 1) * rng.uniform(tlo, thi)`, which multiplies a
    # continuous +-1 by the range and lets the magnitude fall anywhere in
    # `[0, thi]` (caught by this file's own offline test: piece pitches were
    # coming out as low as 0.00 deg against a claimed 3 deg floor). The
    # `rng.choice((-1.0, 1.0)) * rng.uniform(...)` form is the same one
    # `shift_spec`/`car_shift_spec` already use elsewhere in this module for
    # exactly this reason.
    tlo, thi = _RAFT_TILT_RANGE_DEG.get(kind, _RAFT_TILT_RANGE_DEFAULT)
    pitch = rng.choice((-1.0, 1.0)) * rng.uniform(tlo, thi)
    roll = (rng.uniform(0.0, 360.0) if kind == "log"
            else rng.choice((-1.0, 1.0)) * rng.uniform(tlo, thi))
    yaw = rng.uniform(0.0, 360.0)

    # THE TRUE VERTICAL HALF-EXTENT of the rotated box -- `planks._lay`'s
    # own formula, R = Rz(yaw) . Ry(pitch) . Rx(roll); yaw does not change
    # the z-extent so it is omitted here exactly as that function omits it.
    p, r = math.radians(pitch), math.radians(roll)
    half_extent = 0.5 * (abs(t * math.cos(p) * math.cos(r))
                         + abs(w * math.cos(p) * math.sin(r))
                         + abs(l * math.sin(p)))
    half_extent = max(half_extent, 1e-4)

    f = _RAFT_DRAFT_FRAC.get(kind, _RAFT_DRAFT_DEFAULT)
    f += rng.uniform(-_RAFT_DRAFT_JITTER, _RAFT_DRAFT_JITTER)
    f = min(_RAFT_DRAFT_MAX, max(_RAFT_DRAFT_MIN, f))
    submerged = f * 2.0 * half_extent
    z = float(kn["water_level_m"]) + half_extent - submerged
    if tangle_idx > 0:
        z += rng.uniform(0.0, _RAFT_LEAN_FRAC) * submerged

    spec = {
        "kind": kind, "x": x, "y": y, "z": z,
        "l": l, "w": w, "t": t,
        "yaw": yaw, "pitch": pitch, "roll": roll,
    }
    if skin:
        nm = skin.get(_RAFT_SKIN_CLASS.get(kind))
        if nm:
            spec["skin"] = nm
    return spec


# STUB BRANCHES ON A LOG, ADDED (DEBRIS D5 review, part of "floating tree
# trunks/fallen down trees"): "some with 1-2 stub branches (a smaller box at
# a 20-40 deg angle near one end)". A whole-bole `log` raft is otherwise a
# perfectly smooth cylinder-as-box, which reads fine at altitude for MOST of
# the class but not for every one -- a real uprooted or storm-snapped tree
# usually still carries a stub or two where a limb tore off rather than a
# clean-sawn round. `_LOG_BRANCH_P` keeps this a MINORITY of logs (most
# stay a plain bole); the branch is a second, smaller box attached near one
# end, angled off the log's own long axis, sharing that log's `kind` (so it
# merges into the exact same `build_rafts` mesh/material -- no new group,
# no extra prim) and, where the log carries one, its `skin` too (a house-
# matched log makes no sense, but this keeps the field consistent should a
# future caller ever thread one through).
_LOG_BRANCH_P = 0.35
_LOG_BRANCH_COUNT = (1, 2)
_LOG_BRANCH_ANGLE_DEG = (20.0, 40.0)
_LOG_BRANCH_LEN_M = (0.5, 1.3)
_LOG_BRANCH_DIAM_M = (0.08, 0.18)
# How far toward one end the branch attaches, as a fraction of the log's own
# HALF-length -- 0.85 puts it near the tip without hanging off the end of
# the bole entirely.
_LOG_BRANCH_END_FRAC = 0.85
# A branch pokes up MORE than the log body it is attached to (a real stub
# catches less water than the round bole it juts from) -- a shallower draft
# than `_RAFT_DRAFT_FRAC["log"]`'s 0.50, jittered the same bounded way
# `_one_raft` jitters every other kind's draft. Computed from the WATER
# LEVEL directly with the branch's OWN rotated half-extent (`_one_raft`'s
# own formula) rather than inherited from the parent log's `z` -- the two
# floats are only ever a few centimetres apart in practice (both anchor to
# the same water surface) and deriving it independently is what GUARANTEES
# the hard "top > water > bottom" invariant every other raft in this module
# already satisfies, for any pitch/roll draw, rather than hoping an
# inherited offset happens to still clear the surface after the branch's
# own tilt is added on top of it.
_LOG_BRANCH_DRAFT = 0.35
_LOG_BRANCH_DRAFT_JITTER = 0.08
_LOG_BRANCH_DRAFT_MIN, _LOG_BRANCH_DRAFT_MAX = 0.12, 0.85


def _log_branch_specs(log_spec, rng, water_level_m):
    """0-2 small stub-branch box specs for one `log`-kind raft spec, or `[]`
    (the common case, `1 - _LOG_BRANCH_P` of the time). Every returned spec
    is schema-identical to any other raft box (`kind="log"`, same `_box`
    schema `build_rafts`/`_merge_boxes` already consume) so it costs nothing
    beyond a few extra points in that one merged mesh.

    POSITIONED OFF THE LOG'S OWN ROTATED AXIS, not a fresh random spot: a
    branch has to look ATTACHED. `ax` below is the log's local +X (its own
    length axis) expressed in world space via the same `R = Rz(yaw) .
    Ry(pitch) . Rx(roll)` `planks._box`/`_one_raft` already use, so walking
    `s` metres along it from the log's own centre lands the branch's
    (x, y) base point on the log's actual surface line, whichever way it
    has rolled. The branch is then given its own small pose
    (`_LOG_BRANCH_ANGLE_DEG` off the log's pitch, a little yaw/roll scatter)
    rather than reusing the log's pose outright -- a branch that stuck out
    from the trunk at the log's own flat attitude would look like a second,
    parallel log, not a limb. `z` is computed independently (see
    `_LOG_BRANCH_DRAFT`'s own docstring) from `water_level_m`, NOT inherited
    from `log_spec["z"]`, so the invariant holds regardless of the branch's
    own tilt.
    """
    if rng.random() > _LOG_BRANCH_P:
        return []
    hl = 0.5 * float(log_spec["l"])
    yaw = math.radians(float(log_spec["yaw"]))
    pitch = math.radians(float(log_spec["pitch"]))
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    # The log's own local +X (length) axis, in world space -- first COLUMN
    # of R = Rz(yaw) . Ry(pitch) . Rx(roll) (roll rotates ABOUT that axis,
    # so it does not move it).
    ax = (cy_ * cp, sy_ * cp, -sp)

    out = []
    for _ in range(rng.randint(*_LOG_BRANCH_COUNT)):
        s = rng.choice((-1.0, 1.0)) * hl * _LOG_BRANCH_END_FRAC
        blen = rng.uniform(*_LOG_BRANCH_LEN_M)
        bdiam = rng.uniform(*_LOG_BRANCH_DIAM_M)
        ang = rng.uniform(*_LOG_BRANCH_ANGLE_DEG) * rng.choice((-1.0, 1.0))
        b_yaw = math.degrees(yaw) + rng.uniform(-25.0, 25.0)
        b_pitch = math.degrees(pitch) + ang
        b_roll = float(log_spec["roll"]) + rng.uniform(-15.0, 15.0)

        # SAME true-vertical-half-extent formula `_one_raft`/`planks._lay`
        # use, for THIS branch's own (l, w=t=bdiam, pitch, roll).
        bp, br = math.radians(b_pitch), math.radians(b_roll)
        half_extent = 0.5 * (abs(bdiam * math.cos(bp) * math.cos(br))
                             + abs(bdiam * math.cos(bp) * math.sin(br))
                             + abs(blen * math.sin(bp)))
        half_extent = max(half_extent, 1e-4)
        f = _LOG_BRANCH_DRAFT + rng.uniform(-_LOG_BRANCH_DRAFT_JITTER,
                                            _LOG_BRANCH_DRAFT_JITTER)
        f = min(_LOG_BRANCH_DRAFT_MAX, max(_LOG_BRANCH_DRAFT_MIN, f))
        submerged = f * 2.0 * half_extent
        bz = float(water_level_m) + half_extent - submerged

        spec = {
            "kind": "log",
            "x": log_spec["x"] + ax[0] * s,
            "y": log_spec["y"] + ax[1] * s,
            "z": bz,
            "l": blen, "w": bdiam, "t": bdiam,
            "yaw": b_yaw, "pitch": b_pitch, "roll": b_roll,
        }
        if log_spec.get("skin"):
            spec["skin"] = log_spec["skin"]
        out.append(spec)
    return out


def _append_raft(out, x, y, rng, kn, weights=None, tangle_idx=0, skin=None,
                 exclude_fps=None):
    """`_one_raft(...)`, appended to `out`, PLUS any stub-branch specs
    (`_log_branch_specs`) if the draw came up `log` -- the one call every
    raft-placing loop in this module should use instead of a bare
    `out.append(_one_raft(...))`, so a floating trunk's branches (see that
    function's own docstring) show up wherever logs can appear at all:
    the open-water background scatter, house/obstacle clusters, the strand
    line and the mid-water drift lines alike. Returns the main spec (not
    the branches), matching `_one_raft`'s own return contract for any
    caller that wants to inspect what it just placed.

    `exclude_fps`, ADDED alongside the branch fix itself: a branch attaches
    up to `hl * _LOG_BRANCH_END_FRAC` (up to ~4.7 m, at the new longer `log`
    range) from the log's OWN (already footprint-checked) `(x, y)` — far
    enough that a branch on a log placed just outside a house's exclusion
    circle can still land inside it, which is exactly the "rafts must not
    intersect what they are collected against" invariant every other piece
    in this module already respects (`_inside_any_footprint`,
    `test_d_rafts_never_intersect_footprints`). Pass the SAME `all_fps` list
    the caller already built for its own `_inside_any_footprint` checks; a
    branch that fails it is silently dropped (the log itself still stands —
    only that one stub is skipped), never counted against `_LOG_BRANCH_P`'s
    own draw. `None` (a caller with no footprint list to hand, if one ever
    exists) skips the check entirely.
    """
    spec = _one_raft(x, y, rng, kn, weights, tangle_idx=tangle_idx, skin=skin)
    out.append(spec)
    if spec["kind"] == "log":
        for b in _log_branch_specs(spec, rng, kn["water_level_m"]):
            if exclude_fps and _inside_any_footprint(b["x"], b["y"],
                                                     exclude_fps):
                continue
            out.append(b)
    return spec


def _footprint_list(entries, default_fp):
    """`entries` (`(x, y)` or `(x, y, footprint_m)` tuples) normalised to
    `(x, y, footprint_m)` triples, `default_fp` filling in a missing third
    element. Shared by `raft_specs`'s houses/obstacles/exclusion handling
    so the three do not each re-derive this."""
    out = []
    for e in (entries or ()):
        fp = float(e[2]) if len(e) > 2 else float(default_fp)
        out.append((float(e[0]), float(e[1]), fp))
    return out


def _inside_any_footprint(px, py, footprints):
    """True if `(px, py)` falls inside any `(x, y, footprint_m)` circle.

    RAFTS MUST NOT INTERSECT WHAT THEY ARE COLLECTED AGAINST. A circle of
    diameter `footprint_m` is the same coarse approximation
    `_measure_house`'s own `12 x 9 m` fallback already is (a house is not
    circular, but the review's own suggestion was "a footprint per style ...
    or a 12x10 m box" — a circle is the cheaper of the two and errs the
    same direction, slightly over-excluding a square footprint's corners
    rather than under-excluding them)."""
    for fx, fy, ffp in footprints:
        r = 0.5 * ffp
        if (px - fx) ** 2 + (py - fy) ** 2 < r * r:
            return True
    return False


# ---------------------------------------------------------------------------
# SIZE-MIX CAP, ADDED (DEBRIS D3 review, 2026-08-31): "real mats are mostly
# small stuff with a few large pieces." Measured against the shipped render
# (`~/hurricane_previews/ROUND1_L3/raft_field_obl.png`), the mats against a
# house and along the fence line are DOMINATED by `wall` (1.5-3.5 x 1.0-2.4
# m) and `roof_top`/`roof_under` (2.0-4.0 m) pieces — every one of which has
# a MINIMUM possible plan area already past 2 m2 (`wall` >= 1.5, `roof_*` >=
# 3.0, `panel` >= 2.6), so the size skew is not a rare outlier draw, it is
# every single piece of those kinds.
#
# TWO BUCKETS, NOT ONE, because "against a house" and "drifting in open
# water" are different photographs: a real collection point piles debris
# thickly enough that big sections DO end up there, while open water is
# mostly small stuff that separated and drifted. `_AREA_CAP_M2` is the
# plan-area threshold a piece counts as "large" past; `_AREA_CAP_NEAR`/
# `_AREA_CAP_FAR` are the requested max SHARE of a bucket's pieces that may
# be large ones.
_AREA_CAP_M2 = 2.0
_AREA_CAP_NEAR_HOUSE_M = 15.0
_AREA_CAP_NEAR_FRAC = 0.25
_AREA_CAP_FAR_FRAC = 0.10
# A piece over the cap is SHRUNK, never deleted or re-kinded: deleting would
# quietly thin the field's overall density (a caller tuned `raft_bg_per_100m2`
# etc. expecting a certain COUNT) and re-kinding would distort the very kind
# mix `_RAFT_KIND_WEIGHTS`/`raft_kind_weights` were tuned to produce. A
# smaller `wall` or `roof_top` is still an honest object — `_RAFT_DIMS`'s own
# `panel` comment already treats "smaller than nominal" as a torn fragment,
# not an error — so scaling `l`/`w` down (uniformly, preserving aspect) to
# just under `_AREA_CAP_M2` keeps the kind, the material and the piece count
# all intact and only changes which pieces read as "the few big ones".
_AREA_CAP_SHRINK_TARGET_M2 = _AREA_CAP_M2 * 0.92


def _plan_area_m2(spec):
    return float(spec["l"]) * float(spec["w"])


def _shrink_plan_area(spec, target_area_m2, water_level_m):
    """`spec` (an `_one_raft`-shaped dict) with `l`/`w` scaled down (aspect
    preserved, `t` untouched) so its plan area is at or under
    `target_area_m2`, and `z` RECOMPUTED so the piece keeps the SAME draft
    fraction (fraction of its own vertical extent submerged) it was drawn
    with — shrinking `l`/`w` shrinks the rotated box's own vertical
    half-extent too (`_one_raft`'s own formula), so leaving `z` alone would
    quietly change how much of the piece rides above the surface, and for a
    piece that started only barely breaking the surface could sink it
    entirely or lift it clear -- exactly the invariant
    `test_a_draft_invariant_every_kind` pins. A no-op (returns `spec`
    unchanged) if the piece is already at or under the target.
    """
    l0, w0, t0 = float(spec["l"]), float(spec["w"]), float(spec["t"])
    area0 = l0 * w0
    if area0 <= target_area_m2:
        return spec
    scale = math.sqrt(target_area_m2 / area0)
    l1, w1 = l0 * scale, w0 * scale
    p, r = math.radians(float(spec["pitch"])), math.radians(float(spec["roll"]))

    def _half_extent(l_, w_):
        return max(1e-4, 0.5 * (abs(t0 * math.cos(p) * math.cos(r))
                               + abs(w_ * math.cos(p) * math.sin(r))
                               + abs(l_ * math.sin(p))))

    half0 = _half_extent(l0, w0)
    frac_submerged = (half0 - (float(spec["z"]) - float(water_level_m))) \
        / (2.0 * half0)
    half1 = _half_extent(l1, w1)
    z1 = float(water_level_m) + half1 - frac_submerged * 2.0 * half1
    out = dict(spec)
    out["l"], out["w"], out["z"] = l1, w1, z1
    return out


def _cap_large_pieces(specs, house_fps, water_level_m,
                      near_m=_AREA_CAP_NEAR_HOUSE_M,
                      near_frac=_AREA_CAP_NEAR_FRAC,
                      far_frac=_AREA_CAP_FAR_FRAC,
                      area_m2=_AREA_CAP_M2):
    """`specs` with the excess "large" pieces (`_plan_area_m2 > area_m2`)
    SHRUNK, so at most `near_frac`/`far_frac` of the pieces within
    `near_m` of any house footprint centre / everywhere else are large
    ones. Order-preserving; every spec that started at or under the cap, or
    that the fraction already tolerates, is returned byte-identical.

    "Within `near_m` of a house" is centre-to-centre — a generous yard-and-
    collection-point zone, not a tight footprint band; the far bucket is
    simply everything else (mid-lake open water and the strand line alike).
    """
    def _near_house(s):
        for hx, hy, _fp in house_fps:
            if (float(s["x"]) - hx) ** 2 + (float(s["y"]) - hy) ** 2 \
                    <= near_m * near_m:
                return True
        return False

    near_idx, far_idx = [], []
    for i, s in enumerate(specs):
        (near_idx if _near_house(s) else far_idx).append(i)

    out = list(specs)
    for idx_list, frac in ((near_idx, near_frac), (far_idx, far_frac)):
        if not idx_list:
            continue
        large = sorted((i for i in idx_list if _plan_area_m2(out[i]) > area_m2),
                       key=lambda i: -_plan_area_m2(out[i]))
        allowed = int(math.floor(frac * len(idx_list)))
        excess = len(large) - allowed
        for i in large:
            if excess <= 0:
                break
            out[i] = _shrink_plan_area(out[i], _AREA_CAP_SHRINK_TARGET_M2,
                                       water_level_m)
            excess -= 1
    return out


# ---------------------------------------------------------------------------
# THE STRAND LINE -- ADDED (DEBRIS stream review): a wrack line at the TRUE
# waterline (depth ~ 0), distinct from `raft_min_depth_m`'s 0.25 m threshold
# for ordinary floating rafts (a plank does not need 25 cm of water to
# strand; a tangle of canopy and limbs does not need to float at all). The
# old background scatter placed NOTHING between depth 0 and
# `raft_min_depth_m` -- a dead band exactly where a real flood's single most
# legible feature, the debris line the water's edge leaves behind, actually
# sits: measured on the shipped L3 plate, ~25 pieces along a ~900 m
# shoreline (`tools/hurricane_debris_plot.py`'s own offline measurement).
# ---------------------------------------------------------------------------

_STRAND_TRIGGER_M = 0.02       # "just wet" -- the TRUE shoreline, well
                                 # inside `raft_min_depth_m`'s later gate
_STRAND_GRID_M = 5.0             # contour-march grid step (m)
# WIDENED AND MADE ASYMMETRIC, 2026-08-31 (DEBRIS D5 review: "the debris
# seems to form the border of the flooded area ... and that looks weird" —
# see this section's own "PATCHY, NOT CONTINUOUS" note just below for the
# other half of that fix). The old `_STRAND_HALFWIDTH_M = 1.1` drew a
# SYMMETRIC +/-1.1 m band straddling the true edge, which is part of what
# made the line read as a drawn contour: every piece sat within the same
# ~2.2 m of the exact waterline, banked evenly either side of it. Real
# stranded wrack does not respect that symmetry — some of it is carried a
# few metres up the bank as the water recedes, some of it never made it all
# the way in and sits a bit out in the shallows. `nvx, nvy` (below, in
# `_emit`) point toward DEEPER water (`surge._grad_deg`'s own convention),
# so NEGATIVE `off_n` is inland/toward the bank and POSITIVE is toward open
# water — `_STRAND_INLAND_MAX_M`/`_STRAND_WATER_MAX_M` are the two ends of
# that now-asymmetric range. TUNED, NOT SOURCED for the exact numbers (the
# requested 1-4 m inland / 2-6 m water band); this module already accepts
# the same "z comes from `water_level_m` alone, not the true local ground
# height" simplification at the OLD, narrower width (see `_one_raft`'s own
# docstring) — widening the band widens how far that approximation is
# asked to hold, not the category of approximation itself.
_STRAND_INLAND_MAX_M = 8.0        # toward the bank (away from the water)
_STRAND_WATER_MAX_M = 18.0         # toward open water
_STRAND_TANGENT_JITTER_M = 1.0   # along-shore jitter per piece -- keeps a
                                  # station's own draws off a single
                                  # dead-straight line ("in tangles")
# PATCHY, NOT CONTINUOUS -- ADDED (DEBRIS D5 review, same note as above).
# Every shoreline crossing used to call `_emit` unconditionally, so the
# strand line was one small tangle every `_STRAND_GRID_M`-ish ALONG THE
# ENTIRE PERIMETER -- from the air that reads as a drawn outline of the
# flood, not as debris. `_clump_state`/`_in_clump` (inside
# `_strand_line_specs`) turn that into alternating CLUMPS
# (`_STRAND_CLUMP_LEN_M`) and GAPS (`_STRAND_GAP_LEN_M`) along the sequence
# of crossings, using the running EUCLIDEAN distance between consecutive
# crossing points as a stand-in for true along-shore arc length. That stand-
# in is EXACT for a straight or gently-curved shoreline (this preset's own
# flood boundary, and the straight-line fixture `test_d2_strand_line_
# density_matches_spacing` already uses) and merely approximate for a
# sharply lobed one -- an acceptable trade for a wrack line nobody measures
# to the metre, and no worse than every other TUNED-NOT-SOURCED
# approximation already in this module. Mean clump 9 m / mean gap 19 m puts
# overall shoreline COVERAGE at 9/(9+19) ~= 32%, inside the requested
# 25-40% band -- see `tools/hurricane_debris_plot.py` for the offline
# measurement this is checked against.
_STRAND_CLUMP_LEN_M = (2.0, 8.0)
_STRAND_GAP_LEN_M = (25.0, 90.0)
# A jump between consecutive crossings bigger than this is treated as a
# DIFFERENT patch of shoreline (a disconnected lobe, or simply the march
# order jumping from the row scan to the column scan) rather than a
# continuation of the current clump/gap run -- the state re-rolls fresh
# instead of carrying a stale run length across a gap the eye would never
# perceive as continuous anyway.
_STRAND_CLUMP_JUMP_RESET_M = 20.0
# (water_level_m, spacing_m) calibration points -- ONE piece per
# `spacing_m` of shoreline, pinned at the two FROZEN presets' own
# still-water level (`surge.water_level`): L2 (2.0 m) -> 1.2 m spacing, L3
# (2.8 m) -> 0.7 m spacing. A deeper/more energetic surge leaves a heavier
# wrack line, so density scales with `water_level_m` instead of being one
# flat constant both levels would have to share despite drawing from
# different presets. Linearly interpolated/extrapolated from these two
# points and clamped. TUNED, NOT SOURCED for the exact numbers; the
# DIRECTION (deeper surge -> denser wrack) is the physically-motivated part.
#
# THICKENED MODESTLY, 2026-08-31 ("it's already good" -- the review's own
# words, unlike the background/cluster passes above): both reference spacings
# divided by 1.3 (a ~30% density raise, well under the background/cluster
# multiples) -- L2 1.2 m -> 0.92 m, L3 0.7 m -> 0.54 m.
_STRAND_SPACING_REF = ((2.0, 1.2 / 1.3), (2.8, 0.7 / 1.3))
_STRAND_SPACING_MIN_M = 0.4
_STRAND_SPACING_MAX_M = 3.0
# COMPOSITION -- "mostly vegetation/limbs/small timber in tangles", not the
# full raft mix (a wrack line does not carry whole fence panels or torn
# roof sections). `_strand_mix` restricts whatever `kind_weights` the
# caller already resolved (the live `debris_mix`-driven mix, the same one
# `_house_mix` re-skews for a house cluster) to this small/organic subset
# and renormalises -- one source of truth for the vegetation SHARE, not a
# second hardcoded split.
_STRAND_KINDS = ("vegetation", "log", "timber", "sheet", "siding_strip")


def _strand_spacing_m(water_level_m):
    (wl0, s0), (wl1, s1) = _STRAND_SPACING_REF
    wl = float(water_level_m)
    if wl1 == wl0:
        s = s0
    else:
        t = (wl - wl0) / (wl1 - wl0)
        s = s0 + t * (s1 - s0)
    return max(_STRAND_SPACING_MIN_M, min(_STRAND_SPACING_MAX_M, s))


def _strand_mix(weights):
    """`weights` (a `_RAFT_KIND_WEIGHTS`-shaped dict) restricted to
    `_STRAND_KINDS` and renormalised. Falls back to pure vegetation if
    every one of those classes happens to be zero-weighted."""
    w = dict(weights or _RAFT_KIND_WEIGHTS)
    sub = {k: float(w.get(k, 0.0)) for k in _STRAND_KINDS}
    tot = sum(sub.values())
    if tot <= 0.0:
        return {"vegetation": 1.0}
    return {k: v / tot for k, v in sub.items()}


def _strand_line_specs(kn, region, rng, depth_fn, all_fps, kind_weights=None):
    """The wrack line: small tangles of debris in CLUMPS along the shore,
    separated by clear gaps (`_STRAND_CLUMP_LEN_M`/`_STRAND_GAP_LEN_M`), at
    the TRUE waterline (`_STRAND_TRIGGER_M`) rather than
    `raft_min_depth_m`'s later threshold.

    MARCHES THE SHORELINE WITH A PLAIN GRID -- every horizontal/vertical
    edge whose two endpoints straddle the `_STRAND_TRIGGER_M` depth contour
    gets one linearly-interpolated crossing point (the same "grid boundary
    cells x cell size" idiom this fix's offline verification uses to report
    contour length). EVERY crossing is fed through `_in_clump` first (a
    running clump/gap state machine keyed on the distance between
    consecutive crossings, see `_STRAND_CLUMP_JUMP_RESET_M`'s own docstring
    section above) -- only a crossing that lands inside an active CLUMP
    calls `_emit`, so density INSIDE a clump is unchanged from before this
    fix while roughly a third of the shoreline (not all of it) ever gets
    debris at all.

    `_emit` jitters `_draw(h / spacing, rng)` pieces per crossing along the
    local shore-PARALLEL tangent (small jitter -- a tangle, not a smear) and
    across it (the local NORMAL from `surge._grad_deg`, which already
    points toward deeper water, so `-_STRAND_INLAND_MAX_M ..
    +_STRAND_WATER_MAX_M` spans the true edge ASYMMETRICALLY -- see that
    pair's own docstring). Reuses `_append_raft` for the draft/attitude/z of
    every piece (and, where the draw comes up `log`, its stub branches --
    see that function's own docstring): a stranded piece is still built and
    merged as a raft (same schema, same `build_rafts` per-kind mesh) -- it
    is only found at a shallower depth than the background/cluster passes
    above.
    """
    x0, y0, x1, y1 = surge._bounds(region)
    h = _STRAND_GRID_M
    nx = max(2, int(round((x1 - x0) / h)))
    ny = max(2, int(round((y1 - y0) / h)))
    xs = [x0 + (x1 - x0) * i / nx for i in range(nx + 1)]
    ys = [y0 + (y1 - y0) * j / ny for j in range(ny + 1)]
    grid = [[depth_fn(x, y) for x in xs] for y in ys]
    trig = _STRAND_TRIGGER_M

    spacing = _strand_spacing_m(kn["water_level_m"])
    weights = _strand_mix(kind_weights)
    out = []

    def _emit(px, py):
        ang = math.radians(surge._grad_deg(depth_fn, px, py))
        nvx, nvy = math.cos(ang), math.sin(ang)   # toward deeper water
        tvx, tvy = -nvy, nvx                       # shore-parallel tangent
        for _ in range(surge._draw(h / spacing, rng)):
            off_n = rng.uniform(-_STRAND_INLAND_MAX_M, _STRAND_WATER_MAX_M)
            off_t = rng.gauss(0.0, _STRAND_TANGENT_JITTER_M)
            qx = px + nvx * off_n + tvx * off_t
            qy = py + nvy * off_n + tvy * off_t
            if _inside_any_footprint(qx, qy, all_fps):
                continue
            _append_raft(out, qx, qy, rng, kn, weights, exclude_fps=all_fps)

    # CLUMP/GAP STATE MACHINE -- see the "PATCHY, NOT CONTINUOUS" note above
    # `_STRAND_CLUMP_LEN_M`. Starts already partway into a random-length gap
    # (rather than always beginning a fresh clump at the very first crossing
    # found) so the pattern's phase is not pinned to wherever the grid march
    # happens to start.
    _clump = {"mode": "gap",
             "remaining": rng.uniform(0.0, _STRAND_GAP_LEN_M[1]),
             "last": None}

    def _in_clump(px, py):
        last = _clump["last"]
        dist = 0.0 if last is None else math.hypot(px - last[0], py - last[1])
        _clump["last"] = (px, py)
        if dist > _STRAND_CLUMP_JUMP_RESET_M:
            # A disconnected patch of shoreline -- re-roll fresh rather than
            # carrying a stale run length across a gap the eye would never
            # read as continuous with what came before it.
            _clump["mode"] = rng.choice(("clump", "gap"))
            _clump["remaining"] = rng.uniform(*(
                _STRAND_CLUMP_LEN_M if _clump["mode"] == "clump"
                else _STRAND_GAP_LEN_M))
            dist = 0.0
        _clump["remaining"] -= dist
        while _clump["remaining"] <= 0.0:
            _clump["mode"] = ("gap" if _clump["mode"] == "clump"
                              else "clump")
            _clump["remaining"] += rng.uniform(*(
                _STRAND_CLUMP_LEN_M if _clump["mode"] == "clump"
                else _STRAND_GAP_LEN_M))
        return _clump["mode"] == "clump"

    for j in range(ny + 1):
        row = grid[j]
        for i in range(nx):
            a, b = row[i], row[i + 1]
            if (a > trig) != (b > trig):
                t = 0.5 if a == b else (trig - a) / (b - a)
                cx_pt, cy_pt = xs[i] + t * (xs[i + 1] - xs[i]), ys[j]
                if _in_clump(cx_pt, cy_pt):
                    _emit(cx_pt, cy_pt)
    for i in range(nx + 1):
        for j in range(ny):
            a, b = grid[j][i], grid[j + 1][i]
            if (a > trig) != (b > trig):
                t = 0.5 if a == b else (trig - a) / (b - a)
                cx_pt, cy_pt = xs[i], ys[j] + t * (ys[j + 1] - ys[j])
                if _in_clump(cx_pt, cy_pt):
                    _emit(cx_pt, cy_pt)
    return out


# ---------------------------------------------------------------------------
# MID-WATER DRIFT LINES -- ADDED 2026-08-31 (user: "the debris in the
# flooded area needs to increase a lot in number"). Raising
# `raft_bg_per_100m2` alone turns MORE water into confetti; a real flood's
# open water is not confetti at any density -- wind stress on the surface
# sorts loose floating debris into WINDROWS, long, narrow streaks parallel to
# the wind, the same mechanism that lines up foam and scum on any wind-driven
# lake or bay. This is the STRUCTURE half of the density raise: a handful of
# elongated, loose clusters laid down BEFORE the plain background scatter (so
# `_inside_any_footprint` sees them too, but neither pass excludes the
# other's ground) rather than another turn of the per-cell dial.
#
# WIND BEARING, NOT THE DEPTH GRADIENT. Every OTHER directional thing in this
# function (`_cluster`'s house/obstacle standoff) uses `surge._grad_deg`
# because "the upstream face" is defined by where the SURGE came from. A
# drift line is defined by where the WIND is blowing NOW, a different vector
# this module has never needed before -- `land_debris_specs` already takes a
# `wind_bearing_fn(x, y)` plain callable for exactly this reason (the module
# docstring's "THE RAFT FIELD WAS ALL BUILDING MATERIAL": numbers/callables
# cross as plain values, `hurricane` is never imported here). `raft_specs`'s
# new `wind_bearing_fn=` argument is the same convention; `None` (every
# existing caller) authors no drift lines at all.
_DRIFT_LINE_COUNT = (6, 12)          # lines per `raft_specs` call
_DRIFT_LINE_LEN_M = (20.0, 60.0)     # requested band
_DRIFT_LINE_HALFWIDTH_M = 1.6        # across-line jitter -- a narrow streak,
                                       # not a fat smear
_DRIFT_LINE_BEARING_JITTER_DEG = 12.0  # "aligned to the wind bearing +/- a
                                         # little"
_DRIFT_LINE_SPACING_M = 3.0          # along-line station spacing
_DRIFT_LINE_PIECES_PER_STATION = (1, 2)  # a loose knot, not a single plank
# An anchor has to read as OPEN WATER, not the waterline band or a house's
# own collection point -- otherwise "mid-water" drift lines would just be a
# second, redundant way to draw the strand line or the house clusters.
_DRIFT_LINE_MIN_DEPTH_MULT = 2.0
_DRIFT_LINE_HOUSE_CLEAR_M = 15.0
_DRIFT_LINE_MAX_TRIES = 40


def _drift_line_specs(kn, region, rng, depth_fn, all_fps, wind_bearing_fn,
                      kind_weights=None):
    """6-12 elongated, wind-aligned debris streaks in open water (see this
    section's own docstring above). Composition is the OPEN-WATER
    `kind_weights` unchanged -- NOT `_house_mix` -- a mid-lake windrow is not
    "from a house."

    Returns `[]` immediately if `wind_bearing_fn` is `None` -- the default,
    so every existing `raft_specs` caller is unaffected until it threads one
    through (`suburb_hurricane_launch_script.py`'s "# DEBRIS RAFTS" block
    now does, the same `lambda x, y: hu.wind_bearing_at(hcfg, x, y)` the
    "# LAND DEBRIS" block a few lines below it already built).
    """
    if wind_bearing_fn is None:
        return []
    x0, y0, x1, y1 = surge._bounds(region)
    min_depth = float(kn["raft_min_depth_m"])
    deep_gate = (min_depth * _DRIFT_LINE_MIN_DEPTH_MULT
                + float(kn["raft_waterline_band_m"]))
    out = []
    for _ in range(rng.randint(*_DRIFT_LINE_COUNT)):
        anchor = None
        for _try in range(_DRIFT_LINE_MAX_TRIES):
            ax = rng.uniform(x0, x1)
            ay = rng.uniform(y0, y1)
            if depth_fn(ax, ay) < deep_gate:
                continue
            if any((ax - fx) ** 2 + (ay - fy) ** 2
                  < (_DRIFT_LINE_HOUSE_CLEAR_M + 0.5 * ffp) ** 2
                  for fx, fy, ffp in all_fps):
                continue
            anchor = (ax, ay)
            break
        if anchor is None:
            continue    # this plate's open water is too tight -- skip, not
                        # crash; the background scatter still covers it
        ax, ay = anchor
        length = rng.uniform(*_DRIFT_LINE_LEN_M)
        bearing = (float(wind_bearing_fn(ax, ay))
                  + rng.gauss(0.0, _DRIFT_LINE_BEARING_JITTER_DEG))
        th = math.radians(bearing)
        ux, uy = math.cos(th), math.sin(th)     # along the line
        nx_, ny_ = -uy, ux                       # across it
        n_stations = max(2, int(round(length / _DRIFT_LINE_SPACING_M)))
        for i in range(n_stations + 1):
            s = -0.5 * length + length * i / n_stations
            sx = ax + ux * s
            sy = ay + uy * s
            off = rng.gauss(0.0, 0.5 * _DRIFT_LINE_HALFWIDTH_M)
            px, py = sx + nx_ * off, sy + ny_ * off
            if depth_fn(px, py) < min_depth:
                continue
            if _inside_any_footprint(px, py, all_fps):
                continue
            for p_i in range(rng.randint(*_DRIFT_LINE_PIECES_PER_STATION)):
                ppx = px + rng.gauss(0.0, 0.4)
                ppy = py + rng.gauss(0.0, 0.4)
                if _inside_any_footprint(ppx, ppy, all_fps):
                    continue
                _append_raft(out, ppx, ppy, rng, kn, kind_weights,
                             tangle_idx=p_i, exclude_fps=all_fps)
    return out


# ---------------------------------------------------------------------------
# NEAREST-HOUSE SKIN MATCHING, WIDENED -- DEBRIS D6 review, 2026-09-01: "it
# still looks too much like the same green debris everywhere but the houses
# don't have any green so match them to the nearest house's colors." Before
# this pass, `skin` only ever reached a raft through `_cluster`'s own
# per-house loop (see `raft_specs`'s "clustered against standing obstacles"
# section) -- i.e. only the pieces this module ITSELF collected against one
# specific house's footprint wore that house's siding/deck material. Every
# OTHER skinnable piece (open-water background scatter, the strand line, the
# mid-water drift lines, and even a piece piled against a TREE or CAR
# obstacle cluster) stayed on `_RAFT_TINT`'s generic fallback regardless of
# how close it actually sat to a real house -- which is most of the field by
# piece count (the background/strand/drift passes dwarf the house clusters
# at the raised D5/D6 densities).
#
# `_apply_nearest_house_skin`, below, is a POST-PROCESS over a FINISHED spec
# list rather than a change threaded into every placement call above: every
# one of those calls already knows its own (x, y); re-deriving "which house
# is this piece nearest" once, after the fact, over the same `houses` list
# `raft_specs` already receives, is simpler and strictly separable from the
# placement maths, and it costs nothing extra for a caller that has no
# houses or no skins to offer (the loop below is a no-op unless at least one
# house carries one).
_HOUSE_SKIN_MATCH_R_M = 35.0   # TUNED, NOT SOURCED -- "the nearest house's
                                # colours" without a stated radius; 35 m is
                                # roughly 3 house-widths on this suburb's own
                                # 10-20 m stock (`fp_by_style`), wide enough
                                # to cover a lot's own water frontage and the
                                # first few metres of open water past it
                                # without reaching all the way across a
                                # street to a house on the FAR bank.


def _apply_nearest_house_skin(specs, house_pts, rng, max_r_m=_HOUSE_SKIN_MATCH_R_M,
                              skin_pool=None):
    """Give every SKINNABLE piece in `specs` (`kind` in `_RAFT_SKIN_CLASS`)
    that does not already carry one a `"skin"`, mutating `specs` in place and
    also returning it.

    `house_pts` is `(x, y, fp, skin_or_None)` tuples -- `raft_specs`'s own
    `houses` argument, EVERY house on the plate, not just the ones this
    module already clustered against (a piece can be nearest a PRISTINE
    house, which still has a real siding/roof colour even though nothing
    was ever clustered against it). For each unskinned piece, the nearest
    house whose centre-to-piece distance minus that house's own footprint
    radius (`fp * 0.5` -- the same "fp is an effective radius" convention
    `_cluster`'s own `standoff` and `_inside_any_footprint` already use) is
    within `max_r_m` wins; if that house has a skin for this piece's class
    (`_RAFT_SKIN_CLASS[kind]`), the piece wears it.
    Note this is a genuine NEAREST-NEIGHBOUR search, not "the nearest house
    that happens to have a skin" -- a piece 5 m from a pristine house and
    40 m from the nearest wrecked one matches the PRISTINE house (and, if
    that house's own skin is unknown to the caller, falls through to
    `skin_pool` below exactly as if no house were in range at all) rather
    than reaching past it to a farther one just because that one has a
    known colour.

    Beyond `max_r_m` of every house (or when the nearest one has no skin on
    record), `skin_pool` -- an OPTIONAL list of the same `{"siding": <name>,
    "deck": <name>}` dicts, one per distinct STYLE actually present on this
    plate -- is drawn from uniformly per piece via `rng.choice`, so an
    open-water piece far from any specific house still wears a REAL colour
    drawn from THIS neighbourhood's own palette rather than one hardcoded
    generic tint. Every name in that pool traces back to
    `detail.modular_house.PALETTES`, none of which is green, which is the
    actual guarantee this function makes -- not "close to a house" but
    "never an arbitrary flat tint with no real material behind it." `None`
    (the default) leaves an out-of-range, unmatched piece exactly as it was
    before this function existed: unskinned, `_RAFT_TINT`'s own kind colour.

    Pieces that already carry a `"skin"` (the `_cluster` house-loop's own
    doing) are left untouched -- this only fills gaps, it never overrides a
    more specific answer a caller already computed.
    """
    if not specs:
        return specs
    pts = [(float(h[0]), float(h[1]),
           float(h[2]) if len(h) > 2 and h[2] else 10.0,
           h[3] if len(h) > 3 else None)
          for h in (house_pts or ())]
    pool = [p for p in (skin_pool or ()) if p]
    for s in specs:
        if s.get("skin"):
            continue
        cls = _RAFT_SKIN_CLASS.get(s.get("kind"))
        if cls is None:
            continue
        sx, sy = float(s["x"]), float(s["y"])
        best_skin, best_d = None, None
        found_any_in_range = False
        for hx, hy, hfp, hskin in pts:
            d = math.hypot(hx - sx, hy - sy) - 0.5 * hfp
            if d > max_r_m:
                continue
            if best_d is None or d < best_d:
                best_d, best_skin, found_any_in_range = d, hskin, True
        nm = (best_skin or {}).get(cls) if found_any_in_range else None
        if not nm and pool:
            nm = (rng.choice(pool) or {}).get(cls)
        if nm:
            s["skin"] = nm
    return specs


def raft_specs(cfg, region, rng, houses, depth_fn, kind_weights=None,
              obstacles=None, wind_bearing_fn=None, skin_pool=None):
    """Floating debris mats: a light background scatter across open water, a
    boosted waterline BAND a little past `raft_min_depth_m`, a proper WRACK/
    STRAND LINE right at the true waterline (`_strand_line_specs`, ADDED for
    the D2 review -- see that function's own docstring for why it needed a
    dedicated contour march rather than the background grid above), plus
    clusters collected against the upstream (seaward) face of whatever is
    still standing. Pure Python — see `build_rafts` for the geometry.

    `houses` is `(x, y)`, `(x, y, footprint_m)` OR `(x, y, footprint_m,
    skin)` tuples — the same shape `tornado.car_blockers`'s `standing`
    argument takes, plus one OPTIONAL trailing field, so a caller that
    already built the plain 2/3-tuple list for the vehicle pass can hand it
    here unchanged and get every house's cluster generic, exactly as
    before. `skin`, ADDED for the "match them to the houses they are near"
    fix (see `_one_raft`'s own docstring): a `{"siding": <name>, "deck":
    <name>}` dict, or `None`/omitted for a house with no known palette (a
    pristine house nobody wrecked, or a caller that never tracked one) —
    every raft this function clusters against THAT house's footprint
    carries it, so a `wall`/`siding_strip`/`roof_top` piece there wears the
    same material the standing house does. `depth_fn` is
    `surge.depth_at(surge_cfg, region, rng)` (or any `(x, y) -> depth_m`
    callable); this function never imports `surge`'s `cfg` shape, only
    calls what it is given.

    `obstacles`, ADDED 2026-08-31: an OPTIONAL second `(x, y)` /
    `(x, y, footprint_m)` list -- trees, cars, anything else standing in
    the water that is not a house -- clustered the same way houses are
    (see "clusters on the up-flow side of standing houses/trees/cars" in
    the review) but at the smaller `raft_per_house`-sibling rate
    `raft_per_obstacle`, and WITHOUT `_house_mix`'s skew toward building
    material (a tree is not "the source" of siding). `None` (the default)
    authors no obstacle clusters at all -- existing callers are unaffected.

    NEITHER LIST'S OWN FOOTPRINT IS EVER BUILT ON: every piece this
    function places, background, strand line or clustered, is skipped if it
    falls inside ANY house's or obstacle's footprint circle
    (`_inside_any_footprint`) -- a raft collects AGAINST a wall, never
    through it.

    `kind_weights` is `_RAFT_KIND_WEIGHTS`-shaped (`raft_kind_weights(...)`
    builds one from a live vegetation share) and overrides the static
    fallback table for every raft this call authors, background and
    clustered alike. `None` (the default) keeps every existing caller's
    behaviour — the fallback table, unchanged.

    UPSTREAM MEANS SEAWARD. `surge._grad_deg(depth_fn, x, y)` is documented
    there as pointing toward DEEPER water — i.e. where the flow arriving at
    this house came FROM — which is exactly the face debris piles against.
    Reusing it here means a raft cluster automatically follows the plate's
    own ragged, noise-perturbed shoreline gradient rather than one fixed
    global bearing, the identical trick `surge.wrack_specs` already plays.

    `wind_bearing_fn`, ADDED 2026-08-31: an OPTIONAL `(x, y) -> degrees`
    callable, the same convention `land_debris_specs` already takes --
    authors 6-12 MID-WATER DRIFT LINES (`_drift_line_specs`, see that
    section's own docstring) so the open-water density raise reads as
    wind-organised windrows rather than uniform confetti. `None` (the
    default) authors no drift lines at all -- every existing caller is
    unaffected.

    `skin_pool`, ADDED (DEBRIS D6 review, 2026-09-01) alongside the widened
    nearest-house matching below: an OPTIONAL list of `{"siding": <name>,
    "deck": <name>}` dicts, one per distinct house STYLE actually present on
    this plate -- see `_apply_nearest_house_skin`'s own docstring. `None`
    leaves an out-of-range piece unskinned, exactly today's behaviour.

    NEAREST-HOUSE SKIN MATCHING IS NO LONGER LIMITED TO THIS FUNCTION'S OWN
    HOUSE CLUSTERS. Before returning, every skinnable piece this call placed
    -- background scatter, waterline band, strand line, drift lines and
    obstacle (tree/car) clusters alike, not just the per-house clusters
    right below -- is run through `_apply_nearest_house_skin` against the
    FULL `houses` list (every house, not only the ones with a known skin) at
    `_HOUSE_SKIN_MATCH_R_M` (35 m); within range it takes the nearest
    house's own siding/deck material, and beyond range it draws from
    `skin_pool` if one was given. See that function's own docstring for the
    full contract.
    """
    kn = resolve_cfg(cfg)
    rng = surge._as_random(rng)
    x0, y0, x1, y1 = surge._bounds(region)
    out = []

    house_fps = _footprint_list(houses, 10.0)
    obstacle_fps = _footprint_list(obstacles, 3.0)
    all_fps = house_fps + obstacle_fps
    min_depth = float(kn["raft_min_depth_m"])
    # HOUSE-MATCHED SKIN, per house, SAME ORDER as `house_fps`
    # (`_footprint_list` preserves order and drops nothing) -- a plain
    # optional 4th tuple element, `(x, y, footprint_m, skin)`, so every
    # EXISTING caller passing 2- or 3-tuples is unaffected and gets `None`
    # (generic) for every house, exactly today's behaviour. `skin` is a
    # `_RAFT_SKIN_CLASS`-shaped dict (`mh.palette_skins(...)`'s own return
    # shape) — see `_one_raft`'s own docstring for why this module accepts
    # that dict as a plain value instead of importing the module that builds
    # it. Obstacles (trees, cars) never carry one — see `_cluster`'s call
    # below.
    house_skins = [(e[3] if len(e) > 3 else None) for e in (houses or ())]

    # -- background + STRAND LINE: open water is not an empty sheet, and
    # the waterline is where flood debris actually collects ---------------
    cell = max(8.0, float(kn["raft_cell_m"]))
    nx = max(1, int(round((x1 - x0) / cell)))
    ny = max(1, int(round((y1 - y0) / cell)))
    dx, dy = (x1 - x0) / nx, (y1 - y0) / ny
    area_100 = dx * dy / 100.0
    bg_rate = float(kn["raft_bg_per_100m2"])
    band = float(kn["raft_waterline_band_m"])
    boost = float(kn["raft_waterline_boost"])
    for iy in range(ny):
        for ix in range(nx):
            ax, ay = x0 + ix * dx, y0 + iy * dy
            cx, cy = ax + dx * 0.5, ay + dy * 0.5
            d_here = depth_fn(cx, cy)
            if d_here < min_depth:
                continue
            rate = bg_rate * boost if d_here <= min_depth + band else bg_rate
            for _ in range(surge._draw(rate * area_100, rng)):
                px = ax + rng.random() * dx
                py = ay + rng.random() * dy
                if _inside_any_footprint(px, py, all_fps):
                    continue
                _append_raft(out, px, py, rng, kn, kind_weights,
                            exclude_fps=all_fps)

    # -- clustered against standing obstacles ------------------------------
    saturate = float(kn["raft_saturate_depth_m"])
    per_house = float(kn["raft_per_house"])
    per_obstacle = float(kn["raft_per_obstacle"])
    lo, hi = kn["raft_standoff_m"]
    jitter = float(kn["raft_bearing_jitter_deg"])

    def _cluster(cx0, cy0, fp, rate, weights, skin=None):
        d = depth_fn(cx0, cy0)
        if d < min_depth:
            return
        bearing = surge._grad_deg(depth_fn, cx0, cy0)
        for _ in range(surge._draw(rate * min(1.0, d / saturate), rng)):
            standoff = 0.5 * fp + rng.uniform(lo, hi)
            ang = math.radians(bearing + rng.gauss(0.0, jitter))
            px = cx0 + math.cos(ang) * standoff
            py = cy0 + math.sin(ang) * standoff
            # one TANGLE, not one board -- see `_RAFT_TANGLE`. Later
            # members (`tangle_idx > 0`) overlap/lean on the earlier ones.
            for _p in range(rng.randint(*_RAFT_TANGLE)):
                ppx = px + rng.gauss(0.0, _RAFT_TANGLE_R_M)
                ppy = py + rng.gauss(0.0, _RAFT_TANGLE_R_M)
                if _inside_any_footprint(ppx, ppy, all_fps):
                    continue
                _append_raft(out, ppx, ppy, rng, kn, weights,
                            tangle_idx=_p, skin=skin, exclude_fps=all_fps)

    for (hx, hy, fp), hskin in zip(house_fps, house_skins):
        _cluster(hx, hy, fp, per_house, _house_mix(kind_weights), skin=hskin)
    for ox, oy, ofp in obstacle_fps:
        # NOT `_house_mix`, and NO SKIN -- a tree or a car is not "the
        # source" of the siding/panel/fence classes the way a wrecked house
        # is, so its catch keeps whatever composition the caller already
        # chose and stays generic (`_RAFT_SKIN_CLASS`'s own docstring:
        # "open water and a tree/car catch are not 'from a house'").
        _cluster(ox, oy, ofp, per_obstacle, kind_weights)

    # -- the wrack/strand line, at the TRUE waterline ----------------------
    out.extend(_strand_line_specs(kn, region, rng, depth_fn, all_fps,
                                  kind_weights))

    # -- MID-WATER DRIFT LINES, at the true open-water composition ---------
    out.extend(_drift_line_specs(kn, region, rng, depth_fn, all_fps,
                                 wind_bearing_fn, kind_weights))

    # -- NEAREST-HOUSE SKIN, WIDENED to the whole field -- see this
    # function's own docstring and `_apply_nearest_house_skin`'s. Runs over
    # EVERY house (`houses`, not `house_fps`/`house_skins`'s zipped subset --
    # this needs the ORIGINAL per-house records so a piece can match a
    # PRISTINE house too), after every placement pass above so nothing this
    # call authored is missed.
    _apply_nearest_house_skin(out, houses, rng, skin_pool=skin_pool)

    # -- SIZE-MIX CAP -- see that function's own docstring -----------------
    out = _cap_large_pieces(out, house_fps, float(kn["water_level_m"]))
    return out


def summarise(cfg, region, rng, houses=(), depth_fn=None, kind_weights=None,
             obstacles=None, wind_bearing_fn=None):
    """Cheap numbers for a bench, the `surge.summarise`/`tornado.summarise`
    idiom: resolved raft knobs always, plus raft/wet-house counts and a
    per-kind raft tally if a `depth_fn` is supplied. `kind_weights`,
    `obstacles` and `wind_bearing_fn` pass straight through to `raft_specs`
    — see `raft_kind_weights` and `raft_specs`'s own `obstacles`/
    `wind_bearing_fn` docstrings."""
    kn = resolve_cfg(cfg)
    houses = list(houses or ())
    out = {
        "water_level_m": round(float(kn["water_level_m"]), 3),
        "raft_cell_m": round(float(kn["raft_cell_m"]), 2),
        "houses_total": len(houses),
    }
    if depth_fn is not None:
        rafts = raft_specs(kn, region, rng, houses, depth_fn, kind_weights,
                           obstacles=obstacles,
                           wind_bearing_fn=wind_bearing_fn)
        out["rafts"] = len(rafts)
        by_kind = {}
        for r in rafts:
            by_kind[r["kind"]] = by_kind.get(r["kind"], 0) + 1
        out["rafts_by_kind"] = by_kind
        out["houses_wet"] = sum(
            1 for h in houses if depth_fn(float(h[0]), float(h[1])) > 1e-6)
    return out


# ---------------------------------------------------------------------------
# FLOATING FALLEN TREES -- ADDED (DEBRIS D5 review, second half of "I don't
# really see floating tree trunks/fallen down trees in the flood. Add
# that."). `_RAFT_DIMS["log"]`'s authored box already covers the GENERIC
# bole -- a plank-schema cylinder-as-box, at whatever count `raft_per_house`/
# `raft_bg_per_100m2` put in the field -- but a real hurricane also carries
# off WHOLE UPROOTED TREES, crown and branches still attached, and the
# launcher's own tree pass ALREADY HAS the exact archetype for that:
# `tree_<species>_fallen.usd` (baked in metres, lying with crown on the
# ground, base at origin -- `ARCH_DIR`'s own tree library, the SAME files
# the land windthrow pass already references for a tree that fell on DRY
# ground). This is that same asset, referenced a SECOND time, in OPEN
# water, with a Z translate that seats its TRUNK at the waterline instead
# of on grade.
#
# A REFERENCED ARCHETYPE, NOT AN AUTHORED BOX, because the crown and branch
# structure is the entire point of "I don't see fallen trees" -- a bigger
# `log` box would still just be a bigger box. `planks._box`/`_one_raft`
# have no way to approximate a canopy and no reason to try when a real
# asset already exists.
# ---------------------------------------------------------------------------

# TRUNK GEOMETRY, MEASURED OFFLINE (bare `pxr`, no Kit -- the `Nucleus USD
# without Kit` idiom, applied to a LOCAL archetype file instead of an
# `omniverse://` one) against every `tree_*_fallen.usd` archetype shipped in
# `scene_gen/assets/archetypes_hurricane/`: `(trunk_diameter_m,
# trunk_z_center_m)`.
#
# `trunk_diameter_m` is NOT the trunk mesh's own overall bounding-box
# diameter -- that box is contaminated by root/branch flare wherever the
# main limbs fork off it (measured mid-length z-span 4.1-5.4 m on every one
# of these archetypes, clearly not a single round bole). It is the Z-EXTENT
# of the trunk mesh's own points restricted to the 1 m band nearest `y=0`
# ("base at origin" — the reviewer's own language, confirmed by measurement:
# every archetype's trunk mesh spans from Y~=0 down to a large negative Y,
# i.e. the tree lies along -Y from its base), the same "walk the mesh's own
# points, never `UsdGeom.BBoxCache` on a compound prim" discipline
# `measure_fence` already uses for exactly this reason (a bbox-of-bbox
# inflates on anything that is not axis-aligned and un-branched).
# `trunk_z_center_m` is that same band's mean Z -- the trunk axis's own
# local-frame height, needed because these archetypes are NOT authored
# resting with their trunk centred on local Z=0 the way a simple placement
# formula would assume (measured 0.46 m high on `American_Beech`, ~0 on the
# other three). `rotateZ` (this module's own `yaw`, and the launcher's)
# does not move Z, so this centre is unaffected by whichever yaw a given
# instance draws.
#
# The trunk mesh itself is found by name (`trunk`, or `*_bark_Mat` — the
# convention `Apple_bark_Mat` uses where no prim is literally named
# "trunk") — see `tools/hurricane_debris_plot.py` for the measurement
# script this table is transcribed from.
_FLOAT_TREE_TRUNK_M = {
    "tree_American_Beech_fallen":    (1.21, 0.46),
    "tree_Common_Apple_fallen":      (1.78, 0.01),
    "tree_Largetooth_Aspen_fallen":  (0.42, 0.03),
    "tree_Shumard_Oak_fallen":       (0.73, -0.04),
}
# A DEFAULT for any archetype name this table does not carry (a future
# species added to the tree bake without a matching offline measurement) --
# a middling diameter and a centred trunk, rather than refusing the
# instance outright. TUNED, NOT SOURCED.
_FLOAT_TREE_TRUNK_DEFAULT_M = (0.6, 0.0)

_FLOAT_TREE_COUNT = (15, 30)
# OPEN WATER, GENUINELY -- "depth > 0.8 m" in the review's own words, well
# past `raft_min_depth_m`'s "a board floats here" bar: a whole tree needs
# real depth under it, not a few centimetres of sheet flow.
_FLOAT_TREE_MIN_DEPTH_M = 0.8
# "at least 12 m from any house footprint" -- clear of a standing building,
# so a floating trunk never overlaps or reads as docked against one; that
# read belongs to `log`'s own house-cluster catch (`raft_per_house`)
# instead, which is a much smaller box and is DESIGNED to pile against a
# wall.
_FLOAT_TREE_HOUSE_CLEAR_M = 12.0
_FLOAT_TREE_DRAFT_LO, _FLOAT_TREE_DRAFT_HI = 0.40, 0.60
_FLOAT_TREE_MAX_TRIES = 60


def floating_tree_specs(kn, region, rng, depth_fn, houses=None,
                        archetypes=None, count=None):
    """15-30 (`count`, or `_FLOAT_TREE_COUNT`) whole fallen-tree archetype
    placements in OPEN water -- `depth_fn(x, y) > _FLOAT_TREE_MIN_DEPTH_M`
    and at least `_FLOAT_TREE_HOUSE_CLEAR_M` clear of any house footprint
    (`houses`, the SAME `(x, y)` / `(x, y, footprint_m)` shape `raft_specs`
    takes — normalised here the same way, via `_footprint_list`).

    HOUSES ONLY, DELIBERATELY -- NOT `raft_specs`'s `obstacles` (trees,
    cars). The review's own clearance rule is "at least 12 m from any HOUSE
    footprint"; a suburban plate's TREE population is dense enough (this
    module's own bake target runs one tree roughly every 10-15 m of plate)
    that reusing `raft_specs`'s `obstacles=` convention here and feeding it
    every tree on the plate was tried and MEASURED to fail outright: on the
    real 500 m L3 GT layout (94 houses, 1,684 trees), passing the full tree
    list as a second exclusion population left almost no point on the
    entire plate more than 12 m from its nearest tree, and the function
    placed 2 of the requested 15-30 trees. Houses-only, on the identical
    field, places 27. A floating trunk overlapping a tree canopy is not the
    defect this function exists to avoid; overlapping a HOUSE is.

    Returns `[{"archetype": <name>, "x", "y", "z", "yaw", "draft"}, ...]`.
    `archetype` is a KEY into `_FLOAT_TREE_TRUNK_M` — the launcher's own
    `arch` dict is keyed identically (`os.path.splitext(f)[0]` over
    `ARCH_DIR`'s filenames), so `arch.get(spec["archetype"])` is the launch
    script's whole lookup. `x`/`y` are world metres; `z` is the WORLD METRE
    Z the launcher's reference loop should translate the archetype to (on
    top of its own local origin — the tree-pass `_ref` idiom takes an
    always-zero Z, so this needs its own small reference loop rather than a
    call to that shared helper), computed so the TRUNK — not the whole
    prim's bounding box — rides at the waterline: a random draft fraction
    in `[_FLOAT_TREE_DRAFT_LO, _FLOAT_TREE_DRAFT_HI]` of the archetype's own
    MEASURED trunk diameter sits below `kn["water_level_m"]`, the same
    physical shape `_one_raft`'s own `log` draft formula uses
    (`z = water_level + half_extent * (1 - 2 * draft_frac)`), applied to the
    trunk's MEASURED local Z-centre instead of a box's own half-extent —
    see `_FLOAT_TREE_TRUNK_M`'s own docstring for how that diameter and
    centre were measured, offline, with bare `pxr`. `yaw` is a full random
    `rotateZ`, same convention every other yaw in this module uses.

    `archetypes`, OPTIONAL: an iterable of archetype names to draw from —
    omit for every species this module has trunk geometry for
    (`_FLOAT_TREE_TRUNK_M`'s own keys). A caller whose archetype library is
    missing one of these (a smaller catalogue variant) should pass only the
    names it actually has; this function never touches the filesystem or
    `pxr` to check — PURE PYTHON, matching every other `*_specs` function in
    this module.

    Returns `[]` (skips a placement, never crashes) wherever
    `_FLOAT_TREE_MAX_TRIES` draws in a row all land too shallow or too close
    to a house — a small or mostly-dry plate simply gets fewer floating
    trees than requested rather than an infinite loop or an exception.
    """
    x0, y0, x1, y1 = surge._bounds(region)
    rng = surge._as_random(rng)
    names = list(archetypes) if archetypes else list(_FLOAT_TREE_TRUNK_M)
    if not names:
        return []
    all_fps = _footprint_list(houses, 10.0)
    min_depth = float(_FLOAT_TREE_MIN_DEPTH_M)
    clear_m = float(_FLOAT_TREE_HOUSE_CLEAR_M)

    n = rng.randint(*(count or _FLOAT_TREE_COUNT))
    out = []
    for _ in range(n):
        pos = None
        for _try in range(_FLOAT_TREE_MAX_TRIES):
            px = rng.uniform(x0, x1)
            py = rng.uniform(y0, y1)
            if depth_fn(px, py) < min_depth:
                continue
            if any((px - fx) ** 2 + (py - fy) ** 2
                  < (clear_m + 0.5 * ffp) ** 2 for fx, fy, ffp in all_fps):
                continue
            pos = (px, py)
            break
        if pos is None:
            continue    # this plate's open water is too tight/shallow --
                        # skip, not crash
        px, py = pos
        name = rng.choice(names)
        diam, z_center_local = _FLOAT_TREE_TRUNK_M.get(
            name, _FLOAT_TREE_TRUNK_DEFAULT_M)
        draft = rng.uniform(_FLOAT_TREE_DRAFT_LO, _FLOAT_TREE_DRAFT_HI)
        z_world_trunk = float(kn["water_level_m"]) + diam * (0.5 - draft)
        out.append({"archetype": name, "x": px, "y": py,
                    "z": z_world_trunk - z_center_local,
                    "yaw": rng.uniform(0.0, 360.0), "draft": round(draft, 3)})
    return out


# ---------------------------------------------------------------------------
# stage-touching half — `pxr` imported ONLY inside these and the private
# helpers reachable only from them.
# ---------------------------------------------------------------------------

# Fallback footprint (w, d) metres when a prim's own bound cannot be
# measured (empty geometry). TUNED, NOT SOURCED — roughly the shipped
# cottage/villa footprint range (`.agents/skills/build-hurricane-scenes`'s
# prim-count table gives per-house prim counts, not footprints, and this
# module does not depend on `detail.modular_house` to look one up).
_DEFAULT_FOOTPRINT_M = (8.0, 10.0)

# A kit house is 6-30 m across. A footprint measured outside this band is a
# units or reference-frame bug upstream, never a real building — and the
# failure it produces is silent and large: every derived piece of geometry
# (the exposed slab, the yard patch, the mud ring) is authored at that size,
# which on the level-3 plate rendered as one flat white square ~40 m on a
# side sitting in the flood. Refuse it and say so.
_PLAUSIBLE_FOOTPRINT_M = (3.0, 40.0)

_MUD_BAND_H_M = 0.16     # vertical extent of the "wet" tide-mark ribbon
_MUD_STANDOFF_M = 0.05   # how far it stands proud of the wall face
_SCOUR_MARGIN_M = 3.5    # how far past the walls the "scoured" bare-mud
                          # yard patch extends -- roughly a domestic yard's
                          # depth of exposed ground
_SLAB_HEIGHT_M = 0.12    # typical slab-on-grade foundation reveal height
                          # above grade, for the "shifted"/"swept" bare slab


def _measure_house(prim):
    """`(w, d, x, y, z, yaw_deg)` for `prim` — footprint width/depth (in the
    prim's OWN local X/Y, so this is right regardless of world yaw) and its
    current placement, read without altering the prim.

    A measured footprint outside `_PLAUSIBLE_FOOTPRINT_M` is REFUSED and
    replaced by a nominal one, loudly. A house is 6-30 m across; anything
    outside that is a units or frame bug upstream, and authoring a slab at
    that size puts a flat white plate in the middle of the scene rather than
    failing where it can be seen.

    Same MEASURE, DO NOT GUESS discipline `tornado.toss_prim` uses for
    seating, including the fix its docstring records:
    `ComputeUntransformedBound` is contaminated by the prim's own `rotateZ`
    unless the op order is cleared before the query. Restored byte-for-byte
    afterward — unlike `toss_prim`, this function has no reason to change
    the prim at all.
    """
    from pxr import Gf, Usd, UsdGeom

    # DESCEND PAST A BARE WRAPPER FIRST.
    #
    # The assembly gives every house a wrapper Xform with NO xform ops of its
    # own (`inst/h_{i}`) and puts the referenced archetype one level down
    # (`inst/h_{i}/model`), so that this module has an untouched transform to
    # move — USD refuses a duplicate xform op, which is what forced the split.
    #
    # But it breaks the measurement below if taken literally: the wrapper's
    # UNTRANSFORMED bound contains the child's own world translate, so a
    # house sitting at x = -193 measures as ~200 m across, and every derived
    # piece of geometry — the exposed slab, the yard patch, the mud ring — is
    # authored at that size. On the level-3 plate that rendered as a single
    # flat white square about 40 m on a side sitting in the flood.
    #
    # So: if this prim carries no xform ops and has exactly one child, the
    # child is the thing to measure. The caller still gets a pose relative to
    # the wrapper, which is what it will move.
    _xf0 = UsdGeom.Xformable(prim)
    if not _xf0.GetOrderedXformOps():
        _kids = [k for k in prim.GetChildren() if UsdGeom.Xformable(k)]
        if len(_kids) == 1:
            prim = _kids[0]

    xf = UsdGeom.Xformable(prim)
    vals = {}
    for op in xf.GetOrderedXformOps():
        vals[op.GetOpName().split(":")[-1]] = op.Get()
    t = vals.get("translate") or Gf.Vec3d(0.0, 0.0, 0.0)
    yaw = float(vals.get("rotateZ") or 0.0)
    sc = vals.get("scale")

    xf.SetXformOpOrder([])
    try:
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        r = bc.ComputeUntransformedBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            w, d = _DEFAULT_FOOTPRINT_M
        else:
            mn, mx = r.GetMin(), r.GetMax()
            w, d = float(mx[0] - mn[0]), float(mx[1] - mn[1])
    finally:
        xf.AddTranslateOp().Set(t)
        if vals.get("rotateZ") is not None:
            xf.AddRotateZOp().Set(yaw)
        if sc is not None:
            xf.AddScaleOp().Set(sc)
    lo, hi = _PLAUSIBLE_FOOTPRINT_M
    if not (lo <= w <= hi and lo <= d <= hi):
        print("[washaway] implausible footprint {0:.1f} x {1:.1f} m on {2} — "
              "using a nominal 12 x 9 m instead (units or frame bug "
              "upstream)".format(w, d, prim.GetPath()))
        w, d = 12.0, 9.0
    return w, d, float(t[0]), float(t[1]), float(t[2]), yaw


# Fallback/plausibility bounds for `_measure_car`, an order of magnitude
# smaller than the house pair above (`_DEFAULT_FOOTPRINT_M`/
# `_PLAUSIBLE_FOOTPRINT_M`) — reusing those against a car would accept a
# real bounding-box bug as "plausible" (a car well under 3 m is a units
# bug, not a small vehicle) and reject a perfectly ordinary one as
# "implausible". The suburban pool runs 4.40-4.84 m for an ordinary car up
# to a 9.5 m transit bus (`tornado.CAR_REF_LEN_M`'s own docstring), so the
# plausible band is generous on the long end on purpose.
_DEFAULT_CAR_FOOTPRINT_M = (4.6, 1.85)
_PLAUSIBLE_CAR_FOOTPRINT_M = (2.5, 12.0)


def _measure_car(prim):
    """`(l, w, x, y, z, yaw_deg)` for a car prim — length/width in the
    prim's own local X/Y (longer axis first) and its current placement,
    read without altering the prim.

    A CAR-SIZED TWIN OF `_measure_house`, not a parameterised call to it:
    same wrapper-descent (a car may sit under a bare wrapper Xform the same
    way a house does) and the same rotateZ-safe `ComputeUntransformedBound`
    restore (see that function's docstring for the bug this works around),
    but its own fallback and plausibility bounds — this module already
    prefers a local twin over a shared knob for cheap, call-site-specific
    geometry (`_merge_boxes`'s own docstring makes the identical call
    against `planks.build`).

    UNLIKE `_measure_house`, A CALLER IS EXPECTED TO CALL THIS DIRECTLY,
    not just rely on it as an internal step of `apply_car_washaway`. The
    house ladder's `vuln` comes from an independent draw
    (`hurricane.draw_vulnerability`) that exists before any house prim
    does; the car ladder's `length_m` has no such independent source
    unless the caller already tracked one from the asset pool
    (`suburb_scene.car_dims`, as the tornado launch script does for
    `tornado.car_pose`) — and `car_surge_state` needs a length BEFORE
    `apply_car_washaway` ever runs, since the state decides whether it
    runs at all. A caller with no pool-side dimensions in hand measures
    the already-placed car prim with this function first. Crossing a
    leading-underscore boundary this way is not unusual in this module —
    it already does the identical thing to `surge._as_random`/
    `surge._grad_deg`/`tornado._ladder` at module scope.
    """
    from pxr import Gf, Usd, UsdGeom

    _xf0 = UsdGeom.Xformable(prim)
    if not _xf0.GetOrderedXformOps():
        _kids = [k for k in prim.GetChildren() if UsdGeom.Xformable(k)]
        if len(_kids) == 1:
            prim = _kids[0]

    xf = UsdGeom.Xformable(prim)
    vals = {}
    for op in xf.GetOrderedXformOps():
        vals[op.GetOpName().split(":")[-1]] = op.Get()
    t = vals.get("translate") or Gf.Vec3d(0.0, 0.0, 0.0)
    yaw = float(vals.get("rotateZ") or 0.0)
    sc = vals.get("scale")

    xf.SetXformOpOrder([])
    try:
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        r = bc.ComputeUntransformedBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            l, w = _DEFAULT_CAR_FOOTPRINT_M
        else:
            mn, mx = r.GetMin(), r.GetMax()
            l, w = float(mx[0] - mn[0]), float(mx[1] - mn[1])
            if l < w:
                l, w = w, l
    finally:
        xf.AddTranslateOp().Set(t)
        if vals.get("rotateZ") is not None:
            xf.AddRotateZOp().Set(yaw)
        if sc is not None:
            xf.AddScaleOp().Set(sc)
    lo, hi = _PLAUSIBLE_CAR_FOOTPRINT_M
    if not (lo <= l <= hi):
        print("[washaway] implausible car length {0:.2f} m on {1} — using a "
              "nominal {2:.2f} m instead (units or frame bug upstream)"
              .format(l, prim.GetPath(), _DEFAULT_CAR_FOOTPRINT_M[0]))
        l, w = _DEFAULT_CAR_FOOTPRINT_M
    return l, w, float(t[0]), float(t[1]), float(t[2]), yaw


def _merge_boxes(stage, path, specs, material, ssf=1.0, verbose=False):
    """One merged `UsdGeom.Mesh` from `planks._box`-shaped specs. A local
    twin of `planks.build`'s per-group loop rather than a call to it,
    because this module authors at most a handful of boxes per call site (a
    slab, a mud ring) and never needs `planks.build`'s per-class/per-skin
    grouping — but it uses `planks._box`/`planks._FACES` for the geometry
    itself, the exact routine the tornado debris field already uses."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    if not specs:
        return None
    ssf = float(ssf)
    pts, counts, idx, nrm = [], [], [], []
    for s in specs:
        p, n = planks._box(s)
        base = len(pts)
        pts.extend(Gf.Vec3f(float(q[0]) * ssf, float(q[1]) * ssf,
                            float(q[2]) * ssf) for q in p)
        for fi, face in enumerate(planks._FACES):
            counts.append(4)
            idx.extend(base + v for v in face)
            nrm.extend([Gf.Vec3f(*(float(c) for c in n[fi]))] * 4)

    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    m.CreateNormalsAttr(Vt.Vec3fArray(nrm))
    m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    m.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                        Gf.Vec3f(max(xs), max(ys), max(zs))])
    if material is not None:
        # APPLY the schema before binding -- `scour_relief.py`'s own reason:
        # the bare `MaterialBindingAPI(prim).Bind()` form warns on every
        # later stage read.
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material)
    if verbose:
        print("[washaway] {0:<28s} {1:3d} box(es) -> 1 mesh, {2} point(s)"
              .format(path, len(specs), len(pts)))
    return m.GetPrim().GetPath().pathString


def _mud_ring_specs(cx, cy, w, d, yaw_deg, z, band_h, standoff):
    """Four thin proud ribbons framing a `w` x `d` footprint, centred on
    `z` — GEOMETRY, not a texture rebind, per the hurricane skill's
    "L3 is geometry" rule (repainting a per-building albedo would opt that
    building out of instancing). Lengths are padded by `standoff` so
    adjacent ribbons overlap slightly at the corners rather than leaving a
    visible gap."""
    th = math.radians(yaw_deg)
    c, s = math.cos(th), math.sin(th)

    def to_world(lx, ly):
        return cx + lx * c - ly * s, cy + lx * s + ly * c

    half_w, half_d = 0.5 * w, 0.5 * d
    off = 0.5 * standoff
    specs = []
    for ly in (half_d + off, -(half_d + off)):
        wx, wy = to_world(0.0, ly)
        specs.append({"x": wx, "y": wy, "z": z, "l": w + standoff,
                      "w": standoff, "t": band_h, "yaw": yaw_deg,
                      "pitch": 0.0, "roll": 0.0})
    for lx in (half_w + off, -(half_w + off)):
        wx, wy = to_world(lx, 0.0)
        specs.append({"x": wx, "y": wy, "z": z, "l": d + standoff,
                      "w": standoff, "t": band_h, "yaw": yaw_deg + 90.0,
                      "pitch": 0.0, "roll": 0.0})
    return specs


def _slab_material(stage, path):
    """A bare concrete-slab OmniPBR — no texture, since a raw kit slab
    material would need a UV-mapped asset this module does not have, and a
    flat grey plane reads fine at the 30-120 m nadir range the hurricane
    water plan already budgets for ("spend the budget on albedo, not
    detail", transposed from wet asphalt to bare foundation). TUNED, NOT
    SOURCED — an ordinary exposed-aggregate slab colour."""
    return planks.skin_material(stage, path, None, tint=(0.62, 0.60, 0.56),
                                tile_m=1.0, roughness=0.92)


def apply_washaway(stage, house_prim, level, spec, ssf=1.0):
    """Author surge damage on one already-placed house.

    `house_prim` may be a `Usd.Prim` or a path string — same contract as
    `hurricane_flow.wreck_building`. `level` is one of `LEVELS`. `spec` is
    whatever the matching pure-Python function produced:

        "wet"                `surge.house_water_state(...)`'s dict (only
                              `mudline_z` is read; the ring is skipped if
                              absent — a misaligned ring is worse than none)
        "scoured", "swept"   ignored (footprint is MEASURED, not passed)
        "shifted"             `shift_spec(...)`'s dict
        "collapsed"           `collapse_spec(...)`'s dict

    Footprint geometry (`w`, `d`, current `x, y, z, yaw`) is MEASURED off
    the prim's own bound (`_measure_house`) rather than required from the
    caller — the same "measure, don't guess" discipline `tornado.toss_prim`
    uses for seating, applied here so this function needs nothing about the
    house beyond the prim itself.

    Returns the list of new prim paths authored (empty for `"collapsed"`,
    which touches only the existing house prim).
    """
    from pxr import Sdf, UsdGeom

    prim = (stage.GetPrimAtPath(house_prim) if isinstance(house_prim, str)
            else house_prim)
    if prim is None or not prim.IsValid():
        raise ValueError("no valid house prim at {0!r}".format(house_prim))
    if level not in LEVELS:
        raise ValueError("unknown washaway level {0!r}; expected one of {1}"
                          .format(level, ", ".join(LEVELS)))

    path = prim.GetPath().pathString
    w, d, x, y, z, yaw = _measure_house(prim)
    scope = "{0}/Washaway".format(path)
    UsdGeom.Scope.Define(stage, Sdf.Path(scope))

    made = []
    if level == "wet":
        sp = spec or {}
        mudline_z = sp.get("mudline_z")
        if mudline_z is not None:
            ring = _mud_ring_specs(x, y, w, d, yaw, float(mudline_z),
                                   _MUD_BAND_H_M, _MUD_STANDOFF_M)
            mat = planks.skin_material(stage, scope + "/MudLook",
                                       surge.SILT_TEXTURE,
                                       tint=(0.55, 0.48, 0.38))
            p = _merge_boxes(stage, scope + "/MudRing", ring, mat, ssf)
            if p:
                made.append(p)

    elif level == "scoured":
        margin = _SCOUR_MARGIN_M
        yard = [{"x": x, "y": y, "z": surge._SURFACE_Z_M,
                "l": w + 2.0 * margin, "w": d + 2.0 * margin, "t": 0.02,
                "yaw": yaw, "pitch": 0.0, "roll": 0.0}]
        mat = planks.skin_material(stage, scope + "/YardLook",
                                   surge.SILT_TEXTURE, tint=(0.60, 0.52, 0.40))
        p = _merge_boxes(stage, scope + "/YardPatch", yard, mat, ssf)
        if p:
            made.append(p)

    elif level == "shifted":
        sp = spec or {}
        tornado.toss_prim(stage, path, float(sp.get("dx", 0.0)),
                          float(sp.get("dy", 0.0)), float(sp.get("roll", 0.0)),
                          float(sp.get("dyaw", 0.0)),
                          pitch_deg=float(sp.get("pitch", 0.0)))
        slab_z = surge._SURFACE_Z_M + 0.5 * _SLAB_HEIGHT_M
        slab = [{"x": x, "y": y, "z": slab_z, "l": w, "w": d,
                "t": _SLAB_HEIGHT_M, "yaw": yaw, "pitch": 0.0, "roll": 0.0}]
        p = _merge_boxes(stage, scope + "/Slab",
                         slab, _slab_material(stage, scope + "/SlabLook"), ssf)
        if p:
            made.append(p)

    elif level == "collapsed":
        # No bare slab: unlike `shifted`/`swept`, the house did not travel
        # far enough to leave its footprint clean (`collapse_spec`'s own
        # docstring) -- a slab would sit hidden under the heap for nothing.
        sp = spec or {}
        tornado.toss_prim(stage, path, float(sp.get("dx", 0.0)),
                          float(sp.get("dy", 0.0)), float(sp.get("roll", 0.0)),
                          float(sp.get("dyaw", 0.0)),
                          pitch_deg=float(sp.get("pitch", 0.0)))

    elif level == "swept":
        slab_z = surge._SURFACE_Z_M + 0.5 * _SLAB_HEIGHT_M
        slab = [{"x": x, "y": y, "z": slab_z, "l": w, "w": d,
                "t": _SLAB_HEIGHT_M, "yaw": yaw, "pitch": 0.0, "roll": 0.0}]
        p = _merge_boxes(stage, scope + "/Slab",
                         slab, _slab_material(stage, scope + "/SlabLook"), ssf)
        if p:
            made.append(p)
        prim.SetActive(False)

    attr = prim.GetAttribute("waterState:level")
    if not attr:
        attr = prim.CreateAttribute("waterState:level",
                                    Sdf.ValueTypeNames.Token, custom=True)
    attr.Set(level)
    return made


_CAR_LEVELS = ("swamped", "floated", "piled")


def apply_car_washaway(stage, car_prim, level, spec, ssf=1.0):
    """Author surge damage on one already-placed car — the vehicle
    counterpart of `apply_washaway`. Ranked #1 in
    `_plans/hurricane_coverage_audit.md`: see the module docstring's "THE
    VEHICLE LADDER" and "THE GAP THIS FILLS" for the full argument.

    `car_prim` may be a `Usd.Prim` or a path string, the same contract
    `apply_washaway` uses. `level` is one of `'swamped'`, `'floated'` or
    `'piled'` (from `car_surge_state`) — `'dry'` is never passed here, the
    same "the caller does not call the apply function for an untouched
    object" contract `house_surge_state`'s `None` return gets, just spelled
    as a string a caller checks instead of a sentinel it tests for. `spec`
    is `car_shift_spec(...)`'s dict for `'floated'`/`'piled'`, or anything
    (including `None`) for `'swamped'`, which reads no spec at all.

        "swamped"              ATTRIBUTE ONLY, no geometry, no movement. A
                               swamped car did not float — the water on it
                               is a tonal change (a wet, darkened lower
                               body) the coverage audit's own altitude
                               argument (visible at 100-400 m) says is not
                               worth a per-car decal budget across a
                               60-plus-car suburb when MOVEMENT is what
                               actually reads from the air. See the
                               verification report for the reasoning; this
                               is a documented simplification, not an
                               oversight.
        "floated", "piled"     `car_shift_spec(...)`'s dict, applied with
                               `tornado.toss_prim` exactly the way
                               `apply_washaway`'s `"shifted"` case drives
                               the same function for a house. `piled` gets
                               no different CODE PATH from `floated` here —
                               the jam/mount pitch that makes a piled car
                               look piled is already baked into `spec` by
                               `car_shift_spec`'s own march, so this
                               function does not need to know which of the
                               two it was told.

    `ssf` is accepted for call-site symmetry with `apply_washaway` and is
    UNUSED: unlike that function, this one authors no scaled merged-box
    geometry of its own, only a rigid transform on the car's existing prim
    (`hurricane.knobs_from_env`'s own docstring notes the identical
    "kept for signature symmetry, not read" pattern elsewhere in this
    pipeline).

    Returns `True` if the prim was moved (`tornado.toss_prim`'s own
    boolean contract) — `False` for `'swamped'`, which never calls it.
    """
    from pxr import Sdf

    prim = (stage.GetPrimAtPath(car_prim) if isinstance(car_prim, str)
            else car_prim)
    if prim is None or not prim.IsValid():
        raise ValueError("no valid car prim at {0!r}".format(car_prim))
    if level not in _CAR_LEVELS:
        raise ValueError(
            "unknown washaway car level {0!r}; expected one of {1}"
            .format(level, ", ".join(_CAR_LEVELS)))

    path = prim.GetPath().pathString
    moved = False
    if level in ("floated", "piled"):
        sp = spec or {}
        moved = tornado.toss_prim(stage, path, float(sp.get("dx", 0.0)),
                                  float(sp.get("dy", 0.0)),
                                  float(sp.get("roll", 0.0)),
                                  float(sp.get("dyaw", 0.0)),
                                  pitch_deg=float(sp.get("pitch", 0.0)))

    attr = prim.GetAttribute("waterState:level")
    if not attr:
        attr = prim.CreateAttribute("waterState:level",
                                    Sdf.ValueTypeNames.Token, custom=True)
    attr.Set(level)
    if spec and spec.get("arrested_by"):
        aattr = prim.GetAttribute("waterState:arrestedBy")
        if not aattr:
            aattr = prim.CreateAttribute("waterState:arrestedBy",
                                        Sdf.ValueTypeNames.Token, custom=True)
        aattr.Set(str(spec["arrested_by"]))
    return moved


# BARK, NOT ASH_PLANKS -- ADDED (DEBRIS D5 review: "I don't really see
# floating tree trunks/fallen down trees ... add that"). Every OTHER raft
# kind reuses `planks.wood_material`'s Ash_Planks map because there is no
# dedicated per-kind asset and a multiply-tint is the only lever available
# (`_RAFT_TINT`'s own "no dedicated asset, so tint what is already bound"
# trade) -- but `vegetation.bark_material` already exists (real photographed
# bark, not tinted framing lumber) and a bole is the one raft kind for which
# swapping in a photograph-real texture is a one-line change rather than a
# new asset. MEASURED the same way `_RAFT_TEX_MEAN_LINEAR` measures
# Ash_Planks: `vegetation.BARK_BASE` (bark_oak_diff.jpg), full 4096x4096,
# sRGB->linear per-channel mean (`tools/hurricane_debris_plot.py`'s own
# offline measurement records the method).
_LOG_BARK_TEX_MEAN_LINEAR = (0.2149, 0.1787, 0.1467)
# A THICKER tile than `vegetation.bark_material`'s own 1.7 m default -- this
# module's `log` is now up to 0.7 m through (`_RAFT_DIMS`), noticeably
# thicker than the branch/limb debris that default was tuned for, and a
# coarser ridge spacing reads better on a bole this size at the altitude
# this scene renders from.
_LOG_BARK_TILE_M = 2.2


def build_rafts(stage, parent_path, specs, ssf=1.0, skin_mats=None):
    """Author the floating-debris field from `raft_specs`'s output. ONE MESH
    PER (KIND, SKIN) — the same trade `planks.build` makes for the same
    reason: a raft field is a few hundred flat mats sharing a handful of
    materials, and a prim each would be the wrong cost for geometry that
    never moves. Returns the merged prim paths.

    `skin_mats`, ADDED (DEBRIS D5 review: "match them to the houses they
    are near"): an OPTIONAL `{name: UsdShade.Material}` dict, the EXACT
    convention `planks.build(skin_mats=...)` already uses for the land
    debris field — a caller builds one material per distinct name it finds
    on `spec.get("skin")` across the specs it is about to hand this
    function (typically via `mh.palette_texture` + `planks.skin_material`,
    the same two calls the launcher's `# LAND DEBRIS` block already makes)
    and passes it here unchanged; often the SAME dict, since a `siding`/
    `deck` material name means the same thing whether the piece is on dry
    land or floating. A spec whose `skin` is absent, or present but not a
    key in `skin_mats`, groups and renders exactly as before this fix —
    `None` (the default) is BYTE-IDENTICAL to the old per-kind-only
    behaviour for every existing caller, since no spec this function has
    ever been handed carried a `"skin"` key before `raft_specs`'/
    `land_debris_specs`' skin-threading fix. Mesh count grows only by the
    number of DISTINCT `(kind, skin)` pairs actually present in `specs` —
    typically a handful of extra meshes for the palettes actually used on a
    plate, not one per house.

    PATCHES `wood_material`'s SHADER RATHER THAN EDITING IT. `planks.py` is
    outside this module's scope, and its `diffuse_color_constant` tint is a
    no-op once a texture is bound (see `_RAFT_TINT`'s docstring for the
    verification). The real per-kind colour goes on `diffuse_tint` here,
    exactly the fix `surge._dry_material` applies to the same MDL trap for
    the mud/silt/washover looks.

    THE FALLBACK WAS NEVER TINTED — the render bug behind "the mats show the
    TINT ALONE, flat mint/khaki" (DEBRIS D4 review, 2026-08-31). This
    function used to call `wood_material(..., tint=(1, 1, 1))` — literally
    NEUTRAL WHITE — and patch only `diffuse_tint` afterward, on the theory
    that `diffuse_color_constant` is inert once `diffuse_texture` is bound
    (true when the texture DOES resolve). But `diffuse_color_constant` is
    also OmniPBR's documented FALLBACK if the bound texture ever fails to
    load at render time (`wood_material`'s own comment: "it is what OmniPBR
    uses if the texture ever fails to resolve") — a condition the previous
    "path resolves" offline check (a string join, not a Nucleus HEAD request)
    cannot rule out on every render host. Left at white, that fallback
    reproduces the exact reported defect: `diffuse_tint * white == diffuse_
    tint`, i.e. the mats show the per-channel-normalised `t` below with NO
    texture variation at all — and `t`'s own numbers land suspiciously on
    the reported hues: converted sRGB, `panel`'s `t` (0.53, 0.48, 0.35) is
    (0.76, 0.73, 0.62) — light khaki — and `vegetation`'s `t` (0.63, 1.02,
    0.83) is (0.82, 1.0, 0.92) — pale mint. Every OTHER late-`diffuse_tint`
    patch site in this codebase (`damage._pbr` + `quake_rubble_usd.
    _apply_diffuse_tint`, the pattern `surge._dry_material`/`quake_flow`
    also follow) sets `diffuse_color_constant` to the REAL absolute colour
    too, belt-and-suspenders; this function was the one place that left it
    neutral. Fixed by creating the material with the real (pre-texture-mean-
    division) `tint` instead of `(1, 1, 1)` — `wood_material` puts that on
    BOTH `diffuse_color_constant` and `diffuse_tint` at creation — then
    patching `diffuse_tint` alone, below, to the texture-mean-normalised
    `t` the multiply actually needs. `test_washaway_debris.py`'s
    `test_raft_material_fallback_is_not_neutral` pins this offline.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    if not specs:
        return []
    UsdGeom.Scope.Define(stage, Sdf.Path(parent_path))
    by_group = {}
    for s in specs:
        sk = s.get("skin") if (skin_mats and s.get("skin") in skin_mats) \
            else None
        by_group.setdefault((s["kind"], sk), []).append(s)

    made = []
    for (kind, sk), group in sorted(by_group.items(),
                                    key=lambda kv: (kv[0][0], kv[0][1] or "")):
        if sk:
            # HOUSE-MATCHED: the caller already built this material (the
            # SAME `planks.skin_material`-over-`mh.palette_texture` pattern
            # `planks.build(skin_mats=...)` uses for the land debris field —
            # see this function's own docstring). No wet-darkening here: the
            # house's own material is rendered as authored, exactly the way
            # a `siding`/`deck` land-debris piece already is, rather than
            # this function reaching into a material it did not create to
            # patch a multiply-tint it may not even expose.
            mat = skin_mats[sk]
            path = "{0}/{1}".format(parent_path, "%s__%s" % (kind, sk))
            p = _merge_boxes(stage, path, group, mat, ssf, verbose=True)
            if p:
                made.append(p)
            continue

        tint = _RAFT_TINT.get(kind, (1.0, 1.0, 1.0))
        if kind not in _ORGANIC_KINDS:
            # WET DARKENING -- see `_WET_DARKEN`'s docstring. Every raft now
            # has a real draft, so the visible part of a building-material
            # piece is soaked, not dry lumber.
            tint = tuple(c * _WET_DARKEN for c in tint)
        mat_path = "{0}/RaftLooks/{1}".format(parent_path, kind)
        # `tint`, NOT `(1, 1, 1)` — see this function's own "THE FALLBACK
        # WAS NEVER TINTED" docstring section. `wood_material`/
        # `bark_material` both put this on BOTH `diffuse_color_constant`
        # (the no-texture/failed-load fallback) and `diffuse_tint`
        # (overwritten below to the texture-mean-normalised value once the
        # texture is assumed present).
        if kind == "log":
            # REAL BARK, NOT A TINTED PLANK -- see this function's own
            # "BARK, NOT ASH_PLANKS" section above `_LOG_BARK_TEX_MEAN_
            # LINEAR`.
            mat = vegetation.bark_material(
                stage, mat_path, tile_m=_LOG_BARK_TILE_M, tint=tint,
                roughness=0.90)
            tex_mean = _LOG_BARK_TEX_MEAN_LINEAR
        else:
            mat = planks.wood_material(
                stage, mat_path, tile_m=0.6, tint=tint, roughness=0.6)
            tex_mean = _RAFT_TEX_MEAN_LINEAR
        sh = UsdShade.Shader.Get(stage, mat_path + "/Shader")
        if sh:
            # PER-CHANNEL, NOT A SHARED SCALAR — see `_RAFT_TEX_MEAN_LINEAR`'s
            # docstring for the bug this replaced and the render evidence.
            t = tuple(min(8.0, c / max(1e-4, float(m)))
                     for c, m in zip(tint, tex_mean))
            sh.CreateInput("diffuse_tint",
                          Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*t))
            # VEGETATION KEEPS NO GRAIN. There is still no foliage map to
            # bind (this dict's own comment), and a multiply-tint alone
            # cannot turn visible board grain into matted canopy texture —
            # only its BRIGHTNESS was ever going to be honest. Desaturating
            # it hard mutes the wood-grain contrast so the shape (a dark,
            # low mat) carries the read rather than the pattern fighting it;
            # the other three kinds are genuinely sawn/sheet goods, so they
            # keep the grain at full saturation. `log` is real bark now, not
            # a tinted plank, so it keeps its own grain too.
            sh.CreateInput("albedo_desaturation", Sdf.ValueTypeNames.Float
                          ).Set(0.55 if kind == "vegetation" else 0.0)
        p = _merge_boxes(stage, "{0}/{1}".format(parent_path, kind),
                         group, mat, ssf, verbose=True)
        if p:
            made.append(p)
    return made


# ---------------------------------------------------------------------------
# LAND DEBRIS -- wind-torn material littering the yard downwind of a
# damaged house. ADDED 2026-08-31.
#
# `HUR_DEBRIS` ("vegetation debris pieces per damaged house", the launch
# script's own docstring, default 60) authors NOTHING: it is read into
# `N_DEBRIS` and never referenced again anywhere in
# `suburb_hurricane_launch_script.py`. A hurricane PEELS roofs (`_RAFT_DIMS`'s
# own docstring, and the `build-hurricane-scenes` skill's "Debris is 70%
# vegetation -- the opposite of the tornado plank field"), and the peeled
# material off a damaged house has to land SOMEWHERE dry, not only in the
# water this module already covers (`raft_specs`). This is that pass, and it
# is deliberately a THIN orchestration layer over machinery that already
# exists rather than a new scatter model: `planks.scatter_from_wreck` already
# authors a correctly-seated (`planks._lay`'s points-based, never-`BBoxCache`
# vertical extent, exactly the discipline `fix-floating-debris` demands),
# correctly-classed (`classes=`, this file's own addition to `planks.py`)
# downwind comet for exactly one wrecked building. `land_debris_specs` below
# only decides WHICH houses shed debris, WHAT they shed, and HOW FAR --
# capped at the first downwind neighbour -- then hands each house to that
# function unchanged.
# ---------------------------------------------------------------------------

# `hurricane.HOUSE_LEVELS`, duplicated here as a plain tuple rather than
# imported -- this module's import boundary is explicit that `hurricane`
# never crosses it (module docstring, "THE RAFT FIELD WAS ALL BUILDING
# MATERIAL": numbers cross as plain values, not as a live import). `"swept"`
# is deliberately ABSENT: it is a WASHAWAY/surge state (a slab with nothing
# left on it, its material carried off by water, never authored as wind
# debris), so a wrecks entry at that level is skipped by `land_debris_specs`
# the same way one at `"pristine"` is. Pass `level_order=hurricane.HOUSE_LEVELS`
# explicitly if a caller wants a guarantee against this copy drifting.
_HOUSE_LEVEL_ORDER = ("pristine", "shingles_lost", "cover_lost",
                     "deck_panels_lost", "roof_stripped", "roof_collapsed",
                     "partial_collapse", "leveled")

# LEVEL -> the `planks.STOCK` classes that level's damage sheds, per the
# review: "cover_lost: shingle sheets + siding; deck_panels_lost/
# roof_stripped: sheathing panels + rafters; collapse rungs: everything."
# `deck` is the roof slab class (shingle sheets, in `planks.STOCK`'s own
# naming) and `joist` is rafters/joists/headers -- see that dict's comments.
# `roof_collapsed` joins the "everything" bucket with `partial_collapse`/
# `leveled` rather than the `roof_stripped` one: the ROOF STRUCTURE itself
# has come apart at that rung, not just its covering, so framing is loose
# too.
LAND_DEBRIS_CLASSES = {
    "cover_lost":       ("deck", "siding"),
    "deck_panels_lost": ("sheathing", "joist"),
    "roof_stripped":    ("sheathing", "joist"),
    "roof_collapsed":   tuple(planks.STOCK),
    "partial_collapse": tuple(planks.STOCK),
    "leveled":          tuple(planks.STOCK),
}

# "~30-60 pieces per damaged house at L3, ~15-30 at L2" is a difference in
# per-house INTENSITY, not a separate "which preset" switch this module
# would need reading -- L3's field simply runs hotter, which is exactly
# `it` (the same per-house intensity every caller already threads through
# `hurricane.house_level_for_intensity`). TUNED, NOT SOURCED for the exact
# bounds and the reach; the SHAPE (a comet, widening gaussian tail) is
# `planks.scatter_from_wreck`'s.
LAND_DEBRIS_N_LO = 15
LAND_DEBRIS_N_HI = 60
# RAISED 15.0 -> 40.0 (D2 review, "ring, not a comet"): at the old value,
# `scatter_from_wreck`'s own `s = reach * u**1.9` put the tail's MEDIAN
# downwind draw under 4 m -- inside the base cloud's own spread (a 12 m
# house's base cloud runs `gauss(0, fp*0.34)` ~= 4 m sigma) -- so the
# "directional tail" and the "symmetric cloud around the slab" were
# indistinguishable and the whole field read as a halo, not a trail.
# Measured on the real L3 house population (`GT_hurricane.json`): the old
# value put a plate median of 2.7 m downwind displacement and 28% of a
# house's pieces inside a +-35 deg cone about its own wind bearing. 40 m
# (at intensity 1.0, BEFORE `scatter_from_wreck`'s own internal
# `reach * (0.35 + 0.65 * i)` scaling and before any `_reach_cap` shrinks it
# further), paired with `LAND_DEBRIS_TAIL_POW`/`LAND_DEBRIS_TAIL_LATERAL_*`
# below, moves that to ~15 m median downwind and ~76% cone fraction --
# comfortably past the requested 70%, with the median inside the requested
# 1-3 building-widths (10-40 m) band. TUNED, NOT SOURCED for the exact
# value; searched empirically against the real house population, see
# `tools/hurricane_debris_plot.py`'s own offline verification.
LAND_DEBRIS_REACH_M = 40.0
# THE TAIL SHAPE, ADDED for the same fix, threaded into
# `planks.scatter_from_wreck`'s new optional `tail_pow`/`tail_lateral_base`/
# `tail_lateral_growth` kwargs (that function's own docstring). The tornado
# field's defaults (1.9 / 0.30 / 0.26) bias `u**p` toward the BUILDING (most
# debris lands close, a minority carries far) and fan out loosely, which
# reads right for material thrown by a violent, localised vortex. A
# hurricane house sheds roof/wall material into steady, near-uniform wind
# instead: `tail_pow` well under 1 biases `u**p` toward `reach` (most
# pieces travel most of the way before settling) and the tightened lateral
# coefficients keep the trail narrow as it travels, which is what makes it
# read as a directional trail agreeing with `wind_bearing_fn` rather than a
# fan. TUNED, NOT SOURCED; searched jointly with `LAND_DEBRIS_REACH_M`
# above against the real house population.
LAND_DEBRIS_TAIL_POW = 0.30
LAND_DEBRIS_TAIL_LATERAL_BASE = 0.08
LAND_DEBRIS_TAIL_LATERAL_GROWTH = 0.05
LAND_DEBRIS_GROUND_Z_M = 0.02  # the suburb z-ladder's GRASS rung
                                # (`suburb_scene._Z_GRASS`), before
                                # `ground_z_scale` -- see this function's
                                # own docstring for why grass and not a
                                # per-piece road/walk/drive lookup
LAND_DEBRIS_MIN_REACH_M = 3.0
# Depth (m) past which a land-debris piece's landing point counts as
# SUBMERGED rather than dry land -- see `land_debris_specs`'s `depth_fn`
# argument ("LAND DEBRIS ON SUBMERGED GROUND", D2 review): a house standing
# IN the flood still sheds wind debris, but a piece that lands on ground
# under real water should float (join the raft field), not sit invisibly
# on the sea floor. Matches `raft_min_depth_m`'s own neighbourhood (a board
# floats in a few centimetres) but is its own named constant rather than a
# reference to that one -- the two knobs answer different questions (that
# one gates whether an ALREADY-FLOATING raft piece is placed at all; this
# one gates whether an already-authored LAND piece should have been a raft
# instead) and tying them together would make a future change to one
# silently retune the other.
LAND_DEBRIS_SUBMERGED_DEPTH_M = 0.10
# LAND -> RAFT KIND MAP, for the pieces `LAND_DEBRIS_SUBMERGED_DEPTH_M`
# catches. `planks.STOCK`'s six classes each become the nearest `_RAFT_DIMS`
# shape: `deck` (a shingle-covered roof slab, `planks.STOCK`'s own
# docstring) becomes a floating `sheet`; `siding` becomes the raft field's
# own long cladding strip; `sheathing` becomes a `panel` (both are the same
# 4x8 OSB/plywood object); `joist`/`stud` (rafters/joists/headers and
# dimensional framing) become `timber`; `board` (fence boards, trim, bare
# interior decking) becomes `timber` too -- the closest sawn-goods shape in
# `_RAFT_DIMS`. `limb` is not a `planks.STOCK` name today (land debris only
# ever draws the six classes above) but is mapped defensively to
# `vegetation`, matching the review's own stated mapping language, so a
# future woody-debris class does not silently fall through to the
# `timber` default below.
LAND_TO_RAFT_KIND = {
    "deck": "sheet",
    "siding": "siding_strip",
    "sheathing": "panel",
    "joist": "timber",
    "stud": "timber",
    "board": "timber",
    "limb": "vegetation",
}


def _reach_cap(hx, hy, heading_deg, others, min_reach_m=LAND_DEBRIS_MIN_REACH_M):
    """How far a comet leaving `(hx, hy)` toward `heading_deg` may travel
    before it reaches the NEAR edge of the first other footprint downwind
    -- "stopped at the first house footprint downwind, pile against it".

    `others` is a `(x, y, footprint_m)` list (every OTHER wrecked house on
    the plate -- callers pass `wrecks` with the house itself excluded).
    Only footprints inside a narrow FORWARD CONE are considered: a
    footprint behind the house, or well off to one side, is not in the
    comet's path and must not shorten it. Returns the distance to the
    nearest blocker's near edge, or `None` if nothing narrows the reach
    (open yard/street all the way to `LAND_DEBRIS_REACH_M`).
    """
    th = math.radians(float(heading_deg))
    ux, uy = math.cos(th), math.sin(th)
    best = None
    for ox, oy, ofp in others:
        dx_, dy_ = ox - hx, oy - hy
        s = dx_ * ux + dy_ * uy               # forward (downwind) distance
        if s <= min_reach_m:
            continue
        t = -dx_ * uy + dy_ * ux              # lateral offset from the axis
        if abs(t) > 0.6 * ofp + 4.0:          # narrow forward cone only
            continue
        near_edge = s - 0.5 * ofp
        if best is None or near_edge < best:
            best = near_edge
    return best


def land_debris_specs(wrecks, wind_bearing_fn, rng, *, min_level="cover_lost",
                      n_lo=LAND_DEBRIS_N_LO, n_hi=LAND_DEBRIS_N_HI,
                      reach_m=LAND_DEBRIS_REACH_M,
                      tail_pow=LAND_DEBRIS_TAIL_POW,
                      tail_lateral_base=LAND_DEBRIS_TAIL_LATERAL_BASE,
                      tail_lateral_growth=LAND_DEBRIS_TAIL_LATERAL_GROWTH,
                      ground_z_m=LAND_DEBRIS_GROUND_Z_M, scale=1.0,
                      level_order=None, skins_fn=None, depth_fn=None,
                      water_level_m=None,
                      submerged_depth_m=LAND_DEBRIS_SUBMERGED_DEPTH_M):
    """Wind-torn debris downwind of every sufficiently-damaged house in
    `wrecks`. Returns `planks._box`-shaped specs (`planks.scatter_from_wreck`'s
    own schema) ready for `planks.build` -- UNLESS `depth_fn` is given, see
    "LAND DEBRIS ON SUBMERGED GROUND" below, which changes the return shape.

    `tail_pow`/`tail_lateral_base`/`tail_lateral_growth`, ADDED for the
    "ring, not a comet" fix: threaded straight through to
    `scatter_from_wreck`'s identically-named kwargs (see that function's own
    docstring). Defaulting to `LAND_DEBRIS_TAIL_POW`/`LAND_DEBRIS_TAIL_
    LATERAL_BASE`/`LAND_DEBRIS_TAIL_LATERAL_GROWTH` rather than that
    function's own tornado-tuned defaults is what makes THIS caller's field
    read as a directional trail agreeing with `wind_bearing_fn`; a test
    reaching past this default to reproduce the OLD symmetric-halo shape can
    still pass `tail_pow=1.9, tail_lateral_base=0.30,
    tail_lateral_growth=0.26` explicitly.

    LAND DEBRIS ON SUBMERGED GROUND, ADDED for the same review round: a
    house still standing IN the flood sheds wind debris same as a dry one,
    but a piece whose comet lands on ground that is actually underwater
    should float, not sit invisibly on the sea floor under an opaque water
    body. `depth_fn` (a plain `(x, y) -> depth_m` callable, e.g. the
    caller's own `surge.depth_at(...)`, THE SAME CONTRACT `raft_specs`
    already takes -- this module still never imports `surge`'s `cfg` shape)
    is OPTIONAL and, when given, is checked against every piece this
    function places: one whose landing point has `depth_fn(x, y) >
    submerged_depth_m` is pulled OUT of the land list and instead built as
    a RAFT spec (`_RAFT_DIMS`'s schema, via `_one_raft`'s own draft/attitude
    formula) at the mapped kind (`LAND_TO_RAFT_KIND`) and `water_level_m`
    (the caller's own `surge.water_level(surge_cfg)`, falling back to
    `DEFAULTS["water_level_m"]` if omitted -- the same bare-bench fallback
    `raft_specs`'s own `water_level_m` default provides). THE SKIN CROSSES
    THE CONVERSION TOO (DEBRIS D5 review: "match them to the houses they
    are near") -- a `siding`/`deck` piece that started with `sp["skin"]`
    set (from `skins_fn`, below) keeps that same material name on the raft
    it becomes; `LAND_TO_RAFT_KIND`'s `deck`/`siding` mappings (-> `sheet`/
    `siding_strip`) are exactly the two entries a skinned piece can ever
    carry, so the raft's `kind` after conversion is still one
    `build_rafts`'s `(kind, skin)` grouping recognises.

    RETURN SHAPE: `depth_fn=None` (the default) returns the ORIGINAL flat
    list, unchanged for every existing caller. Passing `depth_fn` returns a
    `(land, rafts)` PAIR instead -- `land` in the original schema (minus
    whatever was pulled out), `rafts` in `_RAFT_DIMS`'s schema, ready for
    `build_rafts` (merge it into the SAME call as `raft_specs`'s own output,
    or a second `build_rafts` call under its own parent path -- either way
    the per-kind merge keeps the extra prim count fixed regardless of how
    many pieces converted).

    `wrecks` is `(x, y, footprint_m, intensity, level, palette)` tuples --
    EXACTLY the list `suburb_hurricane_launch_script.py` already collects
    during the wind pass for every non-pristine house (that script's own
    comment: `"(x, y, footprint, intensity, level, palette)"`; the trailing
    `palette` is read only if `skins_fn` is given). `min_level` gates on
    `_HOUSE_LEVEL_ORDER` (or `level_order`, if given -- pass
    `hurricane.HOUSE_LEVELS` for a guaranteed-in-sync order): only houses AT
    OR PAST it shed anything on dry land -- a `shingles_lost` house is
    cosmetically fine and a `swept` one lost its material to the WATER
    (`raft_specs` owns that), not to the wind, so `"swept"` is absent from
    the default order and such a house is skipped exactly as a `"pristine"`
    one is.

    `wind_bearing_fn` is `(x, y) -> degrees`, e.g.
    `lambda x, y: hurricane.wind_bearing_at(hcfg, x, y)` -- this module
    still never imports `hurricane` at module scope (the module docstring's
    import-boundary rule); it only calls what it is handed.

    KINDS COME FROM `LAND_DEBRIS_CLASSES[level]`. COUNT interpolates
    `n_lo..n_hi` by this house's own `intensity` (see that dict's docstring
    for why intensity, not a preset switch, is what separates "L2 numbers"
    from "L3 numbers") and is back-solved through
    `scatter_from_wreck`'s OWN internal `n_pieces * (0.25 + 0.75 * i)`
    scaling so the count that actually lands matches `n_lo..n_hi`, not that
    formula applied a second time on top of it.

    REACH IS CAPPED AT THE FIRST DOWNWIND FOOTPRINT (`_reach_cap`, over
    every OTHER house in `wrecks`) so a comet piles against a neighbour
    rather than overshooting it.

    SEATED AT ONE GROUND_Z (grass), not per-piece by surface type: this
    module has no road/sidewalk polygon data (owned by `suburb_scene`/
    `ground`) to pick asphalt or walk for a piece that happens to land on
    the street, so every piece is seated as if on the block's own grass
    rung -- a documented simplification, not an oversight (see the module
    docstring). The SEATING ITSELF is still points-based and correct:
    `planks._lay` (inside `scatter_from_wreck`) computes the exact vertical
    half-extent of the rotated box in closed form, never `UsdGeom.BBoxCache`.

    `skins_fn(wreck)`, if given, returns a `scatter_from_wreck(skins=...)`
    dict (or `None`) for one `wrecks` entry -- e.g.
    `lambda w: detail.modular_house.palette_skins(w[5])` -- so `siding`/
    `deck` pieces can wear the SAME house's own wall/roof material instead
    of the generic fallback tint. Omit it and every piece falls back to
    `planks._TINT`, exactly like the tornado corridor scatter.
    """
    order = list(level_order) if level_order else list(_HOUSE_LEVEL_ORDER)
    gate = order.index(min_level) if min_level in order else 0
    footprints = [(float(w[0]), float(w[1]),
                  float(w[2]) if len(w) > 2 else 12.0) for w in wrecks]

    out = []
    rafts = []
    kn = None
    if depth_fn is not None:
        wl = float(water_level_m) if water_level_m is not None \
            else float(DEFAULTS["water_level_m"])
        kn = {"water_level_m": wl}

    for i, w in enumerate(wrecks):
        level = w[4] if len(w) > 4 else "cover_lost"
        if level not in order or order.index(level) < gate:
            continue
        hx, hy = float(w[0]), float(w[1])
        fp = float(w[2]) if len(w) > 2 else 12.0
        it = max(0.0, min(1.0, float(w[3]) if len(w) > 3 else 0.5))
        classes = LAND_DEBRIS_CLASSES.get(level, tuple(planks.STOCK))
        heading = float(wind_bearing_fn(hx, hy))

        others = [f for j, f in enumerate(footprints) if j != i]
        cap = _reach_cap(hx, hy, heading, others)
        reach = min(float(reach_m), cap) if cap is not None else float(reach_m)
        reach = max(LAND_DEBRIS_MIN_REACH_M, reach)

        n_target = n_lo + (n_hi - n_lo) * it
        denom = max(1e-6, 0.25 + 0.75 * it)   # undo `scatter_from_wreck`'s
        n_pieces = max(1, int(round(n_target / denom)))

        skins = skins_fn(w) if skins_fn else None
        pieces = planks.scatter_from_wreck(
            hx, hy, fp, it, heading, reach, rng, n_pieces=n_pieces,
            ground_z=float(ground_z_m), scale=scale, skins=skins,
            classes=classes, tail_pow=tail_pow,
            tail_lateral_base=tail_lateral_base,
            tail_lateral_growth=tail_lateral_growth)

        if depth_fn is None:
            out.extend(pieces)
            continue
        for sp in pieces:
            if depth_fn(sp["x"], sp["y"]) > float(submerged_depth_m):
                kind = LAND_TO_RAFT_KIND.get(sp.get("class"), "timber")
                # THE SKIN CROSSES TOO -- ADDED (DEBRIS D5 review: "match
                # them to the houses they are near"). `sp["skin"]` is
                # already the wrecked house's OWN material name
                # (`planks.scatter_from_wreck(skins=...)`'s doing, only ever
                # set on a `siding`/`deck` piece) -- dropping it here would
                # have been the same "this house's debris wears no house's
                # colour" defect the water-cluster half of this fix targets,
                # just for the pieces that started on dry land and got
                # reclassified as floating because the ground under them
                # turned out to be submerged. `_append_raft` (not a bare
                # `_one_raft`) so a piece that happens to land-kind into
                # `log`... it never will (`LAND_TO_RAFT_KIND` has no `log`
                # entry) -- named here for the same reason `_append_raft` is
                # used at every OTHER raft call site: consistency, not a
                # live risk at this call site specifically.
                r = _append_raft(rafts, sp["x"], sp["y"], rng, kn,
                                 weights={kind: 1.0})
                if sp.get("skin"):
                    r["skin"] = sp["skin"]
            else:
                out.append(sp)

    return out if depth_fn is None else (out, rafts)


# ---------------------------------------------------------------------------
# FENCES -- ADDED (DEBRIS D3 review, 2026-08-31): "fences stand intact in
# 2 m of surge." `suburb_scene`/`scene_generator.apply_placements` lands
# ~600 picket/rail fence panels under `<PARENT>/fence_<group>_<i>`, one
# placement per panel (`suburb_scene._lay_fence_run` calls `pools.place(...,
# "fence", ...)` once per module along a boundary run, not once per whole
# run), and nothing in this file or the launcher's water pass has ever
# touched them: `apply_washaway`/`apply_car_washaway` move houses and cars,
# `raft_specs`/`land_debris_specs` scatter loose debris, but a fence panel
# standing in a metre of surge, or squarely downwind of a levelled house,
# renders exactly as platted.
#
# THE SAME PURE/STAGE SPLIT AS EVERY OTHER LADDER HERE. `fence_specs` below
# takes already-MEASURED geometry (`x, y, yaw_deg, length_m` -- see
# `measure_fence`'s own docstring for why length is measured off the real
# asset rather than assumed from a nominal size) plus depth/wind/house
# callables and returns one of three outcomes per fence with nothing
# touching `pxr`; `apply_fence_pose` (stage-touching) walks that list and
# edits the actual prims. The launcher's `# FENCES` block is a THIN
# orchestration layer over both: walk the prims, measure, decide, apply.
# ---------------------------------------------------------------------------

# Past this much surge a fence panel is treated as carried off rather than
# merely wet -- the review's own number ("in 2 m of surge"; `raft_specs`'s
# own `raft_min_depth_m` default of 0.25 m is "does a BOARD float here",
# a much lower bar than "does a panel lashed to two posts let go").
FENCE_WATER_DEPTH_GONE_M = 0.4
# How far downwind of a wrecked house's OWN footprint, in building widths
# (`house[2]`, the same footprint_m `land_debris_specs`/`raft_specs` already
# use), a fence counts as "in the debris/wind shadow" of it. TUNED, NOT
# SOURCED for the exact multiple; "1-3 building widths" was the requested
# band and 1.5 sits inside it, matching `LAND_DEBRIS_REACH_M`'s own median
# downwind travel (~15 m on a 12 m house, i.e. ~1.25 widths) rather than the
# comet's long tail.
FENCE_DOWNWIND_WIDTHS = 1.5
FENCE_DOWNWIND_LATERAL_FRAC = 0.75
# Only a house that has lost its own roof COVERING (or worse) is treated as
# a wind source strong enough to flatten a fence behind it -- an
# unremarkable `shingles_lost` house has not seen wind that would explain a
# fence coming down too.
FENCE_MIN_HOUSE_LEVEL = "deck_panels_lost"
FENCE_WIND_PCTILE = 0.70
# "rotate ~85 degrees" -- a small per-panel spread so a whole downed run
# does not look like one stamped copy repeated.
FENCE_FLAT_LEAN_LO_DEG, FENCE_FLAT_LEAN_HI_DEG = 80.0, 88.0
# A DOWNED FENCE RUN IS NOT A TIDY ROW OF FLAT PANELS (2026-09-01, user on
# the live 500 m plate: "the fallen fences look fine now except it's too
# perfectly fallen. Some of them should blow away, disappear, overlap,
# etc"). Three separate defects behind that one sentence, three knobs:
#
#  1. EVERY flattened panel lay at 80-88 deg, so a whole run went down at
#     one angle. A real run fails panel by panel as each post breaks or each
#     rail lets go, and a fair share end up PARTLY down — leaning on the
#     next panel, on a bush, on its own broken post — rather than flat.
#     `FENCE_PARTIAL_SHARE` of them now draw from `FENCE_PARTIAL_LEAN_*`.
#  2. NOTHING WAS EVER CARRIED OFF ON DRY LAND. `action == "gone"` fired
#     only past `FENCE_WATER_DEPTH_GONE_M`, i.e. only where surge floated
#     the panel away — so on the dry half of the plate a fence could be
#     flattened but never removed, even in the strongest wind. A light
#     timber panel in a design-level gust is exactly the thing that goes
#     over the neighbourhood; `FENCE_WIND_GONE_SHARE` of the wind-flattened
#     ones now simply leave.
#  3. PANELS STAYED ON THEIR SURVEY LINE. A fallen panel skids downwind and
#     twists as it goes, and adjacent ones end up lying ACROSS each other —
#     the overlap the report asks for. `FENCE_SLIDE_M` / `FENCE_YAW_JITTER_
#     DEG` displace and twist each one independently, so a run that fails
#     together does not land in formation.
# RAISED 2026-09-01 on the second look: "they still seem very in order. They
# need to be more scattered. It's a fence, a hurricane will blow it away."
# The first pass was too timid — a 0.55 m skid is less than a fifth of a
# panel, so a failed run still landed recognisably ON its own survey line,
# and 18% carried off left most of a flattened run present and accounted
# for. A timber panel is one of the lightest things on the lot and the
# thing most likely to simply leave.
FENCE_PARTIAL_SHARE = 0.34
FENCE_PARTIAL_LEAN_LO_DEG, FENCE_PARTIAL_LEAN_HI_DEG = 22.0, 68.0
FENCE_WIND_GONE_SHARE = 0.42    # was 0.18 -- most of a downed run leaves
FENCE_SLIDE_M = 1.9             # was 0.55 -- sigma of the downwind skid, m
FENCE_YAW_JITTER_DEG = 38.0     # was 14.0 -- sigma of the twist in plan
FENCE_GONE_RAFT_LO, FENCE_GONE_RAFT_HI = 1, 3
FENCE_GONE_DRIFT_LO_M, FENCE_GONE_DRIFT_HI_M = 2.0, 6.0
# A raft below this depth is effectively beached, not floating -- pulling a
# "gone" fence's drift back toward its own (wetter) position rather than
# letting it land on ground `depth_fn` calls dry is what "stranding at the
# waterline if the strand band is near" means in practice: a short drift
# near the shore is walked back until it is still genuinely in water.
FENCE_GONE_STRAND_MIN_DEPTH_M = 0.05


def fence_wind_threshold(fences, wind_intensity_fn, pctile=FENCE_WIND_PCTILE):
    """The `pctile`-th percentile of `wind_intensity_fn(x, y)` sampled over
    every fence in `fences` (`(x, y, yaw_deg, length_m)` tuples) -- "the
    field's OWN 70th percentile" is a property of the whole fence
    population, not of one fence, so it is resolved once here and threaded
    through `fence_specs` as a plain float rather than re-derived per call
    (which would make every fence's own position part of "the" threshold).

    Returns 1.01 (an intensity no real field ever reaches, so the "windy"
    branch never fires) for an empty `fences` -- there is no population to
    take a percentile of, and refusing to guess is the same convention
    `raft_kind_weights(None)` uses for "no live data, do not fabricate it".

    THE FIELD HAS A REAL PLATEAU, MEASURED, NOT HYPOTHETICAL. The first
    version of this docstring predicted this branch would "rarely be the
    deciding factor" because `hurricane.intensity_field` is documented as
    near-uniform. Measured against the real `suburb_hurricane_500_l2`
    layout (706 fences, `SEED=11`) that prediction was WRONG in a specific,
    fixable way: `gust_field`'s coastal-falloff term saturates
    (`coastal_max_reduction`) over a large contiguous inland share of the
    plate, so ~35% of all 706 fences shared the EXACT SAME sampled
    intensity (0.549917...), and that tied value happened to span sorted
    ranks 0.40-0.75 -- straddling the requested 0.70. `fence_specs`
    originally compared with `>=`, so EVERY fence at that one tied value
    counted as "windy", and the wind branch alone flattened 60% of the
    plate's fences instead of the intended ~30%.
    Fixed in `fence_specs`, not here: it compares with STRICT `>`, so a
    threshold that lands ON a tied plateau (this function still returns
    that exact value -- it is a faithful percentile) counts NONE of the
    plateau as windy rather than ALL of it, leaving only the fences
    genuinely ABOVE the saturated baseline (measured: ~25% of the L2
    population once the plateau is excluded, matching the requested band
    far better than 60% did). `>` slightly under-counts at a percentile
    landing between two distinct values (excludes the boundary value
    itself) but that error is a few percent, not a factor of two, and errs
    toward LEAVING A FENCE STANDING rather than flattening one on a tie it
    should not have.
    """
    if not fences:
        return 1.01
    vals = sorted(float(wind_intensity_fn(float(f[0]), float(f[1])))
                 for f in fences)
    p = max(0.0, min(1.0, float(pctile)))
    idx = min(len(vals) - 1, int(round(p * (len(vals) - 1))))
    return vals[idx]


def _house_downwind_of(fx, fy, house, wind_bearing_fn,
                       downwind_widths=FENCE_DOWNWIND_WIDTHS,
                       lateral_frac=FENCE_DOWNWIND_LATERAL_FRAC):
    """True if `(fx, fy)` sits in the downwind wind-shadow strip of `house`
    (`land_debris_specs`'s own `wrecks` shape, `(x, y, footprint_m, ...)`):
    forward of the house along the LOCAL wind bearing AT THE HOUSE, within
    `downwind_widths` building-widths, and inside a lateral corridor
    `lateral_frac` of a building-width either side of that axis.

    A PLAIN FORWARD/LATERAL PROJECTION, not `_reach_cap`'s footprint-aware
    cone -- that function decides how far ONE house's own debris comet may
    travel before it reaches the NEXT footprint downwind (it needs to know
    about every OTHER house on the plate); this only asks "is this fence in
    the wind shadow behind this one house", which does not.
    """
    hx, hy, fp = float(house[0]), float(house[1]), float(house[2])
    bearing = math.radians(float(wind_bearing_fn(hx, hy)))
    ux, uy = math.cos(bearing), math.sin(bearing)
    dx, dy = float(fx) - hx, float(fy) - hy
    s = dx * ux + dy * uy               # forward, along the wind
    t = -dx * uy + dy * ux              # lateral, across the wind
    return (0.0 <= s <= downwind_widths * fp
           and abs(t) <= lateral_frac * fp)


def _fence_fall_lean(yaw_deg, wind_bearing_deg, lean_deg):
    """The SIGNED lean (`+lean_deg` or `-lean_deg`) that tips a fence panel
    -- hinged along its own base edge, world bearing `yaw_deg` (the only
    axis it CAN hinge on; the base edge is fixed by the panel's own
    geometry) -- so it falls TOWARD `wind_bearing_deg` (the compass bearing
    the wind blows TOWARD, `hurricane.wind_bearing_at`'s own documented
    convention -- "away from the wind" means downwind, i.e. toward that
    bearing) rather than away from it.

    THE MATH: rotating a vertical panel by angle `theta` about the
    horizontal axis `(cos(yaw), sin(yaw), 0)` sends its "up" edge
    `(0, 0, 1)` to, by Rodrigues' rotation formula (`axis . up == 0`, so the
    formula reduces to `up*cos(theta) + (axis x up)*sin(theta)`, and
    `axis x up = (sin(yaw), -cos(yaw), 0)`):

        world bearing = atan2(-cos(yaw)*sin(theta), sin(yaw)*sin(theta))

    which resolves to exactly `yaw - 90` at `theta = +90` and `yaw + 90` at
    `theta = -90` (verified offline for `yaw` in {0, 30, 90, 180, 270}).
    Picking whichever of those two perpendicular bearings is closer to
    `wind_bearing_deg` and returning the matching sign is what makes "falls
    toward the wind" an exact pose rather than a coin flip.
    """
    b_plus = (float(yaw_deg) - 90.0) % 360.0
    b_minus = (float(yaw_deg) + 90.0) % 360.0

    def _ang_dist(a, b):
        return abs((a - b + 180.0) % 360.0 - 180.0)

    d_plus = _ang_dist(b_plus, float(wind_bearing_deg))
    d_minus = _ang_dist(b_minus, float(wind_bearing_deg))
    return float(lean_deg) if d_plus <= d_minus else -float(lean_deg)


def fence_specs(fences, depth_fn, wind_bearing_fn, wind_intensity_fn,
                houses, water_level_m, rng, *,
                water_depth_gone_m=FENCE_WATER_DEPTH_GONE_M,
                min_house_level=FENCE_MIN_HOUSE_LEVEL,
                downwind_widths=FENCE_DOWNWIND_WIDTHS,
                lateral_frac=FENCE_DOWNWIND_LATERAL_FRAC,
                wind_pctile=FENCE_WIND_PCTILE,
                lean_lo_deg=FENCE_FLAT_LEAN_LO_DEG,
                lean_hi_deg=FENCE_FLAT_LEAN_HI_DEG,
                partial_share=FENCE_PARTIAL_SHARE,
                partial_lo_deg=FENCE_PARTIAL_LEAN_LO_DEG,
                partial_hi_deg=FENCE_PARTIAL_LEAN_HI_DEG,
                wind_gone_share=FENCE_WIND_GONE_SHARE,
                slide_m=FENCE_SLIDE_M,
                yaw_jitter_deg=FENCE_YAW_JITTER_DEG,
                n_raft_lo=FENCE_GONE_RAFT_LO, n_raft_hi=FENCE_GONE_RAFT_HI,
                drift_lo_m=FENCE_GONE_DRIFT_LO_M,
                drift_hi_m=FENCE_GONE_DRIFT_HI_M,
                strand_min_depth_m=FENCE_GONE_STRAND_MIN_DEPTH_M,
                level_order=None):
    """One decision per fence in `fences` (`(x, y, yaw_deg, length_m)`
    tuples, ALREADY MEASURED off the placed prims -- see `measure_fence`).
    Returns a list, same order and length as `fences`, of

        {"x", "y", "yaw", "length", "action", ...}

    `action` is one of:

      "gone"    -- standing in more than `water_depth_gone_m` of surge.
                   Carries `"rafts"`: `rng.randint(n_raft_lo, n_raft_hi)`
                   `_one_raft`-shaped specs, kind `"fence"`, drifted a few
                   metres TOWARD SHALLOWER water from the fence's own
                   position (the direction a collapsed panel actually
                   drifts -- shoreward, not further out to sea; see
                   `surge._grad_deg`'s own "points toward deeper water"
                   convention, reused here negated) and walked back toward
                   the fence's own position if that drift would land a
                   piece on ground too shallow to float at all --
                   "stranding at the waterline if the strand band is near".
      "flat"    -- within `downwind_widths` building-widths of a house at
                   or past `min_house_level` (`_HOUSE_LEVEL_ORDER`/
                   `level_order`) AND inside its lateral wind-shadow
                   (`_house_downwind_of`), OR the local wind intensity is
                   STRICTLY ABOVE this field's OWN `wind_pctile`-th
                   percentile (`fence_wind_threshold`, computed once over
                   `fences` -- see that function's docstring for why `>`
                   rather than `>=`: the real field has measured tied
                   plateaus, and `>=` would sweep an entire one in).
                   Carries `"lean_deg"` (signed, `_fence_fall_lean`) and
                   `"azimuth_deg"` (`wind_bearing_fn` at the fence) for
                   `apply_fence_pose` to build the tip rotation from.
      "stands"  -- everything else; untouched.

    `houses` is `wrecks`' own shape, `(x, y, footprint_m, intensity, level,
    palette)` -- the SAME list a caller already built for
    `land_debris_specs`, handed here unchanged. `rng` is a `random.Random`
    (or anything `surge._as_random` normalises).
    """
    rng = surge._as_random(rng)
    order = list(level_order) if level_order else list(_HOUSE_LEVEL_ORDER)
    gate = order.index(min_house_level) if min_house_level in order else 0
    wrecked = [h for h in houses
              if len(h) > 4 and h[4] in order and order.index(h[4]) >= gate]

    threshold = fence_wind_threshold(fences, wind_intensity_fn, wind_pctile)
    kn = {"water_level_m": float(water_level_m)}

    out = []
    for f in fences:
        fx, fy, fyaw, flen = (float(f[0]), float(f[1]), float(f[2]),
                              float(f[3]))
        depth = float(depth_fn(fx, fy))

        if depth > water_depth_gone_m:
            n = rng.randint(int(n_raft_lo), int(n_raft_hi))
            ang = math.radians(surge._grad_deg(depth_fn, fx, fy))
            # `_grad_deg` points toward DEEPER water; a fence coming apart
            # drifts the OTHER way, toward shore.
            sx, sy = -math.cos(ang), -math.sin(ang)
            rafts = []
            for _ in range(n):
                drift = rng.uniform(drift_lo_m, drift_hi_m)
                tx, ty = fx + sx * drift, fy + sy * drift
                for _ in range(4):
                    if depth_fn(tx, ty) >= strand_min_depth_m:
                        break
                    tx, ty = 0.5 * (tx + fx), 0.5 * (ty + fy)
                tx += rng.gauss(0.0, _RAFT_TANGLE_R_M)
                ty += rng.gauss(0.0, _RAFT_TANGLE_R_M)
                rafts.append(_one_raft(tx, ty, rng, kn,
                                       weights={"fence": 1.0}))
            out.append({"x": fx, "y": fy, "yaw": fyaw, "length": flen,
                        "action": "gone", "rafts": rafts})
            continue

        near_house = any(_house_downwind_of(fx, fy, h, wind_bearing_fn,
                                            downwind_widths, lateral_frac)
                         for h in wrecked)
        # STRICT `>`, NOT `>=` -- see `fence_wind_threshold`'s docstring for
        # the measured plateau this guards against.
        windy = float(wind_intensity_fn(fx, fy)) > threshold
        if near_house or windy:
            bearing = float(wind_bearing_fn(fx, fy))
            # CARRIED OFF. See `FENCE_WIND_GONE_SHARE`: a light panel in a
            # design-level gust leaves the lot entirely. No rafts — this one
            # went downwind over dry ground, not out on the water, and the
            # plate-wide land-debris field is already what represents
            # material that came from somewhere nobody can name.
            if rng.random() < wind_gone_share:
                out.append({"x": fx, "y": fy, "yaw": fyaw, "length": flen,
                            "action": "gone", "rafts": []})
                continue
            # PART-WAY DOWN, for a third of the rest — see
            # `FENCE_PARTIAL_SHARE`.
            if rng.random() < partial_share:
                lean_deg = rng.uniform(partial_lo_deg, partial_hi_deg)
            else:
                lean_deg = rng.uniform(lean_lo_deg, lean_hi_deg)
            out.append({"x": fx, "y": fy, "yaw": fyaw, "length": flen,
                        "action": "flat",
                        "lean_deg": _fence_fall_lean(fyaw, bearing, lean_deg),
                        "azimuth_deg": bearing,
                        # SKID AND TWIST, so adjacent panels overlap instead
                        # of landing in formation. Applied by
                        # `apply_fence_pose`; a decision dict without them
                        # (an older caller's) still poses exactly as before.
                        "slide_m": rng.gauss(0.0, slide_m),
                        "yaw_jitter_deg": rng.gauss(0.0, yaw_jitter_deg)})
            continue

        out.append({"x": fx, "y": fy, "yaw": fyaw, "length": flen,
                    "action": "stands"})
    return out


# ---------------------------------------------------------------------------
# stage-touching (fences) — `pxr`/`bake` imported only inside these two.
# ---------------------------------------------------------------------------

def measure_fence(stage, prim, ssf=1.0):
    """`(x_m, y_m, yaw_deg, length_m)` for a fence prim placed by
    `scene_generator.apply_placements` (op order translate -> rotateXYZ
    (roll, pitch, yaw) -> scale -- that function's own convention).

    `length_m` is MEASURED, POINTS-BASED, never `UsdGeom.BBoxCache`: a
    fence panel is a thin flat vertical sheet, exactly the "thin diagonal
    sliver in its own local box" shape the `fix-floating-debris` skill
    documents as inflating enormously under a bbox-of-bbox query once
    rotated even a little off-axis. `bake.world_point_bounds` (this
    module's own required discipline for anything that measures where
    geometry actually is) gives the tight WORLD AABB of one Mesh's own
    points; this walks every Mesh descendant of `prim` (a fence asset may
    be posts + rails + pickets as separate meshes) and projects each
    corner onto the panel's OWN yaw axis, so the result is the panel's
    real length whichever asset this house's lot happened to draw, not a
    `_RAFT_DIMS`/`planks.STOCK`-style nominal guess.
    """
    from pxr import Gf, Usd, UsdGeom

    from . import bake

    xf = UsdGeom.Xformable(prim)
    vals = {}
    for op in xf.GetOrderedXformOps():
        vals[op.GetOpName().split(":")[-1]] = op.Get()
    t = vals.get("translate") or Gf.Vec3d(0.0, 0.0, 0.0)
    rot = vals.get("rotateXYZ")
    yaw = float(rot[2]) if rot is not None else 0.0
    x, y = float(t[0]) / ssf, float(t[1]) / ssf

    # THE PANEL'S LONG AXIS IS NOT `rotateXYZ[2]`. FIXED 2026-09-01 on the
    # user's report against the live 500 m plate: "a lot of fences look like
    # they're on their short side (which is wrong) and are standing up
    # straight".
    #
    # `rotateXYZ[2]` is the AUTHORED yaw, and for most fence assets that is
    # the run bearing PLUS the asset's own `yaw-offset` — the correction that
    # turns art modelled along its own local axis onto the boundary line.
    # `suburban.yaml`'s privacy panel (objaverse 1cec32ae..., the one in the
    # report) is native 0.08 x 3.51 with its long axis on local +Y and
    # `yaw-offset: 90.0`, so the authored yaw points ACROSS the panel, not
    # along it. Projecting onto it returned the 0.08 m THICKNESS as
    # `length_m` — and `apply_fence_pose` then hinged the "blown flat"
    # rotation about that same across-panel axis, tipping each panel over its
    # END onto its short side. One bug, both symptoms.
    #
    # SELF-CORRECTING, rather than plumbing `yaw-offset` through: measure the
    # extent along BOTH candidate axes and keep the longer. A fence panel is
    # long and thin by definition (3.51 x 0.08 here, 5.28 x 0.33 for the
    # railing, 2.00 x 0.09 for the picket), so the two are never close and
    # the choice is unambiguous. This also fixes every other asset in the
    # pool at once, whatever offset each declares, and keeps working if a new
    # one is added with a different convention.
    # THE POINTS THEMSELVES, NOT THEIR AABB. This function's docstring has
    # always promised "MEASURED, POINTS-BASED, never `UsdGeom.BBoxCache`",
    # and `bake.world_point_bounds` does honour the never-BBoxCache half —
    # but it returns the world AXIS-ALIGNED BOX of those points, and
    # projecting that box's four plan corners onto a candidate bearing is
    # not the same as projecting the geometry. For a panel aligned with
    # world X/Y they agree exactly; at any oblique bearing the box is
    # strictly larger than the panel inside it, so `length_m` over-read by
    # up to the panel's own thickness (measured: 3.587 m at a 37 deg run
    # bearing, 3.571 m at 205 deg, against a true 3.510 m).
    #
    # That never flipped the long-axis CHOICE — the error is bounded by the
    # thickness and the two candidates differ by the whole length — so the
    # hinge was already correct. It made `length_m` itself wrong, which
    # `fence_specs` threads into its decision dict for downstream use. Since
    # `world_point_bounds` already transforms every point in order to bound
    # them, projecting them directly costs nothing extra and removes the
    # error rather than documenting it.
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    corners = []
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
        if not pts:
            continue
        m = xc.GetLocalToWorldTransform(p)
        for v in pts:
            w = m.Transform(Gf.Vec3d(float(v[0]), float(v[1]), float(v[2])))
            corners.append((w[0] / ssf, w[1] / ssf))

    def _extent(a_deg):
        ca, sa = math.cos(math.radians(a_deg)), math.sin(math.radians(a_deg))
        lo = hi = None
        for cx, cy in corners:
            q = cx * ca + cy * sa
            lo = q if lo is None else min(lo, q)
            hi = q if hi is None else max(hi, q)
        return 0.0 if lo is None else (hi - lo)

    len_along, len_across = _extent(yaw), _extent(yaw + 90.0)
    if len_across > len_along:
        yaw = (yaw + 90.0) % 360.0
        length = len_across
    else:
        length = len_along
    return x, y, yaw, length


def apply_fence_pose(stage, prim_path, decision, ssf=1.0, ground_z_m=None):
    """Apply one `fence_specs` decision to the already-placed prim at
    `prim_path`. Returns True if it changed anything (False for
    `"stands"` or a prim that no longer resolves).

    "gone": `prim.SetActive(False)` -- the panel disappears; its
    water-borne remains are the `decision["rafts"]` list `fence_specs`
    already built (the caller merges these into a `build_rafts` call, the
    SAME per-kind-merged-mesh machinery the rest of the raft field uses —
    this function authors no raft geometry itself).

    "flat": REBUILDS the xform op order as translate -> orient -> scale
    (`vegetation.tip_tree`'s own idiom of replacing rather than appending —
    the tornado launcher's `tornado.toss_prim` documents exactly why
    appending a rotate to an already-placed prim's op list "applies it in
    the wrong frame"). Unlike `tip_tree`, the placement `rotateXYZ` is not
    kept as a SEPARATE op alongside a new one: it is folded into ONE
    combined `orient` quaternion (`place_rot * lean_rot` -- `Gf.Rotation`'s
    `*` composes LEFT-FIRST; this docstring asserted "RIGHT operand first"
    for a long time and it is simply false, see the verification in the
    body) so there is no ambiguity left about which frame
    `decision["lean_deg"]`'s axis is expressed in.

    SEATED POINTS-BASED, NEVER BBOX: after the rotation is authored, every
    Mesh descendant's WORLD points are measured (`bake.world_point_bounds`)
    and the translate Z is shifted so the LOWEST of them lands at
    `ground_z_m` (default: the prim's OWN pre-tip Z — this module has no
    road/sidewalk/yard surface lookup, the same documented simplification
    `land_debris_specs`'s single grass rung already makes, and a fence's
    own base is already sitting at the right local ground height because
    the SAME ground-aware placement pipeline put it there).
    """
    from pxr import Gf, Usd, UsdGeom

    from . import bake

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    action = decision.get("action")
    if action == "gone":
        prim.SetActive(False)
        return True
    if action != "flat":
        return False

    xf = UsdGeom.Xformable(prim)
    vals = {}
    for op in xf.GetOrderedXformOps():
        vals[op.GetOpName().split(":")[-1]] = op.Get()
    t = vals.get("translate") or Gf.Vec3d(0.0, 0.0, 0.0)
    sc = vals.get("scale")
    rot = vals.get("rotateXYZ")
    # THE GROUND IS MEASURED OFF THE STANDING PANEL, not read off its
    # translate (fixed 2026-09-01: "it's floating").
    #
    # `t[2]` IS NOT THE GROUND. `apply_placements` folds the asset's
    # anchor->centroid offset into the translation, so a placed prim's
    # translate z is the ground PLUS that offset — and seating the tipped
    # panel's lowest point at `t[2]` therefore parks it that offset ABOVE
    # the grass. It went unnoticed while the composition bug had panels
    # standing on their ends (they were wrong in a louder way); with the
    # pose corrected, the leftover gap is the only thing still visibly
    # wrong.
    #
    # The panel as PLACED is already sitting correctly on the ground —
    # `suburb_scene` places fences through `apply_placements`' own
    # `ground_snap`. So its own lowest world point, measured BEFORE anything
    # is re-authored, IS the local ground height, whatever `t[2]` happens to
    # mean for this asset. Measuring it also makes the seat robust to the
    # skid below: a panel that slides a metre keeps a ground reference taken
    # from its own geometry rather than from a stale translate.
    _pre = None
    for _p in Usd.PrimRange(prim):
        if not _p.IsA(UsdGeom.Mesh):
            continue
        _b = bake.world_point_bounds(_p, UsdGeom.XformCache(
            Usd.TimeCode.Default()))
        if _b is None:
            continue
        _pre = _b[0][2] if _pre is None else min(_pre, _b[0][2])
    ground_z_stage = (float(ground_z_m) * ssf if ground_z_m is not None
                      else (_pre if _pre is not None else float(t[2])))

    # TWO DIFFERENT ANGLES, AND THEY ARE NOT THE SAME ONE (fixed 2026-09-01
    # alongside `measure_fence`'s long-axis fix):
    #
    #   * the HINGE runs along the panel's own length, and that is
    #     `decision["yaw"]` — which `measure_fence` now reports as the LONG
    #     axis, correcting for whatever `yaw-offset` the asset declares.
    #   * the PLACEMENT rotation is whatever `apply_placements` authored on
    #     this prim, offset included. Rebuilding it from `decision["yaw"]`
    #     was only ever correct while the two happened to coincide; now that
    #     `measure_fence` reports the true long axis they differ by the
    #     asset's offset, and reusing it here would re-yaw every panel 90 deg
    #     off its boundary.
    #
    # So the placement half is read back off the PRIM, not reconstructed —
    # and read back in FULL. The old code folded only Z, silently discarding
    # any roll and pitch `apply_placements` had authored (its op is
    # rotateXYZ, all three components); for an asset needing an up-axis
    # correction that alone would drop a standing panel onto its side.
    hinge_yaw = float(decision["yaw"])
    axis = Gf.Vec3d(math.cos(math.radians(hinge_yaw)),
                    math.sin(math.radians(hinge_yaw)), 0.0)
    lean_rot = Gf.Rotation(axis, float(decision["lean_deg"]))
    place = rot if rot is not None else Gf.Vec3f(0.0, 0.0, 0.0)
    place_rot = (Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), float(place[2]))
                 * Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), float(place[1]))
                 * Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), float(place[0])))
    # OPERAND ORDER: `Gf.Rotation.__mul__` composes LEFT-FIRST. Verified in
    # this repo's own pxr rather than assumed, because this function's
    # docstring asserted the opposite for as long as it has existed
    # ("`Gf.Rotation`'s `*` performing the RIGHT operand first") and that
    # claim is simply false:
    #
    #     (Gf.Rotation(Z,90) * Gf.Rotation(X,90)).TransformDir((1,0,0))
    #        -> (0, 0, 1)      which is Z-then-X, i.e. LEFT applied first
    #        -> (0, 1, 0)      is what right-first would have given
    #
    # So `lean_rot * place_rot` leaned the panel FIRST, while its geometry
    # was still in its own unplaced local frame, using a hinge axis
    # expressed in WORLD coordinates — a frame mismatch — and only then
    # yawed it onto the boundary. Measured across four run bearings on the
    # real 0.08 x 3.51 x 1.83 panel, the resulting world Z extent was
    # 3.510 / 2.851 / 0.080 / 3.215 m at 0 / 37 / 90 / 205 deg: the panel
    # ends up standing on its 3.51 m END at most bearings (the reported
    # "on their short side"), and is only correct at 90 deg by coincidence.
    # `place_rot * lean_rot` places first and then leans about the world
    # axis, which is the intent, and gives a 0.08 m Z extent — the panel's
    # own thickness, i.e. genuinely flat — at every bearing.
    combined = place_rot * lean_rot
    q = combined.GetQuat()
    im = q.GetImaginary()

    # THE SKID, along the fall azimuth (the bearing the panel goes DOWN
    # toward, which `fence_specs` already records). Adjacent panels each
    # draw their own, so a run that fails together still ends up lying
    # ACROSS itself rather than in a tidy line — the "overlap" half of the
    # report. Zero for a decision dict built before this existed, and zero
    # when no azimuth was recorded, so no older caller changes.
    _slide = float(decision.get("slide_m") or 0.0)
    _az = decision.get("azimuth_deg")
    if _slide and _az is not None:
        _ar = math.radians(float(_az))
        t = Gf.Vec3d(float(t[0]) + math.cos(_ar) * _slide * ssf,
                     float(t[1]) + math.sin(_ar) * _slide * ssf,
                     float(t[2]))

    def _apply(tz):
        xf.SetXformOpOrder([])
        xf.AddTranslateOp().Set(Gf.Vec3d(float(t[0]), float(t[1]), tz))
        xf.AddOrientOp().Set(Gf.Quatf(float(q.GetReal()), float(im[0]),
                                      float(im[1]), float(im[2])))
        if sc is not None:
            xf.AddScaleOp().Set(Gf.Vec3f(float(sc[0]), float(sc[1]),
                                         float(sc[2])))

    _apply(float(t[2]))

    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    lowest = None
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        b = bake.world_point_bounds(p, xc)
        if b is None:
            continue
        z0 = b[0][2]
        lowest = z0 if lowest is None else min(lowest, z0)
    if lowest is not None:
        _apply(float(t[2]) + (ground_z_stage - lowest))
    return True
