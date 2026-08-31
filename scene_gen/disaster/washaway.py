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
everywhere else. `surge`, `tornado` and `planks` are imported at module
scope for their pure-Python helpers (`surge._as_random`, `surge._bounds`,
`surge._grad_deg`, `surge._draw`, `tornado._ladder`, `tornado.toss_prim`,
`planks._box`, `planks._FACES`, `planks.wood_material`,
`planks.skin_material`) — none of those three modules import `pxr` at
module scope either (verified by reading them), so this chain stays
importable with no USD on the path. `hurricane` is deliberately NOT
imported here at all — see "THE RAFT FIELD WAS ALL BUILDING MATERIAL"
above for why its number crosses as a plain float instead.

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


def shift_spec(depth_m, flow_bearing_deg, rng, *, dist_min_m=SHIFT_DIST_MIN_M,
               dist_max_m=SHIFT_DIST_MAX_M):
    """Where a floated house comes to rest, and how it sits when it gets
    there — `{dx, dy, dyaw, pitch, roll, d_m}`, world metres and degrees.

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
    """
    rng = surge._as_random(rng)
    d = max(0.0, float(depth_m))
    past = max(0.0, d - SHIFT_DIST_REF_DEPTH_M)
    frac = min(1.0, past / SHIFT_DIST_SATURATE_M)
    dist = dist_min_m + (dist_max_m - dist_min_m) * (frac ** 0.8)
    dist *= rng.uniform(0.75, 1.25)
    bearing = float(flow_bearing_deg) + rng.gauss(0.0, SHIFT_BEARING_JITTER_DEG)
    rad = math.radians(bearing)
    return {
        "dx": math.cos(rad) * dist,
        "dy": math.sin(rad) * dist,
        "dyaw": rng.choice((-1.0, 1.0)) * rng.uniform(*SHIFT_YAW_DEG),
        "pitch": rng.uniform(-SHIFT_PITCH_MAX_DEG, SHIFT_PITCH_MAX_DEG),
        "roll": rng.uniform(-SHIFT_ROLL_MAX_DEG, SHIFT_ROLL_MAX_DEG),
        "d_m": round(dist, 3),
    }


COLLAPSE_DIST_MIN_M = 0.5      # a heap does not travel -- see docstring
COLLAPSE_DIST_MAX_M = 3.0
COLLAPSE_DIST_SATURATE_M = 2.0
COLLAPSE_YAW_DEG = (0.0, 20.0)
COLLAPSE_PITCH_DEG = (18.0, 42.0)   # heavy list -- most of the read
COLLAPSE_ROLL_DEG = (18.0, 42.0)


def collapse_spec(depth_m, rng):
    """Pose for a house whose frame came apart in the water — `{dx, dy,
    dyaw, pitch, roll}`, the same shape `shift_spec` returns so
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
    """
    rng = surge._as_random(rng)
    d = max(0.0, float(depth_m))
    frac = min(1.0, d / COLLAPSE_DIST_SATURATE_M)
    dist = COLLAPSE_DIST_MIN_M + (COLLAPSE_DIST_MAX_M -
                                  COLLAPSE_DIST_MIN_M) * frac
    ang = math.radians(rng.uniform(0.0, 360.0))
    return {
        "dx": math.cos(ang) * dist,
        "dy": math.sin(ang) * dist,
        "dyaw": rng.choice((-1.0, 1.0)) * rng.uniform(*COLLAPSE_YAW_DEG),
        "pitch": rng.choice((-1.0, 1.0)) * rng.uniform(*COLLAPSE_PITCH_DEG),
        "roll": rng.choice((-1.0, 1.0)) * rng.uniform(*COLLAPSE_ROLL_DEG),
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
    "raft_bg_per_100m2": 0.03,
    "raft_min_depth_m": 0.5,   # a mat needs enough water to float ON, not
                                # sit in a puddle at the margin

    # -- clustered against standing obstacles -- TUNED, NOT SOURCED --
    "raft_per_house": 1.4,
    "raft_saturate_depth_m": 2.0,
    "raft_standoff_m": (0.3, 2.0),
    "raft_bearing_jitter_deg": 20.0,
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
        raft_per_house=_f("RAFT_PER_HOUSE", DEFAULTS["raft_per_house"]),
        raft_saturate_depth_m=_f("RAFT_SATURATE_DEPTH_M",
                                 DEFAULTS["raft_saturate_depth_m"]),
    )


# Flat "mats" of debris, in `planks._box`'s spec schema (l/w/t + x/y/z +
# yaw/pitch/roll). Four classes now — timber, sheet goods, whole wall
# sections, and vegetation — sized as broken pieces rather than a size range
# drawn from nothing, the same argument `planks.STOCK` makes.
# (len_range_m, width_range_m, thickness_range_m)
_RAFT_DIMS = {
    "timber": ((0.8, 3.5), (0.10, 0.30), (0.03, 0.08)),
    "sheet":  ((1.0, 2.4), (0.60, 1.20), (0.02, 0.05)),
    "wall":   ((1.5, 3.5), (1.00, 2.40), (0.08, 0.16)),
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
_RAFT_KIND_WEIGHTS = {"vegetation": 0.72, "timber": 0.28 * 0.5,
                      "sheet": 0.28 * 0.3, "wall": 0.28 * 0.2}
# Reuses `planks.wood_material`'s texture (the same "pale, bare-framing"
# argument `planks.py` makes — there is no dedicated foliage-mat asset in
# the repo) but tinted per class so a raft field is not one uniform colour —
# the same trick `planks._TINT` plays.
_RAFT_TINT = {
    "timber": (1.00, 0.97, 0.90),
    "sheet":  (0.80, 0.76, 0.68),
    "wall":   (0.88, 0.86, 0.82),
    # DARK AND ORGANIC ON PURPOSE — the opposite end of the palette from the
    # pale bare-framing tints above. A mat of stripped, waterlogged canopy
    # reads near-black-green against bright floodwater, which is exactly the
    # class separator the coverage audit calls out (S2 row 2: "a strong,
    # high-texture-frequency albedo against a muddy or reflective water
    # surface"). Same "no dedicated asset, so tint what is already bound"
    # trade `_slab_material` makes for bare concrete, applied to
    # `planks.wood_material`'s texture instead of an untextured OmniPBR.
    "vegetation": (0.30, 0.34, 0.20),
}
_RAFT_TILT_DEG = 4.0   # a floating mat is not dead flat -- it bobs
_RAFT_BOB_M = 0.03


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

    THE BUILDING THIRD IS SCALED, NOT RENORMALISED. `timber`/`sheet`/`wall`
    already sum to 1.0 within themselves at a 5:3:2 ratio (0.5/0.3/0.2);
    multiplying each by `(1 - veg_share)` keeps that ratio and lets
    `vegetation` take the rest, exactly how `_RAFT_KIND_WEIGHTS` itself is
    built above.
    """
    if veg_share is None:
        return dict(_RAFT_KIND_WEIGHTS)
    v = max(0.0, min(1.0, float(veg_share)))
    bld = 1.0 - v
    return {"vegetation": v, "timber": bld * 0.5, "sheet": bld * 0.3,
           "wall": bld * 0.2}


def _weighted_kind(rng, weights=None):
    w = weights or _RAFT_KIND_WEIGHTS
    r = rng.random() * sum(w.values())
    acc = 0.0
    for name, wgt in w.items():
        acc += wgt
        if r <= acc:
            return name
    return "timber" if "timber" in w else next(iter(w))


def _one_raft(x, y, rng, kn, weights=None):
    kind = _weighted_kind(rng, weights)
    lo_l, hi_l = _RAFT_DIMS[kind][0]
    lo_w, hi_w = _RAFT_DIMS[kind][1]
    lo_t, hi_t = _RAFT_DIMS[kind][2]
    z = float(kn["water_level_m"]) + rng.uniform(-_RAFT_BOB_M, _RAFT_BOB_M)
    return {
        "kind": kind, "x": x, "y": y, "z": z,
        "l": rng.uniform(lo_l, hi_l), "w": rng.uniform(lo_w, hi_w),
        "t": rng.uniform(lo_t, hi_t), "yaw": rng.uniform(0.0, 360.0),
        "pitch": rng.uniform(-_RAFT_TILT_DEG, _RAFT_TILT_DEG),
        "roll": rng.uniform(-_RAFT_TILT_DEG, _RAFT_TILT_DEG),
    }


def raft_specs(cfg, region, rng, houses, depth_fn, kind_weights=None):
    """Floating debris mats: a light background scatter across open water,
    plus clusters collected against the upstream (seaward) face of whatever
    is still standing. Pure Python — see `build_rafts` for the geometry.

    `houses` is `(x, y)` or `(x, y, footprint_m)` tuples — the same shape
    `tornado.car_blockers`'s `standing` argument takes, so a caller that
    already built that list for the vehicle pass can hand it here unchanged.
    `depth_fn` is `surge.depth_at(surge_cfg, region, rng)` (or any
    `(x, y) -> depth_m` callable); this function never imports `surge`'s
    `cfg` shape, only calls what it is given.

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
    """
    kn = resolve_cfg(cfg)
    rng = surge._as_random(rng)
    x0, y0, x1, y1 = surge._bounds(region)
    out = []

    # -- background: open water is not an empty sheet ---------------------
    cell = max(8.0, float(kn["raft_cell_m"]))
    nx = max(1, int(round((x1 - x0) / cell)))
    ny = max(1, int(round((y1 - y0) / cell)))
    dx, dy = (x1 - x0) / nx, (y1 - y0) / ny
    area_100 = dx * dy / 100.0
    bg_rate = float(kn["raft_bg_per_100m2"])
    min_depth = float(kn["raft_min_depth_m"])
    for iy in range(ny):
        for ix in range(nx):
            ax, ay = x0 + ix * dx, y0 + iy * dy
            cx, cy = ax + dx * 0.5, ay + dy * 0.5
            if depth_fn(cx, cy) < min_depth:
                continue
            for _ in range(surge._draw(bg_rate * area_100, rng)):
                px = ax + rng.random() * dx
                py = ay + rng.random() * dy
                out.append(_one_raft(px, py, rng, kn, kind_weights))

    # -- clustered against standing obstacles ------------------------------
    saturate = float(kn["raft_saturate_depth_m"])
    per_house = float(kn["raft_per_house"])
    lo, hi = kn["raft_standoff_m"]
    jitter = float(kn["raft_bearing_jitter_deg"])
    for h in (houses or ()):
        hx, hy = float(h[0]), float(h[1])
        fp = float(h[2]) if len(h) > 2 else 10.0
        d = depth_fn(hx, hy)
        if d < min_depth:
            continue
        bearing = surge._grad_deg(depth_fn, hx, hy)
        for _ in range(surge._draw(per_house * min(1.0, d / saturate), rng)):
            standoff = 0.5 * fp + rng.uniform(lo, hi)
            ang = math.radians(bearing + rng.gauss(0.0, jitter))
            px = hx + math.cos(ang) * standoff
            py = hy + math.sin(ang) * standoff
            out.append(_one_raft(px, py, rng, kn, kind_weights))
    return out


def summarise(cfg, region, rng, houses=(), depth_fn=None, kind_weights=None):
    """Cheap numbers for a bench, the `surge.summarise`/`tornado.summarise`
    idiom: resolved raft knobs always, plus raft/wet-house counts and a
    per-kind raft tally if a `depth_fn` is supplied. `kind_weights` passes
    straight through to `raft_specs` — see `raft_kind_weights`."""
    kn = resolve_cfg(cfg)
    houses = list(houses or ())
    out = {
        "water_level_m": round(float(kn["water_level_m"]), 3),
        "raft_cell_m": round(float(kn["raft_cell_m"]), 2),
        "houses_total": len(houses),
    }
    if depth_fn is not None:
        rafts = raft_specs(kn, region, rng, houses, depth_fn, kind_weights)
        out["rafts"] = len(rafts)
        by_kind = {}
        for r in rafts:
            by_kind[r["kind"]] = by_kind.get(r["kind"], 0) + 1
        out["rafts_by_kind"] = by_kind
        out["houses_wet"] = sum(
            1 for h in houses if depth_fn(float(h[0]), float(h[1])) > 1e-6)
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


def build_rafts(stage, parent_path, specs, ssf=1.0):
    """Author the floating-debris field from `raft_specs`'s output. ONE MESH
    PER KIND — the same trade `planks.build` makes for the same reason: a
    raft field is a few hundred flat mats sharing a handful of materials,
    and a prim each would be the wrong cost for geometry that never moves.
    Returns the merged prim paths."""
    from pxr import Sdf, UsdGeom

    if not specs:
        return []
    UsdGeom.Scope.Define(stage, Sdf.Path(parent_path))
    by_kind = {}
    for s in specs:
        by_kind.setdefault(s["kind"], []).append(s)

    made = []
    for kind in sorted(by_kind):
        tint = _RAFT_TINT.get(kind, (1.0, 1.0, 1.0))
        mat = planks.wood_material(
            stage, "{0}/RaftLooks/{1}".format(parent_path, kind),
            tile_m=0.6, tint=tint, roughness=0.6)
        p = _merge_boxes(stage, "{0}/{1}".format(parent_path, kind),
                         by_kind[kind], mat, ssf, verbose=True)
        if p:
            made.append(p)
    return made
