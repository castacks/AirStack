"""fire_people — where the people are in an URBAN STRUCTURE FIRE, and why.

A PURE PLANNER. No stage, no Isaac, no `pxr`, no Nucleus. It takes the city
placements dump (`urban_fire_city_launch_script.dump_city_placements`, schema
`fire_city_placements_dump.v1`), the fire manifest (`tools/fire_city_dry_run.py`,
`_plans/fire_city_<seed>.json`) and — when a bake has already run — the bake
sidecars (`fire_bake.sidecar`), and returns placement RECORDS. Authoring them
onto a stage is the launcher's job; the whole model runs and asserts host-side,
which is how the geometry actually gets tested (`tools/fire_people_dry_run.py`,
`tests/test_fire_people.py`).

Companion skills, and they are prerequisites, not references:
`.agents/skills/place-people-in-scenes` (the RenderPeople pack, the pose
z-offset machinery, the "every failure here is SILENT" argument, the 2D dry
run discipline) and `.agents/skills/place-people-in-tornado-scenes` (the burial
mechanics this module reuses for its two casualty classes, and the bench that
is the only way to verify one).


=============================================================================
1.  WHY THE WILDFIRE AND TORNADO MODELS BOTH FAIL HERE
=============================================================================

`disaster/people.py` is a WILDLAND-URBAN-INTERFACE evacuation model: parking
lots, open ground, pools, gridlocked egress arteries, cul-de-sac bulbs, front
yards at the burn edge. Every one of those is a feature of a suburb under a
moving flame front. A downtown block on fire has none of them — no lots to
speak of, no cul-de-sacs, no pools, no front yards, and no moving front to
flee from. Its geometry is a street canyon and its population is vertical.

`disaster/tornado_people.py` is a CASUALTY model: every figure is a body
lying in the debris, because a tornado's survivors are the easy problem and
its casualties are the benchmark. A structure fire inverts that. Its
fatalities are INSIDE — and inside is the one place a drone benchmark cannot
score — so the population a camera can see is almost entirely alive: people
who got out, people who did not get out and are at a window or on a roof
waiting for a ladder, and the crowd that a fireground always attracts.

So the class list here is its own, and the shares are argued below rather
than carried over.


=============================================================================
2.  THE RESEARCH THE SHARES COME FROM
=============================================================================

Where fire victims actually are. Change a share, change this section.

| source | finding | what it does here |
|---|---|---|
| **NFPA, Ahrens, *Home Structure Fires*** (annual series) | Home fires are ~75 % of US civilian fire deaths, and victims are overwhelmingly recovered INSIDE — in the room of fire origin, in sleeping areas, and on egress paths. Roughly a third were trying to escape when overcome. | **NOTHING IS PLACED INDOORS.** Not a modelling gap — a deliberate refusal, and the same one `people.py` made when it retired `exposed_interior`: *a target that cannot be seen cannot be labelled, and an annotation for an invisible one is worse than none.* The consequence is that this module's casualty classes are its SMALLEST (0.10 of the budget between them), which is the exact inverse of `tornado_people`, and that inversion is the point. |
| **NIST / NFPA high-rise fire studies** — One Meridian Plaza (Philadelphia 1991), Cook County Administration Building (Chicago 2003), and the Grenfell Tower Inquiry Phase 1 report (Moore-Bick, 2019) | In every one, occupants who could not use the stair went to WINDOWS for air and to be seen, on floors above the fire, and were visible from the street on multiple levels for a long time. | the `window` class (0.16) — **strictly above the fire band**, on venting and adjacent elevations. |
| **Fireground rescue doctrine** (IFSTA *Essentials*; NFPA 1500) | The two exterior places a trapped occupant is found and reached are **the window and the roof**. Roof refuge is only meaningful while the roof deck is intact; the collapse zone for a wall that may fail is **1.5 x the building height**, and crowds are held outside it. | the `roof` class (0.12), gated on an intact deck; and the standoff model in §4. |
| **Blanchi & Haynes, Australian fire fatalities 1901-2011** (quoted in the wildfire skill) | 58 % open air, 28 % structures, 8 % vehicles. | the outdoor classes together take 0.54 + 0.08, i.e. the same order of split, even though the fire type is different — people who are outside a fire are on the ground beside it, and a few are at a car. |
| **Camp Fire 2018 (NIST, Maranghides et al.) / Lahaina 2023 (FSRI)**, via the wildfire skill | People cluster. 31 temporary refuge areas held groups; recovered victims were with others far more often than alone. | groups everywhere, never a uniform scatter — see `group_sizes`. |
| **Chiu et al. 2013**, 247 Alabama tornado decedents, via the tornado skill | found with others 72 %, **ALONE 7.7 %**. | the casualty classes cluster too: `cluster_chance` puts a second body near the first. |
| **`disaster/people.py`'s own review record** ("a lone figure is unfindable; a group is not"), and `benchmark-disaster-dataset` (a person is ~13 px in a 946 px frame at benchmark altitude, ~3 px after the detector's resize) | one person per location is not a findable target at 15-40 m AGL. | minimum group size 2 for every street class. |

**What is deliberately absent, and why** (the same discipline the tornado
module keeps):

* **Anybody indoors.** See above. This is the single largest population in
  the real event and it is out of scope for an aerial benchmark.
* **Firefighters, apparatus, hose lines, aerial ladders, command posts,
  tape lines.** They are what a real fireground at T+hours looks like and
  they are a different dataset: a scene full of turnout gear scores
  "find the fire service", not "find the people". The absence is also what
  makes the standoff model a rule rather than a depiction — nothing in the
  scene is enforcing it, so the geometry has to.
* **Occupants INSIDE cars.** Not a share that was set to zero — a refusal.
  The downtown car pool is `Muyang/DownTown/Assets/Vehicle_A|Taxi|Police`
  and `vehicles.CABIN_RULES` records all of them as a single `LOD0` mesh
  with the windows painted into the texture: *"There is nothing to remove
  and no rule will ever make an occupant visible in one."* An occupant
  there is an invisible target. `at_car` figures stand or sit BESIDE the
  car instead.
* **Jumpers.** Real, documented, and not something this dataset needs.
* **Anybody on ground the fire never reached.** `people.py`'s
  `min_burn_age_s` gate, transposed: every figure here is tied to a named
  burning building from the manifest, so there is no unattached population
  to filter.


=============================================================================
3.  THE CLASSES
=============================================================================

| class | share | where | visible how |
|---|---|---|---|
| `evacuee` | 0.34 | small groups (2-5) on the sidewalk or the closed carriageway, **UPWIND** of their own burning building, past the standoff, with line of sight to a burning elevation | plan view, high contrast on pavement |
| `onlooker` | 0.20 | 1.3-2.4 x the standoff out, at street corners and across intersections, groups of 2-4 | the distractor population at range |
| `at_car` | 0.08 | 1-2 beside a kerb parking bay on the upwind block | partial occlusion by the car |
| `window` | 0.16 | leaning out of / sitting in a window opening, **strictly above the fire band**, venting or adjacent elevation | torso or legs past the facade plane |
| `roof` | 0.12 | 2-4 near the roof edge of a building whose deck is intact, on the side away from the venting elevations | the easiest aerial target in the scene, and the control case |
| `casualty_apron` | 0.07 | prone at the OUTER edge of an F5c building's debris windrow, partially under it | tornado-style partial burial |
| `roof_debris` | 0.03 | prone under roof-deck material that came out over a failed elevation | ditto; **rarest, and degrades to zero** |

**The shares are normalised, and they DEGRADE AND REFLOW — two different
things, both reported.**

* **DEGRADE** — a class with NO eligible building at all is zeroed before the
  passes run and its share is handed to `evacuee`/`onlooker` in proportion.
  `roof_debris` is the one this exists for: the coordinator's parallel policy
  change puts roof collapse on ~2 low-rise timber buildings city-wide, so on
  many manifests it has no candidate. Reported as `degraded`.
* **REFLOW** — a class can be STARVED rather than ineligible: it has
  candidates but cannot spend its budget on them. **Measured on the real
  seed-4 city: exactly ONE of the sixteen burning buildings has an intact
  deck under the drone ceiling** (six are F4, three have a band that reaches
  the top storey, five are 59-88 m tall, one is F5c), so `roof` places 7 of
  a 9-figure budget however hard it tries; `window` has two eligible
  buildings. The shortfall goes to the street classes at the end rather than
  shrinking the scene's head count for a reason that has nothing to do with
  how many people were there. Reported as `shortfall_before_reflow`, so
  "the roof class placed 7 of 9" stays visible instead of being quietly
  topped up.

**THE REAL CITY IS TALLER THAN THE CLASS LIST ASSUMES, and that is the single
biggest thing a reviewer should know.** `downtown_fire_500`'s burnable set
runs to 88 m even in the `brick_midrise` district, and both 3-D refuge
classes are capped at 45 m because the benchmark flies at 15-40 m AGL. If the
roof and window classes are wanted at strength, the lever is the SPREAD
(more, shorter buildings burning) or the flight envelope — not a bigger
share, which has nowhere to go.


=============================================================================
4.  THE STANDOFF MODEL — every number traceable to the code or the doctrine
=============================================================================

Three separate distances, and conflating them is how a crowd ends up standing
in a debris windrow.

**(a) THE DEBRIS APRON — a hard keepout, measured off the module that
authors the debris.** `fire_collapse` lays the windrow from a failed
elevation with `OUT_DEPTH_M = (1.15, 2.0)` metres of height at the wall line
and `OUT_SPREAD = (0.20, 0.34)` of the mass height of REACH out into the
street (`quake_flow._heap`). So the run-out of an F5c collapse is
`<= 0.34 * H`, and that — not a precautionary figure — is what
`apron_run_m()` returns for a collapsed elevation. It is the one number here
that is read off the code that will actually author the geometry, so it
cannot drift away from it.

**(b) THE FALLING-GLASS / SPALL ZONE — a hard keepout on any BURNING
elevation.** Glazing failing out of a fully involved compartment lands within
roughly a third of the drop height; the fireground habit of clearing "a third
of the building out" is the same number. `GLASS_FALL_FRAC = 0.33`, applied on
fire sides at every level. On a 48 m GAC midrise that is 16 m, which puts the
crowd on the far sidewalk of a 25 m street — which is where it is.

**(c) THE COLLAPSE ZONE — 1.5 x H, NFPA 1500 / IFSTA — is RECORDED, NOT
ENFORCED, and that is a deliberate compromise.** 1.5 x 48 m is 72 m: in a
500 m downtown with 20-30 m streets there is no legal position at all for
most burning buildings, so enforcing it would empty the scene. What a real
fireground does instead is put the crowd at the CORNER and on the elevation
that is not the exposure, which is what the line-of-sight cone and the upwind
rule below actually produce. Every street record therefore carries
`collapse_zone_frac = d_wall / (1.5 * H)`; the dry run reports its
distribution, and a reviewer who wants the crowd further out turns
`standoff_scale` up rather than arguing with the geometry.

**(d) UPWIND.** `heading_deg` is the direction the wind BLOWS TOWARD
(`urban_fire_spread._wind_factor`: *"`wind_dir` is the direction the wind
blows toward, in radians"*), in the same +X = 0 convention as
`urban_fire_spread.bearing`. So the downwind unit vector is
`(cos h, sin h)` and upwind is its negative. `downtown_fire_500` sets
`heading_deg: 45` — wind toward the NE, crowd to the SW. This is the single
most defensible directional rule in a structure fire (smoke and brands go
downwind; the crowd, the staging and the command post go upwind), and it is
what makes the placement field asymmetric instead of a ring. When the upwind
cone and the line-of-sight cone do not intersect for a given building, the
pass relaxes to crosswind and SAYS SO in the record's `reason`.

**(e) NOBODY ON THE CARRIAGEWAY IS A WILDFIRE RULE AND IT IS INVERTED
HERE.** The wildfire harness asserts that no human is on a road outside the
`gridlock` scenario, because a suburban street in a moving fire is an
evacuation route. A structure fire CLOSES the street: the block is taped, the
apparatus is in it, and the crowd stands on the asphalt behind the line. So
`road` is a legal surface for `evacuee`/`onlooker`/`at_car` here, weighted
below `sidewalk` but not refused. If that ever reads wrong on a render, the
knob is `road_share`, not a new keepout.


=============================================================================
5.  THE THREE-DIMENSIONAL CLASSES
=============================================================================

## 5a. WINDOWS — and why "at or above the fire" has to mean ABOVE

`urban_fire.BAND` gives the involved storey band per level, and for
`F4`/`F5`/`F5c`/`F6` its upper bound is 99, i.e. *"everything from the origin
up"* — `plan_fire` comments that this is deliberate, so `roof` is true by
construction on those levels. A building at F4 or worse therefore has NO
storey above its fire. A figure at a window on a floor that is itself fully
alight is not a rescue target, it is a fatality behind flame, and this dataset
does not place those. So:

> **window storeys are strictly above `fire.top`, up to `n_storeys - 1`.
> A building whose band reaches the roof contributes no window figures.**

On the seed-4 manifest that leaves the F1 and F3 buildings — nine of sixteen
— which is plenty, and it is a rule a reviewer can check off the manifest's
own level column.

**WHERE THE OPENING GEOMETRY COMES FROM, and the honest limitation.** A bake
sidecar's `events[*].ops[*]` carry a MEASURED opening each: the wall frame
`fr = (ox, oy, yaw, width, height, depth, dw)` and `span = (u0, u1, z_sill,
z_head)` in the bake's own frame. `fire_bake.place` is what moves them into a
city cell (rotate `(ox, oy)` about the origin by the cell yaw, add the yaw to
`fr[2]`, translate by the cell `x/y`), and `_face_point()` below reproduces
`quake_flow._b_face_pt` exactly — those two together are the whole transform
and they are pinned by a test.

**But the sidecar only records openings the fire VENTED THROUGH**, and by the
rule above this module wants openings on the storeys ABOVE the fire, where
there are no events. So the openings are not read, they are EXTRAPOLATED: the
ops on a side give the real bay pitch (median spacing of opening centres),
the real opening width, and the real sill/head heights relative to their own
storey floor, and `openings_for_side()` replicates that grid upward.
`openings_source` records which happened:

  `sidecar_grid`  a real measured grid on this side, lifted to a higher storey
  `derived`       no sidecar for this building: a default grid, and the
                  record additionally carries `needs_bench`

The clean fix is for the bake to record every opening it measured, not just
the venting ones. That is `fire_bake`'s call, not this module's, and it is
logged in §8.

**THE POSE VOCABULARY DOES NOT CONTAIN A LEAN-OUT, so two variants are built
out of what `scene_generator._HUMAN_POSES` actually has**, and both are
measured rather than asserted (segment fractions of stature from Drillis &
Contini 1966, the same source `people._LATERAL_HALF_BREADTH_H` uses):

* **`sill_sit`** — `sit_edge` with the SEAT being the window sill
  (`seat="sill"`, `z` = the sill height, which is what `people.
  _human_placement` wants for a seated pose). Thighs run horizontally forward
  0.273 H, so with the pelvis `SILL_INSET_M` inside the facade the knees and
  the hanging shins are ~0.3 m OUTSIDE it and the whole torso is in the
  opening. The most visible of the two, and the default wherever the opening
  is tall and wide enough to hold a sitter.
* **`lean_out`** — `stand_slump` on the storey floor: trunk pitched 28 deg
  (spine_01 -10, spine_02 -12, spine_03 -6), which carries the crown
  0.470 H * sin(28 deg) = 0.221 H forward of the pelvis, i.e. ~0.39 m, and
  the acromion ~0.24 m. With the pelvis half a body-depth in — the belly
  against the sill, which is what leaning out IS — that is **~0.22 m** of
  head and shoulder past the plane. **Marginal, and every
  `lean_out` record carries `needs_bench`** — 0.22 m is the difference
  between "a face in a dark hole" and "head and shoulders over the street"
  and no arithmetic settles which one it renders as.

Both variants record `protrusion_m`, and `MIN_PROTRUSION_M` (0.15) is a hard
refusal: **a window figure that does not break the facade plane is an
interior figure with extra steps.**

## 5b. ROOFS — `deck_z`, never `top_z`

`gac_fire` measures the real roof-deck height as `deck_z` and its own comment
is explicit that the bbox top *"is the parapet coping, not the deck"*; a
figure seated on `top_z` stands on top of the parapet. So: `deck_z` from
`doc["fire"]["deck_z"]` (where `gac_fire` puts it), else the main mass's
`deck_z`, else `H - PARAPET_EST_M` with `deck_source="estimated"`.

Eligibility is three gates, all of them checkable off the manifest:

1. **the deck is intact** — level in `ROOF_OK_LEVELS` (`F1`/`F2`/`F3`) and
   the band does not reach the top storey. By `urban_fire.BAND`, F4 and worse
   fail this by construction.
2. **an aerial appliance could plausibly be why they went up** — `H <=
   roof_max_h_m` (45 m by default; a 100 ft aerial reaches ~30 m and the
   margin covers a short tower). Above that, roof refuge is a helicopter
   problem and a different scene. The no-fire district rule already keeps
   `tower`/`highrise` blocks out of the burnable set entirely, so this gate
   mostly catches tall midrises.
3. **there is a side away from the fire** — the group goes on the roof edge
   whose outward normal is most opposed to the mean venting normal, inset
   `roof_edge_band_m` from the parapet so nobody is standing on the coping.

## 5c. THE TWO BURIAL CLASSES — tornado conventions, verbatim

`casualty_apron` (F5c) and `roof_debris` (F5c with roof involvement) reuse
`place-people-in-tornado-scenes` wholesale, because the mechanics of a body
partly under fallen building material do not care what knocked the building
down:

* **attitude** is one of the seven `people.LYING_POSES`; the placement is
  `prone=True` and the roll/pitch come from `people.LYING_POSES` /
  `people.LYING_SPIN`, never from a coin flip;
* **the lift** is `people._lying_lift` — half the body depth face-up/down,
  `0.115 * H` on a side — reproduced here as `lying_lift()` so the planner
  stays import-free, and pinned against the real function by a test;
* **nothing is ever fully buried.** `MAX_COVERED_FRAC = 0.55`, the value the
  1 km tornado review settled on ("some are completely obscured ... I need
  more visibility"), and the occlusion vocabulary is restricted to the light
  end (`none` 0.30 down to `flank` 0.08) so no pattern needs trimming — the
  tornado module's `test_20` pairing, carried over;
* **never centred in the heap.** The tornado module's `in_wreck` keepout
  exists because *"a body in the middle of the deepest material is invisible
  from any angle"*. Here the equivalent is a radial band: a figure lies at
  `APRON_BAND` = 0.72-1.00 of the windrow run-out, i.e. in its outer quarter,
  where the pile has thinned to a few tens of centimetres. `z` follows the
  windrow's own profile (`OUT_DEPTH_M` at the wall, ramping to grade at the
  run-out edge);
* **`needs_bench` on every single one.** The tornado skill is unambiguous
  that the ONLY way to know what a partially-buried figure looks like is to
  photograph it close in on a dedicated bench, and that a plan-level
  `covered_frac` is a claim about this module's own pieces and nothing else.
  Nothing in the 2D dry run can discharge that flag.

`roof_debris` additionally refuses any building with four walls standing.
An F5/F6 shell has lost its floors INWARD — the deck material is inside the
walls, where no camera goes — so the only aerially visible roof-debris
figure is one at a FAILED elevation, where the deck came out into the
street. That refusal is `roof_debris_indoors` and it is the aerial-visibility
filter earning its place rather than a share being quietly rounded down.


=============================================================================
6.  THE AERIAL-VISIBILITY FILTER
=============================================================================

Applied after the placement rules, because the placement rules answer "where
would a person be" and this one answers "can the benchmark score it". Every
record carries `aerial_visible`, and `plan_people` DROPS the ones that are
false (counting them by reason) rather than shipping an unlabelable target:

| class | the test |
|---|---|
| every class | not inside any building footprint + `footprint_margin_m`, except the sanctioned `window`/`roof`/`roof_debris` classes at their own z |
| `window` | `protrusion_m >= MIN_PROTRUSION_M`, i.e. some named body part is past the facade plane |
| `roof` | the record's z is the deck, and the position is inside the roof plan by at least `roof_edge_band_m` (not floating past the parapet) |
| `casualty_apron`, `roof_debris` | `covered_frac <= MAX_COVERED_FRAC`, and the body's position is OUTSIDE the standing shell footprint — under an intact slab is not a placement, it is a hole |
| street classes | on `road`, `sidewalk` or `paved` ground, never a block interior behind the building line |

`check_rules()` re-derives all of it from the records alone, so the dry run
and the tests assert the same thing the planner claimed.


=============================================================================
7.  ROADS AND SIDEWALKS FROM A DUMP THAT DOES NOT CARRY THEM
=============================================================================

`ground_class.GroundClass` is the sampler this project already uses for
"what is under this point" (`quake_flow`'s ground-response family reads it
through `ctx["ground_at"]`), and it consumes a `city_layout` with
`region` / `blocks` / `road_corridors` / `sidewalk_rects` / `paved_blocks`.

**The FC dump carries none of those** — `dump_city_placements` writes only
`region_m`, the house placements, and `typology.blocks` (the rects
`districts.rezone_blocks` keyed its typology map on). So `derive_layout()`
rebuilds the layout the sampler wants out of what IS there:

* `region` from `region_m`, centred on the origin the same way
  `scene_generator.build_city` centres it (`-w/2 .. +w/2`);
* `blocks` = the typology rects verbatim;
* `road_corridors` = **the exact rectangular complement of the blocks inside
  the region**, by coordinate-grid decomposition (all block edges plus the
  region bounds make a lattice; every cell not inside a block is a corridor
  rect). Exact for axis-aligned rects, and there are only tens of blocks;
* `sidewalk_rects` = `ground_class._sidewalk_rings(blocks, sidewalk_width_m)`
  — the module's OWN fallback ring construction, called directly rather than
  reimplemented, so the two can never disagree;
* `paved_blocks` = the blocks, because this is a downtown.

`GroundClass` then paints in its own documented priority order (paved,
sidewalk, road) and `at(x, y)` answers exactly as it does for a real layout.
**If a caller HAS the real `city_layout`, pass it as `layout=` and it is used
verbatim** — the derivation is the fallback, not the preference, and the
census says which was used.


=============================================================================
8.  OPEN ITEMS — say these out loud rather than letting them be discovered
=============================================================================

* **The sidecar records only VENTING openings.** Everything above the fire
  band is an extrapolated grid. The fix belongs in `fire_bake`/`soot_plume`
  (record the measured opening set, not just the event set); until then
  `openings_source` tells a reader which records are on measured pitch and
  which are on a default.
* **There is no `lean_out` pose.** `stand_slump` gives ~0.17 m of protrusion
  and that is at the edge of useful. A trunk pitched 55-65 deg hinged ABOVE
  the sill with the forearms on it is the correct art, and it belongs in
  `scene_generator._HUMAN_POSES` where the rest of the pose table lives.
* **The roof class is starved by the city's own height distribution.** One
  eligible deck on the seed-4 city, and the reflow is what keeps the head
  count. If the class matters, it needs a spread that lights shorter
  buildings — see the note at the end of section 3.
* **No car index.** `at_car` figures are planned at a kerb bay derived from
  the road/sidewalk boundary and carry `needs_car`; the launcher should
  ADOPT a real parked car near that point rather than adding one (the
  wildfire skill's "ADOPT PARKED CARS, DO NOT ADD MORE").
* **`covered_frac` is this module's own claim.** Same caveat the tornado
  module carries: it describes the debris THIS plan implies, not the baked
  windrow mesh, the standing shell fragments or anything else in the cell.
  `needs_bench` is the only honest resolution.
* **Nothing here has been rendered.** Every distance in §4 and every
  protrusion in §5a is arithmetic. The 2D dry run gates the x/y classes; the
  3D classes need the bench.
"""

import json
import math
import os
import random

# ---------------------------------------------------------------------------
# The character pool.
#
# Transcribed from `config/asset_sets/suburban.yaml`'s `usds.humans` block —
# the /Projects/SEI-COA copy, the one that carries the three posed statics as
# well as the six rigged characters. Kept here as a DEFAULT so the planner
# runs with no compiled config (the dry run has none); a caller with a real
# `asset_pools` should pass `humans=`/`humans_posed=` instead and this table
# is never consulted.
#
# TWO KINDS AND THE DIFFERENCE MATTERS: `RIGGED_HUMANS` are 95-joint UE4
# skeletons that `scene_generator._bind_human_pose` can pose; `POSED_HUMANS`
# have NO SKELETON at all and are frozen standing. Handing a posed static a
# pose does nothing and ships the figure upright, silently — so they are
# offered ONLY where a stander is wanted, and never for a lying, seated or
# window placement.
# ---------------------------------------------------------------------------
_PEOPLE_ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/"
                "SEI-COA/People/Assets/")
RIGGED_HUMANS = tuple(_PEOPLE_ROOT + n for n in (
    "rp_carla_rigged_001_ue4.usd",
    "rp_claudia_rigged_002_ue4.usd",
    "rp_eric_rigged_001_ue4.usd",
    "rp_manuel_rigged_001_ue4.usd",
    "rp_nathan_rigged_003_ue4.usd",
    "rp_sophia_rigged_003_ue4.usd",
))
POSED_HUMANS = tuple(_PEOPLE_ROOT + n for n in (
    "rp_dennis_posed_004.usd",
    "rp_fabienne_percy_posed_001.usd",
    "rp_mei_posed_001.usd",
))

# THE PER-ASSET CORRECTIONS, transcribed from the same `usds.humans` block.
# Every entry in that pool carries `scale: 0.01` (the RenderPeople exports are
# authored in CENTIMETRES) and `yaw-offset: 90` (the art faces -Y, not +X);
# all nine are Z-up, so no `axis_up` override.
#
# `HUMAN_YAW_OFFSET_DEG` IS ADDED BY THE PRODUCER, NOT THE CONSUMER.
# `scene_generator.apply_placements` reads `p["yaw_deg"]` and authors it
# verbatim — it applies no per-asset offset of its own (the offset lives in
# `build_city`'s own placement constructor, `_yo(usd)`, which this module does
# not go through, and in `people._human_placement`, which does
# `float(yaw) + ap.yaw_of(usd)`). A record's `yaw_deg` here is the FACING
# BEARING the figure should end up looking along (+X = 0, the same frame as
# `urban_fire_spread.bearing`); `to_placements` adds the offset. Drop it and
# every figure in the scene is turned a quarter circle — which is exactly the
# failure `place-people-in-scenes` records for car occupants ("`yaw_deg` IS
# NOT THE HEADING").
HUMAN_SCALE = 0.01
HUMAN_YAW_OFFSET_DEG = 90.0
HUMAN_AXIS_UP = "Z"

# Nominal stature. The pack measures 1.73-1.86 m in Isaac; the planner needs
# ONE number for its protrusion and lift arithmetic and it does not have a
# resolver, so it uses this and records it. The launcher re-derives the true
# per-character values through `people._human_placement`, which measures.
NOMINAL_HEIGHT_M = 1.78

# Body fractions of stature (Drillis & Contini 1966), the same source
# `people._LATERAL_HALF_BREADTH_H` cites. Used for the window protrusion and
# the lying lift.
_HIP_H = 0.530           # greater-trochanter / lumbar hinge height
_ACROMION_H = 0.818      # shoulder height
_CROWN_H = 1.000
_THIGH_L = 0.273         # hip -> knee, i.e. the forward reach of a sitter
_LATERAL_HALF_BREADTH_H = 0.115   # people._LATERAL_HALF_BREADTH_H, copied
_BODY_HALF_DEPTH_M = 0.17         # half the A-pose bbox depth (sy 0.33-0.40)

CLASSES = ("evacuee", "onlooker", "at_car", "window", "roof",
           "casualty_apron", "roof_debris")
# The classes whose budget absorbs a degraded class's share.
_FALLBACK_CLASSES = ("evacuee", "onlooker")
# The classes that live on the ground plane and go through the street gates.
STREET_CLASSES = ("evacuee", "onlooker", "at_car")
# The classes that are legitimately over or inside a building footprint.
AERIAL_EXEMPT_CLASSES = ("window", "roof")

# Poses, by class. Every name is a key of `scene_generator._HUMAN_POSES`;
# `people.BANNED_POSES` ("wave") appears nowhere and must not be added.
_STAND_POSES = ("idle", "walk", "stand_slump")
_CLASS_POSES = {
    # A crowd behind a fire line is standing and watching; a few have sat
    # down on the kerb. `sit_edge` is legal here because the caller CAN see
    # the seat (the kerb) and passes `seat=` — the rule `people.add_person`
    # enforces.
    "evacuee": (("idle", 0.34), ("walk", 0.16), ("stand_slump", 0.24),
                ("sit_edge", 0.16), ("crouch", 0.10)),
    "onlooker": (("idle", 0.52), ("walk", 0.30), ("stand_slump", 0.18)),
    "at_car": (("idle", 0.34), ("stand_slump", 0.30), ("sit_edge", 0.36)),
    "roof": (("idle", 0.40), ("walk", 0.14), ("stand_slump", 0.22),
             ("crouch", 0.12), ("sit_ground", 0.12)),
}
# The lying attitudes, and the mix. A third face-up, a third face-down, a
# third on a side — `tornado_people`'s own split, and for the same reason it
# gives: there is no literature on the attitude distribution, so the set
# covers the space evenly and says so.
_LYING_POSES = (("lying_supine", 0.17), ("lying_supine_open", 0.16),
                ("lying_prone", 0.17), ("lying_prone_reach", 0.16),
                ("lying_side_l", 0.13), ("lying_side_r", 0.13),
                ("lying_curled_l", 0.08))
# `people.LYING_POSES` — the roll that lays each one down. Copied rather than
# imported so this module has no import-time dependency; `tests/
# test_fire_people.py` asserts the two agree.
LYING_ROLL = {"lying_prone": 90.0, "lying_prone_reach": 90.0,
              "lying_supine": -90.0, "lying_supine_open": -90.0,
              "lying_side_l": 90.0, "lying_side_r": 90.0,
              "lying_curled_l": 90.0}
LYING_SPIN = {"lying_side_l": 90.0, "lying_side_r": -90.0,
              "lying_curled_l": 90.0}

# The occlusion vocabulary, restricted to the light end. Names and covered
# spans are `tornado_people`'s; the WEIGHTS are the post-1-km-review ones
# (the four widest patterns are off), and every pattern listed fits under
# MAX_COVERED_FRAC untrimmed — the pairing `tornado_people`'s `test_20`
# exists to protect, carried over here as its own test.
OCCLUSION = (("none", 0.00, 0.30), ("feet_shins", 0.30, 0.22),
             ("legs", 0.52, 0.18), ("midriff", 0.28, 0.12),
             ("torso", 0.38, 0.10), ("flank", 0.23, 0.08))
MAX_COVERED_FRAC = 0.55

# A window figure must break the facade plane by at least this much or it is
# an interior figure with extra steps.
MIN_PROTRUSION_M = 0.15

DEFAULTS = {
    # None => derived as `per_building * n_burning`, clamped to [24, 90].
    "total": None,
    "per_building": 4.5,
    "total_min": 24,
    "total_max": 90,

    "shares": {"evacuee": 0.34, "onlooker": 0.20, "at_car": 0.08,
               "window": 0.16, "roof": 0.12, "casualty_apron": 0.07,
               "roof_debris": 0.03},
    "group_sizes": {"evacuee": (2, 5), "onlooker": (2, 4), "at_car": (1, 2),
                    "roof": (2, 4), "window": (1, 1),
                    "casualty_apron": (1, 2), "roof_debris": (1, 1)},
    # Chiu et al. 2013: a casualty found ALONE is the 7.7 % case.
    "cluster_chance": 0.55,

    # --- standoff (see section 4) ---
    "glass_fall_frac": 0.33,       # (b), fire sides, every level
    "apron_spread": 0.34,          # (a), fire_collapse.OUT_SPREAD top of range
    "apron_min_m": 6.0,            # ...but never less than this
    "wall_clear_frac": 0.12,       # non-fire sides: a nominal clearance
    "wall_clear_min_m": 3.0,
    "wall_clear_max_m": 10.0,
    "collapse_zone_frac": 1.5,     # (c), NFPA 1500 / IFSTA — RECORDED only
    "standoff_scale": 1.0,         # one knob to push the whole crowd out
    # A CEILING ON THE STANDOFF, AND IT IS A SCENE CONSTRAINT, NOT A PHYSICS
    # CLAIM. `glass_fall_frac * H` is a rule of thumb calibrated on mid-rise
    # frontage and it does not extrapolate: the 2026-08-31 manifest put
    # SKYSCRAPERS in the burnable set (`height_class: skyscraper`, 140 m,
    # 231 m and 302 m tall), where 0.33 H is a 100 m keepout and the onlooker
    # band then threw figures 182 m from their own building — three streets
    # away, on somebody else's block, attributed to a fire they are not
    # standing at. Falling glass from 300 m really can travel that far; a
    # crowd on a 500 m plate cannot be put there and still read as THIS
    # building's crowd. So the standoff is capped, and the wall distance with
    # it. If a taller scene ever wants the real zone, raise both together and
    # look at the picture.
    "standoff_max_m": 35.0,
    "max_wall_dist_m": 60.0,
    "onlooker_band": (1.3, 2.4),   # x the standoff
    "evacuee_band": (1.0, 1.55),

    # --- direction ---
    "heading_deg": 45.0,           # overridden from the preset when readable
    "upwind_cos_evacuee": 0.34,    # within ~70 deg of upwind
    "upwind_cos_onlooker": -0.17,  # anywhere but straight downwind
    "los_half_angle_deg": 75.0,    # must be able to see a burning elevation

    # --- surfaces and spacing ---
    # SURFACE PREFERENCE, best first. A fireground CLOSES the street, so the
    # carriageway is legal here where the wildfire model refuses it (section
    # 4(e)) — but the frontage sidewalk is where a held-back crowd actually
    # stands, and a `paved` BLOCK INTERIOR is the back of the block: real,
    # reachable by a rear exit, and much harder to see from above between two
    # buildings. So a draw that lands on a lower-ranked surface is NUDGED
    # toward a better one before it is accepted, and kept only if the nudge
    # fails. The first real-city run put 9 of 24 evacuees and 8 of 13
    # onlookers on block interiors, which is what this exists to correct.
    "surface_rank": ("sidewalk", "road", "paved"),
    "road_share": 0.30,            # of the draws that may STAY on the road
    "footprint_margin_m": 0.6,
    "min_sep_m": 0.9,
    "group_radius_m": (1.4, 3.0),
    "sidewalk_width_m": 2.0,
    "kerb_bay_offset_m": 2.2,      # into the carriageway from the kerb line
    "kerb_z_m": 0.13,              # sidewalk top above the carriageway

    # --- windows ---
    "sill_h_m": 1.00,              # default sill above its storey floor
    "head_h_m": 2.10,              # ...and head, when no sidecar says otherwise
    "opening_w_m": 1.20,
    "bay_pitch_m": 3.20,
    # HOW FAR INSIDE THE FACADE PLANE THE PELVIS SITS, per variant — and
    # they are different anchors, which is why there are two numbers.
    #   sill_sit  the pelvis is ON the sill, set back onto the inner part of
    #             it so the sitter is not balanced on the outer arris.
    #   lean_out  the pelvis is half a body depth in, i.e. the BELLY is
    #             against the sill — which is what leaning out of a window
    #             is. Charging this variant the sill inset AS WELL as its own
    #             body depth (the first draft did) left it 0.02 m of
    #             protrusion, below `MIN_PROTRUSION_M`, so every `lean_out`
    #             candidate was refused and the variant could never appear —
    #             a whole branch dead behind a plausible-looking number.
    "sill_inset_m": 0.20,
    "lean_out_inset_m": _BODY_HALF_DEPTH_M,
    "sit_head_clear_m": 0.95,      # opening height needed to hold a sitter
    "sit_width_m": 0.65,
    "sill_sit_share": 0.70,
    # HOW MANY WINDOW FIGURES ONE BUILDING MAY CARRY. 3 is the nominal, but
    # the cap ADAPTS UPWARD when few buildings are eligible: on the real
    # seed-4 city only two of sixteen burning buildings have a storey both
    # above the fire band and under the drone ceiling, so a hard 3 left half
    # the class's budget unspent. Several people at windows on DIFFERENT
    # floors and elevations of the same building is exactly what every
    # high-rise fire record shows (One Meridian Plaza, Cook County, Grenfell),
    # so spending the budget on the eligible buildings is more faithful than
    # under-filling the class. `window_max_per_building_hard` is the ceiling
    # the adaptation may not pass.
    "window_max_per_building": 3,
    "window_max_per_building_hard": 8,
    # A CEILING ON HOW HIGH A WINDOW FIGURE MAY BE, and it is the drone's,
    # not the building's. `benchmark-disaster-dataset` puts the urban
    # search band at 15-40 m AGL (`max_altitude_agl_m`), so a figure at a
    # 27th-storey window 83 m up — which the first real-city run produced,
    # on an F1 building whose fire band topped out at storey 14 — is ABOVE
    # every camera that will ever look for it. Real, and unobservable by
    # this benchmark's flight envelope, so it is refused
    # (`window_too_high`) rather than shipped as an unfindable label. Same
    # number as `roof_max_h_m`, and for the same reason.
    "window_max_z_m": 45.0,

    # --- roofs ---
    "roof_max_h_m": 45.0,
    # ONE GROUP PER ROOF. The first real-city run put all nine roof figures
    # on a single building (`rng.choice` with a `used` set that stopped
    # filtering once it was full): nine people on one roof and none on the
    # eight other eligible ones is one location, not nine, and coverage is
    # counted in LOCATIONS (`place-people-in-scenes`: "Count LOCATIONS, not
    # just people").
    # TWO, not one. On the real seed-4 city exactly ONE of the sixteen
    # burning buildings has an intact deck under the drone ceiling (the
    # other fifteen are F4-or-worse, band-to-the-top, or 59-88 m tall), and
    # a cap of 1 turned the whole class into a single group of three. Two
    # groups on DIFFERENT edges of the same deck is two locations, which is
    # what coverage is counted in — see `far_sides`.
    "roof_max_groups_per_building": 2,
    "roof_edge_band_m": (1.2, 3.5),
    "parapet_est_m": 1.0,

    # --- burial ---
    "apron_band": (0.72, 1.00),    # of the run-out; the OUTER quarter
    # ROOF DECK LANDS FURTHER OUT THAN THE WALL WINDROW'S OWN TAIL: it came
    # off the top and slid over the failed elevation, so its band starts a
    # little inboard of the wall rubble's. Still never the mound.
    "roof_debris_band": (0.62, 0.95),
    "out_depth_m": (1.15, 2.0),    # fire_collapse.OUT_DEPTH_M
    "max_covered_frac": MAX_COVERED_FRAC,

    "roof_ok_levels": ("F1", "F2", "F3"),
    "collapse_levels": ("F5c",),
    "gutted_levels": ("F5", "F6"),
    "max_tries": 220,
}

# `urban_fire.BAND`, copied. (lo, hi) storeys involved above the origin; a
# `hi >= 99` means "everything from the origin up" and hence roof involvement
# by construction — the fact section 5a turns into the window rule.
BAND = {"F0": (1, 1), "F1": (1, 2), "F2": (1, 2), "F3": (3, 6),
        "F4": (4, 99), "F5": (4, 99), "F5c": (4, 99), "F6": (4, 99)}

# `quake_flow._SIDE_NORMAL`, copied. +Y is North, +X is East — confirmed
# against `gac_fire`'s own face map (`{"S": lo[1], "E": hi[0], "N": hi[1],
# "W": lo[0]}` over a (x, y, z) bbox).
SIDE_NORMAL = {"S": (0.0, -1.0), "N": (0.0, 1.0),
               "E": (1.0, 0.0), "W": (-1.0, 0.0)}
_ADJACENT = {"N": ("E", "W"), "S": ("E", "W"),
             "E": ("N", "S"), "W": ("N", "S")}


# ===========================================================================
# Small geometry
# ===========================================================================
def _rot(x, y, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return x * c - y * s, x * s + y * c


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _norm(x, y):
    d = math.hypot(x, y)
    return (x / d, y / d) if d > 1e-9 else (1.0, 0.0)


def side_normal_world(side, yaw_deg):
    """The outward unit normal of one elevation of a building placed at
    `yaw_deg`. Reproduces `quake_flow._outward` for a mass whose `yaw` is the
    cell yaw, which is exactly what `fire_bake.place` leaves it at."""
    nx, ny = SIDE_NORMAL[side]
    return _rot(nx, ny, yaw_deg)


def face_center(rec, side):
    """World `(x, y)` of the middle of one elevation, and its half-width."""
    W, D = float(rec["W"]), float(rec["D"])
    yaw = float(rec.get("yaw_deg", 0.0))
    lx, ly = SIDE_NORMAL[side]
    ox, oy = lx * W / 2.0, ly * D / 2.0
    half = (W / 2.0) if side in ("N", "S") else (D / 2.0)
    dx, dy = _rot(ox, oy, yaw)
    return float(rec["x"]) + dx, float(rec["y"]) + dy, half


def point_in_obb(px, py, cx, cy, W, D, yaw_deg, margin=0.0):
    """Is `(px, py)` inside the axis-aligned box `W x D` centred at
    `(cx, cy)` and turned by `yaw_deg`, grown by `margin`?"""
    dx, dy = px - cx, py - cy
    u, v = _rot(dx, dy, -yaw_deg)
    return abs(u) <= W / 2.0 + margin and abs(v) <= D / 2.0 + margin


def dist_to_obb(px, py, cx, cy, W, D, yaw_deg):
    """Distance from a point to the nearest wall of a turned box. Zero
    inside."""
    dx, dy = px - cx, py - cy
    u, v = _rot(dx, dy, -yaw_deg)
    du = max(0.0, abs(u) - W / 2.0)
    dv = max(0.0, abs(v) - D / 2.0)
    return math.hypot(du, dv)


def obb_exit_m(cx, cy, W, D, yaw_deg, ux, uy):
    """Distance from the centre of a turned box to its surface along the unit
    direction `(ux, uy)`.

    THIS IS WHY A STREET DISTANCE IS NOT MEASURED FROM THE CENTRE. The first
    draft placed a group at `radius * k + standoff` from the building centre,
    where `radius` was the half-diagonal — so on a 30 x 30 m building the
    whole near half of the standoff band fell INSIDE the footprint and was
    refused (1,611 `in_footprint` refusals on the first synthetic run), and
    the survivors were the draws that happened to point along a diagonal.
    Ray-box exit gives the real half-extent along the bearing actually drawn,
    so `standoff * band` means metres from the WALL, which is what every
    number in section 4 is quoted in.
    """
    lx, ly = _rot(float(ux), float(uy), -float(yaw_deg))
    ts = []
    for comp, half in ((lx, W / 2.0), (ly, D / 2.0)):
        if abs(comp) > 1e-9:
            ts.append(half / abs(comp))
    return min(ts) if ts else max(W, D) / 2.0


def rect_complement(region, rects):
    """The EXACT rectangular complement of `rects` inside `region`.

    Coordinate-grid decomposition: every rect edge plus the region bounds
    makes a lattice; a lattice cell whose centre is inside no rect is part of
    the complement. Exact for axis-aligned rects, and with tens of blocks the
    lattice is a few thousand cells. Adjacent cells are NOT merged — the only
    consumer is `ground_class.GroundClass`, which rasterises anyway, so
    merging would buy nothing but a bug surface.
    """
    x0, y0, x1, y1 = (float(v) for v in region)
    xs = sorted({x0, x1} | {float(r[k]) for r in rects for k in (0, 2)})
    ys = sorted({y0, y1} | {float(r[k]) for r in rects for k in (1, 3)})
    xs = [x for x in xs if x0 - 1e-6 <= x <= x1 + 1e-6]
    ys = [y for y in ys if y0 - 1e-6 <= y <= y1 + 1e-6]
    out = []
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            ax, bx = xs[i], xs[i + 1]
            ay, by = ys[j], ys[j + 1]
            if bx - ax < 1e-6 or by - ay < 1e-6:
                continue
            mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
            inside = any(r[0] <= mx <= r[2] and r[1] <= my <= r[3]
                         for r in rects)
            if not inside:
                out.append((ax, ay, bx, by))
    return out


# ===========================================================================
# Inputs
# ===========================================================================
def load_dump(path):
    """The city placements dump, validated. Same schema check
    `fire_city_dry_run.load_placements_dump` makes, and the same refusal to
    treat a malformed dump as an empty one."""
    with open(path) as fh:
        doc = json.load(fh)
    if doc.get("schema") != "fire_city_placements_dump.v1":
        raise ValueError(
            "{0!r} is not schema 'fire_city_placements_dump.v1' (got {1!r})"
            .format(path, doc.get("schema")))
    if not doc.get("placements"):
        raise ValueError("{0!r} carries no placements".format(path))
    return doc


def load_manifest(path):
    """The fire manifest written by `tools/fire_city_dry_run.py`."""
    with open(path) as fh:
        doc = json.load(fh)
    if not isinstance(doc.get("records"), list):
        raise ValueError("{0!r} has no 'records' list".format(path))
    return doc


def load_sidecars(directory):
    """`{tag_or_cell: doc}` for every `fire_bake` sidecar under `directory`.

    Keyed BOTH by the sidecar's own `tag` and by its `city.cell` (when the
    city driver recorded one), because a manifest record can be matched by
    either and neither is guaranteed present. Files that are not fire-bake
    sidecars are skipped in silence; an unreadable one is reported.
    """
    out = {}
    if not directory or not os.path.isdir(directory):
        return out
    for name in sorted(os.listdir(directory)):
        if not name.endswith(".json"):
            continue
        path = os.path.join(directory, name)
        try:
            with open(path) as fh:
                doc = json.load(fh)
        except (ValueError, OSError) as exc:
            print("[fire_people] sidecar {0} unreadable: {1}".format(path, exc))
            continue
        if not isinstance(doc, dict) or "fire" not in doc:
            continue
        if doc.get("tag"):
            out[str(doc["tag"])] = doc
        cell = ((doc.get("city") or {}).get("cell"))
        if cell:
            out[str(cell)] = doc
    return out


def derive_layout(dump, sidewalk_width_m=None):
    """A `city_layout` for `ground_class.GroundClass` out of an FC dump.

    See section 7. Returns the layout dict plus a `_source` note the census
    prints, so a reader can never mistake a derivation for the real thing.
    """
    from . import ground_class as gc

    rm = dump.get("region_m") or [500.0, 500.0]
    w, h = float(rm[0]), float(rm[1])
    region = (-w / 2.0, -h / 2.0, w / 2.0, h / 2.0)
    blocks = [tuple(float(v) for v in b["rect"])
              for b in ((dump.get("typology") or {}).get("blocks") or [])
              if b.get("rect") and len(b["rect"]) == 4]
    blocks = [(min(b[0], b[2]), min(b[1], b[3]),
               max(b[0], b[2]), max(b[1], b[3])) for b in blocks]
    sw = float(sidewalk_width_m if sidewalk_width_m is not None
               else DEFAULTS["sidewalk_width_m"])
    return {
        "region": region,
        "blocks": blocks,
        "road_corridors": rect_complement(region, blocks),
        # The sampler's OWN ring construction, called rather than copied.
        "sidewalk_rects": gc._sidewalk_rings(blocks, sw),
        "paved_blocks": list(blocks),
        "_source": "derived from FC dump typology blocks",
    }


def layout_rects(layout, key):
    """`[(x0, y0, x1, y1), ...]` for one `city_layout` list, whatever shape
    its entries are in.

    `road_corridors` entries are DICTS (`scene_generator._subdivide_region_
    metric` emits `{"x0", "y0", "x1", "y1", ...}`) while `blocks`,
    `paved_blocks` and `sidewalk_rects` are bare tuples — a real layout mixes
    both, and a renderer or a check that indexes `r[0]` works on the derived
    layout and silently draws nothing on the real one. `ground_class._rect`
    already normalises exactly this, so it is called rather than repeated.
    """
    from . import ground_class as gc
    return gc._rects((layout or {}).get(key) or [])


def make_ground(dump, layout=None, sidewalk_width_m=None, cell_m=1.0):
    """`(GroundClass, layout)`. A real `city_layout` passed as `layout` is
    used verbatim; otherwise `derive_layout` runs."""
    from . import ground_class as gc

    if layout is None:
        layout = derive_layout(dump, sidewalk_width_m)
    return gc.GroundClass(layout, sidewalk_width_m=sidewalk_width_m,
                          cell_m=cell_m), layout


# ===========================================================================
# The fire model, per building
# ===========================================================================
def band_top(rec, doc=None):
    """The highest storey involved in this building's fire.

    From the sidecar when there is one (`fire.top`, the number the bake
    actually planned on); otherwise the CONSERVATIVE (highest) value
    `urban_fire.BAND` allows for the level, from the manifest's own `origin`
    storey and `n_storeys`. Conservative on purpose: it is the floor of the
    window class, and over-estimating it only ever refuses a window that
    might have been legal, never places one inside the fire.
    """
    if doc:
        f = doc.get("fire") or {}
        if f.get("top") is not None:
            return int(f["top"])
    n = max(1, int(rec.get("n_storeys") or 1))
    origin = max(0, min(n - 1, int(rec.get("origin") or 0)))
    lo, hi = BAND.get(str(rec.get("level")), (1, 1))
    if hi >= 99:
        return n - 1
    return min(n - 1, origin + hi - 1)


def roof_involved(rec, doc=None):
    """Does the fire reach the top storey? `urban_fire.plan_fire` computes
    exactly `storeys[-1] >= n - 1` and stores it as `fire.roof`."""
    if doc:
        f = doc.get("fire") or {}
        if f.get("roof") is not None:
            return bool(f["roof"])
    return band_top(rec, doc) >= max(1, int(rec.get("n_storeys") or 1)) - 1


def roof_collapsed(rec, doc=None):
    """Has this building's ROOF DECK come down?

    Checked in decreasing order of authority, so a future explicit flag wins
    over every inference:

      1. `doc["fire"]["roof_collapse"]`   — a flag the bake may grow
      2. a sidecar note naming roof collapse / burn-through
      3. `rec["roof_collapse"]`           — the manifest may grow the same
      4. inference: an F5/F6 shell has lost its floors and its roof inward,
         and an F5c building whose fire reached the top storey has lost its
         deck over the failed elevation.
    """
    if doc:
        f = doc.get("fire") or {}
        if f.get("roof_collapse") is not None:
            return bool(f["roof_collapse"])
        for note in (doc.get("notes") or []):
            s = str(note).lower()
            if "roof_collapse" in s or "roof collapse" in s \
                    or "roof_burnthrough" in s or "roof burn-through" in s:
                return True
    if rec.get("roof_collapse") is not None:
        return bool(rec["roof_collapse"])
    lvl = str(rec.get("level"))
    if lvl in DEFAULTS["gutted_levels"]:
        return True
    return lvl in DEFAULTS["collapse_levels"] and roof_involved(rec, doc)


def deck_z(rec, doc=None, parapet_est_m=None):
    """`(z, source)` of the ROOF DECK — never the parapet coping.

    `gac_fire` measures it and puts it on the fire dict (`fire["deck_z"] =
    m.get("deck_z")`); its own comment on why `top_z` is wrong is quoted in
    section 5b. The estimate is `H - parapet_est_m`.
    """
    pe = float(parapet_est_m if parapet_est_m is not None
               else DEFAULTS["parapet_est_m"])
    if doc:
        f = doc.get("fire") or {}
        if f.get("deck_z") is not None:
            return float(f["deck_z"]), "sidecar"
        for m in (doc.get("masses") or {}).values():
            if isinstance(m, dict) and m.get("deck_z") is not None:
                return float(m["deck_z"]), "sidecar_mass"
    return max(2.5, float(rec["H"]) - pe), "estimated"


def storey_period(rec, doc=None):
    """Metres per storey. `H / n_storeys`, which is what the manifest's own
    storey count was derived from in the first place."""
    n = max(1, int(rec.get("n_storeys") or 1))
    return float(rec["H"]) / n


def apron_run_m(rec, cfg):
    """How far a failed elevation's debris windrow reaches into the street.

    `fire_collapse.OUT_SPREAD` is `(0.20, 0.34)` of the mass height, and this
    takes the TOP of that range so the keepout covers the worst draw. Section
    4(a).
    """
    return max(float(cfg["apron_min_m"]),
               float(cfg["apron_spread"]) * float(rec["H"]))


def standoff_m(rec, side, cfg):
    """The HARD keepout from one elevation of one burning building.

    A burning elevation gets the falling-glass zone (`glass_fall_frac * H`);
    a burning elevation on a partially collapsed building gets whichever of
    that and the debris run-out is larger; anything else gets a nominal wall
    clearance. All scaled by `standoff_scale`. Section 4.
    """
    H = float(rec["H"])
    fire_side = side in (rec.get("sides") or ())
    if fire_side:
        d = float(cfg["glass_fall_frac"]) * H
        if str(rec.get("level")) in cfg["collapse_levels"]:
            d = max(d, apron_run_m(rec, cfg))
    else:
        d = min(float(cfg["wall_clear_max_m"]),
                max(float(cfg["wall_clear_min_m"]),
                    float(cfg["wall_clear_frac"]) * H))
    return min(float(cfg["standoff_max_m"]), d * float(cfg["standoff_scale"]))


def building_standoff_m(rec, cfg):
    """The worst standoff over this building's elevations — the radius a
    street group is placed OUTSIDE of."""
    return max(standoff_m(rec, s, cfg) for s in ("N", "E", "S", "W"))


def wind_vectors(heading_deg):
    """`(downwind, upwind)` unit vectors. `heading_deg` is the direction the
    wind BLOWS TOWARD, +X = 0 — `urban_fire_spread._wind_factor`'s own
    convention, and `urban_fire_spread.bearing`'s frame."""
    a = math.radians(float(heading_deg))
    d = (math.cos(a), math.sin(a))
    return d, (-d[0], -d[1])


# ===========================================================================
# Openings
# ===========================================================================
def _face_point(fr, u, v, out=0.0):
    """`quake_flow._b_face_pt`, reproduced exactly: the world point at `u`
    along a wall frame at world height `v`, pushed `out` metres along the
    piece's OUTWARD normal (local -Y, world `(sin yaw, -cos yaw)`)."""
    ox, oy, yaw, width, height, depth, dw = fr
    ca, sa = math.cos(yaw), math.sin(yaw)
    if dw:
        u = u - width / 2.0
    d = -depth
    return (ox + ca * u + sa * (d + out), oy + sa * u - ca * (d + out), v)


def place_frame(fr, dx, dy, yaw_deg):
    """One opening frame moved from the bake's own origin frame into its city
    cell — `fire_bake.place`'s frame branch, for a single frame.

    Rotate `(ox, oy)` about the origin by the cell yaw, add the yaw (in
    RADIANS, which is what `fr[2]` already is) and translate. Everything else
    in the 7-tuple is a length or a flag and does not move.
    """
    fr = list(fr)
    ox, oy = _rot(float(fr[0]), float(fr[1]), yaw_deg)
    fr[0] = ox + dx
    fr[1] = oy + dy
    fr[2] = float(fr[2]) + math.radians(yaw_deg)
    return tuple(fr[:6]) + (bool(fr[6]),)


def _side_ops(doc, side):
    """Every measured opening on one elevation of a bake, in the bake's own
    frame: `[(fr, u0, u1, z_sill, z_head, storey)]`."""
    out = []
    for ev in (doc.get("events") or []):
        for op in (ev.get("ops") or []):
            if str(op.get("side") or ev.get("side")) != side:
                continue
            span = op.get("span") or ()
            if len(span) < 4:
                continue
            out.append((tuple(op["fr"]), float(span[0]), float(span[1]),
                        float(span[2]), float(span[3]),
                        int(op.get("storey", ev.get("storey", 0)))))
    return out


def openings_for_side(rec, side, storey, cfg, doc=None):
    """Candidate window openings on one elevation at one storey.

    Returns `[{u, u0, u1, z_sill, z_head, fr, source}]` in the CITY frame
    (already through `place_frame`) when a sidecar is available, or in a
    synthesised frame otherwise.

    THE SIDECAR ONLY RECORDS OPENINGS THE FIRE VENTED THROUGH (section 5a),
    and this module wants the storeys above the fire, where there are none.
    So a sidecar's ops on this side are used to MEASURE the real grid — bay
    pitch (median spacing of opening centres), opening width, and sill/head
    heights relative to their own storey floor — and that grid is replicated
    at `storey`. `source` says which happened, and `derived` records carry
    `needs_bench`.
    """
    period = storey_period(rec, doc)
    floor_z = storey * period
    yaw = float(rec.get("yaw_deg", 0.0))
    ops = _side_ops(doc, side) if doc else []

    if ops:
        # Measured grid: pitch from the centres, extents from the widest
        # opening, sill/head from their own storey floor.
        centres = sorted((o[1] + o[2]) / 2.0 for o in ops)
        gaps = [b - a for a, b in zip(centres, centres[1:]) if b - a > 0.4]
        pitch = (sorted(gaps)[len(gaps) // 2] if gaps
                 else float(cfg["bay_pitch_m"]))
        width = max(0.5, sum(o[2] - o[1] for o in ops) / len(ops))
        sill_rel = sum(o[3] - o[5] * period for o in ops) / len(ops)
        head_rel = sum(o[4] - o[5] * period for o in ops) / len(ops)
        fr = place_frame(ops[0][0], float(rec["x"]), float(rec["y"]), yaw)
        u_lo, u_hi = min(o[1] for o in ops), max(o[2] for o in ops)
        source = "sidecar_grid"
    else:
        # No sidecar: a synthetic frame for this elevation, built the same
        # way `gac_fire.side_frame` builds one — origin at the left end of
        # the face, `yaw` the in-plane direction, `depth` zero (the frame IS
        # the outer face), `dw` False.
        cx, cy, half = face_center(rec, side)
        nx, ny = side_normal_world(side, yaw)
        # In-plane direction: the outward normal turned +90 deg.
        tx, ty = -ny, nx
        fr = (cx - tx * half, cy - ty * half, math.atan2(ty, tx),
              2.0 * half, float(rec["H"]), 0.0, False)
        # `_face_point` pushes along `(sin yaw, -cos yaw)` = `(-tx... )`;
        # solve the sign once here rather than trusting it.
        px, py, _ = _face_point(fr, half, 0.0, 1.0)
        if (px - cx) * nx + (py - cy) * ny < 0.0:
            fr = (cx + tx * half, cy + ty * half,
                  math.atan2(-ty, -tx), 2.0 * half, float(rec["H"]), 0.0,
                  False)
        pitch = float(cfg["bay_pitch_m"])
        width = float(cfg["opening_w_m"])
        sill_rel = float(cfg["sill_h_m"])
        head_rel = float(cfg["head_h_m"])
        u_lo, u_hi = 0.0, 2.0 * half
        source = "derived"

    out = []
    span = max(0.0, u_hi - u_lo)
    n = max(1, int(span / max(1.0, pitch)))
    for k in range(n):
        u = u_lo + (k + 0.5) * span / n
        out.append({"u": u, "u0": u - width / 2.0, "u1": u + width / 2.0,
                    "z_sill": floor_z + sill_rel, "z_head": floor_z + head_rel,
                    "fr": fr, "source": source, "floor_z": floor_z})
    return out


# ===========================================================================
# Burial arithmetic — the tornado conventions, reproduced
# ===========================================================================
def lying_lift(pose, height_m=NOMINAL_HEIGHT_M, depth_m=None):
    """Metres to raise a laid-down rig so its body rests ON the surface.

    `people._lying_lift`, reproduced so the planner needs no import: half the
    body DEPTH face-up/face-down, `0.115 * H` on a side (biacromial breadth
    0.245 H, hips 0.191 H, shoulder flesh compresses — Drillis & Contini
    1966). `tests/test_fire_people.py` asserts this agrees with the real
    function.
    """
    if str(pose) in LYING_SPIN:
        return _LATERAL_HALF_BREADTH_H * float(height_m)
    return 0.5 * float(depth_m if depth_m is not None
                       else 2.0 * _BODY_HALF_DEPTH_M)


def apron_surface_z(t, depth_m):
    """The windrow's own top surface at `t` (0 at the wall, 1 at the
    run-out edge). `fire_collapse` authors it through `quake_flow._heap`,
    which is a mound: deepest at the wall line, tailing to grade. The 1.3
    exponent makes the outer quarter — the only part this module places
    anybody on — a shallow tail rather than a cliff."""
    return max(0.0, float(depth_m)) * max(0.0, 1.0 - float(t)) ** 1.3


def window_protrusion_m(variant, inset_m, height_m=NOMINAL_HEIGHT_M):
    """How far past the facade plane the furthest body part reaches, given
    the pelvis `inset_m` behind that plane.

    Section 5a. Both are the FORWARD REACH of the pose from the pelvis, less
    the inset — the inset is the ONLY term that positions the body, and
    charging a body-depth correction on top of it double-counts (see
    `lean_out_inset_m`).

      `sill_sit`  the thigh reach, `0.273 H` forward of the pelvis, with the
                  shins hanging below and outside it.
      `lean_out`  the crown, carried `(_CROWN_H - _HIP_H) H sin(28 deg)`
                  forward by `stand_slump`'s trunk pitch (spine_01 -10,
                  spine_02 -12, spine_03 -6).
    """
    H = float(height_m)
    if variant == "sill_sit":
        return _THIGH_L * H - float(inset_m)
    return ((_CROWN_H - _HIP_H) * H * math.sin(math.radians(28.0))
            - float(inset_m))


# ===========================================================================
# The planner
# ===========================================================================
def resolve_cfg(cfg=None):
    """`DEFAULTS` merged with a caller's overrides, one level deep for the
    nested dicts so a caller can move one share without restating them all."""
    out = dict(DEFAULTS)
    out["shares"] = dict(DEFAULTS["shares"])
    out["group_sizes"] = dict(DEFAULTS["group_sizes"])
    for k, v in (cfg or {}).items():
        if k in ("shares", "group_sizes") and isinstance(v, dict):
            out[k].update(v)
        else:
            out[k] = v
    return out


class _Building(object):
    """One burning building, with everything the passes ask of it computed
    once. Wraps a manifest record and (optionally) its bake sidecar."""

    def __init__(self, rec, cfg, doc=None):
        self.rec = rec
        self.doc = doc
        self.i = int(rec.get("i", -1))
        self.cell = rec.get("cell")
        self.x = float(rec["x"])
        self.y = float(rec["y"])
        self.W = float(rec["W"])
        self.D = float(rec["D"])
        self.H = float(rec["H"])
        self.yaw = float(rec.get("yaw_deg", 0.0))
        self.level = str(rec.get("level"))
        self.sides = tuple(rec.get("sides") or ())
        self.n_storeys = max(1, int(rec.get("n_storeys") or 1))
        self.typology = rec.get("typology")
        self.band_top = band_top(rec, doc)
        self.roof_involved = roof_involved(rec, doc)
        self.roof_collapsed = roof_collapsed(rec, doc)
        self.deck_z, self.deck_source = deck_z(rec, doc,
                                               cfg["parapet_est_m"])
        self.standoff = building_standoff_m(rec, cfg)
        self.apron_run = apron_run_m(rec, cfg)
        self.radius = 0.5 * math.hypot(self.W, self.D)

    # -- the three eligibility questions the 3-D passes ask ---------------
    def window_storeys(self, cfg=None):
        """Storeys strictly above the fire band AND below the drone's own
        ceiling. Empty for F4 and worse, by `urban_fire.BAND` — section 5a —
        and empty for a building whose fire band already tops the search
        altitude band (`window_max_z_m`)."""
        st = list(range(self.band_top + 1, self.n_storeys))
        if cfg is None:
            return st
        period = storey_period(self.rec, self.doc)
        zmax = float(cfg["window_max_z_m"])
        return [s for s in st if s * period + float(cfg["sill_h_m"]) <= zmax]

    def roof_ok(self, cfg):
        """`(ok, reason)` for a roof-refuge group."""
        if self.level not in cfg["roof_ok_levels"]:
            return False, "roof_deck_involved({0})".format(self.level)
        if self.roof_involved:
            return False, "band_reaches_top"
        if self.H > float(cfg["roof_max_h_m"]):
            return False, "too_tall({0:.0f}m)".format(self.H)
        return True, ""

    def collapse_sides(self):
        """The elevations a partial collapse could have taken. `fire_collapse`
        draws them from the VENTING sides (`plan_partial_collapse`'s own
        rule: the fire is venting on the elevation that failed), so those are
        the ones whose apron carries debris."""
        return tuple(self.sides) or ("S",)

    def far_sides(self):
        """The building's elevations RANKED away from the fire: most opposed
        to the mean venting normal first, and never a venting elevation.

        A ranking rather than a single answer because a deck may carry more
        than one group, and two groups on the same edge read as one cluster
        from above — the second goes to the next-best edge instead.
        """
        others = [s for s in ("N", "E", "S", "W") if s not in self.sides]
        if not others:
            others = ["N", "E", "S", "W"]
        if not self.sides:
            return others
        mx = sum(side_normal_world(s, self.yaw)[0] for s in self.sides)
        my = sum(side_normal_world(s, self.yaw)[1] for s in self.sides)
        mx, my = _norm(mx, my)
        return sorted(others, key=lambda s: _dot(
            side_normal_world(s, self.yaw), (mx, my)))

    def far_side(self, cfg=None):
        return self.far_sides()[0]


class Plan(object):
    """The output. `records` is the ground truth; everything else is
    provenance so a census can be printed without re-deriving anything."""

    def __init__(self, cfg, meta):
        self.cfg = cfg
        self.meta = meta
        self.records = []
        self.refused = {}
        self.degraded = {}
        self.dropped = {}
        self._group = 0

    def refuse(self, why):
        self.refused[why] = self.refused.get(why, 0) + 1

    def next_group(self):
        self._group += 1
        return self._group

    def add(self, rec):
        rec["id"] = len(self.records)
        self.records.append(rec)
        return rec


def _weighted(rng, table):
    """One entry from `[(value, weight), ...]`."""
    total = sum(w for _v, w in table)
    r = rng.random() * total
    acc = 0.0
    for v, w in table:
        acc += w
        if r <= acc:
            return v
    return table[-1][0]


def _occlusion(rng):
    """`(pattern, covered_frac)` from the light-end vocabulary."""
    pat = _weighted(rng, [(p, w) for p, _c, w in OCCLUSION])
    frac = dict((p, c) for p, c, _w in OCCLUSION)[pat]
    return pat, frac


def _pick_human(rng, pose, allow_posed=True, humans=None, posed=None):
    """A character for this pose.

    THE THREE POSED STATICS HAVE NO SKELETON. Handing one a pose does nothing
    and ships the figure upright, silently — `place-people-in-scenes` says so
    twice — so they are only ever offered where a plain stander is wanted.
    """
    rigged = list(humans or RIGGED_HUMANS)
    statics = list(posed if posed is not None else POSED_HUMANS)
    if allow_posed and statics and pose in ("idle", None) \
            and rng.random() < 0.35:
        return rng.choice(statics), None, False
    return rng.choice(rigged), pose, True


class _Solver(object):
    """Holds the indices every pass shares: the building footprints, the
    ground sampler, the people already placed, and the wind."""

    def __init__(self, dump, manifest, cfg, rng, layout=None, sidecars=None):
        self.cfg = cfg
        self.rng = rng
        self.dump = dump
        self.manifest = manifest
        self.ground, self.layout = make_ground(dump, layout,
                                               cfg["sidewalk_width_m"])
        rm = dump.get("region_m") or [500.0, 500.0]
        self.region = (-float(rm[0]) / 2.0, -float(rm[1]) / 2.0,
                       float(rm[0]) / 2.0, float(rm[1]) / 2.0)

        # EVERY house in the city is a footprint keepout, not just the
        # burning ones — a bystander standing inside an untouched office
        # block is as invisible as one inside a burning one.
        self.footprints = []
        for p in dump.get("placements") or []:
            self.footprints.append((float(p["x_m"]), float(p["y_m"]),
                                    float(p["W"]), float(p["D"]),
                                    float(p.get("yaw_deg", 0.0))))

        sc = sidecars or {}
        self.buildings = []
        for rec in manifest.get("records") or []:
            doc = sc.get(str(rec.get("cell"))) or sc.get(_bake_tag(rec))
            self.buildings.append(_Building(rec, cfg, doc))

        self.down, self.up = wind_vectors(cfg["heading_deg"])
        self.placed = []          # [(x, y)] for the spacing test

    # -- gates -----------------------------------------------------------
    def in_region(self, x, y, pad=1.0):
        x0, y0, x1, y1 = self.region
        return x0 + pad <= x <= x1 - pad and y0 + pad <= y <= y1 - pad

    def in_any_footprint(self, x, y, margin=None):
        m = self.cfg["footprint_margin_m"] if margin is None else margin
        for (cx, cy, W, D, yaw) in self.footprints:
            if point_in_obb(x, y, cx, cy, W, D, yaw, m):
                return True
        return False

    def clear_of_aprons(self, x, y):
        """Outside every burning building's own hard standoff, on every one
        of its elevations. Section 4(a)/(b)."""
        for b in self.buildings:
            for side in ("N", "E", "S", "W"):
                need = standoff_m(b.rec, side, self.cfg)
                cx, cy, half = face_center(b.rec, side)
                nx, ny = side_normal_world(side, b.yaw)
                # Only the half-space in front of this elevation is its
                # apron; a point round the back is another side's problem.
                if (x - cx) * nx + (y - cy) * ny <= 0.0:
                    continue
                if dist_to_obb(x, y, b.x, b.y, b.W, b.D, b.yaw) < need:
                    return False
        return True

    def near_enough(self, b, x, y):
        """Is this point still close enough to `b` to read as ITS crowd?

        THE CAP HAS TO BE TESTED ON EVERY MEMBER, NOT JUST THE GROUP SEED.
        Members are scattered up to a grown `group_radius_m` off the seed and
        the surface nudge can move one 8 m further, so a seed placed exactly
        at the limit produced a figure 61.9 m out against a 60 m cap — the
        rule held where it was enforced and not where it was checked.
        """
        return dist_to_obb(x, y, b.x, b.y, b.W, b.D, b.yaw) <= \
            float(self.cfg["max_wall_dist_m"])

    def spaced(self, x, y):
        s = float(self.cfg["min_sep_m"])
        return all((x - px) ** 2 + (y - py) ** 2 >= s * s
                   for px, py in self.placed)

    def surface_ok(self, x, y, classes=("road", "sidewalk", "paved")):
        return self.ground.at(x, y) in classes

    def street_point(self, b, band, upwind_cos, plan, want_los=True):
        """One legal street position beside burning building `b`.

        Rejection sampling, and every refusal is TALLIED by name so a class
        that comes back short says why. The order is cheapest test first.
        """
        cfg = self.cfg
        tries = int(cfg["max_tries"])
        relaxed = False
        for k in range(tries):
            # Relax the upwind requirement in the last third: when a
            # building's venting elevations all face upwind there is no
            # position that is both upwind and in line of sight, and a
            # crosswind crowd is the honest answer. Recorded per record.
            cos_need = upwind_cos
            if k > tries * 0.66:
                cos_need = -1.0
                relaxed = True
            # DRAW THE BEARING INSIDE A BURNING ELEVATION'S CONE rather
            # than uniformly over the circle and rejecting three quarters of
            # it: the line-of-sight rule is a hard requirement of the class,
            # not a filter, and sampling it directly leaves the refusal tally
            # describing the interesting failures (downwind, apron, surface)
            # instead of drowning them.
            if want_los and b.sides:
                s = self.rng.choice(list(b.sides))
                nx, ny = side_normal_world(s, b.yaw)
                spread = math.radians(float(cfg["los_half_angle_deg"]))
                ang = math.atan2(ny, nx) + self.rng.uniform(-spread, spread)
            else:
                ang = self.rng.uniform(0.0, 2.0 * math.pi)
            ux, uy = math.cos(ang), math.sin(ang)
            if ux * self.up[0] + uy * self.up[1] < cos_need:
                plan.refuse("downwind")
                continue
            if want_los and not self._sees_fire(b, ux, uy):
                plan.refuse("no_line_of_sight")
                continue
            # MEASURED FROM THE WALL, not the centre — see `obb_exit_m` —
            # and capped, so a 300 m tower's standoff cannot throw a figure
            # onto the next block but one (see `max_wall_dist_m`).
            off = min(float(cfg["max_wall_dist_m"]),
                      b.standoff * self.rng.uniform(*band))
            d = obb_exit_m(b.x, b.y, b.W, b.D, b.yaw, ux, uy) + off
            x, y = b.x + ux * d, b.y + uy * d
            if not self.in_region(x, y):
                plan.refuse("off_plate")
                continue
            if self.in_any_footprint(x, y):
                plan.refuse("in_footprint")
                continue
            if not self.clear_of_aprons(x, y):
                plan.refuse("in_apron")
                continue
            if not self.surface_ok(x, y):
                plan.refuse("wrong_surface")
                continue
            if not self.spaced(x, y):
                plan.refuse("too_close")
                continue
            return x, y, relaxed
        return None

    def _sees_fire(self, b, ux, uy):
        """Is the bearing `(ux, uy)` from the building's centre inside the
        line-of-sight cone of one of its burning elevations?"""
        lim = math.cos(math.radians(float(self.cfg["los_half_angle_deg"])))
        for s in (b.sides or ("S",)):
            nx, ny = side_normal_world(s, b.yaw)
            if ux * nx + uy * ny >= lim:
                return True
        return False


def _bake_tag(rec):
    """`fire_bake.bake_tag`'s shape, for matching a manifest record to a
    sidecar when the sidecar carries no `city.cell`."""
    name = rec.get("asset") or rec.get("style") or "?"
    return "{0}_{1}_{2}_s{3}".format(rec.get("kind"), name,
                                     rec.get("level"), rec.get("seed"))


# ---------------------------------------------------------------------------
# The passes
# ---------------------------------------------------------------------------
def _group_size(cfg, cls, rng):
    lo, hi = cfg["group_sizes"][cls]
    return rng.randint(int(lo), int(hi))


def _street_group(sol, plan, cls, b, want, band, upwind_cos):
    """One cluster of `want` people beside `b`. Returns how many landed."""
    cfg, rng = sol.cfg, sol.rng
    seed = sol.street_point(b, band, upwind_cos, plan)
    if seed is None:
        return 0
    sx, sy, relaxed = seed
    gid = plan.next_group()
    r_lo, r_hi = cfg["group_radius_m"]
    made = 0
    mine = []
    for k in range(want):
        # EXPANDING RADIUS. A fixed 1.4-3.0 m disc round the seed fails often
        # against the same gates the seed passed (an apron edge, a kerb, a
        # footprint corner), and every failure that empties a group costs the
        # whole group — 264 `singleton_group_withdrawn` on the first tuned
        # run. Growing the disc keeps the cluster reading as a cluster while
        # letting a member step round the obstacle that refused it.
        for _try in range(64):
            if k == 0:
                x, y = sx, sy
            else:
                grow = 1.0 + 0.9 * (_try / 64.0)
                a = rng.uniform(0.0, 2.0 * math.pi)
                r = rng.uniform(r_lo, r_hi) * grow
                x, y = sx + math.cos(a) * r, sy + math.sin(a) * r
            if not sol.in_region(x, y) or sol.in_any_footprint(x, y):
                continue
            if not sol.clear_of_aprons(x, y) or not sol.spaced(x, y):
                continue
            if not sol.surface_ok(x, y) or not sol.near_enough(b, x, y):
                continue
            break
        else:
            plan.refuse("group_member")
            continue

        surf = sol.ground.at(x, y)
        # Surface preference: a fireground closes the street, so the
        # carriageway is legal — but weighted below the sidewalk. Section
        # 4(e). A road draw that landed on the sidewalk anyway is kept.
        want_better = (surf == "paved"
                       or (surf == "road"
                           and rng.random() > float(cfg["road_share"])))
        if want_better:
            x, y, surf = _nudge_surface(sol, x, y, surf, rng, near=b)

        pose = _weighted(rng, list(_CLASS_POSES[cls]))
        seat = "kerb" if pose == "sit_edge" else None
        z = float(cfg["kerb_z_m"]) if surf in ("sidewalk", "paved") else 0.0
        usd, pose, rigged = _pick_human(rng, pose,
                                        allow_posed=pose in ("idle", "walk"))
        # Face the fire — a crowd watches the building it came out of.
        yaw = math.degrees(math.atan2(b.y - y, b.x - x))
        d_wall = dist_to_obb(x, y, b.x, b.y, b.W, b.D, b.yaw)
        plan.add({
            "cls": cls, "group": gid, "usd": usd, "rigged": rigged,
            "x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
            "yaw_deg": round(yaw, 1), "pose": pose, "prone": False,
            "seat": seat, "z_mode": "ground", "surface": surf,
            "alive": True, "needs_bench": False,
            "building_i": b.i, "building_cell": b.cell,
            "building_level": b.level, "building_sides": list(b.sides),
            "d_wall_m": round(d_wall, 2),
            "standoff_m": round(b.standoff, 2),
            "collapse_zone_frac": round(
                d_wall / max(1e-6, float(cfg["collapse_zone_frac"]) * b.H), 3),
            "upwind_cos": round(_dot(_norm(x - b.x, y - b.y), sol.up), 3),
            "reason": ("{0} of a {1} building, {2}, {3:.0f} m from the wall"
                       " ({4})".format(
                           cls, b.level, "crosswind (no upwind position with "
                           "line of sight)" if relaxed else "upwind",
                           d_wall, surf)),
        })
        sol.placed.append((x, y))
        mine.append((plan.records[-1], (x, y)))
        made += 1
    # A LONE FIGURE IS UNFINDABLE. `place-people-in-scenes` says so in those
    # words, and the benchmark skill puts a person at ~13 px in a 946 px
    # frame at survey altitude. A street group that came back with one member
    # (every other candidate refused) is not a smaller group, it is a target
    # nobody will find — so it is withdrawn rather than shipped.
    if cls != "at_car" and len(mine) == 1:
        rec, pt = mine[0]
        plan.records.remove(rec)
        sol.placed.remove(pt)
        plan.refuse("singleton_group_withdrawn")
        return 0
    return made


def _nudge_surface(sol, x, y, surf, rng, near=None):
    """Step a placed figure toward a better-ranked surface without re-drawing
    it, so the group stays together. Several radii, because the kerb can be
    anywhere from a metre to a lane away; the original position is kept if
    nothing better is reachable."""
    rank = list(sol.cfg["surface_rank"])
    here = rank.index(surf) if surf in rank else len(rank)
    for step in (2.0, 3.2, 4.6, 6.0, 8.0):
        for _s in range(9):
            a = rng.uniform(0.0, 2.0 * math.pi)
            nx, ny = x + math.cos(a) * step, y + math.sin(a) * step
            s2 = sol.ground.at(nx, ny)
            if s2 not in rank or rank.index(s2) >= here:
                continue
            if sol.in_any_footprint(nx, ny) or not sol.clear_of_aprons(nx, ny):
                continue
            if not sol.spaced(nx, ny) or not sol.in_region(nx, ny):
                continue
            if near is not None and not sol.near_enough(near, nx, ny):
                continue
            return nx, ny, s2
    return x, y, surf


def _pass_street(sol, plan, cls, budget, band, upwind_cos):
    """`evacuee` / `onlooker`: groups spread over the burning buildings,
    deepest-burning first so the crowd is where the fire is worst."""
    if budget <= 0 or not sol.buildings:
        return 0
    order = sorted(sol.buildings,
                   key=lambda b: -float(b.rec.get("age_s") or 0.0))
    made = 0
    bi = 0
    guard = 0
    while made < budget and guard < budget * 6 + 24:
        guard += 1
        b = order[bi % len(order)]
        bi += 1
        want = min(_group_size(sol.cfg, cls, sol.rng), budget - made)
        made += _street_group(sol, plan, cls, b, want, band, upwind_cos)
    return made


def _pass_at_car(sol, plan, budget):
    """1-2 people beside a kerb parking bay on a burning building's upwind
    block. NO OCCUPANTS — see the module docstring on `vehicles.CABIN_RULES`
    and the downtown car pool."""
    if budget <= 0 or not sol.buildings:
        return 0
    cfg, rng = sol.cfg, sol.rng
    made = 0
    guard = 0
    while made < budget and guard < budget * 8 + 24:
        guard += 1
        b = rng.choice(sol.buildings)
        seed = sol.street_point(b, (1.0, 1.8),
                                float(cfg["upwind_cos_onlooker"]), plan)
        if seed is None:
            continue
        sx, sy, relaxed = seed
        # Walk toward the nearest carriageway and stop a bay's width in.
        bay = _nearest_bay(sol, sx, sy)
        if bay is None:
            plan.refuse("no_kerb_bay")
            continue
        bx, by, bear = bay
        gid = plan.next_group()
        want = min(_group_size(cfg, "at_car", rng), budget - made)
        for k in range(want):
            # Stand on the kerb side of the bay, not in the traffic lane.
            # 1.25 m apart, not `min_sep_m` exactly: the record's x/y are
            # rounded to 3 dp and a pair placed exactly at the limit can
            # round INSIDE it. Same class of bug the tornado module records
            # for its own 3 dp positions.
            off = 1.3 + 1.25 * k
            x = bx + math.cos(math.radians(bear + 90.0)) * off
            y = by + math.sin(math.radians(bear + 90.0)) * off
            if not sol.in_region(x, y) or sol.in_any_footprint(x, y) \
                    or not sol.clear_of_aprons(x, y) or not sol.spaced(x, y) \
                    or not sol.near_enough(b, x, y):
                plan.refuse("group_member")
                continue
            pose = _weighted(rng, list(_CLASS_POSES["at_car"]))
            seat = "car_sill" if pose == "sit_edge" else None
            usd, pose, rigged = _pick_human(rng, pose,
                                            allow_posed=pose == "idle")
            surf = sol.ground.at(x, y)
            d_wall = dist_to_obb(x, y, b.x, b.y, b.W, b.D, b.yaw)
            plan.add({
                "cls": "at_car", "group": gid, "usd": usd, "rigged": rigged,
                "x": round(x, 3), "y": round(y, 3),
                "z": round(float(cfg["kerb_z_m"])
                           if surf in ("sidewalk", "paved") else 0.0, 3),
                "yaw_deg": round(math.degrees(
                    math.atan2(b.y - y, b.x - x)), 1),
                "pose": pose, "prone": False, "seat": seat,
                "z_mode": "ground", "surface": surf, "alive": True,
                "needs_bench": False, "needs_car": True,
                "bay_xy": [round(bx, 2), round(by, 2)],
                "bay_bearing_deg": round(bear, 1),
                "building_i": b.i, "building_cell": b.cell,
                "building_level": b.level, "building_sides": list(b.sides),
                "d_wall_m": round(d_wall, 2),
                "standoff_m": round(b.standoff, 2),
                "collapse_zone_frac": round(
                    d_wall / max(1e-6,
                                 float(cfg["collapse_zone_frac"]) * b.H), 3),
                "reason": ("beside a kerb bay {0:.0f} m from a {1} building; "
                           "NO occupant — the downtown car pool's glass is "
                           "painted into the texture (vehicles.CABIN_RULES)"
                           .format(d_wall, b.level)),
            })
            sol.placed.append((x, y))
            made += 1
    return made


def _nearest_bay(sol, x, y):
    """`(x, y, bearing_deg)` of a kerb parking bay near `(x, y)`: a point on
    the carriageway `kerb_bay_offset_m` in from the nearest road/sidewalk
    boundary, with the bearing of the kerb line."""
    off = float(sol.cfg["kerb_bay_offset_m"])
    best = None
    for a in range(0, 360, 15):
        ux, uy = math.cos(math.radians(a)), math.sin(math.radians(a))
        prev = sol.ground.at(x, y)
        for step in range(1, 26):
            px, py = x + ux * step, y + uy * step
            cur = sol.ground.at(px, py)
            if prev in ("sidewalk", "paved") and cur == "road":
                bx, by = px + ux * off, py + uy * off
                if sol.ground.at(bx, by) != "road":
                    break
                d = step
                if best is None or d < best[0]:
                    best = (d, bx, by, (a + 90.0) % 360.0)
                break
            prev = cur
    return None if best is None else (best[1], best[2], best[3])


def _pass_window(sol, plan, budget):
    """Figures at window openings STRICTLY ABOVE the fire band, on venting
    and adjacent elevations. Section 5a."""
    cfg, rng = sol.cfg, sol.rng
    cands = []
    for b in sol.buildings:
        if not b.window_storeys():
            plan.refuse("no_storey_above_fire")
            continue
        if not b.window_storeys(cfg):
            plan.refuse("window_too_high")
            continue
        cands.append(b)
    if budget <= 0 or not cands:
        return 0
    per_cap = min(int(cfg["window_max_per_building_hard"]),
                  max(int(cfg["window_max_per_building"]),
                      int(math.ceil(budget / float(len(cands))))))
    made, guard = 0, 0
    per = {}
    # ONE FIGURE PER OPENING. `rng.choice` over a side's openings will draw
    # the same one twice on a building allowed three figures, and two rigs at
    # identical coordinates render as one — a target the ground truth counts
    # twice and a camera sees once.
    used_ops = set()
    while made < budget and guard < budget * 10 + 30:
        guard += 1
        b = rng.choice(cands)
        if per.get(b.i, 0) >= per_cap:
            continue
        storeys = b.window_storeys(cfg)
        # Weight the LOWER storeys above the fire: those are the floors the
        # smoke reaches first and the ones a ladder can reach.
        storey = storeys[min(len(storeys) - 1,
                             int(len(storeys) * rng.random() ** 1.6))]
        vent = list(b.sides) or ["S"]
        adj = [s for v in vent for s in _ADJACENT[v] if s not in vent]
        side = rng.choice(vent + adj) if adj else rng.choice(vent)
        ops = openings_for_side(b.rec, side, storey, cfg, b.doc)
        if not ops:
            plan.refuse("no_openings")
            continue
        op = rng.choice(ops)
        key = (b.i, side, storey, round(op["u"], 2))
        if key in used_ops:
            plan.refuse("opening_taken")
            continue

        w = op["u1"] - op["u0"]
        h = op["z_head"] - op["z_sill"]
        can_sit = (h >= float(cfg["sit_head_clear_m"])
                   and w >= float(cfg["sit_width_m"]))
        variant = ("sill_sit" if can_sit
                   and rng.random() < float(cfg["sill_sit_share"])
                   else "lean_out")
        inset = float(cfg["sill_inset_m"] if variant == "sill_sit"
                      else cfg["lean_out_inset_m"])
        prot = window_protrusion_m(variant, inset)
        if prot < MIN_PROTRUSION_M:
            plan.refuse("no_protrusion")
            continue

        # `out` in `_face_point` pushes along the frame's OUTWARD normal, so
        # a NEGATIVE `out` is the inward offset that puts the pelvis behind
        # the facade plane; the normal itself is `(sin yaw, -cos yaw)`,
        # straight off the frame — see `quake_flow._b_face_pt`.
        px, py, _z = _face_point(op["fr"], op["u"], 0.0, -inset)
        f_yaw = float(op["fr"][2])
        nx, ny = math.sin(f_yaw), -math.cos(f_yaw)
        if variant == "sill_sit":
            pose, z, z_mode, seat = "sit_edge", op["z_sill"], "sill", "sill"
        else:
            pose, z, z_mode, seat = ("stand_slump", op["floor_z"], "floor",
                                     None)
        # THE CEILING IS ENFORCED ON THE OPENING THAT WAS ACTUALLY CHOSEN,
        # not only on the storey pre-filter. `window_storeys(cfg)` screens
        # candidates with the DEFAULT sill height (`sill_h_m`), but with a
        # sidecar the sill comes from the bake's own MEASURED ops and can sit
        # metres higher up its storey — so the cheap filter passed a storey
        # whose real opening was above the drone ceiling and four records
        # shipped over it (2026-08-31, the first sidecar run). The pre-filter
        # keeps its job of choosing candidates cheaply; this is the rule.
        if z > float(cfg["window_max_z_m"]):
            plan.refuse("window_too_high")
            continue
        if not sol.spaced(px, py):
            plan.refuse("too_close")
            continue
        usd, pose, rigged = _pick_human(rng, pose, allow_posed=False)
        plan.add({
            "cls": "window", "group": plan.next_group(), "usd": usd,
            "rigged": rigged, "x": round(px, 3), "y": round(py, 3),
            "z": round(z, 3), "yaw_deg": round(math.degrees(
                math.atan2(ny, nx)), 1),
            "pose": pose, "prone": False, "seat": seat, "z_mode": z_mode,
            "alive": True,
            # `lean_out` clears the facade by ~0.17 m; that is at the edge of
            # useful and only a close render settles it. See section 5a.
            "needs_bench": (variant == "lean_out"
                            or op["source"] == "derived"),
            "variant": variant, "protrusion_m": round(prot, 3),
            "inset_m": inset, "sill_z": round(op["z_sill"], 3),
            "head_z": round(op["z_head"], 3), "floor_z": round(op["floor_z"], 3),
            "side": side, "storey": storey, "openings_source": op["source"],
            "building_i": b.i, "building_cell": b.cell,
            "building_level": b.level, "building_sides": list(b.sides),
            "band_top": b.band_top,
            "reason": ("{0} at a {1} opening on storey {2} of {3} ({4}), "
                       "side {5} ({6}), {7:.2f} m past the facade"
                       .format(variant, op["source"], storey,
                               b.n_storeys - 1, b.level, side,
                               "venting" if side in vent else "adjacent",
                               prot)),
        })
        used_ops.add(key)
        sol.placed.append((px, py))
        per[b.i] = per.get(b.i, 0) + 1
        made += 1
    return made


def _pass_roof(sol, plan, budget):
    """2-4 near the roof edge, away from the venting side, on a building
    whose deck is intact. Section 5b."""
    cfg, rng = sol.cfg, sol.rng
    cands = []
    for b in sol.buildings:
        ok, why = b.roof_ok(cfg)
        if ok:
            cands.append(b)
        else:
            plan.refuse("roof:" + why)
    if budget <= 0 or not cands:
        return 0
    made, guard = 0, 0
    used = {}
    # ROUND-ROBIN, tallest-first so the most legible roofs are used before
    # the marginal ones, and at most `roof_max_groups_per_building` groups on
    # any one deck.
    order = sorted(cands, key=lambda b: -b.H)
    bi = 0
    cap = int(cfg["roof_max_groups_per_building"])
    while made < budget and guard < budget * 8 + 24:
        guard += 1
        b = order[bi % len(order)]
        bi += 1
        if used.get(b.i, 0) >= cap:
            if all(used.get(c.i, 0) >= cap for c in order):
                plan.refuse("roof_decks_exhausted")
                break
            continue
        nth = used.get(b.i, 0)
        used[b.i] = nth + 1
        ranked = b.far_sides()
        side = ranked[nth % len(ranked)]
        gid = plan.next_group()
        want = min(_group_size(cfg, "roof", rng), budget - made)
        e_lo, e_hi = cfg["roof_edge_band_m"]
        # Local roof plan: the edge `side` runs across the building, and the
        # group stands `e` metres in from it.
        placed_here = 0
        for k in range(want):
          # ONE DRAW PER MEMBER IS NOT ENOUGH. Without a retry the first
          # real-city run put a single figure on each of three roofs (7
          # `group_member` refusals against a budget of 9): two members
          # landing within `min_sep_m` of each other on a small deck loses
          # one outright, and a roof group of one is the lone-figure problem
          # again. Same expanding retry the street groups use.
          for _try in range(48):
            e = rng.uniform(e_lo, e_hi)
            lx, ly = SIDE_NORMAL[side]
            # In the BUILDING'S OWN frame: `out` runs along the chosen edge's
            # normal and stops `e` metres short of the parapet; `along` runs
            # across that edge and is kept off the corners (0.72 of the half
            # width) so nobody stands on a corner coping.
            half_along = (b.W / 2.0) if side in ("N", "S") else (b.D / 2.0)
            half_out = (b.D / 2.0) if side in ("N", "S") else (b.W / 2.0)
            along = rng.uniform(-0.72, 0.72) * half_along
            out = max(0.4, half_out - e)
            if side in ("N", "S"):
                ox, oy = along, ly * out
            else:
                ox, oy = lx * out, along
            dx, dy = _rot(ox, oy, b.yaw)
            x, y = b.x + dx, b.y + dy
            if not sol.in_region(x, y) or not sol.spaced(x, y):
                continue
            break
          else:
            plan.refuse("group_member")
            continue
          if True:            # keeps the member body at one indent level
            pose = _weighted(rng, list(_CLASS_POSES["roof"]))
            usd, pose, rigged = _pick_human(
                rng, pose, allow_posed=pose in ("idle", "walk"))
            # Look out over the parapet.
            nx, ny = side_normal_world(side, b.yaw)
            plan.add({
                "cls": "roof", "group": gid, "usd": usd, "rigged": rigged,
                "x": round(x, 3), "y": round(y, 3), "z": round(b.deck_z, 3),
                "yaw_deg": round(math.degrees(math.atan2(ny, nx)), 1),
                "pose": pose, "prone": False,
                "seat": None, "z_mode": "deck", "alive": True,
                "needs_bench": False,
                "deck_source": b.deck_source, "roof_edge_m": round(e, 2),
                "side": side,
                "building_i": b.i, "building_cell": b.cell,
                "building_level": b.level, "building_sides": list(b.sides),
                "reason": ("roof refuge on the {0} edge ({1} m in from the "
                           "parapet) of a {2} building, deck z {3:.1f} m "
                           "({4}); venting sides {5}"
                           .format(side, round(e, 1), b.level, b.deck_z,
                                   b.deck_source, ",".join(b.sides) or "-")),
            })
            sol.placed.append((x, y))
            placed_here += 1
            made += 1
        # A ROOF GROUP OF ONE IS THE LONE-FIGURE PROBLEM. Withdraw it, the
        # same way a street group is withdrawn, so a deck either reads as a
        # group of survivors or carries nobody.
        if placed_here == 1:
            rec = plan.records[-1]
            plan.records.remove(rec)
            sol.placed.pop()
            plan.refuse("singleton_roof_withdrawn")
            made -= 1
    return made


def _burial_record(sol, plan, b, cls, side, t, gid, note):
    """One partly-buried figure at `t` along a collapsed elevation's windrow.
    Shared by `casualty_apron` and `roof_debris` — the burial mechanics are
    the same, only the material differs. Section 5c."""
    cfg, rng = sol.cfg, sol.rng
    cx, cy, half = face_center(b.rec, side)
    nx, ny = side_normal_world(side, b.yaw)
    tx, ty = -ny, nx
    run = b.apron_run
    along = rng.uniform(-0.78, 0.78) * half
    d = run * t
    x = cx + nx * d + tx * along
    y = cy + ny * d + ty * along
    if not sol.in_region(x, y):
        plan.refuse("off_plate")
        return None
    # NEVER INSIDE THE STANDING SHELL. `tornado_people`'s `in_wreck`: a body
    # in the deepest material is invisible from every angle.
    if sol.in_any_footprint(x, y, margin=0.0):
        plan.refuse("in_footprint")
        return None
    if not sol.spaced(x, y):
        plan.refuse("too_close")
        return None

    pose = _weighted(rng, list(_LYING_POSES))
    pattern, frac = _occlusion(rng)
    depth = rng.uniform(*cfg["out_depth_m"])
    surf = apron_surface_z(t, depth)
    usd, pose, _rigged = _pick_human(rng, pose, allow_posed=False)
    lift = lying_lift(pose)
    yaw = rng.uniform(0.0, 360.0)
    rec = plan.add({
        "cls": cls, "group": gid, "usd": usd, "rigged": True,
        "x": round(x, 3), "y": round(y, 3),
        # `z` IS THE SUPPORT SURFACE ON EVERY CLASS, without exception — the
        # debris top here, the ground/kerb for a stander, the sill for a
        # sitter, the deck for a roof group. It is NOT the authored prim z:
        # the lying LIFT (half the body depth, or 0.115 H on a side) is added
        # by `to_placements`, which is also where `people._human_placement`
        # would add it from the rig's MEASURED depth. Baking the lift in here
        # made `z` mean two different things in two classes and would have
        # been applied twice by any converter that trusted the contract.
        "z": round(surf, 3),
        "yaw_deg": round(yaw, 1), "pose": pose, "prone": True,
        "roll_deg": LYING_ROLL[pose], "pitch_deg": LYING_SPIN.get(pose, 0.0),
        "seat": None, "z_mode": "debris", "alive": False,
        # THE BENCH IS THE ONLY WAY TO CHECK ONE OF THESE. The tornado skill
        # is unambiguous and nothing in a 2-D dry run discharges it.
        "needs_bench": True,
        "occlusion": pattern, "covered_frac": frac,
        "apron_t": round(t, 3), "apron_run_m": round(run, 2),
        "debris_depth_m": round(depth, 2), "surface_z": round(surf, 3),
        "lift_m": round(lift, 3), "side": side,
        "building_i": b.i, "building_cell": b.cell,
        "building_level": b.level, "building_sides": list(b.sides),
        "reason": note,
    })
    sol.placed.append((x, y))
    return rec


def _pass_casualty_apron(sol, plan, budget):
    """Prone figures in the OUTER quarter of an F5c building's debris
    windrow, partially under it. Section 5c."""
    cfg, rng = sol.cfg, sol.rng
    cands = [b for b in sol.buildings
             if b.level in cfg["collapse_levels"]]
    if not cands:
        plan.refuse("no_collapsed_building")
    if budget <= 0 or not cands:
        return 0
    lo, hi = cfg["apron_band"]
    made, guard = 0, 0
    while made < budget and guard < budget * 10 + 24:
        guard += 1
        b = rng.choice(cands)
        side = rng.choice(list(b.collapse_sides()))
        gid = plan.next_group()
        want = 2 if rng.random() < float(cfg["cluster_chance"]) else 1
        want = min(want, budget - made)
        for _k in range(want):
            t = rng.uniform(lo, hi)
            r = _burial_record(
                sol, plan, b, "casualty_apron", side, t, gid,
                "in the outer quarter of the {0}-elevation debris windrow "
                "of a partially collapsed ({1}) building, {2:.0f} % of the "
                "run-out from the wall".format(side, b.level, 100.0 * t))
            if r is not None:
                made += 1
    return made


def _pass_roof_debris(sol, plan, budget):
    """A rare figure under ROOF-DECK material that came out over a failed
    elevation.

    REFUSES A BUILDING WITH FOUR WALLS STANDING. An F5/F6 shell dropped its
    floors and its deck INWARD, where no camera goes — placing a figure there
    would be `people.py`'s retired `exposed_interior` under a new name.
    Section 5c.
    """
    cfg, rng = sol.cfg, sol.rng
    cands = []
    for b in sol.buildings:
        if not b.roof_collapsed:
            continue
        if b.level not in cfg["collapse_levels"]:
            # Roof down but every wall up: the debris is indoors.
            plan.refuse("roof_debris_indoors")
            continue
        cands.append(b)
    if not cands:
        plan.refuse("no_roof_collapse_building")
    if budget <= 0 or not cands:
        return 0
    made, guard = 0, 0
    while made < budget and guard < budget * 12 + 20:
        guard += 1
        b = rng.choice(cands)
        side = rng.choice(list(b.collapse_sides()))
        t = rng.uniform(*cfg["roof_debris_band"])
        r = _burial_record(
            sol, plan, b, "roof_debris", side, t, plan.next_group(),
            "under roof-deck material that came out over the failed {0} "
            "elevation of a {1} building".format(side, b.level))
        if r is not None:
            made += 1
    return made


# ---------------------------------------------------------------------------
# The aerial-visibility filter
# ---------------------------------------------------------------------------
def _visible(sol, rec):
    """`(ok, why)` — section 6. Applied to every record; the false ones are
    DROPPED, not shipped with a flag, because an unlabelable target in the
    ground truth is worse than a missing one."""
    cls = rec["cls"]
    if cls == "window":
        if float(rec.get("protrusion_m", 0.0)) < MIN_PROTRUSION_M:
            return False, "window_no_protrusion"
        return True, ""
    if cls == "roof":
        if rec.get("z_mode") != "deck":
            return False, "roof_not_on_deck"
        return True, ""
    if cls in ("casualty_apron", "roof_debris"):
        if float(rec.get("covered_frac", 0.0)) > MAX_COVERED_FRAC:
            return False, "over_covered"
        if sol.in_any_footprint(rec["x"], rec["y"], margin=0.0):
            return False, "under_intact_shell"
        return True, ""
    # street classes
    if sol.in_any_footprint(rec["x"], rec["y"]):
        return False, "in_footprint"
    if rec.get("surface") not in ("road", "sidewalk", "paved"):
        return False, "wrong_surface"
    return True, ""


# ---------------------------------------------------------------------------
# The entry point
# ---------------------------------------------------------------------------
def plan_people(dump, manifest, seed=0, cfg=None, layout=None,
                sidecars=None, heading_deg=None):
    """Plan the whole population. Returns a `Plan`.

    `dump`      the FC city placements dump (`load_dump`)
    `manifest`  the fire manifest (`load_manifest`)
    `sidecars`  `{cell_or_tag: doc}` from `load_sidecars`, or None
    `layout`    a real `city_layout` if the caller has one; else derived
    `heading_deg` the wind direction, TOWARD; else the cfg default
    """
    cfg = resolve_cfg(cfg)
    if heading_deg is not None:
        cfg["heading_deg"] = float(heading_deg)
    rng = random.Random(int(seed))

    sol = _Solver(dump, manifest, cfg, rng, layout=layout, sidecars=sidecars)
    n_burn = len(sol.buildings)
    total = cfg["total"]
    if total is None:
        total = int(round(float(cfg["per_building"]) * n_burn))
    total = max(int(cfg["total_min"]), min(int(cfg["total_max"]), int(total)))

    meta = {
        "seed": int(seed),
        "manifest_seed": manifest.get("seed"),
        "preset": manifest.get("preset") or dump.get("preset"),
        "epoch_s": manifest.get("epoch_s"),
        "heading_deg": cfg["heading_deg"],
        "n_burning": n_burn,
        # A manifest may carry more than one ignition point (`origins`, e.g.
        # the 2026-08-31 union of seed 4 + seed 35). NOTHING IN THIS MODULE
        # READS IT: every pass keys off a building's OWN `sides`, `level` and
        # `age_s`, so an N-origin manifest needs no generalisation — it is
        # just a longer record list with more than one `via` root. Carried
        # into the meta as provenance so a reader of the ground truth can see
        # which fire plan it was solved against.
        "origins": (manifest.get("origins")
                    or ([manifest["origin"]] if manifest.get("origin")
                        is not None else [])),
        "n_placements": len(dump.get("placements") or []),
        "region_m": dump.get("region_m"),
        "total_requested": total,
        "layout_source": sol.layout.get("_source", "caller-supplied layout"),
        "sidecars": len(sidecars or {}),
        "nominal_height_m": NOMINAL_HEIGHT_M,
    }
    plan = Plan(cfg, meta)

    # --- budgets, with degradation ------------------------------------
    shares = dict(cfg["shares"])
    eligible = {
        "evacuee": n_burn > 0,
        "onlooker": n_burn > 0,
        "at_car": n_burn > 0,
        "window": any(b.window_storeys(cfg) for b in sol.buildings),
        "roof": any(b.roof_ok(cfg)[0] for b in sol.buildings),
        "casualty_apron": any(b.level in cfg["collapse_levels"]
                              for b in sol.buildings),
        "roof_debris": any(b.roof_collapsed
                           and b.level in cfg["collapse_levels"]
                           for b in sol.buildings),
    }
    give_back = 0.0
    for c in CLASSES:
        if not eligible.get(c):
            give_back += shares.get(c, 0.0)
            plan.degraded[c] = shares.get(c, 0.0)
            shares[c] = 0.0
    if give_back > 0.0:
        base = sum(shares[c] for c in _FALLBACK_CLASSES) or 1.0
        for c in _FALLBACK_CLASSES:
            shares[c] += give_back * shares[c] / base
    ssum = sum(shares.values()) or 1.0
    budget = {c: int(round(total * shares[c] / ssum)) for c in CLASSES}
    meta["budget"] = dict(budget)
    meta["shares_effective"] = {c: round(shares[c] / ssum, 4) for c in CLASSES}

    # --- the passes, in a fixed order ---------------------------------
    # Order matters only through the shared spacing index and the shared
    # rng: the 3-D classes go FIRST so a street group can never take a
    # position that a window or roof figure would have wanted (they cannot
    # move; a street group can).
    _pass_window(sol, plan, budget["window"])
    _pass_roof(sol, plan, budget["roof"])
    _pass_casualty_apron(sol, plan, budget["casualty_apron"])
    _pass_roof_debris(sol, plan, budget["roof_debris"])
    _pass_street(sol, plan, "evacuee", budget["evacuee"],
                 cfg["evacuee_band"], float(cfg["upwind_cos_evacuee"]))
    _pass_street(sol, plan, "onlooker", budget["onlooker"],
                 cfg["onlooker_band"], float(cfg["upwind_cos_onlooker"]))
    _pass_at_car(sol, plan, budget["at_car"])

    # --- REFLOW ---------------------------------------------------------
    # A class can be STARVED rather than degraded: `roof` has exactly one
    # eligible deck on the real seed-4 city, `window` only two eligible
    # buildings, so both spend less than their budget however hard they try.
    # Leaving the shortfall unspent shrinks the scene's head count for a
    # reason that has nothing to do with how many people were there — so it
    # is handed to the street classes, which always have somewhere to go.
    # Recorded, because "the roof class placed 3 of 9" is a fact a reviewer
    # needs and a silent top-up would hide it.
    short = total - len(plan.records)
    plan.meta["shortfall_before_reflow"] = short
    if short > 0:
        n_e = int(round(short * 0.6))
        _pass_street(sol, plan, "evacuee", n_e,
                     cfg["evacuee_band"], float(cfg["upwind_cos_evacuee"]))
        _pass_street(sol, plan, "onlooker", short - n_e,
                     cfg["onlooker_band"], float(cfg["upwind_cos_onlooker"]))

    # --- the aerial-visibility filter ---------------------------------
    kept = []
    for rec in plan.records:
        ok, why = _visible(sol, rec)
        rec["aerial_visible"] = bool(ok)
        if ok:
            kept.append(rec)
        else:
            plan.dropped[why] = plan.dropped.get(why, 0) + 1
    plan.records = kept
    for k, r in enumerate(plan.records):
        r["id"] = k
    plan.meta["total_placed"] = len(plan.records)
    plan.solver = sol
    return plan


# ---------------------------------------------------------------------------
# THE CONVERTER — records -> `scene_generator.apply_placements` dicts
# ---------------------------------------------------------------------------
# `apply_placements` reads exactly these keys off a placement: `usd`,
# `category`, `x_m`, `y_m`, `z_m`, `yaw_deg`, `roll_deg`, `pitch_deg`,
# `scale`, `axis_up`, and `pose` (which it passes to `_bind_human_pose` ONLY
# when truthy). It authors `translate -> rotateXYZ(roll, pitch, yaw) ->
# scale`, and it writes `prim_path` BACK into the dict it was given, which is
# the cheapest way for a caller to join an authored prim to the record it
# came from (order is preserved and the mapping is 1:1 over the kept records
# — see `to_placements`'s return).
# ---------------------------------------------------------------------------
def _placement_no_ctx(rec, height_m=NOMINAL_HEIGHT_M):
    """One placement dict WITHOUT a scene context — the host-side path.

    Reproduces `people._human_placement` with the pack's nominal constants in
    place of a measured rig:

      * the pose drop comes from `scene_generator.pose_z_offset`, which is
        importable host-side and degrades to its own stature-scaled
        `_POSE_Z_OFFSET` table when it cannot open the rig to measure a hip
        (it needs Nucleus for that; every host run lands on the table);
      * the lying lift comes from `lying_lift`, i.e. half a NOMINAL body
        depth rather than the rig's measured `sy`;
      * the resolver's `base` (the asset's own bbox floor) is 0 — these rigs
        are authored with their soles at the origin, which is the assumption
        `people._human_placement` also makes for everything but a prop.

    THE MEASURED PATH IS BETTER AND THE LAUNCHER HAS IT. Pass `ctx=` and the
    whole job is delegated to `people._human_placement`, which measures each
    character's stature and depth, solves ground poses against that rig's own
    hip, and applies the male seated correction (`_MALE_SEATED_DZ_M`, -0.15 m
    — "no amount of scaling fixes it"). This path exists so the converter is
    testable and previewable with no Isaac, and it says so.
    """
    pose = rec.get("pose")
    yaw = float(rec["yaw_deg"]) + HUMAN_YAW_OFFSET_DEG
    if rec.get("prone"):
        z = float(rec["z"]) + lying_lift(pose, height_m)
        roll = float(LYING_ROLL[str(pose)])
        pitch = float(LYING_SPIN.get(str(pose), 0.0))
    else:
        dz = 0.0
        if pose:
            try:
                import scene_generator as sg
                dz = float(sg.pose_z_offset(rec["usd"], pose, height_m))
            except Exception:
                dz = 0.0
        z = float(rec["z"]) + dz
        roll = pitch = 0.0
    out = {"usd": rec["usd"], "x_m": float(rec["x"]), "y_m": float(rec["y"]),
           "z_m": z, "yaw_deg": yaw, "roll_deg": roll, "pitch_deg": pitch,
           "scale": HUMAN_SCALE, "category": "human",
           "axis_up": HUMAN_AXIS_UP}
    if pose:
        out["pose"] = pose
    return out


def _convertible(rec, known_pool):
    """`None` if this record can be authored, else the SKIP REASON.

    Every one of these is a case `people._human_placement` either raises on
    or silently gets wrong, and the module's own rule is that a figure it
    cannot author correctly is DROPPED with a counted reason rather than
    authored wrong — the same discipline as the aerial-visibility filter.
    """
    pose = rec.get("pose")
    prone = bool(rec.get("prone"))
    if str(pose) in LYING_ROLL and not prone:
        # `people._human_placement` RAISES here, and it is right to: authored
        # upright, `lying_prone` is a standing figure with an arm stretched
        # over its head.
        return "lying_pose_not_prone"
    if prone and str(pose) not in LYING_ROLL:
        # The prone branch takes its roll SIGN from the pose. Without one it
        # would fall back to a coin flip on (x + y) and land face-down as
        # often as face-up.
        return "prone_without_lying_pose"
    if not rec.get("rigged"):
        # A POSED STATIC HAS NO SKELETON. Binding a pose does nothing and
        # ships the figure upright, silently; laying one down makes it a shop
        # dummy, because the prone lift assumes an arms-down rig.
        if pose not in (None, "idle"):
            return "static_with_pose"
        if prone:
            return "static_prone"
    if known_pool is not None and rec.get("usd") not in known_pool:
        # No scale / axis_up / yaw offset for an asset this module does not
        # know. With a `ctx` the asset pools answer for it, so this only
        # fires on the host path.
        return "unknown_asset"
    return None


def to_placements(plan_or_records, ctx=None, tag_ids=False,
                  height_m=NOMINAL_HEIGHT_M):
    """Solver records -> `scene_generator.apply_placements` placement dicts.

    ONE CALL FOR THE LAUNCHER:

        placements, skipped = fire_people.to_placements(plan, ctx=people_ctx)
        sg.apply_placements(stage, placements, parent_path=...,
                            scene_scale_factor=ssf, resolver=resolver,
                            instance_categories=set())

    **PASS `ctx` IF YOU HAVE ONE.** `ctx` is `disaster.people`'s own context
    (anything with `ctx["asset_pools"]` and `ctx["resolver"]`, e.g. what
    `people.build_ctx` returns): the conversion is then delegated verbatim to
    `people._human_placement`, which measures each character's stature and
    body depth, solves a ground pose against that rig's own hip, applies the
    male seated correction, and reads the per-asset `scale` / `axis_up` /
    `yaw-offset` off the compiled asset pool instead of this module's
    transcribed constants. Without a `ctx` the nominal path
    (`_placement_no_ctx`) runs — correct in shape, approximate in the
    centimetres that only a measurement can settle.

    **`z` IN A RECORD IS THE SUPPORT SURFACE, NOT THE PRIM z**, on every
    class: the ground or kerb top for a stander, the sill for a sill-sitter,
    the storey floor for a lean-out, the roof deck for a roof group, the
    debris top for a burial. The pose drop (`pose_z_offset`) and the lying
    lift are added HERE, exactly once. That is the whole reason the burial
    records stopped carrying a pre-lifted z.

    **`yaw_deg` IN A RECORD IS THE FACING BEARING** (+X = 0), and the pack's
    `yaw-offset: 90` is added here — `apply_placements` adds none of its own.
    See `HUMAN_YAW_OFFSET_DEG`.

    **A RECORD THAT CANNOT BE AUTHORED CORRECTLY IS SKIPPED, NOT FUDGED.**
    `_convertible` names each case; the second return value is
    `{reason: [record ids]}`. `needs_bench` is NOT one of them — those
    figures are placed and correct as far as arithmetic goes, and the bench
    is for looking at them.

    Returns `(placements, skipped)`. `placements[k]` is the conversion of the
    k-th KEPT record, in record order, so a caller can zip them; with
    `tag_ids=True` each dict also carries `fire_people_id` (the record's
    `id`), which `apply_placements` ignores. It is OFF by default so the
    dicts match the documented contract key-for-key.
    """
    recs = (plan_or_records.records if hasattr(plan_or_records, "records")
            else list(plan_or_records))
    known = None if ctx is not None else set(RIGGED_HUMANS) | set(POSED_HUMANS)
    placements, skipped = [], {}
    for rec in recs:
        why = _convertible(rec, known)
        if why:
            skipped.setdefault(why, []).append(rec.get("id"))
            continue
        if ctx is not None:
            from . import people as ppl
            p = ppl._human_placement(ctx, rec["usd"], rec["x"], rec["y"],
                                     rec["z"], rec["yaw_deg"],
                                     rec.get("pose"),
                                     prone=bool(rec.get("prone")))
            # `_human_placement` always writes `pose`, including `None` for a
            # posed static. `apply_placements` only calls `_bind_human_pose`
            # on a truthy one, so a None is harmless — but dropping it keeps
            # the dict to the documented contract either way.
            if not p.get("pose"):
                p.pop("pose", None)
        else:
            p = _placement_no_ctx(rec, height_m)
        if tag_ids:
            p["fire_people_id"] = rec.get("id")
        placements.append(p)
    return placements, skipped


# ---------------------------------------------------------------------------
# Census and rule checks — one implementation, used by the dry run AND the
# tests, so neither can pass against a different definition than the other.
# ---------------------------------------------------------------------------
def summarise(plan):
    recs = plan.records
    by_cls = {}
    for r in recs:
        by_cls[r["cls"]] = by_cls.get(r["cls"], 0) + 1
    groups = {}
    for r in recs:
        groups.setdefault((r["cls"], r["group"]), 0)
        groups[(r["cls"], r["group"])] += 1
    sizes = {}
    for (_c, _g), n in groups.items():
        sizes[n] = sizes.get(n, 0) + 1
    bench = sum(1 for r in recs if r.get("needs_bench"))
    vis = sum(1 for r in recs if r.get("aerial_visible"))
    czf = sorted(r["collapse_zone_frac"] for r in recs
                 if r.get("collapse_zone_frac") is not None)
    dw = sorted(r["d_wall_m"] for r in recs if r.get("d_wall_m") is not None)
    return {
        "total": len(recs),
        "by_class": by_cls,
        "locations": len(groups),
        "group_sizes": sizes,
        "needs_bench": bench,
        "aerial_visible": vis,
        "poses": _tally([dict(r, pose=(r.get("pose")
                                       or "posed_static(no pose)"))
                         for r in recs], "pose"),
        "surfaces": _tally([r for r in recs if r.get("surface")], "surface"),
        "openings_source": _tally(
            [r for r in recs if r.get("openings_source")], "openings_source"),
        "deck_source": _tally(
            [r for r in recs if r.get("deck_source")], "deck_source"),
        "window_variant": _tally(
            [r for r in recs if r.get("variant")], "variant"),
        "occlusion": _tally(
            [r for r in recs if r.get("occlusion")], "occlusion"),
        "d_wall_m": _quantiles(dw),
        "collapse_zone_frac": _quantiles(czf),
        "characters": len({r["usd"] for r in recs}),
        "rigged": sum(1 for r in recs if r.get("rigged")),
    }


def _tally(recs, key):
    out = {}
    for r in recs:
        out[str(r.get(key))] = out.get(str(r.get(key)), 0) + 1
    return dict(sorted(out.items(), key=lambda kv: -kv[1]))


def _quantiles(vals):
    if not vals:
        return None
    def q(p):
        return round(vals[min(len(vals) - 1, int(p * len(vals)))], 2)
    return {"min": round(vals[0], 2), "p10": q(0.10), "p50": q(0.50),
            "p90": q(0.90), "max": round(vals[-1], 2)}


def group_hi(cfg, cls):
    """The upper bound of a class's group size — used by `check_rules` and
    by the dry run, so both quote the same ceiling."""
    return int(cfg["group_sizes"][cls][1])


def check_rules(plan):
    """Every rule the model claims, re-derived from the records alone.

    Returns `[(name, ok, detail)]`. The dry run prints it and
    `tests/test_fire_people.py` asserts on it, so a rule cannot be "checked"
    in one place against a different definition than it is enforced with.
    """
    sol = plan.solver
    cfg = plan.cfg
    recs = plan.records
    out = []

    def add(name, bad, n_checked, note=""):
        out.append((name, not bad, {
            "violations": bad[:6], "n_violations": len(bad),
            "n_checked": n_checked, "note": note}))

    # 1. nobody in a building footprint, except the sanctioned classes
    bad = [r["id"] for r in recs
           if r["cls"] not in AERIAL_EXEMPT_CLASSES
           and sol.in_any_footprint(r["x"], r["y"], margin=0.0)]
    add("no_one_in_a_footprint", bad,
        len([r for r in recs if r["cls"] not in AERIAL_EXEMPT_CLASSES]),
        "window/roof are exempt by class and carry their own z")

    # 2. nobody inside a burning building's hard standoff (street classes)
    bad = [r["id"] for r in recs
           if r["cls"] in STREET_CLASSES and not sol.clear_of_aprons(
               r["x"], r["y"])]
    add("street_standoff_respected", bad,
        len([r for r in recs if r["cls"] in STREET_CLASSES]),
        "glass-fall zone 0.33H on fire sides; fire_collapse run-out on F5c")

    # 3. nobody in a collapse/debris apron's inner three quarters
    band_of = {"casualty_apron": cfg["apron_band"],
               "roof_debris": cfg["roof_debris_band"]}
    bad = [r["id"] for r in recs
           if r.get("apron_t") is not None
           and r["apron_t"] < band_of[r["cls"]][0] - 1e-3]
    add("burials_in_the_outer_band", bad,
        len([r for r in recs if r.get("apron_t") is not None]),
        "apron_band {0}, roof_debris_band {1}; never centred in the "
        "heap".format(cfg["apron_band"], cfg["roof_debris_band"]))

    # 4. nothing fully buried
    bad = [r["id"] for r in recs
           if float(r.get("covered_frac", 0.0)) > MAX_COVERED_FRAC + 1e-9]
    add("nothing_fully_buried", bad,
        len([r for r in recs if r.get("covered_frac") is not None]),
        "max_covered_frac {0}".format(MAX_COVERED_FRAC))

    # 5. every burial carries needs_bench
    bad = [r["id"] for r in recs
           if r["cls"] in ("casualty_apron", "roof_debris")
           and not r.get("needs_bench")]
    add("burials_flagged_for_the_bench", bad,
        len([r for r in recs if r["cls"] in ("casualty_apron", "roof_debris")]))

    # 6. every window figure breaks the facade plane
    bad = [r["id"] for r in recs if r["cls"] == "window"
           and float(r.get("protrusion_m", 0.0)) < MIN_PROTRUSION_M]
    add("windows_protrude", bad,
        len([r for r in recs if r["cls"] == "window"]),
        "min {0} m".format(MIN_PROTRUSION_M))

    # 7. window storeys are strictly above the fire band
    bad = [r["id"] for r in recs if r["cls"] == "window"
           and int(r.get("storey", -1)) <= int(r.get("band_top", 0))]
    add("windows_above_the_fire", bad,
        len([r for r in recs if r["cls"] == "window"]),
        "urban_fire.BAND: F4+ has no storey above its fire")

    # 8. roofs only on intact decks, and only on eligible buildings
    by_i = {b.i: b for b in sol.buildings}
    bad = []
    for r in recs:
        if r["cls"] != "roof":
            continue
        b = by_i.get(r["building_i"])
        if b is None or not b.roof_ok(cfg)[0]:
            bad.append(r["id"])
    add("roofs_on_intact_decks", bad,
        len([r for r in recs if r["cls"] == "roof"]),
        "levels {0}, H <= {1} m, band below the top storey".format(
            cfg["roof_ok_levels"], cfg["roof_max_h_m"]))

    # 9. group spacing
    bad = []
    pts = [(r["id"], r["x"], r["y"]) for r in recs]
    # The planner tests the UNROUNDED position and the record carries 3 dp,
    # so two figures placed exactly at the limit can round inside it by up to
    # 2 * 0.0005 * sqrt(2). Tolerate exactly that and no more.
    s = float(cfg["min_sep_m"]) - 0.0015
    for a in range(len(pts)):
        for b2 in range(a + 1, len(pts)):
            if (pts[a][1] - pts[b2][1]) ** 2 + (pts[a][2] - pts[b2][2]) ** 2 \
                    < s * s:
                bad.append((pts[a][0], pts[b2][0]))
    add("min_separation", bad, len(pts),
        "min_sep_m {0} (3 dp rounding tolerated)".format(cfg["min_sep_m"]))

    # 10. no lone figure in a street class
    counts = {}
    for r in recs:
        if r["cls"] in STREET_CLASSES:
            counts[(r["cls"], r["group"])] = counts.get(
                (r["cls"], r["group"]), 0) + 1
    bad = ["{0}/{1}".format(*k) for k, n in counts.items()
           if n < 2 and k[0] != "at_car"]
    add("street_groups_are_groups", bad, len(counts),
        "a lone figure is unfindable; at_car may be 1")

    # 11. on the plate
    bad = [r["id"] for r in recs if not sol.in_region(r["x"], r["y"], pad=0.0)]
    add("on_the_plate", bad, len(recs), "region {0}".format(sol.region))

    # 12. no banned pose, and no posed static carrying a pose
    bad = [r["id"] for r in recs if str(r.get("pose")) == "wave"]
    bad += [r["id"] for r in recs
            if not r.get("rigged") and r.get("pose") not in (None, "idle")]
    add("pose_rules", bad, len(recs),
        "people.BANNED_POSES; the three posed statics have no skeleton")

    # 12b. nobody is further from their own burning building than the scene
    #      can explain
    lim = float(cfg["max_wall_dist_m"]) + 1e-6
    bad = [r["id"] for r in recs
           if r.get("d_wall_m") is not None and float(r["d_wall_m"]) > lim]
    add("crowd_belongs_to_its_building", bad,
        len([r for r in recs if r.get("d_wall_m") is not None]),
        "max_wall_dist_m {0} m — a skyscraper's 0.33H zone is 100 m and "
        "would not read as this fire's crowd".format(cfg["max_wall_dist_m"]))

    # 13a. no window figure above the drone's own ceiling
    bad = [r["id"] for r in recs if r["cls"] == "window"
           and float(r["z"]) > float(cfg["window_max_z_m"]) + 1e-6]
    add("windows_under_the_drone_ceiling", bad,
        len([r for r in recs if r["cls"] == "window"]),
        "window_max_z_m {0} m — the benchmark flies 15-40 m AGL".format(
            cfg["window_max_z_m"]))

    # 13b. roof groups are spread over the eligible decks
    per = {}
    for r in recs:
        if r["cls"] == "roof":
            per[r["building_i"]] = per.get(r["building_i"], 0) + 1
    cap = group_hi(cfg, "roof") * int(cfg["roof_max_groups_per_building"])
    bad = ["{0}:{1}".format(i, n) for i, n in per.items() if n > cap]
    add("roof_groups_spread", bad, len(per),
        "at most {0} figures on any one deck ({1} group(s) of up to "
        "{2})".format(cap, cfg["roof_max_groups_per_building"],
                      group_hi(cfg, "roof")))

    # 14. every lying figure is laid down, with the pose's own roll
    bad = [r["id"] for r in recs
           if str(r.get("pose")) in LYING_ROLL
           and (not r.get("prone")
                or abs(float(r.get("roll_deg", 0.0))
                       - LYING_ROLL[str(r["pose"])]) > 1e-6)]
    add("lying_poses_are_laid_down", bad,
        len([r for r in recs if str(r.get("pose")) in LYING_ROLL]),
        "people._human_placement RAISES on a lying pose placed upright")

    return out


def write_records(path, plan):
    """Ground truth, in the envelope `people.write_records` uses (`meta` +
    `people`) so a reader that already handles `humans.json` can handle
    this."""
    doc = {"meta": dict(plan.meta), "people": plan.records,
           "census": summarise(plan),
           "refused": dict(plan.refused), "dropped": dict(plan.dropped),
           "degraded": dict(plan.degraded)}
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, "w") as fh:
        json.dump(doc, fh, indent=1)
    return path
