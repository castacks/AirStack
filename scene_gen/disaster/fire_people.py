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
| `window` | 0.16 | leaning out of a window opening, **strictly above the fire band**, venting or adjacent elevation, HIPS SUNK to the real sill height | torso and head past the facade plane; legs occluded by the wall below the sill |
| `roof` | 0.12 | 2-4 near the roof edge of a building whose deck is intact, on the side away from the venting elevations | the easiest aerial target in the scene, and the control case |
| `roof_victim` | see 5b2 | 2-3 near the roof edge of a building whose deck is intact, biased toward the STREET-facing edge, kept clear of the building's own smoke seats | a stranded figure, not a refuge crowd — the class that exists because `roof` is starved on tall real buildings |
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

**`sill_sit` WAS BACKWARDS AND HAS BEEN RETIRED (2026-08-31, user on sight:
"they need to be lower half inside the building and upper half outside").**
It seated the pelvis ON the sill, `SILL_INSET_M` INSIDE the facade, with
`sit_edge`'s thighs running forward 0.273 H — so the VISIBLE protruding part
was the dangling calves and feet (the least identifiable part of a person),
while the torso and head, which sit straight up from the pelvis, stayed
recessed inside the facade plane the whole time: a face in a dark hole with a
pair of legs hanging out from under it. That is the exact inverse of a
rescue-window read and of what the user asked for.

**THE POSE VOCABULARY DOES NOT CONTAIN A LEAN-OUT, so the single surviving
variant, `lean_out`, is built out of what `scene_generator._HUMAN_POSES`
actually has**, and it is measured rather than asserted (segment fractions of
stature from Drillis & Contini 1966, the same source
`people._LATERAL_HALF_BREADTH_H` uses):

* **`lean_out`** — `stand_slump`, HIPS SUNK TO THE REAL SILL HEIGHT
  (`op["z_sill"]`, the sidecar's own measured or lifted-grid opening — never
  a guessed constant): `z` (the feet, the record's support surface) is
  `z_sill - _HIP_H * H`, so the pelvis lands exactly at the sill. The pelvis
  sits `lean_out_inset_m` (half a body depth) BEHIND the facade plane, and
  since the legs hang almost straight down from it, the calves and feet stay
  at that same recessed position — INSIDE the building, below the sill,
  where the intact wall panel occludes them. `stand_slump`'s 28 deg forward
  trunk pitch (spine_01 -10, spine_02 -12, spine_03 -6) then carries the
  crown `0.470 H * sin(28 deg) = 0.221 H` forward of the pelvis, i.e.
  ~0.39 m, and the acromion ~0.24 m — so with the pelvis half a body-depth
  in, that is **~0.22 m** of head and shoulder PAST the plane: the torso and
  head are what protrude, over the street, above the wall the legs are
  hidden behind. **Marginal, and every `lean_out` record carries
  `needs_bench`** — 0.22 m is the difference between "a face in a dark hole"
  and "head and shoulders over the street" and no arithmetic settles which
  one it renders as. A slight forward pitch toward the street is already what
  the pose gives; there is no pose in the vocabulary with more of one.

The record's `protrusion_m` is this quantity, and `MIN_PROTRUSION_M` (0.15) is
a hard refusal: **a window figure that does not break the facade plane is an
interior figure with extra steps.** A separate `lean_head_clear_m` (0.55 m)
refuses an opening too short for a leaning head to clear the head frame —
`z_head - z_sill` has to be at least that, or the candidate is skipped as
`opening_too_short` rather than clipping the lintel.

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

## 5b2. `roof_victim` — ADDED 2026-08-31, because `roof` is starved on the
## REAL city and the user asked to see somebody up there

**"I don't see any humans standing on roofs (non collapsed) as victims. I
need some of those."** They were right not to see any: on `city_138`'s real
32-building manifest, `roof`'s own `roof_max_h_m` (45 m) and its
`ROOF_OK_LEVELS` (`F1`/`F2`/`F3`) leave exactly ZERO eligible buildings — the
only two whose sidecar shows a genuinely intact deck (`i=280`, `i=282`, both
`F3`) are 47-48 m, a couple of metres over the cap that exists for the
aerial-appliance argument in 5b's point 2. `roof_victim` is a SEPARATE class
rather than a `roof` config change because the two describe different
people — `roof` is doctrine's "refuge crowd, reachable by a ladder,"
`roof_victim` is "somebody stranded up there, still visible, whether or not
a ladder could reach them" — and because raising `roof_max_h_m` itself would
also loosen the `roof` class's own aerial-appliance argument for everybody
already using it.

Eligibility, checkable off the manifest AND the sidecar (never guessed):

1. **not a collapse level** — `level not in roof_victim_excluded_levels`
   (`F5c`, `F6` by default). Named explicitly rather than left to inference,
   so a sidecar that is silent about roof collapse cannot accidentally admit
   a shell that dropped its deck inward.
2. **the sidecar's own fire state says the roof survived** —
   `not roof_involved(rec, doc)` and `not roof_collapsed(rec, doc)`, the same
   two functions `roof.roof_ok` and 5c's burial classes already trust, so a
   real bake's measured `fire.roof`/`fire.top` overrides the manifest's
   conservative band estimate exactly the way it does everywhere else in this
   module.
3. **`H <= roof_victim_max_h_m`** — 90 m by default, deliberately HIGHER than
   `roof_max_h_m` (45 m). This is the whole reason the class exists at all:
   at 45 m the real city offers nothing, and the user's ask was to see some.
   90 m is not a claim that an aerial appliance reaches a stranded victim
   there — it is a claim that a drone can still SEE one there, which is the
   only thing this benchmark's camera cares about.

**Placement is `roof`'s own edge geometry** (inset `roof_victim_edge_band_m`,
default `(1.0, 3.5)` m — "keep them ~1 m back from the parapet edge", user —
scattered `-0.72..0.72` of the half-width along the edge, same as `roof`),
with two differences:

* **BIASED TOWARD THE STREET-FACING EDGE**, not just the edge away from the
  fire. `_roof_victim_street_sides` samples a point `roof_victim_street_
  test_m` (8 m) past each non-venting edge and asks `sol.ground` whether it
  reads as `road`/`sidewalk`/`paved` — an edge that does is ranked ahead of
  `far_sides()`'s own away-from-the-fire order, an edge that does not (a
  party wall, a service yard, a courtyard) falls to the back. "Visible from a
  drone" (user) is mostly about which side of the roof a searching aircraft
  is likely to be over, and that is the street, not the alley.
* **KEPT CLEAR OF THE BUILDING'S OWN SMOKE.** "Keep them away from the smoke
  tho" (user) — `_clear_of_smoke` reads the bake sidecar's own `seats`
  dict (`fire_bake.sidecar`'s `{"interior": [...], "roof": [...]}`, the
  positions `soot_plume` actually seats its flame/smoke sources at),
  rotates every seat into the CITY frame the same way `fire_bake.place`
  moves them (`_rot` by the cell yaw, translate by `(b.x, b.y)` — z is
  untouched, exactly `place`'s own contract), and refuses any candidate
  within `roof_victim_min_smoke_dist_m` (6 m, "several metres") of any of
  them, `interior` and `roof` groups alike — a plume seated on the roof deck
  itself would only exist on a building whose roof IS involved, which this
  class's own eligibility already excludes, but the check costs nothing and
  is not written to assume that stays true forever.

**No singleton, same reason as `roof`.** A group of one is withdrawn and the
budget it held is returned to the pool (`singleton_roof_victim_withdrawn`).
`roof_victim_max_groups_per_building` (2, `roof`'s own default) keeps the
whole class from landing on one deck the way the first real-city `roof` run
did (5b's own point 3's citation). Poses lean toward distress rather than
`roof`'s mixed idle/walk/stand crowd — `stand_slump`, `crouch`, `sit_ground`
are the majority — because a `roof_victim` is stranded, not spectating.

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
* **`deck_z` NOW SURVIVES A REAL BAKE SIDECAR — RESOLVED 2026-08-31.**
  `fire_bake.sidecar()` was measured (against `fire_bakes/city_4/
  dtc_Building_11_F3_o5_SW_s66.json`) to write no `deck_z` at all; it has
  SINCE been updated to persist the measured value into both `fire.deck_z`
  and `masses.main.deck_z` (numeric for gac/dtc, e.g. 69.2755 for
  SM_Building_19; `None` for a kit bake, which never measured one).
  `deck_z()` reads `fire.deck_z` first, then `masses.main.deck_z`, then
  falls back to `"estimated"` (`H - parapet_est_m`) — hit only for a kit
  bake or a bake from before this fix. `SIDECAR_FIELD_USE` and
  `_Building.sidecar_report()` say which path each building actually took.
* **A MATCHED `i` IS NOT A MATCHED BUILDING.** A manifest solved against an
  older run of the same BSP-generated city can have every one of its `i`
  values still present in a fresh dump while most of them now name a
  DIFFERENT house (measured 2026-08-31: 17 of 20, up to 309 m away, some a
  different size entirely — `_Solver.__init__`'s own account). Every record
  is re-verified against that dump placement's own `x/y/W/D/H`
  (`_manifest_matches_dump`), not just looked up, and a manifest this stale
  degrades to an EMPTY plan rather than seeding a crowd around geometry the
  scene no longer has.
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

# LOCAL MIRROR FIRST, Nucleus fallback. The 2026-08-31 Nucleus outage
# composed a live city with humans whose rig references never resolved
# ("missing references", user review) — `scene_gen/assets/people/` is a
# byte-mirror of the Assets tree (rigs + *_mat.usd + Textures/, fetched by
# tools/_mirror_people_once.py), so a checkout that has run the mirror never
# touches the server for people at all. A checkout without the mirror (CI, a
# fresh clone) resolves every name to the Nucleus URL exactly as before.
_LOCAL_PEOPLE_ROOT = os.environ.get(
    "LOCAL_PEOPLE_ROOT",
    os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "..", "assets", "people")))


def _people_asset(name):
    local = os.path.join(_LOCAL_PEOPLE_ROOT, name)
    return local if os.path.isfile(local) else _PEOPLE_ROOT + name


RIGGED_HUMANS = tuple(_people_asset(n) for n in (
    "rp_carla_rigged_001_ue4.usd",
    "rp_claudia_rigged_002_ue4.usd",
    "rp_eric_rigged_001_ue4.usd",
    "rp_manuel_rigged_001_ue4.usd",
    "rp_nathan_rigged_003_ue4.usd",
    "rp_sophia_rigged_003_ue4.usd",
))
POSED_HUMANS = tuple(_people_asset(n) for n in (
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

CLASSES = ("evacuee", "onlooker", "at_car", "window", "roof", "roof_victim",
           "casualty_apron", "roof_debris", "interior_trapped")
# The classes whose budget absorbs a degraded class's share WHEN THEY ARE
# THEMSELVES ELIGIBLE. Kept as the historical default (the street classes are
# ordinarily the deepest pool); `plan_people` no longer trusts this list
# blindly — see ITEM 3 below — because with `street_classes` off it points at
# two classes that are themselves ineligible and the give-back would vanish.
_FALLBACK_CLASSES = ("evacuee", "onlooker")
# The classes that live on the ground plane and go through the street gates.
STREET_CLASSES = ("evacuee", "onlooker", "at_car")
# The classes that are legitimately over or inside a building footprint.
AERIAL_EXEMPT_CLASSES = ("window", "roof", "roof_victim", "interior_trapped")

# ITEM 3, 2026-08-31 user review ("No people on the ground that aren't like
# part of the damage"): the three STREET classes (evacuee/onlooker/at_car —
# crowd and bystanders on the sidewalk/road/kerb, none of them tied to a
# specific piece of damage) are OFF BY DEFAULT. Every remaining class is
# damage-tied: a window leaner, a roof victim, a burial, or a figure visible
# through a broken wall. The code is not removed — a reviewer who wants the
# fireground crowd back sets `FP_STREET_CLASSES=1` (env) or
# `cfg["street_classes"] = True` (programmatic) — but the DEFAULT export, the
# bench and the full-city solve now ship without it.
STREET_CLASSES_ENV = "FP_STREET_CLASSES"

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
    # STAND_CALM / WAVE_HELP ONLY — 2026-08-31 user review, quoted: "The roof
    # people need to just be standing. not like bent... don't need to be on
    # the ledge." `idle`/`walk`/`crouch`/`sit_ground` all read as a figure
    # bent at the waist or down on the deck (`stand_slump`'s -10/-12/-6 spine
    # pitch is the same silhouette the review called "about to jump off the
    # roof"); neither belongs on a refuge crowd standing on a flat, intact
    # deck. `wave_help` gives a third of the group the rescue-signal read.
    "roof": (("stand_calm", 0.65), ("wave_help", 0.35)),
    # STRANDED, NOT SPECTATING — weighted MORE toward the rescue signal than
    # `roof`'s crowd, because a `roof_victim` is alone up there and reaching
    # for a ladder is the whole point of the class (section 5b2). Same two
    # poses as `roof` for the same reason: nothing here may crouch or sit.
    "roof_victim": (("wave_help", 0.55), ("stand_calm", 0.45)),
    # `interior_trapped` draws its own two pose tables
    # (`_INTERIOR_CONSCIOUS_POSES` / `_INTERIOR_UNCONSCIOUS_POSES`, section
    # 5d) rather than an entry here — it needs the eye-height/sightline
    # split by variant, which this flat per-class table has no room for.
}
# The lying attitudes, and the mix. A third face-up, a third face-down, a
# third on a side — `tornado_people`'s own split, and for the same reason it
# gives: there is no literature on the attitude distribution, so the set
# covers the space evenly and says so.
#
# `buried_reach` ADDED 2026-08-31 (user review item 1/2): "supine ... one arm
# extended — for partial burial." It is the only pose in this table authored
# FOR this module rather than borrowed from `tornado_people`'s set, so it
# carries its own share (0.14) taken evenly off the rest rather than added on
# top — the mix still sums to a normalised distribution, not a longer list at
# the old weights.
_LYING_POSES = (("lying_supine", 0.15), ("lying_supine_open", 0.14),
                ("lying_prone", 0.15), ("lying_prone_reach", 0.14),
                ("lying_side_l", 0.11), ("lying_side_r", 0.11),
                ("lying_curled_l", 0.06), ("buried_reach", 0.14))
# `people.LYING_POSES` — the roll that lays each one down. Copied rather than
# imported so this module has no import-time dependency; `tests/
# test_fire_people.py` asserts the two agree. `buried_reach` started as
# fire_people's own extra entry and has SINCE been added to
# `people.LYING_POSES` too (2026-08-31, same review) — the two tables agree
# key for key today, and the test pins that.
LYING_ROLL = {"lying_prone": 90.0, "lying_prone_reach": 90.0,
              "lying_supine": -90.0, "lying_supine_open": -90.0,
              "lying_side_l": 90.0, "lying_side_r": 90.0,
              "lying_curled_l": 90.0, "buried_reach": -90.0}
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

    # ITEM 3, 2026-08-31: OFF by default. "No people on the ground that
    # aren't like part of the damage" — the street crowd is not tied to any
    # specific damage, so `evacuee`/`onlooker`/`at_car` are excluded from the
    # census unless explicitly turned back on (`FP_STREET_CLASSES=1` env, or
    # `cfg["street_classes"] = True`). See `STREET_CLASSES_ENV`.
    "street_classes": False,

    # POSE FALLBACK, 2026-08-31 bench-v2 REJECTION, PARTIALLY RESOLVED
    # 2026-09-01. The user rejected `stand_calm`/`wave_help`/`lean_window`
    # on sight ("these poses are completely wrong ... Just spawn people
    # only"). `roof_use_new_pose` STAYS OFF — the user's later verdict on
    # `idle` roof figures was "the roof people on non collapsed roofs are
    # good", i.e. the FALLBACK is what shipped and was approved; there is
    # no still-pending "real" roof pose waiting behind this flag. `window_
    # use_lean_pose` is now ON — the 8-variant bench pose row (`WINDOW_
    # POSE_VARIANTS`) resolved the mapping question empirically and
    # `lean_window` was rebuilt pelvis-hinged around the winning variant
    # (see that pose's own comment in `scene_generator._HUMAN_POSES`).
    "roof_use_new_pose": False,
    "window_use_lean_pose": True,
    # THROWAWAY window-pose empirical search, 2026-09-01 (see
    # `_pass_window_pose_experiment`) — RETIRED, off by default and left
    # unused: the row already produced its verdict (`lean_window` above).
    # The mechanism is kept rather than deleted ("gate stays for posterity
    # if trivial" — coordinator) since leaving it off costs nothing and a
    # future pose question can reuse the same bench-row scaffolding.
    "window_pose_experiment": False,

    "shares": {"evacuee": 0.34, "onlooker": 0.20, "at_car": 0.08,
               "window": 0.16, "roof": 0.12, "roof_victim": 0.08,
               "casualty_apron": 0.07, "roof_debris": 0.03,
               # ITEM 2, 2026-08-31: figures visible INSIDE a partially
               # collapsed building through its broken-open wall — see
               # section 5d / `_pass_interior_trapped`.
               "interior_trapped": 0.06},
    "group_sizes": {"evacuee": (2, 5), "onlooker": (2, 4), "at_car": (1, 2),
                    "roof": (2, 4), "roof_victim": (2, 3), "window": (1, 1),
                    "casualty_apron": (1, 2), "roof_debris": (1, 1),
                    "interior_trapped": (1, 2)},
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
    # HOW FAR INSIDE THE FACADE PLANE THE PELVIS SITS — half a body depth,
    # i.e. the BELLY is against the sill, which is what leaning out of a
    # window is. This is the ONLY term that positions the body horizontally;
    # charging it the sill inset AS WELL as its own body depth (the first
    # draft did, back when there was a second `sill_sit` variant) left it
    # 0.02 m of protrusion, below `MIN_PROTRUSION_M`, so the variant could
    # never appear — a whole branch dead behind a plausible-looking number.
    "lean_out_inset_m": _BODY_HALF_DEPTH_M,
    # THE PELVIS-HINGED `lean_window`'s OWN INSET, FINALIZED 2026-09-01 —
    # narrower than `lean_out_inset_m` (half a body depth, sized for the
    # retired spine-only pose's larger head displacement). MEASURED: at
    # `lean_out_inset_m` (0.17 m) the new pose's 0.152 H head reach gives
    # only 0.10 m of net protrusion — UNDER `MIN_PROTRUSION_M` (0.15),
    # which would refuse the whole class despite the pose being the one the
    # bench row's own render approved. The pelvis hinge does not need as
    # deep an inset to still read as "leaning out" — this is what the row
    # actually showed — so the companion constant is re-tuned with it
    # rather than re-deriving the pelvis angle a second time.
    "lean_window_inset_m": 0.09,
    # OPENING HEIGHT NEEDED FOR A LEANING HEAD TO CLEAR THE HEAD FRAME —
    # `z_head - z_sill` has to be at least this or the candidate is refused
    # as `opening_too_short` rather than clipping the lintel. Less than the
    # old sitting figure's clearance (0.95) because leaning needs less
    # headroom than sitting upright in the opening did.
    "lean_head_clear_m": 0.55,
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
    # ITEM 5, 2026-08-31 user review: "they can't be right next to the open
    # flame. They have to be away from it." A window figure is refused if its
    # opening is within this planar distance of any flame-bearing bake EVENT
    # on the SAME elevation — "~2 openings / ~6 m", the user's own number.
    # See `_clear_of_flame`.
    "window_flame_clear_m": 6.0,
    # An event counts as "flame" (as opposed to a low-severity smoke-only
    # touch) at or above this `intensity` (the sidecar's own 0-1 event field,
    # the same one `soot_plume`'s EC1 flame model reads). Conservative on
    # purpose — a marginal event still keeps its clearance.
    "window_flame_intensity_min": 0.30,
    # HARD CAP, 2026-09-01 user follow-up on bench-v4: "Don't do any window
    # leans on the top 2-3 stories always below." Applies on EVERY building,
    # every level — not just collapse ones — a window figure is never in
    # the top N storeys. See `_Building.window_storeys`.
    "window_top_storeys_excluded": 3,
    # CORNER MARGIN, item 3 (bench-v2 rejection: a figure centred on a
    # pilaster). A derived (unmeasured) opening grid never draws a candidate
    # within this many metres of either end of the wall face — see
    # `openings_for_side`'s derived branch.
    "corner_margin_m": 1.5,
    # BENCH-V3 REJECTION (fallback window geometry): how far behind the
    # facade plane the STANDING (`idle`) figure sits — a real body depth,
    # not `lean_out_inset_m`'s 0.17 m, which was solved for a pose that
    # reaches OUT from the sill and reads as "at the glass" on a figure that
    # does not lean at all.
    "window_stand_inset_m": 0.6,
    # Minimum spandrel/sill band, `z_sill - floor_z`, for the STANDING
    # fallback only — an opening with less than this is floor-to-ceiling
    # glazing with nothing to hide legs behind, and is skipped
    # (`no_spandrel`) rather than placed anyway.
    "window_min_spandrel_m": 0.8,
    # BENCH-V3 REJECTION (burial covering): max metres between a covering
    # piece's bottom face and the body's own authored surface for the piece
    # to count as "resting on the body" rather than floating detached from
    # it — generous enough for the crest's own bands, tight enough that a
    # half-metre gap (the reported symptom) fails the gate.
    "cover_contact_tol_m": 0.35,

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
    # WIDENED 2026-08-31 (user: "don't need to be on the ledge. Centre is
    # fine"). (1.2, 3.5) hugged the parapet; this keeps the group a genuine
    # few metres onto the deck instead of right at the coping. Still an inset
    # off ONE edge (the group looks out over it, per `roof`'s own comment) —
    # not the deck's geometric centre — but `out = max(0.4, half_out - e)`
    # means a small deck still clamps sanely rather than pushing a group past
    # its own midline.
    "roof_edge_band_m": (2.5, 6.0),
    "parapet_est_m": 1.0,

    # --- roof_victim (section 5b2) ---
    # NEVER a collapse level, checked by NAME rather than left to inference —
    # a sidecar silent on roof collapse must not admit a shell that dropped
    # its deck inward.
    "roof_victim_excluded_levels": ("F5c", "F6"),
    # DELIBERATELY HIGHER THAN `roof_max_h_m` (45 m). At 45 m the real
    # `city_138` manifest offers ZERO eligible buildings (measured
    # 2026-08-31: the only two with a genuinely intact deck per their own
    # sidecar are 47-48 m) — this class exists so the user's "I need some of
    # those" has somewhere to land. Still a real cap, not none: a stranded
    # figure on an actual skyscraper roof is unobservable by this
    # benchmark's flight envelope the same way an 83 m window figure was
    # (`window_max_z_m`'s own account).
    "roof_victim_max_h_m": 90.0,
    "roof_victim_max_groups_per_building": 2,
    # WIDENED with `roof_edge_band_m` (user: "centre is fine") but kept
    # tighter than `roof`'s — a `roof_victim` is specifically sited to be
    # SEEN from the street (`_roof_victim_street_sides`), so it stays nearer
    # its edge than the general refuge crowd does, just not hugging the
    # coping the way (1.0, 3.5) did.
    "roof_victim_edge_band_m": (2.0, 5.0),
    # "KEEP THEM AWAY FROM THE SMOKE THO" (user). Planar distance from every
    # one of the building's own bake-sidecar `seats` (interior AND roof
    # plume groups, rotated into the city frame the same way
    # `fire_bake.place` moves them) — "several metres".
    "roof_victim_min_smoke_dist_m": 6.0,
    # How far past a non-venting edge to sample the ground class when
    # deciding whether that edge is "the street side" — has to clear the
    # sidewalk and land on the carriageway/paved surface actually meant by
    # "visible from a drone", not the kerb line itself.
    "roof_victim_street_test_m": 8.0,

    # --- burial ---
    # RE-ANCHORED CLOSER TO THE WALL, 2026-09-01 user verdict on bench-v4:
    # "the people who are under rubble look good but they are too far away
    # from the actual debris" (coordinator: "the figures sit in the outer
    # apron band while the heap is at the wall ... clean asphalt metres
    # away"). The outer quarter (0.72-1.00) was where the windrow THINS to
    # a few tens of centimetres — correct for "never buried past visibility"
    # but, on the real `apron_run_m()` distances this module solves against
    # (`apron_spread * H`, uncapped except by `max_wall_dist_m`), a wide
    # tail on a tall building put the body well clear of anything a camera
    # would read as rubble. Moved to (0.40, 0.65) — still never the deepest
    # material at the wall line itself (`in_footprint` keeps the standing
    # shell's own footprint off-limits regardless of band), but now over
    # the SUBSTANTIAL part of the heap (`apron_surface_z`'s own (1-t)^1.3
    # falloff means t=0.40-0.65 is 1.9-4.8x the debris depth of the old
    # 0.72-1.00 band), not its thinning tail.
    "apron_band": (0.40, 0.65),
    # ROOF DECK LANDS FURTHER OUT THAN THE WALL WINDROW'S OWN TAIL: it came
    # off the top and slid over the failed elevation, so its band starts a
    # little inboard of the wall rubble's. Still never the mound. Re-
    # anchored the same direction and for the same reason as `apron_band`.
    "roof_debris_band": (0.35, 0.60),
    "out_depth_m": (1.15, 2.0),    # fire_collapse.OUT_DEPTH_M
    "max_covered_frac": MAX_COVERED_FRAC,
    # WHETHER A BURIAL FIGURE GETS REAL COVERING DEBRIS AUTHORED OVER IT, not
    # just a `covered_frac` number on the record. ITEM 2, 2026-08-31 user
    # review: "the people must be partially visible through the rubble ...
    # not part of the damage". See `_cover_burial`, which reuses
    # `tornado_people`'s `_crest`/`_trim_spans`/`_cover_piece` per the
    # coordinator's "look at tornado code" instruction. On by default — this
    # is the fix, not an opt-in.
    "author_burial_cover": True,
    # HOW FAR THE BODY SINKS INTO THE WINDROW, as a fraction of its own
    # DEPTH (not the windrow's) — `tornado_people`'s own `sink_frac`, and its
    # own post-1-km-review value: 0.32 buried too much of the silhouette
    # ("some are completely obscured"), 0.18 is what that review settled on.
    # `apron_band`'s outer quarter is already thin, so this is a modest
    # extra sink into it, not the whole story of how "torso/arm visible
    # above the heap" is achieved — the covering pieces do most of that.
    "sink_frac": (0.0, 0.18),

    # --- interior_trapped (section 5d) ---
    # A partial-collapse (F5c) building keeps everything BELOW its fire
    # origin — `fire_collapse.plan_partial_collapse`'s own invariant, "NOTHING
    # BELOW THE FIRE'S ORIGIN, EVER" — so the storey directly below the
    # origin is a guaranteed-surviving floor slab, visible through the hole
    # the failed elevation left. `fire.sides[0]` (the sidecar's own venting
    # side) is used as the failed/lost elevation — the same inference
    # `fire_collapse.plan_partial_collapse` itself makes by default.
    "interior_trapped_conscious_share": 0.5,   # rest are passed out (prone)
    # SET BACK FROM THE BROKEN EDGE — "don't have them right at the edge
    # though, they need to be a little inside at a safe distance" (user).
    # Metres in from the exposed slab/wall-opening edge.
    "interior_setback_m": (1.5, 3.0),
    # How far in from the SIDE walls of the slab (perpendicular to the
    # setback direction) a figure may sit — keeps it off the flanks, in the
    # part of the floor actually behind the hole rather than behind a
    # standing side wall.
    "interior_lateral_frac": 0.55,
    # The oblique sightline check (a hole several storeys tall still shows a
    # floor a couple of metres in) — minimum degrees above the horizontal a
    # camera would need to look down the opening to see the figure, capped so
    # a figure is never placed somewhere only a plumb-down shot could find.
    "interior_min_sightline_deg": 25.0,

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

# ---------------------------------------------------------------------------
# EXACTLY WHICH SIDECAR FIELD EACH PASS READS, AND THE FALLBACK WHEN IT IS
# ABSENT — for a reviewer, and for `tools/fire_people_dry_run.py`'s
# per-building report (`building_sidecar_report`/`print_sidecar_report`).
# `(field path in the sidecar doc, what reads it, what happens without it)`.
#
# THE deck_z ROW WAS STALE AND HAS BEEN CORRECTED (2026-08-31). Measured
# against a REAL bake sidecar (`fire_bakes/city_4/dtc_Building_11_F3_o5_
# SW_s66.json`) at first, `fire_bake.sidecar()` carried no `deck_z` anywhere
# — its `"fire"` dict had only `origin, storeys, top, sides, n_storeys,
# mass, roof, level, state, finish`, and `mass_to_json` dropped it from the
# mass dict too (LOSSY ON `spec`, deliberately). `fire_bake` has SINCE been
# updated to persist it: `sidecar()` now writes the measured value into BOTH
# `fire.deck_z` and `masses.main.deck_z` (numeric for a gac/dtc bake, e.g.
# 69.2755 for SM_Building_19; `None` for a kit bake, which never measured
# one — `deck_z()` treats that `None` as absent and falls through). A
# building whose bake predates this fix, or a kit-path building, is still on
# the "estimated" (`H - parapet_est_m`) path — that is a real, load-bearing
# case, not a leftover claim.
SIDECAR_FIELD_USE = (
    ("fire.top", "band_top() -> window_storeys()",
     "urban_fire.BAND's ceiling off the manifest's own level+origin "
     "(conservative: never under-refuses a window as safe)"),
    ("fire.roof", "roof_involved()",
     "band_top() >= n_storeys - 1, the same test the bake itself used"),
    ("fire.roof_collapse (not a real field; inferred from `notes` text or "
     "the level)", "roof_collapsed()",
     "F5/F6 always; F5c only if roof_involved() — no sidecar round-trips "
     "an explicit flag today"),
    ("fire.deck_z, then masses.main.deck_z", "deck_z() for the roof class",
     "H - parapet_est_m, deck_source='estimated' — hit whenever BOTH are "
     "absent or None (a kit bake, or a bake from before fire_bake persisted "
     "this)"),
    ("events[*].ops[*] on a side", "openings_for_side() sidecar_grid vs "
     "derived",
     "a synthesised default grid (bay_pitch_m/opening_w_m/sill_h_m/"
     "head_h_m), flagged needs_bench; the sidecar only ever carries "
     "VENTING openings anyway (section 5a/8), so even a MATCHED sidecar "
     "cannot supply the storeys this class actually wants"),
    ("(none — not read at all)", "casualty_apron / roof_debris debris "
     "footprint", "fire_collapse.OUT_SPREAD/OUT_DEPTH_M off the manifest's "
     "own H; no sidecar field is consulted for either burial class"),
)


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

    **ALSO keyed by `(cell_or_tag, level)`, found 2026-08-31**
    (`tools/people_float_audit.py`, cross-checking the live `city_138`
    manifest against its own on-disk sidecars). A cell that was RE-BAKED at
    more than one severity for iteration — the F3/F4/F5/F5c pairs this bake
    directory actually carries for 10 cells, e.g.
    `kit_apartment_tall_F3_o4_SNW_s758.json` AND
    `..._F5_o4_SNW_s758.json`, same `tag` ("k14"), same `city.cell` — used
    to collide on the BARE `cell`/`tag` key: `sorted(os.listdir(...))`
    visits `F3` before `F5`, so the later file silently overwrote the
    earlier one in `out`, and whichever bake happened to sort last won
    regardless of which level the CURRENT manifest record actually is.
    Measured: `i=257`'s manifest record is `F3`, but the plain `cell` key
    returned the `F5` doc — 9 storeys of the wrong fire band, wrong
    `fire.top`, wrong openings — for every run against this bake set until
    now. The tuple key lets a caller ask for the SAME level it already
    knows off the manifest record and get the file that actually matches;
    the bare key is kept exactly as before (last-alphabetical-wins) as the
    fallback for a caller with no level to hand, e.g. `_bake_tag`'s own
    fallback path lower down."""
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
        # THE BAKE USD SITS RIGHT NEXT TO ITS SIDECAR, SAME BASENAME
        # (`fire_city_bake.sh`'s own naming: `<tag>.json` / `<tag>.usd` in
        # the same directory) — recorded here, under a leading-underscore key
        # no bake writer uses, so `local_roof_z` can open the REAL geometry
        # for a per-point deck height instead of trusting `deck_z`'s single
        # scalar everywhere on the roof. Absent (not an error) for a sidecar
        # whose sibling `.usd` was moved or never baked.
        doc["_sidecar_path"] = path
        level = doc.get("level")
        if doc.get("tag"):
            tag = str(doc["tag"])
            out[tag] = doc
            if level is not None:
                out[(tag, str(level))] = doc
        cell = ((doc.get("city") or {}).get("cell"))
        if cell:
            cell = str(cell)
            out[cell] = doc
            if level is not None:
                out[(cell, str(level))] = doc
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

    UPDATED 2026-08-31: `fire_bake.sidecar()` now PERSISTS this — it writes
    the measured value into both `fire.deck_z` and `masses.main.deck_z`
    (numeric for a gac/dtc bake, e.g. 69.2755 for SM_Building_19; `None` for
    a kit bake, which never measured one). So the priority here is
    `fire.deck_z`, then `masses.main.deck_z` specifically (not any mass —
    an annex/wing mass is not the roof this class stands on), then a
    sidecar-measured estimate, then a manifest-only one; every step is
    `is not None`, so a kit bake's `None` falls through cleanly to the next
    one instead of being mistaken for a zero.

    **THE ESTIMATE PREFERS THE SIDECAR'S OWN `top_z` OVER `rec["H"]`, WHEN A
    SIDECAR IS THERE AT ALL (found 2026-08-31, `people_float_audit.py`).**
    `rec["H"]` is what the fire was SOLVED against; `doc["top_z"]` is
    measured off the geometry that is actually STANDING in the scene, and
    the two are not always the same building. The "brownstone mini blocks"
    mechanic (`tools/fire_city_force_blocks.py`) re-skins a cell's `style`
    to a kit archetype in place *"style may legally diverge from the cell's
    real asset"*) without touching the manifest record's `x/y/W/D/H` — those
    still describe whatever taller building the fire was originally solved
    against at that cell. On `city_138`'s 3 forced brownstone cells
    (`i=261/264/269`) that left `rec["H"]` 21.8-22.3 m against a REAL baked
    `kit_brownstone` height of 17.1-18.8 m (`doc["top_z"]`) — a 2.4-4.1 m
    error that put every `roof`/`roof_victim` figure on those three decks
    floating in mid-air above the actual roof line, all under
    `deck_source="estimated"` so nothing about the record looked wrong on
    its own. `top_z` is only ever used for `roof_ok_levels` (F1-F3, never a
    level with a rubble pile in it) via this class's own eligibility gates,
    so it is never confused with a post-collapse debris top the way a bare
    GAC bbox top would be (section 5b's own caution, which is about a
    DIFFERENT quantity — `gac_fire`'s bbox top on a building whose own
    `fire.deck_z` this branch is never reached for, because gac/dtc bakes
    always measure one).
    """
    pe = float(parapet_est_m if parapet_est_m is not None
               else DEFAULTS["parapet_est_m"])
    if doc:
        f = doc.get("fire") or {}
        if f.get("deck_z") is not None:
            return float(f["deck_z"]), "sidecar"
        main = (doc.get("masses") or {}).get("main")
        if isinstance(main, dict) and main.get("deck_z") is not None:
            return float(main["deck_z"]), "sidecar_mass"
        # `masses.main.top` — MEASURED 2026-08-31, `people_bench`'s own F3 kit
        # apartment (`kit_apartment_tall_F3_o4_SNW_s758`), the ONLY roof-
        # eligible building in the whole bench trio. `fire.deck_z` and
        # `masses.main.deck_z` are both `None` (a kit bake never measures
        # either), so this building fell all the way through to the
        # `sidecar_top_z` branch below: `top_z` (35.82) - `parapet_est_m`
        # (1.0) = 34.82. Opening the bake's own USD with bare `pxr` and
        # measuring every mesh's world bbox found the real walkable deck
        # (`deck_k14_4317`, a 27.00 x 17.00 m mesh matching `main["W"]`/
        # `main["D"]` almost exactly) at z=[32.84, 33.00] — and `top_z`
        # (35.82) turned out to be the top of `bulkcap_k14_2`, a 4.5 x 3.3 m
        # ROOFTOP BULKHEAD CAP, not the deck at all. So `sidecar_top_z` was
        # 1.82 m TOO HIGH on the one building the bench could test it on —
        # "some buildings have railings so the roof is below that lip and
        # people look like they're floating" (user, item 4), except the
        # riser here is a stair/mechanical bulkhead rather than a railing.
        # `masses.main["top"]` (33.0) matches the measured deck to two
        # decimal places: it is the mass's own known constructed height —
        # authoritative about the STRUCTURE, unlike `top_z`, which is a bbox
        # over the WHOLE prim and picks up whatever rides highest on the
        # roof (a bulkhead here; a railing/parapet on another archetype).
        # Preferred over `sidecar_top_z` for exactly that reason, and NOT
        # flagged `needs_bench` below — it is measured, not estimated, even
        # though the number was not itself carried by `fire.deck_z`.
        if isinstance(main, dict) and main.get("top") is not None:
            return max(2.5, float(main["top"])), "sidecar_mass_top"
        top_z = doc.get("top_z")
        if top_z is not None:
            return max(2.5, float(top_z) - pe), "sidecar_top_z"
    return max(2.5, float(rec["H"]) - pe), "estimated"


# Sources trusted enough that a roof/roof_victim placement does not need a
# bench check of its OWN z (a window figure's `needs_bench` is about the
# opening grid, a separate question). `sidecar_top_z`/`estimated` are still
# guesses — the one thing the 2026-08-31 measurement proved is that guessing
# from a flat parapet height can be badly wrong.
DECK_Z_BENCH_FREE_SOURCES = ("sidecar", "sidecar_mass", "sidecar_mass_top")

# ---------------------------------------------------------------------------
# LOCAL ROOF-Z SAMPLING — item 4, 2026-08-31 user review: "Some buildings have
# railings so the roof is below that lip and people look like they're
# floating rn." `deck_z()` above is ONE scalar per building's main mass,
# measured at BAKE TIME (`gac_fire.mass_from_grid`'s area-binning walk down
# from the bbox top) off the PRE-SLICE merged mesh. This module never has
# that mesh — the sidecar is JSON, not geometry — so a candidate roof point
# has always trusted the one global number, and a stepped or multi-tier deck
# (a mechanical bulkhead, a raised plant platform, a genuinely tiered roof)
# floats or sinks wherever the real local surface departs from it.
#
# `urban_fire._local_roof_z`/`_roof_tiles` already solve exactly this for a
# LIVE, in-progress build — but by reading `role="roof"` elements off
# `ctx["stage"]`, i.e. Python-side bookkeeping from the SAME session that
# built the roof, which this module (a pure planner with no stage) never has.
# MEASURED, 2026-08-31: a bare bake USD's roof meshes carry NO "roof" (or
# "deck"/"parapet") substring anywhere in their prim paths — the role tag is
# pure ctx-dict metadata that does not survive the export — so `_roof_tiles`'s
# own selection mechanism cannot be ported verbatim to a cold-opened file.
#
# WHAT THIS DOES INSTEAD: the bake USD sits right next to its sidecar
# (`load_sidecars` now records `doc["_sidecar_path"]`; same basename, `.usd`
# extension), and it is a REAL, already-settled scene — opening it with a
# bare `pxr` import (no Isaac, no live stage; same discipline
# `people.bole_bearing_deg` already uses for a re-measured fallback) costs
# nothing and needs no Nucleus. `local_roof_z` collects every mesh POINT
# within `radius_m` of the candidate (x, y) and within a narrow Z band around
# the sidecar's own `deck_z` hint, and returns their MEDIAN. Narrow — not the
# `[deck_z-1.2, deck_z+2.8]` a first pass used — because a wide band pulled in
# an unrelated raised structure on a real `city_138` roof (SM_Building_11:
# points 2.17-2.52 m above the recorded `deck_z`, most likely a mechanical
# bulkhead near that XY, not the walkable deck) and there is no semantic tag
# left to tell "roof" apart from "rooftop plant" once the file is a bare mesh
# soup. A band this tight instead answers a narrower, safer question — "is
# there real deck surface within a few tens of centimetres of what the global
# number already claims" — and gets NO ANSWER (falls through to the global
# scalar) rather than a confidently wrong one whenever it is not. On the same
# `city_138` bake this correctly returned nothing at every sampled point
# (`SM_Building_11`'s local surface near typical candidate positions is not
# within the tight band of its own recorded `deck_z` at all — worth a
# reviewer's own look at that specific building; recorded as a discrepancy
# rather than silently resolved either way) while on a second building
# (`SM_Building_02`) it agreed with the global scalar to within 3-10 cm at
# every one of eight sampled points — i.e. the common case is confirmation,
# and the rare case is a flag, not a guess.
_ROOF_Z_CACHE = {}


def local_roof_z(usd_path, x, y, deck_z_hint, radius_m=2.2,
                  band_lo=0.8, band_hi=1.0, min_points=8):
    """The MEDIAN height of real mesh geometry within `radius_m` of `(x, y)`
    and within `[deck_z_hint - band_lo, deck_z_hint + band_hi]`, or `None`
    when `pxr` is unavailable, the file cannot be opened, or too few points
    fall in the window to trust (`min_points`) — the caller's contract is to
    fall back to `deck_z_hint` on `None`, never to raise.

    Results are cached per `(usd_path, round(x,1), round(y,1))` — a roof pass
    calls this once per CANDIDATE, and a small deck sees the same handful of
    candidate cells retried across `_try` loops and, for `roof_victim`, a
    second class on the same building.
    """
    key = (str(usd_path), round(float(x), 1), round(float(y), 1),
           round(float(deck_z_hint), 2))
    if key in _ROOF_Z_CACHE:
        return _ROOF_Z_CACHE[key]
    z = _local_roof_z_uncached(usd_path, x, y, deck_z_hint, radius_m,
                               band_lo, band_hi, min_points)
    _ROOF_Z_CACHE[key] = z
    return z


def _local_roof_z_uncached(usd_path, x, y, deck_z_hint, radius_m, band_lo,
                           band_hi, min_points):
    try:
        from pxr import Usd, UsdGeom
    except Exception:
        return None
    if not usd_path or not os.path.isfile(str(usd_path)):
        return None
    try:
        stage = Usd.Stage.Open(str(usd_path))
    except Exception:
        return None
    if stage is None:
        return None
    zlo = float(deck_z_hint) - float(band_lo)
    zhi = float(deck_z_hint) + float(band_hi)
    r2 = float(radius_m) ** 2
    zs = []
    try:
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh) or not prim.IsActive():
                continue
            pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if not pts:
                continue
            try:
                xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())
            except Exception:
                continue
            for p in pts:
                w = xf.Transform(p)
                wz = float(w[2])
                if wz < zlo or wz > zhi:
                    continue
                dx, dy = float(w[0]) - float(x), float(w[1]) - float(y)
                if dx * dx + dy * dy <= r2:
                    zs.append(wz)
    except Exception:                                   # a malformed bake is data
        return None
    if len(zs) < int(min_points):
        return None
    zs.sort()
    return zs[len(zs) // 2]


def bake_usd_path(doc):
    """The `.usd` sibling of a loaded sidecar's own `.json`, or `None`.
    `load_sidecars` stashes the source path under `doc["_sidecar_path"]`."""
    p = (doc or {}).get("_sidecar_path")
    if not p:
        return None
    usd = os.path.splitext(str(p))[0] + ".usd"
    return usd if os.path.isfile(usd) else None


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


def venting_sides(rec, doc=None):
    """Which elevations the fire actually vents through — from the sidecar
    when there is one (`fire.sides`), otherwise the manifest's own `sides`.
    Same "sidecar first, manifest as a conservative-ish fallback" pattern
    `band_top`/`roof_involved`/`roof_collapsed` above already use, extended
    to cover the one field those three don't touch.

    THIS IS WHERE THE FIX FOR THE 2026-08-31 SMOKE-ON-A-BLANK-WALL BUG HAS
    TO LAND. `gac_fire.prepare` now reconciles a GAC building's venting
    sides against its MEASURED real glazing before baking — a manifest
    `entry_side`/`sides` is the CONTAGION fact (which neighbour lit this
    building) and is never rewritten, but the sidecar's `fire.sides` can
    disagree with it (`SM_Building_26`: manifest says N/W by contagion,
    sidecar says E — its only real glazing). Every OTHER read of "which
    wall is this fire on" in this module goes through `_Building.sides`,
    which this feeds; reading `rec.get("sides")` directly anywhere else
    (as `standoff_m` used to) re-opens the same bug for the one thing this
    module computes that never went through `_Building` -- the falling-
    glass/debris KEEPOUT radius, which would otherwise still protect the
    blank wall the manifest names instead of the real one people are
    actually near."""
    if doc:
        f = doc.get("fire") or {}
        sd = f.get("sides")
        if sd:
            return tuple(sd)
    return tuple(rec.get("sides") or ())


def standoff_m(rec, side, cfg, doc=None):
    """The HARD keepout from one elevation of one burning building.

    A burning elevation gets the falling-glass zone (`glass_fall_frac * H`);
    a burning elevation on a partially collapsed building gets whichever of
    that and the debris run-out is larger; anything else gets a nominal wall
    clearance. All scaled by `standoff_scale`. Section 4.

    `doc`, the bake sidecar, decides which elevation counts as "burning"
    (`venting_sides` above) — the manifest's own `sides` is a fallback for
    when there is no sidecar yet, not a second opinion once one exists.
    """
    H = float(rec["H"])
    fire_side = side in venting_sides(rec, doc)
    if fire_side:
        d = float(cfg["glass_fall_frac"]) * H
        if str(rec.get("level")) in cfg["collapse_levels"]:
            d = max(d, apron_run_m(rec, cfg))
    else:
        d = min(float(cfg["wall_clear_max_m"]),
                max(float(cfg["wall_clear_min_m"]),
                    float(cfg["wall_clear_frac"]) * H))
    return min(float(cfg["standoff_max_m"]), d * float(cfg["standoff_scale"]))


def building_standoff_m(rec, cfg, doc=None):
    """The worst standoff over this building's elevations — the radius a
    street group is placed OUTSIDE of."""
    return max(standoff_m(rec, s, cfg, doc) for s in ("N", "E", "S", "W"))


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
    frame: `[(fr, u0, u1, z_sill, z_head, storey)]`. `doc=None` (no sidecar
    matched at all) is a normal, common case — every caller used to guard it
    individually (`if doc else []` / `has_doc and ...`); guarded HERE
    instead so a new caller (`_pass_window`'s measured-side preference,
    2026-09-01) cannot reintroduce the same crash by forgetting to."""
    if not doc:
        return []
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
        # CORNER MARGIN — item 3, 2026-08-31 bench-v2 REJECTION: the lead's
        # own close-up read found `window_02` centred on a facade PILASTER,
        # not an opening. A derived grid has no idea where the real bays
        # are; it can at least stop OFFERING the two candidates most likely
        # to be structure rather than glass — the corner returns, where a
        # pilaster, a column or a party-wall return is disproportionately
        # likely on real facades of this pack (GAC/kit alike tend to frame
        # a corner rather than glaze through it). Inset both ends by the
        # larger of `corner_margin_m` and one full window width, so the
        # first and last candidate slot is never adjacent to the corner.
        margin = max(float(cfg.get("corner_margin_m", 1.5)),
                    float(cfg["opening_w_m"]))
        u_lo, u_hi = margin, max(margin, 2.0 * half - margin)
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


def window_protrusion_m(inset_m, height_m=NOMINAL_HEIGHT_M):
    """How far the head/shoulders clear the facade plane, given the pelvis
    `inset_m` behind that plane and `lean_window`'s forward pitch — the ONLY
    window geometry now (section 5a; the old seated `sill_sit` put the
    recessed and the protruding halves backwards and was retired, and
    `stand_slump`'s 28 deg variant that replaced it is retired in turn,
    2026-08-31: "torso poking outside... legs inside" wanted more than 0.22 m
    gave, and `stand_slump` never re-aimed the arms toward the opening at
    all).

    **THIS IS A MEASURED FRACTION, NOT A RIGID-ROTATION FORMULA — the
    formula was wrong and the error was large.** The original version of
    this function modelled the crown as carried `(_CROWN_H - _HIP_H) * H *
    sin(pitch)` forward of the pelvis, i.e. as if the whole crown-to-hip span
    pivoted rigidly about the hip by the pose's own cumulative spine angle.
    It does not: `scene_generator._HUMAN_POSES`'s own `dig_bent` account
    measures the identical effect (a cumulative 78 deg of RAW joint angle
    across three spine bones carries `spine_03` forward by only 0.195 m, not
    the ~0.35 m a rigid triangle would give), because each spine joint's
    delta is applied on top of the PARENT's already-rotated frame rather than
    as one hinge at the pelvis — a segmented chain accumulates displacement
    far more slowly than a single rotation of the same total angle. Feeding
    `lean_window`'s actual cumulative pitch (78 deg of raw joint angle, "hip
    hinge ~30-40 deg" being the visual read that angle produces, not the
    literal joint value) into the old formula predicted 0.31 m of protrusion
    for a pose whose REAL FK-measured head position is only ~0.17 m forward
    of the pelvis — i.e. the old formula would have shipped a window class
    that reads as `protrusion_m: 0.31` in every record while the rendered
    torso barely clears the sill at all.

    RE-MEASURED 2026-09-01 — `lean_window` is now the PELVIS-HINGED pose the
    8-variant bench row settled on (`scene_generator._HUMAN_POSES`'s own
    account of the render verdict), not the retired spine-chain version this
    docstring's derivation history above was written against. MEASURED
    (`scene_generator._pose_joint_transforms`, rp_carla, 1.731 m): the head
    sits 0.262 m of -Y off the pelvis (note the SIGN — every other trunk-lean
    pose in the table leans +Y; this one, empirically, leans the other way —
    see the pose's own comment) — 0.152 of stature.
    `_LEAN_WINDOW_HEAD_FRAC_H` is that fraction; the inset is the ONLY term
    that positions the body horizontally, so it is not charged a second time
    on top of it (see `lean_out_inset_m`). This is a LOWER BOUND on what
    actually protrudes: `lean_window`'s arms brace forward of the torso by
    considerably more (0.612 m past the pelvis on rp_carla) but the formula
    stays conservative on the torso/head term alone, the same part of the
    body `MIN_PROTRUSION_M` was written against.
    """
    H = float(height_m)
    return _LEAN_WINDOW_HEAD_FRAC_H * H - float(inset_m)


# See `window_protrusion_m`'s own account: MEASURED head-forward-of-pelvis
# fraction for the FINALIZED pelvis-hinged `lean_window`, rp_carla
# (0.262 m / 1.731 m stature).
_LEAN_WINDOW_HEAD_FRAC_H = 0.152


# ===========================================================================
# The planner
# ===========================================================================
def resolve_cfg(cfg=None):
    """`DEFAULTS` merged with a caller's overrides, one level deep for the
    nested dicts so a caller can move one share without restating them all.

    `FP_STREET_CLASSES` (env) OVERRIDES `street_classes` WHEN SET, the same
    precedence every other env-driven knob in this pipeline uses (the env is
    for a human at a shell; a caller with a real `cfg` dict is doing it
    programmatically and is assumed to mean it). Unset, the caller's `cfg` (or
    `DEFAULTS`, off) wins.
    """
    out = dict(DEFAULTS)
    out["shares"] = dict(DEFAULTS["shares"])
    out["group_sizes"] = dict(DEFAULTS["group_sizes"])
    for k, v in (cfg or {}).items():
        if k in ("shares", "group_sizes") and isinstance(v, dict):
            out[k].update(v)
        else:
            out[k] = v
    if STREET_CLASSES_ENV in os.environ:
        out["street_classes"] = bool(int(os.environ[STREET_CLASSES_ENV]))
    return out


class _Building(object):
    """One burning building, with everything the passes ask of it computed
    once. Wraps a manifest record and (optionally) its bake sidecar."""

    def __init__(self, rec, cfg, doc=None):
        self.rec = rec
        self.doc = doc
        self.cfg = cfg          # kept for sidecar_report(); every other
                                 # method still takes cfg explicitly
        self.i = int(rec.get("i", -1))
        self.cell = rec.get("cell")
        self.x = float(rec["x"])
        self.y = float(rec["y"])
        self.W = float(rec["W"])
        self.D = float(rec["D"])
        self.H = float(rec["H"])
        self.yaw = float(rec.get("yaw_deg", 0.0))
        self.level = str(rec.get("level"))
        self.sides = venting_sides(rec, doc)
        self.n_storeys = max(1, int(rec.get("n_storeys") or 1))
        self.typology = rec.get("typology")
        self.band_top = band_top(rec, doc)
        self.roof_involved = roof_involved(rec, doc)
        self.roof_collapsed = roof_collapsed(rec, doc)
        self.deck_z, self.deck_source = deck_z(rec, doc,
                                               cfg["parapet_est_m"])
        self.standoff = building_standoff_m(rec, cfg, doc)
        self.apron_run = apron_run_m(rec, cfg)
        self.radius = 0.5 * math.hypot(self.W, self.D)

    # -- the three eligibility questions the 3-D passes ask ---------------
    def window_storeys(self, cfg=None):
        """Storeys strictly above the fire band, below the drone's own
        ceiling, AND NEVER IN THE TOP `window_top_storeys_excluded` STOREYS
        of the building — 2026-09-01 user follow-up on the bench-v4 ledge-
        stander complaint: "The partially collapsed roof people might [be]
        window lean people but too high. Don't do any window leans on the
        top 2-3 stories always below." A HARD CAP on every building, every
        level — not just collapse ones — because the same read (a standing
        figure near the roofline with nothing obviously enclosing them
        above) applies wherever the top storeys are. STACKS with the fire-
        band floor: with a lowered origin, "above the band" is often
        already the top storeys, so the eligible slice can be empty
        (`band_top < storey <= n_storeys - 1 - excluded`) — that building
        simply gets no window figures rather than the rule bending for it.

        Empty for F4 and worse by `urban_fire.BAND` — section 5a — empty
        for a building whose fire band already tops the search altitude
        band (`window_max_z_m`), and now also empty for a building short
        enough that the top-storey exclusion eats every storey the fire
        band left standing.
        """
        excl = int((cfg or {}).get("window_top_storeys_excluded", 3)) \
            if cfg is not None else int(DEFAULTS["window_top_storeys_excluded"])
        hi = self.n_storeys - 1 - excl
        st = list(range(self.band_top + 1, min(self.n_storeys, hi + 1)))
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

    def roof_victim_ok(self, cfg):
        """`(ok, reason)` for a stranded `roof_victim` — section 5b2.

        Looser than `roof_ok` on two of its three gates (no `ROOF_OK_LEVELS`
        restriction beyond the explicit collapse-level exclusion, and a
        higher height cap), because it exists precisely for the buildings
        `roof_ok` refuses on the real city. Never looser on the one gate that
        actually matters: the roof has to be genuinely intact, per the
        sidecar's own fire state when there is one.
        """
        if self.level in cfg["roof_victim_excluded_levels"]:
            return False, "collapse_level({0})".format(self.level)
        if self.roof_involved:
            return False, "band_reaches_top"
        if self.roof_collapsed:
            return False, "roof_collapsed"
        hcap = cfg.get("roof_victim_max_h_m")
        if hcap is not None and self.H > float(hcap):
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

    def sidecar_report(self):
        """Everything `tools/fire_people_dry_run.py`'s per-building report
        prints: whether a sidecar matched at all, which of `SIDECAR_FIELD_USE`
        it actually supplied, and which classes are eligible here versus
        which of THOSE are bench-free by construction (never `needs_bench`).

        `bench_free_classes`/`needs_bench_classes` are about THIS building's
        3-D classes only (window/roof/burial) — street classes (evacuee/
        onlooker/at_car) are always bench-free and are not building-specific,
        so they are reported once by the dry run rather than per row.
        """
        has_doc = self.doc is not None
        ops_sides = tuple(s for s in ("N", "E", "S", "W")
                          if has_doc and _side_ops(self.doc, s))
        window_ok = bool(self.window_storeys(self.cfg))
        roof_ok, roof_why = self.roof_ok(self.cfg)
        collapse_ok = self.level in self.cfg["collapse_levels"]
        roof_debris_ok = self.roof_collapsed and collapse_ok
        bench_free, needs_bench = [], []
        if roof_ok:
            bench_free.append("roof")
        if window_ok:
            (bench_free if ops_sides else needs_bench).append(
                "window({0})".format("measured" if ops_sides else "derived"))
        if collapse_ok:
            needs_bench.append("casualty_apron")     # always — see 5c
        if roof_debris_ok:
            needs_bench.append("roof_debris")         # always — see 5c
        return {
            "building_i": self.i, "cell": self.cell, "level": self.level,
            "H_m": round(self.H, 1), "has_sidecar": has_doc,
            "deck_z_source": self.deck_source,
            "window_sides_measured": list(ops_sides),
            "window_eligible": window_ok,
            "roof_eligible": roof_ok,
            "roof_ineligible_why": None if roof_ok else roof_why,
            "collapse_eligible": collapse_ok,
            "roof_debris_eligible": roof_debris_ok,
            "bench_free_classes": bench_free,
            "needs_bench_classes": needs_bench,
        }


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
        # ITEM 2, 2026-08-31 user review: real covering debris authored over
        # a burial figure, not just a `covered_frac` NUMBER on the record —
        # see `_cover_burial`. One list, shared by `casualty_apron` and
        # `roof_debris`, in `planks`-spec shape (`x, y, z, len, wide, t, yaw,
        # pitch, roll, class, propped`) so a launcher already able to author
        # a `planks` field can author these with the same box-mesh call,
        # bound to a rubble/char material instead of `planks.wood_material`.
        self.covering = []

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

        # A MANIFEST RECORD'S `i` MUST STILL BE THE SAME BUILDING IN *THIS*
        # DUMP, and "the index exists" is NOT enough to conclude that.
        # MEASURED, 2026-08-31, `_plans/fire_city_500m.json` (20 records)
        # against a FRESH `_plans/fc_dump_500.json` (87 houses): every single
        # one of the 20 `i` values IS present in the fresh dump — the BSP
        # city regenerates the same index range run to run — but only 3 of
        # the 20 still sit where the manifest says (`dist == 0.00`); the
        # other 17 have moved 9.5-309 m, several with a DIFFERENT W/D/H
        # entirely (e.g. `i=273`: same index, 211 m away, 103 m taller) —
        # because a BSP re-layout renumbers which house lands at a given
        # full-placement-list position. Checking presence alone would have
        # waved all 20 through and put a crowd standing around a burning
        # office block outline that, in this dump, is empty street. So a
        # record is matched by `i` AND then re-verified against that same
        # dump placement's own `x_m/y_m/W/D/H`; either check failing drops
        # the record — never built into a `_Building`, never budgeted, never
        # placed against geometry it does not own — and BOTH failure modes
        # are counted and named rather than merged into one tally, so a
        # reader can tell "stale index" from "reindexed BSP city" at a
        # glance. See `tools/fire_people_dry_run.py`'s per-record report and
        # `test_fire_people.py`'s
        # `test_manifest_records_are_reverified_against_the_dump_not_just_indexed`.
        dump_by_i = {}
        for p in dump.get("placements") or []:
            if p.get("i") is not None:
                dump_by_i[int(p["i"])] = p

        sc = sidecars or {}
        self.buildings = []
        # [{"i", "cell", "level", "reason", ...}], reported by the dry run
        # and rolled into `plan.meta`.
        self.skipped_records = []
        for rec in manifest.get("records") or []:
            ri = rec.get("i")
            dp = dump_by_i.get(int(ri)) if ri is not None else None
            if dp is None:
                self.skipped_records.append({
                    "i": ri, "cell": rec.get("cell"),
                    "level": rec.get("level"), "reason": "index_not_in_dump"})
                continue
            ok, dist_m, size_diff_m = _manifest_matches_dump(rec, dp)
            if not ok:
                self.skipped_records.append({
                    "i": ri, "cell": rec.get("cell"),
                    "level": rec.get("level"), "reason": "geometry_drift",
                    "dist_m": round(dist_m, 1),
                    "size_diff_m": round(size_diff_m, 1),
                    "dump_cell": dp.get("cell")})
                continue
            # LEVEL-QUALIFIED FIRST. A cell re-baked at more than one
            # severity for iteration collides on the bare `cell`/`tag` key
            # (`load_sidecars`'s own account, 2026-08-31) — the tuple key
            # asks for the SAME level this manifest record already claims,
            # so a re-baked cell resolves to the sidecar that actually
            # matches it rather than to whichever file sorted last.
            cell_s, level_s = str(rec.get("cell")), str(rec.get("level"))
            doc = (sc.get((cell_s, level_s))
                   or sc.get((_bake_tag(rec), level_s))
                   or sc.get(cell_s) or sc.get(_bake_tag(rec)))
            self.buildings.append(_Building(rec, cfg, doc))
        if self.skipped_records:
            by_reason = {}
            for r in self.skipped_records:
                by_reason[r["reason"]] = by_reason.get(r["reason"], 0) + 1
            print("[fire_people] {0} of {1} manifest record(s) SKIPPED — no "
                  "longer this dump's own building (stale manifest vs a "
                  "different city layout): {2}".format(
                      len(self.skipped_records),
                      len(manifest.get("records") or []), by_reason))

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
                need = standoff_m(b.rec, side, self.cfg, b.doc)
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


# Tolerances for `_manifest_matches_dump`. Generous next to a real
# building's own footprint (tens of metres) and tight next to the drift a
# re-laid-out BSP city actually produces at the SAME index — see
# `_Solver.__init__`'s account of the 2026-08-31 measurement (0.00 m on an
# untouched house, 9.5-309 m on a reindexed one).
MANIFEST_POSITION_TOL_M = 3.0
MANIFEST_SIZE_TOL_M = 2.0


def _manifest_matches_dump(rec, dp):
    """`(ok, dist_m, size_diff_m)` — is manifest record `rec` still
    describing the SAME building as the dump placement `dp` it was matched
    to by `i`?

    Compares `rec`'s own `x/y/W/D/H` (what the fire was actually solved
    against) to `dp`'s `x_m/y_m/W/D/H` (what stands at that index in THIS
    dump). Both must agree within tolerance; an index that merely exists is
    not evidence the geometry behind it still does.
    """
    dist_m = math.hypot(float(rec.get("x", 0.0)) - float(dp.get("x_m", 0.0)),
                        float(rec.get("y", 0.0)) - float(dp.get("y_m", 0.0)))
    size_diff_m = max(
        abs(float(rec.get("W", 0.0)) - float(dp.get("W", 0.0))),
        abs(float(rec.get("D", 0.0)) - float(dp.get("D", 0.0))),
        abs(float(rec.get("H", 0.0)) - float(dp.get("H", 0.0))))
    ok = (dist_m <= MANIFEST_POSITION_TOL_M
          and size_diff_m <= MANIFEST_SIZE_TOL_M)
    return ok, dist_m, size_diff_m


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


def _side_has_flame(doc, side, min_intensity):
    """Does elevation `side` carry ANY flame-bearing bake event at all?

    Presence only, no distance — the CONSERVATIVE half of item 5 ("prefer
    non-flame elevations entirely when available"): `_pass_window` tries a
    flame-free side first and falls back to `_clear_of_flame`'s per-opening
    distance refusal only when every candidate elevation has fire on it
    somewhere.
    """
    if not doc:
        return False
    thresh = float(min_intensity)
    for ev in (doc.get("events") or []):
        if float(ev.get("intensity", 0.0)) < thresh:
            continue
        for op in (ev.get("ops") or []):
            if str(op.get("side") or ev.get("side")) == side:
                return True
    return False


def _clear_of_flame(b, x, y, side, min_dist, min_intensity):
    """Is `(x, y)` at least `min_dist` from every FLAME-BEARING bake event on
    `side` of this building? Item 5, 2026-08-31 user review: "the people
    through windows ... can't be right next to the open flame. They have to
    be away from it."

    Reads `events[*].ops[*]` the way `_side_ops` does, filtered to `side` and
    to `intensity >= min_intensity` (the sidecar's own 0-1 event field — the
    one `soot_plume`'s EC1 flame model reads, so "flame" here means the same
    thing it means to the module that actually draws the fire). Each
    surviving op's underlying element position (`op["e"]["x"/"y"]`, the
    bake's own frame) is rotated into the CITY frame exactly the way
    `_clear_of_smoke` rotates a seat: `_rot` by the cell yaw, translate by
    the building's own `(x, y)`.
    """
    if not b.doc:
        return True
    thresh = float(min_intensity)
    for ev in (b.doc.get("events") or []):
        if float(ev.get("intensity", 0.0)) < thresh:
            continue
        for op in (ev.get("ops") or []):
            if str(op.get("side") or ev.get("side")) != side:
                continue
            e = op.get("e") or {}
            if "x" not in e or "y" not in e:
                continue
            wx, wy = _rot(float(e["x"]), float(e["y"]), b.yaw)
            wx, wy = wx + b.x, wy + b.y
            if math.hypot(x - wx, y - wy) < float(min_dist):
                return False
    return True


def _pass_window(sol, plan, budget):
    """Figures LEANING OUT of window openings STRICTLY ABOVE the fire band,
    on venting and adjacent elevations. Section 5a.

    LOWER HALF INSIDE, UPPER HALF OUTSIDE (user, 2026-08-31): the pelvis is
    sunk to the real, sidecar-measured sill height (`op["z_sill"]` — never a
    guessed constant, and never a storey whose opening does not come out of
    `openings_for_side`'s own grid) and set back `lean_out_inset_m` behind
    the facade plane, so the legs — everything below hip height — stay at
    that same recessed position, inside the solid wall panel under the sill,
    where it occludes them. `lean_window`'s hip-hinged forward pitch (a
    cumulative 78 deg of raw spine-chain joint angle — a segmented chain
    needs roughly double the raw angle a rigid hinge would, see
    `window_protrusion_m`'s own account — reading as the "~30-40 deg hinge"
    the review asked for) then carries the torso and head past the facade
    plane, arms braced forward as if on the sill —
    over the street. The retired `sill_sit` variant had this backwards (5a's
    own account).

    AND NEVER RIGHT NEXT TO THE OPEN FLAME (item 5, same review): a
    candidate opening is refused if it is within `window_flame_clear_m` of
    any flame-bearing event on the SAME elevation — see `_clear_of_flame`.
    """
    cfg, rng = sol.cfg, sol.rng
    use_lean = bool(cfg.get("window_use_lean_pose", False))
    # BENCH-V3 REJECTION, 2026-08-31: "she stands FULLY VISIBLE head-to-toe
    # in FRONT of the glass, zero leg occlusion." The fallback (`idle`,
    # standing on the floor) was using `lean_out_inset_m` (0.17 m) — a number
    # solved for the OLD spine-hinged lean pose's pelvis-behind-the-sill
    # geometry, not for a figure standing square with no forward reach.
    # 0.17 m is less than a body's own depth, so an upright stander at that
    # inset is still effectively AT the glazing plane, not visibly recessed
    # behind it. `window_stand_inset_m` is a real body depth further in.
    #
    # THE FINALIZED (pelvis-hinged) LEAN POSE USES ITS OWN, narrower inset
    # (`lean_window_inset_m`, see that constant's own account) — NOT `lean_
    # out_inset_m`, which is sized for the retired spine-only mechanism's
    # larger head displacement and would leave the new pose under
    # `MIN_PROTRUSION_M`.
    inset = (float(cfg["lean_window_inset_m"]) if use_lean
            else float(cfg["window_stand_inset_m"]))
    # A CONSTANT, NOT A PER-RECORD DRAW, WHEN LEANING. There is only one
    # geometry, so whether it clears `MIN_PROTRUSION_M` is decided once — a
    # config that sinks the pelvis too far in refuses the whole class up
    # front. THE FALLBACK HAS NO SINGLE ANSWER: "how far the head clears the
    # sill" depends on the sill height of the actual opening drawn, so it is
    # computed per-candidate below instead (`head_clear_m`).
    prot = window_protrusion_m(inset) if use_lean else None
    cands = []
    top_excl = int(cfg.get("window_top_storeys_excluded", 3))
    for b in sol.buildings:
        # DISTINGUISH "no storey above the fire" FROM "the top-storey cap
        # ate every storey the fire band left standing" — 2026-09-01: the
        # user's own instruction is to count and report the latter rather
        # than bend the rule for a short building. `band_only` re-derives
        # the fire-band floor ALONE (no top-storey exclusion) so the two
        # causes cannot be confused with each other.
        band_only = list(range(b.band_top + 1, b.n_storeys))
        if not band_only:
            plan.refuse("no_storey_above_fire")
            continue
        if not b.window_storeys():
            plan.refuse("top_storeys_excluded")
            continue
        if not b.window_storeys(cfg):
            plan.refuse("window_too_high")
            continue
        cands.append(b)
    if use_lean and prot < MIN_PROTRUSION_M:
        plan.refuse("no_protrusion")
        return 0
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
        cand_sides = (vent + adj) if adj else vent
        # ITEM 5, 2026-08-31 user review: "away from it" — CONSERVATIVE
        # preference, not just a per-opening distance refusal. When at least
        # one of this storey's candidate elevations has NO flame-bearing
        # event on it at all, draw the side from that flame-free subset
        # first; only fall back to every candidate side (still gated by
        # `_clear_of_flame`'s per-opening check below) when none is.
        flame_free = [s for s in cand_sides
                     if not _side_has_flame(b.doc, s,
                                            float(cfg["window_flame_intensity_min"]))]
        # BENCH-V7 REJECTION, 2026-09-01: "the leaners [are] pasted flat on
        # a BLANK BRICK wall (no windows anywhere in frame)." NOT a yaw
        # mismatch (checked: the appended bench record's yaw matches the
        # other three's own convention exactly — original city yaw_deg,
        # unrotated). The real cause: `_ADJACENT` sides frequently have NO
        # sidecar ops at all (`_side_ops` empty — the sidecar only ever
        # records the elevation(s) that actually vented), so `openings_for_
        # side` falls through to the synthetic uniform grid — a claim
        # nobody has measured, and on a real building (measured on SM_
        # Building_26's own N/S faces, `_side_ops` empty on both) that claim
        # can be flatly WRONG: a blank party wall, not a row of windows.
        # `flame_free` alone cannot see this — it answers "is there fire
        # here", not "is there GLASS here". Now `flame_free` is intersected
        # with MEASURED sides (`_side_ops` non-empty) first: a side this
        # building's own bake actually recorded openings on beats an
        # unverified guess, even a flame-free one. Only when NO measured
        # side is flame-free does a measured-but-venting side get tried
        # (the per-opening `_clear_of_flame` check below still keeps any
        # individual choice safe), and only when NO side has ANY measured
        # data at all does this fall through to a flame-free derived guess
        # — the original behaviour, for the (still common) building whose
        # sidecar simply never recorded anything to check against.
        measured = [s for s in cand_sides if _side_ops(b.doc, s)]
        pool = ([s for s in flame_free if s in measured] or measured
               or flame_free or cand_sides)
        side = rng.choice(pool)
        # `openings_for_side` is the ONLY source of an opening's geometry —
        # a real measured grid (`sidecar_grid`) when the sidecar has one on
        # this side, else a synthesised default (`derived`, flagged
        # `needs_bench`). Never a hand-rolled height.
        ops = openings_for_side(b.rec, side, storey, cfg, b.doc)
        if not ops:
            plan.refuse("no_openings")
            continue
        op = rng.choice(ops)
        key = (b.i, side, storey, round(op["u"], 2))
        if key in used_ops:
            plan.refuse("opening_taken")
            continue
        # A leaning head needs the opening tall enough to clear the lintel.
        if op["z_head"] - op["z_sill"] < float(cfg["lean_head_clear_m"]):
            plan.refuse("opening_too_short")
            continue
        # BENCH-V3 REJECTION: "the lower body must be hidden by building
        # fabric ... prefer openings that HAVE a solid spandrel/sill band
        # below them ... an opening whose sill is near floor level, full-
        # height glazing, cannot hide legs: skip those bays." Only the
        # FALLBACK needs this — the lean pose sinks the pelvis TO the sill by
        # construction, so a low sill there is a short reach, not a bare-leg
        # exposure; the standing fallback's legs run all the way down to
        # `floor_z` regardless of the sill, so THIS is the gate that decides
        # whether anything is between them and the camera.
        if not use_lean and (op["z_sill"] - op["floor_z"]
                             < float(cfg["window_min_spandrel_m"])):
            plan.refuse("no_spandrel")
            continue
        # THE CEILING IS ENFORCED ON THE SILL, NOT THE FEET. The sill (and
        # everything above it — the torso, the head) is the highest point a
        # camera could ever see; the feet, sunk below it so the hips land at
        # sill height, are always lower still, so gating on the sill is the
        # STRICTER of the two and automatically clears the feet check too.
        # `window_storeys(cfg)` screens candidates with the DEFAULT sill
        # height cheaply; a sidecar's MEASURED sill can sit metres higher up
        # its own storey, which is why this is re-checked on the opening
        # actually chosen (2026-08-31, the first sidecar run shipped four
        # records over it before this existed).
        if op["z_sill"] > float(cfg["window_max_z_m"]):
            plan.refuse("window_too_high")
            continue

        # `out` in `_face_point` pushes along the frame's OUTWARD normal, so
        # a NEGATIVE `out` is the inward offset that puts the pelvis behind
        # the facade plane; the normal itself is `(sin yaw, -cos yaw)`,
        # straight off the frame — see `quake_flow._b_face_pt`.
        px, py, _z = _face_point(op["fr"], op["u"], 0.0, -inset)
        f_yaw = float(op["fr"][2])
        nx, ny = math.sin(f_yaw), -math.cos(f_yaw)
        # ITEM 1 FALLBACK, 2026-08-31 bench-v2 REJECTION ("stomachs are out
        # (bent the wrong way)... arms also look wrong"): `lean_window`
        # cannot be proven correct without a render, and the lead's own
        # close-up read (window_02: bolt upright, arms splayed like the rest
        # pose, hip hinge and neck/head reading backwards) means shipping it
        # again unverified is not an option. `cfg["window_use_lean_pose"]`
        # (False by default now) is the switch back once a render confirms
        # the pose; see WINDOW_POSE_DIAGNOSIS below for the comparison this
        # is waiting on.
        #
        # FALLBACK GEOMETRY — "position-only protrusion, upright figure"
        # (coordinator's own words): `idle` never moves a leg joint, so a
        # figure standing on the STOREY'S OWN FLOOR (`op["floor_z"]`, feet at
        # 0 relative to that storey) has its legs and hips behind the solid
        # wall panel below the sill and its torso/head inside the opening's
        # own [z_sill, z_head] vertical band at ordinary stature (~1.0-1.7 m
        # off the floor) — the same "lower half inside, upper half outside"
        # read the lean pose was trying to earn, produced here by GEOMETRY
        # alone rather than by a pose whose joint math cannot be confirmed
        # this round. No sill-sink term at all: sinking the hips to the sill
        # only makes sense for a pose that leans forward from there.
        head_clear = None
        if use_lean:
            # HIPS AT THE SILL: the feet (the record's own support surface)
            # sit `_HIP_H * H` below it, so `lean_window`'s untouched,
            # near-straight legs put the pelvis exactly at `op["z_sill"]`.
            pose = "lean_window"
            z = op["z_sill"] - _HIP_H * NOMINAL_HEIGHT_M
        else:
            pose = "idle"
            z = op["floor_z"]
            # PER-CANDIDATE, NOT A CLASS CONSTANT (unlike `prot`): whether an
            # `idle` stander's head clears the sill depends on THIS opening's
            # own sill height relative to the floor it stands on, which
            # varies with `openings_source` (measured vs derived) and the
            # storey drawn. `_ACROMION_H` (shoulder) rather than `_CROWN_H`
            # (crown) — the shoulders are the highest point a leaning-out
            # pose would have guaranteed past the sill; holding the standing
            # fallback to the same bar keeps the two variants comparable.
            head_clear = z + _ACROMION_H * NOMINAL_HEIGHT_M - op["z_sill"]
            if head_clear < MIN_PROTRUSION_M:
                plan.refuse("no_head_clearance")
                continue
        z_mode, seat = "floor", None
        # A WINDOW FIGURE MUST SIT ON ITS OWN BUILDING'S WALL — the same
        # invariant `test_19` already asserts. A multi-module GAC/kit bake
        # can carry several ops each measured relative to ITS OWN piece
        # frame while sharing one recorded `side`; `openings_for_side`
        # reuses a single op's frame (`ops[0]`) for every `u` the grid
        # produces on that side, and a `u` that belonged to a DIFFERENT
        # piece's frame reconstructs a point nowhere near this wall (found
        # 2026-08-31 on a real 42 m `bld_apartment_long` sidecar: a `u` of
        # ~37 m against a frame whose own piece spans a few metres put the
        # figure 40 m off the actual facade, and once off the plate
        # entirely). Never guessed away — refused and counted, the same
        # discipline every other geometry check in this pass already keeps.
        if dist_to_obb(px, py, b.x, b.y, b.W, b.D, b.yaw) > 2.0:
            plan.refuse("opening_off_building")
            continue
        # ITEM 5: never right next to the open flame.
        if not _clear_of_flame(b, px, py, side,
                               float(cfg["window_flame_clear_m"]),
                               float(cfg["window_flame_intensity_min"])):
            plan.refuse("too_close_to_flame")
            continue
        if not sol.spaced(px, py):
            plan.refuse("too_close")
            continue
        usd, pose, rigged = _pick_human(rng, pose, allow_posed=False)
        prot_out = prot if use_lean else head_clear
        plan.add({
            "cls": "window", "group": plan.next_group(), "usd": usd,
            "rigged": rigged, "x": round(px, 3), "y": round(py, 3),
            "z": round(z, 3), "yaw_deg": round(math.degrees(
                math.atan2(ny, nx)), 1),
            "pose": pose, "prone": False, "seat": seat, "z_mode": z_mode,
            "alive": True,
            # A default (extrapolated) grid is a claim nobody has measured;
            # marginal protrusion/clearance is also, on its own, at the edge
            # of useful. Either one earns the flag. See section 5a. The
            # fallback ALSO always needs the bench — it is a geometry-only
            # substitute for a pose that has not been proven yet.
            "needs_bench": (op["source"] == "derived"
                            or prot_out < 2.0 * MIN_PROTRUSION_M
                            or not use_lean),
            "flame_clear_m": float(cfg["window_flame_clear_m"]),
            "variant": "lean_out" if use_lean else "standing_at_opening",
            "protrusion_m": round(prot_out, 3),
            "inset_m": inset, "sill_z": round(op["z_sill"], 3),
            "head_z": round(op["z_head"], 3), "floor_z": round(op["floor_z"], 3),
            "hip_target_z": round(op["z_sill"], 3),
            "side": side, "storey": storey, "openings_source": op["source"],
            "building_i": b.i, "building_cell": b.cell,
            "building_level": b.level, "building_sides": list(b.sides),
            "band_top": b.band_top,
            "reason": (
                ("leaning out of a {0} opening on storey {1} of {2} ({3}), "
                 "side {4} ({5}): hips sunk to the sill ({6:.2f} m) so the "
                 "wall occludes the legs, {7:.2f} m of torso and head past "
                 "the facade"
                 .format(op["source"], storey, b.n_storeys - 1, b.level,
                         side, "venting" if side in vent else "adjacent",
                         op["z_sill"], prot_out))
                if use_lean else
                ("standing upright on the floor of a {0} opening on storey "
                 "{1} of {2} ({3}), side {4} ({5}): legs behind the wall "
                 "panel below the sill ({6:.2f} m), {7:.2f} m of shoulder "
                 "clearance above it — fallback pose, `lean_window` not "
                 "shipped this round pending a render (bench-v2 rejection)"
                 .format(op["source"], storey, b.n_storeys - 1, b.level,
                         side, "venting" if side in vent else "adjacent",
                         op["z_sill"], prot_out))),
        })
        used_ops.add(key)
        sol.placed.append((px, py))
        per[b.i] = per.get(b.i, 0) + 1
        made += 1
    return made


# THROWAWAY EIGHT-VARIANT bench pose row for `lean_window`'s DIRECTION —
# 2026-09-01, user: "the people in windows look completely wrong, they're
# just standing straight." Coordinator: "empirical search instead of
# derivation ... one render settles the UE/USD mapping question that three
# rounds of paper derivation have not." See `scene_generator._HUMAN_POSES`'
# own "THROWAWAY" block for the eight names and what each one hinges on.
WINDOW_POSE_VARIANTS = ("lw_pelvis_negx", "lw_pelvis_posx",
                       "lw_pelvis_negy", "lw_pelvis_posy",
                       "lw_spine_negx", "lw_spine_posx",
                       "lw_mixed_negx", "lw_mixed_posx")


def _pass_window_pose_experiment(sol, plan):
    """One figure per `WINDOW_POSE_VARIANTS` name, at CONSECUTIVE real
    openings of the bench's intact (non-collapse) building — gated on
    `cfg["window_pose_experiment"]`, off by default; never part of the real
    census, never budgeted, never rule-checked by the normal gate. Bypasses
    the flame/eligibility machinery `_pass_window` runs — this is a one-off
    diagnostic bench row, not a placement the ground truth ships.

    Picks the TALLEST eligible (non-collapse-level) building with real
    measured openings (`sidecar_grid`) on some side, and a storey with at
    least `len(WINDOW_POSE_VARIANTS)` openings so every variant gets its
    own bay rather than doubling up — falls back to the derived grid, and
    to fewer variants, rather than placing nothing.
    """
    cfg, rng = sol.cfg, sol.rng
    # NOT JUST "not F5c" — a roof-collapsed/gutted F5/F6 shell is not what
    # the coordinator meant by "the intact F3 building" either (found
    # 2026-08-31: excluding only `collapse_levels` picked the bench's GAC
    # F5 tower over its actual F3 kit apartment). Excludes every damage
    # level worse than F3 the same way `roof_ok_levels` restricts itself to
    # F1-F3 for the SAME "is this deck/facade still a normal building"
    # reason, then prefers the SHORTEST eligible building — the intact F3
    # kit apartment over any taller shell that happens to still qualify —
    # so consecutive real bays land on one recognisable facade.
    cands = [b for b in sol.buildings if b.level in ("F1", "F2", "F3")]
    if not cands:
        plan.refuse("window_pose_experiment_no_building")
        return 0
    cands.sort(key=lambda b: b.H)
    made = 0
    for b in cands:
        # EVERY STOREY, NOT `window_storeys()` — this pass is a pose
        # diagnostic on a real facade, not a fire-eligible-opening claim,
        # so it is deliberately exempt from both the fire-band floor and
        # the `window_top_storeys_excluded` cap those exist to enforce for
        # the real census (found 2026-09-01: gating this pass on `window_
        # storeys()` left the bench's OWN F3 building with zero eligible
        # storeys once the top-3 cap landed, since its real sidecar band
        # already reaches storey 8 of 11 — the experiment would have had
        # nowhere to place a single figure on the one building it exists
        # to test poses against).
        storeys = list(range(b.n_storeys))
        best_ops, best_side, best_storey = [], None, None
        for side in (list(b.sides) or ["S", "N", "E", "W"]):
            for storey in storeys:
                ops = openings_for_side(b.rec, side, storey, cfg, b.doc)
                if len(ops) > len(best_ops):
                    best_ops, best_side, best_storey = ops, side, storey
        if not best_ops:
            continue
        n = min(len(WINDOW_POSE_VARIANTS), len(best_ops))
        nx, ny = side_normal_world(best_side, b.yaw)
        inset = float(cfg["window_stand_inset_m"])
        for k in range(n):
            pose = WINDOW_POSE_VARIANTS[k]
            op = best_ops[k]
            px, py, _z = _face_point(op["fr"], op["u"], 0.0, -inset)
            f_yaw = float(op["fr"][2])
            fnx, fny = math.sin(f_yaw), -math.cos(f_yaw)
            z = op["floor_z"]
            usd, pose, rigged = _pick_human(rng, pose, allow_posed=False)
            plan.add({
                "cls": "window", "group": plan.next_group(), "usd": usd,
                "rigged": rigged, "x": round(px, 3), "y": round(py, 3),
                "z": round(z, 3),
                "yaw_deg": round(math.degrees(math.atan2(fny, fnx)), 1),
                "pose": pose, "prone": False, "seat": None,
                "z_mode": "floor", "alive": True, "needs_bench": True,
                "variant": "pose_experiment", "protrusion_m": 0.0,
                "inset_m": inset, "sill_z": round(op["z_sill"], 3),
                "head_z": round(op["z_head"], 3),
                "floor_z": round(op["floor_z"], 3),
                "side": best_side, "storey": best_storey,
                "openings_source": op["source"],
                "building_i": b.i, "building_cell": b.cell,
                "building_level": b.level,
                "building_sides": list(b.sides),
                "reason": ("WINDOW POSE ROW variant {0!r} at opening {1} of "
                          "{2} ({3}), storey {4} — throwaway, see "
                          "scene_generator._HUMAN_POSES".format(
                              pose, k, b.level, best_side, best_storey)),
            })
            sol.placed.append((px, py))
            made += 1
        break
    if made == 0:
        plan.refuse("window_pose_experiment_no_openings")
    return made


def _roof_seat_z(b, x, y):
    """`(z, source, needs_bench)` for a roof-class figure at `(x, y)` on
    building `b`.

    ITEM 4, 2026-08-31 user review: "Some buildings have railings so the
    roof is below that lip and people look like they're floating rn." Prefers
    a LOCAL geometry sample (`local_roof_z`, real mesh points near this exact
    (x, y) rather than one global scalar for the whole deck) whenever the
    building has a local bake USD and the sample is not too sparse to trust;
    falls back to `(b.deck_z, b.deck_source)` otherwise. KIT bakes carry a
    sibling `.usd` in `city_138` just like gac/dtc ones (MEASURED, the bench
    re-solve: 6 of 9 roof-class figures on `kit_apartment_tall_F3_o4_SNW_
    s758` came back `local_mesh`), so the fallback is not a per-kind rule —
    it fires per-CANDIDATE, whenever the sparse window genuinely finds no
    mesh near this (x, y), and a sparse window means "no real mesh near
    here", not "the deck is at this exact height". On GAC buildings whose
    recorded `deck_z` does not match the local surface at all (the
    SM_Building_11 discrepancy in `local_roof_z`'s own account) the tight
    band finds nothing everywhere and the WHOLE building rides the global
    scalar — conservative by design, and the bench relaunch is where that
    building's deck gets looked at.
    """
    usd_path = bake_usd_path(b.doc) if b.doc else None
    if usd_path:
        z = local_roof_z(usd_path, x, y, b.deck_z)
        if z is not None:
            return z, "local_mesh", False
    return b.deck_z, b.deck_source, b.deck_source not in DECK_Z_BENCH_FREE_SOURCES


def _roof_interior_point(sol, b, cfg, side, rng, e_min, extra_ok=None):
    """One point in the MIDDLE of a roof deck, kept off every parapet by a
    hard `e_min`, leaning mildly toward `side` for diversity between more
    than one group on the same deck — or `None`.

    ITEM 4, 2026-08-31 user review: "don't need to be on the ledge. Centre
    is fine." The old geometry inset a group `roof_edge_band_m` in from ONE
    chosen edge and hugged it; this draws over the WHOLE interior of the
    footprint instead (`+-0.6` of each half-extent, `along` and `out` both),
    with a small lean toward `side` (`SIDE_NORMAL[side]`, the same unit
    vector `_pass_roof`'s look-out yaw already uses) so two groups on one
    deck do not land on top of each other. `e_min` is still enforced as a
    HARD minimum clearance from every wall, not a band — nobody stands on
    the coping either way.
    """
    bx, by = SIDE_NORMAL[side]
    half_w, half_d = b.W / 2.0, b.D / 2.0
    for _try in range(48):
        lx = rng.uniform(-0.60, 0.60) * half_w + bx * 0.16 * half_w
        ly = rng.uniform(-0.60, 0.60) * half_d + by * 0.16 * half_d
        clear = min(half_w - abs(lx), half_d - abs(ly))
        if clear < e_min:
            continue
        dx, dy = _rot(lx, ly, b.yaw)
        x, y = b.x + dx, b.y + dy
        if not sol.in_region(x, y) or not sol.spaced(x, y):
            continue
        if extra_ok is not None and not extra_ok(x, y):
            continue
        return x, y, clear
    return None


def _pass_roof(sol, plan, budget):
    """2-4 in the MIDDLE of the deck, on a building whose roof is intact.
    Section 5b."""
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
        e_min = float(cfg["roof_edge_band_m"][0])
        placed_here = 0
        for k in range(want):
          # ONE DRAW PER MEMBER IS NOT ENOUGH. Without a retry the first
          # real-city run put a single figure on each of three roofs (7
          # `group_member` refusals against a budget of 9): two members
          # landing within `min_sep_m` of each other on a small deck loses
          # one outright, and a roof group of one is the lone-figure problem
          # again. Same expanding retry the street groups use.
          found = _roof_interior_point(sol, b, cfg, side, rng, e_min)
          if found is None:
            plan.refuse("group_member")
            continue
          if True:            # keeps the member body at one indent level
            x, y, clear = found
            # FALLBACK TO `idle` BY DEFAULT (bench-v2 rejection: "still on
            # the edge in unnatural poses" — the user's own words). `idle`
            # is not the reason the review flagged this class (the review
            # also singled out placement, "on the edge"), but a stander in a
            # provably-correct pose while the new ones are unverified is
            # what "just spawn people only" is asking for. `roof_use_new_
            # pose=True` restores `stand_calm`/`wave_help` once proven; both
            # still need a skeleton, so a posed static stays ineligible
            # either way (it used to be eligible for plain `idle`/`walk`,
            # before the pose restriction).
            pose = (_weighted(rng, list(_CLASS_POSES["roof"]))
                    if cfg.get("roof_use_new_pose") else "idle")
            usd, pose, rigged = _pick_human(rng, pose, allow_posed=False)
            # Look out toward this group's assigned side, same yaw
            # `_roof_interior_point`'s own lean uses — a centred figure still
            # needs a facing, and away from the venting side is the doctrine
            # this class was built on (section 5b).
            nx, ny = side_normal_world(side, b.yaw)
            z, deck_source, needs_bench = _roof_seat_z(b, x, y)
            plan.add({
                "cls": "roof", "group": gid, "usd": usd, "rigged": rigged,
                "x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
                "yaw_deg": round(math.degrees(math.atan2(ny, nx)), 1),
                "pose": pose, "prone": False,
                "seat": None, "z_mode": "deck", "alive": True,
                # DECK Z NEEDS THE BENCH UNLESS THE SOURCE IS MEASURED
                # (`local_mesh` counts). See `deck_z()`'s 2026-08-31 account
                # of the 1.82 m bulkhead-cap error on `sidecar_top_z` —
                # `estimated` is the same guess with no bake at all behind
                # it.
                "needs_bench": needs_bench,
                "deck_source": deck_source, "roof_clear_m": round(clear, 2),
                "side": side,
                "building_i": b.i, "building_cell": b.cell,
                "building_level": b.level, "building_sides": list(b.sides),
                "reason": ("roof refuge in the middle of the deck ({0} m "
                           "clear of the nearest parapet) of a {1} building, "
                           "deck z {2:.1f} m ({3}); venting sides {4}"
                           .format(round(clear, 1), b.level, z,
                                   deck_source, ",".join(b.sides) or "-")),
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


def _roof_victim_street_sides(sol, b, cfg):
    """`far_sides()`'s own away-from-the-fire order, with the STREET-FACING
    edges moved to the front. Section 5b2.

    Sampling a point `roof_victim_street_test_m` past each non-venting edge
    and asking `sol.ground` whether it reads as `road`/`sidewalk`/`paved`
    tells the difference between an edge overlooking the street and one
    overlooking a party wall, a service yard or a courtyard — a distinction
    `far_sides()` on its own has no way to make, since it only knows which
    way the fire is venting.
    """
    test_m = float(cfg["roof_victim_street_test_m"])
    ranked = b.far_sides()
    street, other = [], []
    for s in ranked:
        cx, cy, _half = face_center(b.rec, s)
        nx, ny = side_normal_world(s, b.yaw)
        tx, ty = cx + nx * test_m, cy + ny * test_m
        if sol.ground.at(tx, ty) in ("road", "sidewalk", "paved"):
            street.append(s)
        else:
            other.append(s)
    return (street + other) or list(ranked)


def _clear_of_smoke(b, x, y, min_dist):
    """Is `(x, y)` at least `min_dist` from every one of `b`'s own bake-
    sidecar smoke seats (`interior` AND `roof` plume groups)?

    `fire_bake.sidecar()`'s own `seats` dict (`{"interior": [...],
    "roof": [...]}`, each entry at least `x`/`y`/`z` in the BAKE'S OWN
    frame) is rotated into the CITY frame exactly the way `fire_bake.place`
    moves it: rotate `(x, y)` about the origin by the cell yaw, translate by
    the building's own `(x, y)`, leave `z` untouched (a yaw about the
    vertical axis never moves height, and `place`'s own docstring says so).
    Reproduced with `_rot` rather than calling `fire_bake.place` so this
    module keeps no import-time dependency on it, the same discipline
    `_face_point`/`place_frame` already follow for the opening frames.
    """
    seats = (b.doc or {}).get("seats") or {}
    for group in ("interior", "roof"):
        for s in seats.get(group) or []:
            wx, wy = _rot(float(s.get("x", 0.0)), float(s.get("y", 0.0)),
                          b.yaw)
            wx, wy = wx + b.x, wy + b.y
            if math.hypot(x - wx, y - wy) < float(min_dist):
                return False
    return True


def _pass_roof_victim(sol, plan, budget):
    """2-3 stranded near the roof edge of a building whose deck is INTACT —
    section 5b2. `roof`'s own edge geometry (inset from the parapet, kept off
    the corners), with the edge choice biased toward the street and every
    candidate kept clear of the building's own smoke.
    """
    cfg, rng = sol.cfg, sol.rng
    cands = []
    for b in sol.buildings:
        ok, why = b.roof_victim_ok(cfg)
        if ok:
            cands.append(b)
        else:
            plan.refuse("roof_victim:" + why)
    if budget <= 0 or not cands:
        return 0
    made, guard = 0, 0
    used = {}
    order = sorted(cands, key=lambda b: -b.H)
    bi = 0
    cap = int(cfg["roof_victim_max_groups_per_building"])
    min_smoke = float(cfg["roof_victim_min_smoke_dist_m"])
    while made < budget and guard < budget * 8 + 24:
        guard += 1
        b = order[bi % len(order)]
        bi += 1
        if used.get(b.i, 0) >= cap:
            if all(used.get(c.i, 0) >= cap for c in order):
                plan.refuse("roof_victim_decks_exhausted")
                break
            continue
        nth = used.get(b.i, 0)
        used[b.i] = nth + 1
        ranked = _roof_victim_street_sides(sol, b, cfg)
        side = ranked[nth % len(ranked)]
        gid = plan.next_group()
        want = min(_group_size(cfg, "roof_victim", rng), budget - made)
        e_min = float(cfg["roof_victim_edge_band_m"][0])

        def _smoke_ok(x, y, b=b, min_smoke=min_smoke, plan=plan):
            if not _clear_of_smoke(b, x, y, min_smoke):
                plan.refuse("roof_victim_near_smoke")
                return False
            return True

        placed_here = 0
        for k in range(want):
          # SAME EXPANDING RETRY `roof` USES — one draw per member is not
          # enough on a small deck once the smoke keepout is added on top of
          # the corner/spacing rules.
          found = _roof_interior_point(sol, b, cfg, side, rng, e_min,
                                       extra_ok=_smoke_ok)
          if found is None:
            plan.refuse("group_member")
            continue
          if True:            # keeps the member body at one indent level
            x, y, clear = found
            # See `_pass_roof`'s identical fallback comment.
            pose = (_weighted(rng, list(_CLASS_POSES["roof_victim"]))
                    if cfg.get("roof_use_new_pose") else "idle")
            usd, pose, rigged = _pick_human(rng, pose, allow_posed=False)
            # Look out toward this group's assigned (street-biased) side,
            # same as `roof`.
            nx, ny = side_normal_world(side, b.yaw)
            z, deck_source, needs_bench = _roof_seat_z(b, x, y)
            plan.add({
                "cls": "roof_victim", "group": gid, "usd": usd,
                "rigged": rigged,
                "x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
                "yaw_deg": round(math.degrees(math.atan2(ny, nx)), 1),
                "pose": pose, "prone": False,
                "seat": None, "z_mode": "deck", "alive": True,
                "needs_bench": needs_bench,
                "deck_source": deck_source, "roof_clear_m": round(clear, 2),
                "side": side, "min_smoke_dist_m": min_smoke,
                "building_i": b.i, "building_cell": b.cell,
                "building_level": b.level, "building_sides": list(b.sides),
                "reason": ("stranded in the middle of the deck of a {0} "
                           "building (i={1}), {2} m clear of the nearest "
                           "parapet toward the {3} (street-facing) side, "
                           "deck z {4:.1f} m ({5}), >= {6:.0f} m clear of "
                           "every smoke seat"
                           .format(b.level, b.i, round(clear, 1), side,
                                   z, deck_source, min_smoke)),
            })
            sol.placed.append((x, y))
            placed_here += 1
            made += 1
        # NO SINGLETON, same reason as `roof`.
        if placed_here == 1:
            rec = plan.records[-1]
            plan.records.remove(rec)
            sol.placed.pop()
            plan.refuse("singleton_roof_victim_withdrawn")
            made -= 1
    return made


# ---------------------------------------------------------------------------
# REAL COVERING DEBRIS FOR A BURIAL FIGURE — reusing tornado_people's
# machinery. ITEM 2, 2026-08-31 user review, quoted: "the people must be
# partially visible through the rubble ... [not] part of the damage."
# Coordinator instruction, same review: "Look at tornado code for this:
# Partial burial it does it very well" — REUSE the covered-fraction rules and
# the seating-on-the-body's-own-crest geometry FROM `tornado_people` rather
# than reimplementing them, because that module's own review history (the
# 1 km tornado review's `max_covered_frac` 0.80 -> 0.55 correction, and the
# `_crest`/`_BODY_RISE` fix for "boards passing through raised knees") is
# what tuned every one of those numbers, and this module already CITES them
# ("tornado conventions, verbatim", section 5c) without ever having called
# the code that enforces them — `covered_frac`/`occlusion` were a DRAWN
# number with no debris authored to make it true.
#
# WHAT TRANSFERS AND WHAT DOES NOT. The seating math — `_crest` (a rigid
# piece rests on the body's own highest point over its footprint, not a
# flat-chest height), `_trim_spans` (a pattern is SHORTENED before any piece
# is authored so the module can never claim a body it has made invisible),
# `_cover_piece` (the flat/propped placement geometry: x/y/z/yaw/pitch/roll)
# — is pure body-and-debris-pile geometry and does not know or care what
# knocked the building down; `_union`, likewise, is just span arithmetic.
# What does NOT transfer is `tornado_people`'s STOCK: `_draw_cover_stock`
# draws sawn lumber and sheet goods sized for a tornado's plank field, and a
# fire collapse's windrow is masonry, concrete and charred structural debris
# — `_FIRE_COVER_STOCK` below is this module's own draw, sized for the outer
# quarter of a collapse windrow (`apron_band`, "the pile has thinned to a
# few tens of centimetres") rather than a fresh plank mat, and always the
# dark/char end of the palette (matching `fire_assembly_lib.APRON_CHAR_P`'s
# own rule for anything lying on the ground near a burnt building).
# ---------------------------------------------------------------------------

# `tornado_people._OCCLUSION`'s own spans, for the patterns fire_people's
# lighter-weighted vocabulary (module-level `OCCLUSION` above) actually
# draws from. Every `cover_at` in that table IS the length of the matching
# span here (`feet_shins` 0.30 <-> (0.00, 0.30), `legs` 0.52 <-> (0.00,
# 0.52), etc.) — which is how `OCCLUSION`'s own numbers were chosen: off
# this table, transcribed to a scalar, long before any code walked the span.
_FIRE_OCCLUSION_SPANS = {
    "none": None,
    "feet_shins": (0.00, 0.30),
    "legs": (0.00, 0.52),
    "midriff": (0.42, 0.70),
    "torso": (0.46, 0.84),
    "flank": "lateral",
}

# (class, weight, along_m, across_m, thick_m) — "along"/"across" are named
# for the BODY the same way `tornado_people._COVER_STOCK`'s are: `along` is
# the stretch of the body's own length the piece hides, `across` is its
# width crossing the body.
_FIRE_COVER_STOCK = (
    # `along` (the stretch of the body's own length ONE piece hides) is
    # capped at 0.40 m across every class, 2026-09-01 bench-v4 rejection
    # ("beyond material, make them read as debris: 2-4 smaller elongated
    # pieces per figure ... crossing the body, not one big slab") — the old
    # `rubble_slab` range (0.45-0.95 m) could cover a whole `feet_shins`
    # span (0.00-0.30 of reach, ~0.53 m at NOMINAL_HEIGHT_M) in ONE piece,
    # which is exactly the monolithic-slab read that was rejected.
    ("rubble_slab", 0.42, (0.22, 0.40), (0.55, 1.05), (0.06, 0.14)),
    ("brick_chunk", 0.34, (0.18, 0.34), (0.16, 0.30), (0.14, 0.24)),
    ("char_beam", 0.24, (0.16, 0.26), (0.90, 1.80), (0.10, 0.18)),
)


def _draw_fire_cover_stock(rng):
    r = rng.random() * sum(s[1] for s in _FIRE_COVER_STOCK)
    for (k, w, along, across, th) in _FIRE_COVER_STOCK:
        r -= w
        if r <= 0.0:
            return (k, rng.uniform(*along), rng.uniform(*across),
                    rng.uniform(*th))
    k, _w, along, across, th = _FIRE_COVER_STOCK[-1]
    return (k, rng.uniform(*along), rng.uniform(*across), rng.uniform(*th))


def _cover_burial(plan, rec, pose, x, y, ux, uy, reach, base_z, deck_z,
                  pattern, height, rng, lift):
    """Lay `pattern`'s covering pieces on one burial figure and write the
    REAL `covered_frac`/`boards` fields — `tornado_people._cover`, ported
    with a fire-appropriate stock draw (see the module note above).

    `base_z` is the body's own ground plane (its support surface, AFTER any
    sink — see `heap_z_at`/`_burial_record`); `deck_z` is the windrow's own
    top surface at this point, the same "propped" reference
    `tornado_people._cover_piece` uses to decide how far a propped piece
    rises before it reaches the body. Appends every authored piece to
    `plan.covering` and returns the covered spans (reach units), matching
    `tornado_people._cover`'s own return.

    `lift` is THIS RECORD's own `lying_lift(pose)` — the fire module's own
    ground-to-centreline rise, already used to author the body itself
    (`_placement_no_ctx`'s prone branch). CLAMPED AGAINST IT, bench-v3
    REJECTION: `tornado_people._BODY_RISE`'s per-pose bands (calibrated for
    a body lying on a debris-BOARD mat, tornado's own context) gave
    `lying_curled_l` a band crest of 0.344 H — 0.61 m at this module's
    NOMINAL_HEIGHT_M — while THIS pose's own `lying_lift` (0.115 H, "half
    the body BREADTH", the number the body is actually authored with) is
    0.205 m: a 0.41 m gap between where a piece was solved to rest and
    where the body it is supposed to be covering actually sits, MEASURED on
    the synthetic fixture's own record 68 (`burial_covering_contacts_the_
    body` failing before this clamp existed). Two different tables, two
    different calibrations, and nothing before this reconciled them. The
    ceiling — `lift * 2.2`, i.e. is a bit more than a body's own full
    thickness/breadth above the ground it is lying on — still lets a raised
    knee or a drawn-up side pose earn SOME extra clearance over the flat
    minimum, it just cannot run away to a gap a camera reads as floating.
    """
    from . import tornado_people as tp

    cap = float(plan.cfg.get("max_covered_frac", MAX_COVERED_FRAC))
    span = _FIRE_OCCLUSION_SPANS.get(pattern)
    flank = (span == "lateral")
    if span is None:
        rec["occlusion"] = "none"
        rec["covered_frac"] = 0.0
        rec["boards"] = 0
        return ()
    if flank:
        lo = rng.uniform(0.05, 0.22)
        hi = min(1.0, lo + rng.uniform(0.50, 0.72))
        trimmed = ((lo, hi),)
    else:
        trimmed = tp._trim_spans((span,), cap)
    if not trimmed:
        rec["occlusion"] = "none"
        rec["covered_frac"] = 0.0
        rec["boards"] = 0
        return ()

    laid, made = [], 0
    for (lo, hi) in trimmed:
        s = lo
        guard = 0
        while s < hi - 0.02 and guard < 10:
            guard += 1
            klass, along, across, thick = _draw_fire_cover_stock(rng)
            w = along / max(reach, 1e-6)
            if s + w > hi:
                w = hi - s
                along = w * reach
                if along < 0.10:
                    break
            t = s + w * 0.5
            px = x + ux * t * reach
            py = y + uy * t * reach
            # ON THE HIGHEST THING UNDER IT, exactly `tornado_people._cover`'s
            # own rule — a rigid piece bears on the body's crest across its
            # own footprint, not on the flat-chest height everywhere. CLAMPED
            # to `lift * 2.2` above `base_z` — see this function's own
            # docstring for the 0.41 m gap this closes.
            crest = min(tp._crest(pose, s, s + w, height), lift * 2.2)
            top_z = base_z + crest
            propped = rng.random() < 0.30
            sp = tp._cover_piece(px, py, ux, uy, along, across, thick,
                                 klass, top_z, deck_z, rng, propped)
            sp["for"] = "fire_burial"
            # RECORD ID, so a covering piece can be traced back to the ONE
            # body it covers — 2026-08-31 bench-v3 rejection: without this,
            # "does this piece touch its own body" cannot be checked per
            # figure, only "does SOME piece touch SOME body somewhere",
            # which cannot catch a piece authored against the wrong figure's
            # geometry.
            sp["over_record_id"] = rec.get("id")
            sp["cover_top_z"] = round(top_z, 3)
            plan.covering.append(sp)
            laid.append((s, s + w))
            made += 1
            # A small gap sometimes — a fitted lid is not what real debris
            # does, `tornado_people._cover`'s own comment.
            s += w * rng.uniform(0.85, 1.10)

    cov = tp._union(laid)
    if flank:
        cov *= 0.5                      # half the WIDTH, so half the cover
    cov = min(cov, cap)
    rec["occlusion"] = pattern
    rec["covered_frac"] = round(cov, 3)
    rec["boards"] = made
    return tuple(laid)


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
    # SAME CEILING THE STREET CLASSES CARRY, AND FOR THE SAME REASON
    # (`test_39b`): `apron_run_m` is `apron_spread * H` with NO cap of its
    # own, so a wide enough / tall enough F5c building can throw a burial
    # figure further from the wall than `FC_PEOPLE_MAX_DIST_M`
    # (`urban_fire_city_launch_script.py`, default 120 m) will keep on the
    # stage — the launcher's cull is a GUARD on a datum this solver already
    # owns (distance to the burning FOOTPRINT), not a second opinion, so it
    # must never be the thing that drops a figure this solver placed on
    # purpose. `max_wall_dist_m` (60 m) is comfortably under that guard for
    # every real apron run measured so far and is reused here rather than a
    # second constant, so `check_rules`'s `crowd_belongs_to_its_building`
    # (rule 12b) covers burial figures the moment they carry `d_wall_m` too.
    d_wall = dist_to_obb(x, y, b.x, b.y, b.W, b.D, b.yaw)
    if d_wall > float(cfg["max_wall_dist_m"]):
        plan.refuse("too_far_from_building")
        return None

    pose = _weighted(rng, list(_LYING_POSES))
    pattern, _frac = _occlusion(rng)
    depth = rng.uniform(*cfg["out_depth_m"])
    surf = apron_surface_z(t, depth)
    usd, pose, _rigged = _pick_human(rng, pose, allow_posed=False)
    lift = lying_lift(pose)
    yaw = rng.uniform(0.0, 360.0)
    # SUNK INTO THE PILE, NOT JUST RESTING ON IT — `tornado_people`'s own
    # `sink_frac` (item 2, 2026-08-31: "torso/arm visible above the heap
    # surface"). A fraction of the BODY's own depth, not the windrow's — the
    # windrow is already thin out here (`apron_band`'s outer quarter), and
    # sinking by the windrow's depth would put a body's spine through the
    # slab it is lying on. `covered_frac` never counts this (it counts
    # authored PIECES; see `_cover_burial`'s own comment on `flank`), so
    # `sunk_frac` is recorded separately, `tornado_people`'s own convention.
    body_depth = 2.0 * _BODY_HALF_DEPTH_M
    sink_lo, sink_hi = cfg.get("sink_frac", (0.0, 0.18))
    sink = body_depth * rng.uniform(float(sink_lo), float(sink_hi))
    base_z = surf - sink
    rec = plan.add({
        "cls": cls, "group": gid, "usd": usd, "rigged": True,
        "x": round(x, 3), "y": round(y, 3),
        # `z` IS THE SUPPORT SURFACE ON EVERY CLASS, without exception — the
        # debris top here (SUNK by `sink`, see above), the ground/kerb for a
        # stander, the sill for a sitter, the deck for a roof group. It is
        # NOT the authored prim z: the lying LIFT (half the body depth, or
        # 0.115 H on a side) is added by `to_placements`, which is also
        # where `people._human_placement` would add it from the rig's
        # MEASURED depth. Baking the lift in here made `z` mean two
        # different things in two classes and would have been applied
        # twice by any converter that trusted the contract.
        "z": round(base_z, 3),
        "yaw_deg": round(yaw, 1), "pose": pose, "prone": True,
        "roll_deg": LYING_ROLL[pose], "pitch_deg": LYING_SPIN.get(pose, 0.0),
        "seat": None, "z_mode": "debris", "alive": False,
        # THE BENCH IS THE ONLY WAY TO CHECK ONE OF THESE. The tornado skill
        # is unambiguous and nothing in a 2-D dry run discharges it.
        "needs_bench": True,
        "apron_t": round(t, 3), "apron_run_m": round(run, 2),
        "debris_depth_m": round(depth, 2), "surface_z": round(surf, 3),
        "sink_m": round(sink, 3),
        "sunk_frac": round(sink / max(body_depth, 1e-6), 3),
        "lift_m": round(lift, 3), "side": side,
        "d_wall_m": round(d_wall, 2),
        "building_i": b.i, "building_cell": b.cell,
        "building_level": b.level, "building_sides": list(b.sides),
        "reason": note,
    })
    # REAL COVERING DEBRIS, NOT JUST A DRAWN NUMBER — item 2, and the
    # coordinator's "look at tornado code" instruction. `author_burial_cover`
    # is on by default (see DEFAULTS); off degrades to the OLD metadata-only
    # `covered_frac`/`occlusion` from the draw above, for a caller that wants
    # the class without paying for the extra debris authoring.
    if cfg.get("author_burial_cover", True):
        from . import tornado_people as tp
        ux, uy = tp._body_axis(pose, yaw, LYING_ROLL[pose])
        _cover_burial(plan, rec, pose, x, y, ux, uy, NOMINAL_HEIGHT_M,
                     base_z, surf, pattern, NOMINAL_HEIGHT_M, rng, lift)
    else:
        rec["occlusion"] = pattern
        rec["covered_frac"] = _frac
        rec["boards"] = 0
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
# `interior_trapped` — section 5d, ADDED 2026-08-31
#
# ITEM 2, user review: "For partially collapsed buildings the people must be
# partially visible ... in the building itself visible through the broken
# parts." Then, on sight: "What about humans that passed out on the floors
# of partially collapsed buildings. You gotta have that" — and then, on the
# placement itself: "don't have them right at the edge though, they need to
# be a little inside at a safe distance."
#
# The class is two variants of the same situation, roughly evenly split
# (`interior_trapped_conscious_share`): a CONSCIOUS figure standing or
# leaning near the break, and an UNCONSCIOUS one lying where it fell on the
# surviving slab. Both are placed the same way — a slab, a failed elevation,
# a setback — and differ only in pose and attitude.
# ---------------------------------------------------------------------------
def _slab_z(b, standing=False):
    """`(z, storey)` of the surviving floor slab BELOW the fire's origin, or
    `(None, None)` if the sidecar cannot supply it.

    `fire_collapse.plan_partial_collapse`'s own invariant, stated verbatim in
    its source: "NOTHING BELOW THE FIRE'S ORIGIN, EVER." So storey
    `origin - 1` (clamped to 0 — if the fire started on the ground floor, the
    ground floor itself is the guaranteed surviving slab) always keeps its
    floor, and `masses.main.levels` (persisted by `fire_bake.sidecar()`, a
    full per-storey elevation array — NOT the lossy `spec`) gives its real
    world z. `fire.sides[0]` (also persisted) is the elevation the collapse
    actually failed on, in `plan_partial_collapse`'s own default (`sides =
    (side or fire_sides[0],)`) — the same inference this module already
    makes for smoke/window siting off that field elsewhere. Both are
    ESTIMATES in the sense that the exact plan-view span of the hole is not
    itself in the sidecar (`region[side]` never round-trips) — see
    `_interior_trapped_ok`'s `needs_bench`.

    `standing=True` GOES ONE STOREY FURTHER DOWN — bench-v4 ledge-stander
    complaint, 2026-09-01: `origin - 1` is the TOP surviving storey, right
    at the broken roof line, and a `stand_calm` figure planted there reads
    from outside as someone standing on the ledge of a collapsed building
    (dumped and confirmed against a real bench solve: the F5c office's own
    `interior_trapped` conscious figure sat at exactly `origin - 1`).
    Coordinator's own rule: "standing interior figures live at least one
    full storey below the broken roof line; lying figures may stay higher."
    `origin - 2`, clamped to 0 same as the base case — and if that clamp
    lands on the SAME storey `origin - 1` already clamped to (only possible
    when `origin <= 1`, i.e. there is no second storey to retreat to),
    returns `(None, None)`: a standing figure with nowhere lower to go is
    refused for this building rather than placed at the break anyway.
    """
    if not b.doc:
        return None, None
    main = (b.doc.get("masses") or {}).get("main")
    levels = main.get("levels") if isinstance(main, dict) else None
    origin = (b.doc.get("fire") or {}).get("origin")
    if not levels or origin is None:
        return None, None
    base_storey = max(0, min(len(levels) - 1, int(origin) - 1))
    if standing:
        storey = max(0, min(len(levels) - 1, int(origin) - 2))
        if storey >= base_storey:
            return None, None
    else:
        storey = base_storey
    try:
        return float(levels[storey]), storey
    except (TypeError, ValueError, IndexError):
        return None, None


def _interior_trapped_ok(b):
    """Eligibility for `interior_trapped`.

    Restricted to `collapse_levels` (F5c) exactly like `casualty_apron`: an
    F5/F6 shell has lost its floors INWARD on every side (`roof_debris`'s own
    account), so there is no single "lost elevation" with a slab behind it —
    only a PARTIAL collapse has that shape. Requires a real bake sidecar
    (`_slab_z` needs `masses.main.levels`/`fire.origin`) and at least one
    recorded failed elevation (`fire.sides`) — this class places a figure to
    a specific storey inside a specific building, too particular a claim to
    synthesise the way a window's default opening grid can be.
    """
    if b.level not in DEFAULTS["collapse_levels"]:
        return False
    z, _storey = _slab_z(b)
    return z is not None and bool(b.sides)


def _interior_sightline_ok(setback_m, storey_period_m, min_deg, eye_h_m):
    """Can an oblique drone camera, looking down at least `min_deg` above the
    horizontal, still see a figure `setback_m` inside the broken-open
    elevation, without the intact floor ONE STOREY UP (the origin's own
    floor — the fire consumed the walls above this slab, not that floor)
    occluding the view?

    A conservative planar model, not a render: the opening's headroom is
    treated as one storey (`storey_period_m`, this building's own measured
    metres-per-storey) and the figure's visible point as `eye_h_m` above the
    slab it stands or lies on — a torso height, not the crown, because a
    torso is what the ground truth's occlusion classes actually score. The
    line from a camera at `min_deg` grazing the top of that headroom reaches
    `clear / tan(min_deg)` metres in before it is blocked by the floor above;
    a deeper setback than that is refused rather than assumed visible.
    """
    clear = float(storey_period_m) - float(eye_h_m)
    if clear <= 0.05:
        return False
    max_depth = clear / math.tan(math.radians(float(min_deg)))
    return float(setback_m) <= max_depth


# Poses for the CONSCIOUS variant — standing or leaning near the break, not
# fully composed (`stand_calm`) and not signalling for a ladder
# (`wave_help` is a roof/window gesture, not an interior one): mostly calm,
# some crouched over the opening looking for a way down.
_INTERIOR_CONSCIOUS_POSES = (("stand_calm", 0.55), ("crouch", 0.25),
                             ("wave_help", 0.20))
# Poses for the UNCONSCIOUS variant — "passed out", `prone=True`. `buried_
# reach` (item 1's new pose, "for partial burial") reads just as well for a
# figure that collapsed mid-motion as for one under rubble; `lying_supine`/
# `lying_prone` cover the rest so a slab does not show the same silhouette
# every time.
_INTERIOR_UNCONSCIOUS_POSES = (("buried_reach", 0.40), ("lying_supine", 0.30),
                               ("lying_prone", 0.30))
# Eye height above the slab used for the sightline check, by variant — a
# lying figure's highest visible point is much lower than a standing one's.
_INTERIOR_EYE_H_M = {"conscious": 1.40, "passed_out": 0.30}


def _pass_interior_trapped(sol, plan, budget):
    """1-2 figures per partial-collapse building, visible INSIDE through the
    broken-open wall, on the surviving floor slab. Section 5d.

    Roughly `interior_trapped_conscious_share` conscious (standing/leaning,
    facing the opening) and the rest passed out (lying where they fell —
    small random yaw and a lateral jitter off the setback line, NOT centred
    and squared to the wall, because a collapsed figure was not arranged).
    Every candidate is SET BACK from the broken edge
    (`interior_setback_m`, ~1.5-3 m) and sightline-checked
    (`_interior_sightline_ok`) rather than placed at the lip.
    """
    cfg, rng = sol.cfg, sol.rng
    cands = [b for b in sol.buildings if _interior_trapped_ok(b)]
    if not cands:
        plan.refuse("no_interior_trapped_building")
    if budget <= 0 or not cands:
        return 0
    conscious_share = float(cfg["interior_trapped_conscious_share"])
    lo_setback, hi_setback = cfg["interior_setback_m"]
    lat_frac = float(cfg["interior_lateral_frac"])
    min_deg = float(cfg["interior_min_sightline_deg"])
    made, guard = 0, 0
    while made < budget and guard < budget * 12 + 24:
        guard += 1
        b = rng.choice(cands)
        conscious = rng.random() < conscious_share
        # STANDING GOES ONE STOREY LOWER — bench-v4 ledge-stander fix, see
        # `_slab_z`'s own account. A building with nowhere lower to retreat
        # to (`origin <= 1`) cannot host a CONSCIOUS figure at all; rather
        # than waste the draw, it becomes a passed-out one instead — a
        # lying figure at the top surviving storey does not read as a
        # ledge-stander, so it does not need the lower slab.
        slab_z, storey = _slab_z(b, standing=conscious)
        if slab_z is None and conscious:
            conscious = False
            slab_z, storey = _slab_z(b, standing=False)
        if slab_z is None:
            continue
        side = rng.choice(list(b.collapse_sides()))
        cx, cy, half = face_center(b.rec, side)
        nx, ny = side_normal_world(side, b.yaw)
        tx, ty = -ny, nx
        period = storey_period(b.rec, b.doc)
        variant = "conscious" if conscious else "passed_out"
        eye_h = _INTERIOR_EYE_H_M[variant]

        x = y = setback = None
        for _try in range(24):
            cand_setback = rng.uniform(lo_setback, hi_setback)
            if not _interior_sightline_ok(cand_setback, period, min_deg,
                                          eye_h):
                plan.refuse("interior_no_sightline")
                continue
            along = rng.uniform(-lat_frac, lat_frac) * half
            cx2 = cx - nx * cand_setback + tx * along
            cy2 = cy - ny * cand_setback + ty * along
            if not sol.in_region(cx2, cy2):
                plan.refuse("off_plate")
                continue
            # THE ONE PLACE THIS CLASS *WANTS* TO BE INSIDE THE FOOTPRINT —
            # it is the `interior_trapped` half of `AERIAL_EXEMPT_CLASSES`,
            # the counterpart of `window`/`roof` carrying their own z.
            if not point_in_obb(cx2, cy2, b.x, b.y, b.W, b.D, b.yaw,
                                margin=0.0):
                plan.refuse("interior_off_slab")
                continue
            if not sol.spaced(cx2, cy2):
                plan.refuse("too_close")
                continue
            x, y, setback = cx2, cy2, cand_setback
            break
        if x is None:
            continue

        gid = plan.next_group()
        if conscious:
            pose = _weighted(rng, list(_INTERIOR_CONSCIOUS_POSES))
            usd, pose, rigged = _pick_human(rng, pose, allow_posed=False)
            # Face the opening — the same "look out" framing `roof` uses.
            yaw = math.degrees(math.atan2(ny, nx))
            rec = plan.add({
                "cls": "interior_trapped", "group": gid, "usd": usd,
                "rigged": rigged, "x": round(x, 3), "y": round(y, 3),
                "z": round(slab_z, 3), "yaw_deg": round(yaw, 1),
                "pose": pose, "prone": False, "seat": None,
                "z_mode": "slab", "alive": True, "needs_bench": True,
                "variant": "conscious", "storey": storey,
                "setback_m": round(setback, 2), "side": side,
                "min_sightline_deg": min_deg,
                "building_i": b.i, "building_cell": b.cell,
                "building_level": b.level, "building_sides": list(b.sides),
                "reason": ("standing on the surviving storey-{0} slab of a "
                           "partially collapsed ({1}) building, {2:.1f} m "
                           "in from the broken {3} elevation, facing the "
                           "opening".format(storey, b.level, setback, side)),
            })
        else:
            pose = _weighted(rng, list(_INTERIOR_UNCONSCIOUS_POSES))
            usd, pose, _rigged = _pick_human(rng, pose, allow_posed=False)
            # COLLAPSED WHERE THEY FELL — a small random yaw, not squared to
            # the wall the way a placed prop would be.
            yaw = rng.uniform(0.0, 360.0)
            lift = lying_lift(pose)
            rec = plan.add({
                "cls": "interior_trapped", "group": gid, "usd": usd,
                "rigged": True, "x": round(x, 3), "y": round(y, 3),
                "z": round(slab_z, 3), "yaw_deg": round(yaw, 1),
                "pose": pose, "prone": True,
                "roll_deg": LYING_ROLL[pose],
                "pitch_deg": LYING_SPIN.get(pose, 0.0),
                "seat": None, "z_mode": "slab", "alive": False,
                "needs_bench": True, "variant": "passed_out",
                "storey": storey, "setback_m": round(setback, 2),
                "side": side, "lift_m": round(lift, 3),
                "min_sightline_deg": min_deg,
                "building_i": b.i, "building_cell": b.cell,
                "building_level": b.level, "building_sides": list(b.sides),
                "reason": ("passed out on the surviving storey-{0} slab of "
                           "a partially collapsed ({1}) building, {2:.1f} m "
                           "in from the broken {3} elevation"
                           .format(storey, b.level, setback, side)),
            })
        sol.placed.append((x, y))
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
        # THROWAWAY pose-row records are diagnostic, not a real protrusion
        # claim — exempt so the bench row actually ships (2026-09-01).
        if rec.get("variant") == "pose_experiment":
            return True, ""
        if float(rec.get("protrusion_m", 0.0)) < MIN_PROTRUSION_M:
            return False, "window_no_protrusion"
        return True, ""
    if cls in ("roof", "roof_victim"):
        if rec.get("z_mode") != "deck":
            return False, "{0}_not_on_deck".format(cls)
        return True, ""
    if cls in ("casualty_apron", "roof_debris"):
        if float(rec.get("covered_frac", 0.0)) > MAX_COVERED_FRAC:
            return False, "over_covered"
        if sol.in_any_footprint(rec["x"], rec["y"], margin=0.0):
            return False, "under_intact_shell"
        return True, ""
    if cls == "interior_trapped":
        # THE INVERSE OF EVERY OTHER CLASS'S FOOTPRINT CHECK — this one
        # WANTS to be inside its own building (on the slab), which is the
        # whole reason it is in `AERIAL_EXEMPT_CLASSES`.
        if rec.get("z_mode") != "slab":
            return False, "interior_not_on_slab"
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
        # A manifest record skipped because it is no longer THIS dump's own
        # building — see `_Solver.__init__` / `_manifest_matches_dump`. Never
        # a crash: `n_burning` above is already the SURVIVING count, so a
        # fully-stale manifest degrades to an empty plan rather than raising.
        "n_manifest_records": len(manifest.get("records") or []),
        "manifest_records_skipped": len(sol.skipped_records),
        "manifest_records_skipped_by_reason": {
            r: sum(1 for s in sol.skipped_records if s["reason"] == r)
            for r in sorted({s["reason"] for s in sol.skipped_records})},
    }
    plan = Plan(cfg, meta)

    # --- budgets, with degradation ------------------------------------
    shares = dict(cfg["shares"])
    street_on = bool(cfg.get("street_classes"))
    eligible = {
        # ITEM 3: the street classes are ineligible OUTRIGHT when
        # `street_classes` is off — not "no burning buildings", a policy
        # choice, and `plan.degraded` (below) reports it as one rather than
        # conflating it with a class that genuinely has nowhere to go.
        "evacuee": street_on and n_burn > 0,
        "onlooker": street_on and n_burn > 0,
        "at_car": street_on and n_burn > 0,
        "window": any(b.window_storeys(cfg) for b in sol.buildings),
        "roof": any(b.roof_ok(cfg)[0] for b in sol.buildings),
        "roof_victim": any(b.roof_victim_ok(cfg)[0] for b in sol.buildings),
        "casualty_apron": any(b.level in cfg["collapse_levels"]
                              for b in sol.buildings),
        "roof_debris": any(b.roof_collapsed
                           and b.level in cfg["collapse_levels"]
                           for b in sol.buildings),
        "interior_trapped": any(_interior_trapped_ok(b)
                                for b in sol.buildings),
    }
    give_back = 0.0
    for c in CLASSES:
        if not eligible.get(c):
            give_back += shares.get(c, 0.0)
            plan.degraded[c] = shares.get(c, 0.0)
            shares[c] = 0.0
    # THE GIVE-BACK GOES TO WHATEVER IS STILL ELIGIBLE, NOT A HARD-CODED
    # PAIR. `_FALLBACK_CLASSES` (evacuee/onlooker) used to be a safe sink
    # because street classes were always eligible whenever ANY building was
    # burning — but with `street_classes` off by default (item 3) they are
    # now ROUTINELY the ones giving their own share back, and redistributing
    # into a pair that just zeroed itself is a silent loss (the giveback
    # computes `shares[c] * 0 / 0-ish` and vanishes). Spread it, weighted by
    # each class's own remaining share, over every class this run actually
    # kept eligible — `window`/`roof`/`roof_victim`/`casualty_apron`/
    # `roof_debris`/`interior_trapped` when street is off, the historical
    # pair (plus everything else) when it is on.
    if give_back > 0.0:
        sinks = [c for c in CLASSES if eligible.get(c) and shares.get(c, 0.0) > 0.0]
        base = sum(shares[c] for c in sinks) or 1.0
        for c in sinks:
            shares[c] += give_back * shares[c] / base
    ssum = sum(shares.values()) or 1.0
    budget = {c: int(round(total * shares[c] / ssum)) for c in CLASSES}
    meta["budget"] = dict(budget)
    meta["shares_effective"] = {c: round(shares[c] / ssum, 4) for c in CLASSES}
    meta["street_classes"] = street_on

    # --- the passes, in a fixed order ---------------------------------
    # Order matters only through the shared spacing index and the shared
    # rng: the 3-D classes go FIRST so a street group can never take a
    # position that a window or roof figure would have wanted (they cannot
    # move; a street group can).
    _pass_window(sol, plan, budget["window"])
    _pass_roof(sol, plan, budget["roof"])
    _pass_roof_victim(sol, plan, budget["roof_victim"])
    _pass_casualty_apron(sol, plan, budget["casualty_apron"])
    _pass_roof_debris(sol, plan, budget["roof_debris"])
    _pass_interior_trapped(sol, plan, budget["interior_trapped"])
    # THROWAWAY window-pose bench row, 2026-09-01 — off by default, no
    # budget, not part of the census; see `_pass_window_pose_experiment`.
    if cfg.get("window_pose_experiment"):
        _pass_window_pose_experiment(sol, plan)
    if street_on:
        _pass_street(sol, plan, "evacuee", budget["evacuee"],
                     cfg["evacuee_band"], float(cfg["upwind_cos_evacuee"]))
        _pass_street(sol, plan, "onlooker", budget["onlooker"],
                     cfg["onlooker_band"], float(cfg["upwind_cos_onlooker"]))
        _pass_at_car(sol, plan, budget["at_car"])

    # --- REFLOW -----------------------------------------------------------
    # A class can be STARVED rather than degraded: `roof` has exactly one
    # eligible deck on the real seed-4 city, `window` only two eligible
    # buildings, so both spend less than their budget however hard they try.
    # Leaving the shortfall unspent shrinks the scene's head count for a
    # reason that has nothing to do with how many people were there.
    #
    # WITH STREET CLASSES ON, the shortfall is handed to them, as before —
    # they always have somewhere to go. WITH THEM OFF (the default), there
    # is no damage-tied class guaranteed to have room for an arbitrary
    # top-up (a slab holds two or three figures, not an open lot), so the
    # shortfall is left unspent rather than forced somewhere that would
    # overcrowd a keepout-bound class — the same "the head count is meant to
    # drop" precedent `disaster.people`'s `min_burn_age_s` gate set. Recorded
    # either way, because "the roof class placed 3 of 9" is a fact a
    # reviewer needs and a silent top-up (or a silent non-top-up) would hide
    # it.
    short = total - len(plan.records)
    plan.meta["shortfall_before_reflow"] = short
    if short > 0 and street_on:
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
    # REMAP `over_record_id` ALONGSIDE THE REINDEX, not after it and not
    # separately — `_cover_burial` tags each covering piece with its OWNING
    # record's `id` at the moment it is authored, which is BEFORE this
    # filter can drop an earlier record and shift every id after it. Build
    # the old-id -> new-id map from the SAME loop that assigns the new ids,
    # so the two can never disagree, then drop any piece whose owner did not
    # survive the filter — a piece over a body the ground truth no longer
    # ships is not evidence of anything.
    old_to_new = {}
    for k, r in enumerate(plan.records):
        old_to_new[r["id"]] = k
        r["id"] = k
    if getattr(plan, "covering", None):
        remapped = []
        for sp in plan.covering:
            old_id = sp.get("over_record_id")
            if old_id is None or old_id not in old_to_new:
                continue
            sp["over_record_id"] = old_to_new[old_id]
            remapped.append(sp)
        plan.covering = remapped
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

    THE MEASURED PATH IS BETTER, and the LAUNCHER DOES NOT HAVE IT —
    `urban_fire_city_launch_script.place_people` calls
    `fpl.to_placements(recs)` with no `ctx=` (found 2026-08-31,
    `tools/people_float_audit.py`, part 2), so THIS is the path that
    actually authors the live city, not just a host-side preview of it. Pass
    `ctx=` when you have one and the whole job is delegated to
    `people._human_placement`, which measures each character's stature and
    depth and solves ground poses against that rig's own hip instead of the
    nominal constants below — the remaining gap between the two paths.

    One correction is cheap enough to close even without a measured rig:
    `people._seated_asset_dz` (`_MALE_SEATED_DZ_M`, -0.15 m — "no amount of
    scaling fixes it") is a per-ASSET-NAME lookup, not a measurement, so it
    is applied here too, by name, exactly like `_human_placement` applies
    it — REUSED, not copied, so the two paths cannot drift apart on which
    poses or which rigs it fires for. Without it, every MALE rig
    (`rp_eric`/`rp_manuel`/`rp_nathan`/`rp_dennis`) `fire_people` ever draws
    in `sit_edge` (the only `_SEAT_PLACED_POSES` member this module uses —
    kerb and car-sill sitters, `evacuee`/`at_car`) is authored 0.15 m above
    its kerb or sill in the live scene.
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
            try:
                from . import people as ppl
                dz += float(ppl._seated_asset_dz(rec["usd"], pose))
            except Exception:
                pass
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
        # fires on the host path. Membership is by BASENAME, not full path:
        # a plan solved before the local People mirror existed carries the
        # Nucleus URL for the same rig this module now resolves locally, and
        # the asset is exactly as known either way (`_people_asset` maps the
        # name back to the current best path at authoring time).
        base = str(rec.get("usd") or "").rsplit("/", 1)[-1]
        if base not in {u.rsplit("/", 1)[-1] for u in known_pool}:
            return "unknown_asset"
    return None


#: the keys a SERIALISED plan can carry its records under, most specific
#: first. `tools/fire_people_dry_run.py` writes `people`; a hand-rolled or
#: future writer may well use `records`, the name `Plan.records` uses in
#: memory. See `_records_of`.
RECORD_KEYS = ("people", "records")


def _records_of(plan_or_records):
    """The record list out of a `Plan`, a bare list, or a LOADED JSON DICT.

    THE 2026-08-31 CRASH. `to_placements` used to fall back to
    ``list(plan_or_records)``, and a dict iterates its KEYS — so
    `json.load(open(fire_people_final.json))` handed straight in produced the
    string ``"census"`` as the first "record" and `_convertible` died on

        AttributeError: 'str' object has no attribute 'get'

    inside `urban_fire_city_launch_script.place_people`, which took the whole
    500 m city launch down with it AFTER every bake and every emitter was
    already up. A dict is the shape a caller most naturally has (it is what
    the dry run writes), so it is accepted here rather than left to fail on
    the generic path — and anything else raises a TypeError that NAMES the
    shapes, instead of a nonsense AttributeError three frames deeper.
    """
    if hasattr(plan_or_records, "records"):
        return plan_or_records.records
    if isinstance(plan_or_records, dict):
        for key in RECORD_KEYS:
            v = plan_or_records.get(key)
            if isinstance(v, list):
                return v
        raise TypeError(
            "to_placements got a dict with keys {0} — a serialised plan must "
            "carry its records under one of {1} (tools/fire_people_dry_run.py "
            "writes {2!r})".format(sorted(plan_or_records), list(RECORD_KEYS),
                                   RECORD_KEYS[0]))
    if isinstance(plan_or_records, (str, bytes)):
        raise TypeError(
            "to_placements got a {0}; it takes a Plan, a list of records, or "
            "a loaded plan dict — not a path (load the JSON first)".format(
                type(plan_or_records).__name__))
    return list(plan_or_records)


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
    recs = _records_of(plan_or_records)
    known = None if ctx is not None else set(RIGGED_HUMANS) | set(POSED_HUMANS)
    placements, skipped = [], {}
    for rec in recs:
        why = _convertible(rec, known)
        if why:
            skipped.setdefault(why, []).append(rec.get("id"))
            continue
        # Re-anchor the rig to the CURRENT best path (local mirror first,
        # Nucleus fallback) so a plan solved under the other root still
        # authors — and heals — to whatever resolves today.
        base = str(rec.get("usd") or "").rsplit("/", 1)[-1]
        cur = _people_asset(base)
        if cur != rec.get("usd") and base in {
                u.rsplit("/", 1)[-1]
                for u in RIGGED_HUMANS + POSED_HUMANS}:
            rec = dict(rec, usd=cur)
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
# The sidecar-completeness report, one implementation for the dry run to
# print and the tests to assert on. See `_Building.sidecar_report` and
# `SIDECAR_FIELD_USE`.
# ---------------------------------------------------------------------------
def sidecar_reports(plan):
    """`[dict, ...]`, one per SURVIVING building (i.e. after the index/
    geometry check in `_Solver.__init__`), in manifest order."""
    return [b.sidecar_report() for b in plan.solver.buildings]


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
    # THROWAWAY pose-row records (`variant == "pose_experiment"`,
    # 2026-09-01) are diagnostic-only — never budgeted, never flame-
    # checked, never meant to satisfy the real window contract — so the
    # WHOLE gate is blind to them, the same way it is blind to nothing else
    # in `plan.records`. They still convert and render (`to_placements`
    # does not filter by variant), just outside every rule below.
    recs = [r for r in plan.records if r.get("variant") != "pose_experiment"]
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
        "max_wall_dist_m {0} m, street AND burial classes alike — a "
        "skyscraper's 0.33H zone is 100 m and would not read as this "
        "fire's crowd, and it is also the guard against a stale sidecar "
        "throwing debris further than FC_PEOPLE_MAX_DIST_M keeps on stage"
        .format(cfg["max_wall_dist_m"]))

    # 13a. no window figure above the drone's own ceiling — checked on the
    #      SILL (the highest point a camera could ever see for this class),
    #      not the feet, which sit lower still now that the hips are sunk to
    #      the sill; see `_pass_window`'s own account.
    bad = [r["id"] for r in recs if r["cls"] == "window"
           and float(r.get("sill_z", r["z"]))
           > float(cfg["window_max_z_m"]) + 1e-6]
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

    # 15. roof_victim only on genuinely intact, non-collapsed roofs —
    #     section 5b2. Re-derived off `_Building.roof_victim_ok`, exactly the
    #     eligibility test the pass itself uses, so this rule cannot drift
    #     from the placement code by restating a looser copy of it.
    bad = []
    for r in recs:
        if r["cls"] != "roof_victim":
            continue
        b = by_i.get(r["building_i"])
        if b is None or not b.roof_victim_ok(cfg)[0]:
            bad.append(r["id"])
    add("roof_victim_on_intact_roofs", bad,
        len([r for r in recs if r["cls"] == "roof_victim"]),
        "excluded levels {0}; never roof_involved or roof_collapsed per the "
        "sidecar (or the manifest's own conservative band estimate when "
        "there is none)".format(cfg["roof_victim_excluded_levels"]))

    # 16. roof_victim stays clear of its own building's smoke seats —
    #     "keep them away from the smoke tho" (user).
    bad = []
    min_smoke = float(cfg["roof_victim_min_smoke_dist_m"]) - 1e-6
    for r in recs:
        if r["cls"] != "roof_victim":
            continue
        b = by_i.get(r["building_i"])
        if b is not None and not _clear_of_smoke(b, r["x"], r["y"], min_smoke):
            bad.append(r["id"])
    add("roof_victim_clear_of_smoke", bad,
        len([r for r in recs if r["cls"] == "roof_victim"]),
        "roof_victim_min_smoke_dist_m {0} m from every interior/roof seat "
        "in the building's own bake sidecar".format(
            cfg["roof_victim_min_smoke_dist_m"]))

    # 17. roof / roof_victim are authored ON THE DECK, never the parapet
    #     coping — RE-DERIVED THROUGH `_roof_seat_z`, the same function the
    #     placement passes call, so a record whose z came from a local mesh
    #     sample is not flagged as wrong just for disagreeing with the
    #     building's global `deck_z` scalar (item 4, 2026-08-31: that
    #     disagreement is the whole point of sampling locally).
    bad = []
    for r in recs:
        if r["cls"] not in ("roof", "roof_victim"):
            continue
        b = by_i.get(r["building_i"])
        if b is None or r.get("z_mode") != "deck":
            bad.append(r["id"])
            continue
        want_z, want_source, _nb = _roof_seat_z(b, r["x"], r["y"])
        if (abs(float(r["z"]) - want_z) > 1e-3
                or str(r.get("deck_source")) != want_source):
            bad.append(r["id"])
    add("roof_on_deck", bad,
        len([r for r in recs if r["cls"] in ("roof", "roof_victim")]),
        "z (and its source) equals _roof_seat_z(building, x, y) — the local "
        "mesh sample when one is trustworthy, else the global deck_z — "
        "never top_z/parapet")

    # 18. roof / roof_victim never stand closer to a parapet than the
    #     configured minimum — ITEM 4, 2026-08-31 user review: "don't need
    #     to be on the ledge." Re-checked geometrically off the building's
    #     own footprint rather than trusting the record's own `roof_clear_m`
    #     (which `_roof_interior_point` computed against the SAME formula,
    #     so re-deriving it here is what makes this an independent check).
    bad = []
    for r in recs:
        if r["cls"] not in ("roof", "roof_victim"):
            continue
        b = by_i.get(r["building_i"])
        if b is None:
            bad.append(r["id"])
            continue
        lx, ly = _rot(r["x"] - b.x, r["y"] - b.y, -b.yaw)
        clear = min(b.W / 2.0 - abs(lx), b.D / 2.0 - abs(ly))
        e_key = ("roof_edge_band_m" if r["cls"] == "roof"
                 else "roof_victim_edge_band_m")
        e_min = float(cfg[e_key][0])
        if clear < e_min - 1e-3:
            bad.append(r["id"])
    add("roof_clear_of_parapet", bad,
        len([r for r in recs if r["cls"] in ("roof", "roof_victim")]),
        "roof_edge_band_m[0] / roof_victim_edge_band_m[0] as a HARD minimum "
        "clearance from every wall, not a band hugging one edge")

    # 18b. NO ROOF-STANDING FIGURE OF ANY CLASS ON ANY BUILDING WHOSE ROOF
    #      IS BREACHED AT ALL — 2026-08-31, second bench-v2 round, quoted:
    #      "we can't have people on roofs where the roof has collapsed ...
    #      the partial collapse has a roof collapse but people are still on
    #      the ledge there." `roof_ok()`/`roof_victim_ok()` already gate on
    #      `self.level`/`self.roof_involved`/`self.roof_collapsed`, which is
    #      WHY the bench trio's F5c office already places zero (verified
    #      2026-08-31 against the real `city_138` sidecar: `roof_ok=(False,
    #      'roof_deck_involved(F5c)')`, `roof_victim_ok=(False, 'collapse_
    #      level(F5c)')` — the rejected build was almost certainly a stale
    #      snapshot from mid-session). This rule does not trust that those
    #      two functions stayed wired to the placement passes: it RE-DERIVES
    #      `roof_involved`/`roof_collapsed` straight from the module-level
    #      functions of the same name, off the building's own manifest
    #      record and sidecar doc — not off `_Building`'s cached attributes,
    #      which a future refactor could desync from the raw fields — and
    #      ALSO refuses on `level` alone (F4/F5/F5c/F6, the union of both
    #      classes' own exclusion sets) so a sidecar that is silent about
    #      roof damage cannot accidentally admit one of these levels either.
    bad = []
    breached_levels = set(cfg["roof_victim_excluded_levels"]) | {
        lvl for lvl in ("F1", "F2", "F3", "F4", "F5", "F5c", "F6")
        if lvl not in cfg["roof_ok_levels"]}
    for r in recs:
        if r["cls"] not in ("roof", "roof_victim"):
            continue
        b = by_i.get(r["building_i"])
        if b is None:
            bad.append(r["id"])
            continue
        if (b.level in breached_levels
                or roof_involved(b.rec, b.doc)
                or roof_collapsed(b.rec, b.doc)):
            bad.append(r["id"])
    add("no_roof_figure_on_a_breached_roof", bad,
        len([r for r in recs if r["cls"] in ("roof", "roof_victim")]),
        "level in {0}, or roof_involved()/roof_collapsed() true per the "
        "sidecar — re-derived independently of _Building's cached "
        "attributes; F1-F3 with an intact, unbreached deck only"
        .format(sorted(breached_levels)))

    # 19. every window figure clears every flame-bearing event on its own
    #     elevation by `window_flame_clear_m` — ITEM 5, 2026-08-31 user
    #     review: "they can't be right next to the open flame." Re-derived
    #     through `_clear_of_flame`, the same function `_pass_window` calls.
    bad = []
    flame_min = float(cfg["window_flame_intensity_min"])
    flame_clear = float(cfg["window_flame_clear_m"])
    for r in recs:
        if r["cls"] != "window":
            continue
        b = by_i.get(r["building_i"])
        if b is None or not _clear_of_flame(
                b, r["x"], r["y"], r.get("side"), flame_clear, flame_min):
            bad.append(r["id"])
    add("windows_clear_of_flame", bad,
        len([r for r in recs if r["cls"] == "window"]),
        "window_flame_clear_m {0} m from any bake event with intensity >= "
        "window_flame_intensity_min {1} on the SAME elevation".format(
            flame_clear, flame_min))

    # 20. a burial figure with `author_burial_cover` on and a non-"none"
    #     pattern actually carries authored debris — ITEM 2, the coordinator
    #     instruction ("look at tornado code ... partial burial"): a
    #     `covered_frac` with no `boards` behind it is the exact gap that
    #     instruction was about, and this rule is what would have caught it.
    burial_cls = ("casualty_apron", "roof_debris")
    bad = []
    if bool(cfg.get("author_burial_cover", True)):
        for r in recs:
            if r["cls"] not in burial_cls:
                continue
            if r.get("occlusion") not in (None, "none") \
                    and int(r.get("boards", 0)) <= 0:
                bad.append(r["id"])
    add("burial_cover_is_authored", bad,
        len([r for r in recs if r["cls"] in burial_cls
             and r.get("occlusion") not in (None, "none")]),
        "author_burial_cover {0}; a non-'none' occlusion pattern must carry "
        "boards > 0 (real pieces in plan.covering), not just a drawn "
        "covered_frac".format(cfg.get("author_burial_cover", True)))

    # 21. nothing is sunk further than its own measured body depth allows —
    #     `sink_frac`'s own ceiling (0.18 by default; `tornado_people`'s own
    #     post-1-km-review number), so a burial figure is never sunk deep
    #     enough to explain full invisibility on its own.
    bad = []
    sink_hi = float((cfg.get("sink_frac") or (0.0, 0.18))[1]) + 1e-6
    for r in recs:
        if r["cls"] not in burial_cls:
            continue
        if float(r.get("sunk_frac", 0.0)) > sink_hi:
            bad.append(r["id"])
    add("burial_sink_bounded", bad,
        len([r for r in recs if r["cls"] in burial_cls]),
        "sunk_frac <= sink_frac[1] ({0})".format(sink_hi))

    # 22. `interior_trapped` sits on the SURVIVING slab, set back from the
    #     broken elevation by no less than the configured minimum and no
    #     more than the sightline actually allows — re-derived through
    #     `_interior_sightline_ok`, the same function the placement pass
    #     calls, so this cannot drift into a looser copy of the same test.
    bad = []
    setback_lo = float(cfg["interior_setback_m"][0]) - 1e-6
    min_deg = float(cfg["interior_min_sightline_deg"])
    for r in recs:
        if r["cls"] != "interior_trapped":
            continue
        b = by_i.get(r["building_i"])
        setback = r.get("setback_m")
        if b is None or setback is None or float(setback) < setback_lo:
            bad.append(r["id"])
            continue
        period = storey_period(b.rec, b.doc)
        eye_h = _INTERIOR_EYE_H_M.get(r.get("variant"), 1.40)
        if not _interior_sightline_ok(setback, period, min_deg, eye_h):
            bad.append(r["id"])
    add("interior_trapped_setback_and_sightline", bad,
        len([r for r in recs if r["cls"] == "interior_trapped"]),
        "interior_setback_m[0] {0} m minimum, and never deeper than "
        "_interior_sightline_ok allows at {1} deg".format(
            cfg["interior_setback_m"][0], min_deg))

    # 23. `interior_trapped` is on the GROUND-truth-eligible building only
    #     (F5c, a real slab z, a recorded failed elevation) — re-derived off
    #     `_interior_trapped_ok`, the same eligibility test the pass uses.
    bad = []
    for r in recs:
        if r["cls"] != "interior_trapped":
            continue
        b = by_i.get(r["building_i"])
        if b is None or not _interior_trapped_ok(b):
            bad.append(r["id"])
    add("interior_trapped_eligible_building", bad,
        len([r for r in recs if r["cls"] == "interior_trapped"]),
        "collapse_levels only, with a real masses.main.levels/fire.origin/"
        "fire.sides in the sidecar")

    # 22. interior_trapped is genuinely INSIDE its own building's footprint
    #     — the inverse of rule 1, and the one class that WANTS this to be
    #     true (section 5d).
    by_i2 = {b.i: b for b in sol.buildings}
    bad = []
    for r in recs:
        if r["cls"] != "interior_trapped":
            continue
        b = by_i2.get(r["building_i"])
        if b is None or not point_in_obb(r["x"], r["y"], b.x, b.y, b.W, b.D,
                                         b.yaw, margin=0.0):
            bad.append(r["id"])
    add("interior_trapped_is_inside_its_building", bad,
        len([r for r in recs if r["cls"] == "interior_trapped"]),
        "the AERIAL_EXEMPT class that wants the footprint check to fail")

    # 23. interior_trapped never right at the broken edge — "they need to be
    #     a little inside at a safe distance" (user).
    lo_set, hi_set = cfg["interior_setback_m"]
    bad = [r["id"] for r in recs
           if r["cls"] == "interior_trapped"
           and not (lo_set - 1e-6 <= float(r.get("setback_m", -1.0))
                    <= hi_set + 1e-6)]
    add("interior_trapped_setback_respected", bad,
        len([r for r in recs if r["cls"] == "interior_trapped"]),
        "interior_setback_m {0}".format(cfg["interior_setback_m"]))

    # 24. interior_trapped sits on its own building's real slab z, not a
    #     guessed one — re-derives `_slab_z` off the same sidecar the
    #     placement pass read, so this cannot drift from the placement code.
    #     `standing=` MATCHES THE RECORD'S OWN VARIANT (bench-v4 fix,
    #     2026-09-01) — a `conscious` figure is authored one storey lower
    #     than a `passed_out` one, so re-deriving with the base (non-
    #     standing) slab for both would flag every ledge-fix-compliant
    #     conscious record as wrong.
    bad = []
    for r in recs:
        if r["cls"] != "interior_trapped":
            continue
        b = by_i2.get(r["building_i"])
        standing = (r.get("variant") == "conscious")
        slab_z, storey = (_slab_z(b, standing=standing) if b is not None
                          else (None, None))
        if (b is None or slab_z is None
                or abs(float(r["z"]) - slab_z) > 1e-3
                or int(r.get("storey", -1)) != storey):
            bad.append(r["id"])
    add("interior_trapped_on_its_own_slab", bad,
        len([r for r in recs if r["cls"] == "interior_trapped"]),
        "z == _slab_z(building), the storey directly below fire.origin")

    # 25. interior_trapped clears the geometric sightline check — re-derives
    #     `_interior_sightline_ok` off the record's own `setback_m` and the
    #     building's own storey period, so an unwired or loosened gate cannot
    #     pass silently.
    bad = []
    min_deg_cfg = float(cfg["interior_min_sightline_deg"])
    for r in recs:
        if r["cls"] != "interior_trapped":
            continue
        b = by_i2.get(r["building_i"])
        if b is None:
            bad.append(r["id"])
            continue
        period = storey_period(b.rec, b.doc)
        eye_h = _INTERIOR_EYE_H_M.get(r.get("variant"), 1.0)
        if not _interior_sightline_ok(float(r.get("setback_m", 1e9)), period,
                                      float(r.get("min_sightline_deg",
                                                   min_deg_cfg)), eye_h):
            bad.append(r["id"])
    add("interior_trapped_has_a_sightline", bad,
        len([r for r in recs if r["cls"] == "interior_trapped"]),
        "interior_min_sightline_deg {0} deg above horizontal, through one "
        "storey of headroom".format(min_deg_cfg))

    # 26. `street_classes` off means NO street-class figures at all — the
    #     default this module now ships (item 3), re-checked against the
    #     plan's own recorded knob so a caller that flips it back on is not
    #     flagged.
    bad = ([r["id"] for r in recs if r["cls"] in STREET_CLASSES]
           if not plan.meta.get("street_classes") else [])
    add("street_classes_off_means_no_street_figures", bad, len(recs),
        "FP_STREET_CLASSES / cfg['street_classes'], default off")

    # 27. every burial covering piece actually CONTACTS the one body it
    #     covers — bench-v3 rejection, quoted: "figures FLOAT ... covering
    #     pieces render as ... boxes hovering ABOVE the bodies, touching
    #     nothing." Two checks per candidate piece, NOT the same for both
    #     geometries `_cover_piece` can return: (a) FOR A FLAT PIECE
    #     (`propped` False) — `_cover_piece` was solved to put its bottom
    #     face at `cover_top_z`; if the piece's own `z`/`t` do not reproduce
    #     that, the `planks`-spec contract drifted after the piece was
    #     solved. A PROPPED piece's `z` is deliberately NOT its bottom face
    #     (it is a diagonal beam from the debris surface up past the body —
    #     `_cover_piece`'s own `rise`/`run`/`off` geometry — so this check
    #     does not apply to one and is skipped rather than made to fail on a
    #     shape it was never testing). (b) PLAUSIBLE CONTACT, both
    #     geometries — `cover_top_z` (what the piece was SOLVED to reach,
    #     regardless of how it got there) is not detached from the body's
    #     OWN authored surface (`z` + `lift_m`, its support surface plus the
    #     lying lift = its centreline) by more than `cover_contact_tol_m`,
    #     generous enough for a body's real depth and the crest's own bands
    #     but tight enough that a piece solved a half-metre off — the exact
    #     symptom reported, and the exact gap `_cover_burial`'s `lift * 2.2`
    #     clamp (same round) was added to close — fails loudly rather than
    #     passing because SOME piece exists somewhere. Matched by
    #     `over_record_id`, remapped alongside the aerial-visibility reindex
    #     above, not by proximity — a nearby piece over the WRONG body would
    #     pass a proximity check and prove nothing.
    by_rec_id = {}
    for sp in (getattr(plan, "covering", None) or []):
        by_rec_id.setdefault(sp.get("over_record_id"), []).append(sp)
    tol = float(cfg.get("cover_contact_tol_m", 0.35))
    bad = []
    for r in recs:
        if r["cls"] not in ("casualty_apron", "roof_debris"):
            continue
        if float(r.get("covered_frac", 0.0)) <= 1e-6:
            continue                  # "none" pattern authors no pieces
        pieces = by_rec_id.get(r["id"]) or []
        if not pieces:
            bad.append(r["id"])
            continue
        body_surface = float(r["z"]) + float(r.get("lift_m", 0.0))
        ok_any = False
        for sp in pieces:
            top_z = float(sp.get("cover_top_z", sp["z"]))
            if not sp.get("propped"):
                piece_bottom = float(sp["z"]) - float(sp["t"]) / 2.0
                if abs(piece_bottom - top_z) > 0.05:
                    continue           # (a) flat-piece authoring drift
            if abs(top_z - body_surface) <= tol:
                ok_any = True          # (b) plausible contact
        if not ok_any:
            bad.append(r["id"])
    add("burial_covering_contacts_the_body", bad,
        len([r for r in recs if r["cls"] in ("casualty_apron", "roof_debris")
            and float(r.get("covered_frac", 0.0)) > 1e-6]),
        "cover_contact_tol_m {0} m between a piece's bottom face and the "
        "body's own z + lift_m".format(tol))

    # 28. the STANDING fallback window figure sits a real body depth behind
    #     the facade AND only ever draws an opening with a genuine spandrel/
    #     sill band below it — bench-v3 rejection, quoted: "she stands
    #     FULLY VISIBLE head-to-toe in FRONT of the glass, zero leg
    #     occlusion."
    bad = []
    for r in recs:
        if r["cls"] != "window" or r.get("variant") != "standing_at_opening":
            continue
        if (float(r.get("inset_m", 0.0))
                < float(cfg["window_stand_inset_m"]) - 1e-6):
            bad.append(r["id"])
            continue
        spandrel = float(r.get("sill_z", 0.0)) - float(r.get("floor_z", 0.0))
        if spandrel < float(cfg["window_min_spandrel_m"]) - 1e-6:
            bad.append(r["id"])
    add("window_standing_fallback_is_recessed", bad,
        len([r for r in recs if r["cls"] == "window"
            and r.get("variant") == "standing_at_opening"]),
        "window_stand_inset_m {0} m behind the facade, "
        "window_min_spandrel_m {1} m of solid wall below the sill".format(
            cfg["window_stand_inset_m"], cfg["window_min_spandrel_m"]))

    # 29. no window figure ever in the top `window_top_storeys_excluded`
    #     storeys of its own building — 2026-09-01 user follow-up on the
    #     bench-v4 ledge-stander complaint: "Don't do any window leans on
    #     the top 2-3 stories always below." Re-derived off the building's
    #     own `n_storeys`, not the record's own `storey` field trusted
    #     blindly.
    by_i3 = {b.i: b for b in sol.buildings}
    bad = []
    top_excl = int(cfg["window_top_storeys_excluded"])
    for r in recs:
        if r["cls"] != "window":
            continue
        b = by_i3.get(r["building_i"])
        if b is None or int(r.get("storey", -1)) > b.n_storeys - 1 - top_excl:
            bad.append(r["id"])
    add("window_below_the_top_storeys", bad,
        len([r for r in recs if r["cls"] == "window"]),
        "window_top_storeys_excluded {0} — storey <= n_storeys - 1 - {0} "
        "on every building".format(top_excl))

    # 30. NO STANDING interior_trapped FIGURE AT OR ABOVE THE TOP SURVIVING
    #     STOREY — bench-v4 ledge-stander complaint, 2026-09-01, coordinator's
    #     own rule: "on a collapse-level building, NO standing figure of any
    #     class at or above (top_surviving_storey - 1) ... standing interior
    #     figures live at least one full storey below the broken roof line;
    #     lying figures may stay higher." Re-derives the BASE (non-standing)
    #     slab storey independently and asserts every `conscious` record's
    #     own storey is STRICTLY BELOW it; `passed_out` is exempt by name.
    bad = []
    for r in recs:
        if r["cls"] != "interior_trapped" or r.get("variant") != "conscious":
            continue
        b = by_i2.get(r["building_i"])
        _sz, base_storey = (_slab_z(b, standing=False) if b is not None
                            else (None, None))
        if (b is None or base_storey is None
                or int(r.get("storey", 1 << 30)) >= base_storey):
            bad.append(r["id"])
    add("no_standing_interior_figure_at_the_break", bad,
        len([r for r in recs if r["cls"] == "interior_trapped"
            and r.get("variant") == "conscious"]),
        "a conscious (standing) interior_trapped figure is always at least "
        "one storey below the top surviving slab; lying figures are exempt")

    # 31. EVERY WINDOW FIGURE'S WORLD POSITION FALLS INSIDE A REAL OPENING
    #     RECT OF THE ELEVATION IT CLAIMS — bench-v7 REJECTION, 2026-09-01:
    #     "the leaners [are] pasted flat on a BLANK BRICK wall (no windows
    #     anywhere in frame)." Coordinator's hypothesis was a YAW/FRAME
    #     mismatch between the appended bench record's yaw and the sidecar's
    #     own opening frame; DISPROVEN by direct inspection (i=274's yaw_deg
    #     matched its source city record exactly, same as the other three
    #     bench buildings, and `_Building.sides` correctly resolved `('E',)`
    #     from the sidecar). The REAL cause: `_pass_window`'s side choice
    #     preferred a flame-free elevation over the only elevation with real
    #     glazing data, and the "derived" synthetic grid it fell onto (a
    #     party wall, on this asset) does not exist as glass at all — fixed
    #     by preferring MEASURED sides (`_side_ops` non-empty) in the side
    #     pool above. This rule is the geometric backstop the coordinator
    #     asked for regardless of that fix's cause: "compute each window
    #     figure's world position and confirm it lies within an ...opening
    #     rect of the PLACED building... That assertion becomes a gate rule
    #     ... so a frame mismatch can never render again."
    #
    #     Re-derives `openings_for_side(b.rec, r['side'], r['storey'], cfg,
    #     b.doc)` FRESH — never the `op` `_pass_window` already chose — so a
    #     future edit that lets placement and this rule diverge (a stale
    #     cached frame, a yaw applied twice, a storey mismatch) is caught
    #     here even if the placement code that produced `r["x"]/r["y"]`
    #     itself has already gone stale. Every recomputed opening shares one
    #     `fr` (`openings_for_side` builds it once per call and stamps it on
    #     every returned dict), so the record's local coordinate along the
    #     wall — `u = (x - fr_x) * cos(fr_yaw) + (y - fr_y) * sin(fr_yaw)`,
    #     the exact inverse of `_face_point`'s own u-term, EXACT regardless
    #     of the inset/depth term because that term is entirely along the
    #     orthogonal (outward-normal) axis — must land inside SOME
    #     recomputed opening's own `[u0, u1]` band, and that same opening's
    #     `z_sill` must match the record's own stamped `sill_z` (the sill
    #     the pose was built against). A record on the wrong elevation, the
    #     wrong storey, or shifted by a stray rotation fails both checks at
    #     once; a genuine leaner standing in its own real bay passes both.
    bad = []
    u_tol, z_tol = 0.10, 0.05
    for r in recs:
        if r["cls"] != "window":
            continue
        b = by_i3.get(r["building_i"])
        if b is None or not r.get("side") or r.get("storey") is None:
            bad.append(r["id"])
            continue
        ops = openings_for_side(b.rec, r["side"], int(r["storey"]), cfg,
                                b.doc)
        hit = False
        for op in ops:
            fr = op["fr"]
            fx, fy, fyaw = float(fr[0]), float(fr[1]), float(fr[2])
            u = ((float(r["x"]) - fx) * math.cos(fyaw)
                + (float(r["y"]) - fy) * math.sin(fyaw))
            if (op["u0"] - u_tol <= u <= op["u1"] + u_tol
                    and abs(op["z_sill"] - float(r.get("sill_z", -1e9)))
                    <= z_tol):
                hit = True
                break
        if not hit:
            bad.append(r["id"])
    add("window_figure_is_in_an_opening", bad,
        len([r for r in recs if r["cls"] == "window"]),
        "the figure's world (x, y) inverts to a local `u` inside a FRESHLY "
        "recomputed opening's own [u0, u1] on its stamped side/storey, and "
        "that opening's z_sill matches the record's own sill_z, within "
        "{0} m / {1} m — re-derived independently of the placement pass "
        "so a yaw/frame mismatch cannot render undetected".format(
            u_tol, z_tol))

    return out


def write_records(path, plan):
    """Ground truth, in the envelope `people.write_records` uses (`meta` +
    `people`) so a reader that already handles `humans.json` can handle
    this."""
    doc = {"meta": dict(plan.meta), "people": plan.records,
           "census": summarise(plan),
           "refused": dict(plan.refused), "dropped": dict(plan.dropped),
           "degraded": dict(plan.degraded),
           "manifest_records_skipped": list(
               getattr(plan.solver, "skipped_records", []) or []),
           "sidecar_report": sidecar_reports(plan),
           # THE REAL COVERING DEBRIS `_cover_burial` authored over the
           # burial classes — `planks`-spec shape, `["for"] ==
           # "fire_burial"` — for a launcher to build alongside the people
           # themselves. Empty when `author_burial_cover` is off.
           "covering": list(getattr(plan, "covering", []) or [])}
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, "w") as fh:
        json.dump(doc, fh, indent=1)
    return path
