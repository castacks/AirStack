---
name: place-people-in-scenes
description: >-
  Put survivors (and the cars they are in) into a disaster scene — where they go and why, the RenderPeople rigs, the pose/z-offset machinery, and the long list of things that make a person or an occupant INVISIBLE. Read before touching scene_gen/disaster/people.py, scene_gen/detail/vehicles.py, _HUMAN_POSES, or any launcher that authors humans. Most of the failures here are silent: the placement succeeds and you see nothing.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Place people (and occupants) in a disaster scene

## Why this file exists

Every failure in this pipeline is SILENT. A person placed inside an intact
house, behind a car window, under a closed canopy or at the wrong z is
authored without error, counted in the tally, written to the ground truth —
and cannot be seen. The scene reports 60 people and the reviewer says "I don't
see any people", and both are correct.

So the recurring question is never "did it place?" but **"can a camera see
it, and is it where a person would actually be?"** Everything below is a way
one of those two went wrong.

Companion skills: [build-wildfire-scenes](../build-wildfire-scenes/SKILL.md)
(the fire, damage and archetype pipeline this consumes) and
[run-isaac-sim-launcher](../run-isaac-sim-launcher/SKILL.md) (how to run a
scene and get pictures out of it).

## The modules

| file | does |
|---|---|
| `scene_gen/disaster/people.py` | the whole planner: scenarios, quotas, the seat table, blockage debris. Touches NO stage — pure Python, so it runs and is asserted host-side |
| `scene_gen/detail/vehicles.py` | `CABIN_RULES`, `open_cabin`, `can_open_cabin`, `strip_glass`, `car_pose_for_lane` |
| `scene_gen/scene_generator.py` | `_HUMAN_POSES`, `_POSE_Z_OFFSET`, `_bind_human_pose` — the UsdSkel posing |
| `scene_gen/scene_api.py` | `build_scene` authors the plan onto the stage; owns Flow, the glades, the locator poles, and the ORDER (cars before the scorch pass, people after) |
| `simulation/isaac-sim/launch_scripts/suburb_assemble_launch_script.py` | the thin launcher for `build_scene` — env knobs, banner, SNAP_DIR |
| `simulation/isaac-sim/launch_scripts/car_occupants_launch_script.py` | the occupant bench — five cars, one person each, three cameras per car |
| `simulation/isaac-sim/launch_scripts/people_showcase_launch_script.py` | every survivor situation side by side on one small plate |

---

# WHERE PEOPLE GO, AND WHY

Scattering people uniformly is worthless: a model trained on it learns a
person is equally likely anywhere, which is the one thing the after-action
literature says is false. The shares come from real events.

| source | finding |
|---|---|
| **Camp Fire 2018** (NIST, Maranghides et al.) | 31 temporary refuge areas: **14 parking lots, 7 roadways, 6 structures, 3 natural areas**. 17 burnovers hit 300–500 evacuating civilians, 12 on primary egress arteries. **85 vehicles abandoned in 1.6 km** of Skyway |
| **Lahaina 2023** (FSRI / Maui PD) | 42 in structures, 39 outdoors, 15 in vehicles, 1 in water. >60% were fleeing; recovered on average **244 m** from home. Jams from downed poles, dead signals, locked gates |
| **Australia 1901–2011** (Blanchi & Haynes) | 58% open air, 28% structures, 8% vehicles |

**Parking lots dominate for a reason that generalises**: a hectare of
non-combustible ground, next to open ground, with a road to it.

## Scenario list

`parking_refuge`, `open_ground`, `pools`, `gridlock`, `cul_de_sac`,
`at_home`. Shares are config, in the preset's `people:` block.

**`at_home` IS NOT INSIDE A HOUSE.** It is front yards and driveways at the
burn EDGE — the stay-or-go case, where front-yard rescues and fatalities both
cluster. The name has caused this confusion more than once; if you rename it,
`front_yard` is the honest name.

**`exposed_interior` IS RETIRED — DO NOT BRING IT BACK.** It put one person on
the floor plate of a `roof_collapsed` house, on the argument that a fallen
roof leaves the floor open to the sky. It does not: what is open is a hole
BETWEEN fallen roof planes, and from any altitude a drone flies the figure is
behind a wall, a rafter or a chimney stack. A target that cannot be seen
cannot be labelled, and an annotation for an invisible one is worse than none.
Structure interiors are out of scope for this benchmark.

---

# THE BUG CATALOGUE

## Posture rules — hard, reviewed on sight (2026-08-26)

**Nobody waves, in any scene.** The raised-arm pose looked like a mannequin
with a broken limb. It is gone from `scene_generator._HUMAN_POSES`,
`_bind_human_pose` / `pose_z_offset` raise on any pose name not in that table
(an unknown name used to leave the rig silently in its A-pose), and
`people.add_person` raises on `people.BANNED_POSES`. Do not add it back, and
do not add a `wave_share` to a scenario — `tornado_people` ignores one.

**In an undamaged scene nobody sits or lies on the ground.** A person sitting
on a lawn, crouching on a turnaround, or face-down on a drive with nothing to
flee is a person the scene cannot explain. `resolve_cfg(..., has_disaster=False)`
sets `peacetime`, and with it:

- `add_person` turns any `sit_ground` / `sit_edge` / `crouch` / `prone`
  request into a stander and counts it (`plan.coerced`, printed as a note);
- `at_home` drops its nominal-front-step `sit_edge`, `cul_de_sac` drops the
  crouch, `casualty_share` is ignored;
- **seated is allowed only on a seat the caller can see**: a car seat
  (`in_vehicle=` — occupants are fine, they are the harder detection case) or
  a bench (`seat="bench"`, parks and the urban kit place them). Nothing sits
  on a bench yet; when a scenario does, pass `seat=` and the seat pan height,
  and verify the facing on the bench first — the bench assets' front is not
  recorded anywhere.

Disaster scenes keep `sit_ground` (open-ground refuge), `sit_edge` (kerb,
front step, coping), `crouch` and prone — those postures are the story there.
`scene_gen/tests/test_people_rules.py` pins all of the above without Isaac.

## Nobody outside the burn — hard, requested on sight (2026-08-28)

**A survivor on ground the fire never reached is not in this dataset.** Asked
for in those words: *"I don't want people who are not in the fire damaged
area. No people in undamaged areas."*

It was not a bug — it was **three deliberate designs**, all of which put
figures ahead of the front, and all of which the gate now overrides:

| pass | why it was out there |
|---|---|
| `parking_refuge` | `_parking_refuge` sorts the lots by burn age and takes the `lots_used` deepest. **"Deepest" is not "inside"** — with one lot in the black the other two are just the least-unburnt, and the code prints each lot's age and takes it anyway (*"a TRA on unburnt asphalt beside the fire is exactly what a TRA is"*) |
| `gridlock` | the queue runs **outbound past the blockage by construction** — that is the direction people are driving — so its walkers are ahead of the front on purpose |
| `at_home` | admits the whole band `-0.20*span < age < 0.12*span`, i.e. up to a fifth of a span **ahead** of the front, because the stay-or-go decision is only live at the edge |

`people.DEFAULTS["min_burn_age_s"]` (0.0, i.e. ON; `null` disables) is the
gate, tested in `_Plan.add_person` against **the person's own `ctx["age"](x,
y)`** — the same field the damage ladder, the ground scar and the archetype
levels key off, so the people agree with the scene's own definition of burnt
rather than with a second one. It is the counterpart of what `min_intensity`
would be for `tornado_people` against `ctx["intensity_at"]`.

**Measured on `suburb_wildfire_1000`**, host-side, `total: 95`,
`build_scene(burn_frac=0.45, seed=11)`:

| layout seed | gate off | gate on | were on `age < 0` | which pass |
|---|---|---|---|---|
| 23 | 95 | **69** | 26 (27%) | refuge 19, at_home 5, queue 2 |
| 10 | 86 | **80** | 6 (7%) | queue 3, at_home 3 |

**IT IS A FILTER, NOT A RE-SITING, and the head count is meant to drop.**
Nothing retries and nothing moves; on both seeds the kept plan measured as a
strict subset of the ungated one (95−26=69, 86−6=80, every kept figure at a
position the ungated plan also used). Do not "fix" the shortfall by raising
`total` — an all-unburnt refuge lot with a bigger quota is still an all-unburnt
refuge lot. `plan_people` prints `unburnt:<scenario>` per refusal, so the pass
that disagrees with the fire names itself.

**THE GATE SITS AFTER THE CHARACTER DRAW, and that looks wrong until you count
draws.** `add_person` calls `pick_human` (1–3 values out of the shared `rng`)
before it tests anything. Refusing above that call takes those draws out of the
stream and **every survivor planned afterwards moves** — the plan stops being
comparable to the one you measured. Refuse after it. Same reason the gate does
not retry.

**Peacetime is exempt, and it has to be.** `resolve_cfg(..., has_disaster=False)`
runs against an `age` that is `-1.0` everywhere by construction, so an
unconditional gate refuses the entire population and every peacetime scene
comes back empty. `_Plan.min_burn_age_s` is `None` under `peacetime`.
`scene_gen/tests/test_people_rules.py` is what catches this — its peacetime
fixture is exactly that `age`.

**One thing downstream goes quiet.** `scene_api` sizes the affected-area polygon
from `region.people_lead_s(p_recs)` = `max(-burn_age_s)`, i.e. off the survivor
furthest **ahead** of the front. With the gate on there is no such survivor, so
that term is 0 on every scene and `affected_polygons` falls back to its
`lead_frac * span` floor — `lead_bound` reads `"floor"` where it used to read
`"people"`. Correct (there is nobody out there to cover), but
`tests/test_affected_region.py` pins the old behaviour against hand-entered
points from a **pre-gate** run, so that test is now describing a scene the
planner no longer produces.

## Making a person visible at all

**A trunk keep-out is not a clearing.** `open_ground` requires 15 m from any
structure or tree TRUNK — and these crowns are 10–25 m across, so a group can
satisfy the rule and still stand under closed canopy. The scenario IS "people
on open ground", so the assembly OPENS the ground: it deactivates every tree
reference within `glade_r_m` (16 m) of an open-ground group. Cheaper and more
certain than fighting the planting passes for a gap, and a burnt stand with a
hole in it is what a fire leaves anyway.

**Sixty people over 1600 x 1200 m are individually invisible.** Finding them
to judge them costs more than judging them does. One 25 m magenta pole per
GROUP (not per person) is authored on EVERY run under a single scope
`/World/stage/generated/_people_poles`, and left DEACTIVATED — so nothing is
visible by default and looking costs a prim toggle rather than a re-assembly.
Activate that one scope prim in the stage tree to reveal the whole set;
`PEOPLE_POLES=1` does the same at build time. Row-home courts get cyan poles
under `/_rowhome_poles`, and those are still only built when the env var is
set — they mark BUILDINGS, so they give a searcher nothing. The launcher
prints each pole's scenario, group size and coordinates — fly to those.

**NEVER LEAVE THEM ACTIVE IN A SCORED RUN.** A 25 m magenta pole over every
survivor group is the answer key standing in the scene the drones are
searching, and a camera-driven planner can steer by it rather than by what it
detects. Every `osmo/missions/*.yaml` that had `PEOPLE_POLES: "1"` now carries
it commented out for that reason. Toggle the prim AFTER the run instead.

**A lone figure is unfindable; a group is not.** `at_home` with `houses: [3,6]`
split five people across five houses — one 1.7 m person per front yard.
`[2, 3]` puts 2–3 in a yard, which reads as a household. Same argument sizes
`cul_de_sac` bulbs: 15 people over 2 bulbs, not 6 over 5.

## Posing (UsdSkel)

**The rig's origin is at the FEET and posing joints does not move it.** Bend
the hips and knees for a seated pose and the pelvis stays at standing height
with the legs folded in front — the character sits in mid-air. `_POSE_Z_OFFSET`
is the per-pose drop that puts the support back on the ground: `sit_ground`
−0.80, `sit_edge` / `seated_car` −0.85, `crouch` −0.61. Derived on a 1.80 m
mannequin (`POSE_REF_HEIGHT_M`) and **scaled by measured height**, because the
pack runs 1.73–1.86 m and an unscaled offset buries the short ones.

**THE MALE RIGS SIT 0.15 M HIGH, and height scaling does not fix it.** These
are not the same body scaled: the male RenderPeople carry a longer torso above
the hips at the same stature, so a pose authored to put the PELVIS on a seat
lands their hips proud of it and their heads into the roof.
`people._MALE_SEATED_DZ_M = -0.15`, applied to seated/crouching poses only —
standing is anchored at the feet, where the difference does not show.

**A driver's pose in a car with no wheel reads as reaching through the
windscreen.** `seated_car` carries the elbows forward onto a steering wheel;
none of these assets models an interior a hand could rest on, so it looks like
the figure is clawing at the glass. `seated_car_arms_down` is the same legs
with the arms at the sides — the correct default for a passenger, and usually
for the driver too.

**Three of the nine characters have NO SKELETON.** `rp_dennis_posed_004`,
`rp_fabienne_percy_posed_001` and `rp_mei_posed_001` are posed statics, always
standing. Reach for them where a stander is wanted; never pass them a `pose`.

## Putting a person INSIDE a car

This is the single hardest thing here and it fails in five different ways.

**EVERY WINDOW IN THIS LIBRARY RENDERS OPAQUE.** The renderer forces
fractional cutout opacity to 1.0 (see the wildfire skill), so a windscreen
authored at 0.2 opacity draws SOLID. An occupant behind glass is an occupant
nobody will ever see, labelled or not. The glass has to be REMOVED, not made
transparent.

**`glass_separable` IS THE WRONG QUESTION.** The tag records that some mesh
binds a transparent material. It is not a recipe, and it gets two assets wrong
in opposite directions:

- **`Nissan_Fairlady_Z`** — its transparent materials (`Material`,
  `Material_009`) are bound to the ROOF AND BODY PANELS as well as the
  windows, and every mesh is named `Object_N`. A generic `strip_glass` is
  "correct" and turns the car into a pillarless convertible. What it needs is
  ONE named mesh removed: `Object_16`.
- **`Vehicle_A` / `Vehicle_Taxi` / `Vehicle_Police` / `Car_01_0`** — a single
  `LOD0` mesh with the windows painted into the texture. There is nothing to
  remove and no rule will ever make an occupant visible in one.

`vehicles.CABIN_RULES` holds the per-asset recipe and `open_cabin()` applies
it; `can_open_cabin()` is the gate for whether a car may carry an occupant at
all. **Gate occupants on `can_open_cabin`, never on the tag** — doing the
latter left the Fairlady simultaneously tuned for passengers and forbidden
from carrying any.

**A seat pan is not derivable.** It is inside an asset nobody modelled for
passengers. The formula (`seat ≈ roof/3`, capped at `roof − 1.00` so the head
clears) is a fallback; the real values are MEASURED per car on
`car_occupants_launch_script.py` and live in `people._CAR_SEATS`, keyed by USD
basename, as `(fwd, lat, dz, dyaw, pose)` offsets along the car's heading.
Re-measure there after adding a vehicle.

**A seated adult needs ~0.85 m over the pan.** A car whose roof is under
~1.18 m cannot hold one without the head coming through it. Check `ht` before
seating anybody.

**`yaw_deg` IS NOT THE HEADING.** A car placement's `yaw_deg` is the ART
rotation — it includes the asset's `yaw-offset` (90 on every car in this
pool). Handing it to `_human_placement` adds the HUMAN's own offset on top and
the occupant sits 90 degrees across the seat. `_car_placement` records
`heading_deg` separately for exactly this; use that.

**Some vehicles seat two.** Where `_CAR_SEATS` has two rows for an asset, use
both — a car with one occupant and an empty passenger seat is a wasted target.
The van's rear seat FACES BACKWARDS (`dyaw: -210`), which is a property of the
art, not a preference.

## Cars, generally

**MEASURE A NEW CAR BEFORE ADDING IT.** Two assets in this pool are broken and
both passed a casual look:
- `Old_Rusty_Car.usdz` — 17 x 21 m, because a ground plane is baked in.
- `ZIS-101A_Sport_1938.usdz` — 6.95 x 10.48 m, wider than a carriageway, with
  its bbox ~57 m off the origin.
The `[scene_gen] measured car:` line the resolver prints on every run is the
check. A car chosen from its mesh list rather than its bounding box is a car
nobody has actually looked at.

**The RetroNeighborhood cars are ~1.5x undersized at the pack default.**
`130.usdz` measures 1.22 x 2.94 m with an 0.86 m roof at `scale: 0.01` — two
thirds of a sedan, which is why people towered over them and why the occupant
gate refused to seat anyone. Corrected per asset (`130` → 0.016, Fairlady →
0.015).

**`yaw-offset` CANNOT be derived and MUST NOT be read off a top-down.** A
bounding box says which axis is longest, never which end is the nose. Worse,
a plumb camera has no horizontal component to derive an up-vector from — see
the snapshot bug below — so a top-down capture can be silently rotated a
quarter turn and "prove" the wrong offset. **Measure from the SIDE**, where a
car seen head-on cannot be confused with one seen broadside.

## Cars overlapping each other

**`Ground.solid` is a POINT-to-box index.** Two 4.6 x 1.9 m cars can overlap
with every corner outside the other's centre distance, so a point test passes
while the cars interpenetrate. Cars must be tested **box against box**.

**Cars come from six passes and they must all share one index**: `build_cars`
(driveway / kerb / row-home court) plus the planner's refuge, queue and
cul-de-sac. Concrete holes that existed:
- `_cul_de_sac` marked a 4.6 m car with `ground.take(x, y)` — a 1.2 m PERSON
  point — so 2–4 cars on a 14.64 m turnaround interpenetrated on nearly every
  seed.
- Queue cars registered their boxes only AFTER the whole queue was built, so
  cars within one queue never tested against each other.

`_Plan.add_car` is now the only door: it tests, returns `None` on refusal, and
tallies `role:car` rejections the way `build_cars` does.

**ADOPT PARKED CARS, DO NOT ADD MORE.** `build_cars` fills ~60% of every
row-home court before the survivor pass runs, so the bays a refuge scenario
wanted were already taken and it kept refusing them. Adding was the wrong
instinct anyway: a refuge lot inside the burn is not a lot people just drove
to — it is the lot that was already there, whose parked cars became refuge
cars when the front arrived. Adopting costs no geometry, cannot overlap
(they are already placed and mutually clear), and yields far more cars than
the pass could ever have parked. Gate it on `age > 0`: an unburnt lot full of
parked cars is just a car park.

## Blocked roads

**An archetype alone does not block a road.** A `tree_*_fallen` reference is
one object dropped at the kerb, and its own ground debris is a disc around its
STUMP — out on the verge where the tree stood, not across the lanes. From the
air it reads as a tree that fell near a road.

**And do not drop a whole tree archetype ON the carriageway** — it brings a
stump rooted at its origin, and a stump in the tarmac is the tell. Standing
burnt trunks belong at the verge.

What makes a blockage legible is an authored litter field ON the asphalt:
`people._blocker_debris` plans trunk sections (0.30–0.48 m radius) across the
lanes at a spread of bearings plus limbs between and on top, returned as
world-space tube specs the launcher authors. Lift everything to ~0.10 m: the
carriageway is at 0.02 and a piece authored at 0 sinks into it.

**Bind the timber to a photographed charred surface, not a flat dark PBR.** A
plain colour has no normal or ORM map, so a cylinder lit by one sun reads as
painted pipe.

**AND SEAT IT ON MEASURED VERTICES, AT THE GROUND DATUM.** "Lift everything to
~0.10 m" was the rule here and it was wrong three ways, each of which shipped
and was reported: the 0.10 is the 1600 m plat's carriageway height and the z
ladder scales with plate span; a limb "resting on the trunks" must have ONE end
on a trunk and the other on the ground, not be balanced at its middle; and
`log_mesh` jitters every ring vertex by +-17% of the radius, so the lowest
VERTEX is not at `z - r` and no spec-level check can see the difference.
`people._seat_debris` measures each piece through `vegetation.log_points` and
drops it; the datum is `_Z_GRASS * z_scale`, not the road, because the litter
scatters onto the verge and the ladder puts grass below asphalt. Full account
in [build-wildfire-scenes](../build-wildfire-scenes/SKILL.md); pinned by
`scene_gen/tests/test_blocker_debris.py`.

**Do not roll or pitch a baked archetype.** It was settled by PhysX and its
debris laid on the ground at z=0, all frozen into one object; turning it about
X or Y tips the debris field with it and a third of the sticks end up on end
or hanging. Vary a fallen tree by SPECIES and YAW only.

**Keep the fire off the queue.** Cars back up BEHIND the blockage, so an
emitter centred on it sits among them. Put flame ~7 m and smoke ~15 m PAST the
blockage along the outbound bearing (`spec["out_bear_deg"]`), and stand the
queue head back (`blocker_gap_m` 9–15 m). Not every blockage should burn
either — `fire_chance` — a plat where every one is alight reads as staged.

**Flow needs time before it is photographed.** Emitters inject fuel per step,
so a capture at t=0 shows an empty grid. Start the timeline and pump ~240
frames before the cameras fire.

## Verification

**RUN THE 2D DRY RUN BEFORE PAYING FOR AN ISAAC BUILD.** `people.py` touches
no stage, so the whole composition — layout, row homes, park lot, pools, fire
field, damage levels, `plan_people` — runs host-side in seconds. The harness
asserts: no human overlaps a house/car/tree/pool/fence, none is on a road
outside the queue scenario, group separation ≥ 1.2 m, **zero overlapping car
pairs across all passes**, every scenario non-empty, in-car occupants sourced
from the tuned table rather than the fallback, and blockages with/without
fire. It caught "only one blockage" and "two scenarios silently empty" before
either cost a ten-minute build.

Two traps in the harness itself, both of which produced confident false
readings:
- **Feed it the real config.** `load_scene_config()`, not the raw preset — the
  preset only NAMES an asset set and the compiler is what merges it in, so a
  raw read has no `usds` and every asset pool comes back empty.
- **Take the layout seed from the preset**, not a literal. A hardcoded seed
  silently validates a different plat from the one the scene builds.

**Straight-down captures are rotated.** In `snapshots._look_at`, a plumb
camera has `dx = dy = 0`, `atan2(0, 0) = 0` and the yaw came out −90 — so
every top-down was turned a quarter turn with world +X running UP the frame.
Nothing about the picture looks wrong, which is the problem: a car correctly
pointing along +Y was read off one of those and an asset's `yaw-offset` was
"fixed" against it. Pinned to 0 for a plumb view.

**Viewport decorations land in the capture.** `capture_viewport_to_file` grabs
what the viewport draws, INCLUDING Kit's world reference grid — a regular
lattice over the whole frame that was twice diagnosed as a defect in the scene
(once as a burn-scar overlay printing a 3 m grid). `snapshots.hide_decorations()`
clears grid, selection outline and gizmos before every shot.

---

# Ground truth

`humans.json`, one record per person: `id, scenario, group, usd, x, y, z, yaw,
pose, alive, in_vehicle, nearest_house, burn_age_s`, plus scene meta (seed,
elapsed, fire origin/heading, blockers). `in_vehicle` carries the car's id —
plat cars adopted by a refuge lot are given a stable `plat_car_NNNN`.

**Count LOCATIONS, not just people.** A group is what a drone flies to, so
`(scenario, group)` pairs are the number that matters for coverage. 95 people
at ~30 locations is a very different set from 95 at 8.

# Knobs worth knowing

| knob | where | note |
|---|---|---|
| `total`, per-scenario `share` | preset `people:` | shares are normalised; `total` is just how many |
| `in_car_share` | `parking_refuge`, `gridlock` | occupants had a hard ceiling of 2 per queue; real jams are mostly people still in cars early on |
| `lots_used` | `parking_refuge` | spread the crowd over N burn-deepest lots; one lot piles everyone in one place |
| `glade_r_m` | `open_ground` | trees deactivated around a group |
| `queues`, `fire_chance`, `blocker_gap_m` | `gridlock` | number of jams; whether each burns; how far the head stands back |
| `bulbs`, `cars` | `cul_de_sac` | fewer bulbs = bigger clusters |
| `min_burn_age_s` | top level | seconds of burn a person's own ground must have; 0.0 (on) refuses anyone on `age < 0`, `null` turns it off. A filter — the head count drops |
| `casualty_share` | top level | 0.0 by default — this set is about finding LIVE people |
| `PEOPLE_POLES=1` | env | reveals the locator poles at build time. They are authored deactivated on every run, so the prim toggle is the usual route. NOT in a scored run — see above. |
| `PEOPLE_JSON` | env | ground-truth path |
