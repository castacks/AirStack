---
name: freeze-disaster-dataset
description: >-
  The PLAN and the tracking record for `final_disaster_dataset/` — the frozen 1 km x 1 km benchmark scenes. Four disasters x two locales x three intensity levels x five people placements, each exported as ONE self-contained USD beside GT_people.json and GT_hints.json. Read before building, exporting or re-cutting any dataset scene: it owns the matrix, the directory contract, the intensity ladders, the hint class vocabulary, the self-contained-export design (and why `stage.Flatten()` is the wrong tool), and the per-cell status of what actually exists today.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Freeze the disaster dataset

## What this is

`final_disaster_dataset/` is the shippable artefact this whole repository has
been building toward: a fixed set of 1 km x 1 km scenes, each frozen to disk so
that flying it costs a file open rather than a twenty-minute build, and each
labelled well enough to score a search against.

Everything upstream — the wildfire pipeline, the tornado pipeline, the
earthquake pipeline, the people planners, the archetype bake — is a GENERATOR.
This is the step that stops generating and starts keeping. The consequences of
that are the whole point of this file: a frozen scene cannot be retuned, so
what goes in has to be right; and a frozen scene has no repository behind it,
so every asset it references has to travel inside it.

Companion skills, all of them prerequisites for the cell you are working on:
[build-wildfire-scenes](../build-wildfire-scenes/SKILL.md),
[build-tornado-scenes](../build-tornado-scenes/SKILL.md),
[build-earthquake-scenes](../build-earthquake-scenes/SKILL.md),
[place-people-in-scenes](../place-people-in-scenes/SKILL.md),
[place-people-in-tornado-scenes](../place-people-in-tornado-scenes/SKILL.md),
[run-isaac-sim-launcher](../run-isaac-sim-launcher/SKILL.md).

---

# The matrix

| axis | values | count |
|---|---|---|
| disaster | Fire, Tornado, Earthquake, Hurricane | 4 |
| locale | Urban, Suburban | 2 |
| intensity level | 1, 2, 3 — **a different layout AND a different intensity** | 3 |
| people placement | 1..5 — **same geometry, five casts** | 5 |

**120 frozen scenes.** 24 distinct geometries, each exported five times with a
different survivor plan.

Two things about that shape drive the engineering:

- **The level axis is a layout axis, not only an intensity one.** Level 1, 2
  and 3 of a given cell are three DIFFERENT plats. That is deliberate — three
  intensities of one layout would score as three views of one scene — and it
  means each level pays a full layout + bake + assemble.
- **The people axis is NOT.** Five placements over one geometry is one build
  and five survivor draws. `PEOPLE_VARIANT=k` in
  `freeze_dataset_launch_script.py` offsets `people.seed` and NOTHING else, so
  the layout, the fire/track/shake field and every damage level come out
  bit-identical across the five. Anything that re-seeds the layout for a people
  variant has broken the axis.

## The directory contract

    final_disaster_dataset/
      <Disaster>/                    Fire | Tornado | Earthquake | Hurricane
        <Locale>/                    Urban | Suburban
          level_<n>/                 1 | 2 | 3
            <k>/                     1..5   (the people placement)
              <disaster>_<locale>_lvl<n>_<k>.usd
              Materials/
              GT_people.json
              GT_hints.json

Lowercase in the FILENAME, capitalised in the PATH — that is what was asked
for, it is not a typo, and `fire_suburban_lvl1_1.usd` is the canonical example.
`Fire` covers both the wildfire and the urban-fire cells; the locale is what
separates them.

Two files that are not in the contract but are written anyway, because a scene
nobody can explain is a scene nobody trusts:

    build_stats.json    what `scene_api.build_scene` returned
    snaps/              the review captures

**IT LIVES OUTSIDE THE REPO, ON ITS OWN MOUNT.** `~/SEI-COA/final_disaster_dataset`
on the host, bind-mounted at `/isaac-sim/final_disaster_dataset` in both the
`isaac-sim` and headless compose blocks, host side overridable with
`FINAL_DATASET_DIR` in `.env`. At the full matrix this is hundreds of megabytes
a cell across 120 cells — inside the AirStack mount it sits in the working tree
where a stray `git add -A` picks it up and every `du` on the repo counts it.
(`final_disaster_dataset/` is still in `.gitignore` in case someone points
`FREEZE_OUT` back into the repo.)

**Adding that mount needs a container RECREATE**, and `airstack down` throws
away the runtime-installed `manifold3d` / `shapely` / `mapbox_earcut`. The
assembly path does not fracture so it does not need them, but re-install them
anyway — a fresh container does not pay one pip install, it pays one RUINED RUN
the next time anything does fracture.

---

# Intensity, per disaster

**The definition of a level changes with the disaster and the user sets it when
that disaster comes up for export.** What follows is the proposal for each, and
the CURRENT state of each proposal. Do not silently invent one.

## Fire (wildfire, suburban) — SETTLED, and these are the built values

Intensity is *how much of the plate the front reached and how deep into the
damage ladder it got*, and A LEVEL IS ALSO A DIFFERENT LAYOUT — three
intensities of one plat would score as three views of one scene.

| level | layout seed | `severity` | `burn_frac` | ignition | reads as |
|---|---|---|---|---|---|
| 1 | 23 | 0.45 | 0.25 | (-460, 255) hdg 0 | a fire that came to the edge of town |
| 2 | 10 | 0.60 | 0.45 | (-250, -250) hdg 45 | the tuned middle; the whole ladder present |
| 3 | 19 | 0.80 | 0.75 | (255, -460) hdg 90 | most of the plat in the black |

Measured on the built level_1 and level_2, which is what settled it:

| | level_1 | level_2 |
|---|---|---|
| houses | 250 | 271 |
| intact | 187 (75%) | 149 (55%) |
| `burned_out` | 10 | 42 |
| `partial_collapse` | 16 | 43 |
| `roof_collapsed` | 28 | 27 |
| `rubble` | 1 | 5 |
| trees intact | 3,874 (66%) | 2,904 (52%) |

**THE SKEW MATTERS MORE THAN THE COUNT.** level_2's largest damaged class is
`partial_collapse` / `burned_out` — houses that came apart. level_1's is
`roof_collapsed`, with one rubble pile on the whole plate: a fire that took
roofs rather than one that levelled the block. A milder level that only shrinks
the damaged COUNT reads as the same event over fewer lots.

**THE IGNITION POINT HAS TO MOVE WITH THE LEVEL, and along an EDGE.** The park
is central on most seeds (seed 23: x -227..193, y -194..106), so any corner-to-
corner diagonal runs straight through it and the scene reads as "the fire
started in the park" — reported on sight twice. Running the front along a
different edge per level is what makes three levels look like three events from
the air rather than one picture at three exposures.

**SEED CHOICE IS NOT FREE — CHECK THE POOL COUNT.** `pools` is a documented
survivor scenario (a pool is `damage.INCOMBUSTIBLE`, comes through the fire
intact and is among the highest-contrast features left in the black), and seed
10 plats only TWO on a 1 km plate, so level_2's planner prints "pools SKIPPED:
no pool inside the burn" and that scenario is empty. Measured across seeds at
1 km: 10 -> 2 pools, 23 -> 13, 3 -> 23, 19 -> 6, 42 -> 0, 7 -> 0. Seed 23 takes
that scenario from 0/7 to 7/7. Sweep for pools as well as house count.

## Fire — the superseded proposal, kept for the reasoning

Intensity is *how much of the plate the front reached and how deep into the
damage ladder it got*, which is exactly two knobs the pipeline already has:

| level | `burn_frac` | `severity` | reads as |
|---|---|---|---|
| 1 | 0.25 | 0.45 | a fire that came to the edge of town — a quarter of the houses touched, mostly `scorched` with a handful of `roof_collapsed`; three quarters of the plat intact |
| 2 | 0.45 | 0.60 | the tuned middle — the whole ladder present, a clear burnt/unburnt boundary running across the plate |
| 3 | 0.75 | 0.80 | most of the plat in the black, `rubble` and `burned_out` dominant, standing snags |

`burn_frac` is the share of HOUSES inside the burn and the front is run to that
quantile of arrival times — never to `max(arrival)`, which burns the whole plat
and leaves nothing to fly toward. `MINI_ELAPSED` overrides it outright and
should be left alone: the wildfire skill's *"do not hard-code the burn clock"*
applies, because how long the front needs depends on where the layout put the
houses.

**A level is a layout too**, so each level takes its own `seed`. The wildfire
preset's seed 10 was chosen for where the row-home courts land relative to the
epicentre; levels 2 and 3 need the same check (`tools/fire_png.py`) rather than
an arbitrary bump.

## Tornado — PROPOSED

Intensity is the EF proxy, and the tornado field already has the two knobs:
`peak` (the centreline intensity) and `width_m` (the swathe). Levels should
move BOTH — a stronger tornado is wider as well as more violent — and the
compiler caps `width_m` at 32% of the plate for the reason the tornado skill
gives: a corridor whose edges are off-frame is not a corridor.

| level | reads as |
|---|---|
| 1 | EF1-2: a narrow track, `roof_stripped` / `roof_collapsed` dominant, no `swept` |
| 2 | EF3: the tuned scene — the full ladder, `leveled` on the centreline |
| 3 | EF4-5: a wide track with a `swept` core and a deep plank field |

`tools/tornado_png.py --config <preset>` gates every one of these: a run that
does not print `OK  gradient and coverage both in band` is not worth building.

## Earthquake — PROPOSED

`magnitude` is the physical knob and `compile_spec` DEFINES severity from it
(a `severity:` in the same spec is ignored and says so). So levels are
magnitudes — M5.5 / M6.5 / M7.5 — and the damage-grade ladder follows.

## Hurricane — DESIGNED 2026-08-29, NOT BUILT

`presets/hurricane.yaml` exists and compiles ("tornado-like damage mechanisms
spread evenly over the whole region at lower intensity"), but there is still no
hurricane PIPELINE: no archetype ladder, no surge water, no debris model. See
the status table.

**The design now exists** — read
[build-hurricane-scenes](../build-hurricane-scenes/SKILL.md) before building any
hurricane cell, plus `scene_gen/_plans/hurricane_{research,water,wind_field,survey}.md`.

The proposed ladder, following the measured contract above (a level is a
different seed + a different severity + a moved event + one geometry override).
**Not yet built, and the user sets these when the cell comes up for export:**

| level | `site_gust_mps` | reads as | `surge_m` |
|---|---|---|---|
| 1 | ~38 (85 mph) | cladding only; **green** scene with litter; screen cages down | 0.9 |
| 2 | ~55 (123 mph) | the **brown** scene; roofs are the story; structural damage a minority | 2.0 |
| 3 | ~70 (157 mph) | you can see INTO buildings; canopy is bare sticks; poles down in runs | 2.8 |

Three things about that ladder that differ from every other disaster here:

- **It is driven by a LOCAL SITE GUST, not the Saffir-Simpson category.**
  Marshall's 11,105-structure Katrina survey found damage-derived gusts averaging
  41 m/s inside a storm rated Cat 3 at landfall.
- **The spatial variance comes from the BUILDINGS, not the field.** The field is
  uniform over a 1 km plate; what makes neighbours differ is construction era.
  A hurricane cell needs a per-house quality attribute no other cell has.
- **Level 3 surge is 2.8 m deliberately, not 3.6.** At 3.6 the whole plate is
  under water and the surface is featureless — true to the reference imagery, and
  also the level where a search benchmark has the least to find. 2.8 leaves ~35%
  of the plate dry: the inland edge of a Cat 4 surge rather than its centre.

`gt_hints.EXTRA_CLASSES["hurricane"]` gains `Flooded Road`, `Standing Water`,
`Debris Raft`, `Washover Fan`; submersion is carried as `water_state` /
`submerged_m` **attributes** on the existing `Car`/`Building`/`Tree` records
rather than as new classes, because a 20 cm waterline is not a box a human
labeller would draw from the air.

---

# GT_hints.json

`disaster/gt_hints.py`. One record per labelled object, world frame, metres.

## The class vocabulary is fixed and is the caller's, verbatim

    Building            Damaged building      Tree
    Burnt Tree          Fallen Tree           Debris
    Car                 Van                   Truck
    Toppled

plus per-disaster extras from `gt_hints.EXTRA_CLASSES` — `Pool` and
`Parking Lot` for the fire cells, `Parking Lot` alone for the wind and shake
ones. **A rename is a dataset break.** Downstream evaluation keys off these
exact strings.

## What each class means, and the decisions inside that

- **`Building` vs `Damaged building`** is the archetype's damage level and
  nothing else: `pristine` is a Building, every other level is a Damaged
  building, with the level carried in `damage_level` so a consumer can grade
  finer if it wants to.
- **`Burnt Tree` vs `Fallen Tree` is per-disaster and the ladders do not line
  up.** `snag` is a standing dead trunk and reads as burnt; `snapped` is a
  wind-broken bole with its bark on and does NOT. Calling a snapped tree burnt
  in a tornado scene is a label error a detector would learn. `_TREE_CLASS` is
  the per-disaster table.
- **`Car` / `Van` / `Truck` is by ASSET STEM, not by tag.** The pool's tags say
  where a vehicle may be PARKED (`residential`, `street`, `parked_only`), not
  what body it has, so a tag lookup puts the delivery van and the saloon in one
  class.
- **`Toppled` is an attitude, and it REPLACES the base class rather than adding
  to it.** Anything past 25 degrees of roll or pitch — a rolled car, a felled
  streetlight — is emitted once as `Toppled` with its `subclass` naming what it
  is. Emitting both would double-count every object in the corridor.
- **`Debris` is one record per FIELD, not per stick.** A road blockage's litter
  is a dozen 0.4 m limbs; a hint file with one box round each is noise, and no
  labeller would draw them. The record carries the field's bbox and its piece
  count. The tornado plank field will aggregate the same way, by cell.
- **`Pool` is taken from the PLAN, not measured off the stage.** A pool is a
  hole cut in the ground sheet with a water plane in it — there is no single
  prim whose bound is the pool. `suburb_scene._record_pool` has the exact ring.

## Two measurement rules that are not optional

- **The bbox cache must carry BOTH purposes**, `[default_, render]`. A
  `[default_]`-only cache silently declines to measure anything authored under
  `render`, which is the same disagreement that had `bake.export_object`
  reporting a clean file that `audit_archetype` then found floating.
- **The boxes are AABB, and `ComputeUntransformedBound` is NOT the way to an
  OBB.** It does not exclude the prim's own `rotateZ` — measured, worst at
  45 degrees, exact at multiples of 90 — so it hands back the inflated box it
  looks like it avoids. Every record carries `yaw_deg` instead, so a consumer
  that wants the oriented box can rebuild it.

## What is NOT in the hints, and why

- **Fences, walks, kerbs, road paint, street furniture that is still
  standing.** They are scene fabric, not targets or confusers, and a hint file
  of 2,500 fence panels buries the ten things that matter.
- **Individual archetype fragments.** A `rubble` house is thousands of loose
  meshes; the building's own record covers them and a per-fragment `Debris`
  class would be 10^4 records a plate.
- **The burn scar / scour corridor as a polygon.** `build_stats.json` carries
  `burn_xy` and `affected_xy` already; a region is not an object and putting
  it in the object list would make the class histogram meaningless.

---

# GT_people.json

`disaster.people.write_records` (wildfire, and every non-tornado cell) or
`disaster.tornado_people` (tornado), written straight to
`<out>/GT_people.json` by the launcher — it is the existing `humans.json` under
the dataset's name, not a new format.

Two properties it already has that matter here:

- **Count LOCATIONS, not people.** A group is what a drone flies to, so
  `(scenario, group)` pairs are the coverage number. 95 people at 30 locations
  is a very different benchmark from 95 at 8.
- **The locator poles are authored DEACTIVATED on every run.** A 25 m magenta
  pole over each survivor group is the answer key standing in the scene.
  `PEOPLE_POLES` must never be set on a dataset build; the prim toggle is there
  for looking afterwards.

**OPEN: the head count per placement.** The 1 km wildfire preset carries
`people.total: 95`, tuned so that the three large-open-ground scenarios do not
come out with one or two figures apiece. Whether a benchmark scene wants 95
targets or 5 is a dataset decision that has not been made.

---

# The freeze: exporting a self-contained USD

`disaster/freeze.py`, run by `freeze_dataset_launch_script.py` under
`FREEZE_EXPORT=1`. Two Kit steps and one filesystem step, and each exists
because the others cannot do its job.

## 1. Flatten, with KIT — and instancing SURVIVES

`omni.usd.get_context().export_as_stage_async` composes the whole stage into
one layer. Core USD's `stage.Flatten()` cannot be used here: every kit mesh,
GeomSubset and Material carries an `assetInfo` dict whose value core USD cannot
unpack (`Usd_CrateFile::_UnpackValue ... unsupported type enum value 0`), and
reading, copying, clearing or overwriting that field all raise. Kit's exporter
handles it. Same reason `disaster.bake` flattens with Kit and slices with USD.

**AN EARLIER DRAFT OF THIS FILE CLAIMED A FLATTEN EXPANDS THE PROTOTYPES AND
THAT A HAND-WRITTEN "internal reference" EXPORTER WAS NEEDED. THAT WAS WRONG,
AND IT WAS WRONG IN THE EXPENSIVE DIRECTION** — it would have bought a few
hundred lines of by-value copying to reproduce what the flatten already does.
MEASURED before the exporter was written, because the difference is a 300 MB
file against a 30 GB one:

    40 instanceable references to a 1.0 MB archetype
      -> flattened file 1.0 MB, 1 prototype retained,
         5,120 meshes reachable through Usd.TraverseInstanceProxies()

So a plat with 5,563 tree references costs one copy per ARCHETYPE, not per
tree, and the archetype library the scene was assembled from is exactly what
lands inside the file. `freeze.verify` re-counts the prototypes for this
reason: a flatten that silently expanded them produces a correct scene at 50x
the size and reports success.

## 2. Collect, with KIT — because MDL is opaque to USD

The flat layer still POINTS AT its textures and MDL modules.
`UsdUtils.LocalizeAsset` (USD 24.05, present in the image) relocates USD-level
asset paths — but a texture named INSIDE an `.mdl` module
(`diffuse_texture: texture_2d("./textures/...")`) is invisible to USD, and most
of the look here is MDL: the kit house materials, the AEC bark
(`TreeBark_07.mdl`), `Grass_Cut.mdl`, the RetroNeighborhood set.
`omni.kit.usd.collect` parses MDL and is the only thing available that does.

`Collector(flat, collect_dir, flat_collection=True)`. Flat collection drops the
source directory structure (there is nothing worth preserving about a Nucleus
path) and groups textures BY MDL, so two packs that both ship `BaseColor.png`
do not overwrite each other.

## 3. Land it in the contract's shape

The collector writes `materials/` and `textures/` beside the root USD; the
contract asks for one `Materials/`. Both are **moved together**, so an `.mdl`'s
relative link to its own `../textures/...` is untouched and only the ROOT
layer's asset paths need the prefix. `freeze._prefix_asset_paths` walks SPECS
rather than composed prims — after a flatten, an instanced archetype's
materials live inside a PROTOTYPE, which a `Usd.Stage` prim walk does not
visit — and it reads `default` and nothing else, because `assetInfo` is the
field that cannot be touched.

## The verification is the point, not the file

`freeze.verify` opens the result COLD and asks four things. Three of them fail
in ways that look like success:

- **nothing unresolved** (`UsdUtils.ComputeAllDependencies`) — a missed MDL
  texture;
- **nothing outside the folder** — a dependency that resolved because THIS
  machine has the archetype library is not self-contained, and that only shows
  up on somebody else's disk;
- **the prototypes survived** — see above;
- **the plate is the right size** — a bbox that disagrees with the build means
  geometry was dropped.

`freeze_report.json` is written beside the scene with all of it.

## Open questions on the export

- **Five copies or one plus five people layers?** The contract says one
  self-contained USD per people placement, so the default is five full copies —
  five times the disk for geometry that is bit-identical. Measure one first.
- **Flow prims.** The wildfire scenes carry NVIDIA Flow emitters for the flame
  and smoke. They export as ordinary prims but render only with `omni.flowusd`
  enabled and mean nothing to a non-Isaac consumer. Kept: they are tiny.
- **The Pegasus default environment** is loaded to give the World a base and
  then deactivated, not removed, so its reference is still authored on the
  prim. Whether the collector pulls it in is worth checking against the
  dependency list.
- **Colliders.** ~36,700 static triangle-mesh colliders were cooked on the last
  block for debris nothing will ever hit. A frozen scene meant to be flown does
  not need them and they are a large part of the load time.

---

# Review log

Each frozen scene is signed off on sight before it is exported. What the review
found is recorded here, because a defect visible in one cell is usually present
in every cell built by the same pass.

## Fire / Suburban / level 1 — reviewed 2026-08-27

Layout seed 11, people seed 91, `burn_frac 0.45`. 271 houses (149 intact / 122
damaged), 5,563 trees, 88 survivors at 28 locations, 3 road blockages.
**Judged good apart from four faults, all fixed without touching either seed** —
the tallies are bit-identical across the fix, which is the check that the layout
and the survivor plan really did not move.

| reported | cause | fix |
|---|---|---|
| tree debris at the road blockages is floating | `_blocker_debris` lifted to a hard-coded 0.10 m; this plate's `roads.z_scale` puts the asphalt at 0.015 m, and a third of the limbs took a further 0.30-0.75 m of "resting on the trunks" lift over ground with no trunk under it | seat on `_road_z(plan)` from `ctx["z_scale"]`; the raised limbs are seated on an actual trunk section. rng draws kept in place so nothing downstream moved |
| street lights and hydrants on top of the cul-de-sac | `_RoadIndex` models roads as centrelines + half widths and a turnaround is a 14.64 m DISC, so `on_road` called the bulb clear; `verge` then offset the prop from the spliced bulb arc onto the kerb line | bulbs registered as zero-length segments with `half_w` = bulb radius; `bulb_verge` pushes the prop radially out to the verge instead of dropping it |
| the park textures look wrong — the lot, courts and paths should be cracked asphalt, the pitch burnt grass | the park pass charred every `park_*` ground prim the front reached, so slabs and paths came out as black sheets | hard surfaces take `Damaged_Asphalt` (what the roads in the burn take), soft ones keep the char, the line work is left alone |
| the park's bin is not a park bin | `_park_placements` drew the whole shared `trash_cans` pool, and this seed picked the blue domestic wheelie bin over the `park`-tagged civic one | the park pass prefers the `park`-tagged subset, falling back to the full pool |

Full write-ups, with the measurements, are in the
[build-wildfire-scenes](../build-wildfire-scenes/SKILL.md) bug catalogue;
`scene_gen/tests/test_road_index_bulbs.py` pins the turnaround one offline.

**THE BLOCKAGE DEBRIS TOOK THREE ROUNDS AND IS THE LESSON OF THIS REVIEW.**
Reported floating, fixed, reported again, fixed, reported again. Wrong datum,
then wrong support, then wrong GEOMETRY — the third being that `log_mesh`
jitters every ring vertex by +-17% of the radius, so the lowest vertex is not
at `z - r` and **no test that asserts on the planner's own `{p0, p1, r0, r1}`
can see the error**. Rounds one and two were each "verified" that way and each
shipped. `vegetation.log_points` splits the barrel-vertex maths out of the
pxr-only path so the seating pass can measure what will actually be authored
and `scene_gen/tests/test_blocker_debris.py` can measure the same thing on the
host: 2,432 pieces, four plate sizes, sixteen seeds, no Isaac.

Generalise it: **if a look defect is reported twice, the check is measuring the
wrong object.** The rule for this repo is to assert on the geometry that gets
written, not on the parameters it is written from — the same reason
`bake.audit_archetype` probes meshes rather than trusting the bake's own
report.

**A fifth was found in the ground truth rather than on sight**, and it is the
one worth generalising: `gt_hints` tested a car's raw `roll_deg` against the
topple threshold and called **124 ordinary driveway cars "Toppled"** in a
wildfire, where nothing tips a car. `AssetPools.roll_of` gives every Y-up asset
+90 degrees purely to stand it up — an ART correction, the same trap as
"`yaw_deg` IS NOT THE HEADING" one axis over. `gt_hints.art_roll` is the
baseline and the deviation from it is the attitude. **Read the ground truth
before shipping it**; a class histogram would have caught this and a render
never would.

---

# Review punch list — Fire / Suburban

Everything reported on sight against the level_1 and level_2 builds, with what
each one actually was. **A fix here invalidates every already-frozen cell**,
because the defect is in the generator, not in the export — so the list doubles
as the rebuild trigger.

| # | reported | diagnosis | state |
|---|---|---|---|
| 1 | survivors standing outside the fire-damaged area | the wildfire people planner has no burn gate. `tornado_people` has `min_intensity` for exactly this and the wildfire path never grew one | **TODO** — and it must be applied to the already-built cells too |
| 2 | `/World/stage/Environment` still active in the frozen scene | the deactivate list named `/World/Environment`; Pegasus' env composes at `/World/stage/Environment` once the plat is authored under `/World/stage`, so the wrong prim was turned off | **done** — both paths in `freeze.DEACTIVATE_DEFAULT` |
| 3 | `fence_1_283` extends past its boundary | `_fence_run` scaled every module to make the run span its corner. Measured seed 23: **803 of 1,713 modules (47%) STRETCHED**, median +2.3%, max +14.7% — a 5.28 m railing at 6.06 m | **done** — `max_fit` 1.15 -> 1.0. Re-measured: 0 stretched, max fit 1.000, dangling ends unchanged at 20 |
| 4 | `inst/h_38/house_roof_7_37` lying on the floor | CONFIRMED: `house_terrace_pristine`'s roof plane at z -0.000..2.405 against 2.994..9.271 for every other roof on the same house. `_reseat_roots` cannot tell an OVERHANG from a floater and dropped it — on a PRISTINE house, which has nothing loose to reseat | **code done** (`reseat` off for `pristine`/`scorched`); **rebake pending** |
| 5 | houses with a pool but no fence | the `large` package is the only one carrying `pool: 1.0` and it carries `fence: 1.0` with it, so the plat never issues one without the other. The fence is removed later by the all-or-nothing sweep, which does not know the lot has a pool | **TODO** |
| 6 | park trees too close to the basketball court | `BASKETBALL["pad"]` is 2.0 m of run-off and the planting keep-out is struck from the zone rectangle, so a crown can overhang the court | **TODO** |
| 7 | a car parked where the drive and the front walk are one strip | `plan_lot` strikes the drive kerb->GARAGE and the walk kerb->DOOR; on a style whose door adjoins the garage the two runs are within a metre and draw as one apron | **done** — car OBB tested against that lot's walk, rejects as `drive:front_walk`. 101 -> 83 driveway cars |
| 8 | cars parked too close to the house | `drive_nose_clear_m` 0.8 m with `back` capped at 1.0 m put every bumper 0.8-1.8 m off the wall | **done** — 2.0 m + up to 1.5 m set-back |
| 9 | the fire looks like it started in the park again | the park is CENTRAL on most seeds (seed 23: x -227..193, y -194..106), so any corner-to-corner diagonal crosses it | **done** — `FREEZE_EPICENTER` / `FREEZE_HEADING`; level_1 now ignites at (-460, 255) heading 0 and runs the northern edge |

## What still has to happen after the list clears

1. **Rebake the archetype library** for (4) — and remember that a fresh bake
   reintroduces the floating debris, so `bake.reseat_meshes_in_file` must run
   after it (see [fix-floating-debris](../fix-floating-debris/SKILL.md)).
2. **Rebuild level_1 AND level_2.** Every fix above is in the generator, so the
   frozen `level_2/1` and `level_2/4` on disk are stale.
3. **Then the people placements**, all five per level in one batch — deferred
   deliberately until the scenes are signed off, because each placement is a
   full 265 MB export and regenerating them after a scene change is pure waste.
4. **level_3** — the third layout at `severity 0.80` / `burn_frac 0.75`.

---

# Status, per cell

Legend: **ready** = the pipeline builds this today; **partial** = it exists but
something named is missing; **gap** = no pipeline.

| disaster | locale | state | what is missing |
|---|---|---|---|
| Fire | Suburban | **ready** | 1 km preset (`suburb_wildfire_1000`) exists and archetypes are freshly baked (78 archetypes, 154 MB, 0 unresolved, 2026-08-27). Levels 2 and 3 need their own seeds and a `fire_png` check |
| Fire | Urban | **gap** | no urban FIRE damage ladder. The urban kit has an earthquake archetype library (`archetypes_quake`) and nothing else; a fire ladder over the same kit is a bake, not a new pipeline |
| Tornado | Suburban | **partial** | pipeline complete at 500 x 500 (`suburb_tornado`); needs a 1 km preset and an archetype bake — `scene_gen/assets/archetypes_tornado/` currently holds only house cells |
| Tornado | Urban | **gap** | no urban wind ladder |
| Earthquake | Urban | **partial** | `downtown_earthquake` + `archetypes_quake` exist; PAUSED 2026-08-27 with the damage judged good and building diversity judged lacking |
| Earthquake | Suburban | **partial** | `suburb_earthquake.yaml` compiles; no suburban quake archetype bake |
| Hurricane | Urban | **gap** | preset compiles; DESIGNED 2026-08-29, no pipeline |
| Hurricane | Suburban | **gap** | preset compiles; DESIGNED 2026-08-29, no pipeline |

**Six of eight cells need work before they can be frozen, and two of them need
a pipeline that does not exist.** Fire/Suburban is the one that is ready today,
which is why it is first.

---

# Workflow

## Build one scene

    docker exec isaac-sim tmux send-keys -t isaac C-c
    docker exec isaac-sim tmux clear-history -t isaac
    docker exec isaac-sim tmux send-keys -t isaac 'clear; \
      SCENE_CONFIG=suburb_wildfire_1000 \
      ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes \
      MINI_SEED=11 MINI_BURN_FRAC=0.45 MINI_ELAPSED=0 \
      PEOPLE_VARIANT=0 FREEZE_DISASTER=wildfire FREEZE_SNAPS=1 \
      FREEZE_OUT=/isaac-sim/AirStack/final_disaster_dataset/Fire/Suburban/level_1/1 \
      PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh \
      /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/freeze_dataset_launch_script.py \
      --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts' ENTER

Never `airstack down` between iterations: it removes the container and throws
away the runtime-installed `manifold3d` / `shapely` / `mapbox_earcut`, and a
fresh container does not pay one pip install, it pays one RUINED RUN (trimesh
caches engine availability at import, so every capped slice comes back empty
and the banner still looks complete).

**Pass every knob explicitly.** The image declares `ARCH_DIR`, `ARCH_SEED`,
`SETTLE_STEPS` and friends as EMPTY strings, so `os.environ.get(k, default)`
never reaches its default. The launcher reads through its own `env()` helper
for this, and the command line above passes them anyway.

## Review it

`snaps/` holds an overview, a 3 x 3 walk of the plate at ~1/3 span (a 1 km
overview puts a house at a few pixels — it cannot be judged), and a close pair
on the refuge lot, each road blockage and one member of each survivor scenario.
The scene is left running in the GUI.

## Then freeze

Not implemented. `FREEZE_EXPORT=1` is the reserved knob; see the design above.

---

# Known gaps

- **The export does not exist.** Everything above the "freeze" heading works;
  the freeze itself is a design.
- **No urban fire, no urban tornado, no hurricane of any kind.** Half the
  matrix.
- **The intensity ladders for tornado and earthquake are proposals**, and the
  fire one has not been confirmed either.
- **The people head count per placement is undecided** — see GT_people above.
- **`Debris` is thin in a wildfire scene.** The only authored loose debris on
  the fire path is the road blockages; everything else is inside a baked
  archetype. A tornado plate is the opposite (thousands of loose boards), so
  the class will be wildly unbalanced across disasters and a consumer should
  not read the counts as comparable.
- **No OBB.** Boxes are world AABB plus a yaw; a house at 45 degrees has a box
  up to 41% larger than the building.
- **Nothing checks that the five people variants really are the same
  geometry.** They should be, by construction (`PEOPLE_VARIANT` touches
  `people.seed` alone), but a diff of the two `build_stats.json` house/tree
  tallies would prove it and nothing does that yet.
