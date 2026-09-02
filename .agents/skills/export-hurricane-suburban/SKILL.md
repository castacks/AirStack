---
name: export-hurricane-suburban
description: >-
  Everything needed to take the Hurricane / Suburban cells from generator to
  `final_disaster_dataset/` — the three 1 km level presets and why each level is
  a different LAYOUT as well as a different severity, the eight-level house
  ladder the hurricane runs on (and the `_HOUSE_CUTS` / `intensity_field`
  calibration mismatch that forced level 3's gust to 80 m/s), the 24 cladding
  archetypes that MUST be baked or level 1 and 2 silently render undamaged, the
  fact that `freeze_dataset_launch_script.py` cannot drive this pipeline at all,
  the scene model as it stands after the 2026-09-01 review round (green stripped
  foliage, bent trees, fences, street furniture, casualties under debris), and
  the bug catalogue of traps that each cost a build. Read BEFORE baking,
  rendering, exporting or pushing any hurricane cell.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Export the Hurricane / Suburban dataset cells

**Status, 2026-09-01: PREPPED, NOT EXPORTED.** Presets exist and compile, the
scene model has been through a full review round on the live 500 m plate, and
the runbook is written. Nothing has been baked, frozen or pushed. Two things
block the first render and both are named below.

Companion files, all prerequisites:
[freeze-disaster-dataset](../freeze-disaster-dataset/SKILL.md) (owns the matrix
and the directory contract),
[freeze-disaster-dataset/HURRICANE_RUNBOOK.md](../freeze-disaster-dataset/HURRICANE_RUNBOOK.md)
(the mechanical command sequences — §2 to §5 are still current; **its §1 is
superseded by "The bake" below**),
[build-hurricane-scenes](../build-hurricane-scenes/SKILL.md) (the pipeline),
[run-isaac-sim-launcher](../run-isaac-sim-launcher/SKILL.md).

---

# 1. What is being produced

Hurricane / Suburban is **15 frozen scenes: 3 levels x 5 people variants**, over
**3 distinct geometries**. The directory contract is
`freeze-disaster-dataset`'s and is not restated here except for the one thing
people get wrong: capitalised in the PATH, lowercase in the FILENAME —
`Hurricane/Suburban/level_2/3/hurricane_suburban_lvl2_3.usd`.

**The level axis is a layout axis.** Three intensities of one plat would score
as three views of one scene, so each level has its own layout seed as well as
its own severity. The people axis is NOT: five draws over one geometry,
`PEOPLE_VARIANT=k` offsetting `people.seed` and nothing else.

## The three presets

`scene_gen/config/presets/suburb_hurricane_1000_l{1,2,3}.yaml`. All 1 km —
the 500 m `suburb_hurricane_500_l{2,3}` presets are development plates, **not**
dataset cells.

| | seed | `site_gust_mps` | `surge_m` | `shore_offset_m` | `slope_pct` |
|---|---|---|---|---|---|
| l1 | 13 | 38.0 | 0.9 | -520.0 | 0.5 |
| l2 | 10 | 55.0 | 2.0 | -520.0 | 0.5 |
| l3 | 19 | **80.0** | 2.8 | -520.0 | 0.5 |

**`shore_offset_m` and `slope_pct` are PLATE-RELATIVE and cannot be copied from
the 500 m presets.** Doubling only the offset gives a much drier plate (16% wet
at l2 against the 500 m's 38%). Self-similar scaling — offset x2, slope /2 —
reproduces the ladder: **86% / 63% / 46% dry** across the three levels.

---

# 2. The house ladder, and the one number that is not the design's

The hurricane runs on its OWN eight-level ladder,
`hurricane.house_level_for_intensity`:

    pristine  shingles_lost  cover_lost  deck_panels_lost
    roof_stripped  roof_collapsed  partial_collapse  leveled

It shipped for a while on the tornado's six-level
`tornado_level_for_intensity` (the 2026-08-31 "STREAM S" parity decision —
"tornado and hurricane are both largely wind damage"). That is defensible at
the top of the range and indefensible at the bottom, and level 1 is what
proves it. Replayed on level 1's own intensity field:

| ladder | level 1 |
|---|---|
| six-level (tornado) | 86.8% pristine, 13.2% `roof_stripped` |
| **eight-level (hurricane)** | **80% pristine, 20% `shingles_lost`** |

The six-level vocabulary has **no cladding rung**, so its only way to show any
damage at 38 m/s is to jump a house straight to `roof_stripped` — the entire
covering gone. Level 1 is specified as "cladding only; green scene with
litter", which that vocabulary cannot express at all.

Restored to the eight-level ladder 2026-09-01. `test_hurricane_tornado_parity_
launcher.py`'s ladder assertion is INVERTED accordingly; every other assertion
in that file (the swept override, `house_water_state` as the sole source of
`swept`, the archetype key format, the row-recolour truth table) is untouched
and still load-bearing.

## What the three levels actually produce

Replayed over 4,000 houses per level against each preset's own field:

| | gust | pristine | cladding rungs | `roof_stripped` | structural |
|---|---|---|---|---|---|
| l1 | 38 | 80% | **20%** | 0% | 0% |
| l2 | 55 | 15% | **85%** | 0% | 0% |
| l3 | **80** | 0% | 56% | 18% | **26%** |

## Why l3's gust is 80 and not the design's 70

**A calibration mismatch between two modules, and it is still open.**

`intensity_field` maps `i = gust/100` EXACTLY — verified 38 -> 0.380,
55 -> 0.550, 70 -> 0.700, 80 -> 0.800. But `_HOUSE_CUTS` puts `roof_collapsed`
at 0.780 and `partial_collapse` at 0.900, so at i = 0.70 almost nothing reaches
a structural rung: **7% structural**, against the **25%** that
`house_level_for_intensity`'s own docstring specifies for "level 3 (70 m/s)"
and against the dataset ladder's "you can see INTO buildings". 80 m/s gives
26%, which is both figures.

Re-cutting `_HOUSE_CUTS` was tried first and is the WRONG fix:

- Its three documented per-level targets are **mutually unsatisfiable**. Level 2
  is specified as *both* 18% pristine *and* 2% collapse, which needs a wider
  spread than the cuts plus jitter can produce.
- A uniform downward shift of 0.08 does fix level 3 (24% structural) and
  **destroys level 1**, collapsing its "cladding only" read from 80% pristine
  to 34%.
- The table is shared by every hurricane caller; one per-cell preset value is
  not.

80 m/s is 179 mph — a legitimate Cat-5 **local site gust**, which is the
quantity this knob has always been. The skill's own citation makes the point:
Marshall measured damage-derived gusts averaging 41 m/s inside a storm rated
Cat 3 at landfall, so site gust and Saffir-Simpson category are not the same
number.

**OPEN ITEM.** The `_HOUSE_CUTS` / `intensity_field` disagreement is papered
over for this cell by one preset value. It affects any other caller of that
ladder and deserves a proper reconciliation.

---

# 3. THE BAKE — DONE 2026-09-02, and the way it failed twice was silent

**This supersedes `HURRICANE_RUNBOOK.md` §1**, which concluded "no bake
needed". That was correct for the six-level ladder and is wrong now.

**Settled: ONE MERGED LIBRARY in `archetypes_hurricane`, and
`HOUSE_ARCH_DIR`'s DEFAULT NOW POINTS THERE.** The baker already builds all
seven non-pristine rungs and `_link_shared` copies `pristine`/`swept` across
from the tornado bake, so one directory carries the complete eight-rung ladder
(8 styles x 8 rungs = 64 house files) beside the 34 tree archetypes `ARCH_DIR`
reads out of the same place. The old default (`archetypes_tornado`, a
SIX-rung library) was the silent-pristine trap itself, and
`test_hurricane_tornado_parity_launcher.py` was *pinning* it — that assertion
is inverted now, with the reasoning in its docstring.

The bake command that produced the shipped library:

    ARCH_SEED=7 HUR_LINK_MODE=copy BAKE_STRICT=1 \
    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_hurricane \
    TORNADO_ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
    ... bake_hurricane_archetypes_launch_script.py

**It needs `SimulationApp`** — it cannot be run offline. ~3 minutes for all 56
roof states on an RTX 5090.

## 3a. FIRST GET THE ASSETS — none of this is in git

`scene_gen/**/*.usd` is gitignored, so a fresh clone has **no archetypes at
all** and `aec/` + `objaverse/` are empty too. Everything needed is on the
Nucleus mirror and pulls with a bare `omni.client.copy` (no SimulationApp,
~1 minute for the archetypes, ~1 minute for the rest):

| tree | files | size | why it is needed |
|---|---|---|---|
| `archetypes_tornado` | 74 | 478 MB | the `pristine`/`swept` rungs `_link_shared` copies |
| `archetypes_hurricane` | 34 | 195 MB | the tree ladder (houses are baked, not pulled) |
| `aec` | 3230 | **5.8 GB** | the green species USDs `_GREEN_SPECIES_USD` names |
| `objaverse` | 979 | 333 MB | 41 props; without them `compile_disaster` warns "will render as placeholder prisms" |

The modular house kit itself is NOT pulled — `modular_house` references
`omniverse://.../Library/Stages/RetroNeighborhood/...` directly and streams.

**`docker exec` WITHOUT `-i` DISCARDS STDIN.** `docker exec c bash -c 'cat >
/tmp/x.py' < local.py` writes an EMPTY file, then the run exits 0 having
printed nothing, which reads like a silent library failure rather than a
missing script. Use `docker exec -i`.

## 3b. The two silent bake bugs, both found by MEASURING the output

**The three POSE rungs baked out IDENTICAL TO PRISTINE.** `roof_collapsed`,
`partial_collapse` and `leveled` — 24 of the 72 files — came out with their
pristine roof and wall geometry to the centimetre. `terrace/roof_collapsed`
was byte-identical to `terrace/shingles_lost`. Nothing raised; the banner said
"exported 56 roof archetype(s), 0 failed".

*Cause.* Every wall/roof pose starts at `hurricane_flow._single_transform_op`,
which returns the prim's LONE `xformOp:transform` or `None` — and its own
docstring says what it was written against: "Every piece `bake.export_object`
writes carries exactly one of these". A kit piece on the BAKE stage does not:
`sg.apply_placements` authors a translate/rotateXYZ/scale STACK on the
placement holder and leaves the referenced mesh with no op of its own. So
`_single_transform_op` is `None` for every bay and every wall, each pose
returns 0, and `wreck_building` raises nothing. There is a second reason a
live-stage pose would be wrong even with an op to write to: `_hinge_matrix`
documents `old_matrix` as mapping local points to their CURRENT WORLD
position, and every pivot/cap/drop is compared against world Z from
`_floor_levels` — on a nested kit piece that silently mixes two frames.

*Fix.* The pose rungs now take a ROUND TRIP: export the untouched assembly
(`export_object` flattens to `/Baked/<mesh>`, one `xformOp:transform` each,
world-baked), reference it back at `/World/pose_bench/...` at identity, pose
THAT, and export the posed reference. The pose functions are unchanged — they
are simply handed their documented input. Before/after, on the exported files:

| | pristine | roof_collapsed | leveled |
|---|---|---|---|
| ranch roof z | 3.23–7.88 | **3.04–5.45** | **0.31–1.69** |
| ranch wall z | -0.01–3.50 | -0.01–3.50 | **0.06–0.60** |

0 bays posed became **154**.

**The baker's own check could not see it.** It snapshotted
`q.GetAttribute("xformOp:transform").Get()` before and after — which returns
`None` on a prim that has no such attribute, so the comparison was `None ==
None` and the warning it printed ("NONE posed — the roof is untouched") read
like a tuning problem rather than a structural one. The check now runs against
the referenced archetype, where every mesh really does carry one transform, so
a no-op shows as a matrix that did not change.

**A SCOPED REBAKE TRUNCATED THE MANIFEST.** `ARCH_LEVELS` exists so one rung
can be re-cut without touching the others, but `archetypes.json` was written
with a plain overwrite — so an `ARCH_LEVELS=roof_collapsed,partial_collapse,
leveled` pass left a **40-record manifest describing a 72-file library**, the
four covering rungs simply absent. It broke nothing downstream, which is
exactly what makes it dangerous: `suburb_hurricane_launch_script` builds its
`harch` index by GLOBBING the directory, never from the manifest, so the file
quietly stops being a description of the library and no consumer complains.
It merges on `(style, level)` now, newest wins, dropping records whose file is
gone.

## 3c. How the missing-archetype failure presents (unchanged, still true)

The assembly resolves a house with `harch.get(key) or
harch.get("house_<style>_pristine")`. A missing cladding archetype does not
raise, does not warn, and does not leave a hole — it silently substitutes an
**undamaged house**. Level 1 is 20% cladding rungs and level 2 is **85%**, so
an unbaked library renders those two cells as an essentially pristine suburb
with some water in it.

`test_the_eight_rung_hurricane_library_is_complete` now asserts a file exists
for every `hurricane.HOUSE_LEVELS` rung x every style, and was checked to go
RED on one removed archetype. That is the assertion whose absence cost this
build — its sibling proved the *tornado* six-level library complete and
therefore passed throughout.

---

# 4. Export is not wired for this pipeline

`freeze_dataset_launch_script.py` **cannot drive a hurricane cell** and should
not be bent into doing so. Its only build path is `scene_api.build_scene()`,
which is architecturally a wildfire monolith: every damage decision keys off a
fire arrival-time field, `has_disaster` is gated on `config["disaster"]["fire"]`,
and there is no wind/surge logic anywhere in it. The hurricane pipeline is
~2,250 lines inline in `suburb_hurricane_launch_script.py`'s own `main()` and
is not an importable, disaster-agnostic function.

Export support therefore belongs INSIDE the hurricane launcher, and
**it is wired now (2026-09-02)** — modelled on
`suburb_tornado_launch_script.py`'s own freeze block (the closest sibling: the
hurricane house loop is that file's loop verbatim) and on
`freeze_urban_fire_city_launch_script.py` for the two waive knobs. Knob names
are `freeze_dataset_launch_script.py`'s verbatim so the dataset tooling drives
a hurricane cell with the same lines it drives a wildfire one:

    FREEZE_OUT  FREEZE_NAME  FREEZE_EXPORT  FREEZE_COLLECT  FREEZE_SNAPS
    FREEZE_EXIT  FREEZE_WAIVE_VEGETATION  FREEZE_WAIVE_ABOVE_INSTANCES
    FREEZE_WAIVE_MIRRORED  PEOPLE_VARIANT

`FREEZE_OUT` moves `GT_people.json`, `GT_hurricane.json`, `GT_hints.json`,
`build_stats.json` and `snaps/` into the cell; unset reproduces the old
behaviour exactly. `FREEZE_EXIT` WINS over `KEEP_OPEN`, or a loop over cells
never reaches its second iteration.

**`PEOPLE_VARIANT` offsets ONLY the people RNG** —
`random.Random(SEED + 191 + 1000 * VARIANT)`. The wind field (`SEED + 23`),
houses/water (`+5`), trees (`+9`) and cars (`+77`) are untouched, which is
what makes the k cells of a level five casts over ONE geometry. Never fold the
variant into `SEED`.

**The cars had to be walked off the stage.** `gt_hints` wants
`info["cars"]`, `binfo["cars"]` is never filled by the suburb generator, and
this launcher's own car walk lives inside `if DO_WASHAWAY` — so a cell built
with the wash-away pass off would ship a hint file with no vehicles at all.
There is now an unconditional walk. Two traps in it:
`GetMetadata("references")` returns an `Sdf.ReferenceListOp`, which is **not
iterable** — the obvious one-liner raises `TypeError` on the first car and
takes the whole walk down — so read the asset path off `GetPrimStack()`
instead, checking all four list-op fields; and build a **fresh
`UsdGeom.XformCache`**, because the wash-away/surge/settle passes re-author
xformOps and a cache from earlier in `main()` reports every floated car at its
pre-drift pose. Measured result: 220 vehicles classified (186 Car / 5 Van /
29 Truck).

`info["blockers"]` is `[]` — this pipeline has no road-blockage model (its
land debris is a different, larger mechanism), so `Fallen Tree`/`Debris`
road-blockage records and the **`Toppled` street-furniture records are absent
from `GT_hints.json`** even though the scene places ~500 street-furniture
items. The tornado has the same gap. It degrades the hint file; it breaks
nothing.

**`disaster/freeze.py` had ZERO offline test coverage** and the hurricane was
its first non-fire exercise. It found a real problem — see §4a.

## 4a. THE PORTABILITY GATE FIRES ON THIS PIPELINE. Use the mirror waiver.

The first level-1 export **failed the gate**, and correctly:

    sky_lights            2  (/World/FrozenDome, /World/FrozenSun)   PASS
    cross_scope_bindings  0                                          PASS
    build_local         162  (34,217 bindings)                       FAIL

`make_portable` rewrote 1,226 asset paths to verified mirror targets, moved
and re-bound 23 cross-scope looks, and converged its de-instancing fixpoint in
one round — and **162 paths still came out build-local** in the flattened
file: 110 `objaverse/`, 42 `aec/`, 10 `materials/`. That is the scan-vs-flatten
divergence `freeze.make_portable`'s own dated note records (the live scan
reports 0 offenders while the cold file still carries the bindings), and
`FREEZE_WAIVE_MIRRORED=1` is the documented ship path for exactly it.

**Check the twins before turning the waiver on, not after.** The waiver waives
only paths with a stat-verified Nucleus twin and fails a twin-less one, so the
honest pre-flight is to stat all of them yourself:

    # for each build_local path P under /isaac-sim/AirStack/scene_gen/assets/
    omni.client.stat(ASSET_MIRROR + P[len(ASSET_LOCAL_PREFIX):])

162 of 162 existed, which is expected once §3a's pull has been done — the
local trees came FROM the mirror. If any is missing, staging it is the fix;
forcing the waiver is not, and `portable=False` is never the answer for a
dataset build.

**Verify COLD, on the artefact.** `_enforce_portable` gates the export, but it
verifies a file the same process just wrote. `freeze.verify(path,
expect_self_contained=False)` imports standalone under the container's
`omni.usd.libs` (no SimulationApp, sub-second) — re-open every shipped cell in
a fresh process and read `portable_ok` / `sky_lights` / `build_local` /
`cross_scope_bindings` back. That is `freeze-portable-scenes` checklist item 5
and neither sibling launcher does it.

Expect `ComputeAllDependencies` to fail with the kit `assetInfo` poison
(`_UnpackValue ... unsupported type enum value 0`) on every real cell; `verify`
falls back to a `Stage.Traverse()` shader-attribute scan and says so. That is
normal, not a defect.

---

# 5. The scene model as it stands

The 2026-09-01 review round changed a lot. Anything here that reads as a
preference is a decision with a measurement behind it.

**Trees are GREEN.** `bake_hurricane_trees.TREE_LEAF_TINT` defaults OFF. A tree
stripped by a few hours of hurricane wind has LOST foliage — the damage is
mechanical, and leaves still attached are the green they were that morning;
browning is a days-to-weeks response and this scene's epoch is hours after
landfall. `HUR_TREE_TINT=1` restores the dead-foliage colouring for a scene
deliberately depicting a later epoch.

**Foliage retention is real.** `_FOL_KEEP`: pristine 1.00, defoliated **0.14**,
limbed 0.09, leaning 0.07, fallen 0.05, snapped top 0.03 — the 86-94% Cat-3
broadleaf leaf loss the literature gives. This was 0.80 for `defoliated` while
a brown tint carried the damage signal; with the tint gone, retention carries
it. The survivors are biased to the upper/outer crown by a `"top"` selection
mode, which is what makes this safe: the pre-2026-08-31 attempt at 10-12%
rendered as bare pixels from 400 m because the survivors sat in the shaded
low/inner crown. **Verified on the live plate — trees read as green crowns at
full-plate zoom.**

*Douglas_Fir is unaffected*: a separate conifer floor (50-62%) now dominates
every damaged level since the broadleaf base dropped below it, so firs keep
47-69% of their needles. If firs look untouched beside stripped broadleaves,
that floor is the knob, not `_FOL_KEEP`.

**Bent trees.** `_DRY_WINDTHROW_SLOPE` 0.28 -> 1.60, tripling dry-ground
windthrow (88 -> 280 leaning on the reference plate). Two test bands were
widened deliberately to allow it — L3 structural 28-32% -> 28-40%, L3
dry-subpopulation 5-8% -> 5-25% — because those targets were solved when a
brown crown tint carried the damaged read; with colour gone, bent geometry is
the only cue and has to be commoner. **Both lower bounds were held**, so a
regression that quietly stops bending trees still fails. Yaw jitter narrowed
+-38 -> +-26 so a stand reads as one storm's doing.

*Pristine trees are no longer eligible for windthrow promotion.* A tree that
still has its full crown is precisely one the wind missed.

**Fences.** Not floating (see the bug catalogue), and no longer "too perfectly
fallen": `FENCE_WIND_GONE_SHARE` 0.42 (carried off on DRY land, which was
previously impossible — `gone` only fired past the surge depth),
`FENCE_PARTIAL_SHARE` 0.34 with leans 22-88 deg, plus per-panel skid
(`FENCE_SLIDE_M` 1.9) and twist (`FENCE_YAW_JITTER_DEG` 38) so a run that fails
together does not land in formation. Reference plate: 600 -> 234 gone, 158
flat, 208 standing.

**Street furniture** (`disaster/street_furniture.py`, new). Per-class and
DISCRIMINATING, which is the whole point:

| | l1 (38) | l2 (55) | l3 (80) |
|---|---|---|---|
| `sign` | 98% stand | 30/37/33 stand/lean/flat | 91% flat |
| `streetlight` | 100% stand | 100% stand | 67/22/11 stand/lean/flat |
| `trash_can` | 65% flat | 81% flat, 19% gone | 79% gone |
| `fire_hydrant` | **stands** | **stands** | **stands** |

`fire_hydrant` never moves — 0 of 8000 at maximum intensity — and is the
deliberate control proving the pass chooses rather than flattening everything.
**`trash_can` and `mailbox` have NO `leaning` rung**: a wheelie bin and a
post-mounted box are either upright or over, unlike a 9 m steel column that
bends at its base plate first. Signs were retuned away from 62-84% `gone`,
because a deleted sign is invisible to a detector while an uprooted one lying
on the verge is a findable object.

**Casualties.** Dry-land casualties are under real debris (the covering boards
were being planned and thrown away — see the bug catalogue), **41% inside a
wrecked footprint** (was 0%), median 8.6 m from the house, and **nobody is
thrown** — validated against the mortality literature and written up in
[build-hurricane-scenes/PEOPLE_RESEARCH.md](../build-hurricane-scenes/PEOPLE_RESEARCH.md).
Wind is 8% of direct US tropical-cyclone deaths and the Andrew medical-examiner
series lists every one at or under the structure; the ~37%-recovered-outdoors
signal that justifies the TORNADO's throw has no hurricane counterpart.

**A near-field debris apron** (`HUR_DEBRIS_APRON`, default 300/wreck) surrounds
each wrecked house isotropically, on top of the directional downwind comet. Two
mechanisms, two shapes: the collapse sheds its own material all round the lot
before the wind has any say. Measured effect on casualties left on bare grass:
**45% -> 13%**. At 48/wreck it was worth almost nothing (42%) — 1.4% ground
coverage.

**Roads** use aspect-preserving COVER mapping (`roads.stretch_cover_m`, default
14 m), not a single image stretched over each segment.

---

# 6. Bug catalogue — every one of these cost a build

**`Gf.Rotation`'s `*` composes LEFT-OPERAND-FIRST.** `apply_fence_pose`'s
docstring asserted the opposite for as long as it existed and it is simply
false: `(Gf.Rotation(Z,90) * Gf.Rotation(X,90)).TransformDir((1,0,0))` lands at
`(0,0,1)`, only reachable by applying the LEFT operand first. The wrong order
tipped every panel while its geometry was still in its unplaced local frame,
using a hinge axis expressed in WORLD coordinates. Measured Z extent across
four bearings: 3.510 / 2.851 / 0.080 / 3.215 m — panels standing on their
3.51 m END at three of four bearings, correct at 90 deg by coincidence.

**A placed prim's translate z is NOT the ground.** `apply_placements` folds the
asset's anchor->centroid offset into the translation, so seating a tipped item
by putting its lowest point at `t[2]` parks it that offset ABOVE the grass.
Measure the item's own lowest world point BEFORE re-authoring anything.

**One `UsdGeom.XformCache` must not span a mutation.** It memoises per-prim
transforms and does not invalidate when xformOps are re-authored, so a
post-pose measurement silently reads STALE pre-pose transforms — objects
landed ~0.8 m off. Build a fresh cache per measurement.

**`blocked=` is a CALLABLE, and this shipped wrong TWICE.**
`washaway.shift_spec` / `collapse_spec` / `car_shift_spec` march a drift against
`blocked(x, y) -> tag`. Handing them a list raises INSIDE the march, the
surrounding `except` swallows it into one warning line, and the whole pass
silently does nothing — first for cars ("0 moved"), then for houses, where
every shifted and collapsed house kept its original pose while the tally still
reported them as displaced. Guarded now by
`test_hurricane_washaway_blockers.py`, both call sites, with an inline control
asserting a list still raises.

**A planned decision that is never authored.** `hurricane_people.plan_people`
returns `(humans, debris, records)`; the launcher bound `p_debris` and never
built it, so 24 of 28 dry casualties were labelled as buried with 35 boards
that existed only in the JSON. Ground truth described a covering that was not
on the stage.

**`ctx["deck_points"]` was `[]`.** `_Deck` measures the plank field, and on a
wrecked lot the boards are the THIN part of the debris — the deep part is the
baked wreck USD, which the planner cannot see because it is a referenced
INSTANCE. With no archetype samples the deck reads ~0 over the whole lot,
`_DECK_BAND["pile"]`'s floor never clears, and every casualty is pushed off the
wreckage. `tornado_people._Deck`'s own docstring records the tornado hitting
this exact failure in the reviewer's words.

**`tint=None` unbinds materials.** `_author_foliage_cull` guards BOTH the
replacement-material creation AND the `Bind()` behind `if tint is not None`, so
switching the tint off that way left 27 foliage meshes with NO MATERIAL BOUND —
flat default grey. Use a neutral sentinel resolved at ONE point, and note that
`_hue_flip_tint` FORCES R > G > B "for ANY texel mean": re-aiming it at the
leaf's own colour still returns `(1.000, 0.683, 1.000)`, so the solve must be
short-circuited, not re-targeted.

**Do not let a test's oracle carry the bug it hunts.** The fence-axis test
measured extents by projecting AABB corners, which over-reads for a diagonally
lying panel — it reported 4.91 x 4.52 m where the truth is 3.51 x 1.83 and
FAILED a correct fix. Points-based, always. Likewise `measure_fence` promised
"points-based, never BBoxCache" while projecting the AABB *of* the points, and
over-read `length_m` by up to the panel's thickness.

**An absolute threshold breaks when the range shrinks.** The `limbed` asymmetry
test demanded a 15-percentage-point gap, which was satisfiable at 35% retention
and arithmetically IMPOSSIBLE at 9% — the whole range is 12 points wide. Ratios
are scale-free; use them for shape properties.

**`git stash push -- <paths>` with an untracked path in the list** stashes the
WHOLE working tree (including other sessions' uncommitted work) before
rejecting the pathspec. To A/B a fix, copy to the scratchpad and edit the copy.

---

# 7. Pre-flight checklist

Each line has the command that checks it. Run all of them before the first
render.

0. **The assets are on disk at all** (§3a). A fresh clone has NONE of them —
   `scene_gen/**/*.usd` is gitignored, `aec/` and `objaverse/` likewise.
   `ls scene_gen/assets/archetypes_tornado/*.usd | wc -l` (expect 71+) and
   `du -sh scene_gen/assets/aec` (expect ~5.5 GB). A compile that warns
   "N Objaverse asset(s) are not cached and will render as placeholder prisms"
   is telling you `objaverse/` is empty.
1. **The full eight-rung library exists**, 8 styles x 8 rungs = **64** house
   files in `archetypes_hurricane` —
   `python3 -m pytest scene_gen/tests/test_hurricane_tornado_parity_launcher.py -q`
   now asserts exactly this (`test_the_eight_rung_hurricane_library_is_complete`).
   A bare `ls | grep -c` is weaker: it cannot tell you WHICH style/rung pair is
   missing, and a missing one is silently replaced by a pristine house.
2. **The launcher calls the eight-level ladder** —
   `grep -c "hu.house_level_for_intensity" simulation/isaac-sim/launch_scripts/suburb_hurricane_launch_script.py`
   must be >= 1.
3. **The three presets compile at 1 km with distinct seeds** — load each and
   assert `region_m == [1000, 1000]` and seeds 13 / 10 / 19. The host needs
   `pyyaml`/`numpy` and a stubbed `pxr` (the pattern is in
   `suburb_hurricane_500_l2.yaml`'s header).
4. **Leaves are green** — `TREE_LEAF_TINT` must be `False`. Read the default
   off the source (`grep -A1 "^TREE_LEAF_TINT" scene_gen/tools/bake_hurricane_trees.py`
   -> `HUR_TREE_TINT` defaulting to `"0"`); importing the module needs `pxr`
   and `numpy`, which the host does not have and Kit's python does not either.
5. **The tree archetypes audit clean** —
   `python3 scene_gen/tools/hurricane_tree_audit.py`. NOTE it cannot run in
   this container: it imports `bake_hurricane_trees`, which imports `numpy`,
   which Kit's python lacks. Either install numpy for it or verify the trees
   from the built scene instead (`trees N placed, 0 fell back to the green
   species USD` in the run log is the equivalent live check).
6. **The offline suite is green** — the hurricane test files
   (`test_hurricane_{fences,fence_axis,people,trap_debris,washaway_blockers,street_furniture,tornado_parity_*}.py`)
   plus `test_washaway_debris.py`. Run them PER FILE, not batched.
   `test_hurricane_trees.py` cannot collect on a host without `pxr` — that is
   an environment gap, not a failure.
7. **`PEOPLE_VARIANT` and the `FREEZE_*` knobs exist in the hurricane
   launcher** (§4) — `grep -c FREEZE_OUT` must be nonzero.
8. **The Nucleus target is where you think it is, and additive** —
   `omni.client.list` the dataset root and confirm whether `Hurricane/`
   already exists before the first upload.
9. **Nobody else is on the container.** `docker inspect isaac-sim` and read
   the `/isaac-sim/AirStack` MOUNT SOURCE: on a shared box it may be another
   user's checkout entirely, in which case none of your code or assets is
   what the launcher will see. `container_name: isaac-sim` and the fixed IP
   `172.31.0.200` mean there can only be one.

---

# 8. Open items

- The `_HOUSE_CUTS` / `intensity_field` calibration mismatch (§2), currently
  absorbed by one preset value.
- **The 162 waived build-local paths (§4a).** The cells ship because every one
  has a verified Nucleus twin, not because the paths are right. The underlying
  scan-vs-flatten divergence in `make_portable` — the live scan reporting 0
  offending prototypes while the cold flattened file still carries the
  bindings — is unfixed, and a consumer with no route to
  `airlab-nucleus.andrew.cmu.edu` cannot render these cells. `FREEZE_COLLECT=1`
  is the real fix and is off for the same reason it is off on every shipped
  cell (the `Usd_CrateFile::_UnpackValue` poison).
- **`GT_hints.json` carries no street furniture.** `info["blockers"]` is `[]`
  because this pipeline has no road-blockage model, so the ~500 signs,
  streetlights, hydrants and bins the scene places — and the whole `Toppled`
  class with them — are absent from the hint file. The tornado cells have the
  identical gap. Deciding whether `street_furniture.py`'s own tally should feed
  `gt_hints` is a design call nobody has made.
- **`disaster/freeze.py` still has no offline tests.** It has now been run on a
  hurricane stage (three cells, §4a) and behaved correctly, including failing
  the gate when it should have — but that is one exercise, not coverage.
- `washaway` reports an **implausible 42.2 x 33.1 m footprint on `h_49`** and
  substitutes a nominal 12 x 9 m. The guard works; the upstream units/frame bug
  it points at has not been chased.
- **The level-1 review snapshots have nothing to photograph.** `worst_house`
  and `stripped_roof_house` come back 47-72% background because level 1 has no
  house above `shingles_lost` at all; the launcher says so
  ("no cover_lost/deck_panels_lost/roof_stripped house on this plate") and
  falls back to the most severe available. Correct behaviour, useless frames —
  the subject list should skip a subject the plate cannot supply rather than
  render the plate edge.
- Douglas_Fir keeps 47-69% of its needles at every damage level (§5).
- The `freeze-disaster-dataset` skill describes level 2 as "the brown scene".
  That referred to the removed tree tint. Brown ground and scour are fine;
  leaves stay green.
