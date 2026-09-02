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

# 3. THE BAKE — the blocker, and the way it fails is silent

**This supersedes `HURRICANE_RUNBOOK.md` §1**, which concluded "no bake
needed". That was correct for the six-level ladder and is wrong now.

The eight-level ladder needs three archetypes that **have never been baked
anywhere in this repository**:

    house_<style>_shingles_lost.usd
    house_<style>_cover_lost.usd
    house_<style>_deck_panels_lost.usd

8 styles x 3 levels = **24 archetypes**. Verify with:

    ls scene_gen/assets/archetypes_tornado/ scene_gen/assets/archetypes_hurricane/ \
      | grep -cE "shingles_lost|cover_lost|deck_panels_lost"      # must be 24

**HOW IT FAILS, AND WHY YOU WILL NOT NOTICE.** The assembly resolves a house
with `harch.get(key) or harch.get("house_<style>_pristine")`. A missing
cladding archetype therefore does not raise, does not warn, and does not leave
a hole — it silently substitutes an **undamaged house**. Level 1 is 20%
cladding rungs and level 2 is **85%**, so an unbaked library renders those two
cells as an essentially pristine suburb with some water in it, and the only
symptom is that the scene looks wrong.

The baker is `simulation/isaac-sim/launch_scripts/bake_hurricane_archetypes_
launch_script.py` (its `ROOF_LEVELS` already covers all seven non-pristine
states). **It needs `SimulationApp`** — it cannot be run offline.

Note also that `HOUSE_ARCH_DIR` defaults to `archetypes_tornado` while the
baker writes to `archetypes_hurricane`. Settle how the low rungs reach the
resolver (merged library, changed default, or both directories searched)
BEFORE the first bake — getting this wrong reproduces the silent-pristine
failure with a full library sitting on disk.

---

# 4. Export is not wired for this pipeline

`freeze_dataset_launch_script.py` **cannot drive a hurricane cell** and should
not be bent into doing so. Its only build path is `scene_api.build_scene()`,
which is architecturally a wildfire monolith: every damage decision keys off a
fire arrival-time field, `has_disaster` is gated on `config["disaster"]["fire"]`,
and there is no wind/surge logic anywhere in it. The hurricane pipeline is
~2,250 lines inline in `suburb_hurricane_launch_script.py`'s own `main()` and
is not an importable, disaster-agnostic function.

Export support therefore belongs INSIDE the hurricane launcher. The seven
pieces are line-cited in `HURRICANE_RUNBOOK.md` §2b. The genuinely new one is a
**`PEOPLE_VARIANT` knob, which that launcher does not have** — without it the
five-cast people axis cannot be produced at all. `gt_hints.py` is already
hurricane-aware, and `disaster.freeze.export_scene()` is fully generic and needs
no hurricane-specific change.

**`disaster/freeze.py` has ZERO offline test coverage.** Every previous freeze
was proven live against wildfire only. The hurricane will be its first
non-fire exercise, and its portability gate has never run on a stage carrying
surge water, rafts or washaway-displaced houses.

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

1. **The 24 cladding archetypes exist** (§3) —
   `ls scene_gen/assets/archetypes_* | grep -cE "shingles_lost|cover_lost|deck_panels_lost"`
   must print 24, and they must be on `HOUSE_ARCH_DIR`'s resolution path.
2. **The launcher calls the eight-level ladder** —
   `grep -c "hu.house_level_for_intensity" simulation/isaac-sim/launch_scripts/suburb_hurricane_launch_script.py`
   must be >= 1.
3. **The three presets compile at 1 km with distinct seeds** — load each and
   assert `region_m == [1000, 1000]` and seeds 13 / 10 / 19.
4. **Leaves are green** —
   `python3 -c "import sys;sys.path.insert(0,'scene_gen/tools');import bake_hurricane_trees as b;print(b.TREE_LEAF_TINT)"`
   must print `False`.
5. **The tree archetypes audit clean** —
   `python3 scene_gen/tools/hurricane_tree_audit.py` must report
   `0 unresolvable material binding(s), 0 ... 0 unresolvable relative Asset path(s)`.
6. **The offline suite is green** — the hurricane test files
   (`test_hurricane_{trees,fences,fence_axis,people,trap_debris,washaway_blockers,street_furniture,tornado_parity_*}.py`)
   plus `test_washaway_debris.py`.
7. **`PEOPLE_VARIANT` exists in the hurricane launcher** (§4). Without it the
   five-cast axis cannot be produced.

---

# 8. Open items

- The `_HOUSE_CUTS` / `intensity_field` calibration mismatch (§2), currently
  absorbed by one preset value.
- Export support inside the hurricane launcher, including `PEOPLE_VARIANT` (§4).
- `disaster/freeze.py` has no offline tests and has never run on a hurricane
  stage (§4).
- `washaway` reports an **implausible 42.2 x 33.1 m footprint on `h_49`** and
  substitutes a nominal 12 x 9 m. The guard works; the upstream units/frame bug
  it points at has not been chased.
- Douglas_Fir keeps 47-69% of its needles at every damage level (§5).
- The `freeze-disaster-dataset` skill describes level 2 as "the brown scene".
  That referred to the removed tree tint. Brown ground and scour are fine;
  leaves stay green.
