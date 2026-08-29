---
name: build-hurricane-scenes
description: Build HURRICANE-damaged scenes in scene_gen — the near-uniform wind field with a rotating direction (not a track), the roof-first failure ladder that tops out where the tornado ladder starts, the surge/inundation water rendered as static geometry with a muddy MDL, and the water-damage signatures (mud line, windrow, scour) that separate a hurricane from a tornado from the air. Read before creating disaster/hurricane.py, a hurricane surge/water module, or any hurricane launcher. Prerequisites - build-tornado-scenes (the wind mechanics) and simulate-water-in-isaac-sim (the water tiers).
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Build Hurricane Scenes

**Status: DESIGN, nothing built yet.** This file is the living record. It is
written before the code so that the code has something to be wrong about.
Update it as things are built and as bugs are found — that is the point of it.

## Prerequisites, in order

1. [build-tornado-scenes](../build-tornado-scenes/SKILL.md) — the wind damage
   machinery (`wind_flow`, `planks`, `settle` bias, windthrown trees) is 80% of
   the hurricane's structural half. Read it and its prerequisite,
   [build-wildfire-scenes](../build-wildfire-scenes/SKILL.md), first.
2. [simulate-water-in-isaac-sim](../simulate-water-in-isaac-sim/SKILL.md) — the
   three water tiers. Tier 1 (static plane + water MDL) is the answer here.
3. [freeze-disaster-dataset](../freeze-disaster-dataset/SKILL.md) — hurricane is
   one of the four disasters in the 4 x 2 x 3 x 5 matrix and it is the ONLY one
   with no pipeline. Both its cells (Urban, Suburban) are gaps.

---

# What exists today (verified 2026-08-28)

Almost nothing. The word "hurricane" appears in exactly seven places and every
one of them is the LEGACY generic-damage path, not the archetype pipeline that
built the fire, tornado and earthquake scenes:

| where | what it is |
|---|---|
| `scene_gen/config/presets/hurricane.yaml` | 9 lines: `locale: downtown`, `disaster-type: hurricane`, `severity: 0.6` |
| `scene_gen/config/low_level/compiled/hurricane.yaml` | the compiled output of that |
| `scene_gen/compile_disaster.py:545` `compile_hurricane` | prop-toppling fractions + `field: {kind: uniform, inside: 1.0}` |
| `scene_gen/GENERATION.md:462` | "Tornado-like mechanisms spread evenly over the whole region at lower intensity. No untouched zone, no track." |
| `scene_gen/disaster/gt_hints.py:72,104` | `EXTRA_CLASSES["hurricane"] = ("Pool", "Parking Lot")`; tree classes copied from tornado |
| `scene_gen/tests/test_scene_modularity.py` | compiles the preset |
| `.agents/skills/freeze-disaster-dataset/SKILL.md:199` | "Hurricane — NOT DESIGNED" |

**The legacy `compile_hurricane` is not wrong, it is thin.** Its one real
insight — that the field is UNIFORM, with no untouched zone and no track — is
correct and survives into the design below (see "The model"). What it lacks is
everything else: no archetype ladder, no roof-cover damage state, no water, no
directional model, no tree model, no urban story.

---

# The model — PENDING (research in flight)

<!-- filled from _plans/hurricane_research.md -->

# Building damage — PENDING (research in flight)

<!-- filled from _plans/hurricane_research.md -->

# Water — PENDING (research in flight)

<!-- filled from _plans/hurricane_water.md -->

# Bug catalogue

Nothing yet. Every bug found while building this goes here, in the
item -> cause -> fix form the earthquake skill uses.
