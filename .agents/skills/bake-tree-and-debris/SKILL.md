---
name: bake-tree-and-debris
description: Bake the TREE half of a damage archetype library — windthrown trees, their broken limbs and the log/debris field around them — so that every piece comes to rest on the ground instead of floating over it or sinking through it. Covers which debris gets physics and which is only laid flat by arithmetic (SIM_ALL_DEBRIS), why a branchy limb needs a convex DECOMPOSITION and a stick does not (SETTLE_DECOMP_M), the drop-datum bug that lifted seated logs by |min-z|, and the KEEP_PHYSICS / bake-in-place loop that is the ONLY way to ship a pile somebody has actually looked at. Read before touching bake_tornado_archetypes_launch_script.py's tree loop, vegetation.wood_debris / fell_branches / wind_tree, or bake.export_object's seating.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Bake trees and their debris so nothing floats

The tree half of an archetype bake is not a smaller version of the house half.
A house is one object that breaks into fragments which fall a couple of metres
onto their own slab. A tree is **one authored pose plus three unrelated debris
populations**, only some of which are ever simulated, thrown up to 25 m, and
the thing everybody looks at first is whether the logs are lying on the grass.

Read [build-tornado-scenes](../build-tornado-scenes/SKILL.md) first for the
wind pipeline as a whole. This file is only about getting the tree cells and
their debris to REST.

## The four populations, and only two of them touch the solver

`vegetation.wind_tree` returns four lists and they are not interchangeable.
Reading the wrong one is how "the branches never settled" gets misdiagnosed.

| list | what it is | physics? |
|---|---|---|
| `loose` | `fell_branches` limbs, and every `wood_debris` piece longer than `simulate_above_m` | **rigid bodies** |
| `seated` | every `wood_debris` piece SHORTER than that | **none** — laid flat by arithmetic |
| `statics` | the snapped spar, the root ball | colliders only |
| `anchored` | the tree itself + the root ball | never moved by any seating pass |

**`seated` IS THE ONE THAT SURPRISES PEOPLE.** `wood_debris` seats anything
under `simulate_above_m` (default **0.8 m**) by translating it so its own
lowest point is at `ground_z - 0.001`, and hands it back as static geometry.
The argument is sound at plate scale — a 0.2 m stick lying flat looks the same
whether PhysX put it there or arithmetic did, and 15,000 of them is 15,000
colliders to cook — but it means **most of the debris under a tree is never
simulated**, and no amount of raising the step budget will change where it
sits. Measured on this library at the default threshold: 23 tree cells,
510 rigid bodies and 90 seated pieces.

`SIM_ALL_DEBRIS=1` sets the threshold to 0 so every piece is thrown, falls and
comes to rest under the solver: 597 rigid, 0 seated. **On a 23-cell tree grid
that costs nothing** (20-30 s of solving) and it is the right setting whenever
the question being asked is "why did these not settle". It is NOT obviously
right for a full plate, which is what the threshold exists for.

## The per-cell line that makes all of this visible

The bake prints one row per tree cell, and it is the first thing to read when
debris looks wrong:

    [tarch]   Shumard_Oak  limbed  bodies  13 loose (limbs 10) / 0 seated (no physics) / 0 static

`limbs N` is what `wind_plan` ASKED `fell_branches` for, not what it made —
and the gap between those two is itself a finding, because **half this library
has no woody PointInstancer to bake limbs out of**. Largetooth_Aspen, American
Beech and Common Apple ship their crown as one plain mesh, so they get zero
limbs however many are requested and their only debris is `wood_debris`. A
species whose branches look untouched is usually this, not a solver problem.

## A BRANCHY LIMB NEEDS A DECOMPOSITION. A STICK DOES NOT.

This is the single biggest difference from the house bake and it is the fix for
"the broken branches didn't settle".

`settle.prepare` cooks every dynamic collider as a **convex hull** by default,
and for a wall fragment that is both right and cheap — the hull IS the piece.
A limb torn off an oak is not convex: it is a stem with sub-branches coming off
it, and its hull is the blob that contains all of them. The limb then comes to
rest balanced on the hull of its own branch tips, **standing on end and part
buried**, which is exactly what it looks like on screen. Measured on a Shumard
oak `limbed` cell before the fix: three limb meshes spanning z **-0.98 to
+2.23** — 3.2 m tall, a metre into the ground, beside a trunk that was
perfectly seated.

`SETTLE_DECOMP_M=2.5` sends any loose piece whose world bbox DIAGONAL is 2.5 m
or more to `convexDecomposition` and leaves everything smaller as a hull. On
the tree grid that is **42 of 597 bodies** — the limbs and the longest logs,
which is precisely the set that needs it.

**DO NOT turn decomposition on for everything.** The wildfire bake's
`approx_map = {p: "convexDecomposition" for p in tree_loose}` cost **20+
minutes on a settle that takes 61 seconds with hulls**: branchy pieces hit
`ConvexDecompositionTask: polygon limit reached` and the hull sets that come
out of that make every solver step crawl. There is no error and no traceback —
the process sits at ~30% of one core logging nothing, and that log line is the
entire diagnosis. `settle.DECOMP_LIMITS` (8 hulls, 32 vertices) is what keeps
the per-piece cost bounded; `decompose_larger_than` is what keeps the COUNT
bounded. Use both.

## `drop_to_ground` USED TO LIFT EVERY CORRECTLY-SEATED LOG

Recorded in full in the tornado skill (§ "Cause 4"); the short version, because
it is a tree-specific trap and it will be re-introduced by anyone who
reasonably assumes the drop is harmless:

`bake.export_object(drop_to_ground=True)` takes the world min-z of the FIRST
root — the tree — and subtracts it from **every mesh in the file**. A tree
whose own geometry dips below grade therefore has a NEGATIVE datum, and
subtracting it raises the whole file, seated logs included, by `|min-z|`.
Measured: `tree_Shumard_Oak_snapped` and `_limbed` had three independent sticks
each sitting at **exactly 0.53 m** with clear air under them, 35-40% of all
meshes unsupported, while every species whose base is at its origin was clean
and both `_NO_DROP` levels were clean at every species.

Now: the drop moves the OBJECT, the debris keeps the absolute Z it was authored
at, and `bake._reseat_roots` corrects only what the solver actually misplaced.
The bake prints the datum per archetype so it can never be invisible again —

    [tarch]   seat  tree_Shumard_Oak_limbed.usd   datum -0.533 m, 13 root(s) moved, worst 0.533 m

A positive datum is just as real: `snapped` deactivates the bole, so the tree
root's remaining geometry can start well ABOVE grade (measured +2.14 m on
Largetooth Aspen) and the drop then LOWERS the file by that much.

## FREEZE THE TREE AND THE ROOT BALL, SEAT EVERYTHING ELSE

Trees export with `reseat=True, reseat_first=False, reseat_freeze=anchored`.
Both frozen prims look exactly like "sank through the floor" to a sink test and
both are authored on purpose:

- `tip_tree` bisects the lean down until the crown is just INTO the turf
  (`seat_band = (-1.1, -0.15)`). Reseating a fallen tree stands it back up on
  the lawn and deletes the single most legible feature of a track from the air.
- the root ball is centred on `lift = r_plate * 0.5` so it STRADDLES the tipped
  trunk's open base, which puts its underside at `-r_plate / 2`. Reseating it
  lifts it off the end of the trunk it exists to cover.

`wind_tree` publishes both as `res["anchored"]`; pass that straight through. A
frozen root still contributes its box, so a log can come to rest ACROSS a
fallen trunk.

## THE PILE YOU LOOKED AT IS THE ONLY ONE THERE IS

**A settle does not reproduce, and this is the most expensive lesson here.**
Two back-to-back runs of this script with the same seed, the same knobs and the
same code finished:

    run A   800 of 8000 steps;    0 body(s) still moving;   0 clamped after the bake
    run B  2000 of 8000 steps;    1 body still moving;    595 clamped after the bake

Same arguments. A solver stepped by a live Kit app is at the mercy of frame
timing, and "re-run it with the same seed to export it" throws away the good
pile and ships a different one. It was done once, on a pile that had been
looked at and approved, and the result was worse.

So the review loop does not re-run anything:

```bash
# 1. build, settle, and STOP — physics still on, gravity on, timeline paused
ARCH_KINDS=tree KEEP_PHYSICS=1 SIM_ALL_DEBRIS=1 SETTLE_DECOMP_M=2.5 \
SETTLE_STEPS=2400 SETTLE_MAX_STEPS=8000 SETTLE_QUIET_STEPS=1200 \
ARCH_SEED=7 ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
  <the usual python.sh line>

# 2. look at it. PLAY steps it further if anything is still in the air.
#    STOP then PLAY replays the whole settle from the authored start.

# 3. when it looks right, bake WHAT IS ON SCREEN — no second settle:
docker exec isaac-sim touch /isaac-sim/.bake_now
```

`KEEP_PHYSICS=1` writes NOTHING and returns nothing to disk, so the library
already on disk is safe while the new one is being judged. The trigger file
makes the run call `settle.bake` on the bodies exactly where they stand, strip
the physics, and run the ordinary export / audit / manifest path on that stage.

**PAUSE, NOT STOP, when holding a settled pile.** `timeline.stop()` rewinds
every rigid body to its AUTHORED transform, which for this grid is the
pre-settle pose — every log back up in the air where `wood_debris` threw it.
That reads as "it's floating again" and is just the rewind.

## What the approved bake measured

The library baked in place on 2026-08-27 from a pile that had been looked at,
7,467 rigid bodies frozen where they stood:

| | before | after |
|---|---|---|
| pile-level meshes with nothing under them | 11-29% | **0.1%** (4 of 6442) |
| pile-level meshes below grade | 21-42% | **0.5%** (30 of 6442) |
| `tree_Shumard_Oak_snapped` airborne | 39.5% | **0.0%** |
| `tree_Shumard_Oak_limbed` airborne | 35.3% | **0.0%** |

Every tree archetype measures 0.0% airborne on an independent bare-`pxr` probe
except Black_Oak, whose flagged meshes are `Black_Oak_branch2` at z = 10.4 and
11.6 m and `Black_Oak_branch1` at 0.8-1.2 m — **crown branches on a standing
tree**, which is the audit's known false positive and not debris at all.

## Verifying without Isaac

The exported archetypes can be measured from a bare `pxr` under the container's
python, no `SimulationApp`, safe beside a running sim — see the
[run-isaac-sim-launcher](../run-isaac-sim-launcher/SKILL.md) standalone `pxr`
section. Per mesh: world bbox, and "is there anything overlapping me in plan
whose top lands inside my vertical span". That measurement is what found the
0.53 m lift, and the bake now runs it on itself (`ARCH_AUDIT=1`) and prints a
table per archetype.

**Read the `pristine` / `roof_stripped` / `limbed` / `leaning` rows as false
positives** — a standing tree's crown legitimately has air under it. Only the
pile levels (`fallen`, `snapped`, and the house `leveled` / `swept` /
`partial_collapse`) are signal.

`scene_gen/tests/test_bake_reseat.py` pins the seating arithmetic offline — the
composition of the drop and the reseat, the freeze, and the Shumard case as a
regression — with a stand-in bbox cache and no GPU.

## The environment traps that cost whole runs

- **The container exports these env vars EMPTY**, so
  `os.environ.get("ARCH_SEED", "7")` returns `""` and `int("")` raises fourteen
  seconds into the launch. Worse, `ARCH_DIR` becomes `""` and the library is
  written into the container's CWD with a clean banner. Every knob goes through
  `_env()` now; pass them explicitly on the command line anyway.
- **`ARCH_KINDS=tree` does not touch `house_*.usd`** — the house loop is
  skipped entirely and `merge_manifest` keeps the house records. That is the
  safe way to iterate on trees against a house library somebody has approved.
  Back the approved half up anyway before a combined run.
