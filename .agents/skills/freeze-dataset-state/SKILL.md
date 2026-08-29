---
name: freeze-dataset-state
description: >-
  What is ACTUALLY in `final_disaster_dataset/` right now, and what is deliberately not. The frozen scenes are baked-in POSITIONS with textures and MDL still referenced from Nucleus and the repo; the review camera is present but deactivated; the collect/localisation step is off and why it stalls. Also: the `FROZEN_SCENE` launcher path that flies a cell, and the damage-derived search area a benchmark gets from one. Read before consuming a frozen scene, before assuming a cell is redistributable, and before starting the public-release cleanup — this file is the punch list for it.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: The state of `final_disaster_dataset/`

## What these files are FOR, today

**Running baselines on this machine and on machines that can reach the same
Nucleus — and as of 2026-08-29 there is a launcher path that does it.**
`FROZEN_SCENE=<cell>` on `example_multi_drone_scene_import.py` references a
cell in instead of building a plat, and
`simulation/isaac-sim/utils/frozen_annotations.py` reshapes the cell's own
`GT_people.json` / `GT_hints.json` into the three names the stack reads
(`<scene>.json`, `<scene>_obstacles.json`, `<scene>_region.json`). The search
area a benchmark flies is measured from the DAMAGE in the cell, not from the
disaster model — the tornado cells have no `build_stats.json` to re-derive a
model from. Full account:
[benchmark-disaster-dataset](../benchmark-disaster-dataset/SKILL.md) §2b. It is
unit-tested offline and **has not been launched**.

The scenes are frozen in the sense that matters for a benchmark: the layout,
the damage, the debris and the people are fixed, baked in, and identical every
time the file is opened. Nothing about the geometry depends on rebuilding
anything — which is what lets four search methods be compared on one scene and
have the difference mean something.

**They are NOT redistributable yet.** A frozen scene still references its
textures and MDL modules from Nucleus and from this repo, so it will open with
missing materials on a machine that has neither. Making them standalone is the
public-release job and is deliberately deferred — see *The punch list*.

The matrix, the directory contract and the per-disaster intensity ladders live
in [freeze-disaster-dataset](../freeze-disaster-dataset/SKILL.md). This file is
only about the condition of what is on disk.

---

# Where it lives

    host       ~/SEI-COA/final_disaster_dataset          (FINAL_DATASET_DIR in .env)
    container  /isaac-sim/final_disaster_dataset

Bind-mounted in both the `isaac-sim` and headless compose blocks, and
**outside the repo on purpose**: at the full matrix this is hundreds of MB a
cell across 120 cells, which has no business in the working tree.
`final_disaster_dataset/` is still in `.gitignore` in case someone points
`FREEZE_OUT` back inside the repo.

**Adding that mount needed a container recreate**, and `airstack down` throws
away the runtime-installed `manifold3d` / `shapely` / `mapbox_earcut`. The
assembly path does not fracture so it does not need them, but reinstall them
anyway.

# What a cell contains

    <Disaster>/<Locale>/level_<n>/<k>/
        <disaster>_<locale>_lvl<n>_<k>.usd     the frozen scene, ~265 MB
        GT_people.json                          every survivor
        GT_hints.json                           every other labelled object
        build_stats.json                        what build_scene returned
        snaps/                                  review captures (variant 1 only)

`Materials/` is in the contract and is **not written yet** — nothing localises
the textures, so there is nothing to put in it.

---

# The condition of the frozen USD

## Positions are baked. Looks are referenced.

`omni.usd.get_context().export_as_stage_async` composes the whole stage into
one layer: every archetype reference is resolved in, every transform is
resolved, and the file stands alone for GEOMETRY. Measured on
`Fire/Suburban/level_1/1`:

    265 MB      447,450 prims      239,948 meshes      88 prototypes
    extent 1031 x 1031 m, z -4.0 .. 19.7

**Instancing is preserved** — 88 prototypes back ~5,800 referenced objects,
which is the only reason a 1 km plate fits in 265 MB rather than tens of GB.
Verified before the exporter was written: 40 instanceable references to a
1.0 MB archetype flatten to 1.0 MB with one prototype retained.

What is still external: **textures and MDL modules**, on
`omniverse://airlab-nucleus.andrew.cmu.edu` and under
`scene_gen/assets/` in this repo, plus one https:// sky texture from
NVIDIA's S3. Open the file on a machine with both and it renders correctly;
open it elsewhere and the geometry is right and the materials are missing.

## The review camera is PRESENT and DEACTIVATED

`snapshots.place_camera` reuses one prim, `/World/ReviewCamera`, so a frozen
scene would otherwise carry whatever pose the last capture left it at. It is
now `active = false`.

**Deactivated rather than removed, on purpose.** `stage.RemovePrim` edits the
current EDIT TARGET, so on a composed stage it can report success and leave the
prim composing from a stronger layer — measured: a run whose banner said it had
stripped the camera produced a file that still contained it. `SetActive(False)`
is an opinion that composes, so it always takes, and it is reversible.

Same treatment for Pegasus' default `/World/Environment`, which is loaded only
to give the World a base.

## The survivor locator poles

`build_scene` authors one 25 m magenta pole per survivor GROUP under
`/World/stage/generated/_people_poles` and leaves the scope **deactivated** on
every run. In `level_1/1` the scope was REMOVED outright (that export used the
earlier strip-not-deactivate path); from `level_1/2` onward it is present and
inactive, like the camera.

**This is the answer key.** It is inert as shipped — nothing renders and
`PEOPLE_POLES` is not set — but it is one prim toggle from marking every
survivor in a file handed to a searcher. The locations are in `GT_people.json`
where they belong. **Removing the scope is on the punch list**, and until then a
scored run must not toggle it.

## Debris is seated on real geometry

Every loose piece in the archetype library was re-seated on POINTS rather than
on `BBoxCache` — the whole story is in
[fix-floating-debris](../fix-floating-debris/SKILL.md). Acceptance as shipped:

    6,005 loose pieces (log_ / frag_ / brk_ / debris / bole_)
    p50 0.0000   p90 0.0000   p99 0.0000   MAX 0.0197 m
    over the 2 cm bar: 0

**The repair is applied to the baked archetype FILES, not to the bake code.**
`bake_archetypes_launch_script.py` still seats through `BBoxCache`, so a fresh
archetype bake reintroduces the floating and must be followed by
`bake.reseat_meshes_in_file`. That is the single most important thing to know
before re-baking anything.

---

# The collect step is OFF

`FREEZE_COLLECT=0` is the default. `disaster.freeze.export_scene` flattens and
lands the file; it does not localise textures.

**It is off because it stalls, and the cause is understood.** Kit's flatten
carries the kit assets' poisoned `assetInfo` into the flat layer, so every
core-USD traversal of that file raises

    Usd_CrateFile::_UnpackValue : Attempted to unpack unsupported type enum value 0

**11,405 specs carry the field and 11,069 of them refuse `ClearInfo`.** The
collector burns 150-200% CPU on unpack failures and writes nothing for 20+
minutes. Measured on the real file.

The way in is probably `Sdf`-level: `Sdf.Layer.Traverse` walks all 1,208,791
specs in **1.8 s** and `ListInfoKeys` never raises, so the poisoned field can be
FOUND cheaply — it is only `ClearInfo` that dies. `Sdf.Layer.EraseField` is the
obvious next thing to try and **has not been tried**. If it works, strip
`assetInfo` from the flat layer and the collector should run normally.

---

# The punch list for public release

In rough order of how much it matters:

1. **Localise textures and MDL** into `Materials/` — the `assetInfo` strip
   above, then `omni.kit.usd.collect`, then `freeze._relocate` (written and
   unit-checked, not yet exercised on a real scene). Until this is done the
   dataset is not distributable.
2. **Remove the locator poles**, not just deactivate them. An answer key inside
   the artefact is a correctness problem, not a tidiness one.
3. **Remove the review camera and the Pegasus environment**, once removal is
   done at the `Sdf` layer where it actually takes.
4. **Drop the colliders.** ~36,700 static triangle-mesh colliders were cooked
   for debris nothing will ever touch; a scene meant to be flown does not need
   them and they are a large share of the load time.
5. **Decide on the Flow prims.** The wildfire scenes carry NVIDIA Flow emitters
   for flame and smoke. They export as ordinary prims, render only with
   `omni.flowusd` enabled, and mean nothing to a non-Isaac consumer. Kept
   because they are tiny.
6. **Five full copies per level is the current cost of the people axis.** The
   geometry is bit-identical across variants 1-5 and each carries its own
   265 MB. A sublayer split would collapse that at the price of each `.usd` no
   longer standing alone — worth revisiting once the size of a localised cell
   is known.
7. **`GT_hints.json` boxes are world AABB plus a yaw**, not oriented boxes. A
   house at 45 degrees has a box up to 41% larger than the building.

---

# What is verified, and how to re-verify it

    # every loose piece in the archetype library, on points
    tools/debris_float_probe.py --archetypes scene_gen/assets/archetypes

    # a frozen cell: prim/mesh/prototype counts, extent, dangling references
    python3 -c "from disaster import freeze; freeze.report(freeze.verify(PATH))"

`freeze.verify` reports `self_contained` and lists anything unresolved or
outside the folder. **It is EXPECTED to report external dependencies today** —
that is the collect step being off, not a fault, and `report()` labels it as
such.
