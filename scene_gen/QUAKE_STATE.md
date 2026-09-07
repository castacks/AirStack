# quake pipeline — state

Earthquake mesh damage, as it stands 2026-08-25 on `disaster-dataset`.
`mesh_damage` is the API, `quake.py` the earthquake script, `archetypes/` the
Stage A library. Numbers here were measured, not estimated; where something is
unverified it says so.

## The pipeline, and the invariant it exists to keep

Every fragment must be a piece of something that was really there — a wall, a
floor, a column. That requires the intact asset to be finished before the
damage starts, so `mesh_damage.damage_building` runs in this order:

1. **subdivide** to `max_edge_m` (`SCENE_SUBDIVIDE_M` coarsens it per run).
2. **field** — `failure_field` over world space; below `release` everywhere the
   building is dismissed untouched and nothing downstream runs.
3. **measure the inside ONCE** (`interior_fill`, on the asset as it arrived).
4. **thicken** (`solidify`) — only a hollow shell. Skipped when the pack marks
   the asset `solid: true`, and when step 3 found an interior already.
5. **fill** (`fill_interior`) — floor slabs and a storey-tall column grid for
   the shells thickened in step 4. Skipped under `min_radius_m` (a house's
   shell IS the house) and for assets that came with their own interior.
6. **cut** (`fracture_to_stage`) — Voronoi cells from the field, every cut
   capped so fragments are closed solids.
7. **settle** — PhysX; the live path does it once per scene, Stage A per cell.

Steps 3-5 are one decision made once. Re-deriving it in step 5 was a bug
(fixed `c1f0e5b0`): `solidify` extends walls inward into the very plan cells
`interior_fill` erodes to, so the second measurement crossed `already_filled`
and the interior pass declined. Buildings reached the cutter hollow, and their
rubble came out as wall chunks sealed with cap faces — rock-shaped, no floors
in them, while the standing half kept thick walls around empty space.

## What a building is made of

Declared per asset in the pack, beside its scale:
`{usd: ".../BG_Building_F.usd", scale: 0.01, material: glass}`. Read by
`scene_generator.asset_materials`; `mesh_damage.STRUCTURE_OF` turns the FACADE
into the structure a break exposes — glass → steel, brick → masonry, concrete
and wood as themselves. That decides fragment shape (`grain`), wall thickness,
and the material on every cut face and authored floor.

All 28 urban building assets are tagged and match
`config/asset_packs/urban.buildings.materials.yaml` exactly. **The suburban
pack has 26 building assets with no assignment at all**; they fall back to the
locale material (timber), which is plausible and unverified.

Both paths read it. Stage A did not until `8f3a8f28` — it used the locale's
single material, so the whole library was cut as masonry however the pack
described it. Verified in the geometry: `MBuilding01_cracked` carries
`FractureCore_masonry` in the old library and `FractureCore_concrete` now.

## What it leaves on the ground

The rubble around a wreck is planned by `disaster/debris.py` and authored by
whoever did the damage — `mesh_damage.apply_to_stage` for a live cut,
`archetypes/bake.py` into the cell, where it settles with the fragments and is
merged into the archetype. `disaster_stage` only covers the buildings no cutter
reached.

Two things it is now that it was not: **matched** (a debris asset declares a
`material:` and `debris.SHEDS` says which structures may shed it, so a
brownstone drops brick and a concrete frame never does) and **measured** (the
amount is a volume, `shed_m3_per_m` times the perimeter times
`debris.fallen(report)` — the share of the building that actually came free,
counting what `consume` pulverised and not the orphaned slabs). Piece count
follows from piece size.

Every debris asset in `urban_nucleus` and `suburban_nucleus` was rendered and
labelled on 2026-08-25; the surveys are
`config/asset_packs/{urban,suburban}.debris.materials.yaml`. **Neither set has
one piece of steel or glazing debris** and the urban set has six steel and
three glass buildings, so those fall back to concrete rubble through `SHEDS`.
Bent corrugated sheet, snapped mullion and glazing are the gap to fill.

Baking debris in adds bodies to every cell's settle — the pieces are small and
land in under a second, but the convergence gate is `still_moving <= 5% of
bodies` and item 1 below is already about cells that never converge, so the
rejection rate wants re-measuring on the next bake.

Baking debris in also means **the library needs re-baking** for it, alongside
the `c1f0e5b0` re-bake below — an archetype baked before this has a clean lot
around it. Each cell records `debris_r_m` on its manifest entry, which is what
lets `targets` keep casualties out of rubble it can no longer see as
placements.

## The two paths, and what they cost

**Live** (`SCENE_ARCHETYPES=0`) cuts every building at load: 40-90 s each,
250-330 s for `MBuilding05` (311k points). Linear in damaged buildings, so a
200 m map with 14 wrecks is ~35 min and a 500 m map cannot use it.

**Baked** (Stage A) references pre-settled, merged, static wrecks: no fracture
at load. 500 m with 25-30 wrecks loads in ~5-7 min against ~90 s of Kit
startup that no scene avoids.

Bake cost, 86 cells at 250 cells/9 m: 2,522 s total, 34 s median per cell —
**settle is 53% of it**, fracture+export the rest. Fracture's median is 14 s
with a 90 s tail on the heavy assets; settle is superlinear in bodies in
contact, which is why big cells stop converging rather than merely slowing.

## Budgets

Memory is the binding constraint, and it is HOST RAM, not GPU. A 14 GB library
referenced by 67 buildings exhausted 125 GB and the kernel killed Kit (zero
Vulkan OOM lines; the GPU sat at 1.5 of 32 GB). The fracture inflates a
building ~14x — `MBuilding01_cracked` is 7.09M points against the source's
0.5M — and references were not instanced, so every placement was resident in
full.

**ARCHETYPES ARE INSTANCED NOW** (2026-08-26), which is the real answer to that
rather than a smaller library. The bake was never the memory problem — Stage A
has always built, settled, exported and UNLOADED one cell at a time, so it
holds one wreck at a time whatever the library ends up weighing. The cost was
all at load, and it was duplication: eighty buildings drawn from thirty
archetypes cost eighty copies. The blocker recorded here and in
`apply_placements` was that an instanceable prim has no traversable children,
so `scene_prep.add_colliders` could not reach inside to apply
`UsdPhysics.CollisionAPI` and the drone would fly through the building. That
blocker does not apply: `disaster/bake.export_object` already authors
`CollisionAPI` plus a triangle-mesh approximation on every merged mesh it
writes, and `add_colliders` skips what already has the API. Nothing else
reaches inside one either — mesh damage skips archetypes, the settle skips
them, the debris they shed is a sibling scope, and `mark_cut_geometry` reads
the survey flag. `disaster_stage` therefore sets `_instanceable` on the
archetype branch and `apply_placements` honours it per placement, so the
resident cost is the number of DISTINCT archetypes rather than the number of
buildings.

| knob | what it bounds |
|---|---|
| `SCENE_ARCHETYPE_BUDGET` | how many buildings get a baked wreck; the rest tilt and sink |
| `SCENE_CELL_BUDGET` | scene-wide cells, spent on the worst-hit buildings at full fragment size |
| `SCENE_MAX_CELLS` | per-building ceiling — a fragment size in disguise, prefer the budget |
| `SCENE_SUBDIVIDE_M` | pre-fracture tessellation. **Not** the dominant term in archetype file size — measured 2026-08-25 |
| `SCENE_DAMAGE_BUDGET` | live-fractured buildings |

GPU is bound separately by fragment COUNT: ~3,000 fragments load on the 500 m
map, ~5,400 exhaust 32 GB during the settle.

**INSTANCING VERIFIED AT CITY SCALE, 2026-08-30.** The claim above had never
been measured on a real library in a real scene. `urban_v3_quake` (1 km, seed
42, severity 0.9) placed **166 archetype references drawn from 16 distinct
types** on top of the 67k-prim city: 80,537 prims, 106,773 colliders, ~25 min
to ready, peak **58 GB resident / 68 GB of 125 GB host**, GPU never above
5.5 GB of 32.

Against the failure this replaced — a 14 GB library referenced by SIXTY-SEVEN
buildings taking all 125 GB and getting Kit OOM-killed — that is 2.5x the
references for a fraction of the memory, and it lands where the theory said:
the resident cost is the number of DISTINCT archetypes, not of placements. The
run also confirms the binding constraint is host RAM, not GPU: the GPU sat at
17% of its memory throughout.

## What actually makes an archetype big

Measured on `MBuilding01` (140k points) during the `urban_quake_500` bake:

| archetype | size |
|---|---|
| `_pristine` (the source asset re-exported) | 9.9 MB |
| `_cracked` at `SCENE_SUBDIVIDE_M` unset (4 m) | 283 MB |
| `_cracked` at `SCENE_SUBDIVIDE_M=9` | 289 MB |

**A 29x inflation, and the subdivision knob does not touch it.** The cracked
cell is 145 loose fragments plus 496 static ones — 641 pieces at ~450 KB each.
What costs that is what the pipeline adds before the cut: `solidify` doubles
the point count, `fill_interior` authors floor slabs and a storey-tall column
grid (5,174 boxes on one building), and `fracture_to_stage` closes every
fragment with a cap fan and carries the source UVs and material subsets onto
it. Subdivision only adds triangles where an edge is already longer than the
threshold, and on an asset whose edges are mostly under 4 m there is nothing
for a coarser threshold to remove.

So the levers for library SIZE are the cell count (`SCENE_MAX_CELLS`,
`SCENE_CELL_BUDGET`) and, at load time rather than bake time,
`SCENE_ARCHETYPE_BUDGET`. Note that `SCENE_SUBDIVIDE_M` reached only the LIVE
cut until `mesh_damage.subdivide_edge_m` was factored out — a bake with it set
produced a byte-identical library, which is how the table above came to be
measured in the first place.

## What the library records about itself

An archetype is a COOKED artifact: it carries no trace of the code that cut it,
so a wreck baked before a fix and one baked after are the same shape on disk
and behave differently in a scene. Two of those are already in this file —
everything before `c1f0e5b0` was cut hollow, everything before the debris pass
stands on a clean lot — and both were caught by somebody REMEMBERING that a
re-bake was owed. That is not a mechanism, so now every record carries:

| field | what it answers |
|---|---|
| `baked_at` | when this rung was cut (ISO, with an offset) |
| `pipeline_version` | which declared pipeline — `archetypes/version.py`, `CHANGES` says why each exists |
| `pipeline_fingerprint` | a hash over the 20 sources that decide geometry |
| `preview` | `{obl, top}`, PNGs of this rung, relative to the manifest |
| `used_by` | `{config: count}` — which scenes place this type, from the census |

The version and the fingerprint are both there because they fail differently.
The version is DECLARED, so it is the field worth reading and the one that says
whether a re-bake is owed; it also does not change unless somebody acts, which
is the failure this whole section exists to fix. The fingerprint is MEASURED, so
a stale library whose author forgot the bump still reads as stale — and it moves
on a comment edit, so it cannot be the thing a human reasons about. Neither
alone is enough. `archetypes/version.py <manifest>` splits a library into
current and stale and says which pipeline each stale record came from;
`tools/preview_sheet.py` marks the same thing per tile.

`library.py` is deliberately NOT fingerprinted. It is the read side — naming,
manifest layout — and re-organising it must not invalidate 39 GB of geometry.

### Previews, taken during the bake

`archetypes/preview.py` photographs each cell between its export and its
unload, which is the only window: the export is what decides the cell is worth
keeping, and the unload is what takes the geometry off the stage. Two frames —
`_obl` for whether it reads as a damaged building, `_top` for what it left on
the ground, which is the debris ring a drone actually flies over. Framed off
the measured bbox, never a constant: the library runs from a 12 m shed to a
96 m tower.

The rest of the settle batch is HIDDEN for the duration. Cells are `GRID_M`
apart so they cannot touch, but a camera framed to fit a 40 m building stands
~120 m back, which puts the neighbours well inside the frame.

On by default (`SCENE_ARCH_PREVIEW=0` to stop it). An opt-in review is one
nobody runs, and this is the only look at a library that stays cheap once the
library is too big to open. `tools/render_archetypes.py` is the other half —
Cycles, off the exported USDs, properly lit — and is what to use on a finished
library; this is what to use while a bake is still running.

### The census — which types a scene actually places

`archetypes/census.py`, written by a scene run under Kit (`SCENE_CENSUS=<path>`
on `scene_launch_script.py`). The usual reason to take it there is that packing
keys off MEASURED footprints and a plain `python3` cannot open a Nucleus asset,
so it falls back to `fallback_sizes` and names different buildings — the
warning `plan.used_by_scene` carries, measured at 5 host types against 2 under
Kit on `urban_quake_tiny`.

**That warning is about a COLD CACHE, and it is worth knowing when it does not
apply.** `measure_cache.py` persists every measurement to
`assets/.measurements.json` — 654 entries for `urban_v3`, 327 of them Nucleus —
and the host reads the same file Kit wrote, so with it warm the two agree
exactly. Checked 2026-08-30 rather than assumed: a host knob sweep and the Kit
census of the chosen row both came back 72/86 models, top 6.3%, twin-within-50 m
6.0%, closest 13.4 m, 430 buildings. So layout knobs can be swept on the host in
minutes and confirmed under Kit once, instead of a 25-minute Kit run per row.

`bake_cli --census <file>` then does two things: marks `used_by` on every
record it recognises, and BAKES THE USED TYPES FIRST, most-placed first, so a
bake stopped by the disk or by the morning has finished the archetypes a scene
actually references. `--census-only` narrows to just those.

## What the first real v5 bake found

Three bugs, all caught by RUNNING the pipeline rather than reading it, and each
invisible to the tests that existed. Recorded because each has a shape worth
recognising again.

**1. `disaster/bake._portable_asset` had no `from pxr import Sdf`.** That module
carries no module-level `pxr` import on purpose — it is read on the host, where
Kit's USD build is not importable until Kit starts — so every function imports
what it needs itself, and this one did not. All three of its `Sdf.AssetPath`
calls raised `NameError`, and **every damaged rung failed to export while
`pristine` came out fine**: the three call sites are only reached by a fracture
material, case 1 being the bare `OmniPBR.mdl` that every `FractureCore_*`
carries. A full bake would have produced 94 pristine shells and 382 failures.
Now covered by a test that walks the module's AST and asserts no top-level
function uses a `pxr` name it did not import (nested defs inherit the parent's,
which is why the check is scope-aware).

**2. One sunk fragment was throwing away whole archetypes.** The gate held
`through_floor` to zero ABSOLUTELY. Measured over the first 31 structure cells:
**4 rejected, 13%, every one of them `1 still moving, 1 through the floor`
against 144-455 bodies** — 0.2-0.7% of the wreck costing the archetype, which
Stage B then replaces with a rung further down, so a partial collapse renders as
merely cracked. It is one piece tunnelling through the ground early (`SETTLE_DT`
of 1/20 is the conservative end of where that starts) and then free-falling for
the whole budget, which is why it is the still-moving body too.
`Baker._drop_sunk` now deletes those pieces before the gate judges — a fragment
below the floor is invisible and costs only bytes. **Rejections went 13% -> 0%**
over the first 39 cells of the re-run, with 5 archetypes saved that the previous
run had discarded. What the gate is really for still trips it: drop most of a
wreck and the cell is empty, which `export_object` reports.

**3. `ensure_deps` installs a backend the process can never see.**
`trimesh.creation._engines` is a module-level list built at import time, so a
wheel pip-installed afterwards is invisible for the life of the process:
`triangulate_polygon` kept raising "No available triangulation engine!" while a
FRESH interpreter reported `[('earcut', True), ('manifold', True)]` from the
same site-packages. `vtk_fracture._refresh_trimesh_engines` reloads the
submodule after installing. It only ever hit tree slicing, so it is a fire and
tornado bug, not an earthquake one.

### Trees are not an earthquake effect

The earthquake `VEGETATION` ladder was `pristine` + `fallen` (with a `stump`
variant) on the reasoning that `compile_earthquake` topples a few — but that
fraction was `lerp(0.0, 0.1, sev)`, nothing at all below the top of the range,
and shaking is not what fells a tree. Ladder is now `pristine` only and
`trees_toppled_fraction` is a flat `0.0`, which takes the plan from 488
archetypes to 476. Fire (burns them) and tornado (breaks them) are untouched.

### What a bake actually costs

Measured on the v5 run, `urban_v3` / earthquake / 476 archetypes:

| | |
|---|---|
| wall clock | **50 s per cell** |
| the trace's own `seconds` | 17 s median — build + amortised settle share + export ONLY |
| structure range | 2 s to 247 s; the tail is the 300k-point assets |
| library size | ~61 MB per archetype, ~29 GB projected |

**Do not price a bake off the trace's `seconds`.** It excludes the batch settle
wall time, the debris pass and Kit overhead, and it reads three times faster
than the clock — which is how a 6-hour job got quoted as 2-3 hours twice.

### Degenerate slabs are non-collidable — and predate the VTK port

PhysX complains during the bake, in three flavours:

    ConvexMeshCookingTask: adjusted the thickness of a very thin or very small mesh
    rigid body .../slab_008 ... invalid inertia tensor ... and a NEGATIVE MASS
    Provided mesh geom with a PhysicsCollisionAPI does not have INDICES,
        collision will not be created

Counted over the `urban_v3` bake: 88 thin-mesh adjustments, 30 negative masses,
34 missing-index colliders across ~180 archetypes. **That last one matters** —
a piece with no collider is one a drone flies straight through, and it fails
silently.

ATTRIBUTED, NOT ASSUMED: the assets named in those warnings span BOTH clippers
— `SM_MERGED_BP_MBuilding01`, `SM_Building_24`, `federal_bureau`, `block_09`
are numpy/v5; `Building_TypeA_C`, `block_03`, `mini_auto_service`,
`Building_TypeD_B` are vtk/v6. So this is a property of the fracture pipeline,
not of the backend, and it is present in the protected v5 library too.

The offenders are all `slab_*` prims — the orphaned-component path, where
`unsupported` releases a whole retained piece rather than a cut fragment. Those
are selected, not clipped, so they never pass through `_clip_by_plane` and get
their rim closed by `_cap_shell_rim` instead; a selection that comes out empty
or zero-thickness is what reaches PhysX as a negative mass.

## Open

1. ~~**15-17 of 86 cells never converge**~~ — **LARGELY CLOSED 2026-08-30 by
   `Baker._drop_sunk`** (see "What the first real v5 bake found"). The
   rejections measured on this pack were not piles that failed to settle: they
   were single tunnelled fragments tripping an absolute `through_floor == 0`.
   Dropping those took rejections from 13% to 0% over 39 cells.

   The paragraphs below still stand for the OTHER failure mode — a genuinely
   unsettleable pile, which is what the low-poly DownTown assets produce — and
   that one is unfixed. It now shows up as WEAK DAMAGE rather than as a
   rejection: `BG_Building_D_pancaked` is a near-intact six-storey building
   with rubble at its base, while `abandoned_brick_building_pancaked` is a
   proper heap. Same root cause the note below names — `cells_for` sizes the
   cut from `fragment_m` and the building's EXTENT and takes no account of how
   much surface there is to cut. 16 placements in the 1 km layout, so it is
   worth fixing.

   The original note, for the mechanism:

   MEASURED AGAIN 2026-08-26 on `urban_quake_500` (the `urban_intact` pack, not
   the tiny config above): **7 of the first 20 cells rejected, 35%**, against
   the ~19% here. The worst are the low-poly DownTown assets — `BG_Building_A`
   is 3.5k points at source and is being cut into 1,300-1,600 cells, of which
   1,073-1,339 are still moving at the step ceiling. That is 80% of the pile,
   not a few creeping bodies: cutting a nearly-empty shell that finely makes
   paper-thin fragments with no mass to settle. `cells_for` sizes the cut from
   `fragment_m` and the building's EXTENT, and takes no account of how much
   surface there is to cut — which is the missing term.

   NOT THE DEBRIS. Checked, because adding rigid bodies to every cell was the
   obvious suspect: debris tops out at 119 pieces a cell against 1,339 still
   moving, so it cannot account for more than 9% of the worst failures and the
   rejected cells fail on their fragments alone. They burn the
   full 1,800-step budget first — about a fifth of bake time settling piles
   that are then thrown away. The gate is right to reject them: the pieces are
   moving at 9.3 m/s (p95 3.6) after 30 s of sim, which is debris in flight,
   not a pile creeping. Deleting the movers is not the answer either — in the
   worst cells they are 86-95% of the wreck. Wants an early bail-out, and a
   physical fix.
2. **A partial collapse leaves its upper block unsupported** now that interiors
   exist: the support graph decides what comes free, and it was calibrated
   before there was structure inside to hold anything up.
3. **`MBuilding05` measures 0.33 against a 0.35 threshold** but is recorded at
   0.667 with real floor plates, so it is treated as hollow and given a second
   set of floors through its own. The gap the threshold sat in has closed.
4. ~~**The library needs re-baking**~~ — **IN PROGRESS 2026-08-30**, as pipeline
   v5 (`archetypes/version.py`), and this is the last time that fact has to be
   remembered rather than read: every record now carries `baked_at`,
   `pipeline_version` and `pipeline_fingerprint`, and
   `archetypes/version.py <manifest>` splits a library into current and stale.
   Run against the old 5.4 GB library it reported **0 current, 69 stale, all
   with no provenance at all**, which is the correct reading of a pre-module
   bake. The memory budget still needs re-measuring once the new library is
   complete — 5,174 authored boxes on one building is a lot of new geometry.
5. ~~`tests/test_findability.py::test_the_gate_is_not_a_rubber_stamp` depends
   on the baked library existing on disk~~ — it now SKIPS when nothing is
   `cut`, which is the honest reading: without a library no building is cut, so
   there is no collapsed footprint to bury anyone in. It fired for real during
   a re-bake and read as a code regression.
