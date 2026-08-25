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
0.5M — and references are not instanced, so every placement is resident in
full.

| knob | what it bounds |
|---|---|
| `SCENE_ARCHETYPE_BUDGET` | how many buildings get a baked wreck; the rest tilt and sink |
| `SCENE_CELL_BUDGET` | scene-wide cells, spent on the worst-hit buildings at full fragment size |
| `SCENE_MAX_CELLS` | per-building ceiling — a fragment size in disguise, prefer the budget |
| `SCENE_SUBDIVIDE_M` | pre-fracture tessellation, the dominant term in archetype file size |
| `SCENE_DAMAGE_BUDGET` | live-fractured buildings |

GPU is bound separately by fragment COUNT: ~3,000 fragments load on the 500 m
map, ~5,400 exhaust 32 GB during the settle.

## Open

1. **15-17 of 86 cells never converge** and are rejected, so those levels fall
   back down the ladder and the tower renders as merely cracked. They burn the
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
4. **The library needs re-baking** for `c1f0e5b0`: every archetype in it was
   cut hollow. 5,174 authored boxes on one building is a lot of new geometry,
   so the memory budget needs re-measuring after.
5. `tests/test_findability.py::test_the_gate_is_not_a_rubber_stamp` depends on
   the baked library existing on disk; it should build a fixture or skip.
