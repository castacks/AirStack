---
name: generate-urban-city
description: Build a road-networked downtown with scene_gen's real city generator — the preset/asset-set config stack, restricting the building library to chosen assets, OSM-calibrated block and road-hierarchy knobs, districts and height sampling, city_detail street furniture, and the traps that silently flatten the grid. Use whenever a scene needs streets, blocks and frontages rather than buildings placed by hand.
---

# Generating an urban city

There is a full city generator in this repo. It does road hierarchy, OSM-calibrated
block sizes, district zoning, height distributions, sidewalk furniture and road
markings. **Do not write a layout packer — configure this one.**

Writing a parallel placer is the mistake this skill exists to prevent. It has been
made twice: once for `urban_fire_city250`, once for `undamaged_city250`. Both
reinvented block packing badly, and both hit bugs (`prim.Load()` for nested
payloads, typeless defs, `raw_pivot`) that `apply_placements` had already solved
and documented.

## Running one

```bash
SCENE_CONFIG=downtown \
ISAAC_SIM_SCRIPT_NAME=scene_launch_script.py \
airstack up isaac-sim
```

`scene_launch_script.py` is the ONLY launcher that runs the v2 passes
(`city_detail`, `districts`, `road_markings`, `layout/city_layout`). Loading a
downtown preset through any other launcher is not an error — it just silently
gives you a city with no street furniture, because the preset zeroes the built-in
furniture passes so `city_detail` can own them.

The script's docstring carries a Script-Editor snippet that regenerates the scene
in place after a YAML edit, without restarting Kit. Use it — a restart is ~2 min.

## The config stack

```
presets/downtown.yaml          what THIS scene is: region, seed, disaster, overrides
  └── asset-set: urban         adds Objaverse + AEC content (needs pre-flight)
        └── urban_nucleus      Nucleus-only buildings + debris; works offline
              └── shared       furniture, greenery, tiles, vehicles, people
```

- `extends:` chains asset sets. `<key>+` **appends** to the inherited pool; a bare
  key **replaces** it.
- Any USD entry is either a bare path or a dict:
  `{usd: "...", scale: 0.01, axis-up: "Y", yaw-offset: 90, tags: ["park"]}`.
  Paths are relative to `asset_root`.
- `asset-set: urban` pulls Objaverse assets and needs a HOST pre-flight
  (`prepare_assets.py`, `UV_PYTHON=3.13` is load-bearing — `bpy` only ships a
  cp313 wheel). Uncached assets render as placeholder prisms. **Point at
  `urban_nucleus` to skip all of that.**

## Restricting the building library

This is the whole answer to "use only these buildings". The pool is
`usds.buildings.intact` (plus optional `damaged` / `destroyed`). Write an asset
set that REPLACES it — a bare key, not `intact+`:

```yaml
# config/asset_sets/urban_gac.yaml
extends: urban_nucleus
usds:
  buildings:
    intact:
      - usd: "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/SM_Building_01.usd"
        scale: 0.01          # this pack is centimetre-authored; USD does not convert
      - usd: ".../SM_Building_02.usd"
        scale: 0.01
```

then a preset with `asset-set: urban_gac`. Nothing else needs touching — roads,
blocks, districts and furniture are unchanged.

A disaster pass can need the same override for a different reason:
`downtown_fire_500.yaml` overrides `usds.buildings.lowrise`/`midrise_v2` to
drop every asset `kit_substitute.route()` cannot burn — `districts.
_BurnabilityGuard` can swap at most one unburnable offender per block, so a
pool that is mostly unburnable to begin with (the shared `urban_gac.yaml`
pools mix burnable kit archetypes with permanently-unburnable shopfronts and
factory sheds) starves the fire spread regardless. See `build-urban-fire-scenes`
for the burnability table and the guard.

**Check the footprints against the block targets before you run.** A block's short
axis must exceed the largest building footprint or nothing tall can be placed at
all; the shipped comment records exactly this, at `block_short_m: [36,46]` against
a 51.3 m `BG_Building_A`.

## Layout knobs (`overrides.layout`)

| key | what it does |
|---|---|
| `max_block_m` | hard cap. **Inherited default 70 is below every measured block short side and silently caps the per-typology targets.** Downtown raises it to 200. |
| `anisotropic.long_axis` | `x` makes blocks run long east–west |
| `anisotropic.block_short_m` / `block_long_m` | street-to-street and avenue-to-avenue targets. Only a FALLBACK while `districts.enabled` — zoning supplies the real ones per typology |
| `main_road_chance_long` / `_short` | arterial density. Splitting the long axis is what creates an avenue |
| `stop_chance` | 1 − chance of a further split once in range |
| `parking_lane_m`, `parking_policy` | kerb strips; policy is drawn once per corridor and held its whole length |
| `zones.<ring>` | per-district-ring overrides of all of the above. **Keys must match `districts.rings` names.** |

Calibration already done, from the preset's own measurements: Manhattan runs
~80 m short side, ~274 m long, 35 blocks/km². The built-in BSP at 1.2× gives
38 × 50 m and 350 blocks/km² — three to ten times too fine, and that is what makes
a grid read as generated.

## Region size is a layout parameter, not just extent

Measured in the preset: **400 m → 14 blocks, 3 arterial corridors, no readable
hierarchy. 800 m → 31 blocks, 34 corridors, a real core/mid/edge gradient.** A
274 m block does not fit twice into 400 m. If the city must be small, shrink the
block targets too or accept superblocks.

## Districts and heights (`overrides.districts`)

Heights are **sampled from a per-ring log-normal, then the nearest fitting library
asset wins** — the library quantises a smooth distribution rather than being it, so
adding one asset improves the whole scene. Cutting the library into per-ring bands
is what previously made neighbours jump between two extremes.

- `neighbour_weight` / `neighbour_radius_m` — pull toward nearby placed heights so
  a street steps gradually
- `pick_sigma` — log-height tolerance when matching
- `repeat_penalty` — damps an asset by `(1 + times placed) ** p`. Without it two
  small-footprint models took **66%** of every slot, because an asset is eligible
  whenever it fits
- `infill.enabled` / `gap_m` — fills block interiors; raise `min_gap_m` for
  courtyards or to cap the point budget

## Where placements actually get written

`build_city(config, resolver) -> (placements, layout)`; `apply_placements` writes
them. Read that function's docstring before doing anything custom — it carries
hard-won handling:

- a **typeless def** takes the reference, so an asset whose root prim is a Mesh
  keeps its type instead of being overridden into an Xform that composes
  attributes and draws nothing;
- **`prim.Load()` per prim**, because a prim composed into a running stage does not
  auto-load nested payloads the way `Usd.Stage.Open` does — the reference lands
  with a correct bbox and transform and **no visible geometry**;
- **`raw_pivot`** opts a self-assembling kit out of the bbox-centroid correction,
  which would otherwise pull the pieces apart;
- **not instanced by default.** Pass `instance_categories` to opt a category in.
  Instancing is opt-in because `prune_prims` deactivates sub-prims inside placed
  assets and USD forbids editing inside an instance — any category that pass
  touches must stay un-instanced. A preset can opt in wholesale with
  `instance_placements: true` (added 2026-08-31 for the 500 m fire city's
  composition OOM — 66,590 placements of 87 unique USDs cost ~39 GB RSS
  un-instanced and OOM-killed Kit twice); `generate_scene.py` then instances
  every placement category not excluded by a `prune_prims` rule. If the preset
  key doesn't survive whichever config-loading path a launcher uses,
  `SG_INSTANCE_PLACEMENTS=1` works unconditionally as an env override.

## Traps

- **Instancing a Mesh-rooted asset used to render nothing — fixed 2026-08-31.**
  When a referenced asset's ROOT prim is itself a Mesh, `SetInstanceable(True)`
  puts everything the reference composes (materials, GeomSubsets, the mesh
  itself) into the shared prototype while the drawn point stays outside it —
  Hydra draws the prototype and the placement disappears; the identical root
  cause on an asset whose per-face materials live as children of that same root
  mesh instead reads as flat grey (a lit post or bench with "no texture").
  Measured on the 500 m fire city: 14 of 110 distinct assets are gprim-rooted
  and accounted for 161 of 1,469 placements (every streetlight and 50 of 62
  benches among them). `apply_placements` now refuses to `SetInstanceable` any
  placement whose composed prim `IsA(UsdGeom.Gprim)` and prints which asset it
  skipped instead. Before trusting a newly-instanced pool, audit it with
  `scene_gen/tools/fc_instance_material_probe.py` — it composes every asset
  twice (plain vs. instanceable) and diffs the computed bound material of
  every mesh, so the difference is measured, not inferred from the asset's
  shape. Cost a long debug on the CitySample kit before this landed.
- **`SizeResolver` falls back to a flat 30 x 20 x 24 m box for anything not
  locally mirrored.** GAC/downtowncity are covered by checked-in caches
  (`_plans/gac_buildings.json` / `dtc_buildings.json`); Muyang
  (`BG_Building_*`, `SM_MERGED_BP_MBuilding*`), Dmytro (`Building_Type*`) and
  `standalone/buildings/...` are not, and measure as `fallback_sizes.house` on
  a host-side (no-Nucleus) build — `stepped_tower.usdc` is REALLY 65 x 78 x
  81 m, an ~8x footprint-area error that changes which buildings a layout
  even considers viable neighbours. `scene_gen/tools/measure_standalone_via_nucleus.py`
  (bare `usd_python.sh`, no Kit) plus `scene_gen/tools/seed_standalone_cache.py`'s
  checked-in `config/harvested/standalone_buildings.json` fix the SIZE.
  They cannot make a host-side PACK match what Kit actually built — a bigger
  real footprint can tip two buildings from clear to touching in the packer's
  own spacing decision — so treat a host-side layout as a size source, never
  as the layout itself: the only ground truth for what a scene contains is a
  Kit dump (`FC_INTACT_ONLY=1 FC_DUMP=<path>` on the city launcher, ~45 s
  headless), and any offline tool solving on top of a layout should load that
  dump rather than rebuild the layout host-side.
- **`docker logs isaac-sim` is empty** — read the tmux pane. See
  `run-isaac-sim-launcher`.
- **Verify what is ON STAGE, not what was authored.** A builder returns the height
  it computed, which is true whether or not geometry arrived. Measure the bbox
  *excluding* anything authored inline (a roof slab sits exactly at the computed
  height and makes the check pass for free).
- **Blank elevations.** Kit buildings are modelled only where the artist expected
  them to be seen. `tools/gac_faces.py` bins each building's triangles by
  elevation and reports density per side; on GreatAmericanCity most carry detail on
  ONE face (`SM_Building_04`: 77.8 tri/m² on E, 0.14 on W). Judge blankness
  RELATIVE to that building's own best side — a fixed threshold misses the ornate
  ones. Such buildings need their good face to the street and their blank flanks
  against neighbours.

## Files

| file | role |
|---|---|
| `scene_gen/scene_generator.py` | `build_city`, `_pack_block_with_buildings`, `apply_placements` |
| `scene_gen/layout/city_layout.py` | block subdivision, road corridors |
| `scene_gen/detail/districts.py` | zoning rings, height sampling |
| `scene_gen/detail/city_detail.py` | NACTO sidewalk zones, frontage/corner slots, signals |
| `scene_gen/detail/road_markings.py` | crosswalks, stop bars |
| `scene_gen/config/presets/downtown.yaml` | the worked, heavily-commented example |
| `scene_gen/tools/gac_faces.py` | per-elevation detail density, for placement rules |
