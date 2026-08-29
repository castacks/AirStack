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
  touches must stay un-instanced.

## Traps

- **Instancing a Mesh-rooted asset renders nothing.** The prims stay typed, loaded
  and correctly bounded, so every check passes and the viewport is empty. Cost a
  long debug on the CitySample kit.
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
