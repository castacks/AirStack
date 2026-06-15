# Procedural City Generator

Builds a grid-based neighborhood in Isaac Sim by referencing a library of USD
props (tiles, houses, trees, streetlights) instead of hand-placing them in a 3D
editor. Generation is deterministic (seeded), so a config + seed always
reproduces the same city.

## Relevant files

| File | Role |
|------|------|
| [`utils/scene_generator.py`](utils/scene_generator.py) | Core module: footprint measurement, city layout, USD composition. Also runs as an offline CLI. |
| [`config/scene_generator_config.yaml`](config/scene_generator_config.yaml) | City spec — grid size, asset library, damage fraction, tree/streetlight density, exclusions, seed. |
| [`launch_scripts/generated_scene_launch_script.py`](launch_scripts/generated_scene_launch_script.py) | Single-drone launcher that builds the city at runtime, then preps it and spawns the drone. |
| [`utils/scene_prep.py`](utils/scene_prep.py) | Existing helpers (`scale_stage_prim`, `add_colliders`, `add_dome_light`, `get_stage_meters_per_unit`) the launch script reuses. |

## How it works

The city is an **N×M grid of cells**, one house per cell, with roads running
between cells. `build_city(config, resolver)` lays it out in three steps:

1. **Ground** — a road lattice (intersection tiles at crossings: four-way
   interior, tee on edges, dead-end at corners; straight tiles between) plus a
   grass carpet per cell, a concrete driveway from each house to its road, and
   an optional sidewalk ring.
2. **Houses** — one house per cell, drawn from the library; a configurable
   `damaged_fraction` is rendered damaged (swapped for a damaged USD if you
   provide one, otherwise faked by tilting + sinking an intact house).
3. **Detail** — trees scattered on open grass (avoiding house/driveway/exclusion
   footprints) and streetlights at uniform spacing along the roads.

**Footprints are measured, not hard-coded.** `SizeResolver` opens each USD and
reads its bounding box (`UsdGeom.BBoxCache`) so tiles and houses lay out at their
true size and sit on the ground (via each asset's base-Z offset). If a USD can't
be opened (e.g. offline with no Nucleus access), it falls back to per-category
`fallback_sizes` from the config. Set `measure_usds: false` to always use the
fallbacks.

Layout (pure Python) is separate from USD writing: `apply_placements` defines an
`Xform` per placement referencing the asset USD, with translate / rotateXYZ
(roll, pitch, yaw — the tilt makes damaged houses possible) / scale ops, marked
`instanceable` so the hundreds of repeated tiles share geometry. Two entrypoints
wrap it: `generate_scene_usd(config, output)` (offline bake) and
`generate_scene_on_stage(stage, config, ...)` (runtime compose).

All coordinates are **metric (meters)** in the world frame (+X East, +Y North,
+Z Up); they're multiplied by `scene_scale_factor` (= `1 / meters_per_unit`) at
write time to land in stage units.

## Usage

### Option A — generate at runtime (interactive)

Point the Isaac Sim launcher at the generated-scene script. It loads a base
(flat/terrain-only) environment, scatters the configured assets, adds colliders
+ a dome light, then spawns the drone:

```bash
# Inside the isaac-sim container, after `airstack up isaac-sim`:
docker exec airstack-isaac-sim-1 bash -c \
  "cd /path/to/simulation/isaac-sim && \
   python launch_scripts/generated_scene_launch_script.py"
```

Or set `ISAAC_SIM_SCRIPT_NAME=generated_scene_launch_script.py` in `.env` to use
it as the default scene script.

Edit the constants at the top of the launch script to taste:

| Constant | Meaning |
|----------|---------|
| `ENV_URL` | Base environment to scatter onto (default: flat grid). |
| `STAGE_SCALE` | Scale on `/World/stage` — `0.01` for cm assets, `1.0` for metric. |
| `SCENE_CONFIG` | Path to the YAML spec. |
| `SNAP_TO_GROUND` | Raycast each asset onto terrain colliders for its Z (needs physics stepped). |

### Option B — pre-bake a reusable USD (batch / repeatable)

Bake the scene once to a `.usd` and load it like any other environment. Good
for CI and for sharing a fixed scene:

```bash
docker exec airstack-isaac-sim-1 bash -c \
  "cd /path/to/simulation/isaac-sim && \
   python utils/scene_generator.py \
     --config config/scene_generator_config.yaml \
     --output assets/scenes/generated_demo.usd \
     --scale-factor 1.0"
```

Then set `ENV_URL` (in any launch script) to
`file://.../assets/scenes/generated_demo.usd`.

## Config reference

See [`config/scene_generator_config.yaml`](config/scene_generator_config.yaml)
for a worked example. Top-level keys:

| Key | Meaning |
|-----|---------|
| `seed` | Reproducibility — same seed produces the identical city. Bump for a new variant. |
| `grid` | `cols`, `rows` (cells = houses), `lot_margin_m` (gap from house to lot/road edge). |
| `asset_scale` | Geometry scale applied to every prop (e.g. `0.01` for cm-authored USDs in a meters stage). |
| `measure_usds` | `true` to measure each USD's footprint from its bbox at generation time; `false` to always use `fallback_sizes`. |
| `damaged_fraction` | Fraction of houses rendered damaged (0–1). |
| `trees` | `per_cell: [min, max]` count on open grass, `min_separation_m`, `house_margin_m`. |
| `streetlights` | `spacing_m` along roads, `setback_m` off the road edge. |
| `orientation` | Yaw offsets (deg) correcting authored facing: `road_straight`, `house_front`. |
| `fallback_sizes` | Per-category `[x, y]` (or `[x, y, z]`) footprints (m) used when measurement is off/fails. Keys: `grass`, `road`, `concrete`, `sidewalk`, `house`, `tree`, `streetlight`. |
| `usds` | Asset library (below). |
| `exclusions` | Regions kept clear of **clutter** — houses/trees/streetlights skip them; ground + roads still cover them. Circle `{center:[x,y], radius:r}` or rect `{bounds:[[xmin,ymin],[xmax,ymax]]}`. |

The **`usds`** library:

| Key | Meaning |
|-----|---------|
| `tiles.grass` | List of grass tile USDs (one chosen at random per grass cell). |
| `tiles.concrete` | List of concrete/driveway tile USDs (first used for driveways). |
| `tiles.sidewalk` | List of sidewalk tile USDs; empty → sidewalk ring skipped. |
| `tiles.road` | Map of `four_way` / `tee` / `straight` / `dead_end` road tile USDs. `straight` is required. |
| `houses.intact` | List of intact house USDs (one per cell, random). |
| `houses.damaged` | Optional list of damaged house USDs; empty → damaged cells fake it via tilt + sink. |
| `trees` | List of tree USDs scattered on open grass. |
| `streetlights` | List of streetlight USDs placed along the roads. |

> **Note:** The example config points at the `RetroNeighborhood` library on the
> AirLab Nucleus server. Swap in your own USD paths as needed. Footprint
> measurement requires the URLs to be openable; if you bake offline without
> Nucleus access, set `measure_usds: false` and tune `fallback_sizes`.

> **Orientation caveat:** road intersection/straight, house-front, and driveway
> facings assume a default authored orientation. If a category points the wrong
> way after a first look, adjust the `orientation` yaw offsets.

## Iterating without restarting Isaac Sim

`SimulationApp` startup (loading Kit + extensions) is the slow part — the scene
generation itself just writes prims under `/World/stage/generated`. So while the sim
is running you can tweak the YAML and regenerate **in place**, no container or
stack restart needed. Open the Kit **Script Editor** (Window → Script Editor)
and run:

```python
import os, sys
import omni.usd, omni.timeline

ISAAC_SIM_DIR = "AirStack/simulation/isaac-sim/"   # adjust to your checkout
sys.path.insert(0, os.path.join(ISAAC_SIM_DIR, "utils"))

import importlib, scene_generator, scene_prep
importlib.reload(scene_generator)                  # pick up code edits too

stage = omni.usd.get_context().get_stage()
omni.timeline.get_timeline_interface().stop()      # don't edit geometry mid-step

_, ssf = scene_prep.get_stage_meters_per_unit(stage)
scene_generator.reload_scene_on_stage(
    stage,
    os.path.join(ISAAC_SIM_DIR, "config", "scene_generator_config.yaml"),
    scene_scale_factor=ssf,
    add_colliders_fn=scene_prep.add_colliders,     # so physics sees new geometry
)
omni.timeline.get_timeline_interface().play()
```

`reload_scene_on_stage` clears the old `/World/stage/generated` subtree and re-reads
the YAML from disk, so edits to the grid / asset library / damage fraction take
effect immediately. Bump `seed` for a fresh layout, or edit anything else and
re-run the cell.

Connect to the running Kit instance via the GUI/livestream window to reach the
Script Editor.

## Extending

- **New prop categories / layout steps:** the three steps live in `build_city`
  in [`utils/scene_generator.py`](utils/scene_generator.py); each appends to the
  shared `placements` list via the local `add(...)` helper. Add a step or
  category there, plus a `fallback_sizes` entry.
- **Damaged houses:** drop real damaged USDs into `usds.houses.damaged` and they
  replace the tilt-and-sink fake automatically.
- **Orientation:** if roads/houses face the wrong way, tune the `orientation`
  yaw offsets — the intersection/tee/dead-end yaw logic is in `build_city`.
- **Terrain-following:** set `SNAP_TO_GROUND = True` in the launch script. It
  uses `_make_physx_ground_snap`, which raycasts down onto terrain colliders;
  requires colliders applied and the physics scene stepped first.
