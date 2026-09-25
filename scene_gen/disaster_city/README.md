# Disaster City (TEEX) for Isaac Sim, rebuilt from real data

A recreation of TEEX Disaster City, College Station TX, built from Google 3D
Tiles (internal use only), OpenStreetMap, and two drone videos. The output is
one USD stage, `data/recon/disaster_city.usda`: Z-up, metres, in the world
frame of `data/blender_data/disaster_city.blend` (Blosm origin lat 30.57891,
lon -96.35235). Every part carries a `semantics:labels:class`: ground, road,
water, vegetation, rubble, building, vehicle, or other.

This is not the procedural site plan in `config/site_plans/disaster_city.yaml`
(`layout/site_plan.py`). That traces the site into the generator's own layout
and dresses it with library assets. This pipeline reconstructs the real place,
and the two share only the feature IDs.

`LABELS.yaml` names every feature (B01, R01, S01, and so on); use those IDs
when talking about the site. `label_map.py` draws them on a map
(`data/LABELS_map.png`) and into the blend. `INVENTORY.yaml` describes the
source images and videos.

## Code here, data elsewhere

The data is tens of GB: the blend, the drone videos, the COLMAP work dirs and
the built USDs. It lives outside git, in `$DISASTER_CITY_DATA` or, if that is
unset, in `data/` here (gitignored; make it a symlink). See `_paths.py`.

```
data/
  blender_data/disaster_city.blend      Blosm import: Google 3D Tiles + OSM layers
  mocaps/raw/{A,B}.mp4, mocaps/clips/   drone videos; clips.tsv indexes the named cuts
  recon/                                everything built; package.py ships this tree
    tiles_*.npz, osm.npz, ortho_site.*  inputs cut from the blend
    rubble_west/, b01/                  COLMAP work dirs (images, sparse, dense/fused.ply, to_world.json)
    b01/ s01/ b03/ b06/ pad/            hero USDs (+ generated specs)
    site_ground.usd, buildings_lod1.usd, trees.usd, vehicles.usd, debris.usd
    assets/                             local copies of the tree/car/debris assets used
    disaster_city.usda                  the assembled scene
  dist/disaster_city/                   package.py output (about 420 MB, self-contained)
```

Environments: `~/.venvs/recon` (Python 3.12: `pycolmap-cuda12 open3d usd-core
opencv-python-headless numpy trimesh pyyaml imageio-ffmpeg`), `blender` 5.x on
PATH, and `~/isaacsim/python.sh` (Isaac Sim 6.0.1) for the check.

## What is in the scene, and how good it is

| Part | Source | Fidelity |
|---|---|---|
| Terrain, water, parking (`site/ground,water,road`) | Tile mesh reduced to bare earth, with OSM water and parking lots detected in the imagery | 1 m grid, texture from a 12.5 cm top-down render of the tiles |
| Roads (`site/road_osm`) | OSM road and path polygons, subdivided to 2 m, draped 5 cm over the terrain | Smooth kerb lines in the segmentation |
| **B01** "133" building | Dense drone reconstruction; box model measured off it (`measure_sheets.py`, `specs/B01.yaml`) | **Enterable.** Windows, doorways, stairs, decks within about 0.2 m. The back face and interior were not filmed. |
| **S01** drill tower | Tile footprint and height; opening pattern from a Google oblique | **Enterable.** 6 floors, external stair, a door on every landing. The opening positions follow the pattern and are not surveyed. |
| **B03** strip mall | Roof-height grid from the tiles; storefronts from a Google oblique | **Enterable.** Five units, each with its own roof state (intact, gone, or pancaked). |
| **B06** south warehouse | Ortho (canopy, walkways, ridge) and tile height | **Enterable.** Gabled hall: 2 bay doors and a personnel door under the canopy, main entrance opposite. The door sizes are guessed. |
| Industrial pad (`PAD`: S02–S06, mast, 3 cabins) | Tile blobs (position, footprint, height); identities from the Google obliques | Primitives: rail tank car, sphere vessel, pipe rack, tank on legs, X-braced lattice tower, cabins with doors |
| **R01** west rubble pile | Dense drone reconstruction, 2.5D heightfield at 0.2 m | About 34% from drone footage, the rest from the tiles; colour matched to the tiles |
| R02 east pile, R03 collapsed houses | Tile mound plus 537 debris pieces | The mound is the collider; the pieces are visual only |
| Other buildings (`lod1/`) | The tile outline, extruded: main roof level plus any attached lower annex | Closed, with the real roof texture. Not enterable. |
| Trees (`trees/`) | Canopy peaks in the tile heights; 4 NVIDIA tree species as instanceable references | Position and height from the tiles; invisible trunk and crown colliders |
| Vehicles (`vehicles/`) | Road vehicles: vehicle-sized blobs on roads, parking lots and near vehicle labels, fitted in the world frame. Rail cars: hand-surveyed car by car (`specs/rail_cars.yaml`), because the tiles merge coupled and derailed cars | Cars, vans and buses from the standalone pack; box truck, dump truck and container from `assets/lib`; tank cars, box cars, coaches and a locomotive from Nucleus + Objaverse. Nothing vehicle-sized is left as raw tile mesh or a box |
| Props (`specs/props.yaml`, S08) | S10 water tower (library asset), S08 canopy (spec) | Measured off the tiles / ortho |
| Everything else raised (`site/tiles_*`) | Raw tile triangles, classified | Crude: kept so the silhouette is complete |

Measured with `isaac_check.py` in host Isaac Sim 6.0.1 on an RTX 5090: about
5.5 ms per frame at 1600x900, and 12.6k prims.
- **Raycasts:** B01's window and doorway are open, B01's wall blocks, and the
  pile, a tree trunk and a bus all hit.
- **Semantics:** Isaac's `semantic_segmentation` returns all 8 classes.

## Rebuilding

Run from this directory. Each script's docstring has the details.

```bash
PY=~/.venvs/recon/bin/python
# 0. inputs cut from the blend
blender -b data/blender_data/disaster_city.blend --python ortho.py -- 75 -300 560 4480 data/recon/ortho_site.png
blender -b data/blender_data/disaster_city.blend --python tiles_crop.py -- 75 -300 290 data/recon/tiles_site.npz
blender -b data/blender_data/disaster_city.blend --python tiles_crop.py -- 39 -388 150 data/recon/tiles_R01.npz
blender -b data/blender_data/disaster_city.blend --python tiles_crop.py -- 47 -410 150 data/recon/tiles_B01.npz
blender -b data/blender_data/disaster_city.blend --python osm_export.py -- data/recon/osm.npz
blender -b data/blender_data/disaster_city.blend --python tiles_extract.py -- 188 -390 26 data/recon/rubble_east/R02_tiles.usd
# 1. drone reconstructions: hours on the GPU; the dense step needs ~15 MB of disk per image while it runs
$PY frames.py rubble_west && $PY recon_sfm.py data/recon/rubble_west && $PY recon_dense.py data/recon/rubble_west
$PY recon_align.py data/recon/rubble_west data/recon/tiles_R01.npz --pairs ...   # rough start: 2 picked corners
$PY recon_georef.py data/recon/rubble_west --views 24 --iters 4                  # render-and-compare vs the tiles
$PY frames.py b01 && $PY recon_sfm.py data/recon/b01
$PY recon_extend.py data/recon/b01 Bh B07,B08,B09,B10,B02,B03,A02,A03,A06        # the 4 fps interior frames
#    densify a subset: every Bh frame + every other outside frame, as a model with the rest deregistered
#    (sparse_sub; see recon_dense.py), then:
$PY recon_dense.py data/recon/b01 1200 data/recon/b01/sparse_sub
$PY recon_georef.py data/recon/b01 --model sparse_ext --views 40 --iters 5       # start: rubble_west's transform carried over
# 2. models
$PY lod1_buildings.py
$PY measure_sheets.py data/recon/b01 B01 --yaw 47.35                              # the sheets specs/B01.yaml was measured on
$PY recon_mesh.py data/recon/rubble_west R01 data/recon/tiles_R01.npz --radius 24 --margin 0.5
$PY build_hero.py specs/B01.yaml
for g in drill_tower:s01/S01 strip_mall:b03/B03 warehouse:b06/B06 industrial_pad:pad/PAD; do
  $PY gen_${g%%:*}.py && $PY build_hero.py data/recon/${g#*:}_spec.yaml; done
# 3. site layer, assets, scene. ground.py runs twice: place_assets.py reads its rasters, then ground.py cuts out the replaced vehicles
$PY ground.py && $PY place_assets.py && $PY ground.py && $PY assemble_scene.py
# 4. check and ship
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh isaac_check.py data/recon/disaster_city.usda data/recon/shots/<name>
# third-party assets: mirror from Nucleus (Kit) / fetch from Objaverse, then normalise + look at them
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh nucleus_mirror.py FactoryDistrict/Meshes/CargoCar_mdl.usd ...
../../.venv/bin/python ../objaverse_assets.py ensure <uid> --target-size 17 --fit max
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh asset_library.py                    # -> data/recon/assets/lib/*.usda
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh asset_gallery.py <out_dir>          # one render per library asset
# before/after: raw tiles vs grafted scene from the same cameras
blender -b data/blender_data/disaster_city.blend --python tiles_extract.py -- 75 -300 400 data/recon/raw_tiles/tiles_site.usd
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh before_after.py shots.json <out_dir>
$PY footprint_check.py                        # every model vs its tile footprint -> data/recon/footprints.tsv + overlays
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh package.py                          # data/dist/disaster_city/ (Kit USD)
```

`place_assets.py` reads local copies of assets under `data/recon/assets/`, taken
from `scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/` (4 trees and
their `materials/`) and `scene_gen/assets/standalone/{cars,debris/pieces}/`.

To add an enterable building, write a spec in `specs/` (walls with `openings`,
boxes, `stair`, `rail`, `beam`, `cyl`, `sphere`, and optional `lights`; see
`build_hero.py`), or a
generator like `gen_strip_mall.py`. Then add the building to `PIECES` in
`assemble_scene.py`. Its LOD1 box is switched off, and `ground.py` masks the
tile surface under it.

## Things that bit, so they don't bite again

- **Nucleus assets are kit-written USD crates.** The pip `usd-core` cannot open them, and
  `UsdUtils.ComputeAllDependencies` aborts on them even under Kit. Mirror, normalise and package
  under `~/isaacsim/python.sh` (Kit's USD), and walk dependencies with a tolerant walker. Their
  `/Game/...` material paths are Unreal leftovers that resolve nowhere; the MDLs carry the look.
- **Look at every third-party asset before placing it** (`asset_gallery.py`). Two Objaverse
  picks were rejected on sight: one converted standing on its end, one a cartoon cart. The
  cylindrical tank car's bounding box is 7.7 m tall (it includes a stand), so rail cars are capped
  at 4.6 m. The heavy ones (45k-99k triangles) took the scene from 0.8M to 2.7M triangles
  (12.5 ms per frame); decimate them if frame time matters.

- **Aligning a drone reconstruction to the tiles.** SIFT on the orthos (different capture dates),
  height-map correlation and FPFH+RANSAC all failed. The 2-corner picks + ICP route that
  replaced them "worked" but left R01 5 m and B01 ~13 m off with the scale 12% wrong -- ICP on
  flat ground cannot fix a horizontal shift -- and B01 was modelled on that. What works is
  `recon_georef.py`: render the tile mesh from the reconstruction's own cameras, LoFTR-match
  each photo to its render (SIFT finds nothing across the photo / tile-texture gap), ray-cast
  the matches into the mesh, PnP each camera, and fit a similarity to the camera centres.
  It converges in 3 iterations to 0.1-0.2 m camera residuals.
- **Every model is checked against the tiles.** `footprint_check.py` compares each model's plan
  footprint with the tile blob under it (IoU, offset, long-axis direction). It found the tank
  car 90 deg off (typed-in yaws; the pad is now fitted blob by blob in the world frame), the
  box buildings' missing annexes (LOD1 is now the extruded tile outline, two levels), and
  wrecked cars invisible at a building-sized height cut. Flags that remain are explained:
  S02 (a lattice the tiles smear into a lump), B01 (the tiles still show its collapsed wing),
  cars in dense rows (neighbours).
- **Replacing means replacing the whole blob.** Masking only the tile triangles inside a model's
  box left melted fragments around it; `ground.py` now swallows every raised tile blob a model
  touches, clipped to 4 m (2 m for LOD1, 1.5 m for vehicles).
- **The tiles are older than the drone footage.** B01's blue-roofed wing has
  since collapsed; its slab is now the tilted "arch" beside B01. Trust the drone
  data where the two disagree.
- **Blosm's tile materials are Emission shaders**, which the Blender USD
  exporter drops. `tiles_extract.py` swaps them for a Principled BSDF.
- **`displayColor` is linear.** sRGB values written into it render washed out.
- **A PointInstancer gets no semantic labels** in Isaac's segmentation, even
  with the label on its prototypes. Trees and debris are instanceable
  references instead.
- **Dense PatchMatch writes about 15 MB per image.** `recon_dense.py` deletes
  its depth maps after fusion. To densify a subset of images, write a sparse
  model with the rest deregistered; a list of image names alone crashes
  PatchMatch.
- **The LABELS points for the pad objects were up to 9 m off** (for example,
  the lattice tower). `gen_industrial_pad.py` places everything from the tile
  blobs instead.
- **Interiors were black.** Real-time RTX has no bounce light into rooms, and
  the stray dome (next item) took the skylight away too; turning on
  `/rtx/indirectDiffuse` did nothing. The enterable heroes now carry
  interior fill lights: a `lights:` grid per storey in the spec, emitted under
  `/<ID>/lights` (45 SphereLights in total). Deactivate that scope for a dark
  building. B03's roofless and pancaked units have none. Hero geometry also
  binds rough UsdPreviewSurface materials, because Isaac's default material
  showed every light as a glint on the walls.
- **Blender's USD export writes the Blender world as a black DomeLight.** It
  came in with the R02 tile cut-out, and because RTX uses one dome, it blacked
  out the sky over the whole site for days; it looked like a headless streaming
  issue. `tiles_extract.py` now passes `convert_world_material=False,
  export_lights=False`, and `isaac_check.py` warns if the stage holds anything
  but exactly one DomeLight.
- **The scene carries its own sun and sky** (`/World/Environment`, with the HDR
  under `data/recon/sky/`). A frozen scene with no sky light renders black on
  another machine (`.agents/skills/freeze-portable-scenes`).
