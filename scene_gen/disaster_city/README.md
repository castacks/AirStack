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
| Other buildings (`lod1/`) | Footprint, yaw and height from the tiles | Closed boxes with the real roof texture. Not enterable. |
| Trees (`trees/`) | Canopy peaks in the tile heights; 4 NVIDIA tree species as instanceable references | Position and height from the tiles; invisible trunk and crown colliders |
| Vehicles (`vehicles/`) | Tile blobs fitted with car, van or bus assets, or a box proxy | 13 assets and 12 box proxies (rail cars, trailers, dumpsters); 18 irregular blobs keep their tile mesh |
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
$PY recon_align.py data/recon/rubble_west data/recon/tiles_R01.npz                 # writes orthos: pick 2 corners on them
$PY recon_align.py data/recon/rubble_west data/recon/tiles_R01.npz --pairs 861.7,316.7:48.5,-396.5 716.7,560:32.2,-414
$PY frames.py b01 && $PY recon_sfm.py data/recon/b01
$PY recon_extend.py data/recon/b01 Bh B07,B08,B09,B10,B02,B03,A02,A03,A06        # the 4 fps interior frames
#    densify a subset: every Bh frame + every other outside frame, as a model with the rest deregistered
#    (sparse_sub; see recon_dense.py), then:
$PY recon_dense.py data/recon/b01 1200 data/recon/b01/sparse_sub
$PY recon_align.py data/recon/b01 data/recon/tiles_B01.npz --pairs 670,627.5:32.2,-414 560,1030:48.5,-396.5
$PY recon_icp.py data/recon/b01 data/recon/rubble_west --at 45.6,-409.8           # ICP onto the R01 cloud
# 2. models
$PY lod1_buildings.py
$PY measure_sheets.py data/recon/b01 B01                                          # the sheets specs/B01.yaml was measured on
$PY recon_mesh.py data/recon/rubble_west R01 data/recon/tiles_R01.npz --radius 24 --margin 0.5
$PY build_hero.py specs/B01.yaml
for g in drill_tower:s01/S01 strip_mall:b03/B03 warehouse:b06/B06 industrial_pad:pad/PAD; do
  $PY gen_${g%%:*}.py && $PY build_hero.py data/recon/${g#*:}_spec.yaml; done
# 3. site layer, assets, scene. ground.py runs twice: place_assets.py reads its rasters, then ground.py cuts out the replaced vehicles
$PY ground.py && $PY place_assets.py && $PY ground.py && $PY assemble_scene.py
# 4. check and ship
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh isaac_check.py data/recon/disaster_city.usda data/recon/shots/<name>
$PY package.py                                                                     # data/dist/disaster_city/
```

`place_assets.py` reads local copies of assets under `data/recon/assets/`, taken
from `scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/` (4 trees and
their `materials/`) and `scene_gen/assets/standalone/{cars,debris/pieces}/`.

To add an enterable building, write a spec in `specs/` (walls with `openings`,
boxes, `stair`, `rail`, `beam`, `cyl`, `sphere`; see `build_hero.py`), or a
generator like `gen_strip_mall.py`. Then add the building to `PIECES` in
`assemble_scene.py`. Its LOD1 box is switched off, and `ground.py` masks the
tile surface under it.

## Things that bit, so they don't bite again

- **Automatic alignment of a drone reconstruction to the tiles failed three
  ways.** SIFT on the top-down images failed because the captures are from
  different dates. Height-map correlation latched onto the wrong place.
  FPFH+RANSAC was ambiguous at every scale. What works is 2 hand-picked
  building corners (`recon_align.py --pairs`), then ICP against an
  already-aligned cloud from the same flights (`recon_icp.py`).
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
- **Interiors are dark.** The scene carries no interior lights, which matters
  for RGB-only search inside B01, S01, B03 and B06.
- **The sky renders black in the headless check**, with both the Nucleus HDR
  and a local one. The lighting still works, so this is cosmetic.
