# standalone — the unified urban asset pack, as files

Everything `config/asset_packs/standalone.yaml` references that is not in
`shared.yaml` or `assets/downtowncity/`. Converted 2026-08-26 from
`~/coasei/new_assets/` (Sketchfab/Quixel/Tripo downloads) with
`tools/extract_objects_to_usd.py`; gitignored (414 MB) and reproducible from
the commands below.

Every USD is **metres, Z-up, footprint centred on the origin, base at z = 0**,
in its own folder with its own `textures/` (several sources reuse texture file
names). Orientation is baked in: cars face +X, the midrise blocks have their
shopfront on +X, the terrace houses their door on -X (what
`districts._lay_terrace` turns to the street).

| folder | count | what | source | notes |
|---|---|---|---|---|
| `cars/` | 10 | 4 low-poly 80s cars, sedan, red hatchback, delivery van, police car, 2 burned-out wrecks | `cars/*.usdz` | fit to 4.4-5.4 m; the police car's wheels were at the rig origin in the source and were put on the axles by hand (`police_fix.py`, this session's scratchpad) and the car decimated 256k -> 100k tris |
| `buildings/intact/tower/` | 1 | cream precast office tower on a car-park podium, 93 m | `buildings/fbx.zip` (ASCII FBX -> glb via FBX2glTF) | untextured, material colours only |
| `buildings/intact/midrise/` | 9 | street-wall blocks, 12-26 m, shopfront base, one detailed face | `buildings/buildings.usdz` | grouped by object-name prefix `Building_NN_` |
| `buildings/intact/rowhouse/` | 5 | three-storey rendered houses, 10-11 m, 4-8 m deep | `houses/*.zip` (Tripo glb) | scaled by eye to 3 storeys, decimated 1.5M -> 50k tris |
| `buildings/destroyed/tower/` | 6 | ruined high-rises, 50-69 m | `buildings/post_apocalyptic_buildings.usdz` | x28 (~3.3 m storeys) |
| `buildings/destroyed/midrise/` | 1 | abandoned brick factory, 21 m | `buildings/simple_low_poly_abandoned_brick_building.usdz` | inches (x0.0254); ground slab dropped |
| `buildings/intact/tower/` (drop 2) | 8 | `stepped_tower` (65x78x81 m, inches), `city_tower_01..07` (40-149 m wide, 125-271 m tall) | `large_low_poly_building.usdz`, `city_buildings_blackthornprod_video.usdz` | the BlackthornProd set split on object names; 60-632 tris each — the cheap towers |
| `buildings/intact/midrise/` (drop 2) | 3 | `japanese_building` (28 m, external stairs), `mini_auto_service` (8.5 m showroom), `federal_bureau` (21 m stone U-plan, x0.1) | `janpanese_building.usdz`, `mini_auto_srevice.usdz`, `federal_bureau_...usdz` | fronts yawed onto +X |
| `buildings/intact/rowhouse/` (drop 2) | 2 | `old_brick_shop` (9.5 m, x0.0156), `graffiti_brick_house` (12.3 m, cm) | `old_building.usdz`, `brick_building_graffiti.usdz` | doors yawed onto -X for the terrace morphology |
| `buildings/intact/tower/` (drop 3) | 2 | `glass_curved_tower` (55.6x54.8x44.1 m, 2.8M->200k tris), `slab_tower` (48.7x64.6x83.3 m, **310 tris**) | `glass_curved_building.usdz`, `3d_building.usdz` | `296.fbx` from the same drop is EXCLUDED — a CAD site plan (kerb outlines, railings), no building |
| `buildings/intact/tower/` (drop 4) | 1 | `podium_highrise` (26.7x44.0x82.5 m, 122k tris) | `highrise_building.usdz` | slender white tower on a wide dark podium; glazed balcony stack + signed entrance yawed onto +X |
| `assets/nucleus/selected_citydemo/` | 20 | 18 of the 20 curated AEC CityDemo towers (45-113 m) + 2 midrise blocks, **36-147 tris each** | Nucleus `Projects/SEI-COA/selected_citydemo/` | mirrored locally only so the gallery can render them; the pack references the NUCLEUS path. Metres/Z-up/base z=0, so **no `scale:`** — unlike every other Nucleus entry |
| `debris/pieces/` | 34 | 12 slabs, 9 chunks, 6 lumps (concrete); 3 corrugated sheets, 4 rebar tangles (steel) | `debris/debris3.FBX`, `debris/debris.FBX` | split on loose parts; sources had no textures, so each carries one flat colour |
| `debris/piles/` | 2 | Quixel concrete rubble spread, Quixel rocky ground | `debris/*.usdz`, `debris/nordic_*.zip` | the FBX one's maps were wired up from the loose jpgs |

Construction materials (`material:`) were judged from Cycles renders of every
asset; the calls and reasons are in the pack yaml.

## Rebuilding

```bash
V=~/coasei/scenegen/.venv/bin/python           # bpy 5.2 + scipy
X=scene_gen/tools/extract_objects_to_usd.py
A=~/coasei/new_assets
O=scene_gen/assets/standalone

# inventory of any source (no export)
$V $X $A/cars/low_poly_cars.usdz --list

# cars: one per named object, fit to a length, yawed so the front is +X
$V $X $A/cars/low_poly_cars.usdz -o $O/cars --group '^(car\d)' --fit-length 4.6 --yaw -90 --per-asset-dir
$V $X $A/cars/red_car.usdz       -o $O/cars --join red_car --fit-length 4.4 --yaw 90 --per-asset-dir
$V $X $A/cars/standard_delivery_can.usdz -o $O/cars --join delivery_van --scale 0.0254 --yaw 180 --per-asset-dir

# blocks: one per name prefix; houses: joined, decimated, scaled, yawed
$V $X $A/buildings/buildings.usdz -o $O/buildings/intact/midrise --group '^(Building_\d+)_' --yaw 90 --per-asset-dir
$V $X <house>.glb -o $O/buildings/intact/rowhouse --join house_01 --decimate-to 50000 --scale 13.1 --yaw -90 --per-asset-dir

# ruins
$V $X $A/buildings/post_apocalyptic_buildings.usdz -o $O/buildings/destroyed/tower --group '^(Building__\d)_' --scale 28 --per-asset-dir
$V $X $A/buildings/simple_low_poly_abandoned_brick_building.usdz -o $O/buildings/destroyed/midrise \
    --join abandoned_brick_building --exclude 'Material2\.00[34]' --scale 0.0254 --per-asset-dir

# debris: connected components, a subset by name, one flat colour
$V $X $A/debris/debris3.FBX -o $O/debris/pieces --loose-parts --min-tris 50 \
    --flat-material 0.55,0.53,0.50,0.9,0.0 --match '^debris3\.(001|002|...)$' --per-asset-dir
$V $X nordic/Nordic_Beach_Rocky_Ground_ukoncdamw_Low.fbx -o $O/debris/piles --join rocky_ground --pbr-maps nordic/ --per-asset-dir

# the ASCII FBX needs FBX2glTF first (Blender reads binary FBX only)
FBX2glTF -i fbx.fbx -o fbx --binary
$V $X fbx.glb -o $O/buildings/intact/tower --join office_tower --no-textures --per-asset-dir
```

Output files were then renamed (`car1` -> `lowpoly_coupe`, `Building_01` ->
`block_01`, `debris3_001` -> `slab_01`, ...) by moving the folder and the
`.usdc` together — texture paths are relative to the folder, so that is safe.

Previews: `scene_gen/render_usd.py <dir>` (Cycles contact sheets) is how the
materials and the fronts were judged.

## Adding an asset (the running procedure)

`~/coasei/new_assets/{cars,buildings,houses,debris}` is watched; a new file is
processed like this, and `.sources_processed.json` here records what has been
handled (seeded 2026-08-26 with the 22 original sources).

1. Inventory: `$V $X <file> --list` — what objects, sizes, units.
2. Stage-1 convert at natural scale + `render_usd.py` previews to judge unit,
   material, typology and which face is the front (the four-azimuth sheet
   from `scratchpad/axis_views.py` answers the last one).
3. Final convert into `assets/standalone/<type>/...` with the extractor
   (`--fit-length`/`--scale`, `--yaw` so the front is +X / -X for terrace
   houses, `--decimate-to` for dense AI meshes, `--per-asset-dir`).
4. Add the entry to `config/asset_packs/urban_v2.yaml` (and `standalone.yaml`)
   with its `material:`; check `plan_png.py --config urban_v2_small` builds.
5. Gallery: `building_gallery.py --pack urban_v2 --only <name>` renders just
   that building and re-stitches `grids/` + `grid_all.png`. The same-scene
   `all_buildings.png` / `groups/` are not regenerated until asked.
6. RTX check when in doubt (black-under-RTX materials):
   `asset_catalogue_launch_script.py` with `CAT_MATCH=<name>`.

## Checking the pack

`tools/pack_report.py` is the one command to run after hand-editing a pack:

    python3 tools/pack_report.py --pack urban_v2        # reconcile + report
    python3 tools/pack_report.py --pack urban_v2 --no-render

It renders any building the pack gained, DELETES the renders of any it lost
(commenting an entry out otherwise leaves it in the grid — the stitcher globs
the folder), re-stitches, then prints: typology x condition, construction
material, entries per pool with a `disabled` column counting commented-out
lines, the tag histogram, where the assets live (Nucleus / repo-local /
Objaverse), and gallery coverage. It reads the pack under AirStack's 3.11 venv
and shells the renderer out to `$BPY_PYTHON` (scenegen's 3.13), since `pxr` and
`bpy` cannot share a process.

## urban_v2 sources from Nucleus (2026-08-27)

`config/asset_packs/urban_v2.yaml` no longer contains a single `airstack://`
path: all 105 were rewritten to absolute `omniverse://` URLs under
`Projects/SEI-COA/scene_gen/assets/`, where the same files were uploaded. The
three `urban_v2*` presets' asphalt override went with them. The files here stay
— `standalone.yaml` still references them locally, and they are what the Cycles
gallery renders (`building_gallery.resolve` maps a Nucleus URL back to the
checkout copy, then to the `assets/nucleus/` mirror).

**The catch, and the fix.** Only Kit can open an `omniverse://` stage, so the
HOST cannot measure these any more and would pack the city against
`fallback_sizes` — a 1000 m host build dropped from 1,138 buildings to 525.
`assets/.measurements.json` absorbs it: the cache keys a remote asset on path
alone (by design — see `measure_cache.py`), so the measurements were taken from
the byte-identical local copies and stored under the new URLs, each size-checked
against Nucleus first. Host and sim now agree exactly. One asset failed that
check and was deliberately left uncached: `aec/brownstone/.../Douglas_Fir.usd`
is 401 bytes different on Nucleus, so it falls back host-side until Kit
measures it.

The twelve `objaverse://` props followed on the same day: `urban_v2` now names
**264 assets and every one is an `omniverse://` URL** — no local scheme is left
in the pack or in its three presets. Two consequences: the Objaverse pre-flight
is no longer needed (`prepare_assets.py` looks for `objaverse://<uid>` and
correctly finds nothing to do), and `target-size-m` / `fit` on those entries are
now inert — only the pre-flight ever read them, and it bakes the normalisation
into the cached USD, so the file on Nucleus is already the right size. They are
kept as the record of how each prop was normalised.
