---
name: reassemble-buildings-in-frame
description: Putting a baked, sliced, swapped or kit-built building BACK into the generated city in the right frame — the three frame bugs that each read as "buildings on top of the road" and "empty blocks" (a cropped manifest composed in the full-city frame, a bake swapped under the intact cell's 0.01 scale and centroid shift, a world-space centring delta written as a local translate under a yawed holder), the frame contract every reassembly must obey, the offline audit tool and unit tests that gate a cell without Isaac, and why per-building review captures never showed any of it. Read before touching any code that hides an intact building and composes something in its place.
---

# Reassembling buildings in the right frame

Every disaster pipeline does the same thing to a damaged building: it hides
the INTACT placement the city generator made and composes something else —
a fire bake, an earthquake archetype, a live-sliced GAC source, a kit build —
where it stood. Three separate pipelines got the frame of that "where it
stood" wrong, and all three read identically to a reviewer: a building
standing on a road or through a neighbour, and an empty lot (sometimes a
whole block) where the hidden intact used to be. The user's report,
2026-09-02: "buildings are spawning on top of the road, they are empty
blocks ... I was getting the same issue in earthquake ... in tornado ...
there's definitely a frame issue, something changes when we reassemble it."

This skill is the frame contract, the three incidents with their measured
numbers, the fixes, and the offline gate. None of the three needed Isaac to
find or to prove; all three were found in one afternoon with `usd-core`,
the shipped `GT_hints.json`, and a host-side layout rebuild.

## The frame contract

A city cell is `(x_m, y_m, z_m, yaw_deg)` — the placement's VISUAL centre
in plan, the ground, the yaw the packer chose. `apply_placements` turns that
into transform ops on the intact cell:

```
translate = (x_m − c, y_m − c, z_m − c)   c = the asset's bbox-centroid offset, rotated by yaw
rotateXYZ = (roll, pitch, yaw)            roll = +90 for a Y-up asset
scale     = pack scale                    0.01 for GreatAmericanCity (centimetre-authored)
```

Everything you compose in its place is authored in METRES, centred on the
origin in plan, base at z = 0, Z-up, yaw 0. Measured with usd-core, not
assumed: every `gac_quake/*.usd`, every `archetype/bld_*_DG*.usd`, every
fire bake — bbox centre ≈ (0, 0), base ≈ 0 (a collapsed grade spills debris
and shifts the bbox a few metres; the standing part is still centred).

So the reassembly's transform is ONLY the cell's `(x_m, y_m, z_m)` and `yaw`,
scale 1 (times the stage's `ssf`), and NOTHING from the intact asset's ops
may survive: not its scale, not its roll/pitch, not its centroid shift. The
fire launcher's `place_holder` docstring stated this rule in August; the
earthquake and the live tornado path each broke it a different way.

Two corollaries:

- **Any centring delta is expressed in the frame it is applied in.** A
  translate op on a child of a yawed holder acts in the holder's rotated
  frame; a delta measured in world coordinates is wrong there by the
  rotation. Measure with `BBoxCache.ComputeRelativeBound(prim, cell)`.
- **A record's coordinates belong to the frame the record was solved in.**
  A manifest solved on a re-centred crop carries cropped coordinates; the
  stamp `x_orig`/`y_orig` is the only full-city value on it, and it wins
  whether or not a crop window is set at launch.

## Why GAC is the canary

`_plans/gac_buildings.json` (the checked-in GreatAmericanCity size cache):
the pack's pivots sit at a CORNER, not the centroid.

| asset | footprint (m) | cx (m) | cy (m) |
|---|---|---|---|
| SM_Building_30 | 60 x 142 | −28.0 | +70.5 |
| SM_Building_10 | 45 x 109 | −21.2 | +54.6 |
| SM_Building_09 / 07 / 13 / 14 / 15 / 16 | 84–86 x 57–60 | −42 to −43 | +28 to +30 |

Downtowncity pivots are all (0, 0); kit archetypes and fire/quake bakes are
origin-centred. So a frame bug that costs a kit building nothing costs a GAC
building half its footprint — which is why every one of these showed up on
"the GAC buildings" first, and why the user read it as a slicing problem.
The slicer's output is correct; the frame is lost on the way back in.

## The three incidents

### 1. Fire — a cropped manifest composed in the full-city frame (`urban_fire_city_launch_script.py`)

The three shipped `Fire/Urban/level_{1,2,3}/1` cells. Manifests were solved
on re-centred 1 km crops of the 1.5 km dump (`fc_dump_crop.py`; shift =
minus the window centre: L1 (−180, 180), L2 (100, 150), L3 (20, 230)), then
composed into the full 1.5 km stage WITHOUT `FC_CROP_WINDOW`. `load_fire`
only shifted records back when that env var was set; `resolve_cell` prefers
`x_orig` and hid the RIGHT intact building; `compose_bakes` read `rec["x"]`
and put the holder one window-centre away.

Measured on the shipped GT against a host-rebuilt road network
(`tools/city_layout_audit.py`): substitutes with > 3 % of their footprint on
a road 14/25, 38/71, 78/130; six level-3 blocks lost EVERY building (all
hidden for fire) and read as empty; intact buildings off-block 0/997. At the
`x_orig` coordinate: 0 / 0 / 0.

Fix: `record_xy(rec)` (x_orig wins) is the one coordinate rule for both
`resolve_cell` and `compose_bakes`; `load_fire` un-shifts on the stamp with
or without a crop window (idempotent); the FRAME GUARD refuses a bake whose
holder would sit more than `CELL_MATCH_TOL_M` (0.5 m) from its cell BEFORE
anything is hidden; a post-compose BLOCK AUDIT reports bakes under
`BLOCK_FRAC_MIN` inside a block. The shipped cells need a re-freeze; the
code change does not touch them.

### 2. Earthquake — a bake swapped under the intact cell's scale and centroid (`disaster/quake.py`)

`quake.assemble` used "keep the transform, swap the reference":
`ClearReferences(); AddReference(bake); prim.Load()` on the intact cell. A
GAC cell carries `scale 0.01`, so the metre-authored bake composed at 1/100:
measured on the real `gac_SM_Building_26_DG3_s348.usd` under a corner-pivot
cell — a 0.44 x 0.25 x 0.78 m speck, 14 m from the lot. 21 of the 27 GAC
originals in `eq500_v5_local` went that way; the records' W/D were measured
BEFORE the swap (`_mono_dims`), so `quake_buildings.json` looked perfectly
in-block while the rubble ring, heap clearance and dust halo were drawn at
full size round an invisible building. Round 6 had root-caused those as
"dressed plazas". A same_art twin (scale 1) inherited only the centroid
shift and stood up to half a footprint off its lot.

Fix: `quake._swap_reference(stage, prim, p, usd, ssf)` — swap, then
re-author the cell as translate `(x_m, y_m, z_m)`, rotateXYZ `(0, 0, yaw)`,
scale `ssf`, precision-safe through `scene_generator._set_xform_ops`; then
the COMPOSE GUARD measures the result and prints `[quake] ***` when it is
empty or under `SWAP_MIN_FOOTPRINT_M` on both axes. All four swap sites in
`assemble` use it. The pair-interaction DG4 bump in `_d_geom_collapse`
acts on a cell that already has the right frame and was left alone.

### 3. Tornado (live slicing) — a world-space centring delta under a yawed holder (`disaster/gac_fire.py`)

`gac_fire.place_source` centres the merged source on the cell by
subtracting the WORLD bbox centre from the cell's world translation and
writing that as the asset's LOCAL translate. Right under an unrotated cell
(every bake launcher and probe, the bench's yaw-0 holders). The tornado
city launcher yaws every holder by the placement's yaw; the bench's A-row
holders are yawed 180. Measured with usd-core on a corner-pivot box the
size of SM_Building_30 under a holder at (100, 50): dead on at yaw 0;
(−41, +101) m off at yaw 90; (−60, −142) m at yaw 180; (+101, +41) m at
yaw 270.

Fix: `ComputeRelativeBound(holder, cell)` — the asset measured in the cell's
own frame — so the correction is exact for any yaw and byte-identical for
yaw 0 (which is why the bake launchers and probes were never wrong).

## Why three review rounds missed all of it

- Per-building captures frame on the RECORD's coordinate, so a displaced
  bake is photographed where it landed and looks fine.
- Records carry the dimensions measured before the swap, so any audit of
  the records says "in-block".
- `snapshots_rp.overview` is a PERSPECTIVE camera 0.95 x span up on an 18 mm
  lens: a 93 m tower at the plate edge of a 500 m plate appears ~57 m
  outward, over the border road. Tall edge buildings LOOK like overhang in
  every top-down; judge overhang from records or an orthographic capture.
- Nothing measured the composed result. The two guards above exist because
  a one-line bbox check after composing would have caught bugs 2 and 3 on
  the first launch, and the frame guard catches bug 1 before it hides
  anything.

## The offline gate — run it before calling any urban cell done

```bash
# 1. the unit tests (bare usd-core; no Kit)
uv run --with usd-core --with pytest python -m pytest -q \
    scene_gen/tests/test_city_layout_audit.py \
    scene_gen/tests/test_quake_swap_frame.py \
    scene_gen/tests/test_place_source_frame.py

# 2. the built cell against its own blocks and roads
python3 scene_gen/tools/city_layout_audit.py \
    --gt <cell>/GT_hints.json --dump <dump the manifest was solved on> \
    --manifest <manifest.json> [--layout <host-built layout json>] \
    --png ~/fire_previews/road_overlap/<cell>.png
```

Pass condition: `record_xy max 0.00 m`, 0 substitutes overhanging a block,
0 on a road, and every non-park block with a building in it. The
`--layout` input is a host-side `fire_city_dry_run.build_layout` rebuild
(blocks + road corridors; it matched the Kit dump's blocks 49/49, 51/51,
60/61 on the three fire seeds — the one miss was a window-clipped block).
On the next Isaac launch, grep the pane for `[fc] ***` and `[quake] ***`:
zero lines is the pass condition; then run the audit on the capture.

## What is NOT yet verified

No render has been made since the three fixes (user hold on Isaac). The
unit tests prove the frames on synthetic and on real bake files; a render
still has to confirm that the earthquake's GAC bakes stand in their lots
and that the tornado city's yawed GAC records sit on their cells. The
shipped fire cells must be re-frozen (with `FC_CROP_WINDOW=cx,cy,1000,1000`
for the dataset's 1 km contract, or without it for the 1.5 km plate; both
now place bakes on their cells).

## Files

| file | role |
|---|---|
| `scene_gen/tools/city_layout_audit.py` | offline audit: off-block, on-road, empty blocks, manifest frame check, figure |
| `scene_gen/tests/test_city_layout_audit.py` | the fire launcher's coordinate rule and guards (AST-executed), audit geometry, shipped fixtures |
| `scene_gen/tests/test_quake_swap_frame.py` | the earthquake swap: naive idiom as negative control, fixed swap, Y-up roll dropped |
| `scene_gen/tests/test_place_source_frame.py` | `place_source` centring at every yaw |
| `simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py` | `record_xy`, `_cell_distance`, `CELL_MATCH_TOL_M`, `BLOCK_FRAC_MIN`, `_frac_inside_blocks` |
| `scene_gen/disaster/quake.py` | `_swap_reference`, `SWAP_MIN_FOOTPRINT_M` |
| `scene_gen/disaster/gac_fire.py` | `place_source` |
| `scene_gen/tools/baseline_layouts.py` | the committed (seed, window centre) table the fire cells were cropped with |
