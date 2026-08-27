---
name: build-earthquake-scenes
description: Build or modify EARTHQUAKE-damaged URBAN scenes in scene_gen — the kit-building fit-out (floors, columns, furniture), the per-construction-type damage ladder (URM wall peel / parapet / corner / masonry collapse; RC infill / soft storey / pancake; glass fallout; lean-and-sink), the archetype bake and the downtown assembly. Read before touching disaster/quake_flow.py, disaster/quake.py, the eq_building_bench, bake_quake_archetypes or downtown_quake launchers, or the urban_quake asset set / downtown_earthquake preset. The wildfire and tornado skills are prerequisites; this one is about what a quake does that neither does, and why the urban buildings needed an interior first.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Build Earthquake Scenes (urban building damage)

## Read `build-wildfire-scenes` and `build-tornado-scenes` first

Every trap in those two files applies unchanged — the fracture/settle
mechanics, `trimesh` caching its engines at import (a fresh container pays a
ruined run), the `assetInfo`-poisons-CopySpec bake trap, tmux relaunch not
`airstack down`, `SNAP_DIR` under the mounted log dir. None of it is repeated.
**This file is only about what an earthquake does that fire and wind do not,
and about the URBAN buildings, which the other two never touched.**

The plan and the research it rests on: `scene_gen/_plans/earthquake_plan.md`
(numbers from EMS-98, FEMA P-154, Christchurch / Kahramanmaraş / Mexico City
/ Amatrice / Niigata reconnaissance; the two source reports are summarised
there).

## The pipeline

    downtown layout -> [bake once per (style x grade)] -> assemble (swap by field) -> tilt

| piece | file |
|---|---|
| per-building damage (the recipes, the fit-out, the materials) | `scene_gen/disaster/quake_flow.py` |
| scene assembly: field -> grade -> archetype swap, lean-and-sink | `scene_gen/disaster/quake.py` |
| single-building bench (one style per row, one recipe per column) | `simulation/isaac-sim/launch_scripts/eq_building_bench_launch_script.py` |
| archetype bake, settled one style-row at a time | `bake_quake_archetypes_launch_script.py` |
| the 250 m scene, no drone | `downtown_quake_launch_script.py` |
| building pools = pristine bakes | `scene_gen/config/asset_sets/urban_quake.yaml` |
| the preset (downtown.yaml shrunk to 250 m, earthquake on) | `scene_gen/config/presets/downtown_earthquake.yaml` |

Bench (iterate here first — everything is fractured and settled live):

    docker exec isaac-sim tmux send-keys -t isaac 'clear; SCENE_CONFIG=downtown \
      EQ_STYLE=commercial,office EQ_RECIPES=pristine,parapet_fall,corner_fail,out_of_plane,soft_storey,masonry_collapse,pancake,tilt_sink \
      SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/eq_bench PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
      /isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/eq_building_bench_launch_script.py \
      --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts' ENTER

An `EQ_RECIPES` entry is a grade (`DG1..DG5`, looked up in
`quake_flow.LADDER` for the style's construction type), a bare recipe name
run with its defaults, or `pristine`. Every building gets six camera prims
under `/World/ReviewCams` (top, four compass obliques, street) and a PNG
each.

---

# What is different, and why the fire/wind code cannot just be reused

## The buildings are URBAN KIT SHELLS, and they are hollow

`detail/urban_building.py` assembles a building from façade BANDS x sides x
corners plus a flat roof — no floors, no columns, nothing inside. A timber
house that burns or blows apart goes to a heap, so the suburb never needed an
interior. An earthquake EXPOSES interiors: a peeled masonry wall shows the
floors behind it (the "dollhouse"), a pancaked frame IS its slabs stacked, a
curtain wall that sheds its glass shows slab edges.

So `quake_flow.fit_interior` runs BEFORE any recipe, on every damaged
building: floor slabs (timber deck for masonry, concrete for frames),
columns on the module pitch for the concrete families, plaster partitions,
and cheap furniture referenced from Nucleus (`FURNITURE`: the Isaac Office
kit for offices, `ModularNeighborhood` for residential — both verified to
resolve, seated on the slab by measured bound). Pristine archetypes get no
fit-out; nothing can see it.

## Construction type decides the recipe

`FAMILY_TYPE` maps the kit families to three types and `LADDER[type][grade]`
is the recipe list:

| family | type | ladder |
|---|---|---|
| 01 stone, 03 brownstone, 04 brick, dw_b storefront, church | `urm` | glass -> parapet_fall -> corner_fail -> out_of_plane -> masonry_collapse |
| 02 office, civic | `rc` | infill_fail -> soft_storey -> pancake |
| 05 podium + glass tower | `rc_glass` | glass_fallout -> podium soft_storey -> tilt_sink |

Reusing one ladder for all of them is the fastest way to build the wrong
disaster: a brick block does not pancake and an office does not shed its
whole street wall.

The full recipe vocabulary (`quake_flow.RECIPES`), grouped as they were built:

| batch | recipe | what it does | where |
|---|---|---|---|
| core | `parapet_fall`, `corner_fail`, `out_of_plane`, `masonry_collapse` | masonry ladder | urm DG2-5 |
| core | `infill_fail`, `soft_storey`, `pancake` | frame ladder | rc DG2-5 |
| core | `glass_fallout`, `glass_loss` | curtain-wall / window glass | rc_glass; all |
| A | `settlement` | level sink 0.35-1.6 m, heaved kerb, boils | level `SETTLE` |
| A | `tilt_severe` | 10-30 deg on a raft that levers out, silt on the low side | level `TILT` |
| A | `overturn` | 60-90 deg about a base edge, shell whole, landing crush | level `OV` (not towers) |
| B | `mid_storey` / `storey_collapse` | crushed band 1..n-2 storeys up, block above twisted 2-8 deg and offset | rc DG4 (1 in 3) |
| B | `droop` | floors at an opening hinge down 12-35 deg (USAR lean-to / V) | inside `out_of_plane` |
| B | `balcony_fail` | cantilevers hang 20-60 deg or drop | rc DG2-4 |
| B | `roof_hole` | 20-50 % of the roof through onto a broken top floor | urm DG3-4 |
| C | `facade_scars` | plaster-loss patches + X-cracks on piers (geometry, no decal flags) | DG1-4 |
| C | `rooftop_fail` (+ `dress_roof`) | water tanks / AC units on every roof; tipped from DG1 | all |
| C | `signage_fail` | shop signs on end on the sidewalk | DG2-3 |
| C | `_lantern`, `_shaft` | buckled-bar column heads at a crushed storey; a lift shaft left standing in a pancake | inside soft_storey / pancake |

The foundation family is chosen by the ASSEMBLY, not the field: `quake.
_soft_soil` draws one ellipse per scene (`disaster.soft_soil: {center, rx_m,
ry_m, rate}` in the preset, `false` to disable) and inside it a share of the
still-standing buildings become `SETTLE` / `TILT` / `OV` by slenderness, one
overturn per scene and only where `_blocked` finds the fall clear.

## Gravity only — but out-of-plane walls need a PER-BODY velocity

No `bias` (that is wind) and `consume` is only used to thin foil (below).
The one directional thing is a masonry wall failing out of plane: it rotates
about its foot, so the top leads and the wall lands as a FAN on the street.
`settle.prepare(velocity_map={path: (vx, vy, vz)})` was added for this — a
single `bias` is one vector for the scene and cannot express it. Fragments
high on the wall get 0.4 + 2.4 * (z/H) m/s outward.

## The grade is DRAWN from the field, not thresholded on it

`quake_flow.level_for_intensity` draws `v = i * u` and cuts it, with the
cuts per type set so that at full intensity the mix reproduces the worst-block
reconnaissance figures (URM 5/20/30/25/20 % none/DG1-2/DG3/DG4/DG5, RC
20/30/28/12/10, glass towers 55/30/13/2/<1). Thresholding the field directly
made every building inside the strong-shaking radius DG5, which no real
block is — total collapse is a minority even in Antakya.

---

# Round 2 (2026-08-27): the review, and what it changed

The user's review of city 9 named six things; each was traced to a cause on
the bench before anything was changed, and each fix is a set of functions with
an owner prefix (`_a_` ragged breaks, `_b_` props and scars, `_c_` ground,
`_d_` pairs) so the next reader can find the round's work by grep.

| review item | cause (measured) | fix |
|---|---|---|
| floating tanks / AC units | `dress_roof` authored plant at the pristine roof height as STATIC; any recipe that moved or removed the roof left it hanging | `_b_settle_roof_plant` hands every plant prim to the solver after the recipes (rests on an intact roof, drops through a hole, rides a tilt, buried on total collapse); tanks are one merged mesh; per-object edge inset |
| no soil where a building tilted / sank | round 1 put one small `_berm` ring on the hot side only, and the CITY's mild lean (`_tilt_prim`) had no ground at all; the heave was on the wrong side (rotation sign) | `_c_ground_response`: heave wedge + buckled slabs + kerb on the low side, opened gap + exposed raft + lip on the high side, subsidence dish for settlement, mud line, fissures, boils; used by all four foundation recipes AND by `quake._tilt_prim`; everything clamped to the plate |
| straight shear lines | single-scale Voronoi (4-5 m cells) judged cell by cell; `shrink` is a ratio (8 cm on a 5 m cell = a black crack); `roughen` per fragment crazes the survivor; kit module seams; box-shaped roof-hole outline | `fracture.fracture_split` — two-scale split (dense 0.3 m cells along the break, refinement of boundary cells, chewed edges both ways), `roughen_field`/`inset` in metres; stepped course judges for URM, torn lines for RC; neighbouring storeys/bays lose a ragged band so no edge is a seam; `_a_roofify` converts a whole roof at once (no seam), face normals on fragments |
| white debris and wooden Xs | `damage._pbr` colours are LINEAR albedo: `plaster` 0.66 rendered 0.84 (white); scars were proud white blobs and 45 mm bars; signs were plaster boxes; glass white; `planks` tint is a no-op with a texture bound | palette darkened (`plaster` 0.32, `mortar` 0.24 linear; `plaster_dusty` etc.), recessed dark spall patches + jagged crack polylines within the pier, painted sign panels, tinted glass, `SETTLE_CULL_LEDGES` drops fragments resting on sills; heap material mix table |
| rectangular roof-collapse pieces | `r_roof_hole` outline was a rounded box; margins were rectangles | `_a_hole_outline` (closed wobbly polygon), sagging rim pieces, dropped pieces on the floor below, radiating cracks |
| buildings interacting | never modelled; `_blocked` refused any overturn toward a neighbour | `r_lean_on` (bearing failure to the contact angle, both buildings damaged at the contact), `r_collapse_onto` (upper storeys thrown onto a lower roof, punch-through), `r_pounding`, `_d_party_collapse` for terraces; `quake._d_interactions` runs a capped number of pairs LIVE at assembly (`QUAKE_INTERACT`, ~35 s each) and the rest geometrically |

Also in this round: `magnitude` (compile_disaster), several cities in one
stage (`CITIES`), monoliths (`urban_quake_v2`), the headless serialised runner
(`scene_gen/tools/eq_bench.sh`) and the two-at-a-time bake driver
(`bake_quake_headless.sh`). The agents' full notes: `scene_gen/_plans/
eq_round2_{A,B,C,D}.md`.

## Headless: how the bench, bake and city are run now

    scene_gen/tools/eq_bench.sh <snap> EQ_STYLE=commercial EQ_RECIPES=DG3,DG5 EQ_SEED=4 SETTLE_STEPS=2200
    LAUNCHER=downtown_quake_launch_script.py scene_gen/tools/eq_bench.sh city CITIES=M9.5,M5.5 CITY_SIZE_M=200
    scene_gen/tools/bake_quake_headless.sh            # all 16 styles, two Isaac processes at a time

* `docker exec` + `ISAAC_SIM_HEADLESS=true`, NOT the tmux pane, so several
  agents can queue runs; at most `GPU_SLOTS` (2) processes run on the card
  (a 16 GB card holds two fracturing benches; three do not fit). Captures
  under `~/docker/isaac-sim/logs/<snap>/`, the launcher's stdout in
  `<snap>.log` beside it (`docker logs` is empty).
* A CITY run wants the card to itself: two concurrent two-city runs with
  live pairs and the round-2 archetypes (a DG5 is ~4k static prims now) hit
  `ERROR_OUT_OF_DEVICE_MEMORY` and segfaulted in `librtx.scenedb` (final_2city,
  2026-08-27). Bench and bake rows fit two at a time; cities run with
  `GPU_SLOTS=1` or after the other slot is free.
* `PYTHONUNBUFFERED=1` is load-bearing: Kit hard-exits on `close()` and
  block-buffered prints never reach the log, so the DONE banner was lost.
* `.env` leaks EMPTY strings for every var compose forwards (`SETTLE_STEPS`,
  `SCENE_CONFIG=suburb`, ...): the runner supplies defaults, the launchers
  read ints with `os.environ.get(X) or default`. A city run that reports
  "0 buildings" compiled the SUBURB preset — check `SCENE_CONFIG`.
* The city prints its TIMING banner BEFORE its captures: the runner waits for
  `snapshots ->` or `EXIT 0`, never the banner.
* Editing `eq_bench.sh` while a run is in flight kills that run (bash reads
  the script incrementally).

## Building interaction (agent D)

A pair is (leaner, neighbour). `_d_pairs` scores standing DG0-DG3 buildings
whose top can reach a neighbour (`H sin(theta) - B (1 - cos(theta))` over the
gap, min 4 deg, max 26 deg) and the OV candidates `_blocked` refused; only
buildings with `intensity x grade_scale >= D_LEAN_MIN_INTENSITY` (0.6)
qualify, so an M5.5 city gets none. The first `QUAKE_INTERACT` pairs run the
recipe LIVE (the archetype steps aside, the kit is rebuilt in place,
fractured and settled — ~35 s a pair, 400-450 bodies); the rest up to
`QUAKE_INTERACT_PAIRS` are geometric (rigid lean to the contact angle, authored
spall bands, static wedge). Known gap: the neighbour is still a reference, so
a live pair damages the leaner and authors bands on the neighbour rather than
fracturing it. Pounding needs a real party wall: the preset's
`packing.building_gap_m` is 2.0 now (was 6.0 — read the comment there before
lowering it further; the guillotine packer leaves the slack at the block edge).

## Magnitude, several cities, monoliths

* `magnitude:` in a spec (or `MAGNITUDE=` on the launch line) DEFINES severity
  through `compile_disaster.magnitude_to_severity` (I0 = 1.5 M - 1.5, EMS-98
  intensity -> ladder) and shapes the field: M >= 7.5 shakes a plate uniformly
  (core 60 %, corners 85 %), M >= 8 over-drives the grade draw (`grade_scale`
  up to 1.35 at M9.5: a great earthquake is not "the worst block everywhere",
  it is worse), liquefaction scales from 0 at M5.5 to full at M7.5. Measured
  on a 200 m plate: M9.5 -> ~40 % DG5, 10 % DG4, 3 lean-ons; M5.5 -> DG0-DG2
  with one or two DG3 and nothing on the ground.
* `CITIES=M9.5,M5.5 CITY_SIZE_M=200 CITY_GAP_M=100 [CITY_SEEDS=3,5]` builds a
  row of plates west->east, each generated under its own `generated_cN`
  parent in city-local metres and then TRANSLATED (the parent must be an
  `Xform` — the generator's holder is typeless and a translate on it is
  silently ignored, which composed both cities at the origin once). A
  soil-bound void plane sits under the row. Captures are prefixed `cN_`.
* `ASSET_SET=urban_quake_v2` adds a minority of standalone monoliths and the
  six ruin towers — OPT-IN: those `standalone/buildings/intact` models came
  up as untextured cream boxes (their materials do not resolve in Isaac), so
  the deliverable uses the kit-only `urban_quake`. `quake._mono_pass` gives a monolith what a rigid body can
  show: ruin swap for a DG5 tower, heavy lean + ground for DG4-5, mild lean
  for half the DG3s. Sizes come from the world bound (`_mono_dims`), so the
  layout's 0/90/180/270 yaws are exact and a 45 deg one would be overestimated.

# Round 3 (2026-08-27): triangles, glass, solid walls, load time

The user's second review (two-city scene): breaks "very triangular — like
something added to the edge of a rectangular break"; glass towers "just had
some panes disappear, random panes hanging"; "I don't believe we can't
fracture zero-wall-thickness buildings"; and "optimize after we get the look
— scenes can't take hours to load". Research FIRST this time
(`_plans/earthquake_research.md` §11-§13, the field dump
`eq_round3_glass_recon_dump.md`, the implementer summary `eq_round3_R.md`),
then the census, then the code. Agent notes: `eq_round3_{M,R,T,P,G,G2,O}.md`.

| finding | what changed |
|---|---|
| census: 121/134 buildings fail the boundary-edge / back-face tests (16/16 kit styles; 105/118 of the urban_v2 drop) — but the kit walls are TWO SKINS with no rim (0.14-0.70 m apart), 01/02/03 façades have no back face, roofs are two triangles at 0.000 m | `scene_gen/tools/wall_thickness_census.sh` (40 s, no Kit), JSON + tables in `eq_round3_M.md` |
| the round-2 triangles were (a) failed CAPS — `slice_plane(cap=True)` fanning each open section into one 2-3 m triangle — and (b) 3-D Voronoi seeds in a member thinner than the cell pitch | `fracture.solidify` before every slice (`EQ_SOLID=1`, `T_SOLID_M` per type/role, glass stays a thin closed plate; `EQ_SOLID_N` seed scale, 0.85 recommended); seeding from the plane of weakness: `mode="brick"` (running-bond lattice, course/stretcher pitch, clusters by `keep`), `mode="prism"` (2-D seeds extruded through the thickness), staircase judges quantised to the bond (`_p_staircase`), rocking MACROBLOCKS for out-of-plane, lintels/quoins as monoliths, sliver rejection (needles only), no surface roughening (`P_ROUGH_M` 0.003) |
| acceptance test for shape | `scene_gen/tools/test_break_shape.py` on a bench's `frags.jsonl` (`EQ_DUMP_FRAGS=1`): oblique-face area, EI, blades, pieces > 2 m, equidimensional share, FI. Round-3 finals: oblique 0.1 %, EI 2-15 %, blades < 2 %, 166-172 rafts > 2 m; FI 33-37 % and equidimensional 38-42 % still miss R's demolition-waste numbers — those are for 50 mm-screened waste and would cost 4x the body count at the metre scale (documented, not chased) |
| glass: curtain-wall frame failure 1 in 371; loss is a contiguous BAND on the racked storeys; cracked-but-retained only for annealed/laminated; cracks corner-rooted; DG5 "out" 40-55 % not 85 %; no curtain-wall tower has collapsed in ten events | `r_curtain_wall` (bands per drift profile, survivors inside, cage + spandrels kept, corner-crack strips, crazed laminated panes, gasket strips, dice piles ≈ 1.2 x panel area, one merged mesh per (kind, side, storey), ≤ 5 materials, no bodies), `r_storefront_glass` + `r_window_glass` (Zhao's in-plane ladder: cracked → many out → empty openings with racked/bulging frames, sashes jammed rectangular in parallelogram frames, sill litter; `_G2_WIN_FACES` = 65 MEASURED openings on 25 kit modules, provenance under `_plans/glazing_probe/`), `r_glass_follow` (glass rides its wall through any later mover); `rc_glass` ladder rewritten; the kit's mirror curtain wall softened (`urban_building.GLASS_ROUGHNESS` 0.22) |
| the tower's mullion grid is PAINTED (0.3326 tiles/m, panes 1.645 / 1.362 x 2.89 m) and `SkyscraperFacade_B` is a 5 x 1 x 3 m glazed box with a ledge per storey | openings authored on the painted grid so the painted cage frames every hole; the cage/stripe already existed and only had to be left alone |
| duration, not magnitude, drives engineered-frame collapse (42 s vs 6 s record: -29 % capacity); magnitude is a poor severity proxy (Christchurch M6.2 > M7.1) | `disaster.duration_boost` (1.0 at M6.5 → 2.5 at M9+) multiplies the rc DG4/DG5 share in `level_for_intensity`; URM untouched |
| load time was NOT the archetype USD read: it was Hydra sync + collider cooking of ~300k prims, and the live pairs | `bake.py` merge at export (`BAKE_MERGE=on`, default in the driver; `both` writes a `_raw` twin): one mesh per (material x shading signature) for unique geometry, material dedup by network fingerprint, flat faceVarying normals → uniform, dead specs dropped; repeats (kit modules) left alone (crate already shares their points — merging them GREW files); PointInstancer measured and rejected (`_a_lump` corners are jittered, not prototypes). Two 200 m cities: env→ready 142 s → 78 s, 297k → 93k prims, first frame 16 s → 1.6 s, library 749 → 425 MB, 20 copies of a DG5 to first frame 5.6 s → 0.5 s; look unchanged (`_o_geom_diff`, control run) |

Still open after round 3 (also in Known gaps): live pairs are 62 % of the city
load (bake pairs as archetypes-of-two); `bld_apartment_tall_DG5` 33 MB
(1.4 M points); the 8.2 m curved corner bay of family 02 has no glazing
entry; DG1-DG2 glass is sub-pixel from the air (correct per the record,
unsatisfying); `_shard_field` at agent D's two call sites is round-2 code;
`dw_terrace`'s `storefront_a_3_5m` piece is 1.5 m off between the asset and
`ub.PIECES`; no cars/cordon under glass fall zones; timber deck plates on
the DG5 crown.


**Kit roof tiles are ZERO-THICKNESS QUADS.** `SM_MBuilding01_Roof` is
5 x 5 x 0.000 m, four points. Fractured, they come out as paper and PhysX
cooks no hull for a flat polygon, so the pieces never collide. Any roof a
recipe breaks or moves is first swapped for an authored 0.25 m slab at its
own world bbox (`_roof_box`), bound to the tile's own texture triplanar;
`_break_box_like` then fractures THAT. Masonry roofs use a 0.14 m timber
deck in `plank` mode instead.

**A kit roof tile's "texture" is the façade ATLAS.** `_roof_box` first bound
the tile's own base-colour map triplanar, and the office family's roofs came
up covered in windows in the assembled city (the tile samples one patch of
the atlas through UVs; a UV-less slab gets the whole sheet). Roof slabs are
plain concrete now — grey from the air is what a flat roof is.

**Kit façades are single-sided OPEN SHELLS, so their fragments are foil.**
0.2-0.75 m deep but no back face. A pile made only of them read as crumpled
paper on the first bench. The pile's MASS is now an authored heap of solid
chunks (`_heap`: a dome over the footprint for a collapse, a windrow along a
wall for a peel / parapet fall), the shell fragments are thinned with
`consume` (0.2-0.35, large-biased) and settle ONTO the heap. Heap heights
come from the research: H/3 for a total collapse, 1-2 m deep windrows
spreading 0.3-0.6 H for a shed wall, 0.5-1.5 m for a parapet.

**White plaster reads as paper; everything under a collapse is dusty.** The
first materials were a bright plaster and a flat beige "timber". Every flat
colour now sits at 0.45-0.68 luma (`materials()`), timber is the sawn-plank
TEXTURE from `planks.wood_material` darkened, and masonry rubble draws brick
/ mortar / plaster at 45/30/25.

**The AEC `Dirt.usda` is UV-space; authored meshes have no UVs.** The
lean-and-sink berm rendered as one flat brown mat. Soil is the megascans
`Soil_Mud.usda` (world-projected, the tornado scour's pack). Same rule as
`damage._pbr`: anything authored here must bind a triplanar material.

**Per-building prim names must be unique across recipes.** `_heap` numbered
its chunks per call, so a corner fan and a windrow on the same building
named their first chunk alike and the second `Define` raised `xformOp:
translate already exists`. `_uid(ctx)` is the per-building counter; use it
for every authored prim.

**Hole edges on module seams look machined — the user said so.** Removing
whole 4-5 m x 3 m modules leaves rectangular cut-outs with pristine
neighbours. `_break_split(judge)` fractures a piece and splits the cells by a
caller-supplied POSITION judge (`_edge_judge`, `_toward_judge`) with a
wandering line, so the bays adjoining a hole lose their near ends, slabs and
the roof lose a broken strip along the opening, `_disturb_interior` snaps or
topples partitions and litters the exposed floors, and `_spall` takes a
course off otherwise intact walls. Do this for every new recipe before
showing it.

**Do not Voronoi-fracture a whole roof or slab to break its edge.** The
first ragged-edge pass fractured the entire slab and kept the far cells
static: from above the intact roof was a crack mosaic in mixed materials.
`_split_strip` cuts the box into an untouched REMAINDER and a strip along
the failed side, and only the strip is fractured (`_ragged_slabs`,
`_corner_break`); the surviving strip cells are re-bound to the roof's own
material so the seam is the only tell.

**`infill_fail` must skip the ground band.** On the concrete families the
ground band is a 7 m glazed lobby of curved storefront pieces; targeting it
removed two shop windows and read as nothing. The masonry infill that drops
out of a frame is on storeys 1..N, so that is where the recipe now works,
with a hollow-block windrow under the dropped panels.

**Everything fitted into a storey has to come down with it.** The first
concrete pancake left the upper-storey partitions standing at their storey
heights over the stack — a grid of white panels in mid-air. `pancake`,
`masonry_collapse` and `soft_storey` (for the crushed storey) now break
partitions and drop props along with the slabs.

**A long bake process can stop fracturing, silently.** The first full bake
ran 16 styles in one process; from the 7th style on, every module came back
with a handful of fragments or none (DG3 6 loose where the same family gave
199 earlier), heaps still authored, no error anywhere, and the bench never
saw it because a bench fractures BEFORE it settles. The same styles fractured
normally in a fresh process, and a second 10-style run stayed healthy to the
end — so it is accumulated process state, not a recipe. Two defences now:
`fracture_mesh` prints `[fracture] EMPTY: ... (slice exceptions N, empty
slices N, culled N)` whenever a module yields nothing (the failure used to be
swallowed by `except Exception: frag = None` and `verbose=False`), and
`scene_gen/tools/bake_quake_by_style.sh` bakes one style per Isaac process
through the pane, merging the manifest each time. **Read the per-grade
`loose` counts in the pane as the bake runs**: a DG5 under ~800 on a
mid-rise is the symptom.

**A baked archetype's root must be an XFORM, or `apply_placements` drops
its transform.** `bake.export_object` used to write a `Scope` root
(`/Baked`). `scene_api._ref` (the suburb path) defines its holder prim as an
Xform before referencing, so the Scope never mattered there. The downtown
path goes through `scene_generator.apply_placements`, whose holder is
TYPELESS so the asset's own root type wins — a Scope is not Xformable, the
ops are skipped with a one-line `WARN: ... composed no Xformable prim`, and
all 46 buildings composed on top of each other at the origin (the plat
looked like one collapsed building in an empty city). `export_object` now
writes an Xform root; older bakes are fixed in place by setting the default
prim's type (`scene_gen/_scratch_eq/fix_root.py` did the quake library).

**Never `import` another launcher for its helpers.** Every
`*_launch_script.py` constructs `SimulationApp(...)` at module level, so
`import scene_launch_script as sl` inside `downtown_quake_launch_script`
built a second Kit app in the same process and segfaulted within a second
(`PyImport_ImportModuleLevelObject` at the top of the crash stack). Copy the
three helpers (`_remove_env_clutter`, `_disable_sky_sun`, `wait_for_stage`)
or move them to `utils/`; do not import the launcher.

**Pin the Kit log to a file NEWER than the launch, not to "the newest after
40 s".** The tornado skill's race, met again: a quick previous bake left its
banner in the newest log, the new process had not created its own yet, and
the driver reported two styles "done in 0 s" and moved on (killing the real
run with the next C-c). `bake_quake_by_style.sh` records the newest log
before sending the launch line and polls until the newest differs.

**Grep the Kit log for `Segmentation fault`, never `Segmentation`.** Kit
lists its OmniGraph node types at start-up and one is
`InstanceSegmentationLegacy`; a driver that waits on
`Traceback\|Aborted\|Segmentation` declares every run failed at second 11
and moves on. `bake_quake_by_style.sh` matches `Traceback (most recent`,
`Segmentation fault`, `Aborted (core`. Same trap for any harness that reads
the Kit log. (And do not `pkill -f <script name>` from a shell whose own
command line contains that name — it kills the shell; `pgrep -f
"name_[s]uffix"` with a bracketed character avoids self-matching.)

**Seed 7 builds no balconies.** `urban_building._plan_band` draws one
balcony mode per band and `none` is one of five; with the bench/bake seed
of 7 every balcony-capable style rolled it, so `balcony_fail` had nothing to
break and printed "no balconies on this style". Measured host-side over
seeds 1-12: 0 / 16 / 32 / 72 balconies depending on seed. The bake seed is 4.

**"Tips outward-and-down" is +theta about the LEFT perpendicular of the
outward vector, on every side.** `a = (-oy, ox)`, `a x o = -z`. The first
droop/balcony code flipped the sign on N/E and hinged those balconies UP
into the wall. `_droop_strip`, `r_balcony_fail` use +theta; `_tilt_*` use
-theta because there the pivot is on the far edge and the mass is inboard.

**Anything authored INTO a building must ride its rigid transform.** The
overturn moved `_everything(ctx)` (kit pieces + fit-out) and left the roof
REMAINDER slabs (`_roof_box` output, neither) floating where the roof had
been — a grey plate 20 m over an empty footprint. `r_overturn` carries
them explicitly (`carried`); check this list whenever a recipe authors a
new kind of piece before a transform.

**A hole-cutting recipe must not fracture the tiles it merely passes near.**
`roof_hole` with a 6 m margin fractured most of a 22 x 14 m roof and the
whole-roof crack mosaic came back; the margin is now ~2.6 m (a tile), and
`corner_fail` leaves kit roof tiles that do not reach the corner untouched
so `roof_hole` can still find them afterwards.

**Families without a parapet band (brick commercial, storefront, church).**
`parapet_fall` finds no parapet pieces and was a silent no-op. It now takes
the top course of the top storey with a high partial cut instead, over a
wider share of the side.

**Corner pieces classify to S or N, never E/W.** `_side_of` picks by nearest
wall line and a corner is equidistant; dict order breaks the tie. Harmless
as long as every recipe iterates `_els(role=...)` rather than assuming each
side owns its corners.

**Props must be seated by MEASURED bound.** Furniture pivots are arbitrary
(the residential kit is cm-authored, the office kit metres); `_prop`
references the asset, reads its world bound and shifts it so min-z sits on
the slab and the bound centre sits at the target.

**`DISASTER_TYPE=none` and `REGION_M` leak from `.env` too.** The suburb
missions set them and the container inherits them; a launcher that merges
`DISASTER_TYPE` into its spec overrides the way the drone launcher does
compiles the earthquake preset WITHOUT an earthquake, and the plat comes up
pristine with no error. `downtown_quake_launch_script` ignores any value but
`earthquake` and prints that it did.

**`generate_scene.check_duplicate_yaml_keys` scans EVERY preset.** It is
fatal on any duplicate mapping key in any preset or asset set, not only the
one being built — five suburb presets carried `house_gap_m` twice and took
the downtown launcher down with a ValueError before layout. Fixed by
deleting the first copy (PyYAML keeps the last, so that is the
behaviour-preserving edit); run the check host-side after touching a preset.

**`SCENE_CONFIG` leaks from `.env` into the container.** Pass it on the
relaunch line even for the bench (it reads nothing from it, but the
downtown launchers do).

**The v1 earthquake path is still in `build_city`.** `compile_earthquake`
emits `damaged_fraction` / `destroyed_fraction`, and `build_city` spends
them on the asset set's `damaged` / `destroyed` pools (Muyang's baked
destroyed_building_* shells, inherited from `urban_nucleus.yaml`) or, with
no pool, on tilting intact buildings at layout time. `urban_quake.yaml`
clears both pools and `downtown_earthquake.yaml` zeroes both fractions under
`overrides.disaster`; every grade, tilt and heap comes from
`quake.assemble`.

**250 m needs its own layout numbers.** `downtown.yaml` at 250 m gives 2
blocks and no district rings (`min_region_m: 300`). The earthquake preset
halves the block targets, drops the ring floor to 120 m, zeroes the rowhouse
mix (the kit has no party-wall units for the terrace morphology) and
lowers the tower heights — measured 6 blocks / ~36 buildings on the dry run.

---


## Round 3 additions (2026-08-27)

* **The kit walls are two skins with no rim, not zero-thickness.** The census
  (`scene_gen/tools/wall_thickness_census.sh`, notes `eq_round3_M.md`) found
  121 of 134 buildings "zero-wall-thickness" by boundary-edge / back-face
  tests — 16/16 kit styles, 105/118 of the urban_v2 drop — but agent T's ray
  probe showed the kit walls are DOUBLE-SIDED (0.14-0.70 m between sheets)
  with the rim missing, while MBuilding01/02/03 façades have no back face at
  all, and every roof tile is two triangles at 0.000 m.
* **The "triangles" were failed caps.** `slice_plane(cap=True)` cannot cap an
  open mesh: vtkCutter returns open polylines and the stripper fans each into
  one 2-3 m triangle, so every fragment carried two or three sheets. That —
  plus 3-D Voronoi seeds in a thin skin — was the round-2 "very triangular"
  look. `fracture.solidify` (cap small rims, extrude single skins to a real
  thickness, reveals round openings) runs before slicing (`EQ_SOLID=1`
  default, thickness table `T_SOLID_M` per type/role, glass stays a thin
  CLOSED plate); a cap that still fails retries uncapped instead of leaving
  the module whole.
* **`ValueError: No available triangulation engine!` was an import-order
  bug**, not the polygon and not (only) the install race: `trimesh.creation`
  resolves its engines once at import, and `ensure_deps` pip-installed
  `shapely` after trimesh was imported. `_reload_trimesh()` purges the
  modules after an install. Symptom: every EMPTY line in a bake log carries
  that message.
* **Wall solidification is cheap** (0.6-1.5x faces per module, +40 %
  fracture, settle 0.58x because solid fragments cook real convex hulls); a
  per-vertex normal offset is NOT usable on this kit (65 % of vertices sit on
  folds > 60 deg — mitring fired 0.9 m spikes).
* **`vtkContourTriangulator` can hang** (13+ min on a degenerate contour);
  it is opt-in (`FRACTURE_CAP=contour`), the default fan cap is safe.
* **Break patterns are material grids, not noise** (research §11): masonry
  cracks stair-step through mortar joints (riser = k x course, run = m x
  half-stretcher), fragments are bricks and brick clusters (60-80 % single /
  half bricks by count), out-of-plane failure is 1-4 rocking MACROBLOCKS;
  concrete fragments are prisms with faces normal to the surface, slabs hinge
  and hang as bay-sized rafts, spall = the cover thickness. Demolition rubble
  is blocky (FI < 15 %, EI < 25 %, ~0 blades).
* **Glass keeps its cage** (research §12 and `eq_round3_glass_recon_dump.md`):
  curtain-wall frame failure was 1 system in 371 at Christchurch; loss is a
  contiguous band on the racked storeys; cracked-but-retained exists only for
  annealed/laminated (tempered: crack drift = fallout drift); cracks are
  corner-rooted; no curtain-wall tower collapsed in ten reviewed events.
* **Duration, not magnitude, is the extra severity driver for engineered
  frames** (research §13): `duration_boost` (1.0 at M6.5 -> 2.5 at M9+)
  multiplies the rc DG4/DG5 share in the grade draw; URM is unchanged.

## Round 2 additions

* **The flat palette is LINEAR albedo** (`damage._pbr` -> `diffuse_color_constant`;
  screen ~= linear ** 0.42). `plaster` 0.66 rendered 0.84 — white. A dark
  debris colour is 0.02-0.09 linear; author new tints as `screen ** 2.38` and
  never bind bare `plaster` / `mortar` / `dark_concrete` to something in a heap.
* **A tint does nothing once a texture is bound**: OmniPBR's
  `diffuse_color_constant` is what the map REPLACES; `diffuse_tint` multiplies,
  `albedo_desaturation` desaturates (`/isaac-sim/kit/mdl/core/Base/OmniPBR.mdl`).
  `planks.wood_material(tint=...)` is therefore a no-op; `_c_look` sets the
  multiplier + desaturation after `_pbr`. Setting `albedo_brightness` too stacks
  (black mud).
* **Megascans packs bound BY REFERENCE sample UV space** on an authored mesh —
  one flat patch per fragment. Go through `_pbr(texture=...)` (`project_uvw`,
  world-space repeats per metre); `_C_TEX`/`_c_look` do.
* **`shrink` is a ratio and `roughen` is per fragment** — right for a pile,
  wrong for a partial break (long black cracks, a crazed survivor). Partial
  breaks use `fracture_split` / `roughen_field` / `inset` in metres.
* **A size cap on fragments needs a floor** or it culls the whole module, and
  `fracture_prim` then returns early WITHOUT deactivating the source (a DG5 came
  out as intact floor plates in mid-air). The cap never takes more than half.
* **A roof converted piecemeal has a straight material seam**: `_a_roofify`
  converts a mass's whole roof at once; `_roof_box` binds damaged asphalt, not
  pavement. Fragments need authored face normals or Hydra smooth-shades them
  (pillows, and a light line where a strip meets a `_box`).
* **A recipe that kills the only roof tile starves every later recipe** —
  `_roof_box` registers its slabs in `ctx["roof_slabs"]`, `r_roof_hole` falls
  back to them.
* **`_transform_prims` post-multiplies per occurrence** — dedupe the path list.
  A rebar tuft is authored in world space: place it AFTER the transform.
* **The overturn fell the wrong way**: `-angle` about the base-edge axis folded
  the block back over its own footprint; `+angle` lays it outward over `side`,
  which is what the crater, the shove and `_blocked`'s sweep assumed. The tilt
  pivots also double-rotated the yaw (`_outward` then `_to_world`) — invisible
  at yaw 0, wrong on a placed city building; `_SIDE_NORMAL` + `_c_tilt_matrix`.
* **A negative base under a fractional power is a `complex`, not an error**,
  and an `except TypeError` round the call turned it into silently-missing
  ground. No broad excepts round a helper you own.
* **`EQ_RECIPES=TILT|SETTLE|OV` raised `KeyError`** in the bench (only `DG*` was
  mapped through the ladder) while the runner still said DONE — fixed, and the
  runner now needs the DONE banner or `EXIT 0`.
* **Four agents editing one 6000-line module**: a bench can import a half-written
  function of someone else's; read the traceback's function name before
  believing it is yours, rerun. `faces += ...` inside a nested function rebinds
  the name (`UnboundLocalError`) — use `.extend()`.
* **`fracture_partial` can only take the TOP off a module**: a band clipping a
  module's foot removes 88 % of it. Skip modules a band only clips from below.
* **A hole-cutting recipe must not fracture the tiles it passes near** (one
  tile of margin), and a heap authored over the hole hides the hole.
* **A squat block cannot lean on anything** (`H sin(theta) - B (1 - cos(theta))`
  is 1.4 m for a 22 x 40 m block at 30 deg): `r_lean_on` says so and does nothing.

# Current knob values, and why

| knob | value | why |
|---|---|---|
| slab thickness | 0.30 timber / 0.22 concrete | research: 150-250 mm RC slabs; a joisted floor is deeper |
| wall inset for slabs | 0.55 m | inside the thickest kit module |
| column | 0.45 m at the module pitch | reads as a frame, not a forest |
| `FURNITURE_PER_100M2` | 1.6 | enough to see through an opening; ~30 props on a 22 x 18 x 5 block |
| shell fragment seeds | 7-12 per wall | fewer than fire (9-16); the heap carries the mass |
| shell `consume` | 0.22 peel / 0.30 collapse | thins the foil; the material is in the heap |
| heap height, total collapse | 0.28 H | FEMA 0.33 air-space factor, Amatrice LiDAR |
| heap spread, masonry | 0.2-0.34 H | research 0.4-0.7 H beyond the footprint at the toe |
| windrow, shed wall | 1.0-1.9 m deep, 0.22-0.4 H | research 1-2 m, 0.3-0.6 H |
| windrow, parapet | 0.5-1.1 m deep, 0.14 H | Christchurch: 1-4 m into the roadway |
| soft storey crush | 0.28-0.42 of the storey | the crushed storey is rubble, not air |
| soft storey lean | 2.5-6.5 deg | Northridge Meadows / Antakya photos |
| pancake pitch | 0.55-0.95 m per slab | ~1 m of debris per 3 m storey |
| `tilt_sink` | 8 deg / 1.0 m default; assembly draws 3-9 deg, 0.4-1.4 m | Adapazarı 0.004-1.6 m; Niigata 80 deg is a once-per-scene event |
| tilt gate | slender (H/B > 1.6) x 1.3, else x 0.6 | Adapazarı: only H/B > 2 blocks tilted significantly |
| `density` | 1900 kg/m3 | masonry / concrete, not the fire path's 420 timber |
| `max_speed` | 6.0 m/s | the 4.0 default clamps the wall-top fan |
| settle steps | 2200 | 1500 left 49 bodies moving on a masonry collapse; `highrise_04`'s DG5 still had 465 movers at 2200. A style row of 1.4-4.2k bodies solves in 2-8 min; the whole 96-archetype library was 19 min fracture + 77 min settle — `_plans/earthquake_timings.md` |
| grade cuts (URM) | 0.05/0.14/0.25/0.55/0.80 on `i * u` | worst-block mix 5/8/12/30/25/20 |

# Tuning — every knob, what it moves, and what it was calibrated against

## The three scales, and how they are coupled

    region_m  (plate size)  ──┐
    severity  (0..1)        ──┼─> compile_earthquake ─> disaster.field      (WHERE the shaking is strong)
    epicenter (x, y)        ──┘                        disaster.grade_scale (HOW hard the ladder is drawn)
                                                       disaster.soft_soil   (WHERE the ground fails)
                                                       disaster.dust        (how far the halo reaches)

Everything downstream reads those four, so `REGION_M=400 SEVERITY=0.6
EPICENTER=80,-30` on the launch line re-derives the whole scene. Nothing in
the earthquake path is in metres unless a preset pins it deliberately.

| what | formula (compile_disaster.compile_earthquake) | 250 m, sev 0.85 | 250 m, sev 0.5 | 800 m, sev 0.85 |
|---|---|---|---|---|
| full-intensity core radius | `max(w,h) * lerp(0.10, 0.28, sev)` | 63 m | 48 m | 202 m |
| falloff beyond it | `max(w,h) * 0.45` | 112 m | 112 m | 360 m |
| intensity at the far corners | `lerp(0.05, 0.35, sev)` | 0.31 | 0.20 | 0.31 |
| grade_scale (multiplies the field before the draw) | `min(1, lerp(0.55, 1.1, sev))` — 1.0 from severity ~0.8 | 1.0 | 0.83 | 1.0 |
| soft-soil ellipse | opposite quadrant to the epicentre, `0.36 w x 0.24 h` | 90 x 60 m | 90 x 60 m | 288 x 192 m |
| foundation rate inside it | `lerp(0.25, 0.85, sev)` | 0.76 | 0.55 | 0.76 |
| dust reach DG5 / DG4 (x H) | `lerp(0.7, 1.2, sev)` / `lerp(0.35, 0.6, sev)` | 1.1 / 0.56 | 0.95 / 0.48 | 1.1 / 0.56 |
| dust opacity max | `lerp(0.25, 0.45, sev)` | 0.42 | 0.35 | 0.42 |

Calibration: the ladder cuts in `quake_flow.level_for_intensity` reproduce
the worst-block mixes from the reconnaissance literature at intensity 1.0
(URM 5/8/12/30/25/20 % across DG0..DG5, RC 20/16/13/27/13/11, glass towers
mostly DG0-2); so severity 1.0 with a large core is "Antakya", 0.5 is a
district where most buildings are cracked and a handful collapsed. The
compiler's first cut (0.15-0.45 core, 0.55 falloff, 0.10-0.45 outside) put a
whole 250 m plate at intensity 1.0 and a quarter of it pancaked — that is why
the fractions above are what they are.

## Where each knob lives

**Launch line / `.env` (forwarded by `docker-compose.yaml`; both service blocks):**

| env | launcher | does |
|---|---|---|
| `SCENE_CONFIG` | all | preset; `downtown_earthquake` |
| `REGION_M` | city | plate size, `250`, `400x400` |
| `SEVERITY` | city | 0..1, drives everything in the table above |
| `EPICENTER` | city | `x,y` metres |
| `SOFT_SOIL` | city | `off`, or `x,y[,rx,ry[,rate]]` |
| `DISASTER_TYPE` | city | only `earthquake` is honoured (`.env` leaks `none`) |
| `QUAKE_SEED` | city | grade / foundation / ground draws (the layout seed is the preset's `seed`) |
| `QUAKE_TILT` | city | mild-lean chance (`disaster.debris.tilt_chance`), `0` disables |
| `QUAKE_GROUND` | city | `0` skips dust / fissures / boils / pounding |
| `ARCH_DIR` | city, bake | the archetype library |
| `SNAP_DIR` | all | captures, under `/isaac-sim/.nvidia-omniverse/logs/` |
| `ARCH_STYLES`, `ARCH_GRADES`, `ARCH_SEED`, `ARCH_VARIANTS`, `SETTLE_STEPS` | bake | which styles / levels, façade seed (4), variants per level, physics ceiling |
| `EQ_STYLE`, `EQ_RECIPES`, `EQ_SEED`, `EQ_SPACING`, `KEEP_PHYSICS` | bench | rows, columns, seed, pitch, leave bodies live |
| `MAGNITUDE` | city | e.g. `9.5` — defines severity and the field shape (see Round 2); `SEVERITY` still pins the plain path |
| `CITIES`, `CITY_SIZE_M`, `CITY_GAP_M`, `CITY_SEEDS` | city | a row of plates, one magnitude/severity each |
| `ASSET_SET` | city | `urban_quake` (kit only) or `urban_quake_v2` (+ monoliths and ruin towers) |
| `QUAKE_INTERACT`, `QUAKE_INTERACT_PAIRS`, `QUAKE_POUND_GAP` | city | live pairs (default 3, 0 = geometric only), total pairs, pounding gap threshold |
| `SETTLE_CULL_LEDGES` | bench, bake | `1` deletes bodies that came to rest on sills / cornices (19-142 per row) |
| `EQ_REFINE_MAX` | bench, bake | second-scale refinement cap for `fracture_split` (0 = single scale, 3-4x cheaper, straight edges again) |
| `EQ_NEIGHBOUR` | bench | `<style>,<gap_m>,<side>` — a pristine neighbour for lean_on / collapse_onto / pounding |
| `EQ_MILD_TILT`, `EQ_YAW` | bench | `<deg>,<sink>` runs the CITY's `_tilt_prim` path on a `pristine` column; build the column at a yaw |
| `ISAAC_SIM_HEADLESS`, `KEEP_OPEN` | all | off-screen; stay open after the captures |
| `EQ_SOLID` (1), `EQ_SOLID_N` (1.0; 0.85 = -16 % fracture time), `EQ_SOLID_CORE`, `EQ_SOLID_EDGE`, `FRACTURE_CAP` (fan / contour — contour can hang) | bench, bake | round 3: solid walls before slicing |
| `EQ_SLIVER` (1), `EQ_SLIVER_SEEDS`, `EQ_DUMP_FRAGS` (1 writes `frags.jsonl` for `test_break_shape.py`) | bench | round 3: needle rejection; shape acceptance dump |
| `BAKE_MERGE` (on / off / both), `BAKE_MERGE_REPEAT`, `BAKE_MERGE_UNIFORM_N` | bake | round 3: export-time consolidation (`both` keeps a `_raw` twin for `_o_geom_diff`) |
| `MAGNITUDE` → `duration_boost` (compiled) | city | round 3: rc DG4/DG5 share x 1.0-2.5 |

**Preset (`config/presets/downtown_earthquake.yaml`):** `severity`, `region_m`,
`epicenter`, `seed`, optional top-level `soft-soil: false | {center, rx_m,
ry_m, rate, angle_deg}`; under `overrides.disaster`: `field`, `soft_soil`,
`dust`, `grade_scale`, `debris.tilt_chance` pin any compiled value in metres.
The layout block sizes there are tuned for 250 m (`block_short_m` 52-72,
`block_long_m` 90-130, `min_region_m` 120); `REGION_M` up to 800 reuses them
(denser grid than `downtown.yaml`'s 800 m defaults — raise the block targets
for a full-scale city).

**Asset set (`config/asset_sets/urban_quake.yaml`):** which kit styles the
packer may place, per pool (`intact`, `midrise`, `tower`). Adding a style =
add its `bld_<style>_DG0.usd` here and bake it (`bake_quake_by_style.sh
<style>`).

**Per-building (`disaster/quake_flow.py`):**

| table / constant | does |
|---|---|
| `LADDER[type][grade]` | the recipe list per construction type and grade — the vocabulary |
| `FOUNDATION` | the three foundation levels' recipes |
| `level_for_intensity` cuts | the grade mix at a given intensity (per type) |
| `FAMILY_TYPE` | kit family -> `urm` / `rc` / `rc_glass` |
| `SLAB_T`, `WALL_INSET`, `COLUMN_W`, `FURNITURE`, `FURNITURE_PER_100M2` | the fit-out |
| recipe kwargs (`frac`, `sides`, `storeys`, `from_storey`, `tilt_deg`, `sink_m`, `angle_deg`, …) | per-recipe strength; every recipe takes its numbers as kwargs so a ladder entry can dial it |
| `_heap` args (`spread_frac`, `depth_m`), `RAFT_T`, `ROOF_T`, `HEAP_MIX`, `A_DEBRIS` | rubble geometry and its material mix per construction type |
| `EDGE_CELL_M` 0.32, `EDGE_MAX` 16, `REFINE_MAX` 12, `CHEW_OUT` 0.20, `CHEW_IN` 0.14, `CRACK_FRAC` 0.16, `GAP_*`, `ROUGH_M`, `STEP_H`/`STEP_V` | the two-scale break line: cell size along the break, refinement, chewed edges, radiating cracks, masonry course toothing |
| `B_ROOF_PLANT_BURIED` 0.55, `SCARS_MAX_PROJ`, `r_facade_scars(patches, cracks, crack_p, crumb_p)` | roof plant on total collapse; scar counts |
| `C_TILT_PIVOT_FRAC` 0.35, `C_DROP_REF` 1.8, `C_RISE_REF` 0.9, `C_CREST_M`, `C_REACH_M`, `C_GAP_W`/`C_GAP_D`, `C_BOIL_DROP`, `C_FISSURE_M`, `_C_TEX` | the ground response: pivot, crest and reach vs sink, gap size, boil threshold, ground looks |
| `r_lean_on(max_deg, crush_m, sink_frac, crush_storeys, band_storeys)`, `r_collapse_onto(storeys, spill_frac)`, `r_pounding(p_break, corner_m)`, `D_LEAN_MIN_INTENSITY` | pair recipes |
| `MONO_RUIN_MIN_H` 35, `MONO_HEAVY_DEG/SINK`, `MONO_MILD_P/DEG/SINK` (quake.py) | what a monolith may do |
| `T_SOLID_M` (urm wall 0.38, parapet 0.25, rc wall 0.20), `T_GLASS_M` 0.010 | round 3: solidified thickness per type/role |
| `P_BOND` / `P_STYLE_BOND` (course / stretcher / wythe per style), `P_ROUGH_M` 0.003, `BRICK_BLOCKY` 1.5, `BRICK_NMAX` 2.4, `PRISM_AR_MAX` 2.2, `SLIVER_MAX_DROP` 0.40 | round 3: the brick lattice and prism seeding |
| `G_GRADE`, `G_GLASS_KIND`, `G_SHOP_GRADE`, `_G_PEEL_BUDGET`, `G2_WIN_GRADE`, `G2_SASH_SHARE`, `G2_LEAN/BULGE`, `G2_FLOOR_BAND`, `G2_REMNANT_P/M`, `_G_CW_FACES` / `_G_SHOP_FACES` / `_G2_WIN_FACES` (measured pane tables) | round 3: glass per grade, per glass type; which faces are glazed |
| `GLASS_ROUGHNESS` 0.22 (`detail/urban_building.py`) | the kit's mirror curtain wall softened |
| `materials()` colours | the dusty palette |

**Scene (`disaster/quake.py`):** `_soft_soil` (reads `disaster.soft_soil`),
the OV gate (`slender > 1.5`, `inten > 0.4`, not a tower, `H <= 36`, clear
fall), `_blocked` sweep, `ground_effects` (reads `disaster.dust`; fissure count
2-4 per patch, boil count from patch area).

## Reading a run

The city banner prints the grade tally, the soft-soil patch and its
SETTLE/TILT/OV counts, the ground pass counts and `TIMING`. A scene that
prints DG5 > ~25 % of buildings has too large a core for its plate; one that
prints `SETTLE 0, TILT 0` has its patch on the collapsed side of town.

# Known gaps

Round 2 (2026-08-27), still open:

* (was wrongly written up as "zero-thickness quads fail to fracture") The 169
  `[fracture] EMPTY` modules in `commercial`'s round-2 bake all carry
  `first: ValueError: No available triangulation engine!` — trimesh's cap
  triangulation (`mapbox_earcut`) was not importable in THAT process: a fresh
  container, two bakes starting together, both racing `fracture.ensure_deps`'
  pip install. The pieces themselves are ordinary open-shell kit panels
  (4 triangles, 4 x 0.7 x 3 m) and a 454-face cornice, and they fracture fine
  once the engine imports. Symptom to recognise: every EMPTY line in a log
  with the same `first:`; cure: re-bake that style (`ensure_deps` now checks
  the engine and serialises the install).
* In a LIVE city pair only the leaner is rebuilt from the kit; the neighbour
  stays a reference, so its contact damage is authored bands, not fracture
  ("0 module(s) crushed on the neighbour" in the banner is expected).
* `r_collapse_onto` lands little of the thrown mass on the neighbour's roof.
* `urban_quake_v2`'s standalone monoliths and ruin towers render untextured.
* Pounding scars need a gap under `QUAKE_POUND_GAP` (1.5 m); the packer's
  `building_gap_m` 2.0 gives a closest pair of ~2-2.4 m, so cities print
  "0 pounding scar(s)" unless the preset goes lower.
* The downtown-quake path is NOT wired into the drone launcher
  (`scene_api.build_scene` is the suburb builder and the launcher's banner /
  annotations read suburb-only stats); `downtown_quake_launch_script.py` is
  the looking launcher.

- **Lateral spreading** (whole rafts of buildings translated 5-50 m over a
  torn road) is not built; fissures and boils on the soft-soil patch stand
  in for it.
- **Chimneys and gables** — the kits have neither; the suburb kit does.
- **Stair flights, standing water, crushed cars** — not built.
- **Repeated styles repeat their wreckage.** One bake per (style, level);
  `ARCH_VARIANTS=2` on the bake launcher and `quake._variants` already
  support `_vN` suffixes, so this is a bake-time cost, not code.
- **Overturning is rare by design** (one per scene, slender, clear fall,
  never a tower); a seed can draw none. Force one with a preset `soft_soil`
  centred on a slender block if a scene needs it.
- **The drone launcher** (`example_multi_drone_scene_import.py`) only knows
  the suburb `scene_api.build_scene` path; the downtown quake plat is a
  looking-launcher scene until that is wired.
