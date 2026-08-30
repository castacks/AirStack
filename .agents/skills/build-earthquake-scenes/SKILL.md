---
name: build-earthquake-scenes
description: Build or modify EARTHQUAKE-damaged URBAN scenes in scene_gen — the state of the pipeline after four rounds (2026-08-30), what it costs, and what gates it (building diversity, and now an Isaac verification pass). The kit-building fit-out, the per-construction-type damage ladder (URM wall peel / corner / parapet / masonry collapse on a brick lattice; RC infill / soft storey / pancake as prisms and rafts; curtain-wall bands with the mullion cage kept; lean-and-sink with a ground response; building-to-building pairs), solid walls before fracture, the merged archetype bake, the multi-city assembly by magnitude, the headless runner, every knob, and the bug catalogue. Round 4 (OFFLINE ONLY — host tests and Blender previews, no Isaac run yet) rebuilt the collapse pile as RUBBLE V2 — a heightfield mound with large elements and PointInstancer scatter, routed from `quake_flow` behind `EQ_RUBBLE` — fixed the soft-storey lean sign, cleared street furniture under a heap, taught `bake.py` to carry PointInstancers, and found + guarded a VTK segfault (`FRACTURE_VTK_GUARD`) that also unblocks fracturing sliced GAC/downtowncity pieces; `disaster/quake_sliced.py` (earthquake on sliced whole-asset buildings) is written but ON HOLD — that destruction/assembly scope moved to the fire session. Read before touching disaster/quake_flow.py, disaster/quake.py, disaster/quake_rubble.py, disaster/quake_rubble_usd.py, disaster/fracture.py, disaster/bake.py, the eq_building_bench / bake_quake_archetypes / downtown_quake launchers, the urban_quake asset sets or the downtown_earthquake preset. The wildfire and tornado skills are prerequisites.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Build Earthquake Scenes (urban building damage)

## Status on 2026-08-30 — read this first

**What rounds 1-3 built and verified IN ISAAC** (unchanged by round 4, still
the deliverable if the user asks for a scene today): 16 kit building styles
(5 façade families + storefront / civic / church) × 9 damage levels (EMS-98
DG0-DG5 + SETTLE / TILT / OV) baked as 144 merged archetypes
(`omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetype/`, 631 MB, 81 min two-at-a-time), and a
city launcher that lays out any number of plates, draws every building's
grade from a magnitude-derived shaking field, swaps in the archetypes,
leans / sinks buildings on a soft-soil patch, runs a few building pairs LIVE
(lean-on, collapse-onto), and authors the ground (heave, gaps, fissures,
boils, dust). The last accepted scene is two 200 m cities with a 100 m gap,
M9.5 west / M5.5 east: 35 buildings DG5=14 DG4=3 DG3=5 LEAN=3 TILT=2 versus 21
buildings DG0=9 DG1=4 DG2=6 DG3=2; **env→ready 74 s** (48.7 s of it the three
live pairs), 95,293 stage prims, first frame 2.0 s, 75 fps at 1280x720.
Captures: `/home/krrishjain/docker/isaac-sim/logs/final_r3/` (`plat_top.png`,
`c0_*` / `c1_*` corner views, `b0..b9_*` the ten worst buildings). Every
capture directory named in this file lives under
`/home/krrishjain/docker/isaac-sim/logs/`.

That scene's DG4/DG5 heaps are what round 4 replaced: `final_r3/b0_
apartment_tall_DG5_obl.png` and `b3_highrise_04_DG5_obl.png` show street
trees standing untouched THROUGH the rubble and a pile of same-sized toy
blocks with a hard dome edge at the sidewalk. See "Rubble v2 (round 4)"
below for the fix and "BUILDING DIVERSITY" further down for the gate that
fix does not touch.

**What round 4 built — OFFLINE ONLY, no Isaac run yet.** Say this plainly so
nobody mistakes a numpy unit test for a verified look: every number in the
"Rubble v2" section below comes from host-side tests (`pytest` /
`python3 -m py_compile` against pure-python/`usd-core` modules, no Kit) and
headless Blender/Cycles renders (`tools/rubble_preview.py`), never from
Isaac Sim. Seven work packages landed this way: the rubble planner
(`quake_rubble.py`), the USD emitter (`quake_rubble_usd.py`), the routing of
six `quake_flow` collapse recipes onto it behind `EQ_RUBBLE=v2`, a soft-
storey mechanics fix (the old pivot leaned buildings the wrong way), a heap-
clearance pass that removes/tips/leans/buries street furniture and cars
under a DG4/DG5 pile, `PointInstancer` support in `bake.export_object` (so
the scatter survives the archetype bake), and a VTK segfault fix
(`FRACTURE_VTK_GUARD`) found while chasing a crash on sliced GAC geometry.
**The next step, when the user gives the go, is opening Isaac**: does the
mound read against a real kit stub, do panels sit right on a fan, does the
soft-storey collar look like a collapse and not a smear, what is the actual
prim/body budget per archetype — then re-bake the 16 kit styles' 144
archetypes with `EQ_RUBBLE=v2`. Nothing above has been seen through Kit's
own renderer.

**Scope rule (user, 2026-08-30): earthquake damage to GAC / downtowncity
whole-asset buildings is NOT this session's work.** "Don't work on
destroying/assembling GAC or downtownwest. I have that already being done
for the urban fire scene. You will be able to take code from there and also
read the documented skill." Concretely: `disaster/quake_sliced.py` (the
earthquake ladder for `gac_storey_slice`-cut buildings — removal-on-the-grid
+ rigid displacement, since a sliced piece cannot be fractured, see below)
and its 25 tests are written and pass, but **ON HOLD — no further work**.
When the fire session's GAC/downtowncity slicing, placement and bake code
and its skill (`build-urban-fire-scenes`) land, reuse that code rather than
finishing `quake_sliced.py`'s own ladder from here. The `ARCH_SOURCE=gac`
bake-launcher path and its asset set were written and then reverted the same
day for this reason — do not resurrect them without re-reading the scope
note in `_plans/earthquake_round4_plan.md`.

**What gates the NEXT round — two independent things:**

1. **The Isaac verification above** (rubble v2 has never been rendered by
   Kit) — do this before touching anything else in the earthquake path.
2. **BUILDING DIVERSITY**, unchanged since round 3 and NOT addressed this
   round. The user's verdict on the last Isaac scene: "the buildings look
   good but the diversity in number of buildings just isn't there — if we
   don't have diversity in terms of more buildings it's pointless." 16
   styles from 5 façade textures repeat 2-3× per block. The two levers, in
   order of payoff:
   1. **Damage the monoliths.** `asset_sets/urban_v2.yaml` (118 buildings,
      173 assets) is the diversity — 85 `selected_citydemo` towers/midrises
      with their own façade textures — but the earthquake path only ever
      used 5 of them (the untextured `standalone/intact` set, as rigid
      lean/sink). Round 3 built what a monolith needs: `solidify` gives a
      shell wall real thickness, `brick`/`prism` seeding cuts it on a
      material grid; a fitted interior from the bbox would let the same
      recipes run on a monolith. Cost estimate from round-3 timings: ~5-8
      min per model × 9 levels two-at-a-time ≈ 5-6 h of bake, 2-3 GB of
      merged archetypes.
   2. **More kit styles.** `detail/urban_building.py`'s grammar can emit
      40-60 silhouettes — more counts, still 5 façade textures.

**Round 4's last loose end, closed**: the v4 Blender previews found a black
z-fighting band where the mound/apron rim touched the ground plate, and the
textured FAB debris clusters (`brick_debris_pile` etc.) read pale from directly
overhead. Both halves of the rim fix landed (`MOUND_LIP_M` 0.008 / `APRON_LIP_M`
0.012 in `quake_rubble.py` — nothing is authored exactly at z0 — and the
preview ground plane at -0.02 in `rubble_preview.py`); the v5 renders
(`~/scorch_previews/rubble_r4/v5/`) show the band gone in every view. The FAB
clusters now carry a per-asset copy material with `diffuse_tint` 0.85 and still
read pale from the air: their scans hold near-white flecks that a
multiplicative tint cannot compress — judge them under RTX in Isaac before
chasing (a 0.6-0.7 tint or a highlight roll-off is the lever).

**Minor details still wrong in the current (round-3) Isaac cities** (the
user: "a later issue"): a mullion frame hovering off the tower glass at the
epicentre camera (`final_r3/c0_ne_obl.png`), punched-window interiors reading
cartoonish from 40 m, timber deck plates on DG5 crowns, the family-02 curved
corner bay never loses glass, `apartment_tall_DG5` is 34 MB. Full list in
"Known gaps".

**Do not** start Isaac or containers for this work without the user's go;
design and code can proceed offline (`ast.parse`, the host-side tests,
`scene_gen/tools/_t_pxr.sh` for pxr-only probes needs the container).

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

## The pipeline and its files

    downtown layout -> [bake once per (style x level), merged] -> assemble (swap by field) -> foundation pass -> live pairs -> ground -> captures

| piece | file |
|---|---|
| per-building damage: fit-out, materials, every recipe, the ladders, the grade draw | `scene_gen/disaster/quake_flow.py` (~7k lines; owner prefixes `_a_ _b_ _c_ _d_` round 2, `_t_ _p_ _g_ _g2_` round 3) |
| scene assembly: field → grade → archetype swap, soft-soil foundation pass, mild lean + ground, monolith pass, live pairs, ground effects | `scene_gen/disaster/quake.py` |
| fracture: `solidify`, seeding modes (`uniform`, `char`, `plank`, `brick`, `prism`), `fracture_split` two-scale, `slice_plane` cap/retry, `ensure_deps` + `_reload_trimesh` | `scene_gen/disaster/fracture.py` |
| physics settle (per-body velocity, density, ledge cull) | `scene_gen/disaster/settle.py` |
| by-value archetype export with export-time merge (`BAKE_MERGE`), round-4 `PointInstancer` carry (`_author_instancer`, `_copy_prototype_tree`) | `scene_gen/disaster/bake.py` |
| round-4 rubble PLANNER: mound heightfield, large elements, instance sets — pure numpy/stdlib, no `pxr` | `scene_gen/disaster/quake_rubble.py` |
| round-4 rubble EMITTER: the only rubble module that imports `pxr`, materials, `PointInstancer` wiring | `scene_gen/disaster/quake_rubble_usd.py` |
| earthquake on SLICED whole-asset (GAC/downtowncity) buildings — written, 25 tests pass, **ON HOLD** (scope moved to the fire session, 2026-08-30) | `scene_gen/disaster/quake_sliced.py` |
| magnitude → severity / field / soft soil / dust / grade_scale / duration_boost | `scene_gen/compile_disaster.py` (`compile_earthquake`, `magnitude_to_severity`) |
| single-building bench (one style per row, one recipe or grade per column, 7 review cameras) | `simulation/isaac-sim/launch_scripts/eq_building_bench_launch_script.py` |
| archetype bake, one style per process | `bake_quake_archetypes_launch_script.py` |
| the city / multi-city looking launcher (no drone) | `downtown_quake_launch_script.py` |
| headless serialised runner (2 GPU slots), bake driver | `scene_gen/tools/eq_bench.sh`, `scene_gen/tools/bake_quake_headless.sh` |
| offline tests (no Isaac) | `scene_gen/tools/test_break_lines.py`, `test_break_shape.py`, `_o_merge_check.py`, `_g2_check_table.py`, `_c_offline/` |
| round-4 rubble tests (no Isaac; `usd-core` for the emitter/bake ones) | `scene_gen/tests/test_quake_rubble.py`, `test_quake_rubble_usd.py`, `test_bake_instancer.py`, `test_quake_heap_clearance.py`, `test_quake_flow_rubble_routing.py`, `test_quake_sliced.py` (on hold) |
| round-4 rubble preview / asset tools | `scene_gen/tools/rubble_preview.py` (Blender/Cycles look check), `nucleus_fetch.py` (Nucleus → local mirror, run inside the container), `make_tileable.py` (debris atlas → seamless ground tile), `_vtk_shell_probe.py` (VTK segfault repro, per-stage subprocess) |
| census and probes (standalone pxr, no Kit) | `scene_gen/tools/wall_thickness_census.sh`, `_t_pxr.sh`, `_t_shell_probe.py`, `_g_*`/`_g2_*` glazing probes, `_o_usd_stat.py`, `_o_geom_diff.py` |
| building pools = pristine bakes (+ opt-in monoliths) | `scene_gen/config/asset_sets/urban_quake.yaml`, `urban_quake_v2.yaml` |
| the preset (downtown.yaml shrunk to 250 m, earthquake on, no metres — everything compiled) | `scene_gen/config/presets/downtown_earthquake.yaml` |
| research | `scene_gen/_plans/earthquake_research.md` (§1-§10 round 1; §11 break geometry by material, §12 glass and curtain walls, §13 magnitude classes / duration), `eq_round3_glass_recon_dump.md` (ten-event field evidence, provenance-tagged), `eq_round4_rubble_research.md` (WP A — mound morphology: crown/height, run-out, repose, the standing stub, the windrow/fan taper, 20 sources tagged M/E/S) |
| plan, timings, agent notes | `scene_gen/_plans/earthquake_plan.md`, `earthquake_timings.md`, `eq_round2_{A,B,C,D}.md`, `eq_round3_{M,R,T,P,G,G2,O}.md`, `earthquake_round4_plan.md` (the round-4 brief, API contract, scope change, per-package progress log) |

## How to run (headless, from the host)

    # bench: iterate here first — everything is fractured and settled live
    scene_gen/tools/eq_bench.sh <snap> EQ_STYLE=commercial EQ_RECIPES=DG3,DG5,out_of_plane EQ_SEED=4 SETTLE_STEPS=3000
    # bake the library (16 styles, two Isaac processes at a time, merge on)
    SETTLE_STEPS=3000 EQ_SOLID_N=0.85 scene_gen/tools/bake_quake_headless.sh            # or: ... commercial tower
    # the city / two cities — alone on the GPU
    GPU_SLOTS=1 LAUNCHER=downtown_quake_launch_script.py scene_gen/tools/eq_bench.sh <snap> \
        CITIES=M9.5,M5.5 CITY_SIZE_M=200 CITY_GAP_M=100 QUAKE_SEED=8 QUAKE_INTERACT=3
    # on screen instead (the tmux pane; stays open): same env, `docker exec isaac-sim tmux send-keys -t isaac '...' ENTER`

`EQ_RECIPES` entries: a grade (`DG1..DG5`, `SETTLE`, `TILT`, `OV`, looked up
in `quake_flow.LADDER` for the style's type), a bare recipe name with its
defaults, or `pristine`. Every column gets seven camera prims under
`/World/ReviewCams` (top, four obliques, street at 35 m, close at 2 m) and a
PNG each; the runner leaves captures in `~/docker/isaac-sim/logs/<snap>/`
and the launcher's stdout in `<snap>.log` beside it (`docker logs` is empty).


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
| 2 | `lean_on`, `collapse_onto`, `pounding`, `_d_party_collapse` | a building pair: bearing failure to the contact angle; upper storeys thrown onto a lower roof; slab-level spall bands; a terrace unit collapsing beside its party wall | city `_d_interactions`; bench `EQ_NEIGHBOUR` |
| 3 | `curtain_wall`, `storefront_glass`, `window_glass`, `glass_follow` | curtain-wall bands with the cage kept, Zhao's in-plane ladder for shopfronts and punched windows, glass riding its wall through later movers | rc_glass DG1-5; every ladder's glass slot |

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

`duration_boost` (round 3, research §13): a long record (M8+, minutes of
shaking) lowers the collapse capacity of engineered frames — the compiled
boost (1.0 at M6.5 → 2.5 at M9+) multiplies the rc / rc_glass DG4-DG5 share
in the draw; URM is left alone.

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

## Solid walls, and breaks on the material grid (round 3)

Every kit piece goes through `fracture.solidify` before it is sliced
(`EQ_SOLID=1`): a rim is capped or a single skin is extruded to the type's
thickness (`T_SOLID_M`: URM wall 0.38, parapet 0.25, RC wall 0.20; glass
stays a 10 mm CLOSED plate), window openings get reveals, and the new inner
faces are bound to a brick-core / dark-concrete material — so every chunk
reads as masonry from any angle. Then the seeding comes from the plane of
weakness, not from noise: `mode="brick"` (a running-bond lattice at the
course / stretcher / wythe pitch of `P_BOND`, clusters by `keep`, jitter ≤
0.1 pitch, no roughening), `mode="prism"` (2-D seeds on the surface extruded
through the thickness — never a 3-D seed in a member thinner than the cell
pitch), staircase judges quantised to the bond (`_p_staircase`: risers k ×
course, runs m × half-stretcher), 1-4 rocking MACROBLOCKS for out-of-plane
failure, lintels / quoins emitted as monoliths, needle rejection.
`scene_gen/tools/test_break_shape.py` scores a bench's fragments (oblique
face area, elongation, blades, rafts > 2 m, equidimensional share, flakiness)
against research §11.

## Glass (round 3)

`r_curtain_wall`: the storeys that racked (soft storey / podium transition /
lower third; top storey for a roof push; one elevation preferentially) lose
glass in CONTIGUOUS bands with survivors inside and strays outside; mullions,
transoms, spandrels and gaskets stay; per-grade "out" fractions 0-1 / 2-5 /
10-20 / 25-40 / 40-55 %; cracked panes keep a corner-rooted crack (annealed /
laminated only — tempered goes straight to an empty frame); dice piles ≈
1.2 × panel area under the band; one merged mesh per (kind, side, storey), ≤ 5
materials, no bodies. The tower's mullion grid is PAINTED (0.3326 tiles/m;
panes 1.645 / 1.362 × 2.89 m), so openings are authored on that grid and the
painted cage frames every hole. `r_storefront_glass` / `r_window_glass` do the
in-plane ladder on shopfronts and the 65 measured punched windows of the
brick families (`_G2_WIN_FACES`; probe outputs under `_plans/glazing_probe/`);
`r_glass_follow` carries authored glass through any later mover. The kit's
mirror curtain wall is softened in `urban_building.apply_glass_tint`
(`GLASS_ROUGHNESS` 0.22) — its perfect reflections read as damage.

---

# Rubble v2 (round 4)

**OFFLINE ONLY as of 2026-08-30** — every number below is a host test or a
Blender render, never an Isaac frame. Design and code: `_plans/
earthquake_round4_plan.md` (the brief, the API contract, the scope change,
the per-package progress log — read it before changing anything in this
section). Research: `_plans/eq_round4_rubble_research.md` (WP A, 20 sources,
tagged **M**/**E**/**S** for measured / estimated / synthesised — cited
below by tag). Modules: `disaster/quake_rubble.py` (planner, pure numpy/
stdlib, no `pxr`), `disaster/quake_rubble_usd.py` (emitter, the only rubble
module that imports `pxr`), routed from `disaster/quake_flow.py::_rubble`.

## Why the round-3 heap had to go

`quake_flow._heap` authors a pile's mass as 1,500-5,600 `_a_lump` boxes
(0.28-1.7 m, jittered corners, flat tints from `HEAP_MIX`) plus a "skin" of
0.15-0.5 m dust-tinted lumps. Measured on `final_r3/b0_apartment_tall_DG5_
obl.png` and `b3_highrise_04_DG5_obl.png`: every object in the pile has the
same character — a discrete convex block — so from 40 m it reads as a heap
of toy blocks, not rubble:

* no continuous FINES surface (research §7.1.7/§7.2.6: mortar sand and dust
  "render as the dust film, not as pieces");
* no large recognisable elements (§7.2.6: "the pile's silhouette is set by
  3-8 large rafts, not by thousands of cells"; `max_piece_m` capped every
  fragment at 1.2-2.6 m);
* the same brick-cube skin under an RC highrise as under a brownstone;
* a smooth dome with a hard edge at the sidewalk;
* street trees standing untouched through the pile.

## The six-layer design

Bottom-up, per collapsed mass or shed wall:

| layer | what | prims | physics |
|---|---|---|---|
| 1 mound | ONE heightfield mesh: dome / windrow / fan footprint, ~35° repose on the flanks, flattened crown, fbm relief, toe apron; world-projected rubble texture per type with the dust tint | 1 | static triangle collider |
| 2 large elements | RC: slab rafts, column stubs, rebar tangles, 1 corrugated sheet, 1-3 wall PANELS (the building's own kit modules kept whole, half-buried). URM: lintel/quoin monoliths, timber joists, macroblock panels, no concrete rafts | ≤ 20 | none — posed on the mound surface, sunk by `bury` |
| 3 clusters | textured FAB spreads (`concrete_debris_elements`, `huge_concrete_rubble_pile`, `brick_debris_pile`, sidewalk/paving pieces at the toe) sunk into the flanks | PointInstancer, 3-10 instances | none |
| 4 scatter | `chunk_01..09` / `lump_01..06` (flakes), density falling crown → toe, a run-out beyond the toe | 2 PointInstancers | none |
| 5 shell fragments | the existing `_break` output, thinned harder | as today × ~0.6 | bodies (as today) |
| 6 dust film | existing `quake.ground_effects` dust | — | unchanged this round |

Budget target: ≤ 1 mound + 20 large + 4 instancers (≤ 900 instances) + shell
fragments per DG5 archetype — prims down from ~7,000 authored per pile to
under 2,000 before the bake merge, bodies unchanged or fewer.

## The debris catalogue

`quake_rubble.CATALOGUE` — every entry gives `url` (relative to
`RUBBLE_ASSET_ROOT`), native `size` (metres, Z-up, footprint centred on the
origin, base at z=0), `tris`, `kind`, `textured`, `material`.

| kind | assets | size range | tris | textured | material | Nucleus path |
|---|---|---|---|---|---|---|
| raft | `slab_01..12` | 1.3-4.5 m footprint, 0.19-1.2 m thick | 568-9,586 | no | concrete | `standalone/debris/pieces/slab_NN/` |
| chunk | `chunk_01..09` | ~0.7-1.0 m boxes | 74-76 | no | concrete | `standalone/debris/pieces/chunk_NN/` |
| flake | `lump_01..06` | 0.15-0.37 m | 522-4,088 | no | concrete | `standalone/debris/pieces/lump_NN/` |
| rebar | `rebar_01..04` | ~4 m × 1.2-2.75 m × 0.16-0.23 m tangles | 2,494-3,754 | no | steel | `standalone/debris/pieces/rebar_NN/` |
| sheet | `sheet_01..03` | ~4 m × 1.8-2.9 m corrugated | 3,181-9,209 | no | steel | `standalone/debris/pieces/sheet_NN/` |
| cluster/spread | `brick_debris_pile` (6.1×4.9×1.2 m, 58k tris; `_hp` twin 189k — flagged, never instanced), `concrete_debris_elements` (3.5×2.6×0.4 m, 96.8k), `concrete_slabs`, `huge_concrete_rubble_pile` (8.0×7.7×1.5 m, 71.3k), `concrete_rubble_pile` (725 tris, cheap Quixel), `rocky_ground` (earth, 7.5k) | building-scale | 0.7k-190k | yes | brick/concrete/earth | `concrete_rubble_debris/split/<dir>/`, `standalone/debris/piles/<dir>/` |
| street/toe | `concrete_sidewalk_elements`, `cracked_paving_slabs`, `lamppost_block[_v2]` | 0.16-1.8 m | 20.5k-29.4k | yes | concrete | `concrete_rubble_debris/split/<dir>/` |

`raft`/`rebar`/`sheet` are drawn ONCE per large-element slot and authored as
individual references (a 2-6 m raft has to be sized and counted, not left to
an instance draw); `chunk`/`flake`/`cluster`/`toe` feed `PointInstancer`s.
`*_hp` twins are the same prop at 2-3× the triangle count — never instanced
by mistake, but available if a hero shot needs one raft close-up.

**Local mirror**: `tools/nucleus_fetch.py` runs INSIDE the isaac-sim
container on bare `pxr` + `omni.client` (`scene_gen/tools/_t_pxr.sh
scene_gen/tools/nucleus_fetch.py`, no Kit) and copies every catalogue dir's
`.usdc` + `textures/*` (skipping `*_hp` siblings) into `scene_gen/assets/`
under the container's repo mount — 160 MB, gitignored. `RUBBLE_ASSET_ROOT`
(env, read by `quake_rubble_usd.ASSET_ROOT`) points the emitter at that local
tree instead of `omniverse://...` for previews and host-side tests.

## The API contract

```
quake_rubble.plan_pile(m, btype, rng, kind="dome"|"windrow"|"fan", ...)
    -> {"mound", "apron", "large": [...], "instances": {set: {...}}, "stats"}
quake_rubble_usd.author(stage, parent, plan, mats, tag, uid) -> USD prims
```

`m` is the same mass dict `quake_flow` builds (`cx, cy, W, D, yaw, z0, top,
levels`). **`pos` in every `"large"` entry and every instance is FINAL**: the
world position of the element's own bottom-centre origin, already sunk by
`bury` × its ROTATED thickness below the mound surface
(`rotated_extent(size, scale, rot)` measures the tilted bounding box, not the
native one — a column stub lying at 60-90° has most of its "height" in x/y
once rotated). The emitter translates to `pos` exactly and never re-applies
`bury` — see "Iteration history" below for the bug this fixes. `look` is a
per-entry / per-instance-set material tag (`"brick"`, `"concrete"`, `"rust"`,
`"stone"`, `"timber"`, `"dust"`, or `None` to keep the asset's own referenced
material) resolved by `quake_rubble_usd._material_for_look` and bound with
`bindingStrength=strongerThanDescendants` so an override actually wins over a
referenced asset's own material. Prototypes are authored as CHILDREN of
their instancer (`<instancer>/Prototypes/<name>`) — the same "a
`PointInstancer`'s children are not drawn directly" convention `bake.py`
already depends on (see below).

## The research constants, and where they came from

| constant | value | tag | memo section |
|---|---|---|---|
| `REPOSE_DEG` / `REPOSE_DESIGN_DEG` | 35° flanks (design target 32°, headroom for noise) | **E** | sec1c |
| `APRON_REPOSE_DEG` | 31° (pure-fines toe sits lower than the flank) | **E**/**S** | sec1c |
| `CROWN_FRAC` | urm 0.28, rc 0.30, rc_glass 0.12 | **S**/already-established | sec1a |
| `CROWN_CAP_M` | 12 m — no measurement exists above ~11 m building height | **S**, low confidence | sec1a |
| run-out, fall/street side | Moya et al. 2020, 851 LiDAR-measured collapsed buildings: 0.65×H at H≤4m → 0.40×H at H=12m → 0.35×H at H≥20m, linear between; `RUNOUT_CAP_FALL_M`=10 | **M**→**E** | sec1b |
| run-out, blind/party-wall side | flat 0.10×H, `RUNOUT_CAP_BLIND_M`=3, floor 1.5 m | **S**, anchored on **M** | sec1b |
| URM run-out correction | ×0.85 (Moya's fit is wood-frame — a more-complete-toppling upper bound) | **E** | sec1b |
| windrow/fan reach | ≈1.0× the FALLEN ELEMENT's own height (fire-service 90°-collapse-zone rule), not a fraction of the whole building | **E**/**M** | sec5b |
| `CHUNK_DENSITY` | crown 2.5, mid 1.2, toe 0.35 per m², cap 4,000 | round-4 review (was 1.2/0.3/600 — "read as a dune") | — |
| `BURY_FRAC` | raft 0.10-0.30, chunk/cluster 0.30-0.60, rebar 0.50-0.80 | **S** | sec3b |
| stub retained after total collapse | ~0.4-0.6 of buildings (`STUB_KEEP_P` in the research memo) | **S**, anchored on Galvis et al. 2020's 57%/43% Mexico City soft-storey split | sec5a |
| stub height when present | 0-2.7 m absolute (Noto Peninsula D5 detection threshold) | **S**, anchored on **M** | sec5a |

**`STUB_KEEP_P` is a research recommendation, not a wired knob**: no code in
`quake_rubble.py` or `quake_flow.py` gates stub survival by a probability.
`r_masonry_collapse(keep_stub=True)` always keeps a ground-storey stub (the
default never flips), drawing a per-wall `stub_frac = U(0.25, 0.62)` of the
storey height. If a future round wants "40-60% of buildings keep no stub at
all," that gate has to be added — it is not there today.

## Budgets, measured

| archetype (dome) | crown | large | instances | mound tris | apron tris |
|---|---|---|---|---|---|
| rc 30×30×55 | 10.5 m | 16 (rafts/columns/rebar/1 sheet) | ~2,890 (2,490 chunks + 396 flakes + cluster/toe) | 20.7k | 8.8k |
| urm 22×18×15 | 4.3 m | — | 982 chunks + 410 flakes | 9.5k | — |

Compare to round-3 `_heap` on the same piles: 1,500-5,600 authored boxes.
The mound+large+instancer representation is O(20) individually-authored
prims plus 2-4 `PointInstancer`s, not thousands of `_a_lump` boxes — the
prim-count win the round-4 budget was written around.

## How `quake_flow._rubble` routes each recipe

`EQ_RUBBLE` (env, default `v2`; `v1` reproduces the old `_heap` byte-for-
byte — `fire_collapse.py`, another live session, still calls `_heap`
directly and is untouched). `_rubble(ctx, m, kind, ...)` is the one call
site every routed recipe goes through:

| recipe | kind | stub_h_m | panels | elem_h_m / depth_m | shell consume / cap |
|---|---|---|---|---|---|
| `r_masonry_collapse` | dome | mean kept-stub height | 1-2 storey-1 window/façade modules (`_pick_opening_panels`, v2 only) | crown = 0.28×H, spread 0.2-0.34 | 0.8 / 0.9 m (v1: 0.66 / 1.2 m) |
| `r_pancake` | dome, `budget={"n_large":4}` | 0 (nothing survives standing) | none — the re-authored slab STACK carries the rest | crown = pitch × n_lv × 0.85, spread 0.3-0.45 | unchanged (stack authored separately) |
| `r_out_of_plane` | fan | — | the kept macroblocks | elem_h_m = top − z_fail (the fallen run's own height) | consume 0.22, unchanged |
| `r_parapet_fall` | windrow | — | — | depth_m fixed 0.45 (v1 drew U(0.5,1.1)); elem_h_m = tallest chosen course | unchanged |
| `_corner_break` / `r_corner_fail` | fan | — | — | depth_m fixed 0.8 (v1 drew U(0.6,1.2)); elem_h_m = `top - levels[k0]`, the storeys the corner dropped | unchanged |
| `r_soft_storey` collar | windrow, all 4 sides, authored at `z_lo` via `_pile_mass` | — | — | depth_m = U(0.5,0.9) + crush×0.25; elem_h_m = h_st | consume 0.45, unchanged |

Everywhere `_a_lump` skins are dropped; `_p_lintels`'s own boxes only run
under `v1` (v2's planner emits its own lintels/quoins/sills).

## Soft-storey mechanics fix

The ORIGINAL `r_soft_storey` pivoted on the LEAN side's own base edge with
`sign = -1.0` — the bug: this leans the block AWAY from the side it names
and pushes the far base edge `span·sin(lean)` into the storey below (2.9 m
on a 30 m frame at 5.5°). `_soft_storey_geometry` replaces it with two
mechanisms, drawn 60/40:

* **Differential crush (60%)**: `r_lean = U(0.15, 0.40)·h_st` on the lean
  side, `r_far = min(0.95·h_st, r_lean + span·sin(lean_drawn))` on the far
  side, `lean = asin((r_far − r_lean) / span)` — the far base edge pivots at
  `z_lo + r_far`, the lean side drops to its own (lower) residual. Measured:
  a 30 m span gives `r_lean=0.64 m, r_far=3.04 m, lean=4.6°`; a 12 m span
  gives `lean=5.5°`.
* **Sideways sway (40%)**: the block stays PLUMB and slides sideways
  `h_st·sin(phi)` toward the lean side, `phi = U(8, 25)°`, drops
  `h_st·(1−cos phi) + U(0.15, 0.35)·h_st` — Northridge Meadows / Antakya's
  racked, not tilted, storeys.

`_soft_storey_residual` interpolates the standing wall height along the two
flank sides between `r_lean` and `r_far` so `_squash` follows the wedge, not
a single uniform factor.

## Heap clearance (`disaster/quake.py`)

Round-3 review: street trees stood untouched through a DG5 pile, a bus sat
clean at the toe. `quake._clear_under_heaps` (round-4, agent F; `quake.py`
now imports with NO module-level `pxr` — every `pxr` import moved per-
function so the pure half can be host-tested) runs before `_d_interactions`
for every DG4/DG5 building and every monolith swapped for a ruin:

* **0 to 0.3×H** from the wall line: BURIED — a tree is removed, a lamppost
  is tipped (poles tip across the whole reach, not just this band), a car is
  sunk to its roofline.
* **0.3×H to the reach**: lighter — a tree leans 25-45°, a lamppost still
  tips, a car is untouched (the outer rim doesn't reach a car's roofline the
  way it reaches a standing post).
* Reach is per-SIDE (`heap_reach_sides`/`_heap_reach_for`): the street/fall
  side gets the height-dependent run-out fraction above, a second side
  (stable-hashed from the prim path via `zlib.crc32`, never Python's salted
  `hash()`) also gets it, the remaining two sides get the flat blind
  fraction. `_heap_reach_for` prefers a MEASURED `{side: reach_m}` dict off
  the manifest record (`r["reach_m"]` / `r["fall_sides"]`) when the bake
  populated one, falling back to the drawn `heap_reach_sides` otherwise —
  wave-2 work (bake launcher package I) is what would populate it from
  `plan_pile`'s own `stats["reach_m"]`/`stats["fall_sides"]`; today's kit
  archetypes still draw.

## The bake's `PointInstancer` support (`disaster/bake.py`, agent E)

`export_object` used to walk `UsdGeom.Mesh` prims only, so a `PointInstancer`
under an exported object silently vanished — an archetype would bake with
the mound and large elements but none of the scatter that makes a heap read
as rubble. `_author_instancer` copies one instancer WHOLE: its own schema
attributes (`protoIndices`, `positions`, `orientations`, `scales`, ...) by
value, unconditionally exempt from the merge's dead-attribute stripping,
`positions`/etc. left UNCHANGED in the instancer's own local frame (only its
wrapping xform is rebaked to world, the same local-points-plus-one-world-
xform split every Mesh in this file already uses). `_copy_prototype_tree`
deep-copies each prototype in its OWN local frame — a referenced prototype
(from Nucleus) keeps its reference arc (never flattened by value, via
`_direct_references`, which resolves a relative reference against ITS OWN
source file the same way `_reanchor_assets` does for attribute values); an
inline prototype is copied attribute-by-value and walked recursively.
`export_object`'s `prototypes` relationship is remapped to the copied
prototypes' new paths — reading and writing the SAME relationship object
here would silently re-point the SOURCE stage instead of authoring the
export. Instancers (and anything under `proto_skip`, a target that lives
outside its own instancer's subtree) are pruned from the ordinary per-mesh
merge walk so nothing is authored twice. `validate` counts `PointInstancer`s
and their instance totals in its report.

## The offline review loop

`tools/rubble_preview.py` (`uv run --script`, `bpy` + `usd-core` + numpy +
pillow): builds a plan via `quake_rubble.plan_pile` (or a clearly-logged
synthetic fallback if that module didn't exist yet), authors it with
`quake_rubble_usd.author()` against the LOCAL asset mirror
(`RUBBLE_ASSET_ROOT` = `scene_gen/assets/`), drops in a ground plane and a
standing wall stub for scale, and renders four headless Cycles views (two
obliques, a near-nadir top, a ground-level "contact" crop) into
`~/scorch_previews/rubble_r4/{v2,v3,v4}/`. `tools/make_tileable.py` (numpy +
PIL) turns a debris-atlas photo into a seamless world-projected ground tile
for the mound/apron material.

**What Blender cannot show, and the fallback for it**: `damage._pbr` only
ever authors `outputs:mdl:surface` — correct for Kit/RTX, which resolves the
named "mdl" render context, but invisible to `bpy.ops.wm.usd_import`, which
resolves the UNIVERSAL (unsuffixed) context. Left alone, every mound/apron/
ground/box material this module creates would import into Blender with no
usable shader and render solid black. `_add_preview_fallback` authors a
plain `UsdPreviewSurface` on the SAME material path, bound to the universal
context, so Kit is unaffected (it never sees it) while Blender gets a real
shader. The MDL side stays `project_uvw` triplanar, world-space, with no
UVs needed at all — but `UsdPreviewSurface`'s texture reader needs a `st`
primvar to sample through, so `_author_heightfield` additionally authors
`primvars:st` = world (x, y) × the SAME repeats-per-metre scale the MDL side
uses, `vertex`-interpolated, existing ONLY for this fallback's benefit. An
authored "large" box has no UVs at all and stays a flat tint in both
renderers, matching what the MDL side does for an unmapped box too. None of
this substitutes for a real Kit/Isaac render of a curved or vertical
surface — it is a look CHECK, not a verification.

## Iteration history — what each render round found

| render | finding | fix |
|---|---|---|
| v1 (root of `rubble_r4/`, no `_close.png`) | floating rafts (`place()` treated `pos` as the piece CENTRE, so a raft floated ~0.4× its own thickness above the mound); a DOUBLE bury (the emitter subtracted `bury` again as metres on top of the planner's own fraction) | `pos` redefined as FINAL in the API contract (above); both bugs fixed in the planner/emitter |
| v2 | "smooth dune with sprinkled pebbles" — a comment now baked into `quake_rubble.py` itself, citing this exact review | `GRID_CELL_M` dropped to 0.5, fbm relief split into 5 octaves (long crown waves at half amplitude, 1-2 m real surface lumps added in absolute metres), patchy power-law chunk scatter, "shoulder" bumps under every raft/panel/column/cluster so it reads embedded rather than resting on top |
| v3 | crown height and run-out right; but the apron was a textured RECTANGLE (read as a plaza); the crown was too sparse (~0.3 chunks/m² vs ~2.5 target); the mound tile showed mirrored-ghost / atlas-chip islands ("plaid" — a debris atlas's own UV-island layout repeating at ground scale); the fan was a flat carpet; URM chunks read concrete-dark, not brick | apron redesigned dome-only / lobed / 1.15-1.25× toe; `CHUNK_DENSITY` raised to the crown/mid/toe table above; shoulder bumps made dome-only (a windrow/fan is already a thin ridge sized to `depth_m` — a bump there overshot it); `make_tileable.py --composite` (a high-passed atlas blend over a toned `Dirt_Rough` base) replaced the raw-atlas-crop tile; per-set/per-entry `look` keys added |
| v4 (`~/scorch_previews/rubble_r4/v4/`) | confirmed fixed: crown reads as a genuine heap (2,893 vs 738 instances on the same rc dome), apron a thin (~6 cm) lobed dust skirt — real but subtle, not a plaza — fan a real 3-D wedge (measured 0.82 m at the wall → 0.15 m at the toe, matching the 0.8 m spec), URM chunks read brick after fixing a `diffuse_tint`-multiplies-not-replaces bug (a first pass tinted brick texture with the FLAT-material rgb, rendering near-black — `_C_TEX["brick"]`'s own proven tint fixed it). NEW residual found in the same pass: a BLACK Z-FIGHT band where the mound/apron rim meets the ground plate, and pale, washed-out FAB clusters seen from directly overhead — confirmed by direct inspection of `rc_dome_s3_contact.png` / `urm_dome_s1_contact.png` for this write-up | `MOUND_LIP_M=0.008` / `APRON_LIP_M=0.012` landed in `quake_rubble.py` (nothing is ever authored exactly at z0); `MOUND_LIP_M`/`APRON_LIP_M` on the mesh side; the preview ground at -0.02 and the ×0.85 per-asset cluster tint landed with v5 |
| v5 (`~/scorch_previews/rubble_r4/v5/`) | the black rim is GONE in every view (rims at z0 + 8/12 mm, preview ground at -0.02); the FAB clusters still read pale from the air — their scans' near-white flecks survive a multiplicative tint | judge under RTX in Isaac; a 0.6-0.7 tint or a highlight roll-off is the lever |

---

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
| `EQ_RUBBLE` (`v2` default, `v1` = old `_heap`) | bench, bake, city | round 4: routes the six collapse recipes through `quake_rubble.plan_pile` + `quake_rubble_usd.author` instead of `_heap`'s box crate |
| `RUBBLE_ASSET_ROOT` | bench, bake, city, `tools/rubble_preview.py` | round 4: where the rubble catalogue resolves from — `omniverse://...` by default, a local mirror (`scene_gen/assets/`, via `tools/nucleus_fetch.py`) for previews and host tests |
| `FRACTURE_VTK_GUARD` (1 default, `0` = round-3 behaviour) | bench, bake | round 4: validates ids/finiteness at the VTK boundary (`_vtk_arrays`/`_strip_input`/`_from_vtk`) so a clipped-shell cut cannot segfault `vtkStripper`; measured < 3% cost, bit-identical fragments on/off for every kit module |

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
add its `bld_<style>_DG0.usd` here and bake it (`bake_quake_headless.sh
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
2-4 per patch, boil count from patch area), round-4 `_clear_under_heaps` /
`heap_reach_m` / `_heap_reach_for` / `style_of` (below).

**Rubble v2 (`disaster/quake_rubble.py`, round 4):**

| table / constant | does |
|---|---|
| `CATALOGUE`, `PROTO_SETS[btype]` | the debris asset library and which pool (raft/chunk/flake/rebar/sheet/cluster/toe) each construction type draws from |
| `CROWN_FRAC`, `CROWN_CAP_M` | crown height as a fraction of building H, capped (no data above ~11 m) |
| `REPOSE_DEG` / `REPOSE_DESIGN_DEG`, `APRON_REPOSE_DEG` | flank / apron angle of repose; design target sits 3° under the hard cap so noise has headroom |
| `_RUNOUT_FALL_ANCHORS`, `RUNOUT_BLIND_FRAC`, `RUNOUT_FLOOR_M`, `RUNOUT_CAP_FALL_M` (10), `RUNOUT_CAP_BLIND_M` (3), `URM_RUNOUT_MULT` (0.85) | per-side run-out vs. building height (`runout_frac`) |
| `CHUNK_DENSITY` (crown/mid/toe per m², cap), `FLAKE_N`, `CLUSTER_N`, `RUNOUT_CHUNK_FRAC/MULT` | instance-set density and the toe run-out share |
| `BURY` (per kind: raft/chunk/cluster/flake/panel/column/lintel/joist/rebar) | how deep each element sinks into the mound surface |
| `PANEL_LEAN_DEG`, `RAFT_*_TILT_DEG`, `COLUMN_TILT_FROM_VERTICAL`, lintel/joist/column size ranges | resting angles and sizes for authored large elements |
| `WINDROW_DEPTH_PARAPET/WALL`, `WINDROW_REACH_FRAC` (1.0), `FAN_WIDEN` | windrow/fan depth and reach vs. the fallen element's own height |
| `MOUND_LIP_M` (0.008), `APRON_LIP_M` (0.012) | round-4 review: nothing is ever authored exactly at z0 (the black z-fight band fix) |
| `GRID_CELL_M` (0.5), `GRID_TRI_CAP`, `NOISE_AMP_SCALE_123`, `RELIEF_WAVELENGTH/AMP_4/5` | heightfield resolution and the fbm relief octaves (round-2 review: "smooth dune with sprinkled pebbles") |
| `SHOULDER_BUMP_*` | the local surface raise under a raft/panel/column/cluster so it reads embedded (dome only — a windrow/fan is already sized to `depth_m`) |
| `ANGLE_LOBES`, `ANGLE_AMP`, `APRON_WIDER_FRAC` | the lobed, non-rectangular footprint outline (round-3 review: "no straight edges anywhere") |
| `rotated_extent(size, scale, rot)` | the tilted bounding box used for burial depth and the floating check |
| `plan_pile(...)`, `author(...)` | the API contract entry points — see "Rubble v2 (round 4)" above |

## Reading a run

The city banner prints the grade tally, the soft-soil patch and its
SETTLE/TILT/OV counts, the ground pass counts and `TIMING`. A scene that
prints DG5 > ~25 % of buildings has too large a core for its plate; one that
prints `SETTLE 0, TILT 0` has its patch on the collapsed side of town.

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
| settle steps | 3000 (round 3; 2200 left a pancake with 31 movers) | 1500 left 49 bodies moving on a masonry collapse; `highrise_04`'s DG5 still had 465 movers at 2200. A style row of 1.4-4.2k bodies solves in 2-8 min; the whole 96-archetype library was 19 min fracture + 77 min settle — `_plans/earthquake_timings.md` |
| grade cuts (URM) | 0.05/0.14/0.25/0.55/0.80 on `i * u` | worst-block mix 5/8/12/30/25/20 |

---

# History — the three review rounds

The user reviews captures; every item is traced to a measured cause on the
bench before anything changes. Round 1 (2026-08-26) built the vocabulary
(`earthquake_plan.md` v1-v8). Rounds 2 and 3 are below; the agent notes hold
the run tables and per-agent bug lists.


## Round 2 (2026-08-27)

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

## Round 3 (2026-08-27)

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

---

# The bug catalogue

## Round 1

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

**Kit façades are single-sided OPEN SHELLS, so their fragments are foil.** *(Round 3 superseded the workaround: `fracture.solidify` gives them thickness before slicing; the heap is still authored.)*
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

**A long bake process can stop fracturing, silently.** *(Round 3 found the mechanism: `trimesh.creation` resolves its triangulation engines once at import and `ensure_deps` installed `shapely` afterwards — `_reload_trimesh` fixes it; the one-process-per-style driver stays.)* The first full bake
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

## Round 4 (2026-08-30)

* **`vtkStripper::GetPointCells` reads off the heap on a clipped shell — a
  real SIGSEGV, not a fracture bug.** `vtkStripper` sizes a link table by the
  polydata's point count and walks it with ids taken FROM THE CELLS;
  `GetPointCells` validates neither, so one cell naming a point the array
  lacks reads off the end. Reproduced ONLY by that exact condition
  (`tools/_vtk_shell_probe.py`'s `oob` case) — 254 sliced `SM_Building_09`
  pieces × 8 seeds × the whole ladder never crashed offline; the real crash
  was in Kit's own dump (`_vtk_slice` → `strip.Update()` under a fire-
  collapse `_break`), where a sliced cut puts 894-1,233 stripper segments
  through vs. 88-290 for a kit module. Fix: `fracture._vtk_arrays`/
  `_strip_input`/`_from_vtk` validate ids and finiteness at both ends of the
  VTK boundary. `FRACTURE_VTK_GUARD=0` restores round-3 behaviour; measured
  < 3% cost, bit-identical fragments guard on/off. Also unblocks fracturing
  sliced GAC/downtowncity pieces for whoever resumes `quake_sliced.py`.
* **`r_soft_storey` leaned buildings AWAY from the side it named.** The
  original pivot used the LEAN side's own base edge with `sign = -1.0`,
  which pushes the FAR base edge `span·sin(lean)` into the storey below —
  2.9 m on a 30 m frame at 5.5°, invisible until someone asked "which way
  does this tilt" and checked the matrix rather than a render.
  `_soft_storey_geometry` pivots the FAR base edge instead and lets the lean
  side drop to its own (lower) residual — see "Soft-storey mechanics fix"
  above for the differential-crush/sway split that replaced it.
* **`quake.style_of` returned `(None, None)` for `SETTLE`/`TILT`/`OV`.** Only
  `_DG<n>` ever parsed. `_mono_pass` skips anything `style_of` resolves (a
  resolved style means "a kit archetype `assemble` already handled"), but
  `assemble`'s foundation pass runs BEFORE `_mono_pass` and re-points a
  standing building's reference at `bld_<style>_TILT.usd` — with the old
  parser that read as an unrecognised monolith, and `_mono_pass` could
  `_tilt_prim` it a SECOND time. `_tilt_prim` composes its matrix onto
  whatever local transform is already there, so this was a real compounding
  double-transform, not a relabelling. Fixed by recognising the foundation
  family (`SETTLE`/`TILT`/`OV`, with an optional `_vN` suffix) in
  `style_of` too.
* **`damage._pbr(tint=...)` is a no-op once a texture is bound — use
  `diffuse_tint`.** Round 2 already found this for `planks.wood_material`;
  round 4 re-found it inside `quake_rubble_usd`'s own look resolver: a first
  pass tinted the brick texture with `A_DEBRIS["brick_dusty"]`'s FLAT-
  material rgb, and because `diffuse_tint` MULTIPLIES the sampled photo
  rather than replacing it, every URM chunk rendered near-black instead of
  brick-red (`urm_dome_s1_close.png`). Fixed to `_C_TEX["brick"]`'s own
  proven tint × 0.85 dust, the same pattern the mound/apron materials
  already used — caught and fixed before it shipped past a preview.
* **Catalogue assets are bottom-centre origin, not centre — the floating-
  raft bug.** Every catalogue asset is footprint-centred with its BASE at
  z=0, but the planner's first `place()` treated a piece's `pos` as its
  CENTRE, so a raft floated ~0.4× its own thickness above the mound.
  Compounded by a second bug in the same pass — the emitter subtracted
  `bury` again as METRES on top of the planner's own fractional sink. Both
  found in the same v1 render; the contract now states `pos` is explicitly
  FINAL rather than leaving the origin convention implicit between two
  files built in parallel.
* **An atlas's own base-colour map is not a ground tile.** The first mound/
  apron texture was a straight copy of a FAB debris asset's UV atlas —
  correctly TEXTURED, but an atlas is packed as many small islands for ONE
  mesh's unwrap, and world-projecting that at ground scale repeats the same
  packed-island grid (the "plaid" v3 review). `make_tileable.py` breaks the
  atlas's own periodicity (rotated/shifted blends through smooth masks, a
  roll-and-feather seam repair) and, from v4, layers a high-passed copy of
  that blend over a toned `Dirt_Rough` scan instead of using it alone.
* **Coplanar rims z-fight with the ground plate — give every outer edge a
  lip.** A mound/apron vertex authored at EXACTLY the ground's own z0
  flickers against the ground plane in render — the same class of bug
  `quake_flow._c_dish`'s ring offsets already existed to prevent, on a mesh
  built after that lesson and re-hitting it anyway. Fix: `MOUND_LIP_M`
  (8 mm) / `APRON_LIP_M` (12 mm) — no vertex in either mesh is ever
  authored at plain z0.
* **`AddRotateXYZOp` composes Rz·Ry·Rx — proven, not assumed, with a
  `BBoxCache` check.** The planner (pure numpy, no `pxr`) and the emitter
  (the only rubble module that imports `pxr`) are built in parallel against
  a shared Euler-angle convention with no way to cross-import and check
  each other directly. Agent C2 computed the same rotated bounding box
  independently in numpy and via `UsdGeom.BBoxCache` and confirmed the two
  agree within 6 cm — the conventions actually match, not merely look like
  they should.
* **A `PointInstancer`'s children are not drawn directly, and its
  `prototypes` relationship must be REMAPPED, not just copied.** Round 3's
  `bake.py` never instanced anything (round 3's heap had no two identical
  objects to instance); round 4's scatter does, and Hydra never draws a
  `PointInstancer` child by itself, only through the instancer's own
  `protoIndices` — a prototype belongs as a CHILD of its instancer
  (`<instancer>/Prototypes/<name>`), get the nesting wrong and it either
  double-renders or renders as nothing. `bake.export_object`'s
  `_author_instancer` deep-copies each prototype under the NEW instancer's
  own `Prototypes` scope and points a fresh `CreatePrototypesRel()` at
  those copies — writing back onto the relationship object read FROM the
  source stage would silently re-point the source scene's own instancer at
  paths that only exist in the still-being-built export.

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

---

# Known gaps and next work

**Gate: diversity** (see Status) — damage the `selected_citydemo` monoliths;
more kit styles.

Open after round 3 (user: "a later issue", but real):

* a mullion frame hovering off the tower glass at the epicentre camera
  (`final_r3/c0_ne_obl.png`) — G's one-per-scene peel or a remnant; check;
* punched-window interiors (G2's pale floor band + head remnant) read a
  little cartoonish from 40 m; DG1-DG2 glass is sub-pixel from the air
  (correct per the record, unsatisfying);
* the 8.2 m curved corner bay of family 02 (`SM_MBuilding02_FirstFloor_A`)
  has no glazing entry — a curved-face pass is needed, not another rectangle;
* `dw_terrace`'s `storefront_a_3_5m` disagrees with `ub.PIECES` by 1.5 m
  (pre-existing, flagged, not fixed);
* timber deck plates on the DG5 crown (`plank` mode is exempt from the
  round-3 seeding); `_shard_field` at agent D's two call sites is round-2
  code; no cars / cordon under glass fall zones;
* `bld_apartment_tall_DG5` is 34 MB (1.4 M points; levers `EQ_SOLID_N`,
  `BRICK_BLOCKY`); the live pairs are 62 % of the city load — bake pairs as
  archetypes-of-two;
* in a LIVE pair only the leaner is rebuilt from the kit; the neighbour stays a
  reference and gets authored bands, not fracture; `r_collapse_onto` lands
  little of the thrown mass on the neighbour's roof;
* pounding scars need a gap under `QUAKE_POUND_GAP` (1.5 m); the packer's
  `building_gap_m` 2.0 gives ~2-2.4 m, so cities print "0 pounding scar(s)";
* `urban_quake_v2`'s `standalone/intact` monoliths and the ruin towers render
  untextured (their materials do not resolve in Isaac) — opt-in only;
* the downtown-quake path is NOT wired into the drone launcher
  (`scene_api.build_scene` is the suburb builder; the launcher's banner and
  annotations read suburb-only stats) — `downtown_quake_launch_script.py` is
  the looking launcher;
* lateral spreading, chimneys / gables (the kit has none), stair flights,
  standing water, crushed cars — not built; overturning is rare by design
  (one per scene, slender, clear fall, never a tower); `ARCH_VARIANTS=2`
  is supported but not baked.

Open after round 4 (rubble v2 — see that section for full context):

* **no Isaac verification yet** — every rubble-v2 claim above is a host test
  or a Blender render; the mound against a real kit stub, panels on a fan,
  the soft-storey collar, and the actual prim/body budget per archetype have
  never been seen through Kit. Gates the re-bake of the 16 kit styles' 144
  archetypes with `EQ_RUBBLE=v2` — do this first;
* **shoulder-bump slope overshoot** — several large elements landing close
  together can locally push the mound a few degrees past `REPOSE_DEG` even
  though the relaxation targets `REPOSE_DESIGN_DEG` with headroom; cosmetic;
* **the toe ring is an approximation, not a distribution fit** — `TOE_RING_*`
  scatters a fixed count of small chunks in a fixed annulus beyond the toe
  to avoid a hard debris-free edge; not fit to any measured density curve
  (none exists in the research);
* **the FAB brick/concrete clusters still read pale from the air** even with
  the ×0.85 per-asset tint that landed with v5 — the scans' near-white flecks
  survive a multiplicative tint; decide under RTX, then a 0.6-0.7 tint or a
  highlight roll-off;
* `disaster/quake_sliced.py` (earthquake on sliced GAC/downtowncity
  buildings) is written, tested (25 tests) and **frozen by user scope
  decision** — do not resume it without checking what the fire session's
  slicing/placement/bake code and skill (`build-urban-fire-scenes`) cover;
* the interior litter / ragged courses `quake_sliced.py` deliberately skips
  (anything that fractures a sliced piece) are now UNBLOCKED by the
  `FRACTURE_VTK_GUARD` fix — worth revisiting once that module is resumed,
  since the constraint that shaped its whole vocabulary no longer fully holds.
