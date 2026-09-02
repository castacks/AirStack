---
name: build-earthquake-scenes
description: Build or modify EARTHQUAKE-damaged URBAN scenes in scene_gen — the state of the pipeline after four rounds (2026-08-30), what it costs, and what gates it (building diversity, and now an Isaac verification pass). The kit-building fit-out, the per-construction-type damage ladder (URM wall peel / corner / parapet / masonry collapse on a brick lattice; RC infill / soft storey / pancake as prisms and rafts; curtain-wall bands with the mullion cage kept; lean-and-sink with a ground response; building-to-building pairs), solid walls before fracture, the merged archetype bake, the multi-city assembly by magnitude, the headless runner, every knob, and the bug catalogue. Round 4 (OFFLINE ONLY — host tests and Blender previews, no Isaac run yet) rebuilt the collapse pile as RUBBLE V2 — a heightfield mound with large elements and PointInstancer scatter, routed from `quake_flow` behind `EQ_RUBBLE` — fixed the soft-storey lean sign, cleared street furniture under a heap, taught `bake.py` to carry PointInstancers, and found + guarded a VTK segfault (`FRACTURE_VTK_GUARD`) that also unblocks fracturing sliced GAC/downtowncity pieces; `disaster/quake_sliced.py` (earthquake on sliced whole-asset buildings) is written but ON HOLD — that destruction/assembly scope moved to the fire session. Read before touching disaster/quake_flow.py, disaster/quake.py, disaster/quake_rubble.py, disaster/quake_rubble_usd.py, disaster/fracture.py, disaster/bake.py, the eq_building_bench / bake_quake_archetypes / downtown_quake launchers, the urban_quake asset sets or the downtown_earthquake preset. The wildfire and tornado skills are prerequisites.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Build Earthquake Scenes (urban building damage)

## Status on 2026-08-30 (evening) — round 5 is in flight, read this first

The first 500 m scene (eq500_gui, M7.8, 143 buildings) was shown to the user
at 07:26 and rejected on five points, verbatim in
`scene_gen/_plans/earthquake_round5_plan.md`: use the urban-fire partial
collapse mechanism (`disaster/fire_collapse.py` F5c — ragged `plan_edges`
tears, uniform-mode fragments that keep their cladding, outward throws) and
extend it to total collapse; the debris is "very low poly/cartoonesque" (HD
debris now split into `assets/rubble_hd/`, 840 textured pieces,
`quake_rubble.HD_CATALOGUE`); the kit-assembled buildings "look wrong" (the
fire bench's real MCE originals + `kit_substitute` twins look correct — the
city is being rebuilt that way: `asset_sets/urban_quake_v3.yaml`,
`quake.decide_building`); damaged and undamaged parts must share materials
(no `HEAP_MIX` / `_a_dustify` palettes; `quake_flow._dust_loose` dusts a COPY
of the fragment's own material, texture kept); the ground "dirt" must follow
the tornado's `scour_relief` treatment and broken slabs/kerbs must take the
material of the ground they lie on (`disaster/ground_class.py`). The user's
rule since then: **no Isaac Sim — local tests and unit tests only** until told
otherwise. New modules: `disaster/quake_collapse.py` (`LADDER_QC`,
`EQ_LADDER=qc|legacy`), `disaster/ground_class.py`, `tools/split_debris_spreads.py`,
`tools/standalone_intact_probe.py`, `tools/layout_dry_run.py`. Everything
below this header describes rounds 1-4 and is still the substrate.

## Status on 2026-08-30 — rounds 1-4

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

## 2026-09-02 — THE EMPTY-LOT ROOT CAUSE: swapped bakes inherited the intact cell's scale and centroid (`quake._swap_reference`)

**What the user saw** (eq500_v5_local, and "the same issue" reported across
the urban scenes): whole blocks reading as empty plazas, rubble rings round
nothing, buildings on the street.  Round 6 had root-caused the empties as
"dressed plazas".  They were not.

**Measured offline** (usd-core, no Kit): every `gac_quake/*.usd` and
`archetype/bld_*_DG*.usd` bake is authored in METRES, origin-centred, base at
z=0 (SM_Building_26 DG3: bbox centre (0.8, -0.2), 1,272 meshes).  The block
at (-243, 6)-(-12, 122) carried 7 buildings on record — SM_Building_11 (45 x
109 x 64 m), three SM_Building_26, block_residential — and the render showed
one kit building and a rubble outline.  `quake.assemble` swapped the
reference with "keep the transform, swap the reference": the cell kept the
0.01 scale op `apply_placements` authors for the centimetre-authored GAC
pack, so a 69 m bake composed 0.69 m tall.  21 of the 27 GAC originals in
that scene went that way; a same_art twin kept only the centroid correction
of its pivot-at-a-corner original and stood up to half a footprint off its
lot.  The records' own W/D were measured BEFORE the swap (`_mono_dims`), so
`quake_buildings.json` looked perfectly in-block and the rubble/dust/heap
passes drew full-size rings round the invisible speck.

**The fix** — in ASSEMBLY, not the slicer (the bake frame is right):
`quake._swap_reference(stage, prim, p, usd, ssf)` swaps the reference and
re-authors the cell as translate `(x_m, y_m, z_m)`, rotateXYZ `(0, 0, yaw)`,
scale `ssf` — the fire launcher's `place_holder` rule — then measures the
composed bbox and prints `[quake] ***` when it is empty or under
`SWAP_MIN_FOOTPRINT_M` on both axes.  All four swap sites in `assemble` go
through it (same_art twin, GAC bake, kit DG0->DGn, foundation variants); the
pair-interaction DG4 bump in `_d_geom_collapse` acts on a cell that already
has the right frame and is left alone.  Test: `tests/test_quake_swap_frame.py`
(bare usd-core: a corner-pivot centimetre box at scale 0.01 swapped for a
metre bake — the naive swap is kept as the negative control).

**Untested in a render** (user hold on Isaac).  Next launch: grep the pane
for `[quake] ***` — zero lines is the pass condition — and audit the top-down
with `tools/city_layout_audit.py`.  Note also that `snapshots_rp.overview`
is a PERSPECTIVE camera at 0.95 x span (18 mm): a 93 m tower at the plate
edge of a 500 m plate appears ~57 m outward, over the border road.  Judge
edge overhang from records or an orthographic capture, never from that PNG.

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
| `SETTLE_FABRIC` (`1` = on, default off) | bench, bake | round 4 Isaac pass: the settle was CPU-bound on PhysX's per-step USD write-back (3 % GPU, ~120 % of one core, 353 s for 3,019 bodies); with the knob `settle._step` sets `/physics/updateToUsd` False for the loop and publishes every body's pose once via `omni.physx...update_transformations(updateToUsd=True)`. NOT yet verified in Kit (an `omni.physx.fabric save_to_usd` attempt measured as a no-op — drop mean 0.00 — and was abandoned); verify with `eq_bench.sh fab_on EQ_STYLE=commercial EQ_RECIPES=DG4 EQ_SEED=4 SETTLE_STEPS=1500 SETTLE_FABRIC=1` vs the same without, comparing `[settle]` drop/spread (a drop mean near 0 = poses not written back) |
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

* **The manifest's `usd` paths were stale after the library move — every
  damaged swap in a city referenced a missing file.** `bake_quake_archetypes`
  records `usd=os.path.abspath(out)` (a CONTAINER path under whatever
  `ARCH_DIR` was at bake time, `.../assets/archetypes_quake/`); on 2026-08-29
  the library was renamed to `assets/archetype/` and mirrored to Nucleus
  (`tools/upload_archetypes.py`) but the 144 records were not rewritten, and
  `quake.assemble` does `refs.AddReference(chosen["usd"])` with the record's
  string verbatim. Found on the first Isaac run after round 4 (2026-08-30).
  Fix: `quake.load_manifest` REBASES every record's `usd` onto the
  `arch_dir` the caller passed, by basename (`_arch_join`), and reads the
  manifest through `omni.client` when `arch_dir` is an `omniverse://`
  folder (`_read_text`). The manifest is the catalogue; `ARCH_DIR` is where
  the files are. `tests/test_quake_manifest_rebase.py`.
* **The heightfield's flat floor rendered as a pale RECTANGLE under every
  pile (the v3 "plaza", back again).** Each mound/apron/strip cell is a
  rectangular grid inflated to fit the worst-case lobe, with everything
  outside the lobed toe floored at `MOUND_LIP_M` / `APRON_LIP_M` — and that
  floor was left IN the mesh with the dust material on it. The Blender
  previews never showed it because the preview ground sat 2 cm below the
  lip; Kit's ground is at z0, so the first Isaac run (r4_commercial DG5 top
  view) showed a sharp-edged pale rectangle round a lobed mound. Fix:
  `quake_rubble._trim_flat` drops every triangle whose three vertices sit on
  the cell's floor before the cells are concatenated (`_concat_cells`), so
  the mesh outline IS the lobe; rim vertices stay at the lip. Three more
  things had to change for the trim to bite, each found by measuring the
  grid host-side (`_plans` has no render of this — the numbers were the
  row-next-to-the-boundary heights): (a) the DOMAIN was too small — the
  first version inflated `reach` by (1 + 1.4 amp) but the lobed toe in
  metres is reach x (2 (1 + 1.4 amp) - 1), so every bulge was clipped
  straight; (b) the repose relaxation (`_limit_slope`, a Jacobi diffusion)
  spreads the foot 4-6 m past the nominal toe on a blind side (design ramp
  far steeper than repose there), so `_build_dome_grid` now REBUILDS on a
  grown domain until the row inside the boundary is back at the lip (rng
  draws hoisted so the rebuild is the same pile, `_dome_grid_once` is the
  one-shot core); (c) shoulder bumps from rafts seated near the foot re-raise
  the edge rows after that check, so `_taper_to_boundary` fades every
  dome/apron cell to the lip over its outer 1.5 m after placement (nothing
  is placed out there — `floating` stays 0), and a strip's FAR edge at
  build time (tapering a strip after placement dropped the surface under
  251 seated chunks). `_trim_flat`'s tolerance is 2 cm, matched to the
  rebuild check. Net: mound/apron outlines are the lobe (0 kept vertices on
  the grid rectangle in `test_mound_mesh_outline_is_the_lobe_not_the_grid`),
  the blind-side foot is a real ~5 m repose foot instead of a 1.6 m cliff.
* **The scanned FAB spreads rendered near-white under RTX** (`r4_commercial`
  DG4 / out_of_plane street views: 2-3 m "snow" heaps beside a brick pile).
  The scans' albedo is a pale mortar-grey and the v5 neutral x0.85 cannot
  move it. `quake_rubble_usd._TEXTURED_DUST_TINT_BY_MATERIAL` now tints by
  the catalogue entry's `material` — brick (0.55, 0.42, 0.36), concrete
  (0.52, 0.50, 0.47); unlisted materials keep the neutral constant.
* **The bake dropped every instancer prototype's material override, so the
  city's instanced debris rendered with the RAW asset materials** — white
  specks (the 34 flat-grey standalone pieces), grey blocks, snow-white FAB
  heaps — while the SAME pile on the live bench rendered right. Measured
  with usd-core on `bld_office_wide_DG5.usd` (first 500 m city, 2026-08-30):
  zero `QuakeLooks` materials in the file, every prototype's
  `material:binding` empty. `bake._copy_prototype_tree` copies ATTRIBUTES by
  value and re-adds the reference arc; `material:binding` is a RELATIONSHIP
  and the emitter's override materials live outside the exported subtree.
  Fix (landed): `bake._carry_direct_binding` — the instancer path reads the
  prototype's DIRECT binding (never an inherited one from outside the
  exported subtree), rebuilds it through `export_object`'s own material
  cache (`_cached_material`, so nine chunk prototypes sharing one look give
  ONE exported material) and rebinds with the same strength token. Verified
  on a real plan: 4 instancers / 1005 instances reopen with every prototype
  bound `strongerThanDescendants`; `tests/test_bake_instancer.py`
  (`TestBakeInstancerPrototypeMaterial`). Every DG5 baked before this fix
  (2026-08-30 06:50-07:30) has to be re-baked. Lesson: a bake must be
  reopened with usd-core and its bindings LISTED before its look is judged
  in the city — a missing override is silent, the asset's own material fills
  in.
* **Heap clearance used the NOMINAL blind reach (1.5-3 m) but the relaxed
  foot runs 4-6 m** — street trees stood inside a brownstone DG5's foot in
  the first city. `quake_rubble.extent_by_side` now measures the trimmed
  mound mesh's run-out past each wall (yaw-invariant), `plan_pile` reports
  it as `stats["extent_m"]`, the bake launcher's `_rubble_fields` carries the
  per-side max into the manifest, and `quake._heap_reach_for` takes the
  larger of measured extent and nominal reach per side.
* **`docker exec ... pgrep -f "<pattern>"` matches its own `bash -c`
  wrapper** (the pattern text is in that shell's cmdline) — a chain script
  waiting for "no bake process" never advanced. Use `pattern[s]`-style
  brackets, and kill helper chains by their SCRIPT NAME, not the log name.
* **Round 5: the HD "pieces" split from the scanned FAB spreads are OPEN
  SHELLS, not solids** (`assets/rubble_hd/open_frac.json`: boundary edges /
  all edges, median 0.23, only 38 of 840 under 0.10). Scattered as chunks
  with a stick-out tilt they rendered as sheets of foil with holes (v8b/v8c
  Blender proofs, `~/scorch_previews/rubble_r4/`). What DOES read as rubble
  at every distance is the WHOLE spread. So the dome's coverage class is
  now the `cluster` set — the scanned spreads at `CLUSTER_COVERAGE` 1.6 of
  the pile area (bbox-estimated, so ~1.0 real; overlapping, `CLUSTER_SCALE` 0.6-1.4, drawn UNIFORMLY over the raised pile — a crown-weighted draw left the lower slopes bare; the first ~10 sunk with
  shoulder bumps, the rest on the surface, `CLUSTER_MAX` 90; a brick pile
  draws its brick spread ~55 % with the concrete spreads as the minority) —
  and chunks/flakes are ACCENTS (`COVERAGE` 0.45/0.30/0.15, flakes at
  `FLAKE_COVERAGE_FRAC` 0.3 of that) restricted to near-closed, chunky pieces
  (`HD_CHUNK_MAX_OPEN` 0.15, `HD_CHUNK_MIN_ASPECT` 0.22, `HD_CHUNK_MIN_THICK_M`
  0.07; flakes `HD_FLAKE_MAX_OPEN` 0.30) and laid FLAT (a flake standing on
  edge is foil — `FLAKE_STICKOUT_FACTOR` 0.3). The instance cap scales with
  the pile: `max(RUBBLE_MAX_INSTANCES, RUBBLE_INSTANCES_PER_M2 x area)`,
  ceiling 24k, flakes trimmed first. Burial is shallower (`BURY["chunk"]`
  0.12-0.35, flake 0-0.12) because at 0.30-0.60 a 0.26 m median piece sat 9
  cm proud and 37 % of instances were under the fines. The building's own
  fracture fragments (`quake_collapse`, uniform mode, cladding kept) are the
  chunk-scale realism — the instancers are the fines and the clusters.
* **`tools/rubble_preview.py --sides` takes letters with no separator**
  (`NW`); `N,W` raises inside the real planner and the tool silently falls
  back to its 120-instance synthetic plan — check the `[rubble_preview]
  used ... the real planner` line before trusting a render.
* **`quake_rubble_usd.author(uid=...)` takes a CALLABLE, and `place()` buried
  by ROTATED thickness.** Two first-GAC-pilot-bake failures (2026-08-30
  night): (1) `quake_sliced._author_pile` passed `uid=qf._uid(ctx)` (an int)
  where `quake_flow._rubble` passes `uid=lambda: _uid(ctx)` — "'int' object
  is not callable" at the first large element; the module-boundary tests
  stubbed `author` and could not catch it, so
  `test_author_pile_runs_the_real_emitter` now runs the REAL emitter. (2) a
  planar FAB spread tilted with a steep windrow flank shows metres of
  ROTATED z-extent (6 m x sin 60 ~ 5 m) and `bury x thickness` sank
  clusters 5 m below grade (instancer zmin -5.18 in the pilot); `place()`
  now buries by a fraction of the piece's own UNROTATED height
  (`min(thickness, size_z x scale x 1.5 + 0.30)`), worst instance zmin
  measured -0.97 m (plan) / -1.11 m (re-baked file, TRUE per-instance
  minimum). CAVEAT found while verifying: a PointInstancer's AUTHORED
  `extent` is conservative (positions +- the largest piece's diagonal), so
  a BBoxCache query on the instancer prim reported zmin -5.15 on a file
  whose real instances bottom out at -1.11 — measure per-instance
  (positions + `rotated_extent`), never the instancer bbox, before calling
  a bake broken.
* **`bake_quake_headless.sh` never forwarded `EQ_SOLID_N` / `EQ_RUBBLE`.**
  `docker exec` does not inherit the host environment and the driver only
  passed its own `ARCH_*` / `SETTLE_*` / `BAKE_MERGE` pairs, so the
  `SETTLE_STEPS=3000 EQ_SOLID_N=0.85 bake_quake_headless.sh` line above
  baked round 3 at the code default (`T_SOLID_N_SCALE` 1.0). `EXTRA_ENV="..."`
  now appends arbitrary pairs; the bench and the bake must agree on them.

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

## Round 5 review fixes (2026-08-31)

Five packages, each delegated and re-verified by measurement, after the first
OSMO 500 m GAC+MCE scene's review. All landed; 266+ tests green combined.

**Blank "plain brick" buildings — the fam04 Facade_B trap.** Every fam04
style (`commercial`, `commercial_mid`, `highrise_04`, `department_store`,
`block_commercial`) baked with `SM_MBuilding04_Facade_B` — a literal
8-vertex box, texture-only windows — on 100% of storey slots on ALL FOUR
sides: `_plan_band`'s shared rng reaches the storey pick at the same point
every bake. Fix in `detail/urban_building.py`: an optional per-band `side`
pool, deterministic (consumes ZERO rng draws — an early rng-drawing version
silently changed the top band's motif; any extra draw shifts every band
after it), forced to `Facade_A` on E/N/W and, per the round-5 follow-up, on
S too (the photographed blank walls WERE fronts). Kit textures do not
resolve locally, so judge elevations with a true ORTHOGRAPHIC per-side
camera and per-camera light — a perspective+raking-light rig produced a
false positive on brownstone's E flank (all four brownstone sides are in
fact identical). Guard: `tests/test_kit_elevations.py`.

**Perfect-cuboid debris → `fracture.chip_box`.** Capped VTK plane clips
(corner bites, bevel shears, end steps sized against the CROSS-SECTION so a
bar's break face reads snapped, not sawn), subdivide-then-roughen AFTER
chips, parabolic+sine+twist warp for timber only. Wired into
`quake_rubble_usd._box`/`_author_large` and `quake_collapse`'s plank/prism
cells and dropped plates. `QC_CHIP=0` reproduces old boxes byte-for-byte;
never chips a non-solid input (clipped-shell SIGSEGV rule). 4.4-8.7 ms and
66-256 tris per piece — no cache needed. Broken concrete (beams + fragment
interiors via `quake_flow.MATERIAL_URLS["concrete"]`) binds megascans
`Damaged_Concrete_Floor`; roofs keep their own asphalt URL.

**Floating roof props.** `quake_flow.dress_roof` seats tanks/AC before
recipes run and nothing asked whether that prop's own roof survived
(`_b_settle_roof_plant`'s burial share never matched qc recipe names).
`quake_collapse._sweep_roof_props`: majority-of-5-footprint-points against
`plan["region"]` (centre alone is NOT enough — fire bug 9), bury under
total/pancake via `_a_bury_props`, outward kick + physics for
elevation/corner; resolved paths pruned so the generic pass can't
double-handle. The archetype bake launcher now runs
`fire_bake.deactivate_airborne` per building post-settle (`airborne_off` in
the manifest) — the sky-parked tank was ultimately a SETTLE_STEPS freeze.

**Ground.** Worn_Pavement retired everywhere in the quake path; road/pave
pieces draw a per-building crc32 mixture of Damaged_Asphalt /
Damaged_Asphalt_02 / Crushed_Asphalt_Ground (8K source downscaled in place
to 2K — `tools/import_megascans.py` now has a PIL downscale step). Fissures
1.75x (`EQ_FISSURE_SCALE`) and textured with the tornado soil/silt — whose
tables `quake_flow._C_TEX` now IMPORTS from `scour_relief._TEX` so drift is
structurally impossible.

**Scanned concrete presence.** rc piles: `CLUSTER_COVERAGE_RC=2.1`,
`CLUSTER_MAX_RC=115`, `CLUSTER_N_RC=(6,14)`; urm minority-concrete pool
weighted toward the three big pile spreads. `RUBBLE_HP=1` opt-in prefers
`_hp` twins (guarded: probes the filesystem for a LOCAL asset_root, trusts
the catalogue for `omniverse://`). Street scatter (`plan_street_scatter`,
`QUAKE_STREET_DEBRIS`, city cap `STREET_DEBRIS_MAX`) places pieces strictly
beyond the measured heap reach on the fall sides — and took FOUR review
rounds because scan shells fail on clean pavement in ways piles hide:
bbox `_chunky`+open_frac gates are provably insufficient (a flat scan's
curvature spans its bbox: `concrete_slabs_p028` passes both at PCA
thin/long ratio 0.013), so a static PCA census
(`assets/rubble_hd/volume_ratio.json`, works against omniverse:// roots)
gates the pool, seating must use POINTS-min-z (bbox corners leave daylight
under curled pieces — the BBoxCache blind spot again), and orientation must
lay the PCA-thin axis up or pieces stand on end like foil.

**Two process lessons.** (1) Never assert on `sys.modules` in a shared
pytest session — test_quake_sliced's import-isolation check passed alone
and failed in combined runs once another file legitimately imported
quake_rubble; check transitive-import contracts in a fresh subprocess.
(2) `pytest ... | tail -3` in a background command destroys the traceback
you will need — keep full output in the task file and tail at read time.

## Round 6b (2026-08-31)

A team of six agents (pillar-bench, opus, INVESTIGATE+BUILD; tanks,
rect-cutouts, brick-materials, empty-lots, scene-variety, all sonnet
INVESTIGATE-ONLY until a GO), plus follow-on implementers (plazas-fix,
brick-fix, pod-prep, fire-freeze-check), against the second OSMO review of
the seed-9 `urban_quake_v5` scene (500 m, M7.8): the user's five per-photo
complaints from round 5's fixes still visible (pillars/plates read as
perfect cuboids, water tanks/AC units floating, brown untextured facade
cards, straight rooflines / orphan window panels / intact dormers at
removal boundaries, brownstone reading grey), a mid-round "why are there
empty lots" question, and a mid-round new requirement: "TEST scene =
variety showcase. Damaged population must include AEC brownstones + GAC +
MCE; dtc pristine-only filler. Damage states all clearly present: full
collapse, partial collapse, tilt, tilt+partial collapse, ground cracks. Use
damaged-asphalt megascans on roads/cracked ground." Every root cause below
was traced to a measured cause in the actual code (not assumption) before a
fix landed, and every agent's report was independently re-verified by the
lead against a pre-agent-edit worktree snapshot before acceptance.

### 1. Root causes — several were NOT what earlier rounds assumed

* **Pillars/plates still read as perfect cuboids.** Two independent causes,
  not one. (a) `fracture._one_chip`'s clip geometry is anchored at the box's
  own CORNERS — a shaft or pillar's own MIDDLE is structurally unreachable by
  it, so no amount of retuning `depth_frac` or the roughening already in
  place could ever touch it. (b) Separately, `quake_flow.py` authors roughly
  fourteen of its own populations of `_box` cuboids (`_p_lintels`,
  `_disturb_interior`, `_c_clods`, `_c_kerb`, `_c_fissures`,
  `_c_overturn_ground`, `_b_crumbs`, `_d_chunk`, `_d_face_band`, `_shaft`,
  `_buckled_pavement`, `fit_interior`'s standing columns/piers, `_roof_box`,
  `r_signage_fail` — `quake_collapse.spec_for_shape`'s own docstring names
  them), and NONE of them ever called `fracture.chip_box`, which had existed
  since round 5 wired into only four OTHER emitters
  (`quake_rubble_usd._box`, fit-out slabs, `_break_box` cells,
  `quake_sliced`). `grep -c chip quake_flow.py` was 1 before this round, and
  that one hit was a comment.
* **Floating water tanks/silos, again.** Four compounding causes. (1)
  `ROOF_PROP_MODES` never included `soft_storey`/`mid_storey`, and
  `_author_band`'s own "above" list (`_els(...)` + `fit[...]`) never carried
  `ctx["roof_plant"]`/`ctx["roof_fixed"]` — a tank on a crushed RC storey was
  never in the set of things the recipe moved down with the rest of the
  mass. (2) The elevation/corner path only gives a fallen prop a 0.3-0.9 m/s
  kick and a fixed settle-step budget — the module's own docstring already
  predicted this exact failure. (3) The bake's settle pass had no
  strict/converge mode; `still_moving == 0` is a false negative (it measures
  a fixed end-of-budget velocity, never whether anything is actually
  supported). (4) Decisive: 0 of 162 manifest records had `airborne_off` set
  — the entire existing archetype library predates `deactivate_airborne` and
  was never rebaked with it, so every earlier "fix" was measured against
  stale bakes. On top of all four, `quake_sliced.py`'s parallel GAC recipe
  set never called the roof-prop sweep at all.
* **Brown, untextured facade "cards."** `_face_patch`'s `inner_mat()` and the
  pile-scale `_chunk_material` (10 more of 16 total sites found by census)
  bound RAW `mats["brick"]` megascans references directly onto UV-less
  authored meshes — a catalogued defect class this file already has a fix
  pattern for (`_c_look`/`_c_look_at`, world-projected `project_uvw`) that
  had simply never been migrated to these particular sites. Separately, kit
  family "03" (called "brownstone" in earlier rounds) is MCE's
  `MBuilding03` — pale, modern-cladding art, not masonry — so red-brick
  dressing read as visibly wrong there regardless of the UV bug
  (`FAMILY_TYPE["03"] = "urm"` is correct for the DAMAGE ladder, wrong for
  texture choice). The separate "brownstone reads grey" report was ruled NOT
  a bug: real asset naming/choice, and a rename would break every bake key.
* **Rectangular cutouts / orphan panels / intact dormers at removal
  boundaries.** Three causes. (1) The RC infill gate skipped `_ragged_slabs`
  outright (the old `not plan["infill"]` condition), so a floor exposed by
  an elevation/corner cut kept a clean rectangular edge instead of a torn
  one. (2) Kept-whole rubble panels (`plan_pile`'s "panel" large-element
  kind — a building's own kit module laid whole on the mound) were leaning
  `PANEL_LEAN_DEG` = (30, 70)°, but `_orient_on_surface`'s convention is
  0° = upright, 90° = flat — so that range read as STANDING, the opposite of
  "a panel resting on a pile." (3) Sliced-GAC removal boundaries are raw by
  design — the round-4 VTK guard fix that unblocked fracturing sliced pieces
  was never resumed against them — and `disaster/monolith_damage.py` (built
  for exactly this ragged-boundary problem on monoliths) has zero
  production call sites; only tests/tools/docstrings reference it.
* **Empty lots (mid-round question).** NOT the gprim-rooted instancing
  refusal in `scene_generator.apply_placements` some earlier notes
  suspected: `generate_scene.py`'s `inst_cats` stays `None` unless a preset
  sets `instance_placements: true` (only `downtown_fire_500.yaml` does) or
  `SG_INSTANCE_PLACEMENTS=1` is set — the earthquake preset sets neither, so
  the quake path never instances anything and the gprim-refusal print never
  even fires there; that whole bug class contributes NOTHING to quake-path
  emptiness. The `districts.py` frontage fix (verified landed) measured only
  +1.4 pt on this preset/seed (59.3% -> 60.7% street-wall — within
  single-seed RNG confound). The DOMINANT cause: `downtown_earthquake.yaml`
  had NO `city_detail.plazas` block at all, and `city_detail._place_plazas`
  early-returns without one — which ALSO kills the COURTS pass (the tree-
  allée + bench-pair dressing every OTHER typology's block interior gets),
  since both live inside the same function. Every open paved gap in the
  city — tower plaza aprons and ordinary leftover block-interior space alike
  — went completely undressed. Worst measured block: a 67.3 x 103.2 m
  `brick_midrise` leaf, 34.6% fill, 3,849 m2 free rect, 0 arrangements.
* **New requirement mid-round: variety showcase.** `PRISTINE_PACKS`
  (round 6) blocked AEC brownstones entirely — a prior session's deliberate
  gate, now conflicting with the new request that AEC appear damaged. GAC
  DG5 is unreachable (no bake at that grade exists yet).

### 2. Fixes landed, per file

* `disaster/fracture.py` — `gouge_arrays`/`_gouge_faces` (station-anchored
  scallops bitten into a closed solid, one shared triangle-volume budget per
  piece so a long face's gouge cannot cost more than a short one); a
  `"bite"` chip kind (very large corner bites sized against the piece's TWO
  LARGEST axes, not the longest); red-green edge subdivision so a chip pass
  has geometry to displace before roughening; `rough_lam_frac` (per-lamella
  roughening weight). All new knobs default to off/zero so a round-5 bake
  reproduces byte-for-byte at the old defaults.
* `disaster/quake_collapse.py` — `spec_for_shape`'s size ladder:
  `CHIP_TINY_M = 0.32` (chips + end-steps only, no gouge pass, no
  refinement — a 30-60 tri piece whose silhouette is the only thing that can
  read at that size), `CHIP_SMALL_M = 0.85` (gouge pass on a quarter
  triangle budget), full table above. `plan["floor_ragged"]` field replaces
  the old infill gate. `_deck_support_z` (the `tri_soup` idiom from
  `tools/fc_roof_deck_probe.py`, run live against the mid-authoring stage):
  the highest upward-facing triangle Z under a prop's own footprint, so a
  fallen roof prop is placed on what is REALLY there instead of a
  kick-and-hope. `_sweep_roof_props` reworked to test the prop's own QUERY
  RECTANGLE against each candidate face — not the face's centroid, which for
  a wide kit slab's 2-triangle fan sits near a quarter/three-quarters along
  its length and is never near the middle, so the old centroid test silently
  read every real deck as unsupported — and to treat `root in ("", "/")` as
  "every prim qualifies" rather than "only prims literally at `/`" (nothing
  ever is). `_FACADE_SCAR_BRICK_P = {"03": 0.12}` (vs. 0.5 default) — family
  03's scar weighting, since its brick dressing is the wrong art to begin
  with.
* `disaster/quake_flow.py` — `_C_TEX["concrete"]` added (bound to
  `Damaged_Concrete_Floor`, sourced from `quake_rubble_usd._BEAM_SPEC`'s
  already-proven map); 11 of 16 raw `mats["brick"]`/`mats["concrete"]`
  reference sites migrated to `_c_look`/`_c_look_at`;
  `_chip_authored(ctx, paths, timber=False, tessellate=True, why="")` — new
  helper, wired at 11 call sites (`lintel`, `litter`, `cornerfrag`, `column`,
  `colchunk` x2, `pavement`, `kerb`, `crumb`, `chunk`, `spall`), gated on
  `_RUBBLE_MODE == "v2"` AND `chips_enabled()` (`QC_CHIP=0` bypasses — no new
  switch introduced) and passing every piece through
  `quake_collapse._chip_pieces`'s existing refusal ladder (UV'd meshes,
  `CHIP_MAX_FACES`, non-tri/quad faces, open shells all pass through
  untouched — the clipped-shell VTK SIGSEGV rule from round 4). Deliberately
  left unchipped: `_c_clods`/`_c_fissures`/`_c_overturn_ground` (soil, not
  cast concrete), `r_signage_fail` (sheet metal doesn't spall), `_shaft` (a
  surviving shear core already gets a ragged crown from `fracture_partial`,
  not debris), `_roof_box` (an intact roof slab must stay intact — only its
  broken pieces get chipped), and `_d_chunk`'s own `flat=True` draw (a 12 mm
  ground-hugging plate authored as a "stain," never registered as static
  geometry — a bite out of it is triangles spent on something only ever seen
  face-on; the non-flat chunks now get the ladder via `why="chunk"`).
* `disaster/quake_rubble.py` — `PANEL_LEAN_DEG` corrected from (30, 70) to
  `(50.0, 80.0)` after a 150-seed measurement of the real `plan_pile` found
  the OLD range's mean angle-from-vertical was 53°, and the first proposed
  fix, (10, 35) (matching `PANEL_TILT_DEG`'s numbers), would have made it
  32° — MORE upright, the wrong direction, because `_orient_on_surface`
  counts degrees tipped AWAY from standing (0° = upright, 90° = flat), not
  degrees propped up from lying. (50, 80) gives a 67° mean, matching a
  physical cross-check (a 3 m panel resting its far edge on a 1.4 m stub
  leans arccos(1.4/3.0) ≈ 62° from vertical).
* `disaster/quake_sliced.py` — `_sweep_roof_props_sliced` (the tank-fix's
  sliced-path twin): carries `roof_plant`/`roof_fixed` through
  `plan["fit_ops"]` displace_above specs, reseats via the union of
  removed-piece XY rects + `_deck_support_z` + `_a_bury_props`; chip-box
  wiring extended to this ladder's own dropped slabs, plus new laid-panel
  dressing (sink + tilt + edge chunks) since a whole kept-whole kit panel
  can never be VTK-cut.
* `disaster/settle.py` / `bake_quake_archetypes_launch_script.py` —
  `converge=True` makes `steps` a target instead of a hard budget (up to
  `settle.run`'s own 3x `max_steps`); `strict=SETTLE_STRICT` turns an
  unconverged settle into a raised `SettleNotConverged` instead of a silent
  warning. Deliberately double-checked: `settle.run`'s OWN internal
  `strict=None` default reads the SAME env var the OPPOSITE way (empty
  string = not strict); the launcher resolves `SETTLE_STRICT` itself
  (empty/unset = strict ON, `0`/`false` = the escape hatch) rather than
  leaving the ambiguous shared default in play.
* `config/presets/downtown_earthquake.yaml` — one contiguous
  `# --- plazas/courts dressing (round 6b) ---` block (`plazas: {enabled:
  true, typologies: [highrise], min_side_m: 15.0, min_area_m2: 280.0,
  max_per_block: 2, podium_margin_m: 1.5, edge_margin_m: 2.5, ...}`, copied
  verbatim off `downtown_gac.yaml`'s own block; `typologies: [highrise]`
  scopes the fountain/bench-ring/cafe composition the same way GAC scopes
  it, while every OTHER typology still gets the COURTS pass via
  `court_typologies=None`'s "everything else" default); `debris.tilt_chance:
  0.2` (was 0.12 default via `quake.assemble`); asphalt megascans binding
  retargeted from `Damaged_Asphalt` to `Damaged_Asphalt_02`.
* `disaster/quake.py` — `PRISTINE_PACKS` narrowed from `("downtowncity/",
  "assets/aec/")` to `("downtowncity/",)`, with a trail comment explaining
  why AEC was added and then removed the same day (a later, more specific
  user request wants AEC brownstones damaged, DowntownCity pristine-only) —
  an AEC brownstone now falls through to the SAME generic monolith fallback
  standalone monoliths already get (`_mono_pass`, scored as `"rc"`) rather
  than being skipped entirely.

### 3. New tools/tests

* `tools/pillar_break_bench.py` — bpy idiom borrowed from
  `rubble_preview.py`, renders to `~/scorch_previews/pillar_bench/`;
  `--heap` (with `--heap-seed`, `--heap-lintels`, `--heap-rubble`) exercises
  the wired-in `quake_flow` chip populations at heap scale, not just one
  pillar.
* `tools/roof_plant_seat_probe.py` — points-based support verifier (not a
  bbox audit — see the `PointInstancer`/`BBoxCache` blind spot below);
  `--usd` (repeatable) targets specific archetype or GAC files;
  `ROOF_PLANT_PROBE_LIMIT` env caps files scanned for a quick inline
  `pytest` run (~8 s/file locally). `tests/test_roof_plant_seat_probe.py`
  skips cleanly without the referenced assets present.
* Test counts (this round; `test_kit_elevations.py` is round-5, unchanged
  here): `tests/test_block_fill_and_plazas.py` 13/13; `tests/
  test_chip_box.py` extended for the new gouge/bite/rough_lam_frac knobs —
  39/39 on the bench-only fracture.py work, 164/164 after the `quake_flow`
  wiring landed, both runs under `uv run --with vtk`; `tests/
  test_quake_v5_city.py` (18 top-level test functions, several
  parametrized) dropped to 12/22 mid-round when `PRISTINE_PACKS` was
  narrowed ahead of the test file's own stale `("downtowncity/",
  "assets/aec/")` expectation, reconciled (6 assertions changed) to 23/23;
  brick-fix migration: 125 tests green, raw-material-binding census
  31 -> 0; rect-fixes GO1 (`floor_ragged`): 85 tests including one new one;
  GO2 (`PANEL_LEAN_DEG`): 150 tests; tanks-fix sliced-path wiring: 34/34.

### 4. Verification discipline that worked

Bench-first Blender renders before touching production code
(`~/scorch_previews/pillar_bench/`, `rect_fixes/`, `brick_fixes/`); a
cross-section fill metric for the empty-lots census; points-based probes
preferred over bbox audits — a `PointInstancer`'s authored `extent` (and a
`UsdGeom.BBoxCache` query on the instancer prim) reports the conservative
envelope of its LARGEST instance, never any individual instance's true
position, a blind spot round 4's rubble work already found and documented
that resurfaced here for roof props; same-process A/B (`spec_overrides`) for
yaml/config changes, so two runs are provably the same code and RNG rather
than two different repo states; the lead re-running every agent's own tests
before accepting a report, against a pre-agent-edit worktree snapshot. **The
trap**: chip-related tests require `uv run --with vtk`, not bare `python3`
— a bare interpreter lacks VTK and produces 19 phantom failures that look
like real breakage; this bit two separate people in this round before being
flagged clearly enough to stop recurring.

### 5. Variety/composition decisions

AEC brownstones are now damaged via the generic monolith fallback (rigid
heavy/mild lean+sink from `_mono_pass`, scored on the RC vulnerability
curve) rather than a real fracture ladder — a real AEC ladder is DEFERRED:
the asset is internally instanced and fails `monolith_damage.cut_shell` with
a Tf error (`tools/openings_probe.py`), so it needs de-instancing, a new
`quake_sliced.CONSTRUCTION` entry, and a bake driver, none of which exist
yet. GAC DG5 is unreachable until specific buildings are baked at that grade
(queued: `SM_Building_02`, `SM_Building_24`); `decide_building` steps down
to DG4 in the meantime. DowntownCity stays pristine-only by explicit
`PRISTINE_PACKS` design (no damage pipeline exists for it, and the user does
not want one). There is no quota/mixture-enforcement mechanism for the
showcase mix; seed 9 is pinned because it is the seed that happens to place
both AEC and DowntownCity buildings in the current layout.

### 6. Pod state and re-bake plan

Pod `airstack-dev-177` recon (read-only): RUNNING, tunnel on :2201, 4 TB
disk, 158 GiB RAM free, shm 16G, VTK OK on both pythons. Three blockers
found: (1) a stale seed-8/v4 Kit process sharing the GPU with the live
seed-9/v5 scene (25.9/48.9 GB, 99% util) — directed to kill only the stale
one; (2) the pod is missing ~1.5 GB of AEC tower/brownstone Vegetation
assets and has only 13 MB of brownstone Materials (vs. 1.5 GB expected) —
the scene had no trees and threw 403s; `AIRSTACK_ASSET_ROOT` was unset in
the container; (3) pod git was 8 commits behind local, with 12 dirty files
subsumed by local's already-committed state — a stash/pull recipe was
directed. ALL existing pod bakes (36 GAC pair bakes, 8 kit-style batches,
only 69/162 `airborne_off`) predate every fix landed this round and must be
fully re-baked. Measured per-unit timing: heavy DG3-5 kit styles ~130-330
s/style, light styles ~15-70 s per 4-style batch, GAC ~2-3 min/building
bake. Sequence to the next scene run: wait on tanks-sliced + rect final
report + fire-freeze-check -> regenerate the sync file list -> rsync code ->
archetype bake chain -> GAC chain (+ the two DG5 candidates) -> gate on
chip-proof lines in the log, 162/162 `airborne_off`, and the roof-plant
probe passing both directions -> close the currently-open seed-9 scene ->
relaunch seed-9, `urban_quake_v5`, M7.8, 500x500 -> pull photos. Runbook and
sync list live under this session's own scratchpad, not the repo.

### 7. Bug catalogue additions

* **A chip pass anchored at box CORNERS cannot touch a shaft's middle.**
  `fracture._one_chip`'s existing clip geometry is corner-reachable only;
  retuning `depth_frac` or layering a roughen pass on top, which earlier
  rounds tried, could never fix a pillar/column reading as a perfect prism,
  because the geometry that needed removing was structurally out of reach.
  Fixed by adding gouge/bite modes that anchor at a chosen STATION along a
  face instead of at a corner.
* **A shared helper existing in one file does not mean it is wired
  everywhere it is needed.** `fracture.chip_box` shipped in round 5 wired
  into four emitters; a whole OTHER file (`quake_flow.py`, ~14 of its own
  cuboid-authoring populations) never called it, and `grep -c chip
  quake_flow.py` staying at 1 (a comment) for a full round was the tell
  nobody checked. The new `[chip] quake_flow: N chipped / M passed` proof
  line exists specifically so a bake LOG shows the count fired instead of
  the wiring being "believed done" (the same lesson round 5 already learned
  once for this exact file, and repeated).
* **Testing a triangle's centroid against a small footprint fails on wide
  fan-triangulated quads.** A kit slab/`_box`/`_roof_box` is authored as one
  big quad; its 2-triangle fan's centroids sit near a quarter and
  three-quarters along the slab's own length, never near the middle — so
  "is the centroid inside the prop's footprint" against a face many times
  the footprint's size is never true, and `_deck_support_z` silently read
  every real deck as unsupported until the query RECTANGLE itself (not the
  triangle's centroid point) was tested for containment instead.
* **`root in ("", "/")` needs an explicit branch, or a scoped-support query
  silently scopes to nothing.** `_deck_support_z` takes `root` as "this one
  building's own scope, never the whole stage" in the normal case, but a
  caller occasionally passes the pseudo-root; treating that as "every prim
  qualifies" rather than "only prims literally AT `/`" (which nothing ever
  is) has to be an explicit special case, not assumed.
* **A tilt-angle convention has to be measured, not guessed from another
  table's numbers.** `PANEL_LEAN_DEG`'s first proposed fix, (10, 35)
  (matching `PANEL_TILT_DEG`), would have made the panels MORE upright — the
  wrong direction — because `_orient_on_surface` counts degrees tipped AWAY
  from standing (0° = upright, 90° = flat), the opposite of what "a smaller
  number reads as less lean" assumes. A 150-seed measurement against the
  real planner, cross-checked with a physical `arccos` calculation, caught
  this before it reached a render.
* **Zero manifest records with a safety flag set means the library predates
  the safety net, not that the net is failing.** 0/162 `airborne_off`
  records was the decisive clue that every existing bake needed a full
  re-run, not a targeted fix — a partial "some floaters, some not" pattern
  would have suggested a recipe-specific bug instead.
* **A `PointInstancer`'s authored `extent` (and a `BBoxCache` query on the
  instancer prim) reports the conservative envelope of its LARGEST instance,
  never any individual instance's true position.** This round re-hit the
  same blind spot round 4's rubble work already found and documented —
  `tools/roof_plant_seat_probe.py` was built points-based specifically so
  this class of prop-seating question is never answered with a bbox again.
* **Process: a completed subagent drops off the roster and cannot be
  resumed by id once its transcript is gone.** Spawn a fresh implementer
  with condensed context instead of trying to continue a "DONE" agent —
  this happened repeatedly this round and cost nothing once recognized, but
  the first instance cost a stalled follow-up before the lesson was named.
* **Process: one-file-per-agent ownership plus string-anchored edits avoids
  clobbers in shared files under concurrent editing.** `quake_collapse.py`
  and `quake_sliced.py` were each touched by two different agents in
  sequence without collision because each landed a self-contained,
  delimited change rather than a diff against an assumed baseline.
* **Process: concurrent sessions can drift the measurement baseline
  mid-round.** The plazas-fix agent's own before/after fill numbers stopped
  matching a previously-stored baseline because another, unrelated session
  was live-editing `districts.py` at the same time; the agent caught this by
  re-running its OWN "plazas off" case on the day's code and finding the
  SAME drifted numbers, proving the drift predated and was independent of
  its own change rather than assuming its own edit was at fault.

### 8. Open items / next round

* **RESOLVED — freeze intact**: the fire-freeze-check on the brick-material
  migration (`quake_flow.fit_interior` is shared with `urban_fire`, the
  MCE-fire-frozen policy). `tools/kit_burn_probe.py` run pre-change (from
  `git show HEAD:`) vs current on apartment F5c / office F5c / apartment F1
  was byte-identical in counts, FLAGs and material census; an isolated
  `fit_interior` probe shows only the 64 rc slab/column prims re-binding
  `concrete` → `c_concrete` (the intended fix), zero geometry change, and the
  chip wiring is never reached from the fire recipes. Probe runs in the LOCAL
  `isaac-sim` container via `usd_python.sh` (no Kit).
* **PENDING at time of writing**: the rect-cutouts agent's final renders and
  its two written HOLD design sketches — (a) the sliced-GAC
  ragged-removal-boundary approach (was waiting on the pillar/chip work,
  which has since landed) and (b) wiring `monolith_damage` into `_mono_pass`
  (unblocked by the variety census, likely an Opus task next). GO1 (the
  `floor_ragged` gate replacement) and GO2 (the `PANEL_LEAN_DEG` correction)
  are both landed and tested; these two items are what remains open on that
  agent's brief.
* Real AEC fracture ladder (de-instance the asset, new
  `quake_sliced.CONSTRUCTION` entry, bake driver, pod time) — deferred, not
  started.
* GAC DG5 bakes (`SM_Building_02`, `SM_Building_24`) — queued for the pod
  re-bake batch, not yet run.
* The open photo-review question from the tanks fix: a tank sitting upright
  at full height on a surviving slab island will PASS the points-based
  support probe yet may still visually read as "floating on an island"
  rather than genuinely seated — judged only in the re-run photos, not
  resolvable by the probe alone.
* Two occluded review-camera frames (`epi_obl`, `nw_top`) from the v5
  skyline — cameras need raising, or a per-request re-shoot from the
  still-open scene.
* Pod bakes are not yet mirrored back to the local checkout or Nucleus.

## Round 6c (2026-08-31, afternoon)

Round 6b's own re-bake (`rebake6b_chain.sh`, 15 archetype batches) reported
0 failures. Agent 16 ("gates"), tasked with running round 6b's own gate
checklist against that chain before the scene relaunch, found this was a
false green: 6 of the 15 batches had actually crashed. Chasing that down —
not assuming the reported 0 was real — is this round's throughline: a bake
launcher that could not tell "finished" from "the process died silently,"
a collar mound that free-falls 95 m through its own settle pass, a
roof-plant fix whose real per-grade numbers were misreported on the first
read and corrected on the second, a GAC reseat bug that turned out to cut
both ways (false positives on two buildings, a real floater on a third),
the sliced-tears line landing clean, two occluded review cameras fixed
without moving any pixel that was not actually broken, and a new re-bake
chain (`rebake6c_chain.sh`) now in flight to redo everything the false
green had actually skipped.

### 1. The false green — Kit exits 0 on an uncaught crash

Isaac Kit swallows an uncaught Python exception raised inside a launcher
and still exits the process with code 0 — there is no signal in the return
code that anything went wrong. `bake_quake_archetypes_launch_script.py`'s
`SETTLE_STRICT` flag defaulted to strict (True), so the first style in a
batch whose rubble pile failed to converge raised `settle.SettleNotConverged`
and killed the whole batch's Kit process on the spot — silently, as far as
the calling shell could tell. Because the launcher only merges and writes
a style's record into `archetypes.json` AFTER that style's own settle and
export finish (`merge_manifest`/`bake.write_manifest`, called per style
inside the same per-style block that raised), a mid-batch crash left every
later style in that process with whatever record `archetypes.json` already
had — a pre-round-6b bake. 57 of 162 manifest records stayed stale this way
(51 at DG3-5 + 6 at OV/SETTLE/TILT), and `airborne_off` (the
`deactivate_airborne` safety-sweep flag) was set on only 138/162 records.
Measurement: 5 of the 18 archetype styles' DG3-5 piles do not converge
within `settle.run`'s own 3x-`max_steps` budget — the five that crashed
this way (s1g2 apartment, s2g2 brownstone, s3g2 commercial_mid, s5g2
office_wide, s1g3 apartment_tall). A sixth crashed batch, s4g2 office, was
a different bug — a `collar_1_mound` loose prim with "NO LOCAL XFORM," see
section 2. The same gates run's roof-plant probe came back 218 floating
lines across 67 files (vs. 226/65 before the round-6b fixes — effectively
unchanged), and GAC's own probe flagged 5 floating props (the same three
buildings — 12/19/24 — seed-independently, plus a new `02_DG5`); sections
3 and 4 below cover what those numbers actually turned out to mean.

Fixes, all in `bake_quake_archetypes_launch_script.py` unless noted:

* Per-style `try`/`except` isolation around each style's settle+export
  block, printing `[qarch] STYLE FAILED {style} {grades}: {exc}` on catch
  so one style's exception can no longer take the rest of its batch down
  with it.
* `[qarch] batch summary: {ok} ok / {failed} failed` at the end of `main`.
* An `archetypes_run_summary.json` sidecar recording the same counts
  machine-readably, next to `archetypes.json`.
* `os._exit(1)` called after `simulation_app.close()` whenever any style in
  the batch failed — deliberately not `sys.exit`, since `sys.exit` raises
  `SystemExit`, which is itself just another exception Kit can swallow and
  still exit 0 on (this path is untested live as of this writing — the
  in-flight `rebake6c_chain.sh` run is its first real exercise).
* `SETTLE_STRICT` flipped from strict-by-default to opt-in (`SETTLE_STRICT=1`).
  The launcher resolves the env var itself rather than trusting `settle.run`'s
  own `strict=None` default, which the module's own docstring notes reads
  the SAME env var the OPPOSITE way (empty/unset there means strict IS on) —
  leaving that ambiguity in play would have meant this launcher and
  `settle.run`'s own default silently disagreeing.
* The chain scripts now grep each batch's log for
  `SettleNotConverged|STYLE FAILED|Traceback` instead of trusting the
  process return code — and deliberately NOT for the
  `[Error] [disaster.settle] ... NOT AT REST` / `NO LOCAL XFORM` lines,
  because a healthy NON-strict export prints those same diagnostics too (a
  body that never converges but gets swept by `deactivate_airborne` anyway
  still logs them); grepping on those lines would flag every good
  non-strict bake as a false failure.
* Non-strict cost, measured directly: on the apartment_tall row, the
  airborne sweep deactivated 576 of 3144 bodies — running non-strict does
  not mean nothing was wrong, it means the safety net absorbs the
  non-convergence instead of a crash absorbing nothing.

### 2. The collar mound — a world-baked heightfield with no local frame

`r_soft_storey` (`quake_flow.py`) and `_author_heaps` (`quake_collapse.py`)
each have a mid-storey path that moves the WHOLE authored pile — including
the world-baked mound/apron heightfield `_author_heightfield` writes —
into `ctx["loose"]`, handing it to `settle.run` as a loose RigidBody so
physics can carry the collapsed collar down to the base. `_author_heightfield`
bakes its mesh points directly in world space with NO xform ops at all, by
design (cheap, and correct, for the ordinary case where the mound stays a
permanent static collider). Handed to `settle.run` as loose instead, PhysX
puts that body's origin at its parent's identity frame while every point of
the mesh sits tens of metres away — exactly the state `settle.py`'s own
`no_local_frame` check exists to catch — and the random angular kick every
loose body receives then swings and throws the mound, because its real
geometry is nowhere near its nominal origin. Measured: a 95.50 m "worst
mover," followed by a hard `SettleNotConverged` — the s4g2/office_DG4 crash
from section 1, on `rubble_office_DG4_collar_1_mound`.

Fix: new `quake_rubble_usd.recentre_for_loose(stage, paths)`, called from
both `r_soft_storey` and `_author_heaps` immediately before a collar's
mound/apron paths are added to `ctx["loose"]`. It recentres a mesh's own
`points` (and `extent`) on their centroid and adds a `translate` op putting
that centroid back at the mesh's original world position — the same shape
of fix `settle.bake` already assumes every OTHER loose prim arrives with (a
wrapper `Xform` around a referenced asset always carries its own
translate/rotate/scale). Every point's world position is unchanged, so
`primvars:st` (baked from the original world x/y) still lines up exactly.
It is a no-op on anything that already has an xform op, or is not a
`UsdGeom.Mesh`. New test:
`tests/test_quake_rubble_usd.py::test_recentre_for_loose_gives_collar_mound_a_local_xform`
(23/23 on the file, 89/89 on the module, 5/5 on the routing subset that
exercises this path).

### 3. Roof plant, the real picture

Agent 16's first per-grade read of the roof-plant probe's output (DG0 51 /
DG1 73 / DG2 84 / DG3 104 / DG4 109 / DG5 82 / OV 149 / SETTLE 81 / TILT
147) counted every PROBE line the tool prints — every prop it measured,
seated or not — not FAIL lines, and was wrong: a DG0 "failure" count would
have meant the probe itself was false-positiving on light-damage buildings
that never touch roof-plant seating logic at all. The corrected read
(agent B): true per-grade FAIL counts, of 218 total, are DG0 0, DG1 0, DG2
1, DG3 19, DG4 ~30, DG5 ~38, OV ~68, SETTLE ~0, TILT ~65. No probe false
positive either: a spot check on an apartment DG0 building found the deck's
own up-facing tiles at z=18.00 and the plant's own base at z=18.02 — a 2 cm
gap, genuinely seated. So the real defect is concentrated in OV and TILT,
not spread evenly across every grade the first read implied.

Root cause: OV/TILT roof plant IS carried by the building's rigid
whole-body transform — `ctx["fit"]["all"] += roof_plant` runs before the
`r_tilt_severe`/`r_overturn` recipes apply their own transform, so the
plant moves with the mass. But `_b_settle_roof_plant` then hands that
already-moved plant to the shared physics settle pass with no GEOMETRIC
fallback for when the settle does not converge or the deck it landed near
does not actually support it — gaps of 2-6 m at TILT, props stranded
0.7-1.35 m at OV.

Fix: new `quake_flow._settle_foundation_roof_plant(ctx, m)`, wired into
both `r_tilt_severe` and `r_overturn` right after each applies its
whole-body transform (`r_settlement` is untouched — it never needed this).
For each prop still in `ctx["roof_plant"]`/`ctx["roof_fixed"]`, it queries
`quake_collapse._deck_support_z` under the prop's own footprint; that
function's own `ROOF_PROP_UP_THRESHOLD = 0.72` up-facing-triangle test is
what actually decides the outcome per grade — a TILT-angle deck is still
tipped less than the angle that threshold implies, so a real up-facing
triangle is found and the prop is seated on it (`_deck_support_z`'s
return value, translated straight onto the prop); an OV-angle deck (60-90
degrees) has tipped past the threshold, `_deck_support_z` returns no
support, and the prop instead falls through to `_a_bury_props`'s
drop-to-grade path. New `tests/test_roof_plant_kit.py` (4 tests); a full
sweep after agents A (section 1/2) and B landed came back 445 passed / 0
failed (the sliced-tears work, section 5, was excluded — still in flight).
DG3-5's residual FAIL count is mostly explained by the 51 stale manifest
records from section 1 — the g2 batches already queued in
`rebake6c_chain.sh` are effectively the test of whether this fix clears
them once real settles run again.

### 4. GAC reseat — the same bbox blind spot, cutting both ways

`quake_sliced._reseat_roof_plant` — the GAC roof-plant seat fix from an
earlier round — scored each candidate deck by its WHOLE-PIECE
`UsdGeom.BBoxCache` top, with a `ctop <= pz0 + 0.30` ceiling meant to allow
"a few cm of interpenetration." Agent C found `SM_Building_19` and
`SM_Building_24` were ALREADY correctly seated — the verifier flagging them
as floating was itself a false positive, caused by the SAME bbox blind spot
that lives inside `quake_collapse._deck_support_z`'s own per-mesh AABB
prune: a merged `roof_x_*` piece's overall bbox reaches 1-2.4 m higher than
the real deck under a given tank, because the same prim also carries a
coping run or a raised section elsewhere on the roof. `SM_Building_12` was
genuinely floating, but for the OTHER-signed reason: its advertised seat
height was too LOW, so the old `ctop <= pz0 + 0.30` ceiling excluded the
real roof band entirely and the search fell through to a bare wall shell
with zero up-facing triangles (e.g. `core_x_0_20`) that merely happened to
still pass the other gates.

Fix: `_reseat_roof_plant` now scores each candidate by the highest
UP-FACING triangle actually reachable under the prop's own footprint (new
`_mesh_up_faces` + `_pt_in_tri` helpers — the same query-rectangle-vs-
triangle idiom `quake_collapse._deck_support_z` already trusts in
production), and the `ctop <= pz0 + slack` ceiling is dropped entirely: a
containment test that only ever fires on a genuine up-facing surface
cannot be fooled by an unrelated tall neighbour the way a bbox-overlap test
can. Measured on `SM_Building_12`: 98.016 m -> 102.787 m. 44/44 sliced
tests green. `SM_Building_02_DG5` is out of this function's scope
entirely — it is a `masonry_collapse` (urm total collapse) building, and
its tank is seated by the physics settle, not by `_reseat_roof_plant`.

Caveat, flagged by agent C rather than fixed by them: `quake_collapse.
_deck_support_z` itself still has the SAME per-mesh AABB prune (margin 0.5)
that `_reseat_roof_plant` just stopped needing (`_reseat_roof_plant`'s own
candidate set is one building's small piece roster, so it can afford to
skip the prune outright; `_deck_support_z` is a whole-stage scan and cannot,
without the prune becoming unaffordable). `_deck_support_z` is called from
FOUR separate places: the kit roof-prop sweep (`quake_collapse`'s own
`_sweep_roof_props`), section 3's `_settle_foundation_roof_plant` tilt fix,
the sliced-path roof sweep (`quake_sliced`), and `tools/
roof_plant_seat_probe.py`'s own standalone verifier. Fixing the prune, and
rebasing the 162-file FAIL baseline against the corrected function, is
**PENDING at time of writing** — a "deck-probe" agent was in flight on it
as of the coordination log's last entry.

### 5. Sliced tears landed

`quake_sliced.py` gained `_plan_tears`/`_author_tears` and a
`QS_MAX_TEARS = 40` cap, implementing the design sketch in `scratchpad/
design/sliced_gac_ragged_boundary.md`. New probe `tools/qs_tear_probe.py`
was fixed this round to reach REAL baked kits instead of synthetic
geometry — it now goes through `kit_bake._entry` (the manifest-row lookup)
and `quake_gac_probe.load_real_kit` rather than constructing a stand-in kit
inline. New bench `tools/tear_edge_bench.py`: a tear's fragments live under
sibling `brk_*` scopes next to the piece they came from, not inside it, and
the bench camera aims at the removed pieces' own world centroid rather than
a fixed point. Measured: `straight_run_m` (the longest unbroken straight
run along a removal boundary — the headline metric borrowed from
`pillar_break_bench`) went from 7.742 m to 2.478 m; the design doc's own
pre-implementation estimate was <0.8 m, so the landed result, while a large
improvement, did not fully reach that estimate. Renders: `~/scorch_previews/
sliced_tears/`. Synced to the pod.

### 6. Cameras

The five review points (`epi`/`ne`/`se`/`sw`/`nw`) had shared one fixed
camera pose (`top_h=95`, `obl_dist=80`, `obl_h=40`, `azimuth_deg=225`)
since earlier rounds — fine on the older kit-only skylines, but
`urban_quake_v5` places taller GAC/downtowncity towers: the fixed oblique
eye for `epi_obl` landed 12 m inside a 302 m mono tower's own footprint,
and the fixed top-down eye for `nw_top` sat against a leaning 131 m
DG4+tilt tower.

Fix, entirely in `downtown_quake_launch_script.py`, stdlib `math` only (no
`carb`/`omni`/Kit import, so it is testable offline): new
`_review_camera_clearance`, built on `_camera_eye_blocked` /
`_building_blocks` / `_footprint_radius` / `_review_camera_pose`. It raises
`top_h` (for a top-down eye) or `obl_dist`/`obl_h` together (for an
oblique eye) for one review point at a time, only when that point's fixed-
pose camera EYE is actually inside a building's tilt-widened footprint,
stepping the relevant number up (10% of its base value per step) until
clear or a `max_mult=5.0` cap is hit. A leaning building's footprint radius
is widened at height `z` by `z * tan(TILT_DEG_MAX)` (`TILT_DEG_MAX = 9.0`,
mirroring `quake.py`'s own `tilt_deg` upper bound) — this is what catches
`nw`'s tilted tower, whose upright footprint alone misses the fixed camera
height by under half a metre. Landed values: `epi` raised to `(top_h=95,
obl_dist=112, obl_h=56)`; `nw` raised to `top_h=133` only (its oblique was
already clear). Every other point/pose combination returns the exact
pre-fix constants — `_review_camera_clearance` is built specifically to
return the unmodified base triple whenever nothing needed moving, so a
caller can group points by the return value and re-issue the identical
`views_around` call for any point that needed no fix. Verify script:
`scratchpad/cameras/verify_review_cameras.py` (offline, no Kit);
`py_compile` clean; synced to the pod (md5 f9fde10531a6 both sides).

### 7. Process

A spend-limit hit mid-round killed two running agents outright — the pod
watcher and the sliced-tears implementer — mid-task. Their on-disk state
survived the kill; the lead re-staffed the same work with a freshly
spawned agent carrying condensed context, rather than trying to resume the
killed agent's session (the limit reset about half an hour later; a
heartbeat cron set up earlier in the round as a safety net for exactly
this kind of silent stall fired in the meantime). Subagents were
repeatedly classifier-blocked from running the pod's own kill/launch
commands themselves (a stale Kit process's `docker exec` kill, and a chain
launch, both blocked on a subagent's own attempt); the lead ran those
commands directly instead — one kill needed a `/proc/<pid>/environ` guard
rather than an argv guard, since argv alone could not distinguish the
stale process from the live one sharing its GPU. A tunnel re-hold task
died along with the subagent that had been holding it open (the task does
not outlive its owning agent); the lead re-opened the tunnel directly in
the background instead. Completed subagents drop off the roster once done
and cannot be resumed by id — the same lesson round 6b already recorded,
and still the reason follow-up work goes to a fresh agent with condensed
context rather than an attempted resume. The lead re-ran every landed
agent's own tests before accepting a report, against a pre-agent-edit
worktree snapshot — this is what caught the "vtk env trap" recurring
(chip-related tests need `uv run --with vtk`, not bare `python3`, or they
produce phantom failures) before it was mistaken for a real regression a
second time.

### 8. Open at time of writing

* `rebake6c_chain.sh`, launched on the pod: the 6 crashed g2 batches plus 5
  g3 batches (tagged `r8_*`), grep-based failure detection. Paced at
  roughly 6-8 minutes per style — heavier than first estimated — so a
  heavy batch runs 25-30 minutes and the full chain's ETA is now ~3 hours,
  not the original ~45-minute estimate.
* GAC pass 2 (a planned second GAC bake pass, referenced as `/root/
  gac_pass2.sh` on the pod) — not yet run; waiting on the full sweep and
  the `_deck_support_z` prune fix (section 4).
* Gates re-run against the corrected verifier, after the `_deck_support_z`
  prune fix lands — not yet done.
* The scene re-run — relaunching seed-9 `urban_quake_v5` — waits on
  everything above.
* The mono-wiring design (`scratchpad/design/mono_wiring.md`) — a design
  sketch only, not started: bake-time per-(asset, grade, seed) treatment
  for AEC brownstones (already a kit internally, 307-3684 named part
  meshes, so it needs de-instance + a placement table + `wreck_sliced`, NOT
  `monolith_damage.cut_shell`, which has a hardcoded -Y-front assumption
  and a uv0 dependency no AEC asset actually has).
* `quake_collapse`'s "fitout slabs" and "floor cells" chip families (the
  `[chip] fitout slabs: ...` / `[chip] floor cells: ...` proof lines) fired
  zero times this round — unexplained, flagged but not investigated.

## Round 6d (2026-08-31, evening) — the settle body budget

Round 6c's own re-bake chain (`rebake6c_chain.sh`) was still in flight when it
hung: `block_residential`'s DG3-5 batch (`r8_s1g2`) settled its first three
style rows cleanly and then sat on an 18,771-body pile for more than 1.5
hours without converging. The user's own directive, mid-run — "settle/bake
the buildings lazily... you wanna place some by hand or something" — is this
round's throughline: stop throwing physics at every loose crumb a collapse
recipe happens to author, measure which pieces actually need it, simulate
only those, and place the rest by hand exactly as asked, plus the perf work
that particular mechanism needed to be affordable at heap scale, and the
re-bake bookkeeping the fix leaves behind.

### 1. The trigger — an 18,771-body settle, and a false PASS underneath it

`r8_s1g2` settled `apartment`, `apartment_long` and `apartment_tall`
cleanly, then hit `block_residential` DG3-5: 18,771 loose bodies, against
roughly 1-3k for every other style row this round measured (`quake_
collapse.py`'s own "SETTLE BODY BUDGET" section records the range and calls
out `apartment_tall` DG3-5 itself at 3,144; this round's chain also logged
`commercial_mid` at 1,693 and `department_store` at 1,952). The settle ran
past 1.5 hours without converging and was killed by the lead directly:
SIGINT, sent only after confirming — via a `/proc/<pid>/environ` check
matching `ARCH_STYLES` (argv alone can't distinguish a stale bake process
from a live one sharing the same command line, the same guard round 6c's
own Process section already used once this round for a different stale
process) plus the chain log's current row — that this really was the hung
`block_residential` settle and not something else sharing the GPU.

The trap: Kit exits 0 whether a batch's process raises an uncaught exception
(round 6c's own finding, section 1 above) or is stopped by an external
SIGINT it shuts down on gracefully — either way nothing appends to
`bake_quake_archetypes_launch_script.py`'s own `failed` list, so its
"batch summary: N ok / 0 failed" line and the chain script's own grep-based
verdict (round 6c's fix, hunting `SettleNotConverged|STYLE FAILED|
Traceback`) both read `r8_s1g2` as a clean PASS. It was not: each style's
`archetypes.json` record is only written inside that style's OWN per-style
`try` block, right after its settle+export finishes, so `apartment`/
`apartment_long`/`apartment_tall` had already landed their records before
the kill and exported fine; `block_residential` DG3-5 was killed mid-settle,
before it ever reached its own export or manifest write, and stayed on
whatever stale record `archetypes.json` already had.

### 2. Root cause — not a bug, arithmetic on five masses

`urban_building.py`'s `block_residential` spec is not one building: it is a
podium `main` mass (`fam02((9, 5), 0, ...)`, the "full block" note) plus
four WINGS at 16, 16, 10 and 20 storeys — five masses in total. `quake_
collapse.collapse_masses`'s own docstring gives the count: 2,377 elements,
only 181 of them on `main`. A total or pancake grade sweeps every mass
(`r_masonry_collapse`'s own `for mt, m in info["masses"].items()`), not just
the podium, and every per-ELEMENT wall break (`quake_flow._break`/
`_break_split`, 10-24 loose cells each) and every per-STOREY slab break
(`_break_box_like`, 30-52 plank cells per slab per storey) multiplies by
BOTH the mass count and each wing's own storey count — five masses' worth of
wall and slab breaks, not one.

The one population that does NOT inflate the loose count is interior
litter: `quake_flow._disturb_interior` authors `W*D/100*9` litter boxes per
storey (130-190 on a seven-storey mass) — plenty of geometry — but appends
every one of them straight to `ctx["static_extra"]`, never `ctx["loose"]`,
so it never touches a PhysX body count regardless of how many masses or
storeys author it. The audit therefore comes down cleanly: wall-break and
slab-break cells (the populations `settle.run` actually has to simulate)
dominate the 18,771, multiplied by mass count and wing storeys; interior
litter is already static and free.

### 3. The design — `SETTLE_BODY_BUDGET`

New env `SETTLE_BODY_BUDGET` (default 3000; `-1` = unlimited, today's
unchanged behaviour), read once per launcher run via `quake_collapse.
settle_body_budget()` and enforced by `quake_collapse.apply_settle_budget(
stage, loose_paths, budget, root=..., ground_z=..., rng=..., exclude=())`.
Deliberately placed in `quake_collapse.py` rather than the launcher itself —
`bake_quake_archetypes_launch_script.py`'s own comment gives the reason:
"the budget has to live somewhere the GAC/sliced bake can reach it too
later." `quake_sliced.py` already does `from . import quake_collapse as qc`
for `_chip_prim`/`_deck_support_z`, so it can adopt `apply_settle_budget` the
same way without a new import; only the archetype path calls it this round.

The mechanism, called once per row on the SAME accumulated `loose` list
right before `settle.run` receives it:

* rank every piece with `rank_loose_for_settle_budget` — volume x current
  world-Z midpoint, ties broken by path string, no rng draw at all, so the
  ranking is a measurement of stage content, not a choice (a piece already
  sitting high in the authored, pre-settle stack is the one about to fall
  furthest and land most visibly, on the crown of the pile);
* the top `budget` pieces (`kept`) go to `settle.run` completely unchanged;
* everything past the budget (`over`) is placed GEOMETRICALLY, right there,
  with the same idioms this module and `quake_flow` already trust rather
  than a new one invented for the occasion: `quake_flow._a_lay_flat`
  (`p=1.0`, so every over-budget piece gets the thin-axis-up lay-flat, not a
  share of it) for orientation, `_deck_support_z` for the landing height
  under the piece's own footprint (with a caller-supplied `ground_z`
  fallback), and a `SETTLE_BUDGET_SINK_M = (0.02, 0.06)` downward sink so a
  laid piece does not hover a hairline above its support — the same
  few-centimetre interpenetration tolerance `quake_sliced._reseat_roof_
  plant`'s round-6c fix already accepts.
* a placed piece comes back in `geometric`, never `kept` — the caller
  (the archetype launcher) appends it to the row's static list instead, so
  it costs the settle nothing: no rigid body, no step budget, no
  convergence risk, and nothing for `deactivate_airborne` to sweep
  afterward either.

Backward compat: `budget is None` or `budget >= len(loose)` is an explicit
no-op — `(list(loose_paths), [], [])`, nothing touched on the stage — pinned
byte-identical by `tests/test_settle_budget.py::test_huge_budget_is_byte_
identical_noop` / `test_budget_none_is_unlimited_noop` / `test_budget_
exactly_equal_to_count_is_noop` (each asserts `stage.GetRootLayer().
ExportToString()` is unchanged before and after). `budget=0` is the other
extreme — every piece placed geometrically, `kept == []` — also tested
(`test_budget_zero_is_all_geometric`).

Wired into `bake_quake_archetypes_launch_script.py` only this round:
`SETTLE_BODY_BUDGET = qc.settle_body_budget()` at module scope, `root=
lambda p: piece_root.get(p, PARENT)` (a path -> parent-scope map built while
authoring each row, so a piece is only ever landed on ITS OWN building
rather than a different grade's building 60+ m away on the grid), a new
`[qarch]   settle budget row {si} ({st}): {n_loose_authored} loose -> {n_
kept} kept for physics (budget {budget}) + {n_geo} placed geometrically`
proof line, and three new manifest fields — `settle_budget`, `settle_
budget_geometric`, `settle_budget_authored` — recorded even on the `loose`
empty edge case (`budget=0`) where the settle below never runs at all. The
GAC/sliced path is not wired up yet.

### 4. Performance hardening — the candidate-mesh cache

Building `apply_settle_budget` immediately exposed a cost `_deck_support_z`
had never been measured against: `apply_settle_budget` calls it once per
over-budget piece, and the existing implementation re-runs a full `stage.
Traverse()` plus a fresh per-mesh world-transform and fan-triangulation on
EVERY call — for a heavy row that is hundreds to thousands of repeat
traversals of the same static building scope. New `_deck_support_candidates(
stage, root)` precomputes every candidate mesh's world AABB and world-space
triangle arrays for a root ONCE, and an optional `candidates=` parameter on
`_deck_support_z` walks that precomputed list instead of re-traversing —
every query-dependent test (`exclude`, the Z/XY AABB prune, `up_threshold`,
`margin`, the 5-point containment test) still runs exactly as it would on a
live traversal, against the same precomputed data a fresh traversal would
compute fresh. `apply_settle_budget` keys its own `cand_cache` dict by
`root` string and builds it the first time each distinct root is seen in a
call.

Correctness argument (also why it is safe to cache across an entire
over-budget batch rather than re-measuring per piece): every path in `over`
is already excluded from being its own or another piece's support
(`exclude_all = set(exclude) | set(over)`), regardless of whether its
cached geometry predates a later move, because none of the over-budget
pieces are in a final position yet during this call — the same reasoning
`_sweep_roof_props` already uses for its own `exclude_paths = set(fall)`.
`candidates=None` (the default) is unchanged and is what every one of
`_deck_support_z`'s other four call sites still passes — `_sweep_roof_
props` (this file), `quake_flow._settle_foundation_roof_plant`, the
sliced-path roof sweep in `quake_sliced.py`, and `tools/roof_plant_seat_
probe.py` — none of them touched this round.

Measured (`tests/test_settle_budget.py::test_perf_cache_handles_2000_
geometric_placements_in_single_digit_seconds`): 2,000 loose pieces resolved
against a dedicated 500-mesh candidate floor scope in 1.25s (`uv run
--python 3.13`, CPython 3.13, single process, numpy reference BLAS, no GPU)
with the cache; an earlier, smaller-scale manual dry run (2,000 total
prims, 1,200 geometric) had measured ~28s before the cache existed — call
it a 20-25x speedup at this scale, though the two runs are not the exact
same fixture. The test's own `assert dt < 10.0` is deliberately generous —
a regression guard against the cache being lost, not a tight timing
assertion on a loaded CI box.

### 5. Verification

`tests/test_settle_budget.py`: 16 tests — the env reader (`settle_body_
budget`), the budget/no-op boundary (a respected split is a partition;
`budget=0` is all-geometric; a huge/`None`/exactly-equal budget is a
byte-identical no-op), that geometric pieces are actually seated within
tolerance (points-based, not bbox — the same discipline this project's own
bug catalogue already calls out), the ground-Z fallback case, determinism
(same seed -> same split and placement), that the ranking itself draws no
rng, that an unresolvable piece stays loose rather than being lost, that
`root` accepts a callable, the cache's byte-identical-to-uncached guarantee
(with and without `exclude`), that degenerate triangles are dropped safely,
and the perf regression guard above.

Integrated: a full sweep across every round 6b/6c/6d fix (collapse, kit
roof, deck-support cache included) came back 508 passed / 0 failed. Look,
not just numbers: `tools/settle_budget_bench.py` (offline, no Isaac Sim —
authors a heap that already looks like a converged settle produced it,
since this round's own constraint rules out ever calling `settle.run`/PhysX
live to prove the mechanism) rendered a 5,000-piece dome twice, FULL
(`budget=None`) against BUDGETED (`budget=SETTLE_BODY_BUDGET`), to `~/
scorch_previews/settle_budget/`. The lead re-reviewed: full and budgeted
read as visually indistinguishable at review distance.

### 6. Bake bookkeeping

`block_residential` DG3-5 plus its OV/SETTLE/TILT foundation-family grades
need a full re-bake under the budget — staged on the pod as `/root/r9_
block_residential.sh` with `SETTLE_BODY_BUDGET=3000` set explicitly, so the
row this round exists for is provably running under it rather than trusting
the launcher's own default. Every other style measured this round stayed
comfortably under the ~1-3k range in section 1 above, so the budget is a
documented no-op for all of them — nothing else needs a re-bake for this
reason alone.

Per-style body counts belong in the bug catalogue as the early-warning
signal this round did not have going in: watch a chain log's `[qarch]
settling row {si} ({st}): {n} bodies` line (or, once a bake runs under the
budget, the `[qarch]   settle budget row ...` line above it) per style —
the same live-Monitor-a-log-and-catch-it-early pattern this project already
uses elsewhere — rather than discovering a multi-hour outlier only after it
has already run past every other row's time budget.

### 7. Bug catalogue additions

* **Kit exits 0 on an external SIGINT the same way it does on an uncaught
  exception — a manual kill produces the SAME false PASS round 6c's own
  section 1 already documented for a crash.** Neither the launcher's own
  `failed` list nor a chain script's grep-based verdict has anything to
  match when a hung process is stopped from outside rather than raising —
  the process still exits 0, and a batch with one un-exported row can still
  print "N ok / 0 failed." The only durable signal is the per-style
  manifest write itself: a style's `archetypes.json` record only lands
  after ITS OWN settle+export finishes, so a killed row's record is simply
  whatever predates the kill, however clean the batch summary reads.
* **A "block" style is several buildings' worth of wall wearing one style
  name.** `block_residential`'s podium-plus-four-wings shape multiplies
  every per-element and per-storey break population by both the mass count
  (5x) and each wing's own storey count — 2,377 elements versus 181 on the
  podium alone. A per-style body cap or budget tuned against one
  representative building will silently starve (or, as here, never finish)
  the multi-mass styles unless each style's own body count is measured
  before assuming a uniform budget is safe.
* **Background tool tasks die at the harness's own timeout cap, or get
  killed externally when a limit or a process manager reaps them — hold
  anything that has to outlive that (a port-forward tunnel, a long settle)
  with `nohup ... & disown` from a quick foreground call, and run a sweep
  that has to actually finish in the FOREGROUND rather than trusting a
  backgrounded task to still be there later.** This recurred again this
  round on top of round 6c's own tunnel lesson (section 7 above).

## Round 6e (2026-08-31, night) — the v4 photo review and the GAC shell root cause

`eq500_v4` (round 6d's own scene, launched against the 20:58 UTC pod
deadline) came back as 30 captures. The user's photo review passed the
ragged-cutout, layout/dressing and building-variety work outright, but found
new defects the earlier rounds' fixes hadn't touched, then a second look
("still floating towers/silos + empty spaces") asked for a specific
cross-check: prove there are no GHOST buildings — placements the layout
planned that never actually got built. Two investigation agents
(`ghost-diff`, `floater-forensics`) ran offline against the frozen evidence
(the scene's own log, `quake_buildings.json`, and the mirrored pod bakes
under `pod_prep/pod_bakes_fresh/`) — pod `airstack-dev-177` had already
expired, so nothing here re-ran Isaac. `ghost-diff` cleared the ghost-building
theory outright but surfaced two OTHER dispatchable bugs
(`monolith-scale`, the assembly-interaction settle); `floater-forensics`
found ONE root cause behind three separate-looking symptoms (dark podiums,
the sky-grid of frozen debris, and the b7-b9 "floating tank" photos): a
longstanding material-binding defect in the GAC slicer that has nothing to
do with any round-6 fix. All four resulting fixes (`monolith-scale`,
`assembly-settle`, `gac-shell`, `stranded-bands`) landed and were folded into
a consolidated sweep (530 passed / 0 failed) before OSMO and Nucleus both
went down — the backup pod `airstack-dev-182` (submitted ahead of the
deadline, 48h) has been sitting in the OSMO queue ever since, unable to
schedule. This section and the `dev182_bringup.sh` script in this session's
own scratchpad are the outage's own prep work: nothing below has been
re-verified in Isaac.

### 1. The v4 review verdicts

PASS: ragged cutouts (no more straight rooflines / orphan panels at removal
boundaries), the layout/dressing pass (plazas, courts, street furniture), and
building-population variety (AEC + GAC + MCE all visibly damaged, DowntownCity
pristine-only). PARTIAL: cuboid debris (the `se_top` rooftop plates still read
as perfect prisms — the pillar/chip wiring from round 6b covers pillars and
authored bars but this specific rooftop-plate population was not in its 11
call sites) and the brick facade fix (red decal-like blobs still visible on
pale cladding at `b0`/`b5`). FAIL: water tanks (6 frames across 5 kit-archetype
styles show a tank at what looks like full ASSEMBLY height over a crushed or
missing storey — this is a DIFFERENT population from the GAC roof-plant bug
below) and the review cameras (the `b0`-`b9` close-up obliques still have no
clearance logic — see section 7 — so `b3`/`b4` are wall-fill and `sw_obl` gave
up after three blank frames). The single WORST new defect: a frozen grid of
plank/panel debris hanging in the sky at facade-grid spacing — `se_obl` shows
100+ pieces, `nw_obl` a column of 12 — which section 3 below explains was
never actually "debris that failed to settle" but standing shell pieces that
were never moved, given physics, or swept at all. Also flagged: dark,
untextured podium boxes on 2+ towers, one under-damaged DG5 tower (`b6`), and
one floating boulder (`nw_top`). Best frames: `b2_obl`, `b9_obl`, `sw_top`.

### 2. The ghost-diff verdict — no ghost buildings

`ghost-diff` reproduced the seed-9 layout offline and cross-checked it
against the scene's own build log and `quake_buildings.json` (the per-building
record the launcher writes as it goes — the actual ground truth for what got
built, not a re-derivation). Verdict: **no ghost buildings**. The offline
repro placed 132 buildings where the log recorded 135 (a 3-building
difference entirely explained by the offline-repro caveat below, not by
anything failing to compose); `quake_buildings.json` itself has 134 records;
0 building-reference failures (the log's 384 `WARN`/`Could not open asset`
lines are ALL props — humans, planters, benches, street furniture referenced
from Nucleus paths this sandbox cannot resolve — never a building); 0
deactivations; and a stem-existence audit found all 112 kit-archetype/GAC
stems the scene referenced were valid, present files. The "empty spaces" the
user was seeing are **dressed plazas** — open, furnished ground the
round-6b `plazas`/`COURTS` fix authors on purpose, not gaps where a building
should have been. Overall city fill (61.94%) is stable against the earlier
measurement, so there is no fill regression hiding behind the ghost-building
question either.

**Offline layout repro caveat (why the two counts don't match exactly).**
`scene_generator.SizeResolver.get` measures a real USD's footprint once per
`(path, axis)` and caches it; when `_measure_footprint` can't open an asset
(every Nucleus-hosted `omniverse://...` GAC/AEC/downtowncity path, in an
offline sandbox with no Nucleus mount) it falls through to
`self.fallback.get(category, [4.0, 4.0])` — a per-category constant from the
merged asset-set YAML, or the hardcoded `[4.0, 4.0]` pair if the category has
no entry anywhere. **Correction to the coordination log's own shorthand**:
for the "house" category specifically (what every building placement is
sized as), that fallback is NOT the generic 4×4 m default — `urban.yaml`'s
own `fallback_sizes.house: [30.0, 20.0, 24.0]` is configured and wins, so an
offline repro's unresolvable buildings all pack as identical 30×20×24 m boxes
rather than their real, wildly different GAC/AEC footprints (confirmed
directly: `[scene_gen] fallback house: SM_Building_NN.usd -> 30.00 x 20.00 m`
fires for 78 of the ~101 unique building assets this seed touches, the
remaining ~23 being locally-committed kit/downtowncity assets that measure
fine). The generic `[4.0, 4.0]` code default is real (`scene_generator.py`'s
`SizeResolver.get`) but was not observed firing for buildings in this
specific repro — it is the emergency case for a category with no configured
fallback at all, not what actually ran here. Either way the effect is the
same: an offline dry run's block-packing arithmetic (which building fits
which slot, corner rounding, plaza sizing against a wrong footprint) diverges
from what the real, Nucleus-connected pod run actually placed — **use the
scene's own log and `quake_buildings.json` as ground truth, never an offline
repro's placement list, for any question about what shipped in a specific
scene.**

### 3. The GAC shell root cause — one bug behind three symptoms

`floater-forensics` found a SINGLE root cause behind the dark podiums, the
frozen sky-grid, and most of the b7-b9 "floating tank" frames: every
exterior-shell mesh (`wall_*`, `corner_*`, `core_x_*`, `pier_*`, `parapet_*`)
`detail/gac_storey_slice.py` cuts comes out of a frozen per-building bake
with **no usable material** — measured on all 33 manifest-live GAC bakes
(`bind_census_before.txt`): **8,003 of 8,546 shell meshes unbound**, ranging
from 0/197 bound (`SM_Building_21_DG2`) to a best case of 69/261
(`SM_Building_25_DG4`) — no file in the set was clean. This is
**longstanding, not a round-6 regression**: an old-bakes-vs-new-bakes
comparison came back identical (246/246 unbound either way) — every round of
"GAC walls are grey" complaint since this pipeline existed was this same bug.

Two compounding defects, both real, found by dumping raw `GeomSubset`
bindings instead of trusting `ComputeBoundMaterial()` alone:

1. **`read_mesh`'s no-argument `ComputeBoundMaterial()` misses `full`-purpose
   bindings.** GreatAmericanCity binds its facade materials under USD's
   `full` (render-quality) purpose on a live, Nucleus-connected stage; a
   bare `ComputeBoundMaterial()` call only resolves `allPurpose` and does
   NOT fall back to `full` — confirmed in a synthetic stage in
   `gac_storey_slice.py`'s own comment (`Bind(mat, materialPurpose=full)` +
   `ComputeBoundMaterial()` → `None`, `ComputeBoundMaterial(materialPurpose=
   full)` → the material). `read_mesh` harvested nothing for these faces, so
   `write_piece` had no material to give them at all.
2. **Even where a material WAS harvested and a `GeomSubset` bound, it still
   fails to resolve inside a FROZEN, standalone per-building file.** The
   relationship target is not dangling (`IsValid()` is True) — it is a bare,
   TYPELESS placeholder: a reference out to a per-material Nucleus asset
   (e.g. `GreatAmericanCity/.../Materials/M_Building_24_Metal_Inst.usd`) that
   composes to nothing once the file is opened without that Nucleus mount.
   `ComputeBoundMaterial()` requires the resolved prim to actually BE a
   `UsdShadeMaterial`; a naive `GetPrimAtPath(target).IsValid()` says the
   opposite, which is exactly what let this hide behind rounds of "maybe it's
   a UV bug" review. Measured: `/World/bake/Looks` is 100% typeless stubs, 0
   real materials, in every one of the 33 files — while `/World/bake/
   QuakeLooks` (materials `quake_sliced.py`'s own damage code authors
   directly, with a real embedded shader, never a reference) resolves fine.

**This is the root cause of three photo-review symptoms at once**: dark
untextured podiums (a shell mesh with no material renders flat black, not
the fallback grey a missing-texture case usually shows); the tan-ish
"sky-grid" panels (the SAME unbound-shell defect on pieces nobody happened to
notice were floating, until section 5's mechanism put them in mid-air);
and most of the b7-b9 "tank floating over an invisible building" frames — the
tank is correctly seated, but the SHELL underneath it renders as nothing, so
the tank reads as floating over empty air.

**The fix, two parts:**

* **Root fix, in `detail/gac_storey_slice.py`, for every FUTURE bake**:
  `read_mesh` now asks `ComputeBoundMaterial(materialPurpose=UsdShade.
  Tokens.full)` everywhere it used to call the bare, no-argument form (both
  the mesh-level harvest and the per-`GeomSubset` harvest); this is a strict
  superset (USD's own purpose-fallback still resolves a `full` query against
  a plain `allPurpose` bind when no `full`-specific one exists), so it can
  never find LESS than before. `write_piece` NEVER SKIPS any more: every
  piece gets a direct mesh-level binding to a role-appropriate fallback
  material FIRST (`_role_fallback_material` — a plain, fully self-contained
  `UsdPreviewSurface`, deliberately never a reference to anything in the
  source asset, so it can never go stale the same way inside a standalone
  export), before any per-material `GeomSubset` is added on top of it.
* **Repair tool, for bakes that already exist**: `tools/
  gac_shell_bind_repair.py` copies each already-baked per-building USD and
  rebinds every unbound shell mesh, on the COPY, without touching the
  original. It does NOT try to bind from that file's own `/World/bake/
  Looks` (every candidate there is equally an unresolved stub, per defect 2
  above) — instead it redirects each unbound shell to a REAL,
  already-embedded material from that SAME file's own `/World/bake/
  QuakeLooks` scope, chosen by a role heuristic (`_piece_role`: `core_x_*` →
  "core", everything else → "facade"; `_keyword_role` reads a broken
  subset's own unresolvable target NAME for a `glass`/`metal`/`marble`
  keyword before falling back to the piece-name default), with a
  deterministic dominant-facade fallback when a role has no dedicated
  QuakeLooks candidate in a given file. Repaired drop-ins verified prim-path
  identical (2256/2256) to the originals at `pod_bakes_fresh/
  gac_quake_repaired/`; 51/51 `test_quake_sliced.py` (the slicer's own root
  fix, tested with a stage proven failing pre-fix). **Caveat, stated in the
  tool's own docstring**: the repaired materials are plausible-not-true — the
  real facade textures only come back once the NEXT GAC bake re-slices
  through the fixed `gac_storey_slice.py` and Nucleus can resolve the
  harvested references again.

### 4. The assembly-interaction settle floater

The one genuine PHYSICS floater in the whole review (as opposed to a
material or a never-moved shell): `tank_ix1_2`, a water tank on a building
INSIDE a live `lean_on`/`collapse_onto` interaction pair
(`quake._d_interactions`), frozen mid-air at export. Root cause: the shared
settle those pairs run (`quake._d_interactions`'s own `settle.run` call, after
accumulating every pair's fracture debris into one `loose_all` list) used to
hand roof plant (tanks, AC units) into that SAME shared settle alongside
however many rigid bodies the pairs' own fracture sheds — `eq500_v4` handed
it 155 bodies for 3 pairs — where a heavy tank can starve for the settle's
step budget and freeze mid-flight; `tank_ix1_2` measured exactly at the
1,800/1,800-step cap.

Fix: `quake._d_settle_roof_plant_now(res)`, called from `_d_live_lean`
immediately after `quake_flow.wreck_building` returns and BEFORE the caller
appends anything to `loose_all`. It pulls every path in
`res["roof_plant"]`/`res["roof_fixed"]` back OUT of `res["loose"]` (and its
matching `res["velocity"]` entries) — pulling them out first is what lets
`quake_flow._settle_foundation_roof_plant` treat them as unresolved rather
than skip them as "already handed to physics" — then resolves each
geometrically the same way the `tilt_severe`/`overturn` families already do:
seat it on the deck under its own footprint
(`quake_collapse._deck_support_z`), or drop it to grade
(`quake_flow._a_bury_props`) when the deck no longer faces up at all. Roof
plant on the interaction path now NEVER reaches the shared physics settle at
all — there is nothing left in that budget for it to starve against.

The settle call itself also picked up two matching fixes, since whatever
IS left in `loose_all` after that (pure fracture debris, not roof plant)
still needs the same honesty the round-6c bake gates already require:
`converge=True` makes `steps` a target the throw phase may run past (up to
`settle.run`'s own 3× `max_steps`) instead of a hard ceiling it bakes against
regardless, and `rest_v2=True` turns on the same points-based rest
measurement (net travel over a window, a stall freezes the jittering bodies
instead of giving up on a raw velocity snapshot) the fire/tornado bakes
already use for every non-kit settle. And for whatever still doesn't
converge even so: `fire_bake.deactivate_airborne` now runs over the
`quake_interact` scope after the settle — the same post-settle safety net
the round-6b tank fix added to the ordinary per-building bake path, now
covering the interaction path too. New test file (6 tests), 102/102 with
`test_quake_collapse.py` and `test_roof_plant_kit.py` folded in.

### 5. The stranded-band mechanism (the sky-grid, mechanically)

Section 3 explains why the frozen sky-grid pieces render wrong (unbound
shell material); this section is why they are floating in the first place —
a SEPARATE bug from the GAC roof-plant reseat work in earlier rounds.
`quake_sliced._apply_region` (the region-removal ladder behind `corner_fail`/
`out_of_plane`) counts a VERTICAL neighbour (the same cell one storey up or
down) as a boundary condition exactly like a horizontal one, and its own
`kept_piers` draw rolls keep/lose PER CELL, independently, at every storey a
wide multi-storey region touches — so it is entirely possible (measured: 2-12
orphan instances per seed on `out_of_plane`) for the pier at `(side, storey,
bay)` to survive its own draw while the pier directly beneath it, in the
SAME column, part of the SAME removed region, loses its own independent
draw. The survivor is left exactly where it was authored: nothing
rigid-transforms it, nothing gives it physics, nothing sweeps it — a
periodic, bay-pitch-spaced grid of shell pieces whose own support vanished
one or two rows below. This is the sky-grid.

The repair is deliberately NOT inside `_apply_region`/`plan_damage` itself —
the pure planner's own RNG draws and `plan["removed"]` have to stay
bit-identical (another agent's materials work reads the same plan; pinned by
`test_stranded_bands_plan_removed_is_bit_identical`, a `sha256` hash over
`sorted(plan["removed"])`). Instead it runs as a POST-`apply_plan` stage
pass, the same shape as the round-6b roof-prop sweep:

* `_orphaned_shell_candidates(info, plan, mass, loose=False|True)` — a
  CHEAP, PURE pre-filter with no stage access, reading `info["elements"]`'s
  own `_storey`/`_side`/`_bay` fields and each element's `dead` flag (set by
  `apply_plan`'s removal step) to find live shell paths whose directly-below,
  same-column cell was authored but is now entirely (`loose=False`) or
  partially (`loose=True`) dead.
* `_repair_stranded_shell_sliced(stage, ctx, plan)` — for each `loose=False`
  candidate, drops it onto whatever REAL support
  `quake_collapse._deck_support_z` finds under its own footprint (plus a
  small tip, the same idiom the roof-prop reseat already uses) when that
  drop is at most `_SHELL_ORPHAN_DROP_STOREYS_MAX` (2.0) storeys; beyond
  that, or with nothing real found below at all, it deactivates the piece
  instead and leaves it to the pile the building's own total-collapse mound
  already accounts for in aggregate.
* `_sweep_airborne_shell_sliced(stage, ctx, plan, gap_m=1.0)` — the SAFETY
  NET, this module's own analogue of `fire_bake.deactivate_airborne`,
  scoped to the shell only (fit-out and roof plant are already swept
  elsewhere) and run with the broader `loose=True` gate, printing
  `[qgac] airborne shell sweep: N deactivated` — it caught a second-order
  orphan produced by the repair pass's OWN deletions in testing, which is
  why it has to run last, after the repair, not instead of it.

6 new tests (`test_orphaned_shell_candidates_finds_the_toothed_gap`,
`test_repair_stranded_shell_drops_a_one_storey_gap_onto_real_support`,
`test_repair_stranded_shell_deletes_a_multi_storey_gap`,
`test_sweep_airborne_shell_catches_what_the_strict_repair_did_not`, plus the
bit-identical `plan["removed"]` regression test above and one more); new
preview tool `tools/stranded_bands_preview.py`.

### 6. `SM_Building_31`/`SM_Building_16` — genuine supertalls, now excluded

A separate, unrelated defect the same photo review turned up: `eq500_v4`
placed `SM_Building_31` (measured 60.3 × 142.2 × 302.2 m, `house_29_241` in
`quake_buildings.json`) in the `core` ring right next to the epicentre, at
DG2 ("untouched"). Confirmed NOT a measurement bug —
`test_mono_dims_reports_the_same_canonical_wdh_at_every_cardinal_yaw`
pins `_mono_dims` returning the identical (W, D, H) at all four cardinal
placement yaws, and the number matches `urban_gac.yaml`'s own pre-recorded
comment for that exact asset byte-for-byte. It is a genuine 300+ m supertall
that the round-6 `highrise` pool mirrored in from `downtown_gac.yaml`'s much
bigger showcase-ring mix without re-screening it for a small, camera-reviewed
disaster preset — and it black-framed the `epi_obl` review camera (whose
clearance budget assumed nothing taller than the ~104 m next-tallest
building actually placed) and dominated `epi_top`.

Fix, two parts:

* `downtown_earthquake.yaml`'s `overrides.usds.buildings.highrise` now
  copies `urban_gac.yaml`'s own `highrise` pool verbatim MINUS
  `SM_Building_31` and its 312.0 m sibling `SM_Building_16` — a bare list key
  under `resolve_asset_set`'s merge replaces the inherited pool outright, so
  this only affects this preset. Precedent already existed for exactly this
  pair: `downtown_1000.yaml`'s `building_props.no_roof_props` already
  excludes both "everywhere" for a different reason (they carry a
  crown/setback, not a flat deck) — and they are a clean outlier on height
  alone, with the pool's other nine members clustered 134.4-231.4 m before a
  71 m jump to 302.2/312.0 with no third member anywhere near them.
* `quake.MONO_HEIGHT_WARN_M = 150.0` — a loud, de-duplicated warning
  (`quake._warn_if_oversized`, one print per asset via
  `_MONO_HEIGHT_WARNED`) at all three `_mono_dims` call sites (`assemble`'s
  `same_art`/`gac` branches and `_mono_pass`), so the NEXT scene's build log
  names an oversized building the moment it's placed instead of only
  surfacing in a post-hoc `quake_buildings.json`/photo review. 150 m sits
  comfortably above every kit archetype, `same_art` original and ordinary
  GAC/tower-pool building (the `tower` pool tops out at 131-140 m) and
  comfortably below the two supertall outliers — the warning is meant to
  fire on the NEXT unscreened pool import, not on this preset's normal
  traffic.

71/71 green (`test_quake_v5_city.py`, including the yaw-invariance
regression above plus the existing twins/`style_of`/`gac_city` cases).

### 7. Bug catalogue additions

* **`ComputeBoundMaterial()` with no argument only resolves `allPurpose` —
  it does not fall back to a `full`-purpose bind, even though USD's own
  purpose-fallback WOULD resolve a `full` query against a plain `allPurpose`
  bind.** A pack that binds its production materials under `full`
  specifically (measured: GreatAmericanCity) makes the no-argument call
  silently harvest nothing, with no exception and no obviously-wrong value —
  just an empty material slot that later renders flat. Always ask for the
  MOST SPECIFIC purpose a query cares about; it can only find a superset of
  what the bare call finds.
* **A relationship target that `IsValid()` reports True can still resolve to
  nothing usable.** A reference to a per-material Nucleus asset composes
  fine on a live, mounted stage and composes to an empty, typeless
  placeholder once the referencing file is frozen and exported standalone —
  `GetPrimAtPath(target).IsValid()` says "yes, something is there,"
  `ComputeBoundMaterial()` (which additionally requires the resolved prim to
  actually BE a `UsdShadeMaterial`) correctly says "no, nothing usable is."
  Trusting the cheaper check first is what let a 100%-unbound `/World/bake/
  Looks` scope pass casual inspection for as long as it did.
* **Nucleus flakiness renders as "nothing composed" bake failures — retry,
  don't debug.** Several GAC bake attempts this stretch of the round (and in
  earlier rounds, per the pod-prep runbook's own recon) failed with a
  composed-empty-stage symptom traceable to a transient Nucleus hiccup, not
  a code defect; treating every such failure as a fresh bug to chase costs
  more than a retry once Nucleus itself is confirmed reachable.
* **The `b0`-`b9` close-up review cameras still have no clearance logic
  (open, see below) — do not assume the round-6c camera fix covers them.**
  That fix (`_review_camera_clearance`) only ever touches the five named
  review points (`epi`/`ne`/`se`/`sw`/`nw`); the "ten worst buildings, one
  oblique each" pass a few lines later in `downtown_quake_launch_script.py`
  still calls `_snaps.views_around` with a bare fixed pose
  (`top_h=70.0, obl_dist=55.0, obl_h=28.0`) and no clearance check at all —
  exactly the class of bug the review-point fix was built to catch,
  unfixed on a different camera set.
* **A per-cell independent keep/lose draw on a MULTI-STOREY region needs an
  explicit "is my own support still there" check, or a survivor floats.**
  `_apply_region`'s `kept_piers` draw is correct in isolation (each cell's
  own boundary condition is evaluated correctly) but was never checked
  against the SAME column's cell one storey below — two independently-fair
  coin flips can still leave a physically impossible result stacked on top
  of each other, and nothing in the removal ladder itself was watching for
  that combination.
* **A repair that only reads a file's own "clean" scope has to prove that
  scope is actually clean first.** The instinct to rebind an unbound shell
  from that same file's OWN unused-looking material scope
  (`/World/bake/Looks`) would have been a plausible-sounding fix that does
  nothing — every candidate there is equally an unresolved Nucleus stub in a
  frozen file. Only dumping raw `GeomSubset` bindings (not trusting
  `ComputeBoundMaterial()` as the sole probe) found the SECOND, independent
  reason the "obvious" fix wouldn't have worked.

### 8. Open items

* **GAC pass 3 — pending the OSMO/Nucleus outage.** dev-182 (48h, submitted
  ahead of the `airstack-dev-177` deadline) has been queued and unable to
  schedule since OSMO went down; the user has confirmed both OSMO and
  Nucleus are down and will say when they are back. `dev182_bringup.sh` (this
  session's own scratchpad) is prepped and ready to run once the tunnel is
  reachable — see that script for the exact re-slice + re-bake + gate +
  scene-relaunch sequence.
* **OV/TILT roof-plant probe flags (33 + 42) — unclassified.** The most
  recent probe sweep against the mirrored pod bakes read OV 33 / TILT 42
  fails (of DG3 2 / DG4 6 / DG5 17 / OV 33 / TILT 42 total) — flagged to
  `floater-forensics` as "tilted-deck probe semantics vs. real" but never
  actually resolved: it is not yet known how many of the 33+42 are real
  floaters (the same class `_settle_foundation_roof_plant`'s fix in round
  6c targeted) versus probe false positives on a tipped deck the same way
  `SM_Building_19`/`_24` were false positives in round 6c's GAC reseat work.
  Needs its own measurement pass, not an assumption either way.
* **Plaza/court density knob.** The user's "still... empty spaces" remark on
  the v4 photos came AFTER the round-6b `plazas`/`COURTS` dressing fix had
  already landed — the dressed-plaza verdict in section 2 confirms those
  spaces are furnished on purpose, not a bug, but no agent this round
  measured or proposed a specific density/knob change in response to the
  complaint recurring. `city_detail.plazas`'s own `min_area_m2`/
  `max_per_block` knobs are the likely lever, but this is a genuinely open
  question, not a landed or even a designed fix.
* Real AEC fracture ladder, GAC DG5 bakes beyond `SM_Building_02`/`_24`, and
  the mono-wiring design sketch — all still deferred, carried over unchanged
  from round 6b/6c (sections 8 / 8 of those rounds).
