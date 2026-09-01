---
name: build-urban-fire-scenes
description: Build or modify STRUCTURE fire damage on urban kit buildings in scene_gen — disaster/urban_fire.py's F0-F5 compartment-fire ladder (climbing, not sweeping; soot above openings, not below them; the shell stands, it does not burn away), the per-construction-type recipes (urm/rc/rc_glass), the soot_plume fire-EVENT model (2026-08-30: flames and scorch from ONE event list, EN 1991-1-2 flame + Heskestad plume + Riahi-Beyler thermophoretic deposition; wall_overlay/facade_bake are superseded), kit_substitute's per-pack routing for whole-asset city buildings (ModernCityEnvironment to its kit twin, GreatAmericanCity sliced, Muyang DownTown excluded), and the bug catalogue: the Scope.Define trap that snapped every kit building to the origin, the world-baked _cyl rebar that flew 205 m under a 6 m/s cap, fractionalCutoutOpacity's two required forms, the rectangular-scorch-edge fix, contiguous flame runs replacing a meaningless stride, and the Flow emitter budget that renders a city with no smoke while reporting success. Also covers CITY-SCALE fire, rebuilt 2026-08-31 on the downtown_fire_500 preset and urban_fire_city_launch_script.py: instancing ghosts on gprim-rooted assets (apply_placements, fc_instance_material_probe.py), burnability-aware layout (burnability_table.json, _BurnabilityGuard), the multi-seed fire_city_union.py manifest union, SETTLE_REST_V2 in the fire_bake.sh/fire_city_bake.sh drivers, and fire_people's manifest-vs-dump verification. Read before touching disaster/urban_fire.py, disaster/soot_plume.py, disaster/kit_substitute.py, disaster/urban_fire_city.py, or the urban_fire_bench / mce_fire / urban_fire_city launch scripts.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Build Urban Fire Scenes (structure fire on kit buildings)

## 2026-08-30 — THE SOOT IS `disaster/soot_plume.py` NOW. Read this first.

Everything further down about `wall_overlay`, `facade_bake`, `_facade_field`,
`_opening_holes`, `_flame_runs`, `building_skin` and the X-plume is HISTORY:
`urban_fire.py` no longer calls any of it (`wall_overlay.py` and
`facade_bake.py` are still on disk, unused). The user's verdict on that
lineage: "the pattern looks completely wrong ... Don't use any of the code
we've been using so far".

**One list of fire EVENTS per building**, unchanged since the redesign:
`soot_plume.plan_events(ctx, _severity)` runs in `burn_building` right after
`plan_fire`, stored as `ctx["fire"]["events"]` — a `flame` / `smoulder` /
`out` / `stain` state per contiguous run of openings on one storey of one
elevation. `r_flames` places its emitters FROM that list and `r_smoke_stain`
rasterises the soot FROM that list, so they cannot disagree. The physics
(EN 1991-1-2 external flame, Heskestad plume, Riahi-Beyler thermophoretic
deposition, Beer-Lambert darkening) is cited in full in `soot_plume.py`'s own
module docstring — read that for the equations. What follows here is
everything that changed getting the soot from that skin onto the actual
kit materials, all landed and rendered once on 2026-08-30.

**1. THE ATLAS FINDING, and the through-UV bake.** The first design cropped
`soot_plume.skin()`'s canvas to a module's span and STRETCHED it corner to
corner over the module's base map. `tools/soot_uv_probe.py` (bare USD in the
container via `scene_gen/tools/usd_python.sh`, no SimulationApp) measured why
that is wrong: kit base-colour maps are UV ATLASES with islands, not
corner-to-corner addressing. `SM_MBuilding01_Facade_A`'s wall face uses
v 0.034..0.727 of its map with the module's OWN bottom vertices at v~0.44 and
top at v~0.55 (non-monotone); `SM_MBuilding04_Facade_B`'s outer face is the
u 0.002..0.559, v 0.002..0.420 corner of its map; some faces cover only 13 %
of their map's texels. A corner-to-corner stretch lands soot on texels a
module never draws from and skips the ones it does. The per-prim CROP
(`soot_plume.piece_crop`) is now only a PREFILTER ("does any soot reach this
module's rectangle at all?") and a preview device (`soot_png.py`,
`soot_elevation.py --mode skin`) — never what gets baked. `disaster/soot_bake.py`
bakes through each module's OWN UVs instead: `uv_position_map` rasterises a
mesh's triangles into its own UV space once per KIT PIECE (cached in
`ctx["cache"]["soot_posmap"]`, keyed on piece/mesh/subset, paid once per
piece type, not per placement); `sample_skin` transforms those texel
positions to world and samples the skin; `bake_module` composites over the
base map with the same per-pixel formula the old crop merge used. Row 0 is
v = 1; `SOOT_BAKE_PX` (default 768) sizes the canvas. `urban_fire._bind_soot`
drives this PER GeomSubset, not per module. Verified 2026-08-30 with
`tools/soot_bake_probe.py` (a test-pattern skin — soot above 2 m plus a
vertical stripe — baked through real kit meshes, outputs in
`~/docker/isaac-sim/logs/soot_bake_probe/`) and with `tools/soot_elevation.py
--mode baked` agreeing with `--mode skin`.

**2. THE MATERIAL COPY.** `soot_plume.piece_material_like` composes an
INTERNAL REFERENCE to the module's own material prim with only the
base-colour input overridden (`soot_plume.find_basecolor` records the shader
path and input name to target) — normal, roughness, metallic and AO stay the
kit's own. A diffuse-only OmniPBR (`piece_material`) made every sooted
module "a flat, differently-lit rectangle next to its untouched neighbour"
(uf_soot commercial F4); it is now only the fallback for instance proxies,
which cannot be referenced. Verified with `tools/_soot_mat_probe.py` (8
shader prims composed on a real module, every other map kept, rebinding
resolves). Modules whose covered texels the skin never reaches are left
completely alone, materials included. An untextured subset (window glass,
painted trim) binds the flat soot tone instead of a bake, only when the
module's own soot coverage is >= 0.35 — a lightly-fringed module going flat
dark read as pane-shaped rectangles rather than a film.

**3. NO BURN-MAP STAMPS ON THE FAÇADE ANY MORE.** The user's rule: everything
ash/scorch on the façade comes from the material bake; the burnt
char/scorch/ash textures are for debris and broken parts only.
`r_char_facade`'s `roles` collapses from every wall-like role to
`("roof",)` whenever a soot skin exists — stamping a char map over a third
of the already-sooted subsets read as "hard cutoff in rectangular shapes",
gravel rectangles cut into a continuous stain. `r_roof_scorch`'s `parapets`
list is forced empty the same way — binding the maps to a share of parapet
subsets put "random parts of the other side... changing their material to a
burnt texture", black rectangles on elevations that never burned (corner
bleed + the parapet ring reading as one band). The roof deck still takes the
maps outright: the skin does not cover it.

**4. THE EVENTS' OWN RNG.** `soot_plume.event_seed` / `event_rng` hash the
building's tag, level, origin, sides, mass and position into a stable seed;
`plan_events` and the skin's noise draw from THAT, never the shared
`random.Random` the recipes use in ladder order — drawing events from the
shared rng shifted every later recipe's draws, so the SAME `UF_SEED`
produced a different `fire_collapse` and debris scatter run to run ("for
building 5 it looks like the roof is floating even though previously it
wasn't", dw_terrace F5). `burn_building` now calls
`spl.plan_events(ctx, _severity)` with no `rng` argument. The roof-lid fix
below follows the same rule: its `fracture_prim` / material-pick draw is
seeded off `event_seed(ctx) ^ 0xB0F`, not the shared rng.

**5. THE LOWER EDGE**, in `soot_plume._deposit_plume`. Beside an opening the
jamb reveals see the outflow layer; below the neutral plane there is only
downwash — `NEUTRAL_FRAC` / `NEUTRAL_WOBBLE` ramp the two together smoothly
instead of at a step, wandering the neutral plane per column so the stain's
lower edge is a ragged fringe at the sills, not a ruled line. The lateral
profile is a flat-topped, steep-sided sheet the width of the opening on rows
BELOW the head (`pw = 4`) and a Gaussian above it (`pw = 2`) — a Gaussian
tail at flame temperature down at opening height had pushed the edge into a
comb pattern. The column window widened from 3 to SIX half-widths: at 3.2 b
the tail of an hour-long event was still a visible stain and the window's
own edge showed as a vertical cut through it (highrise_step F3); at 6 b the
tail is 1e-11 of the axis. `openings()` drops any opening that projects to a
point or falls off its own side — a corner module's vent projected to
u 26.0..26.0 on a 25 m side otherwise made a 0.3 m "window" with its own
plume off the end of the wall. The glass film is hardened on log10 of the
deposit, not on alpha — `1 - exp(-8.7 m)` saturates so fast a ramp on alpha
itself collapsed the edge into a ~1 m cliff (highrise_step F3); two decades
of deposit below saturation now form the film's fringe.

**6. THE ROOF LID.** In `r_fire_collapse`, static remnants the position
sweep moves (`_deck_slab`'s footprint-sized deck, or `r_roof_hole`'s rim
remainder that nothing ever renames into `roof_slabs`) that are wider than
`LID_AREA_M2` (20 m²) are shattered with `fracture.fracture_prim` (plank
mode, `n_pieces = area / 6` clamped to 8..40) before being handed to
physics. Left whole, a slab that size fell the ~3 m to storey s0-1 and
found a stable rest on that storey's own window piers instead of the floor
below — `fit_interior` authors no columns for urm, so the "beams" the user
saw were those piers ("the roof is floating... some beams holding it up",
dw_terrace F5). Gotcha recorded here on purpose: `fracture_prim` wants a
numpy `Generator` (it calls `rng.permutation`), not a `random.Random` — the
first relaunch of this fix crashed on exactly that mismatch.

**7. TOOLS AND ENV.**

| what | where / how |
|---|---|
| `soot_png.py` | host; draws the skin over a synthetic wall with every event outlined |
| `soot_elevation.py` | container (`usd_python.sh`); `--mode base\|skin\|baked`; one real elevation assembled from the modules' own maps; outputs in `~/docker/isaac-sim/logs/soot_elev/` |
| `soot_uv_probe.py` | container; per-module UV bounding box and atlas layout |
| `soot_bake_probe.py` | container; synthetic skin baked through real UVs, outputs in `~/docker/isaac-sim/logs/soot_bake_probe/` |
| `_soot_mat_probe.py` | container; checks `piece_material_like` composes and keeps every non-diffuse map |
| env `SOOT_SKIN_DIR` | writes each building's skin PNG |
| env `SOOT_BAKE_PX` | bake canvas size, default 768 |
| `scene_gen/tests/test_soot_plume.py` | events/skin arithmetic, no USD |
| `scene_gen/tests/test_soot_bake.py` | the actual UV bake path |

Bench relaunch line: `UF_SET=default UF_SEED=7 ... SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/uf_soot2
SOOT_SKIN_DIR=... SOOT_BAKE_PX=768`. Read the bench's own notes after a run:
`smoke: soot skin from N event(s) ... M merged material(s), 0 unreadable base
map(s), 0 flat-material fallback(s)` and `fire collapse: ... K
footprint-sized lid(s) shattered`.

**8. STATUS.** Rendered in Isaac on 2026-08-30 (the `uf_soot2` bench).
Visual verdict from the user: "looking fantastic" — the rectangles are
gone, the soot is continuous across modules and sits on the real cladding.
OPEN, FOR THE BAKING PASS: on a COLLAPSED building (dw_terrace F5) the
Flow smoke sources float ABOVE the rubble instead of rising from inside
it — `_interior_smoke` seats its sources on the fit-out slabs at their
ORIGINAL height and `_roof_plume` at `m["top"]`, and `fire_collapse` drops
the slabs and the deck without moving the emitters. Smoke is placed after
the damage is simulated/baked, so the fix belongs there: seat every smoke
source on the SETTLED geometry (the heap, the surviving slabs) rather than
on the pre-collapse plan (user, 2026-08-30: "make sure the smoke in
building 5 isn't floating above it but is actually inside it").


## Why this is not `build-wildfire-scenes` with buildings instead of trees

`disaster/urban_fire.py`'s own module docstring makes the case in full and it
is worth reading directly before anything else — this section is the short
version. A wildfire is a FRONT that sweeps a plate at ground level: an
elliptical arrival time per point, a soot wash that runs UP the outside of
every wall from its base, a ground scar, a debris field of consumed timber.
None of that describes a fire in a city block:

- **A structure fire climbs, it does not sweep.** It starts in ONE
  compartment on ONE floor and goes up — the signature is a VERTICAL stripe
  of blackened storeys on one elevation with clean masonry under it, not a
  plate-wide gradient.
- **The openings are the fire's exits, and the soot goes ABOVE them.** The
  wildfire model has this backwards for its own case (ground-level arrival,
  wash rising from the base); a compartment fire vents from openings and is
  heaviest under the eaves/window heads. Every plume tongue is rooted at a
  window HEAD here.
- **These buildings do not burn away.** A masonry or concrete block is a
  non-combustible container for a fuel load entirely INSIDE it.
  Fire-induced collapse of a masonry/RC building is rare and specific; what a
  burnt-out block looks like is a standing black SHELL with its floors gone,
  not a rubble pile (a rubble pile is an earthquake).
- **The debris is inside, and it is dark.** No pale sawn timber scattered
  across a lot (that is the tornado's signature) — a black interior seen
  through empty openings, glass on the sidewalk, a modest apron of spalled
  render at the wall foot.

What IS reused, and is safe to reuse: everything about the BUILDING —
`quake_flow.describe`, the element records, `fit_interior`, `_break` /
`_break_box_like`, `_heap`, `_roof_box`, `_face_patch`, the measured window
tables, the on-face authoring primitives — because those are construction
geometry, not earthquake physics, built for exactly this kit. Everything
about the FIRE comes from `disaster.damage` (char/scorch/ash maps,
`scorched_material`) and `disaster.fire` (NVIDIA Flow). The one wildfire
habit deliberately NOT carried over is `damage.damage_placements` /
`damage_flow.BREAK_PLAN`, which fracture a building to a structural level on
the assumption the structure itself is burning.

Read `build-earthquake-scenes` for the shared `quake_flow` construction
machinery (kit families, element tables, `_mass_specs`) if you have not — it
is the geometry substrate this file damages. This file is not otherwise
downstream of the wildfire or earthquake skills; its fire model is its own.

## The pipeline

```
place a kit building (urban_building.build_building / apply_placements)
  -> plan_fire()       which storeys, from where up, which elevations
  -> fit_interior()    (quake_flow) so burnt-out windows look into something
  -> LADDER[btype][level] recipes, in ORDER (collapse first — see below)
  -> RECIPES dispatch: smoke_stain, char_facade, window_burnout, spall,
     render_peel, gut_interior, floor/roof_burnthrough, curtain_burn,
     fire_collapse, street_debris, flames
  -> settle.run() on the returned loose/static_extra/velocity
```

`burn_building(stage, parent, style, placements, x, y, yaw, level, rng, nrng,
mats, tag, flow_root=None, ...)` in `disaster/urban_fire.py` is the single
entry point. It requires the building already on stage with each
placement's `prim_path` set (`apply_placements` has run). It returns the
same ctx shape `quake_flow.wreck_building` does — `loose` / `static_extra` /
`velocity` — so a caller hands it straight to `settle.run`.

**`burn_building` damages by taking NAMED elements away — it needs a
building assembled from parts, not a single merged mesh.** A merged
whole-asset export has no addressable window, wall module or floor slab, so
every recipe in the ladder degenerates to "blacken a rectangle" over the
whole thing. See "Whole-asset buildings and `kit_substitute`" below for what
this means for each city asset pack.

**`burn_monolith` does not exist any more — see bug 2, below, for its
history and why city-scale fire is on hold until the per-building look is
signed off.**

## The severity ladder — F0 through F5

Not EMS-98 (that is shaking). A fire officer's ladder, and the distinction
that matters most is BURNING vs BURNT OUT, because those photograph
completely differently:

| Level | Name | What it looks like |
|---|---|---|
| F0 | untouched | — |
| F1 | smoke-damaged | staining above a few openings, glass cracked/out on one or two floors |
| F2 | compartment fire, ACTIVE | one or two storeys well alight: flame out of windows, heavy soot plumes, glass gone in that band |
| F3 | fully involved, ACTIVE and climbing | 4+ storeys, vertical black stripe, render peeling / spalling, roof plant tipped, top of block venting |
| F4 | burnt out | fire has PASSED — no flame, heavy smoulder; every opening in the involved block empty and black, floors gone or hanging, shell standing |
| F5 | burnt collapse | the rare one — a burnt-out URM shell that lost a wall or dropped its floors, or a long-span deck came down; charred rubble, no dust (that is what separates it from a quake heap) |

`LADDER[btype][level]` is the recipe list, keyed by `quake_flow.FAMILY_TYPE`'s
same construction types, because how a building burns depends on what it is
made of as much as how it shakes:

- **urm** — masonry shell, TIMBER floors and roof. The floors are the fuel;
  this is the only type that gets gutted and collapses.
- **rc** — concrete frame with masonry infill. Spalls, does not fall (F5 is
  the Plasco/Windsor case: a partial collapse of upper storeys onto the
  floors below, frame still standing — `fire_collapse` takes only the top of
  the block).
- **rc_glass** — curtain-wall tower. Glass goes in a vertical stripe; the
  structure does nothing at all. **F5 on a tower has no collapse entry by
  design** — a curtain-wall tower on an RC/steel core has never come down in
  a fire in the reviewed record (WTC 7 and Plasco were neither), so F5 there
  is F4 over more of the building.

`ACTIVE = {F2/F3: "flame", F4: "smoulder", F5: "residual"}` and `FINISH =
{F1: "scorch", F2/F3: "char", F4/F5: "ash"}` drive both the Flow state and
the material finish — fresh char is wet-black and glossy; cooled char going
to ash is grey, which is most of what separates F3 from F4.

**The origin is drawn low-biased** (`u**1.7`, ~60% of draws land in the
bottom third) and the fire band runs UP from it — nothing below the origin
is touched at all. That clean band under a black stripe is the single
strongest "this is a building fire, not a bombing/quake/wildfire" cue.
F4/F5 always reach the top of the mass (`hi >= 99` in `BAND`), by design: a
draw that stopped short once left a `commercial` F4 with a pristine top
storey and roof — invisible from the one view this dataset is actually shot
from, directly above.

**Collapse recipes run FIRST in F5, and the order is load-bearing.** `_els`
skips elements a recipe has marked `dead`; anything that takes a wall away
must run before the passes that author art ON walls, or the art is left
standing in the air after its wall is gone (measured: soot tongues drawn on
modules that `fire_collapse` then broke away stayed behind as "a row of grey
flags standing in the sky over an open shell", `uf_bench2 dw_terrace`,
2026-08-28 — the comment is in both the `urm` and `rc` F5 lists in
`urban_fire.py`).

## Per-construction-type materials — why there is no timber in the palette

`_FLAT` in `urban_fire.py` is a superset of `quake_flow.materials()`, and it
deliberately drops the wildfire palette's `char_timber`: a warm brown
straight from "a house IS timber" reads as scorched lumber lying in a
masonry street on a brick/concrete block ("the burnt wood texture looks out
of place", user review, 2026-08-28). Everything structural here is charred
CONCRETE or calcined masonry — neutral-to-cool greys — with one light tone
(`calcined`, pale chalky brick) so a burnt shell doesn't read as a
silhouette with no material in it. Every constant is **linear albedo, not
screen grey**: 0.30 linear renders at ~0.60 on screen (sRGB, `**0.42`), so
"dark char" is authored at 0.012-0.02 linear, not what a colour picker calls
dark.

**The burn maps are swapped, not just the ones the wildfire set draws.**
`_URBAN_MAPS` points `char`/`scorch`/`ash` at `Burn_Char_Ref.png` and
`Burn_Ash_Over_Char.png` only — measured mean linear RGB rules out
`Burn_Scorch.png` (60% redder than blue — the "brown" complaint) and
`Burn_Char_Alligator.png` (a photograph of cracked wood). `albedo_brightness`
trims what is left because a texture-bound OmniPBR ignores `diffuse_color_
constant` (the earthquake round-2 finding, reused here).

**Scorch is graded by distance from the seat of the fire**
(`_grad_bucket`/`_GRAD_MULT`, 4 steps, capped under 2x), because one
intensity stamped over a whole burnt run reads as flat black rectangles at
both ends of a run that should fade toward the edges ("scorch marks spread
and get lighter", user review, 2026-08-29). Materials are cached per grade
step in `ctx["mats"]`, not authored per element — `damage.char_materials`
makes 9 shader networks per call, so an ungated per-element `sev` would be
"hundreds of duplicate materials."

## The bug catalogue

Every entry below was reproduced against the code in this repository on
2026-08-29 (line numbers are current as of this write-up and will drift).

### 1. Whole-asset buildings need `kit_substitute` — different packs, different pipelines, by design

`burn_building` requires named parts (see above). Every whole-asset city pack
MEASURED (`tools/pack_structure_probe.py`) is a single merged mesh with
material-group "subsets" spanning the building's whole height — nothing can
be removed, nothing bound per storey. `disaster/kit_substitute.py`'s
`route(usd, W, D, H, btype)` is the one decision point:

- **`unburnable(usd)`** — `('skip', reason)`. Today this is `Muyang/DownTown/`
  only: MEASURED, one merged mesh, 5 subsets, NO glass subset (windows are
  painted into the texture) — nothing to take apart, nothing to bind per
  storey, no openings to empty, too few points to slice into a storey/bay
  grid. Excluded rather than damaged badly (user, 2026-08-29: "since muyang
  is the only unusable don't use it for fire").
- **`pack_of(usd) == "same_art"`** (currently `ModernCityEnvironment/` only)
  — `('kit', style)` via `best_style()`, or `('skip', reason)` if nothing is
  within `MAX_H_RATIO=1.6` / `MAX_AREA_RATIO=2.6`. This pack is special: its
  merged `SM_MERGED_BP_MBuilding*` assets are exports of the SAME source art
  as the kit's own `ModernCityEnvironment01` façade modules ("both moderncity
  asset packs are exports of the same scene... one is the kit bash meshes
  other is the same meshes assembled", user, 2026-08-29). Substituting the
  merged building for its kit twin is therefore re-assembly, not a look
  swap — and it is **refused, never sliced**, when no kit style is close
  enough: slicing recovers separability by POSITION, never by IDENTITY, and
  degrades every named-part recipe to "blacken a rectangle" — the exact
  artefact the user rejected for this pack ("why are we splitting up the
  moderncity buildings if we have versions of them that look good? Just use
  those", 2026-08-29).
- **`pack_of(usd) == "kit"`** — already a kit build, no substitution.
- **anything else** (GAC, AEC brownstones, downtowncity, per the current
  `route()` docstring) — `('slice', None)`: `gac_storey_slice.slice_to_kit`
  cuts these on a measured storey/bay grid, because they have their own
  real (if unnamed) parts geometry-cuttable in a way MCE's shared-art
  merges are not.

**The AEC brownstones route through `slice`, not a separate de-instance
path.** An earlier draft of this document reported "de-instanced because
they already have parts" as an unverified claim; checked directly against
`kit_substitute.route()`, that framing was stale — the function's own
docstring groups the AEC brownstones with GAC and downtowncity under
`('slice', None)`, the same as everything else that is not `same_art`,
`kit` or unburnable. `tools/openings_probe.py` separately records an older,
unrelated finding — the AEC brownstone asset is internally INSTANCED and
failed `monolith_damage.cut_shell` with a Tf error for that reason
("composed of instanced sub-prims and may be addressable after
de-instancing") — but `monolith_damage.py` is a disaster-neutral generic
damage module with no connection to `urban_fire`/`kit_substitute`, and
nothing wires a de-instancing step into the fire routing. Go by
`route()`: AEC brownstones slice.

`build_kit(stage, cell, style, seed, ssf, hide=None)` is the assembly
helper — see bug 3 for why its caller must pass it `<cell>/parts`, never
`cell` itself.

### 2. `burn_monolith` — history, now closed

`burn_monolith` (a whole-asset burn path: soot in the asset's own materials,
plume tongues, no part removal) existed for one commit: added in `018ad326`
("updated fire + new buildings", 2026-08-28 23:01), removed by `6734c6ca`
("fire reverted", 2026-08-29 11:55), which cut 804 lines from
`urban_fire.py` back toward its pre-`018ad326` state. Its entire repertoire
was one flat multiplier over every material on the asset — no part removal,
no windows, no per-storey anything — which is what made a burnt downtown
render as a field of uniformly grey boxes; `kit_substitute.route()` (bug 1,
above) exists to replace it, not to sit beside it.

Removing `burn_monolith` orphaned its callers: the city-scale fire modules
and launch scripts that dispatched to it (`disaster/urban_fire_city.py`,
`urban_fire_city_launch_script.py`, `urban_fire_city250_launch_script.py`,
`downtown_fire_launch_script.py`, `tests/test_urban_fire_city.py`) still
referenced a function that no longer existed. On **2026-08-29** those
orphaned files were deleted outright (`git rm`, recoverable from history)
rather than patched, per the user's decision: **city-scale fire is on hold
until per-building damage is signed off, and will be rebuilt on
`kit_substitute.route()` once it is.** `tools/urban_fire_dryrun.py` went
with them — it imported `urban_fire_city` and would have broken on its own.

**Kept, deliberately: `scene_gen/disaster/urban_fire_spread.py`** — a
standalone ignition-schedule solver with no dependency on `burn_monolith`
or the deleted `urban_fire_city` module, reusable when city scale is
rebuilt. **Kept, with their prose updated to past tense rather than
deleted, because the reasoning in them is still worth having:**
`pack_damage_bench_launch_script.py`, `detail/gac_props.py`,
`tools/pack_structure_probe.py`, `tools/openings_probe.py` — each now
records that the monolith path was removed on 2026-08-29 and that
whole-asset buildings route through `kit_substitute.route()`.

Verified 2026-08-29: `grep -rln burn_monolith --include=*.py .` returns
nothing. There is no caller anywhere in the repository — this is closed,
not a live gap.

### 3. `UsdGeom.Scope.Define` converts a prim to a Scope — and a Scope is not Xformable

`scene_generator.apply_placements(stage, placements, parent_path, ...)`
opens with `UsdGeom.Scope.Define(stage, Sdf.Path(parent_path))` on whatever
`parent_path` it is handed. If that path is a per-cell `Xform` that already
carries an authored `xformOp:translate` (the "link prim" a bench uses to lay
buildings out before building into them), `Scope.Define` **converts it to a
Scope with no error and no warning** — a Scope has no xform ops, so the
translate is silently discarded and the building composes at its parent's
frame origin.

MEASURED (`mce_fire_launch_script.py`, 2026-08-29): six cells laid out 60 m
apart audited as **15 of 15 pairs overlapping**, each overlap area exactly
equal to the smaller building's own footprint — i.e. perfectly concentric,
every building at (0, 0).

**The fix**: hand `apply_placements` (via `kit_substitute.build_kit`) a
CHILD path, `<cell>/parts`, not `cell` itself — that lets `Scope.Define`
demote a child the cell owns while the link prim above stays an `Xform` and
keeps its translate. `burn_building` still gets `cell` (the damage art it
authors lands under the same link prim). See
`mce_fire_launch_script.py`'s "BUILD INTO A CHILD, NOT INTO THE LINK PRIM
ITSELF" comment for the full account, including the exact call:
`ksub.build_kit(stage, cell + "/parts", r["style"], seed=SEED + ci, ssf=ssf)`.

### 4. `fractionalCutoutOpacity` needs BOTH forms — the startup flag and the post-composition carb re-assert

OmniPBR's `opacity_constant`/`opacity_texture` (used by the soot overlay and
the glass smoke deposits) is a FRACTIONAL CUTOUT, and RTX Real-Time forces it
to opaque unless `/rtx/raytracing/fractionalCutoutOpacity` (and its
pathtracing twin) is on. **Both forms are required, and each is insufficient
alone**: the `SimulationApp(extra_args=[...])` command-line flag does not
survive stage composition on its own, and a `carb.settings.get_settings()
.set_bool(...)` call is too late if it is the only form — it has to run
**after** the stage is composed, on top of the startup flag
(`urban_fire_bench_launch_script.py`'s post-build block: "THE STARTUP FORM
ALONE DOES NOT SURVIVE COMPOSITION; THE CARB FORM ALONE IS TOO LATE FOR
STARTUP. Both launchers do both.").

Getting this wrong is invisible in the log and looks like an art bug: soot
overlays and glass deposits render as hard-edged stamps instead of graded
staining ("burn edges don't fade. Most of the building just looks normal",
user, 2026-08-29). **Checked 2026-08-29** against `urban_fire.materials()`'s
own note, at a point when the city-scale launchers this originally named
(`downtown_fire_launch_script.py`, `urban_fire_city_launch_script.py`,
`urban_fire_city250_launch_script.py`) still existed — none of them passed
the startup flag, so the `wall_overlay` soot quads and the `_GLASS_DEP_*`
gradient bands rendered fully opaque (a binary cutout, not translucent) on
every one of them. Those three files were deleted on 2026-08-29 along with
the rest of the `burn_monolith`-dependent city-scale path (bug 2, above), so
that specific finding is now historical — but the underlying trap is not
pack-specific and is worth checking again on whatever launch script
succeeds them. Of the urban-fire launch scripts left in the repo today,
`urban_fire_bench_launch_script.py` and `mce_fire_launch_script.py` set
`KIT_ARGS` and re-assert correctly; `pack_damage_bench_launch_script.py`
(kept, prose updated — see bug 2) does **not** pass the flag, so its own
kit-row soot/glass will render as hard cutouts until it does. This is
authored correctly anyway in `urban_fire.materials()`/`wall_overlay.
overlay_material()` — it costs nothing and is correct the day the flag is
added elsewhere. Cross-reference `disaster/ground.py` (the burn-scar overlay
this pattern was copied from) and the `run-isaac-sim-launcher` skill.

### 5. Scorch cannot be bound per element — the edge is a rectangle by construction

`_bind_subsets` rebinds a whole placed module's own GeomSubsets, so a burnt
region's outline is the union of whichever modules cleared `_severity`'s
threshold — a rectangle BY CONSTRUCTION, however good the texture on it is.
`triplanar=True` (the previous fix) corrects the texture's REPEAT inside
that rectangle and cannot touch its edge ("the scorching still looks too
rectangular... Creating a larger scorch pattern and projecting it onto the
smaller tiles", user, 2026-08-29 — referencing the suburb's own ground-scar
technique).

**The fix is `disaster/wall_overlay.py`** — new this session, the façade
equivalent of `disaster.ground`'s burn-scar overlay: ONE low-frequency mask
(`_level_set_mask`, a level-set-perturbed threshold field, same trick as
`scorch.burn_mask_map`), sized to a whole contiguous RUN of same-side wall
pieces on one mass (`_wall_run_frame`, not one placed module), authored as a
single translucent quad (`author_quad`) spanning every module in the run and
revealed as soot through OPACITY (`overlay_material`, `opacity_mode=2`
mono_luminance, `enable_opacity_texture`). The mask's own ragged contour
therefore runs THROUGH module boundaries, because it was never bound to a
module in the first place. Driven by `_facade_field` — the REAL storey-
resolved `_severity` curve per row (peaked at the origin, hard-cut below it,
decaying above), not a generic monotonic wash — with window/shopfront
openings suppressed via a smooth falloff (`holes`/`hole_pad`) rather than a
hard punched rectangle.

**Dropped, not kept as a base tint**: the old per-element composite for
"wall"/"corner" roles is gone entirely (not layered under the overlay),
because its own edge is a module rectangle at the SAME proud offset as the
overlay, so it would show through wherever the overlay's opacity is
partial — which is most of its ragged margin. `r_char_facade`'s severity-
graded stamping is unrelated and still runs underneath.

Verify with `urban_fire.check_overlay_edge()` (below) — host-side, no stage,
asserts the run's world-space width exceeds one module's own width AND that
the baked mask varies within a single element's own pixel span.

### 6. A curtain wall cannot be composited — it has to be a flat bind

`scorched_material` paints a soot wash INTO a surface's own base-colour
texture, which is right on brick or stone (courses survive under the stain)
and wrong on a tower family, whose base colour is a pale glazing atlas: the
result is "white panes with black ink-runs down them" (measured, `uf5
skyscraper_a`, 2026-08-28). `r_smoke_stain` branches on `ctx["info"]["type"]
== "rc_glass"` and binds a flat `soot`/`soot_mid` tone directly instead, at
a SHARE of coverage rather than all of it — full coverage turned every pane
in the band opaque black, "one flat dark slab with no glazing left in them"
(`uf6 skyscraper_a`, 2026-08-28). `r_curtain_burn` (separate recipe) owns
which panes are fully GONE; the flat bind owns which are merely FILMED.

`r_curtain_burn` also does not reuse `quake_flow.r_curtain_wall` — that
function's band is drawn from a drift profile and put glass loss at storeys
26-27 of a tower whose fire was at 17-20 on the bench (`uf_bench2`,
2026-08-28): a horizontal band of missing panes floating five storeys above
the fire, wrong axis entirely. A fire's own vertical stripe (mullions melt
at 660°C and take the cage with them, unlike a quake) is authored separately
in `urban_fire.py`.

### 7. Loose vs. static_extra vs. deactivated — three related but distinct floating-object failure modes

- **A prim that should fall but is never handed to `loose`** stays exactly
  where it was authored. `_rafter_teeth` deliberately keeps its rafter
  stubs OUT of `loose` (`static_extra` instead — a burnt stub standing
  upright in a wall pocket is the point), but if such a piece is also left
  out of `static_extra`, a LATER collapse pass cannot find it in its
  position sweep to hand to the solver when something falls onto it:
  `rafter_b5_58` rendered floating over a collapsed F5 shell whose roof had
  already burnt through under it (reported 2026-08-29).
- **A too-thin collider is unsimulatable.** `_cyl`'s radius has a hard 0.06 m
  floor because PhysX cooks nothing for a sub-centimetre tube; below it
  `settle` reports "loose prim(s) NEVER SIMULATED (no cookable mesh under
  them)" and the piece stays exactly where authored — on a roof that has
  since fallen, that is a stick hanging in the sky (`uf_r2j`, 2026-08-28).
- **A deactivated prim must not be left in `loose`.** `r_gut_interior`
  deletes a share of partitions/props; if a path stays in `ctx["loose"]`
  after deactivation, `settle` cannot cook a mesh for it and reports the
  same "NEVER SIMULATED" diagnostic, which reads as a physics bug but is
  really a stale reference (`uf_r2j`/`uf_r2k`, `part_main_4_1`).
  `burn_building`'s own end-of-run filter —
  `ctx["loose"] = [q for q in ctx["loose"] if stage.GetPrimAtPath(q).IsValid()
  and stage.GetPrimAtPath(q).IsActive()]` — is the fix, applied once at the
  very end after every recipe has run.

### 8. `quake_flow._cyl` baked world coordinates into mesh points with no xform ops — "the vent bug"

Every OTHER `_cyl` caller in the codebase authors decorative statics kept out
of `loose`; `urban_fire.dress_roof_urban` hands its vent-stack cylinders to
`settle.prepare` via `roof_plant -> loose`. `_cyl`'s points used to be
authored directly in world space with no `xformOp:translate`, so
`UsdPhysics.RigidBodyAPI` (which treats the prim's own transform as the
body's origin, and local points as the shape's OFFSET from it) put the rigid
body's origin at its parent's frame origin — MEASURED up to ~145 m from the
shape's own collision geometry.

This is a moment arm: angular velocity moves the swept SHAPE (and the
translate op `settle.bake` reads back) by roughly `omega * arm` per step, so
a `max_speed=6.0` **linear-speed cap on the origin** produced a **205 m**
"worst mover" (`vent_b5_14`, `uf_fix1`, 2026-08-29) — the cap bounds the
origin's speed, never the swept distance of a shape that is not co-located
with it. Also the likely cause of tunnelling through the ground plane
despite CCD: CCD's sweep and PhysX depenetration both reason about the
body's own pose, which was nowhere near the geometry actually being
resolved.

**Fixed by centring**: `_cyl`'s points are now local, centred on the tube's
own midpoint, with an `xformOp:translate` carrying that midpoint's world
position — same pattern `_box` already used. Fixes every current AND future
caller. Side effect: moves tiny meshes off single-precision coordinates of
100+ (where `Vt.Vec3fArray` float32 loses a millimetre or two) onto
coordinates near zero (exact to a fraction of a micron).

### 9. Roof plant and the roof hole are drawn from independent RNG — a shared pad rides out a burn-through

`urban_fire.r_roof_burnthrough` reuses `quake_flow.r_roof_hole` (shared
code, fixed once for both quake and fire) to cut the burnt-through polygon.
A housekeeping pad carrying a condenser row spans several hole cells and
only needs to keep a FRACTION of its own footprint supported to stay resting
near its authored height — an AC unit riding on that pad never loses its own
support because the pad under it does not move, so the row reads as "the
roof survived here" from directly above even though most of the deck beneath
it is gone (`ac_b2_3`, F4 `commercial` bench building, 2026-08-29). Same
shape as the "floating water tanks on rooftops" bug `_b_settle_roof_plant`
already exists to fix, one level up.

**Fix**: `_mostly_in_hole(pth)` in `quake_flow.py` tests MAJORITY OF THE
FOOTPRINT (a `UsdGeom.BBoxCache` world bound against the hole polygon), not
the centre point alone — a small AC unit is near enough to a point that it
doesn't matter, but a 10+ m pad needs the fairer test or an item hanging off
its far end never trips it. Anything mostly over the hole is dropped
explicitly, authored at floor level, the same way a share of the broken roof
cells already are.

### 10. The GAC slicer's memory — stream band by band, and take a lock

`gac_storey_slice.slice_to_kit` used to build every band mesh and accumulate
every piece before writing any of them, so peak memory was the WHOLE
building — measured at 81 band meshes plus several hundred piece point-
arrays alive at once on `SM_Building_16`. Several concurrent slices are not
slow, they are fatal: "five or six concurrent verification runs during an
agent session exhausted RAM and took the whole machine down" (user,
2026-08-29: "it was taking too much memory and OOM was causing the computer
to crash ... i saw like 5-6 of them running at once").

**Fixed two ways**: `slice_to_kit` now streams band by band and frees each as
it goes (`start_index` lets `as_placements` write incrementally and still
produce the exact prim names an accumulate-first pass would have). And
`slice_lock()` (`gac_storey_slice.SLICE_LOCK_PATH`, `fcntl.flock`) serialises
slicing MACHINE-WIDE — one slice at a time — deliberately at the slice level
rather than in each tool, because ad-hoc verification scripts will not
remember to take a lock themselves. Opt out with `GSS_NO_LOCK=1`.

## Flame placement — contiguous runs, and the Flow emitter budget

**Emitters go at the openings, not scattered over the floor plate.**
`fire.add_structure_fire`'s tree-house model is wrong here — a masonry/
concrete block confines the fire completely, so every visible bit of
flame/smoke is coming out of a hole. `_flame_sources` places
`FLAME_PER_OPENING` (3) `FlowEmitterBox` sources (a sheet, not a point —
NVIDIA's own incident extension agrees for a window-shaped source) across
each chosen opening's HEAD (hot gases leave through the top two-thirds; a
mid-height source puts the flame's root below the cill and it reads as a
fire in the street).

**Which openings get chosen was the bug**: the original selection sorted
every candidate by severity then took every third one (`ops[::3]`) to keep
licks from merging into one bar across the wall — a real failure the stride
was built to fix (`office_wide`, 14 emitters at 0.14 m cells, `uf_bench`,
2026-08-28). But the stride was applied to a list already reordered by
severity, so "every third" had no spatial meaning left — "the fire looks
like it's in alternating windows... random blobs of fire" (user,
2026-08-29). **`_flame_runs`** fixes this by ordering spatially FIRST:
openings are grouped by `(mass, side, storey)` — one wall run — sorted along
the wall, and a run of `FLAME_RUN_LEN` (2-4) CONTIGUOUS neighbours is grown
from a random start within the hottest group, groups visited hottest-storey-
first. A compartment fire spreads to the compartment next door, not to every
third window.

**The Flow emitter/block budget is a shared pool, and it is easy to exceed
without any error.** `r_flames`'s own accounting: worst case per building is
`max_emitters * FLAME_PER_OPENING` (27 at defaults) + `SMOKE_EXTRA_MAX` (3)
+ `INTERIOR_SMOKE_MAX` (3) + 2 roof plumes ≈ 35 Flow prims, drawing from the
`rtx/flow/maxBlocks` pool `fire.setup_flow_stack` allocates. This is still
live and worth watching on any single-building bench.

**At CITY scale this compounded, badly — evidence worth keeping even though
the module that produced it is gone.** `urban_fire_city.py` (deleted
2026-08-29 along with the rest of the `burn_monolith`-dependent path — bug
2, above) MEASURED, before removal, on a 500 m plate: 68,216 stage prims
whose geometry + BLASes left only **704 MB free** on an RTX 5070 Ti (16 GB).
Carrying the single-building bench's settings across
(`density_cell_size_m=0.10`, `max_blocks=32768`, 9 emitters × 21 burning
buildings including a 302 m tower) asked Flow for a volume it could not
have:

```
[carb.graphics-vulkan.plugin] Out of GPU memory allocating resource 'flow'
[rtx.flow.plugin] Failed to allocate 1x1x1 texture. Allocating 1x1x1
                  fallback texture to avoid crash.
```

**It does not crash and does not raise. It renders a city with NO smoke in
it** — indistinguishable from a fire pass that never ran; the scene comes
up, the banner reports N buildings involved, every capture is of an
apparently untouched downtown. This is the single most important thing to
carry forward into whatever rebuilds city-scale fire on
`kit_substitute.route()`: **any launch that asserts "N buildings involved,
smoke authored" must be checked against the Kit log for the Vulkan OOM
lines above, not trusted on its own banner.** The fix `urban_fire_city.py`
used before it was deleted — worth reapplying rather than rediscovering — was
a coarser cell (`FLOW_CELL_M=0.40`, vs. the bench's 0.10; voxel count goes as
`1/cell**3`, so 4x coarser is ~64x cheaper over the same volume), a smaller
pool sized to what the card has left (`FLOW_MAX_BLOCKS=8192`, vs. 32768), and
a hard emitter total spent on the WORST buildings only (`FLOW_EMITTER_
BUDGET=48` across the whole plate — soot, plumes and debris are geometry and
cost nothing per building, so the rest of the city still carries visible
fire without a volumetric source).

## Reachability — a burnt elevation must connect to the fire's origin

"some parts of the building look burnt on the other side of the building
that isn't even on fire... make sure the burnt parts are at least connected"
(user, 2026-08-29). `_severity` alone cannot enforce this — it only knows
STOREY distance, never SIDE. `_side_reach(ctx, side, roof_like=False)` is
the one place adjacency is decided: 1.0 if `side` is one of the fire's own
`sides`, `SIDE_BLEED` (0.28) if it corner-shares with one (`_side_neighbors`,
one hop only — not "any side not on fire," which is what the original bug
did), 1.0 for the roof/parapet once `f["roof"]` is true (the deck is one
continuous plane), else exactly 0. Verified exhaustively (240 cases) by
`check_scorch()`, below.

## Verification tools

- **`urban_fire.check(verbose=True)`** — host-side, no stage: every ladder
  recipe name resolves in `RECIPES`, every construction type has all six
  levels, every `urban_building.STYLES` entry's family maps to a ladder via
  `quake_flow.FAMILY_TYPE`.
- **`urban_fire.check_scorch(verbose=True)`** — host-side, no stage, no
  `pxr`: machine-checks the two invariants behind the connectivity/gradient
  review round. (A) `_side_reach` exhaustively over every non-empty subset
  of `sides` (15) × every `side` (4) × `roof_like` (2) × `f["roof"]` (2) —
  240 cases — must return exactly 0 for anything not reachable. (B)
  `_grad_bucket` must be non-increasing as severity rises AND actually vary
  (a flat `lambda sev: 0` is "monotonic" too — that IS the original one-
  intensity bug), checked both in isolation and end-to-end through the real
  `_severity`'s above-the-band tail.
- **`urban_fire.check_overlay_edge(verbose=True)`** — host-side, no stage:
  builds a real kit building via `urban_building.build_building` +
  `quake_flow.describe` (pure placement math), then asserts the
  `wall_overlay` run's world-space width exceeds one module's own width
  (proving it is not per-element) and that the baked mask genuinely varies
  within a single element's own pixel span (proving the edge is not
  module-aligned even at the geometry level).
- **`kit_substitute.check(verbose=True)`** — host-side (run as
  `python3 disaster/kit_substitute.py` — note the lazy `sys.path` shim at
  module scope, needed only when run as a bare script rather than imported
  under Kit): the live style table loads and covers every
  `urban_building.STYLES` entry, self-match sanity, a 302 m tower is
  correctly REFUSED (never approximated or handed to the slicer) even
  against the corrected height table, and the three real measured MCE
  dimensions each route to their expected named style.
- **`tools/urban_fire_dryrun.py` — deleted 2026-08-29, along with the rest
  of the `burn_monolith`-dependent city-scale path (bug 2, above); it
  imported the now-gone `urban_fire_city` module and would have broken on
  its own.** It solved the CITY-SCALE ignition schedule (which buildings
  catch and when) host-side, no Kit, in ~1 s, over a REAL generated layout —
  a different question from `check_scorch`/`check_overlay_edge`, since it
  never called `plan_fire`, `_severity` or any per-element material pass and
  so could not see either of those bugs. **`disaster/urban_fire_spread.py`,
  the standalone ignition-schedule solver it drove, was deliberately kept**
  (it depends on neither `burn_monolith` nor `urban_fire_city`) — a
  replacement dry-run tool over it is the natural first thing to write when
  city-scale fire is rebuilt on `kit_substitute.route()`.
- **`tools/debris_float_probe.py`** — standalone `pxr`, no `SimulationApp`,
  safe beside a live sim: finds floating debris in a BUILT scene and
  reports which of five independently-authored populations it belongs to
  (road-blockage litter, baked tree/house archetype debris, and — the two
  urban-fire-specific ones — `prop_`/`slab_`/`part_`/`col_` fit-out geometry
  authored live by `quake_flow.fit_interior`, called from `urban_fire.
  burn_building`, referenced straight onto the building's own parent with
  no `/inst` boundary). A CONSISTENT height across many prims is the tell of
  a uniform-offset bug rather than a real physics problem — the tool prints
  spread per population for exactly that reason.

## Whole-asset buildings in practice: `mce_fire_launch_script.py`

The bench that exercises `kit_substitute` end to end: for each named
ModernCityEnvironment merged building, the merged original stands in column
0 next to its damaged kit substitute across the F1-F5 ladder in columns
1-5. Layout is computed BEFORE anything is built (footprint/height are known
from `urban_building.footprint` and `quake_flow._mass_specs` before a single
recipe runs) — the previous version measured cells AFTER building into them
and got a 421 m-wide cell (debris and authored art reach far outside a
building), which drove the pitch to 926 m and cells still ended up
overlapping. Env: `MF_ASSETS` (default `MBuilding01,MBuilding05,MBuilding02`),
`MF_LEVELS` (default `F1,F2,F3,F4,F5`), `MF_GAP_M` (30), `MF_SEED` (7),
`MF_FLOW` (1), `SETTLE_STEPS`, `SNAP_DIR`, `KEEP_OPEN`.

## Running the per-building bench

There is no `.sh` runner for urban fire (unlike `eq_bench.sh` for
earthquake) — bring it up directly via the `run-isaac-sim-launcher` skill's
`airstack up` + tmux pattern:

```
UF_SET=default SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/uf_bench \
    ISAAC_SIM_SCRIPT_NAME=urban_fire_bench_launch_script.py airstack up isaac-sim
```

`UF_SET` selects a named row set from `SETS` in
`urban_fire_bench_launch_script.py`:

| Set | Purpose |
|---|---|
| `default` | variety is the deliverable — spans all three construction types, five of eight kit families, 17-103 m, every severity F1-F5 (7 buildings) |
| `ladder` | one style (`commercial`), all five severities — judge the ladder in isolation |
| `towers` | five `rc_glass` towers at varying severity — judge the curtain-wall stripe |
| `masonry` | five `urm` styles — judge the collapse/spall path |
| `concrete` | five `rc` styles — judge spalling without collapse |

Or spell it out explicitly with `UF_BUILDINGS=<style>:<level>[:origin_storey],...`,
overriding `UF_SET`. Other env: `UF_SEED` (7), `UF_SPACING` (2.2× widest,
min 60 m), `UF_FLOW` (1 — authors Flow/flames; 0 is ~40 s faster and right
for judging geometry alone), `UF_ORIGIN` (force every fire's start storey),
`UF_SIDES` (force burning elevations, e.g. `S` or `S,E`), `SETTLE_STEPS`
(1600), `KEEP_PHYSICS`, `SNAP_DIR` (must be under
`/isaac-sim/.nvidia-omniverse/logs/`), `KEEP_OPEN`.

## Known gaps

- **City-scale fire was rebuilt 2026-08-31 — the paragraph below is
  historical.** `burn_monolith` and everything downstream of it
  (`urban_fire_city.py`, its launch scripts, its test,
  `tools/urban_fire_dryrun.py`) were deleted 2026-08-29 (bug 2, above), on the
  user's decision to hold city scale until per-building damage was signed
  off and to rebuild it on `kit_substitute.route()` once it was. That is
  exactly what happened: `disaster/urban_fire_city.py` was recreated, and
  city-scale fire now runs on the `downtown_fire_500` preset through
  `urban_fire_city_launch_script.py` — see the "City-scale fire" section
  below for the pipeline and its own bug catalogue (instancing ghosts,
  burnability-aware layout, the multi-seed manifest union, `SETTLE_REST_V2`
  in the bake drivers, the people pass). Per-building damage (this document,
  `urban_fire.py`, `kit_substitute.py`, the `urban_fire_bench` / `mce_fire`
  benches, GAC's own per-building bake) remains the substrate city scale is
  built on — nothing about it changed.
- **The `fractionalCutoutOpacity` startup flag is missing from
  `pack_damage_bench_launch_script.py`** (bug 4, above; kept with updated
  prose — see bug 2) — it does not pass `KIT_ARGS`, so the `wall_overlay`
  soot wash and the glass smoke-deposit gradient on its kit-row fire
  buildings render as hard binary cutouts. The SHAPE of the overlay's edge
  (bug 5's fix) is unaffected — only the graded translucency is gated on
  this. Whatever launch script eventually carries city-scale fire forward
  should be checked for the same flag before its renders are trusted.
- **The Flow emitter/block budget is per-file, not centrally owned.**
  `urban_fire_bench_launch_script.py` hardcodes its own cell size /
  `max_blocks` / emitter cap against whatever GPU headroom was measured on
  that bench, on that card, on that date. There is no runtime check that a
  given scene/GPU combination has enough free VRAM before Flow is asked to
  allocate — the deleted `urban_fire_city.py`'s "704 MB free, city with no
  smoke" measurement (Flow emitter budget section, above) is the shape of
  the failure to expect the moment city scale scales past a single bench
  again, and there is no guard against it today.


## Partial collapse (fire) — `disaster/fire_collapse.py`, level `F5c`

**Added 2026-08-30, NOT YET RENDERED.** Everything below is verified
host-side only (`scene_gen/tests/test_fire_collapse.py`, 17 checks, and
`fire_collapse.check()`); the "what a render still has to settle" list at the
end is honest, not a formality.

### What it is, and why F5 was not it

The user (2026-08-30): *"I want some partial collapse buildings for fire in
all sets ... spawn the modern city env with 1 partial collapsed building."*

The ladder's only collapse until now was `r_fire_collapse` (F5/F6) — the
Windsor / Plasco case: the burnt-out TOP one or two storeys drop into the
floors below and the lower frame, never heated, stands. From the street that
building still has four walls; what changed is its skyline. What was missing
is the other half of the vocabulary: a building that has lost part of its
SHELL. One burnt elevation lying in the road with the floors behind it sagged
and dropped, or a corner gone from the fire floor up, the interior on show
through the hole, and the rest of it standing.

`r_partial_collapse` (in the new `disaster/fire_collapse.py`, registered in
`urban_fire.RECIPES` as `partial_collapse`) is that. The two are
complementary; neither replaces the other, and `r_fire_collapse` was not
touched.

### It has to read as a FIRE, not as an earthquake

Partial collapse is the earthquake pipeline's home ground
(`quake_flow.r_out_of_plane`, `r_corner_fail`,
`monolith_damage.partial_collapse`), so the whole design is about borrowing
its GEOMETRY and refusing its LOOK. Three rules, each enforced in code:

1. **The rubble is black and wet, not dusty.** `quake_flow._heap`'s default
   `HEAP_MIX` is mortar dust over pale brick and `_a_dustify` greys every
   fragment on top of it — the dust plume IS the quake signature. Every heap
   here is authored with an explicit fire `mat_fn`
   (`char_concrete`/`soot`/`calcined`) and every `quake_flow` helper that is
   borrowed has its output rebound afterwards by `_refire`, using the same
   `before`/`made` diff `r_roof_burnthrough` already uses around
   `r_roof_hole`. Loose pieces take `_debris_mat` (char/scorch, the dark
   end); statics — the standing half of a torn bay, the ragged remainder of a
   floor — take `_burn_mat(finish, 0.85)`, a *textured* graded burn material,
   because a flat dark bind over a hundred small cells is bug 5 in miniature.
   `rebar_`/`sbar_` are exempt: steel stays steel.
2. **The failure starts where the FIRE was.** The lost elevation comes from
   `ctx["fire"]["sides"]` and the failure line is `>= ctx["fire"]["origin"]`,
   always. The clean masonry band under a black stripe survives the collapse.
3. **The break line is a STAIRCASE that widens upward**, per
   `monolith_damage`'s own rule ("a staircase follows bays and courses; no
   long diagonal triangle"). `PROFILE_FOOT` (0.55) is the share of the full
   width lost at the failure line, ramping to 1.0 at the top. Set it to 1.0
   and you get the rectangle the profile exists to avoid — the test catches
   exactly that.

### The two modes

| mode | what it does | which ladder uses it |
|---|---|---|
| `elevation` | one burning elevation gone from the failure line up; the wall fell OUTWARD (per-fragment velocity `0.4 + 2.4 z/H` m/s, `r_out_of_plane`'s own profile), so the windrow is in the STREET and a smaller heap is inside | `urm` F5c — masonry fails out of plane, rocking about its foot as one plane, which is *why* the rubble ends up in the road |
| `corner` | the corner two burning elevations share, plus a run of each adjoining wall, from the failure line up | `rc` F5c — a concrete frame is continuous; its infill burns out and drops bay by bay but a whole elevation does not peel as one plane. That is a URM mechanism and on an office block it reads as a bomb |
| `auto` | corner when the fire vents on two elevations that share one and the building's own seed says so, else elevation | — |

`rc_glass` F5c is F5 verbatim, **no collapse**, for the same reason F5 has no
`fire_collapse` on a tower: no curtain-wall tower has lost part of its shell
in a fire in the reviewed record. The entry exists so `UF_SET`/`MF_LEVELS`
can name one level across a mixed row (and because `urban_fire.check` demands
every type carry every level) — a tower asked for F5c does not fall over.

### The ladder entry — `F5c`, beside F5 rather than above it

`LEVELS` is now `F0 F1 F2 F3 F4 F5 F5c F6`. `ACTIVE["F5c"] = "residual"`,
`FINISH["F5c"] = "ash"`, `BAND["F5c"] = BAND["F5"]`. The recipe list is F4's,
in F4's order, with `partial_collapse` inserted after `floor_burnthrough` /
`roof_burnthrough` and **before every pass that authors art on a wall** —
the same ordering argument the F5 lists carry in their own comments (`_els`
skips `dead` elements, so a wall taken away after its soot is baked leaves
the soot standing in the sky: "a row of grey flags", uf_bench2 dw_terrace).
`roof_burnthrough` deliberately stays ahead of it so the deck exists;
`r_partial_collapse` authors one itself (`urban_fire._deck_slab`) if nothing
has, so it also stands alone.

### THE ONE TRAP THAT WOULD HAVE SHIPPED SILENTLY

**`soot_plume.plan_events` returns `[]` for a level it has never heard of.**
Its first two lines are `if level not in DURATION_S: return []`. A new level
name therefore produces **no fire events at all** — no soot skin, no glass
film, no Flow emitters — while `burn_building` runs the whole ladder and
reports success. The visible result is a partially collapsed building with
pristine cladding, i.e. a demolition site. `fire_collapse.py` registers
`F5c` into `soot_plume.DURATION_S` at import time (same duration as F5) and
`urban_fire` imports `fire_collapse` at module scope *for that reason* —
`plan_events` runs in `burn_building` before the first recipe does, so a lazy
import inside the recipe would be too late. There is a test
(`test_soot_plume_knows_the_level`) whose whole job is this.

Consequence worth knowing: `plan_events`'s `burnt_out` is a literal tuple
`("F4", "F5", "F6")`, so its "the compartment of origin is always an event"
fallback can label one event `flame` on an F5c shell. `r_partial_collapse`
demotes any `flame` event to `out` when `ctx["fire"]["state"]` is not
`"flame"` — `urban_fire.ACTIVE[level]` is the authority — and it runs first
in the ladder, so `r_smoke_stain` and `r_flames` both see the corrected list.

### The RNG contract — this recipe consumes ZERO shared draws

Every recipe in `LADDER` draws from one `random.Random` in ladder order, so a
recipe that draws a different number of values moves every later recipe's
outcome (the 2026-08-30 section, item 4). A recipe meant to be *inserted*
into a ladder must not disturb that sequence at all, so:

* its own generators are `random.Random(soot_plume.event_seed(ctx) ^ 0xC011)`
  and `numpy.random.default_rng(same)`;
* `_own_rng` **installs both on the ctx** for the duration of the authoring
  and restores them in `__exit__`, because `quake_flow._heap`,
  `_ragged_slabs`, `_ragged_neighbours`, `r_droop` and
  `urban_fire._debris_mat` / `_joist_stubs` read `ctx["rng"]`/`ctx["nrng"]`
  off the ctx and take no generator argument;
* `_break` gets the private pair explicitly. **A numpy `Generator`, never a
  `random.Random`** — `fracture_prim` calls `rng.permutation`, and that exact
  mismatch crashed the roof-lid relaunch on 2026-08-30.

A test asserts the plan is identical after advancing the shared rng 50 draws.

### Everything that is bounded, and why

| knob | default | why it exists |
|---|---|---|
| `MAX_FALL_STOREYS` | 6 | "from the origin up" is right on 3-7 storey masonry and a *bombing* on a twenty-storey slab. The band is capped from the TOP DOWN and the failure line rises to meet it — never sinks below the fire's origin. |
| `MAX_MODULES` | 110 | each killed module is a `_break` (8-13 pieces + a solidify): the recipe's whole cost. Over budget the failure line rises a storey at a time, so the notch gets shallower rather than narrower and the staircase survives. |
| `CORNER_REACH_MODULES` | 1.3-2.3 modules | MEASURED: a *fraction* of the side (the first cut) made 0.45 of `block_residential`'s 88 m south elevation — 41 m, half the building, 183 modules — into a "corner". `r_corner_fail` writes the same reach as `max(4, module) * 1.6` for the same reason. `CORNER_REACH_MAX_FRAC` 0.40 is the ceiling on a small building. |
| `SPAN_FRAC` | 0.62-0.86 | most of the elevation, never all of it: a fire collapse with no stub of wall left at either end reads as a demolition, and the stub is what tells the viewer how tall the wall was. |
| `PROFILE_FOOT` | 0.55 | the staircase (above). |
| `OUT_DEPTH_M` / `OUT_SPREAD` / `IN_DEPTH_M` | 1.15-2.0 / 0.20-0.34 / 2.2-4.2 m | the street windrow (`_heap(fill=False)`, which places outward from the wall line by construction) and the smaller heap inside. `r_out_of_plane` uses 1.0-1.9 m for the same event. |
| `REGION_DEPTH_FRAC` / `_MIN_M` | 0.26 / 2.6 m | how deep into the plan the loss reaches — the second predicate on the position sweep. |
| `BREAK_PIECES` / `BREAK_CONSUME` | 8-13 / 0.26 (`UF_PC_CONSUME`) | same shape as `r_fire_collapse`'s; the mass of the pile is the authored windrow, not the shells. |
| `THROW_BASE` / `THROW_TOP` | 0.4 / 2.4 m/s | the wall rotates about its foot, so the top leads. Never saturates the bench's `max_speed=6.0`. |

### Three things it does that `r_fire_collapse` does not, and must not

* **The position sweep is restricted in plan.** `r_fire_collapse`'s sweep is
  "everything static above the failure line", which is right when the whole
  top of the block is coming down and catastrophic here — it would drop the
  far half of the roof deck, its plant and three untouched elevations. The
  reason for using a POSITION test at all still holds (the roof is
  `_roof_box` slabs, then `_split_strip` remainders, then a deck slab, each
  in a different list or in none), so the test gains a second predicate:
  above the failure line **and** inside `plan["region"]`. It also only looks
  at statics that existed BEFORE the recipe ran — everything the recipe adds
  to `static_extra` is the part that STAYS UP (the surviving cells of a torn
  bay, the ragged remainder of a slab, the windrow), and an unfiltered sweep
  would hand the standing half of every torn wall to the solver.
  `r_fire_collapse` gets away without that filter only because `_break` with
  `partial=None` returns no statics at all.
* **Both vertical edges of the hole have an owner.**
  `quake_flow._ragged_neighbours` only ever tears the two RETURN walls (it
  skips `e["side"] == side` by construction), which is the whole answer when
  an entire elevation goes and *no* answer when the span stops mid-wall — so
  it is called only where the loss actually runs into a corner. The mid-wall
  edges go to `_tear_edge`, which cracks the surviving bay on a wandering,
  bond-quantised VERTICAL line (`quake_flow._p_vcrack_judge`, FEMA 306's
  crack at a wall return). **The crack goes through the MIDDLE of that bay,
  not on the span's own edge**: which modules die is quantised to the kit
  grid, so the hole's real edge is always a module seam — cracking at the
  span edge shaved a 16 cm sliver and left the seam exactly where it was
  (measured, `commercial_mid` storey 6: span edge 4.84 m, module seam
  5.00 m).
* **`r_droop` is gated and the whole-slab drop is elevation-only.**
  `_droop_strip` splits a strip 25-50 % of the mass deep along the WHOLE of
  that side and rotates it about its inner edge; behind a wall that is still
  standing that is a floor tilting into a room for no visible reason, so it
  runs only when the loss covers >= 60 % of the elevation. And a corner that
  let go takes the corner of each slab (`_ragged_slabs` breaks the open
  edge), never the whole plate — sending an 88 x 16 m slab to the solver
  because a 7 m corner failed is `r_fire_collapse` wearing this recipe's
  name.

### The bench set and the launch line

`SETS["mce_collapse"]` in `urban_fire_bench_launch_script.py`: the five kit
styles `kit_substitute.route()` sends a merged ModernCityEnvironment building
to — `commercial_mid` and `commercial` (family 04, urm), `apartment` (01,
urm), `block_residential` (02, rc), `highrise_step` (05, rc_glass) — SIX at
F1-F5 and **exactly one at F5c**. The F5c building is the same style
(`commercial_mid`) as the F5 one on purpose: side by side they are the whole
argument for the recipe.

Set rows now take an optional FOURTH field, the burning elevations for that
building (`("commercial_mid", "F5c", 1, "S,E")`); `UF_SIDES` is still the
blunt override for the whole row. It matters because the bench's `face` /
`obl` / `close` cameras aim at `fire["sides"][0]` and
`plan_partial_collapse` loses that same elevation — pin it and the collapse
is what gets photographed. `UF_BUILDINGS` rows may be separated by `;` so the
sides field can use a comma (`commercial_mid:F5c:1:S/E` also works).

```
UF_SET=mce_collapse UF_SEED=7 UF_SPACING=130 \
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/uf_collapse \
    ISAAC_SIM_SCRIPT_NAME=urban_fire_bench_launch_script.py airstack up isaac-sim
```

(`UF_SPACING=130` because `block_residential` is 88 m wide and the default
`2.2 x widest` puts the row at 1.3 km; 130 m still leaves 42 m clear between
neighbours, which is five times the windrow's reach. Drop `UF_SPACING` to use
the default. `UF_FLOW=0` is ~40 s faster and right for judging the geometry
alone.)

Read back from the run: `partial collapse (elevation): S lost from storey 1
up (6 storey(s), 83 % of the elevation at the top), 35 module(s) taken, 1
slab(s) dropped whole; outside heap on S at +2.6 m of the wall line, inside
heap on S at -1.7 m` — and the second note with the fragment / torn-bay /
rebound / swept / rubble counts. **A negative "outside heap" offset means the
windrow is inside the building.**

Same level on the MCE bench, which stands each merged original next to its
damaged kit twin:

```
MF_ASSETS=MBuilding01 MF_LEVELS=F3,F4,F5,F5c MF_SIDES=S,E \
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/mce_collapse \
    ISAAC_SIM_SCRIPT_NAME=mce_fire_launch_script.py airstack up isaac-sim
```

`MF_SIDES` is new and exists for the same camera reason: that bench shoots
each row from -Y, so S must stay first.

### Verification without Isaac

* `python3 scene_gen/tests/test_fire_collapse.py` (or under pytest) — 17
  checks over seven real kit buildings (`urban_building.build_building` +
  `quake_flow.describe`, no USD, no Kit) through the real
  `plan_fire` -> `plan_partial_collapse` path: everything lost is on a
  burning elevation at or above the origin; the untouched elevations keep
  every module and the storeys below the failure line keep theirs; the
  staircase is monotone and actually steps; the outside heap is strictly
  outside the wall line and the inside one strictly inside, at every yaw;
  both vertical edges of the hole have an owner; the corner mode takes a
  corner; the band is capped; the plan is stable and rng-independent; the
  ladder can select the level and `soot_plume` knows it. Mutation-checked:
  `PROFILE_FOOT = 1.0`, a flipped heap sign and a failure line below the
  origin each fail a named test.
* `fire_collapse.check()` — the same invariants over both modes, callable
  from a launch script. Both benches now run it, but **only when the row
  actually asks for an F5c building** (~2 s host-side).
* The decision logic is deliberately factored out of the authoring:
  `plan_partial_collapse` touches no `pxr`, `r_partial_collapse` is only the
  USD authoring of a plan that has already been checked.

### What a render still has to settle — NONE of this is verified

* **Whether the wall lands in the windrow.** The per-fragment outward
  velocity, `_heap`'s depth/spread and `settle`'s `max_speed=6.0` /
  `SETTLE_DECOMP_M=0.8` have never been run together for this recipe. The
  windrow is authored where the wall *should* land; the fragments are thrown
  and settled independently, and they may land short (a pile against the
  plinth) or long (fragments past the windrow, on bare asphalt).
* **Whether the notch reads as a notch** from the three bench views, or as a
  rectangle anyway — the staircase is 1-2 module steps over a 4-6 storey
  band and that may be too subtle at 30 m.
* **Whether `_refire` catches everything.** It rebinds by diffing the loose /
  static / authored lists around the borrowed `quake_flow` calls; anything
  those helpers author into a list this recipe does not watch keeps the quake
  palette, and a pale dusty chunk in a fire scene is the single most obvious
  wrong-disaster tell.
* **Cost.** `commercial_mid` F5c plans 35 broken modules plus ~12 torn bays,
  six ragged slabs and a roof strip; `block_residential` in corner mode is 27
  modules. Nothing has been timed, and the bench's Flow budget has not been
  re-checked with an extra building in the row (the "renders a city with no
  smoke and reports success" failure in the emitter-budget section above is
  the shape to watch for).
* **The smoke seating.** The OPEN item from the 2026-08-30 section applies
  here too and probably harder: `_interior_smoke` seats its sources on the
  fit-out slabs at their ORIGINAL height and `_roof_plume` at `m["top"]`, and
  this recipe drops a slab and breaks the deck edge without moving any
  emitter.

**Structure reads as steel, not timber (review 2026-08-30: "rods, probably
structural — they looked wooden; we want metal, or better pillars").** Three
things had made the exposed structure read as wood: (1) the quake palette's
`rebar` tone is a warm rust (0.30, 0.19, 0.13) and every `rebar_*`/`sbar_*`
bar the collapse and roof-hole recipes author kept it — `fire_collapse._refire`
and `r_roof_burnthrough` now rebind those bars to `_FLAT["steel"]`
((0.040, 0.043, 0.048), cool grey) instead of exempting them; (2)
`_joist_stubs`/`_rafter_teeth` were thin `_cyl` rods in `burnt_metal` (warm) —
they are now square `_box` posts in `steel`, 0.14-0.24 m, and a joist stub
STANDS on the floor below the failed slab (it used to hang from the slab
line and floated between two floors); (3) `fit_interior`'s frame columns
(`col_*`, rc/rc_glass only) were bound to the pale megascans `concrete` and no
fire pass touched them — `r_gut_interior` rebinds them to `steel` at severity
>= 0.25, and `r_expose_interior`'s piers and rc beams are `steel` too.
Offline check: `scene_gen/tools/kit_burn_probe.py` (bare USD, both an urm and
an rc style at F5c) prints a material census and four FLAG lines —
rebar-tone, pale-columns, cyl-rods, floating-joists — all must read 0.

**The hole's perimeter is torn by ADJACENCY, and everything round it is
scorched (second review, 2026-08-30: "sharp straight or rectangular cuts ...
parts of the surface look pristine").** `fire_collapse.plan_edges` walks every
SURVIVING module of the burning mass and records which of its edges a dead
module touches — `left`/`right`, `below` (the tread under the failure line),
`above` (capped at 0.25-0.40 penetration so nothing reads as floating),
`return` (the adjoining elevation within a bay of a lost corner) — and
`_tear_perimeter` runs one `quake_flow._break_split` per module with the union
of its judges, 0.25-0.60 of the module, from the PRIVATE rng. Three straight
cuts came from real bugs: `reaches_end` was the top storey's answer applied to
every storey, `_p_ragged_courses` only ran inside that branch (a mid-wall span
tore nothing under the hole), and `_piece_frame` is a LINE so a yawed corner
block projected to a point and never counted as a neighbour
(`el_footprint` now measures the bbox from `urban_building.PIECES`). The
scorch is the soot model's: `burn_zone_rects` writes one rect per storey per
lost elevation (+ the return face) into `ctx["fire"]["burn_zone"]`, and
`soot_plume.skin(burn_zone=)` lifts alpha to 0.85-0.95 inside with a
1.5-2.5 m ramp wandered by the existing mottle/streak fields — no extra rng
draw, so `burn_zone=None` is bit-identical. Probe:
`tools/collapse_edge_probe.py STYLE` (edge table must read 100 %, zone alpha
inside ≥ 0.8; skin PNGs to `~/scorch_previews/collapse/`). Freeze: the four
non-F5c levels diff identical through `kit_burn_probe.py`.

**Broken pieces keep their façade (third review, 2026-08-30: "clean
rectangular breaks ... the broken/debris material is much darker than the
intact façade — match it").** `_break`/`_break_split` bind the piece's own
cladding at prim level and put the pipeline-invented faces in a `core`
GeomSubset (`_t_core_bind`); binding `_debris_mat`/`_burn_mat` OVER the prim
(`strongerThanDescendants`) took the cladding off the outward faces too.
`fire_collapse.bind_break(ctx, path, mat, cut_only)` now chars only the
`core` subset of a STATIC remainder (kit F5c and GAC); loose fragments in
the heap char whole; a GAC shell fragment with no `core` (a clipped shell
has no thickness) keeps its façade untouched ("kept") — the burn zone in the
skin darkens the wall round the hole. Measured: `_debris_mat` luminance
0.148, `_burn_mat(ash)` 0.135; the zone-sooted skin was 0.050 with
`ZONE_ALPHA (0.85, 0.95)`/`ZONE_TONE 0.85` and is 0.082 with `(0.58, 0.74)`
/ `0.10` — the residual gap is `SOOT_DARK`/`SOOT_ASH` being LINEAR albedo
composited into sRGB maps (`merge_rgb`), shared with the frozen kit ladder,
so left alone. Fit-out in the hole falls too: partitions/columns whose
FOOTPRINT touches the lost region, and everything on a dropped storey,
go loose (`part_main_6_*` stood upright in mid-air in the second row).
GAC F5c fits out `range(s0-2, n_st)` from a plan computed on a shim ctx in
`burn_gac` (15 → 8 storeys on SM_Building_05). Probe:
`tools/_break_material_probe.py gac:NAME:F5c` (static pieces: prim material
must be the façade, `core` char). Known gap: on GAC the burn zone reaches
only the per-piece bakes — `prepare` bakes the pre-slice atlases before the
plan exists.

**The burn zone reaches the pre-slice atlases (GAC).** `prepare` now
computes `fire_collapse.plan_partial_collapse` on a shim ctx BEFORE the skin
is baked into the unique atlases and hands `burn_zone_rects(plan, m)` to
`soot_plume.skin(burn_zone=)`; the plan is stashed as `pre["collapse_plan"]`
so `burn_gac`'s fit-out uses the same failure line. Measured: SM_Building_05
F5c brick atlas 92 % sooted, 14 zone rects at run time. Also: a GAC shell
fragment with no `core` subset keeps its façade (`bind_break` → "kept").

## GAC (whole-asset) buildings: soot before the slice

Every ladder recipe above damages a KIT — a building assembled from named
façade modules whose `prim_path`s the ladder can address. GreatAmericanCity
ships the opposite shape: ONE merged mesh, ~14 material subsets spanning the
building's whole height, a few dozen textures shared by the whole façade.
`slice-buildings-into-kits` cuts that into addressable pieces by POSITION,
but a naive per-piece soot bake on top of a slice like that would write
hundreds of copies of the same 2K atlas. `disaster/gac_fire.py` is the fix:
bake the soot into the merged asset's OWN atlases once, before it is sliced,
then slice and hand the sooted, sliced building to the exact same
`burn_building` every kit building runs through.

### The pipeline

    place_source            reference the asset, centre it on the cell
      -> window_rects        glass faces -> islands (grid-hashed union-find,
                             ISLAND_CELL_M=0.30) per elevation
      -> mass_from_grid,     a quake_flow-shaped mass box from the measured
         plan_fire,          grid, the usual plan_fire, and a synthetic
         openings_provider   per-side wall frame so `_flame_sources` can
                             still land Flow emitters on the real façade
                             plane with no measured window table to read
      -> plan_events         soot_plume, from the window islands — exactly
                             the record shape a kit building's own
                             `openings()` would produce
      -> skin                the soot canvas around the mass box
      -> bake_atlases        through the MERGED MESH'S OWN UVs, once per
                             material rather than once per piece; the side
                             of every texel is whichever wall line it is
                             nearest, so one bake covers all four elevations
      -> slice_to_kit         (or `kit_bake.load_kit`) cuts the now-sooted
                             asset into placements, same as any other GAC
                             building
      -> rebind_sooted        every sliced piece's GeomSubset still bound to
                             a material that now has a sooted copy is
                             rebound to it
      -> burn_building(events=, openings_fn=, soot_prebaked=<sooted material
                        paths>, fire=, skin=)   the rest of the ladder —
                        windows, gutting, roof, collapse, flames — from the
                        SAME events, skipping its own per-piece soot bake on
                        anything already prebaked
      -> darken_glass         glass subsets on the burning storeys -> the
                             void tone (the slicer cannot burn a window out)

`burn_gac(stage, cell, name, level, rng, nrng, mats, tag, ...)` in
`gac_fire.py` is the entry point: it calls `prepare` (everything up to the
bake) then runs the slice/rebind/burn/darken tail.

**Sides are chosen by window-island count, not drawn at random.** A GAC
asset carries its glazing on one or two elevations and blank party walls on
the rest (`SM_Building_02`: 36 islands each on E and W, none on S/N) — a
side drawn uniformly at random is a blank wall half the time and the
building gets no fire events at all. `prepare` ranks the elevations by
island count and takes as many as the level's plan wants off the top.

### The SHARED-TEXEL test — why some atlases bake per PIECE instead

A UV atlas that TILES up the building (the same texel rectangle reused on
every storey) cannot be baked once before the slice — one storey's soot
would land on every storey that reuses the same texel. `bake_atlases`
detects this by rasterising the SAME faces twice, forward and reversed
(`SHARED_TEXEL_M`=2.0 m, `SHARED_FRAC_MAX`=0.08): a texel that two faces
more than 2 m apart in height both cover is SHARED, and an atlas with more
than 8% shared texels is TILED and left alone here — baked per PIECE after
the slice instead, the same path a kit building's own modules use. MEASURED
on `SM_Building_24`: ceiling 100% tiled, office interiors 91%, glass 41%,
metal 28%; `SM_Building_02`'s brick atlas is unique (not tiled) and bakes
cleanly pre-slice.

### The material copy, and the glass the slicer cannot burn out

`piece_material_like` composes an internal reference to the source material
with only the diffuse map swapped — normal, roughness and metallic stay the
asset's own, the same trick `soot_plume.piece_material_like` uses for a kit
module. `rebind_sooted` then walks every sliced piece's GeomSubsets and
rebinds anything still bound to a material that now has a sooted copy.

The slicer cannot address a single window, so `r_window_burnout` does
nothing on a GAC piece — there is no named opening to remove. `darken_glass`
covers for that afterwards: glass subsets of pieces on the burning storeys
bind to the void tone instead, which is what a burnt-out floor reads as from
a drone. A subset already rebound to a sooted copy has to be traced back to
its ORIGINAL texture (through `sooted["_png"]`) before the glass-name test
can fire at all, and the tiled-atlas per-piece copies made after the slice
are traced the same way through `ctx["soot_mats"]` (keyed on `(original
material path, png)`) — a piece bound to either kind of sooted copy is still
checked against its real base texture, never the soot PNG's own name.

### Timings (bare USD, no Kit — `tools/gac_burn_probe.py`)

| asset | level | end to end |
|---|---|---|
| `SM_Building_02` | F3 | 25 s |
| `SM_Building_24` | F5 | 157 s |
| `SM_Building_09` | F6 | 133 s |

### The crash catalogue

1. **`fracture_prim` wants a numpy `Generator`, never a `random.Random`.**
   Every lid/roof shatter it is handed calls `rng.permutation`; passing it
   the shared `random.Random` crashes on first use. Same rule as the roof-lid
   fix in the 2026-08-30 section above, hit again here.
2. **The roof-lid shatter in `r_fire_collapse` segfaulted VTK three times on
   `SM_Building_09` F6** — once on a sliced `roof_*` piece, once on a
   referenced roof-plant Xform, once on a `frag_*` rim remainder. A sliced
   GAC piece is a clipped OPEN SHELL with per-material GeomSubsets and
   thousands of triangles, not the closed, low-poly authored box
   `fracture_prim` was built for. Fixed with a POSITIVE LIST: only a Mesh
   prim named `deck_*`/`roofslab_*`, with NO GeomSubsets, under 600 faces,
   is ever handed to `fracture_prim`; everything else that reaches that
   sweep — a `frag_*` remainder, a referenced Xform, any sliced GAC piece —
   drops whole instead. **A clipped shell must never go to `fracture_prim`.**

### Tools and the bench

| what | where / how |
|---|---|
| `tools/gac_fire_probe.py` | bare USD; up to the atlas bake only — measure, plan, bake, write PNGs (skin, sooted atlases, window-island map) |
| `tools/gac_burn_probe.py` | bare USD; the FULL chain (`burn_gac`) — place, measure, plan, bake, slice, rebind, run the ladder, darken the glass. Run this before any launch |
| `tools/soot_elevation.py --gac NAME --level F3 --mode base\|skin\|baked` | one real elevation of the MERGED asset assembled from its own atlases, base vs. skin vs. baked |
| `tools/_gac_budget_probe.py` | the piece-count/timing budget probe (`slice-buildings-into-kits`) |
| `gac_fire_bench_launch_script.py` | six GAC buildings in a row, each at a different level, sliced and burned; `GF_*` env knobs; `GF_EXTRA_KIT=style:level[:sides]` adds MCE KIT columns after the row (e.g. a partial-collapse `F5c` building standing beside the GAC stock) |

### Status

A first Isaac render of the six-building GAC row is running as this is
written — render under review.

---

## 2026-08-30 — THE PER-BUILDING BAKE. One building per process, then assemble.

The six-building GAC row plus its two `F5c` MCE columns was built in ONE Kit
process. What that cost, measured on the combined run:

* **25 GB RSS.** Every building's merged source, its slicer state, its
  `_bind_soot` position-map cache (`3,158 unique sliced pieces x 7 MB`) and
  its physics bodies alive at the same time.
* **688 of ~2,350 loose bodies STILL MOVING at bake time.** Not a physics
  failure — a step budget shared eight ways. `settle.run(converge=True)` will
  keep going up to `max_steps`, but `max_steps` was sized for one pile.
  Frozen mid-flight is what "the debris is in the air" looks like.
* A slow scene open, because all of that is still in the stage.

The user's fix (2026-08-30): *"you can do 1 building at a time, bake it then
launch them together as static. That might be faster. I don't need these
bakes to be saved on host since we will need to work on them, they can be
container only."*

### The two halves

| | what it does |
|---|---|
| `simulation/isaac-sim/launch_scripts/fire_bake_launch_script.py` | ONE building per headless process: build at the origin under `/World/bake/<tag>`, burn it (`gac_fire.burn_gac` or `urban_building.build_building` + `urban_fire.burn_building`), settle it ALONE, export a self-contained `.usd` + a `.json` sidecar |
| `simulation/isaac-sim/launch_scripts/fire_assembly_launch_script.py` | reference the bakes at their column x as static geometry, ground + light, then re-place the Flow emitters from the sidecars |
| `scene_gen/tools/fire_bake.sh` | the host driver: one `docker exec` per building, sequential, `--dry-run`, `--verify-only`, per-building wall time and output size, and it prints the exact assembly line |
| `scene_gen/disaster/fire_bake.py` | the pure-python half — sidecar schema, `events_to_json`/`events_from_json`, `translate`, `strip_physics`, `rehome_for_export`, `verify_export`, manifest parsing |
| `scene_gen/tests/test_fire_bake.py` | host-side schema tests; under `tools/usd_python.sh` it also reproduces and closes the material trap offline |

```
scene_gen/tools/fire_bake.sh \
  gac:SM_Building_02:F1 gac:SM_Building_24:F2 gac:SM_Building_01:F3 \
  gac:SM_Building_04:F4 gac:SM_Building_06_Small:F5 gac:SM_Building_09:F6 \
  kit:commercial_mid:F5c::S,E kit:office_wide:F5c::S,E
```

Entries are `kind:name:level[:origin[:sides[:seed]]]`, empty fields absent.
Building `i` gets `FB_SEED + 31*i` for the burn and `FB_SEED + 7*i` for
`build_building` — **exactly** what `gac_fire_bench_launch_script.py` gives
its column `i`, so a per-building bake reproduces that bench's building,
not a different draw of it.

### A BAKE CARRIES NO FLOW, AND THAT IS THE WHOLE DESIGN

Smoke and flame are authored in the ASSEMBLY. A Flow emitter is not
geometry — it is a live source that has to share the one `FlowSimulate`
layer the launcher authors — so a bake has `flow_root=None`, `r_flames`
no-ops with its own note, and the bake instead records the fire EVENT list
as plain data.

What has to survive that round trip is not obvious, so it is written down.
`urban_fire._flame_sources` — the function `r_flames` itself calls — reads,
per opening: `hua/hub/hva/hvb` (falling back to `ua/ub/va/vb`), `fr` and
`out` (→ `quake_flow._b_face_pt`, the façade plane), `m` and `side` (→
`_outward`, which way it vents), and `storey` + `e` (→ `_severity`, which
hashes `e["x"]/["y"]/["z"]` through `_el_jitter`). `r_flames` additionally
needs each event's `state`, `storey`, `id` and `ops`. All of that is in
`fire_bake.events_to_json`, and `test_fire_bake.py` asserts every field
individually — a silently dropped one gives a correct-looking burnt building
with the flames in the wrong place, and nothing in either launcher says so.

**The `fr` 7-tuple's last element is a bool (`dw`) and it must survive.**
`_b_face_pt` re-centres a `dw` frame on the piece; lose it and every emitter
is half a module along the wall.

**The mass is stored ONCE, not per opening** — a few hundred records would
otherwise each carry a copy — and `mass_to_json` is deliberately LOSSY on
`spec`, keeping only the parapet bands `soot_plume.parapet_height` sums, so
the sidecar cannot drift out of step with `urban_building`'s style table.

**On the GAC path the op's own mass WINS over `ctx["info"]["masses"]`.**
They are two different boxes with the same tag: `gac_fire.openings_provider`
frames come from the MEASURED box (`mass_from_grid`), while
`quake_flow.describe` derives one from the registered synthetic style. The
measured box is the one every emitter was placed against, so the sidecar
takes it first and only fills the gaps from `describe`.

### THE COLUMN OFFSET — why the frames move and `e["x"]/["y"]` do not

A bake is built at the origin and referenced at `x = column`. A Flow emitter
is authored under `/World/flow/emitters`, which does **not** inherit the
column's transform. `fire_bake.translate` therefore moves each opening's
wall FRAME origin and each mass centre by the column offset, and
`_flame_sources` then places world-correct emitters with no change to
`urban_fire` at all.

It deliberately does NOT touch `e["x"]/["y"]`: `_el_jitter` hashes those to
give each module its stable severity wobble, so shifting them would re-roll
the wobble and the assembled flames would no longer agree with the soot
baked into the building's own textures.

### SMOKE ON WHAT IS LEFT, NOT ON WHAT WAS PLANNED

`_roof_plume` seats its sources at `m["top"] - 0.6` and `_interior_smoke` at
`m["levels"][storey] + 0.4` — where the roof and the floors were PLANNED.
After `fire_collapse` has dropped the top storeys into the shell those
heights are metres above anything that still exists, and the plume hangs in
clear air over the hole (user, 2026-08-30). So:

* the baker measures the SETTLED bbox of the cell and records `top_z`;
* it runs `_interior_smoke`'s and `_roof_plume`'s *selection* itself — while
  `ctx["fit"]["slabs"]` is still addressable, which it is not once the
  fit-out is geometry inside a referenced file — and records the resulting
  world points as `seats`, already clamped to `top_z`;
* the assembly RE-CLAMPS against the bbox it measures on the referenced
  geometry, which also catches a bake whose reference did not compose.

`_wall_vents` — `r_flames`' fallback for a band with no openings at all — is
NOT reproduced in the assembly: it walks `ctx["info"]["elements"]`, which a
bake does not carry. A building that would have needed it says so in the log
and gets its roof/interior smoke only.

### THE MATERIAL TRAP, AGAIN — and this time it is closed by a test

`tools/bake_gac_kits.py` records it for the SLICE. It bites a fire bake
twice as hard, because the sooted materials are the ones that break:
`soot_plume.piece_material_like` composes the source material with an
`AddInternalReference` and overrides only its diffuse map, so
`<cell>/SootLooks/mN` is a prim whose PATH says nothing and whose
composition arcs point straight into `<cell>/src`. Drop the source and every
piece renders WHITE with its geometry and its UVs perfectly intact.

`fire_bake.rehome_for_export` is the fix:

1. ask every Mesh/GeomSubset under the bake root what it is bound to
   (`ComputeBoundMaterial`) — not the slicer's internals, so this works for
   a live slice, a baked kit and an MCE kit alike;
2. `depends_on` flags a material whose path is under the doomed subtree **or
   whose `Usd.PrimCompositionQuery` arcs target it** — the second is the one
   that catches the sooted copies;
3. `gac_slice._material_source` finds the material's OWN `Materials/*_Inst.usd`
   (measured: every GAC material is a separate file), a fresh prim under
   `<bake root>/Looks` references it, and the LOCAL PNG override is re-applied
   at the same RELATIVE shader path inside the freshly referenced network;
4. every binding is rebound and the old prims removed;
5. only THEN is `<cell>/src` dropped.

**If any material cannot be rehomed, the source is KEPT** (invisible) and
the sidecar says `src_kept: true`. An invisible source subtree costs load
time; a white building is not shippable.

`test_fire_bake.py` builds exactly that shape offline — a material in its own
`Materials/*_Inst.usd`, referenced under `<cell>/src`, with a
`piece_material_like` copy overriding one texture — rehomes, drops the
source, exports, reopens COLD and checks the sooted piece still resolves to
the soot PNG and the clean one to the clean PNG. A second test does the same
thing with the rehome SKIPPED and asserts `verify_export` FAILS; a verifier
that cannot fail on the bad file is decoration. Both run with `pxr` only —
no Kit, no GPU, no Nucleus.

### The other three export rules

* **ROOT LAYER ONLY** (`stage.GetRootLayer().Export`), never
  `stage.Flatten()` / `stage.Export()`. Kit meshes, subsets and materials
  carry an `assetInfo` dict core USD cannot unpack
  (`Usd_CrateFile::_UnpackValue: unsupported type enum value 0` — see the
  `freeze-disaster-dataset` skill and `disaster/freeze.py`). Everything the
  baker authors is a fresh prim or a fresh reference, which root-layer export
  preserves without composing a spec out of the poisoned source.
* **NO PHYSICS.** `settle.bake` freezes the transforms and sets
  `physics:rigidBodyEnabled = false` — "off, not removed", right for the
  settle and dead weight in a file that is only ever referenced (`bake.py`
  measured 994 of 4,824 meshes carrying ~22 such attributes each).
  `strip_physics` removes every `Physics*`/`Physx*` applied schema and every
  `physics:`/`physx*` property, plus `/World/physicsScene` and
  `/World/settleGroundPlane` — which would otherwise land in the assembly
  eight identical ground planes deep. It walks `GetPropertyNames()`, not
  `GetAttributes()`: on a sliced GAC building the second builds an attribute
  object for every property on tens of thousands of prims.
* **NOTHING BUT `/World` AT THE ROOT**, and `/World` is the `defaultPrim` (USD
  requires a root prim there), so the assembly can reference the file with no
  prim path. Kit's own viewport cameras live in the session layer and are
  already excluded; anything else that appeared at the root would ride into
  every reference.

### The soot PNGs travel with the bake

`soot_plume.OUT_DIR` is `scene_gen/assets/materials/soot` — inside the repo,
and therefore on the HOST through the bind mount. The bakes are container-
only working files, so `fire_bake_launch_script.py` re-points `OUT_DIR` at
`<FB_OUT>/textures` before building. That one module global is read at call
time by all three writers (`gac_fire.bake_atlases` via its `out_dir`,
`urban_fire._bind_soot`, `soot_plume.merge_piece`), so no file anybody else
owns has to change. The materials reference the PNGs by ABSOLUTE container
path, and `verify_export` reopens the file cold and resolves every one of
them.

### Two things the first assembled row taught (fire_row1, 2026-08-30)

**Unbound sooted copies ride into the export with a dangling arc.**
`_bind_soot` makes a `piece_material_like` copy per subset it bakes; when a
later pass rebinds that subset (`gac_fire.darken_glass` blacks out band glass
AFTER the per-piece soot bake) the copy is left unbound. `rehome_for_export`
only sees BOUND materials, so 12 (SM_Building_24 F2) and 33 (SM_Building_06_Small
F5) orphans shipped with an internal reference into the dropped `<cell>/src`:
nothing rendered wrong, every open logged an unresolved-reference error per
orphan and `verify_export` refused the file. `fire_bake.prune_orphan_materials`
now runs inside `rehome_for_export`; it also repairs an already-exported bake
in place (open, prune, `GetRootLayer().Save()`), and `depends_on` sees
UNRESOLVED reference list-ops too (the composition query lists only arcs that
composed).

**Small things hang in the air after a collapse, and the settle cannot tell
you.** Fragments of a shattered roof lid stayed at deck height over the hole,
spall halos stayed where the wall came down, rafter teeth stood on a deck
that dropped, joist stubs stood over a storey with no floor, heap chips sat
over nothing, and five `still_moving` bodies were baked mid-flight.
`fire_bake.deactivate_airborne(stage, root)` — points-based boxes, a prim is
seated if another prim's z-range contains its bottom over its footprint,
otherwise the gap to the highest top below (or the ground) must be under
1 m, only prims with a bounding diagonal under 3 m are judged — runs in the
bake launcher after the settle and BEFORE the bbox/`top_z` measurement (a
fragment frozen 40 m up would otherwise seat the smoke). First row: 0/0/11/57/
69/0/19/23 prims per building. The settle's still-moving list is now kept in
full (`settle.run` → `still_moving_paths`, in the sidecar) and those bodies are
deactivated too. Root-cause fixes made alongside: `_joist_stubs` only where
`fit["slabs"][(mass, storey-1)]` exists.

**"Perfect rectangular parts of the building that randomly didn't get
scorched" (GAC, second review 2026-08-30) — three causes, all in the
per-piece bake, none of them the fire model.** Measured with a subset census
(`_bind_soot` stats + a walk of every piece subset's bound material) on
SM_Building_04 F4: `inward: 173`. (1) A sliced GAC piece carries the
building's INTERIOR — floors, ceilings, inner walls — in the same GeomSubsets
as its façade faces, so the whole-subset facing test (`SOOT_FACING_MIN`,
"skip a subset whose faces are < 15 % outward") threw most façade subsets
away and their outward faces kept the clean map. On the sliced path the
test is now per FACE: only the outward faces of a subset go into
`uv_position_map`; the interior texels keep the base. (2) The region cut's
merged pieces (placement `_side == "x"`: below the origin and above the band)
face every elevation; `describe` files them under one ring side from their
centroid, so they were cropped and sampled as if they were on that side.
`_bind_soot` now detects `_side == "x"` and samples the skin per texel by the
nearest elevation (`_skin_sample`, `bake_module(sampler=)`), outward =
mostly-vertical faces whose normal points away from the mass centre; the
crop prefilter is bypassed for them. (3) A subset whose material has no
diffuse map was bound a flat tone (`soot`/`soot_mid`) or left alone — a
uniform rectangle either way; on the sliced path it is now baked over the
material's constant colour (`_flat_diffuse`) into a fresh
`piece_material`. The role list of `r_smoke_stain` is also extended to
every role the kit has (GAC's `pier`/`core` map to `wall` in `describe`, so
this is defensive). ALL of it is gated to the sliced path (`one_off`,
`soot_prebaked` set, `_side == "x"`): the kit path is FROZEN — proven by
running `tools/kit_burn_probe.py` on a pristine copy and on the live file and
diffing the census (identical, 247 lines). `SOOT_BAKE_PX_SLICE` dropped to
256 (the per-piece maps were 864 MB of the row's 1.4 GB texture budget —
`tools/bake_vram_census.py`).

**"Lots of floating debris" on GAC (second review) was ONE missing floor,
not stray chips.** SM_Building_09 F6 baked 6,186 `fireheap` chips at z 44-49 m
over nothing: a sliced GAC building is fitted out on its top storeys only,
the slabs of the failed storeys go down with the collapse, the storey the
heap lands on had no slab, and `r_expose_interior`'s catch plate sat 3 m
ABOVE the failure. The debris stays ("we want the debris to be there",
user); `r_fire_collapse` now (GAC path only) authors a `catch_` plate at the
heap's base storey when no active floor exists there and sends any floor at
or above the failure down instead of keeping it static.
`fire_bake.deactivate_airborne` (now a `vtkStaticCellLocator` ray test,
name-based candidates, `tools/airborne_probe.py`) refuses to touch a family
of which more than 25 % is "unsupported" — that is a missing floor, and it
prints a WARNING naming the family instead.

**Row 3 (2026-08-30): two more floater sources.** (1) The bake-time airborne
sweep ran while the INVISIBLE merged source (`<cell>/src`, hidden by
`slice_to_kit`) was still on the stage, so every ray inside the envelope hit
the intact original and reported support; the export drops the source, and
`tools/airborne_probe.py` on the exported file found 33-114 more per
building (parapet/slab/wall fragments whose support another recipe had
removed). `fire_bake._judge_candidates` now skips geometry whose computed
visibility is `invisible`. (2) `_heap` spreads chips a little past the plan;
on an upper-floor fire heap the overhang had nothing under it — 58 black
flecks at 49.6 m beside SM_Building_09 F6. `r_fire_collapse` (GAC) drops any
`fireheap` chip whose centre lies outside the authored floor plate. Verify a
bake with `airborne_probe.py <bake>.usd` (dry run): the flagged count should
be near zero, and anything left should be explainable by name.

**The airborne judge, as it stands (row 3, 2026-08-30):** rays start 5 cm
ABOVE the candidate's bottom (a flush neighbour counts); 13 sample points —
centroid, quarter points, edge midpoints and corners inset ≤ 0.15 m — so a
roof-deck rim fragment held along its edges is seated (the five-point
version deleted every collapse-level GAC roof deck as "unsupported");
invisible geometry (the hidden merged source) is never support; lean-contact
counts only for pieces with a bounding diagonal ≥ 1.2 m (`_LEAN_MIN_DIAG_M`
— a chip touching a façade hangs, it does not lean) EXCEPT the wall-attached
stamp families (`_WALL_ATTACHED`: spall, spallhalo, sbar, rebar, crack — the
size rule stripped 100+ spall marks per damaged wall in row 3e); the 25 % family cap
protects pile families only. Review captures: `snapshots.views_around(...,
azimuth_deg, aim_h)` — the assembly points the oblique at the burning
elevations and the band's mid-height (`fire["sides"]`, `fire["storeys"]`).

**Row 3f / Downtown row 1 (2026-08-30 evening): the last three floater
sources.** (1) Wall stamps on a SLICED piece were placed from the piece
frame's bbox depth — a cornice, awning or interior floor face pushes that
0.6-1.8 m (SM_Building_23) to 15 m (SM_Building_09) off the façade, so spall
patches and halos hovered in front of every burnt GAC wall; `_stamp_pt`
snaps them onto `fire["planes"][side]` (measured in `gac_fire.prepare`),
kit path untouched (no planes). (2) In the judge a decal "leaned" on its own
co-located halo — lean support now has to come from a prim that is not a
wall stamp. (3) On an rc collapse the fit-out COLUMN grid of the failed
storeys stayed static after walls and slabs went — a 4 m-pitch forest three
storeys tall with the catch floor and heap on it, a floating platform
(dtc Carved_13 F5); `r_fire_collapse` (GAC) sends columns of storeys ≥ s0
to physics. Downtown row 1 measured 9.0 GB / 492 MB per building (the 85 m
Carved blocks), 89 emitters incl. the F1 wisp.

### Knobs worth knowing

| env | default | why |
|---|---|---|
| `SETTLE_STEPS` | **2400** (bench: 1600) | one building's pile gets the whole budget |
| `SETTLE_QUIET` | **400** (bench: 60) | "the single cheapest way to drive `still_moving` to zero" — `settle.run`'s own docstring, and the 688 movers are what it is for |
| `SETTLE_DECOMP_M` | 0.8 | unchanged; see `urban_fire_bench_launch_script.py` for why 0.8 and not 2.5 |
| `FB_OUT` | `/isaac-sim/.cache/fire_bakes` | container-side, NOT the repo |
| `FA_MAX_BLOCKS` | 32768 | eight buildings is ~200-280 emitters. Flow's block pool is a **carb setting** (`rtx/flow/maxBlocks`), not the USD attribute; past it every further emitter gets no voxels and the row renders with no smoke **while every count in the log looks right**. If only the first few buildings smoke, raise this — do not go looking at the emitters. |
| `FA_BAKES` | the bake dir | prefer the explicit comma list `fire_bake.sh` prints: `$FB_OUT` accumulates, so a re-bake at a different `FB_SEED` leaves the old stems in place and the assembly quietly builds sixteen columns |

### Prerequisite that is easy to miss

`have_kit()` is FALSE for all six GAC assets as of 2026-08-30 —
`scene_gen/assets/kits/` does not exist, so every `gac:` bake falls back to a
LIVE `gac_storey_slice.slice_to_kit`. That is correct but slow (it is most of
a bench's build time) and it takes the machine-wide slicer lock. Run
`tools/bake_gac_kits.py --assets <names>` once first and every subsequent
bake of that asset skips the cut.

### Verified without Isaac, and what still needs a Kit run

Verified: the sidecar round trip field by field; that a round-tripped opening
produces the IDENTICAL `_b_face_pt` world point and `_severity`; that
`translate` moves the emitter by exactly the column offset and leaves z and
the element coordinates alone; the material trap and its detection, offline,
against a cold reopen; `strip_physics`; `verify_export` failing on a bad
bake; every symbol and signature both launchers call; `LADDER[urm][F5c]` and
`LADDER[rc][F5c]` for the two kit columns; all six GAC assets present in
`_plans/gac_buildings.json`; and the driver's `--dry-run` / `--verify-only`.

**Not verified — needs a real Kit run:** the settle itself (does one building
alone actually converge?), the export of a genuinely burned GAC building
(does `rehome_for_export` find every real material, or does `src_kept` go
true?), the wall-clock and file size per building, and the assembly render —
`place_fire` has never authored a Flow prim.

### This bake pipeline is not fire-specific — see `slice-buildings-into-kits`

Almost every stage above — place the source, measure the grid/mass box/wall
frames, `slice_to_kit(region=)`, settle alone, `strip_physics` +
`rehome_for_export` + root-layer export, an assembly that references bakes
at their column x — has nothing to do with fire; it is a general recipe for
damaging one GAC building alone and shipping it static. `slice-buildings-
into-kits`'s **"Baking and damaging a GAC building — the pipeline any
disaster reuses"** section walks the same ten stages naming exactly which
are disaster-agnostic and which are fire-specific (the soot skin in
`bake_atlases`, the `fire`/`events` sidecar schema, `place_fire`'s Flow
emitters), what `quake_sliced.wreck_sliced` already reuses today (the
`region=None` full-slice case — earthquake shakes the whole building, so
there is no band for `region=` to save) and would still need to plug into
this same bake (its own sidecar shape, no Flow-equivalent effect yet), and a
checklist for wiring in a new disaster. Read it before assuming any part of
this pipeline needs to be re-derived for tornado, earthquake or hurricane.

### The starved-events trap — a shape filter can silently unplug the whole fire (fire_dtc3, 2026-08-30)

`gac_fire.window_rects`' island shape filter (WINDOW_* constants) exists to stop
ornament/AC-unit blobs being burned as windows. But an island rejected for SIZE can
be real glazing: dtc Building_12's mid-façade is storey-STRIP windows (4.5 x 18 m,
one island per bay column, spanning storeys 2-8), and dtc Amar_Tower's curtain wall
union-finds into ONE 38 x 192 m island per elevation. Dropping those left ZERO
openings inside the burning band, `soot_plume.plan_events` returned an empty list,
and the bake completed "successfully" as a building with a fire plan and no fire:
no flames, no smoke, and no soot (the skin binds per event). The assembly's only
tell is `[fa] bN fire: 0 flame source(s) over 0 opening(s), 0 smoke (state=flame)`
— grep for that contradiction after every row.

Fix in `_islands`: an island that fails the shape test but is tall/wide and mostly
glass (`STRIP_FILL_MIN`) is SPLIT into a synthetic bay grid (`STRIP_ROW_M` 3.2 m,
`STRIP_COL_M` 4.0 m) and each cell re-tested — B12@4 F3 went 0 → 30 events, Amar
F5c 2 → 316. Verify any such change offline with
`tools/_dtc_open_probe.py kind:Name@origin` (the origin rides after `@`; a second
positional arg is parsed as another asset and crashes on a None prim path).

Related, found the same night: `urban_fire.r_curtain_burn` took its bays from
`qf._els(role=("wall","corner"))`, which is EMPTY for every sliced building — so a
sliced rc_glass tower's F-ladder note read "curtain burn: 0 bay(s) out, 0 crazed"
at every level and the tower kept a pristine skin. The recipe now falls back to
per-bay frames built from the measured openings table (`ctx["soot_openings"]`)
when there are no kit elements. The kit path is untouched by construction: kit
contexts never set `soot_openings`, `_piece_frame` consumes no rng, and the
candidate/roll order is preserved element for element (kit_burn_probe FLAGs all 0
after, urm + rc + a family-05 skyscraper at F5c). Remember `rc_glass` F5c is
DESIGNED to stand — F5c on a curtain-wall tower is F5 (Grenfell: the cladding
burns, the frame does nothing) — so "no collapse on the glass tower" is correct,
not a bug.

## 2026-08-31 — City-scale fire, rebuilt on `kit_substitute.route()`

The `downtown_fire_500` preset + `simulation/isaac-sim/launch_scripts/
urban_fire_city_launch_script.py` is the pipeline the 2026-08-29 decision (see
"Known gaps", above) promised: `tools/fire_city_dry_run.py` solves a real
layout + `disaster.urban_fire_spread` host-side into a manifest;
`tools/fire_city_union.py` unions several seeds' manifests into one denser
city; `tools/fire_city_bake.sh` / `tools/fire_bake.sh` drive one Kit process
per building (the "PER-BUILDING BAKE" design above, unchanged); the launcher
assembles the bakes as static geometry and re-places Flow. Five things this
scale surfaced that per-building damage never could.

### Instancing ghosts — a gprim-rooted asset cannot be instanced

`downtown_fire_500.yaml` sets `instance_placements: true` (the fix for a
composition OOM — 66,590 un-instanced placements cost ~39 GB RSS). Whenever a
placement's referenced ROOT prim is itself a Mesh (every Unreal/Muyang export
this repo already special-cases elsewhere: `BG_Building_C`,
`Building_Type{A,D}_{A,B}`, `SM_lightpost_light_post_b`, `SM_bench_wood_a`,
`Car_01_0`, `SM_light_streetlight_complete`, `PlanterLarge_A`, the fountain
parts), `SetInstanceable(True)` puts the root's materials, GeomSubsets and
(for Hydra) its geometry into the shared prototype while the drawn point
stays outside it — the placement renders as NOTHING, or as flat grey where
only a binding was lost. Measured: 14 of 110 distinct assets, 161 of 1,469
placements (all 58 streetlights, 50 of 62 benches, 16 cars, 12 traffic
lights, 7 planters, 8 fountain/water pieces, 10 buildings) — this is the
mechanism behind "some props like the street lights also look like they have
no texture" and "this roof house is floating with no building near it"
(user, 2026-08-31). `scene_generator.apply_placements` now refuses
`SetInstanceable` on any placement whose composed prim `IsA(UsdGeom.Gprim)`;
`urban_fire_city_launch_script._uninstance_gprim_roots`
(`FC_UNINSTANCE_GPRIM_ROOTS=1` default) is a second, redundant repair at the
launcher level for the same 161. Diagnose with
`scene_gen/tools/fc_instance_material_probe.py` — it composes an asset twice
(plain vs. instanceable) and diffs every mesh's computed bound material, so
`GEOMETRY-GONE` vs. `MATERIALS-LOST` is measured, never guessed. **Open,
unverified in a render:** the `human` category is not excluded by any
`prune_prims` rule in this preset, and `scene_generator._bind_human_pose`
authors its per-placement pose animation as a CHILD of the placement prim
before `SetInstanceable` is called on it later in the same `apply_placements`
loop — instancing that subtree risks pedestrians of the same asset sharing
one baked-in pose.

### Burnability-aware layout — an unburnable asset is a firebreak

A single ignition saturates at 6-14 buildings when the spread graph is full
of dead ends. `scene_gen/config/harvested/burnability_table.json`
(`{typology: {asset: bool}}`, generated by `tools/gen_burnability_table.py
--prove`, which checks its table against a synthetic layout run through the
REAL `urban_fire_city.burnable()` gate) is keyed by TYPOLOGY, not by pool —
`districts.typologies.<name>.pools` is the real map (the `midrise` typology
draws from pool `midrise_v2`, not a pool named `midrise`). `districts.
_BurnabilityGuard` allows at most one unburnable asset per block and swaps
the rest, but the swap has to check the ROTATED footprint fit of the
replacement against the original, not an area ratio — area alone let an
oversized substitute spill into whatever the packer placed next to it
(fixed: `_burnable_substitute` may never exceed the original's own rotated
extents; `districts.repair_overlaps`, see `urban-layout`, is the safety net
behind even that). None of this fixes a POOL that is mostly unburnable to
start with: `downtown_fire_500.yaml` overrides `usds.buildings.lowrise`
(26 → 7 entries) and `midrise_v2` (22 → 14) to burnable-only kit archetypes,
because the shared `urban_gac.yaml` pools measured 10/15 `lowrise` and 10/29
`midrise` houses unburnable on a seed-4 Kit build — most of two typologies
never entering the spread graph at all. **Known gap:** `tower`/`highrise`
are left untouched (their own unburnable fraction — 6/22, 3/10 — is
`podium_highrise`/`slab_tower`/`BG_Building_C`, the `>FIRE_MAX_H_M`
landmarks and oversized standalone towers the guard already handles as well
as it can) because there is no burnable substitute at that footprint class to
reach for.

### The fire manifest union — one origin does not reach a whole city

`urban_fire_spread.solve` is a single Dijkstra tree from one origin — some
origins reach 30+ buildings, some reach 1. `tools/fire_city_union.py` runs
`fire_city_dry_run.run_dry_from_dump` once per seed against the SAME
placements dump and unions the record sets (`--auto` searches for a good
seed combination; `--seeds a,b,c` reruns an explicit one). A naive
concatenation gets two things wrong, both fixed here: the `F5c`/`F6`
roof-outcome share budget is enforced PER MANIFEST, so three seeds each
independently allowed 2 roof outcomes can total 6 — `union_records`
re-applies the same budget to the final unioned list; and a placement-geometry
defect in the dump (two buildings closer than `check_footprint`'s overlap
tolerance) fails regardless of how many seeds are unioned, fixed by treating
the higher-indexed member of each overlapping pair as an additional firebreak
(`extra_blocked_global`) and re-solving to a fixed point. `--auto` selects on
CONCENTRATION, not raw count — `adjacency_share` and `n_components` at a 25 m
radius (a purely visual "does this read as a clump", not the fire mechanism's
own 13/55 m reach) plus `street_facing_share` (how much of the chosen venting
geometry honours the "prefer a street-facing façade" policy). `--max-records`
trims a full union to a VRAM-constrained subset for a bake run (~250 MB per
composed bake measured; a full union does not fit a 16.3 GB card but does fit
a 32/48 GB one), keeping the tallest/most-clustered records. `FIRE_MAX_H_M`
(232.0, set just above `Amar_Tower`'s own 231.4 m) and the `gac_fire.PACKS`
blacklist (`Carved_`, `Building_11`, `Building_12` — the "B1, B3-B5 kinda all
look the same" review) are the two other gates a stale manifest can silently
disagree with after a layout change — a manifest's record indices name
whatever building is at that cell in the dump it was solved against; always
re-solve (`FC_INTACT_ONLY=1 FC_DUMP=<path>`, then re-run the union) rather
than reuse an old manifest on a new dump.

### `SETTLE_REST_V2` in the bake drivers

`disaster/settle.py`'s below-grade/rest audit had the same `UsdGeom.
BBoxCache` blind spot the `fix-floating-debris` skill documents for the
suburb wildfire plate — read that skill for the mechanism. Here, the
practical facts: `scene_gen/tools/fire_bake.sh` and `scene_gen/tools/
fire_city_bake.sh` set `SETTLE_REST_V2=1` for every baked record whose kind
is not `kit` (the MCE kit look stays frozen, byte-for-byte); `FB_REST_STRICT`
makes a record that never came to rest a driver FAILURE instead of a loud
print; both scripts also pin `PYTHONHASHSEED=0`, because `urban_fire.
r_render_peel` used to iterate a `set` of side letters — a bake is not
reproducible run to run unless that set is read back `sorted()`, which it now
is. `deck_z` (the real roof-deck height, distinct from the parapet-coping
bbox top `gac_fire.mass_from_grid` used to fall back to) now round-trips
through the sidecar on both `ctx["fire"]["deck_z"]` and
`masses["main"]["deck_z"]` — `fire_people.deck_z()` reads the first, then the
second; before 2026-08-31 both were dead.

### People — verify against the dump, and hide the extras

`fire_people._manifest_matches_dump` re-checks every people-manifest record
against the dump BY GEOMETRY (not just by index `i`) before placing it — a
record naming a building that has since moved or resized is skipped and
counted, not placed against geometry it no longer owns. `scene_gen/tools/
fire_people_rerun.sh MANIFEST DUMP SIDECAR_DIR` is the one-command gate: it
re-solves, prints the manifest<->dump match, the sidecar-completeness table,
the census and the rule-check table, and exits non-zero if any of it cannot
be trusted, so a lead can run it blind and read the exit code. Separately,
the city generator plants its OWN ~128 background pedestrians on this plate
(`category == "human"` from `scene_generator`/`detail/parks.py`) with no
knowledge of where the fire is — `urban_fire_city_launch_script.
cull_background_people` hides any of those further than
`FC_PEOPLE_MAX_DIST_M` (120 default) from the nearest BURNING building's
footprint, per the user's "only keep humans that are in the disaster"
(2026-08-31). The people PASS itself (survivor placement) already names its
own burning building per record, so this cull is a guard on the background
extras, not the mechanism for the survivors.
