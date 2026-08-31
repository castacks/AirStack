"""surge stage — hurricane storm-surge water, as static geometry.

WHAT THIS IS, AND WHAT IT IS NOT
---------------------------------
Everything a post-surge scene needs to LOOK like standing and receding
floodwater, authored once as STATIC geometry with a material that reads
right from 30-120 m: an inundation VOLUME (a real, closed mesh with
thickness — see "THE WATER VOLUME" below; a flat quad only behind
`SURGE_FLAT_WATER=1`), isolated ponding, silt/wrack/washover deposits.
NOTHING here runs a fluid solver, NOTHING here is a physics body, and
NOTHING here animates. That is a constraint from the user, verbatim (see
`_plans/hurricane_water.md`): *"we don't need fluid simulations, we just need
it to LOOK like water/water damage. The actual meshes can be static with
material that looks like water."* Giving the water body real thickness (a
LATER request, also verbatim — see "THE WATER VOLUME") does not relax this:
the volume is still one static mesh, authored once, with no physics API of
any kind on it.

This module is the water sibling of `wind_flow.py` and it follows
`scour_relief.py`'s split EXACTLY, for the same reason recorded there: a
pure-Python `field`/`scatter` half that a test can pin with no `pxr` on the
path, and a `build`/`materials` half that touches the stage. `pxr` is
imported ONLY inside the public build/material entry points
(`water_materials`, `build_inundation`, `build_ponding`, `build_deposits`)
and their private stage-touching helpers (`_build_inundation_volume`,
`_build_inundation_flat`, `_dry_material`). Every OTHER function in this
file — every pure-Python field, scatter and reporting function,
`silt_coverage`, `review_points`, `water_volume_mesh` and the Beer-Lambert
helpers (`water_absorption_per_m`, `beer_lambert_alpha`) all included —
runs on a bare `python3` with `pxr` absent, which is the actual system-test
interpreter (`.agents/skills/build-hurricane-scenes/SKILL.md`, "Test idiom").

THE CORE PROBLEM: THE SUBURB PLATE IS PERFECTLY FLAT
------------------------------------------------------
`_plans/hurricane_water.md` S1.5 and the hurricane skill's "The single
biggest cost is NOT the wind. It is that there is no ground" say the same
thing from two directions: there is no elevation, no slope, no drainage
anywhere in either layout (`suburb_scene.apply_ground` lays a mosaic of flat
polygons on a Z LADDER, not a height field). A flat water plane at height z
over flat ground floods either EVERYTHING or NOTHING, and its shoreline is a
straight line at the plate edge — exactly what a flood must not look like.

So `ground_z` below invents its own SYNTHETIC terrain: a constant slope away
from `shore_bearing_deg` at `slope_pct`, plus band-limited relief for local
highs and lows. This is the single most important thing to understand about
this file, so it is said three times, because getting it wrong here breaks
every downstream number:

    `ground_z` DOES NOT DISPLACE ANYTHING IN THE SCENE. No mesh anywhere is
    moved to match it. It exists ONLY so this module can decide WHERE water
    should be drawn and HOW DEEP it should read — it is a bookkeeping
    surface, not a rendered one. The one place this leaks into an honesty
    problem is documented at `depth_at` and mirrors the water plan's S3.8:
    the apparent depth gradient the alpha map paints and the literal
    definition `water_level - ground_z` are the same number here (unlike a
    real flood, where a wall's mud line and a road's puddle depth are
    different physical quantities) — that is a known simplification, not an
    oversight, and it is what lets a single scalar field drive both the
    shoreline shape (`alpha_map`) and a plausibility check (`coverage`)
    consistently.

    Every prim this module DOES author (L2 pond discs, L4 wrack ridges and
    washover domes) sits at a small, FIXED real-world Z near the suburb's
    actual flat grade (`_SURFACE_Z_M`) — never at the synthetic
    `ground_z(x, y)` value, which can be metres away from the real ground on
    the "deep offshore" side of the shore ramp. `water_level(cfg)` — the L1
    water surface's own top — is the one other real-world Z this module
    authors.

    THE ONE DELIBERATE EXCEPTION is the water volume's own BOTTOM surface
    (`build_inundation`, "THE WATER VOLUME" below): it sits at literal
    `ground_z(x, y)`, on purpose, because that surface is never meant to be
    seen as ground — it is always beneath the water it bounds, hidden behind
    the SAME translucency gradient that field also drives (shallow near the
    shore, where `ground_z` is close to `water_level` by construction;
    opaque toward the interior, where it has sunk furthest below it). Nothing
    else in the scene ever reads or displaces to this surface, so the
    invariant above still holds for every OTHER prim: `ground_z` remains a
    bookkeeping field the rest of the module never renders directly.

WHY `house_water_state` HAS NO `region` ARGUMENT, AND WHY THAT IS SAFE
------------------------------------------------------------------------
`ground_z`'s slope term is anchored to the WORLD ORIGIN, not to `region`'s
own centre — which is the same thing everywhere in this codebase's
convention, because every `region` in every disaster module here IS centred
on the origin already (`tests/test_scour_relief.py: REGION = (-250, -250,
250, 250)`; `tests/test_affected_region.py: KM_REGION = (-500, ..., 500,
500)`; etc.). And its relief term is a small fixed number of world-anchored
sine harmonics (`_relief_harmonics`, cached on `cfg["seed"]`), not a raster
baked over a specific region's bounding box. So `ground_z` is evaluable at
ANY `(x, y)` without ever consulting `region`'s bounds, which is exactly what
lets `house_water_state(cfg, x, y, rng)` — called once per house, with no
region in scope at all — report EXACTLY the same terrain that
`depth_at`/`coverage`/`alpha_map` compute over a bounded plate. `region` is
still accepted by the region-taking functions (for the raster resolution
`coverage`/`alpha_map` need, and for early shape validation) but the terrain
maths itself never reads it.

WHY THE TERRAIN NEVER CONSUMES THE PASSED-IN `rng`
----------------------------------------------------
Every function below takes an `rng`, for signature symmetry with the
scatter-style functions (`pond_specs`, `wrack_specs`, `house_water_state`)
that genuinely need one. But `ground_z`, `depth_at`, `coverage` and
`alpha_map` are all independent entry points that must agree on the SAME
terrain even when called in different orders, from different call sites,
against an `rng` object that is in a different state each time (or is a
different object entirely). Deriving the relief harmonics from the caller's
`rng` would make that impossible — two calls with the same `cfg` could
disagree. So the relief field is seeded from `cfg["seed"]` alone
(`_relief_harmonics`, `functools.lru_cache`d on the handful of scalars that
define it) and the passed-in `rng` is simply not consumed by the terrain.
Flagged in the final report as the judgement call it is.

THE SHORELINE USED TO COME FROM AN ALPHA MAP. NOW IT IS REAL GEOMETRY
------------------------------------------------------------------------
Original design: `ground.build_overlay` (the burn-scar engine) had to
quantise its coverage field into bands because OmniPBR carries only one
`texture_scale` for every map it samples, so it cannot tile a diffuse while
stretching a mask once across the plate. Floodwater did not need a tiled
diffuse — its colour is nearly uniform past 30-50 cm (S2.2) — so `alpha_map`
rendered `depth_at` straight into ONE stretched opacity raster with a
feathered edge, cut into a single flat quad spanning the WHOLE plate. That
is `_build_inundation_flat` below now — kept as a `SURGE_FLAT_WATER=1`
escape hatch, not the default. It has two problems the alpha map alone
cannot fix, both named verbatim by the user: *"the water looks like a
floating rectangle prim with like no width"* (a flat quad IS zero-width,
regardless of what its material does), and a "low poly" shoreline (a raster
cutout is only as smooth as its pixel grid, and cranking that resolution is
the exact GPU-memory-for-crispness trade every other alpha-cutout map in
this codebase already makes).

THE WATER VOLUME — real thickness, and why that does not fall out for free
------------------------------------------------------------------------------
The default `build_inundation` now authors ONE mesh that is an actual closed
volume: a TOP surface flat at `water_level(cfg)`, a BOTTOM surface following
`ground_z(x, y)`, tessellated ONLY where `depth_at > 0` and stitched shut at
the shoreline — see `water_volume_mesh`'s docstring for the marching-
triangles construction, and "WHY `house_water_state` HAS NO `region`
ARGUMENT" above (this is the same `ground_z`/`_signed_depth_point` split
every other feature in this file already uses) for where the unclamped
signed depth this needs comes from. Outside the wetted footprint the volume
has no thickness because it has no geometry at all — the ragged shoreline is
now a real mesh boundary, not a cutout, and it is exactly as smooth as the
grid resolution says it is (a genuine trade, not a free lunch — see
`water_volume_mesh`'s own docstring for the resolution chosen and why).

That answers the user's literal question — *"can we make it translucent but
give it width"* — with real width. It does NOT, by itself, answer their
follow-up: *"will that make it translucent throughout or layer the
translucency so the thicker parts are more opaque?"* **A surface shader like
`OmniPBR_ClearCoat` shades the fragment it is looking at; it has no idea how
far away the mesh's OTHER side is.** Giving the mesh width changes nothing
about the picture unless something ALSO makes the material's opacity a
function of that width. Two ways to get that:

1. **True volumetric transmission** — a dielectric (`OmniGlass`, `OmniSurface`)
   with `thin_walled = False`, where the renderer ray-marches into the real
   geometry and Beer-Lambert falls out of actual light transport, no baking
   required. Investigated and NOT used, for two independent reasons, one
   physical and one this module cannot rule out without a render it is not
   allowed to run here:
     - **Physical**: `GlassUtils.get_volume_absorption` (`_plans/
       hurricane_water.md` S3.7, S1.8, read from the shipped MDL) sets
       volume SCATTERING to zero. Turbid floodwater is bright and tan
       *because of* sediment backscatter (S2.2) — a zero-scattering medium
       this absorptive would read as near-BLACK away from a specular
       glint, not the sourced "brown, opaque" appearance. That finding
       predates this rework but does not depend on the water having no
       thickness; it is about the shader family, not the geometry, and it
       still applies now that there is a real volume to shade.
     - **Unverifiable here**: this module cannot launch Isaac Sim (no GPU
       on this host), and this codebase's own experience with the OTHER
       advanced-MDL-feature case in this exact renderer — `OmniSurface`
       geometry displacement, `greeble.py`/`tools/texture_depth.py`: *"RTX
       Real-Time does not tessellate OmniSurface displacement"* — is that a
       feature existing in the `.mdl` is not the same as it being honoured
       by the renderer this project actually runs scenes in. Whether real
       per-fragment refractive transmission through a closed mesh renders
       correctly, and at what cost, under RTX Real-Time is exactly the kind
       of claim that needs a render to trust, so it is not claimed here.
2. **Fake it analytically** — compute what a Beer-Lambert transmittance WOULD
   be from the depth this module already knows exactly (because it just
   built the mesh from that same field), and author it directly as
   per-triangle material opacity. This is what `build_inundation` does: see
   `water_absorption_per_m`/`beer_lambert_alpha` for the derivation and
   `_n_opacity_bands`/`water_materials`'s `"inundation_bands"` for how it is
   quantised into a handful of discrete, pre-authored materials rather than
   a texture (a texture would need its own `texture_scale`, and that slot is
   now spoken for — see below). This is the documented fallback the task
   allows for exactly this situation, taken deliberately rather than by
   default: it reproduces the SHAPE of Beer-Lambert honestly (the arithmetic
   is shown in code, not guessed), at the cost of being a discretised,
   pre-computed approximation rather than the renderer doing real light
   transport. Banding a smooth field into materials is not a new idea in
   this file — `ground.build_overlay` already does it for the burn scar and
   for the reason it gives there (`OmniPBR`'s one `texture_scale`) — but it
   is applied here to strictly non-overlapping FACES of one mesh via
   `UsdGeom.Subset`, not to stacked, overlapping quads, so the "double
   composite at every interior edge" trap `ground.py` warns about for a
   *translucent* banded overlay does not apply: two adjacent bands share an
   edge, they do not overlap on top of each other.

WHY THE SWAMP-WATER TEXTURE NEEDED THE OPACITY OFF THE BASE TEXTURE SLOT
----------------------------------------------------------------------------
`OmniPBR` (the base layer under the clearcoat) has exactly one
`texture_scale`/`texture_translate` pair shared by every base-level map —
`diffuse_texture`, `opacity_texture`, `normalmap_texture`, `ORM_texture` all
read it (module docstring precedent above, and `_plans/hurricane_water.md`
S1.3.4/S3.7). The OLD `alpha_map` route needed that ONE slot stretched
`1/span` across the whole plate — a non-repeating cutout mask has to cover
the region exactly once. `WATER_DIFFUSE_TEXTURE` (S3's "surface variation"
ask) wants the OPPOSITE: a small, REPEATING tile (`_WATER_DIFFUSE_REPEATS_
PER_M`, one 20 m tile) so sediment streaks and scum read as texture up close
rather than one smeared photo stretched over 500 m. Both cannot own the same
`texture_scale` at once. Moving opacity OFF the base texture slot entirely —
onto a small set of pre-authored per-band `opacity_constant` materials
(above) — is what frees `texture_scale` for the diffuse tile alone. The
CLEARCOAT's ripple normal was always independent of this (its own
`clearcoat_texture_scale`), and stays exactly as it was.

THE MATERIAL: `OmniPBR_ClearCoat`, NEVER THE VENDORED vMaterials WATER
--------------------------------------------------------------------------
`Water_Blue_Ocean_Perlinwaves.mdl` cannot be made brown — its `water_tint`
is hue-clamped 0.47-0.53 (cyan-to-blue) and its scattering colour is a
hard-coded literal blue, verified by reading the MDL (S1.1). `water_materials`
below uses `OmniPBR_ClearCoat` instead: a Fresnel dielectric coat
(`clearcoat_ior 1.333`, water — not the 1.56 automotive default) over a
diffuse sediment body, all core Kit MDL already in the container. Numbers
from `_plans/hurricane_water.md` S3.7. This is unchanged by the volume
rework — the coat is still what a water film needs regardless of what is
underneath it.

NO PHYSICS, ANYWHERE, EVER
----------------------------
No `PhysicsCollisionAPI`, no `RigidBodyAPI`, on anything this module
authors. A water collider would become "the ground" for `_make_physx_ground_
snap`'s downward raycasts and would occlude ground-truth queries (risk #4 in
the water plan). Every prim here is a static, unphysicalised mesh.
"""

import functools
import hashlib
import math
import os
import random

import numpy as np

from . import scour_relief
from . import tornado

_HERE = os.path.dirname(os.path.abspath(__file__))
# Sibling of `scorch.OUT_DIR` ("assets/materials/scorched") and
# `tools/burn_textures.py`'s `_OUT` ("assets/materials/burn") — the same
# convention, a new leaf for a new disaster's generated maps.
OUT_DIR = os.path.join(os.path.dirname(_HERE), "assets", "materials", "flood")

# A few centimetres above the suburb's real flat grade — the same order of
# magnitude as `suburb_scene._Z_GRASS` (0.02 m) per the water plan's S1.5/S3.11
# risk #1, chosen so authored relief (wrack, washover) does not sink into or
# z-fight the lawn sheet. This module has no access to the actual per-point
# z-ladder value (grass vs walk vs drive), so it uses one constant everywhere
# rather than guess which ladder rung applies — TUNED, NOT SOURCED.
_SURFACE_Z_M = 0.03

# Reused verbatim from `scour_relief._TEX["soil"]` — the flood-silt look IS
# the tornado's mud-scour look; both are turbid ground water leaves behind.
#
# UPGRADED TO THE 2K MAP (`T_pjuph20_2K_*`), not the original `1K`, per a
# second review round: *"the displaced small parts of mud look better but
# they still look mono color. We have a soil texture."* Everything this
# constant feeds — the pond mud base, wrack windrows, the water volume's own
# submerged bed (`build_inundation`), and the launcher's silt overlay
# (`sgw.SILT_TEXTURE` in `suburb_hurricane_launch_script.py`) — is a CLOSE,
# seen-from-low-altitude-or-on-the-ground surface, exactly the
# `Soil_Mud_2K.usda` wrapper's own "use this one for anything seen close"
# guidance. `texture_scale` is repeats-PER-METRE, independent of the source
# image's pixel resolution, so every existing `scale_uv`/`texture_scale`
# tuned against the 1K map tiles at the SAME real-world frequency here —
# only sharper, not differently sized.
SILT_TEXTURE = ("airstack://scene_gen/assets/materials/megascans/"
                "Soil_Mud/T_pjuph20_2K_B.png")
# There is no sand pack in the repo (`_plans/hurricane_water.md` S1.2, S3.10.5:
# "the single highest-value asset acquisition here"), and no 2K version of
# this stand-in was acquired either — `Dirt_Rough` stays 1K. Tinted pale via
# `diffuse_tint`/`albedo_desaturation` in `_dry_material`, it still stands in
# for washover's sandy look.
WASHOVER_TEXTURE = ("airstack://scene_gen/assets/materials/megascans/"
                    "Dirt_Rough/T_yd0lfcqcc_1k_B.png")
# Normal/ORM siblings of the two maps above, same `_B`/`_N`/`_ORM` naming
# `tools/import_megascans.py` documents and generates from ("T_<id>_<res>_
# {B,N,ORM}"), confirmed present on disk next to each `_B` file used here.
# Diffuse-only was the whole of the "flat untextured tan polygon" complaint
# (ponds, wrack, washover all bound `diffuse_texture` alone) — a bound
# normal map is what gives a small, close-up feature actual surface relief
# instead of a painted-on flat photo, and the ORM's occlusion/roughness
# channels are what stop it looking uniformly matte-flat under a raking
# light. Bound in `_dry_material` and in `water_materials`' pond looks.
SILT_NORMAL_TEXTURE = ("airstack://scene_gen/assets/materials/megascans/"
                       "Soil_Mud/T_pjuph20_2K_N.png")
SILT_ORM_TEXTURE = ("airstack://scene_gen/assets/materials/megascans/"
                    "Soil_Mud/T_pjuph20_2K_ORM.png")
WASHOVER_NORMAL_TEXTURE = ("airstack://scene_gen/assets/materials/megascans/"
                          "Dirt_Rough/T_yd0lfcqcc_1k_N.png")
WASHOVER_ORM_TEXTURE = ("airstack://scene_gen/assets/materials/megascans/"
                       "Dirt_Rough/T_yd0lfcqcc_1k_ORM.png")

# The Megascans "swamp water" surface (`tgmjffbqx`), 2K, installed
# specifically for the water BODY's diffuse — see the module docstring's
# "THE WATER VOLUME" section for why a texture is needed at all when S2.2
# already argues floodwater is opaque and therefore near-uniform in colour
# past 30-50 cm: that argument is about the body colour, and it stands: this
# map supplies the SURFACE VARIATION (sediment streaks, slicks, scum) a flat
# constant cannot, which is why the first renders read as "sand" from
# altitude. Bind as `diffuse_texture` with the sediment RGB as `diffuse_tint`
# underneath (`water_materials`'s per-band `_make` calls) — never as a
# normal map. The source asset's own `_N` sibling exists on disk but is
# deliberately UNUSED: `_write_ripple_normal_png` already generates a
# band-limited ripple tuned to a real wavelength range (see that function's
# docstring), and Swamp_Water's normal is not band-limited to anything this
# module's `ripple_m` knob controls.
WATER_DIFFUSE_TEXTURE = ("airstack://scene_gen/assets/materials/megascans/"
                         "Swamp_Water/T_tgmjffbqx_2K_B.png")
WATER_ORM_TEXTURE = ("airstack://scene_gen/assets/materials/megascans/"
                     "Swamp_Water/T_tgmjffbqx_2K_ORM.png")
# 0.05 repeats/m = one 20 m tile, copied from `Swamp_Water.usda`'s own
# comment: flood-surface features (streaks, scum lines) are metres-to-tens-
# of-metres, not centimetres. Safe to use verbatim as the BASE's
# `texture_scale` now that opacity is banded PER-MATERIAL rather than
# painted into a world-stretched raster on the same base texture slot — see
# "THE WATER VOLUME" in the module docstring for the one-`texture_scale`
# conflict this sidesteps.
_WATER_DIFFUSE_REPEATS_PER_M = 0.05


# ---------------------------------------------------------------------------
# defaults and knobs
# ---------------------------------------------------------------------------
#
# Every value is either lifted verbatim from `_plans/hurricane_water.md`
# (cited by section) or marked TUNED, NOT SOURCED where this module had to
# invent a number the plan does not give (mostly the discrete-feature scatter
# rates, which are new to this file — the plan specs the LAYERS, not their
# per-100m2 densities).
DEFAULTS = {
    # Terrain-only seed. Deliberately NOT the `rng` any function below is
    # passed — see the module docstring's "WHY THE TERRAIN NEVER CONSUMES".
    "seed": 0,

    # -- the surge field itself ----------------------------------------
    # S3.0 intensity-level table, level 2 (Cat 2-3 slab-on-grade zone).
    "surge_m": 2.0,
    # Compass-free bearing (mathematical convention: 0 = +x, degrees
    # counter-clockwise, matching every other `*_deg` in this codebase, e.g.
    # `tornado.heading_deg`). Points INLAND — the direction elevation rises
    # in. TUNED, NOT SOURCED: an arbitrary default for a bare `cfg`; a real
    # scene sets it from the preset's coastline.
    "shore_bearing_deg": 0.0,
    # Where the shoreline (ground_z == 0, before relief) crosses the
    # `shore_bearing_deg` axis, measured from the WORLD ORIGIN — which is
    # "distance from plate centre" (S3.0 "the field that drives everything")
    # because every region in this codebase is centred on the origin (see
    # module docstring). 0.0 = shoreline runs through the origin.
    "shore_offset_m": 0.0,
    # TUNED, NOT SOURCED FOR THIS PLATE SIZE. The plan's physically-sourced
    # `land_slope` values (S2.4's "inland extent = surge / slope", S3.0's
    # 1/600-1/900) are calibrated against a 1 km plate. Applied unmodified to
    # the 500 m test plate this module is verified against, they leave the
    # scene either almost entirely dry or almost entirely flooded depending
    # on `shore_offset_m` — there is no offset that reproduces the plan's
    # "~35% dry at level 3" on a plate a fifth the size. This value is
    # picked instead so that outcome (S3.0, "why level 3 is 2.8 and not
    # 3.6") holds AT THIS PLATE SIZE with `shore_offset_m = 0`; see the
    # verification report for the numbers. Expressed as a PERCENTAGE
    # (rise/run x 100), matching the literal name.
    "slope_pct": 4.0,
    # S3.0 "the field that drives everything", item 2: amplitude of the
    # virtual relief that turns a straight contour into a ragged shoreline.
    "relief_m": 0.45,
    "relief_wavelength_m": (60.0, 250.0),
    # S3.0 item 3: the street trench. Only applied where `cfg["pavement_at"]`
    # is supplied (see `_ground_z_point`) — off by default because this
    # module has no road-corridor data of its own.
    "street_drop_m": 0.18,
    "pavement_at": None,

    # -- L1 inundation surface / alpha map -------------------------------
    "d_opaque_m": 0.35,     # S2.2: opaque past 30-50 cm at 300+ FNU
    "alpha_gamma": 0.7,     # S3.0 L1 alpha formula
    "alpha_px": 2048,       # raised from the original 512 after a review
                            # called the shoreline a "visibly stepped edge" —
                            # 512 over a 500 m plate is ~1 m/px, coarse enough
                            # to show as blocks even from altitude. Plan's own
                            # production value is 4096 (S3.0 "Resolution");
                            # 2048 (~0.24 m/px on a 500 m plate) is kept as
                            # the DEFAULT rather than jumping straight to
                            # 4096, since this is still the value every bench
                            # run pays for, not just a final export -- pass a
                            # larger `n` to `alpha_map`/`build_inundation`
                            # for a full production bake. Only consumed by
                            # the LEGACY flat quad (`SURGE_FLAT_WATER=1`) and
                            # by `pond_specs`' "already opaque, skip" gate —
                            # the default volumetric `build_inundation` does
                            # not rasterise an alpha map at all (see
                            # `water_volume_cell_m` below).

    # -- L1 water VOLUME (the default `build_inundation` path) ------------
    # Grid spacing for `water_volume_mesh`'s marching-triangles footprint,
    # in metres. TUNED against the 500 m verification plate: the terrain's
    # relief harmonics (`relief_wavelength_m`, above) run 60-250 m, and 8 m
    # gives ~7.5 samples across the SHORTEST of those wavelengths — enough
    # that the shoreline reads as a smooth curve rather than the "low poly"
    # facets a review already called out once for pond outlines
    # (`_POND_SIDES`'s own docstring). On the 500x500 m region this module is
    # verified against that lands at ~2,300 footprint vertices / ~4,400
    # triangles before doubling for the top+bottom volume (see
    # `water_volume_mesh`'s docstring) — thousands, not tens of thousands.
    # A launcher targeting a much larger plate should raise this
    # (`SURGE_VOLUME_CELL_M`) rather than pay the same density over more
    # area: this value is an absolute metre spacing, like `pond_cell_m`/
    # `wrack_cell_m` below, not one that auto-scales with `region`.
    "water_volume_cell_m": 8.0,

    # -- water material ---------------------------------------------------
    "turbidity": 0.8,       # S3.0 L1 knob table
    "palette": "sediment",  # S3.0 L1 knob table; S3.7 gives the four palettes
    "chop": 0.10,           # S3.7 chop table, "flooded street, mild wind" row
    "ripple_m": 2.5,        # S3.0 L1 knob table

    # -- L2 ponding -- TUNED, NOT SOURCED (the plan specs the LAYER, not a
    # density; scaled off `scour_relief.DEFAULT_KNOBS`'s per-100m2 idiom) --
    "pond_cell_m": 12.0,
    "pond_edge_width_m": 0.30,
    "pond_base_per_100m2": 0.15,
    "pond_edge_boost_per_100m2": 1.2,
    "pond_radius_m": (0.6, 3.5),

    # -- L4b wrack windrows -- TUNED, NOT SOURCED for the rates; the
    # geometry itself reuses `scour_relief._ridge_spec` verbatim (S3.0 L4b) --
    "wrack_cell_m": 10.0,
    # "along the alpha = 0.02 contour" (S3.0 L4b) is, by the L1 alpha formula
    # with d_opaque=0.35, gamma=0.7, a depth of ~1 mm -- for scatter purposes
    # that is simply "right at the waterline", so the hump below is centred
    # on depth 0.0 with a narrow width rather than solving the alpha
    # equation for its exact depth.
    "wrack_peak_depth_m": 0.0,
    "wrack_width_m": 0.15,
    "wrack_per_100m2": 0.20,
    "wrack_height_scale": 1.0,
    "obstruction_boost": None,   # optional (x,y)->weight hook; see wrack_specs

    # -- L4a silt (`silt_coverage`, consumed by the LAUNCHER's own
    # `ground.build_overlay` call -- see `build_deposits`'s docstring for why
    # that overlay is not ALSO authored inside this module) -- peak/width
    # TUNED, NOT SOURCED --
    "silt_peak_depth_m": 0.15,
    "silt_width_m": 0.6,

    # -- L4c washover -- height range is S2.5 verbatim ("0.5-1 m thick" at
    # North Core Banks / Florence, generalised here to 0.3-1.0 m); placement
    # hump and radius are TUNED, NOT SOURCED --
    "washover_peak_depth_m": -0.05,   # just inland (dry side) of the margin
    "washover_width_m": 0.25,
    "washover_fans": 6,               # S3.0 L4c: "a handful"
    "washover_radius_m": (3.0, 9.0),
    "washover_height_m": (0.30, 1.0),

    # -- ground truth / building state ------------------------------------
    # NOT precisely sourced: the water plan does not give a still-water
    # depth threshold for "swept clean" (that finding, S2.4, is expressed as
    # wave-crest-vs-lowest-floor, which this water-only module does not
    # model). Anchored loosely to Kennedy et al.'s Bolivar/Ike study, where
    # the destroyed zone sat in 3-5 m of still water with 1-2 m waves — set
    # below that, since this module has no wave-height field of its own.
    # Flagged in the report as approximate.
    "swept_depth_m": 2.5,
    # S2.5 / S3.0 L3: "water_z + peak_drop_m, default 0.35 m at T+3h on the
    # falling limb".
    "peak_drop_m": 0.35,
}


def resolve_cfg(config):
    """`DEFAULTS` with `config` layered over it. Idempotent: resolving an
    already-resolved cfg is a no-op, so every function below can call this
    on whatever it is handed without worrying whether it already ran."""
    kn = dict(DEFAULTS)
    kn.update(config or {})
    return kn


def knobs_from_env(span_m):
    """`DEFAULTS` overridden by `SURGE_*` environment variables.

    Parallel to `ground.knobs_from_env`/`scour_relief.knobs_from_env`: what
    gets tuned on a bench means the same thing in a scene. `span_m` sets the
    one knob whose sensible default scales with the plate (`shore_offset_m`
    defaults to the LOW edge of the plate along `shore_bearing_deg`, so an
    unconfigured scene floods from one side rather than from its own centre
    outward — closer to what an actual coastal plate looks like).
    """
    span_m = float(span_m)

    def _f(name, default):
        raw = os.environ.get("SURGE_" + name)
        return float(default) if raw is None or raw.strip() == "" else float(raw)

    return dict(
        seed=int(_f("SEED", DEFAULTS["seed"])),
        surge_m=_f("LEVEL_M", DEFAULTS["surge_m"]),
        shore_bearing_deg=_f("SHORE_BEARING_DEG", DEFAULTS["shore_bearing_deg"]),
        shore_offset_m=_f("SHORE_OFFSET_M", -0.5 * span_m),
        slope_pct=_f("SLOPE_PCT", DEFAULTS["slope_pct"]),
        relief_m=_f("RELIEF_M", DEFAULTS["relief_m"]),
        street_drop_m=_f("STREET_DROP_M", DEFAULTS["street_drop_m"]),
        d_opaque_m=_f("OPAQUE_M", DEFAULTS["d_opaque_m"]),
        turbidity=_f("TURBIDITY", DEFAULTS["turbidity"]),
        palette=os.environ.get("SURGE_PALETTE", DEFAULTS["palette"]),
        chop=_f("CHOP", DEFAULTS["chop"]),
        ripple_m=_f("RIPPLE_M", DEFAULTS["ripple_m"]),
        alpha_px=int(_f("ALPHA_PX", DEFAULTS["alpha_px"])),
        water_volume_cell_m=_f("VOLUME_CELL_M", DEFAULTS["water_volume_cell_m"]),
    )


def enabled():
    """`SURGE=0` turns the whole pass off — the `SCOUR_RELIEF`/`GROUND_*`
    convention (`_plans/hurricane_water.md` S3.0 "Knobs")."""
    return os.environ.get("SURGE", "1").strip().lower() not in (
        "0", "false", "no")


# ---------------------------------------------------------------------------
# region handling
# ---------------------------------------------------------------------------

def _bounds(region):
    """Normalise `region` to `(x0, y0, x1, y1)`.

    Every sibling module (`ground`, `scour_relief`, `tornado`) is called
    with the plate's four corners already, and every shipped example is
    CENTRED ON THE ORIGIN (`tests/test_scour_relief.py: REGION = (-250,
    -250, 250, 250)` for a 500 m plate). This additionally accepts the bare
    `(width_m, height_m)` a `region_m` preset leaf carries, centring it the
    same way every plate in this codebase is centred — so `(500, 500)`
    means the same 500 m plate `test_scour_relief.py`'s explicit corners do.
    """
    region = tuple(float(q) for q in region)
    if len(region) == 4:
        return region
    if len(region) == 2:
        w, h = region
        return (-0.5 * w, -0.5 * h, 0.5 * w, 0.5 * h)
    raise ValueError(
        "region must be (x0,y0,x1,y1) or (w,h), got {0!r}".format(region))


# ---------------------------------------------------------------------------
# the synthetic terrain
# ---------------------------------------------------------------------------
#
# See the module docstring's "THE CORE PROBLEM" and "WHY `house_water_state`
# HAS NO `region`" sections for why this is analytic (a handful of sine
# harmonics) rather than a raster, and why it is seeded from `cfg["seed"]`
# rather than from any function's `rng` argument.

# 5, not 3: `scour_relief._wob` gets away with 3 harmonics because it is
# shaping a single metre-scale mound's OUTLINE; this is shaping a whole
# plate's elevation as seen from directly overhead, which needs more terms
# before the sum stops reading as "one visible wave direction". TUNED.
_N_RELIEF_HARMONICS = 5


@functools.lru_cache(maxsize=64)
def _relief_harmonics_cached(seed, lo_m, hi_m, n_terms):
    """`n_terms` deterministic `(wavelength_m, direction_rad, phase, weight)`
    tuples. Cached because every point query (`house_water_state`,
    `pond_specs`, `wrack_specs`'s per-candidate evaluation) would otherwise
    reconstruct a fresh `random.Random` and redraw the same terms."""
    local = random.Random(seed * 1000003 + 7)
    out = []
    for _ in range(n_terms):
        out.append((local.uniform(lo_m, hi_m),
                   local.uniform(0.0, 2.0 * math.pi),
                   local.uniform(0.0, 2.0 * math.pi),
                   local.uniform(0.5, 1.0)))
    return tuple(out)


def _relief_harmonics(cfg):
    lo, hi = cfg.get("relief_wavelength_m", DEFAULTS["relief_wavelength_m"])
    return _relief_harmonics_cached(int(cfg.get("seed", 0)), float(lo),
                                    float(hi), _N_RELIEF_HARMONICS)


def _ground_z_point(cfg, x, y):
    """The synthetic terrain height at one point, in metres. See module
    docstring — this is a bookkeeping surface, never a rendered one."""
    theta = math.radians(float(cfg["shore_bearing_deg"]))
    u = x * math.cos(theta) + y * math.sin(theta)
    z = (float(cfg["slope_pct"]) / 100.0) * (u - float(cfg["shore_offset_m"]))
    harmonics = _relief_harmonics(cfg)
    total_w = sum(h[3] for h in harmonics) or 1.0
    relief = 0.0
    for wl, ang, ph, w in harmonics:
        k = 2.0 * math.pi / max(1.0, wl)
        relief += w * math.sin(k * (x * math.cos(ang) + y * math.sin(ang)) + ph)
    z += float(cfg["relief_m"]) * relief / total_w
    pav = cfg.get("pavement_at")
    if pav is not None and pav(x, y):
        z -= float(cfg["street_drop_m"])
    return z


def _ground_z_array(cfg, xs, ys):
    """Vectorised `_ground_z_point` over `numpy` meshgrid arrays `xs`, `ys`.

    The optional `pavement_at` hook is evaluated per point in a Python loop
    when present — it is an arbitrary Python callable, not itself
    vectorised, and this module has no road-corridor data of its own to
    supply one by default. The common (offline, no launcher-supplied roads)
    case stays fully vectorised.
    """
    theta = math.radians(float(cfg["shore_bearing_deg"]))
    u = xs * math.cos(theta) + ys * math.sin(theta)
    z = (float(cfg["slope_pct"]) / 100.0) * (u - float(cfg["shore_offset_m"]))
    harmonics = _relief_harmonics(cfg)
    total_w = sum(h[3] for h in harmonics) or 1.0
    relief = np.zeros_like(xs, dtype=float)
    for wl, ang, ph, w in harmonics:
        k = 2.0 * math.pi / max(1.0, wl)
        relief = relief + w * np.sin(
            k * (xs * math.cos(ang) + ys * math.sin(ang)) + ph)
    z = z + float(cfg["relief_m"]) * relief / total_w
    pav = cfg.get("pavement_at")
    if pav is not None:
        drop = float(cfg["street_drop_m"])
        it = np.nditer(z, flags=["multi_index"])
        for _ in it:
            i, j = it.multi_index
            if pav(float(xs[i, j]), float(ys[i, j])):
                z[i, j] -= drop
    return z


def ground_z(cfg, region, rng):
    """`(x, y) -> SYNTHETIC terrain height (m)`.

    `region` is validated (so a malformed caller fails here, not three
    calls later inside a numpy reshape) but not otherwise consulted — see
    the module docstring. `rng` is accepted for signature symmetry and not
    consumed; the terrain is deterministic in `cfg` alone.
    """
    kn = resolve_cfg(cfg)
    _bounds(region)

    def f(x, y):
        return _ground_z_point(kn, float(x), float(y))

    return f


def water_level(cfg):
    """Scalar world Z (m) of the still-water plane.

    Equal to `surge_m` — the height above grade the definition of surge
    already is (S2.1: "inundation = water depth over normally dry ground").
    Floored at 0.30 m per `_plans/hurricane_water.md` S3.11 risk #1: the
    suburb's ground z-ladder runs grass 0.02 -> dash 0.24 m, and a water
    plane inside that range would poke through sidewalks and z-fight the
    kerb geometry.
    """
    kn = resolve_cfg(cfg)
    return max(0.30, float(kn["surge_m"]))


def depth_at(cfg, region, rng):
    """`(x, y) -> inundation depth (m), 0 if dry`.

    `depth_at = water_level(cfg) - ground_z(x, y)`, clamped at 0 — the
    literal definition the task spec gives, and the one place this module's
    "apparent depth is a fiction outside the shading" simplification
    (`_plans/hurricane_water.md` S3.8) lives: because `ground_z` is not real
    displacement, this number drives BOTH what `alpha_map` paints and what
    `coverage`/ground-truth read, which is internally consistent (the label
    always agrees with the picture) but is not the same as a real flood's
    depth field, where a wall's mud line and a road's puddle are different
    physical measurements. The real fix is a ground height field — a
    separate project, per the plan's "Known gaps" #1.
    """
    kn = resolve_cfg(cfg)
    gz = ground_z(kn, region, rng)
    wl = water_level(kn)

    def f(x, y):
        return max(0.0, wl - gz(x, y))

    return f


def _signed_depth_point(cfg, x, y):
    """`water_level - ground_z`, UNCLAMPED: positive when wet, negative when
    dry, with the magnitude telling you how far above or below the waterline
    the synthetic terrain sits.

    This is NOT `depth_at` (which clamps at 0, correctly, since a physical
    depth cannot be negative) and every "how close is this point to the
    shoreline" hump in this module (`pond_specs`'s edge boost, `wrack_specs`,
    `_washover_specs`, `silt_coverage`) must use THIS, not `depth_at`'s
    clamped output. Caught during verification: with the clamped depth, a
    point one metre past the flood boundary and a point a kilometre past it
    are BOTH exactly 0.0 — indistinguishable — so a hump built on `depth_at`
    alone does not decay with distance from the margin at all once the point
    is dry; it stays at whatever value depth-zero maps to, EVERYWHERE dry.
    `silt_coverage`'s own docstring promises "outside the wetted band this is
    IDENTICALLY ZERO", which the clamped version cannot deliver.
    """
    return water_level(cfg) - _ground_z_point(cfg, float(x), float(y))


def coverage(cfg, region, rng, n=96):
    """Fraction of the plate with `depth_at > 0`, sampled on an `n x n`
    lattice of cell centres."""
    kn = resolve_cfg(cfg)
    x0, y0, x1, y1 = _bounds(region)
    n = max(1, int(n))
    xs = np.linspace(x0 + 0.5 * (x1 - x0) / n, x1 - 0.5 * (x1 - x0) / n, n)
    ys = np.linspace(y0 + 0.5 * (y1 - y0) / n, y1 - 0.5 * (y1 - y0) / n, n)
    xg, yg = np.meshgrid(xs, ys)
    z = _ground_z_array(kn, xg, yg)
    depth = np.maximum(0.0, water_level(kn) - z)
    return float(np.mean(depth > 1e-6))


def alpha_map(cfg, region, rng, n=512):
    """`n x n` array of 0..1 opacity, host-side (no `pxr`).

    `alpha(x,y) = 0` where dry, `clamp(depth/d_opaque)**gamma` in the
    margin, `1` once fully opaque — `_plans/hurricane_water.md` S3.0 L1
    alpha formula, the feathered-edge idea `ground.feathered_coverage` uses
    for the burn scar transposed onto `depth_at` instead of a fire-arrival
    field. Row 0 of the returned array is the region's low-Y edge (`y0`);
    a PNG writer wanting conventional top-left-origin image rows should
    flip vertically (`build_inundation`'s `_write_alpha_png` does this).
    """
    kn = resolve_cfg(cfg)
    x0, y0, x1, y1 = _bounds(region)
    n = max(2, int(n))
    xs = np.linspace(x0 + 0.5 * (x1 - x0) / n, x1 - 0.5 * (x1 - x0) / n, n)
    ys = np.linspace(y0 + 0.5 * (y1 - y0) / n, y1 - 0.5 * (y1 - y0) / n, n)
    xg, yg = np.meshgrid(xs, ys)
    z = _ground_z_array(kn, xg, yg)
    depth = np.maximum(0.0, water_level(kn) - z)
    d_opaque = max(1e-6, float(kn["d_opaque_m"]))
    gamma = float(kn["alpha_gamma"])
    return np.clip(depth / d_opaque, 0.0, 1.0) ** gamma


def _alpha_at(cfg, depth_m):
    """Scalar form of the `alpha_map` formula, for per-point decisions
    (`pond_specs`'s "already opaque, do not draw a pond here" gate)."""
    d_opaque = max(1e-6, float(cfg["d_opaque_m"]))
    gamma = float(cfg["alpha_gamma"])
    return max(0.0, min(1.0, float(depth_m) / d_opaque)) ** gamma


def _grad_deg(fn, x, y, h=1.0):
    """Direction (degrees) of steepest ascent of scalar field `fn` at
    `(x, y)`, by central differences — the same construction
    `ground.feathered_coverage` uses for its arrival-time gradient. For
    `depth_at`, "steepest ascent" points toward deeper water, i.e. seaward:
    exactly `flow_deg`'s meaning in `scour_relief._ridge_spec` (the
    direction material is moving), which is what lets `wrack_specs` hand
    that function this value directly and get a shore-PARALLEL windrow for
    free (`_ridge_spec` orients its ridge at `flow_deg + 90`)."""
    dx = (fn(x + h, y) - fn(x - h, y)) / (2.0 * h)
    dy = (fn(x, y + h) - fn(x, y - h)) / (2.0 * h)
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return math.degrees(math.atan2(dy, dx))


def _arc_length(stations):
    total = 0.0
    for a, b in zip(stations, stations[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def _draw(lam, rng):
    """A count from an expected value: integer part plus a coin on the
    remainder. The same draw `scour_relief._draw`/`planks.scatter_over_region`
    use, copied rather than imported because it is two lines and importing a
    single private free function across a module boundary for that would be
    more code than the function itself."""
    n = int(lam)
    return n + (1 if rng.random() < (lam - n) else 0)


class _NumpyRandomAdapter(object):
    """The slice of `random.Random`'s scalar interface this module's reused
    `scour_relief` helpers need (`.gauss`, `.randrange`), backed by a
    `numpy.random.Generator`. See `_as_random`."""

    def __init__(self, gen):
        self._gen = gen

    def random(self):
        return float(self._gen.random())

    def uniform(self, a, b):
        return float(self._gen.uniform(a, b))

    def gauss(self, mu, sigma):
        return float(self._gen.normal(mu, sigma))

    def randrange(self, n):
        return int(self._gen.integers(0, n))

    def choice(self, seq):
        return seq[self.randrange(len(seq))]


def _as_random(rng):
    """Coerce `rng` to something with `random.Random`'s scalar interface.

    This module's discrete scatter (`pond_specs`, `wrack_specs`,
    `_washover_specs`, `house_water_state`) threads `rng` straight into
    `scour_relief`'s reused helpers (`_ridge_spec`, `_wob`), which are
    written against `random.Random` and call `.gauss()` — a method
    `numpy.random.Generator` does not have (it has `.normal()` instead).
    The assembly launcher that consumes this module passes a
    `random.Random` to most calls but a `numpy.random.Generator` to
    `summarise` (consistent with the numpy-seeded noise fields elsewhere in
    the hurricane pipeline, e.g. `silt_coverage` below), so `summarise` —
    which calls the scatter functions to report counts — would otherwise
    crash on the first `.gauss()` call. Detected by duck-typing
    (`.gauss`/`.randrange` present -> already compatible) rather than an
    `isinstance` check, so a third rng-like object with the same interface
    also passes through untouched.
    """
    if hasattr(rng, "gauss") and hasattr(rng, "randrange"):
        return rng
    return _NumpyRandomAdapter(rng)


# ---------------------------------------------------------------------------
# discrete features — pure Python, `build_*` draws no random numbers of its
# own beyond what these specs already fixed (the `scour_relief` contract:
# "EVERY SPEC IS A COMPLETE DESCRIPTION")
# ---------------------------------------------------------------------------

def pond_specs(cfg, region, rng):
    """Isolated standing-water discs: puddles in road hollows, slack
    corners, the flooded parking lot, the still cul-de-sac.

    Per `_plans/hurricane_water.md` S3.0 L2, these are the ONLY surfaces in
    the scene genuinely mirror-like at nadir (a wet road is not — S2.2's
    nadir Fresnel correction). Density is a low BACKGROUND rate everywhere
    (rain fills hollows regardless of the surge) plus a boost near the
    shallow flood margin, where residual pockets are common as the water
    recedes; suppressed where `alpha_map`'s opacity is already >= 0.6, since
    a pond drawn under an already-opaque L1 sheet would be invisible.
    """
    kn = resolve_cfg(cfg)
    rng = _as_random(rng)
    x0, y0, x1, y1 = _bounds(region)
    depth_fn = depth_at(kn, region, rng)
    pav = kn.get("pavement_at")
    edge_w = max(1e-6, float(kn["pond_edge_width_m"]))
    cell = max(4.0, float(kn["pond_cell_m"]))
    nx = max(1, int(round((x1 - x0) / cell)))
    ny = max(1, int(round((y1 - y0) / cell)))
    dx, dy = (x1 - x0) / nx, (y1 - y0) / ny
    area_100 = dx * dy / 100.0
    base = float(kn["pond_base_per_100m2"])
    boost = float(kn["pond_edge_boost_per_100m2"])
    r_lo, r_hi = kn["pond_radius_m"]

    out = []
    for iy in range(ny):
        for ix in range(nx):
            ax, ay = x0 + ix * dx, y0 + iy * dy
            cx, cy = ax + dx * 0.5, ay + dy * 0.5
            d = depth_fn(cx, cy)
            if _alpha_at(kn, d) >= 0.6:
                continue
            paved = bool(pav(cx, cy)) if pav is not None else False
            # SIGNED depth for the hump, not the clamped `d` above — a point
            # far inland and a point one centimetre past the margin are both
            # `d == 0.0`, so a hump built on the clamped value would not
            # decay with distance from the shore at all once dry (see
            # `_signed_depth_point`'s docstring). `base` still uses `d`-blind
            # uniform placement on purpose: rain fills hollows everywhere,
            # not only near the flood.
            sd = _signed_depth_point(kn, cx, cy)
            hump = math.exp(-(sd / edge_w) ** 2)
            rate = base * (1.7 if paved else 1.0) + boost * hump
            for _ in range(_draw(rate * area_100, rng)):
                px = ax + rng.random() * dx
                py = ay + rng.random() * dy
                dp = depth_fn(px, py)
                depth_m = dp if 0.02 < dp < 0.30 else rng.uniform(0.03, 0.12)
                out.append({
                    "x": round(px, 3), "y": round(py, 3),
                    "r_m": round(rng.uniform(r_lo, r_hi), 3),
                    "depth_m": round(depth_m, 3),
                    "paved": paved,
                })
    return out


def wrack_specs(cfg, region, rng):
    """Wrack windrows: the semi-continuous line of marsh grass, leaf litter,
    plastic and lumber stranded at the falling waterline (S2.5).

    Geometry is `scour_relief._ridge_spec` UNCHANGED (S3.0 L4b names this
    exact reuse) — a windrow and a tornado's mud-scour ridge are the same
    landform for a different reason. `flow_deg` is `depth_at`'s local
    gradient direction (`_grad_deg`), which orients each windrow
    shore-PARALLEL automatically (see `_grad_deg`'s docstring), so the line
    follows the plate's own ragged, noise-perturbed shoreline rather than one
    fixed global bearing. `cfg["obstruction_boost"]`, if given, is an
    `(x, y) -> weight` hook a launcher with fence/building data can supply to
    thicken windrows against obstructions (S2.5); defaults to uniform weight
    because this module has no parcel data of its own.
    """
    kn = resolve_cfg(cfg)
    rng = _as_random(rng)
    x0, y0, x1, y1 = _bounds(region)
    depth_fn = depth_at(kn, region, rng)
    peak = float(kn["wrack_peak_depth_m"])
    width = max(1e-6, float(kn["wrack_width_m"]))
    rate = float(kn["wrack_per_100m2"])
    hs = float(kn["wrack_height_scale"])
    obstruct = kn.get("obstruction_boost") or (lambda x, y: 1.0)
    cell = max(6.0, float(kn["wrack_cell_m"]))
    nx = max(1, int(round((x1 - x0) / cell)))
    ny = max(1, int(round((y1 - y0) / cell)))
    dx, dy = (x1 - x0) / nx, (y1 - y0) / ny
    area_100 = dx * dy / 100.0

    out = []
    for iy in range(ny):
        for ix in range(nx):
            ax, ay = x0 + ix * dx, y0 + iy * dy
            cx, cy = ax + dx * 0.5, ay + dy * 0.5
            # SIGNED depth -- see `_signed_depth_point`'s docstring. `peak`
            # is 0.0 (the waterline itself), and without the sign a point a
            # kilometre inland would score identically to one metre inland.
            hump = math.exp(-((_signed_depth_point(kn, cx, cy) - peak)
                              / width) ** 2)
            n_w = _draw(rate * area_100 * hump * float(obstruct(cx, cy)), rng)
            for _ in range(n_w):
                px = ax + rng.random() * dx
                py = ay + rng.random() * dy
                sd_here = _signed_depth_point(kn, px, py)
                ramp = math.exp(-((sd_here - peak) / width) ** 2)
                flow_deg = _grad_deg(depth_fn, px, py)
                spec = scour_relief._ridge_spec(px, py, _SURFACE_Z_M,
                                                flow_deg, ramp, hs, rng)
                spec["cls"] = "wrack"
                st = spec["stations"]
                sx, sy = st[0][0], st[0][1]
                ex, ey = st[-1][0], st[-1][1]
                spec["yaw"] = round(math.degrees(
                    math.atan2(ey - sy, ex - sx)), 2)
                spec["len_m"] = round(_arc_length(st), 3)
                spec["width_m"] = round(max(s[4] + s[5] for s in st), 3)
                spec["height_m"] = round(max(max(s[2], s[3]) for s in st), 3)
                out.append(spec)
    return out


def _washover_specs(cfg, region, rng):
    """Low sand fan/mound "shadow" specs near the shore, in
    `scour_relief.geometry`'s `mound`/`fan` schema. Pure Python — see
    `build_deposits` for why the overlay-colour half of the plan's "both
    routes" recommendation (S3.0 L4c) is not authored here.
    """
    rng = _as_random(rng)
    x0, y0, x1, y1 = _bounds(region)
    peak = float(cfg["washover_peak_depth_m"])
    width = max(1e-6, float(cfg["washover_width_m"]))
    n_target = max(0, int(cfg["washover_fans"]))
    r_lo, r_hi = cfg["washover_radius_m"]
    h_lo, h_hi = cfg["washover_height_m"]
    bearing = float(cfg["shore_bearing_deg"])

    out = []
    tries = 0
    max_tries = max(1, n_target) * 60
    while len(out) < n_target and tries < max_tries:
        tries += 1
        x = rng.uniform(x0, x1)
        y = rng.uniform(y0, y1)
        # SIGNED depth: `peak` is slightly NEGATIVE (just inland/dry of the
        # margin), which only means anything against the unclamped value —
        # see `_signed_depth_point`.
        w = math.exp(-((_signed_depth_point(cfg, x, y) - peak) / width) ** 2)
        if rng.random() > w:
            continue
        rx = rng.uniform(r_lo, r_hi)
        out.append({
            "kind": "fan" if rng.random() < 0.5 else "mound",
            "cls": "sand",
            "x": x, "y": y, "z": _SURFACE_Z_M, "base": _SURFACE_Z_M,
            "rx": rx, "ry": rx * rng.uniform(0.5, 0.9),
            "h": rng.uniform(h_lo, h_hi),
            "yaw": bearing + rng.gauss(0.0, 15.0),
            "wob": scour_relief._wob(rng),
        })
    return out


def house_water_state(cfg, x, y, rng):
    """Per-point flood state: `{depth, mudline_z, seed_line_z, swept}`.

    No `region` argument — see the module docstring for why `ground_z` does
    not need one. `mudline_z`/`seed_line_z` are `None` at a dry point (there
    is no waterline to draw on a house the flood never reached). `swept` is
    True only past `swept_depth_m` — the launcher's cue to place a bare slab
    instead of a standing house (`_plans/hurricane_water.md`: "slab swept
    clean is surge, never wind", and it must stay inside the inundation
    footprint, never scattered inland — enforced here for free, since it can
    only be True where `depth > swept_depth_m > 0`).
    """
    kn = resolve_cfg(cfg)
    rng = _as_random(rng)
    gz = _ground_z_point(kn, float(x), float(y))
    wl = water_level(kn)
    depth = max(0.0, wl - gz)
    wet = depth > 1e-6
    if not wet:
        return {"depth": 0.0, "mudline_z": None, "seed_line_z": None,
               "swept": False}
    # Per-house jitter on how far the tide had fallen: USGS's own
    # high-water-mark uncertainty is quoted in tenths of a foot (~3 cm, S2.5),
    # i.e. real seed lines vary house to house rather than sitting at one
    # fixed offset everywhere.
    peak_drop = float(kn["peak_drop_m"]) * rng.uniform(0.85, 1.15)
    return {
        "depth": round(depth, 3),
        "mudline_z": round(wl, 3),
        "seed_line_z": round(wl + peak_drop, 3),
        "swept": depth >= float(kn["swept_depth_m"]),
    }


def summarise(cfg, region, rng, n=64):
    """Everything worth knowing about a configuration, with no stage —
    the `scour_relief.summarise`/`tornado.summarise` idiom: the numbers that
    decide whether a pass is worth a container launch, all cheap."""
    kn = resolve_cfg(cfg)
    ponds = pond_specs(kn, region, rng)
    wrack = wrack_specs(kn, region, rng)
    washover = _washover_specs(kn, region, rng)
    return {
        "surge_m": round(float(kn["surge_m"]), 3),
        "water_level_m": round(water_level(kn), 3),
        "slope_pct": round(float(kn["slope_pct"]), 3),
        "shore_bearing_deg": round(float(kn["shore_bearing_deg"]), 1),
        "shore_offset_m": round(float(kn["shore_offset_m"]), 1),
        "coverage_frac": round(coverage(kn, region, rng, n=n), 4),
        "ponds": len(ponds),
        "ponds_paved": sum(1 for p in ponds if p.get("paved")),
        "wrack_windrows": len(wrack),
        "washover_fans": len(washover),
    }


def silt_coverage(cfg, region, rng, gamma=0.85, islands=0.05):
    """`(x, y) -> 0..1` for `ground.build_overlay`: flood silt.

    Modelled on `tornado.scour_coverage` (same shape of problem: an intensity
    field, a fingered/quantile-thresholded island removal reusing
    `tornado._island_field` verbatim, a `gamma` that widens or narrows the
    visibly muddy band without moving the underlying field) but driven by
    INUNDATION rather than by distance from a track. Silt is deposited where
    water STOOD AND DRAINED, which is a hump peaking at a shallow depth just
    inside the shoreline (`silt_peak_depth_m`/`silt_width_m`) and fading BOTH
    seaward (fast, deep flow scours rather than deposits) and landward (dry —
    nothing to deposit) — `_plans/hurricane_water.md` S3.0 L4a.

    `rng` MUST be a `numpy.random.Generator` (NOT the `random.Random` most of
    this module's other scatter functions take) — it is passed straight into
    `tornado._island_field`, which draws a `scorch._noise` array and needs
    `.normal(size=...)`. This mirrors `tornado.scour_coverage`'s own
    (undocumented, but consistent) assumption and how `scene_api.py` already
    calls the tornado/burn-scar family of coverage functions elsewhere.

    Outside the wetted band this is IDENTICALLY ZERO: the hump is already 0
    for any depth far from `silt_peak_depth_m`, and the island field can only
    ever SUBTRACT coverage (`1.0 - isl(x, y)`), never add it — the same rule
    `ground.py`'s docstring states for the burn scar ("the noise only ever
    MOVES the boundary or REMOVES coverage"). So dry ground never comes out
    speckled with mud regardless of how the island threshold is tuned.
    """
    kn = resolve_cfg(cfg)
    peak = float(kn["silt_peak_depth_m"])
    width = max(1e-6, float(kn["silt_width_m"]))
    _f, isl = tornado._island_field(_bounds(region), rng, islands)
    g = max(1e-3, float(gamma))

    def coverage_at(x, y):
        # SIGNED depth, not `depth_at`'s clamped output -- see
        # `_signed_depth_point`'s docstring. Verified during this module's
        # own review: with the clamped depth, every dry point (regardless of
        # distance from the shore) mapped to the SAME hump value, which is
        # exactly the "speckled with mud a kilometre inland" failure this
        # docstring promises does not happen.
        base = math.exp(-((_signed_depth_point(kn, x, y) - peak) / width) ** 2)
        if base <= 0.0:
            return 0.0
        return min(1.0, base ** g) * (1.0 - isl(x, y))

    return coverage_at


def review_points(cfg, region):
    """2-4 world points `{name: (x, y)}` for a review camera, chosen along
    the WATER GRADIENT rather than along a track — there is no track here.

    `"shoreline"` sits at a partly-opaque depth (half of `d_opaque_m`): wet
    enough to be unambiguously in frame, not so deep that the shore itself is
    out of shot. `"deep_water"` is the single deepest point on the plate
    (dropped if the plate is entirely dry). `"dry_inland"` is the driest
    (highest-`ground_z`) point. No `rng` — deterministic in `cfg`/`region`
    alone, sampled on a modest lattice (cheap: `48*48` `depth_at` calls).
    """
    kn = resolve_cfg(cfg)
    x0, y0, x1, y1 = _bounds(region)
    n = 48
    depth_fn = depth_at(kn, region, None)
    target = 0.5 * float(kn["d_opaque_m"])

    best_shore, shore_err = None, float("inf")
    best_deep, deep_d = None, -1.0
    best_dry, dry_d = None, float("inf")
    for j in range(n):
        y = y0 + (j + 0.5) * (y1 - y0) / n
        for i in range(n):
            x = x0 + (i + 0.5) * (x1 - x0) / n
            d = depth_fn(x, y)
            err = abs(d - target)
            if err < shore_err:
                shore_err, best_shore = err, (x, y)
            if d > deep_d:
                deep_d, best_deep = d, (x, y)
            if d < dry_d:
                dry_d, best_dry = d, (x, y)

    out = {}
    if best_shore is not None:
        out["shoreline"] = (round(best_shore[0], 2), round(best_shore[1], 2))
    if best_deep is not None and deep_d > 1e-6:
        out["deep_water"] = (round(best_deep[0], 2), round(best_deep[1], 2))
    if best_dry is not None:
        out["dry_inland"] = (round(best_dry[0], 2), round(best_dry[1], 2))
    return out


# ---------------------------------------------------------------------------
# THE WATER VOLUME — pure Python: Beer-Lambert optics and the clipped
# top/bottom footprint mesh. No `pxr` anywhere below until the next banner;
# `build_inundation` is the only caller that turns this into prims.
# ---------------------------------------------------------------------------
#
# See the module docstring's "THE WATER VOLUME" section for the design
# argument (why geometry alone does not buy the depth-graded translucency,
# why OmniGlass/OmniSurface were investigated and not used, why this ends up
# as pre-authored banded materials instead of a texture or true light
# transport). This section is the arithmetic and the geometry construction
# that design calls for.

# Beer-Lambert: T(d) = exp(-sigma_a * d), so alpha(d) = 1 - T(d). S3.7's
# number ("opaque past 30-50 cm", `d_opaque_m`) is a statement about an
# EXTINCTION LENGTH, not directly a coefficient — converting it needs one
# more choice: how much transmittance survives at that depth before a human
# calls it "opaque". Photographic/rendering convention treats ~3 optical
# depths (attenuation lengths) as visually opaque: exp(-3) ~= 4.98%, under
# 5% of incident light surviving, indistinguishable from fully opaque at
# normal viewing contrast. So:
#
#     T(d_opaque) = exp(-sigma_a * d_opaque) = exp(-3)
#     =>  sigma_a = 3.0 / d_opaque_m
#
# At the DEFAULT `d_opaque_m` = 0.35 m: sigma_a = 3.0 / 0.35 = 8.571 /m.
# Sanity read at a few depths with that sigma_a (`alpha(d) = 1-exp(-sigma_a*d)`):
#     depth 0.02 m -> alpha 0.157   (a wet road film)
#     depth 0.05 m -> alpha 0.349
#     depth 0.10 m -> alpha 0.576
#     depth 0.35 m -> alpha 0.950   (the design point: "opaque")
#     depth 1.00 m -> alpha 0.9998  (indistinguishable from fully opaque)
# This is a CONTINUOUS, physically-derived curve — not the old `alpha_map`'s
# ad hoc `clamp(depth/d_opaque)**alpha_gamma`, which is kept only for the
# legacy flat quad and `pond_specs`' suppression gate (see those call sites).
_OPAQUE_OPTICAL_DEPTHS = 3.0


def _sigma_a_from_d_opaque(d_opaque_m):
    """The arithmetic above, as one line: `_OPAQUE_OPTICAL_DEPTHS /
    d_opaque_m`, guarded against a zero/negative `d_opaque_m`."""
    return _OPAQUE_OPTICAL_DEPTHS / max(1e-6, float(d_opaque_m))


def water_absorption_per_m(cfg):
    """Beer-Lambert absorption coefficient (1/m) for the water VOLUME's
    material, derived from `cfg["d_opaque_m"]` — see the comment above
    `_OPAQUE_OPTICAL_DEPTHS`. Pure Python; `build_inundation` calls this to
    decide which pre-authored opacity band each triangle gets."""
    kn = resolve_cfg(cfg)
    return _sigma_a_from_d_opaque(kn["d_opaque_m"])


def beer_lambert_alpha(depth_m, sigma_a):
    """`1 - exp(-sigma_a * depth)`, i.e. the fraction of light ABSORBED
    (opacity), clamped to a non-negative depth so a dry/negative signed
    depth reads as fully transparent rather than raising or going negative.
    """
    d = max(0.0, float(depth_m))
    return 1.0 - math.exp(-float(sigma_a) * d)


# Opacity bands the continuous Beer-Lambert curve above is quantised into —
# see the module docstring's "1. True volumetric transmission... 2. Fake it
# analytically" for why this is a set of pre-authored MATERIALS
# (`water_materials`'s `"inundation_bands"`) rather than a texture. 16, not
# `ground.build_overlay`'s own 14 (the same idea, applied there to the burn
# scar): banding is UNIFORM IN ALPHA, not in depth, and alpha(d) is
# compressive (`d_opaque_m`'s worth of depth already covers 95% of the 0..1
# range — see the sanity read above), so almost every band boundary falls
# within roughly the first metre of depth, which is also where the mesh
# itself has the most triangle rows (the shoreline slope is gentlest there
# by definition — see `_ground_z_point`). Past that point the deep interior,
# however much deeper it gets, saturates into the last one or two bands,
# which is correct: there is no visible difference between 99% and 99.99%
# opaque. `SURGE_OPACITY_BANDS` overrides it, for the same reason every
# other density/resolution knob in this file is overridable — `water_
# materials` takes no `cfg` (see its own docstring), so this is read from
# the environment directly rather than threaded through `resolve_cfg`.
_WATER_OPACITY_BANDS = 16


def _n_opacity_bands():
    raw = os.environ.get("SURGE_OPACITY_BANDS")
    try:
        n = int(raw) if raw is not None and raw.strip() != "" else \
            _WATER_OPACITY_BANDS
    except ValueError:
        n = _WATER_OPACITY_BANDS
    return max(1, n)


def _opacity_band_index(alpha, n_bands):
    """Which of `n_bands` equal-width bins over `[0, 1]` `alpha` falls in,
    clamped so `alpha == 1.0` lands in the last band rather than overflowing
    it (`int(1.0 * n_bands) == n_bands`, one past the end)."""
    a = max(0.0, min(1.0, float(alpha)))
    return min(int(n_bands) - 1, int(a * int(n_bands)))


def _clip_wet_triangle(ordered, node_key_fn, edge_key_fn, vtx_fn):
    """Marching-triangles: clip one triangle to the portion where its
    per-vertex signed depth is `> 0`, returning 0, 1 or 2 output triangles
    as VERTEX INDICES from `vtx_fn`.

    `ordered` is `[(pA, kA), (pB, kB), (pC, kC)]` in the triangle's own
    winding order, each `pX = (x, y, signed_depth)` and `kX` an opaque,
    HASHABLE identity for that grid node (its `(i, j)` pair) — used only to
    build a canonical edge key (`edge_key_fn`) so that a crossing point on an
    edge shared by two adjacent grid cells is computed and DEDUPLICATED
    identically regardless of which cell asks for it first, which is what
    makes the output mesh watertight with no rounding-tolerance heuristic
    (see `water_volume_mesh`'s docstring, "no cracks").

    All three "how many corners are wet" cases preserve the input winding:
    interpolating linearly IN from a vertex (the `n==1` case) or straight
    ACROSS between two edges sharing the dry vertex (the `n==2` case) both
    keep the same orientation sign as the original triangle — verified
    empirically in this module's own verification pass (a 0/4426 count of
    reversed-winding output triangles on the reference 500x500 m config).
    """
    pos = [i for i, (p, _k) in enumerate(ordered) if p[2] > 0.0]
    n = len(pos)
    if n == 0:
        return []
    if n == 3:
        idxs = [vtx_fn(node_key_fn(*k), p[0], p[1], p[2]) for p, k in ordered]
        return [tuple(idxs)]

    def cross(p_wet, k_wet, p_other, k_other):
        dw, do = p_wet[2], p_other[2]
        t = dw / (dw - do)
        x = p_wet[0] + t * (p_other[0] - p_wet[0])
        y = p_wet[1] + t * (p_other[1] - p_wet[1])
        # Exactly 0.0, not a recomputed/re-sampled depth: this point IS the
        # zero crossing by construction, and this is also what makes the
        # water volume's top and bottom coincide exactly at the shoreline
        # (see `build_inundation`) rather than by a floating-point epsilon.
        return vtx_fn(edge_key_fn(k_wet, k_other), x, y, 0.0)

    if n == 1:
        k0 = pos[0]
        (pA, kA), (pB, kB), (pC, kC) = (ordered[k0], ordered[(k0 + 1) % 3],
                                       ordered[(k0 + 2) % 3])
        ia = vtx_fn(node_key_fn(*kA), pA[0], pA[1], pA[2])
        i_ab = cross(pA, kA, pB, kB)
        i_ac = cross(pA, kA, pC, kC)
        return [(ia, i_ab, i_ac)]

    # n == 2: the one vertex NOT in `pos` is the dry one.
    dry = ({0, 1, 2} - set(pos)).pop()
    (pC, kC), (pA, kA), (pB, kB) = (ordered[dry], ordered[(dry + 1) % 3],
                                    ordered[(dry + 2) % 3])
    ia = vtx_fn(node_key_fn(*kA), pA[0], pA[1], pA[2])
    ib = vtx_fn(node_key_fn(*kB), pB[0], pB[1], pB[2])
    i_bc = cross(pB, kB, pC, kC)
    i_ac = cross(pA, kA, pC, kC)
    return [(ia, ib, i_bc), (ia, i_bc, i_ac)]


def water_volume_mesh(cfg, region, rng, cell_m=None):
    """The wetted FOOTPRINT of the water volume: a 2D triangulation covering
    exactly `depth_at > 0`, clipped watertight at the shoreline.

    Returns `{"points": [(x, y), ...], "depths": [d, ...], "triangles":
    [(ia, ib, ic), ...], "cell_m", "nx", "ny", "area_m2", "wetted_frac"}`.
    `depths[i]` is the UNCLAMPED-then-floored signed depth at `points[i]`
    (`>= 0` everywhere by construction, `== 0.0` exactly at every shoreline
    vertex) — `build_inundation` turns each footprint point into a TOP point
    at `water_level(cfg)` and a BOTTOM point at `water_level(cfg) -
    depths[i]` (`== ground_z` there, but computed from the value this
    function already has rather than a second `_ground_z_point` call), so
    top and bottom coincide exactly at the boundary and the volume closes
    shut with no separate "skirt" wall needed — see that function.

    THE GRID. A regular `(nx+1) x (ny+1)` node lattice at `cell_m` spacing
    (`cfg["water_volume_cell_m"]`, DEFAULT 8.0 m — see that DEFAULTS entry
    for the derivation from `relief_wavelength_m`), each cell split into two
    triangles on a FIXED diagonal (`00-10-11`, `00-11-01`). `_ground_z_array`
    gives every node's UNCLAMPED signed depth (`water_level - ground_z`) in
    one vectorised pass; the per-cell clipping loop itself is plain Python —
    this file's other scatter functions (`pond_specs`, `wrack_specs`) are
    already plain-Python loops over a similar cell count, and the clip
    itself (`_clip_wet_triangle`) is too small a per-cell cost to be worth
    vectorising at the resolutions this module runs at (~4,400 triangles at
    the DEFAULT `cell_m` on the 500x500 m verification region).

    WHY NO CRACKS. Every crossing vertex is deduplicated through
    `_clip_wet_triangle`'s `edge_key_fn` — a canonical `(min(node), max(node))`
    pair — so the SAME physical grid edge, approached from either of the two
    cells that share it, always resolves to the identical output vertex: no
    rounding tolerance, no near-duplicate seam. Verified in this module's own
    check: every INTERIOR edge of the output triangulation is shared by
    EXACTLY two triangles and every BOUNDARY edge (the shoreline, plus
    wherever the wetted region reaches the plate's own edge) by exactly one
    — the signature of a manifold 2D mesh with a well-formed boundary, not a
    hole or a crack.

    WHY THIS MATCHES `coverage()`. Both integrate the SAME signed-depth
    field, just on different lattices (this function's regular grid vs.
    `coverage`'s independent `n x n` cell-centre sample) — since the
    terrain has no content below `relief_wavelength_m`'s 60 m floor, the two
    should and do agree to a fraction of a percent regardless of `cell_m`
    (verified: 0.5566 here vs. `coverage`'s 0.5562 at the reference config,
    a 0.07% relative difference, well inside "a few percent" and NOT the
    kind of mismatch this module's own review process would let stand
    quietly — see the module docstring's `depth_at`/`ground_z` discussion
    for what a REAL mismatch there would mean).
    """
    kn = resolve_cfg(cfg)
    x0, y0, x1, y1 = _bounds(region)
    if cell_m is None:
        cell_m = kn.get("water_volume_cell_m", DEFAULTS["water_volume_cell_m"])
    cell_m = max(1.0, float(cell_m))
    nx = max(1, int(round((x1 - x0) / cell_m)))
    ny = max(1, int(round((y1 - y0) / cell_m)))
    xs = np.linspace(x0, x1, nx + 1)
    ys = np.linspace(y0, y1, ny + 1)
    xg, yg = np.meshgrid(xs, ys)          # shape (ny+1, nx+1)
    gz = _ground_z_array(kn, xg, yg)
    wl = water_level(kn)
    sd = wl - gz                          # UNCLAMPED signed depth, node grid

    points, depths, triangles = [], [], []
    vindex = {}

    def node_val(i, j):
        return (float(xs[i]), float(ys[j]), float(sd[j, i]))

    def node_key(i, j):
        return ("n", i, j)

    def edge_key(na, nb):
        return ("e", na, nb) if na < nb else ("e", nb, na)

    def vtx(key, x, y, d):
        idx = vindex.get(key)
        if idx is None:
            idx = len(points)
            vindex[key] = idx
            points.append((x, y))
            depths.append(max(0.0, d))
        return idx

    for j in range(ny):
        for i in range(nx):
            v00, v10 = node_val(i, j), node_val(i + 1, j)
            v11, v01 = node_val(i + 1, j + 1), node_val(i, j + 1)
            k00, k10 = (i, j), (i + 1, j)
            k11, k01 = (i + 1, j + 1), (i, j + 1)
            triangles.extend(_clip_wet_triangle(
                [(v00, k00), (v10, k10), (v11, k11)],
                node_key, edge_key, vtx))
            triangles.extend(_clip_wet_triangle(
                [(v00, k00), (v11, k11), (v01, k01)],
                node_key, edge_key, vtx))

    area = 0.0
    for ia, ib, ic in triangles:
        (ax, ay), (bx, by), (cx, cy) = points[ia], points[ib], points[ic]
        area += 0.5 * abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay))
    region_area = (x1 - x0) * (y1 - y0)

    return {
        "points": points, "depths": depths, "triangles": triangles,
        "cell_m": cell_m, "nx": nx, "ny": ny,
        "area_m2": area,
        "wetted_frac": (area / region_area) if region_area > 0 else 0.0,
    }


# ---------------------------------------------------------------------------
# host-side texture generation — PIL/numpy only, no `pxr`. Called from
# `build_inundation`, kept out of the "pxr ONLY inside these four" functions
# because they do not touch pxr at all.
# ---------------------------------------------------------------------------

def _write_alpha_png(cfg, region, rng, out_dir=None):
    """Cache-and-write `alpha_map` as a greyscale PNG. Returns a local path.

    Same caching idiom as `scorch.burn_mask_map`/`scorch.scorched_texture`:
    an md5 of the recipe as the filename, so retuning a knob without
    bumping the key does not silently keep serving the old map. Flipped
    vertically on write: `alpha_map` documents row 0 as the region's low-Y
    edge, and PIL's row 0 is conventionally the image TOP (+Y in most
    texture tools), which is the opposite sense.
    """
    from PIL import Image

    out_dir = out_dir or OUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    px = max(2, int(cfg.get("alpha_px", DEFAULTS["alpha_px"])))
    bounds = _bounds(region)
    key = hashlib.md5(
        "alpha|{0:.2f},{1:.2f},{2:.2f},{3:.2f}|{4}|{5:.4f}|{6:.4f}|{7:.4f}"
        "|{8:.4f}|{9:.4f}|{10:.4f}|{11}".format(
            bounds[0], bounds[1], bounds[2], bounds[3], px,
            float(cfg.get("seed", 0)), float(cfg["shore_bearing_deg"]),
            float(cfg["shore_offset_m"]), float(cfg["slope_pct"]),
            float(cfg["d_opaque_m"]), float(cfg["alpha_gamma"]),
            float(cfg["surge_m"]),
        ).encode("utf-8")).hexdigest()[:16]
    path = os.path.join(out_dir, "surge_alpha_{0}.png".format(key))
    if os.path.exists(path):
        return path
    arr = np.asarray(alpha_map(cfg, region, rng, n=px), dtype=np.float64)
    arr = arr[::-1, :]     # row 0 (low-Y) -> image bottom
    Image.fromarray((np.clip(arr, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8),
                    "L").save(path)
    return path


def _normal_from_height(h, strength=6.0):
    """Tangent-space normal map from a height field. Same five-line
    construction as `tools/facade_maps.normal_from_height`, copied rather
    than imported: `disaster/` does not depend on `tools/` anywhere in this
    repo and this is not enough code to justify being the first."""
    gy, gx = np.gradient(h * strength)
    n = np.dstack([-gx, -gy, np.ones_like(h)])
    n = n / np.linalg.norm(n, axis=2, keepdims=True)
    return n * 0.5 + 0.5


def _write_ripple_normal_png(ripple_m, chop, out_dir=None):
    """Cache-and-write a tileable ripple normal map. Returns a local path.

    Generic and seeded independently of any scene (`np.random.default_rng(1)`
    fixed) because the pattern is meant to tile at `ripple_m` and carries no
    scene-specific information — unlike the terrain relief, there is no
    consistency requirement pulling this toward `cfg["seed"]`.
    """
    from PIL import Image

    from . import scorch

    out_dir = out_dir or OUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    px = 512
    key = hashlib.md5("ripple|{0:.3f}|{1:.3f}|{2}".format(
        float(ripple_m), float(chop), px).encode("utf-8")).hexdigest()[:16]
    path = os.path.join(out_dir, "surge_ripple_{0}.png".format(key))
    if os.path.exists(path):
        return path
    # BAND-LIMITED, not `scorch._noise`'s raw output. Unbanded, a beta=2.2
    # spectrum is dominated by its LOWEST available frequency — one cycle
    # across the whole 512x512 canvas, i.e. ONE broad hump spanning the
    # entire `ripple_m` tile. Tiled at `clearcoat_texture_scale = 1/ripple_m`
    # that reads as a single slow swell every `ripple_m` — a real-world
    # wavelength of METRES, not the "centimetres to decimetres" a wind-chop
    # ripple actually is — and it is soft enough at that size to barely bump
    # the surface at all, which is what "no sense of a surface" and a
    # "low poly / glossy / plastic" specular sheen both look like on a
    # render (the review's own words). `ground.edge_fields.band()` solves
    # the identical problem — a pattern dominated by the plate's own size —
    # the same way: convert a wavelength range in real units to
    # cycles-per-pixel and mask the spectrum to it. `hi_m` is kept well
    # under `ripple_m` on purpose so the tile-sized term never comes back;
    # `lo_m` stays well above the 2-pixel Nyquist limit for `px`.
    mpp = float(ripple_m) / float(px)             # metres per pixel, THIS tile
    lo_m, hi_m = 0.04, 0.5
    lo, hi = mpp / hi_m, mpp / lo_m
    h = scorch._noise(np.random.default_rng(1), px, px, beta=2.2,
                      lo=lo, hi=hi)
    # Chop scales bump strength (S3.0 "Ripples": 0.3 sheltered .. 1.4 open
    # water) -- HEIGHT AND NORMAL MAPS ARE DATA, NOT COLOUR
    # (`tools/facade_maps.py:148-151`), written/read linear here throughout.
    n = _normal_from_height(h, strength=4.0 + 10.0 * max(0.0, float(chop)))
    Image.fromarray((np.clip(n, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8),
                    "RGB").save(path)
    return path


# ---------------------------------------------------------------------------
# `pxr` ONLY BELOW THIS LINE
# ---------------------------------------------------------------------------

# Body colour, LINEAR RGB. `_plans/hurricane_water.md` S3.7 exactly.
_PALETTE = {
    # mineral load 300-1300 FNU: Harvey, Ian, Katrina (S2.2 USGS turbidity)
    "sediment": (0.155, 0.115, 0.070),
    "sediment_light": (0.115, 0.090, 0.062),
    # CDOM/tannin -- Carolina coastal plain, Gulf swamp
    "blackwater": (0.035, 0.032, 0.024),
    # clean-sand/carbonate coast; still OPAQUE, grey-green -- "there is
    # deliberately no blue option" (S3.7)
    "carbonate": (0.105, 0.115, 0.098),
}


def water_materials(stage, parent_path, suffix=""):
    """The `OmniPBR_ClearCoat` water looks under `<parent_path>/WaterLooks`.

    Returns `{"inundation": mat, "inundation_bands": [mat, ...],
    "pond_unpaved": mat, "pond_paved": mat, "pond_unpaved_rim": mat,
    "pond_paved_rim": mat}`.

    `"inundation"` (kept for the LEGACY flat quad, `SURGE_FLAT_WATER=1` —
    see `_build_inundation_flat`) is left WITHOUT `opacity_constant`/
    `opacity_texture` set — that legacy path adds those once it knows the
    region-specific alpha map and world-projection scale, exactly as
    `diffuse_texture` replaces `diffuse_color_constant` rather than
    multiplying it (`damage._pbr`'s documented behaviour) -- see that
    function.

    `"inundation_bands"` is what the DEFAULT `build_inundation` actually
    uses: `_n_opacity_bands()` materials, index 0 the most transparent and
    the last the most opaque, each identical except a FIXED representative
    `opacity_constant` at its band's midpoint (`(i + 0.5) / n_bands`) — see
    the module docstring's "THE WATER VOLUME" for why this is a set of
    pre-authored materials rather than a texture, and `build_inundation` for
    how a triangle's Beer-Lambert alpha picks which one of these it binds
    to. Unlike the legacy `"inundation"`, these carry `WATER_DIFFUSE_TEXTURE`
    (the Swamp_Water surface-variation map) as their diffuse, tinted by the
    same sediment `rgb` — safe to tile at its own small `texture_scale` now
    that opacity does not also need that slot (module docstring, "WHY THE
    SWAMP-WATER TEXTURE NEEDED THE OPACITY OFF THE BASE TEXTURE SLOT").

    The `_rim` pair is a fainter copy of the matching pond look, for
    `build_ponding`'s soft-edge ring — see that function for why a pond
    needs one and the water volume does not (a real mesh boundary needs no
    feathering to hide a raster edge that no longer exists).

    No `cfg` parameter (fixed by the task's API contract), so the palette,
    chop and band-count knobs are read from `SURGE_*` environment variables
    here, falling back to `DEFAULTS`/`_WATER_OPACITY_BANDS` -- the same
    values `resolve_cfg`/`knobs_from_env`/`_n_opacity_bands` would produce
    for an unconfigured scene, so a launcher that drives both this function
    and `resolve_cfg` from the same environment gets a consistent result.
    """
    from pxr import Gf, Sdf, UsdShade

    import scene_generator as sg

    def _env_f(name, key):
        raw = os.environ.get("SURGE_" + name)
        return float(DEFAULTS[key]) if raw is None or raw.strip() == "" \
            else float(raw)

    palette_name = os.environ.get("SURGE_PALETTE", DEFAULTS["palette"])
    turbidity = max(0.0, min(1.0, _env_f("TURBIDITY", "turbidity")))
    chop = _env_f("CHOP", "chop")
    ripple_m = _env_f("RIPPLE_M", "ripple_m")

    if palette_name == "sediment":
        lo, hi = _PALETTE["sediment_light"], _PALETTE["sediment"]
        rgb = tuple(lo[i] + (hi[i] - lo[i]) * turbidity for i in range(3))
    else:
        rgb = _PALETTE.get(palette_name, _PALETTE["sediment"])

    ripple_png = None
    try:
        ripple_png = _write_ripple_normal_png(ripple_m, chop)
    except Exception as exc:
        print("[surge] ripple normal map unavailable ({0})".format(exc))

    def _make(name, ccr, bump, opacity_const=None, diffuse_tex=None,
             normal_tex=None, orm_tex=None, tex_scale=(1.0, 1.0)):
        path = "{0}/WaterLooks/{1}{2}".format(parent_path, name, suffix)
        mat = UsdShade.Material.Define(stage, Sdf.Path(path))
        sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
        # `info:id` MUST NAME THE SAME SHADER as the source asset and the
        # sub-identifier. It said "OmniPBR" here while the other two said
        # "OmniPBR_ClearCoat" — every other shader author in this repo keeps
        # the triple consistent (`ground.py:112-114`), and a mismatched id is
        # a documented way to have the coat inputs quietly ignored, which is
        # exactly what the first render looked like: opaque tan, no Fresnel,
        # no sky in it.
        sh.CreateIdAttr("OmniPBR_ClearCoat")
        sh.SetSourceAsset(Sdf.AssetPath("OmniPBR_ClearCoat.mdl"), "mdl")
        sh.SetSourceAssetSubIdentifier("OmniPBR_ClearCoat", "mdl")
        # BASE = the sediment volume, approximated as a diffuse body (S3.7)
        sh.CreateInput("diffuse_color_constant",
                      Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        if diffuse_tex:
            # Ponds (`SILT_TEXTURE`) and the water-volume bands
            # (`WATER_DIFFUSE_TEXTURE`) both want this — never the LEGACY
            # `"inundation"` quad (S2.2: nearly uniform past 30-50 cm, and it
            # has no free `texture_scale` slot -- module docstring, "THE
            # SHORELINE USED TO COME FROM AN ALPHA MAP").
            sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
                Sdf.AssetPath(sg._join_asset_root(diffuse_tex, "")))
            # `diffuse_texture` REPLACES `diffuse_color_constant`
            # (`damage.py`'s documented OmniPBR behaviour) -- `diffuse_tint`
            # is the multiply that still lets `rgb` (palette/turbidity)
            # steer the look.
            sh.CreateInput("diffuse_tint",
                          Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            if normal_tex:
                # Ponds want their own bump (small, close-up, no independent
                # ripple system of their own). The water volume's bands pass
                # `normal_tex=None` on purpose: `WATER_DIFFUSE_TEXTURE`'s own
                # `_N` sibling is NOT band-limited to `ripple_m`, and the
                # `clearcoat_normalmap_texture` ripple below already is —
                # see `WATER_DIFFUSE_TEXTURE`'s own docstring.
                sh.CreateInput("normalmap_texture",
                              Sdf.ValueTypeNames.Asset).Set(
                    Sdf.AssetPath(sg._join_asset_root(normal_tex, "")))
            if orm_tex:
                sh.CreateInput("enable_ORM_texture",
                              Sdf.ValueTypeNames.Bool).Set(True)
                sh.CreateInput("ORM_texture", Sdf.ValueTypeNames.Asset).Set(
                    Sdf.AssetPath(sg._join_asset_root(orm_tex, "")))
            sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
            sh.CreateInput("world_or_object",
                          Sdf.ValueTypeNames.Bool).Set(True)
            sh.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(
                Gf.Vec2f(*tex_scale))
        sh.CreateInput("reflection_roughness_constant",
                      Sdf.ValueTypeNames.Float).Set(0.90)
        # Pinned to 0.0 REGARDLESS of the ORM texture above:
        # `_plans/hurricane_water.md` risk #7 (unverified, but cheap to
        # guard against) is that an ORM's roughness channel can silently
        # override a constant once a texture is bound -- keeping this at
        # 0.0 is what makes `reflection_roughness_constant` above
        # authoritative either way.
        sh.CreateInput("reflection_roughness_texture_influence",
                      Sdf.ValueTypeNames.Float).Set(0.0)
        sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
        # F0 = 0.08 * 0.05 = 0.004; the COAT owns the Fresnel, not the base.
        sh.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.05)
        # COAT = the air-water interface (S3.7)
        sh.CreateInput("enable_clearcoat", Sdf.ValueTypeNames.Bool).Set(True)
        sh.CreateInput("clearcoat_ior", Sdf.ValueTypeNames.Float).Set(1.333)
        sh.CreateInput("clearcoat_weight", Sdf.ValueTypeNames.Float).Set(1.0)
        sh.CreateInput("clearcoat_tint",
                      Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0))
        sh.CreateInput("clearcoat_transparency",
                      Sdf.ValueTypeNames.Float).Set(1.0)
        sh.CreateInput("clearcoat_reflection_roughness",
                      Sdf.ValueTypeNames.Float).Set(float(ccr))
        sh.CreateInput("clearcoat_flatten", Sdf.ValueTypeNames.Float).Set(1.0)
        sh.CreateInput("clearcoat_bump_factor",
                      Sdf.ValueTypeNames.Float).Set(float(bump))
        if ripple_png:
            sh.CreateInput("clearcoat_normalmap_texture",
                          Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(ripple_png))
            k = 1.0 / max(0.1, float(ripple_m))
            # INDEPENDENT of the body's own `texture_scale` (S1.8) -- this is
            # the escape from OmniPBR's one-`texture_scale` constraint that
            # forced `ground.build_overlay` into bands.
            sh.CreateInput("clearcoat_texture_scale",
                          Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(k, k))
        sh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
        sh.CreateInput("opacity_threshold", Sdf.ValueTypeNames.Float).Set(0.0)
        if opacity_const is not None:
            sh.CreateInput("opacity_constant",
                          Sdf.ValueTypeNames.Float).Set(float(opacity_const))
        mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
        mat.CreateDisplacementOutput("mdl").ConnectToSource(
            sh.ConnectableAPI(), "out")
        mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
        return mat

    n_bands = _n_opacity_bands()
    bump = 0.3 + 1.1 * min(1.0, chop / 0.24)
    bands = [
        _make("Body_band{0:02d}".format(i), chop, bump,
             opacity_const=(i + 0.5) / n_bands,
             diffuse_tex=WATER_DIFFUSE_TEXTURE, normal_tex=None,
             orm_tex=WATER_ORM_TEXTURE,
             tex_scale=(_WATER_DIFFUSE_REPEATS_PER_M,) * 2)
        for i in range(n_bands)
    ]

    return {
        # LEGACY (`SURGE_FLAT_WATER=1`) only -- see this function's own
        # docstring and `_build_inundation_flat`.
        "inundation": _make("Body", chop, bump),
        # DEFAULT `build_inundation` path -- see this function's docstring.
        "inundation_bands": bands,
        # L2 ponds: the opposite of L1 -- a smooth thin mirror rather than a
        # rough opaque body (S3.0 L2). Grass holds a flatter sheet than
        # asphalt, hence the lower roughness (S3.0 L2 "Material"). Textured
        # (see `_make`'s `diffuse_tex` branch) rather than a flat constant --
        # the reviewed "flat untextured tan polygon" complaint was ponds,
        # named specifically.
        "pond_unpaved": _make("PondGrass", 0.03, 0.15, opacity_const=0.70,
                              diffuse_tex=SILT_TEXTURE,
                              normal_tex=SILT_NORMAL_TEXTURE,
                              orm_tex=SILT_ORM_TEXTURE),
        "pond_paved": _make("PondPaved", 0.05, 0.15, opacity_const=0.65,
                            diffuse_tex=SILT_TEXTURE,
                            normal_tex=SILT_NORMAL_TEXTURE,
                            orm_tex=SILT_ORM_TEXTURE),
        # RIM variants for `build_ponding`'s soft edge: same look, roughly
        # HALF the core's opacity, so a pond's outline fades into the
        # grass/asphalt over two steps instead of one hard cliff --
        # `ground.build_overlay`'s own banded-opacity fix for an identical
        # problem (a translucent cutout has a visible edge), here applied to
        # a pond's outline instead of a scar's. 0.45x, not 0.5x, so the rim
        # reads as visibly fainter rather than a second near-equal step.
        "pond_unpaved_rim": _make("PondGrassRim", 0.03, 0.15,
                                  opacity_const=0.70 * 0.45,
                                  diffuse_tex=SILT_TEXTURE,
                                  normal_tex=SILT_NORMAL_TEXTURE,
                                  orm_tex=SILT_ORM_TEXTURE),
        "pond_paved_rim": _make("PondPavedRim", 0.05, 0.15,
                                opacity_const=0.65 * 0.45,
                                diffuse_tex=SILT_TEXTURE,
                                normal_tex=SILT_NORMAL_TEXTURE,
                                orm_tex=SILT_ORM_TEXTURE),
    }


def _use_legacy_flat_water():
    """`SURGE_FLAT_WATER=1` reverts `build_inundation` to the pre-rework
    single flat quad + alpha-map cutout (`_build_inundation_flat`) -- a
    cheap escape hatch for a quick bench iteration, or to rule the volume
    out while debugging something else downstream of it. Off by default:
    the volume is the entire point of this rework (module docstring, "THE
    WATER VOLUME")."""
    return os.environ.get("SURGE_FLAT_WATER", "0").strip().lower() in (
        "1", "true", "yes")


def build_inundation(stage, parent_path, cfg, region, rng, *, ssf=1.0,
                     materials=None, alpha_png=None):
    """L1, the water surface. DEFAULT: a real closed VOLUME
    (`_build_inundation_volume`) -- top flat at `water_level(cfg)`, bottom
    following `ground_z`, tessellated only where wet. Behind
    `SURGE_FLAT_WATER=1`: the original single flat quad with an alpha-map
    cutout (`_build_inundation_flat`). See the module docstring's "THE WATER
    VOLUME" for the full argument; this function is just the dispatch.

    `ssf`/`materials` mean the same thing on both paths — see either
    private function's own docstring. `alpha_png` is consulted ONLY by the
    legacy path (the volume path does not rasterise an alpha map at all).

    Returns `[water_body_prim_path]`, or `[]` if the config is entirely dry
    (both paths).
    """
    if _use_legacy_flat_water():
        return _build_inundation_flat(stage, parent_path, cfg, region, rng,
                                      ssf=ssf, materials=materials,
                                      alpha_png=alpha_png)
    return _build_inundation_volume(stage, parent_path, cfg, region, rng,
                                    ssf=ssf, materials=materials)


def _build_inundation_volume(stage, parent_path, cfg, region, rng, *,
                             ssf=1.0, materials=None):
    """The default L1 path: ONE closed-volume mesh from `water_volume_mesh`,
    doubled into a TOP layer (flat at `water_level`) and a BOTTOM layer
    (`water_level - depth`, i.e. `ground_z`) sharing the SAME footprint
    indices offset by `N = len(mesh["points"])` — see `water_volume_mesh`'s
    docstring for why top and bottom coincide exactly at the shoreline with
    no separate skirt geometry needed to close the volume.

    OPACITY is per-triangle, not per-pixel: each TOP face's Beer-Lambert
    alpha (`beer_lambert_alpha`, from that triangle's mean vertex depth) is
    bucketed (`_opacity_band_index`) into one of `materials["inundation_
    bands"]`'s pre-authored materials and bound via a `materialBind`
    `UsdGeom.Subset` -- see the module docstring's "THE WATER VOLUME" for
    why a texture is not used here. Every BOTTOM face is bound to a single
    submerged-mud look (`water_mud`, built fresh via `_dry_material` unless
    `materials` already carries one) -- see the module docstring's "ground_z
    DOES NOT DISPLACE..." section for why this is the one place in the
    module that IS allowed to sit at literal `ground_z`.

    `ssf`/`materials` — see `build_inundation`. Returns `[]` (no mesh
    authored at all) if the config is entirely dry.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    kn = resolve_cfg(cfg)
    ssf = float(ssf)
    wl = water_level(kn)
    # SANITY, PRINTED, NOT ASSERTED -- see `_build_inundation_flat`'s
    # identical check; unchanged by the volume rework.
    if wl > 15.0:
        print("[surge] WARNING water_level={0:.2f} m is above any recorded "
             "storm surge -- check `surge_m` for a units bug before trusting "
             "this render".format(wl))

    mesh = water_volume_mesh(kn, region, rng)
    tris = mesh["triangles"]
    if not tris:
        print("[surge] water_body(volume): 0 wet triangles -- config is "
             "entirely dry, no mesh authored (coverage={0:.4f})".format(
                 coverage(kn, region, rng)))
        return []

    UsdGeom.Scope.Define(stage, Sdf.Path(parent_path))
    mats = materials if materials is not None else water_materials(
        stage, parent_path)
    bands = mats.get("inundation_bands") or [mats["inundation"]]
    n_bands = len(bands)
    mud_mat = (materials or {}).get("water_mud") or _dry_material(
        stage, "{0}/DepositLooks/water_mud".format(parent_path),
        rgb=(0.16, 0.13, 0.09), rough=0.95, scale=(0.5, 0.5), desat=0.30,
        texture=SILT_TEXTURE, normal=SILT_NORMAL_TEXTURE,
        orm=SILT_ORM_TEXTURE)

    points, depths = mesh["points"], mesh["depths"]
    n_verts = len(points)
    top_z = wl * ssf
    sigma_a = water_absorption_per_m(kn)

    pts3 = [None] * (2 * n_verts)
    normals = [None] * (2 * n_verts)
    for i, (x, y) in enumerate(points):
        bottom_z = (wl - depths[i]) * ssf   # == ground_z(x,y)*ssf; see
                                            # `water_volume_mesh`'s docstring
        pts3[i] = Gf.Vec3f(x * ssf, y * ssf, top_z)
        pts3[n_verts + i] = Gf.Vec3f(x * ssf, y * ssf, bottom_z)
        normals[i] = Gf.Vec3f(0.0, 0.0, 1.0)
        normals[n_verts + i] = Gf.Vec3f(0.0, 0.0, -1.0)

    counts, indices = [], []
    band_faces = [[] for _ in range(n_bands)]
    for face_i, (ia, ib, ic) in enumerate(tris):
        d_mean = (depths[ia] + depths[ib] + depths[ic]) / 3.0
        alpha = beer_lambert_alpha(d_mean, sigma_a)
        band_faces[_opacity_band_index(alpha, n_bands)].append(face_i)
        counts.append(3)
        indices += [ia, ib, ic]
    n_top_faces = len(tris)
    mud_faces = []
    for face_i, (ia, ib, ic) in enumerate(tris):
        # Same footprint triangle, BOTTOM-layer indices, winding REVERSED
        # (b/c swapped) so its normal faces down into the mud rather than
        # up into the water it bounds.
        counts.append(3)
        indices += [n_verts + ia, n_verts + ic, n_verts + ib]
        mud_faces.append(n_top_faces + face_i)

    path = "{0}/water_body".format(parent_path)
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray(pts3))
    m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    m.CreateNormalsAttr(Vt.Vec3fArray(normals))
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr(True)
    xs_all = [p[0] for p in pts3]
    ys_all = [p[1] for p in pts3]
    zs_all = [p[2] for p in pts3]
    m.CreateExtentAttr([Gf.Vec3f(min(xs_all), min(ys_all), min(zs_all)),
                        Gf.Vec3f(max(xs_all), max(ys_all), max(zs_all))])
    m.CreateDisplayColorAttr([Gf.Vec3f(0.155, 0.115, 0.070)])
    # NO PhysicsCollisionAPI, NO RigidBodyAPI -- see module docstring. A
    # water collider becomes "the ground" for `_make_physx_ground_snap`'s
    # downward raycasts (water plan S3.11 risk #4).
    #
    # Mesh-level fallback binding FIRST (the deepest/most-opaque band — "this
    # is water, unmistakably" is the safer default than the palest one), THEN
    # per-face `materialBind` GeomSubsets on top, which win for their own
    # faces. Same layering `build_ponding`/`gac_fire.build_fire`
    # (`disaster/gac_fire.py:1335-1346`) already rely on; a subset created
    # WITHOUT `UsdShade.Tokens.materialBind`/`UsdGeom.Tokens.partition` is
    # silently ignored by the renderer and the whole mesh falls back to one
    # material (the exact bug `slice-buildings-into-kits`' skill catalogues
    # for a sliced building turning into a flat brown box) — both required
    # here for the same reason.
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(bands[-1])
    for i, faces in enumerate(band_faces):
        if not faces:
            continue
        sub = UsdGeom.Subset.CreateGeomSubset(
            m, "waterBand{0:02d}".format(i), UsdGeom.Tokens.face,
            Vt.IntArray(faces), UsdShade.Tokens.materialBind,
            UsdGeom.Tokens.partition)
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(bands[i])
    if mud_mat is not None and mud_faces:
        sub = UsdGeom.Subset.CreateGeomSubset(
            m, "mudBase", UsdGeom.Tokens.face, Vt.IntArray(mud_faces),
            UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mud_mat)

    # PRINTED EXTENT AND COVERAGE CROSS-CHECK, always -- this is the one prim
    # in the whole pass that, authored at the wrong scale or height, would
    # fill or blot out a frame rather than merely look wrong in a corner.
    # `wetted_frac` vs `coverage()` is `water_volume_mesh`'s own consistency
    # check, re-surfaced here because THIS is the function whose output a
    # render actually shows.
    cov = coverage(kn, region, rng)
    print("[surge] water_body(volume): {0} pts ({1} top + {1} bottom), "
         "{2} faces ({3} top in {4} band(s) used, {5} mud), "
         "x[{6:.1f},{7:.1f}] y[{8:.1f},{9:.1f}] z[{10:.3f},{11:.3f}] "
         "stage-units (water_level={12:.3f} m, cell_m={13:.2f}, "
         "wetted_frac={14:.4f} vs coverage()={15:.4f}, "
         "sigma_a={16:.3f}/m, ssf={17:.4f})".format(
             len(pts3), n_verts, len(counts), n_top_faces,
             sum(1 for f in band_faces if f), len(mud_faces),
             min(xs_all), max(xs_all), min(ys_all), max(ys_all),
             min(zs_all), max(zs_all), wl, mesh["cell_m"],
             mesh["wetted_frac"], cov, sigma_a, ssf))
    return [path]


def _build_inundation_flat(stage, parent_path, cfg, region, rng, *, ssf=1.0,
                           materials=None, alpha_png=None):
    """LEGACY (`SURGE_FLAT_WATER=1`): ONE quad at `water_level(cfg)`,
    shoreline from an alpha map. Preserved unchanged from before the volume
    rework — see `build_inundation`'s docstring for the dispatch and the
    module docstring's "THE SHORELINE USED TO COME FROM AN ALPHA MAP" for
    why this is no longer the default.

    `ssf` is the stage-scale factor, applied to every authored coordinate —
    the same convention `scour_relief.build`/`ground.build_overlay`/the
    launcher's own `_ref` use. `materials`, if given (a dict as returned by
    `water_materials`), is used AS-IS instead of calling `water_materials`
    again — the launcher builds one shared dict once and passes it to
    `build_inundation` AND `build_ponding` so the looks are not authored
    twice; if `None` this calls `water_materials(stage, parent_path)` itself,
    so the function is still fully self-contained for a bench or a test.
    `alpha_png`, if given, is a pre-baked opacity texture path (e.g. from a
    future `tools/water_maps.py`, per the plan's "what must be built" #1);
    if `None`, this generates and caches one itself via `_write_alpha_png`.

    Geometry mirrors `scene_generator._make_plane_mesh` exactly (4 points,
    vertex normals, `subdivisionScheme "none"`, an explicit 0..1 `st`
    primvar per the water plan's S3.0 L1 geometry spec) even though the
    material world-projects and ignores `st` — cheap to author, and it
    means this mesh looks like every other flat ground plane in the repo to
    anything that inspects `st` rather than guessing from `project_uvw`.

    Returns `[water_body_prim_path]`.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    kn = resolve_cfg(cfg)
    x0, y0, x1, y1 = _bounds(region)
    ssf = float(ssf)
    wl = water_level(kn)
    z = wl * ssf
    span = max(x1 - x0, y1 - y0)
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    # SANITY, PRINTED, NOT ASSERTED -- a scene that already has a genuine
    # scale bug elsewhere should still finish and say so loudly, not crash
    # here. NOAA's own record storm surge is 8.5 m (`_plans/hurricane_water.
    # md` S2.1); `water_level` cannot exceed `surge_m`, so anything above
    # ~15 m through this path is a units slip upstream (cm mistaken for m in
    # a preset, most likely), not a real flood depth.
    if wl > 15.0:
        print("[surge] WARNING water_level={0:.2f} m is above any recorded "
             "storm surge -- check `surge_m` for a units bug before trusting "
             "this render".format(wl))

    UsdGeom.Scope.Define(stage, Sdf.Path(parent_path))
    mats = materials if materials is not None else water_materials(
        stage, parent_path)
    mat = mats["inundation"]
    sh = UsdShade.Shader.Get(stage, mat.GetPath().AppendChild("Shader"))

    if alpha_png is None:
        alpha_png = _write_alpha_png(kn, region, rng)
    # Shoreline: ONE stretched opacity map (S3.0 "The ragged shoreline, on
    # flat ground" -- "one mesh + a stretched cutout alpha map"), using
    # `ground.overlay_material`'s exact world-projection formula (`:121-125`)
    # so the same half-tile-offset lesson applies: scale, then translate.
    sh.CreateInput("enable_opacity_texture", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("opacity_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(alpha_png))
    sh.CreateInput("opacity_mode", Sdf.ValueTypeNames.Int).Set(2)  # mono_luminance
    sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
    k = 1.0 / max(1e-6, span)
    sh.CreateInput("texture_scale",
                  Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(k, k))
    sh.CreateInput("texture_translate", Sdf.ValueTypeNames.Float2).Set(
        Gf.Vec2f(0.5 - cx * k, 0.5 - cy * k))

    path = "{0}/water_body".format(parent_path)
    pts = Vt.Vec3fArray([
        Gf.Vec3f(x0 * ssf, y0 * ssf, z), Gf.Vec3f(x1 * ssf, y0 * ssf, z),
        Gf.Vec3f(x1 * ssf, y1 * ssf, z), Gf.Vec3f(x0 * ssf, y1 * ssf, z),
    ])
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * 4))
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr(True)
    UsdGeom.PrimvarsAPI(m).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
        Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1),
                      Gf.Vec2f(0, 1)]))
    m.CreateExtentAttr([Gf.Vec3f(x0 * ssf, y0 * ssf, z),
                        Gf.Vec3f(x1 * ssf, y1 * ssf, z)])
    m.CreateDisplayColorAttr([Gf.Vec3f(0.155, 0.115, 0.070)])
    # NO PhysicsCollisionAPI, NO RigidBodyAPI -- see module docstring. A
    # water collider becomes "the ground" for `_make_physx_ground_snap`'s
    # downward raycasts (water plan S3.11 risk #4).
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    # PRINTED EXTENT, always -- this is the one prim in the whole pass that,
    # authored at the wrong scale or height, would fill or blot out a frame
    # rather than merely look wrong in a corner. 4 points, so nothing here
    # is expensive to print every run.
    print("[surge] water_body: 4 pts, x[{0:.1f},{1:.1f}] y[{2:.1f},{3:.1f}] "
         "stage-units, z={4:.3f} (water_level={5:.3f} m, region span={6:.1f} "
         "m, ssf={7:.4f})".format(x0 * ssf, x1 * ssf, y0 * ssf, y1 * ssf,
                                  z, wl, span, ssf))
    return [path]


# Vertices per pond outline. `scour_relief._DOME_N` (12) is this repo's own
# precedent for a wobble-outlined shape in the same size class (0.9-3.2 m
# mounds vs this module's 0.6-3.5 m ponds); 8 was BELOW that precedent, and
# the review of the first render called it out by name: "flat untextured tan
# POLYGONS with visibly straight edges... look like cartoon cut-outs" and,
# separately, "low poly". `_wobble_at` is a smooth, continuously-evaluable
# curve (three sine harmonics) — the facets were never in the deformation,
# only in how coarsely it was being sampled. 14 (not 12) because a pond runs
# larger than a mound (3.5 m vs 3.2 m radius) and the review named ponds as
# "the worst offender".
_POND_SIDES = 14
# How much wider the rim is than the core, as a radius multiplier — see
# `build_ponding`.
_POND_RIM_GROW = 1.22


def build_ponding(stage, parent_path, cfg, region, rng, *, ssf=1.0,
                  materials=None):
    """L2: pond discs merged into ONE mesh per surface class (paved /
    unpaved), per `pond_specs`. A pond sits a few millimetres above the
    real flat grade (`_SURFACE_Z_M`) — never at the synthetic `ground_z`
    value, which has no meaning as a real-world height (module docstring).

    Each pond now authors up to THREE meshes, not one — a direct response to
    two review rounds calling ponds "flat untextured tan polygons... cartoon
    cut-outs" and "low poly / animated":

      - the CORE disc, `_POND_SIDES` vertices (not 8) with an independent
        per-vertex jitter on top of the usual smooth wobble, and now bound
        to a textured pond look (`water_materials`) instead of a flat
        constant colour;
      - a RIM ring `_POND_RIM_GROW` wider, at roughly HALF the core's
        opacity, so the outline fades into the grass/asphalt over two steps
        instead of one hard cliff. This is `ground.build_overlay`'s own fix
        for the identical problem — a translucent cutout has a visible
        edge — banded opacity, applied here to a pond's own outline instead
        of a scar's;
      - for UNPAVED ponds only, a MUD BASE in the CORE's own footprint (not
        the rim's — see below), at true grade, directly under the water. A
        translucent pond sitting straight over the suburb's grass sheet
        shows 20-45% bright green through it regardless of how the water
        itself is shaded, which is a real part of why thin standing water
        reads as fake (module docstring: "THE FLOODED GROUND UNDER THE
        WATER SHOULD BE MUD, NOT GRASS"). The mud is kept at the CORE
        radius, not the wider rim, so it never itself becomes a second,
        harder-edged cutout sitting naked against the grass beyond the
        water's own footprint — only the (fainter, fading) rim sits over
        bare grass, same as before. Paved ponds get no mud base: wet
        asphalt is already dark (`build-hurricane-scenes` skill:
        "vegetation does not darken... asphalt's visible change is gloss,
        not albedo").

    All three reuse ONE set of per-vertex angles and radius multipliers per
    pond (computed once, in `_factors`), so the core, rim and mud base are
    flush with each other at every seam and never gap or double-cover.

    `ssf`/`materials` — see `build_inundation`. Returns the list of merged
    mesh prim paths (up to 3 per surface class, depending on which classes
    `pond_specs` actually populated).
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    kn = resolve_cfg(cfg)
    rng = _as_random(rng)
    specs = pond_specs(kn, region, rng)
    if not specs:
        return []
    ssf = float(ssf)
    z = (_SURFACE_Z_M + 0.01) * ssf
    z_mud = _SURFACE_Z_M * ssf

    UsdGeom.Scope.Define(stage, Sdf.Path(parent_path))
    root = "{0}/ponding".format(parent_path)
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    mats = materials if materials is not None else water_materials(
        stage, parent_path)
    # A pond's own soil look, shared by every unpaved pond's mud base —
    # `materials`, if supplied, MAY carry one already (the `build_deposits`
    # "wrack"/"sand" convention); otherwise build it fresh here.
    mud_mat = (materials or {}).get("pond_mud") or _dry_material(
        stage, "{0}/DepositLooks/pond_mud".format(parent_path),
        rgb=(0.20, 0.17, 0.12), rough=0.95, scale=(1.0, 1.0), desat=0.35,
        texture=SILT_TEXTURE, normal=SILT_NORMAL_TEXTURE,
        orm=SILT_ORM_TEXTURE)

    def _factors():
        """`_POND_SIDES` `(angle, radius multiplier)` pairs for one pond's
        outline. The multiplier is the smooth 3-harmonic wobble
        (`scour_relief._wobble_at`) TIMES an independent per-vertex jitter
        — the wobble alone is a smooth curve, and a smooth curve sampled at
        any vertex count still reads as a drawn shape up close; real puddle
        margins are ragged, not smooth. +-7% is enough to break that up
        without moving the mean radius enough to disturb `pond_specs`' own
        footprint accounting. Drawn once per pond and reused at every
        radius (core/rim/mud) so they stay flush at their shared edges.
        """
        wob = scour_relief._wob(rng)
        out = []
        for i in range(_POND_SIDES):
            ang = 2.0 * math.pi * i / _POND_SIDES
            out.append((ang, scour_relief._wobble_at(wob, ang)
                       * rng.uniform(0.93, 1.07)))
        return out

    def _ring_pts(cx, cy, r, factors, z_m):
        return [Gf.Vec3f((cx + r * mult * math.cos(ang)) * ssf,
                         (cy + r * mult * math.sin(ang)) * ssf, z_m)
               for ang, mult in factors]

    def _finish(path, pts, counts, idx, mat, z_m):
        mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * len(pts)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        mesh.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), z_m),
                              Gf.Vec3f(max(xs), max(ys), z_m)])
        mesh.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.33, 0.30)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        return path

    made = []
    for paved_flag, mat_key, rim_key, tag in (
            (False, "pond_unpaved", "pond_unpaved_rim", "grass"),
            (True, "pond_paved", "pond_paved_rim", "paved")):
        group = [s for s in specs if bool(s.get("paved")) == paved_flag]
        if not group:
            continue

        core_pts, core_counts, core_idx = [], [], []
        rim_pts, rim_counts, rim_idx = [], [], []
        mud_pts, mud_counts, mud_idx = [], [], []
        for s in group:
            cx, cy = float(s["x"]), float(s["y"])
            r = float(s["r_m"])
            # SANITY, PRINTED, NOT ASSERTED -- `pond_radius_m` is (0.6, 3.5)
            # m in `DEFAULTS`; anything an order of magnitude past that is a
            # spec bug, not a real puddle, and would author a mesh far
            # larger than a pond has any business being.
            if r > 20.0:
                print("[surge] WARNING pond r_m={0:.2f} m at ({1:.1f}, "
                     "{2:.1f}) is far outside `pond_radius_m` -- check "
                     "`pond_specs` before trusting this mesh".format(
                         r, cx, cy))
            factors = _factors()
            inner = _ring_pts(cx, cy, r, factors, z)
            outer = _ring_pts(cx, cy, r * _POND_RIM_GROW, factors, z)

            base = len(core_pts)
            core_pts.extend(inner)
            center_i = len(core_pts)
            core_pts.append(Gf.Vec3f(cx * ssf, cy * ssf, z))
            for i in range(_POND_SIDES):
                core_counts.append(3)
                core_idx += [center_i, base + i,
                            base + (i + 1) % _POND_SIDES]

            # RIM: a ring of quads between the core's own edge (`inner`) and
            # `outer` — flush with the core by construction, same angles.
            rbase = len(rim_pts)
            rim_pts.extend(inner)
            rim_pts.extend(outer)
            for i in range(_POND_SIDES):
                j = (i + 1) % _POND_SIDES
                rim_counts.append(4)
                rim_idx += [rbase + i, rbase + j,
                           rbase + _POND_SIDES + j, rbase + _POND_SIDES + i]

            if not paved_flag:
                # MUD BASE, in the CORE's footprint (`inner`, not `outer`)
                # — see the docstring for why not the wider rim.
                mbase = len(mud_pts)
                mud_pts.extend(Gf.Vec3f(p[0], p[1], z_mud) for p in inner)
                center_i = len(mud_pts)
                mud_pts.append(Gf.Vec3f(cx * ssf, cy * ssf, z_mud))
                for i in range(_POND_SIDES):
                    mud_counts.append(3)
                    mud_idx += [center_i, mbase + i,
                               mbase + (i + 1) % _POND_SIDES]

        made.append(_finish("{0}/{1}".format(root, tag),
                            core_pts, core_counts, core_idx,
                            mats[mat_key], z))
        made.append(_finish("{0}/{1}_rim".format(root, tag),
                            rim_pts, rim_counts, rim_idx,
                            mats[rim_key], z))
        if mud_pts and mud_mat is not None:
            made.append(_finish("{0}/{1}_mud".format(root, tag),
                                mud_pts, mud_counts, mud_idx,
                                mud_mat, z_mud))
        xs = [p[0] for p in core_pts]
        ys = [p[1] for p in core_pts]
        print("[surge] ponding/{0}: {1} pond(s), x[{2:.1f},{3:.1f}] "
             "y[{4:.1f},{5:.1f}] stage-units, z={6:.3f} (ssf={7:.4f})".format(
                 tag, len(group), min(xs), max(xs), min(ys), max(ys), z, ssf))
    return made


def _dry_material(stage, path, rgb, rough, scale, desat, texture,
                  normal=None, orm=None):
    """A tinted, world-projected `damage._pbr` look — the exact pattern
    `scour_relief.materials()` uses for its soil/silt/sod classes, reused
    here for wrack, washover and pond mud: `diffuse_tint`/
    `albedo_desaturation`, NOT `diffuse_color_constant`, is what can take a
    texture's own colour toward a target tint (`scour_relief.py`'s
    documented reason).

    `normal`/`orm`, if given, are the diffuse map's `_N`/`_ORM` siblings
    (`tools/import_megascans.py`'s own naming and the one place in this repo
    that already binds them this way: `normalmap_texture`,
    `enable_ORM_texture` + `ORM_texture`). `_pbr` alone binds diffuse only —
    every existing caller of it in this file (before this fix) left a
    close-up soil surface completely flat, which was the reviewed "flat
    untextured tan polygon" complaint. `reflection_roughness_texture_
    influence` is pinned to 0.0 whenever an ORM is bound, regardless of what
    `_pbr` left it at: `_plans/hurricane_water.md`'s risk #7 (unverified,
    but cheap to guard against here) is that an ORM's roughness channel can
    silently override a material's own tuned `rough`/`roughness` constant
    once a texture is present — this keeps `rough` above authoritative
    either way.
    """
    from pxr import Gf, Sdf, UsdShade

    import scene_generator as sg
    from . import damage

    try:
        mat = damage._pbr(stage, path, rgb, rough, tint=rgb, scale_uv=scale,
                          texture=sg._join_asset_root(texture, ""))
        sh = UsdShade.Shader.Get(stage, path + "/Shader")
        if sh:
            sh.CreateInput("diffuse_tint",
                          Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            sh.CreateInput("albedo_desaturation",
                          Sdf.ValueTypeNames.Float).Set(float(desat))
            if normal:
                sh.CreateInput("normalmap_texture",
                              Sdf.ValueTypeNames.Asset).Set(
                    Sdf.AssetPath(sg._join_asset_root(normal, "")))
            if orm:
                sh.CreateInput("enable_ORM_texture",
                              Sdf.ValueTypeNames.Bool).Set(True)
                sh.CreateInput("ORM_texture", Sdf.ValueTypeNames.Asset).Set(
                    Sdf.AssetPath(sg._join_asset_root(orm, "")))
                sh.CreateInput("reflection_roughness_texture_influence",
                              Sdf.ValueTypeNames.Float).Set(0.0)
        return mat
    except Exception as exc:
        print("[surge] material {0} unavailable ({1})".format(path, exc))
        return None


def build_deposits(stage, parent_path, cfg, region, rng, *, ssf=1.0,
                   materials=None):
    """L4b + L4c-shadow: wrack windrows and washover fan/mound geometry,
    merged by class into one `scour_relief.build` call (it already merges by
    class, and both `wrack_specs` and `_washover_specs` are
    `scour_relief.geometry`-shaped specs — `ridge` for wrack, `mound`/`fan`
    for washover).

    L4a (the silt film) is NOT authored here. It is `silt_coverage` +
    `ground.build_overlay`, called by the LAUNCHER directly (see that
    function's docstring) — authoring it a second time in here as well, over
    a different root, would double-composite the same translucent mud band
    in the overlap and darken it, exactly the class of bug this codebase's
    ground-overlay code is written to avoid elsewhere. `build_deposits`
    covers only the two layers that have no other caller.

    NOT AUTHORED EITHER: the COLOUR half of L4c ("both routes" per the water
    plan). A pale washover sheet needs a TINTED overlay (`Dirt_Rough` alone
    is not sand-pale), and `ground.overlay_material` has no tint input —
    adding one means editing `ground.py`, outside this module's scope. The
    washover SHAPE (raised, pale-tinted fan/mound geometry, still catching a
    shadow) carries the signature on its own; flagged in the report as a
    scope reduction, not an oversight.

    `ssf` — see `build_inundation`. `materials`, if given, is a dict that MAY
    carry `"wrack"`/`"sand"` looks to reuse instead of building fresh ones
    (the launcher's example does not pass this; it is accepted for the same
    reason `build_inundation`/`build_ponding` accept a shared dict).
    """
    from pxr import UsdGeom, Sdf

    kn = resolve_cfg(cfg)
    ssf = float(ssf)
    UsdGeom.Scope.Define(stage, Sdf.Path(parent_path))
    root = "{0}/deposits".format(parent_path)
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    made = []

    combined = wrack_specs(kn, region, rng) + _washover_specs(kn, region, rng)
    if combined:
        # SANITY, PRINTED, NOT ASSERTED -- on plain spec dicts, before the
        # one pxr call in this function. `scour_relief.build` applies `ssf`
        # correctly to whatever it is handed (verified by reading it), so a
        # huge mesh here would mean a bad SPEC (an `rx`/`len_m` this module
        # generated too large), not a scale bug in the authoring step.
        big = sorted({s.get("cls", "?") for s in combined
                     if max(float(s.get("rx", 0) or 0), float(s.get("ry", 0) or 0),
                            float(s.get("len_m", 0) or 0),
                            float(s.get("h", 0) or 0),
                            float(s.get("height_m", 0) or 0)) > 50.0})
        if big:
            print("[surge] WARNING deposit spec(s) of class {0} exceed a "
                 "50 m extent -- that is not a windrow or a sand fan, check "
                 "`wrack_specs`/`_washover_specs` before trusting this "
                 "render".format(big))
        materials = materials or {}
        wrack_mat = materials.get("wrack") or _dry_material(
            stage, "{0}/DepositLooks/wrack".format(parent_path),
            rgb=(0.30, 0.27, 0.20), rough=0.92, scale=(0.9, 0.9), desat=0.30,
            texture=SILT_TEXTURE, normal=SILT_NORMAL_TEXTURE,
            orm=SILT_ORM_TEXTURE)
        sand_mat = materials.get("sand") or _dry_material(
            stage, "{0}/DepositLooks/sand".format(parent_path),
            rgb=(0.78, 0.72, 0.58), rough=0.85, scale=(0.6, 0.6), desat=0.15,
            texture=WASHOVER_TEXTURE, normal=WASHOVER_NORMAL_TEXTURE,
            orm=WASHOVER_ORM_TEXTURE)
        made += scour_relief.build(
            stage, "{0}/relief".format(root), combined,
            {"wrack": wrack_mat, "sand": sand_mat}, ssf, verbose=False)
        print("[surge] deposits: {0} spec(s) ({1} wrack, {2} washover) -> "
             "{3} merged mesh(es) {4} (ssf={5:.4f})".format(
                 len(combined),
                 sum(1 for s in combined if s.get("cls") == "wrack"),
                 sum(1 for s in combined if s.get("cls") == "sand"),
                 len(made), made, ssf))

    return made
