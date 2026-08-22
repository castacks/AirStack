---
name: simulate-water-in-isaac-sim
description: Add flood, storm-surge or standing water to an Isaac Sim scene. Three tiers — shaded flood plane, Warp-deformed surface, PhysX particle fluid — with the tier-1 plane recommended for aerial disaster datasets. Use when building flood/hurricane scenes in scene_gen, or when asked how to put water in Isaac Sim.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Simulate Water in Isaac Sim

## When to Use

Adding water to a `scene_gen` disaster scene — flood, hurricane storm surge, standing
water in streets. Also the reference for "can Isaac Sim do water?" questions.

Not for fire or smoke. Those are NVIDIA Flow — see `scene_gen/disaster/fire.py` and
the Flow notes in `scene_gen/GENERATION.md`.

## The one thing to know first

**Flow does not do water.** NVIDIA Flow (`omni.flowusd`) is a sparse-voxel *gas*
simulation with a combustion model. It renders fire and smoke. It has no liquid
surface, no free-surface tracking, no wetting. Reaching for Flow to make a flood is
the most common wrong turn here.

Isaac Sim has **no built-in water**. NVIDIA staff confirm on the forums that oceanic
environments are not officially supported and that PhysX has no hydrodynamics —
buoyancy must be computed by the user and applied to rigid bodies. Everything below
is assembled from parts.

## Tier 1 — flood plane + water MDL (**recommended** for aerial datasets)

A single large mesh at the flood height, bound to a water MDL. No simulation, no
per-frame cost, and it carries the entire visual signal of a flood from a drone's
altitude: a water surface intersecting houses, cars and fences at a known depth.

The material is **already vendored in this repo**:

```
scene_gen/assets/aec/brownstone/Materials/vMaterials_2/Liquids/Water_Blue_Ocean_Perlinwaves.mdl
```

Exported materials (pick with `info:mdl:sourceAsset:subIdentifier`):

| Material | Use |
|---|---|
| `Water_Ocean` | default, "Caribbean Sea" tuning |
| `Water_Ocean_Light_Blue` | shallower / lighter |
| `Water_Ocean_Blue_Reef`, `Water_Ocean_Green_Reef` | murkier |
| `Water_Pool_Clear_Blue`, `Water_Pool_Clear_Green` | still water, pools |
| `Water_Ocean_` | plain variant |

Parameters worth driving from a disaster config:

| Input | Meaning | Note |
|---|---|---|
| `waves_strength` | wave amplitude | 1.0 default |
| `waves_roughness` | surface break-up | 0.847 default |
| `waves_stretch` | anisotropy of the wave pattern | 0.734 default |
| `waves_phase` | static phase offset | |
| `level_progressive_v_motion` | **animate this for motion** | costs nothing |
| `texture_scale` | wave feature size vs. scene units | default `(1, 2)` — tune for metres |
| `water_tint`, `water_absorbtion`, `water_scattering_amount` | murk / colour | flood water is brown, not Caribbean |
| `ior` | 1.333 | leave it |

### Why this tier is the right one for a dataset

Flood **depth becomes a config knob**, not an emergent property. Drive
`water_level_m` from `severity` and you get, for free:

- correct partial submersion of every prop by geometry alone
- **exact per-object ground truth**: object bbox min-z vs. water plane z. That is a
  cleaner label than any fluid sim would produce.
- composition with the existing `disaster.field` — in `compile_flood` the radial
  field already biases toward low ground at the epicentre, so the water level can
  follow the field and flood deeper there.

## Tier 2 — dynamic surface via Warp

`omni.warp-1.8.2` ships in the Isaac Sim container with working sample scenes:

| Asset (under `/isaac-sim/extscache/omni.warp-1.8.2/`) | What it is |
|---|---|
| `data/scenes/ocean.usda` | ready ocean scene |
| `omni/warp/nodes/_impl/OgnSampleOceanDeform.py` \| `.ogn` | `WarpSampleOceanDeform` OmniGraph node |
| `data/scenes/wave_solver.usda` + `OgnWaveSolve.ogn` | `WarpWaveSolve`, 2D wave-equation grid |
| `data/scenes/assets/materials/ocean_material.usda` | reference material |
| `data/scenes/assets/geometries/ocean.usd`, `plane_256x256.usd` | grids to deform |

`WarpSampleOceanDeform` inputs — a Tessendorf-style spectral ocean:
`amplitude`, `windSpeed` (0–30 m/s at 10 m), `waterDepth` (1–1000 m), `direction`,
`directionality` (isotropic ↔ directional), `scale` (horizontal, **use it to match
metres**), `clipmapCellSize`, `cameraPos`, `time`.

The **clipmap** is the reason this scales: the grid re-centres on `cameraPos`, so it
reaches the horizon without a horizon-sized mesh. This is the storm-surge /
hurricane option.

`WarpWaveSolve` inputs: `size`, `resolution`, `amplitude`, `speed`, `damping` — a
shallow-water wave equation you can perturb locally. Use when you want a flood
surface that reacts, rather than a deep-water spectrum.

Reference material `ocean_material.usda` is thin-walled `OmniGlass.mdl` at
`glass_ior 1.325`, with mdl `surface`, `displacement` and `volume` all connected.

Cost: GPU mesh deformation every frame. No physics coupling.

## Tier 3 — PhysX particle fluid (PBD). Local shots only.

`omni.physx` 107.3.26. Demos in `/isaac-sim/extscache/omni.physx.demos-*/omni/physxdemos/scenes/`:

- `FluidBallEmitterDemo.py` — emitter → fluid
- `ParticleSamplerDemo.py` — fill a mesh with particles
- `ParticlePostProcessingDemo.py` — smoothing / anisotropy / **surface extraction**

GPU-only (`omni.physx` particle sim requires GPU dynamics). Realistic budget is a
room or a street corner, not a 400 m region — SPH/PBD tracks millions of particles
and does not survive a city-scale domain.

Use it for one deliberate shot (a surge hitting a wall) and composite, or skip it.

## Buoyancy

PhysX models rigid bodies only; there are no hydrodynamic forces or moments. The
NVIDIA staff answer (Philipp Reist, forum thread 280374) is: compute the forces
yourself — Warp is the suggested vehicle — and apply them to the rigid body based on
submerged volume, water density and gravity.

Community reference implementation: `leonlime/isaac_underwater` on GitHub (floating
box, ROV). `OceanSim` (arXiv 2503.01074) is the research-grade underwater perception
framework.

**For an aerial dataset you probably do not need buoyancy at all.** You need water
that reads correctly from above and props sitting at plausible depths — tier 1 gives
that by placement.

## Rain and reduced visibility

Do not reach for particles. RTX has fog settings, free and adjustable at runtime via
`omni.kit.commands.execute("ChangeSetting", path=..., value=...)`:

```
rtx/fog/enabled              rtx/fog/fogStartDist      rtx/fog/fogHeightDensity
rtx/fog/fogColor             rtx/fog/fogEndDist        rtx/fog/fogHeightFalloff
rtx/fog/fogColorIntensity    rtx/fog/fogDistanceDensity
rtx/fog/fogZup/enabled       rtx/fog/fogStartHeight
```

Set `rtx/fog/fogZup/enabled` — these scenes are Z-up. The same settings double as
wildfire haze.

## Dead ends — do not spend time here

- **`isaacsim.replicator.incident` liquid spill.** The docs list "liquid spills" as a
  supported incident type. In the shipped `0.1.28`, `spill_demon/spill_event.py` is
  **entirely commented out**. The `data/Puddle/Puddle.usd` asset and its
  albedo/normal maps are real and reusable as a decal, but the event is not wired up.
- **Flow for water.** See above.
- **`FlowEmitterNanoVdb` for a liquid.** It imports a NanoVDB volume into the *gas*
  solver. Not a free surface.

## Version note

Verified against **Isaac Sim 5.1** (`omni.physx-107.3.26`, `omni.warp-1.8.2`,
`omni.flowusd-107.1.8`) as shipped in this repo's `isaac-sim` container. Re-check on
6.0 — the 6.0 early-developer announcement adds a FlowUSD fire/smoke SDG writer but
says nothing about water, so tiers 1–3 are expected to stand.

## Sources

- [Forum: building an ocean environment for marine robots in Isaac Sim](https://forums.developer.nvidia.com/t/how-to-build-an-ocean-environment-for-marine-robots-in-issac-sim/280374)
- [Forum: buoyancy physics in Isaac Sim](https://forums.developer.nvidia.com/t/how-to-do-the-physics-of-buoyancy-in-isaac-sim/349879)
- [leonlime/isaac_underwater](https://github.com/leonlime/isaac_underwater)
- [OceanSim (arXiv 2503.01074)](https://arxiv.org/pdf/2503.01074)
- [Replicator Incident extension docs](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/action_and_event_data_generation/tutorial_replicator_incident.html)
