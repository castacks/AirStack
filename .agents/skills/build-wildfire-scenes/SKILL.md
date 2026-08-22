---
name: build-wildfire-scenes
description: Build or modify burnt/damaged scenes in scene_gen — fire and smoke via NVIDIA Flow, mesh fracture, physics settling, scorch materials, and the burn-age model that ties them together. Read before touching scene_gen/disaster/, the damage benches, or suburb_mini_wildfire. Contains the bug catalogue; most "obvious" fixes here have already been tried and were wrong.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Build Wildfire / Damage Scenes

## Why this file is long

Almost everything in this pipeline has a counter-intuitive failure mode, and
several knobs have been set the "physically obvious" way, looked wrong, and
been reversed. The catalogue below is the point of the document: it is cheaper
to read than to rediscover, and a plausible-sounding change here has roughly a
50% chance of being one that was already tried.

## The pipeline

    layout -> generate -> damage -> fracture -> settle -> scorch -> fire

One number drives the last four stages: **burn age**, the time since the fire
front reached a given point. `disaster.fire` solves when an elliptical front
arrives anywhere, and `damage.level_for_age` turns that into a structural
level, a finish (fresh char vs cooled ash), a scorch coverage and a fire state
*together*. Scattering those independently is what makes a scene read as four
unrelated effects instead of one event.

## The modules

| file | does |
|---|---|
| `scene_gen/disaster/fire.py` | NVIDIA Flow stack, emitters, spread model, timeline driver |
| `scene_gen/disaster/damage.py` | damage levels, break-up, scorch/soot materials, burn-age mapping |
| `scene_gen/disaster/fracture.py` | Voronoi mesh fracture, partial (ragged) breaks, fragment consumption |
| `scene_gen/disaster/settle.py` | PhysX drop + bake to static, with drop/spread measurement |
| `scene_gen/disaster/scorch.py` | composites soot onto a surface's own texture; bakes ground burn scars |
| `scene_gen/disaster/rubble.py` | procedural convex debris (superseded by real fracture; kept for comparison) |
| `scene_gen/tools/burn_textures.py` | generates the char / scorch / ash maps |
| `scene_gen/tools/import_megascans.py` | Megascans/glTF surface zip -> bound `.usda` material |
| `scene_gen/tools/fire_png.py` | host-side burn plan, no Isaac Sim needed |

Launchers: `house_damage_test_launch_script.py` (severe bench),
`house_partial_damage_test_launch_script.py` (partial bench),
`suburb_mini_wildfire_launch_script.py` (250 x 250 m full scene),
`fire_test_launch_script.py` (four Flow stacks on empty ground).

## Environment

`trimesh` ships with Isaac Sim but is **crippled** without two pip packages:
`manifold3d` (boolean engine) and `shapely` (capped plane slicing). Both are in
`simulation/isaac-sim/docker/Dockerfile.isaac-ros`, and
`fracture.ensure_deps()` installs them at runtime for containers built from an
older image. Without them the only fracture available is "scatter whole
panels".

**Iterate with tmux, not `airstack down`.** `docker exec isaac-sim tmux
send-keys -t isaac C-c` then re-send the launch command — see
`docker-compose.yaml` for the exact line. `airstack down` REMOVES the
container and destroys anything pip-installed into it. Prefix the relaunch
with `clear;` — stale scrollback has repeatedly been misread as a fresh run.

---

# The bug catalogue

## Flow / fire

**Flow prims have no children, and `_set` failed silently.**
`FlowCreatePrim` is literally `stage.DefinePrim(path, type_name)` — it creates
ONE prim. Every setting that matters lives on child prims (`advection`,
`colormap`, `rayMarch`, `renderSettings`), so reading them back gave invalid
prims and the writes were dropped. Result: combustion never enabled, flame
colours never authored, and Flow rendered smoke and nothing else. This is also
why `isaacsim.replicator.incident` has `setup_pyro_settings()` commented out
and loads `warehouse_fire_settings.usda` instead. Fixed by `_child()`, which
defines each param prim with its schema type. **`_set` now warns instead of
returning quietly** — that silence cost three debugging rounds.

**The block pool is a carb setting, not a USD attribute.**
`flowRender/renderSettings.maxBlocks` does nothing on its own; the pool is
`rtx/flow/maxBlocks`. Symptom: `Maximum Flow blocks of 4096 in use` and
emitters past the first few silently getting no allocation — which reads as
"the fire is only in a couple of places". Also **straight VRAM**: 65536 blocks
will lose a 16 GB card. 16384 is the current value.

**Temperature is the glow, not fuel.** A "smoke only" emitter still injecting
`temperature` renders as a glowing ball with no visible smoke. `STATE_EMISSION`
zeroes temperature AND `coupleRateTemperature` for the non-flaming states.

**Smoke channel damping/fade is a scale trap.** NVIDIA's warehouse values
(0.30 / 0.65) kill a plume within a few metres — correct for a burning crate,
and on a building it produces a string of smoke BALLS. But going too far the
other way (0.04 / 0.06) keeps every puff alive almost indefinitely and sends
columns into the sky. Current: 0.14 / 0.26.

**Emitters on module positions sit on the perimeter.** A house's modules ARE
its walls, so seeding emitters at them puts every plume on the outside edges.
Use the FLOOR modules: `modular_house` tiles the interior with
`Floor_Quart_01`, so the floor plate is the space inside the walls. Averaging
all modules is also wrong — porch and bay protrusions dragged the centre 1.6 m
off. And `half_x` is already HALF the footprint; an extra `* 2.0` put emitters
1.1 half-widths out, back through the walls.

**Plume height must come from settled geometry.** Taking it from the authored
roof z left smoke hanging where the roof used to be. `add_structure_fire` takes
`base_z`/`top_z`; the caller measures the real world bbox AFTER settling. Both
flame and smoke are then capped by that height — once damaged houses collapse
outright there is nothing for flame to climb either, so fixing only the smoke
left floating fireballs behind.

## Fracture

**Kit meshes are not watertight.** They are game art: open shells with no back
faces, so `mesh.volume` is meaningless and **every boolean intersection
returns nothing**. Do not fracture with booleans. `slice_mesh_plane(...,
cap=True)` against each Voronoi bisector produces the identical partition, caps
the open edges, and does not care about manifoldness. (`manifold3d` is still
needed as trimesh's engine; `shapely` is what capping actually uses.)

**Instanced prims cannot be authored into and cannot be read through.**
`fence`, `tree`, `sidewalk`, `streetlight`, `sign` etc. are in
`instance_categories`. `Usd.PrimRange` does not descend into an instance, so
material binding silently did nothing AND `prim_to_mesh` returned None — the
scorch and the fracture both failed, silently, and widening thresholds twice
changed nothing. Fixes: `Usd.TraverseInstanceProxies()` for reading, and
un-instance burnable categories in the preset. Metal (hydrants, poles, signs)
stays instanced; it survives a fire.

**Roughening makes fragments overlap, and PhysX detonates.** Voronoi cells
share exact boundaries, so displacing vertices outward makes every fragment
interpenetrate its neighbours; PhysX resolves that at unbounded speed. Fix:
~3% inset about the centroid, plus `maxDepenetrationVelocity` capped at 0.6.
Measured max spread went 24.8 m -> 5.16 m.

**A clean cut is not a partial collapse.** Slicing a wall at a plane and
fracturing the top leaves a razor edge, which no failed wall has. Fracture the
WHOLE wall and judge each fragment against a break line that wanders with
horizontal position; the surviving edge is then made of real cell boundaries.

**`min_volume_frac` caps how fine a cut is worth making.** Past ~20 seeds every
new cell falls under the floor and is discarded, so raising seeds alone stops
shrinking the median (0.34 m3 plateau). Lowering it to 0.0012 does produce
finer rubble (0.27 m3) — but that read as gravel, not a collapsed house, so it
stays at 0.004.

**Fragment consumption: skew LARGE, not small.** This has been all three ways:
- *small-biased* — "a splinter is consumed, a beam chars" is true of the
  material, but it strips exactly the rubble that makes a pile read as rubble
  and leaves a heap of slabs.
- *uniform* — better; the slabs are still what you notice.
- *large-biased* — what reads. A big surviving panel looks like a wall that
  fell over; remove those and what is left is debris. Also defensible: a large
  panel has the most surface exposed and least chance of being shielded.
Candidates are drawn from the large end and chosen among, so it is a bias, not
a rule. Consumption happens BEFORE authoring, so a dropped fragment never
becomes a prim, a collider, or a rigid body.

## Physics

**`timeline.play()` + `app.update()` does not reliably step physics** in a
standalone SimulationApp — nothing attaches the physics scene, so the app
renders happily while the solver never runs and every body stays where it was
authored. Use `SimulationContext.initialize_physics()` + `step()`.

**A settle that does nothing looks identical to one that found everything at
rest.** `settle.run` therefore MEASURES: mean/max displacement, split into
**drop** (down, = collapsing) and **spread** (outward, = exploding). Those two
numbers are the diagnostic; total displacement cannot tell the cases apart.
Healthy figures on the mini scene: drop ~-2.2 m, spread ~0.65 m mean.

**A flat quad cannot be a convex-hull collider.** The ground is planar, its
hull is degenerate, PhysX cooks nothing, and everything falls through the
world. Static colliders use `approximation="none"` (triangle mesh, legal for
static and more accurate); only dynamic bodies get hulls. There is also a
backstop infinite plane, because one uncooked collider loses a piece to
infinity and a 25 m fall is indistinguishable from a settle in the logs.

**Damage must be gravity-aware.** Picking modules independently destroys a
ground-floor wall while the wall above, the floor it carried and the roof over
that all survive — floating. `cascade_supports` closes the pick under support:
repeatedly add anything ABOVE a broken module within 4 m, until nothing new is
added. Walls are picked lowest-first (fire burns upward).

**Broken pieces must be dynamic, including stubs.** `fracture_partial` returns
`(statics, loose)`; treating the statics as static colliders pins them where
the cut left them, so a stub over a consumed floor fragment hangs. Everything
in a damaged building is now simulated EXCEPT an intact wall — the only element
still carrying its own load. `pristine` and `scorched` houses stay static
entirely; nothing in them was broken.

**Baking is the point.** `settle.run(bake_result=True)` writes resting
transforms back as static and disables every body, so the scene ships as
ordinary geometry arranged by gravity at zero runtime cost. `KEEP_PHYSICS=1`
leaves bodies live for hand-testing — drag a piece, press play, it falls. If
"nothing falls when I move it", that is baking working as designed.

## Materials

**Read the shader before writing to it.** An early soot pass guessed input
names and force-set any it found. `albedo_add` is a **float** in OmniPBR;
writing a colour into it turned doors and windows blue. The real knobs are
`diffuse_color_constant` (color), `albedo_brightness` (**float** — the
darkening one), `diffuse_tint` (color).

**Kit materials expose no tint at all,** so darkening the bound material does
nothing on walls. Instead read the surface's own base-colour TEXTURE and build
our own OmniPBR around it — parameters are then known because we authored them.

**Every material in the kit is named `UnrealMaterial`.** The name tells you
nothing; identity is the base-colour texture filename (this is how
`modular_house.apply_palette` does it). Interior vs exterior is decided by the
placement CATEGORY (`house_floor` is the interior), not by the material.

**The base-colour picker must exclude map types.** A fallback that accepts any
image will grab NORMAL and ORM maps, which is why brick came out unscorched or
oddly tinted. Exclude `normal`, `orm`, `rough`, `metal`, `height`, `ao`, ...

**A prim handle does not survive its own subtree being deactivated.** A kit
module's material usually lives INSIDE the module (`.../fence_11_29/_materials/
Planks_Color`), so fracturing expires it and traversing raises `Invalid range
starting with expired 'Material' prim`. Resolve to a path STRING first —
`damage.bound_texture()`.

**Fracture fragments carry no UVs.** `_write_mesh` authors points and faces
only, so a UV-space material has nothing to map with and the scorch does not
appear. Pass `triplanar=True`, which projects from world coordinates — and on
a vertical face the texture's V lands on world Z, so an up-wash gradient still
runs the right way.

**Bucket 0 is the LIGHTEST scorch, not none.** `soot_materials` skipping on
`level <= 0` left everything the fire had only just reached completely clean —
which is why heavily damaged buildings still had pristine-looking walls. Skip
only on genuinely zero coverage.

**Scorch direction depends on where the fire was.** A WILDFIRE arrives at
ground level and washes UP the outside of a wall, tapering into licks. A
compartment fire vents from the openings and is heaviest under the eaves. The
first cut assumed the compartment case and looked upside down. Floors keep the
top-down variant — a floor is fouled by what falls on it.

**Interiors burn harder than exteriors** — a compartment fire is hot, enclosed
and oxygen-starved, so internal surfaces sit in the flame while the outside
sees only what vents out. `interior_bonus` pushes `house_floor` darker.

**You cannot scorch water.** `soot_materials` walks every placement, so it
happily composited a soot wash over pool water and tile and corrupted both.
`damage.INCOMBUSTIBLE` (substring match: `pool`, `water`, `fire_hydrant`,
`streetlight`, `sign`, ...) is skipped by scorching, breaking, consumption and
settling alike. This is not just a rendering nicety — a pool surviving intact
is one of the clearest features in real burn-scar imagery, and metal street
furniture comes through a fire visibly untouched.

**You cannot scorch water.** `soot_materials` walks EVERY placement, so it
happily composited a soot wash over pool water and tile surround and corrupted
both. `damage.INCOMBUSTIBLE` (substring match: `pool`, `water`,
`fire_hydrant`, `streetlight`, `sign`, `manhole`, `storm_drain`) is now
excluded from scorching, breaking, consumption AND settling. This is not a
rendering nicety: a pool surviving intact is one of the clearest features in
real burn-scar imagery, and metal street furniture comes through a fire
visibly untouched, so excluding them is more correct as well as less broken.

**Consumption goes through one guarded choke point.** Every "the fire consumed
this" removal calls `consume_prim()`, which refuses on incombustible
categories. A guard that lives next to each filter is one that gets forgotten
when a new pass is added, and losing a pool to a stray substring match is hard
to spot — the symptom is a hole in the scene, not an error.

**Coverage, not brightness.** Scaling brightness darkens uniformly and reads as
a dimmer switch. Moving COVERAGE grows the fouled area, which is what looks
scorched. Uniform is the one thing soot never is.

## Textures

**Value noise on a lattice shows its grid.** The first generator built
axis-aligned value noise and indexed the same lattice coordinate on both axes —
separable, so at the high cell counts the ash flecks needed the grid showed
through as rectangles. It also was not seamless, so UV tiling laid a second
grid over the first. Fixed by **spectral synthesis**: white noise shaped by a
radial `1/f^(beta/2)` falloff in the frequency domain. Depends only on radial
frequency (no preferred axis, no cells) and the DFT is periodic (tiles
exactly).

**Seamless is not enough — low-frequency contrast is what makes a tile
visible.** One big recognisable blotch is tracked by the eye across every
copy. But killing the low frequencies entirely gives flat grey paint. The
landing point is MID-frequency band-limiting: structure at 1-20 cm, nothing
large enough to recognise when it repeats.

**Fractal noise cannot make plates or flakes** — it has no hard edges. Charred
wood is alligator CHECKING (cells elongated along the grain) and ash is angular
FLAKES with hard value jumps. Both need a jittered **Voronoi** field, with
anisotropy applied in the DISTANCE METRIC (scaling one axis before the
nearest-neighbour lookup), not by using fewer rows — that just makes bigger
square cells.

**Charred wood is BLACK with pale ash ON it,** not pale with black specks. An
inside-out version reads as white paint. Research finding worth keeping: cracks
in charred wood run mainly PERPENDICULAR to the fibres, not along them, because
charring drives the stress thermally rather than by drying shrinkage — that is
what produces the alligator grid, and it is the opposite of the intuitive
"splits along the grain".

**A fire scar on the ground is a MOSAIC**, not a gradient — patchy mottled
interior inside an irregular fingered outline, with unburned islands the fire
skipped. No directionality, nothing like the wash on a wall. A tiled material
cannot express it; bake one texture for the whole plate
(`scorch.ground_burn_map`).

**Cache keys must include the recipe.** Retuning `SOOT_RGB` or `char_bite`
silently reused maps baked under the old values, so nothing appeared to change.

## Scene / config

**100 x 100 m produces ZERO houses.** `suburb_net` needs room for its street
hierarchy and a lot is 21-30 m wide before carriageway and verge. Measured:
100/150/200 m -> 0 placements; 250 m -> ~630 placements, ~12-16 buildings,
2 blocks. **250 m is the floor.**

**Do not hard-code the burn clock.** The config's own duration left 7 of 13
buildings pristine; widening and running longer made all 13 rubble. Neither
number is knowable in advance because it depends on where the generator put
houses relative to the ignition point. DERIVE it: run the clock to the moment
the last building is reached and scale the phase lengths to the spread of
arrival times.

**The road z ladder scales with the plate.** `_Z_ASPHALT = 0.10`,
`_Z_DASH = 0.24` were chosen for a 1600 x 1200 m plate where 14 cm is
sub-pixel; on a 250 m block the roads visibly hover. The offsets are not a
property of the road — they follow from camera distance, which tracks plate
size. `apply_ground` now derives `z_scale = clamp(span / 1600, 0.08, 1.0)`;
1600 m reproduces the tuned values exactly.

**Cut pool holes from `ground_base` too.** The block meshes subtract
`pool_rects`, but the backdrop plane did not — so the hole revealed an opaque
plane and the pool was never visible.

---

# Current knob values, and why

| knob | value | why |
|---|---|---|
| `densityCellSize` | 0.1 m | 0.25 dilutes the burn until temperature never reaches the colormap's flame band; NVIDIA's warehouse uses 0.05 |
| `rtx/flow/maxBlocks` | 16384 | 4x NVIDIA's default; 65536 exhausts a 16 GB card |
| `colormap_x_max` | 1.0 | NVIDIA's own; the 0.35 that was here explained flameless smoke that turned out to be missing param prims |
| smoke channel damping/fade | 0.14 / 0.26 | 0.30/0.65 = smoke balls, 0.04/0.06 = columns into the sky |
| `consume` | 0.30 | 0.45 removed too much; skewed to LARGE pieces |
| `min_volume_frac` | 0.004 | 0.0012 gives finer rubble that reads as gravel |
| Voronoi seeds | 9-14 by severity | finer looked unrealistic |
| `kick` | 0.15 m/s | 0.9 + 25 rad/s was an explosion |
| `maxDepenetrationVelocity` | 0.6 | the explosion knob |
| `SOOT_LEVELS` | 0.52-1.00 | coverage, not brightness |
| ellipse head:flank | 4:1 (2:1 on the mini block) | 6:1 is a ~330 m cigar that reads as scattered burning trees |
| region floor | 250 m | below this the generator yields nothing |
| `consume` skew | large-biased | see the three-way history above |
| plume z (flame) | 0.3 - min(2.0, height) | capped by REAL settled height; collapsed houses have nothing to climb |
| plume z (smoke) | 0.15 - min(1.1, height) | smoke never climbs on its own; a debris bed emits at ground level |
| `rubble` smoke strength | 1.9x | a fully collapsed house is almost pure smoke; partly standing shells are not boosted |

# Diagnostics

- `settle` prints **drop vs spread** — down = collapsing, out = exploding.
- `soot_materials` prints materials it could not scorch (no base colour found).
- `fracture_prim(verbose=True)` prints seeds -> fragments, face count and
  watertightness per module.
- `tools/fire_png.py` renders the burn plan host-side in ~1 s: emitter count,
  arrival spread, peak CONCURRENT emitters (the frame-rate number), and the
  t=0 state mix. Use it before spending a container launch.
- `fire_test_launch_script.py` prints a **type check** of every Flow prim plus
  a combustion readback. If those are wrong, no amount of tuning will help.

# Known gaps

- **`scorch.ground_burn_map` is written and tested but never runs** in the mini
  scene: the guard looks for `ground/ground_base`, but the ground is built as
  per-block meshes (and now `ground_base_N` when pools are cut). The ground is
  still plain tiled grass. **This is the top outstanding item.**
- **Road line materials are imported but unbound** — lane dashes use
  `displayColor` with no material by design, so they need explicit rebinding.
- **No burnable props exist** in the mini block (categories are only `fence`,
  `house_*`, `plot_pool*`), so the prop-burning pass has nothing to act on.
- Fragments are cut from the module's outer shell, so they are hollow inside.
- Fracturing at scene-build time will not scale to a full plat; bake fractured
  results to disk and reference them.
