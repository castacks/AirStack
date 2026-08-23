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
| `scene_gen/disaster/vegetation.py` | burnt trees — defoliation, dead foliage, bole char, snapping, toppling |
| `scene_gen/disaster/vtk_fracture.py` | Voronoi mesh fracture, partial (ragged) breaks, fragment consumption |
| `scene_gen/disaster/settle.py` | PhysX drop + bake to static, with drop/spread measurement |
| `scene_gen/disaster/scorch.py` | composites soot onto a surface's own texture; bakes ground burn scars |
| `scene_gen/disaster/ground.py` | the ground burn scar: translucent banded overlay following the fire field, feathered / fingered edge, islands |
| `scene_gen/disaster/bake.py` | export each damaged house/tree to a self-contained USD and reference it back — the scaling path off per-build fracture/settle |
| `scene_gen/disaster/rubble.py` | procedural convex debris (superseded by real fracture; kept for comparison) |
| `scene_gen/tools/burn_textures.py` | generates the char / scorch / ash maps |
| `scene_gen/tools/import_megascans.py` | Megascans/glTF surface zip -> bound `.usda` material |
| `scene_gen/tools/fire_png.py` | host-side burn plan, no Isaac Sim needed |

Launchers: `house_damage_test_launch_script.py` (severe bench),
`house_partial_damage_test_launch_script.py` (partial bench),
`tree_damage_test_launch_script.py` (six burnt-tree methods on grass),
`burn_ground_preview_launch_script.py` (the floor and nothing else),
`suburb_mini_wildfire_launch_script.py` (250 x 250 m full scene),
`fire_test_launch_script.py` (four Flow stacks on empty ground).

## Environment

`trimesh` ships with Isaac Sim but is **crippled** without THREE pip packages:
`manifold3d` (boolean engine), `shapely` (polygon ops for capping) and
`mapbox_earcut` (the cap's triangulation — without it every
`slice_mesh_plane(cap=True)` logs "try running pip install mapbox-earcut" and
returns EMPTY, which surfaced as `wood_debris` returning nothing to unpack).
`vtk_fracture.ensure_deps()` installs all three at runtime, and the Dockerfile
now pins all three (plus `vtk`), so a rebuilt image pays no pip install at all.
An image built before that still does, once, on its first fracture — and
**`airstack down` throws it away** — see the tmux relaunch in
[run-isaac-sim-launcher](../run-isaac-sim-launcher/SKILL.md).

**Two fracture engines share this directory, and they are unrelated.**
`vtk_fracture.py` is the one this skill is about — VTK Voronoi + CSG against
real placed geometry, used by the benches, `damage_flow.py` and `vegetation.py`.
`fracture.py` is the separate trimesh/scipy engine behind `mesh_damage.py` on
the staged pipeline, built on `solids.py`. Each arrived on its own branch under
the same name; the port kept both. Check which one you are importing.

**Iterate with tmux, not `airstack down`** — the full procedure (start, relaunch,
where the output actually goes) is the [run-isaac-sim-launcher](../run-isaac-sim-launcher/SKILL.md)
skill; the short form: `docker exec isaac-sim tmux
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

## Vegetation

**A tree is a bole MESH plus a set of POINT INSTANCERS, and that is the whole
mechanism.** Every AEC tree is built the same way: one solid Mesh for the
trunk and primary limbs bound to a bark material, plus N
`UsdGeomPointInstancer`s each holding exactly ONE prototype, bound to either a
leaf/needle material or a bark one. Fire consumes fine fuel and leaves wood,
which maps exactly onto "drop the leaf instances, keep the wood ones" — no
mesh rebuild, no fracture, no boolean, and per-instance `positions` give a
height for free, so a crown can be thinned with a gradient rather than
switched off.

**`invisibleIds` DOES NOTHING IN THIS KIT, and it fails silently.** It is the
correct USD answer, the attribute exists on every one of these instancers, and
the write succeeds — and this Hydra delegate does not honour it. 2,894 of
2,988 leaf instances were marked invisible and every one still rendered, with
no warning anywhere; the symptom is "the leaves are still on all the trees"
after a log line saying they were hidden. `vegetation.keep_instances()`
rewrites the per-instance ARRAYS instead, which always works. Two rules follow:
filter EVERY per-instance array together (`positions`, `protoIndices`,
`orientations`, `scales`, `velocities`, `angularVelocities`, `ids`) or they
desynchronise and instances take each other's poses; and when nothing
survives, `SetActive(False)` the instancer outright. `MakeInvisible()` on a
plain Mesh is the same trap — deactivate it.

It is also a big RENDER SAVING, not a cost. Black_Oak's four leaf instancers
expand to ~2.68M points against a 41k trunk; a burnt stand is roughly 30x
cheaper than a green one. Every cost note in `suburban.yaml` is about green
trees and none of it carries over.

**On a burnt tree, EVERY instancer goes — wood included.** The tempting rule
is "fire takes fine fuel and leaves wood", so keep the woody branch
scatterers. Wrong: the line is not wood-vs-leaf, it is THICKNESS, and every
one of those scatterers holds branchlets a centimetre or two through. They
burn. Keeping them leaves 747 sticks per oak hanging in the canopy, which
reads as a cloud of disconnected debris, not a skeleton — and the silhouette
that DOES read is already carried by the one mesh named `*_trunk` or `*_base`,
because on these assets that mesh is the bole AND its primary limbs (41k
points, 20 x 20 x 17 m on Black_Oak).

So the burnt-tree rule is by PRIM NAME and deliberately blunt:

    keep      trunk, base, bark, bole, stem
    remove    every PointInstancer, and any mesh named
              branch / leaf / leaves / needle / twig / foliage

Name, not material: `Black_Oak_branch2` binds bark and is still a twig. Fall
back to the material only for what matches neither list — that is what catches
Common_Apple, whose meshes are named `Apple_bark_Mat` / `Apple_leaf_Mat` after
their materials rather than after the part.

**NOT EVERY TREE IS INSTANCED, and three of six are not.** Black_Oak, Shumard
and Douglas_Fir scatter their crowns with PointInstancers. Largetooth_Aspen,
American_Beech and Common_Apple ship ONE PLAIN MESH literally called `leaves`
— and Black_Oak carries four extra leaf meshes at its crown top ON TOP of its
instancers. A defoliation pass that walks only the instancers leaves half the
library fully green with no error anywhere. Mesh foliage is all-or-nothing
(there are no per-instance ids to thin), so it takes one roll against the
survival at its own height.

**The texture is inside the MDL, not in the USD.** These shaders carry only
`info:mdl:sourceAsset = TreeBark_07.mdl`; the map lives in that module as
`diffuse_texture: texture_2d("./textures/...")`. So `damage.bound_texture()`
returns None on EVERY tree and the scorch falls through to plain darkening — a
dim tree, not a burnt one, silently. `vegetation.material_texture()` reads the
MDL when the USD has nothing. Paths inside an MDL are relative to the MODULE,
not to the USD layer that names it.

**Prim and material names are useless; the resolved MDL module is not.**
Douglas_Fir's `branchM` prototype binds a material literally called
`Default_Material` that resolves `TreeBark_10.mdl` — it is wood, and any
name-based classifier calls it unknown or foliage. Classify on the MDL module
or the texture filename.

**Leaf materials have no opacity map.** Checked before rebuilding them,
because an alpha-cut leaf card whose material is rebuilt without its cutout
becomes a solid rectangle. These are plain OmniPBR with a diffuse texture —
the leaves are modelled geometry — so rebuilding around the same texture is
safe. But build the replacement in the asset's OWN UV space: `damage._pbr`
turns on world triplanar whenever it is given a texture, which is right for
UV-less fracture debris and turns a leaf atlas into coloured noise.

**Black_Oak's woody branchlet prototype binds NO MATERIAL AT ALL.** The
123-point mesh its 747-instance crown scatterer points at has no binding, so
those branchlets render untextured. Invisible on a green tree because the
leaves cover them; strip the leaves for a torched skeleton and the whole crown
is suddenly the unshaded thing you notice. `bind_bark()` repairs it from the
tree's own bark so it then takes the same char pass as the trunk. This is an
asset defect, and `measure_assets.py`'s bind count is what finds this class of
thing.

**The wash is what made it "look like repeated cylinders".** `soot_mask`
stacks a directional gradient on two spectral-noise terms. The noise is
seamless by construction (its DFT is periodic); the WASH is not, so a map that
repeats three times up a trunk comes back as a repeating sawtooth of light and
dark bands. A wall's map covers it once and the direction carries real
information, so a wall keeps it. `wash_weight=0` is now the default for boles.

**`Burnt_Forest_Floor` on the wood beats compositing, and it is worth knowing
why.** It is a photographed charred surface authored world-triplanar at
`texture_scale` 0.11 — one tile per ~9 m, so no visible repeat at trunk scale
— and it carries real normal and ORM maps that a composited diffuse-only map
cannot. What it costs is the species' bark identity. `char_bole
(material_path=...)` takes either; the bench's back row stands four
treatments side by side.

**Scorch height is GEOMETRY, not texture.** A bark map tiles several times up
a trunk, so a directional wash composited into it repeats as a stack of bands.
At low severity the black-bottom / clean-top bole IS the signature of a
surface fire that never got into the crown, so the bole is CUT at scorch
height and the two pieces take different coverages. Above `scorch_height >= 1`
nothing is cut, because a torched bole really is charred end to end.

**The bole's longest axis is X, not Z.** Black_Oak's woody bole measures
20.6 x 19.8 x 17.2 m — the limbs reach further than the tree is tall — so
`_seeds`' "longest bbox axis is the grain" assumption picks X and `splinter`
mode turns a standing trunk into horizontal shards. `fracture_mesh(axis=2)`
was added for this; a trunk's grain is Z whatever its crown does.

**`consume=0.30` punches holes through a trunk.** The default is tuned for a
timber wall, where a third of the material genuinely burns away. On a standing
stub it deletes cells out of the middle and leaves the sections above them
hanging. `fracture_partial(consume=...)` is now exposed; trunks pass ~0.05.

**`roughen` scales by the piece's LONGEST extent.** 0.035 on a 20 m bole
fragment is a 0.7 m displacement. Trunks pass `rough=0` and rely on the shrink
alone for PhysX clearance.

**DO NOT VORONOI-FRACTURE A TREE.** This is the one that took three passes to
see. A Voronoi cell is a region of SPACE, and a tree is mostly air — so a cell
routinely contains a section of this limb, a section of that one and a stub of
a third, none of them touching. Authored as one mesh with one rigid body, that
is a clump of branches hanging in formation in mid-air, and no amount of
tuning fixes it because the cells are doing exactly what they are defined to
do. It is the right tool for a wall, which is a slab, and the wrong one here.

What works instead: break with a PLANE (tilted a few degrees so the spar is
not guillotined level), DELETE what comes off, and put constructed geometry in
its place — lengths cut from the low trunk, where the bole is a single
connected column, each passed through `largest_component()`. Every piece is
then one solid object by construction. `trimesh.split(only_watertight=False)`
is the check; the watertight filter rejects every kit mesh, since they are
open shells.

**A fallen tree with its branches on does not lie down.** The limbs hold the
trunk metres clear of the ground and it reads as floating — but it is not a
physics bug, the tree really is resting on its branch tips because nothing
broke them. Real ones lose their limbs going over: thin, fire-weakened, and
they take the whole impact. `clip_to_column()` cuts the bole to a prism about
its own axis before it falls; limb stubs inside the radius survive, which is
what keeps it a tree rather than a sawn pole.

**Consumption skews SMALL on wood and LARGE on a house, and both are right.**
`fracture_mesh`'s own `consume` is large-biased because a big surviving panel
reads as a wall that fell over. A tree is the opposite case: what actually
breaks off a burnt bole and lies on the ground is trunk sections and major
limbs, while the thin outer branches do not fall — they are CONSUMED, being
the first fuel to go and most of the reason the top came off at all. Breaking
high and keeping everything left the ground littered with twigs and no timber.
`consume_thin()` filters by POINT COUNT, not bounding box: these are Voronoi
cells, so a cell holding a few branch tips still has a bbox metres across.

**An instance is not a prim, so it cannot be simulated.** After a bole is cut,
the crown's woody branchlets hang in the air exactly where they were — they
have no transform to author, no collider and no rigid body. The way out is
`ComputeInstanceTransformsAtTime`, which gives every instance's matrix, so a
handful can be BAKED into ordinary meshes at their own transforms, handed to
the solver, and their instances hidden. The rest of the crown stays instanced
and costs nothing.

**`_write_mesh` BINDS NOTHING.** It authors points and faces and nothing else
— no material, no UVs — so every splinter, fallen limb and snapped segment
comes out unbound and renders as untextured grey beside a charred trunk. The
house bench binds its fragments explicitly right after fracturing; the
vegetation path routes every generated piece through `bind_all(wood_material
(...))` for the same reason. Triplanar, since those meshes have no UVs.

**Cut debris from the FOOT of the tree, and filter by size rather than sorting
by it.** A slab taken between 18% and 42% of a 17 m oak comes from where the
LIMBS are, and a Voronoi cell there spans the whole limb spread — which is how
"splinters" came out several metres across. Down at 2-13% the bole is just
trunk, about a metre through, so the same seed count gives firewood-sized
pieces. And reject over a length cap instead of sorting biggest-first and
taking the top N: that sort guarantees you keep the worst offenders.

**A branching bole's convex hull is a 20 m blob.** PhysX cooks hulls for
dynamic bodies, so a whole toppled tree comes to rest on the hull of its own
limb tips — floating, with a metre of air under the trunk. The fix is
`settle.run(dynamic_approximation="convexDecomposition")`. It is NOT to cut
the bole into many segments: four segments removed the floating and removed
the fallen tree with it, because a line of separate logs on the ground does
not read as a tree that went over. Break it ONCE and fix the collider.

**Baking was NOT what made pieces float, and blaming it cost a round.**
`bake_result=True` does freeze whatever the pre-roll left mid-air, so it is a
plausible suspect and turning it off does make the symptom move — the pieces
carry on falling instead of stopping. But the cause was the FRAGMENTATION: a
Voronoi cell holding several disconnected branch sections rests wherever its
combined hull lands, and it will do that baked or live. Fix the fragmentation
and baking is free. Keep enough settle steps to outlast the LONGEST fall in
the scene, not the average one, and bake.

**Cut the debris stock ONCE.** Slicing each piece out of the whole 41k-point
bole is fine at ten pieces and minutes of start-up at thirty. Isolate the
low-trunk band and clip it to the trunk column first, then cut every piece
from that — same geometry, a fraction of the work, and it guarantees the
stock is a single connected column before anything else runs.

**Tip it PAST BALANCE, do not lay it flat.** The first topple rotated the bole
84 degrees — already horizontal — and handed physics a piece with nowhere left
to fall, so the resting pose was authored by a formula and read as one
("fallen in an unnatural way"). 38 degrees plus gravity gets a pose the solver
found. Same argument `damage.damage_placements(move_felled=False)` makes about
walls: felling by formula and then simulating is the worst of both.

**Budget settle steps for a real fall.** The house benches use 360 because a
house's pieces start at head height. A bole segment tipped past balance at
13 m has a genuine fall ahead of it, and a settle that stops mid-air BAKES the
piece where it was — which looks exactly like the floating it was meant to
fix.

**The crown is a different prim from the bole, so cutting one leaves the
other in the air.** Snapping a trunk at 7.7 m leaves 747 woody branchlets and
four leaf meshes hanging above the break, because none of them is the mesh
that was cut. `prune_above()` hides everything still standing over the cut,
wood included — fine twigs above a burn-through are gone by definition and are
most of why the top came off.

**Half this pipeline wants a numpy Generator and half wants `random.Random`.**
`vtk_fracture._seeds` calls `rng.uniform(lo, hi, size=...)` and `scorch._noise`
calls `rng.normal()`; `damage`, `settle` and every launch script pass a stdlib
`random.Random`. Both mistakes surface as a `TypeError` or `AttributeError`
tens of seconds into a container launch. `vegetation._nprng()` converts at the
boundary so callers keep one rng.

**Trees STAY UP, and this is the domain fact that matters most.** The
intuition that a burnt tree falls over is wrong on the timescale a capture
cares about: immediately behind a front a burnt stand is a field of standing
black poles, and they come down over months to years as roots rot. A scene
full of downed trunks reads as windthrow — a tornado — not a fire. `fallen` is
deliberately rare in the level mix (~12% of dead trees) and `stump_chance`
rarer still.

**CROWN SCORCH IS REAL AND IT WAS STILL CUT — the scene is the judge.** The
plume kills foliage without burning it: leaves turn orange-brown, stay on the
tree, and a "red belt" is most of what a mixed-severity fire leaves by area.
That argument is correct and it lost on sight. Sixty full crowns standing in a
burnt block read as trees the pass MISSED, whatever colour they are, and no
amount of being right about fire behaviour survives that. Every burnt level
now strips the crown; what still separates `scorched` from `torched` is the
BOLE — charred to a third of its height rather than end to end — plus a third
of the ground debris. Keep the distinction where it costs nothing to read.

**Severity is a property of the STAND, not the tree.** Crown fire carries
between touching canopies, so a dense clump burns to one severity while an
isolated tree with clearance usually gets only scorched. Scattering tree
levels independently across a cluster is what makes a burnt wood look like
noise instead of a fire.

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

**Cache keys must include the recipe.** Retuning `SOOT_RGB` or `char_bite`
silently reused maps baked under the old values, so nothing appeared to change.

---

# THE GROUND: FOUR ATTEMPTS, ALL REJECTED

The burn scar on the ground is the single hardest thing in this pipeline and
NOTHING SHIPPED. Four approaches were built and each was rejected on sight;
the block ships plain grass. This section exists so the fifth attempt starts
where the fourth stopped instead of retracing the first.

`burn_ground_preview_launch_script.py` is the bench: the floor, the fire
field and a row of material configurations, with no houses or trees in the
way. Every one of the failures below was diagnosed slowly inside a
twenty-minute full-block build before that bench existed. Use it.

## What a burn scar actually is

**A fire scar is a MOSAIC**, not a gradient — patchy mottled interior inside
an irregular fingered outline, with unburned islands the fire skipped where
the ground was wetter or something broke the run. No directionality, nothing
like the wash on a wall.

**The fire burns the ground BETWEEN things, not just under them.** A per-tree
burnt ring says only the ground touching each trunk burned, which no fire
does — a surface fire runs over everything and leaves continuous black ground
under and between a stand. What varies is SEVERITY (heavy fuel burns down to
pale ash and bare soil; open grass gets a fast flashy blackening), not
presence. Green strips between cluster trees were the giveaway.

**Ground scorch is not wall scorch.** `soot_mask` stretches its noise 8x
vertically because soot on a wall RISES and smears into licks — that stretch
is the whole character of it. Ground has no up: run the wall pattern over open
ground and you get a field of parallel streaks nothing in the scene explains.
`streak_stretch=1` with `wash_weight=0` leaves pure non-directional mottling.

**The ground has THREE states, not two.** Blending grass straight into bare
burnt floor skips the state most of a real scar is in. The middle one is
SCORCHED GRASS — the grass's own base colour with soot composited onto it,
exactly what the walls get. The line between scorched and bare is fuel
continuity: a stand's crowns and litter feed each other so the fire sits and
burns the ground out completely, while one tree in the open has nothing to
sustain that.

## Attempt 1 — bake the whole plate

Composite grass and burnt ground into ONE image for the entire block
(`scorch.ground_burn_map`), so there is no tiling repeat and the scar can be
painted rather than uniform.

**Why it failed:** resolution and maps. 250 m into 2048 px is 12 cm a pixel
and 320 m into 1024 px is 31 cm — mush at any sane camera height. And a baked
composite is diffuse-only, so the grass loses the normal and ORM maps that
were doing most of the work. It reads as flat paint.

## Attempt 2 — tiling material revealed by a baked MASK

Keep both surfaces at full resolution and tiling at their own scale; bake only
the coverage MASK, which is genuinely low-frequency and loses nothing by being
baked. Reveal burnt ground over grass through that mask as an opacity.

Sound reasoning, and it needs two textures at two different scales — the burnt
ground tiling every ~9 m, the mask stretched exactly once across the plate.
**OmniPBR cannot do that**: it carries ONE `texture_scale` / `project_uvw`
pair shared by every texture it samples. `UsdPreviewSurface` can, because each
texture gets its own `UsdUVTexture` with its own `st`.

**Why it failed:** this renderer translates USD materials to MDL (`UsdToMdl`),
and a `UsdPreviewSurface` whose `opacity` is driven by a texture comes out the
other side WITHOUT it. The plane drew fully opaque over the whole plate — "you
have made the whole ground ash color". No error, no warning.

Two real bugs were found and fixed on the way, both worth keeping:

- **`sourceColorSpace` must be set on every `UsdUVTexture`.** Leaving it at
  `auto` turned the ground white. MEASURE THE SOURCES FIRST — the burnt floor
  averages 0.18 luma with 0.1% above 0.6, so a pale washed-out render cannot
  come from the content and must come from the decode. An sRGB image taken as
  linear renders ~2.5x too bright; a NORMAL map taken as sRGB decodes to
  garbage normals and blows speculars out to white. Diffuse `sRGB`; masks,
  normals and ORM are data and must be `raw`.
- **Do not blend noise ADDITIVELY into a coverage field.** `f * 1.15 +
  (blotch - 0.5) * 0.55` is the obvious way to write "perturb the field", and
  where `f` is zero the noise still contributes up to +0.275 — so ground the
  fire never reached came out speckled with up to 33% burnt overlay across 45%
  of the plate. Perturb the LEVEL SET instead: `(f - (t0 + noise)) / soft`
  gives the same fingered boundary and is identically zero where the field is.

## Attempt 3 — opaque patch geometry

No alpha at all. Irregular polygons with ordinary tiling OmniPBR materials, in
graded rings: grass -> light scorch -> medium -> heavy -> bare burnt floor,
with satellites of each ring scattered across its own boundary so the two
stipple through each other. Uses only the plain-mesh-plus-OmniPBR path that
everything else here proves works.

**Why it failed:** it never stopped looking like patches. The scorched-grass
maps tile visibly (a soot map repeating every ~3 m across a 30 m patch is ten
copies of the same blotches), and the steps between rings read as steps rather
than as a transition however many were added. Band-limiting the noise, three
maps per level and per-patch scale and offset jitter all helped and none of
them fixed it.

Worth keeping from it: `vegetation.scar_patch` wobbles its outline at THREE
frequencies, because one harmonic gives a wavy circle and a circle is exactly
what the eye picks out; and coplanar overlapping quads need a ~1 mm z step or
they z-fight.

## Attempt 4 — banded overlay with a constant opacity

Put the falloff in a per-band `opacity_constant` rather than a mask. That
removes the two-UV-scale problem entirely, so the material can be an OmniPBR —
MDL natively, nothing to lose in translation. Dice the region into cells,
bucket by the fire's own `coverage_at`, one merged mesh per band.

This is the one that follows the FIRE rather than the trees, which is right:
`coverage_at` is the same field that sets every building's damage level, so
the scar is the ellipse the front actually swept.

**Why it failed — DIAGNOSED, from the MDL source in the Isaac image.** The
bands built correctly (5 bands, 3,844 cells, opacities 0.48-0.87) and nothing
drew because OmniPBR turns `opacity_constant` into a FRACTIONAL cutout opacity
(`kit/mdl/core/Base/OmniPBRBase.mdl`: `cutout_opacity = enable_opacity ?
opacity_value : 1.0`), and **RTX Real-Time discards any fractional cutout
unless `/rtx/raytracing/fractionalCutoutOpacity` is on** (Path Tracing:
`/rtx/pathtracing/...`). The "unset `opacity_texture` samples as 0" guess was
wrong — OmniPBR uses the constant whenever the map is invalid. With the
setting on, every band and every square of the six-square row draws,
confirmed on sight. **It must be a command-line flag** — passed through
`SimulationApp(extra_args=[...])`; `carb.settings.set_bool` at STARTUP is too
late (the value is mapped onto a USD render property at ~12 s).

**AND the command-line flag is not enough in a scene that LOADS A STAGE.**
The flag maps onto a render-settings USD property at ~12 s; the mini launcher
then brings up a Pegasus ENVIRONMENT stage, and loading a stage with authored
render settings RESETS that property to its default (OFF), so the overlay is
fully transparent even though the flag was right on the command line. The
bench keeps it because its stage is an empty `new_stage()` with none. Symptom:
"I don't see the ground at all", and toggling Fractional Cutout Opacity off/on
in the RTX settings (Ray Tracing -> Translucency, gated by the Translucency
section's own enable) brings it back. FIX: re-assert `set_bool(...)` for BOTH
the rt and pt keys AFTER the final stage is built, right before the render
loop, then pump a few `app.update()`s — set THAT late it does push onto the
live property. `suburb_mini_wildfire_launch_script` does this; any launcher
that loads an environment must. (`disaster.ground.KIT_ARGS` is the flag list.)

**Attempt 2 was a different bug, also now understood.** `UsdPreviewSurface`'s
`opacity` is NOT a cutout in this renderer: `kit/mdl/rtx/UsdPreviewSurface.mdl`
blends diffuse against TRANSMISSION by it, gated by a hidden
`enable_specular_transmission = false`, so fractional opacity is forced to
1.0 and only `opacityThreshold > 0` (a hard cutout) does anything. The mask
was read; it could never have blended.

**What it took to make it SHIP (`disaster/ground.py`, judged good on sight):**

- ONE tile projected across the whole overlay (`GROUND_TILE_M=0`): the burnt
  floor tiled at ~8 m read as a grid of small squares from altitude. 12 cm/px
  up close is the price.
- Opacity 0.08-0.50 over 12 bands at 3 m cells; 0.14-0.92 read as an ash
  sheet.
- **The hard edge was the FIELD, not the ellipse.** `coverage_at` stepped
  from 0 to 0.45 at the arrival line. `feathered_coverage` ramps over
  `edge_m` (10% of the plate) PERPENDICULAR to the front —
  `(elapsed - t) / |grad t|` by central differences; measuring along the ray
  from the origin gave ~10 m on a flank because the ray meets it obliquely —
  wobbles the line by +-0.8 edge of 25-80 m band-limited noise, and removes
  6% of the area as 20-60 m islands (an 8-25 m island band thresholded that
  hard was one-cell confetti). Noise only MOVES the boundary or REMOVES
  coverage, so clean ground stays clean.
- **"Front past the far corner" burns the whole plate and leaves nothing to
  judge** — what was being judged as "the ellipse" was the opacity bands.
  The bench now stops at a burnt-area quantile (`GROUND_BURNT_FRAC=0.55`).
- The overlay runs 60 m PAST the plate on the bench, because the ignition
  point sits on the plate corner and an overlay clipped to the plate cut the
  scar square there.
- It sits between grass and asphalt on the z ladder, so roads, drives and
  walks come through unburnt (correct), and pool holes are skipped (you
  cannot scorch water). `generate_suburb_on_stage(info_out=...)` exposes the
  region, pool rings and ladder factor for it.

**The z ladder was always at its 0.08 floor.** `apply_ground` took
`max(region[0], region[1])` as the span — the two MINIMUM corners of an
`(x0, y0, x1, y1)` region, negative on a centred plate — so the scale
clamped to 0.08 for every plate size, and "1600 m reproduces the tuned
values" was never true. `suburb_scene.ground_z_scale` uses the extents;
the 250 m block's ladder is now 0.156x (asphalt 1.6 cm over grass, was
0.8 cm).

Still a translucent overlay: the burnt map's normals do not blend with the
grass, and up close the single tile is soft. The in-material blend below
remains the better long-term answer; this one is what the scene ships with.

## If there is a fifth attempt

Blend INSIDE one material rather than stacking two. `OmniSurfaceBlend.mdl`
(shipped in `kit/mdl/core/Base/`) takes two generic `material`s, mixes BSDFs
with `df::weighted_layer`, normals with `add_detail_normal`, and samples its
weight image through its OWN uvw block (world + `projection_planar` + own
scale) — the two-scales-in-one-material thing OmniPBR cannot do. Feed it
`scorch.burn_mask_map` (written, level-set-correct, never called) as the
weight, and add world-space `base::perlin_noise_texture` /
`base::worley_noise_texture` terms in the shader for mottling and islands
that cannot repeat. The AEC grass is already a hand-written MDL wrapping
OmniPBR (`Grass_Cut.mdl`), so a custom module referenced from a `.usda`
wrapper is the pack's own proven path. A per-vertex `primvars:burn` read
with `scene::data_lookup_float` is the alternative to the baked mask.

---

# The bug catalogue, continued

## Scene / config

**`open_tree_clear_m` is sized for the SMALLEST crown in the pool.** The
planting keep-outs clear the TRUNK by that distance — the code documents the
3.0 m default as "a crown radius at the small end of the pool" — so the whole
model assumes every species in the pool is about that wide. Drop a Black_Oak
(25.4 m crown) in and its trunk clears the house by the required 3 m while its
canopy sits on the roof. That is what "trees spawning too close to structures"
looks like, and the generator's keep-outs are working exactly as written. Fix
it by matching the pool to the clearance, not by widening the clearance until
a park specimen fits.

**The container has `SCENE_CONFIG=downtown` in its environment**, which
silently beats a launch script's own default. Symptom: a "250 x 250 m" scene
generating 83 houses and 3,284 trees. The only tell is the
`[compile_disaster] compiled high-level spec` line naming the wrong preset —
read it on every run, and prefix the relaunch with the config you want.

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
| tree `fall_chance` | 0.12 | a burnt stand is STANDING; more downed trunks reads as windthrow |
| tree `consume` (trunk) | 0.05 | the wall default of 0.30 punches holes through a standing stub |
| bole scorch height, `scorched` | 0.34 | black bottom / clean top is the surface-fire signature |
| crown `keep_top`, `torched` | 0.06 | a crown at exactly zero looks deleted rather than burnt |
| snag break height | 0.34 | 0.45 detached only fine upper branches — twigs on the ground, no timber |
| bole `wash_weight` | 0.0 | the gradient is the only non-seamless term; on a tiling bark map it bands |
| topple lean | 38 deg | past balance, not flat — gravity finds the pose |
| topple segments | 4 | one hull per segment; a whole branching bole hulls to a 20 m blob |
| `consume_thin` keep | 0.16 of max pts | by POINT COUNT — a Voronoi cell's bbox measures the cell, not the wood |
| debris piece length | 0.14-0.42 m at `scorched` to 0.30-2.10 m at `fallen` | one recipe for every level put metre-long split logs under lightly damaged trees |
| debris length bias | `random() ** 2.2` | real debris is mostly small with a few large pieces, not a uniform spread |
| settle steps (trees) | 420 | a segment tipped at 13 m has a real fall; stopping early bakes it mid-air |

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

# Baking damage to disk, and why it is the only thing that scales

Fracture (CPU/trimesh) and settle (GPU PhysX) are the entire cost of a burnt
block — the 250 m mini takes ~1660 s (27 min), almost all of it there — and
they do NOT scale to a full plat. The fix is `disaster.bake`: do that work
ONCE, export each damaged house/tree to a self-contained USD, and REFERENCE it.
Measured on the mini: 317 objects / 41,459 meshes exported in ~30 s, and a
reload that references them all loads in **6.5 s — ~255x faster than rebuilding**.

## The trap: `assetInfo` poisons Sdf.CopySpec AND stage.Flatten()

The obvious exporter — `Sdf.CopySpec` of each object's subtree, or one
`stage.Flatten()` — DOES NOT WORK on the kit house/tree modules. Every kit
mesh, GeomSubset and Material carries an `assetInfo` metadata dict whose value
is a crate type core USD cannot unpack:

    Usd_CrateFile::_UnpackValue ... unsupported type enum value 0

and READING, COPYING, CLEARING or OVERWRITING that field all raise — so any
CopySpec/Flatten that touches such a prim dies. Freshly authored fracture
FRAGMENTS have no `assetInfo`, which is the whole reason an early version
"worked on fragments and failed on everything else". Kit's own renderer and
`export_as_stage_async` handle it; core USD does not.

## What works: rebuild BY VALUE

`bake.export_object` never touches prim metadata. `UsdAttribute.Get()` reads
every geometry attribute fine (it does not unpack `assetInfo`), so it:

- authors fresh meshes from the read-back points/normals/uvs/subsets,
- bakes each mesh's WORLD transform onto it (`XformCache`), so the hierarchy
  flattens and a plain reference lands the object where it was,
- rebuilds each material as a fresh `Material` shell (no `assetInfo`) whose
  Shader children ARE CopySpec-able, remapping bindings and connections to the
  new paths,
- binds any UNBOUND mesh to the object's own bark/wood material — Black_Oak's
  branchlet prototype ships with no binding and would otherwise render grey
  (the same asset defect `vegetation.bind_bark` covers at build time).

Kit meshes bind materials PER-SUBSET (`Section0`/`Section1`), not at the mesh
level; `validate()` checks the subsets, and they resolve through a reference.

## Flatten with Kit, slice with USD

To get a stage core USD can read from at all, flatten with KIT first
(`omni.usd.get_context().export_as_stage_async(tmp)`), then open that file and
run the by-value export off it — `export_as_stage` inlines geometry and
normalises what it can, and reading attribute VALUES off it is safe.

## Two entry points

- **In-process (exact grouping):** `suburb_mini_wildfire_launch_script.py` with
  `MINI_BAKE_DIR=<dir>` — after the full build it exports each house
  (cluster items + `brk_*` fragments) and tree (`+ tree_debris`) using the
  launcher's own `per_building` / `trees` groupings, writes `manifest.json`,
  and keeps the app live even if the bake fails.
- **Offline (no rebuild):** the launcher's Kit-flatten leaves `_flat_tmp.usd`
  on disk; `bake.export_object` runs against it with STANDALONE pxr (no app):

      P=/isaac-sim/extscache/omni.usd.libs-*/; \
      LD_LIBRARY_PATH=$P/bin PYTHONPATH=$P:.../scene_gen \
      /isaac-sim/kit/python/bin/python3 offline_bake.py

  Groupings are reconstructed by clustering house/`brk_` prims by position and
  name-matching `debris_tree_XX_YY` to its tree. This is how the 255x number
  was produced without paying a second 27-min build — invaluable for iterating
  on the exporter (each test is seconds, not half an hour).

`suburb_reload_launch_script.py` (`MINI_RELOAD_DIR=<dir>`) references every
baked object from the manifest onto a fresh stage and prints the load time.

## Per-ARCHETYPE assembly — the 1600 x 1200 plat, IMPLEMENTED

The mini bake above is per-INSTANCE. The full-plat win is per-ARCHETYPE, and it
is built:

- `bake_archetypes_launch_script.py` (`ARCH_DIR=<dir>`) builds every house
  (8 styles x 6 levels) and tree (6 species x 5 burnt levels) ONCE on a
  spread-out grid, fractures/burns each with the SAME code the live scene uses
  (`disaster.damage_flow.damage_building`, `disaster.vegetation.burn_tree`),
  settles the whole grid in one pass, and exports each object RE-CENTRED to the
  origin (`bake.export_object(recenter=(X, Y, 0))`). Measured: 78 archetypes,
  110 MB, ~570 s, 0 unresolved. `damage_flow` is the mini launcher's
  per-building loop extracted so the wreckage is identical.
- `suburb_scene.generate_suburb_on_stage(assembly=True)` builds the CHEAP layer
  live — streets, ground, drives, walks, fences, props — and RETURNS each
  house's (style, pose) in `info_out["house_instances"]` and each tree's
  (species, pose) in `["tree_instances"]` INSTEAD of building their geometry
  (`build_placements(house_instances=...)` records-and-skips; the tree
  placements are pulled out before `apply_placements`).
- `suburb_assemble_launch_script.py` references `house_<style>_<level>.usd` /
  `tree_<species>_<level>.usd` at each instance's pose and fire-derived level,
  builds the ground scar with `disaster.ground`, and marks every reference
  `SetInstanceable(True)` so repeated archetypes SHARE geometry.

Measured on `suburb_wildfire` (1600 x 1200): 504 houses + 9,465 trees
referenced (0 missing) in ~450 s total (236 s of it the layout), 10.4 GB GPU,
NO OOM. The live fracture/settle pipeline does not reach this size at all.

Gotchas found the hard way:
- **Instance the references or the trees OOM.** 9k green/burnt trees
  un-instanced is the 186M-point wall again; `SetInstanceable(True)` on the
  reference root (transform ops there are still legal) keeps it to ~10 GB.
- **Do not shadow a module alias with a local.** A local `sp = ...` inside
  `generate_suburb_on_stage` made the module-level `from detail import
  suburb_parcel as sp` local for the WHOLE function and an earlier `sp.DENSITY`
  raised `UnboundLocalError`. Name assembly locals `_species` etc.
- **Tree variety is thin without `stand_outcome`.** `veg.level_for_age` alone
  returns `snag` for almost every burnt tree (9,157 of 9,465). The mini adds
  `veg.stand_outcome` for the fallen/torched mix; the assembly should too. And
  the burn currently covers most of the plat — tune `elapsed`/`duration_s` for
  a clear unburnt fraction.

Collapse geometry is independent of wall colour, so palette and scorch stay
runtime binds and never multiply the ~78 geometry bakes.

# Known gaps

- **The ground scar is a translucent overlay** (`disaster/ground.py`,
  `MINI_GROUND=0` disables it). Diffuse-only, one soft tile; the in-material
  blend described under "If there is a fifth attempt" is the upgrade path. `scorch.ground_burn_map` and `scorch.burn_mask_map`
  both still exist and neither is called by anything.
- **Road line materials are imported but unbound** — lane dashes use
  `displayColor` with no material by design, so they need explicit rebinding.
- **No burnable props exist** in the mini block (categories are only `fence`,
  `house_*`, `plot_pool*`), so the prop-burning pass has nothing to act on.
- **Mesh foliage cannot be thinned, only switched off.** Aspen, Beech and
  Apple carry one `leaves` mesh with no per-instance ids, so in a graded
  cluster they come out either fully leafy or fully bare beside properly
  thinned instanced neighbours. Fix is the same slice trick the bole uses:
  cut the leaf mesh into height bands and hide bands.
- **A toppled tree's crown is pruned, not carried down.** Rotating a
  PointInstancer's `positions` is easy; its per-instance `orientations` are
  quaternions and getting them subtly wrong scatters twigs at impossible
  angles down the fallen trunk.
- **Tree ash rings are opaque discs with a hard edge.** Only the tree bench
  draws them; the block does not. They have the same unsolved problem as the
  ground scar and should wait for the same answer.
- **`snap` and `topple` never fire in the block.** Every tree lands in
  `scorched` or `torched`, because `level_for_age` only reaches `snag` after
  the full flame AND smoulder phases and the block's clock does not get there.
  Both are exercised only in the tree bench. Raise `MINI_ELAPSED` to see them.
- **Seated debris still carries a collider.** ~12,000 pieces are static
  geometry the solver never touches, and each is still cooked as a triangle
  mesh — 36,734 static colliders on the last run. Nothing will ever hit them;
  they could skip collision entirely.
- **GPU PhysX is on but unmeasured.** `settle.prepare(gpu=True)` sets
  `enableGPUDynamics` and the capacities, and `run` prints `Xs solving (GPU)` —
  168.6 s for 5,479 bodies over 1,400 steps on the last block. No CPU baseline
  was captured before the switch, so whether it helped is still unknown.
- Fragments are cut from the module's outer shell, so they are hollow inside.
- Fracturing at scene-build time will not scale to a full plat; bake fractured
  results to disk and reference them.
