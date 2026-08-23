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
| `scene_gen/disaster/fracture.py` | Voronoi mesh fracture, partial (ragged) breaks, fragment consumption |
| `scene_gen/disaster/settle.py` | PhysX drop + bake to static, with drop/spread measurement |
| `scene_gen/disaster/scorch.py` | composites soot onto a surface's own texture; bakes ground burn scars |
| `scene_gen/disaster/rubble.py` | procedural convex debris (superseded by real fracture; kept for comparison) |
| `scene_gen/tools/burn_textures.py` | generates the char / scorch / ash maps |
| `scene_gen/tools/import_megascans.py` | Megascans/glTF surface zip -> bound `.usda` material |
| `scene_gen/tools/fire_png.py` | host-side burn plan, no Isaac Sim needed |

Launchers: `house_damage_test_launch_script.py` (severe bench),
`house_partial_damage_test_launch_script.py` (partial bench),
`tree_damage_test_launch_script.py` (six burnt-tree methods on grass),
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
`fracture._seeds` calls `rng.uniform(lo, hi, size=...)` and `scorch._noise`
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

**A fire scar on the ground is a MOSAIC**, not a gradient — patchy mottled
interior inside an irregular fingered outline, with unburned islands the fire
skipped. No directionality, nothing like the wash on a wall.

**The fire burns the ground BETWEEN the trees too.** A per-tree burnt ring
says only the ground touching each trunk burned, which no fire does — a
surface fire runs over everything and leaves continuous black ground under and
between a stand. What varies is SEVERITY (heavy fuel burns down to pale ash
and bare soil; open grass gets a fast flashy blackening), not presence. The
green strips between cluster trees were the giveaway.

**OPAQUE GEOMETRY CANNOT FADE, SO BUILD THE TRANSITION OUT OF STEPS.** One
boundary carrying the whole change from green grass to bare burnt ground reads
as a cut-out however good its outline is. Four levels — light, medium, heavy
scorch, then burnt floor — spread that change so no single edge carries it.
And INTERLEAVE each boundary rather than drawing it: scatter satellites of the
inner material out past the edge and satellites of the outer one in behind it,
so the two stipple through each other over a couple of metres. That is how an
opaque renderer gets a blend.

**One material on one big patch is a wallpaper.** A soot map tiling every ~3 m
across a 30 m patch is ten copies of the same blotches, and the eye finds them
instantly — this is the same "low-frequency contrast is what makes a tile
visible" finding as the burn textures, arriving from the other direction. It
needs all three fixes together: BAND-LIMIT the noise so there is no feature
large enough to recognise, generate SEVERAL maps per level from different
seeds, and vary the projection scale AND world offset between neighbouring
patches so nothing lines up.

**THE GROUND HAS THREE STATES, NOT TWO.** Blending grass straight into bare
burnt floor skips the state most of a real burn scar is actually in, and reads
as a texture swap rather than as fire damage. The middle state is SCORCHED
GRASS — the grass's own base colour with soot composited onto it, exactly what
the walls and the boles get — and it is both the transition INTO bare ground
and the only treatment a lone tree gets at all.

The line between them is fuel continuity: a stand's crowns and litter feed
each other, so the fire sits in it and burns the ground out completely, while
one tree in the open has nothing to sustain that and the grass under it chars
and survives. So bare burnt floor goes under CLUSTERS, nested INSIDE a wider
scorched ring, and a single tree gets scorch only.

**Ground scorch is not wall scorch.** `soot_mask` stretches its noise 8x
vertically because soot on a wall RISES and smears into licks — that stretch
is the whole character of it. Ground has no up: run the wall pattern over open
ground and you get a field of parallel streaks nothing in the scene explains.
`streak_stretch=1` with `wash_weight=0` leaves pure non-directional mottling,
which is what a burnt patch of grass is.

**BAKE THE MASK, NOT THE GROUND.** `ground_burn_map` bakes grass and burnt
into one plate-sized image, which kills the repeat but pays twice: 320 m into
1024 px is 31 cm per pixel — mush at any sane camera height — and the result
is diffuse-only, so the grass loses the normal and ORM maps doing most of the
work. It looked terrible. A coverage mask is genuinely low-frequency and loses
nothing by being baked, so bake ONLY that (`scorch.burn_mask_map`) and let a
burnt-ground material TILE over the grass at its own scale, revealed by the
mask as an opacity. Both surfaces keep full detail and all their maps.

**SET `sourceColorSpace` ON EVERY `UsdUVTexture`.** Leaving it at `auto` is
what turned the burnt ground white. The diagnostic is worth copying: MEASURE
the sources first — the burnt floor averages 0.18 luma with 0.1% of it above
0.6, and every other map here is darker still, so a pale washed-out render
cannot be coming from the texture content and must be coming from the decode.
An sRGB image taken as linear renders about 2.5x too bright; a NORMAL map
taken as sRGB decodes to garbage normals and blows the speculars out to white.
Diffuse is `sRGB`; masks, normals and ORM are data and must be `raw`.

**Blend noise ADDITIVELY into a coverage field and you paint the whole map.**
`f * 1.15 + (blotch - 0.5) * 0.55` is the obvious way to write "perturb the
field", and where `f` is zero the noise still contributes up to +0.275 — so
ground the fire never reached came out speckled with up to 33% burnt overlay
across 45% of the plate. Perturb the LEVEL SET instead — `(f - (t0 + noise))
/ softness` — which gives the same fingered boundary and is identically zero
wherever the field is, because nothing crosses a threshold it was never near.

**A MASKED TRANSLUCENT OVERLAY DOES NOT SURVIVE THIS RENDERER — use geometry.**
The overlay needs two textures at two scales (burnt ground tiling every ~9 m,
mask stretched once across the plate), and OmniPBR carries ONE `texture_scale`
/ `project_uvw` pair shared by every texture it samples — so it cannot do it,
and `UsdPreviewSurface` can. That is where it ends, because Hydra here
translates USD materials to MDL (`UsdToMdl`) and a UsdPreviewSurface whose
`opacity` is driven by a texture comes out the other side WITHOUT it. The
plane then draws fully opaque over the entire plate: "you've made the whole
ground ash color".

MEASURE BEFORE REDESIGNING — it is what separated this from the colour-space
bug that looked identical. The scorched-grass map came out at 0.157 luma with
nothing above 0.5, and the mask at 87.8% fully transparent. Neither can
produce a white ground, so the fault was not in what was authored but in
whether it was applied at all.

The working answer is PATCH GEOMETRY: irregular polygons with ordinary tiling
OmniPBR materials (`vegetation.scar_patch`). No alpha, no second UV set, only
the plain-mesh-plus-OmniPBR path everything else here already proves works.
Give each patch a ~1 mm z step, since coplanar overlapping quads are what
z-fighting is. And a real fire scar edge is fairly sharp — a metre or two, not
a long fade — so a hard irregular boundary is closer to the truth than a soft
one. Wobble the outline at THREE frequencies: one harmonic gives a wavy
circle, and a circle is exactly what the eye picks out.

**Cache keys must include the recipe.** Retuning `SOOT_RGB` or `char_bite`
silently reused maps baked under the old values, so nothing appeared to change.

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

# Known gaps

- **`scorch.ground_burn_map` still never runs in the mini scene** — the guard
  looks for `ground/ground_base` and the ground is built as per-block meshes
  (and `ground_base_N` when pools are cut). But prefer `burn_mask_map` +
  a translucent overlay to it now; see the ground note in the catalogue.
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
- **Tree ash rings are opaque discs with a hard edge.** They want the same
  treatment as the ground burn scar (baked, fingered, with unburned islands),
  which is the top outstanding item above.
- **Vegetation is not wired into `suburb_mini_wildfire` yet.** `tree` is one
  of `suburb_scene`'s `instance_categories`, so burnable trees have to be
  un-instanced there exactly as the fences were.
- Fragments are cut from the module's outer shell, so they are hollow inside.
- Fracturing at scene-build time will not scale to a full plat; bake fractured
  results to disk and reference them.
