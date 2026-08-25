---
name: build-tornado-scenes
description: Build or modify TORNADO-damaged scenes in scene_gen — the track field, directional debris, sawn-plank wreckage, windthrown trees with their leaves on, and the mud scour along the path. Read before touching disaster/tornado.py, disaster/wind_flow.py, disaster/planks.py, vegetation.wind_tree, or the suburb_tornado launchers. The wildfire skill is the prerequisite; this one is mostly about what is DIFFERENT and why the fire code cannot be reused as-is.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Build Tornado / Wind-Damage Scenes

## Read `build-wildfire-scenes` first

This pipeline is the wildfire one with a different driver, and it inherits
every one of that skill's traps unchanged: the Flow/fracture/settle/material
mechanics, the ground-overlay history (four rejected attempts, the
fractional-cutout flag, the re-assert-after-stage-load rule), the
`assetInfo`-poisons-CopySpec bake trap, the instanced-prims-cannot-be-authored
rule, and the whole bake-and-assemble scaling argument. None of that is
repeated here. **This file is only about what a tornado does that a fire does
not**, and about the places where reusing the fire code silently produces the
wrong scene.

`tornado.jpeg` in the repo root is the reference photograph. Every design
decision below is traceable to something in it.

## The pipeline

    layout -> generate -> [bake once per archetype] -> assemble -> planks -> scour

Two launchers, in this order:

| launcher | what it does | cost |
|---|---|---|
| `bake_tornado_archetypes_launch_script.py` | every (house style x damage level) and (tree species x wind level) built, wrecked, settled and exported ONCE | minutes, once |
| `suburb_tornado_launch_script.py` | the 500 x 500 scene: layout live, houses/trees BY REFERENCE, plank field and mud scour authored | seconds |

Host-side, no Isaac Sim needed, and **run it before every container launch**:

    python3 scene_gen/tools/tornado_png.py --config suburb_tornado

It runs the real field and the real ladders and prints the four numbers that
decide whether a scene is worth building, plus a plan PNG.

## The one number that drives everything

The wildfire pipeline is driven by **burn age** — time since an elliptical
front arrived — and `damage.level_for_age` turns it into a level, a finish and
a fire state together. The tornado pipeline is driven by

    intensity(x, y)   0..1, an EF proxy

and it is a function of **distance from a LINE**, not from a point. That one
change is most of what makes the two scenes look nothing alike, and scattering
anything independently of it is what makes a scene read as four unrelated
effects instead of one event.

## The modules

| file | does |
|---|---|
| `scene_gen/disaster/tornado.py` | the track: frame, meander, intensity field, throw field, both damage ladders, the scour coverage |
| `scene_gen/disaster/wind_flow.py` | wreck one building — the no-fire counterpart of `damage_flow.py` |
| `scene_gen/disaster/planks.py` | the sawn-timber debris field: stock list, comet scatter, merged box meshes, plain wood material |
| `vegetation.wind_tree` + `tip_tree` + `root_plate` | windthrown trees, leaves ON |
| `scene_gen/tools/tornado_png.py` | the host-side plan |

Shared with the fire path, unchanged: `fracture` (plus a new `plank` seed
mode), `settle` (plus a `bias`), `ground.build_overlay`, `bake`.

---

# What is different, and why the fire code cannot just be reused

## The scene is PALE, not dark

A fire consumes and blackens. Wind BREAKS, and breaking exposes the inside of
everything it breaks — framing, sheathing, the unpainted back of siding, fresh
splintered wood. A tornado debris field photographs **light against green
grass**; a burn scar photographs dark. Getting this backwards is the fastest
way to build the wrong disaster, and it is why `planks.WOOD_BASE` is
`Ash_Planks` (mean luma 0.61) rather than anything in `assets/materials/burn/`.

## `damage.INCOMBUSTIBLE` IS THE WRONG LIST, and half of it inverts

That tuple says what does not BURN. It contains `streetlight` and `sign`
*because* metal street furniture comes through a wildfire visibly untouched —
and those are exactly the things a tornado shears off and throws. Reusing it
leaves a corridor of levelled houses with every street sign standing perfectly
upright in it. `wind_flow.IMMOVABLE` is the right list: what survives is what
is IN THE GROUND or full of water — `pool`, `water`, `manhole`,
`storm_drain`, `slab`, `foundation`.

## Nothing is consumed — except at `swept`, where it means something else

`damage_flow` passes `consume=0.55, consume_pool=1.02`: over half the
fragments are never authored, biased hard toward the LARGEST, because a timber
building genuinely burns away and a big surviving panel reads as "a wall that
fell over" rather than as debris. **A tornado conserves material and displaces
it**, and the big recognisable pieces are precisely what makes a track read as
a track. `wind_flow.BREAK_PLAN` is `consume=0` up to `leveled`.

The exception is `swept` (0.62), where a high consume is correct and means the
opposite thing: at EF4-5 the material really is gone from the lot, because it
is downwind. `planks.scatter_from_wreck` is where it went.

## Every material pass in `disaster.damage` is a fire pass with no off switch

This is worth stating flatly because the obvious fix does not work.
`damage.scorched_material`'s `level` indexes `damage.SOOT_LEVELS`, whose
LOWEST entry is 0.52 coverage — **there is no bucket below "half sooted"**, so
the composite path cannot be asked for clean timber at any argument. Likewise
`soot_materials` composites a soot wash over every surface it walks with no
parameterisation that turns it off, and `char_materials` reads
`assets/materials/burn/*`.

So a no-fire scene calls **none** of `char_materials`, `char_placements`,
`scorched_material`, `soot_materials`, or `bucket`. What it does use, because
both are completely fire-agnostic and essential:

- `damage.bound_texture(stage, path)` — the module's cladding texture, read
  BEFORE fracturing (a prim handle does not survive its own subtree being
  deactivated; a string does).
- `damage._pbr(...)` — the material primitive. Given a texture it turns on
  world triplanar, which UV-less fragments require.

`vegetation.plain_wood()` exists for the same reason on the tree side.

## The `_seeds` mode is `plank`, and it is a lattice

A Voronoi diagram of a **regular lattice** is exactly a grid of boxes: for a
rectangular lattice the bisectors are three families of parallel planes. So
anisotropic lattice SPACING gives anisotropic BOXES. `mode="plank"` spaces the
lattice ~5x longer along the grain than across it and jitters by 0.18 of the
cell pitch.

This is the OPPOSITE construction to `mode="char"`, which is also a lattice
but a dense-along / sparse-across one, because a crack pattern in charred wood
runs *across* the grain (Bartlett et al., arXiv 1604.01249). A wall in a wind
does not craze — it DELAMINATES along its fasteners into the sections the mill
cut.

## A REGULAR LATTICE MAKES IDENTICAL CELLS — that is the whole point and it is
## also the problem

Reported off the first assembled scene, verbatim: *"in the debris there's a
lot of the roofplate for some reason. It's very repetitive and looks weird."*

It is not a tuning miss, it is the construction. A lattice of N sites gives N
cells of about one lattice volume EACH, and jitter perturbs a cell's POSITION
rather than its SIZE — so a 5 x 5 x 0.1 m roof plate at `counts=[2, 12, 1]`
came apart into two dozen boards identical to within a few centimetres. The
same mechanism that makes the cells rectangular makes them interchangeable.

Three things fix it, and they are independent:

- **DROP ~30% OF THE LATTICE SITES.** This is the one that matters. Every cell
  adjacent to a hole expands to absorb it, so the output is a MIX — single-
  width boards, double-width ones, the occasional big panel where two holes
  fell together. It is also what a building actually sheds: sheathing tears
  along some fastener lines and not others, so a real debris field is mostly
  standard sections with a minority of larger pieces still joined. Build the
  lattice for `n / keep` sites so the surviving count still lands near what
  the caller asked for.
- **DRAW THE ASPECT RATIO PER MODULE, and by module TYPE.** One `ar` for the
  whole building gives every module the same board proportion. `aspect` is a
  (lo, hi) range and `_seeds` draws inside it per module: a roof sheds SHEETS
  (1.3–2.6), a floor mid (2.4–4.8), a wall long framing (3.5–7.0). Giving the
  roof the wall's ratio is exactly what produced the identical strips.
- **VARY THE MATERIAL, and give roofs their underside.** Bare fragments all
  taking one pale timber removed the last thing separating one piece from the
  next — they now draw across the whole `planks.STOCK` list, which spans
  near-white framing to dark decking. And a roof's own cladding is shingle, so
  twenty roof fragments is twenty identical dark slabs; half a torn roof lands
  face-down on its bare deck, so roof fragments take timber 18% more often.

Jitter went 0.18 → 0.34 alongside, but on its own it is not enough — it moves
the cells, it does not resize them.

## Three settings have to move with `mode="plank"`

Missing any one produces nothing, with no error:

- **`rough` down to ~0.010.** 0.045 on a thin board visibly bows it, and a
  snapped stud has crisp sawn faces with one splintered end.
- **`min_volume_frac` down to 0.0008.** The cull compares a fragment's
  BOUNDING-BOX volume against a fraction of the module's, which is generous to
  a stubby chunk and hostile to a sheet — a 2.4 x 1.2 x 0.015 m panel has a
  hundredth the bbox volume of a cube with the same longest edge. The 0.004
  default discards exactly the broken sheets this field is made of. It is now
  exposed on `fracture_prim` / `fracture_partial` for this.
- **`wood_debris(min_aspect=...)` down to ~1.25** on the tree side. The
  aspect gate rejects anything stubbier than `min_aspect:1` and a board is
  wide by definition, so a caller asking for wide pieces and leaving the gate
  at 2.0 burns its whole `n_pieces * 6` attempt budget and returns an empty
  list, silently.

## DO NOT COPY THE FIRE BAKE'S `convexDecomposition`

The wildfire archetype bake passes
`approx_map = {p: "convexDecomposition" for p in tree_loose}`, and it needs
to: the burnt path TOPPLES a bole, and a whole branching trunk hulls to a
20 m blob, so it comes to rest balanced on its own limb tips and reads as
floating.

**The wind path never topples anything.** A windthrown tree is `tip_tree` — a
transform on the intact prim, simulated by nothing — so the only loose tree
geometry is debris sticks and `fell_branches` limbs, and a stick is convex to
within its own bark. Carrying the map across cost **20+ minutes on a settle
that takes 61 seconds with plain hulls**: the limbs are branchy enough to hit
`ConvexDecompositionTask: polygon limit reached`, and the hull sets that come
out of that make every solver step crawl.

**It does not look like a bug.** There is no error and no traceback; the
process sits at ~30% of one core, logs nothing at all, and the last line in
the Kit log is that `polygon limit reached` info message. That line is the
entire diagnosis. The check that tells slow from deadlocked is
`ps -eo pid,pcpu,comm` inside the container — burning CPU means slow, idle
means wedged.

## Waiting on a relaunch: PIN THE LOG FILE

A harness that waits for a banner by grepping
`$(ls -t "$KITLOGDIR" | head -1)` has a race that produces FALSE SUCCESS.
Between sending a relaunch to the pane and the new process creating its own
log, the newest file is still the PREVIOUS run's — which already contains the
banner being waited for. The waiter returns instantly, the chain advances,
and it reports a step finished that has not started. Capture the log path once
after the relaunch (with a sleep long enough for the new process to create
it) and grep that fixed path.

## `settle` needed a `bias`, and TWO ceilings eat it if you forget them

`kick` is deliberately zero-mean — it exists only to break the perfect
interlock of a Voronoi partition so gravity can take hold. A scene settled
with it alone drops every piece into its own footprint, which is right for a
fire and wrong for wind. `settle.prepare(bias=(bx, by, bz))` adds a constant
velocity, and:

- `maxLinearVelocity` defaults to **4.0 m/s**. A 9 m/s throw authored against
  it comes out as 4 m/s and the debris lands under half as far — which looks
  like the bias "not working" rather than like a clamp. Pass `max_speed`.
- `linearDamping` defaults to **0.55**, bleeding better than half the speed
  away per second. Pass `damping` (0.16 is what the bake uses).

The bias also carries a small **upward** component (`throw_bias` uses
`0.35 * speed`). With a purely horizontal launch every fragment ploughs
straight into whatever is beside it and stops a metre out, which reads as a
collapse with a limp.

**The `[settle]` spread diagnostic inverts.** "spread mean … (large =
exploding)" is the explosion warning for a collapse and the SUCCESS criterion
for a wind event. `settle.run` now labels it by whether a bias was given; a
tornado settle reporting 0.6 m of spread threw nothing.

## Trees: `burn_tree` CANNOT be used, and it is one branch

`burn_tree`'s stage 4a is gated on the LEVEL NAME, not on an argument:

    if level in ("scorched", "torched", "snag", "fallen", "stump"):
        strip_to_trunk(...)

and `strip_to_trunk` deactivates **every PointInstancer** plus every mesh named
branch/leaf/needle/twig/foliage. No parameter reaches it. Since "the leaves
stay on" is the entire difference between a windthrown tree and a burnt one,
`vegetation.wind_tree` is a separate entry point with the same return shape.

**`prune_above` is the second place leaves die.** Both `snap` and `topple`
call it, and it deletes every instance above the cut, wood and leaf alike.
`wind_tree` uses `snap` only for the `snapped` level, where losing the crown is
correct (a snapped bole's crown is somewhere else entirely).

### `tip_tree` is the cheapest correct thing in the whole pipeline

A windthrown tree is the **same tree, rotated** — crown, instancers, materials
and all. So there is no mesh surgery, no fracture, no physics and no
per-instance quaternion arithmetic: rotate the PARENT prim about its own local
origin (which on every tree in this library is the base of the trunk) and the
whole crown comes along. `topple` exists because the burnt case cannot do this
— its docstring records that rotating a PointInstancer's own `orientations`
"scatters twigs at impossible angles". Nothing here touches them.

Two things that are not obvious:

- **The op order must be rebuilt as translate → orient → scale.** Rotation has
  to come after scale in *application* order (before it in the list) or the
  tree is scaled along the tilted axes and comes out sheared. `tip_tree` reads
  the existing ops BY NAME rather than by position, because a tree may have
  been placed by `apply_placements` (translate + rotateZ + scale) or by a bake
  grid (translate + scale).
- **Lean is an OUTPUT, not an input — measure it.** Flat (90°) puts the bole
  IN the ground and reads as a log somebody placed. Worse, rotating about the
  base drives whatever was on the downwind side of the trunk *under* the lawn,
  and how far under is set by the CROWN RADIUS, which in this pool spans 6 m
  (Douglas_Fir) to 25 m (Black_Oak). One fixed lean is therefore wrong for
  every species but the one it was tuned on. Measured at lean 77 with a 1.4 m
  lift, the lowest point of each tree:

  | species | min z |
  |---|---|
  | Douglas_Fir | −0.12 m (barely touches) |
  | Largetooth_Aspen | −0.98 m |
  | American_Beech | −0.47 m |
  | Shumard_Oak | **−3.81 m** |
  | Black_Oak | **−11.95 m** — buried to mid-canopy |

  The physics behind the fix is the wildfire skill's own observation read from
  the other side: *a fallen tree with its branches on does not lie down*. Its
  limbs hold it clear of the ground, and that is not a bug — "the tree really
  is resting on its branch tips because nothing broke them". A fire-killed
  tree loses those limbs and drops flat; a windthrown tree **in leaf** keeps
  them and comes to rest propped on its own crown, 20–30° above horizontal.

  So `tip_tree(seat_band=(lo, hi))` treats `lean_deg` as a MAXIMUM and
  bisects down until the tree's lowest point lands just into the turf
  (−1.1 to −0.15 m). `min_z` falls monotonically with lean, so plain bisection
  converges in a handful of `BBoxCache` reads. **Build a fresh `BBoxCache` per
  measurement** — it is a cache, and reusing one across a transform change
  hands back the pre-change bound, so the bisection converges on a number that
  is not what is on the stage. Result after seating: every species lands in
  band except the two widest crowns, which clamp at `lean_min_deg` (46°).

- **`tornado.NO_UPROOT` — the widest crowns SNAP instead.** Black_Oak is still
  8.2 m under at the shallowest allowed lean, and no angle fixes a 25 m crown
  lying on a flat plane. The real bias is the answer: a tree that large tends
  to fail in the STEM rather than at the roots, because its root plate is
  enormous and its trunk section runs out of capacity first. So
  `tree_level_for_intensity(..., species=...)` promotes `fallen` to `snapped`
  for those species, and the bake skips the combination so no stale archetype
  is left on disk for a future caller to reference by accident. The effect is
  small — the asset set plants Black_Oak only in open ground, never on the
  frontage — but without it every park specimen in the track is half-buried.

### THE TRUNKS ARE OPEN SHELLS, AND TIPPING ONE SHOWS YOU

Reported off the second scene: *"the fallen down ones look like they are
hollow cylinders instead of tree trunks that broke or uprooted"* — and it is
true of the ASSETS, not of the damage code. Measured boundary edges in the
lowest 3% of each bole: **Shumard_Oak 75, American_Beech 19, Douglas_Fir 9**.
Every trunk is an open tube at the bottom. Standing, that hole is underground
and nobody has ever seen it. `tip_tree` lifts the base clear of the ground and
rotates it toward the camera, and you look straight down the inside.

`root_ball` plugs it, and does the second job too — a tree that was CUT leaves
a stump, a tree that BLEW OVER leaves a mass of root and earth at the end of
its trunk, which is the clearest way to tell them apart at any distance.

**It took two tries and both failures were the same mistake: sizing in the
abstract instead of deriving from the geometry.**

- *A flat plate* (`root_plate`, now unused). One polygon standing on edge. At
  160 fallen trees it read as brown cardboard cutouts over the mud — and,
  being a plane, it plugged nothing, because a plane has no inside.
- *A ball at an absolute 1.1-1.9 m radius.* A trunk on these species is about
  0.3 m, so every ball rendered three to five times the tree it belonged to.
  With a hard taper (0.72) on 11 sides it read as a CONE: a field of brown
  tents. Worse than the plates.
- *Sized to the trunk.* `wood_debris` already measures the bole for its own
  cuts, so it publishes `info["trunk_r"]` and the ball is `1.5 x` that, capped
  at 0.85 m, taper softened to 0.88 on 13 sides. A lump at the base instead of
  a landmark.

The same lesson runs through the tree lean (bisect against the measured
crown, do not pick an angle) and the plank cells (a regular lattice guarantees
identical pieces). **Derive it from the asset.**

### MESH FOLIAGE MUST BE EXEMPTED FROM `defoliate`

Half this library ships its crown as ONE PLAIN MESH rather than as
PointInstancers — Largetooth_Aspen, American_Beech, Common_Apple. A mesh has
no per-instance ids, so it cannot be thinned: `defoliate` takes ONE roll
against the survival at its height and either keeps it whole or deactivates it
whole. That is a fair approximation for a fire, where a crown either survives
or is consumed.

It is wrong for wind, and visibly so. At `limbed`'s `keep=0.80 / keep_top=0.60`
those three species come out **fully bare about a third of the time** — which
is the one thing a windthrown tree must not look like, and on a mixed-species
street a scatter of bare trees reads as a burnt stand standing in an otherwise
undamaged suburb. `defoliate(keep_meshes=True)` skips them: instanced crowns
still thin, mesh crowns come through intact. `wind_tree` always passes it.

### `drop_to_ground` MUST be off for the tipped levels

`bake.export_object(drop_to_ground=True)` seats an object by its FIRST root's
world min-z. For a tipped tree that minimum is a crown branch below grade, so
dropping lifts the whole tree until the lowest twig touches z=0 and leaves the
trunk floating metres up — the same class of bug the fire bake records for
sunken bole segments, arriving from the opposite direction. `_NO_DROP =
("leaning", "fallen")` in the bake launcher.

### `fell_branches` was written for this and never called until now

`vegetation.fell_branches` is dead code in the wildfire path and its docstring
reserves it explicitly for "a wind or storm-damage pass". It bakes crown
PointInstancer instances into real world-space meshes via
`ComputeInstanceTransformsAtTime`, binds bark, and removes the taken instances
from the instancer (or the same branch renders twice). It only works on
species that HAVE woody instancers — Aspen, Beech and Apple ship one plain
`leaves` mesh and get their limbs from `wood_debris` instead.

## Trees STAY UP in a fire and COME DOWN in a wind

The wildfire skill's most load-bearing domain fact, run backwards. Immediately
behind a fire front a burnt stand is a field of standing black poles, and
`fall_chance` is 0.12 there precisely because "a scene full of downed trunks
reads as windthrow — a tornado — not a fire". Here downed trunks are the
majority case inside the corridor, and `_TREE_CUTS` puts `fallen` in the
widest band on purpose: a windthrown tree with its green crown on the ground
is the single most legible feature of a track from the air.

---

# The plank field: authored, not simulated

This is the piece that carries the directional read, and it is deliberately
NOT fracture output.

**Why not fracture.** It does not scale (fracture + settle is the whole
~27-minute cost of a 250 m block, and a track is tens of thousands of loose
boards); a Voronoi cell is a chunk and a building sheds BOARDS; and a board
lying flat on open ground has nothing for a solver to discover. Authoring it
directly also means the field can be **retuned without rebaking anything**,
which is most of what iterating on this scene consists of.

**One mesh per stock class, not one per board.** Same argument
`ground.build_overlay` makes for its bands: 8 points and 6 faces a board,
merged, so a plate's worth of debris is five prims. It is also why the pieces
are boxes rather than referenced assets — an instanced reference cannot be
merged, and merging is worth more here than instancing.

**Normals must be authored `faceVarying`.** A board is a hard-edged box and
the renderer's fallback averages normals at shared vertices, which rounds every
corner. A stack of lumber then looks like a heap of pillows, and the crisp
rectangular read that is the entire point of authoring boxes is gone.

**The stock list is a cutting list, not a size range.** A uniform draw over
"length 0.5-4 m, width 0.05-0.6 m" gives a cloud of arbitrary rectangles, and
arbitrary is the one thing building material is not. `planks.STOCK` names five
real sections — `stud`, `joist`, `sheathing`, `board`, `deck` — so the mix is
tunable by what it IS. `sheathing` is what you actually see from altitude: a
stud is a stick at 60 m, a half-sheet of OSB is an object.

**The scatter is a COMET.** Distance downtrack goes as `reach * u ** 1.9`, so
most pieces stay near the slab and a minority carry a long way; a linear draw
gives an even smear that reads as a painted stripe. Lateral offset is gaussian
with a width that GROWS with distance — a constant width is a corridor, and a
corridor is what a bulldozer leaves. Spreading is what says the debris was
airborne.

**Scatter across the CORRIDOR too, not only on the lots.** In the reference
photograph the debris does not stop at property lines; it runs continuously
across roads, verges and the field beyond, because what is lying there came
from somewhere else. A field assembled only from per-house trails leaves clean
green gaps between the lots and the corridor stops reading as one event.
`scatter_over_region` samples on a lattice with density scaled by the local
intensity, so the corridor's own gradient carries into the debris with no
separate edge to tune.

---

# The archetype yaw problem, and how it is resolved

Every archetype is baked with its debris thrown toward **local +X**, and the
assembly chooses each reference's yaw. That choice is the whole trick:

- `roof_stripped` / `roof_collapsed` / `partial_collapse` → yawed to the
  **STREET**. A house facing the wrong way is the most obvious defect a suburb
  can have, and the debris field at these levels is small enough that its
  direction is not the read.
- `leveled` / `swept` → yawed to the **TRACK**. Nobody can tell which way a
  pile was facing, and where its material went is the entire point.
- Trees: `leaning` / `fallen` → yawed to the track (the yaw IS the fall
  bearing) with a **wide ±38° jitter**, because trees go over in the direction
  their own rooting and neighbours allow as much as in the direction the wind
  was going. A stand all pointing within five degrees reads as a logging
  operation, not as windthrow.

Yaw about Z is legal on a baked archetype. The wildfire skill's "do not roll
or pitch a baked archetype" warning is about X and Y — the object was settled
by PhysX with its debris laid flat at z=0, and turning it about a horizontal
axis tips the debris field with it. A yaw carries the bed round intact.

This is also why the archetype count stays at 8 styles x 6 levels. Baking a
throw direction per (style, level, direction) bin would be 8 x 8 = 64x the
work for a difference the plank field already provides.

---

# The ground scour

Same machinery as the burn scar and every one of its lessons —
`ground.build_overlay` with a `coverage_at` from `tornado.scour_coverage` and
the `Soil_Mud` megascans surface instead of the burnt floor. A coverage field
that peaks on the centreline IS an opacity that peaks on the centreline,
because `build_overlay` buckets coverage and maps the bucket onto `op_range`.

Three things differ from the burn overlay:

- **`tile_m` is NOT the whole plate.** `ground` projects one tile across the
  entire scar because the burnt floor is a 4K map and a burn covers
  everything. The mud pack is **1K**, and one tile across 500 m is half a
  metre a pixel — mush at any altitude worth flying. ~45 m a tile is 4.4 cm a
  pixel and repeats about a dozen times along a track, which is acceptable
  because a corridor is a ragged band rather than a full plate and there is no
  regular grid for the eye to lock onto. If a 4K soil pack turns up, drop the
  tile and re-point `tornado.MUD_TEXTURE` — the resolution is in the filename
  and a stale path draws UNTEXTURED rather than failing.
- **More bands** (14 vs 12). The whole read here is the cross-track gradient,
  and coarse quantisation of it shows as stripes parallel to the track.
- **Islands are ON** (0.05, against the burn scar's 0). Inside a burn scar
  they read as patches the pass missed; scour genuinely is patchy at the edges
  of a path and a few surviving green patches inside the corridor is what a
  real one looks like.

`gamma < 1` widens the visibly muddy band without widening the structural
damage, which is correct: peeling turf takes less wind than failing a wall.

**Both fractional-cutout rules from the wildfire skill apply unchanged**: the
Kit flags must be on the command line (`SimulationApp(extra_args=...)`) AND
re-asserted with `carb.settings.set_bool` after the environment stage is
loaded, because loading a stage with authored render settings resets the
property to OFF. Symptom of missing either: "I don't see the ground at all."

---

# The field itself

## The noise MOVES THE BOUNDARY, it never adds to the value

The level-set lesson, inherited verbatim from `ground.py`. Writing
`i * k + (noise - 0.5) * m` is the obvious way to perturb a field and it means
ground the tornado never touched comes out speckled with damage. In
`intensity_field` the noise is added to the CROSS-TRACK DISTANCE before the
profile is evaluated, so it can only move the edge in and out — outside the
widened path the profile is identically zero.

## The track must MEANDER, and it must not SNAKE

A perfectly straight corridor is the single most artificial thing this scene
can show. But the first cut used `wobble_period_m = 0.70 * span`, which
completes two full cycles across the plate and reads as a drawn S-curve — the
exact failure the wobble exists to avoid. **1.6 x span with 0.032 amplitude**
looks like the track drifted rather than like it was steered. Two harmonics,
never one, for the reason `scar_patch` uses three: one sine is a recognisable
shape and the eye locks onto it.

## Width and strength BREATHE, out of phase

Real tracks swing two EF categories along one path. `along_min` and
`width_min` are what put weak stretches inside the corridor where a house
survives, and they go UP with severity, not down: weak tornadoes skip — touch
down, lift, touch down again — while a violent one is continuously on the
ground for its whole length.

## The light damage classes get the WIDEST bands

Splitting 0..1 evenly put 62% of the damaged houses in one class, which reads
as a corridor with a hard edge and nothing outside it. EF0-EF1 damage —
covering off, soffits out, a garage door in — extends well past the swathe
anything else can be seen in, and by count it is most of what a tornado does.
Current cuts are `0.08 / 0.36 / 0.54 / 0.70 / 0.87`.

## Jitter is what stops the levels from being contour lines

One draw per object, applied to the intensity before the ladder lookup.
Without it every boundary is a clean curve parallel to the track — five
stripes of identical houses, which reads as a gradient map. The fire path gets
this for free from the front's arrival jitter. Trees get a LARGER jitter
(0.09 vs 0.07) than houses, for the same reason the wildfire skill gives for
severity being a property of the STAND: whether a given tree goes over depends
on its rooting, its lean, its exposure and what its neighbours did, none of
which this scene models.

## `width_m` is in METRES, not a fraction of the plate

A deliberate departure from the generic `field.width_m` the disaster compiler
also emits. A tornado path is a physical width; scaling it with the plate
means a 500 m scene and a 1600 m one show the same picture at different zoom,
when the smaller scene should show a narrower path through fewer houses.
Capped by the compiler at **32% of the plate**, measured off `tornado.jpeg`,
because a corridor whose edges are off-frame is not a corridor.

---

# People: `disaster/tornado_people.py`

**Not `disaster.people` with the names changed.** A wildfire gives hours and
people MOVE, so that module is a model of EGRESS — refuge car parks, roadways,
gridlock, the cul-de-sac trap, location as a function of the road network. A
tornado gives ten to fifteen minutes and nobody evacuates: people shelter where
they already were, the building fails around them, and the survivors walk out.
Location is a function of where they LIVED and what the wind did to it. Four of
the six wildfire scenarios have no counterpart here at all.

## The fact the module turns on: almost everyone survives, on their feet

| | |
|---|---|
| Joplin **catastrophic** zone (total structural destruction) | 4,716 people, 122 dead — **97.4% survived** |
| Joplin whole path | 13,547 people, 161 dead (1.2%), ~1,371 injured — **~89% alive and mobile** |
| Injury severity (Niederkrotenthaler 2013, n=1,398 across 39 hospitals) | **89% minor** (ISS<10), 6% moderate, 5% severe; **86% discharged home** |
| Deaths | **86.6% on scene** (Chiu 2013); 94% (May 2000) |

A scene built on "a levelled block is a morgue" is wrong in the most costly way
available: it would be EMPTY of the targets the benchmark exists to find. The
corridor is full of standing, walking, digging people, and the visible
population does NOT thin toward the centreline — with the houses gone there is
nothing left to occlude anyone.

## People cluster — over 90% of them

Chiu et al. recorded who each victim was found with: with other deceased AND
survivors 26.3%, with other deceased 24.7%, with other survivors 21.1%,
**ALONE only 7.7%**. Group placement is the empirical default, and a CLUSTER is
itself the strongest aerial indicator that somebody is trapped under it.

## The epoch is T+30-60 min, and it is not a lighting choice

The only window in which aerial victim detection has any value, and the peak of
visible human density. Joplin's professional rubble-rescue count was
**seventeen**, against 1,371 injured; the earthquake literature puts civilian
extrication at 60-100%. Greensburg: 68% of households did SAR, 28% starting
immediately. So: **no responders, no heavy equipment, no search markings, no
triage tents** — those are T+12-24 h artefacts, and by then there is nobody
left alive to find.

## `thrown` is capped at one or two, and here is the arithmetic

CDC surveilled all 338 deaths in the April 2011 outbreak, recording location of
INJURY and of RECOVERY separately (MMWR 61(28)): **90.5% injured indoors, 3.3%
outdoors when hit, but 37.0% of bodies recovered outdoors.** Brown 2002:
"picked up / blown by tornado" was the mechanism for **43% of hospitalised**
injuries and only **6% of treated-and-released** — being thrown is a severity
marker, not a common experience.

Applied to ~39 damaged houses (~100 residents, Joplin path rates): 0.8 dead x
37% + 1.25 hospitalised x 43% = **under one person in the whole 500 m scene.**
Long-range lofting is record-book territory (398 m, Matt Suter, an F2,
GPS-measured; 76 m for a family on a mattress at Dawson Springs) and there is
**no published distribution of throw distances**. And a thrown figure is
PRONE — anyone walking in the open emerged and walked there.

**Do not drive deposition with the `throw` field.** The 78%-left statistic is
for lightweight debris lofted into the parent storm, not bodies; near-surface
flow is CONVERGENT toward the centreline (Karstens et al., 10,300 tree falls at
Joplin, 94,500 at Tuscaloosa). There is no published azimuthal distribution for
victim deposition, so a strong fan would be an invented claim.

## `trapped_partial` is the hard one, and it is a dataset decision

Fully buried is an unlabellable target; fully exposed is not a trapped person.
The scenario emits its OWN boards across the lower body rather than trusting
whatever the wreck archetype left there, and sinks the figure by a fraction of
the local debris depth. **Its correctness cannot be read off the code** — it
has to be confirmed from a render at capture altitude, which is what the bench
is for.

It is also deliberately over-represented: Chiu records "trapped in rubble" as
the mechanism for **1.7%** of fatalities and "crushed" for 18.7%, so a true
rate would put well under one in the scene. A dozen is a benchmark decision,
recorded as such in the module so nobody later reads the share as an estimate.

## What is deliberately absent

- **`parking_refuge`** — the largest wildfire scenario (0.30) and it has NO
  tornado counterpart. Moore, Joplin and Mayfield had no public shelters at
  all; Moore's EMA explains why (10-15 min is not enough to leave, drive, park
  and get inside). Observed use ~4% where one exists, 0% where none does.
- **`gridlock`** — only Oklahoma City ever produced a mass traffic reversal
  (El Reno 2013, broadcast-directed, metro-arterial). Moore 2013 had ZERO
  vehicle deaths, and fleeing by car was PROTECTIVE in every study that
  measured it (Daley 2005: severe-injury OR 0.2).
- **Cellars, safe rooms, house interiors** — where people actually shelter, and
  unscoreable by a drone. This is `disaster.people`'s `exposed_interior`
  lesson and it is not relitigated.
- **Bystanders at the track edge** — real (40% go outside to look) but out of
  scope; this scene is about the people the tornado HIT.

## Vehicles: use the measured displacement rates

Paulikas, Schmidlin & Marshall 2016, 959 vehicles across 12 tornadoes:

| | displaced | rolled / tipped |
|---|---|---|
| EF0 | 10% | — |
| EF1-EF2 | 36% | 5% |
| EF3-EF4 | 63% | 15% |
| EF5 | 69% | 31% |

**These are UNCONDITIONAL shares of all vehicles in the zone.** Testing a tip
probability only on cars that had already passed a move probability multiplies
the two and produced an effective 6% where the data says 15% — measured, 5 of
25 cars moved and NONE tipped. One draw against nested thresholds.

And `toss_prim` MEASURES the prop to seat it. A car pivots about its own origin
on the ground, so rolling it 90 degrees swings half the BODY below grade by
half the car's WIDTH — which is not the 0.7 m the first version guessed for
everything, and not the same for a hatchback and a pickup.

---

# Current knob values, and why

| knob | value | why |
|---|---|---|
| `width_m` cap | 0.32 of the plate | the swathe is a quarter to a third of the reference frame; intact fabric either side IS the read |
| `core_frac` | 0.22 | 0.30 put too much of the corridor at the top two damage classes |
| house cuts | 0.08 / 0.36 / 0.54 / 0.70 / 0.87 | light classes widest; even splits collapsed the gradient |
| tree cuts | 0.07 / 0.30 / 0.46 / 0.80 | `fallen` widest — a downed green crown is the most legible feature |
| `wobble_period_m` | 1.60 x span | 0.70 x span is two cycles and reads as a drawn S |
| `wobble_m` | 0.032 x span | a drift, not a steer |
| `curl_deg` | 20 (toward the LEFT of travel) | cyclonic: rotational and translational winds add on the right flank, so material lofted there is dropped on the left. Also legible — debris exactly along the centreline is hidden by it |
| `spread_deg` | 42 -> 28 by severity | a hard heading puts every piece on one line, which reads as a fence |
| `throw_m` | 12 -> 42 by severity | the PLANK field's reach; free, because it is authored |
| `throw_speed_mps` | 4 -> 11 by severity | the SETTLE bias; every metre of it costs settle steps |
| settle `max_speed` | 2.2 x throw | the 4.0 default silently clamps the bias |
| settle `damping` | 0.16 | 0.55 bleeds over half the speed away per second |
| settle steps (bake) | 1200 | MEASURED: 700 consumed all of itself and left 182 of 7,219 bodies still moving — baked mid-flight. A thrown fragment is airborne far longer than a falling one. `steps` is a CEILING with early exit, so a generous budget is free on the quick cells; watch the STILL MOVING line |
| `consume` | 0 to `leveled`, 0.62 at `swept` | wind displaces; only at EF4-5 is the material genuinely off the lot |
| `consume_pool` | 1.6 | soft bias — the big recognisable pieces are what make a track read |
| `rough` (plank) | 0.010 | 0.045 bows a thin board |
| `min_volume_frac` (plank) | 0.0008 | 0.004 discards every broken sheet |
| `min_aspect` (tree debris) | 1.25 | 2.0 rejects every plank-shaped piece, silently |
| `len_bias` (tree debris) | 1.15 | the fire path's 2.2 produces mostly charcoal-sized pieces |
| tree debris `thick_m` | 0.10-0.30 | a limb torn off a live tree, not a burnt stick |
| root ball radius | 1.5 x measured `trunk_r`, capped 0.85 m | absolute metres gave 3-5x the tree — a field of brown tents |
| root ball taper / sides | 0.88 / 13 | a hard taper on few sides reads as a cone |
| tree lean (`fallen`) | 74-82 deg MAX, then seated | the resting angle is set by the crown, not chosen — `seat_band` bisects it down per species |
| `seat_band` | (-1.1, -0.15) m | the crown presses into the turf rather than hovering over it |
| `lean_min_deg` | 46 | below this it reads as leaning, not fallen; the two widest crowns clamp here |
| `tilt_p` (planks) | 0.22 | debris photographs as a mat; a uniform tilt is a jackstraw pile |
| plank yaw | heading + 90, sigma 46 deg | long thin objects lie across a flow more often than along it |
| `MUD_TILE_M` | 45 | 1K pack; one tile across 500 m is 0.5 m/px |
| `MUD_BANDS` | 14 | the cross-track gradient IS the read |
| `MUD_ISLANDS` | 0.05 | scour is genuinely patchy; unlike a burn scar's islands these help |
| bake `GRID` | 50 m | 40 is the fire value; a thrown fragment carries most of ten metres |

---

# Workflow

## COUNT BUILDINGS, NOT MODULES

The first cut of `tornado_png.py` filtered the placement list for anything
categorised `house*`. That counts the KIT MODULES a house is assembled from:
it reported **247 houses on a plate the assembly then built 38 of**, and every
number downstream — the damage tallies, the plank budget, the "has the
gradient collapsed" warning — was wrong by that factor, in the direction that
makes a sparse scene look adequately dense on paper.

The modules carry no per-building id, so there is nothing to group them by.
The fix is to ask for the list the assembly itself uses:
`fence_png.build(house_instances=[])` threads that straight into
`suburb_scene.build_placements`, which records each house's (style, pose) and
skips emitting its ~28 modules. Host-side and assembly counts then agree by
construction rather than by coincidence.

**The plate was genuinely sparse once the count was honest.** 37 houses on
500 x 500 m against a reference photograph of dense post-war suburb. Fixed in
the preset alone — no re-bake, since density is a layout property:
`block_area_target_m2` 26000 -> 19000, `min_gap_m` 78 -> 64, `lot_width_m`
[26,38] -> [22,32], `house_gap_m` 6.0 -> 4.5, and `density_mix` shifted toward
the tight classes. Result: **151 houses, 41 of them in the track.**

## Iterate host-side first

    python3 scene_gen/tools/tornado_png.py --config suburb_tornado

Prints the in-path fraction, the full level tallies for houses and trees, the
plank budget, and warnings for the three ways this scene fails:

- the track covers most of the plate (no intact fabric to contrast against),
- the track barely touches it (wrong `epicenter` / `heading_deg`),
- one damage class is >70% of the damaged houses (the gradient has collapsed).

A run that prints `OK  gradient and coverage both in band` is worth building.

## Then bake, then assemble

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
    ISAAC_SIM_SCRIPT_NAME=bake_tornado_archetypes_launch_script.py \
    airstack up isaac-sim

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
    SCENE_CONFIG=suburb_tornado \
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/tornado \
    ISAAC_SIM_SCRIPT_NAME=suburb_tornado_launch_script.py \
    airstack up isaac-sim

Relaunching between edits goes through tmux, never `airstack down` — see
[run-isaac-sim-launcher](../run-isaac-sim-launcher/SKILL.md). `down` removes
the container and throws away `manifold3d` / `shapely` / `mapbox_earcut`, and
**a fresh container does not pay one pip install, it pays one RUINED RUN**:
trimesh caches engine availability at import time, so every capped slice comes
back empty, every archetype bakes with zero debris, and the banner still looks
complete.

**`ARCH_DIR` must not be the wildfire library.** Both write
`house_<style>_<level>.usd` and the ladders share `roof_collapsed` and
`partial_collapse`, so a shared directory silently overwrites burnt archetypes
with unburnt wreckage.

`ARCH_KINDS=tree` re-bakes only the vegetation half (the manifest is merged),
which turns the iteration loop from minutes into a fraction of one.

## Assembly knobs, all env, no rebake needed

| var | default | what |
|---|---|---|
| `TOR_PLANKS` | 140 | boards per wrecked house; 0 disables the field |
| `TOR_TRACK_PER100` | 4.5 | boards per 100 m2 of corridor; scaled by `intensity ** 1.4`, so the edge stays sparse |
| `TOR_GROUND` | 1 | 0 disables the mud overlay |
| `MUD_TILE_M` / `MUD_BANDS` / `MUD_OPACITY_*` / `MUD_ISLANDS` / `MUD_GAMMA` | see `tornado.knobs_from_env` | overlay tuning |
| `TOR_SEED` | 11 | the damage draws (not the layout seed, which is in the preset) |
| `SNAP_DIR` | — | viewport PNGs; MUST be under `/isaac-sim/.nvidia-omniverse/logs/` or the host cannot read them |

The snapshots are taken **along the track** rather than at a list of
landmarks: three points down the centreline plus one on each flank, top-down
and oblique, because what has to be judged is a gradient across a corridor.

---

# Known gaps

- **The scour is a translucent overlay**, with the same limitation the burn
  scar has: diffuse only, so the mud's normals do not blend with the grass.
  The in-material `OmniSurfaceBlend.mdl` route described under "If there is a
  fifth attempt" in the wildfire skill is the upgrade path for both.
- **The soil pack is 1K.** Every other ground surface here is 2K or 4K. A 4K
  re-import would let `MUD_TILE_M` go back to the whole plate.
- **No debarking.** Above about EF3 real trees are stripped of bark and small
  branches entirely, which is a strong severity cue this pipeline does not
  produce — `snapped` loses its crown but keeps its bark texture.
- **The `swept` level still leaves fragments on the slab.** `consume=0.62` is
  a fraction, not a sweep; a true EF5 slab is bare. Doing it properly means a
  separate "slab only" archetype rather than a consume value.
- **Fences are deleted, not felled.** There is no leaning-fence archetype and
  what a track leaves of a timber fence is post stubs plus boards elsewhere —
  which the plank field already authors — but the post stubs are missing.
- **Cars are moved but not deformed.** `_toss` pushes and rolls them; a car
  that has been through an EF3 is also crushed, and nothing here does that.
- **No power lines, poles or transformers.** Downed lines are one of the most
  characteristic features of a real track and the asset set has no cable.
- **The plank field carries a collider-free mesh with no LOD.** It is five
  merged meshes, so it is cheap, but at ~8k boards on a 500 m plate a 1600 m
  plat would want chunking by region rather than by stock class alone.
- **No people.** Deliberate — survivors are a separate pass; see
  [place-people-in-scenes](../place-people-in-scenes/SKILL.md).
