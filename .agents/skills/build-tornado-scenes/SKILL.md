---
name: build-tornado-scenes
description: Build or modify TORNADO-damaged scenes in scene_gen — the track field, directional debris, sawn-plank wreckage, windthrown trees with their leaves on, the mud scour along the path and the 3D earth standing on it (cycloidal marks, windrows, spoil heaps, rolled sod). Read before touching disaster/tornado.py, disaster/scour_relief.py, disaster/wind_flow.py, disaster/planks.py, vegetation.wind_tree, or the suburb_tornado launchers. The wildfire skill is the prerequisite; this one is mostly about what is DIFFERENT and why the fire code cannot be reused as-is.
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

    layout -> generate -> [bake once per archetype] -> assemble -> planks
          -> scour (flat overlay) -> scour relief (authored earth) -> people

Two launchers, in this order:

| launcher | what it does | cost |
|---|---|---|
| `bake_tornado_archetypes_launch_script.py` | every (house style x damage level) and (tree species x wind level) built, wrecked, settled and exported ONCE | minutes, once |
| `suburb_tornado_launch_script.py` | the 500 x 500 scene: layout live, houses/trees BY REFERENCE, plank field, mud scour and its 3D relief authored | seconds |

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
| `scene_gen/disaster/tornado.py` | the track: frame, meander, intensity field, throw field, both damage ladders, the scour coverage, and `from_track` (the inverse map the relief draws its marks through) |
| `scene_gen/disaster/scour_relief.py` | the 3D ground: cycloidal marks, windrows, spoil heaps, rolled sod mats, clods, road wash — scatter in pure Python, merged meshes on the stage |
| `scene_gen/disaster/wind_flow.py` | wreck one building — the no-fire counterpart of `damage_flow.py` |
| `scene_gen/disaster/planks.py` | the sawn-timber debris field: stock list, comet scatter, merged box meshes, plain wood material |
| `vegetation.wind_tree` + `tip_tree` + `root_plate` | windthrown trees, leaves ON |
| `scene_gen/tools/tornado_png.py` | the host-side plan, including the relief layer and its budget (`--no-relief` to skip) |
| `scene_gen/tests/test_scour_relief.py` | ten offline checks on the relief — the placement rules, the winding, the seating, the z ladder at the launcher seam |

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

## FLOATING DEBRIS: three causes, and the two everybody guesses are not among them

*"A lot of the debris still looks like it's floating."* Investigated 2026-08-27
with offline replays of the placement maths, a bare-`pxr` probe over the baked
archetypes, and shadow measurement on the renders. Three real causes, ranked.
**Two obvious suspects were measured and RULED OUT — check this list before
re-deriving them.**

### The amplifier: the sun is at 24 degrees

The scene borrows its sky from `RetroNeighborhood.stage.usd`
(`asset_sets/shared.yaml`), whose rig is at **16.36 h with the sun 24.4 degrees
above the horizon**. That is **2.21 m of shadow displacement per metre of
height**. Four centimetres of float throws a 9 cm shadow gap; 40 cm throws
0.9 m. A detached shadow is what the eye reads as "floating", so this scene
punishes vertical error about twice as hard as a midday one. Any float test
should be done on the OBLIQUE, where the shadow is visible.

### Cause 1 (dominant): `planks._lay` levered every board onto one corner

The seating was `z = ground_z + half_h`, where `half_h` is half the vertical
extent of the ROTATED box — i.e. the lowest corner exactly touches the ground.
**So no board in the field ever lay on its face.** Replayed over 200,000 draws
against the real `STOCK` table and weights, the board's CENTRE finished at

    p50 0.079 m   p75 0.122   p90 0.373   p95 0.534   p99 0.819   max 1.415

and its high corner at p50 0.150 / p90 0.750 m. On a 758-board plate: 245 (32%)
more than 10 cm up, 113 (15%) more than 25 cm — every one with clear air under
it. Confirmed on the render: a lawn patch held 12 boards and 12 **fully
detached** shadows, centroid offsets 0.57-2.42 m.

`tilt_p = 0.22` made it worse — ~167 boards a plate at pitch +-34 / roll +-30
**with nothing authored underneath**. A tilted board over open lawn is not
"came to rest on something else", it is a floater.

Fixed three ways:

  * **seat on the FACE and let it bed in.** `_BED_M` (0.02 m) of sink, capped
    so the board never goes past resting on its own centre-plane thickness. A
    flat board is unaffected; the levering is gone. Measured after: **every**
    board's lowest corner is now at or below grade (-0.003 to -0.020 m), and
    the >10 cm float population is **zero**;
  * **the flat band scales with length.** It was +-4 pitch / +-6 roll for
    everything, which lifts the far end of a 4.2 m board 0.29 m — the "flat"
    case was itself a prop. Now `x (2.0 / max(2.0, ln))`;
  * **the tilt share follows the DEPTH of the mat.** `scatter_from_wreck`
    passes 0.34 within 0.75 footprints of the slab, 0.12 out to 1.4, and 0.02
    beyond — and `scatter_over_region`'s corridor scatter takes the 0.05
    default. A tilted piece is also capped so its top rises at most ~1.2 m
    (about the depth of a levelled house's pile) instead of standing a 5 m
    joist 2.8 m in the air.

There was also a small term error: `|w sin r|` should carry the pitch cosine,
`|w cos p sin r|`. Worth p99 1.8 cm / max 5.7 cm, always in the float
direction. `scour_relief._clod` had copied the same slip and is fixed with it.

### Cause 2: the baked archetypes ship fragments frozen in mid-air

Probed on disk (for each mesh: is there any other mesh overlapping it in plan
whose top comes within 0.10 m of its underside?). Axis-aligned boxes
over-count support, so these are LOWER bounds:

| archetype | meshes | airborne | median gap | max |
|---|---|---|---|---|
| `house_terrace_partial_collapse` | 429 | **28.9%** | 0.18 m | 0.96 m |
| `house_terrace_leveled` | 448 | 24.8% | 0.19 m | 0.87 m |
| `house_l_family_leveled` | 561 | 20.9% | 0.19 m | **2.61 m** |
| `house_ranch_leveled` | 253 | 17.0% | 0.24 m | 0.74 m |
| `house_cottage_leveled` | 172 | 11.6% | 0.22 m | 0.53 m |
| `tree_Shumard_Oak_snapped` | 43 | 39.5% | 0.28 m | 0.53 m |

**Read the `pristine` / `roof_stripped` / `roof_collapsed` rows as false
positives** — a roof and an upper floor legitimately have air under them. Only
`leveled`, `swept` and `partial_collapse` are the signal, because those are
supposed to be low piles. This preset places 3 leveled + 2 swept + 1
roof_collapsed, so **150-200 genuinely airborne fragments a plate**.

The mirror defect is as large: **21-42% of each wrecked archetype's meshes sit
below -0.05 m**, whole-object `z_min` down to **-2.9 m**. That is material lost
through the harness floor — `/World/arch_ground` is a four-vertex quad and at
`THROW_MPS = 9` the fragments reach ~20 m/s, fast enough to tunnel it.
(No archetype floats as a WHOLE: every `z_min` is negative. `pristine` is a
clean -0.2796 m for every style, which is its footing.)

Two root causes, one fix:

  * `settle.run(steps=...)` is a **budget with an early exit, not a
    convergence test**. Nothing asserts `info["still_moving"] == 0`; it is
    only printed. The 2026-08-27 bake used all 1200 steps with 4 bodies still
    moving, and an earlier one froze 182 of 7,219 mid-flight.
  * `_safe_export` passed `drop_to_ground=False` for houses, and the
    "raise anything the solver lost through the floor" rescue inside
    `bake.export_object` only runs under `drop_to_ground=True`.

**`bake._reseat_roots` is the fix and it is PURE GEOMETRY** — no
re-simulation, and it runs and is unit-tested on the host with a stand-in bbox
cache. It raises any root below grade and lowers any root with no support
within `_AIR_TOL_M` onto the highest thing beneath it, bottom-up so a stack
settles. Houses pass `reseat=True`; `drop_to_ground` cannot be reused for them
because its first act is to redefine the datum from the first root, and for a
house that is one module of many.

**THE SUPPORT TEST IS THE PART TO GET RIGHT, and it took three goes.** Both
wrong versions are recorded, because both are the obvious thing to write.

  * **v1 — "anything lying entirely below us" (`q_top <= z0`).** Drops every
    intact gable roof to the lawn: a roof's box includes its overhanging
    soffit, so the walls holding it up have tops ABOVE the roof's own min-z.
  * **v2 — "anything overlapping in plan that reaches our underside"
    (`q_top >= z0 - air_tol`).** Fixes the roofs and does almost nothing else.
    MEASURED on a rebake: `house_ranch_leveled` went 17.0% airborne to
    **15.1%**. A levelled pile is full of tall wall stubs, and a stub spanning
    0 to 2.5 m "supports" everything inside its plan footprint at every
    height — including a fragment frozen at 1.75 m with nothing but air under
    it.
  * **v3 — the support's top must land IN OUR VERTICAL SPAN**,
    `z0 - air_tol <= q_top <= z1`, AND the plan overlap must be a SEAT rather
    than a corner clip (at least 20% of the smaller of the two plan areas —
    measured against the smaller so a stub can still hold up a sheet, which is
    how a wall holds up a roof).

A box that simply TOWERS PAST us is not support. It might be a wall we are
wedged against and an axis-aligned test cannot tell, so **the tie breaks toward
dropping**: a fragment lying on the ground is a pose the solver could plausibly
have reached, and one hanging in mid-air never is. That is the same call
`bake.export_object` already records for tree debris — *"a log lifted to rest
flat on the ground is a pose it could plausibly have settled into anyway."*

### Cause 3 (latent, and it bites the moment a car rolls): `toss_prim` over-lifts

`UsdGeom.BBoxCache.ComputeUntransformedBound` does **not** exclude the prim's
own `rotateZ`. Measured on a 2.0 x 0.5 x 1.0 m box under translate + rotateZ +
scale:

    yaw   0     -> (-1.000, -0.250, 0) .. (1.000, 0.250, 1)   correct
    yaw  37 deg -> (-1.240, -1.211, 0) .. (1.240, 1.211, 1)   WRONG

The box comes back rotated and then re-aligned, inflating the across-axis half
extent 0.25 -> 1.211 m. `lift` is computed from exactly those extents, so a
4.7 x 1.8 m car parked at **yaw 45** reported a half-width of ~3.25 m instead
of 0.9 and, rolled onto its side, was lifted **3.25 m — 2.35 m of air under
it**. Exact at multiples of 90 degrees, worst at 45, which is why it hid.
Fixed by clearing the op order BEFORE the query (the ops are rebuilt from the
cached values either way). `scale` IS correctly excluded, so the separate
multiply stays.

**AND THE INSTANCING HYPOTHESIS IS FALSE — it was tested.**
`ComputeUntransformedBound` returns a non-empty bound on an instanced prim,
byte-identical to the non-instanced case. `lift` does not silently become 0.

### Cause 4 (the TREE debris, and it is arithmetic not physics): the drop datum

*"There is some floating logs (tree debris), etc. Why? place them close to
z=0, shouldn't be that hard."* Reported 2026-08-27 on the assembled plate,
AFTER causes 1-3 were fixed and after `tree_level_and_yaw` had already stopped
trees throwing their beds off the edge of the ground sheet. A different bug
with the same symptom, and the only one of the four that is pure arithmetic.

**MEASURE THE ARCHETYPES BEFORE THEORISING** — a bare-`pxr` probe, no
SimulationApp, safe beside a running sim (see the run-isaac-sim-launcher
skill's standalone `pxr` section). Per mesh: world bbox, and "is anything
overlapping me in plan with its top inside my vertical span". The table names
the bug on its own:

| archetype | meshes | airborne | z_min |
|---|---|---|---|
| `tree_Shumard_Oak_snapped` | 43 | **39.5%** | -0.98 |
| `tree_Shumard_Oak_limbed` | 17 | **35.3%** | -0.98 |
| `tree_Douglas_Fir_limbed` / `_snapped` | 61 / 47 | 0.0% | -0.21 / -0.52 |
| `tree_{Beech,Aspen,Apple}_{limbed,snapped}` | 15-41 | 0.0% | -0.00 |
| every `leaning` / `fallen`, all species | 10-68 | 0.0% | -0.68..-2.03 |

and inside the two bad files **three independent sticks sat at exactly
z = 0.53 m** with clear air under them. A solver does not leave unrelated
bodies at the same height to the centimetre. A single uniform offset does —
and `bake.export_object`'s own docstring already records that signature from
the opposite cause ("every seated stick in `tree_American_Beech_torched` sat at
exactly 0.567 m").

**The mechanism.** `wood_debris` does not simulate its small pieces at all: it
SEATS them, `piece.bounds[0][2] = ground_z - 0.001`, and returns them as
static geometry. `export_object(drop_to_ground=True)` then takes the world
min-z of the FIRST root — the tree — as `rz` and subtracts it from every mesh
in the file. **When the tree's own geometry dips below grade** (a root flare
under the turf, a low branch reaching past the trunk's base) **`rz` is
negative, and subtracting it LIFTS the whole file**, every analytically-seated
stick with it, by exactly `|rz|`.

The distribution confirms it end to end. Anything that had sunk below `rz` was
rescued individually to `rz` and therefore lands at exactly 0.00 — the large
cluster of logs at 0.00 in the probe, and the reason the file looks half
right. Anything correctly seated was ABOVE `rz`, kept its relative height, and
came out floating by `|rz|`. And `_NO_DROP` (`leaning`, `fallen`) never applies
the drop at all, which is why those files are clean at every species. **The
float appears exactly, and only, where the drop was applied.**

**The fix is a separation, not a tolerance.** The drop moves the OBJECT; the
debris keeps the absolute Z it was authored at (`export_object` compensates the
datum shift back out for every root that is not the first); and `_reseat_roots`
— the pure-geometry pass the houses already had — is what corrects a piece the
solver genuinely misplaced. Trees now pass `reseat=True, reseat_first=False`.

**`reseat_first=False` IS LOAD-BEARING AND THE OBVIOUS VERSION IS WRONG.** A
windthrown tree's pose is AUTHORED, not settled: `tip_tree` bisects the lean
down until the crown is just into the turf (`seat_band = (-1.1, -0.15)`) and
`wind_tree` lifts a fallen base by the root plate's own radius on purpose.
Both look exactly like "sank through the floor" to the sink test, so reseating
the tree along with its debris stands every fallen trunk back up on the lawn —
deleting the single most legible feature of a track from the air. `freeze` in
`_reseat_roots` keeps a root as SUPPORT while never moving it, so a log can
still come to rest across a fallen trunk.

**And the two bbox caches disagreed.** `export_object` measured over
`[default_]` while `audit_archetype` measures over `[default_, render]`. A
root the seating pass cannot measure is a root it silently declines to
correct — which audits as floating debris that the bake reported as clean.
Both are `[default_, render]` now.

The arithmetic is sliced out as `bake._seat_plan` and pinned offline in
`scene_gen/tests/test_bake_reseat.py` — 20 checks, a dozen lines of stand-in
for the bbox cache, no Isaac. Composition is the part worth testing: each half
was right on its own.

### RULED OUT — measured, do not re-derive

**The ground z-ladder is not the cause.** At this preset's `roads.z_scale =
0.10` the ladder is grass **2.0 mm**, mud overlay 6.0, asphalt 10.0, driveway
16.0, walk 17.0. Boards are laid at `ground_z = 0.0`. So a board floats **2 mm**
over block grass and is BURIED 2-13 mm in the mud, asphalt, drive and walk. One
pixel of the 60 m top-down is **54.6 mm** — every one of those is sub-pixel and
they run in the SINK direction. Worst case even if the override failed to land:
9.6 mm.

**The scour relief is not the cause.** Nothing in `scour_relief` goes below the
base plane except `_SINK_M = 0.02`, a deliberate skirt so a rim keys in. There
are **no troughs or gouges**: mounds <= 0.53 m, ridges <= 0.385, sod rolls
<= 0.34, road fans 0.035-0.13, all ABOVE. And it is authored AFTER the plank
field, so its only effect on a board is to bury it — the opposite of the fault.

## THE DEBRIS IS NOT ALL SAWN TIMBER, and it read as a lumber yard until it was

Reviewed on sight 2026-08-27: *"all the debris is now the wood, even the debris
that's from the roof or house — it's supposed to retain the colour of the
house/roof. Only the additional wood planks/splinters/logs are supposed to be
the wood material."* Two independent faults produced it and both are worth
knowing about, because they are in different files and one of them is
counter-intuitive.

**1. THE LOOSE PLANK FIELD HAD NO CLADDING CLASS AT ALL.** `planks.STOCK` was
stud / joist / sheathing / board / deck, all drawn on one sawn-timber map with
a per-class tint. That is the INSIDE of a house. What a debris photograph
actually shows is a mat of house-coloured cladding and grey roof slab with bare
framing between them, and the cladding is the largest single area of any
wrecked house. So:

  * `STOCK` gained **`siding`** (w 0.18) and `deck` was raised 0.06 → 0.15;
  * `scatter_from_wreck(skins={"siding": ..., "deck": ...})` stamps each such
    piece with the WRECKED HOUSE'S OWN wall and roof material name, from
    `modular_house.palette_skins(palette)`;
  * `build(skin_mats=...)` groups by **(class, skin)** and binds it, so a plate
    with four palettes costs four extra meshes and not four hundred.

**BIND THE TEXTURE, NOT THE MATERIAL.** The first attempt bound the kit's own
MDL (`modular_house.palette_material`) to the plank meshes and put a field of
BLACK roof slab in the corridor. Kit materials are UV-space and a plank is an
authored box with **no `st` at all** — which is the whole reason
`planks.wood_material` is triplanar. `modular_house.palette_texture` pulls the
base-colour map and the palette's albedo multiplier out of the kit material and
`planks.skin_material` re-projects it from world coordinates.

**AND SEARCH THE INPUT NAME BEFORE THE FILENAME.** `palette_texture` first
matched on the FILE name (`..._BaseColor.png`), which works for the standalone
kit materials and does NOT work for the RetroNeighborhood ones, whose maps are
called things like `T_Rooftiles_03_1K.png`. Both shingle palettes reported "no
base-colour map" and the roof slab silently fell back to pale timber. The MDL
input name is unambiguous where the filename is not, so it is tried first —
and normal / ORM / roughness maps are excluded explicitly, or a slab comes out
blue.

**2. THE BAKED ARCHETYPE TURNED ITS OWN HOUSE INTO WOOD, and this was the
bigger half.** `wind_flow._debris_material` binds each fragment either to the
module's own cladding texture or to bare timber, at `_BARE` per level. Measured
on the archetypes with a pxr probe (share of bound targets wearing
`Ash_Planks_BaseColor.png`):

| style | palette | pristine | roof_collapsed | leveled | swept |
|---|---|---|---|---|---|
| ranch | brick_red | 0.0% | 34.4% | **63.2%** | **73.0%** |
| villa | stucco | 0.0% | 32.9% | 70.1% | 70.4% |
| terrace | brick_red | 0.0% | 39.3% | 69.1% | 69.4% |
| wide_house | siding_cream | 0.0% | 24.1% | 67.8% | 72.5% |
| cottage | wood_white | 0.0% | 38.0% | 74.6% | 72.4% |

**Identical whatever the palette** — a brick ranch and a painted cottage
converge on the same pale plank. Pristine is 0%, which acquits both
`apply_palette` and `fracture` (which binds nothing at all: no material, no
GeomSubset, no soot wash). Two causes:

  * `_BARE` was 0.68 at `leveled` and 0.72 at `swept`. Two thirds of a levelled
    house being framing is true of its VOLUME and false of the AREA a camera
    sees, because cladding and roof covering are sheet goods. **Halved** —
    0.40 and 0.44.
  * the bare draw ran `planks._weighted(rng)` over the WHOLE cutting list, so
    once `siding` and `deck` existed it was handing **a third of every bare
    fragment** to the two classes that only exist to be skinned — and on the
    shared `planks.materials()` map those two are the `_TINT` fallback, i.e.
    the same Ash_Planks a stud gets. **`_BARE_STOCK`** now restricts the draw
    to stud / joist / sheathing / board.

The tempting fix — skin `siding` and `deck` per style in the bake — was written
and taken out again. A fragment of a wrecked house is **not** a loose board: it
already knows what it was clad in, and `_debris_material`'s other branch binds
exactly that. The skinned classes belong to the loose field, where there is no
module texture to inherit.

**AND THE RETAINED CLADDING WAS TILED AT 2.2 m.** `_debris_material`'s clad
path used `damage._pbr(..., scale_uv=(0.45, 0.45))` — `_pbr`'s generic
reference value — while the bare timber came through `planks.wood_material` at
1.1 m WITH a normal and an ORM map. A brick course is 0.075 m: at one tile per
2.2 m the bricks come out the size of a door and the wall reads as a smear,
so the timber won the eye twice over. Now 0.95 repeats/m, to agree with
`planks.skin_material`'s 1.05 — a fragment and a board lying beside it have to
be the same material at the same scale or the pile reads as two buildings.

**ANY CHANGE HERE NEEDS A REBAKE.** `_BARE`, `_BARE_STOCK` and the clad tile
are all consumed by `wind_flow.wreck_building` at BAKE time, not at assembly
time. `ARCH_KINDS=house` rebuilds the 48 house cells without touching the
trees.

## PROPS: what the wind CARRIES and what it FELLS are different lists

Also from the 2026-08-27 review — *"some props like street lights, mail boxes,
don't seem affected by the tornado"*, then *"why don't we get street light to
fall over"*. Three faults:

**1. `category` LIES ABOUT YARD PROPS.** `detail.suburb_yardplan` charges
everything it places to the `plant` budget — that is what its purse is
denominated in — so a mailbox, a wheelie bin and a patio table all ship with
`category: "plant"`. The corridor pass matches on category, so the entire
yard-furniture population came through untouched: every mailbox on the plate
standing perfectly upright in a levelled block. `emit(..., prop_kind=...)` now
stamps the truthful name alongside, and the corridor pass tests both. Nothing
moved between budgets.

**2. DELETING WAS THE ONLY OUTCOME.** Anything that failed the removal roll
stood there plumb. That is the right model for a timber fence — what a track
leaves is a line of post stubs and its boards somewhere else, and the boards
are already being authored by the plank field — and the wrong one for a 5 m
steel lamp standard on a cast base, which BENDS OVER. So there are two outcome
bands now, drawn from ONE number against nested thresholds (the same structure
`tornado.car_pose` uses, and for the same reason: testing a lean only on props
that already survived a removal roll multiplies the two probabilities and
leaves the tail standing anyway).

**3. THE SHARES DEPEND ON WHAT THE THING IS.** `CARRIED` (fence, bin, mailbox,
chair, table, planter, bench) goes away first: `p_gone = 0.18 + 0.86 i`.
`FELLED` (streetlight, sign, bus stop, play structure, swing, goal, hoop, bike
rack) goes OVER first: `p_down = 0.55 + 0.40 i`, with only `0.55 i (1 - p_down)`
carried off — it takes the very core to shear a standard off its base. A felled
pole also lies FLATTER, 62-100 degrees against a fence panel's 38-96, because
it hinges at the base.

`toss_prim` does the seating and it works on an INSTANCED prim: the ops live on
the instance root, which is still editable — the same reason
`apply_placements` sets `instanceable` AFTER authoring them.

## Vehicles: the mix, and forcing a rolled car for a review scene

*"Reduce the rate of spawning delivery truck/police car. Spawn more of the
normal cars."* Two knobs in `suburb_scene.build_cars`:

  * `street_livery_chance` **0.22 → 0.08**. One in five was chosen against a
    long street where it puts two or three trade vehicles and reads as
    ordinary. It does not survive the small plate or the eye: the four street
    assets are the most visually distinctive in the pool — a white parcel van,
    a yellow construction truck, a taxi and a squad car — against seven
    ordinary saloons.
  * the GMC motorhome carries `residential` (it is one of only three vehicles
    a person can be SEEN inside, so it must stay reachable) and was therefore
    one of seven entries drawn uniformly: **14% of every driveway on the
    plate**, about ten times the real rate. It now has its own `rv_chance`
    (0.03) band, taken out of the MIDDLE of the same single draw so the
    vintage and street bands at either end are untouched.

**`TOR_MIN_TIPPED` IS A REVIEW KNOB AND DEFAULTS TO 0.** `car_pose` puts
`p_move` at `0.10 + 0.62 i` and `p_tip` at `0.05 + 0.30 i^1.5`, and a 100 m
plate holds about eight in-track cars sitting mostly on the SHOULDER where
`i` is 0.2-0.3 — so `p_tip` is under a tenth apiece and **"no toppled car" is
the ordinary outcome, not a fault**. Measured on `TOR_SEED=5`: 8 cars in the
path, 0 moved, three builds running, because the scene is deterministic and
re-rolling the seed is a poor way to answer "show me what a rolled car looks
like". `TOR_MIN_TIPPED=n` forces the highest-intensity in-track cars over via
`car_pose(force="tip")` — which skips the DRAW and nothing else, so the
attitude mix, the throw distance, the march against the blockers and the
resting heading are all the ordinary model — and the banner says how many were
forced. Never set it on a scene that is being measured.

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

# The 3D scour relief

The overlay is the COLOUR. It is a flat translucent film on a flat lawn, so
from 40 m up the corridor is the right brown and the ground is still a
billiard table. `disaster/scour_relief.py` is the SHADOW: authored earth
standing on top of the overlay, which is where the whole read comes from at
the hour a survey flight photographs (low sun, long shadows).

**Keep both.** The relief without the overlay is heaps of earth on an unstained
lawn, which reads as molehills; the launcher gates it on `TOR_GROUND` for
exactly that reason.

## Six kinds, and what each one is in the photograph

| kind | what it is | where it goes |
|---|---|---|
| `arc` | a cycloidal ground mark: the groove a suction vortex cut, authored as its flanking spoil | the core only, on the RIGHT flank |
| `ridge` | a windrow — soil and torn grass swept into a low transverse bank | core and shoulders, biased LEFT |
| `mound` | a heap of turned-up subsoil | anywhere well scoured, biased LEFT |
| `sod` | turf peeled off in a mat and ROLLED UP like carpet | the coverage EDGE |
| `clod` | a lump of earth thrown clear | everywhere, densest in the core |
| `fan` | mud washed across a carriageway | roads only, at PAVEMENT grade |

`sod` is the one that explains the other five. The overlay says the ground is
brown; a rolled mat of turf lying at the edge of the brown says WHY, and
nothing else in this scene does. It is placed on the coverage **gradient**
(a hump at `sod_at = 0.46`, not the peak) because that is where turf actually
tears — in the middle of the core there is no turf left to peel, and a sod roll
on the centreline is a mat that came from somewhere it cannot have come from.
It takes the DARK root-mat material, not grass: a mat that has rolled has
turned its green side inward, and a green cylinder in a mud corridor reads as
a hedge.

## The arcs are the vortex, not ornament

Cycloidal marks are the canonical aerial signature of a violent tornado (Van
Tassel on Scottsbluff 1955; Fujita's suction-vortex analyses), and they are the
only ground feature that says which way the thing was TURNING rather than only
which way it was going. They are generated from the mechanism. A vortex
orbiting at radius `R` and rate `w` about a centre translating at `V` traces a
trochoid in track coordinates,

    along(t) = V t + R cos(w t + phase)
    cross(t) =       R sin(w t + phase)

and with `k = R w / V > 1` that curve LOOPS, which is what the marks look like
from the air. **Two things fall out of the same algebra for free**, and both
agree with what `tornado.py` already says about this scene:

- the vortex's speed over the ground is `V sqrt(1 - 2k sin(theta) + k^2)`,
  maximised at `theta = -pi/2` — the RIGHT flank, where rotation and
  translation add. A mark is cut only where that factor clears
  `arc_speed_min`, so the arcs land right of the centreline;
- material lofted on the strong side is carried across and dropped on the weak
  one, which is exactly why `tornado.DEFAULTS` throws debris `curl_deg` to the
  LEFT. So the deposition kinds are biased left by `left_bias` while the
  erosional one is biased right — **out of one model, not two guesses**.

The spoil banks to the outside of each turn with `bias = (sin(theta) - k) /
speed`, negative for every looping mark, so the two levees of a mark are never
the same size and the big one is always on the same side. Symmetric levees read
as a moulding.

## We cannot cut a groove, so we author its spoil

A real mark is a trench — Jarrell 1997 scoured to nearly half a metre. The
ground here is an opaque sheet at `_Z_GRASS` laid by `suburb_scene.apply_ground`
and **anything authored below it is hidden by it**: a trench floor at -0.10 m
is under the plate and the plate is what you see. Cutting a hole in the plate
is not available either — it is one merged sheet per region with the overlay
bands laid on top.

So an `arc` is what came OUT of the trench: two asymmetric levees flanking a
strip of bare ground at grade. With the mud running between them that reads as
a gouge from any altitude worth flying, and it fails only from ground level at
a grazing angle. The honest fix is a displaced ground mesh, which means
`apply_ground` taking a height field. Listed in the known gaps.

## What was taken from the EARTHQUAKE pipeline, and what could not be

`quake_flow._c_ground_response` and the passes under it already author real
earth round a building that leaned or sank, and its round-2 bench notes are the
record of what it took to make them read. Three lessons transfer verbatim:

- **a flat polygon is a paper cut-out**, however it is textured
  (`_c_soil_patch` raises the centre of every spilt fan for this reason).
  Nothing in the relief is flat; the lowest feature still stands 4 cm proud;
- **the material has to be world-projected.** These meshes carry no UVs at all,
  so a referenced UV-space `.usda` — the AEC `Dirt`, or a megascans pack bound
  directly — renders as one flat cream mat. It has to go through
  `damage._pbr(texture=...)`, and the tint has to go on **`diffuse_tint`**
  (not `diffuse_color_constant`, which the map replaces) with
  `albedo_desaturation` beside it, or no neutral multiplier will take the
  orange out of an orange mud map;
- **a smooth extrusion reads as a moulding.** `_c_clods` exists because the
  round-1 berm was a clean swept surface and looked machined. Loose material
  ON a crest, at more than one size, is what makes earth read as earth — which
  is why every arc emits clods along itself.

**What does not transfer**, and why the quake code could not simply be called:

- *framing.* Every one of those passes takes `m`, a mass dict with `W`, `D`,
  `yaw`, `cx`, `cy`, `z0`, and authors around a FOOTPRINT: `_c_perim` walks a
  rectangle, `_c_heave` wedges soil against one of its four sides, `_berm`
  rings it. A tornado track has no footprint — it is a field over the plate,
  and there is no `m` to pass;
- *cost.* They author ONE PRIM PER PIECE, which is right for a few dozen clods
  round one lean and does not survive a 500 m plate. So the relief follows
  `planks` instead: `scatter*` in pure Python, `build` merging everything into
  **one mesh per material class** — three prims for the plate.

## Traps this pass has already fallen into

- **The dome apex has to stay INSIDE the first ring.** Offsetting it by a flat
  fraction of `rx` put it outside the ring on a mound whose wobble pulled that
  ring in, which reverses the plan winding of the two triangles either side of
  it: a lit mound with one dark facet. It is measured against the ring's own
  radius now (0.35 of it).
- **A rejection you can retry is not a rejection.** `left_bias` was drawn
  inside the four-try point loop, where a rejected point is immediately
  re-rolled in the same cell — a bias of 0.55 came out as `1 - 0.45**4 = 0.96`
  and measured as 53% of mounds on the left against the 65% the knob asks for.
  One coin per CELL.
- **Test the pavement at the POINT, not at the cell.** Which passes run is
  decided per 8 m cell, which is right for a rate; taking the cell's answer for
  the placement put mounds on the asphalt and clods at LAWN grade inside a road
  surface 4 cm above it. And a windrow is up to 11 m long, so its whole
  polyline has to clear the kerb, not just its seed point.
- **Re-test the coverage at the point too.** `scour_coverage` cuts islands of
  surviving turf out of the band and they are smaller than a cell. A heap of
  subsoil standing on the green patch the vortex missed is the one artefact
  here that says "scattered" rather than "scoured".
- **Clip on GEOMETRY, not on an inset.** `apply_ground` lays its base sheet
  over exactly `region`, so a mound whose centre is legal but whose skirt is
  not hangs over the void. Rejecting centres against an inset instead needs the
  inset to be the biggest feature's reach, which prints a clean border round
  the whole scene.
- **A section's points must be ordered the same way for every kind.** The sod
  arch was walked right-to-left while the ridge and arc sections run
  left-to-right, which flips the winding for that kind alone — sod rolls
  invisible from above and lit from inside below.

## Judging it without a container

    python3 scene_gen/tools/tornado_png.py --config suburb_tornado

now prints the relief budget and draws every kind in plan. What is being
judged there is not "is there enough of it" but **is each kind where its rule
says it should be** — marks right of the centreline, heaps left of it, sod
rolls ringing the band rather than filling it. All three are obvious in the
plan and none of them is obvious in a render.

    python3 scene_gen/tests/test_scour_relief.py

is the ten-check offline suite: the placement rules as measured statistics, the
winding and normals face by face, the seating on both grades, the plate clip,
the budget, determinism, and the launcher's own `7a` block sliced out of the
script and run against stubs so the **z ladder at the seam** is pinned — the
relief based above the mud band, the road wash above the asphalt.

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

# People

Survivors, diggers, trapped and thrown figures, the epidemiology behind every
share, the planner API and the bench that verifies a partially-buried figure
all live in
[place-people-in-tornado-scenes](../place-people-in-tornado-scenes/SKILL.md).

Four things from there that bite THIS pipeline directly:

- **`disaster.people` does not transfer.** A wildfire gives hours and people
  MOVE (that module is a model of egress); a tornado gives minutes and nobody
  evacuates. Four of its six scenarios have no counterpart here — in
  particular `parking_refuge`, its LARGEST at 0.30, because Moore, Joplin and
  Mayfield had no public tornado shelters at all.
- **A levelled block is not a morgue.** Joplin's catastrophic zone held 4,716
  people and killed 122 — **97.4% survived**, and with the houses gone there is
  nothing left to occlude them. The corridor core should be the DENSEST part of
  the visible population, not the emptiest.
- **Do not drive victim deposition with `throw_field`.** The 78%-left
  deposition statistic is for lightweight debris lofted into the parent storm.
  Bodies are heavy debris and stay in the swath, and near-surface flow is
  CONVERGENT toward the centreline (Karstens et al., 104k tree falls). There is
  no published azimuthal distribution for victims.
- **The people pass runs LAST**, after the scour — every pass before it moves,
  deletes or re-materialises something, and a survivor is not debris.

Vehicle displacement rates (Paulikas et al. 2016, 959 vehicles) are in that
skill too, along with the nesting bug they exposed: those rates are
UNCONDITIONAL shares of all vehicles, so testing a tip probability only on cars
that already passed a move probability multiplies the two — measured, 5 of 25
cars moved and NONE tipped.

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
| `mounds_per_100m2` | 1.6 | ~250 heaps a plate — four or five in the 120 m square a drone frames at 60 m. At the first sweep's 0.55 it was one, which reads as a molehill somebody left |
| `clods_per_100m2` | 5.0 | the small-scale breakup that stops the heaps reading as blobs |
| `ridges_per_100m2` / `sod_per_100m2` | 0.26 / 0.32 | accents; a windrow is 3.5-11 m long and a sod roll 1.4-4.6 m |
| `min_coverage` | 0.24 | the relief holds tighter to the core than the mud does, because peeling turf takes less wind than gouging subsoil |
| `sod_at` / `sod_width` | 0.46 / 0.17 | a hump, not a ramp: turf tears at the EDGE of the scour |
| `arc_min_coverage` | 0.66 | cycloidal marks are an EF3+ feature; on a `peak = 0.92` track this holds them to the core |
| `arc_k` | 1.25-2.40 | `R w / V`; below 1 the mark is a wavy line, at 1 it cusps, above 1 it LOOPS |
| `arc_speed_min` | 0.55 | 0 draws the whole trochoid, which is a doodle |
| `left_bias` | 0.55 | share of right-flank CELLS that keep their deposition — 0.55 is ~65% of heaps on the left |
| relief heights | mound 0.10-0.53 m, arc levee 0.08-0.41 m, clod 0.10-0.55 m | tall enough to throw a shadow at low sun, short enough not to be boulders |
| `_SINK_M` | 0.02 | every rim is set below grade or the silhouette shows a hairline of background under it |
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
| `TOR_GROUND` | 1 | 0 disables the mud overlay AND the 3D relief that rides it |
| `MUD_TILE_M` / `MUD_BANDS` / `MUD_OPACITY_*` / `MUD_ISLANDS` / `MUD_GAMMA` | see `tornado.knobs_from_env` | overlay tuning |
| `SCOUR_RELIEF` | 1 | 0 disables the 3D relief alone, leaving the flat overlay |
| `SCOUR_HEIGHT` | 1.0 | the one to reach for first: scales everything vertical without moving anything. A clod and a sod roll scale bodily, because their height IS their size |
| `SCOUR_MOUNDS_PER_100M2` / `_CLODS_` / `_RIDGES_` / `_SOD_` | see `scour_relief.DEFAULT_KNOBS` | per 100 m2 of AFFECTED ground, weighted by coverage |
| `SCOUR_PAVE_WASH` | 0.30 | share of the rate that survives onto a carriageway, as low fans at pavement grade |
| `SCOUR_ARC_VORTICES` / `_ARC_MIN_COVERAGE` / `_ARC_SPEED_MIN` | 3 / 0.66 / 0.55 | how many cycloidal marks and how choosy they are |
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
  fifth attempt" in the wildfire skill is the upgrade path for both. The 3D
  relief is the OTHER half of that answer and is already in — it gives the
  corridor a silhouette the overlay cannot.
- **A cycloidal mark is spoil, not a trench.** Nothing here cuts DOWN, because
  the ground is one opaque sheet and everything below it is hidden. The fix is
  a displaced ground mesh, which means `suburb_scene.apply_ground` accepting a
  height field and the overlay bands riding it — a real change to the ground
  pass, not to `scour_relief`.
- **The relief is not a collider and carries no LOD.** Three merged meshes for
  a plate, so it is cheap, but a 1600 m plat would want chunking by region the
  way the plank field would. Nothing walks on it either: a 0.4 m spoil heap is
  scenery, not terrain, and a ground robot would drive through it.
- **Nothing is scoured where a house USED to be.** A levelled lot gets the same
  relief as open lawn, when in a real track the slab is swept bare and the
  scour starts at its downwind edge. The list is already in the launcher
  (`standing` is its complement); what is missing is a rule.
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
