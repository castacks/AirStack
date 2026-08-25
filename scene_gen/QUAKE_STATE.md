# quake pipeline — state

**Done & pushed** (`4f3c11b6`, `ed09e719`, branch `disaster-dataset`):
`mesh_damage` = the API, `quake.py` = the earthquake script, `damage_flow` = kit-only plugin.
`earthquake.py`/`fracture.py` deleted (debt noted in quake's header). Rungs compose mechanisms;
rubble sized in metres via `_fracture_hier` (~1,160 cells/building, capped at `MAX_CELLS_CAP=1200`).

**Landed 2026-08-25 (coasei-a1 [320d95], commits 93775b51 → 260fd9bf):**

1. **Archetype-backed buildings were EMPTY on `airstack up`** (bfb42c7e). `bake.export_object`
   rooted every archetype at a `Scope`; `apply_placements` references onto a typeless holder so
   the asset's type wins, a Scope is not Xformable, and the placement was skipped — two debris
   rings around two empty lots in `urban_quake_tiny`. Root is now an `Xform` (+ Z-up/metres
   metadata), a Scope-rooted reference is promoted in place (old libraries place without a
   re-bake), and `_measure_footprint_raw` expands `airstack://`. Every look/timing taken on an
   archetype preset before this was of a scene with no wrecked buildings in it.
2. **The settle was judging the wrong point** (260fd9bf, `scene_prep.settle_rigid_props`).
   Fragments have no xformOp, so travel and "through the floor" were measured at the BUILDING's
   origin. Same 872-fragment run, both rulers: origin 378 through-floor / 352 flung, centroid
   142 / 98, PhysX reporting 0 bodies moving. Reverting the "failures" to their authored pose
   put them back inside the intact shell — that was Mission 3's mid-air ghost. Now: centroid
   travel, rest-checked chunked stepping (ceiling from the highest piece's fall time),
   fragments judged on horizontal travel with a drop-scaled limit and DEACTIVATED when over
   it, infinite half-space backstop under the ground (0.3 m slab pieces at the 20 m/s cap
   cross a zero-thickness quad in one step: through-floor 142 -> 0, max travel 131 -> 52 m).
3. **Tilt-and-sink stand-in only beyond the mesh-damage budget** (`disaster_stage`,
   `mesh_damage.damage_budget`). A building about to be cut keeps its true pose — pitched
   6 deg about its corner an 80 m tower had one end authored underground.
4. **Cut faces show a fracture core** (`mesh_damage.core_material`, `CORE_LOOKS`): one shared
   OmniPBR per material kind (masonry = worn-brick megascans, world-projected so caps need no
   UVs; timber/steel flat). `cap_mat` threads through `_clip_by_plane`; `quake.shatter` and
   `apply_to_stage` opt in. One material prim per scene (renderer compiles each distinct MDL).
5. `wall_for`/`cells_for` duplicates deleted from quake.py (685ca60f).
6. **Archetype-backed buildings rendered as BLACK BOXES** — the bake re-authored every
   texture path verbatim (`Textures/Building_C_Proxy_BaseColor.png`, relative to the
   source layer on Nucleus) into `assets/archetypes/earthquake/`, where no `Textures/`
   exists; a missing `UsdUVTexture` file is a silent fallback, and the diffuse fallback is
   (0, 0, 0). `bake.validate` said "bound", which is not "textured". Now `anchor()` resolves
   a relative path against the layer that authored it when `resolvedPath` is empty,
   `bake.unresolved_textures()` counts maps that resolve to nothing, and the Stage A
   exporter REJECTS an archetype with any. Confirmed not lighting: identical under a key
   light (`_damage_lab`-free evidence: `~/docker/isaac-sim/logs/quake_showcase_r2/`).
7. **Review captures**: `SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/<name>` on the
   `scene_launch_script.py` line writes overview / street obliques / wreck close-ups / one
   victim (`review_snapshots`), with a review-only key light; cameras pick the bearing with
   the most clearance from building FOOTPRINTS (centre-distance put the eye inside towers).

**Measured, 2026-08-25, `urban_quake_live` seed 42 (BG_Building_F partial_collapse, 884 loose):**
rest 703, through-floor 0, flung 181 at a limit of 1/3 the drop (raised to 1/2 in 260fd9bf,
not yet re-measured), still-moving 0, settle 8 s sim / 14.6 s phase. Findability gate
(`findability.py --config urban_quake_tiny --seeds 1,2,3`): PASS, 12/12 clear.

**Open after this pass, in priority order:**
0. **THE BAKE'S SETTLE DID NOT SETTLE — fix in tree, UNVALIDATED until the next re-bake.**
   `archetypes/bake.py` now builds, settles, exports and UNLOADS one cell at a time
   (`_settle_cell` / `_export_cell` / `_unload`, `SETTLE_STEPS` 420 -> 900 as a ceiling with
   `settle.run`'s early exit) and `write_manifest` MERGES (`library.merge_manifest`). Needs
   one `bake_cli --config urban_quake_tiny --used-only` in the container to confirm: expect
   no Vulkan OOM, `[settle] drop median` clearly negative per cell, and pancaked archetypes
   that are piles. The measured failure it replaces: `bake_cli --config urban_quake_tiny --used-only`
   (2026-08-25): 16 cells resident on one stage, GPU out of memory from cell 6 onward
   (26,888 Vulkan OOM errors), then ONE settle over 5,064 bodies: 975 s, 4825 still moving
   at 420/420 steps, drop median -0.02 m, spread max 179 m. Nothing fell, so every baked
   tower is a cracked plate standing at full height — and always was. Wants settle + export
   + unload PER CELL (a cell is ~900 bodies and settles in 8 s in the scene path), or the
   renderer off; and `write_manifest` must MERGE, not replace (a `--used-only` bake dropped
   the showcase's other 8 types; records reconstructed by hand, marked `stale: true`).
1. **Re-bake the archetype library** — tiny's 14/16 done (stumps missing), textured; On disk it predates ed09e719 (quake wiring) AND the
   core material AND the Xform root. `bake_cli.py --config urban_quake_tiny --used-only`
   (31 archetypes under Kit's packing). Blocked 2026-08-25 on the container being `down`ed.
2. **G2 gallery per seed** — nothing rendered yet since the fixes; the live scene never got a
   snapshot (`scene_launch_script.py` has no `SNAP_DIR`; add one or use `targets_showcase`).
3. **The rest check never triggers** on a real pile (455/456 frames used, PhysX asleep): a
   body somewhere creeps > 4 cm/s for the whole ceiling. Harmless (ceiling bounds it) but the
   early exit Mission 2 wanted is not happening on collapses; parked-car scenes do exit early.
4. **The stand-in's pitch is about the CORNER**, so a beyond-budget tower still has one end
   metres in the air or underground. Wants a pivot at the footprint centre — or a sink only.
5. `urban_quake_tiny` places 4 buildings where its header promises 6 ("8/9 large buildings
   have no block that fits") — packing vs measured footprints, not a unit bug. G2 scope.
6. **Load cost of a textured, placed wreck** (post-fix warm tiny 55 s vs 25 s pre-fix, which
   had EMPTY wrecks): renderer warm-up 25.8 s with the same 18 MDL materials — driven by the
   ~60 fragment meshes per wreck, not by material count — and `settle cook 6.7 s`, PhysX
   cooking a convex hull per fragment mesh on every launch for geometry that is fixed at
   bake time. Levers (Mission 2's read, unmeasured): cook colliders once at bake and store
   them in the archetype; merge fragments that ended up touching into one mesh at export;
   cheaper approximations for small debris.
7. Older items below still stand (moment test, pile inflation, slab over-fragmentation).

**Done, uncommitted (2026-08-24 pass, landed in 93775b51):**

1. **Floating slabs — fixed, and the fix was in the wrong half of the graph.**
   `unsupported` used to hand the RETAINED remainder (faces under `support`, never cut)
   to the support graph as unconditional ROOTS, i.e. as ground, and to test contact by
   AABB overlap. Both are wrong for a banded mechanism: the block above a soft storey is
   untouched, untouched is not supported, and its box overlaps every fragment in the
   storey below it so one surviving chunk anywhere declared it standing. Rewritten as
   ONE graph over fragments *and* connected components of the retained remainder, with
   contact resolved on voxel-hashed surface samples (`SUPPORT_CELL_M = 0.5`) instead of
   boxes. A retained component with no load path is cut out of its source prim, authored
   as a single body (`report["slabs"]`), and settled — a soft storey now drops its block
   whole. Slabs are excluded from `consume` (they are the largest pieces, so the
   large-biased draw deleted them first) and get `convexDecomposition` colliders.
2. **Voronoi seeds pushed OFF the corners, not onto them.** `CORNER_GAIN` had the sign
   backwards: a seed is the *centre* of a fragment and the cracks are the bisectors
   between seeds, so seeding a corner buries it inside an intact cell. Replaced by
   `CORNER_AVOID = 0.9` / `CORNER_FLOOR = 0.1`, which puts the pair of seeds either side
   of an edge and their bisector along it. Pinned by
   `test_the_cracks_run_along_the_edges_not_around_them`, which measures distance to the
   nearest bisector for high- vs low-`cornerness` faces.
3. **`support` is now a MECHANISM property** (`Mech.support`, dominant mechanism wins).
   Every rung carries `crack` as its background, `crack` puts ~0.2 damage over the whole
   building, and a flat 0.12 threshold therefore diced every tower on every rung — the
   block a soft storey is supposed to drop intact was confetti before the settle began.
   `crack` 0.30 / `soft_storey` 0.42 / `shear_off` 0.28 / `pancake` 0.12.
4. **The pile had nothing to land on.** `collapse` handed the settle only the
   ANCHORED FRAGMENTS as static colliders; the retained, never-cut part of the building
   was not a collider at all. On `shear_off` that was 825 rigid bodies against TWO
   statics — the sheared wing fell through the half meant to hold it and the settle spent
   its whole 4,000-step budget with 566 bodies still moving. `fracture_to_stage` now
   reports `standing`; `settle._apply_collider` gained `skip_dynamic` and the static pass
   moved after the dynamic one, which is what makes it safe to hand over a whole
   single-mesh building whose fragments are its own children.
5. **`settle` reports `drop_median`** as well as the mean, and prints both. The mean is
   dragged positive by a handful of launched pieces and stops describing the pile.
6. **`hash()` was salted, so nothing seeded off it was seeded.** `bake.py`'s `_rng`
   docstring claimed "re-baking one archetype reproduces it exactly"; PYTHONHASHSEED is
   random per process, so it never did — the bake looked right and was different every
   time. `mesh_damage.stable_seed` (crc32) replaces every `abs(hash(...))` in
   `archetypes/bake.py`, `disaster/damage.py`, `tools/damage_spread.py` and
   `tools/quake_mechanisms.py`, pinned by `test_seeds_are_the_same_in_every_process`.
7. **`quake.at_level(plan=...)`** takes an explicit `[(mechanism, share), ...]`, so a
   composition the ladder does not name can be rendered. `tools/quake_mechanisms.py`
   uses it for the mechanism grid; `tools/_lab_stage.py` holds the shared Isaac
   boilerplate.

**The retained remainder fragments too finely.** bg_f reports up to 76 slab bodies on
one building (`pancake+shear_off`). Every one really has lost its load path, so this is
not a correctness bug, but 76 rigid bodies where a handful of blocks belong is both
wrong-looking and expensive. `_voxel_components` at `SUPPORT_CELL_M` is splitting on
gaps a building does not have; wants a dilation before labelling, or a floor that merges
scraps into the nearest component.

**Verified on the sheet** (`_damage_lab/mechanisms/sheet.png`, `tools/quake_mechanisms.py`):
nothing floats in any cell of the BG_Building_C row. `soft_storey+crack` — the cell that
used to render as a roof plate hanging over a pile — topples the block above the failed
storey onto its own rubble as one body.

**Open, in priority order:**
1. **The graph tests REACHABILITY, not moment.** `shear_off` alone bites a hole out of
   the middle of the tower and leaves the mass above it cantilevered off the standing
   core: a legal load path and not a building anyone would leave up. Wants a span test
   (free length against depth, per component), not a better contact test.
2. **The pile INFLATES, and the cap that controls it is not a visible fix.**
   Measured on bg_f `shear_off+crack` (600 loose bodies, identical cut and seed, only
   `settle.prepare`'s `maxDepenetrationVelocity` varied — now a `depenetration` argument
   on `settle.run`/`prepare`, forwarded by `quake.collapse`):

       cap m/s   drop median   drop mean   spread mean   spread max   still moving
       0.6         +5.23        +12.81        39.0         194.9          517
       0.05        +1.87        +12.85        32.5         182.3          428
       0.005       +0.78         +9.80        33.3         170.2          450

   The median tracks the cap, so the BULK rise really is depenetration: convex hulls of
   Voronoi-cut SHELL fragments overlap by metres (a cell holding bits of two opposite
   walls has a hull spanning the void between them), and PhysX oozes them apart for the
   whole budget. But the three renders are nearly indistinguishable
   (`_damage_lab/depen/sheet.png`) and the MEAN barely moves — so lowering the cap buys a
   better number and not a better picture. The default is deliberately LEFT AT 0.6 until
   someone decides on evidence that the geometry improves. The outliers (mean ~+10,
   spread max 170 m) are a second mechanism the cap does not touch.
3. **The displacement statistics disagree with the renders.** `shear_off+crack` reports
   `drop_median +1.40 m` and `spread max 125 m` while rendering as an ordinary rubble
   pile. `spread max` is a genuine handful of launched outliers; the POSITIVE median is
   not yet explained and is a question about the measurement (fragments are authored with
   no xformOp, so `_positions` reads the body frame rather than the geometry) before it is
   a question about the settle. Do not tune against either number until this is settled.
4. **`settle.run` has no runaway net, and the repo already has one.**
   `simulation/isaac-sim/utils/scene_prep.py:settle_rigid_props` documents this exact
   failure — pieces spawning interpenetrating, PhysX resolving with a separating impulse,
   "debris ends up scattered over many building-widths" — and defends with BOTH
   `maxDepenetrationVelocity` (which `settle.prepare` has, at 0.6) and `max_travel_m=12`,
   which reverts any prop that ends further than that from where it started. `settle.run`
   has only the first, which is why `spread max` reads 115-125 m. Porting the safety net
   is a small change and would make the spread numbers mean something again.
5. **Cap binds on every asset**, so rubble size still varies with building size. Uncapped
   wants ~3.5k cells (bg_c) / ~5.6k (bg_f) at 200–400 s/rung today.
6. `archetypes/bake.py` never re-baked since wiring `quake` in; `damage_flow` still
   duplicates the cutter and has NOT had the support-graph fix.

**Tools:** `tools/damage_spread.py` (ladder grid + timings), `tools/quake_mechanisms.py`
(one mechanism per column, then the pairs), `tools/quake_preview.py` (one ladder),
`tools/render_damage_gallery.py <manifest> --res 560` (sheet). Run from repo root:
`uv run --env-file .env.host python scene_gen/tools/...`; render with `uv run --script`.
