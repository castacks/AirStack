# quake pipeline — state

**Done & pushed** (`4f3c11b6`, `ed09e719`, branch `disaster-dataset`):
`mesh_damage` = the API, `quake.py` = the earthquake script, `damage_flow` = kit-only plugin.
`earthquake.py`/`fracture.py` deleted (debt noted in quake's header). Rungs compose mechanisms;
rubble sized in metres via `_fracture_hier` (~1,160 cells/building, capped at `MAX_CELLS_CAP=1200`).

**Done, uncommitted (this pass):**

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
