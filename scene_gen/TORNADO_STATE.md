# tornado pipeline — state

**Done** (not yet rendered — see Open #1): `disaster/tornado.py`, the second
per-disaster script on the `mesh_damage` API, mirroring `quake.py` rung for
rung. Registered in `mesh_damage.DAMAGE_SCRIPTS`, so `apply_to_stage`,
`damage_one`, `archetypes/bake.py` and `tools/damage_spread.py` all route to it
without a type check — the `if self.disaster == "earthquake"` branch in the
baker is gone.

**The four rungs** are `levels.LADDERS["tornado"][STRUCTURE]`, and each is a set
of mechanisms over wedges of the plan (`RUNG_PLAN`), worst first:

| rung | mechanisms | what it is |
|---|---|---|
| `roof_stripped` | `peel` | covering off the top sixth, nothing under it |
| `roof_lost` | `roof_off` + `peel` | deck and rafters, plus the top course of wall |
| `walls_breached` | `rack` + `roof_off` + `peel` | windward wall out to the sill, leeward wall standing and fully textured |
| `swept_clean` | `sweep` | crushed to the slab and carried off |

Measured on a 12 x 10 x 8 m house, seed 3, heading 0:

    rung              mean   base  freed  consume
    roof_stripped     0.11   0.00   0.04     0.00
    roof_lost         0.17   0.00   0.14     0.20
    walls_breached    0.31   0.12   0.35     0.35
    swept_clean       0.77   0.60   1.00     0.70

**Three things this does that `quake` deliberately does not**, all of them the
storm signature rather than tuning:

1. **One bearing for every mechanism.** `quake.field_for` spreads its
   mechanisms around the plan on a golden angle; here they all take the storm's
   heading and are masked to wedges about the windward face. Without this a
   street is a row of unrelated bomb sites. `mesh_damage.apply_to_stage` now
   forwards `heading_deg` to a SCRIPT as well as to `damage_building` — it
   previously only reached the unscripted path.
2. **`carry_off` draws from the SMALL fragments**, where `quake.consume` draws
   from the large. A collapse pulverises its big panels into its own pile; a
   storm lofts what it can lift and leaves the heavy slabs. Same deletion,
   opposite end of the distribution.
3. **`floor` on the wind profile** (new, `mesh_damage._field_wind`, default 0 so
   nothing else moves). `t ** exponent` is zero at the sill for any positive
   exponent, so before this no rung could fail a ground floor however hard it
   was driven, and `swept_clean` rendered as `roof_lost` standing on an
   untouched storey. `sweep` carries 0.75 of its load to the ground.

**What the first render found** (four bungalows, tornado column only):

1. **`roof_stripped` produced NOTHING on two of four houses.** `peel`'s
   `exponent` 6.0 was "the top sixth", tuned against a 70 m tower where that is
   a storey; on a 6.4 m bungalow it is the ridge line, and only 0.8% of the
   Chase's faces cleared `support` — too small a soup for one Voronoi cell. The
   eaves on these models are at t ~ 0.5 and the face count above them is thin,
   so the band is now fitted to measured geometry rather than to a fraction
   that sounded right. 22-36 cells on every house now.
2. **The heavy rungs EXPLODED.** +7.5 m mean vertical displacement on
   `walls_breached`, +19.8 m on `swept_clean` — the wreckage going up. Ejecta
   is a rigid TRANSLATION applied before the settle, proportional to each
   fragment's own damage, so at a throw of 2.2x the radius neighbouring cells
   were driven one to two metres through each other and PhysX resolved it by
   launching the pile. `throw` is now 0.10-0.60 and `blast` 0.0-0.5.
3. **The light rungs floated.** Same cause, opposite end: `roof_stripped` frees
   three panels off an otherwise intact house, and a 2 m throw put them inside
   the roof they came off, so they were ejected upward (+4.8 m median). The
   gentle rungs now get the SMALLEST throw, because by `swept_clean` there is
   nothing left standing to be thrown into.

Measured after the fix (drop median, metres — negative is material going down):

    rung              chase   dawson   ryder   gilchrist
    roof_stripped     -7.35    -0.45   -1.64      -0.16
    roof_lost         -1.47    -0.52   -2.96      -0.66
    walls_breached    -2.91    -0.64   -2.02      -0.46
    swept_clean       -2.34    -0.38   +3.82      -0.92

**Materials — a timber house no longer breaks like masonry** (`--set material`,
`_damage_lab/tornado_material/sheet.png`):

`mesh_damage.MATERIALS` restores the `Material` table `quake.py`'s header lists
as debt 1 ("a brick fragment came out a lump, a timber a plank"). Two knobs:
`wall_m` (0.50 masonry / 0.15 timber) and `grain`, a per-axis multiplier on the
cut spacing normalised to unit geometric mean so it changes a fragment's SHAPE
and not its size. `tornado.MATERIAL` defaults to timber.

The anisotropy is a CHANGE OF COORDINATES, not a new cutter: seeds are thinned
on `(p - q) / g` and the whole cut runs with vertices and seeds divided by `g`,
then maps back. A diagonal affine map takes planes to planes, so every bisector
is still a plane and every closed fragment stays closed — `_cap_fan`,
`_clip_by_plane`, `_kd_groups` and the radius early-exit are untouched.

    median fragment bbox at walls_breached
    material   long   mid  short   long/mid  mid/short
    masonry    2.47  1.96   1.62      1.19      1.20   <- a lump
    timber     4.82  1.83   0.96      2.58      1.90   <- a plank

**Timings in this file and in the gallery sheets are STALE, in our favour.**
`coasei-db` rewrote `_clip_by_plane` as a masked numpy gather and swapped the
`Vt` array construction for `FromNumpy` (uncommitted, same working tree). Every
per-cell wall-clock baked into `_damage_lab/*/sheet.png` predates it. Verified
on this pipeline rather than taken on trust:

    Chase, timber, swept_clean    238.6s -> 40.6s   (5.9x)
    output identical: 206 cells, 62 loose, drop +0.11/+0.46 m

and the masonry/timber fragment-shape table below reproduces to the last
decimal on all eight rows. That is the check worth having: their equivalence
test used random soups, which never exercises the anisotropic path, where the
vertices handed to the clipper have already been divided by `grain`. It is
byte-stable through it.

Consequence: `mesh_damage.MAX_CELLS_CAP = 1200` is a COMPUTE ceiling whose
justification moved by ~7x. It is shared with quake, so raising it is not a
tornado decision — but the 1.4 m debris this ladder asks for stops being
budget-bound long before the old cost did, and `cells_capped` in the report is
what to watch.

**RETRACTION — `drop` AND `spread` DO NOT MEASURE WHAT I READ THEM AS, AND
SEVERAL FINDINGS ABOVE ARE VOID.**

`settle._positions` samples `ExtractTranslation()` of each body's
local-to-world transform — the rigid body's ORIGIN. Fragments are authored
with an identity transform carrying their points in root-local space, and on
these assets the geometry sits ~62 m from that origin (measured: 60.5-62.1 m
on every fragment of the Colbert). So a body that merely ROTATES while
settling swings its reported position tens of metres without its geometry
moving at all, and `drop_mean` / `drop_median` / `spread_mean` are dominated
by that lever arm rather than by where the debris went.

Checked against the exported USDs, which carry the settled geometry:

    cell                                    fragments below z = -0.5 m
    colbert masonry walls_breached  (-11.56)      0 of 96
    chase   masonry walls_breached  ( -1.10)      0 of 83
    colbert timber  walls_breached  ( +5.01)      0 of 100

Zero, in all three, including the two I called broken. Every pile is resting
on the ground (5th-percentile min-z = -0.36 m, which is the asset's own dip
below the origin). And `tools/floor_probe.py` drops a fragment-sized mesh box
onto the real `settle.prepare` floor from 1, 5, 15, 40 and 70 m: it holds at
every height, up to a 37 m/s impact and 0.62 m of travel per step.

SO THE FOLLOWING ARE WITHDRAWN:

  * "fragments tunnel through the settle floor" — they do not. The floor is
    neither inert nor tunnelling at any speed this pipeline produces.
  * "the heavy rungs exploded (+7.5 m, +19.8 m)" and "a plank pile launches"
    — nothing launched. Those numbers are rotating origins.
  * "the light rungs float (+4.8 m)" — same artefact.
  * the THROW and BLAST retune was justified by those numbers, so its
    JUSTIFICATION is void even though the values may still be reasonable. It
    has to be re-argued against a valid metric before anyone treats
    `throw`/`blast` as tuned.

WHAT SURVIVES, because it never depended on `drop`:

  * the ladder and rung recipes, which are cell and release counts;
  * `roof_stripped` producing nothing on two of four houses (cell counts) and
    the exponent/support refit that fixed it;
  * the whole `MATERIALS` result — masonry lumps vs timber planks — which is
    fragment bounding boxes;
  * `still_moving`, which compares a body against ITSELF over 20 steps and so
    is not confounded by where its origin is. The timber non-convergence at
    `walls_breached` (43 of 62, 41 of 55) is real.

THE FIX, and it is shared: measure the GEOMETRY, not the origin. Either sample
the fragment's own point centroid, or set each body's origin to its centroid
when authoring. Until then, `still_moving` and `steps_used` are the only
settle statistics worth reading on these assets.

**Open, in priority order:**

0. **A PLANK PILE NEVER REACHES REST — it does not "launch", and I had that
   wrong.** `still_moving` is the number that settles it: at `walls_breached`
   the timber runs end with 43 of 62 (Colbert) and 41 of 55 (Chase) bodies
   still moving at the 4000-step ceiling, so the reported `+5.01` / `+2.51 m`
   is a mid-flight sample and not where the pile came to rest. Every masonry
   run and every `swept_clean` run converges (`still_moving = 0`). The timber
   solve is also FASTER while failing to converge (34.1s vs masonry's 54.1s),
   which is the jammed-pile signature `quake.py`'s header already describes
   from the other side: "a pile that actually falls reaches rest, while one
   that is jammed grinds against itself for the whole step budget" — except
   here it grinds cheaply, because thin interlocking planks have few contacts
   each.
   So the lever is convergence, not throw. Raise `STEPS` and re-read
   `still_moving` BEFORE touching `rack`'s throw; the throw hypothesis below
   was formed while I was reading `drop` alone and is unsupported.
0a. **Colbert masonry `walls_breached` is a separate, still-unexplained cell:**
   converged (`still_moving = 0`) yet -11.6 m median with 30.7 m of horizontal
   spread on a 12 m house. Converged-but-underground is not the same failure as
   not-converged, and it is not explained by fragments starting below the floor
   — measured, only 7 of 69 start under z=0 and by at most 0.16 m.
0b. **A PLANK PILE LAUNCHES WHERE A LUMP PILE SETTLES.** Same house, same seed,
   `walls_breached`: -1.1 m as masonry, +2.5 m as timber (Chase); -11.6 m and
   +5.0 m (Colbert). `swept_clean` is fine either way (-2.9 -> +0.1 Chase,
   -2.0 -> -3.3 Colbert), so it is this one rung. The inset was the obvious
   suspect and it is NOT the cause — `fracture_to_stage` grew `gap_m` to inset
   by an absolute distance instead of a fraction, and it made things worse
   (+12.8 m), identically with and without a floor, which proves the clamp
   never bound. Reverted to off, measurement kept in `tornado.GAP_M`. The live
   suspect is the ejecta THROW: it displaces each fragment proportionally to
   its own damage, and a 4.8 m plank driven a metre past its neighbour overlaps
   far more of it than a 2.4 m lump does. Test by dropping `rack`'s throw to
   ~0.1 before touching anything else.
1. **THE LADDER IS NOW SUBURBAN-SPECIFIC, and that is a trade I made on
   purpose without being able to check the other end.** `peel`'s exponent went
   6.0 -> 2.6 to put the band on a bungalow's roof, and a fraction of the
   height means something different on every building — which is the exact bug
   the first version had, fixed in one direction only. Measured on
   `MBuilding01` (29 m): `roof_stripped` now cuts 420 cells and frees 253
   pieces, i.e. the top ~12 m of a tower, which is not a stripped roof.
   THE RIGHT FIX IS A WORLD LENGTH, the same move `DEBRIS_M` already makes for
   fragment size: a roof is 2-3 m deep on a bungalow and on a tower alike, so
   the band should be `1 - roof_depth_m / height` per building rather than a
   fixed exponent. Until then the tornado ladder should be considered tuned for
   `--set suburban` only.
0a2. **`roof_stripped` is seed-fragile on the Gilchrist.** `peel` releases only
   ~20-70 of its 89k faces (0.02-0.08% across 8 seeds, against the Colbert's
   0.4% and the Chase's 1.7%), so whether any Voronoi cell's MEAN clears
   `release` is a coin flip: the same asset gave 22 cells under one row key and
   0 under another, the seed being the only difference. Dropping `peel` to
   support 0.17 / release 0.42 improves it ~3x but puts its `support` under
   `roof_off`'s and inverts the ladder, so the two want re-spacing together —
   or the world-length band above makes it moot.
0a3. **Fragments tunnel through the settle floor on some assets.** The Colbert
   reads -11.6 m MEDIAN at `walls_breached` on a 6.5 m house, so most of the
   pile finished ~8 m underground; the Chase on the same rung reads -1.1 m.
   `settle.prepare` gives the floor a `UsdGeom.Plane`, which is infinitely thin
   — small fast fragments pass through it. Wants CCD on the bodies or a box
   floor with depth.
0. **The Ryder still explodes at `swept_clean`** (+3.82 m median, the one bad
   cell of sixteen; it was +19.8 before). Asset-specific — the Chase and Dawson
   are fine at the same settings — so the next step is `sweep`'s throw 0.60 ->
   ~0.35 and blast 0.5 -> 0.2, which is the same lever that fixed the other
   three.
0b. **The walls survive more than `walls_breached` claims.** `rack` is supposed
   to take the windward wall to the sill; in the render the siding is still
   standing on the Chase and the Ryder at that rung and most of the visible
   loss is still roof and debris. Suspect the wedge (`share` 0.45 about the
   windward bearing) is narrow enough that the wall's own faces mostly fall
   outside it. Worth a `--levels walls_breached --plan` mechanism-only render
   before touching the numbers.
0c. **A few fragments escape through the floor plane.** On the Ryder the light
   rungs read median -1.6 / -3.0 m against a MEAN of -15.3 / -18.1 m, so one
   or two bodies of ten fell far past the `settle` floor at z=0. These assets
   dip slightly below z=0 (the Chase's faces run to -0.34 m), so a fragment
   can start under the plane. Cosmetically invisible — the gallery camera is
   framed on the pristine building — but it makes the mean useless, which is
   why `damage_spread` now prints the median first.
1. **RENDERED, and three bugs came out of it** — all three now fixed, see
   "What the first render found" below. `_damage_lab/tornado_suburb/sheet.png`
   is the current sheet (4 bungalows x 4 rungs + pristine).
   `tools/quake_preview.py` is still earthquake-only — it wants a `--disaster`
   flag rather than a copy.

1b. **THE GALLERY CAMERA MUST BE WINDWARD, and its default is not.**
   `render_damage_gallery.py` defaults to `--az 38`, and `compile_tornado`
   defaults the storm to `heading_deg: 35` — so the camera sits almost exactly
   downwind and looks at the LEE face, which is the one side a windstorm
   deliberately leaves alone. Rendered that way the whole ladder reads as "the
   roof was diced in place" and the rungs are hard to tell apart; rendered from
   `--az 215` (heading + 180) the same USDs read as peeled covering ->
   sheathing exposed -> structure opened -> pile. Nothing about the geometry
   changed. The renderer does not know the storm bearing, so this cannot be
   defaulted from inside it; either pass `--az` or teach the manifest to carry
   the heading. An earthquake has no bearing and is unaffected, which is why
   the default was never wrong before.
2. **Floating slabs** (`QUAKE_STATE.md` #1) applies here too and probably worse:
   a roof plate is exactly what `peel` frees, and `roof_stripped`'s whole
   appearance is that plate landing downwind rather than hanging over the
   house.
3. **`throw` vs the settle.** Ejecta displaces released fragments before PhysX
   sees them, so at `sweep`'s 2.2x building radius a swept house's debris
   starts ~26 m downwind and may land on the neighbouring lot's geometry —
   correct for a tornado, but nothing has checked it does not intersect the
   building next door. `BLAST` is 1.5 (against the quake's 6.0) precisely
   because the separation is already done by then.
4. **`support` is per-mechanism**, mirroring the `Mech.support` that landed in
   `quake.py` this pass for the same reason: every rung below the top carries
   `peel` as its background, and a flat 0.12 threshold would cut the whole
   house on a rung whose name says only the roof went. `peel` 0.34 /
   `roof_off` 0.24 / `rack` 0.20 / `sweep` 0.10, dominant mechanism wins.
   These are guesses off the field arithmetic and have not been rendered.
5. **`wall_for` / `cells_for` are duplicated.** They are asset-geometry
   budgeting, not disaster semantics, so they now live in `mesh_damage`
   (`WALL_M`, `REF_RADIUS_M`, `MAX_CELLS_CAP` with them) and this script uses
   them from there. *(Resolved 2026-08-25: `quake.py`'s copies deleted; it
   now calls `md.wall_for` / `md.cells_for`.)*
