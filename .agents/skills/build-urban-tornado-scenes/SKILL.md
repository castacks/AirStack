---
name: build-urban-tornado-scenes
description: TORNADO damage to URBAN buildings (GAC / downtowncity slices, and the same element table for kit styles) — the T0-T4 ENVELOPE ladder that never demolishes a skyscraper (glazing -> envelope -> stripped -> breached; chunks out of the windward side land in the street as debris), the near-surface wind-DIRECTION model (`tornado.wind_at`: cyclonic tangential + translation + convergent inflow) that decides which faces are hit, the pure planner / USD-apply split (`disaster/tornado_urban.py` / `tornado_urban_usd.py`), the city dry run over the fire levels' Kit dumps (`tools/tornado_city_dry_run.py`, `downtown_tornado_1500*` presets), the bare-python container probe that runs the chain on a REAL slice, and the bug catalogue: the shader-connection glass matcher, volume-derived fragments 600 m thick, massless-ballistic reach, one-bay towers on the regular grid, and the fire's 232 m height cap leaving supertalls pristine on the centreline. Round 1 (2026-09-01) is OFFLINE — nothing here has been rendered; the bake launcher and city assembly are the next round. Read before touching disaster/tornado_urban*.py, disaster/tornado_city.py, tools/tornado_city_dry_run.py, tools/tornado_urban_probe.py or the downtown_tornado presets.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Build Urban Tornado Scenes

## Read these first

* [build-tornado-scenes](../build-tornado-scenes/SKILL.md) — the suburb
  track model this reuses unchanged (`tornado.intensity_field`,
  `frame`/`from_track`, the floating-debris causes).
* [model-tornado-paths](../model-tornado-paths/SKILL.md) — the sourced
  verdict on the path model and the knob values (curvature, ramps, the
  `wind_at` constants). Written in this round from the literature.
* [slice-buildings-into-kits](../slice-buildings-into-kits/SKILL.md) — the
  piece grid every recipe here damages, and its "To add disaster X"
  checklist, which this pipeline followed.
* `scene_gen/_plans/urban_tornado_plan.md` — the design brief (ladder,
  wind model, plan schema, ownership) and `_plans/urban_tornado_research.md`
  (R2: EF-scale DOD tables, Lubbock / Fort Worth / Joplin, debris regimes),
  `_plans/tornado_path_research.md` (R1: paths, wind decomposition).

## The constraints that shaped everything (user, 2026-09-01)

1. **No completely demolished skyscrapers.** Chunks out of the envelope,
   fallen as debris "like the tornado hit the side of it" — never a
   pancake, a storey loss, a lean or a sink. The record agrees AND the EF
   scale agrees: DI 18 (mid-rise) and DI 19 (high-rise) have **no collapse
   DOD at all** — their top rung is "permanent structural deformation" at
   EXP 210 / 228 mph (Lubbock 1970's Great Plains Life Building: 12 inches
   off plumb, still standing; Joplin's St John's: frame intact, envelope
   gone, local wind assessed at <= EF3). So the ladder stops at `T4` and
   there is deliberately no `T5`. Caveat carried honestly: untested at a
   true direct EF4/5 hit on a dense high-rise cluster — no such event
   exists in the record.
2. **The urban-fire blacklist, unchanged.** `gac_fire.PACKS["dtc"]
   ["blacklist"]` (`Carved_`, `Building_11`, `Building_12`), `kit_substitute.
   unburnable()` (Muyang DownTown), and the fire's height cap carried as
   `tornado_city.TORNADO_MAX_H_M` (232 m, env-overridable). See the height
   cap entry in the bug catalogue for what the cap costs a tornado scene.
3. **Offline only.** Everything in this round runs on the host (pytest,
   bare `usd-core`) or in the container as bare python (`usd_python.sh`:
   pxr + VTK + Nucleus, no `SimulationApp`). Nothing has been rendered.

## The pipeline

    city dump (fire's Kit build) -> tornado_city_dry_run -> manifest + PNG
                                                          |
    per building:  slice (region=None) -> quake_flow.describe -> tornado.wind_at
                -> tornado_urban.plan_damage (PURE) -> tornado_urban_usd.apply_plan
                -> [next round: settle-free static export, sidecar, assembly]

| file | role |
|---|---|
| `disaster/tornado.py` | the track (unchanged for the suburb) + NEW `curvature_deg_per_km`, `touchdown_m`/`liftoff_m`/`ramp_m`, and `wind_at(cfg, x, y)`. Byte-identical for every existing preset (`tests/test_tornado_path_model.py` pins a 41x41 snapshot of two suburb presets) |
| `disaster/tornado_urban.py` | the PURE planner: `level_for_intensity`, `height_class_for`, `side_weights`, `LADDER_T[btype][level]`, nine `t_*` recipes, `_guard` + post-hoc caps, the debris ledger, `plan_damage(...) -> plan` (`tornado_urban_plan.v1`) |
| `disaster/tornado_urban_usd.py` | the USD half: `apply_plan(stage, ctx, plan)` (deactivate, void glass, rigid-displace, sweep roof props, author debris), `build_debris`, `debris_material`, `wreck_urban(...)` with `quake_sliced.wreck_sliced`'s ctx shape |
| `disaster/tornado_city.py` | the city gates (`damageable`), `height_class_for`, `btype_for`, manifest `record` / `entry_string` / `parse_entry` (`tornado:<kind>:<name>:<level>:<bearing>:<seed>`) |
| `tools/tornado_city_dry_run.py` | host-side, reads a Kit dump, writes `_plans/tornado_city_<seed>.json`, `_report.md`, `.png`; seven checks; `--all-levels`, `--check-only` |
| `config/presets/downtown_tornado_1500{,_lvl2,_lvl3}.yaml` | the fire levels' LAYOUT (same seeds 4/2/3, same 1 km crop windows) with a tornado track: severity 0.55 / 0.75 / 0.92-class, `curvature_deg_per_km: 6.0` |
| `tools/tornado_urban_probe.py` | the `gac_burn_probe.py` of this pipeline — the whole chain on a REAL merged asset in the container, no Kit |
| `tests/test_tornado_path_model.py`, `test_tornado_urban.py`, `test_tornado_urban_usd.py`, `test_tornado_city_dry_run.py` | host tests, one per stream |

## The wind decides the faces — `tornado.wind_at`

A tornado's near-surface wind at a point is tangential (cyclonic, CCW) +
translational + a convergent inflow (Karstens et al. 2013 tree-fall; NIST
NCSTAR 3's Rankine fit for Joplin: `Gmax` 4.5-5.0, inflow angle 15-25 deg
below 20 m AGL). In the track frame with `c` metres LEFT of the centreline:

    vt     = the cross-track profile `intensity_field` uses (shared helper)
    t_dir  = -heading if c > 0 (left flank, blows BACKWARD)
             +heading if c < 0 (right flank, blows FORWARD — rotation and
                               translation add, which is why the right
                               flank is the strong one and debris curls left)
    V      = translation_frac (0.22 = 1/Gmax) * peak, along +heading
    inflow = inflow_frac (0.50) * vt, toward the centreline
    wind   = vt * t_dir + V * heading_dir + inflow * inward_dir

Returns `{bearing_deg (direction the wind blows TOWARD, math convention),
speed_frac, cross_frac, over}`; `over = |r| < over_frac` where `over_frac`
defaults to `core_frac` — the vortex passed OVER the building and every face
was loaded in turn. **`inflow_frac` 0.5 is a compromise**: the literature
is ~2:1 radial:tangential at the ground and ~0 by 30 m AGL, and this is one
2-D value per building. A z-aware `wind_at` is the follow-up; the anchor
numbers are in `DEFAULTS`' comments.

Measured on a real right-flank building (probe, heading 35): bearing 57.6
(forward + 22.6 deg inward tilt), `cross_frac` -0.33, `speed_frac` 1.0.

`tornado_urban.side_weights` turns the bearing into per-face weights from
`quake_flow._outward(m, side)`: windward `max(0, -n.d)`, leeward suction
`0.35 * max(0, n.d)`, edge suction `0.55 * (1 - |n.d|)`, corners = mean of
the two adjacent sides + 0.15 ("corners dominate", hurricane research
§2.1). Every recipe scales its fraction by the weight — one bearing gives a
windward stripe, lightly-touched flanks and a near-clean lee; an over-run
core building is hit all round (0.7-1.0 per side, drawn from the seeded
rng). Measured on the real SM_Building_02 slice at bearing 57.6: S 0.93,
W 0.79, E 0.44, N 0.38, SW corner 1.00 — and the removals landed S 10 / W 3
/ E 3 / N 0.

## The ladder — T0..T4, envelope only

| level | i (after a 0.06 per-building jitter) | what |
|---|---|---|
| T0 | < 0.10 | nothing; not a record |
| T1 glazing | 0.10-0.36 | windward glass 5-15 % (rc_glass 2-6 % — the first curtain-wall glass DOD is EXP 101 mph, the EF1/EF2 boundary, so glass on a tower is a late-T1 thing while rooftop props go early); roof props swept |
| T2 envelope | 0.36-0.56 | glass 20-45 % windward (rc_glass 35-60 %: Fort Worth's F1-F2 debris cascade), parapet/coping 30-60 % off windward + one flank, 5-10 % of the windward OPENING panels out in the top third (rc infill 20-35 %) |
| T3 stripped | 0.56-0.74 | ONE contiguous cladding band on the windward face, 2-4 storeys x 40-70 % of that side's bays, upper-half biased, TOOTHED via `quake_sliced._apply_region`; glass 50-80 % windward and weighted elsewhere; parapets off 3 sides; 1-3 boundary panels HANGING (rigid pitch outward 25-70 deg about the bottom outer edge — `quake_sliced._disp` spec) |
| T4 breached | >= 0.74 | T3 plus ONE chunk at the windward corner: 1-2 bays each side x `max_chunk_storeys` storeys, anchored anywhere in the upper 40 % of the height (drawn per building — the first version always notched the roof corner); urm lowrise/midrise also lose the top 1-2 storeys' windward wall out-of-plane (macroblocks in the street); lowrise urm at i >= 0.85 may lose its top storey's exterior walls and shed the roof piece. Core, floors and roof STAY otherwise |

Per construction type (`quake_sliced.CONSTRUCTION`, measured): **urm**
parapets/cornices first, windward top wall peel at T4; **rc** infill panels
from T2, piers survive (`keep_pier` 0.45-0.65), chunk at T4; **rc_glass**
GLASS at every level — the inversion of `quake_sliced.LADDER_S`, where a
curtain wall moves LEAST in a quake — skin-only chunk (corner pieces in the
top two storeys of the region), never a macroblock.

Height-class caps, enforced twice (`_guard` rewrites the recipe list and
notes it; `_finalise` trims the largest region farthest-from-anchor first):

| class | H | max removed façade AREA fraction | max chunk storeys | out-of-plane |
|---|---|---|---|---|
| lowrise | < 18 m | 0.35 | 2 | yes |
| midrise | 18-45 | 0.25 | 3 | yes (urm) |
| highrise | 45-100 | 0.15 | 3 | no |
| tower | >= 100 | 0.10 | 2 | no |

**By AREA, not by piece count** — see the bug catalogue. Plus: never a
`core`/`roof` piece (except the lowrise top-storey rule), never an emptied
storey, never a ground-storey pier on a tower.

## Debris — the removed pieces become street debris

Every removed piece is ledgered (`plan["debris"]`), PURE: fragments by
class (`panel`/`block`/`coping`/`deck`, plus `glass` shards from every voided
pane at 1 per 0.35 m2, capped 60 per pane and `GLASS_SHARDS_MAX_PER_
BUILDING` 400 per building, thinned by stride so every broken pane still
carpets the street below it). Release at the piece centroid, travel along
`bearing_deg` +- N(0, 18 deg), reach `C_kind * speed * sqrt(2 z / g) *
lognormal(0.35)` with `speed = 25 + 70 i` m/s, capped at 1.5 H (glass
0.8 H); `C_kind` block/coping 0.08, panel 0.22, glass 0.35, deck 0.45 — a
dense block hits terminal velocity in a metre and drifts a small fraction of
wind x fall-time; only sheet goods fly. A fragment's THICKNESS is the source
piece's own (clamped 0.05-0.40 m), never derived from volume; the COUNT
absorbs the volume (`N_MAX_PER_PIECE` 40, `DEBRIS_MAX_PER_BUILDING` 600).
Landing points inside the source footprint are pushed out to the façade
line + 1.5 m along the bearing. The planner does not know about
neighbouring buildings; relocating fragments that land inside another
footprint is the assembly's job (the fire's `fire_debris_apron` shape).

`tornado_urban_usd.build_debris`: ONE merged mesh per (kind, material),
boxes with 24 `faceVarying` normals each (a shared-vertex box renders as a
pillow), SEATED ON THE FACE with `planks._lay`'s 2 cm bedding (flat piece
at `t/2 - bed`; a tilted piece's lowest corner at `-bed`) — floating-debris
cause 1 of the tornado skill, guarded by a test that every fragment's lowest
vertex is within [-0.03, +0.01] of grade. Materials are world-projected
(`damage._pbr` triplanar, `airstack://` texture paths resolved through
`scene_generator._join_asset_root` AT AUTHORING TIME — a filesystem path is
the freeze skill's "build-machine asset path" trap), cached per class in
`ctx["mats"]`. Glass shards and voided panes share ONE dark glossy
near-black material (`TornadoDebrisLooks/void`), never transparent glass —
a see-through shard over asphalt at 60 m is invisible.

## The city dry run

    python3 scene_gen/tools/tornado_city_dry_run.py --all-levels

reads the fire levels' Kit dumps (`_plans/city_placements_downtown_fire_
1500{,_lvl2,_lvl3}.json`, 1036-1090 buildings, the only ground truth for
what a scene contains — never a host-rebuilt layout), compiles the
`downtown_tornado_1500*` preset, samples `intensity_field` and `wind_at` at
each footprint centre, gates (`tornado_city.damageable`: house category,
`kit_substitute.route` in kit/slice, the pack blacklist via the LIVE
`gac_fire.PACKS`, `TORNADO_MAX_H_M`), draws the level with a per-building
`random.Random(seed * 1000003 + placement i)`, and writes the manifest,
report and plan PNG. Seven checks, each with its number in the report:
corridor coverage 8-30 % of the crop window's buildings and >= 60 % of the
track length inside it; gradient (no level > 55 %); the core reaches the
fabric — LEVEL-AWARE (peak < 0.80: >= 8 T3; 0.80-0.95: >= 6 T4 incl. a
highrise/tower; >= 0.95: >= 15 T4 incl. 3); nothing blacklisted or over
the cap; wind sides (left/right flank bearings ~160 deg apart — the inflow
tilt is why it is not 180); determinism; refused list complete.

Measured, 2026-09-01 (final cut table, level-aware check 3; all seven
checks PASS on all three):

| level | seed | track | records T1/T2/T3/T4 | refused | cap-refused towers in corridor (T4 among them) | window coverage / track in window |
|---|---|---|---|---|---|---|
| 1 (EF2) | 4 | [-160,160] hdg 150, w 212 m, peak 0.75 | 87: 25/46/16/0 | 16 | 2 (0) | 12.6 % / 64 % |
| 2 (EF3) | 2 | [100,150] hdg 60, w 277 m, peak 0.90 | 144: 26/29/74/15 | 24 | 4 (2) | 20.6 % / 65 % |
| 3 (EF4) | 3 | [20,230] hdg 110, w 308 m, peak 0.97 | 108: 23/11/24/50 | 20 | 5 (2) | 20.4 % / 68 % |

Level 1 has NO T4 by design — an EF2 track does not breach a building; its
first version was routed lengthwise along the tower column to satisfy a
"≥ 6 T4 at every level" check that was wrong (bug 6).

**The T3 share is a property of the FIELD, not the cuts.** Re-binning the
same intensities under four cut tables moved level 2's T3 from 82 to 69 at
most: a ~280 m corridor over 30-90 m footprints puts most in-corridor
buildings at i 0.55-0.75. The suburb's "light classes widest" lesson was
about 15 m houses in a 150 m corridor. The cuts were widened modestly
(0.32/0.52/0.72 -> 0.36/0.56/0.74) and left there.

## The container probe — run it before believing anything

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \
      bash scene_gen/tools/usd_python.sh scene_gen/tools/tornado_urban_probe.py SM_Building_02 T3 7 35"
    # TP_REGION=1 slices with bays only on the windward + adjacent sides

Places the real asset (`gac_fire.place_source`), slices it
(`slice_to_kit(region=None)`, 6-34 s), `quake_flow.describe`, `wind_at` on a
synthetic right-flank track, plans, applies, exports the root layer to
`/isaac-sim/.cache/tornado_probe/<name>_<level>_s<seed>.usd` (+ `.plan.json`)
and prints the census, stats, notes, debris bbox. It is what found every
entry in the bug catalogue below; the synthetic fixture found none of them.
The export keeps `<cell>/src`, so a cold open renders the pieces white —
this is a geometry probe, not a bake (`fire_bake.rehome_for_export` is the
bake launcher's job, next round).

Final round-1 probe results (real slices, bearing 35 -> `wind_at` 57.6, seed 7):

| building | class / btype | pieces (sides) | level | removed (by side) | area frac | glass voided / candidates | fragments | debris bbox | slice |
|---|---|---|---|---|---|---|---|---|---|
| SM_Building_02 | midrise urm, 42.7 m | 239 (S 72 N 72 W 36 E 13) | T3 | 16 (S 10 W 3 E 3) | 0.074 | 2 / 18 | 279 | 80 x 63 m, z <= 0.59 | 6 s |
| SM_Building_13 | tower rc, 139.9 m | 324 (36 per side, 1 bay) | T4 | 16 (S 10 W 3 E 1) | — | 3 / 94 (`fake_interior` windows) | 595 (440 panel) | 178 x 185 m, z <= 0.72 | 33 s |
| dtc:Amar_Tower | tower rc_glass, 231.4 m | 94 (W 30 E 30 **N 2 S 1**) | T3 | **0** | 0.0 | 4 / 54 | 233 glass | 229 x 186 m, z <= 0.27 | 29 s |

Amar is the limit case: the slicer collapses its short sides to one or two
pieces, so the windward S face cannot lose a band at all and the curtain
wall loses glass in four whole-storey pieces — see bug 4 and next-round
item 4.

Real-slice facts the planner is built against: SM_Building_02 -> 239 pieces
(corner 30 / pier 116 / wall 59 / core 11 / parapet 18 / parapet_corner 4 /
roof 1), `_side` is `S/N/W/E` for runs, `SW/SE/NW/NE` for corner AND
parapet_corner pieces, `-` for core/roof; sides are ASYMMETRIC in bay count
(W 36 pieces, E 13); the parapet band's `_storey` (11) is one above
`describe`'s own `storey` (10) — `_Grid` prefers `_storey`.

## Bug catalogue — every one found by the probe on a real slice

1. **Glass never voided (0 of 13 / 0 of 40 panes).** Every GAC material
   prim is named `UnrealMaterial` (`<cell>/src/asset/LOD0/Section6/
   UnrealMaterial`) and `quake_sliced._tex_of` reads `inp.Get()` on the
   surface shader's `diffuse*` input — but on these materials
   `UsdPreviewSurface.diffuseColor` is CONNECTED to a `UsdUVTexture` whose
   `inputs:file` carries the name, so `Get()` is None and neither the name
   test nor the texture test can ever match. `gac_slice.window_centres._tex`
   (which found 7,698 windows on this stock) and `gac_fire._diffuse_of`
   follow the connection. `tornado_urban_usd._void_glass` reproduces
   `window_centres._tex` and matches with `gac_slice.is_glazing(tex,
   mat_name=...)`, the one matcher every consumer uses. **`quake_sliced.
   _void_glass` has the same blind spot and its GAC earthquake bakes void 0
   panes** — not fixed here (earthquake stream's file), recorded so it is.
   **And then still 0 — because the PANES ARE NOT IN THE `wall` PIECES.**
   Measured on the live slice (`tools/_tp_glass_probe2.py`): SM_Building_02's
   glazing (Section7, `M_Building_01_Windows`, 10,860 faces) sits in 18
   PIER pieces and 9 core pieces; every `wall` (opening) piece carries one
   subset, the blind `M_Building_01_WallBack`. The slicer's `BAY_SPLITS`
   phase puts the opening in the NARROW sub-panels on this asset, so a
   role-based "the wide middle panel is the pane" pick can never find
   glass. Fix: `tornado_urban_usd.annotate_glazing(stage, placements)`
   stamps every piece with its measured `_glass_faces` / `_glass_frac`
   BEFORE `quake_flow.describe`, and `t_glass_loss` picks by that evidence
   (falling back to roles only when no piece carries the field — the
   synthetic fixtures). Never assume a role means a material.
   **A "pane" in this ladder is a PIECE's glazing subset, and on a real
   slice glazing CONCENTRATES**: SM_Building_02's 13,800 glazing faces sit
   in 27 of 239 pieces (18 piers, 9 core), so a T3 windward draw at frac
   0.5 voids 1-2 pieces per side — `n_glass 2` in the stats while each of
   those pieces holds dozens of windows. Read `n_glass_candidates` and the
   `glass_bands` record, not `n_glass` alone, before calling a draw starved;
   shard counts scale with the piece's measured `_glass_frac`, so they track
   AREA, not piece count.
2. **Fragments 600 m thick.** `_dims_for` derived a fragment's third
   dimension from its volume share; a whole 40 x 4 x 0.5 m tower wall run
   (80 m3) as ONE 0.8 x 0.15 m "panel" is 600 m thick — the exported
   debris bbox reached z = 308 m. Thickness is now the piece's own,
   clamped; the count absorbs the volume.
3. **Coping blocks 745 m away.** Massless-ballistic reach with C 0.35-0.80
   threw dense blocks off the plate. Only sheet goods fly; C lowered
   (0.08 for block/coping) and capped at 1.5 H.
4. **Piece-count caps are meaningless on a coarse grid.** SM_Building_13
   (140 m, painted windows -> `grid measured=False`, regular-grid fallback,
   ONE bay per side) lost three whole 40 m wall runs — a 40 x 12 m hole —
   at `removed_frac` 0.04 by count. Caps are by façade AREA now, and a
   1-bay side on highrise/tower limits a band/chunk to 2 storeys. The
   `TP_REGION=1` windward-hot-sides region slice did NOT recover bays
   (324 pieces either way): the regular-grid bay pitch lives in
   `gac_storey_slice.plan_slice_budget`, a slicer knob for the next round.
5. **The height cap leaves supertalls pristine in the core.** The fire's
   232 m cap (`SM_Building_16` 312 m, `_31` 302 m — 19 copies in a 1090-
   building city) refuses 3-5 towers inside each level's corridor, 0/3/2 of
   them in the T4 core: a 312 m tower untouched on the centreline of an
   EF4 track. Kept at the fire's value because the user said to stick to
   the fire's list; `TORNADO_MAX_H_M=1e9` lifts it and the dry run's report
   lists exactly which towers it would affect. THIS IS THE USER'S CALL.
6. **Check 3 forced a fake track.** "≥ 6 T4 at every level" made level 1
   (an EF2 track, peak 0.75) route its core lengthwise along the tower
   column to pass. Level-aware now.
7. **Brief errors caught by the research**: the Joplin report is NIST
   NCSTAR **3**, not 2; the EF damage indicators are DI **18** mid-rise,
   **19** high-rise, **20** institutional (not 20/21/22); `wind_at`'s
   priors 0.25/0.30/0.18 became 0.22/0.50/`core_frac`.

## Knobs

| knob | where | value | why |
|---|---|---|---|
| `curvature_deg_per_km` | urban presets | 6.0 (+ = left) | discrete left-biased heading changes in the record (Joplin, El Reno; Nixon & Allen 2021); 3-15 range |
| `touchdown_m`/`liftoff_m`/`ramp_m` | `tornado.DEFAULTS` | None / None / 120 | a real spin-up is ~3 km, longer than the plate; opt-in per preset |
| `translation_frac`, `inflow_frac`, `over_frac` | `tornado.DEFAULTS` | 0.22 / 0.50 / None (= core_frac) | see model-tornado-paths |
| `_URBAN_CUTS` | `tornado_urban` | 0.10 / 0.36 / 0.56 / 0.74 | above |
| `HEIGHT_CAPS` | `tornado_urban` | table above | R2 §8.6: strongly supported, do not loosen |
| `GLASS_SHARDS_MAX_PER_BUILDING` (`TU_GLASS_SHARDS_MAX`) | `tornado_urban` | 400 | a 100-building corridor at 60/pane is 150k boxes for a glass carpet |
| `DEBRIS_MAX_PER_BUILDING` (`TU_DEBRIS_MAX`) | `tornado_urban` | 600 | same argument for panels |
| `TORNADO_MAX_H_M` | `tornado_city` (env) | 232.0 | the fire's cap; see bug 5 |
| `_C_KIND`, reach caps | `tornado_urban` | block/coping 0.08, panel 0.22, glass 0.35, deck 0.45; 1.5 H / 0.8 H | bug 3; R2 §8.5's "median in the tens of metres" |

## ROUND 2 (2026-09-01, same day) — the user's review of the first scene, and the redo

The first 500 m GUI scene was rejected on sight: *"the tornado can't really
be hitting skyscrapers ... I don't see any building damages, partial
collapses ... Generate brownstones, low rises, mid rises. Skyscrapers can't
be in the middle of the tornado path ... I also don't see the ground
evidence of the route ... There shouldn't be all this wood debris
everywhere ... you can't just copy it directly [from suburb]."* Encoded as
R1-R6 in `_plans/urban_tornado_plan.md` §7. What landed (160 tests across
six files):

* **The bench** — `config/presets/downtown_tornado_bench_500.yaml`: 93 %
  rowhouse/lowrise/midrise fabric, sky districts small and off the track;
  pools are damage-capable only (kit archetypes restored, AEC
  `Reference_Brownstone*Row` whole assets OUT — the kit brownstone styles
  replace them; Muyang out). `districts` zoning is RADIAL, so a literal
  corner tower pocket is impossible from config — measured, documented,
  the nearest achievable (a small non-central disc) shipped.
* **Kit buildings damage for real** — `disaster/tornado_kit.py`: an
  ADAPTER that stamps `quake_flow.describe`'s kit elements with the
  slicer-shaped fields (`_side`/`_role`/`_storey`/`_bay`/`_size`,
  window/door -> the opening sub-panel so `is_opening` is true on exactly
  them), then the UNCHANGED planner runs; `wreck_kit` = `kit_substitute.
  build_kit` into `<cell>/parts` -> annotate -> plan -> apply. Probed on
  real brownstone_row/walkup/office/dw_terrace builds. The T4 stack
  (cladding band + corner chunk + windward top-wall peel + top-storey
  loss) is the visible partial collapse the user asked for.
* **Ground evidence** — `disaster/tornado_urban_ground.py`: a corridor
  debris field (brick bits, concrete chunks, gravel drifts, membrane
  sheets, glass glitter, sparse paper — classes cited from
  `_plans/urban_tornado_research.md` §5; merged mesh per class; OBB
  rejection against every building footprint; density x intensity^1.4;
  downwind elongation) + a low-opacity `Damaged_Asphalt_02` stain overlay
  through `ground.build_overlay` + `tornado.scour_coverage` at z = 0.025 m
  (above the road z-ladder). NO timber anywhere in the urban path —
  `deck` fragments became `membrane`/`metal`, `planks.wood_material` is
  unreachable (grep-proven + a test).
* **The track is SEARCHED, not tuned by hand** — `--tune-track` grids
  epicenter x heading (3,042 candidates; 14 admissible) under HARD
  constraints; winner `[-75, -125]` hdg 150: a southern low/mid crossing
  that clears the tower district entirely. Bench result: 29 records
  (T1 6 / T2 5 / T3 7 / T4 11), 5 T4 partial collapses on urm low/mid,
  100 % damage-capable coverage, protected-sky max exposure 0.297.
* **The gate** (R6): offline tests + the 2D figure (blocks by typology,
  buildings by level, PROTECTED tall stock hatched, corridor + core +
  ground wash) go to the user BEFORE Isaac.

### Round-2 bug catalogue — each found by review or a probe, not by tests

8. **The exposure check scanned only damage RECORDS.** The manifest held
   `SM_Building_16` (312 m, cap-refused) at raw i = 0.787, 14 m off the
   centreline — a pristine supertall in the core, invisible to a check
   that never looked at `refused`/`t0_footprints`. Exposure checks scan
   EVERY protected placement, and sample the OBB corner NEAREST the
   centreline (a 60 x 142 m footprint's centre can read safe while its
   corner is in the core).
9. **"Skyscraper" needed a definition the eye and the check share.**
   Height-class `highrise` starts at 45 m, which hatched half the midrise
   stock and would have protected it from damage. The protected set is
   class `tower` OR H >= 75 m (DI 18 mid-rise runs to ~20 storeys, DI 19
   above — R2 research §1); 45-75 m buildings are damageable mid-rises.
   The figure hatches EXACTLY the protected set.
10. **`t_chunk` anchored by storey INDEX.** `dw_terrace`'s 1.2 m trim band
    owns a full storey index between two 6 m bands, so "index >= 0.6 x
    top" landed a chunk at 31 % of H (13/19 draws). The eligible set is
    now built from each storey's own base elevation against 0.6 x H.
11. **Kit meshes carry NO GeomSubsets**, and none of their materials pass
    `is_glazing` (Downtown_West window modules: untextured shared-atlas
    subsets; MCE families: windows painted into ONE façade atlas). Kit
    "pane out" is therefore the window MODULE knocked out (name-matched
    `window`/`wnd`/`_door`, deactivated to a dark opening — the fire's own
    "windows out to a black void" read), via `tornado_kit`'s window-name
    prior + `apply_plan`'s knock-out branch. MCE styles honestly have no
    glass vocabulary at all (removal/chunks carry their damage).
12. **A banner "preserved byte-for-byte" printed failures under an
    "intact" label** after the accounting changed underneath it. Labels
    follow the data, never the diff-minimisation.
13. **The launcher compiled `disaster-type: tornado` straight into
    `build_city`**, whose generic damage knobs would have tilt/sunk random
    intact buildings as "damage stand-ins" (no `damaged` pool variants in
    `urban_gac`). `_neutralise_layout_disaster()` zeroes the
    building/human fractions unconditionally and the street/path-debris
    knobs by default (`UT_LAYOUT_STREET=1` opts back in).
14. **`ssf` composes twice on a kit build under a holder** — the holder
    already carries `scale = ssf`, so `wreck_kit(..., ssf=1.0)` inside it;
    passing `ssf` again scales every piece by `ssf**2`.
15. **A NEW launcher shipped UNLIT — again.** The bench launcher (round 3)
    authored no light at all; Pegasus's "Default Environment" carries
    nothing the snapshot renderer sees, the GUI viewport's default camera
    light hides it from a person at the desk, and every capture came back
    black — the same defect the frozen-dataset cells once shipped with.
    Every launcher that composes its own stage MUST author light
    explicitly (`sky_presets.apply_sky_preset(stage, "mid_day", ...)` is
    the one-line answer; an unrecognised name falls back to mid_day,
    never to darkness). Check the FIRST capture of any new launcher for
    brightness before reading anything else from it.

## ROUND 3 / 3b (2026-09-01, same day) — the aerial-first redesign and the
## structural-coherence fixes

The lit bench drew two more user verdicts and both rebuilt the pipeline's
priorities. First: *"it looks like wooden planks scattered everywhere
rather than actual street props"* + no roof damage -> §8b's AERIAL SHOT
LIST became the governing spec (roofs, berms, flipped/thrown cars, felled
pole runs, the gradient, blocked streets, people — nothing gets built a
60-90 m drone cannot see or the benchmark score). Then, on the first lit
bench: *"floating roofs, missing chunks from buildings ... The debris
doesn't seem to match the building next to it. Look at the various skills"*
-> §8e's three fixes, each an under-applied lesson from this repo's own
skills. What landed (all measured on real geometry; 313/314 host tests, the
one failure being the documented needs-vtk environment gap — run the kit
stub-stage test via `uv run --with vtk --with trimesh`):

* **The roof system** (`disaster/tornado_roof.py`): windward-snapped peel
  patches (substrate quad + curled lip + torn sheets), gravel-scour band,
  coping strip, prop sweep; coverage monotone T1->T4 (0.07 -> 0.77); its
  rng is `crc32(tag)`-derived so façade plans stay byte-identical.
* **The roof SUPPORT rule** (`_shed_unsupported_roof` in `_finalise`):
  parapets shed when their wall band empties (`_wall_band_storey` — GAC's
  whole-band parapet storey holds NO wall pieces, so the naive same-storey
  test never fires); roof pieces split by area at `_ROOF_TILE_FRAC` 0.25 —
  a KIT's per-bay tiles shed by local support, a slice's single slab sheds
  at `ROOF_SHED_FRAC` 0.35 of the top storey gone. No more floating roofs.
* **Ragged tears + interiors**: `quake_sliced._plan_tears` wired for T3+
  (a thin LOCAL `_author_tears` — quake's own would call `_a_roofify` and
  overwrite per-tile roof survival); `quake_flow.fit_interior` + a
  0.08-linear-albedo TEXTURED backing box behind every opened (storey,
  side) — a hole now reads as a gutted floor, not a black slab. Real T4
  probe: 36 tears, 40 fit prims, 8 backing quads.
* **Debris provenance** (`annotate_surface` -> fragment `source_tex` ->
  one triplanar `_pbr` per distinct texture): the brownstone berm binds
  `M_MBuilding03_Facades`, coping binds `RoofLedge`, membrane stays
  roofing-grey — the suburb's "bind the texture, not the material" fix.
* **Berms that stack** (`z_lift` honoured by `build_debris` — DB measured
  the flattening gap honestly instead of faking heights), drifts biased to
  building feet/corners, the plate-region clamp (`TU_PLATE_REGION` env),
  the darker `Wet_Destroyed_Asphalt` stain.
* **The street pass** (`tornado_street.plan_street` + `apply_street`, the
  apply half EXTRACTED into the module so the bench and the city launcher
  share one contract): felled pole RUNS (+0.20 within 40 m of a felled
  neighbour), carried furniture, snapped/tipped street trees, Paulikas
  cars with core flips and a 20-60 m thrown share, `UT_MIN_TIPPED` floor.
* **The two real collapse classes** (user: "industrial buildings and
  brownstones ... could collapse ... show those"): `t_facade_collapse`
  (urm lowrise <= 5 storeys — the cap moved 4 -> 5 because Waco's own
  Dennis Building was five storeys and `brownstone_row` measures 5; fires
  incl. the ground storey, 2 leaning macroblocks, roof shed, CAP-EXEMPT by
  §8c's named carve-out) and `disaster/tornado_collapse.py` (tilt-up:
  perimeter panels OUTWARD 0.78 at total, roof down inside — Joplin's
  Home Depot mechanics; partial at i 0.5-0.7, this class fails EARLY).
* **The bench** (`urban_tornado_bench_launch_script.py`, §8d): 11 labelled
  exemplar cells + the 12-figure D-row placed by
  `disaster/urban_tornado_people.py`, which encodes the sourced urban
  victim rules (`_plans/urban_tornado_people_research.md` — NIST NCSTAR 3
  Table 4-3; berm-base casualties on the FRONT retail side, the
  refuge-DOORWAY figure, the outdoor wall-crush class, ~20 % vehicles,
  <= 1 plaza pedestrian). Plan-only mode (`UTB_PLAN_ONLY=1`, bare
  usd_python.sh, ~90 s) is the pre-launch gate.

### Bug catalogue additions (16-20)

16. **A new launcher shipped UNLIT (again) and with Pegasus's Default
    Environment furniture in-frame** — every capture black, then a ~250 m
    checkered GroundPlane mid-plate. Author light explicitly
    (`sky_presets.apply_sky_preset`) and strip `{"GroundPlane",
    "Environment"}` in every launcher that composes its own stage.
17. **`quake_sliced._author_tears` would overwrite F1's roof survival** —
    its floor-edge half calls `quake_flow._a_roofify`. Reuse the PLANNER
    (`_plan_tears`), write your own thin apply.
18. **`tornado_roof._roof_already_shed` greps region TAGS, not
    `plan["roof_shed"]`** — `t_facade_collapse` set the flag but tagged
    `facade_collapse`, so peel art was drawn on missing roofs. Normalise
    the region tag; a boolean two modules disagree on is not a contract.
19. **C3's launcher-hook snippet passed WORLD-frame wind into a
    cell-frame pass** — every pass under a yawed holder rotates
    `bearing_deg` by -yaw; the lead's applied hook corrects it.
20. **A blanket cap test re-imposed a documented exemption** — when the
    5-storey cap made `facade_collapse` reachable on `brownstone_row`,
    `test_caps_hold_by_height_class` failed at 0.3508 vs 0.35; the test
    now honours §8c's carve-out instead of silently re-capping it.
21. **Do not send a sliced `core_x/core_y` shell through the normal Voronoi
    tear pass.** These are not kit ornament: an exposed square end can be
    visible at a tornado opening, but the prim may contain a large merged
    structural mesh. On A3 (`SM_Building_02`), adding two such pieces to
    `_tear_perimeter` changed an approximately 31 s focused build into a
    >4 min CPU pass. Cut only a shallow edge band (or re-slice that band),
    and borrow the nearest exterior facade texture; the core's own
    WallBack/concrete binding produces a conspicuous grey patch.
22. **A matching texture URL is not a matching fragment material.** A3's
    intact facade uses authored material graphs plus face-varying atlas UVs;
    rebinding a fracture to a new triplanar shader using that same PNG changed
    both colour and scale. `fire_collapse.skin_fragment` is the exact path:
    preserve the source material and reconstruct its UVs on both standing and
    tornado-loose tear cells; bind `TornadoTearLooks` only to the invented
    `core` subset. The tornado tear planner must also temporarily align raw
    `element["storey"]` with sliced `p["_storey"]`: otherwise genuine hole
    borders receive no tear job and expose the rectangular slice grid. The
    focused A3 audit after both fixes reported 715 exact-source skins, 711 cut
    subsets, zero surface overrides, and zero uncovered wall-hole borders.
23. **Facade cells localise damage; they must not become its visible
    boundary.** Deactivating a window module, dropping a precast panel intact,
    or tilting one rectangular slice cell exposes the authoring grid. Reuse
    `quake_flow.r_curtain_wall` / `r_window_glass` for retained cracked and
    crazed panes, irregular remnants, mullions and sill/ground shards. Use the
    `fire_collapse` perimeter tear for masonry/concrete, with zero untorn
    geometric hole-border pieces as a hard gate. High-rises/towers retain the
    structural shell: no stacked cladding band, whole-panel loss or rectangular
    hanger; at T4 permit at most one <=2-storey, one-edge-bay ragged bite.

## Known gaps — the next round, in order

1. **Bake launcher**: copy `quake_gac_bake_launch_script.py` /
   `tools/quake_gac_bake.sh`, swap `wreck_sliced` for `tornado_urban_usd.
   wreck_urban`, no settle (debris is authored; hanging panels and
   macroblocks are rigid transforms), `fire_bake.rehome_for_export`, root-
   layer export, a `tornado` sidecar (plan + wind + bbox/top_z).
2. **Kit-style buildings** (`bld_*_DG0`, MCE `same_art`): ~half the
   corridor's records route `kit`; the planner runs on `quake_flow.
   describe`'s table unchanged but no kit bake path exists — per (style,
   level, wind-side class) archetypes, `bake_quake_archetypes`-shaped.
3. **City assembly**: hide intact, reference bakes (the fire launcher's
   transform trap), the suburb launcher's corridor street pass verbatim
   (trees, felled streetlights, carried bins, `tornado.car_pose`), debris
   relocation out of neighbouring footprints, glass-carpet decals, people.
4. **Glass at WINDOW-ISLAND granularity, and slicer bay pitch on hot
   sides** (bug 4). On Amar Tower and every regular-grid tower a "pane" is a
   whole storey's glazing subset, so a curtain wall cannot show the Fort
   Worth stripe (SW face ~90 %, NW ~60 %, SE ~12 %, corner-concentrated) —
   it loses glass in storey-wide slabs on the faces that happen to have
   pieces. `gac_fire.window_rects` / `_islands` already group the glazing
   faces into per-window islands by grid-hashed union-find; `t_glass_loss`
   on `rc_glass` should void ISLANDS (author one GeomSubset per lost window)
   with a per-face fraction, independent of the piece grid. Separately,
   `gac_storey_slice.plan_slice_budget` needs a minimum bay count on the
   hot sides for the cladding band and chunk to have anything to remove
   on a 1-bay face.
5. **Roof furniture**: `wreck_urban` does not call `quake_flow.dress_roof`
   (earthquake-only material keys), so `roof_props: sweep` is a documented
   no-op until roof plant is placed for this ladder.
6. **z-aware `wind_at`** (inflow ~2:1 at grade, ~0 by 30 m); ground-floor
   debris-driven glazing as its own term (R2 §8.3); a tilt-up / CMU-light-
   roof low-rise class that fails by connections at F1-F2 (R2 §8.2);
   `quake_sliced._void_glass`'s blind spot (bug 1).
7. **Nothing has been rendered.** Every number above is measured geometry;
   the look is unverified.
