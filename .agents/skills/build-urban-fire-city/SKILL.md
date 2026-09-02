---
name: build-urban-fire-city
description: 'THE end-to-end procedure for a 1 km urban-fire cell, and the shared urban layout every other disaster builds on. Two commands: `make_cell_plan.sh` on the host (CPU only — layout, scored seed choice, wind-driven corridor fire, lazy-bake worklist, four review PNGs, gates) then `urban_fire_cell.sh` on the pod (clear, bake, assemble, people, freeze, cold-verify). Rewritten 2026-09-02: the 1.5 km-plate-plus-crop is GONE (1 km generated directly — the crop had taken highrise+tower from 27-31% of the plate to 46-58% of the delivered cell), the burnable-only pool cut is GONE (48-64 -> 75-88 distinct models; undamageable buildings are left undamaged instead of excluded from the city), the multi-ignition selection is GONE (one contiguous wind-driven corridor, because half the old checkerboard was a BAKE GAP rather than fire behaviour), and the level ladder is a real fire ladder rather than three copies of the same 5.6 h epoch. Carries the per-stage bake/assembly/freeze reference and bug catalogue, the four bake kinds, the determinism rule, why seed is first-class at 1 km, the block-strip monotony fix and its measured trade-off, per-unit brownstone variance, the overview camera that framed the middle 60% of every cell, and a gate list where every entry exists because a launcher printed correct counts, exited 0 and shipped an empty cell.'
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Urban fire city, end to end

Two commands (below). This skill was rewritten on 2026-09-02: the layout and
fire-selection stages were replaced outright, and the bake/assembly/freeze
stages were re-gated. The per-stage reference for those later stages — still
accurate, and still the bug catalogue that matters — is retained verbatim at
the end under REFERENCE.

`build-urban-fire-scenes` remains the physics, the F0-F5 ladder and the
per-construction-type recipes; this is the procedure over it.

Four things changed, each because something measurable was wrong.

## Why this exists — the four findings

**1. The 1.5 km-plate-plus-1 km-crop over-weighted skyscrapers.** District
rings are fractions of the region half-diagonal (`districts.py:538-544`), and
`core` had been widened 0.276 -> 0.36 to fix a *500 m* problem. On a 1.5 km
plate that multiplies by 9, and all three crop windows sat within 250 m of the
plate centre, so the delivered cell kept the core and threw away the outer
rings:

| | full 1.5 km plate | delivered 1 km crop |
|---|---|---|
| highrise+tower, block area | 27-31 % | **46-58 %** |
| rowhouse+lowrise buildings | 36 % | **7 %** |

Generating 1 km DIRECTLY makes the fractions mean what they say and removes
the whole crop/frame bug class with it.

**2. The burnable-only pool cut caused the repetition.** The fire presets
replaced `lowrise` 26 -> 7 and `midrise_v2` 22 -> 14 so everything was fuel.
Of the 7 lowrise, exactly ONE sat inside the packer's area band for a large
gap, so every large lowrise gap drew the same model — six identical department
stores in a line, repeated verbatim on four blocks. Measured, cut vs full
pools at 1 km:

    distinct models      48-64  ->  75-88
    copies per model    7.2-9.6 -> 4.8-5.9
    in a <=60 m repeat  22.7-37.5 % -> 15.3-22.6 %

**3. The level ladder was not a fire ladder.** `downtown_fire_1500`, `_lvl2`
and `_lvl3` were byte-identical in the fire — all `duration_s: 28800`,
`start_offset_frac: 0.70`. Every level solved the same 5.6 h fire, by which
point everything burning is burnt out, and the levels were separated only by
`fire_city_union`'s post-hoc `rebalance_severity`. That is why "Level 1 —
early / contained" shipped holding nothing but F4 and F5.

**4. Multi-ignition selection produced a checkerboard.** Six ignitions each
reaching three or four buildings reads as six unrelated incidents. Worse, the
solver can only touch an asset that HAS a bake, so it routed around every
unbaked building and left it pristine mid-block. Measured on the old L3: of
the intact buildings within 60 m of a burning one, **27 were burnable and 27
had no bake at all** — half the checkerboard was a bake gap, not fire
behaviour. That is also, in hindsight, what finding 2's pool cut was really
solving.

## The model: one wind-driven corridor

`disaster/fire_corridor.py`. One origin, one front, running downwind.
EVERY building in the swathe burns — that is the contiguity guarantee, and it
makes the bake list a geometric consequence rather than a guess. Level comes
from the compartment-fire clock (`soot_plume.DURATION_S`): a building at
along-distance `s` ignites when the front reaches it, so the burnt-out tail
sits at the origin and the fresh head at the front.

Both axes ride the level, because both are consequences of the same thing — a
fire running longer has covered more ground AND burnt it worse. The heading
does NOT, so the three cells do not all burn along one diagonal.

| level | city seed | area | heading | epoch |
|---|---|---|---|---|
| L1 | 3 | 14 % | 70° ENE | 0.22 (1.8 h) |
| L2 | 18 | 24 % | 160° SSE | 0.45 (3.6 h) |
| L3 | 23 | 33 % | 285° WNW | 0.75 (6.0 h) |

A corridor prefers to grow ALONG the wind; when the plate runs out of run at
that heading (a diagonal crosses 1 km in 1414 m, an axis-aligned line in
1000 m) it fans out instead. Without that fallback any non-45° heading is
refused above ~25 % coverage.

## Seed is a first-class parameter

At 1 km there are ~33 district nuclei against ~75 at 1.5 km, so the mix is
high-variance: across 24 seeds, highrise+tower ranged **14.7 %-48.1 %** of
block area. Do not take the preset's seed on faith.

    python3 scene_gen/tools/pick_city_seed.py \
        --config downtown_urban_1000 --region 1000 --seeds 0-23 \
        --out scene_gen/_plans/seed_pick.json

Scores each seed against the same bands the review sheets and the pytest gates
use, hard-gates on empty blocks, ranks them. Re-run it after ANY change to the
subdivider or the districts config — those move every layout and invalidate
the committed seeds.

## The two commands

    # HOST — CPU only. Layout, corridor, bake worklist, review PNGs, gates.
    bash scene_gen/tools/make_cell_plan.sh                 # all three levels
    LEVELS=1 bash scene_gen/tools/make_cell_plan.sh        # one

    # POD — bake, assemble, people, freeze, verify.
    FC_ACK_LAYOUT_GAPS=1 bash scene_gen/tools/urban_fire_cell.sh 1
    BAKE_ARGS=--dry-run  bash scene_gen/tools/urban_fire_cell.sh 1   # preview

The split is the point: a 1 km city lays out in ~90 s on CPU and costs hours
on a pod. Every decision that can be made from arithmetic is made and reviewed
on the host, and the pod only ever sees a plan that already passed.

**Look at the four sheets before spending a pod.** `<REVIEW_DIR>/L<N>_*`:
`districts` (zoning, every block labelled), `diversity` (same-model pairs
within 60 m ringed in red), `blockshapes` (block-size monotony per district),
`damage` (where the fire runs, and a dashed ring for any manifest record that
matched no building — that is what a frame shift looks like).

## Gates, and why each one exists

The recorded failure mode here is NOT a crash. It is a run that prints every
count correctly, exits 0, and ships a cell with nothing in it. So the pod
script asserts on OUTPUT, never on exit codes:

| gate | what it catches |
|---|---|
| `FREEZE_EXPORT=1` | defaults to `"0"` (freeze script:259) — without it the freeze captures snaps, writes GT, and **never writes a USD** |
| `FC_BAKES="$FB_OUT/city_<seed>"` | `fire_city_bake.sh:258` appends `city_<seed>`; the assembly globs `FC_BAKES/*.usd` **non-recursively** (launcher:908). Point it at the parent and ZERO bakes compose — you get the intact city, silently |
| `freeze_report.json` -> `portable_ok`, `sky_lights>0`, bbox ~1 km | `PortabilityError` is raised, caught, printed, and the process still exits 0 (freeze script:884-903) |
| `grep 'Flow OOM check CLEAN'` | "does not crash and does not raise. It renders a city with NO smoke in it." `Kit log not found` is neither a pass nor a fail — treat as failure |
| `manifest/city match: N/N` | the offline dump is a CPU reimplementation of what Kit builds; nothing else proves they agree. A mismatch reads as `4/34` and composes bakes onto the wrong buildings |
| `FB_REST_STRICT=1` | NOT-AT-REST is data loss, not a warning — moving bodies are DELETED from the export and the cell ships missing debris, while the driver exits 0 |
| stale-bake clearing | the cache keys on `<stem>.usd`+`.json` EXISTING, so a pre-2026-09-02 bake with mirrored-atlas soot or roof-piercing columns classifies HAVE and is skipped forever. A settle does not reproduce — delete, never "re-run to get the same pile" |
| cold re-verify | the in-process gate verifies a file the same process just wrote. The `ComputeAllDependencies` `_UnpackValue` fallback here is normal, not a defect |
| `fire_people_rerun.sh` | `fire_people.py` has **no argparse and no `__main__`** — calling it as a module is a silent no-op |
| audit numbers, not exit code | `city_layout_audit.py` has no `sys.exit`; `\|\| die` on it is decorative |
| `/dev/shm`, `ISAAC_SIM_PYTHONPATH`, `PYTHONUNBUFFERED`, container auto-detect | bus errors on a 64 MB shm; vtk vanishing inside a live SimulationApp; a bake that wrote 72 archetypes and printed nothing; `isaac-sim` vs `isaac-sim-livestream` |

`FC_ACK_LAYOUT_GAPS=1` is required because `final_disaster_dataset/Fire/Urban`
is under a do-not-rebuild hold (see "Still open" above) and that is
exactly where `FREEZE_OUT` defaults.

## Layout variance knobs — all OFF by default

Every one perturbs the RNG stream, so `downtown_gac`, the 500 m scenes and
`downtown_earthquake` reproduce draw-for-draw only while they are off. The
shared base turns them on; a pytest gate asserts the defaults.

* `districts.pack_min_candidates` — widen the packer's area band until it
  offers N distinct assets. Removes the pathological one-candidate case where
  all four anti-repeat rules are inert by construction. **On its own this is
  roughly neutral** (rep-share moved both ways across seeds); the lever is
  pool size, not the band.
* `districts.repeat_hard_ladder` — retry the hard repeat radius at R/2 before
  failing open.
* `districts.per_block_rng` — a per-block packing RNG. **Measured: this does
  NOT fix same-district repetition** (0/40 comparable block pairs had
  identical layouts even with it off). Kept because it costs nothing, but the
  cause was elsewhere — see below.
* `layout.anisotropic.target_jitter` — **this is the one that mattered.** The
  first split cuts the plate into STRIPS and every block inherits its strip's
  long side, so long sides landed on ~4 values city-wide (eight consecutive
  245 m blocks, then six at 235 m). Jitter the per-cell target and that breaks
  up: **4 -> 25 distinct long sides**, tower CV 0.02 -> 0.30.

Two traps inside that last one, both found by measurement:
raising the target's LOWER bound makes a cell unsplittable (`_recurse` needs
`w >= 2*t_lo + road`) and emitted a 463 m block against a 340 m ceiling — the
low bound must move DOWN. And lowering it on EVERY cell shrank the whole city:
block variety improved but repeat share went 8.9-13.2 % -> 14.7-19.2 % and no
seed scored 0.00. Restricting the reduction to a quartile (`_LO_JITTER_SHARE`)
keeps the variety at 12.9-15.0 %. **That is a genuine trade, not a free win.**

## Brownstone variance

`Reference_Brownstone<N>Row` is not N houses — it is ONE 6.67 m facade merged
N times, in all seven row assets. Mixing row LENGTHS therefore gives zero
facade variance. `disaster/aec_variety.py` tints brick tone, trim and door per
unit over the MDL brick (a tint, not a replacement — replacing it discards the
mortar and courses), reusing `aec_burn`'s proven unit addressing and composing
with soot rather than fighting it. `AEC_VARIETY=0` disables.

Per-unit GEOMETRY variance, and per-unit collapse under fire, need the row
sliced — see `slice-buildings-into-kits`. Not done.

## Review imagery

Two bugs fixed in `disaster/baseline_captures.py`:

* the overview was hardcoded to `(0, 0)`, so any cell not centred on the stage
  origin photographed the wrong ground;
* its height came from the HORIZONTAL aperture alone, but captures are
  1280x720 and `place_camera` authors only `horizontalAperture` — vertical
  coverage was ~0.62*span, so a square 1 km plate was framed to roughly its
  **middle 60 %**. `overview_height_m` now solves against the smaller
  half-angle: **1619 m for a 1 km plate where the old rule gave 950 m.**

New `blocks/` family: one plumb top-down per city block. The `districts/`
family shoots BURNING CLUSTERS, which on a one-corridor fire is a single
component — three frames for the entire cell.

## Still open

* The `manifest/city match` gate is wired but has never been exercised against
  a real Kit build. Do the one-time `FC_INTACT_ONLY=1 FC_DUMP=` diff on the
  pod before trusting the offline dump.
* **The empty-block hold — believed resolved, not yet signed off.** A separate
  investigation recorded "~70 % of city blocks ship empty" and put
  `final_disaster_dataset/Fire/Urban` under a do-not-rebuild hold. That figure
  did NOT reproduce: an offline seed-4 `downtown_fire_1500` build fills **105
  of 106 blocks** (the 106th is the park), the shipped `level_1` USD has zero
  buildings off-block, and **24 of 24 swept seeds had zero empty blocks**. The
  number is what you get counting CROPPED content (320 houses) against
  UNCROPPED block rects (106) — the same frame confusion as the level_3 bake
  shift. Generating 1 km directly dissolves it either way. The hold is still
  enforced in code (`FC_ACK_LAYOUT_GAPS=1`) until whoever set it agrees.
* Only fire has a corridor. Tornado, quake and a future urban hurricane still
  need their own damage stage on top of this layout.


# REFERENCE — the later stages, unchanged

Stages 3-7 below are the ORIGINAL pipeline's and remain accurate: the
corridor work replaced how buildings are CHOSEN, not how they are baked,
assembled or frozen. The gate table above supersedes any gate wording
here that conflicts with it.

## Stage 3 — the bakes: one building, one Kit process

```bash
scene_gen/tools/fire_city_bake.sh scene_gen/_plans/fire_city_500m.json --dry-run
scene_gen/tools/fire_city_bake.sh scene_gen/_plans/fire_city_500m.json
scene_gen/tools/fire_city_bake.sh scene_gen/_plans/fire_city_500m.json --force
scene_gen/tools/fire_city_bake.sh scene_gen/_plans/fire_city_500m.json --verify-only
```

Sequential by design: one GPU, one machine-wide slicer lock. The driver never
parses the manifest — `fire_city_manifest.py` is the ONE place a record becomes
an entry string (`kind:name:level:origin:sides:seed`) and a cache stem
(`<kind>_<name>_<level>[_o<origin>][_<sides>]_s<seed>`), so a skipped bake and
a described bake always mean the same file. Cache key = both `<stem>.usd` and
`<stem>.json` existing (`HAVE` / `NEED` / `STALE` / `ERROR`).

**Where things land.** Bakes: `$FB_OUT/city_<seed>` in the container, host view
`$HOME/docker/isaac-sim/cache/main/...` (the compose bind mount) — the driver
keeps both because classification runs on the host and the bake in the
container. City sidecars:
`scene_gen/_plans/_fire_city_json/city_<seed>/<stem>.city.json`. Logs:
`$HOME/docker/isaac-sim/logs/city_<seed>_<stem>.log`.

| env | default | note |
|---|---|---|
| `FB_OUT` | `/isaac-sim/.cache/fire_bakes` | container; `city_<seed>` appended |
| `FB_SEED` | `7` | `BUILD_SEED = FB_SEED + 7*i`; the BURN seed is the record's own |
| `SETTLE_STEPS` / `SETTLE_QUIET` | `2400` / `400` | the F5/F5c/F6 tier; `_MID` `1600`/`300` is F4 and `_LOW` `600`/`150` is F0-F3 (`FB_LEVEL_SETTLE=0` flattens all three) |
| `SETTLE_DECOMP_M` | `0.8` | convex decomposition threshold |
| `SETTLE_REST_V2` | `1` | forced to `0` for `kit:` records — the MCE kit look is frozen byte-for-byte |
| `FB_REST_STRICT` | `0` | 1 makes a not-at-rest record a driver FAILURE |
| `FB_BAKED_KITS` / `TIMEOUT_S` / `SOOT_TEX_COMPRESS` | `1` / `5400` / unset | use a pre-baked GAC kit; per-bake timeout; container default is BC1 DDS |

The driver also pins `PYTHONHASHSEED=0`: without it `urban_fire.r_render_peel`
iterates a `set` of side letters in a per-process order and every subsequent
draw from the shared rng changes (two runs of identical code: 326 meshes moved,
prims 6998 -> 6977, top_z 21.69 -> 20.76 m).

**A good bake log line** reads `DONE in <N>s, <M> MB, verify=OK, rest-OK` —
behind it, `FIRE BAKE DONE`, `BAKE VERIFY OK` and a `REST_CHECK ... at_rest=1`.
The run ends `FIRE CITY BAKE seed <s>  <n> baked, <n> skipped, <n> failed, <n>
NOT AT REST`.

**Trap 1 — NOT AT REST is data loss, not a warning.** Bodies still moving at bake
time are DELETED from the export, so the cell ships missing debris. `grep
REST_CHECK $HOME/docker/isaac-sim/logs/city_<seed>_<stem>.log`.

**Trap 2 — the starved-events contradiction.** A bake can complete "successfully"
with a fire plan and no fire: no flames, no smoke, no soot (the skin binds per
event). The only tell is a line reporting zero over zero — grep the assembly for
`fire: 0 flame over 0 opening(s)` after every run, and probe offline with
`tools/_dtc_open_probe.py kind:Name@origin`.

**Manual single-building entries** go through the row driver
(`kind:name:level[:origin[:sides[:seed]]]`, empty fields absent), also the only
place `FB_UNITS=2,3` (an `aec:` row's burning units) is passed through:

```bash
scene_gen/tools/fire_bake.sh aec:Reference_Brownstone5Row:F3
scene_gen/tools/fire_bake.sh gac:SM_Building_02:F1 kit:commercial_mid:F5c::S,E
```

## Stage 4 — the assembly

```bash
docker exec isaac-sim tmux send-keys -t isaac 'clear; \
  ISAAC_SIM_HEADLESS=true SCENE_CONFIG=downtown_fire_500 \
  SG_INSTANCE_PLACEMENTS=1 PYTHONHASHSEED=0 \
  FC_MANIFEST=/isaac-sim/AirStack/scene_gen/_plans/fire_city_500m.json \
  FC_BAKES=/isaac-sim/.cache/fire_bakes/city_4417 \
  FC_PEOPLE_JSON=/isaac-sim/AirStack/scene_gen/_plans/fire_people_final.json \
  SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/fire_city \
  PYTHONUNBUFFERED=1 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh \
  /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window' ENTER
```

**The knob is `FC_BAKES`, not `FA_BAKES`** — `fire_city_bake.sh` prints its
comma list under the legacy heading `FA_BAKES for the city assembly`, but the
city launcher reads `FC_BAKES`. Paste the list, not the directory.

Three log lines gate this stage.

* **The frame guard**, run BEFORE anything is hidden; a pass is silent, and the
  fail refuses the record outright (nothing hidden, nothing placed): `[fc] ***
  record <i> (<stem>): bake holder at (x, y) would sit <d> m from the intact
  cell it replaces` — `CELL_MATCH_TOL_M = 0.5` m.
* **The block audit**, after compose, reported not refused (a collapsed shell
  may legitimately spill past its lot line): `[fc] block audit: <n> of <N>
  composed bake(s) have less than 90% of their footprint inside a block`, plus
  a `[fc] *** d<i> <stem>: only <p>% ... it is standing on a road` per
  offender.
* **The Flow OOM check.** `[fc] Flow OOM check CLEAN (<path>)` is the pass;
  `[fc] FLOW STARVED — "Out of GPU memory allocating resource 'flow'" xN` is
  the fail; `[fc] Kit log not found ... do not trust a capture that shows no
  smoke` is neither.

**That last one matters more than anything else here.** Flow's block pool is a
carb setting (`rtx/flow/maxBlocks`); it does not crash and does not raise, and
past it every further emitter gets no voxels — **the city renders with NO SMOKE
while every count in the banner is right.** The defaults are sized for that
(`FA_CELL_M=0.55`, `FA_MAX_BLOCKS=12288`, `FA_EMITTERS=30`,
`FA_EMITTER_BUDGET=800`, allocated flame > smoulder > wisp then by height); the
single-building showcase measured one OOM line at 0.12 m cells / 24576 blocks on
a 16 GB card with the GUI up. Never carry a bench's Flow settings into a city.
Captures land in `SNAP_DIR` (under `/isaac-sim/.nvidia-omniverse/logs/`) as
`city_top.png`, `wave_downwind.png` and per-building `d<i>_<name>_<level>`
pairs; the banner is `URBAN FIRE CITY DONE`.

## Stage 5 — people

```bash
scene_gen/tools/fire_people_rerun.sh \
    scene_gen/_plans/fire_city_500m.json \
    scene_gen/_plans/fc_dump_500.json \
    $HOME/docker/isaac-sim/cache/main/fire_bakes/city_4417
```

Re-solves and prints the manifest<->dump match (BY GEOMETRY, not by index — a
record naming a building that has since moved or resized is skipped and
counted), the sidecar-completeness table, the census and the rule checks.
Writes `scene_gen/_plans/fire_people_final.json` + `.png`. **Gate:**
`fire_people_rerun.sh: PASS — ... is ready for FC_PEOPLE_JSON.` and exit 0; do
not point `FC_PEOPLE_JSON` at it until then. Separately, the city generator
plants ~128 background pedestrians with no knowledge of the fire;
`cull_background_people` hides any further than `FC_PEOPLE_MAX_DIST_M` (120)
from the nearest burning footprint — a guard on the extras, not the survivors.

## Stage 6 — the audit, before the cell is called done

```bash
python3 scene_gen/tools/city_layout_audit.py \
    --gt <cell>/GT_hints.json --dump <the dump the manifest was solved on> \
    --manifest scene_gen/_plans/fire_city_500m.json \
    --png ~/fire_previews/road_overlap/<cell>.png
```

**Pass condition:** `record_xy max 0.00 m` on the `[audit] manifest frame:` line,
zero substitutes overhanging a block, zero on a road, every non-park block with a
building in it (`MIN_FRAC_IN_BLOCK = 0.97`, `CELL_MATCH_TOL_M = 0.5`). No
exit-code gate — read the lines. Offline tests for the same rules:
`scene_gen/tests/test_city_layout_audit.py`, `test_quake_swap_frame.py`,
`test_place_source_frame.py` (`reassemble-buildings-in-frame` has the `uv` line).

## Stage 7 — freeze and export

Follow `freeze-portable-scenes` and `freeze-disaster-dataset`; do not improvise.
The export goes to
`final_disaster_dataset/Fire/Urban/level_<n>/<k>/fire_urban_lvl<n>_<k>.usd`
beside `GT_people.json` and `GT_hints.json`, on its own mount outside the repo.
`freeze.make_portable(stage)` runs inside `export_scene` — rewrites local shader
paths to the Nucleus mirror, copies any material bound from outside a
`/World/<scope>` into that scope's `Looks`, authors `/World/FrozenDome` +
`/World/FrozenSun` — and `verify()` then fails the cell on `sky_lights`,
`build_local` or `cross_scope_bindings`. Root-layer export only, never
`stage.Flatten()`. Fire-specific: the soot PNGs (`FB_TEX_DIR`) and the AEC
soot-layer PNGs (`AEC_BURN_OUT`) are container-local caches, NOT on Nucleus, and
must travel with the frozen cell. `freeze-dataset-state` has the current state.

**The real command, `freeze_urban_fire_city_launch_script.py`'s own
docstring is missing two things that cost a run each (2026-09-02):**

```bash
docker exec isaac-sim tmux send-keys -t isaac 'clear; \
  FREEZE_OUT=/isaac-sim/AirStack/final_disaster_dataset/Fire/Urban/level_1/1 \
  SCENE_CONFIG=downtown_fire_1500 SG_INSTANCE_PLACEMENTS=1 PYTHONHASHSEED=0 \
  FC_MANIFEST=/isaac-sim/AirStack/scene_gen/_plans/fire_city_l1.json \
  FC_BAKES=/isaac-sim/.cache/fire_bakes/city_4 \
  FC_DUMP=/isaac-sim/AirStack/scene_gen/_plans/city_placements_downtown_fire_1500_crop.json \
  FC_CROP_WINDOW=-180.0,180.0,1000,1000 \
  PEOPLE_VARIANT=1 FREEZE_EXPORT=1 FREEZE_SNAPS=1 FREEZE_WAIVE_MIRRORED=1 \
  PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh \
  /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/freeze_urban_fire_city_launch_script.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window' ENTER
```

1. **`--no-window` is missing from the launch script's own module
   docstring — add it anyway, always.** Without it, freeze segfaults inside
   `carb.eventdispatcher`/`omni.appwindow` a couple of seconds into startup
   (`Failed to acquire IWindowing interface` immediately before the crash),
   before any real freeze work happens. Stage 1 and Stage 4's own examples
   in this skill already carry `--no-window` — the freeze script's docstring
   is simply the one place that dropped it; don't copy it verbatim.
2. **`FC_DUMP` for freeze must be the CROPPED dump, not the full one — this
   is a real bug, not the crop-shift bug `reassemble-buildings-in-frame`
   already fixed.** `disaster/fire_people._manifest_matches_dump` compares a
   manifest record's raw `rec["x"]`/`rec["y"]` straight against
   `dp["x_m"]`/`dp["y_m"]` from whatever dump `FC_DUMP` points at — it does
   **not** apply the `record_xy()` / `x_orig` preference the city launcher's
   own `resolve_cell`/`compose_bakes` use. A manifest is always solved
   against the CROPPED dump (its `x`/`y` are in the recentred frame), so
   pointing `FC_DUMP` at the full 1.5 km dump makes every single record fail
   `_manifest_matches_dump` as `geometry_drift`, and freeze silently writes
   an EMPTY `GT_people.json`
   (`fire_people solved ZERO burning buildings against this manifest/dump
   pair`) — the freeze otherwise "succeeds," `portable_ok: True` and all,
   with a cast-free cell. Always pass the SAME `*_crop.json` file the
   manifest was solved against.

## Determinism: `PYTHONHASHSEED=0` is not only for the bake driver

`fire_city_bake.sh` (Stage 3's driver) has always pinned `PYTHONHASHSEED=0`,
documented as fixing `urban_fire.r_render_peel`'s per-process-random `set`
iteration order. **The exact same class of non-determinism also breaks
CITY-WIDE LAYOUT** — Stage 1 (the `FC_INTACT_ONLY=1` dump) and Stage 4/7
(rebuilding the same city fresh from `SCENE_CONFIG`+seed) are SEPARATE Kit
process launches, and nothing in `urban_fire_city_launch_script.py` pinned
the hash seed for either one until this was found.

**Measured 2026-09-02:** two Stage-1 runs of the IDENTICAL preset and seed,
differing only in whether `PYTHONHASHSEED=0` was set, produced 16,066 vs
15,997 total prims — a genuinely different city layout. Composing a
manifest solved against one layout onto a freshly-rebuilt OTHER layout fails
almost totally: `[fc] manifest/city match: 4/34` (or worse), most records
"cell not found" outright (the prim path `/World/stage/generated/house_N_M`
the manifest names does not exist in this build at all) and a few
"frame mismatch" at tens-to-hundreds of metres — reads exactly like the
crop-shift bug in `reassemble-buildings-in-frame`, but the manifest and dump
were never mismatched in FRAME, they were solved against a LAYOUT that this
particular Stage-4 run never reproduced.

**The fix: `PYTHONHASHSEED=0` (and `SG_INSTANCE_PLACEMENTS=1`, which
measurably contributes too — going from unset to set alone improved one
run's match from ~1/34 to 4/34, though it did not fully fix it on its own)
on EVERY stage that runs `urban_fire_city_launch_script.py` for this cell —
Stage 1, Stage 4, and Stage 7 all need identical flags for the parts they
share.** A partial fix (pinning it on Stage 1 only, or Stage 4 only) still
desyncs the two runs; verified only fully clean once all three stages of a
level 1 rebuild used identical `SG_INSTANCE_PLACEMENTS=1 PYTHONHASHSEED=0` —
that run then hit `30/30` manifest/city match, `0 refused`,
`block audit: 0 of 30`.

## `vtk` disappears inside a live SimulationApp even though it's installed

`gac_storey_slice.py`'s `clip()` (used by every `gac`/`dtc` bake) does
`import vtk` lazily. It is genuinely `pip install`ed at
`/isaac-sim/kit/python/lib/python3.11/site-packages/vtk.py`, and a bare
`docker exec isaac-sim /isaac-sim/kit/python/bin/python3 -c "import vtk"`
(no `SimulationApp`) imports it fine. **Booting a real `SimulationApp` and
THEN importing `vtk` from inside a launch script can still raise
`ModuleNotFoundError: No module named 'vtk'`** — Kit's own extension-loading
does not reliably leave the interpreter's default site-packages on
`sys.path` the way a bare python invocation does. Empirically this looks
like a first-Kit-boot-in-the-container's-lifetime issue more than a
per-run one — a SECOND building's bake in the same `fire_city_bake.sh` run
(same container, fresh Kit process) succeeded without the fix. The safety
net that costs nothing: pass
`PYTHONPATH=/isaac-sim/kit/python/lib/python3.11/site-packages` explicitly
on the `fire_city_bake.sh` invocation
(`export ISAAC_SIM_PYTHONPATH=/isaac-sim/kit/python/lib/python3.11/site-packages`
before launching it, since the driver's own template reads
`PYTHONPATH="$ISAAC_SIM_PYTHONPATH"`) rather than trusting the first boot
to work.

## The four bake kinds

| kind | what it is | named by | what the bake does | known gaps |
|---|---|---|---|---|
| `gac` | GreatAmericanCity merged assets, `scale 0.01`, `soot: "bake"` | asset basename (`SM_Building_02`) | `place_source` -> `gac_storey_slice` region cut -> pre-slice atlas soot bake (per-piece for TILED atlases) -> burn -> settle alone -> `rehome_for_export` -> root-layer export | every pre-2026-09-02 bake carries the mirrored-atlas soot and roof-piercing columns |
| `dtc` | Muyang `downtowncity`, `scale 1.0`, `soot: "bake"`, blacklist `Carved_`, `Building_11`, `Building_12` | asset basename | same as `gac` | strip/curtain windows need the `_islands` synthetic bay grid or events go to zero |
| `kit` | ModernCityEnvironment via `urban_building.build_building` + `urban_fire.burn_building` | STYLE (`commercial_mid`) | assembled from named parts, burnt by recipe, settled, exported | the look is FROZEN: `SETTLE_REST_V2` is forced 0 and any change must reproduce byte-for-byte |
| `aec` | AEC brownstone ROWS, `scale 0.01`, `soot: "overlay"` — **no longer sliced** (left `SLICED_KINDS` on 2026-09-02) | asset basename (`Reference_Brownstone5Row`) | referenced RAW and centred by `gac_fire.place_source`; `aec_burn.measure_row` / `plan_row` / `damage_row` / `author_row`; soot on a conformal layer over the untouched MDL brick; F1-F5 damage by part NAME; collapse bodies in the unscaled `<asset>_debris` sibling settled by the launcher's `settle.run`; sidecar gets an `aec` block (`units`, `n_units`, `damage` census, `asset`) | never yet run inside a real city cell; export references the container-local asset path |

`bake_kind` picks the kind by `gac_fire.PACKS[kind]["dir"]` prefix BEFORE
consulting `kit_substitute.route()`, so `route()`'s stale `('slice', None)` for
an AEC path never fires. An `aec` bake is doomed-subtree-free, so
`verify_export` gets a sentinel (`("/__nothing_doomed__",)`) instead of the
`/src` default — its materials legitimately live there.
`fire_city_manifest.entry_string` names `gac`/`dtc`/`aec` by `asset` and `kit`
by `style`: the test is `kind != "kit"`, NOT `fb.SLICED_KINDS`, which `aec` left
while staying an asset.

## Env knobs — layout, assembly and people

Bake knobs are the stage 3 table. All of these go through the launcher's `_env`
helper: this container exports every launcher variable as an EMPTY STRING, so a
plain `os.environ.get(name, default)` never reaches its default.

| env | default | stage | what it does |
|---|---|---|---|
| `SCENE_CONFIG` | `downtown_fire_500` | 1, 4 | preset; compiled with `disaster-type` forced to `none` |
| `FC_INTACT_ONLY` | `0` | 1 | build the city, write the dump, stop |
| `FC_DUMP` | `_plans/city_placements_<preset>_<seed>.json` | 1 | placements dump path (repo mount, host-visible) |
| `FC_MANIFEST` | newest `_plans/fire_city_*.json` | 4 | the manifest to compose |
| `FC_BAKES` | `/isaac-sim/.cache/fire_bakes/city_<seed>/` | 4 | bake dir OR comma list of `.usd` — paste the driver's list |
| `FC_CROP_WINDOW` | unset | 4 | `cx,cy,W,H`; deactivates outside the window, never moves the stage |
| `FC_UNINSTANCE_GPRIM_ROOTS` | `1` | 4 | second repair for instancing ghosts |
| `FC_HIDE` / `FC_SKY` / `FC_ENV` | `invisible` / `sunset` / `default` | 4 | how intact prims are hidden, sky, environment |
| `FA_FLOW` / `FA_SMOKE` | `1` / `1` | 4 | Flow stack; 0 = flames only |
| `FA_CELL_M` / `FA_MAX_BLOCKS` | `0.55` / `12288` | 4 | Flow density cell, block pool (`rtx/flow/maxBlocks`) |
| `FA_EMITTERS` / `FA_EMITTER_BUDGET` | `30` / `800` | 4 | per-building opening ceiling / GLOBAL emitter cap |
| `FC_SCORCH_VEG` / `FC_FIRE_APRON` / `FC_HIDE_PROPS` / `FC_CONTACT_SNAP` | all `1` | 4 | scorched greenery, ground apron, companion props, opening snap |
| `FC_PEOPLE` / `FC_PEOPLE_JSON` | `1` / `""` | 4, 5 | people pass; the reviewed records file |
| `FC_PEOPLE_MAX_DIST_M` | `120` | 4, 5 | background-pedestrian cull radius from the nearest burning footprint |
| `FC_BASELINE_CAPTURES` / `SNAP_DIR` / `KEEP_OPEN` | `1` / `""` / `0` | 4 | capture pass, output dir, keep the app up |

## Files

Paths are relative to `scene_gen/`, except `launch_scripts/` =
`simulation/isaac-sim/launch_scripts/`.

| file | role |
|---|---|
| `config/presets/downtown_fire_500.yaml` | the 500 m plate; also `downtown_fire_1500{,_lvl2,_lvl3}.yaml` |
| `config/harvested/burnability_table.json` + `tools/gen_burnability_table.py` | the firebreak table and the tool that proves it against the real gate |
| `tools/fire_city_dry_run.py` + `tools/fire_city_union.py` | one-seed solve; multi-seed union (`--auto`, `--profile`, `--max-records`) |
| `tools/fc_dump_crop.py` / `tools/bake_gac_kits.py` | re-centred 1 km window out of a 1.5 km dump / the one-time GAC kit cache |
| `tools/fire_city_manifest.py` | record -> entry string + cache stem; HAVE/NEED classification; `--print-seed`; `--write-city-json` |
| `tools/fire_city_bake.sh` / `tools/fire_bake.sh` | the city driver (cache, settle tiers, city JSON) / the row driver (manual entries, `FB_UNITS`) |
| `tools/fire_people_rerun.sh` / `tools/city_layout_audit.py` / `tools/render_preflight_fire.sh` | the people gate / the frame + on-road + empty-block audit / the stale-file refusal |
| `launch_scripts/fire_bake_launch_script.py` | one building per Kit process: `build_gac` / `build_aec` / `build_kit` |
| `launch_scripts/urban_fire_city_launch_script.py` | the assembly: `compose_bakes`, `load_fire`, `record_xy`, frame guard, block audit, Flow, `cull_background_people` |
| `disaster/fire_bake.py` | `KINDS`, `SLICED_KINDS = ("gac","dtc")`, `parse_entry`, sidecar, `verify_export` |
| `disaster/urban_fire_city.py` | `bake_kind`, `burnable`, `damaged_manifest`, `entry_string` |
| `disaster/aec_burn.py` | `LADDER`, `pick_units`/`parse_units`, `measure_row`/`plan_row`/`damage_row`/`author_row`/`burn_row`, `flames_row` |
| `disaster/gac_fire.py` | `PACKS`, `place_source`, `bake_atlases` (the 3D shared-texel test) |
