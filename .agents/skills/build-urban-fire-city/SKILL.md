---
name: build-urban-fire-city
description: 'The end-to-end PROCEDURE for building a full burning-downtown cell — layout, fire manifest, per-building bakes, assembly, people, audit, freeze — from the `downtown_fire_*` presets and the `fire_city_*` tools. Seven numbered stages, each with the exact command, the env knobs and their defaults, where the outputs land (host vs container), the gate that must pass before the next stage, and the trap that costs a run: clearing the GAC kit cache and stale `FB_OUT` accumulation; `FC_INTACT_ONLY=1 FC_DUMP=` to dump placements and `gen_burnability_table.py --prove` to regenerate the firebreak table; `fire_city_dry_run.py` + `fire_city_union.py --auto/--seeds/--max-records` and the crop / `FC_CROP_WINDOW` / `x_orig` frame rule; `fire_city_bake.sh` over the four bake kinds (gac, dtc, kit, and — new 2026-09-02 — `aec`, the brownstone rows that burn by NAME through `aec_burn` instead of being sliced), the per-level settle tiers, `SETTLE_REST_V2`, and what a good bake log says; the assembly through `urban_fire_city_launch_script.py` with its frame guard, block audit, emitter budget and the Vulkan Flow-OOM grep that separates "a city on fire" from "a city with no smoke and every count right"; `fire_people_rerun.sh`; `city_layout_audit.py`; and the freeze. Read before rebuilding any urban-fire layout — and note that EVERY gac/dtc bake made before 2026-09-02 must be re-baked.'
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Building a full urban fire layout, end to end

`build-urban-fire-scenes` is the physics, the ladder, the recipes and the bug
catalogue — the WHY. This file is the WHAT-YOU-TYPE: the seven stages that take
a preset to a burning 500 m / 1 km / 1.5 km downtown cell, in order, with the
gate for each. Read that skill before changing any damage code; read this one
before rebuilding a layout.

```
0. clear    kit cache, stale bakes (ALL pre-2026-09-02 ones), stale manifests
1. layout   preset -> SCENE_CONFIG -> FC_INTACT_ONLY=1 FC_DUMP=<dump.json>
                                   -> gen_burnability_table.py --prove
2. manifest fire_city_dry_run.py (one seed) -> fire_city_union.py (many seeds)
                                   -> [fc_dump_crop.py for a 1 km window]
3. bakes    fire_city_bake.sh MANIFEST         one Kit process per building
                                               kinds: gac | dtc | kit | aec
4. assembly urban_fire_city_launch_script.py   bakes as static + Flow put back
5. people   fire_people_rerun.sh -> FC_PEOPLE_JSON
6. audit    city_layout_audit.py               the frame gate
7. freeze   freeze-portable-scenes / freeze-disaster-dataset
```

Every Isaac step is run through the `run-isaac-sim-launcher` pattern (`airstack
up isaac-sim`, then `docker exec isaac-sim tmux send-keys`); `docker logs
isaac-sim` is empty for this container. Do not re-derive that here.

## Stage 0 — prerequisites and what to CLEAR

**Why this stage exists today.** Two fixes landed 2026-09-02 that invalidate
every existing gac/dtc bake — most of "we will have to recreate all the
layouts". `gac_fire.bake_atlases`' shared-texel test compared only the height
(`dz`) between its forward and reverse rasterisations, so an atlas MIRRORED
left-to-right passed as unique and was baked once by world position: the soot
laid at the burning SE/NE corners landed on the clean SW/NW corners too. It now
compares the full 3D distance (`d3 = np.linalg.norm(pa[both] - pb[both],
axis=1)`), and an atlas that now flags TILED takes the per-piece bake after the
slice. Separately, `quake_flow.fit_interior` ends the top storey's columns at
`m.get("deck_z", m["top"])`, not the parapet-coping bbox top — pre-fix GAC
bakes have columns standing through the roof.

```bash
airstack up isaac-sim          # the container, per run-isaac-sim-launcher

# 1. GAC kit cache — have_kit() FALSE means every gac: bake does a LIVE slice
#    (most of its wall time, and it takes the machine-wide slicer lock).
#    -> scene_gen/assets/kits/<asset>.usd (gitignored) + kits.json (tracked)
#    gate: "GAC KIT BAKE <ok>/<total>" with no "WARNING ... subset(s) rehomed"
docker exec isaac-sim bash -c '/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
    /isaac-sim/AirStack/scene_gen/tools/bake_gac_kits.py --assets all'

# 2. stale bakes — container-side, under $FB_OUT/city_<seed>/
docker exec isaac-sim bash -c 'rm -rf /isaac-sim/.cache/fire_bakes/city_<seed>'
```

**`FB_OUT` accumulates and nothing prunes it.** It defaults to
`/isaac-sim/.cache/fire_bakes` — CONTAINER path, not the repo, because the bakes
are working files by the user's own instruction. `fire_city_bake.sh` appends
`city_<seed>/`; `fire_bake.sh` writes flat into `$FB_OUT`. A re-bake at a
different seed leaves the old stems in place, and a directory handed to the
assembly quietly builds however many columns it finds — **always paste the
explicit comma list the driver prints.**

**Never reuse an old manifest on a new dump.** Its record indices name whatever
building sat at that cell in the dump it was solved against; `FIRE_MAX_H_M`
(232.0, just above `Amar_Tower`'s 231.4 m) and the `dtc` pack blacklist
(`Carved_`, `Building_11`, `Building_12`) are two more gates a stale manifest
can silently disagree with. Re-solve.

**Rendering on a pod:** `OSMO_SSH_LOCAL_PORT=2204
scene_gen/tools/render_preflight_fire.sh` content-hashes every module the city
launcher imports transitively and refuses a render against stale files
(`build-scenes-on-osmo`).

## Stage 1 — the layout, and the placements dump

The presets: `downtown_fire_500` (500 m plate, `seed: 4`, asset-set
`urban_gac`, `instance_placements: true`) and the three 1.5 km baseline levels
`downtown_fire_1500{,_lvl2,_lvl3}`. `downtown_fire_500.yaml` restricts
`usds.buildings.lowrise` (26 -> 7) and `midrise_v2` (22 -> 14) to burnable-only
pools — an unburnable asset is a firebreak, and the shared `urban_gac` pools
measured 10/15 `lowrise` and 10/29 `midrise` unburnable.

```bash
docker exec isaac-sim tmux send-keys -t isaac 'clear; \
  ISAAC_SIM_HEADLESS=true SCENE_CONFIG=downtown_fire_500 \
  SG_INSTANCE_PLACEMENTS=1 PYTHONHASHSEED=0 \
  FC_INTACT_ONLY=1 \
  FC_DUMP=/isaac-sim/AirStack/scene_gen/_plans/fc_dump_500.json \
  SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/fire_city_intact \
  PYTHONUNBUFFERED=1 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh \
  /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window' ENTER
```

**`SG_INSTANCE_PLACEMENTS=1` and `PYTHONHASHSEED=0` on EVERY stage that
touches layout, not just the bake driver — see the determinism section
after Stage 7.** Both are load-bearing for Stage 1's dump to actually match
what Stage 4 rebuilds from the same preset/seed; omitting either on one
stage but not the other reproduces a different-looking city and every
manifest record fails to find its cell.

`FC_INTACT_ONLY=1` builds the city, writes the dump and stops (only
`city_top.png` is captured). `FC_DUMP` defaults to
`scene_gen/_plans/city_placements_<preset>_<seed>.json`; the repo is
bind-mounted 1:1 at `/isaac-sim/AirStack`, so the dump is host-visible.

Regenerate the firebreak table whenever a pool, a typology's `pools:` list, a
pack blacklist, `FIRE_MAX_H_M`, or `kit_substitute.route`/`bake_kind` changed:

```bash
python3 scene_gen/tools/gen_burnability_table.py --preset downtown_fire_500 --prove --seed 4
```

Writes `scene_gen/config/harvested/burnability_table.json` (CHECKED IN;
`assets[typology][asset_basename] -> bool`, 79 rows over 6 typologies).
**Gate:** the two `[prove] ... (must be 0)` intersections both read 0 and it
prints `PROOF OK`; it exits 1 on `PROOF FAILED`. Keyed by TYPOLOGY, not pool —
the `midrise` typology draws from pool `midrise_v2`.

**Trap — instancing ghosts.** A placement whose composed ROOT prim is itself a
Mesh renders as nothing (or flat grey) once `SetInstanceable(True)` is applied:
measured 14 of 110 distinct assets, 161 of 1,469 placements. `apply_placements`
now refuses it and `FC_UNINSTANCE_GPRIM_ROOTS=1` (default) repairs it again at
the launcher; diagnose with `scene_gen/tools/fc_instance_material_probe.py`.

## Stage 2 — the fire manifest

One ignition is a Dijkstra tree from one origin; some origins reach 30
buildings, some reach 1. Solve once per seed on the SAME dump, then union.

```bash
# ONE seed. --preset for a fresh layout; --placements-json against stage 1's
# dump is the normal path. Host-side, no Kit, no docker, no Nucleus.
cd scene_gen && uv run --python 3.13 --with usd-core --with numpy --with pyyaml \
    python tools/fire_city_dry_run.py --placements-json _plans/fc_dump_500.json --n 16
```

Args: `--preset downtown_fire_500`, `--seed`, `--n 16`, `--collapse 1` (target
F5c count), `--roof-collapse-max` (overrides `ROOF_COLLAPSE_MAX_DEFAULT`, 2),
`--out`, `--md`. Defaults to `scene_gen/_plans/fire_city_<seed>.json` +
`_report.md`. **Gate:** six checks (`district_rule`, `contiguity`,
`level_distribution`, `entry_points`, `bakeability`, `footprint`) plus
`determinism` all print `PASS`; the tool exits 1 otherwise.

```bash
# MANY seeds, unioned. Explicit and reproducible:
python3 scene_gen/tools/fire_city_union.py scene_gen/_plans/fc_dump_500.json \
    --seeds 43,35,0 --n 40 --collapse 1 --roof-collapse-max 2 \
    --out scene_gen/_plans/fire_city_500m.json \
    --md  scene_gen/_plans/fire_city_500m_report.md
# or let it find its own seeds against a REPLACEMENT dump:
python3 scene_gen/tools/fire_city_union.py scene_gen/_plans/fc_dump_NEW.json --auto \
    --sweep-max 500 --target-min 20 --target-pref 26 --out ... --md ...
```

`--auto` selects on CONCENTRATION (`adjacency_share`, `n_components` at
`--adjacency-m 25`, `street_facing_share`), not raw count. `--profile
baseline_l1|l2|l3` carries the built intensity ladder (burn fractions 0.12 /
0.26 / 0.38, roof-collapse budgets 0 / 2 / computed). `--max-records N` also
writes a VRAM-constrained subset beside `--out` as `<out>_<N>.json` — ~250 MB per
composed bake measured, so a full union does not fit a 16 GB card but does fit a
32/48 GB one. **Gate:** the same PASS table plus `subset stem verification: PASS`.

**The crop / frame rule — the bug that shipped three cells.** For the dataset's
1 km contract on a 1.5 km plate:

```bash
python3 scene_gen/tools/fc_dump_crop.py --in scene_gen/_plans/<dump>.json \
    --centre 60 90 --size 1000 1000 --out scene_gen/_plans/<dump>_crop.json
#   (or --window X0 Y0 X1 Y1; --no-recenter is for debugging the crop math only)
```

Such a dump is RE-CENTRED, so every record solved on it carries cropped
coordinates and the stamp `x_orig`/`y_orig` is the only full-city value on it.
`record_xy` is the ONE coordinate rule (`x_orig` wins), `compose_bakes` uses it,
and `load_fire` un-shifts on the stamp whether or not `FC_CROP_WINDOW` is set;
set `FC_CROP_WINDOW=cx,cy,W,H` at launch to deactivate everything outside the
window (it never resizes or translates the stage). Getting this wrong put every
bake one window-centre off its cell: "buildings are spawning on top of the road,
they are empty blocks" (user, 2026-09-01). See `reassemble-buildings-in-frame`.

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

## Known gaps / next

* **The lost wall is bricks and its own fittings, no slabs.** In an `aec`
  F5 bake the cut-out wall becomes `RUBBLE_BRICKS` brick-sized bodies plus
  the window trims, doors and wall packs that sat in the lost region
  (`_lose_wall`, "wall_part": rigid bodies with the wall's outward push —
  left in place they hung in the hole as "floating window borders", user
  2026-09-02); `WALL_STRIP_BODIES = False` because settled wall strips read
  as grey concrete panels in the street. Verified offline only (F5 probe:
  893 loose bodies, 11 wall parts on the 5Row); the first settled bake of it
  is still to be looked at.
* **The AEC bake path has never been run inside a real cell.** `build_aec` is
  wired and its pure helpers are covered by `scene_gen/tests/test_aec_burn.py`,
  but every render so far is from `aec_material_probe` / `aec_gac_showcase`,
  not from `fire_city_bake.sh` -> the city launcher. Watch the holder frame and
  the export on the first city run.
* **The AEC export references the container-local asset path.** The brownstone
  tree is mirrored at
  `omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/aec/brownstone/`
  with relative references, so a Nucleus-anchored `AIRSTACK_ASSET_ROOT` resolves
  it by construction — but the soot PNGs are not, and `_lose_wall`'s re-authored
  remainder binds the LIVE source material because the self-contained clone
  renders white. Run the portable pass, then check `build_local` on the cell.
* **`FB_UNITS` does not reach a city bake.** `fire_bake.sh` passes it through,
  `fire_city_bake.sh` does not, so an `aec` record in a city manifest always
  takes `pick_units`' seeded contiguous draw (one or two units, two 70% of the
  time). Fine for variety, not controllable per record yet.
* **Per-piece atlas routing costs texture memory.** An atlas that now correctly
  flags TILED takes the post-slice per-piece bake at `SOOT_BAKE_PX_SLICE` (256,
  128 small) instead of one shared atlas. Shares that flip:
  `M_Building_05_WallBack` 0% by height vs 100% in 3D; `SM_Building_02`'s
  `Concrete` 4% vs 98%, awnings and trim 76-100%. Budget for it.
* **The Flow budget is per-file and unguarded** — no runtime check that a given
  scene/GPU has the VRAM before Flow allocates. Past the defaults above, the
  OOM grep is the only truth; the banner is not.
* **`tower`/`highrise` pools are still not burnability-restricted** — no
  burnable substitute exists at that footprint class, so their unburnable
  fraction stays a firebreak by default.
