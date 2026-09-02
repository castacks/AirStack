# RAVEN test scene runbook — `suburb_tornado_250`, two drones, two casualties

Owner: WP-D (`_plans/raven_single_rayfronts_shared_plan.md` §6). Produced
entirely offline — nothing in this file has been run against Isaac Sim. The
host-side commands (§1) WERE run, and their output is what the rest of this
file is built from. **Do not launch Isaac Sim, edit `.env`, or touch any
launch script from this runbook without the user's go-ahead** — §2 and §3 are
the exact commands to run when that clearance is given.

---

## 0. What this scene is for

Two drones (`robot_1`, `robot_2`), one shared off-board RayFronts server, one
`raven_nav` instance each, searching a 250 x 250 m tornado-hit suburb for
`person`. `robot_1` starts 20 m from a fully exposed casualty; `robot_2`
starts 20 m from a partially-covered one (`flank`, 33% covered). Both spawns
sit on open, low-intensity ground, not inside a house, not on the swept
centreline.

---

## 1. Host dry runs (already done — reproduce with these exact commands)

Every command below was run from `scene_gen/` with `uv` supplying
`numpy`/`matplotlib`/`pyyaml`/`shapely`/`scikit-learn` on top of the system
`python3` (no `pxr`, no Isaac Sim, no Nucleus). `fence_png.py` (imported by
`tornado_png.py`) stubs the seven `pxr` submodules `scene_generator.py` /
`suburb_scene.py` import at module scope, so the WHOLE offline suburb+damage
pipeline — layout, house/tree damage ladder, plank field, scour relief, AND
`disaster.tornado_people.plan_people` — runs on a bare host.

### 1a. The track PNG

```bash
cd scene_gen
uv run --with numpy --with matplotlib --with pyyaml --with shapely \
    --with scikit-learn python3 tools/tornado_png.py \
    --config suburb_tornado_250 \
    --out /home/krrishjain/raven_previews/suburb_tornado_250_track.png
```

Output (verbatim): `region 250 x 250 m, seed 10`, `track 78 m wide toward 38
deg through (-45, -75)`, `in path 29.9% of the plate, mean intensity 0.46,
peak 0.89`, `houses 60 total, 13 damaged` (3 roof_stripped, 2 roof_collapsed,
5 partial_collapse, 1 leveled, 2 swept), `trees 65 host-side, 28 damaged`,
`plank debris 1220 + 329 = 1549 board(s)`, `scour relief 522 feature(s)`, and
**`OK  gradient and coverage both in band`** — no warnings from the tool's own
three failure checks (track too big/small, one damage class dominating,
fewer than 8 damaged houses).

### 1b. The people plan — `plan_people` DOES run without a stage

`disaster.tornado_people.plan_people(cfg, ctx, rng)` imports nothing from
`pxr` (checked: `math`, `os`, `random` only) and every `ctx` input it reads
(`wrecks`, `intact`, `canopies`, `road_pts`, `plank_specs`, `blockers`,
`intensity_at`, `region`, `humans`, `resolver`, `asset_pools`) is buildable
from `fence_png.build()`'s own return dict plus the same pure functions
`tools/tornado_png.py` already calls (`disaster.tornado.house_level_for_
intensity` / `tree_level_and_yaw`, `disaster.planks.scatter_from_wreck` /
`scatter_over_region`, `disaster.scour_relief.scatter`). The ONE thing that
needs a live stage — `deck_points`, measured off the BAKED WRECK ARCHETYPES
via `Usd.TraverseInstanceProxies` + `BBoxCache` — is documented as OPTIONAL
in `plan_people`'s own docstring and degrades to the flat per-level
`DEBRIS_Z_M` deck, which is exactly the bench/host-test convention the module
already uses for its own catalogue bench.

The tool that does this — **promoted into the repo**, `scene_gen/tools/
tornado_people_dry_run.py` (the original session-scratchpad prototype does
not survive the session, so this is the one to keep re-running). It builds
the layout/damage/plank/relief pipeline (identical to `tornado_png.py`),
runs `plan_people`, writes the casualty JSON, prints every casualty as
`idx, x, y, pose, occlusion, covered_frac`, picks two of them (nearest the
plate centre by default, or `--pick-exposed IDX` / `--pick-partial IDX` to
pin specific ones), searches a ring `--spawn-dist-m` out from each for the
lowest-intensity point clear of every house footprint, and prints the
resulting `SPAWN_CONFIGS` JSON. `--overlay` additionally draws the track PNG
with every casualty coloured by occlusion, the two picks ringed, and both
spawns + facing arrows — the same run that writes the JSON, not a second
build.

```bash
cd scene_gen
uv run --with numpy --with matplotlib --with pyyaml --with shapely \
    --with scikit-learn python3 tools/tornado_people_dry_run.py \
    --config suburb_tornado_250 \
    --out-json /home/krrishjain/raven_previews/suburb_tornado_250_people.json \
    --overlay /home/krrishjain/raven_previews/suburb_tornado_250_people_overlay.png \
    --pick-exposed 16 --pick-partial 9
```

`--pick-exposed 16 --pick-partial 9` PINS THE PAIR THIS RUNBOOK USES. A bare
invocation (no `--pick-*`) auto-picks the `none` casualty and the
`covered_frac in [0.30, 0.55]` casualty each NEAREST THE PLATE CENTRE, which
for this scene lands on idx 16 and idx **2** — a fine default in general, but
idx 2 sits only ~20 m from idx 16 (their spawn rings would nearly touch,
which defeats the point of a TWO-drone, two-sector test). idx 9 sits on the
far side of the corridor from idx 16 (~42 m apart), which is the pair this
runbook actually wants — hence the explicit pick.

Output: **19 casualties placed** (refusals: `in_relief` 9, `in_wreck` 3,
`off_track` 6, `too_close` 11, `wrong_surface` 33 — the planner trying many
more candidates than it keeps, which is its own designed behaviour, not an
error), written to
`/home/krrishjain/raven_previews/suburb_tornado_250_people.json`, plus the
full candidate table and:

```
[tornado_people_dry_run] robot_1: casualty idx=16 (-4.87,-24.86) occlusion=none covered_frac=0.00 -> spawn (-3.12,-4.94) intensity=0.08 yaw=-95.0 deg
[tornado_people_dry_run] robot_2: casualty idx=9 (-16.46,-64.78) occlusion=flank covered_frac=0.33 -> spawn (-3.61,-80.10) intensity=0.13 yaw=130.0 deg
[tornado_people_dry_run] SPAWN_CONFIGS = [{"x_m": -3.12, "y_m": -4.94, "orient": [0.0, 0.0, -0.7373, 0.6756]}, {"x_m": -3.61, "y_m": -80.1, "orient": [0.0, 0.0, 0.9063, 0.4226]}]
[tornado_people_dry_run] overlay -> /home/krrishjain/raven_previews/suburb_tornado_250_people_overlay.png
```

**Caveat, stated once and load-bearing for §2**: this file is a HOST
APPROXIMATION. The real Isaac build's `deck_points` will be non-empty
(measured off the actual baked wreck geometry instead of the flat
`DEBRIS_Z_M` fallback), which can change a handful of `_Deck`-tilt
accept/refuse decisions near a wreck pile — so the real `PEOPLE_JSON` may not
have EXACTLY 19 casualties at EXACTLY these coordinates. `intact`/`wrecks`/
`canopies`/`road_pts`/the plank field are otherwise identical (same seed, same
pure functions), so the population should be close, not identical. **§2's
"re-validate the two casualties" step, below, re-checks this against the REAL
file before anything is flown.**

### 1c. The casualty overlay PNG

Written by the SAME command as §1b (`--overlay ...`), not a second run:
`/home/krrishjain/raven_previews/suburb_tornado_250_people_overlay.png` — the
track PNG with all 19 casualties scattered and coloured by `occlusion`, the
two chosen casualties ringed (`A` = idx 16, `B` = idx 9), and both robot
spawns + facing arrows drawn.

### 1d. The annotation conversion, previewed

```bash
cd scene_gen
python3 tools/people_json_to_annotations.py \
    --people /home/krrishjain/raven_previews/suburb_tornado_250_people.json \
    --out /home/krrishjain/raven_previews/RavenSuburbTornado250_annotations_preview.json
```

Output: `19 person annotation(s), by occlusion {'midriff': 4, 'none': 7,
'flank': 3, 'feet_shins': 2, 'banded': 1, 'torso_head': 2}`. **This preview
file is NOT the real annotations file** — §3's closing step re-runs this
exact command against the real `GT_people.json` once the scene is actually
built and flown, into
`robot/ros_ws/src/global/planners/raven_nav/annotations/<RESULTS_SCENE>.json`.

### 1e. Unit tests

```bash
python3 -m pytest scene_gen/tests/test_people_json_to_annotations.py -q
python3 -m pytest scene_gen/tests/test_tornado_people_dry_run.py -q
```

Results: **16 passed** for the annotation converter (bare `python3`, no
`uv`, no extra packages — the tool and its test only use
`argparse`/`json`/`math`/`os`/`sys`); **19 passed** for
`tornado_people_dry_run`'s pure half (also bare `python3` — the test file
only imports the tool module, never `fence_png`/`suburb_scene`/`disaster.*`,
so nothing pulls in `numpy`/`matplotlib`/`pxr`). The test file covers the
spawn-point math (`yaw_facing`'s distance-independent yaw + unit quaternion,
`spawn_config_entry`'s rounding), the exposed/partial pick logic on a
synthetic 5-record list (nearest-to-centre tie-break, the partial band's
edges, explicit picks honoured with and without a criteria mismatch, both
"no candidate" error paths), and `pick_spawn_point`'s ring search against
synthetic house/intensity callables (avoids a blocked footprint, prefers
lower intensity, returns `None` when the whole ring is blocked) — no scene
build anywhere in it.

---

## 2. Build + freeze the scene (Isaac Sim, one launch — NOT run by this pass)

**Why a freeze, and not a direct fly.** `suburb_tornado_launch_script.py`
(the ONLY launcher that runs `disaster.tornado_people.plan_people`, the
tornado-appropriate casualty model) never spawns a drone — confirmed by grep:
no `SPAWN_CONFIGS`, `spawn_px4_multirotor_node` or `DRONE_CONFIGS` reference
anywhere in that file, only scene assembly + an optional freeze/export block.
Drones are spawned ONLY by `example_multi_drone_scene_import.py`, which reads
`SPAWN_CONFIGS` (`frames_scene_testenv.md` §A) — but that launcher's own
*generated-scene* path (`SCENE_CONFIG=... REGION_M=... DISASTER_TYPE=tornado`)
places casualties with `disaster.people`, the WILDFIRE/age-based model, which
`place-people-in-tornado-scenes/SKILL.md` explicitly says does not transfer to
tornado scenes. There is no single-launch path that has both the right
casualty model and a spawnable drone — the freeze bridges them: build once
with the tornado launcher (right people model), freeze/export the stage to a
portable `.usd`, then fly it with the generic multi-drone importer's
`FROZEN_SCENE` path (`benchmark-disaster-dataset/SKILL.md` §2b), which spawns
drones from `SPAWN_CONFIGS` same as any other scene and does not re-run any
people model at all — it just re-shapes the frozen cell's own `GT_people.json`
for GT tooling.

`suburb_tornado_launch_script.py` already has full freeze/export wiring
(`FREEZE_OUT`, `FREEZE_EXPORT`, `FREEZE_EXIT`, `disaster.freeze.export_scene`)
— confirmed by reading the script (unlike the hurricane launcher, which
`freeze-disaster-dataset/HURRICANE_RUNBOOK.md` found has none of this yet).

### `.env` block — STAGE 1 (build + freeze; no robots yet)

```bash
ARCH_DIR="/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado"
SCENE_CONFIG="suburb_tornado_250"
ISAAC_SIM_SCRIPT_NAME="suburb_tornado_launch_script.py"
TOR_SEED="10"                    # MUST equal the preset's layout `seed: 10` —
                                  # suburb_tornado_100.yaml's own comment: "the
                                  # two must agree or the plan PNG is not the
                                  # scene you get". This also makes the real
                                  # PEOPLE_JSON reproduce §1b's preview as
                                  # closely as the deck_points caveat allows.
TOR_PEOPLE="1"
PEOPLE_JSON="/isaac-sim/AirStack/_test_freeze/raven_suburb_tornado_250/humans_10.json"
FREEZE_OUT="/isaac-sim/AirStack/_test_freeze/raven_suburb_tornado_250"
FREEZE_NAME="RavenSuburbTornado250"   # explicit — FREEZE_OUT is not a
                                       # <Disaster>/<Locale>/level_<n>/<k>
                                       # path, so the launcher's own name
                                       # auto-derivation would fall back to
                                       # the literal string "scene"
FREEZE_EXPORT="1"
FREEZE_EXIT="1"                  # close after export; do not idle-loop
```

`PEOPLE_SNAPS` may be set >0 for a close-in photograph of the worst-covered
casualties, useful for a human sanity check before flying (not required).

### Ordered commands

```bash
# 1. apply the STAGE 1 block above to .env (NOT done by this pass)
# 2. airstack down
# 3. airstack up isaac-sim
airstack down
airstack up isaac-sim
# 4. WAIT for the launcher's own completion banner — with FREEZE_EXIT="1"
#    that is "[tornado] FREEZE_EXIT set — closing after the export" (see
#    run-isaac-sim-launcher/SKILL.md for reading the tmux pane / piped log /
#    Kit log instead of `docker logs`, which is empty for this container)
```

### What to look for

- `[tornado] plank field: ...`, `[tornado] people: N authored, ... ground
  truth -> /isaac-sim/AirStack/_test_freeze/raven_suburb_tornado_250/humans_10.json`
  — should read close to `19 authored` per §1b, not wildly different.
- `[tornado] EXPORT FAILED` must NOT appear. If it does, it is either a
  generic exception (read the traceback the script prints) or a
  `PortabilityError` — either way `freeze_report.json` under `FREEZE_OUT`
  has the detail; see `freeze-portable-scenes/SKILL.md`'s `verify()` gate.
- `FREEZE_OUT` should contain, when done: `GT_hints.json`, `GT_people.json`
  (a copy of `PEOPLE_JSON`), `freeze_report.json`, and the exported `.usd`
  tree `disaster.freeze.export_scene` writes.

### 5. Before flying — re-validate the two casualties against the REAL file

```bash
python3 - <<'PY'
import json
d = json.load(open("/isaac-sim/AirStack/_test_freeze/raven_suburb_tornado_250/humans_10.json"))
for i, p in enumerate(d["people"]):
    if p["occlusion"] == "none":
        print("NONE  ", i, p["x"], p["y"], p["z"])
    if 0.30 <= p["covered_frac"] <= 0.55:
        print("PARTIAL", i, p["occlusion"], p["covered_frac"], p["x"], p["y"], p["z"])
PY
```

Confirm a casualty near `(-4.9, -24.9)` is still `occlusion: none` and one
near `(-16.5, -64.8)` is still a partial pattern in `[0.30, 0.55]`.

If the real draw moved them (the `deck_points` caveat above): the LAYOUT
(houses, track, intensity field) is unaffected by `deck_points` — only which
candidate casualties get accepted can differ — so `tools/
tornado_people_dry_run.py --config suburb_tornado_250` (no other flags) still
regenerates the identical house footprints and intensity field the real
build used. Find the index in ITS candidate table whose `(x, y)` matches the
real file's nearest surviving casualty to `(-4.9, -24.9)` / `(-16.5, -64.8)`
and re-run with `--pick-exposed <that index> --pick-partial <that index>` to
get a spawn search against the real geometry; if neither survived at all,
`tools/tornado_people_dry_run.py`'s own `pick_casualties`/`pick_spawn_point`
functions can be called directly (import the module, no CLI) against the
real `PEOPLE_JSON`'s `people` list plus a fresh `build_ctx(...)`'s
`houses`/`inten`/`tn.frame(tcfg)` for the geometry callbacks. Either way, do
not fly the §3 numbers unmodified against a population that no longer has
these two casualties where §3 assumes.

**Do NOT write `raven_nav/annotations/RavenSuburbTornado250.json` yet** — see
§3's closing step for why running `people_json_to_annotations.py` now would
just get overwritten.

---

## 3. Fly it (Isaac Sim, second launch — NOT run by this pass)

### `.env` block — STAGE 2 (fly, 2 robots, from the frozen cell)

```bash
ISAAC_SIM_SCRIPT_NAME="example_multi_drone_scene_import.py"
FROZEN_SCENE="/isaac-sim/AirStack/_test_freeze/raven_suburb_tornado_250"
# SCENE_CONFIG should be UNSET/empty. The importer's own guard
# (`_FROZEN = bool(FROZEN_SCENE)`, `_BUILT = bool(SCENE_CONFIG) and not
# _FROZEN`) means a leftover SCENE_CONFIG from STAGE 1 does not get built —
# FROZEN_SCENE wins — but the script prints "FROZEN_SCENE is set, so
# SCENE_CONFIG=... is IGNORED" every time it is left set, which is noise
# worth avoiding by just clearing it.
NUM_ROBOTS="2"
SPAWN_CONFIGS='[{"x_m": -3.12, "y_m": -4.94, "orient": [0.0, 0.0, -0.7373, 0.6756]}, {"x_m": -3.61, "y_m": -80.1, "orient": [0.0, 0.0, 0.9063, 0.4226]}]'
RESULTS_SCENE="RavenSuburbTornado250"
ARCH_DIR="/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado"
TOR_PEOPLE="1"          # inert on this path (no tornado_people call here);
                         # kept for consistency with STAGE 1, harmless either way
PEOPLE_JSON="/isaac-sim/AirStack/_test_freeze/raven_suburb_tornado_250/humans_10.json"
GT_ANNOTATIONS="on"     # writes <RESULTS_SCENE>.json, <RESULTS_SCENE>_
                         # obstacles.json and <RESULTS_SCENE>_region.json into
                         # BOTH gcs_visualizer/annotations and raven_nav/
                         # annotations at scene-load time
                         # (benchmark-disaster-dataset/SKILL.md, "Two
                         # annotation files per scene"). IMPORTANT: the
                         # people file it writes uses `frozen_annotations.
                         # people_boxes`'s UPRIGHT box convention
                         # (PERSON_SIZE_M = (0.7, 0.7, 1.8), centred at
                         # z + 0.9) — correct for a standing wildfire/
                         # hurricane survivor, WRONG for a lying tornado
                         # casualty. This auto-write WILL CLOBBER
                         # `raven_nav/annotations/RavenSuburbTornado250.json`
                         # the moment the scene loads — see the closing step
                         # below, which overwrites it again, correctly,
                         # AFTER the scene is up.
ENABLE_LIDAR="false"    # raven_nav's subscription list in the shared plan
                         # (raven_single_rayfronts_shared_plan.md §2.1) has no
                         # lidar topic; RayFronts runs on RGB+depth only.
                         # Matches the current .env default too.
ZED_WIDTH="480"
ZED_HEIGHT="300"
DRONE_Z_M="1.0"         # unchanged from the current .env — this is the
                         # SPAWN height (must match SPAWN_HEIGHT_M below),
                         # not a search altitude; raven_nav's own
                         # min_altitude_agl/max_altitude_agl NODE PARAMS (not
                         # an env var — raven_single_rayfronts_shared_plan.md
                         # §2.1's declared-parameter list) govern cruise
                         # height once airborne
MAP_ANCHOR_ENU="true"
SPAWN_HEIGHT_M="1.0"
```

### Ordered commands

```bash
# apply the STAGE 2 block above to .env (NOT done by this pass)
airstack down
airstack up isaac-sim robot-desktop gcs
```

### What to look for

- Isaac viewport: two drones on open ground, ~20 m from a lying figure each,
  facing it (yaw baked into `SPAWN_CONFIGS`'s `orient`).
- `docker exec isaac-sim tmux capture-pane -p -J -t isaac -S -200 | tail -40`
  — should NOT show `pg.load_environment` silently loading an empty prim
  (`benchmark-disaster-dataset/SKILL.md`'s own warning: a frozen cell has no
  default prim, so the launcher must name `/World/stage` + siblings by `Sdf`,
  not call `pg.load_environment` on the whole file).
- `ros2 topic echo /robot_1/odometry --once` / `/robot_2/odometry --once` —
  both robots present, both `frame_id: map` (their OWN per-robot map, per
  `frames_scene_testenv.md` §A — not a shared world frame).
- If flying a search method (raven or a `search_baselines` arm) rather than
  just confirming spawn: `RESULTS_SCENE=RavenSuburbTornado250` names the
  annotation file `compare_to_groundtruth.py --scene RavenSuburbTornado250`
  reads by default.

### Final step — overwrite the auto-written annotations with the correct box

Once the scene has loaded (the `GT_ANNOTATIONS=on` auto-write from
`frozen_annotations.write_all` has already happened and used the wrong
upright `(0.7, 0.7, 1.8)` box — see the `GT_ANNOTATIONS` comment above), run
`people_json_to_annotations.py` (§1d's tool) against the frozen cell's own
`GT_people.json` (identical content to `PEOPLE_JSON`, written by Stage 1):

```bash
docker exec isaac-sim /isaac-sim/python.sh -c "
import sys; sys.path.insert(0, '/isaac-sim/AirStack/scene_gen')
import tools.people_json_to_annotations as pjta
pjta.main(['--people',
          '/isaac-sim/AirStack/_test_freeze/raven_suburb_tornado_250/GT_people.json',
          '--out',
          '/isaac-sim/AirStack/robot/ros_ws/src/global/planners/raven_nav/annotations/RavenSuburbTornado250.json'])
"
```

(or run the plain CLI form from the host, if the repo mount is the same path
on both sides — `python3 scene_gen/tools/people_json_to_annotations.py
--people .../GT_people.json --out robot/.../annotations/RavenSuburbTornado250.json`
— the tool has no Isaac/ROS dependency, `docker exec` is only needed if the
host cannot see the container's `FREEZE_OUT` path directly). Re-run this any
time the scene is reloaded, and definitely once more right before scoring
with `compare_to_groundtruth.py`.

---

## 4. GPU budget warning (plan §6.5 — read before Stage 1)

**One 16 GB card.** Isaac Sim alone currently measures **14.2 / 16.3 GiB**
used with a 250 m UNDAMAGED suburb loaded (`frames_scene_testenv.md` §C.2,
live `nvidia-smi` on this host, 2026-09-01) — before this tornado-damaged,
people-populated version (more geometry: plank field, scour relief, wreck
archetypes, casualties) is loaded, and before anything else runs. The shared
RayFronts server (WP-B) adds an estimated **3-4 GiB**, and a VLM server for
the LVLM-guided behaviour (WP-A) adds roughly **3 GiB at 3B nf4**. Loading
all three on this card is a real risk of an OOM that looks like an
unrelated hang or crash rather than a clean error. **This is a budget
decision for the user, not something to work around silently** — e.g. by
quietly disabling RayFronts or the VLM, which would invalidate the very
test WP-D exists to set up.

---

## 5. File list (this pass)

- `scene_gen/config/presets/suburb_tornado_250.yaml` (new)
- `scene_gen/tools/tornado_people_dry_run.py` (new — host casualty-plan +
  spawn-pick tool, promoted from the original session scratchpad prototype)
- `scene_gen/tests/test_tornado_people_dry_run.py` (new)
- `scene_gen/tools/people_json_to_annotations.py` (new)
- `scene_gen/tests/test_people_json_to_annotations.py` (new)
- `_plans/raven_test_scene_runbook.md` (this file)
- Previews: `~/raven_previews/suburb_tornado_250_track.png`,
  `~/raven_previews/suburb_tornado_250_people.json`,
  `~/raven_previews/suburb_tornado_250_people_overlay.png`,
  `~/raven_previews/RavenSuburbTornado250_annotations_preview.json`
