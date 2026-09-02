# RAVEN live test — execution checklist

`suburb_tornado_250` → freeze → fly 1 drone → fly 2 drones, shared off-board
RayFronts, `person` search, bags on the final working runs.

Everything in `scripts/raven_live/` was produced offline. **Nothing has been
run against Isaac Sim, the robot containers, or `.env`.** The `isaac-sim`
container was up (5 h) at authoring time and was not touched.

Every command below is copy-pasteable **from the repo root**
(`/home/krrishjain/SEI-COA/disaster-dataset`) and every container interaction
is `docker exec <c> bash -c "..."`, never `-it`.

---

## 0. Before you touch anything

```bash
cd /home/krrishjain/SEI-COA/disaster-dataset

# free VRAM right now — this whole run lives or dies on this number
nvidia-smi --query-gpu=index,name,memory.used,memory.total,utilization.gpu --format=csv

# what is up, and how old
docker ps --format '{{.Names}}\t{{.Status}}\t{{.CreatedAt}}'

# .env fingerprint — every edit_env.py call below quotes it back as a guard
scripts/raven_live/edit_env.py --print-md5
```

**GPU budget, stated once.** One 16 GB card. Isaac alone measured **14.2 GiB**
with the *undamaged* 250 m suburb loaded. This cell is heavier (plank field,
scour relief, wreck archetypes, casualties), and the run adds the RayFronts
encoder (~3–4 GiB) and the VLM (~3 GiB at 3B/nf4). **This does not fit
comfortably.** See §9 for the fallback ladder. Do not silently drop RayFronts
or the VLM to make it fit — that invalidates the test.

**Concurrency.** Other sessions edit and commit this repo. `edit_env.py`
refuses to write when `.env` moved under it; if it refuses, re-read and redo
the decision rather than forcing.

**`_test_freeze/` is NOT in `.gitignore`** (checked). Stage 1 writes a
~400 MB `.usd` into the working tree. Before anyone runs `git add -A`:

```bash
printf '\n# RAVEN live-test freeze output (hundreds of MB of USD)\n_test_freeze/\n' >> .gitignore
```

---

## 1. Stage 1 — build + freeze the cell

Uses the **already-running** `isaac-sim` container via its tmux pane. No
`airstack up/down`, no recreate, no `.env` edit.

> **Why the pane and not `.env`.** The isaac-sim compose service declares
> *none* of `TOR_SEED`, `TOR_PEOPLE`, `TOR_PLANKS`, `TOR_TRACK_PER100`,
> `TOR_GROUND`, `TOR_MIN_TIPPED`, `PEOPLE_JSON`, `PEOPLE_SNAPS`,
> `SCOUR_RELIEF`, `FREEZE_OUT`, `FREEZE_NAME`, `FREEZE_EXPORT` or
> `FREEZE_EXIT`. Compose forwards nothing it does not name, so the Stage-1
> `.env` block in `_plans/raven_test_scene_runbook.md` §2 would be **silently
> dropped at the container boundary** — the launcher would build at its own
> `TOR_SEED=11`, write the people JSON next to the archetypes, and never
> freeze anything, all under a happy banner.

```bash
# 1a. read it first — this prints the exact launch line and touches nothing
scripts/raven_live/stage1_freeze.sh

# 1b. IF NOBODY IS LOOKING AT THE CURRENT SCENE, send it.
#     (This Ctrl-Cs the pane, kills orphaned PX4, clears history, relaunches.)
scripts/raven_live/stage1_freeze.sh --go

# 1c. watch it — polls both the piped pane log and the Kit log
scripts/raven_live/watch_stage1.sh --follow
```

`watch_stage1.sh` exits **0** when the `[tornado] FREEZE_EXIT set` banner is
present with no failure line before it, **1** on failure (it prints the
evidence), **2** while it is still running.

Expect ~2–4 min of silence after `app ready` — that is the RtPso shader
compile (133 s cold / 14 s warm, measured), not a hang. Confirm it is alive:

```bash
docker exec isaac-sim bash -c 'pgrep -af launch_script | head -2; nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader'
```

**Milestones, in order:** `[compile_disaster] compiled high-level spec …
suburb_tornado_250` → `[tornado] track 78 m wide toward 38 deg …` →
`[tornado] plank field: …` → `[tornado] people: ~19 authored …` →
`TORNADO TRACK ASSEMBLED 250 x 250 m` → `[freeze] flattened in …` →
`FROZEN OK` → `[tornado] FREEZE_EXIT set`.

**VRAM checkpoint:** during the build, `nvidia-smi` should show Isaac climbing
toward 14–15 GiB. If it pins at the card limit and the pane goes quiet, that
is the OOM — Stage 1 alone does not fit either, and nothing downstream will.

**If the export fails.** `suburb_tornado_launch_script.py` calls
`freeze.export_scene(collect=False)` with the default
`enforce_portable=True` and — unlike `freeze_dataset_launch_script.py` — reads
**no** `FREEZE_WAIVE_VEGETATION` / `FREEZE_WAIVE_MIRRORED` knob. So the
portability gate is **unwaivable from this launcher**: if it fires, nothing
ships. `_test_freeze/raven_suburb_tornado_250/freeze_report.json` holds the
full `verify()` result (the launcher writes `exc.info` there deliberately) —
read `build_local` and `unresolved` in it before doing anything else.

---

## 2. Stage 1 gate — validate the cell and re-pick the casualties

```bash
scripts/raven_live/validate_freeze.sh
```

Half **A** asserts the cell on disk: `GT_hints.json`, `GT_people.json`,
`freeze_report.json`, and **exactly one top-level `.usd`** — that last one
matters because `frozen_annotations.resolve_cell()` falls back to
"the single `.usd` in this directory" for a path that is not shaped
`<Disaster>/<Locale>/level_<n>/<k>`, and two files make it raise at launch.
It also prints the portability verdict.

Half **B** re-picks the two casualties against the **real** frozen people file
and writes `scripts/raven_live/out/spawns_{1,2}robot.json` +
`spawn_summary.txt`.

| exit | meaning |
|---|---|
| 0 | cell good, both casualties where the runbook says — nothing to change |
| 2 | cell good, but a casualty **moved** and a substitution was made. The `SPAWN_CONFIGS` in `_plans/raven_test_scene_runbook.md` §3 and in `osmo/missions/raven_single_shared_test.yaml` are now **stale**. `stage2_env_*.sh` and the new mission files read the generated JSON, so the rest of this checklist is unchanged — but read the flags. |
| 1 | the cell is not flyable. Fix Stage 1. |

Then look at the pictures (Stage 1 wrote them with `SNAP_DIR`/`PEOPLE_SNAPS`):

```bash
ls -l ~/docker/isaac-sim/logs/raven_t250/
# overview.png first, then p00_* — p00 is the WORST-covered casualty in the
# scene, so a run that reads acceptable at p00 needs no further pictures.
```

---

## 3. Stage 2 env — one drone first

```bash
# read the rationale + the exact command
scripts/raven_live/stage2_env_1robot.sh

# apply it (or paste the command it printed)
scripts/raven_live/stage2_env_1robot.sh --cmd | bash
```

Every key and every reason is in that script's own output. The three that
would otherwise bite:

* **`ZED_PITCH_DEG=0`** — `.env` has **30** today and
  `osmo/missions/raven_single_shared_test.yaml` does not override it. That
  tilts the camera *prim*; the URDF models no mount pitch, so TF cannot see
  it, and RayFronts takes its pose straight off
  `/{robot}/odometry_conversion/odometry` (`src_coord_system: flu`) assuming
  the camera is aligned with the body. Left at 30, every voxel unprojects 30°
  off and the map is quietly, systematically wrong.
* **`PLAY_SIM_ON_START=true`** — the compose entry has **no** `:-` default, so
  an unset value renders empty, `play_on_start` is False, `/clock` never
  ticks, and takeoff times out looking like a PX4 fault.
* **`ENABLE_LIDAR=false`** — the compose entry is `${ENABLE_LIDAR:-}` on
  purpose: *empty means the launcher's own default*, which for this launcher
  is **true**. It must be the literal string `false`.

Snapshots land in `scripts/raven_live/env_snapshots/`. To put `.env` back at
any point:

```bash
scripts/raven_live/edit_env.py --restore scripts/raven_live/env_snapshots/env_<stamp>.bak
```

---

## 4. Bring-up order

**Order matters:** the shared server loads models eagerly at container start,
so it goes up **alone and first** — its load time must not eat into the
robots' `RAYFRONTS_WAIT_TIMEOUT_S` (180 s) or the mission's own step timeouts.

### 4a. Prerequisite, once per machine — warm the RADSeg weight cache

Verified 2026-09-02: `robot/docker/cache` on this host has **no** `torch/hub`
directory and no RADIO/siglip2 entries. The shared server's `encoder=radseg`
will try to download `NVlabs/RADIO c-radio_v3-b` plus the `siglip2`/`sam`
adaptors on first start — slow at best, a crash before the socket binds at
worst. **Do this before anything else, with network:**

```bash
docker run --rm -v "$(pwd)/robot/docker/cache:/root/.cache:rw" \
  $(docker compose -f robot/docker/docker-compose.yaml config --format json \
    | python3 -c "import json,sys; print(json.load(sys.stdin)['services']['robot-desktop']['image'])") \
  python3 -c "import torch; torch.hub.load('NVlabs/RADIO', 'radio_model', \
version='c-radio_v3-b', skip_validation=True, adaptor_names=['siglip2','sam'])"

# confirm
ls robot/docker/cache/torch/hub 2>/dev/null || echo "STILL EMPTY — the encoder will download at start"
```

### 4b. Clear the decks

The isaac-sim container is currently running Stage 1's launcher (or its
leftovers). The frozen cell is on disk, so nothing is lost by taking it down.

```bash
./airstack.sh down                       # `stop` leaves containers behind and
                                         # the next `up` silently no-ops
docker ps -a --format '{{.Names}}'       # must be empty of airstack/isaac-sim
```

### 4c. The shared server, alone

```bash
./airstack.sh up offboard-compute
```

Verify **both halves** before going further:

```bash
# the encoder binds a unix socket, THEN the mapping server starts
docker exec offboard-compute bash -c 'tail -n 60 /tmp/offboard/rayfronts_encoder.log'
docker exec offboard-compute bash -c 'ls -l /tmp/rayfronts/encoder.sock'
docker exec offboard-compute bash -c 'tail -n 60 /tmp/offboard/rayfronts_mapping.log'
docker exec offboard-compute bash -c 'tail -n 40 /tmp/offboard/vlm_server.log'

# what it thinks it was told
docker exec offboard-compute bash -c 'env | grep -E "RAYFRONTS|START_|VLM|RESULTS_SCENE" | sort'
```

Expected: `[offboard-compute] rayfronts: src=… config=shared_humans
config_dir=… robot_ids=[1] compute_prob=True sock=/tmp/rayfronts/encoder.sock`,
then the socket appears, then the mapping server starts. `offboard_compute.sh`
waits **300 s** for that socket and starts the mapper anyway if it never
appears — so a missing socket is a warning in the log, not a hard stop. Check
for it explicitly.

**VRAM checkpoint 1** — with *only* offboard-compute up:

```bash
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
```

Encoder ≈ 3–4 GiB, VLM ≈ 3 GiB. **If the two together already exceed ~7.5 GiB
you will not fit Isaac.** Go to §9 now, not after Isaac OOMs.

### 4d. Isaac + robots + GCS

Two variants — use the first if you did §4b, the second if for some reason
isaac-sim is still up and you do not want to recreate it.

```bash
# VARIANT A (normal, after `airstack down`)
./airstack.sh up isaac-sim robot-desktop gcs

# VARIANT B (isaac-sim already running: relaunch its pane instead of
# recreating the container — keeps runtime pip installs, ~2 min not ~10)
docker exec isaac-sim tmux pipe-pane -t isaac -o 'cat >> /isaac-sim/.nvidia-omniverse/logs/raven_stage2.log'
docker exec isaac-sim tmux send-keys -t isaac C-c
sleep 30
docker exec isaac-sim bash -c 'pkill -9 -f px4_sitl_default/bin/px4; sleep 2'
docker exec isaac-sim tmux clear-history -t isaac
docker exec isaac-sim tmux send-keys -t isaac 'clear; SCENE_CONFIG= FROZEN_SCENE=/isaac-sim/AirStack/_test_freeze/raven_suburb_tornado_250 RESULTS_SCENE=RavenSuburbTornado250 NUM_ROBOTS=1 SPAWN_CONFIGS='"'"'<paste from scripts/raven_live/out/spawns_1robot.json>'"'"' SPAWN_HEIGHT_M=1.0 ENABLE_LIDAR=false ZED_WIDTH=480 ZED_HEIGHT=300 ZED_PITCH_DEG=0 GT_ANNOTATIONS=on PLAY_SIM_ON_START=true ISAAC_SIM_HEADLESS=false PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/example_multi_drone_scene_import.py --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --/rtx/raytracing/fractionalCutoutOpacity=true --/rtx/pathtracing/fractionalCutoutOpacity=true' ENTER
# then still `./airstack.sh up robot-desktop gcs` for the rest of the stack
```

Variant B exists because a pane relaunch is much faster than a recreate — but
**Variant A is the one to use**, because the robot containers must be created
*after* `.env` is right anyway, and mixing the two makes it hard to say what
the containers were actually given.

Verify per service:

```bash
# Isaac: did it load the FROZEN cell and spawn the right fleet?
docker exec isaac-sim tmux capture-pane -p -J -t isaac -S -400 | grep -aE '\[scene\]|\[spawn\]|Traceback' | tail -20
#   expect: [scene] FROZEN cell /isaac-sim/AirStack/_test_freeze/.../RavenSuburbTornado250.usd
#           [spawn] DRONE_CONFIGS=[{"domain_id": 1, "x_m": ..., ...}]
#   MUST NOT show pg.load_environment silently loading an empty prim.

# the robots
docker ps --format '{{.Names}}' | grep robot-desktop
docker exec airstack-robot-desktop-1 bash -c 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=1; timeout 20 ros2 topic echo --once --field connected /robot_1/interface/mavros/state'
docker exec airstack-robot-desktop-1 bash -c 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=1; timeout 20 ros2 topic echo --once /robot_1/interface/mavros/local_position/odom | head -20'
#   `connected: True` alone fires ~25 s too early — local_position/odom is the
#   real precondition for arming (this is exactly what mission_runner's
#   wait_ready gates on).

# the shared mapper has actually anchored this robot
docker exec airstack-robot-desktop-1 bash -c 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=1; timeout 20 ros2 topic echo --once /robot_1/rayfronts/status'
#   want: anchored: true, frames_robot climbing. semantic_search_task's
#   shared-mode wait polls this and needs anchored + frames_robot >= 8.

# GCS
docker ps --format '{{.Names}}' | grep -i gcs
```

**VRAM checkpoint 2** — everything up, before any goal:

```bash
nvidia-smi --query-gpu=memory.used,memory.total,utilization.gpu --format=csv
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
```

If `memory.used` is within ~500 MiB of `memory.total` here, **stop and go to
§9** — the first frames the mapper encodes will push it over, and the failure
will look like a hang, not an OOM.

**If a drone never reaches PX4 ready:** check for orphaned PX4 first. A
leftover process holds the instance dir and the MAVLink ports, the new `px4`
goes `<defunct>`, and MAVROS happily connects to the *orphan* and prints a
healthy FCU version — so `mavros/state` looks fine while
`local_position/odom` never arrives.

```bash
docker exec isaac-sim bash -c 'pgrep -af px4_sitl_default/bin/px4'
```

---

## 5. Fly it — the goal

### 5a. Preferred for the recorded runs: the mission file

Fill the spawn placeholder first, then run:

```bash
# 1 robot
python3 - <<'PY'
import json, pathlib, re
s = json.load(open("scripts/raven_live/out/spawns_1robot.json"))["SPAWN_CONFIGS"]
p = pathlib.Path("osmo/missions/raven_shared_t250_1robot.yaml")
t = p.read_text()
assert "@@FILL_FROM validate output@@" in t, "already filled — edit by hand"
p.write_text(t.replace("'@@FILL_FROM validate output@@'", "'" + s + "'"))
print("filled:", s)
PY

python3 osmo/workspace/mission_runner.py \
    osmo/missions/raven_shared_t250_1robot.yaml --airstack-root "$(pwd)"
```

(The 2-robot run is the same with `spawns_2robot.json` and
`raven_shared_t250_2robot.yaml`.) Validate a file before running it:

```bash
python3 osmo/workspace/mission_runner.py osmo/missions/raven_shared_t250_1robot.yaml \
    --airstack-root "$(pwd)" --dry-run
```

**Know what the runner does to your stack.** `run_iteration` begins with
`ensure_down()` — it **tears the whole stack down first, including a running
isaac-sim** — brings it up itself from `stack.services`, and `down()`s it at
the end. It never edits `.env`: it copies `os.environ`, merges the mission's
`env:` on top, and hands that to `./airstack.sh up` as the subprocess
environment, so mission values beat `.env` for that run only. **So §3's `.env`
edits are for the manual path in §5b; the mission file carries its own copy of
the same values.** Keep them in agreement — they are, as shipped.

**Bags.** `record.scope: both` starts a GCS recorder on domain 0 plus one per
robot on its own domain (for `rayfronts/status`, `debug/voxel_table`,
`raven_nav/lvlm_request` — grep-verified as bridged by neither
`dds_router.yaml` nor `gossip_payloads.yaml`). They write to `/osmo_bags`
(host `osmo/bags/`) and are **moved** before teardown to:

```
osmo/results/<mission-name>_<stamp>/iter_001/bags/
```

`record.required: true` is set in both files, so a recorder that fails to
start aborts the iteration rather than flying unrecorded.

### 5b. Manual, for the first exploratory pass

**ORDER IS MANDATORY: send the takeoff goal FIRST and wait for
`Goal finished with status: SUCCEEDED` before sending semantic_search.**
The search goal cancels active navigate goals at start and immediately
begins the rayfronts wait + raven spawn; sent mid-climb it can interrupt
the takeoff task, and raven starts planning from a drone that is not yet
at altitude. The mission files already sequence takeoff -> wait ->
semantic_search; this rule is for the manual path.

Send the goal directly on the robot's own domain — no GCS relay, so one fewer
moving part while you are still finding out whether this works at all:

```bash
docker exec airstack-robot-desktop-1 bash -c 'source /opt/ros/jazzy/setup.bash && { [ -f /root/AirStack/robot/ros_ws/install/setup.bash ] && source /root/AirStack/robot/ros_ws/install/setup.bash; }; export ROS_DOMAIN_ID=1; ros2 action send_goal --feedback /robot_1/tasks/semantic_search task_msgs/action/SemanticSearchTask '"'"'{"query": "person", "background_queries": "road, asphalt, driveway, sidewalk, grass, dirt, mud, gravel, tree, fallen tree, tree branch, bush, house, roof, brick wall, wooden fence, car, pickup truck, wood planks, lumber, rubble, debris pile, tarp, sky, cloud, utility pole, power line, trash can, mailbox, swimming pool, concrete slab, garage door, window", "search_area": {"points": [{"x": 125.0, "y": 125.0, "z": 0.0}, {"x": 125.0, "y": -125.0, "z": 0.0}, {"x": -125.0, "y": -125.0, "z": 0.0}, {"x": -125.0, "y": 125.0, "z": 0.0}]}, "min_altitude_agl": 2.5, "max_altitude_agl": 12.0, "min_flight_speed": 0.5, "max_flight_speed": 1.0, "confidence_threshold": 0.95, "voxel_score_threshold": 0.5, "score_threshold": 0.5, "voxel_min_cluster_size": 4}'"'"''
```

Takeoff first, the same way:

```bash
docker exec airstack-robot-desktop-1 bash -c 'source /opt/ros/jazzy/setup.bash && { [ -f /root/AirStack/robot/ros_ws/install/setup.bash ] && source /root/AirStack/robot/ros_ws/install/setup.bash; }; export ROS_DOMAIN_ID=1; ros2 action send_goal --feedback /robot_1/tasks/takeoff task_msgs/action/TakeoffTask '"'"'{"target_altitude_m": 2.0, "velocity_m_s": 0.5}'"'"''
```

For robot_2, change **both** `robot_1` → `robot_2` and `ROS_DOMAIN_ID=1` → `2`.

---

## 6. Monitoring

```bash
# everything, merged, into ~/raven_previews/monitor_<stamp>.log
scripts/raven_monitor.sh 1        # or 2

# the shared server's two halves
docker exec offboard-compute bash -c 'tail -F /tmp/offboard/rayfronts_encoder.log'
docker exec offboard-compute bash -c 'tail -F /tmp/offboard/rayfronts_mapping.log'

# is THIS robot anchored and receiving frames? (the exact JSON the
# shared-mode wait loop polls)
docker exec airstack-robot-desktop-1 bash -c 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=1; ros2 topic echo /robot_1/rayfronts/status'

# raven's own tables
docker exec airstack-robot-desktop-1 bash -c 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=1; ros2 topic echo /robot_1/debug/voxel_table'
docker exec airstack-robot-desktop-1 bash -c 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=1; ros2 topic echo /robot_1/debug/ray_table'

# raven's unfiltered stdout tee — only exists once a SemanticSearchTask goal
# has actually started raven on that robot
docker exec airstack-robot-desktop-1 bash -c 'tail -F /tmp/raven_robot_1.log'

# VRAM, every few minutes
watch -n 20 'nvidia-smi --query-gpu=memory.used,memory.total,utilization.gpu --format=csv'
```

`/robot_N/rayfronts/status` is **not** bridged to GCS (no `dds_router.yaml`
entry, no gossip payload) — a `docker exec` into that robot's own container is
the only way to see it.

### What a detection looks like

1. `/robot_N/debug/voxel_table` starts listing `person` rows with a score
   above `voxel_score_threshold` (0.5).
2. `/robot_N/raven_nav/discoveries` gets an entry, then
   `/robot_N/raven_nav/confirmed_targets` once it clears
   `confidence_threshold`.
3. `[event] CONFIRMED …` appears in `/tmp/raven_robot_N.log`.
4. The results JSON lands in the shared cache mount:

```bash
docker exec airstack-robot-desktop-1 bash -c 'ls -l /root/.cache/raven_results/'
docker exec airstack-robot-desktop-1 bash -c 'cat /root/.cache/raven_results/compiled_results.json' | head -40
```

Nothing at step 1 after several minutes of flight means the map is the
problem, not the planner: check `rayfronts/status` `frames_robot`, then the
mapping log, then `ZED_PITCH_DEG` (§3).

---

## 7. Ground truth — do this AFTER the scene has loaded

`GT_ANNOTATIONS=on` auto-writes `raven_nav/annotations/RavenSuburbTornado250.json`
at scene-load time using `frozen_annotations.people_boxes`' **upright**
`(0.7, 0.7, 1.8)` box centred at `z + 0.9`. That is right for a standing
survivor and **wrong for a lying tornado casualty**, and it **clobbers the
file every time the scene loads**. Overwrite it correctly once the scene is
up, and again right before scoring:

```bash
python3 scene_gen/tools/people_json_to_annotations.py \
    --people _test_freeze/raven_suburb_tornado_250/GT_people.json \
    --out robot/ros_ws/src/global/planners/raven_nav/annotations/RavenSuburbTornado250.json
```

That file's records carry `"class": "person"` with a **lying** box
(e.g. `size_xyz_m: [1.19, 1.896, 0.4]`), which is the whole reason the upright
auto-write has to be replaced.

Then score. **`--class-filter` is required here**: it defaults to `'house'`,
so the default invocation would filter out every `person` GT box and score
zero:

```bash
docker exec airstack-robot-desktop-1 bash -c \
  'python3 -m raven_nav.compare_to_groundtruth \
     --compiled /root/.cache/raven_results/compiled_results.json \
     --scene RavenSuburbTornado250 \
     --class-filter person \
     --center-dist 3.0'
```

`--center-dist` defaults to 10 m, which is generous for a 1.9 m lying figure —
3 m is a fairer first number. The report lands next to `compiled_results.json`
as `groundtruth_comparison.json` unless you pass `--out`.

---

## 8. Teardown between runs

```bash
# 1. robots + GCS + the shared server
./airstack.sh down robot-desktop gcs offboard-compute

# 2. isaac-sim: stop the launcher in the pane and kill the PX4 orphans.
#    (Only `down` REMOVES the container; `stop` leaves it and the next `up`
#    silently no-ops.)
docker exec isaac-sim tmux send-keys -t isaac C-c
sleep 30
docker exec isaac-sim bash -c 'pgrep -af px4_sitl_default/bin/px4; pkill -9 -f px4_sitl_default/bin/px4; sleep 2'
docker exec isaac-sim tmux clear-history -t isaac

# 3. confirm nothing is holding the GPU before the next bring-up
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
```

If you used `mission_runner.py`, it does its own `ensure_down()` /`down()` —
but it does **not** kill PX4 orphans in isaac-sim, so step 2 is still yours
between runs.

Between the 1-robot and 2-robot run, also:

```bash
scripts/raven_live/stage2_env_2robot.sh          # read it
scripts/raven_live/stage2_env_2robot.sh --cmd | bash
```

---

## 9. OOM fallback ladder

Take these **in order**, one at a time, and say which rung you are on when
reporting the result — a run at rung 2 is not the test the plan asked for.

| # | change | costs you |
|---|---|---|
| 1 | **Drop the VLM.** `START_VLM_SERVER=false` **and** `RAVEN_LVLM=false`. Frees ~3 GiB. | raven falls back to Voxel > Ray > Frontier. The LVLM-guided tier is untested this run. |
| 2 | 1 robot instead of 2. | the shared mapper's round-robin is untested. |
| 3 | `ISAAC_SIM_HEADLESS=true` (no GUI render target). | you cannot watch it; use the overhead camera and the bags. |
| 4 | Lower `mapping.max_pts_per_frame` (4000 → 1500) and raise `dataset.frame_skip` in `common/rayfronts_configs/shared_humans.yaml`. | small-target recall — the exact thing this scene tests. Last resort. |

```bash
# rung 1
scripts/raven_live/edit_env.py --set START_VLM_SERVER=false --set RAVEN_LVLM=false
```

**Never** set `RAVEN_LVLM=""` — it is a bare-name compose passthrough, so an
empty string is *not* "unset" and beats the node's own `true` default. Use the
literal `false`.

**Do not touch `FASTDDS_BUILTIN_TRANSPORTS`.** `common/fastdds.xml` is what
actually pins the transport; changing that env var once took the readiness
gate from 7/8 to 0/8.

---

## 10. Reverting

```bash
ls -lt scripts/raven_live/env_snapshots/
scripts/raven_live/edit_env.py --restore scripts/raven_live/env_snapshots/env_<stamp>.bak
```

The first snapshot written tonight is the pre-test `.env`. Nothing in
`scripts/raven_live/` is committed, and neither is `_test_freeze/`.
