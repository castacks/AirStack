---
name: run-raven-single-shared
description: Run the single-agent RAVEN + shared off-board RayFronts arm (RAYFRONTS_MODE=shared) — one multi_robot_mapping_server on offboard-compute serving every robot's raven_nav instead of one rayfronts.launch.xml per robot. Covers the exact `airstack up` sequence and env block, the two-robot suburb-tornado test scene and its GPU budget, what to watch (`scripts/raven_monitor.sh`, the status topic, the debug tables), the test harness that must be green first (`scripts/raven_rayfronts_tests.sh`), and — because none of this has been flown — an explicit list of what is UNVERIFIED.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Run RAVEN single-agent + shared off-board RayFronts

## When to use

- Testing `RAYFRONTS_MODE=shared` — one off-board `multi_robot_mapping_server`
  (on `offboard-compute`) serving every robot's `raven_nav`, instead of each
  robot spawning its own `rayfronts.launch.xml`.
- Running the two-drone `RavenSuburbTornado250` smoke test
  (`osmo/missions/raven_single_shared_test.yaml`).
- Debugging why a shared-mode robot never gets a navigator (the status-topic
  wait timing out) or why the shared mapping server never anchors a robot.

This is the runbook half of `_plans/raven_single_rayfronts_shared_plan.md`
§5 (WP-C). It documents CONTRACTS from that plan and code that exists in this
tree — it does **not** report a real flight, because none has happened yet
(see "What is UNVERIFIED" below, and read it before trusting anything else
in this file operationally).

**Do not launch Isaac Sim, edit `.env`'s live values, or start/stop any
compose service from an agent session without the user's explicit
go-ahead.** Everything below is written for a human (or an agent with that
clearance) to execute; an agent reading this skill to answer a question
should not act on §2/§3 itself.

---

## 0. The two things this build changes vs. today

1. `semantic_search_task` (env `RAYFRONTS_MODE=shared`, default `per_robot`
   = today's behaviour, byte-for-byte): instead of spawning
   `rayfronts.launch.xml` and counting `ms/batch` lines from its stdout, it
   subscribes to `/robot_{ROS_DOMAIN_ID}/rayfronts/status` (a `std_msgs/
   String` JSON status the shared mapping server publishes at 1 Hz) and
   waits until that robot reports `anchored: true` and
   `frames_robot >= 8` — same timeout (`RAYFRONTS_WAIT_TIMEOUT_S`, default
   180 s) and abort semantics as the per-robot wait. It never
   spawns or kills `rayfronts.mapping_server` in this mode — that process is
   shared and owned by `offboard-compute`, not this robot's mission.
2. `offboard-compute` (env `START_RAYFRONTS_SERVER=true`, default `false`):
   starts `rayfronts.encoder_server` (the GPU-resident encoder) then
   `rayfronts.multi_robot_mapping_server` (one mapper, round-robining every
   robot's RGB/depth/pose, each on its own `ROS_DOMAIN_ID`), publishing each
   robot's map outputs back onto that robot's OWN domain under its existing
   topic names — `raven_nav` does not know the map is shared.

Everything raven_nav itself does (Voxel > Ray > LVLM-guided > Frontier,
single-agent, no coordination) is WP-A's rewrite — see that work's own
README under `robot/ros_ws/src/global/planners/raven_nav/`, not this file.

---

## 1. Prerequisite: prove the test harness is green

Before touching Isaac Sim at all, run the ONE test command for this whole
build:

```bash
scripts/raven_rayfronts_tests.sh          # host pure tests + a throwaway
                                            # container run; never touches
                                            # the running isaac-sim container
scripts/raven_rayfronts_tests.sh --gpu    # add the rayfronts `-m cuda` tests
                                            # too (briefly claims a GPU —
                                            # check `nvidia-smi` headroom
                                            # first, see §4)
```

It prints a PASS/FAIL/SKIP table (host tier: `raven_nav`, `rayfronts`,
`semantic_search_task`, `scene_gen`'s people-JSON test; container tier: the
same three run inside a throwaway `docker run --rm` of the robot image,
under real ROS + a compiled `rayfronts_cpp`) and exits non-zero on any real
failure. Logs land under `/tmp/raven_rayfronts_tests_<timestamp>/`
(override with `RAVEN_TEST_LOG_DIR`).

**As of this writing this is NOT fully green** — see "What is UNVERIFIED"
below for the exact failing suites. A SKIP row (a test dir another
work-package hasn't landed yet, or no `cuda`-marked test existing yet) is
expected and fine; a FAIL row is not, and this build should not be flown
until the FAIL rows clear (or are individually understood and accepted).

---

## 2. `.env` block (commented template already in `.env`)

`.env` carries a commented-out `RAVEN single-agent + shared off-board
RayFronts` block (search for that heading) with every variable below and a
one-line reason for each. Uncomment what you need; **do not flip anything
else in `.env`**.

| var | where | value for this test | why |
|---|---|---|---|
| `RAYFRONTS_MODE` | robot containers | `shared` | selects the status-topic wait over the spawn+stdout wait |
| `RAVEN_LVLM` | robot containers | unset (default `true`) | raven's LVLM-guided behaviour; set `false` to drop to Voxel>Ray>Frontier only if the VLM server is what blows the GPU budget |
| `RAVEN_LVLM_RAY_THRESHOLD` | robot containers | unset (default `0.9`) | LVLM-guided tier's own ray-score gate (`lvlm_ray_threshold`) — see §6's tuning section |
| `RAVEN_LVLM_INTERVAL_S` | robot containers | unset (default `30.0`) | LVLM-guided tier's request throttle (`lvlm_request_interval_s`) — see §6's tuning section |
| `VLM_URL` | robot containers + offboard-compute | `http://offboard-compute:8000/v1` | endpoint raven's `lvlm_client` and Co-NavGPT2's `vlm_client` both already default toward |
| `START_RAYFRONTS_SERVER` | offboard-compute | `true` | starts `encoder_server` + `multi_robot_mapping_server` there |
| `START_VLM_SERVER` | offboard-compute | `true` | needed for raven's LVLM-guided tier to have anything to call |
| `RAYFRONTS_ROBOT_IDS` | offboard-compute | unset | `offboard_compute.sh` defaults it to `1..NUM_ROBOTS` |
| `RAYFRONTS_CONFIG` | offboard-compute | unset (default `shared_humans`) | WP-B's human-tuned config, `common/rayfronts_configs/shared_humans.yaml` |
| `RAYFRONTS_SRC` | offboard-compute | unset (default `/root/AirStack/common/rayfronts`) | resolves because `offboard-compute`'s own compose entry mounts `common/rayfronts` there (see §5) |
| `NUM_ROBOTS` | both | `2` | this test's robot count |

Equivalently, `osmo/missions/raven_single_shared_test.yaml` (§3 below) sets
all of these itself via its `env:` block — no `.env` edits needed to run it
through `mission_runner.py`.

---

## 3. Running it

### 3a. As a mission (preferred — one command, repeatable)

```bash
python3 osmo/workspace/mission_runner.py \
    osmo/missions/raven_single_shared_test.yaml --airstack-root "$(pwd)"
```

This brings up `isaac-sim robot-desktop gcs offboard-compute`, takes off both
robots, and sends one `semantic_search` goal (`query: person`, background
vocabulary from `common/rayfronts_configs/background_humans.txt`, search
area = the 250 m plate ±125 m, `voxel_min_cluster_size: 4`). See the mission
file's own header comment for the full parameter list and why each value was
picked (several are explicitly flagged there as guesses — never flown).

**Hard prerequisite the mission file cannot satisfy itself**: `FROZEN_SCENE`
(`/isaac-sim/AirStack/_test_freeze/raven_suburb_tornado_250`) must already
exist. That is `_plans/raven_test_scene_runbook.md` §2 (Isaac Sim, ONE
launch, `suburb_tornado_launch_script.py`, build + freeze) — as of this
writing that launch has explicitly **not** been run (its own header: "NOT
run by this pass"). Run it first (with the user's go-ahead), then §2 step 5
of that runbook re-validates the two chosen casualties against the real
frozen file before flying.

### 3b. Manually (more visibility into each stage)

```bash
# 1. The shared server + VLM, alone first (so model-load latency doesn't
#    eat into the mission's own timeouts):
START_RAYFRONTS_SERVER=true START_VLM_SERVER=true \
RAYFRONTS_COMPUTE_PROB=True \
CONAVGPT2_VLM_MODEL=Qwen/Qwen2.5-VL-3B-Instruct CONAVGPT2_VLM_QUANT=nf4 \
airstack up offboard-compute

# 2. Watch it come up (see §4) — wait for the encoder_server socket and the
#    mapping server's first "ms/batch"-style progress before proceeding.

# 3. Then the robots + GCS, already pointed at the shared server and the
#    frozen scene (see the mission file's `env:` block for the full set —
#    FROZEN_SCENE, RESULTS_SCENE, SPAWN_CONFIGS, NUM_ROBOTS=2, RAYFRONTS_MODE
#    =shared, VLM_URL, ...):
RAYFRONTS_MODE=shared VLM_URL=http://offboard-compute:8000/v1 \
NUM_ROBOTS=2 airstack up isaac-sim robot-desktop gcs

# 4. Send the goal — via GCS's mission UI (action_relay), or directly:
#    ros2 action send_goal /robot_1/semantic_search_task/semantic_search_task \
#      task_msgs/action/SemanticSearchTask "{query: person, background_queries: '...', ...}"
```

---

## 4. What to watch

```bash
# The shared server itself (both halves — encoder_server logs the socket
# bind, multi_robot_mapping_server logs per-frame/batch progress):
docker exec offboard-compute tail -F /tmp/offboard/rayfronts_encoder.log
docker exec offboard-compute tail -F /tmp/offboard/rayfronts_mapping.log

# Per-robot: is THIS robot anchored and receiving frames yet? (the exact
# JSON semantic_search_task's shared-mode wait loop polls)
docker exec airstack-robot-desktop-1 bash -c \
  "source /opt/ros/jazzy/setup.bash; ros2 topic echo /robot_1/rayfronts/status"

# raven's own detection/debug tables (std_msgs/String, human-readable):
docker exec airstack-robot-desktop-1 bash -c \
  "source /opt/ros/jazzy/setup.bash; ros2 topic echo /robot_1/debug/voxel_table"
docker exec airstack-robot-desktop-1 bash -c \
  "source /opt/ros/jazzy/setup.bash; ros2 topic echo /robot_1/raven_nav/lvlm_request"

# All of the above, both robots, merged into one file — this is
# scripts/raven_monitor.sh:
scripts/raven_monitor.sh 2
```

`raven_monitor.sh` tails `offboard-compute`'s two logs, each robot's own
unfiltered raven stdout tee (`/tmp/raven_robot_N.log` inside that robot's
container — `semantic_search_task/node.py`'s `_spawn(log_name='raven')`;
this file only exists once a `SemanticSearchTask` goal has actually started
raven on that robot), and the six status/detection topics above, all
prefixed and merged into `~/raven_previews/monitor_<timestamp>.log`. It only
reads from already-running containers (`docker exec`) — it never starts,
stops, or touches any compose service. Ctrl-C stops it cleanly.

---

## 5. GPU budget

**One 16 GB card.** Isaac Sim alone measures 14.2 GiB with just the 250 m
UNDAMAGED suburb loaded (`frames_scene_testenv.md` §C.2, live `nvidia-smi`,
2026-09-01) — before the tornado-damaged, people-populated version (more
geometry) is loaded, and before either of the two things this build adds:

- the shared RayFronts encoder (RADSeg via `encoder_server`) — estimated
  3-4 GiB (WP-B's own estimate; not measured against a real GPU by this
  work).
- the VLM server for raven's LVLM-guided tier — roughly 3 GiB at 3B/nf4
  (`CONAVGPT2_VLM_MODEL=Qwen/Qwen2.5-VL-3B-Instruct`, the LOCAL setting
  `.env` already uses for Co-NavGPT2 on this same card; do **not** flip to
  7B here except on an OSMO-sized 2x48 GiB pod).

Loading all three on one card is a real OOM risk that can look like an
unrelated hang rather than a clean error — this is a budget decision for the
user, not something to silently work around (e.g. dropping `RAVEN_LVLM` to
`false` or skipping the shared server) without saying so.

---

## 6. What is UNVERIFIED (read before trusting a green light above)

This build has never been flown against Isaac Sim. Everything in this list
is either a documented gap, a guessed parameter, or a test-tier result that
is real but does not by itself prove the shared-mode RUNTIME path works.

- **The scene doesn't exist yet.** `_plans/raven_test_scene_runbook.md` §2's
  Isaac launch (build + freeze `RavenSuburbTornado250`) has not been run.
  Nothing past that point in this file has been exercised end to end.
- **The shared mapping server has never processed real camera frames.**
  `scripts/raven_rayfronts_tests.sh`'s container tier proves `import
  rayfronts` + `import rayfronts_cpp` + `rclpy` + `torch` all work together
  in the robot image and that `pytest` runs against the live-mounted
  source — it does NOT start `multi_robot_mapping_server` itself or feed it
  synthetic frames end-to-end (that's `common/rayfronts/tests/
  test_multi_ros_dataset.py`'s job, and see the next point).
- **Current test-tier state (re-run `scripts/raven_rayfronts_tests.sh` for
  the live numbers — this build is still landing across three concurrent
  work packages and these WILL change):**
  - `raven_nav` — 2 known failures (`test_ray_targets.py`'s one pre-existing
    assertion, explicitly not WP-C's to fix; a `test/integration/
    test_node_roundtrip.py` case) plus, in the container tier specifically,
    9 errors across the rest of that same integration-test file
    (`rclpy.exceptions.NotInitializedException: rclpy.init() has not been
    called` — looks like that test file's own `rclpy.Context()`/`rclpy.init
    (context=ctx, ...)` pattern not matching how `RavenNavNode.__init__`
    actually acquires its context; WP-A's file, WP-A's to fix).
  - `common/rayfronts/tests` — real failures in `test_client_encoder.py`
    (the `ClientEncoder` <-> `encoder_server` IPC roundtrip: `EOFError` /
    connection errors reading the server's response — looks like the
    encoder_server subprocess the test fixture spawns is not answering, not
    an environment problem given `import rayfronts`/`torch`/`rclpy` all
    otherwise work in the same container) and a timing assertion in
    `test_multi_ros_dataset.py::test_round_robin_interleaves_the_robots`.
    Both are WP-B's files and, per the build plan's own framing, WP-B's to
    land — not investigated further here.
  - The host tier (`common/rayfronts/tests -m "not ros and not cuda and not
    torch"`) is green — but that filter deliberately excludes exactly the
    ROS/IPC-heavy tests above.
  - No `cuda`-marked rayfronts test exists yet, so `--gpu` currently only
    proves the container can claim a GPU and `torch.cuda.is_available()` is
    True inside it — not that anything CUDA-specific in this build works.
- **`/robot_N/rayfronts/status` is not bridged to GCS.** Neither
  `dds_router.yaml` nor the gossip payload list
  (`gossip_payloads.yaml`) has an entry for it (grep-verified against both
  — this is a brand-new topic this build introduces) — the only way to see
  it today is a direct `docker exec` into that robot's own container
  (§4, and what `raven_monitor.sh` does). If GCS-side visualization of the
  shared server's health is wanted, that is a follow-up change to one of
  those two config files (not owned by this work package).
- **Guessed goal parameters, to tune from the first real run — three
  softmax/timing gates, all currently best-guess, not validated defaults:**
  - `voxel_score_threshold: 0.5` and `score_threshold: 0.5` (the mission's
    goal, both explicit) — a human is a much smaller, rarer-shaped target
    than the building-scale objects the existing scene templates tune these
    against.
  - `RAVEN_LVLM_RAY_THRESHOLD` (env, default 0.9 — `raven_nav/params.py`'s
    `lvlm_ray_threshold`, OG `lvlm_behavior.py:78`) and
    `RAVEN_LVLM_INTERVAL_S` (env, default 30.0 — `lvlm_request_interval_s`,
    OG `LVLM/internvl3.py:35`) — the LVLM-guided tier's own ray-score gate
    and request throttle; both pass through `robot-base-docker-compose.yaml`
    as bare-name env, same as `RAVEN_LVLM` (unset = that node-side default).
  - Tune all three together from the first run's `debug/voxel_table` /
    `rays_sim/q*_person` peaks — they gate on the SAME softmax-over-the-
    query-set scores (`RAYFRONTS_COMPUTE_PROB=True`), just at different
    tiers of the Voxel > Ray > LVLM-guided > Frontier priority.
  - The mission's `min_altitude_agl: 2.5` / `max_altitude_agl: 12.0` (also
    explicit, lower than every OTHER mission in this directory's 5.0/20.0 —
    a lying casualty's standoff is near ground level) is the other knob
    worth watching if the drone hovers too high to see a lying figure.
- **The two spawn points are a HOST APPROXIMATION**
  (`_plans/raven_test_scene_runbook.md`'s own caveat: `deck_points`, only
  measurable off the real baked wreck geometry, can change which candidate
  casualties the planner accepts) — that runbook's §2 step 5 re-validates
  them against the real frozen file's `humans_10.json` before flying; do not
  skip that step.
- **The `robot-base-docker-compose.yaml` `RAYFRONTS_MODE`/`RAVEN_LVLM`
  bare-name passthrough and the `docker-compose.yaml` offboard-compute env +
  volume additions are verified only via `docker compose config`** (static
  YAML rendering — confirmed the merged config contains exactly the right
  keys/mounts, `robot-desktop` unaffected) — not via an actual `airstack up`
  of this arm, which needs the frozen scene above to be useful anyway.
- **`offboard_compute.sh`'s rayfronts branch is proven by direct
  `docker run` calls with FAKE `python3`/`nvidia-smi` shims** (verified: the
  correct `encoder_server`/`multi_robot_mapping_server` argv is constructed
  in every branch — default paths, the missing-mount fallback, an explicit
  `RAYFRONTS_ROBOT_IDS` override, a custom `RAYFRONTS_CONFIG` — and the
  encoder-socket wait loop correctly starts the mapping server once the
  socket appears) — never against the REAL `rayfronts.encoder_server`/
  `multi_robot_mapping_server` Python processes, which is exactly the gap
  the mission/manual run in §3 closes.

---

## 7. File list (this work package)

- `robot/ros_ws/src/global/planners/semantic_search_task/semantic_search_task/node.py`
  (`RAYFRONTS_MODE` shared/per_robot dispatch, `_filter_raven` regex gains
  `LVLM-guided`)
- `robot/ros_ws/src/global/planners/semantic_search_task/test/test_shared_mode.py` (new)
- `robot/ros_ws/src/global/planners/search_baselines/scripts/offboard_compute.sh`
  (the `START_RAYFRONTS_SERVER` branch)
- `robot/docker/docker-compose.yaml` (offboard-compute: env + the two new
  volume mounts)
- `robot/docker/robot-base-docker-compose.yaml` (`RAYFRONTS_MODE`,
  `RAVEN_LVLM` bare-name passthrough)
- `.env` (commented template block only)
- `scripts/raven_rayfronts_tests.sh` (new)
- `scripts/raven_monitor.sh` (new)
- `osmo/missions/raven_single_shared_test.yaml` (new)
- this file
