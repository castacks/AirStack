# AirStack mission specs

A **mission** is a declarative YAML file executed by
[`osmo/workspace/mission_runner.py`](../workspace/mission_runner.py). Each
mission runs one or more full iterations of:

```
airstack down → airstack up → wait for PX4 ready → start mcap recording
→ run steps → stop recording → collect bags + container logs → airstack down
```

Missions live in this directory so they're versioned with the code they
exercise — the OSMO pod clones your branch, so whatever you push is what runs.

Submit to OSMO with:

```bash
airstack osmo:mission osmo/missions/<mission>.yaml --pool <gpu-pool>
```

Or run locally on any machine that can `airstack up` (no OSMO involved):

```bash
python3 osmo/workspace/mission_runner.py osmo/missions/<mission>.yaml \
  --airstack-root "$(pwd)"
```

`--dry-run` validates the spec and prints the merged config without touching
Docker.

## Results

```
osmo/results/<mission-name>/<UTC stamp>/
├── summary.json              # per-iteration status + durations
└── iter_001/
    ├── bags/robot_1/*.mcap   # open directly in Foxglove (no conversion)
    ├── logs/<container>.log  # docker logs snapshot per container
    ├── logs/<robot container>/   # raw /tmp/rayfronts_*.log + /tmp/raven_*.log tees
    ├── ready.json            # per-robot seconds-to-PX4-ready
    ├── steps.json            # per-step command, output tail, pass/fail
    └── iteration.json        # iteration summary
```

On an OSMO pod the actual storage is `/osmo/output/airstack-mission-results`
(with `osmo/results` symlinked to it), so a workflow `outputs:` block uploads
everything automatically when the task exits. Download from your laptop at
any time while the pod is alive with `airstack osmo:fetch [dest]`.

Artifacts are collected **even when an iteration fails** — a failed flight's
bag is usually the most interesting one.

## Schema

Top-level keys (everything except `steps` is optional):

| Key | Default | Meaning |
|---|---|---|
| `name` | filename stem | Results directory name |
| `env` | `{}` | Env vars exported before each `airstack up` (`NUM_ROBOTS`, `COMPOSE_PROFILES`, `ISAAC_SIM_SCRIPT_NAME`, …) |
| `iterations` | `1` | Full up→fly→down cycles. With `environments:` + `environment_order: grouped`, this is **per environment** (total = iterations × #environments) |
| `iteration_attempts` | `1` | Max times to (re)run a single iteration until it passes. `>1`: a failed/errored iteration is redone (clean down→up→fly→down) up to this many attempts; a `passed`, manually `stopped`, or `abort_mission` outcome is never retried. Failed attempts' artifacts are preserved under `iter_NNN_failed_attempt_K`. With a fixed `SPAWN_SEED` the redo reproduces the same spawn layout |
| `environment_order` | `round_robin` | With `environments:`: `round_robin` (iteration i → env[(i-1) % n], `iterations` is the total) \| `grouped` (each env runs `iterations` times in a row) |
| `ready.timeout_s` | `600` | Max seconds to wait for PX4 readiness per iteration |
| `ready.poll_interval_s` | `5` | Seconds between readiness polls |
| `record.enabled` | `true` | Record an mcap per robot per iteration |
| `record.scope` | `gcs` | `gcs` (one mcap on GCS domain 0) \| `robot` (one mcap per robot domain) \| `both` |
| `record.topics` | tf + odom set | Topics to record; `{robot}` → `robot_N` |
| `record.all` | `false` | Record **all** topics (`ros2 bag record -a`) — large |
| `record.exclude` | — | With `all`: regex of topics to drop (`ros2 bag record -a --exclude-regex <regex>`) |
| `record.required` | `false` | Abort the iteration if any recorder fails to start (don't fly unrecorded) |
| `on_step_failure` | `abort_iteration` | `continue` \| `abort_iteration` \| `abort_mission` |
| `up_timeout_s` | `3600` | `airstack up` timeout (first up on a fresh pod pulls images) |
| `down_timeout_s` | `300` | `airstack down` timeout |
| `robot_setup_bash` | robot ws `setup.bash` | Workspace sourced before `ros2` commands |
| `nas_dest` | — | Base path on airlab-storage (e.g. `/volume3/<share>/airstack-missions`). When the `airlab-storage` OSMO credential is set (`airstack osmo:setup`), an OSMO pod rsyncs results to `<nas_dest>/<name>/<stamp>/` then tears itself down. Override per run with `osmo:mission --nas-dest PATH`; suppress with `--no-nas-upload`. Credentials never live in the spec |
| `steps` | — | Ordered list of steps (below) |

### Steps

Every step type accepts the placeholders `{robot}` → `robot_N` and `{n}` → `N`
in its strings, and runs once per robot in `robots:` (`all` by default, or a
list like `[1, 3]`).

**`action`** — send a goal to a robot task action server and wait for the
result. The step passes when the action result reports `success: true`.

```yaml
- action:
    task: takeoff                 # → /robot_N/tasks/<task>
    goal: {target_altitude_m: 10.0, velocity_m_s: 1.0}
    timeout_s: 120                # default 120 (per attempt)
    attempts: 3                   # per-robot retries on failure (default 3)
    retry_delay_s: 10             # wait between attempts (default 10)
    feedback_timeout_s: 15        # via gcs: no relay_feedback within this
                                  # window ⇒ goal presumed lost, retried
    pass_on_feedback: false       # via gcs: pass as soon as feedback is seen
                                  # (the task RAN), regardless of success/fail;
                                  # only "never ran" (no feedback + no result
                                  # after retries) fails the step
    robots: all
    # type: task_msgs/action/TakeoffTask   # derived from task name if omitted
```

Any step may also carry **`optional: true`** (a sibling key, not inside
`action`/`run`/…): the step still runs and its result is recorded, but a
failure neither trips `on_step_failure` nor counts toward an `iteration_attempts`
redo. Use it for steps whose outcome doesn't gate the iteration — e.g. a `land`
after the run is already done.

**`pass_on_feedback` + `iteration_attempts` idiom** — to guarantee a task
*runs* every iteration without caring whether it succeeds: set
`pass_on_feedback: true` on that action and `iteration_attempts: >1` on the
mission. If the task never gets feedback (goal lost / never started) after its
`attempts` retries, the step fails and the whole iteration is redone; once
feedback is seen the iteration is considered satisfied.

Each robot's goal is logged per attempt and retried independently — a goal
can be rejected transiently (relay has no GPS fix yet, PX4 position estimate
not converged, action server still starting), so one robot failing its first
attempt doesn't fail the step unless it exhausts all attempts.

Available tasks (action type is derived as `task_msgs/action/<CamelCase>Task`):
`takeoff`, `land`, `fixed_trajectory`, `navigate`, `exploration`, `coverage`,
`semantic_search`, `chat`. Goal fields are defined in
[`common/ros_packages/msgs/task_msgs/action/`](../../common/ros_packages/msgs/task_msgs/action/).
Multi-robot action goals are sent **in parallel** across robots.

**`wait`** — sleep for N seconds (e.g. hover, let a planner run):

```yaml
- wait: 30
```

**`run`** — arbitrary command; the escape hatch that makes any ROS 2 command
work without runner changes. The step fails on non-zero exit unless
`expect_success: false`. Set `attempts` to retry on failure (e.g. a flaky
model download) before the step is marked failed.

```yaml
- run:
    container: robot_1     # robot_N → exec in the robot container on robot
                           #   N's DDS domain (ros2 is sourced for you);
                           # pod → run on the pod itself (cwd = AirStack root);
                           # any other value → literal container name, domain 0
    cmd: ros2 topic echo --once /{robot}/odometry
    timeout_s: 60
    expect_success: true
    attempts: 1            # retries on failure (default 1 = no retry)
    retry_delay_s: 10      # wait between attempts (default 10)
```

**`topic_pub`** — `ros2 topic pub --once` per robot:

```yaml
- topic_pub:
    topic: /{robot}/some_input
    type: std_msgs/msg/Bool
    msg: {data: true}
```

**`service_call`** — `ros2 service call` per robot:

```yaml
- service_call:
    service: /{robot}/some_service
    type: std_srvs/srv/Trigger
    request: {}
```

## Method selection

Which planner `semantic_search_task` spawns is chosen by env vars in `env:`.
They are mutually exclusive and resolved in a fixed precedence order —
`FRONTIER_ONLY_BASELINE` > `VLFM_BASELINE` > `CONAVGPT_BASELINE` — with the
loser named in a warning, so setting two is a silent mis-run, not an error.

| Env var | Default | Method |
|---|---|---|
| *(none set)* | — | Full multi-raven coordination |
| `FRONTIER_ONLY_BASELINE` | `false` | Frontier-only exploration, no semantic cues |
| `VLFM_BASELINE` | `false` | Greedy semantic-ray (VLFM-style) navigation |
| `LVLM_BASELINE` | `false` | FPV + InternVL3-2B navigation (replaces rayfronts) |
| `CONAVGPT_BASELINE` | `false` | **Co-NavGPT "VLM-Assign"**: one VLM call per round assigns every robot a numbered frontier region; the assignment is gossiped, and a robot without a fresh one falls back to nearest-frontier |

`CONAVGPT_BASELINE` additionally spawns `conavgpt_assigner_node` (InternVL3-2B)
on the leader robot only (`conavgpt_leader_id`, default `robot_1`). Its debug
topics — `/{robot}/conavgpt/assign_request`, `/…/assignment`, `/…/map_image`,
`/…/round_table` — are robot-domain only (the GCS payload visualizer has no
handler for them), so record them via `record.scope: both` +
`record.robot_topics`, as `conavgpt_1robot.yaml` / `conavgpt_5robot.yaml` do.

Because `Stack.apply_env` only *merges* env vars, an omitted flag inherits
whatever the previous mission on that pod set. Pin all four explicitly rather
than relying on defaults.

## GPU sizing vs. fleet size

`semantic_search_task._pick_rayfronts_gpu()` pins robot N's rayfronts to
`GPU (N % n_gpus)`, and Isaac Sim is pinned to GPU 0. The overflow robot
therefore wraps onto the **sim's** card, where rayfronts OOMs loading its
encoder and that drone hovers for the whole mission. The pod needs
`gpu: NUM_ROBOTS + 1`:

| Robots | GPUs | Mapping | Workflow |
|---|---|---|---|
| 1–3 | 4 | sim→0, robot N→N | `osmo/workflows/airstack-mission.yaml` |
| 4 | 5 | sim→0, robot N→N | — (4-robot sweeps ran at `gpu: 4`, robot_4 sharing GPU 0) |
| 5 | 6 | sim→0, robot N→N | `osmo/workflows/airstack-mission-5robot.yaml` |
| 1 + a VLM server | 2 | sim→0, robot_1→1, VLM→1 | `osmo/workflows/airstack-mission-2gpu.yaml` |

`gpu: 5` does **not** fix a 5-robot run: `5 % 5 = 0` moves the collision to
robot_5 instead of removing it. Six is the first safe count.

Under `CONAVGPT_BASELINE` the assigner takes the highest GPU no rayfronts
claims (`_pick_conavgpt_gpu()`), so a 1-robot run on the stock 4-GPU workflow
puts it on GPU 3 alone. When the fleet claims every card — 5 robots on `gpu: 6`
— it shares the leader's card with robot_1's rayfronts and warns in the log.
`CONAVGPT_ASSIGNER_GPU` overrides the choice (`-1` = don't pin).

A mission that also runs a **conavgpt2 VLM server** (`search_baselines.vlm_server`,
started by a `run` step inside robot container 1) must keep it off Isaac Sim's
card too, and for a different reason than rayfronts: the sim does not OOM, its
RTX render graph fails every frame and stops publishing camera images
altogether. `CONAVGPT2_VLM_GPU` (mission `env:`, forwarded by
`robot/docker/robot-base-docker-compose.yaml`) is what the start step exports as
`CUDA_VISIBLE_DEVICES` for the server process; empty = don't pin, which is right
on a one-GPU box and wrong on a pod. `OPENAI_BASE_URL` reaches the node the same
way — `vlm_client.resolve()` reads it when the `vlm_base_url` parameter is empty,
which is its default. `osmo/missions/conavgpt2_wildfire_1robot.yaml` is the
worked example, including the `/health` gate that keeps the node's preflight from
racing the weight load, the `run` step that launches the planner AFTER that gate,
and the collection step that copies the round table and the server's per-request
metrics off the container before it is torn down.

`ZED_PITCH_DEG` travels the same path and is read by BOTH sides: the Isaac launch
script tilts the ZED by it, and `search_planner` uses it as the default for
`camera_pitch_rad`. That is deliberate — TF walks the URDF, the URDF models no
mount pitch, so a camera the sim tilted is one TF cannot see and a planner that
is not told unprojects every point to the wrong bearing without erroring. Set it
in one place or not at all. It is a CoNavGPT setting; every other method here
assumes the level mount, so leave it unset in their missions.

A mission whose fleet outgrows the default `resources:` block selects its own
workflow with `--workflow` (path is repo-relative or absolute):

```bash
airstack osmo:mission osmo/missions/conavgpt_5robot.yaml \
  --pool <gpu-pool> --branch <your-branch> \
  --workflow osmo/workflows/airstack-mission-5robot.yaml
```

## Notes

- For `NUM_ROBOTS > 1` on Isaac Sim, set
  `ISAAC_SIM_SCRIPT_NAME: example_multi_px4_pegasus_launch_script.py` in
  `env:` — the default script spawns a single drone. The scene-import script
  (`example_multi_drone_scene_import.py`) instead takes its fleet size from the
  length of `SPAWN_CONFIGS`, so a one-entry list is a valid 1-robot mission.
- `SPAWN_CONFIGS` layouts must keep every drone inside that start's spawn
  rectangle and all drones >= 3 m apart (`SPAWN_MIN_DIST_M`, the same gate the
  randomized `SPAWN_POLY` path enforces). Not every rectangle admits an
  arbitrary fleet size: `conavgpt_5robot.yaml` drops `fireacademy_s2` and
  `constructionsite_s2` because neither can hold 5 drones at 3 m without moving
  the source-of-truth spawns.
- Missions run unattended: keep `ISAAC_SIM_HEADLESS: "true"`,
  `ISAAC_SIM_USE_STANDALONE: "true"` and `PLAY_SIM_ON_START: "true"` unless
  you're watching via the WebRTC livestream profile.
- Recording camera/LiDAR topics (or `record.all: true`) is the bag-size
  driver — budget pod `storage:` accordingly.
- `/tf` and `/tf_static` are in the default topic set because without them a
  Foxglove 3D panel can't pose anything during replay.
