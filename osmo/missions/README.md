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
| `iterations` | `1` | Number of full up→fly→down cycles |
| `ready.timeout_s` | `600` | Max seconds to wait for PX4 readiness per iteration |
| `ready.poll_interval_s` | `5` | Seconds between readiness polls |
| `record.enabled` | `true` | Record an mcap per robot per iteration |
| `record.scope` | `gcs` | `gcs` (one mcap on GCS domain 0) \| `robot` (one mcap per robot domain) \| `both` |
| `record.topics` | tf + odom set | Topics to record; `{robot}` → `robot_N` |
| `record.all` | `false` | Record **all** topics (`ros2 bag record -a`) — large |
| `on_step_failure` | `abort_iteration` | `continue` \| `abort_iteration` \| `abort_mission` |
| `up_timeout_s` | `3600` | `airstack up` timeout (first up on a fresh pod pulls images) |
| `down_timeout_s` | `300` | `airstack down` timeout |
| `robot_setup_bash` | robot ws `setup.bash` | Workspace sourced before `ros2` commands |
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
    robots: all
    # type: task_msgs/action/TakeoffTask   # derived from task name if omitted
```

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
`expect_success: false`.

```yaml
- run:
    container: robot_1     # robot_N → exec in the robot container on robot
                           #   N's DDS domain (ros2 is sourced for you);
                           # pod → run on the pod itself (cwd = AirStack root);
                           # any other value → literal container name, domain 0
    cmd: ros2 topic echo --once /{robot}/odometry
    timeout_s: 60
    expect_success: true
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

## Notes

- For `NUM_ROBOTS > 1` on Isaac Sim, set
  `ISAAC_SIM_SCRIPT_NAME: example_multi_px4_pegasus_launch_script.py` in
  `env:` — the default script spawns a single drone.
- Missions run unattended: keep `ISAAC_SIM_HEADLESS: "true"`,
  `ISAAC_SIM_USE_STANDALONE: "true"` and `PLAY_SIM_ON_START: "true"` unless
  you're watching via the WebRTC livestream profile.
- Recording camera/LiDAR topics (or `record.all: true`) is the bag-size
  driver — budget pod `storage:` accordingly.
- `/tf` and `/tf_static` are in the default topic set because without them a
  Foxglove 3D panel can't pose anything during replay.
