# Testing (`tests/`)

AirStack's **pytest** tree under `tests/` has three roles:

1. **`tests/system/`** — Docker stack tests (sim + robot + GCS): liveliness, sensor Hz, takeoff/hover/land, image/workspace builds.
2. **`tests/robot/`** — Fast **unit** tests that mirror `robot/ros_ws/src/` (`behavior`, `global`, `interface`, `local`, `perception`, `sensors`). Mark: `unit`.
3. **`tests/sim/`** — Unit tests for simulation-side helpers (e.g. Motive / NatNet emulator). Mark: `unit`.

Shared fixtures live in `tests/conftest.py`. Use `airstack test -m unit -v` for hermetic tests only, or the marks below for the full stack.

<iframe width="1120" height="630" src="https://www.youtube.com/embed/EzgGHnYDI_k?si=vpqER-TXud5XEMUX" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

---

## Test Suite Structure

### System tests (`tests/system/`)

| Module | Mark | What it tests | Hardware required |
|--------|------|---------------|-------------------|
| [`system/test_build_docker.py`](system/test_build_docker.py) | `build_docker` | Docker image builds (robot-desktop, gcs, isaac-sim, ms-airsim); records image sizes | Docker daemon |
| [`system/test_build_packages.py`](system/test_build_packages.py) | `build_packages` | `colcon build` inside each container (robot, GCS, ms-airsim ROS workspace) | Docker daemon |
| [`system/test_liveliness.py`](system/test_liveliness.py) | `liveliness` | Stack bring-up: container Running state, ``/clock`` readiness, tmux panes, sentinel ROS 2 nodes, compute snapshot, infra-only ``test_stable`` (tmux + nodes + compute) | Docker daemon, GPU, sim license |
| [`system/test_sensors.py`](system/test_sensors.py) | `sensors` | After liveliness in collection order: sim + robot stereo/depth Hz (**Isaac:** batched ``ros2 topic hz`` to avoid bridge overload; **ms-airsim:** single batch), filtered LiDAR via ``echo --once`` + cloud sanity (isaacsim), sim RTF, ``test_sensor_streams_stable`` | Docker daemon, GPU, sim license |
| [`system/test_takeoff_hover_land.py`](system/test_takeoff_hover_land.py) | `takeoff_hover_land` | End-to-end flight: PX4 readiness gate, takeoff to 10 m, hover stability, land — one chain per (sim, num_robots, iteration, velocity) | Docker daemon, GPU, sim license |
| [`system/test_fixed_trajectory.py`](system/test_fixed_trajectory.py) | `autonomy` | Fixed-pattern trajectory evaluation: takeoff, execute a trajectory (Circle, Figure8, Racetrack, Line), record path deviation metrics, land — one chain per (sim, num_robots, iteration, trajectory_type) | Docker daemon, GPU, sim license |

### Unit tests (`tests/robot/`, `tests/sim/`)

Hermetic tests use `@pytest.mark.unit` (see [`pytest.ini`](pytest.ini)).

**Co-location + proxy pattern:** test source lives alongside its ROS 2 package at
`robot/ros_ws/src/<layer>/<package>/test/test_*.py` (the ROS 2 / colcon convention).
Files in `tests/robot/` are thin proxies that re-export those tests so that
`pytest tests/` discovers them. Both `airstack test -m unit` and
`colcon test --packages-select <pkg>` run the same test source.

Example: `robot/ros_ws/src/sensors/lidar_point_cloud_filter/test/test_validation_core.py`
tests the numpy-only range validation rules in
`robot/ros_ws/src/sensors/lidar_point_cloud_filter/lidar_point_cloud_filter/validation_core.py`
(also used by `scripts/validate_lidar_filter_clouds.py` inside the robot container).

See [Unit Testing Guide](../docs/development/intermediate/testing/unit_testing.md)
and the `add-unit-tests` agent skill for full details.

Marks can be combined with pytest logic:
`-m unit`, `-m "build_docker or build_packages"`, `-m liveliness`, `-m sensors`, `-m takeoff_hover_land`, `-m autonomy`, or e.g. `-m "liveliness or sensors"` (see **Bring-up scope** below).

### Bring-up scope (`airstack_env`)

`airstack_env` is **class-scoped** and parametrized per `(sim, num_robots, iteration)`. Each test **class** that uses it (`TestLiveliness`, `TestSensors`, `TestTakeoffHoverLand`, `TestFixedTrajectory`, …) performs its **own** ``airstack up`` / ``airstack down`` for that parametrization. Selecting both classes (for example, ``-m "liveliness or sensors"``) runs **two** full stack cycles per tuple (liveliness class, then sensors class). Collection order (see ``conftest.py``) runs **liveliness before sensors** when both are selected. To save wall time, run ``-m liveliness`` or ``-m sensors`` alone when one suite is enough.

---

## Test Infrastructure

All shared fixtures, helpers, and configuration live in [`conftest.py`](conftest.py).

### `airstack_env` fixture

Parametrized over `(sim, num_robots, iteration)` tuples derived from CLI flags. For each combination it:

1. Calls `airstack up` with the appropriate `COMPOSE_PROFILES`, `NUM_ROBOTS`, and headless flags
2. Records `airstack_up_duration_s` to `metrics.json`
3. Yields an `env` dict used by liveliness and sensor tests
4. Tears down with `airstack down` and records `airstack_down_duration_s`

### Isaac Sim and the `sensors` mark

**LiDAR in pytest:** [`conftest.py`](conftest.py) sets
`ENABLE_LIDAR=true` in `SIM_CONFIG["isaacsim"]["extra_env"]` so the multi-drone
Pegasus script (`example_multi_px4_pegasus_launch_script.py`) attaches RTX LiDAR
the same way the single-drone script always does. Without that flag the multi
script would not spawn LiDAR OmniGraphs.

**Topic checks** live in [`sensor_probes.py`](sensor_probes.py)
and are driven by [`system/test_sensors.py`](system/test_sensors.py):

| Path | What we measure | How |
|------|-----------------|-----|
| Sim → `/clock`, stereo images, stereo depth | Publish rate | ``ros2 topic hz`` on the sim container: ``/clock`` alone, then **chunks of two** ``image_rect`` topics, then **chunks of two** depth topics (``ISAACSIM_HZ_CHUNK_SIZE`` in ``sensor_probes.py``). |
| Robot → same topic names (bridge) | Publish rate | Same **two-at-a-time** chunking on the robot container for Isaac. ms-airsim: one batch of four topics. |
| Robot → filtered ``.../ouster/point_cloud`` | Stream alive | ``ros2 topic echo --once`` per robot (not Hz — large ``PointCloud2``). |
| LiDAR geometry | Near-range vs ``near_range_m`` | ``robot/ros_ws/src/sensors/lidar_point_cloud_filter/scripts/validate_lidar_filter_clouds.py`` (raw vs filtered). |

Sim **RTF** (real-time factor from ``/clock``) is also in the `sensors` suite.
**`test_sensor_streams_stable`** repeats sim + robot stereo + LiDAR probes every
`--stable-interval` for `--stable-duration` and records time-series to
`metrics.json` (stereo/depth as ``*.hz_samples``; LiDAR echo-once as ``*.received_samples``).

### `MetricsRecorder`

Writes custom metrics to `tests/results/<timestamp>/metrics.json` after each `record()` call. Keys follow the pattern `test_node_id → metric_key → {value, unit, direction}`. Time-series data (Hz samples, compute snapshots) are stored as `{key}_samples` lists and expanded into scalar aggregates (mean, min, max, start_mean, end_mean) by `parse_metrics.py`.

### Output files

Every test run produces a timestamped directory containing only `summary.txt`,
`results.xml`, and `metrics.json` — there is **no** `logs/` subdirectory and no
per-test log files are written under the run directory.

```
tests/results/
└── 2025-04-21_14-30-00/
    ├── summary.txt        # Human-readable key metrics — open this first
    ├── results.xml        # JUnit XML — test durations and pass/fail status
    └── metrics.json       # Custom metrics (image sizes, Hz, compute, timing)
```

Live test output goes to the terminal (pytest `log_cli`). On failure, assertion
messages include the tail of the last subprocess output (the in-memory
`read_log_tail` of the relevant `docker` / `ros2` subprocess) — no per-test log
files are written under the run directory.

---

## Running Tests

### `airstack test` (primary interface)

`airstack test` is the standard way to run tests. It builds the containerized
test runner from `tests/docker/`, mounts the repo read-only, and forwards all
arguments directly to pytest. No local Python environment needed.

```bash
# From the repo root (AirStack must be set up: airstack setup):

# Unit tests only — no GPU, no full Docker stack (numpy-only + pure Python)
airstack test -m unit -v

# Build tests only — fast, no GPU needed
airstack test -m "build_docker or build_packages" -v

# Liveliness run — ms-airsim, 1 robot, 1 iteration, 60 s stability window
airstack test -m liveliness \
  --sim msairsim \
  --num-robots 1 \
  --stress-iterations 1 \
  --stable-duration 60 \
  -v

# Takeoff/hover/land run — three velocities
airstack test -m takeoff_hover_land \
  --sim msairsim \
  --num-robots 1 \
  --stress-iterations 1 \
  --takeoff-velocities 0.5,1,2 \
  -v

# Sensor topic rates + LiDAR
airstack test -m sensors \
  --sim isaacsim \
  --num-robots 1 \
  --stress-iterations 1 \
  --stable-duration 60 \
  -v

# Show GUI windows (for local visual inspection)
airstack test -m liveliness --gui -v
```

`airstack test` calls `xhost +` automatically so GUI-mode sim containers
can reach the host X server; it is a no-op when `DISPLAY` is not set.

### Prerequisites

- Docker daemon running with your user in the `docker` group
- NVIDIA drivers + `nvidia-container-toolkit` for liveliness, sensors, takeoff_hover_land, and autonomy tests
- `airstack setup` completed (adds `airstack` to `PATH`)

### Direct pytest (for development / debugging)

Run pytest directly when you need faster iteration (no container rebuild) or
want to attach a debugger. Requires a local Python environment.

```bash
export AIRSTACK_ROOT=$(pwd)
pip install -r tests/requirements.txt

# Build tests only
pytest tests/ -m "build_docker or build_packages" -v

# Liveliness run
pytest tests/ -m liveliness \
  --sim msairsim \
  --num-robots 1 \
  --stress-iterations 1 \
  --stable-duration 60 \
  -v

# Sensor streams (after liveliness in default collection order)
pytest tests/ -m sensors \
  --sim isaacsim \
  --num-robots 1 \
  --stress-iterations 1 \
  -v
```

### CLI option reference

| Option | Default | Description |
|--------|---------|-------------|
| `--sim` | `msairsim,isaacsim` | Comma-separated sim targets |
| `--num-robots` | `1,3` | Comma-separated robot counts |
| `--stress-iterations` | `3` | Up/down cycles per (sim, num_robots) config |
| `--stable-duration` | `120` | Seconds ``test_stable`` / ``test_sensor_streams_stable`` poll for |
| `--stable-interval` | `10` | Seconds between polls in those stability tests |
| `--gui` | off | Show simulator GUI (disables headless mode) |
| `--takeoff-velocities` | `0.5,1,2` | Takeoff/land speeds in m/s |

---

## Autonomy Tests (`system/test_takeoff_hover_land.py`)

`TestTakeoffHoverLand` runs a **4-phase flight chain** for every combination of
`(sim, num_robots, iteration, velocity)`. The drone returns to the ground after
each velocity so the next velocity starts from a clean state.

### Phase order

| Phase | Test | What happens |
| ----- | ---- | ------------ |
| 1 | `test_px4_ready` | Waits for MAVROS + PX4 EKF ready; once per env |
| 2 | `test_takeoff` | Sends TakeoffTask; asserts altitude within 10 % |
| 3 | `test_hover` | Captures odom for 10 s; asserts altitude drift < 0.5 m |
| 4 | `test_landing` | Sends LandTask; asserts final altitude < 0.5 m |

If any phase other than `test_hover` fails, the remaining phases for that env
are skipped (the chain guard prevents a stuck-in-air drone from blocking later
velocity sweeps). A hover failure does **not** skip landing, so the drone always
returns to the ground.

### Recorded metrics

| Metric key | Unit | Description |
| ---------- | ---- | ----------- |
| `ready_duration_sys_s` | s | Wall-clock time from test start until PX4 ready |
| `takeoff_duration_sim_s` | s | Sim-time from first motion to 95 % of target |
| `land_duration_sim_s` | s | Sim time from 80 % peak descent to < 0.5 m |
| `velocity_rmse_m_sim_s` | m/s | RMSE of dz/dt vs commanded velocity during climb/descent |
| `altitude_error_m` | m | Signed steady-state error at takeoff success (+ = high) |
| `overshoot_m` | m | Unsigned transient overshoot above target |
| `hover_altitude_mean_error_m` | m | Mean altitude drift during hover |
| `hover_position_stddev_m` | m | 3-D position jitter (sqrt of summed axis variances) |
| `final_altitude_m` | m | Altitude at landing action completion |
| `odometry_error_mean_m` | m | Mean 3-D position error vs ground-truth odom |
| `odometry_error_max_m` | m | Peak 3-D error vs ground-truth odom |
| `odometry_altitude_bias_m` | m | Signed z-axis bias vs ground-truth odom |

Metrics are recorded per robot as `robot_N.<key>` and written to
`tests/results/<timestamp>/metrics.json`.

### Running takeoff_hover_land tests

```bash
# Sweep velocities 0.5, 1, 2 m/s; 1 robot; ms-airsim
airstack test -m takeoff_hover_land \
  --sim msairsim \
  --num-robots 1 \
  --stress-iterations 1 \
  --takeoff-velocities 0.5,1,2 \
  -v

# Single velocity, Isaac Sim, 3 robots
airstack test -m takeoff_hover_land \
  --sim isaacsim \
  --num-robots 3 \
  --stress-iterations 1 \
  --takeoff-velocities 1 \
  -v
```

---

## Fixed Trajectory Tests (`system/test_fixed_trajectory.py`)

!!! note "Detailed guide"
    For the full end-to-end testing guide — architecture, the fixed-trajectory benchmark, metrics, CLI reference, comparing trackers, and baselines — see **[End-to-End Testing](../docs/development/intermediate/testing/end_to_end_testing.md)**.

`TestFixedTrajectory` runs a **4-phase flight chain** for every combination of
`(sim, num_robots, iteration, trajectory_type)`. For each trajectory type the drone
takes off, executes the pattern, then lands — regardless of whether the trajectory
phase passes or fails (a trajectory failure does not skip landing).

Supported trajectory types: `Circle`, `Figure8`, `Racetrack`, `Line` (same patterns as
the `fixed_trajectory_task` ROS 2 action server in `trajectory_controller`).

### Phase order

| Phase | Test | What happens |
| ----- | ---- | ------------ |
| 1 | `test_px4_ready` | Waits for MAVROS + PX4 EKF ready; once per env |
| 2 | `test_takeoff` | Takeoff to 10 m at 1 m/s; asserts altitude within 10 % |
| 3 | `test_fixed_trajectory` | Sends `FixedTrajectoryTask`; captures odom; asserts cross-track error |
| 4 | `test_landing` | Sends `LandTask`; asserts final altitude < 0.5 m |

A failure in `test_fixed_trajectory` does **not** poison the chain — `test_landing` always
runs so the drone returns to the ground before the next trajectory type starts.

### Recorded metrics

| Metric key | Unit | Description |
| ---------- | ---- | ----------- |
| `ready_duration_sys_s` | s | Wall-clock time from test start until PX4 ready |
| `takeoff_duration_sim_s` | s | Sim-time from first motion to 95 % of target altitude |
| `altitude_error_m` | m | Signed steady-state altitude error after takeoff |
| `overshoot_m` | m | Unsigned transient overshoot above target |
| `trajectory_success` | — | 1.0 if action returned `success: true`, 0.0 otherwise (`higher_is_better`) |
| `trajectory_execution_time_sim_s` | s | Sim-time elapsed from action dispatch to completion |
| `cross_track_error_mean_m` | m | Mean 2-D lateral distance from nearest ideal-path point |
| `cross_track_error_max_m` | m | Worst-case lateral deviation |
| `path_rmse_m` | m | 2-D RMSE against the ideal path |
| `final_altitude_m` | m | Altitude at landing action completion |
| `land_duration_sim_s` | s | Sim-time from 80 % peak descent to < 0.5 m |


Metrics reported in one .txt file called summary.txt which automatically populates once your run completes 

### Default trajectory parameters

| Type | Parameters |
| ---- | ---------- |
| Circle | radius=10 m, velocity=2 m/s |
| Figure8 | length=15 m, width=8 m, height=0 m, velocity=2 m/s, max_acceleration=1 m/s² |
| Racetrack | length=30 m, width=10 m, height=0 m, velocity=3 m/s, turn_velocity=1.5 m/s, max_acceleration=1 m/s² |
| Line | length=20 m, height=0 m, velocity=2 m/s, max_acceleration=1 m/s² |

### Running fixed trajectory tests

```bash
# All four trajectory types; ms-airsim; 1 robot
airstack test -m autonomy \
  --sim msairsim \
  --num-robots 1 \
  --stress-iterations 1 \
  --trajectory-types Circle,Figure8,Racetrack,Line \
  -v

# Circle only (quick check of the known failure case)
airstack test -m autonomy \
  --sim msairsim \
  --num-robots 1 \
  --stress-iterations 1 \
  --trajectory-types Circle \
  -v
```

### CLI option reference (trajectory-specific)

| Option | Default | Description |
|--------|---------|-------------|
| `--trajectory-types` | `Circle,Figure8,Racetrack,Line` | Comma-separated trajectory types to sweep |

---

## Metrics Reporting (`parse_metrics.py`)

[`parse_metrics.py`](parse_metrics.py) reads `results.xml` and `metrics.json` from a run directory and produces a markdown report. It has two modes:

### Single-run report

```bash
python tests/parse_metrics.py \
  --current tests/results/2025-04-21_14-30-00/
```

Prints a markdown table of all recorded metrics. Always exits 0.

### Diff / regression check

```bash
python tests/parse_metrics.py \
  --current  tests/results/2025-04-21_14-30-00/ \
  --baseline tests/results/2025-04-20_09-00-00/ \
  --threshold 20          # optional: regression if change% exceeds this (default 20)
  --output   report.md    # optional: also write to file
```

Prints a side-by-side comparison. Exits **1** if any metric regresses beyond the threshold; exits 0 otherwise.

The report has three sections per test module:

- **Metrics** — flat table of scalar metrics (test name, metric key, value/baseline, change%)
- **Sim publishing rates** — pivot table of topic Hz aggregates from the `sensors` mark (`mean`, `start_mean`, `end_mean`, `min`, `max`; sim + robot topics)
- **Compute usage** — pivot table of CPU/memory/GPU metrics per container

Regressions are flagged with :red_circle:, improvements with :green_circle:.

---

## CI/CD Integration

### Workflow: `system-tests.yml`

[`.github/workflows/system-tests.yml`](../../../../.github/workflows/system-tests.yml) runs on:

- **Pull requests** to `main` or `develop` — automatically runs `build_docker or build_packages` tests (no GPU-intensive liveliness run on every PR)
- **Manual dispatch** (`workflow_dispatch`) — fully configurable for liveliness runs and metric comparisons

#### Manual dispatch inputs

| Input | Default | Description |
|-------|---------|-------------|
| `marks` | `liveliness` | pytest marks expression |
| `sim` | `msairsim` | Sim targets |
| `num_robots` | `1` | Robot counts |
| `stress_iterations` | `1` | Iterations per config |
| `stable_duration` | `120` | Stability polling seconds |
| `baseline_run_id` | _(blank)_ | Run ID for comparison; blank = latest `main` run |

#### Jobs

**`run-tests`** runs on a freshly-spawned ephemeral OSMO pod (`[self-hosted, airstack-ephemeral]`). The pod is submitted per-job by the orchestrator described below and destroyed once the job completes. It installs dependencies, runs pytest, and uploads `tests/results/` as an artifact named `test-results-<sha>-<run_id>` with 90-day retention.

**`report`** runs on `ubuntu-latest` after `run-tests` (even if it failed). It:

1. Downloads the current artifact
2. Downloads a baseline artifact (from the base branch for PRs, from `main` for manual runs, or from the specified `baseline_run_id`)
3. Runs `parse_metrics.py` in diff mode if a baseline is found, otherwise in single-run mode
4. Posts the markdown report as a PR comment (PR runs) or to the job summary (all runs)
5. Fails with `::error::` if `parse_metrics.py` exits 1 (regression detected)

#### Required third-party action

The workflow uses [`dawidd6/action-download-artifact@v6`](https://github.com/dawidd6/action-download-artifact) to download artifacts from other workflow runs by branch name. This is a community action and must be trusted in your repository's Actions settings if you use a restricted allowed-actions policy.

---

## CI/CD Orchestrator (OSMO-backed ephemeral runners)

AirStack's tests require a GPU, Docker, and a clean filesystem per run, so they execute on **truly ephemeral [NVIDIA OSMO](https://nvidia.github.io/OSMO/) pods** submitted per-job by an orchestrator. Each test job gets a fresh GPU pod that is destroyed once the job completes — no Docker layer carryover, no leaked containers, no shared host state. (This replaced an OpenStack-Nova backend; the GitHub side and the per-job-destroy model are unchanged.)

### Architecture

```
┌──────────────────────────────────────────────────────────────┐
│  Orchestrator VM  (airstack-ci-cd-orchestrator)              │
│   • polls GitHub for queued workflow_jobs                    │
│   • mints single-use JIT runner tokens                       │
│   • submits / reaps ephemeral OSMO workflows via osmo CLI    │
│   • holds the GitHub PAT and OSMO service-account token      │
└────────────┬───────────────────────────────────┬─────────────┘
             │                                   │
             ▼                                   ▼
┌──────────────────────────────┐   ┌────────────────────────────────┐
│ Ephemeral worker (per job)   │   │ GitHub Actions queue           │
│ Prebaked airstack-ci-runner  │   │  workflow_job  status=queued   │
│ image: Docker + nvidia CTK + │   │  labels: [self-hosted,         │
│ GH runner. Privileged pod    │   │           airstack-ephemeral]  │
│ starts dockerd, runs ONE     │   └────────────────────────────────┘
│ job (JIT), then the pod      │
│ is destroyed.                │
└──────────────────────────────┘
```

### Why this instead of a long-lived self-hosted runner

| Concern | Mitigation |
|---------|------------|
| Cross-job state pollution (Docker cache, dangling networks, leftover artifacts) | Each job runs on a fresh OSMO pod, destroyed within ~30 s of job completion. |
| Fork PRs executing arbitrary code | Workflow's `if: github.event.pull_request.head.repo.full_name == github.repository` — fork PRs skipped. |
| Runner runs privileged (root) for docker-in-docker | The pod is privileged (needed to run `airstack up`/compose), but it is single-use, scoped to the dedicated CI pool, and only same-repo code ever reaches it. |
| Docker socket gives root-equivalent access | Bounded to a single one-shot pod. The orchestrator host doesn't expose Docker at all. |
| Long-lived PAT on the runner host | The PAT lives only on the orchestrator. Workers receive a single-use **JIT runner config** — a base64 token bound to one runner registration. |
| Persistent creds tied to a personal account | Orchestrator authenticates with a shared, non-personal **OSMO service-account token** (revocable, scoped to the CI pool), not an individual's login. |

### Setup

The orchestrator service code, OSMO runner-workflow template, runner image, systemd unit, and full setup runbook live in [`.github/orchestrator/`](../../../../.github/orchestrator/). See [`.github/orchestrator/README.md`](ci-cd-orchestrator.md) for:

- obtaining the OSMO service-account token and a dedicated CI GPU pool (with privileged mode enabled)
- building and pushing the runner image (`runner.Dockerfile`)
- staging the GitHub PAT and the OSMO token
- running `setup.sh` on the orchestrator host (installs the `osmo` CLI)
- filling in osmo_url / pool / platform / runner_image / resources in `/etc/airstack-orchestrator/config.yaml`
- enabling and verifying the `airstack-orchestrator.service` systemd unit

### Runner labels

The workflow file requests `runs-on: [self-hosted, airstack-ephemeral]`. The orchestrator polls for queued jobs whose labels are a superset of `runner_labels` in its config, mints a JIT config registering the ephemeral runner under those same labels, and spawns the worker. To route jobs to a different pool (e.g. CPU-only workers) in the future, add a second label set in config and adjust the workflow's `runs-on`.
