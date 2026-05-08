# System Testing

AirStack's system tests bring up the full Docker-based stack — simulator, robot containers, and GCS — and verify end-to-end behavior: container health, ROS 2 node presence, sensor publishing rates (in the `sensors` mark), and compute resource usage. Tests are written in Python with pytest and live under `tests/` at the repo root.

<iframe width="1120" height="630" src="https://www.youtube.com/embed/EzgGHnYDI_k?si=vpqER-TXud5XEMUX" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

---

## Test Suite Structure

| Module | Mark | What it tests | Hardware required |
|--------|------|---------------|-------------------|
| [`test_build_docker.py`](../../../../tests/test_build_docker.py) | `build_docker` | Docker image builds (robot-desktop, gcs, isaac-sim, ms-airsim); records image sizes | Docker daemon |
| [`test_build_packages.py`](../../../../tests/test_build_packages.py) | `build_packages` | `colcon build` inside each container (robot, GCS, ms-airsim ROS workspace) | Docker daemon |
| [`test_liveliness.py`](../../../../tests/test_liveliness.py) | `liveliness` | Stack bring-up: container Running state, ``/clock`` readiness, tmux panes, sentinel ROS 2 nodes, compute snapshot, infra-only ``test_stable`` (tmux + nodes + compute) | Docker daemon, GPU, sim license |
| [`test_sensors.py`](../../../../tests/test_sensors.py) | `sensors` | After liveliness in collection order: sim + robot stereo/depth Hz (**Isaac:** batched ``ros2 topic hz`` to avoid bridge overload; **ms-airsim:** single batch), filtered LiDAR via ``echo --once`` + cloud sanity (isaacsim), sim RTF, ``test_sensor_streams_stable`` | Docker daemon, GPU, sim license |
| [`test_takeoff_hover_land.py`](../../../../tests/test_takeoff_hover_land.py) | `takeoff_hover_land` | End-to-end flight: PX4 readiness gate, takeoff to 10 m, hover stability, land — one chain per (sim, num_robots, iteration, velocity) | Docker daemon, GPU, sim license |

Marks can be combined with pytest logic:
`-m "build_docker or build_packages"`, `-m liveliness`, `-m sensors`, `-m takeoff_hover_land`, or e.g. `-m "liveliness or sensors"` (see **Bring-up scope** below).

### Bring-up scope (`airstack_env`)

`airstack_env` is **class-scoped** and parametrized per `(sim, num_robots, iteration)`. Each test **class** that uses it (`TestLiveliness`, `TestSensors`, `TestTakeoffHoverLand`, …) performs its **own** ``airstack up`` / ``airstack down`` for that parametrization. Selecting both classes (for example, ``-m "liveliness or sensors"``) runs **two** full stack cycles per tuple (liveliness class, then sensors class). Collection order (see ``conftest.py``) runs **liveliness before sensors** when both are selected. To save wall time, run ``-m liveliness`` or ``-m sensors`` alone when one suite is enough.

---

## Test Infrastructure

All shared fixtures, helpers, and configuration live in [`tests/conftest.py`](../../../../tests/conftest.py).

### `airstack_env` fixture

Parametrized over `(sim, num_robots, iteration)` tuples derived from CLI flags. For each combination it:

1. Calls `airstack up` with the appropriate `COMPOSE_PROFILES`, `NUM_ROBOTS`, and headless flags
2. Records `airstack_up_duration_s` to `metrics.json`
3. Yields an `env` dict used by liveliness and sensor tests
4. Tears down with `airstack down` and records `airstack_down_duration_s`

### Isaac Sim and the `sensors` mark

**LiDAR in pytest:** [`tests/conftest.py`](../../../../tests/conftest.py) sets
`ENABLE_LIDAR=true` in `SIM_CONFIG["isaacsim"]["extra_env"]` so the multi-drone
Pegasus script (`example_multi_px4_pegasus_launch_script.py`) attaches RTX LiDAR
the same way the single-drone script always does. Without that flag the multi
script would not spawn LiDAR OmniGraphs.

**Topic checks** live in [`tests/sensor_probes.py`](../../../../tests/sensor_probes.py)
and are driven by [`tests/test_sensors.py`](../../../../tests/test_sensors.py):

| Path | What we measure | How |
|------|-----------------|-----|
| Sim → `/clock`, stereo images, stereo depth | Publish rate | ``ros2 topic hz`` on the sim container: ``/clock`` alone, then **chunks of two** ``image_rect`` topics, then **chunks of two** depth topics (``ISAACSIM_HZ_CHUNK_SIZE`` in ``sensor_probes.py``). |
| Robot → same topic names (bridge) | Publish rate | Same **two-at-a-time** chunking on the robot container for Isaac. ms-airsim: one batch of four topics. |
| Robot → filtered ``.../ouster/point_cloud`` | Stream alive | ``ros2 topic echo --once`` per robot (not Hz — large ``PointCloud2``). |
| LiDAR geometry | Near-range vs ``near_range_m`` | ``lidar_point_cloud_filter/scripts/validate_lidar_filter_clouds.py`` (raw vs filtered). |

Sim **RTF** (real-time factor from ``/clock``) is also in the `sensors` suite.
**`test_sensor_streams_stable`** repeats sim + robot stereo + LiDAR probes every
`--stable-interval` for `--stable-duration` and records time-series to
`metrics.json` (stereo/depth as ``*.hz_samples``; LiDAR echo-once as ``*.received_samples``).

### `MetricsRecorder`

Writes custom metrics to `tests/results/<timestamp>/metrics.json` after each `record()` call. Keys follow the pattern `test_node_id → metric_key → {value, unit, direction}`. Time-series data (Hz samples, compute snapshots) are stored as `{key}_samples` lists and expanded into scalar aggregates (mean, min, max, start_mean, end_mean) by `parse_metrics.py`.

### Output files

Every test run produces a timestamped directory. **Per-test logs** — for each
pytest function, `pytest_runtest_setup` in `conftest.py` attaches the shared
logger to `logs/test_<module>.<Class>.<test>[<param-id>].log` (param ids are
rewritten for readability, e.g. `msairsim-rob#1-iter0`; see
`pytest_collection_modifyitems`).

**`airstack_env.<…>.log`** — the class-scoped `airstack_env` fixture wraps
`airstack up` / `airstack down` in `logger_to("airstack_env." + <current nodeid>)`
(see `conftest.py`). So you get an extra file whose name is the word
`airstack_env.` plus the **node id of whichever test was running when the
fixture first ran** for that class. For `TestLiveliness` that is almost always
`test_robot_containers_running` (first test in the class), not `test_stable`.
That file holds compose / `airstack` subprocess output; each test still has its
own log for assertions and `docker exec` / `ros2` lines.

```
tests/results/
└── 2025-04-21_14-30-00/
    ├── results.xml        # JUnit XML — test durations and pass/fail status
    ├── metrics.json       # Custom metrics (image sizes, Hz, compute, timing)
    └── logs/
        ├── test_build_docker.TestDockerBuilds.test_build_robot_desktop.log
        ├── airstack_env.test_liveliness.TestLiveliness.test_robot_containers_running[msairsim-rob#1-iter0].log
        ├── test_liveliness.TestLiveliness.test_robot_containers_running[msairsim-rob#1-iter0].log
        ├── test_liveliness.TestLiveliness.test_stable[msairsim-rob#1-iter0].log
        ├── test_sensors.TestSensors.test_sensor_streams_stable[msairsim-rob#1-iter0].log
        └── ...            # More per-test logs; another airstack_env.* per class using the fixture
```

---

## Running Tests

### `airstack test` (primary interface)

`airstack test` is the standard way to run tests. It builds the containerized
test runner from `tests/docker/`, mounts the repo read-only, and forwards all
arguments directly to pytest. No local Python environment needed.

```bash
# From the repo root (AirStack must be set up: airstack setup):

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
- NVIDIA drivers + `nvidia-container-toolkit` for liveliness, sensors, and takeoff_hover_land tests
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

## Autonomy Tests (`test_takeoff_hover_land.py`)

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

## Metrics Reporting (`parse_metrics.py`)

[`tests/parse_metrics.py`](../../../../tests/parse_metrics.py) reads `results.xml` and `metrics.json` from a run directory and produces a markdown report. It has two modes:

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

**`run-tests`** runs on a freshly-spawned ephemeral OpenStack instance (`[self-hosted, airstack-ephemeral]`). The instance is provisioned per-job by the orchestrator described below and destroyed once the job completes. It installs dependencies, runs pytest, and uploads `tests/results/` as an artifact named `test-results-<sha>-<run_id>` with 90-day retention.

**`report`** runs on `ubuntu-latest` after `run-tests` (even if it failed). It:

1. Downloads the current artifact
2. Downloads a baseline artifact (from the base branch for PRs, from `main` for manual runs, or from the specified `baseline_run_id`)
3. Runs `parse_metrics.py` in diff mode if a baseline is found, otherwise in single-run mode
4. Posts the markdown report as a PR comment (PR runs) or to the job summary (all runs)
5. Fails with `::error::` if `parse_metrics.py` exits 1 (regression detected)

#### Required third-party action

The workflow uses [`dawidd6/action-download-artifact@v6`](https://github.com/dawidd6/action-download-artifact) to download artifacts from other workflow runs by branch name. This is a community action and must be trusted in your repository's Actions settings if you use a restricted allowed-actions policy.

---

## CI/CD Orchestrator (OpenStack-backed ephemeral runners)

AirStack's tests require a GPU, Docker, and a clean filesystem per run, so they execute on **truly ephemeral OpenStack instances** spawned per-job by an orchestrator. Each test job gets a fresh VM that is destroyed once the job completes — no Docker layer carryover, no leaked containers, no shared host state.

### Architecture

```
┌──────────────────────────────────────────────────────────────┐
│  Orchestrator VM  (airstack-ci-cd-orchestrator)              │
│   • polls GitHub for queued workflow_jobs                    │
│   • mints single-use JIT runner tokens                       │
│   • spawns / reaps ephemeral instances via OpenStack Nova    │
│   • holds the GitHub PAT and OpenStack application credential│
└────────────┬───────────────────────────────────┬─────────────┘
             │                                   │
             ▼                                   ▼
┌──────────────────────────────┐   ┌────────────────────────────────┐
│ Ephemeral worker (per job)   │   │ GitHub Actions queue           │
│ Image: Ubuntu-24.04-GPU-     │   │  workflow_job  status=queued   │
│        Headless              │   │  labels: [self-hosted,         │
│ cloud-init bootstraps Docker │   │           airstack-ephemeral]  │
│ + nvidia-container-toolkit + │   └────────────────────────────────┘
│ GH Actions runner; runs ONE  │
│ job, then is destroyed.      │
└──────────────────────────────┘
```

### Why this instead of a long-lived self-hosted runner

| Concern | Mitigation |
|---------|------------|
| Cross-job state pollution (Docker cache, dangling networks, leftover artifacts) | Each job runs on a fresh VM. Spent VM is destroyed within ~30 s of job completion. |
| Fork PRs executing arbitrary code | Workflow's `if: github.event.pull_request.head.repo.full_name == github.repository` — fork PRs skipped. |
| Runner running as root | The runner runs as the unprivileged `ubuntu` user inside an instance whose only purpose is one job. |
| Docker socket gives root-equivalent access | Bounded to a single one-shot VM. The orchestrator host doesn't expose Docker at all. |
| Long-lived PAT on the runner host | The PAT lives only on the orchestrator. Workers receive a single-use **JIT runner config** — a base64 token bound to one runner registration. |
| Persistent OpenStack creds tied to a user password | Orchestrator authenticates with an **application credential** (revocable, scoped) instead of `openrc.sh`. |

### Setup

The orchestrator service code, cloud-init template, systemd unit, and full setup runbook live in [`.github/orchestrator/`](../../../../.github/orchestrator/). See [`.github/orchestrator/README.md`](ci-cd-orchestrator.md) for:

- creating the OpenStack application credential and `clouds.yaml`
- staging the GitHub PAT
- running `setup.sh` on the orchestrator VM
- filling in flavor / network / keypair / security-group in `/etc/airstack-orchestrator/config.yaml`
- enabling and verifying the `airstack-orchestrator.service` systemd unit

### Runner labels

The workflow file requests `runs-on: [self-hosted, airstack-ephemeral]`. The orchestrator polls for queued jobs whose labels are a superset of `runner_labels` in its config, mints a JIT config registering the ephemeral runner under those same labels, and spawns the worker. To route jobs to a different pool (e.g. CPU-only workers) in the future, add a second label set in config and adjust the workflow's `runs-on`.
