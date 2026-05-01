---
name: run-system-tests
description: Run, interpret, and extend AirStack's pytest system test suite (build_packages, build_docker, liveliness, takeoff_hover_land), trigger runs via /pytest PR comments, and read metrics.json regression reports. Use for invoking tests, debugging failures from results.xml/metrics.json, or adding a new system test.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Run AirStack System Tests

## When to Use

Use this skill when you need to:

- Invoke the pytest system tests locally (via `airstack test`) or on CI (via `/pytest` PR comment or `workflow_dispatch`)
- Diagnose a failing system test — interpret `results.xml`, per-test logs, and `metrics.json` from `tests/results/<timestamp>/`
- Compare metrics against a baseline run (`parse_metrics.py --baseline`) to confirm a regression or improvement
- Add a new system test to `tests/`: pick the right mark, wire up `airstack_env` parametrization, and record metrics with `MetricsRecorder`

This skill is about the **test harness itself** — pytest marks, fixtures, the metrics pipeline, the CI trigger, and the ephemeral runner constraints. For authoring per-module simulation scenarios (waypoint sequences, RViz checks, scene design), use the `test-in-simulation` skill instead. For diagnosing why a specific module isn't behaving correctly inside a passing harness, use `debug-module`.

## Test Suite Overview

The suite lives at `tests/` (repo root) and is fully pytest-based. Configuration is in `tests/pytest.ini` and shared infrastructure in `tests/conftest.py`. There are exactly four marks today:

| File | Mark | What it tests | Hardware required |
|------|------|---------------|-------------------|
| `tests/test_build_docker.py` | `build_docker` | `airstack image-build` for `robot-desktop`, `gcs`, `isaac-sim`, `ms-airsim`; records image size to `metrics.json` | Docker daemon |
| `tests/test_build_packages.py` | `build_packages` | `colcon build` (`bws`) inside the robot, GCS, and ms-airsim ROS workspaces — brought up with `AUTOLAUNCH=false` | Docker daemon |
| `tests/test_liveliness.py` | `liveliness` | Full stack up, container Running state, tmux pane survival, sentinel ROS 2 nodes (mavros, robot_state_publisher, trajectory_control_node), sim topic Hz, compute usage, sustained `test_stable` polling window, sim realtime factor | Docker daemon, NVIDIA GPU + `nvidia-container-toolkit`, sim license / Omniverse creds |
| `tests/test_takeoff_hover_land.py` | `takeoff_hover_land` | 4-phase flight chain per `(sim, num_robots, iteration, velocity)`: `test_px4_ready` → `test_takeoff` → `test_hover` → `test_landing`. Records altitude error, overshoot, hover stability, landing accuracy, odometry drift | Docker daemon, NVIDIA GPU, sim license |

The four marks are declared in `tests/pytest.ini`. **Do not invent new marks ad-hoc** — register any new mark there or pytest will warn about unknown marks.

### Test ordering (set by `pytest_collection_modifyitems`)

`conftest.py` enforces a deterministic global order so cheap-and-fast-failing tests surface first:

```
test_build_docker → test_build_packages → test_liveliness → test_takeoff_hover_land
```

Within `test_takeoff_hover_land`, items are re-sorted to `(airstack_env, velocity, phase)` so each `(sim, robots, iter)` env brings the stack up once and the drone goes ground → air → ground per velocity before pytest moves to the next velocity.

### `build_packages` is auto-prepended in CI

The `system-tests.yml` workflow's `Parse pytest args` step automatically prepends `build_packages` to the marks expression whenever the user specifies any marks (and `build_packages` isn't already in the expression). For example:

- `/pytest -m liveliness` → effectively runs `-m "build_packages or liveliness"`
- `/pytest -m takeoff_hover_land` → effectively runs `-m "build_packages or takeoff_hover_land"`
- `/pytest` (no marks) → pytest defaults (everything)
- `/pytest -m build_docker` → unchanged (the build_docker tests rebuild from scratch anyway)

This guarantees that ROS 2 workspaces are built inside the containers before any launch/liveliness test tries to source them. If you intentionally want to skip `build_packages` (e.g. you trust the prebuilt images), include it explicitly: `-m "liveliness and not build_packages"` would work, but the simpler path is to run locally where the prepend logic doesn't apply.

When running **locally** via `airstack test` or `pytest` directly, no auto-prepend happens — you control marks exactly.

## Running Tests Locally

### Primary interface: `airstack test`

`airstack test` builds a containerized test runner from `tests/docker/`, mounts the repo read-only, and forwards all args to pytest. No local Python environment is required.

```bash
# Cheap build tests — no GPU needed
airstack test -m "build_docker or build_packages" -v

# Liveliness — single sim, single robot, single iteration
airstack test -m liveliness \
  --sim msairsim \
  --num-robots 1 \
  --stress-iterations 1 \
  --stable-duration 60 \
  -v

# Takeoff/hover/land — sweep three velocities
airstack test -m takeoff_hover_land \
  --sim msairsim \
  --num-robots 1 \
  --stress-iterations 1 \
  --takeoff-velocities 0.5,1,2 \
  -v

# Visual inspection (disables headless mode; calls xhost + automatically)
airstack test -m liveliness --gui -v
```

### Direct pytest (for debugger / fast iteration)

When iterating on the test code itself, run pytest directly to skip the runner-image rebuild:

```bash
export AIRSTACK_ROOT=$(pwd)
pip install -r tests/requirements.txt
pytest tests/ -m liveliness --sim msairsim --num-robots 1 -v
```

### CLI option to parametrization mapping

The `airstack_env` fixture is parametrized over `(sim, num_robots, iteration)` tuples by `pytest_generate_tests` in `conftest.py`. The fixture itself is **only** activated for tests that request it, so `build_docker` and `build_packages` (which don't bring up the full stack) are not multiplied:

| Flag | Default | Affects | Becomes |
|------|---------|---------|---------|
| `--sim` | `msairsim,isaacsim` | `airstack_env` | One env-tuple per sim |
| `--num-robots` | `1,3` | `airstack_env` | Cross-product with sim |
| `--stress-iterations` | `1` | `airstack_env` | Up/down cycles per `(sim, num_robots)` |
| `--stable-duration` | `120` | `test_liveliness::test_stable` | Total seconds polled |
| `--stable-interval` | `10` | `test_liveliness::test_stable` | Seconds between polls |
| `--gui` | off (headless) | `airstack_env` | Sets `QT_QPA_PLATFORM=offscreen` when off |
| `--takeoff-velocities` | `0.5` (current default) | `test_takeoff_hover_land` | One full 4-phase chain per velocity |

Total parametrize cardinality for sim tests = `len(sims) × len(num_robots) × stress_iterations × len(velocities for takeoff)`. Keep this small locally — a 2×2×3×3 sweep on a workstation is several hours.

### Prerequisites

- Docker daemon running, your user in the `docker` group
- For `liveliness` / `takeoff_hover_land`: NVIDIA driver + `nvidia-container-toolkit`
- For `isaacsim`: `simulation/isaac-sim/docker/omni_pass.env` populated with Omniverse credentials (CI generates a `guest`/`guest` version automatically)
- `airstack setup` already run so `airstack` is on `PATH`
- All required compose images present locally — `airstack_env` calls `missing_images()` and fails fast otherwise. Build them first via `airstack test -m build_docker` or `airstack image-build <service>`.

## Running Tests via PR Comment

The `system-tests.yml` workflow accepts three trigger paths:

1. **PR opened** (same-repo only) — auto-runs pytest with conftest defaults. Fork PRs are skipped to keep arbitrary code off the self-hosted runner.
2. **`/pytest` issue comment** on a PR — only honored from users with `OWNER`, `MEMBER`, or `COLLABORATOR` author association. Fork PRs are explicitly rejected by the `Resolve PR head` step (the PR's head repo must equal `${context.repo.owner}/${context.repo.repo}`).
3. **`workflow_dispatch`** — manual run from the Actions tab with form inputs (`marks`, `sim`, `num_robots`, `stress_iterations`, `stable_duration`, `baseline_run_id`).

### `/pytest` comment grammar

The first line of the comment is parsed via `shlex.split` after stripping the `/pytest` prefix; subsequent lines are treated as freeform context. Examples:

```
/pytest -m liveliness --sim msairsim --num-robots 1 --stress-iterations 1
```

```
/pytest -m takeoff_hover_land --takeoff-velocities 0.5,1
notes: testing the new altitude controller
```

```
/pytest
```
(no args — pytest runs its conftest defaults)

The workflow:
1. Posts an acknowledgment PR comment showing the resolved `pytest tests/ <args>` command and a link to the run
2. Opens an in-progress GitHub Check Run on the PR's head SHA so the run shows up in the **Checks** tab (issue_comment events otherwise associate runs with the default branch)
3. Runs pytest on a freshly-spawned ephemeral OpenStack runner (`runs-on: [self-hosted, airstack-ephemeral]`)
4. Uploads `tests/results/` as artifact `test-results-<sha>-<run_id>` (90-day retention)
5. The downstream `report` job runs `parse_metrics.py` against the latest baseline artifact from the PR's base branch and posts a markdown table back as a PR comment + job summary
6. Closes the Check Run with the final conclusion

### Why fork PRs are blocked

The runner is GPU-equipped, has Docker root access, and is reused (briefly) across the lifetime of one job. Running arbitrary fork code on it would let a contributor exfiltrate registry creds, mine crypto, or pivot into the OpenStack tenant. The same-repo guard is the only line of defense and **must not be removed**. If you need to test a fork PR, mirror the branch into the upstream repo first.

## Interpreting Results and Metrics

### Output layout

Every run (local or CI) produces a fresh timestamped directory under `tests/results/`:

```
tests/results/2025-04-21_14-30-00/
├── results.xml        # JUnit XML — durations + pass/fail per test
├── metrics.json       # Custom metrics keyed by test_node_id → metric_key
└── logs/
    ├── test_build_docker.TestDockerBuilds.test_build_robot_desktop.log
    ├── test_liveliness.TestLiveliness.test_stable[msairsim-rob#1-iter0].log
    ├── airstack_env.test_liveliness.TestLiveliness.test_robot_containers_running[...].log
    └── ...
```

**One log file per test execution**, plus separate `airstack_env.*.log` files for fixture narration (the `up`/`down` of each parametrize tuple). The fixture log file is named to track the rewritten test ID so it lands next to the triggering test.

### `metrics.json` structure

```json
{
  "test_liveliness.TestLiveliness.test_stable[msairsim-rob#1-iter0]": {
    "airstack_up_duration_s": {"value": 42.7, "unit": "s", "direction": "lower_is_better"},
    "robot.sensors.front_stereo.left.image_rect.hz_samples": {
      "samples": [{"t": 10, "value": 19.27}, {"t": 20, "value": 19.31}, ...]
    },
    "airstack-robot-desktop.cpu_pct_samples": {"samples": [...]}
  },
  "docker.robot-desktop": {
    "image_size_mb": {"value": 8421.3, "unit": "MB", "direction": "lower_is_better"}
  }
}
```

Keys follow `test_node_id → metric_key → {value, unit, direction, ...}`. Time-series data (Hz samples, compute snapshots) lives in `*_samples` lists; `parse_metrics.py` expands these into scalar aggregates (`mean`, `min`, `max`, `start_mean`, `end_mean`).

### `parse_metrics.py`: single-run vs diff mode

```bash
# Single-run report — markdown table, exits 0 always
python tests/parse_metrics.py --current tests/results/2025-04-21_14-30-00/

# Diff mode — side-by-side, exits 1 on regression
python tests/parse_metrics.py \
  --current  tests/results/2025-04-21_14-30-00/ \
  --baseline tests/results/2025-04-20_09-00-00/ \
  --threshold 20 \
  --output   report.md
```

The report has three sections per test module:

- **Metrics** — flat scalar metrics (test, key, current, baseline, change%)
- **Sim publishing rates** — pivoted Hz aggregates per topic (`mean`, `start_mean`, `end_mean`, `min`, `max`)
- **Compute usage** — pivoted CPU/mem/GPU per container

Regressions exceeding `--threshold` (default 20%) are flagged `:red_circle:`; improvements beyond threshold get `:green_circle:`. CI fails the job on any regression.

When local-debugging a CI regression, download both artifacts (`test-results-<sha>-<run_id>` from the PR run and from the base branch's most recent run), unzip them under `tests/results/`, and run `parse_metrics.py` locally to see the same table the bot posted.

## Adding a New System Test

Follow this checklist when adding a new system test.

### 1. Pick the right mark

If your test...

- Builds a Docker image → reuse `build_docker`
- Builds a colcon workspace → reuse `build_packages`
- Verifies the running stack → reuse `liveliness`
- Drives the autonomy stack to fly → reuse `takeoff_hover_land`
- Doesn't fit any of these → **register a new mark in `tests/pytest.ini`** before using it. Update the table in `tests/README.md` and the AGENTS.md "System Test Suite" table at the same time.

### 2. File location and naming

- File: `tests/test_<short_descriptor>.py` — matches pytest's default test discovery (`test_*.py`)
- Class: `Test<CamelCase>` with the mark applied at the class level: `@pytest.mark.<mark>`
- Add a class-level `@pytest.mark.timeout(<seconds>)` — long-running sim tests need it
- Imports: pull helpers from `conftest` directly (`from conftest import ...`); `tests/` is on `sys.path` because `testpaths = .` in pytest.ini

### 3. Decide if you need `airstack_env`

- **Need full stack up (sim + robot + GCS)?** Take `airstack_env` as a fixture argument. You'll automatically be parametrized over `(sim, num_robots, iteration)` from CLI flags — `pytest_generate_tests` in conftest activates this only for tests that name the fixture.
- **Just need one container or no containers?** Don't take `airstack_env` — bring up only what you need with `airstack_cmd("up", "<service>", env_overrides={"AUTOLAUNCH": "false"})` and tear down in a `try/finally`, the way `test_build_packages.py` does.
- **Need extra parametrization** (e.g. velocity for `takeoff_hover_land`)? Add a module-level `pytest_generate_tests(metafunc)` in your test file. Don't put it in `conftest.py` unless it applies broadly.

### 4. Use the existing helpers

`conftest.py` exports a deliberate API. Prefer these over rolling your own:

| Helper | Purpose |
|--------|---------|
| `airstack_cmd(*args, env_overrides=, timeout=, log_name=)` | Run `airstack.sh` with logging tee |
| `docker_exec(container, cmd, timeout=)` | Run shell in a container, tees to current log |
| `ros2_exec(container, ros2_cmd, domain_id=, setup_bash=)` | Run `ros2 ...` with the right workspace sourced and `ROS_DOMAIN_ID` set |
| `wait_for_container(name_pattern, timeout=)` | Block until a container is Running |
| `get_robot_containers(pattern=)` | Robot containers sorted by replica index |
| `wait_for_first_message(container, topic, ...)` | Wait for one message on a topic — returns elapsed seconds |
| `sample_hz` / `parallel_sample_hz` | Sample publish rates (parallel version is far cheaper for many topics) |
| `sample_compute_usage(sim_container)` | One-shot Docker stats + nvidia-smi snapshot |
| `read_log_tail(log_name=, lines=)` | Tail the current test's log for assertion messages |

### 5. Record metrics

Every numeric you'd want to track across runs goes through `MetricsRecorder`:

```python
from conftest import current_test_id, get_metrics

m = get_metrics()
tid = current_test_id()

# Scalar metric (most common)
m.record(tid, "altitude_error_m", 0.27, unit="m", direction="lower_is_better")

# For "higher is better" (Hz, success rates, RTF):
m.record(tid, "trajectory_publish_hz", 50.1, unit="Hz", direction="higher_is_better")

# Time series — parse_metrics.py auto-derives mean/start_mean/end_mean/min/max
m.record_list(tid, "robot_1.altitude_samples",
              [{"t": 10, "value": 9.8}, {"t": 20, "value": 9.9}])
```

Conventions:
- **Scalar key naming**: `<entity>.<metric>` for per-container/per-robot scalars (`robot_1.altitude_error_m`, `airstack-robot-desktop-1.cpu_pct`). The reporter collapses replica suffixes (`-1`, `-2`) for cross-run comparison while keeping raw per-replica data in `metrics.json`.
- **Sample series naming**: `<entity>.<type>_samples` where `<type>` is one of `hz`, `cpu_pct`, `mem_mb`, `disk_io_mb`, `net_io_mb`, `gpu_pct`, `vram_mb`, `gpu_temp_c`, `gpu_power_w`, `realtime_factor`. These trigger the auto-aggregate logic in `parse_metrics.py` (see `SAMPLE_TYPES`).
- **Always pass `unit=`** so the report renders correctly. `direction=` defaults to `lower_is_better`.

### 6. Fixture extension

If multiple tests need the same setup, add a fixture in `conftest.py` (not in your test file) so it's available repo-wide. Mirror the `airstack_env` pattern: yield a dict, narrate via `logger_to(log)`, record any setup/teardown timing as metrics.

## Common Pitfalls

- **Forgetting `build_packages`**. If you run `-m liveliness` locally on a fresh checkout, the workspace inside the container is empty and sentinel nodes won't appear. Either run `-m "build_packages or liveliness"` or rely on the CI auto-prepend.
- **Mixing marks unintentionally**. `-m "liveliness or takeoff_hover_land"` brings the stack up multiple times (once per parametrize tuple per mark). Combine deliberately, not by reflex.
- **Running on insufficient hardware**. `liveliness` and `takeoff_hover_land` require an NVIDIA GPU plus nvidia-container-toolkit; without them the sim container won't get GPU access and topic Hz checks will time out. If you only have a CPU, scope to `-m "build_docker or build_packages"`.
- **Expecting interactive sim feedback**. `airstack_env` runs headless by default (`MS_AIRSIM_HEADLESS=true`, `ISAAC_SIM_HEADLESS=true`, `QT_QPA_PLATFORM=offscreen`). Don't add stdin prompts, GUI dialogs, or `input()` calls to test code — they will hang in CI. For local visual debugging only, pass `--gui`.
- **Not capturing metrics in a new test**. If a test fails silently (no metric recorded) the regression report has nothing to compare. Always record at least one scalar via `MetricsRecorder` so the test shows up in `metrics.json`.
- **Letting parametrize cardinality explode**. Defaults `--sim msairsim,isaacsim --num-robots 1,3` with `--stress-iterations 3` is 12 stack up/downs per liveliness test — expensive. Override locally to a single tuple while iterating.
- **Hardcoded container names**. Always use `find_container`, `get_robot_containers`, or `wait_for_container` — replica suffixes (`-1`, `-2`, `-3`) and compose project prefixes change.
- **Asserting on stdout instead of using `read_log_tail`**. The conftest tees subprocess output to per-test log files; assertions should reference those logs (`f"airstack up failed:\n{read_log_tail()}"`) so failures attach the relevant context to the JUnit XML.
- **Trying to SSH into a CI runner mid-job**. Workers are ephemeral OpenStack VMs destroyed within ~30s of job completion. Re-running the job creates a fresh VM. For genuine debugging on the runner, see `.github/orchestrator/README.md` (also exposed at `tests/ci-cd-orchestrator.md`) — but in 99% of cases, reproduce locally with `airstack test`.
- **Forgetting to register a new mark**. Adding `@pytest.mark.my_new_mark` without updating `tests/pytest.ini` produces "PytestUnknownMarkWarning" and makes `-m my_new_mark` fail to filter as expected.

## Quick Reference

### Common invocations

```bash
# Cheapest possible smoke check
airstack test -m "build_docker or build_packages" -v

# Single-config liveliness (fastest path to a real signal)
airstack test -m liveliness --sim msairsim --num-robots 1 \
  --stress-iterations 1 --stable-duration 60 -v

# Full takeoff/hover/land sweep with three velocities
airstack test -m takeoff_hover_land --sim msairsim --num-robots 1 \
  --stress-iterations 1 --takeoff-velocities 0.5,1,2 -v

# Local visual debug (drops headless, calls xhost +)
airstack test -m liveliness --gui -v

# Direct pytest (debugger-friendly, no runner rebuild)
AIRSTACK_ROOT=$(pwd) pytest tests/ -m liveliness --sim msairsim -v

# Single-run metrics report
python tests/parse_metrics.py --current tests/results/<run>/

# Diff mode (CI behavior)
python tests/parse_metrics.py \
  --current tests/results/<new>/ --baseline tests/results/<old>/ \
  --threshold 20
```

### `/pytest` PR comment examples

```
/pytest
/pytest -m liveliness --sim msairsim --num-robots 1
/pytest -m takeoff_hover_land --takeoff-velocities 0.5,1
/pytest -m "build_docker or build_packages"
```

(First line parsed; rest is freeform. Requires write access. Same-repo only.)

### Mark cheat sheet

| You want to... | Use mark expression |
|---------------|---------------------|
| Smoke-test image + workspace builds | `-m "build_docker or build_packages"` |
| Verify the stack comes up clean | `-m liveliness` |
| Verify autonomy can fly the drone | `-m takeoff_hover_land` |
| Full PR validation (CI default for manual dispatch) | `-m "liveliness or takeoff_hover_land"` (CI auto-prepends `build_packages`) |
| Run literally everything | omit `-m` |

### Files to know

- `tests/conftest.py` — fixtures, helpers, `MetricsRecorder`, ordering hooks
- `tests/pytest.ini` — mark registration, log format
- `tests/parse_metrics.py` — markdown reporter, regression diff
- `tests/README.md` — user-facing docs (CLI options, output layout, CI/CD orchestrator)
- `.github/workflows/system-tests.yml` — CI workflow with `/pytest` comment trigger
- `.github/orchestrator/README.md` — ephemeral OpenStack runner setup and SSH-debug procedure

## References

- `tests/README.md` — full user reference, CLI option matrix, CI/CD orchestrator architecture
- `.github/orchestrator/README.md` — ephemeral runner setup, debugging a failed job, SSH-into-worker
- `AGENTS.md` "System Test Suite" and "CI/CD" sections — top-level overview
- Related skills:
  - [test-in-simulation](../test-in-simulation) — authoring per-module simulation scenarios (RViz, bag analysis, scene design)
  - [debug-module](../debug-module) — diagnosing why a specific module misbehaves once the harness passes
