# System Testing

AirStack's system tests bring up the full Docker-based stack — simulator, robot containers, and GCS — and verify end-to-end behavior: container health, ROS 2 node presence, sensor publishing rates, and compute resource usage. Tests are written in Python with pytest and live under `tests/` at the repo root.

---

## Test Suite Structure

| Module | Mark | What it tests | Hardware required |
|--------|------|---------------|-------------------|
| [`test_build_docker.py`](../../../../tests/test_build_docker.py) | `build_docker` | Docker image builds (robot-desktop, gcs, isaac-sim, ms-airsim); records image sizes | Docker daemon |
| [`test_build_packages.py`](../../../../tests/test_build_packages.py) | `build_packages` | `colcon build` inside each container (robot, GCS, ms-airsim ROS workspace) | Docker daemon |
| [`test_liveliness.py`](../../../../tests/test_liveliness.py) | `liveliness` | Full stack up: container health, tmux process liveness, sentinel ROS 2 nodes, sim topic publishing rates, compute usage, sustained stability | Docker daemon, GPU, sim license |

Marks can be combined with pytest logic: `-m "build_docker or build_packages"`, `-m liveliness`.

---

## Test Infrastructure

All shared fixtures, helpers, and configuration live in [`tests/conftest.py`](../../../../tests/conftest.py).

### `airstack_env` fixture

Parametrized over `(sim, num_robots, iteration)` tuples derived from CLI flags. For each combination it:

1. Calls `airstack up` with the appropriate `COMPOSE_PROFILES`, `NUM_ROBOTS`, and headless flags
2. Records `airstack_up_duration_s` to `metrics.json`
3. Yields an `env` dict used by every `TestLiveliness` test
4. Tears down with `airstack down` and records `airstack_down_duration_s`

### `MetricsRecorder`

Writes custom metrics to `tests/results/<timestamp>/metrics.json` after each `record()` call. Keys follow the pattern `test_node_id → metric_key → {value, unit, direction}`. Time-series data (Hz samples, compute snapshots) are stored as `{key}_samples` lists and expanded into scalar aggregates (mean, min, max, start_mean, end_mean) by `parse_metrics.py`.

### Output files

Every test run produces a timestamped directory:

```
tests/results/
└── 2025-04-21_14-30-00/
    ├── results.xml        # JUnit XML — test durations and pass/fail status
    ├── metrics.json       # Custom metrics (image sizes, Hz, compute, timing)
    └── logs/
        ├── test_build_docker.TestDockerBuilds.test_build_robot_desktop.log
        ├── test_liveliness.TestLiveliness.test_stable[msairsim-1-iter0].log
        └── ...            # One log file per test execution
```

---

## Running Locally

### Prerequisites

- Docker daemon running with the `runner` user (or your user) in the `docker` group
- NVIDIA drivers + `nvidia-container-toolkit` for liveliness tests
- `pip install -r tests/requirements.txt`

### Direct (recommended for development)

```bash
# From the repo root:
export AIRSTACK_ROOT=$(pwd)

# Build tests only (fast, no GPU needed)
pytest tests/ -m "build_docker or build_packages" -v

# Full liveliness run — ms-airsim, 1 robot, 1 iteration, 60s stability window
pytest tests/ -m liveliness \
  --sim msairsim \
  --num-robots 1 \
  --stress-iterations 1 \
  --stable-duration 60 \
  -v

# Show GUI windows (for local visual inspection)
pytest tests/ -m liveliness --gui -v
```

### Docker-compose wrapper

The `tests/docker/` directory provides a containerized test runner that has Docker CLI and all Python dependencies pre-installed.

```bash
export AIRSTACK_PATH=$(pwd)
docker compose -f tests/docker/docker-compose.yaml run --rm test \
  pytest -m "build_docker or build_packages" -v
```

### CLI option reference

| Option | Default | Description |
|--------|---------|-------------|
| `--sim` | `msairsim,isaacsim` | Comma-separated sim targets |
| `--num-robots` | `1,3` | Comma-separated robot counts |
| `--stress-iterations` | `3` | Up/down cycles per (sim, num_robots) config |
| `--stable-duration` | `120` | Seconds `test_stable` polls for |
| `--stable-interval` | `10` | Seconds between polls in `test_stable` |
| `--gui` | off | Show simulator GUI (disables headless mode) |

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
- **Sim publishing rates** — pivot table of topic Hz aggregates (mean, start_mean, end_mean, min, max)
- **Compute usage** — pivot table of CPU/memory/GPU metrics per container

Regressions are flagged with :red_circle:, improvements with :green_circle:.

---

## CI/CD Integration

### Workflow: `integration-tests.yml`

[`.github/workflows/integration-tests.yml`](../../../../.github/workflows/integration-tests.yml) runs on:

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

**`run-tests`** runs on the self-hosted GPU runner (`[self-hosted, airstack, gpu]`). It installs dependencies, runs pytest, and uploads `tests/results/` as an artifact named `test-results-<sha>-<run_id>` with 90-day retention.

**`report`** runs on `ubuntu-latest` after `run-tests` (even if it failed). It:

1. Downloads the current artifact
2. Downloads a baseline artifact (from the base branch for PRs, from `main` for manual runs, or from the specified `baseline_run_id`)
3. Runs `parse_metrics.py` in diff mode if a baseline is found, otherwise in single-run mode
4. Posts the markdown report as a PR comment (PR runs) or to the job summary (all runs)
5. Fails with `::error::` if `parse_metrics.py` exits 1 (regression detected)

#### Required third-party action

The workflow uses [`dawidd6/action-download-artifact@v3`](https://github.com/dawidd6/action-download-artifact) to download artifacts from other workflow runs by branch name. This is a community action and must be trusted in your repository's Actions settings if you use a restricted allowed-actions policy.

---

## Self-Hosted Runner Setup

AirStack's tests require a GPU and Docker, so they run on a self-hosted OpenStack VM. The setup uses the **ephemeral runner** pattern: each runner process registers, executes exactly one job, and then de-registers. This prevents cross-job environment contamination and stale runner accumulation.

### 1. Create the runner user

```bash
sudo useradd -m -s /bin/bash runner
sudo usermod -aG docker runner   # allows Docker commands without sudo
```

### 2. Install the GitHub Actions runner binary

Download the latest runner tarball from [github.com/actions/runner/releases](https://github.com/actions/runner/releases) and unpack it:

```bash
sudo mkdir -p /opt/actions-runner
cd /opt/actions-runner
# Replace the URL with the current release for linux-x64:
curl -Lo actions-runner.tar.gz https://github.com/actions/runner/releases/download/vX.Y.Z/actions-runner-linux-x64-X.Y.Z.tar.gz
sudo tar xzf actions-runner.tar.gz -C /opt/actions-runner
sudo chown -R runner:runner /opt/actions-runner
```

### 3. Store the GitHub PAT

Create a fine-grained or classic PAT with **`repo`** scope (for private repos) or **`public_repo`** scope (for public repos). Store it securely:

```bash
echo "ghp_YOUR_TOKEN_HERE" | sudo tee /etc/github-runner-pat
sudo chmod 600 /etc/github-runner-pat
sudo chown runner:runner /etc/github-runner-pat
```

### 4. Configure runner environment

Create `/etc/github-runner-env` (loaded by the systemd unit):

```ini
REPO_URL=https://github.com/YOUR_ORG/AirStack
REPO_PATH=YOUR_ORG/AirStack
RUNNER_LABELS=self-hosted,airstack,gpu
RUNNER_GROUP=Default
RUNNER_DIR=/opt/actions-runner
PAT_FILE=/etc/github-runner-pat
```

```bash
sudo chmod 600 /etc/github-runner-env
sudo chown runner:runner /etc/github-runner-env
```

### 5. Install the registration script

```bash
sudo cp .github/runners/register-runner.sh /opt/actions-runner/register-runner.sh
sudo chown runner:runner /opt/actions-runner/register-runner.sh
sudo chmod +x /opt/actions-runner/register-runner.sh
```

### 6. Install and enable the systemd service

```bash
sudo cp .github/runners/airstack-runner.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now airstack-runner.service
```

Check status:

```bash
sudo systemctl status airstack-runner.service
sudo journalctl -u airstack-runner.service -f
```

### 7. Verify runner registration

After the service starts it will loop waiting for a job. Confirm it appears in **GitHub → Repository → Settings → Actions → Runners** with the labels `self-hosted`, `airstack`, `gpu` and status **Idle**.

Trigger a `workflow_dispatch` run and watch the runner pick it up, complete the job, and re-register.

### Security considerations

| Concern | Mitigation |
|---------|------------|
| Fork PRs executing arbitrary code on the runner | Workflow has `if: github.event.pull_request.head.repo.full_name == github.repository` — fork PRs are skipped entirely |
| Cross-job state pollution | `--ephemeral` flag: runner de-registers and the process exits after each job; the systemd loop starts a clean process for the next job |
| Runner running as root | Dedicated non-root `runner` user; never set `RUNNER_ALLOW_RUNASROOT=1` |
| Docker socket gives root-equivalent access | Accepted risk for lab use; the fork PR guard above limits who can reach the runner |
| Long-lived PAT stored on disk | Scope the PAT to the minimum required; rotate it periodically; `chmod 600` and owned by `runner` only |
