# Testing

AirStack uses several complementary test layers, each with a distinct scope and
hardware requirement:

| Layer | Where | Mark / Tool | Hardware |
|---|---|---|---|
| **Unit tests** | `<pkg>/test/` (co-located) | `airstack test -m unit` | None — pure Python |
| **Package tests** | `<pkg>/test/` | `colcon test` | Robot container |
| **System tests** | `tests/system/` | `pytest -m liveliness` etc. | Docker, GPU, sim license |

## Unit tests (`airstack test -m unit`)

Fast, hermetic Python tests that run in seconds with no Docker or GPU. Test source
lives **co-located with its ROS 2 package** (`<package>/test/`); the packages with unit
tests are listed in `tests/colcon_unit_test_packages.yaml`, and the root harness collects
them from there.

```bash
airstack test -m unit -v
# or directly:
pytest tests/ -m unit -v
```

Unit tests run automatically on every update to PRs targeting `main` or
`develop` through `unit-tests.yml` on `ubuntu-latest`, and can also be run
locally with no Docker or GPU needed.

→ **[Unit Testing Guide](unit_testing.md)** — patterns, CI workflow,
  how to add tests for new packages (Python and C++ gtest).

## System tests (`tests/system/`)

Full Docker-stack integration tests, selected by pytest mark — from cheap
build gates (`build_docker`, `build_packages`) through stack bring-up
(`liveliness`, `wiring`, `sensors`) to full flight campaigns
(`takeoff_hover_land`, `autonomy`, `waypoint_flight`). The canonical mark
reference — what each mark runs and verifies — is
**[`tests/README.md`](../../../../tests/README.md)**.

Collection order is defined in `tests/conftest.py` (unit tests first, then
`build_docker` → `build_packages` → `liveliness` → `sensors` → `takeoff_hover_land`
→ `test_fixed_trajectory`). Each mark's test **class** uses **class-scoped**
`airstack_env`, so combining marks with **`or`** runs multiple full stack bring-ups
per `(sim, num_robots, iteration)` — see *Bring-up scope* in `tests/README.md`.

**Isaac Sim:** the `sensors` implementation batches `ros2 topic hz` on sim and
robot paths and avoids `hz` on filtered `PointCloud2`; pytest enables `ENABLE_LIDAR`
for the multi-drone Pegasus script. Details: **`tests/README.md`** → *Isaac Sim and
the sensors mark*.

### Prerequisites

All system tests share the same setup:

```bash
cd /path/to/AirStack
airstack setup
```

- Docker daemon (user in `docker` group)
- NVIDIA GPU + `nvidia-container-toolkit` for sim tests
- Isaac Sim: `simulation/isaac-sim/docker/omni_pass.env` configured

### End-to-end testing

For the full guide — e2e overview, the fixed-trajectory benchmark, metrics, CLI,
comparing trackers, and baselines — see **[End-to-End Testing](end_to_end_testing.md)**.

Quick smoke test:

```bash
airstack test -m "build_packages or autonomy" \
  --sim isaacsim \
  --num-robots 1 \
  --stress-iterations 1 \
  --trajectory-types Circle \
  -v
```

## Other testing docs

- [Unit Testing](unit_testing.md) — `@pytest.mark.unit`, co-located tests, CI workflow
- [System test suite reference](../../../../tests/README.md) — marks, fixtures, CLI options
- [CI/CD Pipeline on OSMO](ci_cd.md) — automatic unit/build gates, selectable full-stack GPU campaigns, triggers, and like-for-like metrics reporting
