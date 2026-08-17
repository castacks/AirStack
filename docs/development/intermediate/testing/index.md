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
# or containerless — the `cd` is load-bearing:
cd tests && pytest -m unit -v
```

No CI workflow currently runs unit tests, so run them locally before pushing.

→ **[Unit Testing Guide](unit_testing.md)** — patterns, CI workflow,
  how to add tests for new packages (Python and C++ gtest).

## System tests (`tests/system/`)

Full Docker-stack integration tests. The canonical reference is
**[`tests/README.md`](../../../../tests/README.md)**. In short:

| Mark | Module | Role |
|---|---|---|
| `build_docker` | `system/test_build_docker.py` | Docker image builds |
| `build_packages` | `system/test_build_packages.py` | `colcon build` inside containers |
| `liveliness` | `system/test_liveliness.py` | Containers, `/clock` readiness, tmux, sentinel ROS 2 nodes, compute, infra-only stability poll |
| `sensors` | `system/test_sensors.py` | Sim + robot stereo/depth Hz, filtered LiDAR (`echo --once` + validation script on Isaac), sim RTF, sensor stability time-series |
| `takeoff_hover_land` | `system/test_takeoff_hover_land.py` | Four-phase flight chain per configuration (takeoff → hover → land) |
| `autonomy` | `system/test_fixed_trajectory.py` | Fixed-pattern path-tracker benchmark (takeoff → trajectory → land) |

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
- [Testing frameworks](testing_frameworks.md) — `colcon test`, rostest patterns
- [Integration testing](integration_testing.md)
- [CI/CD Pipeline on OSMO](ci_cd.md) — how CI runs the full stack on ephemeral GPU pods: architecture, triggers, what each mark catches, and the metrics regression gate
