# Testing

AirStack uses three complementary test layers, each with a distinct scope and
hardware requirement:

| Layer | Where | Mark / Tool | Hardware |
|---|---|---|---|
| **Unit tests** | `tests/robot/`, `tests/sim/` | `pytest -m unit` | None — pure Python |
| **Package tests** | `<pkg>/test/` | `colcon test` | Robot container |
| **System tests** | `tests/system/` | `pytest -m liveliness` etc. | Docker, GPU, sim license |

## Unit tests (`pytest -m unit`)

Fast, hermetic Python tests that run in seconds with no Docker or GPU. Test source
lives **co-located with its ROS 2 package** (`<package>/test/`) and is re-exported
through thin proxy files in `tests/robot/` for centralized discovery.

```bash
airstack test -m unit -v
# or directly:
pytest tests/ -m unit -v
```

A dedicated CI workflow (`.github/workflows/unit-tests.yml`) runs these on every
push and PR on a GitHub-hosted `ubuntu-latest` runner — fast mandatory feedback
before the heavier GPU-based system tests are needed.

→ **[Unit Testing Guide](unit_testing.md)** — patterns, proxy layout, CI workflow,
  how to add tests for new packages (Python and C++ gtest).

## System tests (`tests/system/`)

Full Docker-stack integration tests. The canonical reference is
**[`tests/README.md`](../../../../tests/README.md)**. In short:

| Mark | Module | Role |
|---|---|---|
| `liveliness` | `system/test_liveliness.py` | Containers, `/clock` readiness, tmux, sentinel ROS 2 nodes, compute, infra-only stability poll |
| `sensors` | `system/test_sensors.py` | Sim + robot stereo/depth Hz, filtered LiDAR (`echo --once` + validation script on Isaac), sim RTF, sensor stability time-series |
| `takeoff_hover_land` | `system/test_takeoff_hover_land.py` | Four-phase flight chain per configuration |

Collection order is defined in `tests/conftest.py` (`liveliness` before `sensors`
before `takeoff_hover_land`). Each mark's test class uses **class-scoped**
`airstack_env`, so combining marks with `and` runs multiple full stack bring-ups
per `(sim, num_robots, iteration)` — see *Bring-up scope* in `tests/README.md`.

**Isaac Sim:** the `sensors` implementation batches `ros2 topic hz` on sim and
robot paths and avoids `hz` on filtered `PointCloud2`; pytest enables `ENABLE_LIDAR`
for the multi-drone Pegasus script. Details: **`tests/README.md`** → *Isaac Sim and
the sensors mark*.

## Other testing docs

- [Unit Testing](unit_testing.md) — `@pytest.mark.unit`, proxy pattern, CI workflow
- [Testing frameworks](testing_frameworks.md) — `colcon test`, rostest patterns
- [Integration testing](integration_testing.md)
- [CI/CD](ci_cd.md) — pipeline overview
