# Testing

AirStack uses several test layers: ROS 2 package tests (`colcon test`), and **system tests** under [`tests/`](../../../../tests/) at the repo root (pytest, full Docker stack).

## System tests (`tests/`)

The canonical reference is **[`tests/README.md`](../../../../tests/README.md)** (also included in the MkDocs site). In short:

| Mark | Module | Role |
|------|--------|------|
| `build_docker` | `test_build_docker.py` | Docker image builds |
| `build_packages` | `test_build_packages.py` | `colcon build` inside containers |
| `liveliness` | `test_liveliness.py` | Containers, `/clock` readiness, tmux, sentinel ROS 2 nodes, compute, infra-only stability poll |
| `sensors` | `test_sensors.py` | Sim + robot stereo/depth Hz, filtered LiDAR (`echo --once` + validation script on Isaac), sim RTF, sensor stability time-series |
| `takeoff_hover_land` | `test_takeoff_hover_land.py` | Four-phase flight chain per configuration (takeoff → hover → land) |
| `autonomy` | `test_fixed_trajectory.py` | Fixed-pattern path-tracker benchmark (takeoff → trajectory → land) |

Collection order is defined in `tests/conftest.py` (`build_docker` → `build_packages` → `liveliness` → `sensors` → `takeoff_hover_land` → `test_fixed_trajectory`). Each mark's test **class** uses **class-scoped** `airstack_env`, so combining marks with **`or`** runs multiple full stack bring-ups per `(sim, num_robots, iteration)` — see *Bring-up scope* in `tests/README.md`.

**Isaac Sim:** the `sensors` implementation batches `ros2 topic hz` on sim and robot paths and avoids `hz` on filtered `PointCloud2`; pytest enables `ENABLE_LIDAR` for the multi-drone Pegasus script. Details: **`tests/README.md`** → *Isaac Sim and the sensors mark*.

### Fixed-trajectory path-tracker benchmark

For the full guide — purpose, metrics, CLI, comparing trackers, bug fixes, and baselines — see **[Fixed-Trajectory Path-Tracker Benchmark](fixed_trajectory_testing.md)**.

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

- [Testing frameworks](testing_frameworks.md) — `colcon test`, rostest patterns
- [Integration testing](integration_testing.md)
- [CI/CD](ci_cd.md) — pipeline overview
