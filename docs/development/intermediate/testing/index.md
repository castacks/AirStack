# Testing

AirStack uses several test layers: ROS 2 package tests (`colcon test`), and **system tests** under [`tests/`](../../../../tests/) at the repo root (pytest, full Docker stack).

## System tests (`tests/`)

The canonical reference is **[`tests/README.md`](../../../../tests/README.md)** (also included in the MkDocs site). In short:

| Mark | Module | Role |
|------|--------|------|
| `liveliness` | `test_liveliness.py` | Containers, `/clock` readiness, tmux, sentinel ROS 2 nodes, compute, infra-only stability poll |
| `sensors` | `test_sensors.py` | Sim + robot stereo/depth Hz, filtered LiDAR (`echo --once` + validation script on Isaac), sim RTF, sensor stability time-series |
| `takeoff_hover_land` | `test_takeoff_hover_land.py` | Four-phase flight chain per configuration |

Collection order is defined in `tests/conftest.py` (`liveliness` before `sensors` before `takeoff_hover_land`). Each mark’s test **class** uses **class-scoped** `airstack_env`, so combining marks with **`and`** runs multiple full stack bring-ups per `(sim, num_robots, iteration)` — see *Bring-up scope* in `tests/README.md`.

**Isaac Sim:** the `sensors` implementation batches `ros2 topic hz` on sim and robot paths and avoids `hz` on filtered `PointCloud2`; pytest enables `ENABLE_LIDAR` for the multi-drone Pegasus script. Details: **`tests/README.md`** → *Isaac Sim and the sensors mark*.

## Other testing docs

- [Testing frameworks](testing_frameworks.md) — `colcon test`, rostest patterns
- [Integration testing](integration_testing.md)
- [CI/CD](ci_cd.md) — pipeline overview
