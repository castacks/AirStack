# Robot-side unit tests

Layout mirrors [`robot/ros_ws/src/`](../../robot/ros_ws/src/) autonomy layers:

| Directory | Maps to ROS workspace |
|-----------|----------------------|
| `behavior/` | `robot/ros_ws/src/behavior/` |
| `global/` | `robot/ros_ws/src/global/` |
| `interface/` | `robot/ros_ws/src/interface/` |
| `local/` | `robot/ros_ws/src/local/` |
| `perception/` | `robot/ros_ws/src/perception/` |
| `sensors/` | `robot/ros_ws/src/sensors/` |

Tests here are marked `@pytest.mark.unit` unless they need Docker or a live sim (use `tests/system/` or `tests/sim/` instead).

Keep **test-only** Python (fixtures, pure numeric checks, protocol stubs) under `tests/` — for example `tests/robot/sensors/lidar_point_cloud_filter/validation_core.py`. `tests/conftest.py` extends `sys.path` where needed so `pytest` can import those modules.
