# Robot-side unit test proxies

Layout mirrors [`robot/ros_ws/src/`](../../robot/ros_ws/src/) autonomy layers:

| Directory | Maps to ROS workspace |
|-----------|----------------------|
| `behavior/` | `robot/ros_ws/src/behavior/` |
| `global/` | `robot/ros_ws/src/global/` |
| `interface/` | `robot/ros_ws/src/interface/` |
| `local/` | `robot/ros_ws/src/local/` |
| `perception/` | `robot/ros_ws/src/perception/` |
| `sensors/` | `robot/ros_ws/src/sensors/` |

## Design: co-location + proxy

**Test source** lives co-located with each ROS 2 package (the standard colcon
convention):

```
robot/ros_ws/src/<layer>/<package>/test/test_<name>.py   ← source of truth
```

**This directory** contains thin proxy files that call ``register_unit_tests()``
(from [`tests/conftest.py`](../conftest.py)) to register co-located ``test_*``
functions for ``pytest tests/`` and ``airstack test -m unit``. Each proxy is
~10 lines.

```
tests/robot/<layer>/<package>/test_<name>.py             ← proxy (registers above)
```

Both `airstack test -m unit` (pytest path via proxy) and
`colcon test --packages-select <pkg>` (direct path to source) run the same
test functions from the same file.

All test functions must carry `@pytest.mark.unit`. For adding new tests see the
`add-unit-tests` agent skill.
