# Unit Testing

AirStack unit tests are **fast, hermetic, and purely Python** — no Docker stack, no GPU, no running containers. They run locally in seconds and gate every pull request via a dedicated GitHub Actions workflow on a standard `ubuntu-latest` runner.

## Design principles

- **Co-located with source.** Test files live in `<package>/test/` alongside the code they test. This is the standard ROS 2 / colcon convention and ensures tests are discovered by both `colcon test` and `pytest`.
- **Proxy for centralized discovery.** A thin shim in `tests/robot/<layer>/<package>/` re-exports the test functions so `pytest tests/` (the CI command) and `airstack test -m unit` discover them without any changes to the CI workflow.
- **`@pytest.mark.unit` on every test.** The `unit` mark is the filter that keeps unit tests isolated from system tests that need Docker, GPUs, and sim licenses.

## Repository layout

```
robot/ros_ws/src/
└── <layer>/<package>/
    ├── src/                          # production source
    └── test/
        ├── test_<name>.py            # unit test source  ← canonical location
        ├── test_<name>.cpp           # C++ gtest source (optional)
        └── fake_<name>.hpp           # C++ test doubles (optional)

tests/
├── robot/
│   └── <layer>/<package>/
│       └── test_<name>.py            # thin proxy → package test/
├── sim/                              # future: sim-side unit tests
└── gcs/                              # future: GCS unit tests
```

When pytest collects `tests/robot/…/test_<name>.py`, the `<-` annotation in the
output shows the actual source location:

```
robot/perception/natnet_ros2/test_natnet_ros2.py::test_canonical_quaternion_identity
  <- ../robot/ros_ws/src/perception/natnet_ros2/test/test_natnet_ros2.py  PASSED
```

## Running unit tests

```bash
# Locally — no container or Docker stack required
airstack test -m unit -v

# Or directly with pytest (AIRSTACK_ROOT must point to the repo root)
export AIRSTACK_ROOT=$(pwd)
pip install pytest numpy
pytest tests/ -m unit -v
```

Unit tests complete in under one second for the current suite.

## CI

Unit tests are collected and run as part of `system-tests.yml` via `pytest tests/`
(no marks specified on PR open = all tests including `unit`). Run them locally at
any time with no infrastructure required:

```bash
airstack test -m unit -v
# or directly (requires tests/requirements.txt installed):
AIRSTACK_ROOT=$(pwd) pytest tests/ -m unit -v
```

## Current test coverage

| Package | Test file | What is covered |
|---|---|---|
| `natnet_ros2` | `perception/natnet_ros2/test/test_natnet_ros2.py` | `VisionPoseConverterNode._canonical_quaternion` (ROS stubbed with `sys.modules`) |
| `natnet_ros2` (C++) | `perception/natnet_ros2/test/test_natnet_logic.cpp` | `build_covariance_6x6`, topic name helpers, `ConnectConfig`, `negotiate()` via `FakeNatNetClient` |
| `lidar_point_cloud_filter` | `sensors/lidar_point_cloud_filter/test/test_validation_core.py` | Pure-numpy LiDAR range validation rules |

## Adding a new unit test

### Python

**1. Write the test source in the package:**

```python
# robot/ros_ws/src/<layer>/<package>/test/test_my_module.py
import sys
from pathlib import Path
import pytest

# Make the package importable without a colcon install
_src = Path(__file__).resolve().parent.parent / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))

from my_module import my_function  # noqa: E402


@pytest.mark.unit
def test_basic():
    assert my_function(1, 2) == 3
```

If the production code inherits from `rclpy.node.Node`, stub ROS at the import
boundary:

```python
import sys
from unittest.mock import MagicMock

class _FakeNode:
    def __init__(self, name): pass
    def get_logger(self): return MagicMock()
    def declare_parameter(self, *a, **kw): pass
    def get_parameter(self, name):
        m = MagicMock(); m.value = MagicMock(); return m
    def create_subscription(self, *a, **kw): return MagicMock()
    def create_publisher(self, *a, **kw): return MagicMock()

_rclpy_node_mod = MagicMock()
_rclpy_node_mod.Node = _FakeNode
sys.modules.setdefault("rclpy", MagicMock())
sys.modules["rclpy.node"] = _rclpy_node_mod
# ... then import your module
```

**2. Write the thin proxy in `tests/robot/`:**

```python
# tests/robot/<layer>/<package>/test_my_module.py
"""Proxy: re-exposes <package> unit tests from the package source tree."""
import importlib.util
from pathlib import Path

_repo_root = Path(__file__).resolve().parents[4]  # adjust depth if needed
_real_file = _repo_root / "robot/ros_ws/src/<layer>/<package>/test/test_my_module.py"

_spec = importlib.util.spec_from_file_location("_<package>_unit_tests", _real_file)
_real = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_real)

for _name in dir(_real):
    if _name.startswith("test_"):
        globals()[_name] = getattr(_real, _name)
```

The unique module name (e.g. `"_<package>_unit_tests"`) prevents a circular import:
pytest adds the proxy's directory to `sys.path` at collection time, which would
cause `from test_my_module import *` to import the proxy itself.

**3. Verify:**

```bash
pytest tests/ -m unit -v
```

### C++ (gtest)

C++ tests live entirely in the package and run via `colcon test` — no proxy needed.

**`CMakeLists.txt`:**

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_my_name test/test_my_name.cpp)
  target_include_directories(test_my_name PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>)
endif()
```

**`package.xml`:**

```xml
<test_depend>ament_cmake_gtest</test_depend>
```

**Run inside the robot container:**

```bash
docker exec airstack-robot-desktop-1 bash -c \
  "bws --cmake-args '-DBUILD_TESTING=ON' --packages-select <package>"
docker exec airstack-robot-desktop-1 bash -c \
  "colcon test --packages-select <package> --event-handlers console_direct+"
```

The `build_packages` CI job (`tests/system/test_build_packages.py`) also runs
`colcon test` with `BUILD_TESTING=ON` so C++ gtests are gated in CI as well.

## Extending to sim and GCS

The proxy pattern extends to other components. As sim-side Python logic (e.g. the
[Motive emulator](../../../../tests/sim/motive_emulator/README.md)) and GCS modules
acquire unit-testable code, follow the same layout:

```
simulation/.../<tool>/test/test_<name>.py   ← source
tests/sim/<tool>/test_<name>.py             ← proxy (parents[3] to reach repo root)

gcs/.../<pkg>/test/test_<name>.py           ← source
tests/gcs/<pkg>/test_<name>.py             ← proxy
```

`pytest tests/ -m unit` discovers them automatically — no changes to `pytest.ini`
or CI changes needed.

## See also

- [`.agents/skills/add-unit-tests`](../../../../.agents/skills/add-unit-tests/SKILL.md) — step-by-step agent workflow
- [System tests](../../../../tests/README.md) — full Docker-stack integration tests
- [CI/CD](ci_cd.md) — pipeline overview and ephemeral runner architecture
- [Testing frameworks](testing_frameworks.md) — `colcon test`, ament linters
