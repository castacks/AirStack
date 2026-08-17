# Unit Testing

AirStack unit tests are **fast, hermetic, and purely Python** — no Docker stack, no GPU, no running containers. They run locally in seconds via `airstack test -m unit`. No CI workflow runs them today, so run them yourself before pushing.

## Design principles

- **Co-located with source.** Test files live in `<package>/test/` alongside the code they test. This is the standard ROS 2 / colcon convention and ensures tests are discovered by both `colcon test` and `pytest`.
- **Listed in one place.** `tests/colcon_unit_test_packages.yaml` lists which packages have unit tests. `tests/conftest.py` resolves each to its `test/` dir and collects the non-linter `test_*.py` files under `--import-mode=importlib`. To add a package's unit tests, list it in that YAML.
- **`@pytest.mark.unit` on every test, applied for you.** `conftest.py` marks items by file location, so test sources should not declare it themselves. The `unit` mark keeps unit tests isolated from system tests that need Docker, GPUs, and sim licenses.

## Repository layout

```
robot/ros_ws/src/
└── <layer>/<package>/
    ├── src/                          # production source
    └── test/
        ├── test_<name>.py            # unit test source  ← canonical location (collected directly)
        ├── test_<name>.cpp           # C++ gtest source (optional)
        └── fake_<name>.hpp           # C++ test doubles (optional)

tests/
└── colcon_unit_test_packages.yaml    # lists the packages whose test/ dirs are collected
```

Collected items point straight at the co-located source:

```
../robot/ros_ws/src/perception/natnet_ros2/test/test_natnet_ros2.py::test_canonical_quaternion_identity  PASSED
```

## Running unit tests

```bash
# Locally — no container or Docker stack required
airstack test -m unit -v

# Or directly with pytest. The `cd` is load-bearing: pytest only injects the
# co-located tests when no path is given on the command line.
export AIRSTACK_ROOT=$(pwd)
pip install -r tests/requirements.txt
cd tests && pytest -m unit -v
```

Unit tests complete in under one second for the current suite.

## CI

No workflow runs unit tests today. `system-tests.yml` invokes `pytest tests/`, which
does not collect them, and it only triggers on PR open, a `/pytest` comment, or
`workflow_dispatch`. Run them locally before pushing — no infrastructure required:

```bash
airstack test -m unit -v
# or directly (requires tests/requirements.txt installed):
cd tests && AIRSTACK_ROOT=$(git rev-parse --show-toplevel) pytest -m unit -v
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

# Make the package importable without a colcon install
_src = Path(__file__).resolve().parent.parent / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))

from my_module import my_function  # noqa: E402


def test_basic():
    assert my_function(1, 2) == 3
```

No `@pytest.mark.unit` — `conftest.py` applies it by file location. Import `pytest`
only if you need its API (`approx`, `raises`, `parametrize`, `importorskip`).

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

**2. Register the package in `tests/colcon_unit_test_packages.yaml`:**

```yaml
robot:
  packages:
    - <your_package>          # ← add here; conftest.py collects <pkg>/test/test_*.py
  pytest_args: []             # forwarded to colcon via PYTEST_ADDOPTS; `-m` is ignored there
```

That's the whole registration. If the test imports package code, set up `sys.path` at the
top of the test file — see `test_validation_core.py`, which inserts its package root.
`--import-mode=importlib` (set in `pytest.ini`) means duplicate `test_*.py` basenames
across packages don't collide.

**3. Verify:**

```bash
airstack test -m unit -v
```

### C++ (gtest)

C++ tests live entirely in the package and run via `colcon test`.

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

The mechanism extends to other components via the same YAML. `tests/harness/discovery.py`
(`_WORKSPACE_PKG_TEST_GLOBS`) maps each workspace key to a source glob — `robot` →
`robot/ros_ws/src/**/<pkg>/test`, `sim` → `simulation/**/<pkg>/test`. Add a `sim:` (or a
new `gcs:`) workspace to the YAML, adding the glob for a new tree in `tests/harness/discovery.py`:

```yaml
# tests/colcon_unit_test_packages.yaml
sim:
  packages:
    - <isaac_extension_name>   # → simulation/**/<ext>/test collected directly
```

`airstack test -m unit` discovers them automatically — no changes to `pytest.ini`
needed.

## See also

- [`.agents/skills/add-unit-tests`](../../../../.agents/skills/add-unit-tests/SKILL.md) — step-by-step agent workflow
- [System tests](../../../../tests/README.md) — full Docker-stack integration tests
- [CI/CD](ci_cd.md) — pipeline overview and ephemeral runner architecture
- [Testing frameworks](testing_frameworks.md) — `colcon test`, ament linters
