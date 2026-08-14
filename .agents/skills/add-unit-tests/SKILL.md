---
name: add-unit-tests
description: Add Python or C++ unit tests to an AirStack ROS 2 package. Covers the co-location pattern (test source in package/test/), registering the package in colcon_unit_test_packages.yaml so pytest tests/ and airstack test -m unit collect it, and how to extend to sim and GCS modules.
license: MIT
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Add Unit Tests to an AirStack Module

## When to Use

Use this skill when:

- Adding Python unit tests for a ROS 2 package (perception, sensors, local, global, behavior, interface)
- Adding C++ unit tests (`gtest`) to a package already using `ament_cmake`
- Extending unit tests to sim-side Python (`tests/sim/`) or GCS modules (`tests/gcs/`)
- Verifying that `airstack test -m unit` and `pytest tests/` (CI) pick up your new tests

For system tests (full Docker stack, sim, sensors, takeoff/hover/land) see the
`run-system-tests` skill instead.

## Architecture Overview

Unit test **source lives co-located with its package** (ROS 2 / colcon convention).
`tests/colcon_unit_test_packages.yaml` lists which packages have unit tests, and
`pytest tests/` collects them from there — you only edit files under the package itself.

```
robot/ros_ws/src/<layer>/<package>/
├── src/                        # production source (Python or C++)
├── test/
│   ├── test_<name>.py          # ← unit test SOURCE (collected directly)
│   ├── test_<name>.cpp         # ← C++ gtest SOURCE (optional)
│   └── fake_<name>.hpp         # ← C++ test doubles (optional)
└── CMakeLists.txt              # wires ament_add_gtest under BUILD_TESTING

tests/colcon_unit_test_packages.yaml   # ← list the package here (single source of truth)
```

`tests/conftest.py` reads the YAML, resolves each listed package to its `test/` dir,
and injects the non-linter `test_*.py` files into collection under
`--import-mode=importlib` (set in `tests/pytest.ini`). Each collected item is
auto-tagged `@pytest.mark.unit` by path, so `-m unit` selects it. ament lint files
(`test_copyright.py`, etc.) are excluded — they run under `colcon test`. This means:

| Invocation | What runs |
|---|---|
| `pytest tests/ -m unit` | Package `test/test_*.py`, collected directly from source |
| `airstack test -m unit` | Same path |
| CI `system-tests.yml` (PR open / approved) | Same path via `pytest tests/` |
| `colcon test --packages-select <pkg>` | Real test in `package/test/` (incl. linters + C++) |

## Step-by-Step: Adding a Python Unit Test

### 1. Identify pure-Python logic to test

Good candidates are functions/classes with **no ROS or hardware dependencies**:
- Pure math / geometry helpers
- Protocol parsers
- Data-structure converters
- Any function that takes plain Python types and returns plain Python types

If the code imports ROS types, stub them out at the import boundary
(see `test_natnet_ros2.py` for the `sys.modules` stub pattern).

### 2. Write the test source in the package

Create `robot/ros_ws/src/<layer>/<package>/test/test_<name>.py`:

```python
# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Unit tests for <package> — <what is covered>."""

import sys
from pathlib import Path
import pytest

# Add the package src/ dir so the production module is importable
# without colcon installing the package first.
_src = Path(__file__).resolve().parent.parent / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))

from my_module import my_function  # noqa: E402


@pytest.mark.unit
def test_my_function_basic():
    assert my_function(1, 2) == 3
```

**Key points:**
- Always decorate with `@pytest.mark.unit` — this is the filter for fast runs.
- Compute paths relative to `__file__` (`parent.parent / "src"`) — never hardcode
  absolute paths.
- For packages with a Python module directory (`<pkg>/<pkg>/`), add the package
  root (`parent.parent`) to `sys.path` and import as
  `from <pkg>.<module> import ...`.
- If the code uses ROS types, stub `sys.modules` before importing:

```python
import sys
from unittest.mock import MagicMock

sys.modules.setdefault("rclpy", MagicMock())
sys.modules.setdefault("rclpy.node", MagicMock())
sys.modules.setdefault("geometry_msgs", MagicMock())
sys.modules.setdefault("geometry_msgs.msg", MagicMock())
# ... then import your module
```

For `rclpy.node.Node` subclasses use a real dummy base class instead of a
`MagicMock()` to ensure `__init_subclass__` fires and method bodies are defined
(see `test_natnet_ros2.py` for the full pattern).

### 3. Register the package in colcon_unit_test_packages.yaml

If the package isn't already listed, add it under the `robot` workspace in
[`tests/colcon_unit_test_packages.yaml`](../../../tests/colcon_unit_test_packages.yaml):

```yaml
robot:
  packages:
    - natnet_ros2
    - lidar_point_cloud_filter
    - <your_package>          # ← add here
  pytest_args: "-m not linter"
```

That's the whole registration. `conftest.py` globs
`robot/ros_ws/src/**/<your_package>/test`, collects its non-linter `test_*.py`, and marks
them `unit`. The test file must be self-contained: if it imports package code, set up
`sys.path` at the top of the test file (see `test_validation_core.py`, which inserts its
package root). Same YAML, different workspace key (`sim:`), for Isaac-extension unit tests.

### 5. Run locally to verify

```bash
# From repo root — no container needed
cd tests
pytest -m unit -v
# or
airstack test -m unit -v
```

All 14+ existing tests plus your new ones should pass. Collected items point straight
at the co-located source:
```
../robot/ros_ws/src/<layer>/<package>/test/test_<name>.py::test_my_function_basic PASSED
```

### 6. CI picks it up automatically

Unit tests are discovered by `pytest tests/` and run as part of `system-tests.yml`
(triggered on PR open) — no changes to CI needed.

---

## Step-by-Step: Adding a C++ gtest

C++ tests live entirely within the package and run exclusively via `colcon test`.

### 1. Write the test in `package/test/`

```cpp
// Copyright (c) 2024 Carnegie Mellon University
// MIT License - see LICENSE in the repository root for full text.
#include <gtest/gtest.h>
#include "my_package/my_header.hpp"

TEST(MyGroup, BasicCase) {
    EXPECT_EQ(my_function(1, 2), 3);
}
```

### 2. Wire `ament_add_gtest` in `CMakeLists.txt`

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_my_name test/test_my_name.cpp)
  target_include_directories(test_my_name PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>)
  # Link any production library targets here if needed:
  # target_link_libraries(test_my_name my_lib)
endif()
```

### 3. Add test depend in `package.xml`

```xml
<test_depend>ament_cmake_gtest</test_depend>
```

### 4. Build and run

```bash
# Inside the robot container:
docker exec airstack-robot-desktop-1 bash -c \
  "bws --cmake-args '-DBUILD_TESTING=ON' --packages-select <package>"
docker exec airstack-robot-desktop-1 bash -c \
  "colcon test --packages-select <package> --event-handlers console_direct+"
docker exec airstack-robot-desktop-1 bash -c \
  "colcon test-result --all"
```

The `build_packages` system test in CI (`tests/system/test_build_packages.py`) also
runs `colcon test` with `BUILD_TESTING=ON` for the robot container. Packages gated
there are listed in [`tests/colcon_unit_test_packages.yaml`](../../../tests/colcon_unit_test_packages.yaml)
— add your package under `robot.packages` when it has gtests or pytest tests in
`package/test/`.

---

## Extending to sim and GCS

The same mechanism applies — add the package under a workspace key in the YAML. The
workspace→source glob is defined in `tests/harness/discovery.py` (`_WORKSPACE_PKG_TEST_GLOBS`): `robot` →
`robot/ros_ws/src/**/<pkg>/test`, `sim` → `simulation/**/<pkg>/test`. Add a new workspace
key there (e.g. `gcs`) if you extend to a new tree.

```yaml
# tests/colcon_unit_test_packages.yaml
sim:
  packages:
    - <isaac_extension_name>   # → simulation/**/<ext>/test collected directly
```

---

## Pattern Summary

| Concern | Answer |
|---|---|
| Where does test source live? | `<component>/…/<package>/test/` (co-located with the package) |
| Where does pytest discover tests? | From the package `test/` dir listed in `colcon_unit_test_packages.yaml` |
| How are duplicate basenames handled? | `--import-mode=importlib` (set in `pytest.ini`) |
| What mark do all unit tests use? | `@pytest.mark.unit` (auto-applied by path in `conftest.py`) |
| What CI workflow runs them? | `system-tests.yml` — runs `pytest tests/` which includes unit tests |
| When does that workflow trigger? | PR opened, `/pytest` comment, `workflow_dispatch` |
| Do system tests (`liveliness`, etc.) run too? | No — `-m unit` filters to hermetic tests only |
| Does `colcon test` also run these? | Yes — Python tests in `package/test/` are discovered by colcon's pytest runner |
| Can I add pure C++ gtests? | Yes — `ament_add_gtest` in CMakeLists.txt |

## Reference Implementations

| Package | Python test | What it covers |
|---|---|---|
| `natnet_ros2` | `robot/ros_ws/src/perception/natnet_ros2/test/test_natnet_ros2.py` | `VisionPoseConverterNode._canonical_quaternion` (ROS-stubbed) |
| `natnet_ros2` (C++) | `robot/ros_ws/src/perception/natnet_ros2/test/test_natnet_logic.cpp` | `build_covariance_6x6`, `negotiate()`, `INatNetClient` seam |
| `lidar_point_cloud_filter` | `robot/ros_ws/src/sensors/lidar_point_cloud_filter/test/test_validation_core.py` | Pure-numpy range validation rules |

Both are collected from their package `test/` dir.

## Files to Know

- `.github/workflows/system-tests.yml` — CI workflow (runs `pytest tests/` including unit tests)
- `tests/pytest.ini` — mark registration + `--import-mode=importlib`
- `tests/colcon_unit_test_packages.yaml` — the package list driving unit-test collection
- `tests/conftest.py` — `unit_test_files()` / `pytest_configure` inject package tests; `pytest_itemcollected` auto-marks `unit`
- `tests/README.md` — full test harness reference
