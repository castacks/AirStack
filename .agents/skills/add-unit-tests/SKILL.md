---
name: add-unit-tests
description: Add Python or C++ unit tests to an AirStack ROS 2 package. Covers the co-location pattern (test source in package/test/), the thin proxy that makes tests discoverable by pytest tests/ and airstack test -m unit, and how to extend the pattern to sim and GCS modules.
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

Unit tests follow a **co-location + proxy** pattern:

```
robot/ros_ws/src/<layer>/<package>/
├── src/                        # production source (Python or C++)
├── test/
│   ├── test_<name>.py          # ← unit test SOURCE (canonical location)
│   ├── test_<name>.cpp         # ← C++ gtest SOURCE (optional)
│   └── fake_<name>.hpp         # ← C++ test doubles (optional)
└── CMakeLists.txt              # wires ament_add_gtest under BUILD_TESTING

tests/robot/<layer>/<package>/
└── test_<name>.py              # ← thin PROXY (re-exports tests from above)
```

The **proxy** is a one-file shim that loads the real test module with `importlib`
and re-exports every `test_*` function. This means:

| Invocation | What runs |
|---|---|
| `pytest tests/ -m unit` | Proxy in `tests/robot/` → loads real test from package |
| `airstack test -m unit` | Same path |
| CI `system-tests.yml` (PR open / approved) | Same path via `pytest tests/` |
| `colcon test --packages-select <pkg>` | Real test in `package/test/` directly |

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

### 3. Write the thin proxy in tests/robot/

Create `tests/robot/<layer>/<package>/test_<name>.py`:

```python
# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Proxy: re-exposes <package> unit tests from the package source tree.

Unit test logic lives co-located with the package source (ROS 2 / colcon convention):
  robot/ros_ws/src/<layer>/<package>/test/test_<name>.py

This file makes those tests discoverable by ``pytest tests/`` (CI) and
``airstack test -m unit`` without any changes to the CI workflow.
"""
import importlib.util
import sys
from pathlib import Path

_repo_root = Path(__file__).resolve().parents[N]   # adjust N so this resolves to repo root
_pkg_test = _repo_root / "robot/ros_ws/src/<layer>/<package>/test"
_real_file = _pkg_test / "test_<name>.py"

# If the test imports from a package module, ensure the package root is on sys.path.
# Example: _pkg_root = _pkg_test.parent; sys.path.insert(0, str(_pkg_root))

# Load the real module under a unique name to avoid the circular import that
# would occur if we used `from test_<name> import *` (this file has the same
# name, and pytest adds its directory to sys.path at collection time).
_spec = importlib.util.spec_from_file_location("_<package>_unit_tests", _real_file)
_real = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_real)

# Re-export every test_* symbol so pytest collects them from this proxy.
for _name in dir(_real):
    if _name.startswith("test_"):
        globals()[_name] = getattr(_real, _name)
```

**Counting `parents[N]` to reach the repo root:**

| Proxy location | `parents[N]` for repo root |
|---|---|
| `tests/robot/<layer>/<package>/` | `parents[4]` |
| `tests/sim/<tool>/` | `parents[3]` |
| `tests/gcs/<package>/` | `parents[3]` |

### 4. Ensure the tests/ directory structure exists

```bash
mkdir -p tests/robot/<layer>/<package>
touch tests/robot/<layer>/<package>/__init__.py   # only if needed for conftest path discovery
```

The READMEs in `tests/robot/behavior/`, `tests/robot/global/`, etc. describe the
purpose of each layer mirror. Update the layer README when you add a new package.

### 5. Run locally to verify

```bash
# From repo root — no container needed
cd tests
pytest -m unit -v
# or
airstack test -m unit -v
```

All 14+ existing tests plus your new ones should pass. The proxy output shows:
```
robot/<layer>/<package>/test_<name>.py::test_my_function_basic
  <- ../robot/ros_ws/src/<layer>/<package>/test/test_<name>.py  PASSED
```

### 6. CI picks it up automatically

Unit tests are discovered by `pytest tests/` and run as part of `system-tests.yml`
(triggered on PR open) — no changes to CI needed.

---

## Step-by-Step: Adding a C++ gtest

C++ tests don't use the proxy pattern — they live entirely within the package and
run exclusively via `colcon test`.

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
runs `colcon test` with `BUILD_TESTING=ON` for the robot container.

---

## Extending to sim and GCS

The same proxy pattern applies verbatim:

**Sim-side Python** (e.g. motive emulator protocol logic):
```
simulation/.../<tool>/test/test_<name>.py   ← source
tests/sim/<tool>/test_<name>.py             ← proxy (parents[3] = repo root)
```

**GCS modules**:
```
gcs/.../<pkg>/test/test_<name>.py           ← source
tests/gcs/<pkg>/test_<name>.py             ← proxy (parents[3] = repo root)
```

`pytest tests/ -m unit` discovers them through the proxy without any
pytest.ini or CI changes needed.

---

## Pattern Summary

| Concern | Answer |
|---|---|
| Where does test source live? | `<component>/…/<package>/test/` (co-located with the package) |
| Where does pytest discover tests? | `tests/robot/` (or `tests/sim/`, `tests/gcs/`) via thin proxy |
| How does the proxy avoid circular import? | `importlib.util.spec_from_file_location` with a unique module name |
| What mark do all unit tests use? | `@pytest.mark.unit` |
| What CI workflow runs them? | `system-tests.yml` — runs `pytest tests/` which includes unit tests |
| When does that workflow trigger? | PR opened, `/pytest` comment, `workflow_dispatch` |
| Do system tests (`liveliness`, etc.) run too? | No — `-m unit` filters to hermetic tests only |
| Does `colcon test` also run these? | Yes — Python tests in `package/test/` are discovered by colcon's pytest runner |
| Can I add pure C++ gtests? | Yes — `ament_add_gtest` in CMakeLists.txt, no proxy needed |

## Reference Implementations

| Package | Python test | What it covers |
|---|---|---|
| `natnet_ros2` | `robot/ros_ws/src/perception/natnet_ros2/test/test_natnet_ros2.py` | `VisionPoseConverterNode._canonical_quaternion` (ROS-stubbed) |
| `natnet_ros2` (C++) | `robot/ros_ws/src/perception/natnet_ros2/test/test_natnet_logic.cpp` | `build_covariance_6x6`, `negotiate()`, `INatNetClient` seam |
| `lidar_point_cloud_filter` | `robot/ros_ws/src/sensors/lidar_point_cloud_filter/test/test_validation_core.py` | Pure-numpy range validation rules |

Corresponding proxies: `tests/robot/perception/natnet_ros2/test_natnet_ros2.py`,
`tests/robot/sensors/lidar_point_cloud_filter/test_validation_core.py`.

## Files to Know

- `.github/workflows/system-tests.yml` — CI workflow (runs `pytest tests/` including unit tests)
- `tests/pytest.ini` — mark registration (`unit`, `build_docker`, etc.)
- `tests/robot/` — proxy layer mirroring `robot/ros_ws/src/`
- `tests/sim/` — proxy layer for sim-side code (future)
- `tests/gcs/` — proxy layer for GCS code (future)
- `tests/README.md` — full test harness reference
