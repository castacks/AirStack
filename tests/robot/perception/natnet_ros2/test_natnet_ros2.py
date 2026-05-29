# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Proxy: re-exposes natnet_ros2 unit tests from the package source tree.

Unit test logic lives co-located with its package (ROS 2 / colcon convention):
  robot/ros_ws/src/perception/natnet_ros2/test/test_natnet_ros2.py

This file makes those tests discoverable by ``pytest tests/`` (CI) and
``airstack test -m unit`` without any changes to the CI workflow.
Run ``colcon test --packages-select natnet_ros2`` to also execute the C++
gtests and ament linters.
"""

import importlib.util
import sys
from pathlib import Path

_repo_root = Path(__file__).resolve().parents[4]
_pkg_test = _repo_root / "robot/ros_ws/src/perception/natnet_ros2/test"
_real_file = _pkg_test / "test_natnet_ros2.py"

# Load the real module under a unique name to avoid the circular-import that
# would occur if we used `from test_natnet_ros2 import *` (this file has the
# same name and pytest adds its directory to sys.path at collection time).
_spec = importlib.util.spec_from_file_location("_natnet_ros2_unit_tests", _real_file)
_real = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_real)

# Re-export every test_* symbol so pytest collects them from this proxy.
for _name in dir(_real):
    if _name.startswith("test_"):
        globals()[_name] = getattr(_real, _name)
