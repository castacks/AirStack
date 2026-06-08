# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Proxy: re-exposes validation_core unit tests from the package source tree.

Unit test logic lives co-located with its package (ROS 2 / colcon convention):
  robot/ros_ws/src/sensors/lidar_point_cloud_filter/test/test_validation_core.py

This file makes those tests discoverable by ``pytest tests/`` (CI) and
``airstack test -m unit`` without any changes to the CI workflow.
Run ``colcon test --packages-select lidar_point_cloud_filter`` to also execute
the ament linters.
"""

from conftest import reexport_unit_tests, repo_path

reexport_unit_tests(
    globals(),
    repo_path("robot/ros_ws/src/sensors/lidar_point_cloud_filter/test"),
    "test_validation_core.py",
)
