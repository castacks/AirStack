# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Proxy: registers natnet_ros2 Python unit tests for pytest tests/."""

from conftest import register_unit_tests, repo_path

register_unit_tests(
    globals(),
    repo_path("robot/ros_ws/src/perception/natnet_ros2/test"),
    "test_natnet_ros2.py",
)
