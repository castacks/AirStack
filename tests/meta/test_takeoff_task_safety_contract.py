# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Static safety contract for the public TakeoffTask implementation."""
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

ROOT = Path(__file__).parents[2]


def test_takeoff_anchors_at_current_odometry_and_aborts_horizontal_runaway():
    source = (ROOT / "robot/ros_ws/src/local/planners/takeoff_landing_planner/src/"
              "takeoff_landing_task.cpp").read_text()
    config = (ROOT / "robot/ros_ws/src/local/planners/takeoff_landing_planner/config/"
              "takeoff_landing_planner.yaml").read_text()
    assert "start_point.pose.position.x = robot_odom_.pose.pose.position.x" in source
    assert "takeoff_max_horizontal_displacement_" in source
    assert "horizontal displacement limit exceeded" in source
    assert "takeoff_max_horizontal_displacement: 0.3" in config
