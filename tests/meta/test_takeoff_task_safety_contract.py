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
    assert "altitude overshoot limit exceeded" in source
    assert "takeoff_max_altitude_overshoot: 0.3" in config
    assert "vertical speed limit exceeded" in source
    assert "takeoff_max_vertical_speed: 1.5" in config
    assert "confirm_tracking_point_hold()" in source
    assert "preflight hold not confirmed" in source
    assert "preflight_hold_confirmation_samples: 3" in config


def test_landing_recovery_anchors_at_current_odometry_not_prior_tracking_point():
    source = (ROOT / "robot/ros_ws/src/local/planners/takeoff_landing_planner/src/"
              "takeoff_landing_task.cpp").read_text()
    landing = source[source.index("void TakeoffLandingTaskNode::land_execute"):]
    trajectory_start = landing[:landing.index("RCLCPP_INFO", landing.index("TakeoffTrajectory land_traj"))]
    assert "start_point.pose.position.z = robot_odom_.pose.pose.position.z" in trajectory_start
    assert "start_point = tracking_point_odom_" not in trajectory_start
    assert "Request::ROBOT_POSE" in trajectory_start


def test_all_landing_entry_points_use_bounded_current_odometry_trajectory():
    planner_dir = (ROOT / "robot/ros_ws/src/local/planners/"
                   "takeoff_landing_planner/src")
    task_source = (planner_dir / "takeoff_landing_task.cpp").read_text()
    legacy_source = (planner_dir / "takeoff_landing_planner.cpp").read_text()

    assert "TakeoffTrajectory(-10000" not in task_source
    assert "TakeoffTrajectory(-10000" not in legacy_source
    assert "landing_start.pose.position.z = robot_odom.pose.pose.position.z" in legacy_source
    assert "const double landing_descent = -(landing_start.pose.position.z + 1.0)" in legacy_source
    assert "landing_trajectory.get_trajectory(landing_start)" in legacy_source
