# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Hermetic tests for the bridge's plan-change rule (no ROS required)."""
import pytest

from global_plan_navigate_bridge.plan_key import plan_changed, plan_key

pytestmark = pytest.mark.unit


def test_empty_plan_has_no_key():
    assert plan_key([]) is None


def test_key_is_count_and_final_pose():
    assert plan_key([(0, 0, 0), (1.0, 2.0, 3.0)]) == (2, (1.0, 2.0, 3.0))


def test_republished_identical_plan_is_not_a_change():
    k = plan_key([(0, 0, 0), (5, 0, 2)])
    assert not plan_changed(k, k, 0.5)


def test_small_goal_jitter_is_not_a_change():
    a = plan_key([(0, 0, 0), (5.0, 0.0, 2.0)])
    b = plan_key([(0, 0, 0), (5.2, 0.1, 2.0)])
    assert not plan_changed(a, b, 0.5)


def test_moved_goal_is_a_change():
    a = plan_key([(0, 0, 0), (5.0, 0.0, 2.0)])
    b = plan_key([(0, 0, 0), (9.0, 0.0, 2.0)])
    assert plan_changed(a, b, 0.5)


def test_pose_count_change_is_a_change():
    a = plan_key([(0, 0, 0), (5.0, 0.0, 2.0)])
    b = plan_key([(0, 0, 0), (2, 0, 2), (5.0, 0.0, 2.0)])
    assert plan_changed(a, b, 0.5)


def test_empty_transitions_are_changes():
    k = plan_key([(1, 1, 1)])
    assert plan_changed(k, None, 0.5)
    assert plan_changed(None, k, 0.5)
    assert not plan_changed(None, None, 0.5)
