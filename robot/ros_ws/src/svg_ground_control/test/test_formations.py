"""Tests for named formation profiles (single-command swarm re-targeting).

Constructs the real ``SwarmCommander`` with parameter overrides (no launch
file / interfaces needed) and drives ``formation_callback`` directly.
"""

from __future__ import annotations

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from svg_ground_control.swarm_commander import SwarmCommander  # noqa: E402

TRIANGLE = [0.0, 1.5, 1.2, -1.3, -0.8, 1.2, 1.3, -0.8, 1.2]
LINE = [-1.5, 0.0, 1.2, 0.0, 0.0, 1.2, 1.5, 0.0, 1.2]


@pytest.fixture(scope="module", autouse=True)
def _rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_commander(**params) -> SwarmCommander:
    overrides = [Parameter(k, value=v) for k, v in params.items()]
    return SwarmCommander(parameter_overrides=overrides)


def test_profiles_parsed_from_parameters() -> None:
    node = make_commander(
        drone_names=["drone_1", "drone_2", "drone_3"],
        scenario="goal",
        formation_profiles="line, triangle",
        formation_line=LINE,
        formation_triangle=TRIANGLE,
    )
    try:
        assert set(node.formations) == {"line", "triangle"}
        assert node.formations["triangle"].shape == (3, 3)
        assert np.allclose(node.formations["line"][2], [1.5, 0.0, 1.2])
    finally:
        node.destroy_node()


def test_formation_command_retargets_all_goals() -> None:
    node = make_commander(
        drone_names=["drone_1", "drone_2", "drone_3"],
        scenario="goal",
        formation_profiles="triangle",
        formation_triangle=TRIANGLE,
    )
    try:
        node.formation_callback(String(data="triangle"))
        assert np.allclose(
            node.scenario.goals, np.array(TRIANGLE).reshape(3, 3))
    finally:
        node.destroy_node()


def test_external_drone_goal_left_alone() -> None:
    node = make_commander(
        drone_names=["drone_1", "drone_2", "drone_3"],
        scenario="goal",
        external_drones="drone_3",
        formation_profiles="triangle",
        formation_triangle=TRIANGLE,
    )
    try:
        before = node.scenario.goals[2].copy()
        node.formation_callback(String(data="triangle"))
        assert np.allclose(node.scenario.goals[0], TRIANGLE[0:3])
        assert np.allclose(node.scenario.goals[1], TRIANGLE[3:6])
        assert np.allclose(node.scenario.goals[2], before)  # never commanded
    finally:
        node.destroy_node()


def test_unknown_profile_is_ignored() -> None:
    node = make_commander(
        drone_names=["drone_1", "drone_2", "drone_3"],
        scenario="goal",
        formation_profiles="line",
        formation_line=LINE,
    )
    try:
        before = node.scenario.goals.copy()
        node.formation_callback(String(data="does_not_exist"))
        assert np.allclose(node.scenario.goals, before)
    finally:
        node.destroy_node()


def test_wrong_length_profile_rejected() -> None:
    with pytest.raises(ValueError, match="formation_bad needs 9 values"):
        make_commander(
            drone_names=["drone_1", "drone_2", "drone_3"],
            scenario="goal",
            formation_profiles="bad",
            formation_bad=[1.0, 2.0, 3.0],
        )


def test_next_cycles_profiles_in_config_order_and_wraps() -> None:
    node = make_commander(
        drone_names=["drone_1", "drone_2", "drone_3"],
        scenario="goal",
        formation_profiles="line, triangle",
        formation_line=LINE,
        formation_triangle=TRIANGLE,
    )
    try:
        for expected in (LINE, TRIANGLE, LINE):   # wraps after the last
            node.formation_callback(String(data="next"))
            assert np.allclose(
                node.scenario.goals, np.array(expected).reshape(3, 3))
    finally:
        node.destroy_node()


def test_explicit_profile_reanchors_the_cycle() -> None:
    node = make_commander(
        drone_names=["drone_1", "drone_2", "drone_3"],
        scenario="goal",
        formation_profiles="line, triangle",
        formation_line=LINE,
        formation_triangle=TRIANGLE,
    )
    try:
        node.formation_callback(String(data="triangle"))  # anchor at #2
        node.formation_callback(String(data="next"))      # -> wraps to #1
        assert np.allclose(node.scenario.goals, np.array(LINE).reshape(3, 3))
    finally:
        node.destroy_node()


def test_next_reserved_as_profile_name() -> None:
    with pytest.raises(ValueError, match='"next" is reserved'):
        make_commander(
            drone_names=["drone_1", "drone_2", "drone_3"],
            scenario="goal",
            formation_profiles="next",
            formation_next=LINE,
        )


def test_profiles_on_non_goal_scenario_are_ignored() -> None:
    # hover has no set_goal: construction must succeed, profiles inert.
    node = make_commander(
        drone_names=["drone_1", "drone_2", "drone_3"],
        scenario="hover",
        formation_profiles="line",
        formation_line=LINE,
    )
    try:
        assert "line" in node.formations
    finally:
        node.destroy_node()
