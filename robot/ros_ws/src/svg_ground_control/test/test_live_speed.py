"""Live speed changes: `ros2 param set scenario_speed_mps` must take effect.

Regression for the long-standing "changing the speed does nothing": the
commander read scenario_speed_mps once at construction, so a parameter set
answered successful and changed nothing. Constructs the real node (as
test_exempt.py does) and drives its parameter callback.
"""

from __future__ import annotations

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.parameter import Parameter  # noqa: E402

from svg_ground_control.scenarios import GoalScenario  # noqa: E402
from svg_ground_control.swarm_commander import SwarmCommander  # noqa: E402


@pytest.fixture(scope="module", autouse=True)
def _rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()


def make(**params):
    # The node's default drone_position_offsets is sized for three drones.
    params.setdefault("drone_position_offsets", [0.0] * (3 * len(params["drone_names"])))
    return SwarmCommander(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])


def test_param_set_scenario_speed_applies_live_in_goal_scenario():
    node = make(drone_names=["drone_1", "drone_2"], scenario="goal",
                hover_positions=[0.0, 0.0, 1.0, 1.0, 0.0, 1.0], scenario_speed_mps=0.6)
    try:
        assert node.scenario.nominal_speed == 0.6
        res = node.set_parameters([Parameter("scenario_speed_mps", value=1.0)])
        assert res[0].successful
        assert node.scenario.nominal_speed == 1.0
        np.testing.assert_allclose(node.scenario.speeds, [1.0, 1.0])
        # and it is what the drones are told to fly (far from the goal)
        v = node.scenario.nominal_velocity(np.array([[5.0, 0, 1.0], [6.0, 0, 1.0]]))
        np.testing.assert_allclose(np.linalg.norm(v, axis=1), [1.0, 1.0])
    finally:
        node.destroy_node()


def test_param_set_scenario_speed_applies_live_in_hover_scenario():
    node = make(drone_names=["drone_1"], scenario="hover", hover_positions=[0.0, 0.0, 1.0])
    try:
        node.set_parameters([Parameter("scenario_speed_mps", value=0.9)])
        assert node.scenario.nominal_speed == 0.9
    finally:
        node.destroy_node()


def test_unsupported_param_set_is_refused_with_reason():
    node = make(drone_names=["drone_1"], scenario="hover", hover_positions=[0.0, 0.0, 1.0])
    try:
        res = node.set_parameters([Parameter("scenario_seed", value=3)])
        assert not res[0].successful
        assert "relaunch" in res[0].reason
    finally:
        node.destroy_node()


def test_goal_approach_gain_param_and_live_update():
    node = make(drone_names=["drone_1"], scenario="goal", hover_positions=[0.0, 0.0, 1.0],
                scenario_speed_mps=1.2, goal_approach_gain=3.0)
    try:
        assert node.scenario.approach_gain == 3.0
        # 0.3 m from the goal: gain 3 -> 0.9 m/s; the old fixed 1.5 gave 0.45.
        v = node.scenario.nominal_velocity(np.array([[0.3, 0.0, 1.0]]))
        assert np.linalg.norm(v) == pytest.approx(0.9)
        node.set_parameters([Parameter("goal_approach_gain", value=1.5)])
        v = node.scenario.nominal_velocity(np.array([[0.3, 0.0, 1.0]]))
        assert np.linalg.norm(v) == pytest.approx(0.45)
    finally:
        node.destroy_node()


def test_goal_scenario_default_gain_unchanged():
    assert GoalScenario.DEFAULT_APPROACH_GAIN == 1.5
