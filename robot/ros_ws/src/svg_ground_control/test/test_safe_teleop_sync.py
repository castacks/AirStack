"""safe_teleop keeps its ``max_speed_mps`` equal to the commander's
``teleop_max_speed_mps`` — whichever side is changed.

Constructs the real ``SafeTeleopNode`` with parameter overrides (no pad, no
commander) and drives it through the rclpy parameter API (what ``ros2 param
set`` and the basestation panel's ``set_parameters`` use) and through its
commander-status subscription (a ``std_msgs/String`` JSON snapshot fed
straight to the callback). Skipped where rclpy is unavailable.
"""

from __future__ import annotations

import json

import pytest

rclpy = pytest.importorskip("rclpy")

from rcl_interfaces.msg import SetParametersResult  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from svg_ground_control.safe_teleop.teleop_node import SafeTeleopNode  # noqa: E402


@pytest.fixture(scope="module", autouse=True)
def _rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_teleop(**params) -> SafeTeleopNode:
    overrides = [Parameter(k, value=v) for k, v in params.items()]
    return SafeTeleopNode(parameter_overrides=overrides)


def snapshot(teleop_max_speed_mps) -> String:
    """A commander status message carrying just what the pad reads."""
    return String(data=json.dumps({
        "stamp": 1.0, "node": "/swarm_commander",
        "tuning": {"teleop_max_speed_mps": teleop_max_speed_mps,
                   "goal_accel_mps2": 3.0, "goal_settle_s": 0.3},
    }))


def test_max_speed_is_live() -> None:
    node = make_teleop(max_speed_mps=2.0)
    try:
        assert node.mapper.max_speed == 2.0
        results = node.set_parameters([Parameter("max_speed_mps", value=3.0)])
        assert results[0].successful, results[0].reason
        # The mapper scales the next stick reading with this.
        assert node.mapper.max_speed == 3.0
        assert node.get_parameter("max_speed_mps").value == 3.0
    finally:
        node.destroy_node()


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf")])
def test_max_speed_rejects_non_positive_or_non_finite(bad: float) -> None:
    node = make_teleop(max_speed_mps=2.0)
    try:
        results = node.set_parameters([Parameter("max_speed_mps", value=bad)])
        assert not results[0].successful
        assert "max_speed_mps" in results[0].reason
        assert node.mapper.max_speed == 2.0
        assert node.get_parameter("max_speed_mps").value == 2.0
    finally:
        node.destroy_node()


def test_wiring_parameters_are_refused_at_runtime() -> None:
    node = make_teleop(max_speed_mps=2.0)
    try:
        results = node.set_parameters([Parameter("forward_axis", value=2)])
        assert not results[0].successful
        assert "relaunch" in results[0].reason
    finally:
        node.destroy_node()


def test_follows_the_commander_snapshot() -> None:
    """`ros2 param set /swarm_commander teleop_max_speed_mps` reaches the pad."""
    node = make_teleop(max_speed_mps=2.0)
    try:
        node.commander_status_callback(snapshot(3.5))
        assert node.mapper.max_speed == 3.5
        assert node.get_parameter("max_speed_mps").value == 3.5
        assert node.commander_max_speed == 3.5
        # Adopting the commander's value is not pushed back to it.
        assert node._pending_push is None
        # Same value again: nothing to do.
        node.commander_status_callback(snapshot(3.5))
        assert node.mapper.max_speed == 3.5
    finally:
        node.destroy_node()


@pytest.mark.parametrize("bad", [0.0, -2.0, "fast", None, True])
def test_ignores_a_bad_snapshot_value(bad) -> None:
    node = make_teleop(max_speed_mps=2.0)
    try:
        node.commander_status_callback(snapshot(bad))
        node.commander_status_callback(String(data="not json"))
        node.commander_status_callback(String(data=json.dumps({"cbf": {}})))
        assert node.mapper.max_speed == 2.0
    finally:
        node.destroy_node()


def test_sync_can_be_switched_off() -> None:
    node = make_teleop(max_speed_mps=2.0, sync_max_speed=False)
    try:
        assert node.commander_params is None
        # Still live locally; nothing is pushed anywhere.
        results = node.set_parameters([Parameter("max_speed_mps", value=3.0)])
        assert results[0].successful
        assert node.mapper.max_speed == 3.0
        assert node._pending_push is None
    finally:
        node.destroy_node()


def test_local_set_is_pushed_to_the_commander_when_it_is_up() -> None:
    """A commander is running (its set_parameters service answers): the pad's
    new cap goes to it, so the two stay equal from this side too."""
    node = make_teleop(max_speed_mps=2.0)
    received = []

    def serve(request, response):
        received.extend(request.parameters)
        response.results = [SetParametersResult(successful=True)]
        return response

    commander = rclpy.create_node("fake_commander")
    server = commander.create_service(SetParameters, "/swarm_commander/set_parameters", serve)
    try:
        # Let the client discover the server before setting the parameter.
        for _ in range(50):
            if node.commander_params.service_is_ready():
                break
            rclpy.spin_once(commander, timeout_sec=0.1)
        assert node.commander_params.service_is_ready()
        results = node.set_parameters([Parameter("max_speed_mps", value=3.0)])
        assert results[0].successful
        assert node._pending_push is not None
        for _ in range(100):
            rclpy.spin_once(commander, timeout_sec=0.05)
            rclpy.spin_once(node, timeout_sec=0.05)
            if received and node._pending_push is None:
                break
        assert [(p.name, p.value.double_value) for p in received] == [("teleop_max_speed_mps", 3.0)]
        assert node._pending_push is None
    finally:
        commander.destroy_service(server)
        commander.destroy_node()
        node.destroy_node()
