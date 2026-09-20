"""Tests for the commander's runtime-tunable ``cbf_alpha`` and its status
snapshot (``build_status`` / ``status_topic``).

Constructs the real ``SwarmCommander`` with parameter overrides — no launch
file and no running interfaces — and drives it through the same rclpy
parameter API that ``ros2 param set`` and the Foxglove basestation panel's
``set_parameters`` service use. Skipped where rclpy is unavailable.
"""

from __future__ import annotations

import json

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.parameter import Parameter  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from svg_ground_control.swarm_commander import FlightState, SwarmCommander  # noqa: E402


@pytest.fixture(scope="module", autouse=True)
def _rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_commander(**params) -> SwarmCommander:
    """Construct the commander with parameter overrides.

    The commander's defaults (hover_positions, drone_position_offsets) are
    sized for its default three drones, so tests keep that roster.
    """
    overrides = [Parameter(k, value=v) for k, v in params.items()]
    return SwarmCommander(parameter_overrides=overrides)


# ---------------------------------------------------------------- cbf_alpha

def test_cbf_alpha_applies_at_runtime() -> None:
    node = make_commander(cbf_alpha=2.5)
    try:
        assert node.cbf_alpha == 2.5
        results = node.set_parameters([Parameter("cbf_alpha", value=4.0)])
        assert results[0].successful, results[0].reason
        # The control loop reads self.cbf_alpha every tick — this is the
        # value the next filter_velocities call will get.
        assert node.cbf_alpha == 4.0
        assert node.get_parameter("cbf_alpha").value == 4.0
    finally:
        node.destroy_node()


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf")])
def test_cbf_alpha_rejects_non_positive_or_non_finite(bad: float) -> None:
    node = make_commander(cbf_alpha=2.5)
    try:
        results = node.set_parameters([Parameter("cbf_alpha", value=bad)])
        assert not results[0].successful
        assert "cbf_alpha" in results[0].reason
        # Rejected -> nothing changed, neither the stored param nor the gain.
        assert node.cbf_alpha == 2.5
        assert node.get_parameter("cbf_alpha").value == 2.5
    finally:
        node.destroy_node()


def test_other_parameters_still_settable() -> None:
    # The validation callback must not veto unrelated parameters.
    node = make_commander()
    try:
        results = node.set_parameters([Parameter("hover_kp", value=1.5)])
        assert results[0].successful
    finally:
        node.destroy_node()


# ------------------------------------------------------------ status snapshot

def test_status_is_json_and_reports_gains_and_drones() -> None:
    node = make_commander(
        drone_names=["drone_1", "drone_2", "drone_3"],
        drone_modes="sim,real,sim",
        external_drones="drone_3",
        cbf_alpha=3.0, cbf_safety_radius_m=0.6, cbf_max_speed_mps=1.1,
    )
    try:
        status = node.build_status()
        encoded = json.dumps(status, allow_nan=False)   # what publish_status sends
        decoded = json.loads(encoded)

        assert decoded["scenario"] == "hover"
        assert decoded["mission_active"] is False
        assert decoded["mission_ever_started"] is False
        assert decoded["fence_breached"] is False
        assert decoded["last_command"] is None
        assert decoded["command_seq"] == 0
        assert decoded["cbf"] == {
            "alpha": 3.0, "safety_radius_m": 0.6, "max_speed_mps": 1.1,
            "external_velocity_gain": 1.0, "active": [], "emergency": False,
        }

        drones = {d["name"]: d for d in decoded["drones"]}
        assert set(drones) == {"drone_1", "drone_2", "drone_3"}
        assert drones["drone_2"]["mode"] == "real"
        assert drones["drone_3"]["role"] == "external"
        assert drones["drone_3"]["commanded"] is False
        for d in drones.values():
            # No odometry yet: honest nulls, never fabricated numbers.
            assert d["state"] == "IDLE"
            assert d["position"] is None
            assert d["speed_mps"] is None
            assert d["odom_fresh"] is False
            assert d["odom_age_s"] is None
            assert d["robot_command"] is None
    finally:
        node.destroy_node()


def test_status_position_is_world_frame_and_rounded() -> None:
    import numpy as np
    from nav_msgs.msg import Odometry

    node = make_commander(
        drone_position_offsets=[-2.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    try:
        odom = Odometry()
        odom.pose.pose.position.x = 0.123456
        odom.pose.pose.position.y = -0.5
        odom.pose.pose.position.z = 1.2
        odom.twist.twist.linear.x = 0.3
        odom.twist.twist.linear.y = 0.4
        node.odometry_callback(node.drones[0], odom)

        d1 = node.build_status()["drones"][0]
        # offset (-2, 0, 0) applied, 3 decimals — and the offset itself is
        # published so the panel can work in the same frame.
        assert d1["position"] == [-1.877, -0.5, 1.2]
        assert d1["position_offset"] == [-2.0, 0.0, 0.0]
        assert d1["speed_mps"] == pytest.approx(0.5, abs=1e-3)
        assert d1["odom_fresh"] is True
        assert d1["odom_age_s"] is not None and d1["odom_age_s"] >= 0.0

        # A non-finite estimate reads as "no position" rather than breaking JSON.
        node.drones[0].position = np.array([float("nan"), 0.0, 0.0])
        status = node.build_status()
        assert status["drones"][0]["position"] is None
        json.dumps(status, allow_nan=False)
    finally:
        node.destroy_node()


def test_status_records_lifecycle_outcomes() -> None:
    node = make_commander()
    try:
        # Start before takeoff: rejected, and the rejection is on record.
        resp = node.handle_start(Trigger.Request(), Trigger.Response())
        assert resp.success is False
        status = node.build_status()
        assert status["command_seq"] == 1
        last = status["last_command"]
        assert last["name"] == "start" and last["success"] is False
        assert "not all drones holding" in last["message"]
        assert status["mission_active"] is False

        # Pretend every commanded drone is holding, then start: accepted.
        for d in node.drones:
            d.state = FlightState.ACTIVE
        resp = node.handle_start(Trigger.Request(), Trigger.Response())
        assert resp.success is True
        status = node.build_status()
        assert status["command_seq"] == 2
        assert status["last_command"]["name"] == "start"
        assert status["last_command"]["success"] is True
        assert status["mission_active"] is True
        assert status["mission_started_at"] is not None
        assert all(d["state"] == "ACTIVE" for d in status["drones"])

        # Hold: recorded too, mission stops.
        import numpy as np
        for d in node.drones:
            d.position = np.zeros(3)
        resp = node.handle_hold(Trigger.Request(), Trigger.Response())
        status = node.build_status()
        assert status["command_seq"] == 3
        assert status["last_command"]["name"] == "hold"
        assert status["mission_active"] is False
        assert status["mission_ever_started"] is True
        json.dumps(status, allow_nan=False)
    finally:
        node.destroy_node()


def test_status_publisher_can_be_disabled() -> None:
    node = make_commander(status_rate_hz=0.0)
    try:
        assert node.status_pub is None
        node.publish_status()   # no-op, must not raise
    finally:
        node.destroy_node()
