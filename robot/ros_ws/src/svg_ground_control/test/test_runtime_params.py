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


# ------------------------------------------ teleop cap / goal law, live

def test_tuning_params_apply_at_runtime_and_show_in_status() -> None:
    """The three gains the panel's dropdown adds: live, and in the snapshot."""
    node = make_commander(teleop_max_speed_mps=2.0, goal_accel_mps2=3.0,
                          goal_settle_s=0.3)
    try:
        for name, value in (("teleop_max_speed_mps", 3.0),
                            ("goal_accel_mps2", 6.0), ("goal_settle_s", 0.0)):
            results = node.set_parameters([Parameter(name, value=value)])
            assert results[0].successful, results[0].reason
        assert node.teleop_max_speed == 3.0
        assert node.scenario.tracker.accel == 6.0
        assert node.scenario.tracker.settle == 0.0
        tuning = json.loads(json.dumps(node.build_status()))["tuning"]
        assert tuning["teleop_max_speed_mps"] == 3.0
        assert tuning["goal_accel_mps2"] == 6.0
        assert tuning["goal_settle_s"] == 0.0
    finally:
        node.destroy_node()


@pytest.mark.parametrize("name, bad", [
    ("teleop_max_speed_mps", 0.0), ("teleop_max_speed_mps", -1.0),
    ("teleop_max_speed_mps", float("nan")),
    ("goal_accel_mps2", 0.0), ("goal_accel_mps2", float("inf")),
    ("goal_settle_s", -0.1), ("goal_settle_s", float("nan")),
])
def test_tuning_params_reject_bad_values(name: str, bad: float) -> None:
    node = make_commander(teleop_max_speed_mps=2.0, goal_accel_mps2=3.0,
                          goal_settle_s=0.3)
    try:
        results = node.set_parameters([Parameter(name, value=bad)])
        assert not results[0].successful
        assert name in results[0].reason
        assert node.teleop_max_speed == 2.0
        assert node.scenario.tracker.accel == 3.0
        assert node.scenario.tracker.settle == 0.3
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
        # The speed / tracking gains the panel's gain row edits, and the
        # stick cap safe_teleop mirrors, are reported next to the CBF gains.
        assert decoded["tuning"] == {
            "teleop_max_speed_mps": 1.2, "goal_accel_mps2": 3.0,
            "goal_settle_s": 0.3, "scenario_speed_mps": 0.6,
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


# --------------------------------------------------------------- fence grid

def test_fence_grid_is_clipped_to_fence_and_world_aligned() -> None:
    from visualization_msgs.msg import Marker

    node = make_commander(
        fence_enabled=True,
        fence_min=[-3.0, -6.0, 0.0], fence_max=[5.0, 5.0, 3.0],   # cbf_sim.yaml
        fence_grid_cell_m=0.5,
    )
    try:
        m = node._fence_grid_marker(node.get_clock().now().to_msg())
        assert m is not None and m.type == Marker.LINE_LIST
        assert m.ns == "fence_grid"
        xs = sorted({p.x for p in m.points})
        ys = sorted({p.y for p in m.points})
        # Every line spans exactly the fence footprint on the fence floor...
        assert xs[0] == -3.0 and xs[-1] == 5.0
        assert ys[0] == -6.0 and ys[-1] == 5.0
        assert all(p.z == 0.0 for p in m.points)
        # ...and the lines sit on world multiples of the cell: 17 x-lines
        # (-3.0 .. 5.0) + 23 y-lines (-6.0 .. 5.0), two points each.
        assert len(m.points) == 2 * (17 + 23)
        assert len(m.colors) == len(m.points)
        assert 0.0 in xs and 0.0 in ys              # origin is on the grid
        # Whole-metre lines are brighter than half-metre ones.
        by_x = {}
        for pt, c in zip(m.points, m.colors):
            by_x.setdefault(pt.x, set()).add(round(c.a, 3))
        # x = 0.5 is a vertical minor line; x = 1.0 a major one. (Endpoints of
        # horizontal lines at x=-3/5 also carry their own line's colour.)
        assert min(by_x[0.5]) < max(by_x[1.0])

    finally:
        node.destroy_node()


def test_fence_grid_disabled_or_oversized_returns_none() -> None:
    off = make_commander(fence_enabled=True, fence_grid_cell_m=0.0)
    try:
        assert off._fence_grid_marker(off.get_clock().now().to_msg()) is None
    finally:
        off.destroy_node()
    # The default ±1000 m fence would be thousands of lines: skipped, not drawn.
    huge = make_commander(fence_enabled=True, fence_grid_cell_m=0.5)
    try:
        assert huge._fence_grid_marker(huge.get_clock().now().to_msg()) is None
    finally:
        huge.destroy_node()


# ---------------------------------------------------------------- name label

def test_marker_label_is_name_only_and_dark_on_white() -> None:
    import numpy as np
    from visualization_msgs.msg import Marker

    node = make_commander(drone_modes="sim,real,sim", external_drones="drone_3")
    try:
        for d in node.drones:
            d.position = np.zeros(3)
        published = []
        node.viz_pub.publish = published.append          # capture instead of sending
        node.publish_markers(node.get_clock().now())
        labels = [m for m in published[0].markers if m.ns == "label"]
        assert [m.text for m in labels] == ["drone_1", "drone_2", "drone_3"]
        for m in labels:
            assert m.type == Marker.TEXT_VIEW_FACING
            # Relative luminance < 0.5 -> Foxglove draws a white chip, not black.
            lum = 0.2126 * m.color.r + 0.7152 * m.color.g + 0.0722 * m.color.b
            assert lum < 0.5
            assert 0.0 < m.color.a < 1.0
    finally:
        node.destroy_node()


# ------------------------------------------------------------ drop counters

def test_status_reports_dds_reception_counters() -> None:
    from types import SimpleNamespace
    from nav_msgs.msg import Odometry

    node = make_commander()
    try:
        d1 = node.drones[0]
        # rmw_fastrtps supports the message_lost event -> measured counter.
        assert d1.odom_loss_counter == "dds"
        for _ in range(7):
            node.odometry_callback(d1, Odometry())
        # The DDS event delivers a cumulative total (and the change).
        node._on_odometry_lost(d1, SimpleNamespace(total_count=3, total_count_change=3))
        s = {d["name"]: d for d in node.build_status()["drones"]}
        assert s["drone_1"]["odom_rx_total"] == 7
        assert s["drone_1"]["odom_lost_total"] == 3
        assert s["drone_1"]["odom_loss_counter"] == "dds"
        assert s["drone_2"]["odom_rx_total"] == 0 and s["drone_2"]["odom_lost_total"] == 0
    finally:
        node.destroy_node()


# ------------------------------------------------------------ teleop fence

GEO = dict(fence_enabled=True, fence_min=[-4.0, -2.0, 0.0], fence_max=[4.0, 2.0, 3.0])
TELEOP_BOX = dict(teleop_fence_enabled=True,
                  teleop_fence_min=[-3.0, -1.5, 0.3], teleop_fence_max=[3.0, 1.5, 2.5])


def test_teleop_fence_must_lie_inside_the_geofence() -> None:
    with pytest.raises(ValueError, match="inside the geofence"):
        make_commander(**GEO, teleop_drones="drone_2", teleop_fence_enabled=True,
                       teleop_fence_min=[-5.0, -1.5, 0.3], teleop_fence_max=[3.0, 1.5, 2.5])
    with pytest.raises(ValueError, match="teleop_fence_min"):
        make_commander(teleop_fence_enabled=True,
                       teleop_fence_min=[1.0, -1.5, 0.3], teleop_fence_max=[0.0, 1.5, 2.5])
    # Without a geofence there is nothing to nest in: any box is fine.
    node = make_commander(fence_enabled=False, **TELEOP_BOX)
    node.destroy_node()


def test_keep_in_box_is_the_teleop_fence_for_teleop_drones_only() -> None:
    import numpy as np

    # Geofence in hold_all (latches, never clips) + teleop fence: only the
    # hand-flown drone gets a keep_in wall, at the teleop box.
    node = make_commander(**GEO, fence_behavior="hold_all", teleop_drones="drone_2",
                          **TELEOP_BOX)
    try:
        d1, d2, _ = node.drones
        assert node.keep_in_box(d1) is None
        lo, hi = node.keep_in_box(d2)
        np.testing.assert_allclose(lo, TELEOP_BOX["teleop_fence_min"])
        np.testing.assert_allclose(hi, TELEOP_BOX["teleop_fence_max"])
    finally:
        node.destroy_node()
    # Geofence in keep_in without a teleop fence: everyone gets the geofence.
    node = make_commander(**GEO, fence_behavior="keep_in", teleop_drones="drone_2")
    try:
        for d in node.drones[:2]:
            lo, hi = node.keep_in_box(d)
            np.testing.assert_allclose(lo, GEO["fence_min"])
            np.testing.assert_allclose(hi, GEO["fence_max"])
    finally:
        node.destroy_node()


def test_keep_in_clips_at_the_teleop_wall_with_a_braking_feedforward() -> None:
    import numpy as np

    node = make_commander(**GEO, fence_behavior="hold_all", teleop_drones="drone_2",
                          fence_keep_in_gain=2.0, fence_brake_accel_mps2=4.0,
                          **TELEOP_BOX)
    try:
        d2 = node.drones[1]
        d2.state = FlightState.ACTIVE
        # 1 m inside the teleop +x wall (still 2 m inside the geofence),
        # 3 m/s outward with the stick fully forward.
        d2.position = np.array([2.0, 0.0, 1.0])
        d2.velocity = np.array([3.0, 0.0, 0.0])
        stick = np.array([5.0, 0.0, 0.0])
        v, a = node.keep_in(d2, stick)
        assert v[0] < 3.0 and v[1] == 0.0 and v[2] == 0.0
        assert -4.0 <= a[0] < 0.0 and a[1] == 0.0 and a[2] == 0.0
        # Sticks inward: untouched, no braking (the far wall is 5 m away —
        # its own envelope still allows 4.6 m/s toward it).
        v, a = node.keep_in(d2, np.array([-3.0, 0.0, 0.0]))
        np.testing.assert_allclose(v, [-3.0, 0.0, 0.0])
        np.testing.assert_allclose(a, 0.0)
        # The scenario drone is bounded by the geofence only (hold_all: no clip).
        d1 = node.drones[0]
        d1.state = FlightState.ACTIVE
        d1.position = d2.position.copy()
        d1.velocity = d2.velocity.copy()
        v, a = node.keep_in(d1, stick)
        np.testing.assert_allclose(v, stick)
        np.testing.assert_allclose(a, 0.0)
        # The reference point is clamped into the teleop box as well.
        d2.ref = np.array([3.4, 0.0, 1.0])
        d2.applied = np.zeros(3)
        node.advance_reference(d2)
        assert d2.ref[0] <= TELEOP_BOX["teleop_fence_max"][0] + 1e-9
    finally:
        node.destroy_node()


def test_status_and_markers_carry_both_fences() -> None:
    import numpy as np

    node = make_commander(**GEO, fence_behavior="keep_in", teleop_drones="drone_2",
                          fence_keep_in_gain=3.0, fence_brake_accel_mps2=4.0,
                          **TELEOP_BOX)
    try:
        status = json.loads(json.dumps(node.build_status(), allow_nan=False))
        assert status["fence"]["behavior"] == "keep_in"
        assert status["fence"]["min"] == GEO["fence_min"]
        assert status["fence"]["keep_in_gain"] == 3.0
        assert status["fence"]["brake_accel_mps2"] == 4.0
        assert status["teleop_fence"] == {
            "enabled": True, "min": TELEOP_BOX["teleop_fence_min"],
            "max": TELEOP_BOX["teleop_fence_max"]}
        for d in node.drones:
            d.position = np.zeros(3)
        published = []
        node.viz_pub.publish = published.append
        node.publish_markers(node.get_clock().now())
        by_ns = {m.ns: m for m in published[0].markers}
        assert "fence" in by_ns and "teleop_fence" in by_ns
        box = by_ns["teleop_fence"]
        xs = [p.x for p in box.points]
        assert min(xs) == pytest.approx(-3.0) and max(xs) == pytest.approx(3.0)
        assert box.id != by_ns["fence"].id
    finally:
        node.destroy_node()


def test_keep_in_dynamics_are_live_and_gain_stays_positive() -> None:
    node = make_commander(fence_keep_in_gain=1.0, fence_brake_accel_mps2=4.0)
    try:
        results = node.set_parameters([Parameter("fence_keep_in_gain", value=3.0),
                                       Parameter("fence_brake_accel_mps2", value=0.0),
                                       Parameter("fence_margin_m", value=0.2)])
        assert all(r.successful for r in results), [r.reason for r in results]
        assert node.fence_keep_in_gain == 3.0
        assert node.fence_brake_accel == 0.0      # 0 = the plain barrier, allowed
        assert node.fence_margin == 0.2
        rejected = node.set_parameters([Parameter("fence_keep_in_gain", value=0.0)])
        assert not rejected[0].successful and "fence_keep_in_gain" in rejected[0].reason
        assert node.fence_keep_in_gain == 3.0
        # The boxes are geometry: startup only.
        rejected = node.set_parameters([Parameter("teleop_fence_enabled", value=True)])
        assert not rejected[0].successful
    finally:
        node.destroy_node()


# ------------------------------------------------------------ teleop output

def _capture(node, d):
    published = []
    d.cmd_pub = type("Pub", (), {"publish": staticmethod(published.append)})()
    return published


def test_teleop_trajectory_yaws_with_the_stick_and_holds_heading_when_centred() -> None:
    import numpy as np

    node = make_commander(drone_modes="real,real,real", teleop_drones="drone_2")
    try:
        d2 = node.drones[1]
        assert d2.output == "trajectory"
        d2.position = np.array([1.0, 2.0, 1.0])
        d2.velocity = np.zeros(3)
        yaw = 0.7
        d2.orientation = (0.0, 0.0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2)))
        d2.ref = d2.position.copy()
        now = node.get_clock().now()
        d2.last_teleop_time = now
        out = _capture(node, d2)

        # Yaw stick deflected: rotation ALL zero (w included, the message
        # default is w=1) so px4_interface takes the yaw-rate path.
        d2.teleop_yaw_rate = 0.8
        node.publish_command(d2, np.zeros(3), np.zeros(3), now)
        rot = out[-1].points[0].transforms[0].rotation
        assert rot.x == rot.y == rot.z == rot.w == 0.0
        assert out[-1].points[0].velocities[0].angular.z == pytest.approx(0.8)
        assert d2.teleop_yaw_hold is None

        # Stick centred: the measured heading is adopted and sent as an
        # absolute yaw, yaw rate zero, and it stays put afterwards.
        d2.teleop_yaw_rate = 0.0
        node.publish_command(d2, np.zeros(3), np.zeros(3), now)
        rot = out[-1].points[0].transforms[0].rotation
        assert 2 * np.arctan2(rot.z, rot.w) == pytest.approx(yaw)
        assert out[-1].points[0].velocities[0].angular.z == 0.0
        d2.orientation = (0.0, 0.0, float(np.sin(0.2)), float(np.cos(0.2)))   # drifted
        node.publish_command(d2, np.zeros(3), np.zeros(3), now)
        rot = out[-1].points[0].transforms[0].rotation
        assert 2 * np.arctan2(rot.z, rot.w) == pytest.approx(yaw)

        # Hand-over (reference re-seeded) drops the hold: the next centred
        # tick adopts the heading the drone has then.
        d2.ref = None
        node.advance_reference(d2)
        assert d2.teleop_yaw_hold is None and d2.teleop_profile is None
        node.publish_command(d2, np.zeros(3), np.zeros(3), now)
        rot = out[-1].points[0].transforms[0].rotation
        assert 2 * np.arctan2(rot.z, rot.w) == pytest.approx(0.4)

        # A scenario drone still gets its absolute heading (nose +X here).
        d1 = node.drones[0]
        d1.position = np.zeros(3)
        d1.ref = np.zeros(3)
        out1 = _capture(node, d1)
        node.publish_command(d1, np.zeros(3), np.zeros(3), now)
        rot = out1[-1].points[0].transforms[0].rotation
        assert rot.w == pytest.approx(1.0) and rot.z == pytest.approx(0.0)
    finally:
        node.destroy_node()


def test_reference_is_clamped_into_the_box_only_while_active() -> None:
    import numpy as np

    node = make_commander(**GEO, fence_behavior="keep_in", teleop_drones="drone_2",
                          **TELEOP_BOX)      # teleop floor at z = 0.3
    try:
        d2 = node.drones[1]
        d2.velocity = np.zeros(3)
        d2.applied = np.zeros(3)
        # Landing through the floor: the reference follows the drone down.
        d2.state = FlightState.LANDING
        d2.position = np.array([0.0, 0.0, 0.1])
        d2.ref = np.array([0.0, 0.0, 0.1])
        node.advance_reference(d2)
        assert d2.ref[2] == pytest.approx(0.1)
        # Same on the way up.
        d2.state = FlightState.ASCEND
        node.advance_reference(d2)
        assert d2.ref[2] == pytest.approx(0.1)
        # Flying: held inside the box.
        d2.state = FlightState.ACTIVE
        node.advance_reference(d2)
        assert d2.ref[2] == pytest.approx(0.3)
    finally:
        node.destroy_node()


def test_teleop_stick_is_ramped_with_feedforward_when_enabled() -> None:
    import numpy as np

    node = make_commander(teleop_drones="drone_2", teleop_accel_mps2=5.0,
                          teleop_max_speed_mps=8.0)
    try:
        d2 = node.drones[1]
        now = node.get_clock().now()
        d2.last_teleop_time = now
        d2.teleop_twist = np.array([8.0, 0.0, 0.0])
        d2.applied = np.zeros(3)
        v, a = node.teleop_command(d2, now)
        np.testing.assert_allclose(v, [5.0 * node.control_dt, 0, 0])
        np.testing.assert_allclose(a, [5.0, 0, 0])
        # Live off: the stick passes straight through.
        results = node.set_parameters([Parameter("teleop_accel_mps2", value=0.0)])
        assert results[0].successful
        v, a = node.teleop_command(d2, now)
        np.testing.assert_allclose(v, [8.0, 0, 0])
        np.testing.assert_allclose(a, 0.0)
    finally:
        node.destroy_node()


def test_teleop_ramp_reattaches_to_the_published_velocity_not_the_measured_one() -> None:
    import numpy as np

    node = make_commander(teleop_drones="drone_2", teleop_accel_mps2=5.0,
                          teleop_max_speed_mps=8.0)
    try:
        d2 = node.drones[1]
        now = node.get_clock().now()
        d2.last_teleop_time = now
        d2.teleop_twist = np.zeros(3)                  # sticks released
        d2.published = np.array([3.0, 0.0, 0.0])       # last command: 3 m/s forward
        # A leash pull just handed back the measured velocity (drone
        # sinking at 0.3 m/s, running at 4 m/s): the ramp must ignore it.
        d2.applied = np.array([4.0, 0.0, -0.3])
        v, a = node.teleop_command(d2, now)
        np.testing.assert_allclose(v, [3.0 - 5.0 * node.control_dt, 0.0, 0.0])
        assert a[2] == 0.0
    finally:
        node.destroy_node()


# ------------------------------------------------------------ shipped configs

def _commander_params(config_name: str) -> dict:
    import pathlib
    import yaml
    path = pathlib.Path(__file__).resolve().parents[1] / "config" / config_name
    return yaml.safe_load(path.read_text())["swarm_commander"]["ros__parameters"]


def test_squeeze_rc_intruder_config_flies_drone_3_by_hand() -> None:
    import numpy as np

    params = _commander_params("squeeze_rc_intruder.yaml")
    node = make_commander(**params)      # validates roles, fences, nesting
    try:
        d1, d2, d3 = node.drones
        assert (d1.role, d2.role, d3.role) == ("auto", "auto", "teleop")
        assert all(d.mode == "real" and d.output == "trajectory" for d in node.drones)
        assert node.cbf_exempt_names == {"drone_3"}
        # drone_3 is bounded by the teleop fence, the holders by the geofence.
        lo, hi = node.keep_in_box(d3)
        np.testing.assert_allclose(lo, params["teleop_fence_min"])
        np.testing.assert_allclose(hi, params["teleop_fence_max"])
        lo, hi = node.keep_in_box(d1)
        np.testing.assert_allclose(lo, params["fence_min"])
        # Its take-off point is intruder waypoint A, inside the teleop fence
        # and clear of both posts by more than the CBF keep-out.
        initial = node.scenario.initial_positions()
        a = np.array(params["squeeze_intruder_waypoints"][:3])
        np.testing.assert_allclose(initial[2], a)
        assert np.all(a > params["teleop_fence_min"]) and np.all(a < params["teleop_fence_max"])
        posts = np.array(params["squeeze_holder_positions"]).reshape(2, 3)
        assert np.linalg.norm(posts - a, axis=1).min() > 2 * params["cbf_safety_radius_m"]
        # The pad's ceiling is the commander's.
        assert node.teleop_max_speed >= 3.0 and node.teleop_accel > 0.0
    finally:
        node.destroy_node()
