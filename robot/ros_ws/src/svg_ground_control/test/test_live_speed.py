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


def test_goal_law_params_and_live_update():
    node = make(drone_names=["drone_1"], scenario="goal", hover_positions=[0.0, 0.0, 1.0],
                scenario_speed_mps=1.2, goal_accel_mps2=2.0, goal_settle_s=0.5,
                goal_lead_m=1.0)
    try:
        assert node.scenario.tracker.accel == 2.0
        assert node.scenario.tracker.settle == 0.5
        assert node.goal_lead == 1.0
        for name, value in (("goal_accel_mps2", 4.0), ("goal_settle_s", 0.2),
                            ("goal_lead_m", 3.0), ("goal_velocity_only_settle_s", 0.8)):
            assert node.set_parameters([Parameter(name, value=value)])[0].successful
        assert node.scenario.tracker.accel == 4.0
        assert node.scenario.tracker.settle == 0.2
        assert node.goal_lead == 3.0
        assert node.scenario.velocity_only_settle == 0.8
        # the speed note tells how far a goal must be to reach the setting
        assert "goal >" in node.speed_cap_note(1.2)
    finally:
        node.destroy_node()


def test_real_drones_get_the_trajectory_output_by_default():
    node = make(drone_names=["drone_1", "drone_2"], drone_modes="real,sim", scenario="goal",
                hover_positions=[0.0, 0.0, 1.0, 1.0, 0.0, 1.0])
    try:
        real, sim = node.drones
        assert real.output == "trajectory"
        assert real.cmd_pub.topic_name == "/drone_1/fmu/trajectory_command"
        assert sim.output == "velocity"
        assert sim.cmd_pub.topic_name == "/drone_2/interface/velocity_command"
    finally:
        node.destroy_node()


def test_real_command_mode_velocity_keeps_the_old_topic():
    node = make(drone_names=["drone_1"], drone_modes="real", scenario="hover",
                hover_positions=[0.0, 0.0, 1.0], real_command_mode="velocity")
    try:
        assert node.drones[0].output == "velocity"
        assert node.drones[0].cmd_pub.topic_name == "/drone_1/fmu/velocity_command"
    finally:
        node.destroy_node()


def test_goal_heading_from_xyzt_and_quaternion():
    from std_msgs.msg import Float64MultiArray
    from geometry_msgs.msg import PoseStamped
    from svg_ground_control.swarm_commander import heading_to_yaw, yaw_to_heading
    assert heading_to_yaw(90.0) == pytest.approx(-np.pi / 2)     # clockwise = -yaw
    assert yaw_to_heading(heading_to_yaw(-135.0)) == pytest.approx(-135.0)
    node = make(drone_names=["drone_1"], drone_modes="real", scenario="goal",
                hover_positions=[0.0, 0.0, 1.0])
    try:
        assert node.scenario.headings[0] == 0.0                       # nose on +X
        node.goal_xyzt_callback(0, Float64MultiArray(data=[1.0, 2.0, 1.5, 90.0]))
        np.testing.assert_allclose(node.scenario.goals[0], [1.0, 2.0, 1.5])
        assert node.scenario.headings[0] == pytest.approx(-np.pi / 2)
        node.goal_xyzt_callback(0, Float64MultiArray(data=[0.0, 0.0, 1.0]))
        assert node.scenario.headings[0] == 0.0
        msg = PoseStamped()
        msg.pose.position.z = 1.0
        msg.pose.orientation.z = np.sin(0.25 * np.pi)                  # ENU yaw +90 deg
        msg.pose.orientation.w = np.cos(0.25 * np.pi)
        node.goal_callback(0, msg)
        assert node.scenario.headings[0] == pytest.approx(np.pi / 2)
        node.goal_callback(0, PoseStamped())                            # unset quaternion
        assert node.scenario.headings[0] == 0.0
        assert node.desired_heading(node.drones[0]) == 0.0
    finally:
        node.destroy_node()


def test_hold_brakes_to_a_stop_point_instead_of_the_call_position():
    from std_srvs.srv import Trigger
    from svg_ground_control.swarm_commander import FlightState
    node = make(drone_names=["drone_1"], drone_modes="real", scenario="goal",
                hover_positions=[0.0, 0.0, 1.0], goal_accel_mps2=6.0, hover_kp=1.0,
                fence_enabled=False)
    try:
        d = node.drones[0]
        d.position = np.array([0.0, 0.0, 1.5]); d.velocity = np.array([0.0, 6.0, 0.0])
        d.state = FlightState.ACTIVE
        res = node.handle_hold(Trigger.Request(), Trigger.Response())
        assert res.success and "braking" in res.message
        # v^2/(2a) + v/kp = 36/12 + 6 = 9 m ahead along +y
        np.testing.assert_allclose(d.hold_target, [0.0, 9.0, 1.5], atol=1e-6)
        # at rest: hold where it is
        d.velocity = np.zeros(3)
        node.handle_hold(Trigger.Request(), Trigger.Response())
        np.testing.assert_allclose(d.hold_target, d.position)
        # the hold law continues at the current speed right after the call
        d.velocity = np.array([0.0, 6.0, 0.0])
        node.handle_hold(Trigger.Request(), Trigger.Response())
        from svg_ground_control.trajectory import seek_velocity
        v = seek_velocity(d.position[None], d.hold_target[None], 10.0, 6.0, 1.0)[0]
        assert v[1] == pytest.approx(6.0, abs=0.05)
    finally:
        node.destroy_node()


def test_takeoff_resets_stored_goals_and_goals_accepted_before_start():
    from std_srvs.srv import Trigger
    from std_msgs.msg import Float64MultiArray
    node = make(drone_names=["drone_1"], drone_modes="real", scenario="goal",
                hover_positions=[0.0, 0.0, 1.0])
    try:
        d = node.drones[0]
        d.position = np.array([0.0, 0.0, 0.05])
        node.goal_xyzt_callback(0, Float64MultiArray(data=[0.0, 5.0, 1.5, 90.0]))   # stale
        np.testing.assert_allclose(node.scenario.goals[0], [0.0, 5.0, 1.5])
        res = node.handle_takeoff(Trigger.Request(), Trigger.Response())
        assert res.success
        np.testing.assert_allclose(node.scenario.goals[0], [0.0, 0.0, 1.0])         # reset
        assert node.scenario.headings[0] == 0.0
        # retarget before start: accepted (no gating on mission_active)
        assert not node.mission_active
        node.goal_xyzt_callback(0, Float64MultiArray(data=[1.0, 2.0, 1.5]))
        np.testing.assert_allclose(node.scenario.goals[0], [1.0, 2.0, 1.5])
        # a twin commander blocks takeoff/start
        node.duplicate_commander = True
        res = node.handle_start(Trigger.Request(), Trigger.Response())
        assert not res.success and "another swarm_commander" in res.message
    finally:
        node.destroy_node()


def test_twin_pid_scan_matches_only_node_processes(tmp_path, monkeypatch):
    import os
    from svg_ground_control.swarm_commander import SwarmCommander
    fake = tmp_path / "proc"
    for pid, argv in {
        "100": ["/usr/bin/python3", "/x/install/svg_ground_control/lib/svg_ground_control/swarm_commander", "--ros-args"],
        "101": ["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "run", "svg_ground_control", "swarm_commander"],
        "102": ["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "launch", "svg_ground_control", "ground_control.launch.py"],
        "103": ["/usr/bin/python3", "/x/lib/svg_ground_control/mocap_bridge"],
    }.items():
        (fake / pid).mkdir(parents=True)
        (fake / pid / "cmdline").write_bytes("\0".join(argv).encode() + b"\0")
    real_listdir, real_open = os.listdir, open
    monkeypatch.setattr(os, "listdir", lambda d: real_listdir(fake) if d == "/proc" else real_listdir(d))
    import builtins
    monkeypatch.setattr(builtins, "open", lambda f, *a, **k: real_open(
        str(fake) + f[len("/proc"):] if str(f).startswith("/proc/") else f, *a, **k))
    assert SwarmCommander.twin_pids() == [100]
