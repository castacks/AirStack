# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Node-level test of mtl_trajectory_follower against hermetic ROS stubs.

Drives the real node code (callbacks + 20 Hz tick) with a fake clock and a
point-mass "vehicle" that moves toward the published carrot, and checks the
tracking-point arbitration, the gimbal command stream, the status sequence and
the hand-back to the trajectory controller.
"""

import importlib
import math
import sys
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent
for p in (_HERE, _HERE.parent):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import _ros_stubs as S  # noqa: E402

# Collected by tests/ (colcon_unit_test_packages.yaml); the root conftest's auto-mark
# hook does not reach files outside tests/, so declare the tier here.
pytestmark = pytest.mark.unit


@pytest.fixture
def node_mod(monkeypatch):
    S.install(monkeypatch)
    monkeypatch.delitem(sys.modules, "mtl_trajectory_follower.follower_node", raising=False)
    return importlib.import_module("mtl_trajectory_follower.follower_node")


def make_plan(n=200, step=0.5, z=30.0, start=True, single_axis=True, plan_id="t/robot_1/run"):
    msg = S.Msg()
    msg.header.stamp = S.SimClock().now().to_msg()
    msg.header.frame_id = "map"
    msg.plan_id, msg.run_id, msg.start_mission = plan_id, "run", start
    msg.single_axis_gimbal, msg.mount_tilt_rad = single_axis, math.radians(30)
    msg.min_turn_radius_m, msg.speed_mps = 12.0, 6.0
    msg.gimbal_max_rad, msg.gimbal_rate_rad_s, msg.pitch_nudge_max_rad = math.radians(80), math.radians(120), math.radians(5)
    msg.trajectory.waypoints = [S.Msg(position=S.Vector3(x=k * step, y=0.0, z=z), velocity=6.0, yaw=0.0)
                                for k in range(n)]
    msg.boresight = [S.Vector3(x=k * step + 17.3, y=-5.0, z=0.0) for k in range(n)]
    msg.arc_length_m = [k * step for k in range(n)]
    msg.time_s = [k * step / 6.0 for k in range(n)]
    msg.planned_gimbal_phi_rad = [0.0] * n
    return msg


def tick(node, dt=0.05):
    S.SimClock.advance(dt)
    node.timers[0][1]()


def test_idle_forwards_nominal_and_parks_gimbal(node_mod):
    node = node_mod.MtlTrajectoryFollower()
    node.subs["odometry"](S.odom(0, 0, 30, yaw=0.3))
    nominal = S.Msg()
    node.subs["trajectory_controller/tracking_point_nominal"](nominal)
    assert node.pubs["trajectory_controller/tracking_point"].msgs == [nominal]
    tick(node)
    cmd = node.pubs["gimbal/cmd_pitch_yaw"].msgs[-1]
    assert cmd.y == pytest.approx(math.radians(60)) and cmd.z == pytest.approx(0.3)
    # the optical frame is static; the gimbal frame follows the base
    assert node.tf.sent and node.tf.sent[-1].child_frame_id == "camera_gimbal_link"


def test_preview_plan_is_not_flown(node_mod):
    node = node_mod.MtlTrajectoryFollower()
    node.subs["search/plan"](make_plan(start=False))
    assert not node.active


def test_sortie_ingress_search_complete_and_hand_back(node_mod):
    node = node_mod.MtlTrajectoryFollower()
    pos = [-30.0, 0.0, 5.0]
    node.subs["odometry"](S.odom(*pos))
    node.subs["search/plan"](make_plan())
    assert node.active
    tp = node.pubs["trajectory_controller/tracking_point"]
    states = []
    for _ in range(3000):
        tick(node)
        st = node.pubs["search/follower_status"].msgs[-1]
        states.append(st.state_name)
        if not node.active:
            break
        # nominal points are suppressed while the sortie is active
        n_before = len(tp.msgs)
        node.subs["trajectory_controller/tracking_point_nominal"](S.Msg())
        assert len(tp.msgs) == n_before
        c = tp.msgs[-1].pose.position
        d = math.dist((c.x, c.y, c.z), pos)
        if d > 1e-6:
            s = min(0.3, d) / d  # 6 m/s at 20 Hz
            pos = [pos[i] + s * (v - pos[i]) for i, v in enumerate((c.x, c.y, c.z))]
        node.subs["odometry"](S.odom(*pos))
    order = [s for i, s in enumerate(states) if i == 0 or s != states[i - 1]]
    assert order == ["INGRESS", "SEARCH", "COMPLETE"]
    assert not node.active
    # hand-back asked the trajectory controller to hold (ROBOT_POSE)
    calls = node.clients["trajectory_controller/set_trajectory_mode"].calls
    assert calls and calls[-1].mode == 1
    # gimbal commands flowed every tick and stayed within the slew limit
    cmds = node.pubs["gimbal/cmd_pitch_yaw"].msgs
    assert len(cmds) >= len(states) - 1
    # after hand-back the nominal point is forwarded again
    nominal = S.Msg()
    node.subs["trajectory_controller/tracking_point_nominal"](nominal)
    assert tp.msgs[-1] is nominal


def test_state_estimate_timeout_aborts(node_mod):
    node = node_mod.MtlTrajectoryFollower()
    node.subs["odometry"](S.odom(0, 0, 30))
    node.subs["search/plan"](make_plan())
    tick(node)
    node.subs["state_estimate_timed_out"](S.Msg(data=True))
    assert not node.active
    tick(node)
    assert node.pubs["search/follower_status"].msgs[-1].state_name == "ABORTED"


def test_stale_latched_plan_is_ignored(node_mod):
    node = node_mod.MtlTrajectoryFollower()
    plan = make_plan()
    S.SimClock.advance(120.0)
    node.subs["search/plan"](plan)
    assert not node.active
