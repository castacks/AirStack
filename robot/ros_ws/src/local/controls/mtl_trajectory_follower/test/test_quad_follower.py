# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Quadrotor (body-fixed camera, 6-DOF reference) mode of mtl_trajectory_follower:
follower_core.QuadTrackFollower and the node's quadrotor path, against the ROS stubs."""

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
from mtl_trajectory_follower import follower_core as F  # noqa: E402
from mtl_trajectory_follower.gimbal_math import (boresight_from_euler, euler_zyx_to_matrix,  # noqa: E402
                                                 matrix_to_euler_zyx, quat_to_matrix)

pytestmark = pytest.mark.unit
G = 9.81


def weave(n=600, dt=0.1, v=6.0, h=300.0, amp=8.0, f=0.14):
    """An East-bound quad reference weaving across track (as the quad conversion makes it):
    y = amp sin(2 pi f t); roll from the lateral acceleration, yaw swinging with it."""
    w = 2 * math.pi * f
    t = [k * dt for k in range(n)]
    x = [v * tk for tk in t]
    y = [amp * math.sin(w * tk) for tk in t]
    vy = [amp * w * math.cos(w * tk) for tk in t]
    ay = [-amp * w * w * math.sin(w * tk) for tk in t]
    yaw = [0.4 * math.sin(w * tk) for tk in t]
    yaw_rate = [0.4 * w * math.cos(w * tk) for tk in t]
    # zero yaw-frame lateral accel -> roll = atan(-a_left / g) in the yaw frame (small pitch)
    roll, pitch = [], []
    for k in range(n):
        c, s = math.cos(yaw[k]), math.sin(yaw[k])
        af, al = c * 0.0 + s * ay[k], -s * 0.0 + c * ay[k]
        pitch.append(-math.atan2(af, G))                       # + nose UP (planner convention)
        roll.append(math.atan2(-al, math.hypot(af, G)))
    arc = [0.0]
    for k in range(1, n):
        arc.append(arc[-1] + math.hypot(x[k] - x[k - 1], y[k] - y[k - 1]))
    tilt = math.radians(32.25)
    bx, by = [], []
    for k in range(n):
        cam = F.body_fixed_camera_euler(roll[k], -pitch[k], yaw[k], tilt)
        b = boresight_from_euler(cam[1], cam[2])
        s_ = h / -b[2]
        bx.append(x[k] + s_ * b[0])
        by.append(y[k] + s_ * b[1])
    return F.Track(x=x, y=y, z=[h] * n, yaw=yaw, speed=[math.hypot(v, a) for a in vy], bx=bx, by=by,
                   bz=[0.0] * n, arc=arc, t=t, roll=roll, pitch=pitch, vx=[v] * n, vy=vy, ax=[0.0] * n,
                   ay=ay, yaw_rate=yaw_rate)


def test_reference_at_time_interpolates_every_axis():
    tr = weave()
    r = tr.reference_at_time(10.05)
    k = 100
    assert r["pos"][0] == pytest.approx(0.5 * (tr.x[k] + tr.x[k + 1]))
    assert r["vel"][1] == pytest.approx(0.5 * (tr.vy[k] + tr.vy[k + 1]))
    assert r["roll"] == pytest.approx(0.5 * (tr.roll[k] + tr.roll[k + 1]))
    assert r["yaw"] == pytest.approx(0.5 * (tr.yaw[k] + tr.yaw[k + 1]))
    assert tr.duration == pytest.approx(59.9)


def test_body_fixed_camera_matches_the_mount_geometry():
    tilt = math.radians(32.25)
    roll, pitch, yaw = F.body_fixed_camera_euler(0.0, 0.0, 0.7, tilt)   # level: looks tilt ahead of nadir
    assert pitch == pytest.approx(math.pi / 2 - tilt) and yaw == pytest.approx(0.7)
    # nose down 5 deg: the camera looks 5 deg further down
    assert F.body_fixed_camera_euler(0.0, math.radians(5), 0.0, tilt)[1] == pytest.approx(
        math.pi / 2 - tilt + math.radians(5))
    # right side down (the thrust tilts right: accelerating right) swings the downward camera
    # to the LEFT (+y for an East-bound vehicle): "to look left, a quad must accelerate right"
    b = boresight_from_euler(*F.body_fixed_camera_euler(math.radians(20), 0.0, 0.0, tilt)[1:])
    assert b[1] == pytest.approx(math.sin(math.radians(20)) * math.cos(tilt), abs=1e-9)


def test_quad_follower_tracks_by_time_and_feeds_the_attitude_forward():
    tr = weave()
    fol = F.QuadTrackFollower(tr, F.QuadFollowerConfig(attitude_lead_s=0.0, yaw_lead_s=0.0))
    # starts at the first sample, facing its heading -> SEARCH at t = 0
    out = fol.step((0.0, 0.0, 300.0), 0.0, 0.0)
    assert out.state == F.SEARCH and out.t_ref == pytest.approx(0.0)
    pos = (0.0, 0.0, 300.0)
    for _ in range(200):          # a vehicle that is always exactly on its reference
        out = fol.step(pos, out.carrot_yaw, 0.05)
        pos = out.carrot
    assert out.t_ref == pytest.approx(200 * 0.05)
    ref = tr.reference_at_time(out.t_ref)
    assert out.carrot == pytest.approx(ref["pos"])
    assert out.carrot_velocity == pytest.approx(ref["vel"])
    roll, pitch_down, yaw = out.attitude_ref
    assert (roll, pitch_down, yaw) == pytest.approx((ref["roll"], -ref["pitch_up"], ref["yaw"]))
    assert out.carrot_yaw == pytest.approx(yaw)


def test_lag_governor_holds_the_reference_for_a_slow_vehicle():
    tr = weave()
    fol = F.QuadTrackFollower(tr, F.QuadFollowerConfig(lag_soft_m=5.0, lag_hard_m=20.0))
    fol.step((0.0, 0.0, 300.0), 0.0, 0.0)
    for _ in range(400):          # the vehicle never moves: the reference runs ahead, then stops
        out = fol.step((0.0, 0.0, 300.0), 0.0, 0.05)
    assert out.state == F.SEARCH
    lag = out.gimbal_diag["lag_m"]
    assert 5.0 <= lag <= 20.0 + 6.0 * 0.05
    t_stuck = out.t_ref
    out = fol.step((0.0, 0.0, 300.0), 0.0, 0.05)
    assert out.t_ref == pytest.approx(t_stuck, abs=1e-3)


def test_ingress_until_at_the_start_and_facing_it_then_complete_at_the_end():
    tr = weave(n=60)
    fol = F.QuadTrackFollower(tr, F.QuadFollowerConfig())
    out = fol.step((-40.0, 0.0, 280.0), 0.0, 0.05)
    assert out.state == F.INGRESS and out.attitude_ref is None and out.carrot[2] == 300.0
    out = fol.step((0.0, 0.0, 300.0), math.radians(90), 0.05)        # there, but facing North
    assert out.state == F.INGRESS
    out = fol.step((0.0, 0.0, 300.0), 0.0, 0.05)
    assert out.state == F.SEARCH
    pos = (0.0, 0.0, 300.0)
    for _ in range(400):
        out = fol.step(pos, out.carrot_yaw, 0.05)
        pos = out.carrot
        if out.state == F.COMPLETE:
            break
    assert out.state == F.COMPLETE and out.attitude_ref is None
    assert out.carrot == (tr.x[-1], tr.y[-1], tr.z[-1])


def test_a_quad_track_needs_a_timeline():
    tr = weave(n=10)
    tr.t = []
    with pytest.raises(ValueError, match="timeline"):
        F.QuadTrackFollower(tr, F.QuadFollowerConfig())


# ---------------------------------------------------------------- node
@pytest.fixture
def node_mod(monkeypatch):
    S.install(monkeypatch)
    monkeypatch.delitem(sys.modules, "mtl_trajectory_follower.follower_node", raising=False)
    return importlib.import_module("mtl_trajectory_follower.follower_node")


def quad_plan(tr: F.Track):
    msg = S.Msg()
    msg.header.stamp = S.SimClock().now().to_msg()
    msg.header.frame_id = "map"
    msg.plan_id, msg.run_id, msg.start_mission = "t/robot_1/quad", "run", True
    msg.platform = "quadrotor"
    msg.single_axis_gimbal, msg.mount_tilt_rad = True, math.radians(32.25)
    msg.min_turn_radius_m, msg.speed_mps = 12.0, 6.0
    msg.gimbal_max_rad, msg.gimbal_rate_rad_s, msg.pitch_nudge_max_rad = 0.0, 0.0, 0.0
    msg.gimbal_law = "open_loop"
    msg.trajectory.waypoints = [S.Msg(position=S.Vector3(x=tr.x[k], y=tr.y[k], z=tr.z[k]), velocity=tr.speed[k],
                                      yaw=tr.yaw[k], acceleration=S.Vector3(x=tr.ax[k], y=tr.ay[k], z=0.0))
                                for k in range(len(tr))]
    msg.boresight = [S.Vector3(x=tr.bx[k], y=tr.by[k], z=0.0) for k in range(len(tr))]
    msg.arc_length_m, msg.time_s = list(tr.arc), list(tr.t)
    msg.planned_gimbal_phi_rad = [0.0] * len(tr)
    msg.planned_roll_rad, msg.planned_pitch_rad = list(tr.roll), list(tr.pitch)
    msg.planned_velocity = [S.Vector3(x=tr.vx[k], y=tr.vy[k], z=0.0) for k in range(len(tr))]
    msg.planned_yaw_rate_rad_s = list(tr.yaw_rate)
    return msg


def test_node_flies_a_quad_plan_with_attitude_feedforward_and_a_bolted_camera(node_mod):
    tr = weave(n=120)
    node = node_mod.MtlTrajectoryFollower()
    node.subs["odometry"](S.odom(0.0, 0.0, 300.0, yaw=0.0))
    node.subs["search/plan"](quad_plan(tr))
    assert node.active and node.quad
    for _ in range(40):
        S.SimClock.advance(0.05)
        node.timers[0][1]()
        tp = node.pubs["trajectory_controller/tracking_point"].msgs[-1]
        node.subs["odometry"](S.odom(tp.pose.position.x, tp.pose.position.y, tp.pose.position.z,
                                     yaw=2 * math.atan2(tp.pose.orientation.z, tp.pose.orientation.w)))
    ff = node.pubs["trajectory_controller/attitude_feedforward"].msgs
    assert len(ff) >= 39
    q = ff[-1].quaternion
    roll, pitch_down, yaw = matrix_to_euler_zyx(quat_to_matrix(q.x, q.y, q.z, q.w))
    ref = tr.reference_at_time(node.follower.t_ref + node.quad_attitude_lead_s)
    yref = tr.reference_at_time(node.follower.t_ref + node.quad_yaw_lead_s)
    assert (roll, pitch_down, yaw) == pytest.approx((ref["roll"], -ref["pitch_up"], yref["yaw"]), abs=1e-9)
    # the gimbal command is where the BOLTED camera looks from the odometry attitude
    cmd = node.pubs["gimbal/cmd_pitch_yaw"].msgs[-1]
    assert cmd.y == pytest.approx(math.radians(90 - 32.25), abs=1e-6)   # the stub odometry is level


def test_gimbal_plans_never_publish_a_feedforward(node_mod):
    from test_follower_node import make_plan
    node = node_mod.MtlTrajectoryFollower()
    node.subs["odometry"](S.odom(0.0, 0.0, 30.0))
    node.subs["search/plan"](make_plan())
    for _ in range(20):
        S.SimClock.advance(0.05)
        node.timers[0][1]()
    assert node.active and not node.quad
    assert node.pubs["trajectory_controller/attitude_feedforward"].msgs == []


def test_euler_round_trip_used_by_the_feedforward():
    for r, p, y in ((0.3, -0.2, 2.0), (-0.1, 0.4, -1.0)):
        assert matrix_to_euler_zyx(euler_zyx_to_matrix(r, p, y)) == pytest.approx((r, p, y))


def test_yaw_and_roll_pitch_have_separate_leads():
    tr = weave()
    fol = F.QuadTrackFollower(tr, F.QuadFollowerConfig(attitude_lead_s=0.3, yaw_lead_s=0.45))
    out = fol.step((0.0, 0.0, 300.0), 0.0, 0.0)
    for _ in range(100):
        out = fol.step(out.carrot, out.carrot_yaw, 0.05)
    r, y = tr.reference_at_time(out.t_ref + 0.3), tr.reference_at_time(out.t_ref + 0.45)
    assert out.attitude_ref == pytest.approx((r["roll"], -r["pitch_up"], y["yaw"]))
    assert out.carrot_yaw == pytest.approx(y["yaw"])
