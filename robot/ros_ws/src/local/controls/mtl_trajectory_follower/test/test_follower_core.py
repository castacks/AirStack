# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Unit tests for mtl_trajectory_follower.follower_core (pure Python)."""

import math
import sys
from pathlib import Path

import pytest

_PKG = Path(__file__).resolve().parent.parent
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from mtl_trajectory_follower import follower_core as F  # noqa: E402
from mtl_trajectory_follower.gimbal_math import boresight_from_euler  # noqa: E402

# Collected by tests/ (colcon_unit_test_packages.yaml); the root conftest's auto-mark
# hook does not reach files outside tests/, so declare the tier here.
pytestmark = pytest.mark.unit


def serpentine(lanes=3, lane_len=60.0, spacing=8.0, step=0.5, z=30.0):
    """Boustrophedon whose lanes are closer than the projection window is long."""
    xs, ys, yaws = [], [], []
    for i in range(lanes):
        y = i * spacing
        n = int(lane_len / step)
        rng = range(n + 1) if i % 2 == 0 else range(n, -1, -1)
        for k in rng:
            xs.append(k * step)
            ys.append(y)
            yaws.append(0.0 if i % 2 == 0 else math.pi)
        if i < lanes - 1:  # connector
            x_end = xs[-1]
            for k in range(1, int(spacing / step)):
                xs.append(x_end)
                ys.append(y + k * step)
                yaws.append(math.pi / 2)
    n = len(xs)
    return F.Track(x=xs, y=ys, z=[z] * n, yaw=yaws, speed=[6.0] * n,
                   bx=list(xs), by=[v - 10.0 for v in ys], bz=[0.0] * n)


def cfg(**kw):
    base = dict(min_turn_radius_m=12.0, single_axis=False)
    base.update(kw)
    return F.FollowerConfig(**base)


def test_track_validation_and_interpolation():
    tr = serpentine()
    assert tr.arc[0] == 0.0 and tr.total > 150.0
    x, y, z = tr.position_at(10.25)
    assert (x, y, z) == pytest.approx((10.25, 0.0, 30.0))
    with pytest.raises(ValueError):
        F.Track(x=[0.0], y=[0.0], z=[0.0], yaw=[0.0], speed=[1.0], bx=[0.0], by=[0.0], bz=[0.0])


def test_lookahead_default_is_1p2_turn_radii():
    assert cfg().lookahead == pytest.approx(14.4)
    assert cfg(lookahead_m=5.0).lookahead == 5.0


def test_ingress_then_search_then_complete():
    tr = serpentine()
    f = F.TrackFollower(tr, cfg())
    # starts 50 m away, on the ground
    pos = [-50.0, 0.0, 0.1]
    assert f.start(pos) == F.INGRESS
    out = f.step(pos, 0.0, 0.05)
    assert out.state == F.INGRESS
    assert out.carrot[2] == pytest.approx(30.0)                      # climb to mission altitude
    assert math.dist(out.carrot[:2], pos[:2]) == pytest.approx(14.4)  # one lookahead toward the start
    # arrive at the start
    pos = [0.5, 0.2, 29.5]
    out = f.step(pos, 0.0, 0.05)
    assert out.state == F.SEARCH
    # drive the vehicle exactly onto its own carrot: it must traverse every lane in order
    last = -1.0
    for _ in range(5000):
        out = f.step(pos, 0.0, 0.05)
        assert out.progress_m >= last
        last = out.progress_m
        if out.state == F.COMPLETE:
            break
        # move 1 m toward the carrot
        dx, dy = out.carrot[0] - pos[0], out.carrot[1] - pos[1]
        d = math.hypot(dx, dy)
        pos = [pos[0] + dx / d, pos[1] + dy / d, out.carrot[2]]
    assert out.state == F.COMPLETE
    assert last >= tr.total - 3.0 - 1e-9
    assert out.carrot == (tr.x[-1], tr.y[-1], tr.z[-1])


def test_projection_is_forward_only_on_close_lanes():
    tr = serpentine(spacing=8.0)
    f = F.TrackFollower(tr, cfg())
    f.start([0.0, 0.0, 30.0])
    # fly along lane 0 to x = 40
    for x in range(0, 41, 1):
        out = f.step([float(x), 0.0, 30.0], 0.0, 0.05)
    p0 = out.progress_m
    assert p0 == pytest.approx(40.0, abs=0.6)
    # a vehicle drifting 6 m toward lane 1 (8 m away) must NOT jump onto it
    out = f.step([40.0, 6.0, 30.0], 0.0, 0.05)
    assert out.progress_m == pytest.approx(p0, abs=0.6)
    assert out.cross_track_error_m == pytest.approx(6.0, abs=1e-6)


def test_carrot_leads_by_lookahead_and_velocity_feedforward():
    tr = serpentine()
    f = F.TrackFollower(tr, cfg())
    f.start([0.0, 0.0, 30.0])
    out = f.step([10.0, 0.5, 30.0], 0.0, 0.05)
    assert out.progress_m == pytest.approx(10.0)
    assert out.carrot[0] == pytest.approx(24.4)
    assert out.carrot_velocity[0] == pytest.approx(6.0)
    assert out.remaining_m == pytest.approx(tr.total - 10.0)


def test_two_axis_gimbal_aims_at_scheduled_point_from_actual_position():
    tr = serpentine()
    f = F.TrackFollower(tr, cfg(gimbal_lead_s=0.0))
    f.start([0.0, 0.0, 30.0])
    pos = (10.0, 1.5, 28.0)                  # off-track and low: pointing must still land on target
    out = f.step(pos, 0.0, 0.0)
    roll, pitch, yaw = out.gimbal
    b = boresight_from_euler(pitch, yaw)
    s = (out.aim[2] - pos[2]) / b[2]
    assert pos[0] + s * b[0] == pytest.approx(out.aim[0])
    assert pos[1] + s * b[1] == pytest.approx(out.aim[1])
    assert out.aim == pytest.approx((10.0, -10.0, 0.0))


def test_single_axis_gimbal_slew_rate_is_bounded():
    tr = serpentine()
    c = cfg(single_axis=True, tilt_rad=0.0, gimbal_rate_rad_s=math.radians(90))
    f = F.TrackFollower(tr, c)
    f.start([0.0, 0.0, 30.0])
    prev = f.step([0.0, 0.0, 30.0], 0.0, 0.05).gimbal
    for x in range(1, 30):
        g = f.step([float(x), 0.0, 30.0], math.radians(40 * (x % 2)), 0.05).gimbal
        for a, b in zip(prev, g):
            assert abs(math.remainder(b - a, 2 * math.pi)) <= math.radians(90) * 0.05 + 1e-9
        prev = g
