# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Pure pose->frame builder tests (no USD, no Kit)."""

from __future__ import annotations

import math
import struct

import pytest

from optitrack.natnet.emulator.isaac.frames import (
    MODEL_LIST_CHANGED,
    TRACKING_VALID,
    BodySample,
    apply_pose_noise,
    build_frame,
    make_rigid_body_data,
    to_motive_pose,
)

pytestmark = pytest.mark.unit


def test_to_motive_pose_z_is_identity():
    pos = (1.0, 2.0, 3.0)
    quat = (0.1, 0.2, 0.3, 0.9)
    assert to_motive_pose(pos, quat, up_axis="Z") == (pos, quat)
    # Case-insensitive and default to Z.
    assert to_motive_pose(pos, quat, up_axis="z") == (pos, quat)
    assert to_motive_pose(pos, quat) == (pos, quat)


def test_to_motive_pose_y_swaps_axes_and_quat():
    # (x, y, z) -> (x, z, -y); quat vector part takes the same swap, scalar kept.
    pos, quat = to_motive_pose((1.0, 2.0, 3.0), (0.1, 0.2, 0.3, 0.9), up_axis="Y")
    assert pos == (1.0, 3.0, -2.0)
    assert quat == (0.1, 0.3, -0.2, 0.9)


def test_to_motive_pose_y_maps_isaac_up_to_motive_up():
    # Isaac +Z (up) must become Motive +Y (up) under the Y-up emulation.
    pos, _ = to_motive_pose((0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), up_axis="y")
    assert pos == (0.0, 1.0, 0.0)


def test_make_rigid_body_data_copies_pose_and_sets_valid_bit():
    rb = make_rigid_body_data(
        BodySample(7, (1.0, 2.0, 3.0), (0.0, 0.0, 0.7071068, 0.7071068), valid=True)
    )
    assert rb.ID == 7
    assert (rb.x, rb.y, rb.z) == (1.0, 2.0, 3.0)
    assert rb.qw == pytest.approx(0.7071068)
    assert rb.params & TRACKING_VALID  # client requires this bit or it skips the body


def test_lost_sample_clears_valid_bit_and_is_nan():
    rb = make_rigid_body_data(BodySample.lost(3))
    assert rb.ID == 3
    assert rb.params & TRACKING_VALID == 0
    assert math.isnan(rb.x) and math.isnan(rb.y) and math.isnan(rb.z)


def test_build_frame_no_bodies():
    frame = build_frame(0, [])
    assert frame.iFrame == 0
    assert frame.nRigidBodies == 0
    assert frame.params == 0


def test_apply_pose_noise_zero_std_is_identity():
    position = (1.0, 2.0, 3.0)
    orientation = (0.0, 0.0, 0.0, 1.0)
    pos_out, quat_out = apply_pose_noise(position, orientation, 0.0, 0.0)
    assert pos_out == position
    assert quat_out == pytest.approx(orientation)


def test_apply_pose_noise_preserves_y_position():
    np = pytest.importorskip("numpy")
    np.random.seed(0)
    position = (0.0, 1.5, 0.0)
    orientation = (0.0, 0.0, 0.0, 1.0)
    pos_out, _ = apply_pose_noise(position, orientation, 0.001, 0.0)
    # Before the euler-yaw shadowing bug, y collapsed to ~0 instead of staying near 1.5.
    assert pos_out[1] == pytest.approx(1.5, abs=0.01)


def test_apply_pose_noise_adds_position_jitter():
    np = pytest.importorskip("numpy")
    np.random.seed(1)
    position = (0.0, 0.0, 0.0)
    orientation = (0.0, 0.0, 0.0, 1.0)
    pos_out, _ = apply_pose_noise(position, orientation, 0.001, 0.0)
    assert pos_out != position


def test_build_frame_multiple_bodies_preserve_order():
    samples = [
        BodySample(1, (1.0, 0.0, 0.0)),
        BodySample(2, (0.0, 2.0, 0.0)),
        BodySample(5, (0.0, 0.0, 3.0)),
    ]
    frame = build_frame(42, samples)
    assert frame.iFrame == 42
    assert frame.nRigidBodies == 3
    assert frame.RigidBodies[0].ID == 1 and frame.RigidBodies[0].x == 1.0
    assert frame.RigidBodies[1].ID == 2 and frame.RigidBodies[1].y == 2.0
    assert frame.RigidBodies[2].ID == 5 and frame.RigidBodies[2].z == 3.0


def test_model_list_changed_sets_frame_param_bit():
    assert build_frame(0, [], model_list_changed=True).params & MODEL_LIST_CHANGED
    assert build_frame(0, [], model_list_changed=False).params & MODEL_LIST_CHANGED == 0


def test_frame_packs_and_rigid_body_section_decodes():
    frame = build_frame(9, [BodySample(4, (1.5, -2.5, 3.5), (0.0, 0.0, 0.0, 1.0))])
    payload = frame.pack(natnet_major=4, natnet_minor=4)

    # iFrame, then 4.4 counted sections (count+size each) for markersets & other markers.
    (iframe,) = struct.unpack_from("<i", payload, 0)
    assert iframe == 9
    # rigidbodies section: offset 4 + (4+4) markersets + (4+4) othermarkers = 20
    rb_count, rb_size = struct.unpack_from("<ii", payload, 20)
    assert rb_count == 1
    body_id, x, y, z = struct.unpack_from("<i3f", payload, 28)
    assert body_id == 4
    assert (round(x, 3), round(y, 3), round(z, 3)) == (1.5, -2.5, 3.5)
