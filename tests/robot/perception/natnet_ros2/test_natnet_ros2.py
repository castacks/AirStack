# Copyright 2026 AirLab CMU
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for natnet_ros2 Python source code.

These tests import the actual production source files and stub out ROS at the
import boundary so no ROS installation is required.

Coverage here:
  vision_pose_converter_node.py  → VisionPoseConverterNode._canonical_quaternion()
                                 → VisionPoseConverterNode._on_pose() frame_id assignment

NOT covered here (C++ — requires colcon build + gtest):
  natnet_ros2_node.cpp           → build_covariance_6x6(), topic name construction,
                                   connection_type validation, SDK frame callback logic.
  Add C++ tests under robot/ros_ws/src/perception/natnet_ros2/test/ using ament_add_gtest.
"""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# Stub ROS before importing the source.
#
# The key subtlety: VisionPoseConverterNode inherits from rclpy.node.Node.
# If Node is a plain MagicMock() the class body is never executed (Python's
# metaclass machinery returns a Mock for attribute access instead of running
# __init_subclass__ / defining methods).  We supply a real dummy base class
# so the actual class body — including _canonical_quaternion — is defined.
# ---------------------------------------------------------------------------

class _FakeNode:
    def __init__(self, name: str):
        pass
    def get_logger(self):
        return MagicMock()
    def declare_parameter(self, *args, **kwargs):
        pass
    def get_parameter(self, name):
        m = MagicMock()
        m.value = MagicMock()
        return m
    def create_subscription(self, *args, **kwargs):
        return MagicMock()
    def create_publisher(self, *args, **kwargs):
        return MagicMock()


_rclpy_node_mod = MagicMock()
_rclpy_node_mod.Node = _FakeNode
sys.modules.setdefault("rclpy", MagicMock())
sys.modules["rclpy.node"] = _rclpy_node_mod
sys.modules.setdefault("geometry_msgs", MagicMock())
sys.modules.setdefault("geometry_msgs.msg", MagicMock())

# Add the actual source directory (parents[4] = repo root).
_natnet_src = Path(__file__).resolve().parents[4] / "robot/ros_ws/src/perception/natnet_ros2/src"
if str(_natnet_src) not in sys.path:
    sys.path.insert(0, str(_natnet_src))

from vision_pose_converter_node import VisionPoseConverterNode  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _quat(x: float, y: float, z: float, w: float) -> SimpleNamespace:
    """Minimal quaternion-like object matching the expected interface."""
    return SimpleNamespace(x=x, y=y, z=z, w=w)


# ---------------------------------------------------------------------------
# VisionPoseConverterNode._canonical_quaternion
# ---------------------------------------------------------------------------

import pytest


@pytest.mark.unit
def test_canonical_quaternion_positive_w_unchanged():
    """Quaternion with w > 0 must not be altered."""
    q = _quat(0.1, 0.2, 0.3, 0.9)
    out = VisionPoseConverterNode._canonical_quaternion(q)
    assert out.w == pytest.approx(0.9)
    assert out.x == pytest.approx(0.1)
    assert out.y == pytest.approx(0.2)
    assert out.z == pytest.approx(0.3)


@pytest.mark.unit
def test_canonical_quaternion_negative_w_flipped():
    """Quaternion with w < 0 must have all four components negated."""
    q = _quat(0.1, 0.2, 0.3, -0.9)
    out = VisionPoseConverterNode._canonical_quaternion(q)
    assert out.w == pytest.approx(0.9)
    assert out.x == pytest.approx(-0.1)
    assert out.y == pytest.approx(-0.2)
    assert out.z == pytest.approx(-0.3)


@pytest.mark.unit
def test_canonical_quaternion_zero_w_unchanged():
    """w == 0 satisfies w >= 0 so no flip should occur."""
    q = _quat(1.0, 0.0, 0.0, 0.0)
    out = VisionPoseConverterNode._canonical_quaternion(q)
    assert out.w == pytest.approx(0.0)
    assert out.x == pytest.approx(1.0)


@pytest.mark.unit
def test_canonical_quaternion_identity():
    q = _quat(0.0, 0.0, 0.0, 1.0)
    out = VisionPoseConverterNode._canonical_quaternion(q)
    assert out.w == pytest.approx(1.0)
    assert out.x == pytest.approx(0.0)


@pytest.mark.unit
def test_canonical_quaternion_returns_same_object():
    """The method mutates and returns the same object (not a copy)."""
    q = _quat(0.0, 0.0, 0.0, 1.0)
    out = VisionPoseConverterNode._canonical_quaternion(q)
    assert out is q


@pytest.mark.unit
def test_canonical_quaternion_w_stays_non_negative():
    """After canonicalisation w must always be >= 0."""
    cases = [
        _quat(0.0, 0.0, 0.7071, 0.7071),
        _quat(0.0, 0.0, -0.7071, -0.7071),
        _quat(0.5, -0.5, 0.5, -0.5),
        _quat(0.0, 0.0, 1.0, 0.0),
    ]
    for q in cases:
        out = VisionPoseConverterNode._canonical_quaternion(q)
        assert out.w >= 0.0, f"w={out.w} after canonicalisation of {q}"


@pytest.mark.unit
def test_canonical_quaternion_dual_sign_produces_same_result():
    """q and -q must both canonicalise to the same output."""
    q_pos = _quat(0.1, 0.2, 0.3, 0.9)
    q_neg = _quat(-0.1, -0.2, -0.3, -0.9)
    out_pos = VisionPoseConverterNode._canonical_quaternion(q_pos)
    out_neg = VisionPoseConverterNode._canonical_quaternion(q_neg)
    assert out_pos.w == pytest.approx(out_neg.w)
    assert out_pos.x == pytest.approx(out_neg.x)
    assert out_pos.y == pytest.approx(out_neg.y)
    assert out_pos.z == pytest.approx(out_neg.z)
