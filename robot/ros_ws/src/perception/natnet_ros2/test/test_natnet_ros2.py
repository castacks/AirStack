# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Unit tests for natnet_ros2 Python helpers (no ROS install required).

Stubs rclpy/launch at import time. Covers ``VisionPoseConverterNode`` quaternion
canonicalisation, configurable-topic wiring, and ``natnet_ros2.launch.py``
profile-flattening helpers (server + per-body arrays, env expansion, namespacing).

C++ logic (``natnet_logic.hpp``) is tested in ``test_natnet_logic.cpp`` via colcon.
"""

import importlib.util
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
#
# The fake also records declared params and created sub/pub topics so the
# configurable-topic wiring can be asserted without a ROS install.
# ---------------------------------------------------------------------------

class _FakeNode:
    # Per-test parameter overrides keyed by name; consulted by declare_parameter so
    # values survive the node's super().__init__ (which resets per-instance state).
    _overrides: dict = {}

    def __init__(self, name: str):
        self._params: dict = {}
        self.created_subscriptions: list = []
        self.created_publishers: list = []
    def get_logger(self):
        return MagicMock()
    def declare_parameter(self, name, default=None):
        self._params[name] = self._overrides.get(name, default)
    def get_parameter(self, name):
        return SimpleNamespace(value=self._params.get(name))
    def create_subscription(self, msg_type, topic, callback, qos):
        self.created_subscriptions.append(topic)
        return MagicMock()
    def create_publisher(self, msg_type, topic, qos):
        self.created_publishers.append(topic)
        return MagicMock()


_rclpy_node_mod = MagicMock()
_rclpy_node_mod.Node = _FakeNode
sys.modules.setdefault("rclpy", MagicMock())
sys.modules["rclpy.node"] = _rclpy_node_mod
sys.modules.setdefault("geometry_msgs", MagicMock())
sys.modules.setdefault("geometry_msgs.msg", MagicMock())

# Add the package's src/ directory (co-located: test/ → package root → src/).
_natnet_src = Path(__file__).resolve().parent.parent / "src"
if str(_natnet_src) not in sys.path:
    sys.path.insert(0, str(_natnet_src))

from vision_pose_converter_node import VisionPoseConverterNode  # noqa: E402


# ---------------------------------------------------------------------------
# Load natnet_ros2.launch.py with its heavy launch/ROS deps stubbed, so the
# pure flattening helpers can be unit-tested without a ROS install.
# ---------------------------------------------------------------------------

for _mod in (
    "ament_index_python",
    "ament_index_python.packages",
    "launch",
    "launch.actions",
    "launch.launch_description_sources",
    "launch.substitutions",
    "launch_ros",
    "launch_ros.actions",
):
    sys.modules.setdefault(_mod, MagicMock())

# yaml is only needed by _load_natnet_config (not the flattening helpers); stub it
# if PyYAML is absent so the launch module still imports in a minimal unit env.
try:
    import yaml  # noqa: F401
except ImportError:
    sys.modules.setdefault("yaml", MagicMock())

_launch_path = Path(__file__).resolve().parent.parent / "launch" / "natnet_ros2.launch.py"
_spec = importlib.util.spec_from_file_location("natnet_ros2_launch_under_test", _launch_path)
natnet_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(natnet_launch)


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


# ---------------------------------------------------------------------------
# VisionPoseConverterNode — configurable input/output topics
# ---------------------------------------------------------------------------

@pytest.mark.unit
def test_vision_pose_converter_default_topics():
    """Defaults reproduce the historical relative (remappable) topic names."""
    node = VisionPoseConverterNode()
    assert node.created_subscriptions == ["input_pose"]
    assert node.created_publishers == ["output_pose", "output_pose_cov"]


@pytest.mark.unit
def test_vision_pose_converter_topic_overrides_applied():
    """When the topic params are set, sub/pub use those exact names."""
    _FakeNode._overrides = {
        "input_topic": "/robot_2/perception/optitrack/drone/pose_cov",
        "output_pose_topic": "/robot_2/custom/vision/pose",
        "output_pose_cov_topic": "/robot_2/custom/vision/pose_cov",
    }
    try:
        node = VisionPoseConverterNode()
    finally:
        _FakeNode._overrides = {}
    assert node.created_subscriptions == ["/robot_2/perception/optitrack/drone/pose_cov"]
    assert node.created_publishers == [
        "/robot_2/custom/vision/pose",
        "/robot_2/custom/vision/pose_cov",
    ]


# ---------------------------------------------------------------------------
# natnet_ros2.launch.py — pure config-flattening helpers
# ---------------------------------------------------------------------------

@pytest.mark.unit
def test_expand_env_uses_default_when_unset(monkeypatch):
    monkeypatch.delenv("NATNET_SERVER_IP", raising=False)
    assert natnet_launch._expand_env("$(env NATNET_SERVER_IP 172.31.0.200)") == "172.31.0.200"


@pytest.mark.unit
def test_expand_env_uses_environment_value(monkeypatch):
    monkeypatch.setenv("NATNET_SERVER_IP", "10.0.0.5")
    assert natnet_launch._expand_env("$(env NATNET_SERVER_IP 172.31.0.200)") == "10.0.0.5"


@pytest.mark.unit
def test_namespaced_strips_and_prefixes():
    assert natnet_launch._namespaced("robot_1", "perception/optitrack/drone") == \
        "/robot_1/perception/optitrack/drone"
    assert natnet_launch._namespaced("robot_2", "/already/abs") == "/robot_2/already/abs"


@pytest.mark.unit
def test_build_node_params_flattens_bodies():
    server = {"server_ip": "1.2.3.4", "command_port": 1510, "connection_type": "unicast"}
    profile = {
        "bodies": [
            {
                "rigid_body_name": "Drone",
                "id": 1,
                "topic": "perception/optitrack/drone",
                "pose": True,
                "pose_cov": True,
                "position_covariance": [9.0] * 9,
                "orientation_covariance": [8.0] * 9,
            },
            {
                "rigid_body_name": "Target",
                "id": 100,
                "topic": "perception/optitrack/target",
                "pose": True,
                "pose_cov": False,
            },
        ]
    }
    params = natnet_launch._build_node_params(server, profile)

    assert params["server_ip"] == "1.2.3.4"
    assert params["body_names"] == ["Drone", "Target"]
    assert params["body_ids"] == [1, 100]
    assert params["body_topics"] == ["perception/optitrack/drone", "perception/optitrack/target"]
    assert params["body_pose"] == [True, True]
    assert params["body_pose_cov"] == [True, False]
    # 9 floats per body, flattened in body order.
    assert len(params["body_position_covariance"]) == 18
    assert params["body_position_covariance"][:9] == [9.0] * 9
    # Target omitted its covariance → built-in default fills its slice.
    assert params["body_position_covariance"][9:] == natnet_launch._DEFAULT_POSITION_COVARIANCE


@pytest.mark.unit
def test_build_node_params_expands_env_in_body_name_and_id(monkeypatch):
    """Body name/id accept $(env ...) so a site can retarget the tracked rigid body
    without editing the config; the id must still come out as an int."""
    monkeypatch.setenv("NATNET_BODY_NAME", "Hawk")
    monkeypatch.setenv("NATNET_BODY_ID", "9")
    profile = {"bodies": [{
        "rigid_body_name": "$(env NATNET_BODY_NAME Drone)",
        "id": "$(env NATNET_BODY_ID 1)",
        "topic": "perception/optitrack/drone",
    }]}
    params = natnet_launch._build_node_params({}, profile)
    assert params["body_names"] == ["Hawk"]
    assert params["body_ids"] == [9]


@pytest.mark.unit
def test_build_node_params_body_env_defaults_match_emulator(monkeypatch):
    """Unset env → the in-sim NatNet emulator's body ("Drone", id 1), so the sim path
    works with no override. A mismatch here means a connected client that never
    publishes, since the NatNet client filters frames by numeric id."""
    monkeypatch.delenv("NATNET_BODY_NAME", raising=False)
    monkeypatch.delenv("NATNET_BODY_ID", raising=False)
    profile = {"bodies": [{
        "rigid_body_name": "$(env NATNET_BODY_NAME Drone)",
        "id": "$(env NATNET_BODY_ID 1)",
        "topic": "perception/optitrack/drone",
    }]}
    params = natnet_launch._build_node_params({}, profile)
    assert params["body_names"] == ["Drone"]
    assert params["body_ids"] == [1]


@pytest.mark.unit
def test_build_node_params_empty_profile():
    """A robot with no profile yields empty body arrays (node tracks nothing)."""
    params = natnet_launch._build_node_params({}, {})
    assert params["body_names"] == []
    assert params["body_ids"] == []
    assert params["body_position_covariance"] == []
