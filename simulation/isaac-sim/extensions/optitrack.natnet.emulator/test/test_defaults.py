# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Unit tests for hardcoded tracked-body defaults."""

from __future__ import annotations

import pytest

from optitrack.natnet.emulator.defaults import (
    DEFAULT_DRONE_BINDING,
    DEFAULT_TRACKED_BODY_BINDINGS,
)


pytestmark = pytest.mark.unit


def test_default_drone_binding_matches_natnet_ros2_config():
    assert DEFAULT_DRONE_BINDING.name == "Drone"
    assert DEFAULT_DRONE_BINDING.id == 1
    assert DEFAULT_DRONE_BINDING.parent_id == -1
    assert DEFAULT_DRONE_BINDING.prim_path == "/World/base_link"


def test_default_tracked_body_bindings_contains_drone_only():
    assert DEFAULT_TRACKED_BODY_BINDINGS == (DEFAULT_DRONE_BINDING,)
