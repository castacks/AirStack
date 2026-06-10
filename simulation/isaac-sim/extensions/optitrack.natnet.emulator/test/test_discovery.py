# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Light unit tests for Commit 2 detection/printing (the pure formatter)."""

from __future__ import annotations

import pytest

from optitrack.natnet.emulator.isaac.config import BodyBinding, NatNetInterfaceConfig
from optitrack.natnet.emulator.isaac.manager import format_interface

pytestmark = pytest.mark.unit


def test_format_interface_includes_server_params_and_bodies():
    cfg = NatNetInterfaceConfig(
        server_ip="10.0.0.5",
        bodies=[BodyBinding("Drone", "/World/base_link", 1)],
    )
    text = format_interface("/World/NatNetInterface", cfg)
    assert "/World/NatNetInterface" in text
    assert "10.0.0.5" in text
    assert "Drone" in text
    assert "/World/base_link" in text


def test_format_interface_reports_no_bodies():
    text = format_interface("/World/NatNetInterface", NatNetInterfaceConfig())
    assert "(none)" in text
