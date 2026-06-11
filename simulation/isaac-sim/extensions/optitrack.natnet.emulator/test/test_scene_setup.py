# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Hermetic tests for the launch-script helper (no USD / Kit).

``build_drone_config`` is the pure half of ``scene_setup`` used by the Pegasus
example launch scripts to author one rigid body per drone base_link on load.
"""

from __future__ import annotations

import pytest

from optitrack.natnet.emulator.isaac.config import NatNetInterfaceConfig
from optitrack.natnet.emulator.isaac.scene_setup import (
    DEFAULT_INTERFACE_PATH,
    build_drone_config,
)

pytestmark = pytest.mark.unit


def test_default_interface_path():
    assert DEFAULT_INTERFACE_PATH == "/World/NatNetInterface"


def test_single_drone_maps_name_id_and_target():
    cfg = build_drone_config([("Drone", 1, "/World/base_link")])
    assert isinstance(cfg, NatNetInterfaceConfig)
    assert cfg.server_enabled is True
    assert cfg.server_ip == "172.31.0.200"  # Isaac container default
    assert cfg.mode == "unicast"
    assert len(cfg.bodies) == 1
    body = cfg.bodies[0]
    assert body.rigid_body_name == "Drone"
    assert body.streaming_id == 1
    assert body.target_prim == "/World/base_link"


def test_multi_drone_unique_names_and_ids():
    drones = [(f"Drone{i}", i, f"/World/drone{i}/base_link") for i in range(1, 4)]
    cfg = build_drone_config(drones)
    assert len(cfg.bodies) == 3
    assert [b.streaming_id for b in cfg.bodies] == [1, 2, 3]
    assert [b.rigid_body_name for b in cfg.bodies] == ["Drone1", "Drone2", "Drone3"]
    assert [b.target_prim for b in cfg.bodies] == [
        "/World/drone1/base_link",
        "/World/drone2/base_link",
        "/World/drone3/base_link",
    ]
    # Must pass the config validator (unique names/ids, valid ports).
    cfg.validate()


def test_no_drones_yields_empty_catalog():
    cfg = build_drone_config([])
    assert cfg.bodies == []
    cfg.validate()


def test_server_params_are_forwarded():
    cfg = build_drone_config(
        [("Drone", 1, "/World/base_link")],
        server_ip="10.0.0.5",
        command_port=2510,
        data_port=2511,
        publish_rate=120.0,
        server_enabled=False,
    )
    assert cfg.server_ip == "10.0.0.5"
    assert cfg.command_port == 2510
    assert cfg.data_port == 2511
    assert cfg.publish_rate == 120.0
    assert cfg.server_enabled is False


def test_duplicate_names_raise():
    with pytest.raises(ValueError):
        build_drone_config(
            [("Drone", 1, "/World/a"), ("Drone", 2, "/World/b")]
        )


def test_duplicate_ids_raise():
    with pytest.raises(ValueError):
        build_drone_config(
            [("DroneA", 1, "/World/a"), ("DroneB", 1, "/World/b")]
        )
