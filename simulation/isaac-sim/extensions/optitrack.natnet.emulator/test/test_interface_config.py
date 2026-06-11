# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Hermetic unit tests for the pure-Python NatNet interface config model.

No USD / Isaac imports — exercises dataclasses, dict normalization, the attribute
name builder, instance-key generation, and validation.
"""

from __future__ import annotations

import pytest

from optitrack.natnet.emulator.isaac.config import (
    BodyBinding,
    NatNetInterfaceConfig,
    body_attr_name,
    make_instance_key,
)

pytestmark = pytest.mark.unit


def test_defaults_match_server_expectations():
    cfg = NatNetInterfaceConfig()
    assert cfg.server_enabled is True
    assert cfg.server_ip == "172.31.0.200"
    assert cfg.mode == "unicast"
    assert cfg.command_port == 1510
    assert cfg.data_port == 1511
    assert cfg.up_axis == "Z"  # Isaac/USD native; matches the reference Motive setup
    assert cfg.bodies == []


def test_up_axis_from_dict_normalizes_case():
    assert NatNetInterfaceConfig.from_dict({"up_axis": "y"}).up_axis == "Y"
    assert NatNetInterfaceConfig.from_dict({"up_axis": "z"}).up_axis == "Z"
    # Absent -> default Z.
    assert NatNetInterfaceConfig.from_dict({}).up_axis == "Z"


def test_up_axis_survives_round_trip():
    cfg = NatNetInterfaceConfig.from_dict({"up_axis": "Y"})
    assert NatNetInterfaceConfig.from_dict(cfg.to_dict()).up_axis == "Y"


def test_from_dict_with_bodies_as_list():
    cfg = NatNetInterfaceConfig.from_dict(
        {
            "server_ip": "10.0.0.5",
            "bodies": [
                {"rigid_body_name": "Drone", "target_prim": "/World/base_link", "streaming_id": 1},
            ],
        }
    )
    assert cfg.server_ip == "10.0.0.5"
    assert len(cfg.bodies) == 1
    assert cfg.bodies[0] == BodyBinding("Drone", "/World/base_link", 1, -1)


def test_from_dict_with_bodies_as_prim_mapping():
    # The "dictionary of prims -> rigid body names and stuff" form.
    cfg = NatNetInterfaceConfig.from_dict(
        {
            "bodies": {
                "/World/base_link": {"rigid_body_name": "Drone", "streaming_id": 1},
                "/World/target": {"rigid_body_name": "Target", "streaming_id": 2},
            }
        }
    )
    by_name = {b.rigid_body_name: b for b in cfg.bodies}
    assert by_name["Drone"].target_prim == "/World/base_link"
    assert by_name["Target"].target_prim == "/World/target"
    assert by_name["Target"].streaming_id == 2


def test_to_dict_round_trip():
    cfg = NatNetInterfaceConfig.from_dict(
        {
            "mode": "multicast",
            "publish_rate": 120,
            "bodies": [{"rigid_body_name": "Drone", "target_prim": "/World/base_link"}],
        }
    )
    restored = NatNetInterfaceConfig.from_dict(cfg.to_dict())
    assert restored == cfg


def test_body_from_dict_requires_target_and_name():
    with pytest.raises(ValueError):
        BodyBinding.from_dict({"rigid_body_name": "Drone"})  # no target_prim
    with pytest.raises(ValueError):
        BodyBinding.from_dict({"target_prim": "/World/base_link"})  # no name


def test_body_attr_name_builder():
    assert body_attr_name("Drone", "streamingId") == "natnet:body:Drone:streamingId"


def test_make_instance_key_sanitizes_and_dedupes():
    used: set[str] = set()
    assert make_instance_key("Drone 1", used) == "Drone_1"
    # collision after sanitization -> numeric suffix
    assert make_instance_key("Drone-1", used) == "Drone_1_1"
    # leading digit gets a safe prefix
    assert make_instance_key("3PO", used).startswith("b_")


def test_assign_instance_keys_are_unique():
    cfg = NatNetInterfaceConfig(
        bodies=[
            BodyBinding("Drone", "/World/a", 1),
            BodyBinding("Drone", "/World/b", 2),  # duplicate display name
        ]
    )
    keys = [k for k, _ in cfg.assign_instance_keys()]
    assert len(set(keys)) == 2


@pytest.mark.parametrize(
    "overrides",
    [
        {"mode": "bogus"},
        {"command_port": 0},
        {"data_port": 70000},
        {"command_port": 1510, "data_port": 1510},
        {"publish_rate": 0},
        {"up_axis": "X"},
        {"up_axis": "bogus"},
    ],
)
def test_validate_rejects_bad_server_config(overrides):
    cfg = NatNetInterfaceConfig(**overrides)
    with pytest.raises(ValueError):
        cfg.validate()


def test_validate_rejects_duplicate_streaming_ids():
    cfg = NatNetInterfaceConfig(
        bodies=[
            BodyBinding("A", "/World/a", 1),
            BodyBinding("B", "/World/b", 1),
        ]
    )
    with pytest.raises(ValueError):
        cfg.validate()


def test_validate_rejects_blank_rigid_body_name():
    cfg = NatNetInterfaceConfig(bodies=[BodyBinding("", "/World/a", 1)])
    with pytest.raises(ValueError):
        cfg.validate()


def test_validate_allows_empty_target():
    # An empty target is valid: a freshly added body to be pointed in the UI/Property panel.
    cfg = NatNetInterfaceConfig(bodies=[BodyBinding("Drone", "", 1)])
    assert cfg.validate() is cfg


def test_validate_accepts_good_config():
    cfg = NatNetInterfaceConfig(
        bodies=[BodyBinding("Drone", "/World/base_link", 1)]
    )
    assert cfg.validate() is cfg
