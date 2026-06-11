# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""USD authoring/read round-trip tests for the NatNet interface prim.

Guarded by ``pytest.importorskip("pxr")`` — these run wherever USD Python is
available (the Isaac container, or CI when ``usd-core`` is installed from
``tests/requirements.txt``) and skip cleanly otherwise.
"""

from __future__ import annotations

import pytest

pytest.importorskip("pxr")

from pxr import Usd  # noqa: E402

from optitrack.natnet.emulator.isaac import (  # noqa: E402
    author_interface,
    find_interfaces,
    read_interface,
)
from optitrack.natnet.emulator.isaac.config import BodyBinding, NatNetInterfaceConfig  # noqa: E402

pytestmark = pytest.mark.unit

_CONFIG = {
    "server_enabled": True,
    "server_ip": "172.31.0.200",
    "mode": "unicast",
    "command_port": 1510,
    "data_port": 1511,
    "publish_rate": 100,
    "bodies": {
        "/World/base_link": {"rigid_body_name": "Drone", "streaming_id": 1},
        "/World/target": {"rigid_body_name": "Target", "streaming_id": 2, "parent_id": 1},
    },
}


def _new_stage():
    return Usd.Stage.CreateInMemory()


def test_author_then_find_locates_the_interface():
    stage = _new_stage()
    author_interface(stage, "/World/NatNetInterface", _CONFIG)
    interfaces = find_interfaces(stage)
    assert [p.GetPath().pathString for p in interfaces] == ["/World/NatNetInterface"]


def test_author_read_round_trip():
    stage = _new_stage()
    author_interface(stage, "/World/NatNetInterface", _CONFIG)
    prim = find_interfaces(stage)[0]

    cfg = read_interface(prim)
    assert cfg.server_ip == "172.31.0.200"
    assert cfg.mode == "unicast"
    assert cfg.command_port == 1510
    assert cfg.data_port == 1511

    by_name = {b.rigid_body_name: b for b in cfg.bodies}
    assert by_name["Drone"].target_prim == "/World/base_link"
    assert by_name["Drone"].streaming_id == 1
    assert by_name["Target"].target_prim == "/World/target"
    assert by_name["Target"].parent_id == 1

    # read -> author -> read is stable
    author_interface(stage, "/World/NatNetInterface", cfg)
    assert read_interface(find_interfaces(stage)[0]) == cfg


def test_reauthoring_removes_stale_bodies():
    stage = _new_stage()
    author_interface(stage, "/World/NatNetInterface", _CONFIG)

    single = NatNetInterfaceConfig.from_dict(
        {"bodies": [{"rigid_body_name": "Drone", "target_prim": "/World/base_link", "streaming_id": 1}]}
    )
    author_interface(stage, "/World/NatNetInterface", single)

    cfg = read_interface(find_interfaces(stage)[0])
    assert [b.rigid_body_name for b in cfg.bodies] == ["Drone"]


def test_up_axis_authors_and_reads_back():
    stage = _new_stage()
    # Default (absent) -> Z.
    author_interface(stage, "/World/NatNetInterface", _CONFIG)
    assert read_interface(find_interfaces(stage)[0]).up_axis == "Z"

    # Explicit Y survives the USD round trip.
    cfg = NatNetInterfaceConfig.from_dict({**_CONFIG, "up_axis": "Y"})
    author_interface(stage, "/World/NatNetInterface", cfg)
    assert read_interface(find_interfaces(stage)[0]).up_axis == "Y"


def test_empty_target_round_trips():
    # The UI's "Add body" can create a body with no target yet (set later in the
    # Property panel); it must author and read back cleanly with an empty target.
    stage = _new_stage()
    cfg = NatNetInterfaceConfig(bodies=[BodyBinding("Drone", "", 1)])
    author_interface(stage, "/World/NatNetInterface", cfg)
    read = read_interface(find_interfaces(stage)[0])
    assert read.bodies[0].rigid_body_name == "Drone"
    assert read.bodies[0].target_prim == ""


def test_invalid_config_raises_before_authoring():
    stage = _new_stage()
    with pytest.raises(ValueError):
        author_interface(stage, "/World/NatNetInterface", {"mode": "bogus"})
    assert find_interfaces(stage) == []
