# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Commit 3 — hermetic catalog-builder tests (no USD, no Kit).

Covers no / single / multiple bodies, field fidelity on the wire, name truncation,
the MAX_MODELS guard, and duplicate-target detection.
"""

from __future__ import annotations

import struct

import pytest

from optitrack.natnet.emulator.isaac.catalog import build_catalog, find_duplicate_targets
from optitrack.natnet.emulator.isaac.config import BodyBinding, NatNetInterfaceConfig
from optitrack.natnet.emulator.server import natnet_model_types as mt
from optitrack.natnet.emulator.server.natnet_common import ModelLimits

pytestmark = pytest.mark.unit


def _unpack_bodies(payload: bytes):
    """Decode a packed sDataDescriptions into [(name, id, parentID), ...]."""
    (n,) = struct.unpack_from("<i", payload, 0)
    offset = 4
    bodies = []
    for _ in range(n):
        desc_type, size_bytes = struct.unpack_from("<ii", payload, offset)
        offset += 8
        body = payload[offset : offset + size_bytes]
        offset += size_bytes
        assert desc_type == int(mt.DataDescriptors.Descriptor_RigidBody)
        name, rest = body.split(b"\x00", 1)
        body_id, parent_id = struct.unpack_from("<ii", rest, 0)
        bodies.append((name, body_id, parent_id))
    return n, bodies


def test_no_bodies_packs_empty_catalog():
    catalog = build_catalog(NatNetInterfaceConfig())
    assert catalog.nDataDescriptions == 0
    n, bodies = _unpack_bodies(catalog.pack())
    assert n == 0
    assert bodies == []


def test_single_body_field_fidelity():
    cfg = NatNetInterfaceConfig(
        bodies=[BodyBinding("Drone", "/World/base_link", streaming_id=7, parent_id=-1)]
    )
    catalog = build_catalog(cfg)
    assert catalog.nDataDescriptions == 1
    n, bodies = _unpack_bodies(catalog.pack())
    assert n == 1
    assert bodies == [(b"Drone", 7, -1)]


def test_multiple_bodies_preserve_order_and_fields():
    cfg = NatNetInterfaceConfig(
        bodies=[
            BodyBinding("Alpha", "/World/a", streaming_id=1, parent_id=-1),
            BodyBinding("Bravo", "/World/b", streaming_id=2, parent_id=1),
            BodyBinding("Charlie", "/World/c", streaming_id=5, parent_id=2),
        ]
    )
    catalog = build_catalog(cfg)
    assert catalog.nDataDescriptions == 3
    _n, bodies = _unpack_bodies(catalog.pack())
    assert bodies == [
        (b"Alpha", 1, -1),
        (b"Bravo", 2, 1),
        (b"Charlie", 5, 2),
    ]


def test_long_name_is_truncated_not_rejected():
    long_name = "X" * 400
    cfg = NatNetInterfaceConfig(bodies=[BodyBinding(long_name, "/World/x", streaming_id=1)])
    catalog = build_catalog(cfg)
    _n, bodies = _unpack_bodies(catalog.pack())
    name = bodies[0][0]
    assert name == b"X" * (int(ModelLimits.MAX_NAMELENGTH) - 1)


def test_too_many_bodies_raises():
    bodies = [BodyBinding(f"B{i}", f"/World/b{i}", streaming_id=i + 1) for i in range(int(ModelLimits.MAX_MODELS) + 1)]
    cfg = NatNetInterfaceConfig(bodies=bodies)
    with pytest.raises(ValueError):
        build_catalog(cfg)


def test_find_duplicate_targets_none():
    cfg = NatNetInterfaceConfig(
        bodies=[BodyBinding("A", "/World/a", 1), BodyBinding("B", "/World/b", 2)]
    )
    assert find_duplicate_targets(cfg) == []


def test_find_duplicate_targets_flags_shared_prim():
    cfg = NatNetInterfaceConfig(
        bodies=[
            BodyBinding("A", "/World/shared", 1),
            BodyBinding("B", "/World/shared", 2),
            BodyBinding("C", "/World/other", 3),
        ]
    )
    assert find_duplicate_targets(cfg) == ["/World/shared"]


def test_find_duplicate_targets_ignores_empty_targets():
    cfg = NatNetInterfaceConfig(
        bodies=[BodyBinding("A", "", 1), BodyBinding("B", "", 2)]
    )
    assert find_duplicate_targets(cfg) == []
