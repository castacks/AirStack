# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Unit tests for server MODELDEF payload cache."""

from __future__ import annotations

import struct

import pytest

from optitrack.natnet.emulator import NatNetUnicastServer, TransmissionType, make_default_drone_catalog
from optitrack.natnet.emulator.server import natnet_model_types as mt


pytestmark = pytest.mark.unit


def test_make_default_drone_catalog_packs_drone_body():
    catalog = make_default_drone_catalog()
    payload = catalog.pack()

    n_descriptions, = struct.unpack_from("<i", payload, 0)
    desc_type, size_bytes = struct.unpack_from("<ii", payload, 4)
    body = payload[12 : 12 + size_bytes]

    assert n_descriptions == 1
    assert desc_type == int(mt.DataDescriptors.Descriptor_RigidBody)
    assert body.startswith(b"Drone\x00")


def test_server_init_loads_default_drone_modeldef_payload():
    server = NatNetUnicastServer(
        local_interface="127.0.0.1",
        transmission_type=TransmissionType.UNICAST,
        multicast_address=None,
    )
    payload = server._get_model_def_payload()

    assert payload == make_default_drone_catalog().pack()
    n_descriptions, = struct.unpack_from("<i", payload, 0)
    assert n_descriptions == 1


def test_set_model_def_payload_replaces_cache():
    server = NatNetUnicastServer(
        local_interface="127.0.0.1",
        transmission_type=TransmissionType.UNICAST,
        multicast_address=None,
    )

    updated = mt.sDataDescriptions()
    updated.nDataDescriptions = 1
    desc = updated.arrDataDescriptions[0]
    desc.type = int(mt.DataDescriptors.Descriptor_RigidBody)
    desc.RigidBodyDescription.szName = b"Target"
    desc.RigidBodyDescription.ID = 2
    desc.RigidBodyDescription.nMarkers = 0
    custom_payload = updated.pack()

    server.set_model_def_payload(custom_payload)
    payload = server._get_model_def_payload()

    assert payload == custom_payload
    body = payload[12:]
    assert body.startswith(b"Target\x00")


def test_set_model_def_from_descriptions_packs_once():
    server = NatNetUnicastServer(
        local_interface="127.0.0.1",
        transmission_type=TransmissionType.UNICAST,
        multicast_address=None,
    )

    catalog = make_default_drone_catalog()
    server.set_model_def_from_descriptions(catalog)

    assert server._get_model_def_payload() == catalog.pack()
