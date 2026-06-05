# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Unit tests for NatNet wire serializers (no network)."""

from __future__ import annotations

import ctypes
import struct
import sys
from pathlib import Path

import pytest

_EXT_ROOT = Path(__file__).resolve().parents[1]
if str(_EXT_ROOT) not in sys.path:
    sys.path.insert(0, str(_EXT_ROOT))

from optitrack.natnet.emulator import NatNetUnicastServer, TransmissionType
from optitrack.natnet.emulator.server import natnet_data_types as dt
from optitrack.natnet.emulator.server import natnet_model_types as mt
from optitrack.natnet.emulator.server import natnet_server_types as st
from optitrack.natnet.emulator.server.natnet_common import ModelLimits


pytestmark = pytest.mark.unit


# =============================================================================
# natnet_server_types — transport / handshake
# =============================================================================


def test_packet_header_pack_size_and_endianness():
    header = st.sPacketHeader(
        iMessage=int(st.MessageId.NAT_FRAMEOFDATA),
        nDataBytes=42,
    )
    packed = header.pack()

    assert len(packed) == ctypes.sizeof(st.sPacketHeader) == 4
    message_id, payload_len = struct.unpack("<HH", packed)
    assert message_id == int(st.MessageId.NAT_FRAMEOFDATA)
    assert payload_len == 42


def test_packet_header_length_exceeds_max_packetsize():
    # Encoders can lie in nDataBytes; receivers must bound reads separately.
    header = st.sPacketHeader(
        iMessage=int(st.MessageId.NAT_FRAMEOFDATA),
        nDataBytes=ModelLimits.MAX_PACKETSIZE + 1,
    )
    packed = header.pack()

    _message_id, payload_len = struct.unpack("<HH", packed)
    assert payload_len == ModelLimits.MAX_PACKETSIZE + 1


# =============================================================================
# natnet_data_types — frame streaming (NAT_FRAMEOFDATA payload)
# =============================================================================


def test_empty_frame_pack_length_is_stable():
    frame = dt.sFrameOfMocapData()
    frame.iFrame = 0

    packed = frame.pack()

    # 9 count fields + timestamps + params; all collections empty.
    assert len(packed) == 86
    assert packed == frame.pack()


def test_single_rigid_body_frame_contains_id_and_params():
    frame = dt.sFrameOfMocapData()
    frame.iFrame = 7
    frame.nRigidBodies = 1
    rb = frame.RigidBodies[0]
    rb.ID = 1
    rb.x, rb.y, rb.z = 1.0, 2.0, 3.0
    rb.qx, rb.qy, rb.qz, rb.qw = 0.0, 0.0, 0.0, 1.0
    rb.params = 0x01

    packed = frame.pack()

    assert len(packed) == 86 + 38
    body_offset = 4 + 4 + 4 + 4  # iFrame, nMarkerSets, nOtherMarkers, nRigidBodies
    body_id, = struct.unpack_from("<i", packed, body_offset)
    params_offset = body_offset + 4 + 8 * 4  # id + 8 floats (incl. MeanError)
    params, = struct.unpack_from("<h", packed, params_offset)
    assert body_id == 1
    assert params == 0x01


def test_frame_pack_rejects_rigid_body_count_above_max():
    frame = dt.sFrameOfMocapData()
    frame.nRigidBodies = ModelLimits.MAX_RIGIDBODIES + 1

    with pytest.raises(IndexError):
        frame.pack()


def test_frame_pack_negative_collection_counts_encode_without_validation():
    # Encoder misuse: negative counts serialize to the wire without raising.
    frame = dt.sFrameOfMocapData()
    frame.nRigidBodies = -1

    packed = frame.pack()

    n_rigid_bodies, = struct.unpack_from("<i", packed, 12)
    assert n_rigid_bodies == -1


# =============================================================================
# natnet_server_types — server description (NAT_SERVERINFO payload)
# =============================================================================


def test_server_description_pack_includes_connection_fields():
    server = NatNetUnicastServer(
        local_interface="127.0.0.1",
        transmission_type=TransmissionType.UNICAST,
        multicast_address=None,
        command_port=1510,
        data_port=1511,
    )
    description = server.server_description
    packed = description.pack()

    assert len(packed) == ctypes.sizeof(st.sServerDescription)
    assert description.HostPresent is True
    assert description.bConnectionInfoValid is True
    assert description.ConnectionDataPort == 1511
    assert description.ConnectionMulticast is False
    assert description.NatNetVersion[0] == 4
    assert description.NatNetVersion[1] == 4
    assert description.szHostApp.startswith(b"Motive")


def test_server_init_rejects_invalid_local_interface():
    with pytest.raises(ValueError, match="Invalid local interface IP address"):
        NatNetUnicastServer(local_interface="not-an-ip")


def test_server_init_rejects_identical_command_and_data_ports():
    with pytest.raises(ValueError, match="Command port and data port must be different"):
        NatNetUnicastServer(
            local_interface="127.0.0.1",
            transmission_type=TransmissionType.UNICAST,
            command_port=1510,
            data_port=1510,
        )


# =============================================================================
# natnet_model_types — rigid body description (MODELDEF entry body)
# =============================================================================


def test_rigid_body_description_pack_zero_markers():
    rb = mt.sRigidBodyDescription()
    rb.szName = b"Drone"
    rb.ID = 1
    rb.parentID = -1
    rb.offsetqw = 1.0
    rb.nMarkers = 0

    packed = rb.pack()

    assert packed.startswith(b"Drone\x00")
    body_id, parent_id = struct.unpack_from("<ii", packed, len(b"Drone\x00"))
    assert body_id == 1
    assert parent_id == -1
    pos = struct.unpack_from("<3f", packed, len(b"Drone\x00") + 8)
    rot = struct.unpack_from("<4f", packed, len(b"Drone\x00") + 8 + 12)
    n_markers, = struct.unpack_from("<i", packed, len(b"Drone\x00") + 8 + 12 + 16)
    assert pos == (0.0, 0.0, 0.0)
    assert rot == (0.0, 0.0, 0.0, 1.0)
    assert n_markers == 0


def test_rigid_body_description_pack_rejects_marker_count_above_max():
    rb = mt.sRigidBodyDescription()
    rb.szName = b"Drone"
    rb.nMarkers = ModelLimits.MAX_RBMARKERS + 1

    with pytest.raises(IndexError):
        rb.pack()


def test_rigid_body_description_pack_negative_marker_count_encodes_without_validation():
    rb = mt.sRigidBodyDescription()
    rb.szName = b"Drone"
    rb.nMarkers = -1

    packed = rb.pack()

    n_markers, = struct.unpack_from("<i", packed, len(b"Drone\x00") + 8 + 12 + 16)
    assert n_markers == -1


def test_rigid_body_description_pack_empty_name():
    rb = mt.sRigidBodyDescription()
    rb.nMarkers = 0

    packed = rb.pack()

    assert packed.startswith(b"\x00")


# =============================================================================
# natnet_model_types — data descriptions (NAT_MODELDEF payload)
# =============================================================================


def test_data_descriptions_pack_single_rigid_body():
    descriptions = mt.sDataDescriptions()
    descriptions.nDataDescriptions = 1
    desc = descriptions.arrDataDescriptions[0]
    desc.type = int(mt.DataDescriptors.Descriptor_RigidBody)
    desc.RigidBodyDescription.szName = b"Drone"
    desc.RigidBodyDescription.ID = 1
    desc.RigidBodyDescription.parentID = -1
    desc.RigidBodyDescription.offsetqw = 1.0
    desc.RigidBodyDescription.nMarkers = 0

    payload = descriptions.pack()

    n_descriptions, = struct.unpack_from("<i", payload, 0)
    desc_type, size_bytes = struct.unpack_from("<ii", payload, 4)
    assert n_descriptions == 1
    assert desc_type == int(mt.DataDescriptors.Descriptor_RigidBody)
    body = payload[12 : 12 + size_bytes]
    assert body.startswith(b"Drone\x00")
    assert len(body) == size_bytes


def test_data_descriptions_pack_empty():
    descriptions = mt.sDataDescriptions()
    descriptions.nDataDescriptions = 0

    payload = descriptions.pack()

    n_descriptions, = struct.unpack_from("<i", payload, 0)
    assert n_descriptions == 0
    assert len(payload) == 4


def test_data_descriptions_pack_multiple_rigid_bodies():
    descriptions = mt.sDataDescriptions()
    descriptions.nDataDescriptions = 3

    bodies = [
        ("Drone", 1, -1),
        ("Target", 2, -1),
        ("Payload", 3, 1),
    ]
    for index, (name, body_id, parent_id) in enumerate(bodies):
        desc = descriptions.arrDataDescriptions[index]
        desc.type = int(mt.DataDescriptors.Descriptor_RigidBody)
        desc.RigidBodyDescription.szName = name.encode("utf-8")
        desc.RigidBodyDescription.ID = body_id
        desc.RigidBodyDescription.parentID = parent_id
        desc.RigidBodyDescription.offsetqw = 1.0
        desc.RigidBodyDescription.nMarkers = 0

    payload = descriptions.pack()

    n_descriptions, = struct.unpack_from("<i", payload, 0)
    assert n_descriptions == 3

    offset = 4
    parsed = []
    for expected_name, expected_id, expected_parent_id in bodies:
        desc_type, size_bytes = struct.unpack_from("<ii", payload, offset)
        offset += 8
        body = payload[offset : offset + size_bytes]
        offset += size_bytes

        assert desc_type == int(mt.DataDescriptors.Descriptor_RigidBody)
        assert body.startswith(expected_name.encode("utf-8") + b"\x00")
        name_len = len(expected_name) + 1
        body_id, parent_id = struct.unpack_from("<ii", body, name_len)
        parsed.append((body_id, parent_id))

    assert parsed == [(1, -1), (2, -1), (3, 1)]
    assert offset == len(payload)


def test_data_description_pack_rejects_unsupported_type():
    desc = mt.sDataDescription()
    desc.type = 999

    with pytest.raises(ValueError, match="Unsupported data description type"):
        desc.pack()


def test_data_description_pack_rejects_negative_type():
    desc = mt.sDataDescription()
    desc.type = -1

    with pytest.raises(ValueError, match="Unsupported data description type"):
        desc.pack()


def test_data_descriptions_pack_rejects_count_above_max():
    descriptions = mt.sDataDescriptions()
    for i in range(ModelLimits.MAX_MODELS):
        desc = descriptions.arrDataDescriptions[i]
        desc.type = int(mt.DataDescriptors.Descriptor_RigidBody)
        desc.RigidBodyDescription.szName = b"Body"
        desc.RigidBodyDescription.ID = i + 1
        desc.RigidBodyDescription.nMarkers = 0
    descriptions.nDataDescriptions = ModelLimits.MAX_MODELS + 1

    with pytest.raises(IndexError):
        descriptions.pack()


def test_data_descriptions_pack_negative_count_encodes_without_validation():
    descriptions = mt.sDataDescriptions()
    descriptions.nDataDescriptions = -1

    payload = descriptions.pack()

    n_descriptions, = struct.unpack_from("<i", payload, 0)
    assert n_descriptions == -1
    assert len(payload) == 4
