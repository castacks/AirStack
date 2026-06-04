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
from optitrack.natnet.emulator.server import natnet_server_types as st


pytestmark = pytest.mark.unit


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
