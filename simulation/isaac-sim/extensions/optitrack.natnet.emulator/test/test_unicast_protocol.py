# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""UDP loopback protocol tests for NatNetUnicastServer."""

from __future__ import annotations

import ctypes
import socket
import struct
import sys
import time
from pathlib import Path

import pytest

_TEST_DIR = Path(__file__).resolve().parent
if str(_TEST_DIR) not in sys.path:
    sys.path.insert(0, str(_TEST_DIR))

from natnet_test_helpers import NatNetTestClient, running_unicast_server

from optitrack.natnet.emulator.server import natnet_data_types as dt
from optitrack.natnet.emulator.server import natnet_server_types as st


pytestmark = pytest.mark.unit


def test_nat_connect_receives_serverinfo_on_command_port():
    with running_unicast_server() as (server, command_port):
        client = NatNetTestClient()
        try:
            client.send_message(command_port, st.MessageId.NAT_CONNECT)
            message_id, payload, addr = client.recv_message()
        finally:
            client.close()

        assert addr[1] == command_port
        assert message_id == int(st.MessageId.NAT_SERVERINFO)
        assert len(payload) == ctypes.sizeof(st.sServerDescription)

        description = st.sServerDescription.from_buffer_copy(payload)
        assert description.HostPresent is True
        assert description.ConnectionDataPort == server.data_port


def test_enqueued_frame_arrives_on_command_port():
    with running_unicast_server(publish_rate=200) as (_server, command_port):
        client = NatNetTestClient(timeout=3.0)
        try:
            client.send_message(command_port, st.MessageId.NAT_CONNECT)
            client.recv_message()

            frame = dt.sFrameOfMocapData()
            frame.iFrame = 42
            _server.enqueue_mocap_data(frame)

            deadline = time.time() + 2.0
            got_frame = False
            while time.time() < deadline:
                try:
                    message_id, _payload, addr = client.recv_message()
                except socket.timeout:
                    break
                if message_id == int(st.MessageId.NAT_FRAMEOFDATA):
                    assert addr[1] == command_port
                    got_frame = True
                    break
        finally:
            client.close()

        assert got_frame, "Expected NAT_FRAMEOFDATA on the command socket"


def test_no_frame_sent_without_connected_client():
    with running_unicast_server(publish_rate=200) as (server, command_port):
        listener = NatNetTestClient(timeout=0.5)
        try:
            frame = dt.sFrameOfMocapData()
            frame.iFrame = 1
            server.enqueue_mocap_data(frame)
            time.sleep(0.05)

            with pytest.raises(socket.timeout):
                listener.recv_message()
        finally:
            listener.close()

        assert command_port  # server bound; no clients registered


def test_connect_registers_client_for_subsequent_frames():
    with running_unicast_server(publish_rate=200) as (server, command_port):
        client_a = NatNetTestClient(timeout=3.0)
        client_b = NatNetTestClient(timeout=0.5)
        try:
            client_a.send_message(command_port, st.MessageId.NAT_CONNECT)
            client_a.recv_message()

            frame = dt.sFrameOfMocapData()
            frame.iFrame = 99
            server.enqueue_mocap_data(frame)

            message_id, payload, addr = client_a.recv_message()
            assert message_id == int(st.MessageId.NAT_FRAMEOFDATA)
            frame_num, = struct.unpack_from("<i", payload, 0)
            assert frame_num == 99

            with pytest.raises(socket.timeout):
                client_b.recv_message()
        finally:
            client_a.close()
            client_b.close()
