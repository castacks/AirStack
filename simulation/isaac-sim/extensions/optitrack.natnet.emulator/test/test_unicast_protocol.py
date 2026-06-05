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

from optitrack.natnet.emulator import NatNetUnicastServer, TransmissionType
from optitrack.natnet.emulator.server import natnet_data_types as dt
from optitrack.natnet.emulator.server import natnet_server_types as st


pytestmark = pytest.mark.unit


# =============================================================================
# Handshake — NAT_CONNECT → NAT_SERVERINFO (command socket)
# =============================================================================


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


# =============================================================================
# Frame streaming — NAT_FRAMEOFDATA on command socket (libNatNet 4.4 unicast)
# =============================================================================


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


# =============================================================================
# Client registration — connected vs unconnected endpoints
# =============================================================================


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


# =============================================================================
# Malformed datagrams — handler-level (no network)
# =============================================================================


def test_truncated_datagram_is_ignored():
    server = NatNetUnicastServer(
        local_interface="127.0.0.1",
        transmission_type=TransmissionType.UNICAST,
        command_port=1510,
        data_port=1511,
    )
    server._handle_command_request(b"\x00\x01", ("127.0.0.1", 40000))
    assert len(server.connected_clients) == 0


def test_empty_datagram_is_ignored():
    server = NatNetUnicastServer(
        local_interface="127.0.0.1",
        transmission_type=TransmissionType.UNICAST,
        command_port=1510,
        data_port=1511,
    )
    server._handle_command_request(b"", ("127.0.0.1", 40000))
    assert len(server.connected_clients) == 0


# =============================================================================
# Malformed datagrams — unregistered client (no reply)
# =============================================================================


def test_unknown_message_id_from_unregistered_client_gets_no_reply():
    with running_unicast_server() as (_server, command_port):
        client = NatNetTestClient(timeout=0.5)
        try:
            client.send_message(command_port, 999)
            with pytest.raises(socket.timeout):
                client.recv_message()
        finally:
            client.close()


def test_request_modeldef_without_connect_gets_no_reply():
    with running_unicast_server() as (_server, command_port):
        client = NatNetTestClient(timeout=0.5)
        try:
            client.send_message(command_port, st.MessageId.NAT_REQUEST_MODELDEF)
            with pytest.raises(socket.timeout):
                client.recv_message()
        finally:
            client.close()


# =============================================================================
# Malformed datagrams — handshake edge cases
# =============================================================================


def test_lie_about_payload_length_in_header_still_handshake_on_connect():
    with running_unicast_server() as (_server, command_port):
        client = NatNetTestClient(timeout=2.0)
        try:
            client.send_header_only(
                command_port,
                st.MessageId.NAT_CONNECT,
                declared_payload_len=65535,
            )
            message_id, payload, addr = client.recv_message()
        finally:
            client.close()

        assert addr[1] == command_port
        assert message_id == int(st.MessageId.NAT_SERVERINFO)
        assert len(payload) == ctypes.sizeof(st.sServerDescription)


# =============================================================================
# Malformed datagrams — registered client & recovery
# =============================================================================


def test_unknown_message_from_registered_client_gets_no_reply():
    with running_unicast_server() as (server, command_port):
        client = NatNetTestClient(timeout=0.5)
        try:
            client.send_message(command_port, st.MessageId.NAT_CONNECT)
            client.recv_message()

            client.send_message(command_port, 999)
            with pytest.raises(socket.timeout):
                client.recv_message()

            assert len(server.connected_clients) == 1
        finally:
            client.close()


def test_server_survives_malformed_burst_then_valid_connect():
    with running_unicast_server() as (server, command_port):
        client = NatNetTestClient(timeout=2.0)
        try:
            client.send_raw(b"", command_port)
            client.send_raw(b"\xff", command_port)
            client.send_header_only(command_port, 999, declared_payload_len=50000)
            client.send_message(command_port, st.MessageId.NAT_REQUEST_MODELDEF)

            client.send_message(command_port, st.MessageId.NAT_CONNECT)
            message_id, _payload, _addr = client.recv_message()
        finally:
            client.close()

        assert message_id == int(st.MessageId.NAT_SERVERINFO)
        assert len(server.connected_clients) == 1
