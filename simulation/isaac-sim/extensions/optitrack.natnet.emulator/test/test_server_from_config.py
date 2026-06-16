# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Manager → real ``NatNetUnicastServer``: MODELDEF handshake and frame delivery."""

from __future__ import annotations

import struct
import time

import pytest

from natnet_test_helpers import NatNetTestClient, ephemeral_udp_port

from optitrack.natnet.emulator.isaac.catalog import build_catalog
from optitrack.natnet.emulator.isaac.config import BodyBinding, NatNetInterfaceConfig
from optitrack.natnet.emulator.isaac.manager import NatNetServerManager
from optitrack.natnet.emulator.server import natnet_server_types as st

pytestmark = pytest.mark.unit


def _loopback_config(bodies):
    command_port = ephemeral_udp_port()
    data_port = ephemeral_udp_port()
    while data_port == command_port:
        data_port = ephemeral_udp_port()
    return NatNetInterfaceConfig(
        server_ip="127.0.0.1",
        command_port=command_port,
        data_port=data_port,
        bodies=bodies,
    )


def _request_modeldef(command_port):
    client = NatNetTestClient(timeout=2.0)
    try:
        client.send_message(command_port, st.MessageId.NAT_CONNECT)
        client.recv_message()
        client.send_message(command_port, st.MessageId.NAT_REQUEST_MODELDEF)
        message_id, payload, _addr = client.recv_message()
    finally:
        client.close()
    assert message_id == int(st.MessageId.NAT_MODELDEF)
    return payload


def test_server_from_config_serves_matching_catalog():
    cfg = _loopback_config(
        [
            BodyBinding("Alpha", "/World/a", streaming_id=1),
            BodyBinding("Bravo", "/World/b", streaming_id=2, parent_id=1),
        ]
    )
    mgr = NatNetServerManager()  # default (real) factory
    assert mgr.start_server(cfg) is True
    try:
        time.sleep(0.05)
        payload = _request_modeldef(cfg.command_port)
    finally:
        mgr.stop_server()

    assert payload == build_catalog(cfg).pack()
    (n,) = struct.unpack_from("<i", payload, 0)
    assert n == 2
    assert b"Alpha\x00" in payload
    assert b"Bravo\x00" in payload


def test_stop_frees_port_for_restart():
    cfg = _loopback_config([BodyBinding("Drone", "/World/base_link", 1)])
    mgr = NatNetServerManager()

    assert mgr.start_server(cfg) is True
    mgr.stop_server()
    time.sleep(0.05)

    # Re-binding the same ports must succeed (clean shutdown released them).
    assert mgr.start_server(cfg) is True
    try:
        time.sleep(0.05)
        payload = _request_modeldef(cfg.command_port)
        assert b"Drone\x00" in payload
    finally:
        mgr.stop_server()
