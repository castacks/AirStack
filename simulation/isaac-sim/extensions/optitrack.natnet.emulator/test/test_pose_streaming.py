# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Loopback: sample_once on a USD prim → NAT_FRAMEOFDATA with sampled position.

Real server + UDP sockets + in-memory stage. Requires pxr.
"""

from __future__ import annotations

import socket
import struct
import time

import pytest

pytest.importorskip("pxr")

from pxr import Gf, Usd, UsdGeom  # noqa: E402

from natnet_test_helpers import NatNetTestClient, ephemeral_udp_port  # noqa: E402

from optitrack.natnet.emulator.isaac.config import BodyBinding, NatNetInterfaceConfig  # noqa: E402
from optitrack.natnet.emulator.isaac.manager import NatNetServerManager  # noqa: E402
from optitrack.natnet.emulator.isaac.usd_bindings import author_interface  # noqa: E402
from optitrack.natnet.emulator.server import natnet_server_types as st  # noqa: E402

pytestmark = pytest.mark.unit


def _decode_first_rigid_body(payload: bytes):
    # iFrame(4) + markersets(count4+size4) + othermarkers(count4+size4) = 20, then
    # rigid bodies: count(4) + size(4) at 20, first body at 28: id + xyz.
    rb_count, _rb_size = struct.unpack_from("<ii", payload, 20)
    body_id, x, y, z = struct.unpack_from("<i3f", payload, 28)
    return rb_count, body_id, (x, y, z)


def test_streamed_frame_carries_sampled_usd_pose():
    stage = Usd.Stage.CreateInMemory()
    xform = UsdGeom.Xform.Define(stage, "/World/base_link")
    xform.AddTranslateOp().Set(Gf.Vec3d(7.0, -3.0, 2.0))

    command_port = ephemeral_udp_port()
    data_port = ephemeral_udp_port()
    while data_port == command_port:
        data_port = ephemeral_udp_port()
    cfg = NatNetInterfaceConfig(
        server_ip="127.0.0.1",
        command_port=command_port,
        data_port=data_port,
        bodies=[BodyBinding("Drone", "/World/base_link", 1)],
    )
    author_interface(stage, "/World/NatNetInterface", cfg)

    mgr = NatNetServerManager()  # real factory
    assert mgr.start_from_stage.__self__ is mgr  # scripting entry exists
    assert mgr.start_server(cfg) is True

    client = NatNetTestClient(timeout=2.0)
    try:
        client.send_message(command_port, st.MessageId.NAT_CONNECT)
        client.recv_message()

        got = None
        deadline = time.time() + 2.0
        while time.time() < deadline:
            mgr.sample_once(stage)  # stand in for the physics step callback
            try:
                message_id, payload, addr = client.recv_message()
            except socket.timeout:
                continue
            if message_id == int(st.MessageId.NAT_FRAMEOFDATA):
                assert addr[1] == data_port
                got = payload
                break
    finally:
        client.close()
        mgr.stop_server()

    assert got is not None, "expected a NAT_FRAMEOFDATA frame"
    rb_count, body_id, (x, y, z) = _decode_first_rigid_body(got)
    assert rb_count == 1
    assert body_id == 1
    # abs tolerance absorbs float32 + USD xform-decompose noise on the sampled pose
    # (e.g. 7.0 sampled back as 7.001); mm precision is more than enough here.
    assert (x, y, z) == pytest.approx((7.0, -3.0, 2.0), abs=1e-2)
