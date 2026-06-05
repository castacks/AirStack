"""Shared helpers for optitrack.natnet.emulator unit tests."""

from __future__ import annotations

import socket
import struct
import sys
import time
from contextlib import contextmanager
from pathlib import Path

_EXT_ROOT = Path(__file__).resolve().parents[1]
if str(_EXT_ROOT) not in sys.path:
    sys.path.insert(0, str(_EXT_ROOT))

from optitrack.natnet.emulator import NatNetUnicastServer, TransmissionType
from optitrack.natnet.emulator.server import natnet_server_types as st


def ephemeral_udp_port(host: str = "127.0.0.1") -> int:
    """Return a free UDP port on *host* by binding and releasing a probe socket."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind((host, 0))
        return probe.getsockname()[1]


class NatNetTestClient:
    """Minimal UDP client for NatNet command-port protocol tests."""

    def __init__(self, host: str = "127.0.0.1", timeout: float = 2.0) -> None:
        self._host = host
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, 0))
        self._sock.settimeout(timeout)

    @property
    def local_port(self) -> int:
        return self._sock.getsockname()[1]

    def send_message(
        self,
        server_port: int,
        message_id: st.MessageId | int,
        payload: bytes = b"",
        server_host: str | None = None,
    ) -> None:
        header = st.sPacketHeader(
            iMessage=int(message_id),
            nDataBytes=len(payload),
        )
        self.send_raw(header.pack() + payload, server_port, server_host)

    def send_raw(
        self,
        data: bytes,
        server_port: int,
        server_host: str | None = None,
    ) -> None:
        """Send a raw UDP datagram (for malformed / malicious packet tests)."""
        self._sock.sendto(data, (server_host or self._host, server_port))

    def send_header_only(
        self,
        server_port: int,
        message_id: st.MessageId | int,
        declared_payload_len: int,
        server_host: str | None = None,
    ) -> None:
        """Send a header whose nDataBytes does not match any trailing payload."""
        header = struct.pack("<HH", int(message_id), declared_payload_len)
        self.send_raw(header, server_port, server_host)

    def recv_message(self) -> tuple[int, bytes, tuple[str, int]]:
        data, addr = self._sock.recvfrom(65535)
        message_id, payload_len = struct.unpack("<HH", data[:4])
        payload = data[4 : 4 + payload_len]
        return message_id, payload, addr

    def close(self) -> None:
        self._sock.close()


@contextmanager
def running_unicast_server(
    command_port: int | None = None,
    local_interface: str = "127.0.0.1",
    publish_rate: int = 100,
):
    """Start NatNetUnicastServer on an ephemeral or fixed command port."""
    port = command_port if command_port is not None else ephemeral_udp_port(local_interface)
    server = NatNetUnicastServer(
        local_interface=local_interface,
        transmission_type=TransmissionType.UNICAST,
        multicast_address=None,
        command_port=port,
    )
    server.publish_rate = publish_rate
    server.start()
    time.sleep(0.05)
    try:
        yield server, port
    finally:
        server.shutdown()
