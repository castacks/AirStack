# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Server lifecycle tests with a mocked server factory."""
from __future__ import annotations

import pytest

from optitrack.natnet.emulator.isaac.config import BodyBinding, NatNetInterfaceConfig
from optitrack.natnet.emulator.isaac.manager import (
    NatNetServerManager,
    default_server_factory,
)

pytestmark = pytest.mark.unit


class FakeServer:
    def __init__(self):
        self.start_calls = 0
        self.shutdown_calls = 0
        self.model_def_payload = None

    def set_model_def_payload(self, payload):
        self.model_def_payload = payload

    def start(self):
        self.start_calls += 1

    def shutdown(self):
        self.shutdown_calls += 1


class RecordingFactory:
    """Factory that records every server it builds (one per enable)."""

    def __init__(self):
        self.created = []

    def __call__(self, config):
        server = FakeServer()
        self.created.append(server)
        return server


def _cfg(enabled=True, bodies=None):
    return NatNetInterfaceConfig(
        server_ip="127.0.0.1",
        server_enabled=enabled,
        bodies=bodies if bodies is not None else [BodyBinding("Drone", "/World/base_link", 1)],
    )


def test_start_creates_and_starts_server_once():
    factory = RecordingFactory()
    mgr = NatNetServerManager(server_factory=factory)

    assert mgr.start_server(_cfg()) is True
    assert mgr.is_running is True
    assert len(factory.created) == 1
    server = factory.created[0]
    assert server.start_calls == 1
    assert server.model_def_payload is not None  # catalog was set before start


def test_start_is_idempotent_only_one_server():
    factory = RecordingFactory()
    mgr = NatNetServerManager(server_factory=factory)

    assert mgr.start_server(_cfg()) is True
    # Repeated starts must not build or start a second server.
    assert mgr.start_server(_cfg()) is False
    assert mgr.start_server(_cfg()) is False

    assert len(factory.created) == 1
    assert factory.created[0].start_calls == 1


def test_stop_shuts_down_and_is_idempotent():
    factory = RecordingFactory()
    mgr = NatNetServerManager(server_factory=factory)

    mgr.start_server(_cfg())
    assert mgr.stop_server() is True
    assert mgr.is_running is False
    assert factory.created[0].shutdown_calls == 1
    # Stopping again is a harmless no-op.
    assert mgr.stop_server() is False
    assert factory.created[0].shutdown_calls == 1


def test_toggle_starts_then_stops():
    factory = RecordingFactory()
    mgr = NatNetServerManager(server_factory=factory)

    assert mgr.toggle_server(_cfg()) is True
    assert mgr.is_running is True
    assert mgr.toggle_server(_cfg()) is False
    assert mgr.is_running is False
    # A second enable builds a *fresh* server instance.
    assert mgr.toggle_server(_cfg()) is True
    assert len(factory.created) == 2


def test_apply_enabled_reconciles_to_flag():
    factory = RecordingFactory()
    mgr = NatNetServerManager(server_factory=factory)

    mgr.apply_enabled(_cfg(enabled=False))
    assert mgr.is_running is False
    assert len(factory.created) == 0

    mgr.apply_enabled(_cfg(enabled=True))
    assert mgr.is_running is True
    assert len(factory.created) == 1

    # Re-applying the same enabled state does not build another server.
    mgr.apply_enabled(_cfg(enabled=True))
    assert len(factory.created) == 1

    mgr.apply_enabled(_cfg(enabled=False))
    assert mgr.is_running is False
    assert factory.created[0].shutdown_calls == 1


def test_on_shutdown_stops_running_server():
    factory = RecordingFactory()
    mgr = NatNetServerManager(server_factory=factory)
    mgr.start_server(_cfg())
    mgr.on_shutdown()
    assert mgr.is_running is False
    assert factory.created[0].shutdown_calls == 1


@pytest.mark.parametrize("n_bodies", [0, 1, 3])
def test_start_handles_no_single_and_multiple_bodies(n_bodies):
    factory = RecordingFactory()
    mgr = NatNetServerManager(server_factory=factory)
    bodies = [BodyBinding(f"B{i}", f"/World/b{i}", i + 1) for i in range(n_bodies)]
    assert mgr.start_server(_cfg(bodies=bodies)) is True
    assert mgr.is_running is True


def test_default_factory_rejects_multicast_without_constructing():
    cfg = NatNetInterfaceConfig(mode="multicast", multicast_addr="239.255.42.99")
    with pytest.raises(NotImplementedError):
        default_server_factory(cfg)


def test_default_factory_builds_unicast_server_without_starting():
    cfg = NatNetInterfaceConfig(server_ip="127.0.0.1", command_port=1610, data_port=1611)
    server = default_server_factory(cfg)
    # Constructed but not started: no threads, not running.
    assert server.running is False
    assert server.command_port == 1610
    assert server.data_port == 1611
