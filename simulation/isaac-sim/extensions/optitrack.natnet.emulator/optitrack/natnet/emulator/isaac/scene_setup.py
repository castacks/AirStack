# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Standalone-launch helpers: stand up a drone NatNet interface on scene load.

Used by the Pegasus example launch scripts so a Motive-compatible NatNet server
comes up automatically with one rigid body per drone ``base_link`` — no UI clicks.

Two layers, mirroring the rest of the package:

- ``build_drone_config`` is **pure** (no USD / Kit), so it unit-tests hermetically.
- ``start_drone_natnet_server`` authors the interface prim and owns a
  :class:`~optitrack.natnet.emulator.isaac.manager.NatNetServerManager` that samples
  poses on each physics step. It imports ``pxr``/``omni`` lazily (only when called).
"""

from __future__ import annotations

from typing import Iterable, Sequence, Tuple

from .config import (
    DEFAULT_COMMAND_PORT,
    DEFAULT_DATA_PORT,
    DEFAULT_PUBLISH_RATE,
    DEFAULT_SERVER_IP,
    BodyBinding,
    NatNetInterfaceConfig,
)

# Where the example scripts author the single interface prim.
DEFAULT_INTERFACE_PATH = "/World/NatNetInterface"

# (rigid_body_name, streaming_id, target_prim_path)
DroneSpec = Tuple[str, int, str]


def build_drone_config(
    drones: Iterable[DroneSpec],
    *,
    server_ip: str = DEFAULT_SERVER_IP,
    mode: str = "unicast",
    command_port: int = DEFAULT_COMMAND_PORT,
    data_port: int = DEFAULT_DATA_PORT,
    publish_rate: float = DEFAULT_PUBLISH_RATE,
    server_enabled: bool = True,
) -> NatNetInterfaceConfig:
    """Build a validated config with one rigid body per drone.

    ``drones`` is an iterable of ``(rigid_body_name, streaming_id, target_prim)``
    tuples — typically one per spawned drone, with ``target_prim`` pointing at the
    drone's ``base_link``. Raises ``ValueError`` (via ``validate``) on duplicate
    names/ids or bad ports.
    """
    bodies = [
        BodyBinding(
            rigid_body_name=str(name),
            target_prim=str(target),
            streaming_id=int(streaming_id),
        )
        for name, streaming_id, target in drones
    ]
    cfg = NatNetInterfaceConfig(
        server_enabled=server_enabled,
        server_ip=server_ip,
        mode=mode,
        command_port=command_port,
        data_port=data_port,
        publish_rate=publish_rate,
        bodies=bodies,
    )
    cfg.validate()
    return cfg


def start_drone_natnet_server(
    stage,
    drones: Sequence[DroneSpec],
    *,
    prim_path: str = DEFAULT_INTERFACE_PATH,
    start: bool = True,
    **config_kwargs,
):
    """Author a NatNet interface prim from ``drones`` and return a running manager.

    Authors ``prim_path`` (overwriting any existing interface) with one rigid body
    per drone, then creates a :class:`NatNetServerManager` that subscribes to physics
    steps and starts the server. Pump the sim (``timeline.play()`` / ``world.step``)
    to stream poses. **Keep a reference to the returned manager** so it isn't
    garbage-collected (which would tear down the physics subscription and server).

    Returns the ``NatNetServerManager``. If ``start`` is False (or the config is
    authored disabled), the manager is created but the server is left stopped.
    """
    from .manager import NatNetServerManager
    from .usd_bindings import author_interface

    cfg = build_drone_config(drones, **config_kwargs)
    author_interface(stage, prim_path, cfg)

    manager = NatNetServerManager()
    manager.on_startup()
    if start and cfg.server_enabled:
        manager.start_from_stage()
    return manager
