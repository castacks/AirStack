# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Standalone-launch helpers: stand up a drone NatNet interface on scene load.

Used by the Pegasus example launch scripts so a Motive-compatible NatNet server
comes up automatically with one rigid body per drone ``base_link`` — no UI clicks.

Two layers, mirroring the rest of the package:

- ``build_drone_config`` is **pure** (no USD / Kit), so it unit-tests hermetically.
- ``author_drone_natnet_interface`` writes the interface prim the extension builds
  its server from. It imports ``pxr`` lazily (only when called).
"""

from __future__ import annotations

from typing import Iterable, Sequence, Tuple

from .config import (
    DEFAULT_COMMAND_PORT,
    DEFAULT_DATA_PORT,
    DEFAULT_POSE_NOISE_ENABLED,
    DEFAULT_POSE_NOISE_ROTATION_DEG,
    DEFAULT_POSE_NOISE_STD_METERS,
    DEFAULT_PUBLISH_RATE,
    DEFAULT_SERVER_IP,
    DEFAULT_UP_AXIS,
    BodyBinding,
    NatNetInterfaceConfig,
)

# Where the example scripts author the single interface prim.
DEFAULT_INTERFACE_PATH = "/World/NatNetInterface"

# Default world prim + position for the demo "target" body (a static placeholder
# the example scripts stream alongside the drones so a tracked target is available).
DEFAULT_TARGET_PATH = "/World/target"
DEFAULT_TARGET_POSITION = (2.0, 0.0, 1.0)
DEFAULT_TARGET_STREAMING_ID = 100

# (rigid_body_name, streaming_id, target_prim_path)
DroneSpec = Tuple[str, int, str]


def author_static_target(
    stage,
    prim_path: str = DEFAULT_TARGET_PATH,
    position: Sequence[float] = DEFAULT_TARGET_POSITION,
):
    """Author a static ``Xform`` prim to act as a NatNet-tracked target.

    Creates ``prim_path`` (a plain transform with a single translate op) at
    ``position`` so the emulator can sample it like any other tracked body. The
    prim is static — no physics, no animation — representing a fixed point of
    interest that drones can be commanded toward. Imports ``pxr`` lazily so this
    module stays importable outside Isaac. Returns ``prim_path``.
    """
    from pxr import Gf, UsdGeom

    xform = UsdGeom.Xform.Define(stage, prim_path)
    xform.AddTranslateOp().Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
    return prim_path


def build_drone_config(
    drones: Iterable[DroneSpec],
    *,
    server_ip: str = DEFAULT_SERVER_IP,
    mode: str = "unicast",
    command_port: int = DEFAULT_COMMAND_PORT,
    data_port: int = DEFAULT_DATA_PORT,
    publish_rate: float = DEFAULT_PUBLISH_RATE,
    server_enabled: bool = True,
    up_axis: str = DEFAULT_UP_AXIS,
    pose_noise_enabled: bool = DEFAULT_POSE_NOISE_ENABLED,
    pose_noise_std_meters: float = DEFAULT_POSE_NOISE_STD_METERS,
    pose_noise_rotation_deg: float = DEFAULT_POSE_NOISE_ROTATION_DEG,
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
        up_axis=up_axis,
        pose_noise_enabled=pose_noise_enabled,
        pose_noise_std_meters=pose_noise_std_meters,
        pose_noise_rotation_deg=pose_noise_rotation_deg,
        bodies=bodies,
    )
    cfg.validate()
    return cfg


def author_drone_natnet_interface(
    stage,
    drones: Sequence[DroneSpec],
    *,
    prim_path: str = DEFAULT_INTERFACE_PATH,
    **config_kwargs,
) -> NatNetInterfaceConfig:
    """Author the NatNet interface prim from ``drones``.

    Writes ``prim_path`` (overwriting any existing interface) with one rigid body per
    drone. Call this before starting the timeline: the extension builds the server
    from this prim on Play. Returns the authored config.
    """
    from .usd_bindings import author_interface

    cfg = build_drone_config(drones, **config_kwargs)
    author_interface(stage, prim_path, cfg)
    return cfg
