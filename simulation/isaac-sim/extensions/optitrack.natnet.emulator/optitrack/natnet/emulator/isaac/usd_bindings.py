# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""USD binding layer: author / read / find NatNet interface prims on a stage.

``pxr`` is imported **lazily inside each function** so importing this module never
requires USD — that keeps ``optitrack.natnet.emulator.isaac`` importable in plain
(non-Isaac) test environments. The pure config logic lives in ``config.py``.

Backing today is **plain namespaced custom attributes + relationships** (the
registration-free fallback from the design's Risk #1). Property names follow the
multi-apply schema convention (``natnet:body:<key>:<field>``), so swapping to a
typed codeless applied schema later requires no change to readers/writers.
"""

from __future__ import annotations

from typing import Any

from .config import (
    ATTR_COMMAND_PORT,
    ATTR_DATA_PORT,
    ATTR_MODE,
    ATTR_MULTICAST_ADDR,
    ATTR_NATNET_VERSION,
    ATTR_PUBLISH_RATE,
    ATTR_SERVER_ENABLED,
    ATTR_SERVER_IP,
    BODY_FIELD_PARENT_ID,
    BODY_FIELD_RIGID_BODY_NAME,
    BODY_FIELD_STREAMING_ID,
    BODY_FIELD_TARGET,
    BODY_PREFIX,
    DEFAULT_COMMAND_PORT,
    DEFAULT_DATA_PORT,
    DEFAULT_MULTICAST_ADDR,
    DEFAULT_NATNET_VERSION,
    DEFAULT_PUBLISH_RATE,
    DEFAULT_SERVER_IP,
    MARKER_ATTR,
    BodyBinding,
    NatNetInterfaceConfig,
    body_attr_name,
)


def author_interface(stage, prim_path: str, config: Any) -> Any:
    """Create/overwrite a NatNet interface prim at ``prim_path`` from ``config``.

    ``config`` may be a :class:`NatNetInterfaceConfig` or a plain ``dict`` (passed
    through ``from_dict``). Returns the ``Usd.Prim``.
    """
    from pxr import Sdf

    cfg = config if isinstance(config, NatNetInterfaceConfig) else NatNetInterfaceConfig.from_dict(config)
    cfg.validate()

    prim = stage.DefinePrim(prim_path, "Scope")

    # Overwrite semantics: drop any previously-authored body properties so removed
    # bodies don't linger across re-authoring.
    _clear_body_properties(prim)

    _set(prim, MARKER_ATTR, Sdf.ValueTypeNames.Bool, True)
    _set(prim, ATTR_SERVER_ENABLED, Sdf.ValueTypeNames.Bool, cfg.server_enabled)
    _set(prim, ATTR_SERVER_IP, Sdf.ValueTypeNames.String, cfg.server_ip)
    _set(prim, ATTR_MODE, Sdf.ValueTypeNames.Token, cfg.mode)
    _set(prim, ATTR_MULTICAST_ADDR, Sdf.ValueTypeNames.String, cfg.multicast_addr)
    _set(prim, ATTR_COMMAND_PORT, Sdf.ValueTypeNames.Int, cfg.command_port)
    _set(prim, ATTR_DATA_PORT, Sdf.ValueTypeNames.Int, cfg.data_port)
    _set(prim, ATTR_PUBLISH_RATE, Sdf.ValueTypeNames.Float, cfg.publish_rate)
    _set(prim, ATTR_NATNET_VERSION, Sdf.ValueTypeNames.String, cfg.natnet_version)

    for key, body in cfg.assign_instance_keys():
        _set(prim, body_attr_name(key, BODY_FIELD_RIGID_BODY_NAME), Sdf.ValueTypeNames.String, body.rigid_body_name)
        _set(prim, body_attr_name(key, BODY_FIELD_STREAMING_ID), Sdf.ValueTypeNames.Int, body.streaming_id)
        _set(prim, body_attr_name(key, BODY_FIELD_PARENT_ID), Sdf.ValueTypeNames.Int, body.parent_id)
        rel = prim.CreateRelationship(body_attr_name(key, BODY_FIELD_TARGET), False)
        # Empty target is allowed (e.g. a freshly added body to be pointed in the
        # Property panel); leave the relationship target-less rather than authoring
        # an invalid empty Sdf.Path.
        rel.SetTargets([Sdf.Path(body.target_prim)] if body.target_prim else [])

    return prim


def read_interface(prim) -> NatNetInterfaceConfig:
    """Reconstruct a :class:`NatNetInterfaceConfig` from an authored interface prim."""
    return NatNetInterfaceConfig(
        server_enabled=bool(_get(prim, ATTR_SERVER_ENABLED, True)),
        server_ip=str(_get(prim, ATTR_SERVER_IP, DEFAULT_SERVER_IP)),
        mode=str(_get(prim, ATTR_MODE, "unicast")),
        multicast_addr=str(_get(prim, ATTR_MULTICAST_ADDR, DEFAULT_MULTICAST_ADDR)),
        command_port=int(_get(prim, ATTR_COMMAND_PORT, DEFAULT_COMMAND_PORT)),
        data_port=int(_get(prim, ATTR_DATA_PORT, DEFAULT_DATA_PORT)),
        publish_rate=float(_get(prim, ATTR_PUBLISH_RATE, DEFAULT_PUBLISH_RATE)),
        natnet_version=str(_get(prim, ATTR_NATNET_VERSION, DEFAULT_NATNET_VERSION)),
        bodies=_read_bodies(prim),
    )


def find_interfaces(stage) -> list:
    """Return every prim on the stage marked as a NatNet interface."""
    interfaces = []
    for prim in stage.Traverse():
        attr = prim.GetAttribute(MARKER_ATTR)
        if attr and attr.HasAuthoredValue() and bool(attr.Get()):
            interfaces.append(prim)
    return interfaces


def is_interface(prim) -> bool:
    attr = prim.GetAttribute(MARKER_ATTR)
    return bool(attr and attr.HasAuthoredValue() and bool(attr.Get()))


def read_world_pose(prim):
    """Return ``((x, y, z), (qx, qy, qz, qw))`` from a prim's USD world transform.

    Reads the position/orientation **stored in the USD stage** (the local-to-world
    transform), which is what the physics step writes back each frame. Returns
    ``None`` for an invalid/non-xformable prim so callers can mark the body lost.
    """
    from pxr import Usd, UsdGeom

    if prim is None or not prim.IsValid():
        return None
    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return None
    matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    translation = matrix.ExtractTranslation()
    quat = matrix.ExtractRotationQuat()  # Gf.Quatd, normalized
    imaginary = quat.GetImaginary()
    position = (float(translation[0]), float(translation[1]), float(translation[2]))
    orientation = (
        float(imaginary[0]),
        float(imaginary[1]),
        float(imaginary[2]),
        float(quat.GetReal()),
    )
    return position, orientation


def resolve_targets(stage, config):
    """Split a config's bodies into (existing, missing) by target prim presence.

    A body whose ``target_prim`` is empty or points at a non-existent prim lands in
    ``missing``. Returns two lists of :class:`BodyBinding`.
    """
    existing = []
    missing = []
    for body in config.bodies:
        prim = stage.GetPrimAtPath(body.target_prim) if body.target_prim else None
        if prim is not None and prim.IsValid():
            existing.append(body)
        else:
            missing.append(body)
    return existing, missing


# --- internal helpers ----------------------------------------------------------


def _set(prim, name, type_name, value):
    attr = prim.CreateAttribute(name, type_name)
    attr.Set(value)
    return attr


def _get(prim, name, default):
    attr = prim.GetAttribute(name)
    if attr and attr.HasAuthoredValue():
        return attr.Get()
    return default


def _clear_body_properties(prim) -> None:
    for name in list(prim.GetPropertyNames()):
        if name.startswith(BODY_PREFIX):
            prim.RemoveProperty(name)


def _read_bodies(prim) -> list[BodyBinding]:
    suffix = f":{BODY_FIELD_RIGID_BODY_NAME}"
    keys = [
        name[len(BODY_PREFIX): -len(suffix)]
        for name in prim.GetPropertyNames()
        if name.startswith(BODY_PREFIX) and name.endswith(suffix)
    ]

    bodies: list[BodyBinding] = []
    for key in keys:
        rel = prim.GetRelationship(body_attr_name(key, BODY_FIELD_TARGET))
        targets = rel.GetTargets() if rel else []
        bodies.append(
            BodyBinding(
                rigid_body_name=str(_get(prim, body_attr_name(key, BODY_FIELD_RIGID_BODY_NAME), "")),
                target_prim=str(targets[0]) if targets else "",
                streaming_id=int(_get(prim, body_attr_name(key, BODY_FIELD_STREAMING_ID), 1)),
                parent_id=int(_get(prim, body_attr_name(key, BODY_FIELD_PARENT_ID), -1)),
            )
        )

    # Stable, deterministic order (independent of USD property iteration order).
    bodies.sort(key=lambda b: (b.streaming_id, b.rigid_body_name))
    return bodies
