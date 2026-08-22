# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Pure-Python config model for the stage-driven NatNet interface.

The USD binding layer (author/read against a ``Usd.Stage``) 
lives in ``usd_bindings.py`` and depends on this model.

Attribute names follow the multi-apply schema convention
(``natnet:body:<key>:<field>``).
The custom-attribute backing is for a future typed applied schema.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping

# --- attribute name constants (USD property names) -----------------------------

ATTR_NAMESPACE = "natnet"
MARKER_ATTR = "natnet:isInterface"

ATTR_SERVER_ENABLED = "natnet:serverEnabled"
ATTR_SERVER_IP = "natnet:serverIp"
ATTR_MODE = "natnet:mode"
ATTR_MULTICAST_ADDR = "natnet:multicastAddr"
ATTR_COMMAND_PORT = "natnet:commandPort"
ATTR_DATA_PORT = "natnet:dataPort"
ATTR_PUBLISH_RATE = "natnet:publishRate"
ATTR_NATNET_VERSION = "natnet:natnetVersion"
ATTR_UP_AXIS = "natnet:upAxis"

ATTR_POSE_NOISE_ENABLED = "natnet:poseNoiseEnabled"
ATTR_POSE_NOISE_STD_METERS = "natnet:poseNoiseStdMeters"
ATTR_POSE_NOISE_ROTATION_DEG = "natnet:poseNoiseRotationDeg"

BODY_PREFIX = "natnet:body:"
BODY_FIELD_RIGID_BODY_NAME = "rigidBodyName"
BODY_FIELD_STREAMING_ID = "streamingId"
BODY_FIELD_PARENT_ID = "parentId"
BODY_FIELD_TARGET = "target"

VALID_MODES = ("unicast", "multicast")

# Streamed up-axis. "Z" (default) passes the USD pose through; "Y" emulates a Y-up
# Motive by rotating the streamed pose -90deg about X.
VALID_UP_AXES = ("Y", "Z")

# defaults shared with NatNetUnicastServer
DEFAULT_SERVER_IP = "172.31.0.200"
DEFAULT_MULTICAST_ADDR = "239.255.42.99"
DEFAULT_COMMAND_PORT = 1510
DEFAULT_DATA_PORT = 1511
DEFAULT_PUBLISH_RATE = 100.0
DEFAULT_NATNET_VERSION = "4.4.0.0"
DEFAULT_UP_AXIS = "Z"
DEFAULT_POSE_NOISE_ENABLED = True
DEFAULT_POSE_NOISE_STD_METERS = 0.0005
DEFAULT_POSE_NOISE_ROTATION_DEG = 0.05


def body_attr_name(key: str, field_name: str) -> str:
    """USD property name for a body-binding field on the given instance key."""
    return f"{BODY_PREFIX}{key}:{field_name}"


def make_instance_key(name: str, used: set[str]) -> str:
    """Derive a valid, unique multi-apply instance token from a rigid body name.

    USD property/instance tokens must be identifier-like; sanitize non-alnum chars
    to underscores and disambiguate collisions with a numeric suffix.
    """
    sanitized = "".join(c if c.isalnum() else "_" for c in name).strip("_")
    if not sanitized:
        sanitized = "body"
    if sanitized[0].isdigit():
        sanitized = f"b_{sanitized}"
    key = sanitized
    i = 1
    while key in used:
        key = f"{sanitized}_{i}"
        i += 1
    used.add(key)
    return key


@dataclass
class BodyBinding:
    """One tracked rigid body: a Motive name/ID mapped to a USD prim path."""

    rigid_body_name: str
    target_prim: str
    streaming_id: int = 1
    parent_id: int = -1

    @classmethod
    def from_dict(cls, data: Mapping[str, Any], *, target_prim: str | None = None) -> "BodyBinding":
        d = dict(data)
        resolved_target = target_prim if target_prim is not None else d.get("target_prim")
        if not resolved_target:
            raise ValueError("BodyBinding requires a target_prim (USD path of the tracked prim)")
        if "rigid_body_name" not in d:
            raise ValueError("BodyBinding requires a rigid_body_name")
        return cls(
            rigid_body_name=str(d["rigid_body_name"]),
            target_prim=str(resolved_target),
            streaming_id=int(d.get("streaming_id", 1)),
            parent_id=int(d.get("parent_id", -1)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "rigid_body_name": self.rigid_body_name,
            "target_prim": self.target_prim,
            "streaming_id": self.streaming_id,
            "parent_id": self.parent_id,
        }


def _normalize_bodies(bodies: Any) -> list[BodyBinding]:
    """Accept a list of dicts/BodyBindings, or a ``{prim_path: {...}}`` mapping."""
    if bodies is None:
        return []
    out: list[BodyBinding] = []
    if isinstance(bodies, Mapping):
        for prim_path, body in bodies.items():
            out.append(BodyBinding.from_dict(body, target_prim=prim_path))
        return out
    if isinstance(bodies, Iterable):
        for body in bodies:
            if isinstance(body, BodyBinding):
                out.append(body)
            else:
                out.append(BodyBinding.from_dict(body))
        return out
    raise ValueError(f"`bodies` must be a list or a mapping, got {type(bodies).__name__}")


@dataclass
class NatNetInterfaceConfig:
    """Server-level config plus the body catalog for one NatNet interface prim."""

    server_enabled: bool = True
    server_ip: str = DEFAULT_SERVER_IP
    mode: str = "unicast"
    multicast_addr: str = DEFAULT_MULTICAST_ADDR
    command_port: int = DEFAULT_COMMAND_PORT
    data_port: int = DEFAULT_DATA_PORT
    publish_rate: float = DEFAULT_PUBLISH_RATE
    natnet_version: str = DEFAULT_NATNET_VERSION
    up_axis: str = DEFAULT_UP_AXIS
    pose_noise_enabled: bool = DEFAULT_POSE_NOISE_ENABLED
    pose_noise_std_meters: float = DEFAULT_POSE_NOISE_STD_METERS
    pose_noise_rotation_deg: float = DEFAULT_POSE_NOISE_ROTATION_DEG
    bodies: list[BodyBinding] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "NatNetInterfaceConfig":
        d = dict(data)
        return cls(
            server_enabled=bool(d.get("server_enabled", True)),
            server_ip=str(d.get("server_ip", DEFAULT_SERVER_IP)),
            mode=str(d.get("mode", "unicast")),
            multicast_addr=str(d.get("multicast_addr", DEFAULT_MULTICAST_ADDR)),
            command_port=int(d.get("command_port", DEFAULT_COMMAND_PORT)),
            data_port=int(d.get("data_port", DEFAULT_DATA_PORT)),
            publish_rate=float(d.get("publish_rate", DEFAULT_PUBLISH_RATE)),
            natnet_version=str(d.get("natnet_version", DEFAULT_NATNET_VERSION)),
            up_axis=str(d.get("up_axis", DEFAULT_UP_AXIS)).upper(),
            pose_noise_enabled=bool(d.get("pose_noise_enabled", DEFAULT_POSE_NOISE_ENABLED)),
            pose_noise_std_meters=float(d.get("pose_noise_std_meters", DEFAULT_POSE_NOISE_STD_METERS)),
            pose_noise_rotation_deg=float(d.get("pose_noise_rotation_deg", DEFAULT_POSE_NOISE_ROTATION_DEG)),
            bodies=_normalize_bodies(d.get("bodies")),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "server_enabled": self.server_enabled,
            "server_ip": self.server_ip,
            "mode": self.mode,
            "multicast_addr": self.multicast_addr,
            "command_port": self.command_port,
            "data_port": self.data_port,
            "publish_rate": self.publish_rate,
            "natnet_version": self.natnet_version,
            "up_axis": self.up_axis,
            "pose_noise_enabled": self.pose_noise_enabled,
            "pose_noise_std_meters": self.pose_noise_std_meters,
            "pose_noise_rotation_deg": self.pose_noise_rotation_deg,
            "bodies": [b.to_dict() for b in self.bodies],
        }

    def validate(self) -> "NatNetInterfaceConfig":
        """Raise ``ValueError`` (aggregating all problems) if the config is invalid."""
        errors: list[str] = []
        if self.mode not in VALID_MODES:
            errors.append(f"mode must be one of {VALID_MODES}, got {self.mode!r}")
        if str(self.up_axis).upper() not in VALID_UP_AXES:
            errors.append(f"up_axis must be one of {VALID_UP_AXES}, got {self.up_axis!r}")
        for port_name, port in (("command_port", self.command_port), ("data_port", self.data_port)):
            if not (0 < port < 65536):
                errors.append(f"{port_name} must be in 1..65535, got {port}")
        if self.command_port == self.data_port:
            errors.append("command_port and data_port must differ")
        if self.publish_rate <= 0:
            errors.append(f"publish_rate must be > 0, got {self.publish_rate}")
        if self.pose_noise_std_meters < 0:
            errors.append(
                f"pose_noise_std_meters must be >= 0, got {self.pose_noise_std_meters}"
            )
        if self.pose_noise_rotation_deg < 0:
            errors.append(
                f"pose_noise_rotation_deg must be >= 0, got {self.pose_noise_rotation_deg}"
            )
        for i, body in enumerate(self.bodies):
            if not body.rigid_body_name:
                errors.append(f"body[{i}] rigid_body_name must be non-empty")
        names = [b.rigid_body_name for b in self.bodies]
        if len(set(names)) != len(names):
            errors.append("rigid_body_name values must be unique across bodies")
        ids = [b.streaming_id for b in self.bodies]
        if len(set(ids)) != len(ids):
            errors.append("streaming_id values must be unique across bodies")
        if errors:
            raise ValueError("Invalid NatNetInterfaceConfig: " + "; ".join(errors))
        return self

    def assign_instance_keys(self) -> list[tuple[str, BodyBinding]]:
        """Pair each body with a deterministic, unique multi-apply instance key."""
        used: set[str] = set()
        return [(make_instance_key(b.rigid_body_name, used), b) for b in self.bodies]
