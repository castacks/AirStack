# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Isaac Sim integration for the NatNet emulator (stage-driven config prim).

``config`` is pure Python. ``usd_bindings`` imports ``pxr`` lazily, so
importing this package is safe in non-Isaac environments.
"""

from .config import (
    BodyBinding,
    NatNetInterfaceConfig,
    body_attr_name,
    make_instance_key,
)
from .catalog import build_catalog, find_duplicate_targets
from .frames import BodySample, build_frame, make_rigid_body_data
from .manager import NatNetServerManager, default_server_factory, format_interface
from .scene_setup import (
    DEFAULT_INTERFACE_PATH,
    DEFAULT_TARGET_PATH,
    DEFAULT_TARGET_POSITION,
    DEFAULT_TARGET_STREAMING_ID,
    author_drone_natnet_interface,
    author_static_target,
    build_drone_config,
)
from .usd_bindings import (
    author_interface,
    find_interfaces,
    is_interface,
    read_interface,
    read_world_pose,
    resolve_targets,
)

__all__ = [
    "DEFAULT_INTERFACE_PATH",
    "DEFAULT_TARGET_PATH",
    "DEFAULT_TARGET_POSITION",
    "DEFAULT_TARGET_STREAMING_ID",
    "BodyBinding",
    "BodySample",
    "NatNetInterfaceConfig",
    "NatNetServerManager",
    "author_interface",
    "author_drone_natnet_interface",
    "author_static_target",
    "body_attr_name",
    "build_catalog",
    "build_drone_config",
    "build_frame",
    "default_server_factory",
    "find_duplicate_targets",
    "find_interfaces",
    "format_interface",
    "is_interface",
    "make_instance_key",
    "make_rigid_body_data",
    "read_interface",
    "read_world_pose",
    "resolve_targets",
]
