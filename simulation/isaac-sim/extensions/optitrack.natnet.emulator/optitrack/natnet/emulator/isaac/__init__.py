# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Isaac Sim integration for the NatNet emulator (stage-driven config prim).

``config`` is pure Python (no USD). ``usd_bindings`` imports ``pxr`` lazily, so
importing this package is safe in non-Isaac environments; the USD functions only
require ``pxr`` when actually called.
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
    build_drone_config,
    start_drone_natnet_server,
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
    "BodyBinding",
    "BodySample",
    "NatNetInterfaceConfig",
    "NatNetServerManager",
    "author_interface",
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
    "start_drone_natnet_server",
]
