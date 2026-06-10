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
from .manager import NatNetServerManager, format_interface
from .usd_bindings import (
    author_interface,
    find_interfaces,
    is_interface,
    read_interface,
)

__all__ = [
    "BodyBinding",
    "NatNetInterfaceConfig",
    "NatNetServerManager",
    "author_interface",
    "body_attr_name",
    "find_interfaces",
    "format_interface",
    "is_interface",
    "make_instance_key",
    "read_interface",
]
