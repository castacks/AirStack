# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""
Turn a :class:`NatNetInterfaceConfig` into the server's MODELDEF catalog
(``sDataDescriptions`` of rigid bodies). Pure Python + ctypes (the ``server``
package is stdlib-only), so this is hermetically unit-testable — no USD, no Kit.
"""

from __future__ import annotations

from ..server.natnet_common import ModelLimits
from ..server.natnet_model_types import DataDescriptors, sDataDescriptions
from .config import NatNetInterfaceConfig

# szName is null-terminated on the wire; reserve one byte for the terminator.
_MAX_NAME_BYTES = int(ModelLimits.MAX_NAMELENGTH) - 1
_MAX_MODELS = int(ModelLimits.MAX_MODELS)


def build_catalog(config: NatNetInterfaceConfig) -> sDataDescriptions:
    """Build an ``sDataDescriptions`` rigid-body catalog from the config bodies.

    No bodies -> an empty catalog (``nDataDescriptions == 0``). Names longer than
    the NatNet name field are truncated. Raises ``ValueError`` if there are more
    bodies than the protocol allows.
    """
    bodies = config.bodies
    if len(bodies) > _MAX_MODELS:
        raise ValueError(
            f"Too many bodies for one catalog: {len(bodies)} > {_MAX_MODELS} (MAX_MODELS)"
        )

    descriptions = sDataDescriptions()
    descriptions.nDataDescriptions = len(bodies)
    for i, body in enumerate(bodies):
        desc = descriptions.arrDataDescriptions[i]
        desc.type = int(DataDescriptors.Descriptor_RigidBody)
        rb = desc.RigidBodyDescription
        rb.szName = body.rigid_body_name.encode("utf-8")[:_MAX_NAME_BYTES]
        rb.ID = int(body.streaming_id)
        rb.parentID = int(body.parent_id)
        rb.offsetqw = 1.0  # identity quaternion offset
        rb.nMarkers = 0
    return descriptions


def find_duplicate_targets(config: NatNetInterfaceConfig) -> list[str]:
    """Return target prim paths referenced by more than one body (empties ignored)."""
    counts: dict[str, int] = {}
    for body in config.bodies:
        if body.target_prim:
            counts[body.target_prim] = counts.get(body.target_prim, 0) + 1
    return [path for path, count in counts.items() if count > 1]
