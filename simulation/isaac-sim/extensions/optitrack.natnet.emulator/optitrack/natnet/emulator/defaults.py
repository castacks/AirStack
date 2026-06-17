"""Reference tracked-body defaults for tests and the future Isaac Sim wrapper."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TrackedBodyBinding:
    """Maps a NatNet rigid body to a USD prim path (not sent on the NatNet wire)."""

    name: str
    id: int
    prim_path: str
    parent_id: int = -1


# Single-drone NatNet Pegasus scenes (example_one_px4_pegasus_natnet_launch_script.py).
DEFAULT_DRONE_BINDING = TrackedBodyBinding(
    name="Drone",
    id=1,
    prim_path="/World/base_link",
)

DEFAULT_TRACKED_BODY_BINDINGS: tuple[TrackedBodyBinding, ...] = (DEFAULT_DRONE_BINDING,)
