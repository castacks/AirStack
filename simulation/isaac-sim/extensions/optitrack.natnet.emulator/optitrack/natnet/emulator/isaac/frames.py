# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Pose -> NatNet frame conversion (the data-enqueue path).

Pure Python + ctypes (no USD, no Kit), so it's hermetically unit-testable. Sampled
prim world poses become an ``sFrameOfMocapData`` of rigid bodies that the server
streams to ``natnet_ros2``.

**Frame convention:** AirStack's ``natnet_ros2`` copies the rigid-body pose straight
through (``rb_to_pose`` is an identity copy) and nothing downstream re-axes it, so we
emit the prim's USD world pose **as-is** (no Z-up->Y-up swap). Swapping here would
desync the published pose from the rest of the stack.

**params bits** (must match the client's ``is_tracking_valid`` / ``model_list_changed``):
``0x01`` on a rigid body marks tracking valid — the client *skips* bodies without it.
``0x02`` on the frame signals the model list changed so the client re-requests MODELDEF
(set the frame after the catalog changes, e.g. a body added live).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..server.natnet_data_types import sFrameOfMocapData, sRigidBodyData

TRACKING_VALID = 0x01
MODEL_LIST_CHANGED = 0x02


@dataclass
class BodySample:
    """One sampled rigid body: streaming ID + world pose, or an invalid (lost) body."""

    streaming_id: int
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # qx,qy,qz,qw
    valid: bool = True

    @classmethod
    def lost(cls, streaming_id: int) -> "BodySample":
        """An untracked body (missing prim): NaN position, tracking-invalid bit clear."""
        nan = float("nan")
        return cls(streaming_id, (nan, nan, nan), (0.0, 0.0, 0.0, 1.0), valid=False)


def make_rigid_body_data(sample: BodySample) -> sRigidBodyData:
    """Build one ``sRigidBodyData`` from a sample (sets the tracking-valid bit)."""
    rb = sRigidBodyData()
    rb.ID = int(sample.streaming_id)
    x, y, z = sample.position
    qx, qy, qz, qw = sample.orientation
    rb.x, rb.y, rb.z = float(x), float(y), float(z)
    rb.qx, rb.qy, rb.qz, rb.qw = float(qx), float(qy), float(qz), float(qw)
    rb.MeanError = 0.0
    rb.params = TRACKING_VALID if sample.valid else 0
    return rb


def build_frame(
    frame_number: int,
    samples,
    *,
    timestamp: float = 0.0,
    model_list_changed: bool = False,
) -> sFrameOfMocapData:
    """Assemble an ``sFrameOfMocapData`` of rigid bodies from samples."""
    frame = sFrameOfMocapData()
    frame.iFrame = int(frame_number)
    samples = list(samples)
    frame.nRigidBodies = len(samples)
    for i, sample in enumerate(samples):
        frame.RigidBodies[i] = make_rigid_body_data(sample)
    frame.fTimestamp = float(timestamp)
    frame.params = MODEL_LIST_CHANGED if model_list_changed else 0
    return frame


def is_finite_pose(sample: BodySample) -> bool:
    """True if all position/orientation components are finite (no NaN/inf)."""
    return all(math.isfinite(v) for v in (*sample.position, *sample.orientation))
