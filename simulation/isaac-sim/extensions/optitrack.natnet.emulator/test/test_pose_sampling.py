# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Pose sampling + dynamic catalog (dirty/resync) tests against an in-memory stage.

Guarded by ``importorskip("pxr")``. Uses a fake server (records enqueued frames and
MODELDEF payloads) so no sockets are bound — we exercise the manager's sampling and
resync logic, not the wire.
"""

from __future__ import annotations

import math

import pytest

pytest.importorskip("pxr")

from pxr import Gf, Usd, UsdGeom  # noqa: E402

from optitrack.natnet.emulator.isaac.config import BodyBinding, NatNetInterfaceConfig  # noqa: E402
from optitrack.natnet.emulator.isaac.frames import MODEL_LIST_CHANGED, TRACKING_VALID  # noqa: E402
from optitrack.natnet.emulator.isaac.manager import NatNetServerManager  # noqa: E402
from optitrack.natnet.emulator.isaac.usd_bindings import author_interface, read_world_pose  # noqa: E402

pytestmark = pytest.mark.unit


class FakeServer:
    def __init__(self):
        self.frames = []
        self.payloads = []

    def set_model_def_payload(self, payload):
        self.payloads.append(payload)

    def start(self):
        pass

    def shutdown(self):
        pass

    def enqueue_mocap_data(self, frame):
        self.frames.append(frame)


def _xform(stage, path, translate=(0.0, 0.0, 0.0)):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    return xform


def _manager_with_fake():
    fake = FakeServer()
    mgr = NatNetServerManager(server_factory=lambda cfg: fake)
    return mgr, fake


# --- read_world_pose -------------------------------------------------------------


def test_read_world_pose_returns_translation():
    stage = Usd.Stage.CreateInMemory()
    _xform(stage, "/World/base_link", translate=(1.0, 2.0, 3.0))
    pose = read_world_pose(stage.GetPrimAtPath("/World/base_link"))
    assert pose is not None
    (x, y, z), (qx, qy, qz, qw) = pose
    assert (round(x, 3), round(y, 3), round(z, 3)) == (1.0, 2.0, 3.0)
    assert qw == pytest.approx(1.0)


def test_read_world_pose_invalid_prim_is_none():
    stage = Usd.Stage.CreateInMemory()
    assert read_world_pose(stage.GetPrimAtPath("/World/nope")) is None


# --- sample_once -----------------------------------------------------------------


def test_sample_once_no_bodies():
    stage = Usd.Stage.CreateInMemory()
    author_interface(stage, "/World/NatNetInterface", NatNetInterfaceConfig())
    mgr, fake = _manager_with_fake()
    mgr.start_server(NatNetInterfaceConfig(server_ip="127.0.0.1"))
    frame = mgr.sample_once(stage)
    assert frame is not None and frame.nRigidBodies == 0


def test_sample_once_streams_world_pose():
    stage = Usd.Stage.CreateInMemory()
    _xform(stage, "/World/base_link", translate=(4.0, 5.0, 6.0))
    cfg = NatNetInterfaceConfig(
        server_ip="127.0.0.1", bodies=[BodyBinding("Drone", "/World/base_link", 1)]
    )
    author_interface(stage, "/World/NatNetInterface", cfg)
    mgr, fake = _manager_with_fake()
    mgr.start_server(cfg)

    frame = mgr.sample_once(stage)
    assert frame.nRigidBodies == 1
    rb = frame.RigidBodies[0]
    assert rb.ID == 1
    assert (round(rb.x, 3), round(rb.y, 3), round(rb.z, 3)) == (4.0, 5.0, 6.0)
    assert rb.params & TRACKING_VALID
    # First frame after start resyncs -> client should be told the model list changed.
    assert frame.params & MODEL_LIST_CHANGED


def test_sample_once_missing_prim_is_lost():
    stage = Usd.Stage.CreateInMemory()
    cfg = NatNetInterfaceConfig(
        server_ip="127.0.0.1", bodies=[BodyBinding("Ghost", "/World/missing", 9)]
    )
    author_interface(stage, "/World/NatNetInterface", cfg)
    mgr, fake = _manager_with_fake()
    mgr.start_server(cfg)

    frame = mgr.sample_once(stage)
    rb = frame.RigidBodies[0]
    assert rb.ID == 9
    assert rb.params & TRACKING_VALID == 0
    assert math.isnan(rb.x)


def test_moving_prim_updates_streamed_position():
    stage = Usd.Stage.CreateInMemory()
    xform = _xform(stage, "/World/base_link", translate=(0.0, 0.0, 0.0))
    cfg = NatNetInterfaceConfig(
        server_ip="127.0.0.1", bodies=[BodyBinding("Drone", "/World/base_link", 1)]
    )
    author_interface(stage, "/World/NatNetInterface", cfg)
    mgr, fake = _manager_with_fake()
    mgr.start_server(cfg)

    mgr.sample_once(stage)
    xform.GetOrderedXformOps()[0].Set(Gf.Vec3d(10.0, 0.0, 0.0))
    frame = mgr.sample_once(stage)
    assert round(frame.RigidBodies[0].x, 3) == 10.0


def test_body_added_while_live_is_picked_up_on_resync():
    stage = Usd.Stage.CreateInMemory()
    _xform(stage, "/World/a", translate=(1.0, 0.0, 0.0))
    _xform(stage, "/World/b", translate=(0.0, 2.0, 0.0))
    cfg1 = NatNetInterfaceConfig(
        server_ip="127.0.0.1", bodies=[BodyBinding("A", "/World/a", 1)]
    )
    author_interface(stage, "/World/NatNetInterface", cfg1)
    mgr, fake = _manager_with_fake()
    mgr.start_server(cfg1)

    first = mgr.sample_once(stage)
    assert first.nRigidBodies == 1

    # Add a second body live: re-author the prim, then mark dirty (the UI/USD-notice
    # path calls mark_dirty for us in Kit).
    cfg2 = NatNetInterfaceConfig(
        server_ip="127.0.0.1",
        bodies=[BodyBinding("A", "/World/a", 1), BodyBinding("B", "/World/b", 2)],
    )
    author_interface(stage, "/World/NatNetInterface", cfg2)
    mgr.mark_dirty()

    second = mgr.sample_once(stage)
    assert second.nRigidBodies == 2
    assert second.params & MODEL_LIST_CHANGED  # catalog grew -> tell the client
    ids = {second.RigidBodies[i].ID for i in range(second.nRigidBodies)}
    assert ids == {1, 2}
    # MODELDEF payload was refreshed on the server for the new catalog.
    assert len(fake.payloads) >= 2


def test_target_prim_created_after_start_becomes_valid():
    """A body whose target prim is spawned *after* the server starts (e.g. a Pegasus
    drone base_link created on the first Play tick) must start streaming a valid pose
    as soon as the prim appears — no mark_dirty/resync required, because the target
    path is re-resolved every sample."""
    stage = Usd.Stage.CreateInMemory()
    cfg = NatNetInterfaceConfig(
        server_ip="127.0.0.1", bodies=[BodyBinding("Drone", "/World/drone1/base_link", 1)]
    )
    author_interface(stage, "/World/NatNetInterface", cfg)
    mgr, _fake = _manager_with_fake()
    mgr.start_server(cfg)

    # Prim does not exist yet -> lost.
    first = mgr.sample_once(stage)
    assert first.RigidBodies[0].params & TRACKING_VALID == 0

    # Spawn the target prim later (simulating the Play-tick drone creation).
    _xform(stage, "/World/drone1/base_link", translate=(7.0, 8.0, 9.0))

    # Next sample re-resolves the path -> valid pose, with no mark_dirty().
    second = mgr.sample_once(stage)
    rb = second.RigidBodies[0]
    assert rb.params & TRACKING_VALID
    assert (round(rb.x, 3), round(rb.y, 3), round(rb.z, 3)) == (7.0, 8.0, 9.0)


def test_sample_once_noop_without_server():
    stage = Usd.Stage.CreateInMemory()
    mgr, _fake = _manager_with_fake()
    assert mgr.sample_once(stage) is None
