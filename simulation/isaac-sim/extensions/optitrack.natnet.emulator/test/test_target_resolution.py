# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Commit 3 — target prim resolution against a real (in-memory) USD stage.

Guarded by ``importorskip("pxr")`` so the suite stays green without usd-core.
Covers no / single / multiple bodies, missing prims, empty targets, and multiple
catalog entries pointing at the same prim.
"""

from __future__ import annotations

import pytest

pytest.importorskip("pxr")

from pxr import Usd, UsdGeom  # noqa: E402

from optitrack.natnet.emulator.isaac.config import BodyBinding, NatNetInterfaceConfig  # noqa: E402
from optitrack.natnet.emulator.isaac.usd_bindings import resolve_targets  # noqa: E402

pytestmark = pytest.mark.unit


def _stage_with(*paths):
    stage = Usd.Stage.CreateInMemory()
    for path in paths:
        UsdGeom.Xform.Define(stage, path)
    return stage


def _names(bodies):
    return [b.rigid_body_name for b in bodies]


def test_no_bodies_resolves_empty():
    stage = _stage_with("/World/base_link")
    existing, missing = resolve_targets(stage, NatNetInterfaceConfig())
    assert existing == []
    assert missing == []


def test_single_existing_target():
    stage = _stage_with("/World/base_link")
    cfg = NatNetInterfaceConfig(bodies=[BodyBinding("Drone", "/World/base_link", 1)])
    existing, missing = resolve_targets(stage, cfg)
    assert _names(existing) == ["Drone"]
    assert missing == []


def test_missing_target_is_reported():
    stage = _stage_with("/World/base_link")
    cfg = NatNetInterfaceConfig(bodies=[BodyBinding("Ghost", "/World/does_not_exist", 1)])
    existing, missing = resolve_targets(stage, cfg)
    assert existing == []
    assert _names(missing) == ["Ghost"]


def test_empty_target_is_missing():
    stage = _stage_with("/World/base_link")
    cfg = NatNetInterfaceConfig(bodies=[BodyBinding("Unpointed", "", 1)])
    existing, missing = resolve_targets(stage, cfg)
    assert existing == []
    assert _names(missing) == ["Unpointed"]


def test_mixed_existing_and_missing():
    stage = _stage_with("/World/a", "/World/c")
    cfg = NatNetInterfaceConfig(
        bodies=[
            BodyBinding("A", "/World/a", 1),
            BodyBinding("B", "/World/b", 2),  # missing
            BodyBinding("C", "/World/c", 3),
        ]
    )
    existing, missing = resolve_targets(stage, cfg)
    assert _names(existing) == ["A", "C"]
    assert _names(missing) == ["B"]


def test_multiple_entries_same_prim_all_resolve():
    stage = _stage_with("/World/shared")
    cfg = NatNetInterfaceConfig(
        bodies=[
            BodyBinding("First", "/World/shared", 1),
            BodyBinding("Second", "/World/shared", 2),
        ]
    )
    existing, missing = resolve_targets(stage, cfg)
    assert _names(existing) == ["First", "Second"]
    assert missing == []
