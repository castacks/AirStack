# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""In-flight plan revisions (accept_plan_revisions), used by receding-horizon
planners such as tigris_search_planner: the same plan_id re-published with a
newer stamp and an unchanged prefix extends the active sortie."""

import importlib
import math
import sys
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent
for p in (_HERE, _HERE.parent):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import _ros_stubs as S  # noqa: E402
from test_follower_node import make_plan, tick  # noqa: E402

pytestmark = pytest.mark.unit


@pytest.fixture
def node_mod(monkeypatch):
    S.install(monkeypatch)
    monkeypatch.delitem(sys.modules, "mtl_trajectory_follower.follower_node", raising=False)
    return importlib.import_module("mtl_trajectory_follower.follower_node")


def fly(node, pos, ticks):
    tp = node.pubs["trajectory_controller/tracking_point"]
    for _ in range(ticks):
        tick(node)
        if not node.active:
            break
        c = tp.msgs[-1].pose.position
        d = math.dist((c.x, c.y, c.z), pos)
        if d > 1e-6:
            s = min(0.3, d) / d
            pos = [pos[i] + s * (v - pos[i]) for i, v in enumerate((c.x, c.y, c.z))]
        node.subs["odometry"](S.odom(*pos))
    return pos


def test_revision_is_ignored_by_default(node_mod):
    S.Node.overrides = {}
    node = node_mod.MtlTrajectoryFollower()
    node.subs["odometry"](S.odom(0, 0, 30))
    node.subs["search/plan"](make_plan(n=100))
    S.SimClock.advance(1.0)
    node.subs["search/plan"](make_plan(n=300))
    assert node.follower.track.total == pytest.approx(99 * 0.5)


def test_revision_extends_the_active_sortie(node_mod, monkeypatch):
    monkeypatch.setattr(S.Node, "overrides", {"accept_plan_revisions": True})
    node = node_mod.MtlTrajectoryFollower()
    pos = [0.0, 0.0, 30.0]
    node.subs["odometry"](S.odom(*pos))
    node.subs["search/plan"](make_plan(n=100))            # 49.5 m
    pos = fly(node, pos, 60)                                # ~15 m along
    progress = node.follower.progress
    assert node.state == 2 and progress > 5.0               # SEARCH
    S.SimClock.advance(0.5)
    node.subs["search/plan"](make_plan(n=300))              # same prefix, 149.5 m
    assert node.follower.track.total == pytest.approx(299 * 0.5)
    assert node.follower.progress == pytest.approx(progress)
    # the latched copy of the same revision is a no-op
    node.subs["search/plan"](node.plan)
    pos = fly(node, pos, 3000)
    states = [m.state_name for m in node.pubs["search/follower_status"].msgs]
    assert states[-1] == "COMPLETE"
    assert node.follower.progress > 140.0                   # it flew the extension


def test_late_revision_after_hand_back_does_not_restart(node_mod, monkeypatch):
    monkeypatch.setattr(S.Node, "overrides", {"accept_plan_revisions": True})
    node = node_mod.MtlTrajectoryFollower()
    pos = [0.0, 0.0, 30.0]
    node.subs["odometry"](S.odom(*pos))
    node.subs["search/plan"](make_plan(n=60))
    fly(node, pos, 3000)
    assert not node.active
    S.SimClock.advance(0.5)
    node.subs["search/plan"](make_plan(n=300))
    assert not node.active
