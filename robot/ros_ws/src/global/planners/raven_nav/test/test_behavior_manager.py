"""BehaviorManager — priority chain, mode switching, and the interaction
between the four behaviours on one shared TickContext."""
import numpy as np
import pytest

from helpers import ctx, rays, scores_for, voxel_box, frontier_cloud_flu
from raven_nav.behavior_manager import BehaviorManager


def _mgr(**kw):
    kw.setdefault('rng', np.random.default_rng(0))
    return BehaviorManager(**kw)


def _full_ctx(vox=False, ray=False, guiding_ray=False, frontier=True,
              **kw):
    """A context with any combination of the four behaviours' triggers.
    Columns: 0 person (target), 1 sky, 2 roof (a guiding label)."""
    parts = {}
    if vox:
        pts = voxel_box((20.0, 0.0, 3.0))
        parts['vox_xyz'] = pts
        parts['vox_scores'] = scores_for(pts.shape[0], 3, 0, 0.99)
    origins, dirs, scores = [], [], []
    if ray:
        origins.append((30.0, 0.0, 6.0)); dirs.append((1.0, 0.0, 0.0))
        scores.append([0.99, 0.005, 0.005])
    if guiding_ray:
        origins.append((0.0, 30.0, 6.0)); dirs.append((0.0, 1.0, 0.0))
        scores.append([0.005, 0.005, 0.99])
    if origins:
        parts.update(rays(origins, dirs, np.array(scores)))
    if frontier:
        pts = np.array([[50.0, 0.0, 6.0]]) + np.random.default_rng(3).normal(
            scale=0.2, size=(10, 3))
        parts['frontiers'] = frontier_cloud_flu(pts)
    kw.setdefault('cur_pose', np.array([0.0, 0.0, 6.0]))
    return ctx(query_labels=['person', 'sky', 'roof'],
               target_objects=['person'], **parts, **kw)


def test_priority_order_is_voxel_ray_lvlm_frontier():
    m = _mgr()
    assert [b.name for b in m.behaviors] == [
        'Voxel-based', 'Ray-based', 'LVLM-guided', 'Frontier-based']


def test_frontier_when_nothing_fires():
    m = _mgr()
    c = _full_ctx()
    m.perceive(c)
    assert m.mode_select(c) == 'Frontier-based'


def test_ray_beats_frontier():
    m = _mgr()
    c = _full_ctx(ray=True)
    m.perceive(c)
    assert m.mode_select(c) == 'Ray-based'


def test_voxel_beats_ray():
    m = _mgr()
    c = _full_ctx(vox=True, ray=True)
    m.perceive(c)
    assert m.mode_select(c) == 'Voxel-based'


def test_lvlm_beats_frontier_but_loses_to_ray():
    m = _mgr()
    m.lvlm_behavior.set_guiding_objects('roof')
    c = _full_ctx(guiding_ray=True)
    m.perceive(c)
    assert m.mode_select(c) == 'LVLM-guided'
    c2 = _full_ctx(ray=True, guiding_ray=True)
    m.perceive(c2)
    assert m.mode_select(c2) == 'Ray-based'


def test_lvlm_disabled_falls_through_to_frontier():
    m = _mgr(lvlm_enabled=False)
    m.lvlm_behavior.set_guiding_objects('roof')
    c = _full_ctx(guiding_ray=True)
    m.perceive(c)
    assert m.mode_select(c) == 'Frontier-based'


def test_frontier_only_short_circuits_every_semantic_behaviour():
    """The surviving baseline (deviation 3): even with a voxel cluster and a
    hot ray, navigation stays frontier — but perception still runs."""
    m = _mgr(frontier_only=True)
    c = _full_ctx(vox=True, ray=True)
    m.perceive(c)
    assert m.mode_select(c) == 'Frontier-based'
    assert m.voxel_behavior.clusters, 'passive detection must keep working'
    assert m.ray_behavior.analysis.has_rays


def test_frontier_only_never_triggers_the_lvlm():
    m = _mgr(frontier_only=True)
    m.lvlm_behavior.set_guiding_objects('roof')
    c = _full_ctx(guiding_ray=True)
    m.perceive(c)
    m.mode_select(c)
    assert m.lvlm_behavior.want_trigger is False


def test_behavior_execute_dispatches_by_name():
    m = _mgr()
    c = _full_ctx(vox=True, ray=True)
    m.perceive(c)
    vox_out = m.behavior_execute('Voxel-based', c)
    ray_out = m.behavior_execute('Ray-based', c)
    assert vox_out.path and ray_out.path
    assert vox_out.path[1][0] != ray_out.path[1][0]


def test_unknown_mode_falls_back_to_frontier():
    m = _mgr()
    c = _full_ctx()
    m.perceive(c)
    assert m.behavior_execute('Nonsense', c).path


def test_mode_switch_resets_lock_and_waypoints():
    """OG mapping_server_rosnode.py:508-511 `mode_switch_trigger`. Modelled
    here the way the node does it: on a change, clear lock + both waypoints
    before executing."""
    m = _mgr()
    c1 = _full_ctx(vox=True)
    m.perceive(c1)
    mode1 = m.mode_select(c1)
    out1 = m.behavior_execute(mode1, c1)
    assert mode1 == 'Voxel-based' and out1.waypoint_locked is True

    c2 = _full_ctx(vox=False, ray=True, waypoint_locked=out1.waypoint_locked,
                   target_waypoint=out1.target_waypoint,
                   target_waypoint2=out1.target_waypoint2)
    m.perceive(c2)
    mode2 = m.mode_select(c2)
    assert mode2 == 'Ray-based'
    # the node resets before executing
    c2.waypoint_locked = False
    c2.target_waypoint = None
    c2.target_waypoint2 = None
    out2 = m.behavior_execute(mode2, c2)
    assert not np.allclose(out2.path[1], out1.path[1])


def test_completed_queries_follows_visited_clusters():
    m = _mgr()
    c = _full_ctx(vox=True, cur_pose=np.array([18.0, 0.0, 3.0]))
    m.perceive(c)
    assert m.completed_queries == []
    m.behavior_execute('Voxel-based', c)
    assert m.completed_queries == ['person']


def test_perceive_is_safe_with_no_map_at_all():
    m = _mgr()
    c = ctx(cur_pose=np.zeros(3))
    m.perceive(c)
    assert m.mode_select(c) == 'Frontier-based'
    assert m.behavior_execute('Frontier-based', c).path == []
