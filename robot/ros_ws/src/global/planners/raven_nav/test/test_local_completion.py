"""Shared perception with per-robot completion, using the real node methods."""
import json
from types import MethodType, SimpleNamespace as S

import numpy as np
import pytest

from raven_nav import discoveries as disc
from test_sharing import _nav_method


def target(x=10., status='visited', label='car'):
    return disc.ConfirmedTarget(label, np.array([x, 0., 5.]), np.ones(3)*2,
                                status=status, confidence=.9, ts=1.)


def method(name):
    fn = _nav_method(name)
    fn.__globals__.update({
        k: getattr(disc, k) for k in [
            'ConfirmedTarget', 'merge_confirmed_targets', 'merge_similar_targets',
            'merge_house_boxes', 'same_instance_xy', 'confirmed_targets_to_json']})
    fn.__globals__.update(String=lambda **kw: S(**kw),
                          build_targets=lambda **kw: [],
                          build_discoveries=lambda **kw: [],
                          discoveries_to_json=lambda d: '[]')
    return fn


def node(shared):
    own=[]; published=[]
    obj=S(_use_shared_completion=shared, _MIN_BB_CENTER_Z=-.5,
          _peer_state=S(peer_confirmed_targets={'robot_2':[target()]},
                        peer_ids={'robot_2':2}, peer_positions={},
                        peer_completed={'robot_2':{'peer_only'}}),
          _behavior_manager=S(completed_queries={'own_done'},
                              voxel_behavior=S(peer_visited_bbs=[])),
          _own_confirmed_targets=lambda:own, _peer_visited_bbs=[],
          _observed_bbs={}, _fresh_peers=lambda now:set(),
          _boot_enu=None, _search_area_xy=None, _robot_name='robot_1',
          get_clock=lambda:S(now=lambda:S(nanoseconds=10**9)),
          _record_dropped_bb=lambda *a,**kw:None,
          _confirmed_targets_pub=S(publish=lambda msg:published.append(json.loads(msg.data))),
          _discoveries_pub=S(publish=lambda msg:None),
          _update_target_events=lambda d:None,
          _publish_discoveries_table=lambda d:None)
    for name in ['_peer_confirmed_targets_flat','_completed_target_labels',
                 '_visited_match','_bb_is_visited','_add_visited','_obs_key',
                 '_house_boxes','_remember_observed_bbs','_update_discoveries']:
        setattr(obj,name,MethodType(method(name),obj))
    return obj,own,published


@pytest.mark.parametrize('shared', [True,False])
def test_only_completion_status_changes_not_peer_geometry(shared):
    n,_,_=node(shared)
    original=n._peer_state.peer_confirmed_targets['robot_2'][0]
    t=n._peer_confirmed_targets_flat()[0]
    assert t.status==('visited' if shared else 'observing')
    for name in ['center','size']:
        np.testing.assert_array_equal(getattr(t,name),getattr(original,name))
    assert (t.label,t.confidence,t.ts,t.source_ids)==('car',.9,1.,frozenset({2}))
    assert original.status=='visited'  # never alter the received payload


@pytest.mark.parametrize('shared', [True,False])
def test_completed_labels_are_local_unless_shared_enabled(shared):
    n,_,_=node(shared)
    assert n._completed_target_labels()==({'own_done','peer_only'} if shared else {'own_done'})
    assert n._behavior_manager.completed_queries=={'own_done'}
    assert n._peer_state.peer_completed=={'robot_2':{'peer_only'}}


def test_peer_visit_does_not_poison_local_persistent_state_or_fused_box():
    n,_,_=node(False)
    for _ in range(3):
        n._update_discoveries([])
        assert not n._peer_visited_bbs
        assert not n._behavior_manager.voxel_behavior.peer_visited_bbs
        boxes=n._house_boxes()
        assert len(boxes)==1 and boxes[0].status=='observing'
        assert not n._bb_is_visited('car',boxes[0].center,boxes[0].size)


def test_own_visit_still_excludes_target_and_is_published():
    n,own,published=node(False)
    n._update_discoveries([])
    own.append(target())
    n._update_discoveries([])
    assert n._bb_is_visited('car',own[0].center,own[0].size)
    assert n._house_boxes()[0].status=='visited'
    assert published[-1][0]['status']=='visited'
    # A later peer update cannot undo local completion.
    n._peer_state.peer_confirmed_targets['robot_2'][0].status='observing'
    n._update_discoveries([])
    assert n._house_boxes()[0].status=='visited'


def test_default_shared_mode_preserves_peer_completion():
    n,_,_=node(True)
    n._update_discoveries([])
    assert n._house_boxes()[0].status=='visited'
    assert len(n._behavior_manager.voxel_behavior.peer_visited_bbs)==1
    # Shared completion remains sticky when the peer disappears.
    n._peer_state.peer_confirmed_targets={}
    n._update_discoveries([])
    assert n._house_boxes()[0].status=='visited'


def test_observing_boxes_still_shared_in_both_modes():
    for shared in [True,False]:
        n,_,_=node(shared)
        n._peer_state.peer_confirmed_targets['robot_2'][0].status='observing'
        n._update_discoveries([])
        assert len(n._house_boxes())==1
        assert n._house_boxes()[0].status=='observing'


def test_own_completion_does_not_complete_other_robot_or_other_instance():
    first,own,_=node(False)
    second,_,_=node(False)
    own.append(target())
    first._update_discoveries([])
    second._update_discoveries([])
    assert first._bb_is_visited('car',target().center,target().size)
    assert not second._bb_is_visited('car',target().center,target().size)
    assert not first._bb_is_visited('car',target(x=100).center,target().size)


@pytest.mark.parametrize('shared', [True,False])
def test_peer_visited_box_remains_a_shared_candidate_only_in_local_mode(shared):
    n,own,_=node(shared)
    n._target_objects=[]
    n._accumulate_ray_leads=lambda *a:None
    n._ray_leads=[]
    n._peer_state.peer_ray_leads={}
    n._share_bbs=n._share_rays=True
    n._TASK_MATCH_M=10.
    n._TASK_KEY_GRID=6.
    n._update_discoveries([])
    build=method('_build_consensus_tasks')
    tasks=build(n,[],n._completed_target_labels(),None,1.,{})
    if shared:
        assert tasks==[]
    else:
        assert len(tasks)==1 and tasks[0].eligible_agents is None
        assert tasks[0].status=='bb-observing'
        # Receiving a BB is allowed; only this robot's subsequent completion
        # removes it from this robot's task table.
        own.append(target())
        n._update_discoveries([])
        assert build(n,[],n._completed_target_labels(),None,2.,{})==[]
