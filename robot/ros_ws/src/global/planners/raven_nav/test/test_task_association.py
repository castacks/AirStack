"""Association ablations preserve separate live tasks, not repeated messages."""
from types import MethodType, SimpleNamespace as S

import numpy as np
import pytest

from raven_nav.bid_manager import build_tasks
from raven_nav.discoveries import ConfirmedTarget
from raven_nav.ray_targets import build_targets, ray_aabb_hits, is_same_target
from test_ray_targets import _make_group
from test_sharing import _nav_method
from test_local_completion import node as discovery_node


def bind(obj, names):
    for name in names:
        fn = _nav_method(name)
        fn.__globals__.update(SimpleNamespace=S, ray_aabb_hits=ray_aabb_hits,
                              is_same_target=is_same_target)
        setattr(obj, name, MethodType(fn, obj))
    return obj


def box(x=10., source=1):
    return ConfirmedTarget('car', np.array([x, 0., 5.]), np.ones(3)*4,
                           source_ids=frozenset({source}))


def test_task_builder_keeps_duplicate_boxes_when_bb_association_disabled():
    items = [('car', [x, 0, 5], [4, 4, 4], 'bb-observing') for x in (10, 10, 11)]
    assert len(build_tasks(items, 10, 6)) == 1
    tasks = build_tasks(items, 10, 6, associate_bbs=False)
    assert len(tasks) == 3 and len({t.key for t in tasks}) == 3
    assert [t.key for t in tasks] == [t.key for t in build_tasks(
        list(reversed(items)), 10, 6, associate_bbs=False)]
    assert all(np.array_equal(t.size, [4, 4, 4]) for t in tasks)


@pytest.mark.parametrize('enabled,expected', [(True, 1), (False, 3)])
def test_bb_pipeline_preserves_same_cell_sources_and_fragments(enabled, expected):
    obj, own, published = discovery_node(True)
    obj._associate_bbs = enabled
    own.extend([box(), box(10.1)])
    obj._peer_state.peer_confirmed_targets = {'robot_2': [box(10., 2)]}
    obj._my_id=1
    for _ in range(5):
        obj._update_discoveries([])
        assert len(obj._house_boxes()) == expected
    assert len(published[-1]) == (1 if enabled else 2)
    # Completion still suppresses every hypothesis for the completed instance.
    own[0].status='visited'
    obj._update_discoveries([])
    assert not obj._observed_bbs


def lead_node(ray_assoc=True, ray_bb_assoc=True):
    obj = S(_associate_rays=ray_assoc, _associate_ray_bbs=ray_bb_assoc,
            _ray_leads=[], _target_objects=['car'], _observed_bbs={},
            _peer_visited_bbs=[], _RAY_LEAD_BB_PAD_M=.5,
            _RAY_LEAD_MATCH_M=3., _RAY_LEAD_DIR_COS=np.cos(np.deg2rad(12)),
            _RAY_LEAD_DEDUP_M=1.5, _RAY_LEAD_DEDUP_DIR_COS=np.cos(np.deg2rad(6)),
            _lead_served=lambda *args: False, _lead_reached=lambda *args: False,
            _add_served_lead=lambda *args: None)
    return bind(obj, ['_lead_points_at_known_bb', '_lead_match', '_accumulate_ray_leads'])


@pytest.mark.parametrize('enabled,expected', [(True, 1), (False, 2)])
def test_ray_hypotheses_stay_separate_but_repeated_snapshots_do_not_grow(enabled, expected):
    obj = lead_node(enabled)
    groups = [_make_group('car', [0, 0, 5], [1, 0, 0]),
              _make_group('car', [.5, 0, 5], [1, .01, 0])]
    for tick in range(10):
        obj._accumulate_ray_leads(groups, set(), float(tick), {})
        assert len(obj._ray_leads) == expected
    obj._accumulate_ray_leads(groups, {'car'}, 11., {})
    assert not obj._ray_leads  # completed-label handling is unchanged


@pytest.mark.parametrize('enabled,expected', [(True, 0), (False, 1)])
def test_ray_bb_pruning_gate_preserves_completion(enabled, expected):
    obj = lead_node(ray_bb_assoc=enabled)
    obj._observed_bbs={'bb': box()}
    groups=[_make_group('car', [0, 0, 5], [1, 0, 0])]
    obj._accumulate_ray_leads(groups, set(), 1., {})
    assert len(obj._ray_leads) == expected
    obj._peer_visited_bbs=[('car', np.array([10.,0.,5.,4.,4.,4.]))]
    obj._accumulate_ray_leads(groups, set(), 2., {})
    assert not obj._ray_leads


def prepare_builder(obj):
    obj._my_id=1
    obj._share_rays=obj._share_bbs=True
    obj._peer_state=S(peer_ray_leads={}, peer_ids={'robot_2':2})
    obj._house_boxes=lambda: []
    obj._visited_match=lambda *a: None
    obj._bb_is_visited=lambda *a: False
    obj._min_altitude=1.; obj._max_altitude=20.
    obj._TASK_MATCH_M=10.; obj._TASK_KEY_GRID=6.
    bind(obj, ['_build_consensus_tasks'])
    return obj


@pytest.mark.parametrize('enabled,expected', [(True, 1), (False, 2)])
def test_peer_ray_duplicates_reach_actual_auction_table(enabled, expected):
    obj = prepare_builder(lead_node(enabled))
    groups=[_make_group('car', [0, 0, 5], [1, 0, 0])]
    peer={'label':'car', 'o':np.array([.2,0,5]), 'd':np.array([1.,0,0])}
    obj._peer_state.peer_ray_leads={'robot_2':[peer]}
    for tick in range(5):
        tasks=obj._build_consensus_tasks(groups, set(), None, tick, {})
        assert len(tasks)==expected
        assert len({t.key for t in tasks})==expected
        assert all(t.eligible_agents is None for t in tasks)


@pytest.mark.parametrize('enabled,expected', [(True, ['bb-observing']),
                                             (False, ['bb-observing','ray'])])
def test_ray_and_box_can_coexist_in_actual_auction_table(enabled, expected):
    obj=prepare_builder(lead_node(ray_bb_assoc=enabled))
    obj._observed_bbs={'bb':box()}
    obj._house_boxes=lambda:[box()]
    groups=[_make_group('car', [0,0,5], [1,0,0])]
    tasks=obj._build_consensus_tasks(groups,set(),None,1.,{})
    assert [t.status for t in tasks]==expected


def test_discovery_triangulation_and_bb_attachment_flags_are_independent():
    a=_make_group('car',[0,10,5],[1,0,0])
    b=_make_group('car',[10,0,5],[0,1,0])
    assert len(build_targets([a,b],[],[],None,0.))==1
    separate=build_targets([a,b],[],[],None,0.,associate_rays=False)
    assert len(separate)==2 and all(t.status=='unconfirmed' for t in separate)
    boxes=[(0,'car',np.array([10.,10.,5.,4.,4.,4.]))]
    assert build_targets([a],[],boxes,None,0.)[0].bb_id==0
    assert build_targets([a],[],boxes,None,0.,associate_ray_bbs=False)[0].bb_id is None
    # Disabling ray-BB attachment must not disable direct ray triangulation.
    result=build_targets([a,b],[],boxes,None,0.,associate_ray_bbs=False)
    assert len(result)==1 and result[0].bb_id is None and result[0].status=='confirmed'


def test_peer_ray_conflict_gate_does_not_disable_completed_target_filter():
    obj=S(_associate_rays=False, _peer_visited_bbs=[],
          _ray_yields_to_observer=lambda *a:False,
          _ray_point=lambda *a:np.array([10.,10.,5.]),
          _ray_on_peer_bbs=lambda *a:False, _point_near_bbs=lambda *a:False)
    bind(obj, ['_ray_off_limits', '_same_instance'])
    g=_make_group('car',[0,10,5],[1,0,0])
    assert not obj._ray_off_limits(g,None,[('robot_2','car',[10,0,5],[0,1,0],2)])
    assert not obj._same_instance([0,10,5],[1,0,0],[10,0,5],[0,1,0],'car',None)
    obj._ray_on_peer_bbs=lambda *a:True
    assert obj._ray_off_limits(g,None,[])


def test_ray_bb_gate_disables_observer_yield_and_shared_bb_inference():
    obj=S(_associate_ray_bbs=False)
    bind(obj,['_ray_yields_to_observer','_rays_share_bb'])
    assert not obj._ray_yields_to_observer(None,None,'car')
    assert not obj._rays_share_bb(None,None,None,None,'car')
