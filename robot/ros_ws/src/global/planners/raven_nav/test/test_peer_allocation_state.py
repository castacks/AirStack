"""Shared perception with peer planning state removed; exercise real node methods."""
import ast
from pathlib import Path
from types import MethodType, SimpleNamespace as S

import numpy as np
import pytest

from raven_nav.bid_manager import ConsensusAssigner, Task
from test_sharing import _nav_method


def node(enabled):
    peer = S(peer_ids={'robot_2': 2}, peer_positions={'robot_2': [10., 0., 5.]},
             peer_waypoints={'robot_2': [15., 0., 5.]},
             peer_last_seen={'robot_2': 100.}, peer_explore_dist={'robot_2': 2.},
             peer_committed_instance={'robot_2': ('car', [0, 0, 5], [1, 0, 0])},
             peer_frontiers={'robot_2': np.ones((2, 3))},
             peer_completed_zones={'robot_2': np.ones((1, 2))},
             peer_completed={'robot_2': {'house'}},
             peer_confirmed_targets={'robot_2': ['shared-box']},
             peer_ray_leads={'robot_2': ['shared-ray']})
    obj = S(_use_peer_allocation_state=enabled, _peer_state=peer,
            _my_id=1, _cur_pose=np.array([0., 0., 5.]),
            _target_waypoint=np.array([10., 0., 5.]),
            _PEER_GRACE_S=5., _PEER_TTL_S=20., _independent_greedy=False)
    for name in ('_agent_positions', '_agent_waypoints', '_agent_weights',
                 '_agent_explore_dists', '_peer_weights_by_name',
                 '_planning_peer_state', '_fresh_peer_committed_instances',
                 '_ray_yields_to_observer'):
        fn = _nav_method(name)
        fn.__globals__['SimpleNamespace'] = S
        setattr(obj, name, MethodType(fn, obj))
    return obj


@pytest.mark.parametrize('enabled', [True, False])
def test_all_solver_inputs_follow_gate(enabled):
    obj = node(enabled)
    expected = {1, 2} if enabled else {1}
    peers = {'robot_2'}
    assert set(obj._agent_positions(peers)) == expected
    assert set(obj._agent_waypoints(peers)) == expected
    assert set(obj._agent_weights(100., peers)) == expected
    assert set(obj._agent_explore_dists(4., peers)) == expected
    assert bool(obj._fresh_peer_committed_instances(100.)) == enabled
    assert obj._agent_explore_dists(-1., set()) == {}


def test_peer_motion_and_selection_cannot_change_length_three_bundle():
    obj = node(False)
    tasks = [Task(('car', i), 'car', np.array([10. + 5*i, 0., 5.]),
                  np.ones(3), status='bb-observing') for i in range(4)]
    results = []
    for x in (-100., 10., 15., 100.):
        obj._peer_state.peer_positions['robot_2'] = [x, 0., 5.]
        obj._peer_state.peer_waypoints['robot_2'] = [x + 5., 0., 5.]
        obj._peer_state.peer_explore_dist['robot_2'] = abs(x)
        peers = {'robot_2'}
        result = ConsensusAssigner(3).assign(
            tasks, obj._agent_positions(peers), 0., 3.,
            agent_weight=obj._agent_weights(100., peers),
            agent_waypoints=obj._agent_waypoints(peers),
            explore_dist=obj._agent_explore_dists(100., peers))
        assert set(result) == {1}
        assert len(result[1]) == 3
        results.append(result)
    assert all(r == results[0] for r in results)


def test_behavior_view_retains_shared_information_without_mutating_cache():
    obj = node(False)
    view = obj._planning_peer_state()
    assert not view.peer_positions and not view.peer_waypoints
    assert not view.peer_committed_instance and not view.peer_explore_dist
    for field in ('peer_frontiers', 'peer_completed_zones', 'peer_completed',
                  'peer_confirmed_targets', 'peer_ray_leads'):
        assert getattr(view, field) is getattr(obj._peer_state, field)
    assert obj._peer_state.peer_positions
    assert obj._peer_state.peer_committed_instance
    obj._use_peer_allocation_state = True
    assert obj._planning_peer_state() is obj._peer_state


def test_frontier_repulsion_removed_but_coverage_retained():
    # Load this numerical function without importing ROS/Torch behavior modules.
    path = Path(__file__).parents[1] / 'raven_nav/behaviors/frontier_behavior.py'
    fn = next(n for n in ast.parse(path.read_text()).body
              if isinstance(n, ast.FunctionDef) and n.name == '_peer_penalty')
    ns = {'np': np}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), ns)
    obj = node(False)
    points = np.array([[10., 0., 5.], [15., 0., 5.]])
    raw, _ = ns['_peer_penalty'](points, obj._peer_state, 1)
    masked, _ = ns['_peer_penalty'](points, obj._planning_peer_state(), 1)
    assert np.all(raw > 0)
    np.testing.assert_array_equal(masked, [0., 0.])
    assert obj._planning_peer_state().peer_completed_zones


def test_target_yielding_does_not_consume_peer_inputs_when_disabled():
    obj = node(False)
    # No observer geometry or ray projection needed: the gate must short-circuit.
    assert not obj._ray_yields_to_observer(None, None, 'car')
    assert obj._fresh_peer_committed_instances(100.) == []


def test_omitted_setting_preserves_existing_peer_allocation():
    obj = node(True)
    del obj._use_peer_allocation_state
    assert set(obj._agent_positions({'robot_2'})) == {1, 2}
    assert obj._planning_peer_state() is obj._peer_state
