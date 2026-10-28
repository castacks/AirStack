"""Counterexamples distinguishing each bidding-cost ablation from its control."""
import numpy as np
import pytest

from raven_nav.bid_manager import ConsensusAssigner, IndependentGreedyAssigner, Task
from raven_nav.analyze_auction_solve import replay_solve, norm_logged


def task(i, x, status='bb-observing'):
    return Task(('car', i), 'car', np.array([float(x), 0., 0.]),
                np.zeros(3), status=status)


@pytest.mark.parametrize('assigner', [ConsensusAssigner, IndependentGreedyAssigner])
@pytest.mark.parametrize('status', ['ray', 'ray-localized'])
def test_zero_ray_penalty_changes_head_without_changing_default(assigner, status):
    ray, box = task(0, 10, status), task(1, 20)
    args = ([ray, box], {1: np.zeros(3)}, 8., 1.)
    assert assigner(1).assign(*args)[1] == [box.key]
    solver = assigner(1)
    assert solver.assign(*args, ray_penalty=0.)[1] == [ray.key]
    assert solver.last_debug['params']['ray_penalty'] == 0.


def test_ray_penalty_applies_to_tail_and_preserves_ray_terminal_rule():
    head, ray, box = task(0, 5), task(1, 10, 'ray'), task(2, 20)
    args = ([head, ray, box], {1: np.zeros(3)}, 8., 1.)
    assert ConsensusAssigner(3).assign(*args)[1][:2] == [head.key, box.key]
    solver = ConsensusAssigner(3)
    assert solver.assign(*args, ray_penalty=0.)[1] == [head.key, ray.key]
    assert all(p.get('ray_pen') == 0 for ps in solver.last_debug['passes']
               for p in ps['picks'])


def test_zero_retention_switches_incumbent_but_keeps_ray_penalty():
    incumbent, closer, ray = task(0, 20), task(1, 16), task(2, 5, 'ray')
    for margin, expected in [(8., incumbent.key), (0., closer.key)]:
        solver = ConsensusAssigner(1)
        solver.assign([incumbent], {1: np.zeros(3)}, margin, 1.)
        assert solver.assign([incumbent, closer, ray], {1: np.zeros(3)},
                             margin, 1.)[1] == [expected]
        assert solver.last_debug['params']['ray_penalty'] == 50.
        if margin == 0:
            assert solver.last_debug['passes'][0]['picks'][0]['retention'] == 0.


def test_zero_retention_removes_tail_and_exploration_bonus():
    head, tail, closer = task(0, 5), task(1, 20), task(2, 17)
    for margin, expected in [(8., tail.key), (0., closer.key)]:
        solver = ConsensusAssigner(2)
        solver._prev = {1: [head.centroid, tail.centroid]}
        assert solver.assign([head, tail, closer], {1: np.zeros(3)},
                             margin, 1.)[1] == [head.key, expected]
    unavailable = task(3, 40)
    unavailable.eligible_agents = frozenset({2})
    solver = ConsensusAssigner(1)
    solver._prev_explored = {1}
    solver.assign([unavailable], {1: np.zeros(3)}, 0., 1., explore_dist={1: 10.})
    assert solver.last_debug['passes'][0]['picks'][0]['retention'] == 0.


def test_replay_uses_logged_coefficient_and_can_remove_penalty():
    tasks = [task(0, 10, 'ray'), task(1, 20)]
    for penalty in (0., 50.):
        solver = ConsensusAssigner(1)
        solver.assign(tasks, {1: np.zeros(3)}, 8., 1., ray_penalty=penalty)
        assert replay_solve(solver.last_debug) == norm_logged(solver.last_debug)
        assert replay_solve(solver.last_debug, 'no_ray_penalty')[1] == [tasks[0].key]


@pytest.mark.parametrize('value', [-1., float('nan'), float('inf')])
def test_invalid_ray_penalty_rejected(value):
    with pytest.raises(ValueError, match='finite and nonnegative'):
        ConsensusAssigner().assign([task(0, 10)], {1: np.zeros(3)}, 8., 1.,
                                   ray_penalty=value)


@pytest.mark.parametrize('assigner', [ConsensusAssigner, IndependentGreedyAssigner])
def test_zero_distance_changes_head_and_keeps_ray_penalty(assigner):
    far, near, ray = task(0, 30), task(1, 10), task(2, 5, 'ray')
    args = ([far, near, ray], {1: np.zeros(3)}, 8., 1.)
    assert assigner(1).assign(*args)[1] == [near.key]
    assert assigner(1).assign(*args, distance_weight=1.)[1] == [near.key]
    solver = assigner(1)
    # Equal BB costs now tie by key; the nearer ray still has its penalty.
    assert solver.assign(*args, distance_weight=0.)[1] == [far.key]
    assert solver.last_debug['params']['distance_weight'] == 0.
    assert solver.last_debug['params']['ray_penalty'] == 50.


def test_no_distance_preserves_retention_and_tail_geometry():
    head, near, far = task(1, 10), task(2, 20), task(0, 100)
    solver = ConsensusAssigner(3)
    solver._prev = {1: [head.centroid]}
    result = solver.assign([head, near, far], {1: np.zeros(3)}, 8., 1.,
                           distance_weight=0.)
    assert result[1] == [head.key, near.key]  # far still exceeds detour cap
    picks = [p for ps in solver.last_debug['passes'] for p in ps['picks']]
    assert all(p['base'] == 0. and p['raw_distance'] > 0. for p in picks)
    assert picks[0]['retention'] == -8.
    assert picks[1]['repulsion'] > 0.


def test_no_distance_removes_exploration_travel_only():
    unavailable = task(0, 40)
    unavailable.eligible_agents = frozenset({2})
    solver = ConsensusAssigner(1)
    solver._prev_explored = {1}
    solver.assign([unavailable], {1: np.zeros(3)}, 8., 1.,
                  explore_dist={1: 80.}, distance_weight=0.)
    pick = solver.last_debug['passes'][0]['picks'][0]
    assert pick['base'] == 0. and pick['raw_distance'] == 80.
    assert pick['cost'] == pick['markup'] - 4.


def test_replay_uses_distance_weight_and_old_trace_default():
    tasks = [task(0, 30), task(1, 10)]
    solver = ConsensusAssigner(1)
    solver.assign(tasks, {1: np.zeros(3)}, 8., 1., distance_weight=0.)
    assert replay_solve(solver.last_debug) == norm_logged(solver.last_debug)
    del solver.last_debug['params']['distance_weight']
    assert replay_solve(solver.last_debug)[1] == [tasks[1].key]
    assert replay_solve(solver.last_debug, 'no_distance_cost')[1] == [tasks[0].key]


@pytest.mark.parametrize('value', [-1., float('nan'), float('inf')])
def test_invalid_distance_weight_rejected(value):
    with pytest.raises(ValueError, match='finite and nonnegative'):
        ConsensusAssigner().assign([task(0, 10)], {1: np.zeros(3)}, 8., 1.,
                                   distance_weight=value)


def test_task_repulsion_switch_changes_second_robot_allocation():
    head, near, far = task(0, 0, 'ray'), task(1, 9, 'ray'), task(2, 11, 'ray')
    args = ([head, near, far], {1: np.zeros(3), 2: np.array([10., 0., 0.])}, 8., 1.)
    assert ConsensusAssigner(1).assign(*args)[2] == [far.key]
    solver = ConsensusAssigner(1)
    assert solver.assign(*args, task_repulsion=False)[2] == [near.key]
    assert all(p['repulsion'] == 0 for ps in solver.last_debug['passes'] for p in ps['picks'])


def test_destination_repulsion_switch_changes_head():
    near, far = task(0, 10), task(1, -11)
    args = ([near, far], {1: np.zeros(3)}, 8., 1.)
    wp = {2: near.centroid}
    assert ConsensusAssigner(1).assign(*args, agent_waypoints=wp)[1] == [far.key]
    solver = ConsensusAssigner(1)
    assert solver.assign(*args, agent_waypoints=wp, destination_repulsion=False)[1] == [near.key]
    assert solver.last_debug['passes'][0]['picks'][0]['peer_heading'] == 0


@pytest.mark.parametrize('task_on,destination_on', [(True, True),(False, True),(True, False),(False, False)])
def test_repulsion_switches_are_independent_in_bundle_tails(task_on, destination_on):
    head, tail = task(0, 10), task(1, 20)
    solver = ConsensusAssigner(2)
    solver.assign([head, tail], {1: np.zeros(3)}, 8., 1.,
                  agent_waypoints={2: np.array([20., 0., 0.])},
                  task_repulsion=task_on, destination_repulsion=destination_on)
    picks = [p for ps in solver.last_debug['passes'] for p in ps['picks']]
    assert len(picks) == 2
    assert (picks[1]['repulsion'] > 0) == task_on
    assert all((p['peer_heading'] > 0) == destination_on for p in picks)
    assert all(p['base'] > 0 for p in picks)
    assert solver.last_debug['params']['ray_penalty'] == 50.
    assert solver.last_debug['params']['switch_margin'] == 8.
    assert replay_solve(solver.last_debug) == norm_logged(solver.last_debug)


@pytest.mark.parametrize('destination_on', [True, False])
def test_ray_destination_geometry_survives_trace_replay(destination_on):
    ray, box = task(0, 10, 'ray'), task(1, 12)
    ray.origin = np.zeros(3)
    ray.direction = np.array([1., 0., 0.])
    solver = ConsensusAssigner(1)
    solver.assign([ray, box], {1: np.zeros(3)}, 8., 1., ray_penalty=0.,
                  agent_waypoints={2: np.array([100., 0., 0.])},
                  destination_repulsion=destination_on)
    assert replay_solve(solver.last_debug) == norm_logged(solver.last_debug)
    assert solver.last_debug['result']['1'] == [list(box.key if destination_on else ray.key)]
