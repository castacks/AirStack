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
