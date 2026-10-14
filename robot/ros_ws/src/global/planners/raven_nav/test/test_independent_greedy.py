"""Behavioral controls for the allocation and bundle-length experiments."""
import json
import numpy as np
import pytest

from raven_nav.bid_manager import Task, ConsensusAssigner, IndependentGreedyAssigner
from test_sharing import _nav_method


def task(i, x, status='bb-observing', eligible=None):
    return Task(('car', i), 'car', np.array([float(x), 0., 5.]),
                np.zeros(3), status=status, eligible_agents=eligible)


def test_greedy_allows_duplicate_heads_but_auction_does_not():
    tasks = [task(0, 10), task(1, 30)]
    pos = {1: np.zeros(3), 2: np.array([1., 0., 0.])}
    g = IndependentGreedyAssigner()
    result = g.assign(tasks, pos, 0, 3)
    assert result == {1: [tasks[0].key], 2: [tasks[0].key]}
    auction = ConsensusAssigner(1).assign(tasks, pos, 0, 3)
    assert len({ks[0] for ks in auction.values() if ks}) == 2
    json.dumps(g.last_debug, allow_nan=False)


def test_peer_position_weight_and_waypoint_cannot_change_local_choice():
    tasks = [task(0, 10), task(1, 15)]
    for peer_x in (-100, 10, 15, 100):
        result = IndependentGreedyAssigner().assign(
            tasks, {1: np.zeros(3), 2: np.array([peer_x, 0, 0])}, 0, 3,
            agent_weight={1: 1, 2: .001}, explore_dist={1: 0, 2: 0},
            agent_waypoints={2: np.array([10, 0, 0])})
        assert result[1] == [tasks[0].key]


def test_local_ray_penalty_retention_and_disappearing_task():
    g = IndependentGreedyAssigner()
    pos = {1: np.zeros(3)}
    tasks = [task(0, 5, 'ray'), task(1, 20)]
    assert g.assign(tasks, pos, 8, 3)[1] == [tasks[1].key]
    # A slightly closer new box must not remove the existing hysteresis.
    tasks.append(task(2, 16))
    assert g.assign(tasks, pos, 8, 3)[1] == [tasks[1].key]
    assert g.assign([tasks[0], tasks[2]], pos, 8, 3)[1] == [tasks[2].key]
    assert g.assign([], pos, 8, 3) == {}
    assert g.my_task({}, 1, []) is None


def test_local_eligibility_and_exploration():
    g = IndependentGreedyAssigner()
    tasks = [task(0, 10, eligible=frozenset({2}))]
    result = g.assign(tasks, {1: np.zeros(3), 2: np.zeros(3)}, 0, 3,
                      explore_dist={1: 1, 2: 1})
    assert result == {1: [], 2: [tasks[0].key]}
    assert g.my_task(result, 1, tasks) is None


def test_greedy_rejects_bundle_reservations():
    with pytest.raises(ValueError, match='bundle_len=1'):
        IndependentGreedyAssigner(3)


@pytest.mark.parametrize('limit', [1, 3, 6, 12])
def test_bundle_sweep_can_reach_each_limit_and_stays_exclusive(limit):
    tasks = [task(i, i * 5) for i in range(40)]
    result = ConsensusAssigner(limit).assign(
        tasks, {1: np.zeros(3), 2: np.array([200, 0, 0])}, 0, 3)
    assert all(len(ks) == limit for ks in result.values())
    flattened = [k for ks in result.values() for k in ks]
    assert len(flattened) == len(set(flattened))


def test_ray_terminal_and_detour_gate_remain_active_at_large_limit():
    pos = {1: np.zeros(3)}
    tasks = [task(0, 0), task(1, 100)]
    assert len(ConsensusAssigner(12).assign(tasks, pos, 0, 3)[1]) == 1
    rays = [task(i, i * 5, 'ray') for i in range(10)]
    assert len(ConsensusAssigner(12).assign(rays, pos, 0, 3)[1]) == 1


def test_greedy_node_does_not_yield_to_peer_intentions():
    from types import SimpleNamespace as S
    node = S(_independent_greedy=True)
    assert _nav_method('_fresh_peer_committed_instances')(node, 0) == []
    assert not _nav_method('_ray_yields_to_observer')(node, None, None, 'car')


def test_greedy_still_excludes_visited_ray_targets():
    from types import SimpleNamespace as S
    node = S(_ray_yields_to_observer=lambda *a: False,
             _ray_point=lambda *a: np.zeros(3),
             _peer_visited_bbs=[('car', np.zeros(6))],
             _ray_on_peer_bbs=lambda *a: True)
    ray = S(avg_origin=np.zeros(3), avg_dir=np.ones(3), label='car')
    assert _nav_method('_ray_off_limits')(node, ray, None, [])
