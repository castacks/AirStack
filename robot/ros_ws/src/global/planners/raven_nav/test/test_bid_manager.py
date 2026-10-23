"""Unit tests for raven_nav.bid_manager.

The triangulation-aware CBBA auction is the heart of the multi-drone
coordination: a peer's bid only contests one of mine when our ray groups
converge on the same physical target. Different-target peer bids must NOT knock
me off mine.
"""
import numpy as np

from raven_nav.bid_manager import (
    BidEntry, assign_global, compute_my_bids,
    compute_bb_bids, finalize_bid_values, _same_bid_instance,
    _solve_cbba, _INFEASIBLE_COST,
    Task, build_bb_tasks, ConsensusAssigner,
)
from raven_nav.ray_groups import RayGroup


def _make_group(label, origin, direction, num_rays=5, score=0.9,
                robot_pos=(0, 0, 0)):
    o = np.asarray(origin, float)
    d = np.asarray(direction, float)
    d = d / (np.linalg.norm(d) + 1e-9)
    origins = np.tile(o, (num_rays, 1))
    dirs = np.tile(d, (num_rays, 1))
    scores = np.full(num_rays, score)
    g = RayGroup(label=label, ray_origins=origins, ray_dirs=dirs,
                 ray_scores=scores)
    g._finalize(np.asarray(robot_pos, float))
    g.avg_origin = o
    g.avg_dir = d
    return g


def test_compute_my_bids_one_entry_per_group():
    groups = [
        _make_group('house', [0, 0, 5], [1, 0, 0], robot_pos=(0, 0, 0)),
        _make_group('house', [10, 0, 5], [1, 0, 0], robot_pos=(0, 0, 0)),
        _make_group('car',   [0, 5, 5], [0, 1, 0], robot_pos=(0, 0, 0)),
    ]
    bids = compute_my_bids(groups)
    assert len(bids) == 3
    labels = [b.label for b in bids]
    assert labels.count('house') == 2 and labels.count('car') == 1
    # Bid = -distance; closer (origin at robot) has higher bid.
    house_bids = [b.value for b in bids if b.label == 'house']
    assert max(house_bids) >= min(house_bids)


# ── CBBA consensus assignment ────────────────────────────────────────────────

def test_cbba_removal_prevents_starvation():
    # robot 1 (id 1) is closest to BOTH houses; plain greedy would let it block
    # both. cols = [house_X, house_Y]; removal must give robot 3 the second house.
    cost = np.array([[3.0, 5.0],
                     [8.0, 6.0]])
    assert _solve_cbba(cost, [1, 3]) == {1: 0, 3: 1}


def test_solver_filters_infeasible_pairs():
    cost = np.array([[2.0, _INFEASIBLE_COST]])
    assert _solve_cbba(cost, [1]) == {1: 0}


def test_cbba_ties_break_by_lower_robot_id():
    # Two robots, one target, equal cost → lower id wins it.
    cost = np.array([[4.0],
                     [4.0]])
    assert _solve_cbba(cost, [1, 2]) == {1: 0}


def test_assign_global_solo_picks_closest():
    my = [
        BidEntry('house', -3.0, np.array([0, 0, 5.0]), np.array([1, 0, 0.0]),
                 5, 0.9),
        BidEntry('house', -8.0, np.array([0, 0, 5.0]), np.array([0, 1, 0.0]),
                 5, 0.9),
    ]
    r = assign_global(my_id=1, my_bids=my, peer_bids={}, peer_ids={})
    assert r is not None and r[1].value == -3.0


def test_assign_global_two_different_houses_each_wins():
    my = [BidEntry('house', -5.0, np.array([0.0, 20, 5.0]),
                   np.array([1.0, 0, 0.0]), 5, 0.9)]
    peer = [BidEntry('house', -3.0, np.array([0.0, 80, 5.0]),
                     np.array([1.0, 0, 0.0]), 5, 0.9)]
    me = assign_global(my_id=1, my_bids=my,
                       peer_bids={'robot_2': peer}, peer_ids={'robot_2': 2})
    them = assign_global(my_id=2, my_bids=peer,
                         peer_bids={'robot_1': my}, peer_ids={'robot_1': 1})
    assert me is not None and me[0] == 'house'
    assert them is not None and them[0] == 'house'


# ── BB bids in the auction ───────────────────────────────────────────────────

def test_compute_bb_bids_surface_distance_and_marker():
    bids = compute_bb_bids([('house', [10, 0, 5], [2, 2, 4])], [0, 0, 5])
    assert len(bids) == 1
    b = bids[0]
    assert b.is_bb and b.num_rays == 0
    assert abs(b.value - (-9.0)) < 1e-6
    assert b.avg_dir[0] > 0.9


def test_ray_factor_makes_near_bb_beat_ray():
    ray = BidEntry('house', -5.0, np.array([0, 0, 5.0]),
                   np.array([1, 0, 0.0]), 5, 0.9)
    bb = BidEntry('house', -5.0, np.array([5, 0, 5.0]),
                  np.array([1, 0, 0.0]), 0, 1.0, is_bb=True)
    finalize_bid_values([ray, bb], np.array([1.0, 0.0]), ray_reach_factor=3.0)
    assert abs(bb.value - (-5.0)) < 1e-6
    assert abs(ray.value - (-15.0)) < 1e-6
    assert bb.value > ray.value


def test_behind_penalty_only_behind():
    side = BidEntry('house', -10.0, np.array([0, 10, 5.0]),
                    np.array([0, 1, 0.0]), 0, 1.0, is_bb=True)
    behind = BidEntry('house', -10.0, np.array([-10, 0, 5.0]),
                      np.array([-1, 0, 0.0]), 0, 1.0, is_bb=True)
    finalize_bid_values([side, behind], np.array([1.0, 0.0]), behind_penalty=8.0)
    assert abs(side.value - (-10.0)) < 1e-6
    assert abs(behind.value - (-18.0)) < 1e-6


def test_ray_reach_factor_derates_rays():
    ray = BidEntry('house', -6.0, np.array([0, 0, 5.0]),
                   np.array([1, 0, 0.0]), 5, 0.9)
    finalize_bid_values([ray], None, ray_reach_factor=2.5)
    assert abs(ray.value - (-15.0)) < 1e-6


def test_same_bid_instance_bb_and_ray():
    bb1 = BidEntry('house', -1, np.array([10, 0, 5.0]),
                   np.array([1, 0, 0.0]), 0, 1.0, is_bb=True)
    bb2 = BidEntry('house', -1, np.array([12, 0, 5.0]),
                   np.array([1, 0, 0.0]), 0, 1.0, is_bb=True)
    bb_far = BidEntry('house', -1, np.array([40, 0, 5.0]),
                      np.array([1, 0, 0.0]), 0, 1.0, is_bb=True)
    assert _same_bid_instance(bb1, bb2, None)
    assert not _same_bid_instance(bb1, bb_far, None)
    ray_at = BidEntry('house', -1, np.array([0, 0, 5.0]),
                      np.array([1, 0, 0.0]), 5, 0.9)
    ray_off = BidEntry('house', -1, np.array([0, 0, 5.0]),
                       np.array([0, 1, 0.0]), 5, 0.9)
    assert _same_bid_instance(ray_at, bb1, None)
    assert not _same_bid_instance(ray_off, bb1, None)


def test_assign_global_near_bb_beats_ray_far_bb_loses():
    ray = BidEntry('house', -5.0, np.array([0, 0, 5.0]),
                   np.array([1, 0, 0.0]), 5, 0.9)
    near = BidEntry('house', -8.0, np.array([0, 8, 5.0]),
                    np.array([0, 1, 0.0]), 0, 1.0, is_bb=True)
    finalize_bid_values([ray, near], None, ray_reach_factor=3.0)
    r = assign_global(1, [ray, near], {}, {})
    assert r is not None and r[1].is_bb

    ray2 = BidEntry('house', -5.0, np.array([0, 0, 5.0]),
                    np.array([1, 0, 0.0]), 5, 0.9)
    far = BidEntry('house', -40.0, np.array([0, 40, 5.0]),
                   np.array([0, 1, 0.0]), 0, 1.0, is_bb=True)
    finalize_bid_values([ray2, far], None, ray_reach_factor=3.0)
    r2 = assign_global(1, [ray2, far], {}, {})
    assert r2 is not None and not r2[1].is_bb


# ── CBAA consensus over confirmed-target (BB) tasks ──────────────────────────

def _bb(label, cx, cy, sx=4.0, sy=4.0):
    return (label, np.array([cx, cy, 5.0]), np.array([sx, sy, 6.0]))


def test_build_bb_tasks_merges_same_house_across_robots():
    # Two robots reported the SAME house 9 m apart (cross-robot mapping noise) +
    # a genuinely different house 25 m away. 10 m match -> 2 tasks, not 3.
    bbs = [_bb('house', 0, 0), _bb('house', 9, 0), _bb('house', 0, 25)]
    tasks = build_bb_tasks(bbs, match_m=10.0, key_grid=6.0)
    assert len(tasks) == 2


def test_build_bb_tasks_deterministic_keys_regardless_of_order():
    bbs = [_bb('house', 0, 0), _bb('house', 9, 0), _bb('house', 0, 25)]
    k1 = sorted(t.key for t in build_bb_tasks(bbs, 10.0, 6.0))
    k2 = sorted(t.key for t in build_bb_tasks(list(reversed(bbs)), 10.0, 6.0))
    assert k1 == k2


def _heads(assigned):
    return {ks[0] for ks in assigned.values() if ks}


def _all_keys(assigned):
    return {k for ks in assigned.values() for k in ks}


def test_consensus_two_robots_two_houses_no_conflict():
    # robot 1 near house A (0,0), robot 2 near house B (40,0). Each gets its own.
    tasks = build_bb_tasks([_bb('house', 0, 0), _bb('house', 40, 0)], 10.0, 6.0)
    ca = ConsensusAssigner()
    assigned = ca.assign(tasks, {1: np.array([0, 0, 5.0]), 2: np.array([40, 0, 5.0])},
                         switch_margin=8.0, match_m=10.0)
    assert len(_heads(assigned)) == 2          # conflict-free (distinct tasks)
    a1 = ca.my_task(assigned, 1, tasks)
    a2 = ca.my_task(assigned, 2, tasks)
    assert a1.centroid[0] == 0 and a2.centroid[0] == 40


def test_consensus_identical_on_every_robot():
    # Same inputs -> same assignment regardless of which robot computes it.
    tasks = build_bb_tasks([_bb('house', 0, 0), _bb('house', 40, 0)], 10.0, 6.0)
    pos = {1: np.array([2, 0, 5.0]), 2: np.array([38, 0, 5.0])}
    a = ConsensusAssigner().assign(tasks, pos, 8.0, 10.0)
    b = ConsensusAssigner().assign(tasks, pos, 8.0, 10.0)
    assert a == b


def test_consensus_winner_retention_resists_swap():
    # robot 1 incumbent on A; a small position drift would make a bare-distance
    # solve swap them, but the switch margin keeps the assignment stable.
    tasks = build_bb_tasks([_bb('house', 0, 0), _bb('house', 20, 0)], 10.0, 6.0)
    ca = ConsensusAssigner()
    p0 = {1: np.array([0, 0, 5.0]), 2: np.array([20, 0, 5.0])}
    first = ca.assign(tasks, p0, switch_margin=8.0, match_m=10.0)
    # now both drift so robot 2 is marginally closer to A and robot 1 to B
    p1 = {1: np.array([11, 0, 5.0]), 2: np.array([9, 0, 5.0])}
    second = ca.assign(tasks, p1, switch_margin=8.0, match_m=10.0)
    assert first == second   # retained, no thrash


def test_consensus_three_robots_two_tasks_one_idle():
    tasks = build_bb_tasks([_bb('house', 0, 0), _bb('house', 40, 0)], 10.0, 6.0)
    pos = {1: np.array([0, 0, 5.0]), 2: np.array([40, 0, 5.0]), 3: np.array([20, 0, 5.0])}
    assigned = ConsensusAssigner().assign(tasks, pos, 8.0, 10.0)
    assert len(assigned) == 2          # only 2 tasks -> 3rd robot unassigned
    assert len(_heads(assigned)) == 2


# ── unified task table (rays + BBs) ──────────────────────────────────────────

from raven_nav.bid_manager import build_tasks


def test_build_tasks_unions_bb_fragments():
    # Two BB fragments of one house within match_m fuse into one box.
    items = [
        ('house', np.array([0, 0, 5.0]), np.array([6, 6, 6.0]), 'bb-observing'),
        ('house', np.array([7, 0, 5.0]), np.array([6, 6, 6.0]), 'bb-observing'),
    ]
    tasks = build_tasks(items, match_m=10.0, key_grid=6.0)
    assert len(tasks) == 1 and tasks[0].size[0] > 6.0   # unioned, wider than one


def _ray_task(key_i, x):
    return Task(key=('house', int(round(x / 6)), 0, key_i), label='house',
                centroid=np.array([x, 0, 5.0]), size=np.zeros(3), status='ray')


def test_consensus_peer_repulsion_spreads_to_farther_lead():
    # robot1 takes the lead at its feet (x=0); robot2 then prefers the lead
    # FARTHER from robot1's (peer repulsion), not the equidistant nearer one.
    tasks = [_ray_task(0, 0.0), _ray_task(1, 11.0), _ray_task(2, 9.0)]
    ca = ConsensusAssigner()
    assigned = ca.assign(
        tasks, {1: np.array([0, 0, 5.0]), 2: np.array([10, 0, 5.0])},
        switch_margin=8.0, match_m=10.0)
    assert ca.my_task(assigned, 1, tasks).centroid[0] == 0
    assert ca.my_task(assigned, 2, tasks).centroid[0] == 11   # farther from r1


def test_consensus_closer_peer_takes_drifted_bb():
    # No absolute lock: if the incumbent drifts far and a peer is clearly closer,
    # the peer takes the BB — so no drone flies across the map to hold it.
    bb = Task(key=('house', 1, 0, 0), label='house', centroid=np.array([10, 0, 5.0]),
              size=np.array([4, 4, 6.0]), status='bb-observing')
    ca = ConsensusAssigner()
    ca.assign([bb], {1: np.array([10, 0, 5.0])}, 8.0, 10.0)   # drone 1 takes the BB
    # drone 1 drifted far; drone 2 now sits on the BB -> drone 2 takes it over.
    a2 = ca.assign([bb], {1: np.array([22, 0, 5.0]), 2: np.array([10, 0, 5.0])},
                   8.0, 10.0)
    assert a2.get(2) == [bb.key] and a2.get(1) is None


def test_consensus_retention_holds_against_marginal_peer():
    # Stickiness still works: a peer that is NOT clearly closer can't steal — the
    # incumbent keeps its target (no thrash).
    a = Task(key=('house', 0, 0, 0), label='house', centroid=np.array([0, 0, 5.0]),
             size=np.array([4, 4, 6.0]), status='bb-observing')
    b = Task(key=('house', 3, 0, 1), label='house', centroid=np.array([20, 0, 5.0]),
             size=np.array([4, 4, 6.0]), status='bb-observing')
    ca = ConsensusAssigner()
    ca.assign([a, b], {1: np.array([0, 0, 5.0]), 2: np.array([20, 0, 5.0])}, 8.0, 10.0)
    second = ca.assign([a, b], {1: np.array([11, 0, 5.0]), 2: np.array([9, 0, 5.0])},
                       8.0, 10.0)
    assert second.get(1) == [a.key] and second.get(2) == [b.key]


def test_build_tasks_passes_rays_through_as_is():
    # Every filtered ray is its own task — build_tasks does not re-merge them
    # (clustering already happened upstream in compute_ray_groups).
    items = [
        ('house', np.array([0, 0, 5.0]), np.zeros(3), 'ray', None, np.array([1, 0, 0.0])),
        ('house', np.array([2, 0, 5.0]), np.zeros(3), 'ray', None, np.array([1, 0, 0.0])),
        ('house', np.array([40, 0, 5.0]), np.zeros(3), 'ray', None, np.array([0, 1, 0.0])),
    ]
    tasks = build_tasks(items, match_m=10.0, key_grid=6.0)
    assert len(tasks) == 3
    assert len({t.key for t in tasks}) == 3   # unique keys, none dropped


def test_build_tasks_deterministic_order():
    items = [
        ('house', np.array([0, 0, 5.0]), np.zeros(3), 'ray'),
        ('house', np.array([40, 0, 5.0]), np.zeros(3), 'ray-localized'),
    ]
    a = [t.key for t in build_tasks(items, 10.0, 6.0)]
    b = [t.key for t in build_tasks(list(reversed(items)), 10.0, 6.0)]
    assert sorted(a) == sorted(b)


def test_consensus_assigns_ray_and_bb_tasks_distinctly():
    tasks = build_tasks([
        ('house', np.array([0, 0, 5.0]), np.array([4, 4, 6.0]), 'bb-observing'),
        ('house', np.array([40, 0, 5.0]), np.zeros(3), 'ray'),
    ], 10.0, 6.0)
    pos = {1: np.array([0, 0, 5.0]), 2: np.array([40, 0, 5.0])}
    assigned = ConsensusAssigner().assign(tasks, pos, 8.0, 10.0)
    assert len(_heads(assigned)) == 2


def test_build_tasks_carries_ray_origin_not_bb():
    items = [
        ('house', np.array([40, 0, 5.0]), np.zeros(3), 'ray', np.array([0, 0, 5.0])),
        ('house', np.array([0, 0, 5.0]), np.array([4, 4, 6.0]), 'bb-observing'),
    ]
    tasks = {t.status: t for t in build_tasks(items, 10.0, 6.0)}
    assert tasks['ray'].origin is not None
    assert abs(tasks['ray'].origin[0] - 0.0) < 1e-6
    assert tasks['bb-observing'].origin is None


# ── bundles (L > 1) ──────────────────────────────────────────────────────────

def _bb_task(i, x, y=0.0):
    return Task(key=('house', int(round(x / 6)), int(round(y / 6)), i),
                label='house', centroid=np.array([x, y, 5.0]),
                size=np.array([4, 4, 6.0]), status='bb-observing')


def test_bundle_grows_bb_bb_in_order():
    tasks = [_bb_task(0, 0.0), _bb_task(1, 10.0)]
    assigned = ConsensusAssigner(bundle_len=2).assign(
        tasks, {1: np.array([0, 0, 5.0])}, 8.0, 10.0)
    assert assigned == {1: [tasks[0].key, tasks[1].key]}


def test_bundle_ray_tail_is_terminal():
    # [BB, ray] never grows a 3rd even at L=3; the far BB stays unassigned.
    tasks = [_bb_task(0, 0.0), _ray_task(1, 6.0), _bb_task(2, 40.0)]
    ca = ConsensusAssigner(bundle_len=3)
    assigned = ca.assign(tasks, {1: np.array([0, 0, 5.0])}, 8.0, 10.0)
    assert assigned == {1: [tasks[0].key, tasks[1].key]}
    assert [tasks[2].key] == [tuple(k) for k in
                              ca.last_debug['unassigned_tasks']]


def test_ray_head_never_grows():
    tasks = [_ray_task(0, 0.0), _ray_task(1, 10.0)]
    assigned = ConsensusAssigner(bundle_len=3).assign(
        tasks, {1: np.array([0, 0, 5.0])}, 8.0, 10.0)
    assert assigned == {1: [tasks[0].key]}


def test_bundle_detour_cap_blocks_far_task():
    tasks = [_bb_task(0, 0.0), _bb_task(1, 50.0)]
    assigned = ConsensusAssigner(bundle_len=2).assign(
        tasks, {1: np.array([0, 0, 5.0])}, 8.0, 10.0)
    assert assigned == {1: [tasks[0].key]}


def test_heads_first_no_hoarding():
    # 2 agents, 2 tasks, L=2: every agent gets a head before any bundle grows.
    tasks = [_bb_task(0, 0.0), _bb_task(1, 8.0)]
    assigned = ConsensusAssigner(bundle_len=2).assign(
        tasks, {1: np.array([0, 0, 5.0]), 2: np.array([9, 0, 5.0])}, 8.0, 10.0)
    assert all(len(ks) == 1 for ks in assigned.values())
    assert len(_all_keys(assigned)) == 2


def test_bundle_len_1_no_growth():
    tasks = [_bb_task(0, 0.0), _bb_task(1, 10.0)]
    assigned = ConsensusAssigner(bundle_len=1).assign(
        tasks, {1: np.array([0, 0, 5.0])}, 8.0, 10.0)
    assert assigned == {1: [tasks[0].key]}


def test_my_task_returns_head():
    tasks = [_bb_task(0, 0.0), _bb_task(1, 10.0)]
    ca = ConsensusAssigner(bundle_len=2)
    assigned = ca.assign(tasks, {1: np.array([0, 0, 5.0])}, 8.0, 10.0)
    assert ca.my_task(assigned, 1, tasks).key == tasks[0].key


# ── geo prior + decline (outside option) ─────────────────────────────────────

def test_far_agent_declines_to_explore():
    # a2 is far from a task owned by a1 and has a nearby frontier -> declines;
    # the task waits for its owner instead of luring a2 across the map.
    tasks = [_bb_task(0, 0.0), _bb_task(1, 20.0)]
    pos = {1: np.array([0, 0, 5.0]), 2: np.array([100, 0, 5.0])}
    assigned = ConsensusAssigner().assign(
        tasks, pos, 8.0, 10.0, explore_dist={2: 15.0})
    assert assigned[1] == [tasks[0].key]
    assert assigned[2] == []                       # declined
    assert ConsensusAssigner().my_task(assigned, 2, tasks) is None


def test_far_agent_without_frontier_still_goes():
    tasks = [_bb_task(0, 0.0), _bb_task(1, 20.0)]
    pos = {1: np.array([0, 0, 5.0]), 2: np.array([100, 0, 5.0])}
    assigned = ConsensusAssigner().assign(tasks, pos, 8.0, 10.0)
    assert assigned[2] == [tasks[1].key]


def test_owner_never_declines():
    # The closest agent takes the task even with a frontier at its feet.
    tasks = [_bb_task(0, 0.0)]
    assigned = ConsensusAssigner().assign(
        tasks, {1: np.array([30, 0, 5.0])}, 8.0, 10.0,
        explore_dist={1: 2.0})
    assert assigned == {1: [tasks[0].key]}


def test_owner_takes_own_task_not_foreign_pair():
    # 02-07-04 far-split regression: an agent that owns a ray elsewhere must
    # not cross the map for a BB pair a nearby agent can bundle — it takes its
    # OWN task; the pair goes to the near agent as head + tail.
    ray = Task(key=('house', 10, 0, 2), label='house',
               centroid=np.array([60, 0, 5.0]), size=np.zeros(3), status='ray')
    tasks = [_bb_task(0, 0.0), _bb_task(1, 10.0), ray]
    pos = {1: np.array([0, 0, 5.0]), 2: np.array([50, 0, 5.0])}
    assigned = ConsensusAssigner(bundle_len=2).assign(
        tasks, pos, 8.0, 10.0, explore_dist={2: 12.0})
    assert assigned[2] == [ray.key]
    assert assigned[1] == [tasks[0].key, tasks[1].key]


def test_poach_gate_allows_near_tie_task():
    # Owning something elsewhere doesn't forbid taking a task you're
    # effectively tied on (geo excess within the margin) once its owner is
    # busy — only *clearly-foreign* tasks are gated.
    ray = Task(key=('house', 0, 0, 2), label='house',
               centroid=np.array([2, 0, 5.0]), size=np.zeros(3), status='ray')
    bb2 = Task(key=('house', 1, 0, 0), label='house',
               centroid=np.array([6, 0, 5.0]), size=np.array([4, 4, 6.0]),
               status='bb-observing')
    bb3 = Task(key=('house', 1, 1, 1), label='house',
               centroid=np.array([5, 6, 5.0]), size=np.array([4, 4, 6.0]),
               status='bb-observing')
    pos = {1: np.array([0, 0, 5.0]), 2: np.array([5, 5, 5.0])}
    assigned = ConsensusAssigner().assign([ray, bb2, bb3], pos, 8.0, 10.0)
    assert assigned[2] == [bb3.key]     # owner of both BBs takes the closest
    assert assigned[1] == [bb2.key]     # a1 near-ties on bb2 despite owning ray


def test_decline_hysteresis_holds_explore():
    tasks = [_bb_task(0, 0.0), _bb_task(1, 20.0)]
    ca = ConsensusAssigner()
    a1 = ca.assign(tasks, {1: np.array([0, 0, 5.0]),
                           2: np.array([50, 0, 5.0])},
                   8.0, 10.0, explore_dist={2: 0.0})
    assert a1[2] == []
    # With the current 30 m exploration markup, a2 drifts to where a fresh
    # solve takes the task (27.95 vs 30), but retention holds (26 vs 27.95).
    pos2 = {1: np.array([0, 0, 5.0]), 2: np.array([46, 0, 5.0])}
    a2 = ca.assign(tasks, pos2, 8.0, 10.0, explore_dist={2: 0.0})
    assert a2[2] == []
    fresh = ConsensusAssigner().assign(tasks, pos2, 8.0, 10.0,
                                       explore_dist={2: 0.0})
    assert fresh[2] == [tasks[1].key]


# ── age-weighted peers ───────────────────────────────────────────────────────

def test_stale_peer_loses_ownership_to_live_agent():
    task = _bb_task(0, 10.0)
    pos = {1: np.array([16, 0, 5.0]), 2: np.array([30, 0, 5.0])}
    live = ConsensusAssigner().assign([task], pos, 8.0, 10.0)
    assert live == {1: [task.key]}                 # a1 closer when both live
    faded = ConsensusAssigner().assign(
        [task], pos, 8.0, 10.0, agent_weight={1: 0.1, 2: 1.0})
    assert faded == {2: [task.key]}                # stale a1 stops winning


def test_unit_weights_match_no_weights():
    tasks = [_bb_task(0, 0.0), _bb_task(1, 20.0), _ray_task(2, 35.0)]
    pos = {1: np.array([2, 0, 5.0]), 2: np.array([30, 0, 5.0])}
    a = ConsensusAssigner(bundle_len=2).assign(tasks, pos, 8.0, 10.0)
    b = ConsensusAssigner(bundle_len=2).assign(
        tasks, pos, 8.0, 10.0, agent_weight={1: 1.0, 2: 1.0},
        explore_dist={})
    assert a == b


def test_solve_debug_trace_populated():
    tasks = [_bb_task(0, 0.0)]
    ca = ConsensusAssigner()
    ca.assign(tasks, {1: np.array([0, 0, 5.0])}, 8.0, 10.0)
    dbg = ca.last_debug
    assert dbg['result'] == {'1': [list(tasks[0].key)]}
    assert dbg['passes'][0]['picks'][0]['agent'] == 1
    assert {'base', 'ray_pen', 'repulsion', 'peer_heading', 'retention'} <= set(
        dbg['passes'][0]['picks'][0])
    import json
    json.dumps(dbg)                                # JSON-safe


def test_trace_replay_round_trip():
    # A JSON-serialized solve trace replays to the identical assignment —
    # the invariant the offline analyzer's flip-rate metrics depend on.
    import json
    from raven_nav.analyze_auction_solve import norm_logged, replay_solve
    tasks = [_bb_task(0, 0.0), _bb_task(1, 10.0), _bb_task(2, 20.0),
             _ray_task(3, 35.0)]
    pos = {1: np.array([1, 0, 5.0]), 2: np.array([60, 0, 5.0])}
    ca = ConsensusAssigner(bundle_len=2)
    ca.assign(tasks, pos, 8.0, 10.0,
              agent_weight={1: 1.0, 2: 0.6}, explore_dist={2: 12.0})
    # second tick so the trace carries non-empty prev/prev_explored state
    pos2 = {1: np.array([4, 0, 5.0]), 2: np.array([55, 0, 5.0])}
    ca.assign(tasks, pos2, 8.0, 10.0,
              agent_weight={1: 1.0, 2: 0.6}, explore_dist={2: 12.0})
    trace = json.loads(json.dumps(ca.last_debug))
    assert replay_solve(trace) == norm_logged(trace)
