import numpy as np
from raven_nav.bid_manager import Task, ConsensusAssigner


def test_restricted_tasks_ignore_closer_foreign_bidder():
    from raven_nav.bid_manager import build_tasks
    for status in ('ray', 'bb-observing'):
        tasks = build_tasks([('car', [1, 0, 5], [1, 1, 1], status,
                              None, None, 1.0, {2})], 3, 2)
        solver = ConsensusAssigner(3)
        result = solver.assign(tasks, {1: np.array([0, 0, 5]),
                                       2: np.array([100, 0, 5])}, 1000, 3,
                               explore_dist={1: 1, 2: 1})
        assert result[2] == [tasks[0].key]
        assert result[1] == []
        assert solver.last_debug['tasks'][0]['eligible_agents'] == [2]


def test_restricted_tail_cannot_be_reserved_by_foreign_robot():
    tasks = [Task(('car', i), 'car', np.array([i * 5., 0, 5]), np.ones(3),
                  eligible_agents=frozenset({owner}))
             for i, owner in enumerate((1, 2, 2))]
    result = ConsensusAssigner(3).assign(
        tasks, {1: np.array([0, 0, 5]), 2: np.array([100, 0, 5])}, 0, 3)
    assert result[1] == [tasks[0].key]
    assert set(result[2]) == {tasks[1].key, tasks[2].key}


def test_unknown_or_absent_source_stays_unassigned_and_allows_exploration():
    for sources in (frozenset(), frozenset({3})):
        task = Task(('car', 0), 'car', np.zeros(3), np.ones(3),
                    eligible_agents=sources)
        solver = ConsensusAssigner()
        result = solver.assign([task], {1: np.zeros(3)}, 0, 3,
                               explore_dist={1: 1})
        assert result == {1: []}
        assert solver.last_debug['tasks'][0]['owner'] is None
        import json
        json.dumps(solver.last_debug, allow_nan=False)


def test_fused_bb_accepts_each_actual_producer_only():
    from raven_nav.bid_manager import build_tasks
    items = [('car', [x, 0, 5], [1, 1, 1], 'bb-observing',
              None, None, 1., {aid}) for x, aid in [(0, 1), (1, 3)]]
    tasks = build_tasks(items, 3, 2)
    assert len(tasks) == 1
    assert tasks[0].eligible_agents == frozenset({1, 3})
    result = ConsensusAssigner().assign(
        tasks, {1: np.array([10, 0, 5]), 2: np.array([0, 0, 5]),
                3: np.array([20, 0, 5])}, 0, 3)
    assert result == {1: [tasks[0].key]}
    shared = build_tasks([item[:7] for item in items], 3, 2)
    assert shared[0].eligible_agents is None
    result = ConsensusAssigner().assign(
        shared, {1: np.array([10, 0, 5]), 2: np.array([0, 0, 5])}, 0, 3)
    assert result == {2: [shared[0].key]}


def test_origin_ids_survive_all_bb_fusion_stages():
    from raven_nav.discoveries import (
        ConfirmedTarget, merge_confirmed_targets, merge_similar_targets,
        merge_house_boxes, confirmed_targets_to_json)
    import json
    boxes = [ConfirmedTarget('car', np.array([x, 0., 5.]), np.ones(3) * 4,
                             source_ids=frozenset({aid}))
             for x, aid in [(0., 1), (.1, 2)]]
    for merge in (merge_confirmed_targets, merge_similar_targets, merge_house_boxes):
        result = merge(boxes)
        assert len(result) == 1
        assert result[0].source_ids == frozenset({1, 2})
        assert len(json.loads(confirmed_targets_to_json(result))) == 1


def _nav_method(name):
    # Exercise the real node methods without initializing DDS or a simulator.
    import ast
    from pathlib import Path
    from raven_nav import bid_manager
    source = Path(__file__).parents[1] / 'raven_nav/raven_nav_node.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and any(isinstance(m, ast.FunctionDef) and m.name == name for m in n.body))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    ns = {'np': np, 'bid_manager': bid_manager}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), ns)
    return ns[name]


def test_disabled_ray_sharing_does_not_relabel_peer_raw_rays_as_own():
    from types import SimpleNamespace as S
    own = np.ones((1, 3))
    peer = np.ones((1, 3)) * 10
    node = S(_ray_origins=own, _ray_dirs=own, _ray_scores=np.ones((1, 1)),
             _peer_state=S(peer_rays={'robot_2': S(origins=peer, dirs=peer,
                                                   scores=np.ones((1, 1)))}),
             _share_rays=False)
    merge = _nav_method('_merge_own_and_peer_rays')
    np.testing.assert_array_equal(merge(node)[0], own)
    node._share_rays = True
    assert len(merge(node)[0]) == 2


def test_node_task_table_applies_each_ablation_independently():
    from types import SimpleNamespace as S
    own = dict(label='car', o=np.array([0., 0., 5.]), d=np.array([1., 0., 0.]))
    peer = dict(label='car', o=np.array([30., 0., 5.]), d=np.array([1., 0., 0.]))
    box = S(label='house', center=np.array([80., 0., 5.]), size=np.ones(3),
            status='observing', confidence=1., source_ids=frozenset({2}))
    node = S(_my_id=1, _target_objects=[], _house_boxes=lambda: [box],
             _visited_match=lambda *a: None, _bb_is_visited=lambda *a: False,
             _accumulate_ray_leads=lambda *a: None, _ray_leads=[own],
             _peer_state=S(peer_ray_leads={'robot_2': [peer]}, peer_ids={'robot_2': 2}),
             _lead_points_at_known_bb=lambda *a: False, _lead_reached=lambda *a: False,
             _lead_served=lambda *a: False, _lead_match=lambda *a: None,
             _min_altitude=1., _max_altitude=20., _TASK_MATCH_M=3., _TASK_KEY_GRID=2.)
    build = _nav_method('_build_consensus_tasks')
    for share_rays, share_bbs in [(False, True), (True, False), (False, False), (True, True)]:
        node._share_rays, node._share_bbs = share_rays, share_bbs
        tasks = build(node, [], set(), None, 0, {})
        bbs = [t for t in tasks if t.status.startswith('bb')]
        rays = [t for t in tasks if t.status == 'ray']
        assert bbs[0].eligible_agents == (None if share_bbs else frozenset({2}))
        assert [t.eligible_agents for t in rays] == (
            [None, None] if share_rays else [frozenset({1}), frozenset({2})])
