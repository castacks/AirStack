"""The results file and the two JSON topics, checked against their readers.

`compile_results.py` and `compare_to_groundtruth.py` parse the dump by key
name, and `semantic_search_task._discoveries_cb` parses the discoveries topic
by key name — so these are wire contracts, not internal shapes.
"""
import ast
import json
import os
import pathlib
import tempfile

import numpy as np
import pytest

from raven_nav.detection_memory import DetectionMemory, TargetEventLog
from raven_nav.behaviors.voxel_behavior import VoxelCluster
from raven_nav.discoveries import (
    ConfirmedTarget, build_discoveries, confirmed_targets_to_json,
    discoveries_to_json,
)
from raven_nav.results import (
    CONFIRMED_TARGET_KEYS, RESULT_KEYS, TARGET_EVENT_KEYS,
    build_results_dict, write_results_atomic,
)

HERE = pathlib.Path(__file__).resolve().parent
PKG = HERE.parent / 'raven_nav'


def _payload(**kw):
    m = DetectionMemory()
    m.update([VoxelCluster(label='person',
                           center=np.array([10.0, 5.0, 1.0]),
                           size=np.array([2.0, 2.0, 2.0]),
                           num_voxels=40, confidence=0.97)], 100.0)
    log = TargetEventLog()
    to_world = lambda p: np.asarray(p, dtype=float) + np.array([1000.0, 2000.0, 0.0])
    discoveries = build_discoveries(
        ray_targets=[], confirmed_targets=m.confirmed_targets(),
        contributing_robot='robot_1', now_ts=100.0)
    log.update(discoveries, to_world, 100.0, ['person'])
    base = dict(robot='robot_1', boot_enu=np.array([1000.0, 2000.0, 3.0]),
                alt_ground=3.0, mission_start_ts=90.0, now=123.0,
                completion_reason='', coverage_fraction=0.42,
                coverage_threshold=0.8, path_length_m=17.5,
                num_odom_samples=42, query_labels=['person', 'sky'],
                target_labels=['person'],
                confirmed_targets=m.confirmed_targets(), to_world=to_world,
                target_events=log.events)
    base.update(kw)
    return build_results_dict(**base)


def test_top_level_keys_are_exactly_the_contract():
    assert tuple(_payload().keys()) == RESULT_KEYS


def test_target_event_keys():
    ev = _payload()['target_events'][0]
    assert tuple(ev.keys()) == TARGET_EVENT_KEYS


def test_confirmed_target_keys():
    ct = _payload()['confirmed_targets_enu'][0]
    assert tuple(ct.keys()) == CONFIRMED_TARGET_KEYS


def test_positions_are_lifted_into_global_enu():
    ct = _payload()['confirmed_targets_enu'][0]
    assert ct['center_enu'][0] == pytest.approx(1010.0)
    assert ct['center_enu'][1] == pytest.approx(2005.0)
    # z stays AGL: the ground-truth annotations are ground-relative.
    assert ct['center_enu'][2] == pytest.approx(1.0)


def test_relative_times_are_offsets_from_mission_start():
    ev = _payload()['target_events'][0]
    assert ev['first_discovered_rel_s'] == pytest.approx(10.0)
    assert ev['first_visited_rel_s'] is None


def test_completion_reason_defaults_to_in_progress():
    assert _payload()['completion_reason'] == 'in_progress'
    assert _payload(completion_reason='coverage')['completion_reason'] == 'coverage'


def test_intent_events_is_still_present_and_empty():
    assert _payload()['intent_events'] == []


def test_the_dump_is_json_serialisable_and_atomic():
    with tempfile.TemporaryDirectory() as d:
        path = write_results_atomic(d, 'robot_1', _payload())
        assert os.path.basename(path) == 'robot_1.json'
        with open(path) as f:
            back = json.load(f)
        assert back['robot'] == 'robot_1'
        assert not [f for f in os.listdir(d) if f.endswith('.tmp')]


# ── against the actual readers ──────────────────────────────────────────────
def _string_constants(path, needle_prefix=''):
    """Every string literal in a source file (used to find the keys a reader
    pulls out of our JSON without importing ROS)."""
    tree = ast.parse(pathlib.Path(path).read_text())
    return {n.value for n in ast.walk(tree)
            if isinstance(n, ast.Constant) and isinstance(n.value, str)
            and n.value.startswith(needle_prefix)}


def test_compile_results_reads_only_keys_we_emit():
    """Every `<obj>.get('x')` / `<obj>['x']` key compile_results.py uses on a
    per-robot dump must exist in our payload."""
    consumed = {
        'confirmed_targets_enu', 'target_events', 'path_length_m',
        'mission_duration_s', 'completion_reason', 'coverage_fraction',
        'robot', 'mission_start_ts',
    }
    src = (PKG / 'compile_results.py').read_text()
    payload = _payload()
    for key in consumed:
        assert f"'{key}'" in src or f'"{key}"' in src, \
            f'{key} is no longer read by compile_results.py'
        assert key in payload


def test_compile_results_nested_keys():
    src = (PKG / 'compile_results.py').read_text()
    ct = _payload()['confirmed_targets_enu'][0]
    ev = _payload()['target_events'][0]
    for key in ('label', 'center_enu', 'size', 'status', 'confidence'):
        assert key in ct and f"'{key}'" in src
    for key in ('label', 'pos_enu', 'first_discovered_ts',
                'first_confirmed_ts', 'first_visited_ts'):
        assert key in ev and f"'{key}'" in src


def test_compile_results_can_actually_merge_our_dump():
    import raven_nav.compile_results as cr
    with tempfile.TemporaryDirectory() as d:
        write_results_atomic(d, 'robot_1', _payload())
        merged = cr.main([  # type: ignore[arg-type]
            '--results-dir', d, '--expect', '1', '--wait-timeout', '1',
            '--reason', 'test'])
        out = json.load(open(os.path.join(d, 'compiled_results.json')))
    assert out['houses'], 'the merge produced no houses'
    h = out['houses'][0]
    assert h['label'] == 'person'
    assert h['center_enu'][0] == pytest.approx(1010.0)
    assert merged in (0, None)


# ── the two JSON topics ─────────────────────────────────────────────────────
def test_discoveries_json_matches_what_semantic_search_task_reads():
    """`_discoveries_cb` needs a JSON LIST of objects with `label` and
    `instance_id`; anything else is silently dropped."""
    m = DetectionMemory()
    m.update([VoxelCluster('person', np.array([1.0, 2.0, 3.0]),
                           np.array([2.0, 2.0, 2.0]), 40, 0.9)], 1.0)
    ds = build_discoveries(ray_targets=[],
                           confirmed_targets=m.confirmed_targets(),
                           contributing_robot='robot_1', now_ts=1.0)
    items = json.loads(discoveries_to_json(ds))
    assert isinstance(items, list) and items
    for d in items:
        assert isinstance(d.get('label'), str) and d['label']
        assert isinstance(d.get('instance_id'), str) and d['instance_id']
        assert {'cx', 'cy', 'cz', 'status', 'confidence'} <= set(d)


def test_confirmed_targets_json_keys_match_the_gcs_handler():
    cts = [ConfirmedTarget(label='person', center=np.array([1.0, 2.0, 3.0]),
                           size=np.array([2.0, 2.0, 2.0]), status='visited',
                           confidence=0.9, ts=1.0)]
    item = json.loads(confirmed_targets_to_json(cts))[0]
    assert set(item) == {'label', 'cx', 'cy', 'cz', 'sx', 'sy', 'sz',
                         'status', 'confidence', 'ts'}
