"""Unit tests for raven_nav.discoveries.

Run inside the robot-desktop container:
  bws --packages-select raven_nav
  pytest src/global/planners/raven_nav/test/test_discoveries.py -v
"""
import json

import numpy as np

from raven_nav.discoveries import (
    ConfirmedTarget, Discovery,
    aabb_overlap, build_discoveries, confirmed_targets_to_json,
    discoveries_to_json, merge_confirmed_targets,
)
from raven_nav.ray_targets import RayTarget


def _ct(label, center, size, **kw):
    return ConfirmedTarget(
        label=label, center=np.array(center, float),
        size=np.array(size, float), **kw,
    )


# ---------- AABB overlap ----------

def test_aabb_overlap_intersecting_returns_true():
    a = _ct('house', [10, 10, 2], [8, 8, 4])
    b = _ct('house', [12, 12, 2], [8, 8, 4])
    assert aabb_overlap(a, b)


def test_aabb_overlap_disjoint_returns_false():
    a = _ct('house', [10, 10, 2], [4, 4, 4])
    b = _ct('house', [30, 30, 2], [4, 4, 4])
    assert not aabb_overlap(a, b)


# ---------- merge_confirmed_targets ----------

def test_merge_collapses_two_overlapping_same_label():
    sources = [
        _ct('house', [10, 10, 2], [8, 8, 4], confidence=0.7),
        _ct('house', [12, 12, 2], [8, 8, 4], confidence=0.8),
    ]
    merged = merge_confirmed_targets(sources)
    assert len(merged) == 1
    # AABB union centered between (6..18, 6..18) -> (11, 11, 2)
    assert abs(merged[0].center[0] - 11.0) < 1e-6
    assert abs(merged[0].center[1] - 11.0) < 1e-6
    assert merged[0].confidence == 0.8


def test_merge_keeps_distinct_when_far_apart():
    sources = [
        _ct('house', [10, 10, 2], [4, 4, 4]),
        _ct('house', [40, 40, 2], [4, 4, 4]),
    ]
    merged = merge_confirmed_targets(sources)
    assert len(merged) == 2


def test_merge_label_mismatch_keeps_separate():
    sources = [
        _ct('house', [10, 10, 2], [8, 8, 4]),
        _ct('car',   [10, 10, 2], [8, 8, 4]),
    ]
    merged = merge_confirmed_targets(sources)
    assert len(merged) == 2


def test_merge_visited_status_sticks():
    sources = [
        _ct('house', [10, 10, 2], [8, 8, 4], status='visited', confidence=0.7),
        _ct('house', [11, 11, 2], [8, 8, 4], status='observing', confidence=0.9),
    ]
    merged = merge_confirmed_targets(sources)
    assert merged[0].status == 'visited'


# ---------- build_discoveries ----------

def test_build_discoveries_from_bb_only():
    cts = [_ct('house', [20, 20, 2], [6, 6, 4], status='observing',
               confidence=0.85)]
    discs = build_discoveries(
        ray_targets=[], confirmed_targets=cts,
        contributing_robot='robot_1',
    )
    assert len(discs) == 1
    d = discs[0]
    assert d.label == 'house'
    assert d.size is not None
    assert d.status == 'observing'
    assert 'robot_1' in d.contributing_robots


def test_build_discoveries_ray_target_absorbed_by_matching_bb():
    cts = [_ct('house', [20, 20, 2], [6, 6, 4], confidence=0.7)]
    rts = [RayTarget(
        label='house', position=np.array([20.5, 20.5, 2.0]),
        contributing_group_ids=[0], status='confirmed',
        confidence=0.95, bb_id=99,
    )]
    discs = build_discoveries(rts, cts, contributing_robot='robot_1')
    # The RayTarget is near the BB → absorbed, not separate.
    assert len(discs) == 1
    # Confidence boosted by ray triangulation.
    assert discs[0].confidence == 0.95


def test_build_discoveries_unconfirmed_ray_target_stays_separate():
    rts = [RayTarget(
        label='house', position=np.array([60.0, 60.0, 5.0]),
        contributing_group_ids=[0], status='unconfirmed', confidence=0.7,
    )]
    discs = build_discoveries(rts, [], contributing_robot='robot_1')
    assert len(discs) == 1
    assert discs[0].status == 'unconfirmed'
    assert discs[0].size is None


def test_discoveries_to_json_roundtrips():
    discs = [Discovery(
        instance_id='abc', label='house',
        position=np.array([10.0, 20.0, 3.0]),
        size=np.array([4.0, 4.0, 4.0]),
        status='confirmed', confidence=0.9,
        contributing_robots=['robot_1', 'robot_2'],
        last_update_ts=12345.6,
    )]
    s = discoveries_to_json(discs)
    parsed = json.loads(s)
    assert parsed[0]['label'] == 'house'
    assert parsed[0]['cx'] == 10.0
    assert parsed[0]['sx'] == 4.0
    assert parsed[0]['contributors'] == ['robot_1', 'robot_2']


def test_confirmed_targets_to_json_roundtrips():
    cts = [_ct('house', [1, 2, 3], [4, 5, 6], status='visited',
               confidence=0.9, ts=42.0)]
    s = confirmed_targets_to_json(cts)
    parsed = json.loads(s)
    assert parsed[0]['label'] == 'house'
    assert parsed[0]['cx'] == 1.0 and parsed[0]['sx'] == 4.0
    assert parsed[0]['status'] == 'visited'
