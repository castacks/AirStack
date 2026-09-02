"""DetectionMemory + TargetEventLog — the reporting side (deviation 4)."""
import numpy as np
import pytest

from raven_nav.detection_memory import (
    DetectionMemory, TargetEventLog, aabb_surface_dist,
)
from raven_nav.behaviors.voxel_behavior import VoxelCluster
from raven_nav.discoveries import build_discoveries


def cluster(label='person', center=(20.0, 0.0, 1.0), size=(2.0, 2.0, 2.0),
            conf=0.99, n=40):
    return VoxelCluster(label=label, center=np.array(center, dtype=float),
                        size=np.array(size, dtype=float), num_voxels=n,
                        confidence=conf)


def test_surface_distance_is_zero_inside_the_box():
    assert aabb_surface_dist([0, 0, 0], np.zeros(3), [0, 0, 0], [2, 2, 2]) == 0.0


def test_a_cluster_becomes_an_observing_target():
    m = DetectionMemory()
    out = m.update([cluster()], now_ts=1.0)
    assert len(out) == 1
    assert out[0].label == 'person'
    assert out[0].status == 'observing'


def test_targets_persist_after_the_cluster_disappears():
    m = DetectionMemory()
    m.update([cluster()], 1.0)
    assert len(m.update([], 2.0)) == 1


def test_low_confidence_is_dropped_when_a_floor_is_set():
    m = DetectionMemory(min_confidence=0.5)
    assert m.update([cluster(conf=0.2)], 1.0) == []
    assert len(m.update([cluster(conf=0.8)], 1.0)) == 1


def test_underground_boxes_are_dropped():
    m = DetectionMemory()
    assert m.update([cluster(center=(20.0, 0.0, -5.0))], 1.0) == []


def test_flying_close_flips_a_target_to_visited():
    m = DetectionMemory()
    m.update([cluster(center=(20.0, 0.0, 1.0), size=(2.0, 2.0, 2.0))], 1.0)
    assert m.confirmed_targets()[0].status == 'observing'
    m.mark_reached(np.array([22.0, 0.0, 1.0]))     # 1 m off the surface
    assert m.confirmed_targets()[0].status == 'visited'
    assert m.completed_labels() == ['person']


def test_flying_past_at_range_does_not_flip_it():
    m = DetectionMemory()
    m.update([cluster()], 1.0)
    m.mark_reached(np.array([40.0, 0.0, 1.0]))
    assert m.confirmed_targets()[0].status == 'observing'
    assert m.completed_labels() == []


def test_the_voxel_behaviour_can_mark_it_visited_directly():
    m = DetectionMemory()
    c = cluster()
    m.update([c], 1.0)
    assert m.mark_visited(c.label, c.center, c.size) is True
    assert m.update([c], 2.0)[0].status == 'visited'
    assert m.mark_visited(c.label, c.center, c.size) is False


def test_visited_is_sticky_after_the_cluster_vanishes():
    m = DetectionMemory()
    c = cluster()
    m.update([c], 1.0)
    m.mark_visited(c.label, c.center, c.size)
    m.update([], 2.0)
    targets = m.confirmed_targets()
    assert len(targets) == 1 and targets[0].status == 'visited'


def test_two_labels_report_separately():
    m = DetectionMemory()
    m.update([cluster(label='person', center=(20.0, 0.0, 1.0)),
              cluster(label='car', center=(-20.0, 0.0, 1.0))], 1.0)
    assert sorted(ct.label for ct in m.confirmed_targets()) == ['car', 'person']


def test_frontier_only_style_passive_detection():
    """The frontier baseline never drives the voxel behaviour, but its
    clusters still land here and its fly-bys still count as visits."""
    m = DetectionMemory()
    for t in range(5):
        m.update([cluster(center=(10.0, 0.0, 1.0))], float(t))
        m.mark_reached(np.array([float(t) * 3.0, 0.0, 1.0]))
    assert m.completed_labels() == ['person']


# ── event log ───────────────────────────────────────────────────────────────
def _discoveries(status='observing', with_box=True, center=(10.0, 0.0, 1.0)):
    from raven_nav.discoveries import ConfirmedTarget
    cts = [ConfirmedTarget(label='person', center=np.array(center, dtype=float),
                           size=np.array([2.0, 2.0, 2.0]), status=status,
                           confidence=0.9, ts=0.0)] if with_box else []
    return build_discoveries(ray_targets=[], confirmed_targets=cts,
                             contributing_robot='robot_1', now_ts=0.0)


def test_event_log_records_each_milestone_once():
    log = TargetEventLog()
    ident = lambda p: np.asarray(p, dtype=float)
    got = log.update(_discoveries(), ident, 10.0, ['person'])
    kinds = [k for k, _ in got]
    assert kinds == ['DISCOVERED', 'CONFIRMED']
    assert log.update(_discoveries(), ident, 11.0, ['person']) == []
    got = log.update(_discoveries(status='visited'), ident, 12.0, ['person'])
    assert [k for k, _ in got] == ['VISITED']
    ev = log.events[0]
    assert ev['first_discovered_ts'] == 10.0
    assert ev['first_confirmed_ts'] == 10.0
    assert ev['first_visited_ts'] == 12.0


def test_event_log_filters_by_target_label():
    log = TargetEventLog()
    ident = lambda p: np.asarray(p, dtype=float)
    assert log.update(_discoveries(), ident, 1.0, ['car']) == []
    assert log.events == []


def test_event_log_matches_a_drifting_box_to_the_same_instance():
    log = TargetEventLog()
    ident = lambda p: np.asarray(p, dtype=float)
    log.update(_discoveries(center=(10.0, 0.0, 1.0)), ident, 1.0, ['person'])
    log.update(_discoveries(center=(13.0, 0.0, 1.0)), ident, 2.0, ['person'])
    assert len(log.events) == 1
    log.update(_discoveries(center=(90.0, 0.0, 1.0)), ident, 3.0, ['person'])
    assert len(log.events) == 2


def test_event_positions_go_through_the_world_transform():
    log = TargetEventLog()
    to_world = lambda p: np.asarray(p, dtype=float) + np.array([100.0, 200.0, 0.0])
    log.update(_discoveries(), to_world, 1.0, ['person'])
    assert log.events[0]['pos_enu'][0] == pytest.approx(110.0)
    assert log.events[0]['pos_enu'][1] == pytest.approx(200.0)
