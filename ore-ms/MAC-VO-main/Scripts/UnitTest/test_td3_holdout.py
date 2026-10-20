from types import SimpleNamespace

import numpy as np
import pytest

from Scripts.Setup.td3_holdout_validation import select_windows


class StaticReference:
    def __init__(self, lo=0, hi=20_000_000_000):
        self.time_ns = np.array([lo, hi], dtype=np.int64)

    def at(self, times):
        assert np.all(times >= self.time_ns[0]) and np.all(times <= self.time_ns[-1])
        return np.repeat(np.eye(4)[None], len(times), axis=0)


def rows(count=90):
    return [{'rgb_left': {'time_ns': i*100_000_000},
             'thermal_left': {'time_ns': i*100_000_000+10_000_000}} for i in range(count)]


def test_thirds_and_earliest_tie_break():
    selected = select_windows(StaticReference(), rows())
    assert [w['manifest_start'] for w in selected] == [0, 30, 60]
    assert [w['region'] for w in selected] == [[0, 30], [30, 60], [60, 90]]


def test_thermal_gap_rejects_windows_even_when_rgb_is_continuous():
    data = rows()
    for r in data[5:]:
        r['thermal_left']['time_ns'] += 100_000_000
    assert select_windows(StaticReference(), data)[0]['manifest_start'] == 5


def test_reference_coverage_requires_both_modalities():
    data = rows()
    data[0]['thermal_left']['time_ns'] = -1
    assert select_windows(StaticReference(), data)[0]['manifest_start'] == 1


def test_ineligible_third_is_not_relaxed():
    data = rows()
    for i in range(30):
        data[i]['thermal_left']['time_ns'] = i*200_000_000
    with pytest.raises(ValueError, match='early'):
        select_windows(StaticReference(), data)


def test_insufficient_data_rejected():
    with pytest.raises(ValueError, match='60'):
        select_windows(StaticReference(), rows(59))


def test_maximum_motion_selected_within_each_third():
    class MotionReference(StaticReference):
        def at(self, times):
            poses = super().at(times)
            poses[:, 0, 3] = (times/1e9)**2
            return poses
    selected = select_windows(MotionReference(), rows())
    assert [w['manifest_start'] for w in selected] == [10, 40, 70]
