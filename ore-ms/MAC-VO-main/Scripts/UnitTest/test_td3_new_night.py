import numpy as np
import pytest
from Scripts.Setup.td3_new_night import first_windows


def test_first_chronological_not_motion_selected():
    t = np.arange(90, dtype=np.int64)*100000000
    windows = first_windows([t, t+1000000], 0, 9000000000)
    assert [w['manifest_start'] for w in windows] == [0, 30, 60]


def test_reference_and_gap_boundaries():
    t = np.arange(120, dtype=np.int64)*100000000
    other = t.copy(); other[42:] += 1000000000
    windows = first_windows([t, other], 300000000, 14000000000)
    assert [w['manifest_start'] for w in windows] == [3, 42, 80]


def test_empty_region_and_timeline_mismatch_fail():
    with pytest.raises(ValueError, match='No eligible'):
        first_windows([np.arange(30), np.arange(30)], 0, 30)
    with pytest.raises(ValueError, match='aligned'):
        first_windows([np.arange(30), np.arange(29)], 0, 30)
