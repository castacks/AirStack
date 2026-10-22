import numpy as np
import pytest
from Scripts.Setup.td3_long_night import select_windows


def rows(times):
    return [{m+'_left': {'time_ns': int(t)} for m in ['rgb', 'thermal']} for t in times]


def test_midpoints_ties_and_prior_exclusion():
    sample=rows(np.arange(90))
    selected=select_windows(sample,0,100,set(),frames=10)
    assert [s['manifest_start'] for s in selected]==[10,40,70]
    selected=select_windows(sample,0,100,{10},frames=10)
    assert selected[0]['manifest_start']==11


def test_preserves_large_positive_gaps():
    times=np.arange(90,dtype=np.int64)*100000000;times[45:]+=5000000000
    selected=select_windows(rows(times),0,20000000000,set(),frames=10)
    assert max(selected[1]['gaps_ns']['rgb'])==5100000000
    assert selected[1]['duration_seconds']['rgb']==5.9


def test_reference_and_empty_region_rejected():
    sample=rows(np.arange(90))
    selected=select_windows(sample,12,100,set(),frames=10)
    assert selected[0]['manifest_start']==12
    with pytest.raises(ValueError,match='No eligible'):
        select_windows(sample,0,100,set(range(30)),frames=10)
