import pytest
from Scripts.Setup.td3_long_recording import select_long_windows


def rows(count=600):
    return [{m+'_left':{'time_ns':1_000_000_000+i*100_000_000} for m in ['rgb','thermal']} for i in range(count)]


def test_midpoint_ties_and_prior_exclusion():
    r=rows()
    selected=select_long_windows(r,0,100_000_000_000,set(),frames=100)
    assert [w['manifest_start'] for w in selected]==[50,250,450]
    selected=select_long_windows(r,0,100_000_000_000,set(range(50,70)),frames=100)
    assert selected[0]['manifest_start']==70


def test_missing_captures_require_explicit_gap_bound():
    r=rows()
    for i,row in enumerate(r):
        for m in ['rgb','thermal']:row[m+'_left']['time_ns']+=100_000_000*(i//30)
    with pytest.raises(ValueError,match='No eligible'):select_long_windows(r,0,100_000_000_000,set())
    selected=select_long_windows(r,0,100_000_000_000,set(),max_gap_ns=250_000_000)
    assert all(w['max_gap_ns']['thermal']==200_000_000 for w in selected)
    assert all(w['gaps_over_150ms']['rgb']>0 for w in selected)


def test_coverage_and_insufficient_data_do_not_silently_shorten():
    with pytest.raises(ValueError):select_long_windows(rows(200),0,100_000_000_000,set())
    with pytest.raises(ValueError):select_long_windows(rows(),0,1_000_000_001,set())
