"""Causal availability decisions; no pose-quality ranking is assumed."""
import copy
import pytest
from Scripts.UnitTest.test_pair_arbiter_events import fixture, resequence
from Scripts.Setup.shared_arbiter_replay import advisory, schedule


def inputs():
    reports,events=fixture()
    return reports['rgb']['input_rows'],events


def test_both_available_fixed_priority_and_no_body_claim():
    rows,events=inputs();d=advisory(rows,0,events)
    assert d['action']=='RGB' and d['reason']=='both_available_fixed_rgb_preference'
    assert not d['pose_gate_training_ready'] and d['camera_local_advice_only']
    assert not d['evidence']['rgb']['body_transform_verified']


@pytest.mark.parametrize('blocked',['rgb','thermal','both'])
def test_late_result_fallback_or_abstain(blocked):
    rows,events=inputs()
    for c in (('rgb','thermal') if blocked=='both' else (blocked,)):
        events[c]=events[c][:2] # first submission + health, result not observed
    d=advisory(rows,0,events)
    assert d['action']=={'rgb':'THERMAL','thermal':'RGB','both':'ABSTAIN'}[blocked]


@pytest.mark.parametrize('fault',['unsupported','health_missing','interval_mismatch','partial','execution_error'])
def test_ineligible_rgb_uses_thermal(fault):
    rows,events=inputs();e=events['rgb']
    if fault=='unsupported':e[1]['original_need_interp']=True
    elif fault=='health_missing':e.pop(1);resequence(e)
    elif fault=='interval_mismatch':e[0]['time0_ns']-=1;e[2]['time0_ns']-=1
    elif fault=='partial':e[1]['retained_count']=None
    else:e[1]['execution_error']='RuntimeError'
    assert advisory(rows,0,events)['action']=='THERMAL'


def test_no_retroactive_shutdown_result():
    rows,events=inputs();prefix={c:copy.deepcopy(e[:2]) for c,e in events.items()}
    decision=advisory(rows,0,prefix)
    assert decision['action']=='ABSTAIN'
    for c in events:events[c][2]['stage']='termination'
    assert advisory(rows,0,events)['action']=='RGB'
    assert decision==advisory(rows,0,prefix)
    assert decision['observed_event_counts']=={'rgb':2,'thermal':2}


def test_stale_write_does_not_repair_missing_first_result():
    rows,events=inputs()
    for c in events:
        events[c][2]['first_write_for_submission']=False
    assert advisory(rows,0,events)['action']=='ABSTAIN'


def test_schedule_exact_sensor_order_and_independent_intervals():
    rows,events=inputs();ordered=schedule(rows)
    assert [(c,i) for _,c,i in ordered]==[(c,i) for i in range(3) for c in ('thermal','rgb')]
    assert type(ordered[0][0]) is int and ordered[0][0]>2**53
    d=advisory(rows,0,events)
    assert d['evidence']['rgb']['sensor_interval_ns'][0]-d['evidence']['thermal']['sensor_interval_ns'][0]==8


@pytest.mark.parametrize('fault',['float','backwards'])
def test_bad_schedule_rejected(fault):
    rows,_=inputs()
    rows[1]['rgb_left']['time_ns']=float(rows[1]['rgb_left']['time_ns']) if fault=='float' else rows[0]['rgb_left']['time_ns']-1
    with pytest.raises(ValueError):schedule(rows)
