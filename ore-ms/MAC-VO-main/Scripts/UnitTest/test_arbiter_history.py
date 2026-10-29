"""Symmetric causal confirmation, including cold-start cost and future exclusion."""
import copy
import pytest
from Scripts.UnitTest.test_pair_arbiter_events import fixture,resequence
from Scripts.Setup.arbiter_history import history_advisory


def inputs():
    reports,events=fixture();return reports['rgb']['input_rows'],events


def test_cold_start_abstains_despite_both_available():
    rows,events=inputs();d=history_advisory(rows,0,events)
    assert d['base_availability']=={'rgb':True,'thermal':True}
    assert d['action']=='ABSTAIN' and d['reason']=='cold_start'


def test_two_consecutive_pairs_confirm_original_rgb_preference():
    rows,events=inputs();d=history_advisory(rows,1,events)
    assert d['action']=='RGB' and all(d['confirmed_availability'].values())
    assert not d['reference_or_loss_used'] and not d['pose_gate_training_ready']


@pytest.mark.parametrize('camera',['rgb','thermal'])
@pytest.mark.parametrize('fault',['unsupported','missing_motion','stale_only','mismatch','missing_health'])
def test_previous_failure_blocks_each_expert_symmetrically(camera,fault):
    rows,events=inputs();e=events[camera]
    if fault=='unsupported':e[1]['original_need_interp']=True
    elif fault=='missing_motion':e.pop(2);resequence(e)
    elif fault=='stale_only':e[2]['first_write_for_submission']=False
    elif fault=='mismatch':e[0]['time0_ns']-=1;e[2]['time0_ns']-=1
    else:e.pop(1);resequence(e)
    d=history_advisory(rows,1,events)
    assert d['base_availability'][camera] and not d['confirmed_availability'][camera]
    assert d['action']==('THERMAL' if camera=='rgb' else 'RGB')


def test_no_unconfirmed_single_expert_fallback():
    rows,events=inputs()
    for c in events:events[c][1]['original_need_interp']=True
    events['thermal'][4]['original_need_interp']=True
    d=history_advisory(rows,1,events)
    assert d['base_availability']=={'rgb':True,'thermal':False} and d['action']=='ABSTAIN'


def test_current_failure_blocks_even_when_previous_was_supported():
    rows,events=inputs();events['rgb'][4]['original_need_interp']=True
    assert history_advisory(rows,1,events)['action']=='THERMAL'


def test_newer_pair_cannot_change_target_confirmation():
    rows,events=inputs();before=history_advisory(rows,0,events)
    for c in events:events[c][4]['original_need_interp']=True
    assert before==history_advisory(rows,0,events)


def test_reference_and_loss_metadata_do_not_affect_predictions():
    rows,events=inputs();before=history_advisory(rows,1,events)
    for e in events['rgb']:e.update(reference_loss=1e100,future_tracking_failure=True)
    assert before==history_advisory(rows,1,events)


@pytest.mark.parametrize('pair',[-1,2,1.0,True])
def test_bad_target_rejected(pair):
    rows,events=inputs()
    with pytest.raises(ValueError):history_advisory(rows,pair,events)
