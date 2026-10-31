"""Causal-prefix integration: immutable results, independent clocks and late data."""
import builtins
import copy
import numpy as np
import pytest

from Scripts.Setup.prefix_interval_model import predict_prefix
from Scripts.Setup.shared_arbiter_replay import advisory
from Scripts.UnitTest.test_arbiter_motion_output import journal

I=np.eye(4).tolist()


def run(protocol,entries,frames=None):
    return list(predict_prefix(protocol['input_rows'],entries,frames or {'rgb':I,'thermal':I}))


def rebuild(protocol,entries):
    prefix={c:[] for c in ('rgb','thermal')}
    for i,r in enumerate(entries):
        r.update(sequence=i,controller_monotonic_ns=20+i)
        if r['kind']=='observer_event':prefix[r['camera']].append(r['event'])
        elif r['kind']=='decision':r.update(advisory(protocol['input_rows'],r['pair_ordinal'],prefix))


def test_fixed_policies_and_cold_start_remain_separate_from_verified_admission():
    protocol,entries=journal();out=run(protocol,entries)[0]
    assert out['model_actions']=={'rgb_only':'RGB','thermal_only':'THERMAL','raw_advice':'RGB','history':'ABSTAIN'}
    assert out['history_reason']=='cold_start'
    assert all(e['valid_model_projection'] for e in out['experts'].values())
    assert not out['admission_ready'] and not out['physical_frame_verified'] and not out['acquisition_clock_verified']


def test_static_frame_rotation_has_analytic_expected_translation():
    protocol,entries=journal()
    e=[[0.,-1.,0.,0.],[1.,0.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
    out=run(protocol,entries,{'rgb':e,'thermal':I})[0]
    assert [row[3] for row in out['experts']['rgb']['delta_model'][:3]]==pytest.approx([0,1,0])


def test_future_shutdown_and_reference_metadata_cannot_change_emitted_prefix():
    protocol,entries=journal();before=copy.deepcopy(entries);expected=run(protocol,entries)
    assert entries==before
    assert run(protocol,entries[:7])==expected
    for r in entries[7:]:r['event'].update(derived_camera_delta_xyzw=[999]*7,stage='termination')
    protocol.update(reference_poses='forbidden',reference_loss=1e100)
    assert run(protocol,entries)==expected


def test_predictor_performs_no_file_reads(monkeypatch):
    protocol,entries=journal()
    def forbidden(*a,**k):raise AssertionError('predictor file access')
    monkeypatch.setattr(builtins,'open',forbidden)
    assert run(protocol,entries)[0]['model_actions']['raw_advice']=='RGB'


def test_controller_receipt_not_worker_clock_and_deadline_equality():
    protocol,entries=journal();entries[6]['controller_monotonic_ns']=entries[5]['controller_monotonic_ns']
    out=run(protocol,entries[:7])[0];thermal=out['experts']['thermal']
    assert thermal['source']['received_controller_ns']==out['controller_deadline_ns']==25
    assert out['experts']['rgb']['source']['received_controller_ns']==22


def test_results_received_after_decision_cannot_backfill_projections():
    protocol,entries=journal();prefix=[r for r in entries[:6] if r['event']['kind']!='motion_result_written']
    prefix += [entries[6]]
    late=[r for r in entries[:6] if r['event']['kind']=='motion_result_written']
    all_entries=prefix+late;rebuild(protocol,all_entries)
    out=run(protocol,all_entries)[0]
    assert out['model_actions']['raw_advice']=='ABSTAIN'
    assert all(e['delta_model'] is None for e in out['experts'].values())
    assert run(protocol,all_entries[:5])==[out]


def test_late_health_cannot_be_replaced_by_motion_receipt_alone():
    protocol,entries=journal();prefix=[];late=[]
    for c in ('rgb','thermal'):
        events=[r['event'] for r in entries[:6] if r['camera']==c]
        ordered=[events[0],events[2],events[1]]
        for i,event in enumerate(ordered):event.update(sequence=i,observation_monotonic_ns=10**15+i)
        prefix += [{'kind':'observer_event','camera':c,'event':event} for event in ordered[:2]]
        late.append({'kind':'observer_event','camera':c,'event':ordered[2]})
    all_entries=prefix+[entries[6]]+late;rebuild(protocol,all_entries)
    out=run(protocol,all_entries)[0]
    assert out['model_actions']['raw_advice']=='ABSTAIN'
    assert all(not e['available_by_prefix'] for e in out['experts'].values())


@pytest.mark.parametrize('fault',['unsupported','interval_mismatch','missing_health'])
def test_source_eligibility_remains_binding(fault):
    protocol,entries=journal();e=[r['event'] for r in entries[:6] if r['camera']=='rgb']
    if fault=='unsupported':e[1]['original_need_interp']=True
    elif fault=='interval_mismatch':e[0]['time0_ns']-=1;e[2]['time0_ns']-=1
    else:e[1]['retained_count']=None
    rebuild(protocol,entries)
    out=run(protocol,entries)[0]
    assert not out['experts']['rgb']['valid_model_projection'] and out['model_actions']['raw_advice']=='THERMAL'


def test_terminal_motion_is_not_promoted_even_in_malformed_early_prefix():
    protocol,entries=journal()
    for r in entries[:6]:
        if r['event']['kind']=='motion_result_written':r['event']['stage']='termination'
    out=run(protocol,entries)[0]
    assert out['advice_actions']['raw_advice']=='RGB' and out['model_actions']['raw_advice']=='ABSTAIN'
    assert all(e['reason']=='non_processing_stage' for e in out['experts'].values())


@pytest.mark.parametrize('fault',['controller_clock','controller_sequence','event_sequence','modality','action','pair'])
def test_invalid_source_prefix_rejected(fault):
    protocol,entries=journal()
    if fault=='controller_clock':entries[3]['controller_monotonic_ns']=0
    elif fault=='controller_sequence':entries[2]['sequence']=99
    elif fault=='event_sequence':entries[1]['event']['sequence']=99
    elif fault=='modality':entries[1]['event']['modality']='thermal'
    elif fault=='action':entries[6]['action']='THERMAL'
    else:entries[6]['pair_ordinal']=1
    with pytest.raises(ValueError):run(protocol,entries)
