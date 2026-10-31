"""Exact chaining, temporal exclusion and independent geometric expectations."""
import copy
import math

import pytest

from Scripts.Setup.arbiter_motion_chain import compose
from Scripts.Setup.arbiter_motion_output import admit
from Scripts.UnitTest.test_arbiter_motion_output import inputs

START = 1761615290010015011


def edge(name, a, b, camera='rgb', received=30, pose=None):
    advice, candidates, args = inputs()
    advice['action'] = camera.upper()
    args['target_interval_ns'] = [START+a, START+b]
    args['deadline_ns'] = 25
    record = candidates[camera]['record']
    record['sensor_interval_ns'] = args['target_interval_ns'].copy()
    record['motion'].update(time0_ns=START+a, time1_ns=START+b)
    if pose is not None:
        record['motion']['derived_camera_delta_xyzw'] = pose
    output = admit(advice, candidates, **args)
    assert output['action'] == camera.upper()
    return {'id': name, 'received_ns': received, 'clock_id': 'controller', 'output': output,
            'sensor_clock_id': 'synthetic-acquisition', 'sensor_clock_evidence_ref': 'synthetic common acquisition clock'}


def query(entries, end=200, **kwargs):
    args = dict(target_interval_ns=[START, START+end], target_frame='vehicle',
                controller_clock_id='controller', sensor_clock_id='synthetic-acquisition', deadline_ns=40)
    args.update(kwargs)
    return compose(entries, **args)


def test_mixed_experts_noncommuting_chronological_composition():
    # First turn 90deg at position(1,0), then translate one metre in turned x.
    first = edge('rgb-turn', 0, 100, pose=[1.,0.,0.,0.,0.,math.sqrt(.5),math.sqrt(.5)])
    second = edge('thermal-step', 100, 200, 'thermal')
    entries = [second, first]  # arrival order need not be sensor-time order
    before = copy.deepcopy(entries)
    out = query(entries)
    assert out['action'] == 'COMPOSED'
    assert [r[3] for r in out['delta_target'][:3]] == pytest.approx([1.,1.,0.])
    assert out['delta_target'][0][:3] == pytest.approx([0.,-1.,0.])
    assert [s['entry_id'] for s in out['sources']] == ['rgb-turn', 'thermal-step']
    assert [s['admission_source']['camera'] for s in out['sources']] == ['rgb','thermal']
    assert not out['reference_transport_used'] and not out['physical_accuracy_validated']
    assert entries == before
    out['sources'][0]['admission_source']['frame_evidence_ref'] = 'mutated copy'
    assert entries == before


def test_three_independent_world_pose_increments_telescope_to_endpoint_motion():
    # Absolute world poses: (0,0,0deg), (1,0,90deg), (1,2,180deg), (-2,2,180deg).
    q = math.sqrt(.5)
    entries = [edge('a',0,100,pose=[1.,0.,0.,0.,0.,q,q]),
               edge('b',100,200,pose=[2.,0.,0.,0.,0.,q,q]),
               edge('c',200,300,pose=[3.,0.,0.,0.,0.,0.,1.])]
    out = query(entries,end=300)
    assert [r[3] for r in out['delta_target'][:3]] == pytest.approx([-2.,2.,0.])
    assert out['delta_target'][0][:3] == pytest.approx([-1.,0.,0.])
    assert out['delta_target'][1][:3] == pytest.approx([0.,-1.,0.])


@pytest.mark.parametrize('intervals', [[], [(0,100)], [(0,99),(100,200)], [(0,110),(100,200)], [(1,100),(100,200)], [(-1,100),(100,200)], [(0,100),(100,201)]])
def test_missing_partial_overlapping_and_outside_intervals_never_fill_gap(intervals):
    out = query([edge(str(i),a,b) for i,(a,b) in enumerate(intervals)])
    assert out['action'] == 'ABSTAIN' and out['delta_target'] is None
    assert out['reason'] == 'no_exact_chain' and out['sources'] == []


@pytest.mark.parametrize('extra', ['duplicate_interval','alternate_segmentation'])
def test_multiple_paths_abstain_even_if_the_matrices_happen_to_agree(extra):
    entries = [edge('a',0,100), edge('b',100,200)]
    if extra == 'duplicate_interval': entries.append(edge('c',0,100))
    else: entries.append(edge('c',0,200,pose=[2.,0.,0.,0.,0.,0.,1.]))
    out = query(entries)
    assert out['reason'] == 'ambiguous_exact_chains' and out['delta_target'] is None
    assert out['path_count_capped_at_two'] == 2


def test_future_payload_is_not_read_and_cannot_change_earlier_result():
    entries = [edge('a',0,100),edge('b',100,200)]
    expected = query(entries)
    future = {'clock_id':'controller','received_ns':41,'id':None,'output':None}
    assert query(entries+[future]) == expected
    future.update(id='a',output={'invalid':'also future'})
    assert query(entries+[future]) == expected
    assert query(entries+[edge('alternate',0,200,received=41)]) == expected
    assert query(entries+[edge('alternate',0,200,received=41)],deadline_ns=41)['reason'] == 'ambiguous_exact_chains'


def test_receipt_at_deadline_is_eligible():
    assert query([edge('a',0,200,received=40)])['action'] == 'COMPOSED'


@pytest.mark.parametrize('fault,reason', [('sensor_clock','sensor_clock_mismatch'), ('missing_clock_evidence','unverified_sensor_clock'),
    ('frame','target_frame_mismatch'),('clock','controller_clock_mismatch'),
    ('admission_clock','admission_clock_mismatch'),('abstain','admission_abstained'),
    ('reference','reference_transport_not_excluded'),('missing_reference_flag','reference_transport_not_excluded'),
    ('no_source','missing_source_evidence'),('wrong_source','source_interval_mismatch'),
    ('geometry','invalid_geometry'),('reflection','invalid_geometry')])
def test_incompatible_evidence_is_excluded_with_reason(fault,reason):
    e = edge('a',0,200); o = e['output']
    if fault == 'sensor_clock': e['sensor_clock_id'] = 'unmapped-thermal-clock'
    elif fault == 'missing_clock_evidence': e['sensor_clock_evidence_ref'] = ''
    elif fault == 'frame': o['target_frame'] = 'other_body'
    elif fault == 'clock': e['clock_id'] = 'worker'
    elif fault == 'admission_clock': o['controller_clock_id'] = 'worker'
    elif fault == 'abstain': o.update(action='ABSTAIN',delta_target=None)
    elif fault == 'reference': o['reference_transport_used'] = True
    elif fault == 'missing_reference_flag': del o['reference_transport_used']
    elif fault == 'no_source': o['source'] = None
    elif fault == 'wrong_source': o['source']['native_interval_ns'][0] += 1
    elif fault == 'geometry': o['delta_target'][0][0] = float('nan')
    else: o['delta_target'][0][0] = -1.
    out = query([e])
    assert out['action'] == 'ABSTAIN' and out['delta_target'] is None
    assert out['excluded_received_entries'] == {reason:1}


@pytest.mark.parametrize('fault',['duplicate_id','float_receipt','future_admission','future_source','bool_sensor','negative_receipt','false_abstain'])
def test_malformed_received_contracts_raise(fault):
    e = edge('a',0,200); entries=[e]
    if fault == 'duplicate_id': entries.append(copy.deepcopy(e))
    elif fault == 'float_receipt': e['received_ns'] = 30.
    elif fault == 'future_admission': e['output']['deadline_ns'] = 31
    elif fault == 'future_source': e['output']['source']['received_ns'] = 26
    elif fault == 'bool_sensor': e['output']['sensor_interval_ns'][0] = True
    elif fault == 'negative_receipt': e['received_ns'] = -1
    else: e['output']['action'] = 'ABSTAIN'
    with pytest.raises(ValueError): query(entries)


def test_off_path_branches_do_not_create_false_ambiguity():
    out = query([edge('a',0,100),edge('b',100,200),edge('dead-end',0,50)])
    assert out['action'] == 'COMPOSED' and len(out['sources']) == 2


def test_deadline_never_subtracts_epoch_sensor_times():
    out = query([edge('a',0,200)])
    assert out['sensor_interval_ns'] == [START,START+200]
    assert out['deadline_ns'] == 40 and START > 2**53


def test_exponential_path_population_is_capped_without_enumeration():
    entries = [edge(f'{i}-{j}',i,i+1) for i in range(50) for j in range(2)]
    out = query(entries,end=50)
    assert out['path_count_capped_at_two'] == 2 and out['delta_target'] is None
