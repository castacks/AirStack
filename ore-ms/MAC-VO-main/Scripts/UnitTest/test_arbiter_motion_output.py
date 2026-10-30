"""Analytic geometry, rejected contracts, and immutable controller prefixes."""
import copy
import math

import pytest

from Scripts.Setup.arbiter_motion_output import admit, replay
from Scripts.Setup.pair_arbiter_events import camera_record, index_events
from Scripts.Setup.shared_arbiter_replay import advisory, POSE_FRAME
from Scripts.UnitTest.test_pair_arbiter_events import fixture

I = [[1.,0.,0.,0.], [0.,1.,0.,0.], [0.,0.,1.,0.], [0.,0.,0.,1.]]


def inputs():
    reports, events = fixture(offset=0)
    candidates, frames = {}, {}
    for c in events:
        h, m, _ = index_events(events[c])
        first, last = [r[c+'_left']['time_ns'] for r in reports[c]['input_rows'][:2]]
        record = camera_record(reports[c], h, m, first, last)
        candidates[c] = {'record': record, 'received_ns': 20, 'clock_id': 'controller'}
        frames[c] = {'verified': True, 'source_frame': record['pose_frame'], 'target_frame': 'vehicle',
                     'evidence_ref': 'synthetic analytic fixture only', 'target_from_camera': copy.deepcopy(I)}
    args = dict(target_interval_ns=[first,last], target_frame='vehicle', deadline_ns=20,
                controller_clock_id='controller', frame_contracts=frames)
    return {'action': 'RGB'}, candidates, args


def test_exact_large_timestamp_deadline_equality_and_input_preservation():
    advice, candidates, args = inputs(); before = copy.deepcopy((advice,candidates,args))
    result = admit(advice,candidates,**args)
    assert result['action'] == 'RGB' and result['delta_target'][0][3] == 1
    assert result['sensor_interval_ns'][0] > 2**53
    assert result['source']['received_ns'] == args['deadline_ns']
    assert (advice,candidates,args) == before
    assert not result['reference_transport_used'] and not result['physical_accuracy_validated']


def test_rotation_of_translation_and_camera_to_target_direction():
    advice, candidates, args = inputs()
    args['frame_contracts']['rgb']['target_from_camera'] = [[0.,-1.,0.,0.],[1.,0.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
    result = admit(advice,candidates,**args)
    assert [r[3] for r in result['delta_target'][:3]] == pytest.approx([0.,1.,0.])


def test_rotating_camera_at_offset_induces_expected_lever_arm_translation():
    advice, candidates, args = inputs()
    args['frame_contracts']['rgb']['target_from_camera'][0][3] = 2.
    candidates['rgb']['record']['motion']['derived_camera_delta_xyzw'] = [0.,0.,0.,0.,0.,math.sqrt(.5),math.sqrt(.5)]
    result = admit(advice,candidates,**args)
    assert [r[3] for r in result['delta_target'][:3]] == pytest.approx([2.,-2.,0.])
    assert result['delta_target'][0][:3] == pytest.approx([0.,-1.,0.])


@pytest.mark.parametrize('fault,reason',[
    ('missing','missing_expert_record'), ('unsupported','expert_unavailable'),
    ('partial','expert_unavailable'), ('truthy_support','expert_unavailable'),
    ('missing_motion','missing_motion'), ('source_interval','interval_mismatch'),
    ('motion_interval','interval_mismatch'), ('late','late_result'),
    ('clock','controller_clock_mismatch'), ('unverified','unverified_frame_contract'),
    ('truthy_verified','unverified_frame_contract'), ('empty_evidence','unverified_frame_contract'),
    ('frame','frame_mismatch'), ('nan','invalid_geometry'), ('scaled','invalid_geometry'),
    ('reflection','invalid_geometry'), ('bottom_row','invalid_geometry'),
    ('quaternion','invalid_geometry'), ('nonfinite_pose','invalid_geometry')])
def test_rejected_selected_expert_never_falls_back_or_outputs_zero_motion(fault,reason):
    advice, c, args = inputs(); r = c['rgb']['record']; f = args['frame_contracts']['rgb']
    if fault == 'missing': c.pop('rgb')
    elif fault == 'unsupported': r['motion_available_camera_local'] = False
    elif fault == 'partial': r['health_status'] = 'PARTIAL_HEALTH'
    elif fault == 'truthy_support': r['motion_available_camera_local'] = 1
    elif fault == 'missing_motion': r['motion'] = None
    elif fault == 'source_interval': r['sensor_interval_ns'][0] += 1
    elif fault == 'motion_interval': r['motion']['time0_ns'] += 1
    elif fault == 'late': c['rgb']['received_ns'] += 1
    elif fault == 'clock': c['rgb']['clock_id'] = 'worker'
    elif fault == 'unverified': f['verified'] = False
    elif fault == 'truthy_verified': f['verified'] = 'true'
    elif fault == 'empty_evidence': f['evidence_ref'] = ''
    elif fault == 'frame': f['source_frame'] = 'other'
    elif fault == 'nan': f['target_from_camera'][0][0] = float('nan')
    elif fault == 'scaled': f['target_from_camera'][0][0] = 2.
    elif fault == 'reflection': f['target_from_camera'][0][0] = -1.
    elif fault == 'bottom_row': f['target_from_camera'][3][0] = 1.
    elif fault == 'quaternion': r['motion']['derived_camera_delta_xyzw'][-1] = 2.
    else: r['motion']['derived_camera_delta_xyzw'][0] = float('inf')
    result = admit(advice,c,**args)
    assert result['action'] == 'ABSTAIN' and result['delta_target'] is None
    assert result['reason'] == reason and result['advisory_action'] == 'RGB'


@pytest.mark.parametrize('field,value', [('target_interval_ns',[True,20]), ('target_interval_ns',[1.,20]),
    ('target_interval_ns',[20,20]), ('deadline_ns',20.), ('deadline_ns',True), ('controller_clock_id','')])
def test_invalid_declarations_raise(field,value):
    advice,c,args = inputs(); args[field] = value
    with pytest.raises(ValueError): admit(advice,c,**args)


def test_existing_abstention_is_retained_without_expert_access():
    advice,c,args = inputs(); advice['action'] = 'ABSTAIN'
    out = admit(advice,{},**args)
    assert out['delta_target'] is None and out['reason'] == 'advisory_abstain'


def journal(offset=8):
    reports,events = fixture(offset)
    rows = reports['rgb']['input_rows']; entries = []; prefix = {c:[] for c in events}
    for c in events:
        for e in events[c][:3]:
            # Worker clock is unrelated and intentionally far beyond receipt.
            e['observation_monotonic_ns'] += 10**15
            prefix[c].append(e)
            entries.append({'kind':'observer_event','camera':c,'event':e})
    entries.append({'kind':'decision',**advisory(rows,0,prefix)})
    for c in events:
        for e in events[c][3:]:
            e['observation_monotonic_ns'] += 10**15
            entries.append({'kind':'observer_event','camera':c,'event':e})
    for i,e in enumerate(entries): e.update(sequence=i,controller_monotonic_ns=20+i)
    return {'input_rows':rows},entries


def test_replay_uses_receipt_clock_and_no_future_revision():
    protocol,entries = journal(offset=0)
    _,_,args = inputs(); frames = args['frame_contracts']
    for f in frames.values(): f['source_frame'] = POSE_FRAME
    result = list(replay(protocol,entries,frames))
    assert result[0]['action'] == 'RGB'
    assert result[0]['source']['received_ns'] == 22
    entries[-1]['event']['derived_camera_delta_xyzw'] = [999]*7
    assert list(replay(protocol,entries,frames)) == result
    assert list(replay(protocol,entries[:7],frames)) == result


def test_historical_frame_uncertainty_is_not_promoted():
    protocol,entries = journal()
    out = list(replay(protocol,entries))[0]
    assert out['advisory_action'] == 'RGB' and out['action'] == 'ABSTAIN'
    assert out['reason'] == 'unverified_frame_contract' and out['delta_target'] is None


def test_thermal_selected_on_different_interval_cannot_be_relabelled_as_rgb_interval():
    protocol,entries = journal()
    # Reconstruct a real advisory choosing thermal by withholding RGB first write.
    entries = [e for e in entries[:7] if not (e['kind']=='observer_event' and e['camera']=='rgb' and e['event']['kind']=='motion_result_written')]
    prefix = {c:[e['event'] for e in entries if e['kind']=='observer_event' and e['camera']==c] for c in ('rgb','thermal')}
    entries[-1].update(advisory(protocol['input_rows'],0,prefix))
    for i,e in enumerate(entries): e['sequence'] = i
    out = list(replay(protocol,entries))[0]
    assert out['advisory_action'] == 'THERMAL' and out['reason'] == 'interval_mismatch'
    assert out['action'] == 'ABSTAIN' and out['delta_target'] is None


@pytest.mark.parametrize('fault',['clock','sequence','advice'])
def test_corrupt_timeline_is_rejected(fault):
    protocol,entries = journal()
    if fault == 'clock': entries[3]['controller_monotonic_ns'] = 0
    elif fault == 'sequence': entries[3]['sequence'] = 99
    else: entries[6]['action'] = 'THERMAL'
    with pytest.raises(ValueError): list(replay(protocol,entries))
