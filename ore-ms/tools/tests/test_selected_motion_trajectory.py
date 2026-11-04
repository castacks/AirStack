from copy import deepcopy
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from selected_motion_trajectory import publish, rigid


def transform(angle=0., translation=(0., 0., 0.)):
    c, s = np.cos(angle), np.sin(angle)
    return [[c, -s, 0., translation[0]], [s, c, 0., translation[1]],
            [0., 0., 1., translation[2]], [0., 0., 0., 1.]]


def contract():
    times = [10**18+i*100_000_000 for i in range(5)]
    frames = {c: {'source_frame': c+'_left', 'target_frame': 'common',
                  'target_from_camera': transform(0, (0 if c == 'rgb' else .3, 0, 0)),
                  'frame_evidence_ref': 'analytic_fixture'} for c in ['rgb', 'thermal']}
    slots = []
    for i in range(4):
        slots.append({'target_interval_ns': times[i:i+2], 'deadline_ns': 100+i*20,
                      'action': 'RGB', 'selected': {'controller_clock_id': 'controller',
                      'received_ns': 99+i*20, 'payload': {
                      'id': 'motion-'+str(i), 'camera': 'rgb', 'available': True,
                      'reference_transport_used': False, 'native_interval_ns': times[i:i+2],
                      'sensor_clock_id': 'sensor', 'source_frame': 'rgb_left',
                      'native_segment_id': 0, 'reset_before': False,
                      'delta_camera': transform(.2*(i+1), (.1, .2, .3))}}})
    return {'time_ns': times, 'source_ids': [str(i) for i in range(5)],
            'target_frame': 'common', 'sensor_clock_id': 'sensor',
            'sensor_clock_evidence_ref': 'analytic_shared_clock',
            'controller_clock_id': 'controller', 'max_source_gap_ns': 150_000_000,
            'initialization': {'source_time_ns': times[0], 'deadline_ns': 90,
                               'received_ns': 89, 'controller_clock_id': 'controller',
                               'target_frame': 'common', 'evidence_ref': 'explicit_root'},
            'frame_contracts': frames, 'slots': slots, 'terminal_success': True}


def test_noncommuting_motion_is_right_composed():
    c = contract();out = publish(c)
    expected = np.eye(4)
    for index, row in enumerate(c['slots']):
        expected = expected@np.array(row['selected']['payload']['delta_camera'])
        np.testing.assert_allclose(out['poses'][index+1], expected, atol=1e-13)
    reverse = np.eye(4)
    for row in c['slots']:reverse=np.array(row['selected']['payload']['delta_camera'])@reverse
    assert not np.allclose(expected, reverse)
    assert out['segments'] == [0]*5


def test_camera_switch_does_not_use_native_world_origins():
    c = contract();expected = publish(c)
    e = np.array(c['frame_contracts']['thermal']['target_from_camera'])
    for row in c['slots'][1::2]:
        d = row['selected']['payload'];row['action']='THERMAL'
        d.update(camera='thermal',source_frame='thermal_left',native_segment_id=17,
                 delta_camera=(np.linalg.inv(e)@np.array(d['delta_camera'])@e).tolist())
    got = publish(c)
    np.testing.assert_allclose(got['poses'], expected['poses'], atol=1e-13)
    assert got['segments'] == [0]*5


def test_relative_motion_cancels_arbitrary_native_world_gauge():
    c=contract();world=np.eye(4);poses=[world.copy()]
    for row in c['slots']:
        world=world@np.asarray(row['selected']['payload']['delta_camera']);poses.append(world.copy())
    different_world=np.asarray(transform(.9,(10,-4,3)))
    for i,row in enumerate(c['slots']):
        row['selected']['payload']['delta_camera']=np.linalg.solve(different_world@poses[i],different_world@poses[i+1]).tolist()
    np.testing.assert_allclose(publish(c)['poses'],publish(contract())['poses'],atol=1e-13)


def test_lever_arm_conjugation_includes_translation():
    c=contract();row=c['slots'][0];row['action']='THERMAL'
    row['selected']['payload'].update(camera='thermal',source_frame='thermal_left',delta_camera=transform(np.pi/2))
    got=publish(c)
    np.testing.assert_allclose(np.array(got['poses'][1])[:3,3], [.3,-.3,0], atol=1e-13)


def test_hole_late_motion_and_reset_never_fill_prior_pose():
    c=contract();c['slots'][0]['action']='ABSTAIN'
    c['slots'][2]['selected']['received_ns']=c['slots'][2]['deadline_ns']+1
    got=publish(c)
    assert got['segments']==[0,None,1,None,2]
    assert got['poses'][1] is None and got['poses'][3] is None
    assert got['segment_roots'][1]['root_time_ns']==c['time_ns'][1]
    assert not got['segment_roots'][1]['root_published']
    np.testing.assert_allclose(got['poses'][2], c['slots'][1]['selected']['payload']['delta_camera'],atol=1e-13)
    c=contract();c['slots'][1]['selected']['payload']['reset_before']=True
    assert publish(c)['segments']==[0,0,1,1,1]


def test_no_initialization_or_no_data_creates_no_fake_anchor():
    c=contract();c['initialization']=None
    for row in c['slots']:row['action']='ABSTAIN';row['selected']='never inspect'
    out=publish(c)
    assert out['poses']==[None]*5 and out['segment_count']==0


def test_future_payload_is_unobserved_and_prior_results_immutable():
    c=contract();c['slots'][2]['selected']['received_ns']=9999
    first=publish(c)
    c['slots'][2]['selected']['payload']={'bad':'nonrigid future payload'}
    assert publish(c)==first
    c['slots'][3]['selected']['payload']['delta_camera']=transform(1.2,(3,4,5))
    changed=publish(c)
    assert changed['poses'][:4]==first['poses'][:4]


@pytest.mark.parametrize('field,value,reason', [
    ('native_interval_ns',[10**18+1,10**18+100_000_000],'native_interval_mismatch'),
    ('sensor_clock_id','other','sensor_clock_mismatch'),
    ('source_frame','other','source_frame_mismatch'),
    ('camera','thermal','selected_camera_mismatch'),
    ('reference_transport_used',True,'reference_transport_not_excluded'),
    ('available',False,'selected_motion_unavailable')])
def test_selected_rejection_never_falls_back(field,value,reason):
    c=contract();c['slots'][0]['selected']['payload'][field]=value
    got=publish(c)
    assert got['poses'][1] is None and got['decisions'][0]['reason']==reason
    assert got['decisions'][0]['action']=='RGB'


@pytest.mark.parametrize('where', ['source','target','receipt','deadline'])
def test_bool_float_clock_types_rejected(where):
    c=contract()
    if where=='source':c['time_ns'][0]=float(c['time_ns'][0])
    elif where=='target':c['slots'][0]['target_interval_ns'][0]=float(c['time_ns'][0])
    elif where=='receipt':c['slots'][0]['selected']['received_ns']=True
    else:c['slots'][0]['deadline_ns']=100.
    with pytest.raises(ValueError):publish(c)


def test_duplicate_motion_id_and_backwards_decision_rejected():
    c=contract();c['slots'][1]['selected']['payload']['id']='motion-0'
    with pytest.raises(ValueError,match='duplicate'):publish(c)
    c=contract();c['slots'][1]['deadline_ns']=1
    with pytest.raises(ValueError,match='backwards'):publish(c)


def test_native_epoch_change_and_source_gap_break_segments():
    c=contract();c['slots'][2]['selected']['payload']['native_segment_id']=1
    c['slots'][3]['selected']['payload']['native_segment_id']=1
    assert publish(c)['segments']==[0,0,0,1,1]
    c=contract();c['max_source_gap_ns']=99_999_999
    assert publish(c)['segments']==[0,1,2,3,4]


def test_gross_invalid_geometry_never_repaired():
    matrix=transform(0)
    matrix[0][0]=2
    with pytest.raises(ValueError,match='nonrigid'):rigid(matrix)
    c=contract();c['slots'][0]['selected']['payload']['delta_camera']=matrix
    assert publish(c)['decisions'][0]['reason']=='invalid_selected_geometry'


def test_provided_evidence_does_not_promote_physical_truth():
    c=contract();c['physical_frame_or_clock_verified']=True
    out=publish(c)
    assert out['physical_frame_or_clock_verified'] is False
    assert out['reference_used'] is False and out['time_transport_used'] is False


def test_wrong_controller_clock_does_not_open_payload():
    c=contract();c['slots'][0]['selected'].update(controller_clock_id='other',payload='unobserved')
    assert publish(c)['decisions'][0]['reason']=='controller_clock_mismatch'


def test_boolean_geometry_is_not_a_measurement():
    matrix=transform();matrix[0][0]=True
    with pytest.raises(ValueError,match='boolean'):rigid(matrix)
