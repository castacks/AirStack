"""Sampled encounter evidence stays distinct from causal/continuous claims."""
import copy
import json
import math

import pytest

from encounter_evidence import analyze_encounters, analyze_episode


def geometry():
    return dict(placement_id='saved-realized-geometry',spec=dict(width=4.),
                route=[[0.,0.],[10.,0.],[10.,10.]],
                obstacles=[dict(center=[4.,1.4,1.2],size=[.6,.6,2.4],yaw=0.,route_s=4.)])


def condition(noise=0.,delay=.5):
    return dict(rgb_noise=noise,delay=delay,noise_schedule=dict(start_s=0.,duration_s=0.,period_s=0.),
                delay_schedule=dict(start_s=0.,duration_s=0.,period_s=0.))


def trace(c,stop=20.):
    result=[]
    for n in range(round(stop*4)+1):
        now=n/4
        position=[min(now,10.),max(now-10.,0.),1.2]
        result.append(dict(sim_time_s=now,position_m=position))
    return result


def attach_frame(rows,c,capture,available,sequence=1,*,noise=None,delay=None,rmse=None,confirmed=True):
    row=next(r for r in rows if math.isclose(r['sim_time_s'],available))
    noise=c['rgb_noise'] if noise is None else noise
    delay=c['delay'] if delay is None else delay
    row.update(latest_input_stamp=capture,camera_sequence=sequence,
               latest_input_disturbance=dict(revision=1,noise_schedule=c['noise_schedule'],delay_schedule=c['delay_schedule'],
                    schedule_start_sim_s=0.,schedule_elapsed_s=capture,rgb_noise_stddev=noise,fixed_delay_s=delay),
               sensor_disturbance=dict(schedule_start_sim_s=0.,schedule_elapsed_s=available),
               latest_input_rgb_noise_rmse=(12. if noise else 0.) if rmse is None else rmse,
               worker=dict(sequence=sequence,sim_stamp=capture) if confirmed else None)
    return row


def test_capture_and_available_phase_differ_for_delayed_actual_input():
    c=condition();rows=trace(c)
    attach_frame(rows,c,3.,3.5)
    result=analyze_encounters(rows,geometry(),c)
    obstacle=result['obstacles'][0]
    assert result['status']=='complete'
    assert obstacle['at_capture']['approach']['delay']['active']==1
    assert obstacle['when_available']['passing']['delay']['active']==1
    assert result['overall']['worker_confirmed_frames']==1
    assert 'not exact model consumption' in result['alignment']['available']


def test_noise_schedule_overlaps_turn_approach_but_not_turn():
    c=condition(noise=16.,delay=0.)
    c['noise_schedule']=dict(start_s=7.,duration_s=2.,period_s=0.)
    rows=trace(c)
    attach_frame(rows,c,8.,8.,sequence=1,noise=16.)
    attach_frame(rows,c,10.,10.,sequence=2,noise=0.)
    result=analyze_encounters(rows,geometry(),c)
    assert len(result['turns'])==1
    turn=result['turns'][0]
    assert turn['direction']=='left' and turn['signed_heading_change_deg']==90.
    assert turn['at_capture']['approach']['noise']['active']==1
    assert turn['at_capture']['turn']['noise']['active']==0
    assert turn['at_capture']['turn']['noise']['inactive']==1
    assert 'duration' not in turn['at_capture']['turn']


def test_disabled_clean_factors_stay_inactive_despite_enabled_time_windows():
    c=condition(noise=0.,delay=0.);rows=trace(c)
    attach_frame(rows,c,4.,4.)
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['noise']==dict(active=0,inactive=1,unknown=0)
    assert result['overall']['delay']==dict(active=0,inactive=1,unknown=0)
    # A positive scalar with unproven pixels/latency remains unknown.
    bad=condition(noise=16.,delay=.5);rows=trace(bad)
    attach_frame(rows,bad,4.,4.,noise=16.,delay=.5,rmse=0.)
    result=analyze_encounters(rows,geometry(),bad)
    assert result['overall']['noise']['unknown']==result['overall']['delay']['unknown']==1


def test_duplicate_frame_is_counted_once_and_later_worker_confirmation_is_retained():
    c=condition(delay=0.);rows=trace(c)
    first=attach_frame(rows,c,3.,3.,confirmed=False)
    later=rows[13];later.update({k:copy.deepcopy(v) for k,v in first.items() if k not in ('sim_time_s','position_m')})
    later['worker']=dict(sequence=1,sim_stamp=3.)
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['frame_count']==1 and result['overall']['worker_confirmed_frames']==1


def test_later_worker_row_joins_earlier_frame_when_bridge_has_already_advanced():
    c=condition(noise=16.,delay=0.)
    c['noise_schedule']=dict(start_s=2.,duration_s=1.1,period_s=0.)
    rows=trace(c)
    attach_frame(rows,c,3.,3.,sequence=1,confirmed=False)
    later=attach_frame(rows,c,3.25,3.25,sequence=2,noise=0.,confirmed=False)
    later['worker']=dict(sequence=1,sim_stamp=3.)
    rows[14]['worker']=copy.deepcopy(later['worker'])
    result=analyze_encounters(rows,geometry(),c)
    overall=result['overall']
    assert overall['frame_count']==2 and overall['worker_confirmed_frames']==1
    assert overall['worker_confirmation_unknown_frames']==1
    assert overall['noise']==dict(active=1,inactive=1,unknown=0)
    assert overall['worker_confirmed_noise']==dict(active=1,inactive=0,unknown=0)
    assert result['worker_evidence']['unique_observed_input_identities']==1
    assert result['worker_evidence']['joined_input_identities']==1
    phase=result['obstacles'][0]['at_capture']['approach']
    assert phase['worker_confirmed_noise']['active']==1


def test_absent_worker_join_is_unknown_consumption_not_an_inactive_attack():
    c=condition(noise=16.,delay=0.);rows=trace(c)
    attach_frame(rows,c,3.,3.,confirmed=False)
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['noise']['active']==1
    assert result['overall']['worker_confirmation_unknown_frames']==1
    assert result['overall']['worker_confirmed_noise']==dict(active=0,inactive=0,unknown=0)


@pytest.mark.parametrize('worker', [dict(sequence=2,sim_stamp=3.),dict(sequence=1,sim_stamp=3.0000001)])
def test_worker_identity_join_is_exact_not_nearest_sequence_or_time(worker):
    c=condition(delay=0.);rows=trace(c)
    attach_frame(rows,c,3.,3.,confirmed=False);rows[13]['worker']=worker
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['worker_confirmed_frames']==0
    assert result['worker_evidence']['inputs_without_matching_available_frame']==1


def consumed_worker(frame):
    return dict(sequence=frame['camera_sequence'],sim_stamp=frame['latest_input_stamp'],
                consumed_input=dict(schema_version=1,sequence=frame['camera_sequence'],
                    sim_stamp=frame['latest_input_stamp'],disturbance_revision=1,
                    sensor_disturbance=copy.deepcopy(frame['latest_input_disturbance']),
                    rgb_noise_rmse=frame['latest_input_rgb_noise_rmse']))


def test_direct_consumed_input_metadata_confirms_earlier_matching_frame_once():
    c=condition(noise=16.,delay=.5);rows=trace(c)
    first=attach_frame(rows,c,3.,3.5,confirmed=False)
    later=attach_frame(rows,c,3.25,3.75,sequence=2,confirmed=False)
    later['worker']=consumed_worker(first)
    rows[16]['worker']=copy.deepcopy(later['worker'])
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['worker_confirmed_frames']==1
    assert result['overall']['worker_confirmed_noise']['active']==1
    assert result['overall']['worker_confirmed_delay']['active']==1
    assert result['worker_evidence']['confirmation_sources']=={'worker_consumed_input_metadata':1}


@pytest.mark.parametrize('change', [dict(schema_version=2),dict(sequence=2),dict(sim_stamp=3.25),
                                  dict(disturbance_revision=2),dict(rgb_noise_rmse=3.),
                                  dict(sensor_disturbance={})])
def test_conflicting_consumed_input_metadata_cannot_fall_back_to_legacy_join(change):
    c=condition(delay=0.);rows=trace(c)
    first=attach_frame(rows,c,3.,3.,confirmed=False)
    worker=consumed_worker(first);worker['consumed_input'].update(change)
    rows[13]['worker']=worker
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['worker_confirmed_frames']==0
    assert result['overall']['worker_confirmation_unknown_frames']==1


def test_direct_metadata_without_bridge_frame_has_no_invented_availability():
    c=condition(delay=0.);rows=trace(c)
    frame=attach_frame(rows,c,3.,3.,confirmed=False)
    worker=consumed_worker(frame)
    for key in list(frame):
        if key not in ('sim_time_s','position_m'):del frame[key]
    rows[13]['worker']=worker
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['frame_count']==0
    assert result['worker_evidence']['inputs_without_matching_available_frame']==1


def test_revision_ambiguity_requires_direct_metadata_and_never_double_counts_legacy_join():
    c=condition(delay=0.);rows=trace(c)
    first=attach_frame(rows,c,3.,3.,confirmed=False)
    second=attach_frame(rows,c,3.,3.25,confirmed=False)
    second['latest_input_disturbance']['revision']=2
    rows[14]['worker']=dict(sequence=1,sim_stamp=3.)
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['worker_confirmed_frames']==0
    rows[14]['worker']=consumed_worker(first)
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['worker_confirmed_frames']==1
    assert result['overall']['worker_confirmation_unknown_frames']==1


def test_confirmed_worker_does_not_turn_unknown_pixel_measurement_into_active():
    c=condition(noise=16.,delay=0.);rows=trace(c)
    attach_frame(rows,c,3.,3.,rmse=None)['latest_input_rgb_noise_rmse']=None
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['worker_confirmed_frames']==1
    assert result['overall']['worker_confirmed_noise']==dict(active=0,inactive=0,unknown=1)


def test_conflicting_duplicate_bridge_metadata_cannot_choose_favorable_exposure():
    c=condition(noise=16.,delay=0.);rows=trace(c)
    attach_frame(rows,c,3.,3.,confirmed=False)
    later=attach_frame(rows,c,3.,3.25,rmse=0.)
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['frame_count']==1
    assert result['overall']['noise']['unknown']==1
    assert result['overall']['worker_confirmation_unknown_frames']==1
    assert result['issues']['conflicting_duplicate_frame_metadata']==1


def test_conflicting_consumed_metadata_for_same_identity_is_not_cherry_picked():
    c=condition(noise=16.,delay=0.);rows=trace(c)
    frame=attach_frame(rows,c,3.,3.,confirmed=False)
    rows[13]['worker']=consumed_worker(frame)
    rows[14]['worker']=copy.deepcopy(rows[13]['worker'])
    rows[14]['worker']['consumed_input']['rgb_noise_rmse']=0.
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['worker_confirmed_frames']==0
    assert result['issues']['worker_join_conflicting_consumed_metadata']==1


@pytest.mark.parametrize('change', [dict(disturbance_revision=2),dict(rgb_noise_rmse=0.),
                                  dict(sensor_disturbance={}),dict(schema_version=2),
                                  dict(sequence=2),dict(rgb_noise_rmse=-1.)])
def test_legacy_join_cannot_override_conflicting_direct_metadata(change):
    c=condition(noise=16.,delay=0.);rows=trace(c)
    frame=attach_frame(rows,c,3.,3.,confirmed=True)
    rows[13]['worker']=consumed_worker(frame)
    rows[13]['worker']['consumed_input'].update(change)
    result=analyze_encounters(rows,geometry(),c)
    assert result['overall']['worker_confirmed_frames']==0
    assert result['overall']['worker_confirmation_unknown_frames']==1


def test_availability_time_extrema_do_not_follow_capture_iteration_order():
    c=condition(delay=0.);rows=trace(c)
    attach_frame(rows,c,1.,3.,sequence=1)
    attach_frame(rows,c,2.,2.5,sequence=2)
    result=analyze_encounters(rows,geometry(),c)
    available=result['obstacles'][0]['when_available']['approach']
    assert available['frame_count']==2
    assert available['first_sample_sim_s']==2.5
    assert available['last_sample_sim_s']==3.


def test_pose_gaps_are_not_interpolated_and_resets_are_rejected():
    c=condition(delay=0.)
    rows=[dict(sim_time_s=0.,position_m=[0.,0.,1.2]),dict(sim_time_s=5.,position_m=[5.,0.,1.2])]
    attach_frame(rows,c,3.,5.)
    result=analyze_encounters(rows,geometry(),c)
    assert result['gaps']['gaps_exceeding_pose_limit']==1
    assert result['gaps']['pose_unmatched_capture']==1
    assert result['obstacles'][0]['at_capture']=={}
    rows.append(dict(sim_time_s=1.,position_m=[1.,0.,1.2]))
    assert analyze_encounters(rows,geometry(),c)['status']=='unavailable_clock_regression'


def test_ambiguous_route_projection_does_not_invent_an_encounter():
    g=geometry();g['route']=[[0.,0.],[10.,0.],[0.,.1]]
    c=condition(delay=0.);rows=trace(c,stop=5.)
    attach_frame(rows,c,4.,4.)
    result=analyze_encounters(rows,g,c)
    assert result['issues']['capture_ambiguous_route_projection']==1
    assert result['obstacles'][0]['at_capture']=={}


def test_curved_turn_vertices_are_grouped_and_actual_box_rotation_is_used():
    g=geometry()
    g['route']=[[0.,0.],[4.,0.],[5.,.25],[5.75,1.],[6.,2.],[6.,6.]]
    c=condition(delay=0.);rows=trace(c,stop=4.)
    attach_frame(rows,c,3.,3.)
    result=analyze_encounters(rows,g,c)
    assert len(result['turns'])==1 and result['turns'][0]['vertex_count']>1
    assert result['turns'][0]['signed_heading_change_deg']==pytest.approx(90.)
    invalid=copy.deepcopy(g);invalid['obstacles'][0]['yaw']=float('nan')
    with pytest.raises(ValueError):analyze_encounters(rows,invalid,c)


def test_saved_episode_api_is_read_only_and_thresholds_are_explicit(tmp_path):
    c=condition(delay=0.);rows=trace(c);attach_frame(rows,c,3.,3.)
    documents={'scenario.json':dict(planner='test-model',condition=c),'diagnostic_trace.json':rows,'corridor_geometry.json':geometry()}
    for name,data in documents.items():(tmp_path/name).write_text(json.dumps(data))
    before={p.name:p.read_bytes() for p in tmp_path.iterdir()}
    result=analyze_episode(tmp_path,{'obstacle_approach_m':2.5})
    assert result['planner']=='test-model' and result['thresholds']['obstacle_approach_m']==2.5
    assert {p.name:p.read_bytes() for p in tmp_path.iterdir()}==before
    with pytest.raises(ValueError):analyze_episode(tmp_path,{'obstacle_approach_m':0.})
