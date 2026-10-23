"""Expanded geometry and sensor windows must be physical, replayable evidence."""
import ast
import copy
import importlib.util
import json
import math
from pathlib import Path
import random
import threading
from types import SimpleNamespace

import pytest

from campaign import clean_twin
from conditions import validate_schedule, sensor_parameters
from corridor_space import (VERSION, reference_action, validate_action, spec_for,
                            action_to_episode, mission_for, action_key, sample)
from corridors import geometry, patch_corners, _segment_box_distance
from episode import resolved
from audit_corridor_campaign import audit_scheduled_sensors
import experiment_protocol as experiments
from diagnostic_evaluation import workflow_evidence

BRIDGE=Path(__file__).parents[2]/'robot/ros_ws/src/local/planners/mononav_bridge/mononav_bridge'
spec=importlib.util.spec_from_file_location('scheduled_disturbances_test',BRIDGE/'disturbances.py')
disturbances=importlib.util.module_from_spec(spec);spec.loader.exec_module(disturbances)


def proposal():
    a=reference_action(VERSION)
    a['obstacles']=[dict(route_s=6.,lateral_m=-1.4,width_m=.65,depth_m=.8,height_m=2.5,yaw_deg=8.)]
    a.update(rgb_noise_stddev=24.,delay_s=.3)
    a['noise_schedule']=dict(start_s=2.,duration_s=8.,period_s=16.)
    a['delay_schedule']=dict(start_s=5.,duration_s=12.,period_s=0.)
    return a


def test_rotated_dimensions_change_real_geometry_and_preserve_pairing():
    a=validate_action(proposal(),'kim');g=geometry(spec_for(a))
    assert g['spec']['layout_version']==5
    block=g['obstacles'][0]
    assert block['center']==pytest.approx([2.,-1.4,1.25])
    assert block['size']==[.65,.8,2.5] and block['yaw']==pytest.approx(math.radians(8))
    b=copy.deepcopy(a);b['obstacles'][0]['yaw_deg']=0.
    assert geometry(spec_for(b))['placement_id']!=g['placement_id']
    c=resolved(action_to_episode(a,'kim',42,'scheduled',mission_for('kim')))
    clean=clean_twin(c)
    assert clean['condition']['corridor']==c['condition']['corridor']
    assert clean['condition']['noise_schedule']==c['condition']['noise_schedule']
    assert clean['condition']['rgb_noise']==clean['condition']['delay']==0.
    assert json.loads(sensor_parameters(c['condition'])['rgb_noise_schedule_json'])==a['noise_schedule']


def test_geometry_checks_continuous_route_wall_intersection_and_patch_fit():
    a=proposal()
    for changes in ({'lateral_m':.1},{'width_m':1.4,'depth_m':1.5,'yaw_deg':60},
                    {'height_m':float('nan')},{'yaw_deg':True}):
        b=copy.deepcopy(a);b['obstacles'][0].update(changes)
        with pytest.raises(ValueError):validate_action(b,'kim')
    a.update(patch_enabled=True,patch_mount=dict(obstacle_index=0,face='front',height_m=1.2))
    validate_action(a,'kim')
    corners=patch_corners(geometry(spec_for(a)),.6,a['patch_mount'])
    assert math.dist(corners[0],corners[1])==pytest.approx(.6)
    with pytest.raises(ValueError,match='fit'):validate_action(dict(a,patch_size_m=.9),'kim')
    for target in ('mononav','pulp_dronet'):
        with pytest.raises(ValueError):validate_action(a,target)
    box=dict(center=[0.,0.,1.],size=[1.,1.,2.],yaw=0.)
    assert _segment_box_distance([-2.,0.],[2.,0.],box)==0
    assert _segment_box_distance([-2.,1.],[2.,1.],box)==pytest.approx(.5)
    assert _segment_box_distance([-2.,1.],[-1.,1.],box)==pytest.approx(math.sqrt(.5))


def test_legacy_action_is_not_silently_upgraded():
    a=reference_action();a['obstacles']=[dict(route_s=6.,side=-1)]
    assert validate_action(a)==a and 'action_space_version' not in a
    assert spec_for(a)['layout_version']==4
    with pytest.raises(ValueError):validate_action(dict(a,noise_schedule=proposal()['noise_schedule']))
    expanded=proposal();expanded['rgb_noise_stddev']=0.
    b=copy.deepcopy(expanded);b['noise_schedule']['start_s']=80.
    assert action_key(expanded)==action_key(b) # disabled schedules are no intervention
    b['delay_schedule']['start_s']=80.
    assert action_key(expanded)!=action_key(b)


@pytest.mark.parametrize('bad',[
    {},{'start_s':0.,'duration_s':1.,'period_s':1.},
    {'start_s':0.,'duration_s':0.,'period_s':10.},
    {'start_s':-1.,'duration_s':0.,'period_s':0.},
    {'start_s':True,'duration_s':0.,'period_s':0.},
    {'start_s':float('inf'),'duration_s':0.,'period_s':0.}])
def test_runtime_boundary_rejects_same_invalid_schedules(bad):
    with pytest.raises(ValueError):validate_schedule(bad)
    if bad:
        with pytest.raises(ValueError):disturbances.parse_schedule(json.dumps(bad))
    else:
        assert disturbances.parse_schedule('{}') is None # deliberate legacy ROS sentinel


def test_pulses_use_sim_clock_do_not_advance_while_paused_and_reset_disarms():
    schedule=dict(start_s=2.,duration_s=3.,period_s=8.)
    clock=disturbances.SensorScheduleClock()
    assert not disturbances.schedule_active(schedule,clock.elapsed(100))
    clock.arm(100)
    for now,wanted in [(101.9,False),(102,True),(104.999,True),(105,False),(110,True),(113,False)]:
        assert disturbances.schedule_active(schedule,clock.elapsed(now)) is wanted
        assert disturbances.schedule_active(schedule,clock.elapsed(now)) is wanted
    clock.arm(114);assert clock.start_s==100 # repeated commands never move anchor
    assert clock.elapsed(0) is None
    assert not disturbances.schedule_active(schedule,clock.elapsed(2))
    clock.arm(2);assert clock.elapsed(4)==2
    clock.reset();assert clock.elapsed(80) is None
    assert disturbances.schedule_active(None,None) # legacy constant mode unchanged


def test_variable_delay_releases_newest_due_never_an_older_late_frame():
    b=disturbances.DelayedSampleBuffer(.5)
    b.push('old_delayed',0.,.5)
    b.push('new_undelayed',.1,0.)
    assert b.latest(.1)=='new_undelayed'
    assert b.latest(.5)=='new_undelayed' and b.pending_count==0
    b.push('next_delayed',.6,.5)
    assert b.latest(.9)=='new_undelayed'
    assert b.latest(1.1)=='next_delayed'
    with pytest.raises(ValueError):b.push('bad',2.,float('nan'))


def fake_trace(condition,include_active=True):
    configured=dict(seed=condition['seed'],rgb_noise_stddev=condition['rgb_noise'],
                    fixed_delay_s=condition['delay'],depth_noise_stddev_m=condition['depth_noise'])
    def observation(elapsed):
        noise=disturbances.schedule_active(condition['noise_schedule'],elapsed)
        delay=disturbances.schedule_active(condition['delay_schedule'],elapsed)
        return dict(configured=configured,noise_schedule=condition['noise_schedule'],delay_schedule=condition['delay_schedule'],
                    schedule_elapsed_s=elapsed,revision=1,rgb_noise_stddev=condition['rgb_noise'] if noise else 0.,
                    fixed_delay_s=condition['delay'] if delay else 0.)
    initial=dict(stamp=90.,sensor_disturbance=observation(None),rgb_noise_rmse=0.)
    rows=[]
    for elapsed in ([0.,3.,6.,11.,18.] if include_active else [0.,1.]):
        d=observation(elapsed)
        rows.append(dict(sim_time_s=100+elapsed+d['fixed_delay_s'],latest_input_stamp=100+elapsed,
                         latest_input_disturbance=d,latest_input_rgb_noise_rmse=20. if d['rgb_noise_stddev'] else 0.))
    return initial,rows


def test_audit_requires_measured_pixels_latency_and_reports_absent_exposure(tmp_path):
    c=resolved(action_to_episode(proposal(),'kim',42,'scheduled',mission_for('kim')))['condition']
    initial,rows=fake_trace(c)
    path=tmp_path/'diagnostic_trace.json';path.write_text(json.dumps(rows))
    result=audit_scheduled_sensors(tmp_path,c,initial)
    assert result['exposure_verified'] and result['factors']['noise']['active_frames']==3
    rows[2]['latest_input_rgb_noise_rmse']=0.;path.write_text(json.dumps(rows))
    with pytest.raises(AssertionError,match='pixels'):audit_scheduled_sensors(tmp_path,c,initial)
    initial,rows=fake_trace(c,False);path.write_text(json.dumps(rows))
    result=audit_scheduled_sensors(tmp_path,c,initial)
    assert not result['exposure_verified']
    assert result['factors']['noise']['status']=='not_exposed_before_termination'
    clean=dict(c,rgb_noise=0.,delay=0.)
    initial,rows=fake_trace(clean);path.write_text(json.dumps(rows))
    result=audit_scheduled_sensors(tmp_path,clean,initial)
    assert result['exposure_verified'] and all(v['status']=='disabled' for v in result['factors'].values())
    assert all(r['latest_input_rgb_noise_rmse']==0. and r['latest_input_disturbance']['fixed_delay_s']==0. for r in rows)


def test_runtime_parameter_update_is_atomic_and_clears_clock_and_samples():
    # Execute the actual ROS callback body against minimal boundary objects;
    # no simulator or ROS installation is required for parameter semantics.
    tree=ast.parse((BRIDGE/'bridge_node.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='VisionPlannerBridge')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_update_disturbance')
    module=ast.Module(body=[method],type_ignores=[])
    ns=dict(math=math,parse_schedule=disturbances.parse_schedule,
            SetParametersResult=lambda **kw:SimpleNamespace(**kw),DelayedSampleBuffer=disturbances.DelayedSampleBuffer)
    exec(compile(module,str(BRIDGE/'bridge_node.py'),'exec'),ns)
    node=SimpleNamespace(_lock=threading.Lock(),_schedule_clock=disturbances.SensorScheduleClock(),
                         _fixed_sensor_delay_s=0.,_rgb_noise_stddev=0.,_disturbance_revision=0,
                         _sample='old',_depth_sample='old',_sequence=123,_depth_sequence=123,_last_recorded_sample=(123,0))
    node._schedule_clock.arm(100)
    p=lambda name,value:SimpleNamespace(name=name,value=value)
    update=ns['_update_disturbance']
    r=update(node,[p('rgb_noise_stddev',24.),p('rgb_noise_schedule_json','{"start_s":0}')])
    assert not r.successful and node._rgb_noise_stddev==0. and node._sample=='old'
    r=update(node,[p('rgb_noise_stddev',24.),p('rgb_noise_schedule_json',json.dumps(proposal()['noise_schedule']))])
    assert r.successful and node._rgb_noise_stddev==24. and node._sample is None
    assert node._schedule_clock.start_s is None and node._sequence==node._depth_sequence==0
    r=update(node,[p('rgb_noise_schedule_json','{}'),p('sensor_delay_schedule_json','{}')])
    assert r.successful and node._noise_schedule is node._delay_schedule is None


def test_delay_audit_uses_bridge_clock_and_does_not_mislabel_async_zero_delay(tmp_path):
    c=resolved(action_to_episode(proposal(),'kim',42,'scheduled',mission_for('kim')))['condition']
    initial,rows=fake_trace(c)
    # Actual live regression: a later HTTP frame lies .06s ahead of the earlier
    # scene snapshot, while its own bridge clock shows a valid +.03s age.
    row=rows[-1]
    row['sim_time_s']=row['latest_input_stamp']-.06
    row['sensor_disturbance']=dict(schedule_start_sim_s=100.,schedule_elapsed_s=row['latest_input_stamp']-100.+.03)
    assert row['latest_input_disturbance']['fixed_delay_s']==0.
    # For an active frame, even a stale scene snapshot is acceptable only when
    # the matching bridge clock proves the configured .3s latency elapsed.
    active=rows[2];active['sim_time_s']=active['latest_input_stamp']-.06
    active['sensor_disturbance']=dict(schedule_start_sim_s=100.,schedule_elapsed_s=active['latest_input_stamp']-100.+.3)
    path=tmp_path/'diagnostic_trace.json';path.write_text(json.dumps(rows))
    result=audit_scheduled_sensors(tmp_path,c,initial)
    assert result['exposure_verified'] and result['timing_evidence']['asynchronous_scene_frame_samples']==2
    assert result['timing_evidence']['measurement_sources']['bridge_sim_clock']==2
    # No aligned clock and no lower bound on actual delay: do not count this
    # requested active frame as exposure; preserve explicit unknown evidence.
    active.pop('sensor_disturbance');rows=[active];path.write_text(json.dumps(rows))
    result=audit_scheduled_sensors(tmp_path,c,initial)
    assert result['factors']['delay']['active_frames']==0
    assert result['factors']['delay']['unknown_frames']==1
    assert result['factors']['delay']['status']=='exposure_unknown' and not result['exposure_verified']


def test_random_baseline_can_use_same_new_space_without_starting_comparison():
    actions=[sample(random.Random(i),'kim') for i in range(30)]
    assert all(a['action_space_version']==VERSION for a in actions)
    assert any(a['noise_schedule']['period_s'] for a in actions)
    assert any(a['patch_enabled'] for a in actions)
    assert len({geometry(spec_for(a),a['layout_seed'])['placement_id'] for a in actions})==30


def history_row(a,number=1,exposure=True,outcome='collision',experiment=None):
    from corridor_space import experiment as purpose
    attacked=dict(outcome=outcome,metrics={})
    if exposure is not None:attacked['attack_exposure']={'exposure_verified':exposure}
    return dict(decision=dict(round=number,action=copy.deepcopy(a),experiment=experiment or purpose()),
                pairs=[dict(clean=dict(outcome='route_completed',metrics={}),perturbed=attacked)])


@pytest.mark.parametrize('exposure',[None,False])
def test_unmeasured_v2_trials_cannot_anchor_factor_or_boundary_claims(exposure):
    from corridor_space import experiment
    combined=proposal();noise=copy.deepcopy(combined);noise['delay_s']=0.
    source=history_row(combined,exposure=exposure)
    with pytest.raises(ValueError,match='exposure'):
        experiments.validate(experiment('isolate',1,'rgb_noise'),noise,[source],action_key)
    assert experiments.validate(experiment('repeat',1),combined,[source],action_key)
    source=history_row(noise,exposure=exposure);probe=copy.deepcopy(noise);probe['rgb_noise_stddev']=12.
    with pytest.raises(ValueError,match='exposure'):
        experiments.validate(experiment('threshold',1,'rgb_noise'),probe,[source],action_key)
    repeated=history_row(noise,2,exposure=exposure,experiment=experiment('repeat',1))
    result=workflow_evidence([source,repeated])
    assert result['reproduced_condition_count']==0 and result['flights_to_first_reproduced'] is None


def test_boundary_passing_endpoint_requires_exposure_and_contrasts_exclude_unexposed():
    from corridor_space import experiment
    combined=proposal();single=copy.deepcopy(combined);single['delay_s']=0.
    delay=copy.deepcopy(combined);delay['rgb_noise_stddev']=0.
    probe=copy.deepcopy(combined);probe['rgb_noise_stddev']=12.
    source=history_row(combined)
    passing=history_row(delay,2,exposure=False,outcome='route_completed',experiment=experiment('isolate',1,'delay'))
    with pytest.raises(ValueError,match='exposure'):
        experiments.validate(experiment('threshold',1,'rgb_noise',2),probe,[source,passing],action_key)
    isolated=history_row(single,3,exposure=False,experiment=experiment('isolate',1,'rgb_noise'))
    result=workflow_evidence([source,passing,isolated])
    assert result['distinct_failure_factor_contrasts']==[] and result['isolation_evidence']==[]
    isolated['pairs'][0]['perturbed']['attack_exposure']['exposure_verified']=True
    result=workflow_evidence([source,passing,isolated])
    assert len(result['distinct_failure_factor_contrasts'])==len(result['isolation_evidence'])==1
    source['pairs'][0]['perturbed']['attack_exposure']['exposure_verified']=False
    assert workflow_evidence([source,isolated])['distinct_failure_factor_contrasts']==[]


def test_zero_amplitude_schedule_changes_do_not_create_distinct_settings():
    from corridor_space import experiment
    a=proposal();a['rgb_noise_stddev']=a['delay_s']=0.
    b=copy.deepcopy(a)
    b['noise_schedule']=dict(start_s=300.,duration_s=1.,period_s=2.)
    b['delay_schedule']=dict(start_s=400.,duration_s=2.,period_s=3.)
    assert action_key(a)==action_key(b)
    with pytest.raises(ValueError,match='repeat'):
        experiments.validate(experiment(),b,[history_row(a)],action_key)
