import copy
import random

import pytest

from camera_program import CameraProgram, validate, identity
from camera_policy import sample, baseline_action
from corridor_space import CAMERA_VERSION, reference_action, validate_action, action_to_episode, mission_for
from campaign import clean_twin
from episode import resolved
from camera_exposure import audit
from experiment_budget import validate as validate_budget
import experiment_protocol as experiments


def event(factor='frame_freeze',trigger='mission_time',start=2.,duration=3.,period=0.):
    return dict(factor=factor,trigger=trigger,start_s=start,duration_s=duration,period_s=period,turn_rate_rad_s=.2)


def test_windows_turn_trigger_and_reset():
    p=CameraProgram([event(),event('delay','first_turn')])
    assert p.state(None,.5)['active_factors']==[]
    assert p.state(1.,.5)['active_factors']==[]
    assert p.state(2.,None)['active_factors']==['frame_freeze']
    assert p.state(3.,.21)['active_factors']==['delay','frame_freeze']
    assert p.state(5.,0.)['active_factors']==['delay']
    assert p.state(6.,.5)['active_factors']==[]
    assert p.state(0.,.5)['triggered_at_s']=={}
    p=CameraProgram([event('rgb_noise',duration=2.,period=4.)])
    assert p.state(4.)['active_factors']==[]
    assert p.state(6.)['active_factors']==['rgb_noise']


def test_drop_reproducible_freeze_does_not_replay():
    events=[event('frame_drop',start=0.),event(start=1.,duration=1.)]
    def run():
        p=CameraProgram(events)
        states=[p.arrival(i/10.,None,42,.5) for i in range(40)]
        return states,p.telemetry()
    first,stats=run();assert first==run()[0]
    assert all(s['suppression']=='freeze' for s in first[10:20])
    assert all(not s['suppress'] for s in first[30:])
    assert stats['frozen_arrivals']==10 and 0<stats['dropped_arrivals']<20


@pytest.mark.parametrize('bad',[dict(event(),factor='replay'),dict(event(),duration_s=16.),
    dict(event(),start_s=float('nan')),dict(event(),period_s=2.),dict(event(),command='stop')])
def test_reject_out_of_scope_programs(bad):
    with pytest.raises(ValueError):validate([bad])


def test_every_sample_valid_and_clean_disables_all_events():
    for target in ('mononav','kim','pulp_dronet'):
        for seed in range(40):
            a=sample(random.Random(seed),target)
            assert validate_action(a,target)==a
            c=resolved(action_to_episode(a,target,42,'test',mission_for(target)))
            clean=clean_twin(c)
            assert clean['condition']['camera_program']==[]
            assert clean['condition']['frame_drop_probability']==0
            assert clean['condition']['corridor']==c['condition']['corridor']
            isolated=copy.deepcopy(a)
            for factor in experiments.FACTORS:experiments.disable(isolated,factor)
            assert not experiments.active(isolated)
            validate_action(isolated,target)


def test_enabled_factor_requires_matching_program():
    a=reference_action(CAMERA_VERSION);a['frame_freeze_enabled']=True
    with pytest.raises(ValueError):validate_action(a)
    a['camera_program']=[event()];validate_action(a)
    a['delay_schedule']['start_s']=1.
    with pytest.raises(ValueError):validate_action(a)


def test_exposure_requires_suppression_and_same_run_observed_stall():
    program=[event(start=0.)];p=CameraProgram(program)
    rows=[]
    for i in range(3):
        state=p.arrival(i,None,42,0)
        rows.append(dict(sim_time_s=i,sensor_disturbance={'camera_program_state':state,
            'camera_program_telemetry':p.telemetry()},worker={'run_id':'run','sequence':1,'sim_stamp':0.}))
    c=dict(camera_program=program,rgb_noise=0.,delay=0.)
    assert audit(rows,c,'run')['factors']['frame_freeze']['exposure_verified']
    assert not audit(rows,c,'other')['factors']['frame_freeze']['exposure_verified']
    assert not audit([],c,'run')['factors']['frame_freeze']['exposure_verified']
    for row in rows:row['sensor_disturbance']['camera_program_telemetry']['program_id']='wrong'
    assert not audit(rows,c,'run')['exposure_verified']


def test_budget_counts_pairs_and_reserves_confirmation():
    a=dict(flight_cap=8,discovery_pairs=2,confirmation_pairs=1,diagnosis_pairs=1,
           reason='Small functional pilot.',stop_rule='Stop on missing execution evidence.')
    assert validate_budget(a,12)==a
    for b in [dict(a,flight_cap=14),dict(a,confirmation_pairs=0),dict(a,flight_cap=7)]:
        with pytest.raises(ValueError):validate_budget(b,12)


def test_event_order_or_unused_threshold_does_not_create_new_failure_setting():
    from corridor_space import action_key
    a=reference_action(CAMERA_VERSION)
    a.update(frame_freeze_enabled=True,camera_program=[event(),event(start=8.)])
    b=copy.deepcopy(a);b['camera_program'].reverse()
    b['camera_program'][0]['turn_rate_rad_s']=.5
    assert action_key(a)==action_key(b)


def test_web_uses_agent_budget_with_no_hidden_extra_flights(tmp_path,monkeypatch):
    import dashboard,agent_policies
    from types import SimpleNamespace
    monkeypatch.setattr(agent_policies,'provider_from_name',lambda *_:SimpleNamespace(check_auth=lambda:None))
    command=dashboard.adaptive_command(dict(planner='pulp_dronet',budget=12,policy='agent_search',
        provider='codex',action_space='corridor',camera_programs=True,auto_budget=True,
        clean_validation_runs=0,infrastructure_retries=0,clean_failure_policy='record'),tmp_path)
    assert '--auto-budget' in command and '--camera-programs' in command
    with pytest.raises(ValueError):
        dashboard.adaptive_command(dict(planner='pulp_dronet',budget=12,policy='random',
            action_space='corridor',camera_programs=True,auto_budget=True),tmp_path)


def test_watchdog_requires_recent_live_suppression_and_same_run_stalled_input():
    from camera_exposure import interruption_at_guard
    program=[event('frame_drop',start=0.,duration=10.)]
    c=dict(camera_program=program,frame_drop_probability=.9,rgb_noise=0.,delay=0.)
    rows=[]
    for i in range(5):
        rows.append(dict(sim_time_s=10.+i*.2,
            sensor_disturbance=dict(camera_program_state=dict(program_id=identity(program),active_factors=['frame_drop']),
                camera_program_telemetry=dict(program_id=identity(program),dropped_arrivals=i,frozen_arrivals=0)),
            worker=dict(run_id='run',sequence=15,sim_stamp=8.)))
    evidence=interruption_at_guard(rows,c,'run')
    assert evidence['observed_factors']==['frame_drop']
    assert interruption_at_guard(rows,c,'different-run') is None
    assert interruption_at_guard(rows,dict(c,camera_program=[]),'run') is None
    # A configured attack, an old interruption, and a dead publisher do not
    # explain current stale input. Neither does an advancing worker input.
    for change in ('no_arrivals','wrong_program','inactive','worker_advances'):
        bad=copy.deepcopy(rows)
        for row in bad:
            d=row['sensor_disturbance']
            if change=='no_arrivals':d['camera_program_telemetry']['dropped_arrivals']=10
            if change=='wrong_program':d['camera_program_state']['program_id']='old'
            if change=='inactive':d['camera_program_state']['active_factors']=[]
        if change=='worker_advances':bad[-1]['worker']['sequence']+=1
        assert interruption_at_guard(bad,c,'run') is None


def test_budget_resume_preserves_generated_text_without_hiding_changed_scope():
    from experiment_budget import campaign_objective
    plan = {'plan_id': 'fixed-id'}
    legacy = 'Validate freeze. Shared frozen budget plan fixed-id. Follow its reserved confirmation/diagnosis allocation.'
    previous = {'study_objective': legacy}
    assert campaign_objective('Validate freeze.', plan, previous) == legacy
    assert campaign_objective('Validate drop.', plan, previous) != legacy
    assert campaign_objective('Validate freeze.', {'plan_id': 'new-id'}, previous) != legacy
    assert 'Keep its total cap fixed' in campaign_objective('Validate freeze.', plan)


def test_report_keeps_camera_factors_and_separates_them_from_shared_layout():
    from vulnerability_report import active_factors, analyze
    from agent_analysis import evidence_for_report
    a=reference_action(CAMERA_VERSION)
    a.update(frame_freeze_enabled=True,camera_program=[event(start=0.)])
    condition=resolved(action_to_episode(a,'pulp_dronet',42,'test',mission_for('pulp_dronet')))['condition']
    assert active_factors(condition)==['frame freeze']
    history=[]
    for i in range(2):
        c=copy.deepcopy(condition);c['camera_program'][0]['start_s']=float(i)
        history.append(dict(decision=dict(round=i+1,condition=c),pairs=[dict(planner='pulp_dronet',
            clean=dict(outcome='route_completed',metrics={}),
            perturbed=dict(outcome='planner_stopped',metrics={},
                termination=dict(source='bench_guard',reason='camera_input_guard'),
                attack_exposure=dict(exposure_verified=True)))]))
    report=analyze(history,[],dict(planners=['pulp_dronet'],action_space='corridor'),'complete')
    evidence=evidence_for_report(report)
    assert len(evidence['experimental_design']['distinct_shared_environments'])==1
    assert evidence['experimental_design']['single_factor_pairs']==['round_01','round_02']
    assert all(r['attacked']['termination']['source']=='bench_guard' for r in evidence['rounds'])
    assert 'camera_program' not in evidence['experimental_design']['distinct_shared_environments'][0]['shared_environment']
