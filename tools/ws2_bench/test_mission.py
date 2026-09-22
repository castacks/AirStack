import copy,json
import pytest
from episode import resolved,worker_command
from mission import horizon_outcome,motion_metrics
from campaign import verdict
from vulnerability_report import analyze
from feedback import choose_next
from dashboard import feedback_command,set_view
import dashboard,operator_control

def samples(distance):
    return [{'sim_time_s':float(t),'position_m':[distance*t/120,0,1.2]} for t in range(121)]

def test_model_specific_mission_and_speed_commands():
    kim=resolved({'planner':'kim'});mono=resolved({'planner':'mononav'})
    assert kim['timeout']==120 and kim['mission_mode']=='avoidance'
    assert mono['timeout']==180 and mono['goal_distance']==8 and mono['goal_radius']==.5
    assert horizon_outcome(kim,samples(6))=='completed_horizon'
    assert horizon_outcome(kim,samples(.1))=='insufficient_progress'
    move_then_park=[{'sim_time_s':float(t),'position_m':[min(t/5,4),0,1.2]} for t in range(121)]
    assert horizon_outcome(kim,move_then_park)=='insufficient_progress'
    assert horizon_outcome(mono,samples(6))=='timeout'
    assert '--goal-distance' not in worker_command(kim)
    cmd=worker_command(kim);assert cmd[cmd.index('--maximum-speed')+1]=='0.35'
    assert cmd[cmd.index('--initial-speed')+1]=='0.2'
    mono_cmd=worker_command(mono);assert mono_cmd[mono_cmd.index('--velocity')+1]=='0.3'
    assert cmd[cmd.index('--trajectory-horizon')+1]=='2.0'
    assert verdict({'outcome':'completed_horizon'},{'outcome':'collision'})=='autonomy_failure'
    assert verdict({'outcome':'user_stopped'},{'outcome':'collision'})=='user_stopped'
    with pytest.raises(ValueError):resolved({'planner':'kim','mission_mode':'goal'})
    assert motion_metrics(samples(6))['stationary_fraction']==0

def test_report_confirmation_and_attribution_limits():
    decision=choose_next([])
    pair={'planner':'kim','clean':{'outcome':'completed_horizon','metrics':{'path_length_m':8}},
          'perturbed':{'outcome':'collision','metrics':{'path_length_m':4}}}
    h=[{'decision':decision,'pairs':[pair]}]
    config={'planners':['kim']};rows=[{'outcome':'completed_horizon'},{'outcome':'collision'},{'outcome':'user_stopped'}]
    r=analyze(h,rows,config,'running')
    assert r['findings'][0]['status']=='candidate_needs_repeat'
    assert len(r['findings'][0]['active_factors'])==3 and 'unresolved' in r['findings'][0]['interpretation']
    assert r['evaluated_flights']==2 and r['excluded_flights']==1
    assert analyze(h+h,rows,config,'complete')['findings'][0]['status']=='reproduced_condition'
    bad=copy.deepcopy(h);bad[0]['pairs'][0]['clean']['outcome']='insufficient_progress'
    r=analyze(bad,rows,config,'complete');assert not r['findings'] and len(r['conditions_with_clean_failure'])==1

def test_operator_pause_stop_and_view_validation(tmp_path,monkeypatch):
    p=tmp_path/'control.json';p.write_text('{"action":"pause"}');events=[]
    monkeypatch.setattr(operator_control.time,'sleep',lambda _:p.write_text('{"action":"run"}'))
    operator_control.wait_between_trials(p,events.append);assert events==['paused']
    p.write_text('{"action":"stop"}')
    with pytest.raises(operator_control.UserStop):operator_control.wait_between_trials(p,events.append)
    monkeypatch.setattr(dashboard,'RUNTIME',tmp_path)
    assert set_view({'distance':8})['distance']==8
    assert set_view({'height':4})['distance']==8
    with pytest.raises(ValueError):set_view({'distance':float('nan')})
    for payload in [{'planner':'kim','budget':3},{'planner':'kim','timeout':5}]:
        with pytest.raises(ValueError):feedback_command(payload,tmp_path)
