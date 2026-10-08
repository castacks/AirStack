import copy, json
from dataclasses import replace
from pathlib import Path
import pytest
from challenge_layouts import CHALLENGES,generate_challenge
from generated_layouts import validate_placement,identity,geometry,translated
from expanded_space import ExpandedPolicy,reference_action,proposal_schema
from model_adapters import REGISTRY,register,get_adapter
from episode import resolved,worker_command,fingerprint
from agent_analysis import validate_analysis,analysis_schema
from mission import defaults,horizon_outcome,agent_mission
import sensor_study

@pytest.mark.parametrize('challenge',CHALLENGES)
def test_challenges_have_supported_geometry_and_saved_feasible_routes(challenge):
    p=generate_challenge(challenge)
    assert validate_placement(json.loads(json.dumps(p)))==p
    assert p['feasible_route_world_m'][0]==[-4.,0.,1.2]
    assert p['feasible_route_world_m'][-1]==[4.,0.,1.2]
    if challenge!='offset_gap':assert any(abs(v[1])>=.4 for v in p['feasible_route_world_m'])
    p['feasible_route_world_m']=[[-4,0,1.2],[4,0,1.2]]
    p['placement_id']=identity({k:v for k,v in p.items() if k!='placement_id'})
    with pytest.raises(ValueError,match='route'):validate_placement(p)

def test_challenge_rehash_does_not_allow_start_overlap():
    p=generate_challenge();b=geometry()['sources']['column']['bounds']
    p['offsets']['column']=[-(b[0][0]+b[1][0])/2,-4-(b[0][1]+b[1][1])/2,0.]
    p['bounds_source_m']['column']=translated(b,p['offsets']['column'])
    p['placement_id']=identity({k:v for k,v in p.items() if k!='placement_id'})
    with pytest.raises(ValueError,match='endpoint'):validate_placement(p)

def test_mononav_patch_is_rejected_before_any_flight():
    with pytest.raises(ValueError,match='ZoeDepth'):resolved({'planner':'mononav','condition':{'patch_enabled':True}})
    assert resolved({'planner':'kim','condition':{'patch_enabled':True}})['condition']['patch_enabled']
    assert proposal_schema()['properties']['action']['properties']['patch_enabled']['enum']==[False]

@pytest.mark.parametrize('method',['random','search'])
def test_sensor_profile_never_reenables_patch_after_success(method):
    a=reference_action()
    history=[{'decision':{'rule':'initial','action':a},'pairs':[{'clean':{'outcome':'goal_reached'},'perturbed':{'outcome':'goal_reached'}}]}]
    for seed in range(12):
        action=ExpandedPolicy(method,seed).choose_next(history,1)['action']
        assert action['patch_enabled'] is False and action['patch_start_s']==action['patch_duration_s']==0

def test_third_adapter_uses_common_worker_and_mission_contract_without_model_branch():
    adapter=replace(get_adapter('mononav'),name='contract_test',image='test:not-run',
                    entrypoint='dummy_worker.py',worker_arguments=('--velocity','{velocity}'))
    register(adapter)
    try:
        c=resolved({'planner':'contract_test'});cmd=worker_command(c)
        assert 'dummy_worker.py' in cmd and 'test:not-run' in cmd
        assert agent_mission('contract_test',c)['goal_distance']==8
        assert horizon_outcome(c,[])=='timeout'
        assert c['velocity']==.3
    finally:REGISTRY.pop('contract_test')

def defense_fixture():
    evidence={'rounds':[{'id':'round_01','attack_effect_evaluable':True,'attacked':{'outcome':'collision'}}]}
    value={'summary':'A candidate requires confirmation.','findings':[{'claim':'A failure was observed.','status':'candidate','evidence_rounds':['round_01']}],
           'limitations':['One pair.'],'next_tests':['Repeat.'],'defense_candidates':[{
               'proposal':'Age-aware command gating.','failure_hypothesis':'Stale observations may cause late avoidance.',
               'rationale':'Delay was the isolated factor.','tradeoffs':'More stops and slower progress.',
               'validation_plan':'Compare matched clean and delayed seeds with/without gating; count collisions, progress and false stops.',
               'status':'unvalidated_proposal','evidence_rounds':['round_01']}]}
    return value,evidence

def test_defense_requires_grounding_and_cannot_claim_validated_effectiveness():
    value,evidence=defense_fixture();assert validate_analysis(value,evidence)==value
    for key,v in [('status','validated'),('evidence_rounds',['round_99']),('tradeoffs','')]:
        bad=copy.deepcopy(value);bad['defense_candidates'][0][key]=v
        with pytest.raises(ValueError):validate_analysis(bad,evidence)

def test_single_factor_study_stops_on_bad_clean_and_does_not_fly_attack(tmp_path,monkeypatch):
    monkeypatch.setattr(sensor_study,'RUNTIME',tmp_path);calls=[]
    def fly(c,folder,**kwargs):
        calls.append(c);folder.mkdir();return {'outcome':'collision','metrics':{},'result_dir':str(folder)}
    monkeypatch.setattr(sensor_study,'run_episode',fly)
    sensor_study.run_study(tmp_path/'study','kim')
    assert len(calls)==1 and calls[0]['condition']['rgb_noise']==calls[0]['condition']['delay']==0
    assert json.loads((tmp_path/'study/vulnerability_report.json').read_text())['paired_comparisons']==[]

def test_single_factor_pairing_resume_and_patch_exclusion(tmp_path,monkeypatch):
    monkeypatch.setattr(sensor_study,'RUNTIME',tmp_path);calls=[]
    def fly(c,folder,**kwargs):
        calls.append(copy.deepcopy(c));folder.mkdir();r={'configuration_hash':fingerprint(c),'outcome':'goal_reached','metrics':{},'result_dir':str(folder)}
        (folder/'result.json').write_text(json.dumps(r));return r
    monkeypatch.setattr(sensor_study,'run_episode',fly)
    root=tmp_path/'study';sensor_study.run_study(root,'mononav');sensor_study.run_study(root,'mononav')
    assert len(calls)==4
    assert [c['condition']['rgb_noise'] for c in calls]==[0,32,0,0]
    assert [c['condition']['delay'] for c in calls]==[0,0,0,.5]
    assert all(not c['condition']['patch_enabled'] for c in calls)
    assert all(c['condition']['placement']==calls[0]['condition']['placement'] for c in calls)
    from agent_analysis import evidence_for_report
    report=json.loads((root/'vulnerability_report.json').read_text())
    evidence=evidence_for_report(report)
    assert evidence['experimental_design']['single_factor_pairs']==['round_01','round_02']
    assert evidence['experimental_design']['combined_factor_pairs']==[]
    assert any('Every pair isolates one perturbation' in s for s in report['limitations'])

def test_stopped_sensor_flight_ends_study_without_starting_another(tmp_path,monkeypatch):
    monkeypatch.setattr(sensor_study,'RUNTIME',tmp_path);calls=[]
    def fly(c,folder,**kwargs):
        calls.append(c);folder.mkdir()
        return {'outcome':'goal_reached' if len(calls)==1 else 'user_stopped','metrics':{},'result_dir':str(folder)}
    monkeypatch.setattr(sensor_study,'run_episode',fly)
    root=tmp_path/'stopped';sensor_study.run_study(root,'mononav')
    assert len(calls)==2
    r=json.loads((root/'vulnerability_report.json').read_text())
    assert r['phase']=='stopped' and r['excluded_flights']==1 and not r['paired_comparisons']

def test_analysis_rejects_false_global_single_run_claim_with_repeated_condition():
    evidence={'rounds':[], 'experimental_design':{'condition_repetitions':[{'pair_count':2}]}}
    raw={'summary':'One run per condition was tested.','findings':[],
         'limitations':['Small sample.'],'next_tests':[],'defense_candidates':[]}
    with pytest.raises(ValueError,match='Repeated conditions exist'):validate_analysis(raw,evidence)
    raw['summary']='Two pairs repeated one condition; the sample remains small.'
    assert validate_analysis(raw,evidence)==raw
