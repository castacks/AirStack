import copy
import json
import pytest
import agent_campaign
from expanded_space import ExpandedPolicy, reference_action, action_key, bounds, sample
from experiment_protocol import validate
from episode import fingerprint
from diagnostics import summarize


def round_(action, index=1, attack='collision', clean='goal_reached'):
    return {'decision': {'round':index,'rule':'llm_selected_configuration','action':action},
            'pairs':[{'planner':'mononav','clean':{'outcome':clean,'metrics':{}},
                      'perturbed':{'outcome':attack,'metrics':{},'diagnostics':{'sample_count':10}}}]}


def exp(kind, source=0, factor='none', passing=0):
    return dict(kind=kind,source_round=source,passing_round=passing,factor=factor)


def test_repeat_is_agent_selected_and_receives_diagnostics():
    a=dict(reference_action(),rgb_noise_stddev=32.)
    class Provider:
        chooses_action=True
        last_call_id='test-call'
        def complete(self,purpose,system,payload,schema,validate):
            assert payload['history'][0]['pairs'][0]['perturbed']['diagnostics']['sample_count']==10
            return validate({'hypothesis':'Check reproducibility.','reason':'One failure is insufficient.',
                             'action':a,'experiment':exp('repeat',1)})
    d=ExpandedPolicy('agent_search',42,Provider()).choose_next([round_(a)],1)
    assert d['llm_call_id']=='test-call' and d['experiment']['kind']=='repeat'
    assert d['rule']=='llm_selected_configuration'


def test_isolation_preserves_geometry_and_factor_value():
    base=dict(reference_action(),rgb_noise_stddev=32.,delay_s=.5)
    isolated=dict(base,delay_s=0.)
    assert validate(exp('isolate',1,'rgb_noise'),isolated,[round_(base)],action_key)
    for change in ({'layout_seed':43},{'rgb_noise_stddev':20.},{'delay_s':.1}):
        with pytest.raises(ValueError):
            validate(exp('isolate',1,'rgb_noise'),dict(isolated,**change),[round_(base)],action_key)


def test_threshold_requires_observed_bracket_and_only_one_axis():
    base=dict(reference_action(),rgb_noise_stddev=32.)
    history=[round_(base),round_(dict(base,rgb_noise_stddev=10.),2,attack='goal_reached')]
    probe=dict(base,rgb_noise_stddev=20.)
    assert validate(exp('threshold',1,'rgb_noise',2),probe,history,action_key)
    assert validate(exp('threshold',1,'rgb_noise'),probe,history,action_key)
    for change in ({'rgb_noise_stddev':10.},{'rgb_noise_stddev':32.},{'layout_seed':44},{'delay_s':.1}):
        with pytest.raises(ValueError):
            validate(exp('threshold',1,'rgb_noise',2),dict(probe,**change),history,action_key)
    with pytest.raises(ValueError):
        validate(exp('threshold',1,'patch'),probe,history,action_key)


def test_failed_clean_does_not_support_isolation_or_threshold():
    a=dict(reference_action(),rgb_noise_stddev=32.,delay_s=.5)
    with pytest.raises(ValueError,match='passing clean'):
        validate(exp('isolate',1,'delay'),dict(a,rgb_noise_stddev=0.),[round_(a,clean='collision')],action_key)


def test_target_specific_patch_space_for_all_methods():
    import random
    assert bounds('kim')['patch_enabled']==[False,True]
    assert bounds('mononav')['patch_enabled']==[False]
    assert any(sample(random.Random(i),'kim')['patch_enabled'] for i in range(12))
    assert not any(sample(random.Random(i),'mononav')['patch_enabled'] for i in range(12))
    policy=ExpandedPolicy('search',42);policy.campaign_context={'target_planner':'mononav'}
    with pytest.raises(ValueError,match='ZoeDepth'):
        policy.decision([],dict(reference_action(),patch_enabled=True),'test','test','test')


def test_autonomous_clean_failure_skips_attack_then_continues_and_resumes(tmp_path,monkeypatch):
    monkeypatch.setattr(agent_campaign,'RUNTIME',tmp_path)
    calls=[]
    class Provider:
        chooses_action=True
        last_call_id='test'
        calls=0
        def complete(self,purpose,system,payload,schema,validate):
            self.calls+=1
            assert len(payload['history'])==self.calls-1
            if self.calls==2:
                assert payload['history'][0]['pairs'][0]['perturbed']['outcome']=='not_run_clean_failed'
            return validate({'hypothesis':'Test a fresh layout.','reason':'Qualify then perturb.',
                'action':dict(reference_action(),layout_seed=41+self.calls,rgb_noise_stddev=20.),
                'experiment':exp('explore')})
    def fly(c,path,**kwargs):
        path.mkdir();calls.append(c)
        result={'outcome':'collision' if len(calls)==1 else 'goal_reached','metrics':{},
                'configuration_hash':fingerprint(c),'result_dir':str(path)}
        (path/'result.json').write_text(json.dumps(result));return result
    monkeypatch.setattr(agent_campaign,'run_episode',fly)
    provider=Provider();root=tmp_path/'campaign'
    opts=dict(provider=provider,budget=4,action_space='expanded',clean_failure_policy='record',
              clean_validation_runs=0,retries=0,pause_seconds=0,llm_report=False)
    h=agent_campaign.run_adaptive_campaign(root,'agent_search',**opts)
    assert len(h)==2 and len(calls)==3 and provider.calls==2
    report=json.loads((root/'vulnerability_report.json').read_text())
    assert report['evaluated_flights']==3 and len(report['conditions_with_clean_failure'])==1
    assert not report['findings']
    agent_campaign.run_adaptive_campaign(root,'agent_search',**opts)
    assert len(calls)==3 and provider.calls==2
    live=json.loads((root/'presentation.json').read_text())
    assert live['completed']==3 and live['reserved_flights']==4 and live['skipped_attacks']==1


def test_diagnostics_deduplicate_frames_and_do_not_invent_cause():
    records=[{'sim_time_s':float(i),'position_m':[i*.1,0,1.2],'clearance_m':.4,
              'camera_age_sim_s':.6,'worker':{'sequence':1,'sim_stamp':1.,'inference_ms':100}} for i in range(8)]
    d=summarize(records,0.,'collision')
    assert d['window_start_sim_s']==2 and d['unique_worker_frames']==1
    assert d['mean_motion_speed_m_s']==pytest.approx(.1)
    assert d['camera_age_sim_s']['mean']==pytest.approx(.6)
    assert 'cause' not in d


def test_full_followup_chain_executes_within_budget_and_reaches_report(tmp_path,monkeypatch):
    monkeypatch.setattr(agent_campaign,'RUNTIME',tmp_path)
    combined=dict(reference_action(),rgb_noise_stddev=32.,delay_s=.5)
    isolated=dict(combined,delay_s=0.)
    probe=dict(isolated,rgb_noise_stddev=16.)
    selections=[(combined,exp('explore')),(isolated,exp('isolate',1,'rgb_noise')),
                (probe,exp('threshold',2,'rgb_noise')),(probe,exp('repeat',3))]
    class Provider:
        chooses_action=True
        last_call_id='test'
        calls=0
        def complete(self,purpose,system,payload,schema,validate):
            assert len(payload['history'])==self.calls
            action,experiment=selections[self.calls];self.calls+=1
            return validate(dict(hypothesis='Check the noise hypothesis.',reason='Use measured prior results.',
                                 action=action,experiment=experiment))
    calls=[]
    def fly(c,path,**kwargs):
        path.mkdir();calls.append(c)
        result={'outcome':'collision' if c['condition']['rgb_noise']>=32 else 'goal_reached',
                'metrics':{},'diagnostics':{'sample_count':8},'configuration_hash':fingerprint(c),'result_dir':str(path)}
        (path/'result.json').write_text(json.dumps(result));return result
    monkeypatch.setattr(agent_campaign,'run_episode',fly)
    root=tmp_path/'chain';provider=Provider()
    history=agent_campaign.run_adaptive_campaign(root,'agent_search',8,provider=provider,
        action_space='expanded',clean_validation_runs=0,retries=0,pause_seconds=0,llm_report=False)
    assert len(calls)==8 and provider.calls==4
    assert [r['decision']['experiment']['kind'] for r in history]==['explore','isolate','threshold','repeat']
    from agent_analysis import evidence_for_report
    evidence=evidence_for_report(json.loads((root/'vulnerability_report.json').read_text()))
    assert evidence['rounds'][2]['experiment']['source_round']==2
    assert evidence['rounds'][2]['attacked']['diagnostics']['sample_count']==8
    assert evidence['evaluated_flights']==8


def test_comparison_never_counts_a_skipped_attack_as_a_flight(tmp_path):
    from compare_policies import method_result
    failed=round_(reference_action(),clean='collision')
    failed['pairs'][0]['perturbed']={'outcome':'not_run_clean_failed','metrics':{},'executed':False}
    candidate=round_(dict(reference_action(),rgb_noise_stddev=20.),index=2)
    (tmp_path/'history.json').write_text(json.dumps([failed,candidate]))
    result=method_result(tmp_path)
    assert result['scheduled_flights']==4 and result['executed_flights']==3
    assert result['skipped_flight_slots']==1 and result['clean_failure_pairs']==1
    assert result['flights_to_first_candidate']==3


@pytest.mark.parametrize('summary',[
    'The passing control had negative clearance, so it was not a fully clean pass.',
    'Lower noise and delay so clean controls can pass.'])
def test_report_cannot_reinterpret_clean_or_attack_execution(summary):
    from agent_analysis import validate_analysis
    raw={'summary':summary,'findings':[],'limitations':['Small sample.'],'next_tests':[], 'defense_candidates':[]}
    with pytest.raises(ValueError,match='Clean'):
        validate_analysis(raw,{'rounds':[]})
