import copy
import json
import random
import pytest
import agent_campaign
from corridor_space import (reference_action,validate_action,action_key,action_to_episode,
    mission_for,spec_for,sample,baseline_followup,experiment,CorridorPolicy)
from corridors import geometry,patch_corners,tier_spec
from campaign import clean_twin
from episode import resolved,fingerprint
from experiment_protocol import validate


def pair(a,n=1,clean='route_completed',attack='collision',e=None):
    return dict(decision=dict(round=n,action=a,experiment=e or experiment()),pairs=[dict(planner='mononav',
        clean=dict(outcome=clean,metrics={}),perturbed=dict(outcome=attack,metrics={}))])


def test_continuous_positions_counts_and_physical_patch_mount():
    a=dict(reference_action(),difficulty='medium',obstacles=[dict(route_s=6.,side=1),dict(route_s=18.,side=-1)],
           patch_enabled=True,patch_mount=dict(obstacle_index=1,face='front',height_m=1.4))
    g=geometry(spec_for(validate_action(a,'kim')))
    corners=patch_corners(g,.6,a['patch_mount'])
    assert sum(p[2] for p in corners)/4==pytest.approx(1.4)
    b=copy.deepcopy(a);b['obstacles'][0]['route_s']=6.3
    assert geometry(spec_for(b))['placement_id']!=g['placement_id']
    for target in ('mononav','pulp_dronet'):
        with pytest.raises(ValueError):validate_action(a,target)
    for obstacles in ([dict(route_s=1,side=1),dict(route_s=18,side=1)],
                      [dict(route_s=6,side=1),dict(route_s=6.5,side=-1)]):
        with pytest.raises(ValueError):validate_action(dict(a,obstacles=obstacles),'kim')
    with pytest.raises(ValueError):validate_action(dict(a,patch_size_m=.9,patch_mount=dict(obstacle_index=1,face='inner',height_m=1.4)),'kim')
    c=resolved(action_to_episode(a,'kim',42,'test',mission_for('kim')));clean=clean_twin(c)
    assert clean['condition']['corridor']==c['condition']['corridor']
    assert clean['condition']['patch_mount']==c['condition']['patch_mount']
    assert not clean['condition']['patch_enabled']


def test_legacy_geometry_and_new_sampling():
    assert geometry(tier_spec('elbow','medium'))['spec']['layout_version']==3
    for target in ('mononav','kim','pulp_dronet'):
        values=[sample(random.Random(i),target) for i in range(20)]
        assert len({action_key(a) for a in values})==20
        assert all(not a['patch_enabled'] for a in values) if target!='kim' else any(a['patch_enabled'] for a in values)
        for a in values:resolved(action_to_episode(a,target,42,'test',mission_for(target)))


def test_baselines_get_valid_repeat_isolation_and_threshold():
    a=dict(reference_action(),rgb_noise_stddev=24.,delay_s=.4)
    h=[pair(a)]
    b,e=baseline_followup(h);assert e['kind']=='repeat';validate(e,b,h,action_key)
    h.append(pair(a,2,e=e));b,e=baseline_followup(h)
    assert e['kind']=='isolate' and b['delay_s']==0.;validate(e,b,h,action_key)
    h.append(pair(b,3,e=e));c,e=baseline_followup(h);assert e['kind']=='repeat'
    h.append(pair(c,4,e=e));c,e=baseline_followup(h)
    assert e['kind']=='threshold' and c['rgb_noise_stddev']==12.;validate(e,c,h,action_key)


def test_corridor_agent_full_chain_uses_existing_runner(tmp_path,monkeypatch):
    # Historical v1 fixture predates measured sensor exposure. Current canonical
    # exposure/consumption and versioned planning have separate integration tests.
    import campaign_versions
    monkeypatch.setattr(campaign_versions,'apply',lambda config,previous=None,**kw:config)
    monkeypatch.setattr(agent_campaign,'RUNTIME',tmp_path)
    combined=dict(reference_action(),rgb_noise_stddev=24.,delay_s=.4)
    single=dict(combined,delay_s=0.);threshold=dict(single,rgb_noise_stddev=12.)
    choices=[(combined,experiment()),(combined,experiment('repeat',1)),
             (single,experiment('isolate',1,'rgb_noise')),(threshold,experiment('threshold',3,'rgb_noise'))]
    class Provider:
        chooses_action=True
        last_call_id='synthetic-test'
        calls=0
        def complete(self,purpose,system,payload,schema,validate):
            a,e=choices[self.calls];self.calls+=1
            return validate(dict(action=a,experiment=e,reason='Software fixture only.',hypothesis='Synthetic follow-up.'))
    calls=[]
    def fly(c,path,**kwargs):
        path.mkdir(parents=True);calls.append(c)
        r=dict(outcome='collision' if c['condition']['rgb_noise']>=24 else 'route_completed',metrics={},
               result_dir=str(path),configuration_hash=fingerprint(c))
        (path/'result.json').write_text(json.dumps(r));return r
    monkeypatch.setattr(agent_campaign,'run_episode',fly)
    root=tmp_path/'full'
    h=agent_campaign.run_adaptive_campaign(root,'agent_search',8,provider=Provider(),action_space='corridor',
        clean_validation_runs=0,clean_failure_policy='record',retries=0,pause_seconds=0,llm_report=False)
    assert len(calls)==8 and len(h)==4
    assert all(c['timeout']==500 for c in calls)
    from agent_analysis import evidence_for_report
    ev=evidence_for_report(json.loads((root/'vulnerability_report.json').read_text()))
    assert ev['action_space']=='corridor' and ev['rounds'][3]['experiment']['kind']=='threshold'
    assert ev['rounds'][0]['scene_evidence']['shape']=='elbow'


def test_web_corridor_uses_frozen_target_mission(tmp_path):
    from dashboard import adaptive_command
    command=adaptive_command(dict(planner='pulp_dronet',policy='random',action_space='corridor'),tmp_path)
    assert '--timeout' not in command and '--goal-distance' not in command


def test_comparison_metrics_use_real_interventions_not_llm_self_scores():
    from diagnostic_evaluation import workflow_evidence
    combined=dict(reference_action(),rgb_noise_stddev=24.,delay_s=.4)
    single=dict(combined,delay_s=0.);probe=dict(single,rgb_noise_stddev=12.)
    h=[pair(combined),pair(combined,2,e=experiment('repeat',1)),
       pair(single,3,e=experiment('isolate',1,'rgb_noise')),
       pair(probe,4,attack='route_completed',e=experiment('threshold',3,'rgb_noise'))]
    result=workflow_evidence(h)
    assert result['flights_to_first_reproduced']==4
    assert result['reproduced_condition_count']==1
    assert len(result['isolation_evidence'])==len(result['threshold_evidence'])==1
    assert result['threshold_evidence'][0]['observed_pass']==12
    assert result['threshold_evidence'][0]['observed_fail']==24
    h[-1]['decision']['action']['layout_seed']=7
    assert workflow_evidence(h)['invalid_experiments']


def test_bounds_and_config_are_json_roundtrip_stable():
    from corridor_space import bounds
    for target in ('mononav','kim','pulp_dronet'):
        b=bounds(target)
        assert json.loads(json.dumps(b))==b


def test_three_target_comparison_freezes_equal_budgets_and_common_reporter(tmp_path,monkeypatch):
    import compare_corridor_policies as comparison
    monkeypatch.setattr(comparison,'RUNTIME',tmp_path)
    class Provider:
        model='synthetic-test-model'
        def __init__(self,**kwargs):pass
        def check_auth(self):pass
        def metadata(self):return {'provider':'synthetic_fixture_only'}
    calls=[]
    def campaign(root,method,budget,seed,target,**kwargs):
        calls.append((target,method,budget,kwargs));root.mkdir(parents=True)
        a=dict(reference_action(),rgb_noise_stddev=10.)
        h=[pair(a,i+1,attack='route_completed',e=experiment() if i==0 else experiment('repeat',1)) for i in range(budget//2)]
        for name,value in [('config.json',{'synthetic':True}),('history.json',h),('presentation.json',{'phase':'complete'}),
                           ('llm_analysis.json',{'status':'complete','evidence':{'rounds':[]},'analysis':{'defense_candidates':[]}})]:
            (root/name).write_text(json.dumps(value))
    monkeypatch.setattr(comparison,'ClaudeSubscriptionProvider',Provider)
    monkeypatch.setattr(comparison,'run_adaptive_campaign',campaign)
    def artifact(root,**kwargs):
        (root/'corridor_artifact_audit.json').write_text(json.dumps({'passed':True,'synthetic':True}))
    monkeypatch.setattr(comparison,'artifact_audit',artifact)
    result=comparison.run(tmp_path/'study',flights=4)
    assert result['comparison_valid'] and len(calls)==9
    assert result['protocol']['total_reserved_flights']==36
    assert all(c[2]==4 and c[3]['retries']==c[3]['clean_validation_runs']==0 and isinstance(c[3]['provider'],Provider) for c in calls)
    assert all(r['reproduced_conditions']==r['isolations']==r['bracket_probes']==0 for r in result['results'])
    with pytest.raises(ValueError,match='Frozen'):comparison.run(tmp_path/'study',flights=6)
    assert len(calls)==9


def test_interaction_bracket_requires_measured_matching_endpoint():
    combined=dict(reference_action(),rgb_noise_stddev=24.,delay_s=.4)
    delay=dict(combined,rgb_noise_stddev=0.)
    probe=dict(combined,rgb_noise_stddev=12.)
    h=[pair(combined),pair(delay,2,attack='route_completed',e=experiment('isolate',1,'delay'))]
    e=experiment('threshold',1,'rgb_noise',2)
    assert validate(e,probe,h,action_key)
    with pytest.raises(ValueError,match='conditional'):validate(experiment('threshold',1,'rgb_noise'),probe,h,action_key)
    with pytest.raises(ValueError):validate(e,dict(probe,delay_s=.2),h,action_key)
    from diagnostic_evaluation import workflow_evidence
    h.append(pair(probe,3,attack='route_completed',e=e))
    bracket=workflow_evidence(h)['threshold_evidence'][0]
    assert bracket['fixed_other_factors']=={'delay_s':.4}


def test_baseline_can_probe_interaction_after_passing_isolated_arms():
    combined=dict(reference_action(),rgb_noise_stddev=24.,delay_s=.4)
    noise=dict(combined,delay_s=0.);delay=dict(combined,rgb_noise_stddev=0.)
    h=[pair(combined),pair(combined,2,e=experiment('repeat',1)),
       pair(noise,3,attack='route_completed',e=experiment('isolate',1,'rgb_noise')),
       pair(delay,4,attack='route_completed',e=experiment('isolate',1,'delay'))]
    a,e=baseline_followup(h)
    assert e['kind']=='threshold' and e['passing_round']==4
    assert a['rgb_noise_stddev']==12. and a['delay_s']==.4
    validate(e,a,h,action_key)


def test_search_simplifies_shared_geometry_after_clean_failure():
    a=dict(reference_action(),difficulty='medium',obstacles=[dict(route_s=6,side=1),dict(route_s=18,side=-1)],rgb_noise_stddev=10.)
    p=CorridorPolicy('search',42)
    d=p.choose_next([pair(a,clean='collision')],5)
    assert d['action']['obstacles']==[] and d['action']['difficulty']=='easy'
    assert d['action']['rgb_noise_stddev']==10. and d['experiment']['kind']=='explore'


def test_unperturbed_repeat_disagreement_is_not_attack_evidence(tmp_path):
    from vulnerability_report import analyze
    from diagnostic_evaluation import workflow_evidence
    from compare_policies import method_result
    a=reference_action();h=[pair(a)]
    c=resolved(action_to_episode(a,'mononav',42,'test',mission_for('mononav')))
    h[0]['decision']['condition']=c['condition']
    report=analyze(h,[],{'planners':['mononav'],'mission':mission_for('mononav'),'action_space':'corridor'},'complete')
    assert report['paired_comparisons'][0]['attack_effect_evaluable'] is False
    assert not report['findings'] and workflow_evidence(h)['reproduced_condition_count']==0
    (tmp_path/'history.json').write_text(json.dumps(h))
    assert method_result(tmp_path)['distinct_candidate_conditions']==0


def test_report_rejects_observed_layout_and_defense_misstatements():
    from agent_analysis import validate_analysis
    evidence={'rounds':[dict(id='round_01',attack_effect_evaluable=True,attacked={'outcome':'insufficient_progress'})],
              'experimental_design':{'distinct_shared_environments':[{'obstacles':1},{'obstacles':0}]}}
    defense=dict(proposal='Use a single-frame edge-preserving RGB filter before depth estimation.',failure_hypothesis='Depth noise may cause false blockage.',
        rationale='Noise-only flight failed with a passing clean control.',tradeoffs='Filtering may remove edges and add latency.',
        validation_plan='Compare baseline and filter in clean and noisy flights, repeated and held-out; measure completion, collision and latency.',
        status='unvalidated_proposal',evidence_rounds=['round_01'])
    raw=dict(summary='Two layouts of the same corridor shape were tested.',findings=[dict(claim='A candidate was observed.',status='candidate',evidence_rounds=['round_01'])],
        limitations=['One seed.'],next_tests=[],defense_candidates=[defense])
    validate_analysis(raw,evidence)
    for proposal in ('Before declaring blocked, require agreement across several frames.',
                     'Apply a short multi-frame average while keeping original observation poses.'):
        bad=copy.deepcopy(raw);bad['defense_candidates'][0]['proposal']=proposal
        with pytest.raises(ValueError):validate_analysis(bad,evidence)
    good=copy.deepcopy(raw);good['defense_candidates'][0]['proposal']='Before declaring unblocked, confirm fresh safe observations while keeping the immediate hold.'
    validate_analysis(good,evidence)
    for exclusion in ('No temporal fusion.', 'Do not use temporal averaging.',
                      'Use no temporal averaging, so no cross-pose registration is needed.',
                      'Use the current frame without temporal image averaging.'):
        good=copy.deepcopy(raw);good['defense_candidates'][0]['proposal']='Apply a single-frame filter. '+exclusion
        validate_analysis(good,evidence)
    bad=copy.deepcopy(raw);bad['defense_candidates'][0]['proposal']='No temporal fusion in the RGB filter. Add multi-frame averaging of depth while retaining each original pose.'
    with pytest.raises(ValueError,match='registration'):validate_analysis(bad,evidence)
    bad=copy.deepcopy(raw);bad['limitations']=['All pairs share one fixed layout.']
    with pytest.raises(ValueError,match='Multiple physical'):validate_analysis(bad,evidence)


def test_factor_information_is_deduplicated_and_not_scored_from_purpose_labels():
    from diagnostic_evaluation import workflow_evidence
    combined=dict(reference_action(),rgb_noise_stddev=24.,delay_s=.4)
    single=dict(combined,delay_s=0.)
    h=[pair(combined),pair(combined,2,e=experiment('repeat',1)),
       pair(single,3,e=experiment()),pair(single,4,e=experiment('repeat',3))]
    v=workflow_evidence(h)
    assert not v['invalid_experiments'] and v['experiment_counts']['isolate']==0
    assert v['distinct_failure_factor_contrasts']==[dict(factor='rgb_noise',combined_rounds=[1,2],isolated_rounds=[3,4])]
