"""Offline proposal support, fairness and result-guidance checks; no flights."""
import copy
import json
import random

import pytest

from baseline_proposals_v2 import (Prior, Proposer, ProposalExhausted, _schedule,
                                   distribution, proposal_key, RANKING_VERSION)
from conditions import validate_schedule
from corridor_space import VERSION, action_key, reference_action, validate_action


def history_row(number, *, failed=True, loss=20., eligible=True, seed=None):
    a=reference_action(VERSION)
    a.update(layout_seed=number if seed is None else seed,rgb_noise_stddev=12.)
    p={'clean': {'outcome': 'route_completed', 'executed': True,
                 'metrics': {'route_progress_percent': 98.}},
       'perturbed': {'outcome': 'collision' if failed else 'route_completed', 'executed': True,
                     'metrics': {'route_progress_percent': 98.-loss},
                     'attack_exposure': {'factors': {'noise': {'requested': True, 'exposure_verified': True,
                                                             'status': 'exposed','active_frames': 10}}},
                     'worker_input_exposure': {
                         'schema':'observed_consumed_input_v1','status':'complete','configuration_checked':True,
                         'population':'observed unique worker-consumed source-frame updates',
                         'observed_unique_consumed_frames':10,
                         'factor_counts':{'noise':{'active':10,'inactive':0,'unknown':0},
                                          'delay':{'active':0,'inactive':10,'unknown':0}},
                         'evidence_sources':{'direct_worker_metadata':10,'same_run_exact_frame_join':0,'unavailable':0}}}}
    if not eligible:p['perturbed']['worker_input_exposure']=None
    return {'decision':{'round':number,'action':a},'pairs':[p]}


@pytest.fixture(scope='module')
def kim_samples():
    return [Proposer('random',11,'kim').propose([],i) for i in range(80)]


def test_actual_valid_samples_cover_expanded_geometry_and_patch_domain(kim_samples):
    samples=[r['action'] for r in kim_samples]
    for a in samples:assert validate_action(a,'kim')==a
    assert {a['shape'] for a in samples}=={'elbow','curve','zigzag','u_obstacles'}
    assert {a['difficulty'] for a in samples}=={'easy','medium','hard'}
    assert {len(a['obstacles']) for a in samples}==set(range(8))
    obs=[o for a in samples for o in a['obstacles']]
    assert max(o['width_m'] for o in obs)>1.2
    assert max(o['depth_m'] for o in obs)>1.2
    assert min(o['height_m'] for o in obs)<1. and max(o['height_m'] for o in obs)>2.5
    assert min(o['yaw_deg'] for o in obs)<-40 and max(o['yaw_deg'] for o in obs)>40
    assert min(o['lateral_m'] for o in obs)<-1.6 and max(o['lateral_m'] for o in obs)>1.6
    patched=[a for a in samples if a['patch_enabled']]
    assert patched and len(patched)<len(samples)
    assert max(a['patch_size_m'] for a in patched)>.8
    assert {a['patch_mount']['face'] for a in patched}=={'front','inner'}
    assert any(a['patch_start_s']>80 for a in patched)
    assert any(a['patch_duration_s']==0 for a in patched)
    assert any(a['patch_duration_s']>80 for a in patched)
    assert any(r['audit']['rejections'] for r in kim_samples)


def test_schedule_prior_has_meaningful_mass_and_retains_full_support():
    rng=random.Random(7);p=Prior();samples=[_schedule(rng,p) for _ in range(3000)]
    for s in samples:assert validate_schedule(s)==s
    whole=sum(s=={'start_s':0.,'duration_s':0.,'period_s':0.} for s in samples)/len(samples)
    assert .30<whole<.40
    assert any(s['start_s']>450 for s in samples)
    assert any(s['duration_s']>450 for s in samples)
    assert any(s['period_s']>450 for s in samples)
    assert any(s['start_s']>0 and s['duration_s']==0 for s in samples)
    assert any(2<=s['duration_s']<=30 and s['period_s']==0 for s in samples)
    assert any(0<s['duration_s']<s['period_s'] for s in samples)
    spec=distribution(p)
    assert spec['parameters']['useful_time_horizon_s']==200.
    assert 'NOT uniform' in spec['geometry']


@pytest.mark.parametrize('target',['mononav','pulp_dronet'])
def test_unsupported_patch_never_appears_and_global_start_is_common(target):
    for index in range(8):
        random_result=Proposer('random',1,target).propose([],index)
        search_result=Proposer('search',1,target).propose([],index)
        assert random_result['action']==search_result['action']
        assert random_result['action']['patch_enabled'] is False
        assert search_result['audit']['fallback']=='no_eligible_incumbent'


def test_seed_index_replay_is_exact_and_all_audit_values_serialize(kim_samples):
    repeated=Proposer('random',11,'kim').propose([],3)
    assert repeated==kim_samples[3]
    assert repeated!=Proposer('random',11,'kim').propose([],4)
    assert repeated!=Proposer('random',12,'kim').propose([],3)
    json.dumps(kim_samples,allow_nan=False)


def test_random_distribution_is_independent_of_results_and_never_mutates_history():
    h1=[history_row(1),history_row(2,failed=False)]
    h2=copy.deepcopy(h1)
    for row in h2:
        row['pairs'][0]['clean']['outcome']='collision'
        row['pairs'][0]['perturbed']['outcome']='infrastructure_error'
        row['pairs'][0]['perturbed']['metrics']['route_progress_percent']=0.
    originals=copy.deepcopy((h1,h2));sampler=Proposer('random',52,'pulp_dronet')
    assert sampler.propose(h1,5)==sampler.propose(h2,5)
    assert (h1,h2)==originals


def test_random_excludes_exact_existing_action_without_rewarding_its_outcome():
    sampler=Proposer('random',2,'pulp_dronet')
    first=sampler.propose([],0)
    history=[{'decision':{'round':1,'action':first['action']},'pairs':[]}]
    second=sampler.propose(history,0)
    assert action_key(first['action'])!=action_key(second['action'])
    assert second['audit']['rejections']['final: duplicate action']>=1
    assert second['audit']['duplicate_rejections']['exact']>=1


def test_only_proven_inactive_fields_are_canonicalized_and_raw_action_is_preserved():
    first=reference_action(VERSION);second=copy.deepcopy(first)
    second['noise_schedule']['start_s']=123.
    second['delay_schedule']['duration_s']=33.
    second['patch_size_m']=.9
    second['patch_mount']={'obstacle_index':0,'face':'inner','height_m':1.2}
    original=copy.deepcopy(second)
    assert first!=second and proposal_key(first)==proposal_key(second)
    assert second==original
    second['layout_seed']+=1
    assert proposal_key(first)!=proposal_key(second)
    second=copy.deepcopy(first);second['rgb_noise_stddev']=8.
    changed=copy.deepcopy(second);changed['noise_schedule']['start_s']=123.
    assert proposal_key(second)!=proposal_key(changed)


def test_inactive_setting_change_does_not_buy_an_additional_random_discovery():
    sampler=Proposer('random',2,'pulp_dronet')
    # Find a deterministic proposal with at least one inactive sensor.
    for index in range(12):
        first=sampler.propose([],index)
        a=first['action']
        key=('noise_schedule' if a['rgb_noise_stddev']==0 else
             'delay_schedule' if a['delay_s']==0 else None)
        if key:break
    else:pytest.fail('Expected an inactive sensor in the seeded sample')
    previous=copy.deepcopy(a);previous[key]['start_s']=(previous[key]['start_s']+1.)%500.
    second=sampler.propose([{'decision':{'action':previous}}],index)
    assert proposal_key(a)!=proposal_key(second['action'])
    assert second['audit']['duplicate_rejections']['inactive_setting_equivalent']>=1


def test_search_uses_measured_incumbent_and_does_not_enter_shared_followup_queue():
    # Eligible failure outranks even a very large progress difference on a pass;
    # an unexposed failure is not an incumbent. The exact-repeat rules are absent.
    rows=[history_row(1,failed=False,loss=80.),history_row(2,failed=True,loss=20.),
          history_row(3,failed=True,loss=95.,eligible=False)]
    prior=Prior(elite_size=1)
    results=[Proposer('search',55,'pulp_dronet',prior).propose(rows,i) for i in range(16)]
    local=[r for r in results if r['audit']['source']=='elite_mutation']
    assert local and any(r['audit']['source']=='global' for r in results)
    assert {r['audit']['mutation_group'] for r in local}=={'geometry','attack','timing'}
    old={action_key(row['decision']['action']) for row in rows}
    for result in local:
        assert result['audit']['parent_rounds']==[2]
        assert result['audit']['parent_score']['observed_failures']==1
        assert action_key(result['action']) not in old
        assert validate_action(result['action'],'pulp_dronet')==result['action']
    assert results==[Proposer('search',55,'pulp_dronet',prior).propose(rows,i) for i in range(16)]


def test_search_incumbent_changes_when_only_measured_results_change():
    rows=[history_row(1,loss=10.),history_row(2,loss=50.)]
    changed=copy.deepcopy(rows)
    changed[0]['pairs'][0]['perturbed']['metrics']['route_progress_percent']=5.
    sampler=Proposer('search',41,'pulp_dronet',Prior(elite_size=1))
    for index in range(20):
        a=sampler.propose(rows,index);b=sampler.propose(changed,index)
        if a['audit']['source']==b['audit']['source']=='elite_mutation':
            assert a['audit']['parent_rounds']==[2]
            assert b['audit']['parent_rounds']==[1]
            break
    else:pytest.fail('Expected a seeded local mutation')


def test_selection_pilot_can_require_first_local_attempt_without_changing_default():
    history=[history_row(1)];sampler=Proposer('search',55,'pulp_dronet')
    for index in range(20):
        normal=sampler.propose(history,index)
        if normal['audit'].get('fallback')=='fixed_probability_restart':break
    else:pytest.fail('Expected a seeded probability restart')
    required=sampler.propose(history,index,require_local_attempt=True)
    assert normal['audit']['source']=='global' and normal['audit']['require_local_attempt'] is False
    assert required['audit']['require_local_attempt'] is True
    assert required['audit']['mutation_attempts']>0
    assert required['audit']['source']=='elite_mutation'
    assert sampler.propose(history,index)==normal
    unavailable=sampler.propose([history_row(1,eligible=False)],index,require_local_attempt=True)
    assert unavailable['audit']['fallback']=='no_eligible_incumbent'
    assert unavailable['audit']['mutation_attempts']==0
    with pytest.raises(ValueError):Proposer('random',1,'kim').propose([],0,require_local_attempt=True)
    with pytest.raises(ValueError):sampler.propose([],0,require_local_attempt=1)


@pytest.mark.parametrize('bad_role,change',[
    ('clean',{'outcome':'collision'}),('perturbed',{'executed':False}),
    ('perturbed',{'outcome':'infrastructure_error'}),('perturbed',{'worker_input_exposure':None}),
    ('perturbed',{'attack_exposure':None}),
])
def test_ineligible_results_never_become_attack_search_elites(bad_role,change):
    row=history_row(1);row['pairs'][0][bad_role].update(change)
    result=Proposer('search',2,'pulp_dronet').propose([row],1)
    assert result['audit']['eligible_elite_count']==0
    assert result['audit']['fallback']=='no_eligible_incumbent'


def clearance(row, clean, attacked):
    row['pairs'][0]['clean']['metrics']['minimum_obstacle_clearance_m']=clean
    row['pairs'][0]['perturbed']['metrics']['minimum_obstacle_clearance_m']=attacked
    return row


def selected_parent(rows):
    result=Proposer('search',55,'pulp_dronet',Prior(elite_size=1)).propose(rows,1,require_local_attempt=True)
    assert result['audit']['source']=='elite_mutation'
    return result['audit']


def test_clearance_loss_guides_search_when_both_candidates_complete_with_same_progress():
    rows=[clearance(history_row(1,failed=False,loss=0),.8,.7),
          clearance(history_row(2,failed=False,loss=0),.8,.4)]
    before=copy.deepcopy(rows); audit=selected_parent(rows)
    assert audit['parent_rounds']==[2]
    score=audit['parent_score']
    assert score['ranking_version']==RANKING_VERSION and score['metric_coverage']=='complete'
    assert score['normalized_risk_proxy']==pytest.approx(1.6)
    assert score['components']['progress_loss']['mean_positive_normalized_loss']==0
    assert score['components']['clearance_loss']['normalization_scale']==.25
    assert rows==before
    changed=copy.deepcopy(rows)
    changed[0]['pairs'][0]['perturbed']['metrics']['minimum_obstacle_clearance_m']=.1
    assert selected_parent(changed)['parent_rounds']==[1]


def test_failure_priority_is_unchanged_even_with_large_clearance_loss_on_success():
    rows=[clearance(history_row(1,failed=False,loss=90),3.,0.),
          clearance(history_row(2,failed=True,loss=0),.5,.8)]
    audit=selected_parent(rows)
    assert audit['parent_rounds']==[2] and audit['parent_score']['observed_failures']==1
    assert audit['parent_score']['normalized_risk_proxy']==0
    assert audit['parent_score']['mean_clearance_loss_m']==pytest.approx(-.3)


def test_risk_components_average_observed_pairs_and_do_not_reward_repeat_count():
    rows=[clearance(history_row(1,loss=0,seed=1),.8,.4),
          clearance(history_row(2,loss=20,seed=1),.4,.5)]
    score=selected_parent(rows)['parent_score']
    assert score['eligible_pairs']==2 and score['normalized_risk_proxy']==pytest.approx(.9)
    assert score['mean_progress_loss']==10 and score['mean_clearance_loss_m']==pytest.approx(.15)
    for component in score['components'].values():
        assert component['paired_observations']==2 and component['missing_or_invalid_pairs']==0


@pytest.mark.parametrize('value',[None,float('nan'),float('inf'),True,'0.2'])
def test_missing_clearance_is_explicit_partial_evidence_not_a_zero_observation(value):
    rows=[clearance(history_row(1,loss=20),.7,value)]
    score=selected_parent(rows)['parent_score']
    assert score['metric_coverage']=='partial' and score['normalized_risk_proxy']==pytest.approx(.2)
    component=score['components']['clearance_loss']
    assert component['paired_observations']==0 and component['missing_or_invalid_pairs']==1
    assert component['mean_signed_loss'] is component['mean_positive_normalized_loss'] is None
    assert score['mean_clearance_loss_m'] is None


def test_no_metrics_remains_unavailable_and_does_not_override_measured_risk_tie():
    missing=history_row(1); missing['pairs'][0]['clean']['metrics']={}
    missing['pairs'][0]['perturbed']['metrics']=None
    measured=clearance(history_row(2,loss=0),.8,.8)
    assert selected_parent([missing,measured])['parent_rounds']==[2]
    score=selected_parent([missing])['parent_score']
    assert score['metric_coverage']=='unavailable' and score['normalized_risk_proxy'] is None
    assert all(v['paired_observations']==0 for v in score['components'].values())


def test_random_actions_do_not_depend_on_new_risk_measurements_and_prior_discloses_scale():
    first=[clearance(history_row(1),.8,.4)]
    second=copy.deepcopy(first); second[0]['pairs'][0]['perturbed']['metrics']['minimum_obstacle_clearance_m']=.01
    sampler=Proposer('random',31,'pulp_dronet')
    assert sampler.propose(first,2)==sampler.propose(second,2)
    specification=distribution()['search_ranking']
    assert specification['version']==RANKING_VERSION
    assert '/100' in specification['formula'] and '/0.25m' in specification['formula']
    assert 'null, not zero' in specification['aggregation']


def test_bounded_exhaustion_is_visible_without_repairing_to_easier_action(monkeypatch):
    import baseline_proposals_v2 as module
    def reject(_action,_target):raise ValueError('synthetic impossible domain')
    monkeypatch.setattr(module,'validate_action',reject)
    sampler=Proposer('random',3,'pulp_dronet',Prior(max_scene_attempts=2,max_obstacle_attempts=2))
    with pytest.raises(ProposalExhausted) as failure:sampler.propose([],0)
    assert failure.value.audit['scene_attempts']==2
    assert any('synthetic impossible domain' in reason for reason in failure.value.audit['rejections'])


@pytest.mark.parametrize('kwargs',[{'full_time_probability':0.},{'boundary_probability':.5},
                                  {'max_scene_attempts':0},{'useful_time_horizon_s':501.},
                                  {'restart_probability':float('nan')}])
def test_invalid_or_zero_support_priors_are_rejected(kwargs):
    with pytest.raises(ValueError):Prior(**kwargs)
