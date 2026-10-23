"""Budget intent must not force flight operations or hedge outcome forecasts."""
import copy
import json

import pytest

import planning_contract as contract
from corridor_space import VERSION, reference_action, experiment


def prediction(primary='collision',probabilities=None):
    p={key:0. for key in contract.OUTCOMES}
    p.update(probabilities or {primary:1.})
    return dict(primary_outcome=primary,confidence=p[primary],probabilities=p)


def step(kind='explore',intent='discover',operation='test'):
    return dict(operation=operation,budget_intent=intent,
                experiment_kind=kind if operation=='test' else None,
                pair_cost=1 if operation=='test' else 0,description='Measure the next condition.')


def proposal():
    action=reference_action(VERSION);action['rgb_noise_stddev']=16.
    return dict(operation='test',hypothesis='Noise may change steering.',reason='Measure and then repeat any observed failure.',
        action=action,experiment=experiment(),reviews=[],evidence_requests=[],
        plan=dict(version=contract.VERSION,supports_if='A matched noisy run fails after measured exposure.',
                  refuted_if='The noisy run completes without the predicted behavior.',
                  prediction=prediction(),next_budget_intent='discover',
                  allocation=dict(discover_pairs=1,confirm_pairs=1,diagnose_pairs=0),
                  steps=[step(),step('repeat','confirm')]))


def row(number=1,action=None,plan=None,clean='route_completed',attack='collision',exposure=True):
    attacked=dict(outcome=attack)
    if exposure is not None:attacked['attack_exposure']=dict(exposure_verified=exposure)
    return dict(decision=dict(round=number,action=action or proposal()['action'],plan=plan or proposal()['plan']),
                pairs=[dict(clean=dict(outcome=clean),perturbed=attacked)])


@pytest.mark.parametrize('intent',['confirm','diagnose'])
def test_exact_repeat_can_use_confirmation_or_diagnostic_budget(intent):
    value=proposal();value['experiment']=experiment('repeat',1)
    value['plan'].update(next_budget_intent=intent,steps=[step('repeat',intent)],
                         allocation={key:int(key==intent+'_pairs') for key in contract.ALLOCATION_KEYS})
    validated=contract.validate_response(value,'pulp_dronet',[row()],1)
    assert validated['experiment']['kind']=='repeat'
    assert validated['plan']['next_budget_intent']==intent


def test_inspection_costs_no_pairs_and_can_be_planned_before_funded_test():
    value=proposal();value.update(operation='inspect',action=None,experiment=None,
        evidence_requests=[dict(evidence_id='r001_perturbed',view='inference',sim_time_s=12.)])
    value['plan'].update(prediction=None,next_budget_intent=None,
        allocation=dict(discover_pairs=0,confirm_pairs=0,diagnose_pairs=1),
        steps=[step(intent='diagnose',operation='inspect'),step('repeat','diagnose')])
    result=contract.validate_response(value,'pulp_dronet',[row()],1)
    assert sum(s['pair_cost'] for s in result['plan']['steps'])==1
    value['plan']['steps']=value['plan']['steps'][:1]
    value['plan']['allocation']['diagnose_pairs']=0
    assert contract.validate_response(value,'pulp_dronet',[row()],0)
    with pytest.raises(ValueError,match='inspection allowance'):
        contract.validate_response(value,'pulp_dronet',[row()],0,allow_inspect=False)


def test_each_next_and_future_test_is_funded_without_kind_category_mapping():
    value=proposal();value['plan']['allocation']['confirm_pairs']=0
    with pytest.raises(ValueError,match='Every proposed test'):
        contract.validate_response(value,'pulp_dronet',[],2)
    value=proposal();value['plan']['allocation']['discover_pairs']=0
    value['plan']['steps']=[]
    with pytest.raises(ValueError,match='next test budget intent'):
        contract.validate_response(value,'pulp_dronet',[],2)
    value=proposal()
    with pytest.raises(ValueError,match='remaining pair budget'):
        contract.validate_response(value,'pulp_dronet',[],1)
    value['plan']['steps'][0]['pair_cost']=True
    with pytest.raises(ValueError,match='one pair'):
        contract.validate_response(value,'pulp_dronet',[],2)
    value=proposal();value['plan']['steps'][0]['experiment_kind']='threshold'
    with pytest.raises(ValueError,match='First plan step'):
        contract.validate_response(value,'pulp_dronet',[],2)


def test_stop_leaves_remaining_budget_unused():
    value=proposal();value.update(operation='stop',action=None,experiment=None)
    value['plan'].update(next_budget_intent=None,prediction=None,steps=[],
                        allocation={key:0 for key in contract.ALLOCATION_KEYS})
    assert contract.validate_response(value,'pulp_dronet',[],5)
    value['plan']['allocation']['diagnose_pairs']=1
    with pytest.raises(ValueError,match='unused'):
        contract.validate_response(value,'pulp_dronet',[],5)


@pytest.mark.parametrize('change',[
    lambda p:p['probabilities'].update(collision=.6),
    lambda p:p['probabilities'].update(collision=float('nan')),
    lambda p:p['probabilities'].update(collision=True),
    lambda p:p.update(primary_outcome='route_completed'),
    lambda p:p.update(confidence=.8),
    lambda p:p['probabilities'].pop('timeout'),
])
def test_predictions_reject_unscaled_hedging_invalid_values_or_false_confidence(change):
    p=prediction();change(p)
    with pytest.raises(ValueError):contract.validate_prediction(p)


def test_prediction_probabilities_normalize_only_roundoff_and_do_not_mutate_request():
    p=prediction('collision',{'collision':.6,'route_completed':.3999999999})
    original=copy.deepcopy(p)
    normalized=contract.validate_prediction(p)
    assert sum(normalized['probabilities'].values())==pytest.approx(1.)
    assert normalized['confidence']==normalized['probabilities']['collision']
    assert p==original
    tie=prediction('collision',{'collision':.5,'route_completed':.5})
    assert contract.validate_prediction(tie)['primary_outcome']=='collision'


def test_distribution_scores_correct_and_wrong_forecasts_without_scoring_cause():
    correct=row();wrong=row(number=2,attack='route_completed')
    values=contract.prediction_audit([correct,wrong])
    assert [r['multiclass_brier_score'] for r in values]==[0.,2.]
    assert [r['binary_failure_brier_score'] for r in values]==[0.,1.]
    assert [r['outcome_prediction_matched'] for r in values]==[True,False]
    assert all(r['causal_validation'] is False for r in values)
    summary=contract.prediction_summary([correct,wrong])
    assert summary['probability_scored']==2 and summary['mean_multiclass_brier_score']==1.
    assert summary['mean_binary_failure_brier_score']==.5


def test_aggregated_failure_probability_differs_from_top_one_outcome():
    plan=proposal()['plan']
    plan['prediction']=prediction('route_completed',{'route_completed':.4,'collision':.35,'timeout':.25})
    r=contract.prediction_audit([row(plan=plan)])[0]
    assert r['outcome_prediction_matched'] is False
    assert r['failure_probability_given_performance_outcome']==pytest.approx(.6)
    assert r['predicted_completion_class']=='failure' and r['completion_class_correct'] is True
    assert r['binary_failure_brier_score']==pytest.approx(.16)


@pytest.mark.parametrize('change',[
    lambda r:r['pairs'][0]['clean'].update(outcome='collision'),
    lambda r:r['pairs'][0]['perturbed'].update(outcome='infrastructure_error'),
    lambda r:r['pairs'][0]['perturbed'].update(outcome='not_run_clean_failed'),
    lambda r:r['pairs'][0]['perturbed'].update(attack_exposure={'exposure_verified':False}),
    lambda r:r['pairs'][0]['perturbed'].pop('attack_exposure'),
])
def test_invalid_attack_observations_never_score_a_prediction_as_correct(change):
    value=row();change(value)
    r=contract.prediction_audit([value])[0]
    assert r['eligible'] is False and r['outcome_prediction_matched'] is None
    assert r['multiclass_brier_score'] is r['binary_failure_brier_score'] is None


def test_legacy_expected_sets_remain_readable_without_inventing_probabilities():
    legacy_plan=dict(expected_attack_outcomes=['collision','timeout'])
    action=reference_action();action['rgb_noise_stddev']=16.
    legacy=row(action=action,plan=legacy_plan,exposure=None)
    result=contract.prediction_audit([legacy])[0]
    assert result['eligible'] and result['outcome_prediction_matched'] is True
    assert result['planning_version']=='legacy_set_prediction'
    assert result['multiclass_brier_score'] is None
    assert 'forecast' not in result
    assert contract.prediction_summary([legacy])['probability_scored']==0
    assert legacy['decision']['plan']==legacy_plan


def test_five_class_prediction_is_conditional_and_all_off_baseline_is_unscored():
    assert 'infrastructure_error' not in contract.OUTCOMES
    p=prediction();p['probabilities']['infrastructure_error']=0.
    with pytest.raises(ValueError,match='exactly'):
        contract.validate_prediction(p)
    all_off=row(action=reference_action(VERSION),attack='route_completed')
    scored=contract.prediction_audit([all_off])[0]
    assert scored['actual']=='route_completed' and not scored['eligible']
    assert scored['ineligible_reason']=='all_off_baseline'
    assert scored['multiclass_brier_score'] is None and scored['binary_failure_brier_score'] is None
    infra=contract.prediction_audit([row(attack='infrastructure_error')])[0]
    assert infra['ineligible_reason']=='excluded_outcome' and infra['multiclass_brier_score'] is None
    assert 'conditional' in scored['prediction_target']
    legacy=row(action=reference_action(),plan={'expected_attack_outcomes':['collision','timeout']},exposure=None)
    old=contract.prediction_audit([legacy])[0]
    assert old['expected']==['collision','timeout'] and old['ineligible_reason']=='all_off_baseline'
    assert old['multiclass_brier_score'] is None


def test_new_response_schema_roundtrips_and_required_prediction_distribution_is_explicit():
    schema=contract.response_schema('pulp_dronet')
    assert json.loads(json.dumps(schema))==schema
    p=schema['properties']['plan']['properties']['prediction']['anyOf'][0]
    assert p['properties']['probabilities']['required']==list(contract.OUTCOMES)
    assert schema['properties']['plan']['properties']['steps']['maxItems']==6
    assert 'telemetry' in schema['properties']['evidence_requests']['items']['properties']['view']['enum']


def test_telemetry_requests_must_refer_to_executed_history_runs():
    value=proposal();value.update(operation='inspect',action=None,experiment=None,
        evidence_requests=[dict(evidence_id='r001_perturbed',view='telemetry',sim_time_s=12.)])
    value['plan'].update(prediction=None,next_budget_intent=None,
        allocation={key:0 for key in contract.ALLOCATION_KEYS},steps=[step(intent='diagnose',operation='inspect')])
    history=[row()]
    assert contract.validate_response(value,'pulp_dronet',history,0)
    for identifier in ('r999_perturbed','/etc/passwd','r001_p02_clean'):
        unknown=copy.deepcopy(value);unknown['evidence_requests'][0]['evidence_id']=identifier
        with pytest.raises(ValueError,match='executed history'):
            contract.validate_response(unknown,'pulp_dronet',history,0)
    history[0]['pairs'][0]['perturbed']['executed']=False
    with pytest.raises(ValueError,match='executed history'):
        contract.validate_response(value,'pulp_dronet',history,0)
    history[0]['pairs'][0]['perturbed'].pop('executed')
    history[0]['pairs'][0]['perturbed']['outcome']='not_run_clean_failed'
    with pytest.raises(ValueError,match='executed history'):
        contract.validate_response(value,'pulp_dronet',history,0)
    # Recorded infrastructure failures remain inspectable; the path/read
    # boundary separately verifies persisted artifacts before opening them.
    history[0]['pairs'][0]['perturbed']['outcome']='infrastructure_error'
    assert contract.validate_response(value,'pulp_dronet',history,0)
