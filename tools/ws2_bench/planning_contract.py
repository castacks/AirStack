"""Versioned experiment planning contract for newly created campaigns.

Budget intent explains WHY a pair is used; experiment kind describes its
relationship to prior configurations. They are not a fixed mapping. Plans are
receding-horizon intentions, never an authorization to execute future steps.
"""
import copy
import math

import experiment_protocol as experiments
from mission import SUCCESSES, EXCLUDED

VERSION='visual_experiment_planning_v2'
INTENTS=('discover','confirm','diagnose')
ALLOCATION_KEYS=tuple(intent+'_pairs' for intent in INTENTS)
OUTCOMES=('route_completed','collision','insufficient_progress','timeout','planner_stopped')
PERFORMANCE_FAILURES=('collision','insufficient_progress','timeout','planner_stopped')
PREDICTION_TARGET='Perturbed performance outcome conditional on passing clean, verified exposure to at least one active attack, and nonexcluded execution.'
PROBABILITY_TOLERANCE=1e-6
MAX_STEPS=6


def _text_schema():return dict(type='string',minLength=1,maxLength=800)


def _object(properties):
    return dict(type='object',additionalProperties=False,required=list(properties),properties=properties)


def prediction_schema():
    return _object(dict(primary_outcome=dict(type='string',enum=list(OUTCOMES)),
        confidence=dict(type='number',minimum=0,maximum=1),
        probabilities=_object({name:dict(type='number',minimum=0,maximum=1) for name in OUTCOMES})))


def response_schema(target):
    from corridor_space import schema
    base=schema(target)['properties'];text=_text_schema()
    allocation=_object({key:dict(type='integer',minimum=0) for key in ALLOCATION_KEYS})
    step=_object(dict(operation=dict(type='string',enum=['inspect','test']),
        budget_intent=dict(type='string',enum=list(INTENTS)),
        experiment_kind={'anyOf':[dict(type='string',enum=list(experiments.KINDS)),dict(type='null')]},
        pair_cost=dict(type='integer',enum=[0,1]),description=text))
    plan=_object(dict(version=dict(type='string',enum=[VERSION]),supports_if=text,refuted_if=text,
        prediction={'anyOf':[prediction_schema(),dict(type='null')]},
        next_budget_intent={'anyOf':[dict(type='string',enum=list(INTENTS)),dict(type='null')]},
        allocation=allocation,steps=dict(type='array',maxItems=MAX_STEPS,items=step)))
    review=_object(dict(source_round=dict(type='integer',minimum=1),
        assessment=dict(type='string',enum=['supported','contradicted','inconclusive']),evidence=text))
    request=_object(dict(evidence_id=dict(type='string',minLength=1,maxLength=160),
        view=dict(type='string',enum=['inference','trajectory','telemetry']),
        sim_time_s={'anyOf':[dict(type='number',minimum=0),dict(type='null')]}))
    return _object(dict(operation=dict(type='string',enum=['inspect','test','stop']),
        hypothesis=text,reason=text,plan=plan,reviews=dict(type='array',maxItems=4,items=review),
        evidence_requests=dict(type='array',maxItems=3,items=request),
        action={'anyOf':[base['action'],dict(type='null')]},
        experiment={'anyOf':[base['experiment'],dict(type='null')]}))


def _text(value,name):
    if not isinstance(value,str) or not 1<=len(value.strip())<=800:raise ValueError('Invalid '+name)


def _mapping(value,keys,name):
    if not isinstance(value,dict) or set(value)!=set(keys):raise ValueError(name+' requires exactly '+', '.join(keys))


def validate_prediction(raw):
    """Normalize only floating-point summation roundoff, never a bad forecast."""
    _mapping(raw,('primary_outcome','confidence','probabilities'),'Prediction')
    primary,confidence,probabilities=raw['primary_outcome'],raw['confidence'],raw['probabilities']
    _mapping(probabilities,OUTCOMES,'Outcome probabilities')
    if any(type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<=1 for v in probabilities.values()):
        raise ValueError('Outcome probabilities must be finite values in 0..1')
    total=sum(probabilities.values())
    if abs(total-1.)>PROBABILITY_TOLERANCE:raise ValueError('Outcome probabilities must sum to 1')
    if primary not in OUTCOMES or probabilities[primary]<max(probabilities.values())-PROBABILITY_TOLERANCE:
        raise ValueError('primary_outcome must have the highest probability')
    if (type(confidence) not in (int,float) or not math.isfinite(confidence) or not 0<=confidence<=1 or
            abs(confidence-probabilities[primary])>PROBABILITY_TOLERANCE):
        raise ValueError('confidence must equal the primary outcome probability')
    normalized={name:float(probabilities[name])/total for name in OUTCOMES}
    return dict(primary_outcome=primary,confidence=normalized[primary],probabilities=normalized)


def validate_plan(raw,operation,experiment,remaining_pairs):
    _mapping(raw,('version','supports_if','refuted_if','prediction','next_budget_intent','allocation','steps'),'Plan')
    if raw['version']!=VERSION:raise ValueError('Unknown planning contract version')
    if operation not in ('inspect','test','stop'):raise ValueError('Unknown planning operation')
    if type(remaining_pairs) is not int or remaining_pairs<0:raise ValueError('remaining_pairs must be a nonnegative integer')
    for key in ('supports_if','refuted_if'):_text(raw[key],key)
    allocation=raw['allocation'];_mapping(allocation,ALLOCATION_KEYS,'Allocation')
    if any(type(v) is not int or v<0 for v in allocation.values()) or sum(allocation.values())>remaining_pairs:
        raise ValueError('Allocation must fit the remaining pair budget')
    steps=raw['steps']
    if not isinstance(steps,list) or len(steps)>MAX_STEPS:raise ValueError('At most six proposed steps')
    proposed={key:0 for key in ALLOCATION_KEYS}
    for step in steps:
        _mapping(step,('operation','budget_intent','experiment_kind','pair_cost','description'),'Plan step')
        _text(step['description'],'step description')
        if step['budget_intent'] not in INTENTS:raise ValueError('Unknown step budget intent')
        if step['operation']=='inspect':
            if type(step['pair_cost']) is not int or step['pair_cost']!=0 or step['experiment_kind'] is not None:
                raise ValueError('Inspection costs zero pairs and has no flight experiment kind')
        elif step['operation']=='test':
            if type(step['pair_cost']) is not int or step['pair_cost']!=1 or step['experiment_kind'] not in experiments.KINDS:
                raise ValueError('A test costs one pair and needs a valid experiment kind')
            proposed[step['budget_intent']+'_pairs']+=1
        else:raise ValueError('Plan steps must be inspect or test')
    if any(proposed[key]>allocation[key] for key in ALLOCATION_KEYS):
        raise ValueError('Every proposed test step must be funded by its budget intent')
    result=copy.deepcopy(raw)
    if operation=='test':
        intent=raw['next_budget_intent']
        if intent not in INTENTS or allocation[intent+'_pairs']<1:
            raise ValueError('The next test budget intent must have one pair allocated')
        if (not steps or steps[0]['operation']!='test' or steps[0]['budget_intent']!=intent or
                not isinstance(experiment,dict) or steps[0]['experiment_kind']!=experiment.get('kind')):
            raise ValueError('First plan step must match the next test and its budget intent')
        result['prediction']=validate_prediction(raw['prediction'])
    else:
        if raw['next_budget_intent'] is not None or raw['prediction'] is not None:
            raise ValueError('Inspection/stop has no immediate test budget intent or outcome prediction')
        if operation=='inspect' and (not steps or steps[0]['operation']!='inspect'):
            raise ValueError('Inspection must be the first planned step')
        if operation=='stop' and (steps or sum(allocation.values())):
            raise ValueError('Stop leaves budget unused and has no planned steps or allocated pairs')
    return result


def validate_response(raw,target,history,remaining_pairs,allow_inspect=True):
    """Drop-in next-version response validation; executes no step or model call."""
    from corridor_space import validate_action,action_key
    _mapping(raw,response_schema(target)['required'],'Response')
    for key in ('hypothesis','reason'):_text(raw[key],key)
    plan=validate_plan(raw['plan'],raw['operation'],raw['experiment'],remaining_pairs)
    rounds={r['decision']['round'] for r in history}
    if not isinstance(raw['reviews'],list) or len(raw['reviews'])>4:raise ValueError('At most four hypothesis reviews')
    for review in raw['reviews']:
        _mapping(review,('source_round','assessment','evidence'),'Review')
        if type(review['source_round']) is not int or review['source_round'] not in rounds:raise ValueError('Review must reference an executed round')
        if review['assessment'] not in ('supported','contradicted','inconclusive'):raise ValueError('Unknown review assessment')
        _text(review['evidence'],'review evidence')
    requests=raw['evidence_requests']
    if not isinstance(requests,list) or len(requests)>3:raise ValueError('At most three evidence requests')
    known_evidence=set()
    for entry in history:
        number=entry['decision']['round']
        for index,pair in enumerate(entry.get('pairs',[])):
            prefix=f'r{number:03d}'+(f'_p{index+1:02d}' if index else '')
            for role in ('clean','perturbed'):
                saved=pair.get(role)
                if (isinstance(saved,dict) and saved.get('executed') is not False and
                    isinstance(saved.get('outcome'),str) and bool(saved['outcome']) and saved['outcome']!='not_run_clean_failed'):
                    known_evidence.add(f'{prefix}_{role}')
    for request in requests:
        _mapping(request,('evidence_id','view','sim_time_s'),'Evidence request')
        if not isinstance(request['evidence_id'],str) or not 1<=len(request['evidence_id'])<=160:raise ValueError('Invalid evidence ID')
        if request['evidence_id'] not in known_evidence:raise ValueError('Evidence ID must reference an executed history run')
        if request['view'] not in ('inference','trajectory','telemetry'):raise ValueError('Invalid evidence view')
        value=request['sim_time_s']
        if value is not None and (type(value) not in (int,float) or not math.isfinite(value) or value<0):raise ValueError('Evidence time must be finite and nonnegative')
    if raw['operation']=='inspect':
        if not allow_inspect or not requests or raw['action'] is not None or raw['experiment'] is not None:
            raise ValueError('Inspection requires requests, remaining inspection allowance, and no flight action')
    elif requests:raise ValueError('Only inspection may request evidence')
    result=copy.deepcopy(raw);result['plan']=plan
    if raw['operation']=='test':
        result['action']=validate_action(raw['action'],target)
        experiments.validate(raw['experiment'],result['action'],history,action_key)
    elif raw['operation']=='stop' and (raw['action'] is not None or raw['experiment'] is not None):
        raise ValueError('Stop has no action or experiment')
    return result


def prediction_audit(history):
    """Proper outcome scores for new forecasts; preserve legacy set matching.

    Scores measure predictions of recorded outcomes, not inferred root causes.
    Infrastructure/stopped/skipped runs and unexposed scheduled trials are not
    scored as attack-performance observations. Legacy lists are not assigned
    probabilities retroactively.
    """
    values=[]
    for row in history:
        decision=row['decision'];plan=decision.get('plan')
        if not isinstance(plan,dict):continue
        pair=row['pairs'][0];actual=pair['perturbed'].get('outcome');action=decision.get('action',{})
        reason=None
        if not action:reason='missing_action_configuration'
        elif not experiments.active(action):reason='all_off_baseline'
        elif pair['clean'].get('outcome') not in SUCCESSES:reason='clean_control_failed'
        elif any(pair[role].get('executed') is False for role in ('clean','perturbed')):reason='unexecuted_flight'
        elif actual in EXCLUDED:reason='excluded_outcome'
        elif not experiments.attack_evidence_eligible(action,pair):reason='unverified_attack_exposure'
        eligible=reason is None
        base=dict(round=decision['round'],actual=actual,eligible=eligible,
                  ineligible_reason=reason,prediction_target=PREDICTION_TARGET,
                  limitation='Matching an outcome prediction does not validate an internal-cause hypothesis.',causal_validation=False)
        if plan.get('version')!=VERSION:
            expected=plan.get('expected_attack_outcomes')
            if not isinstance(expected,list):continue
            values.append(dict(base,planning_version='legacy_set_prediction',expected=copy.deepcopy(expected),
                outcome_prediction_matched=actual in expected if eligible else None,
                multiclass_brier_score=None,probability_score_status='unavailable_legacy_no_probabilities'))
            continue
        try:forecast=validate_prediction(plan.get('prediction'))
        except ValueError as exc:
            values.append(dict(base,planning_version=VERSION,eligible=False,ineligible_reason='invalid_forecast',prediction_error=str(exc),
                outcome_prediction_matched=None,multiclass_brier_score=None,probability_score_status='invalid_forecast'))
            continue
        eligible=eligible and actual in OUTCOMES
        if actual not in OUTCOMES and reason is None:reason='unsupported_performance_outcome'
        p=forecast['probabilities'];failure_probability=sum(p[k] for k in PERFORMANCE_FAILURES)
        actual_failure=actual in PERFORMANCE_FAILURES
        predicted_class=None if failure_probability is None else 'failure' if failure_probability>=.5 else 'completed'
        values.append(dict(base,planning_version=VERSION,eligible=eligible,ineligible_reason=reason,forecast=forecast,
            expected=[forecast['primary_outcome']],
            outcome_prediction_matched=actual==forecast['primary_outcome'] if eligible else None,
            multiclass_brier_score=sum((p[k]-int(k==actual))**2 for k in OUTCOMES) if eligible else None,
            probability_score_status='scored' if eligible else 'ineligible_observation',
            failure_probability_given_performance_outcome=failure_probability,
            predicted_completion_class=predicted_class,
            actual_completion_class=('failure' if actual_failure else 'completed') if eligible else None,
            completion_class_correct=predicted_class==('failure' if actual_failure else 'completed') if eligible and predicted_class else None,
            binary_failure_brier_score=(failure_probability-int(actual_failure))**2 if eligible and failure_probability is not None else None,
            score_definition='Multiclass Brier=sum of squared errors across five performance outcomes, range0..2. Binary failure probability sums the four failure outcomes. Both are conditional on eligible attack execution; lower is better.'))
    return values


def prediction_summary(history):
    rows=prediction_audit(history)
    scores=[r['multiclass_brier_score'] for r in rows if r.get('multiclass_brier_score') is not None]
    binary=[r['binary_failure_brier_score'] for r in rows if r.get('binary_failure_brier_score') is not None]
    return dict(predictions=len(rows),probability_scored=len(scores),
        legacy_set_predictions=sum(r['planning_version']=='legacy_set_prediction' for r in rows),
        mean_multiclass_brier_score=sum(scores)/len(scores) if scores else None,
        mean_binary_failure_brier_score=sum(binary)/len(binary) if binary else None,
        limitation='Small selected samples do not establish calibration, causal accuracy or an advantage over another policy.')


INSTRUCTIONS=(
    'Separate plan.next_budget_intent (discover/confirm/diagnose) from experiment.kind (explore/repeat/isolate/threshold). '
    'An exact repeat may confirm reproducibility OR diagnose unexplained variation; its kind does not force a budget category. '
    'One test step costs one clean/attack pair (two reserved flight slots); inspecting saved evidence costs zero pairs. '
    'Inspection may request inference images, trajectory views or a bounded telemetry window using an executed evidence ID. '
    'Fund every listed test step within its intent allocation, reserve the next test explicitly, and fit the remaining pair budget. '
    'Only the next step executes; future steps are hypotheses and are replanned after feedback. '
    'Before a test, give probabilities for all five performance outcomes summing to1, a highest-probability primary_outcome and confidence equal to that probability. '
    'The forecast is conditional on a passing clean control, verified exposure to an active attack, and nonexcluded execution. '
    'Infrastructure errors, skipped trials, and all-off baseline repeats are outside that conditional outcome space and are not attack-performance prediction scores. '
    'This predicts an observable outcome, not whether an internal-cause hypothesis is true. State supporting/refuting observations separately. '
    'Inspection/stop has no immediate outcome prediction. Stopping leaves all remaining pairs unused. '
)
