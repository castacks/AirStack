"""Let the agent propose a bounded budget; freeze it for matched baselines."""
import argparse
import hashlib
import json
from pathlib import Path

SYSTEM = (
    'Propose the smallest useful flight budget for an authorized simulated UAV camera robustness study. '
    'All text must be English. One experiment is a clean+perturbed pair costing TWO flights. '
    'Allocate pairs among discovery, exact confirmation and factor isolation/diagnosis. '
    'This allocation is an initial intention, not immutable sub-budgets. Within the fixed total cap, '
    'the test planner may reallocate unused pairs after inconclusive exposure or a failed clean control. '
    'Reserve confirmation when an eligible candidate exists; if none exists, allow simpler-baseline discovery '
    'or a single-factor exposure check rather than stopping solely because the discovery allocation is spent. '
    'Reserve at least one confirmation pair. Choose a cap within the supplied resource ceiling; '
    'do not treat the ceiling as a target. The SAME chosen cap, action space, seeds and evidence '
    'access will apply independently to Random, Search and the LLM. Do not assume the LLM will win. '
    'Counts include confirmation/diagnosis flights. No defense testing in this phase. '
    'The cap cannot grow in response to an unfavorable comparative result. The agent may stop early; '
    'unused slots, infrastructure attempts, elapsed GPU time and LLM costs must remain visible. '
)


def validate(raw, ceiling):
    keys={'flight_cap','discovery_pairs','confirmation_pairs','diagnosis_pairs','reason','stop_rule'}
    if not isinstance(raw,dict) or set(raw)!=keys:raise ValueError('Invalid budget proposal fields')
    if type(ceiling) is not int or ceiling<4 or ceiling%2:raise ValueError('Ceiling must be even and at least four flights')
    if type(raw['flight_cap']) is not int or not 4<=raw['flight_cap']<=ceiling or raw['flight_cap']%2:
        raise ValueError('Flight cap must be even, within ceiling, and cover confirmation')
    for key in ('discovery_pairs','confirmation_pairs','diagnosis_pairs'):
        if type(raw[key]) is not int or raw[key]<0:raise ValueError('Pair allocations must be nonnegative integers')
    if raw['confirmation_pairs']<1 or raw['discovery_pairs']<1:raise ValueError('Reserve discovery and confirmation')
    if sum(raw[k] for k in ('discovery_pairs','confirmation_pairs','diagnosis_pairs'))*2!=raw['flight_cap']:
        raise ValueError('Pair allocation must equal the flight cap')
    if any(not isinstance(raw[k],str) or not raw[k].strip() or len(raw[k])>1200 for k in ('reason','stop_rule')):
        raise ValueError('Supply concise reason and stopping rule')
    return raw


def propose(provider, target, ceiling, output):
    from corridor_space import bounds, CAMERA_VERSION
    from episode import atomic
    if type(ceiling) is not int or ceiling<4 or ceiling%2:raise ValueError('Ceiling must be even and at least four flights')
    output=Path(output)
    if output.exists():return load(output,target,ceiling)
    schema=dict(type='object',additionalProperties=False,
        required=['flight_cap','discovery_pairs','confirmation_pairs','diagnosis_pairs','reason','stop_rule'],
        properties={'flight_cap':dict(type='integer',minimum=4,maximum=ceiling),
            **{k:dict(type='integer',minimum=0) for k in ('discovery_pairs','confirmation_pairs','diagnosis_pairs')},
            'reason':dict(type='string'),'stop_rule':dict(type='string')})
    context=dict(target=target,resource_ceiling_flights_per_method=ceiling,action_space=bounds(target,CAMERA_VERSION),
                 prior_comparative_results=None,unit='actual clean and perturbed flight attempts; two per test pair')
    choice=provider.complete('experiment_budget',SYSTEM,context,schema,lambda raw:validate(raw,ceiling))
    result=dict(schema='shared_camera_budget_v1',target=target,resource_ceiling=ceiling,
                action_version=CAMERA_VERSION,proposal=choice,provider=provider.metadata(),
                llm_call_id=provider.last_call_id)
    result['plan_id']=hashlib.sha256(json.dumps(result,sort_keys=True,allow_nan=False).encode()).hexdigest()
    atomic(output,result)
    return result


def load(path,target,ceiling):
    raw=json.loads(Path(path).read_text());payload={k:v for k,v in raw.items() if k!='plan_id'}
    if raw.get('plan_id')!=hashlib.sha256(json.dumps(payload,sort_keys=True,allow_nan=False).encode()).hexdigest():
        raise ValueError('Budget plan checksum mismatch')
    if raw.get('schema')!='shared_camera_budget_v1' or raw.get('target')!=target:raise ValueError('Budget target/schema mismatch')
    validate(raw['proposal'],ceiling)
    return raw


def campaign_objective(objective, plan, previous=None):
    """Preserve the exact saved budget suffix when resuming older campaigns."""
    prefix = objective + ' Shared frozen budget plan ' + plan['plan_id'] + '. '
    current = prefix + 'Keep its total cap fixed; explain any reallocation of unused pairs.'
    legacy = prefix + 'Follow its reserved confirmation/diagnosis allocation.'
    # Only the known generated wording is compatible. Changes to the user's
    # objective or plan ID still fail the campaign's exact configuration check.
    if previous and previous.get('study_objective') == legacy:
        return legacy
    return current


if __name__=='__main__':
    from agent_policies import provider_from_name
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--planner',required=True,choices=['mononav','kim','pulp_dronet'])
    p.add_argument('--ceiling',type=int,required=True,help='Resource ceiling per method, including paired controls')
    p.add_argument('--provider',choices=['codex','claude'],default='codex')
    p.add_argument('--output',required=True,type=Path)
    args=p.parse_args();provider=provider_from_name(args.provider,audit_dir=args.output.parent/'budget_llm_calls')
    provider.check_auth()
    print(json.dumps(propose(provider,args.planner,args.ceiling,args.output),indent=2))
