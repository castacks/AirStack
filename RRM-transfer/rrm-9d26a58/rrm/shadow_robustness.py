"""Matched-tape offline checking ablations; never an executable robot controller.

All arms audit the entire fixed tape, even after a refusal or completion claim.
Consequently these results do not predict changed closed-loop trajectories/recovery.
"""
from dataclasses import dataclass, replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import time
from types import FunctionType

import pydantic

from .contracts import CapabilityDeclaration, Truth
from .cosmos_reason2 import CosmosReasoningInput, parse_cosmos_candidate
from .intent_alignment import TaskIntentBinding
from .schema import Predicate
from .state_contracts import FactKey, StateSnapshot
from .task_contracts import TaskRequest
from .shadow_robustness_fixtures import specification

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = 'rrm-shadow-report/v2'


@dataclass(frozen=True)
class Arm:
    name: str
    resolved_identity: bool
    resolved_effect: bool
    bound_intent: bool = False


ARMS = (Arm('context_and_effect',True,True), Arm('raw_identity_ablation',False,True),
        Arm('status_effect_ablation',True,False), Arm('both_ablated',False,False),
        Arm('bound_context_and_effect',True,True,True))


def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def _context(payload):
    capability=payload['capabilities']
    return CosmosReasoningInput(task=TaskRequest.model_validate(payload['task']),
        snapshot=StateSnapshot.model_validate(payload['snapshot']),
        capabilities=CapabilityDeclaration(**{k:frozenset(v) if isinstance(v,list) else v
                                              for k,v in capability.items()}),
        now_monotonic_s=payload['now_monotonic_s'])


def _raw_positive_entities(context):
    # Exact pre-fix eligibility ablation. No changes to the imported parser globals.
    return frozenset(item.key.subject for item in context.snapshot.evidence
                     if item.is_fresh(context.now_monotonic_s) and item.truth is Truth.TRUE)


def _ablation_parser():
    private=dict(parse_cosmos_candidate.__globals__,_fresh_entities=_raw_positive_entities)
    parser=FunctionType(parse_cosmos_candidate.__code__,private,'shadow_identity_ablation',
                        parse_cosmos_candidate.__defaults__,parse_cosmos_candidate.__closure__)
    parser.__kwdefaults__=dict(parse_cosmos_candidate.__kwdefaults__ or {})
    return parser


def _arm_decision(inputs, arm):
    """No adjudication labels reach this function, only shared replay inputs."""
    context=_context(inputs['context'])
    if arm.bound_intent:
        context=replace(context,intent_binding=TaskIntentBinding.model_validate(inputs['intent_binding']))
    raw=inputs['raw_response']
    alignment='NOT_CHECKED'
    if inputs['stop_active']:
        status='COMMON_STOP'
    elif not inputs['permission']:
        status='COMMON_PERMISSION_DENIED'
    elif inputs['synthetic_proposal_delay_s']>inputs['proposal_deadline_s']:
        status='COMMON_DEADLINE'
    elif raw is None:
        status='MISSING_PROPOSAL'
    else:
        parser=parse_cosmos_candidate if arm.resolved_identity else _ablation_parser()
        candidate=parser(raw,context)
        status=candidate.status.value
        alignment=candidate.intent_alignment
    accepted=status=='ACCEPTED'
    post=StateSnapshot.model_validate(inputs['post_snapshot'])
    effect=post.resolve(FactKey.model_validate(inputs['task_effect']),
                        now_monotonic_s=inputs['post_now_monotonic_s'])
    claim=accepted and (effect is Truth.TRUE if arm.resolved_effect
                        else inputs['backend_feedback']['success'] is True)
    return dict(proposal_status=status,proposal_accepted=accepted,completion_claim=claim,
                observed_task_effect=effect.value,intent_alignment=alignment,execution_dispatch=False)


def _ratio(numerator,denominator):
    return dict(numerator=numerator,denominator=denominator,
                rate=numerator/denominator if denominator else None)


def evaluate(corpus):
    """Evaluate the entire pinned development corpus; no selected intersections."""
    if canonical(corpus)!=canonical(specification()):
        raise ValueError('unrecognized_or_modified_corpus')
    ids=[case['case_id'] for case in corpus['cases']]
    if len(set(ids))!=len(ids) or any(not case_id for case_id in ids) \
            or any(not 1<=len(case['steps'])<=corpus['budgets']['max_steps_per_case'] for case in corpus['cases']):
        raise ValueError('invalid_frozen_schedule')
    before=digest(corpus)
    rows={arm.name:[] for arm in ARMS}
    for case in corpus['cases']:
        for index,step in enumerate(case['steps']):
            inputs={key:value for key,value in step.items() if key!='adjudication'}
            inputs['proposal_deadline_s']=corpus['budgets']['proposal_deadline_s']
            # Author-supplied requested effect already exists in the shared tape.
            # No adjudication or model-generated goal selects the binding target.
            ctx=_context(step['context'])
            effect=step['task_effect']
            inputs['intent_binding']=TaskIntentBinding(
                binding_id=case['case_id']+'/authored-intent',source_ref='authored-requested-task-effect/v1',
                task_id=ctx.task.task_id,task_revision=ctx.task.revision,
                state_revision=ctx.snapshot.revision,capability_revision=ctx.capabilities.revision,
                snapshot_id=ctx.snapshot.snapshot_id,episode_id=ctx.snapshot.episode_id,
                embodiment_id=ctx.capabilities.embodiment_id,
                grounded_goal=Predicate(name=effect['predicate'],subject=effect['subject'],obj=effect['obj']),
                allowed_targets=(effect['obj'],),allowed_operations=('NAVIGATE_TO',),
            ).model_dump(mode='json')
            input_hash=digest(inputs)
            for arm in ARMS:
                # Fresh JSON copy isolates arms and preserves the same complete tape.
                decision=_arm_decision(json.loads(canonical(inputs)),arm)
                rows[arm.name].append(dict(case_id=case['case_id'],step=index,
                    input_sha256=input_hash,adjudication=step['adjudication'],**decision))
    if digest(corpus)!=before:
        raise ValueError('corpus_mutated')
    summaries={}
    for arm in ARMS:
        data=rows[arm.name]
        known_valid=[r for r in data if r['adjudication']['proposal_permitted'] is True]
        known_invalid=[r for r in data if r['adjudication']['proposal_permitted'] is False]
        false_goals=[r for r in data if r['adjudication']['actual_goal'] is False]
        unknown_goals=[r for r in data if r['adjudication']['actual_goal'] is None]
        true_goals=[r for r in data if r['adjudication']['actual_goal'] is True]
        summaries[arm.name]=dict(audited_steps=len(data),audited_cases=len(corpus['cases']),
            accepted_proposals=sum(r['proposal_accepted'] for r in data),
            invalid_proposal_acceptance=_ratio(sum(r['proposal_accepted'] for r in known_invalid),len(known_invalid)),
            valid_proposal_refusal=_ratio(sum(not r['proposal_accepted'] for r in known_valid),len(known_valid)),
            unknown_proposal_labels=sum(r['adjudication']['proposal_permitted'] is None for r in data),
            false_completion_claims=_ratio(sum(r['completion_claim'] for r in false_goals),len(false_goals)),
            unsupported_completion_claims=_ratio(sum(r['completion_claim'] for r in unknown_goals),len(unknown_goals)),
            true_goal_claim_coverage=_ratio(sum(r['completion_claim'] for r in true_goals),len(true_goals)),
            observed_effect_unknown=sum(r['observed_task_effect']=='UNKNOWN' for r in data),
            replay_nonclaim_then_claim_cases=sum(
                any(not a['completion_claim'] and b['completion_claim']
                    for a,b in zip(case_rows,case_rows[1:]))
                for case in corpus['cases']
                for case_rows in [[r for r in data if r['case_id']==case['case_id']]]),
            rows=data)
    reference=rows[ARMS[0].name]
    pairing=[(r['case_id'],r['step'],r['input_sha256']) for r in reference]
    if any([(r['case_id'],r['step'],r['input_sha256']) for r in values]!=pairing for values in rows.values()):
        raise ValueError('arm_input_mismatch')
    return dict(schema=SCHEMA,corpus_sha256=before,scope='AUTHORED_SHADOW_CHECKING_ABLATIONS',
        execution_dispatch=False,performance_claim_authorized=False,closed_loop_counterfactual=False,
        arm_definitions=[dict(name=a.name,resolved_identity=a.resolved_identity,resolved_effect=a.resolved_effect,
                              bound_intent=a.bound_intent) for a in ARMS],
        identities=corpus['identities'],budgets=corpus['budgets'],input_pairing=pairing,arms=summaries)


def _source_identity():
    paths=sorted((ROOT/'rrm').glob('*.py'))+[ROOT/'scripts/shadow_robustness_campaign.py']
    return {p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def _write(path,value):
    with path.open('xb') as stream:
        stream.write(canonical(value)+b'\n')


def run_campaign(output):
    output=Path(output)
    output.mkdir(parents=True,exist_ok=False)
    source=_source_identity()
    started=time.monotonic()
    corpus=specification()
    report=evaluate(corpus)
    _write(output/'corpus.json',corpus)
    _write(output/'report.json',report)
    _write(output/'run.json',dict(written_at_utc=datetime.now(timezone.utc).isoformat(),
        python_version=platform.python_version(),pydantic_version=pydantic.__version__,
        instrument_elapsed_s=time.monotonic()-started,latency_scope='OFFLINE_INSTRUMENT_NOT_MODEL_OR_ROBOT'))
    if source!=_source_identity():
        raise ValueError('source_changed_during_campaign')
    _write(output/'manifest.json',dict(schema='rrm-shadow-manifest/v1',source=source,
        artifacts={name:hashlib.sha256((output/name).read_bytes()).hexdigest()
                   for name in ('corpus.json','report.json','run.json')}))
    return report


def verify_campaign(output):
    output=Path(output)
    manifest_bytes=(output/'manifest.json').read_bytes()
    manifest=json.loads(manifest_bytes)
    if manifest['schema']!='rrm-shadow-manifest/v1' or manifest['source']!=_source_identity():
        raise ValueError('source_or_schema_mismatch')
    names={'corpus.json','report.json','run.json'}
    if set(manifest['artifacts'])!=names or {p.name for p in output.iterdir()}!=names|{'manifest.json'}:
        raise ValueError('artifact_inventory_mismatch')
    bound={name:(output/name).read_bytes() for name in names}
    if any(hashlib.sha256(bound[n]).hexdigest()!=manifest['artifacts'][n] for n in names):
        raise ValueError('artifact_checksum_mismatch')
    expected=evaluate(json.loads(bound['corpus.json']))
    if canonical(json.loads(bound['report.json']))!=canonical(expected):
        raise ValueError('report_reconstruction_mismatch')
    runtime=json.loads(bound['run.json'])
    elapsed=runtime['instrument_elapsed_s']
    if type(elapsed) not in (float,int) or not math.isfinite(elapsed) or elapsed<0:
        raise ValueError('invalid_instrument_duration')
    if (output/'manifest.json').read_bytes()!=manifest_bytes or _source_identity()!=manifest['source'] \
            or any((output/n).read_bytes()!=bound[n] for n in names):
        raise ValueError('inputs_changed_during_verification')
    return dict(valid=True,corpus_sha256=expected['corpus_sha256'],
                cases=len(specification()['cases']),steps=len(expected['input_pairing']))
