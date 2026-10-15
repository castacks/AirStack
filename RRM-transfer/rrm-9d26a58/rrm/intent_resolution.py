"""Restricted independent semantic reference, not a general language grounder.

Only a declared grammar and caller-scoped candidate set are supported. UNKNOWN
competitors cause clarification. No model text, expected labels or geometry enters.
"""
from dataclasses import dataclass
import math
import re

from .contracts import Truth
from .cosmos_reason2 import CosmosReasoningInput
from .intent_alignment import TaskIntentBinding
from .schema import Predicate
from .state_contracts import FactKey

RESOLVER_REVISION='restricted-navigation-reference/v1'


@dataclass(frozen=True)
class IntentResolution:
    status: str
    reasons: tuple[str, ...]
    binding: TaskIntentBinding | None = None


def resolve_task_intent(context: CosmosReasoningInput, *, candidate_entity_ids: tuple[str, ...],
                        scope_ref: str) -> IntentResolution:
    """Resolve only within an explicit finite caller-owned scope; never authorize."""
    if not scope_ref.strip() or not candidate_entity_ids or len(set(candidate_entity_ids))!=len(candidate_entity_ids) \
            or any(not isinstance(x,str) or not x.strip() or x=='$self' for x in candidate_entity_ids):
        raise ValueError('explicit unique nonempty candidate scope and provenance required')
    if context.intent_binding is not None:
        raise ValueError('independent resolver does not consume an existing intent binding')
    if not math.isfinite(context.now_monotonic_s) or context.now_monotonic_s<0:
        raise ValueError('finite reasoning clock required')
    if context.task.requested_embodiment_id!=context.capabilities.embodiment_id:
        return IntentResolution('NEEDS_CLARIFICATION',('embodiment_context_mismatch',))
    text=context.task.objective.strip()
    if re.fullmatch(r'inspect(?:\s+.+)?\.?',text,re.IGNORECASE):
        return IntentResolution('UNSUPPORTED',('inspection_not_supported_by_reference_resolver',))
    selected=re.fullmatch(r'(?:navigate to|go to|approach)\s+(?:the\s+)?(blue|orange|red|green)\s+marker\.?',
                          text,re.IGNORECASE)
    explicit=re.fullmatch(r'(?:navigate to|go to|approach)\s+([A-Za-z0-9_-]+)\.?',text,re.IGNORECASE)
    if selected is None and explicit is None:
        return IntentResolution('NEEDS_CLARIFICATION',('outside_supported_task_grammar',))
    if 'NAVIGATE_TO' not in context.capabilities.operations or not context.capabilities.limits_ref.strip():
        return IntentResolution('UNSUPPORTED',('navigation_capability_not_declared',))
    confirmed=[]
    uncertain=[]
    candidates=candidate_entity_ids
    if selected is None:
        identity=explicit.group(1)
        if identity not in candidates:
            return IntentResolution('NEEDS_CLARIFICATION',('explicit_target_outside_scope',))
        candidates=(identity,)
    for identity in candidates:
        keys=[FactKey(subject=identity,predicate='exists')]
        if selected is not None:
            keys.extend((FactKey(subject=identity,predicate='kind',obj='marker'),
                         FactKey(subject=identity,predicate='color',obj=selected.group(1).lower())))
        truths=[context.snapshot.resolve(key,now_monotonic_s=context.now_monotonic_s) for key in keys]
        if Truth.FALSE in truths:
            continue  # Explicitly excluded within the declared selector/scope, not absent from the world.
        if all(truth is Truth.TRUE for truth in truths):
            confirmed.append(identity)
        else:
            uncertain.append(identity)
    if len(confirmed)!=1 or uncertain:
        reason='multiple_matching_targets' if len(confirmed)>1 else (
            'unresolved_possible_competitor' if confirmed and uncertain else 'target_not_confirmed')
        return IntentResolution('NEEDS_CLARIFICATION',(reason,))
    target=confirmed[0]
    binding=TaskIntentBinding(binding_id=context.task.task_id+'/'+RESOLVER_REVISION,
        source_ref=scope_ref+'/'+RESOLVER_REVISION,task_id=context.task.task_id,
        task_revision=context.task.revision,state_revision=context.snapshot.revision,
        snapshot_id=context.snapshot.snapshot_id,episode_id=context.snapshot.episode_id,
        embodiment_id=context.capabilities.embodiment_id,capability_revision=context.capabilities.revision,
        grounded_goal=Predicate(name='near',subject='$self',obj=target),
        allowed_targets=(target,),allowed_operations=('NAVIGATE_TO',))
    return IntentResolution('READY',(),binding)
