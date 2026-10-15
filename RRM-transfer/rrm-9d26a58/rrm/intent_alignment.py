"""Caller-bound semantic consistency, not natural-language understanding or admission.

The caller independently resolves the task. The model cannot supply/override this
binding. References and hashes are provenance, not proof of correct interpretation.
"""
import hashlib
import json
import math

from pydantic import BaseModel, ConfigDict, field_validator, model_validator

from .contracts import CapabilityDeclaration
from .schema import Predicate, Verb
from .state_contracts import StateSnapshot
from .task_contracts import PlanProposal, TaskRequest
from .verbs import expected_effects_of


class BoundIntentGoal(Predicate):
    model_config = ConfigDict(frozen=True, extra='forbid')

    @model_validator(mode='after')
    def validate_ground_goal(self):
        if not self.name.strip() or not self.subject.strip():
            raise ValueError('bound goal name and subject are required')
        if isinstance(self.obj,float) and not math.isfinite(self.obj):
            raise ValueError('bound goal literal must be finite')
        return self


class TaskIntentBinding(BaseModel):
    model_config = ConfigDict(frozen=True, extra='forbid')

    binding_id: str
    source_ref: str
    task_id: str
    task_revision: str
    state_revision: str
    capability_revision: str
    snapshot_id: str
    episode_id: str
    embodiment_id: str
    grounded_goal: BoundIntentGoal
    allowed_targets: tuple[str, ...] = ()
    allowed_operations: tuple[Verb, ...]

    @field_validator('grounded_goal',mode='before')
    @classmethod
    def isolate_goal(cls,value):
        # Base Predicate is mutable; copy into the immutable binding-owned subtype.
        return value.model_dump() if isinstance(value,Predicate) else value

    @model_validator(mode='after')
    def validate_binding(self):
        for name in ('binding_id','source_ref','task_id','task_revision',
                     'state_revision','capability_revision','snapshot_id','episode_id','embodiment_id'):
            if not getattr(self,name).strip():
                raise ValueError(f'{name} is required')
        if any(not target.strip() or target=='$self' for target in self.allowed_targets):
            raise ValueError('allowed targets require explicit non-self entity IDs')
        if len(set(self.allowed_targets))!=len(self.allowed_targets):
            raise ValueError('allowed targets must be unique')
        if not self.allowed_operations or len(set(self.allowed_operations))!=len(self.allowed_operations):
            raise ValueError('allowed operations must be nonempty and unique')
        for entity in (self.grounded_goal.subject,self.grounded_goal.obj):
            if isinstance(entity,str) and entity!='$self' and entity not in self.allowed_targets:
                raise ValueError('bound entity references must occur in allowed targets')
        return self

    def content_sha256(self):
        value=json.dumps(self.model_dump(mode='json'),sort_keys=True,separators=(',',':'),allow_nan=False)
        return hashlib.sha256(value.encode()).hexdigest()


def alignment_reasons(binding: TaskIntentBinding, *, task: TaskRequest,
                      snapshot: StateSnapshot, capabilities: CapabilityDeclaration,
                      eligible_entities: frozenset[str], plan: PlanProposal) -> tuple[str, ...]:
    reasons=[]
    for name,current in (('task_id',task.task_id),('task_revision',task.revision),
                         ('state_revision',snapshot.revision),('capability_revision',capabilities.revision),
                         ('snapshot_id',snapshot.snapshot_id),('episode_id',snapshot.episode_id),
                         ('embodiment_id',capabilities.embodiment_id)):
        if getattr(binding,name)!=current:
            reasons.append('intent_binding_'+name+'_mismatch')
    if task.requested_embodiment_id!=capabilities.embodiment_id:
        reasons.append('intent_context_embodiment_mismatch')
    if not set(binding.allowed_targets)<=eligible_entities:
        reasons.append('intent_binding_target_unresolved')
    if not {v.value for v in binding.allowed_operations}<=capabilities.operations:
        reasons.append('intent_binding_operation_unsupported')
    if binding.grounded_goal.model_dump_json()!=plan.intent.grounded_goal.model_dump_json():
        reasons.append('intent_goal_mismatch')
    if not any(effect.model_dump_json()==binding.grounded_goal.model_dump_json()
               for item in plan.actions for effect in expected_effects_of(item.action)):
        reasons.append('intent_goal_not_supported_by_plan_effects')
    for item in plan.actions:
        if item.action.verb not in binding.allowed_operations:
            reasons.append('intent_operation_not_allowed:'+item.action.id)
        if not set(item.action.targets)<=set(binding.allowed_targets):
            reasons.append('intent_target_not_allowed:'+item.action.id)
    return tuple(reasons)
