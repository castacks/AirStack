"""The RRM control loop. See docs/architecture.md §4.

Outer loop plans and verifies; inner loop executes until expected effects hold."""

from __future__ import annotations

import hashlib
import json
import logging
import math
import time
from dataclasses import asdict, dataclass
from uuid import uuid4

from .schema import (
    AbstractAction, ActionPolicy, Divergence, Predicate, RunMetrics, Task,
    TaskGraph, Termination, Trajectory, WorldBackend, WorldState,
)
from .contracts import (
    ApprovalDecision, ApprovalProvider, ApprovalScope,
    CapabilityDeclaration, DispatchContext, PermissionDeclaration, SafetyDecision,
)
from .core_admission import CoreAdmission
from .core_deadlines import BoundedTracer, CoreCallTimeout, bounded_call
from .world import DispatchCancelled
from .uncertainty import validate_uncertainty
from .verbs import VERB_TABLE, expected_effects_of, holds
from .reasoning import ReasonerBackend
from .safety import NumericSafetyVerifier, SafetyVerifier
from .trace import Tracer

log = logging.getLogger("rrm")

# ---------------------------------------------------------------------------
# Divergence  (docs/architecture.md §3.5)
# ---------------------------------------------------------------------------

def check_divergence(action: AbstractAction, before: WorldState,
                     after: WorldState, *, plan_version: int) -> Divergence:
    expected = expected_effects_of(action)
    unmet = [e for e in expected if not holds(e, after)]

    # Surprise: relations that changed without being declared as an effect.
    declared = {(e.name, e.subject, str(e.obj)) for e in expected}
    before_rels = {(r.subject, r.predicate, r.obj) for r in before.relations}
    after_rels = {(r.subject, r.predicate, r.obj) for r in after.relations}
    surprise = [
        Predicate(name=p, subject=s, obj=o)
        for (s, p, o) in (after_rels - before_rels)
        if (p, s, o) not in declared
    ]

    magnitude = len(unmet) / len(expected) if expected else 0.0
    return Divergence(action_id=action.id, plan_version=plan_version,
                      unmet=unmet, surprise=surprise, magnitude=magnitude)


# ---------------------------------------------------------------------------
# The loop  (docs/architecture.md §4)
# ---------------------------------------------------------------------------

THETA_DIV = 0.01
THETA_UNC = 0.0          # conservative default until aggregate uncertainty is calibrated
REPLAN_BUDGET = 3
CYCLE_BUDGET = 6        # inner-loop cycles before a subtask is declared stalled


@dataclass
class DispatchObservation:
    """Last valid snapshot; a failed observation never promotes it to current."""

    state: WorldState
    digest: str
    failed: bool = False
    phase: str = "begin_dispatch"
    cycles: int = 0
    interruption_recorded: bool = False


class CoreEvidenceUnavailable(RuntimeError):
    """Execution was inhibited/stopped but its complete evidence could not be saved."""


def _semantic_action_record(action: AbstractAction) -> dict:
    """Scene-independent payload used for retry convergence and its evidence."""
    return {
        "verb": action.verb.value,
        "targets": list(action.targets),
        "params": action.params,
    }


def _semantic_action_signature(action: AbstractAction) -> str:
    return json.dumps(
        _semantic_action_record(action), sort_keys=True, separators=(",", ":"), default=str,
    )


def _trace_action_record(action: AbstractAction) -> dict:
    record = {"action_id": action.id, **_semantic_action_record(action)}
    encoded = json.dumps(record, sort_keys=True, separators=(",", ":"), default=str)
    return {**record, "action_digest": hashlib.sha256(encoded.encode("utf-8")).hexdigest()}


def _action_digest(action: AbstractAction) -> str:
    return _trace_action_record(action)["action_digest"]


def _trajectory_record(traj) -> tuple[dict, str]:
    record = traj.model_dump(mode="json")
    return record, _record_digest(record)


def _observe(world: WorldBackend, tracer: Tracer, *, phase: str) -> tuple[WorldState, str]:
    state = bounded_call("observe", tracer.observation_timeout_s, world.observe)
    if not isinstance(state, WorldState):
        raise TypeError("world observation is not a WorldState")
    payload = state.model_dump(mode="json")
    state = WorldState.model_validate(payload)
    validate_uncertainty(state)
    payload = state.model_dump(mode="json")
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    digest = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
    tracer.event("world_state", sim_t=state.t, phase=phase,
                 state_digest=digest, state=payload)
    return state, digest


def _capability_record(capabilities: CapabilityDeclaration) -> dict:
    return {
        "embodiment_id": capabilities.embodiment_id,
        "revision": capabilities.revision,
        "operations": sorted(capabilities.operations),
        "resources": sorted(capabilities.resources),
        "available_resources": sorted(capabilities.available_resources),
        "limits_ref": capabilities.limits_ref,
    }


def _permission_record(permission: PermissionDeclaration) -> dict:
    return {
        "authority_id": permission.authority_id,
        "revision": permission.revision,
        "task_id": permission.task_id,
        "embodiment_id": permission.embodiment_id,
        "operations": sorted(permission.operations),
        "resources": sorted(permission.resources),
    }


def _record_digest(record: dict) -> str:
    encoded = json.dumps(record, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _constraints_record(profile_digest: str) -> dict:
    return {
        "schema_version": "rrm-core-constraints/v1",
        "symbolic_verifier_revision": "core-symbolic-v1",
        "numeric_profile_digest": profile_digest,
        "verb_semantics": {
            verb.value: {
                "arity": spec.arity,
                "preconditions": [item.model_dump(mode="json") for item in spec.preconditions],
                "expected_effects": [item.model_dump(mode="json") for item in spec.expected_effects],
                "required_resources": sorted(spec.required_resources),
            }
            for verb, spec in sorted(VERB_TABLE.items(), key=lambda pair: pair[0].value)
        },
    }


def _plan_record(graph: TaskGraph, *, task_revision: str, task_digest: str) -> dict:
    return {
        "plan_id": graph.mission_id,
        "version": graph.version,
        "task_revision": task_revision,
        "task_digest": task_digest,
        "mission_text": graph.mission_text,
        "graph": graph.model_dump(mode="json"),
        "actions": [_trace_action_record(action) for action in graph.nodes],
    }


def _context_gate(task: Task, graph: TaskGraph, action: AbstractAction,
                  tracer: Tracer, *, phase: str, sim_t: int, state_digest: str,
                  task_revision: str, task_digest: str, plan_id: str,
                  plan_record: dict, plan_digest: str,
                  dispatch_id: str | None = None) -> bool:
    current_task = task.model_dump(mode="json")
    current_plan = _plan_record(
        graph, task_revision=task_revision, task_digest=task_digest,
    )
    observed_task_digest = _record_digest(current_task)
    observed_plan_digest = _record_digest(current_plan)
    expected_actions = {
        item["action_id"]: item["action_digest"] for item in plan_record["actions"]
    }
    reasons = []
    if task.revision != task_revision:
        reasons.append("task_revision_changed")
    if observed_task_digest != task_digest:
        reasons.append("task_payload_changed")
    if graph.mission_id != plan_id:
        reasons.append("plan_id_changed")
    if observed_plan_digest != plan_digest:
        reasons.append("plan_payload_changed")
    if expected_actions.get(action.id) != _action_digest(action):
        reasons.append("action_not_in_bound_plan")
    fields = {
        "phase": phase,
        "sim_t": sim_t,
        "state_digest": state_digest,
        "task_id": task.id,
        "task_revision": task_revision,
        "task_digest": task_digest,
        "observed_task_digest": observed_task_digest,
        "current_task": current_task,
        "plan_id": plan_id,
        "plan_version": plan_record["version"],
        "plan_digest": plan_digest,
        "observed_plan_digest": observed_plan_digest,
        "current_plan": current_plan,
        "action_id": action.id,
        "action_digest": _action_digest(action),
        "verdict": "DENY" if reasons else "ALLOW",
        "reasons": reasons,
    }
    if dispatch_id is not None:
        fields["dispatch_id"] = dispatch_id
    tracer.event("context_gate", **fields)
    return not reasons


def _validate_graph_identity(graph: TaskGraph, *, expected_version: int,
                             expected_plan_id: str | None = None,
                             expected_mission: str | None = None) -> None:
    """Reject ambiguous planner output before any action can be dispatched."""
    if graph.version != expected_version:
        raise ValueError(
            f"plan version {graph.version} does not match expected {expected_version}"
        )
    if not graph.mission_id.strip():
        raise ValueError("plan ID must be nonempty")
    if expected_plan_id is not None and graph.mission_id != expected_plan_id:
        raise ValueError("replan changed the bound plan ID")
    if expected_mission is not None and graph.mission_text != expected_mission:
        raise ValueError("plan changed the bound mission")
    action_ids = [action.id for action in graph.nodes]
    if any(not action_id.strip() for action_id in action_ids):
        raise ValueError("plan action IDs must be nonempty")
    if len(action_ids) != len(set(action_ids)):
        raise ValueError("plan action IDs must be unique within a plan version")


def _uncertainty_allows(ws: WorldState, threshold: float, tracer: Tracer, *,
                        phase: str, state_digest: str,
                        plan_version: int | None = None,
                        action: AbstractAction | None = None,
                        dispatch_id: str | None = None) -> bool:
    passed = ws.uncertainty <= threshold
    fields = {
        "sim_t": ws.t,
        "phase": phase,
        "uncertainty": ws.uncertainty,
        "threshold": threshold,
        "verdict": "PASS" if passed else "FAIL",
        "state_digest": state_digest,
    }
    if action is not None:
        fields.update(
            plan_version=plan_version,
            action_id=action.id,
            action_digest=_action_digest(action),
        )
    if dispatch_id is not None:
        fields["dispatch_id"] = dispatch_id
    tracer.event("uncertainty_gate", **fields)
    return passed


def _stop_latched(admission: CoreAdmission, tracer: Tracer, *, phase: str,
                  sim_t: int, state_digest: str, plan_version: int,
                  action: AbstractAction, dispatch_id: str,
                  decision_id: str, expected_generation: int,
                  observation: DispatchObservation | None = None) -> bool:
    state = admission.guard.snapshot(
        decision_id=decision_id, run_id=tracer.run_id, dispatch_id=dispatch_id,
    )
    latched = state["stopped"] or state["stop_generation"] != expected_generation
    if latched:
        outcome = admission.stop.outcome
        if outcome is not None and not outcome.trace_complete:
            raise CoreEvidenceUnavailable("stop completed with incomplete evidence")
        try:
            tracer.event(
                "interruption_gate", sim_t=sim_t, state_digest=state_digest,
                phase=phase, plan_version=plan_version,
                action_id=action.id, action_digest=_action_digest(action),
                dispatch_id=dispatch_id, expected_generation=expected_generation,
                observed_generation=state["stop_generation"], stopped=state["stopped"],
                intervention_id=outcome.intervention_id if outcome else None,
                safe_status=outcome.status if outcome else "SAFE_UNCONFIRMED",
                verdict="INTERRUPT",
            )
            if observation is not None:
                observation.interruption_recorded = True
        except Exception as error:
            raise CoreEvidenceUnavailable("interruption evidence could not be saved") from error
    return latched


def dispatch(task: Task, graph: TaskGraph, action: AbstractAction,
             world: WorldBackend, policy: ActionPolicy, verifier: SafetyVerifier,
             numeric: NumericSafetyVerifier, tracer: Tracer, *,
             plan_version: int,
             profile_digest: str,
             task_revision: str, task_digest: str,
             plan_id: str, plan_record: dict, plan_digest: str,
             dispatch_id: str,
             approval_digest: str, approval_revision: str,
             approval_decision_id: str,
             authorization_digest: str, authorization_decision_id: str,
             authorization_context_digest: str,
             authorized_state_digest: str, authorized_sim_t: int,
             authorized_stop_generation: int,
             observation: DispatchObservation,
             admission: CoreAdmission,
             uncertainty_threshold: float = THETA_UNC) -> tuple[Termination, int]:
    """Inner loop: run the policy until the action's expected effects hold.

    Termination is by **effect satisfaction**, not by timeout and not by the policy
    signalling completion — GR00T emits no completion signal. The world model already
    knows what should become true (the verb table said so), so it polls for exactly
    that. Timeout is the failure path, not the normal one.
    """
    effects = expected_effects_of(action)
    if _stop_latched(
            admission, tracer, phase="pre_dispatch", sim_t=authorized_sim_t,
            state_digest=authorized_state_digest, plan_version=plan_version,
            action=action, dispatch_id=dispatch_id,
            decision_id=authorization_decision_id,
            expected_generation=authorized_stop_generation, observation=observation):
        return Termination.INTERRUPTED, 0
    bounded_call("begin_dispatch", admission.stop.limits.adapter_s,
                 lambda: world.begin_dispatch(action, dispatch_id), admission.pending_actuation)
    last_applied_t: int | None = None

    def observation_failure(cycle: int, failure_kind: str, *,
                            observed_digest: str | None = None,
                            observed_t: int | None = None) -> tuple[Termination, int]:
        observation.failed = True
        try:
            tracer.event("observation_failure", sim_t=observation.state.t,
                         state_digest=observation.digest, phase="dispatch",
                         plan_version=plan_version, action_id=action.id,
                         action_digest=_action_digest(action), dispatch_id=dispatch_id,
                         cycle=cycle, failure_kind=failure_kind,
                         observed_state_digest=observed_digest,
                         observed_sim_t=observed_t)
        except Exception as error:
            raise CoreEvidenceUnavailable("observation failure evidence could not be saved") from error
        finally:
            admission.stop.request_stop(
                dispatch_id=dispatch_id, reason="observation_unavailable")
        _stop_latched(
            admission, tracer, phase="observation_failure", sim_t=observation.state.t,
            state_digest=observation.digest, plan_version=plan_version,
            action=action, dispatch_id=dispatch_id,
            decision_id=authorization_decision_id,
            expected_generation=authorized_stop_generation, observation=observation,
        )
        return Termination.INTERRUPTED, cycle - 1

    def interrupt(reason: str, phase: str, ws: WorldState, state_digest: str,
                  completed_cycles: int) -> tuple[Termination, int]:
        admission.stop.request_stop(
            dispatch_id=dispatch_id, reason=reason)
        _stop_latched(
            admission, tracer, phase=phase, sim_t=ws.t,
            state_digest=state_digest, plan_version=plan_version,
            action=action, dispatch_id=dispatch_id,
            decision_id=authorization_decision_id,
            expected_generation=authorized_stop_generation, observation=observation,
        )
        return Termination.INTERRUPTED, completed_cycles

    for cycle in range(1, CYCLE_BUDGET + 1):
        observation.phase = "observation"
        observation.cycles = cycle - 1
        try:
            ws, state_digest = _observe(world, tracer, phase="dispatch")
        except CoreCallTimeout:
            raise
        except (TypeError, ValueError, AttributeError):
            return observation_failure(cycle, "invalid_payload")
        except Exception:
            return observation_failure(cycle, "observation_error")
        if last_applied_t is not None and ws.t <= last_applied_t:
            return observation_failure(cycle, "stale_after_apply",
                                       observed_digest=state_digest, observed_t=ws.t)
        observation.state, observation.digest = ws, state_digest
        if _stop_latched(
                admission, tracer, phase="pre_chunk", sim_t=ws.t,
                state_digest=state_digest, plan_version=plan_version,
                action=action, dispatch_id=dispatch_id,
                decision_id=authorization_decision_id,
                expected_generation=authorized_stop_generation, observation=observation):
            return Termination.INTERRUPTED, cycle - 1
        if not _uncertainty_allows(
                ws, uncertainty_threshold, tracer, phase="dispatch",
                state_digest=state_digest,
                plan_version=plan_version, action=action,
                dispatch_id=dispatch_id):
            return interrupt("active_uncertainty", "active_uncertainty", ws, state_digest, cycle - 1)
        observation.phase = "context_validation"
        if not _context_gate(
                task, graph, action, tracer, phase="pre_chunk", sim_t=ws.t,
                state_digest=state_digest, task_revision=task_revision,
                task_digest=task_digest, plan_id=plan_id,
                plan_record=plan_record, plan_digest=plan_digest,
                dispatch_id=dispatch_id):
            return interrupt("active_context_changed", "context_changed", ws, state_digest, cycle - 1)
        if effects and all(holds(e, ws) for e in effects):
            return Termination.COMPLETE, cycle - 1

        observation.phase = "symbolic_verification"
        dynamic_verdict = verifier.verify(action, ws)
        tracer.event("dynamic_safety_gate", sim_t=ws.t,
                     state_digest=state_digest, plan_version=plan_version,
                     action_id=action.id, action_digest=_action_digest(action),
                     dispatch_id=dispatch_id, cycle=cycle,
                     verdict=dynamic_verdict.verdict,
                     checked=dynamic_verdict.checked,
                     violations=[v.model_dump() for v in dynamic_verdict.violations])
        if dynamic_verdict.verdict == "FAIL":
            return interrupt("dynamic_symbolic_safety", "dynamic_safety", ws, state_digest, cycle - 1)

        observation.phase = "policy_step"
        observation.cycles = cycle
        # A proposal source cannot rewrite the immutable evidence snapshot used
        # by independent safety or fault provenance.
        policy_action, policy_state = action.model_copy(deep=True), ws.model_copy(deep=True)
        traj = bounded_call("policy_step", admission.stop.limits.policy_s,
                            lambda: policy.step(policy_action, policy_state))
        if _stop_latched(
                admission, tracer, phase="post_policy", sim_t=ws.t,
                state_digest=state_digest, plan_version=plan_version,
                action=action, dispatch_id=dispatch_id,
                decision_id=authorization_decision_id,
                expected_generation=authorized_stop_generation, observation=observation):
            return Termination.INTERRUPTED, cycle - 1
        observation.phase = "context_validation"
        if not _context_gate(
                task, graph, action, tracer, phase="pre_apply", sim_t=ws.t,
                state_digest=state_digest, task_revision=task_revision,
                task_digest=task_digest, plan_id=plan_id,
                plan_record=plan_record, plan_digest=plan_digest,
                dispatch_id=dispatch_id):
            return interrupt("active_context_changed", "context_changed", ws, state_digest, cycle)
        observation.phase = "trajectory_validation"
        if not isinstance(traj, Trajectory):
            raise TypeError("policy result is not a Trajectory")
        # model_copy/model_construct and later mutation can bypass validation.
        traj = Trajectory.model_validate(traj.model_dump(mode="json"))
        trajectory, trajectory_digest = _trajectory_record(traj)
        observation.phase = "numeric_validation"
        verdict = numeric.verify(traj, ws, expected_action_id=action.id)
        tracer.event("safety2", sim_t=ws.t, plan_version=plan_version,
                     action_id=action.id, action_digest=_action_digest(action), cycle=cycle,
                     dispatch_id=dispatch_id, plan_id=plan_id, plan_digest=plan_digest,
                     approval_digest=approval_digest,
                     approval_revision=approval_revision,
                     approval_decision_id=approval_decision_id,
                     authorization_digest=authorization_digest,
                     authorization_decision_id=authorization_decision_id,
                     authorization_context_digest=authorization_context_digest,
                     state_digest=state_digest, profile_digest=profile_digest,
                     trajectory=trajectory, trajectory_digest=trajectory_digest,
                     verdict=verdict.verdict, checked=verdict.checked,
                     violations=[v.model_dump() for v in verdict.violations])
        if verdict.verdict == "FAIL":
            for v in verdict.violations:
                log.info("        SAFETY#2 FAIL [%s] %s", v.check, v.detail)
            return interrupt("active_numeric_safety", "numeric_safety", ws, state_digest, cycle)

        if _stop_latched(
                admission, tracer, phase="pre_apply", sim_t=ws.t,
                state_digest=state_digest, plan_version=plan_version,
                action=action, dispatch_id=dispatch_id,
                decision_id=authorization_decision_id,
                expected_generation=authorized_stop_generation, observation=observation):
            return Termination.INTERRUPTED, cycle - 1

        try:
            observation.phase = "apply"
            bounded_call("apply", admission.stop.limits.adapter_s,
                         lambda: world.apply(action, traj), admission.pending_actuation)
        except DispatchCancelled:
            if not _stop_latched(
                admission, tracer, phase="apply_cancelled", sim_t=ws.t,
                state_digest=state_digest, plan_version=plan_version,
                action=action, dispatch_id=dispatch_id,
                decision_id=authorization_decision_id,
                expected_generation=authorized_stop_generation, observation=observation,
            ):
                raise
            return Termination.INTERRUPTED, cycle - 1
        observation.phase = "apply_evidence"
        tracer.event("apply", sim_t=ws.t, plan_version=plan_version,
                     action_id=action.id, action_digest=_action_digest(action), cycle=cycle,
                     dispatch_id=dispatch_id, plan_id=plan_id, plan_digest=plan_digest,
                     approval_digest=approval_digest,
                     approval_revision=approval_revision,
                     approval_decision_id=approval_decision_id,
                     authorization_digest=authorization_digest,
                     authorization_decision_id=authorization_decision_id,
                     authorization_context_digest=authorization_context_digest,
                     state_digest=state_digest, profile_digest=profile_digest,
                     trajectory_digest=trajectory_digest,
                     terminal=traj.terminal, max_velocity=traj.max_velocity)
        last_applied_t = ws.t

    observation.phase = "dispatch_terminal_observation"
    try:
        ws, state_digest = _observe(world, tracer, phase="dispatch_terminal")
    except CoreCallTimeout:
        raise
    except (TypeError, ValueError, AttributeError):
        return observation_failure(CYCLE_BUDGET + 1, "invalid_payload")
    except Exception:
        return observation_failure(CYCLE_BUDGET + 1, "observation_error")
    if last_applied_t is not None and ws.t <= last_applied_t:
        return observation_failure(CYCLE_BUDGET + 1, "stale_after_apply",
                                   observed_digest=state_digest, observed_t=ws.t)
    observation.state, observation.digest = ws, state_digest
    if not _uncertainty_allows(
            ws, uncertainty_threshold, tracer, phase="dispatch",
            state_digest=state_digest,
            plan_version=plan_version, action=action,
            dispatch_id=dispatch_id):
        return interrupt("active_uncertainty", "active_uncertainty", ws, state_digest, CYCLE_BUDGET)
    if effects and all(holds(e, ws) for e in effects):
        return Termination.COMPLETE, CYCLE_BUDGET
    return Termination.TIMEOUT, CYCLE_BUDGET


def run(task: Task, world: WorldBackend, reasoner: ReasonerBackend,
        verifier: SafetyVerifier, policy: ActionPolicy,
        numeric: NumericSafetyVerifier,
        tracer: Tracer | None = None, *,
        capabilities: CapabilityDeclaration,
        permission: PermissionDeclaration,
        approval: ApprovalProvider,
        admission: CoreAdmission,
        uncertainty_threshold: float = THETA_UNC) -> RunMetrics:
    """Outer loop. Returns measurements, not a verdict — see docs/benchmarks.md §3."""
    m = RunMetrics(task_id=task.id)
    mission = task.mission
    if admission.call_limits != admission.stop.limits:
        raise ValueError("core call limits changed after supervision was configured")
    tracer = tracer or Tracer(None, {})
    tracer = BoundedTracer(tracer, admission.stop.limits.evidence_s)
    tracer.observation_timeout_s = admission.stop.limits.observation_s
    admission.stop.bind(world, tracer)
    tracer.event("call_limits_declaration", limits=admission.stop.limits.record())
    if not task.id.strip() or not task.revision.strip():
        raise ValueError("task ID and revision must be nonempty")
    task_record = task.model_dump(mode="json")
    bound_goal = task.goal.model_copy(deep=True)
    task_digest = _record_digest(task_record)
    task_revision = task.revision
    tracer.event("task_declaration", task=task_record, task_digest=task_digest,
                 task_revision=task_revision)
    capability_record = _capability_record(capabilities)
    capability_digest = _record_digest(capability_record)
    tracer.event("capability_declaration", capability=capability_record,
                 capability_digest=capability_digest)
    permission_record = _permission_record(permission)
    permission_digest = _record_digest(permission_record)
    tracer.event("permission_declaration", permission=permission_record,
                 permission_digest=permission_digest)
    profile_record = numeric.profile.model_dump(mode="json")
    profile_digest = _record_digest(profile_record)
    tracer.event("numeric_profile_declaration", profile=profile_record,
                 profile_digest=profile_digest)
    constraints_record = _constraints_record(profile_digest)
    constraints_digest = _record_digest(constraints_record)
    tracer.event("constraints_declaration", constraints=constraints_record,
                 constraints_digest=constraints_digest)
    tracer.event("admission_authority", authority_epoch=admission.guard.epoch,
                 stop_generation=admission.guard.generation,
                 evidence_kind=admission.evidence_kind)
    if not math.isfinite(uncertainty_threshold) \
            or not 0.0 <= uncertainty_threshold <= 1.0:
        raise ValueError("uncertainty_threshold must be finite and within [0,1]")

    try:
        ws, state_digest = _observe(world, tracer, phase="planning")
    except Exception as exc:
        failure_kind = ("invalid_payload" if isinstance(exc, (TypeError, ValueError,
                                                                  AttributeError))
                        else "observation_error")
        tracer.event("observation_failure", sim_t=None, state_digest=None,
                     phase="planning", plan_version=None, action_id=None,
                     action_digest=None, dispatch_id=None, cycle=None,
                     failure_kind=failure_kind,
                     observed_state_digest=None, observed_sim_t=None)
        m.aborted = True
        tracer.event("episode_end", sim_t=None, state_digest=None,
                     goal_met=False, terminal_observation="UNAVAILABLE",
                     stop_status="NOT_REQUESTED", **m.model_dump())
        return m
    last_valid_ws, last_valid_digest = ws, state_digest
    if not _uncertainty_allows(
            ws, uncertainty_threshold, tracer, phase="planning",
            state_digest=state_digest):
        m.aborted = True
        m.task_success = task.expect_abort
        tracer.event("episode_end", sim_t=ws.t, state_digest=state_digest,
                     goal_met=False, terminal_observation="OBSERVED",
                     stop_status="NOT_REQUESTED", **m.model_dump())
        return m
    t0 = time.perf_counter()
    graph = reasoner.plan(mission, ws)
    _validate_graph_identity(graph, expected_version=0,
                             expected_mission=mission)
    plan_id = graph.mission_id
    plan_mission = graph.mission_text
    plan_record = _plan_record(graph, task_revision=task_revision,
                               task_digest=task_digest)
    plan_digest = _record_digest(plan_record)
    m.planning_latency_ms += (time.perf_counter() - t0) * 1000
    tracer.event("plan", sim_t=ws.t, version=graph.version,
                 state_digest=state_digest,
                 plan_id=plan_id, plan_digest=plan_digest,
                 plan=plan_record, task_revision=task_revision,
                 task_digest=task_digest,
                 reasoner=reasoner.name, latency_ms=round(m.planning_latency_ms, 3),
                 nodes=[str(n) for n in graph.nodes],
                 actions=[_trace_action_record(n) for n in graph.nodes],
                 goal=str(task.goal))
    log.info("PLAN v%d: %s", graph.version, " -> ".join(str(n) for n in graph.nodes))

    cursor = 0

    def replan_or_abort(state: WorldState, state_digest: str, div: Divergence, *,
                        rejected_action: AbstractAction | None = None) -> bool:
        """Returns True if the loop should continue, False to abort."""
        nonlocal graph, cursor, plan_record, plan_digest
        if m.replans >= REPLAN_BUDGET:
            log.info("        replan budget exhausted -> ABORT")
            m.aborted = True
            return False
        m.replans += 1
        t = time.perf_counter()
        previous_version = graph.version
        graph = reasoner.replan(mission, state, graph, div)
        _validate_graph_identity(graph, expected_version=previous_version + 1,
                                 expected_plan_id=plan_id,
                                 expected_mission=plan_mission)
        plan_record = _plan_record(graph, task_revision=task_revision,
                                   task_digest=task_digest)
        plan_digest = _record_digest(plan_record)
        dt = (time.perf_counter() - t) * 1000
        m.planning_latency_ms += dt
        tracer.event("replan", sim_t=state.t, version=graph.version,
                     state_digest=state_digest,
                     plan_id=plan_id, plan_digest=plan_digest,
                     plan=plan_record, task_revision=task_revision,
                     task_digest=task_digest,
                     latency_ms=round(dt, 3), trigger=div.model_dump(),
                     nodes=[str(n) for n in graph.nodes],
                     actions=[_trace_action_record(n) for n in graph.nodes])
        log.info("        REPLAN v%d: %s", graph.version,
                 " -> ".join(str(n) for n in graph.nodes) or "(empty)")
        cursor = 0
        if rejected_action is not None:
            if not graph.nodes:
                m.aborted = True
                tracer.event(
                    "replan_convergence", sim_t=state.t,
                    reason="NO_ALTERNATIVE_AFTER_REJECTION",
                    rejected_action_id=rejected_action.id,
                    rejected_plan_version=div.plan_version,
                    replacement_action_id=None,
                    replacement_plan_version=graph.version,
                    rejected_action=_semantic_action_record(rejected_action),
                    replacement_action=None,
                )
                log.info("        no alternative after safety rejection -> ABORT")
                return False
            replacement = graph.nodes[0]
            if _semantic_action_signature(replacement) == \
                    _semantic_action_signature(rejected_action):
                m.aborted = True
                tracer.event(
                    "replan_convergence", sim_t=state.t,
                    reason="UNCHANGED_REJECTED_ACTION",
                    rejected_action_id=rejected_action.id,
                    rejected_plan_version=div.plan_version,
                    replacement_action_id=replacement.id,
                    replacement_plan_version=graph.version,
                    rejected_action=_semantic_action_record(rejected_action),
                    replacement_action=_semantic_action_record(replacement),
                )
                log.info("        unchanged rejected action after replan -> ABORT")
                return False
        return bool(graph.nodes)

    pre_action_observation_lost = False
    active_execution_evidence_lost = False
    while cursor < len(graph.nodes):
        action = graph.nodes[cursor].model_copy(deep=True)
        try:
            ws, state_digest = _observe(world, tracer, phase="pre_action")
        except Exception as exc:
            failure_kind = ("invalid_payload" if isinstance(exc, (TypeError, ValueError,
                                                                      AttributeError))
                            else "observation_error")
            tracer.event("observation_failure", sim_t=last_valid_ws.t,
                         state_digest=last_valid_digest, phase="pre_action",
                         plan_version=graph.version, action_id=action.id,
                         action_digest=_action_digest(action), dispatch_id=None,
                         cycle=None, failure_kind=failure_kind,
                         observed_state_digest=None, observed_sim_t=None)
            m.aborted = True
            pre_action_observation_lost = True
            break
        last_valid_ws, last_valid_digest = ws, state_digest
        if not _uncertainty_allows(
                ws, uncertainty_threshold, tracer, phase="pre_action",
                state_digest=state_digest,
                plan_version=graph.version, action=action):
            m.aborted = True
            break
        if not _context_gate(
                task, graph, action, tracer, phase="pre_action", sim_t=ws.t,
                state_digest=state_digest, task_revision=task_revision,
                task_digest=task_digest, plan_id=plan_id,
                plan_record=plan_record, plan_digest=plan_digest):
            m.aborted = True
            break
        required_resources = VERB_TABLE[action.verb].required_resources
        capability_reasons = capabilities.rejection_reasons(
            action.verb.value, required_resources,
        )
        tracer.event(
            "capability_gate", sim_t=ws.t, state_digest=state_digest,
            plan_version=graph.version, action_id=action.id,
            action_digest=_action_digest(action), capability_digest=capability_digest,
            operation=action.verb.value,
            required_resources=sorted(required_resources),
            verdict="DENY" if capability_reasons else "ALLOW",
            reasons=list(capability_reasons),
        )
        if capability_reasons:
            m.aborted = True
            break
        permission_reasons = permission.rejection_reasons(
            task_id=task.id,
            embodiment_id=capabilities.embodiment_id,
            operation=action.verb.value,
            resources=required_resources,
        )
        tracer.event(
            "permission_gate", sim_t=ws.t, state_digest=state_digest,
            plan_version=graph.version, action_id=action.id,
            action_digest=_action_digest(action), permission_digest=permission_digest,
            task_id=task.id, embodiment_id=capabilities.embodiment_id,
            operation=action.verb.value,
            required_resources=sorted(required_resources),
            verdict="DENY" if permission_reasons else "ALLOW",
            reasons=list(permission_reasons),
        )
        if permission_reasons:
            m.aborted = True
            break
        approval_scope = ApprovalScope(
            run_id=tracer.run_id, task_id=task.id,
            task_revision=task_revision, task_digest=task_digest,
            plan_id=plan_id, plan_version=graph.version, plan_digest=plan_digest,
            action_id=action.id, action_digest=_action_digest(action),
        )
        try:
            decision = approval.decide(approval_scope) if approval is not None else None
        except Exception:
            decision = None
        if decision is None:
            approval_reasons = ["approval_missing"]
            approval_record = None
            approval_status = "missing"
        elif not isinstance(decision, ApprovalDecision):
            approval_reasons = ["invalid_approval_decision"]
            approval_record = None
            approval_status = "invalid"
        else:
            approval_reasons = list(decision.rejection_reasons(approval_scope))
            approval_record = decision.as_record()
            approval_status = "present"
        approval_digest = (_record_digest(approval_record)
                           if approval_record is not None else None)
        tracer.event(
            "approval_gate", sim_t=ws.t, state_digest=state_digest,
            plan_version=graph.version, action_id=action.id,
            action_digest=_action_digest(action),
            scope=approval_scope.as_record(),
            approval=approval_record, approval_digest=approval_digest,
            approval_status=approval_status,
            verdict="DENY" if approval_reasons else "ALLOW",
            reasons=approval_reasons,
        )
        if approval_reasons:
            m.aborted = True
            break
        approved_decision = decision
        profile_reasons = []
        if capabilities.limits_ref != numeric.profile.ref:
            profile_reasons.append("limits_ref_mismatch")
        if capabilities.embodiment_id != numeric.profile.embodiment_id:
            profile_reasons.append("profile_embodiment_mismatch")
        tracer.event(
            "numeric_profile_gate", sim_t=ws.t, state_digest=state_digest,
            plan_version=graph.version, action_id=action.id,
            action_digest=_action_digest(action), capability_digest=capability_digest,
            profile_digest=profile_digest,
            verdict="DENY" if profile_reasons else "ALLOW", reasons=profile_reasons,
        )
        if profile_reasons:
            m.aborted = True
            break
        log.info("")
        log.info("  [t=%d] %s", ws.t, action)
        log.info("        expect: %s",
                 ", ".join(str(e) for e in expected_effects_of(action)) or "—")

        verdict = verifier.verify(action, ws)
        tracer.event("safety1", sim_t=ws.t, plan_version=graph.version,
                     action_id=action.id, action_digest=_action_digest(action),
                     state_digest=state_digest,
                     action=str(action),
                     verdict=verdict.verdict, checked=verdict.checked,
                     violations=[v.model_dump() for v in verdict.violations])
        if verdict.verdict == "FAIL":
            m.safety_rejections += 1
            for v in verdict.violations:
                log.info("        SAFETY#1 FAIL [%s] %s", v.check, v.detail)
            if not replan_or_abort(
                    ws, state_digest,
                    Divergence(action_id=action.id, plan_version=graph.version),
                    rejected_action=action):
                break
            continue
        log.info("        safety PASS (%s)", ", ".join(verdict.checked))

        before = ws
        before_digest = state_digest
        dispatch_id = uuid4().hex
        authorization_context = DispatchContext(
            run_id=tracer.run_id, task_revision=task_digest,
            plan_revision=plan_digest, action_id=action.id,
            dispatch_id=dispatch_id, action_digest=_action_digest(action),
            state_revision=state_digest, capability_revision=capability_digest,
            permission_revision=permission_digest, approval_revision=approval_digest,
            constraints_revision=constraints_digest,
            authority_epoch=admission.guard.epoch,
            stop_generation=admission.guard.generation,
        )
        context_record = asdict(authorization_context)
        context_digest = _record_digest(context_record)
        try:
            raw_now = admission.clock()
            now = (float(raw_now) if type(raw_now) in {int, float}
                   and math.isfinite(raw_now) else None)
        except Exception:
            now = None
        decision = None
        guard_state = None
        if now is None:
            authorization_reason = "clock_unavailable"
            decision_status = "missing"
        else:
            try:
                decision = admission.provider.decide(authorization_context, now=now)
            except Exception:
                decision = None
            if decision is None:
                authorization_reason = "decision_missing"
                decision_status = "missing"
            elif not isinstance(decision, SafetyDecision):
                authorization_reason = "invalid_decision"
                decision_status = "invalid"
                decision = None
            else:
                authorization_reason, guard_state = admission.guard.consume_evidenced(
                    decision, authorization_context, now=now,
                )
                decision_status = "present"
        if guard_state is None:
            guard_state = admission.guard.snapshot(
                decision_id=decision.decision_id if decision is not None else "",
                run_id=tracer.run_id, dispatch_id=dispatch_id,
            )
        decision_record = asdict(decision) if decision is not None else None
        decision_digest = (_record_digest(decision_record)
                           if decision_record is not None else None)
        tracer.event(
            "authorization_gate", sim_t=ws.t, state_digest=state_digest,
            plan_version=graph.version, action_id=action.id,
            action_digest=_action_digest(action), dispatch_id=dispatch_id,
            context=context_record, context_digest=context_digest,
            decision=decision_record, decision_digest=decision_digest,
            decision_status=decision_status, now=now, guard_state=guard_state,
            verdict="ALLOW" if authorization_reason is None else "DENY",
            reason=authorization_reason,
        )
        if authorization_reason is not None:
            m.aborted = True
            break
        policy.reset(action.id)
        tracer.event("dispatch_intent", sim_t=ws.t, state_digest=state_digest,
                     task_id=task.id, task_revision=task_revision,
                     task_digest=task_digest, plan_id=plan_id,
                     plan_version=graph.version, plan_digest=plan_digest,
                     action_id=action.id, action_digest=_action_digest(action),
                     dispatch_id=dispatch_id, approval_digest=approval_digest,
                     approval_revision=approved_decision.revision,
                     approval_decision_id=approved_decision.decision_id,
                     authorization_digest=decision_digest,
                     authorization_decision_id=decision.decision_id,
                     authorization_context_digest=context_digest)
        m.action_count += 1
        dispatch_observation = DispatchObservation(ws, state_digest)

        def execution_fault(error: Exception) -> tuple[Termination, int]:
            """Quarantine uncertain execution; cancellation never waits for logging."""
            dispatch_observation.failed = True
            evidence_complete = not isinstance(error, CoreEvidenceUnavailable)
            stop_before_fault = admission.stop.outcome
            try:
                tracer.event(
                    "execution_fault", sim_t=dispatch_observation.state.t,
                    state_digest=dispatch_observation.digest,
                    plan_version=plan_record["version"], action_id=action.id,
                    action_digest=plan_record["actions"][cursor]["action_digest"],
                    dispatch_id=dispatch_id, phase=dispatch_observation.phase,
                    cycles=dispatch_observation.cycles,
                    error_type=type(error).__name__, execution_outcome="UNKNOWN",
                    deadline=error.record() if isinstance(error, CoreCallTimeout) else None,
                    prior_intervention_id=(stop_before_fault.intervention_id
                                           if stop_before_fault else None),
                )
            except Exception:
                evidence_complete = False
            finally:
                outcome = admission.stop.request_stop(
                    dispatch_id=dispatch_id, reason="execution_fault",
                    pending_execution=(lambda: error.pending)
                    if isinstance(error, CoreCallTimeout)
                    and error.label in {"begin_dispatch", "apply", "end_dispatch"} else None)
            if not dispatch_observation.interruption_recorded:
                try:
                    _stop_latched(
                        admission, tracer, phase="execution_fault",
                        sim_t=dispatch_observation.state.t,
                        state_digest=dispatch_observation.digest,
                        plan_version=plan_record["version"], action=action,
                        dispatch_id=dispatch_id, decision_id=decision.decision_id,
                        expected_generation=authorization_context.stop_generation,
                        observation=dispatch_observation)
                except Exception:
                    evidence_complete = False
            if not evidence_complete or not outcome.trace_complete:
                raise CoreEvidenceUnavailable(
                    "dispatch stopped; execution evidence is incomplete") from error
            return Termination.INTERRUPTED, dispatch_observation.cycles

        try:
            try:
                reason, cycles = dispatch(
                    task, graph, action, world, policy, verifier, numeric, tracer,
                    plan_version=graph.version, profile_digest=profile_digest,
                    task_revision=task_revision, task_digest=task_digest,
                    plan_id=plan_id, plan_record=plan_record, plan_digest=plan_digest,
                    dispatch_id=dispatch_id,
                    approval_digest=approval_digest,
                    approval_revision=approved_decision.revision,
                    approval_decision_id=approved_decision.decision_id,
                    authorization_digest=decision_digest,
                    authorization_decision_id=decision.decision_id,
                    authorization_context_digest=context_digest,
                    authorized_state_digest=state_digest, authorized_sim_t=ws.t,
                    authorized_stop_generation=authorization_context.stop_generation,
                    observation=dispatch_observation,
                    admission=admission,
                    uncertainty_threshold=uncertainty_threshold,
                )
                dispatch_observation.cycles = cycles
                if not dispatch_observation.failed:
                    dispatch_observation.phase = "post_dispatch_observation"
                    after, after_digest = _observe(world, tracer, phase="post_dispatch")
            except Exception as error:
                reason, cycles = execution_fault(error)
        finally:
            try:
                bounded_call("end_dispatch", admission.stop.limits.adapter_s,
                             lambda: world.end_dispatch(dispatch_id), admission.pending_actuation)
            except Exception as error:
                if not dispatch_observation.failed:
                    dispatch_observation.phase = "end_dispatch"
                    reason, cycles = execution_fault(error)
        m.inner_cycles += cycles
        observation_lost = dispatch_observation.failed
        active_execution_evidence_lost = observation_lost
        if observation_lost:
            after, after_digest = dispatch_observation.state, dispatch_observation.digest
        last_valid_ws, last_valid_digest = after, after_digest
        tracer.event("dispatch", sim_t=after.t, plan_version=plan_record["version"],
                     action_id=action.id, action_digest=_action_digest(action),
                     dispatch_id=dispatch_id, plan_id=plan_id,
                     plan_digest=plan_digest,
                     approval_digest=approval_digest,
                     approval_revision=approved_decision.revision,
                     approval_decision_id=approved_decision.decision_id,
                     authorization_digest=decision_digest,
                     authorization_decision_id=decision.decision_id,
                     authorization_context_digest=context_digest,
                     intervention_id=(admission.stop.outcome.intervention_id
                                      if reason is Termination.INTERRUPTED
                                      and admission.stop.outcome else None),
                     stop_status=(admission.stop.outcome.status
                                  if reason is Termination.INTERRUPTED
                                  and admission.stop.outcome else "SAFE_UNCONFIRMED"
                                  if reason is Termination.INTERRUPTED else "NOT_REQUESTED"),
                     state_digest=after_digest,
                     state_scope="LAST_KNOWN" if observation_lost else "OBSERVED",
                     action=str(action), termination=reason.value, cycles=cycles)
        log.info("        inner loop: %s after %d cycle(s)", reason.value, cycles)

        if reason in {Termination.UNCERTAIN, Termination.STALE_CONTEXT,
                      Termination.INTERRUPTED}:
            if reason is Termination.INTERRUPTED and admission.stop.outcome \
                    and admission.stop.outcome.reason == "active_numeric_safety":
                m.safety_rejections += 1
            m.aborted = True
            break

        if reason is Termination.UNSAFE:
            m.safety_rejections += 1
            if not replan_or_abort(
                    after, after_digest,
                    Divergence(action_id=action.id, plan_version=graph.version)):
                break
            continue

        div = check_divergence(action, before, after, plan_version=graph.version)
        tracer.event("divergence", sim_t=after.t, plan_version=graph.version,
                     action_id=action.id, action_digest=_action_digest(action),
                     state_digest=after_digest, before_state_digest=before_digest,
                     magnitude=div.magnitude,
                     unmet=[str(p) for p in div.unmet],
                     surprise=[str(p) for p in div.surprise])
        if div.magnitude > THETA_DIV:
            m.divergences += 1
            log.info("        DIVERGENCE %.2f — unmet: %s",
                     div.magnitude, ", ".join(str(p) for p in div.unmet))
            if div.surprise:
                log.info("        surprise: %s", ", ".join(str(p) for p in div.surprise))
            before_replans = m.replans
            if not replan_or_abort(after, after_digest, div):
                break
            if m.replans > before_replans:
                m.recoveries += 1     # provisional; revoked below if the run aborts
            continue

        log.info("        ok")
        if div.surprise:
            log.info("        surprise (recorded): %s",
                     ", ".join(str(p) for p in div.surprise))
        cursor += 1

    if pre_action_observation_lost:
        final, final_digest = last_valid_ws, last_valid_digest
        terminal_observation = "UNAVAILABLE"
        goal_met = False
    elif active_execution_evidence_lost:
        final, final_digest = after, after_digest
        terminal_observation = "UNAVAILABLE"
        goal_met = False  # no verified goal claim without terminal state
    else:
        final, final_digest = _observe(world, tracer, phase="terminal")
        terminal_observation = "OBSERVED"
        goal_met = holds(bound_goal, final)
    # T9 semantics: when the mission is impossible, correctly aborting IS success.
    m.task_success = (False if terminal_observation == "UNAVAILABLE"
                      else m.aborted if task_record["expect_abort"] else goal_met)
    if m.aborted:
        m.recoveries = 0          # an aborted run recovered from nothing

    tracer.event("episode_end", sim_t=final.t, state_digest=final_digest,
                 goal_met=goal_met, terminal_observation=terminal_observation,
                 stop_status=(admission.stop.outcome.status
                              if admission.stop.outcome else "NOT_REQUESTED"),
                 **m.model_dump())
    log.info("")
    log.info("%s %s  (replans=%d, actions=%d, cycles=%d, steps=%d)",
             task.id, "PASS" if m.task_success else "FAIL",
             m.replans, m.action_count, m.inner_cycles, final.t)
    return m
