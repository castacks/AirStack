"""The RRM control loop. See docs/architecture.md §4.

Outer loop plans and verifies; inner loop executes until expected effects hold."""

from __future__ import annotations

import hashlib
import json
import logging
import math
import time

from .schema import (
    AbstractAction, ActionPolicy, Divergence, Predicate, RunMetrics, Task,
    TaskGraph, Termination, WorldBackend, WorldState,
)
from .verbs import expected_effects_of, holds
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


def _validate_graph_identity(graph: TaskGraph, *, expected_version: int) -> None:
    """Reject ambiguous planner output before any action can be dispatched."""
    if graph.version != expected_version:
        raise ValueError(
            f"plan version {graph.version} does not match expected {expected_version}"
        )
    action_ids = [action.id for action in graph.nodes]
    if any(not action_id.strip() for action_id in action_ids):
        raise ValueError("plan action IDs must be nonempty")
    if len(action_ids) != len(set(action_ids)):
        raise ValueError("plan action IDs must be unique within a plan version")


def _uncertainty_allows(ws: WorldState, threshold: float, tracer: Tracer, *,
                        phase: str, plan_version: int | None = None,
                        action: AbstractAction | None = None) -> bool:
    passed = ws.uncertainty <= threshold
    fields = {
        "sim_t": ws.t,
        "phase": phase,
        "uncertainty": ws.uncertainty,
        "threshold": threshold,
        "verdict": "PASS" if passed else "FAIL",
    }
    if action is not None:
        fields.update(
            plan_version=plan_version,
            action_id=action.id,
            action_digest=_action_digest(action),
        )
    tracer.event("uncertainty_gate", **fields)
    return passed


def dispatch(action: AbstractAction, world: WorldBackend, policy: ActionPolicy,
             numeric: NumericSafetyVerifier, tracer: Tracer, *,
             plan_version: int,
             uncertainty_threshold: float = THETA_UNC) -> tuple[Termination, int]:
    """Inner loop: run the policy until the action's expected effects hold.

    Termination is by **effect satisfaction**, not by timeout and not by the policy
    signalling completion — GR00T emits no completion signal. The world model already
    knows what should become true (the verb table said so), so it polls for exactly
    that. Timeout is the failure path, not the normal one.
    """
    effects = expected_effects_of(action)
    world.begin_dispatch(action)

    for cycle in range(1, CYCLE_BUDGET + 1):
        ws = world.observe()
        if not _uncertainty_allows(
                ws, uncertainty_threshold, tracer, phase="dispatch",
                plan_version=plan_version, action=action):
            return Termination.UNCERTAIN, cycle - 1
        if effects and all(holds(e, ws) for e in effects):
            return Termination.COMPLETE, cycle - 1

        traj = policy.step(action, ws)
        verdict = numeric.verify(traj, ws)
        tracer.event("safety2", sim_t=ws.t, plan_version=plan_version,
                     action_id=action.id, action_digest=_action_digest(action), cycle=cycle,
                     verdict=verdict.verdict,
                     violations=[v.model_dump() for v in verdict.violations])
        if verdict.verdict == "FAIL":
            for v in verdict.violations:
                log.info("        SAFETY#2 FAIL [%s] %s", v.check, v.detail)
            return Termination.UNSAFE, cycle

        world.apply(action, traj)
        tracer.event("apply", sim_t=ws.t, plan_version=plan_version,
                     action_id=action.id, action_digest=_action_digest(action), cycle=cycle,
                     terminal=traj.terminal, max_velocity=traj.max_velocity)

    ws = world.observe()
    if not _uncertainty_allows(
            ws, uncertainty_threshold, tracer, phase="dispatch",
            plan_version=plan_version, action=action):
        return Termination.UNCERTAIN, CYCLE_BUDGET
    if effects and all(holds(e, ws) for e in effects):
        return Termination.COMPLETE, CYCLE_BUDGET
    return Termination.TIMEOUT, CYCLE_BUDGET


def run(task: Task, world: WorldBackend, reasoner: ReasonerBackend,
        verifier: SafetyVerifier, policy: ActionPolicy,
        numeric: NumericSafetyVerifier,
        tracer: Tracer | None = None, *,
        uncertainty_threshold: float = THETA_UNC) -> RunMetrics:
    """Outer loop. Returns measurements, not a verdict — see docs/benchmarks.md §3."""
    m = RunMetrics(task_id=task.id)
    mission = task.mission
    tracer = tracer or Tracer(None, {})
    if not math.isfinite(uncertainty_threshold) \
            or not 0.0 <= uncertainty_threshold <= 1.0:
        raise ValueError("uncertainty_threshold must be finite and within [0,1]")

    ws = world.observe()
    if not _uncertainty_allows(
            ws, uncertainty_threshold, tracer, phase="planning"):
        m.aborted = True
        m.task_success = task.expect_abort
        tracer.event("episode_end", sim_t=ws.t, goal_met=False, **m.model_dump())
        return m
    t0 = time.perf_counter()
    graph = reasoner.plan(mission, ws)
    _validate_graph_identity(graph, expected_version=0)
    m.planning_latency_ms += (time.perf_counter() - t0) * 1000
    tracer.event("plan", sim_t=ws.t, version=graph.version,
                 reasoner=reasoner.name, latency_ms=round(m.planning_latency_ms, 3),
                 nodes=[str(n) for n in graph.nodes],
                 actions=[_trace_action_record(n) for n in graph.nodes],
                 goal=str(task.goal))
    log.info("PLAN v%d: %s", graph.version, " -> ".join(str(n) for n in graph.nodes))

    cursor = 0

    def replan_or_abort(state: WorldState, div: Divergence, *,
                        rejected_action: AbstractAction | None = None) -> bool:
        """Returns True if the loop should continue, False to abort."""
        nonlocal graph, cursor
        if m.replans >= REPLAN_BUDGET:
            log.info("        replan budget exhausted -> ABORT")
            m.aborted = True
            return False
        m.replans += 1
        t = time.perf_counter()
        previous_version = graph.version
        graph = reasoner.replan(mission, state, graph, div)
        _validate_graph_identity(graph, expected_version=previous_version + 1)
        dt = (time.perf_counter() - t) * 1000
        m.planning_latency_ms += dt
        tracer.event("replan", sim_t=state.t, version=graph.version,
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

    while cursor < len(graph.nodes):
        action = graph.nodes[cursor]
        ws = world.observe()
        if not _uncertainty_allows(
                ws, uncertainty_threshold, tracer, phase="pre_action",
                plan_version=graph.version, action=action):
            m.aborted = True
            break
        log.info("")
        log.info("  [t=%d] %s", ws.t, action)
        log.info("        expect: %s",
                 ", ".join(str(e) for e in expected_effects_of(action)) or "—")

        verdict = verifier.verify(action, ws)
        tracer.event("safety1", sim_t=ws.t, plan_version=graph.version,
                     action_id=action.id, action_digest=_action_digest(action),
                     action=str(action),
                     verdict=verdict.verdict, checked=verdict.checked,
                     violations=[v.model_dump() for v in verdict.violations])
        if verdict.verdict == "FAIL":
            m.safety_rejections += 1
            for v in verdict.violations:
                log.info("        SAFETY#1 FAIL [%s] %s", v.check, v.detail)
            if not replan_or_abort(
                    ws, Divergence(action_id=action.id, plan_version=graph.version),
                    rejected_action=action):
                break
            continue
        log.info("        safety PASS (%s)", ", ".join(verdict.checked))

        before = ws
        policy.reset(action.id)
        m.action_count += 1
        reason, cycles = dispatch(
            action, world, policy, numeric, tracer, plan_version=graph.version,
            uncertainty_threshold=uncertainty_threshold,
        )
        m.inner_cycles += cycles
        after = world.observe()
        tracer.event("dispatch", sim_t=after.t, plan_version=graph.version,
                     action_id=action.id, action_digest=_action_digest(action),
                     action=str(action), termination=reason.value, cycles=cycles)
        log.info("        inner loop: %s after %d cycle(s)", reason.value, cycles)

        if reason is Termination.UNCERTAIN:
            m.aborted = True
            break

        if reason is Termination.UNSAFE:
            m.safety_rejections += 1
            if not replan_or_abort(
                    after, Divergence(action_id=action.id, plan_version=graph.version)):
                break
            continue

        div = check_divergence(action, before, after, plan_version=graph.version)
        tracer.event("divergence", sim_t=after.t, plan_version=graph.version,
                     action_id=action.id, action_digest=_action_digest(action),
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
            if not replan_or_abort(after, div):
                break
            if m.replans > before_replans:
                m.recoveries += 1     # provisional; revoked below if the run aborts
            continue

        log.info("        ok")
        if div.surprise:
            log.info("        surprise (recorded): %s",
                     ", ".join(str(p) for p in div.surprise))
        cursor += 1

    final = world.observe()
    goal_met = holds(task.goal, final)
    # T9 semantics: when the mission is impossible, correctly aborting IS success.
    m.task_success = m.aborted if task.expect_abort else goal_met
    if m.aborted:
        m.recoveries = 0          # an aborted run recovered from nothing

    tracer.event("episode_end", sim_t=final.t, goal_met=goal_met, **m.model_dump())
    log.info("")
    log.info("%s %s  (replans=%d, actions=%d, cycles=%d, steps=%d)",
             task.id, "PASS" if m.task_success else "FAIL",
             m.replans, m.action_count, m.inner_cycles, final.t)
    return m
