"""The RRM control loop. See docs/architecture.md §4.

Outer loop plans and verifies; inner loop executes until expected effects hold."""

from __future__ import annotations

import logging
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
                     after: WorldState) -> Divergence:
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
    return Divergence(action_id=action.id, unmet=unmet, surprise=surprise,
                      magnitude=magnitude)


# ---------------------------------------------------------------------------
# The loop  (docs/architecture.md §4)
# ---------------------------------------------------------------------------

THETA_DIV = 0.01
REPLAN_BUDGET = 3
CYCLE_BUDGET = 6        # inner-loop cycles before a subtask is declared stalled


def dispatch(action: AbstractAction, world: WorldBackend, policy: ActionPolicy,
             numeric: NumericSafetyVerifier, tracer: Tracer) -> tuple[Termination, int]:
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
        if effects and all(holds(e, ws) for e in effects):
            return Termination.COMPLETE, cycle - 1

        traj = policy.step(action, ws)
        verdict = numeric.verify(traj, ws)
        tracer.event("safety2", sim_t=ws.t, action_id=action.id, cycle=cycle,
                     verdict=verdict.verdict,
                     violations=[v.model_dump() for v in verdict.violations])
        if verdict.verdict == "FAIL":
            for v in verdict.violations:
                log.info("        SAFETY#2 FAIL [%s] %s", v.check, v.detail)
            return Termination.UNSAFE, cycle

        world.apply(action, traj)
        tracer.event("apply", sim_t=ws.t, action_id=action.id, cycle=cycle,
                     terminal=traj.terminal, max_velocity=traj.max_velocity)

    ws = world.observe()
    if effects and all(holds(e, ws) for e in effects):
        return Termination.COMPLETE, CYCLE_BUDGET
    return Termination.TIMEOUT, CYCLE_BUDGET


def run(task: Task, world: WorldBackend, reasoner: ReasonerBackend,
        verifier: SafetyVerifier, policy: ActionPolicy,
        numeric: NumericSafetyVerifier,
        tracer: Tracer | None = None) -> RunMetrics:
    """Outer loop. Returns measurements, not a verdict — see docs/benchmarks.md §3."""
    m = RunMetrics(task_id=task.id)
    mission = task.mission
    tracer = tracer or Tracer(None, {})

    ws = world.observe()
    t0 = time.perf_counter()
    graph = reasoner.plan(mission, ws)
    m.planning_latency_ms += (time.perf_counter() - t0) * 1000
    tracer.event("plan", sim_t=ws.t, version=graph.version,
                 reasoner=reasoner.name, latency_ms=round(m.planning_latency_ms, 3),
                 nodes=[str(n) for n in graph.nodes], goal=str(task.goal))
    log.info("PLAN v%d: %s", graph.version, " -> ".join(str(n) for n in graph.nodes))

    cursor = 0

    def replan_or_abort(state: WorldState, div: Divergence) -> bool:
        """Returns True if the loop should continue, False to abort."""
        nonlocal graph, cursor
        if m.replans >= REPLAN_BUDGET:
            log.info("        replan budget exhausted -> ABORT")
            m.aborted = True
            return False
        m.replans += 1
        t = time.perf_counter()
        graph = reasoner.replan(mission, state, graph, div)
        dt = (time.perf_counter() - t) * 1000
        m.planning_latency_ms += dt
        tracer.event("replan", sim_t=state.t, version=graph.version,
                     latency_ms=round(dt, 3), trigger=div.model_dump(),
                     nodes=[str(n) for n in graph.nodes])
        log.info("        REPLAN v%d: %s", graph.version,
                 " -> ".join(str(n) for n in graph.nodes) or "(empty)")
        cursor = 0
        return bool(graph.nodes)

    while cursor < len(graph.nodes):
        action = graph.nodes[cursor]
        ws = world.observe()
        log.info("")
        log.info("  [t=%d] %s", ws.t, action)
        log.info("        expect: %s",
                 ", ".join(str(e) for e in expected_effects_of(action)) or "—")

        verdict = verifier.verify(action, ws)
        tracer.event("safety1", sim_t=ws.t, action_id=action.id, action=str(action),
                     verdict=verdict.verdict, checked=verdict.checked,
                     violations=[v.model_dump() for v in verdict.violations])
        if verdict.verdict == "FAIL":
            m.unsafe_actions += 1
            for v in verdict.violations:
                log.info("        SAFETY#1 FAIL [%s] %s", v.check, v.detail)
            if not replan_or_abort(ws, Divergence(action_id=action.id)):
                break
            continue
        log.info("        safety PASS (%s)", ", ".join(verdict.checked))

        before = ws
        policy.reset(action.id)
        m.action_count += 1
        reason, cycles = dispatch(action, world, policy, numeric, tracer)
        m.inner_cycles += cycles
        after = world.observe()
        tracer.event("dispatch", sim_t=after.t, action_id=action.id,
                     action=str(action), termination=reason.value, cycles=cycles)
        log.info("        inner loop: %s after %d cycle(s)", reason.value, cycles)

        if reason is Termination.UNSAFE:
            m.unsafe_actions += 1
            if not replan_or_abort(after, Divergence(action_id=action.id)):
                break
            continue

        div = check_divergence(action, before, after)
        tracer.event("divergence", sim_t=after.t, action_id=action.id,
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


