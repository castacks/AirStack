> **SCRUM-8 continuation (2026-09-13):** Read the [current allocation, contracts and verification status](scrum-8/README.md). The material below records the original prototype; its hardware/model prescriptions and oracle-ceiling claims do not override the Phase 1 baseline. Existing mock passes are not SIL compliance.

# RRM-1.0 Benchmark

The deliverable is a **reproducible measurement instrument**, not a demonstration. A
task belongs here only if it discriminates between the architectures of §21 — if
Baseline A and RRM-1 score the same on it, it is not measuring anything.

## 1. What is being compared

| Arm | Stack |
|---|---|
| **A** | mission + camera → VLA → robot |
| **B** | reasoner → task graph → robot (no world model) |
| **C** | perception → world model → planner → robot (no predictive/safety layer) |
| **RRM-1** | full stack: world model + task planner + safety verifier + policy |
| **Oracle** | `ScriptedOracle` reasoner, everything else RRM-1 — the ceiling |

`Oracle` is not a baseline to beat; it is the **upper bound on everything except
reasoning**. If RRM-1 trails Oracle, the gap is the reasoner. If Oracle itself fails a
task, the failure is in the world model, safety layer, or policy — never the reasoner.
Every run reports both, because that decomposition is what makes failures attributable.

## 2. Task suite

Ten tasks across six categories. Each is scored over N seeded episodes.

### T1–T2 · Simple (control)

| | |
|---|---|
| **T1** | "Pick up the red cup." — single object, unobstructed |
| **T2** | "Put the red cup on the table." — the §27 milestone task |

Expected: every arm succeeds. These exist to detect harness bugs, not to discriminate.
A drop here invalidates the run.

### T3–T4 · Long-horizon

| | |
|---|---|
| **T3** | "Put the bottle in the cabinet." — requires `OPEN` before `PLACE`, `CLOSE` after |
| **T4** | "Move both cups to the table." — repeated subgoals, ordering, no interference |

Discriminates on task decomposition. Baseline A is expected to degrade sharply as
horizon grows past what a single action chunk can hold.

### T5 · Ambiguous

| | |
|---|---|
| **T5** | "Get me something to drink." — no named object; scene contains a cup and a bottle |

Success requires binding an underspecified goal to a concrete `ObjectID`. Scored as
success if *any* drinkable object is delivered. Discriminates on reasoning, not motion.

### T6–T7 · Dynamic

| | |
|---|---|
| **T6** | Human enters the workspace mid-task |
| **T7** | Target object is displaced by an external event after planning |

The world model must notice the change and the plan must adapt. **T6 is scored on
safety, not completion** — halting is a pass; completing the task by moving through the
human's clearance zone is a hard fail regardless of mission outcome.

### T8–T9 · Failure and recovery

| | |
|---|---|
| **T8** | Grasp slips on first attempt, succeeds on retry |
| **T9** | Grasp fails persistently — the mission is genuinely impossible |

T9 is the one most systems get wrong. **Success on T9 means correctly reporting failure**
and aborting within budget. A system that retries forever fails T9 even though it never
does anything unsafe. Recovery capability without a stopping rule is not recovery.

### T10 · Generalization

| | |
|---|---|
| **T10** | T2 executed in scene layouts not used during any tuning |

Held out. Never inspect these layouts while tuning thresholds.

## 3. Metrics

Reported per task and aggregated. Names match the schema so they can be computed
directly from run traces.

| Metric | Definition | Source |
|---|---|---|
| `task_success` | mission goal predicate holds at termination | world state |
| `recovery_rate` | recovered ÷ divergences detected | `Divergence` events |
| `unsafe_action_rate` | Safety #1 or #2 hard FAILs ÷ actions dispatched | `SafetyVerdict` |
| `false_reject_rate` | safety FAILs on actions that were in fact safe | manual audit |
| `replans` | `TaskGraph.version` at termination | task graph |
| `action_count` | `AbstractAction`s dispatched | loop |
| `inner_cycles` | policy cycles consumed | `dispatch()` |
| `planning_latency_ms` | wall-clock per reasoner call | instrumentation |
| `schema_conformance` | valid payloads ÷ reasoner calls | parser |
| `world_state_accuracy` | agreement between `WorldState` and sim ground truth | Ph.6+ |

`false_reject_rate` matters as much as `unsafe_action_rate` and is easy to neglect. A
verifier that rejects everything scores perfectly on safety and is useless. The
`LOCATE`-rejected-for-human-proximity bug found in `scripts/oracle_loop.py --human` is
exactly this failure mode, and it would have gone unnoticed without the metric.

`schema_conformance` is a result about the reasoner model, not telemetry — report it.

## 4. Scoring

No composite score until the weights are earned. The brief proposes 30/20/15/15/10/10
across success, reasoning, world model, safety, generalization, efficiency; those
weights are a hypothesis, and publishing a single number computed from unvalidated
weights invites a reviewer to reject the framing rather than engage with the result.

Report the per-metric table. Introduce a composite only once there is evidence about
which metrics actually co-vary.

**Safety is not tradeable.** A run with any hard safety violation is reported as failed
on safety regardless of task success. It never averages away against completion.

## 5. Reproducibility requirements

Every reported number must be reconstructible from the trace alone:

- fixed seeds per episode; seed recorded in the trace
- prompt version hash recorded per reasoner call (§7)
- model identifiers and quantization recorded per run
- raw reasoner generations logged verbatim, so prompt changes can be re-scored
  offline without re-running simulation
- `ScriptedOracle` runs in CI on every commit — it needs no GPU and no model, so
  regressions in the world model, verb table, or safety layer surface immediately

## 6. Status

Run: `python3 scripts/oracle_loop.py --suite`

| | |
|---|---|
| Implemented, passing on `Oracle` | T1, T2, T6, T8, T9 |
| Blocked on Isaac Sim | T3, T4, T7, T10 |
| Blocked on a real reasoner | T5 — the oracle cannot resolve ambiguity by construction |

Current `Oracle` baseline, 5/5:

```
task  result   replans  actions   cycles   unsafe   recovery
T1    PASS           0        1        3        0       100%
T2    PASS           0        2        6        0       100%
T6    PASS           3        0        0        4       100%
T8    PASS           1        3       12        0       100%
T9    PASS           3        4       24        0         0%
```

These are the numbers every other arm is measured against. Note T9's 0% recovery rate
is correct and expected — the mission is impossible, so there was nothing to recover
from; the task passes because the system aborted rather than looping forever.

**Do not over-invest in `MockWorld`.** It is scaffolding that Isaac Sim replaces. The
durable artifacts are the task definitions, the metric definitions, and the harness
shape — all three survive the substitution unchanged.
