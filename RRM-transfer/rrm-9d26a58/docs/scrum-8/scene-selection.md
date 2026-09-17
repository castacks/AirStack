# Scene difficulty and reasoning-evaluation ladder

AirStack's scene catalog is an evaluation resource, not a reason to make flight
control the purpose of RRM. The scene supplies observations, distractors, geometry and
change; RRM is assessed on whether it interprets context, represents uncertainty,
chooses a permitted task-level alternative, asks when necessary, and accounts for its
decision. AirStack task actions are a bounded embodiment harness for testing whether a
proposal can be realized—not the success metric by themselves.

## Keep three difficulties separate

Do not call a test “hard” just because the world renders slowly. Record these axes
separately and vary one at a time:

| Axis | What increases it | Primary RRM question |
| --- | --- | --- |
| Semantic/contextual | Similar targets, natural-language modifiers, task history, constraints | Did R identify the intended entity/goal or request clarification? |
| World-state/observability | Occlusion, stale/missing channel, viewpoint limits, contradictory evidence | Did W retain provenance and make uncertainty block a claim? |
| Planning/change | Alternate routes, moved targets, blocked paths, introduced hazards, recovery options | Did P/M invalidate stale work and select a bounded alternative or assistance? |
| Embodiment/control | Vehicle dynamics, collision margins, controller latency, reset reliability | Can E/S realize an already-approved proposal safely? |
| Compute/simulation | Asset load, RTX sensor cost, real-time factor, scene scale | Is the experiment repeatable with recorded resource use? |

The first three are the primary reasoning variables. The last two are measured
constraints; they must not silently turn a control or rendering failure into a claim
about RRM reasoning.

## Available Isaac Sim ladder

All shortnames below are already in AirStack's
[`simulation/scenes.yaml`](../../../../simulation/scenes.yaml). Do not restart the active
Isaac instance merely to browse them; run each in a separately recorded, controlled
launch after the test task and reset method are fixed.

| Tier | Scene candidates | Difficulty purpose | Recommended reasoning tests | Main caution |
| --- | --- | --- | --- | --- |
| 0 — contract fixture | `flat-plane`, `black-gridroom`, `curved-gridroom` | Near-zero semantic and geometry ambiguity | C01–C05 serialization, stale/unknown evidence, profile refusal, trace replay | It cannot demonstrate contextual or visual reasoning. |
| 1 — bounded semantics | `simple-room`, `office`, `hospital` | Small, inspectable geometry with rooms, doors and object-like landmarks | Contextual target choice, ambiguity, viewpoint/occlusion, constraint interpretation | Existing stage assets are not automatically labeled task entities; begin with an explicit simulator-ground-truth entity shim, then evaluate perception separately. |
| 2 — structured navigation | `warehouse`, `warehouse-shelves`, `full-warehouse` | Repeated structures, longer paths, visual distractors and route alternatives | Multi-step subgoals, target disambiguation, moved-target re-grounding, recovery | Separate visual similarity from route complexity; fixed layouts/seeds are essential. |
| 3 — dynamic change | `warehouse-forklifts`; Pegasus people example in a controlled scene | Moving obstacles/agents and timing-sensitive state invalidation | S02 displacement, S06 protective response, observation freshness, assistance under ambiguity | Dynamics add safety and controller variance; first test with motion inhibited and injected state changes. |
| 4 — operational-scale transfer | `abandoned-factory`, `construction-site`, `chemical-plant`, `retro-neighborhood`, or the Fire Academy import example | Large-scale layout, long horizons, unseen scene transfer and richer visual context | Held-out layout generalization, long-horizon task memory, model comparison | Asset load, stage scale, lighting and reset are confounds. Fire Academy currently has a separate import script and historical sensor-offset caveat. |

`abandoned-warehouse-day/night` are useful matched illumination variants once Tier 2
works: use identical semantic task placements where possible, then report lighting as a
perception shift rather than a reasoning success/failure by itself.

## Recommended campaign order

1. **Tier 0:** Freeze task/state/capability artifacts and verify the same decision,
   trace and refusal are reproducible. This is the deterministic-reference condition,
   not the research result.
2. **Tier 1:** Use `office` or `simple-room` with two visually or linguistically
   confusable target markers. Evaluate contextual grounding and `NEEDS_CLARIFICATION`
   with motion disabled. Then repeat using camera/VLM-derived evidence alongside the
   ground-truth shim; score both separately.
3. **Tier 2:** Move the same task grammar to `warehouse-shelves`. Add a target displaced
   after planning and an alternate permitted route. The evaluation is stale-plan
   rejection/replanning and evidence quality, not whether PX4 happens to follow a path.
4. **Tier 3:** Add one controlled moving actor or forklift-related obstruction after
   state/telemetry are reliable. Exercise hold/assist/stop semantics before authorizing
   any task action.
5. **Tier 4:** Hold out public industrial stages for transfer, visual generalization and
   robustness. Freeze camera setup, scene revision, stage scale, lighting, spawn/reset
   pose, GPU allocation and seeds in each manifest.

For each tier, run the same deterministic reference, candidate learned reasoner(s),
and an ablation without the relevant context/evidence. Report task-intent accuracy,
clarification precision/recall, stale-state rejection, recovery quality, evidence
completeness, latency and failures. Flight completion alone is never the score.

## Model role

A VLM can turn scene images plus task language into candidate entities, relations and
uncertainty for W, and an LLM/VLM/hybrid can propose C04 intent/C05 plans. A separate
post-processor may validate **schema and references**, but should not “repair” a model
silently: an invalid or uncertain candidate must be visible as a refusal/clarification
case. S remains deterministic in the narrower sense of applying declared policy to an
exact proposal and current evidence; that is a safety boundary, not a replacement for
reasoning.

Before downloading a 33 GB candidate model, create a selection record with its exact
revision/license, persistent cache location, VRAM/RAM/latency measurement, input/output
schema, prompt/config hash, expected failure modes, and the Tier 1–4 comparison it must
win. We should cache weights persistently after that decision; repeatedly downloading
them into temporary storage at each OSMO boot makes both the measurement and the
environment non-reproducible.
