# RRM working roadmap

These six stages are a useful planning outline discussed on 2026-10-08, not an
approved requirements baseline or a fixed definition of RRM. Revise or overlap
them when the evidence and project direction warrant it.

RRM's overall purpose is reusable, embodiment-independent reasoning over contextual
goals, world/task memory, capabilities and constraints, with observed outcomes and
bounded recovery or assistance. The stages below support that purpose: recovery
makes evaluation credible; visual grounding supplies tested state evidence; learned
execution and replanning exercise the reasoning loop; comparison measures its value;
another actual embodiment tests portability.

[Architecture and requirements allocation](docs/scrum-8/architecture.md),
[interface contracts](docs/scrum-8/interfaces.md), user direction and verified
evidence govern the work. This roadmap organizes it; it grants no execution authority.

| Phase | Work | Proposed exit check |
| --- | --- | --- |
| **1. Complete recovery evaluation** | Keep recording through STOP, review, fresh-state admission and separate landing. | Full descent evidence, verified grounding, no unexplained capture gaps. |
| **2. Evaluate visual grounding** | Feed fresh camera observations into entity grounding; test ambiguity, occlusion, displaced targets and stale images against separate simulator truth. | Correct target identification, measured uncertainty, and rejection of unsupported/stale evidence. |
| **3. Connect learned reasoning to execution** | Exercise a qualitative goal such as “inspect the blue marker”: visual context → Cosmos reasoning → proposed plan → feasibility/safety → public task action. | Observed target achievement through the complete lifecycle, beyond the deterministic movement parser. |
| **4. Close the observation/replanning loop** | After each action, observe again, compare expected effects, update memory and continue, replan or request assistance. | Changed targets and execution failures produce appropriate bounded recovery; completion requires fresh evidence. |
| **5. Run comparative robustness campaigns** | Freeze scenarios, seeds and assessment rules; compare learned RRM with oracle/baselines and ablations on held-out layouts. | Defensible success, recovery, grounding and latency distributions, including failed/incomplete runs. |
| **6. Extend embodiment coverage** | Carry the same core contracts into manipulation or another actual adapter. | Verified physical outcomes with the shared reasoning core; profile-only tests remain preliminary. |

## Progress and next work

See [current performance status](docs/scrum-8/end-to-end-status.md) for the evidence
and gaps. Isolated v3 grounding identifies both markers in one development frame;
negative-case identification and Office binding remain open. The frozen qualitative
goal→proposal check overlaps stage 3 but exposes unsupported inspection semantics
and missing physical target binding, not achieved inspection or flight. Next work
should connect a genuinely supported goal to independent fresh localization and
admission, without declaring stages 1–3 complete. A service response or unit test
alone does not establish a completed integrated result.

## Numbering

Engineering “Phase 1” refers to the historical needs/requirements baseline;
engineering milestones 1–6 are lifecycle categories. Hand-adapter gates G0–G7 and
original prototype phases are separate historical sequences. C01–C09 are interface
contracts and S01–S10 are evaluation scenarios. None is numerically equivalent to
this working outline. Dated observations retain their original scope and outcomes.
