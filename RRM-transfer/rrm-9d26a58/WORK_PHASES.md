# RRM working roadmap

These six stages are a useful planning outline discussed on 2026-10-08, not an
approved requirements baseline or a fixed definition of RRM. Revise or overlap
them when the evidence and project direction warrant it. The 2026-10-15 scope
clarification below makes backend/sensor qualification distinct from RRM research.

RRM's overall purpose is reusable, embodiment-independent reasoning over contextual
goals, world/task memory, capabilities and constraints, with observed outcomes and
bounded recovery or assistance. The stages below support that purpose: recovery
makes evaluation credible; observation grounding supplies tested state evidence;
execution and replanning exercise the reasoning loop; comparison measures its value;
another actual embodiment tests portability.

[Architecture and requirements allocation](docs/scrum-8/architecture.md),
[interface contracts](docs/scrum-8/interfaces.md), user direction and verified
evidence govern the work. This roadmap organizes it; it grants no execution authority.

| Phase | Work | Proposed exit check |
| --- | --- | --- |
| **1. Complete recovery evaluation** | Keep recording through STOP, review, fresh-state admission and separate landing. | Full descent evidence, verified grounding, no unexplained capture gaps. |
| **2. Evaluate visual grounding** | Feed fresh camera observations into entity grounding; test ambiguity, occlusion, displaced targets and stale images against separate simulator truth. | Correct target identification, measured uncertainty, and rejection of unsupported/stale evidence. |
| **3. Connect reasoning to a qualified execution backend** | Exercise a supported qualitative goal: contextual state → reasoning → proposed plan → feasibility/safety → backend skill. A classical controller, VLA or hybrid can provide the skill. | Observed target achievement through the complete lifecycle, beyond the deterministic movement parser; identify the reasoner/backend actually tested. |
| **4. Close the observation/replanning loop** | After each action, observe again, compare expected effects, update memory and continue, replan or request assistance. | Changed targets and execution failures produce appropriate bounded recovery; completion requires fresh evidence. |
| **5. Run comparative robustness campaigns** | Freeze scenarios, seeds and assessment rules; compare learned RRM with oracle/baselines and ablations on held-out layouts. | Defensible success, recovery, grounding and latency distributions, including failed/incomplete runs. |
| **6. Extend embodiment coverage** | Carry the same core contracts into manipulation or another actual adapter. | Verified physical outcomes with the shared reasoning core; profile-only tests remain preliminary. |

## Progress and next work

### Scope clarification — 2026-10-15

RRM measures robust task reasoning around an existing robot architecture; it does
not require depth cameras, a depth model, a VLA, RL training or a particular planner.
Semantic evidence and capability/feasibility/outcome contracts cross the embodiment
boundary; sensors, metric grounding and motor policies remain behind it. Unknown
evidence stays unknown. Sensor-neutral is not evidence-free, and declaring a skill
is not qualifying it. RRM-EM estimates scoped capability reliability; training a
motor policy is separate backend work, not a prerequisite for the reasoning core.

Stages may proceed in parallel. In particular, frozen semantic-state and recorded
proposal tests can evaluate reasoning-boundary behavior without completing physical
localization or flight. They cannot establish learned reasoning, integrated task
success or cross-robot performance. The October stereo experiments qualify only
one AirStack geometry path; they are not a universal RRM prerequisite.

Next research comparison should hold task corpus, observation evidence, reasoner
model, execution backend, budgets and independently assessed outcomes fixed while
varying the explicit RRM checking/monitoring/recovery treatment. A capable closed-loop
baseline retains feedback/recovery opportunities; permission and protective stop
remain enforced in every executable arm. Switching from a classical backend to a
VLA at the same time would confound controller gains with reasoning robustness.
See the [controlled comparison proposal](docs/scrum-8/retriever-positioning.md#next-comparative-experiment).

The contradiction-grounding regression is a bounded step toward that qualification:
the same authored proposals and C02 snapshots are replayed before/after a parser
fix, with no learned-model or controller change. This is not the full comparison.
The [matched-tape shadow campaign](docs/core-comparison.md#matched-tape-shadow-checking-campaign)
extends instrument qualification to 24 authored cases / 27 steps and two checking
ablations. The same feedback tape and common stop/permission/deadline filters remain
fixed; no physical backend is executed. Wrong-valid-target acceptance remains in
the original unbound controls; a fifth variant rejects it using an independently
supplied, revision-bound semantic intent. A restricted navigation-template reference
now derives bindings independently from fresh facts and a finite caller-owned entity
scope. General qualitative interpretation remains open. The real-worker development
probe initially returned 6/12 cases before a transport error (3/6 valid-contract
intent matches). An approved missing-only continuation returned the remaining six
(1/6 matches; all six plans withheld), completing cross-session development coverage;
see [coverage and limits](docs/core-comparison.md#independent-intent-development-probe).
This development regression does not complete
the learned closed-loop campaign or establish general intent understanding.

### Retained aerial integration gaps

See [current performance status](docs/scrum-8/end-to-end-status.md) for the evidence
and gaps. Isolated v3 grounding identifies both markers in one development frame;
negative-case identification and Office binding remain open. The frozen qualitative
goal→proposal check overlaps stage 3 but exposes unsupported inspection semantics
and missing physical target binding, not achieved inspection or flight. Next work
for the current metric AirStack navigation adapter should connect a genuinely
supported goal to independent fresh localization and admission, without declaring
stages 1–3 complete. This adapter-specific requirement does not prescribe the
implementation of another backend. A service response or unit test
alone does not establish a completed integrated result.

## Numbering

Engineering “Phase 1” refers to the historical needs/requirements baseline;
engineering milestones 1–6 are lifecycle categories. Hand-adapter gates G0–G7 and
original prototype phases are separate historical sequences. C01–C09 are interface
contracts and S01–S10 are evaluation scenarios. None is numerically equivalent to
this working outline. Dated observations retain their original scope and outcomes.
