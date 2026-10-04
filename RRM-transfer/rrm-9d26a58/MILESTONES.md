# RRM milestones

Status as of 2026-10-04. This is the current progress view for RRM. It replaces
Jira/SCRUM labels as the planning vocabulary for new work. Earlier Jira records and
`SCRUM-*` documents remain historical requirement, design, and evidence records; do
not rename or reinterpret their captured results.

## At a glance

| Milestone | Status | What this means now |
| --- | --- | --- |
| Milestone 1 — Needs and CONOPS baseline | **Complete** | The operational needs, use cases, and concept of operations baseline are established. |
| Milestone 2 — Requirements and traceability baseline | **Complete** | The system requirements baseline and its traceability to implementation and verification are established. |
| Milestone 3 — System architecture and interfaces | **Active** | The architecture and logical contracts are specified. The CPU/mock core implements deterministic planning, three-valued state evidence, capability/permission/approval checks, symbolic and numeric safety gates, single-use admission, stop/fault handling, bounded callbacks, and strict trace replay. The dependency-light suite passes 429/429 tests (including 14 simulator preflight/admission tests) and the Oracle baseline is 5/5. This is component evidence, not an integrated safety or robot-performance result. |
| Milestone 4 — Simulation, SIL, and HIL verification pipeline | **Partially demonstrated; instrumented flight failed** | Actuation/authority repair enabled OFFBOARD/lift, but takeoff breached the lateral bound. Hold/LAND accepted, ground/disarm observed, GUI halted. Full ROS/PX4 evidence retained; control/containment remain unqualified. Next: source-only control-path investigation. Ordinary flight remains paused. |
| Milestone 5 — V&V, evidence, and readiness gates | **Active** | Authored event-scoped labels, replay, manifests, and strict evidence reconstruction are implemented. Moving sidecar disk synchronization to attempt finalization produced two fresh 34-attempt exports, each with 34/34 expectation matches and 32/34 complete attempts; intentional trace loss stays UNKNOWN. Earlier 31/34 results remain retained; callback scheduling and storage liveness remain open. No integrated acceptance matrix, external adjudication, or readiness-level claim is complete. |
| Milestone 6 — Safety, risk, and change control | **Active** | Core admission, stop/fault containment, and fail-closed evidence handling are implemented in the mock boundary. Durable simulator-image provenance, physical stop qualification, and a formal change-control process remain open. |

## Current focus

Single instrumented Office diagnostic completed (2026-10-04 01:49 EDT), following
reviewed staging and fresh grounded admission. Effective scaling1.0/readiness7/7
enabled observed armed OFFBOARD and lift. Takeoff failed its 0.3m lateral bound;
ROS takeoff-window maximum altitude estimate0.504m, displacement0.326m at outcome.
Hold/LAND accepted; ground/disarm independently observed; GUI HALTED/inactive.
No retry, tuning, restart, commit or push. Complete180s ROS/PX4 evidence retained.

Next: source-only investigation of tracking/frame alignment, command saturation
and sharp downward excursion, plus OFFBOARD/control still selected after disarm.
The mode-only has_control predicate explains that final authority flag. Its
armed-aware source fix passes actual-plugin transition/rearm regressions; not
deployed. Saved-flight correlation confirms PID/cascade/raw-command math and
observational ENU/NED agreement. Early vertical lag remains unexplained; eight
idle/reset samples include six candidate30ms-future tracking associations. The
reason-coded PID admission diagnostics/isolated clock-order tests are now complete
in source, preserving strict freshness.11gtests,26lifecycle phases and6deterministic
clock phases pass.30msfuture inputs produce explicit rejection reasons and clean
recovery on clock catch-up; this does not prove the earlier flight cause. Next:
separately admitted grounded deployment/read-only coverage checks; no tuning,
runtime change or new flight in this checkpoint.
ROS/PX4 positions are estimates, not independent scene ground truth. Recovery
observation is not physical containment or nominal takeoff/LAND qualification.
All final integrals zero; PID0.71 baseline remains unqualified. Do not use the
[qualification plan](docs/scrum-8/office-control-qualification.md) for another flight
until diagnosis/repair/testing/review and new authorization. Earlier negative
attempts remain retained. Office/Warehouse flight stays paused, within Milestones3–6.
See [handoff](HANDOFF.md).

1. Resolve evidence-write/sidecar liveness without relaxing deadlines or discarding
   incomplete attempts.
2. Make the OSMO Isaac environment reproducible and fail-fast on invalid image
   dependencies before a mission can be started.
3. Preserve the core's scene-independent boundary: geometry, controller limits, and
   qualification stay in versioned adapter/profile evidence, not core scene branches.

Do not expand RRM-EM into routing, GUI selection, online learning, or execution while
these milestones are open. Do not use a green readiness report as scene-physics or
flight qualification.

## Current supporting work

| Workstream | Status | Boundary |
| --- | --- | --- |
| Core evidence-bound RRM | **Active** | Continue deadline, reconstruction, and sidecar-liveness work without relaxing bounds or dropping incomplete attempts. |
| Reproducible simulator environment | **Actuation deployed; control qualification failed** | Digest-pinned Isaac retains NumPy1.26.4; effective child1.0 and authority permit OFFBOARD/lift. Instrumented diagnostic failed lateral bound and recovered to observed ground/disarm. Source-only control-path investigation next; durable robot-image provenance and wider scene/flight qualification remain open. |
| Embodiment-neutral goal path | **Prototype only** | Proposal-only qualitative routing and the Kuka-Allegro preview have CPU coverage. No semantic `GRASP`/`PLACE` goal has completed through live reasoning, admission, actuation, and observation. RRM-EM remains advisory and has no execution authority. |
| Controlled architecture/model comparison | **Not started** | The reporting instrument supports matched same-source runs; distinct parity-controlled arms and independently adjudicated integrated outcomes are still required. |
| Integrated acceptance and portability | **Not started** | A version-pinned campaign must cover nominal, ambiguity, stale-state, unsafe-route, failure, stop, and recovery cases for multiple embodiment profiles before general goal-to-finish or cross-embodiment claims. |

## Evidence and decision sources

- [Current handoff](HANDOFF.md) — dated checkpoints, exact evidence, and open gaps.
- [Goal-to-finish status](docs/scrum-8/end-to-end-status.md) — scope of live evidence
  and claim boundaries.
- [Core acceptance campaign](docs/core-acceptance.md) and
  [paired comparison](docs/core-comparison.md) — current mock-evaluation definitions.
- [Command-console runbook](docs/scrum-8/command-console.md) — active aerial command
  path and operational safety boundaries.
- [Embodiment-learning architecture](docs/embodiment-learning-architecture.md) —
  advisory RRM-EM boundary.

## Updating this document

Update a milestone only when its linked evidence has been retained and replayed where
applicable. Keep failed, unknown, and evidence-incomplete attempts visible. A change
from **Partially demonstrated** to **Complete** requires the milestone's stated
acceptance evidence, not a passing unit suite, a ready service, or an action-server
success response alone.

When importing a historical Jira item, translate its scope into the appropriate
milestone and preserve its original issue ID only as provenance. Do not create new
`SCRUM-*` work items.
