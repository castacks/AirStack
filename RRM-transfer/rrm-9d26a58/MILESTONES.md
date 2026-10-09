# RRM engineering milestones

These are the historical engineering-lifecycle categories recorded on 2026-10-04.
They classify requirements/design/verification work; their numbers are separate
from the revisable stages in [WORK_PHASES.md](WORK_PHASES.md).
Completed needs/requirements baselines do not establish completed runtime evaluation.

| Milestone | Status | Meaning at that checkpoint |
| --- | --- | --- |
| Milestone 1 — Needs and CONOPS baseline | **Complete** | The operational needs, use cases, and concept of operations baseline are established. |
| Milestone 2 — Requirements and traceability baseline | **Complete** | The system requirements baseline and its traceability to implementation and verification are established. |
| Milestone 3 — System architecture and interfaces | **Active** | The architecture and logical contracts are specified. The CPU/mock core implements deterministic planning, three-valued state evidence, capability/permission/approval checks, symbolic and numeric safety gates, single-use admission, stop/fault handling, bounded callbacks, and strict trace replay. The dependency-light suite passes 429/429 tests (including 14 simulator preflight/admission tests) and the Oracle baseline is 5/5. This is component evidence, not an integrated safety or robot-performance result. |
| Milestone 4 — Simulation, SIL, and HIL verification pipeline | **Partially demonstrated; instrumented flight failed** | Actuation/authority repair enabled OFFBOARD/lift, but takeoff breached the lateral bound. Hold/LAND accepted, ground/disarm observed, GUI halted. Full ROS/PX4 evidence retained; control/containment remain unqualified. Next: source-only control-path investigation. Ordinary flight remains paused. |
| Milestone 5 — V&V, evidence, and readiness gates | **Active** | Authored event-scoped labels, replay, manifests, and strict evidence reconstruction are implemented. Moving sidecar disk synchronization to attempt finalization produced two fresh 34-attempt exports, each with 34/34 expectation matches and 32/34 complete attempts; intentional trace loss stays UNKNOWN. Earlier 31/34 results remain retained; callback scheduling and storage liveness remain open. No integrated acceptance matrix, external adjudication, or readiness-level claim is complete. |
| Milestone 6 — Safety, risk, and change control | **Active** | Core admission, stop/fault containment, and fail-closed evidence handling are implemented in the mock boundary. Durable simulator-image provenance, physical stop qualification, and a formal change-control process remain open. |

For current demonstrated behavior, see [performance status](docs/scrum-8/end-to-end-status.md).
The detailed [handoff](HANDOFF.md) retains dated runs, source identity, failed
attempts and unresolved questions. Later checkpoints supersede this snapshot's
then-current deployment status.

Update engineering milestone completion only against its requirement-backed evidence.
Use the working roadmap to organize next steps without treating stage numbers as
new requirements or replacing the RRM architecture.
