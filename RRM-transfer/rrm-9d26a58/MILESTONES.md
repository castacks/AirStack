# RRM milestones

## Retained control diagnosis and visual API compatibility — 2026-10-06 UTC

Retained recovery trace identifies a source-backed precursor to the pre-LAND dip:
tracking TF rejection (4096) at LAND−4.2823 s resets PID integrals and emits idle
thrust 0.71, clearing the prior vertical integral contribution ≈0.0916; lower thrust
appears downstream before the dip. The next rejection is ODOM_BEFORE_ACTIVATION (32),
following reset of activation history. All 185 tracking receipts in the preceding
12 s retain a 1 m height reference and zero vertical reference. The actual PID log
confirms a generic transform failure but omits lookup details. TF failure cause and
counterfactual body causation remain unresolved. Fresh external authority does not
prove uninterrupted internal controller admission. Independent reviewer corroborates.

Actual GUI camera-to-private-worker diagnostic contains no teacher facts or dispatch
request. Worker health is ready, but deployed POST `/v1/verify-entities` returns 404;
zero successful inference, no perception score. Actual Isaac uses the standard Office
launcher, not the authored marker fixture; no independent per-frame labels acquired.
Seven prepared cases remain PREPARED_NOT_RUN. Final live ROS observation confirms
connected/disarmed/landed, GUI VERIFIED/inactive, readiness 7/7. No flight, gain,
guard, simulator image or scene changes in this chunk.

Next substantial work: bounded TF lookup diagnostics and reviewed active-authority
fallback policy; separately align worker entity API/model provenance and marker scene
with frame-bound teacher acquisition, then execute scored shadow cases. Stable hover
and broader robustness remain unqualified. Full recorded LAND coverage still stands.


## Current continuation — 2026-10-06 EDT

Recovery-aware capture and bounded canonical-state reacquisition pass 76 tests and
independent review. Grounded final-source rollover verifies four overlapping control
segments and 574 exact clock windows. Actual GUI completes TAKEOFF/NAV1/NAV2, stops
moving NAV3 and withholds later actions; separate LAND verifies in 11.619 s with its
whole recorded action covered by 99 physical samples and all 18 control streams.
Live session retains 1,854 physical samples, 1,852 exact clock comparisons and two
explicit pending tails. Later 1.046 s rollover gap is outside LAND. Final GUI inactive,
vehicle disarmed/landed, capture COMPLETE, readiness 7/7.

A substantial predispatched body-height excursion (1.019 →0.569 →0.878 m) has no
established cause despite fresh inputs and persistent OFFBOARD/authority. Stable hold
and broader robustness stay open. Next: retained controller/reference diagnosis before
more flight, alongside a teacher-labeled proposal-only visual pilot. Seven visual cases
are PREPARED_NOT_RUN; no perception accuracy claim. Preserve guards and provenance.
See [HANDOFF.md](HANDOFF.md) and [vision protocol](docs/scrum-8/vision-evaluation-protocol.md).

### Previous complex GUI mission checkpoint — 2026-10-06 EDT

An actual GUI six-action Office mission completes VERIFIED: takeoff 1 m, four
relative 0.5 m navigation legs, then land. Two moving STOP trials during NAV1/NAV3
halt and withhold later actions; each interrupted mission stays HALTED. Separate
one-action LAND tasks verify once, and fresh final ROS/PhysX confirms grounding.
Both separate LAND descents remain unqualified across harness raw-capture gaps of
54.175/85.850 s. LAND2 first rejects missing canonical vehicle state before dispatch,
then the same unconsumed plan admits once after fresh observation; no action retry.

Unchanged source, image, planner and Office epoch retain 4,085 physical records,
4,079 exact within-segment clock comparisons and six explicit final pending tails.
No physical cap truncation or timing/retention error. Existing 55 tests apply to
unchanged instrumentation; independent review checks plans and behavioral evidence.
Final GUI is VERIFIED/inactive, readiness 7/7, vehicle disarmed/landed, recording
disabled. Next: recovery-aware capture/admission orchestration, grounded validation,
then full descent coverage. Preserve the 0.5 s guard and candidate provenance.
See [HANDOFF.md](HANDOFF.md).

### Previous completed sampling checkpoint — 2026-10-06 EDT

Completed sampling-loop retention and timestamped maxima pass 55 tests, independent
review, actual GUI reload and readiness 7/7. The grounded comparison retains 44,361
control events/18 streams and 1,525 physical records. All 1,524 preceding sampling
loops deliver complete snapshot/encode/write tails; the final tail is explicitly
pending completed. All clock windows agree exactly, with zero timing/retention
errors. No large pause recurs. Span-start attribution separates comparison maxima
from outside-window events; no total-overhead bound or prior-abort cause follows.
The larger payload remains below 64 MiB here; future warmed comparisons need shorter
windows or bounded segments. Next: unchanged-source warmed observation with byte
margin. Final state is disarmed/landed, mission inactive and capture disabled.
Preserve the 0.5 s guard and uncommitted candidate provenance.
See [HANDOFF.md](HANDOFF.md).

A fraction-preserving Pegasus clock candidate passes 22 tests and two actual
Office GUI grounded reloads, both readiness 7/7. All 549 comparison windows
match counter-plus-remainder to observed duration; integer quantization stays
below 1 µs. The child patch is uncommitted and requires publication/pinning for
reproduction from clean checkouts. No new flight qualification. Next is bounded GUI
mission/stop/recovery testing; acquisition alignment and older collapse remain
open. See [HANDOFF.md](HANDOFF.md).

### Previous grounded measurements — 2026-10-05 EDT

Actual grounded physics callback measurement confirms **4.112 ms backend clock
loss over 41.13 simulation seconds**: all 456 recorded windows exactly match
per-step microsecond truncation. Six observer plus ten recorder tests, independent
review, real Office GUI reload and readiness 7/7 pass. Maximum capture receipt gap
was 0.740 s; this is grounded measurement, not flight performance qualification.
Next is a fraction-preserving clock correction with grounded regression. Older
collapse causation, epoch/acquisition registration and broader reliability remain
open. See [HANDOFF.md](HANDOFF.md) for source identity and measurement limits.

Read-only raw timing capture now retains PX4 packet timestamp fields alongside
raw/converted odometry and available timing metadata. Eight tests pass, and the
reviewer reproduced all 731 unique-position associations with unchanged headers.
Fresh live GUI/grounded readiness passes 7/7. Clock-origin/acquisition registration
remains unresolved; approximately 315 ms header/packet differences are not measured
delay, and absent timing status does not imply zero offset. See [HANDOFF.md](HANDOFF.md).

The latest grounded measurement diagnosis adds a tested offline pose comparison.
Height offsets vary between captures, and receipt-phase timestamps do not establish
PX4 acquisition alignment. Legacy sensor-state fields are physical sensor inputs,
not PX4 belief. Fresh Office GUI, grounded capture and readiness 7/7 pass; no new
flight or timing/control change. See [HANDOFF.md](HANDOFF.md) for the next raw-time
instrumentation step and the retained comparison assumptions.

The 2026-10-05 physical-truth continuation VERIFIED one GUI-reviewed Office
TAKEOFF 1 m → forward 0.25 m → left 0.25 m → ordinary LAND/disarm replay,
with navigation errors 0.023108/0.008781 m and no recovery/retry. Direct PhysX
capture confirms motion and return to rest; physical displacement differs from
odometry by about 3 cm, with timestamp/phase uncertainty retained. The new opt-in
AirStack recorder is reviewed, tested and reloaded while grounded; all seven
readiness gates pass. The older Pegasus diagnostic commit remains unavailable.
This qualifies one short two-leg replay, not mixed-height/longer reliability,
collision containment or the older collapse cause. See [HANDOFF.md](HANDOFF.md).

Live GUI continuation at21:10EDT independently repeated the bounded Office
takeoff1m → forward0.25m → ordinary LAND/disarm sequence, navigation error0.018999m.
The GUI's unknown-scene report and implicit initial selection are repaired.
180s live capture observes all16 control streams,658 active admitted PID callbacks,
and no future-tracking rejection with active authority. This is one narrow replay,
not closure of the older collapse or mixed-height/longer qualification gaps.

Commit `e633a658` supersedes the older source-only deployment state below: it
records one independently verified Office takeoff 1 m → forward 0.25 m → LAND
sequence, navigation endpoint error 0.02198 m, and disarm. This narrow success
does not resolve the older forward-collapse cause, mixed-height qualification,
longer missions, general two-leg reliability, or physical containment. Milestones 3–6 remain active.

The current workspace's old Isaac image was replaced with the published digest
containing NumPy 1.26.4. Seven readiness gates and a fresh camera capture pass.
The initial image-reset step added a read-only control recorder and grounded
coverage check without dispatching flight. The later live replay and physical-truth
results above extend that initial checkpoint. See [HANDOFF.md](HANDOFF.md).

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
