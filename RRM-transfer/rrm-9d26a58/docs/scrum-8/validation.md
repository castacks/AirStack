# Implementation and validation — 2026-09-13 UTC

Source baseline: RRM archive `9d26a58eb8516b6754c5d12b041cdc789950e047`, AirStack `ore_proj` at `a6dad8caf54722e5eba3481367e914ce213e6135`. Changes are staged source files, not a new Git revision or published baseline.

## Current cross-embodiment and end-to-end checkpoint — 2026-09-26 UTC

The dependency-light suite passes **271/271**. Eight tests validate an
embodiment-neutral qualitative `GoalRequest`, capability routing across aerial, ground,
and manipulation profiles, the distinction between unsupported and unavailable
resources, and provenance-bound adapter parameters. Three subsequent tests validate
fail-closed translation of one selected route into the existing C01 `TaskRequest` and
composition through the Kuka-Allegro C01–C05 shadow adapter. They do not dispatch an
action; the resulting plan retains semantic-only feasibility references.

Three GUI/HTTP tests cover the separate Kuka-Allegro goal-preview panel: a supported
red-block request persists hash-addressed goal, route, C01, C04, and C05 evidence; an
under-evidenced blue-block request holds without a plan; and unsupported text or an
unknown target fails before preview artifacts are created. The response and GUI expose
`numeric_feasibility_verified=false`, `execution_dispatch=false`, and
`simulator_action_sent=false`.

This test count is a software-contract result, not a live task-success measurement.
The direct GUI execution path still uses its deterministic aerial command grammar and
bounded inter-action reconciliation; only the separate non-executing hand preview
consumes the neutral goal contract. Neither path performs general semantic replanning.
Historical live evidence includes verified GUI takeoff/land missions, one learned
navigation timeout followed by verified landing, and a takeoff regression that ended
`RECOVERED_HALT` after a 4.076 m recovery altitude excursion. The tracking-lead and
landing-boundary fixes were subsequently verified in Office with 0.005 m takeoff
horizontal displacement. A later `warehouse-shelves` regression failed its takeoff
bound and exposed delayed airborne state after the mission process exited. The mission
supervisor now monitors that uncertain terminal state and admits the predeclared
recovery landing only after repeated fresh evidence proves meaningful flight. That
sequence has deterministic coverage but not a new authorized live regression:
**Office baseline flight is qualified; Warehouse aerial execution remains paused.**
The Kuka-Allegro Gate 4/5 probes remain bounded calibration infrastructure and have
not completed a semantic hand task.

See [goal-to-finish performance status](end-to-end-status.md). No aggregate live rate
can be recomputed because the referenced gitignored runtime artifacts are absent in the
current workspace.

## Independent watchdog thread checkpoint — 2026-09-25 UTC

`start_watchdog(interval_s)` launches a daemon thread that periodically calls the
existing `watchdog_fence()` method. If the fence trips, the thread calls
`watchdog_check()` to durably latch the fault and then exits. The thread never
acquires the simulator/action lock, never calls the articulation, and never writes
the journal directly.

A CPU-only blocking-fake test proved the watchdog thread independently detected
staleness and tripped the fence while a fake `apply_action` call remained blocked.
Motion authority was false before release, one durable fault record was written, a
hold was applied after release, and the thread shut down cleanly via `stop_watchdog()`.
A separate healthy-path test confirmed the thread stays running when ticks are timely
and rejects double-start. The focused suite passes 19/19.

This is an in-process daemon thread, not an out-of-process monitor. It delegates all
fault handling to the existing fence/check code paths. Live action-applying stop/hold
evidence under Isaac scheduler load remains unqualified.

## Asynchronous hand stop fence checkpoint — 2026-09-25 UTC

`request_stop()` now sets a logical motion fence and durably logs the stop request
without acquiring the simulator/action lock. This allows stop requests to return quickly
even if a native `apply_action` call is blocked. The simulator thread asynchronously
reconciles the stop request, clearing pending work and queuing a position hold.
Exact-generation deduplication prevents the same stop request from queuing multiple holds.
Unmatched durable stop requests are reconstructed as inhibited holds upon restart.

A CPU-only blocking-fake test demonstrated the stop returned in under 50 ms while
the fake tick remained blocked, the logical motion fence was set, and the completion
subsequently applied a fake hold. The full dependency-light suite passes 238/238.
This remains an in-process logical fence and does not qualify physical stop under
simulator load, stop-to-hold latency, or live motion.

## Asynchronous watchdog-fence checkpoint — 2026-09-25 UTC

Tick timing is now published behind a heartbeat lock independent of the lock held
across simulator actions. `watchdog_fence()` evaluates clock/tick staleness, sets an
atomic logical fence, and returns without acquiring the simulator/action lock, calling
the articulation, or touching the journal. Motion-enabled reporting and subsequent
admission/apply decisions honor the fence. When the simulator thread returns, normal
completion persists the first liveness fault, clears work, and queues a hold when work
may have been active.

A CPU-only blocking-fake test held `apply_action` open while the watchdog returned in
under 50 ms. The fake tick was still blocked, logical motion authority was already
false, and the action count had not increased beyond the in-progress fake call. After
release, completion wrote exactly one fault and the next fake tick applied the required
hold. Focused gateway tests pass 15/15 and the complete dependency-light suite passes
238/238; changed Python compiles and `git diff --check` pass. No simulator or robot
runtime was touched.

This is not a physical stop or separately deployed watchdog. It cannot interrupt an
already-executing native simulator call, and the fault becomes durable only after that
call returns. Live stop-to-hold, active-articulation timing, safe state, controller,
contact, and motion remain unqualified.

## Completed-tick deadline checkpoint — 2026-09-25 UTC

The isolated gateway now fails closed when a completed simulator-thread tick took
longer than `max_tick_gap_s`. `TICK_DEADLINE_EXCEEDED` is durably recorded, motion is
disabled, queued work and safe-state evidence are cleared, and a hold is queued only
when gateway work may have been active. A tick-completion clock regression uses the
same durable latch. This prevents a long tick from appearing healthy merely because
its completion timestamp is fresh.

CPU-only fake-clock tests cover an overlong disabled idle tick with zero articulation
calls, an overlong action-applying fake tick followed by a required fake hold, restart
inhibition, single fault persistence, and tick-completion clock regression. The full
dependency-light suite passes 237/237; changed Python compiles and `git diff --check`
pass. No simulator, ROS graph, hand, or aircraft was touched. Detection occurs after
the simulator call returns; an independent watchdog capable of acting while a call is
blocked, live stop/hold, and active-articulation timing remain unqualified.

## Inactive-asset Isaac physics-callback checkpoint — 2026-09-25 UTC

The isolated live probe verified the 23-joint Kuka-Allegro profile and zero initial
velocity, removed it from the temporary World registry, and deactivated the asset prim
before physics stepping. Isaac delivered 240/240 callbacks at a declared 120 Hz
simulation step. Every disabled gateway tick returned `IDLE`; liveness ended healthy,
the asset was inactive on every callback, motion and execution remained false, and the
hard wrapper counted zero articulation actions. Wall-clock callback gap was 0.196 ms
median, 0.329 ms p95, and 3.116 ms maximum; callback duration was 6.66 us median,
14.71 us p95, and 47.68 us maximum. The accepted report is
`.rrm-artifacts/hand-live-callback-20260925-d/report.json`.

A zero-gravity-only diagnostic failed the no-state-change gate with 0.279244 rad drift,
demonstrating that zero gravity does not neutralize the asset's authored/reset drive
behavior. It is not accepted evidence. The final inactive-asset fixture sent no stop,
hold, or action and left the active Office process/container identity unchanged. Full
suite: 234/234; changed Python compiles and `git diff --check` passes. This qualifies
callback registration/accounting only, not an active articulation, scheduler load,
watchdog deployment, controller, safe state, stop latency, contact, or motion.

## Live no-action idle-heartbeat checkpoint — 2026-09-25 UTC

A second headless Isaac process, running inside the existing runtime container with its
own `SimulationApp` and `World`, bound the disabled gateway to the live Kuka-Allegro
articulation. Strict comparison found zero mismatches across all 23 joint profiles.
All 1,000 direct ticks returned `IDLE`; final liveness was `HEALTHY`, motion stayed
disabled, and `_NoActionArticulation` counted zero `apply_action` calls. External tick
durations were 2.14 us median, 2.53 us p95, and 18.12 us maximum against a 0.1 s
declared limit. The immutable report is
`.rrm-artifacts/hand-live-heartbeat-20260925-a/report.json`.

The active Office process was neither restarted nor stepped by the probe, and its
container identity/start time remained unchanged and running. The full suite passes
232/232; changed Python compiles and `git diff --check` passes. This probe invoked
ticks directly and did not register an Isaac physics callback, step physics after
reset, request stop, or apply a hold. It is therefore not scheduler-under-load,
independent-watchdog, stop-to-hold, safe-state, controller, or motion qualification.

## Hand gateway liveness checkpoint — 2026-09-25 UTC

The isolated gateway now emits immutable heartbeat and stop-to-hold timing evidence.
An external watchdog latches stale ticks, missed stop deadlines, and clock regression,
fences new work, and durably preserves the fault across restart without calling the
articulation itself. Fake-clock validation measured a 0.020 s timely hold against a
0.050 s deadline and a 0.051 s deadline breach. A 1000-tick idle stress plus 100
concurrent reads made zero articulation calls and produced one durable fault record.
Focused tests pass 21/21 and the complete dependency-light suite passes 231/231;
changed Python compiles and `git diff --check` passes. No live simulator or robot was
touched. Live Isaac scheduler and action-applying stop evidence remain unqualified.

## Signed hand-authority checkpoint — 2026-09-25 UTC

The isolated hand boundary now verifies HMAC-SHA256 grants from configured issuers and
allowed roles. Grants are short-lived and bind subject, purpose, authority epoch, stop
generation, and an exact scope digest over fresh safe evidence or the complete dispatch
decision/context/command/profile. Their IDs are durably consumed before reset,
reconciliation, or dispatch intent and reconstructed after restart. Focused tests cover
tampering, issuer/role/purpose/scope/epoch/generation mismatches, expiry, maximum
lifetime, concurrency, and replay; 17/17 pass. The full dependency-light suite passes
227/227. Changed Python compiles and `git diff --check` passes. No simulator or robot
was touched. This local shared-key contract still requires production identity, secure
key provisioning/rotation, and preferably asymmetric or protected-service verification.

## Hand restart reconciliation checkpoint — 2026-09-25 UTC

CPU-only fake-articulation tests now cover durable replay of stop generations,
consumed IDs, and unresolved dispatches; mandatory post-restart hold; fresh five-sample
safe-state evidence; append-before-clear reconciliation; stale/unauthorized rejection;
and retained deduplication after a clean restart. Reconciliation leaves both layers
motion-inhibited and boundary admission still requires a separate reset. The focused
suite passes 16/16 and the complete dependency-light RRM suite passes 226/226. Changed
Python sources compile and `git diff --check` passes. No simulator, ROS graph, hand, or
aircraft was touched; authenticated authority and live stop-liveness qualification are
still missing.

## Current checkpoint — 2026-09-24 UTC

The complete RRM unit suite passed **199/199** before one bounded aerial-console
takeoff/land regression. Readiness, canonical state, sensor/map freshness, and actual
takeoff/land server checks passed. The console's fresh-container task-discovery
packaging issue was repaired locally by staging the `rrm` dependency; no image was
published. The nominal 1 m takeoff **failed** after 0.805 m lateral displacement in
1.24 s; the recovery-land pre-sample was at 3.205 m. Predeclared contingency landing
was verified at z=0.000725 m, connected and disarmed. Mission state was
`RECOVERED_HALT`; there was no retry. This is transport/safety regression evidence,
not RRM research evidence. Raw records are under
`.rrm-artifacts/command-requests/fbb4f424eddb433fa8616cf383246141/` at the
AirStack root.

The [hand embodiment decision](hand-embodiment-decision.md) selects Kuka-Allegro for
a future controlled tabletop stage. An isolated no-command USD probe found 23 joints
and three matching bare-articulation reset hashes. The separate tabletop no-action
probe now adds a table, two dynamic blocks, tray, and overhead RGB camera. Its latest
run (`hand-tabletop-probe-20260924-f`) produced three matching rounded joint/object
state hashes, frame/state/episode-paired captures, and explicit false controller,
ROS, and dispatch flags. The simulator teacher export contains only 12 `exists`,
`kind`, and `localized` facts. A manually reviewed, image-only candidate for the same
hash-bound frame scored 9 exact facts of 12 (precision 9/9, recall 9/12); it is not
model inference and establishes no grasp/contact/safe-state fact. A proposal-only
C01–C05 hand shadow fixture separately produced nine hash-linked records, a
context-grounded `GRASP` → `PLACE` plan, and a **synthetic** visual-score protocol
result of 5/9 recall. C06 admission, complete C09 evidence, and bounded hand execution
remain disconnected. The isolated controller prerequisite now passes a strict
five-gate evaluator: bounded arm error 0.00330 rad, all 23 velocity peaks inside
unchanged limits, three matching post-command reset hashes, a named calibration
contact after a zero baseline, a safe window, and a 0.917 s independent stop. The
injected contact peaked at 489 N and is observation-channel evidence only—not
stable-contact or grasp evidence. The evaluator allows preparation of one bounded
contact trial while explicitly keeping execution, contact stability, grasp
qualification, and C06/C08/C09 completion false. The complete dependency-light RRM
suite now passes **206/206** tests.

## Direct GUI command execution — 2026-09-21

The console now performs deterministic command-to-public-task translation and serial
simulator execution for takeoff, land, exploration, map waypoint routes, and relative
movement. It dynamically discovers actual action servers rather than trusting client
advertisements, binds relative commands to fresh pose/heading, requires fresh canonical
flight state, and requires a fresh map-frame VDB feed for exploration. STOP requests
public action cancellation; missing physical-stop evidence remains unconfirmed.

Live read-only validation against `full_default` observed fresh `map -> base_link`
odometry, connected grounded vehicle state, a fresh VDB point cloud, and real servers
for TakeoffTask, LandTask, NavigateTask, FixedTrajectoryTask, and ExplorationTask.
SemanticSearchTask was correctly excluded because only a client exists. No goal was
sent. Python compilation passed and the complete CPU suite passed **169/169** tests.

## Current validation checkpoint — 2026-09-20 UTC

The later expanded flight-profile increment adds distinct fail-closed paths for
targetless `TAKEOFF` and exact supplied multi-waypoint `NAVIGATE`. It does not alter the
historical live probe below. Synthetic contract/provider validation now covers
adapter-owned takeoff parameters, grounded/disarmed takeoff state, Takeoff/Land stop
endpoints, complete route binding, per-segment corridor evidence, route truncation and
targetless authorization. The complete suite passes **156 tests**; changed Python
sources compile and `git diff --check` passes. No live task was dispatched for this
increment.

### Original single-waypoint live probe

The body-agnostic live-mission composition now includes dynamic C03 feasibility,
dependency-bound single-use C06 admission, bounded public-task dispatch, independent
effect verification, and re-observation/replanning. The AirStack Office adapter is a
read-only deterministic provider for one straight, single-waypoint `NAVIGATE` action;
it uses current vehicle/controller/planner state, action endpoints and a map-frame
Ouster corridor rather than a learned model as safety authority.

A live read-only probe observed all required channels and task endpoints. It returned
blocking evidence for the current state: grounded/disarmed/no-control, planner-stuck,
and approximately 0.391 m minimum observed clearance against the 0.4 m threshold. No
goal or vehicle command was sent. The complete suite passed **148 tests**; standalone
runner/provider launch was checked without inherited `PYTHONPATH`; Python compilation
and `git diff --check` passed. This validates contracts and the non-dispatching probe,
not an admitted navigation flight or general collision-free route planner.

## Implemented increment

[`rrm/contracts.py`](../../rrm/contracts.py) adds independent primitives for C02 evidence truth, C03 declared semantic support, C06 immutable context/single-use admission and C08 stop-generation/reset checks. A fresh authority epoch prevents permits from a previous guard instance being reused after reset. The initial guard starts inhibited. These primitives do not change `rrm/loop.py`, `rrm/verbs.py`, the oracle, mock safety or the Isaac stubs.

Trusted supervision must supply validated safety decisions and fresh safe-state evidence. The guard does not authenticate callers or move/stop a robot. Its lock protects only local admission bookkeeping; transport/adapter checks, current-state synchronization, durable deduplication and independent stop delivery remain required. Capability support does not validate physical limits, required operation/resource combinations or permission. The old loop still has the handoff's uncertainty, capability, permission and active-stop gaps.

## Results

Verification ran inside the existing AirStack robot image `v0.20.8_robot-x86-64_dev`, with explicit CPU runtime `runc`, 2 CPU / 2 GiB limits, and a source bind mount. Container-local Pydantic 2.13.5 satisfied the original dependency. No GPU/model package or weight was downloaded.

| Check | Result | Interpretation |
| --- | --- | --- |
| Original oracle suite | 5/5 passed: T1, T2, T6, T8, T9 | Regression baseline only; T6 begins with a human present, not entry during motion |
| Original relation inference | PASS | Geometry helper self-test, not an Isaac integration test |
| New contract unit tests | 14/14 passed | Unknown negation, capability/profile handling, immutable inputs, stale context, expiry/nonallow, duplicate admission, stop/reset, restart epoch |
| Architecture allocation | 11/11 rows linked to owner, contract, scenario and planned evidence | Documentation coverage, not requirement satisfaction |
| GPU execution | NOT REQUESTED | This transfer workspace was deliberately submitted with `gpu: 0`; its NVML failure is not an OSMO infrastructure finding |
| Integrated S01–S10 SIL | NOT RUN | No running Isaac scene or conforming adapter/supervisor |

Reproduce CPU checks inside a container with source mounted at `/workspace/rrm` and Pydantic v2 installed:

```sh
cd /workspace/rrm
python3 scripts/oracle_loop.py --suite --trace-dir /evidence/traces
python3 simulation/isaac_backend.py
python3 -m unittest discover -s tests -v
```

Source/evidence checksums, container identity/digest, dependency versions, timestamps and raw test output accompany the export bundle. Runtime files from the input archive were compared byte-for-byte; only the new contract module was added. The original input ZIP and checksum files remain intact.

## Remaining work

Review the architecture against Phase 1; implement task/context interaction and evidence-backed state; connect capability-aware planning, safety/permission authority, monitoring and replayable telemetry; select and validate the hand scene; restore remote GPU access; run the planned deterministic SIL campaign before comparing model candidates. Model selection and adapter numeric/safe-state limits need measured feasibility. No SCRUM-8 Done transition or external publication was made.

## Continuation — 2026-09-17 UTC

`rrm/task_contracts.py` now adds proposal-only C01/C04/C05 records for immutable task
revisions, scoped clarification/approval interactions, interpreted intents and versioned
plans. Plans require a ready intent with matching task/state/capability references,
authored verb arity, unique action IDs and acyclic in-plan dependencies. The records do
not implement C06 admission, C07 dispatch, ROS transport, a controller command or any
execution authority. Five new unit tests passed, alongside all 24 shadow/contract/unit
tests and the 5/5 Oracle regression suite. The active Isaac process remains the Pegasus
drone baseline; no scene, controller, Foxglove client or RRM execution path was changed.

`rrm/state_contracts.py` now adds C02 records for immutable fact evidence and
task-relevant snapshots. A fact resolves as TRUE or FALSE only from fresh,
non-contradictory explicit evidence. Missing, stale, UNKNOWN, contradictory and negated
UNKNOWN evidence resolve to UNKNOWN; coverage metadata does not make an absent fact
negative evidence. This layer is deliberately separate from the legacy mock
`WorldState`, does not establish a closed-world rule, and creates no adapter or dispatch
surface. Five state-contract tests passed; the total unit suite is now 29/29 and the
Oracle regression remains 5/5.

`rrm/airstack_drone.py` and `scripts/airstack_drone_dispatch.py` now add the RRM
output seam for AirStack's existing public `task_msgs` actions. Typed proposals map only
to `/{robot}/tasks/takeoff`, `/navigate` and `/land`; navigation accepts robot-local
`map` waypoints. Dry-run is the default and does not import ROS. `--execute` is
required to create one corresponding ActionClient and send one goal. The adapter does
not use direct PX4, MAVROS, service, publisher or trajectory-control paths, and it is
not a completed C06/C08 supervisor. Four adapter tests passed; the total unit suite is
now 33/33 and the Oracle regression remains 5/5. No task goal was sent during this work.

## Controlled public-task-action SIL evidence — 2026-09-17 UTC

With explicit user approval, RRM submitted a bounded two-goal sequence to the already
running Isaac/PX4 drone through the existing AirStack task-action interface. The runner
used temporary in-container paths `/tmp/rrm-sim-adapter` and `/tmp/rrm-sim-deps` because
the RRM source is not a robot-container mount; this did not modify the AirStack image,
scene, Foxglove, PX4 or source workspace. Dry-run in that identical environment printed
the exact proposed `TakeoffTask(2.0 m, 1.0 m/s)` and `LandTask(1.0 m/s)` goals with
`execution_requested: false`.

After verifying one live server for each of `/robot_1/tasks/takeoff` and
`/robot_1/tasks/land`, preserving the image's existing ROS `PYTHONPATH`, and importing
both `rclpy` and `task_msgs.action.TakeoffTask`, execution produced these task-server
terminal results in order:

| Goal | Result evidence |
| --- | --- |
| `takeoff-1` — `/robot_1/tasks/takeoff` — 2.0 m at 1.0 m/s | `success: true`, `takeoff complete` |
| `land-1` — `/robot_1/tasks/land` — 1.0 m/s | `success: true`, `landing complete` |

The first, incorrectly staged execution invocation raised `ModuleNotFoundError:
rclpy` before it constructed an ActionClient, so it sent no goal. The corrected
invocations used the adapter's single `ActionClient` task route only; they did not use
direct PX4, MAVROS, ROS publishers, ROS services or trajectory interfaces. This is
transport-level SIL evidence only. It needs externally observed state/pose agreement,
negative-path trials, C06 admission, C08 independent stop proof and repeated trials
before it can support an autonomy or flight-readiness conclusion.

## Read-only post-flight observation and outcome-verifier increment — 2026-09-17 UTC

An eight-second `scripts/airstack_shadow.py` run after the controlled sequence produced
`observation_complete: true` at final snapshot eight with dispatch inhibited. Its fresh
independent state was `map -> base_link`, position `(-0.502, -0.328, 0.020) m`, MAVROS
connected, and `armed: false`. Two snapshots transiently reported incomplete evidence,
so the record retains freshness rather than claiming that every sample was complete.
This supports the final landed-state interpretation but was collected after the task
results and therefore does not prove per-goal causality.

To close that correlation gap for any separately approved future run,
`verify_drone_outcome` now requires a pre-dispatch sample fresh at dispatch time and a
post-result sample fresh at completion time. It is ROS-free and returns `VERIFIED`,
`MISMATCH` or `UNCONFIRMED`; takeoff checks absolute map altitude within 0.3 m and
connected/armed state, landing checks near-ground altitude and connected/disarmed state.
The optional dispatcher verification mode has read-only odometry/state subscriptions,
the pre-existing explicit task-action gate, and optional JSON evidence output. No live
goal was sent to validate this code increment; its focused synthetic outcome tests are
recorded with the general regression results.

## Body-agnostic drone decision bridge — 2026-09-17 UTC

`rrm/drone_decision.py` now demonstrates the missing **decision-to-proposal** seam
without adding execution authority. It accepts a C01 `TaskRequest`, C02
`StateSnapshot`, and C03 `CapabilityDeclaration`; the deterministic baseline recognizes
only `navigate to <semantic-target-id>`. It produces C04 intent and C05 plan records
that retain the target ID, authored `NAVIGATE_TO` semantics and revision references.
Only the AirStack drone embodiment adapter owns its map-frame waypoint binding and can
then emit the existing typed `DroneTaskProposal` for `/robot_1/tasks/navigate`.

`scripts/rrm_drone_decision.py` provides a JSON-in/JSON-out dry-run entry point for
that pipeline; it is intentionally separate from `airstack_drone_dispatch.py` and has
no ROS or execution option. This makes model-free capstone evaluation possible now:
freeze the task, evidence snapshot, capability profile and target binding as input
artifacts, compare decisions, then pass a proposal to later supervision only after the
relevant C06/C08 gates exist.

Fresh explicit `localized(target)` evidence and the declared generic `NAVIGATE_TO`
operation plus an available `airframe` resource are required. Missing/stale state holds;
unknown target text requests clarification; unsupported profile or wrong embodiment
refuses. All non-ready outcomes contain no plan or proposal. The bridge imports neither
ROS nor an action client and does not call the separately gated dispatcher. Its four
focused unit tests, the complete 40-test suite, and the legacy 5/5 oracle suite passed
inside the current AirStack robot container. Read-only ROS inspection confirmed the
live endpoint remains `/robot_1/tasks/navigate [task_msgs/action/NavigateTask]`; no
goal was sent.

## Ground-truth world-builder teacher pipeline — 2026-09-17 UTC

`rrm/ground_truth.py` now turns explicitly labelled simulator facts into immutable C02
`StateSnapshot` records. It distinguishes simulator provenance from future sensor,
inferred/VLM and operator evidence; stores the latest evidence only per `(source,
fact)`; rejects received-time regression within a source; preserves cross-source
disagreement so a query resolves to `UNKNOWN`; and clears all belief at an explicit new
simulation episode. The convenience entity API emits semantic `exists`, `kind` and
optional `localized` facts only. It contains no Isaac, ROS, geometry, physics, control
or model dependency.

Four focused tests show a fresh labelled target drives the existing proposal-only RRM
bridge, while stale, contradictory, removed and prior-episode target states withhold a
proposal. The full unit suite passed 44/44, with the 5/5 Oracle regression unchanged.
This is a **teacher/scoring pipeline**, not a claim that MAVROS or a simulator label is
perception. A controlled, read-only scene-specific label extractor must be the next
adapter; the later VLM path must emit the same C02 schema and be evaluated against this
teacher rather than replacing it.

## Learned Cosmos Reason2 RRM cognition — 2026-09-17 UTC

`rrm/cosmos_reason2.py` and `scripts/rrm_cosmos_reason2.py` now form the learned,
body-agnostic C01/C02/C03-to-C04/C05 boundary. The runner sends only an immutable task,
semantic evidence snapshot and declared capability profile to a locally cached Cosmos
Reason2 model, then records the raw response and either a validated intent/semantic plan
or a replayable refusal. It has no ROS, simulator-control, action-client, safety-admission,
geometry, physics/IK or execution-dispatch surface.

On PSC Bridges-2, the local `nvidia/Cosmos-Reason2-8B` revision
`a9fae2cf89dc64db96b12860417f0eb403013bb9` ran on an H100 80GB allocation against the
labelled navigation episode. An initial correct-looking response was rejected solely
because it omitted explicit `grounded_entities`; the contract deliberately did not infer
that missing provenance from the action target. After a prompt-only schema repair, a
second real inference produced `candidate_status=ACCEPTED`, which means the candidate
validated into C04/C05. The persisted record states `execution_dispatch=false`; no
AirStack goal or control action was made.

The complete RRM unit suite passed 51/51 and the legacy Oracle regression passed 5/5
after the repair. This is one text-plus-ground-truth cognition evaluation, not evidence
that the model visually understands an Isaac scene. The next increment must produce
C02 from image/video evidence and score it against this ground-truth teacher before
testing reasoning robustness across the scene-difficulty ladder.

## Visual C02 evidence and persistent PSC evaluation seam — 2026-09-17 UTC

`rrm/visual_world_builder.py` now validates a VLM's catalog-bound visual claims into
the same C02 schema used by the simulator teacher. Each admitted claim is explicitly
`INFERRED`, names a task-scoped known entity only, and carries a durable media source
reference plus SHA-256 digest; unknown IDs, unsupported predicates, malformed JSON and
missing manifest information are refusals. It does not infer unobserved facts, produce
C04/C05 plans, access Isaac/ROS, or control a vehicle.

`scripts/rrm_cosmos_visual_grounding.py` invokes the cached Cosmos model on one frozen
image or video, retains its raw output, emits the validated visual snapshot and can
score it fact-by-fact against a frozen ground-truth C02 snapshot. The score reports
exact matches, mismatches, missed teacher facts and extra candidate facts with explicit
precision/recall denominators; missing VLM evidence remains a miss/UNKNOWN rather than
being converted to a negative claim. `scripts/psc_rrm_cosmos_visual.sbatch` is the
unattended H100 route: it writes persistent PSC logs/results and survives a laptop
disconnect. Unit coverage is now 56/56 and the legacy Oracle remains 5/5. A real
visual-performance result is still pending one selected, frozen Isaac capture and
matching label bundle.
