# RRM remote Codex handoff

## LATEST: GUI commands now execute through AirStack planning tasks — 2026-09-21

This supersedes the proposal-only GUI status below. The command console now saves a
movement command without requiring the Office camera/catalog, discovers actual ROS
action **servers** and current flight state in the active `robot_1` configuration,
compiles the command into typed public AirStack tasks, records the exact plan before
launch, executes tasks serially, verifies each result from fresh causal state, and
halts the sequence on failure or STOP/HOLD.

Supported direct text covers takeoff, land, bounded-duration exploration/survey,
single or multiple robot-local map waypoints, and current-heading-relative
forward/back/left/right/up/down motion. Navigation or exploration from the ground
automatically prepends takeoff. Unsupported text and missing executors fail without
dispatch. Command saving and task compilation are scene-independent, so every catalog
scene and manual Isaac stage configuration uses the same path; scene switching is
blocked during an active mission.

The implementation deliberately reuses AirStack's ownership boundaries:

- `ExplorationTask` activates the configured VDB-aware global planner, which produces
  collision-checked plans and repeatedly delegates to `NavigateTask`.
- `NavigateTask` owns the active DROAN local planner and trajectory-controller cascade.
- `TakeoffTask` and `LandTask` own vertical transitions.
- RRM never publishes trajectories or sends PX4/MAVROS commands directly.

New files are `rrm/airstack_command.py`, `scripts/airstack_task_discovery.py`, and
`scripts/airstack_command_mission.py`; the typed proposal/dispatcher, console, UI, and
tests were extended accordingly. Exploration admission also requires a fresh
map-frame VDB point-cloud feed. Live read-only discovery found fresh state and real
takeoff, land, navigate, fixed-trajectory, and exploration servers. It correctly
excluded `SemanticSearchTask`, which has a client but no server in `full_default`.
Therefore arbitrary object-language commands such as "find the red chair" remain
non-executable until a real semantic-search/grounding executor is served; the GUI does
not fabricate that capability.

The GUI now treats the compiled plan and subsequent planner publications as evidence,
not a per-plan approval gate. The single **Plan and run** click starts execution; two
bounded, scrollable panels show the exact typed action sequence and recent structured
task/planner events. The dispatcher records each distinct post-dispatch
`/robot_1/global_plan` publication with its frame, endpoints, waypoint count, path
length, timestamp, and digest, so exploration replans become visible while the mission
continues automatically. STOP/HOLD remains independently available.

Takeoff plans now also contain a predeclared, digest-bound public `LandTask`
contingency. When takeoff returns terminal success but fresh causal evidence reports a
physical mismatch while the vehicle is connected, armed, and above 0.3 m, the requested
sequence halts and the contingency lands. The landing is independently verified and
reported as `RECOVERED_HALT`; an unverified recovery is `RECOVERY_FAILED`. Unknown or
timed-out takeoff state never causes a blind second command.

The 2026-09-21 drift incident was admitted after Isaac had been recreated at 02:22 UTC
while the robot container and its tracking state remained from 23:35 UTC. The takeoff
server formerly preferred its retained tracking point as the trajectory origin and
checked only altitude for completion. RRM now rejects that container/clock ordering;
`airstack ready` exposes the same epoch gate. `TakeoffTask` anchors at current odometry
and aborts/holds if horizontal displacement exceeds the production 0.3 m bound. The
robot container was restarted after Isaac while grounded, the new package was built,
the parameter is live at 0.3, and readiness passes. A subsequent bounded operator-run
mission took off toward 1.0 m and landed normally: both actions were independently
`VERIFIED`, the takeoff endpoint was 0.182 m horizontally from its start (below the
0.3 m abort bound), and MAVROS confirmed the final connected, disarmed, on-ground
state. The contingency landing was not needed.

Validation: **177 RRM tests passed**, the modified C++ package builds, changed Python
sources compile, `airstack ready` passes all gates, live task/VDB
discovery passed, and the bounded takeoff/landing mission above passed. The completed
GUI command, replanning-evidence, WebRTC, and takeoff-safety work is checkpointed on
`ore_proj`.

## LATEST: separate takeoff and multi-waypoint route gates — 2026-09-20

The AirStack feasibility adapter now preserves the legacy one-waypoint navigation
profile and adds an opt-in `office-airframe-v2` / `office-bounded-flight-v2` profile.
The expanded profile supports targetless semantic `TAKEOFF`, compiled only to
adapter-owned altitude/velocity, and `NAVIGATE_TO` targets bound to an exact manifest
`map_route` of at most 16 waypoints and 25 m total length. The read-only observer checks
the vertical takeoff corridor or every route segment, and feasibility binds the complete
observed waypoint list to the compiled proposal.

Takeoff and navigation have distinct controller/resource rules. Takeoff requires a
connected, grounded, disarmed, nearly stationary vehicle plus live Takeoff/Land task
endpoints. Navigation requires armed, airborne, controlled state, a non-stuck planner,
and live Navigate/Land endpoints. Both require fresh canonical odometry, exact command
binding, sufficient point-cloud range, 0.4 m clearance, and single-use admission. The
adapter still does not generate routes or treat absence of observed points as global
free-space planning. The command console remains proposal-only. Validation: 156 tests
passed, changed Python sources compiled, and `git diff --check` passed.

## LATEST: configured AirStack feasibility adapter — 2026-09-20

The Office simulator now has a deterministic, read-only C03 provider at
`scripts/airstack_drone_feasibility_provider.py` and a ROS observer at
`scripts/airstack_feasibility_observer.py`. For each exact grounded single-waypoint
`NAVIGATE` proposal it samples fresh map odometry, MAVROS connection/arming, airborne
and control state, planner-stuck state, Ouster point cloud, and Navigate/Land action
endpoints. The cloud is transformed to `map` and checked against a 0.4 m clearance
threshold along the straight 3D corridor, with explicit point/range coverage. Its
canonical observation JSON is SHA-256-bound into the feasibility result and single-use
admission evidence.

The provider is intentionally fail-closed and narrow. It admits only an already
airborne, armed, controlled, nearly stationary vehicle with a clear currently observed
straight corridor. It does not perform takeoff, global route search, occlusion
inference, or control. A live read-only probe succeeded, but the current simulator
state was correctly rejected: the vehicle was grounded/disarmed without control, the
planner reported stuck, and the sampled corridor did not meet 0.4 m clearance. No
task, action goal, or vehicle command was sent. No learned physics model is required
for this initial profile; learned feasibility can remain advisory as experience is
collected.

The reviewed provider profile is exact, not inferred from arbitrary capability text:
`aerial-eval`, capability revision `office-airframe-v1`, and limits reference
`office-bounded-nav-v1`. A changed or unknown profile fails the body-limits check.
The corridor start must remain within 0.25 m of the checksum-bound camera observation,
the corridor target must equal the compiled waypoint, and odometry must be canonical
`map -> base_link`. Stale channels or detached corridor evidence return `UNCERTAIN`.

The GUI remains proposal-only. Full composition is available only through the
explicit mission runner with `--execute --simulator-only` and this provider path,
after operator authorization and readiness review. Preserve this separation when
adding GUI execution controls.

Validation after the provider and evidence-binding changes: **148 tests passed** via
`bash scripts/test_rrm.sh`; changed Python sources compiled; standalone runner/provider
launch no longer depends on an inherited `PYTHONPATH`; and `git diff --check` passed.

## Body-agnostic dynamic feasibility and full mission composition — 2026-09-20

The live mission composition now implements the complete causal sequence:

`observe → semantic proposal → embodiment grounding → dynamic feasibility → single-use
admission → bounded dispatch → independent effect verification → fresh observation/replan`.

`rrm/dynamic_feasibility.py` defines the C03 result and admission guard. Every result
is bound to the exact grounded proposal digest, image checksum, state/capability/scene/
profile revisions, and stop generation, and expires on the adapter clock. A feasible
result requires authoritative PASS evidence for grounding, body limits, physics,
controller readiness, resources, and the stop channel. Learned feasibility evidence
is explicitly advisory and cannot substitute for any of those checks. Admission is
single-use and append-before-dispatch evidence is written before the public adapter is
invoked.

`rrm/authorized_live_mission.py` composes those gates with the existing visual entity
verifier, warm Cosmos proposal provider, deterministic embodiment compiler, public
task-action dispatcher, independent outcome verifier, and observation/replan loop.
Every action repeats all gates; an unverified effect or any missing/stale/mismatched
feasibility dependency halts the mission. `scripts/rrm_authorized_live_mission.py`
accepts an adapter-owned executable via `--feasibility-provider`; simulator execution
requires it in addition to `--execute --simulator-only`. The default evaluator is
`UNCERTAIN`, so a missing provider cannot move the drone.

The localhost GUI connects a saved fresh goal to private-worker entity grounding and
a visible RRM proposal. It remains proposal-only. The subsequently added AirStack C03
adapter supplies current sensor-derived corridor evidence; action-server availability
and visual grounding alone are still not treated as collision-free evidence.

## LATEST: any catalog Isaac scene can now switch in-place from the GUI — 2026-09-20

The localhost Command Console now populates **Change Isaac scene** from every `isaac`
entry in `/root/AirStack/simulation/scenes.yaml` (including custom USD references and
their stage scales). Selecting an entry restarts only `isaac-sim-livestream` and
`robot-desktop` through AirStack; it does **not** submit/recreate the OSMO workflow or
restart the warm private Cosmos worker. The endpoint accepts only a catalog shortname,
uses the resolved Isaac reference and scale, prevents concurrent switches, and clears
the prior camera state.

The checked-in RRM manifest is Office-specific. After selecting a non-Office scene,
the GUI accurately reports that live proposals are inhibited rather than applying the
Office entity catalog to a different stage. The selected scene is persisted under the
ignored command-request artifacts so a console restart remains fail-closed; before an
operator selects a scene, it reports `active_scene: null` and disables live proposals.
The console was restarted on port 8787 after this change; it currently lists 21 catalog
scenes and has not restarted Isaac or changed the active scene. Validation: 131 tests
passed, including the catalog parser and a mocked switch verifying its scene/scale
environment and nonmatching-manifest gate.

## LATEST: workspace worker URL is rendered; Remote-SSH console now recovers it — 2026-09-20

The submitted OSMO workflow did render `RRM_COSMOS_WORKER_URL` in the workspace
init process. The value is the worker's private pod hostname on the workflow
cluster network, and a read-only `GET /healthz` returned 200 with
`execution_dispatch: false`. The earlier empty result came from the Remote-SSH / IDE
process tree: it does not inherit the init process environment, so a console launched
from that shell had no worker URL even though the group token was correctly rendered.

`scripts/rrm_command_console.py` now prefers an explicitly inherited
`RRM_COSMOS_WORKER_URL`, then (only when absent) reads that one non-secret variable
from `/proc/1/environ`. It neither queries the scheduler nor exposes a port, reads a
credential, or adds a control path. Restart the localhost console after pulling this
change so its live shadow cycle receives the private endpoint. Validation: 127 RRM
tests passed, including explicit-value precedence, init-environment fallback and
unreadable-init fail-closed cases. The worker remains private and shadow-only.

## LATEST: OSMO worker discovery is not wired in the submitted workflow — 2026-09-20

The `cosmos-worker` task is healthy and listening on `0.0.0.0:8090`; its
`GET /healthz` returned 200 from the group network. The submitted workflow did
**not**, however, place `RRM_COSMOS_WORKER_URL` in the workspace environment:
it was empty when inspected. Nor did `cosmos-worker` resolve from the workspace.
OSMO query reported the worker as `RUNNING` with Pod IP `172.16.231.62` and no
host/address field. Consequently the workspace cannot call the private
`/v1/verify-entities` endpoint through the intended configuration.

The current YAML assumes that `RRM_COSMOS_WORKER_URL:
"http://{{host:cosmos-worker}}:8090"` will be substituted and that the task name
will be DNS-resolvable. That assumption is invalid for this submitted workflow;
do not treat the worker task name as a Kubernetes service name. Before submitting
another two-GPU workflow, validate the rendered workflow spec and use the OSMO
supported group-task address mechanism for the deployed OSMO version (or inject
the worker Pod IP through a documented runtime bootstrap). Keep the endpoint
private and preserve `execution_dispatch: false`; do not work around this with a
public worker port-forward.

## LATEST: shadow-only live action → replan orchestration foundation — 2026-09-19

`rrm/live_replan.py` now persists a provider-neutral mission cycle intended for the
future continuous Cosmos/VLM loop. Each step requires a fresh checksum-bound camera
capture and a separate live entity-verifier record; the provider response is bound to
that exact observation. A multi-action model proposal is retained for planning, but
only its first action is exposed for review. A reviewed, independently verified outcome
is mandatory before a fresh observation/replan is accepted; an unverified outcome or
non-accepted candidate halts the cycle. No ROS, PSC, model-runtime, dispatch, PX4, or
MAVROS capability was added.

This is deliberately not yet wired to the console or a persistent Cosmos service.
The next integration must implement the entity verifier and a warm OSMO provider,
then expose the cycle in shadow mode before connecting any per-step explicit approval
to the existing public ActionClient dispatcher. Validation: six live-cycle unit tests
passed, including observation/response binding, outcome gating, restart identity, and
no-control-surface scan.

### Worker provisioning update — 2026-09-19

The warm-provider code and OSMO workflow are now prepared but not submitted:
`scripts/rrm_cosmos_worker.py` loads one local Cosmos snapshot at startup and exposes
only private `GET /healthz` and `POST /v1/propose` endpoints. The request/response are
bound to a cycle ID, step, and image SHA-256 and return `execution_dispatch: false`.
`osmo/workflows/airstack-live-replan.yaml` starts the existing Isaac workspace and a
separate one-GPU `cosmos-worker` task in the same OSMO group; OSMO substitutes the
worker's private address into `RRM_COSMOS_WORKER_URL`. It requests two fair-share GPUs
total (24 CPU / 96 GiB memory).

The image is published at
`airlab-docker.andrew.cmu.edu/airstack/airstack-rrm-cosmos-worker:latest` (manifest
digest `sha256:ddd2fcbfa57a0b981beca5f66a294c7288c264f0082f3b8852e028553564828c`).
No persistent OSMO model mount is documented in AirStack or its public docs, so the
workflow now downloads the approved pinned `nvidia/Cosmos-Reason2-8B` snapshot into
the worker's task-local storage at every workflow start. The only remaining operator
prerequisite is a user-owned generic OSMO credential named `rrm-huggingface-read`, with
key `hf_token`, after the user accepts the model terms in Hugging Face. The workflow
injects that token only into the worker, uses it for `hf download`, then unsets it
before the private HTTP service starts. It must not be written to source, images,
artifacts, or the Isaac workspace. Validation: worker RPC boundary, live-cycle, Cosmos
parser, console and Office import tests: 52 passed; YAML parsed; source has no
vehicle/scheduler control surface.

## LATEST: per-attempt evidence, live-observation intake and PSC bridge boundary — 2026-09-18 16:20 UTC

The command console now records and displays immutable run history for both the
historical reference and new goals. A run can expose only allow-listed, checksum-checked
downloads for reconciliation, admission, dispatch outcome, STOP request/delivery,
landing, live observation, PSC submission/receipt/result, proposal and approval
records. The SQLite migration preserves old goals/runs and adds explicit inference and
review lifecycle states.

New requests no longer use the historical frozen image. The console requires two
read-only Isaac captures so the second image proves source-timestamp progression; it
then binds the image checksum, `camera_left` frame, capture time, MAVROS connection,
and canonical `map -> base_link` odometry to `observation.json`. Paused timestamps,
wrong frames, stale wall captures, disconnected vehicles, malformed state and checksum
mismatches fail before a request is saved or submitted. A live Office capture confirmed
`camera_left`, MAVROS connected/disarmed and canonical grounded odometry. This is
transport/freshness evidence only, not a claim that a camera image establishes marker
visibility or flight safety.

`rrm/psc_pipeline.py` runs one immutable request in a background worker through a
configured non-interactive bridge. The bridge stages that request and source snapshot,
submits the PSC batch, waits/fetches its own result bundle, and returns a PSC job ID.
The result is accepted only when returned input/image hashes, reviewed scene manifest,
C04/C05 candidate and drone-adapter proposal all validate. Candidate approval requires
the exact stored proposal SHA-256 and writes `execution_dispatch: false`; it does not
yet route a new plan into the public ActionClient dispatcher. A substituted result fails
as `INFERENCE_FAILED`, with no candidate or dispatcher.

**External prerequisite:** PSC key management must provide the approved
non-interactive key/agent used by `RRM_PSC_BRIDGE=1`; the known password-authenticated
transfer is intentionally not automated by a browser/server process. Do not enter PSC,
SSH or Hugging Face credentials into the console or repository. The historical PSC
bundle is absent from the current recreated OSMO workspace, so the live console was not
restarted here. The simulator and robot were not restarted and no PSC job or robot task
was sent.

Validation: 94 RRM tests passed, Python sources compiled, relevant shell scripts passed
`bash -n`, and `git diff --check` passed. The next engineering increment, after PSC
key authorization, is a single controlled live PSC job followed by a separate design
for binding result-age/scene/vehicle revalidation and the existing STOP/LAND/reconcile
supervisor to an explicitly approved new proposal. Do not bypass that boundary.

## LATEST: grounded reconciliation and verified STOP → LAND trial — 2026-09-18 06:36 UTC

The console can now begin a new execution attempt only after a read-only observer
proves a fresh, connected, disarmed, stationary, near-ground vehicle state. The
observer creates no publisher, service client, or action client. Reconciliation
preserves all prior evidence, writes a new record under
`.rrm-artifacts/command-requests/execution/reconciliations/`, increments the stop
generation, and then reopens the exact proposal for approval. The GUI exposes this as
**Confirm safe state / new attempt**; it is not an operator-only assertion.

The ROS interrupt race was fixed in `airstack_drone_dispatch.py`: rclpy no longer
consumes SIGINT and closes its context before the adapter requests cancellation.
Shutdown is also guarded. A clean simulator trial then produced the first valid stop
measurement through the public AirStack task actions:

- Takeoff to the 1.5 m request was VERIFIED at z=1.326 m, connected and armed.
- Navigation dispatch `3b28f3724c104be9be26aeb5ed10f02b` was accepted and reported
  `navigating` before STOP / HOLD.
- HTTP STOP returned in 160.034 ms; supervisor-to-process signal delivery was
  157.980 ms; the action server acknowledged cancellation in 200.890 ms.
- Three fresh odometry samples verified speed at or below 0.10 m/s in 241.277 ms;
  observed speed was 0.0112 m/s. Outcome: `MOTION_STOPPED`, not merely acknowledged.
- LAND NOW dispatch `34f784eee3744f048904a09060522a61` was VERIFIED at z=0.012 m,
  connected and disarmed.

Runtime evidence is intentionally ignored by git at
`.rrm-artifacts/command-requests/execution/<dispatch-id>/`. The operator states now
distinguish `STOPPED_VERIFIED` from `STOPPED_UNCONFIRMED`, and `LAND_VERIFIED` from
`LAND_FINISHED_UNCONFIRMED`; the browser uses plain-language labels.

Validation: **90 tests and 26 subtests passed**, changed Python compiled, and
`git diff --check` passed. The console was restarted with the current source and is
running as PID `236504` on host loopback port 8787. After restart, a fresh grounded
reconciliation (`9f4d89b2a8264054aaf48c67e9779f8e`) restored
`READY_FOR_APPROVAL`. The drone is landed/disarmed and no dispatcher is active.

The next engineering step is not another blind flight. Preserve this checkpoint,
then make the reconciled execution epoch and verified stop/land evidence visible in
the history UI (and later connect new console goals to PSC inference rather than the
current frozen Office result). Full model-to-drone automatic connection remains
deferred.

### Five steps to the full model-to-drone path

1. **Expose execution evidence in the console.** Add reconciliation, approval,
   dispatch, STOP timing, and landing outcomes to each immutable goal/attempt history
   entry, using plain operator language.
2. **Submit saved goals to PSC inference.** Package each newly saved command with its
   frozen or explicitly refreshed observation, submit it as a tracked PSC job, and
   import the resulting bundle without blocking the web process.
3. **Validate and review the new plan.** Apply the existing canonical-ID, capability,
   constraint, and safety gates to that exact result; show its action payload and plan
   fingerprint for explicit approval. Never reuse the historical Office approval.
4. **Dispatch the approved plan through AirStack.** Translate only validated verbs to
   the existing public task actions, serialize execution, retain STOP / HOLD and LAND
   NOW priority, and record independent outcome evidence after every action.
5. **Replace frozen replay with live observations.** Feed timestamped Isaac/Foxglove
   camera and vehicle state into the request, reject stale or frame-mismatched data,
   then run an end-to-end simulator acceptance suite before considering real hardware.

Do these in order. Steps 2–5 must retain the current fail-closed reconciliation,
explicit approval, no-direct-PX4/MAVROS-command boundary, and evidence records.

## LATEST: latency instrumentation complete; live trial safely aborted — 2026-09-18 06:12 UTC

The dispatcher now records operator-stop receipt, cancellation acknowledgement
latency, fresh odometry speed and time to three consecutive samples at or below
0.10 m/s. The supervisor separately records GUI/server-to-signal delivery latency.
All 85 tests and Python compilation passed; no direct-control surface was added.

The first live measurement was **aborted at takeoff**. The public Takeoff task returned
`failed to arm`; PX4 logged `Arming denied: Resolve system health failures first`,
Isaac/PX4 logged repeated time jumps, and robot mapping logged transform future-
extrapolation errors. Fresh odometry was about `(3.16, 11.71, 0.96)` while MAVROS was
connected and disarmed, so this was not the required clean origin. Navigation,
STOP / HOLD and LAND NOW were not sent. No stop-latency number is valid.

Runtime evidence:
`/root/AirStack/.rrm-artifacts/stop-latency-20260918.PbmhyF/takeoff-outcome.json`.
The console remains on port 8787, normal state `READY_FOR_APPROVAL`, and no dispatcher
is active. Before retry: coordinated clean restart in simulator-then-robot order;
prove stable time/TF, origin, connected/disarmed state and takeoff gate, then repeat
navigation → STOP / HOLD → LAND NOW. Do not bypass the failed arming gate.

Follow-up in the same session: a coordinated Isaac-then-robot restart restored origin
and connectivity. A final takeoff retry with a 10 s observation window and 2 s age
bound was VERIFIED. The console navigation dispatcher then exited before ROS import
because its inline `PYTHONPATH` replaced ROS's path (`ModuleNotFoundError: rclpy`); no
navigation goal was sent. STOP / HOLD latched in 1.9 ms at HTTP level, but that is not
a physical-stop measurement. A direct public-action recovery LAND was VERIFIED at
z=0.006 m, connected and disarmed. The launcher now appends its dependencies to the
sourced ROS path and uses the measured ~0.75 Hz state rate bounds. Do not claim a
stop-latency result until a new clean trial exercises the fixed launcher.

Final state for handoff: recovery landing VERIFIED, MAVROS connected/disarmed, no
dispatcher process, console PID 228483 on port 8787. The console correctly starts in
`RECONCILIATION_REQUIRED` with normal admission stopped because the failed admission
evidence still exists. Do not delete it or bypass the latch. The next increment should
record an explicit grounded reconciliation/new execution epoch, then repeat the fixed
launcher trial; no further flight was attempted in this turn.

## LATEST: STOP / HOLD and LAND NOW controls — 2026-09-18 05:40 UTC

This supersedes the earlier single STOP-button description below. The GUI now uses the
minimal operator surface agreed with the user: normal **Approve & send**,
**STOP / HOLD**, and **LAND NOW**. There is deliberately no generic Pause/Resume;
continuation requires fresh state and a new RRM plan.

- STOP / HOLD blocks normal admission first and requests cancellation of the active
  public task action. It never claims stable hover from acknowledgement alone.
- LAND NOW is an operator safety override that writes a typed LAND proposal/evidence,
  blocks normal RRM commands, cancels an active command first, and starts the existing
  public Land action only after cancellation acknowledgement. If acknowledgement is
  missing, it fails closed as `LAND_BLOCKED_UNCONFIRMED` rather than overlapping task
  actions. STOP / HOLD can cancel an active or pending landing.
- The dispatcher now writes an `operator_stop` outcome containing separate
  `cancel_acknowledged` and `physical_stop_verified=false` fields on SIGINT.
- The GUI uses plain-language states. Protocol terms remain in evidence/docs.

Validation before live-console restart: **84 tests passed**, Python compilation,
`git diff --check`, and the no-direct-control scan passed. No live approval, stop,
landing or ROS goal was sent during implementation. Notebook:
ignored `notebook/006-rrm-flight-controls/`.

Live non-motion validation also passed: console PID 215786 on loopback port 8787,
state `READY_FOR_APPROVAL`, no dispatcher process, vehicle connected and disarmed.
Both new controls were served; Pause/Resume are absent. Refresh the forwarded browser.

## LATEST: C06 approval and C08 stop demo boundary — 2026-09-18 05:18 UTC

This section supersedes the later “Pending Implementation” lines near the bottom of
the file. The narrow command-console boundary is included in the current safety
checkpoint. No flight or valid approval was sent while implementing it.

- `rrm/execution_supervisor.py` binds the one imported PSC proposal to a canonical
  SHA-256 digest, writes an admission record before launch, permits one active
  dispatcher, and launches only the existing public-ActionClient dispatcher.
- The console/UI now displays the exact action and goal, requires explicit
  **Approve & send** or **Reject**, and rejects stale plan fingerprints. New task-intake
  requests are still inference-only and cannot inherit this historical approval.
- **STOP** latches admission closed first, increments a stop generation, and then
  interrupts the active dispatcher. After action acceptance, the existing dispatcher
  requests ROS cancellation on SIGINT. The UI correctly reports `SAFE_UNCONFIRMED`;
  cancellation delivery/acknowledgement or process exit is not physical-stop proof.
- A console restart with any prior admission artifact starts inhibited as
  `RECONCILIATION_REQUIRED`. This is a narrow single-process demo, not the complete
  authenticated/distributed C06/C08 contract.
- The prior uncommitted simulator-restart control remains, but is relabeled
  **Restart simulation (development)** and explicitly distinguished from C08.
- Operator-facing text uses plain language such as “plan fingerprint,” “send,” and
  “new commands blocked”; protocol names remain in evidence and developer docs.

Validation: all 80 RRM tests pass in the robot container; Python compilation,
`git diff --check`, exact-digest negative-path HTTP validation and the no-direct-control
source scan pass. No JavaScript engine is installed in the current host or containers,
so automated JS parsing was unavailable; the live server successfully served the new
HTML and required controls. The live invalid-digest attempt returned HTTP 400 and left
state `READY_FOR_APPROVAL`, `active=false`, `dispatch_id=null`.

Runtime at the last check: console PID 208669 on host loopback port 8787; proposal
digest `29fe66c2914b27577f285089e598d6a15413e44e023b40e530c19194d221707e`;
no `airstack_drone_dispatch.py` process; MAVROS connected and disarmed. Isaac, robot
and GCS containers were left running. Refresh the forwarded browser page to load the
new controls. Do not click approval without a fresh scene/path review and supervised
viewer; the current proposal remains `(3.2, 0, 1.5)`.

The checkpoint also includes the later flight's DROAN radius tuning, Office flight
evidence document, simulator reset UI/backend and appended analysis. The console work
adds `rrm/execution_supervisor.py`, tests and documentation. Notebook evidence remains
local and ignored under `notebook/005-rrm-admission-stop/`.

## STOP HERE: exact continuation state — 2026-09-18 00:20 UTC

This section is authoritative over older “deferred / not implemented” language
below. Work has progressed to a **verified learned-plan → public drone-action
dispatch boundary**, but a full end-to-end Office navigation flight is deliberately
not claimed yet.

### What is complete and verified

- PSC Office inference job `46288765` completed ACCEPTED after the canonical-ID
  prompt fix. Its verified decision is `near($self, blue_marker)` with action
  `NAVIGATE_TO(blue_marker)`. The source result is persistent at PSC; an imported,
  hash-checked OSMO copy is under
  `/root/AirStack/.rrm-artifacts/psc-office-46288765.FCRcPE/`.
- The original rejection from job `46280177` was caused by label prose
  (`blue navigation marker`) rather than canonical scene IDs. Regression coverage
  lives in `tests/fixtures/office_46280177_response.json` and
  `tests/test_cosmos_reason2.py`.
- `rrm/airstack_drone.py` and `scripts/airstack_drone_dispatch.py` translate the
  approved RRM action only through AirStack's public ROS 2 ActionClient. They do
  not directly publish PX4/MAVROS setpoints or call PX4/MAVROS services. Takeoff and
  landing were independently proven; takeoff now rejects horizontal displacement
  greater than 0.3 m. An unavailable navigation action server produces a recorded
  `NOT_DISPATCHED` / `UNCONFIRMED` outcome rather than pretending a goal was sent.
- The localhost command console is implemented at port `8787`: immutable saved
  goals, attempts, frozen inference evidence, limited read-only camera refresh, and
  no implicit inference or flight dispatch. Its SQLite data is intentionally ignored
  at `/root/AirStack/.rrm-artifacts/command-requests/tasks.sqlite3`. See
  `docs/scrum-8/command-console.md`. On the laptop, forward port 8787 in the IDE.
  Foxglove remains separate through the laptop's `ws://127.0.0.1:8766` forwarding.
- Test evidence: `PYTHONPATH=/tmp/rrm-canonical-deps:/tmp/rrm-canonical-source
  python3 -m pytest tests -q` inside the robot container passed **75 tests**.
  `python -m py_compile` for changed Python and
  `COMPOSE_PROFILES=desktop docker compose config -q` passed. `git diff --check`
  passed before this handoff update.

### Last simulator diagnosis and current safe state

The first learned-navigation attempt did not move the drone: it correctly withheld
the navigation goal when its live action server was unavailable. The cause was
identified as stale `/tmp/.X99-lock` and `/tmp/.X11-unix/X99` after a robot-container
restart, which made `droan_gl_node` fail its GLAD/OpenGL initialization. Stale DDS
discovery could still show an action name, so always require a live ActionClient
server check immediately before dispatch.

`robot/docker/docker-compose.yaml` now removes only those two stale X99 files, only
when no `Xvfb` process exists, before starting Xvfb. This was manually validated:
Xvfb started, `droan_gl` reported Mesa OpenGL 4.5 and loaded shaders, and a live
`/robot_1/tasks/navigate` server was observed.

The robot desktop container was recreated with the patched Compose file and left
running. At the last check it was connected, disarmed, and at ground altitude near
map position `(-1.269, -0.870, -0.001)`. No action is currently in flight and no
manual diagnostic node remains. Do **not** shut down the OSMO workflow/container;
the user wants it left running until its scheduled 06:00 expiry.

### Next operator/agent steps (do not skip the safety gates)

1. Confirm the running stack, vehicle connection/disarmed state, fresh odometry and
   a live navigation ActionClient server. Do not trust a stale `ros2 action list`.
2. Start the command console only if it is not already running:
   `cd /root/AirStack/RRM-transfer/rrm-9d26a58 && bash scripts/rrm_command_console.sh`.
   It should bind to host loopback `8787`; inspect rather than modify the SQLite DB.
3. Use the imported proposal associated with PSC job `46288765`; re-check its hashes
   and live scene/marker correspondence before any physical action. The Office
   waypoint currently used by the adapter is map `(3.2, 0, 1.5)`, tolerance 0.3 m.
4. Only after the above, run the bounded public dispatcher for a supervised
   takeoff → learned `NAVIGATE_TO` → landing trial. Keep Foxglove/camera observation
   open and record all outcomes. If the server is unavailable or observation is
   stale, stop at `NOT_DISPATCHED`; do not add direct PX4/MAVROS control as a bypass.
5. Preserve runtime evidence before OSMO ends. `.rrm-artifacts` is ignored and
   ephemeral; source changes are committed, but flight/console evidence must be
   archived separately if needed.

Flight-attempt evidence, including the bounded takeoff/land and withheld navigation
outcomes, is in
`/root/AirStack/.rrm-artifacts/office-flight-46288765-20260918T0020Z/`.
It is intentionally not in Git. Never include core dumps from that directory in a
source commit.

## Current database/GUI — 2026-09-18 00:04 UTC

User approved SQLite goal history and selection. Implemented `rrm/task_store.py`:
immutable goals and separate attempt records with artifact paths; current statuses
SAVED_NOT_SUBMITTED and CANDIDATE_ACCEPTED, both NOT_DISPATCHED. Console indexes the
verified historical inference and recovers saved request folders idempotently,
checking hashes. New request manifests carry goal_id. Editing selected goal text
creates a new goal; selecting a goal alone does not submit anything.

Database: `/root/AirStack/.rrm-artifacts/command-requests/tasks.sqlite3`.
Console restarted on host loopback 8787; existing IDE port forwarding remains valid.
Refresh browser to see Saved goals & history / Use goal / Save another request.
75 CPU tests pass, JS syntax checked, live history/download HTTP checked, SQLite
integrity_check=ok. Production store has one historical goal/run from 46288765;
no synthetic requests added. Original bundle checksums still pass.

8787 suffices for GUI and read-only camera refresh. Separate Foxglove requires the
previous forwarding command running on the laptop, with ws://127.0.0.1:8766.
No inference/dispatch integration added. DB/history survive console restarts but are
on ephemeral OSMO storage; preserve DB plus referenced files before workflow ends.
See `docs/scrum-8/command-console.md`.

## Current UI — 2026-09-17 23:53 UTC

User requested a localhost command GUI with an Isaac view where feasible, allowing
Foxglove alongside it. Implemented `scripts/rrm_command_console.py`, launcher `.sh`,
and `scripts/ui/command_console.html`. Running on OSMO host loopback port 8787.
Forward 8787 in VS Code/Cursor's Ports panel to open from Mac. See
`docs/scrum-8/command-console.md` for restart command and scope.

GUI shows previous verified plan, frozen input image, bounded read-only front-camera
refresh, task form and download links. Saves new inference requests only, with
original frozen evidence times; it does not run Cosmos or dispatch. New request
artifacts go under `/root/AirStack/.rrm-artifacts/command-requests/<uuid>/`.
Foxglove remains a separate viewer using the existing ws://127.0.0.1:8766 forwarding.
Two real camera captures succeeded with advancing source timestamps; view is mostly
floor/wall, not proof of current Office alignment. 71 CPU tests pass; JS syntax and
live HTTP camera endpoints checked. Browser graphical rendering not automated.
Full model-to-drone connection remains deferred. The earlier UI-deferred notes below
are historical; this first command-entry/observation GUI is now implemented.

## Current verified state — PSC job 46288765 imported, 2026-09-17 23:27 UTC

Actual PSC bundle was downloaded and imported successfully. Verified locally:
all four SHA256SUMS entries, both recorded inference-source hashes, exact prompt,
raw model response reparse, stored candidate equality and local scene binding.
A fresh import exactly matches the saved decision and proposal. Import status READY;
action ID NAVIGATE_TO retained; target blue_marker; goal near($self, blue_marker).
Adapter waypoint (3.2, 0, 1.5) in map, tolerance 0.3 m. Inference wall time recorded
as 900.1816659809556 s (includes loading); execution_dispatch=false.

Host artifacts:
`/root/AirStack/.rrm-artifacts/psc-office-46288765.FCRcPE/bundle/`
and sibling `imported/decision.json`, `imported/proposal.json`.
Evidence summary: `docs/scrum-8/evidence/office-inference-46288765.md`.
PSC original remains persistent; OSMO copies are ephemeral. No flight, live scene
revalidation or stop verification performed. Full simulator connection remains
deferred. Visual command-entry interface is still a future idea, not implemented.
The authorized retrieval/import milestone is complete; do not repeat it.

## Previous retrieval instructions — completed

User supplied a completed inference console with ACCEPTED, execution_dispatch=false,
and 14m39s checkpoint loading. Pasted raw output selects NAVIGATE_TO blue_marker,
goal near($self, blue_marker), grounded entities blue_marker and orange_marker,
action ID NAVIGATE_TO, no dependencies, recovery budget 0. Avoidance is expressed
in the explanation; it is not independent path-clearance evidence.

User authorized retrieving, preserving and importing this actual bundle into an
unexecuted proposal. Full simulator connection and visual command UI remain deferred.
The unattended DTN SSH check was denied. The actual result bundle has not yet been
retrieved or hash-verified here. Run on OSMO host (authenticate in terminal):

```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
bash scripts/rrm_office_fetch_import.sh 46288765
```

This verifies SHA256SUMS and runs the existing importer (hashes, prompt, raw-output
reparse, candidate equality, scene binding), preserving downloads and proposal under
a new `.rrm-artifacts/psc-office-46288765.*` directory. Uses running robot container
and isolated Pydantic 2 at `/tmp/rrm-canonical-deps`, verified available this session.
No model rerun or robot dispatch. PSC original remains persistent; OSMO copy is ephemeral.

## Latest failure — PSC job 46288321

User supplied accounting: FAILED, exit 1:0, elapsed 1 second, time limit 30 minutes.
Console: missing `/src/rrm/examples/office_visual_eval/navigation_context.json`.
The intended timestamped source directory was not used; the environment-based
source selection fell back to the default. Exact cause of lost/overridden variable
has not been established. Local batch/transfer scripts now pass source-root as a
positional argument and log it; batch default is also 30 minutes.
The already uploaded snapshot can be retried without another transfer by exporting
the intended `RRM_SOURCE_ROOT` in the PSC login shell and submitting with
`sbatch --export=ALL --time=00:30:00 ...`. No new submission claimed here.

## Latest continuation — 2026-09-17, canonical-ID retry

User supplied the console and candidate from PSC Office job `46280177` (later than
`46273277`). The batch reached its completion message with `execution_dispatch=false`.
The candidate was REJECTED for `ungrounded_entity:blue navigation marker` and
`ungrounded_entity:orange navigation marker`: the model used descriptions instead of
`blue_marker` / `orange_marker`. Its goal also used prose instead of symbolic `near`.
The pasted JSON is a test transcription; the actual PSC bundle has not been retrieved.
Slurm accounting was not checked; the completion message alone is not an accounting record.

Prompt now explicitly supplies canonical entity IDs, declared operation semantics
and the navigation goal template; console output includes rejection reasons.
Strict parsing is unchanged. Retry transfer script reuses the prior PSC `input.png`
and prints a new source-snapshot submission command with 30-minute walltime (observed
checkpoint loading was 18m08s). Retry has not been submitted from this session.
See `docs/scrum-8/office-live-demo.md` for the command.

Current user scope: fix/retry inference quickly; defer full simulator connection.
A visual command-entry interface feeding RRM is a later idea, not yet implemented.
This supersedes the older immediate-flight next step below. The current workspace
has no PSC SSH credentials; authentication must occur in the user's terminal.

## Current continuation — 2026-09-17

This update supersedes the historical hand-first/shadow-only/PSC-unknown notes below.
User explicitly requests learned RRM driving an AirStack drone in Isaac Office,
visible via WebRTC and Foxglove. Control is the embodiment adapter, not RRM's purpose.
Physics/IK and VLA are deferred. Preserve the body-agnostic reasoning contracts.

Cosmos-Reason2-8B revision `a9fae2cf89dc64db96b12860417f0eb403013bb9` is cached on
PSC Ocean, project `eng260004p`, user `oabolade`, root
`/ocean/projects/eng260004p/oabolade/physical-ai`. One prior ground-truth-label
inference was ACCEPTED but no real Office inference/flight exists locally yet.
PSC requires user authentication; existing temporary SSH key was denied. Do not ask
for passwords in chat or claim an authenticated remote session. The user submitted
the Office inference job `46273277` on 2026-09-17. It is queued/running independently
of OSMO under `runs/rrm/office/46273277`; obtain its final state with
`squeue -j 46273277`, then inspect its persistent `console.log` and `result.json`.

Office scene is running as `isaac-sim-livestream`, with single-GPU flags in
`rrm_office_visual_eval.py`. Robot/GCS remain running. Camera frame captured;
PX4 ready, connected and disarmed; GCS robot markers ~10 Hz. Frozen original image:
`/root/AirStack/.rrm-artifacts/office-v1/rrm-office-frozen-v1.png`. Local files are
still ephemeral until transferred; no commit/push/upload claimed.

New learned `compile_plan` preserves C05/action IDs; importer checks hashes and
reparses raw model output. Navigation verification now checks independent endpoint
and causal odometry. Public task dispatcher has bounded waits/cancel requests.
66 unit tests pass. Actual flight, live scene revalidation, mission supervision and
independent stop proof remain unverified/incomplete. Do not substitute baseline
navigation or synthetic test outputs for Cosmos.

Next: wait for job `46273277` to finish, retrieve its actual inference bundle,
validate live scene
and path, finish supervision, then execute/observe takeoff→learned navigation→land.
See [Office demo runbook](docs/scrum-8/office-live-demo.md) for exact commands,
viewing instructions and limitations. Mac patched OSMO forwarding remains needed.

---

Prepared 2026-09-13. Read this before implementing changes. This file captures the
user's instructions and the findings from the local review; it does not claim that
the proposed architecture or SIL integration has been implemented or approved as a
completed baseline.

## Start here

You are continuing RRM work inside the user's remote AirStack workspace at AirLab,
using NVIDIA OSMO. The user wants software-in-the-loop (SIL) first. The Mac is for
source preparation and transfer; do not download model weights or install the GPU
stack there. This archive contains the RRM source, not AirStack or model weights.

1. Read this handoff and the applicable AGENTS.md in the remote AirStack checkout.
2. Inspect the actual remote branch, uncommitted changes, containers, and resource
   allocation. Preserve existing work. Do not infer remote state from local findings.
3. Read the Phase 1 sources below and inspect RRM's existing code. If connectors are
   unavailable, use the requirement transcription below and identify that limitation.
4. Complete the SCRUM-8 architecture allocation and logical interface contracts before
   adding components or making detailed model/transport/deployment commitments.
5. Use the capstone curriculum and metrics to define a traceable SIL evaluation plan.
6. Implement incrementally after allocation is clear, preserving the mock regression
   baseline and adding meaningful checks for capability, supervision, and traceability.

Do not mark SCRUM-8 Done based on this handoff or the five mock benchmark passes.
Do not send Jira comments or other messages without the user's authorization.

## User decisions and scope

- Phase 1 is complete in Jira/Confluence: SCRUM-6 / RRM-01 Needs & CONOPS and
  SCRUM-7 / RRM-02 Requirements & Traceability are Done.
- Next work: SCRUM-8 / RRM-03 System Architecture, last read as To Do.
- Map every system requirement to architecture responsibilities; define stable
  interfaces between reasoning, world/task state, planning, safety, execution
  monitoring, telemetry, and embodiment adapters.
- Keep task-level reasoning embodiment-independent. Declared capabilities and limits
  can cause different valid plans without requiring robot-specific reasoning behavior.
- Avoid detailed implementation choices until architecture allocation is clear.
- The current focus is SIL. AirStack/OSMO is the primary execution environment; PSC
  is a possible fallback if problems arise. PSC access and configuration are unknown.
- Select any future VLA, VLM, NLP, or other learned model for the best capability per
  available compute; record the exact model, license, resource needs, interface role,
  and evaluation plan before integration. Begin the drone-SIL integration with a
  model-free deterministic/shadow baseline; no model is needed for that increment.
- The capstone website, including Weeks 1–11, is an intended technical and evaluation
  reference. The full curriculum has NOT yet been reviewed in this session.
- The user intends to ZIP RRM locally and unzip it in the remote AirStack workspace.
  A local AirStack folder was found, but the user explicitly clarified that the actual
  integration target is remote. Do not require a local full AirStack setup.
- Reachy Mini was an aside/reference, not a selected embodiment or scope change.

## Authoritative Phase 1 sources

- CONOPS, page 1048599, version 4 when read:
  https://deboabolade.atlassian.net/wiki/spaces/SCRUM/pages/1048599
- Requirements, page 786434, version 3 when read:
  https://deboabolade.atlassian.net/wiki/spaces/SCRUM/pages/786434
- Architecture, page 1015810, version 1 when read (a short capability outline):
  https://deboabolade.atlassian.net/wiki/spaces/SCRUM/pages/1015810
- https://deboabolade.atlassian.net/browse/SCRUM-6
- https://deboabolade.atlassian.net/browse/SCRUM-7
- https://deboabolade.atlassian.net/browse/SCRUM-8
- Atlassian cloud ID used: 13ecd67f-0309-4e63-838e-f3e1da003e98.

The user's instructions and Phase 1 baseline supersede conflicting prototype choices
in CLAUDE.md, README.md, docs/architecture.md, requirements.txt comments, and setup
scripts. Preserve useful design history but distinguish it from current requirements.
The old repo's phase numbers are not the Jira engineering lifecycle's phase numbers.

### CONOPS summary

RRM interprets contextual task objectives, maintains relevant world/task state,
decomposes and plans permitted tasks, executes through declared embodiment interfaces,
monitors outcomes, and replans, requests assistance, or reaches a safe condition.

The representative scenario is a structured simulated workspace with multiple relevant
objects, a contextual multi-step instruction, at least one ambiguity or changed
condition, and explicit constraints. Variations include a displaced object, an
ambiguous target requiring clarification, unsupported embodiment actions, safety
rejection, and execution failure requiring recovery.

Autonomy is bounded by permissions, safety, capability, observability, and required
human approvals. Human override/stop remains available whenever execution can affect
the environment. Embodiment progression in CONOPS is dexterous hand → arm → bimanual
system → mobile manipulator → humanoid; not every embodiment supports every action.

### System requirements (transcribed from the baseline)

All IDs below have prefix RRM-SYS-REQ-. Parent IDs use RRM-STK-REQ-.

| ID | Requirement | Parent/source | Initial verification intent |
|---|---|---|---|
| 001 | The RRM shall interpret a task objective using available contextual information relevant to the current task. | STK-001 | Demonstration / test |
| 002 | The RRM shall maintain a task-relevant representation of current world and task state during execution. | STK-002 | Inspection + test |
| 003 | The RRM shall decompose a permitted multi-step task into executable subgoals or actions. | STK-002 | Test |
| 004 | The RRM shall evaluate planned actions against declared embodiment capabilities and limits before dispatch. | STK-004 | Test |
| 005 | The RRM shall monitor execution outcomes and compare observed state against expected action effects. | STK-002 | Test |
| 006 | The RRM shall replan, request assistance, or transition to a safe condition when observed execution invalidates the active plan. | STK-002, STK-003 | Off-nominal test |
| 007 | The RRM shall request clarification or approval when uncertainty materially affects task intent, safety, or successful execution. | STK-003 | Scenario test |
| 008 | The RRM shall evaluate applicable safety and permission constraints before dispatching an action to the robot embodiment. | STK-003 | Safety test |
| 009 | The RRM shall support interruption and externally commanded stop or override before and during physical execution where the embodiment can affect the environment. | STK-003 | Demonstration / test |
| 010 | The RRM shall support execution of task-level reasoning across multiple robot embodiments by consuming declared embodiment capabilities and limits without requiring embodiment-specific changes to the task-level reasoning behavior. | STK-004 | Demonstration + architecture inspection |
| 011 | The RRM shall record telemetry sufficient to reconstruct task objectives, relevant state, decisions, dispatched actions, observed outcomes, replanning events, interventions, and failures. | CONOPS Operational Success Criteria | Inspection + test |

Stakeholder requirements cover contextual objectives/constraints (001), multi-step
execution under change/failure (002), human oversight (003), and reusable reasoning
across embodiments (004). New architecture elements must identify the requirements
they satisfy; trace forward to tests and evidence as those are created.

## Current RRM implementation and verified state

Source revision: 9d26a58eb8516b6754c5d12b041cdc789950e047, repository
https://github.com/o-abolade/rrm.git. The source working tree was clean before adding
this handoff. No runtime code, Jira, or Confluence changes were made in this session.
The handoff is an additional uncommitted file included in the transfer archive.

| Path | Current responsibility |
|---|---|
| rrm/schema.py | Pydantic payloads, WorldBackend and ActionPolicy protocols |
| rrm/verbs.py | Authored preconditions/effects and predicate evaluation |
| rrm/reasoning.py | ScriptedOracle backward-chaining planner with predefined goal |
| rrm/loop.py | Outer planning/replanning and inner execution/effect checking |
| rrm/safety.py | Symbolic action and placeholder numeric trajectory safety gates |
| rrm/world.py, rrm/policy.py | Mock backend and policy |
| rrm/trace.py, rrm/benchmark.py | JSONL events and benchmark metrics/harness |
| simulation/isaac_backend.py | Relation inference, initial adapter renderer, Isaac/GR00T stubs |
| scripts/oracle_loop.py | CPU-only benchmark CLI |

Verified locally during review:

```text
python scripts/oracle_loop.py --suite    → 5/5 pass (T1, T2, T6, T8, T9)
python simulation/isaac_backend.py      → relation inference PASS
```

The local commands used the existing .venv, which is excluded from the archive.
Core dependency is pydantic>=2.0. Recreate the environment on the remote machine.
No Isaac/GR00T/learned-reasoner integration or GPU job has been run here.
Mock passes do not establish SIL integration or Phase 1 compliance.

### Gaps and conflicts found before any code changes

1. RobotState assumes one gripper, one held object, and radial reach. Core predicates
   depend on these assumptions. Numeric safety uses hardcoded joint/velocity limits.
   Declared capabilities/limits have no implemented contract.
2. ReasonerBackend combines reasoning and planning, receives no capability declaration,
   and the oracle is given its goal. Contextual interpretation is not implemented.
3. Clarification, approval, permission context, and external stop/override contracts
   are absent. Bounded abort is not confirmation of a physical safe state.
4. Safety rejection replans the same action until budget exhaustion. T6 starts with a
   human already present; it does not test human entry during motion or active stop.
5. World confidence/uncertainty fields do not drive escalation. Unknown predicates are
   treated as false, so negation can turn missing evidence into apparent satisfaction.
6. Telemetry lacks full relevant state, capabilities, permissions, and interventions
   needed for reconstruction. Action IDs are reused across plan versions; correlation
   semantics need definition.
7. Numeric path safety treats the first waypoint coordinates as Cartesian positions
   while also treating waypoint values as joints. It is a mock placeholder, not a
   verified embodiment safety model.
8. Isaac backend and GR00T policy are stubs. Adapter rendering has no declared support
   contract and defaults missing poses to zero; grounding failures need explicit handling.
9. The old docs commit to A10G, Panda, GR00T, Qwen/vLLM, ROS topology, and perception
   timing before requirement allocation. These are historical prototype choices,
   not current constraints. Panda-first differs from the stated CONOPS progression.

Preserve useful properties: abstract actions over object IDs, authored expected effects,
observed-state reconciliation, mandatory safety gates, bounded recovery, and a stable
deterministic oracle regression baseline. Do not assume the oracle is infallible.

## Proposed architecture allocation — not yet implemented

| Logical responsibility | Requirement allocation |
|---|---|
| Operator/task interface and reasoning | 001, 007; supports 003, 006, 009 |
| World/task state, observation ingestion, task memory | 002; supports 001, 005, 006, 011 |
| Planning | 003, 004; supports 006, 008, 010 |
| Safety/permission authority and execution supervision | 008, 009; supports 004, 006, 007 |
| Execution monitoring | 005, 006; supports 009, 011 |
| Embodiment declaration, adaptation, and execution | 004, 010; supports 005, 008, 009 |
| Telemetry/evidence | 011, with evidence from every boundary |

These are logical responsibilities, not a demand for separate processes or new packages.
Contracts to define before implementation:

- Task request: objective, context, constraints, approval/clarification state.
- World/task snapshot: version, observation times, provenance, uncertainty, progress.
- Capability declaration: supported actions/resources, limits, availability, feasibility.
- Reasoning result: interpreted goal, grounded entities, constraints, unresolved ambiguity.
- Plan: versioned actions, preconditions/effects, capability and state references.
- Safety decision: permission/safety result and context it applies to; dispatch must
  not reuse a stale authorization after relevant changes.
- Execution: acceptance, progress, completion/failure/interruption, observed outcomes,
  and explicit stop/cancel/override acknowledgment and safe-state reporting.
- Telemetry: correlated task/plan/action/dispatch/state/decision records sufficient
  for replay and assessment; numeric details and limits remain behind adapters.

## Remote AirStack / OSMO integration

User-supplied operating information (not verified against live cluster):

- 3 machines, 4 RTX PRO 5000 Blackwell GPUs each: 12 GPUs total.
- 48 CPU cores and 216 GB RAM per machine: 144 CPUs and 648 GB RAM total.
- Fair-use target per GPU: 12 CPUs + 48 GB RAM.
- Suggested maximum: 4 GPUs per person/project.
- LOW priority can use spare capacity and can be preempted.

Normal one-GPU request:

```yaml
resources:
  default:
    cpu: 12
    gpu: 1
    memory: 48Gi
    storage: 500Gi
```

500 GiB is the intended job request, NOT a verified pool maximum or persistent volume.
Actual GPU VRAM, driver/runtime compatibility, active workflow ID, remote branch, and
persistent result/cache location still need remote inspection. Do not assume the old
A10G model budget or precision restrictions apply to Blackwell.

### Current live Isaac / OSMO operating procedure (2026-09-13)

This later GPU workflow is live. The user started it through the normal AirStack
OSMO workflow procedure with the one-GPU fair-use request (1 GPU, 12 CPU, 48 GiB
RAM), opened the IDE tunnel with `./airstack.sh osmo ide`, and connected VS Code
to `airstack-osmo`. The AirStack Isaac livestream, robot desktop, and GCS
containers are running. This supersedes the earlier CPU-only transfer-workspace
observations in `docs/scrum-8/remote-state.md`; retain those only as historical
diagnosis of that separate `gpu: 0` workspace.

The desired manual Pegasus PX4 launch uses livestream and all of the following
runtime flags:

```sh
--/renderer/activeGpu=0
--/renderer/multiGpu/enabled=false
--/physics/cudaDevice=0
```

These flags are the deliberate single-GPU fix. In the nested OSMO runtime, Isaac
can enumerate all four physical GPUs even when the workflow requested one; without
the flags, its renderer can initialize allocations on all four. The flags constrain
rendering and physics to GPU 0.

Read-only process inspection on 2026-09-14 found that the *current* Isaac process was
instead started by the Compose `AUTOLAUNCH=true` command. Its command line includes
only `--/app/livestream/enabled=true`, not the three pinning flags above. Treat the
desired manual procedure as not applied to the current process. Do not alter Compose
or restart the live simulator solely to change this without user direction; apply the
pinned command at the next controlled launch.

The initial inspection occurred with the Pegasus timeline stopped, which stops the
PX4 backend while leaving the Isaac Python process alive. The user then pressed Play;
read-only checks confirmed PX4 running, MAVROS connected, and Isaac reporting its
first heartbeat and `Ready for takeoff!`. Leave Isaac open and playing while using the
robot stack.

The live robot stack initially had a MAVROS namespace duplication: MAVROS published
under `/robot_1/interface/mavros/mavros/*`, while AirStack's `robot_interface` and
`odometry_conversion` expected the canonical `/robot_1/interface/mavros/*` paths.
The source cause was `interface.launch.py` pushing `interface` while
`mavros_px4.launch.xml` also supplied `namespace="mavros"`; MAVROS's own relative
`mavros/*` topic names added the second level.

With user authorization, `interface.launch.py` was corrected to pass an empty MAVROS
namespace, and only `robot-desktop` was recreated (Isaac and GCS stayed running).
The workspace rebuilt successfully. MAVROS now lives at `/robot_1/interface/mavros`;
canonical MAVROS odometry has one publisher and feeds `odometry_conversion`, converted
odometry and `map -> base_link` TF stream, and the obsolete nested topic is absent.
A bounded `airstack ready --json` passed all gates: containers, `/clock`, autonomy
nodes, MAVROS-to-PX4 connectivity, and EKF odometry. No task or flight command was
sent. A model-free RRM shadow adapter may now consume the canonical state stream;
retain the separate safety gate before any execution integration.

The first adapter has been implemented and passively verified. Its deterministic core
is `rrm/airstack_shadow.py`; `scripts/airstack_shadow.py` is an `rclpy` observer that
subscribes only to canonical MAVROS odometry/state, `/tf`, and task-status topics. It
has no ROS action client, publisher, service client, trajectory, or PX4 command path.
Every snapshot remains `execution_inhibited=True`; the evidence manifest also declares
`execution_dispatch_enabled: false`. A 15-second live session emitted 15 snapshots
(14 fresh after one explicit startup-incomplete snapshot), with an observation-complete
final replay report. The shadow tests (5), existing safety/contract tests (14), and
the deterministic oracle suite (5/5) passed. The next safe increment is shadow
correlation of a separately user/GCS-initiated task, followed by offline RRM reasoning
evaluation; do not add dispatch authority as part of that work.

The first user-initiated task correlation is also complete. The user connected
Foxglove Desktop through the GCS websocket, then submitted one takeoff and one land;
the shadow observer recorded `EXECUTING → SUCCEEDED` for each. It recorded 160
snapshots (158 fresh after two explicit startup gaps), all execution-inhibited. Final
MAVROS/odometry evidence was connected, disarmed, near ground height, and effectively
stationary; its replay report was observation-complete and dispatch-disabled. GCS,
not RRM, issued the two task actions. Treat this as an evaluation baseline only; no
RRM action authority has been added.

For remote viewing, the user starts the patched Mac-side forwarder with the patched
OSMO binary first in `PATH`, then connects the AirLab Isaac Sim WebRTC Streaming
Client to `127.0.0.1`. The patch is required for the known stock OSMO 6.3.1 UDP
49099 forwarding failure.

OSMO workspace storage and Codex conversation memory are not durable across a
workflow replacement. Persist source and this handoff by committing and pushing;
export non-source evidence separately before workflow termination. In particular,
the AirStack feature `notebook/` is gitignored and is not a handoff mechanism.

Guide supplied by user:
https://docs.theairlab.org/0.20/docs/tutorials/airstack_on_osmo/

Guide architecture: OSMO workspace pod runs sshd and Docker-in-Docker; AirStack's
Isaac Sim, robot-desktop, and GCS run inside it. SSH alias airstack-osmo reaches
localhost:2200 through an authenticated OSMO port-forward. IDE workspace is normally
/root/AirStack. Files in the workspace pod are not automatically present at every path
inside the nested containers; inspect bind mounts and build in the correct container.

The guide's raw tunnel command is:

```bash
osmo workflow port-forward <workflow-id> workspace --port 2200:22 --connect-timeout 86400
```

AirStack branches are cloned remotely from GitHub on job startup; local unpushed edits
are not transferred automatically. ZIP transfer is the user's selected handoff path.
Unzipping stages the source; it does not integrate it into ROS or make it persistent.
Save/push source changes and export evidence before the ephemeral job is terminated.
Keep weights and datasets out of Git and the source archive; download pinned revisions
remotely and record their identifiers, licenses, precision, and checksums where available.

Local AirStack reference inspected earlier was on main, with modified
osmo/workflows/airstack-dev.yaml and untracked osmo/patch_osmo_udp.py. Those files are
NOT in this archive, and that state is NOT evidence of remote state. Remote AGENTS.md
is authoritative for AirStack editing/build conventions. The local reference required
a feature notebook before implementation and described external modules, stacks, and
fleets; inspect the remote version before choosing integration structure.

RRM should consume AirStack state and execute through explicit interfaces while keeping
its task-level core independent of AirStack's drone stack. Do not infer that using the
AirStack runtime changes the RRM scenario to aerial robotics. Select the SIL scene and
embodiment against CONOPS and document any deliberate change.

## Capstone and model assessment

Sources already inspected:

- https://001-physical-ai-book.vercel.app/docs/capstone/project-specification/capstone-project-specification-framework
- https://001-physical-ai-book.vercel.app/docs/capstone/implementation-guide/implementation-guide

Also inspect the evaluation page and the full Weeks 1–11 curriculum:
https://001-physical-ai-book.vercel.app/docs/capstone/evaluation-criteria/evaluation-criteria

The spec includes targets such as simple command processing <2 s, complex planning
<10 s, navigation planning <1 s, emergency stop response <0.1 s, perception >=30 FPS,
manipulation positioning within 1 cm, and average task completion within 30 s. It also
lists operational RAM <8 GB and models/data storage <100 GB. These are source targets
to map and assess, not proof that every target applies to the current SIL scenario or
that the larger OSMO allocation changes an operational requirement.

Next deliverable: source-linked metric → Phase 1 requirement → architecture owner →
scenario/test → measurement definition → acceptance target → evidence mapping.
Record applicability, measurement start/end boundaries, simulation versus wall-clock
time, trial counts, and unresolved conflicts. Keep system performance distinct from
individual model inference performance. No "best model" has been selected or verified.
Compare candidates remotely under the same scenes, inputs, resources, and measurement
conditions; separate reasoning, perception, and execution-policy contributions.

The implementation guide is a broad humanoid engineering checklist (layered design,
interfaces, incremental integration, safety, test and deployment practices), not a
drop-in RRM implementation. Its phase numbering differs from Jira Phase 1.

## Reachy Mini aside

The user asked for a separate architecture review. Reachy Mini has an expressive head,
rotating body, antennas, camera, microphones, and speaker. It uses an application SDK
and hardware daemon, with media streaming and a MuJoCo simulation option. It is useful
as a conceptual example of an embodiment that supports interaction but not grasping
or navigation. It has NOT been chosen as an RRM target.

- https://huggingface.co/docs/reachy_mini/en/SDK/core-concept
- https://huggingface.co/docs/reachy_mini/en/platforms/reachy_mini/hardware
- https://huggingface.co/docs/reachy_mini/en/SDK/media-architecture

An adapter must expose command adjustments and observed outcomes: e.g. a clamped pose
does not necessarily achieve the originally requested effect.

## Transfer status and next action

This archive includes the original RRM runtime code plus this handoff. It excludes
.git, .venv, model weights, caches, and experiment recordings. It is a starting source
snapshot, not a completed SCRUM-8 architecture or an integrated SIL deployment.

Begin with the architecture allocation and metrics mapping above; inspect the remote
environment in parallel with that read-only work. Then make the smallest requirement-
backed changes and verify them locally within the remote development environment,
before advancing to integrated GPU SIL trials.

## Continuation — 2026-09-17 UTC

The complete staged SIL integration plan is now in
[`docs/scrum-8/integration-plan.md`](docs/scrum-8/integration-plan.md). It translates
the logical C01–C09 allocation into gates: freeze the shadow baseline; select a
CONOPS-compatible hand workspace; make task/context/state/plan contracts executable in
shadow; build a dry-run adapter; implement independent supervision and safe-state proof;
then enable the narrow simulator-only command path and run the frozen S01–S10 campaign.
Do not skip directly from the existing read-only drone observer to PX4 or task-action
authority.

The current AirStack process was intentionally started by the user and is the existing
`example_one_px4_pegasus_launch_script.py` / Iris / PX4 drone launch. It remains an
aerial transport baseline, not the RRM manipulation scene. Do not modify or repurpose
it in place. The configured Isaac asset root is
`omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1`; the image enables
Franka and robot-motion/Lula infrastructure, but no local hand, Allegro, Shadow Hand,
or Franka USD asset is cached. Standalone Python does not expose `omni.client`, so no
Nucleus browse was attempted by opening a second Kit process. Select and record a
versioned dexterous-hand asset/controller, reset method, observation channels and safe
state before a separate controlled hand-scene launch. Foxglove is not required for the
contract work and was not started by this continuation.

`rrm/task_contracts.py` and `tests/test_task_contracts.py` now implement the first
proposal-only C01/C04/C05 foundation. `TaskRequest`, `Interaction`, `ReasoningResult`,
`PlannedAction` and `PlanProposal` are immutable Pydantic records. Plans require a
ready intent with matching task/state/capability revisions, authored verb arity, unique
action IDs and acyclic dependencies. Material ambiguity cannot become a plan. These
records have no ROS dependency and do **not** implement C06 admission, C07 dispatch, a
controller command, a publisher, an action client, a service client or any execution
authority.

Validation on 2026-09-17 used Pydantic in an isolated `/tmp/rrm-contract-deps` target
because host Python has neither the dependency nor `venv` support; no system packages,
AirStack image, simulator scene, or Foxglove client were changed. Results: 5/5 new
task-contract tests, 24/24 total shadow/contract/unit tests, and the original Oracle
suite 5/5 (T1, T2, T6, T8, T9). The local feature notebook is gitignored and is not a
handoff mechanism; durable source/doc changes still need commit and push, while runtime
evidence must be exported separately before workflow replacement.

`rrm/state_contracts.py` and `tests/test_state_contracts.py` now add the C02
evidence-backed state foundation. `FactEvidence` records semantic fact identity, truth,
provenance, source reference and freshness bounds; `StateSnapshot` resolves a fact only
from fresh, explicit, non-contradictory evidence. Missing, stale, UNKNOWN,
contradictory and negated-UNKNOWN evidence resolve to UNKNOWN. Coverage metadata does
not silently convert an absent fact into negative evidence. This deliberately remains
separate from the legacy mock `WorldState` and creates no ROS, adapter, safety-admission
or execution path. Five state-contract tests passed; the total unit suite is now 29/29
and the Oracle regression remains 5/5. The feature notebook is gitignored; commit/push
the source and documentation changes before workflow replacement.

The user clarified that “planning” means RRM output to the current drone, not merely
symbolic planning. The chosen integration seam is AirStack's public task-action boundary
used by Foxglove: `/{robot}/tasks/takeoff`, `/navigate`, and `/land`. Do not issue raw
PX4/MAVROS/service/trajectory commands. `rrm/airstack_drone.py` contains immutable
typed proposals; `scripts/airstack_drone_dispatch.py` maps them to the exact
`task_msgs` action goal shape. It prints a dry-run by default without importing ROS;
only `--execute` creates one ActionClient and sends one selected task goal. A dry-run
takeoff fixture verified `/robot_1/tasks/takeoff`, `TakeoffTask`, altitude 2.0 m and
velocity 1.0 m/s with `execution_requested: false`. No live task was sent. Four adapter
tests passed; the total unit suite is 33/33 and the Oracle regression is still 5/5.
This is an output adapter, not the complete C06 admission/C08 stop authority; require an
explicit user-approved simulation proposal before the first `--execute` invocation.

## First controlled drone-action SIL run — 2026-09-17 UTC

The user explicitly approved the bounded Isaac/PX4 sequence “take off to 2 m at 1 m/s,
then land.” This was treated as two separately completed public AirStack task actions,
not a direct PX4/MAVROS command or an autonomous multi-action chain. The live
`airstack-robot-desktop-1`, `isaac-sim-livestream`, and `airstack-gcs-1` containers were
up; `/robot_1/tasks/takeoff` and `/robot_1/tasks/land` each reported one task-action
server at `/robot_1/takeoff_landing_planner/takeoff_landing_task`.

The runner and its Pydantic dependency were staged only under the robot container's
temporary paths `/tmp/rrm-sim-adapter` and `/tmp/rrm-sim-deps`; neither the source
workspace nor the AirStack image is mounted or persistently modified. Both proposals
first completed dry run in that same container: `takeoff-1` mapped to
`TakeoffTask(target_altitude_m=2.0, velocity_m_s=1.0)` and `land-1` mapped to
`LandTask(velocity_m_s=1.0)`.

An initial execution attempt failed before an ActionClient was created because staging
overwrote the ROS `PYTHONPATH`, so no goal was sent. Preserving the existing ROS path
and adding the temporary adapter/dependency paths passed `import rclpy` and
`TakeoffTask` preflight. The subsequent takeoff returned
`success: true, message: "takeoff complete"`; after that terminal result, the land goal
returned `success: true, message: "landing complete"`. The task-server feedback was
received throughout (empty status for takeoff; `landing` for land). This establishes one
successful command-and-result path through RRM → AirStack task actions → existing
drone stack. It does **not** independently prove pose/visual observation agreement,
C06 admission, C08 stop authority, fault handling, repeatability, or autonomous
sequencing; those remain required before any flight-readiness claim.

## Outcome-verification increment — 2026-09-17 UTC

The next robustness increment adds `OdometryEvidence`, `VehicleStateEvidence` and
`verify_drone_outcome` to `rrm/airstack_drone.py`. It is a pure, fail-closed evaluator:
it verifies a successful takeoff only with fresh independent `map -> base_link`
odometry within the AirStack planner's 0.3 m absolute-altitude acceptance distance and
a fresh connected/armed vehicle state; it verifies landing only with fresh near-ground
odometry plus connected/disarmed vehicle state. Missing, stale, future, wrong-frame,
unsuccessful or mismatched evidence remains `UNCONFIRMED` or `MISMATCH`; navigation is
verified only with fresh causal odometry within its proposal endpoint tolerance.

`scripts/airstack_drone_dispatch.py --verify-observation` now adds only read-only
subscriptions to `/{robot}/odometry_conversion/odometry` and
`/{robot}/interface/mavros/state`. It requires fresh pre-dispatch odometry, captures
post-result samples, writes a JSON outcome when `--outcome-json` is supplied and exits
nonzero unless the independent evidence verifies the action-server success. It retains
the explicit `--execute` gate and has no publisher, service client, direct PX4/MAVROS
command or trajectory interface. No task was dispatched while implementing this
increment. A separate eight-second observer-only run after the first SIL sequence
reported `observation_complete: true`, MAVROS connected/disarmed, fresh map/base-link
evidence and `z=0.0201 m`; it corroborates the final landed state but is not causally
attached to either earlier action.

## Office learned-proposal flight attempt — 2026-09-18 UTC

PSC job `46288765` produced an accepted canonical proposal:
`NAVIGATE_TO blue_marker`, mapped by the importer to `(3.2, 0.0, 1.5)` in `map`
with 0.3 m tolerance. The retrieved bundle passed its manifest hashes, local source
hashes, raw-response reparse, candidate equality, and scene-binding import checks.

The running simulator was replaced with `rrm_office_visual_eval.py` and the Office
scene. Restarting Isaac without the older robot stack initially reset `/clock` under
retained TF state; continuous `TF_OLD_DATA` warnings accompanied a drifting takeoff.
That attempt was stopped before navigation and landed/disarmed. Restarting Isaac first
and the robot stack second cleared the warnings. A final hardened takeoff was verified
from `(0.0152,-0.0134,0.0091)` to `(-0.1913,-0.0384,1.2755)`, connected and armed,
with about 0.21 m XY displacement. `verify_drone_outcome` now rejects takeoff XY
motion over the configurable default of 0.3 m.

The learned navigation goal was **not sent**. The `droan_gl_node` owning
`/robot_1/tasks/navigate` had already segfaulted at startup (`exit -11`). Its stale DDS
action name remained visible, but the dispatcher's live `wait_for_server` handshake
failed closed. Server-unavailable attempts now write `goal_sent=false`,
`physical_outcome=NOT_DISPATCHED`, `verdict=UNCONFIRMED` rather than raising without
an outcome record. Recovery landing independently verified
`(-1.2527,-0.8684,0.0061)`, connected and disarmed. Navigation was not retried in
flight, and raw control interfaces were not used.

Ground-only follow-up isolated that crash to OpenGL initialization. Xvfb display 99's
lock/socket survived the robot-container restart while the Xvfb process did not, so
the replacement X server exited with “Server is already active” and `droan_gl_node`
then printed `Failed to initialize GLAD` before dumping core. AirStack's
`robot/docker/docker-compose.yaml` now removes only `/tmp/.X99-lock` and
`/tmp/.X11-unix/X99`, and only after proving no Xvfb process exists. Repeating that
startup path on the grounded vehicle produced Mesa OpenGL 4.5, loaded all Droan GL
shaders, kept the node alive, and exposed a live navigate action server. Compose
configuration validation passed. The manual diagnostic node was stopped afterward;
apply the patched Compose command through a clean robot-container recreation and rerun
preflight before any later flight.

## Office Learned-Proposal Navigation Fix — 2026-09-18 UTC

A subsequent flight trial was executed with the patched Xvfb startup. The dispatcher successfully connected to the `NAVIGATE_TO` action server. The takeoff and land actions were fully verified, yielding perfectly clean odometry (the drone maintained a stable `(0, 0)` XY hold due to a clean simulator boot). However, the `NAVIGATE_TO` goal (waypoint `3.2, 0, 1.5`) timed out after 120s with the drone hovering near the origin.

Root cause analysis revealed that the `droan_local_planner` configuration (`config/droan.yaml`) had a highly restrictive `robot_radius: 1.0` and `obstacle_check_radius: 1.0`. The `blue_marker` cube in the scene is located at `x=4.0` with a size of 0.8m (front face at 3.6m). With a 1.0m radius, the drone perceived the target waypoint `3.2` as being deep inside an obstacle inflation zone and refused to plan a forward path. The planner configuration was subsequently tuned to realistic dimensions (`robot_radius: 0.3`, `obstacle_check_radius: 0.4`) and the autonomy nodes restarted.

## C06/C08 UI & Autonomy Pipeline Requirements

In preparation for closing the execution loop, the following architectural and UI changes were discussed and/or implemented in the Command Console (`scripts/ui/command_console.html`):

1. **GUI Fixes & Reset Functionality:**
   - The "Saved goals & history" panel was made scrollable (`overflow-y: auto`) to accommodate lengthy evaluation sessions.
   - A `Stop & Reset Drone` button was added, which invokes an `/api/reset` endpoint to cleanly run `airstack down` and `airstack up`. (Note: restarting the containers causes the WebRTC stream on `127.0.0.1` to temporarily drop, reinforcing the need for ROS-level stop/cancel controls).
   - Foxglove's `/map` voxel grid was adjusted to be transparent to allow clear visualization of the robot.

2. **C08 Stop Authority & Quick Actions (Pending Implementation):**
   - The user requested dedicated Play/Pause, Takeoff, Land, and Reset (RTL) buttons.
   - Instead of restarting the simulator to reset the position, a ROS-level RTL (`NAVIGATE_TO (0, 0, 0.07)`) and a ROS-level Task Cancel (C08 Stop Authority) should be implemented to abort actions mid-flight without breaking the WebRTC feed.

3. **C06 Admission Control (Pending Implementation):**
   - The GUI explicitly displays "Task intake · execution disconnected", enforcing that the console is an intake boundary, not an autonomous dispatcher.
   - The user requested that the GUI natively present the Cosmos-Reason2 `result.json` flight plan and require explicit "Accept/Reject" approval. This fulfills the **C06 Admission Control** milestone.
   - While the user articulated a Level 5 autonomy vision (where the `NumericSafetyVerifier` replaces the need for human C06 approval and prioritizes its own queue), the immediate Sprint 8 requirement demands that C06 remains in place until the model's bounds are fully validated.

Next operator: Implement the C06 Admission Control UI to read `result.json` and trigger `airstack_drone_dispatch.py` upon approval, and add the C08 ROS-level Stop button to the GUI.
