# Goal-to-finish performance status

## Current continuation — 2026-10-05 EDT

Actual grounded physics callback measurement confirms **4.112 ms backend clock
loss over 41.13 simulation seconds**: all 456 recorded windows exactly match
per-step microsecond truncation. Six observer plus ten recorder tests, independent
review, real Office GUI reload and readiness 7/7 pass. Maximum capture receipt gap
was 0.740 s; this is grounded measurement, not flight performance qualification.
Next is a fraction-preserving clock correction with grounded regression. Older
collapse causation, epoch/acquisition registration and broader reliability remain
open. See [HANDOFF.md](../../HANDOFF.md) for source identity and measurement limits.

Read-only raw timing capture now retains PX4 packet timestamp fields alongside
raw/converted odometry and available timing metadata. Eight tests pass, and the
reviewer reproduced all 731 unique-position associations with unchanged headers.
Fresh live GUI/grounded readiness passes 7/7. Clock-origin/acquisition registration
remains unresolved; approximately 315 ms header/packet differences are not measured
delay, and absent timing status does not imply zero offset. See [HANDOFF.md](../../HANDOFF.md).

The latest grounded measurement diagnosis adds a tested offline pose comparison.
Height offsets vary between captures, and receipt-phase timestamps do not establish
PX4 acquisition alignment. Legacy sensor-state fields are physical sensor inputs,
not PX4 belief. Fresh Office GUI, grounded capture and readiness 7/7 pass; no new
flight or timing/control change. See [HANDOFF.md](../../HANDOFF.md) for the next raw-time
instrumentation step and the retained comparison assumptions.

The 2026-10-05 physical-truth continuation VERIFIED one GUI-reviewed Office
TAKEOFF 1 m → forward 0.25 m → left 0.25 m → ordinary LAND/disarm replay,
with navigation errors 0.023108/0.008781 m and no recovery/retry. Direct PhysX
capture confirms motion and return to rest; physical displacement differs from
odometry by about 3 cm, with timestamp/phase uncertainty retained. The new opt-in
AirStack recorder is reviewed, tested and reloaded while grounded; all seven
readiness gates pass. The older Pegasus diagnostic commit remains unavailable.
This qualifies one short two-leg replay, not mixed-height/longer reliability,
collision containment or the older collapse cause. See [HANDOFF.md](../../HANDOFF.md).

Live GUI replay at21:10EDT VERIFIED takeoff1m → forward0.25m → ordinary LAND/disarm,
navigation endpoint error0.018999m. Scene launch/camera/staging/execution were driven
through the running browser GUI. All16 control streams were captured for180s;
658 active PID callbacks were admitted, with no future-tracking rejection while
armed/control authority was active. No recovery or retry. Unknown-scene presentation
and implicit initial scene selection are repaired. Earlier collapse causation,
mixed-height/longer reliability and physical containment remain open.

Latest committed evidence (`e633a658`) records a narrow serial Office takeoff 1 m,
forward 0.25 m, and ordinary LAND/disarm success with navigation error 0.02198 m.
The older forward-collapse cause and mixed-height/longer mission qualification
remain open. The historical paused-flight and source-only deployment checkpoints
below retain their original dates; they do not describe current deployment.

The current workspace now uses the published NumPy-fix Isaac digest. NumPy 1.26.4,
seven readiness gates and fresh camera capture are verified. A subscription-only
control capture observes deployed admission diagnostics while grounded; receipt
coverage does not establish airborne performance. That initial image-reset step
dispatched no flight; the later GUI replay checkpoints above report actual flights.
See [handoff](../../HANDOFF.md) for coverage, observed reasons and limitations.

This page distinguishes a connected software path from demonstrated autonomous task
performance. The target loop is:

```text
goal -> GUI -> semantic RRM -> capability route -> embodiment grounding
     -> independent safety/admission -> actuation -> observation/replan
     -> independently verified goal or explicit terminal failure
```

## Core evaluation checkpoint — 2026-10-03

Source-only continuation2026-10-04 02:36EDT: PID callback reason snapshots and
deterministic clock-order tests pass (11gtests,26lifecycle,6clock phases).30ms future
tracking/odom inputs yield conservative idle/reset reasons; clock catch-up reactivates
cleanly. No freshness relaxation. Both diagnostics and armed-aware authority repair
remain undeployed. Next is separately admitted grounded deployment/read-only coverage,
not flight. Historical flight lacks these callback reasons; primary cause unproven.

Latest flight checkpoint (2026-10-04 01:49EDT): one reviewed instrumented Office diagnostic
after actuation/authority deployment failed the0.3m lateral takeoff bound. Armed
OFFBOARD and lift were observed; ROS takeoff-window maximum altitude estimate0.504m
and displacement0.326m at outcome. Hold/LAND accepted, ground/disarm independently
observed, GUI HALTED/inactive. Full180s ROS/PX4 traces retained, no retry. Thrust
saturated and a sharp downward estimate excursion occurred; OFFBOARD/control
remained selected after disarm, final integrals0. Next is source-only tracking/
frame/actuation and mode-lifecycle investigation, not another flight. No nominal
goal or physical-containment qualification; ordinary flight stays paused.

Source-only continuation2026-10-04 02:09EDT: armed-aware has_control reporting
passes actual-plugin transition/rearm tests; live interface unchanged. Offline
correlation verifies PID/cascade/raw thrust consistency and observational ROS/PX4
state matching with an explicit uncertain clock fit. Early vertical lag and
candidate future-tracking timestamp resets remain unresolved, not a proved gain
or plant defect. PID future-stamp rejection/clean reactivation passes19isolated
phases, plus9gtests. Next: reason-coded admission/idle diagnostics and isolated
clock-order tests; strict freshness/bounds unchanged, no deployment or new flight.

Prior grounded deployment passed18plugin+21task+45readiness/intent tests. Effective
scaling1.0 and readiness7/7 were confirmed before this attempt. GUI launch itself
does not query the gate; routed interface commands do. Historical checkpoints below
retain their original claim scope and do not override this latest result.

The CPU suite passes 429/429 tests, including 14 simulator dependency preflight and
launch-admission tests. Console source rechecks the pinned Isaac NumPy profile before
new non-LAND missions; LAND/STOP recovery remains available.
The 2026-10-04 candidate image checkpoint supersedes the earlier recreation failure:
the published Isaac candidate is pinned by manifest digest in this branch's `.env`.
Recreation retains NumPy 1.26.4; console state is COMPATIBLE/MATCHED and all readiness
gates pass. An 8-second read-only window received 180 camera frames, 56 raw LiDAR
clouds and a populated VDB map; the GUI camera capture also succeeds. The vehicle
was grounded/disarmed at that image checkpoint; no flight had yet been attempted.
The subsequent 00:15UTC bounded flight checkpoint failed: one GUI 1m takeoff
violated its vertical-speed bound and physically overshot to2.888m. Predeclared
recovery LAND independently VERIFIED grounding/disarm; terminal state RECOVERED_HALT.
Grounded diagnosis confirmed retained vertical PID integral saturating thrust while
disarmed and an automatic reset path available only for ArduPilot, not current PX4.
The retained PX4 log shows high thrust during takeoff/after abort, but flight PIDInfo
is absent; exact initial controller state and full causation remain open.
PID lifecycle/anti-windup source repair subsequently passed9 unit tests and17
synthetic phases in isolated ROS domain197, with reviewer clearance. It is now
deployed to domain1 via mounted source/install and robot-only restart. Two grounded
readbacks show all integrals0 and baseline thrust0.71 reaching MAVROS; PX4 remained
disarmed and readiness passes. This does not qualify airborne baseline behavior
or abort containment. No new flight has occurred.
Takeoff-envelope failover source now requests immediate LAND and keeps recovery
observation-only after sent/uncertain handover. Synthetic action tests cover
authority ordering and cancel/timeout cleanup. This task-node repair is now deployed
alongside PID via another robot-only restart at02:17UTC; all8 isolated scenarios
pass, executable hashes match, readiness passes, and grounded readback keeps all
integrals0/PX4 disarmed. No new flight or physical containment qualification occurred.
Next is the [instrumented Office plan](office-control-qualification.md).
The console now has deployed two-stage plan review/execution; a live no-dispatch
staging check verified saved plan/recovery/hash and retained grounded idle state.
This is admission plumbing evidence, not a new integrated motion outcome.
The subsequent single instrumented attempt HALTED/no lift: live MAVROS raw-setpoint
plugin thrust_scaling=NaN, ignored actuation, no observed OFFBOARD authority. PX4
auto-disarmed grounded; GUI STOP ended pending takeoff. Complete ROS/PX4 evidence
is retained. No nominal takeoff/LAND, active PID or abort-containment qualification.
Grounded config/readiness and observed-authority admission repair is next.
Ordinary Office flight on this candidate is paused pending qualification, with no
retry or relaxed bounds. Outer workspace/robot image provenance and wider physics
qualification remain open. Environment/input recovery and safe terminal grounding
do not constitute successful goal-to-finish evidence.

T now exports a fixed 17-case
[acceptance campaign](../core-acceptance.md) with authored event-scoped safety labels
and a [paired comparison report](../core-comparison.md) over matching, same-source
campaigns. Every scheduled attempt and unknown outcome remains in reporting.
Retained v2 runs exposed evidence-write deadline/replay gaps at the short mock
fixture bounds. Current v3 evidence reconstructs only receipt-proven, adjacent late
commits; pending/missing/reordered writes remain incomplete. Two retained v3 runs
each qualified 31/34 attempts and matched 33/34 expectations, preserving one additional
observed write-liveness failure plus the two intentional trace-loss cases. The latest
fixture increment moves safety-sidecar disk synchronization from each gate to attempt
finalization, before worker result publication. Two fresh exports each matched 34/34
expectations and qualified 32/34 attempts, retaining the two intentional trace-loss
cases as UNKNOWN. Finalization errors cannot publish completed worker results;
remaining callback scheduling and storage stalls are still open. This advances
measurement and reconstruction; it does not add live
goal-to-finish evidence to the dated assessment below. Comparative architecture/model
trials and externally adjudicated integrated outcomes remain outstanding.

## Live-path assessment — 2026-09-27

There is not yet enough repeated live evidence to report a meaningful end-to-end
success rate, latency distribution, or recovery rate. At that checkpoint the dependency-light
suite passed 279/279 tests; unit and fake-adapter coverage is not a physical-task benchmark.

| Segment | What has been demonstrated | Current limitation |
| --- | --- | --- |
| Goal -> GUI | Immutable goal/attempt storage, deterministic parsing of a narrow aerial movement grammar, clarification for selected ambiguous forms, and one exact Kuka-Allegro placement goal exposed as a non-executing preview | The hand preview uses a synthetic fixture and a selected block field; arbitrary language and deictic visual grounding are not general |
| GUI -> RRM | Typed plans, discovered-server checks, numeric provenance, predeclared recovery, evidence display, and persisted `GoalRequest -> route -> C01 -> C04/C05` hand-preview records | The direct execution path remains a deterministic aerial adapter; the neutral hand path ends before numeric feasibility and C06 |
| RRM -> actuation | Public AirStack task actions only; two consecutive Office GUI takeoff/explore/land missions independently verified all actions with over 2.9 m measured exploration radius | No direct PX4/trajectory authority is intended; warehouse-shelves takeoff failed three fresh regressions; other scenes remain unqualified |
| Actuation -> replan | Fresh state is reacquired between actions and produces `CONTINUE`, `SKIP_SATISFIED`, or `HALT`; AirStack exploration/navigation planners can publish route updates | RRM currently performs inter-action reconciliation, not general semantic replanning from arbitrary observed divergence |
| Replan -> finish | Terminal outcomes distinguish `VERIFIED`, `HALTED`, `RECOVERED_HALT`, and `RECOVERY_FAILED`; a failed warehouse takeoff triggered a separately verified recovery landing; uncertain takeoff failures retain supervision for delayed airborne evidence | One recovered failure does not establish a repeated nominal/off-nominal acceptance matrix |
| Other embodiments | Kuka-Allegro has qualified a bounded calibration action and its C06/C08/C09 boundary in an isolated live fixture; neutral goal routing has CPU coverage | No qualitative hand goal has run through GUI -> reasoning -> `GRASP`/`PLACE` -> observed task completion; rover/mobile-manipulator routes are contracts only |

## Live evidence that bounds the claim

- A 2026-09-21 GUI mission took off toward 1.0 m and landed; both task outcomes were
  independently `VERIFIED`, with 0.182 m takeoff horizontal displacement and a final
  connected, disarmed, on-ground observation.
- The learned Office `NAVIGATE_TO` run demonstrated inference, proposal import, public
  action dispatch, and recovery landing, but navigation timed out after 120 seconds and
  the target was not achieved. This is a transport/integration demonstration, not a
  successful end-to-end task.
- A later 2026-09-24 GUI regression aborted takeoff after exceeding its 0.30 m lateral
  bound. During recovery the vehicle climbed to 4.076 m despite a 1 m request, then
  landed grounded/disarmed. The terminal state was `RECOVERED_HALT`, not task success.
- A 2026-09-26 instrumented regression (`ba6d25fa`) completed takeoff/land with both
  actions `VERIFIED`, but MCAP bag analysis revealed 0.65 m X-drift during the mission.
  Root cause: the trajectory controller's sphere-intersection algorithm advanced the
  tracking point to z=1.0 in ~3.5 s while the drone was still on the ground (z=0.017).
  The Sep-24 4.076 m climb is now confirmed to have been caused by this same
  tracking-ahead phase: when lateral abort fired, the old landing code started from the
  stale tracking point rather than fresh odometry. Fixes applied: `sphere_radius`
  reduced 1.0→0.3, `velocity_sphere_radius_multiplier` 1.0→0.5, and landing trajectory
  bounded to `-(altitude + 1.0)` instead of −10000.
- A subsequent 2026-09-26 verification flight (`949b6027`) successfully completed
  the 1.0 m takeoff and landing with both actions `VERIFIED`. Takeoff horizontal
  displacement dropped from 0.65 m to **0.005 m**. The z=-9999 issue is resolved.
  This clears the Office baseline; it does not qualify every Isaac catalog scene.
- A later 2026-09-26 `warehouse-shelves` mission (`a0ae6283`) failed takeoff after
  1.226 m horizontal displacement. Its terminal action observation was armed at
  z=-0.362 m, so immediate recovery correctly refused a blind landing; the simulator
  then evolved to a consistent armed/airborne state after the mission process had
  exited. The vehicle was reset grounded/disarmed without claiming mission success.
  The mission supervisor now persists a post-failure monitor, requires two consecutive
  fresh connected/armed/airborne observations above 0.3 m, and only then sends the
  predeclared recovery landing. Two consecutive fresh grounded/disarmed observations
  terminate safely without dispatch; unresolved evidence becomes explicit
  `RECOVERY_FAILED`. The exact delayed-ascent sequence is covered deterministically,
  but **Warehouse aerial flight remains paused pending a new qualified regression**.
- A subsequent Office goal (`92b3863e`) took off and landed, but its 30-second
  exploration action moved at most 0.107 m from its start. The action's timer returned
  success, causing a false `VERIFIED` mission label. The exploration action and RRM
  verifier now require at least 0.5 m of independently observed horizontal excursion;
  NavigateTask completion now uses physical odometry and the random-walk endpoint
  tolerance is 0.5 m instead of 3 m. These changes compile and pass unit tests but
  had not yet produced a qualified exploration flight at that checkpoint.
- The next bounded Office regression (`719298f1`) never reached exploration: takeoff
  aborted for lateral drift, with 0.574 m measured horizontal displacement at the
  terminal observation. Fresh state remained armed with contradictory low-altitude
  airborne evidence, so the monitor refused a blind landing and the simulator was
  reset. Office aerial execution was paused at that checkpoint.
- A later candidate controller limits the virtual tracking point to 0.5 m from
  physical odometry, rather than the previous 1000.5 m. A landing-stall fallback
  requests PX4 `AUTO.LAND`, and successful disarm returns PX4 to `AUTO.LOITER`.
  Two consecutive Office takeoff/land missions (`office-repeat-b713f6cb` and
  `office-second-c59d7189`) passed independent verification without a restart;
  takeoff horizontal displacement was 0.170 m and 0.028 m. This re-establishes a
  **narrow Office takeoff/land baseline** at that checkpoint.
- A new warehouse-shelves GUI takeoff/explore/land attempt (`2bb8476f`) failed
  takeoff before exploration, with observed altitude −4.65 m and 1.35 m lateral
  displacement. The stage contains a second `PhysicsScene` absent from Office.
  Preserving the World scene and disabling the duplicate did **not** fix the fall:
  a second attempt (`f63e0584`) reached −4.86 m. An explicit test collision slab
  altered the failure but did not qualify flight; a third attempt (`eec771ba`)
  failed takeoff at −1.10 m, with recovery landing independently verified.
  The slab was removed. Warehouse aerial execution remains **paused**. The GUI
  was returned to Office after these tests.
- Two 2026-09-27 Office GUI missions (`3c14d513` and `202ab815`) requested
  takeoff, 15 s of exploration, then landing. Each independently verified all
  three actions and the mission terminal status was `VERIFIED`. Exploration
  reached maximum horizontal radii of **2.981 m** and **2.915 m**, respectively,
  compared with the earlier 0.107 m false success. The second ran immediately
  without a restart. PX4 ended connected, disarmed, and in `AUTO.LOITER`.
  This qualifies the bounded Office deterministic command path for this goal;
  it does not qualify warehouse or general semantic replanning.
- The hand Gate 4/5 fixture exercises only an authenticated, bounded joint calibration
  action. It explicitly does not authorize semantic `GRASP` or `PLACE` execution.

## Meaning of "finish"

`SUCCEEDED` from an action server is not enough. A mission finishes successfully only
when fresh independent observations establish the requested semantic effect. A safe
landing after a failed mission is successful recovery but remains failed task
completion. Unknown evidence produces an explicit halt, never inferred success.

## Minimum evidence needed for a performance claim

Run a version-pinned simulator acceptance matrix with exported artifacts and count all
attempts, including failures:

1. repeated nominal qualitative goals for at least aerial, ground, and manipulation
   profiles;
2. ambiguity, unavailable-capability, stale-state, unsafe-route, execution-failure,
   and stop injections;
3. per-stage latency, semantic-goal accuracy, dispatch count, independently verified
   goal success, safe terminal-state rate, and recovery rate;
4. distinct reporting for deterministic parsing, learned reasoning, route replanning,
   and physical execution;
5. immutable manifests containing source, scene, controller, model, seed, and evidence
   hashes.

Until that matrix exists, the correct characterization is: **strong contract and
fail-closed component coverage; a narrow partially demonstrated aerial loop; no
validated cross-embodiment autonomous goal-to-finish performance result yet.**
