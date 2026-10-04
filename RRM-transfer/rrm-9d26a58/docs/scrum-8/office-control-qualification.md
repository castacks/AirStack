# Office control qualification — next bounded investigation

Prepared2026-10-04 after PID and abort-handover repairs. This is a test plan, not
a completed qualification or authorization to resume ordinary Office/Warehouse
flight. Execute only in the simulator under a separately authorized diagnostic
chunk, with a reviewer. Never apply this plan to physical hardware.

## Admission blocker found — 2026-10-03 22:27 EDT

Source update22:48EDT: the two-stage lifecycle below is now implemented/tested on
`ore_proj`. Backend stages the actual
artifact, requires its reviewed hash, claims launch once, verifies plan/source
snapshots remotely and rechecks admission. Update23:38EDT: deployed console-only,
with no-dispatch live staging check passed. Actual plan hash/recovery/source
bindings verified; no flight. Next is separately authorized instrumented single
attempt under every gate below; staged discovery is not current flight admission.

The previous GUI `/execute` compiled/saved `command-plan.json` and launched the
mission in the same request. There is no opportunity to review that actual saved
artifact before dispatch, so this plan's hard gate was not satisfied. No flight
was dispatched. A20s read-only recorder check received every required stream,
but does not remove this lifecycle blocker.

Next implementation: stage an immutable plan without launching a process; inspect
its actions, distinct recovery LAND and hash; explicitly execute that exact
reviewed hash once, with fresh state/dependency/clock rechecks and fail-closed
handling of changed plans/state. Test/review the two-stage lifecycle separately
before another diagnostic flight. A compiler preview is not the runtime artifact.

## Before dispatch

Source-only checkpoint2026-10-04 02:36EDT: callback reason diagnostics and30ms
deterministic clock-order regressions pass without relaxed freshness. Candidates
remain undeployed; separately admitted grounded deployment/read-only diagnostic
coverage is next. No new flight authorization or prior-flight root-cause claim.

Source-only checkpoint2026-10-04 02:09EDT: armed-aware authority reporting passes
isolated plugin tests but is not deployed. Offline correlation identifies early
vertical lag and candidate future-stamp PID idle resets; exact physical cause
remains unproven. PID future-input rejection/reactivation tests pass without any
freshness relaxation. Reason-coded admission/idle diagnostics and isolated clock-
order tests are next. This checkpoint does NOT authorize another flight.

Instrumented checkpoint2026-10-04 01:49EDT: exactly one newly staged/reviewed execute
after repair observed armed OFFBOARD/lift, then failed0.3m lateral bound. Accepted
hold/LAND was followed by independent ground/disarm, GUI HALTED/inactive. Complete
ROS/PX4 logs show takeoff-window max altitude estimate0.504m, displacement0.326m at
outcome, downward estimate excursion and thrust saturation. Final PID integrals0;
OFFBOARD/control remains selected after disarm. No nominal or physical-containment
qualification. No retry: source-only diagnosis of tracking/frame/actuation and
mode lifecycle, any separate repair/test/review, and fresh authorization are required
before reusing this flight plan. Current control=true also fails idle admission.

Grounded repair checkpoint2026-10-04 00:56EDT: fresh post-request armed/control
gating, authority-loss/uncertain-service containment and durable child initialization
are deployed. Interface independently verifies effective parameter before ARM/
control/takeoff. Child scaling1.0, startup verified, readiness7/7, post12s trace
grounded/disarmed/all integrals0. Reviewer cleared grounded scope, not flight.
Robot restart invalidates prior staging identity: create/review a new immutable
plan before a separately authorized diagnostic; retain all gates below. No retry
or ordinary flight is authorized by this repair checkpoint.

Update2026-10-04 00:01EDT: the single instrumented attempt HALTED without takeoff.
Live raw-setpoint plugin `thrust_scaling` is NaN despite source1.0; MAVROS reports
ignored thrust. PX4 stayed AUTO.LOITER, no observed OFFBOARD/control, auto-disarmed
grounded; GUI STOP ended the still-ascending task. No retry. Both active PID and
abort behavior remain unqualified. Add verified finite configured thrust scaling/
actuation-path readiness and observed authority/bounded takeoff checks via a
separate grounded repair/test/review before this plan can be used again.

Record source/working-tree hash and **running executable** hashes for PID and
takeoff/landing nodes; image/container IDs, Office/scale1.0, NumPy1.26.4, ROS clock
epoch, all readiness gates and inactive mission. Both mounted-source repairs must
be deployed; a release robot image alone does not contain these uncommitted fixes.
Require fresh connected/disarmed/grounded state, near-zero speed, all6 PID integrals
zero, single command publisher and correct canonical state subscriptions. Confirm
fresh camera/LiDAR/VDB and recovery endpoints. Sensor freshness is broader stack
evidence, not proof of takeoff/landing control safety. Do not relax bounds/gains.

Start bounded independent recording BEFORE console admission. Require observed
samples for odometry, tracking point, all6 PIDInfo, is_armed/has_control, MAVROS
state/extended-state and ROS command/raw attitude. Record ROS stamps AND steady
receipt times; retain complete sequence, not just last values. Establish matching
PX4 ULog/actuator collection for the attempt; if unavailable, no causal actuator
claim is allowed. Recorder failure or late start is a hard admission failure;
fail admission if required recording streams are absent/stale.

## One attempt only

Use the public console path with immutable reviewed plan: takeoff to1m at0.5m/s,
then LAND, plus the already declared recovery LAND. Verify the saved console
plan/artifact actually contains that separate recovery LAND before dispatch.
Current boundaries remain lateral0.3m, altitude overshoot0.3m, upward speed1.5m/s,
preflight hold error0.1m /3 confirmation samples /2s timeout,
landing stall5s/total60s. Record actual live values; differences invalidate this
test plan rather than silently changing its envelope.

No automatic retry, scene swap, manual reset, tuning or simultaneous flight owner.
On takeoff envelope violation, the task must log abort_hold/abort_land disposition
and request immediate LAND. Request acceptance is not a physical stop. A sent/
uncertain handover must never re-enter TRACK or publish a recovery trajectory
before grounding. Use observed PX4 mode/velocity/altitude to assess that handover.
Autopilot LAND uses its own descent profile, not the trajectory goal speed.
If actuation/outcome is unknown, preserve the failure and use only the existing
authorized recovery/stop path; do not issue more ordinary motion or midair disarm.
Do not delete/cancel the outer workflow as a substitute for confirmed grounding.

## Classification and exit

Successful qualification requires independent takeoff target/envelope evidence,
grounding/disarm after LAND, no contradictory state and complete retained streams.
Action-server success/readiness alone is insufficient. A recovered failed takeoff
is RECOVERED_HALT, not successful goal completion. A nominal takeoff/landing pass
does **not** exercise or qualify the abort branch; targeted abort testing needs a
separate reviewed plan after nominal behavior is understood. Do not induce a new
overspeed just to test recovery in this first attempt.

On any failure/unconfirmed result, stop ordinary flight, retain all records,
review timing/PID/trajectory/interface/physics separately and update
[handoff](../../HANDOFF.md)/[milestones](../../MILESTONES.md). Warehouse stays paused
regardless of this Office result. No general control, portability or TRL claim.
