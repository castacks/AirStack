# Goal-to-finish performance status

This page distinguishes a connected software path from demonstrated autonomous task
performance. The target loop is:

```text
goal -> GUI -> semantic RRM -> capability route -> embodiment grounding
     -> independent safety/admission -> actuation -> observation/replan
     -> independently verified goal or explicit terminal failure
```

## Current assessment — 2026-09-26

There is not yet enough repeated live evidence to report a meaningful end-to-end
success rate, latency distribution, or recovery rate. The dependency-light suite passes
271/271 tests, but unit and fake-adapter coverage is not a physical-task benchmark.

| Segment | What has been demonstrated | Current limitation |
| --- | --- | --- |
| Goal -> GUI | Immutable goal/attempt storage, deterministic parsing of a narrow aerial movement grammar, clarification for selected ambiguous forms, and one exact Kuka-Allegro placement goal exposed as a non-executing preview | The hand preview uses a synthetic fixture and a selected block field; arbitrary language and deictic visual grounding are not general |
| GUI -> RRM | Typed plans, discovered-server checks, numeric provenance, predeclared recovery, evidence display, and persisted `GoalRequest -> route -> C01 -> C04/C05` hand-preview records | The direct execution path remains a deterministic aerial adapter; the neutral hand path ends before numeric feasibility and C06 |
| RRM -> actuation | Public AirStack task actions only; one documented bounded Office takeoff/land mission independently verified both actions | No direct PX4/trajectory authority is intended; non-Office scenes are not flight-qualified, and Warehouse aerial execution is paused after the failed regression |
| Actuation -> replan | Fresh state is reacquired between actions and produces `CONTINUE`, `SKIP_SATISFIED`, or `HALT`; AirStack exploration/navigation planners can publish route updates | RRM currently performs inter-action reconciliation, not general semantic replanning from arbitrary observed divergence |
| Replan -> finish | Terminal outcomes distinguish `VERIFIED`, `HALTED`, `RECOVERED_HALT`, and `RECOVERY_FAILED`; failed takeoff verification has triggered a separately verified recovery landing; uncertain takeoff failures now retain supervision for delayed airborne evidence | The delayed-recovery monitor has deterministic regression coverage but has not yet been exercised in another authorized live failure; no repeated nominal/off-nominal acceptance matrix has passed |
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
