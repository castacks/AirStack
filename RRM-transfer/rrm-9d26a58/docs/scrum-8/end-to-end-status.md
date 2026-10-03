# Goal-to-finish performance status

This page distinguishes a connected software path from demonstrated autonomous task
performance. The target loop is:

```text
goal -> GUI -> semantic RRM -> capability route -> embodiment grounding
     -> independent safety/admission -> actuation -> observation/replan
     -> independently verified goal or explicit terminal failure
```

## Core evaluation checkpoint — 2026-10-02

The core CPU suite passes 410/410 tests. T now exports a fixed 17-case
[acceptance campaign](../core-acceptance.md) with authored event-scoped safety labels
and a [paired comparison report](../core-comparison.md) over matching, same-source
campaigns. Every scheduled attempt and unknown outcome remains in reporting.
Retained v2 runs exposed evidence-write deadline/replay gaps at the short mock
fixture bounds. Current v3 evidence reconstructs only receipt-proven, adjacent late
commits; pending/missing/reordered writes remain incomplete. Two retained v3 runs
each qualified 31/34 attempts and matched 33/34 expectations, preserving one additional
observed write-liveness failure plus the two intentional trace-loss cases. This advances
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
