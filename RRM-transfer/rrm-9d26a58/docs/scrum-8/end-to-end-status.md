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
258/258 tests, but unit and fake-adapter coverage is not a physical-task benchmark.

| Segment | What has been demonstrated | Current limitation |
| --- | --- | --- |
| Goal -> GUI | Immutable goal/attempt storage, deterministic parsing of a narrow aerial movement grammar, clarification for selected ambiguous forms | The new qualitative `GoalRequest` and capability router are not wired into the GUI; arbitrary language and deictic visual grounding are not general |
| GUI -> RRM | Typed plans, discovered-server checks, numeric provenance, predeclared recovery, evidence display | The direct console path is a deterministic aerial adapter, not the learned cross-embodiment reasoner |
| RRM -> actuation | Public AirStack task actions only; one documented bounded takeoff/land mission independently verified both actions | No direct PX4/trajectory authority is intended; aerial execution is currently paused after the later failed regression |
| Actuation -> replan | Fresh state is reacquired between actions and produces `CONTINUE`, `SKIP_SATISFIED`, or `HALT`; AirStack exploration/navigation planners can publish route updates | RRM currently performs inter-action reconciliation, not general semantic replanning from arbitrary observed divergence |
| Replan -> finish | Terminal outcomes distinguish `VERIFIED`, `HALTED`, `RECOVERED_HALT`, and `RECOVERY_FAILED`; failed takeoff verification has triggered a separately verified recovery landing | A learned Office navigation run timed out without reaching its target; no repeated nominal/off-nominal acceptance matrix has passed |
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
  The cause remains unresolved, so aerial flight is paused.
- The hand Gate 4/5 fixture exercises only an authenticated, bounded joint calibration
  action. It explicitly does not authorize semantic `GRASP` or `PLACE` execution.

The historical runtime artifacts referenced by the handoff are gitignored and are not
present in this workspace. Therefore the repository currently cannot recompute an
aggregate live success rate from raw trials; only the recorded run summaries and
hermetic tests are available.

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
