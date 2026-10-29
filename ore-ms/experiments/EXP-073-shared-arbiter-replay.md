# EXP-073: shared chronological replay and advisory availability

2026-10-29 America/New_York. Branch `ore_ms`, base `f23313025`.

The first causal advisory replay is working on the previously examined 20-group
early-night window. A controller releases 40 frames in sensor-time order to two
independent estimator processes, records their events on a shared host monotonic
timeline, and emits 19 decisions from the observed event prefixes. Eighteen
boundaries have both experts eligible; the fixed preference selects RGB. The
last boundary abstains because its motion is not committed until shutdown.
Both expert outputs preserve all 533 arrays across 23 NPZ files bitwise against
the existing standalone controls. This establishes scheduling and availability
behavior, without measuring VO improvement or real-time capability.

## Scheduling and deadline contract

`Scripts/Setup/shared_arbiter_replay.py` spawns one worker per expert. Each calls
the unchanged v3 runner with independent process-local random state. A worker-local
sequence subclass waits for controller permission before yielding each frame.
A worker-local event-recorder subclass streams the original immutable copied
events to the controller. No optimizer result is additionally polled, no estimator
output is replaced, and advisory decisions do not feed back into either estimator.

Initialization, including the original first-depth probe, precedes the first
release. The controller then serializes frame processing in exact integer left
acquisition timestamp order: thermal then RGB within each group in this window.
Completion is acknowledged after the original run call and CUDA synchronization.
Both workers remain alive, but only one processes a released frame at a time.
This is **offline serial replay**, not concurrent or sensor-rate execution.
Sensor timestamps establish logical release order; they are never subtracted
from monotonic host observation timestamps. The approximately 8 ms camera offsets
remain unmodified. The policy rejects groups whose acquisition times overlap
in a way that violates its declared group-boundary deadline contract.

Pair ordinal i spans groups i to i+1 for each expert's own camera timestamps.
Its ordinary decision occurs after both group i+2 frame calls complete, allowing
one group of observation lag. MACVO normally commits the previous pair's motion
while processing the next frame, so this lag exposes its normal writeback without
changing optimizer behavior. The final pair has no next input group: evaluate
it once after the final frame calls, **before optimizer drain**. Termination
results remain in the journal but cannot change that recorded abstention.
The final boundary is explicitly labeled separately from ordinary boundaries.
These are event boundaries, not millisecond latency budgets.

Each worker is gated again before returning from iteration to termination.
The coordinator releases drains only after all decisions are recorded. Failures
and receive timeouts create controller-error records, retain partial journals
and a failed summary, and clean up child processes. Worker logs remain separate.
The replay is opt-in; the original v2/v3 entry points and estimator defaults
are unchanged.

## Advisory rule

An expert is eligible when its observed prefix contains complete pair health,
original supported tracking, no execution error, and a first camera-local motion
write matching that expert's exact input interval. Missing health, partial health,
missing first writes, unsupported tracking, ambiguous or mismatched intervals
remain explicit evidence. Repeated writes never count as new first results.

| Available experts | Advice | Interpretation |
|---|---|---|
| RGB and thermal | RGB | Fixed baseline preference, not a pose-quality ranking |
| RGB only | RGB | Only eligible result |
| Thermal only | THERMAL | Only eligible result |
| Neither | ABSTAIN | No eligible result at this boundary |

Counts, coverage and covariance summaries are copied into the decision evidence.
They are not cross-camera quality scores, calibrated probabilities, or fitted
coefficients. Each decision records exact camera intervals, source event sequence
numbers, both observed-prefix lengths and its controller monotonic timestamp.
No common-body pose is produced, and `pose_gate_training_ready=false` remains.
This advisory baseline provides the deterministic scheduling/control path for a
later uncertainty/history gate; it does not implement a learned selector.

## Actual replay and verification

| Check | Result |
|---|---|
| Original early-night inputs | Same 20 manifest groups; no reserved recordings |
| Scheduled frame calls | 40, exact acquisition-time order |
| Streamed expert events | RGB 97; thermal 97; identical to saved journals |
| Ordinary decisions | 18 RGB, both experts eligible |
| Final pre-drain decision | 1 ABSTAIN, neither current motion committed |
| Original tracking support | 19 supported pairs per expert after full capture |
| RGB versus EXP071 v2 control | 533 arrays / 23 NPZ files bitwise equal |
| Thermal versus EXP072 v2 control | 533 arrays / 23 NPZ files bitwise equal |
| CPU policy and paired-contract tests | 29 passed, including 14 new policy cases |
| Independent timeline audit | PASS, 798 decision-field checks |
| Semantic audit challenges | All 5 rejected |
| Worker startup error and controller timeout | Failed summaries/journals saved; child cleanup completes |

The existing independent event validator also checks original reports/config,
raw uncertainty health, submission/write ordering, first-result pose composition,
final termination write, and zero observer errors for each expert. Frozen input,
checkpoint and original estimator source hashes remain unchanged. The same
Blackwell image used for the standalone controls is pinned by ID:
`sha256:700f33b2f919364dfd8aefe486568ff69a7832cc5942149f03eef5b561230239`.
The scheduler source is frozen before inference; the post-run independent auditor
records its own source digest.

`validate_shared_arbiter_replay.py` imports neither scheduler nor advisory code.
It reconstructs acquisition order, completion boundaries, available health/motion
from each live prefix, all decision evidence and actions, and exact correspondence
with saved journals. It checks frozen sources/inputs/checkpoints and standalone
preservation proofs. Five mutations are rejected after recomputing the timeline
hash: changed action, future-prefix length, shifted sensor release time, early
deadline and retroactively replacing the final abstention with RGB. These are
software checks, not independent scientific trials.

Authoritative artifacts are under `results/shared-arbiter-events/`: frozen
`capture-plan.json`, capture/worker logs, test JUnit, `run-001/{protocol.json,
timeline.jsonl,summary.json,validation.json,experts/}`, both preservation proofs,
frozen-input validation and audit challenges. Timeline SHA256:
`5b94ee7b594038a236d67e8ee43413d817586eaab44813f925b05c9f6240bc9b`.
Recovery destination: `backups/2026-10-29-exp-073`, dependent on EXP071/072/031.
Credentials and model checkpoints are excluded from recovery artifacts.

From the MAC-VO root in the pinned Docker runtime, with a fresh output directory:

```bash
python -m Scripts.Setup.shared_arbiter_replay \
  --dataset ../results/new-night-validation/prepared-001/datasets/early \
  --manifest ../results/new-night-validation/prepared-001/datasets/early/manifest.json \
  --frames 20 --acknowledge-provisional-geometry \
  --output <fresh-output>
python -m pytest Scripts/UnitTest/test_shared_arbiter_replay.py \
  Scripts/UnitTest/test_pair_arbiter_events.py -q
```

Exact Docker commands, mounts, image and file hashes for the actual run are in
the frozen capture plan. Run the existing `validate_arbiter_event_replay` separately
for each expert against its v2 control. The bounded independent shared audit
expects this experiment's 20-group early window and adjacent capture/preservation
proofs; it deliberately does not silently validate a different dataset/window.
For the saved capture, the auditor also runs with standard Python on the host:

```bash
python3 Scripts/Setup/validate_shared_arbiter_replay.py \
  --root /root/AirStack/ore-ms \
  --run /root/AirStack/ore-ms/results/shared-arbiter-events/run-001
```

## Next step

Exercise this advisory path on a predeclared segment of an already examined
support-collapse window, with explicit missing/unsupported decisions and fixed
comparison baselines. The present supported early window tests the scheduling
interface; it cannot demonstrate a thermal fallback benefit. Before fitting a
pose-quality gate, still establish verified common-body/interval transport and
an applicable independent reference/loss. No VO gain, physical calibration or
real-time throughput claim follows from EXP073.
