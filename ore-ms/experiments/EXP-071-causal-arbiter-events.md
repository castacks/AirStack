# EXP-071: pre-refinement arbiter event recorder

2026-10-29 America/New_York. Base `d3500af07`, branch `ore_ms`.

An opt-in recorder now exposes pair health and raw camera motion result
writebacks before map refinement. Twenty focused tests pass. On a fresh
20-frame RGB GPU control, v2 and recorded v3 produce bitwise-identical values
in all 533 arrays across 23 NPZ files. The event stream passes an independent
timing/interval/health/increment audit. No gate is trained and no estimator,
pose-accuracy or fusion improvement is claimed.

## Problem and implementation

The [input audit](../docs/research/arbiter-input-audit-2026-10-28.md) found
useful saved tracking diagnostics, but trajectories were written after
interpolation/refinement. Using them as online arbiter inputs could introduce
future information and hide unsupported motion. Optimizer results are computed
and committed at different stages, and an old result can be written repeatedly.

New `Scripts/Setup/arbiter_event_recorder.py` observes existing instance methods:
`run_pair`, keypoint selection, observation filtering, optimizer submission,
`write_graph_data`, termination and refinement. Each original method is called
exactly once. The observer never polls or consumes `get_result`/`get_optimal`,
adds optimization work, changes poses/flags or modifies original tensors.
Copied observations are immediately serialized to immutable JSON strings in
memory and flushed at the end of the bounded run. GPU-to-CPU copies and
summaries add overhead; real-time latency is not established.

New `run_td3_execution_v3.py` preserves the v2 execution/config path and adds
`--record-arbiter-events`. Recording is disabled by default. The v2 runner,
uncertainty recorder, MACVO core, optimizer and map-refinement files are
unchanged; source hashes were checked against the starting commit. Historical
runners remain available. The v3 final report is persisted after cleanup so
partial runs can retain their event summary; estimator error/status semantics
are preserved rather than relabelled as successful tracking.

## Event contract

| Event | Content and causal meaning |
|---|---|
| `pair_begin` | Actual input-frame acquisition interval. |
| `optimization_submitted` | A distinct attempt ID and the optimizer's actual parent/target graph indices and timestamps. A subsequent exception is recorded explicitly. |
| `optimization_submit_returned` | Synchronous solve returned, asynchronous dispatch returned, or unspecified optimizer call returned. This is not an inferred convergence result. |
| `pair_end` | Selected/inbound/retained counts, retained 3×3 occupancy, fixed-grid raw temporal variance summaries, original support flag, submission state and any execution error. Skipped/unobserved fields remain null. |
| `motion_result_written` | Copied parent camera-map pose, native optimizer parameters before the normal write, committed camera-map pose after it, and a derived parent-inverse/target increment. Includes original support, submission ID and write ordinal. |
| `motion_result_write_error` / `optimization_error` | Original errors are recorded and still propagate to the caller. |
| `refinement_begin` / `refinement_end` | Delimit the hindsight-processing boundary; refinement errors remain explicit. |

Every event has a sequence number, modality, exact sensor timestamp fields
where applicable, and an integer `time.monotonic_ns` observation timestamp.
Monotonic observation time is a local clock and does not identify sensor latency
or cross-machine clock alignment. A motion event records **commit observation**,
not necessarily the earliest instant the optimizer internally had a result.
Synchronous completion and later commit are separately visible.

Repeated writes retain the same submission ID and increment its write ordinal;
only the first is marked `first_write_for_submission`. They must not be counted
as new motion measurements. The native optimizer parameterization may be local
to a graph frame and some writers transform it in place. It is labelled as
native parameters, never silently called a global pose. The committed map pose
is copied after the original writer completes.

Derived increments use `[tx,ty,tz,qx,qy,qz,qw]` and `parent^-1 * target`.
Quaternion copies are normalized only for that derived logged value; raw
copied poses and estimator tensors remain unchanged. Invalid/nonunit poses
produce an unavailable increment with a reason; nonfinite raw entries are null
in JSON, not a fabricated identity estimate. No body transform, calibrated
pose covariance or optimizer convergence is invented.

Pair health is available after processing that pair. Whole-clip summaries,
refined poses and future pair records are not earlier-frame features. Selection
counts denote selected keypoints, not the full dense candidate population.
Uncertainty summaries are raw temporal flow covariance, not pose covariance.

Observation exceptions are collected as `OBSERVER_ERRORS` without replacing
estimator results or swallowing estimator exceptions. A save failure is recorded
as `OBSERVER_SAVE_FAILED` in the v3 report. A future consumer must reject an
incomplete/error-marked journal; missing measurements do not become zero error
or successful tracking. Wrapper methods are restored on `finish`, including
when a subsequent file save fails.

## Validation

| Check | Result | Scope |
|---|---|---|
| Focused CPU tests | 20 pass | Delayed/stale/terminal writes; unsupported/skipped/error cases; observer failure isolation; immutable copies; integer timestamps; keyword calls; missing/invalid poses; existing v2 and new v3 failure reporting. |
| Native CPU optimizer control | Bitwise equal optimizer results and map arrays with/without observation | Actual two-frame ICP optimizer with 30 synthetic geometric observations, no learned frontend. CUDA timing-stream access is bypassed for CPU profiling only; solve is unchanged. |
| Fresh GPU v2/v3 control | 533/533 arrays equal by dtype, shape and logical bytes, across 23 NPZ files | Same restored runtime, original RGB checkpoint/config/input rows, 20 existing early-night frames, raw uncertainty recording enabled in both. |
| Independent event audit | PASS | All 97 events; 19 pairs and 19 first motion writebacks, zero repeated writes in this supported real control. Interval identity, ordering, original flags, health against saved pair arrays, and increments via independent PyPose composition. |
| Preservation | PASS | Original core sources, checkpoint and 43 restored RGB/metadata files unchanged after both runs. |

The real control's last motion write occurs at termination before refinement.
Synthetic/state tests cover the stale-write, unsupported and failure paths that
this supported RGB window does not exercise. Both tests and live control are
needed; neither substitutes for a broader estimator evaluation.

The existing EXP031 archive and all 1087 member hashes were verified; only the
43 early-window RGB/metadata files were restored. The original RGB release is
77369368 bytes with SHA256
`bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a`.
No thermal checkpoint or reserved MS² recording was acquired for this control.

The fresh GPU image is
`sha256:700f33b2f919364dfd8aefe486568ff69a7832cc5942149f03eef5b561230239`.
Runtime/package inventory is preserved. This is fresh same-runtime observer
parity, not a claim of bitwise identity to the historical EXP033 GPU image.
The CPU test image is
`sha256:3d13d1c97ade9d6bf95dd1aedf79a86554ce4a0e981dcec5dd26e7d8aacfb7dc`.

## Usage and artifacts

For a new bounded instrumented run, use v3 with an existing verified dataset,
original manifest and applicable checkpoint. With ore-ms mounted at `/workspace`:

```bash
python -m Scripts.Setup.run_td3_execution_v3 \
  --dataset /workspace/results/new-night-validation/prepared-001/datasets/early \
  --manifest /workspace/results/new-night-validation/prepared-001/datasets/early/manifest.json \
  --output /workspace/results/arbiter-events/<fresh-run> \
  --modality rgb --frames 20 --record-uncertainty \
  --record-arbiter-events --acknowledge-provisional-geometry
```

`<fresh-run>/rgb/arbiter-events/` contains `events.jsonl` and `summary.json`.
The numerical outputs retain the v2 layout. Construction refuses an existing
event output folder. This implementation is for bounded research recording,
not an unbounded production flight logger.

Tests run in `macvo:event-tests`, built from the existing `Dockerfile.replay`
image plus `Docker/Dockerfile.event-tests`:

```bash
python -m pytest Scripts/UnitTest/test_arbiter_event_recorder.py \
  Scripts/UnitTest/test_td3_execution_v3.py \
  Scripts/UnitTest/test_td3_execution_v2.py \
  Scripts/UnitTest/test_td3_tracking_report.py -q
```

The independent live validator is
`Scripts/Setup/validate_arbiter_event_replay.py`, taking `--baseline`,
`--observed` and `--output`. Validation artifacts are under
`results/arbiter-event-recorder/`: pre-run `gpu-plan.json` with exact Docker
commands/input/source hashes, restoration evidence, separate GPU outputs/logs,
event journal, `gpu-validation.json` and runtime inventory. Recovery target:
`backups/2026-10-29-exp-071`. Source dependencies are the EXP031 early inputs
and the original released RGB checkpoint; credentials/checkpoints/full source
archives are excluded from the new recovery package.

## Decision

The recorder is ready for bounded research use. Its numerical-preservation and
causal-stage tests pass, without changing an estimator default. It does not yet
provide a common-body pose target, an asynchronous arbitration policy, a pose
accuracy reference or a trained gate. The next bounded step is to capture a
paired RGB/thermal interval with this same event contract, verify alignment and
field availability, and specify the comparison target before fitting a gate.
Keep unsupported/abstaining cases explicit and preserve the reserved MS² data.
