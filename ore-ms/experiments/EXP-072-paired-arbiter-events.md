# EXP-072: paired RGB and thermal event inputs

2026-10-29 America/New_York. Base `8ccb26312`, branch `ore_ms`.

The existing early-night window now has verified RGB and thermal event
captures and a machine-readable paired contract. There are 19 paired intervals
from the same 20 manifest groups. Both experts have complete pair-health
records and camera-local motion results. Thermal v2/v3 replay preserves all
533 arrays across 23 NPZ files bitwise; the already verified RGB capture is
reused without repeating its inference. No gate is fitted or compared.

## Fixed scope and recovery

Use the EXP031 `prepared-001/datasets/early` window, manifest start0,20 stereo
frames per expert. These are previously examined inputs, not new scenes or
held-out data. Restore 43 thermal image/metadata files against the existing
EXP031 archive/member hashes. The RGB input rows remain identical to EXP071.

Recover only `macslam/MACT_Offroad.pth` from the authorized Box checkpoint ZIP
(file2037543542268). ZIP-member CRC and exact SHA256 pass:
`eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a`,
76827690 bytes. Bounded ranges transferred71199939 bytes; the full318804129-byte
archive was not downloaded or newly hash-verified. Credentials and transient
signed download URLs are not retained in scientific artifacts.

Before new inference, freeze checkpoint/input/source hashes, exact commands
and the same GPU image as EXP071:
`sha256:700f33b2f919364dfd8aefe486568ff69a7832cc5942149f03eef5b561230239`.
Use unchanged v2 and opt-in v3 with raw uncertainty recording in both. The two
thermal runs are sequential and differ only in event recording. Their outputs
are a preservation control, not two independent evaluations.

Existing RGB output, journal and validation/plan files match all30 applicable
member hashes in the verified EXP071 recovery manifest. No RGB inference is
rerun. Source, inputs and the thermal checkpoint remain unchanged afterwards.

## Capture and preservation results

| Expert | Stereo frames | Pair records / first motion writes | Observer errors | Preservation evidence |
|---|---:|---:|---:|---|
| RGB |20|19 /19|0|EXP071:533 equal arrays/23NPZ;prior output/proof rechecked by recovery hashes |
| Thermal |20|19 /19|0|Fresh v2/v3:533 equal arrays/23NPZ;independent97-event audit PASS |

The existing event validator checks numerical dtype/shape/bytes, original
reports/config, event ordering and original flags, health against raw saved
uncertainty arrays, and increments by independent PyPose composition. The
last thermal result is committed during termination before refinement, matching
the source's normal writeback behavior. No extra optimizer result is polled.

Both experts have zero original unsupported pairs in this early window. RGB
retained matches min/median/max are199/200/200; thermal142/197/200. Counts do
not establish which pose is more accurate. This supported window verifies
the paired interface; it cannot demonstrate a switching or abstention benefit.
The known support-collapse examples in other saved windows remain relevant,
but are not silently added to this capture.

## Interval alignment

Join on the **identical original manifest rows**, retaining both cameras'
actual acquisition intervals. No timestamp is shifted, rounded to another
camera or resampled. All fields remain exact integer nanoseconds.

| RGB minus thermal endpoint offset | Minimum | Median | Maximum |
|---|---:|---:|---:|
| Interval start |8.014917ms|8.045912ms|8.046151ms|
| Interval end |8.014917ms|8.045912ms|8.046151ms|

None of the19 intervals has equal RGB and thermal endpoints. These are
observed acquisition offsets, not calibrated exposure/latency corrections.
Equal ordinal indices identify a grouping, not identical motion intervals.

The RGB and thermal captures ran in **separate replay processes**. Their
monotonic observation timestamps are retained as provenance, but their
differences do not represent concurrent result availability or a real-time
arbitration latency. Sensor epoch timestamps must not be subtracted from
monotonic observation clocks. A subsequent shared-timeline replay needs a
declared scheduling/deadline policy.

## Paired input contract

`pair_arbiter_events.py` creates `paired-events.json`, with one row per expected
manifest transition. Each row contains:

| Field | Meaning |
|---|---|
| `pair_ordinal` | Original manifest transition ordinal, without relabelling it as simultaneity. |
| `cameras.<expert>.sensor_interval_ns` | That expert's actual start/end acquisition times. |
| `health` | Explicit allowlist: selected/inbound/retained counts,3×3coverage,grid temporal covariance summaries,original support,execution error and submission state. |
| `health_status` | Complete,partial or missing pair record. Missing rows remain in the table. |
| `motion` | First normal result commit for that expert/target, with original interval,graph indices,stage,raw camera-map poses and derived camera-local increment. |
| `motion_status` | Matching interval,no first write,interval mismatch or ambiguous first writes. No arbitrary nearest-time selection. |
| `motion_available_camera_local` | A matching non-unsupported valid camera-local result; not a pose-accuracy label. |
| `rgb_minus_thermal_endpoint_ns` | Exact sensor-time offsets for this grouping. |
| `body_transform_verified` | False; the adapter's camera-local identity transform is not an independently verified rig/body extrinsic. |

Repeated writes are counted separately and never become new measurements.
An optimizer's graph interval can differ from the actual input pair after
skipped keyframes; this remains an explicit mismatch instead of being hidden.
Malformed/duplicate first writes, duplicate pair endings, noninteger times,
backward observation clocks, inconsistent counts, wrong modalities or differing
source rows are rejected. Error-marked or hash-mismatched journals are rejected.

Whole-run status and outcomes are not predictor features. A later clip failure
must not retrospectively change an earlier pair's available inputs. An explicit
allowlist prevents including refined poses, whole-clip failure counts or future
records as health inputs.

The contract marks `health_schema_replay_complete=true` for these captures.
It deliberately retains `pose_gate_training_ready=false`: common-body transport,
an asynchronous comparison policy and applicable independent pose-loss labels
are not established, and separate replay clocks do not simulate concurrent
availability. Even exactly equal endpoints would not establish those other
requirements. No camera poses are blended or compared as if they shared a
verified frame/reference.

## Validation and reproduction

Fifteen contract tests pass, including exact large integer timestamps,
unsupported/missing rows, stale writes, graph-interval mismatch, tampered
journals and invariance of past inputs to final-run outcome changes.
Independent source-manifest/journal reconstruction passes1387 field checks,
recomputes all19 pairings and offset/availability summaries, rechecks existing
RGB recovery hashes and frozen thermal sources/runtime/inputs. Check counts
are implementation checks, not independent scientific samples.

The pure builder is `Scripts/Setup/pair_arbiter_events.py`;
`validate_paired_arbiter_events.py` imports no builder code and checks the real
capture against the original manifest. Tests are
`Scripts/UnitTest/test_pair_arbiter_events.py`. From the MAC-VO root:

```bash
python -m Scripts.Setup.pair_arbiter_events \
  --rgb ../results/arbiter-event-recorder/gpu-v3/rgb \
  --thermal ../results/paired-arbiter-events/thermal-v3/thermal \
  --rgb-proof ../results/arbiter-event-recorder/gpu-validation.json \
  --thermal-proof ../results/paired-arbiter-events/thermal-validation.json \
  --output <fresh-paired-output>
python -m Scripts.Setup.validate_paired_arbiter_events \
  --root /root/AirStack/ore-ms --run <fresh-paired-output> \
  --manifest ../results/new-night-validation/prepared-001/datasets/early/manifest.json
```

Use absolute source paths to keep the saved protocol independent of the working directory; the validator records
the paths as supplied. Authoritative artifacts are under
`results/paired-arbiter-events/`: checkpoint/input recovery evidence,
pre-inference thermal plan/commands/hashes,thermal v2/v3 outputs/logs/validation,
RGB reuse proof,`paired-001/{protocol,paired-events,validation}.json`, and
JUnit/test output. The RGB outputs remain in the verified EXP071 artifact.
Recovery target:`backups/2026-10-29-exp-072`; dependencies EXP071 and EXP031.
Credentials,checkpoints,full source archives and unrelated data are excluded.

## Next bounded implementation

The paired schema is ready for health-input replay work. Next establish a
single chronological dual-expert replay with an explicit causal scheduling
policy, so input arrivals and result availability share an observed timeline.
Start with an advisory availability rule and retain missing/unsupported actions;
do not claim pose or VO improvement from this supported-window interface check.
A pose-quality gate additionally needs common-frame interval transport and a
declared reference/loss before fitting. No new CATS scene,MS² holdout,estimator
default or gate coefficient was used or changed here.

Follow-up: [EXP073](EXP-073-shared-arbiter-replay.md) implements the shared
offline chronological replay and advisory availability rule, preserving both
experts bitwise and recording the final pre-drain abstention causally.
