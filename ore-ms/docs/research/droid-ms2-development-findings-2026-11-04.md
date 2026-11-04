# EXP136: DROID metric stereo contract and bounded development limits

2026-11-04, America/New_York. Parent EXP135 / `90f772ec2`.
Initial native freeze `37fc4bad1`; capacity/lifecycle amendment `7dd04bef8`.

The DROID stereo adapter, official checkpoint and Blackwell CUDA runtime are built.
All 72 adapter/regression tests, 23 CPU/CUDA metric-geometry checks and 1,818
independent checks/30 mutation rejections pass for the amended engineering run.
All six cases submit all 20 original source groups. **Only day thermal initializes;
the other five cases remain below the fixed eight-keyframe warmup and retain null
admitted poses.** This is a bounded adapter/initialization result, not a completed
accuracy benchmark or evidence that thermal generally outperforms RGB.

## What this establishes

The research objective remains whether RGB–thermal selection can improve motion
estimation against fixed cameras and simple controls. That benefit is not yet
established. This work establishes the prerequisites for a credible comparison:
the provided metric scale, transform direction, source timestamps, actual model
inputs and explicit initialization/failure behavior of a published stereo baseline.
Longer recording-level development runs and common-target-time qualification are
still needed before comparing complete trajectories.

## Pinned build and checkpoint

[Official DROID source](https://github.com/princeton-vl/DROID-SLAM/tree/2dfd39f0dcad44012ca7bbb8aa70b55edbfa9c99)
is pinned to `2dfd39f0dcad44012ca7bbb8aa70b55edbfa9c99`, with lietorch
`7f687644fcea81ab337831749445224503c1290d`, Eigen
`824272cde8ca2541e8b67b0887f5ded92b128d1f` and Torch Scatter
`6cf77c420f837a427b0d57965e414d68c1bd89ec`. The isolated runtime uses the
digest-pinned PyTorch 2.8.0/CUDA 12.8 base and targets the available SM120 GPU.

Two compatibility changes are explicit: replace lietorch's fixed old GPU build
targets with the declared SM120 build target, and allocate the Python stereo
baseline tensor on the input tensor's device so CPU geometry tests are possible.
The latter leaves CUDA values unchanged. No estimator equation, learned weight,
stereo baseline constant or motion threshold is changed. All extensions compile
and the CPU/CUDA controls pass on the first build.

The [official model download pointer](https://github.com/princeton-vl/DROID-SLAM/blob/2dfd39f0dcad44012ca7bbb8aa70b55edbfa9c99/tools/download_model.sh)
supplies a 16,061,701-byte checkpoint with SHA256
`46476ef64cde45a97504910d6f3de2eef7b398ec1c6e4e668815c29076024526`.
All 102 state tensors/4,004,871 elements are finite before inference. The original
model-loading/head-slicing code is retained. No training or checkpoint modification
occurs; independent checkpoint-training membership remains unverified.

## Metric scale and output direction

Both pinned Python projective geometry and the CUDA stereo bundle-adjustment path
use an internal baseline `0.1`. The native reconstruction therefore receives this
calibration-only unit conversion:

```text
metric_translation = native_translation × provided_baseline_m / 0.1
```

The factors are `2.9918420530585865` for supplied RGB and `3.038995142414822` for
thermal. Rotation is unchanged. The EuRoC example's `1.10` multiplier and
reference-fitted scale are not used. The supplied stereo calibration also has
equal left/right intrinsics, identity relative rotation and a purely horizontal
baseline for all six source stereo pairs; a post-run metadata-only qualification
records these checks without changing inputs or outputs.

CPU and CUDA analytic stereo projections agree with `fx*baseline/Z` for both
provided baselines. The actual CUDA optimizer's stereo self-edge matches the
independent one-step inverse-depth solution, with zero observed float32 difference
in the declared fixture. Known nonidentity poses verify inverse direction and
baseline scaling. These 23 checks use no images, weights or actual pose references.
They validate the provided numeric model, not surveyed physical calibration.

`Droid.terminate()` already returns inverse camera poses. The adapter uses that
camera-to-world convention without another inversion, checks xyzw quaternion
validity and matrix rigidity, and scales only translation. It retains one output
pose or null for every original source timestamp.

## Source times, images and first failed smoke

All inputs reuse EXP134's canonical PNGs from the exposed first 20 stems of the
same day/night/rain recordings. Thermal preprocessing is not recomputed or changed.
Original left/right nanosecond timestamps and source IDs remain exact in sidecars.
DROID internally uses float32 times in fixed 100ms units relative to the first
left timestamp; integer subtraction precedes conversion, quantization is recorded,
and collisions are refused. No epoch float, nearest-timestamp pairing, clock fit
or common RGB/thermal acquisition-clock assertion is introduced.

The initial full-resolution run is preserved:

| Case | Returned source calls /20 | Initial result |
|---|---:|---|
| Night RGB | 20 | One keyframe; uninitialized backend hits an invalid CUDA configuration. |
| Night thermal | 20 | Same one-keyframe initialization limit. |
| Day RGB | 13 | Native correlation volume exceeds 32-bit packed-accessor capacity. |
| Day thermal | 20 | Initialized; 20 finite admitted poses. |
| Rain RGB | 16 | Same 32-bit correlation-capacity failure. |
| Rain thermal | 20 | Native filler returns outputs but frontend remains uninitialized; admitted poses stay null. |

The first independent audit passes 1,825 checks/30 mutations, including the rejected
finite-but-uninitialized filler case. These results are never overwritten or
presented as successful tracking.

## Capacity/lifecycle amendment

The separate amendment precedes new runs. It caps model input area at the pinned
native demo's `384*512` target, without upsampling smaller images, then applies the
native right/bottom crop to multiples of eight. RGB changes from `384×1224` to a
`248×791` resize and `248×784` crop; thermal remains `256×640`. Intrinsics scale
by the **pre-crop** width/height ratios; left/top crop origin stays zero. There is
no re-rectification, temporal subsampling or source-frame exclusion. Source PNGs
are unchanged and every tracking/filler input tensor and intrinsic vector is hashed.
The rule is based on native index capacity, not reference errors or chosen good
results. It also changes the model's pixel motion; it cannot support a same-input
accuracy comparison against the failed full-resolution RGB attempt.

A frontend that has not initialized is stopped explicitly before its offline
backend. No warmup/filter threshold is lowered, no longer window is selected on
quality, and no filled uninitialized output is admitted. Native demo defaults,
seed 0 and the same settings apply to all six cases. The only smaller buffer is
an explicit 64-slot bounded-smoke allocation, sufficient for the 20-frame input
plus native 16-frame filler batches.

| Amended case | Source calls /20 | Final keyframes | Initialized | Admitted poses /20 |
|---|---:|---:|---|---:|
| Night RGB | 20 | 1 | no | 0 |
| Night thermal | 20 | 1 | no | 0 |
| Day RGB | 20 | 7 | no | 0 |
| Day thermal | 20 | 8 | yes | 20 |
| Rain RGB | 20 | 2 | no | 0 |
| Rain thermal | 20 | 6 | no | 0 |

All admitted day-thermal poses are finite native offline outputs. DROID's backend
and trajectory filler use future images and interpolated initialization followed
by image-based refinement. The adapter labels that scope, exposes initialization
and does not invent per-frame tracking-failure status or online pose latency.
Finite output is not proof of physical pose accuracy. The five uninitialized
cases remain terminal failures with their original 20 null output slots.

## Audit, recovery and next experiment

The amended independent audit passes 1,818 checks and rejects 30 mutations:
wrong scale, float source clocks, invented online latency, hidden scale fitting and
false pose admission/alteration. It independently checks xyzw-to-matrix geometry,
calibration conversion, source clocks/IDs, all nulls, tensor hashes, resized/cropped
images, intrinsic scaling, keyframe lineage and initialization. All 276 frozen
source/input pins remain unchanged. The 72 tests include 20 new DROID adapter
cases and 52 prior ORB/publisher/scorer cases. Both metric preflights pass 23 checks.

Exact runtime:
`sha256:d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c`.
It is saved as ordered gzip archive parts, with complete and per-part SHA256 hashes.
The first and amended source freezes, actual inputs, all failures and independent
audits are retained. No actual reference pose/depth input, scale fit, reserve access,
training, physical-accuracy score or production default change occurs.

The next useful development study is a predeclared longer chronological prefix
or full recording on the same exposed conditions, sufficient to observe native
initialization without tuning its threshold. Common-target-time adaptation and
the remaining full-recording contracts precede any mixed-camera/heldout comparison.
EXP133 remains execution-not-ready with active validation/test lists empty.

Artifacts: `results/droid-ms2-development/{source-001,preflight-001,preflight-002,run-001,run-002}`.
Final manifest: `experiments/droid-ms2-development-2026-11-04-v2.json`.
Recovery: `backups/2026-11-04-droid-ms2-development-exp136`.
