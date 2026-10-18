# EXP-003: Bounded camera-local RGB and thermal VO

Date: October 18, 2026 (America/New_York). Branch `ore_ms`, base commit
`af553955e844d42abb164ce5218e6be06cf19c8c` with the accompanying uncommitted
manifest adapter/runner and checkpoint compatibility implementation.

## Outcome and limits

Both modalities execute all 10 frames through stereo depth, temporal matching,
3D covariance propagation, visual constraints, and synchronous CPU pose
optimization. All trajectories are finite with normalized quaternions and
exact original acquisition timestamps. No frames use interpolation fallback.
This is a camera-local execution experiment, not accuracy, robustness, relative
reliability, calibration, or MACThermal training-preprocessing reproduction.

| Modality | Frames | Retained matches | Interpolated frames | Finite retained 3D covariances | Minimum covariance eigenvalue |
|---|---:|---:|---:|---:|---:|
| RGB | 10 | 1800 | 0 | 100% | 2.36626e-07 |
| THERMAL | 10 | 1470 | 0 | 100% | 7.59253e-07 |

The covariance values above describe retained observations and are not a
cross-modal reliability ranking. No geometric error reference was used.
15 focused regression tests pass across checkpoint migration, pairing,
calibration comparison, and the new input adapter.

## Inputs and declared geometry

Dataset and archive provenance are in EXP-001. Manifest from EXP-002 uses
explicit 2 ms stereo and 30 ms cross-modal acquisition bounds. Frames 0–9
are the selected source indices for each of the four streams. Each modality
uses its own acquisition times, not synthetic index timestamps.

RGB input: native 544x1024, left uint8 color and right uint8 grayscale replicated
into three channels. Thermal input: native 512x640, provided processed uint8
three-channel images. Both use BGR→RGB and float32 /255. Raw uint16 thermal is
preserved and rejected by this execution adapter.

Rectified intrinsics come from the projection matrices P, not raw K. Positive
execution baseline is the absolute projection baseline: RGB approximately
0.209692 m, thermal approximately 0.335113 m. The original signed thermal
projection baseline is negative. This declared magnitude convention matches
the existing frontend's absolute disparity conversion; it does not resolve
signed calibration or certify metric geometry.

`T_BS` is identity: reported trajectories are in each rectified left camera's
NED coordinates. No vehicle/world extrinsics, cross-modal pose alignment,
reference poses, trajectory errors, or ground truth are inferred.

## Effective system and validation

Based on `MACVO_Performant.yaml`, with regular `FlowFormerCovFrontend` in fp32,
mapping disabled, synchronous CPU `TwoFrame_PGO`, static motion initialization,
and the original covariance-aware keypoint selection and MatchCovariance model.
No CUDA graph capture, pose-net initialization, or dataset resizing is used.
Seed = 0. Checkpoints are the original RGB and MACT bundle files, loaded through
the audited strict active-state loader. Complete frontend checkpoints avoid
unrelated pretrained encoder downloads.

Runtime remains `macvo:blackwell` on RTX PRO 5000 Blackwell, Torch 2.7.1+cu128.
Optional torch.compile helpers fall back to eager math because the runtime lacks
a C compiler; timings are not throughput benchmarks.

The runner fails on incomplete/nonfinite trajectories, non-unit quaternions,
timestamp mismatch, or no retained matches. It checks first-frame dense positive
finite depth/covariance (100% for both runs) and records retained 3D covariance
finite fraction and eigenvalues. The first two exploratory runs are in
`results/bounded-vo/run-001/`; the final run with graph/covariance exports is
`results/bounded-vo/run-002/`.

## Reproduction and saved outputs

From `MAC-VO-main/` inside the existing GPU runtime, repeat separately for
`rgb` and `thermal`, choosing a fresh output directory:

```bash
python Scripts/Setup/run_td3_execution.py \
  --dataset ../data/tartandrive3/20251027/1_salon_mac_cov_day \
  --manifest ../results/data-access/paired-manifest.json \
  --output ../results/bounded-vo/new-run \
  --modality rgb --frames 10 --acknowledge-provisional-geometry
python -m pytest Scripts/UnitTest/test_research_setup.py \
  Scripts/UnitTest/test_td3_sequence.py -q
```

Each modality directory retains `effective-config.json`, `report.json` with source
pairs/checkpoint hash/baselines, `first-depth.npz`, `tensor-map.npz` with visual
observations/covariances, and `trajectory.npz` with `poses_camera_ned` and int64
`time_ns`. Run metadata records source hashes and runtime image ID. Raw data,
calibration files, checkpoint bytes, and source archive remain unchanged.

## Next research step

Use these exported observations to define a reference-supported geometric error
measurement, reconcile signed calibration and reference-frame conventions,
and validate a moving segment before measuring covariance calibration or
relative RGB/thermal reliability. A short successful execution is insufficient
for those claims.
