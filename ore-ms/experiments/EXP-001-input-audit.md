# EXP-001: First TD3 input audit and thermal frontend execution

Date: 2026-10-17 (America/New_York). Branch: `ore_ms`.
Source commit: `af553955e844d42abb164ce5218e6be06cf19c8c`. No production algorithm changes.

## Question and scope

Does the first available processed TD3 clip contain usable RGB/thermal stereo
inputs, timestamps, calibration, and reference-data inputs? Can the released
thermal checkpoint execute on actual provided thermal imagery?

This is an input/execution audit, not an accuracy or covariance-calibration
experiment. Full VO has not been run.

## Input and environment

- Box source: `ore_ms:ore-ms/1_salon_mac_cov_day.zip`.
- Recording metadata confirms source `/media/airlab/offroad/rosbags/20251027/1_salon_mac_cov_day`.
- Archive: 8,856,044,347 bytes, SHA-256 `02dd584bda8455487a4b6b4763b876e4c6bdeffbb8b5c0cf3ee3b3c4af693bd2`.
- 18,861 files; all 18,925 ZIP entries extracted with per-file CRC checks;
  paths validated and no symlinks accepted. Original files preserved.
- Local dataset: `data/tartandrive3/20251027/1_salon_mac_cov_day/`.
- Runtime: `macvo:blackwell`, image `sha256:90fe85f1a9c139a8f53533e2420992273648267206eec214952e0b0b18f57b99`;
  Torch 2.7.1+cu128, RTX PRO 5000 Blackwell, random seed 0.

## Measured inputs

| Stream | Images | Dimensions | Encoding |
|---|---:|---|---|
| RGB left rectified color | 1710 | 544 × 1024 × 3 | uint8 |
| RGB right rectified | 1710 | 544 × 1024 | uint8 |
| Thermal left raw / processed | 1696 each | 512 × 640 / 512 × 640 × 3 | uint16 / uint8 |
| Thermal right raw / processed | 1690 each | 512 × 640 / 512 × 640 × 3 | uint16 / uint8 |

Dimensions and encodings checked on the first three images per stream, not
every image. Archive includes raw and processed thermal camera-info/intrinsics,
RGB camera-info/intrinsics, `tf/`, SLAM point clouds, and odometry. The first
SLAM point cloud is float32 with shape `(31734, 3)`.

There are 1710 indexed timestamp rows per image stream. Thermal left has 14
missing rows/files, thermal right 20. Filenames agree with finite timestamp
indices for the four checked inference streams. Both processed thermal streams
contain repeated timestamps (21 left, 23 right); equal indices are not proof
of a fresh or simultaneous acquisition.

1676 indices have all four images and finite timestamps. Frames 0–9 form the
first complete contiguous window. Their timestamp offsets are RGB stereo 0 ms,
thermal stereo approximately 1 ms, and RGB-left to thermal-left approximately
23.12 ms. Over the full common-index set, maximum thermal stereo offset is
402 ms and maximum RGB-left/thermal-left offset is 323.86 ms. These are
same-index offsets, not residuals after nearest-time matching. No synchronization
tolerance or pairing policy has been adopted for research comparisons.

## Real-image execution

Two successive provided `thermal_left_processed` PNGs, indices 0 and 1,
were converted BGR to RGB, cast float32 and divided by 255, without resizing.
Original raw thermal files were not changed. This is a declared input recipe;
its equivalence to MACThermal training preprocessing remains unverified.

`MACT_Offroad.pth` strictly loads all 428 entries. At native 512 × 640,
flow and covariance each have shape `(1, 2, 512, 640)`, are finite, and
covariance is positive. First-use elapsed 0.457 s,
not a throughput benchmark. Arrays, exact input timestamps, script, and log
are retained under local ignored `results/data-access/`.

RGB execution is blocked at strict loading of `MACVO_FrontendCov.pth`:
missing `memory_decoder.delta`; seven unexpected keys from encoder norms and
attention positional embeddings. No RGB inference result is claimed. The
production wrapper uses permissive loading; that does not establish checkpoint
compatibility. No weights were discarded or production loaders changed here.

## Calibration and extraction issues to resolve

- The extracted TF schema uses `transforms` with `src_frame` / `dst_frame`;
  the inspected converter reader expects `frames` with `parent` / `frame`.
  The TF list also repeats child frames under different parents. Do not run
  the inspected reader unchanged or collapse these entries without validating
  transform directions and tree consistency.
- Camera-info includes both raw `k` and rectified projection `p`; verify the
  matrix used with each image stream. For example RGB-left has raw fx about
  455.775, but projection fx about 477.605. The previously inspected extraction
  config uses about 455.775 despite naming a rectified image stream.
- Thermal right projection has positive horizontal translation, unlike RGB
  right. Verify stereo ordering, baseline sign, and frame conventions rather
  than transferring RGB assumptions.
- The separately pinned `yamaha.yaml` is still inaccessible through this
  connection. Extracted TF is available, but calibration consistency and
  provenance still require validation.
- Reference depth/poses are not precomputed in this archive. Generating them
  from SLAM points/odometry does not establish independent ground truth.

## Next experiment

Resolve the RGB checkpoint/code mismatch; establish timestamp-based pairing
with duplicate/missing-frame handling; validate extracted TF/calibration and
rectified intrinsics. Then run bounded stereo/depth and reference-pose
extraction before evaluating geometric error or cross-modal covariance rankings.
This one clip is sufficient for the next engineering checks; no full-date
transfer is required yet.
