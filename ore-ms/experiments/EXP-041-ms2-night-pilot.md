# EXP-041: MS² night pilot acquisition and recorded-time audit

Date: 2026-10-23. Starting commit `f814f3204`. The user supplied the official sync_data, proj_depth and odom download scripts. **The fixed night sequence is acquired and a 20-ID paired pilot is preserved. Source schemas pass inspection, but recorded thermal stereo offsets are 15.2–49.8 ms across the sequence, with zero pairs satisfying the inherited 2 ms stereo diagnostic gate. No MAC inference or relative-error result is claimed.**

## Fixed acquisition and source preservation

Choose `_2021-08-13-21-18-04`, the first lexical sequence in the official night test split, before inspecting its pixels or model errors. EXP-040 recovered small day/night/rain odometry archives and verified download access. Sequential bzip2 image/depth archives could not supply a complete pilot through small prefixes. Acquire only this night sequence; do not acquire full day/rain image datasets.

| Archive | Bytes | Locally observed SHA256 |
|---|---:|---|
| sync_data | 19,721,804,466 | `7d4d45ba6aaa719f2b5eda001f45977a3ad2db594f486c62664c5c49d4281b4a` |
| proj_depth | 20,010,206,049 | `aacb1b86860a7b24747c75b2d38066adfcb61f4bd4bfad8453f6433d721916ef` |

Every requested HTTP range must have exact 206 Content-Range and byte count. Assemble ranges in offset order and record whole-file SHA256. No publisher archive digest was available: these hashes record acquired-byte identity, not comparison with a publisher checksum. Safely scan tar headers through indexed-bzip2 1.7.0 with bounded selected-member sizes, rejecting traversal and links. Preserve tar member offsets and bzip2 block maps for subsequent selective extraction. No supplied shell script is executed.

The fixed pilot uses IDs `000000`–`000019`, with no substitution based on outcomes: 80 source images (RGB/thermal × left/right ×20), 120 uint16 projected depth PNGs (single-scan/multi-scan/filtered ×RGB/thermal ×20), 40 corresponding camera pose files, source calibration, and all nine source timestamp series. Both tar scans complete. Full archives remain local under `results/ms2-night-pilot/archives/`; backups preserve the bounded pilot and acquisition provenance, not another40GB archive copy.

## Actual file and calibration schema

RGB is uint8 BGR 1224x384; thermal is **raw uint16** 640x256. Preserve thermal precision. Source previews use per-image p0.5–99.5 thermal contrast only for display; that rendering is not a model preprocessing choice. RGB previews retain their original values and show a lit road. This establishes visual content, not tracking or MAC accuracy.

`calib.npy` is a scalar object-array dictionary. The inspector allows only required NumPy numeric-array constructors during unpickling, then checks dictionary values and bounds. It does not call unrestricted `np.load(...,allow_pickle=True)`. The archive readme confirms rectified camera intrinsics and millimeter extrinsic translations.

| Camera | fx=fy (px) | cx,cy (px) | Baseline (m) |
|---|---:|---:|---:|
| RGB left | 764.5138706 | 579.5470123,153.0054131 | 0.2991842053 |
| Thermal left | 387.7869505 | 300.8201370,151.6257629 | 0.3038995142 |

All40 pilot poses are finite4x4 homogeneous matrices and pass the stated1e-5 rotation orthogonality/determinant screen. EXP040's22 flagged source matrices across three complete recordings are not silently repaired; none is in this pilot. Initial-to-final translation-column changes are0.02391m RGB and0.02373m thermal over1.93310s. These are reference values, not centimeter-accurate physical motion measurements. The opening interval provides little motion for a temporal-error study.

Source depth readmes confirm encoding `meters*256`, zero invalid. Median nonzero coverage over20 frames:

| Camera | Single scan |32-scan merged | Filtered |
|---|---:|---:|---:|
| RGB |1.525% |3.851% |2.962% |
| Thermal |5.712% |7.828% |6.037% |

Coverage is sparse and camera populations differ. The [official setup](https://sites.google.com/view/multi-spectral-stereo-dataset/setup) describes pose/ICP aggregation and network-based filtering; filtered depth is not wholly model-independent ground truth. Preserve raw versus filtered provenance. The [official data format](https://sites.google.com/view/multi-spectral-stereo-dataset/data-format) describes GNSS/IMU-derived camera references; their accuracy and pose direction still need a task-specific audit.

![Fixed MS² night pilot source previews](../results/ms2-night-pilot/pilot-preview.png)

## Recorded-time discrepancy

Every preserved source timestamp series has10,131 strictly increasing integer entries. Interpret recorded values in ns and compare same-ID source rows without resampling, offsets or timestamp correction. Full RGB stereo differences have min/median/max0.000010/0.020955/66.626496ms;10,130/10,131 satisfy2ms.

| Recorded-time difference | Full-sequence min/median/max (ms) |
|---|---|
| Thermal left/right, same ID |15.219450 /33.766417 /49.753598 |
| RGB-left / thermal-left |0.004431 /19.356632 /72.591637 |
| RGB-right / thermal-right |0.012179 /19.536966 /69.400472 |
| Thermal-left / nearest thermal-right |15.219450 /33.742113 /49.753598 |

No same-ID or nearest-right thermal pair is within2ms. Hence the inherited2ms-stereo/30ms-cross diagnostic yields0 eligible joint frames,0 consecutive pairs and no eligible run. The fixed first20 have22.888–45.470ms thermal stereo offsets and0 joint-eligible frames as well. Nearest-right calculations are a sensitivity audit only; no input images are re-paired.

The official setup says thermal stereo uses the left camera's synchronization signal. That statement and the recorded timestamp separation must both be retained. These files alone do **not** establish whether timestamps denote exposure, reception, driver scheduling, a clock offset or a physical acquisition delay. The inherited2ms gate is not a newly validated universal MS² acceptance rule. Do not declare the physical stereo pair unsynchronized, shift timestamps by a fitted constant, relax the gate, or run a motion-sensitive error comparison under an ungrounded timing assumption.

## Validation and replay

5,355 independent source controls pass: selected member byte/SHA identities, PNG chunk CRCs and IHDR-versus-decoder precision/shape, numeric camera dictionary schema, common IDs and scalar pose structure.37 additional independent NumPy/integer controls reproduce full timing medians/extrema, nearest-pair offsets, source hashes and camera intrinsic/baseline plausibility. A passing arithmetic check does not turn failed timing eligibility into a scientific pass.

Reusable `Scripts/Setup/audit_ms2_pilot.py --directory <restored-pilot-root> --output <json>` requires CPU NumPy/OpenCV and preserved input manifests; no GPU/checkpoints. It writes decoded calibration values and source/schema results. Acquisition, extraction, timing audits, independent validator, summary and preview scripts are preserved under `results/ms2-night-pilot/staging/`; exact input/output manifests, source readmes, plots and validation are beside them. The full acquisition is reproducible only with source access and the omitted large archives; the bounded pilot checks replay independently from the artifact backup.

No training, model inference, covariance weighting, camera calibration or VO default changes. Keep this as a source-readiness result. Next work is to resolve thermal timestamp semantics against acquisition documentation/code or an independently justified timing protocol, and identify a moving interval under that protocol before a paired MAC reliability comparison. A separate static depth study would require its own stated target and label assumptions.

Recovery target: Box `backups/2026-10-23-exp-041`, including bounded source inputs/results, source notes, analysis code/notebook and an incremental bundle after EXP040. Credentials, signed URLs, packages and full archives are excluded.
