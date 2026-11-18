# M2P2 night registration and image support

The study used the same 30-second `r2c2_night_1` Dataverse file 5496 as the
[prior timing audit](m2p2-reference-feasibility-2026-11-17.md). The objective
was to qualify a camera-independent reference for the existing RGB–thermal
comparison. The reference did not qualify. The image-space diagnostic
quantifies correspondence supply on this chunk.

## Camera-independent registration test

The bounded test used fixed [point-to-point ICP](https://doi.org/10.1109/34.121791)
from identity, with gyro-only deskew explicitly anchored at each cloud header.
This choice exercised the qualified timing/frame contract on the available CPU
runtime. It did not execute stock LIO-SAM or estimate a reference trajectory.
All 297 adjacent and 293 five-scan-separated pairs within eligible zero-based
scans 1–298 were retained. Reference camera streams, GNSS and wheel odometry were
excluded. Fixed settings were 1–25 m range, 0.15 m voxel means, up to 2,500
systematically sampled points, 20 iterations, 0.35 m correspondence cutoff
and a minimum of 100 correspondences. NumPy 2.5.5, SciPy 1.18.1 and lz4 4.4.5.
Synthetic checks reproduced a known rigid transform and verified deskew
direction and header anchoring.

All 590 pairs returned estimates in the full, reverse, raw and even/odd ring
arms. A separate summary verified all 2,950 transforms and composed adjacent
estimates for a half-second consistency check. For the 293 half-second pairs,
the full-cloud median correspondence fraction was 82.6% and the median
accepted residual RMS was 0.197 m.

| Half-second consistency check | Rotation discrepancy median / p95 / max | Above 1° |
|---|---:|---:|
| Deskew versus raw | 0.544° / 1.289° / 2.168° | 40 / 293 |
| Even versus odd rings | 0.140° / 0.535° / 1.707° | 6 / 293 |
| Forward versus inverse reverse | 0.080° / 0.258° / 0.760° | 0 / 293 |
| Direct versus five composed adjacent estimates | 0.190° / 2.524° / 6.621° | 31 / 293 |

Direct versus composed estimates also differed by 0.274 m at p95. Directed
translation differed by more than 15° in 59 of 268 comparable cases with both
translation magnitudes at least 1 cm. Inverse and ring agreement can coexist
with shared estimator error. No independent physical error bound exists, and
the direct/composed comparison is inconsistent at the study's scoring scale.
Keep this bag out of 1° and joint pose scoring. No parameter or window rescue
was performed.

## Image timing and correspondence support

All 883 camera payloads decoded at recorded dimensions. Published camera-info
records bind to image **bag record time** for all frames, consistent with the
[released patcher](https://github.com/RobotiXX/M2P2/blob/f724e7ad0b14c4d5ebb3550ab459678ddb9744e9/dataset_processing_scripts/rosbag_patcher/patch_rosbags.py#L156-L175).
Image headers were retained for pairing. All 285 thermal frames pair uniquely
to left RGB with signed header deltas −11.76 to −8.64 ms. The median
per-image RGB luma was 4/255 for both cameras, with a median p95–p05 span of
1. The thermal frames are decoded 8-bit grayscale JPEG and have a median span
of 118. The pixel statistics do not supply illuminance or thermal radiometry.

The first attempt planned calibrated SIFT proposals but stopped before writing
predictions. The unchanged sub-0.001 px roundtrip check passed left RGB and
failed thermal near the left corners (worst sampled residual 3.485 px).
No calibration or acceptance threshold was changed. The separate image-space
diagnostic reused the frozen 59 half-second anchors, nearest image headers with
the existing 25 ms endpoint tolerance, native grayscale, SIFT and the mutual
0.75 ratio. It used OpenCV 5.1.0 and NumPy 2.5.5, with no learned inference.

| Camera | Supported input pairs | Pairs with at least eight mutual matches | Mutual matches, min / median / max |
|---|---:|---:|---:|
| Left RGB | 59 / 59 | 0 / 59 | 0 / 0 / 3 |
| Thermal | 56 / 59 | 56 / 59 | 36 / 115 / 682 |

Thermal lacked inputs for anchors 0–2. This shows greater correspondence
supply on the tested chunk. Eight matches only meets an input requirement
for the existing geometric stage. No pose proposals, policy gains or
pose accuracy were scored.

The frozen protocols, scripts, JSON predictions/diagnostics and the failed
calibration check are documented locally under
`/root/ore-ms-artifacts/m2p2-5496-range-probe/`. The single raw bag remains
in local staging and in Box. No additional bag was downloaded or uploaded
for this experiment.
