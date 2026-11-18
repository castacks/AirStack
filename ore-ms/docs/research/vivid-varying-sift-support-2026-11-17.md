# ViViD++ varying-light SIFT proposal support

The [payload intake](vivid-handheld-payload-intake-2026-11-17.md)
left fine pose accuracy unqualified. This test asks whether thermal adds
essential/cheirality-supported proposals to the existing SIFT frontend.
It uses the complete `varying_robust.bag`, all 71 frozen half-second
anchors, the 25 ms endpoint gate, and the published camera calibrations.
The matcher and routing did not use mocap or gyro values.

The existing native grayscale helper maps uint16 to 8-bit using the dtype
range. Thermal was decoded as mono16 with recorded row stride and byte order.
RGB8 was converted to the grayscale helper's BGR convention. The two camera
models passed the grid roundtrip. SIFT, mutual 0.75 matching, focal-400
ray lift and essential/cheirality settings were reused from the repository.
The local runtime used OpenCV 5.1.0 and NumPy 2.5.5.

## Complete-span result

| Policy | Supported proposals / 71 | RGB calls | Thermal calls | Longest sampled no-output run |
|---|---:|---:|---:|---:|
| Fixed RGB | 33 | 65 | 0 | 4 anchors |
| Fixed thermal | 0 | 0 | 68 | 71 anchors |
| RGB-first | 33 | 65 | 35 | 4 anchors |
| Thermal-first | 33 | 65 | 68 | 4 anchors |

RGB returns 11/24, 12/24 and 10/23 proposals in the three fixed time thirds.
Thermal returns zero in each third. RGB has 6 unsupported inputs, 20
insufficient-match results, and 12 insufficient-cheirality results. Thermal
has 3 unsupported inputs and 68 insufficient-match results. All 35 thermal
fallback attempts failed to add a proposal. Calls reflect replayed policies,
with both fixed controls evaluated and no live latency measurement.

All 70 distinct thermal endpoint frames had zero detected SIFT keypoints.
For supported thermal source frames, the 8-bit p95–p05 span after conversion
was at most one level and median gray values clustered near 12. The
[author describes a 14-bit thermal signal](https://visibilitydataset.github.io/4_download.html).
The zero result applies to this unchanged dtype-range preprocessing on the
ViViD++ payload. It provides no evidence that the native thermal scene lacks
structure. The [M2P2 image-space result](m2p2-night-registration-and-support-2026-11-17.md)
used already rendered mono8 JPEG and a different recording. Keep those input
representations distinct.

## Author export scaling control

The [author export script](https://urserver.kaist.ac.kr/publicdata/ViViD++/process_img.py)
divides the thermal samples by 50 before casting to uint8. A second, frozen
pass changed only that conversion, verified absence of uint8 wrap, and used
the same 71 anchors and handheld intrinsics. The example script's image
subsampling and fisheye remap were outside this control. The upper end of the
per-frame p95–p05 span rose from one to two levels. The maximum on either
thermal endpoint was 10 keypoints and the maximum mutual match count was six,
below the existing eight-match minimum. Thermal still produced 0/71 proposals
and all RGB prediction rows were identical. Both runs and their hashes were
retained. Neither scale establishes how learned matching or a different
thermal representation would perform.

The independent summary checked all 142 prediction rows against pinned
timestamps and support states, and verified valid rotations, translation
norms and cheirality on supported outputs. The protocols, hashes and output
files are in `/root/ore-ms-artifacts/vivid-plusplus-intake/`. No new bag was
downloaded, no files were uploaded to Box, and no pose accuracy was scored.
