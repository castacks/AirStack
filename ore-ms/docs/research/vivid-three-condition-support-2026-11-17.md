# ViViD++ bright, varying and dark proposal support

Completed the prospective bright and dark controls for the
[varying-light comparison](vivid-varying-learned-support-2026-11-17.md).
Used robust-motion recordings `global_robust`, `varying_robust` and
`dark_robust` with each complete half-second grid, the same 25 ms endpoint
gate, released handheld calibration and fixed author thermal scaling.
SIFT and the original learned checkpoints used the same frame endpoints
within each recording. No mocap, IMU or light labels entered inference or
routing. Recording-level comparisons are descriptive because scene, exposure
and motion can differ.

| Recording | Anchors | SIFT RGB | SIFT thermal | Learned RGB | Learned thermal | Learned either fallback |
|---|---:|---:|---:|---:|---:|---:|
| Bright | 103 | 80 | 0 | 79 | 83 | 92 |
| Varying | 71 | 33 | 0 | 51 | 62 | 64 |
| Dark | 69 | 0 | 0 | 53 | 65 | 66 |

RGB-first added exactly 13 proposals in each learned recording, on 23 bright,
18 varying and 15 dark thermal attempts. Thermal-first added 9, 2 and 1 RGB
proposals over fixed thermal. The two fallback orders have identical total
support by construction. They choose different sensors on jointly supported
anchors. Longest sampled failure runs for fixed RGB versus either fallback
were 7→4 bright, 6→4 varying and 4→2 dark.

| Recording | Learned projected RGB-only time | Thermal-only | RGB-first | Thermal-first |
|---|---:|---:|---:|---:|
| Bright | 26.26 s | 27.06 s | 32.48 s | 32.27 s |
| Varying | 16.64 s | 18.36 s | 21.52 s | 20.67 s |
| Dark | 17.14 s | 18.55 s | 21.26 s | 19.58 s |

Time is the sum of measured pair calls under each replayed policy. It excludes
startup, acquisition, scheduling and live routing. These results measure
support for an essential/cheirality proposal.

## Dark RGB interpretation

In the dark recording, both endpoint images had 95th-percentile gray level
zero in every one of the 67 supported RGB pairs. The learned frontend
nevertheless produced 53 geometrically supported proposals. Small nonzero
regions can remain, and physical illuminance was not measured here. A
post-hoc diagnostic fed the same learned RGB model an identical all-black
pair at native size. It returned substantial flow magnitudes even though no
correspondences survived the fixed cycle gate and no proposal was produced.
This single control does not explain the 53 recorded-image proposals or
validate their motion. No new screening policy was applied.

## Input and reference audit

The two added bags matched the KAIST header and complete footers and were
hashed locally. All image/IMU headers, sizes and pixel encodings matched the
expected streams. Bright has 98 and dark 66 anchors with both camera inputs
and mocap/gyro brackets. The prior varying audit has 61. On the individual
camera endpoints, the half-second mocap/gyro disagreement p95 is
2.527° RGB / 2.288° thermal for bright and 3.718° RGB / 3.628° thermal for dark.
The comparison therefore remains reference-free. No pose scoring was run.

Verified all 688 new prediction rows, pinned inputs, geometry and strict
checkpoint loads. Both learned controls reproduced their first flows bitwise
and preserved model state. The SIFT log's final print reused the image byte
count and overstated the number of rows. The independent audit verified
206 bright and 138 dark rows in each of the two frontends.

The bright and dark bags add 5.38 GB locally. The three ViViD++ bags occupy
about 7.1 GiB in `/osmo/run/workspace/vivid-plusplus/`. No additional Box
storage was used. Result files and their hashes are under
`/root/ore-ms-artifacts/vivid-plusplus-intake/`.
