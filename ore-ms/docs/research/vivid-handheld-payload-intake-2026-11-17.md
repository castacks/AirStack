# ViViD++ handheld indoor payload intake

Author-provided access resolved the blocker in the
[reference intake](vivid-cart-reference-intake-2026-11-17.md). The preselected
set uses the same robust-motion category for the varying-flashlight sequence,
bright control and dark control. Bag indices, mocap CSVs, and calibration were
obtained for all three.

| Sequence | Bag bytes | RGB / thermal frames | External IMU messages | Mocap rows / duration |
|---|---:|---:|---:|---:|
| varying_robust | 2,217,255,316 | 1,045 / 695 | 3,580 | 7,273 / 36.360 s |
| global_robust | 3,254,542,723 | 1,541 / 1,030 | 5,255 | 10,682 / 53.405 s |
| dark_robust | 2,124,723,744 | 1,040 / 699 | 3,497 | 7,116 / 35.575 s |

The three mocap tables contain finite rows, uniform 5 ms relative timestamps
and unit quaternions to numerical precision. The released calibration binds
the mocap marker to RGB, RGB to the external IMU and thermal to RGB. The
RGB and thermal pinhole/radtan models pass the existing 0.001-pixel grid
roundtrip check. This verifies numerical projection consistency. Physical
extrinsic and clock accuracy require separate checks.

Only `varying_robust.bag` was downloaded in full. The resumed transfer used
the two author-provided mirrors and verified matching header bytes, the
64 KiB overlap at the switch and the complete KAIST footer. SHA-256 is
`23576b393e289a82d1247a7702ea8c4644a0799f44cb3597b2a30e16c779cd19`.
The local path is `/osmo/run/workspace/vivid-plusplus/varying_robust.bag`.
No full control bags or Box uploads were made.

## Varying sequence payload and time diagnostic

All RGB, thermal and external IMU message headers were decoded. Image payload
dimensions and byte counts match the released models, with 640×480 RGB8 and
640×512 mono16 thermal. All three streams have increasing header timestamps.
Maximum gaps are 101 ms RGB, 1,037 ms thermal and 21 ms external IMU. The
published [download instructions](https://visibilitydataset.github.io/4_download.html)
specify using sensor headers. Retain the native thermal values and the actual
gaps. The data have separate sensor and record times.

The mocap CSV starts at relative zero. The calibration lists offsets of
+0.640 s varying, 0.000 s bright and +0.250 s dark. The
[paper](https://arxiv.org/html/2204.06183#S3.SS3) describes the gyro-based
offset procedure. As a fixed diagnostic, used bag start as the relative
origin, evaluated unshifted and the two sign interpretations of the supplied
varying offset, and did not search offsets or fit rotations.

On 0.1 s intervals, adding the published +0.640 s to relative sensor header
time gives gyro/mocap angular-magnitude correlation 0.954 and rotation-vector
difference 0.422° median / 1.024° p95. Unshifted correlation is 0.442 and
subtraction gives 0.277. This supports the positive sign and origin as a
nominal convention, with residual physical binding uncertainty.

Under the existing 0.5 s anchor grid and ±25 ms camera endpoint rule, 65/71 RGB
and 68/71 thermal inputs are supported. Both cameras have input and reference/
gyro coverage on 61 anchors. On each camera's own qualified endpoints,
external gyro versus nominal mocap differs by 1.524° / 3.324° (RGB median / p95)
and 1.501° / 3.347° (thermal). This comparison is an agreement diagnostic,
with gyro noise and integration error contributing unknown amounts. It does
not certify a 1° pose reference or identify a new offset. No matcher or pose
scoring ran. Keep bright and dark staged at index size pending the reference
decision.

Indices, hashes and per-anchor audit are under
`/root/ore-ms-artifacts/vivid-plusplus-intake/`.
