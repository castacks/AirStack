# IRS RTVI visible–thermal mocap comparison

The [ViViD++ depth check](vivid-depth-reference-crosscheck-2026-11-17.md)
did not qualify a stronger pose reference. The
[IRS RTVI release](https://christopherdoer.github.io/datasets/irs_rtvi_datasets_iros2021)
provides hardware-synchronized visible and thermal streams and a separate
Vicon–IMU trajectory. Screened the archive remotely, obtained the five small
mocap CSVs with ZIP CRC checks, and selected `mocap_dark` and `mocap_easy`
for the low-dynamics lights-off/bright comparison. Other bag members were
left at index size. The visible camera supplies monochrome frames.

Used HTTP byte ranges to transfer only the compressed members for the two
bags. Decompressed each to local storage while verifying ZIP CRC and exact
size, then hashed the result. Total raw bag size is 3,138,780,081 bytes.
The full 9.77 GB source archive was not staged. No Box files were added.

| Recording | On-disk bag bytes | Visual / thermal frames | Reference poses / span |
|---|---:|---:|---:|
| mocap_easy | 1,262,246,330 | 1,900 / 1,899 | 1,590 / 79.465 s |
| mocap_dark | 1,876,533,751 | 2,707 / 2,938 | 2,394 / 120.031 s |

The actual streams are 640×512 mono8 and match the supplied equidistant
calibrations. No thermal display conversion was needed. Used the unchanged
SIFT and learned checkpoints, native inputs, 4,096 learned grid queries,
one-pixel cycle gate, focal-400 lift and essential/cheirality solver.
Anchors are nonoverlapping half-second intervals over the complete visual
header span, with a 25 ms endpoint gate. The longer thermal-only tail in
mocap_dark lies outside the visual-anchored comparison.

Nominal motion used the published Vicon–IMU body trajectory, the released
camera-to-IMU transforms and the author loader's signed shifts (+2 ms visual,
−4 ms thermal), applied to each camera's own endpoints. No fit or
extrapolation. Camera-timed mocap/gyro disagreement p95 is approximately
0.40° for easy and 2.40° for dark. The published pose uses the IMU in
the Vicon reconstruction, so gyro is a recording-consistency check with
known dependence on the reference. The inferencers use camera data only.
These comparisons support nominal relative-rotation agreement, with
physical accuracy and stateful recovery outside the evidence.

## Proposal support and nominal rotation agreement

The median visual gray is 117/255 in easy and 5/255 in dark, with median
per-image p95–p05 spans 204 and 8. Thermal gray spans are 100 and 108. These
statistics describe the released images. They do not measure illuminance.
Both bags use separate motions, so the paired-record difference does not
isolate lighting causally.

| Recording | Full anchors | SIFT visual / thermal / either | Learned visual / thermal / either |
|---|---:|---:|---:|
| Easy | 189 | 143 / 109 / 143 | 142 / 131 / 146 |
| Dark | 270 | 21 / 139 / 147 | 152 / 180 / 210 |

On the 157 easy and 239 dark anchors where both cameras have bracketed motion
references, the learned status-only orders are evaluated at the chosen
camera's own endpoints. Counts below require the proposal rotation to be
within 5° of both the nominal Vicon-based orientation and integrated gyro.
The criterion is applied only after replaying the frozen status order.

| Recording | Fixed visual | Fixed thermal | Visual-first | Thermal-first | Proposals from either order |
|---|---:|---:|---:|---:|---:|
| Easy, 157 anchors | 142 | 125 | 146 | 140 | 146 |
| Dark, 239 anchors | 41 | 154 | 83 | 161 | 201 |

The coverage of the two fallback orders is identical by construction.
In dark, thermal-first agrees with both signals on 78 more anchors than
visual-first, largely through the choice when both cameras propose an
estimate. In easy, visual-first agrees on six more anchors. Among paired
dark thermal proposals, learned gives 154/178 agreements versus 5/178
for zero rotation, with 149 added agreements and no losses. Of 170
learned dark thermal proposals where both nominal rotations exceed 5°,
147 agree with both signals within 5°. For SIFT, dark thermal yields
111/137 agreements versus 6/137 for identity. These contrasts are
specific to this published rig and these recordings. The ViViD++ result
stays a separate [negative nominal control](vivid-zero-rotation-control-2026-11-17.md).

Audited all 1,836 prediction rows and the independent policy replays.
Both strict learned loads, deterministic repeat checks and model hashes
passed using the pinned compiler image. The protocol, payload preflights,
prediction locks, nominal ledgers and verified checksums are under
`/root/ore-ms-artifacts/irs-rtvi-intake/`. The two raw bags use about 3.0 GiB
in `/osmo/run/workspace/irs-rtvi/` and are the large cleanup candidates.
