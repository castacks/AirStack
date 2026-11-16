# OutdoorNight: added thermal rotations agree with gyro, but fixed thermal is stronger

16 November2026, America/New_York. Completed the
[prospective raw-binding/gyro protocol](outdoornight-gyro-protocol-2026-11-16.md)
using the unchanged [locked matching outputs](outdoornight-matching-findings-2026-11-16.md).
No new model execution, training, threshold/window/axis/clock/bias fitting or
translation scoring. This is nominal angular agreement with measured gyro,
not independently certified physical pose accuracy or stateful VO recovery.

## Main finding on the frozen moving population

There are242anchors with both cameras' input and gyro support and at least2°
gyro rotation in each sensor's own interval. Missing proposals count as no
agreement in the same242-anchor denominator.

| Learned policy | Outputs /242 | Within1° of gyro /242 | Within5° /242 | Conditional median disagreement |
|---|---:|---:|---:|---:|
| Fixed RGB | 141 | 49 | 81 | 3.2135° |
| Fixed thermal | 235 | 158 | 212 | 0.7047° |
| RGB-first thermal backup | 242 | 125 | 180 | 0.9167° |

Thermal adds101proposals where RGB returns none. Of these,76agree within1° and
99within5°; median disagreement is0.6370°. A zero-rotation assumption on exactly
those same thermal endpoints agrees0/101within1° and13/101within5°. This is
evidence of nonzero angular-motion information beyond merely filling output gaps
or exploiting static intervals, under the nominal published camera/gyro contract.
Two of the101additions nevertheless exceed5°;25exceed1°. No incorrect addition is
hidden and no translation/recovery claim follows.

The mixed result is equally important: fixed thermal has more moving-anchor
agreement than RGB-first at both bounds despite fewer outputs. The rule retains
accepted RGB proposals that may disagree strongly with gyro. Complete availability
does not imply the best angular consistency. Do not introduce a new gate or tune
the already-exposed outcomes to erase this comparison.

## Reference search and actual raw bindings

The official [OutdoorNight missing-reference issue35](https://github.com/yuanjun-gao/ICCV2023_SLAM_Challenge/issues/35)
is open with zero comments at inspection. The maintainer's
[broader reference release](https://github.com/yuanjun-gao/ICCV2023_SLAM_Challenge/issues/31)
links a parent with fusion, LiDAR and visual-inertial folders. The last has six
files without OutdoorNight; the other two were already inspected. This closes
these specific public listings absent new evidence, not all possible references.

M2P2's public V4.1 catalog has684bags and no trajectory sidecar. Its released
README lists wheel/filtered odometry, while its paper discusses lidar-inertial
reference generation. A ready qualified trajectory was not established in the
inspected catalog/58-path source tree. This does not reject M2P2 or prevent a
separately validated LiDAR-reference pipeline. No large M2P2bag was downloaded.
Sources: [project](https://people.cs.gmu.edu/~xiao/Research/M2P2/),
[paper, targeted sensor/reference sections](https://arxiv.org/html/2410.01105v2),
[released code](https://github.com/RobotiXX/M2P2).

The complete11,850,330,134-byte OutdoorNight raw ZIP was acquired with354strict
HTTP206ranges, exact offsets/lengths/totals and observed assembled SHA256:
`b8abc30a6a22fdfb31aa7ec0d90e3ee45ac6241697cb6e04bce4721163160cf7`.
All six original ROSbags pass ZIP CRC and have individual hashes. Their full
connection inventories contain only RGB, thermal and IMU—no pose/odometry topic.

| Full raw/export check | Result |
|---|---:|
| RGB arrays, exact BGR pixels and header stamps | 11,925 /11,925 |
| Thermal arrays, exact uint16 pixels and header stamps | 14,446 /14,446 |
| IMU rows, exact quaternion/gyro/acceleration numeric values and stamps | 96,715 /96,715 |
| Missing, different, unmatched or duplicate rows | 0 |
| Maximum observed raw/export timestamp difference | 0ns |
| Independent raw-IMU reread checks | 193,444 passed |

Raw RGB is`bgr8`,640x480,frame`d`; thermal is`16UC1`,640x512,
frame`thermal/optical_frame`; IMU topic is`/cmu_rc7/imu/data`,frame`epson`.
The IMU message definition specifies rad/s angular velocity. Orientation covariance
begins−1 (orientation unavailable); gyro covariance is all zeros (unknown), not a
calibrated zero-noise estimate. No fused-orientation or acceleration-derived pose
was used. Topic prefixes/frame names do not independently prove the published
SP1 camera-to-IMU extrinsic; its source association remains a nominal assumption.

## Integration, populations and all retained cases

Use left-held raw angular velocity with integer-ns boundary durations at each
camera's actual selected source/target stamps. Source-from-target integrated
rotation is transposed to match target-from-source image proposals. Conjugate
camera rotations by their own published camera-to-IMU rotation. No fitted
offset, axis flip, scale, gyro bias or lever-arm translation.

Of971declared anchors,964have both-camera input/gyro support. Four sensor
intervals at anchors0/1 fall outside IMU coverage; ten intervals at anchors86,
123,137,281,666 cross a >20ms gap. All14unsupported sensor intervals remain.
The242both-camera >=2° anchors are frozen before disagreement calculations.
All3,884saved prediction rows remain; no new inference or errors guide routing.

| Learned population | RGB within1° /5° | Thermal within1° /5° | Backup within1° /5° |
|---|---:|---:|---:|
| All964common-gyro anchors | 384 /487 | 670 /768 | 646 /773 |
| First third,318eligible | 146 /180 | 153 /190 | 178 /212 |
| Middle third,324eligible | 69 /105 | 298 /319 | 237 /287 |
| Last third,322eligible | 169 /202 | 219 /259 | 231 /274 |

All288learned thermal additions have gyro support:262within1°,286within5°,
median0.3453°. Zero rotation gives151/200on those same additions. Across the
whole common population, zero rotation on RGB endpoints gives553/783agreements,
exceeding learned backup at the coarse bound. Static/small-motion mixture can
therefore dominate whole-record averages; keep the moving result separate.

SIFT provides a consistent but weaker control: on the242moving anchors, RGB/
thermal/backup outputs71/146/181, within1° counts0/92/72 andwithin5°27/141/136.
Its110thermal additions give72/109agreements versus0/23for same-endpoint zero.
Fixed thermal again has better angular agreement than RGB-first, retaining the
failure of a status-only rule to distinguish an accepted poor RGB estimate.

## Verification, limits and stopping decision

Ten CPU controls cover rotation direction, noncommuting increments, exact boundary
durations, stationary identity, extrapolation/gap rejection, endian/padded pixel
decoding, deterministic nearest binding and moving/identity denominators. A
separate auditor independently integrates quaternions and reconstructs all
rotations/cohorts/agreements/zero controls:40,227checks pass. Raw gyro is also
independently reread from all six bags and matched numerically to the exports.
The original GPU inference and status-choice locks remain unchanged.

Pixel/gyro identity strengthens recording/export provenance, not physical exposure
metrology. Uncalibrated gyro bias/noise and the nominal camera-to-IMU association
remain; no physical uncertainty bound or independence certificate is supplied.
This is one exposed recording, with temporal dependence and unknown checkpoint
training membership. Do not turn gyro agreement into translation, full-pose,
held-out generalization or persistent-state recovery accuracy.

Evidence: `results/night-reference-readiness/` and `results/outdoornight-gyro/`.
Raw ZIP/bags remain ignored under `data/deployment-characterization/`. A local
member-readback verified packet is under
`/root/ore-ms-artifacts/outdoornight-gyro-2026-11-16/`, with explicit parent matching
packet and raw-input/runtime/model hash prerequisites. No Box upload, author
contact or Git publication. PNG/PDF figure is in the diagnostic output folder.

Stop this diagnostic without a bias/clock/axis rescue. The angular evidence now
supports useful moving thermal additions under its stated contract, while fixed
thermal exposes a limitation of RGB-first. A new selection study needs a separate
multi-record protocol and that fixed-thermal control; translation/trajectory
claims still need a matching qualified reference. Do not repeat the closed public
reference search without a new source.
