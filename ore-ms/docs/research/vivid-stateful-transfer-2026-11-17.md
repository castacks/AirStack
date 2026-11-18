# ViViD++ stateful RRxIO transfer

The [earlier ViViD++ experiments](vivid-three-condition-support-2026-11-17.md)
used independent image pairs. This is the first continuous camera+external
IMU replay in this study. It runs the native monocular RRxIO filter over
the full varying-flashlight, bright and dark handheld robust recordings.
Each recording has one fixed RGB arm and one fixed thermal arm.

Use the [ViViD++ handheld calibration and sensor-header convention](https://visibilitydataset.github.io/4_download.html)
with the native IRS visual and thermal presets. RGB becomes grayscale.
Thermal uses the authors' integer division by 50 at original resolution.
The raw maximum across the three runs was 5,193, so no uint8 wrapping was
possible. The published RGB-to-IMU shift is +11.837 ms. Thermal retains the
sensor header because the released thermal-to-RGB chain supplies no temporal
offset. The thermal geometry composes this chain with the RGB–IMU extrinsic.
The estimator never receives ground truth or other position updates.

## Reference and ingestion checks

The reference follows the marker → RGB → IMU spatial chain described by the
[ViViD++ paper](https://arxiv.org/html/2204.06183#S3.SS3), with the released
sequence-specific marker time offsets. The initial +Z-up binding check
failed consistently. Before scoring the estimators, tested the gravity
convention with low-motion IMU measurements rotated by the nominal
reference orientations. Each run clustered around +Y, with median angle
relative to +Y of 9.0° in bright, 10.9° in varying and 14.1° in dark.
The pre-score amendment rotates reference coordinates from `(x,y,z)`
to `(x,-z,y)`. This changes the world convention with a fixed proper
rotation. Only translation and yaw are then fitted. No axis is selected
using odometry residuals, and the fitted transform has unit scale.

The first build tried to sort ROS bag entries by value, which cannot be
assigned in this ROS version. Sorting timestamped handles fixed the
build. The first varying RGB run skipped three early IMU messages because
the legacy arrival-time start guard was still active in the
header-ordered replay. The final whole-bag wrapper disables that guard.
The output audit matches all image and IMU input counts to preflight.
Direct estimator logs retain all published poses and agree exactly with
overlapping ROS subscriber rows.

The synthetic RRxIO startup test warns about a pixel Jacobian at a
large derivative. An independent check of the actual linked radtan
camera model over 63 in-image rays per camera passed the pixel
round-trip and finite-difference derivative bounds. The startup
warning is retained.

## Transfer result

The bright RGB gate requires nominal position RMSE at most 0.30 m and
at least 90% reference-associated coverage before interpreting any sensor
ordering across conditions. Bright RGB scores **0.718 m** with
**99.87%** associated coverage, so the transfer fails that gate.

| Recording | RGB odometry rows | Thermal odometry rows | RGB median tracked | Thermal median tracked |
|---|---:|---:|---:|---:|
| Bright | 1,539 | 1,028 | 13 | 1 |
| Varying flashlight | 1,041 | 693 | 11 | 2 |
| Dark | 1,036 | 697 | 3 | 2 |

All six runs finished and initialized once. The varying RGB and thermal
estimates end hundreds of metres from their starting positions, while
the supplied IMU-origin reference stays within about 1.5 m of its
start. Thus availability and per-frame tracked-feature counts cannot
qualify these trajectories. The adverse-window errors are retained in
the artifact packet as transfer diagnostics. This configuration supplies
no validated sensor ranking or camera-switch evidence for ViViD++.
Keep the recording staged for an independently qualified stateful
baseline. No replay settings were tuned on these outcomes.

Protocols, input pins, failed attempts, projection check, raw outputs
and the independent audit are under
`/root/ore-ms-artifacts/vivid-plusplus-intake/stateful/`.
No additional bag copies, data downloads or Box writes were needed.
