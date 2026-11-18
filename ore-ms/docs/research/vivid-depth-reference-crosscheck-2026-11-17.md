# ViViD++ registered-depth reference crosscheck

After the [zero-rotation result](vivid-zero-rotation-control-2026-11-17.md),
tested whether the already recorded active-depth stream supplies a stronger
motion crosscheck. The [ViViD++ paper](https://arxiv.org/html/2204.06183#S3)
documents structured-light depth indoors. Used the [ROS REP 118](https://reps.openrobotics.org/rep-0118/)
OpenNI millimeter convention and the depth stream's own CameraInfo.

Verified all 3,678 depth images across the three bags. Each is 640×480
`16UC1` in `/openni_rgb_optical_frame` with exactly matched header-time
CameraInfo. The single recorded model across all three bags has zero
distortion and focal length 570.34 px. Sufficient depth returns are present
even in the dark control. The depth and RGB share a rig and optical frame
and use separate sensor measurements.

Bounded a depth pair to each already supported RGB anchor, using nearest
depth headers within 25 ms of each RGB endpoint. The maximum observed
differences are 16.67 ms bright, 16.15 ms varying and 23.26 ms dark. Keep
the depth timestamp pair for mocap and gyro evaluations. Estimate motion
with the same fixed rigid point-to-point ICP implementation used in the
[M2P2 assay](m2p2-night-registration-and-support-2026-11-17.md), on a
systematic eight-pixel sample of 0.5–4 m depths capped at 2,500 points.
Run direct forward, reverse and checkerboard halves; run a fixed five-step
chain on the same endpoints as a smaller-motion comparison. All 233
supported RGB/depth pairs have estimates. No reference priors, parameter
changes or camera predictions entered registration.

| Recording | Mocap/gyro-bracketed depth pairs | Direct ICP within 5° of both | Five-step ICP within 5° of both | Identity within 5° of both | Direct vs five-step p95 |
|---|---:|---:|---:|---:|---:|
| Bright | 101 | 67 | 71 | 73 | 2.75° |
| Varying | 64 | 33 | 35 | 36 | 6.33° |
| Dark | 66 | 38 | 42 | 41 | 3.98° |

Across the three records, direct ICP's p95 disagreement with nominal mocap
is 9.78°–14.15°, and the chain's is 10.00°–14.41°. Forward versus inverse
reverse direct ICP differs by 3.11°–5.36° at p95 despite checkerboard
half-to-half p95 below 1.09°. Shared scene geometry can yield consistent
solutions with similar bias. The depth route does not provide a qualified
scoring reference.

Conditional diagnostic: among dark RGB proposals with both depth solutions
within 5° of the two nominal signals (31 cases), five RGB proposals differ
from both depth solutions by more than 5°. Bright and varying have zero
such disagreements among 50 and 25 conditional cases. Depth and RGB
endpoints are nearby rather than identical, and the subset conditions on
reference agreement. It is not an estimate of accuracy or error rate.

The first attempt used the ICP result key for a camera proposal and stopped
before producing an output ledger. The corrected run reads the existing
camera rotation key without changing geometry or selection. Both attempts,
protocols, the per-pair ledgers and their verified hashes are stored under
`/root/ore-ms-artifacts/vivid-plusplus-intake/`. No new bag, matcher inference
or Box transfer was needed. Further fine-pose work requires new reference
evidence or an explicitly changed criterion.
