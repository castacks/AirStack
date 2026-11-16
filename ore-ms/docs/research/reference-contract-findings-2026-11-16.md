# Camera/reference audit and a usable scalar motion measurement

16 November 2026. Completed [source audit and angular-motion diagnostic](reference-contract-protocol-2026-11-16.md).
No new inference, clock fit, hand-eye estimate, axis permutation or model change.
This is characterization of exposed cached predictions, not a blind validation or
full MAC-VO/MACThermal reproduction.

Follow-up completed: [SmokeRoom raw bindings and nominal IMU-frame pose errors](smoke-reference-pose-findings-2026-11-16.md).
This uses a different declared source convention and gyro consistency; it does not
repair or replace NGCLAB's still-missing tracked-body transform.

## What the published calibration establishes

The [official calibration release](https://github.com/NGCLAB/multi-spectral-dataset/blob/master/calibration.md)
links a 9-page camera–IMU report and raw calibration captures. Downloaded the
13,563,772-byte report and three raw ZIPs totaling 168,807,569 bytes; all ZIP-member
CRCs pass. The camera report's four intrinsic values exactly match our RGB input
calibration. Its camera→IMU and IMU→camera transforms are explicitly named. It
does not name a transform from the tracked OptiTrack rigid body to either sensor.

A bounded byte-range probe of the actual 8,388,079,380-byte IMU-calibration bag
read the file header and complete connection/chunk index: 9,073 RGB images and
60,974 IMU messages, with only `/camera/image_raw` and `/imu/data` connections.
No mocap poses are in that index. The complete large bag was not downloaded or
pixel-validated. The raw visible–thermal and visible–Kinect archives contain
board captures; Kinect metadata contains its own intrinsics/extrinsics. Their
`calib_pose.yaml` filenames do not establish a camera-to-mocap transform.

The Kalibr report specifies `T_ci` / `T_ic` and a camera-to-IMU time shift of
−0.176167632455 seconds in its September 2020 capture. The tested desk bags are
January 2020. That report does not establish a camera–mocap offset, and its internal
`T_i_b = I` does not identify the Kalibr body with the OptiTrack rigid body.
Neither the transform nor that shift was applied to the evaluated poses.

The author extractor at `6d4bf30ac28d69e9e28f8f634185e0829f39ab9b` writes raw
`/vrpn_client_node/RigidBody/pose` to camera ground truth without an extra transform.
Its README describes camera poses, but the inspected release still does not
independently establish the necessary tracked-body→camera chain for our use.
This is a bounded source-qualification gap, not proof the author's data is wrong.
The prior public issue about world axes does not itself establish the missing edge.

**Clarification:** a fixed choice of world axes cancels in relative pose. If
`T_WC = T_WB T_BC`, then camera-relative motion is
`inverse(T_BC) inverse(T_WB1) T_WB0 T_BC`. The unknown body→camera transform matters;
a common world-frame transform does not. Do not search world-axis flips to rescue
these cached errors or assume that body and IMU frames coincide.

The initial PDF text extractor hit its content-stream limit; PyMuPDF successfully
extracted the report. Original PDF, parser receipt, text, archive indexes, all
payload hashes and the calibration-bag raw index bytes are retained locally.
No author contact or authenticated-sharing change was performed.

## A narrower reference-backed endpoint

The paper places cameras and tracked markers on one rigid rack. For a fixed
body→camera rotation X, relative rotations obey `R_C = X^T R_B X`. Their trace and
rotation angle are identical. Thus **the amount of rotation** can be compared to
raw mocap without guessing X or the body/camera origin. A nonzero camera lever arm
also leaves this rotational identity unchanged.

The diagnostic is `abs(angle(R_estimated) − angle(R_body_reference))`, in degrees,
at each sensor's own nominal image-header times. It does not validate rotation
axis/sign, translation or a trajectory. Rotations about different axes can have
identical angles. Scalar success ≤1° is only a necessary condition for full
orientation error ≤1°, not a sufficient one; its failure is informative while
its success leaves axis error open. Timing/metrology assumptions remain qualified.

Use the original common reference-supported sets: 121 bright, 122 changing,
127 dim intervals. All 1,684 original frontend/sensor rows remain in the score
file; 1,480 rows have common reference support. Reference gaps stay separate.
Rejected/missing cached poses remain failures; no finite-R rescue from rejected
estimates. No camera–IMU time shift or newly fitted offset is used.

## Learned angular-motion amount results

Success is a returned pose with absolute rotation-amount error ≤1°, over every
common-reference interval. Conditional medians use returned poses only; the
success rates include missing estimates. They must be read together.

| Lighting | RGB success | Thermal success | Conditional median error, RGB / thermal |
|---|---:|---:|---:|
| Bright | 95/121 (78.5%) | 73/121 (60.3%) | 0.236° / 0.545° |
| Changing | 85/122 (69.7%) | 61/122 (50.0%) | 0.303° / 0.738° |
| Dim | 94/127 (74.0%) | 83/127 (65.4%) | 0.246° / 0.552° |

Thermal returns more poses in each cohort (104/100/109 versus RGB 98/89/99), but
more of those returned poses have incorrect angular amount. This supplies a
reference-backed counterexample to equating frontend availability with accuracy.
It does not rank full orientation or establish intrinsic sensor superiority:
learned model training/domain, image preprocessing, fields of view and actual
remaining RGB signal differ. This dim cohort is not measured zero lux.

For the separately declared reference-angle ≥2° strata, learned RGB succeeds on
75/91 bright, 60/80 changing and 83/105 dim intervals; thermal succeeds on 55/91,
40/80 and 70/105. The always-available zero-motion predictor succeeds on none of
these intervals, so the ranking is not produced by returning identity on nearly
stationary views. Small-motion strata are also saved and retained.

## Thermal still helps a weak RGB frontend in dim light

| SIFT angular amount | RGB success | Thermal success |
|---|---:|---:|
| Bright | 62/121 (51.2%) | 19/121 (15.7%) |
| Changing | 46/122 (37.7%) | 8/122 (6.6%) |
| Dim | 1/127 (0.8%) | 24/127 (18.9%) |

For ≥2° dim reference motion, SIFT RGB succeeds 1/105 and thermal 23/105.
Thermal's dim benefit therefore survives this limited reference measurement with
SIFT, while the stronger learned RGB baseline changes the comparison. The simple
zero-motion predictor succeeds on 5/127 dim intervals; preserve that control and
all missing outputs. This does not make 18.9% sufficient for dependable deployment.

## Decision, verification and next task

Keep full orientation/translation claims unqualified until the missing frame chain
is independently documented. This bounded NGCLAB release/calibration audit is
closed absent new source material; do not reread the same folders or substitute
an IMU frame. No author request is necessary to use the limited scalar endpoint.

Next qualify the released SmokeRoom odometry's actual parent/child sensor frames
and timing against its camera-to-IMU calibration, using source/raw message metadata.
Its supplied fused reference is a different contract from mocap and includes
visual/LiDAR/inertial information; do not call it pure independent optical truth.
Proceed to its pose-error evaluation only if that explicit chain is established.
This targets the real-smoke accuracy question instead of adding another support
proxy or retuning exposed models.

Synthetic qualification passed 100 random rotation-conjugacy/world-transform cases
(maximum scalar difference 4.3e-14°), quaternion-sign equivalence and a wrong-axis,
same-angle counterexample. An independent auditor reinterpolates raw mocap
quaternions and uses quaternion-dot and trace formulas, without importing the
scorer: 12,084 checks passed. Both scalar results and prior failed full-pose
results remain; none were overwritten. No inference, training or fitting occurred.

Evidence: ignored `results/reference-contract/`, including source/calibration
receipts, bounded bag metadata, frozen protocol/scorer, raw scalar scores,
common/motion strata and independent audit. The local incremental recovery packet
is `/root/ore-ms-artifacts/reference-contract-2026-11-16/`; cached parent inputs
are hash-bound and included. No Box upload or Git publication occurred.
