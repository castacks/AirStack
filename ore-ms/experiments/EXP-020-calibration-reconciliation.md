# EXP-020: Thermal calibration reconciliation and rejected corrections

Date: 2026-10-20 (America/New_York). Branch `ore_ms`, base `523fec36f` plus frozen EXP-020 source. **Diagnostic completed; replacement calibration remains unresolved. Both fitted candidates fail their preregistered validation gates. Original calibration and VO defaults are unchanged.**

## What was resolved

The adapter's use of camera-info is correct: it uses the rectified projection matrix's left3x3 block for processed images. CameraInfo's K describes raw/distorted images, while P describes rectified projection; keeping raw K/D alongside R/P in processed camera-info is not itself a metadata defect. See the [official message definition](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/CameraInfo.msg). All24 fresh left/right camera-info raw K/D checks match the supplied stereo config exactly. This confirms file consistency, not physically correct intrinsics.

The supplied R/P matrices imply exactly the thermal producer's inverse transform, within floating-point tolerance. The producer labels the stored `cam1/T_cn_cnm1` as right-to-left and inverts it. [Kalibr's declared format](https://github.com/ethz-asl/kalibr/wiki/yaml-formats) defines this cam1 field as taking cam0 coordinates into cam1. The [producer code](https://github.com/castacks/ros_torch_converter/blob/main/scripts/get_thermal_data.py) therefore disagrees with the declared format. That is a concrete convention discrepancy; it does not establish whether the numeric matrix was itself stored with the wrong label.

The current [TfManager implementation](https://github.com/castacks/ros_torch_converter/blob/main/ros_torch_converter/tf_manager.py) and [demo](https://github.com/castacks/ros_torch_converter/blob/main/scripts/demo_tf_transform.py) implement T_first<-second. Our YAML composition follows that inspected convention. The pinned static-publisher implementation remains inaccessible (GitHub404); the supplied pinned Yamaha YAML is available and numerically audited. Old serialized TF folder labels are not silently reinterpreted.

Simply using the stored Kalibr direction gives97.6–100% positive triangulated depths among eligible matches on the12 fresh validation pairs, but worsens vertical alignment: medians0.711–3.152px versus0.317–1.659px for the supplied processed geometry. Reversing only translation cannot fix epipolar alignment: it preserves E up to sign and reverses triangulated depth. Thus neither a label change nor a blanket baseline sign flip supplies a validated correction.

## Stage A: free essential-pose candidate — rejected

Training pairs were fixed before outcomes: day-original and night-early-position0, with326 original fitted feature correspondences. The ten other original calibration samples were held out. All original SIFT/F-RANSAC masks were retained; candidate scoring never rematches warped images or selects candidate-specific successful matches.

Processed feature pixels were inverted through their saved R/P into raw undistorted rays. Raw intrinsics remained fixed. A single pooled essential matrix used RANSAC1.5px divided by mean raw focal length, probability.999, seed0 and5000 maximum iterations. Translation norm was fixed to the supplied stereo translation norm, not estimated as metric scale. Training-only single-pair fits tested stability.

The pooled candidate gives at least99.35% positive eligible depth in each original held-out pair, but residual improvement fails on four of ten. More decisively, the single-pair training fits disagree by **73.10 degrees in translation direction** (rotation difference0.215deg). The pooled direction differs22.53deg from the supplied as-stored stereo translation. This demonstrates why positive depths alone do not validate a free scene-based fit. The original gate is retained as failed; no substitute fit or threshold search is used.

## Stage B: fixed body centers, rotation-only fit — rejected

After Stage A failed, a narrower hypothesis was preregistered before its outcomes. The nominal camera centers from the body YAML were held fixed in the left optical frame. Only relative rotation was fitted, with t=-R*C. The same two original training pairs were used; no validation data entered fitting. SciPy least_squares used identity initialization, +/-5deg rotation-vector component bounds, soft_l1 loss/f_scale1.5px, max_nfev2000 and1e-12 tolerances. There was no alternative initialization or loss search.

Fresh validation selected positions4/8 of all six original day/night report windows, yielding12 stereo pairs and1631 fitted correspondences. These frames were held out from this calibration fitting and correspondence audit; some were already used in earlier VO experiments. They remain temporally dependent observations within the same two recordings, not independent new recording evidence. Day images were recovered by selected ZIP byte ranges/CRC checks; night images match their inherited extraction hashes. The whole source archives were not downloaded or rehashed.

The constrained fit reduces epipolar error substantially, but it cannot be accepted as a physical calibration:

| Fresh pair | Supplied vertical median px | Constrained vertical median px | Positive eligible depth, constrained |
|---|---:|---:|---:|
| day original4 | 1.119 | 0.240 | 90.9% |
| day original8 | 0.993 | 0.232 | 95.6% |
| day early4 | 0.802 | 0.242 | 95.3% |
| day early8 | 0.796 | 0.223 | 95.3% |
| day late4 | 0.992 | 0.267 | 74.5% |
| day late8 | 1.132 | 0.247 | 75.5% |
| night early4 | 0.631 | 0.339 | 95.0% |
| night early8 | 0.598 | 0.372 | 93.4% |
| night middle4 | 1.659 | 0.371 | 100% |
| night middle8 | 1.591 | 0.435 | 100% |
| night late4 | 0.325 | 0.230 | 95.8% |
| night late8 | 0.317 | 0.280 | 96.0% |

All12 fresh pairs have at least20 finite parallax-eligible matches. Eleven meet the fixed25% Sampson improvement criterion relative to the better supplied/as-stored geometry; night-late8 fails that improvement threshold. Four fail the95% positive-depth criterion. The two single-pair training rotations differ by **1.254deg**, exceeding the.25deg stability threshold. No rotation-vector bound is saturated. The derived right-body camera center is preserved to3.10e-17m, so that algebraic control passes while the calibration gate fails.

The day-only rotation vector in degrees is approximately(-0.076,+0.729,+0.142); the night-only vector is(-0.085,-0.482,-0.183). These are diagnostics from the prescribed training fits, not separately validated per-recording calibrations or evidence that the hardware moved. Scene geometry, imperfect correspondences, source intrinsics/calibration applicability and acquisition effects remain possible explanations. No per-recording correction is selected from these results.

The primary residual is Sampson distance in fixed undistorted raw pixel coordinates. Candidate-specific rectification changes R/P and focal scaling, so rectified vertical residual is supplementary. Eligible cheirality uses finite triangulation and at least0.1deg ray parallax; no candidate's negative-depth points are dropped from that denominator. Existing F-RANSAC inliers are observations, not verified physical ground truth. Secondary EXP-019 model-grid candidate diagnostics are retained for Stage A and never used as fitting data or independent truth.

## Concrete artifacts and status

- `results/calibration-reconcile/run-001/candidate-calibration.yaml`: free-pose proposal, explicitly `experimental_image_gate_failed`.
- `results/calibration-reconcile/rotation-run-001/candidate-calibration.yaml`: body-center-constrained proposal, explicitly `experimental_image_gate_failed`.
- `results/calibration-reconcile/source-consistent-calibration.yaml`: nominal direct Kalibr interpretation, K/D/R/P and right-body pose derived from the provisional left-body anchor. Explicitly `SOURCE_CONVENTION_ONLY_NOT_VALIDATED_FOR_VO`; not an independently calibrated transform.

Both fitted attempts retain all observations, source candidates, masks,3D points, residuals, gate checks and original test splits. Six fixed raw stereo pairs are re-rectified into each attempt's separate `preview/` directory for inspection; original images and matrices are never overwritten. These previews are diagnostic outputs of rejected candidates, not replacement inputs for the baseline.

## Validation and recovery

20 numeric/regression tests pass, including synthetic known pose/center recovery, rectification-ray inversion, source-P reconstruction, E sign/scale invariance, cheirality, nearzero parallax, empty inputs and rejection-gate failures. The initial synthetic fixture omitted distortion coefficients; this setup failure was fixed before image analysis and its log is retained. All228 Stage A and269 Stage B frozen file hashes pass before/after execution and independent validation.

An independent implementation checks24 pair sets and20,513 candidate-point scores: original rectification inversion, OpenCV's separate scalar Sampson calculation, NumPy SVD triangulation, cheirality, atan2 parallax, source intrinsics and body-center preservation. These numerical checks pass; both scientific calibration gates fail. Successful numerical validation must not be reported as successful calibration. Figures visually inspected.

Runtime is the existing CPU `macvo:geometry`, OpenCV4.10.0.84/NumPy2.2.6, with exact image ID, package inventory, commands and source-reference manifests in staging. No new GPU inference or VO run occurred. Box snapshot `backups/2026-10-20-exp-020` preserves new validation images, both attempts, all arrays/previews, source/tests/reports and notebook evidence. It excludes credentials, checkpoint files, disposable ZIP range cache blocks and full dataset archives. Existing EXP-017/018/019 inputs and original calibration/source recovery remain prerequisites.

## What is needed to resolve the remaining calibration

Camera serial numbers are not required. A raw thermal stereo calibration-target sequence with known target dimensions and varied target positions/orientations would constrain intrinsics and relative pose more directly. Alternatively, an independently measured stereo calibration applicable to these recordings could be checked against these preserved observations. The user has been asked for either source. Existing camera-info and scene images support the audits above but have not yielded a stable, physically validated replacement. Until that evidence is available, retain provisional geometry and do not claim calibrated metric depth, cross-modal covariance calibration or validated fusion.
