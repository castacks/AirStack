# EXP-062: Prediction-based geometry in recording-excluded camera selection

Date: 2026-10-27 America/New_York. Branch `ore_ms`; starting commit `b67bb783b`.

**Finding: prediction-based geometry helps the excluded night window, but does not give consistent gains across recordings.** The combined selector beats uncertainty-only, geometry-only, both fixed cameras, original uncertainty, camera-scale, EXP060 scalar-fit and random in every night frame under both losses/all anchors. On day it loses to uncertainty-only in all20 mean-distance cases and19/20 squared cases. Rain retains severe failures against thermal/original uncertainty. No model/default is adopted.

## Declared model comparison

Before fitting, freeze four families, the target, feature definitions, three recording-exclusion folds and regularization in `results/ms2-geometry-risk-selection/design-declaration.md` (SHA256 `ca8e50bd2ad525010391397871c4e102ec6a28927efa511e2fa091064b995a97`). All windows were already inspected in EXP056–061; this is exploratory recording-excluded evidence, not untouched validation or network-training-held-out evaluation.

| Family | Slope inputs | Parameters including intercept |
|---|---|---:|
| Constant | None |1|
| Uncertainty-only | Natural logs of original RGB and thermal propagated metric variance |3|
| Geometry-only | Natural logs of both predicted Euclidean ranges in shared RGB coordinates; RGB x/1224,y/384 |5|
| Combined | All six inputs above |7|

Predicted3D points, original metric variances and native candidate coordinates supply all numerical features. Reference points/ranges, measured errors, reference-tail masks and illumination labels are not inference-time feature inputs. The pure feature extractor accepts only predictions, variances and RGB coordinates. Nonetheless the original candidate population/correspondences are reference-selected, so this is not a deployable matching/availability pipeline. Uncertainty already contains geometry; extra-input gains do not establish independent information or physical cause. Geometry-only also includes image position, so this experiment does not isolate range from position.

## Objective and recording exclusion

Exclude one whole recording, train on the other two (20frames each). Each training pair’s base weight `b=1/(2*20*n_frame)` gives equal recording/frame mass. Standardize each input with training-only base-weighted mean/STD; replace STD<1e-12 by1. No excluded values enter fits or scalers.

For midpoint training errors define `y=sign(E_T-E_R)` and `w=b*abs(E_T-E_R)/R_random(frame)`, with `R_random=mean((E_R+E_T)/2)`. Fit the cost-sensitive least-squares surrogate

```text
min_theta sum_i w_i * (x_i theta - y_i)^2 + 0.01 * ||theta_slopes||²
```

The intercept is unpenalized. Use float64 normal equations; no hyperparameter selection, clipping, interaction expansion or family selection. RGB is selected at positive margin, thermal at negative margin, exactzero is an equal mixture. Error ties have zero training cost. All-zero-cost synthetic training takes the zero coefficient minimizer. Margins are decision scores, not calibrated probabilities, expected risks or full covariance estimates. This objective is not NLL calibration from the earlier reading note.

Freeze fits before evaluating the excluded recording on both mean-L2/MSE and midpoint/RGB/thermal anchors. The development-selected fixed camera uses the same per-image random-normalized mean-distance objective as EXP060. Preserve all fixed-camera/random/original/camera-scale/scalar-fit controls. Original both-valid masks/predictions/errors and all20 IDs are unchanged; rain’s200 thermal-only candidates remain counted but do not enter this both-valid model comparison. No new frontend inference, network training, physical calibration, time adjustment, VO or fusion. This is a new fitted selector, not training of the original stereo networks.

## Primary comparison: combined versus uncertainty-only

Entries are median paired per-image risk ratios with the number below1. Below1 favors combined. Frames, pixels, anchors and losses repeat dependent observations.

| Excluded recording | Development recordings | Mean-distance ratio (below1) | Squared-error ratio (below1) |
|---|---|---:|---:|
| rain | day, night | 0.9810 (15/20) | 0.9990 (15/20) |
| day | rain, night | 1.1341 (0/20) | 1.2379 (1/20) |
| night | rain, day | 0.9162 (20/20) | 0.8974 (20/20) |

Night’s combined mean-distance risk is8.4% lower than uncertainty-only and5.3% lower than geometry-only at the median paired image. Its mean-distance/RGB ratio0.9162 and squared/RGB0.8981 are below1 in all20 frames. This is a conditional gain for this recording, not a universal method. Day reverses the incremental conclusion: adding these inputs worsens uncertainty-only by13.4% median mean-distance risk; geometry-only also beats combined in17/20 mean-distance cases. Rain’s modest median combined/uncertainty improvement does not resolve fixed-camera failure.

## All families and main midpoint controls

Each entry: median per-image model/comparator ratio (number below1 out of20). No pixel pooling or outcome-selected comparator. Training-fixed is identical to the fitted constant family on these data. EXP060 scalar-fit is the earlier model fitted with the same excluded-recording split.

| Excluded | Loss | Family | Always RGB | Always thermal | Random | Original learned | Camera-scale | Scalar-fit |
|---|---|---|---:|---:|---:|---:|---:|---:|
| rain | mean_l2 | constant | 1.0000 (0/20) | 4.0790 (0/20) | 1.5948 (0/20) | 5.4628 (0/20) | 5.6079 (0/20) | 3.5762 (0/20) |
| rain | mean_l2 | uncertainty | 0.9179 (12/20) | 2.5425 (0/20) | 1.2230 (4/20) | 3.5829 (0/20) | 3.5419 (0/20) | 2.4709 (0/20) |
| rain | mean_l2 | geometry | 0.9896 (19/20) | 4.0543 (0/20) | 1.5842 (0/20) | 5.4383 (0/20) | 5.5743 (0/20) | 3.5598 (0/20) |
| rain | mean_l2 | combined | 0.9810 (20/20) | 3.4941 (0/20) | 1.3344 (2/20) | 4.6426 (0/20) | 4.5530 (0/20) | 3.2474 (0/20) |
| rain | mse | constant | 1.0000 (0/20) | 211.0737 (0/20) | 1.9904 (0/20) | 329.8739 (0/20) | 330.4453 (0/20) | 50.0756 (0/20) |
| rain | mse | uncertainty | 0.9153 (12/20) | 170.8691 (0/20) | 1.7810 (4/20) | 238.5378 (0/20) | 236.4412 (0/20) | 26.2224 (0/20) |
| rain | mse | geometry | 0.9998 (20/20) | 210.8380 (0/20) | 1.9884 (0/20) | 329.5237 (0/20) | 330.0991 (0/20) | 50.0339 (0/20) |
| rain | mse | combined | 0.9990 (20/20) | 199.1815 (0/20) | 1.9571 (1/20) | 314.2699 (0/20) | 315.0716 (0/20) | 48.3661 (0/20) |
| day | mean_l2 | constant | 1.6840 (0/20) | 1.0000 (0/20) | 1.2548 (0/20) | 1.4458 (0/20) | 1.3203 (0/20) | 1.7326 (0/20) |
| day | mean_l2 | uncertainty | 1.0146 (7/20) | 0.6050 (20/20) | 0.7580 (20/20) | 0.8859 (20/20) | 0.7951 (20/20) | 1.0252 (3/20) |
| day | mean_l2 | geometry | 1.1162 (2/20) | 0.6627 (20/20) | 0.8312 (20/20) | 0.9591 (16/20) | 0.8667 (20/20) | 1.1204 (0/20) |
| day | mean_l2 | combined | 1.1524 (0/20) | 0.6946 (20/20) | 0.8568 (20/20) | 0.9931 (10/20) | 0.9181 (18/20) | 1.1728 (0/20) |
| day | mse | constant | 2.2378 (2/20) | 1.0000 (0/20) | 1.3819 (2/20) | 1.7323 (0/20) | 1.2504 (0/20) | 2.6638 (0/20) |
| day | mse | uncertainty | 0.9503 (12/20) | 0.4154 (20/20) | 0.5626 (20/20) | 0.6987 (20/20) | 0.5230 (20/20) | 1.0436 (3/20) |
| day | mse | geometry | 1.1079 (7/20) | 0.4402 (20/20) | 0.6298 (20/20) | 0.7693 (19/20) | 0.5518 (20/20) | 1.1364 (0/20) |
| day | mse | combined | 1.1300 (0/20) | 0.4979 (18/20) | 0.7028 (18/20) | 0.9082 (12/20) | 0.6073 (16/20) | 1.2504 (0/20) |
| night | mean_l2 | constant | 1.2328 (0/20) | 1.0000 (0/20) | 1.1043 (0/20) | 0.9842 (17/20) | 0.9664 (20/20) | 1.0358 (2/20) |
| night | mean_l2 | uncertainty | 1.0000 (6/20) | 0.8111 (20/20) | 0.8957 (20/20) | 0.7982 (20/20) | 0.7841 (20/20) | 0.8411 (20/20) |
| night | mean_l2 | geometry | 0.9722 (19/20) | 0.7900 (20/20) | 0.8692 (20/20) | 0.7801 (20/20) | 0.7644 (20/20) | 0.8226 (20/20) |
| night | mean_l2 | combined | 0.9162 (20/20) | 0.7451 (20/20) | 0.8205 (20/20) | 0.7381 (20/20) | 0.7213 (20/20) | 0.7782 (20/20) |
| night | mse | constant | 1.5628 (0/20) | 1.0000 (0/20) | 1.2196 (0/20) | 0.9487 (19/20) | 0.9298 (20/20) | 1.0328 (3/20) |
| night | mse | uncertainty | 1.0000 (9/20) | 0.6414 (20/20) | 0.7816 (20/20) | 0.6050 (20/20) | 0.5876 (20/20) | 0.6651 (20/20) |
| night | mse | geometry | 0.9673 (19/20) | 0.6200 (20/20) | 0.7505 (20/20) | 0.5877 (20/20) | 0.5711 (20/20) | 0.6378 (20/20) |
| night | mse | combined | 0.8981 (20/20) | 0.5745 (20/20) | 0.6964 (20/20) | 0.5458 (20/20) | 0.5305 (20/20) | 0.5978 (20/20) |

Rain combined/original risk is4.643 median mean-distance and314.270 squared, worse in all20 frames under both losses. Combined/always-thermal is3.494/199.181, likewise worse in all20. Keeping these failures is essential: a favorable relative gain over a poor learned comparator cannot validate a useful selector. Day combined loses to always RGB in all20 frames under both losses, despite beating random in all20 mean-distance cases.

![All families and losses](../results/ms2-geometry-risk-selection/figures/geometry-risk-selection.png)

## Endpoint sensitivities

All endpoint decisions reuse the same fitted model; anchor changes only the measured error. Entries are combined/comparator median ratios (below1 counts). Full four-family endpoint/per-image results and oracle gaps remain in JSON.

| Excluded | Anchor | Loss | Uncertainty-only | Geometry-only | Always RGB | Always thermal | Random |
|---|---|---|---:|---:|---:|---:|---:|
| rain | rgb | mean_l2 | 0.9809 (15/20) | 0.9964 (15/20) | 0.9809 (20/20) | 3.4941 (0/20) | 1.3348 (2/20) |
| rain | rgb | mse | 0.9990 (15/20) | 0.9998 (17/20) | 0.9990 (20/20) | 198.8238 (0/20) | 1.9571 (1/20) |
| rain | thr | mean_l2 | 0.9812 (15/20) | 0.9960 (14/20) | 0.9812 (20/20) | 3.4940 (0/20) | 1.3341 (2/20) |
| rain | thr | mse | 0.9990 (15/20) | 0.9998 (17/20) | 0.9990 (20/20) | 199.4992 (0/20) | 1.9571 (1/20) |
| day | rgb | mean_l2 | 1.1348 (0/20) | 1.0242 (2/20) | 1.1504 (0/20) | 0.6948 (20/20) | 0.8579 (20/20) |
| day | rgb | mse | 1.2375 (1/20) | 1.0494 (4/20) | 1.1294 (0/20) | 0.4980 (18/20) | 0.7032 (18/20) |
| day | thr | mean_l2 | 1.1333 (0/20) | 1.0244 (3/20) | 1.1533 (0/20) | 0.6943 (20/20) | 0.8558 (20/20) |
| day | thr | mse | 1.2397 (1/20) | 1.0480 (4/20) | 1.1305 (0/20) | 0.4979 (18/20) | 0.7025 (18/20) |
| night | rgb | mean_l2 | 0.9162 (20/20) | 0.9467 (20/20) | 0.9162 (20/20) | 0.7450 (20/20) | 0.8205 (20/20) |
| night | rgb | mse | 0.8974 (20/20) | 0.9275 (20/20) | 0.8981 (20/20) | 0.5745 (20/20) | 0.6964 (20/20) |
| night | thr | mean_l2 | 0.9161 (20/20) | 0.9467 (20/20) | 0.9162 (20/20) | 0.7451 (20/20) | 0.8205 (20/20) |
| night | thr | mse | 0.8974 (20/20) | 0.9275 (20/20) | 0.8981 (20/20) | 0.5746 (20/20) | 0.6964 (20/20) |

## Validation and reproducibility

Independent audit PASS: 3,213,914 recorded checks: 128 hashes, 800,646 feature values, 800,646 reconstructed errors, 174 fit fields, 533,764 margin values, 1,067,528 decision values, 4,980 risk fields, 2,880 historical fields, 3,168 summary fields. The independent validator imports no fitting code. It reconstructs numerical features and3D errors from original saved arrays, fits weighted augmented least squares with an alternate solver, checks all coefficients/scalers/training objectives, reconstructs test decisions under both solvers, and recomputes all risks/ratios/aggregates. All decisions agree with the alternate fit; smallest absolute real-data margin exceeds4e-6. Check counts repeat dependent values, not independent observations.

All128 original support/metric/report and EXP060 protocol/report hashes match verified recovery manifests and remain unchanged. The independent historical-risk checks reproduce EXP060 scalar-fit and original controls. Synthetic tests verify prediction-only feature interface, coordinate normalization, range/variance rescaling, zero-margin mixtures, constant-feature scaling, zero-cost ties and invariance to mutation of a completely excluded recording.

Final artifacts are `results/ms2-geometry-risk-selection/run-002/{protocol.json,report.json,<recording>/selection-*.npz}`, `validation-final.json`, `synthetic-validation.json` and PNG/PDF. Saved NPZs contain all six features, four model margins and choices. Reports retain coefficients, normalizers, training objectives/weights and original availability counters. `run-001` is preserved; `run-002` only adds defined all-zero-cost synthetic handling. Both real-data reports are byte-identical; no policy/hyperparameter choices changed. Earlier validation files are retained with their own validator hashes.

CPU Python3.12.3 / NumPy2.2.6, `OPENBLAS_NUM_THREADS=1`; Matplotlib3.10.7 for figures. Fit needs original EXP057/058 scientific inputs and EXP060 protocol/report plus their recovery manifests. No full data archives, checkpoints or GPU. Use fresh output paths:

```bash
OPENBLAS_NUM_THREADS=1 python MAC-VO-main/Scripts/Setup/ms2_geometry_risk_selection.py --root /root/AirStack/ore-ms --recovery /root/ore-ms-recovery --output /path/to/fresh-run
OPENBLAS_NUM_THREADS=1 python MAC-VO-main/Scripts/Setup/validate_ms2_geometry_risk_selection.py --root /root/AirStack/ore-ms --source /path/to/fresh-run --output /path/to/fresh-validation.json
python MAC-VO-main/Scripts/Setup/plot_ms2_geometry_risk_selection.py --source /path/to/fresh-run --output /path/to/fresh-figures
```

## Research decision

The predeclared small-model test is complete. Prediction-based geometry/position can help excluded-night selection beyond either family alone on this conditional benchmark. The day and rain failures prevent adopting a general RGB–thermal reliability model. These results do not establish calibration, physical correspondence, independent-reference accuracy, unseen-network inputs or VO improvement. No uncertainty/fusion defaults change.

Before adding capacity or selecting a favorable variant, the next necessary evidence is performance of a frozen method on genuinely unexamined recordings/reference conditions with camera-scale and both fixed-camera controls. Check existing Box/public MS² coverage before requesting data; unused intervals of existing recordings can test interval stability but do not create independent captures. Additional independent recordings with cross-camera error variation, or a usable CATS reference sample, would materially strengthen a method claim. Current CATS access and checkpoint training membership remain unresolved in the handoff; no upload or author reply was required for this bounded study.

Recovery target: Box `ore-ms/backups/2026-10-27-exp-062`; new code/notes/results/figures and pre-fit declaration/notebook, incremental Git bundle. Input prerequisites: EXP057/058/060; full original datasets/checkpoints/private credentials excluded.
