# EXP-056: Fixed cross-camera support and conditional metric reliability pilot

Date: 2026-10-26 America/New_York. Branch `ore_ms`; starting commit `609ea6f80`.

**Main finding: image-wide camera-scale control largely matches pointwise learned uncertainty.** The full learned rule versus that control has median risk ratios 0.988 under squared loss (11/20 frames improve) and1.002 under mean 3D distance (9/20 improve). This pilot does not establish a consistent additional benefit from within-image variance variation.

On frozen candidate correspondences in the existing MS² rain window, lower propagated learned depth uncertainty selects lower supplied-reference 3D risk than fixed-camera and equal-angular-noise geometry controls. Against the geometry-only control, the median paired reduction is 14.5% for mean 3D distance and 19.8% for squared 3D error; all 20 frames improve under both losses. This is a conditional benchmark on sparse candidate support, not proof of exact shared physical points, numerical covariance calibration, training-held-out performance or improved VO/fusion.

## Frozen support protocol

Use the same EXP054/055 rain sequence `_2021-08-06-16-19-00`, all 20 IDs 000000–000019, original calibration and primary single-scan depth. No new images, inference, training, alignment fit, clock correction or source-ID substitution. The geometry audit had already been inspected; this is an exploratory follow-up rather than an untouched test sample. Model winner scores were inspected only after support and comparison rules were recorded.

1. Backproject every nonzero depth pixel and apply the documented static rig transform in both directions. Preserve positive-Z and in-FOV counts.
2. Query the nearest supported target label in normalized image coordinates. Use the same angular-coordinate radius `1/fx_RGB` in both directions (about 0.001308), equivalent to 1 RGB pixel or 0.507 thermal pixels here. Exclude nearest-neighbor distance ties within1e-12 normalized units.
3. Require both observed source points to be uniquely frontmost in their respective rounded target-pixel z-buffers, using all available positive/in-FOV source depths. Exclude exact-depth front ties. This tests visibility among observed sparse samples, not all real surfaces.
4. Require mutual nearest-label identity and the angular bound in both directions. Retain each pair once, canonically indexed by RGB support. No reference-depth-agreement threshold defines the primary support.

Retained support is 52 / 1221.5 / 2429 pairs per frame (min/median/max),25,754 pairs in total. These are dependent scene samples, not independent trials. Both original sparse supports and every sequential rejection count remain in the outputs. The median of per-image median 3D reference-point separations is 0.0608 m; its range is 0.0084–0.6333 m. ID000001 has only52 pairs and0.6333 m median separation; it is retained.

Both-way depth-consistency sensitivities use2/256 m alone, and max(2/256 m,1%,2%,5% targetZ). The median retained candidate fractions are1.7%,97.6%,~100%,100%, respectively. These tolerances are diagnostics, not calibrated reference-error bounds; none is chosen as the most favorable physical acceptance rule. Original left-camera time separations and failed/undefined comparisons remain visible. The two PNG products do not preserve LiDAR point IDs or independently prove that the same physical surface generated both labels.

## Fixed visual check and decision

Inspect the preregistered IDs 000000/000009/000019, displaying candidates at uniformly spaced positions in sorted RGB support rather than chosen by errors. The overlays and paired patches show plausible alignment of poles, road markings and railings, alongside textureless road patches, different viewpoints and moving vehicles. No dynamic-object mask was applied. This is qualitative plausibility, not an annotation of exact cross-modal correspondences.

The decision saved in `run-001/inspection-decision.md`, before model-winner scoring, permits a **conditional candidate-support benchmark**. No threshold was tuned from winner outcomes. Do not turn the visual check or sparse z-buffer into a physical shared-point certification.

![Fixed candidate overlay](../results/ms2-cross-modal-support/run-001/overlay-000009.png)

## Common target, uncertainty and availability

Express both candidate reference points in the nominal RGB-left rig frame. Freeze their midpoint as the primary common target and both endpoint anchors as separate sensitivities. The same chosen target is used for both predictions; no anchor is picked from model errors. A static rig transformation does not compensate ego-motion or dynamic objects at different acquisition times. Thus this is the supplied-pair candidate protocol, not exact shared-world-point motion alignment.

Read original saved stereo predictions at each camera's own candidate pixel, with original intrinsics/baseline, native input recipe and checkpoint outputs. For ray q and disparity d=-flow_u:

```
Zhat = fx B / d
Xhat = q Zhat
trace(C_X) ≈ ||q||² (fx B / d²)² Var(flow_u)
```

Rotate the thermal point/covariance into the nominal RGB frame. Rigid rotations preserve covariance trace. Choose the smaller propagated trace; exact selector ties use equal mixture. This is a rank-one depth-only first-order propagation with fixed ray/calibration, not full 3D covariance or a calibrated confidence interval. Vertical flow remains separate; it is not added to depth uncertainty. Near-zero positive disparity is ill-conditioned but is not clipped or trimmed away.

Require finite flow/covariance channels, positive variances and horizontal disparity, and finite reconstructed outputs. Preserve all 25,754 reference candidates:25,554 have both predictions valid,200 have thermal only, zero RGB-only and zero neither. Primary paired risks use the25,554 both-valid candidates, with availability counts reported for every frame. One-valid cases are not silently counted as successful paired comparisons. No full-support deployed fallback performance is claimed.

Primary loss is mean squared Euclidean3D point error; mean Euclidean distance is retained as the less tail-sensitive loss. Compute per-image paired risks and ratios, then min/median/max over20 adjacent frames. No pooled-pixel significance, p-values or inference of 20 independent recordings.

## Necessary geometry control

After the first conditional metric results, but before computing these control outcomes, freeze a follow-up using exactly the same support, predictions, references, losses and availability. Replace the learned horizontal **angular** variance by one identical positive constant in both cameras. Propagation gives:

```
geometry-only trace ∝ ||q||² Zhat⁴ / B²
```

The constant cancels in camera ranking. This isolates a fixed angular-noise propagation rule from the learned variance multiplier; it does not eliminate every geometric confounder. Keep equal native-pixel variance as a different sensitivity because camera resolutions/focal lengths differ. Also retain a comparator that chooses the smaller learned horizontal angular variance without metric propagation. No comparator is tuned or adopted as a default from this pilot.

## Camera-scale control changes the interpretation

The learned rule chooses RGB far more often than equal-angular geometry (per-image median 90.6% versus49.3%). Before reading this control's outcomes, freeze one additional comparator: replace every point's learned horizontal angular variance by that camera's **per-image median** on the same both-valid candidate population, then propagate through each point's predicted geometry. This preserves image-wide camera uncertainty scale while removing within-image variance variation. No errors are used to compute the scale; the support itself is still reference-selected, so this is not a complete deployment evaluation or an independently fitted calibration.

The full pointwise rule does not consistently improve on this comparator: median learned/control risk 0.988 under squared loss (11/20 below1) and1.002 under mean 3D distance (9/20 below1). RGB/thermal endpoint anchors retain near-parity median mean-distance ratios 1.0015/1.0021. Hence the earlier improvement against equal-angular geometry is insufficient evidence for an advantage from spatially varying learned uncertainty. Per-camera, per-image scale differences are a necessary baseline in subsequent cross-modal studies. No fixed global camera scale was learned or proven transferable.

This is a sequential exploratory control prompted by inspected selection fractions; support, target, losses and learned rule remain unchanged. It is not a preregistered untouched-dataset confirmation. The pointwise-versus-scale result, rather than the larger random-baseline gain, determines the next research step.

## Risk comparisons on midpoint reference

Every entry is a per-image paired learned-selector/comparator ratio summarized over20 frames. Below1 favors propagated learned uncertainty. Mean3D-distance risk has units metres; squared 3D risk has units m²; ratios are dimensionless.

| Loss | Comparator | Ratio min / median / max | Learned lower risk |
|---|---|---:|---:|
| Squared 3D error | Always RGB | 2.214e-06 / 0.00304 / 0.11 | 20/20 |
| Squared 3D error | Always thermal | 0.4571 / 0.6858 / 0.9421 | 20/20 |
| Squared 3D error | Equal random expectation | 4.428e-06 / 0.006048 / 0.1869 | 20/20 |
| Squared 3D error | Image-wide per-camera median variance | 0.5359 / 0.9883 / 1.318 | 11/20 |
| Squared 3D error | Fixed angular variance | 0.6347 / 0.8024 / 0.9699 | 20/20 |
| Squared 3D error | Fixed native-pixel variance | 0.7575 / 0.93 / 1.028 | 17/20 |
| Squared 3D error | Learned angular variance without depth propagation | 2.214e-06 / 0.003178 / 0.1102 | 20/20 |
| Mean 3D distance | Always RGB | 0.01832 / 0.1902 / 0.5819 | 20/20 |
| Mean 3D distance | Always thermal | 0.6831 / 0.7833 / 0.986 | 20/20 |
| Mean 3D distance | Equal random expectation | 0.03572 / 0.3061 / 0.6559 | 20/20 |
| Mean 3D distance | Image-wide per-camera median variance | 0.9187 / 1.002 / 1.114 | 9/20 |
| Mean 3D distance | Fixed angular variance | 0.7737 / 0.8555 / 0.9714 | 20/20 |
| Mean 3D distance | Fixed native-pixel variance | 0.834 / 0.9234 / 1.03 | 19/20 |
| Mean 3D distance | Learned angular variance without depth propagation | 0.01845 / 0.2057 / 0.6005 | 20/20 |

The learned selector beats fixed-angular geometry under both losses in 20/20 frames, but is not uniformly best across sensitivities: fixed-pixel geometry beats it in3/20 frames under squared loss and1/20 under mean 3D distance. Preserve those counterexamples. Always-RGB and random squared risks are particularly affected by large reconstructed errors; tiny positive disparity can create very large3D predictions. Huge squared-risk reductions are not evidence of correspondingly precise depth.

Median per-image mean 3D distances are20.41 m alwaysRGB,4.95 m alwaysthermal and3.99 m learned selector. These are supplied-reference errors on this selected support, not ground-truth accuracy certificates. The selector chooses RGB for a median 90.6% of candidates but chooses the lower-error modality only55.2% of the time (per-image median; range32.0–60.7%). AlwaysRGB's lower-error fraction has median 52.3%. Reducing average risk by avoiding costly mistakes is different from reliably predicting every point's modality winner.

![Support and metric pilot summary](../results/ms2-cross-modal-support/cross-modal-summary-with-scale.png)

## Reference/support sensitivity

RGB versus thermal endpoint anchors change284/25,554 pointwise winner labels (1.11%). Learned/fixed-angular median risk ratios remain0.8026/0.8021 under squared loss and0.8561/0.8549 under mean 3D distance; each remains below1 in 20/20 frames. Endpoint robustness strengthens this scoped result but does not validate the underlying correspondence or independent truth.

The2/256 m-only consistency sensitivity leaves nonempty both-valid support in19 frames and improves over equalrandom mean 3D-distance risk in17/19. Do not hide the missing frame or the two counterexamples. The1/2/5% relative sensitivities improve over random in 20/20 each; their median ratios are0.287/0.301/0.306. These subsets have different scene populations. None replaces primary support or supplies an externally validated depth tolerance.

## Every original frame remains visible

| ID | Candidate pairs | Median reference separation (m) | Thermal−RGB recorded left time (ms) | Mean-distance learned / angular geometry | Lower-error choice fraction |
|---|---:|---:|---:|---:|---:|
| 000000 | 1527 | 0.0277 | -2.652 | 0.852 | 52.7% |
| 000001 | 52 | 0.6333 | 61.537 | 0.971 | 32.0% |
| 000002 | 857 | 0.1481 | -10.577 | 0.907 | 53.8% |
| 000003 | 1686 | 0.0247 | -2.583 | 0.855 | 55.5% |
| 000004 | 1114 | 0.0774 | -8.306 | 0.864 | 56.4% |
| 000005 | 1149 | 0.0498 | -3.948 | 0.848 | 55.4% |
| 000006 | 653 | 0.1350 | -10.908 | 0.822 | 58.3% |
| 000007 | 1386 | 0.0358 | -1.381 | 0.792 | 56.1% |
| 000008 | 1280 | 0.0672 | -7.074 | 0.856 | 50.7% |
| 000009 | 849 | 0.1262 | -2.864 | 0.880 | 52.2% |
| 000010 | 1003 | 0.1036 | -9.788 | 0.852 | 57.6% |
| 000011 | 2429 | 0.0084 | -0.212 | 0.865 | 60.6% |
| 000012 | 1115 | 0.0795 | -5.981 | 0.863 | 54.3% |
| 000013 | 1438 | 0.0317 | -1.655 | 0.889 | 53.8% |
| 000014 | 696 | 0.1357 | -10.084 | 0.942 | 47.8% |
| 000015 | 2345 | 0.0112 | 1.103 | 0.870 | 52.6% |
| 000016 | 1299 | 0.0544 | -4.555 | 0.830 | 60.7% |
| 000017 | 2293 | 0.0092 | -0.335 | 0.793 | 60.1% |
| 000018 | 1163 | 0.0710 | -8.817 | 0.822 | 57.5% |
| 000019 | 1420 | 0.0386 | -2.107 | 0.774 | 55.0% |

Recorded time separation is metadata, not a measured exposure delay or a causal explanation of reference discrepancy. No frame is excluded based on it. EXP050's benchmark-scope correction remains in force.

## Roadblocks and what papers could help with

There is no current compute, image-access or model-execution blocker for this bounded study. The remaining technical limits, separately from pending owner replies/CATS access, are:

- **Exact surface identity and visibility:** sparse projected depth lacks point IDs and unobserved occluders. Mutual proximity and sparse frontmost checks do not prove same physical points. Motion between cameras' acquisition times is also uncompensated. Papers on occlusion-aware RGB–thermal correspondence with sparse LiDAR and motion-aware reprojection could improve the support protocol. They cannot create missing physical reference evidence from these PNGs.
- **Meaning of propagated uncertainty:** nonlinear inverse disparity can amplify tiny disparities, and first-order rank-one propagation is a ranking score here, not a validated metric error distribution. Papers on nonlinear disparity-to-depth uncertainty, inverse-depth parameterizations and uncertainty calibration across cameras would be useful before confidence/calibration claims. The fixed-angular control does not reproduce the learned rule's whole benefit, but the image-wide camera-scale control largely does. Papers should help distinguish cross-camera scale calibration from local reliability prediction rather than assume the latter is established.
- **Transfer beyond this inspected recording:** one 20-frame rain pilot is insufficient for general cross-modal reliability. The next necessary empirical check is a frozen replication on a distinct existing MS² recording, including the per-image camera-scale baseline and all baseline/reference/support counterexamples. Existing Box day/night snapshots provide a route; no new Geethika calibration question or CATS access is required for that replication. Training membership remains unknown, so do not call it training-held-out.

These reading topics are useful targeted methodological questions, not additional permission gates or prerequisites to reporting the bounded result. No paper search should substitute for the next independent-recording test. No claim that uncertainty-weighted fusion itself is novel follows from this result.

## Validation and replay

Authoritative independent audit: `results/ms2-cross-modal-support/validation-full.json` PASS. It reconstructs 51,508 raw model predictions (including invalid cases), checks120 sampled nearest-label/frontmost results by separate scalar/brute-force arithmetic, independently recomputes 480 selector risks, all 80 reference-consistency masks and160 subset risks, and verifies 256 frozen hashes including repeated references. Independent composition uses4×4 matrix inverses instead of the producers' direct3×3 composition. Both endpoint anchor comparisons, availability masks, geometry/learned selector decisions and oracle bounds pass. Support input hashes match the original EXP054 recovery manifest; historical inputs remain unchanged. The separate scale validator recomputes 51,108 valid raw variance/geometry contributions and120 scale-control risks, with 41 additional frozen-hash checks, all PASS. Do not add repeated reconstruction counts as new samples. These counts measure implementation checks, not independent statistical observations.

Synthetic support fixtures cover nearest ties, empty support, z-buffer occlusion/ties, negative-Z projection and identity/empty geometry. Two analytic/finite-difference disparity cases check the depth Jacobian. Fixed overlays and the PNG/PDF summary were visually inspected. A validator-only indentation typo when adding subset checks was corrected before the full audit; scientific results did not change. An earlier primary-only validation file remains for provenance; do not sum repeated audit checks as new evidence.

Runtime is isolated Python3.12.3, NumPy2.2.6, SciPy1.15.3, Pillow11.3.0 and Matplotlib3.10.7. Original frontend arrays were produced in EXP054's historical GPU runtime; no new frontend/model execution occurred. Source scripts under `MAC-VO-main/Scripts/Setup/`:

- `ms2_cross_modal_support.py`: fixed candidate support, diagnostics and overlays.
- `ms2_cross_modal_metric.py`: fixed common targets, metric propagation and paired-risk analysis.
- `ms2_cross_modal_geometry_control.py`: fixed-angular/pixel and unpropagated comparators.
- `validate_ms2_cross_modal_support.py`: independent raw-input/saved-output audit.
- `ms2_cross_modal_scale_control.py` / `validate_ms2_cross_modal_scale.py`: per-image median-scale control and independent raw-variance audit.
- `plot_ms2_cross_modal_support.py`: PNG/PDF summary.

From `ore-ms`, with the isolated dependencies available and EXP054/055 inputs restored:

```bash
python MAC-VO-main/Scripts/Setup/ms2_cross_modal_support.py --root /root/AirStack/ore-ms --output /path/to/fresh-support --recovery-manifest /root/ore-ms-recovery/2026-10-25-exp-054/manifest.json
# Inspect fixed overlays and record an inspection-decision.md in that support directory before metric scoring.
python MAC-VO-main/Scripts/Setup/ms2_cross_modal_metric.py --root /root/AirStack/ore-ms --support /path/to/inspected-support --output /path/to/fresh-metric --recovery-manifest /root/ore-ms-recovery/2026-10-25-exp-054/manifest.json
python MAC-VO-main/Scripts/Setup/ms2_cross_modal_geometry_control.py --root /root/AirStack/ore-ms --source /path/to/fresh-metric --output /path/to/fresh-control
```

Generate the scale-control directory before plotting, using the same replay base:

```bash
python MAC-VO-main/Scripts/Setup/ms2_cross_modal_scale_control.py --base /path/to/replay-base --output /path/to/replay-base/scale-control-run-001
python MAC-VO-main/Scripts/Setup/validate_ms2_cross_modal_scale.py --root /root/AirStack/ore-ms --base /path/to/replay-base --output /path/to/fresh-scale-validation.json
```

For validation/plotting, the base directory must contain `run-001`, `metric-run-001`, `geometry-control-run-001` and `scale-control-run-001` with the corresponding support/metric/control reports and arrays. The current validator also requires the source-model and reference inputs for independent reconstruction. Use fresh output paths:

```bash
python MAC-VO-main/Scripts/Setup/validate_ms2_cross_modal_support.py --root /root/AirStack/ore-ms --base /path/to/replay-base --output /path/to/fresh-validation.json
python MAC-VO-main/Scripts/Setup/plot_ms2_cross_modal_support.py --base /path/to/replay-base
```

No historical default, checkpoint, pixel, calibration, timestamp, threshold or scientific output is replaced. Generated outputs, frozen protocols and the pre-scoring inspection decision are preserved in the recovery snapshot; models/full source transport archives are not needed for replay from saved predictions.
