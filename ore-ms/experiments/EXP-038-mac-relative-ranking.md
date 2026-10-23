# EXP-038: MAC covariance ranking of RGB versus thermal matching disagreement

Recorded:2026-10-22T23:35:18.410590-04:00; branch `ore_ms`; starting commit `1e5839d16` plus frozen ranking source. **The primary nominal-angular covariance rule agrees with lower independent-tracker disagreement on108/114 paired transitions, below112/114 for always RGB. Six thermal choices help once and hurt five times. A predeclared full-grid uncertainty sensitivity helps on one switch and improves by one decision. This population dependence and strongly imbalanced single-recording sample do not establish a transferable reliability/fusion rule. No model training or new inference.**

## Research question and fixed sample

The user clarified that the target is MAC uncertainty, rather than developing a new VO method. This experiment returns directly to a basic rule on existing MAC-VO RGB and MACThermal outputs: does the smaller covariance select the camera with less observed geometric matching discrepancy, and does that add value over a fixed camera choice?

Reuse all six EXP-030 windows with both modalities:salon-so-day early/middle/late and the first20 frames of long-salon early/middle/late. Both cohorts come from `20251027/0_salon_so_day`, previously inspected and selected in EXP-025/026. Six20-frame paired windows yield114 paired temporal transitions, not114 independent trials or multiple held-out recordings. Thermal-only night windows are excluded by the requirement for an existing corresponding RGB case, not by outcomes. No new images, learned inference, training, covariance floors, scale/sign fitting, trajectory optimization or threshold search.

Restore230 preserved saved artifacts from the verified EXP-030 backup:plan,analysis and228 per-modality full-grid NPZs. Exact source-member hashes match the earlier preservation manifest; the new registry freezes those230 files plus the ranking source. The full historical EXP-030 model/image-input registry is not claimed restored. Saved arrays contain raw temporal uu/vv/uv covariance, model/LK endpoints,K,nominal ray variance and masks; original model/checkpoint provenance remains the prior experiment record.

RGB/thermal paired-manifest rows match exactly at both transition endpoints; all four stream times increase. Both stereo offsets<=2ms and RGB/thermal offsets<=30ms pass for all114 transitions; absolute cross-modal separation ranges19.431–28.415ms. Original intervals/gaps are retained (largest~200ms). This does not verify hardware synchronization, identical physical features or a common instantaneous view. No timestamp interpolation or repaired pairing is introduced.

## Measurement and rule

For each camera separately, primary U is median saved nominal ray covariance trace in rad² over its own independent LK-valid source-grid population. EXP-030 propagated the full raw2x2 MAC temporal matching covariance through the target-ray Jacobian without flooring/fitting; these inherited nominal-camera quantities are not a new probability calibration. Target D is median exact angular MAC–LK endpoint disagreement in radians on the same camera's valid population. Select the camera with lower U; lower D defines the diagnostic winner. Exact ties abstain,missing/nonfinite populations remain explicit; all primary pairs are finite with no ties/abstentions here.

Different cameras' fields of view,features,resolution and selected point populations differ. LK status/forward-backward/visibility gates do not use MAC covariance, but conditioning on independent tracker support can still change both populations and uncertainty summaries. LK is an image algorithm with its own errors and shared visual ambiguities, not physical ground truth. Thus this is ranking relative aggregate matching disagreement, not matched-physical-ray reliability, actual pose accuracy, probability calibration or performance of a fused system.

This targets MAC's raw temporal image-matching covariance component, not the full3D metric covariance or optimized pose covariance. Nominal pinhole K and first-order ray linearization remain camera-dependent assumptions; broad distributions and model target predictions outside the image remain in the inherited arrays without a new validity certificate.

Native-pixel median covariance is compared against the SAME angular D target as a unit sensitivity. Baselines always RGB/always thermal and an oracle lower-D choice are scored on the same eligible pairs. Their realized mean selected D is descriptive; there is no fusion mechanism or retrained model. The oracle is a hindsight diagnostic bound, not an implementable method.

## Primary results

| Window (first20 frames) | Angular U agreement | Always RGB | Pixel U agreement | Thermal choices (help/harm) | Selected mean D / RGB mean D (mrad) |
|---|---:|---:|---:|---:|---:|
| long-salon-early | 19/19 | 19/19 | 19/19 | 0 (0/0) | 0.071 / 0.071 |
| long-salon-late | 19/19 | 19/19 | 19/19 | 0 (0/0) | 0.597 / 0.597 |
| long-salon-middle | 13/19 | 17/19 | 9/19 | 6 (1/5) | 1.835 / 1.618 |
| salon-so-day-early | 19/19 | 19/19 | 19/19 | 0 (0/0) | 1.114 / 1.114 |
| salon-so-day-late | 19/19 | 19/19 | 15/19 | 0 (0/0) | 0.865 / 0.865 |
| salon-so-day-middle | 19/19 | 19/19 | 18/19 | 0 (0/0) | 0.663 / 0.663 |

Pooled angular rule108/114(94.74%),pixel rule99/114(86.84%),always RGB112/114(98.25%),always thermal2/114(1.75%). Angular units improve this raw pixel rule, but do not beat the fixed RGB comparator. Mean selected angular disagreement is0.8575mrad for angular U,0.9054mrad for pixel U,0.8212mrad for always RGB,1.4287mrad for always thermal and0.8136mrad for the oracle. Primary angular choice increases mean disagreement4.42% relative to RGB, with oracle regret0.04387mrad.

All six primary thermal choices occur in long-salon-middle:one chooses the lower-D thermal camera, five choose higher-D thermal; one of the two thermal-favored pairs is missed. Majority-class imbalance matters:112/114 targets favor RGB. Angular-rule recall is107/112 for RGB and1/2 for thermal; balanced accuracy72.77% versus50% for always RGB. The minority denominator of only two prevents treating that balanced result as evidence of robust thermal detection. Counts and magnitudes are retained together; no accuracy threshold or winner is selected after inspection.

## Population sensitivity

| Population | Angular U agreement | Always RGB | Thermal choices(help/harm) | Mean D change vs RGB |
|---|---:|---:|---:|---:|
| primary | 108/114 | 112/114 | 6 (1/5) | +4.42% |
| tight | 110/114 | 113/114 | 5 (1/4) | +3.93% |
| texture | 110/114 | 113/114 | 5 (1/4) | +4.95% |
| all_uncertainty | 113/114 | 112/114 | 1 (1/0) | -0.86% |

Tight uses inherited<=0.5px forward/backward support;texture adds initial21x21 source std>=5. Full-grid U computes uncertainty from every fixed-grid prediction, while keeping primary LK-valid D unchanged. It avoids using the LK mask in U but changes the U aggregation population; it does not solve unknown ground truth or camera comparability.

The full-grid sensitivity chooses thermal exactly once, correctly, yielding113/114 and0.8141mrad mean D,0.86% below always RGB. This single outcome is worth preserving, not selecting as a tuned best method. It misses the second thermal-favored pair. Native and angular policies happen to make the same decisions under full-grid U here. Tight/texture primary-style rules remain worse than their own always-RGB baselines. These are predeclared population views of the same data, not independent replications or a validation split.

Pooled primary Spearman between log(U_RGB/U_T) and log(D_RGB/D_T) is0.499; individual windows range−0.418 to0.711 (including three negative windows). The pooled coefficient includes motion/scene/window differences and cannot establish a stable within-window relationship. No significance test, causal association, forecasting,confidence interval or cross-recording generalization is claimed.

![MAC relative ranking](../results/mac-relative-ranking/run-002/relative-ranking.png)

## Validation and reproducibility

11,452 independent controls pass:all231 frozen hashes,scalar medians/finite counts,exact agreement with inherited EXP-030 pair quantiles,paired row identity/timing gates,independent policy labels/cost/regret/baseline/rank calculations and majority-class/tie/missing/helpful-harmful/unit-reversal behavior checks. Four populations produce456 pair-population records and56 window/policy summaries. Every frozen saved input is unchanged.

An initial native-pixel reduction used float32 median averaging,while the historical quantile helper and independent scalar validator use float64. The validator found a5.82e-11px² difference. Final run-002 explicitly promotes all reductions to float64 to match the historical convention; all456 pair-population policy choices remain unchanged from run-001. Original failed validator log,run-001 outputs and its exact source reconstruction are retained. No scientific threshold or input was changed. Primary angular outputs are unchanged.

CPU Python3.12.3,NumPy2.2.6;plots Matplotlib3.10.1. No GPU,checkpoint or model dependency is needed for this saved-output replay. Run:

```bash
PYTHONPATH=<numpy-package-path> python MAC-VO-main/Scripts/Setup/td3_relative_ranking.py \
  --source results/temporal-angular-consistency/run-001 \
  --output results/mac-relative-ranking/new-run
```

Exact analysis,validation,plot/documentation scripts,commands/runtime and source hash registry are preserved under `results/mac-relative-ranking/staging/`; final plan,per-pair decisions,summary CSVs,validation and standalone PNG/PDF are under `run-002/`. Recovery target Box `backups/2026-10-22-exp-038` includes all230 saved input artifacts for standalone replay,source/report/notebook,outputs and incremental Git bundle. Private credentials,weights,packages and whole source archives are excluded.

## Decision

The primary covariance ranking does not show added selection value over RGB for this daytime matching-disagreement target. Preserve the one-switch full-grid result as a population-sensitive hypothesis. Do not adopt a fusion rule or globally prefer RGB from this single recording:almost no thermal-favored examples are available,night RGB availability is a different target,and D is not physical error.

The next MAC-focused evidence should separate camera-specific calibration/population effects from actual relative-error prediction on another recording where both cameras supply observable geometry and the winner varies. Keep existing weights and MSA experimental calibration. The present result does not say MAC covariance is universally uninformative; it constrains this fixed aggregate rule,target and sample.
