# EXP-059: Factor camera scale and depth propagation in cross-modal score failure

Date: 2026-10-27 America/New_York. Branch `ore_ms`; starting commit `0894c80ff`.

**Finding: neither camera-median normalization nor supplied-reference-depth propagation explains away the night failure.** All four frozen diagnostic scores remain worse than always RGB and equal-random selection in all 20 night frames under both losses. The interventions have different effects on day and rain. No blanket rescaling or propagation replacement is supported, and no correction is fitted or deployed.

The deterministic score factorization identifies a concrete sensitivity: holding the ray and learned variance fixed, replacing reference depth by a prediction changes the propagated score by `(Zpred/Zref)^4`. Underestimated depth can compress the score, but this is not established as the sole cause of bad cross-camera choices. Reference-depth controls borrow evaluation labels and cannot be used as deployment fixes.

## Frozen analysis and populations

Reuse all three previously inspected20-ID windows, original candidate support, both-valid masks, predictions, errors, midpoint/RGB/thermal anchors and depth-consistency sensitivities. There are25,554 rain,22,717 day and85,170 night both-valid pairs; their pixels/frames/anchors/losses are dependent repeated observations. Original availability failures remain in their EXP056–058 records; this diagnosis does not redefine those populations. No new images, model execution, fitting, re-pairing, timing/calibration correction or sample exclusion.

The four interventions and depth-ratio strata were recorded before their outcomes. These are exploratory diagnostic interventions on already scored data, not new training-held-out evaluation. All comparisons use identical errors/predictions and change only which camera the score chooses. No improved score is selected as a new default.

## Exact factorization

For each modality's native left pixel, fixed ray q, original baseline B, focal length fx and learned horizontal flow variance:

```
vangular = Var(flow_u) / fx²
U_pred = (||q||² / B²) Zpred⁴ vangular
```

This is the previously evaluated rank-one, first-order depth score. It is not full 3D covariance or a physically calibrated error distribution. It assumes a fixed ray/calibration and uses estimated disparity/depth in the Jacobian. Vertical flow variance is not silently added to depth variance.

The RGB/thermal log-score ratio decomposes into four retained terms:

```
log(Urgb/Uthr) = log(camera-median angular variance ratio)
              + log(local angular variance ratio / camera-median ratio)
              + log((||q_rgb||²/B_rgb²) / (||q_thr||²/B_thr²))
              + 4 log(Zpred_rgb/Zpred_thr)
```

All per-pair terms and per-image quartiles are saved. Their sum reproduces the original score ratio, and the original score/decision/risks reproduce before interventions. These are additive log terms, not a causal decomposition of reference risk or statistically independent contributions.

## Fixed 2×2 diagnostic

| Policy | Depth in score Jacobian | Angular variance in score | Deployability limitation |
|---|---|---|---|
| Original | Predicted | Original learned | Existing candidate benchmark, not a validated deployed policy |
| Median-normalized | Predicted | Divide each camera's variance by its per-image median | Uses no errors to set scale, but median support is reference-selected; not a fitted cross-camera calibration |
| Reference-depth* | Own supplied single-scan depth at candidate pixel | Original learned | Uses evaluation depth labels; diagnostic only |
| Reference-depth + normalized* | Own supplied single-scan depth | Per-image camera-median-normalized | Uses evaluation depth labels; diagnostic only |

Reference-depth intervention changes only the Jacobian score. The predicted 3D point, its error target, support, covariance network output and camera selection baselines stay fixed. Reference depth is not used to repair a reconstruction. Each modality's reference Z is positive and read at its own original candidate pixel; no favorable anchor or depth product is selected.

For fixed ray/variance:

```
U_pred / U_ref = (Zpred / Zref)⁴
```

A half-depth estimate therefore gives 1/16 the reference-linearized score, and a double-depth estimate gives 16 times that score. This shows how plugging biased predictions into propagation can affect ranking. It does not show reference depth is physical truth or that replacing the term would improve every case.

## Mean-distance risk, all original comparisons

Entries are medians of 20 per-image paired selector/comparator mean 3D-distance risk ratios, with frames below1 in parentheses. Below1 favors the diagnostic selector. No pixels/recordings are pooled for inference.

| Recording | Score | Selector / always RGB (below1) | Selector / equal random (below1) |
|---|---|---:|---:|
| Rain | Original predicted-depth score | 0.190 (20/20) | 0.306 (20/20) |
| Rain | Camera-median normalization | 0.214 (20/20) | 0.341 (20/20) |
| Rain | Reference-depth Jacobian* | 0.991 (20/20) | 1.526 (0/20) |
| Rain | Reference-depth + median normalization* | 0.918 (19/20) | 1.287 (3/20) |
| Day | Original predicted-depth score | 1.157 (0/20) | 0.867 (20/20) |
| Day | Camera-median normalization | 1.659 (0/20) | 1.224 (0/20) |
| Day | Reference-depth Jacobian* | 0.972 (19/20) | 0.725 (20/20) |
| Day | Reference-depth + median normalization* | 1.123 (0/20) | 0.840 (20/20) |
| Night | Original predicted-depth score | 1.253 (0/20) | 1.121 (0/20) |
| Night | Camera-median normalization | 1.247 (0/20) | 1.110 (0/20) |
| Night | Reference-depth Jacobian* | 1.226 (0/20) | 1.106 (0/20) |
| Night | Reference-depth + median normalization* | 1.216 (0/20) | 1.088 (0/20) |

On night, median normalization only slightly changes the RGB/random deficits. Even the reference-depth + normalized score has mean-distance risk21.6% above always RGB and8.8% above random, with neither comparator beaten in any frame. Thus neither tested factor, separately or together in this form, repairs cross-camera ranking on this population.

On day, reference-depth propagation changes learned/RGB median ratio from 1.157 to 0.972 (19/20 below1), while median normalization alone worsens it to 1.659. On rain, replacing predicted depth with reference depth changes the originally useful random-relative ratio 0.306 to 1.526; all 20 become worse than random. These conflicting effects prevent treating reference-depth substitution as a universal explanation or recommending a generic normalization. The reference-assisted outcomes are not a usable method, even where favorable.

![Fixed score interventions](../results/ms2-score-failure-diagnosis/score-diagnosis-final.png)

## Camera scales and depth sensitivity

Each entry below is a median of per-image summaries, not a pooled-pixel statistic. The two depth columns are medians of per-image median predicted/supplied-reference Z. The last columns are fractions of all both-valid pairs under the **original** selector; “wrong” means choosing the higher supplied-reference mean-distance error, not physical error. “Both wrong and under half” is a joint descriptive fraction, not a fraction of error causally explained by depth.

| Recording | Median RGB/thermal angular-variance scale | RGB Zpred/Zref | Thermal Zpred/Zref | Selected estimate below half reference depth | Both wrong choice and selected below half |
|---|---:|---:|---:|---:|---:|
| Rain | 0.041 | 0.979 | 0.939 | 2.0% | 1.6% |
| Day | 0.072 | 0.971 | 0.760 | 7.8% | 6.9% |
| Night | 1.607 | 1.017 | 0.879 | 13.9% | 13.0% |

The median camera-scale ratio differs substantially across the three windows (~0.041,0.072,1.607), so a common unit conversion does not establish a stable relative scale. This observation alone does not identify an erroneous scale: camera appearance, model response, reference error and data selection all differ. No constant is fitted using these outcomes.

Night's selected-depth-under-half fraction has median 13.9%; the joint wrong-and-under-half fraction is 13.0%, while all wrong selections have median 56.2%. Severe under-depth selections are therefore a visible subset of the failure, not a demonstrated explanation of every wrong choice. Thermal median Zpred/Zref0.879 corresponds to a median propagated-score/reference-score factor about 0.597. Reference-assisted scoring still fails, so learning/scale/error-target mismatch and residual reference/support issues remain candidate explanations, not established causes.

Fixed depth-ratio strata `<0.5`, `[0.5,2]`, `>2` retain every prediction and all per-camera counts. No clipping, calibration guarantee, Gaussian likelihood test, outlier deletion or score threshold is introduced.

## Squared-loss sensitivity

| Recording | Score | Selector / always RGB (below1) | Selector / equal random (below1) |
|---|---|---:|---:|
| Rain | Original predicted-depth score | 0.00304 (20/20) | 0.006048 (20/20) |
| Rain | Camera-median normalization | 0.003554 (20/20) | 0.00707 (20/20) |
| Rain | Reference-depth Jacobian* | 0.9996 (20/20) | 1.988 (0/20) |
| Rain | Reference-depth + median normalization* | 0.9289 (20/20) | 1.849 (2/20) |
| Day | Original predicted-depth score | 1.318 (7/20) | 0.799 (20/20) |
| Day | Camera-median normalization | 2.276 (1/20) | 1.406 (1/20) |
| Day | Reference-depth Jacobian* | 0.9531 (18/20) | 0.526 (20/20) |
| Day | Reference-depth + median normalization* | 1.094 (8/20) | 0.6647 (20/20) |
| Night | Original predicted-depth score | 1.653 (0/20) | 1.285 (0/20) |
| Night | Camera-median normalization | 1.651 (0/20) | 1.273 (0/20) |
| Night | Reference-depth Jacobian* | 1.513 (0/20) | 1.176 (0/20) |
| Night | Reference-depth + median normalization* | 1.465 (0/20) | 1.146 (0/20) |

Squared-loss and mean-distance comparisons are both retained rather than selecting the favorable one. The magnitude of rain's random-relative advantage is highly loss-dependent; the night failure survives both. Large-risk reductions do not imply accurately calibrated predictive variances.

## Anchors, support sensitivity and validation

All four policies remain worse than RGB/random in every night frame under both losses with either endpoint anchor. Every policy is also scored with both endpoint anchors and the unchanged 2/256m,1/2/5% depth-consistency sensitivities. All per-image risks, fractions and unavailable subsets remain in `report.json`; no favorable subset replaces primary support. Reference-depth score factors use own single-scan Z regardless of which shared error-reference anchor is evaluated, so score decisions are not tuned to anchor outcomes.

Independent raw-factor audit PASS:266,882 raw camera covariance/disparity/reference-depth factor checks,1440 primary selector risks,1920 depth-subset risks,40,232 scalar risk fields,240 raw-model/depth payload hashes and126 frozen analysis-source hashes. The independent validator reads raw horizontal covariance and disparity, original uint16 depth maps and intrinsics/baselines; independently takes camera medians and reconstructs all four score decisions before recomputing scalar risks. Prior saved 3D errors are reused with frozen hashes, rather than claimed as newly validated physical truth. Existing raw 3D reconstruction audits remain EXP056–058 evidence.

Synthetic checks verify common-positive score scaling preserves choices, per-camera normalization is invariant to constant multiplication of its angular variances, and depth doubling changes the fixed-ray score by 16. Original metric scores/choices/risks and exact factorization identities reproduce on all three windows. Historical scientific files and prediction arrays remain unchanged. Check counts repeat dependent samples and are not independent statistical evidence.

Outputs: `results/ms2-score-failure-diagnosis/run-001/{report.json,protocol.json,<recording>/diagnosis-*.npz}`, `validation.json`, PNG/PDF. Runtime remains isolated Python3.12.3, NumPy2.2.6, SciPy1.15.3, Pillow11.3.0 and Matplotlib3.10.7. Fixed plot inspected. New code under `MAC-VO-main/Scripts/Setup`:

- `ms2_cross_modal_score_diagnosis.py`: exact decomposition and frozen score interventions.
- `validate_ms2_cross_modal_score_diagnosis.py`: raw-factor/decision/risk audit, using the earlier independent scalar-risk helper.
- `plot_ms2_score_diagnosis.py`: all three windows and both baseline comparisons, with reference assistance marked explicitly.

From `ore-ms`, after restoring the existing EXP056–058 candidate/metric outputs and original source/depth/frontend payloads, use fresh output paths:

```bash
python MAC-VO-main/Scripts/Setup/ms2_cross_modal_score_diagnosis.py --root /root/AirStack/ore-ms --output /path/to/fresh-diagnosis
python MAC-VO-main/Scripts/Setup/validate_ms2_cross_modal_score_diagnosis.py --root /root/AirStack/ore-ms --source /path/to/fresh-diagnosis --output /path/to/fresh-validation.json
python MAC-VO-main/Scripts/Setup/plot_ms2_score_diagnosis.py --source /path/to/fresh-diagnosis --output /path/to/existing-fresh-figure-directory
```

The diagnosis intentionally reads the documented original rain/day/night result paths. The night byte-preserving layout must be recreated per EXP058 for raw validation. No full transport archives, checkpoint files, GPU execution or new uploads are needed for this saved-array diagnostic.

## Research decision

The two-factor diagnosis is complete. It identifies predicted-depth fourth-power sensitivity, differing camera-wide variance scales and nonuniform score-intervention effects. It does **not** identify a single validated cause or a universal correction. Keep within-camera uncertainty-ranking findings separate from cross-camera score comparability.

Before proposing fusion or applying a global rescaling, the next method-level question is whether a calibrated camera-risk/selection rule can transfer across recordings while beating fixed-camera and camera-scale baselines. Any such fit needs a declared recording-level development/evaluation protocol; these already inspected windows cannot become an untouched validation set by relabeling them. Reference-assisted scores must not be adopted or used as deployment evidence.

There is no new compute/access blocker, and no additional calibration question to the recording owners is required to report this evidence. Targeted literature on uncertainty calibration across heterogeneous predictors and nonlinear stereo-depth uncertainty remains relevant; source replies, CATS access and checkpoint training membership remain separate pending items.
