# EXP-070: retrospective disparity bias and scale calibration

Date: 2026-10-28 America/New_York. Base `09d1c5078`, branch `ore_ms`.

The declared constant per-camera bias/scale controls do not establish transferable calibration or improve camera selection consistently. Both fitted selectors worsen median mean-distance risk versus the original selector on all four examined recordings. Bias plus scale improves thermal native-disparity likelihood versus scale only, while its selection advantage over scale only appears in the first rain recording and reverses in the other three. Keep defaults and both reserved captures unchanged.

## Protocol and preservation

Executed the [committed declaration](../docs/research/ms2-calibration-protocol-2026-10-28.md) without changing targets, model families, weights or recording exclusions. Four previously examined captures, 20 original IDs each, 106562 both-valid pairs / 213124 camera observations. Train each camera on three captures with equal capture/frame mass and evaluate the excluded capture; no all-capture final fit. Original predicted 3D points, support and finite errors remain unchanged. Native-pixel additive residual bias is fitted with inverse-variance weights. The bias-aware original-point score includes squared bias, `J²*(s²*v+beta²)`; interval recentering does not correct predictions.

All four Box archive hashes and all 1564 archived member hashes pass. Restored only scientific results/data, preserving identical files and refusing divergent destinations; no tracked files were restored from older archives. Input provenance includes 660 pre-fit hashes and native PNG/frontend/support/calibration ancestry from EXP063/064, validated derived inputs from EXP066 and the EXP069 interval baseline. No network inference, retraining, new source imagery or reserved-capture access.

Run002 adds explicit refusal to aggregate partial selection results after a policy failure. Fits, full report and all 80 saved prediction/choice files are byte-identical to run001; neither real-data run has a fit or policy failure. Every comparison remains retrospective because all four recordings influenced this protocol.

## Fitted coefficients

Each row excludes the named recording. `a²` is scale-only variance multiplier; `beta` is native-pixel residual mean and `s²` is centered variance multiplier. These are different camera pixel systems, not cross-camera comparable likelihood or uncertainty magnitudes.

| Excluded recording | Camera | a² | beta (px) | s² |
|---|---|---:|---:|---:|
| exp063_rain | rgb | 4.0644 | -0.0481 | 4.0620 |
| exp063_rain | thr | 0.8815 | 1.0745 | 0.4746 |
| exp064_day | rgb | 13.4880 | 0.2721 | 13.2174 |
| exp064_day | thr | 2.5055 | 1.3198 | 1.1776 |
| exp064_night | rgb | 15.6714 | 0.3052 | 15.2972 |
| exp064_night | thr | 2.3830 | 1.1840 | 1.2051 |
| exp064_rainy | rgb | 14.9932 | 0.3824 | 14.4397 |
| exp064_rainy | thr | 2.3644 | 1.2093 | 1.1842 |

## Interval results

Entries are medians of 20 per-frame quantities, using inclusive ±1.96 sigma intervals; the hypothetical centered-normal benchmark is approximately 95%. Lower endpoint counts are retained without clipping or inversion into finite depth bounds.

| Excluded recording | Camera | Original coverage | Scale coverage | Bias+scale coverage | Bias+scale nonpositive lower endpoint |
|---|---|---:|---:|---:|---:|
| exp063_rain | rgb | 43.18% | 65.65% | 65.83% | 1.52% |
| exp063_rain | thr | 64.01% | 61.35% | 72.32% | 1.15% |
| exp064_day | rgb | 63.10% | 98.80% | 98.87% | 17.96% |
| exp064_day | thr | 99.12% | 99.98% | 99.76% | 22.19% |
| exp064_night | rgb | 91.61% | 100.00% | 100.00% | 68.81% |
| exp064_night | thr | 92.15% | 98.23% | 98.40% | 74.76% |
| exp064_rainy | rgb | 84.84% | 99.88% | 99.52% | 65.16% |
| exp064_rainy | thr | 95.11% | 99.47% | 99.34% | 47.68% |

RGB scale multipliers trained without night/rainy observations are large: night coverage becomes 100% and newer-rain approaches 100%, while original first-rain RGB still covers only about 66% after fitting. Thermal first-rain coverage remains about 72% after bias plus scale, whereas other thermal captures reach 98–100%. Widening intervals is not uniform calibration.

Paired changes below are computed within each frame before taking the median. Negative coverage-error changes favor bias plus scale. Native-pixel NLL changes compare the same camera and frame only.

| Recording | Camera | Bias+scale minus scale: absolute coverage error (percentage points) | Bias+scale minus scale: NLL |
|---|---|---:|---:|
| exp063_rain | rgb | -0.166 | +0.05756 |
| exp063_rain | thr | -11.466 | -0.63191 |
| exp064_day | rgb | +0.000 | -0.01611 |
| exp064_day | thr | -0.176 | -0.29259 |
| exp064_night | rgb | +0.000 | -0.00885 |
| exp064_night | thr | +0.038 | -0.36787 |
| exp064_rainy | rgb | -0.117 | +0.00365 |
| exp064_rainy | thr | +0.000 | -0.34509 |

Thermal likelihood improves versus scale only in all four excluded recordings, but absolute 95% coverage error worsens slightly in night and is unchanged at the newer-rain median. The full 11 groups retain signed means, tails, centered spread, second moments, quantiles, NLL and positivity at 1/1.96/2/3 sigma. Conditional-group findings must accompany marginal results.

For example, bias-plus-scale 95% coverage by original per-frame variance quartile is:

| Recording | Camera | Lowest variance | Bin 2 | Bin 3 | Highest variance |
|---|---|---:|---:|---:|---:|
| exp063_rain | rgb | 33.50% | 50.28% | 83.28% | 96.65% |
| exp063_rain | thr | 66.73% | 76.17% | 70.99% | 75.63% |
| exp064_day | rgb | 97.35% | 99.02% | 99.53% | 100.00% |
| exp064_day | thr | 99.69% | 99.83% | 100.00% | 100.00% |
| exp064_night | rgb | 100.00% | 100.00% | 100.00% | 100.00% |
| exp064_night | thr | 95.43% | 99.53% | 100.00% | 100.00% |
| exp064_rainy | rgb | 98.44% | 100.00% | 100.00% | 100.00% |
| exp064_rainy | thr | 99.05% | 100.00% | 100.00% | 100.00% |

These bins are native within-frame score groups with fixed original cuts, not common camera thresholds or independently sampled populations. Reference-range bands and empty-group/count accounting remain in the full report; no conditional refit was performed.

## Camera-selection results

Entries are medians of 20 paired mean-3D-distance risk ratios at the original midpoint supplied-reference target. Ratios below one favor the fitted policy. Models use the unchanged original points; no favorable post-result model replacement.

| Excluded recording | Policy | / Original | / Scale only | / Always RGB | / Always thermal | / Camera-scale control |
|---|---|---:|---:|---:|---:|---:|
| exp063_rain | scale | 1.2373 | 1.0000 | 1.7618 | 0.8911 | 1.2147 |
| exp063_rain | bias_scale | 1.1568 | 0.9289 | 1.6175 | 0.8164 | 1.1361 |
| exp064_day | scale | 1.0739 | 1.0000 | 0.1607 | 0.7994 | 1.0329 |
| exp064_day | bias_scale | 1.1140 | 1.0328 | 0.1735 | 0.8347 | 1.0732 |
| exp064_night | scale | 1.0700 | 1.0000 | 1.8351 | 0.9733 | 1.1633 |
| exp064_night | bias_scale | 1.0864 | 1.0147 | 1.8558 | 0.9872 | 1.1721 |
| exp064_rainy | scale | 1.0970 | 1.0000 | 1.2587 | 0.9492 | 1.1143 |
| exp064_rainy | bias_scale | 1.1163 | 1.0078 | 1.2755 | 0.9679 | 1.1336 |

Bias plus scale beats scale only in all 20 first-rain mean-distance frames, but loses in all 20 day frames, 19/20 night frames and 15/20 newer-rain frames. Both fitted methods lose to the original selector on median mean-distance risk in every excluded recording. In night, bias plus scale loses to always RGB all 20 frames (median ratio 1.8558). In day both fitted policies beat both fixed cameras all 20 mean-distance frames, while still worsening the original selector median. This favorable case does not establish a general rule.

Squared-error ratios remain separate because their tails differ:

| Excluded recording | Policy | / Original (MSE) | / Scale only (MSE) | / Always RGB (MSE) |
|---|---|---:|---:|---:|
| exp063_rain | scale | 1.37637 | 1.00000 | 2.78806 |
| exp063_rain | bias_scale | 1.28354 | 0.93918 | 2.60400 |
| exp064_day | scale | 0.98978 | 1.00000 | 0.00052 |
| exp064_day | bias_scale | 1.00568 | 1.03968 | 0.00057 |
| exp064_night | scale | 1.03080 | 1.00000 | 2.81000 |
| exp064_night | bias_scale | 1.03302 | 1.00341 | 2.81547 |
| exp064_rainy | scale | 1.15430 | 1.00000 | 0.78323 |
| exp064_rainy | bias_scale | 1.18127 | 1.00304 | 0.77644 |

Thus the claim that every fitted policy worsens the original refers to primary mean-distance medians; day scale-only MSE is a small counterexample (ratio 0.98978). All endpoint-anchor results, both losses, fixed/random/camera-scale baselines, wins/ties/losses, and zero-denominator categories remain published in the report.

| Recording | Candidates | Both valid | RGB only | Thermal only | Neither |
|---|---:|---:|---:|---:|---:|
| exp063_rain | 22277 | 22277 | 0 | 0 | 0 |
| exp064_day | 48270 | 48026 | 0 | 244 | 0 |
| exp064_night | 24387 | 24387 | 0 | 0 | 0 |
| exp064_rainy | 12838 | 11872 | 0 | 966 | 0 |

Selection risk is conditional on both-valid support. Single-camera availability remains explicit; a full operational fallback policy was not evaluated. Dependent spatial pixels/frames and reused recordings do not support independent significance or weather-causal claims.

## Validation and reproduction

Independent audit PASS: 5,234,111 scalar checks, all 106562 pairs. It reconstructs original frontend fields and native uint16 reference PNGs, checks derived disparity/variance/geometry, independently fits weighted least squares, computes direct interval inclusion and sorted quantiles, reconstructs all anchor losses and camera-scale selection, and recomputes every interval/paired/selection summary. Original EXP069 coverage and historical metric risks reproduce; input hashes pass after evaluation. Ten synthetic fixtures pass, including excluded-label mutation, unequal frame densities, zero-scale/invalid-score rejection, exact ties, empty strata and explicit refusal to aggregate policy failures.

Scripts in `MAC-VO-main/Scripts/Setup`: `ms2_bias_scale_calibration.py`, `validate_ms2_bias_scale_calibration.py`, `plot_ms2_bias_scale_calibration.py`. Authoritative results: `results/ms2-bias-scale-calibration/run-002/`; run001 remains the preserved initial execution. Includes pre-fit protocol/input map, frozen training fits, report, 80 per-frame score/choice/residual NPZs, independent/synthetic validations, parity evidence and PNG/PDF figure. CPU runtime: Python3.12.3, NumPy2.2.6, SciPy1.15.3, OpenCV4.10.0.84; no GPU needed.

From MAC-VO root, after verified prerequisite recovery:

```bash
/root/ore-ms-analysis-venv/bin/python -m Scripts.Setup.ms2_bias_scale_calibration \
  --root /root/AirStack/ore-ms --output <fresh-output>
/root/ore-ms-analysis-venv/bin/python -m Scripts.Setup.validate_ms2_bias_scale_calibration \
  --root /root/AirStack/ore-ms --run <fresh-output>
/root/ore-ms-analysis-venv/bin/python -m Scripts.Setup.validate_ms2_bias_scale_calibration \
  --synthetic <fresh-output>/synthetic-validation.json
/root/ore-ms-analysis-venv/bin/python -m Scripts.Setup.plot_ms2_bias_scale_calibration \
  --run <fresh-output>
```

## Decision

Do not adopt a global scale or additive-bias-plus-scale selector. Disparity likelihood, interval coverage and metric camera-selection loss remain distinct. Fixed camera-level mean/scale controls do not resolve the observed transfer failure; this does not prove a physical cause, an implementation bug, or that a particular more flexible model would transfer.

Keep the original inference contract and point estimates. Preserve both untouched captures for a separately frozen complete method and prospective success criterion. The next research decision is a synthesis of this negative constant-calibration result with EXP065–069 before adding model capacity or spending new captures. Independent geometric references would strengthen physical claims; usable CATS access and checkpoint training membership remain unconfirmed. No additional source upload is needed to reproduce this result.

Private recovery target: `ore_ms:ore-ms/backups/2026-10-28-exp-070`; dependencies EXP063/064/066/069. Credentials, checkpoints and full source archives are excluded.
