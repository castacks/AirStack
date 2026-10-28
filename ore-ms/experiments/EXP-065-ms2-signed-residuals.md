# EXP-065: signed residual structure and spatial mean accounting

Date: 2026-10-27 America/New_York. Starting commit: e0de6922c, branch ore_ms.

The large squared-error differences are not explained by a single global residual offset. Thermal has substantially more error in fixed range/location mean components than RGB, but removing each camera's global mean does not reverse the overall camera advantage in either examined night recording. This describes supplied-reference residuals, without identifying physical sensor bias or repairing the failed cross-camera transfer criterion.

## Frozen scope and estimands

Reuse all seven examined captures: original day/night/rain development windows, EXP063 rain and EXP064 day/night/rain. All original IDs000000–000019, 140 dependent frame groups, 240,003 both-valid pairs. No new data, fits, inference, clipping, sample removal or policy changes. The final two official-test recordings remain untouched. These are reused exploratory observations; their prospective status belongs to EXP063/064, not this diagnostic.

Residual vector is predicted XYZ minus reference XYZ in the shared RGB frame. Both endpoint anchors and the midpoint are computed independently. Primary support and strata remain unchanged from EXP061: midpoint reference Euclidean range intervals [0,5),[5,10),[10,20),[20,40),[40,80),[80,infinity), 3×3 native RGB grid, and their54 intersections. Save all70 groups including explicit empties; no count threshold.

For each frame/camera/anchor/group, population-denominator accounting is

`mean ||e||² = ||mean e||² + mean ||e - mean e||²`.

Each fixed partition also satisfies

`MSE = weighted within-cell centered spread + weighted squared cell means`

and the second term equals squared global mean plus weighted between-cell mean variation. Save signed XYZ means, axis second moments/spreads, MSE, mean distance and each partition component. These are spatial empirical summaries. Global centering and cell centering use reference labels; neither is a deployable correction or a learned covariance calibration test.

## Results

Entries below are medians of20 per-image quantities. Ratios are paired within each image before taking the median; ratios of pooled risks or ratios of marginal medians are not used. Global/joint fractions use the same total squared-error denominator. Joint means include the global contribution; columns must not be added. Reversals compare strict per-image camera ordering for total MSE versus global-centered spread, not mean distance.

| Examined capture | RGB global / joint mean fraction | Thermal global / joint mean fraction | RGB/thermal total / centered MSE | Ordering reversals /20 |
|---|---:|---:|---:|---:|
| original_rain | 0.36% / 6.97% | 4.69% / 62.37% | 211.1 / 209.3 | 0 |
| original_day | 2.71% / 49.79% | 34.08% / 94.99% | 0.4475 / 0.7147 | 3 |
| original_night | 3.53% / 67.77% | 25.88% / 84.49% | 0.6399 / 0.8236 | 0 |
| exp063_rain | 2.99% / 57.73% | 27.41% / 94.99% | 0.3228 / 0.3933 | 3 |
| exp064_day | 0.38% / 4.14% | 11.34% / 44.63% | 1309 / 1540 | 0 |
| exp064_night | 5.06% / 64.34% | 27.76% / 94.43% | 0.3542 / 0.5026 | 0 |
| exp064_rainy | 1.75% / 21.26% | 37.30% / 84.42% | 1.221 / 1.834 | 4 |

In the original night capture, RGB/thermal MSE changes from0.6399 to0.8236 after global centering, yet RGB remains lower in all20 frames. In the newer night capture it changes from0.3542 to0.5026, again lower in all20. Thermal's larger global offset accounts for part of those squared-error gaps, while the centered components retain a difference. This does not explain why the learned selector failed.

Thermal global mean fractions range from4.69% to37.30% across capture medians; joint range/grid mean fractions range44.63%–94.99%. Corresponding RGB fractions are0.36%–5.06% globally and4.14%–67.77% jointly. For original rain and EXP064 day, huge RGB squared errors remain after global centering (paired RGB/thermal centered ratios209.3 and1540). A single constant offset is therefore an inadequate account of those outcomes. No inference about tail causes follows from this identity.

The supplied-reference Z residual has consistent signed structure:

| Capture | RGB median mean Z residual (m) | Thermal median mean Z residual (m) | RGB / thermal positive-mean-Z frames |
|---|---:|---:|---:|
| original_rain | 14.442 | -2.088 | 20 / 0 |
| original_day | -1.769 | -12.059 | 3 / 0 |
| original_night | -1.557 | -6.164 | 0 / 0 |
| exp063_rain | -1.979 | -11.258 | 0 / 0 |
| exp064_day | 11.505 | -1.800 | 20 / 0 |
| exp064_night | -2.172 | -11.192 | 0 / 0 |
| exp064_rainy | 1.630 | -8.544 | 18 / 0 |

All140 thermal frame means have negative Z residual. RGB has negative mean Z in both night windows, but positive means in all original-rain and EXP064-day frames. This is a coordinate/reference-specific observation, not proof of physical under/over-ranging or a calibration offset; acquisition, correspondence, visibility, reference production and scene distribution remain possible contributors.

Both endpoint anchors preserve the reported capture-level total/centered median ratio directions. Strict reversal counts remain0,3,0,3–4,0,0,4 in table order (EXP063 changes from3 to4 for RGB endpoint). Sparse groups are retained: median nonempty joint-cell counts are19.5–30/54 across captures. More cell means necessarily absorb more empirical variation, including sampling effects; a high joint fraction is not a significance result or evidence of an exploitable correction. No IID, repeat-measurement stochastic covariance, physical ground truth or causal weather assumption is made.

## Validation and preservation

Independent audit reconstructs residuals from the saved XYZ arrays, independently assigns every range/grid group, checks every populated group's signed means, second moments, mean distances and centered spread, recomputes partitions with direct between-mean deviations, and verifies medians/strict comparisons and280 input hashes. PASS:502,138 scalar/hash/count checks, all140 frames and240,003 pairs. Its second-moment formula differs from the runner's centered-residual formula. Empty/zero/constant/opposing-offset synthetic fixtures pass. Plot inspected. Checks validate arithmetic and preservation, not physical reference accuracy.

Scripts: `ms2_signed_residuals.py`, `validate_ms2_signed_residuals.py`, `plot_ms2_signed_residuals.py` in `MAC-VO-main/Scripts/Setup`. Results: `results/ms2-signed-residuals/run-001/{protocol.json,report.json,validation.json,synthetic-validation.json,figures/}`. Run with `/root/ore-ms-analysis-venv/bin/python -m Scripts.Setup.ms2_signed_residuals --root /root/AirStack/ore-ms --output <fresh-output>` from the MAC-VO root, then the independent validator with `--run <output>`.

Private EXP065 backup includes outputs, scripts, notebook and git bundle. Parent saved arrays remain in verified EXP057/058/063/064 backups. No credentials, checkpoints or full transport archives included.

## Research decision

Keep current defaults and reserve the last two recordings. Do not fit a reference-assisted offset on these frames and call it a deployable improvement. A useful next bounded diagnostic is disparity-domain signed error and uncertainty behavior on the already saved raw frontends: it can separate inverse-depth tail amplification from score ordering without attributing physical causes. First verify which saved outputs support that comparison and keep recording-level results explicit. Independent geometric references would still be needed for physical-error attribution.
