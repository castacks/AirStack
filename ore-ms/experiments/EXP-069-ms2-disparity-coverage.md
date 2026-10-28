# EXP-069: empirical disparity uncertainty coverage

Date:2026-10-28 America/New_York. Base8bc11c5e2, branch ore_ms.

Current uncertainty outputs do not yield uniform empirical Gaussian-style interval coverage across recordings or conditional groups. At nominal approximately95%, per-record median coverage ranges43.18%–91.61% for RGB and64.01%–99.12% for thermal in the four examined windows. A thermal near95% result retains signed bias, asymmetric tails and conditional differences. No probability-calibration guarantee or new selector is adopted.

## Fixed protocol

Same four complete saved-frontend populations from EXP066: EXP063 rain and EXP064 day/night/rainy, original IDs000000–000019,80 dependent frames,106562 both-valid pairs/213124 camera observations. Reused exploratory observations, not new held-out data. No model inference, fitting, clipping, new source acquisition, changed point estimate/default or reserved-capture access. The final two official-test captures remain untouched.

Use each camera's native own-endpoint depth-implied disparity reference, predicted disparity d and horizontal variance v returned by the unchanged inference interface. Define z=(d-d_ref)/sqrt(v); positive z means predicted disparity exceeds the supplied implied reference. Evaluate untruncated symmetric intervals d±k sqrt(v), inclusive boundaries, k=1,1.96,2,3. Hypothetical zero-mean standard-normal benchmark coverages are erf(k/sqrt2), approximately68.27%,94.9996%,95.45%,99.73%; these are comparison conventions, not an assumed proven data distribution or historically verified checkpoint training contract.

For every frame/camera, retain all points and11 groups: all, six unchanged midpoint-reference Euclidean ranges [0,5,10,20,40,80,infinity), and four within-frame predicted-variance bins using linear .25/.5/.75 quantiles. Exact ties stay together by right insertion, so bins are not guaranteed equal-sized; empty groups explicit, no minimum count filter. Reference ranges are label-assisted diagnostics, not inference inputs. Save coverage and signed tail fractions, lower-nonpositive interval endpoints, mean/quantiles/second moments of z, centered z spread, predicted mean variance, observed disparity MSE and descriptive univariate Gaussian NLL. No cross-camera comparison of native pixel NLL or variance magnitudes is made.

All reported entries below are medians of20 per-frame quantities; no pooling between captures or cameras. Marginal medians of components need not add even when each per-frame identity holds. Range/variance-group summaries separately give nonempty-frame counts and min/median/max point counts.

## Results

| Reused capture | Camera | ±1σ coverage | ±1.96σ coverage | Mean z | Mean z² | Nonpositive lower disparity at1.96σ |
|---|---|---:|---:|---:|---:|---:|
| exp063_rain | rgb | 21.91% | 43.18% | 1.214 | 30.773 | 0.17% |
| exp063_rain | thr | 35.59% | 64.01% | 1.755 | 4.906 | 0.28% |
| exp064_day | rgb | 39.69% | 63.10% | 0.500 | 7.552 | 4.93% |
| exp064_day | thr | 84.13% | 99.12% | 0.473 | 0.513 | 14.13% |
| exp064_night | rgb | 72.25% | 91.61% | -0.100 | 1.246 | 22.99% |
| exp064_night | thr | 82.41% | 92.15% | 0.634 | 0.983 | 49.74% |
| exp064_rainy | rgb | 62.06% | 84.84% | -0.237 | 2.661 | 12.33% |
| exp064_rainy | thr | 72.93% | 95.11% | 0.714 | 1.025 | 29.79% |

Newer-rain thermal is close to95% at1.96σ (95.11%) and mean z² is1.025, yet mean z is0.714 and every frame's mean z is positive. No standardized residual falls below-1.96 in any of its20 frames; the positive-tail median is4.886%. A centered Gaussian benchmark would instead have approximately2.5% in each tail. Those marginal matches therefore do not establish a centered Gaussian residual law, conditional calibration or a probability guarantee. Newer-night thermal is similarly signed: mean z0.634 and zero negative1.96σ-tail mass in all20 frames, while marginal coverage is92.15%.

Variance conditioning remains material:

| Capture | Camera | Variance bin1 coverage | Bin2 | Bin3 | Bin4 |
|---|---|---:|---:|---:|---:|
| exp063_rain | rgb | 13.63% | 23.89% | 53.62% | 74.44% |
| exp063_rain | thr | 53.93% | 63.81% | 67.33% | 75.16% |
| exp064_day | rgb | 34.05% | 52.74% | 75.40% | 90.25% |
| exp064_day | thr | 98.36% | 99.80% | 99.68% | 100.00% |
| exp064_night | rgb | 71.71% | 96.46% | 98.39% | 99.70% |
| exp064_night | thr | 74.19% | 95.86% | 100.00% | 100.00% |
| exp064_rainy | rgb | 51.63% | 92.86% | 98.10% | 97.79% |
| exp064_rainy | thr | 89.84% | 96.01% | 97.59% | 100.00% |

All four variance bins are nonempty in all20 frames for each of these camera/capture populations; these are per-frame native-score quantile cuts rather than common cross-camera thresholds. For newer-night RGB,1.96σ coverage rises from71.71% in the lowest-variance bin to99.70% in the highest; newer-rain RGB ranges51.63%–98.10%. A single whole-image coverage number conceals those differences. This observation is not a fitted repair or evidence that a particular recalibration transfers.

Reference-range results also vary. Newer-night thermal median1.96σ coverage is100% at5–10m and10–20m but49.57% at>=80m, all three bands nonempty20/20. Original EXP063 rain thermal>=80m is15.56%. Sparse counts and frame availability remain explicit, particularly newer-day0–5m (one nonempty frame) and far ranges. Dependent spatial pixels/frames and range-selected labels do not supply independent significance tests.

The lower disparity endpoint can be nonpositive even though point predictions and supplied implied disparities are positive. At1.96σ, median fractions are22.99%/49.74% for newer-night RGB/thermal and12.33%/29.79% for newer-rain RGB/thermal. These intervals were not clipped or inverted into finite depth intervals; a nonpositive lower endpoint cannot silently yield a finite positive depth upper bound. This is an interval-geometry diagnostic, not validation of a positive-disparity distribution or depth coverage.

## Validation and artifacts

Independent raw/native-reference audit PASS:698988 scalar/hash/count checks,162 runner input hashes plus492 additional raw/reference/support/calibration/manifest hashes. It reads original frontend fields and native uint16 depth PNGs, reconstructs implied disparity, computes interval inclusion directly in disparity space (rather than the runner's absolute-z comparison), independently constructs ranges and variance bins with sorted interpolation/threshold counts, uses math.fsum for moments/NLL, and recomputes all medians/availability and per-point z/bin arrays. Per-frame coverage+signed-tail fractions=1 and E[z²]=E[z]²+centered spread hold. Hashes are rechecked after processing.

Nine analytical fixtures PASS: inclusive boundaries at four widths, symmetric coverage/tails, zero-residual NLL, constant-score tied/empty bins, linear quantile cuts and explicit empty group. Figures inspected. Validation establishes arithmetic and preservation against supplied references, not physical reference accuracy or uncertainty calibration under independent ground truth.

Scripts: `ms2_disparity_coverage.py`, `validate_ms2_disparity_coverage.py`, `plot_ms2_disparity_coverage.py` in `MAC-VO-main/Scripts/Setup`. Results: `results/ms2-disparity-coverage/run-001/` with protocol/report,160 derived z/bin NPZs, independent/synthetic validations, runtime and PNG/PDF figure. From MAC-VO root, use the analysis venv with `python -m Scripts.Setup.ms2_disparity_coverage --root /root/AirStack/ore-ms --derived /root/AirStack/ore-ms/results/ms2-disparity-diagnostics/run-001 --output <fresh-output>`; validator takes `--root ... --run <output>`. Parent derived fields require verified EXP066; native source/frontends/support/calibration require EXP063/064. Private EXP069 backup includes outputs/code/docs/notebook/bundle, excludes credentials/checkpoints/full archives.

## Decision

Preserve current defaults and distinguish signed bias, uncertainty scale, ranking and camera selection. Neither near95% marginal coverage nor near-unit mean z² proves calibration, a centered Gaussian likelihood, physical error correctness or transfer. EXP068's available-code mismatch remains a separate source fact, not an established cause of these observations. The supplied reference chain, training membership and thermal exact recipe remain limits.

Before further fits, consolidate an explicit calibration target and recording-level development/evaluation protocol: compare scale-only and signed-bias-plus-scale controls separately, judge conditional as well as marginal coverage and selection loss, retain positivity/availability failures, and label any reused-recording comparison retrospective. Do not spend the final two captures on an undeclared recalibration or claim physical sensor calibration from these references. No further source upload or paper access is needed for this result.
