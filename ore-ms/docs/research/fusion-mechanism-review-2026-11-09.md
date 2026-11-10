# Repetition review and the next error-source experiment

9 November 2026, America/New_York. Branch `ore-ms-orb-development`; decomposition `cbe284d5a`. The user asked whether recent experiments repeated earlier work. They substantially did: recent changes isolated components but reused familiar uncertainty, geometry and residual cues. The proposed additional directional score is deferred. This review is a retrospective diagnostic and research decision, not a new accuracy method.

## What was already covered

| Mechanism | Earlier evidence | Consequence for the next step |
|---|---|---|
| Native residuals and conditioning as camera-quality cues | EXP079–080, including reprojection/disparity/whitened residual and conditioning rules | Renaming these cues is not a new direction. |
| Factor geometry, depth/spatial support and leverage | EXP081; native objective/solver behavior EXP082–086 | A further geometry diagnosis needs a specific intervention, rather than another score. |
| Learned versus fixed covariance weights | EXP009: late thermal translation improves with fixed pixel weights, but rotation worsens; depths/matches remain fixed | Weight effects are already demonstrated on older windows; they do not identify the mean-error source. |
| Disparity bias, depth amplification and scale | EXP059/066/069/070 | Mean bias is already a known candidate. EXP070 changes risk/calibration scores, explicitly not predicted point means. |
| Covariance learning/adaptation | EXP095–109, including the fixed-mean objective and conservative covariance-head adaptation | Updating confidence while preserving the mean has already been explored. |
| Full covariance and fixed-native calibration | Recent replay and native-information comparisons | Direction/cross terms have already entered pose weighting. |
| Camera selection versus translation fusion | Latest cue and component-fusion runs | Changing the target or holding RGB orientation fixed is a useful ablation, not a new representation. |
| Stereo mean/geometry controls in another backend | ORB EXP145–152, including baseline initialization repair and remaining depth/travel shortfall | Relevant context, but not a causal result about MAC/MACThermal. |

This comparison uses local experiment records, not a claim that all possible implementations have been exhausted. The reviewed files and hashes are retained in the machine-readable review. Older captures, backends and references differ; their numbers are not pooled with the current prefixes.

## What the current translation errors show

Used all 590/551/594 paired native intervals from the exposed day/night/rain prefixes. Errors are common-frame matrix translation minus supplied reference translation. No prediction is corrected, no new selector is fitted, and no frontend/optimizer run occurs.

The share below is squared-error energy parallel to the **per-interval reference motion direction**, not a camera-axis statistic. The ratio is the median of per-interval translation-norm ratios; it is not accumulated travel or a fitted scale.

| Recording | Thermal motion-axis share of translation MSE | Median thermal/reference motion norm | Thermal mean signed motion-axis error (m) | Centered RGB–thermal error-vector correlation* |
|---|---:|---:|---:|---:|
| Day | 96.1% | 0.673 | −0.130 | +0.075 |
| Night | 89.0% | 0.444 | −0.595 | −0.116 |
| Rain | 97.9% | 0.450 | −0.359 | +0.608 |

*Normalized aggregate centered dot product, not per-axis Pearson correlation or an independent-sample significance result.

Thermal error is predominantly longitudinal and has negative average projection onto reference motion in every prefix. A small constant sensor preference cannot by itself establish why motion magnitude is short. Candidate causes include stereo mean/depth, correspondences, optimization and the nominal time/reference model; this decomposition does not identify which is physical.

The error relationship also changes by recording. In day and night the mean RGB/thermal error vectors oppose each other (dot products −0.001412 and −0.017881 m²). In rain they align (+0.073662 m²), and centered errors are positively correlated. RGB rain motion is short too (median native/reference norm 0.805). Thus apparent fusion headroom is not evidence of two unbiased independent estimators. Reference conventions and correlated failure can contribute to the observed pattern.

For a constant thermal weight `a`, exact translation MSE is `A + 2aB + a²D`, with `B = mean(e_RGB · (t_thermal − t_RGB))`. Decomposed B into mean-error and centered terms; all identities are checked. The mean-error contribution to B is negative in day/night and positive in rain. This makes bias cancellation one contributor, without explaining all interval-dependent gains.

Even a **same-record, reference-fitted MSE-optimal constant weight** does not solve the reported mean-error objective in rain:

| Recording | Oracle constant weight | RGB mean error (m) | Same-record constant mean error (m) |
|---|---:|---:|---:|
| Day | 0.1143 | 0.046297 | 0.042067 |
| Night | 0.0389 | 0.077470 | 0.071083 |
| Rain | 0.0906 | 0.229342 | 0.234091 |

That privileged rain fit lowers MSE slightly while worsening mean error. The loss mismatch was already a limitation of the previous experiment; this calculation quantifies it rather than offering it as a new discovery. No same-record fit is presented as an executable result.

## Decision

Do not launch another covariance projection, residual gate, bias/scale score or ridge-weight variant as the next main experiment. Preserve the recent outcomes and RGB baseline. The useful next question is whether changing **stereo/depth means**, with matches and covariance controls held fixed, changes the thermal motion error.

The next intervention is the [depth-mean × covariance ablation](next-experiment-depth-mean-ablation-2026-11-09.md). Its reference-assisted arm is an error-source diagnostic, not a proposed deployed estimator. This differs from EXP009's fixed-depth weight intervention and EXP070's unchanged-point risk calibration. Mean bias itself is not a new hypothesis; the missing evidence is the controlled motion-level consequence of changing the depth means under matched controls.

If that arm does not recover motion accuracy, do not start stereo-mean training based on this shortfall alone. Inspect correspondence/solver/reference effects using the paired outcomes. If it does, a mean/frontend intervention becomes justified; shared representations remain an option when tied to that mechanism. No broad RGB–thermal research stop follows either outcome.

## Verification and limits

Four analytic tests pass: opposing constant bias, centered anticorrelated noise, identical shared bias and zero-motion/invalid input handling. An independent standard-library scalar reconstruction checks 60 quantities across all three records, pair identities and six source hashes; it imports no producer. MSE/bias/centered and radial/perpendicular identities pass, and per-interval convex oracle results reproduce the earlier packet. No new physical-reference qualification, heldout evidence, inference, runtime metric or trajectory score is claimed.

Raw decomposition and audit: `results/fusion-mechanism-review/run-001`. The [machine-readable review](../../experiments/fusion-mechanism-review-2026-11-09.json) records evidence hashes, complete summaries and the next-step status. Companion Box backup contains the inputs, notebook and receipts.
