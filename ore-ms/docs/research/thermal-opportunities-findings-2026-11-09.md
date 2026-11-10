# Where thermal can improve native pose accuracy

9 November 2026, America/New_York. Branch `ore-ms-orb-development`. Experiment/model freeze `3ac4380f5`; independent audit `46ea00a3e`.

Thermal offers more translation headroom than rotation headroom on these three exposed prefixes, and the opportunities often conflict. A perfect translation-based whole-pose selector lowers equal-record mean translation error by 9.97% but raises rotation error by 21.35%. The tested reference-free rotation cue model does not improve accuracy. The next experiment should isolate translation while retaining native RGB orientation.

## What was measured

Reused the unchanged native means and provided-reference errors from the first 600 source groups of day/night/rain. No new frontend/optimizer calls, neural training or runtime measurements. Every prefix has 599 expected intervals; paired native availability is 590/551/594. RGB-only counts are 9/47/5, with one night interval unavailable in both. These are development-exposed data, including prior reference inspection.

Reference-dependent oracles select one complete native pose per interval: lower rotation error, lower translation error, or thermal only when neither metric worsens (joint oracle). Each resulting pose is evaluated on **both** metrics. They measure available selection gains, not executable method performance or bounds on every possible fusion/frontend improvement.

Wins use an absolute 1e-12 tolerance in degrees/metres. Runs require consecutive pair indices and matching source endpoints; missing/losing intervals break runs. Durations sum actual common intervals rather than assume a fixed frame rate. Sensor capture rates differ; these runs describe the existing common-interval construction, not independent image events.

## Opportunities and tradeoffs

| Recording | Paired intervals | Thermal rotation wins | Thermal translation wins | Thermal wins both | RGB wins both |
|---|---:|---:|---:|---:|---:|
| Day | 590 | 16 | 163 | 4 | 415 |
| Night | 551 | 22 | 39 | 4 | 494 |
| Rain | 594 | 65 | 159 | 13 | 383 |

There are 21 joint thermal wins among 1,735 paired intervals. The remaining opportunities include 82 thermal-rotation/RGB-translation tradeoffs and 340 RGB-rotation/thermal-translation tradeoffs; no ties occur at the stated tolerance.

| Recording | Longest thermal rotation-win run | Longest thermal translation-win run |
|---|---:|---:|
| Day | 1 interval / 0.133 s | 11 intervals / 1.133 s |
| Night | 2 intervals / 0.200 s | 10 intervals / 1.000 s |
| Rain | 4 intervals / 0.400 s | 43 intervals / 4.333 s |

Rain has a sustained translation opportunity; rotation wins are much sparser. This points toward testing motion components separately rather than assuming a single camera is best for the entire pose.

Equal-record averages of mean relative errors on each record's paired support:

| Selection | Rotation (deg) | Translation (m) | Interpretation |
|---|---:|---:|---|
| Native RGB / training-best-fixed | 0.029798 | 0.117703 | Reference baseline |
| Native thermal | 0.130283 | 0.372243 | Fixed camera control |
| Rotation oracle | 0.028797 | 0.129877 | Rotation −3.36%; translation +10.34% |
| Translation oracle | 0.036161 | 0.105974 | Translation −9.97%; rotation +21.35% |
| Joint oracle | 0.029611 | 0.116351 | Rotation −0.63%; translation −1.15% |
| **Reference-free cue model** | **0.030879** | **0.117730** | Rotation +3.63%; translation +0.023% |

The separate metric minima are not a single oracle whole-pose result. The joint oracle shows how little simultaneous benefit is available from choosing whole poses under a per-interval no-regression requirement.

## One fixed cue prediction test

Before execution, froze eight saved, reference-free features: log thermal/RGB ratios of factor count, native residual mean-square, translational/rotational conditional covariance traces and marginal condition numbers; and log1p relative-pose rotation/translation disagreement. Information matrices retain the previous fixed-landmark, unrobust, undamped local approximation; they are not calibrated native pose posteriors. Native residuals and sensor pixel scales differ; these are candidate learned cues rather than physically comparable noise estimates.

For each target recording, fitted a single ridge regression of RGB-minus-thermal rotation error using the other two recordings. Training records have equal total weight; standardization uses only their rows. Fixed penalty lambda=1, unpenalized intercept, thermal iff predicted gain is positive, no hyperparameter search, threshold tuning, temporal smoothing or retry. Target references and previous score reads are blocked during fit/application. All three prediction files were locked before this scoring. This is retrospective recording-excluded development, not new-record validation.

| Recording | Thermal selections | Actual thermal rotation wins among selections | RGB mean rotation → cue (deg) | RGB mean translation → cue (m) |
|---|---:|---:|---:|---:|
| Day | 202 | 7 | 0.017298 → 0.020205 | 0.046297 → 0.046384 |
| Night | 0 | 0 | 0.030447 → 0.030447 | 0.077470 → 0.077470 |
| Rain | 50 | 0 | 0.041648 → 0.041985 | 0.229342 → 0.229336 |

Training-best-fixed selects RGB in every fold. The cue model makes 252 paired thermal selections, only seven correct for its rotation target. It preserves native union availability (599/598/599); all paired rows have valid cues, so no missing-cue fallback is exercised. No adaptive method is adopted. This result applies to this feature set, target and model; it does not establish that uncertainty or shared representations cannot predict useful opportunities.

## Verification and limits

Seven tests pass: oracle tradeoffs/ties, source-gap run boundaries/durations, missing-estimate denominators, feature label independence, exact equal-record ridge fit, recording identity rejection, availability fallback and the inherited filesystem label guard. The initial ridge fixture incorrectly expected zero slope despite its changing pose-disagreement feature; corrected before freezing or execution, with the first log preserved.

Independent SciPy/NumPy reconstruction imports neither producer nor scorer. It verifies 7,062 native error components, all 1,797 selection slots, 285 winning runs/durations, all three training normalizations and ridge solutions through augmented least squares, and paired/oracle aggregate metrics. Prediction and source hashes remain frozen. See the [protocol](../../experiments/thermal-opportunities-freeze-2026-11-09.json) and [full outcome](../../experiments/thermal-opportunities-outcome-2026-11-09.json). Predictions, interval diagnostics and receipts are in `results/thermal-opportunities/run-001` and the companion Box packet.

Errors depend on supplied RGB references and nominal spatial/temporal alignment; physical ground truth, unseen-record transfer and full-trajectory accuracy remain unestablished. No statistical independence or significance is inferred from adjacent intervals. Small headroom on these prefixes does not describe unobserved RGB degradation or place a bound on improved thermal estimation or fusion outside complete-pose selection.

## Next accuracy experiment

Retain native RGB orientation and estimate/select only common-frame translation, using a translation-error target with recording-excluded fitting and a fixed RGB translation control. Score rotation and translation separately with all availability masks retained. First establish whether the persistent rain translation advantage can be predicted from saved cues without target labels; use an oracle only as a diagnostic bound. Freeze the component convention and controls before scoring; do not tune the failed rotation selector after these results.

If a credible interval gain appears, evaluate accumulated trajectories and transfer on new recordings. If the saved translation cues miss it, inspect the identified rain segments to choose a specific thermal depth/frontend or shared-representation improvement. MACThermal/MAC-I² uncertainty machinery remains relevant, but propagation itself is already present and is not a novelty claim.
