# Translation fusion with native RGB orientation

9 November 2026, America/New_York. Branch `ore-ms-orb-development`. Model/protocol freeze `98cfb8b40`; independent audit `042c42e61`.

The primary translation blend improves day but worsens night and rain. Its equal-record mean translation error is 0.46% higher than RGB, and it does not beat the training-fitted constant control overall. Native RGB orientation and availability are preserved. No adaptive method is adopted.

A useful additional finding is that **convex translation fusion has more available headroom than whole-pose selection**: the reference-dependent convex oracle reduces mean translation error by 19.47% while preserving RGB rotation. The tested cues/model do not recover that gain across recordings. This supports investigating the geometry of useful corrections rather than tuning this model on its scored records.

## Frozen experiment

Reused native RGB/thermal outputs for the first 600 source groups of day/night/rain. Both translations already share the previous common frame/interval mapping. Constructed valid rigid transforms by copying native RGB orientation and setting matrix translation to `(1-alpha)*t_rgb + alpha*t_thermal`. This is group-matrix translation blending, not blending full SE(3) logarithms. Saved poses are never reoptimized. Sole-camera output is retained; neither means unavailable. RGB orientation preservation applies wherever RGB exists.

Primary `ridge_blend` uses the previous eight reference-free features: relative factor count, residual mean-square, translation/rotation covariance trace and marginal condition number, and rotation/translation disagreement. The information readout remains conditional on fixed landmarks, without robust corrections or marginalization; it is not a calibrated native posterior.

In each fold, trained on only the other two recordings, with equal total recording weight and training-only normalization. Fit a linear thermal weight by minimizing weighted squared translation-vector error in metres plus a fixed lambda=1 slope penalty; the intercept is unpenalized. The fit is unconstrained, with predicted weight clipped to [0,1] at application. A secondary ridge selector predicts RGB-minus-thermal translation error and switches translation iff predicted benefit is positive. No parameter search, smoothing, threshold tuning or retry.

Controls: RGB, original thermal, thermal translation with RGB orientation, equal translation, an analytic training-fitted constant weight, and training-best-fixed translation camera. The last chooses RGB in every fold. Target references and prior score reads were blocked during fit/application. All predictions were locked before scoring. These recordings were already development-exposed; the exclusion does not constitute fresh validation.

## Accuracy

Mean translation error on identical RGB/primary intervals. All adaptive rotations equal native RGB exactly.

| Recording | Intervals | RGB (m) | Training constant (m) | Primary learned blend (m) |
|---|---:|---:|---:|---:|
| Day | 599 | 0.046604 | 0.043912 | 0.043270 |
| Night | 598 | 0.077280 | 0.079463 | 0.079815 |
| Rain | 599 | 0.231334 | 0.232760 | 0.233570 |

Day improves in mean, RMSE and p95. Night worsens in all three: primary RMSE 0.107152 m versus RGB 0.087493 m. Rain's RMSE/p95 improve slightly, but mean error worsens; that tradeoff is retained rather than reported as a general gain.

All-method common support is 590/551/594 intervals. Equal-record averages of mean errors:

| Method | Rotation (deg) | Translation (m) |
|---|---:|---:|
| RGB / training-best-fixed | 0.029798 | 0.117703 |
| Original thermal | 0.130283 | 0.372243 |
| Thermal translation + RGB rotation | 0.029798 | 0.372243 |
| Equal translation | 0.029798 | 0.221088 |
| Training-fitted constant | 0.029798 | 0.118062 |
| Translation selector | 0.029798 | 0.117752 |
| **Learned blend (primary)** | **0.029798** | **0.118244** |

Primary translation error increases 0.46% against RGB; the constant increases 0.30%; the selector increases 0.04%. A change of target/component alone did not make these cues transferable. The learned blend also falls behind the constant in night and rain.

Union availability stays 599/598/599. Thermal alone has 590/551/594 estimates. Sole-camera fallback counts are 9/47/5; one night interval remains unavailable. Every paired row has valid cues; no missing-cue fallback occurs. Six RGB-orientation-preserving methods give 10,776 exact orientation-copy checks. No new frontend/optimizer execution or runtime measurement occurred.

## Available fusion headroom and missed weights

The post-lock oracle chooses the error-minimizing convex translation weight from references for each paired interval; it is not a deployable method and not a bound on all possible estimators. Its rotation remains exactly RGB.

| Recording | RGB translation mean (m), paired | Convex oracle (m) | Oracle zero / interior / one weights |
|---|---:|---:|---:|
| Day | 0.046297 | 0.034142 | 155 / 344 / 91 |
| Night | 0.077470 | 0.056666 | 140 / 386 / 25 |
| Rain | 0.229342 | 0.193539 | 329 / 118 / 147 |

Equal-record oracle mean is 0.094782 m, a 19.47% reduction. The prior whole-pose translation selector offered 9.97% but harmed rotation; component fusion removes that specific coupling. An interior blend can cancel part of the error even where thermal alone is worse, so camera-win frequency understates fusion opportunity.

The primary assigns nonzero interior thermal weight to **every paired interval**: mean weights day 0.0955, night 0.0981, rain 0.0695. The oracle chooses zero in 155/140/329 intervals, including over half of rain's paired support. Those differences expose missed suppression of harmful corrections. Oracle weights are reference-dependent; they are not a prescription available at execution. This descriptive attribution was computed after scoring and did not alter any method.

## Verification and interpretation

Seven tests pass, covering exact component geometry, analytic equal-record fits, zero translation disagreement, recording identity exclusion, clipping, label-free application, missing cues/cameras and the inherited filesystem label guard. Independent numerical reconstruction imports neither producer nor scorer: all three models are recovered with augmented least squares; 10,410 paired component geometries, 28,614 error components and 1,735 oracle optima pass. All own/common mean, median, RMSE, p95 and maximum metrics are independently checked. Prediction/source hashes remain frozen.

See the [protocol](../../experiments/translation-fusion-freeze-2026-11-09.json) and [complete outcome](../../experiments/translation-fusion-outcome-2026-11-09.json). Predictions, scores, locks and receipts are in `results/translation-fusion/run-001` and the companion Box backup.

These are per-interval errors against supplied RGB reference products under nominal alignment. Physical ground truth, unseen-record transfer and accumulated trajectory improvement remain unestablished. Learned squared-loss weights and mean-error scoring need not optimize the same risk; the observed RMSE/mean differences remain explicit. The model is specific to metre units and the frozen regularizer. Aggregate failure does not rule out better geometry features, robust objectives or thermal frontend/shared-representation improvements.

## Next technical step

Investigate which **directional** properties distinguish helpful and harmful translation corrections: error-vector alignment for diagnosis, then reference-free covariance projected along RGB–thermal translation disagreement and residual consistency as candidate predictors. The tested trace/condition/norm features summarize magnitude and discard much directional structure. That observation motivates the next test; it does not yet prove direction is the missing predictor.

Keep this model frozen. First check the candidate geometry on the existing exposed intervals with recording-excluded assessment, rather than tune the current weights. A credible translation gain should precede trajectory evaluation and new-record tests. If geometry cues cannot distinguish the useful corrections, use the identified segments to choose a specific depth/frontend or shared-representation change. RGB remains the executable accuracy baseline.
