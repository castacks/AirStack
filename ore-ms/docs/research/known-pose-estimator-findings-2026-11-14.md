# Known-pose thermal-texture assessment: accuracy versus dispersion

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus frozen working sources.

## Question and complete result

Does changing native query selection alter true-pose error, centered dispersion,
or both? The six [qualified exact-geometry scenes](known-pose-thermal-replay-next-2026-11-14.md)
provide a target separate from the estimator's baseline output. The checkpoint,
NoDepth selector, covariance packet arithmetic, identity initialization and
explicit IRLS pose harness remain unchanged from the preceding paired study.

The run retains 780 three-pair dense batches (2,340 neural pair evaluations) and 1,560
two-arm fit attempts. Independent audit status: **PASS**, **31,999 checks**;
1560/1,560 fits converge, 0 invalid packets,
6 identical-input dense repeats verified. All 6 scenes are retained.

The table reports the common successful paired population, with own-population
metrics and every attempted status retained in the raw report. Translation and
rotation errors are norms of the true-target-relative left SE(3) Log components;
they are not raw translation-vector subtraction or trajectory ATE. The baseline
column is the unperturbed estimator error, not an assumed zero-error target.

|Artificial scene|Paired draws|Unperturbed translation error mm|Translation RMSE mm, fixed / regenerated|Rotation RMSE deg, fixed / regenerated|Regenerated/fixed variance trace|Translation RMSE change|
|---|---:|---:|---:|---:|---:|---:|
|day-plane4m|128/128|1.007|1.033 / 1.067|0.007360 / 0.008976|1.984×|+3.271%|
|day-two-depths|128/128|2.679|2.648 / 1.984|0.013085 / 0.011981|5.098×|-25.082%|
|night-plane4m|128/128|2.564|2.588 / 2.662|0.036510 / 0.038076|3.495×|+2.855%|
|night-two-depths|128/128|1.679|1.685 / 1.776|0.019888 / 0.020674|3.723×|+5.421%|
|rain-plane4m|128/128|1.103|1.138 / 1.450|0.023549 / 0.026636|3.889×|+27.425%|
|rain-two-depths|128/128|1.497|1.504 / 1.717|0.025000 / 0.026272|7.751×|+14.170%|

Regeneration increases centered normalized variance in 6/6 scenes,
and increases translation RMSE in 5/6 scenes. Variance and
translation-RMSE change have opposing signs in 1/6 scenes. These
are descriptive finite-ensemble comparisons, not significance tests or a
statement of universal superiority of either query policy.

The two-depth day-texture scene is a concrete positive accuracy case for
regenerated queries: translation RMSE drops from 2.648 to 1.984 mm (25.08%),
and rotation RMSE drops from 0.013085° to 0.011981° (8.44%). Meanwhile its
translation covariance trace rises 21.76× and rotation trace 11.43× (the combined
normalized six-dimensional trace rises 5.10×). Thus the opposite directions are
also present within the same translation and rotation coordinates, rather than
being an artifact of mixing translation error with a six-dimensional metric.
The other five scenes have increased translation and rotation RMSE. Preserve
both the positive case and the mixed overall result.

![Known-pose error and spread](../../results/known-pose-estimator-assessment/run-001/known-pose-error-versus-spread.png)


|Artificial scene|Translation covariance trace ratio|Rotation covariance trace ratio|
|---|---:|---:|
|day-plane4m|2.170×|2.144×|
|day-two-depths|21.756×|11.428×|
|night-plane4m|3.455×|3.398×|
|night-two-depths|4.337×|4.013×|
|rain-plane4m|10.608×|11.022×|
|rain-two-depths|10.580×|9.187×|

These block-trace ratios are descriptive arithmetic derived from the independently
audited full covariance matrices; they require no additional predictions.
The figure shows centered translation RMS spread, using the finite-population
variance factor (n−1)/n, alongside true-target translation RMSE.

## Bias and variability are separate quantities

All estimates are evaluated against the same exact target in each scene, using
x=Log(T_est T_true^-1). The empirical mean error vector b, centered sample
covariance C with ddof1, and second moment M obey the independently checked
finite-sample identity

M = ((n−1)/n) C + b bᵀ.

A single shared baseline information covariance supplies the six-dimensional
normalization metric for both arms in each scene. It is a declared comparison
metric, not a claim that this raw matrix is the true-error covariance. Physical
translation/rotation norms are also reported above. Transported readouts, using
the Log Jacobian at each true-target-relative estimate, are retained separately
for the secondary synthetic-target ellipsoid statistic.

Across the 12 matched scene/arm populations, squared mean error accounts for
98.8955–99.9366% of the normalized error second moment.
The full error vectors, covariance/second-moment matrices, trial IDs, baseline
errors and readout arrays are retained. The finite-sample identity does not
identify which prediction head or rendering artifact causes systematic error;
that mechanism has not been isolated by this comparison.

## Frozen controls and independent verification

The 54-check native-input qualification verifies rendered uint8-to-float32
scaling, grayscale channel/batch ordering, source/target stereo/temporal binding,
preview equality and analytic-field native packet/coordinate compatibility.
It performs no learned estimator accuracy test. The main execution freeze was
then bound before any learned predictions on these scenes.

Each scene has 128 paired PCG64 draws using fixed global seed 2026111402 and the
original selector seed 20261114, plus unperturbed baseline and identical-input
null. Independent half-LSB grayscale noise is added after rendering/preparation
and clipped to[0,1], then replicated over channels. This applies to the complete
canvas, including the explicit artificial zero-atlas boundaries. A left tensor
is reused in its stereo and temporal calls. Both query-policy arms consume the
same inferred dense fields for each draw; downstream admission is recomputed.

Predictions/packets/dense outputs are globally locked before true-pose scoring.
The checkpoint loads strictly without migration and remains unchanged. Numeric
thresholds, geometry, textures, noise, initialization and point-selection settings
are not changed after model outcomes. The native selector requests 200 points as
in the preceding harness; variable accepted counts are retained rather than
converted to a count-matched comparison.

The independent auditor reconstructs every source selection with SciPy/native
RNG semantics, backprojects depths and checks independent kernel/3D covariance
arithmetic, then tests stationarity/inverse information on exact serialized
solver inputs. Independent analytic SE(3) Log and Jacobian transport verify true
pose errors, readouts, every mean/covariance/second moment, error norms, secondary
containment, statuses and population denominators. Original packet/stationarity
tolerances are retained. No solver or scoring amendment follows the run.

Protocol: `experiments/known-pose-estimator-freeze-2026-11-14.json`.
Raw evidence: `results/known-pose-estimator-assessment/run-001/` and
`results/known-pose-estimator-qualification/run-001/`.

## Contribution and stopping decision

The preceding natural-pair study established query-policy sensitivity without
true natural-pose accuracy attribution. This controlled extension now measures
both synthetic true-target error and dispersion, so a variance change is not
silently interpreted as an accuracy change. The test supplies a reproducible
uncertainty-target assessment, not a new selection algorithm or verified
publication novelty.

The scenes are artificial plane/two-depth geometries with three already exposed
thermal image atlases. Texture labels do not establish physical day/night/rain
conditions, and 128 noise draws are conditional samples rather than independent
recordings. The known target solves the reference ambiguity for this artificial
problem; it does not establish sensor-noise calibration, natural generalization,
a native MACThermal benchmark or MAC-I² VI reproduction. Systematic error can
reflect this deliberately artificial input domain as well as estimator behavior.

Close this finite six-scene comparison without geometry/noise/NMS/budget/solver
rescue, head training or covariance-scale fitting. It does not close the broader
V3/I2/I4 questions or the other original-paper directions. A future correction
needs a deployment objective, prior-art distinction and accuracy/availability
controls beyond reduced spread alone.
