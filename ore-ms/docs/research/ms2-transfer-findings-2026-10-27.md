# MS² cross-camera selection: findings and next decision

Consolidated2026-10-27 America/New_York through EXP066. This note separates the earlier within-camera ranking result from the cross-camera method tests. It summarizes completed work, not a new inference run or an additional test set.

The measured uncertainties contain useful selection information in some tested populations, but the current fitted camera selectors do not transfer consistently across captures. The prospective rain result is positive; the day and night counterexamples prevent a general selector claim. Keep the existing defaults and preserve both outcomes.

## What changed in the evidence

[EXP046–054 synthesis](ms2-findings-2026-10-26.md) supports within-camera uncertainty ranking against supplied stereo/depth references under several fixed depth/location/loss controls. That is a different test from selecting RGB versus thermal at nominal shared points.

[EXP056–059](../../experiments/EXP-059-ms2-score-failure-diagnosis.md) established concrete cross-camera score failures, including night choices worse than both fixed-camera baselines. Camera-wide scale removal and reference-depth Jacobian interventions did not provide a universal repair. These reference-assisted interventions were diagnostics, never deployment methods.

[EXP060](../../experiments/EXP-060-ms2-camera-scale-transfer.md) tested a fitted relative score multiplier with recording exclusion. Its changes helped some comparisons and hurt others, retaining night failure. [EXP061](../../experiments/EXP-061-ms2-error-structure.md) showed why one headline camera ranking is inadequate: thermal wins at nearer reference ranges and RGB at farther ranges in the tested night window; rain is especially tail-sensitive; mean-distance,median-point and relative-error estimands can favor different cameras. These are conditional patterns, not proven physical causes.

[EXP062](../../experiments/EXP-062-ms2-geometry-risk-selection.md) declared a small uncertainty/geometry model comparison. Combined prediction-based geometry/position and uncertainty improved excluded-night risk, while day/rain retained important failures. [EXP063](../../experiments/EXP-063-ms2-unexamined-recording.md) froze the models fitted on the original three captures before acquiring a new recording. Combined never beat uncertainty-only or always RGB there; strict losses and exact ties are preserved.

[EXP064](../../experiments/EXP-064-ms2-frozen-suite.md) prospectively declared the unchanged uncertainty-only control primary on three further captures. No coefficients,scalers,thresholds or training recordings changed. The complete declaration was separately backed up before scores. Its predeclared consistent-improvement criterion failed.

## The prospective suite

Ratios below1 favor uncertainty-only. Entries are medians of20 per-image paired mean3D-distance risk ratios, using the unchanged midpoint supplied-reference target and both-valid support.

| New capture | U-only/RGB | U-only/thermal | U-only/scalar-fit | Declared per-capture criterion |
|---|---:|---:|---:|---|
| Day `_2021-08-13-15-46-56` |0.1920|1.0355|0.7979|Not met|
| Night `_2021-08-13-22-03-03` |1.0709|0.5828|0.9822|Not met|
| Rain `_2021-08-06-16-59-13` |0.8692|0.6253|0.9149|Met|

The suite required every capture to improve over both fixed cameras and the frozen scalar-fit. Day loses to thermal (9 better/11 worse mean-distance frames; squared risk worse20/20). Night loses to RGB in all20 mean-distance frames. Rain improves over RGB18/20,thermal19/20 andscalar18/20 with one scalar tie; its median reductions are13.1%,37.5% and8.5%. Both endpoint anchors retain these directions. The positive rain result does not remove the other captures’ failures.

The primary model is a fixed learned decision from two log metric-variance inputs. Its fitted coefficients are both negative, so it is not a simple lower-variance policy or a probability-calibration guarantee. Model margins and full covariances have different meanings. All four families,original raw/camera-scale controls,losses,anchors and exact ties remain available; no favorable post-result replacement is adopted.

## Boundaries that remain material

- References/correspondences share the supplied MS² production chain. Nominal shared-point support and endpoint sensitivity do not certify physical common points,visibility,timing or absolute errors.
- All60 new suite frames have both-valid support, but244 day and966 rain candidates are thermal-only. Conditional risk does not evaluate a complete availability policy. The rain inspection also recorded moving foreground RGB occlusion before scores; no frames or pixels were removed to improve results.
- Large finite errors and tiny positive disparities are retained. Squared-loss ratios can therefore be strongly tail-sensitive. These differences do not by themselves label source reference products as wrong.
- Captures,adjacent frames,spatial pixels,losses and anchors are dependent. Statistical significance and weather-causal effects were not established. Source/checkpoint training membership is unknown.
- Source integrity,producer tensors,runtime parity and independent numerical audits check implementation/preservation. They do not certify physical reference accuracy. Model families were selected across stages; each stage’s declared new captures provide its prospective evidence.

## Next necessary work

[EXP065](../../experiments/EXP-065-ms2-signed-residuals.md) completed signed residual accounting on all seven examined captures (140 frames,240003 both-valid pairs). Thermal has larger empirical global and fixed range/grid mean components; removing the global mean still leaves RGB lower squared error in all20 frames of both night windows. Original rain and new day retain huge RGB centered-spread errors. All anchors and explicit empty strata are preserved; independent502138-check audit passes. These reference-assisted spatial summaries identify neither physical sensor bias nor the cause of transfer failure. No selector was fitted or changed.

[EXP066](../../experiments/EXP-066-ms2-disparity-diagnostics.md) completed the availability audit and disparity diagnostic on four complete saved-frontend captures (80 frames,106562 both-valid pairs). In EXP064 day RGB, a median0.684% of points with predicted/reference disparity<0.1 contribute99.05% of axial-depth squared error. Raw uncertainty ranks change with error domain: newer-night thermal variance correlates positively with disparity error² but negatively with depth error². Positive whole-frame propagated ranks also coexist with negative fixed-range ranks. Native-PNG/raw-field independent4512464-check audit passes. This separates score/target behavior without identifying physical causes or validating a new selector.

Before implementing another method, declare a distribution-aware disparity-to-depth risk representation and its tail assumptions, with existing camera-scale/geometry controls. Positive disparity alone does not ensure finite inverse-depth moments; a finite-error linearization comparison is not stochastic coverage validation. No further unchanged capture repeat or feature search is warranted.
Avoid spending the last two untouched official test captures on another unchanged repeat or choosing a more flexible model from these observed outcomes. A genuinely new method needs a declared target/model and reserved evaluation. Independent geometric-reference data such as a usable CATS sample would materially strengthen physical-error claims; access remains unconfirmed. Existing saved arrays suffice for the proposed diagnostic, so no immediate upload is needed.

The [targeted paper note](uncertainty-method-papers-2026-10-27.md) remains useful for distinguishing calibration,discrimination,error-distribution learning and transfer. No new paper-access blocker or author message is introduced by this synthesis.
