# RGB second-pass optimization control

This finite comparison separates a fixed-rotation constraint from the benefit of
an additional RGB optimization pass. It implements the [resume specification](next-optimization-budget-control-2026-11-15.md)
on the qualified night continuation only. The population and references have been
historically exposed; this is a development mechanism test, not blind validation.

## Frozen population and arms

Use every night interval 599→600 through 1198→1199 of `_2021-08-13-22-16-02`.
No interval selection, thermal factors, new inference, training or parameter search.
For each pair, reconstruct accepted RGB factors with the historical native parent
pose using the unchanged `native_pose_feedback.feedback_input` implementation.
Run the unchanged native disparity solver for the first pass. Commit its solution
to float32, matching native writeback. Initialize both independent second-pass
graphs at that identical committed solution and retain identical source points,
covariances and observations. Each pair uses its historical parent; no arm's new
output feeds another pair. This intentionally isolates the solver mechanism from
continuous feedback histories in the earlier continuation experiment.

Arms: native first pass, additional full 6DoF RGB solve, additional translation-only
3DoF RGB solve at exactly the first-pass quaternion. Reuse `FixedRotationGraph`
and `native.optimize`; no changes to native graph, robust loss, precision or solver.
Both second passes use the existing maximum ten scheduler steps, patience two,
decreasing threshold 1e-5, Huber delta .1, PINV, TrustRegion radius 1e3 and minimum
1e-6. Native analytic full-pose representation has seven stored coordinates with
six tangent dimensions. Keep that implementation; do not substitute a different objective.

## Qualification and failure handling

Before capture, check the three first consecutive pairs 599, 600, 601: all input
hashes match prior pins; first-pass relative motions reproduce native event deltas
within 1 mm / 1e-4 rad; full and constrained graphs have identical initial residuals
and covariance arrays, and constrained Jacobians equal the full translation columns.
Check constrained finite differences at step 1e-6 against a fixed 1e-4 relative bound.
Record this qualification separately from population outcomes. No reference scores
are opened in qualification or capture. Reference paths/hashes can be bound first.

Capture freezes protocol, runner, unchanged dependency source and input/reference
hashes and numerical runtime before predictions. Every first pass in the population
must pass the same reproduction bound for qualified aggregate claims. Preserve any
failure and report it; never remove a pair or waive a bound. Failed second passes
remain unavailable on the full 600-pair population; matched-support summaries must
state their counts. Keep partial files and refuse overwriting run directories.

## Measurements and endpoints

Wrap the unchanged solver call with wall and process CPU timers. Record all returned
scheduler losses and actual step counts; alternate second-pass execution order by
pair to balance temporal drift. Qualification warms the solver. Times include
solver setup, weighting and graph conversion inside `native.optimize`, but exclude
graph construction and initial parity instrumentation. Record first-pass time
separately; second-pass comparisons and combined costs are both reported. These
are cached CPU solver costs, not frontend or end-to-end VO costs. No equal-budget
or statistical timing equivalence follows from equal iteration caps.

Lock all predictions before reference scoring. On identical available support,
report per-interval translation-vector error norm and relative rotation angle:
mean, RMSE and p95. Reference deltas are T_source^-1 T_target using the existing
proper-SO3 serialization screen. Also compose the fixed-pair increments from identity
and report 601-pose position/orientation mean, RMSE and p95 without scale/alignment
or gap fill. This path remains a fixed-input replay, not continuous new VO.
Report full 600-pair availability, native reproduction errors, fixed-quaternion
identity, CPU totals/medians/p95, steps and any failures for all three arms.

No significance, physical sensor calibration or novel architecture claim is made.
If full-pose refinement matches the constrained accuracy gain, credit extra
optimization. If the constraint offers better accuracy, compare measured costs and
retain the scope and established decoupling prior art. Mixed T/R endpoints remain
mixed; no post-score weights, solver rescue, successful-window selection or automatic
new recording follows. An independent saved-output audit must recompute transforms,
support, statistics, source locks and cost accounting before accepting the finding.
