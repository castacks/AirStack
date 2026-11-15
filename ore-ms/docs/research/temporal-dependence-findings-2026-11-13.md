# Temporal dependence: known-covariance linearized feasibility

Run at 2026-11-13T21:03:12.310313-05:00. Branch ore-ms-orb-development, baseline a79658f63,
working source/protocol SHA pinned before execution. No prior attempt overwritten.

**PASS_KNOWN_COVARIANCE_LINEARIZED_MECHANICS_ONLY.** Four fixed cases,
4096 trials each,24 landmarks and4 poses (24 tangent-state components).
Exact known synthetic noise, float64 linearized pinhole motion observations.
This is generalized least-squares validation, not a new estimator or learned
covariance method, real-data result or published-system reproduction.

| Case | Method | Normalized pose risk | 95% joint coverage |
|---|---|---:|---:|
| independent | independent | 0.254690 | 95.26% |
| independent | inflation4 | 0.254690 | 100.00% |
| independent | joint_oracle | 0.254690 | 95.26% |
| independent_increments | independent | 0.924627 | 89.28% |
| independent_increments | inflation4 | 0.924627 | 100.00% |
| independent_increments | joint_oracle | 0.606604 | 95.29% |
| correlated_increments | independent | 1.327335 | 87.11% |
| correlated_increments | inflation4 | 1.327335 | 100.00% |
| correlated_increments | joint_oracle | 0.984416 | 95.00% |
| bias_control | independent | 1.285844 | 0.00% |
| bias_control | inflation4 | 1.285844 | 0.00% |
| bias_control | joint_oracle | 0.976802 | 0.00% |

Independent endpoints give identical point estimates. Accumulated independent
increments already create dependent positions: the joint oracle reduces analytic
normalized risk34.09%; with correlated increments the analytic reduction is25.47%.
These percentages are expected risk under the exact declared model, not measured
physical VO improvement. Their empirical counterparts are in the table.
Fourfold global covariance inflation changes uncertainty but not point estimates.
All unbiased oracle coverages fall in the frozen93–97% band. The biased case has
zero95% joint coverage in all three methods: correct centered noise covariance
cannot remove an unmodeled mean shift. This is a deliberate negative control.

The previous track code's independent-*increment* approximation must not be
confused with independent-*observation* weighting. Even under independent
increments, C[t,s] is generally nonzero for accumulated positions. Under a
first-order recursion, retain cross blocks by C[t,s]=F[t] C[t-1,s] when the new
increment is independent of prior errors. Shared image/network increment
correlations require additional terms and are not identified by this test.

Mechanics pass: positive-definite noise, full24-state rank, exact noiseless
recovery, whitened solve/direct precision agreement and inflation invariance.
A separate post-result auditor imports no producer and passes197 checks:
eigendecomposition precision reconstruction, saved covariance, empirical risk,
coverage/NLL, analytic risk, mean-bias sampling bound and increment/position
cross-block consistency. It is not blind or independent data generation.

## Consequence and next boundary

The numerical mechanism is feasible. A realizable next candidate is analytic
cross-time covariance propagation from saved track Jacobians/increment marginals,
not estimation of arbitrary correlations from one trajectory. Before real-data
use, derive the exact residual covariance after source-ray/depth conditioning,
compare against the current factor objective and verify that saved fields suffice.
Otherwise a propagated position covariance could be attached to the wrong residual
or double-count uncertainty. No real-data solver, fitting or promotion is
implicitly authorized by synthetic success. That candidate needs its own freeze.

The current joint arm uses true simulator covariance unavailable to an estimator
in deployment. No nuisance/geometry model, covariance estimator or missing
cross-sensor timing assumption is validated. The assay is linearized; nonlinear
state updates, selection, missing tracks, bias correction and generalization
remain outside its scope. No outcome-driven parameter changes or repeat run.

Protocol: `experiments/temporal-dependence-freeze-2026-11-13.json`.
Producer: `tools/temporal_dependence_feasibility.py`.
Independent saved-output audit: `tools/audit_temporal_dependence.py`.
Artifacts: `results/temporal-dependence/run-001`.
Prior-art and motivation: [gap shortlist](gap-shortlist-2026-11-13.md).
No new source-paper reading or official code execution in this assay.
