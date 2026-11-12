# Shared RGB–thermal motion differentiation: qualification incomplete

**Subsequent resolution:** [independent stationarity audit](shared-motion-stationarity-findings-2026-11-12.md) explains the failed gradient check and passes unchanged tolerances. This report retains the original outcome; no old result is overwritten.

12 November 2026. Branch `ore-ms-orb-development`. Main execution freeze
`1b95a8990`; independent-audit/near-degenerate supplement freeze `1d58df579`.

**Decision: STOP_GRADIENT_QUALIFICATION_INCOMPLETE_NO_TRAINING.** The synthetic
prototype passes its fixed 16-case suite and the supplementary near-degenerate
case. An independently implemented SciPy audit passes six of seven checks but
fails one gradient probe at its declared tolerance. Preserve the disagreement;
do not change tolerances, train a head or interpret this as a VO improvement.

## Implemented mechanism and limits

[Research decision](multisensor-research-reset-2026-11-12.md) selected learning
measurement influence through shared motion as a hypothesis. The isolated
`tools/shared_motion_feasibility.py` now estimates one body motion from two
cameras' UV/disparity residuals. Each camera retains its own intrinsics, stereo
baseline and body extrinsics. Fixed source body points condition the estimator;
source landmark uncertainty and stereo reconstruction errors are not modeled.
Optical XYZ conventions are explicit; this is not a native NED backend adapter.

The solve unrolls twelve damped Gauss–Newton steps in CPU float64. Analytic
local pose Jacobians and Torch linear solves preserve gradients from a smooth
synthetic pose loss to log-variance weights. One common log-variance gauge
preserves relative camera weights. There is no network head, optimizer training
step or learned covariance/calibration result. Duplication invariance checks
that copying the complete objective leaves its optimum unchanged; it does not
justify treating correlated duplicates as independent information.

This is our mechanism prototype, informed by [MAC-VO](https://arxiv.org/html/2409.09479v2)
and the pose-objective differentiation idea of [DNLS](https://arxiv.org/html/2305.09527v2).
It does not implement DNLS's symmetric PNEC residual, official optimizer,
network or training system. No author-system reproduction is claimed.

## Frozen numerical evidence

The first fixed-suite execution completed all 16 cases without source or
parameter changes. Seed17, 12 source landmarks per camera, 12 iterations,
relative diagonal damping1e-8, weighted-Jacobian rank-ratio cutoff1e-10.
Runtime is the existing isolated image
`sha256:fd7be8a3b1fe788972c54cabb5ee79433c30a53ab9951f49cf8c442f0753cf0a`,
Torch2.7.1+cu128, one CPU thread, deterministic algorithms, no network.

| Qualification | Result |
|---|---|
| Noiseless metric pose recovery | Maximum rotation-matrix/translation component error9.18e-14; tolerance1e-7. |
| Anisotropic noise in either camera | Both solves finite; final weighted residual cost below initial cost. No accuracy superiority gate was tested. |
| All-observation duplication, modality order, body-coordinate change | Maximum pose-component differences2.36e-16,3.19e-16,7.63e-16. |
| Analytic residual Jacobian vs independent projection differences | Pass componentwise rtol1e-6/atol1e-6; maximum absolute error3.03e-6, with relative tolerance also applied. |
| Pose-loss autodiff vs same-solver central differences | All12 probes pass rtol0.005/atol2e-8, step1e-4. Both modalities receive finite nonzero gradients in both noisy fixtures. |
| Common log-variance shift | Zero pose difference for a +7 shift. |
| Exact degeneracy, nonfinite observations/weights, behind-camera point | Explicit rejection; no silent repair or dropped observation. |
| Unequal camera intervals | Explicit rejection; no relabeling or reference-assisted bridge. |
| Independent SciPy fitted pose | Maximum pose-component difference1.19e-9; tolerance2e-6. |

The main suite included exact degeneracy. To fulfill the design's separate
near-degeneracy requirement, a supplement was frozen before execution using
seed23 and landmark spread1e-9m. Its singular-value ratio2.9981e-11 is below the
unchanged1e-10 cutoff; the prototype correctly rejects it. The passed main
suite was not changed or rerun to obtain this supplementary result.

## Independent audit disagreement

`tools/audit_shared_motion_feasibility.py` imports neither Torch nor the producer.
It reads the saved RGB-noise packet and uses independent NumPy projection,
SciPy trust-region optimization and outer central differences. It checks the
pose loss and six RGB weight derivatives. This audit does not cover the thermal
packet independently.

The independently fitted pose loss agrees to about1.95e-14. Five gradient probes
pass; row5/disparity fails:

| Quantity | Value |
|---|---:|
| Prototype autodiff derivative | 1.1032901007302423e-6 |
| Same-solver finite difference | 1.1032899875813850e-6 |
| Independent SciPy finite difference | 1.0477589114247024e-6 |
| Independent absolute discrepancy | 5.553118930553992e-8 |

The independent frozen criterion is rtol0.005 plus atol2e-8; this probe does not
pass. The agreement of the two forward pose fits and the same-solver derivative
checks does not establish the cause. Independent nonlinear-solve accuracy and
prototype differentiation remain possibilities, not demonstrated explanations.
No threshold, finite-difference step, solver setting or scientific input was
changed after seeing the result. Every output, including the failed audit, is
retained. There were no execution retries.

## Scope and next decision

The bounded feasibility experiment is complete, with qualification incomplete.
It establishes a working differentiable synthetic shared-motion path but does
not fully validate its gradients independently. No real images, checkpoint,
reserve, label-based VO score, learned model or production backend changed.
No superiority over RGB or cross-modal uncertainty calibration follows.

Do not proceed to real-data training under this numerical qualification. A
subsequent numerical correction must isolate this saved-packet disagreement
with an independently checked stationarity/derivative calculation and an
explicit new contract, rather than loosen this gate or launch a parameter
sweep. This finding neither rejects the research hypothesis nor establishes
that DNLS or joint estimation fails.

The real-data temporal prerequisite also remains unresolved. The synthetic
solver requires exactly shared intervals; it supplies no asynchronous trajectory
model and no evidence that actual RGB/thermal endpoints satisfy that condition.
Training or physical fusion claims require that separate contract.

Protocols/outcome: `experiments/shared-motion-feasibility-{freeze,supplement,outcome}-2026-11-12.json`.
Raw fixed inputs, full per-case evidence, rejected case, failed independent audit
and execution receipts: `results/shared-motion-feasibility/run-001`.
