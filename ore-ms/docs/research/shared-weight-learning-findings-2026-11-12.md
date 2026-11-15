# Task-trained shared measurement weights: no advancement

12 November 2026, America/New_York. Branch `ore-ms-orb-development`; starting
Git baseline `4f28219d9` plus SHA-pinned working-tree source. Recovered prior
baseline artifact identifies `d3f2eafee`; that commit is unavailable from origin.
Qualification, learning and scoring protocols were separately frozen before
real-packet qualification, first parameter update and scoring, respectively.

**Decision: CLOSE_THIS_LEARNED_CONFIGURATION_NO_RETUNING.** All three recording
advancement gates fail. Six heads complete their fixed40 updates and retain
all297 nominal intervals for each method. Rain's learned joint head improves
slightly over the learned RGB head but remains worse than original RGB-cov;
day and night also miss the required comparison. This exact small-head recipe
is closed, not the general hypothesis of task-aligned multisensor learning.

## What changed

The [preceding decision](shared-weight-next-decision-2026-11-12.md) selected a
measurement-weight learning experiment after the fixed-covariance joint solve
failed. An isolated float64 CPU Torch implementation differentiates through all
12 original Gauss–Newton iterations, including whitening, geometric Jacobians,
diagonal damping, linear solves and SE3 updates. It does not differentiate an
assumed stationary optimum. Full original target covariance, frozen source
landmarks/means, identity initialization, source support and nominal same-ID
camera groups stay fixed. The earlier strict-time interface is unchanged.

A shared six-parameter head maps log own-camera source depth, log target UV
covariance trace, log target disparity variance and normalized target u/v to one
bounded scalar precision multiplier per observation. The affine output passes
through2*tanh(x/2); a single across-factor log-precision gauge preserves relative
camera mass. No per-camera mass normalization or modality-specific architecture
is added. Its intercept has zero observable gradient at the zero initialization
under the gauge; the declared sixth parameter remains, without posthoc redesign.
Inputs and all original covariance tensors are frozen. Learned multipliers are
task weights, not physically calibrated covariances.

Each excluded recording has a separately fitted RGB-only head and joint head
with identical architecture and learning budget. Feature mean/std use that
mode's training-fold factors only. Three folds train on two complete exposed
prefixes and predict the third. Exactly40 full-batch Adam updates,lr1e-3,
zero initialization, no checkpoint selection/early stopping/weight decay.
The loss equally weights the two training recordings and averages squared
translation/(0.1m)^2 plus squared rotation-matrix Frobenius/(sqrt(2)*0.01rad)^2.
All actual camera timestamps and one rain joint RGB fallback are retained.

## Qualification before training

Six mechanics tests pass. All594 zero-head RGB/joint outputs reproduce saved
original-covariance results: maximum transform-entry discrepancy9.60e-14,
well below the frozen1e-8 bound. Twelve per-factor and12 dense-head directional
checks pass independent NumPy central differences at h1e-4,atol1e-7,rtol1e-3.
The first jointly available interval of each recording is fixed; both camera
branches are tested at zero and deterministic nonzero weights. These use a
smooth sensor-only pose probe, never supplied reference labels.

Stationarity is recorded but not used to substitute implicit derivatives for
unrolled derivatives. Training only started after every qualification passed.
There were no failed scientific executions, retries or tolerance amendments.

## Execution and target isolation

All240 optimizer updates complete (47,520 training interval-solves); all594
excluded-record predictions complete. Each fold process opens supplied pose
values only for its two training recordings. Its target reference values are
not parsed or used during fit or prediction; all source files, including pose
files, are content-hashed as opaque bytes for integrity. All six final checkpoints and prediction
files are globally hash-locked before scoring. Training on a recording in
another fold does not make these already exposed data blind or fresh tests.

Pre-update training objectives decrease from update1 to update40 for all six
heads; these traces are not target performance or a loss measured after the
40th update. No scientific gate is inferred from training descent.

## Per-record results on primary matched support

Each row below uses all99 intervals of its recording, shared by both learned
methods and RGB-cov. No missing slot is removed to improve a score. All-method
common support, including every old thermal/native control, is99/99/98 for
day/night/rain; own/common and pairwise-matched results, p95, maximum, complete
rows and all original native/fixed/covariance controls are retained separately.

| Recording | Method | Translation mean / RMSE, m | Rotation mean / RMSE, deg |
|---|---|---:|---:|
| day | rgb_cov | 0.045538 / 0.051786 | 0.059202 / 0.063692 |
| day | learned_rgb | 0.046274 / 0.052555 | 0.059287 / 0.063668 |
| day | joint_cov | 0.050236 / 0.055649 | 0.062062 / 0.065458 |
| day | learned_joint | 0.049189 / 0.054491 | 0.061882 / 0.065346 |
| night | rgb_cov | 0.020738 / 0.034281 | 0.036297 / 0.053587 |
| night | learned_rgb | 0.021128 / 0.035043 | 0.035947 / 0.052942 |
| night | joint_cov | 0.022050 / 0.035317 | 0.040978 / 0.065168 |
| night | learned_joint | 0.022065 / 0.035378 | 0.040333 / 0.063906 |
| rain | rgb_cov | 0.191728 / 0.371328 | 0.040476 / 0.046590 |
| rain | learned_rgb | 0.198589 / 0.378175 | 0.040684 / 0.046795 |
| rain | joint_cov | 0.195302 / 0.376953 | 0.040601 / 0.046769 |
| rain | learned_joint | 0.194488 / 0.373370 | 0.040174 / 0.046161 |

The primary requires at least5% lower translation mean AND RMSE than BOTH
learned RGB-only and original RGB-cov on every recording, with no rotation
mean/RMSE increase and no coverage loss. Day/night/rain each fail both required
comparison gates. A joint-vs-native improvement alone cannot demonstrate thermal
benefit when a changed RGB-only backend is stronger. No secondary result replaces
the primary or motivates a longer fit/feature/threshold search.

## Independent verification and disposition

The independent NumPy auditor imports neither solver, trainer nor scorer. It
reconstructs training-only normalization, model features and learned weights,
checks fold label-access receipts and final checkpoints, reprojects every saved
pose, reconstructs594 weighted costs, and recomputes references, errors,
statistics, masks and all gate decisions. All13,180 checks pass. Four semantic
corruption controls reject a target-label receipt leak, wrong normalization,
wrong pose/cost and wrong metric even after refreshing affected byte hashes.
These controls use disposable copies; authoritative results remain unchanged.
The audit does not independently rerun all optimizer updates or frontend inference.

A pinned Docker recipe builds and imports Python3.12.3,NumPy1.26.4,SciPy1.11.4,
Torch2.5.1+cpu successfully. Training used the equivalent isolated container
with the same base-image digest and exact package versions. No GPU/runtime
performance or native backend replacement is claimed.

Retain the original native models and close this learned configuration without
retuning. This result does not establish that weighting cannot help a different
estimator, representation or objective. Its scope remains fixed source landmarks,
conditional target covariance, supplied nominal same-ID pairs/references and
unknown checkpoint overlap, with no calibrated uncertainty, verified physical
synchronization, whole-trajectory or fresh-record generalization claim.

Protocols: `experiments/shared-weight-qualification-freeze-2026-11-12.json`,
`experiments/shared-weight-learning-freeze-2026-11-12.json`, and
`experiments/shared-weight-scoring-freeze-2026-11-12.json`.
Tools: `shared_weight_solver.py`, `qualify_shared_weight_solver.py`,
`train_shared_weights.py`, `score_shared_weights.py`, `audit_shared_weights.py`.
Immutable local artifacts: `results/shared-weight-qualification/run-001` and
`results/shared-weight-learning/run-001`; corruption controls are in the separate
`results/shared-weight-learning/audit-mutations-001` subtree.

This is a local DNLS-inspired weight-learning mechanism, not reproduction of the
published monocular rotation system. Prior [MAC-VO/DNLS reading and source
inspection](multisensor-research-reset-2026-11-12.md) remain the attribution basis;
no new source-paper claim or reading event is asserted here.
