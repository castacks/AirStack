# EXP-082: native robust / weighted solver contract audit

2026-10-29 America/New_York. Branch `ore_ms`, starting commit `0817cae27`.

Bounded native CPU probes confirm that the analytic optimizer uses covariance
weights in its linear system but accepts steps using unweighted raw-residual
Huber loss. A native PINV/TrustRegion toy reaches the16-rejection limit and
returns a candidate with worse acceptance loss. Constructor-only weights are
also ignored in this branch. These are verified solver-contract behaviors, not
evidence that they caused the recorded VO errors. Original estimator files and
defaults remain unchanged.

## Scope and source

Freeze14 tiny affine residual fixtures before executing them. Import the exact
local `Module/Optimization/PyposeOptimizers.py` directly, without loading a neural
frontend, data, checkpoint or ROS stack. Build a focused CPU image using
Torch2.7.1+cpu and PyPose0.9.5, with the existing NumPy1.26.4/SciPy1.17.1 base.
The base image is `sha256:ae59352813af040d1a60000f2b13eed990b8364f8a416f5b87e385d4496aade7`;
the resulting image is
`sha256:d7ef749b30c6c60649f961794c395a17a7916d16c873dee27857df210e8ea157`.
Python3.12, `OMP_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1`,
`TORCHDYNAMO_DISABLE=1`, `NVIDIA_VISIBLE_DEVICES=void`, Docker `runc`.
Dependencies are isolated under `/opt/solver-contract`; the base OS Python
installation is not modified. This is a fresh CPU environment, not a historical
GPU-image reproduction or saved-trajectory parity claim.

Snapshots and SHA256 preserve seven exact methods: local `LM_analytic.step`,
PyPose `RobustModel.loss`, `FastTriggs.forward`, `TrustRegion.update`,
`Huber.forward`, `StopOnPlateau.step`, `_Optimizer.update_parameter`.
The independent validator extracts the installed source through AST locations
and compares each snapshot to the installed files. Original local core/caller
hashes match the starting Git commit after all probes. The audit does not
attribute these local behaviors to an uninspected upstream MAC-VO revision.

## Active linear-system / loss contract

For a residual factor `r_i` and Jacobian `J_i`, FastTriggs computes
`g_i = rho'(r_i.T*r_i)`, then scales **raw** residual and Jacobian by `sqrt(g_i)`.
The local analytic optimizer subsequently applies the explicit dense covariance
precision matrix `W`, forming `A = J_corrected.T W J_corrected` and
`b = -J_corrected.T W r_corrected`. It clamps the diagonal and applies LM damping.
The robust derivative is thus evaluated before covariance whitening.

Acceptance and the reported scalar loss instead call
`RobustModel.loss(input,target) = sum_i rho(r_i.T*r_i)` with no covariance weight.
The trust-region strategy is passed corrected `J,R,D` without `W`; its predicted
reduction is `-(J*D).T*(2*R+J*D)`. Scheduler decisions use this unweighted returned
loss and the rejection counter. The production caller supplies inverse factor
covariances through **explicit** `step(...,weight=weight)` at every iteration.

Three quantities must remain distinct:

- the local weighted/corrected linear system;
- the raw unweighted robust acceptance loss;
- a possible whiten-then-Huber loss `sum rho(r_i.T W_i r_i)`.

The third is an audited mathematical comparator, not a replacement objective
adopted here. Different robustification/weighting orders define different
methods and delta units. There is no outcome-based choice of an objective.

## Verified probes

### 1. Explicit weights work; constructor-only weights do not

Two scalar factors `r=[x+1,x-1]`, initialx0, no robust kernel, fixed damping0.001.
The control solver returns zero so only the exact supplied system is inspected.

| Configuration | Damped A | b | Returned raw loss |
|---|---:|---:|---:|
| No weights |2.002|0|2|
| Explicit `diag(1,9)` |10.010|8|2|
| Constructor-only `diag(1,9)`, no step weight |2.002|0|2|
| Constructor `diag(9,1)`, explicit `diag(1,9)` |10.010|8|2|
| Explicit `diag(9,1)` |10.010|-8|2|

The branch checks the explicit `weight is None`, so constructor-only `self.weight`
is calculated into a variable but not applied. This is a latent API issue; the
active VO caller passes explicit weights, so it does not explain that caller's
historical outputs. Huber inlier/outlier and correlated3D-factor controls also
match independent corrected-residual and matrix equations.

### 2. Candidate acceptance can disagree with a covariance-weighted robust loss

Two factors `r=[x,x-0.2]`, initialx0.05, Huber delta0.1,
explicit `W=diag(9,1)`. Scripted candidate directions isolate the native
accept/reject predicate; they do not claim native VO generated those directions.

| Candidate x | Raw Huber loss | Whiten-then-Huber loss | Native decision |
|---|---:|---:|---|
| Initial0.050 |0.022500|0.040000|Starting state|
|0.075, scripted+0.025 |0.020625|0.050000|Accept: raw loss improves|
|0.025, scripted-0.025 |0.025625|0.030625|Reject: raw loss worsens|

The rejection control then supplies zero as its second candidate, yielding the
originalx0.05. Its rejection counter remains1, which causes the native scheduler
to stop after that step despite the second candidate being accepted.

### 3. Native PINV / TrustRegion reaches a worse-loss return at the rejection cap

On the same factors/weights, remove scripted directions and delegate to actual
PyPose PINV and TrustRegion(radius1000). No solver or optimizer patch is used.
The local native linear gradient is+0.7, while the finite-difference gradient of
the raw acceptance loss is-0.1 and that of whiten-then-Huber is+0.4. The weighted
step therefore points uphill for the raw acceptance objective.

All17 native candidates increase raw acceptance loss. The predicted corrected,
unweighted reductions are also negative. Their ratio is positive and greater
than the strategy's0.5 threshold, so each update halves damping while the local
outer branch rejects the candidate. After16 rejections, the17th worse candidate
is kept because the branch only rolls back while `reject_count < reject`.
The native scheduler then stops on the nonzero rejection count.

| Quantity | Result |
|---|---:|
| Initial x |0.0500000000|
| Returned x |0.0138654202|
| Initial raw acceptance loss |0.0225000000|
| Returned raw acceptance loss |0.0274191658|
| Initial whiten-then-Huber loss |0.0400000000|
| Returned whiten-then-Huber loss |0.0289571649|
| Native solve attempts / rejected count |17 / 16|

The identity-weight native control accepts a single step tox0.0799700300 and
reduces raw loss to0.0204011997. Its native step gradient matches the independent
raw-loss finite-difference gradient within1.15e-12. Recording wrappers preserve
both native control results bitwise versus the unwrapped native solver,
corrector and strategy. An additional scripted rejection-cap control confirms
that the cap can retainx1 withloss2 from an initialx0 withloss0 after three
attempts at reject2.

This demonstrates a concrete consistency/rejection problem in the toy contract.
It does not prove that any saved VO solve exhausted its retry limit. Historical
journals lack the required iteration/rejection telemetry, so those counts remain
unknown. No original trajectory is recomputed or relabelled as failed.

### 4. Correlated-factor and failure-status boundaries

For the fixed two-factor3D residual fixture with correlated UV covariance, the
native linear gradient is `[0.0631888,0.1359241]`; independent raw-Huber gradient
is `[-0.1161440,0.1395180]` and whiten-then-Huber gradient is
`[0.0563322,0.1636908]`. The differences confirm that neither candidate scalar
loss generally has the local weighted/raw-corrected gradient. This is not a
claim that no other mathematical scalar objective could be defined.

A deliberately injected linear-solver exception is printed by the original
catch block; it is not re-raised. The probe returns the unchanged parameter and
finite loss2. Rejection count stays0, and after one scheduler step it would
continue (patience_count1). The printed failure is preserved, not hidden or
called convergence. A finite returned loss is therefore insufficient as a
success/convergence status. This probe injects a failure; it does not assert
that native PINV failed on a historical dataset.

## Validation and artifacts

19 CPU tests pass, including analytic matrices, robust correction ordering,
accept/reject direction controls, native PINV conflict/identity controls,
rejection cap, injected failure, original-core preservation, independent scalar
costs and trust-region sign behavior, plus two bitwise wrapper/native controls.

A separate NumPy validator imports neither PyPose nor the probe producer. It
simulates all14 fixtures,33 solve attempts and32 strategy updates from direct
Huber/corrector/matrix and state-machine equations, checks every intermediate
system/direction/loss/parameter/damping state and scheduler result, and computes
independent central finite-difference gradients. Seven installed source-method
snapshots and all source/version hashes pass. Nine refreshed-digest mutations
are rejected: matrix,gradient,correction,loss,parameter,rejection count,
trust-region state, installed-method snapshot and dependency version.

Reproduce from MAC-VO root, with ore-ms mounted at `/workspace`:

```bash
docker build -f Docker/Dockerfile.solver-contract -t macvo:solver-contract .
docker run --rm --runtime=runc -v "$PWD/..:/workspace" macvo:solver-contract   -m Scripts.Setup.solver_contract_probe --output /workspace/results/solver-contract/<fresh-run>
docker run --rm --runtime=runc -v "$PWD/..:/workspace" macvo:solver-contract   -m Scripts.Setup.validate_solver_contract --output /workspace/results/solver-contract/<fresh-run>
docker run --rm --runtime=runc -v "$PWD/..:/workspace" macvo:solver-contract   -m pytest -o addopts= Scripts/UnitTest/test_solver_contract_probe.py -q
```

Authoritative artifacts: `results/solver-contract/run-001/` with frozen protocol,
probes, seven native method snapshots, independent validation/equations/gradients,
mutation evidence and execution provenance. Recovery target:
`backups/2026-10-29-exp-082`. The source commit and focused Docker recipe suffice
for these probes; no dataset/model restoration is needed. Dependency access and
fresh container-image availability are required to rebuild the CPU environment.

## Decision

Keep estimator defaults and all previous results unchanged. Correctness of the
weighted linear equations is not correctness of a consistent weighted robust
optimization/acceptance procedure. Before interpreting a quality gate through
those objective values, declare an isolated solver-contract repair experiment:
choose one explicit weighting/robustification objective and delta units, preserve
a best valid state, make rejection exhaustion/failure status explicit, and test
both synthetic and frozen saved-factor controls against the original solver.
A controlled alternative would be a new method arm, not a silent baseline fix;
pose accuracy and trajectory preservation would require separate verification.
