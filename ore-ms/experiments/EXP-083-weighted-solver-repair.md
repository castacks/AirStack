# EXP-083: isolated consistent weighted-Huber solver arm

2026-10-29 America/New_York. Branch `ore_ms`, starting commit `fd397fe25`.

An isolated research solver consistently whitens residuals/Jacobians before Huber
correction, evaluates the same objective for acceptance, restores rejected
parameters exactly and returns explicit termination/failure status. On six
preselected fixed-factor problems, the original solver reproduces every recorded
native pose vector exactly; the new arm lowers its declared weighted objective
in all six. Every saved repair run reaches the10-iteration cap, so convergence
is not established. No production/default or original estimator source changes.

## Declaration and method boundary

Freeze cases before solving by the first eligible ordinal per camera/window in
EXP080's label-free original-deadline predictions: early RGB/thermal0/0,
middle RGB/thermal96/0, day RGB/thermal0/0. These are previously examined windows,
not unseen recordings. The sparse RGB96 survivor enters through the same fixed
chronological rule, not largest-error selection. No labels, images, checkpoints,
neural inference, new reference, gate fit or reserved MS² data are used.

Each graph uses the exact saved retained factors, K, baseline, target covariance
and fixed landmark coordinates. The recorded causal parent pose supplies the
initial pose, matching the inspected StaticMotionModel source. The recorded
optimized pose is used only as an original-replay control. Original native graph
class definitions and NED projection are compiled unchanged from their source
AST into an isolated namespace to avoid neural package imports. Native PyPose
SE3 retraction and the exact original graph forward/Jacobian remain in use.
This is not a full VO replay: new upstream estimates would alter future
landmarks, tracking and initialization. No trajectory is composed or scored.

The new explicit objective is `F=sum_i rho(||L_i^-1 r_i||²)`, where
`C_i=L_i L_i.T`, fixed finite symmetric positive-definite target covariance,
and Huber delta0.1 in **dimensionless whitened-block norm**. It has a different
meaning from legacy0.1 in raw pixel residual norm. The numeric delta is declared
for a contract experiment, not tuned or claimed physically calibrated.
Landmark/cross-observation uncertainty and learned variance calibration remain
outside this conditional fixed-landmark objective.

Whiten both residual and analytic J before robust correction. Use native PINV,
initial damping1e-3, diagonal clamp[1e-6,1e32]. Predicted decrease is
`-(2*g.T*D+D.T*H*D)` from the same corrected whitened system. Accept only finite
candidates with positive predicted and actual reduction and ratio>=1e-4.
Rejected candidates restore exact parameter snapshots. Multiply damping by10 on
rejection; on acceptance divide2 if ratio>.75, multiply10 if ratio<.25, otherwise
hold. Clamp damping to[1e-12,1e12]. Keep the best valid state and return its
recomputed objective. Explicit statuses distinguish invalid inputs,
linearization/solver error, rejected-attempt exhaustion, small-step no-progress,
gradient tolerance and iteration limit. They do not imply physical accuracy.

Budgets are10outer iterations and17candidate attempts/outer; gradient infinity
tolerance1e-10 and step norm1e-10. A tiny step is `NO_PROGRESS_SMALL_STEP`, not
silently successful convergence. The original arm uses unmodified LM_analytic,
FastTriggs, PINV, TrustRegion(radius1000) and original StopOnPlateau settings
(10steps,patience2,decreasing1e-5). It can stop sooner. Both objectives are
reported at both outputs; unlike scalar objectives are never treated as equal.

## Frozen saved-factor controls and results

Original replay tolerance was declared1e-3m/0.01deg. All six controls pass;
observed translation/rotation differences are zero and the seven serialized pose
entries match their recorded native vectors exactly in these six cases. This is
a bounded CPU graph/solver control, not historical GPU-image or whole-pipeline
bitwise reproduction.

| Saved case / pair | Factors | Initial new objective | Original final new objective | Repair final new objective | Original / repair outer steps |
|---|---:|---:|---:|---:|---:|
|early/rgb / 0|200|73.841617|70.101903|69.786151|2 / 10|
|early/thermal / 0|200|14.433039|7.750898|7.478441|3 / 10|
|middle/rgb / 96|18|14.790602|17.861083|12.261847|6 / 10|
|middle/thermal / 0|192|378.979628|15.125823|14.338380|10 / 10|
|day-middle/rgb / 0|186|1287.929846|106.787350|106.597823|10 / 10|
|day-middle/thermal / 0|186|869.167071|18.676971|18.639474|10 / 10|

| Saved case | Original final raw Huber | Repair final raw Huber | Original rejection counts per outer step |
|---|---:|---:|---|
|early/rgb|430.958892|430.552538|0,16|
|early/thermal|11.239880|11.259855|0,0,16|
|middle/rgb|269.463849|284.677844|0,0,0,0,0,1|
|middle/thermal|20.340433|18.685496|0,0,0,0,0,0,0,0,0,0|
|day-middle/rgb|257.481046|256.608576|0,0,0,0,0,0,0,0,0,0|
|day-middle/thermal|21.902790|21.867165|0,0,0,0,0,0,0,0,0,0|

All60 accepted saved-arm steps reduce the new objective. All six saved repairs
terminate `MAX_ITERATIONS` at10outer steps, with no calibrated convergence or
accuracy claim. Lower new-objective values are expected to be favored by a
solver targeting that objective; six cases do not establish general superiority.
Early thermal and middle RGB have higher raw-Huber loss under the repair,
showing the declared objective tradeoff. Different original stopping times also
mean this is not an equal-work speed comparison.

The original early RGB replay returns raw loss430.958892 after a prior iterate
at430.945783; its last step has16rejections. Early thermal similarly ends at
11.239880 versus the previous11.239854 with16rejections. The original scheduler
stops after these capped steps. Thus the EXP082 rejection-limit behavior appears
in controlled replay of these actual saved factors, with exact final-pose
reproduction. Historical journals still lack iteration telemetry, and the
physical error caused by a repaired trajectory remains unmeasured. Middle RGB
stops with rejection_count1 after sixouter steps; other original controls reach
tensteps with zero recorded rejections. No historical result is overwritten.

## Toy and failure controls

All14 frozen EXP082 affine controls are rerun against their exact original
records. Toy repair budgets are oneouter iteration to match the original
one-step contract probes; saved-factor arms each have a10step cap. Repair always
uses its declared weighted Huber objective, including legacy no-kernel equation
fixtures; those fixtures test contracts, not equal-objective optimization gains.
Constructor-only legacy weights are converted explicitly to covariance for the
new arm, whose interface has no constructor/step-weight ambiguity.

In the original native conflict toy, raw/weighted gradients disagree. The
repair accepts a consistent negative direction, changingx0.05 tox0.02002997
and reducing the new objective0.04 to0.02960480 within oneiteration. The
scripted raw-improving/weighted-worsening direction is rejected without changing
the best state and returns `REJECTION_LIMIT`; the opposite consistent direction
is accepted, givingx0.025 and objective0.030625. Zero scripted steps return
explicit no-progress status. Stationary controls do not invoke a solver just to
manufacture failure. Separate nonstationary error fixtures exercise failures.

18 CPU tests pass: correlated-covariance finite-difference gradient, identity
covariance/native correction agreement, monotone best-state return, nonpositive
model-reduction guard, rejection cap, solver exception and nonfinite step,
nonfinite candidate restoration, stationary/small-step distinction, covariance
shape/symmetry/SPD rejection, invalid Jacobian, iteration cap, preservation of an
improved state after a later solver error and bitwise SE3 rollback after an
actually evaluated worse nonlinear candidate.

## Independent verification and numerical details

A NumPy validator imports neither repair nor runner. It reconstructs native NED
projection, covariance whitening, Huber system, SE3 left-exponential retraction,
all candidate states/objectives/reductions/damping decisions, best-state returns
and original replay costs/pose controls. It checks88candidate attempts and
63accepted steps across14toys and6saved problems. Original affine rows exactly
match the independently validated EXP082 records.

A uniform1e-6 finite-difference pose step initially failed the gradient check
near late precision-weighted Huber transitions. A multiscale numerical audit on
all60 saved linearizations gave maximum normalized gradient differences about
0.36536 at1e-6,0.01626 at1e-7,1.13e-5 at1e-8 and1.50e-4 at1e-9. The validator uses
one fixed1e-8 step for every saved state (1e-6 for affine toys), under the same
1e-3 numerical criterion. No case, solver/objective setting or data threshold
changed. This validation-scale adjustment and all per-state errors are preserved
in `finite-difference-scales.json`; neither a numerical criterion nor a
small gradient is a statistical confidence statement.

A guarded full rerun blocks proxy/reference/model data reads and yields an
identical comparison apart from its timestamp. Nine refreshed-digest semantic
mutations are rejected: acceptance flag, candidate loss, best loss, returned
state, predicted decrease, damping, original pose-tolerance result, case identity
and fabricated physical-convergence status. Source/raw/config hashes are checked
before and after execution. Original core/caller/graph/projection sources and
all historical artifacts remain unchanged.

Reuse EXP082 CPU image
`sha256:d7ef749b30c6c60649f961794c395a17a7916d16c873dee27857df210e8ea157`,
Torch2.7.1+cpu,PyPose0.9.5,NumPy1.26.4,SciPy1.17.1,Python3.12,
`OMP_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1`, `TORCHDYNAMO_DISABLE=1`,
`NVIDIA_VISIBLE_DEVICES=void`, Docker `runc`. No GPU is used.
From MAC-VO root with ore-ms mounted at `/workspace`, choose a fresh output:

```bash
python3 -m pytest -o addopts= Scripts/UnitTest/test_weighted_huber_research.py -q
python3 -m Scripts.Setup.solver_repair_experiment prepare --root /workspace --output <fresh-run>
python3 -m Scripts.Setup.solver_repair_experiment run --root /workspace --output <fresh-run>
python3 -m Scripts.Setup.validate_solver_repair --root /workspace --output <fresh-run>
```

Authoritative output: `results/solver-repair/run-001/` with protocol, original/
repair comparison and complete traces, independent validation, scale audit,
guard,mutations and runtime provenance. Recovery target:
`backups/2026-10-29-exp-083`, with EXP079 raw-factor audit/EXP080 label-free
predictions, their EXP073/074/077 raw experts and EXP082 probe/runtime dependencies.
Credentials, images and model weights are excluded.

## Decision

The research arm repairs the declared objective/acceptance and best-state
contracts on these controls. Keep it opt-in; no production solver import,
configuration or default is replaced. Objective reduction does not establish
better motion accuracy, convergence or a transferable camera gate. A next
separately declared experiment would run a bounded end-to-end original/repair
VO control on an existing inspected interval, retain tracking/abstention and
runtime differences, and evaluate the inherited qualified reference separately.
No reserved data or label-based tuning is needed to define that control.
