# Saved shared-motion gradient disagreement resolved

12 November 2026. Branch `ore-ms-orb-development`. Pre-execution freeze
`426be5d2b`. One immutable saved RGB-noise packet; no prototype, input,
finite-difference step or derivative acceptance tolerance changed.

**Result:** the previous independent-audit discrepancy is explained by
insufficient stationarity of its float64 plus-perturbation fit. An independently
implemented80-digit calculation agrees with all six stored prototype gradient
probes and with a high-precision central difference at the previously failing
probe. The original failed audit remains unchanged and historically failed.
No training, real-data timing qualification or VO accuracy result follows.

## What was tested

The [previous bounded qualification](shared-motion-feasibility-findings-2026-11-12.md)
passed16 synthetic cases and near-degenerate rejection, but its independently
reoptimized finite differences failed one of six derivatives. This follow-up
pins that packet, original outputs and both old source files. It does not
regenerate fixtures or reselect a gradient probe.

`tools/audit_shared_motion_stationarity.py` imports neither Torch nor the
prototype. It independently implements optical projection and Rodrigues
rotation using mpmath1.3.0 at80 decimal digits. It first reproduces the original
SciPy fits, then solves the full objective's six stationarity equations at the
base and the failed plus/minus perturbations. Fixed Newton budget8, root
tolerance1e-60, explicit final gradient bound1e-45. No parameter sweep or retry.

For objective `S(x,v) = 0.5 sum_j exp(-v_j + mean(v)) r_j(x)^2`, let
`g = dS/dx`, `H = dg/dx`, and `L` be the unchanged synthetic pose loss.
At a stationary point, `dx/dv = -H^-1 dg/dv`. The independent derivative uses
this relation and the **full Hessian**, including residual curvature, rather
than substituting the Gauss–Newton approximation. The common weight gauge is
included in `dg/dv`. Derivatives and full Hessians are computed independently
with mpmath differentiation. Positive definiteness is checked at all three
refined roots. The finite-difference perturbation remains1e-4.

## Evidence and cause

| State | Exact objective-gradient infinity norm at old SciPy fit | At high-precision stationary point |
|---|---:|---:|
| Base | 1.19835e-8 | 5.13e-83 |
| Failed probe, plus | 1.36135e-5 | 1.40e-71 |
| Failed probe, minus | 1.34577e-8 | 3.74e-83 |

All three SciPy runs returned status2, `ftol` termination. The
[official SciPy documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html)
distinguishes cost-change termination from gradient termination. Accepting
`fit.success` alone in the old auditor did not establish stationarity accurate
enough for outer finite differences.

The old plus fit differs from the high-precision root by only6.28e-10 in the
largest pose component. Nevertheless, its pose-loss error is approximately
-1.11190e-11, compared with -1.27980e-14 for the minus fit. Subtracting the losses
and dividing by2e-4 amplifies that unequal fit error into the observed gradient
discrepancy. This is a limitation of that audit's numerical qualification,
not a demonstrated SciPy implementation bug.

| Previously failed derivative, row5/disparity | Value |
|---|---:|
| Unchanged prototype autodiff | 1.1032901007302423e-6 |
| Original float64 independent finite difference | 1.0477589114247024e-6 |
| New independent full-Hessian implicit derivative | 1.1032901007222353e-6 |
| High-precision independent central difference | 1.1032901024880500e-6 |

The loss-error accounting reconstructs the old-versus-high-precision derivative
difference of -5.5531191063e-8 within the frozen accounting bound1e-18.
The old SciPy finite-difference value is reproduced exactly in this runtime.
This supports a measured explanation, rather than attributing the mismatch
solely from better agreement with another solver.

All six stored RGB-noise gradient probes agree with independent implicit
derivatives: maximum absolute discrepancy2.077e-16. Both the implicit and
high-precision finite-difference comparisons pass the **unchanged** derivative
criterion rtol0.005 plus atol2e-8. All three roots satisfy the explicit
stationarity bound and have positive definite Hessians; minimum eigenvalues
are about3902.4 in the declared coordinates. All six aggregate report checks
pass on the first execution. No learned model or estimator setting changed.

## Decision and remaining scope

The saved-packet gradient discrepancy is resolved. The old failed audit is
retained as a reproducible example of why a successful forward optimizer return
is insufficient for a derivative audit. Use the independent stationarity-based
check for this packet; do not silently relabel the old report as passing.

The result supports numerical differentiation of this local synthetic shared
motion mechanism. Independent probe coverage remains the saved RGB-noise
packet's six probes, although the earlier producer suite separately exercised
both noisy modalities. This is not exhaustive gradient validation across all
weights, geometries, initializations or real data. It is not covariance
calibration, a DNLS reproduction, learned fusion benefit or broader sensor
transfer. [DNLS](https://arxiv.org/html/2305.09527v2) remains inspiration for
task-aligned differentiation, not a system reproduced by this prototype.

The next unresolved prerequisite is the real-data interval/rig contract from
the [research decision](multisensor-research-reset-2026-11-12.md). The synthetic
solver rejects unequal camera intervals and supplies no asynchronous trajectory
model. Resolve whether an existing capture supports the proposed shared-motion
measurement objective before fitting a head; numerical success alone does not
justify training on nominally paired camera frames.

Protocol: `experiments/shared-motion-stationarity-freeze-2026-11-12.json`.
Results: `results/shared-motion-stationarity/run-001/{report,stationarity-progress}.json`.
The original fixed packet, failed audit and prototype remain unchanged.
