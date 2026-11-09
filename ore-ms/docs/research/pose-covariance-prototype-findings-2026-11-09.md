# Pose-covariance prototype: real-data replay and numerical verification

2026-11-09, America/New_York. Kernel/smoke freezes `286593d2f`, stable-loss amendment `a0293fbb3`, exact-prefix/independent-check freeze `6308b61f3`. Implements the first stage of the [new experiment](next-experiment-pose-covariance-2026-11-09.md). No new neural inference, reference-label join, pose-accuracy comparison, training or fused trajectory run occurred.

## What is implemented

An opt-in NumPy/SciPy replay tool takes paired 3D observations and both endpoint covariances from the saved native tensor maps. It estimates the target-to-source transform with a translation-first left SE(3) perturbation. The exact declared Gaussian residual objective includes both Mahalanobis error and log determinant, and its gradient includes the covariance's rotation dependence. Fisher scoring with backtracking reports convergence, line-search failure, numerical rank failure or the fixed iteration cap explicitly.

At a stationary solution it exports the undamped expected-Fisher inverse and the mean-only information inverse separately. Independent factor/endpoint noise is an approximation; these matrices are model covariances, not empirically calibrated physical confidence. This extends the MAC-I²-inspired mean-information approximation with covariance derivatives in a consistent residual likelihood; it is not presented as an exact reproduction of MAC-I²'s weighted-PGO solve.

The module also provides local covariance transport and Gaussian covariance intersection in a caller-declared common linear tangent. These are tested primitives. Cross-camera nonlinear fusion, complete frame/time transport and reference scoring are not yet implemented.

Files: [replay kernel](../../tools/pose_covariance_replay.py), [numerical tests](../../tools/test_pose_covariance_replay.py), [independent checker](../../tools/validate_pose_covariance_smoke.py).

## Data feasibility and preservation

Verified the 35,870,648-byte original development packet against SHA256 `1aaa832858c62328b70f088c704cc7898f7da25f0ac7c3ac5385f4c2730ed2e7` and all 713 regular member hashes. Restored results to a new ignored snapshot tree. Both cameras have original source/target 3D covariance, pixels, depths, exact source clocks and first-write receipts. This avoids treating saved world landmarks as exact or using post-refinement poses. The 20-group smoke is `_2021-08-13-21-18-04`, distinct from the planned 600-group cohort.

Verified the full day/night evaluation archive SHA256 `953a8bf36912573b7ea1d35fa79a9d428be735e0b07a9d40f808ef8c42a3595a` and restored only eight manifest-matched tensor/event files. No reference labels were extracted. All raw input hashes match before/after replay.

The original source axes remain intact. The prefix option selects exactly groups 0–599, including unavailable source intervals, rather than choosing intervals by support. Native first-committed relative poses initialize the new solve; their original observation stage remains explicit.

## Numerical checks

All **15 tests pass** in the pinned CPU container. Checks cover objective gradients, covariance derivatives, SE(3) retraction, frame-change invariance, local transport, invalid covariance and degeneracy rejection, explicit nonconvergence, exact-prefix preservation and covariance intersection with correlated identical inputs.

A seeded **160-trial** small-noise synthetic check converges on every trial. Mean squared Mahalanobis error is **6.563** for a six-dimensional error; empirical/predicted coordinate variance ratios range **0.970–1.202**. This checks the assumed local Gaussian model, not real sensor calibration or generalization.

The first real smoke converged on 16/19 RGB and 19/19 thermal intervals. Three RGB line searches stalled at score norms approximately 1.18e-6, 2.47e-6 and 8.59e-6. A separate synthetic regression demonstrated loss-decrease cancellation below the precision of the large total objective. The stable-loss amendment evaluates the same objective difference through inverse/determinant increment identities. Inputs, tolerance and iteration budget did not change. The initial outputs are retained. The amended smoke converges on **19/19 per camera**, including 18 processing-stage and one explicitly terminal write each. No terminal output is relabelled as on-time availability.

## Planned day/night prefixes: label-free preflight

Each stream contributes 599 adjacent intervals from its first 600 original source groups.

| Recording / camera | Converged | Iteration cap | No native first write | Total |
|---|---:|---:|---:|---:|
| `_2021-08-13-16-31-10` / RGB | 599 | 0 | 0 | 599 |
| `_2021-08-13-16-31-10` / thermal | 592 | 3 | 4 | 599 |
| `_2021-08-13-22-16-02` / RGB | 598 | 0 | 1 | 599 |
| `_2021-08-13-22-16-02` / thermal | 503 | 55 | 41 | 599 |
| **Total** | **2,292** | **58** | **46** | **2,396** |

No covariance is emitted for a capped/nonstationary solve. The 58 capped cases are retained without increasing the 100-iteration budget. Night thermal capped score norms span 1.61e-6 to 11.09, median 2.38e-4; these are not all near-roundoff cases and cannot be dismissed as the earlier loss-subtraction problem.

The independent checker imports no producer code. It reconstructs inputs via inverse-intrinsic optical backprojection, rebuilds both information matrices and objective, verifies saved covariance inverses, and checks stationarity using matrix-exponential finite differences in covariance-scaled directions. All **13,752 directions across 2,292 converged prefix solves pass**; maximum scaled derivative is 1.40e-6. The 38-interval amended smoke separately passes 228 directions. Input bindings, counts and failure statuses remain explicit.

These are solver/covariance-availability results. They do not show lower pose error, better uncertainty calibration, superiority of thermal/RGB, or a fusion benefit. All recordings are already development-exposed.

## What this changes for the experiment

Both-endpoint data recovery and the initial numerical prototype are complete. Existing day/night cached outputs suffice for this part without restoring model weights or a GPU inference runtime. The planned full three-record comparison is not complete.

Next work:

1. Add the experiment's common-frame/interval covariance transport and matched equal/scalar/full covariance combination. Use an explicit common tangent and preserve all means/control bindings.
2. Treat the 58 missing replay covariances as an availability cost. Compare against native and equal-weight controls on complete source axes as well as shared support; do not make them disappear through matched-only reporting. A label-free solver improvement would require its own numerical justification and freeze, preserving this first result.
3. Recover or capture the exact continuous rain groups0–599 using the existing frozen frontend. The checked EXP114 manifest contains separate 100-group windows (starting1,3957,7913), not a substitute600-group continuous stream. No rain payload was downloaded during this step.
4. Freeze complete prediction/control outputs before reference scoring, then run the declared comparison and update the weekly brief with error/calibration/coverage results.

## Reproduction and artifacts

Runtime: `sha256:40838ec9ce83b7f6d05b90b000aa2f0a76f2ddbbf21b180dba717707e220a2fe`, Python3.12.3, NumPy1.26.4, SciPy1.17.1, CPU, BLAS/OMP threads1. This is an isolated offline analysis runtime; the historical neural runtime was not silently replaced for inference. Source/data hashes and solver rules are in the three tracked freeze JSONs under `experiments/`.

With `ore-ms` mounted as `/workspace`, working directory `/workspace/tools`:

```bash
python3 -m unittest -v test_pose_covariance_replay
python3 pose_covariance_replay.py --capture <original-capture-directory> --groups 600 --output <new-output.json>
python3 validate_pose_covariance_smoke.py --reports <output.json> --output <new-check.json>
```

CLI refuses existing output files. Raw outputs, initial failures, amended smoke, recovery checks and prefix checks are under `results/pose-covariance/`; the original recovery archives are prerequisites rather than duplicated backup payload. No credentials are included.
