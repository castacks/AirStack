# Temporal residual contract: fields sufficient under declared model

Run at 2026-11-13T21:13:48.403834-05:00. Branch ore-ms-orb-development, baseline a79658f63
plus source-hashed working changes. Frozen source and606 input hashes verified
before and after execution. One attempt, no source/tolerance changes or retries.

**FIELDS_SUFFICIENT_FOR_DECLARED_INDEPENDENT_INCREMENT_VARIABLE_DEPTH_MODEL.**
This is a mechanical input/derivation result, not statistical qualification of
real errors or a real-data accuracy result.

| Check | Result |
|---|---:|
| Camera windows retained | 120 /120 |
| Admitted tracks checked | 17838 |
| Non-root UV blocks checked | 70665 |
| Synthetic-state source-pixel Jacobian checks | 596 |
| Maximum reconstructed marginal absolute difference | 2.91e-11 |
| Maximum reconstructed marginal relative difference | 1.61e-15 |
| Minimum conditioned target-UV covariance eigenvalue | 0.000230997 |
| Maximum projection-Jacobian finite-difference discrepancy | 1.72e-09 |
| Maximum full residual-covariance reconstruction difference | 9.09e-13 |

Full source treatment, equations and integration controls are in the
[derivation](temporal-residual-derivation-2026-11-13.md).
All admitted tracks preserve their original prefix validity and source IDs.
Root-conditioned target covariance is positive definite for each checked track.
The uncertain-root residual block is reconstructed on the first admitted track
per window at fixed synthetic camera-local motions, including disparity/source-
pixel sensitivity; after deterministic root-UV removal it is positive definite.
No body-rig/reference state was fitted. Jacobian checks use the camera-local
projection form, not a new audit of physical rig transforms.

## Consequence

The variable-depth solver can receive a reference-free, independent-increment
track covariance from existing witnesses. The current implementation only uses
observation-wise marginals. Repeated source uncertainty cannot simply be added
again: root UV is fixed by construction, while source disparity already appears
as the depth-estimation factor.

The next implementation comparison should retain original marginal-independent,
source-conditioned marginal-independent, source-conditioned joint and a declared
scale control. Source conditioning changes marginal weights; its effect must not
be misreported as the benefit of cross-time terms. Begin with a state-independent
conditioned model; the uncertain-root state-dependent likelihood is a different
implementation and needs covariance-derivative qualification. No covariance
floors, track removals or weight tuning have been selected here.

This does not estimate shared network/image correlation, prove independence of
UV/stereo noise, certify flow covariance as process noise, or claim nonlinear
solver benefit. It establishes no calibration, training, physical timing or
fresh-record generalization result. No pose solver, native inference, reference
pose payload or new model was used. Original code/data were read only.

Artifacts: `results/temporal-residual-contract/run-001` contains the report,
post-primary intrinsics metadata supplement and exact inspected source copies.
Protocol: `experiments/temporal-residual-contract-freeze-2026-11-13.json`.
Input recovery: existing Box packet
`backups/2026-11-12-persistent-tracks-window-geometry-51cc40db/`, verified separately.
This audit reads only declared fields of existing tensor/track/witness NPZs;
it does not load the pose arrays co-located inside native tensor files.
