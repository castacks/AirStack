# EXP112: matched geometry capture and fixed RGB–thermal selector evaluation

2026-11-02 America/New_York. Completed qualified development comparison.
**Original all-array preservation gate FAIL; active-estimator preservation PASS;
fixed policy's strict quality criterion FAIL. No adoption.**

[Full findings](../docs/research/sensor-admission-findings-2026-11-02.md).
Twelve final shared captures, 24 native expert runs, 1,120 frame executions,
six original windows, three exposed recordings, 274 retained decisions. Original
checkpoint/inputs/policy/budgets/solver unchanged. First runtime's missing-compiler
failure retained; compiler amendment frozen and all captures rerun in one runtime.

198 derived covariance/depth-diagnostic arrays differ numerically; 14,518 other
arrays including active weights, points, masks and poses match bitwise. Scoped
preservation amendment committed before actions/labels retains the strict failure.
451 native information events and 439 interval transports independently rebuilt;
all policy/errors/scores audited. 78 tests, 51,654 numeric checks, 1,651 source/input
and nine reference hashes pass. These do not establish physical accuracy.

Night raw translation proxy RMSE 0.101162 → 0.039674 m, rotation 4.702700 → 0.645755
degrees at identical 116-decision coverage; both days unchanged. Entire gain comes
from known recovery middle/96. Current-geometry-only and candidate are identical
on all 274 actions; recovery memory adds no value here. History has identical
matched errors and six fewer cold-start decisions overall; thermal-only remains
slightly better on night. Every strict per-record comparison fails.

Original attempts under `results/sensor-admission-execution/run-001`, final scoped
outputs under `run-002`. No training, reserve access, threshold sweep, joint graph
fusion, live latency or other-sensor result. Backup:
`ore_ms:ore-ms/backups/2026-11-02-sensor-admission-execution`.
