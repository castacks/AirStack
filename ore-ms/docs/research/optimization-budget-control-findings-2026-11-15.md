# Extra RGB optimization explains the position gain in the fixed-pair control

15 November 2026. Research baseline `16b60bef6`, with the source-pinned runner and
[pre-capture protocol](optimization-budget-control-protocol-2026-11-15.md).
All 600 qualified night intervals, source 599→600 through 1198→1199, were retained.
This is a cached, historical-parent fixed-pair mechanism test, not continuous VO.

## Decision

An additional full-pose RGB solve obtains essentially the same position gain as
an additional fixed-rotation translation solve. The two 601-pose path position
RMSEs differ by 0.430 mm, with full-pose slightly better. This does not establish
statistical equivalence, but gives no evidence that the rotation constraint is
responsible for the approximately 7.02% position improvement in this replay.
Credit the additional optimization, rather than a multisensor mechanism or a
new decoupling architecture, for that position gain.

The constrained pass has a separate useful tradeoff: exact native rotation
preservation and 48.71% less measured second-pass CPU time. Including the common
first pass, measured solver CPU falls 18.92% relative to full-pose refinement.
These are one-capture cached CPU measurements, not repeated timing statistics,
frontend savings or end-to-end VO speedups. The constraint remains an existing
family of methods, not established algorithm novelty.

## Primary interval endpoints

Both refinements receive identical accepted RGB measurements, source coordinates,
covariances and committed first-pass initialization for every pair. Each first
pass starts at that pair's historical native parent; new states do not propagate
between pairs. Translation-vector error and rotation-angle error use the supplied
reference increment on the same 600 intervals. No exclusion, alignment, scale fit,
clock correction, thermal factors, new inference or training is involved.

| Arm | Translation mean / RMSE / p95, m | Rotation mean / RMSE / p95, deg | Available |
|---|---|---|---|
| Native first pass | 0.072553 / 0.198469 / 0.485325 | 0.029481 / 0.066050 / 0.147574 | 600/600 |
| Extra full-pose RGB | 0.069039 / 0.190445 / 0.469093 | 0.029284 / 0.065659 / 0.147400 | 600/600 |
| Extra translation RGB | 0.069074 / 0.190573 / 0.469927 | 0.029481 / 0.066050 / 0.147574 | 600/600 |

Full-pose and translation-only refinement reduce interval translation RMSE by
4.043% and 3.979%, respectively. Full-pose slightly improves interval rotation
statistics, while its accumulated path orientation worsens. Preserve that
endpoint distinction rather than describing rotation as uniformly improved.

## Secondary composed fixed-pair paths

The 600 increments are composed from identity, retaining all 601 poses and the
initial reference gauge. The proper-SO3 serialization screen is unchanged.
These fixed-parent results are not exact re-executions of the earlier experiment's
arm-specific feedback histories; do not substitute their values into that study.

| Arm | Position mean / RMSE / p95, m | Orientation mean / RMSE / p95, deg |
|---|---|---|
| Native first pass | 32.289420 / 33.483146 / 36.312361 | 4.757796 / 4.989471 / 7.280048 |
| Extra full-pose RGB | 30.018410 / 31.131047 / 33.811739 | 4.910218 / 5.145332 / 7.448670 |
| Extra translation RGB | 30.018662 / 31.131477 / 33.810843 | 4.757796 / 4.989471 / 7.280048 |

Path position RMSE improves 7.0247% with the extra full-pose solve and 7.0234%
with the extra translation solve. Full-pose path orientation RMSE worsens 3.124%;
translation-only retains native rotation exactly at every pair.

## Measured solver work

Both additional passes use the unchanged native ten-step cap, stopping scheduler,
robust loss and precision model. Execution order alternates by pair. Timings wrap
`native.optimize`, including its graph conversion, solver setup and weighting;
they exclude graph construction and parity instrumentation. Scheduler steps count
outer optimizer calls, not internal rejected trust-region proposals.

| Arm | Solver CPU total, s | Actual scheduler steps | Combined CPU including native first pass, s |
|---|---|---|---|
| Native first pass | 23.998518 | 4,421 | 23.998518 |
| Extra full-pose RGB | 15.238642 | 1,371 | 39.237160 |
| Extra translation RGB | 7.815608 | 1,511 | 31.814127 |

Different stopping behavior is retained, not forced to equal iteration counts.
The CPU advantage occurs despite more outer calls for the translation arm.
Wall-time, cost distribution mean/RMSE/p95 and all per-call losses are in the
saved scores and predictions; no runtime significance claim is made.

## Qualification, provenance and audit

The exact Torch 2.7.1+cpu, PyPose 0.6.8, NumPy 2.2.6 and SciPy 1.15.3 runtime
was restored under Python 3.12; ancillary package versions can differ from the
historical environment. No original code was changed. Four night RGB input members
were recovered from the SHA-verified 1.21 GB decisive-evaluation archive and matched
the continuation's existing input pins; no new raw images or model weights were needed.

Three consecutive pre-capture pairs pass exact initial residual/covariance/
translation-Jacobian parity. Maximum constrained finite-difference relative error
is 5.009e-7 against the frozen 1e-4 bound. All 600 first-pass native reproductions
pass the 1 mm / 1e-4 rad bound; maxima are 0.06104 mm and 1.205e-7 rad. Both additional
passes produce outputs on all 600 pairs. Predictions were locked before scoring.

The independent auditor imports neither the runner nor optimizer. It reconstructs
accepted-factor counts, initial residuals and precision inputs from saved RGB
measurements, validates pose-delta/parent/initialization bindings, and independently
recomputes interval/path statistics, availability, timing totals and step accounting.
All 17,499 checks pass. Seven deliberate corruptions are rejected: population,
initialization, rotation, parent, step count, reference score and availability.
Source/input/reference locks and the complete capture stream are verified.

Evidence: `ore-ms/results/optimization-budget-control/`, including recovery proof,
qualification, freeze, capture stream, locked predictions, scores and audit receipt.
Raw outputs remain ignored in Git. The runner reuses the frozen native solver and
constrained wrapper; the separate auditor is required for independent validation.

## Stopping decision

The planned mechanism comparison is complete. Do not infer persistent thermal
benefit from the RGB refinement gain, or start a new fusion gate, weight search,
clock correction or window selection. The [short thermal transfer gains](fixed-rotation-recording-transfer-findings-2026-11-15.md)
and [negative longer thermal increment](fixed-rotation-continuation-findings-2026-11-15.md)
remain separate evidence. A future continuous-feedback or different-recording
comparison requires a distinct finite protocol against the strongest RGB control.
