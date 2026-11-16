# Full-span day thermal increment is small against the strongest RGB control

15 November2026. The [predeclared fallback protocol](long-day-fallback-protocol-2026-11-15.md)
completes all599increments of the first600frames of original candidate day
`_2021-08-13-16-31-10`. Thermal with fixed precision0.1 and RGB-only fallback
improves path positionRMSE0.140%against the strongest RGB control, but worsens
path orientation approximately1.94%and costs more solverCPU. This is a modest
translation-specific result, not a clear overall improvement or new method.

## Full-span results

All arms return599/599increments and600poses without exclusion, restart, scale
fit, trajectory alignment or gap filling. Each arm carries its own final float32
state through next-pair registration and initialization. Native uses one solve;
refinement arms use their own RGB first solve plus the declared second solve.
The constrained arms retain exactly their own first-pass quaternion, not the
independent native or full-pose RGB stream.

| Arm | Path position RMSE, m | Path orientation RMSE, deg | Total solver CPU, s |
|---|---|---|---|
| Native RGB | 11.115217 | 4.036908 | 19.495101 |
| Extra full-pose RGB | 11.052653 | 3.959881 | 34.147071 |
| Extra translation RGB | 11.074904 | 4.036536 | 26.924612 |
| Thermal0.1 translation + RGB fallback | 11.037158 | 4.036548 | 37.830246 |

The primary position endpoint beats BOTH refined RGB controls with full coverage:
0.140%over full-pose and0.341%over translation-only. The stronger RGB position
control here is full-pose, not translation-only. Its orientation is also better.
Thermal's absolute positionRMSE increment over that control is only0.01550m,
versus approximately6.4%on the separate short day transfer prefix. Do not pool
those populations or treat this as a longer version of that transfer recording.

| Arm | Interval translation mean / RMSE / p95, m | Interval rotation mean / RMSE / p95, deg |
|---|---|---|
| Native | .046604 / .083481 / .120190 | .017523 / .028727 / .065782 |
| Extra full pose | .046492 / .083349 / .119142 | .017421 / .028534 / .065639 |
| Extra translation | .046477 / .083335 / .118962 | .017522 / .028726 / .065782 |
| Thermal + fallback | .046332 / .083243 / .118917 | .017522 / .028726 / .065782 |

Thermal improves interval translation statistics, but full-pose RGB retains better
interval rotation. Tiny constrained-arm rotation differences are not exact equality.
All path mean/p95, interval metrics and per-call costs/losses remain in the packet.

## Explicit fallback and computation

The thermal arm uses current calibrated thermal factors on595intervals. At pairs
25,27,28,32 it uses RGB-only translation refinement at its OWN parent/first-pass
state. It never copies another arm's pose or retains stale thermal factors.
Fallback is triggered only by unsupported saved native thermal first-write, not
by reference scores, residual size, solve failure or a learned decision. No other
binding/solve failed. The earlier stopped all-thermal attempt remains preserved.

Thermal solverCPU is10.79%higher than extra-full-pose RGB and40.50%higher than
extra-translation RGB. These are one-capture cached solver measurements, including
both stages but excluding frontend, registration and graph construction; not
end-to-end VO timing or repeated statistical evidence. Four-arm order rotates.
Actual outer scheduler calls total3313native,4914full-pose,5198translation-only,
7122thermal+fallback, with both stages included for refined arms.

## Qualification and saved-output audit

Whole-span preflight verifies599RGB and595thermal bindings and exactly the four
declared fallback pairs. At all four, packet=None thermal graph and native RGB
graph have bitwise-identical residuals,covariances,translation Jacobians and final
committed solutions/losses under identical inputs. The adapter reuses the existing
capture loop and NativeThermalGraph's no-packet RGB behavior; no frozen solver or
runner is edited. Adapter/protocol/preflight/source/input/reference hashes are
frozen before predictions; all599predictions lock before reference scoring.

The original25interval pre-fallback prefix reproduces the stopped attempt exactly
in500pose/loss/step checks; timing is excluded. All599native delta reproductions
pass the unchanged1mm/1e-4rad gate. No threshold waiver or successful-window selection.

A separate auditor imports neither capture adapter nor optimizer. It checks source
locks, input support from raw event maps, accepted thermal factor indices, exact
fallback identities, stream/pose/state continuity, own-first-pass rotation retention,
stage/cost accounting and independently recomputed reference statistics. All37813
checks pass. Six semantic corruptions—population,state,rotation,cost,score,fallback—
are rejected. This is saved-output/input-binding verification, not an independent
thermal optimizer reproduction or physical reference certification.

## Decision and limits

The declared full-coverage position endpoint passes, with small magnitude and
rotation/compute costs against the strongest RGB control. Retain it as scoped
positive evidence alongside the stronger short-day benefit and longer-night thermal
regression. Do not call it broad superiority, matched-accuracy compute savings,
novel weighting/decoupling or persistent thermal benefit across conditions.

This is exposed supplied-reference development data and cached-measurement state
feedback, not new frontend inference, blind transfer, physical exposure calibration
or live VO. No weight/head/window/clock tuning or new dataset follows automatically.
The finite comparison is complete; no further experiment is selected.

Evidence is local under `results/long-day-fallback-control/`: preflight/equivalence,
freeze, capture stream, prediction lock, scores and independent audit. Raw outputs
remain outside Git. The earlier incomplete run is preserved separately under
`results/long-day-strong-controls/`. No Box upload is performed.
