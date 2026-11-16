# Fixed thermal contribution survives stronger RGB controls on the day prefix

15 November2026. The [frozen ablation](thermal-strong-rgb-protocol-2026-11-15.md)
adds the existing fixed thermal precision0.1 to the matched own-state day-prefix
comparison. PositionRMSE improves approximately6.4%over both stronger RGB controls.
This confirms the earlier scoped day benefit; it is not a new model, new recording
or discovery of a previously untested thermal gain.

## Matched accuracy and coverage

All99intervals of the exposed100frame `_2021-08-06-11-23-45` prefix are retained.
The three RGB streams are copied unchanged from their qualified locked predictions.
The thermal arm runs its own RGB first pass, then a translation-only solve with
native RGB factors plus the existing calibrated thermal factors at precision0.1.
Its final float32 pose drives the next pair's source-point/covariance registration.
There are no new weights, gates, inference, training, reference alignment or gaps.

| Arm | Path position RMSE, m | Path orientation RMSE, deg | Available |
|---|---|---|---|
| Native RGB | 1.584581 | 2.063867 | 99/99 |
| Extra full-pose RGB | 1.549625 | 2.058741 | 99/99 |
| Extra translation RGB | 1.548920 | 2.063869 | 99/99 |
| Extra translation RGB + thermal0.1 | 1.449768 | 2.063819 | 99/99 |

Thermal improves positionRMSE over both extra-optimization RGB controls, not merely
over native. Path orientation is essentially unchanged against translation-only
RGB but slightly worse than extra full-pose RGB. The constraint preserves exactly
the thermal arm's own first-pass quaternion, not the other arms' full rotation streams.

| Arm | Interval translation mean / RMSE / p95, m | Interval rotation mean / RMSE / p95, deg |
|---|---|---|
| Strongest position RGB: extra translation | .045992 / .052460 / .095998 | .047218 / .051366 / .080962 |
| RGB + thermal0.1 | .044870 / .051262 / .094531 | .047218 / .051366 / .080967 |

Thermal also improves interval translation mean/RMSE/p95. The minute interval
rotation differences are retained rather than called identical or a broad rotation
gain. All four arms' full path and interval mean/RMSE/p95 remain in the saved scores.

## Computation and audit

Thermal costs8.518761s of measured solverCPU including both stages, versus5.571023s
for translation-only RGB and6.824099s for full-pose RGB. This is an accuracy gain
at added computation, not compute savings. RGB and thermal times come from separate
captures; these are descriptive cached CPU measurements, not randomized same-run
comparisons, timing significance or end-to-end VO performance.

All three thermal capture members match the existing Box manifest hashes, and100
thermal timestamps match the original source manifest. Protocol, source, all inputs,
references and RGB prediction locks were frozen before the99new outputs; predictions
were locked before scoring. There was no binding/solve failure or interval exclusion.
The native reproduction gate remains passing for every copied RGB baseline interval.

The audit confirms every copied RGB row object is unchanged after removing the new
thermal fields. It reuses the independent RGB audit arithmetic and separately
reconstructs thermal state continuity, own-first-pass quaternion retention, pose
deltas, both-stage cost/loss accounting and interval/path metrics from saved outputs.
The thermal arithmetic is a separate routine in the study driver, not a separately
maintained auditor module. All4636checks pass; four deliberate thermal corruptions
(state,rotation,cost,score) are rejected. This is saved-output verification, not an
independent reimplementation of the thermal optimizer or calibrated physical truth.

## Decision and limits

Retain this positive incremental thermal result alongside the
[negative longer-night increment](fixed-rotation-continuation-findings-2026-11-15.md).
It does not establish persistent thermal improvement, novel weighting/decoupling,
full frontend integration, physical timing qualification or blind generalization.
The recording/prefix and references are historically exposed, roughly10seconds long.

The finite stronger-control ablation is complete; no next experiment is selected.
Further validation needs a distinct qualified population and fixed endpoint, not
another weight/head/window search. Any practical adoption needs stronger evidence
than this prefix. Evidence is local at `results/thermal-strong-rgb-day/` with freeze,
locked predictions, scores and audit. Raw outputs remain outside Git; no Box upload.
