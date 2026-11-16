# Extra RGB optimization gain survives cached pose feedback

15 November 2026. The [frozen own-state protocol](optimization-feedback-protocol-2026-11-15.md)
extends the [fixed-pair control](optimization-budget-control-findings-2026-11-15.md)
to all600night intervals599→600 through1198→1199. Both refinements achieve
approximately7.03%lower path positionRMSE than native. Full-pose refinement again
explains essentially the same position gain as translation-only refinement.

## Results and interpretation

Each arm starts at the same archived source599pose and carries its final float32
state into source-point/covariance registration and initialization for the next
pair. Refinement arms each execute their own first native solve, then the declared
second pass. Accepted cached measurements remain unchanged; no frontend is rerun.
All three arms deliver600/600intervals and601poses, without gap filling or alignment.

| Arm | Path position RMSE, m | Path orientation RMSE, deg | Total solver CPU, s |
|---|---|---|---|
| Native RGB | 33.482860 | 4.989486 | 21.969049 |
| Extra full-pose RGB | 31.129045 | 5.136693 | 36.142292 |
| Extra translation RGB | 31.127841 | 4.977854 | 28.804168 |

The refined positionRMSEs differ by1.204mm, favoring translation-only. This is not
a statistical equivalence test or evidence of a meaningful architectural accuracy
advantage. The principal position gain remains attributable to extra optimization.
Translation-only uses20.30%less total measured solverCPU than full-pose refinement,
including each arm's own first solve. It still costs more than native's single pass.
This is one CPU capture, not repeated timing statistics or end-to-end VO performance.

Unlike the fixed-pair experiment, translation-only does not preserve the independent
native arm's entire rotation stream. It preserves exactly its own first-pass
quaternion at each pair; different prior states change later first-pass solutions.
Its slightly improved path orientation must retain that feedback qualification.
Full-pose path orientation worsens even though interval rotationRMSE improves.

| Arm | Interval translation mean / RMSE / p95, m | Interval rotation mean / RMSE / p95, deg |
|---|---|---|
| Native RGB | .072554 / .198469 / .485325 | .029481 / .066050 / .147571 |
| Extra full pose | .069038 / .190440 / .469127 | .029284 / .065638 / .147400 |
| Extra translation | .069067 / .190565 / .469840 | .029486 / .066050 / .147658 |

Full mean/p95 path metrics, per-call wall/CPU times and losses are retained in
`scores.json` and `predictions.json`. Actual outer scheduler calls total4421native,
5841full-pose and5952translation-only; refinement totals include both stages and
do not count inner rejected trust-region proposals. Arm execution order rotates
by interval. Timing excludes graph construction and instrumentation.

## Qualification and audit

Source, protocol, input and601reference hashes were frozen before capture, and
predictions locked before reference scoring. The unchanged native graph/solver
and constrained wrapper inherit the passing three-pair qualification from the
fixed-pair study. Torch2.7.1+cpu/PyPose0.6.8/NumPy2.2.6/SciPy1.15.3 run single-threaded.
All600native relative-motion reproductions pass the frozen1mm/1e-4rad gate.

The independent auditor imports neither experiment runner nor optimizer. It
verifies source/input/reference locks, the capture stream, each arm's state
continuity, pose-delta bindings, first-pass rotation retention, both-stage cost/loss
accounting, native reproduction and independently recomputed support/interval/path
statistics. All19299checks pass. Four semantic corruptions—state continuity,
rotation, step accounting and scores—are rejected. No attempts or intervals failed.

The existing scoring utility is reused unchanged. Its JSON field `fixed_pair_path`
is a schema name; here its increments come from own-state feedback. Freeze scope
and this report explicitly identify that distinction. These are supplied-reference,
historically exposed development data, not fresh-record or physical calibration
evidence, fresh frontend execution, live VO or established novelty.

## Decision and recovery

The finite feedback comparison is complete. Retain translation-only refinement as
a scoped cost/orientation tradeoff against extra full-pose optimization, not a new
thermal contribution. The short thermal gains and negative longer thermal increment
remain separate. No new weights, clock search, thermal head or recording is selected.
A different-recording validation would require its own finite protocol and controls.

Evidence is local at `results/optimization-feedback-control/`, with freeze,
streamed capture, locked predictions, scores and independent audit. Original input
hashes and Box recovery locations are inherited from the fixed-pair packet. Raw
outputs remain outside Git; no Box upload is performed by this comparison.
