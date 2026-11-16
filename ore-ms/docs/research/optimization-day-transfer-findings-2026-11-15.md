# RGB second-pass benefit transfers to the exposed day prefix

15 November2026. The [frozen day-transfer protocol](optimization-day-transfer-protocol-2026-11-15.md)
tests the unchanged three-arm own-state control on all99increments of
`_2021-08-06-11-23-45`. Both extra solves improve positionRMSE by approximately2.2%.
The full-pose solve again explains essentially the same position gain as the
translation constraint. This is a different recording from the600interval night
study, but an exposed100frame prefix of about10seconds, not blind/long-run validation.

## Results

Each arm feeds its final float32 pose into the next interval's point/covariance
registration and initialization. Accepted RGB measurements and solver settings
are unchanged. There are no thermal factors, new images, inference, training,
parameter searches, alignment or gap filling. All arms deliver99/99increments.

| Arm | Path position RMSE, m | Path orientation RMSE, deg | Total solver CPU, s |
|---|---|---|---|
| Native RGB | 1.584581 | 2.063867 | 4.392153 |
| Extra full-pose RGB | 1.549625 | 2.058741 | 6.824099 |
| Extra translation RGB | 1.548920 | 2.063869 | 5.571023 |

The refined path positionRMSEs differ by0.705mm, favoring translation-only. This
small descriptive difference is not a statistical equivalence test or evidence
of meaningful new architectural accuracy. Translation-only uses18.36%less total
measured solverCPU than full-pose refinement, but costs more than native. This is
one capture of cached CPU solves; graph construction is excluded, with no repeated
timing significance or end-to-end VO claim. Order rotates across the three arms.

| Arm | Interval translation mean / RMSE / p95, m | Interval rotation mean / RMSE / p95, deg |
|---|---|---|
| Native | .046390 / .052889 / .096042 | .047218 / .051366 / .080961 |
| Extra full pose | .046027 / .052479 / .096006 | .047372 / .051606 / .081479 |
| Extra translation | .045992 / .052460 / .095998 | .047218 / .051366 / .080962 |

Full-pose slightly improves path orientation while worsening interval rotation
statistics. Translation-only holds exactly its own first-pass quaternion; it does
not promise equality to the independent native stream once feedback states diverge.
Actual scheduler calls total680native,1088full-pose and1083translation-only;
refinement totals include both stages. Per-call losses and CPU/wall timings,
all interval/path mean/RMSE/p95 and availability are preserved in the raw packet.

## Qualification and audit

All105required source/config/RGB capture/reference files match SHA256values in
the earlier fixed-rotation-transfer Box manifest. The static/all-keyframe,
mapping-disabled disparity configuration and100source timestamps match. Existing
solver/wrapper qualifications are retained; every new native delta passes the
frozen1mm/1e-4rad reproduction gate. No failed interval, restart or waiver occurred.

Protocol, source and input/reference hashes were frozen before capture. Predictions
were locked before scoring. The independent saved-output auditor imports neither
runner nor optimizer; it verifies source locks, capture-stream identity, state
continuity, stage counts/costs, own-first-pass rotation, pose-delta reconstruction,
native gates and independently recomputed interval/path statistics. All3725checks
pass. Five deliberate corruptions—population,state,rotation,cost,score—are rejected.
No protected data, new recording outcome or physical timing qualification is claimed.

## Decision

The night result is supported by this short day prefix: credit extra optimization
for the position benefit, and treat the translation constraint as a scoped CPU/
orientation tradeoff. No new rotation/translation novelty or thermal contribution
is established. The earlier short thermal gain remains its own experimental result;
this RGB-only comparison does not replace it with a newly matched thermal score.

The finite transfer check is complete. No further experiment is selected. Any
longer recording, thermal incremental test or deployment needs a distinct protocol,
the strongest RGB control and appropriately measured cost/coverage endpoints.

Evidence is local under `results/optimization-day-transfer/`: freeze, capture,
prediction lock, predictions, scores and independent audit. Raw results stay outside
Git. This driver changes the recording/population while reusing the frozen native
solver, timed optimizer, feedback registration and constrained wrapper. Its separate
auditor preserves an implementation independent of the runner. No Box upload occurs.
