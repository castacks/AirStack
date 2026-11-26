# Task calibration and screening

The candidate event is a feasible visible-target localization within 1 px of the reference centroid, conditional on the frontend emitting a candidate. All candidates enter this denominator, including candidates on negatives. Bin each frozen raw score into ten equal-width bins with 1 included in the final bin. Fit an independent mapping for each arm on calibration using (correct + 1)/(candidates + 2). The unoccupied-bin prior is 0.5 and is retained with its zero count. Brier score and observed bin outcomes use locked holdout candidates. Scores used for threshold selection remain raw contrast rankings.

Development ranking applies the frozen screening rule diagnostically and selects the arm by correct localization count, then fewer absent/hidden false-supported cases, then declared arm order. Within each arm and split the threshold sweep first screens absent and fully hidden separately, then maximizes correct count, minimizes primary false-supported count and selects the higher cutoff on ties. The candidate threshold requires useful issuance. If none qualifies, record the above-maximum cutoff and withhold an operating claim. B03 ranked highest on development with 2,167 correct, versus 2,095 for B01 and 1,663 for B02. The holdout never selects the comparator or cutoff.

| Arm | Calibration cutoff | Calibration correct | Absent and hidden, each | Upper Wilson 95%, each |
|---|---:|---:|---:|---:|
| B01 RGB | 0.20 | 2,085 | 0/128 | 2.914% |
| B02 LWIR | 0.20 | 1,661 | 0/128 | 2.914% |
| B03 equal fusion | 0.30 | 2,170 | 0/128 | 2.914% |

Use a two-sided Wilson 95% interval with z=1.96 and screen its upper endpoint against 5%. All three holdout points passed separately for absent and hidden, with 0/256 observed and 1.478% upper bounds in each arm and stratum. A zero observed count still has a positive upper bound. Distractor and insufficient-partial safety claims are unsupported. Combined counts over related interventions are descriptive. The single-case cells each have one opportunity per independent realization.

Report overall coverage as issued/all and feasible coverage as issued feasible/all feasible. Selective risk is incorrect or infeasible/issued. Localization error is the mean Euclidean error among issued feasible opportunities, including errors above 1 px. The conditional error table counts candidate correctness on feasible opportunities where both sensors supplied eligible pixels. Candidate absence is a failure in that table. [Locked calibration and sweep](evidence/calibration.json) · [Holdout tables](evidence/holdout.json)
