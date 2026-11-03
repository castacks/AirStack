# EXP121: continuation fails the overall availability gate

2026-11-03. Freeze c3e76e894; parent EXP120. [Full findings](../docs/research/readiness-continuation-findings-2026-11-03.md).

Four fresh dual captures preserve the synthetic day100 outages and native estimator. The candidate adds inactive requests only, capped at four attempts per session and twenty extra requests per run.

All four restoration episodes regain on-time adjacent output, with source-group delays 4/3/2/5. Original bounded probing observes only the third episode. Candidate uses 12 extras and at most three attempts per session. However, eligible preselected deliveries fall to 84/95 from 85/95; unavailable selections rise to five from four. The primary development contract FAILS. No adoption.

At handover 51, RGB source50 had been skipped after a completed readiness session. The next estimate spans 49→51 and cannot satisfy the adjacent 50→51 target. First eligible alternate delivery moves from control52 to candidate53. Past readiness does not guarantee current handover continuity.

Candidate pair calls 139 versus original133/full158; native processing 49.067 versus46.954/55.903 seconds. Across 84 exact matched eligible intervals, candidate translation/rotation proxy RMSE is 0.066300m/0.561546deg; full-read 0.066281/0.561573 and original 0.066338/0.561550. These small differences do not override the availability failure.

Forty-three tests and 28,412 independent checks pass; 951 pins remain unchanged; all 5,226 unperturbed arrays are exact. All masks, sessions, caps and missed results remain preserved. Synthetic serial-offline evidence does not establish natural-fault or physical accuracy.

Next: separately freeze handover continuity or bounded buffered catch-up, with explicit input-retention, deadline and cost rules. No such tuning occurs here. Backup: backups/2026-11-03-readiness-continuation. Original data/weights/compiler EXP112, base runtime EXP106, control/inherited coverage EXP120.
