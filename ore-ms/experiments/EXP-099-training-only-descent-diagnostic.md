# EXP099 — train-only optimization/descent diagnostic

2026-11-01 America/New_York. Pre-outcome protocol571eee374.
[Findings](../docs/research/training-only-descent-findings-2026-11-01.md).

Training NLL(initial/8/16)−6.323683→−6.476601→−6.616049;all16pairs improve,
same9,375,466components/192meanfields. FirstbatchAdam loss−3.897780→−4.021797;
fixed±1e-4gradient probe:FD141.7435vsanalyticquantized142.2233,0.337%discrepancy,
correctdirection;initialstate restores exactly. Independent1,174checks pass.

No validation decoded,newdata,settings sweep,candidate export/promotion or source
changes. Existing validation regression and all checkpoint/result bytes retained.
Localtrainingdescent works;training-to-validation gap observed,cause notproven.
Unknownbaseoverlap,syntheticRGBscope;next broader recording-level development
plan,not a loss-sign patch or thermal/physical/generalization claim.
