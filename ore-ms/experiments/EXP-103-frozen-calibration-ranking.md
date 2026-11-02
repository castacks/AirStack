# EXP103 — frozen calibration and uncertainty ranking

2026-11-02 (client date, America/New_York), branch ore_ms.
Contract d1cf5ed82; source/candidates EXP102 at48333b736.

[Full findings, per-record curves and figures](../docs/research/frozen-calibration-ranking-findings-2026-11-02.md).

Same36 exposed RGB pairs and3 frozen models, final stage only. No training,
fitting, data acquisition, export or promotion. Final-layer has closer nominal95
coverage on each record; added-two macro89.154% vs original87.331% (still below95).
Full-block ranks better at50% retention: macroMSE1.098781 vs final-layer1.340195
and original1.409787 pixel². Thus NLL/95% coverage/ranking favor different candidates.

First GPU capture / CPU metrics / independent audit pass. 6,469 checks,36 retained
per-pixel NPZs, exact EXP102 mean/raw/mask/NLL reproduction, frozen model state,
599 source files and protected old inputs/artifacts unchanged. Full curves,
oracle/random references, normalized moments and source bindings retained.
Unknown pretraining overlap; synthetic RGB prefixes; no independent-test,
thermal/physical VO or generalization claim. Prior7 MS2 readiness gaps unchanged.

Recovery: backups/2026-11-02-frozen-calibration-ranking. Next freeze training-side
ranking-objective design and comparison contract before new optimization, with
separate future evaluation evidence required. No automatic next training cycle.
