# EXP106 — Matched stage/objective training comparison

Frozen execution: `c7c2c7fd3`. [Findings](../docs/research/objective-comparison-findings-2026-11-02.md).

Three matched 48-update arms and all three 24→48 replays passed. All 36 observed synthetic RGB development pairs retained. Primary new-two-record macro MSE at 50% retention: recurrent 1.098781, final diagonal 1.268779, final trace 1.230109 px². Final trace improves against final diagonal by 3.05%, with worse aggregate 95% coverage and final diagonal NLL; office and legacy ranking regressions retained. No dominance or promotion.

111 CPU tests / 3 skips; 3 CUDA tests; fresh native preflight exact to historical evidence. Training audit 10,829 checks; final audit 15,863 including repeated training checks. Full checkpoint/RNG/optimizer/journal replay exact for all arms; 24 changed covariance tensors / 404 frozen entries, all 432 mean fields shared. All arms clip 48/48 updates.

Recovery included 8 verified packets / 1,417 members and rebuilt pinned-base runtime `9c0ea271…`; historical baseline/recurrent stage hashes and recurrent model match exactly. Harness failures retained; scientific commands first-attempt success, no tuning. Unknown pretraining / exposed synthetic RGB / no thermal or physical claim; prior MS² requirements and reserves unchanged. Next: separately frozen recording-level evaluation before further tuning.

Recovery: `backups/2026-11-02-objective-comparison`, including exact rebuilt image.
