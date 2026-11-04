# EXP138: explicit conditional common-time motion model

2026-11-04 UTC. Parent EXP137; freeze `692f85ad0`.
[Findings](../docs/research/common-time-model-findings-2026-11-04.md).

Separate offline model uses stationary SE3 body twist and explicit provided-ns
identity clock assumption, preserving both original time axes and source lineage.
Strict exact-native publisher unchanged. Twelve fixed-control trajectories retain
all 1,200 edges; thermal accepts day100/night98/rain98 of100, leaving four native
horizon gaps. No bridging, nearest-time search, fitting or fabricated online receipts.

105tests;49,118independent evaluated checks;30standard+13supplemental mutations
rejected;23new and2,467parent pins unchanged. All594coarse/fine predicted consistency
checks retained. Rain translation disagreement reaches1.147m; this is not truth
error or a camera quality signal. Accelerated synthetic control shows .08m model
error despite passing horizon. No actual reference scoring/inference/training/reserve
access. Physical clocks/calibration, model accuracy and adaptive benefit remain
unestablished; EXP133 benchmark stays execution-not-ready. Recovery
`backups/2026-11-04-common-time-model-exp138`; runtime inherited EXP136.
