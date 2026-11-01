# EXP096 — manifest-bound covariance-head finetuning runner

2026-11-01 America/New_York. Implementation4687a5649→40b7e57b6→c1619bc4c.
[Findings/CLI/contracts](../docs/research/covariance-retraining-runner-findings-2026-11-01.md).

27new software tests;232CPU pass/1CUDA-onlyskip plusCUDA1pass. Synthetic CPU/CUDA
uninterrupted/resumed fixture executions match model/optimizer/RNG/journals exactly.
Actual RGB backend passes final synthetic backward-only check:12stages,24trainable
covariance parameter tensors,finite gradients,frozenmean/allstate unchanged,
zerooptimizersteps. Retain initial different-freeze-control failure and v2/v3
amendments;inputs/targets/zero-tolerance assertion unchanged.

591prior source/2weights unchanged. Explicit source/label/data/init/runtime pins,
recording-level partitioning,strict full-state resume and atomic checkpoints.
Actual dense-flow training manifest not identified in bounded local/Box metadata
checks;published template intentionallyincomplete. No real-data fit,reserve/test
payload or qualityclaim. Next source-backed labels/preparation/split/budget.
