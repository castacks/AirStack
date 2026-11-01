# EXP098 — real-data covariance sanity fit and exact resume

2026-11-01 America/New_York. Execution protocol frozen before outcomes;public summary1951dcbfb.
[Findings](../docs/research/tartanair-covariance-sanity-fit-findings-2026-11-01.md).

One fixed16-step RGB synthetic-flow head finetune on16pairs,full4pair validation,
and8→16exactresume replay completed. Validated NLLworsens−1.831284→−1.551135
(delta+0.280149;lowerbetter),same2,191,698components. Noqualitygain/adoption claim.
IndependentFP64 NLL agrees withCUDA32 atfixedtolerance. 1,967resume comparisons
pass;24covariance tensors changed,404frozenentries/48validationmeanfields identical;
model/optimizer/RNG/last8journals exact on resume. 597source/data/init pins unchanged.

CandidateSHA762c65bfc98da634a998a08352795be56d0aca70faa760f6109aad7922382f6f.
Basepretraining overlapunknown;syntheticRGB only,no thermal/physical/VO proof.
No settings/window/checkpoint selection. Next training-only descent diagnostic,
not retrospective validation tuning. Full artifacts/checkpoints in Box recovery.
