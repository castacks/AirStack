# EXP101 — conservative final covariance convolution

2026-11-01 America/New_York. Newpolicy/tests/frozenexperimentdedcf163e.
[Findings](../docs/research/final-layer-adaptation-findings-2026-11-01.md).

Samebroad48data/init/optimizer/48updates;onlyconv4weight/bias(2tensors/1154scalars)
vsfullblock24/3,075,202. Validation recurrentNLL base/full/last−1.831284/−1.712671/
−1.806925;finalstage−0.403946/−0.360315/−0.395218. Smalleradaptationreducesregression,
butinitializationstillbest;no candidatepromotion. Sharedold16trainfinalstage
actuallyworsens vsbase;allcohort/metricoutcomes retained.

Valteacher→studentKL0.024672→0.003897,RMSrawshift0.148962→0.064207;diagnostic
nottrainingpenaltyorqualitycriterion.250CPUtests pass1CUDA-onlyskip;1222independent
checks,240meanfields/426frozenentries unchanged;actualGPU24→48resume exactmodel/
optimizer/RNG/journal.597oldsource/data/candidate/validationSHAunchanged,newpolicy
andtestsadditive. Basepretrainingunknown/syntheticRGB/oneexposedvalrecording;
notthermal/physical/independenttest. Next frozen-candidate multirecordmonitoring
beforefurthertuning. Allnewmodels/proofs inBoxrecovery.
