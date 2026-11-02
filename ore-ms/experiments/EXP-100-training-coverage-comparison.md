# EXP100 — matched48-update coverage comparison

2026-11-01 America/New_York. Prepixel18af8a0b7;execution28fc479d9;hostfixbdd96285d.
[Findings](../docs/research/training-coverage-findings-2026-11-01.md).

Original16pairs×3passes vs48pairs/3trainingenvironments×1roundrobinpass;
sameinitialization/optimizer/48updates,unchanged4pairdevelopmentvalidation.
Validation recurrentNLL base/narrow/broad−1.831284/−1.286259/−1.712671;
finalstage−0.403946/−0.278459/−0.360315. Broaderarm reducesregression vsnarrow,
butinitialization remainsbest. No promotion or independentqualityclaim.

32newpairs/98rawmembers,352independentprepchecks/128arrays;24recordledgerold22
preserved. All5modelcommands firstactualsuccess afterprelaunchhostorchestration
failure(no modelstarted)retained/fixed. 484cohort/NLL/meanchecks pass;24covariance
changes/404frozenentries eacharm;240commonmeanfields identical;597source/old
candidate/data/validationSHA unchanged. Checkpoint counters/order correct.
RGBsynthetic,unknownbaseoverlap,oneexposedvalidationrecording;corpus/frequency/
ordering differ,notpurecausaldiversityidentification. Allartifacts/checkpoints
retained inBoxrecovery. Prior7MS2requirements unchanged.
