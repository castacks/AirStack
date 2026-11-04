# EXP132 — Unchanged signal transfer on saved TD3 captures

Date:2026-11-04. Branch:ore_ms. Parent:8f6e76f3e. Pre-extraction/outcome freeze:57a852bd8.

Ninewindows/four additional exposedTD3records;295core/171joint. Exact originalRGB+EXP023MSAthermal cohort, reported separately from providedexports. Sufficiency/recovery passes;all171score,0exclusions/fallback. No new inference or fitting. Sixfixedsignals/rules unchanged;all13readouts retain both original training sources for each ridge variant.

Pooled rotation RGB0.420089deg,T0.442255,raw0.178943,angular0.151686,oracle0.126388. Pooledtranslation RGB0.040137m,T0.044622,raw0.032869,angular0.033210. Neither fixeduncertainty rule beats recordingbestfixed rotation. Recording-level hindsightfixed mean0.130997 accounts98.431%of pooledoracle headroom. Known middle/96 contributes118.465%/106.434% of raw/angular netrotation gains vsRGB;other170collectivelylose. No broadadaptivity/SOTA/physicaltruth claim.

Combinedridge trainedsalon3 rotation0.203429deg;trainedsalon alwaysT0.442255. No refit/averaging/adoption.2701sufficiency+2979auditchecks/6mutations pass. Firstcoreaudit importpathfailure retained; predeclared same-source secondrun passes with PYTHONPATH added. Prediction/scoring unchanged.50directpins,9recovered sourcefiles SHAverified from153MBpriorBoxpacket. No reserves/source-output overwrites.

[Findings](../docs/research/signal-transfer-findings-2026-11-04.md) · [Protocol](../docs/research/signal-transfer-protocol-2026-11-04.md)

Next freeze independentbenchmark protocol/baselines and input/groundtruth qualification;furtherretuning these exposedwindows cannot establish generalization.
