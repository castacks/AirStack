# EXP132: unchanged signal transfer to saved MSA day/night captures

2026-11-04, branch `ore_ms`, parent `8f6e76f3e`, pre-recovery/extraction/outcome freeze `57a852bd8`.

The saved captures are sufficient for all 171 joint intervals across four additional exposed TD3 recordings. Fixed uncertainty rules improve the pooled rotation and translation means against the best pooled fixed camera. Their rotation advantage is concentrated in the single previously known RGB-collapse interval; neither fixed uncertainty rule beats the best fixed camera within any individual recording. The two frozen combined-ridge models transfer differently, so no model or rule is adopted.

This is a separate cohort using unchanged original RGB and EXP023 MSA thermal preparation. It is not pooled with the archive-provided exports in EXP130/131. All reference outcomes and recordings were already exposed in development. There is no independent physical truth, prospective generalization or SOTA benchmark claim.

| Method / frozen training source | Rotation mean (deg) | Translation mean (m) | RGB / thermal choices |
|---|---:|---:|---:|
| fixed_rgb | 0.420089 | 0.040137 | 171 / 0 |
| fixed_thermal | 0.442255 | 0.044622 | 0 / 171 |
| training_best_fixed | 0.442255 | 0.044622 | 0 / 171 |
| raw_uncertainty | 0.178943 | 0.032869 | 123 / 48 |
| angular_uncertainty | 0.151686 | 0.033210 | 139 / 32 |
| retention | 0.442255 | 0.044622 | 0 / 171 |
| occupancy | 0.383603 | 0.040061 | 31 / 140 |
| ridge_uncertainty@night_thermal_so_salon3 | 0.217793 | 0.033203 | 111 / 60 |
| ridge_health@night_thermal_so_salon3 | 0.367477 | 0.038834 | 37 / 134 |
| ridge_combined@night_thermal_so_salon3 | 0.203429 | 0.033439 | 116 / 55 |
| ridge_uncertainty@night_thermal_so_salon | 0.442255 | 0.044622 | 0 / 171 |
| ridge_health@night_thermal_so_salon | 0.442255 | 0.044622 | 0 / 171 |
| ridge_combined@night_thermal_so_salon | 0.442255 | 0.044622 | 0 / 171 |

The rotation oracle mean is 0.126388deg; the translation oracle is 0.026890m. The raw/angular fixed rules reduce pooled rotation mean by 57.404%/63.892% versus fixed RGB. Both have lower pooled translation mean here, unlike the provided-export cohort where their translation discrepancy increased substantially. The recipe/condition dependence prevents a universal tradeoff claim.

A hindsight best fixed camera chosen separately for each recording has weighted rotation mean 0.130997deg, compared with the per-interval oracle 0.126388deg. That recording-level choice accounts for 98.431% of the pooled rotation oracle headroom; only 3.518% further reduction remains below that comparator. This is a descriptive hindsight comparator, not a fitted or feasible recording classifier.

| Recording | Joint / core rows | Fixed RGB rotation | Fixed thermal rotation | Raw rule rotation | Angular rule rotation |
|---|---:|---:|---:|---:|---:|
| td3/20251027/0_salon_so_day | 95 / 95 | 0.106913 | 0.506804 | 0.152010 | 0.142821 |
| td3/20251027/1_salon_mac_cov_day | 45 / 45 | 0.129570 | 0.468139 | 0.215803 | 0.129570 |
| td3/20251027/night_mac_salon_warehousefig8 | 16 / 110 | 3.194976 | 0.105903 | 0.108040 | 0.108040 |
| td3/20251027/night_thermal_so_salon2 | 15 / 45 | 0.315217 | 0.314569 | 0.314569 | 0.320734 |

The only jointly supported middle-night interval is `exp112/middle`, pair96, the known failure examined in earlier work. RGB rotation discrepancy is 50.187674deg and thermal 1.337599deg. Both uncertainty rules choose thermal there. This one interval contributes 118.465% of the raw rule’s net pooled rotation improvement over RGB and 106.434% of the angular rule’s improvement. The other 170 intervals collectively lose to RGB on rotation. This is a post-score arithmetic influence diagnostic; the primary171-row population and frozen outcomes are unchanged, and no interval is removed.

For the combined ridge trained on salon3, pooled rotation mean is 0.203429 degrees; the model trained on salon chooses thermal on 171/171 intervals and has 0.442255 degrees. They use exactly their EXP131 means/scales/intercepts/slopes, without fitting, averaging, model selection or threshold adjustment. The small combined-salon3 advantage over fixed thermal on the night-mac recording (0.105242 vs0.105903deg) does not offset its worse-than-RGB daytime results under recording-level comparison. All 13 readouts, including both training sources for all three ridge variants, are reported.

![Frozen transfer comparison](../../results/signal-transfer/run-001/signal-transfer.png)

Population and recovery: nine fixed TD3 windows,295 guarded intervals,171 joint intervals and124 thermal-only intervals. Both zero-joint EXP113 middle/late windows remain in availability accounting. All 171 joint rows scored with0 reference/model exclusions and0 invalid-feature fallbacks. Source frame info is sensor_init->vehicle; all reference endpoints satisfy exact-ns support and <=150ms bracketing, finite/unit pose and monotone timestamp requirements. Nominal physical transforms and acquisition-clock equivalence remain unverified; the reference is interpolated recorded SLAM and its camera dependence remains unresolved. Checkpoint training overlap is unknown.

Six EXP112 windows already had local metadata/references. The exact153,043,760-byte EXP113 backup tar was downloaded from existing authorized Box backup and verified against manifest SHA256 `a19f1ce91bad66934436e93fc48da5b3e8d2ca4133576330fb28c248da842c3b`. Only nine selected files were restored to the new recovery subtree: six camera-info files before feature extraction and three reference files after prediction lock. Every selected member matches both archive manifest and historical source-plan hash. No source archive images or native inference were needed. Historical outputs were not overwritten. The full tar is a recoverable prerequisite and omitted from the incremental backup.

Preservation boundary: EXP112 had an original broad all-array mismatch on auxiliary arrays, with its scoped active input/mean/mask/event preservation subsequently passing. EXP113 strict off/on equality passed. This study reuses those saved native outputs under their existing qualifications; it does not promote the broader EXP112 equality claim or relax EXP128’s failed MSA producer replay.

Validation: 2,701 sufficiency checks and 2,979 independent audit checks pass, with 6 semantic mutations. Independent checks rebuild source graph/rectification, native eligibility, event features, frozen-model choices, SE3 matrix-log/exponential projection, reference interpolation and losses, and all selected summaries. A separate guarded reexecution runs the frozen predictor twice with in-memory writes and poisoned labels; its parsed inputs exclude labels/scores/evaluation and its outputs remain identical. Fifty direct source/input pins remain unchanged; the guarded auditor was committed in the same pre-extraction freeze.

The first core audit stopped at its final reference-mutation import with `ModuleNotFoundError: Scripts`. The failed log is retained, and a declaration pins its source/output hashes before a second run. The unchanged auditor then passed with only `PYTHONPATH=/workspace/MAC-VO-main` added to the same pinned runtime. No prediction, reference, method, scoring parameter or numerical package changed. Both attempts are retained. The known CPU driver-init warning also appears with successful exit codes.

Prediction SHA256: `afac44abf20d42e1535d3d06f92db6d1c6e26bb946a824ee7470e706397745d8`. Evaluation SHA256: `685adc31cebaaaa144387edab41802fc933a1a144e184a4a74b320babdc08c3c`. Artifacts: `results/signal-transfer/run-001/` contains sufficiency, feature/motion records, prediction lock, source reference audit, labels, all 13 readouts, core/causal audits, retained failed attempt/declaration and figures. `results/signal-transfer/recovery-001/` contains recovery manifest/selected sources. [Protocol](signal-transfer-protocol-2026-11-04.md).

Research consequence: the unchanged uncertainty rules respond usefully to a known collapse and broad recording differences, while stable within-recording quality superiority and learned transfer remain unestablished. Further tuning on these exposed windows would not resolve that evidence gap. The next stage should freeze an independent benchmark protocol with fixed-camera and simple condition controls, official metrics, disclosed training overlap and recipe-specific reporting. Additional physical-accuracy claims still require the documented reference/clock/mounting evidence; unknown training membership alone does not prohibit a clearly qualified provided-reference comparison. No policy is adopted.
