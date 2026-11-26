# Frozen experiments and dispositions

“Eligible” means at least one sensor passed the acquisition and mask criteria unless the row is a context-specific admission study. Each single scenario has 256 independent realization groups and 256 opportunities. Multi-scenario rows count paired opportunities across the same 256 groups. Rates across repeated interventions are descriptive. The scenario table contains the pointwise grouped intervals and per-arm denominators for every frozen cell. A 256/256 or 0/256 endpoint is degenerate under resampling this fixture population. [All scenario metrics and intervals](evidence/scenario_metrics.csv)

| Study | Opportunities / eligible | Measured result | Qualification |
|---|---:|---|---|
| E01 historical replay | 6,912 / 6,400 | B01 4,176, B02 3,324, B03 4,327 correct out of 5,120 feasible. All 16 PF1 hashes reproduced. | Historical thresholds unchanged. |
| E02 constant reliability | 6,912 / 6,400 | B04 4,324/5,120 correct at 73.71% coverage. | A01 − B04 +5.55 points, paired 95% interval +5.35 to +5.74. |
| E03 A01 current evidence | 6,912 / 6,400 | 4,608/5,120 correct at 77.78% coverage. | A01 − B03 +5.49 points, paired 95% interval +5.31 to +5.68. |
| E04 M07 | 6,912 / 0 context eligible | Fallback is identical to E03. | No admissible association and transfer contract. No context-effect estimate. |
| E05 M08 | 6,912 / 0 context eligible | Fallback is identical to E03. | No t=0 observations. No forecast-effect estimate. |
| E06 M09 | 6,912 / 0 context eligible | Fallback is identical to E03. | No t=0 observations. No learned forecast or novelty-effect estimate. |
| E07 environmental ablation | 6,912 / 6,400 | Identical to E03 at every opportunity. | E03 has no context to remove. |
| E08 learned reliability ablation | 6,912 / 6,400 | 4,327/5,120 correct, with 5,095 issued. | Restores the fixed-conflict abstention behavior. |
| E09 dependence ablation | 6,912 / 6,400 | 4,608/5,120 correct, with 5,376 issued and 0.07643 Brier. | No observed correct-count effect. |
| E10 missing channels | 768 / 512 | 512/768 feasible correct. 256 through each single modality. All 256 both-missing abstained. | No improvement over B03 for missing sensors. |
| E11 detector degradation | 1,024 / 768 | Clipped, noisy, RGB-attenuated and LWIR-attenuated scenarios. A01 768/1,024 feasible correct, with 0/256 clipped. | No clipped detector recovery. Gain over B03 confined to the attenuation controls. |
| E12 thermal crossover | 256 / 256 | RGB fallback localized 256/256. LWIR found no candidate. | Same as B03. No contrast-based LWIR reliability claim. |
| E13 occlusion and partial support | 1,280 / 1,280 | Correct 512/512 feasible across partial and 4/16. Full hidden and 3/12 had zero issues. The 4/20 fixture had 256/256 false issues. | Withhold 4/20 operating claim. |
| E14 distractors and equivalence | 768 / 768 | 256/256 reference correct and 512/512 negative issues across the two distractor cells. | Same admissible inputs produced the same decisions in all 256 matched pairs. |
| E15 nuisance mismatch | 768 / 768 | Correct 768/768 across wrong air and unknown material or range using detector evidence. | No atmospheric/source inference or recovery claimed. |
| E16 dynamics shift | 768 / 768 | Correct 768/768 and blind-future output exactly matched reference. | No future-dynamics identification or forecast qualification. |
| E17 registration and clock | 768 / 768 | Correct 768/768. 512 declared LWIR exclusions and 256 RGB resolutions of the fixed hidden conflict. | Gain dominated by one direction and size of LWIR offset. |
| E18 end-to-end | 6,912 / 6,400 | A01 achieved 90.00% correct per feasible at 77.78% coverage, lower selective risk and 0.687 s restricted-issuance runtime. | Resolution, qualification and compute are conditional on the frozen setup. |

The holdout model and threshold were fixed before adaptive holdout inference. Variant outcomes are secondary. Matching the A01 conflict fixture performance does not validate learned reliability or dependence correction outside the controlled scenario matrix. [Detailed results](RESULTS.md)
