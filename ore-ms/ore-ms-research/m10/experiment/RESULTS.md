# Controlled synthetic evaluation

Each arm saw 6,912 opportunities from 256 holdout realization groups, with 5,120 feasible opportunities. At least one sensor was eligible on 6,400 opportunities, RGB on 6,144, LWIR on 5,632, and both on 5,376. The frozen primary A01 correctly localized 4,608 feasible opportunities. B03 correctly localized 4,327. The difference of **+5.49 percentage points** has a paired 95% interval **+5.31 to +5.68 points** across 10,000 bootstrap draws seeded with 10091.

| Arm | Correct / feasible | Issued / abstained | Coverage overall / feasible | Selective risk | Conditional mean error | Candidate Brier / ECE |
|---|---:|---:|---:|---:|---:|---:|
| B01 RGB | 4,176 / 5,120 | 4,979 / 1,933 | 72.03% / 82.25% | 16.13% | 0.020760 px | 0.12932 / 0.00743 |
| B02 LWIR | 3,324 / 5,120 | 4,352 / 2,560 | 62.96% / 70.00% | 23.62% | 0.217638 px | 0.17544 / 0.00334 |
| B03 equal | 4,327 / 5,120 | 5,095 / 1,817 | 73.71% / 84.51% | 15.07% | 0.000085 px | 0.12671 / 0.00586 |
| B04 constant reliability | 4,324 / 5,120 | 5,095 / 1,817 | 73.71% / 84.51% | 15.13% | 0.003536 px | 0.12649 / 0.00077 |
| A01 adaptive | 4,608 / 5,120 | 5,376 / 1,536 | 77.78% / 90.00% | 14.29% | 0.002613 px | 0.07625 / 0.00090 |
| O01 unconstrained diagnostic | 4,608 / 5,120 | 4,608 / 2,304 | 66.67% / 90.00% | 0% | 0.000119 px | — |
| O01 restricted to B03 issuance opportunities | 4,327 / 5,120 | 4,327 / 2,585 | 62.60% / 84.51% | 0% | 0.000085 px | — |

The O01 scorer picks the nearest correct location from the actual B01–B04 and A01 proposals on that opportunity. It abstains when none is correct. The B03-mask variant may evaluate a candidate only where B03 issued. No new location or claim of deployable confidence comes from the oracle. A01 reaches the available-candidate correct-count ceiling while issuing 768 false-supported locations the oracle withholds.

| Paired holdout contrast | Point gain | 95% interval |
|---|---:|---:|
| A01 − B01 | +8.44 points | +8.14 to +8.71 |
| A01 − B02 | +25.08 points | +25.02 to +25.16 |
| A01 − B03 | +5.49 points | +5.31 to +5.68 |
| A01 − B04 | +5.55 points | +5.35 to +5.74 |
| A01 − B03 issuance coverage | +4.07 points | +3.94 to +4.21 |

All 281 A01 recoveries were feasible. They came from the fixed hidden three-pixel LWIR registration shift (256), RGB attenuation (22), and LWIR attenuation (3). A01 lost zero B03-correct opportunities. Missing-modality and thermal-crossover outcomes were identical between these arms. As a post-hoc sensitivity check excluding the hidden-registration fixture, the gain falls to **+0.51 points**, or 25/4,864, with a descriptive paired interval of +0.33 to +0.72 points. The result does not establish robustness to other misregistration geometries, calibration errors or a different sensor population.

## Matched issuance coverage

Percentages below are correct per feasible opportunity. Each row ranks candidate scores and uses a fractional exact-tie allocation at the same coverage across arms. These are frozen-score descriptive curves. The cutoffs they imply are not additional deployment rules.

| All-opportunity coverage | B01 | B02 | B03 | B04 | A01 |
|---|---:|---:|---:|---:|---:|
| 50% | 57.39% | 51.35% | 57.44% | 57.53% | 63.10% |
| 60% | 69.63% | 61.00% | 69.82% | 68.20% | 76.00% |
| 65% | 75.53% | Insufficient candidates | 75.48% | 73.53% | 82.75% |
| 70% | 79.77% | Insufficient candidates | 80.07% | 79.44% | 86.45% |

## Safety, calibration, dependence and failures

All nonprivileged arms gave zero false support in 256 absent and separately 256 fully hidden controls, with a 1.478% Wilson upper endpoint per cell. Every nonprivileged arm also issued on 512/512 distractor opportunities and 256/256 of the 4/20 partial-support control. Every arm withheld on all 256 of the 3/12 control. The 512 pooled partial controls had 256 issues from the associated 256 realization groups and are descriptive. A01 issued on all 256 reference positives and all 256 identical-observation matched distractors. Withholding every member of those matched-observation pairs would also cost all 256 reference localizations. The original negative identities and denominators remain intact.

On development the M10 fit used 2,490 RGB candidates and 2,175 LWIR candidates from 128 independent groups. The five-parameter scores converged in seven and six Newton steps, respectively. A01's calibrated raw cutoff was 0.40. B04's cutoff was 0.70. Each had 0/128 absent and 0/128 hidden false support on calibration, with 2.914% upper bounds. B01–B03 retained 0.20, 0.20 and 0.30. Brier scores and ECE are conditional on qualified candidates, including infeasible candidate events. Bin counts and rates are retained. No interval for a physical sensor is implied.

On holdout, both sensors were eligible in 5,376 opportunities. There were 3,955 paired candidates, with 2,892 both correct, 260 RGB-only correct, 35 LWIR-only correct and 768 both wrong. Among both-eligible observations with missing candidates, 256 had only RGB evidence, 141 only LWIR, and 1,024 neither. A01 issued 1,024 RGB and 397 LWIR single-candidate fallbacks. It resolved 259 conflicts to RGB and 22 to LWIR. It warned on 281 of 692 task-degradation opportunities with zero false warnings among the other 2,892 feasible both-eligible opportunities. The 411 misses include a weak or missing contrasting candidate that does not produce a spatial conflict. Conflict warnings are therefore incomplete as a degradation monitor.

The independence-assuming ablation had the same correct and issued counts as A01 at its own calibrated 0.50 cutoff, slightly higher Brier (0.07643), and its own ranking probabilities. Removing learned reliability features recovered the B03 correct count. The evidence supports learned conflict resolution under these controls. It does not isolate a localization benefit from the explicit common-failure correction.

The restricted CPU issuance process took 0.687 s for 6,912 frames and all eight new/ablation rows per frame, including process startup and model/mask I/O. The historical PF1 candidate replay measured 1.75 s for all 13,824 frames and all three baseline rows per frame. These are different workload dimensions, so the measurements do not imply an isolated hardware latency comparison. [Full measured outputs and intervals](evidence/metrics.json) · [Scenario table](evidence/scenario_metrics.csv) · [Sensitivity check](evidence/sensitivity.json)
