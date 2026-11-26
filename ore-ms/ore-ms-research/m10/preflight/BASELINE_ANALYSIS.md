# Locked holdout baseline analysis

The fixed equal-contribution comparator selected on development localized 4,327 of 5,120 feasible opportunities correctly. All 6,912 holdout opportunities per arm enter the overall issuance denominator, including 1,792 infeasible opportunities.

| Arm | Correct / feasible | Grouped 95% CI | Issued / abstained | Coverage | Selective risk | Feasible issued mean error | Candidate Brier |
|---|---:|---:|---:|---:|---:|---:|---:|
| B01 RGB | 4,176/5,120 (81.56%) | 81.29–81.86% | 4,979 / 1,933 | 72.03% | 16.13% | 0.02076 px | 0.12932 |
| B02 LWIR | 3,324/5,120 (64.92%) | 64.84–64.98% | 4,352 / 2,560 | 62.96% | 23.62% | 0.21764 px | 0.17544 |
| B03 equal fusion | 4,327/5,120 (84.51%) | 84.32–84.69% | 5,095 / 1,817 | 73.71% | 15.07% | 0.000085 px | 0.12671 |

Against B03, the paired B01 difference was −2.95 percentage points (95% CI −3.24 to −2.66), and the B02 difference was −19.59 points (−19.77 to −19.41). The 10,000 draws use seed 10091 and resample the same 256 realization IDs with replacement for every arm, keeping all associated controls together. Per-scenario intervals in the output are pointwise. Infeasible-only strata have no correct-per-feasible interval.

All arms saw the same 6,144 RGB-available and 5,632 LWIR-available opportunities. B03 produced 1,024 RGB and 397 LWIR fallbacks (20.56% of all opportunities combined), and 281 candidate conflicts. All 1,421 fallback candidates were issued at the frozen cutoff. For 3,584 feasible cases with both sensors available, the RGB/LWIR error-dependence table contains 2,892 both correct, 516 RGB only correct, 176 LWIR only correct and zero both wrong (candidate-level phi −0.0932). The output includes conditional tables by intervention.

| Negative label | False-supported in each arm | Wilson upper 95% |
|---|---:|---:|
| Absent | 0/256 | 1.478% |
| Fully hidden | 0/256 | 1.478% |
| Empty projected support | 0/256 | 1.478% |
| Insufficient partial | 256/512 | 54.315% descriptive pooled |
| Distractor | 512/512 | 100% |

The 4/20 partial fixture and both distractor cells produced all of the false-supported cases. The 3/12 partial fixture produced none. Every arm produced no candidate in the clipped cell. B02 localized none of the hidden registration-shift cases within 1 px, while B03 withheld all those conflicts. These results are conditional on the frozen synthetic fixture population. [Overall and scenario metrics, reliability bins and confidence intervals](evidence/holdout.json)
