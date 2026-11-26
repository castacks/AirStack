# Adaptive fusion generalization

The separate benchmark has 5,184 frame opportunities from 48 independently seeded scene groups. Each group crosses 2/4/6 px targets, nine offsets, and clear/edge/partial/distractor controls. Axis offsets are `(d,0)`. Diagonal offsets `(d,d)` have magnitude `sqrt(2)*d`. The 12-bit codes represent synthetic contrast in both modalities. The raster independently generates current pixel inputs from silhouettes and foreground geometry. Frozen PF1 frontends, M10 feature parser, model weights, association logic, calibration-bin probabilities and cutoffs consumed those inputs through their existing interfaces. Scene labels and target truth stayed outside the policy inputs.

| Arm | Correct / 2,448 feasible | Issued / 5,184 | Issued on 2,736 negatives | Selective risk | Candidate Brier |
|---|---:|---:|---:|---:|---:|
| B01 RGB | 2,403 | 4,176 | 1,728 | 42.46% | 0.376 |
| B02 LWIR | 368 | 4,304 | 1,856 | 91.45% | 0.250 |
| B03 equal | 661 | 1,992 | 896 | 66.82% | 0.597 |
| B04 constant | 760 | 1,992 | 896 | 61.85% | 0.461 |
| A01 adaptive | 647 | 1,181 | 397 | 45.22% | 0.407 |

The paired A01−B03 localization gain was −0.57 percentage points (95% scene-cluster bootstrap −3.08 to +1.62). A01 still recovered 41/144 clear cases at +3 px horizontally where B03 withheld all 144. On the associated zero-offset clear stratum A01 returned 48/144 versus 144/144 for B03. Across the sweep 2,103 A01 forecasts were withheld for an unqualified score bin. A01 issued on 48/144 zero-offset distractors and zero zero-offset partial cases. It still issued on 397 negative scenes overall. Brier scores cover only each arm's candidates with an available frozen probability, so denominators differ. Raw reason counts, bins and denominators are retained in the evidence.

Across 4,176 paired-candidate opportunities, changing the diagnostic candidate-distance threshold from 1 to 2 to 3 px would flag 3,240, 2,312 and 1,384 conflicts. Only the original threshold was used in decisions. The results reproduce the large-offset selective recovery mechanism and expose its dependence on association and calibration transfer. They do not reproduce the aggregate PF1 gain. The M11 fixture distribution is separate from historical holdout and no threshold was fit on either. [Scores and strata](evidence/fusion_checks.json) · [Replay and raw decision checksum](REPRODUCTION.md)
