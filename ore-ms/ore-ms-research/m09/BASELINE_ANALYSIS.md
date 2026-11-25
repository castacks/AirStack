# Matched horizon, data efficiency and calibration results

The 128 test IDs are the exact original M08 holdout, with fresh and disjoint 768/256 training/validation realizations. Each neural result below is the mean of three frozen seeds. Lower CRPS is better. The aggregate averages **2/6/18/48 s** per realization first and excludes the current-state diagnostic at 0 s.

| Inputs, full training size | Data | Ablation | Physics | M08 same mode |
|---|---:|---:|---:|---:|
| RGB | 0.098901 | 0.098818 | 0.098681 | 0.098669 |
| LWIR | 0.103456 | 0.103514 | 0.103236 | 0.102854 |
| Joint | 0.099062 | 0.098892 | 0.098800 | 0.098667 |

Joint Physics minus M08 was +0.000133 with pointwise paired 95% interval [−0.000722, +0.000988]. **There is no supported learned gain over matched M08.** The other learned references are retained rather than selecting the best test checkpoint. M08 climatology, persistence, unconditioned physics and privileged oracle are also scored on the same IDs. At 2 s their burden CRPS values were 0.11455, 0.10234, 0.11454 and 0.07858, respectively. All original classical coverage, CRPS and Brier results were reproduced to within 2.78e−17.

| Joint burden CRPS | Issue (0 s) | 2 s | 6 s | 18 s | 48 s |
|---|---:|---:|---:|---:|---:|
| Physics, 768 | 0.02579 | 0.08040 | 0.10200 | 0.09743 | 0.11537 |
| Ablation, 768 | 0.02677 | 0.08051 | 0.10203 | 0.09712 | 0.11590 |
| Data, 768 | 0.02577 | 0.08096 | 0.10242 | 0.09713 | 0.11574 |
| M08 joint | 0.02498 | 0.08063 | 0.10243 | 0.09784 | 0.11377 |

## Physics loss and data size

Physics minus its identical-architecture ablation at 128 training realizations was **−0.001569** [−0.002277, −0.000861] for joint future burden. All three seed means had the same sign, with range −0.002023 to −0.001138. At 768 the difference was −0.000092 [−0.000493, +0.000310], with seed means −0.000401 to +0.000063. This supports a limited low-data benefit from the added teacher loss in this frozen comparison. Increasing cohort size reduced ablation CRPS by 0.001410 on average, with CI [−0.003184,+0.000364]. The Physics large-minus-small difference was +0.000067 [−0.001259,+0.001392]. The tested size comparison alone does not establish a generic data-efficiency curve. Intervals are pointwise and do not adjust for the full comparison matrix.

## Calibration and event probability

Joint Physics 768 matched coverage across 0/2/6/18/48 s was 92.71%, 93.49%, 97.14%, 97.92%, 95.57%. Its 2 s burden MAE/RMSE were 0.11563/0.14453 with bias +0.00019, mean spread 0.14105 and interval width 0.52341. At 48 s width reached 0.71055. These aggregate seed means describe empirical calibration under this model and dataset. The per-seed tables retain Wilson intervals.

At 2 s, RGB transmission below 0.55 occurred 82 times of 128. Its mean predicted probability was 0.64648, Brier 0.15756 and AUC 0.81420. LWIR below 0.88 occurred 78 times, with probability 0.59989, Brier 0.15614 and AUC 0.82632. The event tables report counts, probabilities and Wilson intervals within fixed bins. Seed-repeated forecasts are not recounted as independent observations.

[Horizon scores](evidence/scores.csv) · [Paired realization intervals](evidence/paired.csv) · [Reliability bins](evidence/reliability.csv) · [Training curves and forecast plot](evidence/learning_curves.png)
