# Holdout estimation and interval calibration

The controlled matched-noise arm uses **256 independent holdout realizations**. Means and errors below are in dimensionless effective burden. Empirical coverage is for nominal 95% posterior intervals, using a pointwise Wilson interval for the observed covered fraction. Relative error excludes true burden below 0.05. The prior baseline is uniform on [0,1].

| Mode | Bias | MAE | RMSE | Mean relative error | 95% coverage (Wilson) | Mean interval width |
|---|---:|---:|---:|---:|---:|---:|
| Prior | −0.00808 | 0.24026 | 0.27974 | 86.11% | 96.09% [92.96%, 97.86%] | 0.95000 |
| RGB | −0.00014 | 0.03339 | 0.04461 | 8.36% | 94.92% [91.51%, 97.01%] | 0.16667 |
| LWIR | +0.01281 | 0.12947 | 0.16301 | 44.48% | 96.48% [93.45%, 98.14%] | 0.62770 |
| Joint | +0.00056 | 0.03235 | 0.04297 | 8.11% | 94.53% [91.03%, 96.71%] | 0.16264 |

Joint MAE improved by 0.00104 relative to RGB alone in this matched experiment. Both inform one scalar. A rank increase or universal improvement does not follow. Each bound and prior is part of the declared synthetic model.

The 32 M06 holdout realizations supply 96 timestamped open-target frames and 96 corresponding known blocked frames. RGB mean absolute error was 0.000654 with maximum 0.001636 on the open-target probe. LWIR abstained because the path temperature was unavailable, and joint matched RGB exactly. The RGB coverage figure of 92.71% is descriptive across repeated time samples under a noiseless quantization approximation. One retained grid-residual warning at realization 96, 6 s reached 5.401 nominal sigma. A separate finer numerical diagnostic found a residual of 0.474 LSB. The warning is compatible with the 401-cell grid being coarse relative to quantization. The scored estimator and warning were preserved.

When actual air was 318 K but the likelihood assumed 294 K, LWIR-only MAE grew to 0.41189 and coverage fell to 32.42%. Joint MAE was 0.04752 with 85.16% coverage. It flagged only seven of 256 mismatched samples. Residual warnings alone cannot establish correct nuisance constraints. Severe RGB cases all flagged weak, known occluded or unconstrained cases abstained, and missing/invalid modalities left the valid channel or channels usable. Equal target and air thermal radiance removed LWIR information in the crossover case. See the [full per-arm metrics](evidence/metrics.csv), including both partitions, all failure statuses and prior-only comparisons.
