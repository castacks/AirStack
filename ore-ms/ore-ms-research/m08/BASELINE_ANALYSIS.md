# Holdout forecast calibration and comparisons

Each horizon uses **128 independent holdout realizations**, with the same realizations paired across methods. The aggregate paired comparison averages each realization's horizons before estimating uncertainty. Lower CRPS and Brier are better. The zero-second row checks issue uncertainty. Forecasting skill is assessed separately at nonzero horizons.

| Horizon (s) | Joint burden CRPS | RGB-only CRPS | LWIR-only CRPS | Climatology CRPS | Persistence CRPS | Oracle CRPS | Joint 95% coverage | Joint width |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 0.02498 | 0.02543 | 0.08065 | 0.11796 | 0.02498 | 0 | 93.75% | 0.1581 |
| 2 | 0.08063 | 0.08067 | 0.09879 | 0.11455 | 0.10234 | 0.07858 | 93.75% | 0.5296 |
| 6 | 0.10243 | 0.10250 | 0.10120 | 0.10552 | 0.16298 | 0.09772 | 96.88% | 0.6848 |
| 18 | 0.09784 | 0.09763 | 0.09745 | 0.09742 | 0.20027 | 0.09860 | 96.88% | 0.7157 |
| 48 | 0.11377 | 0.11387 | 0.11397 | 0.11482 | 0.20244 | 0.11419 | 95.31% | 0.7081 |

At 2 s the paired joint-minus-climatology difference was −0.03391, with approximate 95% interval [−0.04821, −0.01962]. The comparable 6 s interval [−0.00919, 0.00301] included zero. Across all horizons the realization-mean paired joint-minus-RGB difference was −0.000090 [−0.000359, 0.000179]. Joint conditioning supplies no clear advantage over RGB in this fixture. LWIR-only had slightly lower CRPS at 6 s. The unconditioned physics reference was close to climatology on every horizon, as expected under the stationary law. Persistence's interval held width 0.1581 as process uncertainty developed, and only 27.34% of 48 s labels fell inside it.

The matched joint nominal 95% coverage Wilson intervals were [88.15%, 96.80%] at 2 s and [90.15%, 97.83%] at 48 s. Its 2 s burden MAE and RMSE were 0.11541 and 0.14474. Mean spread grew from 0.04143 at issue to 0.19768 at 48 s. The 2 s burden bin log score was 2.4721. The nonmonotonic 18/48 s interval widths reflect finite banks and phase relative to the path. They should not be read as a calibrated monotonic-width guarantee.

At 2 s, the joint RGB threshold recorded 82 events of 128 with mean forecast probability 0.65875, Brier 0.15808 and AUC 0.81575. LWIR recorded 78 events, mean probability 0.61267, Brier 0.15458 and AUC 0.82910. At 48 s the Brier scores were 0.23107 and 0.24584, with AUC 0.51458 and 0.54337. The [reliability table](evidence/reliability.csv) retains five frozen bins, event counts and bin-wise Wilson intervals, including empty bins. The [score table](evidence/scores.csv) gives bias, spread, MAE, RMSE, intervals, widths, CRPS, categorical log scores, Brier and event diagnostics. The [paired contrasts](evidence/paired.csv) retain grouped uncertainty.

## Failures and limits

With actual time constant 2 s and stationary process scale 1.6 against the assumed 6 s/1.0 process, joint coverage fell to 69.53% at 2 s and 77.34% at 48 s. Assuming lower innovation amplitude on otherwise matched truth gave 73.44% at 48 s. The slow assumed process gave 89.06% at 2 s. These controls were fixed before evaluation. No model-discrepancy correction was fitted.

When actual issue air was 318 K against 294 K in the likelihood, joint issue coverage fell to 85.16% and LWIR-only issue coverage to 35.16%. Only 7 of 128 wrong-air joint issues fell back under the residual screen. A small residual does not establish correct nuisance assumptions. Missing, clipped, occluded and flat LWIR probes yielded labeled prior fallbacks. Unknown range/material/domain and unqualified dynamics produced no distribution. These flags do not predict future occlusion or saturation.
