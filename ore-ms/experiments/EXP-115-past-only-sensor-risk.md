# EXP115: original-rig timing and past-only sensor-risk probe

Client date2026-11-03. Frozen8b4daf442; [full findings](../docs/research/past-only-sensor-risk-findings-2026-11-03.md).

Seven exposed recordings/eighteen windows; 1955causalforecasts,28recording-excluded analytic fits, no native model training or reserve use. Primary0.5s criterion FAIL: history changes4/1072choices, worsens original-rig macroT MSE0.02716774→0.02726328; MS²tiny0.02704044→0.02704037. TwoTD3night records only11/10joint targets remain a sample-size limitation. 1s sensitivity meets fixed numeric inequality with tiny changes; not promoted.

Original-rig dt² persistence:14/18unchanged,2improve,2worsen,median0.4623unchanged.24source stream/window checks (1120bindings) match native timestamps exactly;12day1checks lack original streams. Physical exposure timing remains unverified. Existing prefixmedian1.355sTD3/0.960sMS²leaves0/1072positive0.5sbudgets; this is instrumented offline timing. Current/history select107unavailable sensors versus18for origin information control. No actual skipping or memory architecture adopted.

| Held-out recording | Joint target comparisons | Latest-state T RMSE (m) | With history T RMSE (m) |
|---|---:|---:|---:|
| ms2/_2021-08-06-11-23-45 | 264 | 0.10705815 | 0.10705815 |
| ms2/_2021-08-06-16-19-00 | 266 | 0.25924973 | 0.25924974 |
| ms2/_2021-08-13-21-18-04 | 250 | 0.04949179 | 0.04948989 |
| td3/20251027/0_salon_so_day | 88 | 0.04237731 | 0.04666891 |
| td3/20251027/1_salon_mac_cov_day | 30 | 0.04966940 | 0.04966940 |
| td3/20251027/night_mac_salon_warehousefig8 | 11 | 0.30264847 | 0.30264847 |
| td3/20251027/night_thermal_so_salon2 | 10 | 0.11318996 | 0.11318996 |

Sixfocusedtests,81112independent fit/causality/scorechecks,1708timing/rankchecks;2538input/sourcepins and3label hashes unchanged. Next measure earlier health/covariance availability and test future availability before quality complexity. Backup backups/2026-11-03-past-only-sensor-risk; native/model/runtime prerequisites EXP112–114.
