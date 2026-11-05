# EXP147: native depth against supplied single-scan LiDAR projection

Freeze568b46473; parent 9fb25b7b4.

2026-11-05 — [EXP147 native depth / single-scan LiDAR proxy](../docs/research/native-depth-proxy-findings-2026-11-05.md): freeze 568b46473; same fixed 20sample frames per camera, no new native inference. Median of per-image median native/reference depth ratios RGB1.003929, thermal 0.826784 (17/20 thermal below 1); absolute relative errors 10.54%/26.59%. Compared only 309RGB/1,089thermal features,3–39/26–90 perimage; no clipping/interpolation/fit. Forty maps acquired via 48HTTP206ranges/192MiB,2archived overlaps exact. Independent sensing modality, but projection/visibility/physical timing qualification unresolved: conditional proxy agreement, not GT or causal VO-gap explanation.6 tests,406,384 checks/4corruptions pass. All cases/unsupported features retained; no reserve/default/EXP133 promotion. Next projection/visibility sensitivity and acquisition/export timing evidence. Backup backups/2026-11-05-native-depth-proxy-exp147.

