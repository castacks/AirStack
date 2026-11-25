# Missing observations, misspecification and OOD

The issue CLI rejects disallowed columns. Rails and invalid measurements are masked. Dropping or invalidating LWIR returns the exact RGB-only output. Fixed metadata identifies known occlusion and flat thermal probes, which use prior fallback. Fully missing and clipped cases also return the explicit prior fallback. Unknown range/material/domain/process arms hard abstain. Unconstrained thermal input drops LWIR. The matched and blind future-dynamics cases have identical issue-time codes and forecasts. The novelty score cannot separate them and its AUROC is 0.5 across all seeds.

| Joint Physics 768 | Matched | Shifted |
|---|---:|---:|
| Actual dynamics shifted to 2 s/1.6, 2 s burden coverage | 93.49% | 66.41% |
| Actual dynamics shifted, 2 s burden CRPS | 0.08040 | 0.16192 |
| Wrong issue air (318 K vs 294 K), issue coverage | 92.71% | 76.30% |
| Wrong issue air, issue CRPS | 0.02579 | 0.03992 |

Under wrong air the large-cohort joint novelty AUROC was 0.92987. At the frozen 99th percentile training cutoff it flagged 51.56% of shifted issues and 0.78% of matched issues. LWIR-only AUROC was 0.96390, and RGB-only was 0.5 because the RGB codes were unchanged. This diagnostic does not warrant assurance under misspecification. Deliberate dynamics mismatch combines changed time constant and process amplitude. The unchanged matched M08 slow and low-amplitude parameter sensitivity arms are also retained in the scoring output. No OOD threshold or model was adjusted on test data. See [all diagnostics](evidence/ood.csv) and [all score controls](evidence/scores.csv).
