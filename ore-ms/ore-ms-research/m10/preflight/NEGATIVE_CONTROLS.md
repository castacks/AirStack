# Negative controls and retained failures

| Control | Measured result | Disposition |
|---|---|---|
| High foreground atmospheric transmission, full geometric occlusion | 512 cases with zero first-hit target pixels and zero baseline candidates in all arms. Foreground RGB transmission at 550 nm was at least 0.502 along the 5 m axial reference path. | PASS |
| Thermal crossover | On holdout B02 had zero candidates while B01 correctly localized 256/256. | PASS |
| Missing both modalities | Zero candidates in all arms across 512 opportunities. | PASS |
| Unknown range | Blank range in all 512 controls. Opaque-ID-adjusted outputs exactly matched reference. | PASS |
| Distractor identity | Independent geometry verifies first hit is distractor and target feasibility is false. Each arm issued on 256/256 matched distractors and 256/256 contrast-without-target opportunities on holdout. | Label PASS. Baseline failure retained. |
| Mutation of privileged labels | Standalone candidate bytes identical after injecting modified truth. Seven prohibited columns rejected. | PASS |
| Blind future shift | All 512 current detector and admissible metadata pairs and their predictions matched, with a changed future burden in all pairs. | PASS |
| Registration and clock qualification | Known spatial offset and stale samples caused LWIR exclusion and RGB fallback. The unannounced offset remained shifted by three pixels and caused B03 conflict. A 2001 ms injected measurement was excluded. | PASS |

At 4/20 support every arm issued falsely on all 256 holdout cases. The 3/12 control had zero issuance, consistent with the four-pixel component minimum. A four-pixel image signature cannot establish what fraction of the target is visible. The matched distractor also has exactly the same observations as its paired reference target. This observational equivalence prevents these detectors from certifying target identity. The benchmark retains both outcomes and does not support an operating claim in those strata. [Independent controls](evidence/independent_verification.json) · [Holdout results](evidence/holdout.json)
