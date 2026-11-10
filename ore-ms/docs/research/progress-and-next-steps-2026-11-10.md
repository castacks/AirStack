# RGB–Thermal Visual Odometry: Weekly Progress

**10 November 2026 · error and accuracy**

The objective is better RGB–thermal visual odometry. Earlier uncertainty calibration, sensor selection and translation-fusion experiments did not consistently beat RGB. A reference-assisted depth ablation was parked because recovered sparse maps lacked joint support at native track endpoints.

We completed two mean-changing fits. The first updated the decoder's shared horizontal head, but training loss worsened and rain accuracy did not improve. Numerical probes showed a mismatch between its truncated recurrent gradient and recomputed forwards. A separate final-output residual then passed gradient and descent checks.

**Latest: the verified residual completed a fixed 40-update fit and accuracy evaluation.** It starts at zero, adds 2,305 trainable scalars after the frozen decoder and uses the same 20 day + 20 night training frames. Training loss falls **1.71775 → 1.41456 (−17.65%)**. The final checkpoint was locked before any rain scoring, with no checkpoint or budget search.

On the same 20-frame exposed rain recording:

| Accuracy metric | Original thermal | Fitted residual | RGB |
|---|---:|---:|---:|
| Stereo signed-flow MAE, px | 2.1903 | 2.1343 | — |
| Stereo depth AbsRel | 0.1792 | 0.2069 | — |
| Stereo mean per-frame depth RMSE, m | 6.9581 | 12.8433 | — |
| VO translation mean, m | 0.9600 | 0.9308 | 0.6700 |
| VO translation RMSE, m | 1.0490 | 1.0305 | 0.8674 |
| VO rotation mean, deg | 0.07527 | 0.06763 | 0.03128 |

VO scores use 18 identical eligible motion intervals. Translation/rotation means improve **3.04%/10.15%** over thermal. Depth accuracy worsens, translation p95 rises 0.64%, and RGB remains better. Coverage is unchanged. All four native arms complete 20 frames; the zero-residual control reproduces all original pose/time arrays exactly.

**Decision:** the fit learns, but misses the predeclared accuracy rule. Close this adapter recipe without further tuning; retain the original models. This bounded negative/mixed result does not close the broader study.

Nine tests and an amended independent audit pass: 358 frozen hashes, 40 final training predictions, 200 stereo metrics and 73 mapped motions. Auditor-only variable/call-count fixes and a precision-aware residual check are documented. An independent FP64 residual confirms the depth degradation. [Detailed findings](final-residual-fit-findings-2026-11-10.md).

**Next:** choose one materially different correspondence/representation intervention, with a direct correspondence measurement and downstream motion accuracy endpoint. Review prior experiments before implementation. No further training-budget or rain-checkpoint search for this recipe.

These remain previously exposed development prefixes with supplied references and nominal alignment. No fresh-record/generalization, full-trajectory or runtime claim is made.
