# RGB–Thermal Visual Odometry: Weekly Progress

**9 November 2026 · error and accuracy**

The research question is whether RGB/thermal uncertainty can predict relative geometric reliability and improve visual odometry. MACThermal already propagates uncertainty into pose estimation; improved cross-sensor accuracy is the target.

Completed work:

- Reproduced native RGB/thermal pipelines and captured motion, uncertainty, support and failures. Earlier within-camera tests showed controlled selective-risk median reductions around 9–37% against matched random under supplied-reference assumptions; cross-camera transfer remains recording-dependent.
- Compared day/night/rain prefixes of 600 source groups each. A separate Gaussian 3D replay backend weakened native estimates, so retained the native backend. A follow-up preserved native means exactly and tested raw/calibrated information-based weighting. The primary calibration increased equal-record mean rotation/translation error by 8.95%/3.75% against RGB on shared support; coverage was preserved.
- Completed the next experiment: measured native thermal accuracy opportunities and tested one frozen recording-excluded cue model. No new inference or runtime benchmarking.

**The latest finding is a rotation–translation tradeoff.** Thermal beats RGB on translation in 361 of 1,735 paired intervals, on rotation in 103, and on both in only 21. Rain includes a 43-interval translation-win run spanning 4.33 seconds; rotation wins are brief and sparse.

Equal-record mean relative errors on paired support (day 590 / night 551 / rain 594 intervals):

| Method or diagnostic | Rotation (deg) | Translation (m) |
|---|---:|---:|
| Native RGB | 0.029798 | 0.117703 |
| Perfect rotation-based whole-pose selection | 0.028797 | 0.129877 |
| Perfect translation-based whole-pose selection | 0.036161 | 0.105974 |
| Perfect selection requiring neither metric to worsen | 0.029611 | 0.116351 |
| Reference-free cue model | 0.030879 | 0.117730 |

The oracles use reference errors and are diagnostic bounds on complete-pose selection, not deployable results. Translation-based selection has about 10% translation headroom but raises rotation error about 21%. The cue model used support/residual/geometry/disagreement features, trained on the other two recordings with no parameter search. It increased rotation error 3.63%; retain RGB as the baseline. All three folds chose RGB as training-best-fixed.

Seven new tests and an independent audit pass, checking 7,062 native error components, 1,797 selections, all three fits and 285 winning runs. Previous 39-test weighting verification remains intact. Native availability is preserved, including the one unavailable night interval. [Detailed findings](thermal-opportunities-findings-2026-11-09.md).

**Next:** test translation-specific prediction/fusion while keeping native RGB orientation. Target the observed translation opportunities, then evaluate trajectories and new recordings if an interval gain appears. Thermal depth/frontend and shared-representation improvements remain options when tied to the identified errors.

All results use exposed development prefixes, provided RGB references and nominal alignment. Physical ground truth, unseen-record transfer and full-trajectory accuracy remain unestablished. These outcomes narrow the next experiment; they do not close broader RGB–thermal research.
