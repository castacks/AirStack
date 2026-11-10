# RGB–Thermal Visual Odometry: Weekly Progress

**9 November 2026 · error and accuracy**

The research question is whether RGB/thermal uncertainty can predict relative geometric reliability and improve visual odometry. MACThermal already propagates uncertainty into pose estimation; improved cross-sensor accuracy is the target.

Completed work:

- Reproduced native pipelines and captured motion, uncertainty, support and failures. Earlier within-camera tests showed controlled selective-risk median reductions around 9–37% against matched random under supplied-reference assumptions; cross-camera transfer remains recording-dependent.
- Compared day/night/rain prefixes of 600 source groups each. A Gaussian 3D replay backend weakened native estimates, so retained native poses. Fixed-native information calibration increased equal-record mean rotation/translation error by 8.95%/3.75% against RGB.
- Measured thermal opportunities: 361 translation wins, 103 rotation wins and 21 joint wins among 1,735 paired intervals. Perfect translation-based whole-pose selection offered about 10% translation gain but raised rotation error about 21%.
- Completed the next test: translation-only selection/fusion with native RGB orientation copied exactly. No new inference or runtime benchmarking.

**Latest method result:** the primary learned translation blend improves day, but worsens night and rain. Mean translation errors on identical RGB/primary intervals:

| Recording | Intervals | RGB (m) | Training constant blend (m) | Learned blend (m) |
|---|---:|---:|---:|---:|
| Day | 599 | 0.046604 | 0.043912 | 0.043270 |
| Night | 598 | 0.077280 | 0.079463 | 0.079815 |
| Rain | 599 | 0.231334 | 0.232760 | 0.233570 |

On all-method paired support (590/551/594 intervals), equal-record mean translation error rises **0.46%** versus RGB. RGB rotation and availability are unchanged. Training-best-fixed selects RGB in every fold; the learned blend also trails the constant control overall. No adaptive method is adopted.

**Useful new finding:** perfect reference-based convex translation blending offers **19.47%** mean translation reduction with RGB rotation preserved. This oracle is a diagnostic bound, not an executable result. It shows fusion opportunity beyond choosing whole poses. The learned model assigns some thermal weight everywhere, while the oracle rejects thermal entirely on over half of rain's paired intervals.

Seven tests and an independent audit pass: three reconstructed fits, 10,410 paired component geometries, 28,614 error components and 1,735 oracle optima. Models used only the other two recordings and were locked before scoring. [Detailed findings and controls](translation-fusion-findings-2026-11-09.md).

**Next:** test directional geometry and residual consistency to distinguish helpful from harmful translation corrections. Covariance projected along RGB–thermal disagreement is a concrete candidate; existing trace/norm summaries discard directional information. Keep current weights frozen. If an interval gain transfers, evaluate trajectories and new recordings; otherwise target a named thermal depth/frontend or shared-representation mechanism.

All results use exposed development prefixes, supplied RGB references and nominal alignment. Physical ground truth, unseen-record transfer and full-trajectory improvement remain unestablished. The tested models have not provided a cross-record gain; broader fusion research remains open.
