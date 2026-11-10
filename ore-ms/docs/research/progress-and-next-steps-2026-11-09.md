# RGB–Thermal Visual Odometry: Weekly Progress

**9 November 2026 · focus: error and accuracy**

Can RGB and thermal uncertainty predict relative geometric reliability and improve visual odometry? MACThermal already propagates uncertainty into pose estimation; the target is better cross-sensor accuracy.

Completed work:

- Reproduced RGB/thermal pipelines and captured motion, uncertainty, support and failures. Earlier within-camera tests showed controlled selective-risk median reductions of about 9–37% against matched random under supplied-reference assumptions. Cross-camera transfer remains recording-dependent.
- Completed a three-record day/night/rain comparison, using the first 600 source groups each. Recovered rain inputs and ran both original experts for 600 frames. A separate Gaussian 3D replay/full-covariance variant weakened native poses and did not beat RGB. Kept the native backend.
- Completed the follow-up isolating weights: preserved native means exactly, reconstructed conditional local information matrices, and tested raw/calibrated scalar/full combination with native RGB/thermal, equal/angular and training-best-fixed controls. Each calibration fold used only the other two recordings; predictions were locked before scoring. No runtime benchmarking.

Latest result: **the calibrated full-matrix combination does not improve accuracy.** Mean relative errors use identical intervals for each RGB/combination pair:

| Recording | Intervals | RGB rotation / translation | Calibrated full rotation / translation |
|---|---:|---:|---:|
| Day | 599 | 0.017523 deg / 0.046604 m | 0.022993 deg / 0.048137 m |
| Night | 598 | 0.030350 deg / 0.077280 m | 0.031404 deg / 0.088229 m |
| Rain | 599 | 0.041860 deg / 0.231334 m | 0.043152 deg / 0.231129 m |

On all-method common support (590/551/594 intervals), equal-record mean rotation/translation error increases by 8.95%/3.75% against RGB. Raw weighting remains close to RGB; global scale calibration worsens both aggregate metrics. Training-best-fixed selects RGB in every fold. Coverage is preserved, so this result is not explained by discarded intervals or the prior replay fallback.

39 tests and three independent audits pass. The readout is an inverse conditional information approximation, not a calibrated native pose posterior. These are exposed development prefixes with supplied RGB references and nominal alignment; unseen-record generalization, physical ground truth and full-trajectory accuracy remain unestablished. [Detailed findings and all controls](native-information-weights-findings-2026-11-09.md).

Next: quantify where thermal actually beats RGB, including joint gains, tradeoffs and contiguous opportunities. Use an oracle bound only to measure available headroom, then test whether existing geometry/support/residual cues predict those opportunities across recordings. This will distinguish weak reliability cues from weak expert estimates and guide targeted thermal frontend, covariance or shared-representation changes. Retain native poses initially and assess full trajectories after a credible interval gain. The tested global-weight model has not helped; broader RGB–thermal research remains open.
