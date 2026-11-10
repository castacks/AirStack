# RGB–Thermal Visual Odometry: Weekly Progress

**9 November 2026 · error and accuracy**

The goal is improved RGB–thermal visual odometry. MACThermal already propagates uncertainty into pose estimation; propagation itself is not a new contribution.

Completed method work:

- Reproduced native pipelines and captured motion, uncertainty, support and failures. Earlier within-camera selective-risk tests showed controlled median reductions around 9–37% against matched random under supplied-reference assumptions; cross-camera transfer remains inconsistent.
- Compared day/night/rain prefixes of 600 source groups. A Gaussian replay backend weakened native poses. Fixed-native information calibration increased aggregate mean rotation/translation error by 8.95%/3.75% against RGB.
- Translation-only fusion preserves RGB rotation and coverage, but its learned blend improves day and worsens night/rain: aggregate mean translation error increases 0.46%. Reference-dependent optimal convex blending offers 19.47% translation headroom; the learned model does not recover it.
- Reviewed repetition. Residual/conditioning rules, geometry diagnostics, bias/scale calibration and covariance adaptation have substantial prior coverage. Deferred another confidence variant. Thermal motion error lies predominantly along the motion direction, without establishing its physical cause.

**Latest progress:** completed 120 unchanged native pair solves with exact saved-parameter parity. Continued the depth-mean/covariance ablation preflight by finding an older exact-record rain backup and recovering 40 single-scan projected depth maps. All 80 source-image hashes match the current capture.

The recovered maps revealed a support limitation:

| Rain camera | Pairs with both endpoint maps | Native factors | Factors with depth at both native endpoints | Eligible pose problems |
|---|---:|---:|---:|---:|
| RGB |19|3,736|0|0|
| Thermal |19|1,187|1|0|

At least three suitable correspondences and full geometric rank are required. Two additional pair slots remain missing frame 20 labels. Three sampling tests and an independent 54-hash/40-map/80-image/9,846-endpoint audit pass.

**The four-arm experiment remains unexecuted.** Restoring these exact maps was useful, but they cannot support the declared native-correspondence intervention. Park that assay for this packet; further downloads, hole filling or reduced qualification would add work without preserving the test. The earlier depth-mean-versus-weight question remains unanswered. [Recovery and support findings](depth-label-retrieval-findings-2026-11-09.md).

Next substantial method work should change the estimated means/frontend under a feasible sparse-supervision and VO-evaluation contract, rather than repeat confidence weighting or disparity-error surveys. Sparse valid depth pixels can supervise stereo means without requiring both native track endpoints to have LiDAR labels. Matched native controls and recording separation are necessary; no such training run or accuracy gain is claimed yet.

All results use exposed development data, supplied references and nominal alignment. Physical ground truth, unseen-record transfer and full-trajectory improvement remain unestablished. No new frontend inference or runtime benchmarking in this continuation; RGB remains the executable accuracy baseline.
