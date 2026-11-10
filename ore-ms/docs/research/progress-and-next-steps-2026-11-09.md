# RGB–Thermal Visual Odometry: Weekly Progress

**9 November 2026 · focus: error and accuracy**

## Main question

Can RGB and thermal uncertainty predict relative geometric reliability and improve visual odometry? MACThermal already propagates feature uncertainty into covariance-weighted pose estimation. Propagation alone is not a new contribution; improved cross-sensor accuracy is the target.

## Completed work

- Reproduced RGB/thermal pipelines and recorded uncertainty, motion, support and failures. Existing within-camera tests show controlled selective-risk median reductions of about9–37%against matched random under supplied-reference assumptions.
- Tested scalar cross-camera rules and recording transfer. Results depend strongly on recording; the previous whole-record angular-rule test stopped for insufficient joint evaluation coverage and showed no matched benefit over RGB. This does not rule out stronger uncertainty models or fusion.
- Implemented a separate Gaussian3D replay backend, local pose covariance propagation and equal/scalar/full covariance combination controls. Preserved unchanged native MAC/MACT estimates as baselines.
- Completed the new three-record development comparison: first 600 groups each of day, night and rain. Reused cached day/night outputs; recovered the continuous rain input and ran both original experts for 600 frames each. Locked all ten methods before reference scoring. No training or runtime benchmarking.

## Latest accuracy result

**The new replay/full-covariance variant does not beat native RGB.** Mean relative errors below use identical intervals for each pair of methods:

| Recording | Intervals | Native RGB rotation /translation | Replay +full covariance rotation /translation |
|---|---:|---:|---:|
| Day |599|0.017523deg /0.046604m|0.038433deg /0.057849m|
| Night |598|0.030350deg /0.077280m|0.360826deg /0.351439m|
| Rain |599|0.041860deg /0.231334m|0.092440deg /0.320584m|

Full covariance improves over equal fusion of the replayed poses, but the replay backend itself weakens the estimates. On common support, its single-camera RGB and thermal results are worse than their native counterparts. Equal fallback when a replay covariance is missing adds substantial night error. These effects are separated in the [complete findings](pose-covariance-accuracy-findings-2026-11-09.md); an improvement over the weakened replay baseline is not reported as a system gain.

31 numerical/scoring tests and independent combination/error reconstruction pass. Missing estimates, capped solves, fallback decisions and source denominators remain explicit. Scores use provided RGB reference products and the existing nominal frame/time model; physical ground truth and new-record generalization are not established. Per-interval errors are not full-trajectory accuracy.

## Next technical step

1. **Retain the native pose backend.** The replay variant is not an accuracy improvement and should not replace it.
2. **Change weighting independently of pose estimation.** Test a native-compatible uncertainty/readout or calibration model with fixed native means, strong RGB/thermal/fixed-fusion controls and an explicit missing-covariance fallback. Freeze that new comparison before scoring.
3. **Choose later improvements from the observed limitation.** Native thermal frontend/normalization or covariance learning, shared representations and IMU fusion remain credible routes. Each should address a named error mechanism rather than changing all components simultaneously.

The weekly brief records progress; it is not the endpoint of the research. The latest negative result applies to the tested backend/fusion variant, not to all RGB–thermal fusion.
