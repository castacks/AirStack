# RGB–Thermal Visual Odometry: Weekly Progress

**9 November 2026 · error and accuracy**

The goal is better RGB–thermal visual odometry through useful geometric reliability. MACThermal already propagates uncertainty into pose estimation; propagation itself is not a new contribution.

Completed work:

- Reproduced native pipelines and captured motion, uncertainty, support and failures. Earlier within-camera tests showed controlled selective-risk median reductions around 9–37% against matched random under supplied-reference assumptions; cross-camera transfer remains inconsistent.
- Compared day/night/rain prefixes of 600 source groups. A Gaussian replay backend weakened native poses. Fixed-native information calibration then increased equal-record mean rotation/translation error by 8.95%/3.75% against RGB.
- Isolated translation fusion while copying RGB rotation exactly. The primary learned blend improves day but worsens night/rain; aggregate mean translation error rises 0.46%. Availability stays unchanged. Reference-dependent optimal convex blending offers 19.47% translation headroom, which the learned model does not recover.
- Reviewed repetition after the user's concern. Residual/conditioning cues, factor geometry, bias/scale calibration, covariance learning and full-matrix weighting all have substantial prior coverage. The recent component tests are useful ablations, not new research directions. Deferred another directional/confidence variant.

**Latest diagnostic:** thermal translation error is predominantly along the reference motion direction and its estimated motion magnitude is short:

| Recording | Thermal motion-axis share of translation MSE | Median thermal/reference interval motion norm |
|---|---:|---:|
| Day | 96.1% | 0.673 |
| Night | 89.0% | 0.444 |
| Rain | 97.9% | 0.450 |

These are paired-support diagnostics, not accumulated travel or proof of depth miscalibration. RGB/thermal mean errors oppose in day/night but align in rain; centered rain errors correlate positively. Oracle headroom therefore does not imply unbiased independent sensor estimates. Even a privileged rain constant weight optimized for MSE worsens mean translation error, confirming the loss tradeoff.

**Next experiment:** a matched 2×2 depth-mean/covariance ablation in the native thermal motion problem: original versus reference-assisted stereo/depth means, each with native versus fixed covariance. Earlier work varied weights at fixed depths or calibrated risk scores without changing means. This control tests the motion-level consequence of changing means before investing in another reliability model or frontend training.

The assisted-depth arm is diagnostic, not deployable. Exact input/support qualification and unchanged-replay parity come first; the four-arm experiment has not run. A positive result would justify a targeted stereo-mean/frontend intervention. A negative result would redirect attention to correspondences, optimization or reference assumptions. [Review and evidence](fusion-mechanism-review-2026-11-09.md), [next experiment](next-experiment-depth-mean-ablation-2026-11-09.md).

Four new decomposition tests and 60 independent scalar checks pass; prior model audits remain intact. No new inference or runtime benchmarking. All findings use exposed development data, supplied references and nominal alignment; physical ground truth, unseen-record transfer and full-trajectory improvement remain unestablished. RGB remains the executable accuracy baseline.
