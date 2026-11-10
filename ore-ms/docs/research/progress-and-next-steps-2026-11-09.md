# RGB–Thermal Visual Odometry: Weekly Progress

**9 November 2026 · error and accuracy**

The goal is better RGB–thermal visual odometry through useful geometric reliability. MACThermal already propagates uncertainty into pose estimation; propagation itself is not a new contribution.

Completed accuracy work:

- Reproduced native pipelines and captured motion, uncertainty, support and failures. Earlier within-camera tests showed controlled selective-risk median reductions around 9–37% against matched random under supplied-reference assumptions; cross-camera transfer remains inconsistent.
- Compared day/night/rain prefixes of 600 source groups. A Gaussian replay backend weakened native poses. Fixed-native information calibration increased equal-record mean rotation/translation error by 8.95%/3.75% against RGB.
- Isolated translation fusion with exact RGB orientation. The learned blend improves day but worsens night/rain; aggregate mean translation error rises 0.46%, with availability unchanged. Reference-dependent optimal convex blending offers 19.47% translation headroom; the learned model does not recover it.
- Reviewed repetition: residual/conditioning rules, geometry diagnostics, bias/scale calibration, covariance adaptation and full-matrix weighting have substantial prior coverage. Deferred another confidence variant. Thermal translation error is89–98%along reference motion, with short motion magnitudes; this does not prove depth is the cause.

**Latest execution:** completed the bounded preflight for the depth-mean × covariance ablation. All120unchanged native pair solves reproduce saved parameter arrays exactly:

| Recording | RGB/thermal pair solves | Exact matches |
|---|---:|---:|
| Day |40|40|
| Night |40|40|
| Rain |40|40|

This establishes replay parity, not convergence or a new accuracy gain. The intended four-arm comparison changes original/reference-assisted depth means and native/fixed covariance separately. Reference-assisted means are diagnostic input, not a deployable method.

**The four-arm test has not run.** It requires126exact own-camera depth maps for the three current recordings. The two inspected depth-reference backup manifests contain0matching maps: their labels belong to different recordings and are sampled every100frames. Those labels cannot substitute for consecutive source/target depths. Only manifests were retrieved; no large archive download, new dataset, training or additional model experiment followed.

Two focused tests and an independent audit pass, checking32source hashes,120native event/parameter identities and all missing-map paths. [Preflight findings and exact input list](depth-mean-ablation-preflight-findings-2026-11-09.md).

**Next required step:** targeted recovery of the listed RGB/thermal depth maps for IDs000000–000020in each exact sequence. Reuse the qualified replay, qualify label/pixel semantics and matched factor support, then freeze and run the four arms. Do not repeat the history review or substitute another weighting variant. This input dependency leaves the depth-mean hypothesis unanswered.

All accuracy findings use exposed development data, supplied references and nominal alignment. Physical ground truth, unseen-record transfer and full-trajectory improvement remain unestablished. No runtime benchmarking; RGB remains the executable accuracy baseline.
