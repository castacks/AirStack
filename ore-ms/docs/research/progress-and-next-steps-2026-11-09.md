# RGB–Thermal Visual Odometry: Progress and Next Steps

**Progress brief · 9 November 2026**

## Research question

Can RGB and thermal uncertainty predict which sensor provides more reliable geometric measurements, and can that information improve visual odometry (VO)?

**Current conclusion:** the study establishes useful within-camera uncertainty ranking and a reproducible evaluation workflow. The tested cross-camera selection rules have not demonstrated reliable improvement over simple camera baselines. This limits the current method; it does not disprove the broader research hypothesis.

## Work completed and findings

| Work completed | Main finding |
|---|---|
| Reproduced RGB/thermal pipelines and recorded uncertainty, support, motion and failures. | Established a repeatable basis for comparing sensor reliability. |
| Evaluated within-camera uncertainty against supplied geometric references. | At fixed coverage, controlled mean-L2 selective-risk median reductions were about **9–37% versus matched random** on the examined windows. Ranking usefulness does not establish calibrated covariance or cross-camera comparability. |
| Tested cross-camera selection and recording transfer. | Results depend on recording and baseline. Large pooled improvements can be dominated by one known RGB-collapse interval; they did not establish consistent recording-level superiority. |
| Executed full-record baseline comparisons. | Thermal ORB had higher tracking coverage but higher translation RPE in day, night and rain. Tracking continuity and accuracy are separate outcomes. |
| Completed the frozen day/night uncertainty-selection evaluation. | All four continuous native runs completed. The evaluation stopped under its predeclared coverage gate; matched comparisons also showed no benefit over RGB. |

Sources: [within-camera results](ms2-findings-2026-10-26.md), [transfer analysis](signal-transfer-findings-2026-11-04.md), [ORB comparison](orb-full-record-findings-2026-11-05.md), [final evaluation](decisive-uncertainty-findings-2026-11-06.md).

## Final evaluation: what the stop means

| Measure | Day | Night | Predeclared requirement |
|---|---:|---:|---|
| Joint reference-qualified coverage | 95.12% | 89.68% | At least 95% per recording |
| Rotation oracle headroom | 9.04% | 12.20% | At least 10% per recording |
| Angular rule versus RGB, matched rotation error | Equal | 1.32% worse | At least 5% improvement against every declared baseline |

The formal outcome is **STOP_INSUFFICIENT_EVALUATION**, because night coverage fails. Day also lacks the required oracle headroom. Available matched results do not support the intended adaptive benefit. We retain those outcomes rather than relax thresholds or tune on the evaluation recordings. Both recordings are now development-exposed.

Errors use supplied/recorded reference proxies and documented nominal frame/time assumptions. Physical synchronization and frame truth, checkpoint training overlap, generalization and downstream adaptive VO benefit remain unverified. The final selector runs both experts and establishes no compute saving.

## What is next

This is a weekly progress brief, not the endpoint of the study. The next technical experiment is [geometry-aware RGB–thermal motion uncertainty](next-experiment-pose-covariance-2026-11-09.md), building on MAC-VO/MACThermal and MAC-I².

1. **Recover and pin a bounded development cohort.** Use the first 600 original groups from each specified day/night/rain recording, checking saved paired 3D covariances before requesting any new capture.
2. **Build the feature-to-motion covariance bridge.** Verify the registration solve and SE(3) covariance transport. Compare equal weighting, block-isotropic pose covariance and full pose covariance on the same replayed motions; retain native single-camera and scalar-selector controls.
3. **Measure and choose the next mechanism.** Report per-recording rotation/translation error, coverage and calibration. A useful gain motivates recording-separated replication and later VO integration; biased means or poor covariance calibration motivate a targeted frontend or uncertainty-learning experiment.

Implementation and execution are pending. Existing scalar-test outcomes remain unchanged. Shared representations and IMU fusion remain credible future directions; this first experiment isolates the visual uncertainty-to-motion mechanism rather than changing all components together.

*This brief synthesizes completed records; it introduces no new empirical result. Source through project commit `4f642ec45`.*
