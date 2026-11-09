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

## Current implementation progress and next steps

The [pose-covariance prototype](pose-covariance-prototype-findings-2026-11-09.md) is implemented. It uses saved feature geometry and both endpoint covariances;15 numerical tests and160 synthetic trials pass. On the first600groups of the planned day/night records,2,292/2,396 camera intervals converge;58 hit the fixed solver cap and46 lack a native first motion write. Independent numerical checks pass. No new frontend inference or reference-error scoring occurred. These counts establish numerical feasibility and expose an availability issue; they do not demonstrate improved VO.

Next: finish common-frame covariance transport and equal/scalar/full combination; account for capped/missing outputs in coverage; recover/capture the exact continuous rain prefix; then freeze predictions and score the original accuracy/calibration question. Existing short rain windows will not replace the declared600-group stream.

This is a weekly progress update, not the endpoint of the research. Calibration, MACThermal frontend improvements, shared representations and IMU fusion remain available next hypotheses, chosen from the measured failure mode rather than all changed at once.

*Updated with numerical implementation/preflight results through `6308b61f3`; reference-error results retain their original source records. No new adaptive-accuracy claim.*
