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

1. **Finish the characterization deliverable.** Use this brief and the [detailed report](characterization-report-2026-11-09.md) for advisor discussion: positive ranking evidence, cross-camera counterexamples, failed gates and scope limitations. This is the immediate deliverable, requiring no new runs.
2. **Choose one justified next hypothesis.** A candidate is whether local RGB–thermal correspondence features add predictive value for relative geometric error beyond uncertainty and simple context. First identify the existing reliability failure it would address and a defensible error target. Otherwise retain the completed characterization as the current research outcome.
3. **Keep any future experiment bounded.** Use an existing pretrained model, fixed baselines, recording-separated evaluation and a stopping rule. Do not begin representation training, replace the VO frontend or reopen the parked diagnostic chains without evidence that doing so answers the main question.

Shared-space literature is relevant background: ImageBind for semantic alignment; TartanMatch and self-supervised cross-modal correspondence for geometry. Our [compatibility check](shared-representation-feasibility-2026-11-09.md) found accessible TartanMatch code/checkpoint metadata, but no demonstrated reliability-prediction benefit. A common embedding does not automatically calibrate uncertainty.

**Decision for the next discussion:** retain the main RGB–thermal reliability question, accept the current fixed-rule result, and decide whether a narrowly specified representation-based hypothesis merits a separate experiment. No broader model-development project is currently started.

*This brief synthesizes completed records; it introduces no new empirical result. Source through project commit `4f642ec45`.*
