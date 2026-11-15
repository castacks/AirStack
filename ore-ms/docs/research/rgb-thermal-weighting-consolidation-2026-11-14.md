# RGB–thermal weighting experiments: consolidated decision

14 November2026 America/New_York; ore-ms-orb-development baseline53afa19d1.
Evidence consolidation only. No new model, inference, training or accuracy run.

## What was repeated and what changed

| Study | Shared question | Actual added comparison |
|---|---|---|
| Shared-weight learning,12November | Can pose-trained observation weights improve joint RGB–thermal estimation? | Six-parameter affine head using depth,covariance,imageposition; different gauge/range and fullbatch training. |
| Translation fusion,9November | Can sensor evidence predict useful thermal influence? | Eight summary cues regress pose-level translation blends; RGB orientation retained. Different recordings/timing convention from later100group work. |
| Image/temporal feature comparison | Same prediction-of-thermal-utility question | Added global gradient/anisotropy/KLT summaries to a ridge pose-blend model; no improvement. |
| Local factor geometry | Does local thermal geometry improve a joint estimate? | Standard conditionalinformation vsuniform64factor controls; translation/rotation tradeoffs, no learned model. |
| Local relationship learning | Same pose-trained observation-weighting question as12November | Nonlinear24input layer, owncamera patch descriptors and RGB-relative directional information; fixedRGBprecision and boundedthermalgate; matched simpler controls. |

The latest learning run is an input/model ablation of an existing direction, not
a fundamentally new research solution. Its justification was testing whether the
new inputs add value. They did not establish an increment over constant/affine
weighting. This distinction should have been stated before execution; architectural
clarification about MACThermal's thermal-only inputs did not erase the existing
local RGB–thermal weighting experiments.

## What survives

Conservative thermal precision in the conditional joint estimator gives scoped
translation benefits on day/night. The fullMLP yields3.08%/1.49%lowerT-RMSE
versus same-backendRGB, with−0.15%/+0.78%R-RMSE changes; rainT/RRMSE worsen
0.16%/0.04%. A fixed0.1thermalgate is nearly identical; affineT-RMSE is lower
thanfullMLP in every fold. These are already exposed same-ID nominal100group
records, qualified provided references, fixedsource-landmark solver and relative
motion metrics. They are not fullnativeVO trajectory, physicalGT, deployment,
compute-saving or algorithmnovelty results.

Earlier studies use different cohorts,solvercontracts,trainingranges and targets.
Do not pool their percentages or describe them as exact reruns. The evidence
supports specific thermalutility cases and repeated failure to demonstrate an
advantage from our more elaborate weighting models. It does not prove that
thermal contribution is unpredictable or that representation learning cannot work.

## Decision

Treat the tested weighting family as completed groundwork. Do not advance another
head,featurepack,gate,epochbudget,covarianceprojection or scalarblend merely as a
renamed new direction. Preserve the useful simplefusion baseline and the negative
incremental findings together. No next experiment or new research direction is
selected by this consolidation. A future proposal must identify its actual
mechanism difference, applicable priorwork and a remaining application failure
before numerical execution. A bounded extension can be useful, but label it as
such and explain why its expected information gain justifies the run.

## Authoritative evidence

- [Shared affine weighting](shared-weight-learning-findings-2026-11-12.md).
- [Saved-pose translation fusion](translation-fusion-findings-2026-11-09.md).
- [Image/temporal input ablation](thermal-incremental-comparison-2026-11-14.md).
- [Local geometry intervention](local-thermal-geometry-findings-2026-11-14.md).
- [Completed local relationship learning](local-relationship-learning-findings-2026-11-14.md).
- [Architecture clarification](rgb-thermal-architecture-and-contribution-reset-2026-11-14.md).

Allsource/result links retain their own populations,limitations and backup locations.
No new paperreading,publicationnovelty claim or author-system reproduction.
