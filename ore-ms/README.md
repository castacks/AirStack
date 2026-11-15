# RGB–thermal geometric uncertainty research

This project studies geometric uncertainty in visual and thermal estimation,
including matching supervision, selection, continuity and sensor transfer. Its
initial RGB–thermal question asks whether the modalities' uncertainties are
comparable and predict relative geometric reliability. A distinct contribution and consistent adaptive pose improvement
have not yet been established.

Start with the [research handoff](MS_RESEARCH_HANDOFF.md) for current context
and the original research question. The [scope decision](docs/research/research-scope-decision-2026-11-13.md)
records the requirement for a bounded, evidence-backed direction before further
experiments. The [contribution draft](docs/research/contribution-statement-2026-11-13.md)
separates supported findings from claims that remain unverified.

The [current contribution decision](docs/research/contribution-synthesis-decision-2026-11-14.md)
connects the completed studies to the original papers' remaining gaps. The
[updated technical report](docs/research/uncertainty-interfaces-report-draft-2026-11-13.md)
collects supported findings and their limits.

The [sensor-transfer inventory](docs/research/sensor-transfer-readiness-2026-11-14.md)
records current input availability, preprocessing and the claims supported by
nominal references versus qualified physical sensor evidence.

The [active-perception motion decision](docs/research/active-perception-motion-decision-2026-11-14.md)
selects matched acquisition histories as a controlled question. The
[completed estimator comparison](docs/research/active-motion-estimator-findings-2026-11-15.md)
finds small, mixed timing effects; no planning policy is validated.

The [cross-match covariance study](docs/research/cross-match-covariance-findings-2026-11-15.md)
uses saved trials to separate within-match variation from cross-match dependence,
with no new inference or covariance correction.

The [recording-excluded covariance comparison](docs/research/covariance-transfer-findings-2026-11-15.md)
finds partial predictive gains beyond a fitted scalar, with mixed transfer and
containment; it changes no pose estimate.

The [shape/scale diagnostic](docs/research/covariance-shape-scale-findings-2026-11-15.md)
interprets those fixed predictions using an explicitly test-informed oracle;
no predictive result or uncertainty interval is replaced.

Raw experiment outputs are kept outside Git. The
[verified backup index](docs/research/artifact-backup-index-2026-11-15.json)
records Box artifact locations and readback checksums.

## Where to find things

| Location | Purpose |
|---|---|
| [Research handoff](MS_RESEARCH_HANDOFF.md) | Current research context, original scope and historical decisions |
| [Research notes](docs/research/) | Dated protocols, findings and detailed analyses |
| [Paper registry](docs/research/references/README.md) | Primary sources, reading depth, implementation pins and limitations |
| [Experiments](experiments/) | Experiment declarations and reproducibility records |
| [Research tools](tools/) | Instrumentation, analysis and audit scripts |
| [MAC-VO source](MAC-VO-main/) | Extracted upstream pipeline and setup instructions |
| [Agent instructions](AGENTS.md) | Research continuity and operating constraints |

Large datasets, checkpoints and generated results are stored outside Git.
See the [setup instructions](MAC-VO-main/Scripts/Setup/README.md) and the dated
experiment records for their required environments and recovery artifacts.
Historical runtime verification does not establish the current machine's state.

## Documentation convention

Keep this README as a project entry point. Put detailed results in dated research
notes and link current decisions from the handoff; do not copy the progress log
into both files. Older proposals describe their publication-time status and may
have been superseded by later executions.

The former README is preserved in [README_HISTORY.md](README_HISTORY.md).
It is an archive, not the current action list.
