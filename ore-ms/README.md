# RGB–thermal geometric uncertainty research

This project studies whether RGB and thermal geometric uncertainties are
comparable, predict relative geometric reliability, and can improve visual
odometry. A distinct contribution and consistent adaptive pose improvement
have not yet been established.

Start with the [research handoff](MS_RESEARCH_HANDOFF.md) for current context
and the original research question. The [scope decision](docs/research/research-scope-decision-2026-11-13.md)
records the requirement for a bounded, evidence-backed direction before further
experiments. The [contribution draft](docs/research/contribution-statement-2026-11-13.md)
separates supported findings from claims that remain unverified.

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
