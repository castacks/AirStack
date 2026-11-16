# RGB–thermal geometric uncertainty research

This project studies geometric uncertainty, selection and sensor combination for
visual and thermal motion estimation. The application goal is improved accuracy
or preserved accuracy at lower measured computation. Local RGB–thermal extensions
have scoped gains; a consistently superior fusion method and publication novelty
have not been established.

MAC-VO uses visible stereo, MACThermal uses thermal stereo, and our extensions
combine evidence from both. See the
[architecture and contribution contract](docs/research/rgb-thermal-architecture-and-contribution-reset-2026-11-14.md)
for the research scope.

The [historical contribution decision](docs/research/contribution-go-no-go-2026-11-15.md)
applies to new-method novelty. The current handoff governs the user-selected
deployment characterization of accuracy, availability and cost across conditions.

## Start here

| Resource | Purpose |
|---|---|
| [Current handoff](MS_RESEARCH_HANDOFF.md) | Present evidence, unresolved issues and the next bounded task |
| [Research and experiment index](docs/research/INDEX.md) | Topic entry points and numbered experiment catalog |
| [Paper registry](docs/research/references/README.md) | Primary sources, reading depth, implementation pins and limitations |
| [Setup instructions](MAC-VO-main/Scripts/Setup/README.md) | Runtime, model and data recovery requirements |
| [Agent instructions](AGENTS.md) | Research operating rules |
| [Research history](MS_RESEARCH_HISTORY.md) | Historical checkpoints; proposed next steps may be superseded |
| [README history](README_HISTORY.md) | Immutable early project narrative |

## Repository contents

Detailed protocols and findings live in [research notes](docs/research/).
Numbered declarations and frozen manifests live in [experiments](experiments/).
[Research tools](tools/) contain runners, analysis, plots, recovery helpers and
independent auditors. [MAC-VO source](MAC-VO-main/) contains the extracted upstream
pipeline and research integration scripts. Frozen script paths and source hashes
are part of the reproducibility record.

Large datasets, checkpoints and generated results are stored outside Git.
The [backup index](docs/research/artifact-backup-index-2026-11-15.json) records
verified Box locations and checksums for a subset of evidence packets. Historical
runtime verification does not establish the current machine's state.

Keep this page focused on orientation. Update the current handoff for active work
and place numerical details in dated findings linked from the index.

Run `python3 ore-ms/maintenance/check_docs.py` from the repository root before
submitting documentation changes. CI checks concise entry pages, local file links,
experiment-index coverage and the integrity of the frozen checkpoint history.
