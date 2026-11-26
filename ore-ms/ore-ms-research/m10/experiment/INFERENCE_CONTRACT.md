# Inference and context contract

The isolated process receives a per-partition observation directory containing the approved PF1 frame metadata, modality masks, and the original B01/B02 outputs. B01/B02 encode current detector observations through the frozen contrast frontend. The loader validates both sensor timestamps, known registration, mask values, frame shape, and the candidate schema and bounds. RGB and LWIR eligibility are independent. An eligible observation may still lack candidate evidence. Partition and opportunity IDs are used to align stage I/O and are excluded from the model's feature tuple. No result depends on filename, row position, split, or a latent state. The implementation extracts no range or material feature.

Deployment state contains only the fitted parameters and calibration-only decision rules. The candidate policy returns a bounded ranking score, route, and optional conflict warning. The issuance step has one calibrated threshold per arm and withholds proposals with unqualified calibration bins. It emits a location with a candidate-conditional task probability or an explicit abstention. The reported probabilities are qualified for the declared synthetic population and measured calibration bins. A missing modality may leave a calibrated single-modality route. Both missing, out-of-time, rail-only or no spatial evidence leads to abstention unless a different eligible modality provides a candidate.

Linux Landlock restricts file reads to the exact observation directory and model state, writes to the designated output file, and denies new TCP access. The independent test shows a real scoring path cannot be opened and that altering it leaves decision bytes unchanged. A further six forbidden columns are rejected. This is an experimental process boundary. [Linux Landlock documentation](https://docs.kernel.org/userspace-api/landlock.html)

## Context admissibility results

| Study | Admissible context | Measured disposition |
|---|---|---|
| E04 M07 context | Constrained burden requires known association, qualified transfer and known nuisance sources. A declared search plane or material flag alone cannot establish that a detector pixel is measuring the independently constrained object or that its path meets the M07 likelihood. | 0/6,912 admitted. Uses E03 unchanged. |
| E05 M08 context | Horizon-2-second context needs an inference-only observation at t=0 and a qualified reference path. PF1 stores nominal t=2 observations and a 1.9 s timing control. | 0/6,912 admitted. Uses E03 unchanged. |
| E06 M09 context | The learned forecast and its novelty score require the same t=0 observations and qualified checkpoint inputs. A later frame or a generator state would evade the clock or truth boundary. | 0/6,912 admitted. Uses E03 unchanged. |
| E07 no context | The prespecified primary E03 already uses current detector and candidate evidence alone. | Identical to E03. |

These are failed context admissions, not estimates of context effectiveness. No generator metadata, prior forecast output for a different realization, or synthetic t=0 observations were joined to PF1. Wrong air and shifted dynamics remain scoring strata. The benchmark contains no independently verified mechanism diagnostic at the decision clock. [Scenario outcomes](evidence/scenario_metrics.csv)
