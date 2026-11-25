# M01 acceptance report

- **Experiment** ORE-MS-EXP-PER-001
- **Project** ore-ms-research
- **Scope** M01 reconciliation under R2 and Amendment A1
- **Recommendation** ACCEPT WITH CONDITIONS. The recorded evidence passes G0–G6. Project Director review is required for milestone acceptance. Claims remain limited to synthetic and numerical verification, and organizationally independent review requires an external reviewer.

The [Research Roadmap Mandate](RESEARCH_ROADMAP_MANDATE.md) governs milestone order and authorization. M01 is in verification. M02–M13 remain pending.

## Requirements compliance

| Task and requirements | Status | Evidence |
|---|---|---|
| T01, R01 | PASS | The [capability audit](00_EXISTING_CAPABILITY_AUDIT.md) records host detections and versions where available, license sources, maturity, compute needs and reuse limits. [Provenance](evidence/provenance.log) identifies the starting revision, scope and source hashes. |
| T02, R02 | PASS | [Foundations](04_MATHEMATICAL_FOUNDATIONS.md) give E1–E5, units, limits, and assumptions. The [reference catalog](01_SCIENTIFIC_REFERENCE_CATALOG.md) identifies reading scope. |
| T03, R03 | PASS | `test_rgb_boundaries` checks zero, finite and thick path limits. `test_paired_capture` checks all RGB channels against a separate affine expression. |
| T04, R04 | PASS | `test_planck_units_and_boundaries`, `test_band_reference_and_convergence`, and `test_paired_capture` check Planck units, monotonicity, temperature/emissivity limits, response, transport and separate spectral integrals. |
| T05, R05 | PASS | [Common scene](08_COMMON_SCENE.md) and `scene.json` document geometry, materials and atmosphere. `test_paired_capture` verifies range, visibility, surface and radiance. |
| T06, R07 | PASS | The [taxonomy](05_ENVIRONMENTAL_DEGRADATION_TAXONOMY.md) separates geometric visibility, path attenuation, crossover, noise and coupled mechanisms. |
| T07, R07 | PASS | The [stochastic](15_STOCHASTIC_ARCHITECTURE.md) and [learning](16_MACHINE_LEARNING_ARCHITECTURE.md) designs cover state estimation, classical forecasting, physics-informed learning and adaptive fusion as future work. |
| T08, R06/R08 | PASS | [Reproduction instructions](21_REPRODUCTION.md), [verification log](evidence/verification.log), [catalog audit](evidence/reconciliation.log) and [master matrix](26_MASTER_VERIFICATION_MATRIX.csv) cover executed checks and evidence. |

[R01–R08 traceability](13_REQUIREMENTS_TRACEABILITY.csv) and the [G0–G6 evidence matrix](24_ACCEPTANCE_GATES.md) provide requirement and gate detail.

| Physical effect | M01 representation | Deferred verification or extension |
|---|---|---|
| RGB boundary radiance and path radiance | Linear effective bands and prescribed homogeneous path sources with Beer–Lambert transmission | Spectral response, measured sources and angular/multiple scattering |
| LWIR emission, reflection and transfer | Planck emission, opaque gray diffuse emissivity, effective isotropic reflection and isothermal slab | Measured spectral emissivity, heterogeneous transfer and calibrated detector response |
| Spectral response and crossover | Illustrative 8–14 µm weighted response and matched-temperature equality check | Measured response and crossover under physical conditions |
| Geometry | One camera frame, opaque first surface, slant range and shared pixel grid | Dynamic occluders, partial coverage and real cross-sensor registration |
| Sensor noise | Seeded additive Gaussian radiance perturbation | Shot noise, blur, drift, clipping, quantization and correlated faults |

## Reproduced claims

Run commands in the package root. Set `out` to an empty writable directory. The source configuration is `scene.json` and its SHA-256 is recorded in [provenance](evidence/provenance.log).

| Claim | Exact command and method | Evidence |
|---|---|---|
| 442 pairs | `python3 simulate.py scene.json "$out/observations.csv"`, then `python3 verify.py "$out/observations.csv"`. `test_paired_capture` checks 221 per case, with 197 target-visible and 24 foreground pixels per case. | `evidence/observations.csv` in the downloadable archive, [audit log](evidence/reconciliation.log) |
| Five passing tests | `python3 verify.py "$out/observations.csv" > "$out/verification.log" 2>&1`. Tests cover RGB limits, Planck limits, quadrature convergence, noise and registered capture. | [Verification log](evidence/verification.log) |
| Maximum relative error 0.000678370% | `test_band_reference_and_convergence` compares 32/64/128 trapezoids to independently expressed 32-digit Decimal Simpson quadrature at 1024/2048 intervals. `python3 audit_reconciliation.py "$out/observations.csv" "$out/verification.log"` computes the worst 128-interval relative error across four cases. | [Verification log](evidence/verification.log), [audit log](evidence/reconciliation.log) |
| Byte-identical repeated runs | `python3 simulate.py scene.json "$out/repeat.csv"` and `cmp "$out/observations.csv" "$out/repeat.csv"`. A second `cmp` matches the historical capture. | [Replay hashes](evidence/reproduction.log) |
| 105 tasks | `python3 audit_reconciliation.py "$out/observations.csv" "$out/verification.log"` checks counts 25/30/30/20, unique IDs, 147 acyclic ordered dependency edges, all thirteen milestones and five phases. | [Task catalog](14_WORK_BREAKDOWN_STRUCTURE.csv), [dependencies](17_TASK_DEPENDENCIES.csv), [audit log](evidence/reconciliation.log) |

The analytic checks establish agreement with the stated model. The numerical checks establish integration error and convergence for the tested spectra. Captures establish synthetic registration and truth consistency. Physical validity, measured sensor response, portability of byte-level reproducibility and formal independent acceptance remain open. Development and verification use separate plans, references, code and records within the same Codex effort. Independent simulator reproduction, controlled physical tests and organizationally independent acceptance need external review under their respective scope.

## Identifier migration and evidence

The deterministic work started at `cecb41fafea60d6d9b5b8f1ba6362ee6d6fa6c93`. Its original implementation is in `4eb24007d52f0f1780901b74603b80ed5f6dd7fd`. R2 replaces the old identifier `RRM-EXP-PER-001` and retires the earlier named baseline prerequisite. Active files moved from `ore-ms/rrm_per_m01` to `ore-ms/ore-ms-research`. The simulator, verification harness and scene are byte-identical to the original commit. The original archive and manifest are preserved at their historical identifiers, and the original archive hash is verified in [provenance](evidence/provenance.log).

Reconciliation ran in the provided remote AirStack workspace on the designated branch. The session exposed no OSMO job identifier, so pool provenance could not be independently confirmed. Box is optional supporting storage. No Box archive or dataset was downloaded or modified during this reconciliation.

The [active manifest](PACKAGE_SHA256SUMS.txt) covers repository source, configuration, reports and lightweight evidence. The downloadable archive also includes the byte-identical observation CSV and a manifest covering that captured data. The exact prior names for documents 01–25 were unavailable, so the table reconciles each existing filename to its intended deliverable by content.

## Deliverable mapping

| Slot | Existing filename | Intended deliverable |
|---|---|---|
| 00 | [00_EXISTING_CAPABILITY_AUDIT.md](00_EXISTING_CAPABILITY_AUDIT.md) | Existing software capability audit |
| 01 | [01_SCIENTIFIC_REFERENCE_CATALOG.md](01_SCIENTIFIC_REFERENCE_CATALOG.md) | Scientific references |
| 02 | [02_SYSTEM_REQUIREMENTS.md](02_SYSTEM_REQUIREMENTS.md) | System requirements |
| 03 | [03_SYSTEM_ARCHITECTURE.md](03_SYSTEM_ARCHITECTURE.md) | System architecture |
| 04 | [04_MATHEMATICAL_FOUNDATIONS.md](04_MATHEMATICAL_FOUNDATIONS.md) | Mathematical and physical foundations |
| 05 | [05_ENVIRONMENTAL_DEGRADATION_TAXONOMY.md](05_ENVIRONMENTAL_DEGRADATION_TAXONOMY.md) | Environmental degradation taxonomy |
| 06 | [06_RGB_BASELINE.md](06_RGB_BASELINE.md) | RGB baseline design and implementation guide |
| 07 | [07_LWIR_BASELINE.md](07_LWIR_BASELINE.md) | LWIR baseline design and implementation guide |
| 08 | [08_COMMON_SCENE.md](08_COMMON_SCENE.md) | Common simulation scene and frame contract |
| 09 | [09_SIMULATION_CONFIGURATION.md](09_SIMULATION_CONFIGURATION.md) | Simulation configuration and seeds |
| 10 | [10_GENERATED_OBSERVATIONS.md](10_GENERATED_OBSERVATIONS.md) | Generated observations and truth inventory |
| 11 | [11_VERIFICATION_PLAN.md](11_VERIFICATION_PLAN.md) | Automated verification plan |
| 12 | [12_VERIFICATION_RESULTS.md](12_VERIFICATION_RESULTS.md) | Executed verification results |
| 13 | [13_REQUIREMENTS_TRACEABILITY.csv](13_REQUIREMENTS_TRACEABILITY.csv) | Requirements traceability matrix |
| 14 | [14_WORK_BREAKDOWN_STRUCTURE.csv](14_WORK_BREAKDOWN_STRUCTURE.csv) | 105-task work breakdown |
| 15 | [15_STOCHASTIC_ARCHITECTURE.md](15_STOCHASTIC_ARCHITECTURE.md) | Future stochastic architecture |
| 16 | [16_MACHINE_LEARNING_ARCHITECTURE.md](16_MACHINE_LEARNING_ARCHITECTURE.md) | Future machine learning architecture |
| 17 | [17_TASK_DEPENDENCIES.csv](17_TASK_DEPENDENCIES.csv) | Task dependencies |
| 18 | [18_RESEARCH_ROADMAP.md](18_RESEARCH_ROADMAP.md) | Research roadmap and mandate reference |
| 19 | [19_ALGORITHM_REUSE_MATRIX.csv](19_ALGORITHM_REUSE_MATRIX.csv) | Algorithm reuse matrix |
| 20 | [20_KNOWN_LIMITATIONS.md](20_KNOWN_LIMITATIONS.md) | Known limitations |
| 21 | [21_REPRODUCTION.md](21_REPRODUCTION.md) | Reproduction instructions |
| 22 | [22_DEVELOPMENT_RECORD.md](22_DEVELOPMENT_RECORD.md) | Development evidence |
| 23 | [23_IVV_EVIDENCE_RECORD.md](23_IVV_EVIDENCE_RECORD.md) | Separate IV&V evidence |
| 24 | [24_ACCEPTANCE_GATES.md](24_ACCEPTANCE_GATES.md) | Acceptance gate evidence |
| 25 | [25_FROZEN_HANDOFF.md](25_FROZEN_HANDOFF.md) | Frozen handoff and manifest reference |
| 26 | [26_MASTER_VERIFICATION_MATRIX.csv](26_MASTER_VERIFICATION_MATRIX.csv) | Master verification matrix |

The numbered records are accompanied by `physics.py`, `simulate.py`, `verify.py`, `audit_reconciliation.py`, `scene.json`, the recorded logs, and the raw observation CSV in the downloadable handoff archive.
