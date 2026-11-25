# Acceptance gates

| Gate | Evidence disposition | Evidence | Remaining limits |
|---|---|---|---|
| G0 | PASS | [Provenance log](evidence/provenance.log) records the starting revision, scoped diff, source hashes and historical checksum verification. [Replay](evidence/reproduction.log) matches the original observation output. | Repeatability was tested on the recorded runtime. |
| G1 | PASS | [Foundations](04_MATHEMATICAL_FOUNDATIONS.md) and [scope matrix](M01_ACCEPTANCE_REPORT.md) cover units, assumptions and modeled effects. | Slab atmosphere, gray surfaces and idealized sensors. |
| G2 | PASS | [Verification log](evidence/verification.log) verifies analytic transport, path radiance and foreground/target radiance at all 442 paired pixels. | Linear effective bands and prescribed RGB radiances. |
| G3 | PASS | [Verification log](evidence/verification.log) records separate Decimal Simpson references, temperature and emissivity checks and convergence. | Illustrative spectral response. Physical response remains unmeasured. |
| G4 | PASS | [Metrics and catalog audit](evidence/reconciliation.log) confirms shared geometry, configured cases and foreground/target counts. [Verification log](evidence/verification.log) checks all ranges and radiances. | Synthetic truth and ideal registration. |
| G5 | PASS | [Verification log](evidence/verification.log) records five passing tests. [Replay](evidence/reproduction.log) records byte-identical runs and comparison with the historical output. | Cross-runtime byte identity remains untested. |
| G6 | PASS | [Acceptance report](M01_ACCEPTANCE_REPORT.md) maps 00–26. [Manifest](PACKAGE_SHA256SUMS.txt) covers the active package and its lightweight evidence. | Organizationally independent review remains open. |

Gate dispositions describe executed evidence. Project Director acceptance remains pending under the [Research Roadmap Mandate](RESEARCH_ROADMAP_MANDATE.md).
