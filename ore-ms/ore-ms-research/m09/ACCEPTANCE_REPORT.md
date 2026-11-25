# M09 acceptance report

- **Experiment** ORE-MS-EXP-PER-001
- **Directive** M09-R1
- **Recommendation** ACCEPT WITH CONDITIONS for the declared synthetic learning comparison. All ten verification gates passed. Director acceptance and any broader calibration or physical generalization claim remain open.

The data-driven network predicts separate bounded marginals. The physics model and matched ablation use the same exact spectral maps, with the physics model alone trained using the accepted M08 full-field forecast as a probabilistic teacher. Training spans 128/768 realizations and three seeds. Validation uses 256 independent realizations, and every comparison uses the 128 original M08 holdout IDs. [Model](SCIENTIFIC_MODEL.md) · [Traceability](REQUIREMENTS_TRACEABILITY.csv)

The physics loss improved the small-cohort comparison against its matched ablation. With 768 training realizations, its joint future CRPS was 0.098800 versus unchanged M08's 0.098667, with paired interval including zero. There is no demonstrated gain over the classical forecast. Physics reduced issue latency in the reported warm CPU probe. Thermal and dynamics mismatches degraded coverage and the latter could not be detected from identical issue codes. [Score analysis](BASELINE_ANALYSIS.md) · [Robustness](ROBUSTNESS.md) · [Compute](COMPUTE.md)

The data/teacher/oracle boundary, deterministic replay and prior archive regressions passed. AC18 closed with M08's restricted approval, while AC10, AC17 and AC19 remain open. The register adds M09 approval and learned robustness conditions. M10 awaits explicit authorization. [Verification](VERIFICATION_REPORT.md) · [Reproduction](REPRODUCTION.md) · [Acceptance conditions](../ACCEPTANCE_CONDITIONS_REGISTER.md)
