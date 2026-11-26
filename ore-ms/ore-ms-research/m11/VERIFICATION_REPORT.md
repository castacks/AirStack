# M11-R1 verification report

Base revision `ff64941152ad10b0656a2c68d80eb4121b070b97`. Separate numerical references, fresh synthetic scenarios and immutable subject interfaces were used. [Reference qualification](REFERENCE_QUALIFICATION.md) · [Prospective plan](VERIFICATION_PLAN.md) · [Traceability](REQUIREMENTS_TRACEABILITY.csv)

| Gate | Disposition | Measured basis |
|---|---|---|
| V01 | PASS | Git comparison to the issued base preserves all historical artifacts; only the designated live register, roadmap, mandate and research index changed outside the new M11 package. |
| V02 | PASS | Reference import boundary checked in tests. Independent QUADPACK, matrix exponential, spectral Jacobian, raster and statistical implementations have explicit shared-model limits. |
| V03 | PASS | 24 RGB matched cases within 1.11e-16, Gaussian detector moments and rails within bounds. Lossless multiple-scattering challenge shows approximation discrepancy. |
| V04 | PASS | Planck relative error 1.74e-15, final band error 4.94e-5 W m⁻² sr⁻¹, expected crossover signs and detector codes. |
| V05 | PASS | 3,300 rays with zero ID discrepancies, 3.55e-15 m maximum range error. Independent registration fixtures report downstream sensitivity. |
| V06 | PASS | 24,000 OU trajectories agree within 0.04 standardized units, exact transition variance to 2.22e-16, Euler convergence and 10,000 finite steps. |
| V07 | CONDITIONAL | Seven paths meet all band and code criteria, including foreground substitution and advection. Full sequence parity and external physical-model coupling remain unverified. |
| V08 | CONDITIONAL | M07 posterior max residual 3.67e-5, M08 covariance residual 2.22e-16, M09 fresh-input transfer executed. Wrong-air and future-only failure detected. Full M08 bank and M09 training-seed parity remain unverified. |
| V09 | CONDITIONAL | Frozen five-arm study executed on 48 scene groups. Large-offset recovery observed, overall gain inconclusive and substantial false support remains. |
| V10 | PASS | Prospective per-case budgets, cluster interval, coverage intervals and independent parameter sensitivity calculations recorded. Claims restricted to declared populations. |
| V11 | CONDITIONAL | 11 deterministic files replayed byte-for-byte, host wall latency and RSS measured. Live transport and renderer telemetry remain untested. |
| V12 | PASS | Source-based capability matrix, architecture proposal and IS01–IS08 disposition delivered. Pilot readiness is a separate decision. |
| V13 | PASS | R1 scope and 105-task catalog preserved. Pending downstream authorizations and scientific conditions remain explicit. |

The suite verifies matched numerical behavior in the stated domains. The original M10 effect remains supported for its own frozen holdout. Scoped independent challenges reproduced both a large-offset recovery mode and failure of broader calibration transfer. [Discrepancy register](DISCREPANCY_REGISTER.md) · [Evidence and replay](REPRODUCTION.md)
