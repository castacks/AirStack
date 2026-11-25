# M05 acceptance report

- **Experiment** ORE-MS-EXP-PER-001
- **Directive** M05-R1
- **Recommendation** ACCEPT WITH CONDITIONS. All nine frozen verification criteria passed. Project Director review remains pending.

M05 implements exact discrete transitions for a Gaussian OU coefficient field, periodic Fourier spatial covariance, prescribed advection, and a forced clear–degraded–clear regime. It produces time- and space-correlated burden and temperature, maps to bounded atmospheric quantities, and uses separate RGB extinction/scattering and LWIR absorption. See the [scientific model](SCIENTIFIC_MODEL.md) and [traceability](REQUIREMENTS_TRACEABILITY.csv).

Five stationary arms used 4,096 independent realizations each. The forced population used 512. Four output scenarios retained 3,664 time-position records, and 16 long-run realizations covered 131,072 stress evaluations with zero violations. The widest discrepancy from theoretical normalized temporal/spatial covariance was 0.02559/0.03716. The deterministic moving peak matched all ten prescribed positions, and every output table replayed byte for byte. The M01–M04 regression suites and archived evidence passed. See [verification and confidence intervals](VERIFICATION_REPORT.md).

The live [Acceptance Conditions Register](../ACCEPTANCE_CONDITIONS_REGISTER.md) separates later claim gates, scheduled validation and optional extensions. Required obligations include external organizational assurance and resolving identifiability before estimation acceptance. Independent simulator and physical validation are deferred to M11 and M12. Open-boundary or multidimensional transport and weather-specific microphysics are optional unless required by a separately approved experiment. Project Director decision and explicit authorization gate any M06 work. Neither combined rendering nor later prediction, estimation or fusion was implemented.
