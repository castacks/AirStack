# M03 acceptance report

- **Experiment** ORE-MS-EXP-PER-001
- **Directive** M03-R1
- **Recommendation** ACCEPT WITH CONDITIONS. All seven frozen verification criteria passed. The numerical and experimental evidence establishes behavior for the declared synthetic LWIR model. Project Director approval, physical sensor validation and external independent review remain open.

The model separates spectral surface emission, reflected environmental radiance, transmission, emitted path radiance and detector response. It provides nonuniform layer composition, a configurable illustrative 8–14 µm passband, seeded noise, quantization, both clipping rails and explicit visibility reporting. See the [scientific specification](SCIENTIFIC_MODEL.md) and [requirements and task traceability](REQUIREMENTS_TRACEABILITY.csv).

The 53-case matrix varies target and background temperature, emissivity, gray transmission, reflected radiance, atmospheric self-emission, spectral profiles and detector settings. It includes a designated crossover sweep and a supplied static hidden-target control. In the clear reference the target band was 56.701 W·m⁻²·sr⁻¹ against 46.407 background. At τ=0.5 shared-path contrast halved to 5.147, and τ=0 eliminated direct contributions. Reversing the spectral layers changed path radiance from 11.955 to 12.442 W·m⁻²·sr⁻¹ while leaving direct transfer unchanged. The two saturation controls clipped every target reading. The visible sweep brackets a crossover near 316.917 K. See the [full metrics](evidence/metrics.csv) and [crossover analysis](CROSSOVER_ANALYSIS.md).

The worst full-spectrum Stefan–Boltzmann discrepancy was 6.92e-9 relative. The frozen band discretization differed from its independent Simpson reference by 3.69e-5 W·m⁻²·sr⁻¹ with approximately second-order convergence. Both previous milestones replayed successfully, and fresh M03 runs matched all three output tables byte for byte. See the [verification report](VERIFICATION_REPORT.md) and [reproduction instructions](REPRODUCTION.md).

Measured spectral response, directional emissivity, atmospheric coefficients, calibrated detector noise and multiple scattering remain unresolved. The model does not recover temperature from a single measurement. The [roadmap mandate](../RESEARCH_ROADMAP_MANDATE.md) records the directive's conditional acceptance of M01/M02 and M03 in verification. M04–M13 await authorization.
