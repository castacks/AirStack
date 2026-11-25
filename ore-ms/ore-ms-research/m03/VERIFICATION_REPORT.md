# M03 verification report

The configuration and acceptance tolerances were hashed before implementation and evaluation. Forward transfer and detector code live in `thermal_model.py` and `experiment.py`. The verification harness uses 45-digit Decimal spectral references, RK4 and a separately composed transfer integral in `reference.py`. This is procedural separation within the same development effort.

| Check | Status | Executed result |
|---|---|---|
| V01 Planck reference and physical admissibility | PASS | Twelve point comparisons gave maximum relative error 8.88e-16. Temperature monotonicity, sampled Wien peak and finite broad tails passed. |
| V02 Stefan–Boltzmann | PASS | Maximum broad-spectrum relative discrepancy 6.92e-9 across 280–340 K. Maximum difference on refining from 2048 to 4096 intervals was 1.51e-16 relative to σT⁴/π. |
| V03 surface and gray atmosphere | PASS | Blackbody, perfect-reflector and matched-temperature bounds passed. All five prescribed transmissions reproduced the analytic isothermal solution. |
| V04 layered transfer | PASS | Maximum difference against RK4 was 1.78e-14 W·m⁻²·sr⁻¹·µm⁻¹. The 128/256-step reference difference was at most 7.64e-14 in the same units. Layer reversal preserved transmission and changed path emission. |
| V05 spectral integration and detector | PASS | The band reference was 44.9797842017 W·m⁻²·sr⁻¹. The 48/96/192-interval errors were 5.90e-4, 1.48e-4 and 3.69e-5 in the same units, with successive reduction factors 4.00004 and 4.00001. The noise sample variance was 0.09036 against 0.09 (W·m⁻²·sr⁻¹)². Quantization, seeded repeatability and both rails passed. |
| V06 cases, contrast and metrics | PASS | 53 cases, 20,458 spectral rows and 6,784 detector samples. Cross-checked integrals, component closure, measured errors, clipping and target contrast. The one crossover bracket, visible equality and hidden-target controls passed. |
| V07 repeatability and scope | PASS | Fresh runs matched spectra, digital measurements and metrics byte for byte. M01 and M02 archives verified and both regression harnesses replayed. |
| External physical calibration and organizationally independent verification | NOT TESTED | The controlled fixtures and coefficients are synthetic, and no hardware transfer claim was evaluated. |

All seven automated tests passed. See [verification](evidence/verification.log), [replay](evidence/replay.log), [prior regressions](evidence/prior_regressions.log), and [provenance](evidence/provenance.log).
