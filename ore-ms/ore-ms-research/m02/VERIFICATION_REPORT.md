# M02 verification report

The verification plan and primary configuration were hashed before implementation and execution. [Reference code](reference.py) uses a 45-digit Decimal slab solution, an independent continuous RK4 integration and exact-per-segment Simpson projection. The forward solver instead composes exponential layers and uses a closed-form product integral. These are separate computational references within the same development effort. Organizationally independent review remains open.

| Check | Result | Executed evidence |
|---|---|---|
| V01 analytical radiative transfer | PASS | `test_analytic_transfer` compares vacuum, absorption, isotropic equilibrium, near-zero and optically thick slabs and ordered layered composition against the reference. |
| V02 spatial convergence | PASS | The 16/32/64-layer errors were 2.822e-7, 7.055e-8 and 1.764e-8 W·m⁻²·sr⁻¹·nm⁻¹. Refinement ratios were 3.99927 and 3.99982. The 2048/4096-step RK4 references agreed within the frozen 1e-12 bound. |
| V03 projection, illumination, noise and detector | PASS | Exact-integral and independent projection checks passed. The Gaussian read/shot sample variances were 144.365 and 501.266 e⁻² against 144 and 500. Quantization, clipping, seed checks and illumination scaling passed. |
| V04 metrics and controlled trends | PASS | Each primary case retained 484 RGB pixels. Clear SSIM was 1. All radiometric RMSE values were recomputed from raw channel records. Monotone direct attenuation, distance and matched path-contrast tests passed. |
| V05 phase and physical admissibility | PASS | The g=0.65 phase integral was 0.9999998214 under sphere quadrature and g=0 matched isotropic scattering. Invalid inputs were rejected and recorded radiance components remained nonnegative. |
| V06 repeatability and isolation | PASS | Two fresh invocations produced byte-identical tables for each configuration. The M01 archived checksum and replay remained valid. Mission scope, catalog and dependency checks passed. |
| Independent simulator and physical sensor validation | NOT TESTED | The coefficients, response functions, and scene are synthetic. No measured phase, camera or atmospheric data were used. |

All seven automated tests passed. See [primary verification log](evidence/verification.log), [replay log](evidence/replay.log), [headroom check](evidence/headroom.log), and [provenance log](evidence/provenance.log).
