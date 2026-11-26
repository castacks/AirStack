# RGB radiative transfer

The 24 matched slab comparisons span three wavelengths, four optical depths per nominal 550 nm slab, and two incident illumination levels. The independent QUADPACK path integrates local emission at its observer-side optical depth. The largest residual across transmission, direct and path spectral radiance was 1.11e-16. Spectral extinction matched at all three wavelengths. The transparent limit retained boundary radiance and removing illumination eliminated scattering source. M01 legacy transport is also tested directly. [RGB data](evidence/rgb_checks.json) · [M01 data](evidence/legacy_checks.json)

At 50/200 electrons, decoded means were 49.985/200.052 electrons and variances were 58.93/207.31 e² against 59/209 e². The detector was evaluated on 20,000 independent draws per level. Rail checks covered black, interior, exactly full well and over-range. This verifies the Gaussian shot/read model and quantizer within the sampled range.

The lossless two-stream comparator conserves energy to 1.11e-16. At optical depths 0.1/1/4 its reflected fractions were 0.0476/0.3333/0.6667, compared with the corresponding single-reversal fractions of 0.0453/0.2162/0.2499. Multiple scattering changes the answer materially in this discrete challenge. It does not establish an optical-depth operating limit for real aerosols.
