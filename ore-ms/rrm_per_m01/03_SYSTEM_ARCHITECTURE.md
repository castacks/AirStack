# System architecture

The configuration supplies known geometry, material parameters, atmosphere, spectral response and seeds. The simulator traces the shared pinhole rays, resolves the first opaque surface and passes the material and slant range to the observation model. The observation model returns noiseless radiance. Independent modality-specific PRNG streams generate measured radiance proxies. The CSV records both stages for each modality and carries shared geometry fields.

`physics.py` contains scalar forward kernels. `simulate.py` binds geometry, sensor configuration and CSV output. `verify.py` consumes the frozen configuration and observations and compares the outputs against separate analytic constructions. A future estimator should consume observed fields and calibrated sensor metadata. Geometry and environment truth belong to scoring and simulation.
