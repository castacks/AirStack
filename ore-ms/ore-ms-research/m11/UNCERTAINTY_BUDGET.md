# Discrepancy and uncertainty budget

[Prospective tolerances](VERIFICATION_PLAN.md) were committed before comparison. The table gives the interpretation of each contributor and its observation scale.

| Contributor | Accounting |
|---|---|
| Floating point and quadrature | Double precision. RGB slab absolute tolerance 1e-11 in dimensionless transmission or per-nm radiance. Planck relative tolerance 1e-12. Use absolute crossover residual when the contrast denominator vanishes. QUADPACK epsabs/epsrel 1e-11 and explicit knots. Band tolerance in W m⁻² sr⁻¹. |
| Spatial and temporal discretization | Pixel-center masks use inclusive polygon edges. Range tolerance measured in metres. Thermal trapezoid checks use 12 through 96 intervals. Advected midpoint segment bias is tracked separately. Compare Euler variance bias with an exact integrated OU transition in latent units squared. |
| Stochastic sampling | Detector noise uses 20,000 samples per level. OU uses 24,000 independent trajectories, each with linked temporal samples. Estimation and forecast branches reuse issue states intentionally. Coverage uses Wilson intervals and CRPS uses realization-level Monte Carlo standard errors. Fusion bootstrap clusters before resampling and has 48 independent scenes. Swept offsets and arms within a scene are correlated. |
| Parameter and calibration uncertainty | Nine equally weighted tau/SD combinations provide a sensitivity budget, without claiming a fitted parameter distribution. Apply total variance (E[Var(Y\mid\theta)]+Var(E[Y\mid\theta])) at fixed initial state. Innovation-amplitude variation changes conditional variance. Wrong-air and frozen fusion-bin transfer are measured distinctly. |
| Model-form discrepancy | Two discrete scattering directions bound no physical operating regime. Absorption-only thermal transfer, periodic OU, Gaussian code likelihood, and logit-normal learned output each delimit the claim. Mismatched intervals can narrow as coverage degrades. |
| Sensor and geometry mismatch | Pixel offsets are integer translation fixtures. Diagonal `(d,d)` has Euclidean magnitude `sqrt(2)*d`. Rectangular occlusion probes and synthetic contrast do not measure real camera alignment or partial-pixel response. |
| Unexplained residual | Keep measured residuals and all failed or low-support predictions. [Discrepancy register](DISCREPANCY_REGISTER.md). |

Exact tolerances for the frozen controls do not translate into aerosol, temperature, pose, or deployment error bars. Width and coverage are reported together. Structural range/extinction ambiguity persists even as numerical and sampling errors approach zero.
