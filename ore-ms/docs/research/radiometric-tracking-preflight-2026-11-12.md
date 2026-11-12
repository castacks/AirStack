# Raw thermal tracking: bounded feasibility check

12 November 2026, America/New_York. Start from recovered commit `dc96e91c5`.
This is a new temporal correspondence-mean hypothesis, not a rescue of the
closed TartanMatch/NEC, fixed-covariance or patch-gap confidence configurations.

## Primary-source basis and reading limits

- [TP-TIO, section III-C](https://arxiv.org/html/2012.03455v1): inspected the
  tracking equation and surrounding method. It uses full 16-bit radiometric
  samples, patch-mean normalization, SE(2) alignment, pyramids and an IMU prior.
  Our prototype retains only a normalized patch residual and fits translation.
  No ThermalPoint, IMU, inverse-compositional solver or paper reproduction is
  claimed. Synthetic translation tests do not reproduce its experiments.
- [Self-TIO publisher abstract](https://ieeexplore.ieee.org/document/10803066/):
  describes hybrid learned-flow/KLT tracking. The [authors' dataset repository](https://github.com/junwoonlee/self-tio-dataset)
  explicitly states they have no plans to publish the network/code. Verified
  repository contents and availability statement; full article not inspected.
  A faithful executable Self-TIO baseline is therefore not supplied here.
- [KLTNet, sections III-A/B](https://arxiv.org/html/2608.24544v1): inspected
  dense-flow initialization and learned triplet-patch refinement. Its fixed
  reference patch, learned refiner and observation weights exceed this pairwise
  prototype. No author implementation/checkpoint readiness or thermal transfer
  was established. It supports separating initialization from local refinement,
  not a claim that our classical aligner inherits its performance.

These sources motivate a testable mechanism; they do not establish why our
existing thermal errors occur or prove local radiometric tracking will help.

## What changes and why this is distinct

Hypothesis: retaining raw thermal contrast for a local endpoint correction can
improve precision relative to the existing processed-image flow. The original
MAC mean initializes the correction; the closed pretrained replacement is not
used. EXP027/030 used LK disagreement as a diagnostic on processed images;
they did not test raw-uint16 refinement of original MAC endpoints as an
intervention. The closed patch-gap work concerned confidence/selection, not
this translation-only photometric fit. This distinction warrants a feasibility
check, not automatic promotion to another accuracy experiment.

The [pre-execution freeze](../../experiments/radiometric-tracking-preflight-2026-11-12.json)
fixes a 15×15 patch, correction box ±2 pixels per coordinate, 30 function
evaluations, minimum raw standard deviation 1 code and minimum Jacobian
singular-value ratio 0.001. Interpolate float64 copies of original uint16
samples; never quantize the interpolant to uint16. Minimize patch-mean-normalized
intensity differences, scaled by the source mean/std for conditioning. This
is multiplicative-intensity invariant, not invariant to arbitrary additive
changes, nonuniformity correction, occlusion or local deformation.

Keep every attempt and reason for rejection: borders, nonfinite coordinates,
flat texture, aperture ambiguity, exhausted solve, correction limit or invalid
target. A bounded solution and lower residual are not correspondence truth.
There is no validity-conditioned success rate presented as tracking accuracy.

Before execution, 16 tests pass, covering known integer/subpixel translations,
intensity gain, low contrast lost by a fixed 8-bit shift, input preservation,
aperture/flat/border failures, incorrect large prior and zero target patches.
The initial low-contrast fixture straddled two 8-bit codes; its DC level was
corrected to center the unchanged texture inside one code. No algorithm or
real-data result was changed in response.

## Fixed existing-data check

Use only pair 000000→000001 of each exposed day/night/rain 100-frame prefix,
all 585 original grid coordinates and original MAC endpoints. Verify the six
uint16 image hashes against source manifests and the three packet hashes
against archived acquisition records, then bind their hashes and this code.
No model inference, matcher call, pose optimization, reference-value read,
score replay or parameter search is part of this check.

## Integration constraint and subsequent decision

`Odometry/MACVO.py` selects `kp0_uv`, samples dense flow to form `kp1_uv`,
checks inbound coordinates, then samples target depth and covariance. A proper
future sparse hook belongs after initial `kp1_uv` construction and before
inbound filtering/target-depth lookup. It must pass original raw images
alongside the processed model input without altering existing inference.
Sparse refined points must not be broadcast into a dense flow map or silently
change the motion initializer/point selector.

The original flow covariance is not calibrated for refined endpoints. Any
future pilot must state its covariance control and retain rejection/coverage
costs; dropping difficult tracks cannot manufacture a gain. A no-op hook must
reproduce original outputs before an intervention is run. No production hook,
model training, full-motion integration or motion accuracy result is implemented
in this preflight.

Stop at mechanical feasibility. Before a downstream trial, lock a native
original/passthrough/refined/RGB comparison on the same exposed three-record
prefixes, retaining all nominal intervals. Require improved motion mean/RMSE
against thermal without rotation or coverage regression per recording; any
claim of improvement over RGB also requires beating RGB per recording. Patch
cost and accepted counts cannot replace those endpoints. Independent real
flow error remains unavailable; synthetic truth establishes only mechanical
correctness. No new image upload is needed for this preflight.
