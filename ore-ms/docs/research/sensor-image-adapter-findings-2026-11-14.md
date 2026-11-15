# Four-control thermal image adapter: implemented and independently qualified

Continuation: [four-arm estimator comparison completed](sensor-estimator-findings-2026-11-14.md). This adapter note records the earlier image-only qualification.

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus frozen working sources.
**IMAGE_ADAPTER_QUALIFIED; NO_LEARNED_PREDICTIONS.**

## Delivered artifact

The adapter produces four distinct image-formation controls from one known
continuous scene/rig/trajectory: instantaneous sharp, thermal memory only,
rolling readout only, and both. Nine texture/motion cases contain144individual
stereo views. Every arm has the same explicit reference-time pose labels and
shared support metadata. It emits no purported exact flow label for an integrated
pixel. This completes the implementation requested after the
[sensor-time contract](sensor-time-contract-decision-2026-11-14.md).

It is our controlled planar-image adapter, not TRGS-SLAM/NeRF code execution,
a new sensor model, a physical MS2 calibration or an estimator accuracy result.

## Fixed scene, virtual profile and timeline

Reuse the three existing day/night/rain thermal textures, prepared once by the
pinned historical native recipe and interpreted as static unit-bounded radiance
proxies on a plane4m ahead of a canonical optical camera. Bilinear spatial
sampling occurs before temporal filtering;8bit quantization occurs once afterward.
These proxies are not measured scene temperatures or a reproduction of the real
raw-radiance-to-CLAHE sensor/preprocessing pipeline.

Virtual raw/prepared grid256×640, identical pixel coordinates, identity lens
distortion and no resize. Borrowed optical K/stereo baseline are held fixed.
The source-inspired profile uses8ms thermal time constant,27.8microseconds row
period and0.5ms trigger/readout delay. Those values are declared synthetic inputs,
not measured parameters of the MS2 camera or a complete reproduction of the
source camera. There is no FPN, added random noise or learned correction.

Reference times are0.2s and0.3s, defined as the center of sensor readout. The
corresponding trigger is reference−delay−readout_span/2. Per-pixel readout offsets
span−3.558378ms to+3.558378ms; both stereo cameras evaluate the same continuous
rig at each pixel's own time. Sharp/memory-only controls use the common reference
time everywhere. Rolling/both controls use the per-pixel time field.

Three fixed motions:

- Stationary: identity rig pose at all times.
- Constant body twist: body velocity[.3,0,.1]m/s and optical Y rate.1rad/s,
  integrated on SE(3), not independent world-linear translation.
- Accelerating: world center[.3t+.3t²,0,.1t+.1t²]m and Y angle.1t+.2t²rad.

All trajectories are analytic throughout the required history. The memory
kernel integrates the preceding64ms (eight time constants), with finite-history
normalization and discrete DC preservation. For unit-bounded irradiance, the
omitted infinite-history contribution changes intensity by at mostexp(−8),
about0.0855grey codes; this finite-history model remains explicit.

| Motion (three textures each) | Reference translation, mm | Reference rotation, deg | Shared pixel support |
|---|---:|---:|---:|
| stationary | 0.0000 | 0.00000 | 94.26% |
| twist | 31.6226 | 0.57296 | 90.18% |
| accelerating | 47.4342 | 1.14592 | 88.06% |

Pose targets are `inverse(world_from_left(t0)) @ world_from_left(t1)` and are
identical across all four controls. A common pose timestamp does not assert that
the integrated image equals an instantaneous image at that timestamp.

## Numerical integration and support

Predeclared Gauss–Legendre refinement was32→64→128→256nodes, stopping at the
first higher grid satisfying0.5grey-code maximum and0.05code RMS difference
on common supported pixels.64nodes qualifies against32:maximum0.30023197codes,
RMS0.00298835codes over22,142,619supported intensity values. No image/trajectory
parameter was selected from neural outcomes; no neural inference occurred.

Per-arm/view support requires positive ray-plane intersections and a conservative
one-pixel atlas margin at sampled history nodes and endpoints. Shared masks
intersect all arms, with a further conservative coordinate intersection over
stereo/time views. This sampled support is explicit; it is not a natural-scene
occlusion oracle or measured-flow truth. A future estimator comparison must bind
how common synthetic eligibility is applied and preserve its denominators.

All full-frame images and continuous values remain saved, including finite-atlas
boundary behavior. Independent pixel checks include the fixed sample locations
whether or not a particular view's support is valid. Convergence metrics refer
to the declared supported pixels, not a rigorous full-image integration bound.

![Image controls on the day texture](../../results/sensor-image-adapter/image-controls.png)

The difference plot is a rendering illustration, not a pose improvement result.
Its display clips differences to±10grey codes without changing any image or
qualification metric.

## Independent qualification

The independent audit passes7,229checks, including3,456fixed pixel/arm/view
comparisons. It reconstructs inverse-K rays, plane intersections, independent
Rodrigues/SE(3)poses, manual bilinear texture values, adaptive-time integrals,
reference-time/stereo pose labels, shared masks, selected full-pixel convergence
statistics and one-time quantization. Stationary images match bitwise across
all arms; zero-memory and zero-readout software limits also pass.

The largest sampled difference from adaptive quadrature is0.47857717grey codes,
within the frozen0.5code bound. Adaptive quadrature reported roundoff warnings
at its requested1e−9internal radiance tolerance. A separately source-bound
precision supplement retained all1,728memory-integral diagnostics, including
160warning cases. Maximum estimated reference integration error is0.00007413
codes; maximum image difference plus that estimate is0.47857743codes, still
within the same bound. The reference-error ceiling0.001code was bound before
this supplement; no rendering/acceptance tolerance changed. Estimated quadrature
errors are numerical diagnostics, not a proof of a uniform mathematical bound.

## Retained implementation failure

The first producer computed the32/64comparison but stopped when writing a
NumPy boolean to JSON. The original source snapshot, failure log and partial
JSON remain in`render-001/`. Freeze002 changes only that output flag to a Python
boolean. Numerical convergence values and population repeat exactly, and the
complete result is`render-002/`. No physics, integration grid, motion, texture or
tolerance change was made to rescue an outcome. The original passing auditor
remains unchanged; the reference-precision diagnostic is separate.

## Paper relationship and next boundary

[TRNeRF](https://umautobots.github.io/trnerf) and
[TRGS-SLAM](https://arxiv.org/html/2603.20443v1) already supply the underlying
microbolometer modeling context. This adapter provides a controlled evaluation
input/pose contract for our thermal frontend work, not an established method
contribution. No new author-paper or full-system code reproduction was performed.

The immediate next step is a separately frozen four-arm native-estimator
comparison, including how the common synthetic support is admitted. Report all
planned stationary/moving attempts, translation/rotation error and uncertainty
readouts separately; do not infer covariance calibration from this deterministic
profile or label integrated-pixel disagreement as exact optical-flow error.
This adapter alone does not establish that the existing model fails, needs a
new head, or that any published result is wrong. No severity search follows.

## Artifacts

Sources:`tools/sensor_image_adapter.py`, `tools/audit_sensor_image_adapter.py`,
`tools/audit_sensor_quadrature_precision.py`.
Freezes:`experiments/sensor-image-adapter-freeze[-002]-2026-11-14.json` and
`experiments/sensor-image-reference-precision-freeze-2026-11-14.json`.
Outputs:`results/sensor-image-adapter/` (original failure,144views/continuous
values, coarse values, masks/pose targets, scalar replay check, two audits,
precision diagnostics, CSV and figures). Local notebook remains excluded from
repository/backup.
