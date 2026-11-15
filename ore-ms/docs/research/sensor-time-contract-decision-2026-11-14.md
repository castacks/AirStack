# Next direction: thermal sensor time support and a defensible pose target

Continuation: [image adapter implemented and qualified](sensor-image-adapter-findings-2026-11-14.md). The historical readiness status below describes this note at creation.

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus frozen working sources.
Contribution decision and source/math readiness complete. No new image rendering,
learned inference, training, calibration fit or physical accuracy experiment.

## Decision across the original-paper gaps

Select **sensor-aware evaluation for thermal supervision/uncertainty** as the
next technical direction, connecting MACThermal T2/T3 and MAC-VO V4. First qualify
an image-formation/pose-target adapter; do not train a new head yet. This is a
feasibility/interpretation choice, not the only remaining question or an
established novel method. No actual model failure under thermal memory/readout
has been demonstrated by our current experiments.

| Direction | Current evidence and decision |
|---|---|
| More anchor gates or harder negatives | Existing selector already separates the fixed constructed set. Added cycle loses a valid association. Close that recipe; no severity/threshold search to force failure. |
| Thermal temporal image formation | Prior half-LSB/gamma/instantaneous-atlas assays do not model thermal memory or per-pixel readout. Published image model and source are available; qualify this distinct measurement/target regime next. |
| Thermal learning/representation | Still open. ThermalSfMLearner/raw-patch/small-weight configurations do not exhaust it. Generic self-supervision/16bit features already have substantial prior art. |
| Continuous persistent estimation | Reference choice and bias/variance matter locally; neither generic BA nor fixed-anchor recovery is new. A specific state/dependence mechanism is still needed. |
| Sensor transfer/active perception | Distinct paper openings remain. Broader sensor inputs/planner source and task-level evidence require their own qualification; this review does not declare them exhausted or untouched. |

## Primary sources and established overlap

[TRNeRF](https://openaccess.thecvf.com/content/WACV2025/html/Carmichael_TRNeRF_Restoring_Blurry_Rolling_Shutter_and_Noisy_Thermal_Images_with_WACV_2025_paper.html)
(WACV2025) and [TRGS-SLAM3.1–3.3](https://arxiv.org/html/2603.20443v1)
already model the temporal response, rolling readout and fixed-pattern offsets
of microbolometer images. Therefore generic thermal-aware rendering, correction
or adding blur is not our novelty. TRGS-SLAM also estimates continuous trajectories
and performs SLAM under those effects; do not imply no successful methods exist.

Fresh depth: TRGS3.1–3.3 image model and five pinned author renderer/FPN/data files;
TRNeRF official project rendering/evaluation overview and indexed image-model
passage. Direct full CVF PDF retrieval returned403, so do not label this a fresh
full TRNeRF paper/figure reading. No author code imported/executed. The older
TRGS mapping/limitations review remains separately recorded.

The official TRNeRF project distinguishes Aprilgrid-based restoration evaluation
from pseudo-ground-truth imagery. TRGS data/reference requirements likewise do
not create independently measured thermal trajectory truth for our MS2 prefixes.
No new dataset payload was downloaded in this review.

## Timing/measurement contract

For a static scene viewed along a time-dependent ray, let p(u,v,t) be sharp
irradiance in the raw sensor grid. The ideal first-order response is

    m(u,v,r) = integral_0^infinity exp(-a/tau)/tau * p(u,v,r-a) da.

Readout time r varies across pixels:

    r(u,v) = trigger + delay + u * row_period/width + v * row_period.

An additive fixed-pattern component is a separate term. The thermal response
integrates previous irradiance rather than one instantaneous point. Distortion,
resizing, initial history and the meaning of the output pose timestamp must be
specified before numerical image/pose evaluation. Gamma changes and independent
pixel noise do not emulate that temporal support.

A blurred/rolling image generally has no single exact instantaneous optical-flow
field. Do not use a sharp-image warp as certified measured-pixel flow truth or
train on every resulting discrepancy as a correspondence error. A pose target
can instead be defined at an explicit camera/body reference time; its convention
must be the same for all estimator arms and their labels.

## Why the next trajectory must be chosen carefully

The exponential response has mean sample age tau and second moment2tau².
For a radiance signal locally affine in time, the integral equals its value at
r−tau. For quadratic temporal radiance the integral differs from that single
sample by a second-order term; a generic constant timestamp correction is not
an exact image-formation solution.

Separately, if T(t)=exp(t*Xi) has constant body twist, then

    inv(T(t0−d)) * T(t1−d) = exp((t1−t0)*Xi)
                           = inv(T(t0)) * T(t1).

A common effective timestamp shift can therefore disappear from the relative
pose target under that motion. This is an algebraic property, not a claim that
blurred pixels exactly correspond to delayed poses. It does not show that all
blur/rolling effects cancel. Nor do we retrospectively assert that our earlier
linearly translating/rotating sequences were exactly constant-body-twist motion.

With acceleration, the relative-displacement change need not cancel. A meaningful
new timing assay must contain a time-varying motion segment and an explicit pose
reference time, while keeping a constant-motion segment as a null/control.
These are established filtering/rigid-motion facts, not new mathematics.

## Source parameters and numerical qualification

Author source commit:
`07ba901afb9d9296f2e4399240d9fd4dc6d2398c`.

Targeted inspection covers renderer offsets/integration weights/DC correction,
FPN spline parameterization, TRNeRF image timestamp/calibration configuration,
and the base dataset's row-period/downsampling relationship.

The dataset-config defaults are thermal time constant8ms, trigger-to-readout
delay0.5ms and full-resolution row readout27.8microseconds. These are that source's
profile defaults, not measured MS2/FLIR parameters. Its base dataset scales row
period with downsampling; copying a full-resolution row period onto our resized
images without adapting the raw-grid timing would change the physical model.

The author renderer approximates a finite history using temporal raster weights
and brightness correction. Our25checks qualify analytic response identities,
not that full renderer or its discretization. They compare closed formulas with
independent numerical quadrature and matrix exponentials: response moments,
step response, finite-history DC normalization/mean age, affine/quadratic temporal
signals, constant-twist invariance, acceleration contrast and readout strides.

An eight-time-constant omitted tail has mass0.00033546; truncation, normalization
and startup history must remain explicit. In the illustrative unit-acceleration
translation fixture, an8ms shared time shift changes relative displacement over
a0.1s interval by−0.8mm. This is a mathematical example, not an observed camera,
neural pose error or measured lag. No severity was selected from model scores.

## Finite next deliverable

Question: **do the native pose estimates and uncertainty readouts retain their
meaning when the images have known temporal support, with a fixed reference-time
pose target?** This is a baseline-evaluation hypothesis; no correction selected.

Prepare and qualify one continuous-time scene/rig/target adapter with four fixed
arms: instantaneous sharp images, thermal memory only, rolling readout only,
and both. Hold the latent scene, continuous trajectory, camera rig, intensity
mapping, model, selector and solver fixed. Include stationary/constant-twist
nulls and one prespecified time-varying trajectory. Do not add FPN or noise until
these temporal effects and their history/integration target are qualified.

Before inference bind the source-specific synthetic profile or independently
measured profile (clearly label which), raw-to-prepared coordinate/time mapping,
common reference timestamp, past-history initialization, integration accuracy
checks, shared valid rendering support and every planned pose attempt. Sharp
instantaneous latent geometry remains useful for renderer qualification; it is
not automatically the flow label of the integrated observations.

Only after this adapter passes should one finite model comparison be frozen.
Report pose error/availability and uncertainty ranking with complete denominators;
no physical covariance claim from synthetic profiles. If the existing estimator
also succeeds here, preserve that result and close the exact assay. Do not
increase motion/severity until a failure appears or infer a need for new training
from the mere presence of a sensor model. Any method intervention still requires
its own prior-art distinction and controlled evidence.

## Artifacts and status

Status: **SOURCE_AND_TARGET_MATH_QUALIFIED; IMAGE_ADAPTER_NOT_IMPLEMENTED**.
This completes the next contribution/readiness decision. It is not a learned
accuracy result or an executed four-arm sensor simulation.

Freeze:`experiments/sensor-time-contract-freeze-2026-11-14.json`.
Math/source metadata:`results/sensor-time-contract/`.
Tool:`tools/qualify_sensor_time_contract.py`.
Author snapshots/paper text remain outside repository/backup; only their hashes
and precise reading history are retained.
