# SmokeRoom supplied-reference relative-pose comparison

16 November 2026. No new model calls. This study consumes all 3,352 locked
SIFT/learned RGB/thermal predictions from the full 838-anchor SmokeRoom recording.
Prior support outcomes are exposed; no pose errors under this contract precede
this protocol. This is a nominal supplied-reference comparison, not independent
physical metrology, full trajectory VO or a complete author-system reproduction.

## Qualification and reference convention

The official sensor-fusion challenge defines trajectories as IMU poses in a fixed
world frame. Use that declared convention for the released integrated-odometry
CSV and camera→IMU calibrations. The CSV omits frame identifiers, and no reference
odometry or TF messages are in the raw image/IMU bags. The source convention and
observed-motion gyro check support a nominal frame contract; they do not certify
reference origin, independent optical truth or exact exposure synchronization.

Before pose scoring: bind every raw image header/time to the exported image and
reproduce its frozen preprocessed NPY hash; verify all retained raw IMU samples
against the supplied IMU CSV. Preserve two CSV IMU samples absent at raw bag
splits and all gaps, not a synthetic repair. Require the prespecified no-fit gyro
frame/timing check to pass on supported half-second moving intervals (≥2° body
motion, ≥20 intervals, median error≤0.5°, p95≤2°, raw IMU gaps≤20ms, reference
brackets≤50ms). Unsupported gyro checks remain explicit. No bias, axis, pose or
clock fit is permitted. This check uses raw IMU also involved in the fused
reference, so it is a convention-consistency test, not an independent truth test.

Preserve original calibration YAML. Construct thermal SE(3) from the supplied
upper3×4 because its stored last row is zero; explicitly set [0,0,0,1]. Normalize
only tiny rotation export error by closestSO(3), maximum Frobenius correction
1e-5, for both sensors. This is serialization/roundoff handling, not calibration
estimated from evaluation poses or images. Stop if those conditions fail.

## Camera-relative reference and whole-span preflight

At each cached source/target image header time, interpolate supplied IMU world
position linearly and orientation with quaternion SLERP. Use an exact supplied
pose when timestamps coincide. No extrapolation; nonexact interpolation brackets
must be≤50ms. Compose T_WC=T_WI*T_IC and target-from-source motion
T_C1C0=inverse(T_WC1)*T_WC0. No world alignment is fitted. Use header times rather
than bag-record receipt/arrival times. Report the parent image association and
all missing reference/support before looking at pose errors.

Retain all original sensor intervals. Define the paired common-reference cohort
before scoring as anchors supported for both sensors. Primary full relative
rotation endpoint: valid cached pose with SO(3) error≤1°. Report≤5° separately,
solver availability, conditional median/p90 error and the always-available
identity-rotation baseline. Missing/rejected poses count as failures. No rescue
from finite rotations rejected by the frozen cheirality gate.

Secondary translation-direction error and joint (R≤5°, directed t≤15°) use anchors
where both calibrated cameras have reference translation≥1cm, declared before
scoring. This avoids treating stationary/noisy translation directions as truth.
All smaller-motion intervals remain in rotation/support summaries and are counted
explicitly. Translation results additionally depend on the nominal reference
IMU origin/lever-arm convention; no independent origin metrology is asserted.

## Conditions and stopping decision

Report the entire mixed recording and the prospectively fixed thirds from the
parent study. Retain the clear, static, dynamic-person and visibly obscured
segments. A third or a global-contrast bin is not a smoke-density ground-truth
label. No favorable window selection. Also report fixed reference-angle strata
<2°/≥2° and the declared translation eligibility, including counts.

All errors are against a supplied fused reference whose paper describes map,
visual, LiDAR and inertial constraints. Report possible reference dependence and
retain all negative/null cases. Freeze protocol, runner, reference manifests,
calibration normalization, source bindings and input hashes after whole-span
preflight. Score only afterward. An independent auditor redoes quaternion
interpolation, rigid reference composition, SO(3)/directed-t errors and all
cohort/threshold arithmetic. Stop after this finite cached comparison; no new
model, threshold, time shift or transform follows from an adverse outcome.
