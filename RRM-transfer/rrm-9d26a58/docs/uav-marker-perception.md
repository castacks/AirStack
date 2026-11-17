# Restricted UAV marker perception

Date: 2026-11-15. Simulation-only development, no Cosmos or model weights.

`simulation/isaac-sim/launch_scripts/uav_marker_qualification.py` subclasses the shared PegasusApp. It adds static red and blue 0.6 m cube fixtures at (4,0,1.2) and (4,-1.5,1.2), outside the bounded 2 m flight radius. These represent mounted fixtures, not free bodies. Colliders remain enabled. `UAV_FIXTURE_MANIFEST` optionally records authored identities/positions and scene-source hash for separate assessment. The observer never reads this manifest.

`rrm/uav_marker.py` detects red components from RGB only. No red returns ABSENT; multiple qualifying components return AMBIGUOUS. It erodes the component boundary and requires sufficient finite positive depth. Missing depth stays DEPTH_UNKNOWN. It selects an actually observed valid pixel rather than inventing the independent median x/y ray through a mask hole. Calibrated pinhole projection returns a camera-relative surface point, not object center or map coordinates.

`scripts/uav_marker_capture.py` is passive: no task services/actions or actuation publishers. It requires exact RGB/depth/camera-info source stamps, matching frames/dimensions and explicit 32FC1 simulated-depth units. It stores source hashes, observations and representative PNG/depth NPZ records. Run inside the robot overlay with explicit domain1 and optional dependencies numpy/cv2/CvBridge. Keep the source overlay's PYTHONPATH when adding the RRM import path.

Initial live capture:19/20 frames OBSERVED,1 AMBIGUOUS; preserve the abstention. Following the observed-pixel correction,20/20 frames OBSERVED in the same development scene. The prior empty-grid camera frame returns ABSENT. These are development cases, not a held-out accuracy distribution. Thirteen synthetic tests pass, including ambiguity, unknown depth, invalid calibration and mask holes.

The simulated `depth_ground_truth` stream is oracle-like range evidence; this result does not qualify stereo-derived depth or a real sensor. Optical z depth differs from Euclidean range and from the fixture's center. Exact header pairing does not prove renderer acquisition synchronization. General recognition, occlusion, displaced targets, tracking, world-frame binding and semantic task completion remain open.

See [UAV execution](uav-task-execution.md) for public-action safety gates and physical evidence, and [capability matrix](uav-task-campaign.md) for scoped task dispositions.

An additional marker-scene flight completed takeoff, a predeclared0.75m viewpoint, physical settling, fresh image, landing and disarm. Concurrent passive capture retained2,000 OBSERVED exact-stamp pairs. The saved inspection image has a matching RGB/depth/calibration observation at source stamp157.099996488s, red-surface optical z2.72554m (preflight sample3.53891m). This demonstrates bounded target observation at a benchmark viewpoint; the target was not world-localized by the detector and no learned inspection semantics were exercised.

## Observation-conditioned viewpoint checkpoint

`rrm/uav_viewpoint.py` selects a single horizontal move from three fresh exact-pair observations and the camera optical-forward axis transformed into map at the selected image stamp. It abstains on unknown/ambiguous, cropped, unstable, off-center or invalid inputs and insufficient predicted gain/standoff. Maximum translation0.75m, predicted optical surface standoff at least2.5m, desired frontal-pinhole bbox-area ratio1.5. This is a static-fixture resolution proxy using oracle depth, not semantic information gain or object-center/world localization.

One declared flight passes: planned translation0.635471m, independent capture displacement0.554962m, median bbox-area ratio1.407407, optical surface depth3.465420→2.904359m. Acquisition TF, all six PNG/depth/calibration records and independently recomputed detector outputs agree with the logged decision and commanded goal. Capture physical speeds0.01755/0.02291m/s; full envelope/contact/navigation/LAND/disarm checks pass. Fixed calibration rules out attributing this result to a changed focal length. General recognition, occlusion and robustness remain unqualified.

Use `--marker-viewpoint --execute-simulation` with the existing probe and actual RRM package on PYTHONPATH. It is exclusive with other route modes. `scripts/assess_uav_viewpoint.py` binds the raw frames, policy source, acquisition transform and goal to a matching independent contact/truth assessment; action success alone cannot earn PASS.

## Offline abstention development — 2026-11-17

`scripts/uav_marker_abstention_campaign.py --output <new-directory>` creates a fixed
13-case synthetic calibrated RGB/depth intervention campaign. Run with the RRM
package on PYTHONPATH and NumPy/OpenCV/Pydantic installed. It has no ROS dependency
or actuation path. Each new output retains PNG/NPZ inputs, source snapshots, SHA256
hashes, expected dispositions, actual detector/policy outputs and limitations;
existing output directories are refused. Missing/bad evidence is never repaired
into an observation. Three repeated observations test disposition propagation,
not independent acquisition or temporal tracking.

All 13 authored disposition checks passed: nominal SELECTED; duplicate/split red
components AMBIGUOUS with no goal; absent/full cover ABSENT with no goal; zero,
nonfinite, negative, out-of-range and sparse depth DEPTH_UNKNOWN with no goal;
cropped target observed but policy ABSTAIN. **Partial connected occlusion remains
OBSERVED/SELECTED.** The expected result deliberately retains this limitation:
color segmentation alone has no independent full-visibility or semantic-identity
model. Passing this campaign does not qualify arbitrary occlusion detection,
physical fault handling, general recognition or learned reasoning.

Malformed policy containers and fields now return explicit ABSTAIN/no goal rather
than TypeError/AttributeError; tuple and list payloads behave consistently. The
detector itself and nominal geometric policy are unchanged. Focused detector,
viewpoint, persistence and existing raw-viewpoint-assessor regressions: 47 passed
inside robot Docker. Original flight source hashes remain distinct from this
updated policy; do not reassess old flight evidence as source-matched implicitly.

Restored-workspace discrepancy: historical UAV raw captures are absent from the
checked host artifact directories; the running simulator uses the default scene,
not the historical marker fixture. No historical physical result was reproduced,
no scene switched, and no flight dispatched for this offline development campaign.
