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

## Fresh passive current-view evidence — 2026-11-17

Two passive captures each returned20ABSENT observations from the running default
scene; a saved frame was visually inspected. The first log retained only a pairing
Boolean. The second uses `rrm/camera_pairing.py` to retain all three original
headers, dimensions, encodings, calibration, per-channel receipt times and hashes
for saved PNG/NPZ detector inputs. Empty/mismatched frames, invalid source stamps,
dimension mismatches and non-32FC1 depth are rejected. Four dependency-light tests
with adversarial subcases pass. This does not impose camera/depth requirements on
the embodiment-independent reasoner or qualify renderer synchronization.

The [curated sample](uav-results/passive-default-20261117/README.md) retains all20
metadata records and three raw detector-input pairs in Git. The
[audit](uav-results/passive-camera-audit-20261117.json) checked all20headers and
independently recomputed all3saved pairs:ABSENT. The other17raw observations were
not saved and cannot be independently recomputed. Source span0.570s; maximum
channel receipt span0.01517wall seconds. Receipt alignment is not acquisition
synchronization, and ABSENT in the current view does not establish scene-wide
semantic absence. No flight, motion request, scene change or model inference.

## Initial total-spread profile — 2026-11-17 (superseded default)

A connected red component can contain incompatible ranges without RGB ambiguity.
Baseline split-depth and central-outlier cases still returned OBSERVED. The detector
at this earlier checkpoint required valid interior depth spread at most 0.1 m by default, configurable by
the positive finite `max_depth_spread_m` fixture profile. This uses the same numeric
scale as the existing temporal viewpoint depth-stability bound; it is an authored
conservative fixture setting, not a learned threshold or real sensor noise model.

The detector records min/max/spread and the declared limit. Excessive spread returns
DEPTH_INCONSISTENT with no surface point, and the existing viewpoint policy abstains.
The hard min/max rule intentionally rejects even one large valid-depth outlier.
Sloped/curved genuine objects may also abstain; this path remains restricted to the
static frontal fixture. Partial connected occlusion with coherent depth can still
be OBSERVED/SELECTED and is not claimed solved.

The [expanded synthetic record](uav-results/marker-depth-consistency-20261117.json)
matches all 17 authored dispositions, including coherent/excessive gradients,
split depth and one outlier. Original 13 case dispositions remain unchanged;
46 focused tests pass. All 34 saved input and three source hashes were checked.
No physical flight or real sensor qualification. The earlier passive-sample audit
must still use its matching detector revision (`230348184`); its source hashes and
historical results were not rewritten for this new detector.

## Static-window spatial consistency — 2026-11-17

The viewpoint policy now requires pairwise bbox intersection-over-union at least
0.5 across all three observations, alongside existing area, depth, centering and
standoff checks. The declared threshold is an authored conservative static-camera/
static-fixture setting. A location jump can no longer pass merely because area
and range agree. The minimum overlap is reported on selection and spatial rejection.
Small jitter and the exact 0.5 overlap boundary pass; disjoint windows and windows
whose extremes disagree abstain without a goal.

This is spatial consistency, not semantic tracking, motion compensation or proof
that the same physical object persisted. Identical-looking replacements at the
same image position remain undetected. Coherent partial occlusion still can pass.
Moving-camera or moving-target use is outside this restricted policy's qualification.

The [spatial campaign](uav-results/viewpoint-spatial-20261117.json) passes 20 authored
dispositions with the original 17 unchanged. Its three spatial cases each retain
three distinct authored PNG/depth inputs; they are not timestamped acquisitions.
All 52 input and three source hashes were verified. Focused regression: 52 tests
pass, including existing raw-viewpoint assessment. No scene change or flight.

## Prepared passive scene variants

`uav_marker_fixtures.py` defines six source-controlled scenarios for the existing
marker launcher: nominal, absent, duplicate-red, split/partial/full occlusion.
Nominal geometry/colors are preserved. All fixtures remain static colliders outside
the authored 2 m flight radius; this geometric placement is not contact qualification.
Occluder coverage is checked analytically against a declared reference camera,
not treated as actual camera truth. Intended detector labels are evaluation metadata
and are never passed to the observer. Manifests explicitly say `rendered_qualified=false`.

Six tests pass using the installed USD 24.5 libraries in a separate process and
private in-memory stages, with no Kit application or live stage access. They cover
world bounds, shaders, material bindings, colliders, pre-Kit rejection of invalid
configuration, nominal compatibility and refusal to overwrite manifests. CLI dry-run
and environment passthrough for both service definitions pass. These variants have
subsequently received the private static-render checks below; physical execution remains unqualified.

## Local depth support and private rendered controls — 2026-11-17

The first rendered nominal cube falsified the total-spread default above: its front
and underside span 0.402 m despite representing one continuous surface. The current
default instead requires one four-connected valid interior depth patch and no
four-neighbor depth step above 0.1 m (`max_depth_step_m`). The numeric limit was
retained; total min/max/spread remain diagnostic. An explicit positive finite
`max_depth_spread_m` still selects the stricter total-spread profile; its default
is now `None`. Missing-depth gaps separating support return DEPTH_DISCONNECTED.
Both discontinuity statuses withhold the surface point and viewpoint goal.

The [saved private renderer evidence](uav-results/private-static-20261117/README.md)
contains six cases and 18 separately rendered frames, with RGB/depth/K, projection
metadata, stages, fixture definitions, baseline source and a replay audit. All 54
raw input hashes and intrinsics against the renderer projection were checked.
Nominal now observes/selects; absent/full cover abstain as ABSENT; duplicate/split
abstain as AMBIGUOUS; partial cover abstains on its measured local depth jump.
Partial connected occlusion with coherent depth can still pass, as the synthetic
case explicitly demonstrates. No visibility or identity inference was added.

The [updated synthetic campaign](uav-results/marker-local-depth-20261117.json)
passes 21 dispositions. The old excessive_slope input now passes as
continuous_wide_slope: wide continuous slopes are intentionally allowed. A missing
stripe hiding a depth jump is newly rejected. The older 17/20-case reports retain
their original source hashes and results. Current focused tests:55 PASS; full core
unittest outcome is recorded in the handoff. These are development controls that
informed the revision, not held-out accuracy or robot-camera qualification. The
0.1 m pixel-neighbor setting depends on resolution/view and can reject steep real
surfaces. Real noise, RGB/depth edge mismatch and general objects remain unqualified.

## Marker window freshness at submission — 2026-11-17

The probe now validates three strictly increasing positive source stamps, fixed
camera calibration/frame/size and every channel's receipt within the same observation
window. Freshness uses same-process monotonic time, with a strict age below two
seconds. It is checked at decision and again after action-server discovery, before
marker navigation submission; a five-second discovery wait cannot reuse an expired
window. Existing source-to-odometry and state/actuation checks remain in place.
These checks do not authenticate the source clock or prove synchronized acquisition.

Clock-controlled regressions cover expiry during server discovery, per-channel
old/future/pre-window receipts, duplicate/reversed stamps and malformed/changed
metadata. Focused camera/marker/viewpoint tests65PASS; full core685PASS. New source
has not executed a flight. The [passive receipt-window record](uav-results/passive-window-20261117/README.md)
contains20triples,18complete recent windows, and3independently recomputed raw pairs
(all ABSENT in the current default view). Maximum recorded window receipt age0.132s.
The other17raw images were not saved. Saved monotonic times support replay of that
check only, never freshness or motion permission in a later process.
