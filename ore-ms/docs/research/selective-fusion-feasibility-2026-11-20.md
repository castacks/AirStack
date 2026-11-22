# Selective RGB–thermal fusion feasibility

## What related work establishes

Brunner et al. evaluated sensor data quality before combining visible
and thermal observations in SLAM. The publisher abstract describes
single-modality, unfiltered combination and selective combination
comparisons under recorded smoke and abrupt lighting changes. The full
paper was unavailable from the publisher and institutional repository
during this check. Exact gates, source availability and reference
protocol were not verified.
[Publisher abstract](https://doi.org/10.1002/rob.21464),
[institutional record](https://opus.lib.uts.edu.au/handle/10453/23591).

Khattak et al. describe visible and thermal landmark updates with an
inertial filter and entropy-based selection of image regions. Their mine
trial reports longitudinal endpoint error against a surveyed marker and
compares estimated pose covariance with and without the selection mask.
These endpoints do not establish accuracy throughout a camera handoff.
The paper motivates independent landmark histories feeding a common
state. No released implementation was reproduced.
[Khattak et al., methods and mine experiment](https://arxiv.org/pdf/1903.01656).

## Local source preflight

The existing [stereo factor study](native-thermal-constraints-findings-2026-11-15.md)
already appends thermal residuals to an RGB pose. Its
[long-day continuation](long-day-fallback-findings-2026-11-15.md) yielded
a small position improvement and higher orientation error and solver cost
against a strong RGB control. Repeating the generic idea of adding thermal
factors would overlap that work. The outstanding comparison is whether
retaining camera-specific observations helps the stateful dimming
interval and early recovery on the same estimator and reference.

- **SmokeRoom VINS-Fusion.** The pinned tracker takes the feature IDs
  found in the first camera and associates the second image to those
  points in stereo mode. The estimator uses second-camera observations
  as right-camera factors on those tracks. Supplying thermal as the
  second image would require cross-spectral matches and would not create
  an independent thermal temporal tracker.
  [Pinned feature tracker](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/featureTracker/feature_tracker.cpp),
  [pinned estimator](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp).
- **IRS RRxIO.** The pinned filter exposes a compile-time camera count,
  separate camera calibration paths and a second image callback. Its
  rosbag loader routes decoded camera messages to callback zero.
  The shared image callback resets its batch on a timestamp change and
  queues an update only when every slot is valid. Simply enabling two
  cameras would therefore leave the second slot empty in the loader.
  Routing both inputs still requires a verified policy for paired and
  missing arrivals. The existing released camera shifts differ by 6 ms,
  so an exact timestamp check must precede any pairing decision.
  [Pinned loader](https://github.com/christopherdoer/rrxio/blob/ae29afe0d7876f3838056b457292e4aecf9656be/rrxio/src/nodes/rrxio_rosbag_loader.cpp),
  [pinned callbacks](https://github.com/christopherdoer/rrxio/blob/ae29afe0d7876f3838056b457292e4aecf9656be/rrxio/include/rrxio/RRxIONode.hpp).

## Bounded comparison after input qualification

Use IRS Medium first. Restore the public inputs and verify both native
controls against the prior holdout before testing combinations. Check
all corrected image stamps against the frozen dimming and recovery
intervals, including missing and unmatched arrivals. Establish that
the same body state accepts independently tracked updates from each
camera and logs their source and timestamps. A two-camera configuration
should reproduce single-camera routing with the other camera disabled
before comparing its output quality.

When that gate passes, compare continuous visible, fixed thermal, the
existing hard switch and a shared-state dual-camera arm. Add selective
rejection as a separate arm only after an unfiltered dual control works.
Fix any rejection rule before pose scoring. Keep the known dimming
schedule available as a controlled intervention, with no inference
about natural-degradation detection. Use the existing Medium metric
cohort and early alignment, and report recovery RMSE, corrected
coverage, gaps, processed camera updates and measured cost.
[IRS protocol](irs-rtvi-causal-dimming-2026-11-18.md).

This is a source preflight. It qualifies neither a shared-state
configuration nor a selective-fusion accuracy result.

## IRS Medium timestamp preflight

Restored `mocap_medium.bag` and the companion Vicon–IMU CSV from
the public archive using byte ranges. Both pass the release ZIP
member CRCs. The 1,161,826,000-byte bag SHA-256 matches the
[historical bright replay](irs-rtvi-bright-medium-metric-2026-11-18.md).
The bag contains 1,748 visible, 1,749 thermal and 35,792 IMU messages.
[Author dataset release](https://christopherdoer.github.io/datasets/irs_rtvi_datasets_iros2021).

Read camera header stamps and apply the replay shifts of +2 ms visible
and −4 ms thermal. For diagnostics, bind each visible stamp to the
nearest thermal stamp, with earlier stamps breaking ties. The table
counts unique nearest thermal members separately from nominal thermal
frame counts. It uses the already published alignment, dimming and
recovery windows.

| Window | Visible / thermal frames | Unique nearest thermal | Exact matches | Within 10 ms | Median thermal minus visible |
| --- | ---: | ---: | ---: | ---: | ---: |
| Full bag | 1,748 / 1,749 | 1,747 | 0 | 1,746 | −4.501 ms |
| Alignment | 200 / 200 | 200 | 0 | 200 | −4.501 ms |
| Dimming | 200 / 200 | 200 | 0 | 200 | −4.501 ms |
| First two seconds after return | 40 / 40 | 40 | 0 | 40 | −4.499 ms |
| Later recovery | 160 / 160 | 160 | 0 | 160 | −4.501 ms |

All nearest offsets in the alignment and scored windows fall between
−4.556 and −4.316 ms. The full bag contains two offsets beyond 10 ms
and one reused nearest thermal member. This verifies that exact-time
batching fails even with complete local stream coverage. The 10 ms
diagnostic does not establish physical exposure timing or authorize
replacing individual header stamps with a shared time. The next
routing gate must keep timestamp differences explicit and handle
single-camera arrivals. No estimator was run.

Verified inputs, extraction source, header rows, pairing rows and
summary are under `/root/ore-ms-artifacts/irs-dual-preflight/`.


## IRS Medium component-fusion qualification, 21 November

Restored the pinned RRxIO source, the release bag, the three release
calibration files and the Vicon–IMU CSV. The bag passed ZIP CRC and matches
the historical SHA-256. Native ROS Melodic replays reproduced 0.089 m
visible and 0.248 m thermal position RMSE on 1,237 common scored reference
epochs. In-process odometry capture retained 1,746 visible and 1,747
thermal poses. The historical thermal subscriber trace contained 1,741.

The bag's record order contains 2,814 backward steps in corrected
camera/IMU time. Every camera event in the alignment, dimming and recovery
windows has an IMU bracket at most 2.46 ms wide. One thermal image before
the first IMU is outside these windows. A derived replay bag places events
in corrected sensor order with original header stamps. Independent readback
checks all 39,289 messages and verifies that exactly 200 visible images
were divided by 16, with all other serialized payloads preserved.

A separate two-camera build submits a single valid pyramid per camera
event, protects inactive feature histories and caps both-active feature
allocation at eight visible and seven thermal slots. It disables
cross-camera matching and stereo initialization. Neither cap is applied
during one-camera qualification. Replays used the original bright bag
with only camera slot zero active, retaining each native preset and
time shift. Position scores use the existing 100 ms interpolation cap,
unit scale and a yaw/translation fit over the first 10 s of shared support.

| Camera | Matched output stamps | Common reference support | Native RMSE | Two-camera build RMSE | Maximum output attitude difference |
| --- | ---: | ---: | ---: | ---: | ---: |
| Visible | 1,746 | 1,439 | 0.089 m | 0.207 m | 3.00° |
| Thermal | 1,747 | 1,438 | 0.248 m | 0.253 m | 2.68° |

The visible-only control fails the existing 0.01 m RMSE parity bound
despite retaining all native reference support. A divergence exceeding
the 1e-8 diagnostic tolerance in the pose-vector comparison appears by
the eighth visible output, roughly 0.35 s after the first pose. Startup
also reports camera-1 feature-transform and landmark Jacobian discrepancies.
The Jacobian failures do not establish the cause of visible-only drift.
Both two-camera replays are byte-identical on repeat. Compiling the same
patched source with one camera yields byte-identical traces to the native
controls for both modalities, isolating the observed divergence to the
two-camera configuration under this replay.
The gate stops this shared-state configuration before dual-arm or
component-constrained accuracy scoring.

Source overlays, restored inputs, logs, qualification scores and hashes
are under `/root/ore-ms-artifacts/irs-dual-preflight/`.

### Follow-up on the visible parity failure

Corrected the cross-camera feature-transform rotation signs and the
landmark transform's camera-indexed extrinsic columns. Both startup
checks now pass. The two bright trajectories are byte-identical to the
initial two-camera replays, which rules out these repairs as a remedy for
the observed drift.

With cross-camera matching disabled, feature retirement now evaluates
quality in the feature's own camera. The previously unused camera could
keep a feature alive through its initial visibility and quality priors.
The repaired two-camera visible control reaches **0.106167 m** against
**0.088822 m** native, reducing the gap from 118 to 17 mm. The thermal
control reaches **0.248150 m** against **0.248150 m** native to six decimal
places. All output timestamps and reference support still match. Maximum
visible attitude difference is 1.62°.

A 1,101-output trace reproduced each full run's pose prefix byte for byte.
At 53.249 s after the first pose, feature 498 is rejected for pixel-intensity
error by the native replay while the repaired replay does not emit that
rejection. The two paths retire different features over the next frames.
This identifies where the small preexisting numerical differences turn
into different tracking decisions. The source of those initial differences
remains unresolved. The visible gap still exceeds the frozen 10 mm bound,
so component-fusion accuracy remains unmeasured. The repair scripts,
paired logs and scores are retained in the same artifact directory.
