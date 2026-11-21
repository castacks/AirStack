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
