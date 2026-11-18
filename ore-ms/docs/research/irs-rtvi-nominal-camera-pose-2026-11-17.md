# IRS RTVI nominal camera rotation and translation direction

This analysis extends the [IRS visual–thermal comparison](irs-rtvi-mocap-comparison-2026-11-17.md)
and [fast-dark interval control](irs-rtvi-dark-fast-replication-2026-11-17.md)
from nominal rotation agreement to the existing joint 5° rotation and 15°
directed-translation endpoint. It uses the already locked camera-only
predictions. Missing proposals and unsupported reference rows remain explicit.

For each camera, linearly interpolate its published Vicon–IMU position
at the same signed exposure-header offsets and accepted reference intervals
used by the rotation audit. Use SLERP for orientation and compose each
world-from-body pose with the released IMU-from-camera transform. Compare
the source-to-target camera rotation and the target-camera translation
direction. This transports the camera origins using the recorded lever arms.
A synthetic +X camera-motion test verifies the expected −X source-origin
translation in the target frame. Homogeneous composition and the prior
rotation audit independently check the frame/order of the nominal result.

The thermal calibration's inverse matrix is published as whitespace-delimited
row strings. The first parser attempt stopped at those rows and the second
at an overly tight numerical inverse check. The final check reads the exact
published entries and accepts the observed inverse residual below 1e-6.
Using the alternate published thermal-to-visual-to-IMU chain changes the
nominal translation direction by less than 0.00002° at the 1 cm floor.
This tests agreement between the published chains. It does not measure
physical extrinsic accuracy.

## Learned joint nominal agreements

The denominators include all anchors where both cameras have bracketed poses
and at least 1 cm of published camera displacement. A missing proposal does
not agree. Policies are replayed from output status alone.

| Recording / interval | Eligible anchors | Fixed visual | Fixed thermal | Visual-first | Thermal-first |
|---|---:|---:|---:|---:|---:|
| Easy / 0.5 s | 137 | 137 | 110 | 137 | 123 |
| Dark / 0.5 s | 229 | 15 | 132 | 52 | 134 |
| Fast dark / 0.5 s | 123 | 5 | 39 | 16 | 40 |
| Fast dark / 0.1 s | 121 | 12 | 105 | 22 | 105 |

At a 10 cm displacement floor, the ordering is unchanged. On the 106
fast-dark anchors meeting that floor for both cameras and both durations,
thermal-first rises from 33 to 98 nominal joint agreements, with proposals
rising from 60 to 106. At the 1 cm floor the same matched analysis gives
40→104 agreements on 120 anchors. A 0.1 s interval has less motion than a
0.5 s interval and the result applies to these distinct pair tasks.

This reports relative pose agreement against a published camera-independent
reference at the tested tolerances. The estimates contain a unit translation
direction with no scale. The reference uses Vicon and IMU and retains
unmeasured physical timing, tracking and calibration error. These results
do not certify sub-degree accuracy, metric translation error or trajectory
drift. All 3,220 rows and the four policy replays at both motion floors passed
an independent counting audit.

## Live recovery boundary

An essential-matrix proposal uses two images and can be computed independently
of earlier outcomes. A run of missing proposals is a sampled frontend
diagnostic. Live recovery needs a stateful estimator, an explicit loss and
reacquisition definition, and a sequential replay or hardware run that retains
state across camera switches and measures elapsed recovery and latency. The
camera-tagged unit translations cannot be concatenated into a metric body
trajectory. The IRS rig has one visible and one thermal camera, so the existing
same-modality stereo systems need a different input configuration for this
recording. A prospective live study can use a sensor-matched stateful estimator
with frozen failure/recovery and compute criteria. That is a later experiment.

Scripts, frozen controlled inputs, the nominal ledger and checksums are
under `/root/ore-ms-artifacts/irs-rtvi-intake/`. No new downloads, model
calls, Box writes or live replay were needed.
