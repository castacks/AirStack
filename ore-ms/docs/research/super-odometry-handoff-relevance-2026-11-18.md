# Super Odometry and the thermal handoff recovery gap

The [project site](https://superodometry.com/) describes the Science
Robotics paper [Resilient odometry via hierarchical adaptation](https://doi.org/10.1126/scirobotics.adv1818).
The [author manuscript](https://arxiv.org/html/2608.25427v1) separates
adaptation at the feature, state-direction and estimator levels. Its
state-direction method uses LiDAR geometry to assess which motion
components need independent constraints. Its estimator-level mechanism
uses sensor confidence to select constraints, with learned inertial
odometry for simultaneous exteroceptive degradation. Table 2 evaluates
the full method on SubT-MRS. These results do not isolate visible-to-thermal
handoff or establish IRS RTVI position accuracy.

The [2021 system](https://arxiv.org/pdf/2104.14938v2), sections III–IV,
uses an IMU-centered state with biases constrained by camera and LiDAR
pose information. The [public ROS 2 release](https://github.com/superxslam/SuperOdom/tree/f10e65cd50007767b22e4c401689665e20d827d6)
describes a slim LiDAR or LiDAR–IMU implementation. It provides no
drop-in camera handoff for this rig. The existing RRxIO replay already
preserves its inertial body state across the switch. The research question
is how that state is constrained before and after the change of camera.

## Direction of the measured recovery error

Reuse the locked grids and ten-second pre-dimming fits from the
[causal brightness test](irs-rtvi-causal-dimming-2026-11-18.md).
Decompose world-frame residuals along Vicon XY and Z. The squared
horizontal and vertical RMSEs sum to the squared 3D RMSE. The squared
magnitude of the average error vector divided by the total mean squared
error measures how much of the error remains as a consistent offset
within each epoch. No fit uses scoring-window samples.

| Recording and switch | Epoch after light returns | Horizontal RMSE | Vertical RMSE | Mean-offset share of squared 3D error |
| --- | ---: | ---: | ---: | ---: |
| Medium, brightness-triggered | 0–2 s | 0.183 m | 0.007 m | 99.1% |
| Difficult, brightness-triggered | 0–2 s | 0.254 m | 0.017 m | 99.0% |
| Difficult, brightness-triggered | 2–10 s | 0.323 m | 0.032 m | 93.7% |
| Difficult, scheduled | 2–10 s | 0.123 m | 0.030 m | 50.8% |

Horizontal residuals account for over 99% of squared early recovery
error in both brightness-triggered runs. In Difficult, the later
brightness-triggered penalty over scheduled handoff is also predominantly
horizontal. These are realized position residuals in the reference frame.
Their direction and persistent mean do not identify estimator
observability, a bias failure, or the cause of the offset.

A discriminating next control keeps the same three-frame brightness
decision and thermal arrival times, withholds dim-flagged visible
updates before handoff, and preserves the IMU process. Comparing
against the existing brightness-triggered arm at identical switch
times isolates whether applying those visible updates changes recovery.
Retain the scheduled and fixed-camera controls and the 0.15 m position
and 0.15 s output-gap criteria. This is an RRxIO measurement-admission
test inspired by the paper. Directional weighting or a learned IMU
prior would each require a different experiment.

All 24 decompositions reconstruct the published 3D scores and pass
the mean-plus-centered-error identity check. The two versioned paper
PDFs, calculation, fixed method, and checksums are under
`/root/ore-ms-artifacts/irs-author-rescore/superodometry/`.
