# Foxglove data validation

| Stream | Pod bridge evidence | Display or source check |
|---|---|---|
| Clock | Advertised, zero frames in first probe | Domain 1 clock active. Screenshot showed the Unix epoch displayed in EST. |
| Dynamic TF | No frames in first probe | Domain 1 dynamic TF active and map-to-base transform resolved. |
| Static TF | Frame delivered | Screenshot showed map axes. Optical calibration alignment unverified. |
| RGB and CameraInfo | Both delivered | Browser rendered a right camera frame. ROS confirms RGB8 dimensions and left K. |
| Depth | Frames delivered | ROS declares 32FC1. Display and numerical depth convention unverified. |
| Pose and map | Odometry and VDB markers delivered | ROS confirmed a map-frame marker with 853 points. Post-repair visual agreement was not independently verified. |
| Robot state and diagnostics | Unobserved among selected domain 0 streams | Domain 1 samples were present, including connected and disarmed state. |

A binary bridge receipt establishes transport. No paired GUI/ROS trace, acquisition/display latency measurement, or geometry test was available. Foxglove was used as an observer. Scoring references come from the independent numerical fixture.
