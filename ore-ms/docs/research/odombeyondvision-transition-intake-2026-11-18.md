# OdomBeyondVision natural-transition intake

OdomBeyondVision does not clear the metadata gate for an independent-pose
lighting-transition study. The [paper, sections III-B to III-D](https://arxiv.org/html/2206.01589)
documents RGB, thermal and IMU, and defines lighting labels by frame content.
A dim or dark label alone cannot establish a sustained within-run transition.

I read all three CSV tabs of the [author sequence inventory](https://docs.google.com/spreadsheets/d/1n2KkD_vjE7b5-2_qaq-sq0c-cB4JdQMBo3zMRFuTT_w/edit#gid=1265042687)
and cross-checked the UAV dates against the [public bag folder](https://drive.google.com/drive/folders/1RWCzvdBjZBn7AE7pel15ZAtUiHrX1ekT).

| Platform | Bright | Dim | Dark | Published reference |
| --- | ---: | ---: | ---: | --- |
| UAV | 13 | 0 | 0 | Vicon |
| Handheld | 40 | 0 | 4 | A-LOAM LiDAR odometry |
| UGV | 69 | 4 | 3 | Wheel odometry |

The 13 UAV labels map to eight bags. Five rows name additional segments
of the same January 20 bag. The release does not identify a Vicon-backed
dim or dark interval. Handheld LiDAR odometry could support a separate
nominal-reference study after validation, and wheel odometry cannot
supply independent six-degree-of-freedom motion capture.

This screen checked the author metadata and [pinned release README](https://github.com/mfkiwl/OdomBeyondVision/tree/d30b06427457bf65a06c9c52245874d736b585fa).
Bag payloads, timing, calibration and image transitions remain untested.
No bags were downloaded and no estimator was run. Reopen this route if
a Vicon-backed transition interval is documented or an alternative
reference can be qualified for the stated evaluation.
