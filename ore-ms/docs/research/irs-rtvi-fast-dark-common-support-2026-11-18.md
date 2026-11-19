# IRS RTVI fast-dark metric position on common timestamps

Reran the author RRxIO loader on the [public IRS RTVI release](https://christopherdoer.github.io/datasets/irs_rtvi_datasets_iros2021).
Use the uninterrupted fast-dark bag, the released camera calibrations and
camera-specific presets, the same IMU stream, and 15 features. Both camera
runs disable radar and external velocity. The visual and thermal camera
shifts are +2 ms and −4 ms, respectively. The pipeline consumes no reference
poses. This is one lights-off recording and a fixed-camera comparison.

Freeze and hash the scorer before replay. Score metric body position on Vicon–IMU
timestamps supported by both estimates. Accept exact samples or linearly
interpolate between samples separated by at most 100 ms, with no extrapolation.
Fit one gravity-axis yaw and translation per arm on the first 10 seconds
of shared reference coverage. Score the remainder with that transform and
unit scale. Also report a whole-overlap fit for comparability with the
[historical run](irs-rtvi-stateful-replay-2026-11-17.md).

| Alignment and scoring | Shared positions scored | Visual RMSE | Thermal RMSE |
| --- | ---: | ---: | ---: |
| First 10 seconds fit, remainder scored | 1,222 | 4.302 m | 0.282 m |
| Full overlap fit and score | 1,418 | 2.127 m | 0.142 m |

Each arm published 1,735 pose rows. Of 1,420 Vicon–IMU timestamps, 1,418
are supported by both arms, with 196 rows in the first 10 seconds.
Under the early fit, median errors are 4.398 m visual and 0.280 m
thermal. Maximum errors are 5.606 m and 0.398 m. Independent complex
rotation arithmetic reproduces the four shared-set RMSE values to within
1e-10 m. Synthetic rigid alignment and interpolation-limit checks pass.

Individual full-fit RMSE is 2.12694 m visual and 0.14210 m thermal, close
to the earlier 2.127 m and 0.125 m. The thermal result differs by about
17 mm and the output counts differ from the historical run. Exact
historical capture parity is unverified. The common timestamps and
early-only fit preserve the substantially lower thermal error on this bag.

The loader source is pinned to
`ae29afe0d7876f3838056b457292e4aecf9656be` in ROS Melodic image
`sha256:b4366ca0536e2ad3654ed66fb4475bc43b7a0c15bd8a5be91fd5ac9dfaf935d0`.
Build adaptations declare the radar library's existing YAML dependency,
disable YAML tools and tests, build only the 15-feature loader variant,
and use GCC 7.5 with `-O1` for the core catkin targets. The camera presets
disable frame display. The first build failed on the missing radar YAML
dependency and was rebuilt successfully before replay.

Verified the bag against the release ZIP member CRC, then recorded
SHA-256 `0a334b5efcb3fc9610c1b8be42cf51032a311241cc381a5bfc3c4c6e5c808ebb`.
The two raw pose CSVs, reference, shared-support ledger, commands, logs,
and input and output hashes are in
`/root/ore-ms-artifacts/irs-author-rescore/`.
