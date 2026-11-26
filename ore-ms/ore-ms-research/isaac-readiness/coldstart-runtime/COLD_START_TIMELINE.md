# Cold-start timeline

All times are UTC on 26 November 2026. The workspace's task stdout and scheduler history were unavailable from this session.

| Time | Observed event |
|---|---|
| 18:21:51 | Inner Docker completed initialization and opened its API socket. |
| 18:24:20 | Git reflog records the AirStack clone at the accepted Phase A commit. |
| 18:24:28 | Docker logged `unknown: artifact airstack/airstack:v0.20.8_robot-x86-64_dev not found` on the robot image pull. |
| 18:25:42 and 18:29:10 | Docker emitted image pull events for the AirStack registry repository. Both the expected GCS tag and Isaac digest were present on inspection. |
| 18:29:11 | Docker created the AirStack network. Docker inspect records creation timestamps for Isaac and GCS in the same second. |
| 18:29:14 | Docker's event history emitted both container create events. |
| 18:37 | Workspace SSH listened on port 22. Docker showed Isaac and GCS in `created` with zero start timestamps and empty container logs. No robot container existed. |

The PID 1 command referenced rclone and the baked entrypoint. Their timestamped startup output and rclone's outcome were unavailable. No Kit, Pegasus, ROS bridge, PX4, MAVROS, robot desktop, GCS application or Foxglove initialization was observed. The startup record establishes a workspace with a working inner Docker daemon and a stack that did not start. Task stdout is needed to reconstruct the exact command and establish whether the attempted launch was unattended throughout.

[Sanitized diagnostics](evidence/diagnostics.txt)
