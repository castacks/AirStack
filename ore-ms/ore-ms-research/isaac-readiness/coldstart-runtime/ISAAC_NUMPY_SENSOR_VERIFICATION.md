# Isaac, NumPy and sensor verification

The Isaac container was created with the qualified digest and never started. Its logs were empty. The running-container `/isaac-sim/python.sh` NumPy assertion and writer regression check require an active Isaac process. RGB, CameraInfo, depth and LiDAR publication could not be sampled. PX4, MAVROS, odometry and map were also unavailable. No rates, header timestamps, frame IDs or dimensions were measured in this workflow.

The earlier remediation run demonstrated NumPy 1.26.4, RGB8, 32FC1 depth, CameraInfo and a raw LiDAR cloud in its existing pod. [Prior smoke test](../numpy-remediation/SENSOR_SMOKE_TEST.md)
