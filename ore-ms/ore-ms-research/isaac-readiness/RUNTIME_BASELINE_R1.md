# ORE-MS Runtime Baseline R1

Qualified on 26 November 2026 UTC for the stationary, single-robot Isaac Sim workspace. The checkout was `castacks/AirStack`, branch `ore-ms-orb-development`, at `6b1780a9fba12244aca61a338bdf4694fe209c06` with the MAVROS correction in the bind-mounted source. This baseline records the running system and persists that correction in Git. The earlier readiness run supplied by the operator reported five passing gates, exit 0 and 6 s elapsed.

## Directly observed during R1

- `./airstack.sh ready --json` returned `ready: true`, exit 0 and 8 s elapsed. Containers, simulation clock, robot_1 autonomy sentinels, MAVROS connectivity and PX4 EKF odometry all passed. The EKF gate infers armability from streaming local odometry. MAVROS reported `armed: false`.
- The 12-second subscription probe received nine state messages on `/robot_1/interface/mavros/state` and 262 odometry messages on `/robot_1/interface/mavros/local_position/odom`. The latest state reported `connected: true`.
- TF lookups from `map` to `base_link`, `camera_left`, `camera_right` and `ouster` passed. The left RGB, right RGB, right depth and Ouster lookups also passed at an interior message's exact header stamp and frame.
- All six sampled sensor streams had positive, strictly increasing header stamps. The four timestamped TF samples were about 30–60 ms behind the most recent `/clock` message received by the probe. The two latest received RGB headers matched exactly, and the sample contained 273 exact left/right stamp pairs. These are subscriber observations over one window. Clock age includes subscription scheduling and does not measure transport latency or physical exposure synchronization.
- `/robot_1/sensors/lidar/point_cloud` had zero publishers and one subscriber, `/robot_1/topic_keepalive`. R1 uses the live Ouster stream below.

| ROS topic | Header frame | Samples | Received Hz, wall time | Received Hz, simulation time |
| --- | --- | ---: | ---: | ---: |
| `/robot_1/sensors/front_stereo/left/image_rect` | `camera_left` | 285 | 23.88 | 31.87 |
| `/robot_1/sensors/front_stereo/right/image_rect` | `camera_right` | 284 | 23.78 | 31.76 |
| `/robot_1/sensors/front_stereo/left/camera_info` | `camera_left` | 297 | 24.87 | 33.22 |
| `/robot_1/sensors/front_stereo/right/camera_info` | `camera_right` | 297 | 24.87 | 33.22 |
| `/robot_1/sensors/front_stereo/right/depth_ground_truth` | `camera_right` | 277 | 23.17 | 30.98 |
| `/robot_1/sensors/ouster/point_cloud` | `ouster` | 86 | 7.79 | 10.42 |

The rates divide the received sample intervals by the elapsed time between first and last samples. Best-effort DDS delivery and the simulation real-time factor affect these numbers. Raw results are in [runtime-baseline-r1.json](evidence/runtime-baseline-r1.json).

## Namespace defect and correction

The committed launch file at the observed checkout explicitly passed an empty `namespace` to the AirStack MAVROS wrapper. The wrapper forwards the argument to Jazzy MAVROS `node.launch`, which uses it as the node namespace. Passing an empty string overrides the wrapper's `mavros` default under the robot's `interface` namespace. Archived local robot logs before the correction showed `robot_1.interface.mavros_router`; logs from the corrected startup showed `robot_1.interface.mavros.mavros_router`. AirStack readiness expects canonical state and local odometry under `/robot_1/interface/mavros/`.

The source change persists the observed `'namespace': 'mavros'` argument and its comment. The source and the live installed launch entry matched SHA-256 `ef25fb28ac09b5e0008be350db73fa719b63900795c32c10b20e29264d61a8b5`. The wrapper, FCU arguments and simple-sim branch retain their existing behavior. [AirStack launch source](../../../robot/ros_ws/src/interface/interface_bringup/launch/interface.launch.py)

## Deployment and reproducibility

| Component | Observed image reference | Local image config ID |
| --- | --- | --- |
| Robot desktop | `sha256:f2c7bce4797008bff889dbef2881fd015780c972f16b8e344c13a22bba02cbf4` | `sha256:3d8fa569dfed6f7249faee9c205bc49eb44c711991db9e7c65d1fa406d46fdda` |
| Isaac Sim livestream | `sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697` | `sha256:b45a696338c10c984487b9b91a215b20c579afabc1f00a216dc43dacfb76bacd` |
| GCS | `v0.20.8_gcs`; local repo digest `sha256:d25c4ae77de5fd6da43c334220e833dacedd61bceafce389e882b735ae41af9a` | `sha256:9e44be1c0c38e297ba5b9fc463f472ca5c224091969f50e417825a412e61aa7e` |

All three images are from `airlab-docker.andrew.cmu.edu/airstack/airstack`. Workspace hostname observed locally was `aadb4af61cb54256-f3a1f0333d394b98`. The submitted OSMO YAML, workflow ID and outer workspace image digest were unavailable for independent verification. The robot container was created at 19:54:53 UTC and its inspected start time was 20:32:48 UTC. Compose labels report project `airstack`, Compose 5.1.3, and the root Compose file plus `/tmp/ore-ms-robot-reuse/robot-reuse.yaml`.

- The repository OSMO entrypoint clones recursively to `/root/AirStack` from `AIRSTACK_BRANCH` if the checkout is absent. Existing checkouts are reused. The stock workflow has a floating outer tag and a default branch of `main`. A replay needs the baseline commit and recursive submodule pins, including Pegasus `8c7a664095396bc9c4f4609531445d4cf6390fe4`.
- Root Compose includes the robot Compose file, where `robot-desktop` inherits the robot base service. The observed robot mount maps the entire workspace `/root/AirStack/robot/ros_ws` to that same path in the container, including `src`, `build` and `install`. The stack, configuration and generated files also have host mounts. Isaac binds the checkout and Pegasus extension.
- The robot reuse overlay sets the pinned robot image and `build: !reset null`. The existing overlay hashed to `c08fb7a0147ae9e9711ae96bebbe57c117c6cd1143949831a394a2c53e27bfc1`. Static rendering with `desktop,isaac-sim-livestream`, one robot and autolaunch enabled reproduced all three image references and confirmed the robot `build` stanza was removed. The simulator and GCS still have build stanzas in that render. The controlled launch relies on the no-build path and on supplying the overlay again when recreating services. [Controlled test inputs](robot-image-reuse/CONTROLLED_TEST_R2_HANDOFF.md)
- The Compose autolaunch shell builds the mounted workspace with `colcon build --symlink-install`, sources its install overlay and starts `autonomy_bringup`. The live robot uses `full_default`, which includes the interface launch via package share. `ros2 pkg prefix --share interface_bringup` resolved to `/root/AirStack/robot/ros_ws/install/interface_bringup/share/interface_bringup`. Its launch file was a symlink to the corrected mounted source. MAVROS came from `/opt/ros/jazzy`, package version `2.14.0-1noble.20260615.151804`.
- **Restart inference.** A new launch in the same container with the same mount and valid install overlay reads the current source through that symlink. A running launch has already evaluated the file. **Recreation inference.** With the same Compose inputs and baseline checkout, the bind-mounted source survives container replacement. A fresh checkout needs a successful in-container workspace build or an equivalent verified install overlay. An older checkout, a copied or stale install, a missing mount, or a missing temporary overlay could change the result. Verify the package-share path and file hash after provisioning.

Remaining deployment gaps include confirming the actual submitted workflow and outer digest, persisting or regenerating the overlay in a fresh workspace, confirming the GCS digest on replay, and independently replaying the exact asset and runtime inputs. Compose rendered with unset `DISPLAY` warnings in the audit shell. No restart, recreation, cold replay, flight, geometry pilot or M12/M13 test was executed during R1. No runtime regression was observed within the sampled checks.

## Read-only regression procedure

From the checkout root on the already-running R1 workspace, run both commands. Run the second probe inside the robot container so it inherits the deployment network. The R1 robot uses DDS domain 1 and the mounted Fast DDS profile.

```bash
timeout 60 ./airstack.sh ready --json
timeout 30 docker exec -i airstack-robot-desktop-1 bash -c '
  source /root/AirStack/robot/ros_ws/install/setup.bash
  export ROS_DOMAIN_ID=1
  export FASTRTPS_DEFAULT_PROFILES_FILE=/root/AirStack/robot/ros_ws/src/fastdds.xml
  python3 -
' < ore-ms/ore-ms-research/isaac-readiness/check_runtime_baseline.py
```

Expect both commands to exit 0. The subscriber probe requires clock progress, connected state, at least two odometry and sensor messages per required stream, positive increasing sensor stamps, all four latest TF links, exact-time TF for one interior sample of each RGB/depth/Ouster stream, and at least one exact stereo pair. It prints sampled age and rates without enforcing a latency or rate threshold. Inspect the JSON and the DDS domain on failure. The commands subscribe and inspect only.
