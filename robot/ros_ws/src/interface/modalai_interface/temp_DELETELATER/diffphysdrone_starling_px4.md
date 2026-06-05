# DiffPhysDrone Starling PX4 Integration

This note captures the integration contract for running a
[DiffPhysDrone](https://github.com/HenryHuYu/DiffPhysDrone) policy on a
Starling vehicle through PX4, first in AirStack/Pegasus and then on hardware.

The key design choice is that the policy wrapper should talk to PX4 offboard
setpoints, not to Isaac Sim or Pegasus rotor commands directly. In simulation,
PX4 will still generate the motor outputs that Pegasus receives over MAVLink.
On the real vehicle, the same wrapper can publish the same PX4 offboard command
class and let PX4 drive the real actuators.

## System Boundary

Recommended command path:

```text
policy wrapper
  -> AirStack cmd_attitude_thrust or PX4 offboard setpoint topics
  -> PX4 attitude/rate controllers
  -> PX4 motor allocation
  -> Pegasus HIL_ACTUATOR_CONTROLS in sim, or ESC outputs on hardware
```

Do not make the first integration publish motor speeds or `ActuatorMotors`.
That would bypass PX4 control allocation and safety behavior, and it would make
the sim wrapper less representative of the real deployment path.

## Current AirStack/Pegasus State

The preliminary Starling asset is now present locally and registered with
Pegasus:

- Nucleus source:
  `omniverse://airlab-nucleus.andrew.cmu.edu/Public/DTC/robots/starling2_preliminary.usd`.
- Local Pegasus asset:
  `pegasus/simulator/assets/Robots/Starling/starling2_preliminary.usd`.
- `pegasus/simulator/params.py` now has `ROBOTS["Starling"]` pointing at that
  asset.
- `pegasus/simulator/logic/vehicles/multirotors/starling.py` defines an initial
  `StarlingConfig` and `Starling` class using the same PX4 MAVLink backend path
  as Iris.

A USD file alone is not a complete Pegasus vehicle integration. The saved scene
or spawning code can reference the USD directly, but credible flight requires
that the asset prim names, physics properties, thrust curve, drag model, rotor
order, spin directions, camera pose, and PX4 airframe all match the actual
Starling. The copied preliminary USD contains the expected `body` and
`rotor0`-`rotor3` tokens, but it still needs an Isaac Sim prim/physics validation
pass inside the stage.

## Starling USD Requirements

The current asset location is:

```text
pegasus/simulator/assets/Robots/Starling/starling2_preliminary.usd
```

The asset/config still needs validation or replacement with measured Starling
data for:

- A rigid body prim named `/body`.
- Four rotor rigid bodies named `/rotor0`, `/rotor1`, `/rotor2`, `/rotor3`.
- Four rotor animation joints named `joint0`, `joint1`, `joint2`, `joint3`.
- Correct mass, inertia, center of mass, collision geometry, and visual meshes.
- Correct rotor locations relative to `/body`.
- Correct rotor order and spin directions matching the PX4 airframe.
- Camera prims/extrinsics matching the DiffPhysDrone depth input.
- Starling-specific `QuadraticThrustCurve`, drag estimate, hover throttle, and
  PX4 vehicle/airframe parameters.

The physical parameters needed before credible flight tests are: mass, inertia,
arm length/rotor positions, motor/prop thrust coefficient, rotor moment
coefficient, max rotor speed, motor order, spin direction, hover throttle/PX4
`MPC_THR_HOVER`, and the real camera intrinsics/extrinsics/latency.

## DiffPhysDrone Action Contract

The available checkpoint is:

```text
Host:      /home/ubuntu/volume/home/ubuntu/media/airlab-storage/chiron/models/diffphysdrone/checkpoint0004.pth
Container: /airlab-storage/chiron/models/diffphysdrone/checkpoint0004.pth
```

It is a bare PyTorch `state_dict`, not a full training checkpoint with metadata.
Its layer shapes match the upstream odometry-enabled DiffPhysDrone model:

- `v_proj.weight`: `(192, 10)`, so the policy expects a 10-D state.
- `fc.weight`: `(6, 192)`, so the policy outputs six values.

The 10-D state used by the upstream training code is:

```text
local_velocity(3), local_target_velocity(3), attitude_feature(3), margin(1)
```

The policy output is reshaped as `(3, 2)` in row-major order, so the default raw
action layout is:

```text
[a_x, v_x, a_y, v_y, a_z, v_z]
```

During training, the raw action is post-processed as:

```python
a_pred, v_pred, *_ = (R @ act.reshape(B, 3, -1)).unbind(-1)
act = (a_pred - v_pred - env.g_std) * env.thr_est_error[:, None] + env.g_std
```

Important detail: `R` here is the yaw-stabilized DiffPhys frame built from the
vehicle forward direction projected onto world XY, not the full rolled/pitched
body frame. The wrapper therefore converts raw policy actions through that same
yaw frame before producing a world-frame ENU net acceleration command.

## Implemented Wrapper Package

The ROS 2 package is:

```text
robot/ros_ws/src/interface/diffphysdrone_px4_wrapper
```

It currently launches two runtime nodes:

- `diffphysdrone_policy` from `diffphysdrone_px4_wrapper`: loads the checkpoint,
  subscribes to depth + odometry + optional target velocity, and publishes both
  `diffphysdrone/raw_action` and `diffphysdrone/accel_cmd`.
- `diffphysdrone_attitude_bridge` from `diffphysdrone_px4_bridge`: converts
  `diffphysdrone/accel_cmd` into `mav_msgs/msg/AttitudeThrust`. This bridge is
  C++ because the current `mav_msgs/msg/AttitudeThrust` Python type support
  segfaults in this container.

Default policy inputs/outputs:

```text
front_stereo/depth                 sensor_msgs/Image
odometry                           nav_msgs/Odometry, AirStack ENU/FLU
optional diffphysdrone/target_velocity  geometry_msgs/Vector3Stamped, ENU

diffphysdrone/raw_action           std_msgs/Float32MultiArray, 6 values
diffphysdrone/accel_cmd            geometry_msgs/Vector3Stamped, ENU net accel
```

Default PX4-facing output:

```text
cmd_attitude_thrust                mav_msgs/AttitudeThrust
```

`px4_interface` already converts this AirStack ENU/FLU attitude/thrust command
into PX4 `VehicleAttitudeSetpoint` in NED/FRD and sends normalized thrust as
`thrust_body[2] = -thrust_norm`.

## Generic Environment Launcher

The policy wrapper is environment-agnostic. It only depends on ROS topics for
depth, odometry, optional target velocity, and PX4/AirStack attitude-thrust
commands. To test in scenes other than TEEX, use the generic Isaac launcher:

```text
simulation/isaac-sim/launch_scripts/generic_env_px4_pegasus_launch_script.py
```

This launcher requires both the environment USD and the drone USD to be explicit.
It does not default to Starling, so a missing `PEGASUS_DRONE_USD` fails early
instead of silently testing Iris. For the current Starling test run, use the
vehicle-free `starling_empty_env.usda` scene for bringup, then switch to
`starling_obstacle_env.usda` for obstacle-avoidance tests. Do not use
`simple_pegasus.scene.usd` for this test; it contains an embedded Iris/PX4 setup.

For the current Starling test run:

```bash
cd /home/ubuntu/volume/home/ubuntu/dtc/airlab_ws/autonomy_ws/src/simulation/AirStack-DTC

ENV_USD_PATH=/isaac-sim/AirStack/simulation/isaac-sim/assets/scenes/starling_empty_env.usda \
PEGASUS_DRONE_USD=/isaac-sim/AirStack/simulation/isaac-sim/extensions/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/assets/Robots/Starling/starling2_preliminary.usd \
ISAAC_SIM_USE_STANDALONE=true \
ISAAC_SIM_SCRIPT_NAME=generic_env_px4_pegasus_launch_script.py \
NUM_ROBOTS=1 \
PLAY_SIM_ON_START=false \
GENERIC_SPAWN_X=0.0 \
GENERIC_SPAWN_Y=0.0 \
GENERIC_SPAWN_Z=1.0 \
./airstack.sh up isaac-sim
```

`ENV_USD_PATH` can also be an Omniverse/Nucleus URL, for example
`omniverse://airlab-nucleus.andrew.cmu.edu/.../scene.usd`, as long as the Isaac
container has access to that server. Before spawning, the launcher removes pre-existing drone/PX4 prims from the
loaded environment USD by default. This prevents a scene such as
`simple_pegasus.scene.usd` from keeping an embedded Iris while the launcher also
spawns Starling. The requested drone is the one passed through
`PEGASUS_DRONE_USD`, and it is attached at `/World/drone1/base_link`. The
launcher also attaches the ZED camera graph there and logs the expected policy
depth topic:

```text
/robot_1/sensors/front_stereo/right/depth_ground_truth
```

Useful optional knobs:

```text
GENERIC_SPAWN_POSITIONS="0,0,1;2,0,1"
GENERIC_STAGE_SCALE=1.0
GENERIC_REMOVE_EXISTING_DRONES=true
GENERIC_EXISTING_DRONE_ROOTS=/World/drone
GENERIC_ADD_COLLIDERS=true
GENERIC_ADD_DOME_LIGHT=true
GENERIC_CAMERA_OFFSET=0.2,0.0,-0.05
GENERIC_CAMERA_ROTATION_OFFSET=0.0,0.0,0.0
```

Leave `GENERIC_REMOVE_EXISTING_DRONES=true` for the Starling policy test. Set it
to `false` only if the environment USD is known to contain no active Pegasus/PX4
vehicle and you intentionally want to preserve every top-level prim. For
`simple_pegasus.scene.usd`, the known embedded Iris root is `/World/stage/World`,
and the generic launcher removes it automatically when it contains Pegasus/PX4
vehicle markers.

## Sim Bringup and Arming Sequence

The wrapper can publish valid commands while the simulated vehicle still does
nothing. The full control path is only active after Isaac/Pegasus, MAVROS,
AirStack's robot interface, and PX4 offboard mode are all healthy.

Start Isaac with the timeline playing. If using the TEEX PX4 launch script,
`PLAY_SIM_ON_START=true` avoids loading the stage paused:

```bash
cd /home/ubuntu/volume/home/ubuntu/dtc/airlab_ws/autonomy_ws/src/simulation/AirStack-DTC

PEGASUS_DRONE_USD=/isaac-sim/AirStack/simulation/isaac-sim/extensions/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/assets/Robots/Starling/starling2_preliminary.usd \
ISAAC_SIM_USE_STANDALONE=true \
ISAAC_SIM_SCRIPT_NAME=teex_multi_px4_pegasus_launch_script.py \
NUM_ROBOTS=1 \
PLAY_SIM_ON_START=false \
./airstack.sh up isaac-sim
```

In the robot desktop container, verify that MAVROS and the simulator streams are
fresh. `ros2 topic hz` runs until interrupted, so stop each check with `Ctrl-C`
before running the next command:

```bash
source /root/AirStack/robot/ros_ws/install/setup.bash

ros2 topic hz /robot_1/interface/mavros/state
ros2 topic hz /robot_1/interface/mavros/local_position/pose
ros2 topic hz /robot_1/odometry_conversion/odometry
ros2 topic hz /robot_1/sensors/front_stereo/right/depth_ground_truth
```

Launch the wrapper in the same container. Use the debug output first; switch the
output topic to AirStack's command topic only when ready to command PX4:

```bash
ros2 launch diffphysdrone_px4_wrapper diffphysdrone_px4_wrapper.launch.xml \
  output_topic:=/robot_1/interface/cmd_attitude_thrust
```

Verify that the policy and bridge are publishing:

```bash
ros2 topic hz /robot_1/diffphysdrone/accel_cmd
ros2 topic echo --once /robot_1/interface/mavros/setpoint_raw/attitude
```

Then request PX4 offboard control and arm through AirStack's robot command
service:

```bash
ros2 service call /robot_1/interface/robot_command airstack_msgs/srv/RobotCommand "{command: 0}"
sleep 2
ros2 topic echo /robot_1/interface/has_control --once

ros2 service call /robot_1/interface/robot_command airstack_msgs/srv/RobotCommand "{command: 1}"
sleep 1
ros2 topic echo /robot_1/interface/is_armed --once
ros2 topic echo --qos-durability volatile --once /robot_1/interface/mavros/state
```

Expected healthy state:

```yaml
connected: true
armed: true
mode: OFFBOARD
```

If `ARM` fails, check PX4's rejection reason:

```bash
ros2 topic echo /robot_1/interface/mavros/statustext/recv --once
```

If `/robot_1/interface/mavros/state` appears old or `AUTO.LOITER` after a
restart, use `--qos-durability volatile` for a fresh sample. If Isaac logs show
`px4_mavlink_backend.py` failing with a `NoneType` MAVLink connection, restart
the Isaac/Pegasus side before debugging the wrapper; the policy can publish
commands even when Pegasus is no longer exchanging actuator controls with PX4.

## PX4 Input Level

PX4 supports offboard position, velocity, acceleration, attitude, body-rate,
thrust/torque, and direct-actuator setpoint modes. For this policy, the two
practical options are:

1. `VehicleAttitudeSetpoint`: publish desired attitude quaternion plus normalized
   thrust. This matches AirStack's existing `cmd_attitude_thrust` path and leaves
   PX4's attitude/rate loops active.
2. `TrajectorySetpoint.acceleration`: publish acceleration in NED with position
   and velocity set to `NaN`. This is semantically close to the policy output,
   but AirStack's current `px4_interface` does not expose an acceleration
   callback yet.

Use option 1 first. It requires a calibrated normalized thrust scale, but it is
the shortest path through the current AirStack interface and keeps the wrapper
portable between sim and hardware.

## Wrapper Math

The first wrapper should publish `mav_msgs/msg/AttitudeThrust` to
`cmd_attitude_thrust`. `px4_interface` already converts AirStack ENU/FLU to PX4
NED/FRD and sends `VehicleAttitudeSetpoint`.

Inputs:

- Policy checkpoint.
- Depth image in the same preprocessing convention as training.
- Odometry in AirStack ENU/FLU.
- Target vector or target velocity matching the training state.
- Starling hover thrust parameter `u_hover`.
- Safety limits for tilt, thrust, acceleration, and command timeout.

Core conversion:

```text
g_enu = [0, 0, -9.80665]
a_cmd_enu = postprocessed policy acceleration-like output
f_des_enu = a_cmd_enu - g_enu
z_body_des = normalize(f_des_enu)
```

Then choose a yaw reference from the mission or current yaw:

```text
x_heading = [cos(yaw_ref), sin(yaw_ref), 0]
y_body_des = normalize(cross(z_body_des, x_heading))
x_body_des = cross(y_body_des, z_body_des)
R_enu_flu = [x_body_des y_body_des z_body_des]
```

Convert `R_enu_flu` to a quaternion in AirStack's FLU-to-ENU convention and put
it in `mav_msgs/AttitudeThrust.attitude`.

Normalized thrust:

```text
thrust_norm = clamp(norm(f_des_enu) / 9.80665 * u_hover,
                    thrust_min,
                    thrust_max)
```

Publish:

```text
AttitudeThrust.attitude = q_enu_flu
AttitudeThrust.thrust.z = thrust_norm
```

`px4_interface` will send PX4 `thrust_body[2] = -thrust_norm`, which is the
expected multicopter sign convention for body FRD.

## Safety Gates

Before real flight, the wrapper should enforce:

- A heartbeat/watchdog that stops publishing policy commands on stale images,
  stale odometry, stale checkpoint inference, or invalid depth.
- Minimum and maximum normalized thrust.
- Maximum tilt angle.
- Maximum acceleration and jerk.
- NaN/Inf rejection.
- A neutral hover command fallback.
- Offboard arming only after setpoints have streamed for at least one second.
- A bench test with props removed and PX4 logs checked for command signs.

## Implementation Plan

Completed locally:

1. Copied `starling2_preliminary.usd` from Nucleus into the Pegasus robot assets.
2. Registered `ROBOTS["Starling"]` in Pegasus params.
3. Added an initial `StarlingConfig`/`Starling` vehicle class using the PX4
   MAVLink backend.
4. Added `diffphysdrone_px4_wrapper` plus the C++ `diffphysdrone_px4_bridge` with
   pure conversion math, unit tests, a checkpoint inference node, and a launch
   file for policy + PX4 wrapper.
5. Verified Python compilation, unit tests, checkpoint load/inference shape, XML
   parsing, and `colcon build --packages-select diffphysdrone_px4_bridge diffphysdrone_px4_wrapper`.

Next validation steps:

1. Open the Starling USD in Isaac Sim and verify actual prim paths, mass,
   collisions, rotor ordering, joint axes, camera pose, and scale.
2. Replace placeholder Starling thrust/drag/PX4 airframe parameters with measured
   values.
3. Remap the policy `depth_topic` to the actual Starling/Pegasus depth topic and
   confirm the image encoding/resolution matches the preprocessing path.
4. Run sim with very conservative thrust and tilt limits, then compare PX4 logs
   against expected attitude/thrust signs before any real flight.
5. Move the same ROS wrapper to hardware with only topic/transport/parameter
   changes.

## References

- DiffPhysDrone repository:
  <https://github.com/HenryHuYu/DiffPhysDrone>
- Paper:
  <https://arxiv.org/abs/2407.10648>
- PX4 offboard mode:
  <https://docs.px4.io/main/en/flight_modes/offboard>
- PX4 `VehicleAttitudeSetpoint`:
  <https://docs.px4.io/main/en/msg_docs/VehicleAttitudeSetpoint>
- PX4 `TrajectorySetpoint`:
  <https://docs.px4.io/main/en/msg_docs/TrajectorySetpoint>


### Obstacle Environment

For obstacle-avoidance checks, rerun the same generic launcher with:

```bash
ENV_USD_PATH=/isaac-sim/AirStack/simulation/isaac-sim/assets/scenes/starling_obstacle_env.usda
```

Keep `PLAY_SIM_ON_START=false`, verify the obstacle appears in `/robot_1/diffphysdrone/debug_depth_input`, then arm and publish a small target velocity first, for example `x: 0.2`.

## Real Starling / VOXL Deployment Gate

The first hardware-facing implementation should preserve the ROS/PX4 contract
that already works in simulation, then change one variable at a time.

### 1. Export Checkpoint to ONNX

Inside the robot workspace after building this package:

```bash
source /root/AirStack/robot/ros_ws/install/setup.bash
ros2 run diffphysdrone_px4_wrapper diffphysdrone_export_onnx \
  --checkpoint /airlab-storage/chiron/models/diffphysdrone/checkpoint0004.pth \
  --output /airlab-storage/chiron/models/diffphysdrone/checkpoint0004.onnx
```

The exported ONNX model takes the preprocessed policy tensors, not raw camera
images:

```text
depth:  [1, 1, 12, 16] float32
state:  [1, 10] float32 for checkpoint0004.pth
hidden: [1, 192] float32

outputs:
action:      [1, 6] float32
next_hidden: [1, 192] float32
```

### 2. Compare ONNX Against PyTorch

Run a deterministic recurrent parity test before using ONNX in sim or on VOXL:

```bash
ros2 run diffphysdrone_px4_wrapper diffphysdrone_check_onnx_parity \
  --checkpoint /airlab-storage/chiron/models/diffphysdrone/checkpoint0004.pth \
  --onnx /airlab-storage/chiron/models/diffphysdrone/checkpoint0004.onnx \
  --steps 64
```

This catches export/runtime differences in both the policy action and recurrent
hidden state.

### 3. Run ONNX Backend In Simulation

The launch file defaults to PyTorch. To switch only the policy inference backend
while keeping the same ROS topics and PX4 bridge:

```bash
ros2 launch diffphysdrone_px4_wrapper diffphysdrone_px4_wrapper.launch.xml \
  backend:=onnx \
  onnx_path:=/airlab-storage/chiron/models/diffphysdrone/checkpoint0004.onnx \
  output_topic:=/robot_1/interface/cmd_attitude_thrust
```

The expected ROS graph is unchanged:

```text
depth + odometry + target velocity
  -> diffphysdrone_policy (ONNX backend)
  -> /robot_1/diffphysdrone/raw_action
  -> /robot_1/diffphysdrone/accel_cmd
  -> diffphysdrone_attitude_bridge
  -> /robot_1/interface/cmd_attitude_thrust
```

### 4. VOXL Dry Run Before Flight

On VOXL, first prove the exported model can run without commanding PX4:

```bash
ros2 run diffphysdrone_px4_wrapper diffphysdrone_voxl_policy_smoke \
  --onnx /path/to/checkpoint0004.onnx \
  --dim-obs 10 \
  --steps 500
```

This only feeds fixed tensors through the ONNX model and reports inference rate
and the last action. It does not subscribe to sensors or publish commands.

### VOXL / Qualcomm Runtime Direction

For Starling 2 / VOXL 2, ONNX is the first portable artifact and parity gate.
ModalAI's documented onboard neural-network path is usually LiteRT/TFLite via
`voxl-tflite-server`, which can use CPU/GPU/NNAPI delegates on VOXL 2's QRB5165.
That means the likely final accelerator path is:

```text
PyTorch checkpoint -> ONNX -> TensorFlow/LiteRT/TFLite -> voxl-tflite-server
```

If we choose not to integrate with `voxl-tflite-server`, the alternate path is a
custom onboard app using ONNX Runtime CPU. That is simpler for proof-of-life but
less aligned with ModalAI's accelerated deployment stack.

### Frame Transform Contract

The current AirStack sim path is ROS-style ENU world and FLU body until the
AirStack/PX4 interface converts to PX4. PX4 and VOXL use NED world and FRD body
internally. The simple vector transforms are:

```text
ENU -> NED: (x_east, y_north, z_up) -> (y_north, x_east, -z_up)
FLU -> FRD: (x_forward, y_left, z_up) -> (x_forward, -y_left, -z_up)
```

As long as the real deployment still publishes the same ROS topics into this
wrapper and sends commands through the same MAVROS/AirStack interface, that
conversion remains outside the policy node. If we bypass MAVROS/AirStack and
publish directly to PX4 uORB/RTPS/VOXL APIs, this conversion must move into the
hardware command adapter.

