# Deploying a policy through SVG

Use SVG's deployment interface to fly a learned policy in AirStack simulation
and on a physical drone. You provide the policy, its observation builder, and
its output type. SVG handles the control loop, takeoff, goal handoff, geofence,
stale-odometry hold, and landing. Changing `drone_mode` routes the same policy
through simulated PX4 or the real PX4 interface.

At each control tick, SVG reads AirStack odometry into `RobotState`, builds the
policy's observation, calls `Policy.compute()`, and publishes the returned
attitude, velocity, or waypoint setpoint.

## Add a policy

1. Put the policy/checkpoint where the **robot container** can read it. Check
   the input encoding and output units from training; SVG cannot infer them.
2. In a small module under `deploy/`, implement `ObservationBuilder.build(state)`
   and `.reset()`. `RobotState` provides ENU world position/velocity, FLU-to-ENU
   orientation, a goal, and an optional perception payload. Subscribe to any
   extra sensor in your commander and return it from `get_perception()`.
3. Implement `Policy.compute(obs)` and `.reset()`. Return one of
   `AttitudeSetpoint`, `VelocitySetpoint`, or `WaypointSetpoint` from
   `interfaces.py`, with its documented frame and units. A wrapper around an
   existing policy is fine; you need not rewrite its model.
4. Subclass `PolicyCommanderBase`, providing `make_observation_builder()` and
   `make_policy()`. Use `declare_extra_parameters()` for checkpoint/settings
   and `on_setup_complete()` for extra subscriptions. See `deploy/diffaero.py`.
5. Register its `main` as a `console_scripts` entry in `setup.py`, and add a
   launch file and parameter YAML selecting that executable. Build/source the
   ROS workspace after changing these files.
6. Test the observation and setpoint transformations without ROS, then do a
   short sim flight: launch, call `~/takeoff`, wait for `ACTIVE`, call
   `~/start`, observe goal hold, and call `~/land`. Only after verifying its
   frames, timing, and limits in sim should you use `drone_mode: real` with the
   hardware interface and mocap setup.

Each supported policy currently has a small entry point and config. The flight
lifecycle is shared across them. `deploy_diffaero` is the first example using this API.

## DiffAero example

Set up the single-drone Isaac/PX4 and MAVROS interface in AirStack with
`NUM_ROBOTS=1` and `AUTOLAUNCH=false`. Build and source the ROS workspace,
start the simulator and `scripts/launch_sim_interfaces.sh 1`, and ensure the
checkpoint named in `config/deploy_diffaero_sim.yaml` exists inside the robot
container. Then:

```bash
ros2 launch svg_ground_control deploy_diffaero.launch.xml
ros2 service call /diffaero_commander/takeoff std_srvs/srv/Trigger
# Wait until the commander reports ACTIVE.
ros2 service call /diffaero_commander/start std_srvs/srv/Trigger
# The commander holds on reaching the goal.
ros2 service call /diffaero_commander/land std_srvs/srv/Trigger
```

The generic path has passed CPU interface tests but has not yet flown in Isaac
or on hardware. With `trial_log_dir` set, each Start opens a JSONL trial in the
ROS workspace bind mount. It records policy identity, sim/real mode, goal,
checkpoint path, odometry, policy commands, and why the trial ended. These are
raw data, not a SAFE score: contact labels, obstacle geometry, and independent
tracking (for real flights) still need to be joined before scoring. The sim
config enables logging under `robot/ros_ws/safe_trials/`.
