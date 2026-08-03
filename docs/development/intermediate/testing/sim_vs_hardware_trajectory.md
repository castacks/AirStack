# Comparing Simulation and Hardware

The [fixed-trajectory benchmark](end_to_end_testing.md) measures how well the autonomy stack tracks a known path in simulation. This page covers running the *same* benchmark on a real drone and comparing the two results, so a claim like "the stack behaves the same in sim and in the air" can be backed by numbers.

## Why the comparison is possible

Everything above the interface layer is autopilot-agnostic. The chain that flies a fixed trajectory is:

```mermaid
flowchart LR
    A[FixedTrajectoryTask] --> B[trajectory_controller]
    B -->|tracking_point| C[pid_controller]
    C -->|cmd_roll_pitch_yawrate_thrust| D[RobotInterface plugin]
    D --> E([Autopilot])
    E -->|odometry| F[odometry_conversion]
    F -->|/{robot}/odometry_conversion/odometry| B
```

Only the **RobotInterface plugin** at step D differs between the two worlds:

| | Simulation | Hardware |
| --- | --- | --- |
| Plugin | `mavros_interface::MAVROSInterface` | `px4_interface::PX4Interface` |
| Transport | MAVLink over UDP to PX4 SITL | uXRCE-DDS to the flight controller |
| Node namespace | `/{robot}/interface` | `/{robot}/fmu` |
| Position source | PX4 SITL's own estimate | EKF2 fused with motion capture |

Both plugins subclass `robot_interface::RobotInterface` and both override `roll_pitch_yawrate_thrust_callback`, which is the command the PID controller emits. The planner, trajectory controller, PID gains, and task action servers are byte-identical across the two runs — which is precisely what makes the measured difference meaningful.

## Making the two runs comparable

Three things must match before the numbers mean anything.

**Geometry.** The sim defaults (10 m radius at 10 m altitude) will not fit an indoor arena. Pick a size the flight volume can hold and use it for both runs via `--trajectory-scale`, `--trajectory-altitude`, and `--trajectory-velocity`. A 1.5 m circle at 0.5 m/s and 1.5 m altitude is a reasonable starting point:

```bash
--trajectory-types Circle --trajectory-scale 0.15 \
--trajectory-altitude 1.5 --trajectory-velocity 0.5
```

**Ground truth.** Simulation compares PX4's reported odometry against the ideal path. On hardware, PX4's estimate is *fused with* motion capture, so comparing against it measures the controller plus the estimator's own optimism. Record the raw mocap pose separately and compute cross-track error against that instead; the estimator-versus-mocap difference is itself worth reporting.

**Reference frame.** The ideal path is generated in `base_link` at the moment of dispatch and rotated into the world frame using the robot's pose snapshot. The hardware run must snapshot the same way, from the same odometry topic, or the whole reference path is offset.

## Hardware bring-up

The `local` layer takes two launch arguments that point it at the PX4 interface instead of MAVROS:

| Argument | Simulation default | Hardware value |
| --- | --- | --- |
| `local_interface_ns` | `/{ROBOT_NAME}/interface` | `/{ROBOT_NAME}/fmu` |
| `local_extended_state_topic` | `{ns}/mavros/extended_state` | a PX4-sourced equivalent |

`px4_interface.launch.xml` already starts the interface plugin under `/fmu` and runs `odometry_conversion`, publishing the canonical `/{ROBOT_NAME}/odometry_conversion/odometry` that the rest of the stack consumes.

### Known gaps

Two things are still required before a real circle can be flown end to end.

**Land detection.** `LandTask` only reports success when it sees `mavros_msgs/ExtendedState` with `landed_state == ON_GROUND`; there is no altitude fallback. `px4_interface` does not publish that message today, so a land command would run until timeout. PX4 publishes the equivalent information on `vehicle_land_detected`, and `px4_msgs/VehicleLandDetected` is already vendored in the workspace, so a small translation into `ExtendedState` closes this gap.

**Indoor position estimate.** PX4 will not arm indoors without an external position source. Motion-capture pose must reach EKF2 as external vision, and the relevant EKF2 parameters (`EKF2_EV_CTRL`, `EKF2_HGT_REF=Vision`, `EKF2_GPS_CTRL=0`) must be set per drone. Verify the frame convention on the ground before flying — a wrong sign flies the drone into a wall.

## Reading the comparison

Both runs produce the same metric keys, so `summary.txt` files can be read side by side:

| Metric | What a sim/hardware gap tells you |
| --- | --- |
| `cross_track_error_mean_m` | Steady-state tracking difference — usually gains tuned for sim dynamics |
| `cross_track_error_max_m` | Worst-case lag, typically at the tightest part of the curve |
| `path_rmse_m` | Overall fidelity; the headline number for a sim-to-real claim |
| `trajectory_execution_time_sim_s` | Speed mismatch; a large gap means the tracker is stalling on one side |
| `altitude_error_m` | Thrust/mass mismatch between the simulated and real airframe |

Expect hardware error to exceed sim error. The useful question is not whether they match exactly but whether the *ratio* stays stable as the pattern gets more demanding — that is what indicates the simulation is predictive rather than merely optimistic.
