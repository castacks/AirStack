# SVG Ground Control

Central multi-drone ground controller for mocap flight with a CBF
collision-safety-filter placeholder. N−1 drones hover at configured targets
while one drone is hand-teleoperated as a moving obstacle; every commanded
velocity passes through `svg_ground_control/cbf_filter.py`, which is a
drop-in slot for the velocity-CBF filter from `~/drone_soccer`
(`drone_soccer/cbf.py` — same signature, copy it over this file).

## Architecture

```
                          ┌────────────────────────────────────────────┐
 mocap /drone_i/pose ──▶  │ mocap_bridge   (hardware only)             │
                          │   → /drone_i/fmu/visual_odometry_in        │
                          └────────────────────────────────────────────┘
                          ┌────────────────────────────────────────────┐
 /drone_i/odometry_       │ swarm_commander  (20 Hz)                   │
 conversion/odometry ──▶  │  nominal: hover P-ctrl | teleop input      │
 /svg/teleop_command ──▶  │  → cbf_filter.filter_velocities()  [TODO]  │
                          │  → /drone_i/.../velocity_command           │
                          │  services: ~/takeoff ~/land ~/hold         │
                          └────────────────────────────────────────────┘
                                   │ per-drone robot_interface
                          sim: MAVROS          real: px4_interface (uXRCE-DDS)
```

The commander talks only to the AirStack `robot_interface` abstraction, so
sim and hardware differ **only in the topic templates** in the config YAML
(`config/swarm_sim.yaml` vs `config/swarm_real.yaml`).

Drone roles (config): `hover` (CBF-filtered station-keeping), `teleop`
(operator-driven obstacle, CBF-exempt), `external` (tracked for the filter,
never commanded — e.g. RC-flown).

## Quick start — simulation (3 drones, 1 teleop)

Terminal 1 — Isaac Sim (inside the isaac-sim container):

```bash
NUM_ROBOTS=3 ./python.sh /isaac-sim/launch_scripts/svg_multi_drone_single_domain.py
```

Terminal 2 — robot container, MAVROS interfaces (one per drone, single domain):

```bash
airstack connect robot   # then inside:
bws && sws
export ROS_DOMAIN_ID=0   # must match SVG_DOMAIN_ID of the sim script
./src/svg_ground_control/scripts/launch_sim_interfaces.sh 3
```

Terminal 3 — ground controller:

```bash
ros2 launch svg_ground_control ground_control.launch.py
```

Terminal 4 — teleop (own TTY) and flight:

```bash
ros2 run svg_ground_control keyboard_teleop      # w/s a/d r/f, space=stop
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
# ... fly the obstacle drone into the hover formation ...
ros2 service call /swarm_commander/land std_srvs/srv/Trigger
```

`/swarm_commander/hold` freezes every airborne drone at its current position
(panic button — also converts the teleop drone to hover).

## Quick start — hardware (Starling 2 Max + mocap)

Per drone (once): namespace the uXRCE-DDS client on the VOXL —
`uxrce_dds_client start -n drone_i` — so PX4 topics appear as
`/drone_i/fmu/...`; set EKF2 to fuse external vision (`EKF2_EV_CTRL`), and
configure RC kill switch + offboard-loss failsafe.

Ground PC (one container/shell on the shared domain, workspace sourced):

```bash
# one interface stack per drone
ros2 launch svg_ground_control drone_interface.launch.xml drone_name:=drone_1
ros2 launch svg_ground_control drone_interface.launch.xml drone_name:=drone_2
ros2 launch svg_ground_control drone_interface.launch.xml drone_name:=drone_3

# commander + mocap bridge
ros2 launch svg_ground_control ground_control.launch.py \
    config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/swarm_real.yaml \
    use_mocap:=true
```

Preflight: `ros2 topic echo /drone_1/odometry_conversion/odometry` and move
the drone by hand — position must track the mocap. Then takeoff/land/teleop
exactly as in sim.

## Dropping in the real CBF

Replace `svg_ground_control/cbf_filter.py` with
`~/drone_soccer/drone_soccer/cbf.py` (plus its `solve_safe_commands` /
`build_collision_constraints` internals — the file is self-contained). The
commander calls

```python
filter_velocities(nominal, positions, safety_radius, max_speed, alpha)
```

and uses `result.velocities` / `result.used_emergency_stop`; nothing else
needs to change. Until then the placeholder only enforces the speed cap —
**it does not prevent collisions**.

## Safety notes

- This stack bypasses `drone_safety_monitor`; PX4 failsafes and the RC kill
  switch are the safety net. Configure them before flying.
- Commanded drones with stale odometry (> `state_timeout_s`) are sent zero
  velocity (offboard position-hold-ish), not left on their last command.
- Teleop input times out to zero after `teleop_timeout_s`.
