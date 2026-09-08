# Candidate commits

Every commit in the window whose message matches the fix-keyword regex (plus all others, --all-messages). Read the diff before classifying; the message alone is `low` confidence.

## 06a1f6161d — 2026-06-10 — yikuan — no keyword

**Document ROS domain 1 convention in svg_ground_control tutorial**

https://github.com/castacks/AirStack/commit/06a1f6161dada08d50af121495148b030477c215

+15 / −6 in 1 files:

- `robot/ros_ws/src/svg_ground_control/README.md`

## df58f9a813 — 2026-06-10 — yikuan — no keyword

**Add svg_ground_control: multi-drone mocap ground controller with CBF placeholder**

https://github.com/castacks/AirStack/commit/df58f9a8132ab833fec03f53121f1e7639bccbe6

```
Central swarm commander that flies N-1 drones to hover targets while one
drone is hand-teleoperated as a moving obstacle. All commanded velocities
pass through a CBF collision-safety-filter placeholder shaped to drop in
the drone_soccer velocity-CBF (same filter_velocities signature).

- swarm_commander: 20 Hz loop, per-drone FSM (arm/offboard/ascend/hover/
  land), roles hover|teleop|external, takeoff/land/hold Trigger services,
  stale-state and teleop-timeout failsafes
- mocap_bridge: N x /drone_i/pose (configurable template) -> per-drone
  px4_interface visual_odometry_in (ENU; px4_interface does FRD conversion)
- keyboard_teleop: world-frame velocity teleop for the obstacle drone
- drone_interface.launch.xml: per-drone px4_interface stack (hardware,
  namespaced uXRCE-DDS) / sim_drone_interface.launch.xml +
  launch_sim_interfaces.sh: per-drone MAVROS stacks on one domain (sim)
- svg_multi_drone_single_domain.py: Isaac/Pegasus multi-PX4 spawner with
  all drones on one ROS domain, namespaced drone_1..N
- configs for sim (MAVROS topics) and hardware (fmu topics + mocap)

Validated: colcon build + closed-loop functional test (fake drones
integrating commanded velocities) covering takeoff, ascent, hover hold,
teleop, teleop timeout, land, disarm.
```

+1335 / −0 in 17 files:

- `robot/ros_ws/src/svg_ground_control/README.md`
- `robot/ros_ws/src/svg_ground_control/config/swarm_real.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_sim.yaml`
- `robot/ros_ws/src/svg_ground_control/launch/drone_interface.launch.xml`
- `robot/ros_ws/src/svg_ground_control/launch/ground_control.launch.py`
- `robot/ros_ws/src/svg_ground_control/launch/sim_drone_interface.launch.xml`
- `robot/ros_ws/src/svg_ground_control/package.xml`
- `robot/ros_ws/src/svg_ground_control/resource/svg_ground_control`
- `robot/ros_ws/src/svg_ground_control/scripts/launch_sim_interfaces.sh`
- `robot/ros_ws/src/svg_ground_control/setup.cfg`
- `robot/ros_ws/src/svg_ground_control/setup.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/__init__.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/cbf_filter.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/keyboard_teleop.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/mocap_bridge.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/swarm_commander.py`
- `simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py`

## 2025934374 — 2026-06-11 — yikuan — KEYWORD failure,fix

**Fix squeeze: intruder was CBF-filtered, causing stall/retreat at the gap**

https://github.com/castacks/AirStack/commit/2025934374b269af51564dadbe5c30a2276b64f8

```
Root cause of the reported failure (intruder turning back instead of
squeezing through): only teleop drones were CBF-exempt, so an autonomous
squeeze intruder was filtered like everyone else. The pair-constraint
gradient points away from the holders while approaching, so the filter
pushes the intruder backwards as the gap tightens — with real PX4 velocity
lag it stalls or retreats instead of forcing the holders to yield.

Fix: scenarios can designate CBF-exempt obstacle drones
(Scenario.cbf_exempt_indices); the squeeze intruder is exempt by default
(squeeze_intruder_cbf_exempt param). The commander restores exempt rows
after filtering, same as teleop rows, and no longer lists exempt drones in
the "CBF active" log.

Validated with a new lag-model functional test (first-order velocity
response, tau up to 1.2 s) plus a recorded rosbag (bags/ now gitignored):
crossing now takes 4.9 s for 3 m (nominal-speed, zero retreat, was ~30 s
marginal before), holders yield 0.39 m and return, holder-holder
separation never below its 1.38 m rest gap.
```

+252 / −15 in 7 files:

- `.gitignore`
- `robot/ros_ws/src/svg_ground_control/config/squeeze_3drone.yaml`
- `robot/ros_ws/src/svg_ground_control/experiment.md`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/scenarios.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/swarm_commander.py`
- `robot/ros_ws/src/svg_ground_control/test/functional_squeeze_lag_test.py`
- `robot/ros_ws/src/svg_ground_control/test/test_scenarios.py`

## 6a5f7d0b65 — 2026-06-11 — yikuan — no keyword

**Add position monitoring commands to experiment.md**

https://github.com/castacks/AirStack/commit/6a5f7d0b65b88eb96d3488a0ff118183cde8342e

+12 / −0 in 1 files:

- `robot/ros_ws/src/svg_ground_control/experiment.md`

## 850e795836 — 2026-06-11 — yikuan — no keyword

**Make squeeze geometry explicit: configurable holder posts and intruder waypoints**

https://github.com/castacks/AirStack/commit/850e795836be81e5a7176b7c57eb55fc7497d02e

```
Replaces the derived gap_factor/run_length geometry with two direct
parameters: squeeze_holder_positions (the two steady drones' posts) and
squeeze_intruder_waypoints (the two endpoints the moving drone shuttles
between, starting at A). The commander logs the resulting geometry at
startup and rejects posts closer than the 2r keep-out.

Validated: 11 unit tests + closed-loop functional squeeze test (holders
yielded 0.21 m, min pair distance held at 2r).
```

+123 / −58 in 8 files:

- `robot/ros_ws/src/svg_ground_control/config/squeeze_3drone.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_real.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_sim.yaml`
- `robot/ros_ws/src/svg_ground_control/experiment.md`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/scenarios.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/swarm_commander.py`
- `robot/ros_ws/src/svg_ground_control/test/functional_squeeze_test.py`
- `robot/ros_ws/src/svg_ground_control/test/test_scenarios.py`

## 6e4b78d968 — 2026-06-11 — yikuan — no keyword

**Add tmux pane-close key to experiment.md**

https://github.com/castacks/AirStack/commit/6e4b78d9686334c797fb394ca40bb48d2b594e54

+1 / −1 in 1 files:

- `robot/ros_ws/src/svg_ground_control/experiment.md`

## 2bb81f053c — 2026-06-11 — yikuan — no keyword

**Add tmux single-terminal workflow to experiment.md**

https://github.com/castacks/AirStack/commit/2bb81f053cf333f2f98174c8623edd5127ba1405

+15 / −0 in 1 files:

- `robot/ros_ws/src/svg_ground_control/experiment.md`

## 11a1237f11 — 2026-06-11 — yikuan — no keyword

**Implement real CBF filter, drone_soccer scenarios, squeeze test, NatNet bridge**

https://github.com/castacks/AirStack/commit/11a1237f11ce23612068ea6fdf3229058a4f9091

```
Replaces the CBF placeholder with the actual velocity-CBF from
~/drone_soccer (pairwise barrier + hybrid parallel/Gauss-Seidel Dykstra
projection, constraint pruning, emergency push-apart fallback) and ports
its scenario suite to the swarm commander.

- cbf_filter.py: verbatim port of drone_soccer/cbf.py (same public API)
- scenarios.py: hover, random_walk, random_goals, head_on, antipodal
  (ported) + new squeeze profile: two holders goal-track posts 2.5r apart
  while an intruder flies through the gap; CBF makes the holders yield
  and return
- swarm_commander: scenario-driven nominal control with mission lifecycle
  (takeoff -> start -> hold/land), comma-separated teleop_drones list
  (CBF-exempt operator-driven obstacles; empty = fully autonomous),
  external_drones (tracked, never commanded), per-drone teleop topics
- keyboard_teleop: -p drone:=<name> selects which drone to drive
- real_interfaces.launch.py: all per-drone px4_interface stacks from one
  command (drones:=drone_1,drone_2,...)
- natnet_ros2: OptiTrack NatNet bridge brought over from origin/develop
  (PR #359); body_id -1 streams all Motive rigid bodies as per-name
  PoseStamped topics consumed by mocap_bridge
- config: squeeze_3drone.yaml profile; sim/real configs updated
- experiment.md: canonical maintained command reference (fresh-terminal
  blocks, domain-1 convention, troubleshooting table)

Validated: 10 unit tests (drone_soccer kinematic CBF suite + scenario
tests incl. squeeze rollout) and a closed-loop ROS functional test where
fake drones integrate commanded velocities: intruder crossed, holders
yielded 0.21 m and returned, min pair distance held at exactly 2r.
```

+4838 / −308 in 32 files:

- `robot/ros_ws/src/perception/natnet_ros2/.gitignore`
- `robot/ros_ws/src/perception/natnet_ros2/CMakeLists.txt`
- `robot/ros_ws/src/perception/natnet_ros2/README.md`
- `robot/ros_ws/src/perception/natnet_ros2/config/natnet_config.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/config/vision_pose_converter.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/env-hooks/natnet_library_path.dsv.in`
- `robot/ros_ws/src/perception/natnet_ros2/include/natnet_ros2/natnet_client_adapter.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/include/natnet_ros2/natnet_logic.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/launch/natnet_ros2.launch.py`
- `robot/ros_ws/src/perception/natnet_ros2/launch/vision_pose_converter.launch.xml`
- `robot/ros_ws/src/perception/natnet_ros2/package.xml`
- `robot/ros_ws/src/perception/natnet_ros2/scripts/download-natnet-sdk.sh`
- `robot/ros_ws/src/perception/natnet_ros2/src/natnet_client_adapter.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/src/natnet_ros2_node.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/src/vision_pose_converter_node.py`
- `robot/ros_ws/src/perception/natnet_ros2/test/fake_natnet_client.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/test/test_natnet_logic.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/test/test_natnet_ros2.py`
- `robot/ros_ws/src/svg_ground_control/README.md`
- `robot/ros_ws/src/svg_ground_control/config/squeeze_3drone.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_real.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_sim.yaml`
- `robot/ros_ws/src/svg_ground_control/experiment.md`
- `robot/ros_ws/src/svg_ground_control/launch/ground_control.launch.py`
- `robot/ros_ws/src/svg_ground_control/launch/real_interfaces.launch.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/cbf_filter.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/keyboard_teleop.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/scenarios.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/swarm_commander.py`
- `robot/ros_ws/src/svg_ground_control/test/functional_squeeze_test.py`
- `robot/ros_ws/src/svg_ground_control/test/test_cbf.py`
- `robot/ros_ws/src/svg_ground_control/test/test_scenarios.py`

## 6d0e206e80 — 2026-06-12 — yikuan — KEYWORD fix

**Fix squeeze in sim: per-drone PX4 local origins broke the shared world frame**

https://github.com/castacks/AirStack/commit/6d0e206e809559e4ee188caa3861283a9a6feede

```
Bag analysis of squeeze_191528 (Isaac, 3 drones) showed every drone flying
its commanded geometry PERFECTLY in its own frame and wrongly in the world:
each PX4 SITL's EKF origin is its spawn point, so the commander was mixing
three different local frames as one. drone_1 'held its post' 2 m away from
it, the intruder ran its whole shuttle 2 m right of the gap (closest
physical approach to the holders' midpoint: 1.45 m), and the CBF reacted to
phantom geometry while missing a real 0.49 m near-miss.

Fix: new drone_position_offsets parameter (flat 3N) added to incoming
odometry to translate each drone's local origin into the shared world
frame. Sim configs carry the Isaac spawn offsets ([-2,0,0, 0,0,0, 2,0,0]);
hardware config keeps zeros (mocap anchors all EKFs to one origin). The
commander logs the offsets, or warns when all-zero. Squeeze intruder
waypoint A moved to the +x side so the (non-exempt) takeoff ascent doesn't
have to cross the impassable gap from its x=+2 spawn.

Both functional tests now spawn fake drones at the true Isaac layout and
publish odometry in per-drone local frames, validating the correction end
to end (ideal and tau=1.0s lag): holders reach the real posts, intruder
crosses the real gap with zero retreat, holders yield up to 0.51 m.
```

+92 / −28 in 7 files:

- `robot/ros_ws/src/svg_ground_control/config/squeeze_3drone.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_real.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_sim.yaml`
- `robot/ros_ws/src/svg_ground_control/experiment.md`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/swarm_commander.py`
- `robot/ros_ws/src/svg_ground_control/test/functional_squeeze_lag_test.py`
- `robot/ros_ws/src/svg_ground_control/test/functional_squeeze_test.py`

## 4479ea7c6a — 2026-06-12 — yikuan — KEYWORD fix

**Fix rosbag recording path in experiment.md: record into the mounted ros_ws**

https://github.com/castacks/AirStack/commit/4479ea7c6aa93e9e6e3d1f0672864f0a8d10ecad

+10 / −2 in 1 files:

- `robot/ros_ws/src/svg_ground_control/experiment.md`

## c105a20de9 — 2026-06-15 — yikuan — no keyword

**optimized some parameters**

https://github.com/castacks/AirStack/commit/c105a20de9c0b23f8879233eb4b732a89329734f

+16 / −14 in 8 files:

- `.env`
- `AGENTS.md`
- `robot/ros_ws/src/svg_ground_control/config/goal_single.yaml`
- `robot/ros_ws/src/svg_ground_control/config/goal_tracking.yaml`
- `robot/ros_ws/src/svg_ground_control/config/hybrid_squeeze.yaml`
- `robot/ros_ws/src/svg_ground_control/config/squeeze_3drone.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_real.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_sim.yaml`

## ff2c23afcd — 2026-06-15 — yikuan — no keyword

**Document RViz hand-carry / preflight workflow (no flight needed)**

https://github.com/castacks/AirStack/commit/ff2c23afcd6b5d5dfc2dc98ba1834fe52e7ecfef

+17 / −0 in 1 files:

- `robot/ros_ws/src/svg_ground_control/experiment.md`

## 5967940284 — 2026-06-15 — yikuan — KEYWORD regression

**Add goal-tracking, hybrid sim/real routing, geofence, RViz viz + tests**

https://github.com/castacks/AirStack/commit/59679402842de6ad925476eb86055c15297f9ecb

```
Component-test features for the real-drone bring-up:

- goal scenario (GoalScenario): each drone seeks a per-drone goal set live
  via /svg/{name}/goal_command (PoseStamped) and /svg/{name}/speed_command
  (Float32), CBF-filtered. Backs single- and multi-drone tracking tests.

- per-drone sim/real routing (drone_modes "sim"/"real"): routes each drone's
  velocity_command + robot_command to MAVROS (/interface/) or px4_interface
  (/fmu/) while one CBF sees all drones. Lets a run mix real and simulated
  drones (e.g. squeeze with real holders + simulated intruder). Backward
  compatible: empty drone_modes uses the existing single template set.

- geofence: fence_enabled + fence_min/max latch a swarm-wide freeze if any
  ACTIVE drone leaves the box (climb-out/landing exempt); ~/reset_fence
  clears it; start is blocked while latched.

- RViz: /svg/viz/markers MarkerArray of every drone's offset-corrected WORLD
  position (real=red, sim=cyan, teleop=yellow, external=gray, frozen=orange)
  + safety spheres, labels, goals, fence box; config/svg_drones.rviz.

- configs: goal_single, goal_tracking, hybrid_squeeze; fence params added to
  swarm_sim/real/squeeze; rviz installed; std_msgs/visualization_msgs deps.

- tests: functional_{single_goal,multi_goal,hybrid,fence}_test.py +
  GoalScenario unit test. All pass (13 unit + 6 functional), squeeze
  regression intact, multi-goal CBF held min distance at 2r, hybrid routing
  verified (holders on /fmu/, intruder on /interface/), fence latch/reset OK.

- experiment.md rewritten as a full guide: AirStack + SVG architecture,
  topic/service wiring table, per-terminal/command walkthrough, component
  tests (Part B), hybrid (Part C), hardware (Part D), RViz, geofence,
  rosbag, troubleshooting.
```

+1624 / −247 in 18 files:

- `robot/ros_ws/src/svg_ground_control/README.md`
- `robot/ros_ws/src/svg_ground_control/config/goal_single.yaml`
- `robot/ros_ws/src/svg_ground_control/config/goal_tracking.yaml`
- `robot/ros_ws/src/svg_ground_control/config/hybrid_squeeze.yaml`
- `robot/ros_ws/src/svg_ground_control/config/squeeze_3drone.yaml`
- `robot/ros_ws/src/svg_ground_control/config/svg_drones.rviz`
- `robot/ros_ws/src/svg_ground_control/config/swarm_real.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_sim.yaml`
- `robot/ros_ws/src/svg_ground_control/experiment.md`
- `robot/ros_ws/src/svg_ground_control/package.xml`
- `robot/ros_ws/src/svg_ground_control/setup.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/scenarios.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/swarm_commander.py`
- `robot/ros_ws/src/svg_ground_control/test/functional_fence_test.py`
- `robot/ros_ws/src/svg_ground_control/test/functional_hybrid_test.py`
- `robot/ros_ws/src/svg_ground_control/test/functional_multi_goal_test.py`
- `robot/ros_ws/src/svg_ground_control/test/functional_single_goal_test.py`
- `robot/ros_ws/src/svg_ground_control/test/test_scenarios.py`

## d8b89dd9e9 — 2026-06-18 — yikuan — no keyword

**SVG: per-task CBF-exempt list, any-drone-any-mode configs, Isaac real-drone avatars**

https://github.com/castacks/AirStack/commit/d8b89dd9e98ca9b58d1607ce7a1a5d9a60e68d82

```
Decouple CBF-exemption from the teleop role: a new cbf_exempt_drones config
list names the drones the filter leaves uncorrected (still obstacles for
everyone else). Works for auto or teleop drones and unions with the squeeze
scenario's built-in intruder exemption. Teleop is no longer auto-exempt.

- swarm_commander: declare/parse/validate cbf_exempt_drones and apply it in the
  control loop; report it in the startup log; __init__ takes kwargs so tests can
  inject parameter overrides.
- configs: expose mode/role/cbf-exempt on every task (incl. single-drone
  goal_single); hybrid_squeeze = real holders + sim intruder + exempt, with a
  mocap_bridge section for use_mocap.
- isaac: DRONE_MODES spawns a visual avatar (no SITL) per real drone and
  teleports it each step to its odometry, so real drones appear in the viewport.
- experiment.md: restructured A (sim) -> B (connect+verify a real drone) ->
  C (one 'any drone any mode' task framework) -> D (first-flight safety).
- test: add test_exempt.py (cbf_exempt_drones parsing/validation + teleop
  decoupling).
```

+592 / −154 in 10 files:

- `robot/ros_ws/src/svg_ground_control/config/goal_single.yaml`
- `robot/ros_ws/src/svg_ground_control/config/goal_tracking.yaml`
- `robot/ros_ws/src/svg_ground_control/config/hybrid_squeeze.yaml`
- `robot/ros_ws/src/svg_ground_control/config/squeeze_3drone.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_real.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_sim.yaml`
- `robot/ros_ws/src/svg_ground_control/experiment.md`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/swarm_commander.py`
- `robot/ros_ws/src/svg_ground_control/test/test_exempt.py`
- `simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py`

## 736b0564e7 — 2026-06-20 — yikuan — KEYWORD fail

**SVG docs + VOXL2 real-drone provisioning script**

https://github.com/castacks/AirStack/commit/736b0564e7e61747d99027e927cc9a646e04dbc4

```
- Add scripts/voxl_setup_real_drone.sh: idempotent VOXL2 (ModalAI) comms
  provisioning (repoint microdds_client at the ground PC, namespace topics,
  pin the DDS domain, disable the onboard agent, restart + verify). Field-based
  awk for mawk/busybox portability; verifies the edit landed and restores on fail.
- experiment.md Part B: B0 Wi-Fi/DHCP recovery, B1 rebuilt around the script
  (getting-it-on + ground-side verify), B2 -> MicroXRCEAgent udp4 -p 8888 -v4,
  new B6 VOXL2 diagnostics cheat sheet, and natnet docs updated for the vendored
  package (--symlink-install, /<body>/pose topics, serverIP/clientIP args).
```

+438 / −40 in 2 files:

- `robot/ros_ws/src/svg_ground_control/experiment.md`
- `robot/ros_ws/src/svg_ground_control/scripts/voxl_setup_real_drone.sh`

## c1f69fe4dd — 2026-06-20 — yikuan — no keyword

**Vendor natnet_ros2 (L2S-lab fork) replacing the old package**

https://github.com/castacks/AirStack/commit/c1f69fe4dde7c7f2d3488b2f8f311728eb7dbeb0

```
Clone of L2S-lab/natnet_ros2 @883b095 with the nested .git removed (vendored,
not a submodule, so local launch edits are preserved). Launch defaults set for
this rig: pub_rigid_body=true and serverIP/clientIP. Old AirStack-specific files
(vision_pose_converter, natnet_logic/adapter, in-tree tests) dropped. NatNet SDK
stays gitignored and is auto-downloaded on first build; build needs
--symlink-install.
```

+8621 / −2790 in 45 files:

- `robot/ros_ws/src/perception/natnet_ros2/.github/workflows/AMD64-humble.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/.github/workflows/AMD64-jazzy.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/.github/workflows/ARM64-humble.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/.github/workflows/ARM64-jazzy.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/.gitignore`
- `robot/ros_ws/src/perception/natnet_ros2/AUTHORS`
- `robot/ros_ws/src/perception/natnet_ros2/CHANGELOG.md`
- `robot/ros_ws/src/perception/natnet_ros2/CMakeLists.txt`
- `robot/ros_ws/src/perception/natnet_ros2/LICENSE`
- `robot/ros_ws/src/perception/natnet_ros2/README.md`
- `robot/ros_ws/src/perception/natnet_ros2/config/conf_autogen.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/config/initiate.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/config/natnet_config.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/config/vision_pose_converter.yaml`
- `robot/ros_ws/src/perception/natnet_ros2/deps/NatNetSDK/.gitkeep`
- `robot/ros_ws/src/perception/natnet_ros2/env-hooks/natnet_library_path.dsv.in`
- `robot/ros_ws/src/perception/natnet_ros2/img/streaming.png`
- `robot/ros_ws/src/perception/natnet_ros2/img/ui-1.png`
- `robot/ros_ws/src/perception/natnet_ros2/include/natnet_ros2/natnet_client_adapter.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/include/natnet_ros2/natnet_logic.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/include/natnet_ros2/natnet_ros2.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/include/natnet_ros2/nn_filter.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/include/natnet_ros2/object_data.hpp`
- `robot/ros_ws/src/perception/natnet_ros2/install_sdk.sh`
- `robot/ros_ws/src/perception/natnet_ros2/launch/gui_natnet_ros2.launch.py`
- `robot/ros_ws/src/perception/natnet_ros2/launch/natnet_ros2.launch.py`
- `robot/ros_ws/src/perception/natnet_ros2/launch/vision_pose_converter.launch.xml`
- `robot/ros_ws/src/perception/natnet_ros2/natnet_ros2_py/__init__.py`
- `robot/ros_ws/src/perception/natnet_ros2/natnet_ros2_py/node_module.py`
- `robot/ros_ws/src/perception/natnet_ros2/package.xml`
- `robot/ros_ws/src/perception/natnet_ros2/scripts/download-natnet-sdk.sh`
- `robot/ros_ws/src/perception/natnet_ros2/scripts/helper_node_r2.py`
- `robot/ros_ws/src/perception/natnet_ros2/src/marker_poses_server.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/src/natnet_client_adapter.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/src/natnet_ros2.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/src/natnet_ros2_node.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/src/nn_filter.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/src/vision_pose_converter_node.py`
- `robot/ros_ws/src/perception/natnet_ros2/srv/MarkerPoses.srv`
- `robot/ros_ws/src/perception/natnet_ros2/test/fake_natnet_client.hpp`
- … 5 more

## d3ed7dc552 — 2026-06-21 — yikuan — no keyword

**Clarify mocap_bridge VIO topic params: which applies in which px4_vio_mode**

https://github.com/castacks/AirStack/commit/d3ed7dc55248be4afecb77ec25f15e79b709bf42

+12 / −9 in 1 files:

- `robot/ros_ws/src/svg_ground_control/config/swarm_real.yaml`

## f6441c5f5f — 2026-06-21 — yikuan — no keyword

**Real-drone external vision: mocap_bridge direct VehicleOdometry path**

https://github.com/castacks/AirStack/commit/f6441c5f5f84ea18a2e1202c0ce9efc85e4733f4

```
mocap_bridge can now publish px4_msgs/VehicleOdometry straight to
/{name}/fmu/in/vehicle_visual_odometry (px4_vio_mode: direct, default),
mirroring the proven model_ai_tfpub.cpp: timestamp 0 (uXRCE client restamps
with PX4 HRT — a ground-clock stamp gets rejected by EKF2), quality 100,
velocity NaN (fuse pose only). Frame selectable via px4_vio_frame:
enu_to_ned (standard) or modalai_flip (the reference transform), since
natnet republishes Motive's raw frame, not ROS-ENU. 'via_interface' keeps
the old nav_msgs/Odometry -> px4_interface path.

- swarm_real.yaml: expose px4_vio_mode/px4_vio_frame/vio_quality; also move
  the misplaced geofence block from under mocap_bridge back under
  swarm_commander (it was inert on the real config)
- px4_msgs added as a package dependency
- experiment.md B4b: external-vision -> EKF2 verification (EKF2_EV_CTRL etc.,
  the best_effort echo-QoS gotcha, the frame hand-check) and troubleshooting
  rows for won't-arm / silent /fmu/out echo / mirrored frame
- vendor the reference model_ai_tfpub.cpp + pid_path_tracker_px4.cpp
```

+1128 / −34 in 6 files:

- `robot/ros_ws/src/svg_ground_control/config/swarm_real.yaml`
- `robot/ros_ws/src/svg_ground_control/experiment.md`
- `robot/ros_ws/src/svg_ground_control/package.xml`
- `robot/ros_ws/src/svg_ground_control/reference/model_ai_tfpub.cpp`
- `robot/ros_ws/src/svg_ground_control/reference/pid_path_tracker_px4.cpp`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/mocap_bridge.py`

## a46f04b563 — 2026-06-23 — yikuan — no keyword

**goal tracking verified**

https://github.com/castacks/AirStack/commit/a46f04b563a52b8f58aa761c99edccc938caa134

+162 / −26 in 7 files:

- `robot/docker/.bashrc`
- `robot/docker/Dockerfile.robot`
- `robot/docker/docker-compose.yaml`
- `robot/ros_ws/src/interface/robot_interface/src/robot_interface_node.cpp`
- `robot/ros_ws/src/perception/natnet_ros2/launch/natnet_ros2.launch.py`
- `robot/ros_ws/src/svg_ground_control/config/goal_single.yaml`
- `robot/ros_ws/src/svg_ground_control/experiment.md`

## 94425c2518 — 2026-06-23 — yikuan — no keyword

**Troubleshooting: takeoff/land work but won't move to a goal (start not succeeded)**

https://github.com/castacks/AirStack/commit/94425c2518c714daa1cba3212da5d6305ce27ae6

+1 / −0 in 1 files:

- `robot/ros_ws/src/svg_ground_control/experiment.md`

## 564d43e454 — 2026-08-11 — yikuan — KEYWORD fixed

**CBF external-velocity reactivity, formation profiles, C5 RC-intruder config**

https://github.com/castacks/AirStack/commit/564d43e4542c1e2dd4cd24cebeb21b3fb3843ad2

```
- cbf_filter: fixed (non-adjustable) rows — external/exempt obstacles are
  pinned at the velocity they will actually fly; movable drones absorb the
  full correction (backward compatible, verified bit-identical w/o fixed)
- swarm_commander: feed external drones' measured EKF velocity into the CBF
  (new cbf_external_velocity_gain), pin exempt rows pre-solve; holders now
  yield BEFORE an RC intruder reaches the 2r barrier
- formation profiles: named goal sets in config, one-command swarm
  retargeting via /svg/formation_command, incl. reserved 'next' cycling
- new squeeze_rc_intruder.yaml (C5: RC-flown external intruder) with
  documented CBF reactivity knobs
- voxl_setup_real_drone.sh: restart watchdog instead of enable --now (stale
  watchdog kept resurrecting the client with the old agent IP)
- GoalScenario approach gain 2.0 -> 1.5; natnet default serverIP .5
- tests: fixed-row CBF (4) + formation profiles (9); experiment.md updated
```

+999 / −130 in 15 files:

- `robot/ros_ws/src/perception/natnet_ros2/launch/natnet_ros2.launch.py`
- `robot/ros_ws/src/svg_ground_control/config/goal_single.yaml`
- `robot/ros_ws/src/svg_ground_control/config/goal_tracking.yaml`
- `robot/ros_ws/src/svg_ground_control/config/hybrid_squeeze.yaml`
- `robot/ros_ws/src/svg_ground_control/config/squeeze_3drone.yaml`
- `robot/ros_ws/src/svg_ground_control/config/squeeze_rc_intruder.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_real.yaml`
- `robot/ros_ws/src/svg_ground_control/config/swarm_sim.yaml`
- `robot/ros_ws/src/svg_ground_control/experiment.md`
- `robot/ros_ws/src/svg_ground_control/scripts/voxl_setup_real_drone.sh`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/cbf_filter.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/scenarios.py`
- `robot/ros_ws/src/svg_ground_control/svg_ground_control/swarm_commander.py`
- `robot/ros_ws/src/svg_ground_control/test/test_cbf.py`
- `robot/ros_ws/src/svg_ground_control/test/test_formations.py`
