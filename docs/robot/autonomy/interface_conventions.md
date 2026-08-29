# Interface Conventions Specification

**Spec version: v1.0.1** (semver — see [Versioning and deprecation](#versioning-and-deprecation))

This is the versioned specification of AirStack's **interchange points** — the
narrow waists where modules meet: canonical topic/service/action names,
message types, QoS profiles, TF frames and units, and rate classes. It
is the citable contract behind the topic tables of the
[Module Integration Checklist](integration_checklist.md); the checklist
remains the step-by-step integration workflow.

**Documentation, not enforcement.** This spec is documentation that modules
*default to* and conformance tests check — it is never input to any wiring
machinery. No schema compiles against it, no resolver reads it, and no code
is generated from it. A module's launch file exposes every topic endpoint as
a launch arg and **defaults it to the canonical name below**; in a
conventional stack, including the module therefore requires zero remaps, and
only deviations appear in stack entry files — which is what keeps them
skimmable. Enforcement is by test and by observation: the system-test suite
doubles as conformance tests, each stack's generated `wiring.md` is the
observed truth, and `airstack doctor --live` diffs reality against it.

**Conventions in this table are verified against the observed graph** — types
and QoS below come from `stacks/full_default/wiring.md` (the committed
wiring-snapshot of the running reference stack), not from memory. Where any
other document disagrees with a column here, the observed graph wins.

All names are relative to the robot namespace: canonical topic
`odometry_conversion/odometry` means `/{robot_name}/odometry_conversion/odometry`
at runtime (`ROBOT_NAME` namespacing is pushed by the launch preamble).

## Reading the tables

- **QoS** — publisher profile as observed: RELIABLE or BEST_EFFORT
  reliability; durability is VOLATILE unless noted (TRANSIENT_LOCAL is called
  out explicitly). QoS is named because it is a classic silent failure: a
  best-effort subscriber under a reliable-only publisher (or vice versa)
  receives *nothing*, with no error anywhere.
- **Rate class** — qualitative bands, not measured guarantees:
  `state` (~10–100 Hz), `sensor` (~10–30 Hz), `plan` (~0.1–2 Hz),
  `event` (on change / on command), `latched` (transient-local state).
- **Placement** — `onboard-only` marks interchanges that must never cross a
  machine boundary:
  the **controller** and the **safety executive** stay on the vehicle so link
  loss leaves it able to failsafe. `doctor` **hard-errors** when
  `control_setpoint` or trajectory-group names appear in any split stack's
  `bridge.yaml` — one of doctor's two enumerated hard gates; everywhere else
  it observes and reports.

---

## 1. `sensors/*` — sensor naming convention

Sensor topics are namespaced by sensor **id**: `sensors/<sensor_id>/<signal>`.
Sensor ids are first-class in the vehicle manifest, where
each id pairs the real driver with its sim representation; wiring snapshots
normalize driver nodes to these ids so sim baselines diff cleanly against
hardware bring-ups.

| Canonical name | Type | QoS | Rate class | Notes |
|---|---|---|---|---|
| `sensors/front_stereo/left/image_rect` | `sensor_msgs/msg/Image` | BEST_EFFORT | sensor | rectified; `right/` mirrors |
| `sensors/front_stereo/left/camera_info` | `sensor_msgs/msg/CameraInfo` | BEST_EFFORT | sensor | frame = the camera's optical frame |
| `sensors/ouster/point_cloud` | `sensor_msgs/msg/PointCloud2` | RELIABLE | sensor | *filtered* lidar cloud (post `lidar_point_cloud_filter`); raw is `sensors/ouster/point_cloud_raw` |
| `sensors/lidar/point_cloud` | `sensor_msgs/msg/PointCloud2` | — | sensor | generic lidar slot (sim publishes here when `ENABLE_LIDAR`) |

Units: SI throughout (meters, seconds); image encodings per ROS convention.

## 2. `odometry` — primary state estimate

| Canonical name | Type | QoS | Rate class | Placement |
|---|---|---|---|---|
| `odometry_conversion/odometry` **(v1 canonical)** | `nav_msgs/msg/Odometry` | RELIABLE | state | produced onboard |

> **v2 target:** plain `odometry` (`/{robot_name}/odometry`) is the intended
> canonical name; today every consumer (safety monitor, PID, DROAN,
> random_walk, trajectory controller, task servers) subscribes to
> `odometry_conversion/odometry`, so **v1 records reality**. Renaming is a
> spec-major change (see deprecation policy) with a coexistence window.

Frames/units: `pose` in the `map` frame (ENU, meters); `twist` in the body
frame (`child_frame_id`); yaw right-handed about +Z.

## 3. `global_map` — global world model

| Canonical name | Type | QoS | Rate class | Notes |
|---|---|---|---|---|
| `vdb_mapping/vdb_map_visualization` | `visualization_msgs/msg/Marker` | RELIABLE | plan | today's *de facto* map interchange — the reference global planner consumes it |
| `vdb_mapping/vdb_map_updates` / `_sections` / `_overwrites` | `vdb_mapping_interfaces/msg/UpdateGrid` | RELIABLE | plan | remote/split map synchronization |
| `vdb_mapping/vdb_map_pointcloud` | `sensor_msgs/msg/PointCloud2` | RELIABLE | plan | point-cloud export |

The map lives in the `map` frame. A structured (non-visualization) map
interchange is an acknowledged v2 candidate; v1 documents what the running
graph does.

## 4. `global_plan` — global waypoint path

| Canonical name | Type | QoS | Rate class |
|---|---|---|---|
| `global_plan` | `nav_msgs/msg/Path` | RELIABLE | plan |

Frames: `map` (ENU, meters). Producer: the global planner (onboard in
`full_default`, offboard in `lite_offload_global`); consumers: the local
planner, gossip, keepalive. **`global_plan` is the interchange that MAY cross
a machine boundary** — it is the entire point of the global-offload split.
Contrast §5.

## 5. `trajectory` group — the trajectory controller's contract — **onboard-only**

All names live under the `trajectory_controller/` namespace (served by
relative name inside it). **None of these may appear in a `bridge.yaml`** —
a doctor hard gate: `global_plan` crosses,
trajectory commands don't.

| Canonical name | Kind | Type | QoS | Rate class | Direction |
|---|---|---|---|---|---|
| `trajectory_controller/trajectory_override` | topic | `airstack_msgs/msg/TrajectoryXYZVYaw` | RELIABLE | event | any module → controller (replaces current trajectory) |
| `trajectory_controller/trajectory_segment_to_add` | topic | `airstack_msgs/msg/TrajectoryXYZVYaw` | RELIABLE | plan | local planner → controller (appends) |
| `trajectory_controller/set_trajectory_mode` | service | `airstack_msgs/srv/TrajectoryMode` | (service) | event | task servers → controller |
| `trajectory_controller/tracking_point` | topic | `airstack_msgs/msg/Odometry` | RELIABLE | state | controller → PID/planners (note: **airstack_msgs**, not nav_msgs, and not `PointStamped`) |
| `trajectory_controller/look_ahead` | topic | `airstack_msgs/msg/Odometry` | RELIABLE | state | controller → local planner |
| `trajectory_controller/trajectory_completion_percentage` | topic | `std_msgs/msg/Float32` | RELIABLE | state | controller → task servers |

**Safety floor:** command authority flows through the trajectory controller —
a module emitting `trajectory_override` inherits arming, safety monitoring,
and takeover for free (that is the selling point). Publishing
`tracking_point`/`look_ahead` from anything but the controller is
impersonation; `doctor --live` flags it loudly.

## 6. `control_setpoint` — controller → interface command — **onboard-only**

| Canonical name | Type | QoS | Rate class | Placement |
|---|---|---|---|---|
| `interface/cmd_roll_pitch_yawrate_thrust` | `mav_msgs/msg/RollPitchYawrateThrust` | RELIABLE | state | **onboard-only**; blessed publisher: the PID controller |
| `interface/cmd_pose`, `interface/cmd_velocity` | `geometry_msgs/msg/PoseStamped` / `TwistStamped` | RELIABLE | state | **onboard-only**; alternate command dialects into `robot_interface` |

`control_setpoint` is the spec name for this interchange point; the rows
above are its concrete v1 spellings. Never bridged, never remapped offboard —
the second doctor hard gate covers these alongside the trajectory group.

## 7. `interface_status` group — vehicle state out of the interface layer

| Canonical name | Type | QoS | Rate class |
|---|---|---|---|
| `interface/is_armed` | `std_msgs/msg/Bool` | RELIABLE | state |
| `interface/has_control` | `std_msgs/msg/Bool` | RELIABLE | state |
| `interface/mavros/state` | `mavros_msgs/msg/State` | RELIABLE, **TRANSIENT_LOCAL** | latched |
| `interface/mavros/extended_state` | `mavros_msgs/msg/ExtendedState` | RELIABLE, **TRANSIENT_LOCAL** | latched |
| `interface/mavros/global_position/global` | `sensor_msgs/msg/NavSatFix` | BEST_EFFORT | sensor |
| `interface/robot_command` | service `airstack_msgs/srv/RobotCommand` | (service) | event |

Late-joining subscribers rely on the TRANSIENT_LOCAL rows — a VOLATILE
subscriber there works, but a VOLATILE *re-publisher* silently loses the
latch.

## 8. `tasks/*` — task action servers

Every task executor's action server is exposed at `tasks/<task_name>`
(remapped there in launch; see the
[add-task-executor](https://github.com/castacks/AirStack/blob/develop/.agents/skills/add-task-executor/SKILL.md)
skill). All types come from `task_msgs`:

| Canonical name | Action type | v1 server |
|---|---|---|
| `tasks/takeoff` | `task_msgs/action/TakeoffTask` | takeoff_landing_planner (onboard) |
| `tasks/land` | `task_msgs/action/LandTask` | takeoff_landing_planner (onboard) |
| `tasks/navigate` | `task_msgs/action/NavigateTask` | droan (local planner, onboard) |
| `tasks/fixed_trajectory` | `task_msgs/action/FixedTrajectoryTask` | trajectory_controller pkg (onboard) |
| `tasks/exploration` | `task_msgs/action/ExplorationTask` | random_walk (global planner) |
| `tasks/semantic_search` | `task_msgs/action/SemanticSearchTask` | (module-provided) |
| `tasks/coverage` | `task_msgs/action/CoverageTask` | (defined in `task_msgs`; no shipped executor) |
| `tasks/chat` | `task_msgs/action/ChatTask` | (defined in `task_msgs`; no shipped executor) |

Related service: `takeoff_landing_planner/set_takeoff_landing_command`
(`airstack_msgs/srv/TakeoffLandingCommand`) — the GCS-facing takeoff/land
command. Task goals MAY cross machine boundaries (they are high-level
intents, not control): a split stack lists the crossing actions in its
`bridge.yaml`.

## 9. `safety` — safety executive — **onboard-only**

| Canonical name | Type | QoS | Rate class | Placement |
|---|---|---|---|---|
| `behavior/drone_safety_monitor/state_estimate_timed_out` | `std_msgs/msg/Bool` | RELIABLE | state | **onboard-only** |
| `behavior/drone_safety_monitor/command` | `std_msgs/msg/String` | RELIABLE | event | **onboard-only** |

The safety executive (drone_safety_monitor + the interface's takeover path)
is marked **onboard-only**: link loss must leave the robot
able to failsafe without any ground host in the loop.

## 10. `gossip` — multi-robot coordination

Gossip runs on its own DDS domain (default **99**) so peer discovery does not
flood per-robot domains; the dedicated gossip router bridges it (never the
robot↔GCS router — double-bridging amplifies).

| Canonical name | Type | QoS | Rate class | Notes |
|---|---|---|---|---|
| `/gossip/peers` | `coordination_msgs/msg/PeerProfile` | BEST_EFFORT | state | global (unnamespaced), domain 99 |
| `coordination/peer_registry` | `coordination_msgs/msg/PeerProfile` | RELIABLE, **TRANSIENT_LOCAL** | latched | per-robot registry output |

Custom payloads: see the
[attach-gossip-payload](https://github.com/castacks/AirStack/blob/develop/.agents/skills/attach-gossip-payload/SKILL.md)
skill.

## TF frames and units

| Frame | Parent | Convention |
|---|---|---|
| `world` | — | fixed origin; `world → map` published as a static identity by the launch preamble |
| `map` | `world` | ENU, meters; the planning/state frame (`odometry.pose`, `global_plan`, the map) |
| `base_link` (via `robot_description`) | `map` (through the state estimate) | body frame; `odometry.twist` lives here |

**ENU vs NED is the classic silent failure** at the PX4 boundary: MAVROS
performs the NED↔ENU conversion — everything ROS-side in this spec is ENU.
Angles in radians; right-handed; yaw about +Z.

`/tf` and `/tf_static` are wiring: the wiring snapshot deliberately keeps
them (frame plumbing drifts too).

---

## Versioning and deprecation

This spec is **public API** even though nothing compiles against it —
modules' launch-arg *defaults* and the conformance tests encode it.
Changing a canonical name, type, QoS profile, or frame
convention requires:

1. a **semver-major** bump of this spec,
2. a **coexistence window** (old and new names both served/accepted),
3. a short **written proposal in the registry repo** (`rfcs/` — the
   deprecation registry; until the registry repo exists, proposals live as
   GitHub Discussions on the AirStack repo).

Additions (new interchange points) are semver-minor and are discovered
through drift reports: three forks patching the same tap point = a missing
convention. The `doctor` hard-gate list (dep conflicts;
control/trajectory names in `bridge.yaml`) grows only through the same
proposal process.

## Change log

| Spec | Date | Change |
|---|---|---|
| v1.0.1 | 2026-08-25 | §8: added `tasks/coverage` and `tasks/chat` rows so the table covers all eight `task_msgs` actions; both are defined in `task_msgs` with no shipped executor. Documentation-only. |
| v1.0.0 | 2026-08-20 | Initial versioned spec, recorded from `full_default`'s observed wiring. Known v2 candidates: plain `odometry` as the canonical state topic; a structured `global_map` interchange. |
