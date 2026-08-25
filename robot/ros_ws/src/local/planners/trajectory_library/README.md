# Trajectory Library

Contact: John Keller

`trajectory_library` is a C++ library (no node of its own) for generating and manipulating candidate trajectories for obstacle-avoidance planners. It provides the `Trajectory`/`Waypoint` classes used throughout the local layer to interpolate, transform, trim, merge, and visualize waypoint paths, plus a `TrajectoryLibrary` class that loads a *library* of candidate trajectory generators from a YAML config file — the file the DROAN local planner points its `trajectory_library_config` parameter at. Every trajectory converts to/from [`airstack_msgs/msg/TrajectoryXYZVYaw`](../../../../../../common/ros_packages/msgs/airstack_msgs/README.md), the trajectory-controller command type (see the [Interface Conventions Specification §5](../../../../../../docs/robot/autonomy/interface_conventions.md)).

## Core classes

Defined in [`include/trajectory_library/trajectory_library.hpp`](include/trajectory_library/trajectory_library.hpp), implemented in [`src/trajectory_library.cpp`](src/trajectory_library.cpp):

| Class | What it is | Key operations |
|---|---|---|
| `Waypoint` | One sample: position, yaw, velocity, acceleration, jerk, time | `interpolate()`, `as_odometry_msg()` (→ `airstack_msgs/Odometry`) |
| `Trajectory` | A timed waypoint sequence in a TF frame; constructible from `TrajectoryXYZVYaw` or `nav_msgs/Path` (positions only, velocity 0) | closest-point queries, `get_waypoint(time)`, `get_odom(time)`, `to_frame()`, `merge()`, `trim()`, `get_trimmed_trajectory_between_distances()`, `get_reversed_trajectory()`, `get_markers()` (RViz), `get_TrajectoryXYZVYaw_msg()` |
| `TrajectoryLibrary` | Loads a set of candidate-trajectory generators from a YAML file | `get_static_trajectories()`, `get_dynamic_trajectories(odom)`, `get_markers()` |

Waypoint times are generated lazily from positions and speeds (`generate_waypoint_times()`): the time to each waypoint is segment distance divided by the average of the two endpoint speeds (floored at 0.01 m/s). When a `TrajectoryXYZVYaw` is ingested, each waypoint's scalar `velocity` is turned into a velocity *vector* along the local segment direction.

## Trajectory generator classes

The library distinguishes **static** generators (fixed shape, computed once, `get_trajectory()`) from **dynamic** generators (recomputed from the robot's current odometry, `get_trajectory(odom)`):

| Class | Kind | What it generates | Parameters (constructor) |
|---|---|---|---|
| `CurveTrajectory` | static | A constant-speed arc in the x-y plane of `frame`, integrating heading at a fixed turn rate for `time` seconds in `dt` steps; yaw either follows the heading or is fixed | `linear_velocity` (m/s), `angular_velocity` (rad/s), `frame`, `time` (s), `dt` (s), `use_heading`, `yaw` (rad) |
| `AccelerationTrajectory` | dynamic | Forward-integrates the robot's current position/velocity (transformed into `frame`) under a constant acceleration for horizon `ht` in `dt` steps; per-waypoint speed capped at `max_velocity` | `frame`, `ax, ay, az` (m/s²), `dt` (s), `ht` (s), `max_velocity` (m/s) |
| `TakeoffTrajectory` | dynamic | A 3-waypoint vertical (optionally tilted) climb of `height` meters from the current pose, ending with a near-zero-velocity waypoint | `height` (m), `velocity` (m/s), `path_roll`, `path_pitch` (rad), `relative_to_orientation` |

Only `curve` and `acceleration` can be created from the YAML config; `TakeoffTrajectory` is constructed programmatically (the `takeoff_landing_planner` builds its takeoff/landing trajectories with it). Entries with any other `type` are silently skipped by the parser (`src/trajectory_library.cpp:1284-1344`).

## Config file format (`trajectory_library_config`)

`TrajectoryLibrary(config_filename, node_ptr)` loads a YAML file with a single top-level key, `trajectories:`, a list of generator definitions. Each entry's keys depend on its `type`:

**`type: curve`** → one static `CurveTrajectory`:

| Key | Unit | Meaning |
|---|---|---|
| `linear_velocity` | m/s | Constant speed along the arc |
| `angular_velocity` | **deg/s** | Turn rate (converted to rad/s at load) |
| `frame` | TF frame | Frame the arc starts at the origin of (e.g. `tracking_point_stabilized`) |
| `time` | s | Duration of the arc |
| `dt` | s | Waypoint spacing in time |
| `yaw` | `heading` or **deg** | The literal string `heading` makes yaw follow the direction of travel; a number fixes yaw to that value |

**`type: acceleration`** → one dynamic `AccelerationTrajectory`. The acceleration vector is given either componentwise (`x`/`y`/`z`) **or** polar (`magnitude`/`magnitude_yaw`/`magnitude_pitch`); if neither complete set is present the entry is rejected with a console message:

| Key | Unit | Meaning |
|---|---|---|
| `frame` | TF frame | Frame the integration happens in (e.g. `look_ahead_point_stabilized`) |
| `x`, `y`, `z` | m/s² | Acceleration vector components |
| `magnitude` | m/s² | Alternative: acceleration magnitude… |
| `magnitude_yaw`, `magnitude_pitch` | **deg** | …rotated by this yaw/pitch from the +x axis |
| `dt` | s | Integration/waypoint time step |
| `ht` | s | Horizon time (how long to integrate) |
| `max_velocity` | m/s | Speed cap applied to each generated waypoint |

**`$(param <name>)` substitution:** any scalar value may be the string `$(param name)`, which is replaced at load time with the value of the ROS parameter `name` on the node that constructed the `TrajectoryLibrary` (parser: `include/trajectory_library/trajectory_library.hpp`, `parse<T>()`). This is how one config file serves different speed profiles: DROAN's `droan.yaml` sets `dt`, `ht`, `ht_long`, `max_velocity`, and `magnitude` as node parameters and the trajectory YAML references them. `TrajectoryLibrary`'s constructor declares these five parameter names (with placeholder defaults) so the substitution always resolves.

Real excerpt from [`config/long.yaml`](config/long.yaml) — the default library for `droan_local_planner` (an `acceleration` fan: one entry per `magnitude_yaw` heading, 22.5° apart, plus climbing/descending variants):

```yaml
---
trajectories:
  - dt: $(param dt)
    frame: look_ahead_point_stabilized
    ht: $(param ht_long)
    magnitude: $(param magnitude)
    magnitude_pitch: 0
    magnitude_yaw: 0
    max_velocity: $(param max_velocity)
    type: acceleration
  - dt: $(param dt)
    frame: look_ahead_point_stabilized
    ht: $(param ht_long)
    magnitude: $(param magnitude)
    magnitude_pitch: 0
    magnitude_yaw: 22.5
    max_velocity: $(param max_velocity)
    type: acceleration
  # ... more headings ...
```

And a `curve` example from [`config/backup.yaml`](config/backup.yaml):

```yaml
trajectories:
  - type: curve
    linear_velocity: 1
    angular_velocity: -45
    frame: tracking_point_stabilized
    dt: 0.2
    time: 3
    yaw: heading
```

### Shipped config files

Installed to `share/trajectory_library/config/`; reference them with `$(find-pkg-share trajectory_library)/config/<file>.yaml`:

| File | Contents |
|---|---|
| `long.yaml` | Acceleration fan over `ht_long` horizon with level/climb/descend pitches — **DROAN's default** |
| `flat.yaml` | Acceleration fan, level flight only (`magnitude_pitch: 0`) |
| `acceleration_magnitudes.yaml` | Large acceleration fan parameterized by `$(param magnitude)` |
| `acceleration_trajectories.yaml`, `acceleration_trajectories_fast.yaml` | Fixed-value (no `$(param)`) acceleration sets at low/high accelerations |
| `demo_trajectory_definitions.yaml` | Slow `curve` set (0.2 m/s) for demos |
| `backup.yaml` | `curve` set in `tracking_point_stabilized` plus a straight acceleration entry |
| `fixed_trajectories.yaml` | **Different schema** — a catalog listing which `attributes` each `airstack_msgs/FixedTrajectory` type (Figure8, Racetrack, Circle, Line, Point) takes; not loadable by `TrajectoryLibrary`, and no trunk code reads it |

[`src/trajectory_library_generator.py`](src/trajectory_library_generator.py) is a standalone developer script (not installed) that plots `curve`/`arc` config entries with matplotlib for eyeballing a library before flying it.

## Consumers

| Package | How it uses this library |
|---|---|
| [`droan_local_planner`](../droan_local_planner/README.md) | Constructs `TrajectoryLibrary` from its `trajectory_library_config` parameter (default `$(find-pkg-share trajectory_library)/config/long.yaml`, set in `config/droan.yaml`); each planning cycle calls `get_dynamic_trajectories(look_ahead_odom)` to get the candidate set it collision-checks and scores |
| [`droan_gl`](../droan_gl/README.md) | Links the library for the `Trajectory`/`Waypoint` utility classes (e.g. wrapping the incoming `nav_msgs/Path` global plan); it does **not** load a YAML library — its candidates come from its own graph expansion |
| [`takeoff_landing_planner`](../takeoff_landing_planner/README.md) | Constructs `TakeoffTrajectory` generators programmatically for takeoff and landing |

## See also

- [DROAN Local Planner README](../droan_local_planner/README.md) — the primary consumer of the YAML library
- [DROAN GL README](../droan_gl/README.md) — GPU DROAN variant
- [Trajectory Controller README](../../controls/trajectory_controller/README.md) — where the generated `TrajectoryXYZVYaw` trajectories are sent
- [Interface Conventions Specification](../../../../../../docs/robot/autonomy/interface_conventions.md) — canonical topics/types for the trajectory group
