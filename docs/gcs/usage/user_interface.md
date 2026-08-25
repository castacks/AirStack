# Operating the GCS

The operator interface is **Foxglove Studio**, extended with AirStack's custom panels.
When the GCS container starts, Foxglove opens automatically, already connected
(`ws://localhost:8765` in-container) and already showing the **AirStack default
(`<N>` robots)** layout — rendered to match `NUM_ROBOTS`, no manual import needed
(see [GCS Foxglove Visualization](../foxglove.md)).

## The layout at a glance

The auto-seeded layout is built from these panels:

- **3D panel** — the fleet view in a shared global frame: per-robot meshes,
  live trajectories, global plan polylines, and VDB occupancy maps, all merged
  onto `/gcs/robot_markers` by the GCS visualizer (plus the sim-only textured
  ground plane).
- **Robot Tasks panel** — the custom command panel (from the `robot-commands`
  extension). This is where you fly the fleet.
- **Per-robot tabs** — one tab per robot with **Image** panels showing its camera
  and depth feeds.

The **Waypoint Editor** and **Polygon Editor** are not separate panels — they are
embedded inside the Robot Tasks panel's Navigate, Exploration, and Coverage tabs.

## Commanding a robot

Each task type gets a tab in the Robot Tasks panel: **Takeoff**, **Land**,
**Navigate**, **Exploration**, **Coverage**, **Semantic Search**, and
**Fixed Trajectory**. The flow is always the same: pick the tab, fill in the goal
fields, pick the robot, click **Send**. Goals travel as ROS 2 actions, relayed by
`action_relay` from the GCS domain into each robot's own domain.

A first flight looks like:

1. **Takeoff** — set `target_altitude_m` and `velocity_m_s`, pick the robot, **Send**.
   Watch the drone climb in the 3D panel.
2. **Navigate** — pick a waypoint set (a saved one from the **from:** dropdown, or
   **active** for whatever is in the embedded Waypoint Editor), set
   `goal_tolerance_m`, **Send**.
3. **Land** — set `velocity_m_s`, **Send**.

**Take off before anything else.** Land, Navigate, and the mission tasks all assume
the drone is airborne — `takeoff_landing_planner` publishes `is_airborne`, and the
RViz Tasks Panel greys those tasks out until it's true. The Foxglove Robot Tasks
panel offers the same task set but does not currently gate on `is_airborne`, so keep
the Takeoff-first order yourself.

## Waypoints and geofences

- **Waypoint Editor** (Navigate tab) — ordered 3D waypoints for a Navigate route.
- **Polygon Editor** (Exploration / Coverage tabs) — a closed 2D area used as the
  geofence / `search_bounds` for area tasks.

Both work by enabling click capture and clicking in the 3D panel; point sets can be
named, saved to disk (they survive container restarts), and picked from the task
dropdowns later. Full walkthrough: [Adding Waypoints and Geofences](../waypoints_and_geofences.md).

## Monitoring

- **3D panel** — each robot's executing trajectory, global plan, and occupancy map
  update live; the pose arrow tracks odometry.
- **Per-robot tabs** — camera and depth Image feeds confirm sensors are streaming.
- **Map panel (optional)** — the visualizer publishes each robot's GPS fix on
  `/gcs/<robot_name>/location` (plus a fixed reference on `/gcs/map_origin/location`);
  add a Foxglove Map panel subscribed to those to see the fleet on a world map.
- **Any topic** — this is stock Foxglove: add Raw Messages or Plot panels for
  whatever the bridge exposes (battery, MAVROS state, etc.).

You can edit the layout freely and **Save** it — your edits persist and the seeder
won't overwrite them (delete the layout in **Layouts** to reset to the generated
default).

## Troubleshooting

Robot missing from the 3D view, wrong global position, markers not appearing —
see the [troubleshooting table in GCS Foxglove Visualization](../foxglove.md#troubleshooting).
For container-level debugging:

```bash
airstack logs gcs          # follow the bringup session output
airstack connect gcs       # attach to the container's tmux session
```

## See also

- [GCS Foxglove Visualization](../foxglove.md) — layout seeding, visualizer topics, troubleshooting
- [Adding Waypoints and Geofences](../waypoints_and_geofences.md) — the click-to-place editors in detail
- [GCS Overview](../index.md) — architecture, launch structure, `gcs` vs `gcs-real`
- [Extending the Foxglove Visualizer](../extending_foxglove.md) — adding marker types (maintainers)
