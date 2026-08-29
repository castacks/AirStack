# Fly a Mission from the GCS

A ~20 minute operator lesson. By the end you will have flown a complete
mission from the Ground Control Station: taken off, placed waypoints, drawn a
geofence, navigated a route, landed, and saved the mission set to disk for
next time.

**Prerequisite:** you finished [Getting Started](index.md) and the stack is
running in Isaac Sim (`airstack up`, then `airstack ready` reports the drone
is ready to fly). Foxglove opened automatically with the AirStack layout.

!!! note "What's in the scene"

    The default launch spawns a single drone in the Pegasus **Default
    Environment** — a flat, open ground plane with no obstacles. Perfect for
    a first mission.

## 1. Orient yourself in the layout

Look at the auto-loaded Foxglove layout ([tour](../gcs/usage/user_interface.md)):

- **3D panel** — the fleet view: the drone's mesh, its live trajectory, and a
  textured ground plane.
- **Robot Tasks panel** — the command panel, with one tab per task:
  **Takeoff**, **Land**, **Navigate**, **Exploration**, **Coverage**,
  **Semantic Search**, and **Fixed Trajectory**.
- **Per-robot tabs** — Image panels showing the robot's camera and depth feeds.

**Check:** you can see the drone sitting on the ground plane in the 3D panel,
and a live camera feed in the robot's tab.

## 2. Take off

In the Robot Tasks panel, open the **Takeoff** tab. Keep the defaults
(`target_altitude_m` 10.0, `velocity_m_s` 1.0), pick your robot, and click
**Send**.

**Check:** the panel streams feedback like `3.2 / 10.0 m` while the drone
climbs in the 3D panel, finishing in a hover at altitude.

## 3. Place waypoints

The **Waypoint Editor** is embedded in the **Navigate** tab:

1. Open the **Navigate** tab and toggle **Enable click capture** on.
2. In the 3D panel toolbar (top right), click the **Publish** tool (▷ icon)
   and switch its mode to **Publish 2D point (/clicked_point)**.
3. Set the **Default altitude** field to `10` so waypoints match your hover
   altitude.
4. Click 2–3 spots in the 3D panel, a few tens of meters apart.

**Check:** a numbered sphere appears at each click, in click order. (No
markers? In the 3D panel's Topics list, toggle on `/gcs/waypoints/markers`.)

## 4. Draw a geofence

The **Polygon Editor** works the same way, from the **Exploration** tab:

1. Open the **Exploration** tab and toggle its **Enable click capture** on.
2. With the Publish tool still in **Publish 2D point** mode, click 3–4
   vertices surrounding your waypoints.

**Check:** the vertices render as a closed loop around your route in the 3D
panel. This polygon is the geofence / `search_bounds` that area tasks like
Exploration and Coverage stay inside — details in
[Adding Waypoints and Geofences](../gcs/waypoints_and_geofences.md).

## 5. Fly the route

Back in the **Navigate** tab: in the **from:** dropdown pick **active** (the
waypoints currently in the editor), set `goal_tolerance_m` to `1.0`, pick
your robot, and click **Send**.

**Check:** in the 3D panel the drone tracks through your waypoints in order
while the feedback's `distance_to_goal` shrinks; the task ends with
`Goal reached`. (Under the hood this is a
[NavigateTask action](../robot/autonomy/tasks.md) sent to the robot.)

## 6. Land

Open the **Land** tab, keep the default `velocity_m_s` (0.3), and click
**Send**.

**Check:** the altitude readout falls and the drone settles onto the ground
plane in the 3D panel.

## 7. Save your mission set

Saves survive container restarts, so tomorrow's flight starts from here:

1. In the Navigate tab's Waypoint Editor, type a name (e.g. `first_mission`)
   into the **save name…** field and click **+ Add**.
2. Click **Save** on the new row — it changes to **✓ Saved**.
3. Repeat in the Exploration tab's Polygon Editor for your geofence.

**Check:** on the host, the save files exist:

```bash
ls ~/.airstack/gcs_waypoint_saves.json ~/.airstack/gcs_polygon_saves.json
```

Your saved set now also appears in the Navigate tab's **from:** dropdown —
next flight, pick it instead of **active** and hit **Send**.

Congratulations! You flew a full mission — takeoff, waypoints, geofence,
navigation, landing — and saved it to disk.

**Next:** [Change a Parameter and See the Effect](change_a_parameter.md) — your
first taste of the developer loop. For everything the operator interface can
do, read [Operating the GCS](../gcs/usage/user_interface.md).
