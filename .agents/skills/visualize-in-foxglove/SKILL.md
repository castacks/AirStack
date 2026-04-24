---
name: visualize-in-foxglove
description: Add visualization of a ROS 2 topic to Foxglove/GCS. Use when you want a new topic (path, markers, odometry, etc.) to appear in the Foxglove dashboard on the GCS. Covers robot_marker_node integration and coordinate frame translation.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Visualize a Topic in Foxglove / GCS

## When to Use

You want a topic published by a robot container to be visible in the Foxglove dashboard
running in the GCS container.

## Architecture Overview

```
Robot container (rmw_zenoh_cpp, ROS_DOMAIN_ID=0, namespace /robot_N)
  └─ publishes topics

Zenoh router (rmw_zenohd, runs in GCS container, listens tcp/0.0.0.0:7447)
  └─ provides discovery; all containers connect as peers

GCS container (rmw_zenoh_cpp, ROS_DOMAIN_ID=0)
  ├─ Foxglove bridge → streams to browser
  └─ robot_marker_node → transforms & republishes as /gcs/robot_markers MarkerArray
```

**Key insight:** Since the Zenoh migration, every container shares one Zenoh graph — any
topic a robot publishes is already reachable from the GCS. There is no `dds_router.yaml`
allowlist to maintain; the only thing to wire up is the GCS-side subscription.

---

## Step 1 — (No bridge config needed)

Historical note: this step used to be "add the topic to `dds_router.yaml`'s allowlist".
After the Zenoh migration, that allowlist no longer exists. All topics cross the
rmw_zenoh_cpp graph automatically. If you are reading an old doc or commit that edits
`dds_router.yaml`, it's obsolete.

---

## Step 2 — Subscribe and Visualize on the GCS

There are two paths depending on what you want to display:

### Path A — Display the raw topic directly in Foxglove

If the topic message type is natively supported by Foxglove (e.g. `nav_msgs/Path`,
`sensor_msgs/PointCloud2`, `visualization_msgs/MarkerArray`), just bridge it and add
a panel in Foxglove pointing at the topic. No extra GCS code needed.

**Limitation:** The topic arrives in the robot's local odom frame (`map` frame origin =
drone boot position). If you need it georeferenced (aligned with GPS/ENU), you must
translate it — see Path B.

### Path B — Translate and republish via robot_marker_node

**File:** `gcs/ros_ws/src/gcs_visualizer/gcs_visualizer/robot_marker_node.py`

This node auto-discovers robot topics, applies a GPS boot offset to convert from the
robot's local odom frame to ENU (map frame), and republishes everything as a single
`/gcs/robot_markers` MarkerArray.

**Coordinate frame context:**
- Robot odometry uses a local `map` frame whose origin is the drone's position at boot.
- GPS coordinates are converted to ENU relative to `ORIGIN_LAT/LON/ALT` (Lisbon by default).
- `_gps_boot[robot_name]` = ENU position of the odom origin = offset to add to all
  odom-frame coordinates.
- Trajectory and global plan markers are in odom frame → add boot offset to `points`.
- Do NOT also offset `pose.position` for LINE_STRIP/ARROW markers — their points are
  already in the header frame; double-offsetting the pose causes wrong positions.

**To add a new topic type, follow this pattern (shown for `nav_msgs/Path`):**

1. **Add suffix constant and regex pattern:**
```python
PLAN_SUFFIX = '/global_plan'
self._plan_pattern = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(PLAN_SUFFIX)}$')
```

2. **Add state dicts and subscribed set:**
```python
self._global_plans    = {}   # robot_name -> latest nav_msgs/Path
self._subscribed_plan = set()
```

3. **Discover and subscribe in `_discover_robots`:**
```python
if topic not in self._subscribed_plan:
    m = self._plan_pattern.match(topic)
    if m and 'nav_msgs/msg/Path' in type_list:
        name = m.group(1)
        self.create_subscription(
            Path, topic,
            lambda msg, n=name: self._plan_callback(msg, n),
            10,   # use default RELIABLE QoS for planning topics
                  # use SENSOR_QOS for high-rate sensor streams
        )
        self._subscribed_plan.add(topic)
```

4. **Add callback:**
```python
def _plan_callback(self, msg: Path, robot_name: str):
    self._global_plans[robot_name] = msg
```

5. **Render in `_publish_markers` (skip silently if not yet received):**
```python
plan = self._global_plans.get(robot_name)
if plan is not None and boot is not None:
    line = Marker()
    line.header.frame_id = 'map'
    line.ns = f'{robot_name}_global_plan'
    line.type = Marker.LINE_STRIP
    ...
    for pose_stamped in plan.poses:
        p = pose_stamped.pose.position
        line.points.append(Point(x=p.x + bx, y=p.y + by, z=p.z + bz))
    array.markers.append(line)
```

**QoS guidance:**
- High-rate sensor/visualization streams (odom, trajectory_vis): use `SENSOR_QOS` (BEST_EFFORT)
- Infrequently-published planning topics (global_plan): use `10` (default RELIABLE)

---

## Step 3 — Verify

```bash
# Confirm the robot is publishing
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /robot_1/your_topic --once"

# Confirm the GCS sees it (Zenoh graph is shared — should work without extra config)
docker exec airstack-gcs-1 bash -c "ros2 topic echo /robot_1/your_topic --once"

# Check GCS node subscribed (look for log line)
docker logs airstack-gcs-1 2>&1 | grep "Subscribed to"

# Check the combined marker output
docker exec airstack-gcs-1 bash -c "ros2 topic echo /gcs/robot_markers --once"

# If topics are missing cross-container, sanity-check the Zenoh router is up:
docker exec airstack-gcs-1 bash -c "pgrep -fa rmw_zenohd"
```

---

## Common Pitfalls

| Symptom | Cause | Fix |
|---------|-------|-----|
| Topic visible on robot, not on GCS | Zenoh router not running, or client can't reach it | `pgrep -fa rmw_zenohd` in GCS; verify `ZENOH_ROUTER_IP` in robot env is reachable |
| Topic on GCS but not in Foxglove | Not subscribed in robot_marker_node or Foxglove panel missing | Add subscription or add panel |
| Marker appears at wrong position | Missing boot GPS offset | Apply `bx, by, bz` from `_gps_boot` to all points |
| Marker double-offset | Added boot to both `pose.position` AND `points` | Only offset `points` for LINE_STRIP/ARROW markers |
| Planning topic missed after late publish | Using BEST_EFFORT QoS | Use `10` (RELIABLE) for planning topics |
| New robot not discovered | Topic appeared before discovery timer fired | Discovery runs every 5s; wait or trigger manually |
| Cross-robot visibility (want robot_2 hidden from robot_1) | Zenoh's single graph exposes topic ads across robots | Optional: add keyexpr `allowed_origin` filter in per-robot `/tmp/zenoh_session.json5` |
