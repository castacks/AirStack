# Extending the Foxglove Visualizer

This page is the maintainer guide to extending the GCS visualizer node — `foxglove_visualizer_node` — with new or modified marker types, and to bridging new topics to the GCS. For what the node visualizes today, the topic tables, layouts, and troubleshooting, see [GCS Foxglove Visualization](foxglove.md). The agent workflow for adding a topic is [`.agents/skills/visualize-in-foxglove`](../../.agents/skills/visualize-in-foxglove/SKILL.md).

## How to modify or add a marker type

The visualizer is designed to be extended in-place. The pattern, taken from `gcs/ros_ws/src/gcs_visualizer/gcs_visualizer/foxglove_visualizer_node.py`:

### 1. Add a suffix and regex

```python
PLAN_SUFFIX = '/global_plan'
self._plan_pattern = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(PLAN_SUFFIX)}$')
```

### 2. Add state

```python
self._global_plans   = {}   # robot_name -> latest msg
self._subscribed_plan = set()
```

### 3. Subscribe in `_discover_robots`

```python
if topic not in self._subscribed_plan:
    m = self._plan_pattern.match(topic)
    if m and 'nav_msgs/msg/Path' in type_list:
        name = m.group(1)
        self.create_subscription(
            Path, topic,
            lambda msg, n=name: self._plan_callback(msg, n),
            10,   # 10 = default RELIABLE for planning topics;
                  # SENSOR_QOS for high-rate sensor streams
        )
        self._subscribed_plan.add(topic)
```


### 4. Add a callback

```python
def _plan_callback(self, msg: Path, robot_name: str):
    self._global_plans[robot_name] = msg
```

### 5. Render in `_publish_markers`

```python
plan = self._global_plans.get(robot_name)
boot = self._gps_boot.get(robot_name)
if plan is not None and boot is not None:
    bx, by, bz = boot
    line = Marker()
    line.header.frame_id = 'map'
    line.ns = f'{robot_name}_global_plan'
    line.type = Marker.LINE_STRIP
    for ps in plan.poses:
        p = ps.pose.position
        line.points.append(Point(x=p.x + bx, y=p.y + by, z=p.z + bz))
    array.markers.append(line)
```


### 6. Bridge the source topic across DDS domains

The visualizer can only subscribe to topics that crossed the DDS bridge. Add the source topic to `robot/ros_ws/src/autonomy_bringup/config/dds_router.yaml` under `allowlist` (or, for the split stack, to `stacks/lite_offload_global/bridge.yaml` and regenerate):

```yaml
allowlist:
  - name: "rt/$(env ROBOT_NAME)/your/new_topic"
```

Then restart the robot containers — the router only re-reads its allowlist on startup.

## Bridging a topic without writing a callback

If your topic is already in a Foxglove-native type (`nav_msgs/Path`, `sensor_msgs/PointCloud2`, `visualization_msgs/MarkerArray`) and doesn't need the GPS offset, you can skip the visualizer entirely — just bridge it through the DDS router and add a panel in Foxglove pointing at the topic. The visualizer is only required when you need georeferencing or want everything to flow through the combined `/gcs/robot_markers` namespace.

## See also

- [GCS Foxglove Visualization](foxglove.md) — operator guide: layouts, input/output topics, troubleshooting
- [Coordination Payloads](../robot/autonomy/coordination/payloads.md) — extending visualization with gossip-broadcast payloads
- [`.agents/skills/visualize-in-foxglove`](../../.agents/skills/visualize-in-foxglove/SKILL.md) — agent workflow for adding a topic
