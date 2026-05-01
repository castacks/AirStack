# Payloads & Foxglove Visualization

Payloads let you attach any ROS message to the `PeerProfile` so it gets broadcast to all peers and the GCS alongside GPS/heading. Common uses: sharing maps, frontier viewpoints, semantic rays, or any per-robot data you want visible fleet-wide.

Payloads are **config-driven** — no changes to `gossip_node.py` are needed.

## How payloads work

1. `gossip_node` subscribes to each topic listed in `gossip_payloads.yaml`
2. On each 1 Hz publish tick, the latest message from each topic is serialized and attached to the `PeerProfile`
3. Before attaching, the payload is transformed from the robot's local odom frame → global ENU using the robot's boot GPS position
4. Peers and GCS receive the payload already in world frame — no transform needed on the receiving side

## Step 1 — Add to gossip_payloads.yaml

**File:** `common/ros_packages/coordination/coordination_bringup/config/gossip_payloads.yaml`

```yaml
payload_topics:
  - topic: "/{robot_name}/your/topic"
    type: "your_msgs/msg/YourType"
```

- `{robot_name}` is substituted at runtime (e.g. → `/robot_1/your/topic`)
- Topics that haven't published yet are silently skipped
- Only `MarkerArray` and `PointCloud2` are automatically transformed to world frame; other types pass through as-is

Rebuild after editing:

```bash
bws --packages-select coordination_bringup
```

Verify the payload is being attached:

```bash
ros2 topic echo /gossip/peers --field payloads
# or
ros2 run coordination_bringup peer_registry_monitor
```

## Step 2 — Visualize in Foxglove

Payloads don't appear in Foxglove automatically — you need a handler in `payload_visualizer_node.py` that republishes the payload to its own topic. The manual steps are below; an [AI-native skill](#ai-native-skill) is also available to do both this and Step 1 in one go.

**File:** `gcs/ros_ws/src/gcs_visualizer/gcs_visualizer/payload_visualizer_node.py`

**1. Add a handler method:**

```python
def _handle_your_payload(self, robot_name, msg, i, now):
    # msg is already in global ENU / 'map' frame
    # Apply display z-offset to align with the GCS datum
    out = transform_point_cloud2(msg, 0.0, 0.0, self._display_z_offset())
    out.header.stamp = now
    self._pub_for(f'/gcs/payload/{robot_name}/your_name', PointCloud2).publish(out)
```

**2. Register it in `PAYLOAD_HANDLERS`:**

```python
PAYLOAD_HANDLERS = {
    'your_name': ('your_msgs/msg/YourType', _handle_your_payload),
}
```

**3. Rebuild GCS:**

```bash
docker exec airstack-gcs-1 bash -c "bws --packages-select gcs_visualizer && sws"
```

or restart AirStack.

Foxglove will now show `/gcs/payload/{robot_name}/your_name` as a subscribable topic with full visualization controls.

### AI-native skill

The `attach-gossip-payload` skill automates Step 1 and Step 2 — yaml edit and the GCS handler in a single pass. In Claude Code:

```
Follow the attach-gossip-payload skill to add /{robot_name}/your/topic
of type your_msgs/msg/YourType and visualize it in Foxglove
```

See the full skill at `.agents/skills/attach-gossip-payload`.

## Visualization options

For `PointCloud2` payloads, you have two options:

**Default — Foxglove GUI:** Publish as raw. Foxglove's panel settings control point size, shape, and color per-user. No code changes needed.

**Preconfigured — fixed shape/size/color in code:** Convert to a `MarkerArray` in the handler. An example that renders a point cloud as 0.5 m cubes with per-point RGB colors:

```python
def _handle_my_payload(self, robot_name, msg, i, now):
    marker = point_cloud2_to_cube_marker(
        msg, 0.0, 0.0, self._display_z_offset(),
        ns=f'{robot_name}_my_payload',
        marker_id=i * 100000,
        stamp=now,
        lifetime=Duration(sec=2, nanosec=0),
        fallback_color=None,  # uses per-point rgb field; set to (r, g, b, a) for a solid color
        scale=0.5,   # cube size in metres
    )
    if marker is not None:
        out = MarkerArray()
        out.markers.append(marker)
        self._pub_for(f'/gcs/payload/{robot_name}/my_payload', MarkerArray).publish(out)
```
