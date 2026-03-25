# Skill: Attach Custom Payload to PeerProfile (Gossip Protocol)

## When to use
When you want to broadcast any ROS message to all peer robots via the gossip
protocol — for example, a frontier map, sensor summary, or task status.

## Background

Each robot runs a `gossip_node` that periodically broadcasts a `PeerProfile`
to all other robots on the gossip domain (default domain 99). The profile
carries structured fields (GPS, heading, waypoint) plus an open-ended
`payloads` array of serialized ROS messages.

**Key files:**
| File | Purpose |
|------|---------|
| `robot/ros_ws/src/coordination/coordination_bringup/config/gossip_payloads.yaml` | Lists topics to attach as payloads — **edit this to add payloads** |
| `coordination_bringup/coordination_bringup/gossip_node.py` | Reads config, subscribes, attaches payloads on each publish tick |
| `coordination_bringup/coordination_bringup/peer_profile.py` | `PeerProfile` helper class with `add_payload` / `get_payload` API |
| `coordination_msgs/msg/PeerProfile.msg` | Wire format — `payloads` is `PeerProfilePayload[]` |
| `coordination_msgs/msg/PeerProfilePayload.msg` | `string payload_type` + `uint8[] payload_data` |

## How to add a payload (config-driven — no code changes)

### Step 1 — Edit `gossip_payloads.yaml`

```yaml
payload_topics:
  # existing entries ...

  # Your new payload:
  - topic: "/{robot_name}/your/topic"
    type: "your_msgs/msg/YourType"
```

- `{robot_name}` is automatically substituted at runtime (e.g. → `/robot_1/your/topic`)
- If the topic hasn't published yet, the payload is silently skipped — no crash
- `type` must be the fully-qualified ROS 2 type string

### Step 2 — Rebuild and restart gossip_node

```bash
bws --packages-select coordination_bringup
ros2 launch coordination_bringup gossip.launch.xml
```

### Step 3 — Verify

Check that the payload is being attached:
```bash
ros2 topic echo /gossip/peers --field payloads
# should show entries with your payload_type string
```

Or use the registry monitor:
```bash
ros2 run coordination_bringup peer_registry_monitor
# shows payload_types per peer
```

## How to read a payload on the receiving side

```python
from coordination_bringup.peer_profile import PeerProfile

def on_peer_msg(self, msg):
    profile = PeerProfile.from_ros_msg(msg)

    # Get a specific payload by type string
    rays = profile.get_payload("visualization_msgs/msg/MarkerArray")
    if rays is not None:
        # use rays as visualization_msgs/msg/MarkerArray
        pass

    # List all payload types present
    print(profile.payload_types())

    # Get all payloads deserialized
    for payload in profile.get_all_payloads():
        print(type(payload))
```

## Step 4 — Add GCS visualization

After adding a payload to `gossip_payloads.yaml`, add a handler so it appears in
Foxglove. Each payload is published to its own topic:
`/gcs/payload/{robot_name}/{payload_name}`

This means Foxglove exposes full visualization controls (point size, color mapping,
marker type, etc.) for each payload independently.

**File:** `gcs/ros_ws/src/gcs_visualizer/gcs_visualizer/payload_visualizer_node.py`

### 4a — Read the payload type from `gossip_payloads.yaml`

Open `robot/ros_ws/src/coordination/coordination_bringup/config/gossip_payloads.yaml`
and note the `type:` field for your new entry. This determines how to deserialize it.

If your type is **unique** (not already in `PAYLOAD_HANDLERS`), go to step 4b.
If your type **already exists** (e.g. a second `sensor_msgs/msg/PointCloud2`), go to step 4c.

### 4b — Unique type: add to `PAYLOAD_HANDLERS`

Add a handler and register it:

```python
PAYLOAD_HANDLERS = {
    'visualization_msgs/msg/MarkerArray': _handle_filtered_rays,
    'sensor_msgs/msg/PointCloud2':        _handle_frontier_viewpoints,
    'your_msgs/msg/YourType':             _handle_your_payload,   # ← add
}
```

Handler signature — all handlers must match exactly:
```python
def _handle_your_payload(self, robot_name, msg, boot, i, now):
    # msg  — deserialized ROS message
    # boot — (bx, by, bz) ENU offset; add to all positions to go odom→map frame
    # i    — stable robot index (use for marker IDs: i * 100000 + unique_offset)
    # now  — current ROS timestamp (builtin_interfaces/Time)
    bx, by, bz = boot
    # transform and publish to the payload's dedicated topic:
    self._pub_for(f'/gcs/payload/{robot_name}/your_name', YourMsgType).publish(out)
```

### 4c — Duplicate type: dispatch by index in `_on_peer_profile`

`PAYLOAD_HANDLERS` is a dict and cannot hold duplicate keys. After the
`PAYLOAD_HANDLERS` loop in `_on_peer_profile`, access payloads by their position
among same-type entries — order matches `gossip_payloads.yaml`:

```python
# Example: 2nd sensor_msgs/msg/PointCloud2 (index 1)
pc2_list = [p for p in profile._payloads if p["type"] == "sensor_msgs/msg/PointCloud2"]
if len(pc2_list) >= 2:
    msg = deserialize_message(bytes(pc2_list[1]["data"]), PointCloud2)
    self._payload_cache[(robot_name, 'my_cache_key')] = msg
cached = self._payload_cache.get((robot_name, 'my_cache_key'))
if cached is not None:
    self._handle_my_payload(robot_name, cached, boot, robot_index, now)
```

### 4d — Available transform helpers (`gcs_utils.py`)

Check `gcs/ros_ws/src/gcs_visualizer/gcs_visualizer/gcs_utils.py` before writing
transform logic. Add a new helper there if none fits.

| Helper | Use for |
|--------|---------|
| `transform_marker_array(ma, bx, by, bz, ns, id_base, stamp, lifetime)` | `MarkerArray` → translated `MarkerArray` |
| `transform_point_cloud2(cloud, bx, by, bz)` | `PointCloud2` → translated `PointCloud2` (preserves all fields including `rgb`) |

Publish raw `PointCloud2` when you want Foxglove's viz controls (point size, color
mapping, etc.). Publish `MarkerArray` when the shape/color is fixed by the publisher.

### 4e — Rebuild GCS

```bash
docker exec airstack-gcs-1 bash -c "bws --packages-select gcs_visualizer && sws"
```

Verify topics exist:
```bash
docker exec airstack-gcs-1 bash -c "ros2 topic list | grep /gcs/payload"
```

---

## Current payloads

| Topic in `gossip_payloads.yaml` | Type | GCS topic | Foxglove controls |
|--------------------------------|------|-----------|-------------------|
| `/{robot_name}/filtered_rays` | `visualization_msgs/msg/MarkerArray` | `/gcs/payload/{robot}/filtered_rays` | Fixed (MarkerArray) |
| `/{robot_name}/frontier_viewpoints` | `sensor_msgs/msg/PointCloud2` | `/gcs/payload/{robot}/frontier_viewpoints` | Full (raw PointCloud2) |
| `/{robot_name}/rayfronts/voxel_rgb` | `sensor_msgs/msg/PointCloud2` | `/gcs/payload/{robot}/rgb_voxels` | Full (raw PointCloud2 with RGB) |

## Architecture notes

- `gossip_node` runs on the **robot's domain** (e.g. domain 1 for `robot_1`)
  and can subscribe directly to any topic on that domain, including Rayfronts
- The gossip DDS Router bridges `/gossip/peers` to the shared gossip domain
  (default 99) — the payload bytes travel inside the PeerProfile message,
  so payload topics do **not** need their own DDS router entries
- Payloads are re-serialized every publish tick from the latest cached message;
  stale data is never cleared between ticks (latest-wins per topic)
- Payload size matters for gossip bandwidth — avoid attaching large point clouds
  at high rates; 1 Hz (the default gossip rate) is usually fine

## QoS note

Payload subscriptions use `GOSSIP_QOS` (BEST_EFFORT, KEEP_LAST 1). If your
source topic uses RELIABLE QoS you may need to adjust — see `gossip_node.py`.
