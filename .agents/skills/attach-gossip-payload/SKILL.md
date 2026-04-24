# Skill: Attach Custom Payload to PeerProfile (Gossip Protocol)

## When to use
When you want to broadcast any ROS message to all peer robots via the gossip
protocol — for example, a frontier map, sensor summary, or task status.

## Background

Each robot runs a `gossip_node` that periodically broadcasts a `PeerProfile`
to all other robots. The profile carries structured fields (GPS, heading, waypoint)
plus an open-ended `payloads` array of serialized ROS messages.

Since the Zenoh migration, all containers share one ROS graph (`rmw_zenoh_cpp`,
`ROS_DOMAIN_ID=0`, Zenoh router in the GCS container). The gossip topic
`/gossip/peers` is just a regular topic on this shared graph — no per-robot
domain, no dedicated bridge. The `gossip_domain` launch arg is retained but is
a no-op under Zenoh.

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
### 4b — Add handler and register in `PAYLOAD_HANDLERS`

`PAYLOAD_HANDLERS` is keyed by **payload name** (the last segment of the topic path
in `gossip_payloads.yaml`). This means multiple payloads of the same ROS type work
without any special casing.

Add a handler and register it:

```python
PAYLOAD_HANDLERS = {
    'filtered_rays':       ('visualization_msgs/msg/MarkerArray', _handle_filtered_rays),
    'frontier_viewpoints': ('sensor_msgs/msg/PointCloud2',        _handle_frontier_viewpoints),
    'rgb_voxels':          ('sensor_msgs/msg/PointCloud2',        _handle_rgb_voxels),
    'your_name':           ('your_msgs/msg/YourType',             _handle_your_payload),  # ← add
}
```

The key `'your_name'` must match the last path segment of the topic in `gossip_payloads.yaml`.
For example, `/{robot_name}/rayfronts/voxel_rgb` → key is `voxel_rgb`.

Handler signature — all handlers must match exactly:
```python
def _handle_your_payload(self, robot_name, msg, i, now):
    # msg  — deserialized ROS message (already in global ENU / 'map' frame)
    # i    — stable robot index (use for marker IDs: i * 100000 + unique_offset)
    # now  — current ROS timestamp (builtin_interfaces/Time)
    # transform and publish to the payload's dedicated topic:
    self._pub_for(f'/gcs/payload/{robot_name}/your_name', YourMsgType).publish(out)
```

### 4c — Visualization options

For `PointCloud2` payloads, choose one approach:

**Default:** Publish as raw `PointCloud2` — Foxglove GUI controls point size, shape, and color. No extra code needed.

**Preconfigured shape/size:** Convert to a `CUBE_LIST` `MarkerArray` in the handler (see `voxel_rgb` for a real example). Use this when you want a fixed visual style regardless of user layout settings:

```python
from gcs_visualizer.gcs_utils import point_cloud2_to_cube_marker

def _handle_your_payload(self, robot_name, msg, i, now):
    marker = point_cloud2_to_cube_marker(
        msg, 0.0, 0.0, self._display_z_offset(),
        ns=f'{robot_name}_your_name',
        marker_id=i * 100000,
        stamp=now,
        lifetime=Duration(sec=2, nanosec=0),
        fallback_color=None,  # uses per-point rgb field; set to (r, g, b, a) for a solid color
        scale=0.5,   # cube size in metres
    )
    if marker is not None:
        out = MarkerArray()
        out.markers.append(marker)
        self._pub_for(f'/gcs/payload/{robot_name}/your_name', MarkerArray).publish(out)
```

### 4d — Available transform helpers (`gcs_utils.py`)

Check `gcs/ros_ws/src/gcs_visualizer/gcs_visualizer/gcs_utils.py` before writing
transform logic. Add a new helper there if none fits.

| Helper | Use for |
|--------|---------|
| `transform_marker_array(ma, bx, by, bz)` | `MarkerArray` → translated `MarkerArray` |
| `transform_point_cloud2(cloud, bx, by, bz)` | `PointCloud2` → translated `PointCloud2` (preserves all fields including `rgb`) |
| `point_cloud2_to_cube_marker(cloud, bx, by, bz, ns, marker_id, stamp, lifetime, scale)` | `PointCloud2` → `CUBE_LIST` Marker with fixed voxel size and per-point RGB |

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
| `/{robot_name}/rayfronts/voxel_rgb` | `sensor_msgs/msg/PointCloud2` | `/gcs/payload/{robot}/voxel_rgb` | Fixed (CUBE_LIST MarkerArray, 0.5 m) |

## Architecture notes

- `gossip_node` runs in each robot container and subscribes directly to any topic
  on the shared Zenoh graph, including Rayfronts (now a first-class ROS package
  at `common/rayfronts`)
- Under Zenoh all containers share one graph, so `/gossip/peers` reaches every
  peer and the GCS without a dedicated cross-domain bridge. The payload bytes
  travel inside the PeerProfile message — attached payload topics do **not**
  need any separate bridging config
- Payloads are re-serialized every publish tick from the latest cached message;
  stale data is never cleared between ticks (latest-wins per topic)
- Payload size matters for gossip bandwidth — avoid attaching large point clouds
  at high rates; 1 Hz (the default gossip rate) is usually fine

## Message deduplication

Every `PeerProfile` message is identified by the triple:

```
(robot_name, gps_fix.header.stamp.sec, gps_fix.header.stamp.nanosec)
```

The stamp is set **at publish time** by the originating robot. Each receiver
maintains a seen-set (size 50, FIFO eviction) and drops any message whose ID
has already been processed.

**Expected behaviour:** every drone will forward/receive a message at least
once — this is intentional. The seen-set prevents infinite re-processing but
does not prevent the initial fan-out that comes from all robots being on the
same shared DDS domain.

**Relay fields (reserved for future use):**
- `uint8 source` — `SOURCE_DIRECT (0)` or `SOURCE_RELAYED (1)`
- `uint8 relay_hops` — number of hops the message has traversed

These fields exist in the wire format and Python API but relay logic is not
yet implemented. The seen-set deduplication is already wired to handle it
correctly when relay is activated.

## Registry behaviour

- Each robot keeps a **per-robot inbox** (latest message per peer, drained at
  5 Hz) and a **global registry** (latest-wins, monotonic per robot timestamp)
- Registry entries are **never evicted** — a crashed robot stays visible
  indefinitely until the node is restarted
- The registry is published to `/{robot_name}/coordination/peer_registry` with
  RELIABLE + TRANSIENT_LOCAL QoS so late-joining nodes get the full snapshot

## QoS note

Payload subscriptions use `GOSSIP_QOS` (BEST_EFFORT, KEEP_LAST 1). If your
source topic uses RELIABLE QoS you may need to adjust — see `gossip_node.py`.
