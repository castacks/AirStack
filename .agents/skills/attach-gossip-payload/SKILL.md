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

## Current payloads (Rayfronts example)

| Topic | Type | Purpose |
|-------|------|---------|
| `/{robot_name}/filtered_rays` | `visualization_msgs/msg/MarkerArray` | Semantic filtered rays — peers avoid re-exploring these areas |
| `/{robot_name}/frontier_viewpoints` | `sensor_msgs/msg/PointCloud2` | Candidate exploration targets |

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
