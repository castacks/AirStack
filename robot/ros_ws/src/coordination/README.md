# Coordination

Multi-robot coordination layer for AirStack. Implements a gossip protocol over a shared DDS domain so drones can share state and payloads without a central broker.

## Architecture

```
Robot (domain N)                      Shared gossip domain (99)
┌─────────────────────┐               ┌────────────────────────┐
│  gossip_node        │               │                        │
│  ├─ PeerProfile     │──DDS Router──▶│   /gossip/peers        │◀──▶ GCS
│  │  ├─ GPS/heading  │               │                        │
│  │  ├─ waypoint     │               └────────────────────────┘
│  │  └─ payloads[]   │
│  └─ peer_registry   │◀── drains inbox, publishes snapshot
└─────────────────────┘
```

Every robot publishes its own `PeerProfile` at 1 Hz (wall-clock) and receives profiles from all peers via the shared domain.

## Packages

### `coordination_msgs`
Wire-format message definitions:
- `PeerProfile.msg` — robot identity, GPS, heading, waypoint, and a typed payload array
- `PeerProfilePayload.msg` — a single serialized ROS message (`payload_type` string + `payload_data` bytes)

### `coordination_bringup`
Runtime nodes and configuration:
- **`gossip_node.py`** — publishes own profile, receives peer profiles, maintains registry
- **`peer_profile.py`** — Python helper class for serializing/deserializing `PeerProfile` and its payloads
- **`frame_utils.py`** — GPS ↔ ENU coordinate conversion helpers
- **`config/gossip_payloads.yaml`** — declares which local topics to attach as payloads (config-driven, no code changes)

## Message Deduplication

Each message is identified by `(robot_name, stamp.sec, stamp.nanosec)`, where the stamp is set at publish time by the originating robot. Receivers maintain a seen-set (50 entries, FIFO eviction) and drop already-processed IDs.

Every drone receives a message at least once — this is expected. The seen-set prevents re-processing, not initial fan-out.

**Relay fields** (`source`, `relay_hops`) exist in the wire format for future multi-hop use but relay logic is not yet active.

## Adding a Payload

Edit `coordination_bringup/config/gossip_payloads.yaml`:

```yaml
payload_topics:
  - topic: "/{robot_name}/your/topic"
    type: "your_msgs/msg/YourType"
```

`{robot_name}` is substituted at runtime. Topics that haven't published yet are silently skipped.

See [`.agents/skills/attach-gossip-payload`](../../../.agents/skills/attach-gossip-payload/SKILL.md) for the full workflow including GCS visualization.

## Topics

| Topic | Direction | QoS | Purpose |
|-------|-----------|-----|---------|
| `/gossip/peers` | pub/sub | BEST_EFFORT | Shared profile bus across all robots and GCS |
| `/{robot_name}/coordination/peer_registry` | pub | RELIABLE, TRANSIENT_LOCAL | Snapshot of all known peers (latest-wins) |

## Key Parameters (`gossip_node`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_name` | — | Robot identifier, also used as topic namespace |
| `publish_rate` | `1.0` | Hz, wall-clock (fires even when sim time is paused) |
| `gossip_domain` | `99` | Shared DDS domain for the gossip bus |

## Future Plans

- **Payload version hashing** — hash `payload_data` bytes and skip re-sending unchanged payloads (e.g. static voxel maps). Reduces gossip bandwidth by up to 90% for slow-changing payloads like PointCloud2 maps.

- **OLSR Multipoint Relay (MPR)** — when relay forwarding (`SOURCE_RELAYED`, `relay_hops`) is activated, use OLSR MPR selection to elect the minimal set of relay nodes that cover all 2-hop neighbors. Prevents O(n²) message explosion from naive flooding in partial-mesh / long-range deployments.
