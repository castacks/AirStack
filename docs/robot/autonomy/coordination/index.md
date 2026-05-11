# Coordination

The coordination layer lets drones share state with each other and the GCS without a central broker. Each robot periodically broadcasts a `PeerProfile` — its GPS position, heading, current waypoint, and any custom data payloads — over a shared DDS domain. Every robot and the GCS receives every other robot's profile directly.

## Architecture

```
Robot 1 (domain 1)          Shared gossip domain (99)          GCS (domain 0)
┌──────────────────┐         ┌─────────────────────┐         ┌──────────────┐
│  gossip_node     │──────▶  │   /gossip/peers      │ ──────▶│  GCS         │
│  publishes own   │         │                      │        │  visualizer  │
│  PeerProfile     │ ◀──────  │  (all robots + GCS   │        └──────────────┘
│                  │         │   subscribe here)    │
│  local registry  │         └─────────────────────┘
│  (read-only)     │
└──────────────────┘
         ▲
    DDS Router
    bridges domain 1
    ↔ domain 99
```

Each robot builds a local registry of all known peers from incoming messages. The registry never leaves the robot — only each drone's own profile is transmitted.

## PeerProfile

Every message on `/gossip/peers` is a `PeerProfile` containing:

| Field | Type | Description |
|---|---|---|
| `robot_name` | string | Unique robot identifier |
| `gps_fix` | NavSatFix | Current GPS position (also used as message ID for dedup) |
| `heading` | float64 | Compass heading, degrees CW from North |
| `waypoint` | PoseStamped | Current navigation goal (all-zeros = no plan) |
| `payloads` | PeerProfilePayload[] | Arbitrary serialized ROS messages |
| `source` | uint8 | `0` = direct, `1` = relayed (reserved) |
| `relay_hops` | uint8 | Hop count (reserved) |

## Launch

Coordination is included in the main autonomy bringup automatically. To launch standalone:

```bash
ros2 launch coordination_bringup gossip.launch.xml
```

Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `robot_name` | `$ROBOT_NAME` | Robot identifier and topic namespace |
| `publish_rate` | `1.0` | Publish rate in Hz (wall-clock) |
| `gossip_domain` | `99` | Shared DDS domain |

## Monitoring

```bash
# Live peer registry in the terminal
ROS_DOMAIN_ID=99 ros2 run coordination_bringup peer_registry_monitor

# Filter to one robot
ROS_DOMAIN_ID=99 ros2 run coordination_bringup peer_registry_monitor --robot robot_1

# Inspect raw messages
ros2 topic echo /gossip/peers
```
