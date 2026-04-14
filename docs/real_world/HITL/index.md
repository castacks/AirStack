# Hardware-In-The-Loop Simulation

AirStack supports multi-machine HITL where simulation and GCS run on one host, and one or more robot compute hosts run autonomy containers. HITL services use `network_mode: host` so DDS traffic is exchanged directly on your LAN.

## Requirements

- One simulation/GCS host configured per [Getting Started](/docs/getting_started)
- One or more robot hosts (Jetson or desktop) configured per [Real-World Installation](/docs/real_world/installation/)
- All hosts on the same routed network with mutual `ping` reachability

## Discovery Model (Default)

HITL defaults to Fast DDS Discovery Server mode for scalability:

- Discovery server endpoint defaults to:
  - `DISCOVERY_SERVER_IP=192.168.233.25`
  - `DISCOVERY_SERVER_PORT=11811`
- Discovery clients:
  - `isaac-sim-hitl`
  - `gcs-hitl`
  - `robot-desktop-hitl`
- Optional fallback mode:
  - `HITL_DISCOVERY_MODE=static-peer`
  - `FASTDDS_STATIC_PEER_IP=<peer_ip>`

## Services

### Simulation/GCS Host

Run:

```bash
airstack up isaac-sim-hitl gcs-hitl
```

This starts:

- Isaac Sim (`isaac-sim-hitl`)
- GCS (`gcs-hitl`)
- Fast DDS discovery server process inside the `gcs-hitl` container

### Robot Host

For a desktop robot host:

```bash
airstack up robot-desktop-hitl
```

For a Jetson host:

```bash
airstack up robot-l4t
```

## Two-Computer Validation Checklist

1. Verify discovery variables on both hosts:
   ```bash
   echo "$DISCOVERY_SERVER_IP:$DISCOVERY_SERVER_PORT"
   ```
2. Verify DDS discovery server visibility from robot host:
   ```bash
   ping "$DISCOVERY_SERVER_IP"
   ```
3. Verify ROS graph visibility:
   ```bash
   ros2 topic list
   ```
4. Verify payload flow (not just discovery):
   ```bash
   ros2 topic hz /robot_1/odometry
   ros2 topic echo /robot_1/odometry --once
   ```
5. Verify Foxglove:
   - On GCS host: `ws://localhost:8765`
   - On remote client (optional): `ws://<gcs_host_ip>:8765`

## Multi-Robot / Multi-Computer Scaling

For `N` robot hosts:

- `gcs-hitl` starts and owns the discovery server on the sim/GCS host.
- Start one robot container stack per robot host (`robot-desktop-hitl` or `robot-l4t`).
- Ensure each robot has unique `ROBOT_NAME` and `ROS_DOMAIN_ID` mapping.

Recommended:

- Use discovery server mode for `N >= 3`.
- Configure backup discovery servers using `DISCOVERY_SERVER_BACKUP_IPS` (comma-separated IP list).

## Troubleshooting

### Topics discovered but no data

This is usually not discovery failure. Check:

1. DDS Router allowlist includes the topic
2. ROS domain IDs are correct on both sides
3. QoS compatibility between publisher/subscriber
4. Firewall allows DDS UDP traffic
5. Topic namespaced correctly (`/robot_X/...`)

### Foxglove shows no topics

- Confirm `foxglove_bridge` is running in GCS launch
- Confirm GCS is receiving ROS topics in domain 0
- Use `ws://localhost:8765` on GCS host first, then test remote URL

### Discovery server unreachable

- Verify `DISCOVERY_SERVER_IP` matches the host running `gcs-hitl`
- Verify no port conflict on `11811`
- Verify host firewall/network ACLs