# Creating a Multi-Agent Coordination Algorithm

Each robot runs on its own DDS domain ([robot N gets `ROS_DOMAIN_ID=N`](../../docker/robot_identity.md)), so robots cannot see each other's topics directly — a node on `robot_1` will never discover `/robot_2/odometry`. The supported cross-robot channel is the [coordination layer](index.md): each robot's `gossip_node` broadcasts a `PeerProfile` (GPS, heading, current waypoint, plus arbitrary payloads) on the shared gossip domain (default **99**), bridged from the robot's own domain by a dedicated DDS router. A coordination algorithm in AirStack is therefore: attach your state as a gossip payload, consume the peer registry, decide, and act through the stack's normal interfaces.

Canonical names, types, and QoS for the gossip channel are in the [Interface Conventions Spec §10](../interface_conventions.md#10-gossip--multi-robot-coordination).

## Step 1 — Decide what state must cross robots, and attach it as a payload

The `PeerProfile` already carries GPS position, heading, and the current waypoint for free — many coordination schemes (spatial dispersion, follow-the-leader, deconfliction) need nothing more. If your algorithm needs additional per-robot state (a frontier map, a task bid, a mode string), publish it as a normal topic on the robot's own domain and declare it in `gossip_payloads.yaml` — the gossip node serializes the latest message onto every 1 Hz profile tick, transforming `MarkerArray`/`PointCloud2` payloads into global ENU on the way out. The full recipe is [Payloads & Foxglove Visualization](payloads.md) (or run the [attach-gossip-payload skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/attach-gossip-payload/SKILL.md)); don't re-derive it here. Keep payloads small — they ride inside every profile message at the gossip rate.

**Verify:** `ros2 topic echo /gossip/peers --field payloads` shows an entry with your `payload_type` string.

## Step 2 — Write the coordination node that consumes peer profiles

Your node runs on the robot's **own** domain, like every other stack node. Two verified inputs:

- **`/{robot_name}/coordination/peer_registry`** (`coordination_msgs/msg/PeerProfile`, RELIABLE + TRANSIENT_LOCAL) — the gossip node's latest-wins snapshot of every known peer, republished on the robot's domain each time a peer updates. TRANSIENT_LOCAL means a late-joining node receives the current registry immediately. This is the right input for in-stack consumers: no domain gymnastics, dedup and monotonic-timestamp filtering already done by `gossip_node`.
- **`/gossip/peers`** (same type, BEST_EFFORT) — the raw bus, but only visible on domain 99. Use it for out-of-stack tooling (this is what `peer_registry_monitor` subscribes to), not for stack nodes.

Deserialize payloads with the helper API from `common/ros_packages/coordination/coordination_bringup/coordination_bringup/peer_profile.py`:

```python
from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from coordination_bringup.peer_profile import PeerProfile

def on_peer(self, msg: PeerProfileMsg):
    profile = PeerProfile.from_ros_msg(msg)
    bid = profile.get_payload("std_msgs/msg/String")        # by type
    cloud = profile.get_payload_by_name("raw_frontiers")     # by topic tail
```

For outputs, use the stack's existing verified command mechanisms rather than inventing a side channel: publish a `global_plan` (`nav_msgs/msg/Path` — [the interchange](../interface_conventions.md#4-global_plan--global-waypoint-path) the local planner consumes), or call a task action server under `tasks/*` (e.g. `tasks/navigate` / `task_msgs/action/NavigateTask`, [§8](../interface_conventions.md#8-tasks--task-action-servers)). To share your decision back to peers, attach it as another payload (Step 1).

**Package or module?** The node can live in-tree as a normal ROS 2 package ([add-ros2-package skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/add-ros2-package/SKILL.md)), or ship as a standalone module repo pinned by whichever stacks use it — the same pattern as `asm_macvo`. See [AirStack Modules](../../../development/modules.md) and the [create-module skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/create-module/SKILL.md); `airstack module create --in-tree <name>` scaffolds the module boundary in a fork.

**Verify:** with the stack running, `docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /robot_1/coordination/peer_registry --once"` returns a `PeerProfile`, and your node appears in `ros2 node list`.

## Step 3 — Wire it into a stack

The gossip layer itself is already included in every reference stack — each entry launch file includes `coordination_bringup`'s `gossip.launch.xml` (e.g. `stacks/full_default/launch/stack.launch.xml`, "Gossip coordination layer" block, passing `gossip_domain`/`gossip_publish_rate`). You do **not** add anything to make peer profiles flow.

Your coordination node is a new module include in the stack entry file — one `<include>` with your topic args, per the single-locus rule. If you're modifying a topology, make your own stack first: see [Creating a Custom Stack Topology](../../../development/creating_a_stack.md) and the [integrate-module-into-layer skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/integrate-module-into-layer/SKILL.md). Do not touch `coordination_bringup` or `autonomy_bringup`.

**Verify:** `airstack up --stack <your_stack> --sim isaac` brings the node up (`ros2 node list` inside the robot container shows it alongside `gossip_node`).

## Step 4 — Test with multiple robots

```bash
airstack up --sim isaac --robots 3          # replicas: airstack-robot-desktop-1/-2/-3, domains 1/2/3
# or, for heterogeneous fleets / split stacks:
airstack up --fleet <name> --sim isaac      # config/fleets/<name>.yaml
airstack ready
```

`--robots N` also selects the multi-robot Isaac spawn script; fleet identity/placement details are in [AirStack Fleets](../../../development/fleets.md).

Watch the peer registry converge — every robot should list every other:

```bash
ROS_DOMAIN_ID=99 ros2 run coordination_bringup peer_registry_monitor            # the shared bus view
ROS_DOMAIN_ID=1  ros2 run coordination_bringup peer_registry_monitor            # what robot_1 actually receives
ros2 topic echo /gossip/peers                                                    # raw messages
```

(Run these inside a robot container via `docker exec ... bash -c "..."`.) Then exercise your algorithm and confirm the decisions land: the acted-on robot's `global_plan` changes, or the `tasks/*` action goal is accepted.

**Verify:** the monitor shows all N robots with fresh timestamps and your `payload_type` listed per peer, and your node's output topic/action responds on each robot.

## What the platform does not give you yet

Read `common/ros_packages/coordination/README.md` before designing around these:

- **No relay/multi-hop.** The `source`/`relay_hops` wire fields are reserved; relay logic is not active. Every robot must reach the shared gossip domain directly.
- **No payload delta suppression.** Payloads are re-serialized and re-sent on every publish tick even when unchanged (payload version hashing is a future plan) — budget bandwidth accordingly.
- **Delivery is BEST_EFFORT and at-least-once.** The seen-set (`(robot_name, stamp)` dedup, 50-entry FIFO) prevents re-processing, not loss; design decisions to be idempotent under a 1 Hz latest-wins stream.
- **Registry entries are never evicted.** A crashed peer stays in the registry until the node restarts — check profile timestamps yourself if liveness matters.

Consensus, task allocation, and leader election are yours to build on top of this bus; the platform provides the state-sharing substrate, not the algorithms.
