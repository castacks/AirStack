# RAVEN navigation

RAVEN consumes RayFronts semantic rays and voxel detections, combines local and
gossiped target observations, and selects navigation goals. The semantic-search
task executor launches it per robot; local planning and control execute goals.

```mermaid
flowchart LR
  Perception[RayFronts rays and voxels] --> Tasks[Shared target table]
  Gossip[Peer observations and completion] --> Tasks
  Tasks --> Selection[Auction bundles or independent greedy]
  Selection --> Behavior[Ray, voxel and frontier behaviors]
  Behavior --> Navigation[Navigation goals and local planning]
```

## Allocation parameters

| ROS parameter | Default | Meaning |
|---|---|---|
| `assignment_mode` | `auction` | `auction` or `independent_greedy` |
| `bundle_len` | 1 | Maximum ordered reservation count; missions normally use 3 |
| `share_rays` | true | Allow bidding on peer ray leads |
| `share_bbs` | true | Allow bidding on peer BB/voxel targets |
| `debug_auction` | true | Publish and save assignment diagnostics |

`RAVEN_ASSIGNMENT_MODE`, `RAVEN_SHARE_RAYS` and `RAVEN_SHARE_BBS` pass through
robot Compose into the semantic-search executor, which supplies the ROS
parameters. Bundle length is supplied in the `SemanticSearchTask` action goal.
Independent greedy requires length 1 and rejects other values at startup.

The auction assigns exclusive heads before growing tails, stops at ray tasks,
and limits each tail extension to 20 m. Independent greedy reuses the same
single-robot L1 scoring but does not consider peer allocation or ownership.
It disables target-level peer yielding while preserving shared perception,
completed-target filtering, frontier behavior and navigation safety.

## Interfaces and dependencies

The node depends on ROS 2, NumPy, RayFronts observations and the coordination
peer-state inputs. Topics use the robot namespace. Important outputs include
`debug/auction_solve` (JSON in `std_msgs/String`) and `auction_table`, with
results and intent events also saved by the semantic-search results flow.
See `raven_nav_node.py` and `launch/raven_nav.launch.xml` for topic remappings.

Diagnostics include assignment mode, configured maximum, actual per-agent
bundle occupancy, task count and task status counts, including empty ticks.
In independent mode `local_solves` holds each local decision. Auction-table
rows expose `assigned_agents`; the legacy scalar `assigned` is null when a
task has multiple selectors.

## Validation and experiment

`test/test_independent_greedy.py` checks duplicate selections, independence
from peer inputs, retained local costs, eligibility, completion filtering,
node-level yielding gates and bundle limits 1/3/6/12. Run with the package on
`PYTHONPATH` inside the robot Docker image. Existing sharing tests remain valid.

The full matched experiment and analysis definitions are in
`docs/experiments/raven_bundle_study_20261014.md` at the repository root.
