# mtl_msgs

Interfaces of the [`mtl_search`](../../../../stacks/mtl_search/README.md) stack.

| Interface | Producer → consumer | Purpose |
|---|---|---|
| `action/SearchMission` | operator / `scripts/mtl_start_mission.sh` → `mtl_search_planner` (`/<robot>/search_mission`) | Plan and fly this robot's search. Goal: `start_mission`, `run_id`, optional `scenario_file`. Result: run folder, plan id, planned and flown length, duration, cells planned. Feedback: phase, progress, remaining, cross-track error, position |
| `msg/SearchPlan` | `mtl_search_planner` → follower, logger (`search/plan`, transient local) | One agent's plan in its `map` frame: `airstack_msgs/TrajectoryXYZVYaw` track; per-sample boresight, arc length, time and planned gimbal angles; sensor and aircraft limits; serviced cells; budget |
| `msg/FollowerStatus` | `mtl_trajectory_follower` → planner, logger (`search/follower_status`) | `IDLE / INGRESS / SEARCH / COMPLETE / ABORTED`, progress, cross-track error, carrot, aim point, gimbal command |

The gimbal command itself is a plain `geometry_msgs/Vector3` on `/<robot>/gimbal/cmd_pitch_yaw`,
and the measured state is on `/<robot>/gimbal/state`: `x = roll`, `y = pitch`, `z = yaw`, in
radians. The angles are Z-Y-X Euler in the earth (ENU) frame, and `pitch > 0` looks down.
