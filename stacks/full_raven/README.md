# full_raven

`full_mighty` with the global planner swapped: the random-walk planner
(`random_walk_planner`) is replaced by **RAVEN** — *Resilient Aerial
Navigation via Open-Set Semantic Memory and Behavior Adaptation* (Kim,
Alama, Kurdydyk, Keller, Keetha, Wang, Bisk, Scherer — CMU AirLab, ICRA
2026) — from the external **asm_raven** module. RAVEN builds a RayFronts
semantic ray-frontier map from the ZED left RGB + ground-truth depth and
picks a behavior each frame (voxel-based → ray-based → LVLM-guided →
frontier exploration) to publish a semantic `global_plan` toward an
operator's open-vocabulary prompt ("fire hydrant"). The rest of the stack
(MIGHTY local planner from the `asm_mighty` module, trajectory controller,
PID, safety monitor, takeoff/landing, GCS) is unchanged from `full_mighty`.
MIGHTY plans on the Ouster LiDAR, so navigation keeps working in dark or
untextured scenes where stereo disparity (the DROAN planners' input) does not.

The module has two halves:

- a **GPU sidecar container** (`raven`, ROS 2 Humble image, ~10 GB) running
  the RayFronts mapper + behavior manager — declared by the module's compose
  fragment and started by `airstack up` while the module is synced;
- **`raven_bridge`** in the robot container, relaying the sidecar's plan onto
  the canonical `global_plan` and holding the operator prompt.

RAVEN publishes plans on a topic (it is not a NavigateTask client); the
MIGHTY bridge follows the `global_plan` topic directly, so no adapter is
needed. The only difference vs `full_mighty` is the global-planner include in
`launch/stack.launch.xml` plus the `asm_raven` pin in `modules.repos` (a
droan_gl-based variant would add trunk's `global_plan_navigate_bridge`).

Bring-up:

```bash
airstack module sync            # pulls asm_raven (+ its RayFronts submodule) per this stack's modules.repos
# first run only: build the sidecar image (~15 min on a 32-core host, ~10 GB); also happens
# implicitly on the first `airstack up`
docker compose -f docker-compose.yaml -f .airstack/generated/docker-compose.modules.yaml build raven
airstack up --stack full_raven --sim isaac --scene retro-neighborhood
airstack ready
```

Then set a semantic target (empty string clears → pure frontier exploration):

```bash
ros2 topic pub --once /robot_1/raven/set_prompt std_msgs/msg/String "{data: 'fire hydrant'}"
ros2 service call /robot_1/raven/clear_prompt std_srvs/srv/Trigger
```

Notes:


- RAVEN's scenes are the AirLab Nucleus stages already in
  `simulation/scenes.yaml`: `retro-neighborhood`, `construction-site`,
  `abandoned-factory` (spawn pose via `ISAAC_SIM_SPAWN_XY`).
- Single robot (`robot_1`, DDS domain 1) by default: the sidecar's
  `RAVEN_ROBOT_NAME` / `RAVEN_ROS_DOMAIN_ID` select another; RAVEN's
  visualization topics are global (`/filtered_voxel_bbox`, `/mode_text`, ...).
- The first sidecar start downloads the RADIO and SigLIP models (torch hub /
  Hugging Face) into `modules/asm_raven/cache/`, which persists across
  container recreations.
- The LVLM-guided behavior needs RAVEN's optional external LVLM server
  (`/lvlm_output`); without one it degrades gracefully (falls through to
  frontier exploration). `RAVEN_BEHAVIORS=voxel,ray,frontier` selects the
  no-LVLM ablation explicitly.
- Sidecar rviz layout: `raven_bridge/rviz/raven.rviz` in the module.
- `AIRSTACK_NO_MODULE_COMPOSE=1 airstack up ...` brings the stack up without
  the sidecar (raven_bridge then idles).
- Module repo: [castacks/asm_raven](https://github.com/castacks/asm_raven)
  (pin `v0.1.0` is a placeholder until the first tag).
