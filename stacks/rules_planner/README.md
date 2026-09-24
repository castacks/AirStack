# `rules_planner` — full_default + RayFronts + the task_assigner rules planner

The stack the `rules-planner` branch exists for. It is `full_default` (trunk
reference topology: MIGHTY local planner, VDB world model, safety monitor, DDS
bridge, gossip) with three changes:

1. **RayFronts** semantic mapping per robot, ported from the
   `krrishj/disaster-dataset` branch: the `common/rayfronts` submodule, the
   configs and vocabularies in `common/rayfronts_configs/`, the
   `perception_bringup` launch + odom bridge, the `SKIP_RAYFRONTS` /
   `runtime-rayfronts` robot image stages, and the `offboard-compute`
   container (`robot/docker/docker-compose.yaml`) that can host the shared
   RayFronts mapper and the model servers off the robots.
2. **task_assigner rules planner** — the COA-SEI/task_assigner module
   (`airstack.yaml` declares it from `../task_assigner`). Reads the scenario
   template, asks a locally served LLM for areas of exploration, matches the
   labels RayFronts emits against them, and issues survey tasks to large
   drones and investigate tasks to small ones. Design and rules:
   `task_assigner/docs/rules_planner.md`. One per fleet
   (`RULES_PLANNER=true` in exactly one container), plus its dashboard.
3. **task_assigner task executor**, one per robot, **in place of
   `random_walk_planner`**. It takes the tasks whose `assigned_to` maps to this
   robot and flies them, reporting back so the planner can hand out the next
   one. It is this stack's GLOBAL PLANNER, which is why random walk is gone:
   both publish `global_plan` and both drive `tasks/navigate`, so they cannot
   coexist. `RULES_EXECUTOR=false` gets the trunk random walk back.

## What actually flies the tasks

```
rules_planner_node (robot_1)                        task_executor_node (every robot)
  ──► /dtc/task_queue_updates  ────── DDS router ──►  keep the tasks assigned to MY drones
      {"command":"add_task",            (domain 0)    (fleet file's robot_name: large_1→robot_1,
       "task":{... "assigned_to":                      small_1→robot_2, small_2→robot_3)
             "large_1" ...}}                                   │
                                                               ▼
                                            polygon → lawnmower at the task altitude
                                            bbox/point → centroid + dwell_s
                                                               │
                            per leg (RULES_NAV_BACKEND=trajectory_override, default):
                                    TrajectoryXYZVYaw on /<robot>/trajectory_controller/
                                    trajectory_override + set_trajectory_mode TRACK
                            per leg (RULES_NAV_BACKEND=navigate_action):
                                    NavigateTask goal on /<robot>/tasks/navigate
                                    + nav_msgs/Path on /<robot>/global_plan
                                                               │
                                     (MIGHTY bridge →) trajectory_controller → PID
                                                               │
  ◄── /dtc/rules_planner/task_status ── DDS router ──  active / complete / failed
      mark_task() frees the drone; the next tick gives it the next sector
```

**The seam, with its evidence.** `docs/robot/autonomy/interface_conventions.md`
§8 and `stacks/full_default/wiring.md`:

| what | name | type | server |
|---|---|---|---|
| fly a leg (default) | `/<robot>/trajectory_controller/trajectory_override` | `airstack_msgs/msg/TrajectoryXYZVYaw` | `trajectory_controller` — the same topic `takeoff_landing_planner` publishes takeoff on |
| fly it / hover (default) | `/<robot>/trajectory_controller/set_trajectory_mode` | `airstack_msgs/srv/TrajectoryMode` (`TRACK` = fly, `ROBOT_POSE` = hover) | same node; TRACK **clears** the trajectory, so the mode call goes first and the leg is published from its response |
| navigate (`navigate_action`) | `/<robot>/tasks/navigate` | `task_msgs/action/NavigateTask` (`nav_msgs/Path global_plan`, `float32 goal_tolerance_m`) | the local planner — `mighty_bridge` here |
| follow (`navigate_action`) | `/<robot>/global_plan` | `nav_msgs/Path` | MIGHTY follows this topic directly, which is why `full_raven` and `full_exploration` need no adapter |
| takeoff | `/<robot>/tasks/takeoff` | `task_msgs/action/TakeoffTask` (`float32 target_altitude_m`, `float32 velocity_m_s`) | `takeoff_landing_planner` |
| airborne | `/<robot>/takeoff_landing_planner/is_airborne` | `std_msgs/Bool` | same node |
| where it is | `/<robot>/odometry_conversion/odometry` | `nav_msgs/Odometry` | `odometry_conversion` — this is what decides "reached", not the action result |

**Takeoff is automatic.** `takeoff_landing_task.cpp` sends `RobotCommand::ARM`
and `RobotCommand::REQUEST_CONTROL` itself before climbing, so the executor's
single `TakeoffTask` goal covers arming and offboard control too — no operator
step, and no separate arming call in the executor. The goal is REJECTED while
PX4's EKF has not converged (`tests/system/test_takeoff_hover_land.py` waits for
`interface/mavros/local_position/odom` and for the safety monitor's
`state_estimate_timed_out: false` first, and that is minutes on a cold sim), so
a rejection is treated as transient: the executor **retries**, every
`takeoff_retry_period_s` (10 s) up to `takeoff_max_attempts` (30) — five
minutes — and only then falls through to publishing `global_plan` anyway, so an
operator takeoff from the GCS **Robot Tasks** panel picks the mission up where
it is. `is_airborne` going true at any point ends the retrying.
`RULES_AUTO_TAKEOFF=false` makes the operator path the only one.

**`RULES_NAV_BACKEND` decides which of those two seams the legs use.** The
default `trajectory_override` drives the trajectory controller directly, the
way `takeoff_landing_task.cpp` does: a straight leg densified every 10 m at the
drone's cruise speed, yaw along the direction of travel, climbing to the task
altitude while it flies. There is **no obstacle avoidance** on that path, so it
is for open-air survey altitudes only — which is why the rules planner's
close-look altitudes now have a 15 m floor. `navigate_action` is the old
behaviour (`tasks/navigate` + `global_plan`): MIGHTY is map-based over a
0..6 m sliding window and, with no LiDAR map in this scene, it accepts a goal
for a 70 m leg 300 m away and never moves. Switch back to it once the scene has
a map.

## How tasks cross the domains

Each robot container is on its own DDS domain (`ROS_DOMAIN_ID = N`) and runs
its own `interpolate_dds_router` bridging that domain to the **GCS domain 0**.
`/dtc/...` names are global, not namespaced, so:

```
robot_1 (domain 1) ──router──► domain 0 ──router──► robot_2 (domain 2), robot_3 (domain 3)
                                  │
                                  └──► GCS: Foxglove, rviz
```

Domain 0 is the hub. The **gossip** domain (99) is not involved: its router
carries `coordination_msgs/PeerProfile` only, and a topic in both routers
amplifies (the note at the end of `dds_router.yaml`). The allowlist entries
that make this work are in
[`robot/ros_ws/src/autonomy_bringup/config/dds_router.yaml`](../../robot/ros_ws/src/autonomy_bringup/config/dds_router.yaml)
under `── rules_planner stack ──`.

**Not verified without a running sim:** that three routers bridging the same
global topic to one hub domain do not echo back into their own robot domain.
eProsima's router does not return data to the participant it came from, so it
should not — but it has not been observed here.

### What the planner can and cannot see

In the default **per-robot** RayFronts mode each mapper publishes `emb/*` in
its OWN domain, so the planner in robot_1 sees **robot_1's map only**. Two ways
out, and they are not equal:

- `RAYFRONTS_MODE=shared` — one mapper on `offboard-compute` for the whole
  fleet. Nothing large crosses the router. Prefer this.
- Bridge `emb/*` (the five entries already in the allowlist) and list all three
  prefixes on the planner node
  (`rayfronts_prefixes:=['/robot_1/rayfronts/msg_serv', '/robot_2/...', '/robot_3/...']`).
  **This is expensive**: `emb/voxels` is one point per semantic voxel with a
  128-float embedding (~540 B/point, tens of MB on a 500 m plate) and RayFronts
  republishes the whole map every cycle. Worse, those topics are
  **subscriber-gated** and a DDS-router reader does not open the gate — the
  mapper's own container needs a keepalive subscription
  (`voxel_keepalive_pattern` in `perception_bringup/launch/rayfronts.launch.xml`)
  or the bridge carries nothing at all.

## How to run

```bash
airstack sync                                   # modules.repos ← airstack.yaml, overlay task_assigner
airstack up --stack rules_planner --sim isaac   # robot image builds with RayFronts
```

### The demo, end to end

Ollama on the host (any OpenAI-compatible server will do):

```bash
ollama serve &                       # host network, port 11434
ollama pull qwen2.5:7b-instruct
```

Then the 500 m suburban-earthquake cell with three drones:

```bash
ISAAC_SIM_SCRIPT_NAME=example_multi_drone_scene_import.py \
FROZEN_SCENE=Earthquake/Suburban500/level_1/1 \
REGION_M=500x500 \
NUM_ROBOTS=3 \
SPAWN_CONFIGS='[{"x_m": -6.0, "y_m": 0.0}, {"x_m": 0.0, "y_m": 0.0}, {"x_m": 6.0, "y_m": 0.0}]' \
SPAWN_HEIGHT_M=0.5 \
GT_ANNOTATIONS=on RESULTS_SCENE=EarthquakeSuburban500L1 \
RAYFRONTS_EMB=true \
RULES_PLANNER=true RULES_PLANNER_ROBOT=robot_1 \
RULES_EXECUTOR=true RULES_DASHBOARD=true \
OPENAI_BASE_URL=http://172.31.0.1:11434/v1 \
RULES_LLM_MODEL=qwen2.5:3b-instruct \
RULES_GT_PEOPLE=/root/final_disaster_dataset/Earthquake/Suburban500/level_1/1/GT_people.json \
LAYOUT_TEMPLATE=/root/AirStack/gcs/foxglove_extensions/rules_planner.json \
airstack up --stack rules_planner --sim isaac --robots 3
airstack ready
```

`--fleet sim_rules_planner` names the same three robots but **cannot be
combined with `FROZEN_SCENE`** (see *Fleet files* below); use the env route for
the frozen cell.

That cell is on disk: `${FINAL_DATASET_DIR}/Earthquake/Suburban500/level_1/1`
(`earthquake_suburban_500_lvl1_1.usd`, `GT_hints.json`, and a `GT_people.json`
with **9 survivors**, every one of them `alive: null` — untriaged, which the
GT layer draws as unknown-alive rather than dead). It is a 500 m plate centred
on the origin, and its survivors sit between x −219…−7 and y
78…180 — the north-west quadrant, so the large drone's sweep reaches
them a few cells in rather than immediately. `airstack up` bind-mounts the same
host directory into the isaac-sim container at `/isaac-sim/final_disaster_dataset`
and, now, read-only into each robot container at `/root/final_disaster_dataset`
— which is why `RULES_GT_PEOPLE` above is a `/root/...` path while
`FROZEN_SCENE` is a bare cell path.

**One planner, picked by NAME.** The three robot containers are **replicas of
one compose service**, so they all read the same environment —
`RULES_PLANNER` cannot be true in one of them and false in the others.
`RULES_PLANNER_ROBOT` (default `robot_1`) names the host instead, and
`rules_planner.launch.py` starts the planner and its dashboard only where the
container's own `ROBOT_NAME` matches. `RULES_PLANNER=false` still turns the
planner off fleet-wide. `RULES_EXECUTOR` is the opposite kind of switch: true
everywhere, because the executor is this stack's global planner.

**LLM size vs GPU.** On one 16 GB card shared with Isaac Sim (~7.5 GB) and one
RayFronts mapper (~3.7 GB), `qwen2.5:3b-instruct` (~2.3 GB) fits on the GPU and
answers in a few seconds; `qwen2.5:7b-instruct` does not fit alongside them and
falls back to CPU, where a decision takes minutes. Run the 7B on a second GPU or
a separate host when quality matters more than latency.

**Reaching the host's Ollama from a robot container.** The robot containers are
**not** host-networked — only `robot-l4t` and `robot-voxl` are.
`robot-desktop` joins `airstack_network`, a user-defined bridge on
**172.31.0.0/24**, so the host is the gateway **172.31.0.1**. It is *not*
`172.17.0.1`: that is the default `bridge` network's gateway, and these
containers are not on that network. The Ollama container here runs with host
networking and listens on `*:11434`, so every host address answers, but
`172.31.0.1` is the one that is true by construction for this compose network
— which is why the command above uses it.

`host.docker.internal` works too now: `robot_base` maps it
(`extra_hosts: host.docker.internal:host-gateway`), which Linux does not do by
default. It resolves to Docker's *host-gateway* address (the default bridge,
172.17.0.1, unless the daemon is configured otherwise), so it also depends on
the server listening on that interface — prefer `172.31.0.1` and keep the
alias as the portable fallback. `network_mode: host` is not an option here: it
would collapse the per-robot DDS domains the whole multi-robot design rests on.

Check it from inside before flying:

```bash
docker exec airstack-robot-desktop-1 bash -c \
  "curl -s http://172.31.0.1:11434/v1/models | head -c 200"
```

`qwen2.5:3b-instruct` is pulled alongside the 7B and is the fallback when the
bigger model cannot keep up with `tick_period_s`.

### What to watch

| where | what |
|---|---|
| **Dashboard** | `http://localhost:8791/` — AOEs, open tasks with owner and priority, labelled boxes (life-safety highlighted), per-drone current task, executor reports. `docker compose port robot-desktop 8791 --index 1` if the replica took another host port. `/api/state` is the same as JSON. |
| **Foxglove** | the `rules_planner` layout, below. |
| **Terminal** | `docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /dtc/rules_planner/task_status"` |

### Foxglove

[`gcs/foxglove_extensions/rules_planner.json`](../../gcs/foxglove_extensions/rules_planner.json)
is a `render_layout.py` TEMPLATE, exactly like `airstack_default.json`: tab 0 is
the robot_1 template and the per-robot keys of the 3D panel are expanded for
`NUM_ROBOTS`. Setting `LAYOUT_TEMPLATE` (now forwarded by
`gcs-base-docker-compose.yaml`) makes the GCS render and seed THIS layout on
startup instead of the default one; without it, import the file by hand in
Foxglove (Layouts → Import from file — the directory is mounted at
`/root/AirStack/gcs/foxglove_extensions`).

What is in it:

- **3D panel** — the planner's `/dtc/rules_planner/markers` (a coloured cube +
  label per semantic box), `/dtc/rules_planner/survivors` (red spheres, the
  life-safety boxes) and `/dtc/rules_planner/gt_people` (green spheres, the
  cell's answer key) so found-vs-truth is one glance; per robot
  `rayfronts/voxel_rgb`, `rayfronts/semantic_frontiers`, `rayfronts/frontiers`,
  `global_plan`, `trajectory_controller/trajectory_vis` and an odometry axis
  trail; VDB map and LiDAR present but off.
- **Per-robot tabs** — the Robot Tasks panel (takeoff/land/navigate by hand)
  next to the left camera.
- **Raw messages** on `/dtc/rules_planner/task_status` and
  `/dtc/rules_planner/snapshot`, and a **table** on `/dtc/task_queue_updates`.

`rayfronts/semantic_ray_frontiers` is deliberately absent: it is a
`MarkerArray`, and `voxel_keepalive_node` only subscribes `PointCloud2`, so
nothing can hold its publisher gate open across the router.

Per-fleet knobs (compose env; all forwarded by
`robot/docker/robot-base-docker-compose.yaml`):

| var | default | meaning |
|---|---|---|
| `RULES_PLANNER` | `true` | fleet-wide on/off for the planner |
| `RULES_PLANNER_ROBOT` | `robot_1` | WHICH container runs the one planner (+ dashboard): matched against that container's `ROBOT_NAME`, because replicas share one environment |
| `RULES_EXECUTOR` | `true` | run the per-robot task executor; **replaces** `random_walk_planner` as this stack's global planner |
| `RULES_DASHBOARD` | `true` | serve the planner's page beside the planner node |
| `RULES_DASHBOARD_PORT` | `8791` | port for it (8790 is the DTC dashboard) |
| `RULES_TRACK_SPACING_M` | `25.0` | lawnmower track spacing over a sweep/survey polygon |
| `RULES_AUTO_TAKEOFF` | `true` | executor requests `tasks/takeoff` once before its first waypoint |
| `RULES_TAKEOFF_ALTITUDE_M` | `10.0` | altitude that takeoff climbs to |
| `RULES_NAV_BACKEND` | `trajectory_override` | how a leg is flown: `trajectory_override` drives `trajectory_controller` directly (no obstacle avoidance), `navigate_action` sends `NavigateTask` + `global_plan` to the local planner (needs a map) |
| `RULES_GT_PEOPLE` | the shipped Earthquake/Suburban500 cell | a cell's `GT_people.json` **as the robot container sees it** (`/root/final_disaster_dataset/...`) for the GT layer; a path that is not there is a warning and an empty layer |
| `OPENAI_BASE_URL` | unset | OpenAI-compatible endpoint of the local LLM (Ollama `/v1`, vLLM, llama.cpp) |
| `RULES_LLM_MODEL` | `qwen2.5:7b-instruct` | model name at that endpoint |
| `RULES_LLM_BACKEND` | `openai` | `fake` = no LLM, scripted responses (CI) |
| `RAYFRONTS_EMB` | `false` | the mapper's `emb/*` export the planner reads — **must be true** |
| `RAYFRONTS_MODE` | unset (per_robot) | `shared` = one mapper on `offboard-compute` for every robot |
| `LAYOUT_TEMPLATE` | unset | Foxglove layout the GCS renders and seeds (GCS container path) |


## Scene

The stack flies a **frozen dataset cell**: a finished suburban-earthquake plate
exported on the `krrishj/disaster-dataset` branch by `scene_gen/disaster/freeze.py`
— one self-contained `.usd`, a `Materials/` folder, `GT_hints.json`,
`GT_people.json` and `freeze_report.json`. The *loader* is ported here
(`simulation/isaac-sim/launch_scripts/example_multi_drone_scene_import.py`, plus
`spawn_utils.py` and `utils/{frozen_annotations,scene_annotations}.py`); the
*generator* is not, so `SCENE_CONFIG` exits with a message instead of trying to
build a plat.

Cells live outside the repo, bind-mounted into the isaac-sim container:

```
host       ${FINAL_DATASET_DIR}        (.env, default ~/SEI-COA/final_disaster_dataset)
container  /isaac-sim/final_disaster_dataset
```

`FROZEN_SCENE` is a **bare cell path** relative to `FROZEN_DATASET_ROOT` (the
filename inside the cell is derived from the dataset contract), so the same
string addresses the bind mount and a Nucleus mirror
(`omniverse://<host>/Projects/SEI-COA/final_disaster_dataset`) — switching
between a bench and a pod is one variable.

### Pointing at a cell

Uncomment the FROZEN block in [`.env`](../../.env), or pass the vars on the
command line. The 500 m x 500 m suburban earthquake, level 1:

```bash
ISAAC_SIM_SCRIPT_NAME=example_multi_drone_scene_import.py \
FROZEN_SCENE=Earthquake/Suburban500/level_1/1 \
REGION_M=500x500 \
NUM_ROBOTS=3 \
SPAWN_CONFIGS='[{"x_m": -6.0, "y_m": 0.0}, {"x_m": 0.0, "y_m": 0.0}, {"x_m": 6.0, "y_m": 0.0}]' \
SPAWN_HEIGHT_M=0.5 \
GT_ANNOTATIONS=on RESULTS_SCENE=EarthquakeSuburban500L1 \
airstack up --stack rules_planner --sim isaac --robots 3
```

Compose does not forward the host environment on its own — every one of those
names is listed in `simulation/isaac-sim/docker/docker-compose.yaml`, and a var
absent from that list never reaches the launch script. That reads like the
script ignoring the mission rather than like a plumbing gap, which is exactly
how `FROZEN_SCENE` was lost before.

### The three-drone fleet on the 500 m plate

`SPAWN_CONFIGS` is a JSON list of dicts needing only `x_m`/`y_m` (world metres,
+X east, +Y north). `domain_id` defaults to the list index — `robot_1..3`, which
is what each container's `ROBOT_NAME` matches — `z_m` to `SPAWN_HEIGHT_M`,
`orient` to identity (facing +X), `lidar_min_range` to 0.75 m. Three drones 6 m
apart across the plate centre:

```json
[{"x_m": -6.0, "y_m": 0.0}, {"x_m": 0.0, "y_m": 0.0}, {"x_m": 6.0, "y_m": 0.0}]
```

**Check these against the cell before flying it.** They are the plate centre,
not a surveyed clearing: a suburb plate puts kerbs, houses and yard trees near
the origin, and a drone spawned inside geometry never reports ready. The cell's
`GT_hints.json` carries the obstacle boxes to check against; `SPAWN_POLY`
(a rect `[[x0,y0],[x1,y1]]` or a convex polygon) plus `SPAWN_MIN_DIST_M` is the
alternative when you would rather sample a clear area than name three points.

`SPAWN_CONFIGS` applies to the frozen path only. The ENV_URL path (the Fire
Academy digital twin, the script's default when `FROZEN_SCENE` is unset) keeps
its own three-drone apron fleet.

### Fleet files

[`config/fleets/sim_rules_planner.yaml`](../../config/fleets/sim_rules_planner.yaml)
leaves `sim.scene: default`. **The fleet schema has no way to name a frozen
cell**: `fleet_spawn.fleet_env_url` resolves `sim.scene` to a Pegasus
`SIMULATION_ENVIRONMENTS` key or a USD reference and hands it to
`pg.load_environment`, whereas a frozen cell is composed prim-by-prim by a
different launch script, which also authors its sky, sun and asset rebase.
Naming the cell's `.usd` there would load the geometry unlit and un-rebased.
Use the env route above; `--fleet` and `FROZEN_SCENE` are not combinable today.

**Two fleet files, and they are not the same thing.** This one is AirStack's:
who exists, which vehicle, which stack. The planner's is
`task_assigner/config/rules_planner/examples/fleet_1large_2small.yaml`: which
drone is `large` and which is `small`, and — the field that joins the two —
`robot_name`:

```yaml
drones:
  - drone_id: large_1
    drone_class: large
    robot_name: robot_1     # the AirStack robot that flies it
```

`task_executor_node` reads it (`Fleet.drone_ids_for_robot`) to decide which of
the planner's `assigned_to` ids are its own. Change the mapping in that file,
not in the stack launch. With no fleet file, give each executor `drone_ids`
directly.

### Ground truth

With `GT_ANNOTATIONS=on` and `RESULTS_SCENE=<name>` the loader turns the cell's
`GT_hints.json` / `GT_people.json` into `<name>.json`, `<name>_obstacles.json`
and `<name>_region.json` under `gcs/ros_ws/src/gcs_visualizer/annotations/`
(and `robot/ros_ws/src/global/planners/raven_nav/annotations/` where that module
is checked out). A directory that does not exist is skipped silently, so an
unset `RESULTS_SCENE` costs nothing. Note this branch's `gcs_visualizer` has no
`annotation_viz_node`, so the files are written for the scorer, not drawn.

### What the frozen path needs that a plain scene does not

- **The `disaster-dataset` PegasusSimulator.** The per-drone ZED call passes
  `frame_width`, `frame_height`, `pipeline_mode` and `manage_render_updates`,
  and expects `add_rtx_lidar_subgraph` to return a gate. This branch's submodule
  pin does not have them — check
  `git -C simulation/isaac-sim/extensions/PegasusSimulator log -1` before
  blaming the scene for a `TypeError` during spawn.
- **Nucleus, for lighting.** A cell ships no sky (the freeze deactivates
  `/World/Environment`, and the sun lives in it), so the loader borrows
  `RetroNeighborhood.stage.usd` off Nucleus. Unreachable → it falls back to a
  plain dome and says so; the scene is lit, just flat. `FROZEN_SKY` overrides it.
- **Nucleus, for textures**, unless the cell was built on this machine. A cell's
  texture paths are absolute build-container paths under `scene_gen/assets/`,
  much of which is git-ignored; `FROZEN_REBASE_ASSETS` repoints any path that is
  genuinely absent locally onto the Nucleus mirror.

## Known limits

- Not yet flown in simulation. The planner and the executor are exercised only
  by `task_assigner/tests/rules` (unit + integration with a fake LLM and a fake
  text head); every seam named above was read out of the sources and the
  observed `full_default` wiring, not off a running graph.
- The rules planner is fleet-level but this stack is per-robot. The robot
  containers are compose replicas of one service, so the selection cannot be a
  flag: `RULES_PLANNER_ROBOT` names the host and the launch file compares it to
  each container's `ROBOT_NAME`. Moving the planner to a ground host once the
  split-stack pattern (`lite_offload_global`) is applied here would drop that
  comparison entirely, which is the cleaner end state.
- **Altitude is AGL only by assumption.** The executor publishes the task's
  `altitude_agl_m` as map-frame z (plus `ground_z_m`, default 0). `map` is
  anchored at takeoff, so that is right over flat ground and wrong over a
  slope; there is no DEM in a frozen cell to convert against, and the
  DEM-based conversion in `airstack_waypoint_adapter_node` needs TEEX map
  products this mission does not have.
- **Nothing bounds a leg to the cell.** The executor flies what the planner
  asked for; the search-area polygon comes from the scenario file, so a bad
  `area_polygon` flies the fleet off the plate. There is no geofence here.
- `wiring.md` is not generated yet; run `airstack test -m wiring --stack rules_planner`
  after the first successful bring-up and commit the snapshot.
