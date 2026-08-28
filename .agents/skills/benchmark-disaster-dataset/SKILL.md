---
name: benchmark-disaster-dataset
description: Run a search-and-rescue BENCHMARK on the disaster dataset — bring the whole stack up from `.env`, fly a method, and read its results. Covers the method matrix (raven_nav, frontier-only, LVLM, conavgpt_baseline, and the `search_baselines` arms launched by name — vlfm / conavgpt2 / nearest — off one shared planner), its three config layers, the `.env`-driven `airstack down` / `airstack up isaac-sim robot-desktop gcs` procedure, the per-run `docker exec` sequence (ITM or VLM server, takeoff, planner), VLFM's BLIP-2 ITM value map and the whole-frame anti-correlation finding, 3D voxel frontiers, what has to SCALE when the scene size changes, the Foxglove layers, and the traps that cost a run — stale DDS profiles, orphaned PX4, identity world->map, a detector threshold that finds nobody, two different things called VLFM, topics that never reach the GCS, a goal class the detector cannot see, a goal lock that never releases on arrival, and a missing setup.cfg that kills every mission at launch. Also covers rayfronts-style PERSISTENT frontier accumulation (keep-outside / replace-inside the active window) and the search/markers and frontier_cloud viz topics. Read before running or tuning any benchmark, and before submitting one to OSMO.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Run a benchmark on the disaster dataset

## When to use

Any time you are about to fly a search method and compare it to another — locally
on a bench scene, or on OSMO against the generated plats. Also read it before
changing a planner's map geometry or detector thresholds, because several of
those numbers are only correct at one scene scale and silently wrong at another.

---

## 1. The method matrix — FIVE baselines, nothing else

| baseline | where it lives | how it chooses where to go | area assignment | needs running |
|---|---|---|---|---|
| **lawnmower** | `search_baselines` `lawnmower.launch.xml` | NO frontier: boustrophedon lanes over its sector, fed to droan as short legs (see below) | its own sector | nothing, no GPU service |
| **frontier** (information gain) | `search_baselines` `frontier.launch.xml` | information gain minus visit cost — VLFM without the VLM | its own sector | nothing, no GPU service |
| **vlfm** | `search_baselines` `vlfm.launch.xml` | BLIP-2 ITM scores the live RGB into a VALUE MAP; highest-value cell wins, minus a distance penalty (§3) | its own sector | `itm_server` on :8100, ~2.5 GiB |
| **conavgpt2** | `search_baselines` `conavgpt2_team.launch.xml` (team mode) | a generative VLM sees ONE merged map and, in one call per round, assigns every robot a numbered frontier (arXiv 2310.07937) | **NONE — the whole map.** Cooperative assignment IS the method; sectoring it would be a different arm | `vlm_server` on :8000, ~5.9 GiB at 7B nf4 |
| **raven** | `raven_nav` inside `semantic_search_task` (no env flag) | rayfronts semantic voxels + ray/voxel behaviours — the house method | its own sector: the per-robot `search_area` polygon in the SemanticSearchTask goal, cut from the same damage footprint (§9.t) | rayfronts in-process + the shared detector |

Every one of the four `search_baselines` arms is the SAME `search_planner`
node with `nav_mode` as a parameter, and that is the experiment rather than a
convenience. They build the SAME map from the SAME frames, apply the SAME
search polygon and altitude bounds, and steer through the SAME droan actuation,
so **a difference in results is attributable to the selection policy and to
nothing else**. Two separate implementations would each have to be argued
equivalent before any comparison meant anything.

**Area assignment.** Every baseline except CoNavGPT2 flies its OWN sector: the
damage footprint from the scene (`search_area_source: scene`, §9.t) is cut
into NUM_ROBOTS rectangles (`sector_partition: rect`, §4b) and robot N flies
rectangle N — for the `search_baselines` arms inside the node, for raven as
the polygon the mission hands each robot in its SemanticSearchTask goal. A
detection across the line is another drone's. CoNavGPT2 is the one exception:
one planner in `team_mode` sees every robot and the whole map, and hands out
frontiers itself (`sector_partition: none` in `conavgpt2_team.yaml`).

**Scoring is not yet on one axis.** raven runs inside `semantic_search_task`,
which owns the search polygon, the sim-time budget and coverage/recall
scoring; the four `search_baselines` arms are launched standalone and report
their own round table. `max_sim_seconds` matches the task's budget, so the
sim-time axis lines up, but say in any result table that the two are scored
by different code until the shared planner is wired into
`semantic_search_task`.

**What is in the tree but is NOT a baseline.** `nearest.launch.xml`, the
raven-side env flags `FRONTIER_ONLY_BASELINE` / `VLFM_BASELINE` /
`LVLM_BASELINE` / `CONAVGPT_BASELINE` (the InternVL "VLM-Assign" assigner in
`conavgpt_baseline/`) and `conavgpt2.launch.xml` (the per-robot, sectored
CoNavGPT2) all still launch, and none of them is in any result table. Leave
every flag unset (`false`) in a mission `env:`; a set one silently swaps the
raven arm for something that is not being compared.

### The experiment plan: fleets of 4 and 8

**Every scored experiment is flown at NUM_ROBOTS=4 and NUM_ROBOTS=8. There are
no 1-drone experiments any more.** The `*_1robot` missions (and the 3- and
5-robot ones the 2026-08-27 review was done on) are smoke tests and history,
not results — do not add rows from them. Each (scene, baseline) cell of the
table is therefore two runs, and a mission file is named for its fleet:
`<scene>_4robot_<x>.yaml`, `<scene>_8robot_<x>.yaml` (template:
`osmo/missions/wildfire1km_5robot_a.yaml`, change NUM_ROBOTS and the spawn
list; conavgpt2 has its own team-mode file, `wildfire1km_5robot_conavgpt2.yaml`).

What must change with the fleet, because nothing scales it for you:

* `SPAWN_CONFIGS` must have exactly NUM_ROBOTS entries (the Isaac launcher
  takes the fleet from the list; 6 m apart on the kerb, within
  `map_extent_m`'s slack).
* the ITM scorer: one serves ~3 robots at the shipped 5 Hz keyframe rate —
  `vlfm_keyframe_period_s` 0.5 for 4 robots, 1.0 for 8 (§3), then READ
  `/metrics` on :8100 in the poller rather than assuming.
* the sectors: 4 or 8 `rect` bands of the damage footprint along its
  principal axis — on the 1 km burn ~300 m and ~150 m along the burn
  respectively, 500-620 m across, so at 8 the rectangles are short and fat
  and every drone's transit is under a minute at 7 m/s (§4b, §9.u).
* the detector and VLM servers are shared and stateless, so 8 robots is 8x
  the request rate on one card — the backlog poller in the mission is the
  measurement of whether that holds.
* the bag: `/gcs/{robot}/...` layers (§9.v) x8 is still small; never add
  image streams per robot.

**The lawnmower never sends its path to the drone.** droan_gl steers by the
closest point of the Path it holds, so a whole coverage path — or a bare lane
end 260 m out, which is the same thing at a different scale — gives it one far
pose to fly straight at with nothing to skip if it is unreachable. Instead
`lawnmower.Sweep` walks the drone to the nearest lane end in legs of
`lawnmower_leg_m` (25 m), then along the lanes in legs of the same length, ONE
goal per tick through the same `_command` as the frontier arms (activator
NavigateTask + a two-pose `/global_plan`: the goal, then the next lane point).
A leg is done when reached or passed abeam; one with no progress for
`lawnmower_stall_s` is skipped. `search_area_pad_m` is 0 for this arm so it
flies exactly the area the others do — and so its lane ends fit the grid: a
padded sector wider than `map_extent_m` has its lane ends clipped to the map
edge (clamped and warned about, but fix the config). Verified offline by
`tests/test_lawnmower_baseline.py` — no sim needed.

**`conavgpt2` is no longer a package you launch — it is a LEAF LIBRARY.**
`conavgpt2/conavgpt2/vendor/` holds the vendored upstream code and nothing else:
no executables, no launch files, no config. Any `ros2 launch conavgpt2 ...` or
`python -m conavgpt2.vlm_server` line you find is pre-refactor and will not
resolve (§9.8). The name still names the METHOD and the vendored library — just
nothing you can launch.

The `search_baselines` family is still the odd one out for COMPARISONS: every
raven_nav arm runs inside `semantic_search_task`, which owns the search polygon,
the sim-time budget, and coverage/recall scoring. The shared planner is launched
standalone and reports only its own round table, so **its numbers are not
directly comparable** until it is wired into `semantic_search_task`. Say so in
any result table. `max_sim_seconds` is matched to `semantic_search_task`'s
budget, so at least the sim-time axis lines up.

---

## 2. Running one, locally

**Put the run in `.env`. Do not drive compose per-service with env prefixes.**
Compose interpolates `.env` at container-CREATE time, so a var that is not in
`.env` never reaches the container — and `VAR=x airstack up isaac-sim` only
configures the service you typed it in front of, which is exactly how the sim
and the planner end up disagreeing about camera pitch or the map anchor.

```bash
cd <repo>
$EDITOR .env            # scene, spawn, method, model — see the CoNavGPT2 block
./airstack.sh down
./airstack.sh up isaac-sim robot-desktop gcs
```

`AUTOLAUNCH=true` (the default) means that one command starts Isaac with
`ISAAC_SIM_SCRIPT_NAME`, the robot autonomy stack, and the GCS with Foxglove.
Nothing else is needed to get a flying drone.

**THE MODELS DO NOT LIVE IN THE ROBOT CONTAINERS.** They run in one container
of their own, `offboard-compute`, and every robot reaches them over plain HTTP
on the docker bridge. That is not tidiness: at `NUM_ROBOTS=8` a model loaded
in-process is loaded eight times on one card, and the detector — which EVERY arm
runs — is the worst offender, because YOLO and MobileSAM both come with it. All
three services are stateless (one frame in, an answer out, no per-robot
history), so interleaving robots cannot produce cross-talk. Frames travel in the
request body, so none of this is DDS traffic and none of it touches
`dds_router.yaml`.

```bash
# 1. THE SHARED MODELS. Start ONCE, before any planner, and leave it up. The
#    detector serves every arm; the other two are opt-in per run.
#    (Runs the robot image; loads weights BEFORE binding the port, so a port
#     that answers means ready.)
./airstack.sh up offboard-compute                          # detector :8200
START_ITM_SERVER=true ./airstack.sh up offboard-compute    # + vlfm's scorer :8100
START_VLM_SERVER=true ./airstack.sh up offboard-compute    # + conavgpt2's VLM :8000

until docker exec offboard-compute curl -sf localhost:8200/health | grep -q ok; do sleep 5; done
docker exec offboard-compute tail -f /tmp/offboard/detector_server.log
docker exec offboard-compute curl -s localhost:8200/metrics   # calls, mean_ms, vocab_switches
```

`detector_mode: 'server'` is set in every method config and `detector_url`
defaults to `http://offboard-compute:8200`; the planner PREFLIGHTS it and dies
with an actionable message if it is not there. That is deliberate — there is no
local fallback, because a detector that silently answers "nothing here" is
indistinguishable from a search that found nobody. `detector_mode: 'local'`
restores the old one-model-per-planner behaviour for a bench with no shared host.

Then the two genuinely per-run steps, in the ROBOT container:

```bash
R=disaster-dataset-robot-desktop-1
CFG='$(ros2 pkg prefix search_baselines)/share/search_baselines/config'

# 2. takeoff
docker exec $R bash -lc 'ros2 action send_goal /robot_1/tasks/takeoff \
  task_msgs/action/TakeoffTask "{target_altitude_m: 12.0, velocity_m_s: 2.0}"'

# 3. the planner — the METHOD is the launch file, the SCENE is the argument
docker exec $R bash -c "tmux new-window -t bringup -n search; tmux send-keys -t bringup:search \
  'sws && ros2 launch search_baselines vlfm.launch.xml \
     scene_params_file:=$CFG/modular_house.yaml \
   2>&1 | tee /tmp/conavgpt2/live.log' ENTER"
```

Swap `vlfm.launch.xml` for `frontier.launch.xml` or `lawnmower.launch.xml`
(or `conavgpt2_team.launch.xml` for the team-mode arm) and change NOTHING else
— same scene file, same map, same bounds, same actuation. That is the whole
point of the shared node, and it is also the cheapest way to get a control
arm: `frontier` and `lawnmower` need no VLM at all — only the shared detector,
which every arm needs — so they can be flown while a big model is still
downloading.

**Restart the ROBOT stack after every sim restart.** Sim time resets to 0 and the
robot keeps stale TF and PX4 state; the symptom is `TakeoffTask rejected: state
estimate timed out` with MAVROS looking perfectly healthy.

Watch it:

```bash
docker exec $R bash -c 'tail -f /tmp/conavgpt2/live.log' \
  | grep --line-buffered -E "round [0-9]+ \||sim budget|Traceback"
docker exec $R python3 /tmp/show_rounds.py     # round table WITH the VLM's reasoning
docker exec offboard-compute curl -s localhost:8100/metrics  # vlfm scorer: calls, mean_ms
docker exec offboard-compute curl -s localhost:8200/metrics  # detector: calls, mean_ms, vocab_switches
```

### The three config layers

Applied in this order, all under the top-level YAML key `search_planner:`
(it must match the node name, and the node name is now `search_planner` for
every method):

| layer | file | owns |
|---|---|---|
| base | `config/planner.yaml` | every parameter and its default — topics, frames, cadence, actuation |
| method | `config/vlfm.yaml`, `config/conavgpt2.yaml` (`paper.yaml` is its 2D-faithful variant), `config/frontier.yaml`, `config/nearest.yaml` | `nav_mode`, the scorer, the frontier source. Nothing scene-specific |
| scene | `config/modular_house.yaml`, `config/suburb_mini.yaml`, `config/suburb_wildfire.yaml` | extent, obstacle band, search polygon, altitude bands — and it wins |

The scene layer goes LAST deliberately. `conavgpt2.yaml` (and `paper.yaml`)
restore upstream's own numbers (assignment every 25 agent steps), which describe
a 24 m indoor grid seen by a quadruped; only the scene knows it is a 1700 m plat
(§5). **`sem_threshold` is the exception — it is no longer layered at all**: every
layer pins the same 0.65, so the detector gate cannot differ by arm or by scene
(§6). `conavgpt2.yaml` ALSO sets
`frontier_source: voxel3d`, identical to `vlfm.yaml` / `frontier.yaml`, so the
three arms are offered one candidate list; `paper.yaml` sets no source and
inherits `slab2d` — until 2026-08-26 it was the launch default, so every
conavgpt2 run before then flew the 2D slab while vlfm flew the voxel map. Keep anything the scene must own OUT of the launch args: an individual
`<param name>` BEATS a `<param from>` file in launch_xml whatever order they
appear in, so an arg default silently overrides a whole overlay.

### What it publishes

Method-neutral and robot-namespaced, because one node now serves three methods
and more than one robot:

| topic | what |
|---|---|
| `/{robot}/frontiers` | `MarkerArray` — the candidates the selector was offered |
| `/{robot}/occupancy` | `OccupancyGrid` — the shared RGBD map |
| `/{robot}/value_map` | `OccupancyGrid` on the SAME geometry — VLFM's field, `-1` where nothing has been scored (§3) |
| `/{robot}/voxel_map` | `PointCloud2` — OCCUPIED voxels only, `frontier_source: voxel3d` only (§4). Free voxels are in the grid but NOT published: drawing them renders open air as solid and the map reads as though everything were an obstacle |
| `/{robot}/frontier_cloud` | `PointCloud2` — the same candidates as a CLOUD. A MarkerArray fixes point size at publish time; a cloud lets the viewer scale it, which is the difference between frontiers that read as dots over a 278 m plat and frontiers that swallow it. Green = the committed goal, orange = candidate |
| `/{robot}/search/markers` | `MarkerArray` in the MAP frame — robot, flight trail, and the target lifecycle. Namespaces: `search_detection` (magenta cubes, the RAW detector cloud before clustering — the layer that answers "has it seen anything at all"), `search_detection_id`, `search_target` (RED found / YELLOW visiting / GREEN visited, drawn at the geofence radius so the green disc IS the region that suppresses a revisit), `search_target_id`, `search_robot`, `search_trail`, `search_area` (the sector outline, red for robot_1). **`lawnmower` only:** `search_lanes` (the WHOLE planned boustrophedon path, dim — the only view of ground the sweep has not reached yet), `search_lanes_done` (the same lanes solid up to the current goal, reset every lap), `search_lane_transit` (white, the one-time walk from takeoff to the entry lane end — overhead, not coverage), `search_lane_goal` (amber sphere at `lawnmower_reach_radius_m`, the leg being flown now), `search_lanes_id` (phase / index / lap / lane spacing). A new NAMESPACE needs no GCS or layout change — `foxglove_visualizer_node` translates and republishes the whole MarkerArray, and `render_layout.py` toggles this topic at topic level, not per namespace |

**All six are in the ROBOT'S `map`, anchored at its TAKEOFF POINT — the frame
its odometry, `/global_plan` and `trajectory_vis` are in.** The GCS's `map` is
global ENU (sim ground, GT boxes, `/gcs/robot_markers`). Drawn raw on the GCS
they sit at the WORLD ORIGIN, a spawn-offset away from the drone (100 m on
the 250 m suburb at spawn (72, 74); 340 m on the wildfire plat) — while the
drone mesh, trail and global plan look right, because
`foxglove_visualizer_node` translates THOSE by the robot's map origin before
drawing. It now does the same for these six and republishes them under
`/gcs/{robot}/occupancy`, `/gcs/{robot}/frontiers`, `/gcs/{robot}/frontier_cloud`,
`/gcs/{robot}/voxel_map`, `/gcs/{robot}/value_map`, `/gcs/{robot}/search/markers`;
the rendered layout points at those. **On the GCS, always view the `/gcs/`
names; the raw `/robot_N/` names are only right on the robot-domain
Foxglove (`:8775`).** The offset is measured as `enu(GPS fix) - odometry`
(averaged over 10 pairs, then locked), the same way `map_anchor_node` does,
so a GCS started mid-flight agrees with one started before takeoff. It is a
VISUAL bug only: the planner, droan and the drone all live in the robot's
`map` and agree with each other — the `frame check:` log line (odom vs
`grid_to_map`) reads `offset=(+0.0, +0.0)` throughout. |
| `/{robot}/search/map_image` | upstream's own top-down render |
| `/{robot}/search/vlm_prompt_image` | exactly what the generative VLM was shown |
| `/{robot}/search/round_stats` | per-round telemetry (also JSONL under the results dir) |
| `/{robot}/search/run_complete` | latched `Bool` at `max_sim_seconds` of SIM time |
| `/{robot}/search/agent_image` | the FPV frame the map was built from |

Nothing publishes under `/conavgpt2/...` any more. If you still see those names
on the wire, a config layer is pinning them over the node defaults — check the
`*_topic` block in `planner.yaml` first.

---

## 3. VLFM — the BLIP-2 ITM one

`ros2 launch search_baselines vlfm.launch.xml`. The selection chain, end to end:

1. every `vlfm_keyframe_period_s` (0.2 s shipped) the live RGB is scored against
   the target text by **BLIP-2 ITM** — one forward pass returning a calibrated
   `P(match)` from the match/no-match head, not a generative model asked for a
   number it then has to be parsed out of;
2. the score is painted into a **VALUE MAP** over the camera's FOV cone out to
   `vlfm_value_range_m`, weighted `cos^2((theta / (fov/2)) * pi/2)` so a glancing
   look at the edge of frame updates the map less than a square-on one;
3. fusion is **confidence-weighted, not a running average**:
   `v = (c_new*v_new + c_old*v_old)/(c_new+c_old)` and
   `c = (c_new^2 + c_old^2)/(c_new+c_old)`. The superlinear confidence term is
   the point — repeated agreeing looks HARDEN the estimate, which a mean never
   does, and two tentative looks never add up to a confident belief;
4. the frontier standing in the highest-value cell wins, minus
   `vlfm_distance_penalty` per metre of travel. At 0 it is pure argmax and will
   cross the whole map for a marginally better cell.

Before anything has been scored the field is all zeros, and argmax over that is
an arbitrary choice dressed up as a decision — so the node falls back to the
NEAREST in-bounds frontier and logs that it did. A failed scoring call returns
None, and None is not zero: zero is the claim "this view is unpromising", a
failed call is the absence of a claim, and the value map is not told otherwise.

### The scorer is a SHARED service

`Salesforce/blip2-itm-vit-g` on port 8100 of the **offboard-compute** container
(`START_ITM_SERVER=true ./airstack.sh up offboard-compute`; by hand,
`ros2 run search_baselines itm_server`). One instance serves every robot and
every method that wants image-text matching — as does the detector on :8200
beside it, so a VLFM robot makes two HTTP calls per tick to that one host. **That is safe because VLFM scores single frames with no history**:
there is no per-robot state on the scorer, so requests from different robots
interleave with no cross-talk. It is NOT true of the generative endpoint
conavgpt2 uses, where a shared server still serialises on one GPU lock for
seconds at a time.

Measured on this box (one RTX 5070 Ti), same image, same target:

| scorer | per call | rate | VRAM |
|---|---|---|---|
| generative Qwen2.5-VL-7B nf4, asked for a score | ~2500 ms | 0.4 Hz | 5.93 GiB |
| **BLIP-2 ITM** | **17.4 ms** | **57 Hz** | **2.50 GiB** |
| BLIP-2 ITM, batch 8 | 8.0 ms/image | ~124 img/s | — |
| end to end through the client (encode + HTTP + infer) | 21.3 ms | — | — |

**143x faster and 2.4x smaller**, and the server sustains ~55 calls/s in
aggregate regardless of client count. That is what makes one scorer enough for a
fleet instead of one per robot — and it is why the choice of scorer, not the
choice of frontier, is what decides how many robots fit on a card.

### Whole-frame ITM is ANTI-CORRELATED for small aerial targets

The finding that changed the shipped default, and the reason canonical VLFM
cannot simply be pointed at a drone. BLIP-2 resizes whatever it is given to
224x224, so a person **13 px wide in a 946 px frame becomes ~3 px** before the
model ever sees them. Measured on a real fire-scene frame containing ~8 people:

| what was scored | P(match) |
|---|---|
| whole frame | **0.090** <- BELOW a flat grey image |
| 2x3 tiles, max | 0.423 |
| 3x4 tiles, max | 0.435 |
| flat grey image, 3x4 max | 0.129 <- tiling does NOT inflate a blank |

An empty view outscores a populated one. That is not a weak signal, it is the
WRONG SIGN, and a value map built on it steers away from people. The grey-image
row is the control that matters: tiling is not simply inflating every score.

So the shipped default is `vlfm_tiles: [2, 3]` — six tiles scored in one batch,
**ROWS collapsed by MAX** (a person low in the frame is evidence about that
bearing whatever the sky above them looks like; a mean would dilute it with the
empty tile), and each **COLUMN painted into its own bearing slice** of the cone,
so a target off to one side raises the value in that direction instead of
smearing across the whole FOV. `vlfm_tiles: [1, 1]` restores canonical
whole-frame VLFM for a fidelity run — and should be reported as a separate arm,
not as a bug fix.

Cost: **69 ms per keyframe** at 2x3, batched. That is the fleet budget:

| keyframe rate | duty per robot | robots off one scorer |
|---|---|---|
| 5 Hz (0.2 s) | ~35% | ~3 |
| 2 Hz (0.5 s) | ~14% | ~7 |
| 1 Hz (1.0 s) | ~7% | ~14 |

0.2 s is right for a one-robot bench. **Raise it before adding robots**, and
check `/metrics` rather than assuming.

### Frontier commitment — ours, not the paper's

Argmax every tick makes the drone thrash: the value map shifts as it flies, two
frontiers keep trading places, and it arrives at neither. So a pick is HELD:

| knob | shipped | meaning |
|---|---|---|
| `frontier_lock_s` | 6.0 | minimum hold before a swap is even considered |
| `frontier_swap_margin_frac` | 0.35 | a rival must beat the held score by 35% of it |
| `frontier_unlock_radius_m` | 4.0 | arriving within this releases the lock. Must be >= `goal_tolerance_m` (2.5) or the lock lets go before droan considers the goal reached |

**This is NOT from the VLFM paper.** It is modelled on raven_nav's
`frontier_behavior`. Say so plainly in any write-up: it is a deviation, and the
kind that changes a trajectory plot.

**THE ARRIVAL TEST IS ON THE LOCKED GOAL, NOT THE CHALLENGER.** The release
used to read

```python
if dist(here, cand_xy) <= unlock_radius and list(cand) == list(self._locked_goal):
```

— it required the CHALLENGER to be the same grid cell as the locked goal. The
candidate set is re-extracted every tick, so that cell is rarely in it again,
the condition never fired, and the drone flew to its frontier and PARKED there
for the rest of the run. It reads exactly like a stuck local planner and is
not one: droan is steering fine, nothing is re-issuing a goal. If a run ends
with the drone hovering over a frontier it already reached, check
`grep "reached goal"` in the planner log first — no lines means this.

---

## 4. 3D frontiers (`frontier_source: voxel3d`)

Orthogonal to `nav_mode`. It is a parameter of the SHARED planner, and every
method overlay that is launched by name now sets it (`conavgpt2.yaml`,
`vlfm.yaml`, `frontier.yaml`) — `slab2d` (planner.yaml's default, what
`paper.yaml` inherits) is upstream's behaviour, `voxel3d` is the 3D one, and
either can be paired with any method. For the gpt arm the candidate images are
cut from the edge map BY LABEL, so `_voxel_frontiers` offers one candidate per
grid cell (z-duplicates collapsed to the better-ranked one) and stamps only
free pixels; see `search_baselines/README.md` and
`tests/test_conavgpt2_baseline.py`.

**Why.** Upstream's `Map_Extraction` collapses the merged cloud into ONE
altitude slab, so every frontier it produces is at the same height BY
CONSTRUCTION. Right for a quadruped in a house; wrong for a drone, which can go
over, under and between things.

**What voxel3d is.** rayfronts' definition, reproduced in numpy
(`mapping/frontier_vdb_map.py`): a THREE-STATE voxel map — empty / occupied /
unobserved — built by RAY CARVING, in which a frontier is an **EMPTY voxel with
at least `min_unobserved` unobserved neighbours in a (2r+1)^3 box**.

The three states are the part that cannot be skipped. "Not occupied" is not
"empty": a voxel is empty only once something has actually looked THROUGH it,
and that distinction IS the frontier definition. Carving walks from the camera
to each depth return marking free, then marks the return itself occupied —
occupied written LAST, so a wall is not dissolved by the rays of its neighbours.
Everything untouched stays unobserved.

**Verified standalone** — unit-tested, not flown (§10):

| check | result |
|---|---|
| frontiers extracted | 1546, across **12 distinct heights**, 0.5-11.5 m |
| carving | in front of a wall EMPTY, behind it UNOBSERVED, the wall itself OCCUPIED |
| z-band filter | keeps only the requested band |
| extraction cost | **2 ms** on 108k voxels |

Each frontier carries its own z, and that z becomes the waypoint altitude
(clamped into the flight band), so fixed-height flying goes away.

### Frontiers ACCUMULATE — rayfronts' scheme, not recompute-every-tick

The parameters are rayfronts' own (`frontier_vdb_map` defaults):
`neighborhood_r=1`, `min_unobserved=4`, `min_empty=2`, `min_occupied=0`,
`frontier_subsampling=4`, `frontier_subsampling_min_cells=10`.

So is the UPDATE RULE, and it is the part that matters. rayfronts keeps a
PERSISTENT global frontier set and re-evaluates only the ACTIVE WINDOW — the
bbox of the voxels the latest observation actually wrote:

```python
outside_mask = any(frontiers < bbox_min) or any(frontiers > bbox_max)
self.frontiers = self.frontiers[outside_mask]        # KEEP everything outside
self.frontiers = cat([self.frontiers, update])       # REPLACE only inside
```

A frontier behind the robot survives until the robot looks there again.
`VoxelMap.frontiers_persistent()` reproduces this; `VoxelMap.frontiers()` is
the old recompute-everything call and is kept only for the standalone tests.

**Why it is not a detail.** Recomputing the whole set every tick gives the
candidates UNSTABLE IDENTITY: the frontier the planner committed to is usually
absent from the next tick's set — it either got observed on arrival or fell out
of the top-N — so no goal can be held, and the commitment logic in §3 has
nothing stable to hold. Recompute-every-tick also produces a SHELL: measured on
a stationary drone, all 12 candidates sat 19.1-22.9 m away (mean 21.1, spread
3.8) in every direction, tracing the sensing horizon rather than unexplored
structure. That shell is *correct* frontier behaviour for a single viewpoint —
it only becomes informative once the observed volume grows, which is exactly
what accumulation gives you.

Verified: 9 frontiers after one observation, 19 after a second 30 m away with
all 9 originals surviving, re-observation REPLACES rather than duplicates, and
18/19 keys persist across ticks.

### Why not `vdb_mapping`, which is already in the stack

It publishes `vdb_map_pointcloud` / `vdb_map_visualization`, and those are
**OCCUPIED voxels only**. `VDBMappingTools` never exposes free space — and
empty-vs-unobserved is exactly what a frontier is defined by, so there is
nothing to extract from it. It would also need the lidar on (~3x the sim cost,
§9.5) and it is 360 degrees, where VLFM's value is bound to the camera FOV.

### The frontier band is NOT the flight band

Two different questions, and conflating them is what produced frontiers at
exactly one height:

| param | question it answers | bench | suburbs |
|---|---|---|---|
| `frontier_z_min_m` / `frontier_z_max_m` | where FRONTIERS may be | 3-20 m | 3-20 m |
| `min_altitude_agl_m` / `max_altitude_agl_m` | where the DRONE may fly | 8-25 m | 15-40 m |

Sizing the voxel map to the FLIGHT band puts the map entirely in empty air above
the geometry: nothing is ever carved against a surface, and whatever frontiers
do form sit at one height — which reads exactly like the 2D slab you were trying
to leave behind. The map has to contain the ground and the buildings; the flight
band still filters where the drone actually goes.

---

## 4b. The 3D lawnmower, sectors per drone, and the GT / obstacle files

**Lane legs clear known obstacles (2026-08-26).** `lawnmower.yaml` reads the
scene's obstacle boxes and flies each leg at `max(cruise, top + 3 m)`, clamped
to the band, with the through pose at the NEXT leg's height and reach judged
in xy only (see search_baselines/README.md). A stall lifts the goal 5 m twice
before skipping. Log lines: `known obstacles: N boxes from ...`, `lawnmower:
goal #k leg flown at Z m — tree top T m under it`, `lifting it to Z m`.

**Two annotation files per scene**, both written by
`example_multi_drone_scene_import.py` with `GT_ANNOTATIONS=on` under
`RESULTS_SCENE`, into gcs_visualizer/annotations and raven_nav/annotations:
`<scene>.json` is PEOPLE ONLY (the GT the GCS draws and the scorer reads) and
`<scene>_obstacles.json` holds houses / trees / cars measured off the composed
stage (`scene_annotations.boxes_from_scopes`) — read only by the planner's
`_load_known_obstacles`. `annotation_viz_node` now reloads the GT when the file
changes, because the GCS comes up before the scene is built and used to show
the previous run's people for the whole run.

**Fleet.** NUM_ROBOTS=4 or 8 + as many `SPAWN_CONFIGS` entries; each robot container
runs its own planner (`num_robots` is the TEAM size, `team_mode: false` makes
the node drive one robot and cut NUM_ROBOTS sectors, taking slice `robot_N`-1).
`sector_partition` in the method overlays is now `rect` — equal-WIDTH bands,
each robot's sector the bounding rectangle of its band's share of the search
polygon, roughly (not exactly) equal in area; `sector_axis: principal` (the
1 km wildfire scene) cuts along the polygon's own major axis and hands out
ROTATED rectangles, which is what hugs a diagonal burn (measured: 245 m along
the burn x 500-620 m across, 123-152k m2 each, vs 960 x 192 m whole-plat strips
before). `strips` (equal area) and `grid` remain. Log:
`sector 2/5 (rect, pad 50 m): 4 pts, 149101 m2`. Targets outside the sector
are ignored. All arms detect on the shared
`offboard-compute` server (§2); the VLM / ITM servers start there with
`START_VLM_SERVER` / `START_ITM_SERVER` in `.env`, and the detector checkpoint
is `CONAVGPT2_YOLO_WORLD_WEIGHTS` (yolov8x for suburb_mini, yolov8l-world for
the wildfire plat — the server's choice wins over the scene's).

**Goals are re-checked against the live voxel map every tick (all arms).**
A frontier is chosen where nothing has been observed; if the goal voxel (±1)
turns OCCUPIED as the drone approaches, gpt/vlfm/frontier/nearest BLACKLIST it
and re-pick that tick, the lawnmower lifts then skips the leg. A PERSON whose
approach point is inside occupied voxels is given up (marked visited, log
`GIVING UP`) — a detection the drone can never reach is treated as a false
positive rather than parked on.

**The voxel map forgets.** `voxel_forget_after_s` (60 s) returns voxels not
re-observed within the window to unobserved and drops the frontiers in them:
the map is for navigation and frontier picking, not a survey of the scene.

**The CoNavGPT round no longer blocks the tick.** The VLM call runs on a
worker thread; the tick keeps commanding the current goal (or the nearest
candidate) and applies the answer on the tick it arrives, against the list the
VLM was shown. The vendored parser is lenient (a reply cut off mid-"reason"
still yields `robot_0`), retries are 2 not 5, `max_tokens` 160. Measured
before: 12/13 rounds failed at 26 s x 5 retries with `/global_plan` silent —
that was the "stuck" drone. The VLM server accepts up to 12 images per request
(`--max-images`), matching `max_frontiers_limit`.

**Speed by intent.** droan_gl's rollout cap is a LIVE parameter
(`max_velocity`; `robot/ros_ws/src/local/planners/droan_gl`) — and until
2026-08-27 it did nothing, see §9.u. Every arm except gpt sets it from
`_command`, carried on the NavigateTask goal (`max_speed_mps`, §9.u):
`transit_speed_mps` (7.0) while OUTSIDE its sector, `explore_speed_mps` (1.5)
inside it, `target_speed_mps` (1.5) the moment a detected person is the goal.
Log: `NavigateTask max_speed -> 7.0 m/s (transit to sector)` / `(search)` /
`(target approach)`, then `ground speed ... | cap ...` every 10 sim-s. 0 on
explore/target leaves droan's own `max_velocity`.

**Colours and entry edges in a fleet.** `search/markers` (robot, trail,
sector outline, lanes) use the GCS's own `ROBOT_COLORS[N-1]` through
`fluorescent()`, so robot_2 is the same green on the GCS's mesh and on its
sector. Lawnmower robots with an odd index enter their strip at the corner
FARTHEST from the spawn (`lawnmower: robot index 1 enters at the FAR edge`), so
drones spawned together do not sweep in step.

## 5. The numbers that MUST scale with the scene

This is the part that silently ruins comparisons. `map_cells` is **fixed at 480**
(upstream stamps robot markers on the VLM's candidate images in 480-pixel
coordinates), so `map_extent_m` sets **cell size, not cell count**:

| scene | extent | m/cell |
|---|---|---|
| modular_house bench | 240 m | 0.50 |
| suburb_wildfire plat | 1700 m | 3.54 |

Three parameters are expressed in CELLS and therefore change meaning with it:

| param | unit | bench (0.50 m) | plat (3.54 m) |
|---|---|---|---|
| `frontier_threshold_points` | cells of frontier BOUNDARY | 20 = 10 m | 20 = **71 m** -> use 6 (~21 m) |
| `scene_voxel_m` | metres | 0.25 | 1.0 |
| `max_frontiers` | count | 6 | 12 (auto-derived from extent) |

Verified against `Frontier_Det` with 8 synthetic candidate regions:

```
order=smallest max=6 -> offered=6 dropped=2 areas=[28,29,32,34,37,38]
order=largest  max=6 -> offered=6 dropped=2 areas=[118,38,38,37,34,32]
```

Note what `smallest` drops: the 118-cell region, three times larger than any
other. That is the plat failure mode in one line.

Set `map_extent_m` from the SCENE, measured — not guessed. The generated ground
truth gives it for free:

```python
import json
d = json.load(open('gcs/ros_ws/src/gcs_visualizer/annotations/<Scene>.json'))
xs = [c for b in d for c in (b['bbox_world']['center_xyz_m'][0],)]
```

then take the furthest scene corner from the SPAWN, because the grid is centred
on the map origin (the takeoff point), not on the scene.

**`Frontier_Det` only ever looks at the LARGEST connected free region.**

```python
contour = max(contours, key=cv2.contourArea)   # the biggest free blob, and only it
```

Disconnected explored patches contribute NO frontiers at all. Frontier regions
are then the arcs that region's boundary is cut into by obstacles — which is why
the count is usually small, and why a map that fragments produces fewer
candidates rather than more.

`frontier_order` also inverts with domain: upstream keeps the SMALLEST frontier
regions, which is right indoors (a small frontier is a doorway) and wrong from
the air (a small frontier is a gap between two houses; the big ones are the open
ground nobody has covered). Use `'largest'` on the plats.

---

## 6. Detector thresholds — ONE gate, measured, never inherited

**Reading the gate off a run's log (2026-08-27 on).** The planner logs, per
robot: `detector gate: a "person" box is mapped only when its confidence >
sem_threshold 0.65 ...` at startup; `detector PASS: person 0.712 > gate 0.65
-> mapped into object_pcd | 1 box(es) >= 0.50: 0.71 @ px[312, 201, 330, 240]
18x39 | other classes [...] | run max 0.745 | passes 7/41 proposals` on EVERY
pass; `detector SEEN: person 0.580 (below gate 0.65, not mapped) | 2 box(es)
>= 0.50: ...` on EVERY tick with a person at or above `detector_log_conf`
(0.5, logging only — the gate does not move) — both unthrottled, both with
each box's score and pixel rect, and both ticks also emit the
`detection_image` JPEG; `detector below gate: ... < 0.50` for proposals under
the floor (10 s throttle); and
`detector summary: 312 ticks | "person" proposed on 41 | passed gate 0.65 on 7
| run max 0.745 (tick 188) | conf bins <0.3:20 0.3-0.5:12 0.5-0.65:6
0.65-0.8:7 >=0.8:0` every `detector_log_period_s` (30 s sim) and once more
marked `(FINAL)` when the budget ends. The mission's summary step greps these.
Before this the only score in the log was a 10 s-throttled sample, and the
"max confidence" read off it was a lower bound.

**There are two thresholds in series, and only the second is configurable.**

| | value | where |
|---|---|---|
| raw YOLO NMS floor | `conf=0.1`, HARDCODED | `conavgpt2/vendor/utils/detection_segmentation.py:57` |
| goal-class gate `sem_threshold` | 0.65, every layer | `vendor/agents/ros2_agents.py:195` — `class_id == goal_id and confidence > sem_threshold` |

The 0.1 floor is identical for every arm and is not a ROS parameter; anything
below it never leaves the detector. `sem_threshold` is what sets `found_goal`,
plants `nearest_point` and triggers the descend-and-confirm pass.

**`sem_threshold` is pinned to 0.65 in EVERY layer** — `planner.yaml`,
`conavgpt2.yaml`, `paper.yaml`, `suburb_wildfire.yaml` — deliberately, and
`tests/test_conavgpt2_baseline.py` and `tests/test_frontier_baseline.py` assert
it. The detector is shared by every arm, so a gate that differs by arm makes the
DETECTOR the thing under comparison instead of the search policy: conavgpt2 used
to carry upstream's 0.85 while vlfm ran 0.5, which meant `found_goal` differed
between arms for a reason that was not the selection policy at all. Same
reasoning as the shared map, the shared frontier source and the shared
actuation. `paper.yaml` is faithful to the paper on the 2D map, the cadence and
`num_local_steps`, and deliberately NOT on this.

Upstream's 0.85 is an INDOOR CLOSE-RANGE number and finds nothing at aerial
range. Measured with YOLO-World on this dataset, 2026-08-25:

| scene | what the detector produced |
|---|---|
| modular_house bench, goal `house` | **nothing at all**, at any threshold, even conf 0.05 |
| modular_house bench, other classes | spurious `person` at 0.07-0.32 in a scene with NO people |
| fire scene with ~6-8 real people | `person` 0.743 / 0.648 / 0.571, `car` 0.87, `truck` 0.76 |
| same, at the ZED's 480x300 | `person` 0.701 / 0.601 — one confident detection lost |

So: 0.85 keeps **zero** people. 0.65 keeps the confident ones and sits above
the 0.32 false-positive band measured on the bench — precision over recall,
deliberately, because a false survivor sends the drone across the plat for
nothing. **Run this check on a real frame of your scene before trusting any
threshold** — it is a five-minute test that prevents a whole mission finding
nobody. If a scene genuinely needs a different number, change it in
`planner.yaml` so it moves for EVERY arm at once; a per-scene or per-method
override re-breaks the comparison and trips the parity assertions.

Note the detector MODEL is still per-scene, and matters as much as the number:
`suburb_mini.yaml` runs closed-set COCO `yolov8x.pt` (`person` only, 237 ms),
while `suburb_wildfire.yaml` / `modular_house.yaml` run open-vocab
`yolov8l-world.pt` (1030 ms). `vlfm_min_confidence: 0.02` is NOT a detector
threshold — it is the VLFM value-map floor below which a cell counts as
unscored.

### The goal class must be one the detector can actually see

Measured on the 250 m undamaged suburb, 2026-08-26, from real frames in flight:

| goal class | best confidence ever produced |
|---|---|
| `person` | **0.67**, 0.65, 0.48, 0.35, 0.30 |
| `car` | 0.69, 0.59 |
| `tree` | 0.032 |
| `roof` / `rooftop` / `suburban house` | 0.031 |
| **`house`** | **0.013** |

Swept at conf floor 0.01, at 640 and 1280, and on a 2x upscale: **YOLO-World is
effectively blind to buildings in oblique aerial synthetic renders** while
scoring people in the SAME frames at 0.67. It is trained on ground-level web
imagery; a roof seen at 30 deg from 12 m is nowhere in that distribution.

The failure this produces is silent and expensive. Upstream accumulates
`object_pcd` only when `goal_id == class_id`, so with `goal_name: 'house'` every
one of those 0.67 `person` detections is DISCARDED on the spot — no cloud, no
clustering, no target markers. Watched live it looks like the drone flying
straight to a group of survivors, hovering over them, and leaving: the planner
genuinely never knew they were there.

**So: `goal_name` is not cosmetic, and the log line that tells you is
`goal_max`.** The planner logs, per tick,

```
targets: object_pcd N pts | found_goal B | instances N | visited N |
         detector {'n': N, 'top': [(class, conf), ...], 'goal_hits': N, 'goal_max': X}
```

`goal_max` is the best confidence the GOAL class reached. `n` high with
`goal_max` 0.0 means the detector is working and the goal class is wrong — a
completely different fix from raising or lowering `sem_threshold`. The OSMO
missions write this to `detector_summary.txt` for exactly this reason.

Two structural facts about the detector worth knowing:

* **The class list is competition, not decoration.** YOLO-World assigns each box
  to whichever prompt embeds closest, so distractor classes (`car`, `truck`,
  `house`, `rubble`...) soak up false positives that a bare `['person']` list
  would hand to `person`.
* **There is no verification step.** A detection above threshold becomes a target
  immediately — no CLIP re-check, no multi-view agreement, no VLM confirmation.
  raven_nav does confirmation passes; conavgpt2 does not.

---

## 7. Foxglove

Connect to `ws://localhost:8766` (GCS domain) or `ws://localhost:8775`
(robot_1 domain, carries everything). The GCS shows only what `dds_router.yaml`
bridges.

**A topic with no local subscriber never reaches the GCS.** The DDS router
bridges on discovery, and until something in the robot container subscribes there
is no local reader to discover — the symptom is a topic that appears on the GCS
only while you happen to be running `ros2 topic echo`. That is what
`perception_bringup/scripts/topic_keepalive_node.py` exists for; add new debug
topics to its `TOPICS` list (`absolute=True` for un-namespaced ones).

Layers, and who owns each:

| layer | topic | notes |
|---|---|---|
| sim ground | `/gcs/sim_ground` | needs an overhead camera in the launcher; OPAQUE (`ground_alpha`) so overlays read |
| occupancy | `/gcs/robot_1/occupancy` | 4 classes out of a 3-class message: -1 transparent, 0 white, 100 black, **101 = target** via `invalidColor` |
| frontiers | `/gcs/robot_1/frontiers` | `frontier_marker_style`: `centroids` (default) / `cells` / `both` |
| frontier cloud | `/gcs/robot_1/frontier_cloud` | the same candidates as a cloud, viewer-scaled; green = committed goal |
| search markers | `/gcs/robot_1/search/markers` | robot, trail, raw detections (magenta), targets, search-area outline |
| value map | `/gcs/robot_1/value_map` | `vlfm` only. SAME grid geometry as occupancy, so the two overlay cell-for-cell and you can see which frontier the field is pointing at; `-1` is "never scored" and draws transparent |
| voxel map | `/gcs/robot_1/voxel_map` | the three-state cloud `frontier_source: voxel3d` extracts from; bridged, off by default in the layout (~10^5 cubes) |

The `/gcs/robot_1/...` names are `foxglove_visualizer_node`'s copies of the
robot's `/robot_1/...` topics, shifted by that robot's map origin into global
ENU (§2). **The raw `/robot_1/...` names are ALSO visible on the GCS** (the
router bridges them) and draw at the world origin there — if a layer sits on
the origin while the drone is 100 m away, you have toggled a raw one on.
| ground truth | `/gcs/annotations/bboxes` | generated by the layout generator, per-class namespaces |
| drone | `/tf` | **must be bridged** — see the world->map trap below |

Alpha and colours live in the GENERATED layout (`render_layout.py`), not the
auto-opened one — import `/root/airstack_layout_num_robots_<N>.json`, or set
Alpha per-topic by hand.

---

## 8. Ground truth from the layout generator

A procedural scene knows exactly what it placed, so its GT comes from the
generator rather than hand-authoring: `simulation/isaac-sim/utils/scene_annotations.py`,
enabled with `GT_ANNOTATIONS=on` and named by `RESULTS_SCENE`.

* placements -> world AABBs read off the COMPOSED STAGE (`BBoxCache`), so nothing
  is assumed about asset extents, scale or pivot;
* people -> boxes from the people pass's own `humans.json` records, so the GT and
  the scenario cannot disagree;
* building pieces carry a `group` key and are merged — 8 houses, not 96 walls.

Written to BOTH annotation dirs. `annotation_viz_node` reads `ANNOTATIONS_DIR`,
then the SOURCE tree, then the package share — the share is a build-time copy, so
a GT generated at run time never reaches it without a rebuild.

---

## 9. The traps that cost a run

1. **A stale DDS profile in the isaac-sim IMAGE.** `Dockerfile.isaac-ros` COPYs
   `docker/fastdds.xml` at BUILD time. An image built before that file last
   changed ships the old profile — a UDP-only one cannot match the robot
   containers' TCP locators, and the symptom is camera topics that ADVERTISE and
   never deliver a frame, which reads exactly like a planner that never planned.
   The compose now MOUNTS the file. If cameras are silent, check this first.
2. **Ctrl-C orphans PX4.** MAVROS then connects to the ORPHAN and reports a
   healthy FCU while the new sim hangs on "Waiting for first hearbeat" and
   takeoff is rejected `state estimate timed out`. `pkill -9 -f
   px4_sitl_default/bin/px4`, or `down`+`up` when drones are involved.
3. **`world` -> `map` is published as IDENTITY and that is a lie** whenever the
   drone does not spawn at the world origin — `map` is anchored at the TAKEOFF
   POINT. 45 m out on the bench, **340 m** on the plat. Everything in `map` (TF,
   `/global_plan`, occupancy) then renders that far from the sim ground and the
   GCS markers, both of which are world ENU. `MAP_ANCHOR_ENU=true` swaps in a
   node that MEASURES it. Default off: making it truthful changes the frame every
   recorded mission replays in. **It does NOT fix the GCS picture**: the GCS
   publishes its own identity `world->map` (its `map` IS world ENU) and draws
   robot-local topics by translating them, so on the GCS domain there are two
   publishers of that edge and the display frame must stay `map` (§7). What
   fixes the GCS picture is the `/gcs/robot_N/...` republish (§2).
4. **The camera tilt must be told to BOTH sides.** `ZED_PITCH_DEG` tilts the
   mount AND is `search_planner`'s default `camera_pitch_rad`, because TF walks the URDF
   and the URDF models no pitch. Set it in one place or not at all.
5. **The lidar is not free and mostly not needed.** `search_planner` reads no lidar;
   `vdb_mapping` does. With it on, cameras drop ~53 -> ~17 Hz and `/clock` ~53 ->
   15 Hz — a ~3x wall-clock multiplier on a sim-time-bounded run, i.e. on GPU
   hours, for identical coverage.
6. **Results files append across runs.** `conavgpt2_rounds.jsonl` opens `'a'` and
   `round` restarts at 1 each process, so an iteration retry interleaves with the
   attempt it replaced. Group rows by `run_id`.
7. **Two different things are called VLFM.** `VLFM_BASELINE=true` selects a
   greedy semantic-RAY baseline inside raven_nav; `search_baselines
   vlfm.launch.xml` selects the BLIP-2 ITM value-map method of §3. Different
   package, different scorer, different selection rule — they share a name and
   nothing else. Name the package in every result table, or the comparison is
   unreadable six weeks later.
8. **Anything LAUNCHING `conavgpt2` is pre-refactor.** The node, the launch
   files, the configs and the VLM server moved to `search_baselines`, and the
   configs dropped their prefix on the way (`conavgpt2_paper.yaml` ->
   `paper.yaml`, `conavgpt2_modular_house.yaml` -> `modular_house.yaml`). So
   `ros2 launch conavgpt2 ...`, `python -m conavgpt2.vlm_server` and any
   `conavgpt2_*.yaml` config path are all dead references — in a mission file
   they surface as a failed launch step 20 minutes into a pod. The package name
   is still correct for the vendored library and for the METHOD.
9. **Sizing the voxel map to the FLIGHT band gives frontiers at exactly one
   height.** The map lands in empty air above the geometry, nothing is ever
   carved against a surface, and `voxel3d` degenerates into the 2D slab it was
   meant to replace. That is why the frontier band is a separate parameter (§4).

---

### 9.y The drone takes off and never moves — four separate reasons, measured 2026-08-26

All four happened in ONE session on the 250 m suburb, and each alone parks the
drone at the takeoff point with a healthy-looking planner. Check them in this
order; the signature of each is distinct.

1. **droan_gl has no second view (`min_seen_views`).** droan only flies a
   trajectory point it has SEEN in >= N stored camera views (`droan/traj_debug`:
   everything beyond `seen_radius` = 1 m in `unseen_points`, `free_points`
   only inside 1 m). After a VERTICAL takeoff its 10 stored views are a column
   under the hover point, and with the ZED pitched down the airspace around
   the drone is inside at most one of them — and no new view is stored until
   the drone moves 1 m. Historically hardcoded 2; now the `min_seen_views`
   parameter (`local.launch.xml`, set to 1). `python3 traj_geom.py`-style
   count of free vs unseen points by distance is the test.
2. **The planner's tick is seconds long, and PX4 drops OFFBOARD.**
   `_instance_centroids` ran DBSCAN over the ever-growing `object_pcd`
   (23k points -> 10-17 s per tick, py-spy: every sample in
   `cluster_dbscan`, 500 % CPU). The sim stalls (`simulator_mavlink poll
   timeout`, timesync RTT 650 ms), the setpoint stream gaps past
   `COM_OF_LOSS_T` = 1 s, PX4 logs `Failsafe activated ... Nav state: 2`, and
   NOTHING re-requests OFFBOARD (the MAVROS interface only does that in the
   takeoff sequence). `ros2 topic echo .../mavros/state` showing anything but
   OFFBOARD after takeoff is this. The cloud is now voxel-downsampled before
   clustering (1.0 Hz tick); the failsafe still needs a stack restart.
3. **A planner that died without cancelling latches droan.** droan_gl's
   `task_active_` is a bare bool; a killed planner leaves it set and every
   later activator is `Rejecting NavigateTask goal: task already active`
   (planner side: `droan_gl activator not accepted — retrying`). Cancel-all
   on `/robot_1/tasks/navigate/_action/cancel_goal` (zero goal id) clears it
   without a restart.
4. **An unreachable frontier is held forever.** Both people found and
   visited in 60 s, then the drone parked 16 m from a voxel frontier inside a
   house: droan publishes hover, `droan/stuck` is true, and the lock/swap rule
   never releases (a rival must beat the held score by 35 %). The planner now
   has a stall watchdog (`frontier_stall_s` 20 s without 1.5 m of progress ->
   the goal is BLACKLISTED within `frontier_blacklist_radius_m` 8 m at the
   frontier source and in both pick functions, lock released; log line
   `frontier (x, y) unreachable ... BLACKLISTED`).

Not a reason, though it looks like one: `pid_controller: failed to transform
tracking point` at a few per second is TF lookups losing the race under CPU
load; the takeoff still worked through it.

### 9.z `search_area_xy` is in the ROBOT'S map — say `search_area_frame: world`

The planner's frame is this robot's takeoff-anchored `map`, so a polygon
written as scene coordinates but read as map coordinates is displaced by the
spawn offset — (16.5, 18) m on this bench, 340 m on the plat — and the drone
confines itself to the wrong square. `search_area_frame: 'world'` (scene
overlays) makes the planner subtract its measured map origin
(`enu(fix) - odom`, the `map_anchor_node` derivation) once, on the first tick
with a GPS fix (log: `search area authored in WORLD ... polygon shifted by`).
Size `map_extent_m` to contain the SHIFTED polygon (300 m here, not 280).
Lawnmower lanes are shifted with it.

### 9.v The bag's raw `/robot_N/...` layers are in the ROBOT'S map — record the `/gcs/` copies

Measured on `wildfire1km_5robot_a` (2026-08-27): every `/robot_N/voxel_map`,
`frontier_cloud`, `occupancy`, `value_map`, `frontiers`, `search/markers`,
`odometry` and `global_plan` is stamped `frame_id: map` but lives in THAT
robot's takeoff-anchored map (origin = its spawn, (-30, y_N) there), while
`/gcs/robot_markers`, `/gcs/sim_ground` and `/gcs/annotations/bboxes` are
global ENU under the same `map` name; `/tf` is not bridged (dds_router.yaml)
and records nothing on the GCS scope. A replay draws each robot's voxels and
frontiers 30 m ahead of its mesh along +x — "the drones are behind the
voxels". `foxglove_visualizer_node` republishes every one of those layers
translated under `/gcs/<robot>/<layer>` and latches the origin it used on
`/gcs/<robot>/map_origin` (PointStamped): record THOSE (see the wildfire1km
mission's `record.topics`), keep the raw odometry/global_plan only for
analysis, and add the origin back when you read them
(`world = local + map_origin`).

### 9.u Every drone flew 1 m/s — droan's rollouts converged to a UNIT vector

Same run: p90 ground speed 0.98 m/s on a 330 m open transit with the planner
logging `droan max_velocity -> 3.0 m/s`. `trajectory.cs` drives every rollout
to `vel_desired`, which `gl_interface.cpp` filled with a unit direction and
never scaled; `vel_max` sat in the params buffer unread, so the "2 m/s as
shipped" cap was never real either. Fixed by scaling `vel_desired` by
`vel_max` (init and the live rescale). Speeds above ~3 m/s have NOT been
flown yet: the rollout's collision samples are `vel * dt` (0.2 s) apart and
the trajectory controller's spheres are 1 m, so watch the first fast run.

**The cap rides on the NavigateTask goal.** `task_msgs/NavigateTask.Goal.
max_speed_mps` (0 = droan's `max_velocity`); droan_gl applies it per goal and
ACCEPTS a new goal while one runs — the old one is aborted "Preempted by a
new NavigateTask goal" — so a planner changes gear by re-sending its
activator. No parameter service any more. Three gears in `planner.yaml`:
`transit_speed_mps` 7.0 while OUTSIDE the robot's own sector and not on a
target, `explore_speed_mps` 1.5 inside it, `target_speed_mps` 1.5 on a
person. `semantic_search_task` passes its `max_flight_speed` the same way
(it used to log it and drop it). Four of five drones spent 88-330 s of the
600 s budget reaching their strips at 1 m/s.

**Proving it in the next run's logs** (planner.log / droan pane):
```
[robot_1] NavigateTask max_speed -> 7.0 m/s (transit to sector)
[robot_1] droan_gl activated — steering by /robot_1/global_plan, max_speed 7.0 m/s (speed 7.0 m/s)
NavigateTask accepted: activator (steer by /global_plan), max_speed 7.0 m/s (from the goal) — preempting the previous goal
[droan_gl] rollout speed cap -> 7 m/s
[robot_1] ground speed 6.4 m/s over 10 s sim | cap 7.0 (transit to sector) | OUTSIDE sector
[robot_1] NavigateTask max_speed -> 1.5 m/s (search)
[robot_1] ground speed 1.4 m/s over 10 s sim | cap 1.5 (search) | inside sector
```
A `ground speed` line far under its cap with `OUTSIDE sector` is the
airframe/controller, not the planner; a `task_msgs NavigateTask has no
max_speed_mps — rebuild` warning means the container's msgs predate the field.

### 9.t `search_area_source: scene` — the search area is the scorched ground, not the plat

The launcher (GT_ANNOTATIONS=on) now writes `<RESULTS_SCENE>_region.json`
beside the GT: `burn` (the fire-front ellipse at the scene's `elapsed`, i.e.
exactly the ground `age() >= 0` scorched), `affected` (that ellipse run on
until the survivor staged furthest ahead of the front is inside — on the 1 km
plat most of the map again, because `at_home` and the gridlock queue sit past
the front by design), `region` (the plat) and `meta`. `search_area_scene_key:
burn` + `search_area_pad_m: 50` is the tight search every arm flies on
`suburb_wildfire_1km.yaml`; the launcher prints `people inside k/n` for both
polygons, and survivors outside `burn` are outside the search by design. A
missing file is FATAL for a `scene` source (no silent whole-map fallback);
`search_area_source: config` flies `search_area_xy`. scene_gen/disaster/region.py
is pure python — `python3 -m pytest scene_gen/tests/test_affected_region.py`.

### 9.s Detections you cannot check are detections you cannot trust

Same run: 15 committed `person` targets, every one 78-226 m from the nearest
GT person, all projected 52-92 m out (the `depth_max_m` limit) at ground
height, no rigid transform maps them onto the GT — false positives, not a
frame error, and the 1-robot wildfire runs show the same. There was no image
in the bag to say what YOLO saw. `/{robot}/search/detection_image` (JPEG of
the annotated FPV, published ONLY on a tick whose goal-class score cleared
`sem_threshold`, bridged and recorded) is the evidence to open first.

### 9.w The GCS cloud layers need a BEST_EFFORT reader

The DDS router re-offers `/robot_N/voxel_map` and `/robot_N/frontier_cloud`
on the GCS domain as BEST_EFFORT+VOLATILE (the MarkerArray / OccupancyGrid
topics come through RELIABLE). A RELIABLE subscriber is incompatible and
silently receives nothing — `New publisher discovered ... offering
incompatible QoS` in the gcs_visualizer log, and the voxel map and frontier
cloud simply absent while occupancy and the markers draw. `ros2 topic info -v`
on the GCS shows the writer's QoS; the republisher subscribes best-effort.

### 9.x A missing `setup.cfg` kills every mission at `ros2 launch`

`search_baselines` is `ament_python`. An ament_python package needs

```ini
[develop]
script_dir=$base/lib/search_baselines
[install]
install_scripts=$base/lib/search_baselines
```

Without it colcon installs the console scripts to `install/<pkg>/bin/` instead
of `install/<pkg>/lib/<pkg>/`, `colcon build` reports SUCCESS, and every launch
dies instantly with

```
package 'search_baselines' found at '...', but libexec directory
'.../install/search_baselines/lib/search_baselines' does not exist
```

`build_type: ament_python` in `package.xml` is NOT enough — that was already
correct when this broke. It would have failed all four OSMO missions at step
one. Check with `ls install/search_baselines/lib/search_baselines/`.

---

## 10. What has NOT been flown yet

Everything in §3 and §4 is bench-measured or unit-tested in isolation. **None of
it has been confirmed in a live flight.** Do not report any of it as working,
and do not let a clean startup log stand in for a result:

| capability | how far it has been taken | what is missing |
|---|---|---|
| `frontier_source: voxel3d` | unit-tested standalone (§4) AND flown against sim depth on the 250 m suburb, 2026-08-26: frontiers at 6 distinct heights, the drone reaches them and releases | no completed 600 s run; the persistent-accumulation rewrite (§4) is newer than any long run |
| the VLFM value map driving a run | the scorer, the tiling and the fusion are each measured in isolation (§3); in flight it scores ~380 keyframes with 0 failures at ~90 ms | no full 600 s run where the value map chose the frontiers end to end |
| the frontier band (`frontier_z_min_m` / `frontier_z_max_m`) | set on both scene overlays | never exercised in flight; the failure it fixes was diagnosed, not re-tested afterwards |

The discipline is the one §6 applies to detector thresholds: run it against a
real frame of the real scene and read the number before trusting it.

---

## 11. Submitting to OSMO

The fleet template is `osmo/missions/wildfire1km_5robot_a.yaml` (one planner
per robot container, sectored, `/gcs/` layers recorded) and, for conavgpt2,
`osmo/missions/wildfire1km_5robot_conavgpt2.yaml` (one team-mode planner);
copy them to `_4robot_` / `_8robot_` files per §1.

```bash
airstack osmo:mission osmo/missions/<scene>_8robot_a.yaml \
  --pool <gpu-pool> --branch <branch> \
  --workflow osmo/workflows/airstack-mission-1gpu.yaml
```

The `*_1robot` missions are pre-plan smoke tests — a launch that works, not an
experiment. Check any mission you inherit against §9.8 before submitting; a
pre-refactor line fails at the launch step, 20 minutes in.

Before submitting, check all of:

- [ ] **everything is COMMITTED AND PUSHED** — the pod clones from GitHub; an
      untracked package is a `ModuleNotFoundError` 20 minutes in
- [ ] the mission names `search_baselines` everywhere — launch package, server
      module, and config paths
- [ ] it starts the service THAT method needs: `itm_server` on :8100 for `vlfm`,
      `vlm_server` on :8000 for `conavgpt2`, neither for `frontier` / `lawnmower`
- [ ] the robot image is newer than `Dockerfile.robot`'s last change (`clip` for
      YOLO-World's `set_classes`; the mission self-heals but a rebuild is clean)
- [ ] `sem_threshold` left at the shared 0.65 — or, if the scene truly needs
      another number, changed in `planner.yaml` for every arm at once (§6)
- [ ] `map_extent_m` sized to the scene, and the cell-unit params rescaled (§5)
- [ ] the planner is launched AFTER the `/health` gate — the `conavgpt2` arm's
      VLM preflight is fatal, so a bringup-time launch dies before the server
      exists. `vlfm` preflights the ITM endpoint instead and never contacts the
      generative one at all
- [ ] `vlfm_keyframe_period_s` 0.5 at 4 robots, 1.0 at 8 — one scorer serves ~3
      at the shipped 0.2 (§3); `SPAWN_CONFIGS` has NUM_ROBOTS entries
- [ ] every raven-side baseline flag (`FRONTIER_ONLY_BASELINE`, `VLFM_BASELINE`,
      `LVLM_BASELINE`, `CONAVGPT_BASELINE`) is `false` — none is a baseline (§1)
- [ ] a results-collection step exists — `record:` captures ROS topics only, and
      the round table and server metrics live on the container filesystem
- [ ] `ENABLE_LIDAR` off unless something needs `vdb_mapping` (§9.5) — `voxel3d`
      is NOT a reason to turn it on, it carves from the same camera depth every
      method already consumes (§4)

The 2-GPU split is for the VLM's memory and latency isolation, **not** because
Isaac needs a card to itself — that theory was tested on one 16 GB card and
disproved (Isaac + a 4-bit VLM + YOLO-World co-resident at 12.0/16.3 GB with
cameras at 53 Hz). A `vlfm` mission carries 2.50 GiB of BLIP-2 rather than
5.93 GiB of 7B nf4 and holds the GPU for 17 ms rather than 2.5 s, so the second
card is much less obviously earned on that arm — untried, so size it the same
way until someone measures it.

---

## References

- `COA-docs/conavgpt2_status.md` — what is verified, what is not, and the open items
- `robot/ros_ws/src/global/planners/search_baselines/README.md` — the shared
  planner, the per-method launch files, and what each method needs running
- `robot/ros_ws/src/global/planners/conavgpt2/VENDORED.md` — every deviation from upstream
- `.agents/skills/run-isaac-sim-launcher/` — reading a launcher's output, orphaned PX4
- `.agents/skills/launch-generated-scene-with-drones/` — the generated-scene path
- `.agents/skills/visualize-in-foxglove/` — adding a layer properly
- `osmo/missions/README.md` — mission schema, step types, GPU sizing
