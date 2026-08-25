---
name: benchmark-disaster-dataset
description: Run a search-and-rescue BENCHMARK on the disaster dataset — bring the whole stack up from `.env`, fly a method, and read its results. Covers the method matrix (raven_nav, frontier-only, LVLM, conavgpt_baseline, and the `search_baselines` arms launched by name — vlfm / conavgpt2 / nearest — off one shared planner), its three config layers, the `.env`-driven `airstack down` / `airstack up isaac-sim robot-desktop gcs` procedure, the per-run `docker exec` sequence (ITM or VLM server, takeoff, planner), VLFM's BLIP-2 ITM value map and the whole-frame anti-correlation finding, 3D voxel frontiers, what has to SCALE when the scene size changes, the Foxglove layers, and the traps that cost a run — stale DDS profiles, orphaned PX4, identity world->map, a detector threshold that finds nobody, two different things called VLFM, and topics that never reach the GCS. Read before running or tuning any benchmark, and before submitting one to OSMO.
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

## 1. The method matrix

Two families, and they are launched differently.

**raven_nav-hosted arms.** Chosen by env vars in the mission `env:` (or `.env`
locally). They are mutually exclusive and resolved in a fixed precedence:
`FRONTIER_ONLY_BASELINE` > `VLFM_BASELINE` > `CONAVGPT_BASELINE`.

| method | env | what it is |
|---|---|---|
| raven_nav | (none set) | the house method: rayfronts semantic voxels + ray/voxel behaviours |
| frontier-only | `FRONTIER_ONLY_BASELINE=true` | geometric frontier exploration, no semantics |
| VLFM-in-raven | `VLFM_BASELINE=true` | greedy semantic-ray baseline in raven_nav. **NOT canonical BLIP-2 VLFM** — the ITM one is the `search_baselines` arm below, and they share nothing but the name |
| LVLM | `LVLM_BASELINE=true` | LVLM waypoint baseline |
| conavgpt_baseline | `CONAVGPT_BASELINE=true` | HYBRID: raven still navigates; an InternVL3-2B assigner hands it a region |

**`search_baselines` arms.** No flag anywhere: one launch file per method, so a
run is named after the METHOD.

| method | launch | how a frontier is chosen | needs running |
|---|---|---|---|
| **conavgpt2** | `conavgpt2.launch.xml` | a generative VLM picks from numbered top-down BEVs | `vlm_server` on :8000, ~5.9 GiB at 7B nf4 |
| **vlfm** | `vlfm.launch.xml` | BLIP-2 ITM scores the live RGB into a VALUE MAP; highest-value cell wins, minus a distance penalty (§3) | `itm_server` on :8100, ~2.5 GiB |
| **nearest** | `nearest.launch.xml` | closest in-bounds frontier, no model at all | nothing, no GPU service |

All three are ONE node — `search_planner`, with `nav_mode` as a parameter — and
that is the experiment rather than a convenience. They build the SAME map from
the SAME frames, apply the SAME search polygon and altitude bounds, and steer
through the SAME droan actuation, so **a difference in results is attributable
to the selection policy and to nothing else**. Two separate implementations
would each have to be argued equivalent before any comparison meant anything.

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

Three things are genuinely per-run and stay as `docker exec`. **Start only the
service your method needs** — `vlfm` never contacts the generative endpoint and
does not preflight it, and `nearest` needs neither:

```bash
R=disaster-dataset-robot-desktop-1
CFG='$(ros2 pkg prefix search_baselines)/share/search_baselines/config'

# 1a. the SCORER — vlfm only. One instance serves the whole fleet (§3).
docker exec -d $R bash -lc 'sws && ros2 run search_baselines itm_server'
until docker exec $R bash -lc 'curl -sf localhost:8100/health | grep -q ok'; do sleep 5; done

# 1b. the generative VLM — conavgpt2 only, in the robot container -> hence localhost
docker exec -d $R bash -lc 'mkdir -p /tmp/conavgpt2 && nohup /opt/lvlm-venv/bin/python \
  -m search_baselines.vlm_server --port 8000 --device cuda:0 --model "$CONAVGPT2_VLM_MODEL" \
  --quantization "${CONAVGPT2_VLM_QUANT:-nf4}" --compute-dtype bfloat16 \
  --metrics-jsonl /tmp/conavgpt2/vlm_requests.jsonl > /tmp/conavgpt2/vlm_server.log 2>&1'
until docker exec $R bash -lc 'curl -sf localhost:8000/health | grep -q ok'; do sleep 10; done

# 2. takeoff
docker exec $R bash -lc 'ros2 action send_goal /robot_1/tasks/takeoff \
  task_msgs/action/TakeoffTask "{target_altitude_m: 12.0, velocity_m_s: 2.0}"'

# 3. the planner — the METHOD is the launch file, the SCENE is the argument
docker exec $R bash -c "tmux new-window -t bringup -n search; tmux send-keys -t bringup:search \
  'sws && ros2 launch search_baselines vlfm.launch.xml \
     scene_params_file:=$CFG/modular_house.yaml \
   2>&1 | tee /tmp/conavgpt2/live.log' ENTER"
```

Swap `vlfm.launch.xml` for `conavgpt2.launch.xml` or `nearest.launch.xml` and
change NOTHING else — same scene file, same map, same bounds, same actuation.
That is the whole point of the shared node, and it is also the cheapest way to
get a control arm: `nearest` needs no GPU service and can be flown while a model
is still downloading.

**Restart the ROBOT stack after every sim restart.** Sim time resets to 0 and the
robot keeps stale TF and PX4 state; the symptom is `TakeoffTask rejected: state
estimate timed out` with MAVROS looking perfectly healthy.

Watch it:

```bash
docker exec $R bash -c 'tail -f /tmp/conavgpt2/live.log' \
  | grep --line-buffered -E "round [0-9]+ \||sim budget|Traceback"
docker exec $R python3 /tmp/show_rounds.py     # round table WITH the VLM's reasoning
docker exec $R curl -s localhost:8100/metrics  # vlfm: calls, mean_ms, throughput_hz
```

### The three config layers

Applied in this order, all under the top-level YAML key `search_planner:`
(it must match the node name, and the node name is now `search_planner` for
every method):

| layer | file | owns |
|---|---|---|
| base | `config/planner.yaml` | every parameter and its default — topics, frames, cadence, actuation |
| method | `config/vlfm.yaml`, `config/paper.yaml`, `config/nearest.yaml` | `nav_mode`, the scorer, the frontier source. Nothing scene-specific |
| scene | `config/modular_house.yaml`, `config/suburb_wildfire.yaml` | extent, obstacle band, search polygon, altitude bands, `sem_threshold` — and it wins |

The scene layer goes LAST deliberately. `paper.yaml` restores upstream's own
numbers (assignment every 25 agent steps, `sem_threshold: 0.85`), which describe
a 24 m indoor grid seen by a quadruped; only the scene knows it is a 1700 m plat
(§5, §6). Keep anything the scene must own OUT of the launch args: an individual
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
| `/{robot}/voxel_map` | `PointCloud2` — the three-state voxel map, `frontier_source: voxel3d` only (§4) |
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

`ros2 run search_baselines itm_server` — `Salesforce/blip2-itm-vit-g` on port
8100. One instance serves every robot and every method that wants image-text
matching. **That is safe because VLFM scores single frames with no history**:
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
| `frontier_unlock_radius_m` | 8.0 | arriving within this releases the lock |

**This is NOT from the VLFM paper.** It is modelled on raven_nav's
`frontier_behavior`, and the reference implementation was not available here to
check against. Say so plainly in any write-up: it is a deviation, and it is the
kind of deviation that changes a trajectory plot.

---

## 4. 3D frontiers (`frontier_source: voxel3d`)

Orthogonal to `nav_mode`. It is a parameter of the SHARED planner, so CoNavGPT2
can use it too — `slab2d` (the default) is upstream's behaviour, `voxel3d` is
the 3D one, and either can be paired with any method.

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

## 6. Detector thresholds — measure, never inherit

`sem_threshold` gates every target. Upstream's 0.85 is an INDOOR CLOSE-RANGE
number and finds nothing at aerial range. Measured with YOLO-World on this
dataset, 2026-08-25:

| scene | what the detector produced |
|---|---|
| modular_house bench, goal `house` | **nothing at all**, at any threshold, even conf 0.05 |
| modular_house bench, other classes | spurious `person` at 0.07-0.32 in a scene with NO people |
| fire scene with ~6-8 real people | `person` 0.743 / 0.648 / 0.571, `car` 0.87, `truck` 0.76 |
| same, at the ZED's 480x300 | `person` 0.701 / 0.601 — one confident detection lost |

So: 0.85 keeps **zero** people. 0.6 keeps the confident ones and sits above the
0.32 false-positive band measured on the bench. **Run this check on a real frame
of your scene before trusting any threshold** — it is a five-minute test that
prevents a whole mission finding nobody.

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
| occupancy | `/robot_1/occupancy` | 4 classes out of a 3-class message: -1 transparent, 0 white, 100 black, **101 = target** via `invalidColor` |
| frontiers | `/robot_1/frontiers` | `frontier_marker_style`: `centroids` (default) / `cells` / `both` |
| value map | `/robot_1/value_map` | `vlfm` only. SAME grid geometry as occupancy, so the two overlay cell-for-cell and you can see which frontier the field is pointing at; `-1` is "never scored" and draws transparent |
| voxel map | `/robot_1/voxel_map` | the three-state cloud `frontier_source: voxel3d` extracts from; robot domain only, not bridged |
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
   recorded mission replays in.
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

## 10. What has NOT been flown yet

Everything in §3 and §4 is bench-measured or unit-tested in isolation. **None of
it has been confirmed in a live flight.** Do not report any of it as working,
and do not let a clean startup log stand in for a result:

| capability | how far it has been taken | what is missing |
|---|---|---|
| `frontier_source: voxel3d` | unit-tested standalone (§4): carving, the neighbour predicate, the z-band filter, 2 ms extraction on 108k voxels | never run against sim depth in flight; no frontier from it has ever been flown to |
| the VLFM value map driving a run | the scorer, the tiling and the fusion are each measured in isolation (§3) | no full run where the value map chose the frontiers end to end |
| the frontier band (`frontier_z_min_m` / `frontier_z_max_m`) | set on both scene overlays | never exercised in flight; the failure it fixes was diagnosed, not re-tested afterwards |

The discipline is the one §6 applies to detector thresholds: run it against a
real frame of the real scene and read the number before trusting it.

---

## 11. Submitting to OSMO

`osmo/missions/conavgpt2_wildfire_1robot.yaml` + `osmo/workflows/airstack-mission-2gpu.yaml`.

```bash
airstack osmo:mission osmo/missions/conavgpt2_wildfire_1robot.yaml \
  --pool <gpu-pool> --branch <branch> \
  --workflow osmo/workflows/airstack-mission-2gpu.yaml
```

`conavgpt2_wildfire_1robot.yaml` has been migrated to the refactor: it starts
`search_baselines.vlm_server` and launches `search_baselines
conavgpt2.launch.xml` with `scene_params_file:=$CFG/suburb_wildfire.yaml`,
taking `paper.yaml` from the launch file's own default. **There is no `vlfm`
mission yet** — writing one means swapping the launch file, starting
`itm_server` on :8100 instead of the generative server, and gating on
`localhost:8100/health`. Check any mission you inherit against §9.8 before
submitting; a pre-refactor line fails at the launch step, 20 minutes in.

Before submitting, check all of:

- [ ] **everything is COMMITTED AND PUSHED** — the pod clones from GitHub; an
      untracked package is a `ModuleNotFoundError` 20 minutes in
- [ ] the mission names `search_baselines` everywhere — launch package, server
      module, and config paths
- [ ] it starts the service THAT method needs: `itm_server` on :8100 for `vlfm`,
      `vlm_server` on :8000 for `conavgpt2`, neither for `nearest`
- [ ] the robot image is newer than `Dockerfile.robot`'s last change (`clip` for
      YOLO-World's `set_classes`; the mission self-heals but a rebuild is clean)
- [ ] `sem_threshold` measured on a real frame of THAT scene (§6)
- [ ] `map_extent_m` sized to the scene, and the cell-unit params rescaled (§5)
- [ ] the planner is launched AFTER the `/health` gate — the `conavgpt2` arm's
      VLM preflight is fatal, so a bringup-time launch dies before the server
      exists. `vlfm` preflights the ITM endpoint instead and never contacts the
      generative one at all
- [ ] `vlfm_keyframe_period_s` raised if more than ~3 robots share one scorer (§3)
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
