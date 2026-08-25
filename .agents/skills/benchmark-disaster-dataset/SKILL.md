---
name: benchmark-disaster-dataset
description: Run a search-and-rescue BENCHMARK on the disaster dataset — bring the whole stack up from `.env`, fly a method, and read its results. Covers the method matrix (raven_nav, VLFM, frontier-only, LVLM, conavgpt_baseline, conavgpt2), the `.env`-driven `airstack down` / `airstack up isaac-sim robot-desktop gcs` procedure, the per-run `docker exec` sequence (VLM server, takeoff, planner), what has to SCALE when the scene size changes, the Foxglove layers, and the traps that cost a run — stale DDS profiles, orphaned PX4, identity world->map, a detector threshold that finds nobody, and topics that never reach the GCS. Read before running or tuning any benchmark, and before submitting one to OSMO.
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

Which planner runs is chosen by env vars in the mission `env:` (or `.env`
locally). They are mutually exclusive and resolved in a fixed precedence:
`FRONTIER_ONLY_BASELINE` > `VLFM_BASELINE` > `CONAVGPT_BASELINE`.

| method | env | what it is |
|---|---|---|
| raven_nav | (none set) | the house method: rayfronts semantic voxels + ray/voxel behaviours |
| frontier-only | `FRONTIER_ONLY_BASELINE=true` | geometric frontier exploration, no semantics |
| VLFM | `VLFM_BASELINE=true` | greedy semantic-ray baseline in raven_nav. **NOT canonical BLIP-2 VLFM** |
| LVLM | `LVLM_BASELINE=true` | LVLM waypoint baseline |
| conavgpt_baseline | `CONAVGPT_BASELINE=true` | HYBRID: raven still navigates; an InternVL3-2B assigner hands it a region |
| **conavgpt2** | launched directly, no flag | **upstream Co-NavGPT2 end to end** — own RGBD map, own frontiers, own actuation |

`conavgpt2` is the odd one out and it matters for comparisons: every other method
runs inside `semantic_search_task`, which owns the search polygon, the sim-time
budget, and coverage/recall scoring. conavgpt2 is launched standalone and
reports only its own round table, so **its numbers are not directly comparable**
until it is wired into `semantic_search_task`. Say so in any result table.

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

Three things are genuinely per-run and stay as `docker exec`:

```bash
R=disaster-dataset-robot-desktop-1

# 1. the VLM server (conavgpt2 only), in the robot container -> hence localhost
docker exec -d $R bash -lc 'mkdir -p /tmp/conavgpt2 && nohup /opt/lvlm-venv/bin/python \
  -m conavgpt2.vlm_server --port 8000 --device cuda:0 --model "$CONAVGPT2_VLM_MODEL" \
  --quantization "${CONAVGPT2_VLM_QUANT:-nf4}" --compute-dtype bfloat16 \
  --metrics-jsonl /tmp/conavgpt2/vlm_requests.jsonl > /tmp/conavgpt2/vlm_server.log 2>&1'
until docker exec $R bash -lc 'curl -sf localhost:8000/health | grep -q ok'; do sleep 10; done

# 2. takeoff
docker exec $R bash -lc 'ros2 action send_goal /robot_1/tasks/takeoff \
  task_msgs/action/TakeoffTask "{target_altitude_m: 12.0, velocity_m_s: 2.0}"'

# 3. the planner
docker exec $R bash -c "tmux new-window -t bringup -n conav; tmux send-keys -t bringup:conav \
  'sws && ros2 launch conavgpt2 conavgpt2.launch.xml \
     scene_params_file:=\$(ros2 pkg prefix conavgpt2)/share/conavgpt2/config/conavgpt2_modular_house.yaml \
   2>&1 | tee /tmp/conavgpt2/live.log' ENTER"
```

**Restart the ROBOT stack after every sim restart.** Sim time resets to 0 and the
robot keeps stale TF and PX4 state; the symptom is `TakeoffTask rejected: state
estimate timed out` with MAVROS looking perfectly healthy.

Watch it:

```bash
docker exec $R bash -c 'tail -f /tmp/conavgpt2/live.log' \
  | grep --line-buffered -E "round [0-9]+ \||sim budget|Traceback"
docker exec $R python3 /tmp/show_rounds.py     # round table WITH the VLM's reasoning
```

---

## 3. The numbers that MUST scale with the scene

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

## 4. Detector thresholds — measure, never inherit

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

## 5. Foxglove

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
| occupancy | `/conavgpt2/occupancy` | 4 classes out of a 3-class message: -1 transparent, 0 white, 100 black, **101 = target** via `invalidColor` |
| frontiers | `/conavgpt2/frontiers` | `frontier_marker_style`: `centroids` (default) / `cells` / `both` |
| ground truth | `/gcs/annotations/bboxes` | generated by the layout generator, per-class namespaces |
| drone | `/tf` | **must be bridged** — see the world->map trap below |

Alpha and colours live in the GENERATED layout (`render_layout.py`), not the
auto-opened one — import `/root/airstack_layout_num_robots_<N>.json`, or set
Alpha per-topic by hand.

---

## 6. Ground truth from the layout generator

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

## 7. The traps that cost a run

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
   mount AND is conavgpt2's default `camera_pitch_rad`, because TF walks the URDF
   and the URDF models no pitch. Set it in one place or not at all.
5. **The lidar is not free and mostly not needed.** conavgpt2 reads no lidar;
   `vdb_mapping` does. With it on, cameras drop ~53 -> ~17 Hz and `/clock` ~53 ->
   15 Hz — a ~3x wall-clock multiplier on a sim-time-bounded run, i.e. on GPU
   hours, for identical coverage.
6. **Results files append across runs.** `conavgpt2_rounds.jsonl` opens `'a'` and
   `round` restarts at 1 each process, so an iteration retry interleaves with the
   attempt it replaced. Group rows by `run_id`.

---

## 8. Submitting to OSMO

`osmo/missions/conavgpt2_wildfire_1robot.yaml` + `osmo/workflows/airstack-mission-2gpu.yaml`.

```bash
airstack osmo:mission osmo/missions/conavgpt2_wildfire_1robot.yaml \
  --pool <gpu-pool> --branch <branch> \
  --workflow osmo/workflows/airstack-mission-2gpu.yaml
```

Before submitting, check all of:

- [ ] **everything is COMMITTED AND PUSHED** — the pod clones from GitHub; an
      untracked package is a `ModuleNotFoundError` 20 minutes in
- [ ] the robot image is newer than `Dockerfile.robot`'s last change (`clip` for
      YOLO-World's `set_classes`; the mission self-heals but a rebuild is clean)
- [ ] `sem_threshold` measured on a real frame of THAT scene (§4)
- [ ] `map_extent_m` sized to the scene, and the cell-unit params rescaled (§3)
- [ ] the planner is launched AFTER the `/health` gate — its VLM preflight is
      fatal, so a bringup-time launch dies before the server exists
- [ ] a results-collection step exists — `record:` captures ROS topics only, and
      the round table and server metrics live on the container filesystem
- [ ] `ENABLE_LIDAR` off unless something needs `vdb_mapping` (§7.5)

The 2-GPU split is for the VLM's memory and latency isolation, **not** because
Isaac needs a card to itself — that theory was tested on one 16 GB card and
disproved (Isaac + a 4-bit VLM + YOLO-World co-resident at 12.0/16.3 GB with
cameras at 53 Hz).

---

## References

- `COA-docs/conavgpt2_status.md` — what is verified, what is not, and the open items
- `robot/ros_ws/src/global/planners/conavgpt2/VENDORED.md` — every deviation from upstream
- `.agents/skills/run-isaac-sim-launcher/` — reading a launcher's output, orphaned PX4
- `.agents/skills/launch-generated-scene-with-drones/` — the generated-scene path
- `.agents/skills/visualize-in-foxglove/` — adding a layer properly
- `osmo/missions/README.md` — mission schema, step types, GPU sizing
