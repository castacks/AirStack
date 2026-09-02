# raven_nav — single-agent RAVEN

`raven_nav_node` is the RAVEN paper's search planner, running on the map that
RayFronts publishes over ROS: **Voxel > Ray > LVLM-guided > Frontier**, first
condition that fires wins. One robot, one raven. N robots are N independent
ravens — there is no auction, no consensus, no peer state, no shared map merge,
and the only surviving baseline arm is `frontier_only_baseline`.

Its external interface (parameters, topic names/types/QoS, JSON schemas,
results file, log lines, console scripts) is unchanged from the multi-robot
version, so `semantic_search_task`, the GCS visualiser, `compile_results` and
`compare_to_groundtruth` all keep working.

```
rayfronts ── voxels_sim/all ──┐
             rays_sim/all ────┤                       ┌── global_plan (Path)
             frontiers ───────┤   ┌───────────────┐   ├── navigation_mode
                              ├──►│  RavenNavNode │──►├── raven_nav/discoveries
mavros ───── raw/fix ─────────┤   │ Voxel > Ray > │   ├── raven_nav/confirmed_targets
odometry ─────────────────────┤   │ LVLM > Front. │   ├── completed_targets
search_area (polygon) ────────┘   └───────┬───────┘   └── explored_area_coverage
                                          │
FPV image ──► VLM (HTTP) ──► guiding objects ──► rayfronts new_text_query
```

## The four behaviours

Each is a faithful numpy port of the matching file in
`RayFronts_raven/rayfronts/behaviors/`, and every constant carries a comment
naming its OG file and line. The behaviours import **no ROS at all** — they take
a `TickContext` of numpy arrays and return a `BehaviorOutput`; message building
lives in `raven_nav/ros_io.py`.

| module | OG source | what it does |
|---|---|---|
| `behaviors/voxel_behavior.py` | `voxel_behavior.py` | voxels over `voxel_score_threshold` (OG 0.98) on a target column -> 26-connected components on the 0.5 m grid -> boxes of at least `voxel_min_cluster_size` (OG 30) voxels -> fly to the nearest unvisited box, standing off 1.0 m from its surface; within 3 m it is visited (10 m cuboid gate) |
| `behaviors/ray_behavior.py` | `ray_behavior.py` | rays over `score_threshold` (OG 0.95) -> drop rays pointing back at the drone -> greedy 45 degree XY angle bins -> best group = `argmin(dist - 5*density)` -> waypoints at +6 m and +12 m along its mean bearing |
| `behaviors/lvlm_behavior.py` | `lvlm_behavior.py` + `LVLM/internvl3.py` | ask a VLM for three clue objects near the target, register them with rayfronts as queries, then fly 5 m along the mean bearing of rays scoring > 0.9 on a guiding column |
| `behaviors/frontier_behavior.py` | `frontier_behavior.py` | frontiers above the altitude floor -> DBSCAN(2.7, 3) -> centroids above 2 m are viewpoints -> score = distance + 5*(1 - cos to current motion) -> random pick of the top 5 -> wp2 = wp1 + 2 m |

Priority and the mode-switch reset are `behavior_manager.py`, ported from OG
`rayfronts/behavior_manager.py` and `mapping_server_rosnode.py:387,508-511`.

## The LVLM-guided behaviour

In the paper this was two processes: the behaviour raised `/lvlm_trigger` and a
separate InternVL3-2B node answered on `/lvlm_output`. Here it is one process
and one HTTP call:

1. the behaviour becomes eligible when targets exist and neither Voxel nor Ray
   fired; it publishes `raven_nav/lvlm_trigger` (Bool) every tick;
2. at most once per `lvlm_request_interval_s` (OG 30 s) the node grabs the
   latest FPV frame, JPEG-encodes it and POSTs it with the OG prompt to an
   OpenAI-compatible endpoint **on a worker thread** — the tick never blocks;
   the full record (prompt, model, latency, raw answer, parsed objects) goes out
   on `raven_nav/lvlm_request`;
3. the answer is cleaned with the OG `set_guiding_objects` rules (strip
   `a`/`an`/`the`, strip trailing punctuation, lower-case, dedupe) and published
   on `raven_nav/guiding_objects`;
4. each new guiding object is sent to rayfronts on `{rf}/new_text_query`, and
   the whole current list as JSON on `{rf}/guiding_queries` (latched, for the
   shared multi-robot server);
5. rayfronts answers with extra `q{k}_<label>` score columns; when any ray
   scores over `lvlm_ray_threshold` (OG 0.9) on one of them, the behaviour
   fires.

The prompt is the OG text verbatim (minus InternVL's `<image>` marker, which an
OpenAI-compatible request carries as a separate content part):

> Find {targets}. List three unique objects or areas that are most helpful as
> clues or context to locate the {targets}. Write ONLY the object or area names
> as a plain comma-separated list.

Endpoint resolution: `lvlm_vlm_url` -> `$VLM_URL` -> `$OPENAI_BASE_URL` ->
`http://offboard-compute:8000/v1`. Model: `lvlm_vlm_model` ->
`$CONAVGPT2_VLM_MODEL` -> the endpoint's first served model. If the endpoint is
unreachable at startup the node warns once and disables the behaviour, so the
chain falls through to Frontier. `raven_nav/lvlm_output` (String) accepts an
externally produced answer for tests and manual guidance.

## The four deviations from the OG paper logic

1. **Altitude clamp.** Every waypoint's z is clamped into
   `[min_altitude_agl, max_altitude_agl]`. OG emitted whatever z the map gave
   it; in AirStack the band is part of the `SemanticSearchTask` goal, and a
   casualty lying on the ground yields a standoff point around 0.5 m AGL, which
   droan will not fly. Implemented once, in `behaviors/common.py:clamp_z`.
2. **`search_area` polygon.** Frontier viewpoints, ray waypoints and voxel
   clusters outside the mission polygon are skipped (a cluster outside it is
   still *detected and reported*, just never flown to). OG had no polygon.
3. **`frontier_only_baseline` kept.** A real benchmark arm: navigation is pure
   frontier exploration, while voxel perception keeps running so the arm still
   reports `confirmed_targets` / `discoveries` / `completed_targets`. This is
   OG `behavior_manager.py:28`, the commented-out "pure frontier-based
   exploration" ablation, wired to a parameter.
4. **Detection memory + coverage completion.** RAVEN never finished; AirStack's
   `semantic_search_task` ends a search when raven publishes
   `navigation_mode == 'complete'`. `coverage.py` tracks own observed cells
   (pose, mapped voxels, raycasts toward frontiers) against the polygon, and
   `detection_memory.py` accumulates merged AABBs with an observing/visited
   status for the benchmark. Neither steers the drone.

### Two OG defects fixed (not deviations — the OG code contradicted itself)

* `ray_behavior.py:85` applied the forward-filter mask to the direction array
  only and then indexed the **unfiltered** origin/direction arrays with the
  filtered group indices (`OG:116-117`), so with any ray filtered out the group
  averages came from the wrong rays. The mask is applied to all three arrays
  here.
* `voxel_behavior.py:83-84` built the cluster box as `[min_p, max_p + vox]`,
  i.e. it treated a published voxel position as the voxel's min corner.
  RayFronts publishes voxel **centres**
  (`geometry3d.pointcloud_to_sparse_voxels` rounds `p/v` then multiplies back),
  so the OG box was half a voxel (0.25 m) off along every axis. Corrected here;
  `test_og_parity.py` pins the two implementations together through exactly
  that offset.

Both are asserted against the vendored OG torch code in `test_og_parity.py`.

## Parameters

`semantic_search_task` spawns this node with `ros2 run ... --ros-args -p ...`
and **no** `--params-file`, so `config/raven_nav.yaml` is only read by
`launch/raven_nav.launch.xml` and the `declare_parameter` defaults in
`raven_nav/params.py` are what actually run.

| parameter | default | meaning |
|---|---|---|
| `query_labels` | 3 demo labels | every rayfronts column, in registration order (re-derived at runtime from the `q{k}_<label>` topic names) |
| `target_labels` | `['']` | subset to navigate toward; empty = all |
| `min_altitude_agl` / `max_altitude_agl` | 1.5 / 100.0 | frontier altitude floor, and the deviation-1 clamp band |
| `score_threshold` | **0.95** | ray gate — OG `ray_behavior.py:42` |
| `voxel_score_threshold` | **0.98** | voxel gate — OG `voxel_behavior.py:51` |
| `voxel_min_cluster_size` | **30** | OG `voxel_behavior.py:78` |
| `voxel_min_confidence` | 0.0 | reporting-only floor on a box's mean score |
| `lvlm_enabled` | `$RAVEN_LVLM`, else true | LVLM-guided behaviour on/off |
| `lvlm_request_interval_s` | `$RAVEN_LVLM_INTERVAL_S`, else 30.0 | OG `LVLM/internvl3.py:35` |
| `lvlm_ray_threshold` | `$RAVEN_LVLM_RAY_THRESHOLD`, else 0.9 | OG `lvlm_behavior.py:78` — see the note below |
| `lvlm_vlm_url` / `lvlm_vlm_model` / `lvlm_image_topic` | `''` | see the resolution chains above |
| `coverage_complete_threshold` | 0.90 | polygon fraction that ends the mission (the mission forces 0.80) |
| `coverage_cell_size_m` / `coverage_raycast_range_m` / `coverage_raycast_min_step_m` | 0.5 / 30.0 / 5.0 | coverage grid |
| `results_dir` / `results_dump_period_s` | `''` / 3.0 | per-robot result JSON |
| `timer_period` | 0.5 | tick period |
| `nav_output_enabled` | true | live-settable gate on `global_plan` (plan without commanding) |
| `frontier_only_baseline` | false | deviation 3 |
| `debug_ray_table` / `debug_table_max_rows` / `debug_table_period_sec` / `debug_coordination` | true / 30 / 5.0 / true | the `debug/*` String tables and per-tick chatter |

> **Tuning the LVLM gate.** `lvlm_ray_threshold` is a **softmax** over the
> whole query set, so it is diluted by the background vocabulary: with the
> targets plus a ~33-label background list no column reaches the OG 0.9 and the
> behaviour can never fire. The mission goal has no field for it, so it takes
> its default from `RAVEN_LVLM_RAY_THRESHOLD` (and the request rate from
> `RAVEN_LVLM_INTERVAL_S`); a non-numeric value logs one warning and falls back
> to the OG literal. An explicit `-p` still wins over the environment.

> **Threshold change.** `score_threshold`, `voxel_score_threshold` and
> `voxel_min_cluster_size` now default to the ORIGINAL RAVEN values (0.95 /
> 0.98 / 30) instead of the tuned multi-robot ones (0.65 / 0.65 / 35). Missions
> that set `voxel_score_threshold` per scene are unaffected; missions that
> leave `score_threshold` at the `-1` sentinel now get the paper's 0.95.

Accepted but **inert** (the coordination and the other baselines were removed;
they must stay declared or the node aborts when `semantic_search_task` passes
them, and any set to a non-default value is logged once as ignored):
`vlfm_*`, `conavgpt_*`, `bundle_len`, `voxel_confirm_hits`,
`voxel_track_max_misses`, `voxel_proximity_engage_m`, `ray_confirm_hits`,
`ray_track_max_misses`, `commit_swap_improvement_frac`, `commit_min_hold_s`,
`commit_radius_m`, `ray_reach_factor`, `target_behind_penalty_weight`,
`commit_switch_margin_m`, `debug_auction`, `bb_release_timeout_s`.

## Topics

`{robot}` = `$ROBOT_NAME`, `{rf}` = `/robot_$ROS_DOMAIN_ID/rayfronts/msg_serv`.

**Subscribes** — `{rf}/rays_sim/all`, `{rf}/voxels_sim/all`, `{rf}/frontiers`
(PointCloud2); `/{robot}/odometry` (remapped by the spawner to
`odometry_conversion/odometry`); `/{robot}/interface/mavros/global_position/raw/fix`
(BEST_EFFORT); `/{robot}/raven_nav/search_area` (PolygonStamped, TRANSIENT_LOCAL);
`/input_prompt`; `/{robot}/raven_nav/clear_blacklist` (Empty, now a logged
no-op); `/{robot}/raven_nav/lvlm_output` (String); the FPV image
(BEST_EFFORT depth 1).

**Publishes** — `/{robot}/global_plan` (Path, frame `map`, gated by
`nav_output_enabled`); `/{robot}/navigation_mode` (String:
`idle|frontier|ray|voxel|lvlm|complete`); `/{robot}/completed_targets`,
`/{robot}/raven_nav/discoveries`, `/{robot}/raven_nav/confirmed_targets`
(JSON Strings); `/{robot}/raven_nav/explored_area_coverage` (CoverageGrid);
`/{robot}/filtered_frontiers`, `/{robot}/frontier_viewpoints` (PointCloud2);
`/{robot}/filtered_rays`, `/{robot}/voxel_clusters` (MarkerArray);
`/{robot}/current_target`; `/{robot}/debug/{ray,groups,voxel,frontier,discoveries}_table`;
`/{robot}/raven_nav/{lvlm_trigger,guiding_objects,lvlm_request}`;
`{rf}/new_text_query` and `{rf}/guiding_queries` for the guiding objects.

Removed with the coordination code: `bids`, `committed_target`, `shared_rays`,
`shared_frontiers`, `raven_nav/{auction_table,ray_leads,served_leads,explore_bid}`,
`debug/{bids_table,auction_solve}`, `conavgpt/*`, and the
`coordination/peer_registry` subscription.

## Frames

Rays and voxels arrive RDF and are flipped to FLU in the node's callbacks
(`x = z, y = -x, z = -y`); frontiers stay RDF and the frontier behaviour flips
them itself, as the OG did. Everything the behaviours see is the robot's own
local `map` frame. `_local_to_world` lifts a point to global ENU by adding
`boot_enu` in XY and leaving z as AGL (the ground-truth annotations are
ground-relative). `boot_enu` anchors the odom ORIGIN, not the current pose:
raven is spawned after takeoff, so the first fix is corrected by `_cur_pose`.

## Layout

```
raven_nav/
  raven_nav_node.py     ROS wiring, the tick, frames, reporting  (the only
                        file that owns rclpy)
  behavior_manager.py   the priority chain
  behaviors/
    common.py           TickContext / BehaviorOutput, clamp_z, polygon test
    voxel_behavior.py   ray_behavior.py  lvlm_behavior.py  frontier_behavior.py
  params.py             the declared-parameter table + log-line formats (pure)
  ros_io.py             message building/parsing (the only other ROS importer)
  lvlm_client.py        stdlib OpenAI-compatible VLM client + worker thread
  coverage.py           observed cells and the polygon gate
  detection_memory.py   persistent AABBs + the target event log
  results.py            the results-file schema
  ray_groups.py         the RayGroup value type
  discoveries.py  ray_targets.py  compile_results.py  compare_to_groundtruth.py
  view_ray_tables.py    unchanged
```

## Tests

Pure tests (no ROS, no torch) run on the host:

```bash
cd robot/ros_ws/src/global/planners/raven_nav && \
uv run --with numpy --with scipy --with scikit-learn --with pytest \
       --with opencv-python-headless python -m pytest test -q
```

(Add `--no-project` if you would rather uv did not touch the repo-root `.venv`.)

| file | what it pins |
|---|---|
| `test_frontier_behavior.py` | DBSCAN viewpoints, altitude gates, momentum, top-5 random pick, lock/unlock, both deviations |
| `test_ray_behavior.py` | the 45 degree binning (first-fit, order-sensitive, running-mean centroid), the forward filter, density scoring, +6/+12 m waypoints |
| `test_voxel_behavior.py` | 26-connected CCL, cluster boxes, the visited gate, the ray-cast standoff, both deviations |
| `test_lvlm_behavior.py` | OG cleaning, the OG prompt text, the 30 s throttle, guiding-column mapping, and the whole HTTP client against a real `http.server` |
| `test_behavior_manager.py` | priority, mode-switch reset, `frontier_only` short-circuit, LVLM off |
| `test_detection_memory.py` | observing -> visited, stickiness, passive fly-by detection, the event log |
| `test_coverage.py` | cell stamping, raycast pullback and rate limit, the fraction, the packed grid |
| `test_results_schema.py` | the results keys against `compile_results.py` (it is actually run over a dump), and the two JSON topics against their readers |
| `test_param_contract.py` | every `-p` flag `semantic_search_task/node.py` passes is declared |
| `test_log_contract.py` | every log line survives `semantic_search_task._filter_raven`, loaded by file path |
| `test_node_smoke.py` | the whole node — construction, subscriptions, every mode's tick, the frame lift, the results file, the LVLM plumbing — against the stubbed ROS modules in `test/ros_stubs.py`; skips where real ROS is installed |
| `test_og_parity.py` | the numpy ports against the vendored OG **torch** code in `test/og_reference/` — skipped without torch, runs in the robot container |
| `test/integration/test_node_roundtrip.py` | the whole node over real DDS — skipped without rclpy, runs in the robot container |

`test_ray_targets.py::test_build_targets_single_bearing_yields_unconfirmed`
fails and did so before this rewrite; `ray_targets.py` is unchanged.

In the robot container (adds the parity and integration suites):

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 -m pytest src/global/planners/raven_nav/test -q
```
