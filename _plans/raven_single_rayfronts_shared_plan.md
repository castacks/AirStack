# RAVEN single-agent + shared off-board RayFronts — build plan (2026-09-01)

Owner: lead session. Four work packages run in parallel; each owns the files
listed in its section and nothing else. Reference reports (read before coding):

- `REPORTS/interface_contract.md`  — every external touchpoint of raven_nav
  (spawn argv, topics, JSON schemas, results file, GCS/gossip consumers).
- `REPORTS/rayfronts_internals.md` — rayfronts encoder/mapper/dataset/vis/
  messaging internals, configs, docker layout.
- `REPORTS/frames_scene_testenv.md` — per-robot `map` frames + boot ENU,
  ROS domains, camera topics, scene presets, test environment.

(`REPORTS` = `/tmp/claude-1000/-home-krrishjain-SEI-COA-disaster-dataset/c85c0d3d-0c7a-472c-8dd8-3184a2ccc7c6/scratchpad/reports`)

Original RAVEN sources (read-only reference, cloned):
`/tmp/claude-1000/-home-krrishjain-SEI-COA-disaster-dataset/c85c0d3d-0c7a-472c-8dd8-3184a2ccc7c6/scratchpad/ref/`
- `RayFronts_raven/rayfronts/behaviors/{frontier,ray,voxel,lvlm}_behavior.py`,
  `behavior_manager.py`, `mapping_server_rosnode.py`  ← THE paper logic
- `RayFronts_raven/run_mapping_server_rosnode.sh`, `background.txt`
- `LVLM/internvl3.py` ← the paper's LVLM node (trigger/throttle/prompt)
- `RayFronts/` upstream, `AirStack_raven/` the paper's AirStack branch.

Pre-edit snapshot of everything we touch:
`.../scratchpad/snapshot/{raven_nav_orig,rayfronts_orig,rayfronts_configs_orig,rayfronts.launch.xml}`

## 0. Goal

1. `raven_nav` runs the ORIGINAL single-robot RAVEN paper logic (Voxel > Ray >
   LVLM-guided > Frontier), including the LVLM-guided behaviour, with ALL
   multi-robot coordination removed (auction/consensus, bid manager, peer
   state, weighted/repulsed frontier nav, shared rays/frontiers/leads,
   coverage merging, bundles, baselines). N robots = N independent ravens.
   Its EXTERNAL INTERFACE with AirStack does not change (see §2).
2. RayFronts runs ONCE, on the `offboard-compute` container, subscribing to
   every robot's RGB/depth/pose (each robot on its own ROS domain), polling
   them round-robin into ONE mapper, and publishing map outputs back into
   each robot's domain under that robot's existing topic names and in that
   robot's local `map` frame — so raven_nav does not know the map is shared.
   A `ClientEncoder` (RGB in, features out over a GPU-resident transport) talks
   to a separate `encoder_server` process; the mapping server can also run the
   encoder in-process.
3. Configs tuned for HUMAN targets (small clusters) with a curated background
   vocabulary.
4. Lots of unit tests: logic, component interaction, and the AirStack /
   Isaac-Sim boundary (topic names, frames, param names, JSON schemas).
5. A 250 x 250 m suburb tornado test scene with people; two drones spawned
   next to (a) a fully exposed casualty and (b) a partially covered one.
   NOT launched until the user clears it.

## 1. Architecture

```
 isaac-sim (domain i publishes /robot_i/...)
   │ rgb, depth, camera_info, odometry, navsat            per robot i
   ▼
 offboard-compute ──────────────────────────────────────────────────────────
 │  encoder_server (radseg on GPU)  ◄── CUDA-IPC socket ──►  ClientEncoder   │
 │                                                              │            │
 │  multi_robot_mapping_server: ctx(domain 1) ctx(domain 2) … ──┤ ONE mapper │
 │     per-robot Ros2 input node → anchor (boot ENU) → shift to world frame  │
 │     round-robin frames → SemanticRayFrontiersMap → queries (softmax)      │
 │     per-robot output node: voxels_sim/rays_sim/frontiers/vis/status,      │
 │        shifted BACK to robot i's local frame, published on domain i       │
 └──────────────────────────────────────────────────────────────────────────┘
   ▲ new_text_query, guiding_queries (per robot)   │ voxels_sim, rays_sim, frontiers, status
   │                                               ▼
 robot_i container (domain i): semantic_search_task ──spawns──► raven_nav_node
   raven_nav_node: Voxel > Ray > LVLM > Frontier  →  /robot_i/global_plan
   LVLM: FPV frame + OG prompt → VLM HTTP (offboard-compute:8000) → guiding
         objects → guiding_queries → rayfronts columns → ray behaviour
```

Frames: each robot's odometry is in ITS OWN `map` (anchored at its spawn).
World = local + boot_enu_i (xy only; z stays AGL). boot_enu_i = mean over K
samples of `gps_to_enu(fix_i) - odom_i` (formula + Lisbon origin constants in
`coordination_bringup/frame_utils.py`; rayfronts re-implements the 10 lines).

## 2. Contracts (frozen — every WP codes against these)

### 2.1 raven_nav external interface (UNCHANGED; see interface_contract.md)
- Executable `raven_nav_node`, entry `raven_nav.raven_nav_node:main`.
- Params accepted (all declared; coordination/baseline ones accepted and
  logged as ignored once): `query_labels`, `target_labels`,
  `min_altitude_agl`, `max_altitude_agl`, `use_sim_time`,
  `frontier_only_baseline`, `vlfm_baseline`, `vlfm_use_voxel_targets`,
  `vlfm_value_weight`, `vlfm_ray_blacklist`, `conavgpt_baseline`,
  `conavgpt_leader_id`, `conavgpt_round_period_s`, `conavgpt_max_regions`,
  `conavgpt_assignment_ttl_s`, `conavgpt_replan_on_reach_m`,
  `coverage_complete_threshold`, `results_dir`, `score_threshold`,
  `voxel_score_threshold`, `voxel_min_confidence`, `voxel_min_cluster_size`,
  `bundle_len`, `timer_period`, `nav_output_enabled`, `results_dump_period_s`,
  `coverage_cell_size_m`, `coverage_raycast_range_m`,
  `coverage_raycast_min_step_m`, `voxel_confirm_hits`,
  `voxel_track_max_misses`, `voxel_proximity_engage_m`, `ray_confirm_hits`,
  `ray_track_max_misses`, `commit_swap_improvement_frac`, `commit_min_hold_s`,
  `commit_radius_m`, `ray_reach_factor`, `target_behind_penalty_weight`,
  `commit_switch_margin_m`, `debug_auction`, `debug_coordination`,
  `debug_ray_table`, `debug_table_max_rows`, `debug_table_period_sec`,
  `bb_release_timeout_s`. `frontier_only_baseline=true` MUST still work
  (pure frontier, passive voxel detection) — it is a real baseline arm.
  NEW params: `lvlm_enabled` (bool, default from env `RAVEN_LVLM`, default
  true), `lvlm_request_interval_s` (30.0), `lvlm_ray_threshold` (0.9),
  `lvlm_vlm_url` ('' → env `VLM_URL`, then `OPENAI_BASE_URL`, then
  `http://offboard-compute:8000/v1`), `lvlm_vlm_model` ('' → env
  `CONAVGPT2_VLM_MODEL`, else first served model), `lvlm_image_topic`
  ('' → `/{robot}/sensors/front_stereo/left/image_rect`).
- Subscriptions (same names/types/QoS): `{rf}/rays_sim/all`,
  `{rf}/voxels_sim/all`, `{rf}/frontiers` (rf =
  `/robot_{ROS_DOMAIN_ID}/rayfronts/msg_serv`), `/{robot}/odometry`
  (remapped by the spawner), `/input_prompt`, `/{robot}/raven_nav/clear_blacklist`,
  `/{robot}/interface/mavros/global_position/raw/fix` (BEST_EFFORT),
  `/{robot}/raven_nav/search_area` (PolygonStamped, TRANSIENT_LOCAL).
  DROP: `/{robot}/coordination/peer_registry`, `/{robot}/conavgpt/assignment`.
  NEW: `/{robot}/raven_nav/lvlm_output` (String, for a manual/external LVLM),
  FPV image (BEST_EFFORT, latest only).
- Publications kept (same name/type/payload): `/{robot}/global_plan` (Path,
  frame `map`, gated by `nav_output_enabled`), `/{robot}/navigation_mode`
  (String: idle|frontier|ray|voxel|lvlm|complete — `lvlm` is new; `complete`
  ends the mission), `/{robot}/completed_targets` (JSON list),
  `/{robot}/raven_nav/discoveries` (JSON, schema from `discoveries.py`),
  `/{robot}/raven_nav/confirmed_targets` (JSON, world ENU),
  `/{robot}/raven_nav/explored_area_coverage` (CoverageGrid, own cells),
  `/{robot}/filtered_frontiers`, `/{robot}/frontier_viewpoints`,
  `/{robot}/filtered_rays` (MarkerArray), `/{robot}/voxel_clusters`
  (MarkerArray), `/{robot}/current_target`, `/{robot}/debug/{ray_table,
  groups_table,voxel_table,frontier_table,discoveries_table}` (String).
  NEW: `/{robot}/raven_nav/lvlm_trigger` (Bool), `/{robot}/raven_nav/
  guiding_objects` (String JSON list), `/{robot}/raven_nav/lvlm_request`
  (String JSON: ts, prompt, model, latency, raw answer, parsed objects).
  DROP: `bids`, `committed_target`, `shared_rays`, `shared_frontiers`,
  `raven_nav/{auction_table,ray_leads,served_leads,explore_bid}`,
  `debug/{bids_table,auction_solve}`, `conavgpt/*`.
- Results file `<results_dir>/<robot>.json` — identical schema (keys listed in
  interface_contract.md §4.1); `intent_events` stays as `[]`.
- Log lines: keep the literal substrings `raven_nav started`,
  `waiting for odometry`, `boot GPS captured`, `search_area`, and the per-tick
  status line beginning with `[Frontier-based]` / `[Ray-based]` /
  `[Voxel-based]` / `[LVLM-guided]` (semantic_search_task's `_filter_raven`
  regex only knows the first three; WP-C adds `LVLM-guided` to that regex —
  the ONLY semantic_search_task change on the raven side).
- `raven_nav.compile_results`, `raven_nav.compare_to_groundtruth`,
  `raven_nav.view_ray_tables`, `annotations/*.json`: unchanged.
- To rayfronts: raven publishes each guiding object to
  `{rf}/new_text_query` (String, one per message — works with the legacy
  per-robot server) AND the full current list as JSON to
  `{rf}/guiding_queries` (String, RELIABLE + TRANSIENT_LOCAL depth 1).

### 2.2 Shared rayfronts server (WP-B builds, WP-C launches, WP-A relies on)
Processes (both on offboard-compute, system python3, cwd = rayfronts source):
```
python3 -m rayfronts.encoder_server  encoder=radseg \
        encoder_server.socket=/tmp/rayfronts/encoder.sock
python3 -m rayfronts.multi_robot_mapping_server --config-name shared_humans \
        dataset.robot_ids=[1,2] encoder=client \
        encoder.socket=/tmp/rayfronts/encoder.sock
```
- `configs/encoder/client.yaml` (`_target_: rayfronts.image_encoders.ClientEncoder`,
  `socket`, `timeout_s`, `transport: auto|cuda_ipc|cpu`).
- `configs/dataset/multi_ros2isaacsim.yaml` (`_target_:
  rayfronts.datasets.MultiRobotRos2Subscriber`; `robot_ids: [1]`,
  `domain_ids: null` (= ids), topic TEMPLATES with `{robot}` =
  `robot_{id}`: `rgb_topic: /{robot}/sensors/front_stereo/left/image_rect`,
  `depth_topic: /{robot}/sensors/front_stereo/left/depth_ground_truth`,
  `intrinsics_topic: /{robot}/sensors/front_stereo/left/camera_info`,
  `pose_topic: /{robot}/odometry_conversion/odometry`,
  `pose_msg_type: odometry|pose_stamped`, `navsat_topic:
  /{robot}/interface/mavros/global_position/global`,
  `anchor_mode: gps|static|none`, `anchor_samples: 10`,
  `robot_offsets_xy: null | {id: [x, y]}`, `src_coord_system: flu`,
  `rgb_resolution`, `depth_resolution`, `frame_skip`, `queue_size`,
  `sync_slop_s`).
- `configs/messaging_service/multi_ros.yaml` and `configs/vis/multi_ros.yaml`
  (`topic_prefix_template: /{robot}/rayfronts/msg_serv`,
  `text_query_topic_template: /{robot}/rayfronts/msg_serv/new_text_query`,
  `guiding_queries_topic_template: /{robot}/rayfronts/msg_serv/guiding_queries`,
  `status_topic_template: /{robot}/rayfronts/status`,
  `vis topic_prefix_template: /{robot}/rayfronts`).
- Per-robot OUTPUT topics (domain i, robot i's LOCAL frame, RDF like today):
  `/robot_i/rayfronts/msg_serv/{voxels_sim/all, voxels_sim/q{k}_{label},
  rays_sim/all, rays_sim/q{k}_{label}, frontiers}` with exactly today's field
  layouts; `/robot_i/rayfronts/{pose/pose, pose/img, voxel_rgb, ...}` vis;
  NEW `/robot_i/rayfronts/status` (String JSON, 1 Hz, RELIABLE+TRANSIENT_LOCAL
  depth 1): `{"robot":"robot_i","domain":i,"anchored":bool,
  "boot_enu":[x,y,z]|null,"frames_robot":n,"frames_total":n,
  "queries":[labels in column order],"vox_count":n,"ray_count":n,"ts":t}`.
- Query set = union over robots of (new_text_query adds) ∪ (each robot's
  current guiding list). Column order = registration order; a guiding label
  removed by every robot is deleted from the query set (OG `delete_queries`),
  never a label that arrived via new_text_query.
- `shared_humans.yaml` (in `common/rayfronts_configs/`): derived from
  `low_memory.yaml` (encoder radseg, PCA, compute_prob True, vox 0.5,
  480x480, frame_skip 10) with small-object changes (sem_pruning_thresh→1,
  sem_pruning_period 32→8, max_pts_per_frame 1000→4000, max_rays_per_frame
  500, fronti unchanged) — each knob commented with why.
  `common/rayfronts_configs/background_humans.txt`: curated background list
  for a tornado-hit suburb (see WP-B).

### 2.3 semantic_search_task shared mode (WP-C)
Env `RAYFRONTS_MODE=per_robot` (default, byte-for-byte today) | `shared`.
In `shared`: do not spawn `rayfronts.launch.xml`; instead subscribe to
`/robot_{ROS_DOMAIN_ID}/rayfronts/status` and wait until `anchored` and
`frames_robot >= required_batches` (same timeout / abort semantics, feedback
text `[rayfronts] shared: robot frames k, anchored yes/no`); never kill
`rayfronts.mapping_server` processes. Queries still go to this robot's
`new_text_query`.

### 2.4 offboard-compute (WP-C)
`START_RAYFRONTS_SERVER=false` (default; when `true` start encoder_server +
multi_robot_mapping_server with ROS sourced, `FASTRTPS_DEFAULT_PROFILES_FILE`
and `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` set, robot ids from
`RAYFRONTS_ROBOT_IDS` else `1..NUM_ROBOTS`, config from `RAYFRONTS_CONFIG`
(default `shared_humans`), source tree from `RAYFRONTS_SRC` (default
`/root/AirStack/common/rayfronts`, PYTHONPATH also gets
`/opt/rayfronts/rayfronts/csrc/build` for the compiled extension, and
`--config-dir /root/AirStack/common/rayfronts_configs`), logs to
`/tmp/offboard/rayfronts_{encoder,mapping}.log`.

## 3. WP-A — raven_nav single-agent rewrite (owner: agent A)

Files owned: everything under `robot/ros_ws/src/global/planners/raven_nav/`
EXCEPT `annotations/`, `compile_results.py`, `compare_to_groundtruth.py`,
`discoveries.py`, `ray_targets.py`, `view_ray_tables.py` (keep as-is; they
are pure and their tests pass — except one pre-existing `test_ray_targets`
failure, leave it).

Delete: `bid_manager.py`, `peer_state.py`, `view_auction_tables.py`,
`analyze_auction_solve.py`, `plan.md`, `behaviors/vlfm_behavior.py`,
`behaviors/conavgpt_behavior.py`, `behaviors/pursuit_stuck.py`,
`behaviors/goal_progress.py`, `track_confirmation.py`, `ray_groups.py` (only
if nothing kept needs it; `discoveries.py`/`ray_targets.py` import
`RayGroup` from it — then keep the dataclass), `test/test_bid_manager.py`.
Update `setup.py` console scripts, `package.xml` deps (drop `airstack_msgs`
if BidVector gone; keep `coordination_bringup` for `gps_to_enu`/`dir_to_quat`
and `coordination_msgs` for `CoverageGrid`), `config/raven_nav.yaml`
(document OG defaults), README.md (rewrite: what the paper logic is, the
deviations below, how LVLM is wired, params).

Behaviours = faithful numpy ports of the OG files, operating on the ROS-topic
inputs raven already has (rays: origins/dirs in FLU + softmax score matrix;
voxels: xyz FLU + score matrix; frontiers: xyz RDF cols 0:3). Column k of the
score matrix ↔ label k of the rayfronts query order (parsed from
`q{k}_{label}` topic names, `_detect_rayfronts_labels` logic, matching by
the messaging service's `_sanitize_topic_name` rule; unknown columns keep
the sanitized name).

- Priority: Voxel > Ray > LVLM-guided > Frontier (OG `behavior_manager`).
  Mode switch resets waypoint lock + targets. Publish `navigation_mode`.
- Frontier (OG): frontiers → FLU; z > min_altitude (OG 1.5); DBSCAN(eps 2.7,
  min_samples 3); centroids with z > 2.0 (keep OG constant) = viewpoints;
  score = dist + 5·(1−cos to current motion) when a target exists else dist;
  random pick among top-5; if not locked: wp1 = viewpoint, wp2 = wp1 + 2·dir;
  unlock when within 5 m of wp1. Publish `frontier_viewpoints`,
  `filtered_frontiers`, `debug/frontier_table`.
- Ray (OG): score threshold `score_threshold` (OG 0.95) on target columns;
  keep rays whose (origin+dir) target is ahead of the drone in XY; 45° XY
  angle-binning into groups (OG incremental algorithm, verbatim); group =
  mean origin, mean dir (normalised), density; best = argmin
  dist(origin, drone) − 5·density; wp1 = origin + 6·dir, wp2 = origin + 12·dir;
  unlock when within 4 m of wp2. Publish `filtered_rays` markers.
- Voxel (OG): threshold `voxel_score_threshold` (OG 0.98) on target columns
  (voxel assigned to the target column it passes; OG used any-of, keep
  any-of + argmax label for naming); 26-connected CCL on the 0.5 m grid;
  components with ≥ `voxel_min_cluster_size` (OG 30) voxels → AABB
  (center, size) in FLU; `visited` if within 10 m cuboid distance of a
  visited cluster (OG `is_near_visited` threshold 10.0); execute: nearest
  unvisited, ray-cast to AABB surface, standoff 1.0 m short, wp2 = standoff
  (locked), wp1 = 0.8 blend; within 3 m of wp2 → mark visited, unlock.
  Publish `voxel_clusters` markers, `debug/voxel_table`.
- LVLM-guided (OG `lvlm_behavior` + `LVLM/internvl3.py` merged into raven):
  eligible when targets exist and neither Voxel nor Ray fired; publish
  `lvlm_trigger`; at most one VLM request per `lvlm_request_interval_s`
  (OG: node throttled 30 s); request = latest FPV frame as JPEG data-URL +
  OG prompt (`Find {targets}. List three unique objects or areas that are
  most helpful as clues or context to locate the {targets}. Write ONLY the
  object or area names as a plain comma-separated list.`) via
  OpenAI-compatible `POST {url}/chat/completions` (module
  `raven_nav/lvlm_client.py`, stdlib urllib + cv2 for JPEG; request in a
  worker thread, never block the tick); answer → `set_guiding_objects`
  cleaning (strip a/an/the, trailing punctuation, dedupe, lower) →
  `guiding_objects` topic + rayfronts (`new_text_query` each + JSON
  `guiding_queries`); condition true when any ray scores >
  `lvlm_ray_threshold` (0.9) on a guiding column → execute: mean origin/dir
  of those rays, path [origin, origin + 5·dir]; returns lock False, targets
  None (OG). If the VLM endpoint is unreachable at start: warn once, behaviour
  disabled (falls through to Frontier), `lvlm_enabled` false. Also accept an
  external answer on `raven_nav/lvlm_output` (String) for tests/manual use.
- Detection reporting (interface, not coordination): persistent
  `DetectionMemory` of confirmed AABBs (merge via
  `discoveries.merge_confirmed_targets`; status `observing` → `visited` when
  the drone passes within 3 m of the box surface or the voxel behaviour marks
  the cluster visited). `confirmed_targets` (world ENU) and `discoveries`
  (via `discoveries.build_discoveries`, ray targets from
  `ray_targets.build_targets` over RayGroups built from the OG angle groups)
  published every tick; `target_events` + results dump exactly as today;
  `completed_targets` = labels with ≥1 visited box.
- Coverage/completion (interface): own observed cells (pose + voxels +
  raycast toward frontiers, existing code) → polygon coverage fraction →
  `complete` mode + hover when ≥ `coverage_complete_threshold`. No peer zones.
- Deviations from OG, all required by the AirStack mission interface and to
  be documented in README + reported to the user: (1) waypoint z clamped to
  `[min_altitude_agl, max_altitude_agl]` for every behaviour (OG had no band;
  a lying casualty's standoff point is 0.5 m AGL); (2) `search_area` polygon:
  frontier viewpoints / ray waypoints / voxel clusters outside it are
  skipped (OG had no polygon); (3) `frontier_only_baseline` kept; (4)
  detection memory + coverage completion kept for the benchmark.
- Every other current extension (temporal confirmation, stuck monitors,
  goal progress, blacklists, bearing hysteresis, house merging, dead-BB
  watchdog, peer anything) is REMOVED.

Tests (pytest, pure numpy, run with
`cd robot/ros_ws/src/global/planners/raven_nav && uv run --with numpy --with scipy --with scikit-learn --with pytest python -m pytest test -q`):
`test_frontier_behavior.py`, `test_ray_behavior.py`, `test_voxel_behavior.py`,
`test_lvlm_behavior.py` (cleaning, throttle, column mapping, prompt text,
fake HTTP server via `http.server` in a thread), `test_behavior_manager.py`
(priority + mode-switch reset), `test_detection_memory.py`,
`test_coverage.py`, `test_results_schema.py`, `test_param_contract.py` (the
node's declared-parameter table contains every name in §2.1),
`test_log_contract.py` (import `semantic_search_task.node._filter_raven` by
file path and assert the node's status-line formats match),
`test_og_parity.py` (vendor the OG torch functions under
`test/og_reference/`, skip if torch missing, compare on random inputs —
runs in the robot container). Keep ROS message construction behind a thin
`raven_nav/ros_io.py` so behaviours import no rclpy; guard rclpy imports so
`python -c "import raven_nav.behaviors.ray_behavior"` works without ROS.
Integration (rclpy, runs in the robot container via WP-C's harness):
`test/integration/test_node_roundtrip.py` — instantiate `RavenNavNode` with
fake publishers for rays/voxels/frontiers/odometry/navsat/search_area, drive
ticks, assert global_plan/navigation_mode/discoveries/results file.

## 4. WP-B — RayFronts ClientEncoder + shared multi-robot server (owner: agent B)

Files owned: `common/rayfronts/rayfronts/**` (new files + minimal edits to
`datasets/ros.py`, `visualizers/ros.py`, `messaging_services/ros.py`,
`image_encoders/__init__.py`, `datasets/__init__.py`, `visualizers/__init__.py`,
`messaging_services/__init__.py`, `mapping_server.py` only if needed),
`common/rayfronts/tests/**` (new), `common/rayfronts_configs/**`.
Do NOT touch anything under `robot/`.

1. Context plumbing: give `Ros2Subscriber`, `Ros2Vis`, `Ros2MessagingService`
   optional `context=None, domain_id=None, node_name=None` kwargs; create a
   private `rclpy.Context` initialised with `domain_id` when given; `shutdown`
   destroys the node and shuts down ONLY its own context (never the global
   default context when it was not created here). Legacy behaviour when the
   kwargs are absent must be byte-for-byte unchanged (existing single-robot
   launch keeps working).
2. `image_encoders/client_encoder.py`: `ClientEncoder(LangSpatialGlobalEncoder)`
   proxying every method the mappers/mapping_server use
   (`encode_image_to_feat_map`, `encode_image_to_vector`, `encode_labels`,
   `encode_prompts`, `align_spatial_features_with_language`,
   `align_global_features_with_language`, `is_compatible_size`/resolution
   attrs, anything else the base classes require) to `encoder_server.py`
   over a Unix socket (`multiprocessing.connection.Listener/Client`).
   Transport: `import torch.multiprocessing` so CUDA tensors cross as CUDA
   IPC handles (stay on GPU); protocol = request → response → ack, the
   server keeps a response alive until the ack, the client clones the
   received tensor before acking; automatic fallback to CPU tensors when
   either side has no CUDA or `transport: cpu`. Server: hydra main,
   instantiates `cfg.encoder`, serves N clients (thread per client),
   `--config-name` default, config group `encoder_server` (`socket`,
   `device`, `warmup`). Handle client disconnect, oversized payloads, and
   version/handshake (encoder class name, feature dim, resolution).
3. `datasets/multi_ros.py`: `MultiRobotRos2Subscriber(PosedRgbdDataset)` —
   one per-robot input node (own context/domain), synchronised rgb/depth/
   pose (Odometry or PoseStamped) via `message_filters`, intrinsics from
   camera_info (all robots must agree on intrinsics/resolution; assert and
   log), anchoring per robot (`gps|static|none`), pose shift local→world
   (xy add) BEFORE the FLU→RDF transform, round-robin `__iter__` over
   per-robot queues (skip empty queues, block only when all are empty,
   respect `frame_skip` per robot), each yielded dict carries
   `robot_id`/`robot_name` (as tensors/strings the DataLoader collate can
   handle — or bypass DataLoader in the multi server) plus a per-robot
   `frames_robot` counter; `shutdown()` stops all contexts.
4. `messaging_services/multi_ros.py`: `MultiRobotRos2MessagingService` — one
   node per robot domain; subscribes each robot's `new_text_query` (→ shared
   add_queries) and `guiding_queries` (JSON list → per-robot set; the server
   recomputes the union and deletes guiding labels no robot lists, never
   base/target labels); `publish_query_results` / `publish_pc` publish to
   EVERY robot's prefix, shifting xyz by −boot_enu_i (RDF ⇄ FLU handled: the
   shift is applied in the world FLU frame, i.e. shift RDF (x,y,z) by
   (−by, −bz(=0), −bx) — derive and unit-test it against
   `g3d.get_coord_system_transform`), only when that robot's topic has a
   subscriber; `publish_status(robot_id, dict)` at 1 Hz on the status topic.
5. `visualizers/multi_ros.py`: `MultiRobotRos2Vis` — per-robot layers:
   inputs (`log_pose`, `log_img`, depth) published under the robot that
   produced the frame; map layers published to every robot prefix (gated by
   subscriber count) shifted into that robot's frame. Reuse `Ros2Vis`
   instances internally.
6. `multi_robot_mapping_server.py`: subclass/refactor of `MappingServer`:
   builds the mapper once, iterates the multi-robot dataset (no DataLoader
   batching across robots — batch_size 1), calls the mapper per frame,
   tracks the producing robot for vis, runs queries/frontier publishing on
   the existing periods, publishes status, handles guiding-query deletion
   (port OG `delete_queries` from `mapping_server_rosnode.py`), clean
   shutdown. `--config-name shared_humans` default group overrides:
   `dataset: multi_ros2isaacsim`, `messaging_service: multi_ros`,
   `vis: multi_ros`, `encoder: client`.
7. Configs (§2.2), `common/rayfronts_configs/shared_humans.yaml`,
   `background_humans.txt` (≈25–40 labels: road, asphalt, driveway, grass,
   lawn, dirt, mud, tree, fallen tree, bush, house, roof, shingles, wall,
   brick wall, fence, car, truck, pickup, debris, wood planks, lumber,
   rubble, sky, cloud, power line, utility pole, mailbox, trash can,
   swimming pool, sidewalk, concrete slab, window, door, garage, tarp,
   furniture, tire, pipe — prune to what is high-quality and non-overlapping
   with "person"), and a short `common/rayfronts_configs/README.md`.
8. Tests in `common/rayfronts/tests/` (pytest; mark `ros`, `cuda`, `slow`):
   - `test_client_encoder.py`: DummyEncoder (deterministic conv) served by
     `encoder_server` in a subprocess; roundtrip equality for every proxied
     method on CPU; CUDA IPC path when `torch.cuda.is_available()` (skip
     otherwise); ack/keep-alive correctness (server frees after ack);
     two concurrent clients; server death → clear error; fallback transport.
   - `test_multi_ros_dataset.py` (`ros`): two fake robots on two domains
     (rclpy contexts in-process) publishing synthetic rgb/depth/camera_info/
     Odometry/NavSatFix; assert round-robin interleaving, frame_skip,
     anchoring math (boot from GPS − odom, xy only), world-shifted poses,
     static offsets mode, shutdown leaves other contexts alive.
   - `test_frame_shift.py`: local↔world↔RDF shift identities against
     `geometry3d`.
   - `test_multi_messaging.py` (`ros`): per-robot topic names, field layouts
     (`x,y,z,sim_k`, `x,y,z,theta,phi,sim_k`), frame shift per robot,
     publish-only-if-subscribed, status JSON schema, guiding-query union +
     deletion refcount, sanitized `q{k}_{label}` names.
   - `test_multi_vis.py` (`ros`): per-robot pose/img layer routing.
   - `test_configs.py`: hydra composes `shared_humans` with the multi groups
     and the CLI overrides in §2.2 without instantiating models; background
     list loads; legacy `low_memory` still composes.
   - `test_legacy_unchanged.py`: `Ros2Subscriber`/`Ros2Vis`/
     `Ros2MessagingService` default-kwarg behaviour (node names, no context
     kwargs) unchanged.
   Run: `cd common/rayfronts && python3 -m pytest tests -q -m "not cuda"`
   inside the robot container (WP-C harness), plus `-m cuda` with the GPU.

## 5. WP-C — AirStack integration + test harness (owner: agent C)

Files owned: `robot/ros_ws/src/global/planners/semantic_search_task/**`,
`robot/ros_ws/src/global/planners/search_baselines/scripts/offboard_compute.sh`,
`robot/docker/docker-compose.yaml` (offboard-compute env only),
`robot/docker/robot-base-docker-compose.yaml` (new env passthrough only),
`robot/ros_ws/src/perception/perception_bringup/launch/rayfronts_shared_check.launch.xml`
(optional), `.env` (append a commented RAYFRONTS/RAVEN block only — do NOT
flip any live value), `scripts/raven_rayfronts_tests.sh` (new),
`osmo/missions/raven_single_shared_test.yaml` (new, 2 robots),
`.agents/skills/run-raven-single-shared/SKILL.md` (new runbook).

1. semantic_search_task: §2.3 shared mode; `_filter_raven` regex gains
   `LVLM-guided`; `_cleanup_existing` skips rayfronts patterns in shared
   mode; nothing else changes. Unit tests in
   `semantic_search_task/test/test_shared_mode.py` (pure: the status-gate
   predicate, the regex, env parsing).
2. offboard_compute.sh + compose: §2.4. The script must `source
   /opt/ros/jazzy/setup.bash` ONLY inside the rayfronts branch (the detector
   /VLM servers stay ROS-free), export `FASTRTPS_DEFAULT_PROFILES_FILE=
   /root/AirStack/robot/ros_ws/src/fastdds.xml`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`,
   `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True`, `HYDRA_FULL_ERROR=1`,
   start encoder_server first, wait for its socket (timeout 300 s), then
   the mapping server; both under `/tmp/offboard/`. Compose: pass
   `START_RAYFRONTS_SERVER`, `RAYFRONTS_ROBOT_IDS`, `RAYFRONTS_CONFIG`,
   `RAYFRONTS_SRC`, `RAYFRONTS_COMPUTE_PROB`, `NUM_ROBOTS`,
   `RAVEN_LVLM`, `VLM_URL` through; robot containers get `RAYFRONTS_MODE`,
   `RAVEN_LVLM`, `VLM_URL` (bare-name passthrough like `DETECTOR_URL`).
3. `scripts/raven_rayfronts_tests.sh`: (a) host pure tests (uv) for
   raven_nav + rayfronts non-ros tests; (b) container tests: `docker run
   --rm` the robot image named by `.env` VERSION (derive the tag the way
   `robot/docker/docker-compose.yaml` does), mount the repo like the compose
   file does (`../../:/root/AirStack` equivalents: at least
   `robot/ros_ws`, `common/ros_packages`, `common/rayfronts`,
   `common/rayfronts_configs`, `common/fastdds.xml`), `--network none`
   (fallback: default bridge) with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`,
   `ROS_DOMAIN_ID=77`, no GPU by default (`--gpu` adds `--gpus all` and runs
   the `cuda` marks), `source /opt/ros/jazzy/setup.bash` + workspace
   `install/setup.bash` if present, `PYTHONPATH` with
   `/root/AirStack/common/rayfronts:/opt/rayfronts/rayfronts/csrc/build`,
   runs raven_nav pytest (all), rayfronts pytest, semantic_search_task
   pytest; prints a summary table; exit non-zero on any failure. Must not
   touch the running `isaac-sim` container. Verify it actually runs (the
   image exists locally: see frames_scene_testenv.md C.2) — run it against
   the CURRENT tree first so the harness is proven before A/B land; the
   raven_nav tests will change under you, that is expected.
4. Mission file + runbook: 2-robot local run: `airstack up offboard-compute`
   with `START_RAYFRONTS_SERVER=true START_VLM_SERVER=true`, then robots +
   gcs, the goal (query `person`, background list from
   `common/rayfronts_configs/background_humans.txt`, polygon = the 250 m
   plate, `voxel_min_cluster_size` 4, `voxel_score_threshold` per the
   softmax note), how to watch: `docker exec offboard-compute tail -f
   /tmp/offboard/rayfronts_mapping.log`, `ros2 topic echo
   /robot_1/rayfronts/status`, raven `debug/voxel_table`, `lvlm_request`.
   Include a `scripts/raven_monitor.sh` that tails the shared server log,
   both ravens' logs (`/tmp/raven_<robot>.log` per semantic_search_task's
   `_spawn` naming — verify), and echoes status/detections into
   `~/raven_previews/monitor_<ts>.log`.

## 6. WP-D — 250 m suburb tornado scene with people (owner: agent D)

Files owned: `scene_gen/config/presets/suburb_tornado_250.yaml` (new),
`scene_gen/tools/people_json_to_annotations.py` (new),
`_plans/raven_test_scene_runbook.md` (new), previews in `~/raven_previews/`.
Do NOT launch Isaac Sim, do not edit `.env`, do not touch launch scripts.

1. Preset: 250 x 250 m derived from `suburb_tornado.yaml` (500) with every
   plate-scaled knob re-derived the way `suburb_tornado_100.yaml` documents
   (width, block areas, collectors, min_gap, neighbourhood, grain, lot
   width, dead-ends, max_cars, z_scale) — interpolate between the 500 and
   100 values and justify each in comments; severity 0.82; people ON via the
   launch script's pass (`TOR_PEOPLE=1`, not the preset).
2. Dry runs on the host (no Isaac): `python3 scene_gen/tools/tornado_png.py
   --config suburb_tornado_250` → `~/raven_previews/suburb_tornado_250_track.png`;
   then find out whether `disaster.tornado_people.plan_people` can run on the
   compiled layout without a USD stage (read
   `simulation/isaac-sim/launch_scripts/suburb_tornado_launch_script.py`
   step 7b and `scene_gen/disaster/tornado_people.py`). If yes, produce the
   deterministic people plan for the default seed and write it to
   `~/raven_previews/suburb_tornado_250_people.json` + a PNG overlay of
   casualties coloured by `occlusion` on the track PNG. If not, say exactly
   what the build will write (`PEOPLE_JSON`) and how to pick spawns after
   the build.
3. Spawn proposal: robot_1 15–20 m from a casualty with `occlusion: none`
   (fully exposed), robot_2 15–20 m from one with a partial pattern
   (`legs`, `torso`, `midriff`, `upper_body`…, `covered_frac` 0.3–0.55),
   both spawn points on open ground (not inside a footprint, not on the
   centreline debris), heading (quaternion) facing the casualty, z per
   `.env` `DRONE_Z_M`. Output a ready `SPAWN_CONFIGS` JSON + the full `.env`
   block to apply (SCENE_CONFIG, ISAAC_SIM_SCRIPT_NAME, NUM_ROBOTS=2,
   RESULTS_SCENE, ARCH_DIR, TOR_PEOPLE, ENABLE_LIDAR, ZED_*), determined by
   reading how the tornado scene is flown for the dataset (freeze path +
   `example_multi_drone_scene_import.py` loading the frozen cell, or the
   tornado launch script spawning drones directly — read
   `.agents/skills/freeze-disaster-dataset/HURRICANE_RUNBOOK.md` and
   `.agents/skills/benchmark-disaster-dataset/SKILL.md`). Confirm
   `scene_gen/assets/archetypes_tornado` is complete for the styles the
   preset draws (it exists: `archetypes.json` + house_*.usd).
4. `people_json_to_annotations.py`: humans JSON → `raven_nav/annotations/
   <RESULTS_SCENE>.json` in the format `compare_to_groundtruth.py` reads
   (`class`, `bbox_world.center_xyz_m/size_xyz_m`), class `person`, box
   0.6 x 1.8 x 0.4 oriented by `body_axis_deg` (AABB of it), plus a
   `visibility`/`occlusion` passthrough field; unit test in
   `scene_gen/tests/test_people_json_to_annotations.py`.
5. Runbook: exact commands in order, what to look for in Isaac, and the GPU
   budget warning (one 16 GB card: Isaac currently 14.2 GB with the 250 m
   undamaged suburb loaded; shared rayfronts ≈ 3–4 GB, VLM 3B nf4 ≈ 3 GB —
   the user decides).

## 7. Review gates (lead)

- Diff every WP against the snapshot; re-run the harness; read the README
  and runbooks; verify the interface contract items by grep (topic names,
  param names, JSON keys) and by the integration tests.
- Report to the user: what changed, the four OG deviations, the GPU budget,
  and the go/no-go for launching the scene.
