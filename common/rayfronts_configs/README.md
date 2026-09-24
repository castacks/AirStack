# `common/rayfronts_configs/`

Repo-side RayFronts configs. Two ways they reach the server:

1. **Baked into the robot image.** `robot/docker/Dockerfile.robot`'s
   `runtime-rayfronts` stage copies this whole directory *on top of*
   `/opt/rayfronts/rayfronts/configs/`, so anything here overlays or adds to
   the package's own configs.
2. **Mounted at run time.** When the server runs out of the repo
   (`RAYFRONTS_SRC=/root/AirStack/common/rayfronts`) the package configs come
   from the repo checkout and this directory has to be added explicitly:
   `--config-dir /root/AirStack/common/rayfronts_configs`.

Group configs (`dataset/`, `encoder/`, `messaging_service/`, `vis/`,
`encoder_server/`) live in the **package** — `common/rayfronts/rayfronts/
configs/` — so they are found either way. Only root configs and data files
live here.

## Files

| File | What it is |
|---|---|
| `low_memory.yaml` | The single-robot config the existing per-robot launch file runs (`rayfronts.launch.xml`, `--config-name low_memory`). Unchanged. |
| `shared_humans.yaml` | The shared multi-robot config. One map, N robots, human-sized targets. |
| `background_humans.txt` | ~33-label background vocabulary for a tornado-hit suburb. |
| `background_humans_min.txt` | 6-label, OG-shaped alternative. See the threshold note below. |
| `dataset/ros2isaacsim.yaml` | Single-robot Isaac topics. Unchanged. |
| `low_memory(backup.yaml` | Pre-existing stray backup file, untouched. |

## Running the shared server

Two processes, both on `offboard-compute`, both on system python3:

```bash
# 1. the model, once
python3 -m rayfronts.encoder_server \
        encoder=radseg \
        encoder_server.socket=/tmp/rayfronts/encoder.sock \
        encoder_server.warmup=[480,480]

# 2. the map, once, spanning every robot's ROS domain
python3 -m rayfronts.multi_robot_mapping_server \
        --config-name shared_humans \
        --config-dir /root/AirStack/common/rayfronts_configs \
        dataset.robot_ids=[1,2] \
        encoder=client encoder.socket=/tmp/rayfronts/encoder.sock
```

`PYTHONPATH` must contain the rayfronts source and the compiled extension
(`/opt/rayfronts/rayfronts/csrc/build`); `FASTRTPS_DEFAULT_PROFILES_FILE` and
`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` must be set so the server can see the
robot containers' domains across the docker bridge. `offboard_compute.sh`
(owned by the AirStack-integration work package) does all of that.

## What `shared_humans.yaml` changes vs `low_memory.yaml`

| Knob | low_memory | shared_humans | Why |
|---|---|---|---|
| `dataset` group | `ros2isaacsim` | `multi_ros2isaacsim` | N robots, one per ROS domain, round-robined into one mapper; poses shifted from each robot's own `map` frame into a shared world frame. |
| `vis` / `messaging_service` groups | `ros` | `multi_ros` | Map outputs are published to every robot's prefix, each copy shifted back into that robot's local frame. |
| `encoder` group | `radseg` (in-process) | `client` | The model lives in `rayfronts.encoder_server`; one GPU copy serves every mapping server, and restarting the mapper does not reload it. Pass `encoder=radseg` to go back to in-process. |
| `querying.text_query_mode` | unset (null; the launch file passed `prompts` on the CLI) | `prompts` | So the server cannot start in a state where the first query raises `Invalid query type`. |
| `querying.query_file` | null | `background_humans.txt` | `compute_prob=True` means scores are a softmax across columns; with no background labels every voxel scores ~1.0 for the only query. |
| `mapping.max_pts_per_frame` | 1000 | **4000** | Sampling is uniform over the whole 480x480 frame. A casualty covering ~2% of the image gets ~20 of 1000 points, split across the ~4 voxels it occupies. 4000 lets a single-voxel target survive one pass. |
| `mapping.sem_pruning_thresh` | 5 | **1** | `prune_semantic_voxels` deletes any semantic voxel with occupancy log-odds `<=` this. A briefly-seen human never clears 5. 1 keeps it while still discarding single-observation noise (0 would keep everything). |
| `mapping.sem_pruning_period` | 32 | **8** | Prune 4x more often so a phantom (moving survivor, mis-registered frame) is corrected in ~8 frames instead of lingering for 32 as a false detection. |
| `batch_size` | 1 (default) | 1 (explicit) | Frames come from different robots; they must never be collated into one batch. |
| `status_period_s` | n/a | 1.0 | New: rate of `/robot_i/rayfronts/status`. |

Unchanged on purpose: `vox_size 0.50` (finer multiplies VRAM across a 250 m
plate; a standing adult is already ~1 voxel in plan, ~4 tall),
`max_rays_per_frame 500`, `max_empty_pts_per_frame 3000` (those are *free*
space -- raising them alongside the occupied points would just re-balance the
log-odds back), `fronti_subsampling 5` / `fronti_subsampling_min_fronti 9`
(that is the *where to explore next* layer, not the detection layer),
`compile/amp/depth_limit`, `feat_compressor pca out_dim 100`.

## Threshold warning (background list length)

Scores are a **softmax across every query column**. The original RAVEN
`background.txt` had four labels; `raven_nav`'s `score_threshold` 0.95 (rays)
and `voxel_score_threshold` 0.98 (voxels) were tuned against that. With the 33
labels in `background_humans.txt` a confident `person` voxel typically peaks
well below 0.9, so those thresholds must be lowered or the Ray and Voxel
behaviours will never fire.

Pick one before a run and say which:

* `background_humans.txt` (33 labels) + **lowered** raven thresholds, or
* `background_humans_min.txt` (6 labels) + the OG thresholds.

Either way, look at real values first:

```bash
ros2 topic echo /robot_1/rayfronts/msg_serv/voxels_sim/q0_person --once
```

## New topics

Per robot, in that robot's own `map` frame, on that robot's own domain:

| Topic | Type | Note |
|---|---|---|
| `/robot_i/rayfronts/msg_serv/voxels_sim/{all,q{k}_{label}}` | PointCloud2 | Unchanged field layout: `x,y,z,sim_k` / `x,y,z,sim`. |
| `/robot_i/rayfronts/msg_serv/rays_sim/{all,q{k}_{label}}` | PointCloud2 | Unchanged: `x,y,z,theta,phi,sim_k` / `...,sim`. |
| `/robot_i/rayfronts/msg_serv/frontiers` | PointCloud2 | Unchanged. |
| `/robot_i/rayfronts/msg_serv/new_text_query` | String | Unchanged; one label per message. |
| `/robot_i/rayfronts/msg_serv/guiding_queries` | String (JSON list) | **New.** That robot's full current LVLM guiding list. RELIABLE + TRANSIENT_LOCAL, depth 1. A guiding label survives while any robot lists it; a label added through `new_text_query` is never deleted. |
| `/robot_i/rayfronts/status` | String (JSON) | **New.** 1 Hz, RELIABLE + TRANSIENT_LOCAL, depth 1: `{"robot","domain","anchored","boot_enu","frames_robot","frames_total","queries","vox_count","ray_count","ts"}`. |
| `/robot_i/rayfronts/...` (vis layers) | various | Unchanged names, one copy per robot. |

## Frames

Each robot's odometry is in ITS OWN `map` frame, anchored at that robot's spawn
point. The dataset measures `boot_enu_i = mean(gps_to_enu(fix) - odom)` over
`anchor_samples` fixes (`anchor_mode: gps`; `static` takes it from
`robot_offsets_xy`, `none` disables it), adds the **xy** to each pose *while it
is still FLU*, and only then rotates into RDF for the mapper. z is left alone:
every robot's z is AGL and `boot_enu`'s z is an MSL-datum difference.

Publishing runs the same shift backwards, in RDF: for `flu -> rdf`
(`T = [[0,-1,0],[0,0,-1],[1,0,0]]`) a world FLU offset `(bx, by, bz)` is
`(-by, -bz, bx)` in RDF, so world -> local is `(+by, +bz, -bx)`. With `bz`
forced to 0 that is `(by, 0, -bx)`. `tests/test_frame_shift.py` derives this
against `geometry3d.get_coord_system_transform` in both directions rather than
taking it on trust.
