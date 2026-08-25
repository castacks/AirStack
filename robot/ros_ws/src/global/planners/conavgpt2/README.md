# conavgpt2

Upstream [Co-NavGPT2](https://github.com/ybgdgh/Co-NavGPT2) wrapped as an AirStack
global planner.

Co-NavGPT2 builds its **own** occupancy and explored map straight from RGBD —
no semantic map, no rayfronts — merges every robot's point cloud into one grid,
extracts frontiers from it, renders a synthesised top view with one numbered
frontier highlighted per image, and asks a vision-language model to assign one
frontier to each robot. The selected frontier becomes a goal point, which this
package flies to via `droan_gl`.

This replaces `conavgpt_baseline` + `raven_nav/behaviors/conavgpt_behavior.py`,
which reimplemented the *method* but took frontiers from rayfronts' semantic map.
`conavgpt_baseline` is left in place; retire it once this one runs.

## What is upstream and what is ours

| | |
|---|---|
| **Upstream, unmodified-as-possible** | `conavgpt2/vendor/` — mapping, frontier detection, FMM planning, YOLO-World+MobileSAM detection, the prompt, the chat call, the renders. See [`VENDORED.md`](VENDORED.md) for the commit and every change made to those files. |
| **Ours** | `conavgpt2/conavgpt2_node.py` (ROS wiring), `conavgpt2/airstack_agent.py` (frame + depth + vocabulary overrides), `conavgpt2/vlm_client.py` (backend config + preflight), `conavgpt2/vlm_server.py` (the OpenAI-compatible VLM the node queries), `conavgpt2/vlm_loadgen.py` (its concurrency sweep), the package plumbing. |

Three things had to change, and only these three:

1. **Actuation.** Upstream PUBs `speedctl speed|0.3|0.0|0.0` strings over ZMQ to
   hardcoded Unitree Go2 IPs (`tcp://192.168.100.1:5557`). Here the goal becomes a
   `task_msgs/action/NavigateTask` goal on `/{robot}/tasks/navigate`, so droan_gl
   flies it with obstacle avoidance, and the same path is published on
   `/{robot}/global_plan` for Foxglove and the gossip node. **There is no ZMQ
   anywhere in this package.**
2. **The GUI.** `utils/vis_gui.py` (`ReconstructionWindow`, open3d GUI in a second
   process) is not vendored. The map render is published as `sensor_msgs/Image`.
3. **The VLM endpoint.** Upstream builds `OpenAI()` at import against
   api.openai.com. Here it is any OpenAI-compatible server.

## Running it

```bash
# in the robot container
bws --packages-select conavgpt2 && sws

# 1) an OpenAI-compatible VLM must be reachable first — this package ships one:
#      /opt/lvlm-venv/bin/python -m conavgpt2.vlm_server --port 8000
#    (see "The server", below, for the flags and the measured numbers)
# 2) then
ros2 launch conavgpt2 conavgpt2.launch.xml \
     num_robots:=1 \
     goal_name:=person \
     vlm_base_url:=http://localhost:8000/v1
```

or directly, matching how `semantic_search_task` spawns the other baselines:

```bash
/opt/lvlm-venv/bin/python -m conavgpt2.conavgpt2_node --ros-args \
    -p use_sim_time:=true -p num_robots:=1 -p results_dir:=/root/results
```

What to look at, in order:

| Topic | Why |
|---|---|
| `/conavgpt2/vlm_prompt_image` | the exact frames the VLM was shown, tiled. If an assignment makes no sense, look here first. |
| `/conavgpt2/map_image` | upstream's merged render: obstacle/explored/visited/goal on the left, synthesised top view on the right. **This is the first thing to eyeball** — it says whether the occupancy map has any structure at all. |
| `/{robot}/conavgpt2/agent_image` | the FPV with YOLO-World detections drawn on. |
| `/conavgpt2/round_stats` | one JSON string per VLM round (see Instrumentation). |
| `/{robot}/global_plan` | the path also sent as the NavigateTask goal. |

## Topic and parameter mapping

Every topic is a parameter. Defaults are the AirStack names, verified against
`simulation/isaac-sim/.../spawn_zed_camera.py` and the robot bringup:

| Parameter | Default | Upstream equivalent |
|---|---|---|
| `rgb_topic_template` | `/{robot}/sensors/front_stereo/left/image_rect` | `/robot{i}/camera/color/image_raw` |
| `depth_topic_template` | `/{robot}/sensors/front_stereo/left/depth_ground_truth` | `/robot{i}/camera/aligned_depth_to_color/image_raw` |
| `camera_info_topic_template` | `/{robot}/sensors/front_stereo/left/camera_info` | `/robot{i}/camera/color/camera_info` |
| `odom_topic_template` | `/{robot}/odometry_conversion/odometry` | tf `camera_init_1` → `body_{i}` |
| `navsat_topic_template` | `/{robot}/interface/mavros/global_position/raw/fix` | — (only for `frame_mode: global_enu`) |
| `navigate_action_template` | `/{robot}/tasks/navigate` | **ZMQ PUB, removed** |
| `global_plan_topic_template` | `/{robot}/global_plan` | — |
| `map_image_topic` | `/conavgpt2/map_image` | open3d GUI window |
| `vlm_image_topic` | `/conavgpt2/vlm_prompt_image` | `cv2.imshow("candidate_i", ...)` |
| `agent_image_topic_template` | `/{robot}/conavgpt2/agent_image` | `cv2.imshow("episode_.. agent_..")` |
| `round_stats_topic` | `/conavgpt2/round_stats` | — |

RGB is `rgb8` 480x300, depth is `32FC1` in **metres** with non-finite sky, both
published **BEST_EFFORT** (a RELIABLE subscriber silently receives nothing — this
node subscribes BEST_EFFORT). Intrinsics come off `camera_info` at runtime and the
derived hFOV/vFOV is logged once per robot; upstream's `--hfov 79.0` (a RealSense
D455) is never used.

Frames come from TF by default (`pose_source: tf`, `map → camera_left`, which
walks the real URDF chain). If the lookup fails — which it always will for a robot
on a different ROS domain, since AirStack's TF frames are unprefixed — it falls
back to odometry plus the static extrinsics in `camera_offset_xyz`. Those defaults
were checked against `iris_with_sensors.pegasus.robot.urdf`: the composed
`base_link → camera_left` rotation equals the plain FLU→RDF matrix to 9e-6 rad,
and the translation is `(0.209, 0.060, -0.037)` m.

The full parameter list with defaults is [`config/conavgpt2.yaml`](config/conavgpt2.yaml).

## Ground robot vs drone: what had to be re-scaled

Upstream is tuned for a Unitree Go2 with a level RealSense in a house. The
perception model carries over unchanged — depth is unprojected to a cloud and
projected down into a 2D grid, so the "top view" the VLM reads is **synthesised**,
never a camera image, and a forward-facing ZED gives the same kind of input. The
**scale** does not carry over.

| Parameter | Upstream | Here | Why |
|---|---|---|---|
| `map_extent_m` | 24 m (`map_size_cm=2400`) | **500** | 24 m is a house. Our test plat is 500 m and wildfire is 500–1600 m; at 24 m every frontier is an edge of the map. Set per scene. |
| `map_cells` | 480 | **480** | Not free to change: upstream's `write_number()` stamps the robot markers on the VLM's candidate images in 480-pixel coordinates. Off 480 and the markers land in the wrong place. The node warns if the ratio does not come out at 480. |
| `map_resolution_cm` | 5 cm | **derived** (~104 cm at 500 m) | Follows from the two above. |
| `obstacle_min_z_m` / `obstacle_max_z_m` | `map_height_cm=130`, a 1.3 m band on the camera | **2 / 15 m AGL** | 1.3 m centred on a quadruped's camera means "what I would walk into". From 15–40 m AGL that distinction is meaningless — everything observed is below it, so the whole world reads as free or as obstacle depending on which side the ground falls. The aerial equivalent is **ground vs structure**: bare terrain near z=0 is free, anything standing out of it is an obstacle. Buildings and canopy here top out around 15 m. Ground must stay OUT of the band, because a seen cell with nothing in the band is what upstream calls free, and free cells are what frontiers grow from. |
| `hfov` | 79.0 hardcoded | **from `camera_info`** | Never assume the ZED's optics. |
| `camera_pitch_rad` | n/a (level D455) | **0.0** | This scene mounts the ZED level: `camera_rotation_offset=[0,0,0]` in `suburb_wildfire_drones_launch_script.py`, and the URDF chain adds no pitch either. This parameter describes a **physical mount**, not a preference — setting it non-zero without a matching mount rotates the point cloud into the wrong place. Note `lvlm_baseline` defaults it to 0.1745 rad, which does **not** match this scene. |
| `scene_voxel_m` | 0.05 | **0.25** | 5 cm over a 60 m outdoor scene is millions of points per frame. |
| `scene_dbscan_*` | eps 0.1 m, 15 pts | **disabled** | A 10 cm eps on sparse outdoor returns discards nearly everything as noise. |
| `sem_threshold` | 0.85 | **0.5** | 0.85 is a close-range indoor confidence. |
| `warmup_steps` | 30 (robot spins in place first) | **0** | The drone has no reason to spin up an initial map. |

## The VLM backend

Not OpenAI. Any OpenAI-compatible `/v1` endpoint works because upstream's request
is plain OpenAI vision — verified by building a real two-frontier message and
inspecting it:

```
messages = [ {role: system, content: "<the frontier-assignment prompt>"},
             {role: user,   content: [ {type: text, text: "... find a person"},
                                       {type: image_url,
                                        image_url: {url: "data:image/jpeg;base64,..."}},
                                       ... one per frontier ... ]} ]
response_format = {"type": "json_object"},  temperature = 0.1,  max_tokens = 100
```

Nothing OpenAI-proprietary. `response_format: json_object` is the only field that
needs server support (vLLM implements it with guided decoding).

| Parameter | Default | Env fallback |
|---|---|---|
| `vlm_base_url` | `http://localhost:8000/v1` | `OPENAI_BASE_URL` |
| `vlm_model` | `Qwen/Qwen2.5-VL-3B-Instruct` | `CONAVGPT2_VLM_MODEL` |
| `vlm_api_key` | `EMPTY` | `OPENAI_API_KEY` |
| `vlm_preflight` | `true` | — |

With `vlm_preflight` on, the node calls `/v1/models` at startup and **raises with
the serve command in the message** if the endpoint is unreachable or does not
serve the requested model. Without that, a wrong `base_url` shows up only as
upstream's silent "everyone gets frontier_0" fallback, twenty minutes in.

### The server: `conavgpt2/vlm_server.py`

Not vLLM. vLLM is not in the robot image and the AWQ weights were never pulled, so
the server is a single file in this package that speaks the same OpenAI subset
directly out of `transformers` + `bitsandbytes`:

| Endpoint | Why it exists |
|---|---|
| `GET /v1/models` | `vlm_client.preflight()` calls it and refuses to start if the id it wants is absent. `--served-model-name` must equal the node's `vlm_model`. |
| `POST /v1/chat/completions` | the round. Multimodal `content` parts, `response_format`, `temperature`, `max_tokens`. Non-streaming; `stream: true` is rejected rather than ignored. |
| `GET /metrics` | count, mean/p50/p95 latency, queue wait, prefill/decode split, token counts, throughput, max concurrency observed, live VRAM, per-client breakdown. |
| `POST /metrics/reset` | zeroes the window between sweep levels; the load generator calls it. |
| `GET /health` | readiness plus the quantisation proof (see below). |

```bash
# in the robot container, in the venv the other baselines already use
/opt/lvlm-venv/bin/python -m conavgpt2.vlm_server --port 8000
```

It takes ~10 s to come up from a warm HF cache and prints, before it binds the
port, both the quantisation report and the VRAM it took. Then:

```bash
ros2 launch conavgpt2 conavgpt2.launch.xml num_robots:=1 goal_name:=person \
     vlm_base_url:=http://localhost:8000/v1
```

#### Flags

| Flag | Default | What it is for |
|---|---|---|
| `--model` | `Qwen/Qwen2.5-VL-3B-Instruct` | HF repo id or local path. **This is the knob for OSMO** — on a 48 GB card set `Qwen/Qwen2.5-VL-7B-Instruct` and `--quantization none`. |
| `--served-model-name` | = `--model` | id reported by `/v1/models`. Set it when the weights path differs from the id the node asks for, or preflight fails. |
| `--host` / `--port` | `0.0.0.0` / `8000` | |
| `--device` | `cuda:0` | pinned via `device_map={"": device}` on purpose: accelerate's `"auto"` would silently offload layers to CPU when Isaac has the VRAM, turning a 2.7 s round into a 40 s one with no error to explain it. **Which physical card is chosen with `CUDA_VISIBLE_DEVICES`, not this flag** — the mission exports `CUDA_VISIBLE_DEVICES=$CONAVGPT2_VLM_GPU` for the server process alone so `cuda:0` here is physical GPU 1 and nothing in this process tree can reach Isaac's card by accident. See `osmo/workflows/airstack-mission-2gpu.yaml`. |
| `--quantization` | `nf4` | `nf4` (bitsandbytes 4-bit NF4 + double quant), `int8`, or `none`. |
| `--compute-dtype` | `float16` | dequantised compute dtype. |
| `--attn-implementation` | `sdpa` | `eager` / `flash_attention_2` if ever needed. |
| `--max-pixels` | `501760` (= 640 visual tokens/image) | **the cap that stops Qwen2.5-VL's dynamic resolution exploding the prompt.** One visual token = a 28x28 block, so pixels/784 is the per-image token budget. Measured effect below. |
| `--min-pixels` | `12544` (16 tokens) | floor, so a tiny render is not upscaled into nothing. |
| `--max-images` | `6` | rejects a bigger prompt with 400. Upstream's `Frontier_Det()` stops at `i == 5`, so 6 is the hard ceiling; the equivalent of vLLM's `--limit-mm-per-prompt image=6`. |
| `--default-max-tokens` | `128` | used only when the client omits `max_tokens`. Upstream always sends 100. |
| `--max-tokens-cap` | `1024` | ceiling on a client's `max_tokens`, so one request cannot monopolise the serialised GPU. |
| `--json-prefill` / `--no-json-prefill` | on | opens the assistant turn with `{` when `response_format` is `json_object`. There is no guided decoding here, so this is the cheap substitute: it removes the "Sure, here is..." preamble a 3B model otherwise puts in front of the object. A balanced-brace extractor runs on top of it either way, because upstream feeds the content straight to `ast.literal_eval`. |
| `--prep-workers` | `4` | threads for base64 decode, JPEG decode and the image processor. CPU only — the GPU stays serialised whatever this is. |
| `--metrics-jsonl` | `/tmp/conavgpt2/vlm_server_requests.jsonl` | one JSON object appended per request, line-buffered so a killed run keeps its records. `''` disables. Same directory the node writes `conavgpt2_rounds.jsonl` to, so a run's two halves land together. |
| `--metrics-window` | `2000` | requests retained for percentiles. |
| `--max-body-mb` | `32` | 413 above this. A 6-frontier round is 1.06 MB, so the cap only catches a runaway client. |
| `--api-key` | `$VLM_SERVER_API_KEY` or none | if set, requires `Authorization: Bearer <key>`. |
| `--warmup` / `--no-warmup` | on | one throwaway generate at startup so the first real round is not also paying for kernel autotune. |
| `--log-level` | `info` | one line per request: client, images, KiB, token counts, and the queue/prep/prefill/decode split. |

#### One GPU, one forward pass at a time

A single model on one card cannot run two forward passes concurrently — letting
FastAPI interleave them corrupts state and returns nonsense. So: decode and
preprocess run in `--prep-workers` threads, then **every generate takes one
`asyncio.Lock`, FIFO, and runs on a single-threaded executor**. Time spent blocked
on that lock is `queue_wait_s`, reported per request in a non-standard `x_timings`
block on the response and aggregated at `/metrics`. That is the number the
scaling question is actually about.

`usage` carries real counts from the tokeniser — `prompt_tokens` is the post-expansion
input length, and `prompt_tokens_details.image_tokens` splits out how much of it
was pixels. The node reads `usage` per round; nothing here is estimated or zero.

### Measured, on this box

RTX 5070 Ti, 16 GB (15.45 GiB visible), **co-tenanted with a running Isaac Sim that
held ~8.9 GiB throughout**. `disaster-dataset-robot-desktop-1`, `/opt/lvlm-venv`,
transformers 4.57.6, bitsandbytes 0.50.1, torch 2.9.1+cu130.

**Read the co-tenancy caveat before the numbers.** 8.9 GiB of Isaac plus 3.3 GiB of
VLM took this card to 14.5 of 16.3 GiB, and that pairing is what made Isaac's RTX
render graph fail every frame — the incident written up in
`osmo/workflows/airstack-mission-2gpu.yaml`. So during these runs Isaac was
*resident* but not *rendering*, and the VLM effectively had the SMs to itself. That
is also the configuration the deployment now mandates (Isaac alone on GPU 0, the
VLM on GPU 1), so these numbers are representative of how it will actually run —
but they are **not** a measurement of a healthy Isaac and a VLM sharing one card,
because that configuration does not work at all.

**Load and VRAM.** 9.8–11.3 s from a warm HF cache (the first pull is 7.1 GB of
bf16 weights). Quantisation verified by walking the module tree, not assumed:
**414 `Linear4bit` modules, 1 unquantised `Linear`, `quant_type: nf4`** —
1.602 GiB of 4-bit params plus 0.584 GiB left in fp16 (embeddings, lm_head, norms,
the vision merger). **2.24 GiB torch-allocated, 2.43 GiB reserved, 2.47 GiB
attributable to the process** including the CUDA context; `nvidia-smi` showed
3.3 GiB for the process once cuBLAS workspaces were up. Peak reserved across the
whole sweep, 15 requests deep: **2.91 GiB** — flat, because the GPU is serialised,
so KV cache and vision activations are only ever one request's worth. The VLM itself
never came close to OOM at any concurrency level; what broke under co-tenancy was
Isaac, quietly, in its renderer.

**Budget ~3.5 GiB of VRAM per instance** (2.9 GiB peak reserved plus the CUDA
context), and give it a card Isaac is not on.

**A round.** 6 x 480x480 JPEG at 133–139 KiB = 814 KiB of image, **1.06 MB request
body** — the sizing this package was written against, confirmed.

**Token counts, measured not estimated:**

```
prompt_tokens 2189  =  1734 visual (6 x 289, one per 480x480 render)
                    +   455 text (the system prompt + "N robots need to find a person")
completion_tokens 72–100, mean 77
```

**Latency at concurrency 1:** 2.74 s mean — prep 0.05–0.10 s, **prefill 0.65 s**,
**decode 2.04 s at 38 tok/s**. Decode dominates, and decode is proportional to the
*answer* length, not the image count.

#### The concurrency curve

`vlm_loadgen` fires N CoNavGPT-shaped rounds simultaneously and waits for the last
one, because that is the real failure: every drone's round lands at once and the
last drone is the one flying stale.

```bash
/opt/lvlm-venv/bin/python -m conavgpt2.vlm_loadgen --levels 1,2,5 --rounds 3
```

| clients | lat mean | p50 | p95 | queue mean | queue max | infer | **makespan** | req/s | json ok |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 2.74 | 2.75 | 2.82 | 0.00 | 0.00 | 2.67 | **2.74** | 0.364 | 3/3 |
| 2 | 3.92 | 3.89 | 5.26 | 1.28 | 2.56 | 2.54 | **5.19** | 0.385 | 6/6 |
| 5 | 8.19 | 8.02 | 13.74 | 5.35 | 11.37 | 2.69 | **13.62** | 0.367 | 15/15 |
| 10 | 15.22 | 15.06 | 27.26 | 12.22 | 25.00 | 2.74 | **27.69** | 0.361 | 20/20 |
| 15 | 21.36 | 21.43 | 38.54 | 18.34 | 37.23 | 2.63 | **39.84** | 0.377 | 28/30 |

Seconds. Makespan is 1.00x / 1.89x / 4.97x / 10.1x / 14.5x of the single-client
time — **exactly linear, which is the correct behaviour and the whole finding**.
`infer_s` is flat at 2.6–2.7 s at every level, so nothing is thrashing: the GPU is
doing the same work, one request at a time, and everything above the first client
is pure queueing. Throughput is pinned at **0.36–0.39 rounds/s** no matter how many
clients ask.

**What that means for 1 vs 5 drones.** Note first that Co-NavGPT is *centralised*:
one call assigns the whole team, so five drones **in one team** are still one
request every 30 s and this question does not arise. The concurrency case is five
**independent** `conavgpt2` processes — which is what AirStack actually gives you,
one container and one `ROS_DOMAIN_ID` per robot. For that:

- **5 drones on one 3B instance: comfortable.** 13.6 s mean / 14.1 s worst makespan
  against a 30 s `round_period_s` leaves 55% headroom, and no drone ever waited
  more than 11.4 s for the GPU.
- **The ceiling is ~10 drones** at 30 s (27.7 s makespan) and it is a cliff, not a
  slope: at 15 the makespan is 39.8 s, every round overruns its period, and the
  node starts logging the overrun WARNING every time.
- **Halve those numbers before believing them.** The 3B model returns an
  out-of-range frontier id often enough to trigger upstream's retry loop — seen in
  the end-to-end test, where a 6-frontier round drew `frontier_6` and cost two
  server calls. A round that retries once costs 2x, which moves the 30 s ceiling
  from ~10 drones to ~5.
- **Recommendation: one shared instance, not one per drone**, up to about 5 drones
  per instance. Per-drone instances buy nothing on one card — they would contend
  for the same SMs and each cost another ~3.5 GiB, and the co-tenancy incident above
  says a card with something else already on it is exactly what not to do. Scale by
  adding GPUs, not by adding instances to a GPU. On OSMO's 48 GB cards the right use
  of the headroom is a bigger model, not a second copy of this one: 7B unquantised
  is ~16 GB and fixes the frontier-id errors that are currently costing a retry.

#### `--max-pixels` earns its place

Same 6-image round, same JPEG bytes, only the render's pixel dimensions changed:

| render | `--max-pixels 501760` (default, 640 tok/img) | Qwen stock `1003520` (1280 tok/img) |
|---|---|---|
| 480x480 (what `map_cells: 480` produces) | 2189 tok, 2.70 s | 2189 tok, 3.30 s |
| 960x960 | 4205 tok, 3.42 s | **7391 tok, 5.85 s** |
| 1440x1440 | 4205 tok, 4.17 s | **7805 tok, 5.81 s** |

At 480x480 the cap does not bind and both columns are the same prompt — the 0.6 s
difference there is output length (94 vs 74 tokens at 38 tok/s), not the cap. The
cap only starts mattering above 480, and then it matters a lot: uncapped, a 960 px
render nearly doubles the prompt and adds 71% to the round. This is the guard for
`get_all_candidate_full_maps()` (960x480 combined renders) and for anyone raising
`map_cells`. Prefill scales with it too; a 7800-token prompt is already 5.5 s of
GPU per round, so ~5 drones would overrun 30 s on prompt size alone.

#### What the load generator is

`conavgpt2/vlm_loadgen.py`. It imports the **real** vendored system prompt so the
text token count is real, synthesises 6 frontier top views at `map_cells` pixels
and binary-searches JPEG quality onto the measured 135 KiB/image, and sends the
exact shape `message_prepare()` + `chat_with_gpt4v()` send — `json_object`,
`temperature 0.1`, `max_tokens 100`. Every level resets `/metrics` first and reads
it back after, so the table carries both client-side latency and server-side queue
wait.

| Flag | Default | |
|---|---|---|
| `--base-url` / `--model` | `http://localhost:8000/v1` / `Qwen/Qwen2.5-VL-3B-Instruct` | preflighted against `/v1/models` before anything is sent |
| `--levels` | `1,2,5` | concurrency levels = simultaneous drones |
| `--rounds` | `3` | repeats per level; every repeat fires all clients at once |
| `--images` / `--image-kib` / `--image-size` | `6` / `135.0` / `480` | the payload shape. Keep `--image-size` equal to `map_cells` or the token counts are fiction |
| `--goal` / `--num-robots` | `person` / `2` | the text `message_prepare()` puts in front of the images |
| `--max-tokens` / `--temperature` | `100` / `0.1` | upstream's values |
| `--round-period-s` | `30.0` | the node's cadence; the summary says fits/does not fit against it |
| `--out` | — | per-request results plus the summary as JSONL |
| `--no-warmup` | — | skip the throwaway first round |

#### Two things this measurement exposed about upstream

- **`max_tokens=100` is too tight for this model.** 3 of 78 rounds hit the cap
  mid-`reason` string, and a truncated object is unparseable — `ast.literal_eval`
  fails, upstream burns a retry. The `reason` field is what costs the tokens.
  Either raise `max_tokens` to ~200 or drop `reason` from the required output.
- **The 3B model's frontier-id grounding is shaky.** It returned `frontier_6` for a
  6-frontier round (valid ids are 0–5). Upstream catches this and retries, so it
  costs latency rather than a bad waypoint, but it is the strongest argument for
  7B on OSMO.

#### Not measured

- **Sustained load.** Every number above is a burst: N clients, then idle. Thermal
  throttling and Isaac's own GPU load varying over a 10-minute mission are not in
  these numbers.
- **Contention with a *working* Isaac.** Isaac held ~8.9 GiB throughout but its
  render graph was failing every frame, so it was not doing the GPU work a healthy
  sim does. `infer_s` was flat to within 0.2 s across all five concurrency levels,
  which says the VLM was never starved here — but that is a measurement of a broken
  co-tenancy, not a good one. On the 2-GPU layout the question does not arise;
  there is no 1-GPU layout in which it can be answered favourably.
- **The 7B model, and unquantised.** Neither fits on this card next to Isaac. The
  OSMO numbers have to be taken on OSMO. Expect roughly 2x the per-round time from
  the parameter count alone, i.e. the ~5-drone ceiling above becomes ~2–3 unless the
  card is faster in proportion.
- **Sustained multi-hour drift.** The JSONL is there so a real mission answers this;
  the sweeps above total ~4 minutes of GPU work.
- **Request-shape guard rails end to end.** `--max-images`, the 413 body cap and the
  `stream: true` rejection are implemented and unit-obvious but were not exercised
  against the live server — the container stack was torn down before that check
  ran.

## Instrumentation

The architecture is centralised: **one VLM call assigns the whole team**, so cost
does not scale with robot count. It scales with **frontier count**, because the
prompt carries one rendered top view per candidate. Measured here: each candidate
JPEG is ~135 KiB, ~180 KiB once base64'd, so a 6-frontier round is roughly **1.1 MB
of image payload in a single request**. The failure mode that matters is a round
taking longer than its period, because then every robot is flying to a stale
assignment.

Every round is recorded three ways:

- **JSONL** at `{results_dir or dump_location}/conavgpt2_rounds.jsonl`, one object
  per round.
- **`/conavgpt2/round_stats`** (`std_msgs/String`, JSON, depth 10 so every round
  survives into an mcap).
- **one INFO line**:
  `[conavgpt2] round 7 | 5 frontiers, 1 robot | 3.2 s (server 2.9) | 1840+96 tok | ok`

Fields: `round` (monotonic, so drops show as gaps), `num_robots`,
`num_frontiers`, `num_images`, `image_payload_bytes`, `build_s` (rendering +
base64, our side), `server_s` (the request itself), `call_s`, `total_s`,
`attempts`, `prompt_tokens` / `completion_tokens` / `total_tokens` (read from the
response's `usage` block, not estimated), `parse`
(`ok` / `retried` / `bad_json` / `invalid_frontier_id` / `too_few_robots` /
`failed`), `invalid_ids` (frontier ids the model returned that were not offered),
`round_period_s` (actually achieved, i.e. since the previous round started),
`round_budget_s`, `overran`, `overrun_s`, plus the assignment and the raw response.

`round_period_s` (default 30 s) is the sweepable knob. An overrun is a **WARNING**
naming the overrun in seconds, the frontier count and the payload size — never a
silent slip. Setting `round_period_s: 0.0` reverts to upstream's cadence of one
round every `num_local_steps` ticks.

## Multi-robot

Upstream runs one process for the whole team, and that is kept: `num_robots` makes
this node subscribe to and command every robot. **It only works where the peers'
camera topics are actually visible.** AirStack gives each robot its own container
*and* its own `ROS_DOMAIN_ID` (domain == robot index), and each robot's `map`
frame is anchored at its **own** takeoff GPS — `robot_1`'s (0,0) and `robot_2`'s
(0,0) are different places.

So:

- `num_robots: 1` (default) — works as-is, and is what the wildfire mission
  launches.
- `num_robots: N` — needs three things:
  1. the sensor topics bridged onto one domain. Note only the **right** eye's
     depth currently crosses to the GCS domain
     (`onboard_all/config/dds_router.yaml`), so bridging left depth is a
     prerequisite;
  2. `frame_mode: global_enu`, which subscribes to each robot's NavSatFix, derives
     `boot_enu = gps_to_enu(first_fix) - odom_at_that_instant` exactly as
     `raven_nav` does, merges the clouds in global ENU, and drops the offset back
     off before sending each robot's NavigateTask in its own local `map` frame;
  3. **`map_origin_xy` set to the middle of the search area.** In global ENU, 0,0
     is the GPS origin, and the wildfire drones spawn at about `(302, -144)` — a
     grid centred on 0,0 would crop the entire cloud away. In `local` mode 0,0 is
     the robot's own takeoff point and the default is correct.

## Dependencies

Checked with `docker run --rm --entrypoint bash <robot image> -c 'python3 -c "import X"'`
against `airlab-docker.andrew.cmu.edu/airstack/airstack:v0.19.0-alpha.1-disaster-dataset_robot-x86-64_dev`.

**Was missing and required:**

| Package | Note |
|---|---|
| `scikit-fmm` | `skfmm`, the FMM planner. May build from source on cp312. |
| `open3d` | needs **>= 0.19** for Python 3.12. |
| `ultralytics` | YOLO-World + MobileSAM. |
| `supervision==0.19.0` | pinned by upstream's API use (`ColorPalette.default()`, `BoundingBoxAnnotator`, the 6-tuple `Detections` iteration). Later versions renamed all three. |
| `openai` | the client only; the server is local. |

**Present in `/opt/lvlm-venv` as of 2026-08-25**, imported and version-checked in a
running `disaster-dataset-robot-desktop-1`: `openai 3.3.1`, `supervision`,
`transformers 4.57.6`, `bitsandbytes 0.50.1`, `accelerate 1.14.0`,
`fastapi 0.141.1`, `uvicorn 0.52.4`, `qwen-vl-utils`, `torch 2.9.1+cu130`,
`torchvision 0.24.1+cu130`, `huggingface_hub`. The last seven are what
`conavgpt2/vlm_server.py` needs and are the reason it does not need vLLM.
`scikit-fmm`, `open3d` and `ultralytics` were not re-checked in that session —
only the VLM path was exercised.

The Qwen weights are **not** baked into the image. First start pulls 7.1 GB into
`/root/.cache/huggingface`, which is bind-mounted from `robot/docker/cache/` and so
survives container replacement; on an OSMO pod it is a cold pull every time and
should be a prefetch step alongside the RADIO one.

**Missing but NOT needed** — the two imports that pulled them in are dead in the
files we vendor, and are guarded: `numpy-quaternion` (habitat-only paths) and
`tf-transformations` / `transforms3d` (upstream's `ros_multi_nav.py` used them,
`ROS_Agent` does not; this node does its own quaternion maths in numpy).

**Already present:** `numpy 1.26.4`, `scipy`, `cv2 4.6.0`, `scikit-image 0.26.0`,
`torch 2.9.1+cu130`, `torchvision`, `PIL`, `yaml`, `omegaconf`, `yacs`, `requests`,
`matplotlib`, `message_filters`, `cv_bridge`, `rclpy`, `tf2_ros`. `task_msgs` is
built by colcon, so `bws && sws` first.

### Proposed install path

`/opt/lvlm-venv` is the precedent — created at
`robot/docker/Dockerfile.robot:414-422` with `--system-site-packages` so it reuses
the system torch/numpy/rclpy/cv_bridge and only holds what would otherwise perturb
rayfronts. `semantic_search_task` launches the other baselines with
`/opt/lvlm-venv/bin/python -m <pkg>.<module>`. Same treatment here:

```dockerfile
RUN /opt/lvlm-venv/bin/pip install --no-cache-dir \
      scikit-fmm "open3d>=0.19" ultralytics "supervision==0.19.0" openai
```

Three things to watch when that runs:

- **`supervision` will pull `opencv-python`** (it declares it as a dependency; pip
  cannot see the apt-installed `python3-opencv` the ROS stack uses). A second cv2
  in the venv can shadow the one `cv_bridge` was built against. Prefer
  `--no-deps` for supervision and add only what it actually imports
  (`numpy`, `matplotlib`, `defusedxml`, `pyyaml`, `requests`, `scipy` — all present).
- **`ultralytics`** declares `torch>=1.8`, which system torch 2.9.1 satisfies, so
  pip should leave it alone — but confirm it did not pull a CPU wheel.
- **weights**: `yolov8l-world.pt` (~90 MB) and `mobile_sam.pt` (~40 MB)
  auto-download from GitHub on first use. In an offline container, pre-bake them
  and set `yolo_world_weights` / `mobile_sam_weights`.

Nothing was added to the Dockerfile as part of this work.

## NEEDS TUNING against a real run

Nothing below can be settled without watching `/conavgpt2/map_image`:

- `obstacle_min_z_m` / `obstacle_max_z_m` (2/15 m AGL). The single most likely
  thing to be wrong. If the map comes out all-obstacle, the band is catching the
  ground; all-free, and it is above everything.
- `depth_max_m` (60 m). A level camera at 15–40 m AGL sees ground only at long
  range, so too small a clip maps nothing and too large a clip maps noise.
- `flight_altitude_m` (15 m) — every waypoint is emitted at this altitude; the map
  is 2D and has no opinion about height.
- `scene_voxel_m` / `frontier_threshold_points` — frontier size in cells changes
  completely at ~1 m/cell instead of 5 cm/cell.
- `sem_threshold` (0.5) and `detection_classes` for YOLO-World on aerial imagery.
- `plan_period_s` (1 Hz). `Map_Extraction()` runs a Python loop over the tick's
  cloud for the top-view z-buffer; that cost is unmeasured here.
- `round_period_s` (30 s) versus measured VLM latency — that is exactly what the
  instrumentation is for.
- Whether droan_gl is happy being re-goaled at `goal_change_threshold_m` (5 m). It
  rejects a second goal while one is active, so this node cancels first; the
  cancel/resend cadence has not been exercised against a live server.
