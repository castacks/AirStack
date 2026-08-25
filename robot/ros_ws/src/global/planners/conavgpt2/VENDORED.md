# Vendored upstream code

Everything under `conavgpt2/vendor/` is a copy of the reference implementation of
Co-NavGPT2, kept as close to upstream as possible. Anything outside `vendor/` is
ours.

| | |
|---|---|
| Upstream | <https://github.com/ybgdgh/Co-NavGPT2.git> |
| Commit cloned | `891ba09959278b666ef01a7f8b6c64f8fe8436cc` ("check", 2025-07-03) |
| Paper | Co-NavGPT: Multi-Robot Cooperative Visual Semantic Navigation Using Vision Language Models — <https://arxiv.org/abs/2310.07937v3> |
| Authors | Bangguo Yu, Qihao Yuan, Kailai Li, Hamidreza Kasaei, Ming Cao (University of Groningen) |

**Upstream ships no LICENSE file** at that commit — there is nothing to preserve,
which is also why there is no `LICENSE.upstream` here. The repository is public on
GitHub with no stated terms. Treat the licensing as unresolved before any external
release of this package, and cite the paper when using it.

## What was copied

```
vendor/constants.py                    <- constants.py
vendor/arguments.py                    <- arguments.py
vendor/system_prompt.py                <- system_prompt.py
vendor/utils/{__init__,pose,general_utils,fmm_planner,mapping,
              explored_map_utils,visualization,
              detection_segmentation,chat_utils}.py   <- utils/
vendor/agents/{vlm_multi_agents,ros2_agents}.py       <- agents/
```

`vendor/agents/__init__.py` is new and empty — upstream has no such file, and
`find_packages()` needs it.

## What was deliberately NOT copied

| Upstream path | Why |
|---|---|
| `ros_multi_nav.py`, `ros_single_nav.py` | The entry points we replace. They contain the ZMQ actuation (`tcp://192.168.100.1:5557`) and hardcoded Unitree Go2 lidar↔camera extrinsics. `conavgpt2/conavgpt2_node.py` is their AirStack equivalent. Not vendoring them is what keeps ZMQ out of this package entirely. |
| `utils/vis_gui.py` | `ReconstructionWindow`, an open3d GUI in a second process. Cannot exist headless. The map render is published as a `sensor_msgs/Image` instead. |
| `main.py`, `main_vec.py`, `agents/vlm_agents.py`, `agents/ros2_single_agent.py`, `configs/`, `multi-robot-setting/` | The habitat/HM3D evaluation harness. Needs habitat-sim + habitat-lab. |
| `multi_lidar_icp.py` | Two-Go2 G-ICP registration for the real-robot setup. AirStack anchors robots through GPS/boot-ENU instead. |
| `utils/shortest_path_follower.py` | `import habitat_sim`. |
| `img/`, `README.md`, `requirements.txt`, `.gitignore` | Not code. |

## Every change made to a vendored file

Line counts are `diff` lines against the upstream commit.

### All files: import paths (mechanical, 22 lines total)

Upstream uses top-level absolute imports (`from utils.mapping import ...`,
`from constants import ...`, `from agents.vlm_multi_agents import ...`). Every one
was rewritten to `conavgpt2.vendor.*`. The alternative — putting `vendor/` on
`sys.path` so `utils`/`constants`/`agents` resolve — would have meant zero edits
but leaves three very generic top-level module names able to shadow, or be
shadowed by, anything else in a ROS 2 process.

`vendor/constants.py`, `vendor/system_prompt.py`, `vendor/utils/pose.py`,
`vendor/utils/general_utils.py`, `vendor/utils/fmm_planner.py` and
`vendor/utils/mapping.py` are **byte-identical to upstream**.

### `vendor/arguments.py` (4 lines)

`get_args()` → `get_args(argv=None)`, `parser.parse_args()` →
`parser.parse_args(argv)`. Upstream parses `sys.argv` at import; under
`ros2 run` that swallows `--ros-args` and aborts. The node calls `get_args([])`
for upstream's defaults and then overwrites fields from ROS parameters.

### `vendor/utils/chat_utils.py` (120 lines)

The biggest change, and the one that makes an offline open-weight VLM work.

1. **`client = OpenAI()` at module scope** → `CONFIG` + `get_client()` +
   `reset_client()`. Upstream constructs the client at import, which raises when
   no `OPENAI_API_KEY` is set — i.e. the whole package becomes unimportable on a
   machine with no OpenAI account. Now the client is built on first use, from
   `CONFIG.base_url` / `CONFIG.api_key` / `CONFIG.timeout` that
   `conavgpt2.vlm_client.configure()` fills in.
2. **`args = get_args()` at module scope** removed (same argparse problem);
   `args.num_agents` → `CONFIG.num_agents`.
3. **`model='gpt-4o'` hardcoded in the request** → `CONFIG.model`, and
   `chat_with_gpt4v(chat_history, gpt_type=args.gpt_type)` →
   `chat_with_gpt4v(chat_history, model=None)`. Upstream declared a `--gpt_type`
   flag and then ignored it.
4. **`cv2.imshow` / `cv2.waitKey` removed** from `get_all_candidate_maps()` and
   `get_all_candidate_full_maps()` — no X display in the container.
5. **`LAST_CALL` telemetry added** (new module-level dict + `_reset_last_call()`):
   per-round latency, `usage` token counts, attempt count, parse outcome, and any
   frontier id the model returned that was not offered. Upstream discarded the
   response object, so none of this was observable. Nothing about the request
   changes.
6. The two-robot hardcoded fallback `{"robot_0": "frontier_0", "robot_1":
   "frontier_0"}` now spans `CONFIG.num_agents`, and the length check
   `len(ground_json) == args.num_agents+1` became `>= CONFIG.num_agents` (upstream
   required exactly one extra key, the `"reason"` field, so a model that omitted
   the reason was rejected as malformed).
7. Connection errors now name `CONFIG.base_url` instead of saying "OpenAI API".
8. **`message_prepare`'s "two robots" is now `CONFIG.num_agents`.** Upstream
   writes the literal string `"two robots need to find a "` into the user turn,
   because the paper is a two-robot method. On any other team size that is a
   false statement to the model, and the model ACTS ON IT — observed live at
   `num_agents=1`, every round: *"Robot 0 is closer to the car... Robot 1 should
   explore the other frontier to cover the remaining area."* It reserves a
   frontier for a robot that does not exist, and so declines to send the one
   real robot to the frontier it just called best. The count and the plural are
   now taken from `CONFIG.num_agents`, which is already the team size everywhere
   else in the file.

**The request shape is untouched and is plain OpenAI vision**, verified by
building a real two-frontier message and inspecting it: `messages` =
`[{role: system, content: str}, {role: user, content: [{type: text}, {type:
image_url, image_url: {url: "data:image/jpeg;base64,..."}}, ...]}]` with
`response_format={"type": "json_object"}`, `temperature=0.1`, `max_tokens=100`.
Nothing OpenAI-proprietary. vLLM, llama.cpp and SGLang all accept it as-is;
`response_format: json_object` is the only field that needs server-side guided
decoding.

### `vendor/utils/explored_map_utils.py` (28 lines)

`build_full_scene_pcd()` had `voxel_down_sample(0.05)` and
`cluster_dbscan(eps=0.1, min_points=15)` hardcoded. Those are indoor values with a
4 m depth clip. At outdoor drone scale a 5 cm voxel over a 60 m scene is millions
of points and a 10 cm DBSCAN eps discards nearly all of them as noise. They are
now the module-level `SCENE_VOXEL_M`, `SCENE_DBSCAN_EPS_M` and
`SCENE_DBSCAN_MIN_POINTS`, which the node overwrites from ROS parameters, and
`SCENE_DBSCAN_MIN_POINTS <= 0` skips the clustering. Defaults are upstream's, so
an unconfigured import behaves exactly as before.

### `vendor/utils/visualization.py` (2 lines)

Import path only. `Visualize()` already returned its rendered image upstream, so
publishing it needed no change.

### `vendor/utils/detection_segmentation.py` (8 lines)

`SAM('mobile_sam.pt')` and `YOLO('yolov8l-world.pt')` resolve those bare filenames
relative to the CWD and otherwise download them from GitHub at first use. They now
read `CONAVGPT2_SAM_WEIGHTS` / `CONAVGPT2_YOLO_WORLD_WEIGHTS` from the environment
(set by the node from ROS parameters) and fall back to upstream's strings.

### `vendor/agents/vlm_multi_agents.py` (23 lines)

`import quaternion` (numpy-quaternion) guarded with `try/except ImportError`. It
is reachable only from `reset()` and `get_transform_matrix()`, both habitat-only
and both overridden downstream, and the package is not in the robot image. `yacs`
and `omegaconf` were checked and ARE in the image, so those imports were left
alone.

### `vendor/agents/ros2_agents.py` (28 lines)

1. `from tf_transformations import quaternion_matrix, translation_matrix` guarded.
   Verified unused in this file — upstream's `ros_multi_nav.py` was the only
   caller. Guarding it drops `tf-transformations` and `transforms3d` from the
   package's dependency list.
2. `self.device = "cuda:{}".format(self.args.gpu_id)` →
   `getattr(self.args, "device", None) or "cuda:{}".format(...)`, so a CPU-only
   or explicitly-pinned device is possible without editing further.

## Things upstream does that we deliberately work around from OUTSIDE `vendor/`

Preferring a wrapper to an edit, these are handled in `conavgpt2/airstack_agent.py`
and `conavgpt2/conavgpt2_node.py` rather than by patching upstream:

- **Frame convention.** `ROS_Agent.get_transform_matrix()` hardcodes habitat's
  `T_world_open3d`. `AirStackAgent` overrides it with an ENU→open3d rotation and
  drops upstream's per-robot `inv(T_init)` anchoring, so the grid is anchored at
  the shared map origin (a prerequisite for merging robots at all).
- **Depth units.** `_preprocess_depth()` divides by 1000 (16UC1 mm, RealSense).
  AirStack publishes 32FC1 metres with non-finite sky. Overridden.
- **Detector vocabulary.** `self.classes` is a hardcoded indoor list. YOLO-World is
  open-vocabulary, so the subclass calls `set_classes()` on the already-built model
  after construction instead of editing the list.
- **PNG spam.** `_visualize()` writes a PNG per agent per step whenever
  `args.print_images` is set — but that same flag is what makes upstream compute
  the annotated FPV and the map render at all. The subclass keeps the flag on and
  flips it to 0 around the `super()` call unless `save_debug_images` is set.
- **Unbounded accumulation.** `agent.point_sum` grows forever and the node
  re-rasterises every point ever seen on every tick, including a Python `for` loop
  over the whole cloud for the top-view z-buffer. `Global_Map_Proc` already
  accumulates into its own grids, so the node feeds it only the current tick's
  cloud and clears `point_sum` (parameter `incremental_mapping`, default true).
- **Grid overflow.** `Map_Extraction()` indexes the grid without clamping, so a
  point exactly on the `+halfsize` boundary lands one cell past the end. The node
  crops the cloud to `halfsize - one cell` first.
- **Frontier count cap.** `Frontier_Det()` stops at `i == 5`, i.e. at most 6
  frontiers, which bounds the VLM prompt at 6 images. Left as upstream has it —
  worth knowing, because prompt size and latency scale with that count.
