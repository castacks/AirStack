#!/usr/bin/env bash
# Start the SHARED model servers on the offboard-compute container and stay up.
#
# Why a container of its own: the models these serve are stateless — one frame
# in, an answer out, no per-robot history — so one instance can serve every
# robot, and three planner processes each loading their own copy of YOLO +
# MobileSAM is three times the VRAM for identical answers. It runs the ROBOT
# IMAGE (which already has torch, ultralytics, supervision and the
# /opt/lvlm-venv serving stack) but NOT the autonomy stack: no bringup, no
# robot name, no ROS domain, no DDS. The planners reach it over plain HTTP on
# the docker bridge, so nothing here touches the DDS router.
#
# Everything is env-gated so phase 2 (the ITM scorer and the generative VLM
# moving here too) is a compose variable, not a new service.
#
#   START_DETECTOR_SERVER=true   YOLO + MobileSAM        :8200
#   START_ITM_SERVER=false       BLIP-2 ITM (VLFM)       :8100
#   START_VLM_SERVER=false       generative VLM (gpt)    :8000
#   START_RAYFRONTS_SERVER=false RayFronts encoder_server +
#                                multi_robot_mapping_server (RAYFRONTS_MODE=
#                                shared — build plan §2.4). Unlike the three
#                                servers above, this ONE needs to be a real
#                                ROS 2 participant per robot domain, so it is
#                                the only branch below that sources ROS.
#
# Logs: /tmp/offboard/<name>.log, also tailed to the container's stdout so
# `docker logs offboard-compute` shows what happened.
set -u

LOG_DIR="${OFFBOARD_LOG_DIR:-/tmp/offboard}"
mkdir -p "$LOG_DIR"

# The servers are ordinary python modules in the mounted workspace; no ROS is
# sourced here on purpose (this container must not join a ROS domain), so the
# package is reached by PYTHONPATH instead.
WS="${OFFBOARD_WS:-/root/AirStack/robot/ros_ws}"
export PYTHONPATH="$WS/src/global/planners/search_baselines:${PYTHONPATH:-}"
PY="${OFFBOARD_PYTHON:-/opt/lvlm-venv/bin/python}"

echo "[offboard-compute] python=$PY"
echo "[offboard-compute] PYTHONPATH=$PYTHONPATH"
nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv,noheader 2>&1 | \
  sed 's/^/[offboard-compute] gpu: /'

start() {   # start <name> <port> <module> [args...]
  local name="$1" port="$2"; shift 2
  local log="$LOG_DIR/$name.log"
  echo "[offboard-compute] starting $name on :$port -> $log"
  nohup "$PY" -m "$@" > "$log" 2>&1 &
  echo $! > "$LOG_DIR/$name.pid"
}

if [ "${START_DETECTOR_SERVER:-true}" = "true" ]; then
  start detector_server "${DETECTOR_PORT:-8200}" \
    search_baselines.detector_server \
    --port "${DETECTOR_PORT:-8200}" \
    --device "${OFFBOARD_DEVICE:-cuda:0}" \
    --yolo-weights "${CONAVGPT2_YOLO_WORLD_WEIGHTS:-/root/.cache/conavgpt2_weights/yolov8l-world.pt}" \
    --sam-weights "${CONAVGPT2_SAM_WEIGHTS:-/root/.cache/conavgpt2_weights/mobile_sam.pt}" \
    --weights-dir "${CONAVGPT2_ULTRALYTICS_WEIGHTS_DIR:-/root/.cache/conavgpt2_weights}" \
    --classes "${DETECTOR_CLASSES:-}" \
    --metrics-jsonl "$LOG_DIR/detector_requests.jsonl"
fi

if [ "${START_ITM_SERVER:-false}" = "true" ]; then
  start itm_server "${ITM_PORT:-8100}" \
    search_baselines.itm_server \
    --port "${ITM_PORT:-8100}" \
    --device "${OFFBOARD_DEVICE:-cuda:0}" \
    --metrics-jsonl "$LOG_DIR/itm_requests.jsonl"
fi

if [ "${START_VLM_SERVER:-false}" = "true" ]; then
  start vlm_server "${VLM_PORT:-8000}" \
    search_baselines.vlm_server \
    --port "${VLM_PORT:-8000}" \
    --device "${OFFBOARD_DEVICE:-cuda:0}" \
    --model "${CONAVGPT2_VLM_MODEL:-Qwen/Qwen2.5-VL-7B-Instruct}" \
    --quantization "${CONAVGPT2_VLM_QUANT:-nf4}" \
    --compute-dtype bfloat16 \
    --metrics-jsonl "$LOG_DIR/vlm_requests.jsonl"
fi

# ── RAYFRONTS_MODE=shared: ONE rayfronts server for every robot ─────────────
#
# semantic_search_task/node.py (RAYFRONTS_MODE=shared) does not spawn
# rayfronts itself; instead this container runs it ONCE — an encoder_server
# (the GPU-resident encoder, reached over a Unix socket by a ClientEncoder in
# the mapping process) plus a multi_robot_mapping_server that round-robins
# every robot's RGB/depth/pose (each robot on its own ROS_DOMAIN_ID) into one
# SemanticRayFrontiersMap and republishes outputs back onto each robot's own
# domain under that robot's existing topic names — see the build plan's
# §1 architecture diagram and §2.2 contract.
#
# THIS is the one branch of this script that needs a real ROS 2 participant
# (rclpy contexts per robot domain) — source ROS ONLY here; the three
# detector/ITM/VLM servers above are plain HTTP and stay ROS-free on purpose
# (see the module docstring above).
if [ "${START_RAYFRONTS_SERVER:-false}" = "true" ]; then
  source /opt/ros/jazzy/setup.bash

  # Every robot's participants live on the shared docker bridge network but
  # on their OWN ROS_DOMAIN_ID (see frames_scene_testenv.md §B) — SUBNET
  # discovery (vs. this image's default) is what lets this container's rclpy
  # contexts find them at all, and the same LARGE_DATA transport profile
  # every other container uses is what lets ~600KB RGB frames actually
  # arrive intact.
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/root/AirStack/robot/ros_ws/src/fastdds.xml}"
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  # Long mapping runs fragment the CUDA allocator (OOM with tens of GiB
  # "reserved but unallocated"); expandable segments avoids that — same knob
  # the per-robot rayfronts.launch.xml sets (see interface_contract.md).
  export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"
  export HYDRA_FULL_ERROR=1

  # rayfronts_cpp (the compiled OpenVDB/nanobind extension — mapping/
  # semantic_ray_frontiers_map.py's `import rayfronts_cpp`) is produced only
  # at IMAGE BUILD time under /opt/rayfronts (Dockerfile.robot's
  # runtime-rayfronts stage; rayfronts_internals.md §9). It lives there
  # regardless of which source tree RAYFRONTS_SRC below points python's
  # `-m`/cwd resolution at, so it is always added to PYTHONPATH.
  export PYTHONPATH="/opt/rayfronts/rayfronts/csrc/build:${PYTHONPATH:-}"

  RAYFRONTS_SRC="${RAYFRONTS_SRC:-/root/AirStack/common/rayfronts}"
  RAYFRONTS_CONFIG="${RAYFRONTS_CONFIG:-shared_humans}"
  # Overlay directory for the TOP-LEVEL launch config (shared_humans.yaml,
  # background_humans.txt) — WP-B commits these under common/rayfronts_configs/,
  # a directory the Dockerfile only COPIES into /opt/rayfronts/rayfronts/configs/
  # at BUILD time (Dockerfile.robot; rayfronts_internals.md §9). Running from
  # a LIVE-mounted RAYFRONTS_SRC therefore needs this as an explicit Hydra
  # search path; running from the image-baked fallback below does not (the
  # overlay is already merged into /opt/rayfronts/rayfronts/configs/).
  RAYFRONTS_CONFIG_DIR="${RAYFRONTS_CONFIG_DIR:-/root/AirStack/common/rayfronts_configs}"
  RAYFRONTS_SOCK="${RAYFRONTS_SOCK:-/tmp/rayfronts/encoder.sock}"
  mkdir -p "$(dirname "$RAYFRONTS_SOCK")"

  if [ -n "${RAYFRONTS_ROBOT_IDS:-}" ]; then
    RF_ROBOT_IDS_CSV="$(echo "$RAYFRONTS_ROBOT_IDS" | tr -d '[:space:]')"
  else
    RF_ROBOT_IDS_CSV="$(seq -s, 1 "${NUM_ROBOTS:-1}")"
  fi

  # No compose file in this repo mounts common/rayfronts or
  # common/rayfronts_configs into /root/AirStack today (grep-verified against
  # robot-base-docker-compose.yaml — it mounts ../ros_ws, ../../common/
  # ros_packages, ../../common/fastdds.xml, ../../stacks, and a few
  # devcontainer files, nothing under common/rayfronts*). Until that mount
  # exists (a compose-only change, out of scope for this script), the default
  # RAYFRONTS_SRC will not resolve — fall back to the image-baked copy rather
  # than crash, and skip --config-dir since /opt/rayfronts already has WP-B's
  # overlay merged in at whatever point the image was last built (see the
  # run-raven-single-shared runbook: this fallback will NOT reflect
  # uncommitted rayfronts/rayfronts_configs edits without a rebuild).
  RF_CONFIG_DIR_ARGS=(--config-dir "$RAYFRONTS_CONFIG_DIR")
  if [ ! -d "$RAYFRONTS_SRC" ]; then
    echo "[offboard-compute] RAYFRONTS_SRC=$RAYFRONTS_SRC not found in this container" >&2
    echo "[offboard-compute] falling back to /opt/rayfronts (image-baked; see run-raven-single-shared SKILL.md)" >&2
    RAYFRONTS_SRC=/opt/rayfronts
    RF_CONFIG_DIR_ARGS=()
  elif [ ! -d "$RAYFRONTS_CONFIG_DIR" ]; then
    echo "[offboard-compute] RAYFRONTS_CONFIG_DIR=$RAYFRONTS_CONFIG_DIR not found — omitting --config-dir" >&2
    RF_CONFIG_DIR_ARGS=()
  fi

  echo "[offboard-compute] rayfronts: src=$RAYFRONTS_SRC config=$RAYFRONTS_CONFIG" \
    "config_dir=${RAYFRONTS_CONFIG_DIR:-<none>} robot_ids=[$RF_ROBOT_IDS_CSV]" \
    "compute_prob=${RAYFRONTS_COMPUTE_PROB:-True} sock=$RAYFRONTS_SOCK"

  # Same cwd + `python -m` resolution rayfronts.launch.xml relies on today
  # (cwd="/opt/rayfronts" there; rayfronts_internals.md §9) — bare system
  # python3, NOT $PY (the /opt/lvlm-venv used by the detector/ITM/VLM servers
  # above), matching Dockerfile.robot's comment that rayfronts runs on system
  # python3 precisely so the lvlm venv doesn't need rayfronts' deps.
  start_rayfronts() {   # start_rayfronts <name> [args...]
    local name="$1"; shift
    local log="$LOG_DIR/${name}.log"
    echo "[offboard-compute] starting $name -> $log"
    ( cd "$RAYFRONTS_SRC" && nohup python3 -m "$@" > "$log" 2>&1 & \
      echo $! > "$LOG_DIR/${name}.pid" )
  }

  start_rayfronts rayfronts_encoder \
    rayfronts.encoder_server \
    encoder=radseg \
    encoder_server.socket="$RAYFRONTS_SOCK"

  echo "[offboard-compute] waiting up to 300s for encoder_server socket $RAYFRONTS_SOCK"
  rf_deadline=$((SECONDS + 300))
  until [ -S "$RAYFRONTS_SOCK" ]; do
    if [ "$SECONDS" -ge "$rf_deadline" ]; then
      echo "[offboard-compute] encoder_server socket never appeared after 300s" \
        "— starting the mapping server anyway; see $LOG_DIR/rayfronts_encoder.log" >&2
      break
    fi
    sleep 1
  done

  start_rayfronts rayfronts_mapping \
    rayfronts.multi_robot_mapping_server \
    --config-name "$RAYFRONTS_CONFIG" \
    "${RF_CONFIG_DIR_ARGS[@]}" \
    "dataset.robot_ids=[$RF_ROBOT_IDS_CSV]" \
    encoder=client \
    encoder.socket="$RAYFRONTS_SOCK" \
    querying.compute_prob="${RAYFRONTS_COMPUTE_PROB:-True}"
fi

# Weights load before any port is bound (all three HTTP servers do this on
# purpose, so a port that answers means READY and a planner's preflight
# cannot race the load). Tail keeps the container alive AND surfaces the logs.
sleep 2
tail -n +1 -F "$LOG_DIR"/*.log
