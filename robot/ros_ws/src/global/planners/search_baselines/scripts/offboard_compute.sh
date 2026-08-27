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

# Weights load before any port is bound (all three servers do this on purpose,
# so a port that answers means READY and a planner's preflight cannot race the
# load). Tail keeps the container alive AND surfaces the logs.
sleep 2
tail -n +1 -F "$LOG_DIR"/*.log
