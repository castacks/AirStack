#!/usr/bin/env bash
# Verifies offboard_compute.sh's OFFBOARD_COMPUTE_GPU pin actually reaches a
# launched server's environment, and — the regression this exists to catch —
# that leaving it UNSET does not accidentally define CUDA_VISIBLE_DEVICES as
# an empty string (which CUDA treats as "no devices", not "unset"; see the
# script's own header comment and simulation/isaac-sim/docker/docker-
# compose.yaml's identical trap on the sim side).
#
# Pure host-side shell test: fakes `python3` and `nvidia-smi` on PATH so no
# GPU, no docker, no ROS is needed. Never touches a running container.
#
#   bash test_offboard_compute_gpu_pin.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$HERE/offboard_compute.sh"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

FAKE_BIN="$WORK/bin"
mkdir -p "$FAKE_BIN"

# Fake python: dumps CUDA_VISIBLE_DEVICES (present-or-not, not just its value)
# to a fixed file, then exits — offboard_compute.sh backgrounds it (`nohup ...
# &`), so it must not hang.
cat > "$FAKE_BIN/python3" <<'EOF'
#!/usr/bin/env bash
if [ -z "${CUDA_VISIBLE_DEVICES+set}" ]; then
  echo "CUDA_VISIBLE_DEVICES=<unset>" >> "$OFFBOARD_TEST_ENV_DUMP"
else
  echo "CUDA_VISIBLE_DEVICES=$CUDA_VISIBLE_DEVICES" >> "$OFFBOARD_TEST_ENV_DUMP"
fi
exit 0
EOF
chmod +x "$FAKE_BIN/python3"

cat > "$FAKE_BIN/nvidia-smi" <<'EOF'
#!/usr/bin/env bash
echo "fake-gpu, 0 MiB, 0 MiB"
exit 0
EOF
chmod +x "$FAKE_BIN/nvidia-smi"

run_case() {   # run_case <label> <OFFBOARD_COMPUTE_GPU value or "unset">
  local label="$1" gpu="$2"
  local logdir dump
  logdir="$WORK/log_$label"; mkdir -p "$logdir"
  dump="$WORK/dump_$label.txt"; : > "$dump"

  local env_args=(
    "PATH=$FAKE_BIN:$PATH"
    "OFFBOARD_LOG_DIR=$logdir"
    "OFFBOARD_PYTHON=python3"
    "OFFBOARD_TEST_ENV_DUMP=$dump"
    "START_DETECTOR_SERVER=false"
    "START_ITM_SERVER=false"
    "START_VLM_SERVER=true"
    "START_RAYFRONTS_SERVER=false"
    "CONAVGPT2_VLM_MODEL=dummy"
  )
  if [ "$gpu" != "unset" ]; then
    env_args+=("OFFBOARD_COMPUTE_GPU=$gpu")
  fi

  # offboard_compute.sh ends in `tail -F`, which never returns — `timeout`
  # cuts it off once the fake python (launched in the first couple seconds)
  # has had time to run and write its dump.
  timeout 5 env "${env_args[@]}" bash "$SCRIPT" >/dev/null 2>&1 || true

  if [ ! -s "$dump" ]; then
    echo "FAIL [$label]: fake python3 never ran (dump file empty) — vlm_server never started"
    return 1
  fi
  cat "$dump"
}

echo "=== case: OFFBOARD_COMPUTE_GPU=1 ==="
out_pinned="$(run_case pinned 1)"
echo "$out_pinned"
if ! echo "$out_pinned" | grep -qx "CUDA_VISIBLE_DEVICES=1"; then
  echo "FAIL: pinning OFFBOARD_COMPUTE_GPU=1 did not set CUDA_VISIBLE_DEVICES=1 for the server process"
  exit 1
fi

echo "=== case: OFFBOARD_COMPUTE_GPU unset (regression: must stay UNSET, not empty) ==="
out_unset="$(run_case unset unset)"
echo "$out_unset"
if ! echo "$out_unset" | grep -qx "CUDA_VISIBLE_DEVICES=<unset>"; then
  echo "FAIL: leaving OFFBOARD_COMPUTE_GPU unset must leave CUDA_VISIBLE_DEVICES completely unset in the server's env (an empty string means 'no devices' to CUDA, not 'unset')"
  exit 1
fi

echo "PASS: OFFBOARD_COMPUTE_GPU pin reaches the launched server; unset introduces no CUDA_VISIBLE_DEVICES at all"
