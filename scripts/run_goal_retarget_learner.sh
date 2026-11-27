#!/usr/bin/env bash
# Start the online goal-retargeting learner (drone_soccer's revision 7 plan update) on the
# host, beside the robot container.
#
# The learner needs drone_soccer's `gpu` conda environment (torch, stable-baselines3,
# jax, casadi), none of which the robot container has, so it runs here and exchanges
# files with the policy commander under robot/bags/goal_retarget.
#
# Usage (after launching the commander with policy_commander_drone3_goal_retarget.yaml):
#   scripts/run_goal_retarget_learner.sh --barrier-dataset data_20261119
#   scripts/run_goal_retarget_learner.sh --barrier-corners x1 y1 x2 y2 x3 y3 x4 y4
#   scripts/run_goal_retarget_learner.sh --run-dir robot/bags/goal_retarget/<run> ...
#   scripts/run_goal_retarget_learner.sh --barrier-dataset ... --goal-step-rule mean_displacement --goal-step-gain 0.3
# Every option is passed to scripts/goal_retarget_learner.py (see --help).
#
# Environment overrides:
#   GPU_PYTHON     python of the gpu env   (default: miniforge3's, else miniconda3's)
#   DRONE_SOCCER   host drone_soccer checkout (default: next to this AirStack checkout)

set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -z "${GPU_PYTHON:-}" ]; then
    GPU_PYTHON="$HOME/miniforge3/envs/gpu/bin/python"
    [ -x "$GPU_PYTHON" ] || GPU_PYTHON="$HOME/miniconda3/envs/gpu/bin/python"
fi
DRONE_SOCCER="${DRONE_SOCCER:-$(cd -- "$REPO_ROOT/.." && pwd)/drone_soccer}"

if [ ! -x "$GPU_PYTHON" ]; then
    echo "gpu env python not found: $GPU_PYTHON (set GPU_PYTHON)" >&2
    exit 1
fi
if [ ! -d "$DRONE_SOCCER/drone_soccer" ]; then
    echo "drone_soccer checkout not found: $DRONE_SOCCER (set DRONE_SOCCER)" >&2
    exit 1
fi

# JAX (the surrogate model of the bptt goal step) must not grab the whole GPU.
export XLA_PYTHON_CLIENT_PREALLOCATE=false
export PYTHONPATH="$DRONE_SOCCER${PYTHONPATH:+:$PYTHONPATH}"
cd "$DRONE_SOCCER"
exec "$GPU_PYTHON" "$REPO_ROOT/scripts/goal_retarget_learner.py" \
    --drone-soccer "$DRONE_SOCCER" "$@"
