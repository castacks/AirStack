#!/bin/bash
# Submit with sbatch; this job survives terminal/laptop disconnects.
# Usage: sbatch psc_rrm_cosmos_visual.sbatch <image|video> <media-ref> <catalog.json>
#        <task-id> <episode-id> <observed-s> <received-s> [teacher-snapshot.json]
# Example media/capture and teacher labels must already be in persistent PSC storage.
#SBATCH --job-name=rrm-vlm
#SBATCH --partition=GPU-shared
#SBATCH --gres=gpu:h100-80:1
#SBATCH --time=00:20:00
#SBATCH --output=/ocean/projects/eng260004p/oabolade/physical-ai/runs/rrm/slurm-vlm-%j.out
#SBATCH --error=/ocean/projects/eng260004p/oabolade/physical-ai/runs/rrm/slurm-vlm-%j.err

set -euo pipefail

if (( $# < 7 || $# > 8 )); then
  echo "usage: $0 <image|video> <media-ref> <catalog.json> <task-id> <episode-id> <observed-s> <received-s> [teacher-snapshot.json]" >&2
  exit 2
fi

media_path=$1
media_ref=$2
catalog=$3
task_id=$4
episode_id=$5
observed_s=$6
received_s=$7
teacher=${8:-}

: "${PROJECT:?PSC PROJECT environment variable is required}"
root="${RRM_PERSIST_ROOT:-$PROJECT/physical-ai}"
run_dir="$root/runs/rrm/visual-grounding/${SLURM_JOB_ID}"
mkdir -p "$run_dir"
exec >>"$run_dir/console.log" 2>&1

export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
model_path="${RRM_MODEL_PATH:-$root/checkpoints/cosmos/hf/models--nvidia--Cosmos-Reason2-8B/snapshots/a9fae2cf89dc64db96b12860417f0eb403013bb9}"
python_env="$root/src/cosmos-reason2/.venv/bin/python"
runner="$root/src/rrm/scripts/rrm_cosmos_visual_grounding.py"

media_flag=--image
case "$media_path" in
  *.mp4|*.avi|*.mov|*.mkv) media_flag=--video ;;
esac

args=("$media_flag" "$media_path" --media-ref "$media_ref" --catalog "$catalog"
      --task-id "$task_id" --episode-id "$episode_id"
      --state-revision "$episode_id/vlm-$SLURM_JOB_ID"
      --observed-monotonic-s "$observed_s" --received-monotonic-s "$received_s"
      --model-path "$model_path" --output "$run_dir/result.json")
if [[ -n "$teacher" ]]; then
  args+=(--teacher "$teacher")
fi

PYTHONPATH="$root/src/rrm" "$python_env" "$runner" "${args[@]}"
