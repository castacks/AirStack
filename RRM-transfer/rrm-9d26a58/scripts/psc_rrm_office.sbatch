#!/bin/bash
# Actual pinned Cosmos inference; survives disconnect. No robot execution here.
# Usage: sbatch -A eng260004p scripts/psc_rrm_office.sbatch <frozen-image.png>
#SBATCH --job-name=rrm-office
#SBATCH --partition=GPU-shared
#SBATCH --gres=gpu:h100-80:1
#SBATCH --time=00:20:00
#SBATCH --output=/ocean/projects/eng260004p/oabolade/physical-ai/runs/rrm/office-%j.out
#SBATCH --error=/ocean/projects/eng260004p/oabolade/physical-ai/runs/rrm/office-%j.err
set -euo pipefail
[[ $# == 1 ]] || { echo 'Usage: sbatch -A eng260004p psc_rrm_office.sbatch <image.png>' >&2; exit 2; }
: "${PROJECT:?PSC PROJECT is required}"
: "${SLURM_JOB_ID:?Submit using sbatch on a GPU allocation}"
rrm_root="${RRM_PERSIST_ROOT:-$PROJECT/physical-ai}"
rrm_source="${RRM_SOURCE_ROOT:-$rrm_root/src/rrm}"
rrm_run="$rrm_root/runs/rrm/office/$SLURM_JOB_ID"
mkdir -p "$rrm_run"
exec > >(tee "$rrm_run/console.log") 2>&1
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 PYTHONUNBUFFERED=1
rrm_python="$rrm_root/src/cosmos-reason2/.venv/bin/python"
rrm_model="$rrm_root/checkpoints/cosmos/hf/models--nvidia--Cosmos-Reason2-8B/snapshots/a9fae2cf89dc64db96b12860417f0eb403013bb9"
cp -- "$1" "$rrm_run/input.png"
cp -- "$rrm_source/examples/office_visual_eval/navigation_context.json" "$rrm_run/input.json"
cp -- "$rrm_source/examples/office_visual_eval/scene_manifest.json" "$rrm_run/scene_manifest.json"
sha256sum "$rrm_source/rrm/cosmos_reason2.py" "$rrm_source/scripts/rrm_cosmos_reason2.py" > "$rrm_run/source.sha256"
"$rrm_python" -c 'import torch, torchvision, transformers; assert torch.cuda.is_available(); print(torch.__version__, torchvision.__version__, transformers.__version__, torch.cuda.get_device_name(0))'
PYTHONPATH="$rrm_source" "$rrm_python" "$rrm_source/scripts/rrm_cosmos_reason2.py" \
  --model-path "$rrm_model" --input "$rrm_run/input.json" --image "$rrm_run/input.png" \
  --output "$rrm_run/result.json" --max-new-tokens 1024
cd "$rrm_run"
sha256sum input.json input.png scene_manifest.json result.json > SHA256SUMS
echo "Completed inference bundle: $rrm_run (not yet a flight result)"
