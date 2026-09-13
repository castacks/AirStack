#!/usr/bin/env bash
# Provision the A10G instance for RRM-1.
#
# Target: AWS g5.2xlarge — A10G 24 GB VRAM, 8 vCPU, 32 GiB RAM, Ubuntu 24.04.
# Run in order; each stage is independently re-runnable.
#
#   ./scripts/setup_instance.sh preflight # run LOCALLY first — costs nothing
#   ./scripts/setup_instance.sh check     # report what is present, change nothing
#   ./scripts/setup_instance.sh base      # drivers, CUDA, ROS 2 Jazzy, python deps
#   ./scripts/setup_instance.sh models    # download weights to EBS
#   ./scripts/setup_instance.sh verify    # smoke tests that must pass before Isaac
#
# Isaac Sim itself is NOT installed here — it needs an NVIDIA account and an
# interactive licence acceptance. See docs/architecture.md §9.

set -euo pipefail

# Weights must land on storage that SURVIVES stop/start, because stopping when idle
# is the main cost control. What that means depends on the host:
#   EC2   : EBS volume — instance-store NVMe is wiped on stop
#   Brev  : the persistent volume, usually $HOME
# Override by exporting HF_HOME before running.
pick_hf_home() {
  [[ -n "${HF_HOME:-}" ]] && { echo "${HF_HOME}"; return; }
  for candidate in /mnt/ebs /workspace /persistent; do
    [[ -d "${candidate}" ]] && { echo "${candidate}/hf_cache"; return; }
  done
  echo "${HOME}/.cache/huggingface"
}
export HF_HOME
HF_HOME="$(pick_hf_home)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

log() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[33m !  %s\033[0m\n' "$*"; }

cmd_preflight() {
  # Run this LOCALLY, before starting any paid instance. Everything checked here
  # is GPU-independent, so a failure found now costs nothing and the same failure
  # found on the instance costs money and context-switching.
  cd "${REPO_ROOT}"
  local fail=0

  log "1. Python version compatibility (instance is 3.12 on Ubuntu 24.04)"
  python3 - <<'EOF' || fail=1
import ast, pathlib, sys
bad = []
for f in pathlib.Path(".").rglob("*.py"):
    if ".venv" in f.parts:
        continue
    try:
        ast.parse(f.read_text(), feature_version=(3, 12))
    except SyntaxError as e:
        bad.append(f"{f}: {e}")
print("\n".join(f"  FAIL {b}" for b in bad) or "  ok — all files parse under 3.12")
sys.exit(1 if bad else 0)
EOF

  log "2. Clean-room install (proves requirements.txt is complete)"
  local tmp
  tmp="$(mktemp -d)"
  python3 -m venv "${tmp}/venv" >/dev/null
  "${tmp}/venv/bin/pip" install -q -r requirements.txt || fail=1
  "${tmp}/venv/bin/python" -c "import rrm; print(f'  ok — {len(rrm.__all__)} exports')" || fail=1

  log "3. Benchmark suite"
  "${tmp}/venv/bin/python" scripts/oracle_loop.py --suite 2>&1 | tail -3 || fail=1

  log "4. Relation inference"
  "${tmp}/venv/bin/python" simulation/isaac_backend.py || fail=1

  log "5. Traces are well-formed JSON"
  "${tmp}/venv/bin/python" scripts/oracle_loop.py --suite --trace-dir "${tmp}/traces" >/dev/null
  python3 - "${tmp}/traces" <<'EOF' || fail=1
import json, pathlib, sys
d = pathlib.Path(sys.argv[1])
n = 0
for f in sorted(d.glob("*.jsonl")):
    for line in f.open():
        json.loads(line)
        n += 1
print(f"  ok — {n} records across {len(list(d.glob('*.jsonl')))} episodes")
EOF

  log "6. Committed and pushable"
  if [[ ! -d .git ]]; then
    warn "not a git repo — you cannot clone this onto the instance"
    fail=1
  elif [[ -n "$(git status --porcelain)" ]]; then
    warn "uncommitted changes — they will not travel with a clone"
    git status --short | head -5
    fail=1
  else
    echo "  ok — worktree clean"
  fi

  rm -rf "${tmp}"
  if [[ ${fail} -eq 0 ]]; then
    log "PREFLIGHT PASS — safe to start the instance"
  else
    log "PREFLIGHT FAIL — fix locally, before the meter starts"
  fi
  return "${fail}"
}

cmd_check() {
  log "Hardware"
  nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader \
    || warn "no NVIDIA driver — run 'base'"
  echo "vCPU: $(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo '?')   RAM: $(free -g 2>/dev/null | awk '/^Mem:/{print $2}' || echo '?') GiB"

  log "Compute capability"
  local cc
  cc=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null || echo "?")
  echo "  ${cc}"
  if [[ "${cc}" == 8.6 ]]; then
    warn "Ampere (8.6): FP8 is NOT supported. Use AWQ/GPTQ INT4 for the reasoner."
  fi

  log "Software"
  for c in python3 git colcon ros2; do
    printf '  %-8s %s\n' "$c" "$(command -v "$c" || echo MISSING)"
  done

  log "Storage"
  echo "  HF_HOME=${HF_HOME}"
  df -h "$(dirname "${HF_HOME}")" 2>/dev/null | tail -1 \
    || warn "$(dirname "${HF_HOME}") does not exist"
  echo
  echo "  Mounted volumes — confirm HF_HOME is on one that survives stop/start:"
  df -h --output=target,size,avail 2>/dev/null | grep -vE '^(/dev|tmpfs|udev|overlay)' | head -8 \
    || df -h 2>/dev/null | head -8
  warn "if HF_HOME is on ephemeral disk you will re-download ~8 GB every session"

  log "Isaac Sim / Isaac Lab"
  for p in /isaac-sim "${HOME}/isaacsim" /opt/IsaacLab "${HOME}/IsaacLab"; do
    [[ -e "$p" ]] && echo "  found $p"
  done
  python3 -c "import isaacsim; print('  isaacsim importable')" 2>/dev/null \
    || echo "  isaacsim not importable from this python"
}

cmd_base() {
  log "System packages"
  sudo apt-get update -qq
  sudo apt-get install -y -qq build-essential git curl python3-venv python3-pip

  log "NVIDIA driver"
  if ! command -v nvidia-smi >/dev/null; then
    sudo apt-get install -y -qq nvidia-driver-580
    warn "driver installed — REBOOT, then re-run 'base'"
    exit 0
  fi
  nvidia-smi --query-gpu=driver_version --format=csv,noheader

  log "ROS 2 ${ROS_DISTRO}"
  if [[ ! -d "/opt/ros/${ROS_DISTRO}" ]]; then
    sudo apt-get install -y -qq software-properties-common
    sudo add-apt-repository -y universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt-get update -qq
    sudo apt-get install -y -qq "ros-${ROS_DISTRO}-desktop" "ros-${ROS_DISTRO}-ros2-control"
  fi
  echo "ROS 2 ${ROS_DISTRO} present"

  log "Python environment"
  cd "${REPO_ROOT}"
  [[ -d .venv ]] || python3 -m venv .venv
  ./.venv/bin/pip install -q --upgrade pip
  ./.venv/bin/pip install -q -r requirements.txt
  echo "venv ready"

  log "EBS mount for weights"
  sudo mkdir -p "${HF_HOME}"
  sudo chown -R "$USER" "${HF_HOME%/hf_cache}"
  echo "HF_HOME=${HF_HOME}"
}

cmd_models() {
  log "Downloading weights to ${HF_HOME}"
  mkdir -p "${HF_HOME}"
  cd "${REPO_ROOT}"

  # Action policy. 6.93 GB. LIBERO_PANDA is a pre-registered embodiment, so no
  # fine-tuning is required — see docs/architecture.md §8.0.
  ./.venv/bin/hf download nvidia/GR00T-N1.7-3B

  # Fallback policy if the VRAM budget breaks (~1 GB).
  ./.venv/bin/hf download lerobot/smolvla_base

  # Reasoner precision depends on the GPU:
  #   >=40 GiB VRAM (L40S)  -> bf16 is fine (~8 GB), no quant hunt needed
  #   24 GiB      (A10G)    -> INT4 (AWQ/GPTQ) required; bf16 breaks the budget
  # FP8 needs Ada (sm_89) or newer — it will NOT run on Ampere.
  local vram_mib
  vram_mib=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | head -1 || echo 0)
  if [[ "${vram_mib}" -ge 40000 ]]; then
    echo "  ${vram_mib} MiB VRAM — pulling bf16 reasoner"
    ./.venv/bin/hf download Qwen/Qwen3-4B-Instruct-2507
  else
    warn "${vram_mib} MiB VRAM — bf16 (~8 GB) will not fit alongside Isaac Sim + GR00T"
    warn "  find an INT4 (AWQ/GPTQ) build of Qwen3-4B-Instruct-2507, or use:"
    warn "  ./.venv/bin/hf download HuggingFaceTB/SmolLM3-3B"
  fi

  du -sh "${HF_HOME}" 2>/dev/null || true
}

cmd_verify() {
  cd "${REPO_ROOT}"

  log "1. RRM loop with no GPU, no models"
  ./.venv/bin/python scripts/oracle_loop.py --suite | tail -9

  log "2. Relation inference"
  ./.venv/bin/python simulation/isaac_backend.py

  log "3. GPU visible to torch"
  ./.venv/bin/python - <<'EOF'
try:
    import torch
    print(f"  torch {torch.__version__}  cuda={torch.cuda.is_available()}")
    if torch.cuda.is_available():
        p = torch.cuda.get_device_properties(0)
        print(f"  {p.name}  {p.total_memory / 1024**3:.1f} GiB  sm_{p.major}{p.minor}")
        if (p.major, p.minor) == (8, 6):
            print("  NOTE Ampere: no FP8. INT4 (AWQ/GPTQ) for the reasoner.")
except ImportError:
    print("  torch not installed (fine until Phase 2)")
EOF

  log "4. VRAM headroom"
  nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader 2>/dev/null \
    || warn "no GPU"
  cat <<'EOF'
  Budget (docs/architecture.md §8), 24 GB total:
    Isaac Sim      6-10 GB
    GR00T N1.7-3B  7- 8 GB
    Reasoner INT4  ~3   GB
    -------------------------
    total         16-21 GB

  When serving the reasoner, cap vLLM explicitly:
    --gpu-memory-utilization 0.15
  The 0.9 default claims ~21 GB and starves Isaac Sim.
EOF
}

case "${1:-check}" in
  preflight) cmd_preflight ;;
  check)  cmd_check ;;
  base)   cmd_base ;;
  models) cmd_models ;;
  verify) cmd_verify ;;
  *) echo "usage: $0 {preflight|check|base|models|verify}" >&2; exit 2 ;;
esac
