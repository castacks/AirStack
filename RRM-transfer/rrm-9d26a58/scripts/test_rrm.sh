#!/usr/bin/env bash
# Run RRM's dependency-light unit suite from its isolated local virtual environment.
# This deliberately has no Docker, OSMO, simulator, GPU, or robot-control behavior.
set -euo pipefail

rrm_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
rrm_venv="$rrm_root/.venv"
rrm_python="${PYTHON:-python3}"
rrm_venv_python="$rrm_venv/bin/python"
rrm_deps="$rrm_root/.rrm-deps"

if [[ ! -d "$rrm_venv" ]]; then
  echo "Creating isolated RRM virtual environment at $rrm_venv" >&2
  "$rrm_python" -m venv "$rrm_venv" >/dev/null 2>&1 || true
fi

if [[ -x "$rrm_venv_python" ]] && "$rrm_venv_python" -m pip --version >/dev/null 2>&1; then
  if ! "$rrm_venv_python" -c 'import pydantic; assert pydantic.VERSION.startswith("2.")' \
    >/dev/null 2>&1; then
    echo "Installing RRM test dependencies into $rrm_venv" >&2
    "$rrm_venv_python" -m pip install --disable-pip-version-check --requirement "$rrm_root/requirements.txt"
  fi
  cd "$rrm_root"
  exec "$rrm_venv_python" -m unittest discover -s tests -v
fi

echo "Python venv support is unavailable; using isolated $rrm_deps instead." >&2
if ! "$rrm_python" -m pip --version >/dev/null 2>&1; then
  echo "Neither Python venv nor pip is available. Install python3-venv or pip, then retry." >&2
  exit 1
fi
if ! PYTHONPATH="$rrm_deps" "$rrm_python" -c 'import pydantic; assert pydantic.VERSION.startswith("2.")' \
  >/dev/null 2>&1; then
  echo "Installing RRM test dependencies into $rrm_deps" >&2
  "$rrm_python" -m pip install --disable-pip-version-check --target "$rrm_deps" \
    --requirement "$rrm_root/requirements.txt"
fi
cd "$rrm_root"
exec env PYTHONPATH="$rrm_deps${PYTHONPATH:+:$PYTHONPATH}" \
  "$rrm_python" -m unittest discover -s tests -v
