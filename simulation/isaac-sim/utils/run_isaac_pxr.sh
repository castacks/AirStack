#!/usr/bin/env bash
# run_isaac_pxr.sh
# Convenience wrapper for Isaac Sim Python with pxr/USD library paths.
set -euo pipefail

export PXR_PARENT="${PXR_PARENT:-$(dirname "$(find /isaac-sim/extscache -type d -path '*omni.usd.libs*/pxr' | head -1)")}"
export USD_LIB_DIR="${USD_LIB_DIR:-$PXR_PARENT/bin}"
export PYTHONPATH="$PXR_PARENT:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$USD_LIB_DIR:${LD_LIBRARY_PATH:-}"

exec /isaac-sim/python.sh "$@"
