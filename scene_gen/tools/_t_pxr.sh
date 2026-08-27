#!/usr/bin/env bash
# _t_pxr.sh — run ANY python file inside the isaac-sim container on a bare
# python with `pxr` + the omniverse resolver on the path (the recipe
# measure_fences.sh documents), generalised to an arbitrary script.
#
# No SimulationApp, no Kit, no GPU, so it is safe beside a running sim.
#
#     scene_gen/tools/_t_pxr.sh scene_gen/tools/_t_shell_probe.py [VAR=value ...]
#
# The path is given RELATIVE TO THE REPO ROOT; the repo is mounted at
# /isaac-sim/AirStack in the container.
set -euo pipefail
SCRIPT=${1:?script path relative to repo root}; shift
ENVS=""
for kv in "$@"; do ENVS="$ENVS $kv"; done
docker exec isaac-sim bash -lc '
set -euo pipefail
L=$(ls -d /isaac-sim/extscache/omni.usd.libs-* | head -1)
R=$(ls -d /isaac-sim/extscache/omni.usd_resolver-* | head -1)
C=/isaac-sim/kit/extscore/omni.client.lib
export PYTHONPATH="$L:$R:$C:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$L/bin:$R/lib:$C/bin:${LD_LIBRARY_PATH:-}"
export PXR_PLUGINPATH_NAME="$R/lib/usd/omni_usd_resolver/resources"
cd /isaac-sim && env '"$ENVS"' ./python.sh "/isaac-sim/AirStack/'"$SCRIPT"'"
'
